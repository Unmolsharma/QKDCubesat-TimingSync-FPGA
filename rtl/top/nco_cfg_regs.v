// =============================================================================
// nco_cfg_regs.v  --  NCO runtime settings, written and read over the UART
// -----------------------------------------------------------------------------
// COMMAND FRAME (PC -> board, 8 bytes, 2 Mbaud 8N1 on the USB-UART RX pin)
//
//   0xA5 | cmd | addr | d[31:24] | d[23:16] | d[15:8] | d[7:0] | chk
//   cmd  = 0x57 'W' write, 0x52 'R' read (data ignored, send zeros)
//   chk  = cmd ^ addr ^ d3 ^ d2 ^ d1 ^ d0
//   The parser hunts for 0xA5, then takes 7 bytes. If more than ~100 us pass
//   between bytes of a frame it drops the partial frame and hunts again, so a
//   lost byte only costs that one command.
//
// RESPONSE: every command (good or bad) produces one READBACK frame on the
// transmit side: the 25-byte type-3 frame with its 2 spare bits = 01
// (telemetry uses 00), payload fields MSB first:
//   [172:165] addr  [164:133] value (register after the command)
//   [132:125] status: 0 ok, 1 bad checksum, 2 bad address / read-only, 3 bad cmd
//   [124:117] seq   (counts commands, wraps)            [1:0] = 2'b01
// scripts/nco_config.py builds the commands and decodes the replies.
//
// REGISTER MAP (32-bit; unused bits read as 0). Reset values = the defaults
// the RTL tests and the float model use.
//   0x00 GATE    [15:0] gate_thr (2^-16 slot, 0x2000 = 1/8)  [16] gate_en (1)
//   0x01 LOCK    lock_thr   (2^-32 slot^2, 0x0400_0000 = 1/64)
//   0x02 UNLOCK  unlock_thr (2^-32 slot^2, 0x1000_0000 = 1/16)
//   0x03 AVG     [3:0] mse_shift (5)  [11:8] mse_shift_trk (7)
//   0x04 TAU     [4:0] tau_ovr (0)    [8] tau_ovr_en (0)
//   0x05 GEARS   [3:0] kp_acq (2)  [11:8] kp_trk (6)  [19:16] gear_log2 (6)
//   0x06 REJ     [7:0] rej_max (16)
//   0x07 TIMEOUT [4:0] to_shift (5)
//   0x08 SLIP    [15:0] slip_thr (2^-16 slot, 0x2000 = 1/8)  [16] slip_unlock (1)
//   0x0F ID      read-only 0x4E43_4F01 ("NCO", version 1)
//
// WHEN A NEW VALUE TAKES EFFECT: immediately. nco_top reads its configuration
// inputs on every hit, so a write applies from the next hit on. Gate and
// threshold changes are safe while locked. kp_acq applies at once while the
// loop is acquiring; a kp_trk lowered below the current gear clamps it at
// once, a raised one is reached by the next ramp step; gear_log2 sets the
// next ramp step length. To
// change several settings consistently, write them while the loop is
// unlocked (e.g. during a signal gap) or reset afterwards.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_cfg_regs #(
    parameter integer IDLE_TIMEOUT = 25000      // clocks (100 us at 250 MHz)
)(
    input  wire        clk,
    input  wire        rst,
    // received bytes (uart_rx)
    input  wire        rx_valid,
    input  wire [7:0]  rx_data,
    // readback frame for tdc_uart_readout
    output reg         rb_valid = 1'b0,
    output reg [172:0] rb_data = 173'd0,
    // NCO configuration
    output reg  [3:0]  kp_acq = 4'd2,
    output reg  [3:0]  kp_trk = 4'd6,
    output reg  [3:0]  gear_log2 = 4'd6,
    output reg         gate_en = 1'b1,
    output reg  [15:0] gate_thr = 16'h2000,
    output reg  [3:0]  mse_shift = 4'd5,
    output reg  [3:0]  mse_shift_trk = 4'd7,
    output reg  [31:0] lock_thr = 32'h0400_0000,
    output reg  [31:0] unlock_thr = 32'h1000_0000,
    output reg  [7:0]  rej_max = 8'd16,
    output reg  [4:0]  to_shift = 5'd5,
    output reg         tau_ovr_en = 1'b0,
    output reg  [4:0]  tau_ovr = 5'd0,
    output reg  [15:0] slip_thr = 16'h2000,
    output reg         slip_unlock = 1'b1,
    output reg  [7:0]  cmd_cnt = 8'd0,
    output reg  [7:0]  err_cnt = 8'd0
);
    localparam [7:0] SYNC = 8'hA5, CMD_W = 8'h57, CMD_R = 8'h52;
    localparam [31:0] ID = 32'h4E43_4F01;

    reg [2:0]  nbytes = 3'd0;            // bytes collected after SYNC
    reg        hunting = 1'b1;
    reg [7:0]  b [0:6];
    reg [15:0] idle = 16'd0;
    reg        exec = 1'b0;

    // ---- current register value at an address ----
    function [31:0] rd;
        input [7:0] a;
        begin
            case (a)
            8'h00: rd = {15'd0, gate_en, gate_thr};
            8'h01: rd = lock_thr;
            8'h02: rd = unlock_thr;
            8'h03: rd = {20'd0, mse_shift_trk, 4'd0, mse_shift};
            8'h04: rd = {23'd0, tau_ovr_en, 3'd0, tau_ovr};
            8'h05: rd = {12'd0, gear_log2, 4'd0, kp_trk, 4'd0, kp_acq};
            8'h06: rd = {24'd0, rej_max};
            8'h07: rd = {27'd0, to_shift};
            8'h08: rd = {15'd0, slip_unlock, slip_thr};
            8'h0F: rd = ID;
            default: rd = 32'd0;
            endcase
        end
    endfunction

    wire [7:0]  cmd  = b[0];
    wire [7:0]  addr = b[1];
    wire [31:0] wdat = {b[2], b[3], b[4], b[5]};
    wire [7:0]  chk  = b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5];
    wire        addr_ok = (addr <= 8'h08) || (addr == 8'h0F);
    wire        addr_rw = (addr <= 8'h08);
    wire [7:0]  status = (chk != b[6])                       ? 8'd1 :
                         (cmd != CMD_W && cmd != CMD_R)       ? 8'd3 :
                         (!addr_ok || (cmd == CMD_W && !addr_rw)) ? 8'd2 : 8'd0;

    // stage 2 (exec): check, registered. stage 3 (dec): decode the write enables.
    // stage 4 (wr): write. stage 5 (resp): the readback uses the register value after
    // the write. (A command takes ~35 us to arrive at 2 Mbaud; a few cycles are free.)
    reg        resp = 1'b0, wr = 1'b0, dec = 1'b0;
    reg [7:0]  r_cmd = 8'd0;
    reg [7:0]  r_addr = 8'd0, r_status = 8'd0;
    reg [8:0]  we = 9'd0;
    reg [31:0] w_dat = 32'd0;

    always @(posedge clk) begin
        rb_valid <= 1'b0;
        exec <= 1'b0;
        wr   <= 1'b0;
        dec  <= 1'b0;
        resp <= 1'b0;
        if (rst) begin
            hunting <= 1'b1; nbytes <= 3'd0; idle <= 16'd0;
            kp_acq <= 4'd2; kp_trk <= 4'd6; gear_log2 <= 4'd6; gate_en <= 1'b1;
            gate_thr <= 16'h2000; mse_shift <= 4'd5; mse_shift_trk <= 4'd7;
            lock_thr <= 32'h0400_0000; unlock_thr <= 32'h1000_0000; rej_max <= 8'd16;
            to_shift <= 5'd5; tau_ovr_en <= 1'b0; tau_ovr <= 5'd0;
            slip_thr <= 16'h2000; slip_unlock <= 1'b1; cmd_cnt <= 8'd0; err_cnt <= 8'd0;
        end else begin
            // ---- byte collection ----
            if (rx_valid) begin
                idle <= 16'd0;
                if (hunting) begin
                    if (rx_data == SYNC) begin hunting <= 1'b0; nbytes <= 3'd0; end
                end else begin
                    b[nbytes] <= rx_data;
                    if (nbytes == 3'd6) begin hunting <= 1'b1; exec <= 1'b1; end
                    nbytes <= nbytes + 1'b1;
                end
            end else if (!hunting) begin
                if (idle == IDLE_TIMEOUT - 1) begin hunting <= 1'b1; idle <= 16'd0; end
                else idle <= idle + 1'b1;
            end
            // ---- execute (cycle after the last byte): check, then decode ----
            if (exec) begin
                dec      <= 1'b1;
                r_cmd    <= cmd;
                r_addr   <= addr;
                r_status <= status;
                w_dat    <= wdat;
            end
            if (dec) begin
                wr <= 1'b1;
                we <= (r_status == 8'd0 && r_cmd == CMD_W && r_addr <= 8'h08) ? (9'd1 << r_addr[3:0]) : 9'd0;
            end
            // ---- write ----
            if (wr) begin
                cmd_cnt <= cmd_cnt + 1'b1;
                if (r_status != 8'd0 && err_cnt != 8'hFF) err_cnt <= err_cnt + 1'b1;
                if (we[0]) begin gate_thr <= w_dat[15:0]; gate_en <= w_dat[16]; end
                if (we[1]) lock_thr <= w_dat;
                if (we[2]) unlock_thr <= w_dat;
                if (we[3]) begin mse_shift <= w_dat[3:0]; mse_shift_trk <= w_dat[11:8]; end
                if (we[4]) begin tau_ovr <= w_dat[4:0]; tau_ovr_en <= w_dat[8]; end
                if (we[5]) begin kp_acq <= w_dat[3:0]; kp_trk <= w_dat[11:8]; gear_log2 <= w_dat[19:16]; end
                if (we[6]) rej_max <= w_dat[7:0];
                if (we[7]) to_shift <= w_dat[4:0];
                if (we[8]) begin slip_thr <= w_dat[15:0]; slip_unlock <= w_dat[16]; end
                resp <= 1'b1;
            end
            // ---- readback (registers already updated) ----
            if (resp) begin
                rb_valid <= 1'b1;
                rb_data  <= {r_addr, rd(r_addr), r_status, cmd_cnt, 115'd0, 2'b01};
            end
        end
    end
endmodule

`default_nettype wire
