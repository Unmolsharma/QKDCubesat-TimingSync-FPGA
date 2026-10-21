// =============================================================================
// tdc_arty_top.v  --  board wrapper for the Digilent Arty A7-100T
// -----------------------------------------------------------------------------
//   CLK100MHZ -> MMCM (x10 / 4) -> 250 MHz system clock
//   ja[0] (Pmod JA pin 1)  : external hit input (LVCMOS33)
//   sw[0] : hit source      0 = external (Pmod JA1), 1 = on-chip ring oscillator
//   sw[1] : UART mode       0 = fine codes only (code density), 1 = full timestamps
//   ck_rst: CPU_RESET button (active low)
//   led[0]: heartbeat (~1 Hz); proves the MMCM is locked and the clock is running
//   led[1]: sticky: a hit was DROPPED (UART/FIFO could not keep up)
//   led[2]: sticky: SHORT pulse seen (hit narrower than the chain; see README)
//   led[3]: sticky: SATURATION: the chain did not cover the clock period.
//           If this ever lights, the fine data is incomplete: see README,
//           "Chain coverage".
//   uart_rxd_out: FPGA -> PC serial data (2 Mbaud 8N1, framing in tdc_uart_readout)
//
// Changing the clock: MMCM_MULT / MMCM_OUT_DIV set f_sys = 100 MHz * MULT / DIV
// (VCO = 100 MHz * MULT must stay within 600-1200 MHz on a -1 part). Also update
// the create_generated_clock expectations in the XDC, --period-ps in the
// scripts, CLKS_PER_BIT for the UART, and IDEAL_LSB_PS if the LUT default matters.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_arty_top #(
    parameter integer NUM_CARRY4    = 48,
    parameter real    MMCM_MULT     = 10.0,
    parameter real    MMCM_OUT_DIV  = 4.0,
    parameter         LUT_FILE      = "",
    parameter integer CLKS_PER_BIT  = 125,       // 250 MHz / 2 Mbaud
    parameter integer FIFO_AW       = 9,
    parameter integer STATUS_PERIOD = 67108864,  // 0.27 s
    parameter integer RO_DIV_BITS   = 12,
    // simulation-only model knobs (passed down to the delay line)
    parameter real    TAP_NOM_PS    = 22.0,
    parameter real    JITTER_PS     = 4.0,
    parameter integer SEED          = 1
)(
    input  wire       CLK100MHZ,
    input  wire       ck_rst,
    input  wire [3:0] sw,
    input  wire [0:0] ja,
    output wire [3:0] led,
    output wire       uart_rxd_out
);
    localparam integer FINE_W = 9, COARSE_W = 48, PS_W = 16;

    // ---------------- clocking ----------------
    wire clk, locked;

`ifdef SYNTHESIS
    wire clk_fb, clk_mmcm;
    MMCME2_BASE #(
        .CLKIN1_PERIOD    (10.0),
        .CLKFBOUT_MULT_F  (MMCM_MULT),
        .DIVCLK_DIVIDE    (1),
        .CLKOUT0_DIVIDE_F (MMCM_OUT_DIV)
    ) u_mmcm (
        .CLKIN1   (CLK100MHZ),
        .CLKFBIN  (clk_fb),
        .CLKFBOUT (clk_fb),
        .CLKOUT0  (clk_mmcm),
        .LOCKED   (locked),
        .PWRDWN   (1'b0),
        .RST      (1'b0),
        .CLKFBOUTB(), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(),
        .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6()
    );
    BUFG u_bufg (.I(clk_mmcm), .O(clk));
`else
    // Simulation stand-in for the MMCM: an independent 250 MHz clock.
    reg clk_sim = 1'b0;
    reg locked_sim = 1'b0;
    always #(2.0) clk_sim = ~clk_sim;
    initial #(100.0) locked_sim = 1'b1;
    assign clk = clk_sim;
    assign locked = locked_sim;
`endif

    // ---------------- reset + switch synchronisers ----------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] rst_sync = 3'b111;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sw0_s = 2'b00, sw1_s = 2'b00;
    always @(posedge clk or negedge locked)
        if (!locked) rst_sync <= 3'b111;
        else         rst_sync <= {rst_sync[1:0], ~ck_rst};
    wire rst = rst_sync[2];

    always @(posedge clk) begin
        sw0_s <= {sw0_s[0], sw[0]};
        sw1_s <= {sw1_s[0], sw[1]};
    end

    // ---------------- hit source ----------------
    wire ro_hit;
    tdc_ring_osc #(.DIV_BITS(RO_DIV_BITS)) u_ro (
        .en      (sw0_s[1]),
        .hit_out (ro_hit)
    );

    // Asynchronous mux in front of the chain. Its delay is a constant offset
    // (identical for every hit), so it doesn't affect resolution.
    wire hit = sw0_s[1] ? ro_hit : ja[0];

    // ---------------- TDC channel ----------------
    wire                       m_valid, m_ready;
    wire [COARSE_W+FINE_W-1:0] m_raw;
    wire [COARSE_W+PS_W-1:0]   m_cal;
    wire                       overflow, dead;
    wire [15:0]                drop_cnt, short_cnt, sat_cnt, rej_cnt;
    wire [COARSE_W-1:0]        coarse_now;

    tdc_top #(
        .NUM_CARRY4 (NUM_CARRY4),
        .FINE_W     (FINE_W),
        .COARSE_W   (COARSE_W),
        .PS_W       (PS_W),
        .LUT_FILE   (LUT_FILE),
        .TAP_NOM_PS (TAP_NOM_PS),
        .JITTER_PS  (JITTER_PS),
        .SEED       (SEED)
    ) u_tdc (
        .clk        (clk),
        .rst        (rst),
        .hit        (hit),
        .m_valid    (m_valid),
        .m_ready    (m_ready),
        .m_raw      (m_raw),
        .m_cal      (m_cal),
        .overflow   (overflow),
        .dead       (dead),
        .drop_cnt   (drop_cnt),
        .short_cnt  (short_cnt),
        .sat_cnt    (sat_cnt),
        .rej_cnt    (rej_cnt),
        .coarse_now (coarse_now),
        .cal_we     (1'b0),
        .cal_addr   ({FINE_W{1'b0}}),
        .cal_wdata  ({PS_W{1'b0}})
    );

    // ---------------- UART readout ----------------
    tdc_uart_readout #(
        .COARSE_W      (COARSE_W),
        .FINE_W        (FINE_W),
        .FIFO_AW       (FIFO_AW),
        .CLKS_PER_BIT  (CLKS_PER_BIT),
        .STATUS_PERIOD (STATUS_PERIOD)
    ) u_uart (
        .clk       (clk),
        .rst       (rst),
        .s_valid   (m_valid),
        .s_ready   (m_ready),
        .s_coarse  (m_raw[FINE_W +: COARSE_W]),
        .s_fine    (m_raw[FINE_W-1:0]),
        .mode_ts   (sw1_s[1]),
        .drop_cnt  (drop_cnt),
        .short_cnt (short_cnt),
        .sat_cnt   (sat_cnt),
        .tlm_en    (1'b0),                 // no loop telemetry in the block 1 wrapper
        .tlm_data  (173'd0),
        .rb_valid  (1'b0),                 // no command interface in the block 1 wrapper
        .rb_data   (173'd0),
        .tx        (uart_rxd_out)
    );

    // ---------------- LEDs ----------------
    reg drop_seen = 1'b0, short_seen = 1'b0, sat_seen = 1'b0;
    always @(posedge clk)
        if (rst) begin
            drop_seen <= 1'b0; short_seen <= 1'b0; sat_seen <= 1'b0;
        end else begin
            drop_seen  <= drop_seen  | (drop_cnt  != 16'd0);
            short_seen <= short_seen | (short_cnt != 16'd0);
            sat_seen   <= sat_seen   | (sat_cnt   != 16'd0);
        end

    assign led = {sat_seen, short_seen, drop_seen, coarse_now[27]};
endmodule

`default_nettype wire
