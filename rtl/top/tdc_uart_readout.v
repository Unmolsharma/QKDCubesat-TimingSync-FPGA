// =============================================================================
// tdc_uart_readout.v  --  stream TDC data and loop telemetry over the USB-UART
// -----------------------------------------------------------------------------
// LINK: 8N1 at 2 000 000 baud (exact from 250 MHz, see tdc_uart_tx) =
//       200 kB/s. That is ~100 k fine codes/s or ~22 k full timestamps/s,
//       plenty for a code-density run of 10^5-10^6 hits.
//
// FRAMING (self-synchronising; the PC can join the stream at any byte)
//   byte 0 : 1 T T P P P P P    header: bit7 = 1, TT = frame type, 5 payload bits
//   byte n : 0 P P P P P P P    continuation: bit7 = 0, 7 payload bits
//   The payload is sent MSB first and is right-aligned in its field
//   (field width = 5 + 7 * (bytes - 1)):
//
//   type 0  FINE       2 bytes  12-bit field  = fine code (FINE_W <= 12)
//   type 1  TIMESTAMP  9 bytes  61-bit field  = {coarse, fine}  (COARSE_W + FINE_W <= 61)
//   type 2  STATUS     8 bytes  54-bit field  = {6'b0, drop_cnt, short_cnt, sat_cnt}
//   type 3  TELEMETRY 25 bytes 173-bit field  = tlm_data (block 2 loop telemetry;
//                                               layout in scripts/capture.py)
//
//   Only header bytes have bit 7 set, so a receiver that loses a byte just
//   waits for the next header. scripts/capture.py implements the parser.
//   (Block 1 sent a 7-byte TIMESTAMP with a 32-bit coarse count; the coarse
//   counter is 48 bits since block 2, so the frame grew to 9 bytes.)
//
// BURSTS: hits go into a FIFO (2^FIFO_AW entries, BRAM). When it is full, s_ready
//   drops, and tdc_top then DROPS new hits and counts them (drop_cnt). Nothing
//   already queued is overwritten, and every loss is reported in the next
//   STATUS frame. STATUS frames go out every STATUS_PERIOD clocks (default
//   2^26 = 0.27 s) and once after reset; when tlm_en is high a TELEMETRY
//   frame follows each STATUS frame (tlm_data is sampled when it is queued).
//
// Dropping is decided by FIFO occupancy, which doesn't depend on a hit's fine
// code, so drops don't bias a code-density histogram.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_uart_readout #(
    parameter integer COARSE_W      = 48,
    parameter integer FINE_W        = 9,
    parameter integer FIFO_AW       = 9,
    parameter integer CLKS_PER_BIT  = 125,
    parameter integer STATUS_PERIOD = 67108864
)(
    input  wire                clk,
    input  wire                rst,
    // timestamp stream from tdc_top
    input  wire                s_valid,
    output wire                s_ready,
    input  wire [COARSE_W-1:0] s_coarse,
    input  wire [FINE_W-1:0]   s_fine,
    // 0: send fine codes only, 1: send full timestamps
    input  wire                mode_ts,
    // counters reported in STATUS frames
    input  wire [15:0]         drop_cnt,
    input  wire [15:0]         short_cnt,
    input  wire [15:0]         sat_cnt,
    // loop telemetry (TELEMETRY frames); tie tlm_en low if unused
    input  wire                tlm_en,
    input  wire [172:0]        tlm_data,
    output wire                tx
);
    localparam integer DW  = COARSE_W + FINE_W;
    localparam integer SRW = 173;                 // widest payload field (TELEMETRY)
    localparam [1:0] T_FINE = 2'd0, T_TS = 2'd1, T_STATUS = 2'd2, T_TLM = 2'd3;

    // ---------------- FIFO ----------------
    reg [DW-1:0]    fifo [0:(1<<FIFO_AW)-1];
    reg [FIFO_AW:0] wr_ptr = 0, rd_ptr = 0;
    reg [DW-1:0]    rd_data = {DW{1'b0}};
    reg             rd_en;

    wire empty = (wr_ptr == rd_ptr);
    wire full  = (wr_ptr[FIFO_AW] != rd_ptr[FIFO_AW]) &&
                 (wr_ptr[FIFO_AW-1:0] == rd_ptr[FIFO_AW-1:0]);
    assign s_ready = ~full & ~rst;

    always @(posedge clk) begin
        if (s_valid && s_ready)
            fifo[wr_ptr[FIFO_AW-1:0]] <= {s_coarse, s_fine};
        if (rd_en)
            rd_data <= fifo[rd_ptr[FIFO_AW-1:0]];
        if (rst) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (s_valid && s_ready) wr_ptr <= wr_ptr + 1'b1;
            if (rd_en)              rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // ---------------- status timer ----------------
    reg [31:0] st_timer = 32'd0;
    reg        status_due = 1'b1;
    reg        tlm_due = 1'b0;

    // ---------------- framer ----------------
    localparam [1:0] F_IDLE = 2'd0, F_READ = 2'd1, F_LOAD = 2'd2, F_SEND = 2'd3;
    reg [1:0]     fstate = F_IDLE;
    reg [SRW-1:0] sr = {SRW{1'b0}};   // payload field, left-aligned
    reg [1:0]     ftype = 2'd0;
    reg [4:0]     nleft = 5'd0;       // bytes still to send
    reg           first = 1'b0;
    reg           mode_lat = 1'b0;

    reg        tx_start = 1'b0;
    reg [7:0]  tx_data = 8'd0;
    wire       tx_busy;

    // Zero-extend into the fixed payload fields, then left-align in sr.
    wire [11:0] fine12 = rd_data[FINE_W-1:0];
    wire [60:0] ts61   = rd_data;
    wire [53:0] st54   = {6'd0, drop_cnt, short_cnt, sat_cnt};

    always @(*) rd_en = (fstate == F_IDLE) && !status_due && !tlm_due && !empty && !rst;

    always @(posedge clk) begin
        tx_start <= 1'b0;
        if (rst) begin
            fstate     <= F_IDLE;
            st_timer   <= 32'd0;
            status_due <= 1'b1;
            tlm_due    <= 1'b0;
        end else begin
            if (st_timer == STATUS_PERIOD - 1) begin
                st_timer   <= 32'd0;
                status_due <= 1'b1;
            end else begin
                st_timer <= st_timer + 1'b1;
            end

            case (fstate)
            F_IDLE:
                if (status_due) begin
                    status_due <= 1'b0;
                    tlm_due    <= tlm_en;
                    sr     <= {st54, {(SRW-54){1'b0}}};
                    ftype  <= T_STATUS;
                    nleft  <= 5'd8;
                    first  <= 1'b1;
                    fstate <= F_SEND;
                end else if (tlm_due) begin
                    tlm_due <= 1'b0;
                    sr      <= tlm_data;
                    ftype   <= T_TLM;
                    nleft   <= 5'd25;
                    first   <= 1'b1;
                    fstate  <= F_SEND;
                end else if (!empty) begin
                    mode_lat <= mode_ts;
                    fstate   <= F_READ;          // rd_en is high this cycle
                end
            F_READ:
                fstate <= F_LOAD;                // rd_data valid next cycle
            F_LOAD: begin
                if (mode_lat) begin
                    sr    <= {ts61, {(SRW-61){1'b0}}};
                    ftype <= T_TS;
                    nleft <= 5'd9;
                end else begin
                    sr    <= {fine12, {(SRW-12){1'b0}}};
                    ftype <= T_FINE;
                    nleft <= 5'd2;
                end
                first  <= 1'b1;
                fstate <= F_SEND;
            end
            F_SEND:
                if (nleft == 5'd0) begin
                    fstate <= F_IDLE;
                end else if (!tx_busy && !tx_start) begin
                    tx_start <= 1'b1;
                    if (first) begin
                        tx_data <= {1'b1, ftype, sr[SRW-1 -: 5]};
                        sr      <= sr << 5;
                    end else begin
                        tx_data <= {1'b0, sr[SRW-1 -: 7]};
                        sr      <= sr << 7;
                    end
                    first <= 1'b0;
                    nleft <= nleft - 1'b1;
                end
            endcase
        end
    end

    tdc_uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk   (clk),
        .rst   (rst),
        .start (tx_start),
        .data  (tx_data),
        .busy  (tx_busy),
        .tx    (tx)
    );
endmodule

`default_nettype wire
