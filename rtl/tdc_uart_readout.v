// =============================================================================
// tdc_uart_readout.v  --  stream TDC data to the PC over the Arty USB-UART
// -----------------------------------------------------------------------------
// LINK: 8N1 at 2 000 000 baud (exact from 250 MHz, see tdc_uart_tx) =
//       200 kB/s. That is ~100 k fine codes/s or ~28 k full timestamps/s,
//       plenty for a code-density run of 10^5-10^6 hits.
//
// FRAMING (self-synchronising; the PC can join the stream at any byte)
//   byte 0 : 1 T T P P P P P    header: bit7 = 1, TT = frame type, 5 payload bits
//   byte n : 0 P P P P P P P    continuation: bit7 = 0, 7 payload bits
//   The payload is sent MSB first and is right-aligned in its field:
//
//   type 0  FINE      2 bytes  12-bit field = fine code (FINE_W <= 12)
//   type 1  TIMESTAMP 7 bytes  47-bit field = {coarse, fine}  (COARSE_W+FINE_W <= 47)
//   type 2  STATUS    8 bytes  54-bit field = {6'b0, drop_cnt, short_cnt, sat_cnt}
//   type 3  reserved
//
//   Only header bytes have bit 7 set, so a receiver that loses a byte just
//   waits for the next header. scripts/capture.py implements the parser.
//
// BURSTS: hits go into a FIFO (2^FIFO_AW entries, BRAM). When it is full, s_ready
//   drops, and tdc_top then DROPS new hits and counts them (drop_cnt). Nothing
//   already queued is overwritten, and every loss is reported in the next
//   STATUS frame. STATUS frames go out every STATUS_PERIOD clocks (default
//   2^26 = 0.27 s) and once after reset.
//
// Dropping is decided by FIFO occupancy, which doesn't depend on a hit's fine
// code, so drops don't bias a code-density histogram.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_uart_readout #(
    parameter integer COARSE_W      = 32,
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
    output wire                tx
);
    localparam integer DW = COARSE_W + FINE_W;
    localparam [1:0] T_FINE = 2'd0, T_TS = 2'd1, T_STATUS = 2'd2;

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

    // ---------------- framer ----------------
    localparam [1:0] F_IDLE = 2'd0, F_READ = 2'd1, F_LOAD = 2'd2, F_SEND = 2'd3;
    reg [1:0]  fstate = F_IDLE;
    reg [53:0] sr = 54'd0;          // payload, left-aligned
    reg [1:0]  ftype = 2'd0;
    reg [3:0]  nleft = 4'd0;        // bytes still to send
    reg        first = 1'b0;
    reg        mode_lat = 1'b0;

    reg        tx_start = 1'b0;
    reg [7:0]  tx_data = 8'd0;
    wire       tx_busy;

    // Zero-extend the FIFO word into the fixed payload fields.
    wire [11:0] fine12 = rd_data[FINE_W-1:0];
    wire [46:0] ts47   = rd_data;

    always @(*) rd_en = (fstate == F_IDLE) && !status_due && !empty && !rst;

    always @(posedge clk) begin
        tx_start <= 1'b0;
        if (rst) begin
            fstate     <= F_IDLE;
            st_timer   <= 32'd0;
            status_due <= 1'b1;
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
                    sr    <= {6'd0, drop_cnt, short_cnt, sat_cnt};
                    ftype <= T_STATUS;
                    nleft <= 4'd8;
                    first <= 1'b1;
                    fstate <= F_SEND;
                end else if (!empty) begin
                    mode_lat <= mode_ts;
                    fstate   <= F_READ;          // rd_en is high this cycle
                end
            F_READ:
                fstate <= F_LOAD;                // rd_data valid next cycle
            F_LOAD: begin
                if (mode_lat) begin
                    sr    <= {ts47, 7'd0};
                    ftype <= T_TS;
                    nleft <= 4'd7;
                end else begin
                    sr    <= {fine12, 42'd0};
                    ftype <= T_FINE;
                    nleft <= 4'd2;
                end
                first  <= 1'b1;
                fstate <= F_SEND;
            end
            F_SEND:
                if (nleft == 4'd0) begin
                    fstate <= F_IDLE;
                end else if (!tx_busy && !tx_start) begin
                    tx_start <= 1'b1;
                    if (first) begin
                        tx_data <= {1'b1, ftype, sr[53:49]};
                        sr      <= sr << 5;
                    end else begin
                        tx_data <= {1'b0, sr[53:47]};
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
