// =============================================================================
// tdc_multi_top.v  --  NUM_CH carry-chain TDC channels on ONE time base,
//                      merged into a single time-ordered timestamp stream
// -----------------------------------------------------------------------------
// A BB84 receiver has 4 single-photon detectors (one per polarisation state)
// plus at least one timing/sync channel. All of them must be timestamped on the
// same clock and counter, otherwise "which detector fired in slot k" is
// meaningless. Default NUM_CH = 5 (4 quantum + 1 sync).
//
//   hit[0] -> frontend(48 x CARRY4) -> hit_detect -> calib LUT 0 -> FIFO 0 -+
//   hit[1] -> frontend              -> hit_detect -> calib LUT 1 -> FIFO 1 -+-> merger -> m_*
//   ...                                                                     |   {ch, coarse,
//   hit[N-1] ...                                                  FIFO N-1 -+    fine, fine_ps}
//                      ONE tdc_coarse_counter (coarse_aligned shared by all)
//
// Each channel is exactly block 1's channel (own chain, sync stage, popcount
// encoder, hit detector, calibration LUT); only the coarse counter is shared.
// NUM_CH = 1 gives a single channel with the merger as a pass-through FIFO
// (tdc_top is kept unchanged for the block 1 tests).
//
// PER-CHANNEL CALIBRATION. Each chain has its own tap delays, so each needs its
// own code-density LUT: LUT_DIR/ch<i>.mem (ideal linear LUT if LUT_DIR = "").
// The runtime write port takes a channel select (cal_ch). Channel-to-channel
// SKEW (pad, routing to tap 0, LUT origin) is a constant per channel that the
// TDC cannot see; it must be measured in the lab (one split pulse into all
// channels) and subtracted downstream as per-channel offsets.
//
// MERGER AND ITS ORDERING GUARANTEE
//   Every channel has the same pipeline, so a hit captured at edge c reaches
//   its FIFO no later than coarse_now = c + MAXLAT (sync + encoder 4, detect +
//   confirm <= 4, LUT 1, register 1 -> 10; MAXLAT = 12 with margin). A FIFO
//   head is therefore released only when coarse_now >= c + MAXLAT + 2: by
//   then every hit with an earlier coarse count, on any channel, is already
//   queued. Among the released-eligible heads the merger emits the one with
//   the smallest calibrated time  t = c * T_CLK_PS - fine_ps  (lowest channel
//   number on a tie). Result: the output is sorted by MEASURED time.
//   Limits: (1) two hits closer than the TDC resolution (~12 ps RMS) plus the
//   uncalibrated channel skew may come out in either order; (2) the merger
//   emits at most one hit per clock: sustained aggregate rates above
//   250 M/s (or bursts longer than the per-channel FIFOs) are dropped and
//   counted per channel (ch_drop). QKD rates are orders of magnitude lower.
//   Latency: ~MAXLAT + 3 clocks more than a single channel.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_multi_top #(
    parameter integer NUM_CH        = 5,
    parameter integer CH_W          = 3,        // >= clog2(NUM_CH)
    parameter integer NUM_CARRY4    = 48,
    parameter integer FINE_W        = 9,
    parameter integer COARSE_W      = 48,
    parameter integer PS_W          = 16,
    parameter integer IDEAL_LSB_PS  = 22,
    parameter integer T_CLK_PS      = 4000,
    parameter         LUT_DIR       = "",       // per-channel LUT_DIR/ch<i>.mem
    parameter integer CONFIRM_CYC   = 3,
    parameter integer QAW           = 3,        // per-channel FIFO depth 2^QAW
    parameter integer MAXLAT        = 12,
    // behavioural delay-line model (simulation only); channel i uses SEED + 101*i
    parameter real    TAP_NOM_PS    = 22.0,
    parameter real    TAP_SIGMA_PCT = 15.0,
    parameter real    SKEW_SIGMA_PS = 6.0,
    parameter real    JITTER_PS     = 4.0,
    parameter integer SEED          = 1
)(
    input  wire                       clk,
    input  wire                       rst,
    input  wire [NUM_CH-1:0]          hit,
    // merged, time-ordered stream
    output reg                        m_valid = 1'b0,
    input  wire                       m_ready,
    output reg  [CH_W-1:0]            m_ch = {CH_W{1'b0}},
    output reg  [COARSE_W-1:0]        m_coarse = {COARSE_W{1'b0}},
    output reg  [FINE_W-1:0]          m_fine = {FINE_W{1'b0}},
    output reg  [PS_W-1:0]            m_fine_ps = {PS_W{1'b0}},
    // per-channel status, channel i in bits [16*i +: 16]
    output wire [16*NUM_CH-1:0]       ch_drop,
    output wire [16*NUM_CH-1:0]       ch_short,
    output wire [16*NUM_CH-1:0]       ch_sat,
    output wire [16*NUM_CH-1:0]       ch_rej,
    output wire [COARSE_W-1:0]        coarse_now,
    // calibration write port (channel select)
    input  wire                       cal_we,
    input  wire [CH_W-1:0]            cal_ch,
    input  wire [FINE_W-1:0]          cal_addr,
    input  wire [PS_W-1:0]            cal_wdata
);
    localparam integer LATENCY = 4;
    localparam integer QW = COARSE_W + FINE_W + PS_W;

    wire [COARSE_W-1:0] coarse_aligned;
    tdc_coarse_counter #(.COARSE_W(COARSE_W), .DELAY(LATENCY)) u_cc (
        .clk(clk), .rst(rst), .coarse(coarse_now), .coarse_aligned(coarse_aligned));

    // per-channel FIFO heads
    wire [NUM_CH-1:0]   q_nonempty;
    wire [QW-1:0]       q_head [0:NUM_CH-1];
    reg  [NUM_CH-1:0]   q_pop;

    genvar g;
    generate for (g = 0; g < NUM_CH; g = g + 1) begin : ch
        wire [FINE_W-1:0] fe_fine;
        wire              fe_t0, fe_zero, fe_full;
        tdc_frontend #(
            .NUM_CARRY4(NUM_CARRY4), .FINE_W(FINE_W), .TAP_NOM_PS(TAP_NOM_PS),
            .TAP_SIGMA_PCT(TAP_SIGMA_PCT), .SKEW_SIGMA_PS(SKEW_SIGMA_PS),
            .JITTER_PS(JITTER_PS), .SEED(SEED + 101 * g)
        ) u_fe (.clk(clk), .hit(hit[g]), .fine(fe_fine), .t0(fe_t0), .zero(fe_zero), .full(fe_full));

        wire                ev_valid;
        wire [COARSE_W-1:0] ev_coarse;
        wire [FINE_W-1:0]   ev_fine;
        tdc_hit_detect #(.FINE_W(FINE_W), .COARSE_W(COARSE_W), .CONFIRM_CYC(CONFIRM_CYC)) u_hd (
            .clk(clk), .rst(rst), .fine(fe_fine), .t0(fe_t0), .zero(fe_zero), .full(fe_full),
            .coarse(coarse_aligned), .ev_valid(ev_valid), .ev_coarse(ev_coarse), .ev_fine(ev_fine),
            .dead(), .short_evt(), .sat_evt(), .rej_evt(),
            .short_cnt(ch_short[16*g +: 16]), .sat_cnt(ch_sat[16*g +: 16]), .rej_cnt(ch_rej[16*g +: 16]));

        // calibration LUT: LUT_DIR/ch<g>.mem, or the ideal LUT
        wire [PS_W-1:0] fine_ps;
        if (LUT_DIR == "") begin : ideal
            tdc_calib_lut #(.FINE_W(FINE_W), .PS_W(PS_W), .IDEAL_LSB_PS(IDEAL_LSB_PS), .LUT_FILE("")) u_lut (
                .clk(clk), .rd_addr(ev_fine), .rd_data(fine_ps),
                .cal_we(cal_we && cal_ch == g), .cal_addr(cal_addr), .cal_wdata(cal_wdata));
        end else begin : file
            // file name LUT_DIR/ch<g>.mem (single-digit channel numbers)
            localparam [7:0] CH_CHAR = 8'd48 + g;
            tdc_calib_lut #(.FINE_W(FINE_W), .PS_W(PS_W), .IDEAL_LSB_PS(IDEAL_LSB_PS),
                            .LUT_FILE({LUT_DIR, "/ch", CH_CHAR, ".mem"})) u_lut (
                .clk(clk), .rd_addr(ev_fine), .rd_data(fine_ps),
                .cal_we(cal_we && cal_ch == g), .cal_addr(cal_addr), .cal_wdata(cal_wdata));
        end

        reg                ev1_valid = 1'b0;
        reg [COARSE_W-1:0] ev1_coarse;
        reg [FINE_W-1:0]   ev1_fine;
        always @(posedge clk) begin
            ev1_valid  <= ev_valid & ~rst;
            ev1_coarse <= ev_coarse;
            ev1_fine   <= ev_fine;
        end

        // per-channel FIFO (drop + count when full, never overwrite)
        reg [QW-1:0]  mem [0:(1<<QAW)-1];
        reg [QAW:0]   wr = 0, rd = 0;
        reg [15:0]    drop = 16'd0;
        wire          full = (wr[QAW] != rd[QAW]) && (wr[QAW-1:0] == rd[QAW-1:0]);
        always @(posedge clk) begin
            if (rst) begin
                wr <= 0; rd <= 0; drop <= 16'd0;
            end else begin
                if (ev1_valid) begin
                    if (!full) begin
                        mem[wr[QAW-1:0]] <= {ev1_coarse, ev1_fine, fine_ps};
                        wr <= wr + 1'b1;
                    end else if (drop != 16'hFFFF) begin
                        drop <= drop + 1'b1;
                    end
                end
                if (q_pop[g]) rd <= rd + 1'b1;
            end
        end
        assign q_nonempty[g]     = (wr != rd);
        assign q_head[g]         = mem[rd[QAW-1:0]];
        assign ch_drop[16*g +: 16] = drop;
    end endgenerate

    // ---------------- merger ----------------
    // eligible: head exists and its coarse is old enough that no channel can
    // still deliver an earlier hit. Key: calibrated time c*T - fine_ps.
    reg [NUM_CH-1:0]           elig;
    reg signed [COARSE_W+15:0] key [0:NUM_CH-1];
    reg                        found;
    reg [CH_W-1:0]             best;
    reg signed [COARSE_W+15:0] best_key;
    integer                    i;
    wire out_free = !m_valid || m_ready;

    always @(*) begin
        found = 1'b0;
        best = {CH_W{1'b0}};
        best_key = 0;
        for (i = 0; i < NUM_CH; i = i + 1) begin
            elig[i] = q_nonempty[i] &&
                      ((coarse_now - q_head[i][QW-1 -: COARSE_W]) >= (MAXLAT + 2)) &&
                      !((coarse_now - q_head[i][QW-1 -: COARSE_W]) >> (COARSE_W - 1));
            key[i]  = $signed({16'd0, q_head[i][QW-1 -: COARSE_W]}) * T_CLK_PS
                      - $signed({{COARSE_W{1'b0}}, q_head[i][PS_W-1:0]});
            if (elig[i] && (!found || key[i] < best_key)) begin
                found = 1'b1;
                best = i[CH_W-1:0];
                best_key = key[i];
            end
        end
        q_pop = {NUM_CH{1'b0}};
        if (found && out_free && !rst)
            q_pop[best] = 1'b1;
    end

    always @(posedge clk) begin
        if (rst) begin
            m_valid <= 1'b0;
        end else begin
            if (m_valid && m_ready) m_valid <= 1'b0;
            if (found && out_free) begin
                m_valid   <= 1'b1;
                m_ch      <= best;
                m_coarse  <= q_head[best][QW-1 -: COARSE_W];
                m_fine    <= q_head[best][PS_W +: FINE_W];
                m_fine_ps <= q_head[best][PS_W-1:0];
            end
        end
    end
endmodule

`default_nettype wire
