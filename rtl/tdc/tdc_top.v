// =============================================================================
// tdc_top.v  --  carry-chain TDC channel (block 1 of the timing core)
// -----------------------------------------------------------------------------
//
//            +--------------------------- tdc_frontend -------------------------+
//   hit ---->| CARRY4 x48 --> capture FFs --> sync FFs --> popcount (3 stages) |--> fine
//   (async)  |  192 taps      (in-slice)     (2nd stage)   + t0/zero/full flags|--> flags
//            +------------------------------------------------------------------+
//                                                                  |
//   clk ---> tdc_coarse_counter (delayed by LATENCY=4) ---> coarse |
//                                                                  v
//                                  tdc_hit_detect: trigger / confirm / dead time
//                                                                  |  ev (coarse, fine)
//                                                                  v
//                                  tdc_calib_lut: fine -> fine_ps (1 cycle)
//                                                                  |
//                                                                  v
//                              output register  --> m_valid / m_ready
//                                                   m_raw = {coarse, fine}
//                                                   m_cal = {coarse, fine_ps}
//
// TIME CONVENTION (important for block 2)
//   fine / fine_ps measure how long BEFORE the capture edge the hit arrived:
//
//        t_hit = coarse * T_clk - fine_ps          (+ constant channel offset)
//
//   A larger fine code means an EARLIER hit within that clock period. A hit
//   right at a clock edge can legally come out as (N, ~0) or (N+1, ~T_clk).
//   Both describe the same instant. Always combine coarse and fine with the
//   formula above; never compare fine codes across different coarse values.
//   The constant offset (IO + routing to tap 0 + LUT origin) is identical
//   for every hit, so the tracking loop can absorb it or a system-level
//   calibration can remove it.
//
// OUTPUT INTERFACE (AXI-Stream-like, one beat per hit)
//   m_valid/m_ready handshake, a 1-deep output register. Hits are at least
//   one dead time apart (>= ~12 ns = 3 cycles), so one register is enough when
//   the consumer is always ready. If a new hit arrives while the register is
//   still full (m_valid && !m_ready), that hit is DROPPED and `overflow` pulses
//   (counted in drop_cnt). Earlier data is never overwritten or corrupted.
//
// LATENCY hit -> m_valid: capture edge + 4 (sync + encoder) + 1 (detect)
//   + 1..2 (confirm "full") + 1 (LUT) + 1 (output reg) ~ 8-9 clk (32-36 ns).
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_top #(
    parameter integer NUM_CARRY4    = 48,
    parameter integer FINE_W        = 9,
    parameter integer COARSE_W      = 32,
    parameter integer PS_W          = 16,
    parameter integer IDEAL_LSB_PS  = 22,
    parameter         LUT_FILE      = "",
    parameter integer CONFIRM_CYC   = 3,
    // behavioural delay-line model (simulation only)
    parameter real    TAP_NOM_PS    = 22.0,
    parameter real    TAP_SIGMA_PCT = 15.0,
    parameter real    SKEW_SIGMA_PS = 6.0,
    parameter real    JITTER_PS     = 4.0,
    parameter integer SEED          = 1
)(
    input  wire                       clk,
    input  wire                       rst,          // synchronous, active high
    input  wire                       hit,          // asynchronous hit input
    // timestamp stream
    output reg                        m_valid = 1'b0,
    input  wire                       m_ready,
    output reg  [COARSE_W+FINE_W-1:0] m_raw = {(COARSE_W+FINE_W){1'b0}},
    output reg  [COARSE_W+PS_W-1:0]   m_cal = {(COARSE_W+PS_W){1'b0}},
    // status
    output reg                        overflow = 1'b0,   // 1-cycle pulse per dropped hit
    output wire                       dead,
    output reg  [15:0]                drop_cnt = 16'd0,
    output wire [15:0]                short_cnt,
    output wire [15:0]                sat_cnt,
    output wire [15:0]                rej_cnt,
    output wire [COARSE_W-1:0]        coarse_now,        // live time base
    // calibration LUT write port (for block 4; tie cal_we low if unused)
    input  wire                       cal_we,
    input  wire [FINE_W-1:0]          cal_addr,
    input  wire [PS_W-1:0]            cal_wdata
);
    localparam integer LATENCY = 4;    // must match tdc_frontend

    wire [FINE_W-1:0]   fe_fine;
    wire                fe_t0, fe_zero, fe_full;
    wire [COARSE_W-1:0] coarse_aligned;

    tdc_frontend #(
        .NUM_CARRY4    (NUM_CARRY4),
        .FINE_W        (FINE_W),
        .TAP_NOM_PS    (TAP_NOM_PS),
        .TAP_SIGMA_PCT (TAP_SIGMA_PCT),
        .SKEW_SIGMA_PS (SKEW_SIGMA_PS),
        .JITTER_PS     (JITTER_PS),
        .SEED          (SEED)
    ) u_fe (
        .clk  (clk),
        .hit  (hit),
        .fine (fe_fine),
        .t0   (fe_t0),
        .zero (fe_zero),
        .full (fe_full)
    );

    tdc_coarse_counter #(.COARSE_W(COARSE_W), .DELAY(LATENCY)) u_cc (
        .clk            (clk),
        .rst            (rst),
        .coarse         (coarse_now),
        .coarse_aligned (coarse_aligned)
    );

    wire                ev_valid;
    wire [COARSE_W-1:0] ev_coarse;
    wire [FINE_W-1:0]   ev_fine;

    tdc_hit_detect #(
        .FINE_W      (FINE_W),
        .COARSE_W    (COARSE_W),
        .CONFIRM_CYC (CONFIRM_CYC)
    ) u_hd (
        .clk       (clk),
        .rst       (rst),
        .fine      (fe_fine),
        .t0        (fe_t0),
        .zero      (fe_zero),
        .full      (fe_full),
        .coarse    (coarse_aligned),
        .ev_valid  (ev_valid),
        .ev_coarse (ev_coarse),
        .ev_fine   (ev_fine),
        .dead      (dead),
        .short_evt (),
        .sat_evt   (),
        .rej_evt   (),
        .short_cnt (short_cnt),
        .sat_cnt   (sat_cnt),
        .rej_cnt   (rej_cnt)
    );

    // ---- calibration lookup (1-cycle read) ----
    wire [PS_W-1:0] fine_ps;

    tdc_calib_lut #(
        .FINE_W       (FINE_W),
        .PS_W         (PS_W),
        .IDEAL_LSB_PS (IDEAL_LSB_PS),
        .LUT_FILE     (LUT_FILE)
    ) u_lut (
        .clk       (clk),
        .rd_addr   (ev_fine),
        .rd_data   (fine_ps),
        .cal_we    (cal_we),
        .cal_addr  (cal_addr),
        .cal_wdata (cal_wdata)
    );

    reg                ev1_valid = 1'b0;
    reg [COARSE_W-1:0] ev1_coarse = {COARSE_W{1'b0}};
    reg [FINE_W-1:0]   ev1_fine = {FINE_W{1'b0}};

    always @(posedge clk) begin
        ev1_valid  <= ev_valid & ~rst;
        ev1_coarse <= ev_coarse;
        ev1_fine   <= ev_fine;
    end

    // ---- output register with drop-on-full ----
    always @(posedge clk) begin
        overflow <= 1'b0;
        if (rst) begin
            m_valid  <= 1'b0;
            drop_cnt <= 16'd0;
        end else begin
            if (m_valid && m_ready)
                m_valid <= 1'b0;
            if (ev1_valid) begin
                if (!m_valid || m_ready) begin
                    m_valid <= 1'b1;
                    m_raw   <= {ev1_coarse, ev1_fine};
                    m_cal   <= {ev1_coarse, fine_ps};
                end else begin
                    overflow <= 1'b1;
                    if (drop_cnt != 16'hFFFF) drop_cnt <= drop_cnt + 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
