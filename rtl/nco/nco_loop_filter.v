// =============================================================================
// nco_loop_filter.v  --  type-2 PI loop filter with gear shifting and
//                        hit-rate gain scaling (E stage)
// -----------------------------------------------------------------------------
// One update per loop-updating hit (not per clock). With e = phase error in
// slots (positive = the NCO is AHEAD of the photon):
//
//   phase correction  dphi = -e * 2^-kp                    (proportional path)
//   freq. correction  df   = -e * 2^-(2kp+1) / (2^L * 2^12) slots/ps  (integral)
//
// * Ki = Kp^2/2 per hit gives damping 0.707 (fast settling, no ringing) and
//   keeps both gains pure shifts when Kp = 2^-kp.
// * The frequency integrator lives in nco_core (fps): these are increments.
// * HIT-RATE GAIN SCALING. Between hits, a frequency error turns into a
//   phase error proportional to the time between hits. To keep the loop's
//   per-hit dynamics the same at 10 MHz and at 1 kHz hit rates, the frequency
//   step is divided by the mean hit spacing tau = 2^L clocks (a shift, no
//   divider). The 2^12 is T_CLK = 4000 ps rounded to a power of two (2.4 %
//   gain error, harmless). tau is an exponential average (weight 1/16) of
//   the spacing between loop-updating hits, seeded by the first interval,
//   and rounded to the nearest power of two (break at 1.5x). Registers can
//   force L (tau_ovr_en / tau_ovr), e.g. if the expected rate is known.
// * GEAR SHIFTING. kp starts at kp_acq (2 -> Kp = 1/4: fast pull-in). Once
//   the lock detector says the phase is acquired (state RAMP), kp steps up
//   by one every 2^gear_log2 loop updates until kp_trk (6 -> Kp = 1/64: low
//   noise), then one more 2^gear_log2-update dwell sets gear_done, which
//   the lock detector needs before declaring LOCK. The integrator (fps) is
//   never reset, so the frequency learned at each gear carries over.
//   Why gradual, and why before LOCK: model/loop_float.py docstring.
//
// Rounding: dphi is exact (e24 << (16-kp), kp <= 16); df is round-half-up.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_loop_filter #(
    parameter integer COARSE_W      = 48,
    parameter integer TAU_INIT_LOG2 = 16
)(
    input  wire               clk,
    input  wire               rst,
    // configuration (runtime registers)
    input  wire [3:0]         kp_acq,
    input  wire [3:0]         kp_trk,
    input  wire [3:0]         gear_log2,
    input  wire               tau_ovr_en,
    input  wire [4:0]         tau_ovr,
    // E stage
    input  wire               e_valid,
    input  wire signed [24:0] e24,
    input  wire               noise,
    input  wire               acquired,      // state != ACQ (before this hit)
    input  wire [COARSE_W-1:0] coarse,
    // from the lock detector (F stage / idle cycles)
    input  wire               to_acq,        // back to acquisition
    input  wire               enter_ramp,    // phase acquired: start the gear ramp
    // to nco_core (valid in the F cycle)
    output reg                corr_valid = 1'b0,
    output reg  signed [47:0] corr_dphi = 48'sd0,
    output reg  signed [47:0] corr_df = 48'sd0,
    // state
    output reg  [3:0]         kp = 4'd2,
    output reg                gear_done = 1'b0,
    output wire [4:0]         L
);
    reg [15:0]         gear_cnt = 16'd0;
    reg [35:0]         tavg = 36'd0;          // 32.4 clocks
    reg                tavg_valid = 1'b0;
    reg [COARSE_W-1:0] c_prev = {COARSE_W{1'b0}};
    reg                prev_valid = 1'b0;

    // ---- L = round_log2(tavg >> 4): priority encoder + next bit ----
    wire [31:0] ti = tavg[35:4];
    reg  [5:0]  msb;
    integer     i;
    always @(*) begin
        msb = 6'd0;
        for (i = 0; i < 32; i = i + 1)
            if (ti[i]) msb = i[5:0];
    end
    wire [5:0] l_rnd = (ti == 32'd0) ? 6'd0 : msb + ((msb != 0) ? ti[msb - 1] : 1'b0);
    wire [4:0] l_est = (l_rnd > 6'd30) ? 5'd30 : l_rnd[4:0];
    assign L = tau_ovr_en ? tau_ovr : (tavg_valid ? l_est : TAU_INIT_LOG2[4:0]);

    // ---- gains ----
    wire signed [24:0] ne  = -e24;
    wire        [4:0]  shp = 5'd16 - {1'b0, kp};                 // 16 - kp
    wire signed [47:0] dphi = $signed({{23{ne[24]}}, ne}) <<< shp;
    wire        [6:0]  sdf = {kp, 1'b1} + 7'd12 + {2'd0, L};      // 2kp+1+12+L
    wire signed [95:0] x   = $signed({{39{ne[24]}}, ne, 32'd0});  // -e * 2^32
    wire signed [95:0] xr  = x + ($signed(96'd1) <<< (sdf - 1'b1));
    wire signed [95:0] dfw = xr >>> sdf;

    // ---- tau estimate ----
    wire [COARSE_W-1:0] dc  = coarse - c_prev;
    wire [31:0]         d32 = (|dc[COARSE_W-1:32]) ? 32'hFFFFFFFF : dc[31:0];
    wire signed [37:0]  tdiff = $signed({2'b00, d32, 4'd0}) - $signed({2'b00, tavg});
    wire signed [37:0]  tstep = (tdiff + 38'sd8) >>> 4;

    wire upd = e_valid & ~noise;

    always @(posedge clk) begin
        corr_valid <= 1'b0;
        if (rst) begin
            kp <= kp_acq; gear_cnt <= 16'd0; gear_done <= 1'b0;
            tavg <= 36'd0; tavg_valid <= 1'b0; prev_valid <= 1'b0;
        end else begin
            if (upd) begin
                corr_valid <= 1'b1;
                corr_dphi  <= dphi;
                corr_df    <= dfw[47:0];
                if (prev_valid) begin
                    if (!tavg_valid) begin
                        tavg <= {d32, 4'd0};
                        tavg_valid <= 1'b1;
                    end else begin
                        tavg <= tavg + tstep[35:0];
                    end
                end
                c_prev <= coarse;
                prev_valid <= 1'b1;
                if (acquired) begin
                    if (gear_cnt == (16'd1 << gear_log2) - 1'b1) begin
                        gear_cnt <= 16'd0;
                        if (kp < kp_trk) kp <= kp + 1'b1;
                        else             gear_done <= 1'b1;
                    end else begin
                        gear_cnt <= gear_cnt + 1'b1;
                    end
                end
            end
            // lock-detector decisions (never in the same cycle as an E update:
            // nco_align's POP_GAP guarantees that)
            if (to_acq) begin
                kp <= kp_acq; gear_cnt <= 16'd0; gear_done <= 1'b0;
            end else if (enter_ramp) begin
                gear_cnt <= 16'd0; gear_done <= 1'b0;
            end
        end
    end
endmodule

`default_nettype wire
