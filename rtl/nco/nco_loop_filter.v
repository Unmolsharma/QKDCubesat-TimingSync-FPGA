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
//
// PIPELINE (n = the hit's pop cycle; E = n+7, see nco_top)
//   n+2..n+4  d = coarse - previous coarse, saturated to 32 bits
//   n+5, n+6  next tavg, ready to commit
//   E         sample kp and L; commit tavg, gear step
//   E+1       sdf = 2kp+1+12+L
//   E+2       shift direction and amounts
//   E+3       barrel shifts
//   E+4       round: corr_valid, corr_dphi, corr_df  (n+11)
//
// WHY THERE IS NO 96-BIT ADDER. df = rhu(-e * 2^32, sdf). When sdf <= 32 that is an
// exact left shift by 32-sdf. Otherwise it is rhu(-e, sdf-32), and
// rhu(x, s) = floor(x / 2^s) + x[s-1], which needs only a 26-bit increment.
//
// WHY THERE IS ONE tavg ADDER. Writing tavg = 16*Th + Tl,
//   tavg + rhu(16*d - tavg, 4) = (tavg - Th - 1) + d + (Tl <= 8)
// so it takes one addition with a carry-in, with tavg - Th - 1 = tavg + ~Th
// prepared ahead of time.
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
    input  wire signed [24:0] ne,            // -e24, stable through E+2
    input  wire               noise,
    input  wire               acquired,      // state != ACQ (before this hit)
    input  wire [COARSE_W-1:0] coarse,       // valid from n+1 until after E
    // from the lock detector (F stage / idle cycles)
    input  wire               to_acq,        // back to acquisition
    input  wire               enter_ramp,    // phase acquired: start the gear ramp
    // to nco_core
    output reg                corr_valid = 1'b0,
    output reg  signed [47:0] corr_dphi = 48'sd0,
    output reg  signed [47:0] corr_df = 48'sd0,
    // state
    output reg  [3:0]         kp = 4'd2,
    output reg                gear_done = 1'b0,
    output reg  [4:0]         L = TAU_INIT_LOG2[4:0]
);
    reg [15:0]         gear_cnt = 16'd0;
    reg                gear_last = 1'b0;      // gear_cnt == 2^gear_log2 - 1
    reg [35:0]         tavg = 36'd0;          // 32.4 clocks
    reg                tavg_valid = 1'b0;
    reg [COARSE_W-1:0] c_prev = {COARSE_W{1'b0}};
    reg                prev_valid = 1'b0;

    // ---- L = round_log2(tavg >> 4), over three cycles ----
    wire [31:0] ti = tavg[35:4];
    wire        b4 = |ti[31:16];
    wire [15:0] w4 = b4 ? ti[31:16] : ti[15:0];
    wire        b3 = |w4[15:8];
    wire [7:0]  w3 = b3 ? w4[15:8] : w4[7:0];
    reg  [1:0]  m_hi = 2'd0;
    reg  [7:0]  m_w = 8'd0;
    reg         m_any = 1'b0;
    reg  [5:0]  msb = 6'd0;
    reg  [31:0] ti_d = 32'd0;
    reg         m_any2 = 1'b0;
    wire        b2 = |m_w[7:4];
    wire [3:0]  w2 = b2 ? m_w[7:4] : m_w[3:0];
    wire        b1 = |w2[3:2];
    wire [1:0]  w1 = b1 ? w2[3:2] : w2[1:0];
    reg  [5:0]  l_rnd = 6'd0;
    always @(posedge clk) begin
        m_hi   <= {b4, b3};
        m_w    <= w3;
        m_any  <= |ti;
        ti_d   <= ti;
        msb    <= {1'b0, m_hi, b2, b1, w1[1]};
        m_any2 <= m_any;
        l_rnd  <= !m_any2 ? 6'd0 : msb + ((msb != 6'd0) ? ti_d[msb[4:0] - 1'b1] : 1'b0);
        L      <= tau_ovr_en ? tau_ovr :
                  tavg_valid ? ((l_rnd > 6'd30) ? 5'd30 : l_rnd[4:0]) : TAU_INIT_LOG2[4:0];
    end

    // ---- d = coarse - c_prev, saturated to 32 bits ----
    reg [24:0] dc_lo = 25'd0;
    reg [COARSE_W-25:0] dc_hi = 0;
    reg [31:0] d32 = 32'd0;
    always @(posedge clk) begin
        dc_lo <= {1'b0, coarse[23:0]} - {1'b0, c_prev[23:0]};
        dc_hi <= coarse[COARSE_W-1:24] - c_prev[COARSE_W-1:24] - dc_lo[24];
        d32   <= (|dc_hi[COARSE_W-25:8]) ? 32'hFFFFFFFF : {dc_hi[7:0], dc_lo[23:0]};
    end

    // ---- next tavg: U = tavg + ~Th = tavg - Th - 1, then U + d + (Tl <= 8) ----
    wire [35:0] nth = ~{4'd0, tavg[35:4]};
    reg  [18:0] u_lo = 19'd0, t_lo = 19'd0;
    reg  [17:0] u_hi = 18'd0, t_hi = 18'd0;
    reg         t_ci = 1'b0;
    always @(posedge clk) begin
        u_lo <= {1'b0, tavg[17:0]} + {1'b0, nth[17:0]};
        u_hi <= tavg[35:18] + nth[35:18] + u_lo[18];
        t_ci <= (tavg[3:0] <= 4'd8);
        t_lo <= {1'b0, u_lo[17:0]} + {1'b0, d32[17:0]} + t_ci;
        t_hi <= u_hi + {4'd0, d32[31:18]} + t_lo[18];
    end

    // ---- gains ----
    wire upd = e_valid & ~noise;
    reg        u1 = 1'b0, u2 = 1'b0, u3 = 1'b0;
    reg  [3:0] kp_e = 4'd0;
    reg  [6:0] sdf = 7'd0;

    // E+2: direction and amounts
    reg        left = 1'b0;
    reg  [4:0] sh_l = 5'd0;               // sdf <= 32: left by 32 - sdf (0..19)
    reg  [4:0] sh_r = 5'd0;               // sdf > 32:  right by sdf - 32, clamped to 26
    reg  [4:0] sh_p = 5'd0;               // 16 - kp
    wire [6:0] over = sdf - 7'd32;

    // E+3: barrel shifts
    wire signed [47:0] nx = {{23{ne[24]}}, ne};
    reg  signed [47:0] dp = 48'sd0, v = 48'sd0;
    reg                rb = 1'b0;
    reg                left_d = 1'b0;

    // E+4: round; the right-shift result fits in 26 bits, so the increment is short
    wire [25:0] v_inc = v[25:0] + rb;

    always @(posedge clk) begin
        // E: sample the state this hit uses
        if (e_valid) begin
            kp_e <= kp;
            sdf  <= {kp, 1'b1} + 7'd12 + {2'd0, L};
        end
        u1 <= upd & ~rst;
        // E+1
        left <= (sdf <= 7'd32);
        sh_l <= 7'd32 - sdf;
        sh_r <= (over > 7'd26) ? 5'd26 : over[4:0];
        sh_p <= 5'd16 - {1'b0, kp_e};
        u2   <= u1 & ~rst;
        // E+2
        dp     <= nx <<< sh_p;
        v      <= left ? (nx <<< sh_l) : (nx >>> sh_r);
        rb     <= left ? 1'b0 : ((sh_r == 5'd0) ? 1'b0 : nx[sh_r - 1'b1]);
        left_d <= left;
        u3     <= u2 & ~rst;
        // E+3
        corr_valid <= u3 & ~rst;
        corr_dphi  <= dp;
        corr_df    <= left_d ? v : {{22{v_inc[25]}}, v_inc};
    end

    always @(posedge clk) begin
        gear_last <= (gear_cnt == (16'd1 << gear_log2) - 1'b1);
        if (rst) begin
            kp <= kp_acq; gear_cnt <= 16'd0; gear_done <= 1'b0;
            tavg <= 36'd0; tavg_valid <= 1'b0; prev_valid <= 1'b0;
        end else begin
            if (upd) begin
                if (prev_valid) begin
                    if (!tavg_valid) begin
                        tavg <= {d32, 4'd0};
                        tavg_valid <= 1'b1;
                    end else begin
                        tavg <= {t_hi, t_lo[17:0]};
                    end
                end
                c_prev <= coarse;
                prev_valid <= 1'b1;
                if (acquired) begin
                    if (gear_last) begin
                        gear_cnt <= 16'd0;
                        if (kp < kp_trk) kp <= kp + 1'b1;
                        else             gear_done <= 1'b1;
                    end else begin
                        gear_cnt <= gear_cnt + 1'b1;
                    end
                end
            end
            // Follow runtime changes of the gear settings (nco_cfg_regs): while
            // acquiring, kp IS kp_acq; after that it never exceeds kp_trk. With
            // constant settings both lines are no-ops.
            if (!acquired)
                kp <= kp_acq;
            else if (kp > kp_trk)
                kp <= kp_trk;
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
