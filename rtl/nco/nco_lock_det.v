// =============================================================================
// nco_lock_det.v  --  lock state machine, error averages, slip suspicion
// -----------------------------------------------------------------------------
// STATES   ACQ (0) -> RAMP (1) -> LOCK (2)
//   ACQ   fast acquisition gear, gate open. When the average of e^2 drops
//         below lock_thr the phase is "acquired": -> RAMP.
//   RAMP  the loop filter steps Kp down to the tracking gear, gate armed.
//         NOT locked yet. If the average rises above unlock_thr, rej_max
//         consecutive hits are gated out, or no hit arrives for the timeout,
//         acquisition restarts (abort_cnt). Once the filter reports
//         gear_done and the average is still below lock_thr: -> LOCK.
//   LOCK  slot numbering is claimed continuous. Same exits as RAMP, plus a
//         suspected slip (below); each exit counts in unlock_cnt.
//   Declaring LOCK only at the tracking gear is what made "zero undetected
//   slips" hold in the float model (see model/loop_float.py).
//
// AVERAGES (exponential, shift-only)
//   mse  average of e^2 (2^-32 slot^2). Weight 2^-mse_shift while ACQ/RAMP
//        (fast lock), 2^-mse_shift_trk in LOCK (fewer false unlocks under dark
//        counts). Random phase gives 1/12; restarts at 1/12 on every exit.
//   sgn  average of the SIGNED error over ALL hits while locked, weight 2^-5.
//        Dark counts average to zero; a phase walking towards +/-0.5 slot
//        (the start of a cycle slip) pulls it to one side. |sgn| > slip_thr
//        counts a SUSPECTED SLIP (re-armed below slip_thr/2) and, if
//        slip_unlock is set (default), forces an unlock so the loss of slot
//        integrity is always reported, never silent.
//   Noise limit of any e^2 metric: dark counts at a fraction r of the signal
//   raise the locked average to about (sigma^2 + r/12)/(1 + r). Defaults
//   (1/64, 1/16) declare lock up to r ~ 0.6 and hold it up to r ~ 3.
//
// NO-HIT TIMEOUT: (t_nco - c_last) > 2^(L + to_shift) clocks -> exit. With the
//   default to_shift = 5 that is 32 mean hit spacings (false-timeout probability
//   e^-32 per hit), so at low rates a short signal gap is flywheeled through
//   instead of dropping lock. The check takes 3 cycles: cycle X acts on
//   t_nco[X-3]. It is blocked from a hit's E stage until its F stage + 3, because
//   by then the idle count has caught up with that hit's c_last. model/loop_fixed.py
//   has the same rule.
//
// PIPELINE (E = n+7, F = E+6 = n+13; the next hit's E is at least 8 cycles later)
//   E      mx low   (sq - mse)
//   E+1    mx high
//   E+2    arithmetic shift by w, plus the bit below it (round half up)
//   E+3    mse + shift, low
//   E+4    ... high: mse committed, visible in E+5
//   E+5    threshold compares
//   F      decision: to_acq, enter_ramp, locked_next (combinational)
// The sgn candidates and their threshold flags are ready before F.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_lock_det #(
    parameter integer COARSE_W = 48
)(
    input  wire               clk,
    input  wire               rst,
    input  wire [COARSE_W-1:0] t_nco,
    // configuration
    input  wire [3:0]         mse_shift,
    input  wire [3:0]         mse_shift_trk,
    input  wire [31:0]        lock_thr,
    input  wire [31:0]        unlock_thr,
    input  wire [7:0]         rej_max,
    input  wire [4:0]         to_shift,
    input  wire [15:0]        slip_thr,       // 2^-16 slot
    input  wire               slip_unlock,
    // E stage
    input  wire               e_valid,        // E
    input  wire signed [24:0] e24,            // stable from before E until after F
    input  wire [31:0]        sq,             // valid at E and E+1
    input  wire               noise,
    input  wire               st_commit,      // E+4
    input  wire               busy,           // hit between E and F + 3
    input  wire [COARSE_W-1:0] coarse,        // stable until after F
    // F stage
    input  wire               f_valid,
    input  wire               gear_done,
    input  wire [4:0]         L,
    // decisions
    output wire               to_acq,
    output wire               enter_ramp,
    output wire               locked_next,    // state after this cycle == LOCK
    output reg  [1:0]         state = 2'd0,
    output reg  [31:0]        mse = 32'd357913941,     // 1/12
    output reg  signed [24:0] sgn = 25'sd0,
    // counters (16 bit, saturating)
    output reg  [15:0]        unlock_cnt = 16'd0,
    output reg  [15:0]        abort_cnt = 16'd0,
    output reg  [15:0]        suspect_cnt = 16'd0,
    output reg  [15:0]        noise_cnt = 16'd0,
    output reg  [15:0]        hit_cnt = 16'd0
);
    localparam [1:0]  ACQ = 2'd0, RAMP = 2'd1, LOCK = 2'd2;
    localparam [31:0] MSE_INIT = 32'd357913941;           // floor(2^32 / 12)

    reg [7:0]          rej = 8'd0;
    reg                rej_ge = 1'b0;
    reg                slip_armed = 1'b1;
    reg [COARSE_W-1:0] c_last = {COARSE_W{1'b0}};
    reg                last_valid = 1'b0;

    // ---- mse: mse + rhu(sq - mse, w) ----
    reg  [3:0]         w = 4'd0;
    reg  [17:0]        mx_lo = 18'd0;
    reg  [16:0]        mx_hi = 17'd0;
    wire signed [33:0] mx = {mx_hi, mx_lo[16:0]};
    reg  signed [33:0] sh = 34'sd0;
    reg                sh_rb = 1'b0;
    reg  [16:0]        mn_lo = 17'd0;
    wire [15:0]        mn_hi = mse[31:16] + sh[31:16] + mn_lo[16];   // E+4, with the carry
    always @(posedge clk) begin
        if (e_valid) w <= (state == LOCK) ? mse_shift_trk : mse_shift;
        // (w is used from E+2; at E itself nothing depends on it yet)
        mx_lo <= {1'b0, sq[16:0]} - {1'b0, mse[16:0]};
        mx_hi <= {2'b00, sq[31:17]} - {2'b00, mse[31:17]} - mx_lo[17];
        sh    <= mx >>> w;
        sh_rb <= (w == 4'd0) ? 1'b0 : mx[w - 1'b1];
        mn_lo <= {1'b0, mse[15:0]} + {1'b0, sh[15:0]} + sh_rb;
    end

    // ---- sgn candidates and flags, all settled well before F ----
    wire signed [25:0] sd_w = e24 - sgn;
    reg  signed [25:0] sd = 26'sd0;
    // (kept apart: an unsigned bit-select in the same expression would make >>> logical)
    wire signed [25:0] sd_s = sd >>> 5;
    wire signed [24:0] e_s  = e24 >>> 5;
    reg  signed [25:0] sc = 26'sd0, sn = 26'sd0;
    reg  [24:0]        a_c = 25'd0, a_n = 25'd0;
    wire [24:0]        thr_hi = {1'b0, slip_thr, 8'd0};
    wire [24:0]        thr_lo = {2'b0, slip_thr, 7'd0};
    reg                c_gt = 1'b0, c_lt = 1'b0, n_gt = 1'b0, n_lt = 1'b0;
    always @(posedge clk) begin
        sd  <= sd_w;
        sc  <= sgn + sd_s + $signed({1'b0, sd[4]});
        sn  <= e_s + $signed({1'b0, e24[4]});
        a_c <= sc[24] ? -sc[24:0] : sc[24:0];
        a_n <= sn[24] ? -sn[24:0] : sn[24:0];
        c_gt <= a_c > thr_hi;   c_lt <= a_c < thr_lo;
        n_gt <= a_n > thr_hi;   n_lt <= a_n < thr_lo;
    end

    // ---- threshold flags ----
    reg m_gt_unlock = 1'b0, m_lt_lock = 1'b0;
    always @(posedge clk) begin
        m_gt_unlock <= mse > unlock_thr;
        m_lt_lock   <= mse < lock_thr;
        rej_ge      <= rej >= rej_max;
    end

    // ---- F stage decisions ----
    wire exit_cond  = m_gt_unlock || rej_ge;
    wire acq_ok     = m_lt_lock;
    wire f_to_ramp  = f_valid && (state == ACQ) && acq_ok;
    wire f_exit     = f_valid && (state != ACQ) && exit_cond;
    wire f_to_lock  = f_valid && (state == RAMP) && !exit_cond && gear_done && acq_ok;
    wire stay_lock  = f_valid && (state == LOCK) && !exit_cond;
    wire armed      = f_to_lock ? 1'b1 : slip_armed;
    wire v_gt       = f_to_lock ? n_gt : c_gt;
    wire v_lt       = f_to_lock ? n_lt : c_lt;
    wire suspect    = (f_to_lock || stay_lock) && armed && v_gt;
    wire rearm      = (f_to_lock || stay_lock) && !armed && v_lt;
    wire f_slip_exit = suspect && slip_unlock;

    // ---- timeout: idle - 1 = t_nco + ~c_last; idle > 2^s <=> (idle - 1) >> s != 0 ----
    reg  [24:0]         id_lo = 25'd0;
    reg  [COARSE_W-1:0] id = {COARSE_W{1'b0}};
    reg  [COARSE_W-25:0] t_hi = 0, nc_hi = 0;
    reg  [6:0]          to_s = 7'd0;
    reg  [COARSE_W-1:0] to_mask = {COARSE_W{1'b1}};
    reg                 to_flag = 1'b0;
    always @(posedge clk) begin
        id_lo   <= {1'b0, t_nco[23:0]} + {1'b0, ~c_last[23:0]};
        t_hi    <= t_nco[COARSE_W-1:24];
        nc_hi   <= ~c_last[COARSE_W-1:24];
        id      <= {t_hi + nc_hi + id_lo[24], id_lo[23:0]};
        to_s    <= {2'd0, L} + {2'd0, to_shift};
        to_mask <= (to_s >= COARSE_W) ? {COARSE_W{1'b0}} : ({COARSE_W{1'b1}} << to_s);
        to_flag <= |(id & to_mask);
    end
    wire timeout = !busy && (state != ACQ) && last_valid && to_flag;

    assign to_acq     = f_exit || f_slip_exit || timeout;
    assign enter_ramp = f_to_ramp;
    assign locked_next = !to_acq && ((state == LOCK) || f_to_lock);

    // which state is being left (for the counters)
    wire was_lock = (state == LOCK) || f_to_lock;     // f_to_lock + slip exit = an unlock

    always @(posedge clk) begin
        if (rst) begin
            state <= ACQ; mse <= MSE_INIT; rej <= 8'd0; sgn <= 25'sd0; slip_armed <= 1'b1;
            last_valid <= 1'b0;
            unlock_cnt <= 16'd0; abort_cnt <= 16'd0; suspect_cnt <= 16'd0;
            noise_cnt <= 16'd0; hit_cnt <= 16'd0;
        end else begin
            // E stage: reject run
            if (e_valid) begin
                rej <= noise ? ((rej == 8'hFF) ? rej : rej + 1'b1) : 8'd0;
                if (noise && noise_cnt != 16'hFFFF) noise_cnt <= noise_cnt + 1'b1;
            end
            if (st_commit)
                mse <= {mn_hi, mn_lo[15:0]};
            // F stage: decisions
            if (f_valid) begin
                c_last <= coarse;
                last_valid <= 1'b1;
                if (hit_cnt != 16'hFFFF) hit_cnt <= hit_cnt + 1'b1;
                if (f_to_ramp) state <= RAMP;
                if (f_to_lock) state <= LOCK;
                if (f_to_lock || stay_lock) begin
                    sgn <= f_to_lock ? sn[24:0] : sc[24:0];
                    if (suspect) begin
                        slip_armed <= 1'b0;
                        if (suspect_cnt != 16'hFFFF) suspect_cnt <= suspect_cnt + 1'b1;
                    end else if (rearm) begin
                        slip_armed <= 1'b1;
                    end else if (f_to_lock) begin
                        slip_armed <= 1'b1;
                    end
                end
            end
            if (to_acq) begin
                state <= ACQ;
                mse   <= MSE_INIT;
                rej   <= 8'd0;
                if (f_slip_exit ? was_lock : (state == LOCK)) begin
                    if (unlock_cnt != 16'hFFFF) unlock_cnt <= unlock_cnt + 1'b1;
                end else if (abort_cnt != 16'hFFFF) begin
                    abort_cnt <= abort_cnt + 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
