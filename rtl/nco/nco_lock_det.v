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
// NO-HIT TIMEOUT: checked every cycle in which the E and F stages are idle:
//   (t_nco - c_last) > 2^(L + to_shift) clocks -> exit. With the default
//   to_shift = 5 that is 32 mean hit spacings (false-timeout probability
//   e^-32 per hit), so at low rates a short signal gap is flywheeled through
//   instead of dropping lock.
//
// Pipeline: the E-stage inputs update mse/rej/sgn candidates; the F stage
// (next cycle) makes the state decision. Outputs to_acq / enter_ramp are
// combinational in the decision cycle.
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
    input  wire               e_valid,
    input  wire signed [24:0] e24,
    input  wire [31:0]        sq,
    input  wire               noise,
    input  wire [COARSE_W-1:0] coarse,
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
    reg                slip_armed = 1'b1;
    reg signed [24:0]  sgn_cont = 25'sd0, sgn_new = 25'sd0;
    reg [COARSE_W-1:0] c_last = {COARSE_W{1'b0}};
    reg                last_valid = 1'b0;
    reg [COARSE_W-1:0] coarse_f = {COARSE_W{1'b0}};   // coarse of the hit in F
    always @(posedge clk) if (e_valid) coarse_f <= coarse;

    // ---- E stage arithmetic ----
    wire [3:0]         w     = (state == LOCK) ? mse_shift_trk : mse_shift;
    wire signed [33:0] mdiff = $signed({2'b00, sq}) - $signed({2'b00, mse});
    wire signed [33:0] mrnd  = mdiff + ($signed(34'd1) <<< (w - 1'b1));
    wire signed [33:0] mstep = (w == 0) ? mdiff : (mrnd >>> w);
    wire signed [25:0] sdiff = e24 - sgn;
    wire signed [25:0] sstep = (sdiff + 26'sd16) >>> 5;
    wire signed [25:0] snew  = ($signed({e24[24], e24}) + 26'sd16) >>> 5;

    // ---- F stage decisions ----
    wire exit_cond  = (mse > unlock_thr) || (rej >= rej_max);
    wire acq_ok     = (mse < lock_thr);
    wire f_to_ramp  = f_valid && (state == ACQ) && acq_ok;
    wire f_exit     = f_valid && (state != ACQ) && exit_cond;
    wire f_to_lock  = f_valid && (state == RAMP) && !exit_cond && gear_done && acq_ok;
    wire stay_lock  = f_valid && (state == LOCK) && !exit_cond;
    wire signed [24:0] sgn_val = f_to_lock ? sgn_new : sgn_cont;
    wire               armed   = f_to_lock ? 1'b1 : slip_armed;
    wire [24:0]        sgn_abs = sgn_val[24] ? -sgn_val : sgn_val;
    wire               suspect = (f_to_lock || stay_lock) && armed && (sgn_abs > {1'b0, slip_thr, 8'd0});
    wire               rearm   = (f_to_lock || stay_lock) && !armed && (sgn_abs < {2'b0, slip_thr, 7'd0});
    wire               f_slip_exit = suspect && slip_unlock;

    // ---- timeout (idle cycles only) ----
    wire [COARSE_W-1:0] idle = t_nco - c_last;
    wire [63:0]         to_lim = 64'd1 << (L + to_shift);
    wire                timeout = !e_valid && !f_valid && (state != ACQ) && last_valid &&
                                  ({{(64-COARSE_W){1'b0}}, idle} > to_lim);

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
            // E stage: averages and reject run
            if (e_valid) begin
                mse <= mse + mstep[31:0];
                rej <= noise ? ((rej == 8'hFF) ? rej : rej + 1'b1) : 8'd0;
                sgn_cont <= sgn + sstep[24:0];
                sgn_new  <= snew[24:0];
                if (noise && noise_cnt != 16'hFFFF) noise_cnt <= noise_cnt + 1'b1;
            end
            // F stage: decisions
            if (f_valid) begin
                c_last <= coarse_f;
                last_valid <= 1'b1;
                if (hit_cnt != 16'hFFFF) hit_cnt <= hit_cnt + 1'b1;
                if (f_to_ramp) state <= RAMP;
                if (f_to_lock) state <= LOCK;
                if (f_to_lock || stay_lock) begin
                    sgn <= sgn_val;
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
