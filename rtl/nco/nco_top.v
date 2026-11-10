// =============================================================================
// nco_top.v  --  block 2: clock recovery from sparse photon timestamps
// -----------------------------------------------------------------------------
//
//  TDC timestamps            +-----------+  pop  +-------------+ D  +---------+ E
//  {coarse, fine_ps, tag} -->| nco_align |------>|nco_phase_det|--->| gate    |--+
//  s_valid / s_ready         |  FIFO     |       | A B C D     |    | loop    |  |
//                            +-----------+       +-------------+    | filter  |  |
//                                 ^ t_nco             ^ phase, fps  | lock det|  |
//  time_now (TDC coarse_now) --> -D                   |             +---------+  |
//                                                 +---------+  corrections (F)   |
//                       ext_* correction port --->| nco_core|<-------------------+
//                                                 +---------+
//                                                      F stage: output register
//  per hit out: {slot[47:0], residual_ps, noise_flag, locked, e24, tag}
//
// TIME CONVENTION (same as block 1): a timestamp {c, fine_ps} means
//     t_hit = c * T_clk - fine_ps.
// t_nco = time_now - D is the NCO's own time axis: the phase register at
// t_nco = n is the NCO phase at TDC clock edge n.
//
// WHAT "slot" MEANS: round(NCO phase at the photon). Once locked, every
// signal photon gets slot = (its true source slot) + K, with one constant K
// for the whole lock. K is arbitrary (there is no framing in this block);
// the sync-pattern framing that fixes K is a later block. If the lock is
// ever lost, `locked` drops and unlock_cnt increments: K may have changed and
// must be re-established. While `locked` stays high, K must not change:
// that is the zero-cycle-slip requirement.
//
// PIPELINE (cycles after the pop cycle n), sized for 250 MHz:
//   n+1 .. n+6   phase detector: error ready
//   n+7  = E     gate; sample kp and L; tavg, gear, reject run
//   n+11         loop corrections strobed into nco_core
//   n+12         mse updated
//   n+13 = F     state machine, output register
//   n+15         frequency correction visible in fps
//   n+18         phase correction visible in phase
// A STALE hit (released more than J_LOOP = 24 cycles late: the 5th hit of a
// coincidence) runs through the same stages and is output, but every loop strobe
// is masked for it, so it changes no loop state and adds no loop delay.
// A hit's state updates are all visible by F+2 = E+8 (the registered clears of an
// exit land last), and nco_align releases at most one hit every POP_GAP = 8 cycles, so every hit sees the state left by the
// previous one, exactly as if the loop were not pipelined. The corrections can
// arrive after the next hit has been sampled (hits < 18 cycles apart): that is an
// extra loop delay of up to 2 hits at the highest rates. README: "Loop latency".
// model/loop_fixed.py is the bit- and cycle-exact reference.
//
// OUTPUT: m_valid/m_ready, 1-deep register. A result that finds the register
// still full is dropped and counted in out_drop_cnt (the loop state is never
// stalled: the NCO is a real-time process).
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_top #(
    parameter [47:0]  FPS_NOM       = 48'd7205759403793,  // 100 MHz: round(f*1e-12*2^56)
    parameter integer T_SLOT_PS     = 10000,              // nominal slot, for residual_ps
    parameter integer T_CLK_PS      = 4000,
    parameter integer D             = 23,
    parameter integer J_LOOP        = 24,                 // loop uses hits released <= J_LOOP late
    parameter integer J_MAX         = 56,                 // later (coincidences): delivered only
    parameter integer W_STALE       = 48,                 // ... if within this of the last loop hit
    parameter integer POP_GAP       = 8,
    parameter integer TAG_W         = 4,
    parameter integer FIFO_AW       = 4,
    parameter integer TAU_INIT_LOG2 = 16
)(
    input  wire               clk,
    input  wire               rst,
    input  wire [47:0]        time_now,      // TDC coarse_now
    // timestamps in
    input  wire               s_valid,
    output wire               s_ready,
    input  wire [47:0]        s_coarse,
    input  wire [15:0]        s_fine,        // fine_ps
    input  wire [TAG_W-1:0]   s_tag,
    // per-hit results
    output reg                m_valid = 1'b0,
    input  wire               m_ready,
    output reg  [47:0]        m_slot = 48'd0,
    output reg  signed [15:0] m_resid_ps = 16'sd0,
    output reg                m_noise = 1'b0,
    output reg                m_locked = 1'b0,
    output reg  signed [24:0] m_e24 = 25'sd0,
    output reg  [47:0]        m_coarse = 48'd0,
    output reg  [TAG_W-1:0]   m_tag = {TAG_W{1'b0}},
    // configuration registers (see nco_* modules; defaults in README)
    input  wire [3:0]         cfg_kp_acq,
    input  wire [3:0]         cfg_kp_trk,
    input  wire [3:0]         cfg_gear_log2,
    input  wire               cfg_gate_en,
    input  wire [15:0]        cfg_gate_thr,
    input  wire [3:0]         cfg_mse_shift,
    input  wire [3:0]         cfg_mse_shift_trk,
    input  wire [31:0]        cfg_lock_thr,
    input  wire [31:0]        cfg_unlock_thr,
    input  wire [7:0]         cfg_rej_max,
    input  wire [4:0]         cfg_to_shift,
    input  wire               cfg_tau_ovr_en,
    input  wire [4:0]         cfg_tau_ovr,
    input  wire [15:0]        cfg_slip_thr,
    input  wire               cfg_slip_unlock,
    // external correction port (default unused: tie ext_valid low)
    input  wire               ext_valid,
    input  wire signed [47:0] ext_dphase,
    input  wire signed [47:0] ext_dfreq,
    // telemetry
    output wire [1:0]         tlm_state,
    output wire               tlm_locked,
    output wire [3:0]         tlm_kp,
    output wire [4:0]         tlm_L,
    output wire signed [47:0] tlm_fps,
    output wire [87:0]        tlm_phase,
    output wire [31:0]        tlm_mse,
    output wire signed [24:0] tlm_sgn,
    output wire [15:0]        unlock_cnt,
    output wire [15:0]        abort_cnt,
    output wire [15:0]        suspect_cnt,
    output wire [15:0]        noise_cnt,
    output wire [15:0]        late_cnt,
    output wire [15:0]        hit_cnt,
    output reg  [15:0]        out_drop_cnt = 16'd0,
    output reg  [47:0]        t_nco = 48'd0
);
    // ---------------- reset, local to this block (high fanout) ----------------
    // Stretched past the longest pipeline, so anything in flight when reset
    // arrives has drained before the accumulators start again. Then one register
    // per sub-block, in a chain: the extra cycles of reset don't matter, the blocks
    // are idle when it lifts, and a chain cannot be merged back into one net.
    reg [7:0] rst_sr = 8'hFF;
    reg       rst_r = 1'b1;
    (* shreg_extract = "no", max_fanout = 64 *)
    reg       rst_ali = 1'b1, rst_pipe = 1'b1, rst_lf = 1'b1, rst_ld = 1'b1, rst_core = 1'b1, rst_out = 1'b1;
    always @(posedge clk) begin
        rst_sr   <= {rst_sr[6:0], rst};
        rst_r    <= rst | (|rst_sr);
        rst_ali  <= rst_r;
        rst_pipe <= rst_ali;
        rst_lf   <= rst_pipe;
        rst_ld   <= rst_lf;
        rst_core <= rst_ld;
        rst_out  <= rst_core;
    end

    // ---------------- time: t_lead = t_nco + 4, in two cycles ----------------
    // t_nco[x] = time_now[x-1] - D, as always; t_lead is simply that, 4 cycles early.
    localparam [47:0] LEAD_OFS = D - 5;
    reg [24:0] tl_lo = 25'd0;
    reg [23:0] tl_th = 24'd0;
    reg [47:0] t_lead = 48'd0, t_d1 = 48'd0, t_d2 = 48'd0, t_d3 = 48'd0;
    always @(posedge clk) begin
        tl_lo  <= {1'b0, time_now[23:0]} - {1'b0, LEAD_OFS[23:0]};
        tl_th  <= time_now[47:24];
        t_lead <= {tl_th - LEAD_OFS[47:24] - tl_lo[24], tl_lo[23:0]};
        t_d1   <= t_lead;
        t_d2   <= t_d1;
        t_d3   <= t_d2;
        t_nco  <= t_d3;
    end

    // ---------------- align ----------------
    wire              pop, pop_stale;
    wire [47:0]       pop_c;
    wire [17:0]       pop_f;
    wire [TAG_W-1:0]  pop_t;

    nco_align #(.COARSE_W(48), .FINE_W(16), .TAG_W(TAG_W), .AW(FIFO_AW),
                .J_LOOP(J_LOOP), .J_MAX(J_MAX), .W_STALE(W_STALE), .POP_GAP(POP_GAP), .T_CLK_PS(T_CLK_PS)) u_align (
        .clk(clk), .rst(rst_ali), .t_lead(t_lead),
        .s_valid(s_valid), .s_ready(s_ready), .s_coarse(s_coarse), .s_fine(s_fine), .s_tag(s_tag),
        .pop(pop), .pop_stale(pop_stale), .pop_coarse(pop_c), .pop_fine(pop_f), .pop_tag(pop_t),
        .late(), .late_cnt(late_cnt));

    // ---------------- hit position in the pipeline: st[k] = cycle n+k ----------------
    // sk[k]: that hit is STALE (released J_LOOP < j <= J_MAX late, a coincidence):
    // it goes through the phase detector and out, but the loop ignores it.
    reg [17:1] st = 17'd0;
    reg [13:1] sk = 13'd0;
    always @(posedge clk) begin
        st <= rst_pipe ? 17'd0 : {st[16:1], pop};
        sk <= {sk[12:1], pop && pop_stale};
    end
    wire e_out    = st[7];                // every hit: fields for the output
    wire f_valid  = st[13];
    wire e_valid  = st[7]  && !sk[7];     // loop updates: fresh hits only
    wire l_commit = st[11] && !sk[11];
    wire l_clast  = st[12] && !sk[12];
    wire l_f      = st[13] && !sk[13];
    // no timeout decision from a hit's E stage (n+7) until F + 3 (n+16); nco_lock_det
    // uses this two cycles before the decision
    reg  busy_e = 1'b0;
    always @(posedge clk) busy_e <= |st[13:4];

    // ---------------- NCO ----------------
    wire [87:0]        phase;
    wire signed [47:0] fps;
    wire               corr_valid;
    wire signed [47:0] corr_dphi, corr_df;

    nco_core #(.FPS_NOM(FPS_NOM), .T_CLK_PS(T_CLK_PS)) u_core (
        .clk(clk), .rst(rst_core),
        .loop_valid(corr_valid), .loop_dphi(corr_dphi), .loop_df(corr_df),
        .ext_valid(ext_valid), .ext_dphase(ext_dphase), .ext_dfreq(ext_dfreq),
        .phase(phase), .fps(fps), .fclk());

    // ---------------- phase detector ----------------
    wire [47:0]        a_c;
    wire [TAG_W-1:0]   a_tag;
    wire signed [24:0] e_h, ne_h;
    wire               ge_h;
    wire [31:0]        sq;
    wire [47:0]        slot_h;
    wire signed [15:0] resid_h;

    nco_phase_det #(.COARSE_W(48), .TAG_W(TAG_W), .T_SLOT_PS(T_SLOT_PS)) u_pd (
        .clk(clk), .pop(pop), .pop_coarse(pop_c), .pop_fine(pop_f), .pop_tag(pop_t),
        .phase(phase), .fps(fps), .gate_thr(cfg_gate_thr), .st5(st[5]), .st8(st[8]),
        .a_c(a_c), .a_tag(a_tag), .e_h(e_h), .ne_h(ne_h), .ge_h(ge_h), .sq(sq),
        .slot_h(slot_h), .resid_h(resid_h));

    // ---------------- E stage: gate + loop filter + averages ----------------
    wire [1:0] state;
    wire       noise;
    wire       clr_next, enter_ramp, locked_next;
    wire [3:0] kp;
    wire       gear_done;
    wire [4:0] L;

    nco_gate u_gate (
        .acquired(state != 2'd0), .gate_en(cfg_gate_en), .outside(ge_h), .noise(noise));

    nco_loop_filter #(.COARSE_W(48), .TAU_INIT_LOG2(TAU_INIT_LOG2)) u_lf (
        .clk(clk), .rst(rst_lf),
        .kp_acq(cfg_kp_acq), .kp_trk(cfg_kp_trk), .gear_log2(cfg_gear_log2),
        .tau_ovr_en(cfg_tau_ovr_en), .tau_ovr(cfg_tau_ovr),
        .e_valid(e_valid), .ne(ne_h), .noise(noise), .acquired(state != 2'd0),
        .coarse(a_c), .clr_next(clr_next), .enter_ramp(enter_ramp),
        .corr_valid(corr_valid), .corr_dphi(corr_dphi), .corr_df(corr_df),
        .kp(kp), .gear_done(gear_done), .L(L));

    // Per-hit fields held for the F stage (next hit's a_c arrives at its n+1, >= n+9)
    reg [47:0]      e_c = 48'd0;
    reg [TAG_W-1:0] e_tag = {TAG_W{1'b0}};
    reg             f_noise = 1'b0;
    always @(posedge clk)
        if (e_out) begin
            e_c     <= a_c;
            e_tag   <= a_tag;
            f_noise <= noise;
        end

    nco_lock_det #(.COARSE_W(48)) u_ld (
        .clk(clk), .rst(rst_ld), .t_lead1(t_d3),
        .mse_shift(cfg_mse_shift), .mse_shift_trk(cfg_mse_shift_trk),
        .lock_thr(cfg_lock_thr), .unlock_thr(cfg_unlock_thr), .rej_max(cfg_rej_max),
        .to_shift(cfg_to_shift), .slip_thr(cfg_slip_thr), .slip_unlock(cfg_slip_unlock),
        .e_valid(e_valid), .e24(e_h), .sq(sq), .noise(noise),
        .st_commit(l_commit), .busy_e(busy_e), .st_clast(l_clast), .coarse(e_c),
        .f_valid(l_f), .gear_done(gear_done), .L(L),
        .clr_next(clr_next), .enter_ramp(enter_ramp), .locked_next(locked_next),
        .state(state), .mse(tlm_mse), .sgn(tlm_sgn),
        .unlock_cnt(unlock_cnt), .abort_cnt(abort_cnt), .suspect_cnt(suspect_cnt),
        .noise_cnt(noise_cnt), .hit_cnt(hit_cnt));

    // ---------------- F stage: output register ----------------
    always @(posedge clk) begin
        if (rst_out) begin
            m_valid <= 1'b0;
            out_drop_cnt <= 16'd0;
        end else begin
            if (m_valid && m_ready) m_valid <= 1'b0;
            if (f_valid) begin
                if (!m_valid || m_ready) begin
                    m_valid    <= 1'b1;
                    m_slot     <= slot_h;
                    m_resid_ps <= resid_h;
                    m_noise    <= f_noise;
                    m_locked   <= locked_next;
                    m_e24      <= e_h;
                    m_coarse   <= e_c;
                    m_tag      <= e_tag;
                end else if (out_drop_cnt != 16'hFFFF) begin
                    out_drop_cnt <= out_drop_cnt + 1'b1;
                end
            end
        end
    end

    assign tlm_state  = state;
    assign tlm_locked = (state == 2'd2);
    assign tlm_kp     = kp;
    assign tlm_L      = L;
    assign tlm_fps    = fps;
    assign tlm_phase  = phase;
endmodule

`default_nettype wire
