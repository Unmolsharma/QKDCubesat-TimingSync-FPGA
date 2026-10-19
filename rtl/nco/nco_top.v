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
// PIPELINE (cycles after the pop cycle n): 1-4 phase detector, 4 = E (gate,
// loop filter, averages), 5 = F (state machine, output register, loop
// corrections strobed into nco_core), 6 = corrections visible in phase/fps.
// nco_align releases at most one hit every 2 cycles, so the E stage of a hit
// always sees the F-stage decisions of the previous one.
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
    parameter integer D             = 16,
    parameter integer J_MAX         = 12,
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
    always @(posedge clk) t_nco <= time_now - D;

    // ---------------- align ----------------
    wire              pop;
    wire [47:0]       pop_c;
    wire [15:0]       pop_f;
    wire [TAG_W-1:0]  pop_t;

    nco_align #(.COARSE_W(48), .FINE_W(16), .TAG_W(TAG_W), .AW(FIFO_AW),
                .J_MAX(J_MAX), .T_CLK_PS(T_CLK_PS)) u_align (
        .clk(clk), .rst(rst), .t_nco(t_nco),
        .s_valid(s_valid), .s_ready(s_ready), .s_coarse(s_coarse), .s_fine(s_fine), .s_tag(s_tag),
        .pop(pop), .pop_coarse(pop_c), .pop_fine(pop_f), .pop_tag(pop_t),
        .late(), .late_cnt(late_cnt));

    // ---------------- NCO ----------------
    wire [87:0]        phase;
    wire signed [47:0] fps;
    wire               corr_valid;
    wire signed [47:0] corr_dphi, corr_df;

    nco_core #(.FPS_NOM(FPS_NOM), .T_CLK_PS(T_CLK_PS)) u_core (
        .clk(clk), .rst(rst),
        .loop_valid(corr_valid), .loop_dphi(corr_dphi), .loop_df(corr_df),
        .ext_valid(ext_valid), .ext_dphase(ext_dphase), .ext_dfreq(ext_dfreq),
        .phase(phase), .fps(fps), .fclk());

    // ---------------- phase detector (stages A-D) ----------------
    wire               d_valid;
    wire [47:0]        d_slot;
    wire signed [24:0] d_e24;
    wire [23:0]        d_abs;
    wire [31:0]        d_sq;
    wire signed [15:0] d_resid;
    wire [47:0]        d_coarse;
    wire [TAG_W-1:0]   d_tag;

    nco_phase_det #(.COARSE_W(48), .TAG_W(TAG_W), .T_SLOT_PS(T_SLOT_PS)) u_pd (
        .clk(clk), .rst(rst),
        .pop(pop), .pop_coarse(pop_c), .pop_fine(pop_f), .pop_tag(pop_t),
        .phase(phase), .fps(fps),
        .d_valid(d_valid), .d_slot(d_slot), .d_e24(d_e24), .d_abs(d_abs), .d_sq(d_sq),
        .d_resid(d_resid), .d_coarse(d_coarse), .d_tag(d_tag));

    // ---------------- E stage: gate + loop filter + averages ----------------
    wire [1:0] state;
    wire       noise;
    wire       to_acq, enter_ramp, locked_next;
    wire [3:0] kp;
    wire       gear_done;
    wire [4:0] L;

    nco_gate u_gate (
        .acquired(state != 2'd0), .gate_en(cfg_gate_en), .gate_thr(cfg_gate_thr),
        .abs_e(d_abs), .noise(noise));

    nco_loop_filter #(.COARSE_W(48), .TAU_INIT_LOG2(TAU_INIT_LOG2)) u_lf (
        .clk(clk), .rst(rst),
        .kp_acq(cfg_kp_acq), .kp_trk(cfg_kp_trk), .gear_log2(cfg_gear_log2),
        .tau_ovr_en(cfg_tau_ovr_en), .tau_ovr(cfg_tau_ovr),
        .e_valid(d_valid), .e24(d_e24), .noise(noise), .acquired(state != 2'd0),
        .coarse(d_coarse), .to_acq(to_acq), .enter_ramp(enter_ramp),
        .corr_valid(corr_valid), .corr_dphi(corr_dphi), .corr_df(corr_df),
        .kp(kp), .gear_done(gear_done), .L(L));

    // E -> F pipeline register for the per-hit output fields
    reg               f_valid = 1'b0;
    reg [47:0]        f_slot = 48'd0;
    reg signed [15:0] f_resid = 16'sd0;
    reg               f_noise = 1'b0;
    reg signed [24:0] f_e24 = 25'sd0;
    reg [47:0]        f_coarse = 48'd0;
    reg [TAG_W-1:0]   f_tag = {TAG_W{1'b0}};
    always @(posedge clk) begin
        f_valid  <= d_valid & ~rst;
        f_slot   <= d_slot;
        f_resid  <= d_resid;
        f_noise  <= noise;
        f_e24    <= d_e24;
        f_coarse <= d_coarse;
        f_tag    <= d_tag;
    end

    nco_lock_det #(.COARSE_W(48)) u_ld (
        .clk(clk), .rst(rst), .t_nco(t_nco),
        .mse_shift(cfg_mse_shift), .mse_shift_trk(cfg_mse_shift_trk),
        .lock_thr(cfg_lock_thr), .unlock_thr(cfg_unlock_thr), .rej_max(cfg_rej_max),
        .to_shift(cfg_to_shift), .slip_thr(cfg_slip_thr), .slip_unlock(cfg_slip_unlock),
        .e_valid(d_valid), .e24(d_e24), .sq(d_sq), .noise(noise), .coarse(d_coarse),
        .f_valid(f_valid), .gear_done(gear_done), .L(L),
        .to_acq(to_acq), .enter_ramp(enter_ramp), .locked_next(locked_next),
        .state(state), .mse(tlm_mse), .sgn(tlm_sgn),
        .unlock_cnt(unlock_cnt), .abort_cnt(abort_cnt), .suspect_cnt(suspect_cnt),
        .noise_cnt(noise_cnt), .hit_cnt(hit_cnt));

    // ---------------- F stage: output register ----------------
    always @(posedge clk) begin
        if (rst) begin
            m_valid <= 1'b0;
            out_drop_cnt <= 16'd0;
        end else begin
            if (m_valid && m_ready) m_valid <= 1'b0;
            if (f_valid) begin
                if (!m_valid || m_ready) begin
                    m_valid    <= 1'b1;
                    m_slot     <= f_slot;
                    m_resid_ps <= f_resid;
                    m_noise    <= f_noise;
                    m_locked   <= locked_next;
                    m_e24      <= f_e24;
                    m_coarse   <= f_coarse;
                    m_tag      <= f_tag;
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
