// Simulation-only wrapper for the block 2 unit tests: a free-running 48-bit
// time base (what tdc_coarse_counter provides in the real design) + nco_top.
// Every other port is driven by the cocotb testbench.
`timescale 1ns/1fs
`default_nettype none
module nco_tb_top #(
    parameter integer F_SRC_HZ  = 100000000,
    parameter integer T_SLOT_PS = 10000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        s_valid,
    output wire        s_ready,
    input  wire [47:0] s_coarse,
    input  wire [15:0] s_fine,
    input  wire [3:0]  s_tag,
    output wire        m_valid,
    input  wire        m_ready,
    output wire [47:0] m_slot,
    output wire [15:0] m_resid_ps,
    output wire        m_noise,
    output wire        m_locked,
    output wire [24:0] m_e24,
    output wire [47:0] m_coarse,
    output wire [3:0]  m_tag,
    input  wire [3:0]  cfg_kp_acq, cfg_kp_trk, cfg_gear_log2, cfg_mse_shift, cfg_mse_shift_trk,
    input  wire        cfg_gate_en, cfg_tau_ovr_en, cfg_slip_unlock,
    input  wire [15:0] cfg_gate_thr, cfg_slip_thr,
    input  wire [31:0] cfg_lock_thr, cfg_unlock_thr,
    input  wire [7:0]  cfg_rej_max,
    input  wire [4:0]  cfg_to_shift, cfg_tau_ovr,
    input  wire        ext_valid,
    input  wire [47:0] ext_dphase, ext_dfreq,
    output reg  [47:0] time_now = 48'd0,
    output wire [47:0] t_nco,
    output wire [1:0]  tlm_state,
    output wire [3:0]  tlm_kp,
    output wire [47:0] tlm_fps,
    output wire [87:0] tlm_phase,
    output wire [31:0] tlm_mse,
    output wire [15:0] unlock_cnt, abort_cnt, suspect_cnt, noise_cnt, late_cnt, hit_cnt, out_drop_cnt
);
    // round(F_SRC_HZ * 1e-12 * 2^56), exact integer arithmetic
    localparam [127:0] FPS_X = ((128'd1 << 56) * F_SRC_HZ + 128'd500000000000) / 128'd1000000000000;
    localparam [47:0]  FPS_NOM = FPS_X[47:0];

    always @(posedge clk) time_now <= rst ? 48'd0 : time_now + 1'b1;

    nco_top #(.FPS_NOM(FPS_NOM), .T_SLOT_PS(T_SLOT_PS)) u_nco (
        .clk(clk), .rst(rst), .time_now(time_now),
        .s_valid(s_valid), .s_ready(s_ready), .s_coarse(s_coarse), .s_fine(s_fine), .s_tag(s_tag),
        .m_valid(m_valid), .m_ready(m_ready), .m_slot(m_slot), .m_resid_ps(m_resid_ps),
        .m_noise(m_noise), .m_locked(m_locked), .m_e24(m_e24), .m_coarse(m_coarse), .m_tag(m_tag),
        .cfg_kp_acq(cfg_kp_acq), .cfg_kp_trk(cfg_kp_trk), .cfg_gear_log2(cfg_gear_log2),
        .cfg_gate_en(cfg_gate_en), .cfg_gate_thr(cfg_gate_thr), .cfg_mse_shift(cfg_mse_shift),
        .cfg_mse_shift_trk(cfg_mse_shift_trk), .cfg_lock_thr(cfg_lock_thr),
        .cfg_unlock_thr(cfg_unlock_thr), .cfg_rej_max(cfg_rej_max), .cfg_to_shift(cfg_to_shift),
        .cfg_tau_ovr_en(cfg_tau_ovr_en), .cfg_tau_ovr(cfg_tau_ovr), .cfg_slip_thr(cfg_slip_thr),
        .cfg_slip_unlock(cfg_slip_unlock),
        .ext_valid(ext_valid), .ext_dphase(ext_dphase), .ext_dfreq(ext_dfreq),
        .tlm_state(tlm_state), .tlm_locked(), .tlm_kp(tlm_kp), .tlm_L(), .tlm_fps(tlm_fps),
        .tlm_phase(tlm_phase), .tlm_mse(tlm_mse), .tlm_sgn(),
        .unlock_cnt(unlock_cnt), .abort_cnt(abort_cnt), .suspect_cnt(suspect_cnt),
        .noise_cnt(noise_cnt), .late_cnt(late_cnt), .hit_cnt(hit_cnt), .out_drop_cnt(out_drop_cnt),
        .t_nco(t_nco));
endmodule
`default_nettype wire
