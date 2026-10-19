// Simulation-only wrapper for the block 1 -> block 2 integration test:
// tdc_top (behavioural carry chain, 48-bit coarse) feeding nco_top through the
// valid/ready timestamp interface; the NCO time base is the TDC's coarse counter.
`timescale 1ns/1fs
`default_nettype none
module tdc_nco_tb_top #(
    parameter integer F_SRC_HZ  = 100000000,
    parameter integer T_SLOT_PS = 10000,
    parameter integer SEED      = 1
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        hit,
    // NCO results
    output wire        m_valid,
    output wire [47:0] m_slot,
    output wire [15:0] m_resid_ps,
    output wire        m_noise,
    output wire        m_locked,
    output wire [24:0] m_e24,
    output wire [47:0] m_coarse,
    // configuration
    input  wire [3:0]  cfg_kp_acq, cfg_kp_trk, cfg_gear_log2, cfg_mse_shift, cfg_mse_shift_trk,
    input  wire        cfg_gate_en, cfg_tau_ovr_en, cfg_slip_unlock,
    input  wire [15:0] cfg_gate_thr, cfg_slip_thr,
    input  wire [31:0] cfg_lock_thr, cfg_unlock_thr,
    input  wire [7:0]  cfg_rej_max,
    input  wire [4:0]  cfg_to_shift, cfg_tau_ovr,
    // visibility for the testbench
    output wire        tdc_valid,
    output wire        nco_ready,
    output wire [47:0] tdc_coarse,
    output wire [15:0] tdc_fine_ps,
    output wire [47:0] coarse_now,
    output wire [47:0] t_nco,
    output wire [87:0] tlm_phase,
    output wire [47:0] tlm_fps,
    output wire [15:0] tdc_drop_cnt, tdc_short_cnt, tdc_sat_cnt, tdc_rej_cnt
);
    localparam [127:0] FPS_X = ((128'd1 << 56) * F_SRC_HZ + 128'd500000000000) / 128'd1000000000000;

    wire [56:0] m_raw;
    wire [63:0] m_cal;
    tdc_top #(.COARSE_W(48), .SEED(SEED)) u_tdc (
        .clk(clk), .rst(rst), .hit(hit),
        .m_valid(tdc_valid), .m_ready(nco_ready), .m_raw(m_raw), .m_cal(m_cal),
        .overflow(), .dead(), .drop_cnt(tdc_drop_cnt), .short_cnt(tdc_short_cnt),
        .sat_cnt(tdc_sat_cnt), .rej_cnt(tdc_rej_cnt), .coarse_now(coarse_now),
        .cal_we(1'b0), .cal_addr(9'd0), .cal_wdata(16'd0));
    assign tdc_coarse  = m_cal[63:16];
    assign tdc_fine_ps = m_cal[15:0];

    nco_top #(.FPS_NOM(FPS_X[47:0]), .T_SLOT_PS(T_SLOT_PS)) u_nco (
        .clk(clk), .rst(rst), .time_now(coarse_now),
        .s_valid(tdc_valid), .s_ready(nco_ready), .s_coarse(tdc_coarse), .s_fine(tdc_fine_ps),
        .s_tag(4'd0),
        .m_valid(m_valid), .m_ready(1'b1), .m_slot(m_slot), .m_resid_ps(m_resid_ps),
        .m_noise(m_noise), .m_locked(m_locked), .m_e24(m_e24), .m_coarse(m_coarse), .m_tag(),
        .cfg_kp_acq(cfg_kp_acq), .cfg_kp_trk(cfg_kp_trk), .cfg_gear_log2(cfg_gear_log2),
        .cfg_gate_en(cfg_gate_en), .cfg_gate_thr(cfg_gate_thr), .cfg_mse_shift(cfg_mse_shift),
        .cfg_mse_shift_trk(cfg_mse_shift_trk), .cfg_lock_thr(cfg_lock_thr),
        .cfg_unlock_thr(cfg_unlock_thr), .cfg_rej_max(cfg_rej_max), .cfg_to_shift(cfg_to_shift),
        .cfg_tau_ovr_en(cfg_tau_ovr_en), .cfg_tau_ovr(cfg_tau_ovr), .cfg_slip_thr(cfg_slip_thr),
        .cfg_slip_unlock(cfg_slip_unlock),
        .ext_valid(1'b0), .ext_dphase(48'sd0), .ext_dfreq(48'sd0),
        .tlm_state(), .tlm_locked(), .tlm_kp(), .tlm_L(), .tlm_fps(tlm_fps), .tlm_phase(tlm_phase),
        .tlm_mse(), .tlm_sgn(), .unlock_cnt(), .abort_cnt(), .suspect_cnt(), .noise_cnt(),
        .late_cnt(), .hit_cnt(), .out_drop_cnt(), .t_nco(t_nco));
endmodule
`default_nettype wire
