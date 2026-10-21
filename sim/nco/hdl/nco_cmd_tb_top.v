// Simulation-only wrapper for the UART command test: the real command path
// (uart_rx -> nco_cfg_regs -> readback frames on tdc_uart_readout) driving the
// configuration inputs of nco_top, plus the free-running time base. Timestamps
// are driven directly (no TDC) so the test is fast.
`timescale 1ns/1fs
`default_nettype none
module nco_cmd_tb_top #(
    parameter integer F_SRC_HZ  = 100000000,
    parameter integer T_SLOT_PS = 10000
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        uart_rx_line,      // PC -> board
    output wire        uart_tx_line,      // board -> PC
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
    input  wire        ext_valid,
    input  wire [47:0] ext_dphase, ext_dfreq,
    output reg  [47:0] time_now = 48'd0,
    output wire [47:0] t_nco,
    output wire [1:0]  tlm_state,
    output wire [47:0] tlm_fps,
    output wire [87:0] tlm_phase,
    output wire [15:0] unlock_cnt, abort_cnt, suspect_cnt, noise_cnt, late_cnt, hit_cnt, out_drop_cnt
);
    localparam [127:0] FPS_X = ((128'd1 << 56) * F_SRC_HZ + 128'd500000000000) / 128'd1000000000000;
    always @(posedge clk) time_now <= rst ? 48'd0 : time_now + 1'b1;

    wire        rx_valid, rb_valid;
    wire [7:0]  rx_data;
    wire [172:0] rb_data;
    wire [3:0]  c_kp_acq, c_kp_trk, c_gear, c_mse, c_mse_trk;
    wire        c_gate_en, c_tau_en, c_slip_unlock;
    wire [15:0] c_gate_thr, c_slip_thr;
    wire [31:0] c_lock, c_unlock;
    wire [7:0]  c_rej;
    wire [4:0]  c_to, c_tau;

    uart_rx #(.CLKS_PER_BIT(125)) u_rx (
        .clk(clk), .rst(rst), .rx(uart_rx_line), .valid(rx_valid), .data(rx_data), .frame_err());
    nco_cfg_regs u_cfg (
        .clk(clk), .rst(rst), .rx_valid(rx_valid), .rx_data(rx_data),
        .rb_valid(rb_valid), .rb_data(rb_data),
        .kp_acq(c_kp_acq), .kp_trk(c_kp_trk), .gear_log2(c_gear), .gate_en(c_gate_en),
        .gate_thr(c_gate_thr), .mse_shift(c_mse), .mse_shift_trk(c_mse_trk),
        .lock_thr(c_lock), .unlock_thr(c_unlock), .rej_max(c_rej), .to_shift(c_to),
        .tau_ovr_en(c_tau_en), .tau_ovr(c_tau), .slip_thr(c_slip_thr),
        .slip_unlock(c_slip_unlock), .cmd_cnt(), .err_cnt());
    tdc_uart_readout #(.COARSE_W(48), .FINE_W(9), .FIFO_AW(3), .CLKS_PER_BIT(125),
                       .STATUS_PERIOD(32'h7FFF_FFFF)) u_tx (
        .clk(clk), .rst(rst), .s_valid(1'b0), .s_ready(), .s_coarse(48'd0), .s_fine(9'd0),
        .mode_ts(1'b0), .drop_cnt(16'd0), .short_cnt(16'd0), .sat_cnt(16'd0),
        .tlm_en(1'b0), .tlm_data(173'd0), .rb_valid(rb_valid), .rb_data(rb_data),
        .tx(uart_tx_line));

    nco_top #(.FPS_NOM(FPS_X[47:0]), .T_SLOT_PS(T_SLOT_PS)) u_nco (
        .clk(clk), .rst(rst), .time_now(time_now),
        .s_valid(s_valid), .s_ready(s_ready), .s_coarse(s_coarse), .s_fine(s_fine), .s_tag(s_tag),
        .m_valid(m_valid), .m_ready(m_ready), .m_slot(m_slot), .m_resid_ps(m_resid_ps),
        .m_noise(m_noise), .m_locked(m_locked), .m_e24(m_e24), .m_coarse(m_coarse), .m_tag(m_tag),
        .cfg_kp_acq(c_kp_acq), .cfg_kp_trk(c_kp_trk), .cfg_gear_log2(c_gear),
        .cfg_gate_en(c_gate_en), .cfg_gate_thr(c_gate_thr), .cfg_mse_shift(c_mse),
        .cfg_mse_shift_trk(c_mse_trk), .cfg_lock_thr(c_lock), .cfg_unlock_thr(c_unlock),
        .cfg_rej_max(c_rej), .cfg_to_shift(c_to), .cfg_tau_ovr_en(c_tau_en), .cfg_tau_ovr(c_tau),
        .cfg_slip_thr(c_slip_thr), .cfg_slip_unlock(c_slip_unlock),
        .ext_valid(ext_valid), .ext_dphase(ext_dphase), .ext_dfreq(ext_dfreq),
        .tlm_state(tlm_state), .tlm_locked(), .tlm_kp(), .tlm_L(), .tlm_fps(tlm_fps),
        .tlm_phase(tlm_phase), .tlm_mse(), .tlm_sgn(),
        .unlock_cnt(unlock_cnt), .abort_cnt(abort_cnt), .suspect_cnt(suspect_cnt),
        .noise_cnt(noise_cnt), .late_cnt(late_cnt), .hit_cnt(hit_cnt), .out_drop_cnt(out_drop_cnt),
        .t_nco(t_nco));
endmodule
`default_nettype wire
