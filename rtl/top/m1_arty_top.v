// =============================================================================
// m1_arty_top.v  --  M1 demo: lock to an independent 100 MHz source (Arty A7-100T)
// -----------------------------------------------------------------------------
//
//  waveform generator ~100 MHz --> jb[0] (clock-capable pin) --> sat_emulator
//                                                                  | pulse (15 ns)
//                                                                  v
//                                                      emu_out (Pmod JA pin 2)
//                                                                  | JUMPER wire
//                                                                  v
//  CLK100MHZ -> MMCM -> 250 MHz -----> ja[0] (Pmod JA pin 1) -> tdc_top -> nco_top
//                                                                            |
//                                       uart_rxd_out <- STATUS + TELEMETRY frames
//
// The emulator runs entirely on the generator's clock, the TDC and NCO on the
// board oscillator: two independent crystals, so the NCO really has to find
// the ppm offset between them. The only connection is the jumper wire, like
// the optical link plus detector in the real system.
//
//   sw[1:0] detection probability per slot: 00 = 1e-1, 01 = 1e-2, 10 = 1e-3, 11 = 1e-4
//   sw[2]   1 = drop all pulses (signal gap: the NCO must unlock, then re-lock)
//   sw[3]   1 = also stream raw TDC timestamps (TIMESTAMP frames; lossy at high
//               rates, for debugging only). Telemetry is always on.
//   ck_rst  CPU_RESET (active low): resets the 250 MHz side and the emulator
//           (and the NCO settings back to their defaults)
//   uart_txd_in  PC -> FPGA commands (pin A9): set / read the NCO runtime
//           settings without a rebuild, see nco_cfg_regs.v and scripts/nco_config.py
//   led[0]  heartbeat (MMCM locked, clock running)
//   led[1]  NCO LOCKED
//   led[2]  sticky: an unlock happened (unlock_cnt != 0)
//   led[3]  sticky: TDC saturation (chain coverage failure, see README)
//
// UART (2 Mbaud): STATUS frame (TDC drop/short/sat) + TELEMETRY frame (lock
// state, gear, frequency offset, error averages, counters) every
// STATUS_PERIOD serial clocks (0.34 s); scripts/capture.py --tlm-out decodes them.
//
// CLOCKS: 250 MHz (TDC, NCO, LEDs) and 50 MHz serial (UART receive, settings
// registers, UART framer and transmitter), both from the MMCM. Crossings: raw
// timestamps through an async FIFO, counters + telemetry and the settings as
// coherent snapshots (rtl/top/cdc_snapshot.v).
// See docs/M1_DEMO.md for the procedure and pass criteria.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module m1_arty_top #(
    parameter real    MMCM_MULT     = 10.0,
    parameter real    MMCM_OUT_DIV  = 4.0,
    parameter integer F_SRC_HZ      = 100000000,
    parameter integer T_SLOT_PS     = 10000,
    parameter         LUT_FILE      = "",
    parameter integer MMCM_SER_DIV  = 20,          // serial clock: 1000 MHz VCO / 20 = 50 MHz
    parameter integer CLKS_PER_BIT  = 25,          // 50 MHz / 2 Mbaud
    parameter integer FIFO_AW       = 9,
    parameter integer STATUS_PERIOD = 16777216,    // serial clocks: 0.34 s
    // simulation-only model knobs (behavioural delay line)
    parameter real    TAP_NOM_PS    = 22.0,
    parameter real    JITTER_PS     = 4.0,
    parameter integer SEED          = 1
)(
    input  wire       CLK100MHZ,
    input  wire       ck_rst,
    input  wire [3:0] sw,
    input  wire [0:0] jb,             // emulator clock in (from the waveform generator)
    input  wire [0:0] ja,             // TDC hit in (Pmod JA pin 1)
    output wire       emu_out,        // emulator pulse out (Pmod JA pin 2)
    output wire [3:0] led,
    output wire       uart_rxd_out,
    input  wire       uart_txd_in       // PC -> FPGA (FT2232H TXD), 2 Mbaud commands
);
    localparam integer FINE_W = 9, COARSE_W = 48, PS_W = 16;
    localparam [127:0] FPS_X   = ((128'd1 << 56) * F_SRC_HZ + 128'd500000000000) / 128'd1000000000000;
    localparam [47:0]  FPS_NOM = FPS_X[47:0];

    // ---------------- clocking ----------------
    wire clk, clk_ser, locked, clk_src;
`ifdef SYNTHESIS
    wire clk_fb, clk_mmcm, clk_ser_mmcm;
    MMCME2_BASE #(
        .CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT_F(MMCM_MULT), .DIVCLK_DIVIDE(1),
        .CLKOUT0_DIVIDE_F(MMCM_OUT_DIV), .CLKOUT1_DIVIDE(MMCM_SER_DIV)
    ) u_mmcm (
        .CLKIN1(CLK100MHZ), .CLKFBIN(clk_fb), .CLKFBOUT(clk_fb), .CLKOUT0(clk_mmcm),
        .CLKOUT1(clk_ser_mmcm), .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0),
        .CLKFBOUTB(), .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2(),
        .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6());
    BUFG u_bufg (.I(clk_mmcm), .O(clk));
    BUFG u_bufg_ser (.I(clk_ser_mmcm), .O(clk_ser));
    // emulator clock: dedicated clock input -> global buffer
    BUFG u_bufg_src (.I(jb[0]), .O(clk_src));
`else
    reg clk_sim = 1'b0, clk_ser_sim = 1'b0, locked_sim = 1'b0;
    always #(2.0) clk_sim = ~clk_sim;
    always #(10.0) clk_ser_sim = ~clk_ser_sim;
    initial #(100.0) locked_sim = 1'b1;
    assign clk = clk_sim;
    assign clk_ser = clk_ser_sim;
    assign locked = locked_sim;
    assign clk_src = jb[0];           // the testbench drives an offset/drifting clock
`endif

    // ---------------- serial clock domain (UART, settings): reset ----------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] rst_ser_sync = 3'b111;
    reg rst_ser = 1'b1;
    always @(posedge clk_ser or negedge locked)
        if (!locked) rst_ser_sync <= 3'b111;
        else         rst_ser_sync <= {rst_ser_sync[1:0], ~ck_rst};
    always @(posedge clk_ser) rst_ser <= rst_ser_sync[2];

    // ---------------- reset + switch synchronisers (250 MHz side) ----------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] rst_sync = 3'b111;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sw3_s = 2'b00;
    always @(posedge clk or negedge locked)
        if (!locked) rst_sync <= 3'b111;
        else         rst_sync <= {rst_sync[1:0], ~ck_rst};
    wire [3:0] rst_t;
    rst_tree #(.N(4)) u_rst (.clk(clk), .rst_in(rst_sync[2]), .rst(rst_t));
    wire rst_tdc = rst_t[0], rst_nco = rst_t[1], rst_uart = rst_t[2], rst_led = rst_t[3];
    always @(posedge clk) sw3_s <= {sw3_s[0], sw[3]};

    // ---------------- satellite emulator (its own clock domain) ----------------
    reg [31:0] p_thr;
    always @(*)
        case (sw[1:0])                // P(detect) * 2^32
        2'd0: p_thr = 32'd429496730;  // 1e-1
        2'd1: p_thr = 32'd42949673;   // 1e-2
        2'd2: p_thr = 32'd4294967;    // 1e-3
        default: p_thr = 32'd429497;  // 1e-4
        endcase

    wire        emu_pulse, emu_valid;
    wire [47:0] emu_slot;
    sat_emulator u_emu (
        .clk_src(clk_src), .rst_async(~ck_rst), .p_thr(p_thr), .drop_all(sw[2]),
        .pulse(emu_pulse), .emit_valid(emu_valid), .emit_slot(emu_slot),
        .emit_cnt(), .supp_cnt());
    assign emu_out = emu_pulse;

    // ---------------- TDC ----------------
    wire                       tdc_valid, tdc_ready;
    wire [COARSE_W+FINE_W-1:0] m_raw;
    wire [COARSE_W+PS_W-1:0]   m_cal;
    wire [15:0]                drop_cnt, short_cnt, sat_cnt;
    wire [COARSE_W-1:0]        coarse_now;
    tdc_top #(
        .COARSE_W(COARSE_W), .LUT_FILE(LUT_FILE),
        .TAP_NOM_PS(TAP_NOM_PS), .JITTER_PS(JITTER_PS), .SEED(SEED)
    ) u_tdc (
        .clk(clk), .rst(rst_tdc), .hit(ja[0]),
        .m_valid(tdc_valid), .m_ready(tdc_ready), .m_raw(m_raw), .m_cal(m_cal),
        .overflow(), .dead(), .drop_cnt(drop_cnt), .short_cnt(short_cnt), .sat_cnt(sat_cnt),
        .rej_cnt(), .coarse_now(coarse_now),
        .cal_we(1'b0), .cal_addr({FINE_W{1'b0}}), .cal_wdata({PS_W{1'b0}}));

    // ---------------- PC command interface: NCO runtime settings (serial clock) ----------------
    // The settings cross to the 250 MHz side as one coherent snapshot (cdc_snapshot),
    // a few serial clocks after a write; nco_top reads them on every hit.
    wire        rx_valid;
    wire [7:0]  rx_data;
    wire        rb_valid;
    wire [172:0] rb_data;
    wire [3:0]  s_kp_acq, s_kp_trk, s_gear, s_mse, s_mse_trk;
    wire        s_gate_en, s_tau_en, s_slip_unlock;
    wire [15:0] s_gate_thr, s_slip_thr;
    wire [31:0] s_lock, s_unlock;
    wire [7:0]  s_rej;
    wire [4:0]  s_to, s_tau;
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk_ser), .rst(rst_ser), .rx(uart_txd_in), .valid(rx_valid), .data(rx_data), .frame_err());
    nco_cfg_regs #(.IDLE_TIMEOUT(5000)) u_cfg (          // 100 us at 50 MHz
        .clk(clk_ser), .rst(rst_ser), .rx_valid(rx_valid), .rx_data(rx_data),
        .rb_valid(rb_valid), .rb_data(rb_data),
        .kp_acq(s_kp_acq), .kp_trk(s_kp_trk), .gear_log2(s_gear), .gate_en(s_gate_en),
        .gate_thr(s_gate_thr), .mse_shift(s_mse), .mse_shift_trk(s_mse_trk),
        .lock_thr(s_lock), .unlock_thr(s_unlock), .rej_max(s_rej), .to_shift(s_to),
        .tau_ovr_en(s_tau_en), .tau_ovr(s_tau), .slip_thr(s_slip_thr),
        .slip_unlock(s_slip_unlock), .cmd_cnt(), .err_cnt());

    localparam integer CFG_W = 4*5 + 3 + 16*2 + 32*2 + 8 + 5*2;
    // reset values of nco_cfg_regs, so the NCO sees the defaults from the first cycle
    localparam [CFG_W-1:0] CFG_INIT = {4'd2, 4'd6, 4'd6, 4'd5, 4'd7, 1'b1, 1'b0, 1'b1,
                                       16'h2000, 16'h2000, 32'h0400_0000, 32'h1000_0000,
                                       8'd16, 5'd5, 5'd0};
    wire [3:0]  c_kp_acq, c_kp_trk, c_gear, c_mse, c_mse_trk;
    wire        c_gate_en, c_tau_en, c_slip_unlock;
    wire [15:0] c_gate_thr, c_slip_thr;
    wire [31:0] c_lock, c_unlock;
    wire [7:0]  c_rej;
    wire [4:0]  c_to, c_tau;
    cdc_snapshot #(.W(CFG_W), .INIT(CFG_INIT)) u_cfg_cdc (
        .clk_s(clk_ser),
        .d_s({s_kp_acq, s_kp_trk, s_gear, s_mse, s_mse_trk, s_gate_en, s_tau_en, s_slip_unlock,
              s_gate_thr, s_slip_thr, s_lock, s_unlock, s_rej, s_to, s_tau}),
        .clk_d(clk),
        .q_d({c_kp_acq, c_kp_trk, c_gear, c_mse, c_mse_trk, c_gate_en, c_tau_en, c_slip_unlock,
              c_gate_thr, c_slip_thr, c_lock, c_unlock, c_rej, c_to, c_tau}));

    // ---------------- NCO (block 2), settings from nco_cfg_regs ----------------
    wire               nco_valid, nco_locked, nco_noise;
    wire [47:0]        nco_slot;
    wire [1:0]         st;
    wire [3:0]         kp;
    wire [4:0]         L;
    wire signed [47:0] fps;
    wire [31:0]        mse;
    wire signed [24:0] sgn;
    wire [15:0]        unlock_cnt, abort_cnt, suspect_cnt, noise_cnt, late_cnt, hit_cnt;
    nco_top #(.FPS_NOM(FPS_NOM), .T_SLOT_PS(T_SLOT_PS)) u_nco (
        .clk(clk), .rst(rst_nco), .time_now(coarse_now),
        .s_valid(tdc_valid), .s_ready(tdc_ready), .s_coarse(m_cal[PS_W +: COARSE_W]),
        .s_fine(m_cal[PS_W-1:0]), .s_tag(4'd0),
        .m_valid(nco_valid), .m_ready(1'b1), .m_slot(nco_slot), .m_resid_ps(),
        .m_noise(nco_noise), .m_locked(nco_locked), .m_e24(), .m_coarse(), .m_tag(),
        .cfg_kp_acq(c_kp_acq), .cfg_kp_trk(c_kp_trk), .cfg_gear_log2(c_gear),
        .cfg_gate_en(c_gate_en), .cfg_gate_thr(c_gate_thr), .cfg_mse_shift(c_mse),
        .cfg_mse_shift_trk(c_mse_trk), .cfg_lock_thr(c_lock), .cfg_unlock_thr(c_unlock),
        .cfg_rej_max(c_rej), .cfg_to_shift(c_to), .cfg_tau_ovr_en(c_tau_en), .cfg_tau_ovr(c_tau),
        .cfg_slip_thr(c_slip_thr), .cfg_slip_unlock(c_slip_unlock),
        .ext_valid(1'b0), .ext_dphase(48'sd0), .ext_dfreq(48'sd0),
        .tlm_state(st), .tlm_locked(), .tlm_kp(kp), .tlm_L(L), .tlm_fps(fps), .tlm_phase(),
        .tlm_mse(mse), .tlm_sgn(sgn), .unlock_cnt(unlock_cnt), .abort_cnt(abort_cnt),
        .suspect_cnt(suspect_cnt), .noise_cnt(noise_cnt), .late_cnt(late_cnt),
        .hit_cnt(hit_cnt), .out_drop_cnt(), .t_nco());

    // ---------------- telemetry word (layout: scripts/capture.py) ----------------
    reg  signed [47:0] foff  = 48'sd0;               // registered: telemetry, latency irrelevant
    always @(posedge clk) foff <= fps - $signed(FPS_NOM);
    wire signed [39:0] foff8 = foff >>> 8;            // 2^-48 slot/ps
    wire signed [16:0] sgn8  = sgn >>> 8;             // 2^-16 slot
    wire [172:0] tlm = {st, kp, L, foff8[31:0], mse[31:16], sgn8[15:0],
                        unlock_cnt, abort_cnt, suspect_cnt, noise_cnt, late_cnt, hit_cnt, 2'b00};

    // ---------------- UART (framer and transmitter on the serial clock) ----------------
    wire ts_room;                     // raw timestamps are best effort: skipped when full
    tdc_uart_readout #(
        .COARSE_W(COARSE_W), .FINE_W(FINE_W), .FIFO_AW(FIFO_AW),
        .CLKS_PER_BIT(CLKS_PER_BIT), .STATUS_PERIOD(STATUS_PERIOD)
    ) u_uart (
        .clk(clk), .rst(rst_uart), .clk_ser(clk_ser), .rst_ser(rst_ser),
        .s_valid(tdc_valid & tdc_ready & sw3_s[1] & ts_room), .s_ready(ts_room),
        .s_coarse(m_raw[FINE_W +: COARSE_W]), .s_fine(m_raw[FINE_W-1:0]),
        .mode_ts(1'b1), .drop_cnt(drop_cnt), .short_cnt(short_cnt), .sat_cnt(sat_cnt),
        .tlm_en(1'b1), .tlm_data(tlm), .rb_valid(rb_valid), .rb_data(rb_data),
        .tx(uart_rxd_out));

    // ---------------- LEDs ----------------
    reg unlock_seen = 1'b0, sat_seen = 1'b0;
    always @(posedge clk)
        if (rst_led) begin
            unlock_seen <= 1'b0; sat_seen <= 1'b0;
        end else begin
            unlock_seen <= unlock_seen | (unlock_cnt != 16'd0);
            sat_seen    <= sat_seen    | (sat_cnt != 16'd0);
        end
    assign led = {sat_seen, unlock_seen, (st == 2'd2), coarse_now[27]};
endmodule

`default_nettype wire
