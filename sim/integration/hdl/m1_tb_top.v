// Simulation-only wrapper for the M1 rehearsal: the real board top
// (m1_arty_top) plus what is outside the FPGA on lab day:
//   * the waveform generator: an independent ~100 MHz clock, offset PPM and
//     drifting DRIFT_PPM_S, into jb[0]
//   * the jumper wire from JA pin 2 (emu_out) to JA pin 1 (ja[0]), 1.5 ns
`timescale 1ns/1fs
`default_nettype none
module m1_tb_top #(
    parameter real    PPM           = 20.0,
    parameter real    DRIFT_PPM_S   = 0.0,
    parameter integer STATUS_PERIOD = 50000,
    parameter integer SEED          = 1
)(
    input  wire       ck_rst,
    input  wire [3:0] sw,
    output wire [3:0] led,
    output wire       uart_rxd_out
);
    // waveform generator: f(t) = 100 MHz * (1 + (PPM + DRIFT_PPM_S * t) * 1e-6)
    reg  clk_src = 1'b0;
    real half_ns;
    initial begin
        #(3.3);                                   // arbitrary phase vs the board clock
        forever begin
            half_ns = 5.0 / (1.0 + (PPM + DRIFT_PPM_S * $realtime * 1e-9) * 1e-6);
            #(half_ns) clk_src = ~clk_src;
        end
    end

    wire emu_out, ja_in;
    assign #(1.5) ja_in = emu_out;                // the jumper wire

    m1_arty_top #(.STATUS_PERIOD(STATUS_PERIOD), .SEED(SEED)) u_m1 (
        .CLK100MHZ(1'b0), .ck_rst(ck_rst), .sw(sw), .jb(clk_src), .ja(ja_in),
        .emu_out(emu_out), .led(led), .uart_rxd_out(uart_rxd_out));
endmodule
`default_nettype wire
