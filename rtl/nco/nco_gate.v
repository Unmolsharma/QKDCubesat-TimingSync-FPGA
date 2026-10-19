// =============================================================================
// nco_gate.v  --  phase-window noise gate (combinational, E stage)
// -----------------------------------------------------------------------------
// Once the loop has acquired phase (state RAMP or LOCK), a hit whose phase
// error is outside +/-g is flagged as noise: it is still output (with
// noise_flag set, so the QKD sifting can drop it) but it does NOT update the
// loop. During acquisition (ACQ) the gate is open: nothing is known about
// the phase yet, so every hit must be allowed to steer.
//
// THE 2g CEILING. Dark counts and background photons are uniform in phase,
// so a fraction 2g of them falls inside the window whatever happens. No
// phase gate can reject more than 1 - 2g of uniform noise. Default
// g = 1/8 slot: 75 % of dark counts rejected, while 500 ps RMS jitter at
// 100 MHz (0.05 slot) keeps 98.8 % of the real photons. g is a runtime
// register (gate_thr, 2^-16 slot): narrow it if the jitter is low and the
// noise high, but then the loop has less margin before it starts rejecting
// its own signal during a disturbance.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_gate (
    input  wire        acquired,     // state != ACQ
    input  wire        gate_en,
    input  wire [15:0] gate_thr,     // 2^-16 slot
    input  wire [23:0] abs_e,        // |e24|, 2^-24 slot
    output wire        noise
);
    assign noise = acquired & gate_en & (abs_e >= {gate_thr, 8'd0});
endmodule

`default_nettype wire
