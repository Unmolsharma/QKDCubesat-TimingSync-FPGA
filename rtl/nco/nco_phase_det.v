// =============================================================================
// nco_phase_det.v  --  NCO phase at the photon, slot number, phase error
// -----------------------------------------------------------------------------
// For a hit released in cycle n (t_nco = n, see nco_align) with fine' ps
// before edge n:
//
//   A (edge ending cycle n) register phase[n], fps[n] and fine'
//   B  prod   = fine' * (fps >>> 22)               ONE multiply (25 x 17 bit,
//                                                   one DSP48E1), 2^-34 slot
//   C  phi    = phase[n] - (prod << 6)             NCO phase at the photon,
//                                                   2^-40 slot, mod 2^88
//   D  phi24  = phi >> 16   (floor, 2^-24 slot)
//      slot   = round(phi24)  = (phi24 + 1/2) >> 24 (48 bit)
//      e24    = phi24 - slot  in [-0.5, 0.5)       the phase error
//      e2     = (e24 >>> 8)^2                      for the lock detector
//      resid  = rhu(e24 * T_SLOT_PS, 24)           error in ps (nominal slot)
//
// WHY THESE CHOICES
// * "No dividers": interpolation is one multiply because the NCO frequency
//   is kept in slots per ps (nco_core). The 2^-22 truncation of fps for the
//   multiplier costs < fine' * 2^-34 slot (< 3 fs); the 25-bit operand limits
//   the source to < 976 MHz.
// * Taking the fractional part as the error IS the wrap to [-0.5, 0.5): the
//   phase detector is a sawtooth with period one slot. With fraction
//   t = (phi24 + 1/2) mod 1, e = t - 1/2 is just {~t[23], t[22:0]}: no adder.
// * resid_ps uses the NOMINAL slot period (a parameter). The true period
//   differs by ppm, so the ps value is off by ppm of itself (< 0.01 ps).
//
// Latency pop -> outputs valid: 4 clocks (registered at the end of A, B, C, D).
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_phase_det #(
    parameter integer COARSE_W  = 48,
    parameter integer TAG_W     = 4,
    parameter integer T_SLOT_PS = 10000
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 pop,
    input  wire [COARSE_W-1:0]  pop_coarse,
    input  wire [15:0]          pop_fine,
    input  wire [TAG_W-1:0]     pop_tag,
    input  wire [87:0]          phase,
    input  wire signed [47:0]   fps,
    // stage D outputs
    output reg                  d_valid = 1'b0,
    output reg  [47:0]          d_slot = 48'd0,
    output reg  signed [24:0]   d_e24 = 25'sd0,
    output reg  [23:0]          d_abs = 24'd0,        // |e24|, 0 .. 2^23
    output reg  [31:0]          d_sq = 32'd0,         // (e24>>>8)^2, 2^-32 slot^2
    output reg  signed [15:0]   d_resid = 16'sd0,     // ps
    output reg  [COARSE_W-1:0]  d_coarse = {COARSE_W{1'b0}},
    output reg  [TAG_W-1:0]     d_tag = {TAG_W{1'b0}}
);
    // ---- A ----
    reg               a_valid = 1'b0;
    reg [87:0]        a_phi = 88'd0;
    reg signed [24:0] a_m = 25'sd0;          // fps >>> 22: 2^-34 slot/ps
    reg [15:0]        a_fine = 16'd0;
    reg [COARSE_W-1:0] a_c;
    reg [TAG_W-1:0]   a_tag;
    // ---- B ----
    reg               b_valid = 1'b0;
    reg [87:0]        b_phi = 88'd0;
    reg signed [41:0] b_prod = 42'sd0;
    reg [COARSE_W-1:0] b_c;
    reg [TAG_W-1:0]   b_tag;
    // ---- C ----
    reg               c_valid = 1'b0;
    reg [87:0]        c_phi = 88'd0;
    reg [COARSE_W-1:0] c_c;
    reg [TAG_W-1:0]   c_tag;

    // ---- D combinational ----
    wire [71:0]        phi24 = c_phi[87:16];
    wire [71:0]        t     = phi24 + 72'h800000;            // + 1/2 slot
    wire signed [24:0] e24   = {~t[23], ~t[23], t[22:0]};     // t[23:0] - 2^23, sign-extended
    wire signed [16:0] e16   = e24[24:8];                     // floor(e24 / 2^8)
    wire signed [33:0] sq    = e16 * e16;
    wire signed [40:0] rprod = e24 * $signed(16'd0 + T_SLOT_PS);
    wire signed [40:0] rrnd  = rprod + 41'sd8388608;          // + 2^23: round half up
    wire [23:0]        absv  = e24[24] ? -e24[23:0] : e24[23:0];

    always @(posedge clk) begin
        // A: capture the NCO state of the release cycle
        a_valid <= pop & ~rst;
        if (pop) begin
            a_phi  <= phase;
            a_m    <= fps[46:22];
            a_fine <= pop_fine;
            a_c    <= pop_coarse;
            a_tag  <= pop_tag;
        end
        // B: one multiply
        b_valid <= a_valid & ~rst;
        b_prod  <= $signed({1'b0, a_fine}) * a_m;
        b_phi   <= a_phi;
        b_c     <= a_c;
        b_tag   <= a_tag;
        // C: interpolate back to the photon
        c_valid <= b_valid & ~rst;
        c_phi   <= b_phi - {{40{b_prod[41]}}, b_prod, 6'd0};
        c_c     <= b_c;
        c_tag   <= b_tag;
        // D: slot decision and error
        d_valid  <= c_valid & ~rst;
        d_slot   <= t[71:24];
        d_e24    <= e24;
        d_abs    <= (e24 == -25'sd8388608) ? 24'h800000 : absv;
        d_sq     <= sq[31:0];
        d_resid  <= rrnd[39:24];
        d_coarse <= c_c;
        d_tag    <= c_tag;
    end
endmodule

`default_nettype wire
