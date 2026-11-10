// =============================================================================
// nco_phase_det.v  --  NCO phase at the photon, slot number, phase error
// -----------------------------------------------------------------------------
// For a hit released in cycle n (t_nco = n, see nco_align) with fine' ps
// before edge n:
//
//   prod   = fine' * (fps >>> 22)               ONE multiply (one DSP48E1), 2^-34 slot
//   phi    = phase[n] - (prod << 6)             NCO phase at the photon,
//                                                2^-40 slot, mod 2^88
//   phi24  = phi >> 16   (floor, 2^-24 slot)
//   slot   = round(phi24)  = (phi24 + 1/2) >> 24 (48 bit)
//   e24    = phi24 - slot  in [-0.5, 0.5)       the phase error
//   e2     = (e24 >>> 8)^2                      for the lock detector
//   resid  = rhu(e24 * T_SLOT_PS, 24)           error in ps (nominal slot)
//
// WHY THESE CHOICES
// * "No dividers": interpolation is one multiply because the NCO frequency
//   is kept in slots per ps (nco_core). The 2^-22 truncation of fps for the
//   multiplier costs < fine' * 2^-34 slot (< 3 fs); the 25-bit operand limits
//   the source to < 976 MHz.
// * Taking the fractional part as the error IS the wrap to [-0.5, 0.5): the
//   phase detector is a sawtooth with period one slot. With t = phi24 + 1/2,
//   e = t[23:0] - 1/2, which is just phi24[23:0] read as a signed number: no
//   adder. Only the slot number needs the carry: slot = phi24[71:24] + phi24[23].
// * resid_ps uses the NOMINAL slot period (a parameter). The true period
//   differs by ppm, so the ps value is off by ppm of itself (< 0.01 ps).
//
// PIPELINE. Hits are at least POP_GAP = 8 cycles apart, so a value registered for
// a hit stays still while the following stages work on it, and a wide subtract can
// be split over several cycles with just the carry registered. Cycles after pop:
//   n     pop
//   n+1   a_*: phase, fps, fine' captured
//   n+3   prod (multiplier output)
//   n+4   phi bits 21:0 ...
//   n+5   ... 43:22: this is enough for e24 (phi bits 39:16)
//   n+6   e_h, ne_h, ge_h: error, -error, |error| >= gate threshold
//   n+7   ... 65:44 at n+6, 87:66 at n+7; sq and the raw residual product
//   n+8   slot, resid: captured and held for the output stage
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_phase_det #(
    parameter integer COARSE_W  = 48,
    parameter integer TAG_W     = 4,
    parameter integer T_SLOT_PS = 10000
)(
    input  wire                 clk,
    input  wire                 pop,
    input  wire [COARSE_W-1:0]  pop_coarse,
    input  wire [16:0]          pop_fine,
    input  wire [TAG_W-1:0]     pop_tag,
    input  wire [87:0]          phase,
    input  wire signed [47:0]   fps,
    input  wire [15:0]          gate_thr,      // 2^-16 slot
    input  wire                 st5,           // n+5
    input  wire                 st8,           // n+8
    // the hit's coarse time, valid from n+1
    output reg  [COARSE_W-1:0]  a_c = {COARSE_W{1'b0}},
    output reg  [TAG_W-1:0]     a_tag = {TAG_W{1'b0}},
    // valid from n+6, held until the next hit's n+6
    output reg  signed [24:0]   e_h = 25'sd0,
    output reg  signed [24:0]   ne_h = 25'sd0,
    output reg                  ge_h = 1'b0,
    // valid at n+7 and n+8
    output reg  [31:0]          sq = 32'd0,          // (e24>>>8)^2, 2^-32 slot^2
    // valid from n+8, held until the next hit's n+8
    output reg  [47:0]          slot_h = 48'd0,
    output reg  signed [15:0]   resid_h = 16'sd0     // ps
);
    // ---- n+1 ----
    reg [87:0]        a_phi = 88'd0;
    reg signed [24:0] a_m = 25'sd0;          // fps >>> 22: 2^-34 slot/ps
    reg signed [17:0] a_fine = 18'sd0;       // always non-negative
    always @(posedge clk)
        if (pop) begin
            a_phi  <= phase;
            a_m    <= fps[46:22];
            a_fine <= {1'b0, pop_fine};
            a_c    <= pop_coarse;
            a_tag  <= pop_tag;
        end

    // ---- n+2, n+3: one multiply ----
    reg signed [42:0] m_prod = 43'sd0, prod = 43'sd0;
    always @(posedge clk) begin
        m_prod <= a_fine * a_m;
        prod   <= m_prod;
    end

    // ---- n+4 ... n+7: phi = a_phi - (prod << 6), four 22-bit segments ----
    wire [87:0] sub = {{39{prod[42]}}, prod, 6'd0};
    reg  [22:0] s0 = 23'd0, s1 = 23'd0, s2 = 23'd0;
    reg  [21:0] s3 = 22'd0;
    always @(posedge clk) begin
        s0 <= {1'b0, a_phi[21:0]}  - {1'b0, sub[21:0]};
        s1 <= {1'b0, a_phi[43:22]} - {1'b0, sub[43:22]} - s0[22];
        s2 <= {1'b0, a_phi[65:44]} - {1'b0, sub[65:44]} - s1[22];
        s3 <= a_phi[87:66] - sub[87:66] - s2[22];
    end

    // ---- e24: phi bits 39:16, read as signed ----
    wire [23:0]        frac = {s1[17:0], s0[21:16]};
    wire signed [24:0] e24  = {frac[23], frac};
    wire signed [16:0] e16  = e24[24:8];     // floor(e24 / 2^8)

    // ---- gate: |e24| >= g without a negate-then-compare ----
    reg signed [24:0] thr_p = 25'sd0, thr_n = 25'sd0;
    always @(posedge clk) begin
        thr_p <= {1'b0, gate_thr, 8'd0};
        thr_n <= -$signed({1'b0, gate_thr, 8'd0});
    end

    always @(posedge clk)
        if (st5) begin
            e_h  <= e24;
            ne_h <= -e24;
            ge_h <= (e24 >= thr_p) || (e24 <= thr_n);
        end

    // ---- e^2 (n+6: DSP multiplier register, n+7: output register) ----
    reg signed [33:0] m_sq = 34'sd0;
    always @(posedge clk) begin
        m_sq <= e16 * e16;
        sq   <= m_sq[31:0];
    end

    // ---- residual in ps: floor(x / 2^24) + x[23] is round half up ----
    reg signed [40:0] m_r = 41'sd0, p_r = 41'sd0;
    always @(posedge clk) begin
        m_r <= e24 * $signed(16'd0 + T_SLOT_PS);
        p_r <= m_r;
    end

    // ---- slot = phi[87:40] + phi[39], two segments ----
    reg [24:0] sl_lo = 25'd0;
    reg [23:0] sl_hi = 24'd0;
    always @(posedge clk) begin
        sl_lo <= {1'b0, s2[19:0], s1[21:18]} + frac[23];     // phi[63:40]
        sl_hi <= {s3, s2[21:20]} + sl_lo[24];                // phi[87:64]
        if (st8) begin
            slot_h  <= {sl_hi, sl_lo[23:0]};
            resid_h <= p_r[39:24] + p_r[23];
        end
    end
endmodule

`default_nettype wire
