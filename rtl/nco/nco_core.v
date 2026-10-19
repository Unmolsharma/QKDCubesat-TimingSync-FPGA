// =============================================================================
// nco_core.v  --  the numerically controlled oscillator (phase accumulator)
// -----------------------------------------------------------------------------
// WHAT IT IS
//   A counter of SOURCE slots that runs on our 250 MHz clock. `phase` holds
//   "how many source pulses have happened since reset", as a 48-bit slot
//   index plus a 40-bit fraction. Every clock it advances by `fclk`, the
//   number of source slots per clock period (0.4 for a 100 MHz source, 2.0
//   for 500 MHz). The tracking loop nudges the phase and the frequency so
//   that a photon from slot k arrives exactly when phase = k (+ a constant).
//
// WHY THE FREQUENCY IS STORED AS SLOTS PER PICOSECOND
//   The phase detector needs the NCO phase at the photon's arrival time,
//   which lies fine_ps BEFORE a clock edge:  phase(hit) = phase(edge) -
//   fine_ps * f. If f is in slots/ps that is ONE multiply. The per-clock
//   increment is then fclk = f * T_CLK_PS = f * 4000, a multiply by a
//   constant (4000 = 2^12 - 2^7 + 2^5: three shifted adds, no DSP).
//
// FORMATS (see model/loop_fixed.py, which this must match bit for bit)
//   phase  unsigned 88 bit, 2^-40 slot   (wraps every 2^48 slots: 6.5 days at 500 MHz)
//   fps    signed 48 bit,   2^-56 slot/ps
//   fclk   signed 44 bit,   2^-40 slot   = (fps * 4000) >>> 16 (floor), registered,
//          so fclk[n+1] = f(fps[n]).
//   The truncation of fclk (< 2^-40 slot per clock, 2e-12 relative) is a tiny
//   constant frequency error that the loop's integrator absorbs.
//
// CORRECTIONS (all applied at the clock edge that ends the strobe cycle)
//   loop_*  from the tracking loop, one strobe per loop-updating hit.
//   ext_*   EXTERNAL CORRECTION PORT, unused by default (tie ext_valid low).
//           Intended for later: GPS-disciplined frequency, or a phase/frequency
//           offset measured by two-way time transfer. ext_dphase is in
//           2^-40 slot, ext_dfreq in 2^-56 slot/ps, both signed and additive;
//           a strobe in the same cycle as a loop strobe adds to it. The loop
//           sees the result like any other disturbance, so an external
//           correction should be small or applied while unlocked.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_core #(
    parameter [47:0]  FPS_NOM  = 48'd7205759403793,   // 100 MHz: round(1e-4 * 2^56)
    parameter integer T_CLK_PS = 4000
)(
    input  wire               clk,
    input  wire               rst,
    // tracking-loop corrections
    input  wire               loop_valid,
    input  wire signed [47:0] loop_dphi,     // 2^-40 slot
    input  wire signed [47:0] loop_df,       // 2^-56 slot/ps
    // external correction port (default unused)
    input  wire               ext_valid,
    input  wire signed [47:0] ext_dphase,    // 2^-40 slot
    input  wire signed [47:0] ext_dfreq,     // 2^-56 slot/ps
    // state
    output reg         [87:0] phase = 88'd0,
    output reg  signed [47:0] fps   = FPS_NOM,
    output reg  signed [43:0] fclk  = 44'd0
);
    // fps * 4000 >>> 16; Vivado maps a constant multiply to shift-and-add.
    wire signed [63:0] fps_x4000 = $signed({{16{fps[47]}}, fps}) * $signed(64'd4000);
    wire signed [63:0] nom_x4000 = $signed({16'd0, FPS_NOM}) * $signed(64'd4000);
    wire signed [43:0] fclk_next = fps_x4000[59:16];
    wire signed [43:0] fclk_nom  = nom_x4000[59:16];

    wire [87:0] dphi_l = loop_valid ? {{40{loop_dphi[47]}}, loop_dphi} : 88'd0;
    wire [87:0] dphi_e = ext_valid  ? {{40{ext_dphase[47]}}, ext_dphase} : 88'd0;
    wire signed [47:0] df_l = loop_valid ? loop_df   : 48'sd0;
    wire signed [47:0] df_e = ext_valid  ? ext_dfreq : 48'sd0;

    initial fclk = fclk_nom;

    always @(posedge clk) begin
        if (rst) begin
            phase <= 88'd0;
            fps   <= FPS_NOM;
            fclk  <= fclk_nom;
        end else begin
            phase <= phase + {{44{fclk[43]}}, fclk} + dphi_l + dphi_e;
            fps   <= fps + df_l + df_e;
            fclk  <= fclk_next;
        end
    end
endmodule

`default_nettype wire
