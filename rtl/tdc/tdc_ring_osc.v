// =============================================================================
// tdc_ring_osc.v  --  on-chip free-running hit source for code-density tests
// -----------------------------------------------------------------------------
// Code-density calibration needs hits whose phase relative to the 250 MHz
// clock is uniformly random. A ring oscillator (RO) runs from its own
// gate delays, has no relationship to the MMCM clock, and drifts with
// temperature and voltage. Divided down, it gives a clean square wave
// whose edges walk across the clock period.
//
// Hardware: STAGES LUTs in a loop (1 NAND with enable + STAGES-1 inverters,
//   odd total inversion), ~200-400 MHz, then a 2^DIV_BITS ripple divider.
//   With DIV_BITS = 12 that's roughly 50-100 k hits/s. That is below the UART's
//   ~100 k codes/s, and each pulse is microseconds wide, which easily meets the
//   pulse-width requirement.
//
// VIVADO REQUIREMENTS (also in constraints/tdc_arty_a7.xdc):
//   * DONT_TOUCH on the LUTs, otherwise synthesis collapses the loop.
//   * ALLOW_COMBINATORIAL_LOOPS = TRUE on the loop net, otherwise DRC LUTLP-1
//     stops write_bitstream.
//   * create_clock on the ring output so the divider FFs are timed, and
//     asynchronous clock groups against the system clock.
//
// CAVEAT: an RO on the same die and supply as the TDC can weakly lock onto
// the system clock (injection pulling). The hit phase is then no longer
// uniform, and the histogram shows a comb or periodic ripple that is NOT
// in the chain. Compare against an external pulse generator (Pmod JA1)
// before trusting a calibration built from the RO.
//
// Simulation: a plain `always #HALF_NS` clock with an irrational-ish period
// stands in for the ring. The divider is the same RTL.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_ring_osc #(
    parameter integer STAGES   = 5,          // odd
    parameter integer DIV_BITS = 12,
    parameter real    SIM_HALF_NS = 1.3719   // simulation stand-in only
)(
    input  wire en,
    output wire hit_out
);
    wire ro;

`ifdef SYNTHESIS
    (* DONT_TOUCH = "TRUE", ALLOW_COMBINATORIAL_LOOPS = "TRUE" *)
    wire [STAGES-1:0] n;

    // stage 0: n[0] = en & ~n[last]  (I0 = n[last], I1 = en)
    (* DONT_TOUCH = "TRUE" *)
    LUT2 #(.INIT(4'b0100)) u_ro_nand (.O(n[0]), .I0(n[STAGES-1]), .I1(en));

    genvar g;
    generate
        for (g = 1; g < STAGES; g = g + 1) begin : g_inv
            (* DONT_TOUCH = "TRUE" *)
            LUT1 #(.INIT(2'b01)) u_ro_inv (.O(n[g]), .I0(n[g-1]));
        end
    endgenerate

    assign ro = n[0];
`else
    reg ro_sim = 1'b0;
    always #(SIM_HALF_NS) ro_sim = en ? ~ro_sim : 1'b0;
    assign ro = ro_sim;
`endif

    // Divider clocked by the ring itself. Reset-free on purpose: the phase is
    // meant to be arbitrary.
    (* DONT_TOUCH = "TRUE" *)
    reg [DIV_BITS-1:0] div = {DIV_BITS{1'b0}};
    always @(posedge ro)
        div <= div + 1'b1;

    assign hit_out = div[DIV_BITS-1] & en;
endmodule

`default_nettype wire
