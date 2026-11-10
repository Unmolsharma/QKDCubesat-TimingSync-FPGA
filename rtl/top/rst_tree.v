// =============================================================================
// rst_tree.v  --  pipelined reset distribution
// -----------------------------------------------------------------------------
// One reset net reaching thousands of flip-flops across the die cannot be timed
// at 250 MHz. Each block gets its own tap of a shift register instead. Reset
// latency doesn't matter (each block's reset only has to be long enough, and no
// timestamp is meaningful until reset has lifted), and because every tap has a
// different D input, synthesis cannot merge them back into a single net.
//
// All taps start asserted, so the design is held in reset from configuration
// until rst_in has been low for N cycles.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module rst_tree #(
    parameter integer N = 4
)(
    input  wire         clk,
    input  wire         rst_in,
    (* shreg_extract = "no", max_fanout = 64 *)
    output reg  [N-1:0] rst = {N{1'b1}}
);
    generate
        if (N == 1) begin : one
            always @(posedge clk) rst <= rst_in;
        end else begin : chain
            always @(posedge clk) rst <= {rst[N-2:0], rst_in};
        end
    endgenerate
endmodule

`default_nettype wire
