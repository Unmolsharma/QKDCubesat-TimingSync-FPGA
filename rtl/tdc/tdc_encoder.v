// =============================================================================
// tdc_encoder.v  --  bubble-tolerant thermometer-to-binary encoder
//                    (pipelined popcount, 3-cycle latency)
// -----------------------------------------------------------------------------
// THE PROBLEM: BUBBLES
//   Ideally the sampled code is 111..1100..00. In a real carry chain, adjacent
//   taps have slightly different clock arrival times and FF setup times, and
//   inside a CARRY4 the CO outputs don't switch in strict order. So near the
//   edge you get codes like 1111101100000 (a "bubble").
//
// WHY POPCOUNT (ONES-COUNTER)
//   fine = number of 1s in the code.
//   * If tap i has an effective sampling threshold thr[i], then
//       popcount(tau) = #{ i : thr[i] <= tau }
//     which is MONOTONIC in tau by construction, whatever order the
//     thresholds are in. A bubble only means two taps swapped thresholds, and
//     the count doesn't care about order: every tap adds exactly one code,
//     so no bin is lost.
//   * A priority encoder ("index of the highest 1") is also monotonic while
//     the thresholds stay fixed. But when two taps are out of order, one
//     output code can never occur (a MISSING code) and its neighbour becomes a
//     double-width bin, so resolution is thrown away exactly where the chain
//     is already worst. Its output also depends on ONE flip-flop, the highest
//     1. If that FF is noisy or metastable, the code jumps by the distance to
//     the next 1. Fat-tree / "ones-to-the-first-zero" encoders have the same
//     single-point dependency. Popcount spreads the decision over every tap,
//     so a noisy tap moves the result by at most one LSB. The sweep test
//     measures missing codes for both encoders on the same samples.
//   * Cost: an adder tree of ~N LUTs instead of ~N/2. For 192 taps that is
//     trivial on an A100T.
//
// PIPELINE (chosen to close timing at 250 MHz on a -1 part)
//   stage 1: split into 6-bit groups, one LUT6 level each -> 3-bit counts
//   stage 2: sum 8 of those per group                     -> 6-bit counts
//   stage 3: sum all stage-2 counts                        -> OUT_W bits
//   Latency: 3 clk from `code` to `count`. For N=192: 32 -> 4 -> 1.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_encoder #(
    parameter integer N_TAPS = 192,
    parameter integer OUT_W  = 9
)(
    input  wire              clk,
    input  wire [N_TAPS-1:0] code,
    output wire [OUT_W-1:0]  count
);
    localparam integer G1 = (N_TAPS + 5) / 6;   // stage-1 groups (6 bits)
    localparam integer G2 = (G1 + 7) / 8;       // stage-2 groups (8 x 3 bits)

    // Zero-extend to a whole number of 6-bit groups. Extra bits are 0, so the
    // count is unchanged.
    wire [6*G1-1:0] din;
    assign din = code;

    reg [2:0]       s1 [0:G1-1];
    reg [5:0]       s2 [0:G2-1];
    reg [OUT_W-1:0] s3 = {OUT_W{1'b0}};

    integer i, j;

    function [2:0] pop6;
        input [5:0] x;
        begin
            pop6 = x[0] + x[1] + x[2] + x[3] + x[4] + x[5];
        end
    endfunction

    initial begin
        for (i = 0; i < G1; i = i + 1) s1[i] = 3'd0;
        for (i = 0; i < G2; i = i + 1) s2[i] = 6'd0;
    end

    // stage 1
    always @(posedge clk)
        for (i = 0; i < G1; i = i + 1)
            s1[i] <= pop6(din[6*i +: 6]);

    // stage 2
    reg [5:0] acc2;
    always @(posedge clk)
        for (j = 0; j < G2; j = j + 1) begin
            acc2 = 6'd0;
            for (i = 0; i < 8; i = i + 1)
                if (8*j + i < G1)
                    acc2 = acc2 + s1[8*j + i];
            s2[j] <= acc2;
        end

    // stage 3
    reg [OUT_W-1:0] acc3;
    always @(posedge clk) begin
        acc3 = {OUT_W{1'b0}};
        for (j = 0; j < G2; j = j + 1)
            acc3 = acc3 + s2[j];
        s3 <= acc3;
    end

    assign count = s3;
endmodule

`default_nettype wire
