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
// PIPELINE (250 MHz on a -1 part: no stage adds more than 4 small numbers)
//   stage 1: 6-bit groups, one LUT6 level each       -> 3-bit counts (G1)
//   stage 2: pairs                                   -> 4-bit counts
//   stage 3: pairs                                   -> 5-bit counts
//   stage 4: groups of 4                             -> 7-bit counts
//   stage 5: sum of all                              -> OUT_W bits
//   Latency: 5 clk from `code` to `count`. For N=192: 32 -> 16 -> 8 -> 2 -> 1.
//   (It was 3 stages with an 8-way sum in stage 2: ~7 logic levels, the worst
//   path of the board build.)
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
    localparam integer G2 = (G1 + 1) / 2;       // pairs
    localparam integer G3 = (G2 + 1) / 2;       // pairs
    localparam integer G4 = (G3 + 3) / 4;       // groups of 4

    // Zero-extend to a whole number of 6-bit groups. Extra bits are 0, so the
    // count is unchanged.
    wire [6*G1-1:0] din;
    assign din = code;

    reg [2:0]       s1 [0:G1-1];
    reg [3:0]       s2 [0:G2-1];
    reg [4:0]       s3 [0:G3-1];
    reg [6:0]       s4 [0:G4-1];
    reg [OUT_W-1:0] s5 = {OUT_W{1'b0}};

    integer i, j;

    function [2:0] pop6;
        input [5:0] x;
        begin
            pop6 = x[0] + x[1] + x[2] + x[3] + x[4] + x[5];
        end
    endfunction

    initial begin
        for (i = 0; i < G1; i = i + 1) s1[i] = 3'd0;
        for (i = 0; i < G2; i = i + 1) s2[i] = 4'd0;
        for (i = 0; i < G3; i = i + 1) s3[i] = 5'd0;
        for (i = 0; i < G4; i = i + 1) s4[i] = 7'd0;
    end

    reg [6:0]       acc4;
    reg [OUT_W-1:0] acc5;
    always @(posedge clk) begin
        for (i = 0; i < G1; i = i + 1)
            s1[i] <= pop6(din[6*i +: 6]);
        for (i = 0; i < G2; i = i + 1)
            s2[i] <= s1[2*i] + ((2*i + 1 < G1) ? s1[2*i + 1] : 3'd0);
        for (i = 0; i < G3; i = i + 1)
            s3[i] <= s2[2*i] + ((2*i + 1 < G2) ? s2[2*i + 1] : 4'd0);
        for (j = 0; j < G4; j = j + 1) begin
            acc4 = 7'd0;
            for (i = 0; i < 4; i = i + 1)
                if (4*j + i < G3)
                    acc4 = acc4 + s3[4*j + i];
            s4[j] <= acc4;
        end
        acc5 = {OUT_W{1'b0}};
        for (j = 0; j < G4; j = j + 1)
            acc5 = acc5 + s4[j];
        s5 <= acc5;
    end

    assign count = s5;
endmodule

`default_nettype wire
