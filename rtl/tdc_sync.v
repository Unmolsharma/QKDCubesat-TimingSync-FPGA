// =============================================================================
// tdc_sync.v  --  second register stage on the captured thermometer code
// -----------------------------------------------------------------------------
// The first-stage FFs in the carry-chain slices sample an asynchronous signal,
// so every so often one of them will capture while the edge is passing and go
// metastable. This extra register gives that FF a full clock period (4 ns,
// minus routing) to resolve before anything downstream uses the value.
//
// Why a metastable tap is harmless here: only the one or two taps that the
// edge is crossing at the clock edge can go metastable. Whichever way they
// resolve, the popcount changes by at most one code, which is an error of
// one LSB (~20 ps), not a wrong timestamp.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_sync #(
    parameter integer W = 192
)(
    input  wire         clk,
    input  wire [W-1:0] d,
    output wire [W-1:0] q
);
    (* ASYNC_REG = "TRUE", DONT_TOUCH = "TRUE" *)
    reg [W-1:0] q_r = {W{1'b0}};

    always @(posedge clk)
        q_r <= d;

    assign q = q_r;
endmodule

`default_nettype wire
