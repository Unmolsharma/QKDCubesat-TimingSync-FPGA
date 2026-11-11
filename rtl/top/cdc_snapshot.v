// =============================================================================
// cdc_snapshot.v  --  carry a slowly changing multi-bit value to another clock
// -----------------------------------------------------------------------------
// For settings, counters and telemetry: values that are read as a whole now and
// then, not every cycle. A toggle handshake keeps a coherent copy in the
// destination domain, refreshed every few destination cycles:
//
//   dst: when ack (synchronised) == req, q <= hold and req toggles
//   src: when req (synchronised) != ack, hold <= d and ack <= req
//
// hold only changes after the destination has asked for a new value, and the
// destination copies it only after the source has answered, so all W bits are at
// least two destination cycles stable when they are sampled: no bit can be caught
// mid-change, whatever the clock relation. Only the two toggles are synchronised
// (ASYNC_REG). There is no reset: after a reset on either side the handshake picks
// up again by itself (the toggles only ever compare for equality). q starts at INIT.
//
// Timing constraint for the bus: hold -> q is a multicycle path by construction;
// scripts/build.tcl gives every path between the two clocks set_max_delay
// -datapath_only of one fast period.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module cdc_snapshot #(
    parameter integer W = 8,
    parameter [W-1:0] INIT = {W{1'b0}}
)(
    input  wire         clk_s,
    input  wire [W-1:0] d_s,
    input  wire         clk_d,
    output reg  [W-1:0] q_d = INIT
);
    // ---- source ----
    reg [W-1:0] hold = INIT;
    reg         ack = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] req_s = 2'b00;
    // ---- destination ----
    reg         req = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_d = 2'b00;

    always @(posedge clk_s) begin
        req_s <= {req_s[0], req};
        if (req_s[1] != ack) begin
            hold <= d_s;
            ack  <= req_s[1];
        end
    end

    always @(posedge clk_d) begin
        ack_d <= {ack_d[0], ack};
        if (ack_d[1] == req) begin
            q_d <= hold;
            req <= ~req;
        end
    end
endmodule

`default_nettype wire
