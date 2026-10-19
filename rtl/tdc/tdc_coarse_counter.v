// =============================================================================
// tdc_coarse_counter.v  --  free-running coarse time base
// -----------------------------------------------------------------------------
// Counts system-clock periods. A timestamp is {coarse, fine}: coarse says
// WHICH clock edge captured the hit, and fine says how long before that edge
// it arrived.
//
// Alignment: the fine code reaches the hit detector DELAY cycles after its
// capture edge (sync + encoder pipeline). `coarse_aligned` is the counter
// delayed by the same amount, so the hit detector sees the count value that
// was current just after the capture edge. Doing it with a delay line instead
// of "count - DELAY" makes the alignment exact by construction and costs only
// DELAY*COARSE_W FFs.
//
// 48 bits at 250 MHz wraps every 13 days, so every timestamp of a ~6 min
// satellite pass is unique (block 1 used 32 bits = 17.2 s). Unique
// timestamps are needed by block 2's 48-bit slot index and by any later
// two-way time transfer, which compares timestamps across the whole pass.
// Cost: 16 more counter bits and 16 x DELAY more FFs in the alignment line.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_coarse_counter #(
    parameter integer COARSE_W = 48,
    parameter integer DELAY    = 4
)(
    input  wire                clk,
    input  wire                rst,
    output wire [COARSE_W-1:0] coarse,           // live count
    output wire [COARSE_W-1:0] coarse_aligned    // delayed by DELAY cycles
);
    reg [COARSE_W-1:0] cnt = {COARSE_W{1'b0}};
    reg [COARSE_W-1:0] dly [0:DELAY-1];
    integer i;

    initial for (i = 0; i < DELAY; i = i + 1) dly[i] = {COARSE_W{1'b0}};

    always @(posedge clk) begin
        if (rst) cnt <= {COARSE_W{1'b0}};
        else     cnt <= cnt + 1'b1;
        dly[0] <= cnt;
        for (i = 1; i < DELAY; i = i + 1)
            dly[i] <= dly[i-1];
    end

    assign coarse         = cnt;
    assign coarse_aligned = dly[DELAY-1];
endmodule

`default_nettype wire
