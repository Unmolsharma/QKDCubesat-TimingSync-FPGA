// =============================================================================
// uart_rx.v  --  minimal 8N1 UART receiver (PC -> FPGA commands)
// -----------------------------------------------------------------------------
// CLKS_PER_BIT = f_clk / baud (125: 2 000 000 baud from 250 MHz, the same
// rate as the transmit side). The line is synchronised with two FFs, a start
// bit is a falling edge that is still low half a bit later, and every data
// bit is sampled at its centre. A missing stop bit drops the byte and counts
// a framing error. Idle line = 1.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module uart_rx #(
    parameter integer CLKS_PER_BIT = 125
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,                 // asynchronous serial input
    output reg        valid = 1'b0,       // 1-cycle pulse with a received byte
    output reg  [7:0] data = 8'd0,
    output reg  [7:0] frame_err = 8'd0    // saturating count of bad stop bits
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] rx_s = 2'b11;
    always @(posedge clk) rx_s <= {rx_s[0], rx};
    wire r = rx_s[1];

    localparam [1:0] IDLE = 2'd0, START = 2'd1, BITS = 2'd2, STOP = 2'd3;
    reg [1:0]  st = IDLE;
    reg [15:0] cnt = 16'd0;
    reg [2:0]  nbit = 3'd0;
    reg [7:0]  sh = 8'd0;

    always @(posedge clk) begin
        valid <= 1'b0;
        if (rst) begin
            st <= IDLE; frame_err <= 8'd0;
        end else case (st)
            IDLE:
                if (!r) begin st <= START; cnt <= CLKS_PER_BIT / 2 - 1; end
            START:
                if (cnt == 0) begin
                    if (!r) begin st <= BITS; cnt <= CLKS_PER_BIT - 1; nbit <= 3'd0; end
                    else    st <= IDLE;                      // glitch, not a start bit
                end else cnt <= cnt - 1'b1;
            BITS:
                if (cnt == 0) begin
                    sh  <= {r, sh[7:1]};                     // LSB first
                    cnt <= CLKS_PER_BIT - 1;
                    if (nbit == 3'd7) st <= STOP;
                    nbit <= nbit + 1'b1;
                end else cnt <= cnt - 1'b1;
            STOP:
                if (cnt == 0) begin
                    if (r) begin valid <= 1'b1; data <= sh; end
                    else if (frame_err != 8'hFF) frame_err <= frame_err + 1'b1;
                    st <= IDLE;
                end else cnt <= cnt - 1'b1;
        endcase
    end
endmodule

`default_nettype wire
