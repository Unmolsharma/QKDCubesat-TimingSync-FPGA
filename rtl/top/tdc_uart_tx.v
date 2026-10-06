// =============================================================================
// tdc_uart_tx.v  --  minimal 8N1 UART transmitter
// -----------------------------------------------------------------------------
// CLKS_PER_BIT = f_clk / baud. The default 125 gives exactly 2 000 000 baud from
// 250 MHz (0 % rate error). The Arty's FT2232H generates 2 Mbaud exactly
// (3 MHz / 1.5), so both ends agree perfectly.
// Pulse `start` for one cycle while !busy; `data` is sampled on that cycle.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_uart_tx #(
    parameter integer CLKS_PER_BIT = 125
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire [7:0] data,
    output wire       busy,
    output wire       tx
);
    reg [9:0]  shreg   = 10'h3FF;        // {stop, data[7:0], start}, LSB first
    reg [3:0]  bits    = 4'd0;           // bits left to send
    reg [15:0] baud    = 16'd0;
    reg        tx_r    = 1'b1;

    assign busy = (bits != 4'd0);
    assign tx   = tx_r;

    always @(posedge clk) begin
        if (rst) begin
            bits <= 4'd0;
            tx_r <= 1'b1;
        end else if (!busy) begin
            tx_r <= 1'b1;
            if (start) begin
                shreg <= {1'b1, data, 1'b0};
                bits  <= 4'd10;
                baud  <= 16'd0;
            end
        end else begin
            tx_r <= shreg[0];
            if (baud == CLKS_PER_BIT - 1) begin
                baud  <= 16'd0;
                shreg <= {1'b1, shreg[9:1]};
                bits  <= bits - 1'b1;
            end else begin
                baud <= baud + 1'b1;
            end
        end
    end
endmodule

`default_nettype wire
