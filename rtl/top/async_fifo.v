// =============================================================================
// async_fifo.v  --  dual-clock FIFO (gray-coded pointers), BRAM friendly
// -----------------------------------------------------------------------------
// Write side: w_en must only be asserted while w_ready. w_ready is REGISTERED and
// conservative: high while at most 2^AW - 2 entries were in use (as far as the
// write side knows; the synchronised read pointer can only be stale towards
// "fuller"), so a write in the cycle w_ready is computed and one in the next always
// fit: all 2^AW entries are usable. Read side: r_en while !r_empty; r_data is registered (valid the cycle after
// r_en), as a block RAM read.
// Each pointer crosses as gray code through two ASYNC_REG flip-flops, so at most
// one bit changes per write/read and a sample is always either the old or the new
// value. scripts/build.tcl limits the crossing paths to one fast period.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module async_fifo #(
    parameter integer DW = 8,
    parameter integer AW = 4
)(
    input  wire          w_clk,
    input  wire          w_rst,
    input  wire          w_en,
    input  wire [DW-1:0] w_data,
    output reg           w_ready = 1'b0,
    input  wire          r_clk,
    input  wire          r_rst,
    input  wire          r_en,
    output reg  [DW-1:0] r_data = {DW{1'b0}},
    output wire          r_empty
);
    reg [DW-1:0] mem [0:(1<<AW)-1];

    function [AW:0] g2b;
        input [AW:0] g;
        integer i;
        begin
            g2b[AW] = g[AW];
            for (i = AW - 1; i >= 0; i = i - 1)
                g2b[i] = g2b[i+1] ^ g[i];
        end
    endfunction

    reg [AW:0] wb = 0, wg = 0;                       // write pointer: binary, gray
    reg [AW:0] rb = 0, rg = 0;                       // read pointer: binary, gray

    // ---------------- write side ----------------
    (* ASYNC_REG = "TRUE" *) reg [AW:0] rg_w1 = 0, rg_w2 = 0;
    reg [AW:0] rb_w = 0;
    wire [AW:0] wb_n = wb + 1'b1;
    wire [AW:0] occ  = wb - rb_w;
    always @(posedge w_clk) begin
        rg_w1 <= rg; rg_w2 <= rg_w1;
        rb_w  <= g2b(rg_w2);
        if (w_en) mem[wb[AW-1:0]] <= w_data;
        if (w_rst) begin
            wb <= 0; wg <= 0; w_ready <= 1'b0;
        end else begin
            if (w_en) begin
                wb <= wb_n;
                wg <= wb_n ^ (wb_n >> 1);
            end
            // occ excludes this cycle's write: with this one and the next, at most 2^AW
            w_ready <= (occ <= (1 << AW) - 2);
        end
    end

    // ---------------- read side ----------------
    (* ASYNC_REG = "TRUE" *) reg [AW:0] wg_r1 = 0, wg_r2 = 0;
    wire [AW:0] rb_n = rb + 1'b1;
    assign r_empty = (rg == wg_r2);
    always @(posedge r_clk) begin
        wg_r1 <= wg; wg_r2 <= wg_r1;
        if (r_en) r_data <= mem[rb[AW-1:0]];
        if (r_rst) begin
            rb <= 0; rg <= 0;
        end else if (r_en) begin
            rb <= rb_n;
            rg <= rb_n ^ (rb_n >> 1);
        end
    end
endmodule

`default_nettype wire
