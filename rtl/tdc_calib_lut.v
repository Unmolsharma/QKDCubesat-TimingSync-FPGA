// =============================================================================
// tdc_calib_lut.v  --  raw fine code -> calibrated time (ps)
// -----------------------------------------------------------------------------
// Carry-chain bins are far from uniform: some taps are 2x the average and
// some are ~0. Multiplying the raw code by the nominal LSB gives an
// integral nonlinearity (INL) of several LSB. The standard fix is a lookup
// table holding the measured CENTRE of each bin, in ps, produced by the
// code-density test in scripts/calibrate.py.
//
//   fine_ps = LUT[fine]      (time from the hit to the capture edge, ps,
//                             relative to the start of the lowest bin)
//
// Default (LUT_FILE == ""): ideal linear mapping, LUT[k] = (k + 0.5) * LSB.
// The design therefore produces sensible, if uncalibrated, numbers before any
// calibration run.
//
// A write port lets the future control/calibration block (block 4) update
// the table at run time. Calibration can then track temperature without a
// rebuild. Tie cal_we to 0 if unused.
//
// Implementation: 2^FINE_W x PS_W with a registered read, so it infers one
// BRAM18 (512 x 16), or distributed RAM if Vivado prefers. Read latency is 1.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_calib_lut #(
    parameter integer FINE_W       = 9,
    parameter integer PS_W         = 16,
    parameter integer IDEAL_LSB_PS = 22,
    parameter         LUT_FILE     = ""     // $readmemh file, one hex word per line
)(
    input  wire              clk,
    input  wire [FINE_W-1:0] rd_addr,
    output reg  [PS_W-1:0]   rd_data = {PS_W{1'b0}},
    input  wire              cal_we,
    input  wire [FINE_W-1:0] cal_addr,
    input  wire [PS_W-1:0]   cal_wdata
);
    localparam integer DEPTH = 1 << FINE_W;
    localparam integer PS_MAX = (1 << PS_W) - 1;

    reg [PS_W-1:0] mem [0:DEPTH-1];
    integer k, v;

    initial begin
        if (LUT_FILE == "") begin
            for (k = 0; k < DEPTH; k = k + 1) begin
                v = k * IDEAL_LSB_PS + IDEAL_LSB_PS / 2;
                mem[k] = (v > PS_MAX) ? PS_MAX : v;
            end
        end else begin
            $readmemh(LUT_FILE, mem);
        end
    end

    always @(posedge clk) begin
        if (cal_we)
            mem[cal_addr] <= cal_wdata;
        rd_data <= mem[rd_addr];
    end
endmodule

`default_nettype wire
