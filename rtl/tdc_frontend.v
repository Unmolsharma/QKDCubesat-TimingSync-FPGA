// =============================================================================
// tdc_frontend.v  --  delay line -> sync -> encoder, plus line-state flags
// -----------------------------------------------------------------------------
// Produces one aligned sample per clock:
//   fine  : popcount of the thermometer code (bubble-tolerant)
//   t0    : tap 0 was 1          (the edge has entered the line)
//   zero  : every tap was 0      (line idle / fully reset)
//   full  : every tap was 1      (edge passed the WHOLE line = saturated)
//
// The flags go through their own pipeline, matched to the encoder's latency,
// so all four outputs describe the SAME clock sample. The 192-input AND/OR
// reductions are split over two register stages (32 partials, then 1) so they
// don't limit Fmax.
//
// Latency from the capture edge E (where the slice FFs sample the chain):
//   E+1 sync stage, E+2..E+4 encoder  ->  outputs valid after edge E+4.
//   LATENCY = 4 is exported so tdc_top can delay the coarse count to match.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_frontend #(
    parameter integer NUM_CARRY4    = 48,
    parameter integer FINE_W        = 9,
    parameter real    TAP_NOM_PS    = 22.0,
    parameter real    TAP_SIGMA_PCT = 15.0,
    parameter real    SKEW_SIGMA_PS = 6.0,
    parameter real    JITTER_PS     = 4.0,
    parameter integer SEED          = 1
)(
    input  wire              clk,
    input  wire              hit,
    output wire [FINE_W-1:0] fine,
    output wire              t0,
    output wire              zero,
    output wire              full
);
    localparam integer N       = 4 * NUM_CARRY4;
    localparam integer LATENCY = 4;               // capture edge -> outputs
    localparam integer GP      = (N + 5) / 6;     // partial-reduction groups

    wire [N-1:0] taps_q;     // stage 1 (in-slice capture FFs)
    wire [N-1:0] code;       // stage 2 (metastability register)

    tdc_delay_line #(
        .NUM_CARRY4    (NUM_CARRY4),
        .TAP_NOM_PS    (TAP_NOM_PS),
        .TAP_SIGMA_PCT (TAP_SIGMA_PCT),
        .SKEW_SIGMA_PS (SKEW_SIGMA_PS),
        .JITTER_PS     (JITTER_PS),
        .SEED          (SEED)
    ) u_dl (
        .clk    (clk),
        .hit    (hit),
        .taps_q (taps_q)
    );

    tdc_sync #(.W(N)) u_sync (
        .clk (clk),
        .d   (taps_q),
        .q   (code)
    );

    tdc_encoder #(.N_TAPS(N), .OUT_W(FINE_W)) u_enc (
        .clk   (clk),
        .code  (code),
        .count (fine)
    );

    // ---- flag pipeline (3 stages, same as the encoder) ----
    // Pad with values that don't affect the result: 1 for AND, 0 for OR.
    // (The "+1" avoids a zero-width replication when N is a multiple of 6;
    // the extra MSB is simply truncated by the assignment.)
    wire [6*GP-1:0] code_and = {{(6*GP-N+1){1'b1}}, code};
    wire [6*GP-1:0] code_or  = {{(6*GP-N+1){1'b0}}, code};

    reg [GP-1:0] and_p = {GP{1'b0}}, or_p = {GP{1'b0}};
    reg          t0_a = 1'b0, t0_b = 1'b0, t0_c = 1'b0;
    reg          and_b = 1'b0, or_b = 1'b0, and_c = 1'b0, or_c = 1'b0;
    integer i;

    always @(posedge clk) begin
        // stage 1: 6-input partials (1 LUT each)
        for (i = 0; i < GP; i = i + 1) begin
            and_p[i] <= &code_and[6*i +: 6];
            or_p[i]  <= |code_or [6*i +: 6];
        end
        t0_a  <= code[0];
        // stage 2: reduce partials (32 inputs -> 2 LUT levels)
        and_b <= &and_p;
        or_b  <= |or_p;
        t0_b  <= t0_a;
        // stage 3: align with encoder output
        and_c <= and_b;
        or_c  <= or_b;
        t0_c  <= t0_b;
    end

    assign t0   = t0_c;
    assign zero = ~or_c;
    assign full = and_c;
endmodule

`default_nettype wire
