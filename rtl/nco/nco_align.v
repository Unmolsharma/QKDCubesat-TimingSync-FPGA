// =============================================================================
// nco_align.v  --  hold each timestamp until the NCO reaches its time
// -----------------------------------------------------------------------------
// WHY
//   The NCO is a real-time counter: its phase register only holds the value
//   for "now". A timestamp {c, fine} describes a moment just before clock
//   edge c, but the TDC delivers it ~9 clocks later. So the NCO's own time
//   axis, t_nco, runs D clocks BEHIND the TDC's coarse counter (D = 16 > TDC
//   latency), and this FIFO holds each timestamp until t_nco reaches c. At
//   that moment the phase register holds exactly phase(edge c), and the phase
//   detector interpolates back by fine_ps.
//
// RELEASE RULE (one hit per POP_GAP = 2 cycles at most)
//   The head is released ("popped") in the first cycle n with t_nco >= c and
//   at least POP_GAP cycles after the previous pop. Usually n = c (j = 0).
//   If the hit is released j = n - c cycles late (two channels with the same
//   coarse time, a burst), the timestamp is rewritten exactly as
//        fine' = fine_ps + j * T_CLK_PS
//   because  c*T - fine = n*T - (fine + j*T). Nothing is lost.
//   If j > J_MAX the hit is LATE: dropped, counted (late_cnt), late pulse.
//   That only happens if the input is very out of order or D is too small.
//
//   POP_GAP = 2 is what keeps the control stages hazard-free: every hit's
//   control logic sees the state left by the previous hit (nco_top).
//
// BACK-PRESSURE: s_ready = FIFO not full (2^AW entries). A full FIFO makes
// the TDC drop and count the hit, the same policy as block 1.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_align #(
    parameter integer COARSE_W = 48,
    parameter integer FINE_W   = 16,
    parameter integer TAG_W    = 4,
    parameter integer AW       = 4,
    parameter integer J_MAX    = 12,
    parameter integer T_CLK_PS = 4000
)(
    input  wire                clk,
    input  wire                rst,
    input  wire [COARSE_W-1:0] t_nco,
    // timestamps in
    input  wire                s_valid,
    output wire                s_ready,
    input  wire [COARSE_W-1:0] s_coarse,
    input  wire [FINE_W-1:0]   s_fine,
    input  wire [TAG_W-1:0]    s_tag,
    // release (combinational, in the pop cycle: the phase detector registers
    // the NCO state at the same edge)
    output wire                pop,          // a valid hit is released this cycle
    output wire [COARSE_W-1:0] pop_coarse,
    output wire [FINE_W-1:0]   pop_fine,     // fine' = fine + j*T
    output wire [TAG_W-1:0]    pop_tag,
    output reg                 late = 1'b0,  // 1-cycle pulse per late hit
    output reg  [15:0]         late_cnt = 16'd0
);
    localparam integer DW = COARSE_W + FINE_W + TAG_W;

    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW:0]   wr = 0, rd = 0;
    reg          gap = 1'b0;          // high in the cycle right after a pop

    wire empty = (wr == rd);
    wire full  = (wr[AW] != rd[AW]) && (wr[AW-1:0] == rd[AW-1:0]);
    assign s_ready = ~full & ~rst;

    wire [DW-1:0]       head   = mem[rd[AW-1:0]];
    wire [COARSE_W-1:0] head_c = head[DW-1 -: COARSE_W];
    wire [FINE_W-1:0]   head_f = head[TAG_W +: FINE_W];
    wire [TAG_W-1:0]    head_t = head[TAG_W-1:0];

    // j = t_nco - c, modulo 2^COARSE_W; negative (MSB set) = not yet due
    wire [COARSE_W-1:0] j_full = t_nco - head_c;
    wire                due    = ~j_full[COARSE_W-1];
    wire                is_late = due && (j_full > J_MAX);
    wire                rel = ~empty && due && ~gap && ~rst;

    assign pop        = rel && ~is_late;
    assign pop_coarse = head_c;
    assign pop_fine   = head_f + j_full[4:0] * T_CLK_PS;   // j <= J_MAX < 32
    assign pop_tag    = head_t;

    always @(posedge clk) begin
        late <= 1'b0;
        if (s_valid && s_ready)
            mem[wr[AW-1:0]] <= {s_coarse, s_fine, s_tag};
        if (rst) begin
            wr <= 0; rd <= 0; gap <= 1'b0; late_cnt <= 16'd0;
        end else begin
            if (s_valid && s_ready) wr <= wr + 1'b1;
            gap <= rel;
            if (rel) begin
                rd <= rd + 1'b1;
                if (is_late) begin
                    late <= 1'b1;
                    if (late_cnt != 16'hFFFF) late_cnt <= late_cnt + 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
