// =============================================================================
// nco_align.v  --  hold each timestamp until the NCO reaches its time
// -----------------------------------------------------------------------------
// WHY
//   The NCO is a real-time counter: its phase register only holds the value
//   for "now". A timestamp {c, fine} describes a moment just before clock
//   edge c, but the TDC delivers it ~10 clocks later. So the NCO's own time
//   axis, t_nco, runs D clocks BEHIND the TDC's coarse counter (D = 20), and
//   this FIFO holds each timestamp until t_nco reaches c. At that moment the
//   phase register holds exactly phase(edge c), and the phase detector
//   interpolates back by fine_ps.
//
// RELEASE RULE (one hit per POP_GAP cycles at most)
//   The head is released ("popped") in the first cycle n with t_nco >= c, at
//   least 6 cycles after it was written, and at least POP_GAP cycles after the
//   previous pop. Usually n = c (j = 0). If the hit is released j = n - c
//   cycles late (two channels with the same coarse time, a burst), the
//   timestamp is rewritten exactly as
//        fine' = fine_ps + j * T_CLK_PS
//   because  c*T - fine = n*T - (fine + j*T). Nothing is lost.
//   If j > J_MAX the hit is LATE: dropped, counted (late_cnt), late pulse.
//   That only happens if the input is very out of order or D is too small.
//
//   POP_GAP = 8 is what keeps the control stages hazard-free: the loop's
//   state updates take 7 cycles (nco_top), so every hit sees the state left
//   by the previous one. It also limits sustained throughput to 31.25 Mhit/s.
//
// PIPELINE. A 48-bit compare doesn't fit in one cycle, so this runs ahead of
// the NCO: t_lead = t_nco + LEAD (4), so the decision is ready just in time.
//   z    head registered; j = t_lead - c, low half
//   z+1  j, high half
//   z+2  j flags: due, late
//   z+3  decision (y)
//   z+4  pop, pop_fine = fine + j*T  (t_nco = t_lead[z] = n)
// A write in the cycle ending a is seen in a+1, so z = a+2 and n = a+6.
//
// J_MAX <= 24 keeps fine' within the phase detector's 17 bits for any fine_ps up
// to 35071 ps; real fine_ps is under 5 ns, and 11 ns for the ideal LUT at code 511.
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
    parameter integer J_MAX    = 24,
    parameter integer POP_GAP  = 8,
    parameter integer T_CLK_PS = 4000
)(
    input  wire                clk,
    input  wire                rst,
    input  wire [COARSE_W-1:0] t_lead,       // t_nco + 4
    // timestamps in
    input  wire                s_valid,
    output wire                s_ready,
    input  wire [COARSE_W-1:0] s_coarse,
    input  wire [FINE_W-1:0]   s_fine,
    input  wire [TAG_W-1:0]    s_tag,
    // release, registered: valid in the pop cycle (t_nco = n)
    output reg                 pop = 1'b0,
    output reg  [COARSE_W-1:0] pop_coarse = {COARSE_W{1'b0}},
    output reg  [FINE_W:0]     pop_fine = {(FINE_W+1){1'b0}},   // fine' = fine + j*T
    output reg  [TAG_W-1:0]    pop_tag = {TAG_W{1'b0}},
    output reg                 late = 1'b0,  // 1-cycle pulse per late hit
    output reg  [15:0]         late_cnt = 16'd0
);
    localparam integer DW = COARSE_W + FINE_W + TAG_W;
    localparam integer LO = 24;              // bits of j in the first cycle

    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW:0]   wr = 0, rd = 0;
    reg [3:0]    gap = 4'd0;                 // cycles until the next release is allowed

    wire empty = (wr == rd);
    wire full  = (wr[AW] != rd[AW]) && (wr[AW-1:0] == rd[AW-1:0]);
    assign s_ready = ~full & ~rst;

    // ---- z: registered FIFO head ----
    reg [DW-1:0]        head = {DW{1'b0}};
    reg                 h_v = 1'b0, jl_v = 1'b0, jh_v = 1'b0, fl_v = 1'b0;
    wire [COARSE_W-1:0] head_c = head[DW-1 -: COARSE_W];
    wire [FINE_W-1:0]   head_f = head[TAG_W +: FINE_W];
    wire [TAG_W-1:0]    head_t = head[TAG_W-1:0];

    // ---- j = t_lead - c, modulo 2^COARSE_W, split in two cycles ----
    reg [LO:0]            j_lo = {(LO+1){1'b0}};
    reg [COARSE_W-LO-1:0] t_hi = 0, c_hi = 0, j_hi = 0;
    reg [LO-1:0]          j_lo_d = {LO{1'b0}};

    // ---- flags: negative (MSB set) = not yet due ----
    reg       f_due = 1'b0, f_late = 1'b0;
    reg [4:0] j_f = 5'd0;

    wire rel = fl_v && f_due && (gap == 4'd0) && ~rst;

    // j * T_CLK_PS for j < 32: a small table, one LUT per bit
    reg [16:0] j_ps = 17'd0;
    always @(*) j_ps = j_f * T_CLK_PS;

    always @(posedge clk) begin
        // z
        head <= mem[rd[AW-1:0]];
        h_v  <= ~empty & ~rel & ~rst;
        j_lo <= {1'b0, t_lead[LO-1:0]} - {1'b0, head_c[LO-1:0]};
        t_hi <= t_lead[COARSE_W-1:LO];
        c_hi <= head_c[COARSE_W-1:LO];
        jl_v <= h_v & ~rel & ~rst;
        // z+1
        j_hi   <= t_hi - c_hi - j_lo[LO];
        j_lo_d <= j_lo[LO-1:0];
        jh_v   <= jl_v & ~rel & ~rst;
        // z+2
        f_due  <= ~j_hi[COARSE_W-LO-1];
        f_late <= (|j_hi) || (|j_lo_d[LO-1:5]) || (j_lo_d[4:0] > J_MAX);
        j_f    <= j_lo_d[4:0];
        fl_v   <= jh_v & ~rel & ~rst;
        // z+3: the decision; registered, so pop is valid in cycle n
        pop        <= rel & ~f_late;
        pop_coarse <= head_c;
        pop_fine   <= {1'b0, head_f} + j_ps;
        pop_tag    <= head_t;

        late <= 1'b0;
        if (s_valid && s_ready)
            mem[wr[AW-1:0]] <= {s_coarse, s_fine, s_tag};
        if (rst) begin
            wr <= 0; rd <= 0; gap <= 4'd0; late_cnt <= 16'd0; pop <= 1'b0;
        end else begin
            if (s_valid && s_ready) wr <= wr + 1'b1;
            if (rel) begin
                gap <= POP_GAP - 1;
                rd  <= rd + 1'b1;
                if (f_late) begin
                    late <= 1'b1;
                    if (late_cnt != 16'hFFFF) late_cnt <= late_cnt + 1'b1;
                end
            end else if (gap != 4'd0) begin
                gap <= gap - 1'b1;
            end
        end
    end
endmodule

`default_nettype wire
