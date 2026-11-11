// =============================================================================
// nco_align.v  --  hold each timestamp until the NCO reaches its time
// -----------------------------------------------------------------------------
// WHY
//   The NCO is a real-time counter: its phase register only holds the value
//   for "now". A timestamp {c, fine} describes a moment just before clock
//   edge c, but the TDC delivers it ~10 clocks later. So the NCO's own time
//   axis, t_nco, runs D clocks BEHIND the TDC's coarse counter (D = 31), and
//   this FIFO holds each timestamp until t_nco reaches c. At that moment the
//   phase register holds exactly phase(edge c), and the phase detector
//   interpolates back by fine_ps.
//
// RELEASE RULE (one hit per POP_GAP cycles at most)
//   A hit that arrives in cycle a (s_valid && s_ready, t_nco = a) is released
//   ("popped") in cycle
//        n = max(c, a + 10, n_prev + POP_GAP)
//   n_prev being the previous release. Usually n = c (j = 0). If the hit is
//   released j = n - c cycles late (two channels with the same coarse time, a
//   burst), the timestamp is rewritten exactly as
//        fine' = fine_ps + j * T_CLK_PS
//   because  c*T - fine = n*T - (fine + j*T). Nothing is lost.
//
// ADMISSION (decided before the FIFO, so a dropped hit costs nothing)
//   j <= J_LOOP            released normally: the loop uses it.
//   J_LOOP < j <= J_MAX    released as STALE (pop_stale): slot, error, residual
//                          and output as usual, but the loop ignores it (no loop
//                          delay beyond J_LOOP). Admitted only if its c is within
//                          W_STALE clocks of the last hit the loop used. This is for
//                          coincidences (all 5 channels in one slot: j = 0, 8, 16,
//                          24, 32) and short bursts; under sustained overload it
//                          lapses, so the queue drains back to fresh hits and the
//                          loop is never starved.
//   otherwise              LATE: dropped, counted (late_cnt), late pulse. It never
//                          enters the FIFO and takes no release slot, so under
//                          overload the NCO keeps releasing 31.25 Mhit/s.
//   So every hit of a coincidence is delivered unless the NCO was already 3 or more
//   hits behind when it arrived (j of its first hit > 24).
//   The decision needs n_prev - c, which is tracked with small relative counters:
//     q = n_prev - t_nco      (where the schedule stands, relative to now)
//     r = t_lead - c_loop     (c_loop: coarse time of the last hit the loop used)
//     x = t_lead - c          (this hit, saturated to 8 bits)
//   so the recurrence is a 9-bit compare and add, not 48 bits.
//     a      s_coarse - t_lead, two 24-bit halves (registered: i1)
//     a+1    x, saturated (i2)
//     a+2    constants that depend on x only, each one add or compare (i3)
//     a+3    decision (only the q/r recurrence here), registered
//     a+4    write into the FIFO; late pulse and count
//   so the earliest release is (a + 4) + 6 = a + 10 (ALIGN_LAT in loop_fixed.py).
//
//   POP_GAP = 8 is what keeps the control stages hazard-free: the loop's
//   state updates are visible 8 cycles after E (nco_top), so every hit sees the
//   state left by the previous one. It also limits sustained throughput to 31.25 Mhit/s.
//
// HEAD PIPELINE. A 48-bit compare doesn't fit in one cycle, so this runs ahead of
// the NCO: t_lead = t_nco + LEAD (4), so the decision is ready just in time.
//   z    head registered; j = t_lead - c, low half
//   z+1  j, high half
//   z+2  j flags: due, stale, late
//   z+3  decision (y)
//   z+4  pop, pop_fine = fine + j*T  (t_nco = t_lead[z] = n)
// A write in the cycle ending w is seen in w+1, so z = w+2 and n = w+6. The head
// computes j itself; admission predicted the same j, so the late flag here is
// only a backstop (it fires only for hits far out of order, e.g. c in the future).
//
// fine' must fit 18 bits: fine_ps + J_MAX * T < 2^18, so with J_MAX = 56 fine_ps must
// be below 38144 ps (block 1's is under 5 ns; 11 ns for the ideal LUT at code 511).
// nco_phase_det takes the 18th bit outside its multiplier.
//
// BACK-PRESSURE: admitted hits are at least POP_GAP apart and released at most
// D + J_MAX - 2 cycles after they are written, so in-spec input (c <= time_now)
// never holds more than 10 entries. s_ready (registered) still guards the FIFO:
// high while the FIFO plus the admission pipeline held at most 2^AW - 2 entries.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_align #(
    parameter integer COARSE_W = 48,
    parameter integer FINE_W   = 16,
    parameter integer TAG_W    = 4,
    parameter integer AW       = 4,
    parameter integer J_LOOP   = 24,
    parameter integer J_MAX    = 56,
    parameter integer W_STALE  = 48,
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
    output reg                 pop_stale = 1'b0,  // j > J_LOOP: deliver, don't update the loop
    output reg  [COARSE_W-1:0] pop_coarse = {COARSE_W{1'b0}},
    output reg  [FINE_W+1:0]   pop_fine = {(FINE_W+2){1'b0}},   // fine' = fine + j*T
    output reg  [TAG_W-1:0]    pop_tag = {TAG_W{1'b0}},
    output reg                 late = 1'b0,  // 1-cycle pulse per late hit
    output reg  [15:0]         late_cnt = 16'd0
);
    localparam integer DW = COARSE_W + FINE_W + TAG_W;
    localparam integer LO = 24;              // bits of a difference in the first cycle
    localparam integer HI = COARSE_W - LO;

    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW:0]   wr = 0, rd = 0;
    reg [3:0]    gap = 4'd0;                 // cycles until the next release is allowed

    wire empty = (wr == rd);

    // =========================================================================
    // Admission
    // =========================================================================
    // ---- a: differences, registered (i1) ----
    reg            i1_v = 1'b0, i2_v = 1'b0, i3_v = 1'b0;
    reg [DW-1:0]   i1_d = {DW{1'b0}}, i2_d = {DW{1'b0}}, i3_d = {DW{1'b0}};
    reg [LO:0]     i1_lo = {(LO+1){1'b0}};
    reg [HI-1:0]   i1_hi = {HI{1'b0}};
    wire           s_take = s_valid && s_ready;
    always @(posedge clk) begin
        i1_v  <= s_take && !rst;
        i1_d  <= {s_coarse, s_fine, s_tag};
        i1_lo <= {1'b0, t_lead[LO-1:0]} - {1'b0, s_coarse[LO-1:0]};
        i1_hi <= t_lead[COARSE_W-1:LO] - s_coarse[COARSE_W-1:LO];
    end

    // ---- a+1: x = t_lead[a] - c, saturated to [-128, 127] (i2) ----
    // the true high half is i1_hi - borrow: 0 (small positive) or -1 (small negative)
    wire       b1     = i1_lo[LO];
    wire       hi_z   = b1 ? (i1_hi == {{(HI-1){1'b0}}, 1'b1}) : (i1_hi == {HI{1'b0}});
    wire       hi_m1  = b1 ? (i1_hi == {HI{1'b0}})              : (i1_hi == {HI{1'b1}});
    wire       lo_pos = (i1_lo[LO-1:7] == {(LO-7){1'b0}});   // 0 .. 127
    wire       lo_neg = (i1_lo[LO-1:7] == {(LO-7){1'b1}});   // -128 .. -1 (as 2^24 + v)
    reg  signed [8:0] x2 = 9'sd0;
    always @(posedge clk) begin
        i2_v <= i1_v && !rst;
        i2_d <= i1_d;
        if (hi_z)       x2 <= lo_pos ? $signed({2'b00, i1_lo[6:0]}) : 9'sd127;
        else if (hi_m1) x2 <= lo_neg ? $signed({2'b11, i1_lo[6:0]}) : -9'sd128;
        else            x2 <= i1_hi[HI-1] ? -9'sd128 : 9'sd127;   // far out of range either way
    end

    // ---- a+2: constants of x alone (i3), straight from x (one add or compare each).
    // xd = t_lead[a+3] - c = x + 3. The write is at the end of a+4, so n >= a + 10 = c +
    // xd + 3, and with q = n_prev - t_nco[a+3]:  j = max(0, xd + 3, q + xd + 4).
    //   j > J_MAX  <=> x > J_MAX - 6  || q > J_MAX - 7 - x
    //   j > J_LOOP <=> x > J_LOOP - 6 || q > J_LOOP - 7 - x
    //   q next cycle, if admitted: n - t_nco[a+4] = j - xd + 3 = max(-x, 6, q + 7)
    //   c - c_loop = r - xd  (r = t_lead[a+3] - c_loop); r next cycle, if a loop hit: x + 4
    reg signed [9:0] k_max = 10'sd0, k_loop = 10'sd0, p_m7 = 10'sd0, p_q = 10'sd0;
    reg signed [9:0] xw = 10'sd0, x_r = 10'sd0;
    reg              h_max = 1'b0, h_loop = 1'b0;
    wire signed [9:0] x10 = x2;
    wire              p_big = x10 < -10'sd6;          // -x > 6
    always @(posedge clk) begin
        i3_v   <= i2_v && !rst;
        i3_d   <= i2_d;
        h_max  <= x10 > J_MAX - 6;
        h_loop <= x10 > J_LOOP - 6;
        k_max  <= (J_MAX - 7) - x10;
        k_loop <= (J_LOOP - 7) - x10;
        p_q    <= p_big ? -x10 : 10'sd6;
        p_m7   <= p_big ? (-10'sd7 - x10) : -10'sd1;
        xw     <= x10 + (W_STALE + 3);
        x_r    <= x10 + 10'sd4;
    end

    // ---- a+3: decision ----
    reg signed [9:0] q = -10'sd2;            // floor -2: no constraint left from n_prev
    reg signed [9:0] r = 10'sd255;           // saturates at 255: "long ago"
    wire late_hard = h_max  || (q > k_max);
    wire stale_j   = h_loop || (q > k_loop);
    wire admit     = i3_v && !late_hard && (!stale_j || (r <= xw));
    wire drop      = i3_v && !admit;
    // a+4: the write and the count, from registers
    reg          adm4 = 1'b0, drop4 = 1'b0;
    reg [DW-1:0] i4_d = {DW{1'b0}};
    always @(posedge clk) begin
        adm4  <= admit && !rst;
        drop4 <= drop && !rst;
        i4_d  <= i3_d;
    end
    always @(posedge clk) begin
        if (rst) begin
            q <= -10'sd2;
            r <= 10'sd255;
        end else begin
            q <= admit ? ((q > p_m7) ? q + 10'sd7 : p_q) : ((q > -10'sd2) ? q - 10'sd1 : -10'sd2);
            r <= (admit && !stale_j) ? x_r : ((r < 10'sd255) ? r + 10'sd1 : r);
        end
    end

    // ---- back-pressure: FIFO + admission pipeline ----
    reg s_ready_r = 1'b0;
    wire [AW:0] occ = wr - rd;               // AW+1 bits: wraps with the pointers
    wire [AW+1:0] occ_all = occ + i1_v + i2_v + i3_v + adm4;
    always @(posedge clk) s_ready_r <= ~rst && (occ_all <= (1 << AW) - 2);
    assign s_ready = s_ready_r;

    // =========================================================================
    // Head: release at t_nco = n
    // =========================================================================
    // ---- z: registered FIFO head ----
    reg [DW-1:0]        head = {DW{1'b0}};
    reg                 h_v = 1'b0, jl_v = 1'b0, jh_v = 1'b0, fl_v = 1'b0;
    wire [COARSE_W-1:0] head_c = head[DW-1 -: COARSE_W];
    wire [FINE_W-1:0]   head_f = head[TAG_W +: FINE_W];
    wire [TAG_W-1:0]    head_t = head[TAG_W-1:0];

    // ---- j = t_lead - c, modulo 2^COARSE_W, split in two cycles ----
    reg [LO:0]   j_lo = {(LO+1){1'b0}};
    reg [HI-1:0] t_hi = 0, c_hi = 0, j_hi = 0;
    reg [LO-1:0] j_lo_d = {LO{1'b0}};

    // ---- flags: negative (MSB set) = not yet due ----
    reg       f_due = 1'b0, f_late = 1'b0, f_stale = 1'b0;
    reg [5:0] j_f = 6'd0;

    wire rel = fl_v && f_due && (gap == 4'd0) && ~rst;

    // j * T_CLK_PS for j < 64: a small table, one LUT per bit
    reg [17:0] j_ps = 18'd0;
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
        f_due   <= ~j_hi[HI-1];
        f_late  <= (|j_hi) || (|j_lo_d[LO-1:6]) || (j_lo_d[5:0] > J_MAX);
        f_stale <= j_lo_d[5:0] > J_LOOP;
        j_f     <= j_lo_d[5:0];
        fl_v    <= jh_v & ~rel & ~rst;
        // z+3: the decision; registered, so pop is valid in cycle n
        pop        <= rel & ~f_late;
        pop_stale  <= f_stale;
        pop_coarse <= head_c;
        pop_fine   <= {2'b00, head_f} + j_ps;
        pop_tag    <= head_t;

        late <= 1'b0;
        if (adm4)
            mem[wr[AW-1:0]] <= i4_d;
        if (rst) begin
            wr <= 0; rd <= 0; gap <= 4'd0; late_cnt <= 16'd0; pop <= 1'b0;
        end else begin
            if (adm4) wr <= wr + 1'b1;
            if (drop4 || (rel && f_late)) begin
                late <= 1'b1;
                if (late_cnt != 16'hFFFF) late_cnt <= late_cnt + 1'b1;
            end
            if (rel) begin
                // a backstop-late hit never reaches the pipeline: no gap
                gap <= f_late ? 4'd0 : POP_GAP - 1;
                rd  <= rd + 1'b1;
            end else if (gap != 4'd0) begin
                gap <= gap - 1'b1;
            end
        end
    end
endmodule

`default_nettype wire
