// =============================================================================
// sat_emulator.v  --  stand-in for the satellite's pulsed source + detector
// -----------------------------------------------------------------------------
// For the M1 demo: an INDEPENDENT clock (in the lab: a waveform generator at
// ~100 MHz into a clock-capable pin; in simulation: a clock with a set ppm
// offset and drift) defines the source slots. Each slot is "detected" with a
// programmable probability, using a 32-bit xorshift generator
// (x ^= x<<13; x ^= x>>17; x ^= x<<5: period 2^32 - 1, three XOR stages):
//
//     detect = x < p_thr           P(detect) = p_thr / 2^32
//
// Why not a plain LFSR: consecutive LFSR states are shifted copies of each
// other, so "state < threshold" fires in clusters of adjacent slots (the
// first M1 rehearsal run had 706 suppressed adjacent slots for 1464 emitted
// pulses at p = 1e-2, instead of ~1 %). xorshift decorrelates successive
// draws at the same cost.
//
// A detected slot produces one pulse ~15 ns wide on `pulse` (rises on the
// slot's clock edge, falls on the falling edge 1.5 periods later: a DDR
// output register on hardware). That is wider than the TDC's input
// requirement (chain + 1 clock ~ 8.3 ns), like a SPAD module's output.
//
// Pulses from adjacent slots would overlap (15 ns > 10 ns slot), so a slot
// right after an emitted one is suppressed: there would be no new rising
// edge. Real detectors behave the same way (dead time). The suppressed and
// emitted counts are kept, so offered vs emitted rates can be reported.
//
// SIDE CHANNEL (for checking only): every emitted pulse also strobes
// emit_valid with its true slot index emit_slot (a 48-bit count of source
// clock cycles since reset). In simulation the testbench compares the NCO's
// slot numbers with these; on hardware it is unconnected (or could be
// counted).
//
// drop_all = 1 suppresses every pulse (signal gap test: unlock + re-acquire).
// All inputs are synchronised into the emulator's clock domain.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module sat_emulator #(
    parameter [31:0] RNG_SEED = 32'hACE1_2468
)(
    input  wire        clk_src,      // independent source clock (slot clock)
    input  wire        rst_async,    // asynchronous reset (synchronised here)
    input  wire [31:0] p_thr,        // detection probability * 2^32 (quasi-static)
    input  wire        drop_all,     // 1 = no pulses (signal gap)
    output wire        pulse,        // to the Pmod pin -> jumper -> TDC hit input
    output reg         emit_valid = 1'b0,
    output reg  [47:0] emit_slot = 48'd0,
    output reg  [31:0] emit_cnt = 32'd0,
    output reg  [31:0] supp_cnt = 32'd0
);
    // ---- reset and control synchronisers (emulator domain) ----
    (* ASYNC_REG = "TRUE" *) reg [2:0]  rst_s = 3'b111;
    (* ASYNC_REG = "TRUE" *) reg [1:0]  drop_s = 2'b00;
    (* ASYNC_REG = "TRUE" *) reg [31:0] thr_s1 = 32'd0, thr_s2 = 32'd0;
    always @(posedge clk_src or posedge rst_async)
        if (rst_async) rst_s <= 3'b111;
        else           rst_s <= {rst_s[1:0], 1'b0};
    wire rst = rst_s[2];
    always @(posedge clk_src) begin
        drop_s <= {drop_s[0], drop_all};
        thr_s1 <= p_thr;              // quasi-static: changes only from switches
        thr_s2 <= thr_s1;
    end

    // ---- slot counter and xorshift32 generator ----
    reg  [47:0] slot = 48'd0;
    reg  [31:0] rng = RNG_SEED;
    wire [31:0] x1 = rng ^ (rng << 13);
    wire [31:0] x2 = x1 ^ (x1 >> 17);
    wire [31:0] rng_next = x2 ^ (x2 << 5);

    reg  last_emit = 1'b0;
    wire sel  = (rng < thr_s2) && !drop_s[1];
    wire emit = sel && !last_emit;

    // pulse: high for 1.5 source periods from the emitting edge
    reg p_pos = 1'b0;     // set at the emitting posedge, cleared at the next posedge
    reg p_neg = 1'b0;     // copy of p_pos on the following negedge (adds 0.5 period)
    always @(posedge clk_src) begin
        if (rst) begin
            slot <= 48'd0; rng <= RNG_SEED; last_emit <= 1'b0; p_pos <= 1'b0;
            emit_valid <= 1'b0; emit_cnt <= 32'd0; supp_cnt <= 32'd0;
        end else begin
            slot       <= slot + 1'b1;
            rng        <= rng_next;
            last_emit  <= emit;
            p_pos      <= emit;
            emit_valid <= emit;
            if (emit) begin
                emit_slot <= slot;
                emit_cnt  <= emit_cnt + 1'b1;
            end
            if (sel && last_emit) supp_cnt <= supp_cnt + 1'b1;
        end
    end
    always @(negedge clk_src) p_neg <= p_pos & ~rst;

`ifdef SYNTHESIS
    // Hardware: an output DDR register gives clean, low-skew pulse edges.
    // SAME_EDGE mode: D1 is driven in the first half of the NEXT cycle, D2 in
    // the second half. D1 = p_pos | p_prev, D2 = p_pos gives 1.5 periods high,
    // one source cycle after the emitting edge (a constant offset, absorbed
    // like any other channel delay).
    reg p_prev = 1'b0;
    always @(posedge clk_src) p_prev <= p_pos;
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE")) u_oddr (
        .Q(pulse), .C(clk_src), .CE(1'b1), .D1(p_pos | p_prev), .D2(p_pos),
        .R(1'b0), .S(1'b0));
`else
    // Simulation: high from the emitting posedge until 1.5 periods later.
    assign pulse = p_pos | p_neg;
`endif
endmodule

`default_nettype wire
