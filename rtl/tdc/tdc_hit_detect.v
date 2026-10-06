// =============================================================================
// tdc_hit_detect.v  --  one timestamp per hit, dead time, pulse-width checks
// -----------------------------------------------------------------------------
// Input is one aligned sample per clock from tdc_frontend:
//   t0 (tap 0 set), zero (line empty), full (line saturated), fine, coarse.
//
// TRIGGER
//   A hit is recognised in the first sample where tap 0 turns on after the
//   line was seen completely empty. The fine code of THAT sample is the
//   measurement: it counts how far the edge had travelled when the clock
//   sampled it.
//
// WHY WAIT FOR "FULL" BEFORE EMITTING (CONFIRM state)
//   The popcount only measures arrival time if the hit is still high while
//   its edge travels the whole line. If the pulse is shorter than the chain
//   (~4.2 ns), the line holds a travelling band of 1s and popcount measures
//   the PULSE WIDTH, not the arrival time. That is a silently wrong
//   timestamp. So a candidate is held until a later sample shows the line
//   completely full (every tap 1), which proves the pulse stayed high for
//   longer than the chain delay. If tap 0 drops before that happens, the
//   candidate is discarded and `short_cnt` counts it.
//   With the documented input requirement (pulse width >= chain + T_clk,
//   ~8.3 ns; SPAD modules give 10-30 ns) a full sample always arrives within
//   2 cycles, so this costs 1-2 cycles of latency and never drops a good hit.
//   Pulses in the grey zone chain < W < chain + T may be dropped or kept
//   depending on phase. Kept ones are correct; dropping is the safe failure.
//
// SATURATION (coverage failure)
//   If the trigger sample is already all ones, the edge ran past the end of
//   the chain before the clock sampled it: the line is SHORTER than
//   T_clk + tap-0 delay, and the true time is unknown. Such hits are
//   dropped and counted in `sat_cnt`. Any non-zero sat_cnt on hardware means
//   the chain doesn't cover the period (see README, "chain coverage").
//
// DEAD TIME / RE-TRIGGER REJECTION
//   After a hit, nothing is accepted until the line has been seen completely
//   empty again, meaning the pulse has ended and its falling edge has run
//   off the end of the chain. A new rising edge before that (tap 0 goes
//   0->1 while busy) is rejected and counted in `rej_cnt`. So dead time is
//   about  pulse_width + chain_delay + (0..1) T_clk.
//
// STATE NOTES
//   PENDING covers a bubble at the very start of the line: a skewed tap 1/2
//   can switch on one sample before tap 0. If tap 0 comes on in the next
//   sample it is an ordinary (late-detected) hit. The next clock period's
//   code is still correct, because the coarse count moves by one as well.
//   If tap 0 never comes on, a very short pulse slipped through tap 0
//   between samples, and it is counted as short.
//
// Counters are 16-bit and saturate. They are for monitoring/telemetry, and
// the UART status frame reports them.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_hit_detect #(
    parameter integer FINE_W      = 9,
    parameter integer COARSE_W    = 32,
    parameter integer CONFIRM_CYC = 3      // max samples to wait for "full"
)(
    input  wire                clk,
    input  wire                rst,
    // aligned frontend sample
    input  wire [FINE_W-1:0]   fine,
    input  wire                t0,
    input  wire                zero,
    input  wire                full,
    input  wire [COARSE_W-1:0] coarse,
    // one event per accepted hit
    output reg                 ev_valid = 1'b0,
    output reg  [COARSE_W-1:0] ev_coarse = {COARSE_W{1'b0}},
    output reg  [FINE_W-1:0]   ev_fine = {FINE_W{1'b0}},
    // status
    output wire                dead,
    output reg                 short_evt = 1'b0,   // 1-cycle pulses
    output reg                 sat_evt   = 1'b0,
    output reg                 rej_evt   = 1'b0,
    output reg  [15:0]         short_cnt = 16'd0,
    output reg  [15:0]         sat_cnt   = 16'd0,
    output reg  [15:0]         rej_cnt   = 16'd0
);
    localparam [1:0] S_IDLE    = 2'd0,   // line empty, armed
                     S_PENDING = 2'd1,   // activity seen but tap 0 not yet
                     S_CONFIRM = 2'd2,   // candidate held, waiting for "full"
                     S_WAIT0   = 2'd3;   // dead: waiting for the line to empty

    reg [1:0]          state = S_WAIT0;  // must see an empty line after reset
    reg                t0_prev = 1'b0;
    reg [2:0]          wait_cnt = 3'd0;
    reg [COARSE_W-1:0] cand_coarse = {COARSE_W{1'b0}};
    reg [FINE_W-1:0]   cand_fine = {FINE_W{1'b0}};

    assign dead = (state == S_CONFIRM) || (state == S_WAIT0);

    always @(posedge clk) begin
        ev_valid  <= 1'b0;
        short_evt <= 1'b0;
        sat_evt   <= 1'b0;
        rej_evt   <= 1'b0;
        t0_prev   <= t0;

        if (rst) begin
            state   <= S_WAIT0;
            t0_prev <= 1'b0;
        end else begin
            case (state)
            S_IDLE, S_PENDING:
                if (t0) begin
                    if (full) begin
                        sat_evt <= 1'b1;             // edge beyond the line
                        state   <= S_WAIT0;
                    end else begin
                        cand_coarse <= coarse;
                        cand_fine   <= fine;
                        wait_cnt    <= 3'd0;
                        state       <= S_CONFIRM;
                    end
                end else if (state == S_PENDING) begin
                    short_evt <= 1'b1;               // passed tap 0 unseen
                    state     <= S_WAIT0;
                end else if (!zero) begin
                    state <= S_PENDING;
                end

            S_CONFIRM:
                if (full) begin
                    ev_valid  <= 1'b1;
                    ev_coarse <= cand_coarse;
                    ev_fine   <= cand_fine;
                    state     <= S_WAIT0;
                end else if (!t0 || wait_cnt == CONFIRM_CYC - 1) begin
                    short_evt <= 1'b1;               // never filled the line
                    state     <= S_WAIT0;
                end else begin
                    wait_cnt <= wait_cnt + 1'b1;
                end

            S_WAIT0: begin
                if (t0 && !t0_prev)
                    rej_evt <= 1'b1;                 // re-trigger during dead time
                if (zero)
                    state <= S_IDLE;
            end
            endcase
        end

        // saturating event counters
        if (rst) begin
            short_cnt <= 16'd0; sat_cnt <= 16'd0; rej_cnt <= 16'd0;
        end else begin
            if (short_evt && short_cnt != 16'hFFFF) short_cnt <= short_cnt + 1'b1;
            if (sat_evt   && sat_cnt   != 16'hFFFF) sat_cnt   <= sat_cnt   + 1'b1;
            if (rej_evt   && rej_cnt   != 16'hFFFF) rej_cnt   <= rej_cnt   + 1'b1;
        end
    end
endmodule

`default_nettype wire
