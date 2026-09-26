// =============================================================================
// tdc_delay_line.v  --  tapped delay line + first capture stage
// -----------------------------------------------------------------------------
// WHAT A TAPPED-DELAY-LINE TDC DOES
//   The asynchronous hit edge is launched into a long chain of fast, fixed
//   delay elements (the carry chain). At the next system-clock edge every tap
//   is sampled by a flip-flop at the same moment. Taps the edge has already
//   passed read '1', taps it hasn't reached read '0'. That gives a
//   "thermometer code" 111..1100..00, and the number of 1s tells us how long
//   before the clock edge the hit arrived, at a resolution of one tap delay
//   (~20 ps) instead of one clock period (4 ns).
//
// WHY THE CARRY CHAIN
//   CARRY4 is the only dedicated, hard-wired, uniformly placed fast path in the
//   Artix-7 fabric. Each CARRY4 has four taps (CO[3:0]), and COUT cascades
//   straight up to the CARRY4 in the slice above, with no general routing in
//   between. So the delays are short (~tens of ps), repeatable, and placement
//   is deterministic.
//
// WHY THE CAPTURE FFs SIT IN THE SAME SLICE
//   Each CO[i] is registered by the slice's own AFF/BFF/CFF/DFF (via the "CY"
//   input of the FF D-mux). Using those FFs keeps tap-to-FF routing identical
//   and minimal, so the bins are set by the carry chain and not by random
//   routing. (Placement is enforced in constraints/tdc_chain_place.tcl.)
//
// ARTIX-7 NOTE: CYINIT, NOT CI
//   The spec said "hit drives CI of the first CARRY4". On 7-series, CI is a
//   dedicated cascade pin that can only be driven by the COUT of the CARRY4
//   directly below. General routing (like our hit net) can only enter through
//   CYINIT (the slice's AX pin). So the first CARRY4 gets CYINIT = hit and
//   CI = 0; every later CARRY4 gets CI = previous CO[3].
//
// TWO IMPLEMENTATIONS
//   `ifdef SYNTHESIS : real CARRY4 + FDRE primitives (Vivado defines SYNTHESIS
//                      during synth_design; build.tcl also passes it).
//   else             : a behavioural model for Icarus. The CARRY4 simulation
//                      model has zero delay, so simulating it tells you
//                      nothing about timing. The model instead gives each tap
//                      a sampling threshold (ps) and computes directly which
//                      taps the edge had passed at each clock edge.
//
// BEHAVIOURAL MODEL (simulation only -- NOT a hardware prediction)
//   Tap delay d[i] = TAP_NOM_PS * pattern[i%4] * (1 + sigma*N(0,1)), where
//     pattern = {2.00, 0.35, 1.05, 0.60}  (mean 1.0)
//       - position 0 includes the COUT->CIN hop between slices, which gives
//         the "ultra-wide" bins published for 7-series carry chains
//       - position 1 is very fast, which gives near-zero-width bins
//     ~4% of taps are additionally shrunk to 5% (more near-zero bins).
//   The delays are then rescaled so that mean(d) == TAP_NOM_PS exactly, which
//   makes the total chain length N*TAP_NOM_PS (192*22 = 4224 ps by default).
//   Per-tap sampling threshold thr[i] = sum(d[0..i]) + skew[i]
//     skew[i] ~ N(0, SKEW_SIGMA_PS) is fixed per tap. It stands in for clock
//     skew / FF setup-time differences along the column. Adjacent thresholds
//     can swap order, which gives BUBBLES (e.g. 1110100..) exactly like on
//     real silicon.
//   JITTER_PS: per-sample common-mode Gaussian jitter (clock + hit path).
//   All randomness comes from SEED, so a given SEED always gives the same chain.
//   Not modelled: metastability, PVT drift, supply noise, crosstalk.
//
// COVERAGE WARNING (read this)
//   A hit is first seen at the sample where tap 0 has just turned on, so the
//   largest time the line must represent is T_clk + thr[0] (~4.04 ns). The
//   chain is 4224 ps nominal, which leaves only ~4-5% margin. A faster process
//   corner or a colder die makes the carry chain faster (shorter), and then the
//   line can SATURATE (all ones) before the edge arrives. The coverage test
//   runs a "fast corner" build (TAP_NOM_PS=20) to show this failure on purpose.
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module tdc_delay_line #(
    parameter integer NUM_CARRY4    = 48,     // 48 x 4 = 192 taps; <= 50 fits one clock region
    // ---- behavioural-model-only parameters (ignored by synthesis) ----
    parameter real    TAP_NOM_PS    = 22.0,   // mean tap delay
    parameter real    TAP_SIGMA_PCT = 15.0,   // random per-tap delay spread (%)
    parameter real    SKEW_SIGMA_PS = 6.0,    // fixed per-FF sampling skew (bubbles)
    parameter real    JITTER_PS     = 4.0,    // per-sample common-mode jitter (rms)
    parameter integer SEED          = 1
)(
    input  wire                     clk,
    input  wire                     hit,      // ASYNCHRONOUS to clk, by design
    output wire [4*NUM_CARRY4-1:0]  taps_q    // thermometer code, 1st register stage
);
    localparam integer N = 4 * NUM_CARRY4;

`ifdef SYNTHESIS
    // ------------------------------------------------------------------------
    // Hardware: CARRY4 chain. S=1111/DI=0000 turns each carry mux into a
    // pass-through, so CO[k] is just the hit edge delayed by k+1 mux stages.
    // ------------------------------------------------------------------------
    wire [N-1:0] co;

    genvar g, b;
    generate
        for (g = 0; g < NUM_CARRY4; g = g + 1) begin : g_c4
            wire ci_w, cyinit_w;
            if (g == 0) begin : g_first
                assign ci_w     = 1'b0;
                assign cyinit_w = hit;          // fabric enters through CYINIT/AX
            end else begin : g_rest
                assign ci_w     = co[4*g-1];    // dedicated COUT->CIN cascade
                assign cyinit_w = 1'b0;
            end

            (* DONT_TOUCH = "TRUE" *)
            CARRY4 u_carry4 (
                .CO     (co[4*g +: 4]),
                .O      (),
                .CI     (ci_w),
                .CYINIT (cyinit_w),
                .DI     (4'b0000),
                .S      (4'b1111)
            );

            // Capture FFs. ASYNC_REG tells Vivado this FF samples an
            // asynchronous signal (MTBF analysis, no SRL extraction, no
            // replication). DONT_TOUCH stops opt_design from merging or
            // removing any of them.
            for (b = 0; b < 4; b = b + 1) begin : g_ff
                (* DONT_TOUCH = "TRUE", ASYNC_REG = "TRUE" *)
                FDRE #(.INIT(1'b0)) u_cap (
                    .Q  (taps_q[4*g+b]),
                    .C  (clk),
                    .CE (1'b1),
                    .R  (1'b0),
                    .D  (co[4*g+b])
                );
            end
        end
    endgenerate

`else
    // ------------------------------------------------------------------------
    // Simulation: threshold model (see header).
    // ------------------------------------------------------------------------
    localparam integer HIST = 8;          // hit edges remembered

    real    thr_ps [0:N-1];               // sampling threshold of each tap (ps)
    real    thr_max_ps;
    // Gaussian samples below use $dist_normal(seed, 0, 1000)/1000.0, i.e.
    // N(0,1) at 1e-3 resolution ($dist_normal only returns integers).
    real    hist_t [0:HIST-1];            // hit edge times (ps), [0] = newest
    reg     hist_v [0:HIST-1];            // hit level after that edge
    integer hist_n;
    integer seed_dly, seed_jit;
    reg     [N-1:0] q;

    assign taps_q = q;

    // Hit level at absolute time t (ps), from the edge history.
    function level_at;
        input real t;
        integer k;
        reg found;
        begin
            level_at = 1'b0;
            found    = 1'b0;
            for (k = 0; k < HIST; k = k + 1)
                if (!found && k < hist_n && hist_t[k] <= t) begin
                    level_at = hist_v[k];
                    found    = 1'b1;
                end
            // Older than everything remembered: the level before the oldest
            // edge (edges alternate), or 0 if we haven't seen HIST edges yet.
            if (!found)
                level_at = (hist_n < HIST) ? 1'b0 : ~hist_v[HIST-1];
        end
    endfunction

    // ---- build the chain (once) ----
    real    d  [0:N-1];
    real    pattern [0:3];
    integer i, k;
    real    sum, scale, cum, r;
    reg [8*256-1:0] dump_name;
    integer fd;

    initial begin
        pattern[0] = 2.00; pattern[1] = 0.35; pattern[2] = 1.05; pattern[3] = 0.60;
        seed_dly = SEED;
        seed_jit = SEED * 7919 + 17;
        sum = 0.0;
        for (i = 0; i < N; i = i + 1) begin
            r = 1.0 + (TAP_SIGMA_PCT / 100.0) * ($dist_normal(seed_dly, 0, 1000) / 1000.0);
            if (r < 0.1) r = 0.1;
            d[i] = TAP_NOM_PS * pattern[i % 4] * r;
            if (({$random(seed_dly)} % 100) < 4)
                d[i] = d[i] * 0.05;                     // near-zero bin
            sum = sum + d[i];
        end
        scale = (N * TAP_NOM_PS) / sum;                 // force mean == nominal
        cum = 0.0;
        thr_max_ps = 0.0;
        for (i = 0; i < N; i = i + 1) begin
            cum = cum + d[i] * scale;
            thr_ps[i] = cum + SKEW_SIGMA_PS * ($dist_normal(seed_dly, 0, 1000) / 1000.0);
            if (thr_ps[i] < 1.0) thr_ps[i] = 1.0;
            if (thr_ps[i] > thr_max_ps) thr_max_ps = thr_ps[i];
        end

        hist_n = 0;
        for (k = 0; k < HIST; k = k + 1) begin hist_t[k] = 0.0; hist_v[k] = 1'b0; end
        q = {N{1'b0}};

        // Optional: dump the model's true thresholds so the analysis can compare
        // code-density estimates against ground truth.
        if ($value$plusargs("TDC_MODEL_DUMP=%s", dump_name)) begin
            fd = $fopen(dump_name, "w");
            $fdisplay(fd, "# tdc_delay_line model: tap,threshold_ps (SIMULATED)");
            $fdisplay(fd, "# TAP_NOM_PS=%0f SIGMA_PCT=%0f SKEW_PS=%0f JITTER_PS=%0f SEED=%0d",
                      TAP_NOM_PS, TAP_SIGMA_PCT, SKEW_SIGMA_PS, JITTER_PS, SEED);
            for (i = 0; i < N; i = i + 1)
                $fdisplay(fd, "%0d,%0.4f", i, thr_ps[i]);
            $fclose(fd);
        end
    end

    // ---- record hit edges (async) ----
    always @(hit) begin : rec
        for (k = HIST-1; k > 0; k = k - 1) begin
            hist_t[k] = hist_t[k-1];
            hist_v[k] = hist_v[k-1];
        end
        hist_t[0] = $realtime * 1000.0;               // ns -> ps
        hist_v[0] = (hit === 1'b1);
        if (hist_n < HIST) hist_n = hist_n + 1;
    end

    // ---- sample all taps on the clock edge ----
    // tap i reads the hit level at time (t_edge + jitter - thr[i]).
    always @(posedge clk) begin : samp
        real    now, jit;
        integer j;
        reg [N-1:0] nq;
        now = $realtime * 1000.0;
        jit = (JITTER_PS > 0.0) ? JITTER_PS * ($dist_normal(seed_jit, 0, 1000) / 1000.0) : 0.0;
        if (hist_n == 0 || (now + jit - thr_max_ps - 1.0) > hist_t[0]) begin
            // No edge inside the line's window: every tap sees the same level.
            nq = {N{(hist_n != 0) && hist_v[0]}};
        end else begin
            for (j = 0; j < N; j = j + 1)
                nq[j] = level_at(now + jit - thr_ps[j]);
        end
        q <= nq;
    end
`endif

endmodule

`default_nettype wire
