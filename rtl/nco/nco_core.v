// =============================================================================
// nco_core.v  --  the NCO: phase accumulator and frequency register
// -----------------------------------------------------------------------------
//   phase  88 bit: 48-bit slot index . 40-bit fraction (2^-40 slot)
//   fps    signed 48 bit: frequency in slots per ps, 2^-56 slot/ps
//   fclk   per-clock increment = (fps * T_CLK_PS) >> 16, 2^-40 slot
//
// Storing the frequency in slots/ps (not slots/clock) is what lets the phase
// detector interpolate with one multiply: phase(hit) = P - fine_ps * fps.
//
// PIPELINED FOR 250 MHz. An 88-bit carry chain doesn't fit in 4 ns, so nothing
// here is wider than ~24 bits in one cycle:
//   x    correction strobe (corr_valid or ext_valid)
//   x+1  loop + external correction, low half
//   x+2  ... high half: J valid (cycle y)
//   y+1  I = fclk + J, segment 0; segment k one cycle later (carry registered)
//   y+2  P += I,       segment 0; segment k one cycle later
//   y+5  the top segment is done; lower ones are delayed to line up: `phase`
//
// Only the time at which things become visible changes, so the model stays exact:
//   correction strobed in cycle x: phase[x+7] += dphase, fps[x+4] += dfreq
//   fclk_model[t+7] = (fps[t] * 4000) >> 16, and phase[t+1] = phase[t] + fclk_model[t]
// "phase[t]" means the value of the output register when t_nco = t. Internally the
// accumulator runs ahead of it, which nothing outside can see.
//
// fclk: 4000/65536 = 125/2048, so fclk = floor(fps * 125 / 2^11). Two DSPs multiply
// the halves of fps by 125, and one add joins them.
//
// rst must last at least 8 cycles, longer than these pipelines (nco_top stretches it).
// =============================================================================
`timescale 1ns/1fs
`default_nettype none

module nco_core #(
    parameter [47:0]  FPS_NOM  = 48'd7205759403793,   // 100 MHz: round(1e-4 * 2^56)
    parameter integer T_CLK_PS = 4000                 // documentation: 4000 is built in
)(
    input  wire               clk,
    input  wire               rst,
    // tracking-loop corrections
    input  wire               loop_valid,
    input  wire signed [47:0] loop_dphi,     // 2^-40 slot
    input  wire signed [47:0] loop_df,       // 2^-56 slot/ps
    // external correction port (default unused)
    input  wire               ext_valid,
    input  wire signed [47:0] ext_dphase,    // 2^-40 slot
    input  wire signed [47:0] ext_dfreq,     // 2^-56 slot/ps
    // state
    output wire        [87:0] phase,
    output wire signed [47:0] fps,
    output reg  signed [43:0] fclk
);
    wire signed [63:0] nom_x4000 = $signed({16'd0, FPS_NOM}) * $signed(64'd4000);
    wire signed [43:0] fclk_nom  = nom_x4000[59:16];
    initial fclk = fclk_nom;            // the only initial value: Yosys rejects a second one

    // ---------------- x+1, x+2: merge loop and external corrections ----------------
    wire [47:0] ldp = (loop_valid & ~rst) ? loop_dphi  : 48'd0;
    wire [47:0] edp = (ext_valid  & ~rst) ? ext_dphase : 48'd0;
    wire [47:0] ldf = (loop_valid & ~rst) ? loop_df    : 48'd0;
    wire [47:0] edf = (ext_valid  & ~rst) ? ext_dfreq  : 48'd0;

    reg [24:0] jp_lo = 25'd0, jf_lo = 25'd0;
    reg [23:0] jp_a = 24'd0, jp_b = 24'd0, jf_a = 24'd0, jf_b = 24'd0;
    reg [47:0] j_p = 48'd0, j_f = 48'd0;
    always @(posedge clk) begin
        jp_lo <= {1'b0, ldp[23:0]} + {1'b0, edp[23:0]};
        jf_lo <= {1'b0, ldf[23:0]} + {1'b0, edf[23:0]};
        jp_a  <= ldp[47:24];  jp_b <= edp[47:24];
        jf_a  <= ldf[47:24];  jf_b <= edf[47:24];
        j_p   <= {jp_a + jp_b + jp_lo[24], jp_lo[23:0]};
        j_f   <= {jf_a + jf_b + jf_lo[24], jf_lo[23:0]};
    end

    // ---------------- frequency: two 24-bit segments ----------------
    reg [24:0] f_lo = {1'b0, FPS_NOM[23:0]};
    reg [23:0] f_hi = FPS_NOM[47:24];
    reg [23:0] f_lo_d = FPS_NOM[23:0];
    reg [23:0] jf_hi_d = 24'd0;
    always @(posedge clk) begin
        if (rst) begin
            f_lo   <= {1'b0, FPS_NOM[23:0]};
            f_hi   <= FPS_NOM[47:24];
            f_lo_d <= FPS_NOM[23:0];
        end else begin
            f_lo   <= {1'b0, f_lo[23:0]} + {1'b0, j_f[23:0]};
            f_hi   <= f_hi + jf_hi_d + f_lo[24];
            f_lo_d <= f_lo[23:0];
        end
        jf_hi_d <= j_f[47:24];
    end
    assign fps = {f_hi, f_lo_d};

    // ---------------- fclk = floor(fps * 125 / 2^11) ----------------
    reg         [30:0] m_lo = 31'd0, p_lo = 31'd0;
    reg signed  [31:0] m_hi = 32'sd0, p_hi = 32'sd0;
    wire signed [31:0] up = p_hi + $signed({25'd0, p_lo[30:24]});
    always @(posedge clk) begin
        m_lo <= fps[23:0] * 7'd125;
        m_hi <= $signed(fps[47:24]) * $signed(8'd125);
        p_lo <= m_lo;
        p_hi <= m_hi;
        fclk <= rst ? fclk_nom : {up[30:0], p_lo[23:11]};
    end

    // ---------------- I = fclk + J, four 22-bit segments, one per cycle ----------------
    wire [87:0] fx = {{44{fclk[43]}}, fclk};
    wire [87:0] jx = {{40{j_p[47]}}, j_p};
    reg  [22:0] i0 = 23'd0, i1 = 23'd0, i2 = 23'd0;
    reg  [21:0] i3 = 22'd0;
    reg  [21:0] f1 = 22'd0, j1 = 22'd0;      // inputs of segment 1, one cycle late
    reg  [43:0] f2 = 44'd0, j2 = 44'd0;      // inputs of segments 2-3 ...
    reg  [43:0] f3 = 44'd0, j3 = 44'd0;      // ... two cycles late
    reg  [21:0] f4 = 22'd0, j4 = 22'd0;      // inputs of segment 3, three cycles late
    always @(posedge clk) begin
        i0 <= {1'b0, fx[21:0]} + {1'b0, jx[21:0]};
        f1 <= fx[43:22];  j1 <= jx[43:22];
        f2 <= fx[87:44];  j2 <= jx[87:44];
        i1 <= {1'b0, f1} + {1'b0, j1} + i0[22];
        f3 <= f2;         j3 <= j2;
        i2 <= {1'b0, f3[21:0]} + {1'b0, j3[21:0]} + i1[22];
        f4 <= f3[43:22];  j4 <= j3[43:22];
        i3 <= f4 + j4 + i2[22];
    end

    // ---------------- P += I, one segment per cycle, then line the segments up ----------------
    reg [22:0] p0 = 23'd0, p1 = 23'd0, p2 = 23'd0;
    reg [21:0] p3 = 22'd0;
    reg [21:0] p0_d1 = 22'd0, p0_d2 = 22'd0, p0_d3 = 22'd0;
    reg [21:0] p1_d1 = 22'd0, p1_d2 = 22'd0;
    reg [21:0] p2_d1 = 22'd0;
    always @(posedge clk) begin
        if (rst) begin
            p0 <= 23'd0; p1 <= 23'd0; p2 <= 23'd0; p3 <= 22'd0;
            p0_d1 <= 22'd0; p0_d2 <= 22'd0; p0_d3 <= 22'd0;
            p1_d1 <= 22'd0; p1_d2 <= 22'd0; p2_d1 <= 22'd0;
        end else begin
            p0 <= {1'b0, p0[21:0]} + {1'b0, i0[21:0]};
            p1 <= {1'b0, p1[21:0]} + {1'b0, i1[21:0]} + p0[22];
            p2 <= {1'b0, p2[21:0]} + {1'b0, i2[21:0]} + p1[22];
            p3 <= p3 + i3 + p2[22];
            p0_d1 <= p0[21:0]; p0_d2 <= p0_d1; p0_d3 <= p0_d2;
            p1_d1 <= p1[21:0]; p1_d2 <= p1_d1;
            p2_d1 <= p2[21:0];
        end
    end
    assign phase = {p3, p2_d1, p1_d2, p0_d3};
endmodule

`default_nettype wire
