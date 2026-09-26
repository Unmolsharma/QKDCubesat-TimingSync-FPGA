"""tdc_top tests: detection, coarse/fine combination, dead time, pulse width,
back-pressure, and the code-density data collection.

Absolute-time reconstruction used throughout (see tdc_top.v):
    t_est = t_edge(coarse) - fine_ps
where t_edge(C) is the time of the clock edge after which the live counter
read C. The testbench learns that mapping once after reset.

Errors are always reported relative to the median error of the run. The
constant channel offset (tap-0 delay, LUT origin) is not something a TDC
measures. Only the spread matters.

ALL RESULTS ARE SIMULATED (behavioural carry-chain model).
"""

import os
import random
import statistics
import sys
from pathlib import Path

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from tdc_tb import (NS, PS, T_CLK, config_name, now_fs, out_dir, pulse_train,
                    sample, start_clock, val)

CALIBRATED = bool(os.environ.get("TDC_LUT_FILE"))
# Spread tolerance on (t_est - t_hit) around its median. Uncalibrated, the
# ideal LUT carries the chain's full INL; calibrated, only a few LSB remain.
TOL_PS = 90 if CALIBRATED else 250
W_OK = 12 * NS          # pulse width that satisfies W >= chain + T_clk


class TopTB:
    def __init__(self, dut):
        self.dut = dut
        self.beats = []
        self.fine_w = len(dut.u_hd.ev_fine)
        self.coarse_w = len(dut.coarse_now)
        self.ps_w = len(dut.m_cal) - self.coarse_w

    async def start(self):
        d = self.dut
        start_clock(d.clk)
        d.hit.value = 0
        d.m_ready.value = 1
        d.cal_we.value = 0
        d.cal_addr.value = 0
        d.cal_wdata.value = 0
        d.rst.value = 1
        await ClockCycles(d.clk, 5)
        d.rst.value = 0
        await ClockCycles(d.clk, 10)
        self.t_ref = await sample(d.clk)
        self.c_ref = val(d.coarse_now)
        cocotb.start_soon(self._monitor())
        await RisingEdge(d.clk)

    def t_edge(self, coarse: int) -> int:
        return self.t_ref + (coarse - self.c_ref) * T_CLK

    async def _monitor(self):
        d = self.dut
        while True:
            if not val(d.m_valid):
                await RisingEdge(d.m_valid)
                await ReadOnly()
            while val(d.m_valid):
                if val(d.m_ready):
                    self._record(val(d.m_raw), val(d.m_cal))
                await RisingEdge(d.clk)
                await ReadOnly()

    def _record(self, raw, cal):
        coarse = raw >> self.fine_w
        fine = raw & ((1 << self.fine_w) - 1)
        assert cal >> self.ps_w == coarse, "raw/cal coarse mismatch"
        fine_ps = cal & ((1 << self.ps_w) - 1)
        self.beats.append(dict(coarse=coarse, fine=fine, fine_ps=fine_ps,
                               t_est=self.t_edge(coarse) - fine_ps * PS, t_out=now_fs()))

    async def run_pulses(self, pulses, settle_cycles=30):
        await cocotb.start_soon(pulse_train(self.dut.hit, pulses))
        await ClockCycles(self.dut.clk, settle_cycles)

    def counters(self):
        d = self.dut
        return dict(short=val(d.short_cnt), sat=val(d.sat_cnt),
                    rej=val(d.rej_cnt), drop=val(d.drop_cnt))


def match(beats, hit_times, tol_fs):
    """Assign each output to the nearest true hit. Returns (errors_fs,
    matched_hit_indices). Fails on double-counting or unmatched outputs."""
    errs, used = [], set()
    for b in beats:
        i = min(range(len(hit_times)), key=lambda k: abs(b["t_est"] - hit_times[k]))
        e = b["t_est"] - hit_times[i]
        assert abs(e) < tol_fs, f"output {b} matches no hit (nearest error {e / PS:.0f} ps)"
        assert i not in used, f"hit {i} reported twice"
        used.add(i)
        errs.append(e)
    return errs, used


def spread_ok(errs_fs, tol_ps):
    med = statistics.median(errs_fs)
    worst = max(abs(e - med) for e in errs_fs) / PS
    return worst <= tol_ps, worst, med / PS


async def random_hits(tb, rng, n, width=W_OK, gap=(10 * NS, 26 * NS)):
    """n pulses with random, clock-uncorrelated arrival times, spaced so every
    one is outside the previous one's dead time."""
    t = now_fs() + 50 * NS
    pulses = []
    for _ in range(n):
        pulses.append((t, width))
        t += width + rng.randrange(*gap)
    await tb.run_pulses(pulses)
    return [p[0] for p in pulses]


# -----------------------------------------------------------------------------
@cocotb.test()
async def test_smoke(dut):
    """One hit -> exactly one valid timestamp, no error counters."""
    tb = TopTB(dut)
    await tb.start()
    t_hit = now_fs() + 37 * NS + 1234 * PS
    await tb.run_pulses([(t_hit, W_OK)], settle_cycles=40)

    assert len(tb.beats) == 1, f"expected 1 timestamp, got {len(tb.beats)}"
    b = tb.beats[0]
    err_ps = (b["t_est"] - t_hit) / PS
    lat_ns = (b["t_out"] - tb.t_edge(b["coarse"])) / 1e6
    dut._log.info("SIMULATED smoke: coarse=%d fine=%d fine_ps=%d  error=%.1f ps, "
                  "capture edge -> m_valid %.0f ns (%d clk)", b["coarse"], b["fine"],
                  b["fine_ps"], err_ps, lat_ns, round(lat_ns * 1e6 / T_CLK))
    assert abs(err_ps) < 300
    assert tb.counters() == dict(short=0, sat=0, rej=0, drop=0), tb.counters()


@cocotb.test()
async def test_boundary(dut):
    """Hits stepped through clock edges in 2 ps steps. A hit near an edge may
    decode as (N, small fine) or (N+1, fine ~ T). Both must give the same
    absolute time; an off-by-one-cycle error would show up as ~4000 ps."""
    tb = TopTB(dut)
    await tb.start()

    hits, n_split = [], {"small_fine": 0, "large_fine": 0}
    for _edge in range(6):
        t0 = await sample(dut.clk)               # a known clock edge
        await RisingEdge(dut.clk)
        edge = t0 + 20 * T_CLK
        pulses = []
        for i, delta_ps in enumerate(range(-80, 202, 2)):
            # one pulse per 10 clock periods so the hits are independent,
            # each placed delta_ps BEFORE its own clock edge
            e = edge + i * 10 * T_CLK
            pulses.append((e - delta_ps * PS, W_OK))
        start = len(tb.beats)
        await tb.run_pulses(pulses)
        hits += [p[0] for p in pulses]
        for b, (t_hit, _) in zip(tb.beats[start:], pulses):
            # captured by the edge right after the hit (N, small fine) or,
            # if tap 0 hadn't switched yet, by the following one (N+1, ~T)
            captured = tb.t_edge(b["coarse"])
            n_split["small_fine" if captured - t_hit < T_CLK // 2 else "large_fine"] += 1

    errs, used = match(tb.beats, hits, tol_fs=2000 * PS)
    assert len(used) == len(hits), f"{len(hits) - len(used)} hits lost"
    ok, worst, med = spread_ok(errs, TOL_PS)
    dut._log.info("SIMULATED boundary (%s, %s LUT): %d hits, (N, small fine)/(N+1, ~T) = %d/%d, "
                  "worst deviation %.1f ps (tol %d)", config_name(),
                  "calibrated" if CALIBRATED else "ideal", len(hits),
                  n_split["small_fine"], n_split["large_fine"], worst, TOL_PS)
    assert n_split["small_fine"] and n_split["large_fine"], "sweep didn't cross the edge"
    assert ok, f"boundary inconsistency: worst {worst:.1f} ps > {TOL_PS} ps"


@cocotb.test()
async def test_multihit(dut):
    """Hit trains at various spacings, including spacing inside the dead time.
    Checks: every output matches a distinct real hit (no double counting);
    widely spaced hits are always accepted; hits during dead time are
    rejected; every pulse is accounted for exactly once."""
    tb = TopTB(dut)
    await tb.start()
    rng = random.Random(7)
    W = 9 * NS

    # --- A: low gaps > T_clk (tap 0 always sees the gap) -> exact accounting
    pulses, t = [], now_fs() + 50 * NS
    for _ in range(300):
        pulses.append((t, W))
        t += W + rng.randrange(4500 * PS, 14000 * PS)   # rise-to-rise 13.5-23 ns
    c0, b0 = tb.counters(), len(tb.beats)
    await tb.run_pulses(pulses)
    c1 = tb.counters()
    beats = tb.beats[b0:]
    hits = [p[0] for p in pulses]
    errs, used = match(beats, hits, tol_fs=1000 * PS)
    acc, rej = len(beats), c1["rej"] - c0["rej"]
    short, sat = c1["short"] - c0["short"], c1["sat"] - c0["sat"]
    dut._log.info("SIMULATED multihit A: %d pulses -> %d accepted, %d rejected, "
                  "%d short, %d sat", len(pulses), acc, rej, short, sat)
    assert acc + rej + short + sat == len(pulses), "pulse accounting mismatch"
    assert short == 0 and sat == 0
    # Guaranteed acceptance: previous pulse fell >= chain(4.3 ns) + T_clk ago
    for i in range(1, len(pulses)):
        gap = pulses[i][0] - (pulses[i - 1][0] + W)
        if gap >= 8500 * PS:
            assert i in used, f"pulse {i} (gap {gap / PS:.0f} ps) wrongly rejected"
    ok, worst, _ = spread_ok(errs, TOL_PS)
    assert ok, f"timestamp spread {worst:.1f} ps"

    # --- B: re-triggers inside the line's reset window (gap < chain) must
    # never produce a second timestamp.
    pulses, t = [], now_fs() + 50 * NS
    for _ in range(100):
        pulses.append((t, W))
        g = rng.randrange(1000 * PS, 3500 * PS)          # 1-3.5 ns gap
        pulses.append((t + W + g, W))
        t += 2 * W + g + 30 * NS
    b0 = len(tb.beats)
    await tb.run_pulses(pulses)
    beats = tb.beats[b0:]
    hits = [p[0] for p in pulses]
    errs, used = match(beats, hits, tol_fs=1000 * PS)
    assert all(i % 2 == 0 for i in used), "a re-trigger inside dead time was accepted"
    assert len(used) == 100, f"first pulses lost: {100 - len(used)}"
    dut._log.info("SIMULATED multihit B: 100 pairs with 1-3.5 ns gaps -> "
                  "%d timestamps, 0 second-pulse timestamps", len(beats))

    # --- C: measure dead time vs rise-to-rise spacing for W = 12 ns
    rows, t = [], now_fs() + 50 * NS
    pairs = []
    for gap_ps in range(3000, 10250, 250):
        for _ in range(12):
            t += rng.randrange(0, T_CLK)                  # random phase
            pairs.append((t, t + W_OK + gap_ps * PS, gap_ps))
            t += 2 * W_OK + gap_ps * PS + 40 * NS
    pulses = [p for a, b_, _g in pairs for p in ((a, W_OK), (b_, W_OK))]
    b0 = len(tb.beats)
    await tb.run_pulses(pulses)
    hits = [p[0] for p in pulses]
    _, used = match(tb.beats[b0:], hits, tol_fs=1000 * PS)
    for k, (a, b_, gap_ps) in enumerate(pairs):
        rows.append((gap_ps, (b_ - a) / PS, (2 * k + 1) in used))
    acc_sp = [sp for _g, sp, ok_ in rows if ok_]
    rej_sp = [sp for _g, sp, ok_ in rows if not ok_]
    dead_lo, dead_hi = min(acc_sp), max(rej_sp)
    with open(out_dir() / "deadtime.csv", "w") as f:
        f.write("# SIMULATED dead time, W=12 ns: gap_ps,rise_to_rise_ps,second_accepted\n")
        for r in rows:
            f.write(f"{r[0]},{r[1]:.1f},{int(r[2])}\n")
        f.write(f"# min accepted spacing {dead_lo:.0f} ps, max rejected spacing {dead_hi:.0f} ps\n")
    dut._log.info("SIMULATED dead time (W=12 ns): second hit accepted from %.2f ns, "
                  "always accepted above %.2f ns (rise-to-rise)", dead_lo / 1000, dead_hi / 1000)


@cocotb.test()
async def test_short_pulse(dut):
    """Pulses shorter than the chain would make popcount measure the pulse
    width. They must be flagged (short_cnt) and never produce a timestamp.
    Grey-zone widths (chain < W < chain + T) may be dropped or kept, but
    anything kept must be accurate."""
    tb = TopTB(dut)
    await tb.start()
    rng = random.Random(5)

    def train(width, n=60):
        t, ps_ = now_fs() + 50 * NS, []
        for _ in range(n):
            t += rng.randrange(0, T_CLK)
            ps_.append((t, width))
            t += width + 30 * NS
        return ps_

    for w_ps in (500, 1000, 2000, 3000, 3900):
        c0, b0 = tb.counters(), len(tb.beats)
        pulses = train(w_ps * PS)
        await tb.run_pulses(pulses)
        c1 = tb.counters()
        n_out, n_short = len(tb.beats) - b0, c1["short"] - c0["short"]
        dut._log.info("SIMULATED short pulse W=%4d ps: %d pulses -> %d timestamps, "
                      "%d flagged short", w_ps, len(pulses), n_out, n_short)
        assert n_out == 0, f"W={w_ps} ps produced {n_out} (wrong) timestamps"
        assert n_short == len(pulses), f"W={w_ps} ps: only {n_short} flagged"

    for w_ps in (5000, 6500, 8000):
        c0, b0 = tb.counters(), len(tb.beats)
        pulses = train(w_ps * PS)
        await tb.run_pulses(pulses)
        c1 = tb.counters()
        beats = tb.beats[b0:]
        n_short = c1["short"] - c0["short"]
        errs, _ = match(beats, [p[0] for p in pulses], tol_fs=1000 * PS)
        dut._log.info("SIMULATED grey zone W=%4d ps: %d kept, %d flagged short",
                      w_ps, len(beats), n_short)
        assert len(beats) + n_short == len(pulses)
        if errs:
            ok, worst, _ = spread_ok(errs, TOL_PS)
            assert ok, f"kept grey-zone timestamp off by {worst:.0f} ps"


@cocotb.test()
async def test_backpressure(dut):
    """With m_ready low, the first timestamp is held, later hits are dropped
    and counted (overflow), and nothing is corrupted."""
    tb = TopTB(dut)
    await tb.start()
    dut.m_ready.value = 0
    t = now_fs() + 40 * NS
    pulses = [(t + k * 60 * NS, W_OK) for k in range(5)]
    await tb.run_pulses(pulses)
    assert val(dut.m_valid) == 1
    assert tb.counters()["drop"] == 4, tb.counters()
    assert not tb.beats
    await RisingEdge(dut.clk)
    dut.m_ready.value = 1
    await ClockCycles(dut.clk, 5)
    assert len(tb.beats) == 1
    errs, used = match(tb.beats, [p[0] for p in pulses], tol_fs=1000 * PS)
    assert used == {0}, "held timestamp is not the first hit"
    dut._log.info("SIMULATED back-pressure: 5 hits, ready low -> 1 held, 4 dropped+counted")


@cocotb.test()
async def test_top_coverage(dut):
    """Top-level coverage check: random hits must never saturate the line
    (sat_cnt). Expected to FAIL in the fast_corner configuration."""
    tb = TopTB(dut)
    await tb.start()
    n = 3000
    hits = await random_hits(tb, random.Random(4321), n)
    c = tb.counters()
    dut._log.info("SIMULATED top coverage (%s): %d hits -> %d timestamps, %d saturated",
                  config_name(), n, len(tb.beats), c["sat"])
    assert c["sat"] == 0, f"COVERAGE FAILURE: {c['sat']}/{n} hits saturated the line"
    assert len(tb.beats) == len(hits)


@cocotb.test()
async def test_code_density(dut):
    """Random, clock-uncorrelated hits for calibration (fine_codes.csv) plus a
    SECOND independent set with known true times for validation
    (validation.csv). The calibration is never validated on its own data."""
    tb = TopTB(dut)
    await tb.start()
    n_cal = int(os.environ.get("TDC_N_DENSITY", "50000"))
    n_val = int(os.environ.get("TDC_N_VALID", "20000"))
    out = out_dir()

    for name, n, seed in (("fine_codes", n_cal, 2024), ("validation", n_val, 777)):
        b0 = len(tb.beats)
        hits = await random_hits(tb, random.Random(seed), n)
        beats = tb.beats[b0:]
        assert len(beats) == n, f"{name}: {n - len(beats)} hits lost"
        c = tb.counters()
        assert c["sat"] == 0 and c["short"] == 0, c
        errs = [b["t_est"] - h for b, h in zip(beats, hits)]
        ok, worst, _ = spread_ok(errs, TOL_PS * 1.5)
        assert ok, f"{name}: in-order match failed, worst {worst:.0f} ps"

        path = out / f"{name}.csv"
        with open(path, "w") as f:
            f.write(f"# SIMULATED data, config={config_name()}, seed={seed}, "
                    f"T_clk_ps={T_CLK // PS}, lut={'calibrated' if CALIBRATED else 'ideal'}\n")
            if name == "fine_codes":
                f.write("# one raw fine code per line (calibrate.py input)\n")
                for b in beats:
                    f.write(f"{b['fine']}\n")
            else:
                f.write("# fine,tau_true_ps,fine_ps_rtl  (tau_true = t_edge(coarse) - t_hit)\n")
                for b, h in zip(beats, hits):
                    tau = (tb.t_edge(b["coarse"]) - h) / PS
                    f.write(f"{b['fine']},{tau:.3f},{b['fine_ps']}\n")
        dut._log.info("SIMULATED %s: %d hits written to %s", name, n, path)

    if CALIBRATED:
        # the RTL LUT must return exactly what the .mem file says
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
        from calibrate import read_mem
        lut = read_mem(os.environ["TDC_LUT_FILE"])
        bad = [b for b in tb.beats if lut[b["fine"]] != b["fine_ps"]]
        assert not bad, f"RTL LUT disagrees with .mem file: {bad[:3]}"
