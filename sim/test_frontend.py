"""Frontend tests: delay-line model + sync + popcount encoder (build step 1).

Toplevel: tdc_frontend. There's no hit detector yet, so the tests treat the
"trigger" sample as the first sample in which tap 0 is set, which is the same
rule tdc_hit_detect uses later.

Output alignment: frontend outputs seen just after clock edge c describe the
capture edge c-4 (LATENCY = 4). The sync-register contents seen after edge c
describe capture edge c-1.

ALL RESULTS ARE SIMULATED (behavioural carry-chain model).
"""

import random

import cocotb
from cocotb.triggers import ClockCycles

from tdc_tb import (HIT_W_DEFAULT, PS, T_CLK, config_name, edge_time,
                    out_dir, pulse_at, sample, start_clock, val)

LATENCY = 4


def n_taps(dut) -> int:
    return len(dut.u_sync.q)


def priority_code(code: int) -> int:
    """What a naive 'index of the highest 1' encoder would output."""
    return code.bit_length()


async def measure_one(dut, delta_fs: int, width_fs: int = HIT_W_DEFAULT):
    """Launch one hit `delta_fs` before a known clock edge and find the trigger
    sample. Returns dict(tau_ps, fine, full, prio, bubbles) or None."""
    t_e = await edge_time(dut.clk)
    t_hit = t_e + 2 * T_CLK - delta_fs
    cocotb.start_soon(pulse_at(dut.hit, t_hit, width_fs))

    recs = []                       # (t_edge, t0, fine, full, sync_code)
    for _ in range(14):
        t_c = await sample(dut.clk)
        recs.append((t_c, val(dut.t0), val(dut.fine), val(dut.full),
                     val(dut.u_sync.q)))
    await ClockCycles(dut.clk, 4)   # let the line drain fully

    for idx, (t_c, t0, fine, full, _) in enumerate(recs):
        if t0:
            t_capture = t_c - LATENCY * T_CLK
            code = recs[idx - (LATENCY - 1)][4]     # sync view of same sample
            n = n_taps(dut)
            ones = bin(code).count("1")
            return dict(tau_ps=(t_capture - t_hit) / PS, fine=fine, full=full,
                        prio=priority_code(code), bubbles=(priority_code(code) != ones),
                        popcount_check=ones, n=n)
    return None


@cocotb.test()
async def test_smoke_frontend(dut):
    """A hit produces a thermometer code whose popcount is plausible, and the
    line fills (full) and then drains back to zero."""
    start_clock(dut.clk)
    dut.hit.value = 0
    await ClockCycles(dut.clk, 10)

    r = await measure_one(dut, delta_fs=1500 * PS)
    assert r is not None, "hit never reached tap 0"
    assert r["fine"] == r["popcount_check"], "encoder != popcount of sync code"
    # ~1.5 ns before the edge at ~22 ps/tap -> roughly 68 taps
    assert 40 <= r["fine"] <= 100, f"implausible fine code {r['fine']}"
    dut._log.info("SIMULATED smoke: tau=%.1f ps -> fine=%d (priority enc would say %d)",
                  r["tau_ps"], r["fine"], r["prio"])

    # line must be back to zero once the pulse has left
    await ClockCycles(dut.clk, 2)
    assert val(dut.zero) == 1


@cocotb.test()
async def test_sweep_monotonic(dut):
    """Step the hit across one full clock period in 4 ps steps. Popcount must be
    monotonic in the true time-before-edge. For comparison, it also checks a
    naive priority encoder on the same samples: how many codes it can never
    produce (missing codes), which is resolution lost to bubbles."""
    start_clock(dut.clk)
    dut.hit.value = 0
    await ClockCycles(dut.clk, 10)

    step_ps = 4
    rows = []
    for delta_ps in range(step_ps, 4000 + 2 * step_ps, step_ps):
        r = await measure_one(dut, delta_fs=delta_ps * PS)
        assert r is not None
        rows.append(r)

    rows.sort(key=lambda r: r["tau_ps"])
    back_pop = [rows[i]["fine"] - rows[i + 1]["fine"] for i in range(len(rows) - 1)
                if rows[i + 1]["fine"] < rows[i]["fine"]]
    back_pri = [rows[i]["prio"] - rows[i + 1]["prio"] for i in range(len(rows) - 1)
                if rows[i + 1]["prio"] < rows[i]["prio"]]
    n_bub = sum(r["bubbles"] for r in rows)

    def missing(key):
        seen = {r[key] for r in rows}
        return len(set(range(min(seen), max(seen) + 1)) - seen)
    miss_pop, miss_pri = missing("fine"), missing("prio")

    with open(out_dir() / "sweep.csv", "w") as f:
        f.write("# SIMULATED sweep: tau_ps,popcount,priority\n")
        for r in rows:
            f.write(f"{r['tau_ps']:.3f},{r['fine']},{r['prio']}\n")

    dut._log.info("SIMULATED sweep (%s): %d points, %d had bubbles", config_name(),
                  len(rows), n_bub)
    dut._log.info("  popcount : %d backward steps (max %d codes), %d missing codes",
                  len(back_pop), max(back_pop, default=0), miss_pop)
    dut._log.info("  priority : %d backward steps (max %d codes), %d missing codes",
                  len(back_pri), max(back_pri, default=0), miss_pri)
    with open(out_dir() / "sweep_summary.txt", "w") as f:
        lines = [f"# SIMULATED sweep summary ({config_name()}), step {step_ps} ps",
                 f"points={len(rows)}", f"bubble_samples={n_bub}",
                 f"popcount_backward_steps={len(back_pop)}",
                 f"popcount_missing_codes={miss_pop}",
                 f"priority_backward_steps={len(back_pri)}",
                 f"priority_missing_codes={miss_pri}"]
        f.write("\n".join(lines) + "\n")

    assert n_bub > 0, "model produced no bubbles; the test wouldn't exercise anything"
    if config_name() == "nojitter":
        # With fixed thresholds popcount is monotonic by construction.
        assert not back_pop, f"popcount not monotonic: {back_pop[:10]}"
    else:
        # With per-sample clock jitter, two points closer together than the
        # jitter can swap. Allow only small local reversals.
        assert max(back_pop, default=0) <= 2, f"popcount reversal too large: {back_pop}"


@cocotb.test()
async def test_coverage(dut):
    """Random hit phases. A trigger sample that is all ones means the edge had
    already run off the end of the chain: the line does NOT cover the clock
    period, and that timestamp would be wrong. Any such sample is a FAILURE."""
    start_clock(dut.clk)
    dut.hit.value = 0
    await ClockCycles(dut.clk, 10)

    rng = random.Random(1234)
    n_hits, sat, missed, max_fine = 3000, 0, 0, 0
    n = n_taps(dut)
    for _ in range(n_hits):
        delta = rng.randrange(1, T_CLK + 1)
        r = await measure_one(dut, delta_fs=delta)
        if r is None:
            missed += 1
            continue
        max_fine = max(max_fine, r["fine"])
        if r["full"] or r["fine"] >= n:
            sat += 1

    with open(out_dir() / "coverage.txt", "w") as f:
        f.write(f"# SIMULATED coverage ({config_name()})\n")
        f.write(f"hits={n_hits}\nsaturated={sat}\nmissed={missed}\n"
                f"max_fine={max_fine}\ntaps={n}\n")

    dut._log.info("SIMULATED coverage (%s): %d hits, %d saturated, %d missed, "
                  "max fine %d of %d taps", config_name(), n_hits, sat, missed, max_fine, n)
    assert missed == 0
    assert sat == 0, (f"COVERAGE FAILURE: {sat}/{n_hits} trigger samples saturated -- "
                      f"the {n}-tap chain is shorter than T_clk + tap0 delay")
