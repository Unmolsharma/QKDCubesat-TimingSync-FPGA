"""sat_emulator unit test: is it a fair stand-in for a sparse photon source?

Checks, at p = 0.1 and p = 0.01 (P(detect) per slot):
  * selection rate = p (binomial tolerance, 5 sigma)
  * ADJACENT-SLOT INDEPENDENCE: P(sel[k] and sel[k+1]) = p^2. The first M1
    rehearsal used a plain LFSR, whose consecutive states are shifted copies
    of each other: "state < threshold" then fired in clusters (706 suppressed
    adjacent slots for 1464 emitted at p = 1e-2). This test catches that.
  * emitted fraction = p / (1 + p) (a slot right after an emitted one is
    suppressed: its pulse would overlap the previous one)
  * pulse width = 1.5 source periods (15 ns at 100 MHz)
  * the side-channel slot index equals the emitting slot
  * drop_all suppresses every pulse
ALL RESULTS ARE SIMULATED.
"""

import json
import math
import os
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge, Timer
from cocotb.utils import get_sim_time

ROOT = Path(__file__).resolve().parents[2]


def val(s):
    return int(s.value)


async def collect(dut, n):
    """Per-slot record of (sel, emit_valid, emit_slot, slot) for n slots."""
    sel, emit = [], []
    for _ in range(n):
        await RisingEdge(dut.clk_src)
        await ReadOnly()
        sel.append(val(dut.sel))
        emit.append((val(dut.emit_valid), val(dut.emit_slot), val(dut.slot)))
    await FallingEdge(dut.clk_src)                     # leave the ReadOnly phase
    return sel, emit


@cocotb.test()
async def test_emulator_statistics(dut):
    Clock(dut.clk_src, 10, unit="ns", impl="gpi").start()
    dut.drop_all.value = 0
    dut.p_thr.value = 0
    dut.rst_async.value = 1
    await Timer(50, unit="ns")
    dut.rst_async.value = 0
    await ClockCycles(dut.clk_src, 10)
    res = {}
    for p, n in ((0.1, 40000), (0.01, 120000)):
        dut.p_thr.value = int(p * 2 ** 32)
        await ClockCycles(dut.clk_src, 5)               # through the synchroniser
        sel, emit = await collect(dut, n)
        ns = sum(sel)
        pairs = sum(1 for a, b in zip(sel, sel[1:]) if a and b)
        ne = sum(e for e, _, _ in emit)
        p_hat = ns / n
        pp_hat = pairs / (n - 1)
        e_hat = ne / n
        e_exp = p / (1 + p)
        sd_p = math.sqrt(p * (1 - p) / n)
        sd_pp = math.sqrt(p * p * (1 - p * p) / n)
        res[f"p={p:g}"] = dict(slots=n, p_hat=p_hat, pair_rate=pp_hat, pair_expected=p * p,
                               emitted_frac=e_hat, emitted_expected=e_exp)
        dut._log.info("SIMULATED emulator p=%g: P(sel) %.5f (exp %.5f), P(sel,sel next) %.6f "
                      "(exp %.6f), emitted %.5f (exp %.5f)", p, p_hat, p, pp_hat, p * p, e_hat, e_exp)
        assert abs(p_hat - p) < 5 * sd_p, "selection rate wrong"
        assert abs(pp_hat - p * p) < 5 * sd_pp + 2 / n, "adjacent slots are correlated (clustering)"
        assert abs(e_hat - e_exp) < 5 * sd_p, "emitted fraction wrong"
        # side channel: emit_slot is the slot counter value of the emitting edge
        for i, (e, es, s) in enumerate(emit):
            if e:
                assert es == s - 1, f"slot index {es} vs counter {s}"

    # pulse width: rising to falling edge of `pulse`
    widths = []
    for _ in range(20):
        await RisingEdge(dut.pulse)
        t0 = get_sim_time(unit="ps")
        await FallingEdge(dut.pulse)
        widths.append(get_sim_time(unit="ps") - t0)
    res["pulse_width_ps"] = sorted(set(widths))
    dut._log.info("SIMULATED emulator pulse widths: %s ps", sorted(set(widths)))
    assert all(w == 15000 for w in widths), widths

    # gap: no pulses at all
    dut.drop_all.value = 1
    await ClockCycles(dut.clk_src, 5)
    c0 = val(dut.emit_cnt)
    await ClockCycles(dut.clk_src, 5000)
    assert val(dut.emit_cnt) == c0, "pulses during drop_all"
    out = ROOT / "sim" / "out" / os.environ.get("TDC_CONFIG", "default")
    out.mkdir(parents=True, exist_ok=True)
    (out / "emulator_stats.json").write_text(json.dumps(res, indent=1))
