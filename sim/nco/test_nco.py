"""Block 2 (NCO clock recovery) RTL tests.

Every test checks, hit by hit, that the RTL equals model/loop_fixed.py
(slot, phase error, residual, noise flag, lock flag), plus the final NCO
registers and counters (NcoTB.check_model). Behaviour against the true
source slots is checked separately (NcoTB.truth_check): zero slips while
locked. Runs are SHORT (milliseconds of simulated time); long-run statistics
live in model/loop_float.py. ALL RESULTS ARE SIMULATED.

Built twice by sim/run_tests.py: suite "nco" (100 MHz source, M1) and suite
"nco500" (500 MHz source, M2; a subset of the tests).
"""

import json

import numpy as np

import cocotb

from nco_tb import F_SRC, NcoTB, default_cfg, val  # also puts model/ on sys.path
import hitgen  # noqa: E402
import loop_fixed as lfx  # noqa: E402

T_SLOT_PS = 1e12 / F_SRC
RESULTS = {}
P_DEF = 2e6 / F_SRC          # 2 M offered photons/s at either source rate


def stream(**kw):
    base = dict(f_src_hz=F_SRC, ppm=0.0, p_det=0.02, jitter_ps=100.0 if F_SRC < 2e8 else 50.0,
                dark_hz=0.0, duration_s=0.5e-3, seed=1, tdc_rms_ps=11.7)
    base.update(kw)
    return hitgen.generate(hitgen.StreamCfg(**base))


async def run_case(dut, name, cfg=None, first=True, **kw):
    """Fresh reset, drive one stream, bit-exact check, truth check.
    first=False for the 2nd, 3rd... case inside one cocotb test."""
    tb = NcoTB(dut, cfg)
    await tb.start(clock=first)
    s = stream(**kw)
    await tb.drive_stream(s["t_ps"], s["slot"])
    m = await tb.check_model(name)
    tr = tb.truth_check()
    ppm = kw.get("ppm", 0.0)
    res = dict(hits=len(tb.outs), offered=s["offered"], accepted=s["accepted"], **tr,
               f_ppm=tb.f_ppm(), f_true_ppm=ppm, counters=dict(m.cnt))
    if tr["first_lock"] is not None:
        t_lock_us = (tb.hits[tr["first_lock"]].c - tb.hits[0].c) * 4e-3
        res["lock_time_us"] = t_lock_us
        res["lock_hits"] = tr["first_lock"] + 1
        lk = [o for o, t in zip(tb.outs, tb.truth) if o["locked"] and t >= 0 and not o["noise"]]
        if lk:
            res["rms_resid_ps"] = float(np.sqrt(np.mean([o["resid"] ** 2 for o in lk])))
    dut._log.info("SIMULATED %s @ %.0f MHz: %s", name, F_SRC / 1e6, res)
    RESULTS[name] = res
    return tb, res


def expect_lock(res, max_ppm_err=None):
    assert res["first_lock"] is not None, "never locked"
    assert res["slips"] == 0, f"{res['slips']} slips while locked"
    if max_ppm_err is not None:
        assert abs(res["f_ppm"] - res["f_true_ppm"]) < max_ppm_err, \
            f"frequency estimate {res['f_ppm']:+.3f} ppm vs true {res['f_true_ppm']:+.3f}"


# -----------------------------------------------------------------------------
@cocotb.test()
async def test_static_lock(dut):
    """Nominal source, 2 M photons/s offered: lock, then track with zero slips."""
    tb, res = await run_case(dut, "static_lock", ppm=0.0, p_det=P_DEF, duration_s=0.5e-3)
    expect_lock(res, max_ppm_err=3.0)
    save_results()


def save_results():
    """Merge this run's numbers into sim/out/<config>/nco_results_<f>MHz.json."""
    import os
    from nco_tb import ROOT
    out = ROOT / "sim" / "out" / os.environ.get("TDC_CONFIG", "default")
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"nco_results_{F_SRC / 1e6:.0f}MHz.json"
    old = json.loads(path.read_text()) if path.exists() else {}
    old.update(RESULTS)
    path.write_text(json.dumps(old, indent=1, default=float))


@cocotb.test()
async def test_ppm_sweep(dut):
    """Source offsets across the +/-100 ppm crystal-tolerance range."""
    for i, ppm in enumerate((-100.0, -37.5, 50.0, 100.0)):
        tb, res = await run_case(dut, f"ppm_{ppm:+g}", first=i == 0, ppm=ppm, p_det=P_DEF, duration_s=0.5e-3,
                                 seed=int(abs(ppm)) + 3)
        expect_lock(res, max_ppm_err=3.0)
    save_results()


@cocotb.test()
async def test_sparsity_sweep(dut):
    """Detection probability from 30 % (dead-time limited) down to 1e-3.
    Offered vs accepted rates show the TDC dead time. Lower rates are covered
    by model/loop_float.py (RTL runs stay short)."""
    for i, (p, dur) in enumerate(((0.3, 0.06e-3), (1e-2, 0.9e-3), (3e-3, 2.5e-3), (1e-3, 6.5e-3))):
        tb, res = await run_case(dut, f"p_{p:g}", first=i == 0, ppm=20.0, p_det=p, duration_s=dur, seed=11)
        res["offered_rate"], res["accepted_rate"] = res["offered"] / dur, res["accepted"] / dur
        dut._log.info("SIMULATED p=%g: offered %.3g /s, accepted %.3g /s (TDC dead time)",
                      p, res["offered_rate"], res["accepted_rate"])
        expect_lock(res)
    save_results()


@cocotb.test()
async def test_jitter_sweep(dut):
    """Gaussian timing jitter. 100 MHz: 50-500 ps. 500 MHz: 20-100 ps (500 ps
    at 500 MHz is 0.25 slot RMS: a physical limit, see README)."""
    jits = (50.0, 200.0, 500.0) if F_SRC < 2e8 else (20.0, 50.0, 100.0)
    for i, j in enumerate(jits):
        tb, res = await run_case(dut, f"jitter_{j:g}ps", first=i == 0, ppm=-10.0, p_det=P_DEF, jitter_ps=j,
                                 duration_s=0.6e-3, seed=5)
        expect_lock(res)
    save_results()


@cocotb.test()
async def test_dark_counts(dut):
    """Uniform dark counts at 50 % of the signal rate. Once acquired, the gate
    must flag about 1 - 2g = 75 % of them and almost no signal photons."""
    rate = P_DEF * F_SRC
    tb, res = await run_case(dut, "dark_50pct", ppm=30.0, p_det=P_DEF, dark_hz=0.5 * rate,
                             duration_s=0.8e-3, seed=8)
    expect_lock(res)
    lk = [(o, t) for o, t in zip(tb.outs, tb.truth) if o["locked"]]
    dark = [o for o, t in lk if t < 0]
    sig = [o for o, t in lk if t >= 0]
    rej_dark = sum(o["noise"] for o in dark) / max(1, len(dark))
    rej_sig = sum(o["noise"] for o in sig) / max(1, len(sig))
    dut._log.info("SIMULATED noise gate while locked: %.1f %% of %d dark counts flagged, "
                  "%.2f %% of %d signal photons flagged", 100 * rej_dark, len(dark),
                  100 * rej_sig, len(sig))
    RESULTS["dark_50pct"].update(dark_flagged=rej_dark, signal_flagged=rej_sig)
    assert 0.65 < rej_dark < 0.85, rej_dark
    assert rej_sig < 0.01, rej_sig
    save_results()


@cocotb.test()
async def test_drift_ramp(dut):
    """Linear frequency drift. 0.4 ppm/s (orbital Doppler rate) changes nothing
    in a 1 ms RTL run, so this uses an ACCELERATED 2000 ppm/s ramp (2 ppm over
    the run) to exercise the integrator; real ramps: model/loop_float.py."""
    tb, res = await run_case(dut, "ramp_2000ppm_s", ppm=20.0, ramp_ppm_s=2000.0, p_det=P_DEF,
                             duration_s=1.0e-3, seed=21)
    f_end = 20.0 + 2000.0 * 1.0e-3
    res["f_true_ppm"] = f_end
    dut._log.info("SIMULATED ramp: final NCO %.2f ppm, source %.2f ppm", res["f_ppm"], f_end)
    expect_lock(res, max_ppm_err=3.0)
    save_results()


@cocotb.test()
async def test_gap_reacquire(dut):
    """10 ms signal gap (dark counts continue): the loop must report the loss
    (unlock) and re-acquire when the signal returns. No slips inside any lock
    run; the slot offset may change across the reported unlock."""
    rate = P_DEF * F_SRC
    tb, res = await run_case(dut, "gap_10ms", ppm=-15.0, p_det=P_DEF, dark_hz=0.02 * rate,
                             duration_s=11.2e-3, gaps=[(0.5e-3, 10.5e-3)], seed=31)
    expect_lock(res)
    assert res["counters"]["unlock"] >= 1, "gap not detected"
    assert res["lock_runs"] >= 2, "did not re-acquire after the gap"
    assert tb.outs[-1]["locked"], "not locked at the end"
    save_results()


@cocotb.test()
async def test_coincident_and_late(dut):
    """Hits with the same coarse time (as from two channels) are released POP_GAP
    cycles apart with fine' = fine + j*T, exactly. Hits delivered so late that
    they would be released more than J_MAX cycles after their edge are dropped
    and counted as late."""
    from nco_tb import LAT
    tb = NcoTB(dut)
    await tb.start()
    s = stream(ppm=5.0, p_det=P_DEF, duration_s=0.3e-3, seed=41)
    base = await tb.drive_stream(s["t_ps"], s["slot"])
    c0 = base + int(s["t_ps"][-1] / 4000) + 200
    for k in range(20):                         # bursts of 3 simultaneous hits
        for _ in range(3):
            await tb.drive(c0 + 50 * k, 1000 + 37 * k, lat=LAT)
    n_late = 5
    for k in range(n_late):                     # delivered after t_nco > c + J_MAX
        await tb.drive(c0 + 1000 + 100 * k, 2000, lat=tb.cfg.D + tb.cfg.J_MAX + 5)
    m = await tb.check_model("coincident_late")
    assert m.cnt["late"] == n_late, m.cnt
    RESULTS["coincident_late"] = dict(late=m.cnt["late"], hits=len(tb.outs))
    save_results()


@cocotb.test()
async def test_coincidences(dut):
    """Double and five-fold clicks (several channels in the same slot): every hit is
    delivered with its own correct slot. Groups also come right behind another hit
    and two in a row. Bit-exact with the model, no late hit, no slip; the hits past
    J_LOOP go out as stale (output only, the loop ignores them)."""
    tb = NcoTB(dut)
    await tb.start()
    ppm, seed = 8.0, 71
    s = stream(ppm=ppm, p_det=P_DEF, duration_s=1.0e-3, seed=seed)
    scfg = hitgen.StreamCfg(f_src_hz=F_SRC, ppm=ppm)
    rng = np.random.default_rng(seed + 1)
    k_end = int(s["slot"][s["slot"] >= 0].max())
    t = [np.asarray(s["t_ps"])]
    slot = [np.asarray(s["slot"])]
    tag = [np.zeros(len(s["t_ps"]), int)]
    groups = []                                  # (true slot, size)
    period = 1e12 / F_SRC
    for g, k in enumerate(np.sort(rng.choice(np.arange(int(0.4 * k_end), k_end - 100), 60, replace=False))):
        parts = [(int(k), 5 if g % 2 == 0 else 2)]
        if g % 6 == 2:
            parts.append((int(k) - 1, 1))        # a lone hit one slot before: queue already busy
        if g % 6 == 4:
            parts.append((int(k) + int(round(25 * 4000 / period)), 2))   # a double click 25 clocks later
        for kk, sz in parts:
            tt = hitgen.slot_times(scfg, np.full(sz, float(kk)), s["t_off"]) + rng.normal(0.0, 100.0, sz)
            t.append(tt)
            slot.append(np.full(sz, kk))
            tag.append(np.arange(sz) if sz > 1 else np.array([7]))
            if sz > 1:
                groups.append((kk, sz))
    t, slot, tag = np.concatenate(t), np.concatenate(slot), np.concatenate(tag)
    o = np.argsort(t, kind="stable")
    t, slot, tag = t[o], slot[o], tag[o]
    c, f = hitgen.to_timestamps(t, hitgen.T_CLK_PS)
    await FallingEdgeNow(dut)
    base = val(dut.time_now) + 40
    last = None
    from nco_tb import LAT
    for ci, fi, si, gi in zip(c, f, slot, tag):
        ci = int(ci) + base
        when = ci + LAT if last is None else max(ci + LAT, last + 1)
        await tb.drive(ci, int(fi), truth=int(si), tag=int(gi), lat=when - ci)
        last = when
    m = await tb.check_model("coincidences")
    assert m.cnt["late"] == 0, m.cnt
    tr = tb.truth_check()
    assert tr["slips"] == 0 and tb.outs[-1]["locked"], tr
    outs, _ = lfx.run_hits(tb.cfg, tb.n0, tb.P0, tb.hits)
    # every member of every group delivered, locked, with slot = true slot + K
    k_off = {(o["slot"] - tt) % (1 << 48) for o, tt in zip(tb.outs, tb.truth) if o["locked"] and tt >= 0}
    assert len(k_off) == 1, k_off
    by_slot = {}
    for o, tt in zip(tb.outs, tb.truth):
        by_slot.setdefault(tt, []).append(o)
    for kk, sz in groups:
        got = by_slot[kk]
        assert len(got) >= sz and all(o["locked"] for o in got), (kk, sz, got)
    stale = sum(o.get("stale", False) for o in outs)
    jmax = max(o["j"] for o in outs if not o["late"])
    dut._log.info("SIMULATED coincidences: %d groups (%d five-fold, %d double) all delivered, %d hits "
                  "as stale, max j %d, %s", len(groups), sum(sz == 5 for _, sz in groups),
                  sum(sz == 2 for _, sz in groups), stale, jmax, tr)
    assert stale > 0
    RESULTS["coincidences"] = dict(groups=len(groups), stale=stale, max_j=jmax, **tr)
    save_results()


async def FallingEdgeNow(dut):
    from cocotb.triggers import FallingEdge
    await FallingEdge(dut.clk)


@cocotb.test()
async def test_ext_correction(dut):
    """External correction port: phase and frequency steps strobed while
    locked are applied exactly as the model says, and the loop absorbs them."""
    tb = NcoTB(dut)
    await tb.start()
    s = stream(ppm=12.0, p_det=P_DEF, duration_s=1.0e-3, seed=51)
    half = len(s["t_ps"]) // 2
    base = await tb.drive_stream(s["t_ps"][:half], s["slot"][:half])
    await tb.ext_correction(int(0.05 * 2 ** 40), int(0.5e-6 * tb.cfg.fps_nom))   # +0.05 slot, +0.5 ppm
    await tb.drive_stream(s["t_ps"][half:], s["slot"][half:], t_start_cycle=base)
    await tb.check_model("ext_correction")
    tr = tb.truth_check()
    dut._log.info("SIMULATED ext correction: %s", tr)
    assert tr["slips"] == 0 and tb.outs[-1]["locked"]
    RESULTS["ext_correction"] = tr
    save_results()


@cocotb.test()
async def test_backpressure(dut):
    """m_ready low drops results (counted) but never stalls or disturbs the
    loop: NCO registers still match the model exactly."""
    from cocotb.triggers import FallingEdge, RisingEdge
    import loop_fixed as lfx
    from nco_tb import sval
    tb = NcoTB(dut)
    await tb.start()
    s = stream(ppm=-3.0, p_det=P_DEF, duration_s=0.3e-3, seed=61)

    async def toggle():
        # change m_ready right after rising edges, so it is stable for a
        # whole cycle (what the monitor assumes)
        from cocotb.triggers import ClockCycles
        while True:
            await RisingEdge(dut.clk)
            dut.m_ready.value = 0
            await ClockCycles(dut.clk, 500)
            dut.m_ready.value = 1
            await ClockCycles(dut.clk, 1500)
    t = cocotb.start_soon(toggle())
    await tb.drive_stream(s["t_ps"], s["slot"])
    t.cancel()
    dut.m_ready.value = 1
    await tb.settle()
    await FallingEdge(dut.clk)
    N = val(dut.t_nco)
    outs, m = lfx.run_hits(tb.cfg, tb.n0, tb.P0, tb.hits)
    m.finish(N)
    P, fps, _ = m.registers_at(N)
    assert val(dut.tlm_phase) == P and sval(dut.tlm_fps, 48) == fps
    drops = val(dut.out_drop_cnt)
    assert drops > 0 and drops + len(tb.outs) == len(outs), (drops, len(tb.outs), len(outs))
    dut._log.info("SIMULATED back-pressure: %d results delivered, %d dropped+counted, "
                  "loop unaffected", len(tb.outs), drops)


@cocotb.test()
async def test_five_channel_merge(dut):
    """Five merged channels (model/merge_limit.py) replayed into nco_top: at stress
    rates and with 5-channel coincidences, late drops and every result match the
    model, and the merge model's own late count agrees with the RTL's."""
    import merge_limit as ml
    res = {}
    first = True
    for sc in (ml.Scenario("stress", 100e6, 0.2, 1e6, hits=3000),
               ml.Scenario("coincidences", 100e6, 1e-2, 1e5, burst_hz=1e5, hits=1500)):
        tb = NcoTB(dut)
        await tb.start(clock=first)
        first = False
        t, ch, slot, dead, _ = ml.make_channels(sc, seed=3)
        c, fine = hitgen.to_timestamps(t, hitgen.T_CLK_PS)
        hits, truth, drops, peak, _ = ml.merge(c + 2000, fine, ch, slot, tb.cfg)
        shift = val(dut.time_now) + 40 - hits[0].c
        for h, tr in zip(hits, truth):
            await tb.drive(h.c + shift, h.fine, truth=tr, tag=h.tag, lat=h.a + tb.cfg.D + 1 - h.c)
        m = await tb.check_model(f"five_channel_{sc.name}")
        # the merge model's release rule, on its own arrival cycles
        adm, expect = lfx.Admission(tb.cfg), 0
        for h in hits:
            expect += adm.offer(h.c, h.a)[1] == lfx.LATE
        assert m.cnt["late"] == expect, (m.cnt["late"], expect)
        res[sc.name] = dict(hits=len(hits), late=m.cnt["late"], peak_fifo=peak, ch_fifo_drop=sum(drops))
        dut._log.info("SIMULATED five channels, %s: %s", sc.name, res[sc.name])
    RESULTS["five_channel_merge"] = res
    save_results()
