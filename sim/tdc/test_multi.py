"""tdc_multi_top tests: NUM_CH channels on one time base, merged stream.

Built twice by sim/run_tests.py: "multi" (NUM_CH = 5: 4 quantum + 1 sync)
and "multi1" (NUM_CH = 1, the block 1 configuration through the merger).

Checks
  * every output is a real hit on the right channel, none lost or duplicated
  * the merged stream is sorted by measured time  t = coarse*T - fine_ps
  * channel-to-channel skew: identical simultaneous hits on all channels;
    the per-channel mean offset is what a lab calibration would store
  * per-channel drop counters under a burst the merger cannot absorb

Each channel's behavioural delay line has its own random tap delays
(SEED + 101*ch), like separately placed chains. ALL RESULTS ARE SIMULATED.
"""

import json
import random
import statistics

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from tdc_tb import NS, PS, T_CLK, now_fs, out_dir, pulse_at, sample, start_clock, val

W = 12 * NS


class MultiTB:
    def __init__(self, dut):
        self.dut = dut
        self.n = len(dut.hit)
        self.ps_w = len(dut.m_fine_ps)
        self.out = []

    async def start(self):
        d = self.dut
        start_clock(d.clk)
        d.hit.value = 0
        d.m_ready.value = 1
        d.cal_we.value = 0
        d.cal_ch.value = 0
        d.cal_addr.value = 0
        d.cal_wdata.value = 0
        d.rst.value = 1
        await ClockCycles(d.clk, 5)
        d.rst.value = 0
        await ClockCycles(d.clk, 10)
        self.t_ref = await sample(d.clk)
        self.c_ref = val(d.coarse_now)
        self._hits = [0] * self.n
        cocotb.start_soon(self._mon())
        await RisingEdge(d.clk)

    def t_edge(self, c):
        return self.t_ref + (c - self.c_ref) * T_CLK

    async def _mon(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            await ReadOnly()
            if val(d.m_valid) and val(d.m_ready):
                c, fps = val(d.m_coarse), val(d.m_fine_ps)
                self.out.append(dict(ch=val(d.m_ch), c=c, fine_ps=fps, fine=val(d.m_fine),
                                     t=self.t_edge(c) - fps * PS))

    def drive(self, ch, t_rise):
        """Pulse on one channel's hit line (other bits untouched)."""
        async def go():
            dt = t_rise - now_fs()
            from cocotb.triggers import Timer
            await Timer(dt, unit="fs")
            self._hits[ch] = 1
            self.dut.hit.value = sum(b << i for i, b in enumerate(self._hits))
            await Timer(W, unit="fs")
            self._hits[ch] = 0
            self.dut.hit.value = sum(b << i for i, b in enumerate(self._hits))
        return cocotb.start_soon(go())

    def counters(self, name):
        v = val(getattr(self.dut, name))
        return [(v >> (16 * i)) & 0xFFFF for i in range(self.n)]


def save(key, res):
    p = out_dir() / "multi_results.json"
    old = json.loads(p.read_text()) if p.exists() else {}
    old[key] = res
    p.write_text(json.dumps(old, indent=1))


@cocotb.test()
async def test_multi_random(dut):
    """Independent random hits on every channel. Every output must be one
    real hit on the right channel, and the stream must be time-ordered."""
    tb = MultiTB(dut)
    await tb.start()
    rng = random.Random(3)
    n_per = 150
    t0 = now_fs() + 100 * NS
    truth = []
    tasks = []
    for ch in range(tb.n):
        t = t0 + rng.randrange(0, 40 * NS)
        for _ in range(n_per):
            truth.append((ch, t))
            tasks.append(tb.drive(ch, t))
            t += W + rng.randrange(14 * NS, 120 * NS)    # beyond the dead time
    for tk in tasks:
        await tk
    await ClockCycles(dut.clk, 60)

    assert len(tb.out) == len(truth), f"{len(tb.out)} outputs for {len(truth)} hits"
    used = set()
    errs = []
    for o in tb.out:
        cands = [(abs(o["t"] - t), i) for i, (ch, t) in enumerate(truth) if ch == o["ch"]]
        e, i = min(cands)
        assert e < 400 * PS, f"output {o} matches no hit on ch{o['ch']} (nearest {e / PS:.0f} ps)"
        assert i not in used
        used.add(i)
        errs.append(o["t"] - truth[i][1])
    ts = [o["t"] for o in tb.out]
    inversions = sum(1 for a, b in zip(ts, ts[1:]) if b < a)
    assert inversions == 0, f"{inversions} out-of-order outputs"
    res = dict(channels=tb.n, hits=len(truth), outputs=len(tb.out), inversions=inversions,
               drops=tb.counters("ch_drop"), short=tb.counters("ch_short"), sat=tb.counters("ch_sat"))
    dut._log.info("SIMULATED multi random (%d ch): %s", tb.n, res)
    save(f"random_{tb.n}ch", res)


@cocotb.test()
async def test_multi_skew(dut):
    """The SAME pulse on every channel at once (one detector split N ways).
    The per-channel mean of (t_measured - t_true) is the channel offset a lab
    calibration would store. The behavioural model has no pad/routing skew,
    so here the offsets come only from each chain's own tap delays: large
    with the ideal LUT (config default), small once every channel has its
    own code-density LUT (config multi_calibrated)."""
    tb = MultiTB(dut)
    await tb.start()
    rng = random.Random(9)
    n = 400
    t = now_fs() + 100 * NS
    times = []
    for _ in range(n):
        t += W + rng.randrange(20 * NS, 60 * NS)
        times.append(t)
    async def drive_all():
        from cocotb.triggers import Timer
        for tt in times:
            await Timer(tt - now_fs(), unit="fs")
            dut.hit.value = (1 << tb.n) - 1
            await Timer(W, unit="fs")
            dut.hit.value = 0
    await cocotb.start_soon(drive_all())
    await ClockCycles(dut.clk, 60)

    assert len(tb.out) == n * tb.n, (len(tb.out), n * tb.n)
    per = {ch: [] for ch in range(tb.n)}
    for o in tb.out:
        k = min(range(n), key=lambda i: abs(o["t"] - times[i]))
        per[o["ch"]].append((o["t"] - times[k]) / PS)
    mean = {ch: statistics.mean(v) for ch, v in per.items()}
    rms = {ch: statistics.pstdev(v) for ch, v in per.items()}
    ref = mean[0]
    skew = {ch: mean[ch] - ref for ch in mean}
    # Ties: the N copies of one pulse must come out together, sorted by time
    groups = [tb.out[i:i + tb.n] for i in range(0, len(tb.out), tb.n)]
    assert all(len({g["ch"] for g in grp}) == tb.n for grp in groups), "copies of a pulse not adjacent"
    # Per pulse: d_ch = t_ch - t_0 (same physical instant). Its mean is the
    # channel skew a lab calibration stores; its spread is the relative jitter.
    # Hold-out check of the offset-table method: estimate the offsets on the
    # even pulses, subtract them on the odd pulses -> the residual mean skew
    # must vanish (only the spread remains).
    by_pulse = []
    for grp in groups:
        t = {g["ch"]: g["t"] for g in grp}
        by_pulse.append({ch: (t[ch] - t[0]) / PS for ch in range(tb.n)})
    d_rms = {ch: statistics.pstdev([p[ch] for p in by_pulse]) for ch in range(tb.n)}
    est = {ch: statistics.mean([p[ch] for p in by_pulse[0::2]]) for ch in range(tb.n)}
    resid = {ch: statistics.mean([p[ch] - est[ch] for p in by_pulse[1::2]]) for ch in range(tb.n)}
    import os
    calibrated = bool(os.environ.get("TDC_MULTI_LUTS"))
    res = dict(channels=tb.n, pulses=n, luts="per-channel calibrated" if calibrated else "ideal",
               mean_offset_ps=mean, rms_ps=rms, skew_vs_ch0_ps=skew,
               max_abs_skew_ps=max(abs(v) for v in skew.values()),
               diff_vs_ch0_rms_ps=d_rms,
               residual_skew_after_offset_table_ps=resid,
               max_abs_residual_ps=max(abs(v) for v in resid.values()))
    dut._log.info("SIMULATED inter-channel skew: %s", res)
    save(f"skew_{tb.n}ch_{'calibrated' if calibrated else 'ideal'}", res)
    # what calibration must deliver: a stable offset per channel (removable by
    # an offset table) and a small per-pulse spread
    # (200 hold-out pulses: the mean of d has ~1.5 ps (calibrated) / ~3.5 ps
    # (ideal LUT) statistical error; with the ideal LUT the INL also makes the
    # offset depend slightly on where in the clock period the pulse falls)
    assert res["max_abs_residual_ps"] < (5 if calibrated else 12), "offset table does not remove the skew"
    assert max(d_rms.values()) < (35 if calibrated else 80), d_rms


@cocotb.test()
async def test_multi_backpressure(dut):
    """The consumer stalls (m_ready low) while every channel keeps firing:
    the per-channel FIFOs fill, and further hits are DROPPED AND COUNTED per
    channel (never overwritten). What does come out is valid and in time
    order, and every hit is accounted for: output + FIFO drop + dead-time
    reject."""
    tb = MultiTB(dut)
    await tb.start()
    rng = random.Random(17)
    t0 = now_fs() + 100 * NS
    truth, tasks = [], []
    for ch in range(tb.n):
        t = t0 + rng.randrange(0, 30 * NS)
        for _ in range(40):
            truth.append((ch, t))
            tasks.append(tb.drive(ch, t))
            t += W + rng.randrange(14 * NS, 40 * NS)
    dut.m_ready.value = 0                       # consumer stalled for ~1 us
    await ClockCycles(dut.clk, 250)
    dut.m_ready.value = 1
    for tk in tasks:
        await tk
    await ClockCycles(dut.clk, 80)
    drops = tb.counters("ch_drop")
    rej = tb.counters("ch_rej")
    ts = [o["t"] for o in tb.out]
    inversions = sum(1 for a, b in zip(ts, ts[1:]) if b < a)
    for o in tb.out:                            # every output is a real hit
        e = min(abs(o["t"] - t) for ch, t in truth if ch == o["ch"])
        assert e < 400 * PS, o
    res = dict(channels=tb.n, hits=len(truth), outputs=len(tb.out), fifo_drops=drops,
               deadtime_rejects=rej, inversions=inversions)
    dut._log.info("SIMULATED back-pressure: %s", res)
    save(f"backpressure_{tb.n}ch", res)
    assert sum(drops) > 0, "FIFOs never filled: test did not exercise the drop path"
    assert inversions == 0
    assert len(tb.out) + sum(drops) + sum(rej) == len(truth), "hit accounting mismatch"


@cocotb.test()
async def test_multi_code_density(dut):
    """Per-channel calibration data: random, clock-uncorrelated hits on every
    channel, written as "ch,fine" (input for calibrate.py --per-channel) and
    an INDEPENDENT validation set "ch,fine,tau_true_ps"."""
    import os
    tb = MultiTB(dut)
    await tb.start()
    n_cal = int(os.environ.get("TDC_N_MULTI", "10000"))
    for name, n, seed in (("multi_fine_codes", n_cal, 2025), ("multi_validation", n_cal // 4, 778)):
        rng = random.Random(seed)
        b0 = len(tb.out)
        t0 = now_fs() + 100 * NS
        truth, tasks = [], []
        for ch in range(tb.n):
            t = t0 + rng.randrange(0, 40 * NS)
            for _ in range(n):
                truth.append((ch, t))
                t += W + rng.randrange(10 * NS, 26 * NS)
        # one driver per channel, so pulses are scheduled in time order
        async def drive_ch(ch):
            from cocotb.triggers import Timer
            for c, t in truth:
                if c != ch:
                    continue
                await Timer(t - now_fs(), unit="fs")
                tb._hits[ch] = 1
                dut.hit.value = sum(b << i for i, b in enumerate(tb._hits))
                await Timer(W, unit="fs")
                tb._hits[ch] = 0
                dut.hit.value = sum(b << i for i, b in enumerate(tb._hits))
        ds = [cocotb.start_soon(drive_ch(ch)) for ch in range(tb.n)]
        for d in ds:
            await d
        await ClockCycles(dut.clk, 60)
        outs = tb.out[b0:]
        assert len(outs) == len(truth), (len(outs), len(truth))
        per = {ch: sorted(t for c, t in truth if c == ch) for ch in range(tb.n)}
        idx = {ch: 0 for ch in range(tb.n)}
        path = out_dir() / f"{name}.csv"
        with open(path, "w") as f:
            f.write(f"# SIMULATED multi-channel data ({tb.n} ch), seed={seed}\n")
            f.write("# ch,fine" + (",tau_true_ps" if "validation" in name else "") + "\n")
            for o in outs:                       # outputs are time-ordered; channels in order
                t_true = per[o["ch"]][idx[o["ch"]]]
                idx[o["ch"]] += 1
                fine = o["fine"]
                if "validation" in name:
                    tau = (tb.t_edge(o["c"]) - t_true) / PS
                    f.write(f"{o['ch']},{fine},{tau:.3f}\n")
                else:
                    f.write(f"{o['ch']},{fine}\n")
        dut._log.info("SIMULATED %s: %d hits on %d channels -> %s", name, len(outs), tb.n, path)
