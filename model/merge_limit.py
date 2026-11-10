#!/usr/bin/env python3
"""Five merged TDC channels against the NCO's hit-rate limit. SIMULATED.

The NCO releases at most one hit per POP_GAP = 8 clocks (31.25 Mhit/s). A hit that
would be released more than J_LOOP = 24 clocks after its own edge is dropped before
the FIFO (it costs no release slot), unless it comes within 48 clocks of the last
hit the loop used (a coincidence or a short burst): then it is delivered up to
J_MAX = 56 clocks late, without updating the loop (model/loop_fixed.py, class Admission). This script finds out what that costs for the receiver's
merged stream: 4 quantum channels plus 1 sync channel on one time base
(rtl/tdc/tdc_multi_top.v).

Path of every hit, in 250 MHz clocks:
  source slot -> detector dead time (per channel) -> capture edge c
  -> channel FIFO, 8 deep, written at c + CH_LAT (full: drop, counted in ch_drop)
  -> merger: head eligible once coarse_now >= c + MAXLAT + 2, smallest calibrated
     time first, one hit per clock, 1-deep output register, stalls on NCO full
  -> NCO admission, then FIFO -> released by the rule in model/loop_fixed.py.
Everything from the NCO FIFO on is model/loop_fixed.py itself, so lock and slips
are the fixed-point loop's. The merger part is a behavioural model of tdc_multi_top;
sim/nco/test_nco.py::test_five_channel_merge replays these streams into the RTL.

Run: python model/merge_limit.py     (writes docs/img/merge_limit_results.json)
"""

import json
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hitgen  # noqa: E402
import loop_fixed as lfx  # noqa: E402

CH_LAT = 9                  # capture edge -> channel FIFO (tdc_multi_top: 8 to 10)
MAXLAT = 12                 # tdc_multi_top parameter
CH_DEPTH = 8                # tdc_multi_top QAW = 3
NCO_DEPTH = 14              # nco_align: 2^AW - 2 usable (registered s_ready)
NUM_QUANTUM = 4


@dataclass
class Scenario:
    name: str
    f_src: float            # source slot rate
    p_quantum: float        # P(detection per slot), all 4 quantum detectors together
    sync_hz: float          # sync channel: pulses per second, on source slots
    jitter_ps: float = 200.0
    dark_ratio: float = 0.1 # dark counts / quantum signal
    hits: int = 60000       # roughly how many hits to generate
    burst_hz: float = 0.0   # all 5 channels fire in the same slot, this many times per second


SCENARIOS = [
    # Realistic: README's QKD count rates are kHz to MHz.
    Scenario("realistic: 100 MHz, 100 kcps + 100 kHz sync", 100e6, 1e-3, 100e3, hits=20000),
    Scenario("realistic: 100 MHz, 1 Mcps + 1 MHz sync", 100e6, 1e-2, 1e6),
    Scenario("realistic high: 500 MHz, 10 Mcps + 1 MHz sync", 500e6, 2e-2, 1e6, jitter_ps=50.0),
    # Toward and past the limit
    Scenario("stress: 100 MHz, 20 Mcps + 1 MHz sync", 100e6, 0.2, 1e6),
    Scenario("at the limit: 100 MHz, 30 Mcps + 1 MHz sync", 100e6, 0.3, 1e6),
    Scenario("over the limit: 100 MHz, 50 Mcps + 10 MHz sync", 100e6, 0.5, 10e6),
    # Worst case: every channel limited only by its own dead time
    Scenario("worst case: 500 MHz, p = 0.5, 10 MHz sync", 500e6, 0.5, 10e6, jitter_ps=50.0, dark_ratio=0.0, hits=80000),
    # All channels fire together: stray light, or a sync pulse coupling into the detectors
    Scenario("5-channel coincidences, 100 kHz, on 1 Mcps", 100e6, 1e-2, 1e5, burst_hz=1e5),
]


def make_channels(sc: Scenario, seed=1):
    """Arrival times per channel after each detector's own dead time.
    Returns (t_ps, channel, true slot), sorted in time, and the count lost to dead time."""
    rate = sc.p_quantum * sc.f_src + sc.sync_hz
    duration = sc.hits / rate
    ppm = 12.0
    # Dead time is applied per channel below, so none here.
    cfg = dict(f_src_hz=sc.f_src, ppm=ppm, jitter_ps=sc.jitter_ps, duration_s=duration,
               dead_min_ps=0.0, dead_max_ps=0.0)
    q = hitgen.generate(hitgen.StreamCfg(p_det=sc.p_quantum, dark_hz=sc.dark_ratio * sc.p_quantum * sc.f_src,
                                         seed=seed, **cfg))
    rng = np.random.default_rng(seed + 1000)
    t = [q["t_ps"]]
    ch = [rng.integers(0, NUM_QUANTUM, size=len(q["t_ps"]))]
    slot = [q["slot"]]
    if sc.sync_hz > 0:
        s = hitgen.generate(hitgen.StreamCfg(p_det=sc.sync_hz / sc.f_src, seed=seed + 1, t_offset_ps=q["t_off"],
                                             **{**cfg, "jitter_ps": 50.0}))
        t.append(s["t_ps"])
        ch.append(np.full(len(s["t_ps"]), NUM_QUANTUM))
        slot.append(s["slot"])
    if sc.burst_hz > 0:
        n = int(duration * sc.burst_hz)
        k = np.sort(rng.integers(0, int(duration * sc.f_src), size=n))
        tb = hitgen.slot_times(hitgen.StreamCfg(f_src_hz=sc.f_src, ppm=ppm), k.astype(np.float64), q["t_off"])
        for c in range(NUM_QUANTUM + 1):
            t.append(tb + rng.normal(0.0, sc.jitter_ps, size=n))
            ch.append(np.full(n, c))
            slot.append(k)
    t, ch, slot = np.concatenate(t), np.concatenate(ch), np.concatenate(slot)
    keep = np.zeros(len(t), bool)
    offered = len(t)
    for c in range(NUM_QUANTUM + 1):
        idx = np.nonzero(ch == c)[0]
        idx = idx[np.argsort(t[idx], kind="stable")]
        keep[idx[hitgen.dead_time_filter(t[idx], rng, 16750.0, 20000.0)]] = True
    t, ch, slot = t[keep], ch[keep], slot[keep]
    o = np.argsort(t, kind="stable")
    return t[o], ch[o], slot[o], offered - int(keep.sum()), duration


def merge(c, fine, ch, slot, cfg, n_ch=NUM_QUANTUM + 1):
    """Cycle-by-cycle: channel FIFOs, merger, NCO FIFO occupancy.
    Returns [lfx.Hit], true slots, per-channel drop counts, peak NCO FIFO level."""
    cq = [deque() for _ in range(n_ch)]
    fifo = deque()              # release times (in coarse_now clocks) of hits in the NCO FIFO
    drops = [0] * n_ch
    hits, truth = [], []
    out = None                  # merger's 1-deep output register
    i, N = 0, len(c)
    t = int(c[0] + CH_LAT)
    D = cfg.D
    adm = lfx.Admission(cfg)
    peak = 0
    t0 = int(c[0]) - 1000
    while i < N or out is not None or any(cq):
        while fifo and fifo[0] <= t:
            fifo.popleft()
        # NCO side of the handshake
        if out is not None and len(fifo) < NCO_DEPTH:
            cc, ff, chn, sl = out
            a = t - 1 - D       # t_nco in this cycle (nco_top: t_nco[x] = time_now[x-1] - D)
            n, kind = adm.offer(cc, a)
            # an admitted hit leaves at its release; a late one after the 3 admission stages
            fifo.append(t + 3 if kind == lfx.LATE else n + D + 1)
            peak = max(peak, len(fifo))
            hits.append(lfx.Hit(cc, ff, a, chn))
            truth.append(sl)
            out = None
        # merger picks for the next cycle
        if out is None:
            best, best_key = None, None
            for k, q in enumerate(cq):
                if q and t - q[0][0] >= MAXLAT + 2:
                    key = q[0][0] * lfx.T_CLK_PS - q[0][1]
                    if best is None or key < best_key:
                        best, best_key = k, key
            if best is not None:
                cc, ff, sl = cq[best].popleft()
                out = (cc, ff, best, sl)
        # channel FIFOs are written after the merger looks at them
        while i < N and int(c[i]) + CH_LAT <= t:
            k = int(ch[i])
            if len(cq[k]) < CH_DEPTH:
                cq[k].append((int(c[i]), int(fine[i]), int(slot[i])))
            else:
                drops[k] += 1
            i += 1
        if out is None and not any(cq) and i < N:
            t = max(t + 1, int(c[i]) + CH_LAT)     # nothing to do: jump to the next arrival
        else:
            t += 1
    return hits, truth, drops, peak, t0


def evaluate(sc: Scenario, seed=1, pop_gap=None):
    t, ch, slot, dead, duration = make_channels(sc, seed)
    c, fine = hitgen.to_timestamps(t, hitgen.T_CLK_PS)
    c = c + 2000
    o = np.argsort(c - fine / 4000.0, kind="stable")
    c, fine, ch, slot = c[o], fine[o], ch[o], slot[o]
    cfg = lfx.FixedCfg(fps_nom=lfx.fps_from_hz(sc.f_src), t_slot_ps=round(1e12 / sc.f_src))
    if pop_gap is not None:
        cfg.POP_GAP = pop_gap
    hits, truth, drops, peak, t0 = merge(c, fine, ch, slot, cfg)
    outs, m = lfx.run_hits(cfg, t0, 0, hits)
    delivered = sum(not o["late"] for o in outs)
    # slips: the slot offset must not change inside a run of consecutive locked outputs
    slips, runs, cur, first_lock = 0, 0, None, None
    in_run = False
    locked_sig = 0
    for idx, (o, tr) in enumerate(zip(outs, truth)):
        if o["late"]:
            continue
        if not o["locked"]:
            in_run, cur = False, None
            continue
        if first_lock is None:
            first_lock = idx
        if not in_run:
            runs, in_run = runs + 1, True
        if tr < 0:
            continue
        locked_sig += 1
        off = (o["slot"] - tr) & ((1 << 48) - 1)
        if cur is not None and off != cur:
            slips += 1
        cur = off
    offered = len(t) + dead
    return dict(name=sc.name, offered=offered, offered_rate=offered / duration, into_tdc=len(t),
                lost_dead=dead, ch_fifo_drop=int(sum(drops)), to_nco=len(hits), late=m.cnt["late"],
                delivered=delivered, nco_input_rate=len(hits) / duration, delivered_rate=delivered / duration,
                drop_pct_of_tdc=100.0 * (int(sum(drops)) + m.cnt["late"]) / max(1, len(t)),
                peak_fifo=peak, first_lock=first_lock, lock_runs=runs, slips=slips,
                locked_signal=locked_sig, unlocks=m.cnt["unlock"], locked_at_end=bool(outs[-1].get("locked")))


if __name__ == "__main__":
    results = []
    for sc in SCENARIOS:
        r = evaluate(sc)
        results.append(r)
        print(f"{r['name']:<50} offered {r['offered_rate']/1e6:7.2f} M/s  dead {r['lost_dead']:6d}  chFIFO {r['ch_fifo_drop']:6d}  "
              f"late {r['late']:6d}  ({r['drop_pct_of_tdc']:5.2f} %)  to NCO {r['delivered_rate']/1e6:6.2f} M/s  "
              f"peak FIFO {r['peak_fifo']:2d}  lock@{r['first_lock']}  runs {r['lock_runs']}  slips {r['slips']}  unlocks {r['unlocks']}", flush=True)
    out = Path(__file__).resolve().parents[1] / "docs" / "img" / "merge_limit_results.json"
    out.write_text(json.dumps(results, indent=1))
