"""Photon-arrival stream generator shared by the loop models and the testbenches.

Everything produced here is a SIMULATED input stream, not measured data.

What the stream models
----------------------
* A pulsed source with nominal rate f_src (100 MHz for M1, 500 MHz for M2).
  Relative to our 250 MHz system clock its rate is off by `ppm` and drifts
  linearly by `ramp_ppm_s` (orbital Doppler during a pass is up to about
  +/-25 ppm, changing by up to ~0.4 ppm/s near zenith for a ~500 km orbit).
* Every slot independently produces a detection with probability `p_det`
  (sparse: 0.5 down to 1e-5). Slots are drawn as geometric gaps, so very
  sparse streams cost nothing to generate.
* Each detection gets Gaussian timing jitter (detector + source + TDC), RMS
  `jitter_ps`.
* Dark counts: a Poisson process of rate `dark_hz`, uniform in time.
* Gaps: optional time windows in which the source is off (cloud, pointing
  loss); dark counts continue.
* Dead time: the TDC channel cannot accept a new rising edge until the line
  has emptied (block 1: accepted from 16.75 ns, always above 20.0 ns for 12 ns
  pulses). A rising edge that arrives while the channel is busy is lost AND
  keeps the channel busy, because its 12 ns pulse holds the line full. The
  generator returns both the offered and the accepted streams so the rate
  loss can be reported.

Time is in picoseconds as float64 measured from the start of the stream. The
true slot index of a detection is the integer k of the slot it came from
(dark counts get -1).
"""

from dataclasses import dataclass, field

import numpy as np

T_CLK_PS = 4000.0          # 250 MHz system clock


@dataclass
class StreamCfg:
    f_src_hz: float = 100e6
    ppm: float = 0.0               # source rate offset vs nominal, in ppm
    ramp_ppm_s: float = 0.0        # linear drift of that offset
    p_det: float = 1e-2            # detection probability per slot
    jitter_ps: float = 200.0       # RMS, Gaussian
    dark_hz: float = 0.0           # uniform noise counts
    duration_s: float = 1e-3
    t_offset_ps: float | None = None   # arrival time of slot 0; random if None
    gaps: list = field(default_factory=list)   # [(t0_s, t1_s), ...] source off
    dead_min_ps: float = 16750.0   # block 1 measured dead time window (W = 12 ns)
    dead_max_ps: float = 20000.0
    tdc_rms_ps: float = 11.7       # calibrated single-shot TDC error (block 1, SIMULATED)
    seed: int = 1


def slot_times(cfg: StreamCfg, k: np.ndarray, t_off: float) -> np.ndarray:
    """Arrival time (ps) of slot k for a source whose phase is
        phi(t) = f0 * ((1 + d) t + r t^2 / 2),   d = ppm*1e-6, r = ramp*1e-6 /s.
    Solved for phi(t) = k with the cancellation-free form of the quadratic root."""
    f0 = cfg.f_src_hz
    d = cfg.ppm * 1e-6
    r = cfg.ramp_ppm_s * 1e-6
    x = k / f0                                     # seconds at the nominal rate
    t = 2.0 * x / ((1.0 + d) + np.sqrt((1.0 + d) ** 2 + 2.0 * r * x))
    return t * 1e12 + t_off


def generate(cfg: StreamCfg):
    """Returns a dict of numpy arrays for the ACCEPTED stream (what the TDC
    timestamps) plus offered/accepted counts:
        t_ps      measured arrival time (jitter + TDC error included)
        t_true    arrival time without jitter (signal) / exact (dark)
        slot      true slot index (signal) or -1 (dark count)
    """
    rng = np.random.default_rng(cfg.seed)
    f0 = cfg.f_src_hz
    t_off = rng.uniform(0, 1e12 / f0) if cfg.t_offset_ps is None else cfg.t_offset_ps
    n_slots = int(cfg.duration_s * f0 * (1 + cfg.ppm * 1e-6)) + 1

    # --- signal: geometric gaps between detected slots
    exp_n = cfg.p_det * n_slots
    chunks, k_last = [], -1
    while True:
        m = int(exp_n * 1.1 + 100) if not chunks else int(exp_n * 0.2 + 100)
        g = rng.geometric(cfg.p_det, size=m) if cfg.p_det < 1 else np.ones(m, dtype=np.int64)
        ks = k_last + np.cumsum(g)
        chunks.append(ks)
        k_last = int(ks[-1])
        if k_last >= n_slots:
            break
    k = np.concatenate(chunks)
    k = k[k < n_slots].astype(np.int64)
    t_true = slot_times(cfg, k.astype(np.float64), t_off)
    keep = np.ones(len(k), bool)
    for g0, g1 in cfg.gaps:
        keep &= ~((t_true >= g0 * 1e12) & (t_true < g1 * 1e12))
    k, t_true = k[keep], t_true[keep]
    t_sig = t_true + rng.normal(0.0, cfg.jitter_ps, size=len(k))

    # --- dark counts
    n_dark = rng.poisson(cfg.dark_hz * cfg.duration_s)
    t_dark = np.sort(rng.uniform(0, cfg.duration_s * 1e12, size=n_dark))

    t_all = np.concatenate([t_sig, t_dark])
    tt_all = np.concatenate([t_true, t_dark])
    s_all = np.concatenate([k, -np.ones(n_dark, np.int64)])
    order = np.argsort(t_all, kind="stable")
    t_all, tt_all, s_all = t_all[order], tt_all[order], s_all[order]
    sel = t_all >= 0
    t_all, tt_all, s_all = t_all[sel], tt_all[sel], s_all[sel]

    # --- dead time (sequential; extended by edges that arrive while busy)
    acc = dead_time_filter(t_all, rng, cfg.dead_min_ps, cfg.dead_max_ps)
    t_meas = t_all[acc] + rng.normal(0.0, cfg.tdc_rms_ps, size=int(acc.sum()))
    return dict(
        t_ps=t_meas, t_true=tt_all[acc], slot=s_all[acc],
        offered=len(t_all), offered_signal=int((s_all >= 0).sum()),
        accepted=int(acc.sum()), accepted_signal=int((s_all[acc] >= 0).sum()),
        duration_s=cfg.duration_s, t_off=t_off,
    )


def dead_time_filter(t, rng, dmin, dmax):
    acc = np.zeros(len(t), bool)
    if len(t) == 0:
        return acc
    dead = rng.uniform(dmin, dmax, size=len(t))
    busy_until = -np.inf
    # Fast path: in sparse streams almost every gap exceeds the dead time.
    gaps = np.diff(t, prepend=-np.inf)
    if np.all(gaps[1:] >= dmax):
        acc[:] = True
        return acc
    for i in range(len(t)):
        if t[i] >= busy_until:
            acc[i] = True
            busy_until = t[i] + dead[i]
        else:
            busy_until = max(busy_until, t[i] + dead[i])
    return acc


def to_timestamps(t_ps: np.ndarray, t_clk_ps: float = T_CLK_PS, t0_cycle: int = 0):
    """Convert arrival times to TDC timestamps with block 1's convention:
        t_hit = coarse * T_clk - fine_ps,  0 <= fine_ps < T_clk
    coarse = index of the first clock edge at/after the hit. fine_ps is
    rounded to an integer ps (the calibrated LUT output is integer ps)."""
    coarse = np.ceil(t_ps / t_clk_ps).astype(np.int64)
    fine = np.rint(coarse * t_clk_ps - t_ps).astype(np.int64)
    # rounding can push fine to T_clk exactly: equivalent to (coarse+1, 0)...
    # keep it inside [0, T_clk) by moving such hits to the next edge.
    wrap = fine >= int(t_clk_ps)
    coarse[wrap] -= 1
    fine[wrap] -= int(t_clk_ps)
    return coarse + t0_cycle, fine
