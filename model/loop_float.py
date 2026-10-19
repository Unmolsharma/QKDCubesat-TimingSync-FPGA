#!/usr/bin/env python3
"""Floating-point model of the block 2 clock-recovery loop (NCO + PI + lock).

This is the behavioural reference: it fixes the algorithm and the gains, and
it is the model the long and statistical runs use (the RTL runs stay short).
model/loop_fixed.py then pins every bit width and rounding, and the RTL has
to match THAT model bit for bit. All results here are SIMULATED.

THE PROBLEM
-----------
Photons arrive at   t = (k + phase_offset) / f_src  + jitter
for a sparse random subset of slot numbers k, mixed with dark counts. The loop
has to keep an NCO (a phase accumulator counting slots) aligned with the
source, so that  round(NCO phase at the hit time) = k + constant  for every
signal photon. If the NCO ever gains or loses a whole slot relative to the
source (a CYCLE SLIP), every later photon gets the wrong slot number, and the
key bits are mis-sifted. That is the failure to avoid.

THE LOOP (one update per accepted hit, NOT per clock)
-----------------------------------------------------
    phi_hit = NCO phase at the hit time       (interpolated between clock edges)
    slot    = round(phi_hit)                   (slot number assigned to the hit)
    e       = phi_hit - slot   in [-0.5, 0.5)  (phase error, in slots)
    phase  -= Kp * e                           (proportional path)
    f      -= Ki * e / tau                     (integral path; f in slots/ps)

The phase error is only observable modulo one slot, so the detector is a
sawtooth. That is fine once locked (|e| << 0.5) and is why pull-in is the
hard part at low hit rates.

GAIN CHOICE (and why)
---------------------
Per hit, with u = (frequency error) x (mean hit spacing tau), the linearised
loop is
    p[k+1] = p[k] + u[k] - Kp e[k]          u[k+1] = u[k] - Ki e[k]
with characteristic equation  z^2 - (2 - Kp - Ki) z + (1 - Kp) = 0. For small
gains this is the classic second-order PLL with
    wn = sqrt(Ki)   [rad/hit],    zeta = Kp / (2 sqrt(Ki)).
Ki = Kp^2 / 2 gives zeta = 1/sqrt(2) = 0.707 (fastest settling without
ringing), and if Kp is a power of two, so is Ki: both gains are pure shifts.

* Acquisition gear, Kp = 2^-2 (Ki = 2^-5): wn = 0.18 rad/hit, settles in a
  few tens of hits and tolerates up to ~0.1-0.2 slot of phase drift between
  hits. Its noise bandwidth is wide, so it is only used until lock.
* Tracking gear,  Kp = 2^-6 (Ki = 2^-13): wn = 0.011 rad/hit, noise
  bandwidth ~ Kp/2 per hit, so the NCO phase error is about sqrt(Kp/2) = 1/11
  of the per-hit jitter. Ramp (Doppler-rate) error is e_ss = a / Ki, with
  a = phase acceleration per hit^2, which sets the minimum hit rate for the
  0.4 ppm/s case (plot b). kp_trk is a register: 2^-4 tracks a 0.4 ppm/s
  ramp down to ~2e3 hits/s at the cost of ~2x the steady-state jitter.
* GEAR SHIFTING IS GRADUAL AND HAPPENS BEFORE LOCK IS DECLARED.
  States ACQ -> RAMP -> LOCK. When the e^2 average first drops below the
  lock threshold ("phase acquired"), Kp is halved every 64 loop updates
  (2^-2 -> 2^-3 -> ... -> 2^-6), then the loop dwells 64 more updates at the
  tracking gear; LOCK is declared only if the e^2 average is still below the
  threshold. The integrator is preserved throughout; any trouble during the
  ramp (threshold, rejects, timeout) restarts acquisition.
  Why, in the order the float model forced it:
   1. A direct 2^-2 -> 2^-6 jump at lock fails at low rates: lock is
      declared with a few tenths of a ppm of residual frequency error (the
      acquisition gear's frequency estimate is noisy), the slow gear lets
      the phase walk out to the noise gate, the gate then rejects the real
      signal, and the loop freezes (1 in 24 trials at p = 5e-5).
   2. A gradual ramp that starts AT lock fixes that in typical conditions,
      but with 500 ps jitter and dark counts at 50 % of the signal the
      acquisition gear's frequency estimate can be several ppm (once 21 ppm)
      off when lock is declared. The phase then rotates through the slot
      faster than any detector can react: 2 UNDETECTED slips in 13 M locked
      hits.
   3. Ramping first and declaring lock only at the tracking gear means the
      lock flag is backed by the precise frequency estimate: 0 undetected
      slips and 0 false unlocks in 23 M locked hits (slip hunt below). The
      price is ~320 more hits to lock.

HIT-RATE GAIN SCALING
---------------------
The frequency integrator's effect on phase grows with the time between hits.
With a fixed per-hit Ki the loop would be over-gained at low rates (unstable)
and sluggish at high rates. So the frequency step is divided by tau, the mean
spacing between loop-updating hits, rounded to a power of two (2^L clocks) so
the division is a shift. In slots/ps the step is
    df = -e * Ki / (2^L * T_clk_ps),   with T_clk_ps = 4000 taken as 2^12
(that 2.4 % gain error is harmless). tau is an exponential average (weight
1/16) of the spacing, seeded by the first measured interval; a register can
override it. Rounding L to the nearest power of two moves the effective Ki by
up to a factor sqrt(2) either way: zeta stays between 0.6 and 0.84.

NOISE GATE
----------
Once locked, a hit with |e| >= g is flagged as noise and does not update the
loop. Dark counts are uniform in phase, so a fraction 2g of them still passes
whatever g is: that is the ceiling of any phase-window gate. Default
g = 1/8 slot (25 % of dark counts pass; 1.25 ns at 100 MHz, so 2.5 sigma of
the worst 500 ps jitter). The gate is open (everything updates the loop)
while unlocked: acquisition cannot know which hits are noise.

The gate is armed from the start of the gear ramp. (Leaving it open until the
tracking gear was tried: dark counts then drive the still-fast loop and
cause slips.)

LOCK DETECTOR
-------------
Exponential average of e^2. Random phase gives 1/12 = 0.083. "Phase
acquired" (start of the gear ramp, see above) and "locked" both use the
1/64 threshold; unlocked if it rises above 1/16 (hysteresis), if
N=16 consecutive hits are gated out as noise, or if no hit arrives for
2^(L+5) clocks (32 mean spacings, so a false timeout has probability e^-32
per hit). The average restarts at 1/12 on unlock.
The averaging weight is 2^-5 while acquiring (fast lock) and 2^-7 once
locked: with dark counts at 50 % of the signal, 2^-5 gave 34 false unlocks
per 1.65 M locked hits, 2^-7 gives 1.
Uniform dark counts raise the locked average towards 1/12: with a
noise:signal ratio r it is about (sigma^2 + r/12)/(1+r). With the default
thresholds lock is DECLARED up to r ~ 0.6 and HELD up to r ~ 3. That is a
property of any mean-square metric (both thresholds are registers); a
window-occupancy metric would separate locked/unlocked somewhat better.
A SUSPECTED SLIP is flagged when the average of the signed error (weight
2^-5, over ALL hits while locked) leaves +/-1/8 slot: a real slip has to drag
the phase through +/-0.5, so it shows up there first. Dark counts average to
zero, so including them only adds variance (std ~0.02 slot at 50 % dark,
6 sigma below the threshold). By default a suspected slip also forces an
UNLOCK, so the loss of slot integrity is always reported. This closes the
one undetected-slip mechanism the float model found: lock declared with a
residual frequency error at very low rate, phase walks out to the gate, the
gate rejects the real photons while the few dark counts that land inside it
keep the e^2 average low and break up the consecutive-reject run.
"""

import argparse
import json
import math
import os
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hitgen import T_CLK_PS, StreamCfg, generate  # noqa: E402


@dataclass
class LoopCfg:
    f_nom_hz: float = 100e6     # nominal source rate (the NCO's start frequency)
    kp_acq: int = 2             # Kp = 2^-kp
    kp_trk: int = 6
    gear_log2: int = 6          # loop updates per gear step (2^6 = 64), and the final dwell
    gate: float = 1 / 8         # noise window half-width, slots
    gate_en: bool = True
    mse_shift: int = 5          # lock-detector averaging weight 2^-5 while acquiring
    mse_shift_trk: int = 7      # ... and 2^-7 once locked (fewer false unlocks)
    lock_thr: float = 1 / 64
    unlock_thr: float = 1 / 16
    rej_max: int = 16           # consecutive gated-out hits -> unlock
    to_shift: int = 5           # no-hit timeout = 2^(L + to_shift) clocks
    tau_shift: int = 4          # hit-spacing average weight 2^-4
    tau_init_log2: int = 16     # L before the first interval is measured
    tau_ovr: int | None = None  # force L (register override)
    slip_shift: int = 5         # signed-error average weight 2^-5 (all hits, while locked)
    slip_thr: float = 1 / 8
    slip_unlock: bool = True    # a suspected slip forces an unlock


def round_log2(x: float) -> int:
    """Power-of-two rounding used for tau: L = msb(x) + (x >= 1.5 * 2^msb).
    The fixed-point/RTL version is the same rule on the integer part."""
    if x < 1:
        return 0
    m = int(math.floor(math.log2(x)))
    return m + (1 if x >= 1.5 * 2 ** m else 0)


ACQ, RAMP, LOCK = 0, 1, 2          # loop states (the RTL uses the same encoding)


class FloatLoop:
    """Hit-by-hit floating point loop. Time in ps, phase in slots, f in slots/ps.

    States:
      ACQ   acquisition gear (Kp = 2^-kp_acq), gate open, not locked.
      RAMP  phase acquired (e^2 average < lock_thr): Kp is halved every
            2^gear_log2 loop updates down to 2^-kp_trk, gate armed. Still NOT
            locked: anything going wrong here restarts acquisition.
      LOCK  declared after one more 2^gear_log2-update dwell at the tracking
            gear with the e^2 average still below lock_thr. Only now is the
            slot numbering claimed to be continuous.
    """

    def __init__(self, cfg: LoopCfg, f0_ps=None, phi0=0.0):
        self.c = cfg
        self.f_nom = cfg.f_nom_hz * 1e-12
        self.f = self.f_nom if f0_ps is None else f0_ps
        self.phi, self.t_ref = phi0, 0.0
        self.state = ACQ
        self.kp, self.gear_cnt, self.gear_done = cfg.kp_acq, 0, False
        self.mse = 1 / 12
        self.rej_run = 0
        self.sgn = 0.0
        self.slip_armed = True
        self.L = cfg.tau_init_log2
        self.tavg = None
        self.t_prev_upd = None
        self.t_last = None
        self.unlock_cnt = self.suspect_cnt = self.noise_cnt = self.abort_cnt = 0

    @property
    def locked(self):
        return self.state == LOCK

    def set_locked(self):
        """Start in LOCK at the tracking gear (tracking-only experiments)."""
        self.state, self.kp, self.mse = LOCK, self.c.kp_trk, 0.0

    def phase_at(self, t):
        return self.phi + self.f * (t - self.t_ref)

    def _to_acq(self):
        if self.state == LOCK:
            self.unlock_cnt += 1
        elif self.state == RAMP:
            self.abort_cnt += 1
        self.state = ACQ
        self.kp, self.gear_cnt, self.gear_done = self.c.kp_acq, 0, False
        self.mse = 1 / 12
        self.rej_run = 0

    def step(self, t):
        c = self.c
        L = c.tau_ovr if c.tau_ovr is not None else self.L
        if (self.state != ACQ and self.t_last is not None
                and (t - self.t_last) > 2 ** (L + c.to_shift) * T_CLK_PS):
            self._to_acq()                                   # no-hit timeout
        phi = self.phase_at(t)
        slot = math.floor(phi + 0.5)
        e = phi - slot
        noise = self.state != ACQ and c.gate_en and abs(e) >= c.gate
        kp = self.kp
        self.phi, self.t_ref = phi, t
        if not noise:
            self.phi -= e * 2.0 ** -kp
            self.f -= e * 2.0 ** -(2 * kp + 1) / (2.0 ** L * 4096.0)
            # hit-spacing estimate (loop-updating hits only)
            if self.t_prev_upd is not None:
                d = (t - self.t_prev_upd) / T_CLK_PS
                self.tavg = d if self.tavg is None else self.tavg + (d - self.tavg) / 2 ** c.tau_shift
                self.L = min(round_log2(self.tavg), 30)
            self.t_prev_upd = t
            # gear ramp: halve Kp every 2^gear_log2 updates, then one more
            # dwell at the tracking gear before lock may be declared
            if self.state != ACQ:
                self.gear_cnt += 1
                if self.gear_cnt == 2 ** c.gear_log2:
                    self.gear_cnt = 0
                    if self.kp < c.kp_trk:
                        self.kp += 1
                    else:
                        self.gear_done = True
        else:
            self.noise_cnt += 1
        self.t_last = t
        # lock detector
        self.mse += (e * e - self.mse) / 2 ** (c.mse_shift_trk if self.state == LOCK else c.mse_shift)
        self.rej_run = self.rej_run + 1 if noise else 0
        if self.state == ACQ:
            if self.mse < c.lock_thr:
                self.state, self.gear_cnt, self.gear_done = RAMP, 0, False
        elif self.mse > c.unlock_thr or self.rej_run >= c.rej_max:
            self._to_acq()
        elif self.state == RAMP and self.gear_done and self.mse < c.lock_thr:
            self.state = LOCK
            self.sgn, self.slip_armed = 0.0, True
        if self.state == LOCK:
            # Signed-error average over ALL hits (noise included): dark counts
            # average to zero, but a phase that walks off (and that the gate
            # has started to reject) pulls it to one side.
            self.sgn += (e - self.sgn) / 2 ** c.slip_shift
            if self.slip_armed and abs(self.sgn) > c.slip_thr:
                self.suspect_cnt += 1
                self.slip_armed = False
                if c.slip_unlock:
                    self._to_acq()
            elif not self.slip_armed and abs(self.sgn) < c.slip_thr / 2:
                self.slip_armed = True
        return slot, e, noise, self.state == LOCK


# -----------------------------------------------------------------------------
# One run: stream -> loop -> per-hit trace + truth comparison
# -----------------------------------------------------------------------------
def run(scfg: StreamCfg, lcfg: LoopCfg, start_locked=False, stop_after_verify=None):
    """Feed one generated stream through the loop.

    start_locked: start with the NCO exactly on the source (phase and
    frequency) and the lock detector locked, i.e. measure tracking only.
    stop_after_verify: stop that many signal hits after the first lock.

    Truth: for each signal hit we know its slot k and its jitter-free
    arrival time. nco_err = (NCO phase at the jitter-free time) - k is the
    real tracking error; its rounded value is the NCO's slot offset, and a
    change of that offset after lock is a CYCLE SLIP."""
    s = generate(scfg)
    t, slot_true, t_true = s["t_ps"], s["slot"], s["t_true"]
    f_src_ps = scfg.f_src_hz * 1e-12 * (1 + scfg.ppm * 1e-6)
    if start_locked:
        lp = FloatLoop(lcfg, f0_ps=f_src_ps, phi0=-s["t_off"] * f_src_ps)
        lp.set_locked()
    else:
        lp = FloatLoop(lcfg)
    n = len(t)
    rec = dict(t=np.empty(n), e=np.empty(n), noise=np.zeros(n, bool), locked=np.zeros(n, bool),
               slot=np.zeros(n, np.int64), f_ppm=np.empty(n), nco_err=np.full(n, np.nan),
               state=np.zeros(n, np.int8))
    lock_i, n_sig_after = None, 0
    m = n
    for i in range(n):
        if slot_true[i] >= 0:
            rec["nco_err"][i] = lp.phase_at(t_true[i]) - slot_true[i]
        sl, e, noise, locked = lp.step(t[i])
        rec["t"][i], rec["e"][i], rec["noise"][i], rec["locked"][i] = t[i], e, noise, locked
        rec["slot"][i] = sl
        rec["state"][i] = lp.state
        rec["f_ppm"][i] = (lp.f / lp.f_nom - 1) * 1e6
        if locked and lock_i is None:
            lock_i = i
        if lock_i is not None and slot_true[i] >= 0:
            n_sig_after += 1
            if stop_after_verify and n_sig_after >= stop_after_verify:
                m = i + 1
                break
    for k in rec:
        rec[k] = rec[k][:m]
    rec["slot_true"] = slot_true[:m]
    return rec, s, lp, lock_i


def assess(rec, lock_i, true_ppm, ramp_ppm_s=0.0):
    """Slips, slot errors and frequency error after the first lock."""
    out = dict(locked=lock_i is not None)
    if lock_i is None:
        return out
    sig = (rec["slot_true"] >= 0) & (np.arange(len(rec["t"])) >= lock_i)
    off = np.rint(rec["nco_err"][sig]).astype(np.int64)
    out["slips"] = int(np.count_nonzero(np.diff(off))) if len(off) > 1 else 0
    # UNDETECTED slip: the slot offset changes while the lock flag stayed set
    # on every hit in between. A change across an unlock is a DETECTED loss:
    # the unlock is reported, and slot numbering must be re-established.
    lk = rec["locked"][lock_i:]
    seg = np.cumsum(~lk)                         # increments at every unlocked hit
    seg_sig = seg[sig[lock_i:]]
    lk_sig = lk[sig[lock_i:]]
    same = (np.diff(seg_sig) == 0) & lk_sig[1:] & lk_sig[:-1]
    out["undetected_slips"] = int(np.count_nonzero((np.diff(off) != 0) & same))
    out["unlocks"] = int(np.count_nonzero(lk[:-1] & ~lk[1:]))
    slot_off = rec["slot"][sig] - rec["slot_true"][sig]
    out["slot_errors"] = int(np.count_nonzero(slot_off != (off[0] if len(off) else 0)))
    out["lock_time_s"] = rec["t"][lock_i] * 1e-12
    out["lock_hits"] = int(lock_i + 1)
    # The integrator value is noisy hit to hit at high rates (the loop
    # bandwidth scales with the hit rate), so judge the frequency estimate on
    # its average over the second half of the post-lock window.
    h = lock_i + (len(rec["t"]) - lock_i) // 2
    t_mid = rec["t"][h:].mean() * 1e-12
    out["f_err_ppm"] = float(rec["f_ppm"][h:].mean() - (true_ppm + ramp_ppm_s * t_mid))
    out["f_rms_ppm"] = float(rec["f_ppm"][h:].std())
    return out


# -----------------------------------------------------------------------------
# Sweeps for the plots (parallel over trials)
# -----------------------------------------------------------------------------
def _lock_trial(args):
    p, U, trial, f_src, jitter, dark_ratio, n_lock_max, n_verify = args
    rng = np.random.default_rng(10_000 * trial + 7)
    ppm = [U, -U][trial] if trial < 2 else rng.uniform(-U, U)
    rate = p * f_src
    dur = (n_lock_max + n_verify) / rate
    scfg = StreamCfg(f_src_hz=f_src, ppm=ppm, p_det=p, jitter_ps=jitter,
                     dark_hz=dark_ratio * rate, duration_s=dur, seed=trial * 7919 + int(-math.log10(p) * 100))
    lcfg = LoopCfg(f_nom_hz=f_src)
    rec, s, lp, lock_i = run(scfg, lcfg, stop_after_verify=n_verify)
    a = assess(rec, lock_i, ppm)
    # success: locked in time, then held lock (no unlock, no slip) for the
    # whole verification window with a sane frequency estimate
    ok = (a["locked"] and lock_i < n_lock_max and a["slips"] == 0 and a["unlocks"] == 0
          and abs(a["f_err_ppm"]) < 5.0 and rec["locked"][-1])
    return dict(p=p, U=U, ppm=ppm, ok=bool(ok), lock_time_s=a.get("lock_time_s"),
                lock_hits=a.get("lock_hits"), slips=a.get("slips"), f_err_ppm=a.get("f_err_ppm"),
                undetected_slips=a.get("undetected_slips"), unlocks=a.get("unlocks"),
                acc_rate=s["accepted"] / s["duration_s"], off_rate=s["offered"] / s["duration_s"])


def _hunt_trial(args):
    p, U, k, jitter, dark = args
    rng = np.random.default_rng(77_000 + k)
    ppm = rng.uniform(-U, U)
    rate = p * 100e6
    scfg = StreamCfg(f_src_hz=100e6, ppm=ppm, p_det=p, jitter_ps=jitter, dark_hz=dark * rate,
                     duration_s=20000 / rate, seed=5000 + k)
    rec, s, lp, li = run(scfg, LoopCfg())
    if li is None:
        return dict(locked_hits=0, undetected=0, unlocks=0, suspects=0)
    a = assess(rec, li, ppm)
    return dict(locked_hits=int(rec["locked"].sum()), undetected=a["undetected_slips"],
                unlocks=a["unlocks"], suspects=lp.suspect_cnt)


def slip_hunt(trials=64, procs=None):
    """Many long runs in hard conditions, counting slips that happen while
    the lock flag is continuously set. The claim to test is 'zero'."""
    from multiprocessing import Pool
    out = []
    for jitter, dark in ((200.0, 0.1), (500.0, 0.5)):
        jobs = [(p, U, k, jitter, dark) for p in (1e-2, 1e-3, 1e-4, 3e-5)
                for U in (100.0, 1.0) for k in range(trials)]
        with Pool(procs or os.cpu_count()) as pool:
            r = pool.map(_hunt_trial, jobs, chunksize=1)
        out.append(dict(jitter_ps=jitter, dark_ratio=dark, runs=len(r),
                        locked_hits=sum(x["locked_hits"] for x in r),
                        undetected_slips=sum(x["undetected"] for x in r),
                        detected_unlocks=sum(x["unlocks"] for x in r),
                        suspected_slips=sum(x["suspects"] for x in r)))
        print("slip hunt:", out[-1])
    return out


def lock_sweep(rates, Us, trials, f_src=100e6, jitter=200.0, dark_ratio=0.1,
               n_lock_max=20000, n_verify=2000, procs=None):
    from multiprocessing import Pool
    jobs = [(p, U, k, f_src, jitter, dark_ratio, n_lock_max, n_verify)
            for U in Us for p in rates for k in range(trials)]
    with Pool(procs or os.cpu_count()) as pool:
        return pool.map(_lock_trial, jobs, chunksize=1)


def _ramp_trial(args):
    rate, kp, f_src, jitter, ramp, ppm0, seed = args
    n_hits = 4000
    dur = max(0.2, n_hits / rate)
    scfg = StreamCfg(f_src_hz=f_src, ppm=ppm0, ramp_ppm_s=ramp, p_det=rate / f_src,
                     jitter_ps=jitter, duration_s=dur, seed=seed)
    lcfg = LoopCfg(f_nom_hz=f_src, kp_trk=kp)
    rec, s, lp, _ = run(scfg, lcfg, start_locked=True)
    sig = rec["slot_true"] >= 0
    err = rec["nco_err"][sig]
    lk = rec["locked"][sig]
    off = np.rint(err).astype(np.int64)
    slips = int(np.count_nonzero(np.diff(off)))
    skip = len(err) // 3                       # ignore the start-up transient
    e = (err - off)[skip:][lk[skip:]]          # tracking error while locked
    t_slot_ps = 1e12 / f_src
    # The loop "tracks" only if it never lost lock and never slipped. If it
    # did, the acquisition gear re-captured it, and slot numbering across
    # that unlock is not guaranteed: count it as a failure.
    return dict(rate=rate, kp=kp, acc_rate=s["accepted"] / dur, slips=slips,
                tracked=bool(slips == 0 and lp.unlock_cnt == 0),
                max_err_ps=float(np.max(np.abs(e)) * t_slot_ps) if len(e) else float("nan"),
                rms_err_ps=float(np.sqrt(np.mean(e ** 2)) * t_slot_ps) if len(e) else float("nan"),
                unlocks=lp.unlock_cnt, noise_frac=float(rec["noise"][sig].mean()))


def ramp_sweep(rates, kps, f_src=500e6, jitter=50.0, ramp=0.4, ppm0=25.0, seeds=3, procs=None):
    from multiprocessing import Pool
    jobs = [(r, kp, f_src, jitter, ramp, ppm0, 100 + s) for kp in kps for r in rates for s in range(seeds)]
    with Pool(procs or os.cpu_count()) as pool:
        return pool.map(_ramp_trial, jobs, chunksize=1)


def ramp_theory(rate, kp, f_src=500e6, ramp=0.4):
    """Steady-state type-2 error for a frequency ramp: e_ss = a / Ki,
    a = phase acceleration per hit^2 = f_src * ramp*1e-6 / rate^2 (slots)."""
    a = f_src * ramp * 1e-6 / rate ** 2
    return a / 2.0 ** -(2 * kp + 1) * 1e12 / f_src     # ps


# -----------------------------------------------------------------------------
# Plots
# -----------------------------------------------------------------------------
C1, C2, C3 = "#2a78d6", "#eb6834", "#1f9e74"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def _style():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
        "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
        "ytick.color": INK2, "text.color": INK, "axes.grid": True,
        "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
        "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
        "axes.titleweight": "bold", "legend.frameon": False, "lines.linewidth": 2,
    })
    return plt


def summarize_lock(res, rates, Us):
    rows = []
    for U in Us:
        for p in rates:
            r = [x for x in res if x["U"] == U and x["p"] == p]
            ok = [x for x in r if x["ok"]]
            lt = np.array([x["lock_time_s"] for x in ok]) if ok else np.array([np.nan])
            rows.append(dict(U=U, p=p, trials=len(r), success=len(ok) / len(r),
                             lock_med_s=float(np.median(lt)), lock_p90_s=float(np.percentile(lt, 90)),
                             lock_hits_med=float(np.median([x["lock_hits"] for x in ok])) if ok else None,
                             off_rate=float(np.mean([x["off_rate"] for x in r])),
                             acc_rate=float(np.mean([x["acc_rate"] for x in r])),
                             slips=sum((x["slips"] or 0) for x in r),
                             undetected_slips=sum((x["undetected_slips"] or 0) for x in r),
                             unlocks=sum((x["unlocks"] or 0) for x in r)))
    return rows


def plot_lock(rows, Us, path):
    plt = _style()
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 6.5), sharex=True)
    for U, col in zip(Us, (C2, C1)):
        r = [x for x in rows if x["U"] == U]
        p = [x["p"] for x in r]
        a1.semilogx(p, [100 * x["success"] for x in r], "o-", color=col, label=f"±{U:g} ppm")
        med = np.array([x["lock_med_s"] for x in r])
        p90 = np.array([x["lock_p90_s"] for x in r])
        a2.loglog(p, med, "o-", color=col, label=f"±{U:g} ppm, median")
        a2.loglog(p, p90, "--", color=col, lw=1.2, label=f"±{U:g} ppm, 90th pct")
    a1.set_ylabel("lock success (%)")
    a1.set_ylim(-5, 105)
    a1.legend(loc="lower right")
    a1.set_title("PI-only acquisition, 100 MHz source  [SIMULATED]", loc="left")
    a2.set_ylabel("time to lock (s)")
    a2.set_xlabel("detection probability per slot")
    a2.legend(fontsize=8, loc="upper right")
    a2.invert_xaxis()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def ramp_limit(res, kp):
    """Lowest accepted rate from which every higher rate tracked (all seeds)."""
    rates = sorted({x["rate"] for x in res if x["kp"] == kp})
    lim = None
    for r in reversed(rates):
        xs = [x for x in res if x["kp"] == kp and x["rate"] == r]
        if all(x["tracked"] for x in xs):
            lim = float(np.mean([x["acc_rate"] for x in xs]))
        else:
            break
    return lim


def plot_ramp(res, rates, kps, path, f_src=500e6, ramp=0.4, gate_ps=None, kp_default=6):
    plt = _style()
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    cols = {4: C3, 5: C1, 6: C2}
    t_slot = 1e12 / f_src
    for kp in kps:
        acc, mx, ok = [], [], []
        for r in rates:
            xs = [x for x in res if x["kp"] == kp and x["rate"] == r]
            acc.append(np.mean([x["acc_rate"] for x in xs]))
            ok.append(all(x["tracked"] for x in xs))
            mx.append(np.nanmax([x["max_err_ps"] for x in xs]) if ok[-1] else t_slot / 2)
        col = cols.get(kp, INK2)
        a_ok = [a for a, o in zip(acc, ok) if o]
        m_ok = [m for m, o in zip(mx, ok) if o]
        ax.loglog(a_ok, m_ok, "o-", color=col, label=f"tracking Kp = 2^-{kp}")
        a_bad = [a for a, o in zip(acc, ok) if not o]
        ax.loglog(a_bad, [t_slot / 2] * len(a_bad), "x", color=col, ms=9, mew=2)
        th = np.array([ramp_theory(a, kp, f_src, ramp) for a in acc])
        ax.loglog(np.array(acc)[th > 1], th[th > 1], ":", color=col, lw=1.2)
    lim = ramp_limit(res, kp_default)
    if lim:
        ax.axvspan(min(rates) * 0.7, lim, color=C2, alpha=0.07, lw=0)
        ax.text(min(rates) * 0.8, 6,
                f"below ~{lim:.0f}/s (Kp=2^-{kp_default}):\n"
                "needs Doppler feed-forward\nor a type-3 loop",
                fontsize=8, color=INK2)
    ax.axhline(t_slot / 2, color=INK, lw=1, ls="--")
    ax.text(min(rates) * 0.8, t_slot / 2 * 1.1, "half a slot (x = lost lock or slipped)",
            fontsize=8, color=INK2)
    if gate_ps:
        ax.axhline(gate_ps, color=INK2, lw=1, ls="-.")
        ax.text(max(rates) * 0.9, gate_ps * 1.1, "noise gate edge (1/8 slot)",
                fontsize=8, color=INK2, ha="right")
    ax.set_xlim(min(rates) * 0.7, max(rates) * 1.4)
    ax.set_ylim(3, 3e4)
    ax.set_xlabel("accepted hit rate (1/s)")
    ax.set_ylabel("max |tracking error| while locked (ps)")
    ax.set_title(f"Doppler ramp {ramp} ppm/s at {f_src/1e6:.0f} MHz, 50 ps jitter  [SIMULATED]",
                 loc="left")
    ax.legend(loc="upper right", fontsize=8, title="dotted: a/Ki theory", title_fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_trace(rec, lock_i, true_ppm, path, title):
    plt = _style()
    t_ms = rec["t"] * 1e-9
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 6), sharex=True)
    n = rec["noise"]
    # shade the gear ramp (phase acquired -> lock declared)
    st = rec["state"]
    ramp = np.flatnonzero(st == RAMP)
    for a in (a1, a2):
        if len(ramp):
            a.axvspan(t_ms[ramp[0]], t_ms[ramp[-1]], color=C3, alpha=0.08, lw=0)
        if lock_i is not None:
            a.axvline(t_ms[lock_i], color=C3, lw=1)
    a1.plot(t_ms[~n], rec["e"][~n], ".", ms=2.5, color=C1, label="loop-updating hit")
    a1.plot(t_ms[n], rec["e"][n], ".", ms=2.5, color=INK2, alpha=0.5, label="gated as noise")
    a1.set_ylabel("phase error e (slots)")
    a1.set_ylim(-0.55, 0.62)
    if len(ramp):
        a1.text(t_ms[ramp[0]], 0.53, " gear ramp 2^-2 -> 2^-6", color=C3, fontsize=8)
    if lock_i is not None:
        a1.text(t_ms[lock_i], 0.53, " LOCK", color=C3, fontsize=8, fontweight="bold")
    a2.plot(t_ms, rec["f_ppm"], color=C1, lw=1.5, label="NCO frequency estimate")
    a2.axhline(true_ppm, color=C2, lw=1, ls="--", label=f"true offset {true_ppm:+.2f} ppm")
    a2.set_ylabel("frequency offset (ppm)")
    a2.set_xlabel("time (ms)")
    a1.legend(loc="lower right", fontsize=8, markerscale=4)
    a2.legend(loc="lower right", fontsize=8)
    a1.set_title(title + "  [SIMULATED]", loc="left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description="float loop model: sweeps + plots (SIMULATED)")
    ap.add_argument("--outdir", default=str(Path(__file__).resolve().parents[1] / "docs" / "img"))
    ap.add_argument("--trials", type=int, default=24)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", choices=["lock", "ramp", "trace", "hunt"], action="append")
    args = ap.parse_args(argv)
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    jpath = out / "nco_float_results.json"
    results = json.loads(jpath.read_text()) if jpath.exists() else {}
    results["label"] = "SIMULATED (model/loop_float.py)"
    results["loop_cfg"] = asdict(LoopCfg())
    only = set(args.only or ["lock", "ramp", "trace", "hunt"])

    if "trace" in only:
        scfg = StreamCfg(f_src_hz=100e6, ppm=37.0, p_det=1e-3, jitter_ps=200.0,
                         dark_hz=0.1 * 1e5, duration_s=0.012, seed=3)
        rec, s, lp, lock_i = run(scfg, LoopCfg())
        a = assess(rec, lock_i, 37.0)
        plot_trace(rec, lock_i, 37.0, out / "nco_lock_trace.png",
                   "Typical lock: 100 MHz, +37 ppm, p=1e-3, 200 ps, 10 % dark")
        results["trace"] = a
        print("trace:", a)

    if "lock" in only:
        rates = [1e-1, 3e-2, 1e-2, 3e-3, 1e-3, 3e-4, 1e-4, 5e-5, 3e-5, 2e-5, 1e-5]
        Us = [100.0, 1.0]
        trials = 6 if args.quick else args.trials
        # typical: 200 ps jitter, dark counts 10 % of the signal rate
        # stress:  500 ps jitter (the 100 MHz worst case), dark 50 % of signal
        for name, jit, dark in (("lock_sweep", 200.0, 0.1), ("lock_sweep_stress", 500.0, 0.5)):
            res = lock_sweep(rates, Us, trials, jitter=jit, dark_ratio=dark)
            rows = summarize_lock(res, rates, Us)
            print(f"--- {name}: jitter {jit:g} ps, dark {100 * dark:g} % of signal")
            for r in rows:
                print(f"U=+/-{r['U']:5g} ppm p={r['p']:7.0e}  success {100*r['success']:5.1f} %  "
                      f"lock med {r['lock_med_s']:.3g} s p90 {r['lock_p90_s']:.3g} s  "
                      f"acc {r['acc_rate']:.3g}/s  unlocks {r['unlocks']} slips {r['slips']} UNDETECTED {r['undetected_slips']}")
            results[name] = dict(jitter_ps=jit, dark_ratio=dark, n_lock_max_hits=20000,
                                 rows=rows)
            if name == "lock_sweep":
                plot_lock(rows, Us, out / "nco_lock_vs_rate.png")

    if "ramp" in only:
        rates = [3e2, 1e3, 2e3, 3e3, 5e3, 1e4, 2e4, 3e4, 1e5, 1e6]
        kps = [4, 5, 6]
        res = ramp_sweep(rates, kps, seeds=2 if args.quick else 4)
        plot_ramp(res, rates, kps, out / "nco_ramp_error.png", gate_ps=1e12 / 500e6 / 8)
        rows = []
        for kp in kps:
            for r in rates:
                xs = [x for x in res if x["kp"] == kp and x["rate"] == r]
                rows.append(dict(kp=kp, rate=r, acc_rate=float(np.mean([x["acc_rate"] for x in xs])),
                                 max_err_ps=float(np.max([x["max_err_ps"] for x in xs])),
                                 rms_err_ps=float(np.mean([x["rms_err_ps"] for x in xs])),
                                 theory_ps=ramp_theory(r, kp),
                                 slips=sum(x["slips"] for x in xs),
                                 tracked=all(x["tracked"] for x in xs),
                                 unlocks=sum(x["unlocks"] for x in xs)))
                print(f"ramp kp={kp} rate={r:8.0f}/s  max {rows[-1]['max_err_ps']:9.1f} ps  "
                      f"rms {rows[-1]['rms_err_ps']:8.1f} ps  theory {rows[-1]['theory_ps']:9.1f} ps  "
                      f"slips {rows[-1]['slips']} unlocks {rows[-1]['unlocks']}")
        results["ramp_sweep"] = rows
        results["ramp_limit_hits_s"] = {f"kp{kp}": ramp_limit(res, kp) for kp in kps}
        print("ramp limit (lowest rate that tracked, all seeds):", results["ramp_limit_hits_s"])

    if "hunt" in only:
        results["slip_hunt"] = slip_hunt(16 if args.quick else 64)

    jpath.write_text(json.dumps(results, indent=1))
    print("wrote", jpath)


if __name__ == "__main__":
    main()
