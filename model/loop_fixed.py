#!/usr/bin/env python3
"""Bit-exact, cycle-exact fixed-point model of rtl/nco (the golden reference).

The RTL must reproduce this model on EVERY hit: slot, phase error, residual,
noise flag, lock state, and the NCO phase/frequency registers. Algorithm and
gains come from model/loop_float.py; this file pins the widths, the rounding
and WHEN each register changes. Results are SIMULATED.

NUMBER FORMATS (all integers; "2^-k" = units of the LSB)
  P      NCO phase, unsigned 88 bit: 48-bit slot index . 40-bit fraction (2^-40 slot)
  fps    NCO frequency in slots per ps, signed 48 bit, 2^-56 slot/ps.
         (Storing slots/ps, not slots/clock, is what lets the phase detector
         interpolate with ONE multiply: phase(hit) = P - fine_ps * fps.)
  fclk   per-clock phase increment = (fps * T_CLK_PS) >> 16, 2^-40 slot.
         Multiplying by the constant 4000 is shift-and-add in hardware.
  e24    phase error, signed 25 bit, 2^-24 slot, range [-0.5, 0.5)
  mse    e^2 average, unsigned 32 bit, 2^-32 slot^2
  sgn    signed-error average, signed 25 bit, 2^-24 slot
  tavg   mean hit spacing, unsigned, 2^-4 clock
ROUNDING: every right shift that drops bits is round-half-up,
  rhu(x, s) = (x + 2^(s-1)) >> s  (arithmetic, i.e. floor after adding half),
EXCEPT three documented truncations (floor): fclk, the 25-bit multiplier
operand fps >> 22, and P -> 24 fraction bits before the slot decision
(floor keeps e inside [-0.5, 0.5)).

TIMING (cycle n = the value of the NCO time counter t_nco, which runs D
clocks behind the TDC coarse counter so every hit arrives before the NCO
reaches it)
  a  hit written into the align FIFO at the edge that ends cycle a
  n  hit popped: n = max(c, a + 1, n_prev + POP_GAP); j = n - c.
     j > J_MAX -> LATE (dropped and counted, no output). Otherwise the hit
     sees P[n], fps[n] and uses fine' = fine_ps + j * T_CLK_PS, which is
     exact: t_hit = c*T - fine = n*T - (fine + j*T).
  n+1 multiply (DSP), n+2 subtract, n+3 slot/e/e^2, n+4 = E stage (gate,
  loop filter, averages), n+5 = F stage (state machine, counters, output)
  n+6 loop corrections visible: P[n+6] includes dphi, fps[n+6] includes df,
     fclk[n+7] = (fps[n+6] * 4000) >> 16.
  POP_GAP = 2 guarantees hit k+1's E stage runs after hit k's F stage, so
  every hit sees the state left by the previous one (no pipeline hazard).
  No-hit timeout is checked every cycle the E/F stages are idle; before hit
  k's E stage it has fired iff (E_k - 1) - c_last > 2^(L + to_shift).
  External correction strobe in cycle X: P[X+1] += dphase, fps[X+1] += dfreq.
"""

from dataclasses import dataclass, field

T_CLK_PS = 4000
PH_FRAC, SLOT_W = 40, 48
PH_W = PH_FRAC + SLOT_W                  # 88
PH_MASK = (1 << PH_W) - 1
FPS_FRAC, FPS_W = 56, 48
MUL_SHIFT = 22                           # fps >> 22 -> 2^-34 slot/ps, 25-bit signed operand
E_FRAC = 24
MSE_INIT = (1 << 32) // 12               # 1/12: what random phase gives
ACQ, RAMP, LOCK = 0, 1, 2


def rhu(x: int, s: int) -> int:
    """Round half up, arithmetic: floor(x / 2^s + 1/2)."""
    return x if s <= 0 else (x + (1 << (s - 1))) >> s


def sat(x: int, bits: int) -> int:
    return min(x, (1 << bits) - 1)


def fps_from_hz(f_hz) -> int:
    """Nominal NCO frequency word: round(f * 1e-12 * 2^56) in exact integer
    arithmetic (the Verilog wrappers compute the same from F_SRC_HZ)."""
    f = int(round(f_hz))
    return ((1 << FPS_FRAC) * f + 500_000_000_000) // 1_000_000_000_000


def fclk_of(fps: int) -> int:
    return (fps * T_CLK_PS) >> (FPS_FRAC - PH_FRAC)


def round_log2(x: int) -> int:
    """L = msb(x) + bit(msb-1): power-of-two rounding with the 1.5x break."""
    if x <= 0:
        return 0
    m = x.bit_length() - 1
    return m + ((x >> (m - 1)) & 1 if m >= 1 else 0)


@dataclass
class FixedCfg:
    # build parameters
    fps_nom: int = fps_from_hz(100e6)
    t_slot_ps: int = 10000          # nominal slot period, for residual_ps only
    D: int = 16                     # NCO time lag behind the TDC coarse counter
    J_MAX: int = 12                 # max cycles a hit may be popped after its edge
    POP_GAP: int = 2
    tau_init_log2: int = 16
    # runtime registers (defaults = reset values)
    kp_acq: int = 2
    kp_trk: int = 6
    gear_log2: int = 6
    gate_en: bool = True
    gate_thr: int = 1 << 13         # 2^-16 slot: 1/8
    mse_shift: int = 5
    mse_shift_trk: int = 7
    lock_thr: int = 1 << 26         # 2^-32 slot^2: 1/64
    unlock_thr: int = 1 << 28       # 1/16
    rej_max: int = 16
    to_shift: int = 5
    tau_ovr_en: bool = False
    tau_ovr: int = 0
    slip_thr: int = 1 << 13         # 2^-16 slot: 1/8
    slip_unlock: bool = True


TAU_SHIFT, SLIP_SHIFT = 4, 5
L_MAX = 30


@dataclass
class Hit:
    c: int                # TDC coarse count (48 bit), in the same time base as t_nco
    fine: int             # fine_ps
    a: int                # t_nco value of the cycle in which it was written into the FIFO
    tag: int = 0


class FixedNCO:
    """Event-driven but cycle-exact. P/fps/fclk are piecewise; events are
    applied in cycle order as the model advances."""

    def __init__(self, cfg: FixedCfg, n0: int, P0: int = 0):
        self.c = cfg
        # NCO registers at cycle self.n
        self.n = n0
        self.P = P0 & PH_MASK
        self.fps = cfg.fps_nom
        self.fclk = fclk_of(cfg.fps_nom)
        self.ev = {}                       # cycle -> [dP, dF] applied on entry to that cycle
        # control state
        self.state = ACQ
        self.kp, self.gear_cnt, self.gear_done = cfg.kp_acq, 0, False
        self.mse = MSE_INIT
        self.sgn = 0
        self.slip_armed = True
        self.rej = 0
        self.tavg, self.tavg_valid = 0, False
        self.c_prev, self.prev_valid = 0, False
        self.c_last, self.last_valid = 0, False
        self.n_prev_pop = None
        self.F_prev = None
        self.cnt = dict(unlock=0, suspect=0, abort=0, noise=0, late=0, hits=0)

    # ---------------- NCO register evolution ----------------
    def _add_event(self, cyc, dP=0, dF=0):
        e = self.ev.setdefault(cyc, [0, 0])
        e[0] += dP
        e[1] += dF

    def advance(self, n):
        """Move the NCO registers to cycle n (monotonic)."""
        assert n >= self.n
        while self.n < n:
            nxt = min([k for k in self.ev if k > self.n] + [n])
            # cycles self.n .. nxt-1 have constant fclk unless an fps change
            # at cycle self.n changes fclk at self.n+1: handle that by
            # stepping one cycle at a time around events.
            steps = nxt - self.n
            if self._fclk_pending is not None:
                steps = 1
            self.P = (self.P + self.fclk * steps) & PH_MASK
            self.n += steps
            if self._fclk_pending is not None:
                self.fclk = self._fclk_pending
                self._fclk_pending = None
            if self.n in self.ev:
                dP, dF = self.ev.pop(self.n)
                self.P = (self.P + dP) & PH_MASK
                if dF:
                    self.fps += dF
                    self._fclk_pending = fclk_of(self.fps)   # fclk[X+1] = f(fps[X])
        return self.P, self.fps

    _fclk_pending = None

    def ext_correction(self, X, dphase, dfreq):
        """External correction strobe during cycle X (must be issued in time order
        relative to hits processed afterwards)."""
        self._add_event(X + 1, dphase, dfreq)

    # ---------------- per-hit processing ----------------
    def L_cur(self):
        if self.c.tau_ovr_en:
            return self.c.tau_ovr
        if not self.tavg_valid:
            return self.c.tau_init_log2
        return min(round_log2(self.tavg >> 4), L_MAX)

    def _to_acq(self):
        if self.state == LOCK:
            self.cnt["unlock"] += 1
        elif self.state == RAMP:
            self.cnt["abort"] += 1
        self.state = ACQ
        self.kp, self.gear_cnt, self.gear_done = self.c.kp_acq, 0, False
        self.mse = MSE_INIT
        self.rej = 0

    def process(self, h: Hit):
        cfg = self.c
        n = max(h.c, h.a + 1)
        if self.n_prev_pop is not None:
            n = max(n, self.n_prev_pop + cfg.POP_GAP)
        self.n_prev_pop = n
        j = n - h.c
        if j > cfg.J_MAX:
            self.cnt["late"] = sat(self.cnt["late"] + 1, 16)
            return dict(late=True, n=n, c=h.c, tag=h.tag)
        E = n + 4
        # --- no-hit timeout (per-cycle check while E/F idle) ---
        if self.state != ACQ and self.last_valid:
            to = 1 << (self.L_cur() + cfg.to_shift)
            if (E - 1) - self.c_last > to:
                self._to_acq()
        # --- stage A..D: phase detector ---
        P, fps = self.advance(n)
        fine2 = h.fine + j * T_CLK_PS
        prod = fine2 * (fps >> MUL_SHIFT)                       # 2^-34 slot
        phi = (P - (prod << (PH_FRAC - 34))) & PH_MASK
        phi24 = phi >> (PH_FRAC - E_FRAC)
        t = phi24 + (1 << (E_FRAC - 1))
        slot = (t >> E_FRAC) & ((1 << SLOT_W) - 1)
        e24 = (t & ((1 << E_FRAC) - 1)) - (1 << (E_FRAC - 1))
        e16 = e24 >> 8
        sq = e16 * e16
        resid = rhu(e24 * cfg.t_slot_ps, E_FRAC)
        # --- stage E: gate, loop filter, averages ---
        st0 = self.state
        noise = st0 != ACQ and cfg.gate_en and abs(e24) >= (cfg.gate_thr << 8)
        L = self.L_cur()
        kp = self.kp
        if not noise:
            dphi = (-e24) << (PH_FRAC - E_FRAC - kp)
            df = rhu((-e24) << 32, 2 * kp + 1 + L + 12)
            self._add_event(n + 6, dphi, df)
            if self.prev_valid:
                d = sat((h.c - self.c_prev) & ((1 << 48) - 1), 32)
                if not self.tavg_valid:
                    self.tavg, self.tavg_valid = d << 4, True
                else:
                    self.tavg += rhu((d << 4) - self.tavg, TAU_SHIFT)
            self.c_prev, self.prev_valid = h.c, True
            if st0 != ACQ:
                if self.gear_cnt == (1 << cfg.gear_log2) - 1:
                    self.gear_cnt = 0
                    if self.kp < cfg.kp_trk:
                        self.kp += 1
                    else:
                        self.gear_done = True
                else:
                    self.gear_cnt += 1
        else:
            self.cnt["noise"] = sat(self.cnt["noise"] + 1, 16)
        w = cfg.mse_shift_trk if st0 == LOCK else cfg.mse_shift
        self.mse = self.mse + rhu(sq - self.mse, w)
        self.rej = sat(self.rej + 1, 8) if noise else 0
        sgn_cont = self.sgn + rhu(e24 - self.sgn, SLIP_SHIFT)
        sgn_new = rhu(e24, SLIP_SHIFT)
        # --- stage F: state machine ---
        just_locked = False
        if st0 == ACQ:
            if self.mse < cfg.lock_thr:
                self.state, self.gear_cnt, self.gear_done = RAMP, 0, False
        elif self.mse > cfg.unlock_thr or self.rej >= cfg.rej_max:
            self._to_acq()
        elif st0 == RAMP and self.gear_done and self.mse < cfg.lock_thr:
            self.state = LOCK
            just_locked = True
        if self.state == LOCK:
            self.sgn = sgn_new if just_locked else sgn_cont
            if just_locked:
                self.slip_armed = True
            thr = cfg.slip_thr << 8
            if self.slip_armed and abs(self.sgn) > thr:
                self.cnt["suspect"] = sat(self.cnt["suspect"] + 1, 16)
                self.slip_armed = False
                if cfg.slip_unlock:
                    self._to_acq()
            elif not self.slip_armed and abs(self.sgn) < (cfg.slip_thr << 7):
                self.slip_armed = True
        self.c_last, self.last_valid = h.c, True
        self.cnt["hits"] = sat(self.cnt["hits"] + 1, 16)
        return dict(late=False, n=n, c=h.c, j=j, tag=h.tag, slot=slot, e24=e24, resid=resid,
                    noise=bool(noise), locked=self.state == LOCK, state=self.state, kp=self.kp)

    def finish(self, N):
        """Account for a no-hit timeout that fires after the last hit, as seen
        by a register read in cycle N (the check runs in every idle cycle)."""
        if self.state != ACQ and self.last_valid:
            if (N - 1) - self.c_last > (1 << (self.L_cur() + self.c.to_shift)):
                self._to_acq()

    def registers_at(self, n):
        """NCO P, fps, fclk at cycle n (after all hits up to now)."""
        self.advance(n)
        return self.P, self.fps, self.fclk


def run_hits(cfg: FixedCfg, n0, P0, hits, ext=()):
    """Convenience: process hits (in FIFO order) and external corrections
    [(X, dphase, dfreq)] in time order. Returns (outputs, model)."""
    m = FixedNCO(cfg, n0, P0)
    ext = sorted(ext)
    out = []
    for h in hits:
        # corrections issued before this hit's pop cycle are registered first
        n_est = max(h.c, h.a + 1, (m.n_prev_pop + cfg.POP_GAP) if m.n_prev_pop is not None else 0)
        while ext and ext[0][0] < n_est:
            m.ext_correction(*ext.pop(0))
        out.append(m.process(h))
    for x in ext:
        m.ext_correction(*x)
    return out, m


# -----------------------------------------------------------------------------
# Long-run check against the float model (statistics, not bit-exactness)
# -----------------------------------------------------------------------------
def hits_from_stream(t_ps, t0_cycle=1000):
    import numpy as np
    import hitgen
    c, fine = hitgen.to_timestamps(np.asarray(t_ps), T_CLK_PS)
    c = c + t0_cycle
    return [Hit(int(ci), int(fi), int(ci) - 2) for ci, fi in zip(c, fine)]


if __name__ == "__main__":
    import sys
    from pathlib import Path
    import numpy as np
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import hitgen
    # Same stream through float and fixed models: they should agree on lock
    # and on every slot after lock (up to the constant slot offset).
    for f_src, ppm, p in ((100e6, 37.0, 1e-3), (500e6, -20.0, 2e-4), (100e6, -90.0, 1e-2)):
        s = hitgen.generate(hitgen.StreamCfg(f_src_hz=f_src, ppm=ppm, p_det=p, jitter_ps=200 if f_src < 2e8 else 60,
                                             dark_hz=0.1 * p * f_src, duration_s=4000 / (p * f_src), seed=9))
        cfg = FixedCfg(fps_nom=fps_from_hz(f_src), t_slot_ps=round(1e12 / f_src))
        hits = hits_from_stream(s["t_ps"])
        out, m = run_hits(cfg, n0=0, P0=0, hits=hits)
        lk = np.array([o["locked"] for o in out])
        sig = s["slot"] >= 0
        first = int(np.argmax(lk)) if lk.any() else None
        if first is None:
            print(f"f={f_src/1e6:.0f} MHz {ppm:+} ppm p={p}: NO LOCK")
            continue
        off = np.array([o["slot"] for o in out]) - s["slot"]
        ok = sig & lk
        offs = np.unique(off[ok] % (1 << 48))
        f_ppm = (m.fps / cfg.fps_nom - 1) * 1e6
        print(f"f={f_src/1e6:.0f} MHz {ppm:+} ppm p={p}: lock at hit {first}, "
              f"distinct slot offsets after lock {len(offs)}, final f {f_ppm:+.3f} ppm, counters {m.cnt}")
