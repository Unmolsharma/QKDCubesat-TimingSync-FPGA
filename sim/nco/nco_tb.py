"""cocotb helpers for the block 2 (NCO) testbenches.

The testbench drives TDC-style timestamps {coarse, fine_ps} into nco_top at
the cycle a real TDC would deliver them (LAT clocks after the capture edge),
records every per-hit result, and replays the SAME inputs through
model/loop_fixed.py. Every hit must match bit for bit: slot, phase error,
residual, noise flag and lock flag. At the end the NCO phase/frequency
registers and the counters are compared too.

Independently, the true source slot of each signal photon (from
model/hitgen.py) is used to check that the slot numbering never slips while
locked. ALL RESULTS ARE SIMULATED.
"""

import os
import sys
from pathlib import Path

import numpy as np

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "model"))
import hitgen  # noqa: E402
import loop_fixed as lfx  # noqa: E402

T_CLK_NS = 4
LAT = 9                    # TDC capture edge -> timestamp delivered (block 1: 8-9 clk)
F_SRC = float(os.environ.get("NCO_FSRC", "100e6"))


def val(sig):
    return int(sig.value)


def sval(sig, bits):
    v = int(sig.value)
    return v - (1 << bits) if v >> (bits - 1) else v


def default_cfg(**kw):
    c = lfx.FixedCfg(fps_nom=lfx.fps_from_hz(F_SRC), t_slot_ps=round(1e12 / F_SRC))
    for k, v in kw.items():
        setattr(c, k, v)
    return c


class NcoTB:
    def __init__(self, dut, cfg=None):
        self.dut = dut
        self.cfg = cfg or default_cfg()
        self.outs = []          # RTL results in order
        self.hits = []          # lfx.Hit as driven
        self.truth = []         # true slot per driven hit (-1 = dark count)
        self.ext = []           # (X, dphase, dfreq)
        self._mon = None

    def _drive_cfg(self):
        d, c = self.dut, self.cfg
        d.cfg_kp_acq.value = c.kp_acq
        d.cfg_kp_trk.value = c.kp_trk
        d.cfg_gear_log2.value = c.gear_log2
        d.cfg_gate_en.value = int(c.gate_en)
        d.cfg_gate_thr.value = c.gate_thr
        d.cfg_mse_shift.value = c.mse_shift
        d.cfg_mse_shift_trk.value = c.mse_shift_trk
        d.cfg_lock_thr.value = c.lock_thr
        d.cfg_unlock_thr.value = c.unlock_thr
        d.cfg_rej_max.value = c.rej_max
        d.cfg_to_shift.value = c.to_shift
        d.cfg_tau_ovr_en.value = int(c.tau_ovr_en)
        d.cfg_tau_ovr.value = c.tau_ovr
        d.cfg_slip_thr.value = c.slip_thr
        d.cfg_slip_unlock.value = int(c.slip_unlock)

    async def start(self, clock=True):
        """Reset and snapshot. clock=True starts the clock: do that once per
        cocotb test (cocotb stops it when the test ends); later resets in the
        same test pass clock=False. The GPI (C-level) clock is ~3x faster than
        cocotb's default Python clock."""
        d = self.dut
        if clock:
            Clock(d.clk, T_CLK_NS, unit="ns", impl="gpi").start()
        self._drive_cfg()
        d.s_valid.value = 0
        d.s_coarse.value = 0
        d.s_fine.value = 0
        d.s_tag.value = 0
        d.m_ready.value = 1
        d.ext_valid.value = 0
        d.ext_dphase.value = 0
        d.ext_dfreq.value = 0
        d.rst.value = 1
        for _ in range(5):
            await RisingEdge(d.clk)
        d.rst.value = 0
        # let t_nco = time_now - D become non-negative before the snapshot
        for _ in range(24):
            await RisingEdge(d.clk)
        await FallingEdge(d.clk)
        self.n0 = val(d.t_nco)
        self.P0 = val(d.tlm_phase)
        self.time0 = val(d.time_now)
        if self._mon is None:
            self._mon = cocotb.start_soon(self._monitor())

    async def _monitor(self):
        d = self.dut
        while True:
            await RisingEdge(d.m_valid)
            await ReadOnly()
            while True:
                if val(d.m_ready):
                    self.outs.append(dict(slot=val(d.m_slot), e24=sval(d.m_e24, 25),
                                          resid=sval(d.m_resid_ps, 16), noise=bool(val(d.m_noise)),
                                          locked=bool(val(d.m_locked)), tag=val(d.m_tag),
                                          c=val(d.m_coarse)))
                await RisingEdge(d.clk)
                await ReadOnly()
                if not val(d.m_valid):
                    break

    async def wait_until_time(self, target_time_now):
        """Return at the falling edge where time_now == target (or now, if past)."""
        d = self.dut
        await FallingEdge(d.clk)
        now = val(d.time_now)
        if target_time_now > now:
            # land 1 ns before the target falling edge, then sync to it (a
            # Timer that lands exactly on the edge races with the clock)
            await Timer((target_time_now - now) * T_CLK_NS - 1, unit="ns")
            await FallingEdge(d.clk)
        return val(d.time_now)

    async def drive(self, coarse, fine, truth=None, tag=0, lat=LAT):
        """Present one timestamp `lat` clocks after its capture edge."""
        d = self.dut
        await self.wait_until_time(coarse + lat)
        assert val(d.s_ready), "align FIFO full"
        a = val(d.t_nco)
        d.s_valid.value = 1
        d.s_coarse.value = coarse & ((1 << 48) - 1)
        d.s_fine.value = fine
        d.s_tag.value = tag
        await FallingEdge(d.clk)
        d.s_valid.value = 0
        self.hits.append(lfx.Hit(coarse, fine, a, tag))
        self.truth.append(-1 if truth is None else truth)

    async def drive_stream(self, t_ps, slots, t_start_cycle=None):
        """Drive arrival times (ps, relative) as TDC timestamps."""
        d = self.dut
        await FallingEdge(d.clk)
        base = (val(d.time_now) + 40) if t_start_cycle is None else t_start_cycle
        c, f = hitgen.to_timestamps(np.asarray(t_ps), hitgen.T_CLK_PS)
        last = None
        for ci, fi, si in zip(c, f, slots):
            ci = int(ci) + base
            when = ci + LAT if last is None else max(ci + LAT, last + 1)
            await self.drive(ci, int(fi), truth=int(si), lat=when - ci)
            last = when
        return base

    async def ext_correction(self, dphase, dfreq):
        d = self.dut
        await FallingEdge(d.clk)
        X = val(d.t_nco)
        d.ext_valid.value = 1
        d.ext_dphase.value = dphase & ((1 << 48) - 1)
        d.ext_dfreq.value = dfreq & ((1 << 48) - 1)
        await FallingEdge(d.clk)
        d.ext_valid.value = 0
        self.ext.append((X, dphase, dfreq))

    async def settle(self, cycles=40):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk)

    # ------------------------------------------------------------------
    async def check_model(self, log_name=""):
        """Bit-exact comparison of every hit, the counters and the NCO registers."""
        d = self.dut
        await self.settle()
        await FallingEdge(d.clk)
        N = val(d.t_nco)
        rtl_P, rtl_fps = val(d.tlm_phase), sval(d.tlm_fps, 48)
        rtl_cnt = dict(unlock=val(d.unlock_cnt), abort=val(d.abort_cnt), suspect=val(d.suspect_cnt),
                       noise=val(d.noise_cnt), late=val(d.late_cnt), hits=val(d.hit_cnt))
        rtl_state = val(d.tlm_state)
        outs, m = lfx.run_hits(self.cfg, self.n0, self.P0, self.hits, self.ext)
        m.finish(N)
        mod = [o for o in outs if not o["late"]]
        d._log.info("RTL counters %s state %d out_drop %d | model counters %s",
                    rtl_cnt, rtl_state, val(d.out_drop_cnt), m.cnt)
        assert len(mod) == len(self.outs), f"{len(self.outs)} RTL results vs {len(mod)} model results"
        for i, (r, o) in enumerate(zip(self.outs, mod)):
            for k in ("slot", "e24", "resid", "noise", "locked", "tag"):
                assert r[k] == o[k], (f"hit {i}: {k} RTL {r[k]} != model {o[k]}\n RTL {r}\n model {o}")
        P, fps, _ = m.registers_at(N)
        assert rtl_P == P, f"NCO phase at t_nco={N}: RTL {rtl_P:#x} != model {P:#x}"
        assert rtl_fps == fps, f"NCO fps: RTL {rtl_fps} != model {fps}"
        assert rtl_cnt == m.cnt, f"counters RTL {rtl_cnt} != model {m.cnt}"
        assert rtl_state == m.state, f"state RTL {rtl_state} != model {m.state}"
        d._log.info("BIT-EXACT %s: %d hits (%d late) match loop_fixed.py; counters %s",
                    log_name, len(self.outs), m.cnt["late"], rtl_cnt)
        return m

    def truth_check(self):
        """Compare against the true source slot of every signal photon.

        A LOCK RUN is a stretch of consecutive outputs with locked = 1. Inside
        one run, every signal photon must have the same offset
        (output slot - true slot): a change is a cycle slip the loop did not
        report. Across runs the offset may change (the unlock was reported).
        Assumes no late hits (outputs and driven hits line up)."""
        assert len(self.outs) == len(self.truth)
        first_lock = next((i for i, o in enumerate(self.outs) if o["locked"]), None)
        runs, cur, slips, locked_sig = 0, None, 0, 0
        in_run = False
        for o, t in zip(self.outs, self.truth):
            if not o["locked"]:
                in_run, cur = False, None
                continue
            if not in_run:
                runs, in_run = runs + 1, True
            if t < 0:
                continue
            locked_sig += 1
            off = (o["slot"] - t) % (1 << 48)
            if cur is not None and off != cur:
                slips += 1
            cur = off
        return dict(first_lock=first_lock, slips=slips, lock_runs=runs, locked_signal=locked_sig)

    def f_ppm(self):
        return (sval(self.dut.tlm_fps, 48) / self.cfg.fps_nom - 1) * 1e6
