"""Block 1 -> block 2 integration: behavioural carry-chain TDC feeding the NCO.

Photon pulses (12 ns, as from a SPAD module) are driven onto the TDC hit
input at the true arrival times of a 100 MHz source with a ppm offset. The
TDC's calibrated timestamps {coarse, fine_ps} go straight into nco_top
through the valid/ready interface, and nco_top's time base is the TDC's own
coarse counter. This proves, end to end:

  * the interface (widths, handshake, 48-bit coarse) and
  * the time convention  t_hit = coarse * T_clk - fine_ps :
    if the sign or the edge association were wrong the loop could not lock,
    and slots would not match the source.

Checks: (1) bit-exact NCO results vs model/loop_fixed.py fed with the TDC's
actual output timestamps; (2) after lock, every signal photon's slot equals
its true source slot + one constant (zero slips); (3) the NCO residual RMS is
consistent with jitter + TDC error. ALL RESULTS ARE SIMULATED.
"""

import json
import os
import sys
from pathlib import Path

import numpy as np

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer
from cocotb.utils import get_sim_time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "model"))
sys.path.insert(0, str(ROOT / "sim" / "nco"))
import hitgen  # noqa: E402
import loop_fixed as lfx  # noqa: E402
from nco_tb import default_cfg, sval, val  # noqa: E402

T_CLK_FS = 4_000_000
W_FS = 12_000_000          # SPAD-like pulse width (block 1 needs >= ~8.3 ns)


def now_fs():
    return int(get_sim_time(unit="fs"))


def drive_cfg(d, c):
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


@cocotb.test()
async def test_tdc_to_nco(dut):
    d = dut
    cfg = default_cfg()
    Clock(d.clk, T_CLK_FS, unit="fs", impl="gpi").start()
    drive_cfg(d, cfg)
    d.hit.value = 0
    d.rst.value = 1
    for _ in range(5):
        await RisingEdge(d.clk)
    d.rst.value = 0
    for _ in range(30):
        await RisingEdge(d.clk)
    await FallingEdge(d.clk)
    n0, P0 = val(d.t_nco), val(d.tlm_phase)
    # time origin: the rising edge after which the coarse counter read c_ref
    await RisingEdge(d.clk)
    await ReadOnly()
    t_ref, c_ref = now_fs(), val(d.coarse_now)

    # --- interface monitor: TDC -> NCO handshakes, and NCO outputs ---
    ifc, outs = [], []

    async def mon_ifc():
        while True:
            await RisingEdge(d.clk)
            await ReadOnly()
            if val(d.tdc_valid) and val(d.nco_ready):
                ifc.append(lfx.Hit(val(d.tdc_coarse), val(d.tdc_fine_ps), val(d.t_nco)))

    async def mon_out():
        while True:
            await RisingEdge(d.clk)
            await ReadOnly()
            if val(d.m_valid):
                outs.append(dict(slot=val(d.m_slot), e24=sval(d.m_e24, 25),
                                 resid=sval(d.m_resid_ps, 16), noise=bool(val(d.m_noise)),
                                 locked=bool(val(d.m_locked)), c=val(d.m_coarse), tag=0))
    cocotb.start_soon(mon_ifc())
    cocotb.start_soon(mon_out())

    # --- photon stream: 100 MHz, +23 ppm, 2 M photons/s offered, 150 ps jitter
    ppm = 23.0
    s = hitgen.generate(hitgen.StreamCfg(f_src_hz=100e6, ppm=ppm, p_det=0.02, jitter_ps=150.0,
                                         dark_hz=1e5, duration_s=0.6e-3, seed=77,
                                         tdc_rms_ps=0.0))   # the RTL TDC adds its own error
    # drive pulses; the 12 ns pulses need >= 16.75 ns spacing (dead time),
    # which hitgen's dead-time filter already guarantees
    t0 = now_fs() + 200 * T_CLK_FS
    times = [t0 + int(round(t * 1000)) for t in s["t_ps"]]
    for t in times:
        dt = t - now_fs()
        await Timer(dt, unit="fs")
        d.hit.value = 1
        await Timer(W_FS, unit="fs")
        d.hit.value = 0
    for _ in range(80):
        await RisingEdge(d.clk)
    await FallingEdge(d.clk)
    N = val(d.t_nco)

    # --- (1) bit-exact vs the fixed model fed with the TDC's real output ---
    mod_outs, m = lfx.run_hits(cfg, n0, P0, ifc)
    m.finish(N)
    mod = [o for o in mod_outs if not o["late"]]
    assert len(mod) == len(outs), (len(mod), len(outs))
    for i, (r, o) in enumerate(zip(outs, mod)):
        for k in ("slot", "e24", "resid", "noise", "locked"):
            assert r[k] == o[k], f"hit {i} {k}: RTL {r[k]} model {o[k]}"
    P, fps, _ = m.registers_at(N)
    assert val(d.tlm_phase) == P and sval(d.tlm_fps, 48) == fps

    # --- (2) truth: match each NCO output to its driven pulse by time ---
    # capture edge of a pulse at t: first edge at/after t -> coarse c_ref + ceil((t - t_ref)/T)
    exp_c = np.array([c_ref + -(-(t - t_ref) // T_CLK_FS) for t in times])
    slot_true = s["slot"]
    pairs = []
    for o in outs:
        k = int(np.argmin(np.abs(exp_c - o["c"])))
        if abs(int(exp_c[k]) - o["c"]) <= 1:
            pairs.append((o, int(slot_true[k])))
    lk = [(o, t) for o, t in pairs if o["locked"] and t >= 0]
    offs = {(o["slot"] - t) % (1 << 48) for o, t in lk}
    first = next((i for i, o in enumerate(outs) if o["locked"]), None)
    f_ppm = (sval(d.tlm_fps, 48) / cfg.fps_nom - 1) * 1e6
    rms = float(np.sqrt(np.mean([o["resid"] ** 2 for o, t in lk if not o["noise"]])))
    res = dict(pulses=len(times), tdc_timestamps=len(ifc), nco_results=len(outs),
               first_lock_hit=first, locked_signal=len(lk), distinct_slot_offsets=len(offs),
               f_ppm=f_ppm, f_true_ppm=ppm, rms_resid_ps=rms, counters=m.cnt,
               tdc_drop=val(d.tdc_drop_cnt), tdc_short=val(d.tdc_short_cnt),
               tdc_sat=val(d.tdc_sat_cnt), tdc_rej=val(d.tdc_rej_cnt))
    dut._log.info("SIMULATED TDC->NCO integration: %s", res)
    out = ROOT / "sim" / "out" / os.environ.get("TDC_CONFIG", "default")
    out.mkdir(parents=True, exist_ok=True)
    (out / "integration_tdc_nco.json").write_text(json.dumps(res, indent=1, default=float))
    assert first is not None, "never locked"
    assert len(offs) == 1, f"slot offset changed while locked: {offs}"
    assert len(lk) > 300
    assert abs(f_ppm - ppm) < 3.0
    assert rms < 250.0, f"residual RMS {rms:.0f} ps"
