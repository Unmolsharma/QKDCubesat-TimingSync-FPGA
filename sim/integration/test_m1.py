"""M1 rehearsal in simulation: the whole board top (m1_arty_top) against an
emulated satellite on its own clock.

  emulator clock 100 MHz +20 ppm (independent of the 250 MHz board clock)
  -> sat_emulator (LFSR, p = 1e-2, 15 ns pulses) -> jumper (1.5 ns)
  -> tdc_top (behavioural carry chain) -> nco_top -> UART STATUS + TELEMETRY

Timeline (sw[1:0] = 01: p = 1e-2):
  0       reset released, emulator running        -> acquire, lock
  1.2 ms  sw[2] = 1: all pulses dropped (gap)     -> unlock reported
  1.7 ms  sw[2] = 0: pulses back                  -> re-acquire
  3.2 ms  end (the run must reach LOCK again, and the last telemetry
          frame must show LOCK)

PASS CRITERIA (docs/M1_DEMO.md):
  1. locks within the time the float model predicts (~420 hits; here <= 0.8 ms)
  2. frequency estimate converges to the emulator's true offset, +20.00 ppm:
     average over the locked hold within +/-0.5 ppm
  3. EVERY emitted-and-detected pulse gets its true slot + one constant inside
     each lock run: zero slips
  4. gap -> unlock reported (unlock_cnt, LED, telemetry) -> re-acquire
  plus: NCO results bit-exact vs model/loop_fixed.py fed with the TDC's real
  output, and the UART telemetry decodes (capture.py parser) to the same story.
ALL RESULTS ARE SIMULATED.
"""

import json
import os
import sys
from pathlib import Path

import numpy as np

import cocotb
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer
from cocotb.utils import get_sim_time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "model"))
sys.path.insert(0, str(ROOT / "sim" / "nco"))
sys.path.insert(0, str(ROOT / "scripts"))
import loop_fixed as lfx  # noqa: E402
from capture import T_STATUS, T_TLM, FrameParser  # noqa: E402
from nco_tb import default_cfg, sval, val  # noqa: E402

PPM_TRUE = 20.0
T_CLK_FS = 4_000_000
BIT_FS = 125 * T_CLK_FS
US = 1_000_000_000            # fs


def now_fs():
    return int(get_sim_time(unit="fs"))


@cocotb.test()
async def test_m1_rehearsal(dut):
    m1 = dut.u_m1
    nco = m1.u_nco
    emu = m1.u_emu
    cfg = default_cfg()
    dut.sw.value = 0b0001             # p = 1e-2, no gap, no raw timestamps
    dut.uart_txd_in.value = 1         # command line idle (the FT2232H drives it high)
    dut.ck_rst.value = 0              # CPU_RESET pressed
    await Timer(300, unit="ns")       # MMCM stand-in locks at 100 ns
    clk = m1.clk
    for _ in range(10):
        await RisingEdge(clk)
    dut.ck_rst.value = 1
    for _ in range(40):
        await RisingEdge(clk)
    await FallingEdge(clk)
    n0, P0 = val(nco.t_nco), val(nco.tlm_phase)
    await RisingEdge(clk)
    await ReadOnly()
    t_ref, c_ref = now_fs(), val(m1.coarse_now)

    ifc, outs, emits, rx = [], [], [], bytearray()
    fps_locked = []

    async def mon_clk():
        while True:
            await RisingEdge(clk)
            await ReadOnly()
            if val(m1.tdc_valid) and val(m1.tdc_ready):
                cal = val(m1.m_cal)
                ifc.append(lfx.Hit(cal >> 16, cal & 0xFFFF, val(nco.t_nco)))
            if val(nco.m_valid):
                outs.append(dict(slot=val(nco.m_slot), e24=sval(nco.m_e24, 25),
                                 resid=sval(nco.m_resid_ps, 16), noise=bool(val(nco.m_noise)),
                                 locked=bool(val(nco.m_locked)), c=val(nco.m_coarse),
                                 t=now_fs()))
                if val(nco.m_locked):
                    fps_locked.append((now_fs(), sval(nco.tlm_fps, 48)))

    async def mon_emu():
        while True:
            await RisingEdge(emu.emit_valid)
            await ReadOnly()
            emits.append((now_fs(), val(emu.emit_slot)))

    async def uart_rx():
        tx = dut.uart_rxd_out
        from cocotb.triggers import FallingEdge as FE
        while True:
            await FE(tx)
            await Timer(BIT_FS + BIT_FS // 2, unit="fs")
            b = 0
            for i in range(8):
                b |= val(tx) << i
                await Timer(BIT_FS, unit="fs")
            rx.append(b)

    cocotb.start_soon(mon_clk())
    cocotb.start_soon(mon_emu())
    cocotb.start_soon(uart_rx())

    t_start = now_fs()
    await Timer(1200, unit="us")
    t_gap0 = now_fs()
    dut.sw.value = 0b0101             # sw[2] = 1: drop all pulses
    await Timer(500, unit="us")
    t_gap1 = now_fs()
    dut.sw.value = 0b0001
    await Timer(1500, unit="us")
    for _ in range(60):
        await RisingEdge(clk)
    await FallingEdge(clk)
    N = val(nco.t_nco)

    # --- bit-exact vs the fixed model, fed with the TDC's real timestamps ---
    mod_outs, m = lfx.run_hits(cfg, n0, P0, ifc)
    m.finish(N)
    mod = [o for o in mod_outs if not o["late"]]
    assert len(mod) == len(outs), (len(mod), len(outs))
    for i, (r, o) in enumerate(zip(outs, mod)):
        for k in ("slot", "e24", "resid", "noise", "locked"):
            assert r[k] == o[k], f"hit {i} {k}: RTL {r[k]} model {o[k]}"

    # --- truth: each NCO output <- the emulator pulse that caused it ---
    # pulse rise at t (emulator edge) + 1.5 ns jumper -> first board edge after it
    em_t = np.array([t for t, _ in emits], dtype=np.int64)
    em_s = np.array([s for _, s in emits], dtype=np.int64)
    exp_c = c_ref + -(-(em_t + 1_500_000 - t_ref) // T_CLK_FS)
    runs, cur, slips, locked_sig, in_run = 0, None, 0, 0, False
    matched = 0
    for o in outs:
        k = int(np.searchsorted(exp_c, o["c"]))
        cand = [j for j in (k - 1, k, k + 1) if 0 <= j < len(exp_c) and abs(int(exp_c[j]) - o["c"]) <= 1]
        if not o["locked"]:
            in_run, cur = False, None
            continue
        if not in_run:
            runs, in_run = runs + 1, True
        if not cand:
            continue
        matched += 1
        locked_sig += 1
        off = (o["slot"] - int(em_s[cand[0]])) % (1 << 48)
        if cur is not None and off != cur:
            slips += 1
        cur = off

    first = next((i for i, o in enumerate(outs) if o["locked"]), None)
    t_lock_us = (outs[first]["t"] - t_start) / US if first is not None else None
    relock = next((o for o in outs if o["locked"] and o["t"] > t_gap1), None)
    t_relock_us = (relock["t"] - t_gap1) / US if relock else None
    # frequency: average of the NCO word over the locked hold before the gap
    # (skip the first 200 us after lock: the tracking gear is still settling)
    hold = [f for t, f in fps_locked if outs[first]["t"] + 200 * US < t < t_gap0]
    f_ppm_hold = (np.mean(hold) / cfg.fps_nom - 1) * 1e6 if hold else None
    f_ppm_rms = float(np.std([(f / cfg.fps_nom - 1) * 1e6 for f in hold])) if hold else None

    # --- UART telemetry, decoded with the lab parser ---
    p = FrameParser()
    frames = p.feed(bytes(rx))
    tlm = [f for t, f in frames if t == T_TLM and f["kind"] == "telemetry"]
    st = [f for t, f in frames if t == T_STATUS]
    tlm_lock_ppm = [f["ppm"] for f in tlm if f["state_name"] == "LOCK"]

    res = dict(
        emitted=len(emits), emu_supp=val(emu.supp_cnt), tdc_timestamps=len(ifc),
        nco_results=len(outs), counters=m.cnt,
        lock_time_us=t_lock_us, lock_hits=(first + 1) if first is not None else None,
        relock_after_gap_us=t_relock_us, lock_runs=runs, slips=slips,
        locked_signal_checked=locked_sig,
        f_ppm_hold_mean=f_ppm_hold, f_ppm_hold_rms=f_ppm_rms, f_true_ppm=PPM_TRUE,
        uart_frames=len(frames), tlm_frames=len(tlm), parser_resyncs=p.resyncs,
        tlm_states=[f["state_name"] for f in tlm], tlm_lock_ppm=tlm_lock_ppm,
        tlm_last_unlock=tlm[-1]["unlock"] if tlm else None,
        status_last=st[-1] if st else None, leds=val(dut.led))
    dut._log.info("SIMULATED M1 rehearsal: %s", res)
    out = ROOT / "sim" / "out" / os.environ.get("TDC_CONFIG", "default")
    out.mkdir(parents=True, exist_ok=True)
    (out / "m1_rehearsal.json").write_text(json.dumps(res, indent=1, default=float))

    # emulator sanity: a slot right after an emitted one is suppressed with
    # probability p, so suppressed / emitted ~ p (1e-2). The LFSR version
    # clustered: 706 / 1464.
    assert val(emu.supp_cnt) < 3 * 0.01 * len(emits) + 10, "emulator slots are clustered"
    assert first is not None, "never locked"
    assert t_lock_us <= 800, f"lock took {t_lock_us:.0f} us"
    assert abs(f_ppm_hold - PPM_TRUE) < 0.5, f"frequency {f_ppm_hold:+.3f} ppm"
    assert slips == 0, f"{slips} slips while locked"
    assert m.cnt["unlock"] >= 1, "gap not reported"
    assert runs >= 2 and outs[-1]["locked"], "no re-acquisition after the gap"
    assert p.resyncs == 0 and tlm, "telemetry frames missing or corrupt"
    assert st[-1]["drop"] == 0 and st[-1]["sat"] == 0 and st[-1]["short"] == 0, st[-1]
    assert tlm[-1]["state_name"] == "LOCK" and tlm[-1]["unlock"] >= 1
