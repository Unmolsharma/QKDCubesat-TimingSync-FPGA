"""UART command path -> NCO runtime settings.

For every setting the test
  1. writes it over the real UART line (2 Mbaud, frames built by
     scripts/nco_config.py, the helper used in the lab),
  2. checks the READBACK frame the board sends back (decoded with
     scripts/capture.py's parser) and the value on nco_top's cfg input,
  3. drives the same photon stream as every other case, and checks that the
     NCO results are bit-exact with model/loop_fixed.py configured with that
     value AND differ from the default-configuration model, i.e. the NCO really
     uses the new value.
Plus: the ID register, and that a bad checksum, a write to a read-only
register and an unknown command are rejected with the right status.
ALL RESULTS ARE SIMULATED.
"""

import json
import os
import sys
from dataclasses import replace
from pathlib import Path

import cocotb
from cocotb.triggers import FallingEdge, Timer

from nco_tb import NcoTB, default_cfg, val, ROOT  # puts model/ on sys.path
import hitgen  # noqa: E402
import loop_fixed as lfx  # noqa: E402

sys.path.insert(0, str(ROOT / "scripts"))
import nco_config as nc  # noqa: E402
from capture import T_TLM, FrameParser  # noqa: E402

BIT_NS = 500                     # 2 Mbaud


class CmdTB(NcoTB):
    """NcoTB whose configuration comes from the UART command path."""

    def _drive_cfg(self):
        d = self.dut
        d.uart_rx_line.value = 1     # idle line; config = reset defaults

    async def start(self, clock=True):
        await super().start(clock)
        self.rx = bytearray()
        cocotb.start_soon(self._uart_mon())

    async def _uart_mon(self):
        tx = self.dut.uart_tx_line
        while True:
            await FallingEdge(tx)
            await Timer(BIT_NS + BIT_NS // 2, unit="ns")
            b = 0
            for i in range(8):
                b |= val(tx) << i
                await Timer(BIT_NS, unit="ns")
            self.rx.append(b)

    async def send(self, frame):
        line = self.dut.uart_rx_line
        for byte in frame:
            line.value = 0
            await Timer(BIT_NS, unit="ns")
            for i in range(8):
                line.value = (byte >> i) & 1
                await Timer(BIT_NS, unit="ns")
            line.value = 1
            await Timer(BIT_NS, unit="ns")

    async def transact(self, frame):
        """Send a command, return its readback dict."""
        n0 = len(self.rx)
        await self.send(frame)
        for _ in range(400):                       # up to 200 us for the 25-byte reply
            await Timer(500, unit="ns")
            fr = FrameParser().feed(bytes(self.rx[n0:]))
            rb = [f for t, f in fr if t == T_TLM and f["kind"] == "readback"]
            if rb:
                return rb[0]
        raise AssertionError("no readback frame")


def cfg_from_words(words):
    """FixedCfg from register words (what the model needs to replay the RTL)."""
    g = lambda n: nc.get_field(words[nc.FIELDS[n][0]], n)
    return default_cfg(gate_thr=round(g("gate") * 65536), gate_en=bool(g("gate_en")),
                       lock_thr=round(g("lock") * 2 ** 32), unlock_thr=round(g("unlock") * 2 ** 32),
                       mse_shift=g("mse_shift"), mse_shift_trk=g("mse_shift_trk"),
                       tau_ovr=g("tau_L"), tau_ovr_en=bool(g("tau_en")),
                       kp_acq=g("kp_acq"), kp_trk=g("kp_trk"), gear_log2=g("gear"),
                       rej_max=g("rej_max"), to_shift=g("timeout"),
                       slip_thr=round(g("slip") * 65536), slip_unlock=bool(g("slip_unlock")))


# (register, settings to apply, nco_top port checks, what must change)
CASES = [
    ("GATE",    [("gate", 1 / 32)],            {"cfg_gate_thr": 0x0800}, "outputs"),
    ("GATE",    [("gate_en", 0)],              {"cfg_gate_en": 0}, "outputs"),
    ("LOCK",    [("lock", 1 / 512)],           {"cfg_lock_thr": 1 << 23}, "outputs"),
    ("UNLOCK",  [("unlock", 1 / 128)],         {"cfg_unlock_thr": 1 << 25}, "outputs"),
    ("AVG",     [("mse_shift", 7), ("mse_shift_trk", 9)], {"cfg_mse_shift": 7, "cfg_mse_shift_trk": 9}, "outputs"),
    ("TAU",     [("tau_L", 2), ("tau_en", 1)], {"cfg_tau_ovr": 2, "cfg_tau_ovr_en": 1}, "outputs"),
    ("GEARS",   [("kp_acq", 3), ("kp_trk", 5), ("gear", 4)],
                {"cfg_kp_acq": 3, "cfg_kp_trk": 5, "cfg_gear_log2": 4}, "outputs"),
    ("REJ",     [("rej_max", 1)],              {"cfg_rej_max": 1}, "outputs"),
    ("TIMEOUT", [("timeout", 0)],              {"cfg_to_shift": 0}, "outputs"),
    ("SLIP",    [("slip", 1 / 1024)],          {"cfg_slip_thr": 64}, "outputs"),
    ("SLIP",    [("slip", 1 / 1024), ("slip_unlock", 0)], {"cfg_slip_thr": 64, "cfg_slip_unlock": 0},
                "suspect_only"),
]


@cocotb.test()
async def test_cmd_protocol(dut):
    """ID register and error handling."""
    tb = CmdTB(dut)
    await tb.start()
    rb = await tb.transact(nc.read_cmd(nc.ADDR["ID"]))
    assert rb["value"] == nc.ID_VALUE and rb["status"] == 0, rb
    bad = bytearray(nc.write_cmd(0x00, 0x1234))
    bad[-1] ^= 0xFF
    rb = await tb.transact(bytes(bad))
    assert rb["status"] == 1, rb
    assert val(dut.u_nco.cfg_gate_thr) == 0x2000, "bad-checksum write was applied"
    rb = await tb.transact(nc.write_cmd(nc.ADDR["ID"], 0))
    assert rb["status"] == 2, rb
    rb = await tb.transact(nc.command(0x58, 0x01, 0))
    assert rb["status"] == 3, rb
    for a, w in nc.DEFAULT_WORDS.items():                  # reset values readable
        rb = await tb.transact(nc.read_cmd(a))
        assert rb["value"] == w and rb["status"] == 0, (hex(a), rb)
    dut._log.info("SIMULATED command protocol: ID 0x%08X, bad checksum / read-only / bad cmd "
                  "rejected (status 1/2/3), all reset values read back", nc.ID_VALUE)


@cocotb.test()
async def test_cmd_sets_nco(dut):
    s = hitgen.generate(hitgen.StreamCfg(f_src_hz=100e6, ppm=20.0, p_det=0.02, jitter_ps=300.0,
                                         dark_hz=0.1 * 2e6, duration_s=0.6e-3, seed=91))
    results = {}
    for i, (reg, settings, ports, expect) in enumerate(CASES):
        tb = CmdTB(dut)
        await tb.start(clock=(i == 0))
        words = dict(nc.DEFAULT_WORDS)
        addr = nc.ADDR[reg]
        for name, v in settings:
            words[addr] = nc.set_field(words[addr], name, v)
        rb = await tb.transact(nc.write_cmd(addr, words[addr]))
        assert rb["status"] == 0 and rb["value"] == words[addr], (reg, rb, hex(words[addr]))
        rb = await tb.transact(nc.read_cmd(addr))
        assert rb["value"] == words[addr], (reg, rb)
        for port, want in ports.items():
            got = val(getattr(dut.u_nco, port))
            assert got == want, f"{reg}: nco_top.{port} = {got}, expected {want}"
        await tb.drive_stream(s["t_ps"], s["slot"])
        tb.cfg = cfg_from_words(words)
        m = await tb.check_model(f"{reg} {settings}")       # bit-exact with the new setting
        ref, mref = lfx.run_hits(default_cfg(), tb.n0, tb.P0, tb.hits)
        key = lambda o: (o["slot"], o["e24"], o["noise"], o["locked"])
        same = [key(o) for o in ref if not o["late"]] == [key(o) for o in tb.outs]
        if expect == "outputs":
            assert not same, f"{reg} {settings}: results identical to the default configuration"
        else:   # suspected slips counted, but no forced unlock
            assert m.cnt["suspect"] > 0 and m.cnt["unlock"] == 0, m.cnt
        results[f"{reg} {settings}"] = dict(word=f"0x{words[addr]:08X}", differs=not same,
                                           counters=m.cnt, locked_end=tb.outs[-1]["locked"])
        dut._log.info("SIMULATED cmd %s %s -> word 0x%08X, NCO uses it (bit-exact, differs from "
                      "default: %s, counters %s)", reg, settings, words[addr], not same, m.cnt)
    out = ROOT / "sim" / "out" / os.environ.get("TDC_CONFIG", "default")
    out.mkdir(parents=True, exist_ok=True)
    (out / "nco_cmd_results.json").write_text(json.dumps(results, indent=1, default=str))
