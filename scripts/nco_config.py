#!/usr/bin/env python3
"""Read and set the NCO runtime settings on the board over the USB-UART.

No rebuild needed: the M1 top (rtl/top/m1_arty_top.v) receives 8-byte
commands on the Arty's USB-UART RX pin (A9, 2 Mbaud 8N1) and answers each
with a READBACK frame on the normal telemetry stream (rtl/top/nco_cfg_regs.v
documents the protocol and the register map).

EXAMPLES
  python scripts/nco_config.py --port COM5 show
  python scripts/nco_config.py --port COM5 set gate=0.25 lock=0.05 unlock=0.07
  python scripts/nco_config.py --port COM5 set mse_shift=6 tau=off
  python scripts/nco_config.py --port COM5 set kp_trk=4          # Doppler at low rates
  python scripts/nco_config.py --port COM5 defaults

SETTINGS (human units -> register fields)
  gate=<slot>       noise gate half-width (default 0.125)    reg 0x00 [15:0], 2^-16 slot
  gate_en=0|1       noise gate on/off (default 1)            reg 0x00 [16]
  lock=<slot^2>     lock threshold on the e^2 average (1/64) reg 0x01, 2^-32 slot^2
  unlock=<slot^2>   unlock threshold (1/16)                  reg 0x02
  mse_shift=<n>     e^2 average weight 2^-n while acquiring (5)   reg 0x03 [3:0]
  mse_shift_trk=<n> ... once locked (7)                      reg 0x03 [11:8]
  tau=off|<L>       hit-spacing override: off = measured, L = force 2^L clocks   reg 0x04
  kp_acq=<n> kp_trk=<n> gear=<n>   gears Kp = 2^-n (2, 6), ramp step 2^gear updates (6)  reg 0x05
  rej_max=<n>       consecutive gated hits -> unlock (16)    reg 0x06
  timeout=<n>       no-hit timeout 2^(L+n) clocks (5)        reg 0x07
  slip=<slot>       suspected-slip threshold (0.125)         reg 0x08 [15:0]
  slip_unlock=0|1   suspected slip forces unlock (1)         reg 0x08 [16]
Changes apply from the next hit (see nco_cfg_regs.v for what is safe while locked).
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture import T_TLM, FrameParser  # noqa: E402

SYNC, CMD_W, CMD_R = 0xA5, 0x57, 0x52
ADDR = dict(GATE=0x00, LOCK=0x01, UNLOCK=0x02, AVG=0x03, TAU=0x04, GEARS=0x05, REJ=0x06,
            TIMEOUT=0x07, SLIP=0x08, ID=0x0F)
ID_VALUE = 0x4E434F01
STATUS = {0: "ok", 1: "bad checksum", 2: "bad address / read-only", 3: "bad command"}

# reset values, as register words
DEFAULT_WORDS = {0x00: 0x0001_2000, 0x01: 0x0400_0000, 0x02: 0x1000_0000, 0x03: 0x0000_0705,
                 0x04: 0x0000_0000, 0x05: 0x0006_0602, 0x06: 16, 0x07: 5, 0x08: 0x0001_2000}

# setting name -> (address, low bit, width, encode(value)->int, decode(int)->value)
FIELDS = {
    "gate":          (0x00, 0, 16, lambda x: round(float(x) * 65536), lambda v: v / 65536),
    "gate_en":       (0x00, 16, 1, int, int),
    "lock":          (0x01, 0, 32, lambda x: round(float(x) * 2 ** 32), lambda v: v / 2 ** 32),
    "unlock":        (0x02, 0, 32, lambda x: round(float(x) * 2 ** 32), lambda v: v / 2 ** 32),
    "mse_shift":     (0x03, 0, 4, int, int),
    "mse_shift_trk": (0x03, 8, 4, int, int),
    "tau_L":         (0x04, 0, 5, int, int),
    "tau_en":        (0x04, 8, 1, int, int),
    "kp_acq":        (0x05, 0, 4, int, int),
    "kp_trk":        (0x05, 8, 4, int, int),
    "gear":          (0x05, 16, 4, int, int),
    "rej_max":       (0x06, 0, 8, int, int),
    "timeout":       (0x07, 0, 5, int, int),
    "slip":          (0x08, 0, 16, lambda x: round(float(x) * 65536), lambda v: v / 65536),
    "slip_unlock":   (0x08, 16, 1, int, int),
}


def command(cmd, addr, data=0):
    """8-byte command frame."""
    d = [(data >> s) & 0xFF for s in (24, 16, 8, 0)]
    chk = cmd ^ addr ^ d[0] ^ d[1] ^ d[2] ^ d[3]
    return bytes([SYNC, cmd, addr, *d, chk])


def write_cmd(addr, word):
    return command(CMD_W, addr, word)


def read_cmd(addr):
    return command(CMD_R, addr, 0)


def set_field(word, name, value):
    addr, lo, w, enc, _ = FIELDS[name]
    x = enc(value)
    if not 0 <= x < (1 << w):
        raise ValueError(f"{name}={value} does not fit in {w} bits")
    mask = ((1 << w) - 1) << lo
    return (word & ~mask) | (x << lo)


def get_field(word, name):
    _, lo, w, _, dec = FIELDS[name]
    return dec((word >> lo) & ((1 << w) - 1))


def parse_settings(items):
    """['gate=0.25', 'tau=off', ...] -> list of (name, value) in FIELDS terms."""
    out = []
    for it in items:
        k, _, v = it.partition("=")
        if k == "tau":
            if v == "off":
                out.append(("tau_en", 0))
            else:
                out += [("tau_L", int(v)), ("tau_en", 1)]
        elif k in FIELDS:
            out.append((k, v))
        else:
            raise SystemExit(f"unknown setting '{k}' (see --help)")
    return out


def describe(words):
    """Register words -> readable settings."""
    return {name: get_field(words[a], name) for name, (a, *_rest) in FIELDS.items() if a in words}


class Link:
    """Serial link: send a command, wait for its READBACK frame."""

    def __init__(self, port, baud=2_000_000, f_src=100e6):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.parser = FrameParser(f_src_hz=f_src)

    def transact(self, frame, timeout=1.0):
        self.ser.write(frame)
        t0 = time.time()
        while time.time() - t0 < timeout:
            for ftype, f in self.parser.feed(self.ser.read(256)):
                if ftype == T_TLM and f.get("kind") == "readback" and f["addr"] == frame[2]:
                    if f["status"]:
                        raise RuntimeError(f"board rejected command: {STATUS.get(f['status'])}")
                    return f["value"]
        raise TimeoutError("no readback from the board (wrong port? M1 bitstream loaded?)")

    def read(self, addr):
        return self.transact(read_cmd(addr))

    def write(self, addr, word):
        got = self.transact(write_cmd(addr, word))
        if got != word:
            raise RuntimeError(f"reg 0x{addr:02X}: wrote 0x{word:08X}, read back 0x{got:08X}")
        return got

    def read_all(self):
        return {a: self.read(a) for a in sorted(DEFAULT_WORDS)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--f-src", type=float, default=100e6)
    ap.add_argument("action", choices=["show", "set", "defaults"])
    ap.add_argument("settings", nargs="*")
    a = ap.parse_args(argv)
    link = Link(a.port, f_src=a.f_src)
    ident = link.read(ADDR["ID"])
    if ident != ID_VALUE:
        raise SystemExit(f"unexpected ID 0x{ident:08X} (expected 0x{ID_VALUE:08X})")
    if a.action == "defaults":
        for addr, w in DEFAULT_WORDS.items():
            link.write(addr, w)
    elif a.action == "set":
        words = link.read_all()
        for name, value in parse_settings(a.settings):
            addr = FIELDS[name][0]
            words[addr] = set_field(words[addr], name, value)
        for addr in sorted({FIELDS[n][0] for n, _ in parse_settings(a.settings)}):
            link.write(addr, words[addr])
    words = link.read_all()
    for k, v in describe(words).items():
        print(f"  {k:14s} {v}")
    for addr, w in words.items():
        print(f"  reg 0x{addr:02X} = 0x{w:08X}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
