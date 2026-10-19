#!/usr/bin/env python3
"""Capture TDC data from the Arty A7 USB-UART and write calibrate.py's CSV format.

LINK: 2 000 000 baud, 8N1 (see rtl/top/tdc_uart_readout.v).

FRAMES (self-synchronising: only header bytes have bit 7 set)
  header      1 T T P P P P P   (TT = type, 5 payload bits)
  continuation 0 P P P P P P P  (7 payload bits), payload MSB first
  type 0 FINE        2 bytes -> fine code
  type 1 TIMESTAMP   9 bytes -> {coarse (48 bit), fine}
  type 2 STATUS      8 bytes -> {drop_cnt, short_cnt, sat_cnt} (16 bit each)
  type 3 TELEMETRY  25 bytes -> block 2 loop telemetry, 173-bit field, MSB first:
         state 2 | kp 4 | L 5 | foff 32 (signed, 2^-48 slot/ps) | mse 16 (2^-16 slot^2)
         | sgn 16 (signed, 2^-16 slot) | unlock 16 | abort 16 | suspect 16 | noise 16
         | late 16 | hits 16 | 2 spare
         ppm = foff * 2^8 / FPS_NOM * 1e6, FPS_NOM = round(f_src * 1e-12 * 2^56)

OUTPUT
  --out codes.csv : one raw fine code per line (input for calibrate.py)
  --ts-out ts.csv : coarse,fine per line (only when the board is in timestamp mode)
  --tlm-out t.csv : one line per TELEMETRY frame (lock state, ppm, averages, counters)
  Both start with '#' comment lines recording the source, time and status.

EXAMPLES
  python scripts/capture.py --port COM5 --count 500000 --out hw_codes.csv
  python scripts/capture.py --port /dev/ttyUSB1 --seconds 60 --out hw_codes.csv \
         --raw-out capture.bin
  python scripts/capture.py --raw-in capture.bin --out hw_codes.csv   # offline re-parse

On the Arty the FT2232H exposes two ports. The UART is the SECOND one
(e.g. the higher COM number on Windows, /dev/ttyUSB1 on Linux).
"""

import argparse
import sys
import time
from datetime import datetime, timezone

FRAME_LEN = {0: 2, 1: 9, 2: 8, 3: 25}
T_FINE, T_TS, T_STATUS, T_TLM = 0, 1, 2, 3
STATE_NAMES = {0: "ACQ", 1: "RAMP", 2: "LOCK", 3: "?"}
TLM_FIELDS = (("state", 2), ("kp", 4), ("L", 5), ("foff", 32), ("mse", 16), ("sgn", 16),
              ("unlock", 16), ("abort", 16), ("suspect", 16), ("noise", 16), ("late", 16),
              ("hits", 16), ("spare", 2))
SIGNED = {"foff", "sgn"}


def fps_nom(f_src_hz):
    """Same integer formula as the RTL: round(f * 1e-12 * 2^56)."""
    return ((1 << 56) * int(round(f_src_hz)) + 500_000_000_000) // 1_000_000_000_000


def decode_tlm(v, f_src_hz=100e6):
    out, pos = {}, 173
    for name, w in TLM_FIELDS:
        pos -= w
        x = (v >> pos) & ((1 << w) - 1)
        if name in SIGNED and x >> (w - 1):
            x -= 1 << w
        out[name] = x
    out["state_name"] = STATE_NAMES[out["state"]]
    out["ppm"] = out["foff"] * 256 / fps_nom(f_src_hz) * 1e6
    out["mse_slot2"] = out["mse"] / 65536
    out["sgn_slot"] = out["sgn"] / 65536
    return out


class FrameParser:
    """Incremental parser. feed(bytes) -> list of (type, fields dict)."""

    def __init__(self, fine_w=9, coarse_w=48, f_src_hz=100e6):
        self.fine_w, self.coarse_w, self.f_src_hz = fine_w, coarse_w, f_src_hz
        self.buf = None            # [type, value, bytes_seen]
        self.resyncs = 0           # partial frames thrown away
        self.junk = 0              # continuation bytes outside a frame

    def feed(self, data):
        out = []
        for b in data:
            if b & 0x80:                                   # header
                if self.buf is not None:
                    self.resyncs += 1
                ftype = (b >> 5) & 0x3
                if ftype not in FRAME_LEN:
                    self.buf = None
                    self.junk += 1
                    continue
                self.buf = [ftype, b & 0x1F, 1]
            else:
                if self.buf is None:
                    self.junk += 1
                    continue
                self.buf[1] = (self.buf[1] << 7) | (b & 0x7F)
                self.buf[2] += 1
            if self.buf is not None and self.buf[2] == FRAME_LEN[self.buf[0]]:
                out.append(self._decode(self.buf[0], self.buf[1]))
                self.buf = None
        return out

    def _decode(self, ftype, v):
        fmask = (1 << self.fine_w) - 1
        if ftype == T_FINE:
            return ftype, dict(fine=v & fmask)
        if ftype == T_TS:
            return ftype, dict(fine=v & fmask,
                               coarse=(v >> self.fine_w) & ((1 << self.coarse_w) - 1))
        if ftype == T_TLM:
            return ftype, decode_tlm(v, self.f_src_hz)
        return ftype, dict(sat=v & 0xFFFF, short=(v >> 16) & 0xFFFF, drop=(v >> 32) & 0xFFFF)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port (COMx, /dev/ttyUSBx)")
    src.add_argument("--raw-in", help="parse a previously saved raw byte file")
    ap.add_argument("--baud", type=int, default=2_000_000)
    ap.add_argument("--out", required=True, help="fine-code CSV (calibrate.py input)")
    ap.add_argument("--ts-out", help="coarse,fine CSV for timestamp-mode frames")
    ap.add_argument("--raw-out", help="also save the raw byte stream")
    ap.add_argument("--tlm-out", help="CSV of TELEMETRY frames (block 2 loop)")
    ap.add_argument("--f-src", type=float, default=100e6, help="nominal source rate (ppm decode)")
    ap.add_argument("--count", type=int, default=0, help="stop after N hits (0 = no limit)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after S seconds")
    ap.add_argument("--fine-w", type=int, default=9)
    ap.add_argument("--coarse-w", type=int, default=48)
    args = ap.parse_args(argv)
    if args.port and not (args.count or args.seconds):
        ap.error("give --count and/or --seconds for a live capture")

    parser = FrameParser(args.fine_w, args.coarse_w, args.f_src)
    codes, stamps, status, tlm = [], [], [], []
    raw_out = open(args.raw_out, "wb") if args.raw_out else None

    def consume(data):
        if raw_out:
            raw_out.write(data)
        for ftype, f in parser.feed(data):
            if ftype == T_STATUS:
                if status and f["drop"] != status[-1]["drop"]:
                    print(f"  warning: {f['drop'] - status[-1]['drop']} hits dropped on board")
                status.append(f)
            elif ftype == T_TLM:
                tlm.append(f)
            else:
                codes.append(f["fine"])
                if ftype == T_TS:
                    stamps.append((f["coarse"], f["fine"]))

    t0 = time.time()
    if args.raw_in:
        with open(args.raw_in, "rb") as f:
            consume(f.read())
        source = args.raw_in
    else:
        import serial                                   # pyserial
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            ser.reset_input_buffer()
            source = f"{args.port}@{args.baud}"
            last = 0
            while True:
                consume(ser.read(max(1, ser.in_waiting)))
                el = time.time() - t0
                if el - last >= 1.0:
                    last = el
                    s = status[-1] if status else {}
                    if tlm:
                        t = tlm[-1]
                        s = dict(s, loop=t["state_name"], kp=t["kp"], ppm=round(t["ppm"], 3),
                                 unlocks=t["unlock"])
                    print(f"\r  {len(codes):9d} hits  {el:6.1f} s  status {s}", end="",
                          flush=True)
                if args.count and len(codes) >= args.count:
                    break
                if args.seconds and el >= args.seconds:
                    break
            print()
    if raw_out:
        raw_out.close()
    if args.count:
        codes = codes[:args.count]

    stamp = datetime.now(timezone.utc).isoformat(timespec="seconds")
    last = status[-1] if status else None
    head = [f"# TDC capture from {source} at {stamp}",
            f"# hits={len(codes)} status_frames={len(status)} last_status={last}",
            f"# parser resyncs={parser.resyncs} junk_bytes={parser.junk}"]
    with open(args.out, "w") as f:
        f.write("\n".join(head) + "\n# one raw fine code per line\n")
        f.writelines(f"{c}\n" for c in codes)
    if args.ts_out:
        with open(args.ts_out, "w") as f:
            f.write("\n".join(head) + "\n# coarse,fine\n")
            f.writelines(f"{c},{x}\n" for c, x in stamps)

    if args.tlm_out:
        with open(args.tlm_out, "w") as f:
            f.write("\n".join(head) + "\n# state,kp,L,ppm,mse_slot2,sgn_slot,unlock,abort,"
                    "suspect,noise,late,hits\n")
            for t in tlm:
                f.write(f"{t['state_name']},{t['kp']},{t['L']},{t['ppm']:.6f},{t['mse_slot2']:.6f},"
                        f"{t['sgn_slot']:.6f},{t['unlock']},{t['abort']},{t['suspect']},"
                        f"{t['noise']},{t['late']},{t['hits']}\n")
        print(f"{len(tlm)} telemetry frames -> {args.tlm_out}")
    print(f"{len(codes)} fine codes -> {args.out}")
    if last:
        print(f"board counters: drop={last['drop']} short={last['short']} sat={last['sat']}")
        if last["sat"]:
            print("  !! sat_cnt > 0: the chain does NOT cover the clock period (README).")
    if parser.resyncs or parser.junk:
        print(f"  note: {parser.resyncs} partial frames, {parser.junk} stray bytes (link errors?)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
