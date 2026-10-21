#!/usr/bin/env python3
"""Code-density calibration for the carry-chain TDC.

THE METHOD
  Fire many hits whose arrival times are uniformly distributed relative to the
  system clock (a free-running source uncorrelated with the clock). Each fine
  code k then occurs with probability width_k / T_clk, so

      width_k  = count_k / total * T_clk             (ps)
      LSB      = T_clk / (number of codes in the used range)
      DNL_k    = width_k / LSB - 1                   (LSB)
      edge_k   = sum(width_j for j < k)              (ps, from the lowest code)
      centre_k = edge_k + width_k / 2                (ps)  -> LUT[k]
      INL_k    = (centre_k - ideal_centre_k) / LSB   (LSB, bin-centre INL)

  LUT[k] = centre_k is the best single-number estimate of the time a code
  represents. The residual error is then roughly width_k / sqrt(12) per hit.

INPUT FORMAT (same for simulation and hardware)
  A text/CSV file with ONE INTEGER RAW FINE CODE PER LINE. Blank lines and
  lines starting with '#' are ignored. If a line has several comma-separated
  columns, only the first is used, so validation.csv and board dumps with
  extra columns work unchanged. scripts/capture.py writes this format.

  MULTI-CHANNEL (rtl/tdc/tdc_multi_top.v): every chain has its own tap delays
  and needs its OWN LUT. Give the column holding the channel number and the
  column holding the code, e.g. for "ch,fine" lines:
      --channel-col 0 --code-col 1 --channel 2      one channel -> --out
      --channel-col 0 --code-col 1 --per-channel DIR
                                                    every channel -> DIR/ch<i>.mem
  (the names tdc_multi_top's LUT_DIR parameter expects). With a ring-oscillator
  or other on-chip source, check that each channel really sees hits that are
  uniform in phase: a source injection-locked to the system clock, or routed
  so that it reaches one channel only, gives that channel a biased LUT.

OUTPUT
  calib_lut.mem : 2^FINE_W lines, one hex word per line (PS_W bits), for
                  $readmemh in rtl/tdc/tdc_calib_lut.v. Codes below the used range
                  map to 0, codes above it to T_clk.
  --summary     : optional JSON with widths/DNL/INL statistics.

USAGE
  python scripts/calibrate.py sim/out/default/fine_codes.csv --period-ps 4000 \
      --out build/calib_lut.mem --summary build/calib_summary.json
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def load_columns(path):
    """All non-comment lines of a CSV/text file, split on commas."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                rows.append(line.split(","))
    return rows


def load_codes(path, code_col=0, channel_col=None, channel=None):
    """Raw fine codes from column code_col (default: the first). With
    channel_col and channel, only the lines of that channel."""
    codes = []
    for c in load_columns(path):
        if channel_col is not None and channel is not None and int(c[channel_col]) != channel:
            continue
        codes.append(int(c[code_col]))
    if not codes:
        raise ValueError(f"{path}: no codes found" + (f" for channel {channel}" if channel is not None else ""))
    return np.asarray(codes, dtype=np.int64)


def channels_in(path, channel_col):
    return sorted({int(c[channel_col]) for c in load_columns(path)})


def read_mem(path):
    """Read a $readmemh file (hex words, // comments allowed)."""
    words = []
    for line in Path(path).read_text().splitlines():
        line = line.split("//")[0].strip()
        words += [int(w, 16) for w in line.split()]
    return np.asarray(words, dtype=np.int64)


def write_mem(path, lut, ps_w, header=()):
    digits = (ps_w + 3) // 4
    with open(path, "w") as f:
        for h in header:
            f.write(f"// {h}\n")
        for v in lut:
            f.write(f"{int(v):0{digits}x}\n")


def ideal_lut(depth, lsb_ps, ps_w=16):
    """Same mapping as the RTL default: LUT[k] = k*LSB + LSB/2 (integer ps)."""
    k = np.arange(depth)
    return np.minimum(k * int(lsb_ps) + int(lsb_ps) // 2, (1 << ps_w) - 1)


def code_density(codes, period_ps, depth):
    """Run the code-density analysis. Returns a dict of numpy arrays/scalars."""
    counts = np.bincount(codes, minlength=depth)[:depth].astype(float)
    total = counts.sum()
    used = np.nonzero(counts)[0]
    kmin, kmax = int(used.min()), int(used.max())
    n_codes = kmax - kmin + 1
    lsb = period_ps / n_codes

    widths = counts / total * period_ps                 # ps, zero outside range
    edges = np.cumsum(widths) - widths                  # lower edge of each bin
    centres = edges + widths / 2
    rng = slice(kmin, kmax + 1)
    dnl = np.full(depth, np.nan)
    inl = np.full(depth, np.nan)
    dnl[rng] = widths[rng] / lsb - 1.0
    ideal_centres = (np.arange(depth) - kmin + 0.5) * lsb
    inl[rng] = (centres[rng] - ideal_centres[rng]) / lsb
    # 1-sigma statistical uncertainty of each width estimate (Poisson)
    sigma_w = np.sqrt(counts) / total * period_ps

    lut = np.empty(depth)
    lut[:kmin] = 0.0
    lut[rng] = centres[rng]
    lut[kmax + 1:] = period_ps
    w_in = widths[rng]
    return dict(
        counts=counts, total=int(total), kmin=kmin, kmax=kmax, n_codes=n_codes,
        lsb_ps=lsb, widths_ps=widths, sigma_w_ps=sigma_w, centres_ps=centres,
        dnl=dnl, inl=inl, lut_ps=lut,
        missing_codes=int(np.sum(counts[rng] == 0)),
        width_max_ps=float(w_in.max()), width_min_ps=float(w_in.min()),
        # expected RMS of quantisation alone with a perfect bin-centre LUT
        quant_rms_ps=float(math.sqrt(np.sum(w_in ** 3) / 12.0 / period_ps)),
    )


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("codes", help="CSV/text file, one raw fine code per line")
    ap.add_argument("--period-ps", type=float, default=4000.0, help="system clock period (ps)")
    ap.add_argument("--fine-w", type=int, default=9, help="fine code width (LUT depth 2^FINE_W)")
    ap.add_argument("--ps-w", type=int, default=16, help="LUT word width (bits)")
    ap.add_argument("--out", default="calib_lut.mem", help="output .mem file")
    ap.add_argument("--summary", help="optional JSON summary output")
    ap.add_argument("--code-col", type=int, default=0, help="column holding the fine code")
    ap.add_argument("--channel-col", type=int, help="column holding the channel number")
    ap.add_argument("--channel", type=int, help="calibrate only this channel")
    ap.add_argument("--per-channel", metavar="DIR",
                    help="one LUT per channel: DIR/ch<i>.mem (+ DIR/ch<i>.json)")
    args = ap.parse_args(argv)
    if args.per_channel:
        if args.channel_col is None:
            sys.exit("--per-channel needs --channel-col")
        out = Path(args.per_channel)
        out.mkdir(parents=True, exist_ok=True)
        for ch in channels_in(args.codes, args.channel_col):
            print(f"--- channel {ch}")
            main([args.codes, "--period-ps", str(args.period_ps), "--fine-w", str(args.fine_w),
                  "--ps-w", str(args.ps_w), "--code-col", str(args.code_col),
                  "--channel-col", str(args.channel_col), "--channel", str(ch),
                  "--out", str(out / f"ch{ch}.mem"), "--summary", str(out / f"ch{ch}.json")])
        return

    depth = 1 << args.fine_w
    codes = load_codes(args.codes, args.code_col, args.channel_col, args.channel)
    if codes.max() >= depth or codes.min() < 0:
        sys.exit(f"code out of range 0..{depth - 1}: min {codes.min()} max {codes.max()}")
    r = code_density(codes, args.period_ps, depth)

    lut = np.clip(np.rint(r["lut_ps"]), 0, (1 << args.ps_w) - 1).astype(int)
    write_mem(args.out, lut, args.ps_w, header=[
        f"TDC calibration LUT: fine code -> bin centre (ps), from {Path(args.codes).name}"
        + (f", channel {args.channel}" if args.channel is not None else ""),
        f"hits={r['total']} codes {r['kmin']}..{r['kmax']} T_clk={args.period_ps:g} ps "
        f"LSB={r['lsb_ps']:.3f} ps",
    ])

    rng = slice(r["kmin"], r["kmax"] + 1)
    summary = dict(
        source=str(args.codes), channel=args.channel, hits=r["total"], period_ps=args.period_ps,
        code_min=r["kmin"], code_max=r["kmax"], n_codes=r["n_codes"],
        lsb_mean_ps=r["lsb_ps"], width_max_ps=r["width_max_ps"],
        width_min_ps=r["width_min_ps"], missing_codes=r["missing_codes"],
        dnl_min=float(np.nanmin(r["dnl"][rng])), dnl_max=float(np.nanmax(r["dnl"][rng])),
        inl_min=float(np.nanmin(r["inl"][rng])), inl_max=float(np.nanmax(r["inl"][rng])),
        quant_rms_ps=r["quant_rms_ps"],
        mean_hits_per_code=r["total"] / r["n_codes"],
    )
    if args.summary:
        Path(args.summary).write_text(json.dumps(summary, indent=2))

    print(f"code density: {r['total']} hits, codes {r['kmin']}..{r['kmax']} "
          f"({r['n_codes']} codes, {r['missing_codes']} missing)")
    print(f"  LSB {r['lsb_ps']:.2f} ps   width min/max {r['width_min_ps']:.2f} / "
          f"{r['width_max_ps']:.2f} ps")
    print(f"  DNL {summary['dnl_min']:+.2f} .. {summary['dnl_max']:+.2f} LSB   "
          f"INL {summary['inl_min']:+.2f} .. {summary['inl_max']:+.2f} LSB")
    print(f"  wrote {args.out}")


if __name__ == "__main__":
    main()
