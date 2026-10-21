#!/usr/bin/env python3
"""Plots and metrics for the TDC: histogram, bin widths, DNL, INL, and RMS
resolution before and after calibration.

INPUTS
  --codes       calibration set: one raw fine code per line (calibrate.py format)
  --validation  an INDEPENDENT set with known true times, CSV
                    fine,tau_true_ps[,anything]
                tau_true_ps = true time from the hit to its capture edge,
                i.e. what fine_ps should read. In simulation the testbench
                writes it (sim/out/<config>/validation.csv). On hardware it
                needs a reference: e.g. a delay generator stepping a known
                offset, or two TDC channels fed from one split pulse (then
                use the channel difference and divide the RMS by sqrt(2)).
  --lut         calibrated .mem (default: computed from --codes)
  --model       optional, SIMULATION ONLY: the delay-line model's true tap
                thresholds (model_thresholds.csv), used to check the
                code-density estimate against ground truth.
  --label       text stamped on every figure, e.g. "SIMULATED" or "HARDWARE".

The calibration LUT is never validated on the data it was built from.

OUTPUT
  <outdir>/*.png and <outdir>/results.json
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from calibrate import code_density, ideal_lut, load_codes, read_mem  # noqa: E402

# Reference palette (dataviz skill): slot 1 = calibrated / primary,
# slot 2 = raw / uncalibrated. Text and grid use neutral ink tokens.
C1, C2 = "#2a78d6", "#eb6834"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def style():
    plt.rcParams.update({
        "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
        "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
        "ytick.color": INK2, "text.color": INK, "axes.grid": True,
        "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
        "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
        "axes.titleweight": "bold", "legend.frameon": False, "lines.linewidth": 2,
    })


def finish(fig, ax, path, label, title):
    ax.set_title(f"{title}  [{label}]", loc="left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def load_validation(path, code_col=0, channel_col=None, channel=None):
    """fine,tau_true_ps lines (code in code_col, tau in the next column);
    with channel_col/channel only that channel's lines."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            c = line.split(",")
            if channel_col is not None and channel is not None and int(c[channel_col]) != channel:
                continue
            rows.append((int(c[code_col]), float(c[code_col + 1])))
    a = np.asarray(rows)
    return a[:, 0].astype(int), a[:, 1]


def model_true_widths(path, depth):
    """True popcount bin widths from the model thresholds: code k covers
    tau in [s_(k-1), s_k) of the SORTED thresholds (jitter aside)."""
    thr = np.sort(np.loadtxt(path, delimiter=",", comments="#")[:, 1])
    w = np.full(depth, np.nan)
    w[1:len(thr)] = np.diff(thr)
    return w


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--codes", required=True)
    ap.add_argument("--validation")
    ap.add_argument("--lut")
    ap.add_argument("--model")
    ap.add_argument("--period-ps", type=float, default=4000.0)
    ap.add_argument("--fine-w", type=int, default=9)
    ap.add_argument("--ideal-lsb-ps", type=float, default=22.0,
                    help="LSB of the uncalibrated (ideal linear) LUT")
    ap.add_argument("--label", default="SIMULATED")
    ap.add_argument("--outdir", default="build/analysis")
    ap.add_argument("--code-col", type=int, default=0,
                    help="column of the fine code (validation: tau is the next column)")
    ap.add_argument("--channel-col", type=int, help="multi-channel files: channel column")
    ap.add_argument("--channel", type=int, help="analyse this channel only")
    args = ap.parse_args(argv)

    style()
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    depth = 1 << args.fine_w
    r = code_density(load_codes(args.codes, args.code_col, args.channel_col, args.channel),
                     args.period_ps, depth)
    k = np.arange(depth)
    rng = slice(r["kmin"], r["kmax"] + 1)
    kr = k[rng]
    lut_cal = read_mem(args.lut)[:depth] if args.lut else np.rint(r["lut_ps"])
    lut_raw = ideal_lut(depth, args.ideal_lsb_ps)
    res = dict(label=args.label, hits=r["total"], code_min=r["kmin"], code_max=r["kmax"],
               n_codes=r["n_codes"], lsb_mean_ps=r["lsb_ps"],
               width_min_ps=r["width_min_ps"], width_max_ps=r["width_max_ps"],
               missing_codes=r["missing_codes"],
               dnl_raw_min=float(np.nanmin(r["dnl"][rng])),
               dnl_raw_max=float(np.nanmax(r["dnl"][rng])),
               inl_raw_min=float(np.nanmin(r["inl"][rng])),
               inl_raw_max=float(np.nanmax(r["inl"][rng])),
               quant_rms_ps=r["quant_rms_ps"])

    # 1. histogram
    fig, ax = plt.subplots(figsize=(8, 3.4))
    ax.bar(kr, r["counts"][rng], width=0.85, color=C1)
    ax.axhline(r["total"] / r["n_codes"], color=INK2, lw=1, ls="--")
    ax.text(r["kmax"], r["total"] / r["n_codes"], " uniform", color=INK2, va="center",
            fontsize=8)
    ax.set_xlabel("raw fine code")
    ax.set_ylabel("hits")
    finish(fig, ax, out / "histogram.png", args.label,
           f"Code-density histogram ({r['total']} random hits)")

    # 2. bin widths (+ model ground truth in simulation)
    fig, ax = plt.subplots(figsize=(8, 3.4))
    ax.bar(kr, r["widths_ps"][rng], width=0.85, color=C1, label="code-density estimate")
    ax.axhline(r["lsb_ps"], color=INK2, lw=1, ls="--")
    ax.text(r["kmax"], r["lsb_ps"], f" mean {r['lsb_ps']:.1f} ps", color=INK2,
            va="center", fontsize=8)
    if args.model:
        wt = model_true_widths(args.model, depth)
        ax.plot(kr, wt[rng], "o", ms=3, color=C2, label="model ground truth")
        d = (r["widths_ps"] - wt)[rng]
        res["width_est_vs_true_rms_ps"] = float(np.sqrt(np.nanmean(d ** 2)))
        ax.legend(loc="upper left")
    ax.set_xlabel("raw fine code")
    ax.set_ylabel("bin width (ps)")
    finish(fig, ax, out / "bin_widths.png", args.label, "Bin widths")

    # 3. DNL
    fig, ax = plt.subplots(figsize=(8, 3.0))
    ax.bar(kr, r["dnl"][rng], width=0.85, color=C1)
    ax.axhline(0, color=INK2, lw=1)
    ax.set_xlabel("raw fine code")
    ax.set_ylabel("DNL (LSB)")
    finish(fig, ax, out / "dnl.png", args.label,
           f"DNL, raw  (min {res['dnl_raw_min']:+.2f}, max {res['dnl_raw_max']:+.2f} LSB)")

    # 4/5. validation: errors before/after calibration, residual INL
    if args.validation:
        fine, tau = load_validation(args.validation, args.code_col, args.channel_col, args.channel)
        e_raw = lut_raw[fine] - tau
        e_cal = lut_cal[fine] - tau
        e_raw -= e_raw.mean()           # constant channel offset is not resolution
        e_cal -= e_cal.mean()
        rms_raw, rms_cal = float(np.sqrt(np.mean(e_raw ** 2))), float(np.sqrt(np.mean(e_cal ** 2)))
        # residual INL: mean calibrated error per code, in LSB (codes with >= 20 hits)
        cnt = np.bincount(fine, minlength=depth)[:depth]
        s_cal = np.bincount(fine, weights=e_cal, minlength=depth)[:depth]
        s_raw = np.bincount(fine, weights=e_raw, minlength=depth)[:depth]
        ok = cnt >= 20
        inl_res = np.where(ok, s_cal / np.maximum(cnt, 1), np.nan) / r["lsb_ps"]
        inl_val_raw = np.where(ok, s_raw / np.maximum(cnt, 1), np.nan) / r["lsb_ps"]
        res.update(validation_hits=int(len(fine)), rms_raw_ps=rms_raw, rms_cal_ps=rms_cal,
                   inl_residual_max_lsb=float(np.nanmax(np.abs(inl_res))),
                   inl_residual_max_ps=float(np.nanmax(np.abs(inl_res)) * r["lsb_ps"]),
                   inl_uncal_validation_max_lsb=float(np.nanmax(np.abs(inl_val_raw))))

        fig, ax = plt.subplots(figsize=(8, 3.4))
        ax.plot(kr, inl_val_raw[rng], color=C2, label="uncalibrated (ideal LUT)")
        ax.plot(kr, inl_res[rng], color=C1, label="calibrated (code-density LUT)")
        ax.axhline(0, color=INK2, lw=1)
        ax.set_xlabel("raw fine code")
        ax.set_ylabel("INL (LSB)")
        ax.legend(loc="best")
        finish(fig, ax, out / "inl.png", args.label,
               "INL measured on the independent validation set")

        fig, ax = plt.subplots(figsize=(8, 3.4))
        lim = max(np.percentile(np.abs(e_raw), 99.5), 3 * rms_cal)
        bins = np.linspace(-lim, lim, 121)
        ax.hist(e_raw, bins=bins, color=C2, alpha=0.55,
                label=f"uncalibrated  RMS {rms_raw:.1f} ps")
        ax.hist(e_cal, bins=bins, color=C1, alpha=0.75,
                label=f"calibrated  RMS {rms_cal:.1f} ps")
        ax.set_xlabel("measured - true time (ps), mean removed")
        ax.set_ylabel("hits")
        ax.legend(loc="upper left")
        finish(fig, ax, out / "resolution.png", args.label,
               f"Single-shot error, {len(fine)} validation hits")

    # raw INL from the code-density set (reported as-is)
    fig, ax = plt.subplots(figsize=(8, 3.0))
    ax.plot(kr, r["inl"][rng], color=C1)
    ax.axhline(0, color=INK2, lw=1)
    ax.set_xlabel("raw fine code")
    ax.set_ylabel("INL (LSB)")
    finish(fig, ax, out / "inl_raw.png", args.label,
           f"INL, raw bin centres  ({res['inl_raw_min']:+.2f} .. {res['inl_raw_max']:+.2f} LSB)")

    (out / "results.json").write_text(json.dumps(res, indent=2))
    for key, v in res.items():
        print(f"  {key:30s} {v:.3f}" if isinstance(v, float) else f"  {key:30s} {v}")


if __name__ == "__main__":
    main()
