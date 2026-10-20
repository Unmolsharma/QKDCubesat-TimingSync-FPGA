#!/usr/bin/env python3
"""Operating limits of the block 2 loop, from the float model (SIMULATED).

Three studies, each with the DEFAULT register settings and with TUNED ones
(the gate and lock thresholds are runtime inputs of nco_top: cfg_gate_thr,
cfg_lock_thr, cfg_unlock_thr, cfg_mse_shift, ...):

  jitter   largest photon jitter (RMS) that still locks with 0 slips, at
           100 MHz and 500 MHz. Also the per-photon wrong-slot rate the jitter
           itself causes (a QBER contribution no loop can remove).
  dark     largest dark-count / signal ratio at which lock can still be
           declared (and then held without slips).
  tail     a realistic SPAD response (Gaussian core + exponential diffusion
           tail, model/hitgen.py) through the lock sweep and the slip hunt.

Success criterion everywhere (same as loop_float.lock_sweep): lock declared
within 20 000 hits, then 2 000 signal photons with no unlock and no slip, and
the averaged frequency estimate within 5 ppm (no false lock).
"""

import argparse
import json
import math
import os
import sys
from dataclasses import asdict, replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import loop_float as lf  # noqa: E402
from hitgen import StreamCfg  # noqa: E402

N_LOCK_MAX, N_VERIFY = 20000, 2000

# TUNED SETTINGS (all runtime registers)
#  jitter: wider gate (1/4 slot) so the loop keeps its own photons, and lock /
#          unlock thresholds raised towards the random-phase value 1/12.
#  dark:   lock threshold raised (the locked e^2 average rises with dark
#          counts), slower acquisition average (fewer chance crossings).
TUNED_JITTER = dict(gate=1 / 4, lock_thr=1 / 20, unlock_thr=0.07)
TUNED_DARK = dict(lock_thr=1 / 20, unlock_thr=0.07, mse_shift=6)


def trial(args):
    """One acquisition + verification run. Returns a summary dict."""
    f_src, p, ppm, jitter, dark_ratio, tail_frac, tail_tau, cfg_over, seed = args
    rate = p * f_src
    scfg = StreamCfg(f_src_hz=f_src, ppm=ppm, p_det=p, jitter_ps=jitter,
                     dark_hz=dark_ratio * rate, duration_s=(N_LOCK_MAX + N_VERIFY) / rate,
                     tail_frac=tail_frac, tail_tau_ps=tail_tau, seed=seed)
    lcfg = replace(lf.LoopCfg(f_nom_hz=f_src), **cfg_over)
    rec, s, lp, li = lf.run(scfg, lcfg, stop_after_verify=N_VERIFY)
    a = lf.assess(rec, li, ppm)
    ok = bool(a["locked"] and li < N_LOCK_MAX and a["slips"] == 0 and a["unlocks"] == 0
              and abs(a["f_err_ppm"]) < 5.0 and rec["locked"][-1])
    out = dict(ok=ok, locked=a["locked"], undetected=a.get("undetected_slips", 0),
               unlocks=a.get("unlocks", 0), lock_hits=a.get("lock_hits"))
    if li is not None:
        sig = (rec["slot_true"] >= 0) & (np.arange(len(rec["t"])) >= li) & rec["locked"]
        n = int(sig.sum())
        off = np.rint(rec["nco_err"][sig]).astype(np.int64)
        wrong = (rec["slot"][sig] - rec["slot_true"][sig]) != off
        out["slot_err_frac"] = float(wrong.mean()) if n else None
        out["gated_signal_frac"] = float(rec["noise"][sig].mean()) if n else None
        # wrong slot AND not flagged as noise: these would reach sifting as
        # errors (QBER); flagged ones are only a loss
        out["slot_err_unflagged_frac"] = float((wrong & ~rec["noise"][sig]).mean()) if n else None
        # where the loop centres the photons: mean residual of signal photons
        out["mean_e_slot"] = float(rec["e"][sig].mean()) if n else None
    return out


def sweep(points, trials, procs=None):
    """points: list of (key, args-without-seed). Returns {key: [results]}."""
    jobs, keys = [], []
    for ip, (key, args) in enumerate(points):
        for k in range(trials):
            # reproducible: seeds depend only on the trial index (the same k
            # gives the same offset and stream at every point of a sweep)
            rng = np.random.default_rng(424242 + k)
            ppm = rng.uniform(-100, 100)
            a = list(args)
            if a[2] is None:                       # default: uniform in +/-100 ppm
                a[2] = ppm
            elif isinstance(a[2], tuple):          # ("U", u): uniform in +/-u ppm
                a[2] = ppm * a[2][1] / 100.0
            jobs.append(tuple(a) + (1000 + 7919 * k,))
            keys.append(key)
    with Pool(procs or os.cpu_count()) as pool:
        res = pool.map(trial, jobs, chunksize=1)
    out = {}
    for k, r in zip(keys, res):
        out.setdefault(k, []).append(r)
    return out


def summarize(rs):
    ok = [r for r in rs if r["ok"]]
    se = [r["slot_err_frac"] for r in rs if r.get("slot_err_frac") is not None]
    return dict(trials=len(rs), success=len(ok) / len(rs),
                undetected_slips=sum(r["undetected"] for r in rs),
                unlocks=sum(r["unlocks"] for r in rs),
                lock_hits_med=float(np.median([r["lock_hits"] for r in ok])) if ok else None,
                slot_err_unflagged_frac=float(np.mean([r["slot_err_unflagged_frac"] for r in rs
                                                       if r.get("slot_err_unflagged_frac") is not None]))
                if se else None,
                slot_err_frac=float(np.mean(se)) if se else None,
                gated_signal_frac=float(np.mean([r["gated_signal_frac"] for r in rs
                                                 if r.get("gated_signal_frac") is not None]))
                if se else None)


def limit(rows, key="x"):
    """Largest x such that it and every smaller x had 100 % success, 0 undetected."""
    lim = None
    for r in sorted(rows, key=lambda r: r[key]):
        if r["success"] == 1.0 and r["undetected_slips"] == 0:
            lim = r[key]
        else:
            break
    return lim


# -----------------------------------------------------------------------------
def study_jitter(trials):
    cases = {100e6: (1e-3, [200, 400, 600, 800, 900, 1000, 1100, 1200, 1300, 1400, 1600, 1800, 2000]),
             500e6: (2e-4, [40, 80, 120, 150, 170, 190, 210, 230, 250, 270, 300, 350, 400])}
    pts = []
    for f, (p, jits) in cases.items():
        for name, over in (("default", {}), ("tuned", TUNED_JITTER)):
            for j in jits:
                pts.append(((f, name, j), (f, p, None, float(j), 0.1, 0.0, 300.0, over)))
    res = sweep(pts, trials)
    rows = []
    for (f, name, j), rs in res.items():
        rows.append(dict(f_src=f, cfg=name, x=j, sigma_slot=j * f * 1e-12, **summarize(rs)))
    out = {}
    for f in cases:
        for name in ("default", "tuned"):
            rr = [r for r in rows if r["f_src"] == f and r["cfg"] == name]
            out[f"{f / 1e6:.0f}MHz_{name}"] = dict(limit_ps=limit(rr), rows=sorted(rr, key=lambda r: r["x"]))
    return out


def study_dark(trials):
    ratios = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0, 3.0]
    pts = []
    for name, over in (("default", {}), ("tuned", TUNED_DARK)):
        for r in ratios:
            pts.append(((name, r), (100e6, 1e-3, None, 200.0, float(r), 0.0, 300.0, over)))
    res = sweep(pts, trials)
    out = {}
    for name in ("default", "tuned"):
        rr = [dict(cfg=name, x=r, **summarize(res[(name, r)])) for r in ratios]
        out[name] = dict(limit_ratio=limit(rr), rows=rr)
    return out


SPAD_CASES = {
    # name: (f_src, core RMS ps, tail fraction, tail tau ps[, initial ppm range])
    "100MHz_gauss150": (100e6, 150.0, 0.0, 300.0),
    "100MHz_spad150_15pct_300ps": (100e6, 150.0, 0.15, 300.0),
    "500MHz_gauss150": (500e6, 150.0, 0.0, 300.0),
    "500MHz_spad150_15pct_300ps": (500e6, 150.0, 0.15, 300.0),
    "500MHz_spad150_20pct_500ps": (500e6, 150.0, 0.20, 500.0),
    "500MHz_spad50_15pct_300ps": (500e6, 50.0, 0.15, 300.0),
    # same, +/-1 ppm initial uncertainty: separates pull-in from jitter effects
    "500MHz_gauss150_1ppm": (500e6, 150.0, 0.0, 300.0, 1.0),
    "500MHz_spad150_15pct_300ps_1ppm": (500e6, 150.0, 0.15, 300.0, 1.0),
}


def study_tail(trials):
    rates = [1e6, 1e5, 1e4, 3e3]                      # accepted photons / s
    pts = []
    for name, case in SPAD_CASES.items():
        f, core, tf, tau = case[:4]
        u = ("U", case[4]) if len(case) > 4 else None
        for rate in rates:
            pts.append(((name, rate), (f, rate / f, u, core, 0.1, tf, tau, {})))
    res = sweep(pts, trials)
    out = {}
    for name in SPAD_CASES:
        rows = []
        for rate in rates:
            rs = res[(name, rate)]
            sm = summarize(rs)
            me = [r["mean_e_slot"] for r in rs if r.get("mean_e_slot") is not None]
            sm["mean_e_ps"] = float(np.mean(me) * 1e12 / SPAD_CASES[name][0]) if me else None
            rows.append(dict(rate=rate, **sm))
        out[name] = rows
    return out


def study_tail_hunt(runs):
    """Slip hunt with the SPAD tail: many long runs, count slips while locked."""
    out = {}
    for name in ("100MHz_spad150_15pct_300ps", "500MHz_spad150_15pct_300ps"):
        f, core, tf, tau = SPAD_CASES[name][:4]
        jobs = []
        for k in range(runs):
            rng = np.random.default_rng(55_000 + k)
            p = [1e-2, 1e-3, 1e-4][k % 3] * 100e6 / f
            jobs.append((f, p, rng.uniform(-100, 100), core, 0.1, tf, tau, {}, 9000 + k))
        with Pool(os.cpu_count()) as pool:
            res = pool.map(trial, jobs, chunksize=1)
        out[name] = dict(runs=runs, undetected_slips=sum(r["undetected"] for r in res),
                         unlocks=sum(r["unlocks"] for r in res),
                         success=sum(r["ok"] for r in res) / runs,
                         slot_err_frac=float(np.mean([r["slot_err_frac"] for r in res
                                                      if r.get("slot_err_frac") is not None])),
                         slot_err_unflagged_frac=float(np.mean([r["slot_err_unflagged_frac"] for r in res
                                                                if r.get("slot_err_unflagged_frac") is not None])))
        print("tail hunt", name, out[name])
    return out


# -----------------------------------------------------------------------------
def plot_jitter(js, path):
    plt = lf._style()
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.4))
    for ax, f in zip(axes, (100e6, 500e6)):
        for name, col, ls in (("default", lf.C1, "-"), ("tuned", lf.C2, "--")):
            rr = js[f"{f / 1e6:.0f}MHz_{name}"]["rows"]
            x = [r["x"] for r in rr]
            ax.plot(x, [100 * r["success"] for r in rr], "o" + ls, color=col,
                    label=f"{name}: lock + 0 slips (%)")
            lim = js[f"{f / 1e6:.0f}MHz_{name}"]["limit_ps"]
            if lim:
                ax.axvline(lim, color=col, lw=1, ls=":")
                ax.text(lim, 50 + (8 if name == "tuned" else -8), f" {name} limit\n {lim:g} ps",
                        color=col, fontsize=8)
        rr = js[f"{f / 1e6:.0f}MHz_default"]["rows"]
        x = np.array([r["x"] for r in rr], float)
        sig = x * f * 1e-12
        pe = np.array([math.erfc(0.5 / (s * math.sqrt(2))) for s in sig])
        ax2 = ax.twinx()
        ax2.semilogy(x, np.maximum(pe, 1e-9), color=lf.INK2, lw=1.2, ls="-.",
                     label="photons in wrong slot (jitter alone)")
        ax2.set_ylim(1e-8, 1)
        ax2.set_ylabel("wrong-slot fraction", color=lf.INK2)
        ax2.grid(False)
        if f == 500e6:
            ax.axvline(150, color=lf.C3, lw=1.5)
            ax.text(153, 84, "typical Si SPAD\n~150 ps", color=lf.C3, fontsize=8)
        ax.set_xlabel("photon jitter, RMS (ps)")
        ax.set_ylabel("trials OK (%)")
        ax.set_ylim(-5, 105)
        ax.set_title(f"{f / 1e6:.0f} MHz source  [SIMULATED]", loc="left")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=7, loc="lower left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_dark(ds, path):
    plt = lf._style()
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    for name, col, ls in (("default", lf.C1, "-"), ("tuned", lf.C2, "--")):
        rr = ds[name]["rows"]
        ax.plot([r["x"] for r in rr], [100 * r["success"] for r in rr], "o" + ls, color=col,
                label=f"{name} thresholds")
        if ds[name]["limit_ratio"]:
            ax.axvline(ds[name]["limit_ratio"], color=col, lw=1, ls=":")
    ax.set_xlabel("dark-count rate / signal rate")
    ax.set_ylabel("lock within 20 000 hits, then 0 slips (%)")
    ax.set_ylim(-5, 105)
    ax.set_title("Dark-count limit, 100 MHz, p = 1e-3, 200 ps  [SIMULATED]", loc="left")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default=str(Path(__file__).resolve().parents[1] / "docs" / "img"))
    ap.add_argument("--trials", type=int, default=16)
    ap.add_argument("--only", choices=["jitter", "dark", "tail"], action="append")
    args = ap.parse_args(argv)
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    jp = out / "nco_limits.json"
    res = json.loads(jp.read_text()) if jp.exists() else {}
    res["label"] = "SIMULATED (model/loop_limits.py)"
    res["tuned_jitter"], res["tuned_dark"] = TUNED_JITTER, TUNED_DARK
    only = set(args.only or ["jitter", "dark", "tail"])
    if "jitter" in only:
        js = study_jitter(args.trials)
        for k, v in js.items():
            print(f"jitter {k}: limit {v['limit_ps']} ps")
            for r in v["rows"]:
                print(f"   {r['x']:5g} ps ({r['sigma_slot']:.3f} slot) ok {100 * r['success']:5.1f} % "
                      f"undet {r['undetected_slips']} unl {r['unlocks']} slot_err {r['slot_err_frac']} "
                      f"gated {r['gated_signal_frac']} lock_hits {r['lock_hits_med']}")
        res["jitter"] = js
        plot_jitter(js, out / "nco_jitter_limit.png")
    if "dark" in only:
        ds = study_dark(args.trials)
        for k, v in ds.items():
            print(f"dark {k}: limit ratio {v['limit_ratio']}")
            for r in v["rows"]:
                print(f"   r={r['x']:5g} ok {100 * r['success']:5.1f} % undet {r['undetected_slips']} "
                      f"unl {r['unlocks']} lock_hits {r['lock_hits_med']}")
        res["dark"] = ds
        plot_dark(ds, out / "nco_dark_limit.png")
    if "tail" in only:
        ts = study_tail(args.trials)
        for k, rows in ts.items():
            for r in rows:
                print(f"tail {k:30s} rate {r['rate']:8g} ok {100 * r['success']:5.1f} % undet "
                      f"{r['undetected_slips']} unl {r['unlocks']} slot_err {r['slot_err_frac']} "
                      f"unflagged {r['slot_err_unflagged_frac']} gated {r['gated_signal_frac']} mean_e {r['mean_e_ps']} ps lock_hits {r['lock_hits_med']}")
        res["tail"] = ts
        res["tail_hunt"] = study_tail_hunt(4 * args.trials * 4)
    jp.write_text(json.dumps(res, indent=1, default=float))
    print("wrote", jp)


if __name__ == "__main__":
    main()
