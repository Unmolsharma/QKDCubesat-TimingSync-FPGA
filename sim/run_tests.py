#!/usr/bin/env python3
"""Build and run the cocotb test suites with Icarus Verilog (no make needed).

    python sim/run_tests.py                      # everything
    python sim/run_tests.py --config default     # one model configuration
    python sim/run_tests.py --suite top --test test_smoke

Model configurations (all SIMULATED, see rtl/tdc_delay_line.v):
  default      TAP_NOM_PS=22, sigma 15 %, skew 6 ps, jitter 4 ps, SEED=1
  nojitter     same chain, JITTER_PS=0 (strict monotonicity check)
  fast_corner  TAP_NOM_PS=20: a faster (cold / fast-process) chain.
               test_coverage is EXPECTED TO FAIL here. It shows that 192
               taps don't cover 4 ns if the carry chain is ~10 % faster than
               nominal.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from cocotb_tools.runner import get_runner

SIM_DIR = Path(__file__).resolve().parent
ROOT = SIM_DIR.parent
RTL = sorted((ROOT / "rtl").glob("*.v"))

# suite name -> (python module, HDL toplevel)
SUITES = {
    "frontend": ("test_frontend", "tdc_frontend"),
    "top":      ("test_top", "tdc_top"),
    "uart":     ("test_uart", "tdc_arty_top"),
}

CONFIGS = {
    "default": dict(params={}, suites=["frontend", "top", "uart"], xfail=set()),
    "nojitter": dict(params={"JITTER_PS": 0.0}, suites=["frontend"],
                     tests={"frontend": ["test_sweep_monotonic"]}, xfail=set()),
    "fast_corner": dict(params={"TAP_NOM_PS": 20.0},
                        suites=["frontend", "top"],
                        tests={"frontend": ["test_coverage"], "top": ["test_top_coverage"]},
                        xfail={"test_coverage", "test_top_coverage"}),
}


def junit_results(xml_path: Path):
    out = []
    for tc in ET.parse(xml_path).getroot().iter("testcase"):
        failed = tc.find("failure") is not None or tc.find("error") is not None
        out.append((tc.get("name"), not failed))
    return out


def run(config: str, suite: str, only_test=None, extra_params=None):
    cfg = CONFIGS[config]
    module, top = SUITES[suite]
    params = dict(cfg["params"])
    params.update(extra_params or {})
    build_dir = SIM_DIR / "sim_build" / f"{config}_{suite}"
    out = SIM_DIR / "out" / config
    out.mkdir(parents=True, exist_ok=True)

    runner = get_runner("icarus")
    runner.build(
        sources=RTL,
        hdl_toplevel=top,
        parameters=params,
        build_dir=build_dir,
        timescale=("1ns", "1fs"),
        always=True,
    )
    tests = [only_test] if only_test else cfg.get("tests", {}).get(suite)
    xml = runner.test(
        test_module=module,
        hdl_toplevel=top,
        testcase=tests,
        build_dir=build_dir,
        test_dir=SIM_DIR,
        plusargs=[f"+TDC_MODEL_DUMP={(out / 'model_thresholds.csv').as_posix()}"],
        extra_env={"TDC_CONFIG": config, "TDC_OUT": str(out),
                   "PYTHONPATH": os.pathsep.join([str(SIM_DIR), str(ROOT / "scripts")])},
    )
    return [(config, suite, name, ok, name in cfg["xfail"]) for name, ok in junit_results(Path(xml))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", choices=CONFIGS, action="append")
    ap.add_argument("--suite", choices=SUITES, action="append")
    ap.add_argument("--test")
    args = ap.parse_args()

    results = []
    for config in args.config or CONFIGS:
        for suite in CONFIGS[config]["suites"]:
            if args.suite and suite not in args.suite:
                continue
            if not (SIM_DIR / f"{SUITES[suite][0]}.py").exists():
                print(f"[skip] {suite}: {SUITES[suite][0]}.py not written yet")
                continue
            results.extend(run(config, suite, args.test))

    print("\n================ SUMMARY (SIMULATED) ================")
    bad = 0
    for config, suite, name, ok, xfail in results:
        if xfail:
            status = "XFAIL (expected: demonstrates the risk)" if not ok else "XPASS (unexpected!)"
            bad += ok
        else:
            status = "PASS" if ok else "FAIL"
            bad += not ok
        print(f"  {config:12s} {suite:9s} {name:28s} {status}")
    print("=====================================================")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
