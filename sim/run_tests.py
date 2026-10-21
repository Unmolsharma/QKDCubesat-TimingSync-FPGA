#!/usr/bin/env python3
"""Build and run the cocotb test suites with Icarus Verilog (no make needed).

    python sim/run_tests.py                      # everything
    python sim/run_tests.py --config default     # one model configuration
    python sim/run_tests.py --suite top --test test_smoke
    python sim/run_tests.py --block tdc          # only one block's suites

Tests live in sim/<block>/ (tdc, nco, integration); RTL is every .v under rtl/.

Model configurations (all SIMULATED, see rtl/tdc/tdc_delay_line.v):
  default      TAP_NOM_PS=22, sigma 15 %, skew 6 ps, jitter 4 ps, SEED=1
  nojitter     same chain, JITTER_PS=0 (strict monotonicity check)
  fast_corner  TAP_NOM_PS=20: a faster (cold / fast-process) chain.
               test_coverage is EXPECTED TO FAIL here. It shows that 192
               taps don't cover 4 ns if the carry chain is ~10 % faster than
               nominal.
  calibrated   default chain, but the RTL LUT is loaded from the .mem file that
               scripts/calibrate.py produced from the default config's
               code-density run. It checks the $readmemh path end to end and
               re-runs the boundary test with the tighter calibrated tolerance.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from cocotb_tools.runner import get_runner

SIM_DIR = Path(__file__).resolve().parent
ROOT = SIM_DIR.parent
# all RTL, plus simulation-only wrappers in sim/<block>/hdl/
RTL = sorted((ROOT / "rtl").rglob("*.v")) + sorted(SIM_DIR.glob("*/hdl/*.v"))
SIM_LUT = SIM_DIR / "out" / "default" / "calib_lut.mem"

# suite name -> (block dir under sim/, python module, HDL toplevel, extra build
#                parameters[, extra environment])
SUITES = {
    "frontend": ("tdc", "test_frontend", "tdc_frontend", {}),
    "top":      ("tdc", "test_top", "tdc_top", {}),
    "uart":     ("tdc", "test_uart", "tdc_arty_top",
                 {"FIFO_AW": 3, "STATUS_PERIOD": 50000, "RO_DIV_BITS": 4}),
    # block 2: 100 MHz source (M1) and 500 MHz source (M2)
    "nco":      ("nco", "test_nco", "nco_tb_top", {}, {"NCO_FSRC": "100e6"}),
    "nco500":   ("nco", "test_nco", "nco_tb_top", {"F_SRC_HZ": 500000000, "T_SLOT_PS": 2000},
                 {"NCO_FSRC": "500e6"}),
}
SUITES["integ"] = ("integration", "test_tdc_nco", "tdc_nco_tb_top", {})
# phase 2: M1 rehearsal (board top + emulator on its own clock)
SUITES["m1"] = ("integration", "test_m1", "m1_tb_top", {})
SUITES["emu"] = ("integration", "test_emulator", "sat_emulator", {})
SUITES["ncocmd"] = ("nco", "test_nco_cmd", "nco_cmd_tb_top", {}, {"NCO_FSRC": "100e6"})
NCO500_TESTS = ["test_static_lock", "test_ppm_sweep", "test_jitter_sweep", "test_gap_reacquire"]

CONFIGS = {
    "default": dict(params={}, suites=["frontend", "top", "uart", "nco", "nco500", "ncocmd", "integ", "emu", "m1"],
                    tests={"nco500": NCO500_TESTS}, xfail=set()),
    "nojitter": dict(params={"JITTER_PS": 0.0}, suites=["frontend"],
                     tests={"frontend": ["test_sweep_monotonic"]}, xfail=set()),
    "fast_corner": dict(params={"TAP_NOM_PS": 20.0},
                        suites=["frontend", "top"],
                        tests={"frontend": ["test_coverage"], "top": ["test_top_coverage"]},
                        xfail={"test_coverage", "test_top_coverage"}),
    "calibrated": dict(params={"LUT_FILE": f'"{SIM_LUT.as_posix()}"'}, suites=["top"],
                       tests={"top": ["test_smoke", "test_boundary", "test_code_density"]},
                       env={"TDC_LUT_FILE": str(SIM_LUT), "TDC_N_DENSITY": "2000",
                            "TDC_N_VALID": "5000"},
                       xfail=set()),
}


def ensure_sim_lut():
    """The calibrated config needs a LUT built from the default config's data."""
    codes = SIM_LUT.parent / "fine_codes.csv"
    if not codes.exists():
        raise SystemExit("calibrated config: run the default config first "
                         "(it produces sim/out/default/fine_codes.csv)")
    import subprocess
    subprocess.run([sys.executable, str(ROOT / "scripts" / "calibrate.py"), str(codes),
                    "--out", str(SIM_LUT), "--summary", str(SIM_LUT.with_suffix(".json"))],
                   check=True)


def junit_results(xml_path: Path):
    out = []
    for tc in ET.parse(xml_path).getroot().iter("testcase"):
        failed = tc.find("failure") is not None or tc.find("error") is not None
        out.append((tc.get("name"), not failed))
    return out


def run(config: str, suite: str, only_test=None, extra_params=None):
    cfg = CONFIGS[config]
    block, module, top, suite_params = SUITES[suite][:4]
    suite_env = SUITES[suite][4] if len(SUITES[suite]) > 4 else {}
    params = dict(suite_params)
    params.update(cfg["params"])
    params.update(extra_params or {})
    build_dir = SIM_DIR / "sim_build" / f"{config}_{suite}"
    out = SIM_DIR / "out" / config
    out.mkdir(parents=True, exist_ok=True)

    if config == "calibrated":
        ensure_sim_lut()
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
        test_dir=SIM_DIR / block,
        plusargs=[f"+TDC_MODEL_DUMP={(out / 'model_thresholds.csv').as_posix()}"],
        extra_env={"TDC_CONFIG": config, "TDC_OUT": str(out),
                   "PYTHONPATH": os.pathsep.join([str(SIM_DIR / block), str(ROOT / "model"),
                                                 str(ROOT / "scripts")]),
                   **cfg.get("env", {}), **suite_env},
    )
    return [(config, suite, name, ok, name in cfg["xfail"]) for name, ok in junit_results(Path(xml))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", choices=CONFIGS, action="append")
    ap.add_argument("--suite", choices=SUITES, action="append")
    ap.add_argument("--block", choices=sorted({s[0] for s in SUITES.values()}), action="append")
    ap.add_argument("--test")
    args = ap.parse_args()

    results = []
    for config in args.config or CONFIGS:
        for suite in CONFIGS[config]["suites"]:
            if args.suite and suite not in args.suite:
                continue
            block, module = SUITES[suite][:2]
            if args.block and block not in args.block:
                continue
            if not (SIM_DIR / block / f"{module}.py").exists():
                print(f"[skip] {suite}: sim/{block}/{module}.py not written yet")
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
