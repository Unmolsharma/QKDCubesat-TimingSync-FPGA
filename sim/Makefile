# Linux convenience wrapper around the Python runner (no cocotb Makefile flow
# needed). Assumes the venv is active or python3 has the requirements.
#
#   make -C sim            # everything (default, nojitter, fast_corner, calibrated)
#   make -C sim default    # one configuration
#   make -C sim analyze    # calibrate + plots from the default code-density run

PYTHON ?= python3
ROOT   := $(abspath ..)
OUT    := $(ROOT)/sim/out/default

.PHONY: all default nojitter fast_corner calibrated calibrate analyze clean

all:
	$(PYTHON) run_tests.py

default nojitter fast_corner calibrated:
	$(PYTHON) run_tests.py --config $@

calibrate:
	$(PYTHON) $(ROOT)/scripts/calibrate.py $(OUT)/fine_codes.csv --period-ps 4000 \
	    --out $(OUT)/calib_lut.mem --summary $(OUT)/calib_lut.json

analyze: calibrate
	cd $(ROOT)/scripts && $(PYTHON) analyze.py --codes $(OUT)/fine_codes.csv \
	    --validation $(OUT)/validation.csv --lut $(OUT)/calib_lut.mem \
	    --model $(OUT)/model_thresholds.csv --outdir $(ROOT)/docs/img --label SIMULATED

clean:
	rm -rf sim_build out results.xml __pycache__
