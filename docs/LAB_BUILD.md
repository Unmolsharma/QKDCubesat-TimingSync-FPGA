# Lab build and hardware measurement

This covers the full hardware loop on the lab machine: build, program, capture,
calibrate, rebuild with the LUT, and measure. Vivado is only needed for steps 1-3.
Steps 4 onward work on any machine with Python and the board's USB connection.

**Nothing here has been run yet.** The build script and constraints were written
and syntax-checked on a machine without Vivado. Expect to fix small tool-version
details on the first run, and please feed those fixes back into the repo.

## 0. What you need

- Vivado 2020.x or newer (tested syntax only; any version with Artix-7 support), on PATH
- Arty A7-100T and a micro-USB cable
- Python 3.9+ with `pip install -r requirements.txt`
- Optional: a pulse generator for Pmod JA pin 1 (LVCMOS33, >= 10 ns wide, e.g. 10 kHz-1 MHz)

## 1. Clone and build

```sh
git clone <repo-url> qkd-tdc
cd qkd-tdc
vivado -mode batch -source scripts/build.tcl
```

Takes a few minutes. Outputs: `build/tdc_arty_top.bit` and `build/reports/`.

To build for a different board or part: `-tclargs part=<part>`, then edit the pins
in `constraints/tdc_arty_a7.xdc`.

## 2. Check the reports (do not skip)

| File | What to check |
|---|---|
| `build/reports/summary.txt` | WNS >= 0 on `clk_sys`, chain placement PASS, chain delay PASS (or WARN, see below). Copy WNS/Fmax to the README table. |
| `build/reports/chain_placement.rpt` | **RESULT: PASS**; 48 CARRY4 in one column (e.g. `SLICE_X12Y50`..`Y97`), a single clock region, no gaps, all 192 FFs in their CARRY4's slice. |
| `build/reports/chain_delay.rpt` | **PASS** = static-timing min delay (fast corner) from `CYINIT` of CARRY4 #0 to `CO[3]` of #47 is > 4.04 ns. **WARN** means the line may not cover one 4 ns period at a fast corner: the build still completes, but treat `sat_cnt` on the board as decisive and expect to need the two-chain fallback. Also lists the slow-corner max and a per-CARRY4 breakdown. Record min/max in the README. *(This report has only been run against stubbed Vivado commands so far; check that the per-CARRY4 arc values look like ~50-150 ps on the first real build.)* |
| `build/reports/utilization.rpt` | Slice LUTs / FFs / CARRY4 / BRAM for the README table. |
| `build/reports/timing_summary.rpt` | No unconstrained paths other than the intentional ones (hit, switches, LEDs, UART). The chain itself must NOT appear as a failing path; if it does, the false path didn't match. |
| `build/reports/methodology.rpt` | Expected: warnings about the ring oscillator's combinational loop and its asynchronous clock. Anything touching `u_dl` or `u_sync` needs investigating. |

**If placement fails** (for example the chosen column has a hard block in it): open
`build/post_synth.dcp` in the Vivado GUI, look at the device view for a clean
SLICE column that spans rows Y50-Y99 (or any other 50-row clock region), and change
only the `resize_pblock pb_tdc_chain` line in the XDC. Everything else follows from it.
A column close to the hit input's IO bank shortens the input routing, but that only
changes the constant offset.

Expected build warnings: `LUTLP-1` is waived by `ALLOW_COMBINATORIAL_LOOPS`; there
will be a timing-loop warning for the ring oscillator; and `SYNTHESIS` may be
reported as already defined.

## 3. Program the board

```sh
vivado -mode batch -source - <<'EOF'
open_hw_manager; connect_hw_server; open_hw_target
set_property PROGRAM.FILE build/tdc_arty_top.bit [current_hw_device]
program_hw_devices [current_hw_device]
EOF
```

(or use Hardware Manager in the GUI.) LED0 (LD4) should blink at ~1 Hz, which means
the MMCM is locked and the 250 MHz clock is running. Press **RESET** (the red
CPU_RESET button) to clear the sticky LEDs and counters.

| Switch | Off | On |
|---|---|---|
| SW0 | hit = Pmod JA pin 1 | hit = on-chip ring oscillator |
| SW1 | UART sends fine codes (2 B/hit) | UART sends full timestamps (7 B/hit) |

| LED | Meaning (sticky until reset) |
|---|---|
| LD4 | heartbeat |
| LD5 | a hit was dropped (the UART couldn't keep up). Harmless for code density. |
| LD6 | a hit pulse was shorter than the chain; check the pulse width. |
| LD7 | **saturation: the chain does not cover the clock period.** See README "Chain coverage". |

## 4. Capture code-density data

The FT2232H shows up as two ports. The UART is the **second** one (the higher COM
number on Windows, usually `/dev/ttyUSB1` on Linux).

Preferred: an external pulse generator on Pmod JA1 with SW0 off. It must be free-running
and not locked to the board's clock. Alternative: SW0 on (ring oscillator), which needs
no equipment but read the caveat below.

```sh
# SW1 off (fine-code mode)
python scripts/capture.py --port COM6 --count 500000 --out calib/hw_codes.csv --raw-out calib/hw_codes.bin
```

500 k hits gives ~2 700 hits per code, so width errors are about 2 % of a bin.
`capture.py` prints the board counters. **`sat` must be 0.** `short` should be 0
with a proper pulse source, and `drop` is fine to be non-zero.

**Ring-oscillator caveat:** on the same die and supply, the ring can partially lock to
the system clock. The histogram then shows a regular comb or ripple that isn't a chain
property. If you have a pulse generator, capture with both sources and compare the bin
widths. They should agree to within statistics.

## 5. Calibrate and analyse

```sh
python scripts/calibrate.py calib/hw_codes.csv --period-ps 4000 \
       --out calib/calib_lut.mem --summary calib/calib_summary.json
cd scripts && python analyze.py --codes ../calib/hw_codes.csv --label HARDWARE \
       --outdir ../calib/plots && cd ..
```

Things to look at:
- **Code range.** The max code should sit comfortably below 192. If codes reach
  ~190-192, or LD7 lit, coverage is marginal.
- **Mean LSB** vs the 22 ps assumption. If it is much smaller than 22 ps, coverage is
  at risk.
- **Bin widths.** Expect a repeating 4-tap pattern (CARRY4 structure), some very wide
  bins at CARRY4 or slice boundaries, and some near-zero bins.
- Note the widest and narrowest bin, raw DNL and raw INL for the README.

## 6. Rebuild with the calibration LUT

```sh
vivado -mode batch -source scripts/build.tcl -tclargs lut=calib/calib_lut.mem
```

Only the BRAM contents change. Placement is locked, so the chain is
unchanged. (Later, block 4 can write the LUT at run time through `cal_we/cal_addr/cal_wdata`
without rebuilding.)

## 7. Measure resolution on hardware (the headline number)

RMS resolution needs hits whose true time is known independently. The calibration
data can't be used for this. Options, from easiest:

1. **Period method (single channel, needs only a stable generator).** Drive
   Pmod JA1 with a low-jitter pulse generator at a fixed period P that is NOT a
   multiple of 4 ns (e.g. 1.000 123 MHz). Capture in timestamp mode (SW1 on)
   with `--ts-out`, and compute consecutive differences
   dt = (c[i+1]-c[i])*4000 - (fine_ps[i+1]-fine_ps[i]), with fine_ps taken from the
   LUT. std(dt)/sqrt(2) is the single-shot resolution. It includes the
   generator's and the MMCM's jitter, so it is a conservative number. Because the
   period isn't a multiple of 4 ns, the hits walk across all fine codes.
2. **Two-channel difference (recommended next step).** Instantiate a second
   `tdc_top` with its own chain in an adjacent column, and feed both from one split
   pulse. RMS(t1 - t2) / sqrt(2) is the per-channel resolution, and it is immune to
   generator jitter. This is also how the Jinan-1-style timing link will run.
3. **MMCM phase stepping (in-FPGA sweep).** Generate the test hit from a second MMCM
   output using dynamic phase shift (1/56 of the VCO period, ~17.9 ps steps at
   VCO = 1 GHz). This is the hardware version of `test_sweep_monotonic`. It is useful
   for linearity, but the hit shares the clock tree, so it underestimates jitter.

Record the result in the README's **Hardware** column. Keep the SIMULATED column
as it is.

## 8. Re-calibration over temperature

Carry-chain delay drifts with temperature and core voltage. The size of the drift
on this part isn't known yet, so measure it. For flight, plan periodic code-density runs from the ring
oscillator, or from dark counts / background light, which are random in time.
Block 4 can then reload the LUT through the write port. Capture at two board
temperatures (e.g. with a hot-air gun) to see how much the LUT moves.
