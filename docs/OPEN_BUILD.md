# Open-source build (openXC7): PRELIMINARY

> **Every result from this flow is preliminary.** It comes from Yosys, nextpnr-xilinx
> and Project X-Ray's timing data, not from Vivado, and **no bitstream from it has been
> run on a board**. Do not quote its numbers as TDC performance, do not copy them into
> the README tables, and do not use it to decide whether the design meets timing.
> [`scripts/build.tcl`](../scripts/build.tcl) remains the reference build and
> this flow does not change it.

It is useful for trying the build without a Vivado licence, catching synthesis problems
early, and getting a rough first look at which paths are slow.

Bitstreams are named `<top>_PRELIMINARY.bit`, live under `build/open/`, and every report
starts with a PRELIMINARY banner, so they can't be mistaken for Vivado output.

## Current state of the two tops

Seeds 1-3, ideal LUT, Yosys 0.68, nextpnr-xilinx 0.9.8, commit c2f87c4. **PRELIMINARY,
tool estimates.**

| | `tdc_arty_top` | `m1_arty_top` |
|---|---|---|
| Chain placement (240 cells, `SLICE_X12Y50..Y97`) | PASS | PASS |
| I/O pins and standards | PASS (12) | PASS (15) |
| MMCM in the bitstream source | x10 / 4 = 250 MHz | x10 / 4 = 250 MHz |
| Worst setup slack at 4 ns, seeds 1 / 2 / 3 | -1.02 / -0.57 / -0.42 ns | -3.21 / -2.62 / -2.57 ns |
| Estimated Fmax, seeds 1 / 2 / 3 | ~199 / 219 / 226 MHz | **~138 / 151 / 152 MHz** |
| Failing `clk_sys` endpoints (seed 1) | 80 of 3228 | 1708 of 12612 |
| 50 MHz serial clock `clk_ser` | passes, +13.6 ns or more | passes, +11.9 ns or more |
| Slice LUTs / FFs / CARRY4 | 1699 / 1668 / 174 | 6594 / 5922 / 541 |
| RAMB36 / DSP48 | 1 / 0 | 1 / 5 |
| Build time on the development laptop | ~4-5 min | ~11-21 min |

The MMCM check in the summary reads CLKOUT0 only; CLKOUT1 (the 50 MHz serial clock)
is not decoded from the FASM. Paths between `clk_sys` and `clk_ser` are listed in
`sta.rpt` under "Paths between clocks" (1.3-1.5 ns) and not checked: nextpnr has no
`set_max_delay` (see `constraints.rpt`).

**Neither top meets 250 MHz in this flow, so neither bitstream should be expected to work
reliably.** The chain's structure can be right while the logic behind it is too slow.

History of the M1 estimate (seed 1 unless noted): about 27 MHz unpipelined, about 119 MHz
pipelined, about 132 MHz with the fanout fixes, now 138-152 MHz over three seeds after
pipelining the TDC encoder and the NCO's slip and timeout compares and moving the UART,
settings and telemetry to a 50 MHz clock. The NCO admission logic added for late hits and
coincidences was at first M1's worst path (-5.2 ns); restructured over three commits it is
down to -3.2 ns on seed 1 and out of the worst list on seeds 2 and 3. `tdc_arty_top` went
from ~174 MHz (seed 1) to 199-226 MHz. What is left, all between about -2 and -3.2 ns in
M1 and with no single dominant path:

* NCO align: the registered admit/drop decision (`drop5`, 3 logic stages, placement
  bound) and `late_cnt`.
* TDC hit detector: the capture enables `cand_coarse`, `cand_fine`, `ev_coarse` (2
  logic stages, high fanout, -2.7 ns in M1, -1.0 ns in `tdc_arty_top`), and the encoder's
  4-way stage `u_enc.s4` (-0.3 to -0.6 ns in `tdc_arty_top`).
* NCO lock detector: the timeout mask decode `u_ld.mask` from `L` / `to_shift`, the mse
  threshold compares `m_gt_unlock` / `acq_ok`, the counters' enables.
* NCO loop filter `u_lf.rnd_bit`, `u_lf.v`, `corr_df`, and the gate compare `ge_h`: these
  are in the loop's feedback path, and pipelining them adds loop delay.
* `m_cal` (the TDC output register) in M1, from the NCO's registered `s_ready`.

Seed-to-seed spread: 0.6 ns on both tops in this round. Read each number as one sample.

## Setup on Windows

### Option A: WSL 2 with Docker in WSL (tested)

1. In PowerShell, if you don't have WSL yet: `wsl --install -d Ubuntu`, then reboot.
2. In Ubuntu, install Docker and let your user run it:
   ```sh
   sudo apt update && sudo apt install -y docker.io
   sudo usermod -aG docker $USER
   ```
   Close and reopen the terminal. (This flow was tested on Ubuntu 22.04 with the
   Docker snap; the `apt` package above is the usual route but was not itself tested.)
3. Build, from the repo. WSL sees `C:\` under `/mnt/c`:
   ```sh
   cd /mnt/c/Users/<you>/qkd-tdc
   bash scripts/open_build.sh                  # tdc_arty_top
   bash scripts/open_build.sh top=m1_arty_top
   ```

Or from PowerShell, in the repo folder: `wsl -d Ubuntu -- bash -lc "bash scripts/open_build.sh top=m1_arty_top"`.
The `-l` matters if Docker came from snap, because that is what puts it on `PATH`.

The first run builds the toolchain image (several minutes, about 6 GB of disk).

### Option B: Docker Desktop (not tested)

Install Docker Desktop with the WSL 2 backend and enable integration for your distro under
*Settings > Resources > WSL Integration*. Then follow step 3 above. The script is Bash and
mounts Linux paths, so run it from WSL rather than from PowerShell or `cmd`.

### Without Docker (not tested)

Set `OPEN_NATIVE=1`, plus `CHIPDB` (an xc7a100t `.bin`) and `PRJXRAY_DB` (`prjxray-db/artix7`),
and have the tools on `PATH`. The script refuses any nextpnr-xilinx other than 0.9.8, for
the reason given below. `OPEN_ALLOW_ANY_VERSION=1` overrides that; the post-route checks
still run.

## Usage

Same argument style as `build.tcl`:

```sh
bash scripts/open_build.sh [top=tdc_arty_top|m1_arty_top] [lut=calib/calib_lut.mem] [seed=N]
```

| Output under `build/open/<top>/` | |
|---|---|
| `<top>_PRELIMINARY.bit` | Written only if the structural checks pass |
| `reports/summary.txt` | **Start here** |
| `reports/constraints.rpt` | Every XDC statement and what became of it |
| `reports/chain_placement.rpt` | Chain checks, plus other logic in the reserved column |
| `reports/yosys.log`, `nextpnr.log`, `nextpnr.json` | Raw tool output |

The exit status is non-zero, with no bitstream, if the structural checks fail. Missing
timing does not stop the build, the same as `build.tcl`. The same seed reproduces the same
FASM and frames byte for byte; the `.bit` differs only in a timestamp in its header.

Tool-free tests of the helper script: `python scripts/test_open_flow.py`.

### Loading a bitstream

The image has no programmer. Vivado Hardware Manager can load the `.bit` as in
[LAB_BUILD.md](LAB_BUILD.md) step 3. `openFPGALoader -b arty_a7_100t <file>.bit` should work
too, but hasn't been tried, and from WSL it would need the USB device passed through with
`usbipd`. On a first load, watch for LED0's heartbeat before trusting anything else.

### Calibration LUTs

- `m1_arty_top`: `lut=` changes the design, confirmed by the netlist (LUT6 947 -> 1058 with a
  random table). The table contents were not compared entry by entry.
- **Yosys builds the table as LUTs, not BRAM**, so the statement in LAB_BUILD.md step 6 that
  "only the BRAM contents change" does not apply here. The chain stays pinned, but the rest
  of the design is placed again and timing moves.
- `tdc_arty_top`: `lut=` has no functional effect, because this top streams `m_raw` and leaves
  `m_cal` unconnected. A random table leaves the resource counts unchanged. It calibrates
  offline, see [LAB_BUILD.md](LAB_BUILD.md) step 6.
- Calibration made on a Vivado bitstream does not carry over, or the reverse: routing into
  the chain differs. Calibrate each bitstream separately.

## How the constraints are translated

The Vivado XDC files are the only source for pins and chain position. They are read at build
time, nothing is duplicated, and `scripts/open_flow.py` stops if it meets a statement it has
no rule for rather than skipping it.

nextpnr-xilinx 0.9.8's XDC reader handles only `set_property` on ports, `create_clock` and
`set_multicycle_path`. Fixed placement of a logic cell has to be a `BEL` attribute on the
netlist cell, so that is what the placement becomes:

| Vivado (`tdc_chain_place.tcl`) | This flow |
|---|---|
| CARRY4 #k: `LOC SLICE_X<x0+ch>Y<y0+k>`, `BEL CARRY4` | `BEL = "SLICE_X12Y<50+k>/CARRY4"` in the netlist JSON |
| Capture FF `g_ff[b]`: same slice, `BEL {A,B,C,D}FF` | `BEL = "SLICE_X12Y<50+k>/AFF"` etc. |
| Column and first row from `resize_pblock pb_tdc_chain` | Same line, read from the same XDC |
| Refuses a pblock whose width differs from the channel count | Same check |

### What could not be honored

| Constraint | Status | Consequence |
|---|---|---|
| `LOC`/`BEL` on all 48 CARRY4 + 192 FFs | **Honored**, checked after routing | None |
| Pins, `IOSTANDARD`, `SLEW`, `DRIVE` | **Honored**, checked after routing | None |
| `set_false_path -through CYINIT` | **Not supported.** Replaced by a x20 setup multicycle on the capture FFs | Placement is no longer steered by the path. In `tdc_arty_top` (`sw0_s` -> hit mux -> chain) it still hides real paths from nextpnr's Fmax line: **ignore that line** when the summary says so. The summary uses the slack histogram instead, which agrees with the Fmax line on `m1_arty_top`, where no such path exists |
| `EXCLUDE_PLACEMENT` on the pblock | **Not honored** | Other logic can use the rest of the column. 26 cells on M1, in rows Y98-Y99, above the chain. Listed in `chain_placement.rpt` |
| `ASYNC_REG` | **Not honored** | Second-stage FFs are not kept next to the capture FFs: up to 26 sites away on M1, 9 on `tdc_arty_top`. That eats into the metastability settling time. `-nosrl` at least keeps synchronisers out of SRLs |
| `DONT_TOUCH` | **Not honored**, covered by checks | Yosys may in principle optimise these cells, so the checks confirm they all survive and connect as designed |
| `set_max_delay -datapath_only` between `clk_sys` and `clk_ser` | **Not honored** | The crossings (async FIFO pointers, snapshot buses) are not checked; `sta.rpt` lists their longest delay under "Paths between clocks" |
| `CFGBVS`, `CONFIG_VOLTAGE`, `BITSTREAM.GENERAL.COMPRESS` | **Not honored** | No voltage DRC; the bitstream is uncompressed |
| `set_clock_groups`, `set_false_path` on ports, `ALLOW_COMBINATORIAL_LOOPS` | Not needed | nextpnr has no such checks: it doesn't time unconstrained ports or hold paths between clocks to a requirement, and has no loop DRC |

### What the flow does not provide at all

- **Chain coverage.** There is no equivalent of `chain_delay.rpt`. nextpnr's carry delays are
  coarse (about 0.2 ns per CARRY4 including the cascade, so roughly 10 ns for a chain the
  design assumes is about 4.2 ns), so they cannot answer whether the line covers one clock
  period. Only Vivado's static timing or `sat_cnt` on a board can.
- **Hold timing and clock skew.** Not analysed. A setup-only estimate can look better than
  the design really is.
- **Timing fidelity.** nextpnr's model is not Vivado's, and Yosys does not optimise like
  Vivado. Expect different slack and resource numbers, in either direction.

### Changes made for this flow

None of these touch `rtl/` or the XDC files. The netlist change is stated in `summary.txt` on
every build.

1. **`DI[0]` of CARRY4 #0 is left unconnected in the netlist.** With a fabric-driven `CYINIT`,
   nextpnr places the `S[0]=1` and `DI[0]=0` feed-through LUTs in one LUT pair, both wanting pin
   A1, and routing fails. With `S=1` the carry mux selects CI and never looks at DI, so the logic
   is unchanged. The CARRY4 primitive itself stays as written, and the code refuses to do this
   unless `S[0]` really is 1.
2. **`synth_xilinx -noclkbuf`.** Without it Yosys adds a BUFG on the ring oscillator's output,
   *inside the ring*, which would change its frequency. The tops instantiate the BUFGs they need.
3. **`-nosrl`**, see `ASYNC_REG` above.

All other Xilinx primitives (CARRY4, FDRE, LUTs, MMCME2_BASE, BUFG, ODDR, RAMB) are used as
instantiated.

## Why these tool versions

`docker/openxc7.Dockerfile` starts from the community image `regymm/openxc7` (pinned by digest,
not published by the openXC7 project) and rebuilds nextpnr-xilinx at tag **0.9.8**.

The version matters. The image's own build, `0.8.2-81-g1743d0f4`, cannot build this design: it
stops with `Unexpected IOBUF BEL` on a BEL-pinned FF, and from reading its packer, any `CO[i]`
with a non-carry user ends the chain, which would break the 48 CARRY4s into pieces. 0.9.8
keeps each CARRY4 whole, and the post-route checks are there because a build can succeed while
the chain is wrong.

The nextpnr-xilinx repository is now archived, and development has moved to `openXC7/nextpnr`
(the himbaechel xilinx target). That newer tool has **not** been tested with this flow. Moving
to it would mean re-checking the BEL handling and the `DI[0]` workaround.

## What the checks prove

After routing, `open_flow.py check` reads the routed netlist and the FASM (what the bitstream is
made from). Any failure means no bitstream:

| Check | Why it matters |
|---|---|
| All 240 cells at the requested BEL; column contiguous, one clock region | Same as `chain_placement.rpt` in Vivado |
| Every capture FF has `xFFMUX.CY` | The tap reaches its FF through the slice's dedicated mux, not general routing, so bins are set by the carry chain |
| No tap leaves through `xOUTMUX.CY` | Nothing loads the tap outside the slice |
| `PRECYINIT.AX` on the root, `PRECYINIT.CIN` above | The hit enters at the bottom, and the chain cascades rather than restarting |
| `CO3` -> `CIN` on every step | One unbroken chain |
| `S=1` on all 192 positions (LUT function and A1 tied to `VCC_WIRE`) | Otherwise the edge would not propagate |
| One global clock on all capture FFs | The bins assume a common sampling instant |
| Each capture FF feeds exactly one second-stage FF | Stands in for `DONT_TOUCH` |
| Ring: 5 LUTs, each driving the next directly (`tdc_arty_top`) | Stands in for `DONT_TOUCH` on the ring |
| Pins, `IOSTANDARD`, `SLEW`/`DRIVE`; MMCM counters | The FASM matches what was asked for |

These show the *structure* is right. They don't show the chain is linear, covers the clock
period, or that the design works: **that takes a board.**

Beyond the unit tests, these were confirmed once by hand:

- Flipping one `BFFMUX.CY` in a real M1 FASM makes the check refuse the build.
- Yosys's "multiple conflicting drivers" warnings for `tdc_encoder.i/j` (shared loop integers)
  are harmless: the mapped encoder matched the RTL on 19,991 random and thermometer vectors.
- `SLICE_X12Y50..Y99` is entirely in clock region `X0Y1` according to Project X-Ray's device
  database, which the comment in `tdc_arty_a7.xdc` had left unconfirmed. This is X-Ray's data,
  so still confirm it in Vivado's device view.
- nextpnr warns of 5 combinational loops on `tdc_arty_top`, which is the ring. It warns of 1
  on `m1_arty_top`, where a graph search of the synthesised netlist finds no loop, so this
  looks like a tool artefact. The cause was not pinned down.
