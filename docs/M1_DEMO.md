# M1 demo: lock to an independent 100 MHz source (Arty A7-100T)

**Goal (M1):** the FPGA timing core (block 1 TDC + block 2 clock recovery)
locks to a 100 MHz pulse source that runs on a *different* crystal from the
FPGA. It then assigns every detected pulse the correct slot number, with
zero cycle slips. The procedure below was rehearsed end to end in
**simulation** (`sim/integration/test_m1.py`). Every number on this page
marked SIMULATED comes from that rehearsal, not from hardware.

## What is on the board

```
 waveform generator --(SMA->Pmod/jumper)--> JB pin 1 (jb[0]) : emulator clock, ~100 MHz
                                                 |
                                         sat_emulator (own clock domain)
                                           LFSR picks slots with probability p
                                           15 ns pulse per chosen slot
                                                 |
                                         JA pin 2 (emu_out) ---- jumper wire ----+
                                                                                  |
 CLK100MHZ -> MMCM -> 250 MHz system clock                     JA pin 1 (ja[0]) <-+
                         |                                        |
                         +--------------------------------- tdc_top (carry-chain TDC)
                                                                  |
                                                            nco_top (clock recovery)
                                                                  |
                                     USB-UART 2 Mbaud <- STATUS + TELEMETRY frames
```

Top level: `rtl/top/m1_arty_top.v`. The emulator and the TDC/NCO share
nothing but the jumper wire, the same way the real system shares nothing but
the optical link.

### Pins (checked against Digilent's `Arty-A7-100-Master.xdc`, digilent-xdc master, 2026-10-20)

| Signal | Port | Arty connector | Package pin | Bank function (master XDC) |
|---|---|---|---|---|
| TDC hit input | `ja[0]` | Pmod JA pin 1 | G13 | IO_0_15 |
| Emulator pulse out | `emu_out` | Pmod JA pin 2 | B11 | IO_L4P_T0_15 |
| Emulator clock in | `jb[0]` | Pmod JB pin 1 | E15 | **IO_L11P_T1_SRCC_15: clock-capable** (SRCC, P side of the pair, as a single-ended clock needs), so it can drive the BUFG directly. No change needed. |
| Board clock | `CLK100MHZ` | oscillator | E3 | IO_L12P_T1_MRCC_35 |
| Reset | `ck_rst` | CPU_RESET | C2 | IO_L16P_T2_35 |
| Switches | `sw[3:0]` | SW0..3 | A8 C11 C10 A10 | bank 16 |
| LEDs | `led[3:0]` | LD4..7 | H5 J5 T9 T10 | |
| UART FPGA → PC | `uart_rxd_out` | USB | D10 | IO_L19N_T3_VREF_16 |
| UART PC → FPGA (commands) | `uart_txd_in` | USB | **A9** | IO_L14N_T2_SRCC_16 |

(If another Pmod pin is ever needed for the generator: JB pin 3 / D15 is
IO_L12P_T1_MRCC_15, also clock-capable.)

**Jumper:** one short wire from JA pin 2 to JA pin 1. Keep it short (a few
cm) and use the Pmod GND pins as the return path.

**Waveform generator:** a 100 MHz square wave, 0 to 3.3 V (LVCMOS33 levels,
**never above 3.3 V**). Set it e.g. +20 ppm off nominal (100.002 MHz) so the
loop has a known offset to find. Any offset within ±100 ppm is valid.

### Switches and LEDs

| | Meaning |
|---|---|
| `sw[1:0]` | detection probability per slot: 00 = 1e-1, 01 = 1e-2, 10 = 1e-3, 11 = 1e-4 |
| `sw[2]` | 1 = drop all pulses (signal gap). The loop must unlock, then re-lock when switched back |
| `sw[3]` | 1 = also stream raw TDC timestamps (lossy at high rates, debugging only) |
| LD4 `led[0]` | heartbeat (~1 Hz): MMCM locked, clock running |
| LD5 `led[1]` | **NCO LOCKED** |
| LD6 `led[2]` | sticky: an unlock has happened since reset |
| LD7 `led[3]` | sticky: TDC saturation (chain does not cover the clock period, see README) |

## Procedure

1. **Build:** `vivado -mode batch -source scripts/build.tcl -tclargs top=m1_arty_top`.
   This reads `constraints/tdc_arty_a7.xdc` plus `constraints/m1_arty_a7.xdc`
   (emulator clock and pulse pins: **check them first**, see the table).
   **Not yet run in Vivado.** Known item: the main XDC also constrains block
   1's ring oscillator (`u_ro`), which the M1 top does not contain. If Vivado
   rejects the `create_clock`/`ALLOW_COMBINATORIAL_LOOPS` lines on an empty
   object list, comment out those lines for the M1 build. Check
   `build/reports/summary.txt`: timing met, chain placement PASS, chain delay
   PASS. Then `python scripts/nco_config.py --port COMx show` must pass its ID
   check and print the default settings (proves the command link both ways).
2. **Wire up:** JA1–JA2 jumper; generator → JB1 at 100.002 MHz (+20 ppm),
   3.3 V CMOS, output **off**.
3. **Program**, press CPU_RESET. LD4 blinks. LD5 is off (no signal yet).
4. **Start the capture:**
   `python scripts/capture.py --port COMx --seconds 120 --out m1_codes.csv --tlm-out m1_tlm.csv`
   The console shows the loop state (`ACQ`/`RAMP`/`LOCK`), the gear, the
   frequency offset in ppm and the unlock count about 4 times per second.
5. **sw = 0001 (p = 1e-2), generator output on.** Within a fraction of a
   second (SIMULATED rehearsal: 0.4 ms at p = 1e-2) the state goes ACQ → RAMP → LOCK and
   LD5 lights. The ppm reading settles near +20.
6. **Hold for 60 s.** LD6 must stay off, `unlock` must stay 0, and `ppm` must
   stay within tolerance of the generator setting.
7. **Gap test:** sw[2] on for ~5 s, then off. LD5 goes off, `unlock`
   increments by 1, LD6 lights. After the gap the loop re-locks.
8. **Rate steps:** repeat steps 5–6 with sw[1:0] = 10 (1e-3) and 11 (1e-4).
   Lock takes longer at lower rates.
9. **Offset steps (optional):** generator at −50 ppm and +100 ppm.

## Changing the NCO settings at run time (no rebuild)

The PC can read and set every NCO runtime setting over the same USB-UART
(commands in on pin A9, replies in the normal telemetry stream):

```
python scripts/nco_config.py --port COMx show            # current settings + raw registers
python scripts/nco_config.py --port COMx set gate=0.25 lock=0.05 unlock=0.07
python scripts/nco_config.py --port COMx set mse_shift=6 tau=off kp_trk=4
python scripts/nco_config.py --port COMx defaults        # back to reset values
```

| Setting | Meaning | Default | When to change |
|---|---|---|---|
| `gate` | noise-gate half-width (slot) | 0.125 | 0.25 for jitter above ~0.06 slot (e.g. 150 ps at 500 MHz) |
| `gate_en` | gate on/off | 1 | debugging |
| `lock` / `unlock` | e² thresholds (slot²) | 1/64 / 1/16 | 0.05 / 0.07 for high jitter or dark counts up to 1.5× signal |
| `mse_shift`, `mse_shift_trk` | e² averaging 2^-n, acquiring / locked | 5 / 7 | 6 with heavy dark counts |
| `tau` | hit-spacing override (`off` or L = log2 clocks) | off | known, fixed rate |
| `kp_acq`, `kp_trk`, `gear` | gears 2^-n and ramp step 2^gear | 2, 6, 6 | `kp_trk=4` for Doppler at low rates (M2) |
| `rej_max`, `timeout`, `slip`, `slip_unlock` | unlock rules | 16, 5, 0.125, 1 | rarely |

Protocol: an 8-byte frame `A5 | 'W'/'R' | addr | data[31:0] | XOR checksum`,
answered by a READBACK frame (register value after the command, status, sequence
number). Register map: `rtl/top/nco_cfg_regs.v`. Values take effect from the next
hit; CPU_RESET restores the defaults. **Changing thresholds while locked is safe;
for large changes, write them while unlocked (e.g. with sw[2] on) to start from a
clean acquisition.**
Verified in simulation (`sim/nco/test_nco_cmd.py`): every setting written over
the UART line reads back and reaches `nco_top`. The NCO's results then match
the fixed-point model with that setting, bit for bit, and differ from the
default-setting results. A bad checksum, a write to the read-only ID register
and an unknown command are rejected.

## What the UART shows

Every 0.27 s: one STATUS frame (TDC `drop`/`short`/`sat`, all must be 0) and
one TELEMETRY frame:

| Field | Meaning |
|---|---|
| `state` | ACQ (searching), RAMP (phase found, gear shifting), LOCK |
| `kp` | current gear, Kp = 2^-kp (2 = acquisition ... 6 = tracking) |
| `L` | log2 of the measured mean hit spacing in clocks (gain scaling) |
| `ppm` | NCO frequency minus nominal 100 MHz, in ppm (= source offset relative to the FPGA crystal) |
| `mse_slot2` | e² average, slot²: ~0.083 when unlocked, small when locked |
| `sgn_slot` | signed phase-error average, slot: near 0 when locked |
| `unlock`, `abort`, `suspect` | lock losses, aborted gear ramps, suspected slips |
| `noise`, `late`, `hits` | hits gated as noise, late hits (must be 0), hits processed (16 bit, wraps) |

**Note on `ppm`:** the measurement is of the generator *relative to the Arty's
own 100 MHz oscillator* (itself typically within ±50 ppm). To compare with
the generator's setting, measure the Arty oscillator with a counter, or
accept agreement to within the oscillator tolerance.

## Pass criteria (proposed, to agree with the team lead)

Rehearsal column: `sim/integration/test_m1.py`, SIMULATED. Emulator clock at
+20 ppm, p = 1e-2 (2 666 pulses emitted, 26 suppressed adjacent slots = 1.0 %,
2 647 timestamped; 19 lost to TDC dead time). Timeline: lock and hold
for 1.2 ms, 0.5 ms gap, then 1.5 ms more.

| # | Criterion | Simulated rehearsal |
|---|---|---|
| 1 | Locks (LD5 on, `state` = LOCK) within 1 s at p = 1e-2 and within 5 s at p = 1e-4 | p = 1e-2: **395 µs** (375 hits). p = 1e-4 not rehearsed in RTL (too long to simulate); the float model gives a median of 38 ms from ±1 ppm and 70 ms from ±100 ppm |
| 2 | `ppm` converges to (generator offset − board oscillator offset) and stays within ±0.5 ppm (averaged over 10 telemetry frames) | **+20.018 ppm** mean, 0.046 ppm RMS, for a true +20.000; telemetry LOCK frames 19.94 … 20.01 |
| 3 | No unlock (`unlock` = 0) and no suspected slip in 60 s locked | 0 unlocks and 0 suspected slips while the signal was present (1.2 ms + 1.5 ms of simulated lock, not 60 s) |
| 4 | STATUS `drop`/`short`/`sat` all 0 | 0 / 0 / 0 |
| 5 | Gap (sw[2]) → `unlock` increments, then LOCK again within the time of criterion 1 | unlock = 1 after the 0.5 ms gap; LOCK again **354 µs** after the pulses returned; last telemetry frame LOCK |
| 6 | (Simulation only, not observable on the board) every detected pulse gets its true slot + one constant offset: 0 slips | **0 slips** over 1 901 locked photons checked against the emulator's true slot; NCO results bit-exact vs `loop_fixed.py` |

Criterion 6 is checked only in simulation, because the emulator's true slot
index never leaves the FPGA. On hardware, criteria 3 and 5 are the
observable proxies: a slip would show up as a suspected-slip unlock.
