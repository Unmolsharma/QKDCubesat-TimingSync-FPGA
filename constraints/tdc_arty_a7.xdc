## =============================================================================
## tdc_arty_a7.xdc  --  Arty A7-100T constraints for the carry-chain TDC
## -----------------------------------------------------------------------------
## Read by scripts/build.tcl AFTER synth_design (the pblock/cell queries refer
## to synthesised cells). Per-tap LOC/BEL placement lives in
## constraints/tdc_chain_place.tcl, because it needs a loop, and XDC only
## allows a subset of Tcl.
##
## Pin assignments follow Digilent's Arty-A7-100-Master.xdc. Check them
## against your board revision's master XDC before first power-up.
## =============================================================================

## ---------------------------------------------------------------- clocks ----
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports CLK100MHZ]
create_clock -name clk100 -period 10.000 [get_ports CLK100MHZ]

## Vivado derives the MMCM output clock automatically (250 MHz with the
## default MULT=10 / DIV=4). This only gives it a readable name for the
## reports.
create_generated_clock -name clk_sys [get_pins u_mmcm/CLKOUT0]

## Ring oscillator (sw0 hit source). The divider FFs are clocked by the ring,
## so give it a clock so they are timed at all. The period is a conservative
## guess (the real frequency depends on placement, voltage and temperature).
## It has no phase relationship to clk_sys, by design.
create_clock -name ro_clk -period 2.500 [get_pins u_ro/u_ro_nand/O]
set_clock_groups -asynchronous -group [get_clocks -include_generated_clocks clk100] \
                               -group [get_clocks ro_clk]

## The ring IS a combinational loop, on purpose. Without this, DRC LUTLP-1
## blocks write_bitstream. Scoped to the ring's nets only.
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -of_objects [get_pins -of_objects [get_cells -hier -filter {NAME =~ *u_ro/*u_ro_*}] -filter {DIRECTION == OUT}]]

## ------------------------------------------------ asynchronous hit path ----
## WHY A FALSE PATH INTO THE CHAIN
##   The hit is asynchronous to clk_sys BY DESIGN. The TDC exists to measure
##   the phase of a signal that has no timing relationship to the clock, so
##   there is no setup/hold requirement to meet: the capture FFs are SUPPOSED
##   to see the edge arrive at any moment, including inside their setup window
##   (the sync stage handles the metastability). Without this constraint,
##   Vivado would time e.g. sw0_s -> hit mux -> CYINIT -> 192-tap chain ->
##   capture FF as a single-cycle path (~4-5 ns of carry delay) and report a
##   bogus failure. It could also try to "fix" it by rerouting, which would
##   destroy the uniform tap structure.
##   Everything that passes through the chain's entry pins is cut, and
##   nothing else. Only CARRY4 #0 has a live CYINIT; the others are tied to 0.
##   (Matching all of them avoids escaping "[0]", which a -filter glob would
##   treat as a character class.)
set_false_path -through [get_pins -of_objects [get_cells -hier -filter {REF_NAME == CARRY4 && NAME =~ *u_dl/*}] -filter {REF_PIN_NAME == CYINIT}]
set_false_path -from [get_ports {ja[0]}]

## ---------------------------------------------------- keep the chain intact ----
## (The RTL already carries these attributes; repeated here so they hold even
## if the RTL attributes are ever edited.)
set_property DONT_TOUCH TRUE [get_cells -hier -filter {REF_NAME == CARRY4 && NAME =~ *u_dl/*}]
set_property DONT_TOUCH TRUE [get_cells -hier -filter {REF_NAME == FDRE && NAME =~ *u_dl/*}]
set_property DONT_TOUCH TRUE [get_cells -hier -filter {NAME =~ *u_sync/q_r_reg*}]
set_property ASYNC_REG  TRUE [get_cells -hier -filter {NAME =~ *u_sync/q_r_reg*}]

## ---------------------------------------------------------------- pblock ----
## ONE slice column, ONE clock region. Artix-7 clock regions are 50 CLB rows
## tall (Y0-49, Y50-99, Y100-149, Y150-199 on the A100T), so Y50..Y99 lies in
## a single region. 48 CARRY4 use Y50..Y97. The carry chain can only run
## vertically, so staying inside one region keeps every capture FF on the same
## clock-region spine, with no skew step halfway up the chain.
## To move the chain, edit ONLY this range: tdc_chain_place.tcl derives every
## LOC from it, and build.tcl checks the result (build/reports/chain_placement.rpt).
create_pblock pb_tdc_chain
add_cells_to_pblock pb_tdc_chain [get_cells -hier -filter {(REF_NAME == CARRY4 || REF_NAME == FDRE) && NAME =~ *u_dl/*}]
resize_pblock pb_tdc_chain -add {SLICE_X12Y50:SLICE_X12Y99}
set_property IS_SOFT FALSE [get_pblocks pb_tdc_chain]
set_property EXCLUDE_PLACEMENT TRUE [get_pblocks pb_tdc_chain]

## ------------------------------------------------------------------- I/O ----
## Hit input: Pmod JA pin 1. LVCMOS33 input; a detector with a different
## logic level needs a level translator on the Pmod.
set_property -dict {PACKAGE_PIN G13 IOSTANDARD LVCMOS33} [get_ports {ja[0]}]

## CPU_RESET (active low)
set_property -dict {PACKAGE_PIN C2 IOSTANDARD LVCMOS33} [get_ports ck_rst]

## Slide switches
set_property -dict {PACKAGE_PIN A8  IOSTANDARD LVCMOS33} [get_ports {sw[0]}]
set_property -dict {PACKAGE_PIN C11 IOSTANDARD LVCMOS33} [get_ports {sw[1]}]
set_property -dict {PACKAGE_PIN C10 IOSTANDARD LVCMOS33} [get_ports {sw[2]}]
set_property -dict {PACKAGE_PIN A10 IOSTANDARD LVCMOS33} [get_ports {sw[3]}]

## Green LEDs LD4..LD7
set_property -dict {PACKAGE_PIN H5  IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN J5  IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN T9  IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports {led[3]}]

## USB-UART: FPGA TX -> FT2232H RX
set_property -dict {PACKAGE_PIN D10 IOSTANDARD LVCMOS33} [get_ports uart_rxd_out]

## Slow, human-speed or asynchronous I/O: no timing relationship to clk_sys.
set_false_path -from [get_ports {ck_rst sw[*]}]
set_false_path -to   [get_ports {led[*] uart_rxd_out}]

## ----------------------------------------------------------- bitstream ----
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
