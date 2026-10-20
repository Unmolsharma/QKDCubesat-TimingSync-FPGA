## =============================================================================
## m1_arty_a7.xdc  --  extra constraints for the M1 demo top (m1_arty_top)
## -----------------------------------------------------------------------------
## Read by scripts/build.tcl AFTER tdc_arty_a7.xdc when top=m1_arty_top.
## PIN CHOICES ARE UNVERIFIED (no board or Vivado on the development machine):
## check both against the Digilent Arty A7-100 master XDC and schematic.
## jb[0] MUST be a clock-capable (MRCC/SRCC) pin for a clean BUFG input; if it
## is not, move it, or uncomment the CLOCK_DEDICATED_ROUTE line below.
## =============================================================================

## Emulator clock from the waveform generator (~100 MHz, independent crystal)
set_property -dict {PACKAGE_PIN E15 IOSTANDARD LVCMOS33} [get_ports {jb[0]}]
create_clock -name clk_src -period 10.000 [get_ports {jb[0]}]
# set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets {jb_IBUF[0]}]

## The emulator domain has no phase relationship to the board clock: that is
## the point of the demo. Its only link to clk_sys is the pulse, which goes
## out of the FPGA and back in on ja[0] (already a false path, see the main XDC).
set_clock_groups -asynchronous -group [get_clocks clk_src] \
                               -group [get_clocks -include_generated_clocks clk100]

## Emulator pulse output, Pmod JA pin 2 (jumper to JA pin 1 = ja[0])
set_property -dict {PACKAGE_PIN B11 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports emu_out]
set_false_path -to [get_ports emu_out]
