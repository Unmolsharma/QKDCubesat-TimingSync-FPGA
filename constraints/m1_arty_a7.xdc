## =============================================================================
## m1_arty_a7.xdc  --  extra constraints for the M1 demo top (m1_arty_top)
## -----------------------------------------------------------------------------
## Read by scripts/build.tcl AFTER tdc_arty_a7.xdc when top=m1_arty_top.
## Pins checked against Digilent's Arty-A7-100-Master.xdc (digilent-xdc,
## master branch, fetched 2026-10-20):
##   E15 jb[0]       IO_L11P_T1_SRCC_15  clock-capable (SRCC, P side): OK for
##                   a single-ended clock into a BUFG
##   B11 ja[1]       IO_L4P_T0_15        Pmod JA pin 2 (Sch=ja[2])
##   A9  uart_txd_in IO_L14N_T2_SRCC_16  USB-UART, PC -> FPGA
## (Not yet built in Vivado.)
## =============================================================================

## Emulator clock from the waveform generator (~100 MHz, independent crystal)
set_property -dict {PACKAGE_PIN E15 IOSTANDARD LVCMOS33} [get_ports {jb[0]}]
create_clock -name clk_src -period 10.000 [get_ports {jb[0]}]

## The emulator domain has no phase relationship to the board clock: that is
## the point of the demo. Its only link to clk_sys is the pulse, which goes
## out of the FPGA and back in on ja[0] (already a false path, see the main XDC).
set_clock_groups -asynchronous -group [get_clocks clk_src] \
                               -group [get_clocks -include_generated_clocks clk100]

## Emulator pulse output, Pmod JA pin 2 (jumper to JA pin 1 = ja[0])
set_property -dict {PACKAGE_PIN B11 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 12} [get_ports emu_out]
set_false_path -to [get_ports emu_out]

## PC -> FPGA command UART (2 Mbaud). Asynchronous, synchronised in uart_rx.
set_property -dict {PACKAGE_PIN A9 IOSTANDARD LVCMOS33} [get_ports uart_txd_in]
set_false_path -from [get_ports uart_txd_in]
