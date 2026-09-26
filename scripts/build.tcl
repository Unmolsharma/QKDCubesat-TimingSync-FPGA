# =============================================================================
# build.tcl  --  Vivado non-project build of the TDC for the Arty A7-100T
# -----------------------------------------------------------------------------
# From the repository root, on Windows or Linux:
#
#   vivado -mode batch -source scripts/build.tcl
#   vivado -mode batch -source scripts/build.tcl -tclargs lut=calib/calib_lut.mem
#   vivado -mode batch -source scripts/build.tcl -tclargs part=xc7a35ticsg324-1L
#
# All paths are derived from this script's location, so it works from any
# working directory and on any OS. Outputs go to build/ (gitignored):
#   build/tdc_arty_top.bit, build/*.dcp
#   build/reports/  utilization, timing, clocks, DRC, methodology,
#                   chain_placement.rpt (PASS/FAIL), summary.txt (WNS / Fmax)
# =============================================================================

# ---- the one place to change the device ----
set PART xc7a100tcsg324-1
set TOP  tdc_arty_top
set LUT  ""

foreach arg $argv {
    if {[regexp {^part=(.+)$} $arg -> v]} { set PART $v }
    if {[regexp {^lut=(.+)$}  $arg -> v]} { set LUT  $v }
}

set ROOT [file normalize [file join [file dirname [info script]] ..]]
set OUT  [file join $ROOT build]
set RPT  [file join $OUT reports]
file mkdir $RPT

puts "== TDC build: part=$PART top=$TOP lut=[expr {$LUT eq {} ? {<ideal linear>} : $LUT}]"
set_part $PART

# ---- sources ----
read_verilog [lsort [glob [file join $ROOT rtl *.v]]]

set synth_args [list -top $TOP -part $PART -verilog_define SYNTHESIS=1]
if {$LUT ne ""} {
    # Relative LUT paths are taken from the repository root.
    if {[file pathtype $LUT] ne "absolute"} { set LUT [file join $ROOT $LUT] }
    set LUT [file normalize $LUT]
    if {![file exists $LUT]} { error "LUT file not found: $LUT" }
    lappend synth_args -generic "LUT_FILE=\"$LUT\""
}

# ---- synthesis ----
synth_design {*}$synth_args
write_checkpoint -force [file join $OUT post_synth.dcp]
report_utilization -file [file join $RPT utilization_synth.rpt]

# Constraints are applied to the synthesised netlist: the pblock and chain
# placement refer to cells that only exist after synthesis.
read_xdc [file join $ROOT constraints tdc_arty_a7.xdc]
source   [file join $ROOT constraints tdc_chain_place.tcl]

# ---- implementation ----
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force [file join $OUT post_route.dcp]

# ---- reports ----
report_utilization            -file [file join $RPT utilization.rpt]
report_utilization -hierarchical -file [file join $RPT utilization_hier.rpt]
report_timing_summary -max_paths 20 -report_unconstrained -file [file join $RPT timing_summary.rpt]
report_clock_utilization      -file [file join $RPT clock_utilization.rpt]
report_clock_interaction      -file [file join $RPT clock_interaction.rpt]
report_drc                    -file [file join $RPT drc.rpt]
report_methodology            -file [file join $RPT methodology.rpt]
report_cdc                    -file [file join $RPT cdc.rpt]

set chain_ok [tdc_check_chain_placement [file join $RPT chain_placement.rpt]]

# WNS on the system clock -> Fmax estimate. Fmax = 1 / (T - WNS) is only an
# estimate: the design was placed and routed for 4 ns, and re-running at a
# tighter constraint could come out better or worse.
set paths [get_timing_paths -group clk_sys -max_paths 1 -setup]
set wns   [get_property SLACK [lindex $paths 0]]
set per   [get_property PERIOD [get_clocks clk_sys]]
set fmax  [expr {1000.0 / ($per - $wns)}]
set util_luts [llength [get_cells -hier -filter {REF_NAME =~ LUT*}]]
set util_ffs  [llength [get_cells -hier -filter {REF_NAME =~ FD*}]]
set util_c4   [llength [get_cells -hier -filter {REF_NAME == CARRY4}]]
set util_bram [llength [get_cells -hier -filter {REF_NAME =~ RAMB*}]]
set whs [get_property SLACK [lindex [get_timing_paths -group clk_sys -max_paths 1 -hold] 0]]

set fh [open [file join $RPT summary.txt] w]
puts $fh "part            $PART"
puts $fh "lut_file        [expr {$LUT eq {} ? {ideal (none)} : $LUT}]"
puts $fh "clk_sys period  $per ns"
puts $fh "WNS (setup)     $wns ns"
puts $fh "WHS (hold)      $whs ns"
puts $fh [format "Fmax estimate   %.1f MHz" $fmax]
puts $fh "LUT cells       $util_luts   (see utilization.rpt for slice LUTs)"
puts $fh "FF cells        $util_ffs"
puts $fh "CARRY4          $util_c4"
puts $fh "RAMB            $util_bram"
puts $fh "chain placement [expr {$chain_ok ? {PASS} : {FAIL}}]  (chain_placement.rpt)"
close $fh
set fh [open [file join $RPT summary.txt] r]
puts [read $fh]
close $fh

if {!$chain_ok} {
    error "Carry chain placement check FAILED, see build/reports/chain_placement.rpt"
}
if {$wns < 0} {
    puts "WARNING: timing not met on clk_sys (WNS = $wns ns). Bitstream still written; see timing_summary.rpt"
}

# ---- bitstream ----
write_bitstream -force [file join $OUT tdc_arty_top.bit]
puts "== done: [file join $OUT tdc_arty_top.bit]"
