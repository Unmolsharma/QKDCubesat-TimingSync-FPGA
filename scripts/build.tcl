# =============================================================================
# build.tcl  --  Vivado non-project build of the TDC for the Arty A7-100T
# -----------------------------------------------------------------------------
# From the repository root, on Windows or Linux:
#
#   vivado -mode batch -source scripts/build.tcl
#   vivado -mode batch -source scripts/build.tcl -tclargs lut=calib/calib_lut.mem
#   vivado -mode batch -source scripts/build.tcl -tclargs part=xc7a35ticsg324-1L
#   vivado -mode batch -source scripts/build.tcl -tclargs top=m1_arty_top   (M1 demo)
#
# All paths are derived from this script's location, so it works from any
# working directory and on any OS. Outputs go to build/ (gitignored):
#   build/<top>.bit, build/*.dcp
#   build/reports/  utilization, timing, clocks, DRC, methodology,
#                   chain_placement.rpt (PASS/FAIL), chain_delay.rpt
#                   (PASS/WARN), summary.txt (WNS / Fmax)
# =============================================================================

# ---- the one place to change the device ----
set PART xc7a100tcsg324-1
set TOP  tdc_arty_top
set LUT  ""

foreach arg $argv {
    if {[regexp {^part=(.+)$} $arg -> v]} { set PART $v }
    if {[regexp {^lut=(.+)$}  $arg -> v]} { set LUT  $v }
    if {[regexp {^top=(.+)$}  $arg -> v]} { set TOP  $v }
}

set ROOT [file normalize [file join [file dirname [info script]] ..]]
set OUT  [file join $ROOT build]
set RPT  [file join $OUT reports]
file mkdir $RPT

puts "== TDC build: part=$PART top=$TOP lut=[expr {$LUT eq {} ? {<ideal linear>} : $LUT}]"
set_part $PART

# ---- sources ----
# rtl/<block>/*.v (tdc, nco, top); the simulation testbenches live under sim/.
read_verilog [lsort [glob [file join $ROOT rtl * *.v]]]

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
if {$TOP eq "m1_arty_top"} {
    # emulator clock input + emulator pulse output (docs/M1_DEMO.md)
    read_xdc [file join $ROOT constraints m1_arty_a7.xdc]
}
source   [file join $ROOT constraints tdc_chain_place.tcl]

# ---- implementation ----
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force [file join $OUT post_route.dcp]

# ---- carry-chain delay: does the line cover one clock period? ----
# The TDC only works if a hit edge entering CYINIT of CARRY4 #0 takes longer
# than T_clk to reach CO[3] of the last CARRY4 (README: "Chain coverage").
# The danger is a FAST chain, so the number that matters is the MIN delay at
# the fast process corner. The slow-corner MAX is reported for reference.
#
# Method: sum, along the chain, the CYINIT->CO[3] arc of CARRY4 #0, the
# CI->CO[3] arc of every later CARRY4, and the dedicated CO[3]->CI cascade
# nets, each taken at an explicit corner (timing-arc DELAY_FAST_MIN_RISE /
# DELAY_SLOW_MAX_RISE, net FAST_MIN / SLOW_MAX). Rising edge only: a hit is a
# 0->1 carry. A report_timing of the same pin pair is appended as a
# cross-check.
#
# Caveat: these are Vivado's static timing-model delays, not measured tap
# delays. Code density on hardware (sat_cnt == 0, max code < 192) remains
# the real coverage test; this report is an early warning at build time.
set CHAIN_MIN_NS 4.04   ;# T_clk (4.00 ns) + ~40 ps tap-0/LUT-origin allowance

# Everything is summed in ps. get_net_delays reports ps (and a cascade net is
# ~0-10 ps, so it must NOT be rescaled). A CARRY4 arc is ~50-150 ps, so an arc
# value below 10 can only be in ns: normalise arcs rather than trusting one
# Vivado version's unit convention for timing-arc properties.
proc tdc_ps {v} {
    if {$v eq ""} { return 0.0 }
    return [expr {abs($v) < 10.0 ? $v * 1000.0 : double($v)}]
}

proc tdc_arc_delay {cell from to prop} {
    set arc [get_timing_arcs -of_objects $cell -filter "FROM_PIN =~ */$from && TO_PIN =~ */$to"]
    if {[llength $arc] == 0} { error "no timing arc $from -> $to on $cell" }
    return [tdc_ps [get_property $prop [lindex $arc 0]]]
}

proc tdc_report_chain_delay {rpt_file min_req_ns} {
    # CARRY4s of the delay line, sorted by chain index g_c4[k]
    set idx {}
    foreach c [get_cells -hier -filter {REF_NAME == CARRY4 && NAME =~ *u_dl/*}] {
        if {![regexp {g_c4\[(\d+)\]} $c -> k]} { error "cannot parse chain index from $c" }
        lappend idx [list $k $c]
    }
    set chain {}
    foreach e [lsort -integer -index 0 $idx] { lappend chain [lindex $e 1] }
    set n [llength $chain]

    set fh [open $rpt_file w]
    puts $fh "TDC carry-chain delay, CYINIT of CARRY4 #0 -> CO\[3\] of CARRY4 #[expr {$n-1}]"
    puts $fh "Static timing model (rising edge). min = fast corner, max = slow corner."
    puts $fh ""
    puts $fh [format "%-4s %-10s %-10s %-10s %-10s  %s" k "arc_min" "arc_max" "net_min" "net_max" cell]
    set tmin 0.0
    set tmax 0.0
    for {set k 0} {$k < $n} {incr k} {
        set c [lindex $chain $k]
        set from [expr {$k == 0 ? "CYINIT" : "CI"}]
        set amin [tdc_arc_delay $c $from {CO[3]} DELAY_FAST_MIN_RISE]
        set amax [tdc_arc_delay $c $from {CO[3]} DELAY_SLOW_MAX_RISE]
        set nmin 0.0
        set nmax 0.0
        if {$k < $n - 1} {
            # cascade net CO[3] of #k -> CI of #k+1 (dedicated, ~0 but counted)
            set ci [get_pins -of_objects [lindex $chain [expr {$k+1}]] -filter {REF_PIN_NAME == CI}]
            set nd [get_net_delays -of_objects [get_nets -of_objects $ci] -to $ci]
            if {[llength $nd]} {
                set nmin [get_property FAST_MIN [lindex $nd 0]]
                set nmax [get_property SLOW_MAX [lindex $nd 0]]
            }
        }
        set tmin [expr {$tmin + $amin + $nmin}]
        set tmax [expr {$tmax + $amax + $nmax}]
        puts $fh [format "%-4d %-10s %-10s %-10s %-10s  %s" $k $amin $amax $nmin $nmax $c]
    }
    set tmin [expr {$tmin / 1000.0}]
    set tmax [expr {$tmax / 1000.0}]
    set ok [expr {$tmin > $min_req_ns}]
    set verdict [expr {$ok ? "PASS" : "WARN"}]
    puts $fh ""
    puts $fh [format "min delay (fast corner)  %.3f ns" $tmin]
    puts $fh [format "max delay (slow corner)  %.3f ns" $tmax]
    puts $fh [format "required min             > %.2f ns" $min_req_ns]
    puts $fh [format "%s: fast-corner margin %+.3f ns (%+.1f %%)" $verdict \
        [expr {$tmin - $min_req_ns}] [expr {100.0 * ($tmin - $min_req_ns) / $min_req_ns}]]

    # cross-check: Vivado's own pin-to-pin path report, both corners
    set p0 [get_pins -of_objects [lindex $chain 0] -filter {REF_PIN_NAME == CYINIT}]
    set p1 [get_pins -of_objects [lindex $chain end] -filter {REF_PIN_NAME == {CO[3]}}]
    puts $fh ""
    puts $fh "==== cross-check: report_timing -from CYINIT -to CO\[3\] (min_max) ===="
    if {[catch {report_timing -from $p0 -to $p1 -delay_type min_max -return_string} txt]} {
        puts $fh "report_timing failed: $txt"
    } else {
        puts $fh $txt
    }
    close $fh
    puts [format "TDC chain delay: min %.3f ns (fast) / max %.3f ns (slow), need > %.2f ns -> %s" \
        $tmin $tmax $min_req_ns $verdict]
    return [list $ok $tmin $tmax]
}

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
lassign [tdc_report_chain_delay [file join $RPT chain_delay.rpt] $CHAIN_MIN_NS] \
    delay_ok delay_min delay_max

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
puts $fh [format "chain delay     %s  min %.3f ns (fast) / max %.3f ns (slow), need > %.2f  (chain_delay.rpt)" \
    [expr {$delay_ok ? {PASS} : {WARN}}] $delay_min $delay_max $CHAIN_MIN_NS]
close $fh
set fh [open [file join $RPT summary.txt] r]
puts [read $fh]
close $fh

if {!$chain_ok} {
    error "Carry chain placement check FAILED, see build/reports/chain_placement.rpt"
}
if {!$delay_ok} {
    # WARN, not error: the static model is pessimistic/optimistic in ways only a
    # hardware code-density run settles. Build anyway and check sat_cnt.
    puts "WARNING: fast-corner chain delay $delay_min ns <= $CHAIN_MIN_NS ns; the line may not cover T_clk (see chain_delay.rpt)"
}
if {$wns < 0} {
    puts "WARNING: timing not met on clk_sys (WNS = $wns ns). Bitstream still written; see timing_summary.rpt"
}

# ---- bitstream ----
write_bitstream -force [file join $OUT $TOP.bit]
puts "== done: [file join $OUT $TOP.bit]"
