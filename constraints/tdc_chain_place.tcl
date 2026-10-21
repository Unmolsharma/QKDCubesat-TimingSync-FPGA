# =============================================================================
# tdc_chain_place.tcl  --  fixed placement of the TDC carry chain
# -----------------------------------------------------------------------------
# Sourced by scripts/build.tcl after the XDC, before place_design.
#
#   CARRY4 #k                   -> SLICE_X<x>Y<y0+k>, BEL CARRY4
#   capture FF for CO[b] of #k  -> same slice, BEL {A,B,C,D}FF
#
# x and y0 come from the lower-left corner of pblock pb_tdc_chain (the XDC),
# so the column is defined in exactly one place. Using the slice's own FFs
# keeps CO->D routing identical for every tap (the dedicated CY input of
# the FF D-mux), which is what makes the bins depend on the carry chain
# alone.
# =============================================================================

# Channel of a delay-line cell: tdc_multi_top names its channels ch[<i>]
# (generate block); block 1's single-channel tdc_top has no prefix -> 0.
proc tdc_cell_channel {c} {
    if {[regexp {ch\[(\d+)\]} $c -> i]} { return $i }
    return 0
}

proc tdc_chain_cells {{ch -1}} {
    set carries [get_cells -hier -filter {REF_NAME == CARRY4 && NAME =~ *u_dl/*}]
    set ffs     [get_cells -hier -filter {REF_NAME == FDRE   && NAME =~ *u_dl/*}]
    if {$ch < 0} { return [list $carries $ffs] }
    set cs {}; set fs {}
    foreach c $carries { if {[tdc_cell_channel $c] == $ch} { lappend cs $c } }
    foreach f $ffs     { if {[tdc_cell_channel $f] == $ch} { lappend fs $f } }
    return [list $cs $fs]
}

proc tdc_num_channels {} {
    set n 0
    foreach c [lindex [tdc_chain_cells] 0] {
        set i [tdc_cell_channel $c]
        if {$i + 1 > $n} { set n [expr {$i + 1}] }
    }
    return $n
}

# MULTI-CHANNEL (phase 3): channel i goes in pblock column x0 + i, same rows.
# So for NUM_CH = 5 the pblock must be 5 slice columns wide, e.g.
# SLICE_X12Y50:SLICE_X16Y99. Every column must lie inside ONE clock region
# (checked after routing). Each chain then has its own column, its own
# capture FFs, and the same 48-row extent; the per-chain delay report shows
# whether every chain covers the clock period.
set pb [get_pblocks pb_tdc_chain]
set range [lindex [get_property GRID_RANGES $pb] 0]
if {![regexp {SLICE_X(\d+)Y(\d+):SLICE_X(\d+)Y(\d+)} $range -> x0 y0 x1 y1]} {
    error "pb_tdc_chain range '$range' is not a SLICE range"
}
set nch [tdc_num_channels]
if {$x1 - $x0 + 1 != $nch} {
    error "pb_tdc_chain must be exactly $nch slice column(s) wide, one per channel (got X$x0..X$x1)"
}

set bels {AFF BFF CFF DFF}
for {set ch 0} {$ch < $nch} {incr ch} {
    lassign [tdc_chain_cells $ch] carries ffs
    set x [expr {$x0 + $ch}]
    set n [llength $carries]
    if {$n > ($y1 - $y0 + 1)} {
        error "channel $ch: $n CARRY4s do not fit in pblock rows Y$y0..Y$y1"
    }
    puts "TDC ch$ch: placing $n CARRY4 + [llength $ffs] capture FFs in SLICE_X${x}Y${y0}..Y[expr {$y0 + $n - 1}]"
    foreach c $carries {
        if {![regexp {g_c4\[(\d+)\]} $c -> k]} { error "cannot parse chain index from $c" }
        set_property LOC SLICE_X${x}Y[expr {$y0 + $k}] $c
        set_property BEL CARRY4 $c
    }
    foreach f $ffs {
        if {![regexp {g_c4\[(\d+)\]\.g_ff\[(\d+)\]} $f -> k b]} { error "cannot parse tap index from $f" }
        set_property LOC SLICE_X${x}Y[expr {$y0 + $k}] $f
        set_property BEL [lindex $bels $b] $f
    }
}

# Called by build.tcl after routing: prove the chain is contiguous, in one
# column and in one clock region, with every FF in its CARRY4's slice.
proc tdc_check_chain_placement {rpt_file} {
    set fh [open $rpt_file w]
    set ok 1
    set all_regions {}
    for {set ch 0} {$ch < [tdc_num_channels]} {incr ch} {
        puts $fh "=== channel $ch ==="
        set r [tdc_check_one_chain $fh $ch]
        set ok [expr {$ok && [lindex $r 0]}]
        lappend all_regions {*}[lindex $r 1]
    }
    set all_regions [lsort -unique $all_regions]
    if {[llength $all_regions] != 1} {
        set ok 0; puts $fh "FAIL: channels are in different clock regions: $all_regions"
    }
    puts $fh [expr {$ok ? "RESULT: PASS" : "RESULT: FAIL"}]
    close $fh
    return $ok
}

proc tdc_check_one_chain {fh ch} {
    lassign [tdc_chain_cells $ch] carries ffs
    set ok 1
    set regions {}
    set cols {}
    set rows {}
    puts $fh "TDC carry-chain placement report, channel $ch"
    puts $fh [format "%-6s %-16s %-8s %s" "index" "site" "region" "cell"]
    set sorted {}
    foreach c $carries {
        regexp {g_c4\[(\d+)\]} $c -> k
        lappend sorted [list $k $c]
    }
    foreach kc [lsort -integer -index 0 $sorted] {
        lassign $kc k c
        set site [get_property LOC $c]
        set reg  [get_property CLOCK_REGION [get_sites $site]]
        regexp {SLICE_X(\d+)Y(\d+)} $site -> sx sy
        lappend regions $reg
        lappend cols $sx
        lappend rows $sy
        puts $fh [format "%-6d %-16s %-8s %s" $k $site $reg $c]
    }
    set regions [lsort -unique $regions]
    set ucols   [lsort -unique $cols]
    if {[llength $regions] != 1} { set ok 0; puts $fh "FAIL: chain spans clock regions: $regions" }
    if {[llength $ucols] != 1}   { set ok 0; puts $fh "FAIL: chain spans columns: $ucols" }
    for {set i 1} {$i < [llength $rows]} {incr i} {
        if {[lindex $rows $i] != [lindex $rows [expr {$i-1}]] + 1} {
            set ok 0; puts $fh "FAIL: gap between index [expr {$i-1}] and $i"
        }
    }
    set bad_ff 0
    set carry_loc [dict create]
    foreach c $carries {
        regexp {g_c4\[(\d+)\]} $c -> k
        dict set carry_loc $k [get_property LOC $c]
    }
    foreach f $ffs {
        regexp {g_c4\[(\d+)\]\.g_ff\[(\d+)\]} $f -> k b
        if {[get_property LOC $f] ne [dict get $carry_loc $k]} { incr bad_ff }
    }
    if {$bad_ff} { set ok 0; puts $fh "FAIL: $bad_ff capture FFs not in their CARRY4's slice" }
    puts $fh "carry4=[llength $carries] ffs=[llength $ffs] column=X$ucols region=$regions"
    puts $fh [expr {$ok ? "channel $ch: PASS" : "channel $ch: FAIL"}]
    return [list $ok $regions]
}
