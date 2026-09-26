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

proc tdc_chain_cells {} {
    set carries [get_cells -hier -filter {REF_NAME == CARRY4 && NAME =~ *u_dl/*}]
    set ffs     [get_cells -hier -filter {REF_NAME == FDRE   && NAME =~ *u_dl/*}]
    return [list $carries $ffs]
}

set pb [get_pblocks pb_tdc_chain]
set range [lindex [get_property GRID_RANGES $pb] 0]
if {![regexp {SLICE_X(\d+)Y(\d+):SLICE_X(\d+)Y(\d+)} $range -> x0 y0 x1 y1]} {
    error "pb_tdc_chain range '$range' is not a SLICE range"
}
if {$x0 != $x1} {
    error "pb_tdc_chain must be exactly one slice column (got X$x0..X$x1)"
}

lassign [tdc_chain_cells] carries ffs
set n [llength $carries]
if {$n > ($y1 - $y0 + 1)} {
    error "$n CARRY4s do not fit in pblock rows Y$y0..Y$y1"
}
puts "TDC: placing $n CARRY4 + [llength $ffs] capture FFs in SLICE_X${x0}Y${y0}..Y[expr {$y0 + $n - 1}]"

foreach c $carries {
    if {![regexp {g_c4\[(\d+)\]} $c -> k]} { error "cannot parse chain index from $c" }
    set_property LOC SLICE_X${x0}Y[expr {$y0 + $k}] $c
    set_property BEL CARRY4 $c
}

set bels {AFF BFF CFF DFF}
foreach f $ffs {
    if {![regexp {g_c4\[(\d+)\]\.g_ff\[(\d+)\]} $f -> k b]} { error "cannot parse tap index from $f" }
    set_property LOC SLICE_X${x0}Y[expr {$y0 + $k}] $f
    set_property BEL [lindex $bels $b] $f
}

# Called by build.tcl after routing: prove the chain is contiguous, in one
# column and in one clock region, with every FF in its CARRY4's slice.
proc tdc_check_chain_placement {rpt_file} {
    lassign [tdc_chain_cells] carries ffs
    set fh [open $rpt_file w]
    set ok 1
    set regions {}
    set cols {}
    set rows {}
    puts $fh "TDC carry-chain placement report"
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
    puts $fh [expr {$ok ? "RESULT: PASS" : "RESULT: FAIL"}]
    close $fh
    return $ok
}
