# Pure Tcl checks, shared by the pre-bitstream hook and report-fixture tests.
proc ernic_check_timing_summary {text} {
    set start [string first "Design Timing Summary" $text]
    if {$start < 0} {error "TIMING_GATE_FAILED: missing design timing summary"}
    set found 0
    foreach line [split [string range $text $start end] \n] {
        set fields [regexp -all -inline {\S+} $line]
        if {[llength $fields] != 12} {continue}
        set numeric 1
        foreach field $fields {
            if {![string is double -strict $field]} {set numeric 0; break}
        }
        if {!$numeric} {continue}
        set found 1
        lassign $fields wns tns setup_fail setup_total whs ths hold_fail hold_total wpws tpws pw_fail pw_total
        if {$setup_total <= 0 || $hold_total <= 0 || $pw_total <= 0} {
            error "TIMING_GATE_FAILED: incomplete timing coverage: $fields"
        }
        if {$wns < 0 || $tns < 0 || $setup_fail != 0 ||
            $whs < 0 || $ths < 0 || $hold_fail != 0 ||
            $wpws < 0 || $tpws < 0 || $pw_fail != 0} {
            error "TIMING_GATE_FAILED: WNS=$wns TNS=$tns setup=$setup_fail WHS=$whs THS=$ths hold=$hold_fail WPWS=$wpws TPWS=$tpws pulse_width=$pw_fail"
        }
        puts "TIMING_GATE: setup/hold/pulse-width counts passed, WNS=$wns WHS=$whs WPWS=$wpws"
        break
    }
    if {!$found} {error "TIMING_GATE_FAILED: unrecognized summary format"}
}
proc ernic_check_bus_skew {text} {
    if {[regexp -nocase {VIOLATED} $text]} {
        error "TIMING_GATE_FAILED: bus-skew violation"
    }
    if {[string first "Bus Skew Report" $text] < 0 &&
        ![regexp -nocase {no bus skew|no .*bus_skew|no paths} $text]} {
        error "TIMING_GATE_FAILED: unrecognized bus-skew report"
    }
}
