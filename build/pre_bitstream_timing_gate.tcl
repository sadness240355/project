# Runs inside impl_2 immediately before write_bitstream, against the live routed
# netlist. Never reuse the earlier route_design report after post-route phys_opt.
source [file join [file dirname [info script]] timing_gate_lib.tcl]
set tx_fifo_uram [get_cells -hier -quiet -filter {NAME =~ *tx_dma_read_engine_inst*axis_data_fifo* && REF_NAME =~ URAM*}]
if {![llength $tx_fifo_uram]} {
    error "RDMA_FIFO_GATE_FAILED: TX payload FIFO URAM storage missing; check USE_ADV_FEATURES/data_valid and payload connectivity"
}
puts "RDMA_FIFO_GATE_PASSED: payload FIFO URAM cells=[llength $tx_fifo_uram]"
set camera_host_uram [get_cells -hier -quiet -filter {NAME =~ *video_fork_inst*g_host_queue*host_queue* && REF_NAME =~ URAM*}]
if {![llength $camera_host_uram]} {
    error "CAMERA_FIFO_GATE_FAILED: host frame queue URAM storage missing"
}
puts "CAMERA_FIFO_GATE_PASSED: host frame queue URAM cells=[llength $camera_host_uram]"
set summary [report_timing_summary -delay_type min_max -report_unconstrained -check_timing_verbose -return_string]
set f [open ernic_timing_gate_summary.rpt w]
puts $f $summary
close $f
set skew [report_bus_skew -return_string]
set f [open ernic_timing_gate_bus_skew.rpt w]
puts $f $skew
close $f
# Query exact slack as well: avoid accepting a negative value rounded to -0.000.
foreach delay {max min} {
    set bad [get_timing_paths -delay_type $delay -slack_lesser_than 0 -max_paths 1]
    if {[llength $bad]} {
        error "TIMING_GATE_FAILED: $delay path slack [get_property SLACK [lindex $bad 0]] ns; see ernic_timing_gate_summary.rpt"
    }
}
ernic_check_timing_summary $summary
ernic_check_bus_skew $skew
puts "TIMING_GATE_PASSED: setup, hold, pulse-width and bus-skew checks passed."
