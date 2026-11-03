# vivado -mode batch -source build/rebuild_clean.tcl -tclargs <new-output> <create|synth|bitstream> <jobs>
set root [file normalize [file join [file dirname [info script]] ..]]
set out [file join $root build regenerated]
if {[llength $argv] > 0} {set out [file normalize [lindex $argv 0]]}
set stage create
if {[llength $argv] > 1} {set stage [lindex $argv 1]}
if {$stage ni {create synth bitstream}} {error "stage must be create, synth, or bitstream"}
set jobs 2
if {[llength $argv] > 2} {set jobs [lindex $argv 2]}
if {![string is integer -strict $jobs] || $jobs < 1} {error "jobs must be a positive integer"}
if {[file exists [file join $out vivado]]} {error "Use a new output directory: $out"}
if {![string match 2022.2* [version -short]]} {error "Vivado 2022.2 required to match packaged IP"}
file mkdir $out
exec python3 [file join $root build prepare_vivado.py] $out
source [file join $out create_project.tcl]
if {$stage eq "create"} {close_project; exit}
generate_target all [get_ips]
foreach ip [get_ips] {create_ip_run $ip}
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
if {![string match {*Complete*} [get_property STATUS [get_runs synth_1]]]} {error "synthesis failed; inspect runme.log"}
open_run synth_1
report_utilization -hierarchical -file [file join $out synthesis_utilization.rpt]
report_timing_summary -report_unconstrained -file [file join $out synthesis_timing.rpt]
if {$stage eq "synth"} {close_project; exit}
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
if {![string match {*write_bitstream Complete*} [get_property STATUS [get_runs impl_1]]]} {error "implementation/bitstream failed; inspect runme.log"}
open_run impl_1
report_utilization -hierarchical -file [file join $out implementation_utilization.rpt]
report_timing_summary -report_unconstrained -file [file join $out implementation_timing.rpt]
report_drc -file [file join $out implementation_drc.rpt]
report_cdc -details -file [file join $out implementation_cdc.rpt]
file copy [file join [get_property DIRECTORY [get_runs impl_1]] open_nic_shell.bit] [file join $out open_nic_shell.bit]
close_project
exit
