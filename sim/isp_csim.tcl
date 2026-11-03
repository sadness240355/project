set root [file normalize [file join [file dirname [info script]] ..]]
set out [file normalize $::env(ISP_CSIM_OUT)]
cd $out
open_project -reset project
set_top bayer_rg8_to_rgba8888
add_files [file join $root src hls bayer_rg8_to_rgba8888.cpp]
add_files -tb [file join $root sim tb_isp_fresh.cpp] -cflags "-I[file join $root src hls]"
open_solution solution -flow_target vivado
set_part xcku15p-ffve1517-2-i
create_clock -period 4
csim_design -clean -ldflags "-B/usr/bin"
exit
