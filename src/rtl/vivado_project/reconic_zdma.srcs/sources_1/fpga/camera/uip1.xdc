# Load as FILE_TYPE TCL in a project, or read_xdc -unmanaged in memory.
# Conditional object selection is not supported by managed XDC in Vivado 2022.2.
# UIP1 stopped-session mailbox. COMMIT holds the data until 2FF synchronization,
# pixel latching, 32 pipeline drain clocks, and 2FF acknowledgement complete.
set uip_src [get_cells -quiet -hier -regexp {.*(preprocessing_regs|common_isp_regs)/config_mailbox_reg.*}]
set uip_dst [get_cells -quiet -hier -regexp {.*(preprocessing_regs|common_isp_regs)/pixel_config_reg.*}]
if {[llength $uip_src] && [llength $uip_dst]} {
 set_max_delay -datapath_only 8.0 -from $uip_src -to $uip_dst
 set_bus_skew 4.0 -from $uip_src -to $uip_dst
}
set uip_sync [get_cells -quiet -hier -regexp {.*(preprocessing_regs|common_isp_regs|common_isp_regs.*)/(req_sync_reg\[0\]|ack_sync_reg\[0\])}]
if {[llength $uip_sync]} {set_false_path -to $uip_sync}
# Single-bit drain status crosses AXI -> AXI-Lite in the ERNIC adapter.
set uip_drain [get_cells -quiet -hier -regexp {.*uip_drain_sync_reg\[0\]}]
if {[llength $uip_drain]} {set_false_path -to $uip_drain}
# LUT storage is written only while capture is stopped; dirty/pending prevent START.
# Constrain its asynchronous read propagation instead of assuming clock phase relation.
set uip_lut [get_cells -quiet -hier -filter {REF_NAME =~ RAM* && NAME =~ *tone*}]
set uip_result [get_cells -quiet -hier -regexp {.*(raw_core|rgb_core|pixels)/color_lanes.*result24_reg.*}]
if {[llength $uip_lut] && [llength $uip_result]} {set_max_delay -datapath_only 8.0 -from $uip_lut -to $uip_result}
# Camera geometry/PFNC are session-constant and settled before START, like GEO1.
set uip_geometry [get_cells -quiet -hier -regexp {.*core_inst/(image_width_reg.*|camera_format_reg.*)}]
# Format also selects the processed RAW/RGB output mux feeding the legacy frame
# cache. Its stable-session paths must include that cache, not only UIP registers.
# Source selection stays limited to geometry/format; pixel data and PCIe-domain
# synchronous paths retain ordinary setup/hold timing.
set uip_pixels [get_cells -quiet -hier -filter {IS_SEQUENTIAL == 1} -regexp {.*(preprocessing_pixels|camera_store)/.*}]
if {[llength $uip_geometry] && [llength $uip_pixels]} {set_max_delay -datapath_only 8.0 -from $uip_geometry -to $uip_pixels}

puts "UIP1_CONSTRAINTS_APPLIED mailbox=[llength $uip_src]/[llength $uip_dst] sync=[llength $uip_sync] lut=[llength $uip_lut]/[llength $uip_result] geometry=[llength $uip_geometry]/[llength $uip_pixels]"

# LISP: source snapshot held until 6FF request / destination completion / 3FF ACK.
# Data has two staging flops; delayed request allows the constrained bus to settle.
set live_src [get_cells -quiet -hier -regexp {.*live_isp_regs/mailbox_reg.*}]
set live_meta [get_cells -quiet -hier -regexp {.*live_isp_regs/payload_meta_reg.*}]
if {[llength $live_src] && [llength $live_meta]} {
 set_max_delay -datapath_only 8.0 -from $live_src -to $live_meta
 set_bus_skew 4.0 -from $live_src -to $live_meta
}
set live_sync [get_cells -quiet -hier -regexp {.*live_isp_regs/(request_sync_reg\[0\]|ack_sync_reg\[0\]|failed_sync_reg\[0\])}]
if {[llength $live_sync]} {set_false_path -to $live_sync}
puts "LISP_CONSTRAINTS_APPLIED mailbox=[llength $live_src]/[llength $live_meta] sync=[llength $live_sync]"
