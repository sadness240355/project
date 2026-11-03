# *************************************************************************
#
# Copyright 2020 Xilinx, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# *************************************************************************

create_clock -name pcie_refclk -period 10.000 -waveform {0.000 5.000} [get_ports "pcie_refclk_p"]
set_false_path -through [get_ports pcie_rstn]

create_clock -name cmlk_clk -period 11.765 -waveform {0.000 5.882} [get_ports "cmlk_clk_p"]

# CameraLink IDELAYCTRL reference clock.  The clk_wiz IP normally emits this
# generated clock; this guarded fallback keeps timing constraints stable when
# generated IP XDC output is stale or absent.
set cmlk_ref_clk_pins [get_pins -quiet -hier -filter {REF_PIN_NAME == clk_out2 && NAME =~ */clk_div_inst/clk_out2}]
set cmlk_ref_src_pins [get_pins -quiet -hier -filter {REF_PIN_NAME == clk_in1 && NAME =~ */clk_div_inst/clk_in1}]
if {[llength $cmlk_ref_clk_pins] > 0 && [llength $cmlk_ref_src_pins] > 0} {
    set cmlk_ref_clk_pin [lindex $cmlk_ref_clk_pins 0]
    if {[llength [get_clocks -quiet -of_objects $cmlk_ref_clk_pin]] == 0} {
        create_generated_clock -name cmlk_ref_clk_200 \
            -source [lindex $cmlk_ref_src_pins 0] \
            -multiply_by 4 -divide_by 5 \
            $cmlk_ref_clk_pin
    }
}

# CameraLink lanes are source-synchronous serial inputs.  Their sampling phase is
# adjusted in the receiver with IDELAYE3/ISERDESE3 training logic, so constraining
# the raw lane inputs with a synthetic 0 ns input delay creates invalid hold
# checks from the package pins to the ISERDES D pins.
set cmlk_serial_ports [get_ports -quiet {
    cmlk_clk_p cmlk_clk_n
    cmlk_xData_p[*] cmlk_yData_p[*] cmlk_zData_p[*]
    cmlk_xData_n[*] cmlk_yData_n[*] cmlk_zData_n[*]
}]
set cmlk_serial_sample_pins [get_pins -quiet -hier -filter {
    REF_PIN_NAME == D && NAME =~ *camera_capture_inst/cmlk_decode_inst*iserdes*/D
}]
if {[llength $cmlk_serial_ports] > 0 && [llength $cmlk_serial_sample_pins] > 0} {
    set_false_path -from $cmlk_serial_ports -to $cmlk_serial_sample_pins
}
# XAPP1315 gearbox data is held in a 32-word distributed RAM. Bound only
# the RAM-to-pixel data paths; do not cut the entire related clock pair.
set cmlk_gearbox_rams [get_cells -quiet -hier -filter {
    REF_NAME =~ RAM* &&
    (NAME =~ *rx_channel2/rxd*.sipo/fifo_bit* || NAME =~ *rx_channel2/rxc_gen/fifo_bit*)
}]
set cmlk_gearbox_capture [get_cells -quiet -hier -filter {
    IS_SEQUENTIAL == 1 &&
    (NAME =~ *rx_channel2/rxd*.sipo/px_data_reg* ||
     NAME =~ *rx_channel2/rxd*.sipo/px_rd_last_reg* ||
     NAME =~ *rx_channel2/rxc_gen/px_data_reg* ||
     NAME =~ *rx_channel2/rxc_gen/px_rd_last_reg*)
}]
if {[llength $cmlk_gearbox_rams] && [llength $cmlk_gearbox_capture]} {
    set_max_delay -datapath_only 8.000 -from $cmlk_gearbox_rams -to $cmlk_gearbox_capture
}
# These three chains asynchronously assert reset/ready qualification and release
# through four ASYNC_REG flops in the destination clock domain. Cut only their
# asynchronous control pins; stage-to-stage D paths retain ordinary timing.
set cmlk_reset_async_pins [get_pins -quiet -hier -filter {
    NAME =~ *rxc_gen/rx_reset_sync_reg*/PRE ||
    NAME =~ *rxc_gen/px_reset_sync_reg*/PRE ||
    NAME =~ *rxc_gen/px_rx_ready_sync_reg*/CLR
}]
if {[llength $cmlk_reset_async_pins]} {set_false_path -to $cmlk_reset_async_pins}
puts "CAMERA_CDC_V7_CONSTRAINTS RAMS=[llength $cmlk_gearbox_rams] CAPTURE=[llength $cmlk_gearbox_capture] ASYNC_RESET_PINS=[llength $cmlk_reset_async_pins]"


set axis_aclk [get_clocks -of_object [get_nets axis_aclk]]
foreach cmac_clk [get_clocks -of_object [get_nets cmac_clk*]] {
    set_max_delay -datapath_only -from $axis_aclk -to $cmac_clk 4.000
    set_max_delay -datapath_only -from $cmac_clk -to $axis_aclk 3.103
}

# create_pblock pblock_packet_adapter_tx
# add_cells_to_pblock [get_pblocks pblock_packet_adapter_tx] [get_cells -quiet {cmac_port*.packet_adapter_inst/tx_inst}]
# resize_pblock [get_pblocks pblock_packet_adapter_tx] -add {CLOCKREGION_X0Y3:CLOCKREGION_X0Y4}

# create_pblock pblock_packet_adapter_rx
# add_cells_to_pblock [get_pblocks pblock_packet_adapter_rx] [get_cells -quiet {cmac_port*.packet_adapter_inst/rx_inst}]
# resize_pblock [get_pblocks pblock_packet_adapter_rx] -add {CLOCKREGION_X0Y3:CLOCKREGION_X1Y3}

# Coherent status mailboxes: source holds all bits until the destination
# publication is acknowledged and the four-phase handshake returns idle.
# Request crosses 3FF + XPM request pipeline before data is captured. At the
# 125MHz destination this leaves >24ns; bound payload delay to 8ns and skew 4ns.
set status_mailbox_source [get_cells -quiet -hier -regexp {.*(status_cdc_inst|diag_status_cdc_inst|cmlk_diag_status_cdc_inst|drop_hist_cdc|status_cdc)/mailbox_reg.*}]
set status_mailbox_parents {}
foreach cell $status_mailbox_source {
    if {[regexp {^(.*)/mailbox_reg.*} $cell ignored parent]} {lappend status_mailbox_parents $parent}
}
set status_mailbox_parents [lsort -unique $status_mailbox_parents]
if {[llength $status_mailbox_parents] != 5} {error "Expected five coherent status mailboxes: $status_mailbox_parents"}
set status_mailbox_total_source 0
set status_mailbox_total_dest 0
foreach parent $status_mailbox_parents {
    set source [get_cells -quiet -hier -filter "NAME =~ ${parent}/mailbox_reg* && IS_SEQUENTIAL == 1"]
    set dest [get_cells -quiet -hier -filter "NAME =~ ${parent}/published_reg* && IS_SEQUENTIAL == 1"]
    if {![llength $source] || ![llength $dest]} {error "Empty coherent mailbox timing group: $parent"}
    set_max_delay -datapath_only 8.000 -from $source -to $dest
    set_bus_skew 4.000 -from $source -to $dest
    incr status_mailbox_total_source [llength $source]
    incr status_mailbox_total_dest [llength $dest]
    puts "STATUS_SNAPSHOT_PER_MAILBOX parent=$parent SRC=[llength $source] DST=[llength $dest]"
}
puts "STATUS_SNAPSHOT_V7_CONSTRAINTS GROUPS=[llength $status_mailbox_parents] SRC=$status_mailbox_total_source DST=$status_mailbox_total_dest"
