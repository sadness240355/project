# PCIe Gen4 core=500 MHz, AXI user=250 MHz: sequence feedback is captured
# by seqnum_fifo.data_int on EVERY core clock.  The generated IP output
# multicycle exception must not relax this 2 ns capture to 4 ns.
# Keep the IP-generated files intact. Apply this file LATE, implementation only.
set rqseq_capture [get_pins -hier -filter {NAME =~ */pcie4_uscale_plus_inst/*/seqnum_fifo.seq_fifo_0/data_int_reg*/D}]
set rqseq_source [get_pins -hier -filter {NAME =~ */pcie4_uscale_plus_inst/* && (REF_PIN_NAME =~ PCIERQSEQNUM0* || REF_PIN_NAME == PCIERQSEQNUMVLD0)}]
if {[llength $rqseq_capture] != 7 || [llength $rqseq_source] != 7} {
    error "PCIe sequence constraint expected 7 source/capture pins, got [llength $rqseq_source]/[llength $rqseq_capture]; review changed PCIe IP configuration."
}
set_multicycle_path -setup 1 -start -through $rqseq_source -to $rqseq_capture
set_multicycle_path -hold 0 -start -through $rqseq_source -to $rqseq_capture
