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
# Unless required by applicable law or agreed to in writing, softwareAU30A
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# *************************************************************************

# This file should be read in as unmanaged Tcl constraints to enable the usage
# of if statement
# set_property PACKAGE_PIN AM10 [get_ports pcie_refclk_n]
# set_property PACKAGE_PIN AM11 [get_ports pcie_refclk_p]

# set_property PACKAGE_PIN BD21 [get_ports pcie_rstn]
# set_property IOSTANDARD LVCMOS12 [get_ports pcie_rstn]

# set num_ports [llength [get_ports qsfp_refclk_p]]
# if {$num_ports >= 1} {
#     set_property PACKAGE_PIN M10 [get_ports qsfp_refclk_n[0]]
#     set_property PACKAGE_PIN M11 [get_ports qsfp_refclk_p[0]]
# }
# if {$num_ports >= 2} {
#     set_property PACKAGE_PIN T10 [get_ports qsfp_refclk_n[1]]
#     set_property PACKAGE_PIN T11 [get_ports qsfp_refclk_p[1]]
# }

# ======================PCIE RSTN=========================
#set_property PACKAGE_PIN H19 [ get_ports "pcie_rstn" ]
set_property PACKAGE_PIN AP10 [ get_ports "pcie_rstn" ]
set_property IOSTANDARD LVCMOS18 [ get_ports "pcie_rstn" ]
set_property PACKAGE_PIN U12 [ get_ports "pcie_refclk_p" ]
set_property PACKAGE_PIN U11 [ get_ports "pcie_refclk_n" ]


# ======================CMAC QSFP=========================
set_property PACKAGE_PIN AE29 [get_ports "qsfp_refclk_p"]
set_property PACKAGE_PIN AE30 [get_ports "qsfp_refclk_n"]

#set_property PACKAGE_PIN F19 [get_ports sysclk_p]
#set_property IOSTANDARD DIFF_SSTL15 [ get_ports sysclk_p ]
#set_property PACKAGE_PIN F18 [get_ports sysclk_n]
#set_property IOSTANDARD DIFF_SSTL15 [ get_ports sysclk_n ]

# ======================DDR=========================
set_property PACKAGE_PIN AU35 [ get_ports "c0_sys_clk_n" ]
set_property IOSTANDARD DIFF_SSTL12 [ get_ports "c0_sys_clk_n" ]
set c0_sys_clk_p_port [get_ports -quiet "c0_sys_clk_p"]
if {[llength $c0_sys_clk_p_port] > 0 && [llength [get_clocks -quiet -of_objects $c0_sys_clk_p_port]] == 0} {
    create_clock -period 5.0 $c0_sys_clk_p_port
}
set_property PACKAGE_PIN AU34 [ get_ports "c0_sys_clk_p" ]
set_property IOSTANDARD DIFF_SSTL12 [ get_ports "c0_sys_clk_p" ]


set_property PACKAGE_PIN AV31 [ get_ports "c0_ddr4_reset_n" ]
set_property PACKAGE_PIN AN23 [ get_ports "c0_ddr4_dq[12]" ]
set_property PACKAGE_PIN AR23 [ get_ports "c0_ddr4_dm_dbi_n[1]" ]
set_property PACKAGE_PIN AL30 [ get_ports "c0_ddr4_dq[34]" ]
set_property PACKAGE_PIN AN33 [ get_ports "c0_ddr4_dq[37]" ]
set_property PACKAGE_PIN  AL29 [ get_ports "c0_ddr4_dq[36]" ]
set_property PACKAGE_PIN  AP25 [ get_ports "c0_ddr4_dq[13]" ]
set_property PACKAGE_PIN AT25 [ get_ports "c0_ddr4_dq[11]" ]
set_property PACKAGE_PIN AP24 [ get_ports "c0_ddr4_dq[10]" ]
set_property PACKAGE_PIN AW29 [ get_ports "c0_ddr4_act_n" ]
set_property PACKAGE_PIN AP29 [ get_ports "c0_ddr4_dq[35]" ]
set_property PACKAGE_PIN AM30 [ get_ports "c0_ddr4_dq[38]" ]
set_property PACKAGE_PIN AN30 [ get_ports "c0_ddr4_dqs_t[4]" ]
set_property PACKAGE_PIN AR27 [ get_ports "c0_ddr4_dqs_c[1]" ]
set_property PACKAGE_PIN  AR26 [ get_ports "c0_ddr4_dqs_t[1]" ]
set_property PACKAGE_PIN AU32 [ get_ports "c0_ddr4_ck_t[0]" ]
set_property PACKAGE_PIN  AM28 [ get_ports "c0_ddr4_dm_dbi_n[4]" ]
set_property PACKAGE_PIN  AP28 [ get_ports "c0_ddr4_dq[39]" ]
set_property PACKAGE_PIN AN31 [ get_ports "c0_ddr4_dqs_c[4]" ]
set_property PACKAGE_PIN AP23 [ get_ports "c0_ddr4_dq[8]" ]
set_property PACKAGE_PIN  AV30 [ get_ports "c0_ddr4_adr[6]" ]
set_property PACKAGE_PIN AU33 [ get_ports "c0_ddr4_ck_c[0]" ]
set_property PACKAGE_PIN AP35 [ get_ports "c0_ddr4_adr[2]" ]
set_property PACKAGE_PIN AK31 [ get_ports "c0_ddr4_dqs_t[5]" ]
set_property PACKAGE_PIN AM33 [ get_ports "c0_ddr4_dq[47]" ]
set_property PACKAGE_PIN AK32 [ get_ports "c0_ddr4_dq[46]" ]
set_property PACKAGE_PIN AM29 [ get_ports "c0_ddr4_dq[32]" ]
set_property PACKAGE_PIN  AP26 [ get_ports "c0_ddr4_dq[15]" ]
set_property PACKAGE_PIN AN25 [ get_ports "c0_ddr4_dq[14]" ]
set_property PACKAGE_PIN AT26 [ get_ports "c0_ddr4_dq[9]" ]
set_property PACKAGE_PIN AW30 [ get_ports "c0_ddr4_adr[7]" ]
set_property PACKAGE_PIN AR31 [ get_ports "c0_ddr4_adr[3]" ]
set_property PACKAGE_PIN AL31 [ get_ports "c0_ddr4_dqs_c[5]" ]
set_property PACKAGE_PIN  AJ29 [ get_ports "c0_ddr4_dq[40]" ]
set_property PACKAGE_PIN  AN32 [ get_ports "c0_ddr4_dq[33]" ]
set_property PACKAGE_PIN AT30 [ get_ports "c0_ddr4_cke[0]" ]
set_property PACKAGE_PIN AW31 [ get_ports "c0_ddr4_adr[11]" ]
set_property PACKAGE_PIN  AT31 [ get_ports "c0_ddr4_adr[8]" ]
set_property PACKAGE_PIN AL37 [ get_ports "c0_ddr4_dq[58]" ]
set_property PACKAGE_PIN  AJ39 [ get_ports "c0_ddr4_dq[62]" ]
set_property PACKAGE_PIN AH31 [ get_ports "c0_ddr4_dq[44]" ]
set_property PACKAGE_PIN  AM32 [ get_ports "c0_ddr4_dq[41]" ]
set_property PACKAGE_PIN  AN26 [ get_ports "c0_ddr4_dq[31]" ]
set_property PACKAGE_PIN AK27 [ get_ports "c0_ddr4_dq[30]" ]
set_property PACKAGE_PIN AH26 [ get_ports "c0_ddr4_dqs_t[3]" ]
set_property PACKAGE_PIN  AR33 [ get_ports "c0_ddr4_adr[12]" ]
set_property PACKAGE_PIN AU29 [ get_ports "c0_ddr4_adr[9]" ]
set_property PACKAGE_PIN AT32 [ get_ports "c0_ddr4_adr[5]" ]
set_property PACKAGE_PIN AW28 [ get_ports "c0_ddr4_adr[4]" ]
set_property PACKAGE_PIN  AN38 [ get_ports "c0_ddr4_dq[59]" ]
set_property PACKAGE_PIN AL36 [ get_ports "c0_ddr4_dq[60]" ]
set_property PACKAGE_PIN  AM38 [ get_ports "c0_ddr4_dq[61]" ]
set_property PACKAGE_PIN AK39 [ get_ports "c0_ddr4_dq[63]" ]
set_property PACKAGE_PIN AL32 [ get_ports "c0_ddr4_dq[45]" ]
set_property PACKAGE_PIN AH32 [ get_ports "c0_ddr4_dq[43]" ]
set_property PACKAGE_PIN  AJ30 [ get_ports "c0_ddr4_dq[42]" ]
set_property PACKAGE_PIN AK28 [ get_ports "c0_ddr4_dq[28]" ]
set_property PACKAGE_PIN  AH27 [ get_ports "c0_ddr4_dqs_c[3]" ]
set_property PACKAGE_PIN AP33 [ get_ports "c0_ddr4_cs_n[0]" ]
set_property PACKAGE_PIN  AT29 [ get_ports "c0_ddr4_bg[0]" ]
set_property PACKAGE_PIN  AW34 [ get_ports "c0_ddr4_adr[10]" ]
set_property PACKAGE_PIN  AV28 [ get_ports "c0_ddr4_adr[1]" ]
set_property PACKAGE_PIN  AU28 [ get_ports "c0_ddr4_adr[0]" ]
set_property PACKAGE_PIN AM37 [ get_ports "c0_ddr4_dm_dbi_n[7]" ]
set_property PACKAGE_PIN AK37 [ get_ports "c0_ddr4_dqs_t[7]" ]
set_property PACKAGE_PIN  AK38 [ get_ports "c0_ddr4_dqs_c[7]" ]
set_property PACKAGE_PIN AH30 [ get_ports "c0_ddr4_dm_dbi_n[5]" ]
set_property PACKAGE_PIN AN27 [ get_ports "c0_ddr4_dq[29]" ]
set_property PACKAGE_PIN  AJ28 [ get_ports "c0_ddr4_dq[24]" ]
set_property PACKAGE_PIN AR34 [ get_ports "c0_ddr4_adr[15]" ]
set_property PACKAGE_PIN  AV33 [ get_ports "c0_ddr4_adr[14]" ]
set_property PACKAGE_PIN AW35 [ get_ports "c0_ddr4_adr[13]" ]
set_property PACKAGE_PIN AW26 [ get_ports "c0_ddr4_dq[5]" ]
set_property PACKAGE_PIN  AU23 [ get_ports "c0_ddr4_dq[4]" ]
set_property PACKAGE_PIN  AH33 [ get_ports "c0_ddr4_dq[52]" ]
set_property PACKAGE_PIN  AL39 [ get_ports "c0_ddr4_dq[56]" ]
set_property PACKAGE_PIN  AM39 [ get_ports "c0_ddr4_dq[57]" ]
set_property PACKAGE_PIN AH25 [ get_ports "c0_ddr4_dqs_t[2]" ]
set_property PACKAGE_PIN AH23 [ get_ports "c0_ddr4_dq[16]" ]
set_property PACKAGE_PIN AK24 [ get_ports "c0_ddr4_dq[23]" ]
set_property PACKAGE_PIN AJ23 [ get_ports "c0_ddr4_dq[22]" ]
set_property PACKAGE_PIN AK26 [ get_ports "c0_ddr4_dq[25]" ]
set_property PACKAGE_PIN AT34 [ get_ports "c0_ddr4_adr[16]" ]
set_property PACKAGE_PIN AT36 [ get_ports "c0_ddr4_ba[1]" ]
set_property PACKAGE_PIN AT35 [ get_ports "c0_ddr4_ba[0]" ]
set_property PACKAGE_PIN AT24 [ get_ports "c0_ddr4_dq[0]" ]
set_property PACKAGE_PIN AW23 [ get_ports "c0_ddr4_dm_dbi_n[0]" ]
set_property PACKAGE_PIN AM34 [ get_ports "c0_ddr4_dq[50]" ]
set_property PACKAGE_PIN AL35 [ get_ports "c0_ddr4_dq[53]" ]
set_property PACKAGE_PIN AN35 [ get_ports "c0_ddr4_dqs_t[6]" ]
set_property PACKAGE_PIN AJ25 [ get_ports "c0_ddr4_dqs_c[2]" ]
set_property PACKAGE_PIN AM25 [ get_ports "c0_ddr4_dq[17]" ]
set_property PACKAGE_PIN  AL24 [ get_ports "c0_ddr4_dq[21]" ]
set_property PACKAGE_PIN  AK22 [ get_ports "c0_ddr4_dq[20]" ]
set_property PACKAGE_PIN AL27 [ get_ports "c0_ddr4_dq[26]" ]
set_property PACKAGE_PIN  AL25 [ get_ports "c0_ddr4_dm_dbi_n[3]" ]
set_property PACKAGE_PIN  AU24 [ get_ports "c0_ddr4_dq[6]" ]
set_property PACKAGE_PIN AW25 [ get_ports "c0_ddr4_dq[1]" ]
set_property PACKAGE_PIN  AV23 [ get_ports "c0_ddr4_dq[2]" ]
set_property PACKAGE_PIN  AU30 [ get_ports "c0_ddr4_odt[0]" ]
set_property PACKAGE_PIN AK35 [ get_ports "c0_ddr4_dq[51]" ]
set_property PACKAGE_PIN AN36 [ get_ports "c0_ddr4_dqs_c[6]" ]
set_property PACKAGE_PIN  AJ33 [ get_ports "c0_ddr4_dq[54]" ]
set_property PACKAGE_PIN  AM35 [ get_ports "c0_ddr4_dq[55]" ]
set_property PACKAGE_PIN AM23 [ get_ports "c0_ddr4_dm_dbi_n[2]" ]
set_property PACKAGE_PIN  AK23 [ get_ports "c0_ddr4_dq[19]" ]
set_property PACKAGE_PIN AJ24 [ get_ports "c0_ddr4_dq[18]" ]
set_property PACKAGE_PIN AJ26 [ get_ports "c0_ddr4_dq[27]" ]
set_property PACKAGE_PIN AV27 [ get_ports "c0_ddr4_dq[7]" ]
set_property PACKAGE_PIN  AV25 [ get_ports "c0_ddr4_dqs_c[0]" ]
set_property PACKAGE_PIN AU25 [ get_ports "c0_ddr4_dqs_t[0]" ]
set_property PACKAGE_PIN AV26 [ get_ports "c0_ddr4_dq[3]" ]
set_property PACKAGE_PIN AK33 [ get_ports "c0_ddr4_dm_dbi_n[6]" ]
set_property PACKAGE_PIN AL34 [ get_ports "c0_ddr4_dq[48]" ]
set_property PACKAGE_PIN AJ34 [ get_ports "c0_ddr4_dq[49]" ]

# CameraLink input interface
set_property PACKAGE_PIN B21 [get_ports {cmlk_xData_p[0]}]
set_property PACKAGE_PIN B22 [get_ports {cmlk_xData_p[1]}]
set_property PACKAGE_PIN C20 [get_ports {cmlk_xData_p[2]}]
set_property PACKAGE_PIN H24 [get_ports {cmlk_xData_p[3]}]
set_property PACKAGE_PIN J22 [get_ports {cmlk_yData_p[0]}]
set_property PACKAGE_PIN J20 [get_ports {cmlk_yData_p[1]}]
set_property PACKAGE_PIN K20 [get_ports {cmlk_yData_p[2]}]
set_property PACKAGE_PIN L23 [get_ports {cmlk_yData_p[3]}]
set_property PACKAGE_PIN M19 [get_ports {cmlk_zData_p[0]}]
set_property PACKAGE_PIN M21 [get_ports {cmlk_zData_p[1]}]
set_property PACKAGE_PIN K23 [get_ports {cmlk_zData_p[2]}]
set_property PACKAGE_PIN L21 [get_ports {cmlk_zData_p[3]}]

set_property PACKAGE_PIN A21 [get_ports {cmlk_xData_n[0]}]
set_property PACKAGE_PIN A22 [get_ports {cmlk_xData_n[1]}]
set_property PACKAGE_PIN B20 [get_ports {cmlk_xData_n[2]}]
set_property PACKAGE_PIN G24 [get_ports {cmlk_xData_n[3]}]
set_property PACKAGE_PIN H22 [get_ports {cmlk_yData_n[0]}]
set_property PACKAGE_PIN J21 [get_ports {cmlk_yData_n[1]}]
set_property PACKAGE_PIN K21 [get_ports {cmlk_yData_n[2]}]
set_property PACKAGE_PIN L24 [get_ports {cmlk_yData_n[3]}]
set_property PACKAGE_PIN M20 [get_ports {cmlk_zData_n[0]}]
set_property PACKAGE_PIN M22 [get_ports {cmlk_zData_n[1]}]
set_property PACKAGE_PIN K24 [get_ports {cmlk_zData_n[2]}]
set_property PACKAGE_PIN L22 [get_ports {cmlk_zData_n[3]}]

set_property PACKAGE_PIN G21 [get_ports cmlk_clk_p]
set_property PACKAGE_PIN F21 [get_ports cmlk_clk_n]
set_property IOSTANDARD LVDS [get_ports {cmlk_xData_p[*]}]
set_property IOSTANDARD LVDS [get_ports {cmlk_yData_p[*]}]
set_property IOSTANDARD LVDS [get_ports {cmlk_zData_p[*]}]
set_property IOSTANDARD LVDS [get_ports {cmlk_xData_n[*]}]
set_property IOSTANDARD LVDS [get_ports {cmlk_yData_n[*]}]
set_property IOSTANDARD LVDS [get_ports {cmlk_zData_n[*]}]
set_property IOSTANDARD LVDS [get_ports cmlk_clk_p]
set_property IOSTANDARD LVDS [get_ports cmlk_clk_n]


# Optional Camera Link serial and CC control pins.
#
# Hardware used on this carrier:
#   Camera Link SerTFG/SerTC pairs -> DS90LV019TMTC -> FPGA single-ended TTL/CMOS
#   Camera Link CC1..CC4 pairs     -> DS90LV047ATMX -> FPGA single-ended TTL/CMOS inputs to driver
# Therefore the FPGA top level must be enabled with CAMERA_LINK_CONTROL_TTL_IO,
# not the legacy CAMERA_LINK_CONTROL_IO LVDS option.  Fill the package pins from
# the board schematic and choose the IOSTANDARD to match the translator VCC and
# FPGA bank VCCO.  LVCMOS33 is correct only when the translator logic side and
# bank VCCO are 3.3 V.
#
set_property verilog_define __synthesis__ [current_fileset]
set_property verilog_define CAMERA_LINK_CONTROL_TTL_IO [current_fileset]
set_property PACKAGE_PIN F9 [get_ports cmlk_sertfg_ttl] ;# DS90LV019 ROUT -> FPGA RX input
set_property PACKAGE_PIN F8 [get_ports cmlk_sertc_ttl]  ;# FPGA TX output -> DS90LV019 DIN
set_property PACKAGE_PIN L14 [get_ports {cmlk_cc_ttl[0]}] ;# FPGA CC1 -> DS90LV047A DIN1
set_property PACKAGE_PIN K13 [get_ports {cmlk_cc_ttl[1]}] ;# FPGA CC2 -> DS90LV047A DIN2
set_property PACKAGE_PIN N14 [get_ports {cmlk_cc_ttl[2]}] ;# FPGA CC3 -> DS90LV047A DIN3
set_property PACKAGE_PIN P11 [get_ports {cmlk_cc_ttl[3]}] ;# FPGA CC4 -> DS90LV047A DIN4
set_property IOSTANDARD LVCMOS33 [get_ports {cmlk_sertfg_ttl cmlk_sertc_ttl cmlk_cc_ttl[*]}]
set_property DRIVE 8 [get_ports {cmlk_sertc_ttl cmlk_cc_ttl[*]}]
set_property SLEW SLOW [get_ports {cmlk_sertc_ttl cmlk_cc_ttl[*]}]
set_property PULLUP true [get_ports cmlk_sertfg_ttl]
#
# Optional translator-enable pins.  Use this only if DE/RE/EN/EN* are routed to
# FPGA pins instead of being tied off on the PCB.
# set_property verilog_define {CAMERA_LINK_CONTROL_TTL_IO CAMERA_LINK_CONTROL_ENABLE_IO} [current_fileset]
# set_property PACKAGE_PIN <PIN> [get_ports cmlk_sertc_de]    ;# DS90LV019 DE, active high
# set_property PACKAGE_PIN <PIN> [get_ports cmlk_sertfg_re_n] ;# DS90LV019 RE, active low
# set_property PACKAGE_PIN <PIN> [get_ports cmlk_cc_en]       ;# DS90LV047A EN, active high
# set_property PACKAGE_PIN <PIN> [get_ports cmlk_cc_en_n]     ;# DS90LV047A EN*, active low/open
# set_property IOSTANDARD LVCMOS33 [get_ports {cmlk_sertc_de cmlk_sertfg_re_n cmlk_cc_en cmlk_cc_en_n}]
# set_property DRIVE 8 [get_ports {cmlk_sertc_de cmlk_sertfg_re_n cmlk_cc_en cmlk_cc_en_n}]
# set_property SLEW SLOW [get_ports {cmlk_sertc_de cmlk_sertfg_re_n cmlk_cc_en cmlk_cc_en_n}]
#
