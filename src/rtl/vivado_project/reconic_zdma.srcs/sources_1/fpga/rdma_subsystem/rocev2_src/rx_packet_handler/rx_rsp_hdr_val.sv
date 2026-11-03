////////////////////////////////////////////////////////////////////
// (c) Copyright 2013 - 2015 Xilinx, Inc. All rights reserved.   
//                                                               
// This file contains confidential and proprietary information   
// of Xilinx, Inc. and is protected under U.S. and               
// international copyright and other intellectual property       
// laws.                                                         
//                                                               
// DISCLAIMER                                                    
// This disclaimer is not a license and does not grant any       
// rights to the materials distributed herewith. Except as       
// otherwise provided in a valid license issued to you by        
// Xilinx, and to the maximum extent permitted by applicable    
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND       
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES   
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING     
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-        
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and      
// (2) Xilinx shall not be liable (whether in contract or tort,  
// including negligence, or under any other theory of            
// liability) for any loss or damage of any kind or nature       
// related to, arising under or in connection with these         
// materials, including for any direct, or any indirect,        
// special, incidental, or consequential loss or damage          
// (including loss of data, profits, goodwill, or any type of    
// loss or damage suffered as a result of any action brought     
// by a third party) even if such damage or loss was             
// reasonably foreseeable or Xilinx had been advised of the      
// possibility of the same.                                      
//                                                               
// CRITICAL APPLICATIONS                                         
// Xilinx products are not designed or intended to be fail-      
// safe, or for use in any application requiring fail-safe       
// performance, such as life-support or safety devices or        
// systems, Class III medical devices, nuclear facilities,       
// applications related to the deployment of airbags, or any      
// other applications that could lead to death, personal          
// injury, or severe property or environmental damage            
// (individually and collectively, "Critical                      
// Applications"). Customer assumes the sole risk and            
// liability of any use of Xilinx products in Critical           
// Applications, subject only to applicable laws and             
// regulations governing limitations on product liability.       
//                                                               
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS      
// PART OF THIS FILE AT ALL TIMES.                               
// ////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// rx_rsp_hdr_val.v
//-----------------------------------------------------------------------------
//
// Filename        : rx_rsp_hdr_val.v
// Version         : v1.0
// Description     : This module processes the incoming response packets 
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
// Naming Conventions:
//      active low signals:                     "*_n"
//      clock signals:                          "clk", "clk_div#", "clk_#x" 
//      reset signals:                          "rst", "rst_n" 
//      parameters:                             "C_*" 
//      user defined types:                     "*_TYPE" 
//      state machine next state:               "*_ns" 
//      state machine current state:            "*_cs" 
//      combinatorial signals:                  "*_com" 
//      pipelined or register delay signals:    "*_d#" 
//      counter signals:                        "*cnt*"
//      clock enable signals:                   "*_ce" 
//      internal version of output port         "*_i"
//      device pins:                            "*_pin" 
//      component instantiations:               "<MODULE>I_<#|FUNC>
//-----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// Module Declaration
// 该模块处理传入的响应数据包
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module rx_rsp_hdr_val #(
    parameter C_SIM_DEBUG = 0,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_QP_INDX_WIDTH = 8,
    parameter C_NUM_QP = 256,
    parameter PKT_DESC_FIFO_DATA_WIDTH = 26,
    parameter C_HDR_BUF_ADDR_WIDTH = 9,
    parameter C_RSP_BUF_ADDR_WIDTH = 10,
    parameter C_OSQ_PSN_WIDTH = 8
) (
    // System level interface
    input wire clk,
    input wire reset,

    // ------------------ rsp 包描述符 fifo (rsp_pkt_desc_fifo)接口 ------------------
    input  wire [PKT_DESC_FIFO_DATA_WIDTH-1:0] rsp_pkt_desc_fifo_dout,
    input  wire                                rsp_pkt_desc_fifo_empty,
    output reg                                 rd_rsp_pkt_desc_fifo,

    // ------------------ rsp 包头缓冲区(rsp_hdr_buf)接口 ------------------
    output reg  [C_HDR_BUF_ADDR_WIDTH-1:0] rsp_hdr_buf_addrb,
    output reg                             rsp_hdr_buf_enb,
    input  wire [                   511:0] rsp_hdr_buf_doutb,
    // ------------------ rsp QP ID 缓冲区(rsp_hdr_qpid_buf)接口，格式为{bth_opcode字段, Dest QP字段/qpid} ------------------
    input  wire [                    31:0] rsp_hdr_qpid_buf_doutb,

    // ------------------ XRNIC 寄存器接口 ------------------
    input wire                       i_err_buf_en,
    input wire [               47:0] i_mac_node_addr,
    input wire [               31:0] i_ipv4_node_addr,
    input wire [              127:0] i_ipv6_node_addr,
    input wire [C_QP_INDX_WIDTH-1:0] i_num_qp_enabled,  // 启用的QP数量
    input wire [                2:0] i_rnr_nak_rst_val, // 最大 RNR 重试次数

    output reg [15:0] o_acknack_pkt_cnt,  // 传入 ACK & NACK (ACK、NACK 和 RNR-NACK) 数据包计数
    output reg [15:0] o_inc_nack_pkt_cnt, // 传入 NAK (NACK 和 RNR-NACK) 数据包计数

    output wire [ 7:0] o_last_in_rsp_pkt_opcode,
    output wire [ 7:0] o_last_in_rsp_pkt_qpid,
    output wire [15:0] o_last_in_rsp_pkt_psn,

    // ------------------ QP 配置 ------------------
    input  wire [31:0] i_qp_conf,
    input  wire        qp_conf_vld,
    output reg         qp_conf_en,

    input  wire [9:0] i_stat_nak,   // {STATQPi[30:28], STATQPi[22:16]}
    output reg        stat_nak_en,
    output reg        stat_nak_we,
    output reg  [9:0] o_stat_nak,   // {STATQPi[30:28], STATQPi[22:16]}

    output reg [C_QP_INDX_WIDTH-1:0] o_qp_conf_idx,

    input  wire [23:0] i_stat_resp_psn,   // STATRESPSNi[23:0]: 预期响应的 PSN
    output reg         stat_resp_psn_en,
    output reg         stat_resp_psn_we,
    output reg  [23:0] o_stat_resp_psn,   // STATRESPSNi[23:0]: 预期响应的 PSN

    input  wire [23:0] i_sq_psn,  // SQPSNi[23:0]: SQ PSN
    output reg         sq_psn_en,

    input  wire [47:0] i_remote_qp_mac_addr,  // 目标 MAC 地址
    output reg         remote_qp_mac_addr_en,

    input  wire [127:0] i_remote_qp_ip_addr,  // 目标 IP 地址
    output reg          remote_qp_ip_addr_en,

    input wire [31:0] i_qp_timeout,  // [20:16]: 传出数据包的 RNR NACK 超时值。[13:11]: 最大 RNR 重试次数；[10:8]: 最大重试次数；[5:0]: 超时值；

    //input  wire [C_NUM_QP-1:0]                  i_qp_fatal,
    input  wire                       i_qp_fatal_wr_en,
    input  wire [C_QP_INDX_WIDTH-1:0] i_qp_fatal_wr_idx,
    input  wire                       i_qp_fatal_wr_data,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_fatal_idx,
    output reg                        set_qp_fatal,

    input wire [C_QP_INDX_WIDTH-1:0] req_qp_fatal_idx,
    input wire                       set_req_qp_fatal,

    // ------------------ resp_handler 接口 ------------------
    output wire [          C_QP_INDX_WIDTH-1:0] o_qp_mpsnbuf_idx,
    output wire                                 mpsnbuf_en,
    input  wire                                 mpsnbuf_vld,
    //input  wire [C_NUM_QP-1:0]                  i_mpsnbuf_empty,
    input  wire                                 i_mpsnbuf_empty_set_valid,
    input  wire [          C_QP_INDX_WIDTH-1:0] i_mpsnbuf_empty_set_id,
    input  wire                                 i_mpsnbuf_empty_reset_valid,
    input  wire [          C_QP_INDX_WIDTH-1:0] i_mpsnbuf_empty_reset_id,
    //input  wire [C_NUM_QP-1:0]                  i_qp_retried,
    input  wire                                 i_qp_retried_set_valid,
    input  wire [          C_QP_INDX_WIDTH-1:0] i_qp_retried_set_id,
    input  wire                                 i_qp_retried_reset_valid,
    input  wire [          C_QP_INDX_WIDTH-1:0] i_qp_retried_reset_id,
    //input  wire [C_NUM_QP-1:0]                  i_osq_empty,
    input  wire                                 i_osq_empty_set_valid,
    input  wire [          C_QP_INDX_WIDTH-1:0] i_osq_empty_set_id,
    input  wire                                 i_osq_empty_reset_valid,
    input  wire [          C_QP_INDX_WIDTH-1:0] i_osq_empty_reset_id,
    //input  wire [C_NUM_QP-1:0]                  i_osq_nacked,  
    input  wire [(4*C_QP_INDX_WIDTH+4) - 1 : 0] i_osq_nacked,
    output reg                                  o_mpsnbuf_pop,
    input  wire [          C_OSQ_PSN_WIDTH-1:0] i_os_rd_req_psn_7_0,
    input  wire [       C_M_AXI_ADDR_WIDTH-1:0] i_os_rd_req_dest_addr,
    input  wire [                         31:0] i_os_rd_req_resp_len,         // WQE.OPCODE = 8’h04 时，其为 WQE.LENGTH，否则为 0

    output reg  [C_QP_INDX_WIDTH-1:0] o_qp_acknack_idx,
    output reg                        o_acknack_vld,
    output reg  [               23:0] o_acknack_psn,       // one minus for read-first/middle
    output reg  [                7:0] o_acknack_syndr,
    output wire [               23:0] o_acknack_msn,       // AETH.MSN in ACKNOWLEDGE 
    output reg  [                1:0] o_acknack_opcode,    // 01:read-first/10:read-middle/11:read-last/only/00-acknowledge 
    output reg  [               31:0] o_qp_timeout_per_qp,

    // ------------------ rx_rsp_axi_mstr 接口 ------------------
    output wire [                    31:0] rsp_pkt_id,
    output reg  [                     2:0] rsp_pkt_type,
    output reg  [  C_M_AXI_ADDR_WIDTH-1:0] rsp_dest_addr,
    output wire [C_RSP_BUF_ADDR_WIDTH-1:0] rsp_buf_addr_offset,
    output reg  [                    12:0] rsp_pkt_len,
    output reg  [                    31:0] rsp_imm_data,
    output reg                             rsp_tnfr_vld,
    input  wire                            rdy_to_rsp_tnfr,

    // ------------------ rx_req_axi_mstr 接口 ------------------
    output reg  req_tnfr_vld,
    input  wire rdy_to_req_tnfr,

    // for ecn
    //output reg  [C_NUM_QP-1:0]                  ecn_vect,
    output reg [C_QP_INDX_WIDTH-1:0] ecn_vect_idx,
    output reg                       ecn_vect_vld,

    // Debug signals
    input  wire                       i_clr_debug_sts,
    output reg                        qp_fatal_rsp_hv_sticky,
    output reg  [               31:0] resp_drop_cnt,
    input  wire                       qp_under_recovery_wr_en,
    input  wire [C_QP_INDX_WIDTH-1:0] qp_under_recovery_wr_idx,
    input  wire                       qp_under_recovery_wr_data,

    input wire                       init_start,
    input wire [C_QP_INDX_WIDTH-1:0] init_count
);

    // Local Parameters
    localparam [2:0] PKT_TYPE_ACKNOWLEDGE_ACK = 3'd0;
    localparam [2:0] PKT_TYPE_ACKNOWLEDGE_RNR_NAK = 3'd1;
    localparam [2:0] PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR = 3'd2;  // NAK Code = PSN Sequence Error, 表明 进入的包PSN和ePSN 不匹配
    localparam [2:0] PKT_TYPE_ACKNOWLEDGE_NAK_FATAL = 3'd3;  // NAK Code != PSN Sequence Error
    localparam [2:0] PKT_TYPE_READ_ONLY = 3'd4;
    localparam [2:0] PKT_TYPE_READ_FIRST = 3'd5;
    localparam [2:0] PKT_TYPE_READ_MIDDLE = 3'd6;
    localparam [2:0] PKT_TYPE_READ_LAST = 3'd7;

    localparam [2:0] RSP_AXI_TNFR_TYPE_DISCARD = 3'd0;
    localparam [2:0] RSP_AXI_TNFR_TYPE_ERROR = 3'd1;
    localparam [2:0] RSP_AXI_TNFR_TYPE_IPV4_READ_ONLY_OR_LAST = 3'd2;
    localparam [2:0] RSP_AXI_TNFR_TYPE_IPV4_READ_FIRST = 3'd3;
    localparam [2:0] RSP_AXI_TNFR_TYPE_IPV4_READ_MIDDLE = 3'd4;
    localparam [2:0] RSP_AXI_TNFR_TYPE_IPV6_READ_ONLY_OR_LAST = 3'd5;
    localparam [2:0] RSP_AXI_TNFR_TYPE_IPV6_READ_FIRST = 3'd6;
    localparam [2:0] RSP_AXI_TNFR_TYPE_IPV6_READ_MIDDLE = 3'd7;

    localparam IPv4 = 1'b0;
    localparam IPv6 = 1'b1;

    localparam [4:0] RD_RSP_FIRST = 5'b01101;
    localparam [4:0] RD_RSP_MIDDLE = 5'b01110;
    localparam [4:0] RD_RSP_LAST = 5'b01111;
    localparam [4:0] RD_RSP_ONLY = 5'b10000;
    localparam [4:0] ACKNOWLEDGE = 5'b10001;

    localparam [1:0] ACK = 2'b00;
    localparam [1:0] NAK = 2'b11;
    localparam [1:0] RNR_NAK = 2'b01;

    localparam [4:0] PSN_SEQ_ERR = 5'b0_0000;
    localparam [4:0] INVLD_REQ = 5'b0_0001;
    localparam [4:0] REMOTE_ACC_ERR = 5'b0_0010;
    localparam [4:0] REMOTE_OP_ERR = 5'b0_0011;

    reg                             mpsnbuf_vld_r;
    reg                             hdr_ftchg_done;
    reg                             hdr_ftchg_done_r;
    reg                             hv_data_vld;  // 为高时，hdr_data_c 有效
    reg                             hdr_ftchg_done_r1;
    reg                             hv_data_vld_r;  // 为高时，hdr_data 有效
    reg                             bth_vld_mask;
    reg  [                   511:0] prev_rsp_hdr_buf_doutb;
    reg  [                  1023:0] hdr_data;  // hv_data_vld_r 为高时，hdr_data 有效
    /* Local copy of the IPv6 source/destination address bytes.  Keeping these
     * 256 bits separate prevents the wide, heavily decoded hdr_data register
     * from concentrating the IPv6 comparators and the IPv4/BTH consumers in
     * the same CLBs.  It is captured with hdr_data, so validation latency and
     * packet throughput are unchanged. */
    (* keep = "true" *) reg [255:0] ipv6_addr_hdr_data;
    wire [                  1023:0] hdr_data_c;  // hv_data_vld 为高时，hdr_data_c 有效。比 hdr_data 提前一拍

    reg  [                    31:0] pkt_id;
    reg  [C_RSP_BUF_ADDR_WIDTH-1:0] pkt_src_addr_offset;
    reg  [                    12:0] actual_pkt_len;
    reg                             pkt_fcs_err;
    reg                             ipv4_hdr_chksum_fail;
    reg                             pkt_crc_err;

    wire [                    15:0] ether_type;
    wire                            ip_ver;
    wire [                    15:0] ipv4_total_len;
    wire [                    15:0] ipv4_pyld_len;
    wire [                    15:0] ipv6_pyld_len;
    wire [                    15:0] ipv4_udp_dest_port_addr;
    wire [                    15:0] ipv6_udp_dest_port_addr;
    wire [                    15:0] ipv4_udp_len;
    wire [                    15:0] ipv6_udp_len;
    wire [                    15:0] udp_dest_port_addr;
    wire [                    15:0] ip_pyld_len;
    wire [                    15:0] udp_len;
    wire [                    15:0] total_pkt_len_advt;
    wire [                     3:0] ipv4_bth_tver;
    wire [                     3:0] ipv6_bth_tver;
    wire [                     3:0] bth_tver;
    wire [                    23:0] ipv4_bth_qp_dest_addr;
    wire [                    23:0] ipv6_bth_qp_dest_addr;
    wire [                    23:0] ipv4_bth_qp_dest_addr_c;
    wire [                    23:0] ipv6_bth_qp_dest_addr_c;
    wire [                    23:0] bth_qp_dest_addr_c;
    wire [                    23:0] bth_qp_dest_addr;
    wire [                     7:0] ipv4_bth_opcode;
    wire [                     7:0] ipv6_bth_opcode;
    wire [                     7:0] bth_opcode;
    wire [                     1:0] ipv4_bth_pad_cnt;
    wire [                     1:0] ipv6_bth_pad_cnt;
    wire [                     1:0] bth_pad_cnt;
    wire [                    23:0] ipv4_bth_psn;
    wire [                    23:0] ipv6_bth_psn;
    wire [                    23:0] bth_psn;
    wire [                     7:0] ipv4_aeth_syndr;
    wire [                     7:0] ipv6_aeth_syndr;
    wire [                     7:0] aeth_syndr;
    wire [                    23:0] ipv4_aeth_msn;
    wire [                    23:0] ipv6_aeth_msn;
    wire [                    23:0] aeth_msn;
    wire [                    23:0] qp_conf_min;
    wire [                    23:0] qp_conf_max;
    wire [                    12:0] bth_pyld_len;  // BTH负载长度(不含BTH头部的长度)，这是“最终”的有效载荷长度

    reg  [                    23:0] bth_qp_dest_addr_p;
    reg                             mpsnbuf_en_p;  // no use，看时序图用的
    reg  [     C_QP_INDX_WIDTH-1:0] o_qp_mpsnbuf_idx_p;

    wire                            mac_dest_addr_chk_fail;
    wire                            mac_src_addr_chk_fail;
    wire                            ipv4_ver_chk_fail;
    wire                            ipv4_ihl_chk_fail;
    wire                            ipv4_ecn_chk_fail;
    wire                            ipv4_flg_chk_fail;
    wire                            ipv4_frag_offset_chk_fail;
    wire                            ipv4_prot_chk_fail;
    wire                            ipv4_dest_addr_chk_fail;
    wire                            ipv4_length_chk_fail;
    wire                            ipv4_src_addr_chk_fail;
    wire                            ipv6_ver_chk_fail;
    wire                            ipv6_ecn_chk_fail;
    wire                            ipv6_next_hdr_chk_fail;
    wire                            ipv6_dest_addr_chk_fail;
    wire                            ipv6_length_chk_fail;
    wire                            ipv6_src_addr_chk_fail;
    wire                            ip_src_addr_chk_fail;
    wire                            udp_dest_port_chk_fail;
    wire                            udp_length_chk_fail;
    wire                            pkt_len_chk_fail;
    wire                            bth_ver_chk_fail;
    wire                            dest_qp_chk_fail;  // 报头目的QP不在RNIC范围
    wire                            bth_pad_cnt_chk_fail;
    reg                             aeth_malformed;  // AETH 字段不合法

    reg                             mac_dest_addr_chk_fail_r;
    reg                             ip_ver_chk_fail_r;
    reg                             ipv4_ihl_chk_fail_r;
    reg                             ip_ecn_chk_fail_r;
    reg                             ipv4_flg_chk_fail_r;
    reg                             ipv4_frag_offset_chk_fail_r;
    reg                             ip_dest_addr_chk_fail_r;
    reg                             ipv4_hdr_chksum_fail_r;
    reg                             ip_length_chk_fail_r;
    reg                             udp_length_chk_fail_r;
    reg                             pkt_len_chk_fail_r;
    reg                             bth_ver_chk_fail_r;
    reg                             dest_qp_chk_fail_r;
    reg                             bth_pad_cnt_chk_fail_r;
    reg                             ip_ver_r;
    reg  [                    23:0] bth_psn_r;
    reg  [                     7:0] bth_opcode_r;
    reg  [                    23:0] bth_qp_dest_addr_r;
    reg  [                     7:0] aeth_syndr_r;
    reg  [                    23:0] aeth_msn_r;
    reg  [                    31:0] pkt_id_r;
    reg  [C_RSP_BUF_ADDR_WIDTH-1:0] pkt_src_addr_offset_r;
    reg  [                    12:0] actual_pkt_len_r;

    wire                            qp_state_fatal;
    reg  [                    12:0] qp_pmtu_decoded;
    wire [                    23:0] i_last_ob_req_psn;
    wire [                     2:0] i_rnr_nak_cnt;  // 当前的 RNR 计数（读/写），用于接收应答数据包

    reg  [                    12:0] qp_pmtu_decoded_r;
    reg                             qp_state_fatal_r;
    reg                             aeth_malformed_r;
    reg                             ip_src_addr_chk_fail_r;
    reg                             mac_src_addr_chk_fail_r;
    reg  [                    23:0] stat_resp_psn_r;
    reg  [                    23:0] last_ob_req_psn_r;
    reg  [                     2:0] rnr_nak_cnt_r;

    wire [                    31:0] err_syndr;

    wire [                    23:0] unack_psn_min_muxed;
    wire [                    23:0] unack_psn_max_muxed;
    reg  [                    23:0] unack_psn_min;
    wire [                    23:0] unack_psn_max;

    //reg  [C_NUM_QP-1:0] mpsnbuf_empty_r;
    wire                            i_mpsnbuf_empty_combo;
    reg                             i_mpsnbuf_empty_r;
    reg                             i_mpsnbuf_empty_2r;
    wire                            i_osq_empty_combo;
    wire                            i_osq_invld;
    wire                            i_qp_retried_combo;
    wire                            qp_fatal_combo;

    wire [                    23:0] stat_resp_psn_muxed;
    wire [                    23:0] rnr_nak_cnt_muxed;
    wire [                    12:0] qp_pmtu_decoded_muxed;  // QPCONFi[10:8]: PMTU

    reg  [                    23:0] i_os_rd_req_psn;

    reg                             osq_invld_at_vld;
    wire                            osq_invld;
    reg                             is_rsp_in_unack_range_at_vld;
    wire                            is_rsp_in_unack_range;
    reg                             is_rd_req_in_unack_range_at_vld;
    wire                            is_rd_req_in_unack_range;
    reg  [                    23:0] os_rd_req_psn;
    reg  [  C_M_AXI_ADDR_WIDTH-1:0] os_rd_req_dest_addr;
    reg  [                    31:0] os_rd_req_resp_len;
    reg                             mpsnbuf_empty_at_vld;
    wire                            mpsnbuf_empty;
    reg                             qp_retried;
    reg                             ack_psn_less_os_rd;
    reg                             ack_psn_lessorequal_os_rd;
    reg                             rdrsp_psn_grtr_os_rd;
    reg                             rdrsp_psn_lessorequal_last_ob_req;

    reg  [                     2:0] pkt_type;

    wire                            rdrsp_first_len_chk_fail_1;
    wire                            rdrsp_middle_len_chk_fail_w;
    reg                             rdrsp_middle_len_chk_fail;
    wire                            rdrsp_last_len_chk_fail_w;
    reg                             rdrsp_last_len_chk_fail;
    wire                            rdrsp_only_len_chk_fail_1;
    reg                             rdrsp_first_len_chk_fail_1_r;
    reg                             rdrsp_middle_len_chk_fail_r;
    reg                             rdrsp_last_len_chk_fail_r;
    reg                             rdrsp_only_len_chk_fail_1_r;
    wire                            rdrsp_first_len_chk_fail_1_muxed;
    wire                            rdrsp_only_len_chk_fail_1_muxed;
    wire                            rdrsp_first_len_chk_fail_2;
    reg                             rdrsp_first_len_chk_fail;
    wire                            rdrsp_only_len_chk_fail_2;
    reg                             rdrsp_only_len_chk_fail;

    reg                             rdrsp_fatal_combo;
    reg  [                     4:0] rdrsp_fatal_code_combo;

    reg  [                     1:0] acknack_opcode;
    reg  [                    23:0] acknack_psn;
    reg  [                     7:0] acknack_syndr;
    reg                             acknack_vld;
    reg  [                     2:0] o_rnr_nak_cnt;
    wire [                    31:0] rdrsp_len_remain;
    wire [  C_M_AXI_ADDR_WIDTH-1:0] i_rdrsp_dest_addr;
    wire [                    31:0] i_rdrsp_len_remain;
    reg  [                    31:0] o_rdrsp_len_remain;
    reg                             incr_acknack_pkt_cnt;
    reg                             incr_inc_nack_pkt_cnt;

    //reg [C_NUM_QP-1:0] last_rsp_opcode_reg;
    wire                            last_rsp_opcode_combo;
    reg                             last_rsp_opcode_we;
    reg                             i_last_rsp_opcode;
    reg                             o_last_rsp_opcode;

    reg  [                     4:0] in_err_sts_code;  // 手册 FATAL Codes
    reg  [                     4:0] nxt_in_err_sts_code;  // 手册 FATAL Codes

    reg  [                     2:0] rsp_pkt_type_r;
    reg  [  C_M_AXI_ADDR_WIDTH-1:0] rsp_dest_addr_r;
    reg  [                    31:0] rsp_imm_data_r;

    reg                             mpsnbuf_pop;
    reg  [                     2:0] hv_fsm_cs;
    reg  [                     2:0] hv_fsm_ns;

    reg                             hv_fsm_busy;

    reg  [     C_QP_INDX_WIDTH-1:0] req_qp_fatal_idx_r;
    reg                             clr_qp_fatal_flag;
    reg                             req_qp_fatal_flag;

    reg  [                     2:0] rnr_nak_rst_val_r;

    //reg [C_NUM_QP-1:0]        nxt_ecn_vect;
    reg  [     C_QP_INDX_WIDTH-1:0] nxt_ecn_vect_idx;
    reg                             nxt_ecn_update_flg;
    reg                             ecn_update_flg;

    reg                             buf_en_int;

    reg                             stat_nak_en_int;
    reg                             stat_nak_we_int;
    wire [                     9:0] o_stat_nak_int;

    reg                             stat_resp_psn_en_int;
    reg                             stat_resp_psn_we_int;
    reg  [                    23:0] o_stat_resp_psn_int;

    reg  [                    23:0] rdrsp_dest_addr_idx;
    reg                             rdrsp_dest_addr_en;
    reg                             rdrsp_dest_addr_we;
    reg                             rdrsp_dest_addr_en_int;
    reg                             rdrsp_dest_addr_we_int;
    reg  [  C_M_AXI_ADDR_WIDTH-1:0] o_rdrsp_dest_addr;

    reg  [                    23:0] rdrsp_len_remain_idx;
    reg                             rdrsp_len_remain_en;
    reg                             rdrsp_len_remain_we;
    reg                             rdrsp_len_remain_en_int;
    reg                             rdrsp_len_remain_we_int;
    //output wire [C_QP_INDX_WIDTH-1:0]           o_qp_fatal_idx,
    //output reg                                  set_qp_fatal,
    wire                            osq_nacked_combo_int;
    reg                             osq_nacked_combo;
    wire                            qp_recovery_combo;
    reg                             qp_under_recovery_wr_en_r;

    // 时序对齐，与Intiator交互
    always @(posedge clk) begin
        if (reset) begin
            o_qp_mpsnbuf_idx_p    <= 0;
            mpsnbuf_en_p          <= 0;

            o_qp_conf_idx         <= 0;
            qp_conf_en            <= 0;

            stat_nak_en           <= 0;
            stat_nak_we           <= 0;
            o_stat_nak            <= 0;

            stat_resp_psn_en      <= 0;
            stat_resp_psn_we      <= 0;
            o_stat_resp_psn       <= 0;

            sq_psn_en             <= 0;
            remote_qp_mac_addr_en <= 0;
            remote_qp_ip_addr_en  <= 0;

            rdrsp_dest_addr_idx   <= 0;
            rdrsp_dest_addr_en    <= 0;
            rdrsp_dest_addr_we    <= 0;
            o_rdrsp_dest_addr     <= 0;


            rdrsp_len_remain_idx  <= 0;
            rdrsp_len_remain_en   <= 0;
            rdrsp_len_remain_we   <= 0;

            bth_vld_mask          <= 0;
        end else begin
            if (hv_data_vld == 1'b1 && bth_vld_mask == 1'b0) begin
                o_qp_conf_idx        <= bth_qp_dest_addr_c[C_QP_INDX_WIDTH-1:0];
                o_qp_mpsnbuf_idx_p   <= bth_qp_dest_addr_c[C_QP_INDX_WIDTH-1:0];
                rdrsp_dest_addr_idx  <= bth_qp_dest_addr_c[C_QP_INDX_WIDTH-1:0];
                rdrsp_len_remain_idx <= bth_qp_dest_addr_c[C_QP_INDX_WIDTH-1:0];
            end

            if (qp_conf_en) bth_vld_mask <= 1'b1;
            else if (!hv_fsm_busy) bth_vld_mask <= 1'b0;

            qp_conf_en            <= buf_en_int;
            mpsnbuf_en_p          <= buf_en_int;

            sq_psn_en             <= buf_en_int;
            remote_qp_mac_addr_en <= buf_en_int;
            remote_qp_ip_addr_en  <= buf_en_int;

            stat_nak_en           <= stat_nak_en_int;
            stat_nak_we           <= stat_nak_we_int;
            o_stat_nak            <= o_stat_nak_int;

            stat_resp_psn_en      <= stat_resp_psn_en_int;
            stat_resp_psn_we      <= stat_resp_psn_we_int;
            o_stat_resp_psn       <= o_stat_resp_psn_int;

            o_rdrsp_dest_addr     <= rsp_dest_addr + qp_pmtu_decoded_r;
            rdrsp_dest_addr_en    <= rdrsp_dest_addr_en_int;
            rdrsp_dest_addr_we    <= rdrsp_dest_addr_we_int;


            rdrsp_len_remain_en   <= rdrsp_len_remain_en_int;
            rdrsp_len_remain_we   <= rdrsp_len_remain_we_int;
        end
    end

    always @(posedge clk) if (set_req_qp_fatal) req_qp_fatal_idx_r <= req_qp_fatal_idx;

    always @(posedge clk)
        if (reset) req_qp_fatal_flag <= 1'b0;
        else if (set_req_qp_fatal || clr_qp_fatal_flag) req_qp_fatal_flag <= set_req_qp_fatal;

    //===============================
    // Utility functions
    //===============================
    // x < y ?
    function automatic is_x_less_y;  // 用于比较两个24位有符号数的函数，判断x是否小于y
        input [23:0] x;
        input [23:0] y;
        begin
            is_x_less_y = 1'b0;

            // x[23]为符号位，1表示负数，0表示正数
            if (x[23] && ((x < y) || ({1'b0, x[22:0]} >= y))) is_x_less_y = 1'b1;
            else if (!x[23] && (x < y) && ({1'b1, x[22:0]} >= y)) is_x_less_y = 1'b1;
        end
    endfunction

    // x > y ?
    function automatic is_x_grtr_y;  // 用于比较两个24位有符号数的函数，判断x是否大于y
        input [23:0] x;
        input [23:0] y;
        begin
            is_x_grtr_y = is_x_less_y(y, x);
        end
    endfunction

    // x <= y ?
    function automatic is_x_lessorequal_y;  // 用于比较两个24位有符号数的函数，判断x是否小于等于y
        input [23:0] x;
        input [23:0] y;
        begin
            is_x_lessorequal_y = !is_x_less_y(y, x);
        end
    endfunction

    // x >= y ?
    function automatic is_x_grtrorequal_y;  // 用于比较两个24位有符号数的函数，判断x是否大于等于y
        input [23:0] x;
        input [23:0] y;
        begin
            is_x_grtrorequal_y = !is_x_less_y(x, y);
        end
    endfunction

    // a 在 [y, x] ？，这里要求 y >= x
    function automatic is_in_range;
        input [23:0] x;
        input [23:0] y;
        input [23:0] a;
        begin
            is_in_range = 1'b0;
            if (is_x_lessorequal_y(x, y)) begin
                if ((x > y) && ((a >= x) || (a <= y))) is_in_range = 1'b1;
                else if (!(x > y) && (a >= x) && (a <= y)) is_in_range = 1'b1;
            end
        end
    endfunction
    //===============================

    //===============================
    // 头信息获取控制信号逻辑
    //===============================  
    always @(posedge clk) begin
        if (reset) begin
            hdr_ftchg_done         <= 1'b0;
            hdr_ftchg_done_r       <= 1'b0;
            rsp_hdr_buf_enb        <= 1'b1;
            rd_rsp_pkt_desc_fifo   <= 1'b0;
            rsp_hdr_buf_addrb      <= {(C_HDR_BUF_ADDR_WIDTH) {1'b0}};
            hdr_ftchg_done_r1      <= 1'b0;
            prev_rsp_hdr_buf_doutb <= rsp_hdr_buf_doutb;
            mpsnbuf_vld_r          <= 1'b0;
        end else begin
            mpsnbuf_vld_r <= mpsnbuf_vld;

            if (rsp_pkt_desc_fifo_empty == 1'b0 && hv_fsm_busy == 1'b0 && hdr_ftchg_done == 1'b0) begin
                rsp_hdr_buf_enb        <= 1'b1;
                rd_rsp_pkt_desc_fifo   <= 1'b1;

                hdr_ftchg_done         <= 1'b1;
                hdr_ftchg_done_r       <= 1'b0;
                hdr_ftchg_done_r1      <= 1'b0;

                rsp_hdr_buf_addrb      <= rsp_hdr_buf_addrb + 'd1;
                prev_rsp_hdr_buf_doutb <= rsp_hdr_buf_doutb;
            end else if (hdr_ftchg_done == 1'b1 && hv_fsm_busy == 1'b0 && hdr_ftchg_done_r == 1'b0 && hdr_ftchg_done_r1 == 1'b0) begin
                rsp_hdr_buf_enb      <= 1'b0;
                rd_rsp_pkt_desc_fifo <= 1'b0;
                hdr_ftchg_done_r1    <= 1'b1;
            end else if (hdr_ftchg_done == 1'b1 && mpsnbuf_vld_r == 1'b1 && hdr_ftchg_done_r == 1'b0) begin
                rsp_hdr_buf_enb      <= 1'b1;
                rd_rsp_pkt_desc_fifo <= 1'b0;
                hdr_ftchg_done       <= 1'b0;
                hdr_ftchg_done_r     <= 1'b1;
                rsp_hdr_buf_addrb    <= rsp_hdr_buf_addrb + 'd1;
                hdr_ftchg_done_r1    <= 1'b0;
            end else begin
                rd_rsp_pkt_desc_fifo <= 1'b0;
                hdr_ftchg_done_r1    <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            hdr_data <= 'd0;
            ipv6_addr_hdr_data <= 'd0;
        end else if (rd_rsp_pkt_desc_fifo == 1'b1) begin
            hdr_data[511:0] <= rsp_hdr_buf_doutb;
            ipv6_addr_hdr_data <= rsp_hdr_buf_doutb[(22*8)+:256];
        end
        else if (hdr_ftchg_done_r1 == 1'b1) hdr_data[1023:512] <= rsp_hdr_buf_doutb;
    end

    assign hdr_data_c              = {rsp_hdr_buf_doutb, hdr_data[511:0]};
    assign ipv4_bth_qp_dest_addr_c = {hdr_data_c[47*8+:8], hdr_data_c[48*8+:8], hdr_data_c[49*8+:8]} >= 24'd1 ? {hdr_data_c[47*8+:8], hdr_data_c[48*8+:8], hdr_data_c[49*8+:8]} - 24'd1 : 24'd0;
    assign ipv6_bth_qp_dest_addr_c = {hdr_data_c[67*8+:8], hdr_data_c[68*8+:8], hdr_data_c[69*8+:8]} >= 24'd1 ? {hdr_data_c[67*8+:8], hdr_data_c[68*8+:8], hdr_data_c[69*8+:8]} - 24'd1 : 24'd0;
    assign bth_qp_dest_addr_c      = (ip_ver == IPv4) ? ipv4_bth_qp_dest_addr_c : ipv6_bth_qp_dest_addr_c;

    always @(posedge clk) begin
        if (reset) bth_qp_dest_addr_p <= 'd0;
        else if (rd_rsp_pkt_desc_fifo == 1'b1) bth_qp_dest_addr_p <= rsp_hdr_qpid_buf_doutb[23:0];
    end

    assign mpsnbuf_en       = buf_en_int;
    assign o_qp_mpsnbuf_idx = bth_qp_dest_addr_p[C_QP_INDX_WIDTH-1:0];


    always @(posedge clk) begin
        if (reset) hv_data_vld <= 1'b0;
        else if (!hv_fsm_busy) hv_data_vld <= rd_rsp_pkt_desc_fifo;
    end

    always @(posedge clk)
        if (reset) hv_data_vld_r <= 1'b0;
        else if (!hv_fsm_busy) hv_data_vld_r <= 1'b0;
        else if (hv_data_vld) hv_data_vld_r <= 1'b1;

    generate
        if (C_SIM_DEBUG) begin
            always @(posedge clk) begin
                if (rsp_pkt_desc_fifo_empty == 1'b0 && hv_fsm_busy == 1'b0 && hdr_ftchg_done == 1'b0) pkt_id <= rsp_pkt_desc_fifo_dout[(C_RSP_BUF_ADDR_WIDTH+16)+:32];
            end
        end else begin
            always @(posedge clk) begin
                if (rsp_pkt_desc_fifo_empty == 1'b0 && hv_fsm_busy == 1'b0 && hdr_ftchg_done == 1'b0) pkt_id <= 32'd0;
            end
        end
    endgenerate

    // Descriptor FIFO is FWFT (cmac_rx_intf.rsp_pkt_desc_fifo). Capture its
    // current head on the existing registered consume strobe, before that edge
    // advances the FIFO. This is one clock after admission, still before
    // hv_data_vld_r copies the offset to pkt_src_addr_offset_r.
    // Remove PSN -> combinational hv_fsm_busy -> descriptor offset CE.
    // Header FSM and its admission/retirement cycles are unchanged.
    always @(posedge clk) begin
        if (reset)
            pkt_src_addr_offset <= 'd0;
        else if (rd_rsp_pkt_desc_fifo)
            pkt_src_addr_offset <= rsp_pkt_desc_fifo_dout[16+:C_RSP_BUF_ADDR_WIDTH];
    end

    always @(posedge clk) begin
        if (reset) begin
            actual_pkt_len       <= 'd0;
            pkt_fcs_err          <= 1'b0;
            ipv4_hdr_chksum_fail <= 1'b0;
            pkt_crc_err          <= 1'b0;
        end else begin
            if (rsp_pkt_desc_fifo_empty == 1'b0 && hv_fsm_busy == 1'b0 && hdr_ftchg_done == 1'b0) begin
                actual_pkt_len       <= rsp_pkt_desc_fifo_dout[15:3];
                pkt_fcs_err          <= rsp_pkt_desc_fifo_dout[2];
                ipv4_hdr_chksum_fail <= rsp_pkt_desc_fifo_dout[1];
                pkt_crc_err          <= rsp_pkt_desc_fifo_dout[0];
            end
        end
    end

    // =============================== 

    //===============================
    // 头部有效性检查
    //=============================== 

    //////////// MAC header specific checks //////////////  
    // 包头目的MAC地址 != 本节点MAC地址
    assign mac_dest_addr_chk_fail = ({hdr_data[(0*8)+:8], hdr_data[(1*8)+:8], hdr_data[(2*8)+:8], hdr_data[(3*8)+:8], hdr_data[(4*8)+:8], hdr_data[(5*8)+:8]} != i_mac_node_addr) ? 1'b1 : 1'b0;

    // 以太网类型字段
    assign ether_type = {hdr_data[12*8+:8], hdr_data[13*8+:8]};
    assign ip_ver = (ether_type == 16'h0800) ? IPv4 : IPv6;

    // 包头源MAC地址 != 该QP的目的MAC地址
    assign mac_src_addr_chk_fail = ({hdr_data[(6*8)+:8], hdr_data[(7*8)+:8], hdr_data[(8*8)+:8], hdr_data[(9*8)+:8], hdr_data[(10*8)+:8], hdr_data[(11*8)+:8]} != i_remote_qp_mac_addr) ? 1'b1 : 1'b0;

    //////////// IPv4 header specific checks //////////////  
    assign ipv4_ver_chk_fail = (hdr_data[(14*8+4)+:4] != 4'h4) ? 1'b1 : 1'b0;
    assign ipv4_ihl_chk_fail = (hdr_data[(14*8)+:4] != 4'h5) ? 1'b1 : 1'b0;
    // assign ipv4_ecn_chk_fail         = (hdr_data[(15*8  )+: 2] ==  2'h3 ) ? 1'b1 : 1'b0;
    assign ipv4_ecn_chk_fail = 1'b0;
    assign ipv4_flg_chk_fail = (hdr_data[(20*8+5)+:3] != 3'h2) ? 1'b1 : 1'b0;
    assign ipv4_frag_offset_chk_fail = ({hdr_data[(20*8)+:5], hdr_data[(21*8)+:8]} != 13'h0) ? 1'b1 : 1'b0;
    assign ipv4_prot_chk_fail = (hdr_data[(23*8)+:8] != 8'h11) ? 1'b1 : 1'b0;
    assign ipv4_dest_addr_chk_fail = (i_ipv4_node_addr != {hdr_data[(30*8)+:8], hdr_data[(31*8)+:8], hdr_data[(32*8)+:8], hdr_data[(33*8)+:8]}) ? 1'b1 : 1'b0;
    assign ipv4_total_len = {hdr_data[16*8+:8], hdr_data[17*8+:8]};
    assign ipv4_pyld_len = ipv4_total_len - 16'd20;
    assign ipv4_length_chk_fail = ((ipv4_total_len >= 16'd44) && (ipv4_total_len <= 16'd4144)) ? 1'b0 : 1'b1;
    assign ipv4_src_addr_chk_fail = (i_remote_qp_ip_addr[31:0] != {hdr_data[(26*8)+:8], hdr_data[(27*8)+:8], hdr_data[(28*8)+:8], hdr_data[(29*8)+:8]}) ? 1'b1 : 1'b0;

    //////////// IPv6 header specific checks //////////////
    assign ipv6_ver_chk_fail = (hdr_data[(14*8+4)+:4] != 4'h6) ? 1'b1 : 1'b0;
    // assign ipv6_ecn_chk_fail       = (hdr_data[(15*8+4)+:2] ==  2'h3 ) ? 1'b1 : 1'b0;
    assign ipv6_ecn_chk_fail = 1'b0;
    assign ipv6_next_hdr_chk_fail = (hdr_data[(20*8)+:8] != 8'h11) ? 1'b1 : 1'b0;
    assign ipv6_dest_addr_chk_fail = (i_ipv6_node_addr != {
                   ipv6_addr_hdr_data[(16*8)+:8], ipv6_addr_hdr_data[(17*8)+:8], ipv6_addr_hdr_data[(18*8)+:8],
                   ipv6_addr_hdr_data[(19*8)+:8], ipv6_addr_hdr_data[(20*8)+:8], ipv6_addr_hdr_data[(21*8)+:8],
                   ipv6_addr_hdr_data[(22*8)+:8], ipv6_addr_hdr_data[(23*8)+:8], ipv6_addr_hdr_data[(24*8)+:8],
                   ipv6_addr_hdr_data[(25*8)+:8], ipv6_addr_hdr_data[(26*8)+:8], ipv6_addr_hdr_data[(27*8)+:8],
                   ipv6_addr_hdr_data[(28*8)+:8], ipv6_addr_hdr_data[(29*8)+:8], ipv6_addr_hdr_data[(30*8)+:8],
                   ipv6_addr_hdr_data[(31*8)+:8]}) ? 1'b1 : 1'b0;
    assign ipv6_pyld_len = {hdr_data[18*8+:8], hdr_data[19*8+:8]};
    assign ipv6_length_chk_fail = ((ipv6_pyld_len >= 16'd16) && (ipv6_pyld_len <= 16'd4124)) ? 1'b0 : 1'b1;
    assign ipv6_src_addr_chk_fail  = (i_remote_qp_ip_addr != {
                   ipv6_addr_hdr_data[(0*8)+:8], ipv6_addr_hdr_data[(1*8)+:8], ipv6_addr_hdr_data[(2*8)+:8],
                   ipv6_addr_hdr_data[(3*8)+:8], ipv6_addr_hdr_data[(4*8)+:8], ipv6_addr_hdr_data[(5*8)+:8],
                   ipv6_addr_hdr_data[(6*8)+:8], ipv6_addr_hdr_data[(7*8)+:8], ipv6_addr_hdr_data[(8*8)+:8],
                   ipv6_addr_hdr_data[(9*8)+:8], ipv6_addr_hdr_data[(10*8)+:8], ipv6_addr_hdr_data[(11*8)+:8],
                   ipv6_addr_hdr_data[(12*8)+:8], ipv6_addr_hdr_data[(13*8)+:8], ipv6_addr_hdr_data[(14*8)+:8],
                   ipv6_addr_hdr_data[(15*8)+:8]}) ? 1'b1 : 1'b0;

    assign ip_src_addr_chk_fail = (ip_ver == IPv4) ? ipv4_src_addr_chk_fail : ipv6_src_addr_chk_fail;

    //////////// UDP header specific checks //////////////
    assign ipv4_udp_dest_port_addr = {hdr_data[36*8+:8], hdr_data[37*8+:8]};
    assign ipv6_udp_dest_port_addr = {hdr_data[56*8+:8], hdr_data[57*8+:8]};
    assign ipv4_udp_len = {hdr_data[38*8+:8], hdr_data[39*8+:8]};
    assign ipv6_udp_len = {hdr_data[58*8+:8], hdr_data[59*8+:8]};
    assign udp_dest_port_addr = (ip_ver == IPv4) ? ipv4_udp_dest_port_addr : ipv6_udp_dest_port_addr;
    assign ip_pyld_len = (ip_ver == IPv4) ? ipv4_pyld_len : ipv6_pyld_len;
    assign udp_len = (ip_ver == IPv4) ? ipv4_udp_len : ipv6_udp_len;
    assign total_pkt_len_advt = (ip_ver == IPv4) ? (ipv4_udp_len + 16'd34) : (ipv6_udp_len + 16'd54);
    assign udp_dest_port_chk_fail = (udp_dest_port_addr != 16'd4791) ? 1'b1 : 1'b0;
    assign udp_length_chk_fail = (ip_pyld_len != udp_len) ? 1'b1 : 1'b0;
    assign pkt_len_chk_fail = ((total_pkt_len_advt != {3'b000, actual_pkt_len}) || (actual_pkt_len[1:0] != 2'b10)) ? 1'b1 : 1'b0;

    /////////// BTH header specific checks //////////////
    assign ipv4_bth_tver = hdr_data[43*8+:4];
    assign ipv6_bth_tver = hdr_data[63*8+:4];
    assign ipv4_bth_qp_dest_addr = {hdr_data[47*8+:8], hdr_data[48*8+:8], hdr_data[49*8+:8]} >= 24'd1 ? {hdr_data[47*8+:8], hdr_data[48*8+:8], hdr_data[49*8+:8]} - 24'd1 : 24'd0;
    assign ipv6_bth_qp_dest_addr = {hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]} >= 24'd1 ? {hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]} - 24'd1 : 24'd0;
    assign ipv4_bth_opcode = hdr_data[42*8+:8];
    assign ipv6_bth_opcode = hdr_data[62*8+:8];
    assign ipv4_bth_pad_cnt = hdr_data[(43*8+4)+:2];
    assign ipv6_bth_pad_cnt = hdr_data[(63*8+4)+:2];
    assign ipv4_bth_psn = {hdr_data[51*8+:8], hdr_data[52*8+:8], hdr_data[53*8+:8]};
    assign ipv6_bth_psn = {hdr_data[71*8+:8], hdr_data[72*8+:8], hdr_data[73*8+:8]};
    assign ipv4_aeth_syndr = hdr_data[54*8+:8];
    assign ipv6_aeth_syndr = hdr_data[74*8+:8];
    assign ipv4_aeth_msn = {hdr_data[55*8+:8], hdr_data[56*8+:8], hdr_data[57*8+:8]};
    assign ipv6_aeth_msn = {hdr_data[75*8+:8], hdr_data[76*8+:8], hdr_data[77*8+:8]};

    assign bth_ver = (ip_ver == IPv4) ? ipv4_bth_tver : ipv6_bth_tver;  // BTH TVer字段
    assign bth_qp_dest_addr = (ip_ver == IPv4) ? ipv4_bth_qp_dest_addr : ipv6_bth_qp_dest_addr;  // BTH Destination QP字段
    assign bth_opcode = (ip_ver == IPv4) ? ipv4_bth_opcode : ipv6_bth_opcode;  // BTH Opcode字段
    assign bth_pad_cnt = (ip_ver == IPv4) ? ipv4_bth_pad_cnt : ipv6_bth_pad_cnt;  // BTH Pad字段
    assign bth_psn = (ip_ver == IPv4) ? ipv4_bth_psn : ipv6_bth_psn;  // BTH PSN字段
    assign aeth_syndr = (ip_ver == IPv4) ? ipv4_aeth_syndr : ipv6_aeth_syndr;  // AETH Syndrome字段
    assign aeth_msn = (ip_ver == IPv4) ? ipv4_aeth_msn : ipv6_aeth_msn;  // AETH MSN字段

    assign bth_ver_chk_fail = (bth_ver != 4'd0) ? 1'b1 : 1'b0;
    assign bth_pad_cnt_chk_fail = (((bth_opcode[4:0] == RD_RSP_FIRST) || (bth_opcode[4:0] == RD_RSP_MIDDLE)) && (bth_pad_cnt != 2'd0)) ? 1'b1 : 1'b0;

    // 13'd58 = 14 (Ethernet header) + 20 (IPv4 header) + 8 (UDP header) + 16 (BTH header) + 4 (ICRC)
    assign bth_pyld_len = (ip_ver == IPv4) ? (actual_pkt_len - 13'd58) : (actual_pkt_len - 13'd78);

    // AETH 格式检查，检查头部是否符合规范
    always @(*) begin
        aeth_malformed = 1'b0;
        // ACK包(AETH)
        if (bth_opcode[4:0] == ACKNOWLEDGE) begin
            if (bth_pyld_len != 'd4) aeth_malformed = 1'b1;  // 不符合
            else if ((aeth_syndr[7]) || (aeth_syndr[6:5] == 2'b10)) aeth_malformed = 1'b1;
            else if ((aeth_syndr[6:5] == 2'b11) && (aeth_syndr[4:0] > 5'b0_0011)) aeth_malformed = 1'b1;
        end //
        // RDMA READ response包(AETH, PayLD)
        else if ((bth_opcode[4:0] == RD_RSP_ONLY) || (bth_opcode[4:0] == RD_RSP_FIRST) || (bth_opcode[4:0] == RD_RSP_LAST)) begin
            if (bth_pyld_len <= 'd4) aeth_malformed = 1'b1;  // 不符合
            else if ((aeth_syndr[7]) || (aeth_syndr[6:5] == 2'b10)) aeth_malformed = 1'b1;
            else if ((aeth_syndr[6:5] == 2'b11) && (aeth_syndr[4:0] > 5'b0_0011)) aeth_malformed = 1'b1;
        end
    end

    // 检查目的QP号是否在 RNIC 已配置 QP号的有效范围内
    // 启用QP要顺序启用，不支持离散 QP
    // tip: 启用QP数量为3时，启用的QP就不能是QP4及以后的QP，只能是QP1、QP2、QP3
    // 且 QP 启用数最大为 C_NUM_QP - 1，因为qpid-1改为qpid的原因，索引0为空的
    assign qp_conf_min      = 24'd0;
    assign qp_conf_max      = {{(24 - C_QP_INDX_WIDTH) {1'b0}}, i_num_qp_enabled};
    assign dest_qp_chk_fail = (bth_qp_dest_addr >= qp_conf_max) ? 1'b1 : 1'b0;

    always @(posedge clk) begin
        if (hv_data_vld_r) begin
            mac_dest_addr_chk_fail_r    <= mac_dest_addr_chk_fail;
            ip_ver_chk_fail_r           <= (ip_ver == IPv4) ? ipv4_ver_chk_fail : ipv6_ver_chk_fail;
            ipv4_ihl_chk_fail_r         <= (ip_ver == IPv4) ? ipv4_ihl_chk_fail : 1'b0;
            ip_ecn_chk_fail_r           <= (ip_ver == IPv4) ? ipv4_ecn_chk_fail : ipv6_ecn_chk_fail;
            ipv4_flg_chk_fail_r         <= (ip_ver == IPv4) ? ipv4_flg_chk_fail : 1'b0;
            ipv4_frag_offset_chk_fail_r <= (ip_ver == IPv4) ? ipv4_frag_offset_chk_fail : 1'b0;
            ip_dest_addr_chk_fail_r     <= (ip_ver == IPv4) ? ipv4_dest_addr_chk_fail : ipv6_dest_addr_chk_fail;
            ipv4_hdr_chksum_fail_r      <= (ip_ver == IPv4) ? ipv4_hdr_chksum_fail : 1'b0;
            ip_length_chk_fail_r        <= (ip_ver == IPv4) ? ipv4_length_chk_fail : ipv6_length_chk_fail;
            udp_length_chk_fail_r       <= udp_length_chk_fail;
            pkt_len_chk_fail_r          <= pkt_len_chk_fail;
            bth_ver_chk_fail_r          <= bth_ver_chk_fail;
            dest_qp_chk_fail_r          <= dest_qp_chk_fail;
            bth_pad_cnt_chk_fail_r      <= bth_pad_cnt_chk_fail;
            ip_ver_r                    <= ip_ver;
            bth_psn_r                   <= bth_psn;
            bth_opcode_r                <= bth_opcode;
            aeth_malformed_r            <= aeth_malformed;
            aeth_syndr_r                <= aeth_syndr;
            aeth_msn_r                  <= aeth_msn;
            pkt_id_r                    <= pkt_id;
            pkt_src_addr_offset_r       <= pkt_src_addr_offset;
            actual_pkt_len_r            <= actual_pkt_len;
            //i_last_rsp_opcode <= last_rsp_opcode_reg[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]];
            i_last_rsp_opcode           <= last_rsp_opcode_combo;
            //ecn_vect <= nxt_ecn_vect;
            ecn_vect_idx                <= nxt_ecn_vect_idx;
            ecn_update_flg              <= nxt_ecn_update_flg;
        end
    end

    // ECN (Explicit Congestion Notification) 处理
    // IP头部的ECN字段：
    //     00 – 非 ECN 能力传输，非 ECT
    //     10 – 支持 ECN 的传输，ECT(0)
    //     01 – 支持 ECN 的传输，ECT(1)
    //     11 – 遇到拥塞，CE。
    always @(*) begin
        //nxt_ecn_vect = {(C_NUM_QP){1'b0}};
        nxt_ecn_vect_idx   = {(C_QP_INDX_WIDTH) {1'b0}};
        nxt_ecn_update_flg = 1'b0;
        if (ip_ver == IPv4) begin
            if (hdr_data[(15*8)+:2] == 2'h3) begin  // bits1~0(ECT & CE)为ECN字段
                //nxt_ecn_vect[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]] = 1'b1;
                nxt_ecn_vect_idx   = bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];
                nxt_ecn_update_flg = 1'b1;
            end
        end else if (hdr_data[(15*8+4)+:2] == 2'h3) begin
            //nxt_ecn_vect[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]] = 1'b1;
            nxt_ecn_vect_idx   = bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];
            nxt_ecn_update_flg = 1'b1;
        end
    end

    always @(posedge clk) begin
        if (reset) bth_qp_dest_addr_r <= 24'h00_0000;
        else if (hv_data_vld_r) bth_qp_dest_addr_r <= bth_qp_dest_addr;
    end

    //assign qp_state_fatal = (!i_qp_conf[0] || i_qp_fatal[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]]) ? 1'b1 : 1'b0;  
    assign qp_state_fatal = (!i_qp_conf[0] || qp_fatal_combo || qp_recovery_combo) ? 1'b1 : 1'b0;
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP FATAL LUTRAM START //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_fatal_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (init_start || i_qp_fatal_wr_en),
        .wr_addr (init_start ? init_count : i_qp_fatal_wr_idx),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]),
        .rd_data (qp_fatal_combo)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_recovery_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (init_start || (qp_under_recovery_wr_en && !qp_under_recovery_wr_en_r)),  //qp_under_recovery is a pulse only
        .wr_addr (init_start ? init_count : qp_under_recovery_wr_idx),
        .wr_data (init_start ? 1'b0 : qp_under_recovery_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]),
        .rd_data (qp_recovery_combo)
    );

    always @(posedge clk) qp_under_recovery_wr_en_r <= qp_under_recovery_wr_en;
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP FATAL LUTRAM END ////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////


    always @(*)
        case (i_qp_conf[10:8])
            3'b000:  qp_pmtu_decoded = 13'd0256;
            3'b001:  qp_pmtu_decoded = 13'd0512;
            3'b010:  qp_pmtu_decoded = 13'd1024;
            3'b011:  qp_pmtu_decoded = 13'd2048;
            3'b100:  qp_pmtu_decoded = 13'd4096;
            default: qp_pmtu_decoded = 13'd0256;
        endcase

    assign i_last_ob_req_psn = i_sq_psn - 'd1;
    assign i_rnr_nak_cnt     = i_stat_nak[9:7];  // 当前的 RNR 计数（读/写），用于接收应答数据包

    // reg qp_conf_vld_r;
    always @(posedge clk)
        if (reset) begin
            qp_pmtu_decoded_r       <= 'd0;
            qp_state_fatal_r        <= 'd0;
            ip_src_addr_chk_fail_r  <= 'd0;
            mac_src_addr_chk_fail_r <= 'd0;
            stat_resp_psn_r         <= 'd0;
            last_ob_req_psn_r       <= 'd0;
            rnr_nak_cnt_r           <= 'd0;
            o_qp_timeout_per_qp     <= 'd0;
            rnr_nak_rst_val_r       <= 'd0;
            unack_psn_min           <= 'd0;
        end else begin
            // qp_conf_vld_r <= qp_conf_vld;
            if (qp_conf_vld) begin
                qp_pmtu_decoded_r       <= qp_pmtu_decoded;
                qp_state_fatal_r        <= qp_state_fatal;
                ip_src_addr_chk_fail_r  <= ip_src_addr_chk_fail;
                mac_src_addr_chk_fail_r <= mac_src_addr_chk_fail;
                stat_resp_psn_r         <= i_stat_resp_psn;
                unack_psn_min           <= i_stat_resp_psn + 'd1;
                last_ob_req_psn_r       <= i_last_ob_req_psn;
                rnr_nak_cnt_r           <= i_rnr_nak_cnt;
                o_qp_timeout_per_qp     <= i_qp_timeout;
                rnr_nak_rst_val_r       <= i_rnr_nak_rst_val;
            end
        end

    // 错误标志信息结构体: 对应手册 Packet Validation Error Syndrome
    assign err_syndr = {
        pkt_fcs_err,  // bit 31
        pkt_crc_err,  // bit 30
        1'b0,  // bit 29
        mac_src_addr_chk_fail_r,  // bit 28
        ip_src_addr_chk_fail_r,  // bit 27
        3'b0,  // bit 24-26
        aeth_malformed_r,  // bit 23
        4'b0,  // bit 19-22
        bth_pad_cnt_chk_fail_r,  // bit 18
        2'b0,  // bit 16-17
        qp_state_fatal_r,  // bit 15
        dest_qp_chk_fail_r,  // bit 14
        bth_ver_chk_fail_r,  // bit 13
        pkt_len_chk_fail_r,  // bit 12
        udp_length_chk_fail_r,  // bit 11
        ip_length_chk_fail_r,  // bit 10
        ipv4_hdr_chksum_fail_r,  // bit 9
        ip_dest_addr_chk_fail_r,  // bit 8
        1'b0,  // bit 7
        ipv4_frag_offset_chk_fail_r,  // bit 6
        ipv4_flg_chk_fail_r,  // bit 5
        ip_ecn_chk_fail_r,  // bit 4
        ipv4_ihl_chk_fail_r,  // bit 3
        ip_ver_chk_fail_r,  // bit 2
        1'b0,  // bit 1 
        mac_dest_addr_chk_fail_r  // bit 0
    };

    assign unack_psn_min_muxed = stat_resp_psn_r + 'd1;
    assign unack_psn_max_muxed = last_ob_req_psn_r;
    //assign unack_psn_min         = stat_resp_psn_r + 'd1;
    assign unack_psn_max = last_ob_req_psn_r;
    assign stat_resp_psn_muxed = stat_resp_psn_r;
    assign rnr_nak_cnt_muxed = rnr_nak_cnt_r;
    assign qp_pmtu_decoded_muxed = qp_pmtu_decoded_r;

    always @(posedge clk) begin
        if (reset) begin
            i_os_rd_req_psn <= 'd0;
        end else if (i_last_rsp_opcode) begin
            if (i_os_rd_req_psn_7_0 <= stat_resp_psn_muxed[C_OSQ_PSN_WIDTH-1:0]) i_os_rd_req_psn <= {stat_resp_psn_muxed[23:C_OSQ_PSN_WIDTH], i_os_rd_req_psn_7_0};
            else i_os_rd_req_psn <= {stat_resp_psn_muxed[23:C_OSQ_PSN_WIDTH], i_os_rd_req_psn_7_0} - {{(23 - C_OSQ_PSN_WIDTH) {1'b0}}, 1'b1, {(C_OSQ_PSN_WIDTH) {1'b0}}};
        end else begin
            if (i_os_rd_req_psn_7_0 <= stat_resp_psn_muxed[C_OSQ_PSN_WIDTH-1:0]) i_os_rd_req_psn <= {stat_resp_psn_muxed[23:C_OSQ_PSN_WIDTH], i_os_rd_req_psn_7_0} + {{(23 - C_OSQ_PSN_WIDTH) {1'b0}}, 1'b1, {(C_OSQ_PSN_WIDTH) {1'b0}}};
            else i_os_rd_req_psn <= {stat_resp_psn_muxed[23:C_OSQ_PSN_WIDTH], i_os_rd_req_psn_7_0};
        end
    end

    //assign i_osq_invld = i_osq_empty[bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]] |
    //                     i_osq_nacked[bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]];
    wire                         osq_nacked_set_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_set_id;
    wire                         osq_nacked_set_valid_pending;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_set_id_pending;
    wire                         osq_nacked_reset_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_reset_id;
    wire                         osq_nacked_reset_valid_pending;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_reset_id_pending;

    assign {osq_nacked_set_valid, osq_nacked_set_id, osq_nacked_set_valid_pending, osq_nacked_set_id_pending, osq_nacked_reset_valid, osq_nacked_reset_id, osq_nacked_reset_valid_pending, osq_nacked_reset_id_pending} = i_osq_nacked;

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_nacked_1 (
        .core_clk   (clk),
        .core_rst   (reset),
        .wr_en      (init_start || osq_nacked_set_valid),
        .wr_addr    (init_start ? init_count : osq_nacked_set_id),
        .wr_data    (init_start ? 1'b0 : 1'b1),
        .wr_en_aux  (osq_nacked_reset_valid),
        .wr_addr_aux(osq_nacked_reset_id),
        .wr_data_aux(1'b0),
        .rd_en      (1'b1),
        .rd_addr    (bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]),
        .rd_data    (osq_nacked_combo_int)
    );
    always @(*) begin
        if (osq_nacked_set_valid_pending && osq_nacked_set_id_pending == bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]) begin
            osq_nacked_combo = 1'b1;
        end else if (osq_nacked_reset_valid_pending && osq_nacked_reset_id_pending == bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]) begin
            osq_nacked_combo = 1'b0;
        end else begin
            osq_nacked_combo = osq_nacked_combo_int;
        end
    end
    assign i_osq_invld = i_osq_empty_combo | osq_nacked_combo;

    //assign i_qp_retried_combo = i_qp_retried[bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]];                     

    always @(posedge clk)
        if (mpsnbuf_vld) begin
            os_rd_req_dest_addr <= i_os_rd_req_dest_addr;
            os_rd_req_resp_len  <= i_os_rd_req_resp_len;
            //mpsnbuf_empty_r     <= i_mpsnbuf_empty;
        end

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// MPSNBUF EMPTY LUTRAM START /////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    reg                        mpsnbuf_empty_reset_r;
    reg  [C_QP_INDX_WIDTH-1:0] mpsnbuf_empty_reset_id_r;
    reg                        mpsnbuf_empty_set_r;
    reg  [C_QP_INDX_WIDTH-1:0] mpsnbuf_empty_set_id_r;
    wire                       mpsnbuf_empry_wr_en;
    wire [C_QP_INDX_WIDTH-1:0] mpsnbuf_empty_wr_idx;
    wire                       mpsnbuf_empty_wr_data;

    assign mpsnbuf_empty_wr_en   = init_start || mpsnbuf_empty_set_r || mpsnbuf_empty_reset_r;
    assign mpsnbuf_empty_wr_idx  = init_start ? init_count : (mpsnbuf_empty_set_r ? mpsnbuf_empty_set_id_r : mpsnbuf_empty_reset_id_r);
    assign mpsnbuf_empty_wr_data = (init_start || mpsnbuf_empty_set_r) ? 1'b1 : 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            i_mpsnbuf_empty_r        <= 1'b0;
            i_mpsnbuf_empty_2r       <= 1'b0;
            mpsnbuf_empty_reset_r    <= 1'b0;
            mpsnbuf_empty_reset_id_r <= 'd0;
            mpsnbuf_empty_set_r      <= 1'b0;
            mpsnbuf_empty_set_id_r   <= 'd0;
        end else begin
            i_mpsnbuf_empty_r      <= i_mpsnbuf_empty_combo;
            i_mpsnbuf_empty_2r     <= i_mpsnbuf_empty_r;
            mpsnbuf_empty_set_r    <= i_mpsnbuf_empty_set_valid;
            mpsnbuf_empty_set_id_r <= i_mpsnbuf_empty_set_id;
            if (~(mpsnbuf_empty_reset_r && mpsnbuf_empty_set_r)) begin
                mpsnbuf_empty_reset_r    <= i_mpsnbuf_empty_reset_valid;
                mpsnbuf_empty_reset_id_r <= i_mpsnbuf_empty_reset_id;
            end
        end
    end
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_mpsnbuf_empty_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (mpsnbuf_empty_wr_en),
        .wr_addr (mpsnbuf_empty_wr_idx),
        .wr_data (mpsnbuf_empty_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]),
        .rd_data (i_mpsnbuf_empty_combo)
    );


    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// MPSNBUF EMPTY LUTRAM END ///////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////




    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// OSQ EMPTY LUTRAM START /////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    reg                        osq_empty_reset_r;
    reg  [C_QP_INDX_WIDTH-1:0] osq_empty_reset_id_r;
    reg                        osq_empty_set_r;
    reg  [C_QP_INDX_WIDTH-1:0] osq_empty_set_id_r;
    wire                       osq_empry_wr_en;
    wire [C_QP_INDX_WIDTH-1:0] osq_empty_wr_idx;
    wire                       osq_empty_wr_data;

    assign osq_empty_wr_en   = init_start || osq_empty_set_r || osq_empty_reset_r;
    assign osq_empty_wr_idx  = init_start ? init_count : (osq_empty_set_r ? osq_empty_set_id_r : osq_empty_reset_id_r);
    assign osq_empty_wr_data = (init_start || osq_empty_set_r) ? 1'b1 : 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            osq_empty_reset_r    <= 1'b0;
            osq_empty_reset_id_r <= 'd0;
            osq_empty_set_r      <= 1'b0;
            osq_empty_set_id_r   <= 'd0;
        end else begin
            osq_empty_set_r    <= i_osq_empty_set_valid;
            osq_empty_set_id_r <= i_osq_empty_set_id;
            if (~(osq_empty_reset_r && osq_empty_set_r)) begin
                osq_empty_reset_r    <= i_osq_empty_reset_valid;
                osq_empty_reset_id_r <= i_osq_empty_reset_id;
            end
        end
    end
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (osq_empty_wr_en),
        .wr_addr (osq_empty_wr_idx),
        .wr_data (osq_empty_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]),
        .rd_data (i_osq_empty_combo)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// OSQ EMPTY LUTRAM END ///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////



    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP RETRIED LUTRAM START ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    reg                        qp_retried_reset_r;
    reg  [C_QP_INDX_WIDTH-1:0] qp_retried_reset_id_r;
    reg                        qp_retried_set_r;
    reg  [C_QP_INDX_WIDTH-1:0] qp_retried_set_id_r;
    wire                       qp_retried_wr_en;
    wire [C_QP_INDX_WIDTH-1:0] qp_retried_wr_idx;
    wire                       qp_retried_wr_data;

    assign qp_retried_wr_en   = init_start || qp_retried_set_r || qp_retried_reset_r;
    assign qp_retried_wr_idx  = init_start ? init_count : (qp_retried_set_r ? qp_retried_set_id_r : qp_retried_reset_id_r);
    assign qp_retried_wr_data = qp_retried_set_r ? 1'b1 : 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            qp_retried_reset_r    <= 1'b0;
            qp_retried_reset_id_r <= 'd0;
            qp_retried_set_r      <= 1'b0;
            qp_retried_set_id_r   <= 'd0;
        end else begin
            qp_retried_set_r    <= i_qp_retried_set_valid;
            qp_retried_set_id_r <= i_qp_retried_set_id;
            if (~(qp_retried_reset_r && qp_retried_set_r)) begin
                qp_retried_reset_r    <= i_qp_retried_reset_valid;
                qp_retried_reset_id_r <= i_qp_retried_reset_id;
            end
        end
    end
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_retried_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (qp_retried_wr_en),
        .wr_addr (qp_retried_wr_idx),
        .wr_data (qp_retried_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]),
        .rd_data (i_qp_retried_combo)
    );
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP RETRIED LUTRAM END //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    //assign i_mpsnbuf_empty_combo = mpsnbuf_empty_r[bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]];

    always @(posedge clk)
        if (mpsnbuf_vld_r) begin
            osq_invld_at_vld                  <= i_osq_invld;
            is_rsp_in_unack_range_at_vld      <= is_in_range(unack_psn_min_muxed, unack_psn_max_muxed, bth_psn_r);
            is_rd_req_in_unack_range_at_vld   <= is_in_range(i_os_rd_req_psn, unack_psn_max_muxed, bth_psn_r);
            os_rd_req_psn                     <= i_os_rd_req_psn;
            mpsnbuf_empty_at_vld              <= i_mpsnbuf_empty_r || i_mpsnbuf_empty_2r;
            qp_retried                        <= i_qp_retried_combo;
            ack_psn_less_os_rd                <= is_x_less_y(bth_psn_r, i_os_rd_req_psn);
            ack_psn_lessorequal_os_rd         <= is_x_lessorequal_y(bth_psn_r, i_os_rd_req_psn);
            rdrsp_psn_grtr_os_rd              <= is_x_grtr_y(bth_psn_r, i_os_rd_req_psn);
            rdrsp_psn_lessorequal_last_ob_req <= is_x_lessorequal_y(bth_psn_r, unack_psn_max);
        end


    assign mpsnbuf_empty            = mpsnbuf_empty_at_vld;
    assign osq_invld                = osq_invld_at_vld;
    assign is_rsp_in_unack_range    = is_rsp_in_unack_range_at_vld & ~osq_invld & ~(i_last_rsp_opcode & mpsnbuf_empty);
    assign is_rd_req_in_unack_range = is_rd_req_in_unack_range_at_vld & ~osq_invld & ~mpsnbuf_empty;

    // 响应包类型检测
    always @(posedge clk)
        if (hv_data_vld_r) begin
            case (bth_opcode[4:0])
                RD_RSP_FIRST: pkt_type <= PKT_TYPE_READ_FIRST;
                RD_RSP_MIDDLE: pkt_type <= PKT_TYPE_READ_MIDDLE;
                RD_RSP_LAST: pkt_type <= PKT_TYPE_READ_LAST;
                RD_RSP_ONLY: pkt_type <= PKT_TYPE_READ_ONLY;
                ACKNOWLEDGE:
                case (aeth_syndr[6:5])
                    // ACK
                    2'b00: pkt_type <= PKT_TYPE_ACKNOWLEDGE_ACK;

                    // RNR NAK
                    2'b01: pkt_type <= PKT_TYPE_ACKNOWLEDGE_RNR_NAK;

                    // NAK
                    2'b11: pkt_type <= (aeth_syndr[4:0] == 5'd0) ? PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR : PKT_TYPE_ACKNOWLEDGE_NAK_FATAL;

                    // 不是上述任何一种
                    default: pkt_type <= PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR;
                endcase
                default: pkt_type <= 3'dx;
            endcase
        end

    // 响应包长度检查
    always @(posedge clk)
        if (mpsnbuf_vld_r) begin
            rdrsp_first_len_chk_fail  <= ((bth_pyld_len != (qp_pmtu_decoded_muxed + 'd4)) || (os_rd_req_resp_len <= {19'd0, qp_pmtu_decoded_muxed})) ? 1'b1 : 1'b0;
            rdrsp_middle_len_chk_fail <= ((bth_pyld_len != qp_pmtu_decoded_muxed) || (i_rdrsp_len_remain <= qp_pmtu_decoded_muxed)) ? 1'b1 : 1'b0;
            rdrsp_last_len_chk_fail   <= ((bth_pyld_len < 'd5) || (bth_pyld_len > (qp_pmtu_decoded_muxed + 'd4)) || (bth_pyld_len != (i_rdrsp_len_remain + 'd4))) ? 1'b1 : 1'b0;
            rdrsp_only_len_chk_fail   <= ((bth_pyld_len != (os_rd_req_resp_len + 'd4)) || (os_rd_req_resp_len > {19'd0, qp_pmtu_decoded_muxed})) ? 1'b1 : 1'b0;
        end

    always @(*) begin
        case (pkt_type)
            PKT_TYPE_READ_FIRST: acknack_opcode = 2'b01;
            PKT_TYPE_READ_MIDDLE: acknack_opcode = 2'b10;
            PKT_TYPE_READ_LAST, PKT_TYPE_READ_ONLY: acknack_opcode = 2'b11;
            default: acknack_opcode = 2'b00;
        endcase
    end

    assign o_stat_nak_int = {o_rnr_nak_cnt, aeth_syndr_r[6:0]};

    assign o_qp_fatal_idx = bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0];

    //assign o_qp_acknack_idx = bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0];
    /*                                         
  assign o_acknack_vld = acknack_vld;
  assign o_acknack_psn = acknack_psn;
  assign o_acknack_syndr = acknack_syndr;
*/
    assign o_acknack_msn  = aeth_msn_r;
    /*                                             
  assign o_acknack_opcode = acknack_opcode;
*/
    always @(posedge clk)
        if (reset) o_acknack_vld <= 1'b0;
        else o_acknack_vld <= acknack_vld;

    always @(posedge clk)
        if (acknack_vld) begin
            o_acknack_psn    <= acknack_psn;
            o_acknack_syndr  <= acknack_syndr;
            o_qp_acknack_idx <= bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0];
        end

    always @(posedge clk)
        if (reset) o_acknack_opcode <= 2'b00;
        else if (acknack_vld) o_acknack_opcode <= acknack_opcode;

    always @(posedge clk)
        if (reset) o_mpsnbuf_pop <= 1'b0;
        else o_mpsnbuf_pop <= mpsnbuf_pop;


    /*   
  assign rdrsp_len_remain = ((pkt_type == PKT_TYPE_READ_FIRST) || (pkt_type == PKT_TYPE_READ_ONLY)) ?
                           os_rd_req_resp_len : i_rdrsp_len_remain;
  assign o_rdrsp_len_remain = rdrsp_len_remain - qp_pmtu_decoded_r;
*/
    assign rdrsp_len_remain = ((pkt_type == PKT_TYPE_READ_FIRST) || (pkt_type == PKT_TYPE_READ_ONLY)) ? os_rd_req_resp_len : i_rdrsp_len_remain;

    always @(posedge clk) if (mpsnbuf_vld_r) o_rdrsp_len_remain <= rdrsp_len_remain - qp_pmtu_decoded_r;

    assign rsp_buf_addr_offset = pkt_src_addr_offset_r;
    //assign rsp_pkt_len = actual_pkt_len_r; 
    assign rsp_pkt_id          = pkt_id_r;

    xpm_memory_spram #(  // Common module parameters
        .MEMORY_SIZE       (C_NUM_QP * C_M_AXI_ADDR_WIDTH),  // positive integer
        .MEMORY_PRIMITIVE  ("auto"),                         // string; "auto", "distributed", "block" or "ultra";
        .ECC_MODE          ("no_ecc"),                       // do not change
        .MEMORY_INIT_FILE  ("none"),                         // string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM (""),                             // string;  
        .WAKEUP_TIME       ("disable_sleep"),                // string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL   (0),
        // Port A module parameters
        .WRITE_DATA_WIDTH_A(C_M_AXI_ADDR_WIDTH),             // positive integer
        .READ_DATA_WIDTH_A (C_M_AXI_ADDR_WIDTH),             // positive integer
        .BYTE_WRITE_WIDTH_A(C_M_AXI_ADDR_WIDTH),             // integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),                // positive integer
        .READ_RESET_VALUE_A("0"),                            // string
        .READ_LATENCY_A    (1),                              // non-negative integer
        .WRITE_MODE_A      ("write_first")                   // string; "write_first", "read_first", "no_change"   
    ) xpm_memory_rdrsp_dest_addr (
        // Common module ports
        .sleep         (1'b0),                                      //do not change  
        // Port A module ports
        .clka          (clk),
        .rsta          (1'b0),
        .ena           (rdrsp_dest_addr_en),
        .regcea        (1'b1),
        .wea           (rdrsp_dest_addr_we),
        .addra         (rdrsp_dest_addr_idx[C_QP_INDX_WIDTH-1:0]),
        .dina          (o_rdrsp_dest_addr),
        .injectsbiterra(1'b0),                                      //do not change
        .injectdbiterra(1'b0),                                      //do not change
        .douta         (i_rdrsp_dest_addr),
        .sbiterra      (),                                          //do not change
        .dbiterra      ()                                           //do not change  
    );

    xpm_memory_spram #(  // Common module parameters
        .MEMORY_SIZE       (C_NUM_QP * 32),    // positive integer
        .MEMORY_PRIMITIVE  ("auto"),           // string; "auto", "distributed", "block" or "ultra";
        .ECC_MODE          ("no_ecc"),         // do not change
        .MEMORY_INIT_FILE  ("none"),           // string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM (""),               // string;  
        .WAKEUP_TIME       ("disable_sleep"),  // string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL   (0),
        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),               // positive integer
        .READ_DATA_WIDTH_A (32),               // positive integer
        .BYTE_WRITE_WIDTH_A(32),               // integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  // positive integer
        .READ_RESET_VALUE_A("0"),              // string
        .READ_LATENCY_A    (1),                // non-negative integer
        .WRITE_MODE_A      ("write_first")     // string; "write_first", "read_first", "no_change"   
    ) xpm_memory_rdrsp_len_remain (
        // Common module ports
        .sleep         (1'b0),                                       //do not change
        // Port A module ports
        .clka          (clk),
        .rsta          (1'b0),
        .ena           (rdrsp_len_remain_en),
        .regcea        (1'b1),
        .wea           (rdrsp_len_remain_we),
        .addra         (rdrsp_len_remain_idx[C_QP_INDX_WIDTH-1:0]),
        .dina          (o_rdrsp_len_remain),
        .injectsbiterra(1'b0),                                       //do not change
        .injectdbiterra(1'b0),                                       //do not change
        .douta         (i_rdrsp_len_remain),
        .sbiterra      (),                                           //do not change
        .dbiterra      ()                                            //do not change
    );

    //always @(posedge clk)
    //  if (reset)
    //    last_rsp_opcode_reg <= {(C_NUM_QP-1) {1'b0}};
    //  else if (last_rsp_opcode_we)
    //    last_rsp_opcode_reg[bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]] <= o_last_rsp_opcode;
    //  else if (req_qp_fatal_flag && clr_qp_fatal_flag)
    //    last_rsp_opcode_reg[req_qp_fatal_idx_r] <= 1'b0;

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// last_rsp_opcode_reg LUTRAM START ///////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    reg                        last_resp_opcode_reset_r;
    reg  [C_QP_INDX_WIDTH-1:0] last_resp_opcode_reset_id_r;
    reg                        last_resp_opcode_set_r;
    reg  [C_QP_INDX_WIDTH-1:0] last_resp_opcode_set_id_r;
    reg                        last_resp_opcode_r;
    wire                       last_resp_opcode_wr_en;
    wire [C_QP_INDX_WIDTH-1:0] last_resp_opcode_wr_idx;
    wire                       last_resp_opcode_wr_data;

    assign last_resp_opcode_wr_en   = init_start || last_resp_opcode_set_r || last_resp_opcode_reset_r;
    assign last_resp_opcode_wr_idx  = init_start ? init_count : (last_resp_opcode_set_r ? last_resp_opcode_set_id_r : last_resp_opcode_reset_id_r);
    assign last_resp_opcode_wr_data = last_resp_opcode_set_r ? last_resp_opcode_r : 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            last_resp_opcode_reset_r    <= 1'b0;
            last_resp_opcode_reset_id_r <= 'd0;
            last_resp_opcode_set_r      <= 1'b0;
            last_resp_opcode_r          <= 1'b0;
            last_resp_opcode_set_id_r   <= 'd0;
        end else begin
            last_resp_opcode_set_r    <= last_rsp_opcode_we;
            last_resp_opcode_r        <= o_last_rsp_opcode;
            last_resp_opcode_set_id_r <= bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0];
            if (~(last_resp_opcode_reset_r && last_resp_opcode_set_r)) begin
                last_resp_opcode_reset_r    <= (req_qp_fatal_flag && clr_qp_fatal_flag);
                last_resp_opcode_reset_id_r <= req_qp_fatal_idx_r;
            end
        end
    end

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_last_rsp_opcode_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (last_resp_opcode_wr_en),
        .wr_addr (last_resp_opcode_wr_idx),
        .wr_data (last_resp_opcode_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]),
        .rd_data (last_rsp_opcode_combo)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// last_rsp_opcode_reg LUTRAM END /////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    always @(posedge clk) in_err_sts_code <= nxt_in_err_sts_code;

    always @(posedge clk)
        if (rsp_tnfr_vld) begin
            rsp_pkt_type_r  <= rsp_pkt_type;
            rsp_dest_addr_r <= rsp_dest_addr;
            rsp_imm_data_r  <= rsp_imm_data;
        end

    //===============================
    // 头部验证 FSM
    //===============================  
    always @(*) begin
        // default values
        hv_fsm_busy             = 1'b0;
        hv_fsm_ns               = hv_fsm_cs;
        nxt_in_err_sts_code     = in_err_sts_code;
        o_rnr_nak_cnt           = rnr_nak_cnt_r;
        stat_nak_en_int         = 1'b0;
        stat_nak_we_int         = 1'b0;
        o_stat_resp_psn_int     = bth_psn_r;
        stat_resp_psn_en_int    = 1'b0;
        stat_resp_psn_we_int    = 1'b0;
        o_last_rsp_opcode       = i_last_rsp_opcode;
        last_rsp_opcode_we      = 1'b0;
        mpsnbuf_pop             = 1'b0;
        buf_en_int              = 1'b0;
        rdrsp_len_remain_en_int = 1'b0;
        rdrsp_len_remain_we_int = 1'b0;
        rdrsp_dest_addr_en_int  = 1'b0;
        rdrsp_dest_addr_we_int  = 1'b0;
        rsp_pkt_type            = RSP_AXI_TNFR_TYPE_DISCARD;
        rsp_dest_addr           = i_rdrsp_dest_addr;
        rsp_imm_data            = 32'd0;
        rsp_tnfr_vld            = 1'b0;
        req_tnfr_vld            = 1'b0;
        acknack_psn             = bth_psn_r;
        acknack_syndr           = aeth_syndr_r;
        acknack_vld             = 1'b0;
        incr_acknack_pkt_cnt    = 1'b0;
        incr_inc_nack_pkt_cnt   = 1'b0;
        set_qp_fatal            = 1'b0;
        clr_qp_fatal_flag       = 1'b0;
        ecn_vect_vld            = 1'b0;
        rsp_pkt_len             = actual_pkt_len_r;

        case (hv_fsm_cs)
            3'd0: begin
                nxt_in_err_sts_code = 5'd0;

                clr_qp_fatal_flag   = 1'b1;

                if (hv_data_vld) begin
                    stat_nak_en_int         = 1'b1;  // 获取 {STATQPi[30:28], STATQPi[22:16]}
                    stat_resp_psn_en_int    = 1'b1;  // 获取最后接受的响应 PSN, 即 STATRESPSNi[23:0]: 预期响应的 PSN
                    buf_en_int              = 1'b1;  // 获取对应信息
                    rdrsp_len_remain_en_int = 1'b1;  // 获取当前读响应的剩余长度
                    rdrsp_dest_addr_en_int  = 1'b1;  // 获取当前读响应的目标地址

                    hv_fsm_busy             = 1'b1;
                    hv_fsm_ns               = 3'd1;
                end  // if (hv_data_vld)        
            end  // 3'd0

            3'd1: begin
                hv_fsm_busy = 1'b1;
                if (qp_conf_vld) begin
                    // 为便于流水作业, mpsnbuf_vld 至少应在 qp_conf_vld 之后 3 个时钟
                    if (mpsnbuf_vld_r) begin
                        hv_fsm_ns = 3'd3;
                    end else hv_fsm_ns = 3'd2;
                end  // if (qp_conf_vld)        
            end  // 3'd1

            3'd2: begin
                hv_fsm_busy = 1'b1;
                if (mpsnbuf_vld_r) begin
                    if (err_syndr != 32'd0) begin  // 有错误, 进入错误处理
                        hv_fsm_ns = 3'd6;
                    end else begin  // 没有错误, 继续处理
                        hv_fsm_ns = 3'd3;
                    end
                    if ((err_syndr != 32'd0) && ecn_update_flg) ecn_vect_vld = 1'b1;

                    if ((bth_opcode == ACKNOWLEDGE) && !aeth_malformed) incr_acknack_pkt_cnt = 1'b1;  // ACK&NACK 包计数器增加

                    // aeth_syndr[6:5] != 2'b00 表明包为 NACK 和 RNR-NACK
                    if ((bth_opcode == ACKNOWLEDGE) && (aeth_syndr[6:5] != 2'b00) && !aeth_malformed) incr_inc_nack_pkt_cnt = 1'b1;  // NAK 包计数器增加

                end
            end  // 3'd2

            3'd3: begin  // err_syndr == 32'd0: 没有头验证错误
                case (pkt_type)
                    PKT_TYPE_ACKNOWLEDGE_ACK: begin
                        hv_fsm_ns = 3'd0;

                        if (is_rsp_in_unack_range) begin
                            if (i_last_rsp_opcode) begin
                                if (bth_psn_r == unack_psn_min) begin
                                    rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data = 32'h0040_0000;
                                    rsp_tnfr_vld = 1'b1;
                                    hv_fsm_busy  = 1'b1;
                                    if (rdy_to_rsp_tnfr) begin
                                        nxt_in_err_sts_code = 5'b01_100;
                                        hv_fsm_ns           = 3'd4;
                                    end
                                end else if (!qp_retried) begin
                                    //acknack_psn = os_rd_req_psn;
                                    acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;
                                    // hv_fsm_ns = 3'd6;    // ***
                                    // hv_fsm_busy = 1'b1;  // ***
                                    hv_fsm_ns     = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data  = 32'h2040_0000;
                                    rsp_pkt_len   = 13'h0;
                                    rsp_tnfr_vld  = 1'b1;
                                end
                            end // if (i_last_rsp_opcode)

                            else if ((bth_psn_r == os_rd_req_psn) && !mpsnbuf_empty) begin  // ACK-Fatal case (not full proof)
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h0040_0000;
                                rsp_tnfr_vld = 1'b1;
                                hv_fsm_busy  = 1'b1;
                                if (rdy_to_rsp_tnfr) begin
                                    nxt_in_err_sts_code = 5'b01_100;
                                    hv_fsm_ns           = 3'd4;
                                end
                            end // ACK-Fatal case

                            else if (mpsnbuf_empty || (!mpsnbuf_empty && ack_psn_less_os_rd)) begin
                                acknack_psn          = bth_psn_r;
                                acknack_syndr        = aeth_syndr_r;
                                acknack_vld          = 1'b1;

                                hv_fsm_ns            = 3'd0;

                                o_stat_resp_psn_int  = bth_psn_r;
                                stat_resp_psn_en_int = 1'b1;
                                stat_resp_psn_we_int = 1'b1;

                                o_rnr_nak_cnt        = rnr_nak_rst_val_r;
                                stat_nak_en_int      = 1'b1;
                                stat_nak_we_int      = 1'b1;

                                //To pop the frame data from rsp pkt buffer
                                rsp_pkt_type         = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data         = 32'h2040_0000;
                                rsp_pkt_len          = 13'h0;
                                rsp_tnfr_vld         = 1'b1;
                            end // mpsnbuf_empty、

                            else begin  // bth_psn > os_rd_req_psn
                                if (unack_psn_min != os_rd_req_psn) begin
                                    o_stat_resp_psn_int  = os_rd_req_psn - 'd1;
                                    stat_resp_psn_en_int = 1'b1;
                                    stat_resp_psn_we_int = 1'b1;

                                    o_rnr_nak_cnt        = rnr_nak_rst_val_r;
                                    stat_nak_en_int      = 1'b1;
                                    stat_nak_we_int      = 1'b1;

                                    //acknack_psn = os_rd_req_psn;
                                    acknack_psn          = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr        = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld          = 1'b1;

                                    hv_fsm_ns            = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type         = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data         = 32'h2040_0000;
                                    rsp_pkt_len          = 13'h0;
                                    rsp_tnfr_vld         = 1'b1;
                                end else if (!qp_retried) begin
                                    //acknack_psn = os_rd_req_psn;
                                    acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;

                                    hv_fsm_ns     = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data  = 32'h2040_0000;
                                    rsp_pkt_len   = 13'h0;
                                    rsp_tnfr_vld  = 1'b1;
                                end
                            end  // bth_psn > os_rd_req_psn
                        end  // is_rsp_in_unack_range

                        else begin
                            //To pop the frame data from rsp pkt buffer
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;
                        end
                    end  // PKT_TYPE_ACKNOWLEDGE_ACK

                    PKT_TYPE_ACKNOWLEDGE_RNR_NAK: begin
                        hv_fsm_ns = 3'd0;

                        if (i_last_rsp_opcode) begin
                            if (is_rd_req_in_unack_range) begin
                                if (bth_psn_r == unack_psn_min) begin  //Partial RD_REQ_RETRY //os_rd_req_psn)
                                    if (rnr_nak_cnt_r == 3'd0) begin  // RNR 下降计数器已过期
                                        rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                        rsp_imm_data = 32'h0040_0000;
                                        rsp_tnfr_vld = 1'b1;
                                        hv_fsm_busy  = 1'b1;
                                        if (rdy_to_rsp_tnfr) begin
                                            nxt_in_err_sts_code = 5'b01_001;
                                            hv_fsm_ns           = 3'd4;
                                        end
                                    end else begin  // rnr_nak_cnt_r > 3'd0
                                        acknack_psn   = bth_psn_r;
                                        acknack_syndr = aeth_syndr_r;
                                        acknack_vld   = 1'b1;

                                        hv_fsm_ns     = 3'd0;

                                        //To pop the frame data from rsp pkt buffer
                                        rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                        rsp_imm_data  = 32'h2040_0000;
                                        rsp_pkt_len   = 13'h0;
                                        rsp_tnfr_vld  = 1'b1;
                                        if (rnr_nak_cnt_r != 3'd7) o_rnr_nak_cnt = rnr_nak_cnt_r - 'd1;
                                        else o_rnr_nak_cnt = rnr_nak_rst_val_r;

                                        stat_nak_en_int = 1'b1;
                                        stat_nak_we_int = 1'b1;
                                    end  // rnr_nak_cnt_r > 3'd0
                                end  // bth_psn_r == os_rd_req_psn

                                else if (!qp_retried) begin //Covers the condition when the PSN is > OS READ REQ PSN
                                    //We cannot have a missed PSN for READ Response.
                                    acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;
                                    hv_fsm_ns     = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data  = 32'h2040_0000;
                                    rsp_pkt_len   = 13'h0;
                                    rsp_tnfr_vld  = 1'b1;
                                end
                            end  // is_rd_req_in_unack_range
                        end else begin  // !i_last_rsp_opcode
                            if (is_rsp_in_unack_range) begin
                                if (bth_psn_r == unack_psn_min) begin
                                    if (rnr_nak_cnt_r == 3'd0) begin
                                        rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                        rsp_imm_data = 32'h0040_0000;
                                        rsp_tnfr_vld = 1'b1;
                                        hv_fsm_busy  = 1'b1;
                                        if (rdy_to_rsp_tnfr) begin
                                            nxt_in_err_sts_code = 5'b01_001;
                                            hv_fsm_ns           = 3'd4;
                                        end
                                    end else begin  // rnr_nak_cnt_r > 3'd0
                                        acknack_psn   = bth_psn_r;
                                        acknack_syndr = aeth_syndr_r;
                                        acknack_vld   = 1'b1;
                                        // hv_fsm_ns = 3'd6;    // ***
                                        // hv_fsm_busy = 1'b1;  // ***
                                        hv_fsm_ns     = 3'd0;


                                        //To pop the frame data from rsp pkt buffer
                                        rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                        rsp_imm_data  = 32'h2040_0000;
                                        rsp_pkt_len   = 13'h0;
                                        rsp_tnfr_vld  = 1'b1;
                                        if (rnr_nak_cnt_r != 3'd7) o_rnr_nak_cnt = rnr_nak_cnt_r - 'd1;
                                        else o_rnr_nak_cnt = rnr_nak_rst_val_r;

                                        stat_nak_en_int = 1'b1;
                                        stat_nak_we_int = 1'b1;
                                    end  // rnr_nak_cnt_r > 3'd0
                                end  // bth_psn_r == unack_psn_min

                                else if (mpsnbuf_empty || (!mpsnbuf_empty && ack_psn_lessorequal_os_rd)) begin
                                    acknack_psn          = bth_psn_r;
                                    acknack_syndr        = aeth_syndr_r;
                                    acknack_vld          = 1'b1;
                                    // hv_fsm_ns = 3'd6;    // ***
                                    // hv_fsm_busy = 1'b1;  // ***
                                    hv_fsm_ns            = 3'd0;

                                    o_stat_resp_psn_int  = bth_psn_r - 'd1;
                                    stat_resp_psn_en_int = 1'b1;
                                    stat_resp_psn_we_int = 1'b1;

                                    o_rnr_nak_cnt        = rnr_nak_rst_val_r - 'd1;  // both implicit ACK and RNR-NAK
                                    stat_nak_en_int      = 1'b1;
                                    stat_nak_we_int      = 1'b1;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type         = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data         = 32'h2040_0000;
                                    rsp_pkt_len          = 13'h0;
                                    rsp_tnfr_vld         = 1'b1;
                                end else begin  // bth_psn > os_rd_req_psn
                                    o_stat_resp_psn_int  = os_rd_req_psn - 'd1;
                                    stat_resp_psn_en_int = 1'b1;
                                    stat_resp_psn_we_int = 1'b1;

                                    o_rnr_nak_cnt        = rnr_nak_rst_val_r;
                                    stat_nak_en_int      = 1'b1;
                                    stat_nak_we_int      = 1'b1;

                                    //acknack_psn = os_rd_req_psn;
                                    acknack_psn          = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr        = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld          = 1'b1;
                                    // hv_fsm_ns = 3'd6;    // ***
                                    // hv_fsm_busy = 1'b1;  // ***
                                    hv_fsm_ns            = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type         = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data         = 32'h2040_0000;
                                    rsp_pkt_len          = 13'h0;
                                    rsp_tnfr_vld         = 1'b1;
                                end  // bth_psn > os_rd_req_psn
                            end  // is_rsp_in_unack_range

                            else begin
                                //To pop the frame data from rsp pkt buffer
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                            end
                        end  // !i_last_rsp_opcode
                    end

                    //os_rd_req_psn : Oustanding read request PSN
                    PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR: begin
                        hv_fsm_ns = 3'd0;

                        if (is_rsp_in_unack_range) begin
                            if (i_last_rsp_opcode) begin
                                if (!qp_retried) begin
                                    acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;
                                    // hv_fsm_ns = 3'd6;    // ***
                                    // hv_fsm_busy = 1'b1;  // ***
                                    hv_fsm_ns     = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data  = 32'h2040_0000;
                                    rsp_pkt_len   = 13'h0;
                                    rsp_tnfr_vld  = 1'b1;
                                end
                            end else if (mpsnbuf_empty || (!mpsnbuf_empty && ack_psn_lessorequal_os_rd)) begin
                                acknack_psn          = bth_psn_r;
                                acknack_syndr        = aeth_syndr_r;
                                acknack_vld          = 1'b1;
                                // hv_fsm_ns = 3'd6;    // ***
                                // hv_fsm_busy = 1'b1;  // ***
                                hv_fsm_ns            = 3'd0;

                                o_stat_resp_psn_int  = bth_psn_r - 'd1;
                                stat_resp_psn_en_int = 1'b1;
                                stat_resp_psn_we_int = 1'b1;

                                o_rnr_nak_cnt        = rnr_nak_rst_val_r;
                                stat_nak_en_int      = 1'b1;
                                stat_nak_we_int      = 1'b1;

                                //To pop the frame data from rsp pkt buffer
                                rsp_pkt_type         = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data         = 32'h2040_0000;
                                rsp_pkt_len          = 13'h0;
                                rsp_tnfr_vld         = 1'b1;
                            end else begin  // bth_psn > os_rd_req_psn
                                if (unack_psn_min != os_rd_req_psn) begin
                                    o_stat_resp_psn_int  = os_rd_req_psn - 'd1;
                                    stat_resp_psn_en_int = 1'b1;
                                    stat_resp_psn_we_int = 1'b1;

                                    o_rnr_nak_cnt        = rnr_nak_rst_val_r;
                                    stat_nak_en_int      = 1'b1;
                                    stat_nak_we_int      = 1'b1;

                                    //acknack_psn = os_rd_req_psn;
                                    acknack_psn          = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr        = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld          = 1'b1;
                                    hv_fsm_ns            = 3'd6;
                                    hv_fsm_busy          = 1'b1;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type         = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data         = 32'h2040_0000;
                                    rsp_pkt_len          = 13'h0;
                                    rsp_tnfr_vld         = 1'b1;
                                end else if (!qp_retried) begin
                                    acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;
                                    // hv_fsm_ns = 3'd6;    // ***
                                    // hv_fsm_busy = 1'b1;  // ***
                                    hv_fsm_ns     = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data  = 32'h2040_0000;
                                    rsp_pkt_len   = 13'h0;
                                    rsp_tnfr_vld  = 1'b1;
                                end
                            end  // bth_psn > os_rd_req_psn
                        end  // is_rsp_in_unack_range

                        else begin
                            //To pop the frame data from rsp pkt buffer
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;
                        end
                    end  // PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR

                    PKT_TYPE_ACKNOWLEDGE_NAK_FATAL: begin
                        if (is_rsp_in_unack_range) begin
                            if (i_last_rsp_opcode) begin
                                if (bth_psn_r == unack_psn_min) begin
                                    rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data = 32'h0040_0000;
                                    rsp_tnfr_vld = 1'b1;
                                    hv_fsm_busy  = 1'b1;
                                    if (rdy_to_rsp_tnfr) begin
                                        nxt_in_err_sts_code = 5'b01_010;
                                        hv_fsm_ns           = 3'd4;
                                    end
                                end else if (!qp_retried) begin
                                    //acknack_psn = os_rd_req_psn;
                                    acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;
                                    // hv_fsm_ns = 3'd6;    // ***
                                    // hv_fsm_busy = 1'b1;  // ***
                                    hv_fsm_ns     = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data  = 32'h2040_0000;
                                    rsp_pkt_len   = 13'h0;
                                    rsp_tnfr_vld  = 1'b1;
                                end else begin
                                    hv_fsm_ns = 3'd0;
                                end
                            end else if (mpsnbuf_empty || (!mpsnbuf_empty && ack_psn_lessorequal_os_rd)) begin  //TBD to check if (mpsnbuf_empty) is covered in code coverage. second statement in the or condition will be covered
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h0040_0000;
                                rsp_tnfr_vld = 1'b1;
                                hv_fsm_busy  = 1'b1;
                                if (rdy_to_rsp_tnfr) begin
                                    nxt_in_err_sts_code = 5'b01_010;
                                    hv_fsm_ns           = 3'd4;
                                end
                            end else begin
                                if (mpsnbuf_empty || (!mpsnbuf_empty && ack_psn_lessorequal_os_rd)) begin
                                    rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data = 32'h0040_0000;
                                    rsp_tnfr_vld = 1'b1;
                                    hv_fsm_busy  = 1'b1;
                                    if (rdy_to_rsp_tnfr) begin
                                        nxt_in_err_sts_code = 5'b01_010;
                                        hv_fsm_ns           = 3'd4;
                                    end
                                end else if (!qp_retried) begin
                                    acknack_psn   = os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;
                                    // hv_fsm_ns = 3'd6;    // ***
                                    // hv_fsm_busy = 1'b1;  // ***
                                    hv_fsm_ns     = 3'd0;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data  = 32'h2040_0000;
                                    rsp_pkt_len   = 13'h0;
                                    rsp_tnfr_vld  = 1'b1;

                                    //To pop the frame data from rsp pkt buffer
                                    rsp_pkt_type  = RSP_AXI_TNFR_TYPE_ERROR;
                                    rsp_imm_data  = 32'h2040_0000;
                                    rsp_pkt_len   = 13'h0;
                                    rsp_tnfr_vld  = 1'b1;
                                end else begin
                                    hv_fsm_ns = 3'd0;
                                end
                            end
                        end  // is_rsp_in_unack_range

                        else begin
                            //To pop the frame data from rsp pkt buffer
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;

                            hv_fsm_ns    = 3'd0;
                        end
                    end  // PKT_TYPE_ACKNOWLEDGE_NAK_FATAL

                    PKT_TYPE_READ_ONLY: begin
                        if (!is_rsp_in_unack_range || mpsnbuf_empty) begin
                            //To pop the frame data from rsp pkt buffer
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;

                            hv_fsm_ns    = 3'd0;
                        end else if (i_last_rsp_opcode) begin
                            if (bth_psn_r == unack_psn_min) begin
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                                hv_fsm_busy  = 1'b1;
                                if (rdy_to_rsp_tnfr) begin
                                    nxt_in_err_sts_code[4:3] = 2'b11;
                                    nxt_in_err_sts_code[2]   = 1'b1;
                                    nxt_in_err_sts_code[1]   = 1'b0;
                                    nxt_in_err_sts_code[0]   = 1'b0;
                                    hv_fsm_ns                = 3'd4;
                                end
                            end else begin
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                                if (!qp_retried) begin
                                    //acknack_psn = os_rd_req_psn;
                                    acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;
                                    // hv_fsm_ns = 3'd6;    // ***
                                    // hv_fsm_busy = 1'b1;  // ***
                                    hv_fsm_ns     = 3'd0;
                                end else hv_fsm_ns = 3'd0;
                            end
                        end else if (bth_psn_r == os_rd_req_psn) begin
                            if (rdrsp_only_len_chk_fail || (aeth_syndr_r[6:5] != 2'b00)) begin
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                                hv_fsm_busy  = 1'b1;
                                if (rdy_to_rsp_tnfr) begin
                                    nxt_in_err_sts_code[4:3] = 2'b11;
                                    nxt_in_err_sts_code[2]   = 1'b0;
                                    nxt_in_err_sts_code[1]   = rdrsp_only_len_chk_fail;  //1
                                    nxt_in_err_sts_code[0]   = (aeth_syndr_r[6:5] != 2'b00) ? 1'b1 : 1'b0;  //0
                                    hv_fsm_ns                = 3'd4;
                                end
                            end else begin
                                rsp_dest_addr           = os_rd_req_dest_addr;
                                rsp_pkt_type            = (ip_ver_r == IPv4) ? RSP_AXI_TNFR_TYPE_IPV4_READ_ONLY_OR_LAST : RSP_AXI_TNFR_TYPE_IPV6_READ_ONLY_OR_LAST;
                                rsp_imm_data            = {{(16 - C_QP_INDX_WIDTH) {1'b0}}, bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0], 16'd0};
                                rsp_tnfr_vld            = 1'b1;
                                o_stat_resp_psn_int     = bth_psn_r;
                                stat_resp_psn_en_int    = 1'b1;
                                stat_resp_psn_we_int    = 1'b1;

                                o_last_rsp_opcode       = 1'b0;
                                last_rsp_opcode_we      = 1'b1;

                                rdrsp_dest_addr_en_int  = 1'b1;
                                rdrsp_dest_addr_we_int  = 1'b1;

                                rdrsp_len_remain_en_int = 1'b1;
                                rdrsp_len_remain_we_int = 1'b1;

                                o_rnr_nak_cnt           = rnr_nak_rst_val_r;
                                stat_nak_en_int         = 1'b1;
                                stat_nak_we_int         = 1'b1;

                                acknack_psn             = bth_psn_r;
                                acknack_syndr           = aeth_syndr_r;
                                acknack_vld             = 1'b1;
                                mpsnbuf_pop             = 1'b1;

                                if (!rdy_to_rsp_tnfr) begin
                                    hv_fsm_busy = 1'b1;
                                    hv_fsm_ns   = 3'd5;
                                end else begin
                                    hv_fsm_ns = 3'd0;
                                end
                            end
                        end else begin  // skip read response case
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;
                            if (unack_psn_min != os_rd_req_psn) begin
                                o_stat_resp_psn_int  = os_rd_req_psn - 'd1;
                                stat_resp_psn_en_int = 1'b1;
                                stat_resp_psn_we_int = 1'b1;

                                o_rnr_nak_cnt        = rnr_nak_rst_val_r;
                                stat_nak_en_int      = 1'b1;
                                stat_nak_we_int      = 1'b1;

                                //acknack_psn = os_rd_req_psn;
                                acknack_psn          = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                acknack_syndr        = {1'b0, NAK, PSN_SEQ_ERR};
                                acknack_vld          = 1'b1;

                                hv_fsm_ns            = 3'd0;
                            end else if (!qp_retried) begin
                                //acknack_psn = os_rd_req_psn;
                                acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                acknack_vld   = 1'b1;

                                hv_fsm_ns     = 3'd0;
                            end else hv_fsm_ns = 3'd0;
                        end  // skip read response case
                    end  // PKT_TYPE_READ_ONLY

                    PKT_TYPE_READ_FIRST: begin
                        if (!is_rsp_in_unack_range || mpsnbuf_empty) begin
                            //To pop the frame data from rsp pkt buffer
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;

                            hv_fsm_ns    = 3'd0;
                        end else if (i_last_rsp_opcode) begin
                            if (bth_psn_r == unack_psn_min) begin
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                                hv_fsm_busy  = 1'b1;
                                if (rdy_to_rsp_tnfr) begin
                                    nxt_in_err_sts_code[4:3] = 2'b11;
                                    nxt_in_err_sts_code[2]   = 1'b1;
                                    nxt_in_err_sts_code[1]   = rdrsp_first_len_chk_fail;
                                    nxt_in_err_sts_code[0]   = (aeth_syndr_r[6:5] != 2'b00) ? 1'b1 : 1'b0;
                                    hv_fsm_ns                = 3'd4;
                                end
                            end else begin
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                                if (!qp_retried) begin
                                    //acknack_psn = os_rd_req_psn;
                                    acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                    acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                    acknack_vld   = 1'b1;

                                    hv_fsm_ns     = 3'd0;
                                end else hv_fsm_ns = 3'd0;
                            end
                        end else if (bth_psn_r == os_rd_req_psn) begin
                            if (rdrsp_first_len_chk_fail || (aeth_syndr_r[6:5] != 2'b00)) begin
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                                hv_fsm_busy  = 1'b1;
                                if (rdy_to_rsp_tnfr) begin
                                    nxt_in_err_sts_code[4:3] = 2'b11;
                                    nxt_in_err_sts_code[2]   = 1'b0;
                                    nxt_in_err_sts_code[1]   = rdrsp_first_len_chk_fail;
                                    nxt_in_err_sts_code[0]   = (aeth_syndr_r[6:5] != 2'b00) ? 1'b1 : 1'b0;
                                    hv_fsm_ns                = 3'd4;
                                end
                            end else begin
                                rsp_dest_addr           = os_rd_req_dest_addr;
                                rsp_pkt_type            = (ip_ver_r == IPv4) ? RSP_AXI_TNFR_TYPE_IPV4_READ_FIRST : RSP_AXI_TNFR_TYPE_IPV6_READ_FIRST;
                                rsp_tnfr_vld            = 1'b1;
                                o_stat_resp_psn_int     = bth_psn_r;
                                stat_resp_psn_en_int    = 1'b1;
                                stat_resp_psn_we_int    = 1'b1;

                                o_last_rsp_opcode       = 1'b1;
                                last_rsp_opcode_we      = 1'b1;

                                rdrsp_dest_addr_en_int  = 1'b1;
                                rdrsp_dest_addr_we_int  = 1'b1;

                                rdrsp_len_remain_en_int = 1'b1;
                                rdrsp_len_remain_we_int = 1'b1;

                                o_rnr_nak_cnt           = rnr_nak_rst_val_r;
                                stat_nak_en_int         = 1'b1;
                                stat_nak_we_int         = 1'b1;

                                //acknack_psn = bth_psn_r-'d1;
                                acknack_psn             = bth_psn_r;
                                acknack_syndr           = aeth_syndr_r;
                                acknack_vld             = 1'b1;

                                if (!rdy_to_rsp_tnfr) begin
                                    hv_fsm_busy = 1'b1;
                                    hv_fsm_ns   = 3'd5;
                                end else begin
                                    hv_fsm_ns = 3'd0;
                                end
                            end
                        end else begin  // skip read response case
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;
                            if (unack_psn_min != os_rd_req_psn) begin
                                o_stat_resp_psn_int  = os_rd_req_psn - 'd1;
                                stat_resp_psn_en_int = 1'b1;
                                stat_resp_psn_we_int = 1'b1;

                                o_rnr_nak_cnt        = rnr_nak_rst_val_r;
                                stat_nak_en_int      = 1'b1;
                                stat_nak_we_int      = 1'b1;

                                //acknack_psn = os_rd_req_psn;
                                acknack_psn          = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                acknack_syndr        = {1'b0, NAK, PSN_SEQ_ERR};
                                acknack_vld          = 1'b1;

                                hv_fsm_ns            = 3'd0;
                            end else if (!qp_retried) begin
                                //acknack_psn = os_rd_req_psn;
                                acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                acknack_vld   = 1'b1;

                                hv_fsm_ns     = 3'd0;
                            end else hv_fsm_ns = 3'd0;
                        end  // skip read response case
                    end  // PKT_TYPE_READ_FIRST

                    PKT_TYPE_READ_MIDDLE: begin
                        if (!is_rsp_in_unack_range || mpsnbuf_empty) begin
                            //To pop the frame data from rsp pkt buffer
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;

                            hv_fsm_ns    = 3'd0;
                        end else if (bth_psn_r == unack_psn_min) begin
                            if (rdrsp_middle_len_chk_fail) begin
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                                hv_fsm_busy  = 1'b1;
                                if (rdy_to_rsp_tnfr) begin
                                    nxt_in_err_sts_code[4:3] = 2'b11;
                                    nxt_in_err_sts_code[2]   = ~i_last_rsp_opcode;
                                    nxt_in_err_sts_code[1]   = rdrsp_middle_len_chk_fail;
                                    nxt_in_err_sts_code[0]   = 1'b0;
                                    hv_fsm_ns                = 3'd4;
                                end
                            end else begin
                                rsp_pkt_type            = (ip_ver_r == IPv4) ? RSP_AXI_TNFR_TYPE_IPV4_READ_MIDDLE : RSP_AXI_TNFR_TYPE_IPV6_READ_MIDDLE;
                                rsp_tnfr_vld            = 1'b1;

                                o_stat_resp_psn_int     = bth_psn_r;
                                stat_resp_psn_en_int    = 1'b1;
                                stat_resp_psn_we_int    = 1'b1;

                                o_last_rsp_opcode       = 1'b1;
                                last_rsp_opcode_we      = 1'b1;

                                rdrsp_dest_addr_en_int  = 1'b1;
                                rdrsp_dest_addr_we_int  = 1'b1;

                                rdrsp_len_remain_en_int = 1'b1;
                                rdrsp_len_remain_we_int = 1'b1;

                                o_rnr_nak_cnt           = rnr_nak_rst_val_r;
                                stat_nak_en_int         = 1'b1;
                                stat_nak_we_int         = 1'b1;

                                //acknack_psn = bth_psn_r-'d1;
                                acknack_psn             = bth_psn_r;
                                acknack_syndr           = 8'd0;
                                acknack_vld             = 1'b1;

                                if (!rdy_to_rsp_tnfr) begin
                                    hv_fsm_busy = 1'b1;
                                    hv_fsm_ns   = 3'd5;
                                end else begin
                                    hv_fsm_ns = 3'd0;
                                end
                            end
                        end else begin
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;
                            if (!qp_retried) begin
                                acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                acknack_vld   = 1'b1;

                                hv_fsm_ns     = 3'd0;
                            end else begin
                                hv_fsm_ns = 3'd0;
                            end
                        end
                    end  // PKT_TYPE_READ_MIDDLE

                    PKT_TYPE_READ_LAST: begin
                        if (!is_rsp_in_unack_range || mpsnbuf_empty) begin
                            //To pop the frame data from rsp pkt buffer
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;

                            hv_fsm_ns    = 3'd0;
                        end else if (bth_psn_r == unack_psn_min) begin
                            if (rdrsp_last_len_chk_fail ||  /*!i_last_rsp_opcode ||*/ (aeth_syndr_r[6:5] != 2'b00)) begin
                                rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                                rsp_imm_data = 32'h2040_0000;
                                rsp_pkt_len  = 13'h0;
                                rsp_tnfr_vld = 1'b1;
                                hv_fsm_busy  = 1'b1;
                                if (rdy_to_rsp_tnfr) begin
                                    nxt_in_err_sts_code[4:3] = 2'b11;
                                    nxt_in_err_sts_code[2]   = ~i_last_rsp_opcode;
                                    nxt_in_err_sts_code[1]   = rdrsp_last_len_chk_fail;
                                    nxt_in_err_sts_code[0]   = (aeth_syndr_r[6:5] != 2'b00) ? 1'b1 : 1'b0;
                                    hv_fsm_ns                = 3'd4;
                                end
                            end else begin
                                rsp_pkt_type            = (ip_ver_r == IPv4) ? RSP_AXI_TNFR_TYPE_IPV4_READ_ONLY_OR_LAST : RSP_AXI_TNFR_TYPE_IPV6_READ_ONLY_OR_LAST;
                                rsp_imm_data            = {{(16 - C_QP_INDX_WIDTH) {1'b0}}, bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0], 16'd0};
                                rsp_tnfr_vld            = 1'b1;

                                o_stat_resp_psn_int     = bth_psn_r;
                                stat_resp_psn_en_int    = 1'b1;
                                stat_resp_psn_we_int    = 1'b1;

                                o_last_rsp_opcode       = 1'b0;
                                last_rsp_opcode_we      = 1'b1;

                                rdrsp_dest_addr_en_int  = 1'b1;
                                rdrsp_dest_addr_we_int  = 1'b1;

                                rdrsp_len_remain_en_int = 1'b1;
                                rdrsp_len_remain_we_int = 1'b1;

                                o_rnr_nak_cnt           = rnr_nak_rst_val_r;
                                stat_nak_en_int         = 1'b1;
                                stat_nak_we_int         = 1'b1;

                                acknack_psn             = bth_psn_r;
                                acknack_syndr           = 8'd0;
                                acknack_vld             = 1'b1;
                                mpsnbuf_pop             = 1'b1;

                                if (!rdy_to_rsp_tnfr) begin
                                    hv_fsm_busy = 1'b1;
                                    hv_fsm_ns   = 3'd5;
                                end else begin
                                    hv_fsm_ns = 3'd0;
                                end
                            end
                        end else begin
                            rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                            rsp_imm_data = 32'h2040_0000;
                            rsp_pkt_len  = 13'h0;
                            rsp_tnfr_vld = 1'b1;
                            if (!qp_retried) begin
                                acknack_psn   = unack_psn_min;  //Partial RD_REQ_RETRY //os_rd_req_psn;
                                acknack_syndr = {1'b0, NAK, PSN_SEQ_ERR};
                                acknack_vld   = 1'b1;
                                hv_fsm_ns     = 3'd0;
                            end else begin
                                hv_fsm_ns = 3'd0;
                            end
                        end
                    end  // PKT_TYPE_READ_LAST  

                endcase  // case(pkt_type)
            end  // 3'd3

            3'd4: begin
                rsp_imm_data = {{(16 - C_QP_INDX_WIDTH) {1'b0}}, bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0], 11'd0, in_err_sts_code};
                req_tnfr_vld = 1'b1;

                if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                else begin
                    // reset QP state
                    o_last_rsp_opcode    = 1'b0;
                    last_rsp_opcode_we   = 1'b1;
                    // update stat_nak (for aeth_syndrome)
                    stat_nak_en_int      = 1'b1;
                    stat_nak_we_int      = 1'b1;

                    // added to know on what PSN QP goes to Fatal state
                    o_stat_resp_psn_int  = bth_psn_r;
                    stat_resp_psn_en_int = 1'b1;
                    stat_resp_psn_we_int = 1'b1;

                    set_qp_fatal         = 1'b1;
                    hv_fsm_ns            = 3'd0;
                end
            end  // 3'd4

            3'd5: begin
                rsp_pkt_type  = rsp_pkt_type_r;
                rsp_dest_addr = rsp_dest_addr_r;
                rsp_imm_data  = rsp_imm_data_r;
                rsp_tnfr_vld  = 1'b1;

                if (!rdy_to_rsp_tnfr) hv_fsm_busy = 1'b1;
                else hv_fsm_ns = 3'd0;
            end

            3'd6: begin  // err_syndr != 32'd0
                if (err_syndr != 32'd0) begin
                    if (i_err_buf_en) begin
                        rsp_pkt_type = RSP_AXI_TNFR_TYPE_ERROR;
                        rsp_imm_data = err_syndr;
                        rsp_tnfr_vld = 1'b1;
                        if (!rdy_to_rsp_tnfr) hv_fsm_busy = 1'b1;
                    end
                    if (!i_err_buf_en || (i_err_buf_en && rdy_to_rsp_tnfr)) begin
                        hv_fsm_ns = 3'd0;
                        if (aeth_malformed_r) begin
                            o_rnr_nak_cnt   = rnr_nak_rst_val_r;
                            stat_nak_en_int = 1'b1;
                            stat_nak_we_int = 1'b1;
                        end
                    end
                end else begin  // err_syndr == 32'd0
                    hv_fsm_ns = 3'd0;
                end  // err_syndr == 32'd0
            end

            default: begin
                hv_fsm_ns = 3'dx;
            end  // default      
        endcase  // case (hv_fsm_cs)

    end  // always @(*)  

    //===============================
    // Header Validation FSM
    //===============================  

    always @(posedge clk)
        if (reset) hv_fsm_cs <= 3'd0;
        else hv_fsm_cs <= hv_fsm_ns;

    always @(posedge clk)
        if (reset) o_acknack_pkt_cnt <= 16'd0;
        else if (incr_acknack_pkt_cnt) o_acknack_pkt_cnt <= o_acknack_pkt_cnt + 'd1;

    always @(posedge clk)
        if (reset) o_inc_nack_pkt_cnt <= 16'd0;
        else if (incr_inc_nack_pkt_cnt) o_inc_nack_pkt_cnt <= o_inc_nack_pkt_cnt + 'd1;

    assign o_last_in_rsp_pkt_opcode = bth_opcode_r;
    assign o_last_in_rsp_pkt_qpid   = bth_qp_dest_addr_r[7:0];
    assign o_last_in_rsp_pkt_psn    = bth_psn_r[15:0];

    // =====================================================================
    // Debug Logic
    // =====================================================================      
    always @(posedge clk)
        if (reset) qp_fatal_rsp_hv_sticky <= 1'b0;
        else if (i_clr_debug_sts) qp_fatal_rsp_hv_sticky <= 1'b0;
        else if (set_qp_fatal) qp_fatal_rsp_hv_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) resp_drop_cnt <= 32'd0;
        else if (i_clr_debug_sts) resp_drop_cnt <= 32'd0;
        else if ((hv_fsm_cs == 3'd3 || hv_fsm_cs == 3'd6) && is_rsp_in_unack_range_at_vld && (osq_invld || (pkt_type[2] || i_last_rsp_opcode) && mpsnbuf_empty)) resp_drop_cnt <= resp_drop_cnt + 'd1;

endmodule
