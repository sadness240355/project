// /////////////////////////////////////////////////////////////////
// (c) Copyright 2013 - 2014 Xilinx, Inc. All rights reserved.   
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
// qp_mgr_top.v
//-----------------------------------------------------------------------------
//
// Filename        : qp_mgr_top.v
// Version         : v1.0
// Description     : RNIC configuration/status register logic with AXI4-Lite Interface
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- qp mgr top
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
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module qp_mgr_top #(
    parameter C_S_AXI_LITE_ADDR_WIDTH = 64,   // 32
    parameter C_S_AXI_LITE_DATA_WIDTH = 32,   // 32
    parameter C_M_AXI_ADDR_WIDTH      = 64,   // 64
    parameter C_NUM_QP                = 256,  // 32
    parameter C_QP_INDX_WIDTH         = 8,    // 5
    parameter C_NUM_PD                = 256,  // 2048
    parameter C_PD_INDX_WIDTH         = 8,    // 11
    parameter STB_WIDTH               = 64,   // 4
    parameter C_SQPI_DB_HNDSHK_EN     = 1,    // 1
    parameter RESP_WIDTH              = 2,    // 2
    parameter C_EN_DEBUG_REGS         = 1,    // 0
    parameter C_EN_INITIATOR_LITE     = 1,    // 1
    parameter C_OS_Q_DEPTH            = 16,   // 16
    parameter C_OS_Q_INDX_WIDTH       = 4,    // 4
    parameter C_ECN_THRESHOLD         = 8     // 12
) (
    input wire s_axi_lite_aclk,
    input wire s_axi_lite_aresetn,

    input  wire [C_S_AXI_LITE_ADDR_WIDTH-1:0] s_axi_lite_awaddr,
    output wire                               s_axi_lite_awready,
    input  wire                               s_axi_lite_awvalid,

    input  wire [C_S_AXI_LITE_ADDR_WIDTH-1:0] s_axi_lite_araddr,
    output wire                               s_axi_lite_arready,
    input  wire                               s_axi_lite_arvalid,

    input  wire [C_S_AXI_LITE_DATA_WIDTH-1:0] s_axi_lite_wdata,
    input  wire [              STB_WIDTH-1:0] s_axi_lite_wstrb,
    output wire                               s_axi_lite_wready,
    input  wire                               s_axi_lite_wvalid,

    output wire [C_S_AXI_LITE_DATA_WIDTH-1:0] s_axi_lite_rdata,
    output wire [             RESP_WIDTH-1:0] s_axi_lite_rresp,
    input  wire                               s_axi_lite_rready,
    output wire                               s_axi_lite_rvalid,

    output wire [RESP_WIDTH-1:0] s_axi_lite_bresp,
    input  wire                  s_axi_lite_bready,
    output wire                  s_axi_lite_bvalid,

    input wire core_clk,  // Module Clock  
    input wire core_rstn, // Active low core reset

    // Interface to WQE processor for arbitrated WQ entries
    // 用于仲裁 WQE 的 WQE 处理器接口
    output wire [                 511:0] o_wqe,
    input  wire                          i_wqe_pop,
    output wire                          o_wqe_empty,
    output wire                          o_halt,
    output wire [C_QP_INDX_WIDTH -1 : 0] o_halted_qpid,
    input  wire                          i_wqe_halted,

    // Interrupt signals coming from Header validation module
    // 来自头部验证模块的中断信号
    input wire i_pkt_valdn_err_intr,
    input wire i_mad_pkt_rcvd_intr,
    input wire i_bypass_pkt_rcvd_intr,
    input wire i_rnr_nack_gen_intr,
    input wire i_ill_opc_in_sq_intr,

    output wire o_intr_clr_pkt_valdn_err,
    output wire o_intr_clr_wqe_cmpl,
    output wire o_intr_clr_mad_pkt_rcvd,
    output wire o_intr_clr_bypass_pkt_rcvd,
    output wire o_intr_clr_rnr_nak_gen,
    output wire o_intr_clr_ill_opc_in_sq,
    output wire o_intr_clr_qp_pkt_rcvd,
    output wire o_intr_clr_fatal_err,

    output wire o_intr_en_pkt_valdn_err,
    output wire o_intr_en_wqe_cmpl,
    output wire o_intr_en_mad_pkt_rcvd,
    output wire o_intr_en_bypass_pkt_rcvd,
    output wire o_intr_en_rnr_nak_gen,
    output wire o_intr_en_ill_opc_in_sq,
    output wire o_intr_en_fatal_err,
    output wire o_intr_en_qp_pkt_rcvd,

    // Status inputs from Header validation module
    // 来自头部验证模块的状态输入
    input wire                         i_rq_full_valid,           // Rcv Q full for QPn
    input wire [C_QP_INDX_WIDTH-1 : 0] i_rq_full_idx,
    input wire                         i_rq_full_data,
    input wire                         i_rq_full_reset_valid,
    input wire [C_QP_INDX_WIDTH-1 : 0] i_rq_full_reset_idx,
    input wire                         i_rq_full_reset_data,
    input wire                         i_osq_full_set_valid,      // Outstanding Q full for QPn
    input wire [C_QP_INDX_WIDTH-1 : 0] i_osq_full_set_id,
    input wire                         i_osq_full_reset_valid,
    input wire [C_QP_INDX_WIDTH-1 : 0] i_osq_full_reset_id,
    input wire                         i_osq_empty_set_valid,     // Outstanding Q empty for QPn
    input wire [C_QP_INDX_WIDTH-1 : 0] i_osq_empty_set_id,
    input wire                         i_osq_empty_reset_valid,
    input wire [C_QP_INDX_WIDTH-1 : 0] i_osq_empty_reset_id,
    input wire                         i_qp_retried_set_valid,    // QP retried for QPn
    input wire [C_QP_INDX_WIDTH-1 : 0] i_qp_retried_set_id,
    input wire                         i_qp_retried_reset_valid,
    input wire [C_QP_INDX_WIDTH-1 : 0] i_qp_retried_reset_id,

    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_msn_idx,   // Index for reading/writing MSN
    input  wire [                23:0] i_qp_stat_msn,       // Expected MSN for Qp incoming messages
    output wire [                23:0] o_qp_stat_msn,       // Expected MSN for Qp incoming messages write data from hdr validation
    input  wire                        i_qp_stat_msn_req,
    input  wire                        i_qp_stat_msn_wrn,
    output wire                        o_qp_stat_msn_valid,

    output wire [31:0] o_rx_pkt_hndl_dbg_ctrl,

    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_ssn_idx,
    input  wire [                23:0] i_qp_stat_ssn,
    output wire [                23:0] o_qp_stat_ssn,
    input  wire                        i_qp_stat_ssn_req,
    input  wire                        i_qp_stat_ssn_wrn,
    output wire                        o_qp_stat_ssn_valid,

    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_timeout_idx,  // Index for reading/writing MSN
    output wire [                31:0] o_qp_timeout,      // Expected MSN for Qp incoming messages write data from hdr validation

    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_rq_pi_db_idx,   // Index for reading/writing RQ PI DB
    input  wire [                15:0] i_qp_rq_pi_db,
    output wire [                15:0] o_qp_rq_pi_db,
    input  wire                        i_qp_rq_pi_db_req,
    input  wire                        i_qp_rq_pi_db_wrn,
    output wire                        o_qp_rq_pi_db_valid,

    // Retried SQ PSN for Rx packet handler
    // 为 Rx 数据包处理程序 重试 SQ PSN
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_ret_sq_psn_idx,   // Index for reading/writing RQ PI DB
    output wire [                23:0] o_qp_stat_ret_sq_psn,
    input  wire                        i_qp_stat_ret_sq_psn_req,
    output wire                        o_qp_stat_ret_sq_psn_valid,


    // Status NAK details from RX paket handler {CURR_RNR_NAK_CNT, NAK_SYNDRM_RCVD}
    input  wire [                 9:0] i_qp_stat_nak,
    input  wire                        i_qp_stat_nak_wen,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_nak_idx,
    output wire                        o_qp_stat_nak_valid,
    input  wire                        i_qp_stat_nak_rdreq,
    output wire [                 9:0] o_qp_stat_nak,

    // Status NAK details from resp handler
    input  wire [                 2:0] i_qp_stat_retry_cnt,
    input  wire                        i_qp_stat_retry_cnt_wen,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_retry_cnt_idx,
    output wire                        o_qp_stat_retry_cnt_valid,
    input  wire                        i_qp_stat_retry_cnt_rdreq,
    output wire [                 2:0] o_qp_stat_retry_cnt,
    // Status from WQE proc engine
    input  wire [                31:0] i_wqe_proc_sts,
    input  wire [                31:0] i_rx_pkt_vld_sts,

    //Configuration outputs going to other modules (generic)
    output wire                          o_xrnic_en,
    output wire                          o_xrnic_adv_conf_errbuf_overwr_en,
    output wire                          o_xrnic_adv_conf_retry_cnt_fatal_dis,
    output wire [                   3:0] o_xrnic_adv_base_cnt,
    output wire [                   1:0] o_tx_ack_gen,
    output wire                          o_err_buf_en,
    output wire [                   1:0] o_flow_credits,
    output wire [                  15:0] o_xrnic_udp_src_port,
    output wire                          o_depkt_bypass_en,                     // XRNIC interface bypass feature enable
    output wire [   C_QP_INDX_WIDTH-1:0] o_num_qp_en,                           // Number of QPs enabled
    output wire [                  47:0] o_mac_xrnic_addr,                      // Ethernet MAC destination (own) address from XRNIC registers
    output wire [                  31:0] o_ipv4_xrnic_addr,                     // IPv4 destination (own) address from XRNIC registers
    output wire [                 127:0] o_ipv6_xrnic_addr,                     // IPv6 destination (own) address form XRNIC registers
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_data_buf_ba,
    output wire [                  15:0] o_data_buf_sz_num_bufs,
    output wire [                  15:0] o_data_buf_sz_buf_sz,

    input  wire                          i_qp_sq_ba_req,
    output wire                          o_qp_sq_ba_valid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_qp_sq_ba,
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_sq_ba_idx,    // Index for reading Qp SQ BA

    //output wire [15:0]                                o_connect_io_residual_rq,
    output wire [               12:0] o_connect_io_qp_rq_pi_db_wptr,
    output wire [C_QP_INDX_WIDTH-1:0] o_connect_io_qpid,
    input  wire                       i_connect_io_qp_rq_pi_db_rdy,   // Ready indicates can accept another
    output wire [               15:0] o_rq_pi_db_hw_hndshk,
    output wire                       o_hw_hndshk_disable_to_0,

    output wire [31:0] o_bypass_buf_ba,
    output wire [15:0] o_bypass_buf_sz_num_bufs,
    output wire [15:0] o_bypass_buf_sz_buf_sz,
    input  wire [15:0] i_bypass_buf_wrptr,

    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_rq_err_pkt_buf_ba,
    output wire [                  15:0] o_rq_err_pkt_buf_sz_num_bufs,
    output wire [                  15:0] o_rq_err_pkt_buf_sz_buf_sz,
    input  wire [                  15:0] i_rq_err_pkt_buf_wrptr,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_resp_err_pkt_buf_ba,
    output wire [                  15:0] o_resp_err_pkt_buf_sz_num_bufs,
    output wire [                  15:0] o_resp_err_pkt_buf_sz_buf_sz,
    input  wire [                  15:0] i_resp_err_pkt_buf_wrptr,

    input wire [15:0] i_inc_rresp_pkt_cnt,
    input wire [15:0] i_inc_send_pkt_cnt,
    input wire [15:0] i_inc_ack_pkt_cnt,
    input wire [15:0] i_inc_mad_pkt_cnt,
    input wire [15:0] i_inc_inv_pkt_cnt,
    input wire [15:0] i_inc_dup_pkt_cnt,
    input wire [31:0] i_inc_all_dropped_cnt,
    input wire [15:0] i_inc_nack_cnt,
    input wire [15:0] i_inc_cnp_pkt_cnt,

    input wire [15:0] i_out_rdwr_pkt_cnt,
    input wire [15:0] i_out_send_pkt_cnt,
    input wire [15:0] i_out_mad_pkt_cnt,
    input wire [15:0] i_out_rd_resp_pkt_cnt,
    input wire [15:0] i_out_ack_pkt_cnt,
    input wire [15:0] i_out_nack_cnt,
    input wire [31:0] i_resp_hndler_sts,
    input wire [31:0] i_stat_retry_cnt,
    input wire [31:0] i_min_ipg_stat,
    input wire [31:0] i_ipg_0_4_cnt,
    input wire [31:0] i_ipg_5_9_cnt,
    input wire [31:0] i_ipg_10_14_cnt,
    input wire [31:0] i_ipg_15_19_cnt,
    input wire [31:0] i_all_rd_req_cnt,
    input wire [31:0] i_good_rd_req_cnt,
    input wire [31:0] i_dup_rd_req_cnt,

    output wire [                31:0] o_error_corrupt_conf,
    output wire [                15:0] o_tx_hdr_buf_sz_num_hdrs,
    output wire [                15:0] o_tx_hdr_buf_sz_buf_sz,
    output wire [                31:0] o_tx_sgl_buf_ba,
    output wire [                15:0] o_tx_sgl_buf_sz_num_sgls,
    output wire [                15:0] o_tx_sgl_buf_sz_buf_sz,
    output wire                        o_xrnic_conf_ipver,
    output wire                        o_qp_enable_valid,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_enable_id,
    output wire                        o_qp_enable_data,
    input                              clr_error_corrupt_en,


    //Configuration outputs going to other modules (PER QP)
    input  wire                        i_qp_conf_req_resp_hndl,
    input  wire                        i_qp_conf_req_wqe_proc,
    input  wire                        i_qp_conf_req_rx_pkt,
    output wire                        o_qp_conf_valid_resp_hndl,
    output wire                        o_qp_conf_valid_wqe_proc,
    output wire                        o_qp_conf_valid_rx_pkt,
    output wire [                31:0] o_qp_conf,                  // QP configuration
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_conf_idx,              // Index for reading Qp configuration 

    input  wire                        i_qp_cfg_retry_cnt_rd_req,
    output wire                        o_qp_cfg_retry_cnt_rd_rdy,
    output wire [                31:0] o_qp_conf_replica,          // QP configuration REPLICA
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_conf_replica_idx,      // Index for reading Qp configuration REPLICA
    input  wire                        i_qp_conf_replica_req,
    output wire                        o_qp_conf_replica_valid,

    input  wire                        i_qp_adv_conf_req,
    output wire                        o_qp_adv_conf_valid,
    output wire [                31:0] o_qp_adv_conf,        // QP advance configuration
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_adv_conf_idx,    // Index for reading Qp configuration

    input  wire                          i_qp_rq_ba_req,
    output wire                          o_qp_rq_ba_valid,
    output wire [C_M_AXI_ADDR_WIDTH-9:0] o_qp_rq_ba,
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_rq_ba_idx,    // Index for reading Qp RQ Buffer BA

    //input  wire                                       i_qp_sq_ba_req,
    //output wire                                       o_qp_sq_ba_valid,
    //output wire  [31:0]                               o_qp_sq_ba,
    //input  wire  [C_QP_INDX_WIDTH -1: 0]              i_qp_sq_ba_idx,   // Index for reading Qp SQ BA

    input  wire                          i_qp_cq_ba_req,
    output wire                          o_qp_cq_ba_valid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_qp_cq_ba,
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_cq_ba_idx,    // Index for reading Qp CQ BA

    input  wire                          i_qp_rq_wrptrdb_add_req,
    output wire                          o_qp_rq_wrptrdb_add_valid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_qp_rq_wrptrdb_add,
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_rq_wrptrdb_add_idx,    // Index for reading Qp SQ BA

    input  wire                          i_qp_sq_cmpldb_add_req,
    output wire                          o_qp_sq_cmpldb_add_valid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_qp_sq_cmpldb_add,
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_sq_cmpldb_add_idx,    // Index for reading SQ Completion DB address

    // HW handshake for SQ PI doorbell
    input  wire [15:0] i_qp_sq_pidb_hndshk,
    input  wire [31:0] i_qp_sq_pidb_wr_addr_hndshk,
    input  wire        i_qp_sq_pidb_wr_valid_hndshk,
    output wire        o_qp_sq_pidb_wr_rdy,
    //-------------------------------  
    // HW handshake for RQ CI doorbell
    input  wire [15:0] i_qp_rq_cidb_hndshk,
    input  wire [31:0] i_qp_rq_cidb_wr_addr_hndshk,
    input  wire        i_qp_rq_cidb_wr_valid_hndshk,
    output wire        o_qp_rq_cidb_wr_rdy,
    //-------------------------------  

    input  wire                        i_qp_cq_hdptr_req,
    output wire                        o_qp_cq_hdptr_valid,
    output wire [                15:0] o_qp_cq_hdptr,
    input  wire [                15:0] i_qp_cq_hdptr,
    input  wire                        i_qp_cq_hdptr_wrn,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_cq_hdptr_idx,    // Index for reading CQ head pointer

    input  wire                        i_qp_rq_cidb_req,
    output wire                        o_qp_rq_cidb_valid,
    output wire [                31:0] o_qp_rq_cidb,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_rq_cidb_idx,    // Index for reading RQ CI DB

    // Response PSN status register. Read/write interface to RX pkt handler
    // 响应 PSN 状态寄存器。RX pkt 处理器的读/写接口
    input  wire [                23:0] i_qp_stat_resp_psn,
    input  wire                        i_qp_stat_resp_psn_wen,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_resp_psn_idx,
    output wire                        o_qp_stat_resp_psn_valid,
    input  wire                        i_qp_stat_resp_psn_rdreq,
    output wire [                23:0] o_qp_stat_resp_psn,

    // RQ 缓冲区当前地址
    input  wire [C_M_AXI_ADDR_WIDTH-9:0] i_qp_stat_rq_buf_ca,
    input  wire                          i_qp_stat_rq_buf_ca_wen,
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_stat_rq_buf_ca_idx,
    output wire                          o_qp_stat_rq_buf_ca_valid,
    input  wire                          i_qp_stat_rq_buf_ca_rdreq,
    output wire [C_M_AXI_ADDR_WIDTH-9:0] o_qp_stat_rq_buf_ca,

    input  wire                        i_qp_rq_depth_req,
    output wire                        o_qp_rq_depth_valid,
    output wire [                15:0] o_qp_rq_depth,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_rq_depth_idx,

    // CQ depth information to Response handler. 
    // The CQ depth is equal to SQ depth. This information is shared by QP cache
    // manager and resp handler. Priority is for cache manager

    input  wire                        i_qp_cq_depth_req,
    output wire                        o_qp_cq_depth_valid,
    output wire [                15:0] o_qp_cq_depth,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_cq_depth_idx,

    input  wire                        i_qp_sq_psn_req,
    input  wire                        i_qp_sq_psn_wqe_req,
    input  wire                        i_qp_sq_psn_resp_req,
    output wire                        o_qp_sq_psn_valid,
    output wire                        o_qp_sq_psn_wqe_valid,
    output wire                        o_qp_sq_psn_resp_valid,
    output wire [                23:0] o_qp_sq_psn,
    input  wire                        i_qp_sq_psn_wrn,
    input  wire [                23:0] i_qp_sq_psn,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_idx,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_wqe_idx,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_resp_idx,

    output wire [                31:0] o_qp_last_rq,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_last_rq_idx,
    input  wire [                31:0] i_qp_last_rq,
    input  wire                        i_qp_last_rq_req,
    input  wire                        i_qp_last_rq_wrn,
    output wire                        o_qp_last_rq_valid,

    output wire [                23:0] o_qp_pd_num,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_pd_num_idx,
    input  wire [                23:0] i_qp_pd_num,
    input  wire                        i_qp_pd_num_req,
    input  wire                        i_qp_pd_num_wrn,
    output wire                        o_qp_pd_num_valid,

    output wire [                23:0] o_pd_pd_num,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_pd_num_idx,
    input  wire [                23:0] i_pd_pd_num,
    input  wire                        i_pd_pd_num_req,
    input  wire                        i_pd_pd_num_wrn,
    output wire                        o_pd_pd_num_valid,

    input  wire                        i_pd_virt_addr_0_wrn,
    output wire [                31:0] o_pd_virt_addr_0,
    input  wire [                31:0] i_pd_virt_addr_0,
    output wire                        o_pd_virt_addr_0_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_virt_addr_0_idx,    // Index for reading Q depth
    input  wire                        i_pd_virt_addr_0_req,

    input  wire                        i_pd_virt_addr_1_wrn,
    output wire [                31:0] o_pd_virt_addr_1,
    input  wire [                31:0] i_pd_virt_addr_1,
    output wire                        o_pd_virt_addr_1_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_virt_addr_1_idx,    // Index for reading Q depth
    input  wire                        i_pd_virt_addr_1_req,

    input  wire                        i_pd_buf_base_addr_0_wrn,
    output wire [                31:0] o_pd_buf_base_addr_0,
    input  wire [                31:0] i_pd_buf_base_addr_0,
    output wire                        o_pd_buf_base_addr_0_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_buf_base_addr_0_idx,    // Index for reading Q depth
    input  wire                        i_pd_buf_base_addr_0_req,

    input  wire                        i_pd_buf_base_addr_1_wrn,
    output wire [                31:0] o_pd_buf_base_addr_1,
    input  wire [                31:0] i_pd_buf_base_addr_1,
    output wire                        o_pd_buf_base_addr_1_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_buf_base_addr_1_idx,    // Index for reading Q depth
    input  wire                        i_pd_buf_base_addr_1_req,

    input  wire                        i_pd_buf_r_key_wrn,
    output wire [                 7:0] o_pd_buf_r_key,
    input  wire [                 7:0] i_pd_buf_r_key,
    output wire                        o_pd_buf_r_key_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_buf_r_key_idx,    // Index for reading Q depth
    input  wire                        i_pd_buf_r_key_req,

    input  wire                        i_pd_w_r_buf_len_wrn,
    output wire [                47:0] o_pd_w_r_buf_len,
    input  wire [                47:0] i_pd_w_r_buf_len,
    output wire                        o_pd_w_r_buf_len_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_w_r_buf_len_idx,    // Index for reading Q depth
    input  wire                        i_pd_w_r_buf_len_req,

    input  wire                        i_pd_access_type_wrn,
    output wire [                 3:0] o_pd_access_type,
    input  wire [                 3:0] i_pd_access_type,
    output wire                        o_pd_access_type_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_access_type_idx,    // Index for reading Q depth
    input  wire                        i_pd_access_type_req,

    input  wire                        i_qp_dest_qpid_req_wqe_proc,
    input  wire                        i_qp_dest_qpid_req_rx_pkt,
    output wire                        o_qp_dest_qpid_valid_wqe_proc,
    output wire                        o_qp_dest_qpid_valid_rx_pkt,
    output wire [                23:0] o_qp_dest_qpid,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_dest_qpid_idx,

    output wire [31:0] o_timeoutreg,

    output wire                        o_qp_disable_pulse_valid,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_disable_pulse_id,
    output wire                        o_qp_disable_pulse_data,

    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_out_errsts_q_ba,
    output wire [                  15:0] o_out_errsts_q_sz,
    input  wire [                  15:0] i_out_errsts_q_wrptr,

    output wire [C_M_AXI_ADDR_WIDTH -1:0] o_in_errsts_q_ba,
    output wire [                   15:0] o_in_errsts_q_sz,
    input  wire [                   15:0] i_in_errsts_q_wrptr,
    //  input  wire                                       i_qp_rnr_nck_tval_curr_retry_req,
    //  input  wire                                       i_qp_rnr_nck_tval_curr_retry_wrn,
    //  input  wire  [7:0]                                i_qp_rnr_nck_tval_curr_retry, // CURR_RETRY_CNT [7:5] ;RNR_NACK_TVAL [4:0]

    input  wire                        i_qp_mac_remote_addrl_req,
    input  wire                        i_qp_mac_remote_addrl_wqe_req,
    output wire                        o_qp_mac_remote_addrl_valid,
    output wire                        o_qp_mac_remote_addrl_wqe_valid,
    output wire [                31:0] o_qp_mac_remote_addrl,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_idx,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_wqe_idx,    // Index for reading Q depth

    input  wire                        i_qp_mac_remote_addrl_replica_req,
    output wire [                31:0] o_qp_mac_remote_addrl_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_replica_idx,   // Index for reading Q depth
    output wire                        o_qp_mac_remote_addrl_replica_valid,

    input  wire                        i_qp_mac_remote_addrm_req,
    input  wire                        i_qp_mac_remote_addrm_wqe_req,
    output wire                        o_qp_mac_remote_addrm_valid,
    output wire                        o_qp_mac_remote_addrm_wqe_valid,
    output wire [                31:0] o_qp_mac_remote_addrm,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_idx,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_wqe_idx,    // Index for reading Q depth

    output wire [                31:0] o_qp_mac_remote_addrm_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_replica_idx,   // Index for reading Q depth
    input  wire                        i_qp_mac_remote_addrm_replica_req,
    output wire                        o_qp_mac_remote_addrm_replica_valid,

    input  wire                        i_qp_ip_remote_addr1_req,
    output wire                        o_qp_ip_remote_addr1_valid,
    input  wire                        i_qp_ip_remote_addr1_wqe_req,
    output wire                        o_qp_ip_remote_addr1_wqe_valid,
    output wire [                31:0] o_qp_ip_remote_addr1,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_idx,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_wqe_idx,    // Index for reading Q depth

    output wire [                31:0] o_qp_ip_remote_addr1_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_replica_idx,   // Index for reading Q depth
    input  wire                        i_qp_ip_remote_addr1_replica_req,
    output wire                        o_qp_ip_remote_addr1_replica_valid,

    input  wire                        i_qp_ip_remote_addr2_req,
    input  wire                        i_qp_ip_remote_addr2_wqe_req,
    output wire                        o_qp_ip_remote_addr2_valid,
    output wire                        o_qp_ip_remote_addr2_wqe_valid,
    output wire [                31:0] o_qp_ip_remote_addr2,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_idx,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_wqe_idx,    // Index for reading Q depth

    output wire [                31:0] o_qp_ip_remote_addr2_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_replica_idx,   // Index for reading Q depth
    input  wire                        i_qp_ip_remote_addr2_replica_req,
    output wire                        o_qp_ip_remote_addr2_replica_valid,

    input  wire                        i_qp_ip_remote_addr3_req,
    output wire                        o_qp_ip_remote_addr3_valid,
    input  wire                        i_qp_ip_remote_addr3_wqe_req,
    output wire                        o_qp_ip_remote_addr3_wqe_valid,
    output wire [                31:0] o_qp_ip_remote_addr3,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_idx,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_wqe_idx,    // Index for reading Q depth

    input  wire                        i_qp_ip_remote_addr3_replica_req,
    output wire                        o_qp_ip_remote_addr3_replica_valid,
    output wire [                31:0] o_qp_ip_remote_addr3_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_replica_idx,    // Index for reading Q depth

    input  wire                        i_qp_ip_remote_addr4_req,
    output wire                        o_qp_ip_remote_addr4_valid,
    input  wire                        i_qp_ip_remote_addr4_wqe_req,
    output wire                        o_qp_ip_remote_addr4_wqe_valid,
    output wire [                31:0] o_qp_ip_remote_addr4,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_idx,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_wqe_idx,    // Index for reading Q depth

    input  wire                        i_qp_ip_remote_addr4_replica_req,
    output wire                        o_qp_ip_remote_addr4_replica_valid,
    output wire [                31:0] o_qp_ip_remote_addr4_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_replica_idx,    // Index for reading Q depth

    input  wire                       i_waiting_for_rq_update,
    input  wire [C_QP_INDX_WIDTH-1:0] i_waiting_for_rq_update_on_q_num,
    //Performacne debug counter i/f
    output wire [               15:0] o_global_dbg_cnt_value,
    output wire                       o_global_dbg_cnt_clr,
    output wire                       o_global_dbg_cnt_en,
    input  wire [               15:0] i_wqe_fsm_idle_cnt,
    input  wire [               15:0] i_hdr_backpressure_cnt,
    input  wire [               15:0] i_retry_tx_backpressure_cnt,
    input  wire [               15:0] i_wqe_proc_rd_wqe_cnt,
    input  wire [               15:0] i_wqe_proc_rd_q_info_cnt,
    input  wire [               15:0] i_wqe_proc_wait0_cnt,
    input  wire [               15:0] i_wqe_proc_ip_chksum_cnt,
    input  wire [               15:0] i_wqe_proc_hdr_gen_cnt,
    input  wire [               15:0] i_wqe_proc_hdr_sto_cnt,


    input wire [31:0] i_last_in_pkt_info,
    input wire [31:0] i_last_out_pkt_info,

    // 用于重传逻辑的响应处理程序接口
    input  wire [C_QP_INDX_WIDTH -1 : 0] i_retransmit_qpid,              // QP id for this retransmission is required
    input  wire                          i_retransmit_reqd,              // Retransmission is required
    input  wire [                  47:0] i_retransmit_wqe_bram_din,
    output wire                          o_retransmit_accepted,          // Ack from QP manager that the retransmission is initiated
    input  wire                          i_osq_almost_full_set_valid,
    input  wire [ C_QP_INDX_WIDTH-1 : 0] i_osq_almost_full_set_id,
    input  wire                          i_osq_almost_full_reset_valid,
    input  wire [ C_QP_INDX_WIDTH-1 : 0] i_osq_almost_full_reset_id,
    input  wire [                  23:0] i_psn_to_retry,
    input  wire [                  23:0] i_ssn_to_retry,

    input wire [(2*C_QP_INDX_WIDTH+4)-1:0] i_qp_rnr_nacked,
    input wire [                     31:0] i_qp_pkt_rcvd_intr_wr_en,
    input wire [     C_QP_INDX_WIDTH -1:0] i_qp_pkt_rcvd_intr_wr_addr,
    input wire [                     31:0] i_qp_pkt_rcvd_intr_wr_data,
    input wire                             i_qp_fatal_err_valid,        // QPs those are stalled due to fatal error
    input wire [      C_QP_INDX_WIDTH-1:0] i_qp_fatal_err_id,
    input wire                             i_qp_fatal_err_data,

    output wire                         o_qp_clr_fatal_err_valid,
    output wire [ C_QP_INDX_WIDTH -1:0] o_qp_clr_fatal_err_id,
    output wire                         o_qp_clr_fatal_err_data,
    output wire                         o_qp_under_recovery_valid,
    output wire [ C_QP_INDX_WIDTH -1:0] o_qp_under_recovery_id,
    output wire                         o_qp_under_recovery_data,
    input  wire                         i_qp_wq_cmpl_intr_or_reduce,
    input  wire [                 31:0] i_qp_wq_cmpl_intr_wr_en,
    input  wire [                 31:0] i_qp_wq_cmpl_intr_wr_data,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_qp_wq_cmpl_intr_wr_addr,
    input  wire [                 31:0] i_qp_wq_cmpl_intr_wr_en_pending,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_qp_wq_cmpl_intr_wr_addr_pending,
    output wire [                 31:0] o_rq_intr_sts_clr_wr_en,
    output wire [  C_QP_INDX_WIDTH-1:0] o_rq_intr_sts_clr_wr_addr,
    output wire [                 31:0] o_cq_intr_sts_clr_wr_en,
    output wire [  C_QP_INDX_WIDTH-1:0] o_cq_intr_sts_clr_wr_addr,

    // CNP Config on RX path
    input  wire [               31:0] i_cnp_schedule_info_wr_en,
    input  wire [C_QP_INDX_WIDTH-1:0] i_cnp_schedule_info_wr_addr,
    input  wire [               31:0] i_cnp_schedule_info_wr_data,
    output wire [               31:0] o_cnp_schedule_clr_wr_en,
    output wire [C_QP_INDX_WIDTH-1:0] o_cnp_schedule_clr_wr_addr,
    output wire [                3:0] o_cnp_timeout_val,
    input  wire                       i_cnp_sched_intr_sts,
    output wire                       o_intr_en_cnp,
    output wire                       o_intr_clr_cnp,
    // CNP Config on TX path
    input  wire                       i_tx_path_congestion_valid,
    input  wire [C_QP_INDX_WIDTH-1:0] i_tx_path_congestion_id,
    input  wire                       i_tx_path_congestion_data,
    output wire [                3:0] o_ecn_timeout_val,
    //Pause_Offset
    output wire [                3:0] roce_priority,
    output wire [                3:0] non_roce_priority,
    output wire                       pfc_en_roce,
    output wire                       pfc_en_non_roce,
    output wire [               15:0] roce_pause_on_offset,
    output wire [               15:0] roce_pause_off_offset,
    output wire [               15:0] non_roce_pause_on_offset,
    output wire [               15:0] non_roce_pause_off_offset,
    output wire                       disable_priority_chk,



    // AXI Master 信号组
    // ----------------------------------------------------------
    // Write address channel
    output wire [                   0:0] m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [                   7:0] m_axi_awlen,
    output wire [                   2:0] m_axi_awsize,
    output wire [                   1:0] m_axi_awburst,
    output wire [                   3:0] m_axi_awcache,
    output wire [                   2:0] m_axi_awprot,
    output wire                          m_axi_awvalid,
    input  wire                          m_axi_awready,
    // Write data channel
    output wire [                 511:0] m_axi_wdata,
    output wire [             512/8-1:0] m_axi_wstrb,
    output wire                          m_axi_wlast,
    output wire                          m_axi_wvalid,
    input  wire                          m_axi_wready,
    output wire                          m_axi_awlock,
    // Write Response channel
    input  wire [                 0 : 0] m_axi_bid,
    input  wire [                   1:0] m_axi_bresp,
    input  wire                          m_axi_bvalid,
    output wire                          m_axi_bready,
    // Read address channel
    output wire [                   0:0] m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [                   7:0] m_axi_arlen,
    output wire [                   2:0] m_axi_arsize,
    output wire [                   1:0] m_axi_arburst,
    output wire [                   3:0] m_axi_arcache,
    output wire [                   2:0] m_axi_arprot,
    output wire                          m_axi_arvalid,
    input  wire                          m_axi_arready,
    // Read data/response channel
    input  wire [                   0:0] m_axi_rid,
    input  wire [                 511:0] m_axi_rdata,
    input  wire [                   1:0] m_axi_rresp,
    input  wire                          m_axi_rlast,
    input  wire                          m_axi_rvalid,
    output wire                          m_axi_rready,
    output wire                          m_axi_arlock,

    input wire                         i_osq_pop,
    input wire [C_QP_INDX_WIDTH - 1:0] i_osq_pop_qpid,
    input wire                         i_wqe_dropped,
    input wire [C_QP_INDX_WIDTH - 1:0] i_wqe_dropped_qpid,
    input wire                         init_start,
    input wire [C_QP_INDX_WIDTH-1 : 0] init_count

);

    `include "../../rdma_macros.vh"


    localparam WQE_SIZE = 14'd512;  // WQE entry size is 64 Bytes -> 512bits
    localparam WQE_SIZE_IN_BYTES = WQE_SIZE / 8;
    localparam WQE_FIFO_DEPTH = 3'd4;  //不要超过 4。参见下文*。与 WQE 处理器接口的 FIFO 深度
    localparam WQE_FIFO_IDX_WIDTH = 2;
    localparam OS_Q_ADDR_WIDTH = (clog2(C_OS_Q_DEPTH * C_NUM_QP));

    // SEND WQE FIFO向WQE处理器提供数据。该缓存用于支持WQE的连续处理。
    // 由于采用FIFO机制，若某条数据因响应处理器中该QP的待处理FIFO已满而无法被WQE模块处理，则后续所有数据条目也将被阻塞。
    // 因此仲裁模块会接收osq_almost_full信号，当QP触发该信号时将被排除在仲裁之外。
    // 为确保该逻辑始终正常运行，当FIFO中存在足够空间容纳所有WQE时，应断言almost_full信号。
    // 由于当前OSQ深度设置为8，若WQE FIFO深度超过4，在仅连接单个QP的情况下，OSQ可能严重闲置。

    wire                             halt;
    wire                             sw_override_en;
    wire [      C_QP_INDX_WIDTH-1:0] sw_override_qpid;
    wire [                     15:0] qp_cq_hdptr_from_config;
    wire [   WQE_FIFO_IDX_WIDTH : 0] num_valid_entries;  // SQ WQE FIFO中有效条目的数量
    wire [WQE_FIFO_IDX_WIDTH -1 : 0] wqe_fifo_rd_ptr;  // 从0开始
    wire [WQE_FIFO_IDX_WIDTH -1 : 0] wqe_fifo_wr_ptr;  // 从0开始
    wire                             wqe_full;  // SQ WQE FIFO满信号
    wire                             sq_empty_valid;
    wire [   C_QP_INDX_WIDTH -1 : 0] sq_empty_id;
    wire                             sq_empty_data;
    wire                             arbitrate;  // 仲裁请求
    wire                             sample_arbitrated_sq;
    wire                             arbitration_done;  // 仲裁完成
    wire [                      2:0] cache_fsm_status;
    wire [                     15:0] stat_wqe_cnt;
    wire [                    511:0] bus2ip_data;  // 刚从 DDR 读取的 SQ WQE 数据
    wire                             bus2ip_dvalid;
    reg  [                    511:0] bus2ip_data_r1;
    reg                              bus2ip_dvalid_r1;
    reg                              bus2ip_dvalid_r2;
    wire                             bus2ip_arready;
    wire [   C_M_AXI_ADDR_WIDTH-1:0] qp_sq_ba;
    wire                             axi_ren_valid;
    wire [   C_QP_INDX_WIDTH -1 : 0] qp_sq_arbitrated_idx;  // 通过仲裁选择出来的QP SQ索引
    wire [      C_QP_INDX_WIDTH : 0] num_elements_enabled;
    wire [                     15:0] qp_sq_pidb;
    wire                             qp_curr_sqptr_req;
    wire                             qp_sq_depth_req;
    wire                             qp_sq_depth_valid;
    wire                             qp_sq_ba_req_int;
    wire                             qp_sq_ba_valid_int;
    wire [                     15:0] new_curr_sqptr_proc;
    wire [                     15:0] qp_curr_sqptr_proc;
    wire [                     15:0] qp_sq_depth;
    wire [  C_M_AXI_ADDR_WIDTH -1:0] sq_read_addr_to_axi;
    reg  [                    511:0] wqe_qid_inserted;
    wire                             qp_cq_hdptr_req_retry;
    wire                             qp_cq_hdptr_valid_retry;
    wire [     C_QP_INDX_WIDTH -1:0] qp_cq_hdptr_idx_retry;
    wire [                     15:0] qp_curr_sqptr_proc_retry;
    wire                             qp_curr_sqptr_proc_wen_retry;
    wire [     C_QP_INDX_WIDTH -1:0] qp_curr_sqptr_proc_idx_retry;  // Index for writing curent SQ pointer
    wire                             qp_sq_psn_req_retry;
    wire                             qp_sq_psn_wrn_retry;
    wire [                     23:0] qp_sq_psn_retry;
    wire [     C_QP_INDX_WIDTH -1:0] qp_sq_psn_idx_retry;  // Index for reading Q depth
    wire [     C_QP_INDX_WIDTH -1:0] qp_stat_ssn_idx_retry;  // Index for reading/writing MSN
    wire [                     23:0] qp_stat_ssn_retry;  // Expected MSN for Qp incoming messages write data from hdr validation
    wire                             qp_stat_ssn_req_retry;
    wire                             qp_stat_ssn_wrn_retry;
    wire [                    511:0] wqe_from_fifo;
    wire [                    511:0] wqe_to_fifo;
    wire [                    511:0] cached_wqe;
    wire                             wqe_fifo_push_retry;
    wire                             wqe_fifo_pop_retry;
    wire                             cache_halted;
    wire                             qp_sq_psn_retry_valid;
    wire                             qp_stat_wqe_cnt_valid;
    wire                             qp_stat_wqe_cnt_rdreq;
    wire [                     15:0] qp_stat_wqe_cnt;
    wire                             update_qp_state_wqe_cnt;
    wire [                     15:0] new_qp_stat_wqe_cnt;

    wire [                     48:0] retransmit_wqe_bram_din_a;
    wire [                     48:0] retransmit_wqe_bram_dout_a;
    wire                             retransmit_wqe_bram_wen_a;
    wire [      C_QP_INDX_WIDTH-1:0] retransmit_wqe_bram_idx_a;
    wire [                     48:0] retransmit_wqe_bram_dout_b;


    reg  [                      7:0] axi_rd_err_cnt_ff;
    // reg  [                     15:0] qp_sq_depth_ff;
    // reg  [                     31:0] qp_sq_ba_ff;
    // reg  [                     15:0] qp_curr_sqptr_proc_ff;
    reg                              axi_ren_valid_ff;
    reg  [   C_QP_INDX_WIDTH -1 : 0] qp_sq_arbitrated_idx_ff;
    reg                              wqe_empty_ff;

    wire [                     23:0] qp_stat_ret_sq_psn_int;
    wire [     C_QP_INDX_WIDTH -1:0] qp_stat_ret_sq_psn_idx_int;

    wire [      C_QP_INDX_WIDTH-1:0] wqe_qpid_from_qp_cache;
    wire [      C_QP_INDX_WIDTH-1:0] bus2ip_qid;  // 仲裁后的 SQ QPID
    reg  [      C_QP_INDX_WIDTH-1:0] bus2ip_qid_r1;
    wire                             retransmit_wqe_dropped;
    wire [    C_QP_INDX_WIDTH - 1:0] retransmit_wqe_dropped_qpid;


    wire [                      2:0] arb_reqs;
    wire [                      2:0] arb_gnt;
    wire                             arb_done;

    wire                             osq_pop_fifo_empty;
    wire                             osq_pop_fifo_rden;
    wire [    C_QP_INDX_WIDTH - 1:0] osq_pop_fifo_dout;

    wire                             in_wqe_dropped_fifo_empty;
    wire                             in_wqe_dropped_fifo_rden;
    wire [    C_QP_INDX_WIDTH - 1:0] in_wqe_dropped_fifo_dout;

    wire                             retransmit_wqe_fifo_empty;
    wire                             retransmit_wqe_fifo_rden;
    wire [    C_QP_INDX_WIDTH - 1:0] retransmit_wqe_fifo_dout;

    wire                             arb_gnt_popen;
    reg  [    C_QP_INDX_WIDTH - 1:0] arb_gnt_pop_qpid;
    wire                             arbitrate_pop;

    wire                             retrans_check_done;
    reg                              retrans_check_update;
    wire [                     63:0] retransmit_local_addr;
    wire [                     63:0] retransmit_remote_addr;
    wire [                     31:0] retransmit_dma_len;
    wire                             osq_full_set_valid;
    wire [    C_QP_INDX_WIDTH-1 : 0] osq_full_set_id;
    wire                             osq_full_reset_valid;
    wire [    C_QP_INDX_WIDTH-1 : 0] osq_full_reset_id;
    wire                             sq_empty_and_reduce;  // 所有 QP 的 SEND Queue 当前是否都为空
    wire                             osq_rnr_nacked_ff_valid_pending;
    wire [    C_QP_INDX_WIDTH-1 : 0] osq_rnr_nacked_ff_id_pending;
    wire                             osq_rnr_nacked_ff_data_pending;
    wire                             osq_rnr_nacked_ff_valid;
    wire [    C_QP_INDX_WIDTH-1 : 0] osq_rnr_nacked_ff_id;
    wire                             osq_rnr_nacked_ff_data;
    wire                             prog_full_db_update_curr_sqptr;


    assign num_elements_enabled = o_num_qp_en;  //C_NUM_QP;

    // Intiator
    // 配置寄存器
    rnic_config_reg #(
        .C_S_AXI_LITE_ADDR_WIDTH(C_S_AXI_LITE_ADDR_WIDTH),
        .C_S_AXI_LITE_DATA_WIDTH(C_S_AXI_LITE_DATA_WIDTH),
        .C_M_AXI_ADDR_WIDTH     (C_M_AXI_ADDR_WIDTH),
        .C_NUM_QP               (C_NUM_QP),
        .C_QP_INDX_WIDTH        (C_QP_INDX_WIDTH),
        .C_NUM_PD               (C_NUM_PD),
        .C_PD_INDX_WIDTH        (C_PD_INDX_WIDTH),
        .STB_WIDTH              (STB_WIDTH),
        .RESP_WIDTH             (RESP_WIDTH),
        .C_EN_DEBUG_REGS        (C_EN_DEBUG_REGS),
        .C_EN_INITIATOR_LITE    (C_EN_INITIATOR_LITE)
    ) inst_rnic_config (
        // AXI Lite 信号组
        // ----------------------------------------------------------
        .s_axi_lite_aclk   (s_axi_lite_aclk),
        .s_axi_lite_aresetn(s_axi_lite_aresetn),

        .s_axi_lite_awaddr (s_axi_lite_awaddr),
        .s_axi_lite_awready(s_axi_lite_awready),
        .s_axi_lite_awvalid(s_axi_lite_awvalid),

        .s_axi_lite_araddr (s_axi_lite_araddr),
        .s_axi_lite_arready(s_axi_lite_arready),
        .s_axi_lite_arvalid(s_axi_lite_arvalid),

        .s_axi_lite_wdata (s_axi_lite_wdata),
        .s_axi_lite_wstrb (s_axi_lite_wstrb),
        .s_axi_lite_wready(s_axi_lite_wready),
        .s_axi_lite_wvalid(s_axi_lite_wvalid),

        .s_axi_lite_rdata (s_axi_lite_rdata),
        .s_axi_lite_rresp (s_axi_lite_rresp),
        .s_axi_lite_rready(s_axi_lite_rready),
        .s_axi_lite_rvalid(s_axi_lite_rvalid),

        .s_axi_lite_bresp (s_axi_lite_bresp),
        .s_axi_lite_bready(s_axi_lite_bready),
        .s_axi_lite_bvalid(s_axi_lite_bvalid),
        // ----------------------------------------------------------

        // 来自缓存模块的更新信号
        .i_status_upd_needed(status_upd_needed),       // no use
        .i_status_upd_indx  (qp_sq_arbitrated_idx_ff), // no use

        // 来自/前往头部验证模块的中断信号
        .i_pkt_valdn_err_intr  (i_pkt_valdn_err_intr),
        .i_mad_pkt_rcvd_intr   (i_mad_pkt_rcvd_intr),
        .i_bypass_pkt_rcvd_intr(i_bypass_pkt_rcvd_intr),
        .i_rnr_nack_gen_intr   (i_rnr_nack_gen_intr),
        .i_ill_opc_in_sq_intr  (i_ill_opc_in_sq_intr),

        // RCV Q interrupt status (bitwise) 寄存器（IP核写，上位机读）
        .i_qp_pkt_rcvd_intr_wr_en  (i_qp_pkt_rcvd_intr_wr_en),    // 写使能
        .i_qp_pkt_rcvd_intr_wr_addr(i_qp_pkt_rcvd_intr_wr_addr),  // 写地址
        .i_qp_pkt_rcvd_intr_wr_data(i_qp_pkt_rcvd_intr_wr_data),  // 写数据
        // RCV Q interrupt status (bitwise) 寄存器（上位机写，IP核读）
        .o_rq_intr_sts_clr_wr_en   (o_rq_intr_sts_clr_wr_en),     // RQINTSTSn[31:0]: 该寄存器提供 QP 0 至 31 的 RQ 中断状态。位 [i] 提供 RQi 的中断状态，其中 i=0 至 31。
        .o_rq_intr_sts_clr_wr_addr (o_rq_intr_sts_clr_wr_addr),   // RQINTSTSn中的n，n=1，即 o_rq_intr_sts_clr_wr_addr = 0 说明 o_rq_intr_sts_clr_wr_en 是 RQINTSTS1 的值，表明对应 QP0-31 的 RQ 中断状态。

        // Interrupt Status 寄存器
        // 该状态寄存器提供断言中断的原因。
        .o_intr_clr_pkt_valdn_err  (o_intr_clr_pkt_valdn_err),    // INTSTS[0]: 传入数据包验证错误中断
        .o_intr_clr_wqe_cmpl       (o_intr_clr_wqe_cmpl),         // INTSTS[4]: WQE 完成中断（对 QPCONF[3] 位被设置的 QP 断言）
        .o_intr_clr_mad_pkt_rcvd   (o_intr_clr_mad_pkt_rcvd),     // INTSTS[1]: 收到传入的 MAD 数据包中断
        .o_intr_clr_bypass_pkt_rcvd(o_intr_clr_bypass_pkt_rcvd),  // INTSTS[2]: Reserved
        .o_intr_clr_rnr_nak_gen    (o_intr_clr_rnr_nak_gen),      // INTSTS[3]: RNR NACK 生成中断
        .o_intr_clr_ill_opc_in_sq  (o_intr_clr_ill_opc_in_sq),    // INTSTS[5]: 在发送队列中发布了非法操作代码中断
        .o_intr_clr_qp_pkt_rcvd    (o_intr_clr_qp_pkt_rcvd),      // INTSTS[6]: RQ 数据包接收中断（对于 QPCONF[2] 位被设置的 QP，该中断被断言）
        .o_intr_clr_fatal_err      (o_intr_clr_fatal_err),        // qp致命错误中断，只要有一个QP发生致命错误就会触发
        .o_intr_clr_cnp            (o_intr_clr_cnp),              // INTSTS[8]: 用于 CNP 调度的中断使能

        // Interrupt Enable 寄存器
        // 该寄存器为 XRNIC IP 产生的中断提供配置控制。
        .o_intr_en_pkt_valdn_err  (o_intr_en_pkt_valdn_err),    // INTEN[0]: 传入数据包验证错误中断启用
        .o_intr_en_wqe_cmpl       (o_intr_en_wqe_cmpl),         // INTEN[4]: WQE 完成中断启用
        .o_intr_en_mad_pkt_rcvd   (o_intr_en_mad_pkt_rcvd),     // INTEN[1]: 接收到 MAD 数据包中断启用
        .o_intr_en_bypass_pkt_rcvd(o_intr_en_bypass_pkt_rcvd),  // INTEN[2]: Reserved
        .o_intr_en_rnr_nak_gen    (o_intr_en_rnr_nak_gen),      // INTEN[3]: RNR NACK 产生的中断启用
        .o_intr_en_ill_opc_in_sq  (o_intr_en_ill_opc_in_sq),    // INTEN[5]: 在发送队列中发布了非法操作代码中断启用
        .o_intr_en_fatal_err      (o_intr_en_fatal_err),        // INTEN[7]: 收到致命错误中断启用
        .o_intr_en_cnp            (o_intr_en_cnp),              // INTEN[8]: 用于 CNP 调度的中断使能
        .o_intr_en_qp_pkt_rcvd    (o_intr_en_qp_pkt_rcvd),      // INTEN[6]: RQ 数据包接收中断启用

        // Completion Q interrupt status (bitwise) 寄存器（IP核写，上位机读）
        .i_qp_wq_cmpl_intr_or_reduce      (i_qp_wq_cmpl_intr_or_reduce),
        .i_qp_wq_cmpl_intr_wr_en          (i_qp_wq_cmpl_intr_wr_en),
        .i_qp_wq_cmpl_intr_wr_addr        (i_qp_wq_cmpl_intr_wr_addr),
        .i_qp_wq_cmpl_intr_wr_data        (i_qp_wq_cmpl_intr_wr_data),
        .i_qp_wq_cmpl_intr_wr_en_pending  (i_qp_wq_cmpl_intr_wr_en_pending),
        .i_qp_wq_cmpl_intr_wr_addr_pending(i_qp_wq_cmpl_intr_wr_addr_pending),
        // Completion Q interrupt status (bitwise) 寄存器（上位机写，IP核读）
        .o_cq_intr_sts_clr_wr_en          (o_cq_intr_sts_clr_wr_en),            // CQINTSTSn[31:0]: 该寄存器提供 QP 0 至 31 的 CQ 中断状态。位[i] 提供 CQi 的中断状态，其中 i=0 至 31。
        .o_cq_intr_sts_clr_wr_addr        (o_cq_intr_sts_clr_wr_addr),          // CQINTSTSn中的n，n=1说明 o_cq_intr_sts_clr_wr_en 是 CQINTSTS1 的值，表明对应 QP0-31 的 CQ 中断状态。

        // CNP Packed scheduled interrupt status (bitwise) 寄存器（IP核写，上位机读）
        .i_cnp_schedule_info_wr_en  (i_cnp_schedule_info_wr_en),
        .i_cnp_schedule_info_wr_addr(i_cnp_schedule_info_wr_addr),
        .i_cnp_schedule_info_wr_data(i_cnp_schedule_info_wr_data),
        .i_cnp_sched_intr_sts       (i_cnp_sched_intr_sts),         // INTSTS[8]: 用于 CNP 调度的中断使能
        // CNP Packed scheduled interrupt status (bitwise) 寄存器（上位机写，IP核读）
        .o_cnp_schedule_clr_wr_en   (o_cnp_schedule_clr_wr_en),     // CNPSCHDSTSnREG[31:0]: 位[i] 提供 QP 0 至 31 的 CNP 调度中断。
        .o_cnp_schedule_clr_wr_addr (o_cnp_schedule_clr_wr_addr),   // CNPSCHDSTSnREG中的n，n=1说明 o_cnp_schedule_clr_wr_en 是 CNPSCHDSTS1REG 的值，表明对应 QP0-31 的 CNP 调度中断。

        // XRNIC Advance Configuration (XRNICADCONF) 寄存器 
        .o_cnp_timeout_val(o_cnp_timeout_val),  // XRNICADCONF[11:8]: Reserved
        .o_ecn_timeout_val(o_ecn_timeout_val),  // XRNICADCONF[15:12]: Reserved

        // STATQPi[12]（IP核写，上位机读）
        .i_tx_path_congestion_valid(i_tx_path_congestion_valid),
        .i_tx_path_congestion_id   (i_tx_path_congestion_id),
        .i_tx_path_congestion_data (i_tx_path_congestion_data),

        .i_qp_fatal_err_valid(i_qp_fatal_err_valid),
        .i_qp_fatal_err_id   (i_qp_fatal_err_id),
        .i_qp_fatal_err_data (i_qp_fatal_err_data),

        .o_qp_clr_fatal_err_valid(o_qp_clr_fatal_err_valid),  // qp_conf_bram_wen == 1'b1时，o_qp_clr_fatal_err_valid <= 1'b1，保持两个周期
        .o_qp_clr_fatal_err_id   (o_qp_clr_fatal_err_id),     // qp idx
        .o_qp_clr_fatal_err_data (o_qp_clr_fatal_err_data),   // ~QPCONFi[0]: ~QP enable

        .o_qp_under_recovery_valid(o_qp_under_recovery_valid),  // qp_conf_bram_wen == 1'b1时，o_qp_under_recovery_valid <= 1'b1，保持两个周期
        .o_qp_under_recovery_id   (o_qp_under_recovery_id),     // qp idx
        .o_qp_under_recovery_data (o_qp_under_recovery_data),   // QPCONFi[6]: QP under recovery

        .i_rq_full_valid              (i_rq_full_valid),
        .i_rq_full_idx                (i_rq_full_idx),
        .i_rq_full_data               (i_rq_full_data),
        .i_rq_full_reset_valid        (i_rq_full_reset_valid),
        .i_rq_full_reset_idx          (i_rq_full_reset_idx),
        .i_rq_full_reset_data         (i_rq_full_reset_data),
        .i_osq_full_set_valid         (i_osq_full_set_valid),
        .i_osq_full_set_id            (i_osq_full_set_id),
        .i_osq_full_reset_valid       (i_osq_full_reset_valid),
        .i_osq_full_reset_id          (i_osq_full_reset_id),
        .i_osq_almost_full_set_valid  (i_osq_almost_full_set_valid),
        .i_osq_almost_full_set_id     (i_osq_almost_full_set_id),
        .i_osq_almost_full_reset_valid(i_osq_almost_full_reset_valid),
        .i_osq_almost_full_reset_id   (i_osq_almost_full_reset_id),
        .i_osq_empty_set_valid        (i_osq_empty_set_valid),
        .i_osq_empty_set_id           (i_osq_empty_set_id),
        .i_osq_empty_reset_valid      (i_osq_empty_reset_valid),
        .i_osq_empty_reset_id         (i_osq_empty_reset_id),
        .i_qp_retried_set_valid       (i_qp_retried_set_valid),
        .i_qp_retried_set_id          (i_qp_retried_set_id),
        .i_qp_retried_reset_valid     (i_qp_retried_reset_valid),
        .i_qp_retried_reset_id        (i_qp_retried_reset_id),
        .i_stat_retry_cnt             (i_stat_retry_cnt),
        .i_min_ipg_stat               (i_min_ipg_stat),
        .i_ipg_0_4_cnt                (i_ipg_0_4_cnt),
        .i_ipg_5_9_cnt                (i_ipg_5_9_cnt),
        .i_ipg_10_14_cnt              (i_ipg_10_14_cnt),
        .i_ipg_15_19_cnt              (i_ipg_15_19_cnt),

        .o_rx_pkt_hndl_dbg_ctrl(o_rx_pkt_hndl_dbg_ctrl),

        // Status Message sequence number QPi 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
        .i_qp_stat_msn_idx  (i_qp_stat_msn_idx),   // addrb
        .i_qp_stat_msn      (i_qp_stat_msn),       // dinb, STATMSN[23:0]: 当前预期收到的 MSN
        .o_qp_stat_msn      (o_qp_stat_msn),       // doutb, STATMSN[23:0]: 当前预期收到的 MSN
        .i_qp_stat_msn_req  (i_qp_stat_msn_req),
        .i_qp_stat_msn_wrn  (i_qp_stat_msn_wrn),   // web
        .o_qp_stat_msn_valid(o_qp_stat_msn_valid), // valid <= req

        // Status Sender sequence number QPi 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
        .i_qp_stat_ssn_idx  ((qp_stat_ssn_req_retry | qp_stat_ssn_wrn_retry) ? qp_stat_ssn_idx_retry : i_qp_stat_ssn_idx),  // addrb
        .i_qp_stat_ssn      (qp_stat_ssn_wrn_retry ? qp_stat_ssn_retry : i_qp_stat_ssn),                                    // STATSSNi[23:0]: 当前发出的 SSN（与发出的 MSN 相同）
        .o_qp_stat_ssn      (o_qp_stat_ssn),                                                                                // STATSSNi[23:0]: 当前发出的 SSN（与发出的 MSN 相同）
        .i_qp_stat_ssn_req  (i_qp_stat_ssn_req | qp_stat_ssn_req_retry),                                                    // 可在 WQE 停止时进行 OR 运算
        .i_qp_stat_ssn_wrn  (i_qp_stat_ssn_wrn | qp_stat_ssn_wrn_retry),                                                    // web, 可在 WQE 停止时进行 OR 运算
        .o_qp_stat_ssn_valid(o_qp_stat_ssn_valid),                                                                          // valid <= req

        // Timeout Configuration Register for QPi (TIMEOUTCONFi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成(其中 i = 2 至 C_NUM_QP）。它提供 QPi 的超时配置。QP1 不存在该寄存器。
        .i_qp_timeout_idx(i_qp_conf_idx),  // addrb
        .o_qp_timeout    (o_qp_timeout),   // [20:16]: 传出数据包的 RNR NACK 超时值。[13:11]: 最大 RNR 重试次数；[10:8]: 最大重试次数；[5:0]: 超时值；

        // Status RQ Producer index DB QPi (STATRQPIDBi) 寄存器 (Read only)
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。
        .i_qp_rq_pi_db_idx  (i_qp_rq_pi_db_idx),   // addrb
        .i_qp_rq_pi_db      (i_qp_rq_pi_db),       // [15:0]: RCV Q 生产者索引 DB
        .o_qp_rq_pi_db      (o_qp_rq_pi_db),       // [15:0]: RCV Q 生产者索引 DB
        .i_qp_rq_pi_db_req  (i_qp_rq_pi_db_req),
        .i_qp_rq_pi_db_wrn  (i_qp_rq_pi_db_wrn),   // web
        .o_qp_rq_pi_db_valid(o_qp_rq_pi_db_valid), // valid <= req

        // Status of WQEs posted to QPi (STATWQEi) 寄存器 (Read only)
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。
        .i_qp_stat_wqe_cnt      (new_qp_stat_wqe_cnt),      // [15:0]: QP MGR 为该 QP 推送的 WQE 计数
        .i_qp_stat_wqe_cnt_wen  (update_qp_stat_wqe_cnt),   // web
        .i_qp_stat_wqe_cnt_idx  (qp_sq_arbitrated_idx_ff),  // addrb
        .o_qp_stat_wqe_cnt_valid(qp_stat_wqe_cnt_valid),    // valid <= req
        .i_qp_stat_wqe_cnt_rdreq(qp_stat_wqe_cnt_rdreq),
        .o_qp_stat_wqe_cnt      (qp_stat_wqe_cnt),          // [15:0]: QP MGR 为该 QP 推送的 WQE 计数

        .i_qp_ret_sq_psn_idx      (i_qp_stat_ret_sq_psn_req ? i_qp_stat_ret_sq_psn_idx : qp_stat_ret_sq_psn_idx_int),
        .i_qp_ret_sq_psn          (qp_stat_ret_sq_psn_int),
        .o_qp_ret_sq_psn          (o_qp_stat_ret_sq_psn),
        .i_qp_ret_sq_psn_req      (i_qp_stat_ret_sq_psn_req),
        .i_qp_ret_sq_psn_wrn      (i_qp_stat_ret_sq_psn_req ? 1'b0 : qp_stat_ret_sq_psn_wen_int),
        .o_qp_ret_sq_psn_valid    (o_qp_stat_ret_sq_psn_valid),
        .o_qp_ret_sq_psn_valid_int(qp_stat_ret_sq_psn_valid_int),                                                      // internal from QPM

        // XRNIC Configuration (XRNICCONF) 寄存器
        // 该寄存器为 XRNIC IP 提供基本的全局（非 QP 专用）配置控制。
        .o_xrnic_en          (o_xrnic_en),            // XRNICCONF[0]: XRNIC 启用
        .o_tx_ack_gen        (o_tx_ack_gen),          // XRNICCONF[4:3]: TX ACK 生成。
        .o_err_buf_en        (o_err_buf_en),          // XRNICCONF[5]: 启用错误缓冲区
        .o_flow_credits      (o_flow_credits),        // XRNICCONF[7:6]: Reserved
        .o_xrnic_udp_src_port(o_xrnic_udp_src_port),  // XRNICCONF[23:8]: 外发数据包的 UDP 源端口，即 XRNIC UDP 输出端口
        .o_depkt_bypass_en   (o_depkt_bypass_en),     // XRNICCONF[2]: Reserved

        // XRNIC Advance Configuration (XRNICADCONF) 寄存器
        .o_xrnic_adv_conf_sw_override        (sw_override_en),                        // XRNICADCONF[0]: 允许 SW 写以下只读寄存器 - CQHEADi, STATCURSQPTRi 和 STATRQPIDBi (其中 i 为 QP 索引)
        .o_xrnic_adv_conf_sw_override_qpid   (sw_override_qpid),                      // XRNICADCONF[30:20]: 软件覆盖 QP 编号
        .i_sw_override_per_qp_en             ({sw_override_en, sw_override_qpid}),
        .o_xrnic_adv_conf_errbuf_overwr_en   (o_xrnic_adv_conf_errbuf_overwr_en),     // XRNICADCONF[1]: Reserved
        .o_xrnic_adv_conf_retry_cnt_fatal_dis(o_xrnic_adv_conf_retry_cnt_fatal_dis),  // XRNICADCONF[2]: retry_cnt_fatal_dis
        .o_xrnic_adv_base_cnt                (o_xrnic_adv_base_cnt),                  // XRNICADCONF[19:16]: Base count width

        .o_num_qp_en      (o_num_qp_en),        // XRNIC_CONF_QP_EN[10:0]: 启用的 QP 数量；该寄存器包含多少个 QP 将被激活的信息。
        .o_mac_xrnic_addr (o_mac_xrnic_addr),   // MACXADD[47:0]: 这是本地 (XRNIC) 设备的 MAC 地址，应在 XRNICCONF[0] 设置为 1 之前配置。
        .o_ipv4_xrnic_addr(o_ipv4_xrnic_addr),  // IPv4XADD[31:0]: 本地 (XRNIC) 设备的 IPv4 地址。
        .o_ipv6_xrnic_addr(o_ipv6_xrnic_addr),  // IPv6XADD[127:0]: 本地（ERNIC）设备的 IPv6 地址。应在 XRNICCONF[0] 设置为 1 之前配置。

        // bypass_buf(no use)
        .o_bypass_buf_ba         (o_bypass_buf_ba),
        .o_bypass_buf_sz_num_bufs(o_bypass_buf_sz_num_bufs),
        .o_bypass_buf_sz_buf_sz  (o_bypass_buf_sz_buf_sz),
        .i_bypass_buf_wrptr      (i_bypass_buf_wrptr),

        // data_buf: 数据缓冲区
        // XRNIC 使用这些数据缓冲区保存所有传出的 RDMA WRITE 数据，直到远程主机确认为止。如果发生重传，则从这些缓冲区提取重试数据。
        .o_data_buf_ba         (o_data_buf_ba),           // DATBUFBA[63:0]: 数据缓冲区基地址
        .o_data_buf_sz_num_bufs(o_data_buf_sz_num_bufs),  // DATBUFSZ[15:0]: 数据缓冲区数量
        .o_data_buf_sz_buf_sz  (o_data_buf_sz_buf_sz),    // DATBUFSZ[31:16]: 数据缓冲区大小（字节

        // Connect IP configuration (CON_IO_CONF) 寄存器
        // 当硬件握手刚刚启用时，该寄存器用于按门铃。
        .o_connect_io_qp_rq_pi_db_wptr(o_connect_io_qp_rq_pi_db_wptr),  // CON_IO_CONF[12:0]: Reserved
        .o_connect_io_qpid            (o_connect_io_qpid),              // CON_IO_CONF[26:16]: Queue pair ID
        .i_connect_io_qp_rq_pi_db_rdy (i_connect_io_qp_rq_pi_db_rdy),   // 就绪表示可以接受另一个; CON_IO_CONF[31]:连接 IO 就绪指示。该位只读，设置为 1'b1 时表示连接 IO 配置已完成，可以接受另一个 QPID。
        //.o_connect_io_residual_rq     (o_connect_io_residual_rq),
        .o_rq_pi_db_hw_hndshk         (o_rq_pi_db_hw_hndshk),           // ?
        .o_hw_hndshk_disable_to_0     (o_hw_hndshk_disable_to_0),       // ?

        // rq_err_pkt_buf: 错误缓冲区
        // XRNIC IP 会根据未通过验证的数据包更新这些缓冲区。通过向 XRNICCONF[5]写 1，可启用向该缓冲区写入所有验证错误信息。如果禁用该位，则只有导致 QP 进入 FATAL 状态的数据包才会写入错误缓冲区。
        .o_rq_err_pkt_buf_ba         (o_rq_err_pkt_buf_ba),           // ERRBUFBA[63:0]: 错误缓冲区基地址
        // 该寄存器提供错误缓冲区的大小
        .o_rq_err_pkt_buf_sz_num_bufs(o_rq_err_pkt_buf_sz_num_bufs),  // ERRBUFSZ[15:0]: 错误缓冲区数量
        .o_rq_err_pkt_buf_sz_buf_sz  (o_rq_err_pkt_buf_sz_buf_sz),    // ERRBUFSZ[31:16]: 每个错误缓冲区的大小
        .i_rq_err_pkt_buf_wrptr      (i_rq_err_pkt_buf_wrptr),        // 错误缓冲区写指针，指向下一个可用的错误缓冲区

        // resp_err_pkt_buf: 响应错误缓冲区
        // XRNIC 使用这些错误缓冲区保存所有错误响应错误数据包的报头信息。
        .o_resp_err_pkt_buf_ba         (o_resp_err_pkt_buf_ba),           // RESPERRPKTBA[63:0]: resp_err_pkt_buf 基地址
        // 该寄存器提供每个条目大小和响应错误缓冲区深度的信息
        .o_resp_err_pkt_buf_sz_num_bufs(o_resp_err_pkt_buf_sz_num_bufs),  // RESPERRSZ[15:0]: 响应错误缓冲区条目数
        .o_resp_err_pkt_buf_sz_buf_sz  (o_resp_err_pkt_buf_sz_buf_sz),    // RESPERRSZ[31:16]: 响应错误缓冲区各条目大小
        .i_resp_err_pkt_buf_wrptr      (i_resp_err_pkt_buf_wrptr),        // 响应错误缓冲区写指针，指向下一个可用的响应错误缓冲区

        .i_inc_rresp_pkt_cnt  (i_inc_rresp_pkt_cnt),
        .i_inc_send_pkt_cnt   (i_inc_send_pkt_cnt),
        .i_inc_ack_pkt_cnt    (i_inc_ack_pkt_cnt),
        .i_inc_mad_pkt_cnt    (i_inc_mad_pkt_cnt),
        .i_inc_inv_pkt_cnt    (i_inc_inv_pkt_cnt),
        .i_inc_dup_pkt_cnt    (i_inc_dup_pkt_cnt),
        .i_inc_all_dropped_cnt(i_inc_all_dropped_cnt),
        .i_inc_nack_cnt       (i_inc_nack_cnt),
        .i_inc_cnp_pkt_cnt    (i_inc_cnp_pkt_cnt),

        .i_out_rdwr_pkt_cnt   (i_out_rdwr_pkt_cnt),
        .i_out_send_pkt_cnt   (i_out_send_pkt_cnt),
        .i_out_mad_pkt_cnt    (i_out_mad_pkt_cnt),
        .i_out_rd_resp_pkt_cnt(i_out_rd_resp_pkt_cnt),
        .i_out_ack_pkt_cnt    (i_out_ack_pkt_cnt),
        .i_out_nack_cnt       (i_out_nack_cnt),
        .i_resp_hndler_sts    (i_resp_hndler_sts),
        .i_all_rd_req_cnt     (i_all_rd_req_cnt),
        .i_good_rd_req_cnt    (i_good_rd_req_cnt),
        .i_dup_rd_req_cnt     (i_dup_rd_req_cnt),

        .o_xrnic_conf_ipver(o_xrnic_conf_ipver),  // XRNICCONF[1]: Reserved
        .qp_enable_valid   (o_qp_enable_valid),   // qp_conf_bram_wen == 1'b1时，o_qp_enable_valid <= 1'b1，保持一个周期
        .qp_enable_id      (o_qp_enable_id),      // qpid
        .qp_enable_data    (o_qp_enable_data),    // QPCONFi[0]: QP enable

        // XRNIC_PAUSE_CONF 寄存器
        // 该寄存器提供 PFC 配置。
        .pfc_en_roce         (pfc_en_roce),          // XRNIC_PAUSE_CONF[0]: 暂停启用 RoCE
        .pfc_en_non_roce     (pfc_en_non_roce),      // XRNIC_PAUSE_CONF[1]: 暂停启用非 RoCE
        .roce_priority       (roce_priority),        // XRNIC_PAUSE_CONF[7:4]: RoCE 的暂停优先级（二进制编码）
        .non_roce_priority   (non_roce_priority),    // XRNIC_PAUSE_CONF[11:8]: 非 RoCE（二进制编码）的暂停优先级
        .disable_priority_chk(disable_priority_chk), // XRNIC_PAUSE_CONF[13]: 禁用优先级检查（仅适用于 TX 数据路径）

        // XRNIC_BUF_THRESHOLD_ROCE 寄存器
        // 该寄存器为 RoCE 流量提供 XON 和 XOFF 配置。
        .roce_pause_on_offset (roce_pause_on_offset),  // XRNIC_BUF_THRESHOLD_ROCE[15:0]: RoCE 的 XON 值
        .roce_pause_off_offset(roce_pause_off_offset), // XRNIC_BUF_THRESHOLD_ROCE[31:16]: RoCE 的 XOFF 值

        // XRNIC_BUF_THRESHOLD_NON_ROCE 寄存器
        // 该寄存器为非 RoCE 流量提供 XON 和 XOFF 配置。
        .non_roce_pause_on_offset (non_roce_pause_on_offset),  // XRNIC_BUF_THRESHOLD_NON_ROCE[15:0]: 非 RoCE 的 XON 值
        .non_roce_pause_off_offset(non_roce_pause_off_offset), // XRNIC_BUF_THRESHOLD_NON_ROCE[31:16]: 非 RoCE 的 XOFF 值

        //.o_sq_full                    (),
        // sq_empty_*报告某一个 QP 的 SQ 空状态发生变化
        .o_sq_empty_valid          (sq_empty_valid),      // sq_empty_*有效信号
        .o_sq_empty_id             (sq_empty_id),         // qpid
        .o_sq_empty_data           (sq_empty_data),       // 1-空，0-非空
        .dbg_sq_empty_ff_and_reduce(sq_empty_and_reduce), // Debug registers, 所有 QP 的 SQ 当前是否都为空？

        .core_clk (core_clk),  // Module Clock  
        .core_rstn(core_rstn), // Active high core reset

        .i_waiting_for_rq_update         (i_waiting_for_rq_update),          // 等待更新 RQ 
        .i_waiting_for_rq_update_on_q_num(i_waiting_for_rq_update_on_q_num), // 等待更新 RQ 的 QP 编号

        .i_qp_conf_req_wqe_proc   (i_qp_conf_req_wqe_proc),     // wqe_proc模块 获取 QP 配置请求
        .i_qp_conf_req_resp_hndl  (i_qp_conf_req_resp_hndl),    // resp_handler模块 获取 QP 配置请求
        .i_qp_conf_req_rx_pkt     (i_qp_conf_req_rx_pkt),       // rx_pkt_handler模块 获取 QP 配置请求
        .o_qp_conf_valid_wqe_proc (o_qp_conf_valid_wqe_proc),   // 响应 wqe_proc模块 的获取 QP 配置请求
        .o_qp_conf_valid_resp_hndl(o_qp_conf_valid_resp_hndl),  // 响应 resp_handler模块 的获取 QP 配置请求
        .o_qp_conf_valid_rx_pkt   (o_qp_conf_valid_rx_pkt),     // 响应 rx_pkt_handler模块 的获取 QP 配置请求
        // QP Configuration QPi (QPCONFi) 寄存器
        // 该寄存器定义 QP 的基本配置。该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。每个 QP 的寄存器偏移量为 0x18_0000、0x18_0004，随后的偏移量依此类推。
        .o_qp_conf                (o_qp_conf),
        .i_qp_conf_idx            (i_qp_conf_idx),              // 读取 Qp 配置的索引

        .i_qp_cfg_retry_cnt_rd_req(i_qp_cfg_retry_cnt_rd_req),  // 新添加了 QP_fatal 情况?
        .o_qp_cfg_retry_cnt_rd_rdy(o_qp_cfg_retry_cnt_rd_rdy),  // 新添加了 QP_fatal 情况?

        // QP configuration QPi (QPCONFi) 寄存器复制寄存器
        .o_qp_conf_replica      (o_qp_conf_replica),       // QP configuration REPLICA
        .i_qp_conf_replica_idx  (i_qp_conf_replica_idx),   // Index for reading Qp configuration REPLICA
        .i_qp_conf_replica_req  (i_qp_conf_replica_req),
        .o_qp_conf_replica_valid(o_qp_conf_replica_valid), // valid <= req

        // QP advanced configuration QPi (QPADVCONFi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
        .i_qp_adv_conf_req  (i_qp_adv_conf_req),
        .o_qp_adv_conf_valid(o_qp_adv_conf_valid),  // valid <= req
        .o_qp_adv_conf      (o_qp_adv_conf),        // QP advanced configuration; [5:0]: 流量类别dscp-（保持默认重置值）；[15:8]: 生存时间ttl；[31:16]: 分区密钥pkey
        .i_qp_adv_conf_idx  (i_qp_adv_conf_idx),    // Index for reading Qp advanced configuration

        // RCV Q Buffer base  address QPi (RQBAi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供 RDMA 接收队列缓冲区的基地址。
        .i_qp_rq_ba_req  (i_qp_rq_ba_req),
        .o_qp_rq_ba_valid(o_qp_rq_ba_valid),  // valid <= req
        .o_qp_rq_ba      (o_qp_rq_ba),        // RQBAi[63:8]: 接收 Q 缓冲区基地址地址（256 B 对齐）。256 B 是总大小（单个 RQ 缓冲区元素大小）* rq_depth
        .i_qp_rq_ba_idx  (i_qp_rq_ba_idx),    // Index for reading Qp RQ Buffer BA

        // SEND Q base address QPi (SQBAi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供 RDMA 发送队列缓冲区的基地址。
        .i_qp_sq_ba_req_int  (qp_sq_ba_req_int),                                          // qp_mgr_cache模块用
        .i_qp_sq_ba_req      (i_qp_sq_ba_req),
        .o_qp_sq_ba_valid_int(qp_sq_ba_valid_int),                                        // qp_mgr_cache模块用
        .o_qp_sq_ba_valid    (o_qp_sq_ba_valid),
        .o_qp_sq_ba          (qp_sq_ba),                                                  // SQBAi[63:5]: SQ 基地址（32 B 对齐）
        .i_qp_sq_ba_idx      (i_qp_sq_ba_req ? i_qp_sq_ba_idx : qp_sq_arbitrated_idx_ff), // Index for reading Qp SQ BA

        // CQ base address QPi (CQBAi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（i = 1 至 C_NUM_QP）生成。该寄存器提供 RDMA 完成队列缓冲区的基地址。
        .i_qp_cq_ba_req  (i_qp_cq_ba_req),
        .o_qp_cq_ba_valid(o_qp_cq_ba_valid),
        .o_qp_cq_ba      (o_qp_cq_ba),        // CQBAi[63:5]: 发送 CQ 基地址（32 B 对齐）
        .i_qp_cq_ba_idx  (i_qp_cq_ba_idx),    // Index for reading Qp SQ BA

        // RCV Q Write pointer DB address QPi (RQWPTRDBADDi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供接收队列门铃寄存器的地址。在接收到新的 RDMA SEND 数据包时，XRNIC IP 会更新寄存器所指向地址中的 RQ 门铃值。
        .i_qp_rq_wrptrdb_add_req  (i_qp_rq_wrptrdb_add_req),
        .o_qp_rq_wrptrdb_add_valid(o_qp_rq_wrptrdb_add_valid),
        .o_qp_rq_wrptrdb_add      (o_qp_rq_wrptrdb_add),        // RQWPTRDBADDi[63:0]: RCV Q 写指针门铃地址
        .i_qp_rq_wrptrdb_add_idx  (i_qp_rq_wrptrdb_add_idx),    // Index for reading Qp SQ BA

        // CQ DB address QPi (CQDBADDi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成，提供完成队列门铃寄存器的地址。完成一个新的 SEND 工作队列条目后，XRNIC IP 将更新该寄存器所指向地址中的 CQ 门铃值。
        .i_qp_sq_cmpldb_add_req  (i_qp_sq_cmpldb_add_req),
        .o_qp_sq_cmpldb_add_valid(o_qp_sq_cmpldb_add_valid),
        .o_qp_sq_cmpldb_add      (o_qp_sq_cmpldb_add),        // CQDBADDi[63:0]: 发送 CQ 门铃地址
        .i_qp_sq_cmpldb_add_idx  (i_qp_sq_cmpldb_add_idx),    // Index for reading SQ Completion DB address

        // CQ head pointer QPi (CQHEADi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。该状态寄存器给出发送完成队列的门铃值，并提供已完成发送 WQE 的信息。这是 XRNIC IP 写入 CQDBADDi 寄存器所指向地址的门铃值。
        .i_qp_cq_hdptr_req        (i_qp_cq_hdptr_req),                                                 // ORing as resp_handler halted
        .i_qp_cq_hdptr_retry_req  (qp_cq_hdptr_req_retry),                                             // ORing as resp_handler halted
        .o_qp_cq_hdptr_valid      (o_qp_cq_hdptr_valid),
        .o_qp_cq_hdptr_valid_retry(qp_cq_hdptr_valid_retry),
        .o_qp_cq_hdptr            (qp_cq_hdptr_from_config),                                           // CQHEADi[15:0]: CQ head pointer
        .i_qp_cq_hdptr            (i_qp_cq_hdptr),                                                     // CQHEADi[15:0]: CQ head pointer
        .i_qp_cq_hdptr_wrn        (i_qp_cq_hdptr_wrn),                                                 // web
        .i_qp_cq_hdptr_idx        (qp_cq_hdptr_req_retry ? qp_cq_hdptr_idx_retry : i_qp_cq_hdptr_idx), // Index for reading CQ head pointer

        // RQ Consumer Index QPi (RQCIi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。目标应用在消耗一个新的接收队列条目时更新该寄存器。一旦消耗完毕，XRNIC IP 就可以重新写入 RQ 条目。RQCI 索引也可通过侧带接口更新。
        .i_qp_rq_cidb_req  (i_qp_rq_cidb_req),
        .o_qp_rq_cidb_valid(o_qp_rq_cidb_valid),  // valid <= req
        .o_qp_rq_cidb      (o_qp_rq_cidb),        // RQCIi[15:0]: RCV Q 消费者索引门铃
        .i_qp_rq_cidb_idx  (i_qp_rq_cidb_idx),    // Index for reading RQ CI DB

        // SQ Producer index QPi (SQPIi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。目标应用在发布新的发送工作队列条目时更新该寄存器。SQPI 也可通过侧带接口更新。发送队列是一个循环缓冲区。相应 QP 的 CQHEAD 寄存器提供有关已完成的工作队列条目的信息，目标应用程序可对其进行重写。
        .o_qp_sq_pidb      (qp_sq_pidb),               // SQPIi[15:0]: 发送 Q 生产者索引门铃
        .i_qp_sq_pidb_idx  (qp_sq_arbitrated_idx_ff),  // Index for reading SQ PI DB
        .i_qp_sq_pidb_req  (qp_sq_pidb_req),
        .o_qp_sq_pidb_valid(qp_sq_pidb_valid),         // valid <= req


        // HW handshake for SQ PI doorbell
        .i_qp_sq_pidb_hndshk         (i_qp_sq_pidb_hndshk),
        .i_qp_sq_pidb_wr_addr_hndshk (i_qp_sq_pidb_wr_addr_hndshk),
        .i_qp_sq_pidb_wr_valid_hndshk(i_qp_sq_pidb_wr_valid_hndshk),
        .o_qp_sq_pidb_wr_rdy         (o_qp_sq_pidb_wr_rdy),
        //-------------------------------  
        //  .i_qp_sq_pidb_hndshk          ('b0),
        //  .i_qp_sq_pidb_idx_hndshk      ('b0),
        //  .i_qp_sq_pidb_wr_valid_hndshk ('b0),
        //  .o_qp_sq_pidb_wr_rdy          (o_qp_sq_pidb_wr_rdy),

        // HW handshake for RQ CI doorbell
        .i_qp_rq_cidb_hndshk         (i_qp_rq_cidb_hndshk),
        .i_qp_rq_cidb_wr_addr_hndshk (i_qp_rq_cidb_wr_addr_hndshk),
        .i_qp_rq_cidb_wr_valid_hndshk(i_qp_rq_cidb_wr_valid_hndshk),
        .o_qp_rq_cidb_wr_rdy         (o_qp_rq_cidb_wr_rdy),

        // Q Depth QPi (QDEPTHi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器定义发送、完成和接收队列的队列深度。
        // RQ
        .i_qp_rq_depth_req  (i_qp_rq_depth_req),
        .o_qp_rq_depth_valid(o_qp_rq_depth_valid),      // valid <= req
        .o_qp_rq_depth      (o_qp_rq_depth),            // QDEPTHi[31:16]: RQ 深度
        .i_qp_rq_depth_idx  (i_qp_rq_depth_idx),
        // SQ
        .i_qp_sq_depth_req  (qp_sq_depth_req),
        .o_qp_sq_depth_valid(qp_sq_depth_valid),        // valid <= req
        .o_qp_sq_depth      (qp_sq_depth),              // QDEPTHi[15:0]: SQ 深度 (Send CQ will of the same depth)
        .i_qp_sq_depth_idx  (qp_sq_arbitrated_idx_ff),
        // CQ
        .i_qp_cq_depth_req  (i_qp_cq_depth_req),
        .o_qp_cq_depth_valid(o_qp_cq_depth_valid),      // valid <= req
        .o_qp_cq_depth_rdy  (o_qp_cq_depth_rdy),        // no use
        .o_qp_cq_depth      (o_qp_cq_depth),            // QDEPTHi[15:0]: CQ 深度
        .i_qp_cq_depth_idx  (i_qp_cq_depth_idx),

        // SEND Q PSN for QPi (SQPSNi) 寄存器
        // 该寄存器在连接时由 SW 初始化。此后，硬件会为每个发出的数据包更新该寄存器，而软件则不应更新。该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
        .i_qp_sq_psn_req        (i_qp_sq_psn_req),                                      // highest priority from RX pkt handler
        .i_qp_sq_psn_wqe_req    (i_qp_sq_psn_wqe_req),                                  // can be ORed as WQE is halted
        .i_qp_sq_psn_retry_req  (qp_sq_psn_req_retry),                                  // can be ORed as WQE is halted
        .o_qp_sq_psn_valid      (o_qp_sq_psn_valid),
        .o_qp_sq_psn_wqe_valid  (o_qp_sq_psn_wqe_valid),
        .o_qp_sq_psn_resp_valid (o_qp_sq_psn_resp_valid),
        .o_qp_sq_psn_retry_valid(qp_sq_psn_retry_valid),
        .o_qp_sq_psn            (o_qp_sq_psn),                                          // SQPSNi[23:0]: 发送 Q PSN
        .i_qp_sq_psn            (qp_sq_psn_wrn_retry ? qp_sq_psn_retry : i_qp_sq_psn),  // SQPSNi[23:0]: 发送 Q PSN
        .i_qp_sq_psn_wrn        (i_qp_sq_psn_wrn),                                      // can be ORed as WQE is halted
        .i_qp_sq_psn_retry_wrn  (qp_sq_psn_wrn_retry),                                  // can be ORed as WQE is halted
        .i_qp_sq_psn_idx        (i_qp_sq_psn_idx),                                      // Index for reading PSN 
        .i_qp_sq_psn_wqe_idx    (i_qp_sq_psn_wqe_idx),                                  // Index for reading PSN 
        .i_qp_sq_psn_retry_idx  (qp_sq_psn_idx_retry),                                  // Index for reading PSN 
        .i_qp_sq_psn_resp_idx   (i_qp_sq_psn_resp_idx),                                 // Index for reading PSN 

        // Last RQ req for QPi (LSTRQREQi) 寄存器
        // 该寄存器提供最后一个接收到的 RQ 数据包的详细信息。该寄存器根据参数 C_NUM_QP（其中 i = 2 至 C_NUM_QP）生成。QP1 不存在该寄存器。
        .o_qp_last_rq      (o_qp_last_rq),       // LSTRQREQi[31:24]: RCV Q opcode; LSTRQREQi[23:0]: RCV Q PSN
        .i_qp_last_rq_idx  (i_qp_last_rq_idx),
        .i_qp_last_rq      (i_qp_last_rq),       // LSTRQREQi[31:24]: RCV Q opcode; LSTRQREQi[23:0]: RCV Q PSN
        .i_qp_last_rq_req  (i_qp_last_rq_req),
        .i_qp_last_rq_wrn  (i_qp_last_rq_wrn),   // web
        .o_qp_last_rq_valid(o_qp_last_rq_valid), // valid <= req

        // Protection domain number 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 2 至 C_NUM_QP）生成。该寄存器为 24 位，包含分配给 QP 的 PD 编号。
        // tip: 每个 QP 只能关联一个内存区域。
        .o_qp_pd_num      (o_qp_pd_num),       // QP 索引 关联的 PD 号
        .i_qp_pd_num_idx  (i_qp_pd_num_idx),   // QP 索引
        .i_qp_pd_num      (i_qp_pd_num),       // QP 索引 关联的 PD 号
        .i_qp_pd_num_req  (i_qp_pd_num_req),   // no use
        .i_qp_pd_num_wrn  (i_qp_pd_num_wrn),   // web
        .o_qp_pd_num_valid(o_qp_pd_num_valid), // no use

        // Protection Domain Number (PDPDNUM) 寄存器
        // 内存区域表中的保护域编号
        .o_pd_pd_num      (o_pd_pd_num),       // PD 号
        .i_pd_pd_num_idx  (i_pd_pd_num_idx),   // PD 索引
        .i_pd_pd_num      (i_pd_pd_num),       // PD 号
        .i_pd_pd_num_req  (i_pd_pd_num_req),   // no use
        .i_pd_pd_num_wrn  (i_pd_pd_num_wrn),   // web
        .o_pd_pd_num_valid(o_pd_pd_num_valid), // no use

        // Virtual address LSB (VIRTADDRLSB) 寄存器
        // 该寄存器提供缓冲区基地址 LSB 的虚拟地址。
        .i_pd_virt_addr_0_wrn  (1'b0),
        .o_pd_virt_addr_0      (o_pd_virt_addr_0),
        .i_pd_virt_addr_0      (32'b0),
        .o_pd_virt_addr_0_valid(o_pd_virt_addr_0_valid),
        .i_pd_virt_addr_0_idx  (i_pd_virt_addr_0_idx),
        .i_pd_virt_addr_0_req  (i_pd_virt_addr_0_req),

        // Virtual address MSB (VIRTADDRMSB) 寄存器
        // 该寄存器提供缓冲区基地址 MSB 的虚拟地址。
        .i_pd_virt_addr_1_wrn  (1'b0),
        .o_pd_virt_addr_1      (o_pd_virt_addr_1),
        .i_pd_virt_addr_1      (32'b0),
        .o_pd_virt_addr_1_valid(o_pd_virt_addr_1_valid),
        .i_pd_virt_addr_1_idx  (i_pd_virt_addr_1_idx),
        .i_pd_virt_addr_1_req  (i_pd_virt_addr_1_req),

        // Buffer Base address LSB (BUFBASEADDRLSB) 寄存器
        // 该寄存器提供写入/读取缓冲区基地址 LSB。
        .i_pd_buf_base_addr_0_wrn  (1'b0),
        .o_pd_buf_base_addr_0      (o_pd_buf_base_addr_0),
        .i_pd_buf_base_addr_0      (32'b0),
        .o_pd_buf_base_addr_0_valid(o_pd_buf_base_addr_0_valid),
        .i_pd_buf_base_addr_0_idx  (i_pd_buf_base_addr_0_idx),
        .i_pd_buf_base_addr_0_req  (i_pd_buf_base_addr_0_req),

        // Buffer base address MSB (BUFBASEADDRMSB) 寄存器
        // 该寄存器提供写入/读取缓冲区基地址 MSB。
        .i_pd_buf_base_addr_1_wrn  (1'b0),
        .o_pd_buf_base_addr_1      (o_pd_buf_base_addr_1),
        .i_pd_buf_base_addr_1      (32'b0),
        .o_pd_buf_base_addr_1_valid(o_pd_buf_base_addr_1_valid),
        .i_pd_buf_base_addr_1_idx  (i_pd_buf_base_addr_1_idx),
        .i_pd_buf_base_addr_1_req  (i_pd_buf_base_addr_1_req),

        // Buffer R_Key (BUFRKEY) 寄存器
        // 该寄存器提供写入/读取缓冲区 R_KEY 寄存器。
        .i_pd_buf_r_key_wrn  (1'b0),
        .o_pd_buf_r_key      (o_pd_buf_r_key),
        .i_pd_buf_r_key      (8'd0),
        .o_pd_buf_r_key_valid(o_pd_buf_r_key_valid),
        .i_pd_buf_r_key_idx  (i_pd_buf_r_key_idx),
        .i_pd_buf_r_key_req  (i_pd_buf_r_key_req),

        // Write/Read buffer length (WRRDBUFLEN) 寄存器
        // 该寄存器提供写入/读取缓冲区长度 LSB 寄存器。
        .i_pd_w_r_buf_len_wrn  (1'b0),
        .o_pd_w_r_buf_len      (o_pd_w_r_buf_len),        // {ACCESSDESC[31:16], WRRDBUFLEN[31:0]}
        .i_pd_w_r_buf_len      (48'b0),
        .o_pd_w_r_buf_len_valid(o_pd_w_r_buf_len_valid),
        .i_pd_w_r_buf_len_idx  (i_pd_w_r_buf_len_idx),
        .i_pd_w_r_buf_len_req  (i_pd_w_r_buf_len_req),

        // Access Description (ACCESSDESC) 寄存器
        .i_pd_access_type_wrn  (1'b0),
        .o_pd_access_type      (o_pd_access_type),        // ACCESSDESC[3:0]: 访问类型; 4'b0000: READ Only; 4'b0001: Write Only; 4'b0010: Read and Write; Other values: Not supported。
        .i_pd_access_type      (4'd0),
        .o_pd_access_type_valid(o_pd_access_type_valid),
        .i_pd_access_type_idx  (i_pd_access_type_idx),
        .i_pd_access_type_req  (i_pd_access_type_req),

        //.i_qp_rnr_nck_tval_curr_retry_req (i_qp_rnr_nck_tval_curr_retry_req),
        //.i_qp_rnr_nck_tval_curr_retry_wrn (i_qp_rnr_nck_tval_curr_retry_wrn),
        //.i_qp_rnr_nck_tval_curr_retry     (i_qp_rnr_nck_tval_curr_retry    ),

        // Destination QP configuration for QPi (DESTQPCONFi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。该寄存器由 SW 在连接时配置，提供与该 QP 连接的远程 QPID。从该 QP 发送的所有数据包都以该 QPID 作为目标 QPID。
        .i_qp_dest_qpid_req_wqe_proc  (i_qp_dest_qpid_req_wqe_proc),
        .i_qp_dest_qpid_req_rx_pkt    (i_qp_dest_qpid_req_rx_pkt),
        .o_qp_dest_qpid_valid_wqe_proc(o_qp_dest_qpid_valid_wqe_proc),
        .o_qp_dest_qpid_valid_rx_pkt  (o_qp_dest_qpid_valid_rx_pkt),
        .o_qp_dest_qpid               (o_qp_dest_qpid),                 // DESTQPCONFi[23:0]: 连接到目的地的 QPID
        .i_qp_dest_qpid_idx           (i_qp_dest_qpid_idx),             // Index for reading Q depth

        .o_error_corrupt_conf(o_error_corrupt_conf),  // ?
        .clr_error_corrupt_en(clr_error_corrupt_en),  // ?

        // ?
        .o_tx_hdr_buf_sz_num_hdrs(o_tx_hdr_buf_sz_num_hdrs),
        .o_tx_hdr_buf_sz_buf_sz  (o_tx_hdr_buf_sz_buf_sz),
        .o_tx_sgl_buf_ba         (o_tx_sgl_buf_ba),
        .o_tx_sgl_buf_sz_num_sgls(o_tx_sgl_buf_sz_num_sgls),
        .o_tx_sgl_buf_sz_buf_sz  (o_tx_sgl_buf_sz_buf_sz),

        .o_timeoutreg(o_timeoutreg),  // 固定值: {18'b0, 3'b1, 3'b1, 8'b0} = 32‘h900 = 32'd2304

        .o_qp_disable_pulse_valid(o_qp_disable_pulse_valid),  // qp_conf_bram_wen == 1'b1时，o_qp_disable_pulse_valid <= 1'b1，保持两个周期
        .o_qp_disable_pulse_id   (o_qp_disable_pulse_id),     // qp idx
        .o_qp_disable_pulse_data (o_qp_disable_pulse_data),   // ~QPCONFi[0]: ~QP enable

        // Outgoing Pkt Error status Queue Base address (OPKTERRQBA) 寄存器
        // 该寄存器用于配置传出数据包错误状态队列的基地址。有关传出数据包错误状态队列的详情，请参阅 ERNIC RX Path。
        .o_out_errsts_q_ba   (o_out_errsts_q_ba),    // OPKTERRQBA[63:0]: 出站数据包错误状态队列基地址
        .o_out_errsts_q_sz   (o_out_errsts_q_sz),    // OUTERRSTSQSZ[15:0]: 传出错误 Q 中的条目数。
        .i_out_errsts_q_wrptr(i_out_errsts_q_wrptr), // 固定值: 16'h0000

        // Incoming Pkt Error Status Queue Base address (IPKTERRQBA) 寄存器
        // 该寄存器用于配置传入数据包错误状态队列的基地址。有关传入数据包错误状态队列的详情，请参阅 ERNIC RX Path。
        .o_in_errsts_q_ba   (o_in_errsts_q_ba),    // IPKTERRQBA[63:0]: 传入数据包错误状态队列基地址
        .o_in_errsts_q_sz   (o_in_errsts_q_sz),    // IPKTERRQSZ[15:0]: 收到的错误信息包状态队列条目数。
        .i_in_errsts_q_wrptr(i_in_errsts_q_wrptr), // 传入数据包错误状态队列写指针，指向下一个可写入位置

        // MAC destination address LSB QPi (MACDESADDLSBi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。该寄存器由 SW 在连接时配置，提供与该 QP 连接的远程主机的 MAC 地址。从该 QP 发送的所有数据包都以该 MAC 地址（LSB）作为目标 MAC 地址。
        .i_qp_mac_remote_addrl_req          (i_qp_mac_remote_addrl_req),
        .o_qp_mac_remote_addrl_valid        (o_qp_mac_remote_addrl_valid),
        .i_qp_mac_remote_addrl_wqe_req      (i_qp_mac_remote_addrl_wqe_req),
        .o_qp_mac_remote_addrl_wqe_valid    (o_qp_mac_remote_addrl_wqe_valid),
        .o_qp_mac_remote_addrl              (o_qp_mac_remote_addrl),               // MACDESADDLSBi[31:0]: 目标 MAC 地址 LSB
        .i_qp_mac_remote_addrl_idx          (i_qp_mac_remote_addrl_idx),           // Index for reading Q depth
        .i_qp_mac_remote_addrl_wqe_idx      (i_qp_mac_remote_addrl_wqe_idx),       // Index for reading Q depth
        // MAC destination address LSB QPi (MACDESADDLSBi) 寄存器 复制寄存器
        .o_qp_mac_remote_addrl_replica      (o_qp_mac_remote_addrl_replica),
        .i_qp_mac_remote_addrl_replica_idx  (i_qp_mac_remote_addrl_replica_idx),   // Index for reading Q depth
        .i_qp_mac_remote_addrl_replica_req  (i_qp_mac_remote_addrl_replica_req),
        .o_qp_mac_remote_addrl_replica_valid(o_qp_mac_remote_addrl_replica_valid),

        // MAC destination address MSB QPi (MACDESADDMSBi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。该寄存器由 SW 在连接时配置，提供与该 QP 连接的远程主机的 MAC 地址。从该 QP 发送的所有数据包都以该 MAC 地址（MSB）作为目标 MAC 地址。
        .i_qp_mac_remote_addrm_req          (i_qp_mac_remote_addrm_req),
        .o_qp_mac_remote_addrm_valid        (o_qp_mac_remote_addrm_valid),
        .i_qp_mac_remote_addrm_wqe_req      (i_qp_mac_remote_addrm_wqe_req),
        .o_qp_mac_remote_addrm_wqe_valid    (o_qp_mac_remote_addrm_wqe_valid),
        .o_qp_mac_remote_addrm              (o_qp_mac_remote_addrm),               // MACDESADDMSBi[15:0]: 目标 MAC 地址 MSB
        .i_qp_mac_remote_addrm_idx          (i_qp_mac_remote_addrm_idx),           // Index for reading Q depth
        .i_qp_mac_remote_addrm_wqe_idx      (i_qp_mac_remote_addrm_wqe_idx),       // Index for reading Q depth
        // MAC destination address MSB QPi (MACDESADDMSBi) 寄存器 复制寄存器
        .o_qp_mac_remote_addrm_replica      (o_qp_mac_remote_addrm_replica),
        .i_qp_mac_remote_addrm_replica_idx  (i_qp_mac_remote_addrm_replica_idx),   // Index for reading Q depth
        .i_qp_mac_remote_addrm_replica_req  (i_qp_mac_remote_addrm_replica_req),
        .o_qp_mac_remote_addrm_replica_valid(o_qp_mac_remote_addrm_replica_valid),

        // IP destination address 1 QPi (IPDESADDR1i) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。如果该 QP 的远程主机使用 IPv4 协议配置，则该寄存器指示 IPv4 地址。如果远程主机使用 IPv6 协议配置，寄存器则指示 IPv6 目标地址。
        .i_qp_ip_remote_addr1_req          (i_qp_ip_remote_addr1_req),
        .o_qp_ip_remote_addr1_valid        (o_qp_ip_remote_addr1_valid),
        .i_qp_ip_remote_addr1_wqe_req      (i_qp_ip_remote_addr1_wqe_req),
        .o_qp_ip_remote_addr1_wqe_valid    (o_qp_ip_remote_addr1_wqe_valid),
        .o_qp_ip_remote_addr1              (o_qp_ip_remote_addr1),               // IPDESADDR1i[31:0]: 目标 IP 地址 LSB1
        .i_qp_ip_remote_addr1_idx          (i_qp_ip_remote_addr1_idx),           // Index for reading Q depth
        .i_qp_ip_remote_addr1_wqe_idx      (i_qp_ip_remote_addr1_wqe_idx),       // Index for reading Q depth
        // IP destination address 1 QPi (IPDESADDR1i) 寄存器 复制寄存器
        .o_qp_ip_remote_addr1_replica      (o_qp_ip_remote_addr1_replica),
        .i_qp_ip_remote_addr1_replica_idx  (i_qp_ip_remote_addr1_replica_idx),   // Index for reading Q depth
        .i_qp_ip_remote_addr1_replica_req  (i_qp_ip_remote_addr1_replica_req),
        .o_qp_ip_remote_addr1_replica_valid(o_qp_ip_remote_addr1_replica_valid),

        // IP destination address 2 QPi (IPDESADDR2i) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。如果远程主机使用 IPv6 协议配置，则该寄存器指示 IPv6 目标地址 [63:32]。如果该 QP 的远程主机使用 IPv4 协议配置，则不使用该寄存器。
        .i_qp_ip_remote_addr2_req          (i_qp_ip_remote_addr2_req),
        .o_qp_ip_remote_addr2_valid        (o_qp_ip_remote_addr2_valid),
        .i_qp_ip_remote_addr2_wqe_req      (i_qp_ip_remote_addr2_wqe_req),
        .o_qp_ip_remote_addr2_wqe_valid    (o_qp_ip_remote_addr2_wqe_valid),
        .o_qp_ip_remote_addr2              (o_qp_ip_remote_addr2),               // IPDESADDR2i[31:0]: 目标 IP 地址 LSB2
        .i_qp_ip_remote_addr2_idx          (i_qp_ip_remote_addr2_idx),           // Index for reading Q depth
        .i_qp_ip_remote_addr2_wqe_idx      (i_qp_ip_remote_addr2_wqe_idx),       // Index for reading Q depth
        // IP destination address 2 QPi (IPDESADDR2i) 寄存器 复制寄存器
        .o_qp_ip_remote_addr2_replica      (o_qp_ip_remote_addr2_replica),
        .i_qp_ip_remote_addr2_replica_idx  (i_qp_ip_remote_addr2_replica_idx),   // Index for reading Q depth
        .i_qp_ip_remote_addr2_replica_req  (i_qp_ip_remote_addr2_replica_req),
        .o_qp_ip_remote_addr2_replica_valid(o_qp_ip_remote_addr2_replica_valid),

        // IP destination address 3 QPi (IPDESADDR3i) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。如果远程主机使用 IPv6 协议配置，则该寄存器指示 IPv6 目标地址 [95:64]。如果该 QP 的远程主机使用 IPv4 协议配置，则不使用该寄存器。
        .i_qp_ip_remote_addr3_req          (i_qp_ip_remote_addr3_req),
        .o_qp_ip_remote_addr3_valid        (o_qp_ip_remote_addr3_valid),
        .i_qp_ip_remote_addr3_wqe_req      (i_qp_ip_remote_addr3_wqe_req),
        .o_qp_ip_remote_addr3_wqe_valid    (o_qp_ip_remote_addr3_wqe_valid),
        .o_qp_ip_remote_addr3              (o_qp_ip_remote_addr3),               // IPDESADDR3i[31:0]: 目标 IP 地址 MSB1
        .i_qp_ip_remote_addr3_idx          (i_qp_ip_remote_addr3_idx),           // Index for reading Q depth
        .i_qp_ip_remote_addr3_wqe_idx      (i_qp_ip_remote_addr3_wqe_idx),       // Index for reading Q depth
        // IP destination address 3 QPi (IPDESADDR3i) 寄存器 复制寄存器
        .o_qp_ip_remote_addr3_replica      (o_qp_ip_remote_addr3_replica),
        .i_qp_ip_remote_addr3_replica_idx  (i_qp_ip_remote_addr3_replica_idx),   // Index for reading Q depth
        .i_qp_ip_remote_addr3_replica_req  (i_qp_ip_remote_addr3_replica_req),
        .o_qp_ip_remote_addr3_replica_valid(o_qp_ip_remote_addr3_replica_valid),

        // IP destination address 4 QPi (IPDESADDR4i) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。如果远程主机使用 IPv6 协议配置，则该寄存器指示 IPv6 目标地址 [127:96]。如果该 QP 的远程主机使用 IPv4 协议配置，则不使用该寄存器。
        .i_qp_ip_remote_addr4_req          (i_qp_ip_remote_addr4_req),
        .o_qp_ip_remote_addr4_valid        (o_qp_ip_remote_addr4_valid),
        .i_qp_ip_remote_addr4_wqe_req      (i_qp_ip_remote_addr4_wqe_req),
        .o_qp_ip_remote_addr4_wqe_valid    (o_qp_ip_remote_addr4_wqe_valid),
        .o_qp_ip_remote_addr4              (o_qp_ip_remote_addr4),               // IPDESADDR4i[31:0]: 目标 IP 地址 MSB2
        .i_qp_ip_remote_addr4_idx          (i_qp_ip_remote_addr4_idx),           // Index for reading Q depth
        .i_qp_ip_remote_addr4_wqe_idx      (i_qp_ip_remote_addr4_wqe_idx),       // Index for reading Q depth
        // IP destination address 4 QPi (IPDESADDR4i) 寄存器 复制寄存器
        .o_qp_ip_remote_addr4_replica      (o_qp_ip_remote_addr4_replica),
        .i_qp_ip_remote_addr4_replica_idx  (i_qp_ip_remote_addr4_replica_idx),   // Index for reading Q depth
        .i_qp_ip_remote_addr4_replica_req  (i_qp_ip_remote_addr4_replica_req),
        .o_qp_ip_remote_addr4_replica_valid(o_qp_ip_remote_addr4_replica_valid),

        // Status Current SQ ptr under process (STATCURSQPTRi) 寄存器 (Read only)
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。
        .i_qp_curr_sqptr_proc            (qp_curr_sqptr_proc_wen_retry ? qp_curr_sqptr_proc_retry : new_curr_sqptr_proc),
        .i_qp_curr_sqptr_rdreq           (qp_curr_sqptr_req),
        .i_qp_curr_sqptr_proc_wen        (update_curr_sqptr | qp_curr_sqptr_proc_wen_retry),                                       // 由于 QP 缓存已停止，因此可进行 OR 运算
        .o_prog_full_db_update_curr_sqptr(prog_full_db_update_curr_sqptr),                                                         // Debug registers, 
        .i_qp_curr_sqptr_proc_idx        (qp_curr_sqptr_proc_wen_retry ? qp_curr_sqptr_proc_idx_retry : qp_sq_arbitrated_idx_ff),
        .o_qp_curr_sqptr_proc_valid      (qp_curr_sqptr_proc_valid),
        .o_qp_curr_sqptr_proc            (qp_curr_sqptr_proc),                                                                     // STATCURSQPTRi[15:0]: 当前正在处理的 SQ 指针。这显示了尚未处理并等待远程主机响应的 WQE 数量。

        // Status Response PSN for QPi (STATRESPSNi) 寄存器 (Read only)
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
        .i_qp_stat_resp_psn      (i_qp_stat_resp_psn),        // STATRESPSNi[23:0]: 预期响应的 PSN
        .i_qp_stat_resp_psn_wen  (i_qp_stat_resp_psn_wen),
        .i_qp_stat_resp_psn_idx  (i_qp_stat_resp_psn_idx),
        .o_qp_stat_resp_psn_valid(o_qp_stat_resp_psn_valid),
        .i_qp_stat_resp_psn_rdreq(i_qp_stat_resp_psn_rdreq),
        .o_qp_stat_resp_psn      (o_qp_stat_resp_psn),        // STATRESPSNi[23:0]: 预期响应的 PSN

        // Status RQ buffer current address for QPi (STATRQBUFCAi) 寄存器 (Read only)
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。
        .i_qp_stat_rq_buf_ca      (i_qp_stat_rq_buf_ca),        // STATRQBUFCAi[31:8]: 接收 Q 缓冲区当前地址（256 B 对齐）
        .i_qp_stat_rq_buf_ca_wen  (i_qp_stat_rq_buf_ca_wen),
        .i_qp_stat_rq_buf_ca_idx  (i_qp_stat_rq_buf_ca_idx),
        .o_qp_stat_rq_buf_ca_valid(o_qp_stat_rq_buf_ca_valid),
        .i_qp_stat_rq_buf_ca_rdreq(i_qp_stat_rq_buf_ca_rdreq),
        .o_qp_stat_rq_buf_ca      (o_qp_stat_rq_buf_ca),        // STATRQBUFCAi[31:8]: 接收 Q 缓冲区当前地址（256 B 对齐）

        // Status QPi (STATQPi) 寄存器
        // 该寄存器提供每个 QP 的 QP 状态。该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。
        // STATQPi[30:28]: 当前的 RNR nak 计数（读/写），用于接收应答数据包
        // STATQPi[22:16]: 收到 NACK 综合征（读/写）
        .i_qp_stat_nak            (i_qp_stat_nak),              // {STATQPi[30:28], STATQPi[22:16]}
        .i_qp_stat_nak_wen        (i_qp_stat_nak_wen),
        .i_qp_stat_nak_idx        (i_qp_stat_nak_idx),
        .o_qp_stat_nak_valid      (o_qp_stat_nak_valid),
        .i_qp_stat_nak_rdreq      (i_qp_stat_nak_rdreq),
        .o_qp_stat_nak            (o_qp_stat_nak),              // {STATQPi[30:28], STATQPi[22:16]}
        .i_qp_stat_retry_cnt      (i_qp_stat_retry_cnt),        // STATQPi[26:24]: 当前重试次数（读/写）
        .i_qp_stat_retry_cnt_wen  (i_qp_stat_retry_cnt_wen),
        .i_qp_stat_retry_cnt_idx  (i_qp_stat_retry_cnt_idx),
        .o_qp_stat_retry_cnt_valid(o_qp_stat_retry_cnt_valid),
        .i_qp_stat_retry_cnt_rdreq(i_qp_stat_retry_cnt_rdreq),
        .o_qp_stat_retry_cnt      (o_qp_stat_retry_cnt),        // STATQPi[26:24]: 当前重试次数（读/写）

        // 性能调试计数器 i/f
        .o_global_dbg_cnt_value     (o_global_dbg_cnt_value),
        .o_global_dbg_cnt_clr       (o_global_dbg_cnt_clr),
        .o_global_dbg_cnt_en        (o_global_dbg_cnt_en),
        .i_wqe_fsm_idle_cnt         (i_wqe_fsm_idle_cnt),
        .i_wqe_proc_rd_wqe_cnt      (i_wqe_proc_rd_wqe_cnt),
        .i_wqe_proc_rd_q_info_cnt   (i_wqe_proc_rd_q_info_cnt),
        .i_wqe_proc_wait0_cnt       (i_wqe_proc_wait0_cnt),
        .i_wqe_proc_ip_chksum_cnt   (i_wqe_proc_ip_chksum_cnt),
        .i_wqe_proc_hdr_gen_cnt     (i_wqe_proc_hdr_gen_cnt),
        .i_wqe_proc_hdr_sto_cnt     (i_wqe_proc_hdr_sto_cnt),
        .i_hdr_backpressure_cnt     (i_hdr_backpressure_cnt),
        .i_retry_tx_backpressure_cnt(i_retry_tx_backpressure_cnt),

        .init_start(init_start),
        .init_count(init_count),

        // 来自 WQE proc engine 的状态
        .i_wqe_proc_sts  (i_wqe_proc_sts),
        .i_qp_mgr_sts    ({stat_wqe_cnt, 8'b0, 1'b0, cache_fsm_status, 2'b0, wqe_empty, wqe_full}),  // 赋值给 QPMSTS 寄存器，cache_fsm_status 没用到
        .i_rx_pkt_vld_sts(i_rx_pkt_vld_sts),

        // Debug registers
        .i_axi_rd_err_cnt   (axi_rd_err_cnt_ff),
        .i_last_in_pkt_info (i_last_in_pkt_info),
        .i_last_out_pkt_info(i_last_out_pkt_info)
    );

    assign o_qp_cq_hdptr = qp_cq_hdptr_from_config;
    assign o_qp_sq_ba    = qp_sq_ba;

    reg  [C_NUM_QP-1 : 0] sq_empty;  // 每个QP的SQ是否为空，sq_empty[0]对应QP0的SQ是否为空
    reg  [C_NUM_QP-1 : 0] osq_full;
    reg  [C_NUM_QP-1 : 0] qp_rnr_nacked;
    wire [  C_NUM_QP-1:0] sw_override_per_qp_en;

    always @(posedge core_clk) begin
        if (!core_rstn) begin
            sq_empty      <= {C_NUM_QP{1'b1}};
            osq_full      <= {C_NUM_QP{1'b0}};
            qp_rnr_nacked <= {C_NUM_QP{1'b0}};
        end else begin
            if (sq_empty_valid) begin
                sq_empty[sq_empty_id] <= sq_empty_data;
            end

            if (osq_full_set_valid) begin
                osq_full[osq_full_set_id] <= 1'b1;
            end
            if (osq_full_reset_valid) begin
                osq_full[osq_full_reset_id] <= 1'b0;
            end

            if (osq_rnr_nacked_ff_valid) begin
                qp_rnr_nacked[osq_rnr_nacked_ff_id] <= osq_rnr_nacked_ff_data;
            end
            if (osq_rnr_nacked_ff_valid_pending) begin
                qp_rnr_nacked[osq_rnr_nacked_ff_id_pending] <= osq_rnr_nacked_ff_data_pending;
            end
        end
    end

    assign {osq_rnr_nacked_ff_valid_pending, osq_rnr_nacked_ff_id_pending, osq_rnr_nacked_ff_data_pending, osq_rnr_nacked_ff_valid, osq_rnr_nacked_ff_id, osq_rnr_nacked_ff_data} = i_qp_rnr_nacked;

    // QP RR轮询仲裁器：选取某个有用QP进行处理
    qp_mgr_rr_arb #(
        .C_NUM_ELMNTS_TO_ARB(C_NUM_QP),        // 需要仲裁元素的最大数量
        .C_ELMNT_PTR_WIDTH  (C_QP_INDX_WIDTH)  // SSD编号指针宽度
    ) inst_rr_arbiter (
        // Clock and Reset
        .core_clk(core_clk),   // Module Clock  
        .core_rst(~core_rstn), // Active high core reset

        // Configurations driven from register interface    
        //  .i_num_elements_enabled   (num_elements_enabled), // Number of QPs enabled - right now seting to PARAM, can change 

        // Intreface to sq_arbiter
        .i_arbitrate          (arbitrate),                                                   // 仲裁请求
        .i_enable             (1'b1),                                                        // 启用该模块是为了进行仲裁
        .o_arbitration_done   (arbitration_done),                                            // 仲裁完成，相当于 arbitration_done <= arbitrate
        .o_elmnt_index_one_hot(),                                                            // 获胜元素的一个热门索引
        .o_elmnt_index        (qp_sq_arbitrated_idx),                                        // i_elmnt_unavail 输入后，3周期后输出结果 o_elmnt_index。tips: 需要 SQPIDB 的索引
        .i_elmnt_unavail      (sq_empty | osq_full | qp_rnr_nacked | sw_override_per_qp_en)  // qp不可用向量
    );

    //----------------------------------------------------------------------------
    // cache module 


    qp_mgr_cache #(
        .C_NUM_QP          (C_NUM_QP),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .WQE_FIFO_IDX_WIDTH(WQE_FIFO_IDX_WIDTH),
        .WQE_FIFO_DEPTH    (WQE_FIFO_DEPTH),
        .C_QP_INDX_WIDTH   (C_QP_INDX_WIDTH)
    ) inst_qp_mgr_cache (
        .core_clk (core_clk),
        .core_rstn(core_rstn),

        .o_fsm_status      (cache_fsm_status),
        .o_stat_num_wqe_cnt(stat_wqe_cnt),

        // Configuration details
        .i_sq_empty_and_reduce        (sq_empty_and_reduce),      // 所有 QP 的 SQ 当前是否都为空？
        .i_osq_almost_full_set_valid  (osq_full_set_valid),
        .i_osq_almost_full_set_id     (osq_full_set_id),
        .i_osq_almost_full_reset_valid(osq_full_reset_valid),
        .i_osq_almost_full_reset_id   (osq_full_reset_id),
        .i_qp_sq_arbitrated_idx       (qp_sq_arbitrated_idx_ff),
        .i_wqe_fifo_full              (wqe_full),
        .i_num_valid_entries          (num_valid_entries),
        .i_arbitration_done           (arbitration_done),
        .o_arbitrate                  (arbitrate),
        .o_sample_arbitrated_sq       (sample_arbitrated_sq),

        .o_status_upd_needed(status_upd_needed),  // 1'b0

        // SQ 基地址配置信息
        .o_qp_sq_ba_req  (qp_sq_ba_req_int),
        .i_qp_sq_ba_valid(qp_sq_ba_valid_int),
        .i_qp_sq_ba      (qp_sq_ba),

        // SQ 深度配置信息
        .o_qp_sq_depth_req  (qp_sq_depth_req),
        .i_qp_sq_depth_valid(qp_sq_depth_valid),
        .i_qp_sq_depth      (qp_sq_depth),

        // SQ PIDB 配置信息
        .o_qp_sq_pidb_req  (qp_sq_pidb_req),
        .i_qp_sq_pidb_valid(qp_sq_pidb_valid),
        .i_qp_sq_pidb      (qp_sq_pidb),

        // 正在处理的当前 SQ 指针信息，这显示了尚未处理并等待远程主机响应的 WQE 数量。
        .o_qp_curr_sqptr_rdreq           (qp_curr_sqptr_req),
        .i_qp_curr_sqptr_valid           (qp_curr_sqptr_proc_valid),
        .i_qp_curr_sqptr_proc            (qp_curr_sqptr_proc),
        .o_qp_curr_sqptr                 (new_curr_sqptr_proc),
        .o_update_curr_sqptr             (update_curr_sqptr),
        .i_prog_full_db_update_curr_sqptr(prog_full_db_update_curr_sqptr),

        .i_sw_override_en(sw_override_en),
        .i_xrnic_en      (o_xrnic_en),

        .i_halt                  (halt),
        .o_halted                (cache_halted),
        .i_qp_stat_wqe_cnt_valid (qp_stat_wqe_cnt_valid),
        .o_qp_stat_wqe_cnt_rdreq (qp_stat_wqe_cnt_rdreq),
        .i_qp_stat_wqe_cnt       (qp_stat_wqe_cnt),
        .o_update_qp_stat_wqe_cnt(update_qp_stat_wqe_cnt),
        .o_qp_stat_wqe_cnt       (new_qp_stat_wqe_cnt),

        .o_axi_ren_valid   (axi_ren_valid),
        .o_wqe_qpid        (wqe_qpid_from_qp_cache),
        .o_axi_read_addr   (sq_read_addr_to_axi),
        .i_wqe_addr_latched(bus2ip_arready),
        .i_wqe_dataread    (bus2ip_dvalid_r2),
        .init_start        (init_start),
        .init_count        (init_count)

    );

    assign o_halt = halt;

    // Since the sq_ba and sq_depth is flopped (to have better timing), the ren is also being flopped so have consistent timing.
    // 由于 sq_ba 和 sq_depth 被翻转（以获得更好的时序），ren 也被翻转，以获得一致的时序。
    `MSFF_R(axi_ren_valid_ff, axi_ren_valid, core_clk, ~core_rstn)



    //------------------------------------------------------------------
    // SQ WQE FIFO

    // SQ WQE FIFO 控制逻辑
    q_mgr_queue #(
        .C_Q_DEPTH   (WQE_FIFO_DEPTH),     // 每个队列中的条目数
        .C_ENTRY_SIZE(WQE_SIZE),           // 每个条目的大小（字节）。SQ 大小为 64B
        .C_PTR_WIDTH (WQE_FIFO_IDX_WIDTH)  // 访问 Q 深度所需的指针大小
    ) inst_q (
        // Clock and Reset
        .core_clk(core_clk),   // Clock  
        .core_rst(~core_rstn), // Active high core reset

        // Queue pointers
        .o_rd_addr          (),                   // 读地址堆叠在一条总线上
        .o_wr_addr          (),                   // 写地址堆叠在一条总线上
        .o_rd_ptr           (wqe_fifo_rd_ptr),    // FIFO 读指针，弹出+1
        .o_wr_ptr           (wqe_fifo_wr_ptr),    // FIFO 写指针，推入+1
        .o_num_valid_entries(num_valid_entries),  // 当前FIFO里有效条目数，给 cache 做“是否还能继续从DDR取WQE”的门控
        .o_num_free_entries (),                   // Number of free entries stacked

        // 空满状态。o_wqe_empty 还额外延长 1 拍，防止 BRAM 读延迟导致误判
        .o_q_empty      (wqe_empty),
        .o_q_full       (wqe_full),
        .o_q_almost_full(),           // Q is full -1

        // 配置信号
        .i_base_addr      (32'b0),           // Base address of queues
        .i_q_depth        (WQE_FIFO_DEPTH),
        .i_init           (1'b0),
        .i_threshold_cntrl(1'b0),

        // 推入操作
        // 有新WQE从DDR过来，或者重传模块回推WQE时，入队
        .i_q_push          (bus2ip_dvalid_r2 | wqe_fifo_push_retry),  // Push enable for a queue
        .i_push_num_entries({{(WQE_FIFO_IDX_WIDTH) {1'b0}}, 1'b1}),   // Number of entries to be pushed

        // 弹出操作
        // wqe_proc 消费一条，或重传模块清理/重排时弹出
        .i_q_pop          (i_wqe_pop | wqe_fifo_pop_retry),        // Pop enable for a queue
        .i_pop_num_entries({{(WQE_FIFO_IDX_WIDTH) {1'b0}}, 1'b1})  // Number of entries to be popped
    );

    assign o_wqe_empty = wqe_empty | wqe_empty_ff;  // extending the WQE empty assertion for one more cycle until BRAm data is stable
    `MSFF_R(wqe_empty_ff, wqe_empty, core_clk, ~core_rstn)

    // SQ WQE FIFO 存储逻辑
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (WQE_SIZE * WQE_FIFO_DEPTH),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                     //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),             //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                     //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                         //string;
        .USE_MEM_INIT     (1),                          //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),            //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                          //integer; 0,1
        .ECC_MODE         ("no_ecc"),                   //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                          //Do not Change

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(WQE_SIZE),            //positive integer
        .READ_DATA_WIDTH_A (WQE_SIZE),            //positive integer
        .BYTE_WRITE_WIDTH_A(WQE_SIZE),            //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (WQE_FIFO_IDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),                 //string
        .READ_LATENCY_A    (1),                   //non-negative integer
        .WRITE_MODE_A      ("write_first"),       //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(WQE_SIZE),            //positive integer
        .READ_DATA_WIDTH_B (WQE_SIZE),            //positive integer
        .BYTE_WRITE_WIDTH_B(WQE_SIZE),            //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (WQE_FIFO_IDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                 //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                   //non-negative integer
        .WRITE_MODE_B      ("write_first")        //string; "write_first", "read_first", "no_change" 

    ) inst_wqe_fifo (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (wqe_fifo_push_retry | bus2ip_dvalid_r2),
        .addra         (wqe_fifo_wr_ptr),                                       // 从0开始
        .dina          (wqe_fifo_push_retry ? wqe_to_fifo : wqe_qid_inserted),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (~wqe_empty),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (wqe_fifo_rd_ptr),   // 从0开始
        .dinb          ({WQE_SIZE{1'b0}}),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (cached_wqe),
        .sbiterrb      (),
        .dbiterrb      ()

    );
    // SQ WQE FIFO end
    //------------------------------------------------------------------


    assign o_wqe = cached_wqe;

    `MSFF_R(qp_sq_arbitrated_idx_ff, ((arbitration_done & sample_arbitrated_sq) ? qp_sq_arbitrated_idx : qp_sq_arbitrated_idx_ff), core_clk, ~core_rstn)



    //----------------------------------------------------------------------------
    // 重传模块

    qp_mgr_retransmit #(
        .C_NUM_QP            (C_NUM_QP),
        .C_WQE_FIFO_IDX_WIDTH(WQE_FIFO_IDX_WIDTH),
        .C_QP_INDX_WIDTH     (C_QP_INDX_WIDTH)
    ) inst_retransmit (
        .core_clk (core_clk),
        .core_rstn(core_rstn),

        // Configuration details
        .i_retransmit_reqd        (i_retransmit_reqd),
        .i_retransmit_qpid        (i_retransmit_qpid),          // QP id for this retransmission is required
        .i_retransmit_wqe_bram_din(i_retransmit_wqe_bram_din),  // QP id for this retransmission is required
        .o_retransmit_accepted    (o_retransmit_accepted),
        .i_psn_to_retry           (i_psn_to_retry),
        .i_ssn_to_retry           (i_ssn_to_retry),

        .i_num_valid_entries(num_valid_entries),

        .o_qp_cq_hdptr_req  (qp_cq_hdptr_req_retry),
        .i_qp_cq_hdptr_valid(qp_cq_hdptr_valid_retry),
        .i_qp_cq_hdptr      (qp_cq_hdptr_from_config),
        .o_qp_cq_hdptr_idx  (qp_cq_hdptr_idx_retry),

        // configuration that is updated
        .o_qp_curr_sqptr_proc    (qp_curr_sqptr_proc_retry),
        .o_qp_curr_sqptr_proc_wen(qp_curr_sqptr_proc_wen_retry),
        .o_qp_curr_sqptr_proc_idx(qp_curr_sqptr_proc_idx_retry),  // Index for writing curent SQ pointer

        .o_qp_stat_ret_sq_psn      (qp_stat_ret_sq_psn_int),
        .o_qp_stat_ret_sq_psn_wen  (qp_stat_ret_sq_psn_wen_int),
        .o_qp_stat_ret_sq_psn_idx  (qp_stat_ret_sq_psn_idx_int),
        .i_qp_stat_ret_sq_psn_valid(qp_stat_ret_sq_psn_valid_int),

        .o_qp_sq_psn_req  (qp_sq_psn_req_retry),
        .i_qp_sq_psn_valid(qp_sq_psn_retry_valid),
        .i_qp_sq_psn      (o_qp_sq_psn),
        .o_qp_sq_psn_wrn  (qp_sq_psn_wrn_retry),
        .o_qp_sq_psn      (qp_sq_psn_retry),
        .o_qp_sq_psn_idx  (qp_sq_psn_idx_retry),    // Index for reading Q depth

        .o_qp_stat_ssn_idx               (qp_stat_ssn_idx_retry),           // Index for reading/writing MSN
        .o_qp_stat_ssn                   (qp_stat_ssn_retry),               // Expected MSN for Qp incoming messages
        .i_qp_stat_ssn                   (o_qp_stat_ssn),                   // Expected MSN for Qp incoming messages write data from hdr validation
        .o_qp_stat_ssn_req               (qp_stat_ssn_req_retry),
        .i_prog_full_db_update_curr_sqptr(prog_full_db_update_curr_sqptr),
        .o_qp_stat_ssn_wrn               (qp_stat_ssn_wrn_retry),
        .i_qp_stat_ssn_valid             (o_qp_stat_ssn_valid),

        // Interface to SQ WQE FIFO
        .i_wqe          (wqe_from_fifo),
        .o_wqe          (wqe_to_fifo),
        .o_wqe_fifo_push(wqe_fifo_push_retry),
        .o_wqe_fifo_pop (wqe_fifo_pop_retry),

        .o_halt       (halt),
        .o_halted_qpid(o_halted_qpid),
        .i_halted     (i_wqe_halted & cache_halted),

        .i_osq_pop         (i_osq_pop),
        .i_osq_pop_qpid    (i_osq_pop_qpid),
        .o_wqe_dropped     (retransmit_wqe_dropped),
        .o_wqe_dropped_qpid(retransmit_wqe_dropped_qpid),

        .o_retransmit_wqe_bram_din (retransmit_wqe_bram_din_a),
        .i_retransmit_wqe_bram_dout(retransmit_wqe_bram_dout_a),
        .o_retransmit_wqe_bram_wen (retransmit_wqe_bram_wen_a),
        .o_retransmit_wqe_bram_idx (retransmit_wqe_bram_idx_a)
    );

    ///////////////////////////////////////////////////////////////////////////
    //////////////// Retransmit WQE BRAM instance /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (49 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                //Do not Change

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(49),               //positive integer
        .READ_DATA_WIDTH_A (49),               //positive integer
        .BYTE_WRITE_WIDTH_A(49),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("write_first"),    //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(49),               //positive integer
        .READ_DATA_WIDTH_B (49),               //positive integer
        .BYTE_WRITE_WIDTH_B(49),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("write_first")     //string; "write_first", "read_first", "no_change" 

    ) inst_retransmit_wqe_bram (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (retransmit_wqe_bram_wen_a),
        .addra         (retransmit_wqe_bram_idx_a),
        .dina          (retransmit_wqe_bram_din_a),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (retransmit_wqe_bram_dout_a),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (bus2ip_dvalid),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (bus2ip_qid),
        .dinb          ({49{1'b0}}),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (retransmit_wqe_bram_dout_b),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (1 * C_NUM_QP),     //positive integer
        .MEMORY_PRIMITIVE ("distributed"),    //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(1),               //positive integer
        .BYTE_WRITE_WIDTH_A(1),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (1),                //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //string
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_retrans_check_done (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (bus2ip_dvalid_r1),                //retrans_check_update),
        .addra         (bus2ip_qid_r1),
        .dina          (retransmit_wqe_bram_dout_b[48]),
        .injectsbiterra(1'b0),                            //do not change
        .injectdbiterra(1'b0),                            //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (1'b1),
        .regceb  (1'b1),
        .addrb   (bus2ip_qid),
        .doutb   (retrans_check_done),
        .sbiterrb(),                    //do not change
        .dbiterrb()                     //do not change

    );

    assign retransmit_local_addr  = bus2ip_data_r1[95:32] + retransmit_wqe_bram_dout_b[23:0];
    assign retransmit_remote_addr = bus2ip_data_r1[223:160] + retransmit_wqe_bram_dout_b[23:0];
    assign retransmit_dma_len     = bus2ip_data_r1[127:96] - retransmit_wqe_bram_dout_b[23:0];

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            bus2ip_dvalid_r1 <= 1'b0;
            bus2ip_dvalid_r2 <= 1'b0;
            bus2ip_data_r1   <= 512'd00000000;
            bus2ip_qid_r1    <= 'd0;
            wqe_qid_inserted <= 512'd00000000;
        end else begin
            bus2ip_dvalid_r1 <= bus2ip_dvalid;
            bus2ip_dvalid_r2 <= bus2ip_dvalid_r1;
            bus2ip_data_r1   <= bus2ip_data;
            bus2ip_qid_r1    <= bus2ip_qid;

            // 重传分支
            if ((retransmit_wqe_bram_dout_b[48] != retrans_check_done) && bus2ip_dvalid_r1 && ~(bus2ip_dvalid && bus2ip_qid == bus2ip_qid_r1)) begin
                retrans_check_update <= 1'b1;
                // [511:440]                     ← bus2ip_data_r1[511:440]
                // [439:416]                     ← retransmit_wqe_bram_dout_b[47:24]
                // [415:224]                     ← bus2ip_data_r1[415:224]
                // [223:160]                     ← retransmit_remote_addr
                // [159:152]                     ← 8'h01
                // [151:147]                     ← 5'h00
                // [146:(136 + C_QP_INDX_WIDTH)] ← {(11 - C_QP_INDX_WIDTH){1'b0}}
                // [(135 + C_QP_INDX_WIDTH):136] ← bus2ip_qid_r1
                // [135:128]                     ← bus2ip_data_r1[135:128]
                // [127:96]                      ← retransmit_dma_len
                // [95:32]                       ← retransmit_local_addr
                // [31:0]                        ← bus2ip_data_r1[31:0]
                wqe_qid_inserted <= {bus2ip_data_r1[511 : 440], retransmit_wqe_bram_dout_b[47:24], bus2ip_data_r1[415:224], retransmit_remote_addr, 8'h01, 5'h00, {(11 - C_QP_INDX_WIDTH) {1'b0}}, bus2ip_qid_r1, bus2ip_data_r1[135:128], retransmit_dma_len, retransmit_local_addr, bus2ip_data_r1[31:0]};
            end//
            
            // 普通分支，从DDR中取出SQ WQE执行的分支
            else begin
                // 在手册 WQE 格式的[146:136]插入了 bus2ip_qid_r1，其余保持不变
                wqe_qid_inserted     <= {bus2ip_data_r1[511 : 153], 1'b0, 5'h00, {(11 - C_QP_INDX_WIDTH) {1'b0}}, bus2ip_qid_r1, bus2ip_data_r1[135:0]};
                retrans_check_update <= 1'b0;
            end
        end
    end
    assign wqe_from_fifo = cached_wqe;

    //----------------------------------------------------------------------------
    // 从 DDR 中的 SQ 中抓取 WQE
    qp_mgr_axi_master #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_DATA_WIDTH(512),
        .C_M_AXI_THREAD_ID_WIDTH(1),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .IP2BUS_LEN_WIDTH(14)
    ) inst_axi_master (
        // AXI System signals
        .m_axi_aclk   (core_clk),
        .m_axi_aresetn(core_rstn),

        // AXI Master signals
        .m_axi_awid   (m_axi_awid),
        .m_axi_awaddr (m_axi_awaddr),
        .m_axi_awlen  (m_axi_awlen),
        .m_axi_awsize (m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_awprot (m_axi_awprot),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata  (m_axi_wdata),
        .m_axi_wstrb  (m_axi_wstrb),
        .m_axi_wlast  (m_axi_wlast),
        .m_axi_wvalid (m_axi_wvalid),
        .m_axi_wready (m_axi_wready),
        .m_axi_awlock (m_axi_awlock),
        .m_axi_bid    (m_axi_bid),
        .m_axi_bresp  (m_axi_bresp),
        .m_axi_bvalid (m_axi_bvalid),
        .m_axi_bready (m_axi_bready),
        .m_axi_arid   (m_axi_arid),
        .m_axi_araddr (m_axi_araddr),
        .m_axi_arlen  (m_axi_arlen),
        .m_axi_arsize (m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arcache(m_axi_arcache),
        .m_axi_arprot (m_axi_arprot),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid    (m_axi_rid),
        .m_axi_rdata  (m_axi_rdata),
        .m_axi_rresp  (m_axi_rresp),
        .m_axi_rlast  (m_axi_rlast),
        .m_axi_rvalid (m_axi_rvalid),
        .m_axi_rready (m_axi_rready),
        .m_axi_arlock (m_axi_arlock),

        // Master attachment to IP core side interface. 
        .i_qp_sq_arbitrated_idx(wqe_qpid_from_qp_cache),  // input，仲裁后的 SQ QPID
        .bus2ip_byte_en        (),
        .bus2ip_data           (bus2ip_data),
        .o_qp_sq_idx           (bus2ip_qid),              // i_qp_sq_arbitrated_idx 进入 FIFO 后出来 o_qp_sq_idx, 也就是仲裁后的 SQ QPID
        .bus2ip_dvalid         (bus2ip_dvalid),           // m_axi_rvalid && m_axi_rready_cs(m_axi_rready)
        .bus2ip_arready        (bus2ip_arready),          // m_axi_arready & m_axi_arvalid
        .ip2bus_data           (512'b0),                  // input
        .bus2ip_data_rdy       (),
        .axi_m_en              (axi_ren_valid_ff),        // input，sq_read_addr_to_axi 有效时拉高，使能 AXI 事务
        .wr_rdn                (1'b0),                    // input, 1 = write; 0 = read
        .ip2bus_addr           (sq_read_addr_to_axi),     // input，地址，m_axi_axaddr <= sq_read_addr_to_axi(x取w或r)
        .ip2bus_len            (WQE_SIZE_IN_BYTES),       // input, 长度（以字节为单位）

        // Status indicators to the core IP 
        .axi_master_done  (axi_master_done),
        .axi_master_busy  (axi_master_busy),
        .axi_master_bvalid(axi_master_bvalid),
        .axi_master_error (axi_master_error_nc)

    );

    // 将 sw_override_qpid 信号扩展到每位对应一个 QP
    // sw_override_qpid 单指一个 QP
    genvar k0;
    generate
        for (k0 = 0; k0 < C_NUM_QP; k0 = k0 + 1) begin : sw_override_per_qp_loop
            assign sw_override_per_qp_en[k0] = sw_override_en & ((sw_override_qpid == k0) || (sw_override_qpid == 0));
        end
    endgenerate

    //--------End of debug registers logic---------------//
    `MSFF_R(axi_rd_err_cnt_ff, ((axi_master_error_nc & axi_master_done) ? ((axi_rd_err_cnt_ff == 8'hFF) ? axi_rd_err_cnt_ff : axi_rd_err_cnt_ff + 1) : axi_rd_err_cnt_ff), core_clk, ~core_rstn)


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //this Q manager is used to eleminate the dependency on OSQ almost full signal
    //from response handler, so that the OSQ will reach to its max outstanding 
    //each Q will have a wr and rd pointers. write pointer increments when the WQE
    //is read from the Send queue. read pointer increments when a OSQ is popped
    //from OSQ fifo in response handler
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    /***************************************************************************
   *                                                                          * 
   *                                                                          *
   *            FIFOs and arbitration to decide pop                           *
   *                                                                          *
   *                                                                          *
   ***************************************************************************/

    // i_pop_osq FIFO
    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (16),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (10),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_i_osq_pop (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (i_osq_pop),
        .din          (i_osq_pop_qpid),
        .rd_en        (osq_pop_fifo_rden),
        .empty        (osq_pop_fifo_empty),
        .dout         (osq_pop_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (),
        .rd_data_count(),
        .rd_rst_busy  (),
        .sbiterr      (),
        .underflow    (),
        .wr_ack       (),
        .wr_data_count(),
        .wr_rst_busy  (),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .sleep        (1'b0)
    );

    // i_wqe_dropped FIFO
    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (16),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (10),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_i_wqe_dropped (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (i_wqe_dropped),
        .din          (i_wqe_dropped_qpid),
        .rd_en        (in_wqe_dropped_fifo_rden),
        .empty        (in_wqe_dropped_fifo_empty),
        .dout         (in_wqe_dropped_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (),
        .rd_data_count(),
        .rd_rst_busy  (),
        .sbiterr      (),
        .underflow    (),
        .wr_ack       (),
        .wr_data_count(),
        .wr_rst_busy  (),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .sleep        (1'b0)
    );

    // retransmit_wqe_dropped FIFO
    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (16),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (10),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_retransmit_wqe_droppped (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (retransmit_wqe_dropped),
        .din          (retransmit_wqe_dropped_qpid),
        .rd_en        (retransmit_wqe_fifo_rden),
        .empty        (retransmit_wqe_fifo_empty),
        .dout         (retransmit_wqe_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (),
        .rd_data_count(),
        .rd_rst_busy  (),
        .sbiterr      (),
        .underflow    (),
        .wr_ack       (),
        .wr_data_count(),
        .wr_rst_busy  (),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .sleep        (1'b0)
    );


    // arbiter to arbitrate between the above 3 FIFOs
    assign arb_reqs = {!retransmit_wqe_fifo_empty, !in_wqe_dropped_fifo_empty, !osq_pop_fifo_empty};
    //assign arb_done = 1'b1;

    // Arbiter module
    arbiter #(
        .NUM_REQS(3),
        .BUS_HOLD(0),
        .NEW_ARB (1),
        .SPLIT   (0)
    ) u_fifo_arbiter (
        .clk_arb  (core_clk),
        .rst_n_arb(core_rstn),
        .reqs     (arb_reqs),
        .arb_done (arb_done),
        .gnt      (arb_gnt),
        .gnt_valid()
    );

    assign osq_pop_fifo_rden        = arb_gnt[0] & arb_done;
    assign in_wqe_dropped_fifo_rden = arb_gnt[1] & arb_done;
    assign retransmit_wqe_fifo_rden = arb_gnt[2] & arb_done;

    // based on arbiter output, one of the pop is served
    assign arb_gnt_popen            = |arb_gnt & arb_done;

    always @(*) begin
        if (arb_done) begin
            case (arb_gnt)
                3'b001: begin
                    arb_gnt_pop_qpid = osq_pop_fifo_dout;
                end
                3'b010: begin
                    arb_gnt_pop_qpid = in_wqe_dropped_fifo_dout;
                end
                3'b100: begin
                    arb_gnt_pop_qpid = retransmit_wqe_fifo_dout;
                end
                default: begin
                    arb_gnt_pop_qpid = {C_QP_INDX_WIDTH{1'b0}};
                end
            endcase
        end else begin
            arb_gnt_pop_qpid = {C_QP_INDX_WIDTH{1'b0}};
        end
    end

    // QP OSQ Buffer 管理
    // 不存放具体上下文，只有resp_handler才存放具体上下文
    q_mgr_init_top #(
        .C_NUM_QUEUES        (C_NUM_QP),           // 32 个 QP，每个 QP 对应一个深度为 C_OS_Q_DEPTH 的 OSQ Buffer
        .C_Q_DEPTH           (C_OS_Q_DEPTH),
        .C_ENTRY_SIZE        (1),
        .C_THRESHOLD         (5),
        .C_ECN_THRESHOLD     (C_ECN_THRESHOLD),
        .C_PTR_WIDTH         (C_OS_Q_INDX_WIDTH),  // 4
        .C_ADDR_WIDTH        (OS_Q_ADDR_WIDTH),
        .C_ENABLE_WR_ADDR    (0),
        .C_ENABLE_RD_ADDR    (0),
        .C_FULL_DEPEND_ON_ECN(1)
    ) inst_osq_manager (
        .core_clk               (core_clk),
        .core_rst               (~core_rstn),
        .o_rd_addr              (),
        .o_wr_addr              (),
        .i_wr_addr_rd_en        (1'b0),
        .i_wr_addr_qnum         ({C_QP_INDX_WIDTH{1'b0}}),
        .i_wr_base_addr         ({OS_Q_ADDR_WIDTH{1'b0}}),
        .o_wr_addr_rd_valid     (),
        .i_rd_addr_rd_en        (1'b0),
        .i_rd_addr_qnum         ({C_QP_INDX_WIDTH{1'b0}}),
        .i_rd_base_addr         ({OS_Q_ADDR_WIDTH{1'b0}}),
        .o_rd_addr_rd_valid     (),
        .o_q_full_set_valid     (osq_full_set_valid),
        .o_q_full_set_id        (osq_full_set_id),
        .o_q_full_reset_valid   (osq_full_reset_valid),
        .o_q_full_reset_id      (osq_full_reset_id),
        .vld_free_entry_rden    (1'b0),
        .vld_free_entry_qnum    ({C_QP_INDX_WIDTH{1'b0}}),
        .vld_free_entry_data_vld(),
        .o_num_valid_entries    (),
        .o_num_free_entries     (),
        .i_q_flush              (1'b0),
        .i_q_flush_qpid         ({C_QP_INDX_WIDTH{1'b0}}),
        .o_q_flush_done         (),
        .i_q_depth              (C_OS_Q_DEPTH),
        .i_threshold_cntrl_valid(i_tx_path_congestion_valid),
        .i_threshold_cntrl_id   (i_tx_path_congestion_id),
        .i_threshold_cntrl_data (i_tx_path_congestion_data),
        // push
        .i_q_push               (axi_ren_valid_ff),
        .i_q_push_qpid          (wqe_qpid_from_qp_cache),
        // pop
        .i_q_pop                (arb_gnt_popen),
        .i_q_pop_qpid           (arb_gnt_pop_qpid),
        .o_q_push_idle          (),
        .o_q_pop_idle           (arb_done),
        .init_start             (init_start),
        .init_count             (init_count)
    );


endmodule
