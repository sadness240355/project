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
// resp_handler_top.v
//-----------------------------------------------------------------------------
//
// Filename        : resp_handler_top.v
// Version         : v1.0
// Description     : Module handling response packets from RX pkt handler
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- resp_handler_top.v
//                     -- resp_handler_fsm.v
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
// 处理来自 RX pkt 处理程序的响应数据包的模块
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)

module resp_handler_top #(
    parameter C_M_AXI_ADDR_WIDTH                       = 64,    // 64
    parameter C_NUM_QP                                 = 2048,  // 32
    parameter C_QP_INDX_WIDTH                          = 11,    // 5
    parameter C_OS_Q_DEPTH                             = 16,    // 16
    parameter C_OS_Q_INDX_WIDTH                        = 4,     // 4
    parameter C_EN_NVMOF_HW_HNDSHK                     = 1,     // 启用 NVMoF 硬件握手功能，1
    parameter C_NUM_CYCLES_4US                         = 4096,  // 4 us 的时钟周期数（125Mhz 时钟周期为 512 个时钟周期），1024
    parameter STB_WIDTH                                = 4,     // 4
    parameter C_OSQ_PSN_WIDTH                          = 16,    // 16
    parameter C_ECN_THRESHOLD                          = 8,     // 12
    //parameter   C_EN_WR_RETRY_DATA_BUF  = 0,
    parameter NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE     = 0,     // 0
    parameter NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN   = 0,     // 0
    parameter NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING = 1,     // 0
    parameter RESP_WIDTH                               = 2      // 2
) (
    input wire core_clk,  // Module Clock  
    input wire core_rstn, // Active low core reset

    // WQE 处理器与 未完成缓冲区(OSQ) 的接口
    input  wire [      C_QP_INDX_WIDTH-1:0] i_wqe_qpid,                         // WQE 的 QP ID
    input  wire [                      7:0] i_wqe_opcode,                       // BTH.Opcode of outbound request
    input  wire [                     15:0] i_wqe_wrid,
    input  wire [                     14:0] i_wqe_data_bufid,                   // Data buffer ID for RDMA WRITES
    input  wire                             i_wqe_retried,                      // Retried work request
    input  wire                             i_wqe_fatal_push,                   // WQE request on QP fatal
    input  wire [                     23:0] i_wqe_send_end_psn,                 // End PSN of most recent request (last incase of READ/WRITE)
    input  wire [                     23:0] i_wqe_send_start_psn,               // start PSN of most recent request (first in case of READ/WRITE)
    input  wire [                     23:0] i_wqe_send_msn,
    input  wire [   C_M_AXI_ADDR_WIDTH-1:0] i_wqe_rdma_rd_rcv_ba,               // If request is RDMA READ, receive buffer base address
    input  wire [                     31:0] i_wqe_rdma_rd_res_len,              // If request is RDMA READ, response length in bytes
    input  wire                             i_wqe_push_req,                     // WQE 处理器请求推送 WQE
    input  wire                             i_wqe_push_req_Read,                // request from WQE Processor to push WQE for non RDMA READ opcode
    input  wire                             i_wqe_exp_ack_set,                  // Explicit ack bit is set in this WQE
    output wire                             o_wqe_push_rdy,                     // response to WQE Processor; 0 - busy; 1 - completed  
    output wire                             o_osq_full_set_valid,
    output wire [     C_QP_INDX_WIDTH -1:0] o_osq_full_set_id,
    output wire                             o_osq_full_reset_valid,
    output wire [     C_QP_INDX_WIDTH -1:0] o_osq_full_reset_id,
    output wire                             o_osq_almost_full_set_valid,
    output wire [     C_QP_INDX_WIDTH -1:0] o_osq_almost_full_set_id,
    output wire                             o_osq_almost_full_reset_valid,
    output wire [     C_QP_INDX_WIDTH -1:0] o_osq_almost_full_reset_id,
    output wire                             o_osq_empty_set_valid,
    output wire [     C_QP_INDX_WIDTH -1:0] o_osq_empty_set_id,
    output wire                             o_osq_empty_reset_valid,
    output wire [     C_QP_INDX_WIDTH -1:0] o_osq_empty_reset_id,
    output wire                             o_freeup_data_buf,                  // Freeup RDMA WRITE DATA buffer
    output wire [                     14:0] o_freeup_data_bufid,                // BUffer ID to be freed up
    input  wire                             i_osq_wqe_nack_resp_valid,          // WQE proc has handled the QP nack
    input  wire [     C_QP_INDX_WIDTH -1:0] i_osq_wqe_nack_resp_id,
    input  wire                             i_osq_wqe_nack_resp_data,
    input  wire                             i_osq_wqe_nack_resp_valid_aux,
    input  wire [     C_QP_INDX_WIDTH -1:0] i_osq_wqe_nack_resp_id_aux,
    input  wire                             i_osq_wqe_nack_resp_data_aux,
    input  wire                             i_osq_wqe_nack_resp_valid_pending,
    input  wire [     C_QP_INDX_WIDTH -1:0] i_osq_wqe_nack_resp_id_pending,
    input  wire                             i_osq_wqe_nack_resp_data_pending,
    input  wire                             i_cfg_retry_cnt_fatal_dis,
    input  wire                             i_bypass_en,
    input  wire                             i_qp_fatal_valid,
    input  wire [     C_QP_INDX_WIDTH -1:0] i_qp_fatal_id,
    input  wire                             i_qp_fatal_data,
    input  wire                             i_rd_rsp_wr_done,
    input  wire [      C_QP_INDX_WIDTH-1:0] i_rd_rsp_qpid,
    input  wire [   C_M_AXI_ADDR_WIDTH-1:0] i_rd_rsp_dest_addr,
    input  wire                             i_tx_path_congestion_valid,
    input  wire [      C_QP_INDX_WIDTH-1:0] i_tx_path_congestion_id,
    input  wire                             i_tx_path_congestion_data,
    output wire [    C_OS_Q_INDX_WIDTH : 0] o_num_valid_osq_entries,            // no use, Number of valid OSQ entries 
    input  wire [      C_QP_INDX_WIDTH-1:0] i_qpid_valid_entries,               // QPID for getting valid entries
    output wire [(4*C_QP_INDX_WIDTH+4)-1:0] o_osq_nacked,                       // The outstanding Qs that have been nacked
    input  wire                             i_qp_disable_pulse_set_valid,
    input  wire [     C_QP_INDX_WIDTH -1:0] i_qp_disable_pulse_set_id,

    // Status to QP manager
    output wire [31:0] o_resp_hndler_sts,
    output wire [31:0] o_stat_retry_count,

    // RX 数据包处理程序接口 - MAX PSN Buffer读接口
    input  wire [C_QP_INDX_WIDTH -1 : 0] i_qp_index_mpsnbuf,                   // QP ID to read the max PSN buffer top entry
    input  wire                          i_mpsn_buf_req,                       // Request for reading mpsn buffer (not pop)
    output wire                          o_mpsn_buf_valid,                     // Mpsn buffer output valid (not pop)
    output wire [   C_OSQ_PSN_WIDTH-1:0] o_max_epsn,                           // Max expected response PSN
    output wire [                  23:0] o_trnsfr_length,                      // Transfer length for the WQE
    output wire [                  63:0] o_local_addr,                         // Local address - relevant only for RDMA READ requests
    input  wire                          i_mpsnbuf_pop,                        // Pop the MAX PSN buffer
    output wire                          o_mpsnbuf_empty_set_valid,
    output wire [ C_QP_INDX_WIDTH-1 : 0] o_mpsnbuf_empty_set_id,
    output wire                          o_mpsnbuf_empty_reset_valid,
    output wire [ C_QP_INDX_WIDTH-1 : 0] o_mpsnbuf_empty_reset_id,
    // 完成中断
    input  wire                          i_intr_en_wqe_cmpl,
    input  wire                          i_intr_clr_wqe_cmpl,
    output wire                          o_intr_wqe_cmpl,
    output wire                          o_intr_wqe_cmpl_sts_or_reduce,
    output wire [                31 : 0] o_intr_wqe_cmpl_sts_wr_en,
    output wire [                31 : 0] o_intr_wqe_cmpl_sts_wr_data,
    output wire [ C_QP_INDX_WIDTH-1 : 0] o_intr_wqe_cmpl_sts_wr_addr,
    output wire [                31 : 0] o_intr_wqe_cmpl_sts_wr_en_pending,
    output wire [ C_QP_INDX_WIDTH-1 : 0] o_intr_wqe_cmpl_sts_wr_addr_pending,
    input  wire [                  31:0] i_clr_wqe_cmpl_sts_set_valid,
    input  wire [ C_QP_INDX_WIDTH-1 : 0] i_clr_wqe_cmpl_sts_set_id,

    // 用于处理 ACKED 数据包的 RX 数据包处理程序接口 - ACK PSN Buffer 写接口
    input wire [C_QP_INDX_WIDTH -1 : 0] i_qp_index_acknack,  // QP ID for ack_nack packet PSN
    input wire                          i_acknack_valid,     // Valid signal qualifying the qp_index and acked PSN
    input wire [                  23:0] i_acknacked_psn,     // PSN for the packet being acked/nacked
    input wire [                  23:0] i_acknacked_msn,     // MSN for the packet being acked/nacked
    input wire [                   7:0] i_pkt_nacksyndrome,  // NACK Syndrome. Valid when the i_pkt_nack is asserted
    input wire [                   1:0] i_acknacked_opcode,  // 01:read-first/10:read-middle/11:read-last/only/00-acknowledge

    // 与QP管理器的接口，用于重新传输
    output wire [C_QP_INDX_WIDTH -1 : 0] o_retransmit_qpid,          // QP id for this retransmission is required
    output wire                          o_retransmit_reqd,          // Retransmission is required
    input  wire                          i_retransmit_accepted,      // Ack from QP manager that the retransmission is initiated
    output reg  [                  47:0] o_retransmit_wqe_bram_din,
    output wire [                  23:0] o_psn_to_retry,
    output wire [                  23:0] o_ssn_to_retry,
    input  wire                          i_halt,
    input  wire [C_QP_INDX_WIDTH -1 : 0] i_halted_qpid,

    // 用于 QP 配置的 QP 管理器接口
    output wire [   C_QP_INDX_WIDTH -1:0] o_qp_conf_qp_idx,
    output wire                           o_qp_cq_ba_req,
    input  wire [C_M_AXI_ADDR_WIDTH -1:0] i_qp_cq_ba,                 // CQ base address
    input  wire                           i_qp_cq_ba_valid,
    output wire                           o_qp_cq_depth_req,
    input  wire                           i_qp_cq_depth_valid,        // CQ depth
    input  wire [                   15:0] i_qp_cq_depth,
    output wire                           o_qp_sq_cmpldb_addr_req,
    input  wire                           i_qp_sq_cmpldb_addr_valid,  // SQ Completion DB address (points to RDMAif register)
    input  wire [C_M_AXI_ADDR_WIDTH -1:0] i_qp_sq_cmpldb_addr,

    output wire        o_qp_conf_req,
    input  wire        i_qp_conf_valid,
    input  wire [31:0] i_qp_conf,


    input  wire                       i_qp_sq_psn_resp_valid,
    input  wire [               23:0] i_qp_sq_psn,
    output wire                       o_qp_sq_psn_resp_req,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_sq_psn_resp_idx,

    output wire                          o_qp_cq_hdptr_req,
    input  wire                          i_qp_cq_hdptr_valid,  // CQ head pointer
    input  wire [                  15:0] i_qp_cq_hdptr,
    output wire [                  15:0] o_qp_cq_hdptr,
    output wire                          o_qp_cq_hdptr_wrn,
    output wire [C_QP_INDX_WIDTH -1 : 0] o_qp_cqe_proc_qpid,

    output wire                           o_qp_sq_ba_req,
    input  wire                           i_qp_sq_ba_valid,
    input  wire [C_M_AXI_ADDR_WIDTH -1:0] i_qp_sq_ba,

    // Generic config
    input wire [C_M_AXI_ADDR_WIDTH -1:0] i_data_buf_ba,
    input wire [                   15:0] i_data_buf_sz,

    input wire [C_QP_INDX_WIDTH-1:0] i_num_elements_enabled,
    input wire [                4:0] i_timer_loadval_wqe,        //for 1st WQE goes out
    input wire [                4:0] i_timer_loadval_acknack,    //for ACK/NACK received
    input wire [                3:0] i_cfg_base_cnt,
    input wire                       i_dma_in_idle,
    input wire                       i_en_osq_flush_on_qp_fatal, // For Stopping Completions before Read resp Data write to DDR

    // QP manager should stop arbitrating WQEs if the QP has been RNR nacked
    output wire [(2*C_QP_INDX_WIDTH+4) -1:0] o_osq_rnr_nacked,

    // Status output to QP manager
    output wire                        o_qp_retried_set_valid,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_retried_set_id,
    output wire                        o_qp_retried_reset_valid,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_retried_reset_id,

    // Hardware handshake ports for NVMof (only enabled if C_EN_NVMOF_HW_HNDSHK is set to 1)
    // Direct handshake with RDMAif module
    output wire                           o_send_cq_db_cnt_valid,
    output wire [C_M_AXI_ADDR_WIDTH -1:0] o_send_cq_db_addr,
    output wire [                   31:0] o_send_cq_db_cnt,
    input  wire                           i_send_cq_db_rdy,

    //QP STAT register IF
    input  wire [                2:0] i_qp_curr_retry_cnt,
    output wire                       o_qp_curr_retry_cnt_rd_req,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_curr_retry_cnt_qpid,
    input  wire                       i_qp_curr_retry_cnt_rd_rdy,

    output wire       o_qp_curr_retry_cnt_wr_req,
    input  wire       i_qp_curr_retry_cnt_wr_rdy,
    //output wire [C_QP_INDX_WIDTH-1:0]          o_qp_curr_retry_cnt_qpid,
    output wire [2:0] o_qp_curr_retry_cnt,

    output wire                       o_qp_cfg_retry_cnt_rd_req,
    input  wire                       i_qp_cfg_retry_cnt_rd_rdy,
    input  wire [                2:0] i_qp_cfg_retry_cnt,
    //QP fatal IF for retry limit exceed case
    output wire                       o_qp_retry_fatal_req,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_retry_fatal_qpid,
    input  wire                       i_qp_retry_fatal_rdy,
    // status output
    output wire [               15:0] o_inc_nack_cnt,

    // resp_hndler_m_axi_* 接口
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

    output wire                         o_osq_pop,
    output wire [C_QP_INDX_WIDTH - 1:0] o_osq_pop_qpid,
    output wire                         o_osq_timeout_set_valid,
    output wire [C_QP_INDX_WIDTH-1 : 0] o_osq_timeout_set_id,
    input  wire                         init_start,
    input  wire [C_QP_INDX_WIDTH-1 : 0] init_count,
    input  wire                         i_qp_under_recovery_wr_en,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_qp_under_recovery_wr_idx,
    input  wire                         i_qp_under_recovery_wr_data,
    input  wire                         i_qp_enable_valid,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_qp_enable_id,
    input  wire                         i_qp_enable_data
);
    `include "../../rdma_macros.vh"

    localparam FLOPS_TO_MEM = 1;  // 1


    localparam IP2BUS_LEN_WIDTH = 7;  // 7
    localparam CQE_DB_SIZE_IN_BYTES = 7'd4;  // Bytes，4

    localparam HW_HNDSHK_DATA_WIDTH = C_M_AXI_ADDR_WIDTH + 32;  // 96

    //--------------------
    // MAXPSN / MPSN 缓冲区
    // -------------------------------------------------------------------------------------
    // |BYTE11|BYTE10|BYTE9-|BYTE8-|BYTE7-|BYTE6-|BYTE5-|BYTE4-|BYTE3-|BYTE2-|BYTE1-|BYTE0-|
    // |------|--------------------|-------------------------------------------------------|
    // | FPSN |    LENGTH          |                LOCAL BUFFER ADDRESS                   |
    // |------|--------------------|-------------------------------------------------------|
    // PSN LSB - 8 bits
    // Length  - 24 bits
    // local buffer address (mainly reqd for RDMA READ WQEs) - 64 bits
    // Total = 96 bits
    localparam MAXPSN_BUF_ENTRY_SIZE = 88 + C_OSQ_PSN_WIDTH;  // bits，104
    //--------------------

    //-------------------
    // Outstanding buffer
    //---------------------------------------------------------
    //|BYTE7-|BYTE6-|BYTE5-|BYTE4-|BYTE3-|BYTE2-|BYTE1-|BYTE0-|
    //|------|-|-----------|------|------|------|-------------|
    //| FPSN |R| BUFID     | MSN  | LPSN | OPC  |  WRID       |
    //|------|-|-----------|------|------|------|-------------|
    // FIRST PSN    - 8 bit * see below
    // RETRNSMT WQE - 1 bit
    // QP FATAL PUSH- 1 bit
    // BUFID        - 15 bits
    // MSN LSB      - 8 bits
    // LAST PSN LSB - 8 bits * see below
    // OPcode       - 8 bits
    // WReq ID      - 16 bits
    // Total        - 56 bits
    localparam OSBUF_ENTRY_SIZE = NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING ? 49 + C_OSQ_PSN_WIDTH + C_OSQ_PSN_WIDTH + 16 : 49 + C_OSQ_PSN_WIDTH + C_OSQ_PSN_WIDTH;  // bits

    // * 同时具有第一个和最后一个psn的原因。此函数仅对RDMA READ和RDMA WRITE请求有效。
    // RX数据包处理程序需要第一个PSN（用于READ请求），因为如果没有收到第一个READ PSN和所有后续PSN，则需要生成一个nack。
    // 另一方面，resp_hanlder需要最后一个PSN，因为只有在最后一个PSN被打包后，它才能完成事务。
    // 然而，如果发生误写入交易，必须重试的新PSN必须是第一个PSN，而不是最后一个PSN。因此，它还需要最后一个psn。
    //-------------------

    localparam MASXPSN_BUF_MEM_SZ = C_OS_Q_DEPTH * MAXPSN_BUF_ENTRY_SIZE * C_NUM_QP;  // memory size in bits
    localparam OSBUF_MEM_SZ = C_OS_Q_DEPTH * OSBUF_ENTRY_SIZE * C_NUM_QP;  // memory size in bits

    localparam OS_Q_ADDR_WDITH = (clog2(C_OS_Q_DEPTH * C_NUM_QP));  // 9

    localparam RDMA_READ = 8'b00000100;  // RC RDMA READ REQUEST opcode
    localparam NAK_SEQ_ERR = 8'b01100000;
    localparam RNR_NAK_ERR = 3'b001;  // [7:5] 
    localparam NO_ERROR = 8'b00000000;


    localparam RD_RESP_FIRST = 2'b01;
    localparam RD_RESP_MID = 2'b10;

    // Response handler state is in IDLE
    localparam RESP_H_IN_IDLE = 6'b000000;

    wire [     C_QP_INDX_WIDTH -1 : 0] retransmit_qpid;  // QP id for this retransmission is required
    wire                               phase_to_update_post_retry;
    wire                               nak_seq_error;
    wire                               rnr_nak_error;
    wire                               nack_for_psn_0;
    wire                               retransmit_reqd;
    wire                               wqe_push_for_non_nacked_qp;
    wire                               wqe_push_for_non_nacked_qp_Read;
    wire [                       31:0] resp_hndler_sts;
    wire [                        5:0] resp_h_cs;
    wire [      C_OS_Q_INDX_WIDTH : 0] selected_num_valid_osq_entries;  // Number of valid OSQ entries 
    wire [      C_OS_Q_INDX_WIDTH : 0] push_pop_1_entry;
    wire                               osq_full_reset_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] osq_full_reset_id;
    wire                               osq_full_set_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] osq_full_set_id;
    wire                               osq_almost_full_reset_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] osq_almost_full_reset_id;
    wire                               osq_almost_full_set_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] osq_almost_full_set_id;
    wire                               intr_wqe_cmpl_sts;
    wire [                       15:0] osq_wrid;
    wire [                        7:0] osq_opcode;
    wire [        C_OSQ_PSN_WIDTH-1:0] osq_start_psn;
    wire [        C_OSQ_PSN_WIDTH-1:0] osq_end_psn;
    wire [                        7:0] osq_msn;
    wire [                       23:0] psn_to_retry;
    wire [                       23:0] msn_to_retry;
    wire [      OS_Q_ADDR_WDITH - 1:0] osq_wr_addr_new;
    wire [      OS_Q_ADDR_WDITH - 1:0] osq_rd_addr_new;

    wire [      OS_Q_ADDR_WDITH - 1:0] mpsnbuf_rd_addr_new;
    wire [      OS_Q_ADDR_WDITH - 1:0] mpsnbuf_wr_addr_new;

    wire [                      511:0] ip2bus_data;
    wire [    C_M_AXI_ADDR_WIDTH -1:0] ip2bus_wraddr;
    wire                               ip2bus_wen;
    wire                               arbitrate;
    reg                                arbitration_done;
    wire                               valid_mpsnpush;
    wire [       C_QP_INDX_WIDTH -1:0] osq_arbitrated_idx;
    wire [       C_QP_INDX_WIDTH -1:0] osq_sampled_arbitrated_idx;
    wire [       C_QP_INDX_WIDTH -1:0] arbitrated_idx_for_intr;
    wire                               pop_osq_from_fsm;
    wire [                       23:0] acknacked_psn_for_arbitrated_qp;
    wire [                       23:0] acknacked_msn_for_arbitrated_qp;
    wire [                       23:0] last_acknacked_psn_for_arbitrated_qp;
    wire [       C_QP_INDX_WIDTH -1:0] clr_timeout_n_nack_set_id;  // Clear for i_osq_timed_out
    wire                               clr_timeout_n_nack_set_valid;
    wire                               osq_nacked_cur_qp;
    wire                               osq_nacked_ff_comb;
    wire                               osq_rnr_nacked_set_acknacked_qp;
    reg                                rnr_nak_rcvd;
    //wire [OS_Q_ADDR_WDITH - 1:0]       osq_base_addr_2d_rd_data_0;
    //wire [OS_Q_ADDR_WDITH - 1:0]       osq_base_addr_2d_rd_data_1;
    //wire [OS_Q_ADDR_WDITH - 1:0]       osq_base_addr_2d_rd_data_2;
    wire [      OS_Q_ADDR_WDITH - 1:0] osq_base_addr_2d                                                                      [0:C_NUM_QP - 1];

    wire [                       14:0] osq_data_bufid;
    wire                               osq_wqe_retried;
    wire                               osq_wqe_fatal_push;
    wire [      IP2BUS_LEN_WIDTH -1:0] ip2bus_len;
    wire [      C_OS_Q_INDX_WIDTH : 0] num_free_osq_entries_nc;  // Not connected`
    wire [        C_OS_Q_INDX_WIDTH:0] num_free_mpsn_entries_nc;  // Not connected`
    wire [        C_OS_Q_INDX_WIDTH:0] num_valid_mpsn_entries_nc;  // Not connected`
    wire                               phase_bit_acknack_psn;
    wire [        C_OSQ_PSN_WIDTH-1:0] curr_acknacked_psn_for_curr_qp;
    wire [        C_OSQ_PSN_WIDTH-1:0] last_acknacked_psn_for_curr_qp;
    reg                                qp_sq_psn_resp_valid_ff;
    reg  [                       23:0] qp_sq_psn_ff;

    reg  [                        1:0] acknacked_opcode_ff;  // 01:read-first/10:read-middle/11:read-last/only/00-acknowledge
    reg                                nak_seq_error_ff;
    reg  [                       15:0] inc_nack_cnt;
    reg  [     C_QP_INDX_WIDTH -1 : 0] qp_index_acknack_ff;  // QP ID for ack_nack packet PSN
    reg                                acknack_valid_ff;
    reg  [      C_OS_Q_INDX_WIDTH : 0] num_valid_osq_entries_ff;  // Number of valid OSQ entries 

    reg                                mpsnbuf_req_ff;
    reg                                mpsnbuf_req_2ff;
    reg                                mpsnbuf_req_3ff;
    reg                                mpsnbuf_req_4ff;
    reg                                mpsnbuf_req_5ff;
    reg                                mpsnbuf_req_6ff;
    reg                                mpsnbuf_req_7ff;
    reg                                dma_in_idle_ff;
    // reg mpsnbuf_pop_rd_conflict_ff;
    // reg mpsnbuf_pop_rd_conflict_2ff;
    // reg mpsnbuf_pop_rd_conflict_3ff;

    wire [       OSBUF_ENTRY_SIZE-1:0] osq_in;

    reg  [     OSBUF_ENTRY_SIZE - 1:0] osq_in_q;
    reg  [     OSBUF_ENTRY_SIZE - 1:0] osq_in_2q;


    wire                               send_cq_db_cnt_valid;
    wire [    C_M_AXI_ADDR_WIDTH -1:0] send_cq_db_addr;
    wire [                       31:0] send_cq_db_cnt;
    wire                               send_cq_db_rdy;

    wire                               osq_wr_addr_rd_valid;
    wire                               osq_rd_addr_rd_valid;
    wire [      C_QP_INDX_WIDTH - 1:0] mpsn_pop_qpid;

    wire                               rd_osq;
    wire                               mpsn_wr_addr_rd_valid;

    wire [MAXPSN_BUF_ENTRY_SIZE - 1:0] mpsn_in;
    reg  [MAXPSN_BUF_ENTRY_SIZE - 1:0] mpsn_in_q;
    reg  [MAXPSN_BUF_ENTRY_SIZE - 1:0] mpsn_in_2q;
    wire                               flush_mpsn_from_fsm;
    wire                               flush_mpsn_done_to_fsm;
    wire                               acknacked_psn_rd_req_from_fsm;
    reg                                acknacked_psn_rd_req_done_to_fsm;
    reg                                osq_nacked_to_fsm;
    reg                                osq_rnr_nacked_to_fsm;
    reg                                QP1_acknack_valid;
    wire                               osq_timed_out_set_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] osq_timed_out_set_id;
    wire [      C_QP_INDX_WIDTH-1 : 0] fatal_timeout_qpid;
    reg                                pushback_timeout_fatal_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] pushback_timeout_fatal_id;
    reg                                arbiter_valid_in;
    reg  [      C_QP_INDX_WIDTH-1 : 0] arbiter_Qpid_in;
    reg                                lut_ram_timed_out_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] lut_ram_timed_out_wr_addr;
    reg                                lut_ram_timed_out_wr_data;
    reg                                lut_ram_timed_out_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] lut_ram_timed_out_id_pending;
    wire                               time_out_rd_data_int;
    wire                               time_out_rd_data;
    reg                                lut_ram_fatal_wr_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] lut_ram_fatal_wr_addr;
    reg                                lut_ram_fatal_wr_valid_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] lut_ram_fatal_wr_addr_aux;
    wire                               fatal_rd_data;
    reg                                clear_time_out_valid;
    reg                                clear_fatal_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] clear_time_out_wr_addr;
    reg  [      C_QP_INDX_WIDTH-1 : 0] clear_fatal_wr_addr;
    reg                                fatal_wr_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] fatal_wr_addr;
    reg  [      C_QP_INDX_WIDTH-1 : 0] fatal_check_count;
    wire                               arbitration_done_int;
    wire                               no_OS_pending;
    reg                                no_OS_pending_r;
    reg                                arbitration_done_int_r;
    reg  [      C_QP_INDX_WIDTH-1 : 0] pushback_Qpid;
    reg                                pushback_valid;
    wire                               fifo_busy;
    wire                               fsm_pushback_valid;
    wire                               osq_empty_reset_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] osq_empty_reset_id;
    wire                               osq_empty_set_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] osq_empty_set_id;
    reg                                reload_retry_cnt_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id;
    reg                                reload_retry_cnt_data;
    reg                                reload_retry_cnt_valid_pending_1;
    reg  [      C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id_pending_1;
    reg                                reload_retry_cnt_data_pending_1;
    reg                                reload_retry_cnt_valid_pending_0;
    reg  [      C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id_pending_0;
    reg  [                      1 : 0] condition_reload_retry;
    wire                               first_outgoing_pkt_ff_rd_data_int;
    wire                               first_outgoing_pkt_ff_rd_data;
    reg                                acknacked_psn_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] acknacked_psn_id;
    reg  [                     23 : 0] acknacked_psn_value;
    reg                                acknacked_psn_valid_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] acknacked_psn_id_aux;
    reg  [                     23 : 0] acknacked_psn_value_aux;
    reg                                acknacked_psn_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] acknacked_psn_id_pending;
    reg  [                     23 : 0] acknacked_psn_value_pending;
    reg                                last_acknacked_psn_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] last_acknacked_psn_id_pending;
    reg  [                     23 : 0] last_acknacked_psn_value_pending;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_int_0;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_int_1;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_int_2;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_int_3;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_int_4;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_0;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_1;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_2;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_3;
    wire [                     23 : 0] acknacked_psn_ff_rd_data_4;
    reg                                acknacked_msn_ff_valid;
    reg  [                     23 : 0] acknacked_msn_ff_value;
    reg  [      C_QP_INDX_WIDTH-1 : 0] acknacked_msn_ff_id;
    reg                                acknacked_msn_ff_valid_aux;
    reg  [                     23 : 0] acknacked_msn_ff_value_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] acknacked_msn_ff_id_aux;
    reg                                all_resp_received_set_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] all_resp_received_set_id;
    reg                                all_resp_received_reset_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] all_resp_received_reset_id;
    wire                               all_resp_received_rd_data;
    reg                                last_acknacked_psn_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] last_acknacked_psn_id;
    reg  [                     23 : 0] last_acknacked_psn_value;
    reg                                last_acknacked_psn_valid_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] last_acknacked_psn_id_aux;
    reg  [                     23 : 0] last_acknacked_psn_value_aux;
    wire [                     23 : 0] last_acknacked_psn_ff_rd_data_int_0;
    wire [                     23 : 0] last_acknacked_psn_ff_rd_data_int_1;
    wire [                     23 : 0] last_acknacked_psn_ff_rd_data_0;
    wire [                     23 : 0] last_acknacked_psn_ff_rd_data_1;
    reg                                first_outstanding_pkt_ff_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_id;
    reg                                first_outstanding_pkt_ff_data;
    reg                                first_outstanding_pkt_ff_valid_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_id_aux;
    reg                                first_outstanding_pkt_ff_data_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_id_pending;
    reg                                first_outstanding_pkt_ff_valid_pending;
    wire                               first_outstanding_pkt_ff_rd_data_int;
    wire                               first_outstanding_pkt_ff_rd_data;
    reg  [                       31:0] wqe_cmpl_intr_sts_valid;
    reg  [                       31:0] wqe_cmpl_intr_sts_valid_int;
    reg  [                       31:0] wqe_cmpl_intr_sts_data;
    reg  [      C_QP_INDX_WIDTH-1 : 0] wqe_cmpl_intr_sts_id;
    reg  [                       31:0] wqe_cmpl_intr_sts_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] wqe_cmpl_intr_sts_id_pending;
    wire                               phase_bit_acknack_psn_ff_qp_disable_rd_data_0;
    wire                               phase_bit_acknack_psn_ff_qp_disable_rd_data_1;
    wire                               phase_bit_acknack_psn_ff_qp_disable_rd_data_2;
    wire                               phase_bit_acknack_psn_ff_qp_disable_rd_data_3;
    wire                               phase_bit_acknack_psn_ff_rd_data_int_0;
    wire                               phase_bit_acknack_psn_ff_rd_data_int_1;
    wire                               phase_bit_acknack_psn_ff_rd_data_0;
    wire                               phase_bit_acknack_psn_ff_rd_data_1;
    wire [      C_QP_INDX_WIDTH-1 : 0] fsm_arbitrated_indx;
    wire [      C_QP_INDX_WIDTH-1 : 0] fsm_i_arbitrated_indx;
    reg                                timer_reload_for_acknack_ff_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] timer_reload_for_acknack_ff_id;
    reg                                timer_reload_for_acknack_ff_data;
    reg                                timer_reload_for_acknack_ff_valid_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] timer_reload_for_acknack_ff_id_aux;
    wire                               timer_reload_for_wqe_ff_set;
    wire                               timer_reload_for_wqe_for_i_wqe_qpid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] i_wqe_qpid_r;
    reg                                acknack_psn_for_phase_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] acknack_psn_for_phase_id;
    reg                                acknack_psn_for_phase_data;
    reg                                acknack_psn_for_phase_valid_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] acknack_psn_for_phase_id_aux;
    reg                                acknack_psn_for_phase_data_aux;
    reg                                acknack_psn_for_phase_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] acknack_psn_for_phase_id_pending;
    reg                                acknack_psn_for_phase_data_pending;
    wire                               acknack_psn_for_phase_rd_data_int;
    wire                               acknack_psn_for_phase_rd_data;
    wire                               clr_pending_ack_set_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] clr_pending_ack_set_id;
    wire                               i_osq_wqe_nack_resp_rd_data_0;
    wire                               i_osq_wqe_nack_resp_rd_data_1;
    wire                               osq_nacked_ff_rd_data_0;
    wire                               osq_nacked_ff_rd_data_1;
    wire                               osq_nacked_ff_rd_data_2;
    wire                               osq_nacked_ff_rd_data_3;
    wire                               osq_nacked_ff_rd_data_int_0;
    wire                               osq_nacked_ff_rd_data_int_1;
    wire                               osq_nacked_ff_rd_data_int_2;
    wire                               osq_nacked_ff_rd_data_int_3;
    wire                               osq_rnr_nacked_ff_rd_data_int_0;
    wire                               osq_rnr_nacked_ff_rd_data_int_1;
    wire                               osq_rnr_nacked_ff_rd_data_int_2;
    wire                               osq_rnr_nacked_ff_rd_data_int_3;
    wire                               osq_rnr_nacked_ff_rd_data_0;
    wire                               osq_rnr_nacked_ff_rd_data_1;
    wire                               osq_rnr_nacked_ff_rd_data_2;
    wire                               osq_rnr_nacked_ff_rd_data_3;
    wire [                     23 : 0] acknacked_msn_ff_rd_data;
    reg                                qp_retried_ff_reset_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] qp_retried_ff_reset_id;
    reg                                qp_retried_ff_set_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] qp_retried_ff_set_id;
    reg                                first_outgoing_pkt_2ff_set_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outgoing_pkt_2ff_set_id;
    reg                                osq_nacked_reset_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] mpsnbuf_rd_addr;
    wire                               i_qp_fatal_rd_data_0;
    wire                               i_qp_fatal_rd_data_1;
    wire                               qp_under_fatal_or_recovery_rd_data;
    reg                                i_qp_under_recovery_wr_en_r;
    wire [                       31:0] wqe_cmpl_intr_sts_rd_data;
    reg                                timer_disabled_per_qp_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] timer_disabled_per_qp_id;
    reg                                timer_disabled_per_qp_data;
    reg                                timer_disabled_per_qp_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] timer_disabled_per_qp_id_pending;
    reg                                timer_disabled_per_qp_data_pending;
    wire                               osq_empty_rd_data_0;
    wire                               osq_empty_rd_data_1;
    wire                               osq_empty_rd_data_2;
    wire                               osq_empty_rd_data_3;
    wire                               osq_empty_rd_data_int_0;
    wire                               osq_empty_rd_data_int_1;
    wire                               osq_empty_rd_data_int_2;
    wire                               osq_empty_rd_data_int_3;
    wire                               mpsnbuf_empty_set_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] mpsnbuf_empty_set_id;
    wire                               mpsnbuf_empty_reset_valid;
    wire [      C_QP_INDX_WIDTH-1 : 0] mpsnbuf_empty_reset_id;
    reg                                first_outgoing_pkt_ff_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outgoing_pkt_ff_id;
    reg                                first_outgoing_pkt_ff_data;
    reg                                first_outgoing_pkt_ff_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outgoing_pkt_ff_id_pending;
    reg                                phase_bit_acknack_psn_2ff_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_2ff_id;
    reg                                phase_bit_acknack_psn_2ff_data;
    reg                                phase_bit_acknack_psn_2ff_valid_pending_0;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_2ff_id_pending_0;
    reg                                phase_bit_acknack_psn_2ff_data_pending_0;
    reg                                phase_bit_acknack_psn_2ff_valid_pending_1;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_2ff_id_pending_1;
    reg                                phase_bit_acknack_psn_2ff_data_pending_1;
    reg                                phase_bit_acknack_psn_ff_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_ff_id;
    reg                                phase_bit_acknack_psn_ff_data;
    reg                                phase_bit_acknack_psn_ff_valid_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_ff_id_aux;
    reg                                phase_bit_acknack_psn_ff_data_aux;
    reg                                phase_bit_acknack_psn_ff_qp_disable_valid_aux0;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_ff_qp_disable_id_aux0;
    reg                                phase_bit_acknack_psn_ff_qp_disable_valid_aux1;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_ff_qp_disable_id_aux1;
    reg                                phase_bit_acknack_psn_2ff_qp_disable_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_2ff_qp_disable_id;
    reg                                wqe_cmpl_intr_sts_or_Reduce;
    reg                                Any_ones_in_cmpl_intr_lut;
    reg                                phase_bit_acknack_psn_ff_qp_disable_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_ff_qp_disable_id;
    reg                                first_outstanding_pkt_ff_set_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_set_id;
    reg                                first_outstanding_pkt_ff_set_valid_0;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_set_id_0;
    reg                                first_outstanding_pkt_ff_reset_valid_0;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_reset_id_0;
    reg                                first_outstanding_pkt_ff_set_valid_1;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_set_id_1;
    reg                                first_outstanding_pkt_ff_reset_valid_1;
    reg  [      C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_reset_id_1;
    reg  [        C_QP_INDX_WIDTH : 0] osq_empty_zeros_count;
    wire                               osq_empty_reset_id_rd_data;
    wire                               osq_empty_ff_id_rd_data;
    wire                               osq_empty_reset_id_rd_data_int;
    wire                               osq_empty_set_id_rd_data_int;
    wire                               ack_nack_to_process;
    reg                                i_qp_disable_pulse_set_valid_r;
    reg                                osq_nacked_ff_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id;
    reg                                osq_nacked_ff_data;
    reg                                osq_nacked_ff_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id_pending;
    reg                                osq_rnr_nacked_ff_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_rnr_nacked_ff_id;
    reg                                osq_rnr_nacked_ff_data;
    reg                                osq_rnr_nacked_ff_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_rnr_nacked_ff_id_pending;
    reg                                osq_empty_ff_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_empty_ff_id;
    reg                                osq_empty_ff_data;
    reg                                osq_empty_ff_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_empty_ff_id_pending;
    wire                               i_qp_disable_pulse_set_valid_edge;  // qp_disable_pulse_set_valid 上升沿检测
    reg                                i_qp_disable_pulse_set_valid_edge_r;
    reg  [                        1:0] condition_phase_bit_acknack_psn_2ff;
    reg  [      C_QP_INDX_WIDTH-1 : 0] i_qp_disable_pulse_set_id_r;
    reg  [      C_QP_INDX_WIDTH-1 : 0] timer_reload_for_acknack_ff_id_pending_0;
    reg                                timer_reload_for_acknack_ff_valid_pending_0;
    reg  [      C_QP_INDX_WIDTH-1 : 0] timer_reload_for_acknack_ff_id_pending_1;
    reg                                timer_reload_for_acknack_ff_valid_pending_1;
    wire                               timer_reload_for_acknack_ff_rd_data_int_0;
    reg                                timer_reload_for_acknack_ff_rd_data_0;
    reg                                timer_reload_for_acknack_ff_rd_data_ff_0;
    wire                               timer_reload_for_acknack_ff_rd_data_int_1;
    reg                                timer_reload_for_acknack_ff_rd_data_1;
    reg                                osq_timed_out_sticky_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_id;
    reg                                osq_timed_out_sticky_data;
    reg                                osq_timed_out_sticky_valid_aux;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_id_aux;
    reg                                osq_timed_out_sticky_data_aux;
    reg                                osq_timed_out_sticky_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_id_pending;
    reg                                osq_timed_out_sticky_valid_aux_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_id_aux_pending;
    wire                               osq_timed_out_sticky_rd_data_int_0;
    reg                                osq_timed_out_sticky_rd_data_0;
    reg                                osq_timed_out_sticky_rd_data_0_ff;
    wire                               osq_timed_out_sticky_fsm_arbitrated_indx;
    wire                               osq_timed_out_sticky_rd_data_int_1;
    reg                                osq_timed_out_sticky_rd_data_1;
    wire                               osq_nacked_timer_mem_rdaddr;
    reg                                osq_nacked_timer_mem_rdaddr_i;
    wire                               osq_rnr_nacked_ff_rd_data_comp_int;
    wire                               osq_rnr_nacked_ff_rd_data_comp;
    reg                                osq_rnr_nacked_ff_rd_data_comp_ff;
    wire                               osq_nacked_ff_rd_data_comp_int;
    wire                               osq_nacked_ff_rd_data_comp;
    reg                                osq_nacked_ff_rd_data_comp_ff;
    reg                                timer_reload_for_acknack_ff_data_aux;
    wire [      C_QP_INDX_WIDTH-1 : 0] timer_mem_rdaddr;
    reg                                o_osq_nacked_set_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] o_osq_nacked_set_id;
    reg                                o_osq_nacked_reset_valid;
    reg  [      C_QP_INDX_WIDTH-1 : 0] o_osq_nacked_reset_id;
    reg                                o_osq_nacked_set_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] o_osq_nacked_set_id_pending;
    reg                                o_osq_nacked_reset_valid_pending;
    reg  [      C_QP_INDX_WIDTH-1 : 0] o_osq_nacked_reset_id_pending;
    reg                                i_osq_nacked_wr_pending;
    wire                               i_osq_wqe_nack_resp_rd_data_int_0;
    wire                               i_osq_wqe_nack_resp_rd_data_int_1;
    reg                                push_wr_en_A;
    reg  [      C_QP_INDX_WIDTH-1 : 0] push_wr_id_A;
    reg                                push_wr_en_B;
    reg  [      C_QP_INDX_WIDTH-1 : 0] push_wr_id_B;
    reg  [      C_QP_INDX_WIDTH-1 : 0] buffer_addr_B;
    reg  [        C_QP_INDX_WIDTH : 0] Qpid_buffer_wr_ptr;
    reg  [        C_QP_INDX_WIDTH : 0] Qpid_buffer_rd_ptr;
    reg                                pop_wr_en_B;
    wire                               fifo_empty;
    wire                               fifo_full;
    reg                                push_wr_en_A_r;
    reg                                push_wr_en_B_r;
    reg                                pop_wr_en_B_r;
    reg                                push_pending;
    reg                                push_id_pending;


    always_ff @(posedge core_clk) begin : i_osq_nackded
        if (!core_rstn) begin
            i_osq_nacked_wr_pending <= 1'b0;
        end else begin
            if (i_osq_wqe_nack_resp_valid_aux && i_osq_wqe_nack_resp_valid_pending && i_osq_wqe_nack_resp_id_aux != i_osq_wqe_nack_resp_id_pending) begin
                i_osq_nacked_wr_pending <= 1'b1;
            end else if (i_osq_nacked_wr_pending && (!i_osq_wqe_nack_resp_valid_aux || (i_osq_wqe_nack_resp_valid_aux && i_osq_wqe_nack_resp_id_aux == i_osq_wqe_nack_resp_id_pending) || (i_osq_wqe_nack_resp_valid && i_osq_wqe_nack_resp_id == i_osq_wqe_nack_resp_id_pending))) begin
                i_osq_nacked_wr_pending <= 1'b0;
            end
        end
    end

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_wqe_nack_resp_0 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || i_osq_wqe_nack_resp_valid),
        .wr_addr    (init_start ? init_count : i_osq_wqe_nack_resp_id),
        .wr_data    (init_start ? 1'b0 : i_osq_wqe_nack_resp_data),
        .wr_en_aux  (i_osq_wqe_nack_resp_valid_aux || i_osq_wqe_nack_resp_valid_pending || i_osq_nacked_wr_pending),
        .wr_addr_aux(i_osq_wqe_nack_resp_valid_aux ? i_osq_wqe_nack_resp_id_aux : i_osq_wqe_nack_resp_id_pending),
        .wr_data_aux(i_osq_wqe_nack_resp_valid_aux ? i_osq_wqe_nack_resp_data_aux : i_osq_wqe_nack_resp_data_pending),
        .rd_en      (1'b1),
        .rd_addr    (i_wqe_qpid),
        .rd_data    (i_osq_wqe_nack_resp_rd_data_int_0)
    );
    assign i_osq_wqe_nack_resp_rd_data_0 = ((i_osq_wqe_nack_resp_valid_pending || i_osq_nacked_wr_pending) && i_wqe_qpid == i_osq_wqe_nack_resp_id_pending) ? i_osq_wqe_nack_resp_data_pending : i_osq_wqe_nack_resp_rd_data_int_0;

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_wqe_nack_resp_1 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || i_osq_wqe_nack_resp_valid),
        .wr_addr    (init_start ? init_count : i_osq_wqe_nack_resp_id),
        .wr_data    (init_start ? 1'b0 : i_osq_wqe_nack_resp_data),
        .wr_en_aux  (i_osq_wqe_nack_resp_valid_aux || i_osq_wqe_nack_resp_valid_pending || i_osq_nacked_wr_pending),
        .wr_addr_aux(i_osq_wqe_nack_resp_valid_aux ? i_osq_wqe_nack_resp_id_aux : i_osq_wqe_nack_resp_id_pending),
        .wr_data_aux(i_osq_wqe_nack_resp_valid_aux ? i_osq_wqe_nack_resp_data_aux : i_osq_wqe_nack_resp_data_pending),
        .rd_en      (1'b1),
        .rd_addr    (osq_sampled_arbitrated_idx),
        .rd_data    (i_osq_wqe_nack_resp_rd_data_int_1)
    );
    assign i_osq_wqe_nack_resp_rd_data_1 = ((i_osq_wqe_nack_resp_valid_pending || i_osq_nacked_wr_pending) && osq_sampled_arbitrated_idx == i_osq_wqe_nack_resp_id_pending) ? i_osq_wqe_nack_resp_data_pending : i_osq_wqe_nack_resp_rd_data_int_1;



    axis_fifo #(
        .TDATA_WIDTH(HW_HNDSHK_DATA_WIDTH),
        .FIFO_DEPTH (16)
    ) send_cq_db_axis_fifo (

        .i_clk         (core_clk),
        .i_rstn        (core_rstn),
        .i_inp_data    ({send_cq_db_addr, send_cq_db_cnt}),
        .i_inp_data_vld(send_cq_db_cnt_valid),
        .o_inp_data_rdy(send_cq_db_rdy),
        .o_out_data    ({o_send_cq_db_addr, o_send_cq_db_cnt}),
        .o_out_data_vld(o_send_cq_db_cnt_valid),
        .i_out_data_rdy(i_send_cq_db_rdy)
    );

    `MSFF_R(dma_in_idle_ff, i_dma_in_idle, core_clk, ~core_rstn)

    genvar j;

    // Sample the incoming acked (or nacked) PSN for every QP as it comes from the
    // RX packet handler

    // When a packet is retried, the PSN value is updated to save the last ACKed
    // PSn and not the last NACKed PSN. This makes sure that the same packet is
    // not assumed to be acked. 




    `MSFF_R(acknack_valid_ff, i_acknack_valid, core_clk, ~core_rstn)
    `MSFF_R(qp_index_acknack_ff, i_qp_index_acknack, core_clk, ~core_rstn)

    assign osq_rnr_nacked_set_acknacked_qp = (i_acknack_valid && rnr_nak_error) ? 1'b1 : ((clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_qp_index_acknack) ? 1'b0 : osq_rnr_nacked_ff_rd_data_0);


    // Retry psn inversion is mainly needed for RDMA WRITE transactions where the
    // WRITE MIDDLE and WRITE FIRST psns can be acked and that changes the
    // last_acknaked_psn. This does not happen with RDMA READ transactions as the
    // opcode is known 

    `MSFF_R(acknacked_opcode_ff, i_acknacked_opcode, core_clk, ~core_rstn)
    `MSFF_R(nak_seq_error_ff, nak_seq_error, core_clk, ~core_rstn)

    generate
        if (FLOPS_TO_MEM == 1) begin : ackanacked_psn_move_to_mem  // current

            //==========================================================================
            // acknack_psn_ff
            //==========================================================================
            wire [                  2:0] acknacked_psn_wr_req;
            wire [C_QP_INDX_WIDTH - 1:0] acknacked_psn_wr_req_qpid               [0:2];
            wire [                 26:0] acknacked_psn_wr_req_data               [0:2];

            wire [                  4:0] acknacked_psn_rd_req;
            wire [C_QP_INDX_WIDTH - 1:0] acknacked_psn_rd_req_qpid               [0:4];
            wire [                 26:0] acknacked_psn_rd_req_data               [0:4];
            wire [                  4:0] acknacked_psn_rd_req_done;

            reg  [C_OSQ_PSN_WIDTH - 1:0] curr_acknacked_psn_for_curr_qp_q;
            reg  [                 23:0] acknacked_psn_for_arbitrated_qp_q;
            reg                          chk_arb_prev_4;
            reg                          acknacked_psn_partial_rd_done;
            wire                         phase_bit_acknack_psn_2ff_rd_data_int_0;
            reg                          phase_bit_acknack_psn_2ff_rd_data_0;
            wire                         phase_bit_acknack_psn_2ff_rd_data_int_1;


            reg                          phase_bit_acknack_psn_2ff_rd_data_1;



            wire [C_QP_INDX_WIDTH - 1:0] lastacknacked_psn_wr_req_qpid           [0:2];
            wire [                  2:0] lastacknacked_psn_wr_req;
            wire [                 23:0] lastacknacked_psn_wr_req_data           [0:2];

            wire [C_QP_INDX_WIDTH - 1:0] lastacknacked_psn_rd_req_qpid           [0:1];
            wire [                  1:0] lastacknacked_psn_rd_req;
            wire [                 23:0] lastacknacked_psn_rd_req_data           [0:1];
            wire [                  1:0] lastacknacked_psn_rd_req_done;
            reg                          lastacknacked_psn_for_curr_qp_valid;
            reg  [                 23:0] acknacked_psn_r1;
            reg  [                 23:0] acknacked_psn_r2;
            reg  [                 23:0] acknacked_psn_r3;
            reg                          qp_id_valid_acknack_ff;

            // 3 个写请求
            assign acknacked_psn_wr_req[0]      = i_wqe_push_req & first_outgoing_pkt_ff_rd_data;
            assign acknacked_psn_wr_req_qpid[0] = i_wqe_qpid;
            assign acknacked_psn_wr_req_data[0] = {3'b000, (i_wqe_send_start_psn - 1'b1)};

            assign acknacked_psn_wr_req[1]      = qp_id_valid_acknack_ff;
            assign acknacked_psn_wr_req_qpid[1] = qp_index_acknack_ff;
            assign acknacked_psn_wr_req_data[1] = {phase_bit_acknack_psn_2ff_rd_data_1, osq_nacked_ff_rd_data_0, osq_rnr_nacked_ff_rd_data_1, acknacked_psn_r1[23:0]};

            assign acknacked_psn_wr_req[2]      = clr_timeout_n_nack_set_valid;
            assign acknacked_psn_wr_req_qpid[2] = o_qp_cqe_proc_qpid;
            assign acknacked_psn_wr_req_data[2] = {phase_to_update_post_retry, 2'b00, (psn_to_retry - 1'b1)};

            // 4 个读请求
            assign acknacked_psn_rd_req[0]      = 1'b0;  //i_acknack_valid & (((i_acknacked_opcode != RD_RESP_MID) & (i_acknacked_opcode != RD_RESP_FIRST)) | nak_seq_error);
            assign acknacked_psn_rd_req_qpid[0] = i_qp_index_acknack;

            assign acknacked_psn_rd_req[1]      = 1'b0;  //qp_sq_psn_resp_valid_ff;
            assign acknacked_psn_rd_req_qpid[1] = arbitrated_idx_for_intr;

            assign acknacked_psn_rd_req[2]      = 1'b0;  //acknack_valid_ff;
            assign acknacked_psn_rd_req_qpid[2] = qp_index_acknack_ff;

            // 用于生成 acknacked_psn_for_arbitrated_qp
            assign acknacked_psn_rd_req[3]      = acknacked_psn_rd_req_from_fsm;  //1'b0; // CK：信号必须由有限状态机生成
            assign acknacked_psn_rd_req_qpid[3] = osq_sampled_arbitrated_idx;

            assign acknacked_psn_rd_req[4]      = 1'b0;  //i_acknack_valid;
            assign acknacked_psn_rd_req_qpid[4] = i_qp_index_acknack;
            //assign acknacked_psn_rd_req_done_to_fsm = acknacked_psn_rd_req_done[3]; 
            //`MSFF_R (acknacked_psn_rd_req_done_to_fsm, acknacked_psn_rd_req_done[3], core_clk, !core_rstn)
            `MSFF_R(acknacked_psn_r1, i_acknacked_psn, core_clk, !core_rstn)
            `MSFF_R(acknacked_psn_r2, acknacked_psn_r1, core_clk, !core_rstn)
            `MSFF_R(acknacked_psn_r3, acknacked_psn_r2, core_clk, !core_rstn)
            `MSFF_R(qp_id_valid_acknack_ff, i_acknack_valid, core_clk, !core_rstn)

            always @(posedge core_clk) begin
                if (!core_rstn) begin
                    acknacked_psn_partial_rd_done    <= 1'b0;
                    acknacked_psn_rd_req_done_to_fsm <= 1'b0;
                    osq_nacked_to_fsm                <= 1'b0;
                    osq_rnr_nacked_to_fsm            <= 1'b0;
                end else begin
                    if (acknacked_psn_rd_req_done[3]) begin
                        osq_nacked_to_fsm     <= acknacked_psn_rd_req_data[3][25];
                        osq_rnr_nacked_to_fsm <= acknacked_psn_rd_req_data[3][24];
                    end
                    acknacked_psn_rd_req_done_to_fsm <= acknacked_psn_rd_req_done[3];
                    //if((acknacked_psn_rd_req_done[3] && lastacknacked_psn_rd_req_done[0]) ||
                    //  ((acknacked_psn_rd_req_done[3] || lastacknacked_psn_rd_req_done[0]) && acknacked_psn_partial_rd_done)) begin
                    //    acknacked_psn_partial_rd_done <= 1'b0;
                    //    acknacked_psn_rd_req_done_to_fsm <= 1'b1;
                    //end else if ((acknacked_psn_rd_req_done[3] || lastacknacked_psn_rd_req_done[0]) && !acknacked_psn_partial_rd_done) begin
                    //    acknacked_psn_partial_rd_done <= 1'b1;
                    //    acknacked_psn_rd_req_done_to_fsm <= 1'b0;
                    //end else begin
                    //    acknacked_psn_rd_req_done_to_fsm <= 1'b0;
                    //end
                end
            end

            resp_handler_psn #(
                .DATA_WIDTH     (27),
                .NUM_WR_REQS    (3),
                .WR_FIFO_EN     (1),
                .NUM_RD_REQS    (5),
                .RD_FIFO_EN     (1),
                .C_NUM_QP       (C_NUM_QP),
                .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH)
            ) u_acknacked_psn (
                .core_clk   (core_clk),
                .core_rstn  (core_rstn),
                .wr_reqs    (acknacked_psn_wr_req),
                .wr_req_qpid(acknacked_psn_wr_req_qpid),
                .wr_req_data(acknacked_psn_wr_req_data),
                .wr_req_done(),                           // Unused. Can be used to know when a write is complete
                .rd_reqs    (acknacked_psn_rd_req),
                .rd_req_qpid(acknacked_psn_rd_req_qpid),
                .rd_req_data(acknacked_psn_rd_req_data),  // CK
                .rd_req_done(acknacked_psn_rd_req_done)   // CK
            );

            `MSFF_R(curr_acknacked_psn_for_curr_qp_q, (acknacked_psn_rd_req_done[4] ? acknacked_psn_rd_req_data[4][C_OSQ_PSN_WIDTH-1:0] : {C_OSQ_PSN_WIDTH{1'b0}}), core_clk, !core_rstn)
            always @(posedge core_clk) begin
                if (!core_rstn) begin
                    chk_arb_prev_4 <= 1'b0;
                end else if (acknacked_psn_rd_req_done[2]) begin
                    chk_arb_prev_4 <= 1'b0;
                end else if (acknacked_psn_rd_req_done[4]) begin
                    chk_arb_prev_4 <= 1'b1;
                end
            end
            assign curr_acknacked_psn_for_curr_qp = curr_acknacked_psn_for_curr_qp_q;
            assign nack_for_psn_0                 = i_acknack_valid & (i_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] == 'h0) & (osq_nacked_cur_qp | osq_rnr_nacked_set_acknacked_qp);
            `MSFF_R(acknacked_psn_for_arbitrated_qp_q, (acknacked_psn_rd_req_done[3] ? acknacked_psn_rd_req_data[3][23:0] : acknacked_psn_for_arbitrated_qp_q), core_clk, !core_rstn)
            assign acknacked_psn_for_arbitrated_qp = acknacked_psn_for_arbitrated_qp_q;

            assign o_qp_sq_psn_resp_idx            = i_qp_index_acknack;
            assign o_qp_sq_psn_resp_req            = i_acknack_valid;



            reg qp_sq_psn_req_valid;
            `MSFF_R(qp_sq_psn_req_valid, acknack_valid_ff ? 1'b1 : (qp_sq_psn_resp_valid_ff ? 1'b0 : qp_sq_psn_req_valid), core_clk, ~core_rstn)

            always_ff @(posedge core_clk) begin : all_resp_rcvd_flop_to_mem
                if (~core_rstn) begin
                    all_resp_received_reset_valid <= 1'b0;
                    all_resp_received_reset_id    <= {C_QP_INDX_WIDTH{1'b0}};
                    all_resp_received_set_valid   <= 1'b0;
                    all_resp_received_set_id      <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    if (i_wqe_push_req) begin
                        all_resp_received_reset_valid <= 1'b1;
                        all_resp_received_reset_id    <= i_wqe_qpid;
                    end else begin
                        all_resp_received_reset_valid <= 1'b0;
                    end

                    if (qp_sq_psn_resp_valid_ff && qp_sq_psn_req_valid && ((qp_sq_psn_ff - 1'b1) == acknacked_psn_r1) && ~(i_wqe_push_req && i_wqe_qpid == qp_index_acknack_ff)) begin
                        all_resp_received_set_valid <= 1'b1;
                        all_resp_received_set_id    <= qp_index_acknack_ff;
                    end else begin
                        all_resp_received_set_valid <= 1'b0;
                    end
                end
            end

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_all_resp_received (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || all_resp_received_reset_valid),
                .wr_addr    (init_start ? init_count : all_resp_received_reset_id),
                .wr_data    (1'b0),
                .wr_en_aux  (all_resp_received_set_valid),
                .wr_addr_aux(all_resp_received_set_id),
                .wr_data_aux(1'b1),
                .rd_en      (1'b1),
                .rd_addr    (i_wqe_qpid),
                .rd_data    (all_resp_received_rd_data)
            );


            wire [23:0] wqe_send_start_psn_less_one;
            wire [23:0] psn_to_retry_less_one;
            assign wqe_send_start_psn_less_one = (i_wqe_send_start_psn - 1'b1);
            assign psn_to_retry_less_one       = (psn_to_retry - 1'b1);

            // There is a special case where the PSN = 0 is NACKED. In this case and ONLY
            // in this case the phase bit should not change/toggle. Since in this case the
            // acked PSN is actually from the other "side". However, if the nacked psn is
            // greater than 0, the rollover has occured and the next ack/nack can never go
            // back to the previous "side"
            assign phase_bit_acknack_psn       = (((!i_acknacked_psn[C_OSQ_PSN_WIDTH-1] && acknack_psn_for_phase_rd_data) & ~nack_for_psn_0)) ? ~phase_bit_acknack_psn_2ff_rd_data_0 : phase_bit_acknack_psn_2ff_rd_data_0;

            //###############################################################################################################################################
            //i_wqe_push_req AND clr_timeout_n_nack_set_valid SHOULD HAVE MINIMUN 1 CLOCK GAP.
            //###############################################################################################################################################
            //`MSFF_R(acknack_psn_for_phase[j], ((first_outgoing_pkt_ff[j] & i_wqe_push_req & (i_wqe_qpid == j)) ? wqe_send_start_psn_less_one[C_OSQ_PSN_WIDTH-1] : 
            //(qp_id_valid_acknack[j] ? i_acknacked_psn[C_OSQ_PSN_WIDTH-1] : 
            //(clr_timeout_n_nack[j] ? (psn_to_retry_less_one[C_OSQ_PSN_WIDTH-1]) : acknack_psn_for_phase[j]))), core_clk, ~core_rstn)

            always_ff @(posedge core_clk) begin : acknack_psn_for_phase
                if (~core_rstn) begin
                    acknack_psn_for_phase_valid     <= 1'b0;
                    acknack_psn_for_phase_id        <= {C_QP_INDX_WIDTH{1'b0}};
                    acknack_psn_for_phase_data      <= 1'b0;
                    acknack_psn_for_phase_valid_aux <= 1'b0;
                    acknack_psn_for_phase_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
                    acknack_psn_for_phase_data_aux  <= 1'b0;
                end else begin
                    if (first_outgoing_pkt_ff_rd_data && i_wqe_push_req) begin
                        acknack_psn_for_phase_valid <= 1'b1;
                        acknack_psn_for_phase_id    <= i_wqe_qpid;
                        acknack_psn_for_phase_data  <= wqe_send_start_psn_less_one[C_OSQ_PSN_WIDTH-1];
                    end else if (i_acknack_valid) begin
                        acknack_psn_for_phase_valid <= 1'b1;
                        acknack_psn_for_phase_id    <= i_qp_index_acknack;
                        acknack_psn_for_phase_data  <= i_acknacked_psn[C_OSQ_PSN_WIDTH-1];
                    end else if (acknack_psn_for_phase_valid_pending && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == acknack_psn_for_phase_id_pending)) begin
                        acknack_psn_for_phase_valid <= 1'b1;
                        acknack_psn_for_phase_id    <= acknack_psn_for_phase_id_pending;
                        acknack_psn_for_phase_data  <= acknack_psn_for_phase_data_pending;
                    end else begin
                        acknack_psn_for_phase_valid <= 1'b0;
                    end

                    if (clr_timeout_n_nack_set_valid && !(i_acknack_valid && i_qp_index_acknack == clr_timeout_n_nack_set_id) && !(first_outgoing_pkt_ff_rd_data && i_wqe_push_req && i_wqe_qpid == clr_timeout_n_nack_set_id)) begin
                        acknack_psn_for_phase_valid_aux <= 1'b1;
                        acknack_psn_for_phase_id_aux    <= clr_timeout_n_nack_set_id;
                        acknack_psn_for_phase_data_aux  <= psn_to_retry_less_one[C_OSQ_PSN_WIDTH-1];
                    end else begin
                        acknack_psn_for_phase_valid_aux <= 1'b0;
                    end
                end
            end

            always_ff @(posedge core_clk) begin
                if (~core_rstn) begin
                    acknack_psn_for_phase_valid_pending <= 1'b0;
                    acknack_psn_for_phase_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                    acknack_psn_for_phase_data_pending  <= 1'b0;
                end else begin
                    if (first_outgoing_pkt_ff_rd_data && i_wqe_push_req && i_acknack_valid && i_qp_index_acknack != i_wqe_qpid) begin
                        acknack_psn_for_phase_valid_pending <= 1'b1;
                        acknack_psn_for_phase_id_pending    <= i_qp_index_acknack;
                        acknack_psn_for_phase_data_pending  <= i_acknacked_psn[C_OSQ_PSN_WIDTH-1];
                    end else begin
                        acknack_psn_for_phase_valid_pending <= 1'b0;
                    end
                end
            end

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_acknack_psn_for_phase (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || acknack_psn_for_phase_valid),
                .wr_addr    (init_start ? init_count : acknack_psn_for_phase_id),
                .wr_data    (init_start ? 1'b0 : acknack_psn_for_phase_data),
                .wr_en_aux  (acknack_psn_for_phase_valid_aux),
                .wr_addr_aux(acknack_psn_for_phase_id_aux),
                .wr_data_aux(acknack_psn_for_phase_data_aux),
                .rd_en      (1'b1),
                .rd_addr    (i_qp_index_acknack),
                .rd_data    (acknack_psn_for_phase_rd_data_int)
            );

            assign acknack_psn_for_phase_rd_data       = (acknack_psn_for_phase_valid_pending && acknack_psn_for_phase_id_pending == i_qp_index_acknack) ? acknack_psn_for_phase_data_pending : acknack_psn_for_phase_rd_data_int;

            assign osq_rnr_nacked_for_wqe_push         = (i_acknack_valid && (i_wqe_qpid == i_qp_index_acknack) && rnr_nak_error) ? 1'b1 : ((clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_wqe_qpid) ? 1'b0 : osq_rnr_nacked_ff_rd_data_2);

            assign timer_reload_for_wqe_ff_set         = (first_outstanding_pkt_ff_rd_data || (all_resp_received_rd_data && ~osq_rnr_nacked_for_wqe_push)) && i_wqe_push_req;

            assign timer_reload_for_wqe_for_i_wqe_qpid = (first_outstanding_pkt_ff_rd_data || (all_resp_received_rd_data && ~osq_rnr_nacked_for_wqe_push)) && i_wqe_push_req;

            //###############################################################################################################################################	
            //Assumption for phase_bit_acknack_psn_2ff: i_acknack_valid,
            //clr_timeout_n_nack and i_qp_disable_pulse; all sjould have at least 3 clocks gap 
            //###############################################################################################################################################	
            //`MSFF_R(phase_bit_acknack_psn_2ff[j], (i_qp_disable_pulse[j] ? 1'b0: (((j == i_qp_index_acknack) & i_acknack_valid) ? phase_bit_acknack_psn : 
            //(clr_timeout_n_nack[j] ? phase_to_update_post_retry : phase_bit_acknack_psn_2ff[j]))), core_clk, ~core_rstn)

            assign i_qp_disable_pulse_set_valid_edge   = i_qp_disable_pulse_set_valid && !i_qp_disable_pulse_set_valid_r;
            `MSFF_R(i_qp_disable_pulse_set_valid_edge_r, i_qp_disable_pulse_set_valid_edge, core_clk, !core_rstn)
            always_ff @(posedge core_clk) begin : phase_bit_acknack_psn_2ff
                if (~core_rstn) begin
                    phase_bit_acknack_psn_2ff_valid           <= 1'b0;
                    phase_bit_acknack_psn_2ff_id              <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_2ff_data            <= 1'b0;
                    phase_bit_acknack_psn_2ff_valid_pending_0 <= 1'b0;
                    phase_bit_acknack_psn_2ff_id_pending_0    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_2ff_data_pending_0  <= 1'b0;
                    phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b0;
                    phase_bit_acknack_psn_2ff_id_pending_1    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_2ff_data_pending_1  <= 1'b0;
                    condition_phase_bit_acknack_psn_2ff       <= 2'b00;
                end else begin
                    if (i_qp_disable_pulse_set_valid_edge) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= i_qp_disable_pulse_set_id;
                        phase_bit_acknack_psn_2ff_data  <= 1'b0;
                    end else if (i_acknack_valid) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= i_qp_index_acknack;
                        phase_bit_acknack_psn_2ff_data  <= phase_bit_acknack_psn;
                    end else if (clr_timeout_n_nack_set_valid) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= clr_timeout_n_nack_set_id;
                        phase_bit_acknack_psn_2ff_data  <= phase_to_update_post_retry;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_0) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= phase_bit_acknack_psn_2ff_id_pending_0;
                        phase_bit_acknack_psn_2ff_data  <= phase_bit_acknack_psn_2ff_data_pending_0;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_1) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= phase_bit_acknack_psn_2ff_id_pending_1;
                        phase_bit_acknack_psn_2ff_data  <= phase_bit_acknack_psn_2ff_data_pending_1;
                    end else begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b0;
                    end

                    if (i_qp_disable_pulse_set_valid_edge && i_acknack_valid && i_qp_index_acknack != i_qp_disable_pulse_set_id) begin
                        phase_bit_acknack_psn_2ff_valid_pending_0 <= 1'b1;
                        phase_bit_acknack_psn_2ff_id_pending_0    <= i_qp_index_acknack;
                        phase_bit_acknack_psn_2ff_data_pending_0  <= phase_bit_acknack_psn;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_0 && (!clr_timeout_n_nack_set_valid || (clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == phase_bit_acknack_psn_2ff_id_pending_0))) begin
                        phase_bit_acknack_psn_2ff_valid_pending_0 <= 1'b0;
                    end

                    if (i_qp_disable_pulse_set_valid_edge && i_acknack_valid && clr_timeout_n_nack_set_valid && !(clr_timeout_n_nack_set_id == i_qp_disable_pulse_set_id) && !(clr_timeout_n_nack_set_id == i_qp_index_acknack)) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b1;
                        phase_bit_acknack_psn_2ff_id_pending_1    <= clr_timeout_n_nack_set_id;
                        phase_bit_acknack_psn_2ff_data_pending_1  <= phase_to_update_post_retry;
                        condition_phase_bit_acknack_psn_2ff       <= 2'b00;  //tripple conflict
                    end else if (!i_acknack_valid && i_qp_disable_pulse_set_valid_edge && clr_timeout_n_nack_set_valid && !(clr_timeout_n_nack_set_id == i_qp_disable_pulse_set_id)) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b1;
                        phase_bit_acknack_psn_2ff_id_pending_1    <= clr_timeout_n_nack_set_id;
                        phase_bit_acknack_psn_2ff_data_pending_1  <= phase_to_update_post_retry;
                        condition_phase_bit_acknack_psn_2ff       <= 2'b01;
                    end else if (!i_qp_disable_pulse_set_valid_edge && i_acknack_valid && clr_timeout_n_nack_set_valid && !(clr_timeout_n_nack_set_id == i_qp_index_acknack)) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b1;
                        phase_bit_acknack_psn_2ff_id_pending_1    <= clr_timeout_n_nack_set_id;
                        phase_bit_acknack_psn_2ff_data_pending_1  <= phase_to_update_post_retry;
                        condition_phase_bit_acknack_psn_2ff       <= 2'b10;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && condition_phase_bit_acknack_psn_2ff == 2'b00 && (!phase_bit_acknack_psn_2ff_valid_pending_0)) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b0;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && condition_phase_bit_acknack_psn_2ff == 2'b01 && (!i_acknack_valid || (i_acknack_valid && i_qp_index_acknack == phase_bit_acknack_psn_2ff_id_pending_1))) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b0;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && condition_phase_bit_acknack_psn_2ff == 2'b10 && (!i_qp_disable_pulse_set_valid_edge || (i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == phase_bit_acknack_psn_2ff_id_pending_1))) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b0;
                    end
                end
            end

            always_ff @(posedge core_clk) begin
                i_qp_disable_pulse_set_valid_r <= i_qp_disable_pulse_set_valid;
                i_qp_disable_pulse_set_id_r    <= i_qp_disable_pulse_set_id;
            end

            sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_2ff_0 (
                .core_clk(core_clk),
                .core_rst(~core_rstn),
                .wr_en   (init_start || phase_bit_acknack_psn_2ff_valid),
                .wr_addr (init_start ? init_count : phase_bit_acknack_psn_2ff_id),
                .wr_data (init_start ? 1'b0 : phase_bit_acknack_psn_2ff_data),
                .rd_en   (1'b1),
                .rd_addr (i_qp_index_acknack),
                .rd_data (phase_bit_acknack_psn_2ff_rd_data_int_0)
            );

            always_comb begin
                if (phase_bit_acknack_psn_2ff_valid_pending_0 && i_qp_index_acknack == phase_bit_acknack_psn_2ff_id_pending_0) begin
                    phase_bit_acknack_psn_2ff_rd_data_0 = phase_bit_acknack_psn_2ff_data_pending_0;
                end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && i_qp_index_acknack == phase_bit_acknack_psn_2ff_id_pending_1) begin
                    phase_bit_acknack_psn_2ff_rd_data_0 = phase_bit_acknack_psn_2ff_data_pending_1;
                end else begin
                    phase_bit_acknack_psn_2ff_rd_data_0 = phase_bit_acknack_psn_2ff_rd_data_int_0;
                end
            end

            sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_2ff_1 (
                .core_clk(core_clk),
                .core_rst(~core_rstn),
                .wr_en   (init_start || phase_bit_acknack_psn_2ff_valid),
                .wr_addr (init_start ? init_count : phase_bit_acknack_psn_2ff_id),
                .wr_data (init_start ? 1'b0 : phase_bit_acknack_psn_2ff_data),
                .rd_en   (1'b1),
                .rd_addr (qp_index_acknack_ff),
                .rd_data (phase_bit_acknack_psn_2ff_rd_data_int_1)
            );

            always_comb begin
                if (phase_bit_acknack_psn_2ff_valid_pending_0 && qp_index_acknack_ff == phase_bit_acknack_psn_2ff_id_pending_0) begin
                    phase_bit_acknack_psn_2ff_rd_data_1 = phase_bit_acknack_psn_2ff_data_pending_0;
                end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && qp_index_acknack_ff == phase_bit_acknack_psn_2ff_id_pending_1) begin
                    phase_bit_acknack_psn_2ff_rd_data_1 = phase_bit_acknack_psn_2ff_data_pending_1;
                end else begin
                    phase_bit_acknack_psn_2ff_rd_data_1 = phase_bit_acknack_psn_2ff_rd_data_int_1;
                end
            end

            always_ff @(posedge core_clk) begin : phase_bit_acknack_psn_ff
                if (~core_rstn) begin
                    phase_bit_acknack_psn_ff_valid <= 1'b0;
                    phase_bit_acknack_psn_ff_id    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_ff_data  <= 1'b0;
                end else begin
                    phase_bit_acknack_psn_ff_valid <= acknacked_psn_rd_req_done[3];
                    phase_bit_acknack_psn_ff_id    <= osq_sampled_arbitrated_idx;
                    phase_bit_acknack_psn_ff_data  <= acknacked_psn_rd_req_data[3][26];
                end
            end

            sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_ff (
                .core_clk(core_clk),
                .core_rst(~core_rstn),
                .wr_en   (init_start || phase_bit_acknack_psn_ff_valid),
                .wr_addr (init_start ? init_count : phase_bit_acknack_psn_ff_id),
                .wr_data (init_start ? 1'b0 : phase_bit_acknack_psn_ff_data),
                .rd_en   (1'b1),
                .rd_addr (osq_sampled_arbitrated_idx),
                .rd_data (phase_bit_acknack_psn_ff_rd_data_0)
            );


            //==========================================================================
            // acknack_msn
            //==========================================================================
            wire [C_QP_INDX_WIDTH - 1:0] acknacked_msn_wr_req_qpid[0:1];
            wire [                  1:0] acknacked_msn_wr_req;
            wire [                 23:0] acknacked_msn_wr_req_data[0:1];

            assign acknacked_msn_wr_req[0]      = i_acknack_valid;
            assign acknacked_msn_wr_req_qpid[0] = i_qp_index_acknack;
            assign acknacked_msn_wr_req_data[0] = i_acknacked_msn[23:0];

            assign acknacked_msn_wr_req[1]      = clr_timeout_n_nack_set_valid;
            assign acknacked_msn_wr_req_qpid[1] = o_qp_cqe_proc_qpid;
            assign acknacked_msn_wr_req_data[1] = (msn_to_retry - 1'b1);

            wire                         acknacked_msn_rd_req = acknacked_psn_rd_req_from_fsm;  //1'b0;  // CK: signal has to be generated from FSM
            wire [C_QP_INDX_WIDTH - 1:0] acknacked_msn_rd_req_qpid                                                                                 [0:0];
            assign acknacked_msn_rd_req_qpid[0] = osq_sampled_arbitrated_idx;
            wire        acknacked_msn_rd_req_done;
            wire [23:0] acknacked_msn_rd_req_data   [0:0];
            reg  [23:0] acknacked_msn_rd_req_data_q;



            resp_handler_psn #(
                .DATA_WIDTH     (24),
                .NUM_WR_REQS    (2),
                .WR_FIFO_EN     (1),
                .NUM_RD_REQS    (1),
                .RD_FIFO_EN     (0),
                .C_NUM_QP       (C_NUM_QP),
                .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH)
            ) u_acknacked_msn (
                .core_clk   (core_clk),
                .core_rstn  (core_rstn),
                .wr_reqs    (acknacked_msn_wr_req),
                .wr_req_qpid(acknacked_msn_wr_req_qpid),
                .wr_req_data(acknacked_msn_wr_req_data),
                .wr_req_done(),                           // Unused. Can be used to know when a write is complete
                .rd_reqs    (acknacked_msn_rd_req),
                .rd_req_qpid(acknacked_msn_rd_req_qpid),
                .rd_req_data(acknacked_msn_rd_req_data),
                .rd_req_done(acknacked_msn_rd_req_done)
            );

            `MSFF_R(acknacked_msn_rd_req_data_q, (acknacked_msn_rd_req_done ? acknacked_msn_rd_req_data[0] : acknacked_msn_rd_req_data_q), core_clk, !core_rstn)
            assign acknacked_msn_for_arbitrated_qp = acknacked_msn_rd_req_data_q;

        end  // ackanacked_psn_move_to_mem 

        else begin: flops_logic // FLOPS_TO_MEM != 1
            // This generate statement is unsused and obselete now..
            //assign acknacked_psn_rd_req_done_to_fsm = 1'b1; 
            `MSFF_R(acknacked_psn_rd_req_done_to_fsm, 1'b1, core_clk, !core_rstn)
            `MSFF_R(osq_nacked_to_fsm, 1'b0, core_clk, !core_rstn)
            `MSFF_R(osq_rnr_nacked_to_fsm, 1'b1, core_clk, !core_rstn)
            assign curr_acknacked_psn_for_curr_qp  = acknacked_psn_ff_rd_data_0[C_OSQ_PSN_WIDTH-1:0];
            assign nack_for_psn_0                  = acknack_valid_ff & (acknacked_psn_ff_rd_data_4[C_OSQ_PSN_WIDTH-1:0] == 'h0) & (osq_nacked_cur_qp | osq_rnr_nacked_ff_rd_data_1);
            assign acknacked_psn_for_arbitrated_qp = acknacked_psn_ff_rd_data_1;
            // There is a special case where the PSN = 0 is NACKED. In this case and ONLY
            // in this case the phase bit should not change/toggle. Since in this case the
            // acked PSN is actually from the other "side". However, if the nacked psn is
            // greater than 0, the rollover has occured and the next ack/nack can never go
            // back to the previous "side"
            assign phase_bit_acknack_psn           = (((curr_acknacked_psn_for_curr_qp[C_OSQ_PSN_WIDTH-1:0] < last_acknacked_psn_for_curr_qp[C_OSQ_PSN_WIDTH-1:0]) & ~nack_for_psn_0)) ? ~phase_bit_acknack_psn_ff_rd_data_1 : phase_bit_acknack_psn_ff_rd_data_1;

            always_ff @(posedge core_clk) begin : acknacked_psn_ff
                if (~core_rstn) begin
                    acknacked_psn_valid     <= 1'b0;
                    acknacked_psn_id        <= {C_QP_INDX_WIDTH{1'b0}};
                    acknacked_psn_value     <= {24{1'b0}};
                    acknacked_psn_valid_aux <= 1'b0;
                    acknacked_psn_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
                    acknacked_psn_value_aux <= {24{1'b0}};
                end else begin
                    if (first_outgoing_pkt_ff_rd_data && i_wqe_push_req) begin
                        acknacked_psn_valid <= 1'b1;
                        acknacked_psn_id    <= i_wqe_qpid;
                        acknacked_psn_value <= i_wqe_send_start_psn - 1'b1;
                    end else if (clr_timeout_n_nack_set_valid && ~(i_acknack_valid && i_qp_index_acknack == clr_timeout_n_nack_set_id)) begin
                        acknacked_psn_valid <= 1'b1;
                        acknacked_psn_id    <= clr_timeout_n_nack_set_id;
                        acknacked_psn_value <= psn_to_retry - 1'b1;
                    end else if (acknacked_psn_valid_pending && ~(i_acknack_valid && i_qp_index_acknack == acknacked_psn_id_pending)) begin
                        acknacked_psn_valid <= 1'b1;
                        acknacked_psn_id    <= acknacked_psn_id_pending;
                        acknacked_psn_value <= acknacked_psn_value_pending;
                    end else begin
                        acknacked_psn_valid <= 1'b0;
                    end

                    if (i_acknack_valid && ~(first_outgoing_pkt_ff_rd_data && i_wqe_push_req && i_wqe_qpid == i_qp_index_acknack)) begin
                        acknacked_psn_valid_aux <= 1'b1;
                        acknacked_psn_id_aux    <= i_qp_index_acknack;
                        acknacked_psn_value_aux <= i_acknacked_psn[23:0];
                    end else begin
                        acknacked_psn_valid_aux <= 1'b0;
                    end
                end
            end

            always_ff @(posedge core_clk) begin
                if (~core_rstn) begin
                    acknacked_psn_valid_pending <= 1'b0;
                    acknacked_psn_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                    acknacked_psn_value_pending <= {24{1'b0}};
                end else begin
                    //clr_timeout_n_nack is low frequency interface; hence write can wait;
                    if (first_outgoing_pkt_ff_rd_data && i_wqe_push_req && clr_timeout_n_nack_set_valid) begin
                        acknacked_psn_valid_pending <= 1'b1;
                        acknacked_psn_id_pending    <= clr_timeout_n_nack_set_id;
                        acknacked_psn_value_pending <= psn_to_retry - 1'b1;
                    end else if (acknacked_psn_valid_pending && ((acknacked_psn_valid && acknacked_psn_id == acknacked_psn_id_pending) || (acknacked_psn_valid_aux && acknacked_psn_id_aux == acknacked_psn_id_pending))) begin
                        acknacked_psn_valid_pending <= 1'b0;
                    end
                end
            end


            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(24),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_acknacked_psn_ff_0 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || acknacked_psn_valid),
                .wr_addr    (init_start ? init_count : acknacked_psn_id),
                .wr_data    (init_start ? {24{1'b0}} : acknacked_psn_value),
                .wr_en_aux  (acknacked_psn_valid_aux),
                .wr_addr_aux(acknacked_psn_id_aux),
                .wr_data_aux(acknacked_psn_value_aux),
                .rd_en      (1'b1),
                .rd_addr    (i_qp_index_acknack),
                .rd_data    (acknacked_psn_ff_rd_data_int_0)
            );

            assign acknacked_psn_ff_rd_data_0 = (acknacked_psn_valid_pending && acknacked_psn_id_pending == i_qp_index_acknack) ? acknacked_psn_value_pending : acknacked_psn_ff_rd_data_int_0;

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(24),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_acknacked_psn_ff_1 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || acknacked_psn_valid),
                .wr_addr    (init_start ? init_count : acknacked_psn_id),
                .wr_data    (init_start ? {24{1'b0}} : acknacked_psn_value),
                .wr_en_aux  (acknacked_psn_valid_aux),
                .wr_addr_aux(acknacked_psn_id_aux),
                .wr_data_aux(acknacked_psn_value_aux),
                .rd_en      (1'b1),
                .rd_addr    (osq_sampled_arbitrated_idx),
                .rd_data    (acknacked_psn_ff_rd_data_int_1)
            );

            assign acknacked_psn_ff_rd_data_1 = (acknacked_psn_valid_pending && acknacked_psn_id_pending == osq_sampled_arbitrated_idx) ? acknacked_psn_value_pending : acknacked_psn_ff_rd_data_int_1;

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(24),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_acknacked_psn_ff_2 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || acknacked_psn_valid),
                .wr_addr    (init_start ? init_count : acknacked_psn_id),
                .wr_data    (init_start ? {24{1'b0}} : acknacked_psn_value),
                .wr_en_aux  (acknacked_psn_valid_aux),
                .wr_addr_aux(acknacked_psn_id_aux),
                .wr_data_aux(acknacked_psn_value_aux),
                .rd_en      (1'b1),
                .rd_addr    (i_wqe_qpid),
                .rd_data    (acknacked_psn_ff_rd_data_int_2)
            );

            assign acknacked_psn_ff_rd_data_2 = (acknacked_psn_valid_pending && acknacked_psn_id_pending == i_wqe_qpid) ? acknacked_psn_value_pending : acknacked_psn_ff_rd_data_int_2;

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(24),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_acknacked_psn_ff_3 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || acknacked_psn_valid),
                .wr_addr    (init_start ? init_count : acknacked_psn_id),
                .wr_data    (init_start ? {24{1'b0}} : acknacked_psn_value),
                .wr_en_aux  (acknacked_psn_valid_aux),
                .wr_addr_aux(acknacked_psn_id_aux),
                .wr_data_aux(acknacked_psn_value_aux),
                .rd_en      (1'b1),
                .rd_addr    (arbitrated_idx_for_intr),
                .rd_data    (acknacked_psn_ff_rd_data_int_3)
            );

            assign acknacked_psn_ff_rd_data_3 = (acknacked_psn_valid_pending && acknacked_psn_id_pending == arbitrated_idx_for_intr) ? acknacked_psn_value_pending : acknacked_psn_ff_rd_data_int_3;

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(24),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_acknacked_psn_ff_4 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || acknacked_psn_valid),
                .wr_addr    (init_start ? init_count : acknacked_psn_id),
                .wr_data    (init_start ? {24{1'b0}} : acknacked_psn_value),
                .wr_en_aux  (acknacked_psn_valid_aux),
                .wr_addr_aux(acknacked_psn_id_aux),
                .wr_data_aux(acknacked_psn_value_aux),
                .rd_en      (1'b1),
                .rd_addr    (qp_index_acknack_ff),
                .rd_data    (acknacked_psn_ff_rd_data_int_4)
            );

            assign acknacked_psn_ff_rd_data_4 = (acknacked_psn_valid_pending && acknacked_psn_id_pending == qp_index_acknack_ff) ? acknacked_psn_value_pending : acknacked_psn_ff_rd_data_int_4;

            always_ff @(posedge core_clk) begin : all_resp_received_proc_flops_logic_per_qp
                if (~core_rstn) begin
                    all_resp_received_set_valid   <= 1'b0;
                    all_resp_received_set_id      <= {C_QP_INDX_WIDTH{1'b0}};
                    all_resp_received_reset_valid <= 1'b0;
                    all_resp_received_reset_id    <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    if (qp_sq_psn_resp_valid_ff && ((qp_sq_psn_ff - 1'b1) == acknacked_psn_ff_rd_data_3)) begin
                        all_resp_received_set_valid <= 1'b1;
                        all_resp_received_set_id    <= arbitrated_idx_for_intr;
                    end else begin
                        all_resp_received_set_valid <= 1'b0;
                    end

                    if (qp_sq_psn_resp_valid_ff && ((qp_sq_psn_ff - 1'b1) != acknacked_psn_ff_rd_data_3)) begin
                        all_resp_received_reset_valid <= 1'b1;
                        all_resp_received_reset_id    <= arbitrated_idx_for_intr;
                    end else begin
                        all_resp_received_reset_valid <= 1'b0;
                    end
                end
            end

            assign timer_reload_for_wqe_ff_set         = (first_outstanding_pkt_ff_rd_data || (((acknacked_psn_ff_rd_data_2 + 24'd1) == i_wqe_send_start_psn) && ~osq_rnr_nacked_ff_rd_data_2)) && i_wqe_push_req;

            assign timer_reload_for_wqe_for_i_wqe_qpid = (first_outstanding_pkt_ff_rd_data || (((acknacked_psn_ff_rd_data_2 + 24'd1) == i_wqe_send_start_psn) && ~osq_rnr_nacked_ff_rd_data_2)) && i_wqe_push_req;

            always_ff @(posedge core_clk) begin : phase_bit_acknack_psn_ff
                if (~core_rstn) begin
                    phase_bit_acknack_psn_ff_valid                 <= 1'b0;
                    phase_bit_acknack_psn_ff_id                    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_ff_data                  <= 1'b0;
                    phase_bit_acknack_psn_ff_valid_aux             <= 1'b0;
                    phase_bit_acknack_psn_ff_id_aux                <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_ff_data_aux              <= 1'b0;
                    phase_bit_acknack_psn_ff_qp_disable_valid_aux0 <= 1'b0;
                    phase_bit_acknack_psn_ff_qp_disable_id_aux0    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_ff_qp_disable_valid_aux1 <= 1'b0;
                    phase_bit_acknack_psn_ff_qp_disable_id_aux1    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_ff_qp_disable_valid      <= 1'b0;
                    phase_bit_acknack_psn_ff_qp_disable_id         <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    phase_bit_acknack_psn_ff_valid                 <= acknack_valid_ff;
                    phase_bit_acknack_psn_ff_id                    <= i_qp_index_acknack;
                    phase_bit_acknack_psn_ff_data                  <= phase_bit_acknack_psn;
                    phase_bit_acknack_psn_ff_valid_aux             <= clr_timeout_n_nack_set_valid && ~(acknack_valid_ff && i_qp_index_acknack == clr_timeout_n_nack_set_id);
                    phase_bit_acknack_psn_ff_id_aux                <= clr_timeout_n_nack_set_id;
                    phase_bit_acknack_psn_ff_data_aux              <= phase_to_update_post_retry;

                    phase_bit_acknack_psn_ff_qp_disable_valid      <= i_qp_disable_pulse_set_valid;
                    phase_bit_acknack_psn_ff_qp_disable_id         <= i_qp_disable_pulse_set_id;
                    phase_bit_acknack_psn_ff_qp_disable_valid_aux0 <= acknack_valid_ff && ~(i_qp_disable_pulse_set_valid && i_qp_index_acknack == i_qp_disable_pulse_set_id);
                    phase_bit_acknack_psn_ff_qp_disable_id_aux0    <= i_qp_index_acknack;
                    phase_bit_acknack_psn_ff_qp_disable_valid_aux1 <= clr_timeout_n_nack_set_valid && (~(i_qp_disable_pulse_set_valid && clr_timeout_n_nack_set_id == i_qp_disable_pulse_set_id) && ~(acknack_valid_ff && clr_timeout_n_nack_set_id == i_qp_index_acknack));
                    phase_bit_acknack_psn_ff_qp_disable_id_aux1    <= clr_timeout_n_nack_set_id;
                end
            end

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_ff_0 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || phase_bit_acknack_psn_ff_valid),
                .wr_addr    (init_start ? init_count : phase_bit_acknack_psn_ff_id),
                .wr_data    (init_start ? 1'b0 : phase_bit_acknack_psn_ff_data),
                .wr_en_aux  (phase_bit_acknack_psn_ff_valid_aux),
                .wr_addr_aux(phase_bit_acknack_psn_ff_id_aux),
                .wr_data_aux(phase_bit_acknack_psn_ff_data_aux),
                .rd_en      (1'b1),
                .rd_addr    (osq_sampled_arbitrated_idx),
                .rd_data    (phase_bit_acknack_psn_ff_rd_data_int_0)
            );

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_ff_qp_disable_0 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || phase_bit_acknack_psn_ff_qp_disable_valid_aux0),
                .wr_addr    (init_start ? init_count : phase_bit_acknack_psn_ff_qp_disable_id_aux0),
                .wr_data    (1'b0),
                .wr_en_aux  (phase_bit_acknack_psn_ff_qp_disable_valid),
                .wr_addr_aux(phase_bit_acknack_psn_ff_qp_disable_id),
                .wr_data_aux(1'b1),
                .rd_en      (1'b1),
                .rd_addr    (osq_sampled_arbitrated_idx),
                .rd_data    (phase_bit_acknack_psn_ff_qp_disable_rd_data_0)
            );

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_ff_qp_disable_1 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || phase_bit_acknack_psn_ff_qp_disable_valid_aux1),
                .wr_addr    (init_start ? init_count : phase_bit_acknack_psn_ff_qp_disable_id_aux1),
                .wr_data    (1'b0),
                .wr_en_aux  (phase_bit_acknack_psn_ff_qp_disable_valid),
                .wr_addr_aux(phase_bit_acknack_psn_ff_qp_disable_id),
                .wr_data_aux(1'b1),
                .rd_en      (1'b1),
                .rd_addr    (osq_sampled_arbitrated_idx),
                .rd_data    (phase_bit_acknack_psn_ff_qp_disable_rd_data_1)
            );

            assign phase_bit_acknack_psn_ff_rd_data_0 = (phase_bit_acknack_psn_ff_qp_disable_rd_data_0 & phase_bit_acknack_psn_ff_qp_disable_rd_data_1) ? 1'b0 : phase_bit_acknack_psn_ff_rd_data_int_0;

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_ff_1 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || phase_bit_acknack_psn_ff_valid),
                .wr_addr    (init_start ? init_count : phase_bit_acknack_psn_ff_id),
                .wr_data    (init_start ? 1'b0 : phase_bit_acknack_psn_ff_data),
                .wr_en_aux  (phase_bit_acknack_psn_ff_valid_aux),
                .wr_addr_aux(phase_bit_acknack_psn_ff_id_aux),
                .wr_data_aux(phase_bit_acknack_psn_ff_data_aux),
                .rd_en      (1'b1),
                .rd_addr    (qp_index_acknack_ff),
                .rd_data    (phase_bit_acknack_psn_ff_rd_data_int_1)
            );

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_ff_qp_disable_2 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || phase_bit_acknack_psn_ff_qp_disable_valid_aux0),
                .wr_addr    (init_start ? init_count : phase_bit_acknack_psn_ff_qp_disable_id_aux0),
                .wr_data    (1'b0),
                .wr_en_aux  (phase_bit_acknack_psn_ff_qp_disable_valid),
                .wr_addr_aux(phase_bit_acknack_psn_ff_qp_disable_id),
                .wr_data_aux(1'b1),
                .rd_en      (1'b1),
                .rd_addr    (qp_index_acknack_ff),
                .rd_data    (phase_bit_acknack_psn_ff_qp_disable_rd_data_2)
            );

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_ff_qp_disable_3 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || phase_bit_acknack_psn_ff_qp_disable_valid_aux1),
                .wr_addr    (init_start ? init_count : phase_bit_acknack_psn_ff_qp_disable_id_aux1),
                .wr_data    (1'b0),
                .wr_en_aux  (phase_bit_acknack_psn_ff_qp_disable_valid),
                .wr_addr_aux(phase_bit_acknack_psn_ff_qp_disable_id),
                .wr_data_aux(1'b1),
                .rd_en      (1'b1),
                .rd_addr    (qp_index_acknack_ff),
                .rd_data    (phase_bit_acknack_psn_ff_qp_disable_rd_data_3)
            );

            assign phase_bit_acknack_psn_ff_rd_data_1 = (phase_bit_acknack_psn_ff_qp_disable_rd_data_2 & phase_bit_acknack_psn_ff_qp_disable_rd_data_3) ? 1'b0 : phase_bit_acknack_psn_ff_rd_data_int_1;

            // The last acnacked_psn is stored to identify the roll-over
            // condition. If the last_acknacked_psn is greater than acknacked_psn
            // it means rollover has happened
            // for the first outgoing packet, the psn -1 is saved as the last
            // acknacked psn

            always_ff @(posedge core_clk) begin : last_acknacked_psn_ff
                if (~core_rstn) begin
                    last_acknacked_psn_valid     <= 1'b0;
                    last_acknacked_psn_id        <= {C_QP_INDX_WIDTH{1'b0}};
                    last_acknacked_psn_value     <= {24{1'b0}};
                    last_acknacked_psn_valid_aux <= 1'b0;
                    last_acknacked_psn_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
                    last_acknacked_psn_value_aux <= {24{1'b0}};
                end else begin
                    if (first_outgoing_pkt_ff_rd_data && i_wqe_push_req) begin
                        last_acknacked_psn_valid <= 1'b1;
                        last_acknacked_psn_id    <= i_wqe_qpid;
                        last_acknacked_psn_value <= i_wqe_send_start_psn - 1'b1;
                    end else if (clr_timeout_n_nack_set_valid && ~(i_acknack_valid && i_qp_index_acknack == clr_timeout_n_nack_set_id)) begin
                        last_acknacked_psn_valid <= 1'b1;
                        last_acknacked_psn_id    <= clr_timeout_n_nack_set_id;
                        last_acknacked_psn_value <= psn_to_retry - 1'b1;
                    end else if (last_acknacked_psn_valid_pending && ~(i_acknack_valid && i_qp_index_acknack == acknacked_psn_id_pending)) begin
                        last_acknacked_psn_valid <= 1'b1;
                        last_acknacked_psn_id    <= acknacked_psn_id_pending;
                        last_acknacked_psn_value <= acknacked_psn_value_pending;
                    end else begin
                        last_acknacked_psn_valid <= 1'b0;
                    end

                    if (i_acknack_valid && ~(first_outgoing_pkt_ff_rd_data && i_wqe_push_req && i_wqe_qpid == i_qp_index_acknack)) begin
                        last_acknacked_psn_valid_aux <= 1'b1;
                        last_acknacked_psn_id_aux    <= i_qp_index_acknack;
                        last_acknacked_psn_value_aux <= acknacked_psn_ff_rd_data_0;
                    end else begin
                        last_acknacked_psn_valid_aux <= 1'b0;
                    end
                end
            end

            always_ff @(posedge core_clk) begin
                if (~core_rstn) begin
                    last_acknacked_psn_valid_pending <= 1'b0;
                    last_acknacked_psn_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                    last_acknacked_psn_value_pending <= {24{1'b0}};
                end else begin
                    //clr_timeout_n_nack is low frequency interface; hence write can wait;
                    if (first_outgoing_pkt_ff_rd_data && i_wqe_push_req && clr_timeout_n_nack_set_valid) begin
                        last_acknacked_psn_valid_pending <= 1'b1;
                        last_acknacked_psn_id_pending    <= clr_timeout_n_nack_set_id;
                        last_acknacked_psn_value_pending <= psn_to_retry - 1'b1;
                    end else if (last_acknacked_psn_valid_pending && ((last_acknacked_psn_valid && last_acknacked_psn_id == last_acknacked_psn_id_pending) || (last_acknacked_psn_valid_aux && last_acknacked_psn_id_aux == last_acknacked_psn_id_pending))) begin
                        last_acknacked_psn_valid_pending <= 1'b0;
                    end
                end
            end

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(24),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_last_acknacked_psn_ff_0 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || last_acknacked_psn_valid),
                .wr_addr    (init_start ? init_count : last_acknacked_psn_id),
                .wr_data    (init_start ? {24{1'b0}} : last_acknacked_psn_value),
                .wr_en_aux  (last_acknacked_psn_valid_aux),
                .wr_addr_aux(last_acknacked_psn_id_aux),
                .wr_data_aux(last_acknacked_psn_value_aux),
                .rd_en      (1'b1),
                .rd_addr    (osq_sampled_arbitrated_idx),
                .rd_data    (last_acknacked_psn_ff_rd_data_int_0)
            );

            assign last_acknacked_psn_ff_rd_data_0 = (last_acknacked_psn_valid_pending && last_acknacked_psn_id_pending == osq_sampled_arbitrated_idx) ? last_acknacked_psn_value_pending : last_acknacked_psn_ff_rd_data_int_0;

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(24),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_last_acknacked_psn_ff_1 (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || last_acknacked_psn_valid),
                .wr_addr    (init_start ? init_count : last_acknacked_psn_id),
                .wr_data    (init_start ? {24{1'b0}} : last_acknacked_psn_value),
                .wr_en_aux  (last_acknacked_psn_valid_aux),
                .wr_addr_aux(last_acknacked_psn_id_aux),
                .wr_data_aux(last_acknacked_psn_value_aux),
                .rd_en      (1'b1),
                .rd_addr    (i_qp_index_acknack),
                .rd_data    (last_acknacked_psn_ff_rd_data_int_1)
            );

            assign last_acknacked_psn_ff_rd_data_1 = (last_acknacked_psn_valid_pending && last_acknacked_psn_id_pending == i_qp_index_acknack) ? last_acknacked_psn_value_pending : last_acknacked_psn_ff_rd_data_int_1;



            always_ff @(posedge core_clk) begin : acknacked_msn_ff
                if (~core_rstn) begin
                    acknacked_msn_ff_valid     <= 1'b0;
                    acknacked_msn_ff_id        <= {C_QP_INDX_WIDTH{1'b0}};
                    acknacked_msn_ff_value     <= {24{1'b0}};
                    acknacked_msn_ff_valid_aux <= 1'b0;
                    acknacked_msn_ff_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
                    acknacked_msn_ff_value_aux <= {24{1'b0}};
                end else begin
                    if (i_acknack_valid) begin
                        acknacked_msn_ff_valid <= 1'b1;
                        acknacked_msn_ff_id    <= i_qp_index_acknack;
                        acknacked_msn_ff_value <= i_acknacked_msn[23:0];
                    end else begin
                        acknacked_msn_ff_valid <= 1'b0;
                    end

                    if (clr_timeout_n_nack_set_valid && ~(i_acknack_valid && clr_timeout_n_nack_set_id == i_qp_index_acknack)) begin
                        acknacked_msn_ff_valid_aux <= 1'b1;
                        acknacked_msn_ff_id_aux    <= clr_timeout_n_nack_set_id;
                        acknacked_msn_ff_value_aux <= msn_to_retry - 1'b1;
                    end else begin
                        acknacked_msn_ff_valid_aux <= 1'b0;
                    end
                end
            end

            dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(24),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_acknacked_msn_ff (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || acknacked_msn_ff_valid),
                .wr_addr    (init_start ? init_count : acknacked_msn_ff_id),
                .wr_data    (init_start ? {24{1'b0}} : acknacked_msn_ff_value),
                .wr_en_aux  (acknacked_msn_ff_valid_aux),
                .wr_addr_aux(acknacked_msn_ff_id_aux),
                .wr_data_aux(acknacked_msn_ff_value_aux),
                .rd_en      (1'b1),
                .rd_addr    (osq_sampled_arbitrated_idx),
                .rd_data    (acknacked_msn_ff_rd_data)
            );


            assign last_acknacked_psn_for_arbitrated_qp = last_acknacked_psn_ff_rd_data_0;
            assign last_acknacked_psn_for_curr_qp       = last_acknacked_psn_ff_rd_data_1[C_OSQ_PSN_WIDTH-1:0];
            assign acknacked_msn_for_arbitrated_qp      = acknacked_msn_ff_rd_data;
            assign o_qp_sq_psn_resp_idx                 = osq_sampled_arbitrated_idx;
            assign o_qp_sq_psn_resp_req                 = 1'b1;

        end  // flops_logic
    endgenerate  // end of FLOPS_TO_MEM == 1


    always_ff @(posedge core_clk) begin : qp_retried_ff
        if (~core_rstn) begin
            qp_retried_ff_reset_valid <= 1'b0;
            qp_retried_ff_reset_id    <= {C_QP_INDX_WIDTH{1'b0}};
            qp_retried_ff_set_valid   <= 1'b0;
            qp_retried_ff_set_id      <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (retransmit_reqd & i_retransmit_accepted) begin
                qp_retried_ff_set_valid <= 1'b1;
                qp_retried_ff_set_id    <= retransmit_qpid;
            end else begin
                qp_retried_ff_set_valid <= 1'b0;
            end

            if (i_acknack_valid && ~((retransmit_reqd & i_retransmit_accepted) && i_qp_index_acknack == retransmit_qpid)) begin
                qp_retried_ff_reset_valid <= 1'b1;
                qp_retried_ff_reset_id    <= i_qp_index_acknack;
            end else begin
                qp_retried_ff_reset_valid <= 1'b0;
            end
        end
    end

    always_ff @(posedge core_clk) begin : first_outgoing_pkt_2ff
        if (~core_rstn) begin
            first_outgoing_pkt_2ff_set_valid <= 1'b0;
            first_outgoing_pkt_2ff_set_id    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            first_outgoing_pkt_2ff_set_valid <= first_outgoing_pkt_ff_valid && first_outgoing_pkt_ff_data;
            first_outgoing_pkt_2ff_set_id    <= first_outgoing_pkt_ff_id;
        end
    end

    // `MSFF_RL(first_outgoing_pkt_ff[j], (i_qp_disable_pulse[j]? 1'b1 : ((i_wqe_push_req && (i_wqe_qpid == j)) ? 1'b0 : first_outgoing_pkt_ff[j])), 
    // core_clk, ~core_rstn, 1'b1)
    //`MSFF_RL(first_outgoing_pkt_2ff[j], first_outgoing_pkt_ff[j], core_clk, ~core_rstn, 1'b1)
    //###############################################################################################################################################
    // Assumption for first_outgoing_pkt_ff : Both i_qp_disable_pulse and i_wqe_push_req should have at least 1 clock gap
    //###############################################################################################################################################
    always_ff @(posedge core_clk) begin : first_outgoing_pkt_ff
        if (~core_rstn) begin
            first_outgoing_pkt_ff_valid         <= 1'b0;
            first_outgoing_pkt_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            first_outgoing_pkt_ff_data          <= 1'b0;
            first_outgoing_pkt_ff_valid_pending <= 1'b0;
            first_outgoing_pkt_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_qp_disable_pulse_set_valid_edge) begin
                first_outgoing_pkt_ff_valid <= 1'b1;
                first_outgoing_pkt_ff_id    <= i_qp_disable_pulse_set_id;
                first_outgoing_pkt_ff_data  <= 1'b1;
            end else if (i_wqe_push_req) begin
                first_outgoing_pkt_ff_valid <= 1'b1;
                first_outgoing_pkt_ff_id    <= i_wqe_qpid;
                first_outgoing_pkt_ff_data  <= 1'b0;
            end else if (first_outgoing_pkt_ff_valid_pending) begin
                first_outgoing_pkt_ff_valid <= 1'b1;
                first_outgoing_pkt_ff_id    <= first_outgoing_pkt_ff_id_pending;
                first_outgoing_pkt_ff_data  <= 1'b0;
            end else begin
                first_outgoing_pkt_ff_valid <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge && i_wqe_push_req && i_qp_disable_pulse_set_id != i_wqe_qpid) begin
                first_outgoing_pkt_ff_valid_pending <= 1'b1;
                first_outgoing_pkt_ff_id_pending    <= i_wqe_qpid;
            end else begin
                first_outgoing_pkt_ff_valid_pending <= 1'b0;
            end
        end
    end



    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_first_outgoing_pkt_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || first_outgoing_pkt_ff_valid),
        .wr_addr (init_start ? init_count : first_outgoing_pkt_ff_id),
        .wr_data (init_start ? 1'b1 : first_outgoing_pkt_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_wqe_qpid),
        .rd_data (first_outgoing_pkt_ff_rd_data_int)
    );

    assign first_outgoing_pkt_ff_rd_data = (first_outgoing_pkt_ff_valid_pending && first_outgoing_pkt_ff_id_pending == i_wqe_qpid) ? 1'b0 : first_outgoing_pkt_ff_rd_data_int;

    //###############################################################################################################################################
    // There should be altleast 1 clock gap between i_qp_disable_pulse_set_valid_edge and clr_timeout_n_nack_set_valid
    //###############################################################################################################################################
    //`MSFF_RL(first_outstanding_pkt_ff[j], ((clr_timeout_n_nack[j] | i_qp_disable_pulse[j]) ? 1'b1: ((i_wqe_push_req && (i_wqe_qpid == j)) ? 1'b0 : 
    // first_outstanding_pkt_ff[j])), core_clk, ~core_rstn, 1'b1)

    always_ff @(posedge core_clk) begin : first_outstanding_pkt_ff
        if (~core_rstn) begin
            first_outstanding_pkt_ff_valid         <= 1'b0;
            first_outstanding_pkt_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            first_outstanding_pkt_ff_data          <= 1'b0;
            first_outstanding_pkt_ff_valid_aux     <= 1'b0;
            first_outstanding_pkt_ff_id_aux        <= {C_QP_INDX_WIDTH{1'b0}};
            first_outstanding_pkt_ff_data_aux      <= 1'b0;
            first_outstanding_pkt_ff_valid_pending <= 1'b0;
            first_outstanding_pkt_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_wqe_push_req && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == i_wqe_qpid) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_wqe_qpid)) begin
                first_outstanding_pkt_ff_valid <= 1'b1;
                first_outstanding_pkt_ff_id    <= i_wqe_qpid;
                first_outstanding_pkt_ff_data  <= 1'b0;
            end else begin
                first_outstanding_pkt_ff_valid <= 1'b0;
            end

            if (clr_timeout_n_nack_set_valid) begin
                first_outstanding_pkt_ff_valid_aux <= 1'b1;
                first_outstanding_pkt_ff_id_aux    <= clr_timeout_n_nack_set_id;
                first_outstanding_pkt_ff_data_aux  <= 1'b1;
            end else if (i_qp_disable_pulse_set_valid_edge) begin
                first_outstanding_pkt_ff_valid_aux <= 1'b1;
                first_outstanding_pkt_ff_id_aux    <= i_qp_disable_pulse_set_id;
                first_outstanding_pkt_ff_data_aux  <= 1'b1;
            end else if (first_outstanding_pkt_ff_valid_pending && !(i_wqe_push_req && i_wqe_qpid == first_outstanding_pkt_ff_id_pending)) begin
                first_outstanding_pkt_ff_valid_aux <= 1'b1;
                first_outstanding_pkt_ff_id_aux    <= first_outstanding_pkt_ff_id_pending;
                first_outstanding_pkt_ff_data_aux  <= 1'b1;
            end else begin
                first_outstanding_pkt_ff_valid_aux <= 1'b0;
            end

            if (clr_timeout_n_nack_set_valid && i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id != clr_timeout_n_nack_set_id) begin
                first_outstanding_pkt_ff_valid_pending <= 1'b1;
                first_outstanding_pkt_ff_id_pending    <= i_qp_disable_pulse_set_id;
            end else begin
                first_outstanding_pkt_ff_valid_pending <= 1'b0;
            end
        end
    end

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_first_outstanding_pkt_ff (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || first_outstanding_pkt_ff_valid_aux),
        .wr_addr    (init_start ? init_count : first_outstanding_pkt_ff_id_aux),
        .wr_data    (1'b1),
        .wr_en_aux  (first_outstanding_pkt_ff_valid),
        .wr_addr_aux(first_outstanding_pkt_ff_id),
        .wr_data_aux(first_outstanding_pkt_ff_data),
        .rd_en      (1'b1),
        .rd_addr    (i_wqe_qpid),
        .rd_data    (first_outstanding_pkt_ff_rd_data_int)
    );

    assign first_outstanding_pkt_ff_rd_data = (first_outstanding_pkt_ff_valid_pending && first_outstanding_pkt_ff_id_pending == i_wqe_qpid) ? 1'b1 : first_outstanding_pkt_ff_rd_data_int;

    always_ff @(posedge core_clk) begin
        i_wqe_qpid_r <= i_wqe_qpid;
    end

    // Apart from generating a different counter value for the timeout, nothing
    // special is being done for RNR_NAK. when this special counter expires, the
    // FSM will retry those messages.

    assign osq_nacked_cur_qp  = (i_acknack_valid && nak_seq_error && ~(i_halt && i_halted_qpid == i_qp_index_acknack)) ? 1'b1 : osq_nacked_ff_rd_data_3;

    assign osq_nacked_ff_comb = (clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_qp_index_acknack) ? 1'b0 : ((i_acknack_valid && nak_seq_error && ~(i_halt && i_halted_qpid == i_qp_index_acknack)) | osq_nacked_ff_rd_data_3);

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (qp_index_acknack_ff),
        .rd_data (osq_nacked_ff_rd_data_int_0)
    );
    assign osq_nacked_ff_rd_data_0 = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == qp_index_acknack_ff) ? 1'b1 : osq_nacked_ff_rd_data_int_0;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_1 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_wqe_qpid),
        .rd_data (osq_nacked_ff_rd_data_int_1)
    );
    assign osq_nacked_ff_rd_data_1 = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == i_wqe_qpid) ? 1'b1 : osq_nacked_ff_rd_data_int_1;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_2 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (osq_sampled_arbitrated_idx),
        .rd_data (osq_nacked_ff_rd_data_int_2)
    );
    assign osq_nacked_ff_rd_data_2 = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == osq_sampled_arbitrated_idx) ? 1'b1 : osq_nacked_ff_rd_data_int_2;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_3 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_index_acknack),
        .rd_data (osq_nacked_ff_rd_data_int_3)
    );
    assign osq_nacked_ff_rd_data_3 = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == i_qp_index_acknack) ? 1'b1 : osq_nacked_ff_rd_data_int_3;

    //###############################################################################################################################################
    //Atleast 1 clock gap in clr_timeout_n_nack_set_valid and i_acknack_valid
    //###############################################################################################################################################
    always_ff @(posedge core_clk) begin : osq_nacked_ff
        if (~core_rstn) begin
            osq_nacked_ff_valid         <= 1'b0;
            osq_nacked_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_nacked_ff_data          <= 1'b0;
            osq_nacked_ff_valid_pending <= 1'b0;
            osq_nacked_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (clr_timeout_n_nack_set_valid) begin
                osq_nacked_ff_valid <= 1'b1;
                osq_nacked_ff_id    <= clr_timeout_n_nack_set_id;
                osq_nacked_ff_data  <= 1'b0;
            end else if (i_acknack_valid && nak_seq_error && ~(i_halt && i_halted_qpid == i_qp_index_acknack)) begin
                osq_nacked_ff_valid <= 1'b1;
                osq_nacked_ff_id    <= i_qp_index_acknack;
                osq_nacked_ff_data  <= 1'b1;
            end else if (osq_nacked_ff_valid_pending) begin
                osq_nacked_ff_valid <= 1'b1;
                osq_nacked_ff_id    <= osq_nacked_ff_id_pending;
                osq_nacked_ff_data  <= 1'b1;
            end else begin
                osq_nacked_ff_valid <= 1'b0;
            end

            if (clr_timeout_n_nack_set_valid && i_acknack_valid && nak_seq_error && !(i_halt && i_halted_qpid == i_qp_index_acknack) && clr_timeout_n_nack_set_id != i_qp_index_acknack) begin
                osq_nacked_ff_valid_pending <= 1'b1;
                osq_nacked_ff_id_pending    <= i_qp_index_acknack;
            end else begin
                osq_nacked_ff_valid_pending <= 1'b0;
            end
        end
    end

    //###############################################################################################################################################
    //3 clock gap minimimum in each i_qp_disable_pulse_set_valid_edge and osq_timed_out_set_valid and i_acknack_valid
    //###############################################################################################################################################
    //assign reload_retry_cnt[j]  = ((i_acknack_valid & (j == i_qp_index_acknack ) & (rnr_nak_error | nak_seq_error)) | (osq_timed_out[j])) ?
    //1'b0 : ((i_acknack_valid & (j == i_qp_index_acknack ) & (~rnr_nak_error & ~nak_seq_error))  ? 1'b1 : reload_retry_cnt_ff[j]);
    //`MSFF_RL(reload_retry_cnt_ff[j],(i_qp_disable_pulse[j] ? 1'b1 : reload_retry_cnt[j]), core_clk, ~core_rstn, 1'b1)
    always_ff @(posedge core_clk) begin : reload_retry_cnt_ff
        if (!core_rstn) begin
            reload_retry_cnt_valid           <= 1'b0;
            reload_retry_cnt_id              <= {C_QP_INDX_WIDTH{1'b0}};
            reload_retry_cnt_data            <= 1'b0;
            reload_retry_cnt_valid_pending_1 <= 1'b0;
            reload_retry_cnt_id_pending_1    <= {C_QP_INDX_WIDTH{1'b0}};
            reload_retry_cnt_data_pending_1  <= 1'b0;
            reload_retry_cnt_valid_pending_0 <= 1'b0;
            reload_retry_cnt_id_pending_0    <= {C_QP_INDX_WIDTH{1'b0}};
            condition_reload_retry           <= 2'b00;
        end else begin
            if (i_qp_disable_pulse_set_valid_edge) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= i_qp_disable_pulse_set_id;
                reload_retry_cnt_data  <= 1'b1;
            end else if (osq_timed_out_set_valid) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= osq_timed_out_set_id;
                reload_retry_cnt_data  <= 1'b0;
            end else if (i_acknack_valid) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= i_qp_index_acknack;
                reload_retry_cnt_data  <= (!rnr_nak_error && !nak_seq_error);
            end else if (reload_retry_cnt_valid_pending_0) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= reload_retry_cnt_id_pending_0;
                reload_retry_cnt_data  <= 1'b0;
            end else if (reload_retry_cnt_valid_pending_1) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= reload_retry_cnt_id_pending_1;
                reload_retry_cnt_data  <= reload_retry_cnt_data_pending_1;
            end else begin
                reload_retry_cnt_valid <= 1'b0;
            end


            if (i_qp_disable_pulse_set_valid_edge && osq_timed_out_set_valid && i_qp_disable_pulse_set_id != osq_timed_out_set_id) begin
                reload_retry_cnt_valid_pending_0 <= 1'b1;
                reload_retry_cnt_id_pending_0    <= osq_timed_out_set_id;
            end else begin
                reload_retry_cnt_valid_pending_0 <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge && osq_timed_out_set_valid && i_acknack_valid && i_qp_disable_pulse_set_id != i_qp_index_acknack && osq_timed_out_set_id != i_qp_index_acknack) begin
                reload_retry_cnt_valid_pending_1 <= 1'b1;
                reload_retry_cnt_id_pending_1    <= i_qp_index_acknack;
                reload_retry_cnt_data_pending_1  <= (!rnr_nak_error && !nak_seq_error);
                condition_reload_retry           <= 2'b00;
            end else if (!i_qp_disable_pulse_set_valid_edge && osq_timed_out_set_valid && i_acknack_valid && osq_timed_out_set_id != i_qp_index_acknack) begin
                reload_retry_cnt_valid_pending_1 <= 1'b1;
                reload_retry_cnt_id_pending_1    <= i_qp_index_acknack;
                reload_retry_cnt_data_pending_1  <= (!rnr_nak_error && !nak_seq_error);
                condition_reload_retry           <= 2'b01;
            end else if (!osq_timed_out_set_valid && i_qp_disable_pulse_set_valid_edge && i_acknack_valid && i_qp_disable_pulse_set_id != i_qp_index_acknack) begin
                reload_retry_cnt_valid_pending_1 <= 1'b1;
                reload_retry_cnt_id_pending_1    <= i_qp_index_acknack;
                reload_retry_cnt_data_pending_1  <= (!rnr_nak_error && !nak_seq_error);
                condition_reload_retry           <= 2'b10;
            end else if (reload_retry_cnt_valid_pending_1 && condition_reload_retry == 2'b00 && !reload_retry_cnt_valid_pending_0) begin
                reload_retry_cnt_valid_pending_1 <= 1'b0;
            end else if (reload_retry_cnt_valid_pending_1 && condition_reload_retry == 2'b01 && (!i_qp_disable_pulse_set_valid_edge || (i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == reload_retry_cnt_id_pending_1))) begin
                reload_retry_cnt_valid_pending_1 <= 1'b0;
            end else if (reload_retry_cnt_valid_pending_1 && condition_reload_retry == 2'b10 && (!osq_timed_out_set_valid || (osq_timed_out_set_valid && osq_timed_out_set_id == reload_retry_cnt_id_pending_1))) begin
                reload_retry_cnt_valid_pending_1 <= 1'b0;
            end
        end
    end


    assign o_qp_retried_set_valid   = qp_retried_ff_set_valid;
    assign o_qp_retried_set_id      = qp_retried_ff_set_id;
    assign o_qp_retried_reset_valid = qp_retried_ff_reset_valid;
    assign o_qp_retried_reset_id    = qp_retried_ff_reset_id;

    assign rnr_nak_error            = i_pkt_nacksyndrome[7:5] == RNR_NAK_ERR ? 1'b1 : 1'b0;
    assign nak_seq_error            = i_pkt_nacksyndrome == NAK_SEQ_ERR ? 1'b1 : 1'b0;
    `MSFF_R(inc_nack_cnt, (i_acknack_valid & (rnr_nak_error | nak_seq_error) ? (inc_nack_cnt + 1'b1) : inc_nack_cnt), core_clk, ~core_rstn)
    `MSFF_R(qp_sq_psn_resp_valid_ff, i_qp_sq_psn_resp_valid, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_psn_ff, i_qp_sq_psn, core_clk, ~core_rstn)

    assign o_inc_nack_cnt = inc_nack_cnt;


    // 保留两组未完成的 FIFO。
    // 一个用于MAX PSN——RX数据包处理程序需要有关最大预期PSN的信息。
    // 因此，由 WQE-proc 模块生成的所有WQE，要么设置了显式ack位，要么是RDMA READ请求（因为RDMA READ总是有来自目标的显式ack）
    // MAX PSN 缓冲区的内容为:
    // PSN LSB - 8 bits
    // Length  - 24 bits
    // local buffer address (mainly reqd for RDMA READ WQEs) - 64 bits
    // Total = 96 bits
    // The other buffer is for the response handler module. For every acknowledged
    // WQE (explicit or inexplicit) the completion needs to be posted and the DB
    // needs to be rung to RDMAIf

    generate
        begin
            if (NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING) begin
                assign osq_in = {i_wqe_send_start_psn[C_OSQ_PSN_WIDTH-1:0], i_wqe_fatal_push, i_wqe_retried, i_wqe_data_bufid[14:0], i_wqe_send_msn[7:0], i_wqe_send_end_psn[C_OSQ_PSN_WIDTH-1:0], i_wqe_opcode, i_wqe_wrid};
            end else begin
                assign osq_in = {i_wqe_send_start_psn[C_OSQ_PSN_WIDTH-1:0], i_wqe_fatal_push, i_wqe_retried, i_wqe_data_bufid[14:0], i_wqe_send_msn[7:0], i_wqe_send_end_psn[C_OSQ_PSN_WIDTH-1:0], i_wqe_opcode, i_wqe_wrid};
            end
        end
    endgenerate
    `MSFF_R(osq_in_q, osq_in, core_clk, !core_rstn)
    `MSFF_R(osq_in_2q, osq_in_q, core_clk, !core_rstn)


    //----------------------------------------------------------------------------------
    // 未完成缓冲区(OSQ)
    // 所有 256 个 FIFO 的内存都被合并到一个 XPM 内存实例中，以便更好地优化内存利用率。
    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (OSBUF_MEM_SZ),     //positive integer
        .MEMORY_PRIMITIVE ("ultra"),          //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(OSBUF_ENTRY_SIZE),  //positive integer
        .BYTE_WRITE_WIDTH_A(OSBUF_ENTRY_SIZE),  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (OS_Q_ADDR_WDITH),   //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (OSBUF_ENTRY_SIZE),  //positive integer
        .ADDR_WIDTH_B      (OS_Q_ADDR_WDITH),   //positive integer
        .READ_RESET_VALUE_B("0"),               //string
        .READ_LATENCY_B    (2),                 //non-negative integer
        .WRITE_MODE_B      ("read_first")       //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_osq (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (osq_wr_addr_rd_valid),  //(i_wqe_push_req),
        .addra         (osq_wr_addr_new),
        .dina          (osq_in_2q),
        .injectsbiterra(1'b0),                  //do not change
        .injectdbiterra(1'b0),                  //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (1'b1),
        .regceb  (1'b1),
        .addrb   (osq_rd_addr_new),
        .doutb   ({osq_start_psn, osq_wqe_fatal_push, osq_wqe_retried, osq_data_bufid, osq_msn, osq_end_psn, osq_opcode, osq_wrid}),
        //.doutb          (osq_out),
        .sbiterrb(),                                                                                                                  //do not change
        .dbiterrb()                                                                                                                   //do not change

    );

    assign o_psn_to_retry = psn_to_retry;
    assign o_ssn_to_retry = msn_to_retry;
    //assign o_freeup_data_bufid = osq_data_bufid;

    // End of xpm_memory_sdpram instance declaration


    // 这是 osq 实现。实际内存由 XPM 内存实现。该模块管理内存的指针，并为队列生成 空/满 信号。
    // “OSQ” = Outstanding Send Queue（已发出但尚未完成/确认的发送工作队列条目集合）的管理结构，而不是软件最初提交 WQE 的原始 SQ。
    // 本质是对每个 QP 当前“在途(in-flight)”发送操作的硬件侧影子队列/计数器，用于流控、拥塞反馈、重传与仲裁决策。
    q_mgr_init_top #(
        .C_NUM_QUEUES   (C_NUM_QP),           // Number of queues
        .C_Q_DEPTH      (C_OS_Q_DEPTH),       // Number of entries in each queue
        .C_ENTRY_SIZE   (1),                  // *See below. Size of each entry in Bytes. SQ size is 64B
        .C_THRESHOLD    (5),                  // Threshold for generating almost full
        .C_ECN_THRESHOLD(C_ECN_THRESHOLD),    // Threshold for generating almost full
        .C_PTR_WIDTH    (C_OS_Q_INDX_WIDTH),  // Pointer size reqd for accessing the Q depth 
        .C_ADDR_WIDTH   (OS_Q_ADDR_WDITH)
        //* C_ENTRY_SIZE - This parameter is used to generate the address for the
        //memory which is likely in DDR and generates a BYTE address. However
        //since we are directly using this address for he BRAM which houses the
        //OSQ the address is entry address (as each entry houses the entire OSQ
        //content for one valida transaction). hence a size of 1 is given so that
        //the address maps to the correct entry.
    ) inst_osq_manager (
        // Clock and Reset
        .core_clk(core_clk),   // Module Clock  
        .core_rst(~core_rstn), // Active high core reset

        // Queue pointers
        .i_wr_base_addr    (osq_base_addr_2d[i_wqe_qpid]),
        .i_wr_addr_qnum    (i_wqe_qpid),
        .i_wr_addr_rd_en   (wqe_push_for_non_nacked_qp),    // 读取写地址使能，这里和 push 信号同步，表示这次 push 的地址
        .o_wr_addr         (osq_wr_addr_new),               // 最新写地址
        .o_wr_addr_rd_valid(osq_wr_addr_rd_valid),          // i_wr_addr_rd_en 拉高后两周期输出 o_wr_addr_rd_valid

        .i_rd_base_addr    (osq_base_addr_2d[osq_sampled_arbitrated_idx]),
        .i_rd_addr_qnum    (osq_sampled_arbitrated_idx),
        .i_rd_addr_rd_en   (rd_osq),                                        // 读取读地址使能
        .o_rd_addr         (osq_rd_addr_new),                               // 最新读地址
        .o_rd_addr_rd_valid(osq_rd_addr_rd_valid),                          // i_rd_addr_rd_en 拉高后两周期输出 o_rd_addr_rd_valid

        // 推操作结束，队列为满信号
        .o_q_full_set_valid  (osq_full_set_valid),
        .o_q_full_set_id     (osq_full_set_id),
        // 弹操作 / 刷新操作 结束，就会输出o_q_full_reset_*信号，说明队列满状态重置，不满了
        .o_q_full_reset_valid(osq_full_reset_valid),
        .o_q_full_reset_id   (osq_full_reset_id),

        // 在 push 写入后剩余空间 (i_q_depth - valid_entries) 小于等于阈值 (ECN 或普通)
        .o_q_almost_full_set_valid  (osq_almost_full_set_valid),
        .o_q_almost_full_set_id     (osq_almost_full_set_id),
        // 在 pop 写入后剩余空间重新大于阈值 (ECN 或普通)，或 flush 操作 结束
        .o_q_almost_full_reset_valid(osq_almost_full_reset_valid),
        .o_q_almost_full_reset_id   (osq_almost_full_reset_id),

        // 只要有推送，等待操作结束(回到PUSH_RD状态)，就会输出o_q_empty_reset_*信号，说明队列空状态重置，不空了
        .o_q_empty_reset_valid(osq_empty_reset_valid),
        .o_q_empty_reset_id   (osq_empty_reset_id),
        // 弹操作 / 刷新操作 结束，队列为空信号
        .o_q_empty_set_valid  (osq_empty_set_valid),
        .o_q_empty_set_id     (osq_empty_set_id),

        .vld_free_entry_rden    (1'b0),
        .vld_free_entry_qnum    ({C_QP_INDX_WIDTH{1'b0}}),
        .vld_free_entry_data_vld(),
        .o_num_valid_entries    (selected_num_valid_osq_entries),  // no use
        .o_num_free_entries     (num_free_osq_entries_nc),         // no use
        .i_q_flush              (1'b0),
        .i_q_flush_qpid         ({C_QP_INDX_WIDTH{1'b0}}),
        .o_q_flush_done         (),

        .i_q_depth              (C_OS_Q_DEPTH),
        .i_threshold_cntrl_valid(i_tx_path_congestion_valid),
        .i_threshold_cntrl_id   (i_tx_path_congestion_id),
        .i_threshold_cntrl_data (i_tx_path_congestion_data),

        // Push Pop functions
        .i_q_push     (wqe_push_for_non_nacked_qp),
        .i_q_push_qpid(i_wqe_qpid),
        // CK：这还能进一步优化吗？
        // pop_osq_from_fsm 函数是不是只有在特定队列的 OSQ 空值较低时才会生成？
        .i_q_pop      (pop_osq_from_fsm & !osq_empty_rd_data_0),
        .i_q_pop_qpid (osq_sampled_arbitrated_idx),
        .init_start   (init_start),
        .init_count   (init_count)
    );

    always_ff @(posedge core_clk) begin : osq_empty_ff
        if (!core_rstn) begin
            osq_empty_ff_valid         <= 1'b0;
            osq_empty_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_empty_ff_data          <= 1'b0;
            osq_empty_ff_valid_pending <= 1'b0;
            osq_empty_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (osq_empty_set_valid) begin  // 空
                osq_empty_ff_valid <= 1'b1;
                osq_empty_ff_id    <= osq_empty_set_id;
                osq_empty_ff_data  <= 1'b1;
            end else if (osq_empty_reset_valid) begin  // 不空
                osq_empty_ff_valid <= 1'b1;
                osq_empty_ff_id    <= osq_empty_reset_id;
                osq_empty_ff_data  <= 1'b0;
            end else if (osq_empty_ff_valid_pending) begin  // 不空
                osq_empty_ff_valid <= 1'b1;
                osq_empty_ff_id    <= osq_empty_ff_id_pending;
                osq_empty_ff_data  <= 1'b0;
            end else begin
                osq_empty_ff_valid <= 1'b0;
            end

            // 同时发生空和非空事件
            // 一个队列刚被 push（从空变非空）→ reset 脉冲。另一个队列同周期 pop 后变空 → set 脉冲。
            if (osq_empty_set_valid && osq_empty_reset_valid) begin
                osq_empty_ff_valid_pending <= 1'b1;
                osq_empty_ff_id_pending    <= osq_empty_reset_id;  // 非空的队列 ID
            end else begin
                osq_empty_ff_valid_pending <= 1'b0;
            end
        end
    end

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (osq_sampled_arbitrated_idx),
        .rd_data (osq_empty_rd_data_int_0)
    );
    // 如果 osq_sampled_arbitrated_idx 恰好就是这个 pending 的 reset 队列，当前真实状态已经“非空”，但 RAM 还没更新。为了不给 FSM 造成“误判为空而跳过处理”，用条件覆盖直接输出 0（非空），实现提前可见。
    assign osq_empty_rd_data_0 = (osq_empty_ff_valid_pending && osq_empty_ff_id_pending == osq_sampled_arbitrated_idx) ? 1'b0 : osq_empty_rd_data_int_0;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_1 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (o_osq_pop_qpid),
        .rd_data (osq_empty_rd_data_int_1)
    );
    // 如果 o_osq_pop_qpid 恰好就是这个 pending 的 reset 队列，当前真实状态已经“非空”，但 RAM 还没更新。为了不给 FSM 造成“误判为空而跳过处理”，用条件覆盖直接输出 0（非空），实现提前可见。
    assign osq_empty_rd_data_1 = (osq_empty_ff_valid_pending && osq_empty_ff_id_pending == o_osq_pop_qpid) ? 1'b0 : osq_empty_rd_data_int_1;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_2 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (osq_arbitrated_idx),
        .rd_data (osq_empty_rd_data_int_2)
    );
    // 如果 osq_arbitrated_idx 恰好就是这个 pending 的 reset 队列，当前真实状态已经“非空”，但 RAM 还没更新。为了不给 FSM 造成“误判为空而跳过处理”，用条件覆盖直接输出 0（非空），实现提前可见。
    assign osq_empty_rd_data_2 = (osq_empty_ff_valid_pending && osq_empty_ff_id_pending == osq_arbitrated_idx) ? 1'b0 : osq_empty_rd_data_int_2;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_3 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_fatal_id),
        .rd_data (osq_empty_rd_data_int_3)
    );
    // 如果 i_qp_fatal_id 恰好就是这个 pending 的 reset 队列，当前真实状态已经“非空”，但 RAM 还没更新。为了不给 FSM 造成“误判为空而跳过处理”，用条件覆盖直接输出 0（非空），实现提前可见。
    assign osq_empty_rd_data_3 = (osq_empty_ff_valid_pending && osq_empty_ff_id_pending == i_qp_fatal_id) ? 1'b0 : osq_empty_rd_data_int_3;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1),
        .C_COMBO_BYPASS(1)
    ) inst_osq_empty_4 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (osq_empty_ff_id),
        .rd_data (osq_empty_ff_id_rd_data)
    );

    // osq_empty And_reduce logic
    // 检测所有OSQ是否都为空
    always_ff @(posedge core_clk) begin : and_reduce_osq_empty
        if (~core_rstn) begin
            osq_empty_zeros_count <= {C_QP_INDX_WIDTH + 1{1'b0}};
        end else begin
            case ({
                osq_empty_ff_valid, osq_empty_ff_data, osq_empty_ff_id_rd_data
            })
                3'b110:  osq_empty_zeros_count <= osq_empty_zeros_count - 1;
                3'b101:  osq_empty_zeros_count <= osq_empty_zeros_count + 1;
                default: osq_empty_zeros_count <= osq_empty_zeros_count;
            endcase
        end
    end
    assign ack_nack_to_process = (osq_empty_zeros_count != 'd0) ? 1'b1 : 1'b0;  // 1-表示至少有一个 OSQ 非空，0-表示所有 OSQ 都为空

    //assign selected_num_valid_osq_entries = num_valid_osq_entries;
    `MSFF_R(num_valid_osq_entries_ff, selected_num_valid_osq_entries, core_clk, ~core_rstn)
    assign o_num_valid_osq_entries    = num_valid_osq_entries_ff;

    assign push_pop_1_entry           = {{C_OS_Q_INDX_WIDTH{1'b0}}, 1'b1};

    assign o_wqe_push_rdy             = 1'b1;  // 可稍后更改，以增加延迟

    assign o_osq_empty_set_valid      = osq_empty_set_valid;
    assign o_osq_empty_set_id         = osq_empty_set_id;
    assign o_osq_empty_reset_valid    = osq_empty_reset_valid;
    assign o_osq_empty_reset_id       = osq_empty_reset_id;

    assign wqe_push_for_non_nacked_qp = (osq_nacked_ff_rd_data_1 & i_osq_wqe_nack_resp_rd_data_0) ? 1'b0 : i_wqe_push_req;


    //###############################################################################################################################################
    //`MSFF_R(osq_timed_out_sticky[j], ((i_qp_disable_pulse[j] | clr_timeout_n_nack[j] | timer_reload_for_acknack_ff[j])? 1'b0 : osq_timed_out_sticky_wire[j]),
    // core_clk, ~core_rstn)

    always_ff @(posedge core_clk) begin : osq_timed_out_sticky
        if (!core_rstn) begin
            osq_timed_out_sticky_valid             <= 1'b0;
            osq_timed_out_sticky_id                <= {C_QP_INDX_WIDTH{1'b0}};
            osq_timed_out_sticky_data              <= 1'b0;
            osq_timed_out_sticky_valid_aux         <= 1'b0;
            osq_timed_out_sticky_id_aux            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_timed_out_sticky_data_aux          <= 1'b0;
            osq_timed_out_sticky_valid_pending     <= 1'b0;
            osq_timed_out_sticky_id_pending        <= {C_QP_INDX_WIDTH{1'b0}};
            osq_timed_out_sticky_valid_aux_pending <= 1'b0;
            osq_timed_out_sticky_id_aux_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_qp_disable_pulse_set_valid_edge) begin
                osq_timed_out_sticky_valid <= 1'b1;
                osq_timed_out_sticky_id    <= i_qp_disable_pulse_set_id;
                osq_timed_out_sticky_data  <= 1'b0;
            end else if (clr_timeout_n_nack_set_valid) begin
                osq_timed_out_sticky_valid <= 1'b1;
                osq_timed_out_sticky_id    <= clr_timeout_n_nack_set_id;
                osq_timed_out_sticky_data  <= 1'b0;
            end else if (osq_timed_out_sticky_valid_pending && !(((i_acknack_valid && ~rnr_nak_error) || osq_nacked_ff_comb || i_acknack_valid) && i_qp_index_acknack == osq_timed_out_sticky_id_pending) && !(osq_timed_out_set_valid && osq_timed_out_set_id == osq_timed_out_sticky_id_pending)) begin
                osq_timed_out_sticky_valid <= 1'b1;
                osq_timed_out_sticky_id    <= osq_timed_out_sticky_id_pending;
                osq_timed_out_sticky_data  <= 1'b0;
            end else begin
                osq_timed_out_sticky_valid <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge && clr_timeout_n_nack_set_valid && i_qp_disable_pulse_set_id != clr_timeout_n_nack_set_id) begin
                osq_timed_out_sticky_valid_pending <= 1'b1;
                osq_timed_out_sticky_id_pending    <= clr_timeout_n_nack_set_id;
            end else begin
                osq_timed_out_sticky_valid_pending <= 1'b0;
            end

            if (((i_acknack_valid && ~rnr_nak_error) || osq_nacked_ff_comb || i_acknack_valid) && !(i_qp_disable_pulse_set_valid_edge && i_qp_index_acknack == i_qp_disable_pulse_set_id) && !(clr_timeout_n_nack_set_valid && i_qp_index_acknack == clr_timeout_n_nack_set_id)) begin
                osq_timed_out_sticky_valid_aux <= 1'b1;
                osq_timed_out_sticky_id_aux    <= i_qp_index_acknack;
                osq_timed_out_sticky_data_aux  <= 1'b0;
            end else if (osq_timed_out_set_valid && !(i_qp_disable_pulse_set_valid_edge && osq_timed_out_set_id == i_qp_disable_pulse_set_id) && !(clr_timeout_n_nack_set_valid && osq_timed_out_set_id == clr_timeout_n_nack_set_id)) begin
                osq_timed_out_sticky_valid_aux <= 1'b1;
                osq_timed_out_sticky_id_aux    <= osq_timed_out_set_id;
                osq_timed_out_sticky_data_aux  <= 1'b1;
            end else if (osq_timed_out_sticky_valid_aux_pending && !(i_qp_disable_pulse_set_valid_edge && osq_timed_out_sticky_id_aux_pending == i_qp_disable_pulse_set_id) && !(clr_timeout_n_nack_set_valid && osq_timed_out_sticky_id_aux_pending == clr_timeout_n_nack_set_id)) begin
                osq_timed_out_sticky_valid_aux <= 1'b1;
                osq_timed_out_sticky_id_aux    <= osq_timed_out_sticky_id_aux_pending;
                osq_timed_out_sticky_data_aux  <= 1'b1;
            end else begin
                osq_timed_out_sticky_valid_aux <= 1'b0;
            end

            if (osq_timed_out_set_valid && ((i_acknack_valid && ~rnr_nak_error) || osq_nacked_ff_comb || i_acknack_valid) && i_qp_index_acknack != osq_timed_out_set_id && !(i_qp_disable_pulse_set_valid_edge 
		&& (osq_timed_out_set_id == i_qp_disable_pulse_set_id || i_qp_index_acknack == i_qp_disable_pulse_set_id)) && !(clr_timeout_n_nack_set_valid && (osq_timed_out_set_id == clr_timeout_n_nack_set_id
		|| i_qp_index_acknack == clr_timeout_n_nack_set_id))) begin
                osq_timed_out_sticky_valid_aux_pending <= 1'b1;
                osq_timed_out_sticky_id_aux_pending    <= osq_timed_out_set_id;
            end else begin
                osq_timed_out_sticky_valid_aux_pending <= 1'b0;
            end

        end
    end

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_timed_out_0 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || osq_timed_out_sticky_valid),
        .wr_addr    (init_start ? init_count : osq_timed_out_sticky_id),
        .wr_data    (init_start ? 1'b0 : osq_timed_out_sticky_data),
        .wr_en_aux  (osq_timed_out_sticky_valid_aux),
        .wr_addr_aux(osq_timed_out_sticky_id_aux),
        .wr_data_aux(osq_timed_out_sticky_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (fsm_i_arbitrated_indx),
        .rd_data    (osq_timed_out_sticky_rd_data_int_0)
    );
    always_comb begin
        if (osq_timed_out_sticky_valid_pending && osq_timed_out_sticky_id_pending == fsm_i_arbitrated_indx) begin
            osq_timed_out_sticky_rd_data_0 = 1'b0;
        end else if (osq_timed_out_sticky_valid_aux_pending && osq_timed_out_sticky_id_aux_pending == fsm_i_arbitrated_indx) begin
            osq_timed_out_sticky_rd_data_0 = 1'b1;
        end else begin
            osq_timed_out_sticky_rd_data_0 = osq_timed_out_sticky_rd_data_int_0;
        end
    end
    `MSFF_R(osq_timed_out_sticky_rd_data_0_ff, osq_timed_out_sticky_rd_data_0, core_clk, !core_rstn)

    assign osq_timed_out_sticky_fsm_arbitrated_indx = osq_timed_out_sticky_rd_data_0_ff;


    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_timed_out_1 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || osq_timed_out_sticky_valid),
        .wr_addr    (init_start ? init_count : osq_timed_out_sticky_id),
        .wr_data    (init_start ? 1'b0 : osq_timed_out_sticky_data),
        .wr_en_aux  (osq_timed_out_sticky_valid_aux),
        .wr_addr_aux(osq_timed_out_sticky_id_aux),
        .wr_data_aux(osq_timed_out_sticky_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (timer_mem_rdaddr),
        .rd_data    (osq_timed_out_sticky_rd_data_int_1)
    );
    always_comb begin
        if (osq_timed_out_sticky_valid_pending && osq_timed_out_sticky_id_pending == timer_mem_rdaddr) begin
            osq_timed_out_sticky_rd_data_1 = 1'b0;
        end else if (osq_timed_out_sticky_valid_aux_pending && osq_timed_out_sticky_id_aux_pending == timer_mem_rdaddr) begin
            osq_timed_out_sticky_rd_data_1 = 1'b1;
        end else begin
            osq_timed_out_sticky_rd_data_1 = osq_timed_out_sticky_rd_data_int_1;
        end
    end

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_rnr_nacked_ff_comp (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_rnr_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_rnr_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_rnr_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr),
        .rd_data (osq_rnr_nacked_ff_rd_data_comp_int)
    );
    assign osq_rnr_nacked_ff_rd_data_comp = (osq_rnr_nacked_ff_valid_pending && osq_rnr_nacked_ff_id_pending == timer_mem_rdaddr) ? 1'b0 : osq_rnr_nacked_ff_rd_data_comp_int;
    `MSFF_R(osq_rnr_nacked_ff_rd_data_comp_ff, osq_rnr_nacked_ff_rd_data_comp, core_clk, !core_rstn)

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_comp (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr),
        .rd_data (osq_nacked_ff_rd_data_comp_int)
    );
    assign osq_nacked_ff_rd_data_comp = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == timer_mem_rdaddr) ? 1'b1 : osq_nacked_ff_rd_data_comp_int;
    `MSFF_R(osq_nacked_ff_rd_data_comp_ff, osq_nacked_ff_rd_data_comp, core_clk, !core_rstn)

    `MSFF_R(osq_nacked_timer_mem_rdaddr_i, osq_timed_out_sticky_rd_data_1, core_clk, !core_rstn)
    assign osq_nacked_timer_mem_rdaddr = osq_nacked_timer_mem_rdaddr_i || osq_nacked_ff_rd_data_comp_ff || osq_rnr_nacked_ff_rd_data_comp_ff;

    // {set_valid,set_id,set_valid_pending,set_id_pending,reset_valid,reset_id,reset_valid_pending,reset_id_pending}
    assign o_osq_nacked                = {o_osq_nacked_set_valid, o_osq_nacked_set_id, o_osq_nacked_set_valid_pending, o_osq_nacked_set_id_pending, o_osq_nacked_reset_valid, o_osq_nacked_reset_id, o_osq_nacked_reset_valid_pending, o_osq_nacked_reset_id_pending};

    always_ff @(posedge core_clk) begin : o_osq_nacked_proc
        if (!core_rstn) begin
            o_osq_nacked_set_valid           <= 1'b0;
            o_osq_nacked_set_id              <= {C_QP_INDX_WIDTH{1'b0}};
            o_osq_nacked_reset_valid         <= 1'b0;
            o_osq_nacked_reset_id            <= {C_QP_INDX_WIDTH{1'b0}};
            o_osq_nacked_set_valid_pending   <= 1'b0;
            o_osq_nacked_set_id_pending      <= {C_QP_INDX_WIDTH{1'b0}};
            o_osq_nacked_reset_valid_pending <= 1'b0;
            o_osq_nacked_reset_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (osq_timed_out_set_valid && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == osq_timed_out_set_id) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == osq_timed_out_set_id) && !(i_acknack_valid && i_qp_index_acknack == osq_timed_out_set_id)) begin
                o_osq_nacked_set_valid <= 1'b1;  //timeout
                o_osq_nacked_set_id    <= osq_timed_out_set_id;
            end else if (i_acknack_valid && nak_seq_error && !(i_halt && i_halted_qpid == i_qp_index_acknack) && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == i_qp_index_acknack) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_qp_index_acknack)) begin
                o_osq_nacked_set_valid <= 1'b1;  //nack
                o_osq_nacked_set_id    <= i_qp_index_acknack;
            end else if (i_acknack_valid && rnr_nak_error && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == i_qp_index_acknack) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_qp_index_acknack)) begin
                o_osq_nacked_set_valid <= 1'b1;  //rnr_nack
                o_osq_nacked_set_id    <= i_qp_index_acknack;
            end else if (o_osq_nacked_set_valid_pending && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == o_osq_nacked_set_id_pending) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == o_osq_nacked_set_id_pending)) begin
                o_osq_nacked_set_valid <= 1'b1;
                o_osq_nacked_set_id    <= o_osq_nacked_set_id_pending;
            end else begin
                o_osq_nacked_set_valid <= 1'b0;
            end

            if ((i_qp_disable_pulse_set_valid_edge && (i_qp_disable_pulse_set_id == osq_timed_out_set_id || i_qp_disable_pulse_set_id == i_qp_index_acknack)) || (clr_timeout_n_nack_set_valid && 
		(clr_timeout_n_nack_set_id == osq_timed_out_set_id || clr_timeout_n_nack_set_id == i_qp_index_acknack))) begin
                o_osq_nacked_set_valid_pending <= 1'b0;
            end else if (osq_timed_out_set_valid && i_acknack_valid && ((nak_seq_error && !(i_halt && i_halted_qpid == i_qp_index_acknack)) || rnr_nak_error) && osq_timed_out_set_id != i_qp_index_acknack) begin
                o_osq_nacked_set_valid_pending <= 1'b1;
                o_osq_nacked_set_id_pending    <= i_qp_index_acknack;
            end else begin
                o_osq_nacked_set_valid_pending <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge) begin
                o_osq_nacked_reset_valid <= 1'b1;  //qp_reset
                o_osq_nacked_reset_id    <= i_qp_disable_pulse_set_id;
            end else if (clr_timeout_n_nack_set_valid) begin
                o_osq_nacked_reset_valid <= 1'b1;  //fsm clear req
                o_osq_nacked_reset_id    <= clr_timeout_n_nack_set_id;
            end else if (o_osq_nacked_reset_valid_pending && !(osq_timed_out_set_valid && !(i_acknack_valid && i_qp_index_acknack == osq_timed_out_set_id) && osq_timed_out_set_id
		== o_osq_nacked_reset_id_pending) && !(i_acknack_valid && ((nak_seq_error && !(i_halt && i_halted_qpid == i_qp_index_acknack)) || rnr_nak_error) && i_qp_index_acknack
		== o_osq_nacked_reset_id_pending)) begin
                o_osq_nacked_reset_valid <= 1'b1;
                o_osq_nacked_reset_id    <= o_osq_nacked_reset_id_pending;
            end else begin
                o_osq_nacked_reset_valid <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge && clr_timeout_n_nack_set_valid && i_qp_disable_pulse_set_id != clr_timeout_n_nack_set_id) begin
                o_osq_nacked_reset_valid_pending <= 1'b1;
                o_osq_nacked_reset_id_pending    <= clr_timeout_n_nack_set_id;
            end else begin
                o_osq_nacked_reset_valid_pending <= 1'b0;
            end
        end
    end

    genvar i;
    generate
        for (i = 0; i < C_NUM_QP; i = i + 1) begin : gen_osq_param
            assign osq_base_addr_2d[i] = (i * C_OS_Q_DEPTH);
        end
    endgenerate


    assign o_osq_pop_qpid = osq_sampled_arbitrated_idx;
    assign o_osq_pop      = pop_osq_from_fsm & ~osq_empty_rd_data_1;

    //`MSFF_R(wqe_cmpl_intr_sts[n], (i_clr_wqe_cmpl_sts[n] ? 1'b0 : ((arbitrated_idx_for_intr == n) ? (intr_wqe_cmpl_sts || wqe_cmpl_intr_sts[n]) : 
    //                                                                    wqe_cmpl_intr_sts[n])) , core_clk, ~core_rstn)

    genvar k;
    generate
        if (C_NUM_QP > 32) begin : wqe_cmpl_intr_sts
            reg [(clog2(C_NUM_QP/32)) : 0] num_ones_in_cmpl_intr_lut[32];
            //assuming 32 bit set at a time for i_clr_wqe_cmpl_sts
            //Should be atleast 1 clock gap for consecutive assertion of both |i_clr_wqe_cmpl_sts_set_valid and intr_wqe_cmpl_sts
            always_ff @(posedge core_clk) begin : wqe_cmpl_intr_sts_proc
                if (~core_rstn) begin
                    wqe_cmpl_intr_sts_valid         <= {32{1'b0}};
                    wqe_cmpl_intr_sts_data          <= {32{1'b0}};
                    wqe_cmpl_intr_sts_valid_int     <= {32{1'b0}};
                    wqe_cmpl_intr_sts_id            <= {C_QP_INDX_WIDTH{1'b0}};
                    wqe_cmpl_intr_sts_valid_pending <= {32{1'b0}};
                    wqe_cmpl_intr_sts_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    if (|i_clr_wqe_cmpl_sts_set_valid) begin
                        wqe_cmpl_intr_sts_valid     <= i_clr_wqe_cmpl_sts_set_valid;
                        wqe_cmpl_intr_sts_id        <= i_clr_wqe_cmpl_sts_set_id;
                        wqe_cmpl_intr_sts_valid_int <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data      <= {32{1'b0}};
                    end else if (intr_wqe_cmpl_sts) begin
                        wqe_cmpl_intr_sts_valid_int[arbitrated_idx_for_intr[4:0]] <= 1'b1;
                        wqe_cmpl_intr_sts_id                                      <= {arbitrated_idx_for_intr[C_QP_INDX_WIDTH-1 : 5], 5'd0};
                        wqe_cmpl_intr_sts_valid                                   <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data                                    <= {32{1'b1}};
                    end else if (|wqe_cmpl_intr_sts_valid_pending) begin
                        wqe_cmpl_intr_sts_valid_int <= wqe_cmpl_intr_sts_valid_pending;
                        wqe_cmpl_intr_sts_id        <= wqe_cmpl_intr_sts_id_pending;
                        wqe_cmpl_intr_sts_valid     <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data      <= {32{1'b1}};
                    end else begin
                        wqe_cmpl_intr_sts_valid     <= {32{1'b0}};
                        wqe_cmpl_intr_sts_valid_int <= {32{1'b0}};
                    end

                    if (intr_wqe_cmpl_sts && !(i_clr_wqe_cmpl_sts_set_valid[arbitrated_idx_for_intr[4:0]] == 1'b1 && arbitrated_idx_for_intr[C_QP_INDX_WIDTH-1:5] == wqe_cmpl_intr_sts_id[C_QP_INDX_WIDTH-1:5])) begin
                        wqe_cmpl_intr_sts_valid_pending[arbitrated_idx_for_intr[4:0]] <= 1'b1;
                        wqe_cmpl_intr_sts_id_pending                                  <= {arbitrated_idx_for_intr[C_QP_INDX_WIDTH-1 : 5], 5'd0};
                    end else begin
                        wqe_cmpl_intr_sts_valid_pending <= {32{1'b0}};
                    end
                end
            end

            for (k = 0; k <= 31; k = k + 1) begin
                sdp_lut_ram_wrapper #(
                    .C_RAM_WORD_WIDTH(1),
                    .C_COMBO_BYPASS(1),
                    .C_RAM_DEPTH(C_NUM_QP / 32)
                ) inst_wqe_cmpl_intr_sts (
                    .core_clk(core_clk),
                    .core_rst(!core_rstn),
                    .wr_en   (init_start || wqe_cmpl_intr_sts_valid[k] || wqe_cmpl_intr_sts_valid_int[k]),
                    .wr_addr (init_start ? init_count[C_QP_INDX_WIDTH-1 : 5] : wqe_cmpl_intr_sts_id[C_QP_INDX_WIDTH-1 : 5]),
                    .wr_data (init_start ? 1'b0 : (wqe_cmpl_intr_sts_valid[k] ? 1'b0 : 1'b1)),
                    .rd_en   (1'b1),
                    .rd_addr ((wqe_cmpl_intr_sts_id[C_QP_INDX_WIDTH-1 : 5])),
                    .rd_data (wqe_cmpl_intr_sts_rd_data[k])
                );

                always_ff @(posedge core_clk) begin
                    if (!core_rstn) begin
                        num_ones_in_cmpl_intr_lut[k] <= 'd0;
                    end else begin
                        //case({wqe_cmpl_intr_sts_valid[k],wqe_cmpl_intr_sts_valid_int[k],wqe_cmpl_intr_sts_rd_data[k]})
                        case ({
                            (wqe_cmpl_intr_sts_valid[k] || wqe_cmpl_intr_sts_valid_int[k]), wqe_cmpl_intr_sts_data[k], wqe_cmpl_intr_sts_rd_data[k]
                        })
                            3'b101:  num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k] - 1;
                            3'b110:  num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k] + 1;
                            default: num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k];
                        endcase
                    end
                end
            end  //loop

            always_comb begin
                Any_ones_in_cmpl_intr_lut = 0;
                for (integer i = 0; i <= 31; i = i + 1) begin
                    Any_ones_in_cmpl_intr_lut = Any_ones_in_cmpl_intr_lut || (|num_ones_in_cmpl_intr_lut[i]);
                end
            end
        end // wqe_cmpl_intr_sts

        else begin : wqe_cmpl_intr_sts_less_than_32
            //assuming 32 bit set at a time for i_clr_wqe_cmpl_sts
            //Should be atleast 1 clock gap for consecutive assertion of both |i_clr_wqe_cmpl_sts_set_valid and intr_wqe_cmpl_sts
            always_ff @(posedge core_clk) begin : wqe_cmpl_intr_sts_proc
                if (~core_rstn) begin
                    wqe_cmpl_intr_sts_valid         <= {32{1'b0}};
                    wqe_cmpl_intr_sts_data          <= {32{1'b0}};
                    wqe_cmpl_intr_sts_valid_int     <= {32{1'b0}};
                    wqe_cmpl_intr_sts_id            <= {C_QP_INDX_WIDTH{1'b0}};
                    wqe_cmpl_intr_sts_valid_pending <= {32{1'b0}};
                    wqe_cmpl_intr_sts_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    if (|i_clr_wqe_cmpl_sts_set_valid) begin
                        wqe_cmpl_intr_sts_valid     <= i_clr_wqe_cmpl_sts_set_valid;
                        wqe_cmpl_intr_sts_id        <= i_clr_wqe_cmpl_sts_set_id;
                        wqe_cmpl_intr_sts_valid_int <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data      <= {32{1'b0}};
                    end else if (intr_wqe_cmpl_sts) begin
                        wqe_cmpl_intr_sts_valid_int[arbitrated_idx_for_intr] <= 1'b1;
                        wqe_cmpl_intr_sts_id                                 <= arbitrated_idx_for_intr;
                        wqe_cmpl_intr_sts_valid                              <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data                               <= {32{1'b1}};
                    end else if (|wqe_cmpl_intr_sts_valid_pending) begin
                        wqe_cmpl_intr_sts_valid_int <= wqe_cmpl_intr_sts_valid_pending;
                        wqe_cmpl_intr_sts_id        <= wqe_cmpl_intr_sts_id_pending;
                        wqe_cmpl_intr_sts_valid     <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data      <= {32{1'b1}};
                    end else begin
                        wqe_cmpl_intr_sts_valid     <= {32{1'b0}};
                        wqe_cmpl_intr_sts_valid_int <= {32{1'b0}};
                    end

                    if (intr_wqe_cmpl_sts && !(i_clr_wqe_cmpl_sts_set_valid[arbitrated_idx_for_intr] == 1'b1 && arbitrated_idx_for_intr == wqe_cmpl_intr_sts_id)) begin
                        wqe_cmpl_intr_sts_valid_pending[arbitrated_idx_for_intr] <= 1'b1;
                        wqe_cmpl_intr_sts_id_pending                             <= arbitrated_idx_for_intr;
                    end else begin
                        wqe_cmpl_intr_sts_valid_pending <= {32{1'b0}};
                    end
                end
            end

            reg [31 : 0] num_ones_in_cmpl_intr_lut;
            reg [  31:0] wqe_cmpl_intr_sts_rd_data_reg;



            for (k = 0; k <= 31; k = k + 1) begin
                always_ff @(posedge core_clk) begin
                    if (!core_rstn) begin
                        wqe_cmpl_intr_sts_rd_data_reg[k] <= 1'b0;
                    end else begin
                        if (wqe_cmpl_intr_sts_valid_int[k] || wqe_cmpl_intr_sts_valid[k]) begin
                            wqe_cmpl_intr_sts_rd_data_reg[k] <= wqe_cmpl_intr_sts_data[k];
                        end
                    end
                end

                always_ff @(posedge core_clk) begin
                    if (!core_rstn) begin
                        num_ones_in_cmpl_intr_lut[k] <= 1'b0;
                    end else begin
                        case ({
                            (wqe_cmpl_intr_sts_valid[k] || wqe_cmpl_intr_sts_valid_int[k]), wqe_cmpl_intr_sts_data[k], wqe_cmpl_intr_sts_rd_data_reg[k]
                        })
                            3'b101:  num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k] - 1;
                            3'b110:  num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k] + 1;
                            default: num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k];
                        endcase
                    end
                end
            end

            always_comb begin
                Any_ones_in_cmpl_intr_lut = |num_ones_in_cmpl_intr_lut;
            end

        end  // wqe_cmpl_intr_sts_less_than_32
    endgenerate  // C_NUM_QP <= 32



    `MSFF_R(wqe_cmpl_intr_sts_or_Reduce, Any_ones_in_cmpl_intr_lut, core_clk, !core_rstn)

    assign o_intr_wqe_cmpl_sts_wr_en           = wqe_cmpl_intr_sts_valid | wqe_cmpl_intr_sts_valid_int;
    assign o_intr_wqe_cmpl_sts_wr_addr         = wqe_cmpl_intr_sts_id;
    assign o_intr_wqe_cmpl_sts_wr_data         = wqe_cmpl_intr_sts_data;
    assign o_intr_wqe_cmpl_sts_wr_en_pending   = wqe_cmpl_intr_sts_valid_pending;
    assign o_intr_wqe_cmpl_sts_wr_addr_pending = wqe_cmpl_intr_sts_id_pending;

    assign o_intr_wqe_cmpl                     = (wqe_cmpl_intr_sts_or_Reduce != 1'b0) && i_intr_en_wqe_cmpl;
    assign o_intr_wqe_cmpl_sts_or_reduce       = wqe_cmpl_intr_sts_or_Reduce;

    //----------------------------------------------------------------------------------


    //----------------------------------------------------------------------------------
    //Arbiter to arbitrate between the QPs that need processing
    // No ack is rcvd for QP1, hence extracting outstanding info from transmitted requests
    always_ff @(posedge core_clk) begin
        if (~core_rstn) begin
            QP1_acknack_valid <= 1'b0;
            pushback_valid    <= 1'b0;
            arbiter_valid_in  <= 1'b0;
        end else begin
            if (wqe_push_for_non_nacked_qp && i_wqe_qpid == {{C_QP_INDX_WIDTH - 1{1'b0}}, 1'b1}) begin
                QP1_acknack_valid <= 1'b1;
            end else if (~i_acknack_valid) begin
                QP1_acknack_valid <= 1'b0;
            end

            arbiter_valid_in <= i_acknack_valid || QP1_acknack_valid || pushback_timeout_fatal_valid;
            if (i_acknack_valid) begin
                arbiter_Qpid_in <= i_qp_index_acknack;
            end else if (QP1_acknack_valid) begin
                arbiter_Qpid_in <= {{C_QP_INDX_WIDTH - 1{1'b0}}, 1'b1};
            end else if (pushback_timeout_fatal_valid) begin
                arbiter_Qpid_in <= pushback_timeout_fatal_id;
            end

            //FSM may take mutiple iterations to drain a fatal QP/Nacked psn processing
            //if (arbitrate && (fsm_pushback_valid || (!osq_empty_rd_data_2 && i_qp_fatal_rd_data_1))) begin
            if (arbitrate && fsm_pushback_valid) begin
                pushback_valid <= 1'b1;
                pushback_Qpid  <= osq_arbitrated_idx;
            end else if (~fifo_busy) begin
                pushback_valid <= 1'b0;
            end
        end
    end

    //Skip the Arbitrated fifo if there are no OS for that QP to be processed (otherwise FSM will be busy for nothing)
    always_ff @(posedge core_clk) begin
        no_OS_pending_r        <= no_OS_pending;
        arbitration_done_int_r <= arbitration_done_int;
        if (arbitrate) begin
            arbitration_done <= 1'b0;
        end else begin
            arbitration_done <= arbitration_done_int && ~no_OS_pending && ~no_OS_pending_r;
        end

    end
    assign no_OS_pending = osq_empty_rd_data_2 && arbitration_done_int && ~arbitration_done_int_r;

    FIFO_QPID_Arbiter #(
        .C_MAX_QP(C_NUM_QP),
        .C_MAX_QID_WIDTH(C_QP_INDX_WIDTH)
    ) inst_ack_nack_resp_Arbiter (
        .core_clk        (core_clk),
        .core_rst        (~core_rstn),                 //Arbiter reset is active high
        .init_start      (init_start),                 //lut ram seq_reset interface 
        .init_count      (init_count),                 //lut ram seq_reset interface 
        .valid_in        (arbiter_valid_in),
        .Qpid_in         (arbiter_Qpid_in),
        .fetch_next_id   (arbitrate | no_OS_pending),
        .pushback_Qpid   (pushback_Qpid),
        .pushback_valid  (pushback_valid),
        .arbitration_done(arbitration_done_int),
        .arbitrated_Qpid (osq_arbitrated_idx),
        .fifo_busy       (fifo_busy)
    );


    /******************* FIFO to store QPIDs for qp_under_recovery/timedout QPs *************************************************/
    wire                         push_is_done_A0;
    wire                         push_is_done_A1;
    wire                         push_is_done_B0;
    wire                         push_is_done_B1;
    wire                         push_is_avail_A;
    wire                         push_is_avail_B;
    reg                          i_qp_enable_valid_r;
    reg                          osq_empty_reset_valid_r;
    reg  [C_QP_INDX_WIDTH-1 : 0] osq_empty_reset_id_r;

    xpm_memory_tdpram #(
        .ADDR_WIDTH_A           (C_QP_INDX_WIDTH),
        .ADDR_WIDTH_B           (C_QP_INDX_WIDTH),
        .AUTO_SLEEP_TIME        (0),
        .BYTE_WRITE_WIDTH_A     (C_QP_INDX_WIDTH),
        .BYTE_WRITE_WIDTH_B     (C_QP_INDX_WIDTH),
        .CASCADE_HEIGHT         (0),
        .CLOCKING_MODE          ("common_clock"),
        .ECC_MODE               ("no_ecc"),
        .MEMORY_INIT_FILE       ("none"),
        .MEMORY_INIT_PARAM      ("0"),
        .MEMORY_OPTIMIZATION    ("true"),
        .MEMORY_PRIMITIVE       ("auto"),
        .MEMORY_SIZE            (C_QP_INDX_WIDTH * C_NUM_QP),
        .MESSAGE_CONTROL        (0),
        .READ_DATA_WIDTH_A      (C_QP_INDX_WIDTH),
        .READ_DATA_WIDTH_B      (C_QP_INDX_WIDTH),
        .READ_LATENCY_A         (1),
        .READ_LATENCY_B         (1),
        .READ_RESET_VALUE_A     ("0"),
        .READ_RESET_VALUE_B     ("0"),
        .RST_MODE_A             ("SYNC"),
        .RST_MODE_B             ("SYNC"),
        .SIM_ASSERT_CHK         (0),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT           (0),
        .WAKEUP_TIME            ("disable_sleep"),
        .WRITE_DATA_WIDTH_A     (C_QP_INDX_WIDTH),
        .WRITE_DATA_WIDTH_B     (C_QP_INDX_WIDTH),
        .WRITE_MODE_A           ("read_first"),
        .WRITE_MODE_B           ("read_first")
    ) inst_fatal_timedout_qpid (
        .clka  (core_clk),
        .rsta  (!core_rstn),
        .regcea(1'b1),
        .ena   (push_wr_en_A && push_is_avail_A),
        .wea   (push_wr_en_A && push_is_avail_A),
        .addra (Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH-1 : 0]),
        .dina  (push_wr_id_A),
        .douta (),
        .clkb  (core_clk),
        .rstb  (!core_rstn),
        .regceb(1'b1),
        .enb   ((push_wr_en_B && push_is_avail_B) || pop_wr_en_B),
        .web   ((push_wr_en_B && push_is_avail_B) && !pop_wr_en_B),
        .addrb (buffer_addr_B),
        .dinb  (push_wr_id_B),
        .doutb (fatal_timeout_qpid)
    );

    assign fifo_empty = (Qpid_buffer_wr_ptr == Qpid_buffer_rd_ptr) ? 1'b1 : 1'b0;
    assign fifo_full  = (Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH] != Qpid_buffer_rd_ptr[C_QP_INDX_WIDTH] && Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH-1 : 0] == Qpid_buffer_rd_ptr[C_QP_INDX_WIDTH-1 : 0]) ? 1'b1 : 1'b0;

    always_ff @(posedge core_clk) begin
        pop_wr_en_B_r           <= pop_wr_en_B;
        push_wr_en_A_r          <= push_wr_en_A;
        push_wr_en_B_r          <= push_wr_en_B;
        osq_empty_reset_valid_r <= osq_empty_reset_valid;
        osq_empty_reset_id_r    <= osq_empty_reset_id;
        i_qp_enable_valid_r     <= i_qp_enable_valid;
    end

    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            push_wr_en_A                 <= 1'b0;
            push_wr_id_A                 <= 'd0;
            push_wr_en_B                 <= 1'b0;
            push_wr_id_B                 <= 'd0;
            pushback_timeout_fatal_valid <= 1'b0;
            buffer_addr_B                <= 'd0;
            Qpid_buffer_wr_ptr           <= 'd0;
            Qpid_buffer_rd_ptr           <= 'd0;
            pushback_timeout_fatal_id    <= 'd0;
            push_pending                 <= 1'b0;
            push_id_pending              <= 'd0;
        end else begin
            if (osq_timed_out_set_valid) begin
                push_wr_en_A <= 1'b1;
                push_wr_id_A <= osq_timed_out_set_id;
            end else if (osq_empty_reset_valid_r && qp_under_fatal_or_recovery_rd_data) begin
                push_wr_en_A <= 1'b1;
                push_wr_id_A <= osq_empty_reset_id_r;
            end else if (push_pending) begin
                push_wr_en_A <= 1'b1;
                push_wr_id_A <= push_id_pending;
            end else begin
                push_wr_en_A <= 1'b0;
            end

            if (osq_timed_out_set_valid && (osq_empty_reset_valid_r && qp_under_fatal_or_recovery_rd_data) && osq_timed_out_set_id != osq_empty_reset_id_r) begin
                push_pending    <= 1'b1;
                push_id_pending <= osq_empty_reset_id_r;
            end else begin
                push_pending <= 1'b0;
            end

            if ((i_qp_fatal_valid && i_qp_fatal_data && !osq_empty_rd_data_3) && (osq_timed_out_set_valid || (osq_empty_reset_valid_r && qp_under_fatal_or_recovery_rd_data) || push_pending)) begin
                push_wr_en_B  <= 1'b1;
                push_wr_id_B  <= i_qp_fatal_id;
                buffer_addr_B <= Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH-1 : 0] + 1;
            end else if (i_qp_fatal_valid && i_qp_fatal_data && !osq_empty_rd_data_3) begin
                push_wr_en_B  <= 1'b1;
                push_wr_id_B  <= i_qp_fatal_id;
                buffer_addr_B <= Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH-1 : 0];
            end else begin
                push_wr_en_B  <= 1'b0;
                buffer_addr_B <= Qpid_buffer_rd_ptr[C_QP_INDX_WIDTH-1 : 0];
            end

            if (i_qp_fatal_valid && i_qp_fatal_data && !osq_empty_rd_data_3) begin
                pop_wr_en_B <= 1'b0;
            end else if (!fifo_empty && !pop_wr_en_B && !pop_wr_en_B_r && !pushback_timeout_fatal_valid) begin
                pop_wr_en_B <= 1'b1;
            end else begin
                pop_wr_en_B <= 1'b0;
            end

            if (push_wr_en_A && push_wr_en_B) begin
                Qpid_buffer_wr_ptr <= Qpid_buffer_wr_ptr + 2;
            end else if (push_wr_en_A || push_wr_en_B) begin
                Qpid_buffer_wr_ptr <= Qpid_buffer_wr_ptr + 1;
            end

            if (pop_wr_en_B) begin
                Qpid_buffer_rd_ptr <= Qpid_buffer_rd_ptr + 1;
            end

            if (pop_wr_en_B_r) begin
                pushback_timeout_fatal_valid <= 1'b1;
                pushback_timeout_fatal_id    <= fatal_timeout_qpid;
            end else if (!i_acknack_valid && !QP1_acknack_valid) begin
                pushback_timeout_fatal_valid <= 1'b0;
            end
        end
    end

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_duplicate_push_protection_0 (
        .core_clk   (core_clk),
        .core_rst   (!core_rstn),
        .wr_en      (init_start || (i_qp_fatal_valid && i_qp_fatal_data)),
        .wr_addr    (init_start ? init_count : i_qp_fatal_id),
        .wr_data    (init_start ? 1'b0 : 1'b1),
        .wr_en_aux  (i_qp_enable_valid && !i_qp_enable_valid_r && !i_qp_enable_data),
        .wr_addr_aux(i_qp_enable_id),
        .wr_data_aux(1'b0),
        .rd_en      (1'b1),
        .rd_addr    (osq_empty_reset_id_r),
        .rd_data    (qp_under_fatal_or_recovery_rd_data)
    );

    always_ff @(posedge core_clk) begin
        i_qp_under_recovery_wr_en_r <= i_qp_under_recovery_wr_en;
    end

    assign push_is_avail_A  = 1'b1;
    assign push_is_avail_B  = 1'b1;

    /*
    IN case if overflow protection for FIFO inst_fatal_timedout_qpid is needed; enable this logic (at the cost of nearly 1.8K LUTS for 2K QP)

      dual_wr_sdp_lut_ram_wrapper #(
    	.C_RAM_WORD_WIDTH(1),
    	.C_RAM_DEPTH(C_NUM_QP),
    	.C_COMBO_BYPASS(1)
    )	inst_duplicate_push_protection_1(
    	.core_clk		(core_clk),
    	.core_rst       (!core_rstn),
    	.wr_en          (init_start || push_wr_en_A),
    	.wr_addr        (init_start ? init_count : push_wr_id_A),
    	.wr_data        (init_start ? 1'b0 : 1'b1),
    	.wr_en_aux      (pop_wr_en_B_r),
    	.wr_addr_aux    (fatal_timeout_qpid),
    	.wr_data_aux    (1'b0),
    	.rd_en          (1'b1),
    	.rd_addr        (push_wr_id_A),
    	.rd_data		(push_is_done_A0)
    );

      dual_wr_sdp_lut_ram_wrapper #(
    	.C_RAM_WORD_WIDTH(1),
    	.C_RAM_DEPTH(C_NUM_QP),
    	.C_COMBO_BYPASS(1)
    )	inst_duplicate_push_protection_2(
    	.core_clk		(core_clk),
    	.core_rst       (!core_rstn),
    	.wr_en          (init_start || push_wr_en_B),
    	.wr_addr        (init_start ? init_count : push_wr_id_B),
    	.wr_data        (init_start ? 1'b0 : 1'b1),
    	.wr_en_aux      (pop_wr_en_B_r),
    	.wr_addr_aux    (fatal_timeout_qpid),
    	.wr_data_aux    (1'b0),
    	.rd_en          (1'b1),
    	.rd_addr        (push_wr_id_A),
    	.rd_data		(push_is_done_A1)
    );

    assign push_is_avail_A = !(push_is_done_A0 && push_is_done_A1);

      dual_wr_sdp_lut_ram_wrapper #(
    	.C_RAM_WORD_WIDTH(1),
    	.C_RAM_DEPTH(C_NUM_QP),
    	.C_COMBO_BYPASS(1)
    )	inst_duplicate_push_protection_3(
    	.core_clk		(core_clk),
    	.core_rst       (!core_rstn),
    	.wr_en          (init_start || push_wr_en_A),
    	.wr_addr        (init_start ? init_count : push_wr_id_A),
    	.wr_data        (init_start ? 1'b0 : 1'b1),
    	.wr_en_aux      (pop_wr_en_B_r),
    	.wr_addr_aux    (fatal_timeout_qpid),
    	.wr_data_aux    (1'b0),
    	.rd_en          (1'b1),
    	.rd_addr        (push_wr_id_B),
    	.rd_data		(push_is_done_B0)
    );

      dual_wr_sdp_lut_ram_wrapper #(
    	.C_RAM_WORD_WIDTH(1),
    	.C_RAM_DEPTH(C_NUM_QP),
    	.C_COMBO_BYPASS(1)
    )	inst_qp_under_Recovery_or_fatal(
    	.core_clk		(core_clk),
    	.core_rst       (!core_rstn),
    	.wr_en          (init_start || push_wr_en_B),
    	.wr_addr        (init_start ? init_count : push_wr_id_B),
    	.wr_data        (init_start ? 1'b0 : 1'b1),
    	.wr_en_aux      (pop_wr_en_B_r),
    	.wr_addr_aux    (fatal_timeout_qpid),
    	.wr_data_aux    (1'b0),
    	.rd_en          (1'b1),
    	.rd_addr        (push_wr_id_B),
    	.rd_data		(push_is_done_B1)
    );

    assign push_is_avail_B = !(push_is_done_B0 && push_is_done_B1); */

    assign o_qp_conf_qp_idx = osq_sampled_arbitrated_idx;
    //assign o_qp_sq_psn_resp_idx = osq_sampled_arbitrated_idx;
    //assign o_qp_sq_psn_resp_req = 1'b1;


    //----------------------------------------------------------------------------------

    // 响应处理程序 FSM 实例 -----------------------------------------
    //
    resp_handler_fsm #(
        .NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN  (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN),
        .NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING(NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING),
        .NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE    (NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE),
        .C_NUM_QP                                (C_NUM_QP),
        .C_M_AXI_ADDR_WIDTH                      (C_M_AXI_ADDR_WIDTH),
        .C_QP_INDX_WIDTH                         (C_QP_INDX_WIDTH),
        .C_EN_NVMOF_HW_HNDSHK                    (C_EN_NVMOF_HW_HNDSHK),
        .IP2BUS_LEN_WIDTH                        (IP2BUS_LEN_WIDTH),
        .C_OS_Q_DEPTH                            (C_OS_Q_DEPTH),
        .C_OSQ_PSN_WIDTH                         (C_OSQ_PSN_WIDTH),
        //.C_EN_WR_RETRY_DATA_BUF(C_EN_WR_RETRY_DATA_BUF),
        .C_OS_Q_INDX_WIDTH                       (C_OS_Q_INDX_WIDTH)
    ) inst_resp_handler_fsm (
        .core_clk                    (core_clk),                    // Module Clock  
        .core_rstn                   (core_rstn),                   // Active low core reset
        .i_acks_to_process           (ack_nack_to_process),
        .o_arbitrate                 (arbitrate),
        .fsm_pushback_valid          (fsm_pushback_valid),
        .i_arbitration_done          (arbitration_done),
        .i_arbitrated_indx           (osq_arbitrated_idx),
        .o_sampled_arbitrated_indx   (osq_sampled_arbitrated_idx),
        .o_arbitrated_qp_for_intr    (arbitrated_idx_for_intr),
        .o_resp_hndler_sts           (resp_hndler_sts),
        .o_stat_retry_count          (o_stat_retry_count),
        .o_phase_to_update_post_retry(phase_to_update_post_retry),
        .i_cfg_retry_cnt_fatal_dis   (i_cfg_retry_cnt_fatal_dis),
        .i_bypass_en                 (i_bypass_en),
        .i_qp_fatal_valid            (i_qp_fatal_valid),
        .i_qp_fatal_id               (i_qp_fatal_id),
        .i_qp_fatal_data             (i_qp_fatal_data),

        .i_dma_in_idle             (i_dma_in_idle),
        .i_en_osq_flush_on_qp_fatal(i_en_osq_flush_on_qp_fatal),

        .i_first_outgoing_pkt_valid        (first_outgoing_pkt_ff_valid),
        .i_first_outgoing_pkt_id           (first_outgoing_pkt_ff_id),
        .i_first_outgoing_pkt_data         (first_outgoing_pkt_ff_data),
        .i_first_outgoing_pkt_valid_pending(first_outgoing_pkt_ff_valid_pending),
        .i_first_outgoing_pkt_id_pending   (first_outgoing_pkt_ff_id_pending),
        .i_first_outgoing_pkt_data_pending (1'b0),
        .i_wqe_qpid                        (i_wqe_qpid),
        .i_wqe_send_start_psn              (i_wqe_send_start_psn[C_OSQ_PSN_WIDTH-1:0]),
        .i_wqe_push_req                    (i_wqe_push_req),

        .o_pop_osq               (pop_osq_from_fsm),
        .o_rd_osq                (rd_osq),
        .i_rd_osq_vld            (osq_rd_addr_rd_valid),
        .o_flush_mpsn            (flush_mpsn_from_fsm),
        .i_mpsnbuf_flush_done    (flush_mpsn_done_to_fsm),
        .i_osq_empty             (osq_empty_rd_data_0),
        //.i_mpsnbuf_empty            (mpsnbuf_empty_ff[osq_sampled_arbitrated_idx]),
        .i_phase_bit_acknack_psn (phase_bit_acknack_psn_ff_rd_data_0),
        .i_osq_entry_msn         (osq_msn),
        .i_osq_entry_end_psn     (osq_end_psn),
        .i_osq_entry_start_psn   (osq_start_psn),
        .i_osq_acknacked_psn     (acknacked_psn_for_arbitrated_qp),
        .i_osq_acknacked_msn     (acknacked_msn_for_arbitrated_qp),
        .i_osq_last_acknacked_psn(last_acknacked_psn_for_arbitrated_qp),

        .i_osq_rnr_nacked              ((FLOPS_TO_MEM == 1) ? osq_rnr_nacked_to_fsm : osq_rnr_nacked_ff_rd_data_3),
        .o_clr_timeout_n_nack_set_valid(clr_timeout_n_nack_set_valid),
        .o_clr_timeout_n_nack_set_id   (clr_timeout_n_nack_set_id),
        .o_clr_pending_ack_set_valid   (clr_pending_ack_set_valid),
        .o_clr_pending_ack_set_id      (clr_pending_ack_set_id),
        .i_osq_nacked                  ((FLOPS_TO_MEM == 1) ? osq_nacked_to_fsm : osq_nacked_ff_rd_data_2),
        .i_osq_wqe_nack_resp           (i_osq_wqe_nack_resp_rd_data_1),
        .i_osq_opcode                  (osq_opcode),                                                                 // BTH.Opcode of outbound request
        .i_osq_wrid                    (osq_wrid),
        .o_psn_to_retry                (psn_to_retry),
        .o_msn_to_retry                (msn_to_retry),
        .o_freeup_data_buf             (o_freeup_data_buf),
        .o_freeup_data_bufid           (o_freeup_data_bufid),
        .i_osq_wqe_retried             (osq_wqe_retried),
        .i_osq_wqe_fatal_push          (osq_wqe_fatal_push),
        .i_osq_wqe_data_bufid          (osq_data_bufid),
        .acknacked_psn_rd_req          (acknacked_psn_rd_req_from_fsm),
        .acknacked_psn_rd_req_done     (acknacked_psn_rd_req_done_to_fsm),

        .o_qp_cq_ba_req           (o_qp_cq_ba_req),
        .i_qp_cq_ba               (i_qp_cq_ba),                 // CQ base address
        .i_qp_cq_ba_valid         (i_qp_cq_ba_valid),
        .o_qp_cq_depth_req        (o_qp_cq_depth_req),
        .i_qp_cq_depth_valid      (i_qp_cq_depth_valid),        // CQ depth
        .i_qp_cq_depth            (i_qp_cq_depth),
        .o_qp_sq_cmpldb_addr_req  (o_qp_sq_cmpldb_addr_req),
        .i_qp_sq_cmpldb_addr_valid(i_qp_sq_cmpldb_addr_valid),  // SQ Completion DB address (points to RDMAif register)
        .i_qp_sq_cmpldb_addr      (i_qp_sq_cmpldb_addr),

        .o_qp_sq_ba_req  (o_qp_sq_ba_req),
        .i_qp_sq_ba_valid(i_qp_sq_ba_valid),
        .i_qp_sq_ba      (i_qp_sq_ba),

        .i_data_buf_ba(i_data_buf_ba),
        .i_data_buf_sz(i_data_buf_sz),

        .o_qp_conf_req  (o_qp_conf_req),
        .i_qp_conf_valid(i_qp_conf_valid),
        .i_qp_conf      (i_qp_conf),

        .o_qp_cq_hdptr_req  (o_qp_cq_hdptr_req),
        .i_qp_cq_hdptr_valid(i_qp_cq_hdptr_valid),  // CQ head pointer
        .i_qp_cq_hdptr      (i_qp_cq_hdptr),
        .o_qp_cq_hdptr      (o_qp_cq_hdptr),
        .o_qp_cq_hdptr_wrn  (o_qp_cq_hdptr_wrn),
        .o_qp_cqe_proc_qpid (o_qp_cqe_proc_qpid),

        .i_rd_rsp_wr_done   (i_rd_rsp_wr_done),
        .i_rd_rsp_qpid      (i_rd_rsp_qpid),
        .i_rd_rsp_dest_addr (i_rd_rsp_dest_addr),
        .i_intr_clr_wqe_cmpl(i_intr_clr_wqe_cmpl),
        .o_intr_wqe_cmpl_sts(intr_wqe_cmpl_sts),

        // Interface to QP manager for retrnsmission 
        .o_retransmit_qpid    (retransmit_qpid),       // QP id for this retransmission is required
        .o_retransmit_reqd    (retransmit_reqd),       // Retransmission is required
        .i_retransmit_accepted(i_retransmit_accepted), // Ack from QP manager that the retransmission is initiated

        // Hardware handshake ports for NVMof (only enabled if C_EN_NVMOF_HW_HNDSHK is set to 1)
        // Direct handshake with RDMAif module
        .o_send_cq_db_cnt_valid(send_cq_db_cnt_valid),
        .o_send_cq_db_addr     (send_cq_db_addr),
        .o_send_cq_db_cnt      (send_cq_db_cnt),
        .i_send_cq_db_rdy      (send_cq_db_rdy),

        .i_qp_curr_retry_cnt       (i_qp_curr_retry_cnt),
        .o_qp_curr_retry_cnt_rd_req(o_qp_curr_retry_cnt_rd_req),
        .o_qp_curr_retry_cnt_qpid  (o_qp_curr_retry_cnt_qpid),
        .i_qp_curr_retry_cnt_rd_rdy(i_qp_curr_retry_cnt_rd_rdy),

        .o_qp_curr_retry_cnt_wr_req(o_qp_curr_retry_cnt_wr_req),
        .i_qp_curr_retry_cnt_wr_rdy(i_qp_curr_retry_cnt_wr_rdy),
        //.o_qp_curr_retry_cnt_qpid (o_qp_curr_retry_cnt_qpid   ),
        .o_qp_curr_retry_cnt_wr    (o_qp_curr_retry_cnt),

        .o_qp_cfg_retry_cnt_rd_req(o_qp_cfg_retry_cnt_rd_req),
        .i_qp_cfg_retry_cnt_rd_rdy(i_qp_cfg_retry_cnt_rd_rdy),
        .i_qp_cfg_retry_cnt       (i_qp_cfg_retry_cnt),

        .o_qp_retry_fatal_req(o_qp_retry_fatal_req),
        .o_qp_retry_fatal_qpid(o_qp_retry_fatal_qpid),
        .i_qp_retry_fatal_rdy(i_qp_retry_fatal_rdy),
        .i_reload_retry_cnt_en({reload_retry_cnt_valid_pending_1, reload_retry_cnt_id_pending_1, reload_retry_cnt_data_pending_1, reload_retry_cnt_valid_pending_0, reload_retry_cnt_id_pending_0, reload_retry_cnt_valid, reload_retry_cnt_id, reload_retry_cnt_data}),
        .i_bus2ip_data_rdy(bus2ip_data_rdy),
        .i_axi_master_done(axi_master_done),
        .o_ip2bus_wen(ip2bus_wen),  // Write en to AXI master
        .o_ip2bus_len(ip2bus_len),
        .o_ip2bus_wraddr(ip2bus_wraddr),  // Write address
        .o_ip2bus_data(ip2bus_data),
        .o_retransmit_wqe_bram_din(o_retransmit_wqe_bram_din),
        .fsm_arbitrated_indx(fsm_arbitrated_indx),
        .fsm_i_arbitrated_indx(fsm_i_arbitrated_indx),
        .osq_timed_out_sticky_fsm_arbitrated_indx(osq_timed_out_sticky_fsm_arbitrated_indx),
        .init_start(init_start),
        .init_count(init_count),
        .osq_nacked_ff({osq_nacked_ff_valid_pending, osq_nacked_ff_id_pending, osq_nacked_ff_valid, osq_nacked_ff_id, osq_nacked_ff_data}),
        .osq_rnr_nacked_ff({osq_rnr_nacked_ff_valid_pending, osq_rnr_nacked_ff_id_pending, osq_rnr_nacked_ff_valid, osq_rnr_nacked_ff_id, osq_rnr_nacked_ff_data}),
        .i_qp_under_recovery_wr_en(i_qp_under_recovery_wr_en && !i_qp_under_recovery_wr_en_r),
        .i_qp_under_recovery_wr_idx(i_qp_under_recovery_wr_idx),
        .i_qp_under_recovery_wr_data(i_qp_under_recovery_wr_data)
    );

    //------------------------------------------------------------


    assign o_retransmit_reqd             = retransmit_reqd;
    assign o_osq_full_set_valid          = osq_full_set_valid;
    assign o_osq_full_set_id             = osq_full_set_id;
    assign o_osq_full_reset_valid        = osq_full_reset_valid;
    assign o_osq_full_reset_id           = osq_full_reset_id;
    assign o_osq_almost_full_set_valid   = osq_almost_full_set_valid;
    assign o_osq_almost_full_set_id      = osq_almost_full_set_id;
    assign o_osq_almost_full_reset_valid = osq_almost_full_reset_valid;
    assign o_osq_almost_full_reset_id    = osq_almost_full_reset_id;
    assign o_retransmit_qpid             = retransmit_qpid;
    assign o_resp_hndler_sts             = resp_hndler_sts;

    // Need the fsm state 
    assign resp_h_cs                     = resp_hndler_sts[11+:6];

    //------------------------------------------------------------
    // 未完成 FIFO - MAX PSN。供 RX 数据包验证模块使用。
    // 当 WQE 操作码为 RDMA_READ（RDMA_READ）时，将向该 FIFO 推送一个条目。
    // 在出现聚合 acks 的情况下，它的作用是提供允许的最大 PSN 数量。换句话说，
    // 在 RDMA READ 操作中，任何合并的 ack 都不能跳过显式 ack。
    // 弹出操作由 RX 数据包处理程序 / 数据包验证模块 直接完成。
    // 所有 256 个 FIFO 的内存都被合并到一个 XPM 内存实例中，以便更好地优化内存利用率。

    `MSFF_R(mpsn_in_q, mpsn_in, core_clk, !core_rstn)
    `MSFF_R(mpsn_in_2q, mpsn_in_q, core_clk, !core_rstn)

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin  // current

            assign mpsn_in = {i_wqe_send_start_psn[C_OSQ_PSN_WIDTH-1:0], i_wqe_rdma_rd_res_len[23:0], i_wqe_rdma_rd_rcv_ba};

            // MAX PSN Buffer
            xpm_memory_sdpram #(

                // Common module parameters
                .MEMORY_SIZE      (MASXPSN_BUF_MEM_SZ),  //positive integer
                .MEMORY_PRIMITIVE ("ultra"),             //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),      //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),              //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                  //string;
                .USE_MEM_INIT     (1),                   //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),     //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                   //integer; 0,1

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(MAXPSN_BUF_ENTRY_SIZE),  //positive integer
                .BYTE_WRITE_WIDTH_A(MAXPSN_BUF_ENTRY_SIZE),  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (OS_Q_ADDR_WDITH),        //positive integer

                // Port B module parameters
                .READ_DATA_WIDTH_B (MAXPSN_BUF_ENTRY_SIZE),  //positive integer
                .ADDR_WIDTH_B      (OS_Q_ADDR_WDITH),        //positive integer
                .READ_RESET_VALUE_B("0"),                    //string
                .READ_LATENCY_B    (2),                      //non-negative integer
                .WRITE_MODE_B      ("read_first")            //string; "write_first", "read_first", "no_change" 

            ) inst_xpm_max_psn_q (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (1'b1),
                .wea           (mpsn_wr_addr_rd_valid),
                .addra         (mpsnbuf_wr_addr_new),
                .dina          (mpsn_in_2q),
                .injectsbiterra(1'b0),                   //do not change
                .injectdbiterra(1'b0),                   //do not change

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (~core_rstn),
                .enb     (1'b1),
                .regceb  (1'b1),
                .addrb   (mpsnbuf_rd_addr_new),
                .doutb   ({o_max_epsn, o_trnsfr_length, o_local_addr}),
                .sbiterrb(),                                             //do not change
                .dbiterrb()                                              //do not change

            );
        end else begin  // if (C_M_AXI_ADDR_WIDTH != 64)

            assign mpsn_in = {i_wqe_send_start_psn[C_OSQ_PSN_WIDTH-1:0], i_wqe_rdma_rd_res_len[23:0], {(64 - C_M_AXI_ADDR_WIDTH) {1'b0}}, i_wqe_rdma_rd_rcv_ba};
            // End of xpm_memory_sdpram instance declaration
            xpm_memory_sdpram #(

                // Common module parameters
                .MEMORY_SIZE      (MASXPSN_BUF_MEM_SZ),  //positive integer
                .MEMORY_PRIMITIVE ("ultra"),             //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),      //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),              //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                  //string;
                .USE_MEM_INIT     (1),                   //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),     //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                   //integer; 0,1


                // Port A module parameters
                .WRITE_DATA_WIDTH_A(MAXPSN_BUF_ENTRY_SIZE),  //positive integer
                .BYTE_WRITE_WIDTH_A(MAXPSN_BUF_ENTRY_SIZE),  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (OS_Q_ADDR_WDITH),        //positive integer
                // Port B module parameters
                .READ_DATA_WIDTH_B (MAXPSN_BUF_ENTRY_SIZE),  //positive integer
                .ADDR_WIDTH_B      (OS_Q_ADDR_WDITH),        //positive integer
                .READ_RESET_VALUE_B("0"),                    //string
                .READ_LATENCY_B    (2),                      //non-negative integer
                .WRITE_MODE_B      ("read_first")            //string; "write_first", "read_first", "no_change" 

            ) inst_xpm_max_psn_q (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (1'b1),
                .wea           (mpsn_wr_addr_rd_valid),
                .addra         (mpsnbuf_wr_addr_new),
                .dina          (mpsn_in_2q),
                .injectsbiterra(1'b0),                   //do not change
                .injectdbiterra(1'b0),                   //do not change

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (~core_rstn),
                .enb     (1'b1),
                .regceb  (1'b1),
                .addrb   (mpsnbuf_rd_addr_new),
                .doutb   ({o_max_epsn, o_trnsfr_length, o_local_addr}),
                .sbiterrb(),                                             //do not change
                .dbiterrb()                                              //do not change

            );
        end
    endgenerate
    `MSFF_R(mpsnbuf_req_ff, i_mpsn_buf_req, core_clk, ~core_rstn)
    `MSFF_R(mpsnbuf_req_2ff, mpsnbuf_req_ff, core_clk, ~core_rstn)
    // `MSFF_R(mpsnbuf_req_2ff, i_mpsn_buf_req, core_clk, ~core_rstn)
    `MSFF_R(mpsnbuf_req_3ff, mpsnbuf_req_2ff, core_clk, ~core_rstn)
    `MSFF_R(mpsnbuf_req_4ff, mpsnbuf_req_3ff, core_clk, ~core_rstn)
    `MSFF_R(mpsnbuf_req_5ff, mpsnbuf_req_4ff, core_clk, ~core_rstn)
    `MSFF_R(mpsnbuf_req_6ff, mpsnbuf_req_5ff, core_clk, ~core_rstn)
    `MSFF_R(mpsnbuf_req_7ff, mpsnbuf_req_6ff, core_clk, ~core_rstn)

    // `MSFF_R(mpsnbuf_pop_rd_conflict_ff, (i_mpsnbuf_pop & i_mpsn_buf_req), core_clk, ~core_rstn)
    // `MSFF_R(mpsnbuf_pop_rd_conflict_2ff, mpsnbuf_pop_rd_conflict_ff, core_clk, ~core_rstn)
    // `MSFF_R(mpsnbuf_pop_rd_conflict_3ff, mpsnbuf_pop_rd_conflict_2ff, core_clk, ~core_rstn)
    //assign o_mpsn_buf_valid = mpsnbuf_req_2ff;
    //assign o_mpsn_buf_valid = (~mpsnbuf_pop_rd_conflict_2ff & mpsnbuf_req_2ff) || (mpsnbuf_pop_rd_conflict_3ff & mpsnbuf_req_3ff) ;

    wire                         mpsnbuf_rd_addr_valid;
    wire                         mpsnbuf_rd_addr_en;

    reg  [C_QP_INDX_WIDTH - 1:0] i_qp_index_mpsnbuf_q;
    reg                          i_mpsn_buf_req_stky;
    reg                          i_mpsn_buf_req_stky_q;
    reg  [C_QP_INDX_WIDTH - 1:0] i_qp_index_mpsnbuf_stky;


    reg  [                  3:0] mpsnbuf_rd_addr_valid_dly;
    reg  [                  5:0] i_mpsnbuf_pop_dly;

    `MSFF_R(mpsnbuf_rd_addr_valid_dly[0], mpsnbuf_rd_addr_valid, core_clk, !core_rstn)
    `MSFF_R(i_mpsnbuf_pop_dly[0], i_mpsnbuf_pop, core_clk, !core_rstn)
    generate
        for (j = 1; j < 6; j = j + 1) begin : pop_dly
            `MSFF_R(i_mpsnbuf_pop_dly[j], i_mpsnbuf_pop_dly[j-1], core_clk, !core_rstn)
        end
        for (j = 1; j < 4; j = j + 1) begin : rd_addr_valid_dly
            `MSFF_R(mpsnbuf_rd_addr_valid_dly[j], mpsnbuf_rd_addr_valid_dly[j-1], core_clk, !core_rstn)
        end
    endgenerate
    `MSFF_R(i_qp_index_mpsnbuf_q, (i_mpsnbuf_pop ? i_qp_index_mpsnbuf : i_qp_index_mpsnbuf_q), core_clk, !core_rstn)
    `MSFF_R(i_mpsn_buf_req_stky_q, i_mpsn_buf_req_stky, core_clk, !core_rstn)
    always @(*) begin
        if (|i_mpsnbuf_pop_dly[1:0] & i_mpsn_buf_req & (i_qp_index_mpsnbuf_q == i_qp_index_mpsnbuf)) begin
            i_mpsn_buf_req_stky = 1'b1;
        end else if (i_mpsnbuf_pop_dly[3]) begin
            i_mpsn_buf_req_stky = 1'b0;
        end else begin
            i_mpsn_buf_req_stky = i_mpsn_buf_req_stky_q;
        end
    end

    always @(posedge core_clk) begin
        if (!core_rstn) begin
            i_qp_index_mpsnbuf_stky <= {C_QP_INDX_WIDTH{1'b0}};
        end else if (|i_mpsnbuf_pop_dly[1:0] & i_mpsn_buf_req & (i_qp_index_mpsnbuf_q == i_qp_index_mpsnbuf)) begin
            i_qp_index_mpsnbuf_stky <= i_qp_index_mpsnbuf;
        end
    end

    assign mpsnbuf_rd_addr_en = (i_mpsn_buf_req & !(|i_mpsnbuf_pop_dly[1:0] & (i_qp_index_mpsnbuf_q == i_qp_index_mpsnbuf))) | (i_mpsn_buf_req_stky & i_mpsnbuf_pop_dly[2]);
    assign mpsnbuf_rd_addr    = (i_mpsn_buf_req_stky & i_mpsnbuf_pop_dly[2]) ? i_qp_index_mpsnbuf_stky : i_qp_index_mpsnbuf;


    //assign o_mpsn_buf_valid = mpsnbuf_req_4ff;
    assign o_mpsn_buf_valid   = mpsnbuf_rd_addr_valid_dly[2];

    // 这是 max psn q 实现。
    // 实际内存是在 XPM 内存中实现的(上方 MAX PSN Buffer)。该模块管理内存的指针并生成
    q_mgr_init_top #(
        .C_NUM_QUEUES (C_NUM_QP),
        .C_Q_DEPTH    (C_OS_Q_DEPTH),
        .C_ENTRY_SIZE (1),
        .C_PTR_WIDTH  (C_OS_Q_INDX_WIDTH),
        .C_ADDR_WIDTH (OS_Q_ADDR_WDITH),
        .C_RDADDR_INTF(1)
    ) inst_mpsnbuf_manager (
        // Clock and Reset
        .core_clk             (core_clk),                           // Module Clock  
        .core_rst             (~core_rstn),                         // Active high core reset
        // Queue pointers
        .o_rd_addr            (mpsnbuf_rd_addr_new),
        //.i_rd_addr_rd_en                    (i_mpsn_buf_req),
        .i_rd_addr_rd_en      (mpsnbuf_rd_addr_en),
        //.i_rd_addr_qnum                     (i_qp_index_mpsnbuf),
        .i_rd_addr_qnum       (mpsnbuf_rd_addr),
        .i_rd_base_addr       (osq_base_addr_2d[mpsnbuf_rd_addr]),
        .o_rd_addr_rd_valid   (mpsnbuf_rd_addr_valid),
        .o_wr_addr            (mpsnbuf_wr_addr_new),                // Write address
        .i_wr_addr_rd_en      (valid_mpsnpush),
        .i_wr_addr_qnum       (i_wqe_qpid),
        .i_wr_base_addr       (osq_base_addr_2d[i_wqe_qpid]),
        .o_wr_addr_rd_valid   (mpsn_wr_addr_rd_valid),
        .o_q_empty_reset_valid(mpsnbuf_empty_reset_valid),
        .o_q_empty_reset_id   (mpsnbuf_empty_reset_id),
        .o_q_empty_set_valid  (mpsnbuf_empty_set_valid),
        .o_q_empty_set_id     (mpsnbuf_empty_set_id),

        .vld_free_entry_rden    (1'b0),
        .vld_free_entry_qnum    ({C_QP_INDX_WIDTH{1'b0}}),
        .vld_free_entry_data_vld(),
        .o_num_valid_entries    (num_valid_mpsn_entries_nc),   // Number of valid entries in the queue
        .o_num_free_entries     (num_free_mpsn_entries_nc),    // Number of free entries in the queue
        .i_q_flush              (flush_mpsn_from_fsm),
        .i_q_flush_qpid         (osq_sampled_arbitrated_idx),
        .o_q_flush_done         (flush_mpsn_done_to_fsm),

        .i_q_depth              (C_OS_Q_DEPTH),             // Configurable q depth (less than max as defined by param)
        .i_threshold_cntrl_valid(1'b0),
        .i_threshold_cntrl_id   ({C_QP_INDX_WIDTH{1'b0}}),
        .i_threshold_cntrl_data (1'b0),

        // Push Pop functions
        .i_q_push     (valid_mpsnpush),      // Push enable for a queue
        .i_q_push_qpid(i_wqe_qpid),
        .i_q_pop      (i_mpsnbuf_pop),       // Pop enable for a queue
        .i_q_pop_qpid (i_qp_index_mpsnbuf),
        .init_start   (init_start),
        .init_count   (init_count)
    );

    // When the buffer becomes empty and then an entry is pushed, there is
    // a read-write collision. As a result the data is x for one clock cycle.
    // To avoid this, the empty->not-empty is delayed by 2 clock cycles.


    //This workaround should be done by rx_pkt_handler_block (skipping here to reduce inter-block signaling)
    //delay o_mpsnbuf_empty interface; store to 3 LUT RAMs; while reading do-OR of all 3 outputs
    //assign o_mpsnbuf_empty = mpsnbuf_empty | mpsnbuf_empty_ff | mpsnbuf_empty_2ff;
    assign o_mpsnbuf_empty_set_valid       = mpsnbuf_empty_set_valid;
    assign o_mpsnbuf_empty_set_id          = mpsnbuf_empty_set_id;
    assign o_mpsnbuf_empty_reset_valid     = mpsnbuf_empty_reset_valid;
    assign o_mpsnbuf_empty_reset_id        = mpsnbuf_empty_reset_id;

    // Only READ WQEs go into the max psn buffer
    //assign valid_mpsnpush = (i_wqe_opcode == RDMA_READ) ? wqe_push_for_non_nacked_qp : 1'b0;
    // Register retiming for timing 
    // i_wqe_opcode == RDMA_READ condition check moved to hdr_gen block behind a flop named i_wqe_push_req_Read
    assign wqe_push_for_non_nacked_qp_Read = (osq_nacked_ff_rd_data_1 & i_osq_wqe_nack_resp_rd_data_0) ? 1'b0 : i_wqe_push_req_Read;
    assign valid_mpsnpush                  = wqe_push_for_non_nacked_qp_Read;

    //assign mpsnbuf_wr_addr_ored[0] = 32'b0;
    //assign mpsnbuf_rd_addr_ored[0] = 32'b0;

    // We can OR them because the nack pop QPID and the RX packet handler QPID
    // will never be the same. Once the QP is nacked, that QP cannot be acked
    // unless the packet is retried

    assign mpsn_pop_qpid                   = i_mpsnbuf_pop ? i_qp_index_acknack : osq_sampled_arbitrated_idx;

    //----------------------------------------------------------------------------

    resp_handler_timer #(
        .C_NUM_QP         (C_NUM_QP),
        .C_QP_INDX_WIDTH  (C_QP_INDX_WIDTH),
        .C_NUM_CYCLES_4US (C_NUM_CYCLES_4US),  // Number of clock cycles that make 4 us (for 125Mhz clk period it is 512)
        .C_OS_Q_INDX_WIDTH(C_OS_Q_INDX_WIDTH)
    ) inst_timer (
        .core_clk               (core_clk),                                                                                                                                                                                        // Module Clock  
        .core_rstn              (core_rstn),                                                                                                                                                                                       // Active low core reset
        .i_cfg_base_cnt         (i_cfg_base_cnt),
        .o_q_timed_out_valid    (osq_timed_out_set_valid),
        .o_q_timed_out_wr_addr  (osq_timed_out_set_id),
        //.i_timer_en           ((~osq_empty_ff & ~timer_disabled_per_qp & ~all_resp_received) | osq_rnr_nacked_ff),
        .osq_empty              ({osq_empty_ff_valid_pending, osq_empty_ff_id_pending, osq_empty_ff_valid, osq_empty_ff_id, osq_empty_ff_data}),
        .timer_disabled_per_qp  ({timer_disabled_per_qp_valid, timer_disabled_per_qp_id, timer_disabled_per_qp_data, timer_disabled_per_qp_valid_pending, timer_disabled_per_qp_id_pending, timer_disabled_per_qp_data_pending}),
        .all_resp_received      ({all_resp_received_reset_valid, all_resp_received_reset_id, all_resp_received_set_valid, all_resp_received_set_id}),
        .osq_rnr_nacked_ff      ({osq_rnr_nacked_ff_valid_pending, osq_rnr_nacked_ff_id_pending, osq_rnr_nacked_ff_valid, osq_rnr_nacked_ff_id, osq_rnr_nacked_ff_data}),
        //..i_reset_timer_osq_nacked(osq_nacked_ff | osq_empty | (i_qp_fatal & ~o_osq_nacked)),
        .i_qp_fatal             ({i_qp_fatal_valid, i_qp_fatal_id, i_qp_fatal_data}),
        .i_reload_timer_wqe     (timer_reload_for_wqe_for_i_wqe_qpid),
        .i_reload_timer_wqe_qpid(i_wqe_qpid),

        .i_reload_timer_acknack     (i_acknack_valid & !rnr_nak_error),
        .i_reload_timer_acknack_qpid(i_qp_index_acknack),

        .i_rnr_nak_rcvd(rnr_nak_rcvd),       // 1 clk delay introduced due to lut_ram implementation
        .i_rnr_nak_qpid(i_qp_index_acknack),

        .i_rnr_nak_timer            (i_pkt_nacksyndrome[4:0]),
        .i_reload_value_wqe         (i_timer_loadval_wqe),                                        //re-load value when 1st WQE goes out
        .i_reload_value_acknack     (i_timer_loadval_acknack),                                    //re-load value on other ACK/NACK recieved and other cases
        .init_start                 (init_start),
        .init_count                 (init_count),
        .osq_nacked_timer_mem_rdaddr(osq_nacked_timer_mem_rdaddr),
        .o_timer_mem_rdaddr         (timer_mem_rdaddr),
        .osq_nacked_ff_rd_data      (osq_nacked_ff_rd_data_comp),
        .i_qp_under_recovery_wr_en  (i_qp_under_recovery_wr_en && !i_qp_under_recovery_wr_en_r),
        .i_qp_under_recovery_wr_idx (i_qp_under_recovery_wr_idx),
        .i_qp_under_recovery_wr_data(i_qp_under_recovery_wr_data)

    );

    assign o_osq_timeout_set_valid = osq_timed_out_set_valid;
    assign o_osq_timeout_set_id    = osq_timed_out_set_id;

    //###############################################################################################################################################
    //1 clk gap for both the signals i_acknack_valid and i_wqe_push_req should be there
    //###############################################################################################################################################
    always_ff @(posedge core_clk) begin : timer_disabled_per_qp
        if (~core_rstn) begin
            timer_disabled_per_qp_valid         <= 1'b0;
            timer_disabled_per_qp_id            <= {C_QP_INDX_WIDTH{1'b0}};
            timer_disabled_per_qp_data          <= 1'b0;
            timer_disabled_per_qp_valid_pending <= 1'b0;
            timer_disabled_per_qp_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
            timer_disabled_per_qp_data_pending  <= 1'b0;
        end else begin
            if (i_acknack_valid) begin
                timer_disabled_per_qp_valid <= 1'b1;
                timer_disabled_per_qp_id    <= i_qp_index_acknack;
                timer_disabled_per_qp_data  <= ~(|i_timer_loadval_acknack);
            end else if (i_wqe_push_req) begin
                timer_disabled_per_qp_valid <= 1'b1;
                timer_disabled_per_qp_id    <= i_wqe_qpid;
                timer_disabled_per_qp_data  <= ~(|i_timer_loadval_wqe);
            end else if (timer_disabled_per_qp_valid_pending) begin
                timer_disabled_per_qp_valid <= 1'b1;
                timer_disabled_per_qp_id    <= timer_disabled_per_qp_id_pending;
                timer_disabled_per_qp_data  <= timer_disabled_per_qp_data_pending;
            end else begin
                timer_disabled_per_qp_valid <= 1'b0;
            end

            if (i_acknack_valid && i_wqe_push_req && i_qp_index_acknack != i_wqe_qpid) begin
                timer_disabled_per_qp_valid_pending <= 1'b1;
                timer_disabled_per_qp_id_pending    <= i_wqe_qpid;
                timer_disabled_per_qp_data_pending  <= ~(|i_timer_loadval_wqe);
            end else begin
                timer_disabled_per_qp_valid_pending <= 1'b0;
            end
        end
    end



    assign rnr_nak_rcvd = (i_acknack_valid && rnr_nak_error && ~osq_rnr_nacked_ff_rd_data_0);

    //###############################################################################################################################################
    // 1 clock gap atleast in i_acknack_valid as well as clr_timeout_n_nack_set_valid
    //###############################################################################################################################################
    always_ff @(posedge core_clk) begin : osq_rnr_nacked_ff
        if (~core_rstn) begin
            osq_rnr_nacked_ff_valid         <= 1'b0;
            osq_rnr_nacked_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_rnr_nacked_ff_data          <= 1'b0;
            osq_rnr_nacked_ff_valid_pending <= 1'b0;
            osq_rnr_nacked_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_acknack_valid && rnr_nak_error) begin
                osq_rnr_nacked_ff_valid <= 1'b1;
                osq_rnr_nacked_ff_id    <= i_qp_index_acknack;
                osq_rnr_nacked_ff_data  <= 1'b1;
            end else if (clr_timeout_n_nack_set_valid) begin
                osq_rnr_nacked_ff_valid <= 1'b1;
                osq_rnr_nacked_ff_id    <= clr_timeout_n_nack_set_id;
                osq_rnr_nacked_ff_data  <= 1'b0;
            end else if (osq_rnr_nacked_ff_valid_pending) begin
                osq_rnr_nacked_ff_valid <= 1'b1;
                osq_rnr_nacked_ff_id    <= osq_rnr_nacked_ff_id_pending;
                osq_rnr_nacked_ff_data  <= 1'b0;
            end else begin
                osq_rnr_nacked_ff_valid <= 1'b0;
            end

            if (clr_timeout_n_nack_set_valid && i_acknack_valid && rnr_nak_error && clr_timeout_n_nack_set_id != i_qp_index_acknack) begin
                osq_rnr_nacked_ff_valid_pending <= 1'b1;
                osq_rnr_nacked_ff_id_pending    <= clr_timeout_n_nack_set_id;
            end else begin
                osq_rnr_nacked_ff_valid_pending <= 1'b0;
            end
        end
    end

    assign o_osq_rnr_nacked = {osq_rnr_nacked_ff_valid_pending, osq_rnr_nacked_ff_id_pending, 1'b0, osq_rnr_nacked_ff_valid, osq_rnr_nacked_ff_id, osq_rnr_nacked_ff_data};

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_rnr_nacked_ff_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_rnr_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_rnr_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_rnr_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_index_acknack),
        .rd_data (osq_rnr_nacked_ff_rd_data_int_0)
    );
    assign osq_rnr_nacked_ff_rd_data_0 = (osq_rnr_nacked_ff_valid_pending && osq_rnr_nacked_ff_id_pending == i_qp_index_acknack) ? 1'b0 : osq_rnr_nacked_ff_rd_data_int_0;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_rnr_nacked_ff_1 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_rnr_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_rnr_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_rnr_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (qp_index_acknack_ff),
        .rd_data (osq_rnr_nacked_ff_rd_data_int_1)
    );
    assign osq_rnr_nacked_ff_rd_data_1 = (osq_rnr_nacked_ff_valid_pending && osq_rnr_nacked_ff_id_pending == qp_index_acknack_ff) ? 1'b0 : osq_rnr_nacked_ff_rd_data_int_1;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_rnr_nacked_ff_2 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_rnr_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_rnr_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_rnr_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_wqe_qpid),
        .rd_data (osq_rnr_nacked_ff_rd_data_int_2)
    );
    assign osq_rnr_nacked_ff_rd_data_2 = (osq_rnr_nacked_ff_valid_pending && osq_rnr_nacked_ff_id_pending == i_wqe_qpid) ? 1'b0 : osq_rnr_nacked_ff_rd_data_int_2;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_rnr_nacked_ff_3 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_rnr_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_rnr_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_rnr_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (fsm_arbitrated_indx),
        .rd_data (osq_rnr_nacked_ff_rd_data_int_3)
    );
    assign osq_rnr_nacked_ff_rd_data_3 = (osq_rnr_nacked_ff_valid_pending && osq_rnr_nacked_ff_id_pending == fsm_arbitrated_indx) ? 1'b0 : osq_rnr_nacked_ff_rd_data_int_3;



    //----------------------------------------------------------------------------
    //AXI master to write completions and doorbell to RDMAif
    axi_master #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_DATA_WIDTH(512),
        .C_M_AXI_THREAD_ID_WIDTH(1),
        .IP2BUS_LEN_WIDTH(IP2BUS_LEN_WIDTH)
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
        .bus2ip_byte_en (),
        .bus2ip_data    (),
        .bus2ip_dvalid  (),                 // Only used for writes
        .ip2bus_data    (ip2bus_data),
        .bus2ip_data_rdy(bus2ip_data_rdy),
        .axi_m_en       (ip2bus_wen),
        .wr_rdn         (1'b1),             //1 = write; 0 = read
        .ip2bus_addr    (ip2bus_wraddr),
        .ip2bus_len     (ip2bus_len),       //length in bytes

        // Status indicators to the core IP 
        .axi_master_done  (axi_master_done),
        .axi_master_busy  (axi_master_busy),
        .axi_master_bvalid(axi_master_bvalid),
        .axi_master_error (axi_master_error_nc)

    );

endmodule
