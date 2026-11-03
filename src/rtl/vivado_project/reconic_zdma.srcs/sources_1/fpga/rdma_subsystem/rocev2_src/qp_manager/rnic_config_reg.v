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
// rnic_config_reg.v
//-----------------------------------------------------------------------------
//
// Filename        : rnic_config_reg.v
// Version         : v1.0
// Description     : RNIC configuration/status register logic with AXI4-Lite Interface：带 AXI4-Lite 接口的 RNIC 配置/状态寄存器逻辑
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- qp mgr top
//                     -- rnic_config_reg.v
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
// Intiator
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module rnic_config_reg #(
    parameter C_S_AXI_LITE_ADDR_WIDTH = 18,
    parameter C_S_AXI_LITE_DATA_WIDTH = 32,
    parameter C_M_AXI_ADDR_WIDTH      = 32,
    parameter C_NUM_QP                = 256,
    parameter C_QP_INDX_WIDTH         = 8,
    parameter C_NUM_PD                = 256,
    parameter C_PD_INDX_WIDTH         = 8,
    parameter STB_WIDTH               = 4,
    parameter RESP_WIDTH              = 2,
    parameter C_EN_DEBUG_REGS         = 0,
    parameter C_PD_RKEY_SIZE          = 8,
    parameter C_EN_INITIATOR_LITE     = 1
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

    output reg  [C_S_AXI_LITE_DATA_WIDTH-1:0] s_axi_lite_rdata,
    output wire [             RESP_WIDTH-1:0] s_axi_lite_rresp,
    input  wire                               s_axi_lite_rready,
    output reg                                s_axi_lite_rvalid,

    output wire [RESP_WIDTH-1:0] s_axi_lite_bresp,
    input  wire                  s_axi_lite_bready,
    output reg                   s_axi_lite_bvalid,

    input wire core_clk,  // Module Clock  
    input wire core_rstn, // Active high core reset

    input wire                       i_waiting_for_rq_update,
    input wire [C_QP_INDX_WIDTH-1:0] i_waiting_for_rq_update_on_q_num,

    // 来自/前往头部验证模块的中断信号
    input wire i_pkt_valdn_err_intr,
    input wire i_mad_pkt_rcvd_intr,
    input wire i_bypass_pkt_rcvd_intr,
    input wire i_rnr_nack_gen_intr,
    input wire i_ill_opc_in_sq_intr,

    // Interrupt Status 寄存器
    output wire o_intr_clr_pkt_valdn_err,    // INTSTS[0]: 传入数据包验证错误中断
    output wire o_intr_clr_wqe_cmpl,         // INTSTS[4]: WQE 完成中断（对 QPCONF[3] 位被设置的 QP 断言）
    output wire o_intr_clr_mad_pkt_rcvd,     // INTSTS[1]: 收到传入的 MAD 数据包中断
    output wire o_intr_clr_bypass_pkt_rcvd,  // INTSTS[2]: Reserved
    output wire o_intr_clr_rnr_nak_gen,      // INTSTS[3]: RNR NACK 生成中断
    output wire o_intr_clr_ill_opc_in_sq,    // INTSTS[5]: 在发送队列中发布了非法操作代码中断
    output wire o_intr_clr_qp_pkt_rcvd,      // INTSTS[6]: RQ 数据包接收中断（对于 QPCONF[2] 位被设置的 QP，该中断被断言）
    output wire o_intr_clr_fatal_err,        // qp致命错误中断，只要有一个QP发生致命错误就会触发
    output wire o_intr_clr_cnp,              // INTSTS[8]: 用于 CNP 调度的中断使能

    // Interrupt Enable 寄存器
    output wire o_intr_en_pkt_valdn_err,    // INTEN[0]: 传入数据包验证错误中断启用
    output wire o_intr_en_wqe_cmpl,         // INTEN[4]: WQE 完成中断启用
    output wire o_intr_en_mad_pkt_rcvd,     // INTEN[1]: 接收到 MAD 数据包中断启用
    output wire o_intr_en_bypass_pkt_rcvd,  // INTEN[2]: Reserved
    output wire o_intr_en_rnr_nak_gen,      // INTEN[3]: RNR NACK 产生的中断启用
    output wire o_intr_en_ill_opc_in_sq,    // INTEN[5]: 在发送队列中发布了非法操作代码中断启用
    output wire o_intr_en_fatal_err,        // INTEN[7]: 收到致命错误中断启用
    output wire o_intr_en_cnp,              // INTEN[8]: 用于 CNP 调度的中断使能
    output wire o_intr_en_qp_pkt_rcvd,      // INTEN[6]: RQ 数据包接收中断启用

    output wire [31:0] o_rx_pkt_hndl_dbg_ctrl,  // 手册没有这个寄存器

    // tmp_cmt: status reg input ***from/to rx_pkt_intr_ctrl in rx_pkt_handler, 
    // tmp_cmt: check if interrupt clear output can be removed. 
    input  wire [                31:0] i_qp_pkt_rcvd_intr_wr_en,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_pkt_rcvd_intr_wr_addr,
    input  wire [                31:0] i_qp_pkt_rcvd_intr_wr_data,
    output wire [                31:0] o_rq_intr_sts_clr_wr_en,     // RCV Q interrupt status (bitwise)寄存器
    output wire [ C_QP_INDX_WIDTH-1:0] o_rq_intr_sts_clr_wr_addr,   // RCV Q interrupt status (bitwise) address 寄存器(addr为0, 表示QP0-31, addr为1, 表示QP32-63, 以此类推)

    // tmp_cmt: status reg input ***from/to rx_pkt_handler top,r
    // tmp_cmt: check if interrupt clear output can be removed. 
    input  wire                        i_qp_fatal_err_valid,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_fatal_err_id,
    input  wire                        i_qp_fatal_err_data,
    output reg                         o_qp_clr_fatal_err_valid,
    output reg  [C_QP_INDX_WIDTH -1:0] o_qp_clr_fatal_err_id,
    output reg                         o_qp_clr_fatal_err_data,
    output reg                         o_qp_under_recovery_valid,
    output reg  [C_QP_INDX_WIDTH -1:0] o_qp_under_recovery_id,
    output reg                         o_qp_under_recovery_data,

    // tmp_cmt: status reg input ***from/to response handler 
    // tmp_cmt: check if interrupt clear output can be removed. 
    input  wire                        i_qp_wq_cmpl_intr_or_reduce,
    input  wire [                31:0] i_qp_wq_cmpl_intr_wr_en,
    input  wire [                31:0] i_qp_wq_cmpl_intr_wr_data,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_wq_cmpl_intr_wr_addr,
    input  wire [                31:0] i_qp_wq_cmpl_intr_wr_en_pending,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_wq_cmpl_intr_wr_addr_pending,
    output wire [                31:0] o_cq_intr_sts_clr_wr_en,            // Completion Q interrupt status (bitwise)寄存器
    output wire [ C_QP_INDX_WIDTH-1:0] o_cq_intr_sts_clr_wr_addr,          // Completion Q interrupt status (bitwise) address 寄存器(addr为0, 表示QP0-31, addr为1, 表示QP32-63, 以此类推)


    //CNP
    // tmp_cmt: status reg input ***from/to rx_pkt_handler top, eval  
    input wire i_cnp_sched_intr_sts,

    input  wire [               31:0] i_cnp_schedule_info_wr_en,
    input  wire [C_QP_INDX_WIDTH-1:0] i_cnp_schedule_info_wr_addr,
    input  wire [               31:0] i_cnp_schedule_info_wr_data,
    output wire [               31:0] o_cnp_schedule_clr_wr_en,     // CNP Packed scheduled interrupt status (bitwise)寄存器
    output wire [C_QP_INDX_WIDTH-1:0] o_cnp_schedule_clr_wr_addr,   // CNP Packed scheduled interrupt status (bitwise) address 寄存器(addr为0, 表示QP0-31, addr为1, 表示QP32-63, 以此类推)

    output wire [3:0] o_cnp_timeout_val,  // XRNICADCONF[11:8]: Reserved
    output wire [3:0] o_ecn_timeout_val,  // XRNICADCONF[15:12]: Reserved


    input wire                       i_tx_path_congestion_valid,
    input wire [C_QP_INDX_WIDTH-1:0] i_tx_path_congestion_id,
    input wire                       i_tx_path_congestion_data,

    // tmp_cmt: Status inputs from Header validation module  ****move to mem
    input  wire                         i_rq_full_valid,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_rq_full_idx,
    input  wire                         i_rq_full_data,
    input  wire                         i_rq_full_reset_valid,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_rq_full_reset_idx,
    input  wire                         i_rq_full_reset_data,
    input  wire                         i_osq_full_set_valid,           // Outstanding Q full for QPn
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_osq_full_set_id,
    input  wire                         i_osq_full_reset_valid,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_osq_full_reset_id,
    input  wire                         i_osq_empty_set_valid,          // Outstanding Q empty for QPn
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_osq_empty_set_id,
    input  wire                         i_osq_empty_reset_valid,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_osq_empty_reset_id,
    input  wire                         i_qp_retried_set_valid,         // QP retried for QPn
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_qp_retried_set_id,
    input  wire                         i_qp_retried_reset_valid,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_qp_retried_reset_id,
    // 未使用
    // 仅在设置内部调试模式时使用
    input  wire                         i_osq_almost_full_set_valid,    // Outstanding Q full for QPn
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_osq_almost_full_set_id,
    input  wire                         i_osq_almost_full_reset_valid,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_osq_almost_full_reset_id,
    input  wire [ C_QP_INDX_WIDTH -1:0] i_qp_stat_msn_idx,              // Index for reading/writing MSN
    input  wire [                 23:0] i_qp_stat_msn,                  // Expected MSN for Qp incoming messages
    output wire [                 23:0] o_qp_stat_msn,                  // Expected MSN for Qp incoming messages write data from hdr validation；Status Message sequence number QPi (STATMSN)寄存器
    input  wire                         i_qp_stat_msn_req,
    input  wire                         i_qp_stat_msn_wrn,
    output wire                         o_qp_stat_msn_valid,

    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_ssn_idx,   // Index for reading/writing MSN
    input  wire [                23:0] i_qp_stat_ssn,       // Expected MSN for Qp incoming messages
    output wire [                23:0] o_qp_stat_ssn,       // Expected MSN for Qp incoming messages write data from hdr validation；Status Sender sequence number QPi (STATSSNi)寄存器
    input  wire                        i_qp_stat_ssn_req,
    input  wire                        i_qp_stat_ssn_wrn,
    output wire                        o_qp_stat_ssn_valid,

    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_timeout_idx,  // Index for reading/writing MSN
    output wire [                31:0] o_qp_timeout,      // Expected MSN for Qp incoming messages write data from hdr validation；Timeout Configuration Register for QPi (TIMEOUTCONFi)寄存器

    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_rq_pi_db_idx,   // Index for reading/writing RQ PI DB
    input  wire [                15:0] i_qp_rq_pi_db,
    output wire [                15:0] o_qp_rq_pi_db,       // Status RQ Producer index DB QPi (STATRQPIDBi)寄存器
    input  wire                        i_qp_rq_pi_db_req,
    input  wire                        i_qp_rq_pi_db_wrn,
    output wire                        o_qp_rq_pi_db_valid,

    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ret_sq_psn_idx,       // Index for reading/writing RQ PI DB
    input  wire [                23:0] i_qp_ret_sq_psn,
    output wire [                23:0] o_qp_ret_sq_psn,
    input  wire                        i_qp_ret_sq_psn_req,
    input  wire                        i_qp_ret_sq_psn_wrn,
    output wire                        o_qp_ret_sq_psn_valid,
    output wire                        o_qp_ret_sq_psn_valid_int,

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

    // Status from WQE proc engine
    input wire [31:0] i_wqe_proc_sts,
    input wire [31:0] i_rx_pkt_vld_sts,
    input wire [31:0] i_qp_mgr_sts,

    // tmp_cmt： SEND Q 仲裁的满/空状态 //****check 移动到内存
    output wire                          o_sq_empty_valid,
    output wire [C_QP_INDX_WIDTH -1 : 0] o_sq_empty_id,
    output wire                          o_sq_empty_data,      // 某个 QP 的 SEND Queue 是否为空
    // 来自缓存模块的更新信号
    input  wire                          i_status_upd_needed,
    input  wire [  C_QP_INDX_WIDTH -1:0] i_status_upd_indx,

    //Configuration outputs going to other modules (generic)
    output wire                       o_xrnic_en,                            // XRNIC Configuration寄存器 -- [0] ERNIC Enable
    output wire                       o_xrnic_adv_conf_sw_override,          // XRNIC Advance Configuration寄存器 -- [0] SW override enable
    output wire [C_QP_INDX_WIDTH-1:0] o_xrnic_adv_conf_sw_override_qpid,     // XRNIC Advance Configuration寄存器 -- [30:20] Software Override QP Number
    output wire                       o_xrnic_adv_conf_errbuf_overwr_en,     // XRNIC Advance Configuration寄存器 -- [1] Reserved
    output wire                       o_xrnic_adv_conf_retry_cnt_fatal_dis,  // XRNIC Advance Configuration寄存器 -- [2] retry_cnt_fatal_dis
    // tmp_cmt: SEND Q Full/empty status for arbitration //****check move to mem
    input  wire [  C_QP_INDX_WIDTH:0] i_sw_override_per_qp_en,               // {o_xrnic_adv_conf_sw_override, o_xrnic_adv_conf_sw_override_qpid}
    output wire [                3:0] o_xrnic_adv_base_cnt,                  // XRNIC Advance Configuration寄存器 -- [19:16] Base count width
    output wire [                1:0] o_tx_ack_gen,                          // XRNIC Configuration寄存器 -- [4:3] TX ACK generation.
    output wire                       o_err_buf_en,                          // XRNIC Configuration寄存器 -- [5] Error buffer enable
    output wire [                1:0] o_flow_credits,                        // XRNIC Configuration寄存器 -- [7:6] Reserved
    output wire [               15:0] o_xrnic_udp_src_port,                  // XRNIC Configuration寄存器 -- [23:8] UDP source port for outgoing packets
    output wire                       o_depkt_bypass_en,                     // 启用 XRNIC 接口旁路功能 -- XRNIC Configuration寄存器 -- [2] Reserved
    output wire [C_QP_INDX_WIDTH-1:0] o_num_qp_en,                           // 启用的 QP 数量 -- XRNIC_CONF_QP_EN寄存器 -- [10:0] Number of QPs enabled
    output wire [               47:0] o_mac_xrnic_addr,                      // 来自 XRNIC 寄存器的以太网 MAC 目标（自有）地址 -- MAC XRNIC Address寄存器
    output wire [               31:0] o_ipv4_xrnic_addr,                     // 来自 XRNIC 寄存器的 IPv4 目的地（自身）地址（用于 Depkt 模块） -- IPv4 XRNIC Address寄存器
    output wire [              127:0] o_ipv6_xrnic_addr,                     // XRNIC 寄存器中的 IPv6 目的地（自有）地址（适用于 depkt 模块） -- IPv6 Address寄存器

    output wire [31:0] o_bypass_buf_ba,           // 手册没有这个寄存器
    output wire [15:0] o_bypass_buf_sz_num_bufs,  // 手册没有这个寄存器
    output wire [15:0] o_bypass_buf_sz_buf_sz,    // 手册没有这个寄存器
    input  wire [15:0] i_bypass_buf_wrptr,

    // RNIC 使用这些数据缓冲区来保存所有需要传出的 RDMA WRITE 数据，直到远程主机确认为止。如果发生重传，将从这些缓冲区中提取重试的数据。
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_data_buf_ba,           // Data Buffer Base Address寄存器
    output wire [                  15:0] o_data_buf_sz_num_bufs,  // Data Buffer Size寄存器 -- [15:0] Number of data buffers
    output wire [                  15:0] o_data_buf_sz_buf_sz,    // Data Buffer Size寄存器 -- [31:16] Data buffer size in bytes

    //  output wire [15:0]                                o_connect_io_residual_rq,
    output reg  [               12:0] o_connect_io_qp_rq_pi_db_wptr,  // Connect IP configuration寄存器 -- [12:0] Reserved
    output      [C_QP_INDX_WIDTH-1:0] o_connect_io_qpid,              // Connect IP configuration寄存器 -- [26:16] Queue pair ID
    input  wire                       i_connect_io_qp_rq_pi_db_rdy,
    output reg  [               15:0] o_rq_pi_db_hw_hndshk,
    output wire                       o_hw_hndshk_disable_to_0,

    // Error Buffer: IP 会根据未通过验证的数据包更新这些缓冲区。
    //               通过向 XRNICCONF[5]写 1，可启用向该缓冲区写入所有验证错误信息。
    //               如果禁用该位，则只有导致 QP 进入 FATAL 状态的数据包才会写入错误缓冲区。
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_rq_err_pkt_buf_ba,           // Error Buffer Base Address寄存器
    output wire [                  15:0] o_rq_err_pkt_buf_sz_num_bufs,  // Error Buffer Size寄存器 -- [15:0] Number of Error buffers
    output wire [                  15:0] o_rq_err_pkt_buf_sz_buf_sz,    // Error Buffer Size寄存器 -- [31:16] Size of each Error buffer，单位为字节
    input  wire [                  15:0] i_rq_err_pkt_buf_wrptr,

    // Response Error pkt buffer: IP 使用这些错误缓冲区保存所有错误响应错误数据包的报头信息。
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_resp_err_pkt_buf_ba,           // Response Error pkt buffer base address寄存器
    output wire [                  15:0] o_resp_err_pkt_buf_sz_num_bufs,  // Response Error pkt buffer size address寄存器 -- [15:0] Number of entries for response error buffer
    output wire [                  15:0] o_resp_err_pkt_buf_sz_buf_sz,    // Response Error pkt buffer size address寄存器 -- [31:16] Size of each entry of response error buffer
    input  wire [                  15:0] i_resp_err_pkt_buf_wrptr,

    output reg  [31:0] o_error_corrupt_conf,      // 手册没有这个寄存器
    input              clr_error_corrupt_en,
    output wire [15:0] o_tx_hdr_buf_sz_num_hdrs,  // 手册没有这个寄存器
    output wire [15:0] o_tx_hdr_buf_sz_buf_sz,    // 手册没有这个寄存器

    output wire [31:0] o_tx_sgl_buf_ba,           // 手册没有这个寄存器
    output wire [15:0] o_tx_sgl_buf_sz_num_sgls,  // 手册没有这个寄存器
    output wire [15:0] o_tx_sgl_buf_sz_buf_sz,    // 手册没有这个寄存器

    output wire                        o_xrnic_conf_ipver,  // XRNIC Configuration寄存器 -- [1] Reserved
    // tmp_cmt:
    output reg                         qp_enable_valid,
    output reg  [C_QP_INDX_WIDTH -1:0] qp_enable_id,
    output reg                         qp_enable_data,

    output wire [                  31:0] o_timeoutreg,              // 固定值: {18'b0, 3'b1, 3'b1, 8'b0} = 32‘h900 = 32'd2304
    // tmp_cmt:
    output reg                           o_qp_disable_pulse_valid,
    output reg  [C_QP_INDX_WIDTH -1 : 0] o_qp_disable_pulse_id,
    output reg                           o_qp_disable_pulse_data,

    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_out_errsts_q_ba,    // Outgoing Pkt Error status Queue Base address寄存器
    output wire [                  15:0] o_out_errsts_q_sz,    // Outgoing Error status Q size寄存器
    input  wire [                  15:0] i_out_errsts_q_wrptr,

    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_in_errsts_q_ba,    // Incoming Pkt Error Status Queue Base address寄存器
    output wire [                  15:0] o_in_errsts_q_sz,    // Incoming Pkt Error Status Queue Size寄存器 -- [15:0] Number of incoming error pkt status queue entries
    input  wire [                  15:0] i_in_errsts_q_wrptr,

    //Configuration outputs going to other modules (PER QP)

    input  wire                        i_qp_conf_req_resp_hndl,
    input  wire                        i_qp_conf_req_wqe_proc,
    input  wire                        i_qp_conf_req_rx_pkt,
    output wire                        o_qp_conf_valid_resp_hndl,
    output wire                        o_qp_conf_valid_wqe_proc,
    output wire                        o_qp_conf_valid_rx_pkt,
    output wire [                31:0] o_qp_conf,                  // QP Configuration QPi(QPCONFi) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_conf_idx,              // Index for reading Qp configuration
    output wire                        o_qp_cfg_retry_cnt_rd_rdy,  //added newly for QP_fatal case
    input  wire                        i_qp_cfg_retry_cnt_rd_req,  //added newly for QP_fatal case

    output wire [                31:0] o_qp_conf_replica,       // QP configuration QPi(QPCONFi) REPLICA -- QP 配置 REPLICA
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_conf_replica_idx,   // Index for reading Qp configuration REPLICA
    input  wire                        i_qp_conf_replica_req,
    output wire                        o_qp_conf_replica_valid,

    input  wire                        i_qp_adv_conf_req,
    output wire                        o_qp_adv_conf_valid,
    output wire [                31:0] o_qp_adv_conf,        // QP advanced configuration QPi (QPADVCONFi) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_adv_conf_idx,    // Index for reading Qp configuration

    input  wire                          i_qp_rq_ba_req,
    output wire                          o_qp_rq_ba_valid,
    output wire [C_M_AXI_ADDR_WIDTH-9:0] o_qp_rq_ba,        // RCV Q Buffer base address QPi (RQBAi) 寄存器
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_rq_ba_idx,    // Index for reading Qp RQ Buffer BA

    input  wire                          i_qp_sq_ba_req_int,
    input  wire                          i_qp_sq_ba_req,
    output wire                          o_qp_sq_ba_valid_int,
    output wire                          o_qp_sq_ba_valid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_qp_sq_ba,            // SEND Q base address QPi (SQBAi) 寄存器
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_sq_ba_idx,        // Index for reading Qp SQ BA

    input  wire                          i_qp_cq_ba_req,
    output wire                          o_qp_cq_ba_valid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_qp_cq_ba,        // CQ base address QPi (CQBAi) 寄存器
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_cq_ba_idx,    // Index for reading Qp CQ BA

    input  wire                          i_qp_rq_wrptrdb_add_req,
    output wire                          o_qp_rq_wrptrdb_add_valid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_qp_rq_wrptrdb_add,        // RCV Q Write pointer DB address QPi (RQWPTRDBADDi) 寄存器
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_rq_wrptrdb_add_idx,    // Index for reading Qp SQ BA

    input  wire                          i_qp_sq_cmpldb_add_req,
    output wire                          o_qp_sq_cmpldb_add_valid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_qp_sq_cmpldb_add,        // CQ DB address QPi (CQDBADDi) 寄存器
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_sq_cmpldb_add_idx,    // Index for reading SQ Completion DB address

    input  wire                        i_qp_cq_hdptr_req,
    input  wire                        i_qp_cq_hdptr_retry_req,
    output wire                        o_qp_cq_hdptr_valid,
    output wire                        o_qp_cq_hdptr_valid_retry,
    output wire [                15:0] o_qp_cq_hdptr,              // CQ head pointer QPi (CQHEADi) 寄存器
    input  wire [                15:0] i_qp_cq_hdptr,
    input  wire                        i_qp_cq_hdptr_wrn,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_cq_hdptr_idx,          // Index for reading CQ head pointer

    input  wire                        i_qp_rq_cidb_req,
    output wire                        o_qp_rq_cidb_valid,
    output wire [                31:0] o_qp_rq_cidb,        // RQ Consumer Index QPi (RQCIi) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_rq_cidb_idx,    // Index for reading RQ CI DB

    output wire [                15:0] o_qp_sq_pidb,       // SQ Producer index QPi (SQPIi) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_pidb_idx,   // Index for reading SQ PI DB
    input  wire                        i_qp_sq_pidb_req,
    output wire                        o_qp_sq_pidb_valid,

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

    input  wire                        i_qp_rq_depth_req,
    output wire                        o_qp_rq_depth_valid,
    output wire [                15:0] o_qp_rq_depth,        // Q Depth QPi (QDEPTHi) 寄存器 -- [31:16]: Receive Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_rq_depth_idx,    // Index for reading RQ depth

    input  wire                        i_qp_sq_depth_req,
    output wire                        o_qp_sq_depth_valid,
    output wire [                15:0] o_qp_sq_depth,        // Q Depth QPi (QDEPTHi) 寄存器 -- [15:0]: Send Q depth (Send CQ will of the same depth)
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_depth_idx,    // Index for reading RQ depth

    // CQ depth information to Response handler. 
    // The CQ depth is equal to SQ depth. This information is shared by QP cache
    // manager and resp handler. Priority is for cache manager
    input  wire                        i_qp_cq_depth_req,
    output wire                        o_qp_cq_depth_valid,
    output wire                        o_qp_cq_depth_rdy,
    output wire [                15:0] o_qp_cq_depth,        // Q Depth QPi (QDEPTHi) 寄存器 -- [15:0]: Send Q depth (Send CQ will of the same depth)
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_cq_depth_idx,    // Index for reading Q depth

    input  wire                        i_qp_sq_psn_req,          // RX 路径的请求
    input  wire                        i_qp_sq_psn_wqe_req,      // SQ WQE 路径的请求
    input  wire                        i_qp_sq_psn_retry_req,
    output wire                        o_qp_sq_psn_valid,
    output wire                        o_qp_sq_psn_wqe_valid,
    output reg                         o_qp_sq_psn_resp_valid,
    output wire                        o_qp_sq_psn_retry_valid,
    output wire [                23:0] o_qp_sq_psn,              // SEND Q PSN for QPi (SQPSNi) 寄存器
    input  wire                        i_qp_sq_psn_wrn,
    input  wire                        i_qp_sq_psn_retry_wrn,
    input  wire [                23:0] i_qp_sq_psn,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_idx,          // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_wqe_idx,      // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_retry_idx,    // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_resp_idx,     // Index for reading Q depth

    output wire [                31:0] o_qp_last_rq,       // Last RQ req for QPi (LSTRQREQi) 寄存器
    input  wire [                31:0] i_qp_last_rq,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_last_rq_idx,   // Index for reading Q depth
    input  wire                        i_qp_last_rq_req,
    input  wire                        i_qp_last_rq_wrn,
    output wire                        o_qp_last_rq_valid,

    input  wire                        i_qp_pd_num_wrn,
    output wire [                23:0] o_qp_pd_num,        // Protection domain number 寄存器
    input  wire [                23:0] i_qp_pd_num,
    output wire                        o_qp_pd_num_valid,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_pd_num_idx,
    input  wire                        i_qp_pd_num_req,

    input  wire                        i_pd_pd_num_wrn,
    output wire [                23:0] o_pd_pd_num,        // Protection Domain Number (PDPDNUM) 寄存器
    input  wire [                23:0] i_pd_pd_num,
    output wire                        o_pd_pd_num_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_pd_num_idx,
    input  wire                        i_pd_pd_num_req,

    input  wire                        i_pd_virt_addr_0_wrn,
    output wire [                31:0] o_pd_virt_addr_0,        // Virtual address LSB (VIRTADDRLSB) 寄存器
    input  wire [                31:0] i_pd_virt_addr_0,
    output wire                        o_pd_virt_addr_0_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_virt_addr_0_idx,    // Index for reading Q depth
    input  wire                        i_pd_virt_addr_0_req,

    input  wire                        i_pd_virt_addr_1_wrn,
    output wire [                31:0] o_pd_virt_addr_1,        // Virtual address MSB (VIRTADDRMSB) 寄存器
    input  wire [                31:0] i_pd_virt_addr_1,
    output wire                        o_pd_virt_addr_1_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_virt_addr_1_idx,    // Index for reading Q depth
    input  wire                        i_pd_virt_addr_1_req,

    input  wire                        i_pd_buf_base_addr_0_wrn,
    output wire [                31:0] o_pd_buf_base_addr_0,        // Buffer Base address LSB (BUFBASEADDRLSB) 寄存器
    input  wire [                31:0] i_pd_buf_base_addr_0,
    output wire                        o_pd_buf_base_addr_0_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_buf_base_addr_0_idx,    // Index for reading Q depth
    input  wire                        i_pd_buf_base_addr_0_req,

    input  wire                        i_pd_buf_base_addr_1_wrn,
    output wire [                31:0] o_pd_buf_base_addr_1,        // Buffer base address MSB (BUFBASEADDRMSB) 寄存器
    input  wire [                31:0] i_pd_buf_base_addr_1,
    output wire                        o_pd_buf_base_addr_1_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_buf_base_addr_1_idx,    // Index for reading Q depth
    input  wire                        i_pd_buf_base_addr_1_req,

    input  wire                        i_pd_buf_r_key_wrn,
    output wire [  C_PD_RKEY_SIZE-1:0] o_pd_buf_r_key,        // Buffer R_Key (BUFRKEY) 寄存器
    input  wire [  C_PD_RKEY_SIZE-1:0] i_pd_buf_r_key,
    output wire                        o_pd_buf_r_key_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_buf_r_key_idx,    // Index for reading Q depth
    input  wire                        i_pd_buf_r_key_req,

    input  wire                        i_pd_w_r_buf_len_wrn,
    output wire [                47:0] o_pd_w_r_buf_len,        // Write/Read buffer length (WRRDBUFLEN) 寄存器
    input  wire [                47:0] i_pd_w_r_buf_len,
    output wire                        o_pd_w_r_buf_len_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_w_r_buf_len_idx,    // Index for reading Q depth
    input  wire                        i_pd_w_r_buf_len_req,

    input  wire                        i_pd_access_type_wrn,
    output wire [                 3:0] o_pd_access_type,        // Access Description (ACCESSDESC) 寄存器 -- [3:0] Access Type
    input  wire [                 3:0] i_pd_access_type,
    output wire                        o_pd_access_type_valid,
    input  wire [C_PD_INDX_WIDTH -1:0] i_pd_access_type_idx,    // Index for reading Q depth
    input  wire                        i_pd_access_type_req,

    input  wire                        i_qp_dest_qpid_req_wqe_proc,
    input  wire                        i_qp_dest_qpid_req_rx_pkt,
    output wire                        o_qp_dest_qpid_valid_wqe_proc,
    output wire                        o_qp_dest_qpid_valid_rx_pkt,
    output wire [                23:0] o_qp_dest_qpid,                 // Destination QP configuration for QPi (DESTQPCONFi) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_dest_qpid_idx,             // Index for reading Q depth

    //  input  wire                                       i_qp_rnr_nck_tval_curr_retry_req,
    //  input  wire                                       i_qp_rnr_nck_tval_curr_retry_wrn,
    //  input  wire  [7:0]                                i_qp_rnr_nck_tval_curr_retry, // CURR_RETRY_CNT [7:5] ;RNR_NACK_TVAL [4:0]

    input  wire                        i_qp_mac_remote_addrl_req,
    input  wire                        i_qp_mac_remote_addrl_wqe_req,
    output wire                        o_qp_mac_remote_addrl_valid,
    output wire                        o_qp_mac_remote_addrl_wqe_valid,
    output wire [                31:0] o_qp_mac_remote_addrl,            // MAC destination address LSB QPi (MACDESADDLSBi) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_idx,        // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_wqe_idx,    // Index for reading Q depth

    input  wire                        i_qp_mac_remote_addrl_replica_req,
    output wire [                31:0] o_qp_mac_remote_addrl_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_replica_idx,   // Index for reading Q depth
    output wire                        o_qp_mac_remote_addrl_replica_valid,

    input  wire                        i_qp_mac_remote_addrm_req,
    input  wire                        i_qp_mac_remote_addrm_wqe_req,
    output wire                        o_qp_mac_remote_addrm_valid,
    output wire                        o_qp_mac_remote_addrm_wqe_valid,
    output wire [                31:0] o_qp_mac_remote_addrm,            // MAC destination address MSB QPi (MACDESADDMSBi) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_idx,        // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_wqe_idx,    // Index for reading Q depth

    output wire [                31:0] o_qp_mac_remote_addrm_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_replica_idx,   // Index for reading Q depth
    input  wire                        i_qp_mac_remote_addrm_replica_req,
    output wire                        o_qp_mac_remote_addrm_replica_valid,

    input  wire                        i_qp_ip_remote_addr1_req,
    output wire                        o_qp_ip_remote_addr1_valid,
    input  wire                        i_qp_ip_remote_addr1_wqe_req,
    output wire                        o_qp_ip_remote_addr1_wqe_valid,
    output wire [                31:0] o_qp_ip_remote_addr1,            // IP destination address 1 QPi (IPDESADDR1i) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_idx,        // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_wqe_idx,    // Index for reading Q depth

    output wire [                31:0] o_qp_ip_remote_addr1_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_replica_idx,   // Index for reading Q depth
    input  wire                        i_qp_ip_remote_addr1_replica_req,
    output wire                        o_qp_ip_remote_addr1_replica_valid,

    input  wire                        i_qp_ip_remote_addr2_req,
    input  wire                        i_qp_ip_remote_addr2_wqe_req,
    output wire                        o_qp_ip_remote_addr2_valid,
    output wire                        o_qp_ip_remote_addr2_wqe_valid,
    output wire [                31:0] o_qp_ip_remote_addr2,            // IP destination address 2 QPi (IPDESADDR2i) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_idx,        // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_wqe_idx,    // Index for reading Q depth

    output wire [                31:0] o_qp_ip_remote_addr2_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_replica_idx,   // Index for reading Q depth
    input  wire                        i_qp_ip_remote_addr2_replica_req,
    output wire                        o_qp_ip_remote_addr2_replica_valid,

    input  wire                        i_qp_ip_remote_addr3_req,
    output wire                        o_qp_ip_remote_addr3_valid,
    input  wire                        i_qp_ip_remote_addr3_wqe_req,
    output wire                        o_qp_ip_remote_addr3_wqe_valid,
    output wire [                31:0] o_qp_ip_remote_addr3,            // IP destination address 3 QPi (IPDESADDR3i) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_idx,        // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_wqe_idx,    // Index for reading Q depth

    input  wire                        i_qp_ip_remote_addr3_replica_req,
    output wire                        o_qp_ip_remote_addr3_replica_valid,
    output wire [                31:0] o_qp_ip_remote_addr3_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_replica_idx,    // Index for reading Q depth

    input  wire                        i_qp_ip_remote_addr4_req,
    output wire                        o_qp_ip_remote_addr4_valid,
    input  wire                        i_qp_ip_remote_addr4_wqe_req,
    output wire                        o_qp_ip_remote_addr4_wqe_valid,
    output wire [                31:0] o_qp_ip_remote_addr4,            // IP destination address 4 QPi (IPDESADDR4i) 寄存器
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_idx,        // Index for reading Q depth
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_wqe_idx,    // Index for reading Q depth

    input  wire                        i_qp_ip_remote_addr4_replica_req,
    output wire                        o_qp_ip_remote_addr4_replica_valid,
    output wire [                31:0] o_qp_ip_remote_addr4_replica,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_replica_idx,    // Index for reading Q depth

    input  wire [                15:0] i_qp_curr_sqptr_proc,
    input  wire                        i_qp_curr_sqptr_proc_wen,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_curr_sqptr_proc_idx,    // Index for writing curent SQ pointer
    output wire                        o_qp_curr_sqptr_proc_valid,
    input  wire                        i_qp_curr_sqptr_rdreq,
    output wire [                15:0] o_qp_curr_sqptr_proc,        // Status Current SQ ptr under process (STATCURSQPTRi) 寄存器

    // Response PSN status register. Read/write interface to RX pkt handler
    // 响应 PSN 状态寄存器。RX pkt 处理器的读/写接口
    input  wire [                23:0] i_qp_stat_resp_psn,
    input  wire                        i_qp_stat_resp_psn_wen,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_resp_psn_idx,
    output wire                        o_qp_stat_resp_psn_valid,
    input  wire                        i_qp_stat_resp_psn_rdreq,
    output wire [                23:0] o_qp_stat_resp_psn,

    // Status NAK details from RX paket handler
    // 来自 RX 包处理程序的状态 NAK 详情
    input  wire [                 9:0] i_qp_stat_nak,
    input  wire                        i_qp_stat_nak_wen,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_nak_idx,
    output wire                        o_qp_stat_nak_valid,
    input  wire                        i_qp_stat_nak_rdreq,
    output wire [                 9:0] o_qp_stat_nak,

    // Status NAK details from resp handler
    // 来自响应处理程序的状态 NAK 详情
    input  wire [                 2:0] i_qp_stat_retry_cnt,
    input  wire                        i_qp_stat_retry_cnt_wen,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_retry_cnt_idx,
    output wire                        o_qp_stat_retry_cnt_valid,
    input  wire                        i_qp_stat_retry_cnt_rdreq,
    output wire [                 2:0] o_qp_stat_retry_cnt,

    // RQ buffer current address
    // RQ 缓冲区当前地址
    input  wire [C_M_AXI_ADDR_WIDTH-9:0] i_qp_stat_rq_buf_ca,
    input  wire                          i_qp_stat_rq_buf_ca_wen,
    input  wire [  C_QP_INDX_WIDTH -1:0] i_qp_stat_rq_buf_ca_idx,
    output wire                          o_qp_stat_rq_buf_ca_valid,
    input  wire                          i_qp_stat_rq_buf_ca_rdreq,
    output wire [C_M_AXI_ADDR_WIDTH-9:0] o_qp_stat_rq_buf_ca,

    // WQE push status interface
    input  wire [                15:0] i_qp_stat_wqe_cnt,
    input  wire                        i_qp_stat_wqe_cnt_wen,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_stat_wqe_cnt_idx,
    output wire                        o_qp_stat_wqe_cnt_valid,
    input  wire                        i_qp_stat_wqe_cnt_rdreq,
    output wire [                15:0] o_qp_stat_wqe_cnt,

    // Performance debug counters i/f
    output wire [                 15:0] o_global_dbg_cnt_value,
    output wire                         o_global_dbg_cnt_clr,
    output wire                         o_global_dbg_cnt_en,
    input  wire [                 15:0] i_wqe_fsm_idle_cnt,
    input  wire [                 15:0] i_hdr_backpressure_cnt,
    input  wire [                 15:0] i_retry_tx_backpressure_cnt,
    input  wire [                 15:0] i_wqe_proc_rd_wqe_cnt,
    input  wire [                 15:0] i_wqe_proc_rd_q_info_cnt,
    input  wire [                 15:0] i_wqe_proc_wait0_cnt,
    input  wire [                 15:0] i_wqe_proc_ip_chksum_cnt,
    input  wire [                 15:0] i_wqe_proc_hdr_gen_cnt,
    input  wire [                 15:0] i_wqe_proc_hdr_sto_cnt,
    // Pause_Offset
    output wire [                  3:0] roce_priority,                // XRNIC_PAUSE_CONF -- [7:4] Pause priority for RoCE (Binary encoding)
    output wire [                  3:0] non_roce_priority,            // XRNIC_PAUSE_CONF -- [11:8] Pause priority for non-RoCE (Binary encoding)
    output                              disable_priority_chk,         // XRNIC_PAUSE_CONF -- [13] Disable priority check (applicable only for TX data path)
    output wire                         pfc_en_roce,                  // XRNIC_PAUSE_CONF -- [0] Pause enable RoCE
    output wire                         pfc_en_non_roce,              // XRNIC_PAUSE_CONF -- [1] Pause enable non-RoCE
    output wire [                 15:0] roce_pause_on_offset,         // XRNIC_BUF_THRESHOLD_ROCE -- [15:0] XON Value for RoCE
    output wire [                 15:0] roce_pause_off_offset,        // XRNIC_BUF_THRESHOLD_ROCE -- [31:16] XOFF Value for RoCE
    output wire [                 15:0] non_roce_pause_on_offset,     // XRNIC_BUF_THRESHOLD_NON_ROCE -- [15:0] XON Value for non-RoCE
    output wire [                 15:0] non_roce_pause_off_offset,    // XRNIC_BUF_THRESHOLD_NON_ROCE -- [31:16] XOFF value for non-RoCE
    input  wire                         init_start,
    input  wire [C_QP_INDX_WIDTH-1 : 0] init_count,

    // Debug registers
    input  wire [ 7:0] i_axi_rd_err_cnt,
    input  wire [31:0] i_last_in_pkt_info,
    input  wire [31:0] i_last_out_pkt_info,
    output wire        dbg_sq_empty_ff_and_reduce,       // 所有 QP 的 SQ 当前是否都为空
    output wire        o_prog_full_db_update_curr_sqptr

);
    `include "../../rdma_macros.vh"

    // AXI lite operation registers
    reg [                  20:0] wr_addr_r;
    reg [                  20:0] rd_addr_r;
    reg                          wr_req_r;
    reg                          wr_req_mask_r;
    reg                          clr_wr_req_mask_r;
    reg                          rd_req_r;
    reg                          reset_released_r;
    reg                          rvalid_delayed;
    reg                          rvalid_delayed_i;
    reg                          rd_req_mask;
    reg                          axi_lite_rvalid_i;
    reg [                  31:0] rx_pkt_hndl_dbg_ctrl;
    reg [                  15:0] out_cnp_pkt_cnt;

    reg                          xrnic_conf_xrnic_en;
    reg                          xrnic_conf_ipver;
    reg                          xrnic_conf_depkt_bypass_en;
    reg [                   1:0] xrnic_conf_tx_ack_gen;
    reg                          xrnic_conf_err_buf_en;
    reg [                   1:0] xrnic_conf_flow_credits;
    reg [ C_QP_INDX_WIDTH-1 : 0] xrnic_conf_num_qps_enabled;
    reg [                  15:0] xrnic_conf_udp_src_port;

    reg [                  15:0] xrnic_sw_override_cnt;
    reg                          xrnic_adv_conf_sw_override;  // 上位机设置为0
    reg [   C_QP_INDX_WIDTH-1:0] xrnic_adv_conf_sw_override_qpid;
    reg                          xrnic_adv_conf_sw_override_ff;
    reg                          xrnic_adv_conf_errbuf_overwr_en;
    reg                          xrnic_adv_conf_retry_cnt_fatal_dis;
    reg [                   3:0] xrnic_adv_base_cnt;
    reg [                   3:0] cnp_timeout;
    reg [                   3:0] ecn_timeout;

    reg [                  31:0] mac_xrnic_addr_lsb;
    reg [                  15:0] mac_xrnic_addr_msb;

    reg [                  15:0] roce_pause_on_offset_reg;
    reg [                  15:0] roce_pause_off_offset_reg;
    reg [                  15:0] non_roce_pause_on_offset_reg;
    reg [                  15:0] non_roce_pause_off_offset_reg;
    reg                          pfc_en_roce_reg;
    reg                          pfc_en_non_roce_reg;
    reg [                   3:0] roce_priority_reg;
    reg [                   3:0] non_roce_priority_reg;
    reg                          disable_priority_chk_reg;

    reg [                  31:0] ipv4_xrnic_addr;
    reg [                  31:0] ipv6_xrnic_addr1;
    reg [                  31:0] ipv6_xrnic_addr2;
    reg [                  31:0] ipv6_xrnic_addr3;
    reg [                  31:0] ipv6_xrnic_addr4;
    reg [                  31:0] error_corrupt_conf;
    reg [                  15:0] tx_hdr_buf_sz_num_hdrs;
    reg [                  15:0] tx_hdr_buf_sz_buf_sz;

    reg [                  31:0] tx_sgl_buf_ba;
    reg [                  15:0] tx_sgl_buf_sz_num_sgls;
    reg [                  15:0] tx_sgl_buf_sz_buf_sz;

    reg [                  31:0] bypass_buf_ba;
    reg [                  15:0] bypass_buf_sz_num_bufs;
    reg [                  15:0] bypass_buf_sz_buf_sz;
    //reg [15:0] bypass_buf_wrptr;

    reg [                  31:0] data_buf_ba;
    reg [                  31:0] data_buf_ba_msb;
    reg [                  15:0] data_buf_sz_num_bufs;
    reg [                  15:0] data_buf_sz_buf_sz;
    reg                          hw_hndshk_disable_to_0;

    //reg [15:0] connect_io_residual_rq;
    reg [                  12:0] connect_io_qp_rq_pi_db_wptr;
    reg [                  10:0] connect_io_qpid;

    reg [                  31:0] rq_err_pkt_buf_ba;
    reg [                  31:0] rq_err_pkt_buf_ba_msb;
    reg [                  15:0] rq_err_pkt_buf_sz_num_bufs;
    reg [                  15:0] rq_err_pkt_buf_sz_buf_sz;
    //reg [15:0] err_pkt_buf_wrptr;
    reg [                  31:0] resp_err_pkt_buf_ba;
    reg [                  31:0] resp_err_pkt_buf_msb_ba;
    reg [                  15:0] resp_err_pkt_buf_sz_num_bufs;
    reg [                  15:0] resp_err_pkt_buf_sz_buf_sz;

    reg [                   4:0] timeout;
    reg [                   2:0] retry_cnt;
    reg [                   2:0] rnr_retry_cnt;
    reg [                   4:0] rnr_nack_tval;

    reg [C_QP_INDX_WIDTH -1 : 0] qp_conf_wr_addr_r;
    reg [C_QP_INDX_WIDTH -1 : 0] qp_conf_wr_addr_2r;
    reg                          qp_conf_bram_wen_d;

    reg [                  31:0] out_errsts_q_ba;
    reg [                  31:0] out_errsts_q_ba_msb;
    reg [                  15:0] out_errsts_q_sz;

    reg [                  31:0] in_errsts_q_ba;
    reg [                  31:0] in_errsts_q_ba_msb;
    reg [                  15:0] in_errsts_q_sz;
    reg [                  31:0] global_dbg_cnt;
    reg [                  15:0] int_cnt_value;


    reg                          intr_en_pkt_valdn_err;
    reg                          intr_en_wqe_cmpl;
    reg                          intr_en_mad_pkt_rcvd;
    reg                          intr_en_bypass_pkt_rcvd;
    reg                          intr_en_rnr_nak_gen;
    reg                          intr_en_ill_opc_in_sq;
    reg                          intr_en_fatal_err;
    reg                          intr_en_cnp;
    reg                          intr_en_qp_pkt_rcvd;
    //wire [31:0] o_qp_timeout;
    reg [                  31:0] i_qp_timeout_stat;
    //reg [C_QP_INDX_WIDTH -1: 0] i_qp_timeout_idx;    // Index for reading/writing MSN


    reg                          qp_cq_hdptr_req_ff;
    reg                          qp_cq_hdptr_retry_req_ff;
    reg                          qp_conf_req_wqe_proc_ff;
    reg                          qp_conf_req_resp_hndl_ff;
    reg                          qp_cfg_retry_cnt_rd_req_ff;
    reg                          qp_conf_req_rx_pkt_ff;
    reg                          qp_conf_req_replica_ff;
    reg                          qp_adv_conf_req_ff;
    reg                          qp_rq_pi_db_req_ff;
    reg                          qp_ret_sq_psn_req_ff;
    reg                          qp_ret_sq_psn_req_int_ff;
    reg                          qp_rq_ba_req_ff;
    reg                          qp_sq_ba_req_ff;
    reg                          qp_sq_ba_req_int_ff;
    reg                          qp_cq_ba_req_ff;
    reg                          qp_rq_wrptrdb_req_ff;
    reg                          qp_sq_cmpldb_req_ff;
    reg                          qp_rq_cidb_req_ff;
    reg                          qp_sq_pidb_req_ff;
    reg                          qp_sq_pidb_req_hndshk_ff;
    reg                          qp_rq_cidb_req_hndshk_ff;
    reg                          qp_rq_depth_req_ff;
    reg                          qp_sq_depth_req_ff;
    reg                          qp_cq_depth_req_ff;
    reg                          qp_sq_psn_req_ff;
    reg                          qp_sq_psn_wqe_req_ff;
    reg                          qp_sq_psn_retry_req_ff;
    reg                          qp_last_rq_req_ff;
    reg                          pd_virt_addr_0_ff;
    reg                          pd_virt_addr_1_ff;
    reg                          pd_buf_base_addr_0_ff;
    reg                          pd_buf_base_addr_1_ff;
    reg                          pd_buf_r_key_ff;
    reg                          pd_w_r_buf_len_ff;
    reg                          qp_dest_qpid_req_wqe_proc_ff;
    reg                          qp_dest_qpid_req_rx_pkt_ff;
    reg                          qp_mac_remote_addrl_req_ff;
    reg                          qp_mac_remote_addrl_req_replica_ff;
    reg                          qp_mac_remote_addrm_req_ff;
    reg                          qp_mac_remote_addrm_req_replica_ff;
    reg                          qp_mac_remote_addrl_wqe_req_ff;
    reg                          qp_mac_remote_addrm_wqe_req_ff;
    reg                          qp_ip_remote_addr1_req_ff;
    reg                          qp_ip_remote_addr2_req_ff;
    reg                          qp_ip_remote_addr3_req_ff;
    reg                          qp_ip_remote_addr4_req_ff;
    reg                          qp_ip_remote_addr1_req_replica_ff;
    reg                          qp_ip_remote_addr2_req_replica_ff;
    reg                          qp_ip_remote_addr3_req_replica_ff;
    reg                          qp_ip_remote_addr4_req_replica_ff;
    reg                          qp_ip_remote_addr1_wqe_req_ff;
    reg                          qp_ip_remote_addr2_wqe_req_ff;
    reg                          qp_ip_remote_addr3_wqe_req_ff;
    reg                          qp_ip_remote_addr4_wqe_req_ff;
    reg                          qp_curr_sqptr_req_ff;
    reg                          qp_stat_resp_psn_req_ff;
    reg                          qp_stat_nak_req_ff;
    reg                          qp_stat_rq_buf_ca_rdreq_ff;
    reg                          qp_stat_wqe_cnt_rdreq_ff;
    reg                          qp_stat_msn_req_ff;
    reg                          qp_stat_ssn_req_ff;
    reg                          qp_retry_cnt_req_ff;
    reg                          qp_retry_cnt_req_2ff;
    reg                          qp_rq_wrptr_db_req_ff;
    reg                          qp_rq_wrptrdb_add_req_ff;
    reg                          qp_sq_cmpldb_add_req_ff;
    reg [                  15:0] sqe_backpressure_cnt_ff;
    reg [                  15:0] osq_backpressure_cnt_ff;
    reg [                  15:0] all_sqes_empty_cnt_ff;

    //For reading number of outgoing read resonses per second
    reg [                  31:0] cycle_count;
    reg [                  31:0] rd_resp_pkt_cnt_per_sec;
    reg [                  31:0] rd_req_pkt_cnt_per_sec;
    reg                          clear_rd_req_cnt_per_sec;
    reg                          clear_rd_resp_cnt_per_sec;

    genvar i, j;


    wire                          axi_lite_access_to_rqpidb_reg;
    wire [  C_QP_INDX_WIDTH -1:0] rq_cidb_idx;  // Index for reading RQ CI DB

    wire [C_QP_INDX_WIDTH -1 : 0] qp_conf_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_conf_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_conf_bram_addr;
    reg                           qp_conf_bram_wen_r;  //slow
    reg                           qp_conf_bram_wen_p;  //slow
    wire                          qp_conf_bram_wen;  //slow


    wire [C_QP_INDX_WIDTH -1 : 0] qp_adv_conf_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_adv_conf_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_adv_conf_bram_addr;
    reg                           qp_adv_conf_bram_wen_r;  //slow
    reg                           qp_adv_conf_bram_wen_p;  //slow
    wire                          qp_adv_conf_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_bram_addr;
    reg                           qp_rq_ba_bram_wen_r;  //slow
    reg                           qp_rq_ba_bram_wen_p;  //slow
    wire                          qp_rq_ba_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_msb_bram_addr;
    reg                           qp_rq_ba_msb_bram_wen_r;  //slow
    reg                           qp_rq_ba_msb_bram_wen_p;  //slow
    wire                          qp_rq_ba_msb_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_bram_addr;
    reg                           qp_sq_ba_bram_wen_r;  //slow
    reg                           qp_sq_ba_bram_wen_p;  //slow
    wire                          qp_sq_ba_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_msb_bram_addr;
    reg                           qp_sq_ba_msb_bram_wen_r;  //slow
    reg                           qp_sq_ba_msb_bram_wen_p;  //slow
    wire                          qp_sq_ba_msb_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_bram_addr;
    reg                           qp_cq_ba_bram_wen_r;  //slow
    reg                           qp_cq_ba_bram_wen_p;  //slow
    wire                          qp_cq_ba_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_msb_bram_addr;
    reg                           qp_cq_ba_msb_bram_wen_r;  //slow
    reg                           qp_cq_ba_msb_bram_wen_p;  //slow
    wire                          qp_cq_ba_msb_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_bram_addr;
    reg                           qp_rq_wrptrdb_add_bram_wen_r;  //slow
    reg                           qp_rq_wrptrdb_add_bram_wen_p;  //slow
    wire                          qp_rq_wrptrdb_add_bram_wen;  //slow
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_msb_bram_addr;
    reg                           qp_rq_wrptrdb_add_msb_bram_wen_r;  //slow
    reg                           qp_rq_wrptrdb_add_msb_bram_wen_p;  //slow
    wire                          qp_rq_wrptrdb_add_msb_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_cmpldb_add_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_cmpldb_add_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_cmpldb_add_bram_addr;
    reg                           qp_sq_cmpldb_add_bram_wen_r;  //slow
    reg                           qp_sq_cmpldb_add_bram_wen_p;  //slow
    wire                          qp_sq_cmpldb_add_bram_wen;  //slow
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_cmpldb_add_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_cmpldb_add_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_cmpldb_add_msb_bram_addr;
    reg                           qp_sq_cmpldb_add_msb_bram_wen_r;  //slow
    reg                           qp_sq_cmpldb_add_msb_bram_wen_p;  //slow
    wire                          qp_sq_cmpldb_add_msb_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_hdptr_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_hdptr_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_hdptr_bram_addr;
    wire                          qp_cq_hdptr_bram_wen;  //fast

    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_cidb_wr_addr;
    wire [                  31:0] qp_rq_cidb_wr_addr_hndshk;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_cidb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_cidb_bram_addr;
    reg                           qp_rq_cidb_bram_wen_r;  //slow
    reg                           qp_rq_cidb_bram_wen_p;  //slow
    wire                          qp_rq_cidb_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_hndshk_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_bram_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_readaddr;
    wire                          qp_sq_pidb_bram_wen;  //fast

    wire [C_QP_INDX_WIDTH -1 : 0] qp_q_depth_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_q_depth_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_q_depth_bram_addr;
    reg                           qp_q_depth_bram_wen_r;  //slow
    reg                           qp_q_depth_bram_wen_p;  //slow
    wire                          qp_q_depth_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_psn_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_psn_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_psn_bram_addr;
    wire                          qp_sq_psn_bram_wen;  //***

    wire [C_QP_INDX_WIDTH -1 : 0] qp_last_rq_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_last_rq_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_last_rq_bram_addr;
    reg                           qp_last_rq_bram_wen_r;  //slow
    reg                           qp_last_rq_bram_wen_p;  //slow
    wire                          qp_last_rq_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_pd_num_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_pd_num_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_pd_num_bram_addr;
    reg                           qp_pd_num_bram_wen_r;  //slow  
    reg                           qp_pd_num_bram_wen_p;  //slow  
    wire                          qp_pd_num_bram_wen;  //slow  

    wire [C_PD_INDX_WIDTH -1 : 0] pd_pd_num_wr_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_pd_num_rd_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_pd_num_bram_addr;
    reg                           pd_pd_num_bram_wen_r;  //slow  
    reg                           pd_pd_num_bram_wen_p;  //slow  
    wire                          pd_pd_num_bram_wen;  //slow  

    wire [C_PD_INDX_WIDTH -1 : 0] pd_virt_addr_0_wr_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_virt_addr_0_rd_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_virt_addr_0_bram_addr;
    reg                           pd_virt_addr_0_bram_wen_r;  //slow  
    reg                           pd_virt_addr_0_bram_wen_p;  //slow  
    wire                          pd_virt_addr_0_bram_wen;  //slow  

    wire [C_PD_INDX_WIDTH -1 : 0] pd_virt_addr_1_wr_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_virt_addr_1_rd_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_virt_addr_1_bram_addr;
    reg                           pd_virt_addr_1_bram_wen_r;  //slow  
    reg                           pd_virt_addr_1_bram_wen_p;  //slow  
    wire                          pd_virt_addr_1_bram_wen;  //slow  

    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_base_addr_0_wr_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_base_addr_0_rd_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_base_addr_0_bram_addr;
    reg                           pd_buf_base_addr_0_bram_wen_r;  //slow  
    reg                           pd_buf_base_addr_0_bram_wen_p;  //slow  
    wire                          pd_buf_base_addr_0_bram_wen;  //slow  

    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_base_addr_1_wr_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_base_addr_1_rd_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_base_addr_1_bram_addr;
    reg                           pd_buf_base_addr_1_bram_wen_r;  //slow  
    reg                           pd_buf_base_addr_1_bram_wen_p;  //slow  
    wire                          pd_buf_base_addr_1_bram_wen;  //slow  

    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_r_key_wr_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_r_key_rd_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_buf_r_key_bram_addr;
    reg                           pd_buf_r_key_bram_wen_r;  //slow  
    reg                           pd_buf_r_key_bram_wen_p;  //slow  
    wire                          pd_buf_r_key_bram_wen;  //slow  

    wire [C_PD_INDX_WIDTH -1 : 0] pd_w_r_buf_len_wr_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_w_r_buf_len_rd_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_w_r_buf_len_bram_addr;
    reg                           pd_w_r_buf_len_bram_wen_r;  //slow    
    reg                           pd_w_r_buf_len_bram_wen_p;  //slow    
    wire                          pd_w_r_buf_len_bram_wen;  //slow    

    wire [C_PD_INDX_WIDTH -1 : 0] pd_access_type_wr_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_access_type_rd_addr;
    wire [C_PD_INDX_WIDTH -1 : 0] pd_access_type_bram_addr;
    reg                           pd_access_type_bram_wen_r;  //slow    
    reg                           pd_access_type_bram_wen_p;  //slow    
    wire                          pd_access_type_bram_wen;  //slow    

    wire [C_QP_INDX_WIDTH -1 : 0] qp_dest_qpid_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_dest_qpid_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_dest_qpid_bram_addr;
    reg                           qp_dest_qpid_bram_wen_r;  //slow
    reg                           qp_dest_qpid_bram_wen_p;  //slow
    wire                          qp_dest_qpid_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_mac_remote_addrl_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_mac_remote_addrl_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_mac_remote_addrl_bram_addr;
    reg                           qp_mac_remote_addrl_bram_wen_r;  //slow
    reg                           qp_mac_remote_addrl_bram_wen_p;  //slow
    wire                          qp_mac_remote_addrl_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_mac_remote_addrm_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_mac_remote_addrm_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_mac_remote_addrm_bram_addr;
    reg                           qp_mac_remote_addrm_bram_wen_r;  //slow
    reg                           qp_mac_remote_addrm_bram_wen_p;  //slow
    wire                          qp_mac_remote_addrm_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr1_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr1_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr1_bram_addr;
    reg                           qp_ip_remote_addr1_bram_wen_r;  //slow
    reg                           qp_ip_remote_addr1_bram_wen_p;  //slow
    wire                          qp_ip_remote_addr1_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr2_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr2_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr2_bram_addr;
    reg                           qp_ip_remote_addr2_bram_wen_r;  //slow
    reg                           qp_ip_remote_addr2_bram_wen_p;  //slow
    wire                          qp_ip_remote_addr2_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr3_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr3_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr3_bram_addr;
    reg                           qp_ip_remote_addr3_bram_wen_r;  //slow
    reg                           qp_ip_remote_addr3_bram_wen_p;  //slow
    wire                          qp_ip_remote_addr3_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr4_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr4_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_ip_remote_addr4_bram_addr;
    reg                           qp_ip_remote_addr4_bram_wen_r;  //slow
    reg                           qp_ip_remote_addr4_bram_wen_p;  //slow
    wire                          qp_ip_remote_addr4_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_msn_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_msn_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_msn_bram_addr;
    reg                           qp_stat_msn_bram_wen_r;  //slow
    reg                           qp_stat_msn_bram_wen_p;  //slow
    wire                          qp_stat_msn_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_ssn_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_ssn_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_ssn_bram_addr;
    reg                           qp_stat_ssn_bram_wen_r;  //slow
    reg                           qp_stat_ssn_bram_wen_p;  //slow
    wire                          qp_stat_ssn_bram_wen;  //slow

    wire [  C_QP_INDX_WIDTH -1:0] stat_curr_sqptr_proc_addr;
    wire [  C_QP_INDX_WIDTH -1:0] stat_curr_sqptr_proc_wr_addr;
    wire [  C_QP_INDX_WIDTH -1:0] stat_curr_sqptr_proc_rd_addr;
    wire [  C_QP_INDX_WIDTH -1:0] stat_curr_sqptr_proc_bram_addr;
    wire                          axi_curr_sqptr_proc_read;
    wire                          stat_curr_sqptr_bram_wen;  //fast
    wire                          qp_stat_rq_pi_db_bram_wen;  //fast
    wire [                  15:0] curr_sqptr_proc;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_nak_bram_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_nak_bram_rd_addr;
    wire                          qp_stat_nak_bram_wen;  //slow

    wire [C_QP_INDX_WIDTH -1 : 0] qp_wqe_cnt_bram_addr;
    wire                          qp_wqe_cnt_bram_wen;  //fast
    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_rq_pi_db_bram_addr;

    wire [  C_QP_INDX_WIDTH -1:0] qp_depth_idx;  // Index for reading Q depth
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_sq_psn_idx;

    wire [                  31:0] s_axi_qp_config_rdata;
    wire [                  31:0] s_axi_qp_adv_config_rdata;
    wire [                  31:0] s_axi_rq_ba_rdata;
    wire [                  31:0] s_axi_sq_ba_rdata;
    wire [                  31:0] s_axi_cq_ba_rdata;
    wire [                  31:0] s_axi_rq_ba_msb_rdata;
    wire [                  31:0] s_axi_sq_ba_msb_rdata;
    wire [                  31:0] s_axi_cq_ba_msb_rdata;
    wire [                  31:0] s_axi_rq_wrptrdb_add_rdata;
    wire [                  31:0] s_axi_rq_wrptrdb_msb_add_rdata;
    wire [                  31:0] s_axi_sq_cmpldb_add_rdata;
    wire [                  31:0] s_axi_sq_cmpldb_msb_add_rdata;
    wire [                  31:0] s_axi_cq_hdptr_rdata;
    wire [                  31:0] s_axi_rq_cidb_rdata;
    wire [                  31:0] s_axi_sq_pidb_rdata;
    wire [                  31:0] s_axi_q_depth_rdata;
    wire [                  31:0] s_axi_sq_psn_rdata;
    wire [                  31:0] s_axi_last_rq_rdata;
    wire [                  31:0] s_axi_virt_addr_0_rdata;
    wire [                  31:0] s_axi_virt_addr_1_rdata;
    wire [                  31:0] s_axi_buf_base_addr_0_rdata;
    wire [                  31:0] s_axi_buf_base_addr_1_rdata;
    wire [                  31:0] s_axi_buf_r_key_rdata;
    wire [                  31:0] s_axi_w_r_buf_len_rdata;
    wire [                  31:0] s_axi_dest_qpid_rdata;
    wire [                  31:0] s_axi_timeout_rdata;
    wire [                  31:0] s_axi_mac_remote_addrl_rdata;
    wire [                  31:0] s_axi_mac_remote_addrm_rdata;
    wire [                  31:0] s_axi_ip_remote_addr1_rdata;
    wire [                  31:0] s_axi_ip_remote_addr2_rdata;
    wire [                  31:0] s_axi_ip_remote_addr3_rdata;
    wire [                  31:0] s_axi_ip_remote_addr4_rdata;
    wire [                  31:0] s_axi_curr_sqptr_proc_rdata;
    wire [                  31:0] s_axi_stat_resp_psn_rdata;
    wire [                  31:0] s_axi_stat_rq_buf_ca_rdata;
    wire [                  31:0] s_axi_stat_rq_buf_ca_msb_rdata;
    wire [                  31:0] s_axi_stat_wqe_cnt_rdata;
    wire [                  31:0] s_axi_stat_rq_pi_db_rdata;
    wire [                  31:0] s_axi_stat_ret_sq_psn_rdata;
    wire [                  31:0] s_axi_pd_number_rdata;
    wire [                  31:0] s_axi_stat_msn_rdata;
    wire [                  31:0] s_axi_stat_qpn_rdata;
    wire [                  31:0] s_axi_stat_ssn_rdata;
    wire [                  31:0] s_axi_access_type_rdata;
    wire [                  23:0] s_axi_pd_pd_num_rdata;
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_mac_remote_addrl_idx;  // Index for reading Q depth
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_mac_remote_addrm_idx;  // Index for reading Q depth
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_ip_remote_addr1_idx;  // Index for reading Q depth
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_ip_remote_addr2_idx;  // Index for reading Q depth
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_ip_remote_addr3_idx;  // Index for reading Q depth
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_ip_remote_addr4_idx;  // Index for reading Q depth


    wire                          generic_reg_rd_access;
    wire                          generic_reg_wr_access;  // 地址是否0x10开头(0x10_0000)？1-是，0-不是
    wire                          intr_clr;
    wire [                  15:0] sq_pidb;
    wire                          sq_empty;
    wire                          sq_full;
    wire [15:0] sqe_backpressure_cnt, osq_backpressure_cnt, all_sqes_empty_cnt;
    wire stop_sqe_backpressure_cnt_en, stop_osq_backpressure_cnt_en, stop_all_sqes_empty_cnt_en;

    wire [    C_QP_INDX_WIDTH-1:0] qp_stat_ret_sq_psn_wr_addr;
    wire [    C_QP_INDX_WIDTH-1:0] qp_stat_ret_sq_psn_rd_addr;

    wire [    C_QP_INDX_WIDTH-1:0] qp_stat_rq_pi_db_wr_addr;
    wire [    C_QP_INDX_WIDTH-1:0] qp_stat_rq_pi_db_rd_addr;
    wire [    C_QP_INDX_WIDTH-1:0] qp_timeout_wr_addr;
    wire [    C_QP_INDX_WIDTH-1:0] qp_timeout_rd_addr;
    wire [    C_QP_INDX_WIDTH-1:0] qp_timeout_bram_addr;
    reg                            qp_timeout_bram_wen_r;  //slow
    reg                            qp_timeout_bram_wen_p;  //slow
    wire                           qp_timeout_bram_wen;  //slow

    wire [    C_QP_INDX_WIDTH-1:0] qp_stat_ret_sq_psn_bram_addr;
    reg  [                    2:0] pending_upd_cs;
    // To_be_Removed
    reg  [                    2:0] pending_upd_ns;
    reg                            clear_pend_stat;
    // tmp_cmt: understand logic, minimize to 3
    reg  [                    2:0] msb_cnt;
    reg  [C_QP_INDX_WIDTH -3 -1:0] lsb_cnt;
    reg                            sq_pidb_bram_wen_ff;
    wire                           clear_pend_stat_masked;
    wire [                    7:0] pending_idx;
    reg                            check_sqpi_sqci_addr;
    wire [                    7:0] jump;

    wire                           dbg_sq_empty;
    wire                           dbg_sq_full;
    reg                            dbg_sq_empty_ff_valid;
    reg  [   C_QP_INDX_WIDTH -1:0] dbg_sq_empty_ff_id;
    reg                            dbg_sq_empty_ff_data;
    reg                            dbg_sq_full_ff_valid;
    reg  [   C_QP_INDX_WIDTH -1:0] dbg_sq_full_ff_id;
    reg                            dbg_sq_full_ff_data;
    // To_be_Removed

    reg                            curr_sqptr_enb_r;
    reg                            curr_sqptr_vld_r;
    reg                            curr_sqptr_typ_r;
    reg  [    C_QP_INDX_WIDTH-1:0] curr_sqptr_addr_r;
    reg  [                   15:0] curr_sqptr_data_r;
    wire                           curr_sqptr_vld_c;
    wire [    C_QP_INDX_WIDTH-1:0] curr_sqptr_addr_c;
    wire [                   15:0] curr_sqptr_data_c;
    reg                            curr_sqptr_rden_r;
    wire                           curr_sqptr_fifo_empty;
    wire                           curr_sqptr_fifo_full;
    wire [    C_QP_INDX_WIDTH-1:0] curr_sqptr_fifo_dout;
    wire [                    4:0] curr_sqptr_fifo_dcount;
    wire [                   15:0] curr_sqptr_flag_ptr;  // 正在被硬件消费的 SQ 当前条目（“读指针”）？

    reg                            sq_pidb_enb_r;
    reg                            sq_pidb_req_mask_r;
    reg                            sq_pidb_vld_r;
    reg                            sq_pidb_typ_r;
    reg  [    C_QP_INDX_WIDTH-1:0] sq_pidb_addr_r;
    reg  [                   15:0] sq_pidb_data_r;
    wire                           sq_pidb_vld_c;
    wire [    C_QP_INDX_WIDTH-1:0] sq_pidb_addr_c;
    wire [                   15:0] sq_pidb_data_c;
    reg                            sq_pidb_rden_r;
    wire                           sq_pidb_fifo_empty;
    wire                           sq_pidb_fifo_full;
    wire [    C_QP_INDX_WIDTH-1:0] sq_pidb_fifo_dout;
    wire [                    4:0] sq_pidb_fifo_dcount;
    wire [                   15:0] sq_pidb_flag_ptr;  // 软件通过 SQ 门铃写入的最新生产者指针（“写指针”）？


    reg                            flag_updt_cs;
    reg  [    C_QP_INDX_WIDTH-1:0] sq_flag_updt_addr;
    reg                            sq_flag_updt_vld;
    reg  [    C_QP_INDX_WIDTH-1:0] sq_flag_updt_idx;

    reg  [                   31:0] rq_intr_sts_clr_wr_en;
    reg  [                   10:0] rq_intr_sts_clr_wr_addr;
    reg  [                   31:0] cq_intr_sts_clr_wr_en;
    reg  [                   10:0] cq_intr_sts_clr_wr_addr;
    reg  [                   31:0] cnp_schd_sts_clr_wr_en;
    reg  [                   10:0] cnp_schd_sts_clr_wr_addr;
    wire                           qp_fatal_sts;
    reg                            qp_fatal_sts_ff;
    reg  [                    2:0] mux_select;
    reg  [                   31:0] s_axi_lite_reg_1;
    reg  [                   31:0] s_axi_lite_reg_2;
    reg  [                   31:0] s_axi_lite_reg_3;
    reg  [                   31:0] s_axi_lite_reg_4;
    reg  [                   31:0] s_axi_lite_reg_5;
    reg  [                   31:0] s_axi_lite_reg_6;
    reg  [                   31:0] s_axi_lite_reg_7;
    reg  [                   31:0] s_axi_lite_reg_8;
    reg  [                   31:0] s_axi_lite_reg_9;
    reg  [                   31:0] s_axi_lite_reg_1_r;
    reg  [                   31:0] s_axi_lite_reg_2_r;
    reg  [                   31:0] s_axi_lite_reg_3_r;
    reg  [                   31:0] s_axi_lite_reg_4_r;
    reg  [                   31:0] s_axi_lite_reg_5_r;
    reg  [                   31:0] s_axi_lite_reg_6_r;
    reg  [                   31:0] s_axi_lite_reg_7_r;
    reg  [                   31:0] s_axi_lite_reg_8_r;
    reg  [                   31:0] s_axi_lite_reg_9_r;
    wire [                   31:0] s_axi_lite_reg_1_c;
    wire [                   31:0] s_axi_lite_reg_2_c;
    wire [                   31:0] s_axi_lite_reg_3_c;
    wire [                   31:0] s_axi_lite_reg_4_c;
    wire [                   31:0] s_axi_lite_reg_5_c;
    wire [                   31:0] s_axi_lite_reg_6_c;
    wire [                   31:0] s_axi_lite_reg_7_c;
    wire [                   31:0] s_axi_lite_reg_8_c;
    wire [                   31:0] s_axi_lite_reg_9_c;
    reg  [                    8:0] read_mux;

    reg  [                   31:0] s_axi_stat_qpn_rdata_r;
    reg  [                   31:0] s_axi_curr_sqptr_proc_rdata_r;
    reg  [                   31:0] s_axi_stat_rq_pi_db_rdata_r;

    reg  [                   31:0] s_axi_stat_qpn_rdata_p;
    reg  [                   31:0] s_axi_curr_sqptr_proc_rdata_p;
    reg  [                   31:0] s_axi_stat_rq_pi_db_rdata_p;

    reg  [                   31:0] s_axi_stat_qpn_rdata_p1;
    reg  [                   31:0] s_axi_curr_sqptr_proc_rdata_p1;
    reg  [                   31:0] s_axi_stat_rq_pi_db_rdata_p1;

    reg  [                   31:0] s_axi_stat_qpn_rdata_p2;
    reg  [                   31:0] s_axi_curr_sqptr_proc_rdata_p2;
    reg  [                   31:0] s_axi_stat_rq_pi_db_rdata_p2;
    wire [                   20:0] rd_addr_c1;
    reg  [                   20:0] rd_addr_p1;
    reg  [                   20:0] rd_addr_p2;
    reg  [                   20:0] rd_addr_r1;
    reg  [                    2:0] rd_addr_wen_r;
    reg                            rd_req_p1;
    reg                            rd_req_p2;
    reg                            rd_req_p3;
    reg                            rd_req_r1;
    reg  [                    1:0] wr_addr_wen_r;
    reg  [                   20:0] wr_addr_r1;
    wire [   C_QP_INDX_WIDTH -1:0] wr_addr_mem_r;
    wire [   C_PD_INDX_WIDTH -1:0] wr_addr_mem_r1;
    wire [                   31:0] wr_data_mem_r;
    reg  [   C_QP_INDX_WIDTH -1:0] wr_addr_mem_r_r;
    reg  [   C_QP_INDX_WIDTH -1:0] rd_addr_mem_r_r;
    reg  [   C_PD_INDX_WIDTH -1:0] wr_addr_mem_r1_r;
    reg  [                   31:0] wr_data_mem_r_r;
    reg  [   C_QP_INDX_WIDTH -1:0] wr_addr_mem_r_p;
    reg  [   C_PD_INDX_WIDTH -1:0] wr_addr_mem_r1_p;
    reg  [                   31:0] wr_data_mem_r_p;
    reg                            wr_req_r1;
    reg  [                   31:0] wr_data_r1;
    reg                            osq_full_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_full_id;
    reg                            osq_full_data;
    reg                            osq_full_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_full_id_pending;
    wire                           osq_full_rd_data;
    wire                           osq_full_rd_data_int;
    wire                           i_rq_full_rd_data;
    wire                           fatal_err_rd_data;
    wire                           tx_path_congestion_rd_data;
    wire                           dbg_sq_empty_ff_rd_data;
    wire                           dbg_sq_full_ff_rd_data;
    reg                            osq_empty_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_empty_id;
    reg                            osq_empty_data;
    reg                            osq_empty_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_empty_id_pending;
    wire                           osq_empty_rd_data;
    wire                           osq_empty_rd_data_int;
    wire                           qp_retried_rd_data;
    reg  [    C_QP_INDX_WIDTH : 0] fatal_err_ones_count;
    wire                           fatal_id_rd_data;
    reg  [    C_QP_INDX_WIDTH : 0] expr1_num_ones;
    reg  [    C_QP_INDX_WIDTH : 0] expr2_num_ones;
    wire                           expr1_value;
    wire                           expr2_value;
    wire                           dbg_sq_empty_ff_rd_data_1;
    wire                           osq_almost_full_data_rd_data;
    wire [                   31:0] w_cnp_schedule_info;
    wire [                   31:0] w_qp_pkt_rcvd_intr;
    wire [                   31:0] w_qp_pkt_rcvd_intr_rd_data;
    wire [                   31:0] w_qp_wq_cmpl_intr;
    wire [                   31:0] w_qp_wq_cmpl_intr_int;
    reg                            i_qp_pkt_rcvd_intr_or_reduce;
    reg                            Any_ones_in_qp_pkt_rcvd_intr_lut;
    reg  [    C_QP_INDX_WIDTH : 0] dbg_sq_empty_ff_zeros_count;
    reg                            osq_almost_full_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_almost_full_id;
    reg                            osq_almost_full_data;
    reg                            osq_almost_full_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_almost_full_id_pending;
    reg                            s_axi_lite_wready_i;
    reg                            w_channel_hwshake_before_aw;
    reg                            aw_window_active;
    wire                           prog_full_sq_db_update;
    wire                           prog_full_db_update_curr_sqptr;


    // 将 QP 分成 8 个相等的部分。每块称为 QP_CHUNK
    // 这样做是为了在只有几个 QP 被激活的情况下更快地更新 sq_empty/full 信号
    localparam QP_CHUNK = (C_NUM_QP < 8) ? 1 : (C_NUM_QP / 8);  //C_NUM_QP/8;

    // 以下两个参数的 C_NUM_QP 比较值必须保持一致
    localparam C_CONFIG_MEM_LATENCY = (C_NUM_QP <= 256) ? 3 : 4;  // 3
    localparam CONFIG_MEM_LATENCY_H = C_CONFIG_MEM_LATENCY;  // 3
    localparam CONFIG_MEM_LATENCY_M = C_CONFIG_MEM_LATENCY - 1;  // 2
    localparam CONFIG_MEM_LATENCY_L = 1;
    localparam CONFIG_MEM_LATENCY_K = CONFIG_MEM_LATENCY_L + 1;

    localparam RDMA_OPC_SEND_ONLY = 8'h04;

    localparam IDLE = 3'b000;
    localparam FIND_PEND_UPD = 3'b001;
    localparam WAIT_BRAM = 3'b010;
    localparam INCR_MSB_CNT = 3'b011;
    localparam INCR_LSB_CNT = 3'b100;
    localparam UPDATE_PEND_STAT = 3'b101;

    localparam WAIT_FOR_UPDT = 1'b0;
    localparam READ_SQ_PTR = 1'b1;

    localparam PD_NUMBER_PDN = 8'h00;
    localparam VIRT_ADDR_0_PDN = 8'h04;
    localparam VIRT_ADDR_1_PDN = 8'h08;
    localparam BUF_BASE_ADDR_0_PDN = 8'h0C;
    localparam BUF_BASE_ADDR_1_PDN = 8'h10;
    localparam BUF_R_KEY_PDN = 8'h14;
    localparam W_R_BUF_LEN_PDN = 8'h18;
    localparam ACCESS_TYPE_PDN = 8'h1C;
    //Global reg offsets
    localparam XRNIC_CONF_REG = 21'h10_0000;
    localparam XRNIC_ADV_CONF_REG = 21'h10_0004;
    localparam XRNIC_BUF_THRESHOLD_ROCE_REG = 21'h10_0008;
    localparam XRNIC_BUF_THRESHOLD_NON_ROCE_REG = 21'h10_0018;
    localparam XRNIC_PAUSE_CONF_REG = 21'h10_000C;
    localparam MAC_XRNIC_ADDR_LSB_REG = 21'h10_0010;
    localparam MAC_XRNIC_ADDR_MSB_REG = 21'h10_0014;
    localparam IPV6_XRNIC_ADDR1_REG = 21'h10_0020;
    localparam IPV6_XRNIC_ADDR2_REG = 21'h10_0024;
    localparam IPV6_XRNIC_ADDR3_REG = 21'h10_0028;
    localparam IPV6_XRNIC_ADDR4_REG = 21'h10_002C;
    localparam TX_ERR_CORRUPT_CONF_REG = 21'h10_0030;
    localparam TX_IPG_CONF_REG = 21'h10_0038;
    localparam WQE_SPLIT_GRAN_REG = 21'h10_0040;
    localparam XRNIC_CONF_QP_EN_REG = 21'h10_0044;
    localparam TX_SGL_BUF_SZ_REG = 21'h10_0048;
    localparam BYPASS_BUF_BA_REG = 21'h10_0050;
    localparam BYPASS_BUF_SZ_REG = 21'h10_0058;
    localparam BYPASS_BUF_WRPTR_REG = 21'h10_005C;
    localparam RQ_ERR_PKT_BUF_BA_REG = 21'h10_0060;
    localparam RQ_ERR_PKT_BUF_BA_REG_MSB = 21'h10_0064;
    localparam RQ_ERR_PKT_BUF_SZ_REG = 21'h10_0068;
    localparam RQ_ERR_PKT_BUF_WRPTR_REG = 21'h10_006C;
    localparam IPV4_XRNIC_ADDR_REG = 21'h10_0070;
    localparam OUT_ERRSTS_Q_BA_REG = 21'h10_0078;
    localparam OUT_ERRSTS_Q_BA_REG_MSB = 21'h10_007C;
    localparam OUT_ERRSTS_Q_SZ_REG = 21'h10_0080;
    localparam OUT_ERRSTS_Q_WRPTR_REG = 21'h10_0084;
    localparam IN_ERRSTS_Q_BA_REG = 21'h10_0088;
    localparam IN_ERRSTS_Q_BA_REG_MSB = 21'h10_008C;
    localparam IN_ERRSTS_Q_SZ_REG = 21'h10_0090;
    localparam IN_ERRSTS_Q_WRPTR_REG = 21'h10_0094;
    localparam DATA_BUF_BA_REG = 21'h10_00A0;
    localparam DATA_BUF_BA_MSB_REG = 21'h10_00A4;
    localparam DATA_BUF_SZ_REG = 21'h10_00A8;
    localparam CNCT_IO_CONF_REG = 21'h10_00AC;
    localparam RESP_ERR_PKT_BUF_BA_REG = 21'h10_00B0;
    localparam RESP_ERR_PKT_BUF_BA_REG_MSB = 21'h10_00B4;
    localparam RESP_ERR_PKT_BUF_SZ_REG = 21'h10_00B8;
    localparam RESP_ERR_PKT_BUF_WRPTR_REG = 21'h10_00BC;
    localparam RX_PKT_HNDL_DBG_CTRL_REG = 21'h10_00CC;
    localparam INC_SR_PKT_CNT_REG = 21'h10_0100;
    localparam INC_AM_PKT_CNT_REG = 21'h10_0104;
    localparam OUT_SR_PKT_CNT_REG = 21'h10_0108;
    localparam OUT_AM_PKT_CNT_REG = 21'h10_010C;
    localparam LAST_IN_PKT_REG = 21'h10_0110;
    localparam LAST_OUT_PKT_REG = 21'h10_0114;
    localparam INV_DUP_PKT_CNT_REG = 21'h10_0118;
    localparam RNR_IN_PKT_STS_REG = 21'h10_011C;
    localparam RNR_OUT_PKT_STS_REG = 21'h10_0120;
    localparam WQE_PROC_STS_REG = 21'h10_0124;
    localparam RX_PKT_VLD_STS_REG = 21'h10_0128;
    localparam QP_MGR_STS_REG = 21'h10_012C;
    localparam INC_ALL_DROPPED_CNT_REG = 21'h10_0130;
    localparam INC_NACK_CNT_REG = 21'h10_0134;
    localparam OUT_NACK_CNT_REG = 21'h10_0138;
    localparam RESP_HNDL_STS_REG = 21'h10_013C;
    localparam RETRY_CNT_STS_REG = 21'h10_0140;
    localparam DBG_CNT_OVERRIDE_REG = 21'h10_0144;
    localparam IPG_0_4_PKT_CNT_REG = 21'h10_0148;
    localparam IPG_5_9_PKT_CNT_REG = 21'h10_014C;
    localparam IPG_10_14_PKT_CNT_REG = 21'h10_0150;
    localparam IPG_15_19_PKT_CNT_REG = 21'h10_0154;
    localparam GLOBAL_DBG_CNTR = 21'h10_0158;
    localparam OSQ_BKP_SQE_BKP_CNT = 21'h10_015C;
    localparam RETRY_TX_BKP_HDR_BKP_CNT = 21'h10_0160;
    localparam SQE_EMPTY_WQE_IDLE_CNT = 21'h10_0164;
    localparam WQE_PROC_SM_CNT0 = 21'h10_0168;
    localparam WQE_PROC_SM_CNT1 = 21'h10_016C;
    localparam WQE_PROC_SM_CNT2 = 21'h10_0170;
    localparam IN_CNP_PKT_CNT = 21'h10_0174;
    localparam OUT_CNP_PKT_CNT = 21'h10_0178;
    localparam OUT_RD_RESP_PKT_CNT = 21'h10_017C;
    localparam INTR_EN_REG = 21'h10_0180;
    localparam INTR_STS_REG = 21'h10_0184;
    //Assumption All RQ,CQ and CNP Interrupt offsets must be consecutive; otherwise LUT RAM read addressing will fail..
    localparam RQ_INTR_STS1_REG = 21'h10_0190;
    localparam RQ_INTR_STS2_REG = 21'h10_0194;
    localparam RQ_INTR_STS3_REG = 21'h10_0198;
    localparam RQ_INTR_STS4_REG = 21'h10_019C;
    localparam RQ_INTR_STS5_REG = 21'h10_01A0;
    localparam RQ_INTR_STS6_REG = 21'h10_01A4;
    localparam RQ_INTR_STS7_REG = 21'h10_01A8;
    localparam RQ_INTR_STS8_REG = 21'h10_01AC;

    localparam RQ_INTR_STS9_REG = 21'h10_01B0;
    localparam RQ_INTR_STS10_REG = 21'h10_01B4;
    localparam RQ_INTR_STS11_REG = 21'h10_01B8;
    localparam RQ_INTR_STS12_REG = 21'h10_01BC;
    localparam RQ_INTR_STS13_REG = 21'h10_01C0;
    localparam RQ_INTR_STS14_REG = 21'h10_01C4;
    localparam RQ_INTR_STS15_REG = 21'h10_01C8;
    localparam RQ_INTR_STS16_REG = 21'h10_01CC;
    localparam RQ_INTR_STS17_REG = 21'h10_01D0;
    localparam RQ_INTR_STS18_REG = 21'h10_01D4;
    localparam RQ_INTR_STS19_REG = 21'h10_01D8;
    localparam RQ_INTR_STS20_REG = 21'h10_01DC;
    localparam RQ_INTR_STS21_REG = 21'h10_01E0;
    localparam RQ_INTR_STS22_REG = 21'h10_01E4;
    localparam RQ_INTR_STS23_REG = 21'h10_01E8;
    localparam RQ_INTR_STS24_REG = 21'h10_01EC;
    localparam RQ_INTR_STS25_REG = 21'h10_01F0;
    localparam RQ_INTR_STS26_REG = 21'h10_01F4;
    localparam RQ_INTR_STS27_REG = 21'h10_01F8;
    localparam RQ_INTR_STS28_REG = 21'h10_01FC;
    localparam RQ_INTR_STS29_REG = 21'h10_0200;
    localparam RQ_INTR_STS30_REG = 21'h10_0204;
    localparam RQ_INTR_STS31_REG = 21'h10_0208;
    localparam RQ_INTR_STS32_REG = 21'h10_020C;
    localparam RQ_INTR_STS33_REG = 21'h10_0210;
    localparam RQ_INTR_STS34_REG = 21'h10_0214;
    localparam RQ_INTR_STS35_REG = 21'h10_0218;
    localparam RQ_INTR_STS36_REG = 21'h10_021C;
    localparam RQ_INTR_STS37_REG = 21'h10_0220;
    localparam RQ_INTR_STS38_REG = 21'h10_0224;
    localparam RQ_INTR_STS39_REG = 21'h10_0228;
    localparam RQ_INTR_STS40_REG = 21'h10_022C;
    localparam RQ_INTR_STS41_REG = 21'h10_0230;
    localparam RQ_INTR_STS42_REG = 21'h10_0234;
    localparam RQ_INTR_STS43_REG = 21'h10_0238;
    localparam RQ_INTR_STS44_REG = 21'h10_023C;
    localparam RQ_INTR_STS45_REG = 21'h10_0240;
    localparam RQ_INTR_STS46_REG = 21'h10_0244;
    localparam RQ_INTR_STS47_REG = 21'h10_0248;
    localparam RQ_INTR_STS48_REG = 21'h10_024C;
    localparam RQ_INTR_STS49_REG = 21'h10_0250;
    localparam RQ_INTR_STS50_REG = 21'h10_0254;
    localparam RQ_INTR_STS51_REG = 21'h10_0258;
    localparam RQ_INTR_STS52_REG = 21'h10_025C;
    localparam RQ_INTR_STS53_REG = 21'h10_0260;
    localparam RQ_INTR_STS54_REG = 21'h10_0264;
    localparam RQ_INTR_STS55_REG = 21'h10_0268;
    localparam RQ_INTR_STS56_REG = 21'h10_026C;
    localparam RQ_INTR_STS57_REG = 21'h10_0270;
    localparam RQ_INTR_STS58_REG = 21'h10_0274;
    localparam RQ_INTR_STS59_REG = 21'h10_0278;
    localparam RQ_INTR_STS60_REG = 21'h10_027C;
    localparam RQ_INTR_STS61_REG = 21'h10_0280;
    localparam RQ_INTR_STS62_REG = 21'h10_0284;
    localparam RQ_INTR_STS63_REG = 21'h10_0288;
    localparam RQ_INTR_STS64_REG = 21'h10_028C;


    localparam CQ_INTR_STS1_REG = 21'h10_0290;
    localparam CQ_INTR_STS2_REG = 21'h10_0294;
    localparam CQ_INTR_STS3_REG = 21'h10_0298;
    localparam CQ_INTR_STS4_REG = 21'h10_029C;
    localparam CQ_INTR_STS5_REG = 21'h10_02A0;
    localparam CQ_INTR_STS6_REG = 21'h10_02A4;
    localparam CQ_INTR_STS7_REG = 21'h10_02A8;
    localparam CQ_INTR_STS8_REG = 21'h10_02AC;
    localparam CQ_INTR_STS9_REG = 21'h10_02B0;
    localparam CQ_INTR_STS10_REG = 21'h10_02B4;
    localparam CQ_INTR_STS11_REG = 21'h10_02B8;
    localparam CQ_INTR_STS12_REG = 21'h10_02BC;
    localparam CQ_INTR_STS13_REG = 21'h10_02C0;
    localparam CQ_INTR_STS14_REG = 21'h10_02C4;
    localparam CQ_INTR_STS15_REG = 21'h10_02C8;
    localparam CQ_INTR_STS16_REG = 21'h10_02CC;
    localparam CQ_INTR_STS17_REG = 21'h10_02D0;
    localparam CQ_INTR_STS18_REG = 21'h10_02D4;
    localparam CQ_INTR_STS19_REG = 21'h10_02D8;
    localparam CQ_INTR_STS20_REG = 21'h10_02DC;
    localparam CQ_INTR_STS21_REG = 21'h10_02E0;
    localparam CQ_INTR_STS22_REG = 21'h10_02E4;
    localparam CQ_INTR_STS23_REG = 21'h10_02E8;
    localparam CQ_INTR_STS24_REG = 21'h10_02EC;
    localparam CQ_INTR_STS25_REG = 21'h10_02F0;
    localparam CQ_INTR_STS26_REG = 21'h10_02F4;
    localparam CQ_INTR_STS27_REG = 21'h10_02F8;
    localparam CQ_INTR_STS28_REG = 21'h10_02FC;
    localparam CQ_INTR_STS29_REG = 21'h10_0300;
    localparam CQ_INTR_STS30_REG = 21'h10_0304;
    localparam CQ_INTR_STS31_REG = 21'h10_0308;
    localparam CQ_INTR_STS32_REG = 21'h10_030C;
    localparam CQ_INTR_STS33_REG = 21'h10_0310;
    localparam CQ_INTR_STS34_REG = 21'h10_0314;
    localparam CQ_INTR_STS35_REG = 21'h10_0318;
    localparam CQ_INTR_STS36_REG = 21'h10_031C;
    localparam CQ_INTR_STS37_REG = 21'h10_0320;
    localparam CQ_INTR_STS38_REG = 21'h10_0324;
    localparam CQ_INTR_STS39_REG = 21'h10_0328;
    localparam CQ_INTR_STS40_REG = 21'h10_032C;
    localparam CQ_INTR_STS41_REG = 21'h10_0330;
    localparam CQ_INTR_STS42_REG = 21'h10_0334;
    localparam CQ_INTR_STS43_REG = 21'h10_0338;
    localparam CQ_INTR_STS44_REG = 21'h10_033C;
    localparam CQ_INTR_STS45_REG = 21'h10_0340;
    localparam CQ_INTR_STS46_REG = 21'h10_0344;
    localparam CQ_INTR_STS47_REG = 21'h10_0348;
    localparam CQ_INTR_STS48_REG = 21'h10_034C;
    localparam CQ_INTR_STS49_REG = 21'h10_0350;
    localparam CQ_INTR_STS50_REG = 21'h10_0354;
    localparam CQ_INTR_STS51_REG = 21'h10_0358;
    localparam CQ_INTR_STS52_REG = 21'h10_035C;
    localparam CQ_INTR_STS53_REG = 21'h10_0360;
    localparam CQ_INTR_STS54_REG = 21'h10_0364;
    localparam CQ_INTR_STS55_REG = 21'h10_0368;
    localparam CQ_INTR_STS56_REG = 21'h10_036C;
    localparam CQ_INTR_STS57_REG = 21'h10_0370;
    localparam CQ_INTR_STS58_REG = 21'h10_0374;
    localparam CQ_INTR_STS59_REG = 21'h10_0378;
    localparam CQ_INTR_STS60_REG = 21'h10_037C;
    localparam CQ_INTR_STS61_REG = 21'h10_0380;
    localparam CQ_INTR_STS62_REG = 21'h10_0384;
    localparam CQ_INTR_STS63_REG = 21'h10_0388;
    localparam CQ_INTR_STS64_REG = 21'h10_038C;



    localparam CNP_SCHD_STS1_REG = 21'h10_0390;
    localparam CNP_SCHD_STS2_REG = 21'h10_0394;
    localparam CNP_SCHD_STS3_REG = 21'h10_0398;
    localparam CNP_SCHD_STS4_REG = 21'h10_039C;
    localparam CNP_SCHD_STS5_REG = 21'h10_03A0;
    localparam CNP_SCHD_STS6_REG = 21'h10_03A4;
    localparam CNP_SCHD_STS7_REG = 21'h10_03A8;
    localparam CNP_SCHD_STS8_REG = 21'h10_03AC;
    localparam CNP_SCHD_STS9_REG = 21'h10_03B0;
    localparam CNP_SCHD_STS10_REG = 21'h10_03B4;
    localparam CNP_SCHD_STS11_REG = 21'h10_03B8;
    localparam CNP_SCHD_STS12_REG = 21'h10_03BC;
    localparam CNP_SCHD_STS13_REG = 21'h10_03C0;
    localparam CNP_SCHD_STS14_REG = 21'h10_03C4;
    localparam CNP_SCHD_STS15_REG = 21'h10_03C8;
    localparam CNP_SCHD_STS16_REG = 21'h10_03CC;
    localparam CNP_SCHD_STS17_REG = 21'h10_03D0;
    localparam CNP_SCHD_STS18_REG = 21'h10_03D4;
    localparam CNP_SCHD_STS19_REG = 21'h10_03D8;
    localparam CNP_SCHD_STS20_REG = 21'h10_03DC;
    localparam CNP_SCHD_STS21_REG = 21'h10_03E0;
    localparam CNP_SCHD_STS22_REG = 21'h10_03E4;
    localparam CNP_SCHD_STS23_REG = 21'h10_03E8;
    localparam CNP_SCHD_STS24_REG = 21'h10_03EC;
    localparam CNP_SCHD_STS25_REG = 21'h10_03F0;
    localparam CNP_SCHD_STS26_REG = 21'h10_03F4;
    localparam CNP_SCHD_STS27_REG = 21'h10_03F8;
    localparam CNP_SCHD_STS28_REG = 21'h10_03FC;
    localparam CNP_SCHD_STS29_REG = 21'h10_0400;
    localparam CNP_SCHD_STS30_REG = 21'h10_0404;
    localparam CNP_SCHD_STS31_REG = 21'h10_0408;
    localparam CNP_SCHD_STS32_REG = 21'h10_040C;
    localparam CNP_SCHD_STS33_REG = 21'h10_0410;
    localparam CNP_SCHD_STS34_REG = 21'h10_0414;
    localparam CNP_SCHD_STS35_REG = 21'h10_0418;
    localparam CNP_SCHD_STS36_REG = 21'h10_041C;
    localparam CNP_SCHD_STS37_REG = 21'h10_0420;
    localparam CNP_SCHD_STS38_REG = 21'h10_0424;
    localparam CNP_SCHD_STS39_REG = 21'h10_0428;
    localparam CNP_SCHD_STS40_REG = 21'h10_042C;
    localparam CNP_SCHD_STS41_REG = 21'h10_0430;
    localparam CNP_SCHD_STS42_REG = 21'h10_0434;
    localparam CNP_SCHD_STS43_REG = 21'h10_0438;
    localparam CNP_SCHD_STS44_REG = 21'h10_043C;
    localparam CNP_SCHD_STS45_REG = 21'h10_0440;
    localparam CNP_SCHD_STS46_REG = 21'h10_0444;
    localparam CNP_SCHD_STS47_REG = 21'h10_0448;
    localparam CNP_SCHD_STS48_REG = 21'h10_044C;
    localparam CNP_SCHD_STS49_REG = 21'h10_0450;
    localparam CNP_SCHD_STS50_REG = 21'h10_0454;
    localparam CNP_SCHD_STS51_REG = 21'h10_0458;
    localparam CNP_SCHD_STS52_REG = 21'h10_045C;
    localparam CNP_SCHD_STS53_REG = 21'h10_0460;
    localparam CNP_SCHD_STS54_REG = 21'h10_0464;
    localparam CNP_SCHD_STS55_REG = 21'h10_0468;
    localparam CNP_SCHD_STS56_REG = 21'h10_046C;
    localparam CNP_SCHD_STS57_REG = 21'h10_0470;
    localparam CNP_SCHD_STS58_REG = 21'h10_0474;
    localparam CNP_SCHD_STS59_REG = 21'h10_0478;
    localparam CNP_SCHD_STS60_REG = 21'h10_047C;
    localparam CNP_SCHD_STS61_REG = 21'h10_0480;
    localparam CNP_SCHD_STS62_REG = 21'h10_0484;
    localparam CNP_SCHD_STS63_REG = 21'h10_0488;
    localparam CNP_SCHD_STS64_REG = 21'h10_048C;

    localparam CYCLE_CNT = 21'h10_0490;
    localparam RD_RSP_CNT_PER_SEC = 21'h10_0494;
    localparam RD_REQ_CNT_PER_SEC = 21'h10_0498;
    localparam RD_REQ_CNT = 21'h10_049C;
    // Per Qp Rsgisters Offsets
    localparam QP_CONF_QPN = 8'h00;
    localparam QP_ADV_CONF_QPN = 8'h04;
    localparam RQ_BUF_BA_QPN = 8'h08;
    localparam SQ_BA_QPN = 8'h10;
    localparam CQ_BA_QPN = 8'h18;
    localparam RQ_WRPTR_DB_ADD_QPN = 8'h20;
    localparam RQ_WRPTR_DB_ADD_QPN_MSB = 8'h24;
    localparam SQ_CMPL_DB_ADD_QPN = 8'h28;
    localparam SQ_CMPL_DB_ADD_QPN_MSB = 8'h2C;
    localparam CQ_HEAD_QPN = 8'h30;
    localparam RQ_CI_DB_QPN = 8'h34;
    localparam SQ_PI_DB_QPN = 8'h38;
    localparam Q_DEPTH_QPN = 8'h3C;
    localparam SQ_PSN_QPN = 8'h40;
    localparam LAST_RQ_QPN = 8'h44;
    localparam DEST_QP_CONF_QPN = 8'h48;
    localparam TIMEOUT_QPN = 8'h4C;
    localparam MAC_REMOTE_ADDR_LSB_QPN = 8'h50;
    localparam MAC_REMOTE_ADDR_MSB_QPN = 8'h54;
    localparam IP_REMOTE_ADDR1_QPN = 8'h60;
    localparam IP_REMOTE_ADDR2_QPN = 8'h64;
    localparam IP_REMOTE_ADDR3_QPN = 8'h68;
    localparam IP_REMOTE_ADDR4_QPN = 8'h6C;
    localparam STAT_SSN_QPN = 8'h80;
    localparam STAT_MSN_QPN = 8'h84;
    localparam STAT_QPN = 8'h88;
    localparam STAT_CURR_SQPTR_PROC_QPN = 8'h8C;
    localparam STAT_RESP_PSN_QPN = 8'h90;
    localparam STAT_RQ_BUF_CA = 8'h94;
    localparam STAT_WQE_CNT = 8'h98;
    localparam STAT_RQ_PI_DB = 8'h9C;
    localparam STAT_RET_SQ_PSN = 8'hA0;
    localparam PD_NUMBER_QPN = 8'hB0;
    localparam RQ_BUF_BA_MSB_QPN = 8'hC0;
    localparam SQ_BA_MSB_QPN = 8'hC8;
    localparam CQ_BA_MSB_QPN = 8'hD0;
    localparam STAT_RQ_BUF_CA_MSB = 8'hD8;

    localparam LOG_NUM_QP = clog2(C_NUM_QP);
    localparam LOG_NUM_PD = clog2(C_NUM_PD);

    //******************************************************************************
    // 只有在没有待处理的读取或写入事务时，才会接受写地址阶段。
    // 当读取和写入事务发生在同一时钟上时，读取事务将获得最高优先级并优先处理。
    //******************************************************************************
    //assign s_axi_lite_awready = ((~wr_req_r) && (~wr_req_mask_r) && (!(rd_req_r || s_axi_lite_arvalid))) && reset_released_r;
    assign s_axi_lite_awready = (~wr_req_r) && reset_released_r;
    assign s_axi_lite_bresp   = 2'b00;
    assign s_axi_lite_rresp   = 2'b00;
    //assign s_axi_lite_wready  = wr_req_r && ~s_axi_lite_bvalid;
    //assign s_axi_lite_wready  = wr_req_r && ~s_axi_lite_bvalid && ~(i_qp_sq_pidb_wr_valid_hndshk && sq_pidb_req_mask_r);
    //assign s_axi_lite_arready = ~rd_req_r && ~wr_req_r && reset_released_r;
    assign s_axi_lite_wready  = s_axi_lite_wready_i;
    assign s_axi_lite_arready = ~rd_req_r && reset_released_r;


    //******************************************************************************
    //According to xilinx guide lines after reset the AWREADY and ARREADY siganls
    //should be low atleast for one clock cycle. To achieve this a signal 
    //reset_released is taken and anded with s_axi_lite_awready and s_axi_lite_arready signals,
    //so that the output will show a logic '0' when in reset
    //******************************************************************************
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            reset_released_r            <= 1'b0;
            s_axi_lite_wready_i         <= 1'b0;
            w_channel_hwshake_before_aw <= 1'b0;
            aw_window_active            <= 1'b0;
        end else begin
            reset_released_r <= 1'b1;
            if ((s_axi_lite_wready && s_axi_lite_wvalid) || (s_axi_lite_awvalid && s_axi_lite_awready)) begin
                s_axi_lite_wready_i <= 1'b0;
            end else if (!wr_req_mask_r && wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && (!(rd_req_r || s_axi_lite_arvalid)) && !s_axi_lite_bvalid) begin
                s_axi_lite_wready_i <= 1'b1;
            end

            if ((s_axi_lite_wready && s_axi_lite_wvalid) && !((s_axi_lite_awvalid && s_axi_lite_awready) || aw_window_active)) begin
                w_channel_hwshake_before_aw <= 1'b1;  // in case w channel data comes first; need to wait for aw channel
            end else if (!wr_req_mask_r && wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && (!(rd_req_r || s_axi_lite_arvalid)) && !s_axi_lite_bvalid) begin
                w_channel_hwshake_before_aw <= 1'b0;
            end

            if (s_axi_lite_awvalid && s_axi_lite_awready) begin
                aw_window_active <= 1'b1;
            end else if (!wr_req_r) begin
                aw_window_active <= 1'b0;
            end
        end
    end

    //******************************************************************************
    //AXI Lite trasaction decoding and address latching logic. 
    //when s_axi_lite_a*valid signal is asserted by the master the address is latched 
    //and wr_req_r or rd_req_r signal is asserted until data phase is completed 
    //******************************************************************************


    always @(posedge core_clk) begin
        if (~core_rstn) begin
            wr_req_r      <= 1'b0;
            wr_req_mask_r <= 1'b0;
            rd_req_r      <= 1'b0;
            wr_addr_r     <= 'd0;
            rd_addr_r     <= 'd0;
        end else begin
            if (s_axi_lite_awvalid && s_axi_lite_awready) begin
                wr_req_r  <= 1'b1;
                wr_addr_r <= s_axi_lite_awaddr[20:0];
                //end else if (s_axi_lite_bvalid && s_axi_lite_bready) begin
            end else if (!wr_req_mask_r && wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && (!(rd_req_r || s_axi_lite_arvalid)) && !s_axi_lite_bvalid) begin
                wr_req_r  <= 1'b0;
                wr_addr_r <= 'd0;
            end else begin
                wr_req_r  <= wr_req_r;
                wr_addr_r <= wr_addr_r;
            end

            if (!(i_qp_sq_pidb_wr_valid_hndshk && sq_pidb_req_mask_r) && qp_sq_pidb_bram_wen && !prog_full_sq_db_update) begin
                wr_req_mask_r <= 1'b0;
            end else if (!i_qp_curr_sqptr_proc_wen && stat_curr_sqptr_bram_wen && !prog_full_db_update_curr_sqptr) begin
                wr_req_mask_r <= 1'b0;
                // 0x18_008C: Status Current SQ ptr under process
            end else if (s_axi_lite_awvalid && s_axi_lite_awready && (s_axi_lite_awaddr[7:0] == STAT_CURR_SQPTR_PROC_QPN && s_axi_lite_awaddr[20:19] == 2'b11)) begin
                wr_req_mask_r <= 1'b1;
                // 0x18_0038: SQ Producer index QPi
            end else if (s_axi_lite_awvalid && s_axi_lite_awready && (s_axi_lite_awaddr[7:0] == SQ_PI_DB_QPN && s_axi_lite_awaddr[20:19] == 2'b11)) begin
                wr_req_mask_r <= 1'b1;
            end

            if (s_axi_lite_arvalid && s_axi_lite_arready) begin
                rd_req_r  <= 1'b1;
                rd_addr_r <= s_axi_lite_araddr[20:0];
            end else if (s_axi_lite_rvalid && s_axi_lite_rready) begin
                rd_req_r  <= 1'b0;
                rd_addr_r <= rd_addr_r;
            end else begin
                rd_req_r  <= rd_req_r;
                rd_addr_r <= rd_addr_r;
            end
        end
    end


    generate
        if (C_NUM_QP > 32) begin : Intr_sts_handling  // C_NUM_QP > 32
            reg     [(clog2(C_NUM_QP/32)) : 0] num_ones_in_qp_pkt_rcvd_intr_lut[0 : 31];
            integer                            k;
            //wire [11:0] rq_intr_offset;
            //wire [11:0] cq_intr_offset;
            //wire [11:0] cnp_intr_offset;

            //assign rq_intr_offset = 12'h190;
            //assign cq_intr_offset = 12'h290;
            //assign cnp_intr_offset = 12'h390;

            for (i = 0; i <= 31; i = i + 1) begin
                sdp_lut_ram_wrapper #(
                    .C_RAM_WORD_WIDTH(1),
                    .C_RAM_DEPTH(C_NUM_QP / 32)
                ) inst_i_cnp_schedule_info (
                    .core_clk(core_clk),
                    .core_rst(!core_rstn),
                    .wr_en   (init_start || i_cnp_schedule_info_wr_en[i]),
                    .wr_addr (init_start ? init_count[C_QP_INDX_WIDTH-1 : 5] : i_cnp_schedule_info_wr_addr[C_QP_INDX_WIDTH-1 : 5]),
                    .wr_data (init_start ? 1'b0 : i_cnp_schedule_info_wr_data[i]),
                    .rd_en   (1'b1),
                    .rd_addr (rd_addr_c1[C_QP_INDX_WIDTH-4 : 2] - CNP_SCHD_STS1_REG[C_QP_INDX_WIDTH-4:2]),
                    .rd_data (w_cnp_schedule_info[i])
                );

                sdp_lut_ram_wrapper #(
                    .C_RAM_WORD_WIDTH(1),
                    .C_RAM_DEPTH(C_NUM_QP / 32)
                ) inst_i_qp_wq_cmpl_intr (
                    .core_clk(core_clk),
                    .core_rst(!core_rstn),
                    .wr_en   (init_start || i_qp_wq_cmpl_intr_wr_en[i]),
                    .wr_addr (init_start ? init_count[C_QP_INDX_WIDTH-1 : 5] : i_qp_wq_cmpl_intr_wr_addr[C_QP_INDX_WIDTH-1 : 5]),
                    .wr_data (init_start ? 1'b0 : i_qp_wq_cmpl_intr_wr_data[i]),
                    .rd_en   (1'b1),
                    .rd_addr (rd_addr_c1[C_QP_INDX_WIDTH-4 : 2] - CQ_INTR_STS1_REG[C_QP_INDX_WIDTH-4:2]),
                    .rd_data (w_qp_wq_cmpl_intr_int[i])
                );
                assign w_qp_wq_cmpl_intr[i] = (i_qp_wq_cmpl_intr_wr_en_pending[i] && i_qp_wq_cmpl_intr_wr_addr_pending[C_QP_INDX_WIDTH-1:5] == rd_addr_c1[C_QP_INDX_WIDTH-4 : 2] - CQ_INTR_STS1_REG[C_QP_INDX_WIDTH-4:2]) ? 1'b1 : w_qp_wq_cmpl_intr_int[i];

                sdp_lut_ram_wrapper #(
                    .C_RAM_WORD_WIDTH(1),
                    .C_RAM_DEPTH(C_NUM_QP / 32)
                ) inst_i_qp_pkt_rcvd_intr_0 (
                    .core_clk(core_clk),
                    .core_rst(!core_rstn),
                    .wr_en   (init_start || i_qp_pkt_rcvd_intr_wr_en[i]),
                    .wr_addr (init_start ? init_count[C_QP_INDX_WIDTH-1 : 5] : i_qp_pkt_rcvd_intr_wr_addr[C_QP_INDX_WIDTH-1 : 5]),
                    .wr_data (init_start ? 1'b0 : i_qp_pkt_rcvd_intr_wr_data[i]),
                    .rd_en   (1'b1),
                    .rd_addr (rd_addr_c1[C_QP_INDX_WIDTH-4 : 2] - RQ_INTR_STS1_REG[C_QP_INDX_WIDTH-4:2]),
                    .rd_data (w_qp_pkt_rcvd_intr[i])
                );

                sdp_lut_ram_wrapper #(
                    .C_RAM_WORD_WIDTH(1),
                    .C_RAM_DEPTH(C_NUM_QP / 32),
                    .C_COMBO_BYPASS(1)
                ) inst_i_qp_pkt_rcvd_intr_1 (
                    .core_clk(core_clk),
                    .core_rst(!core_rstn),
                    .wr_en   (init_start || i_qp_pkt_rcvd_intr_wr_en[i]),
                    .wr_addr (init_start ? init_count[C_QP_INDX_WIDTH-1 : 5] : i_qp_pkt_rcvd_intr_wr_addr[C_QP_INDX_WIDTH-1 : 5]),
                    .wr_data (init_start ? 1'b0 : i_qp_pkt_rcvd_intr_wr_data[i]),
                    .rd_en   (1'b1),
                    .rd_addr ((i_qp_pkt_rcvd_intr_wr_addr[C_QP_INDX_WIDTH-1 : 5])),
                    .rd_data (w_qp_pkt_rcvd_intr_rd_data[i])
                );


                always @(posedge core_clk) begin
                    if (!core_rstn) begin
                        num_ones_in_qp_pkt_rcvd_intr_lut[i] <= 'd0;
                    end else begin
                        case ({
                            i_qp_pkt_rcvd_intr_wr_en[i], i_qp_pkt_rcvd_intr_wr_data[i], w_qp_pkt_rcvd_intr_rd_data[i]
                        })
                            3'b110:  num_ones_in_qp_pkt_rcvd_intr_lut[i] <= num_ones_in_qp_pkt_rcvd_intr_lut[i] + 1;
                            3'b101:  num_ones_in_qp_pkt_rcvd_intr_lut[i] <= num_ones_in_qp_pkt_rcvd_intr_lut[i] - 1;
                            default: num_ones_in_qp_pkt_rcvd_intr_lut[i] <= num_ones_in_qp_pkt_rcvd_intr_lut[i];
                        endcase
                    end
                end
            end  // loop

            always @(*) begin
                Any_ones_in_qp_pkt_rcvd_intr_lut = 0;
                for (k = 0; k <= 31; k = k + 1) begin
                    Any_ones_in_qp_pkt_rcvd_intr_lut = Any_ones_in_qp_pkt_rcvd_intr_lut || (|num_ones_in_qp_pkt_rcvd_intr_lut[k]);
                end
            end
        end // if (C_NUM_QP > 32)

        else begin : Intr_sts_handling_less_than_32 // C_NUM_QP <= 32
            reg     [31:0] w_qp_pkt_rcvd_intr_r;
            reg     [31:0] w_cnp_schedule_info_r;
            reg     [31:0] w_qp_wq_cmpl_intr_r;
            integer        k;

            assign w_cnp_schedule_info = w_cnp_schedule_info_r;
            assign w_qp_pkt_rcvd_intr  = w_qp_pkt_rcvd_intr_r;
            always @(posedge core_clk) begin
                if (~core_rstn) begin
                    w_qp_pkt_rcvd_intr_r  <= 32'd0;
                    w_cnp_schedule_info_r <= 32'd0;
                    w_qp_wq_cmpl_intr_r   <= 32'd0;
                end else begin
                    for (k = 0; k < 32; k = k + 1) begin
                        if (i_qp_pkt_rcvd_intr_wr_en[k]) begin
                            w_qp_pkt_rcvd_intr_r[k] <= i_qp_pkt_rcvd_intr_wr_data[k];
                        end
                        if (i_cnp_schedule_info_wr_en[k]) begin
                            w_cnp_schedule_info_r[k] <= i_cnp_schedule_info_wr_data[k];
                        end
                        if (i_qp_wq_cmpl_intr_wr_en[k]) begin
                            w_qp_wq_cmpl_intr_r[k] <= i_qp_wq_cmpl_intr_wr_data[k];
                        end
                    end
                end
            end
            assign w_qp_wq_cmpl_intr_int = w_qp_wq_cmpl_intr_r;
            for (i = 0; i <= 31; i = i + 1) begin
                assign w_qp_wq_cmpl_intr[i] = (i_qp_wq_cmpl_intr_wr_en_pending[i]) ? 1'b1 : w_qp_wq_cmpl_intr_int[i];
            end

            reg [31 : 0] num_ones_in_qp_pkt_rcvd_intr_lut;
            reg [31 : 0] w_qp_pkt_rcvd_intr_data_reg;

            for (i = 0; i <= 31; i = i + 1) begin
                always @(posedge core_clk) begin
                    if (!core_rstn) begin
                        num_ones_in_qp_pkt_rcvd_intr_lut[i] <= 'd0;
                        w_qp_pkt_rcvd_intr_data_reg[i]      <= 'd0;
                    end else begin
                        if (i_qp_pkt_rcvd_intr_wr_en[i]) begin
                            w_qp_pkt_rcvd_intr_data_reg[i] <= i_qp_pkt_rcvd_intr_wr_data[i];
                        end

                        case ({
                            i_qp_pkt_rcvd_intr_wr_en[i], i_qp_pkt_rcvd_intr_wr_data[i], w_qp_pkt_rcvd_intr_data_reg[i]
                        })
                            3'b110:  num_ones_in_qp_pkt_rcvd_intr_lut[i] <= num_ones_in_qp_pkt_rcvd_intr_lut[i] + 1;
                            3'b101:  num_ones_in_qp_pkt_rcvd_intr_lut[i] <= num_ones_in_qp_pkt_rcvd_intr_lut[i] - 1;
                            default: num_ones_in_qp_pkt_rcvd_intr_lut[i] <= num_ones_in_qp_pkt_rcvd_intr_lut[i];
                        endcase
                    end
                end
            end

            always @(*) begin
                Any_ones_in_qp_pkt_rcvd_intr_lut = |num_ones_in_qp_pkt_rcvd_intr_lut;
            end
        end  // C_NUM_QP <= 32
    endgenerate

    `MSFF_R(i_qp_pkt_rcvd_intr_or_reduce, Any_ones_in_qp_pkt_rcvd_intr_lut, core_clk, !core_rstn)
    //******************************************************************************
    //AXI Lite read trasaction processing logic. 
    //when a read transaction is received, depending on address bits [13:2] the
    //data is recovered and sent on to s_axi_lite_rdata signal along with s_axi_lite_rvalid.  
    //The address bits [1:0] are not considred and it is expected that the
    //address is word aligned and reads complete word information. 
    //******************************************************************************

    //assign generic_reg_rd_access = (rd_addr_r[C_AXIS_DATA_WIDTH-1:9] == 'b0);
    assign generic_reg_rd_access = (rd_addr_r[20:19] == 2'b10);

    assign rd_addr_c1            = (C_CONFIG_MEM_LATENCY == 3) ? rd_addr_p1 : rd_addr_p2;

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            mux_select                     <= 3'b000;
            s_axi_lite_reg_1               <= 'd0;
            s_axi_lite_reg_2               <= 'd0;
            s_axi_lite_reg_3               <= 'd0;
            s_axi_lite_reg_4               <= 'd0;
            s_axi_lite_reg_5               <= 'd0;
            s_axi_lite_reg_6               <= 'd0;
            s_axi_lite_reg_7               <= 'd0;
            s_axi_lite_reg_8               <= 'd0;
            s_axi_lite_reg_9               <= 'd0;
            s_axi_lite_reg_1_r             <= 'd0;
            s_axi_lite_reg_2_r             <= 'd0;
            s_axi_lite_reg_3_r             <= 'd0;
            s_axi_lite_reg_4_r             <= 'd0;
            s_axi_lite_reg_5_r             <= 'd0;
            s_axi_lite_reg_6_r             <= 'd0;
            s_axi_lite_reg_7_r             <= 'd0;
            s_axi_lite_reg_8_r             <= 'd0;
            s_axi_lite_reg_9_r             <= 'd0;
            read_mux                       <= 9'b0_0000_0000;

            s_axi_stat_qpn_rdata_p         <= 'd0;
            s_axi_curr_sqptr_proc_rdata_p  <= 'd0;
            s_axi_stat_rq_pi_db_rdata_p    <= 'd0;

            s_axi_stat_qpn_rdata_p2        <= 'd0;
            s_axi_curr_sqptr_proc_rdata_p2 <= 'd0;
            s_axi_stat_rq_pi_db_rdata_p2   <= 'd0;

            s_axi_stat_qpn_rdata_r         <= 'd0;
            s_axi_curr_sqptr_proc_rdata_r  <= 'd0;
            s_axi_stat_rq_pi_db_rdata_r    <= 'd0;
            rd_addr_p1                     <= 'd0;
            rd_addr_p2                     <= 'd0;
            rd_addr_r1                     <= 'd0;
            rd_req_r1                      <= 1'b0;
            rd_req_p1                      <= 1'b0;
            rd_req_p2                      <= 1'b0;
            rd_req_p3                      <= 1'b0;
        end else begin

            rd_addr_p1 <= rd_addr_r;
            rd_addr_p2 <= rd_addr_p1;
            if (C_CONFIG_MEM_LATENCY == 3) begin
                rd_addr_r1 <= rd_addr_p1;
            end else begin
                rd_addr_r1 <= rd_addr_p2;
            end

            //if (rd_addr_c1[17:9]=='b100000000) begin
            //  mux_select <= 3'b001;
            //end else if (rd_addr_c1[16:14]!=3'b111 & rd_addr_c1[17] == 1 & rd_addr_c1[16:9] != 0) begin
            //  mux_select <= 3'b010;
            //end else if (rd_addr_c1[17] == 0) begin
            //  mux_select <= 3'b011;      
            //end else begin
            //  mux_select <= 3'b000;
            //end

            if (rd_addr_c1[20:19] == 2'b10) begin  //global registers
                mux_select <= 3'b001;
            end else if (rd_addr_c1[20:19] == 2'b11) begin  // per QP registers
                mux_select <= 3'b010;
                //end else if (rd_addr_c1[20:19] == 2'b00) begin //PD registers
                //  mux_select <= 3'b011;   
                //Including extended PD range	  
            end else begin
                mux_select <= 3'b011;
            end

            case (rd_addr_c1[10:2])
                XRNIC_CONF_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {8'h00, xrnic_conf_udp_src_port, xrnic_conf_flow_credits, xrnic_conf_err_buf_en, xrnic_conf_tx_ack_gen, xrnic_conf_depkt_bypass_en, xrnic_conf_ipver, xrnic_conf_xrnic_en};
                    read_mux[0]      <= 1'b1;
                end
                XRNIC_ADV_CONF_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {{1'b0, {(11 - C_QP_INDX_WIDTH) {1'b0}}, xrnic_adv_conf_sw_override_qpid}, xrnic_adv_base_cnt, ecn_timeout, cnp_timeout, 5'h0, xrnic_adv_conf_retry_cnt_fatal_dis, xrnic_adv_conf_errbuf_overwr_en, xrnic_adv_conf_sw_override};
                    read_mux[0]      <= 1'b1;
                end
                XRNIC_BUF_THRESHOLD_ROCE_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {roce_pause_off_offset_reg, roce_pause_on_offset_reg};
                    read_mux[0]      <= 1'b1;
                end
                XRNIC_BUF_THRESHOLD_NON_ROCE_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {non_roce_pause_off_offset_reg, non_roce_pause_on_offset_reg};
                    read_mux[0]      <= 1'b1;
                end
                XRNIC_PAUSE_CONF_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {18'h0, disable_priority_chk_reg, 1'b0, non_roce_priority_reg, roce_priority_reg, 2'b0, pfc_en_non_roce_reg, pfc_en_roce_reg};
                    read_mux[0]      <= 1'b1;
                end
                MAC_XRNIC_ADDR_LSB_REG[10:2]: begin
                    s_axi_lite_reg_1 <= mac_xrnic_addr_lsb;
                    read_mux[0]      <= 1'b1;
                end
                MAC_XRNIC_ADDR_MSB_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, mac_xrnic_addr_msb};
                    read_mux[0]      <= 1'b1;
                end
                IPV6_XRNIC_ADDR1_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv6_xrnic_addr1;
                    read_mux[0]      <= 1'b1;
                end
                IPV6_XRNIC_ADDR2_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv6_xrnic_addr2;
                    read_mux[0]      <= 1'b1;
                end
                IPV6_XRNIC_ADDR3_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv6_xrnic_addr3;
                    read_mux[0]      <= 1'b1;
                end
                IPV6_XRNIC_ADDR4_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv6_xrnic_addr4;
                    read_mux[0]      <= 1'b1;
                end
                TX_ERR_CORRUPT_CONF_REG[10:2]: begin
                    //s_axi_lite_reg_1 <= tx_hdr_buf_ba   ;
                    s_axi_lite_reg_1 <= error_corrupt_conf;
                    read_mux[0]      <= 1'b1;
                end
                TX_IPG_CONF_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {tx_hdr_buf_sz_buf_sz, tx_hdr_buf_sz_num_hdrs};
                    read_mux[0]      <= 1'b1;
                end
                WQE_SPLIT_GRAN_REG[10:2]: begin
                    s_axi_lite_reg_1 <= 'd0;  //tx_sgl_buf_ba;
                    read_mux[0]      <= 1'b1;
                end
                TX_SGL_BUF_SZ_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {tx_sgl_buf_sz_buf_sz, tx_sgl_buf_sz_num_sgls};
                    read_mux[0]      <= 1'b1;
                end
                BYPASS_BUF_BA_REG[10:2]: begin
                    s_axi_lite_reg_1 <= bypass_buf_ba;
                    read_mux[0]      <= 1'b1;
                end
                BYPASS_BUF_SZ_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {bypass_buf_sz_buf_sz, bypass_buf_sz_num_bufs};
                    read_mux[0]      <= 1'b1;
                end
                BYPASS_BUF_WRPTR_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, i_bypass_buf_wrptr};
                    read_mux[0]      <= 1'b1;
                end
                RQ_ERR_PKT_BUF_BA_REG[10:2]: begin
                    s_axi_lite_reg_1 <= rq_err_pkt_buf_ba;
                    read_mux[0]      <= 1'b1;
                end
                RQ_ERR_PKT_BUF_BA_REG_MSB[10:2]: begin
                    s_axi_lite_reg_1 <= rq_err_pkt_buf_ba_msb;
                    read_mux[0]      <= 1'b1;
                end
                RQ_ERR_PKT_BUF_SZ_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {rq_err_pkt_buf_sz_buf_sz, rq_err_pkt_buf_sz_num_bufs};
                    read_mux[0]      <= 1'b1;
                end
                RQ_ERR_PKT_BUF_WRPTR_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, i_rq_err_pkt_buf_wrptr};
                    read_mux[0]      <= 1'b1;
                end
                IPV4_XRNIC_ADDR_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv4_xrnic_addr;
                    read_mux[0]      <= 1'b1;
                end
                OUT_ERRSTS_Q_BA_REG[10:2]: begin
                    s_axi_lite_reg_1 <= out_errsts_q_ba;
                    read_mux[0]      <= 1'b1;
                end
                OUT_ERRSTS_Q_BA_REG_MSB[10:2]: begin
                    s_axi_lite_reg_1 <= out_errsts_q_ba_msb;
                    read_mux[0]      <= 1'b1;
                end
                OUT_ERRSTS_Q_SZ_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, out_errsts_q_sz};
                    read_mux[0]      <= 1'b1;
                end
                OUT_ERRSTS_Q_WRPTR_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, i_out_errsts_q_wrptr};
                    read_mux[0]      <= 1'b1;
                end
                IN_ERRSTS_Q_BA_REG[10:2]: begin
                    s_axi_lite_reg_1 <= in_errsts_q_ba;
                    read_mux[0]      <= 1'b1;
                end
                IN_ERRSTS_Q_BA_REG_MSB[10:2]: begin
                    s_axi_lite_reg_1 <= in_errsts_q_ba_msb;
                    read_mux[0]      <= 1'b1;
                end
                IN_ERRSTS_Q_SZ_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, in_errsts_q_sz};
                    read_mux[0]      <= 1'b1;
                end
                IN_ERRSTS_Q_WRPTR_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, i_in_errsts_q_wrptr};
                    read_mux[0]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_1 <= 'd0;
                    read_mux[0]      <= 1'b0;
                end
            endcase

            case (rd_addr_c1[10:2])
                DATA_BUF_BA_REG[10:2]: begin
                    s_axi_lite_reg_2 <= data_buf_ba;
                    read_mux[1]      <= 1'b1;
                end
                DATA_BUF_BA_MSB_REG[10:2]: begin
                    s_axi_lite_reg_2 <= data_buf_ba_msb;
                    read_mux[1]      <= 1'b1;
                end
                DATA_BUF_SZ_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {data_buf_sz_buf_sz, data_buf_sz_num_bufs};
                    read_mux[1]      <= 1'b1;
                end
                //UPdated the CNCT_IO_CONF register to have QPID and
                //address. The QPID is used to get the current RQ_PI_DB
                //value from BRAM which is fed to NVMf via the HW
                //handhsake mechanism
                CNCT_IO_CONF_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {i_connect_io_qp_rq_pi_db_rdy, 4'b0, connect_io_qpid, 3'b0, connect_io_qp_rq_pi_db_wptr};
                    read_mux[1]      <= 1'b1;
                end
                RESP_ERR_PKT_BUF_BA_REG[10:2]: begin
                    s_axi_lite_reg_2 <= resp_err_pkt_buf_ba;
                    read_mux[1]      <= 1'b1;
                end
                RESP_ERR_PKT_BUF_BA_REG_MSB[10:2]: begin
                    s_axi_lite_reg_2 <= resp_err_pkt_buf_msb_ba;
                    read_mux[1]      <= 1'b1;
                end
                RESP_ERR_PKT_BUF_SZ_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {resp_err_pkt_buf_sz_buf_sz, resp_err_pkt_buf_sz_num_bufs};
                    read_mux[1]      <= 1'b1;
                end
                RESP_ERR_PKT_BUF_WRPTR_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {16'b0, i_resp_err_pkt_buf_wrptr};
                    read_mux[1]      <= 1'b1;
                end

                RX_PKT_HNDL_DBG_CTRL_REG[10:2]: begin
                    s_axi_lite_reg_2 <= rx_pkt_hndl_dbg_ctrl;
                    read_mux[1]      <= 1'b1;
                end
                INC_SR_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {i_inc_rresp_pkt_cnt, i_inc_send_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                INC_AM_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {i_inc_mad_pkt_cnt, i_inc_ack_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                OUT_SR_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {i_out_rdwr_pkt_cnt, i_out_send_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                OUT_AM_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {i_out_mad_pkt_cnt, i_out_ack_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                OUT_RD_RESP_PKT_CNT[10:2]: begin
                    s_axi_lite_reg_2 <= {16'd0, i_out_rd_resp_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                LAST_IN_PKT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_last_in_pkt_info;  //{last_in_pkt_psn_lsb, last_in_pkt_qpid, last_in_pkt_opcode};
                    read_mux[1]      <= 1'b1;
                end
                LAST_OUT_PKT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_last_out_pkt_info;  //{last_out_pkt_psn_lsb, last_out_pkt_qpid, last_out_pkt_opcode};
                    read_mux[1]      <= 1'b1;
                end
                INV_DUP_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {i_inc_dup_pkt_cnt, i_inc_inv_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                RNR_IN_PKT_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= 'b0;
                    read_mux[1]      <= 1'b1;
                end
                RNR_OUT_PKT_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= 'b0;
                    read_mux[1]      <= 1'b1;
                end
                WQE_PROC_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_wqe_proc_sts;
                    read_mux[1]      <= 1'b1;
                end
                RX_PKT_VLD_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_rx_pkt_vld_sts;
                    read_mux[1]      <= 1'b1;
                end
                QP_MGR_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_qp_mgr_sts;
                    read_mux[1]      <= 1'b1;
                end
                INC_ALL_DROPPED_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_inc_all_dropped_cnt;
                    read_mux[1]      <= 1'b1;
                end
                INC_NACK_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {16'h0, i_inc_nack_cnt};
                    read_mux[1]      <= 1'b1;
                end
                OUT_NACK_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {16'h0, i_out_nack_cnt};
                    read_mux[1]      <= 1'b1;
                end
                RESP_HNDL_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_resp_hndler_sts;
                    read_mux[1]      <= 1'b1;
                end
                RETRY_CNT_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_stat_retry_cnt;
                    read_mux[1]      <= 1'b1;
                end
                DBG_CNT_OVERRIDE_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {8'h0, i_axi_rd_err_cnt, xrnic_sw_override_cnt};  //i_min_ipg_stat;
                    read_mux[1]      <= 1'b1;
                end
                IPG_0_4_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_ipg_0_4_cnt;
                    read_mux[1]      <= 1'b1;
                end
                IPG_5_9_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_ipg_5_9_cnt;
                    read_mux[1]      <= 1'b1;
                end
                IPG_10_14_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_ipg_10_14_cnt;
                    read_mux[1]      <= 1'b1;
                end
                IPG_15_19_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_ipg_15_19_cnt;
                    read_mux[1]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_2 <= 'd0;
                    read_mux[1]      <= 1'b0;
                end
            endcase

            case (rd_addr_c1[10:2])
                GLOBAL_DBG_CNTR[10:2]: begin
                    s_axi_lite_reg_3 <= global_dbg_cnt;
                    read_mux[2]      <= 1'b1;
                end
                OSQ_BKP_SQE_BKP_CNT[10:2]: begin
                    s_axi_lite_reg_3 <= {osq_backpressure_cnt_ff, sqe_backpressure_cnt_ff};
                    read_mux[2]      <= 1'b1;
                end
                RETRY_TX_BKP_HDR_BKP_CNT[10:2]: begin
                    s_axi_lite_reg_3 <= {i_retry_tx_backpressure_cnt, i_hdr_backpressure_cnt};
                    read_mux[2]      <= 1'b1;
                end
                SQE_EMPTY_WQE_IDLE_CNT[10:2]: begin
                    s_axi_lite_reg_3 <= {all_sqes_empty_cnt_ff, i_wqe_fsm_idle_cnt};
                    read_mux[2]      <= 1'b1;
                end
                WQE_PROC_SM_CNT0[10:2]: begin
                    s_axi_lite_reg_3 <= {i_wqe_proc_rd_q_info_cnt, i_wqe_proc_rd_wqe_cnt};
                    read_mux[2]      <= 1'b1;
                end
                WQE_PROC_SM_CNT1[10:2]: begin
                    s_axi_lite_reg_3 <= {i_wqe_proc_ip_chksum_cnt, i_wqe_proc_wait0_cnt};
                    read_mux[2]      <= 1'b1;
                end
                WQE_PROC_SM_CNT2[10:2]: begin
                    s_axi_lite_reg_3 <= {i_wqe_proc_hdr_sto_cnt, i_wqe_proc_hdr_gen_cnt};
                    read_mux[2]      <= 1'b1;
                end
                IN_CNP_PKT_CNT[10:2]: begin
                    s_axi_lite_reg_3 <= {16'h0, i_inc_cnp_pkt_cnt};
                    read_mux[2]      <= 1'b1;
                end
                OUT_CNP_PKT_CNT[10:2]: begin
                    s_axi_lite_reg_3 <= {16'h0, out_cnp_pkt_cnt};
                    read_mux[2]      <= 1'b1;
                end
                INTR_EN_REG[10:2]: begin
                    s_axi_lite_reg_3 <= {25'b0, intr_en_cnp, intr_en_fatal_err, intr_en_qp_pkt_rcvd, intr_en_ill_opc_in_sq, intr_en_wqe_cmpl, intr_en_rnr_nak_gen, intr_en_bypass_pkt_rcvd, intr_en_mad_pkt_rcvd, intr_en_pkt_valdn_err};
                    read_mux[2]      <= 1'b1;
                end
                INTR_STS_REG[10:2]: begin
                    s_axi_lite_reg_3 <= {25'b0, i_cnp_sched_intr_sts, qp_fatal_sts, i_qp_pkt_rcvd_intr_or_reduce, i_ill_opc_in_sq_intr, i_qp_wq_cmpl_intr_or_reduce, i_rnr_nack_gen_intr, i_bypass_pkt_rcvd_intr, i_mad_pkt_rcvd_intr, i_pkt_valdn_err_intr};
                    read_mux[2]      <= 1'b1;
                end
                RQ_INTR_STS1_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_pkt_rcvd_intr;
                    read_mux[2]      <= 1'b1;
                end
                RQ_INTR_STS2_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_pkt_rcvd_intr;
                    read_mux[2]      <= 1'b1;
                end
                RQ_INTR_STS3_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_pkt_rcvd_intr;
                    read_mux[2]      <= 1'b1;
                end
                RQ_INTR_STS4_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_pkt_rcvd_intr;
                    read_mux[2]      <= 1'b1;
                end
                RQ_INTR_STS5_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_pkt_rcvd_intr;
                    read_mux[2]      <= 1'b1;
                end
                RQ_INTR_STS6_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_pkt_rcvd_intr;
                    read_mux[2]      <= 1'b1;
                end
                RQ_INTR_STS7_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_pkt_rcvd_intr;
                    read_mux[2]      <= 1'b1;
                end
                RQ_INTR_STS8_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_pkt_rcvd_intr;
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS1_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_wq_cmpl_intr;
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS2_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_wq_cmpl_intr;
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS3_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_wq_cmpl_intr;
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS4_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_wq_cmpl_intr;
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS5_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_wq_cmpl_intr;
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS6_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_wq_cmpl_intr;
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS7_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_wq_cmpl_intr;
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS8_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_qp_wq_cmpl_intr;
                    read_mux[2]      <= 1'b1;
                end
                CNP_SCHD_STS1_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_cnp_schedule_info;
                    read_mux[2]      <= 1'b1;
                end
                CNP_SCHD_STS2_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_cnp_schedule_info;
                    read_mux[2]      <= 1'b1;
                end
                CNP_SCHD_STS3_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_cnp_schedule_info;
                    read_mux[2]      <= 1'b1;
                end
                CNP_SCHD_STS4_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_cnp_schedule_info;
                    read_mux[2]      <= 1'b1;
                end
                CNP_SCHD_STS5_REG[10:2]: begin
                    s_axi_lite_reg_3 <= w_cnp_schedule_info;
                    read_mux[2]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_3 <= 'd0;
                    read_mux[2]      <= 1'b0;
                end
            endcase

            case (rd_addr_c1[10:2])
                CNP_SCHD_STS6_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_cnp_schedule_info;
                    read_mux[3]      <= 1'b1;
                end
                CNP_SCHD_STS7_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_cnp_schedule_info;
                    read_mux[3]      <= 1'b1;
                end
                CNP_SCHD_STS8_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_cnp_schedule_info;
                    read_mux[3]      <= 1'b1;
                end
                CYCLE_CNT[10:2]: begin
                    s_axi_lite_reg_4 <= cycle_count;
                    read_mux[3]      <= 1'b1;
                end
                RD_RSP_CNT_PER_SEC[10:2]: begin
                    s_axi_lite_reg_4 <= rd_resp_pkt_cnt_per_sec;
                    read_mux[3]      <= 1'b1;
                end
                RD_REQ_CNT_PER_SEC[10:2]: begin
                    s_axi_lite_reg_4 <= rd_req_pkt_cnt_per_sec;
                    read_mux[3]      <= 1'b1;
                end
                RD_REQ_CNT[10:2]: begin
                    s_axi_lite_reg_4 <= {i_dup_rd_req_cnt[15:0], i_good_rd_req_cnt[15:0]};
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS9_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS10_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS11_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS12_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS13_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS14_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS15_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS16_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS17_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS18_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS19_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS20_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS21_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS22_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS23_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS24_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS25_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS26_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS27_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS28_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS29_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end
                RQ_INTR_STS30_REG[10:2]: begin
                    s_axi_lite_reg_4 <= w_qp_pkt_rcvd_intr;
                    read_mux[3]      <= 1'b1;
                end

                default: begin
                    s_axi_lite_reg_4 <= 'd0;
                    read_mux[3]      <= 1'b0;
                end
            endcase

            // case for additional global interrupt registers 
            case (rd_addr_c1[10:2])
                RQ_INTR_STS31_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS32_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS33_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS34_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS35_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS36_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS37_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS38_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS39_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS40_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS41_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS42_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS43_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS44_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS45_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS46_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS47_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS48_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS49_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS50_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS51_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS52_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS53_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS54_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS55_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS56_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS57_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS58_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS59_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS60_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS61_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end
                RQ_INTR_STS62_REG[10:2]: begin
                    s_axi_lite_reg_5 <= w_qp_pkt_rcvd_intr;
                    read_mux[4]      <= 1'b1;
                end

                default: begin
                    s_axi_lite_reg_5 <= 'd0;
                    read_mux[4]      <= 1'b0;
                end

            endcase

            case (rd_addr_c1[10:2])
                RQ_INTR_STS63_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_pkt_rcvd_intr;
                    read_mux[5]      <= 1'b1;
                end
                RQ_INTR_STS64_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_pkt_rcvd_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS9_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS10_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS11_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS12_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS13_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS14_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS15_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS16_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS17_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS18_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS19_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS20_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS21_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS22_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS23_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS24_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS25_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS26_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS27_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS28_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS29_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS30_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS31_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS32_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS33_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS34_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS35_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS36_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS37_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end
                CQ_INTR_STS38_REG[10:2]: begin
                    s_axi_lite_reg_6 <= w_qp_wq_cmpl_intr;
                    read_mux[5]      <= 1'b1;
                end

                default: begin
                    s_axi_lite_reg_6 <= 'd0;
                    read_mux[5]      <= 1'b0;
                end

            endcase

            case (rd_addr_c1[10:2])
                CQ_INTR_STS39_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS40_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS41_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS42_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS43_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS44_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS45_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS46_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS47_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS48_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS49_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS50_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS51_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS52_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS53_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS54_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS55_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS56_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS57_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS58_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS59_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS60_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS61_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS62_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS63_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CQ_INTR_STS64_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_qp_wq_cmpl_intr;
                    read_mux[6]      <= 1'b1;
                end
                CNP_SCHD_STS9_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_cnp_schedule_info;
                    read_mux[6]      <= 1'b1;
                end
                CNP_SCHD_STS10_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_cnp_schedule_info;
                    read_mux[6]      <= 1'b1;
                end
                CNP_SCHD_STS11_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_cnp_schedule_info;
                    read_mux[6]      <= 1'b1;
                end
                CNP_SCHD_STS12_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_cnp_schedule_info;
                    read_mux[6]      <= 1'b1;
                end
                CNP_SCHD_STS13_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_cnp_schedule_info;
                    read_mux[6]      <= 1'b1;
                end
                CNP_SCHD_STS14_REG[10:2]: begin
                    s_axi_lite_reg_7 <= w_cnp_schedule_info;
                    read_mux[6]      <= 1'b1;
                end

                default: begin
                    s_axi_lite_reg_7 <= 'd0;
                    read_mux[6]      <= 1'b0;
                end
            endcase

            case (rd_addr_c1[10:2])
                CNP_SCHD_STS15_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS16_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS17_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS18_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS19_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS20_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS21_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS22_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS23_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS24_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS25_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS26_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS27_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS28_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS29_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS30_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS31_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS32_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS33_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS34_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS35_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS36_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS37_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS38_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS39_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS40_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS41_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS42_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS43_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS44_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS45_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                CNP_SCHD_STS46_REG[10:2]: begin
                    s_axi_lite_reg_8 <= w_cnp_schedule_info;
                    read_mux[7]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_8 <= 'd0;
                    read_mux[7]      <= 1'b0;
                end

            endcase

            case (rd_addr_c1[10:2])
                CNP_SCHD_STS47_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS48_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS49_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS50_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS51_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS52_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS53_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS54_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS55_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS56_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS57_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS58_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS59_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS60_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS61_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS62_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS63_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                CNP_SCHD_STS64_REG[10:2]: begin
                    s_axi_lite_reg_9 <= w_cnp_schedule_info;
                    read_mux[8]      <= 1'b1;
                end
                XRNIC_CONF_QP_EN_REG[10:2]: begin
                    s_axi_lite_reg_9 <= {{(32 - C_QP_INDX_WIDTH) {1'b0}}, xrnic_conf_num_qps_enabled};
                    read_mux[8]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_9 <= 'd0;
                    read_mux[8]      <= 1'b0;
                end
            endcase

            s_axi_lite_reg_1_r             <= s_axi_lite_reg_1;
            s_axi_lite_reg_2_r             <= s_axi_lite_reg_2;
            s_axi_lite_reg_3_r             <= s_axi_lite_reg_3;
            s_axi_lite_reg_4_r             <= s_axi_lite_reg_4;
            s_axi_lite_reg_5_r             <= s_axi_lite_reg_5;
            s_axi_lite_reg_6_r             <= s_axi_lite_reg_6;
            s_axi_lite_reg_7_r             <= s_axi_lite_reg_7;
            s_axi_lite_reg_8_r             <= s_axi_lite_reg_8;
            s_axi_lite_reg_9_r             <= s_axi_lite_reg_9;


            //s_axi_stat_qpn_rdata_p        <= s_axi_stat_qpn_rdata;
            s_axi_curr_sqptr_proc_rdata_p  <= s_axi_curr_sqptr_proc_rdata;
            s_axi_stat_rq_pi_db_rdata_p    <= s_axi_stat_rq_pi_db_rdata;

            s_axi_stat_qpn_rdata_p1        <= s_axi_stat_qpn_rdata;
            s_axi_curr_sqptr_proc_rdata_p1 <= s_axi_curr_sqptr_proc_rdata_p;
            s_axi_stat_rq_pi_db_rdata_p1   <= s_axi_stat_rq_pi_db_rdata_p;

            s_axi_stat_qpn_rdata_p2        <= s_axi_stat_qpn_rdata_p1;
            s_axi_curr_sqptr_proc_rdata_p2 <= s_axi_curr_sqptr_proc_rdata_p1;
            s_axi_stat_rq_pi_db_rdata_p2   <= s_axi_stat_rq_pi_db_rdata_p1;
            if (C_CONFIG_MEM_LATENCY == 3) begin  // current

                s_axi_stat_qpn_rdata_r        <= s_axi_stat_qpn_rdata;
                //s_axi_stat_qpn_rdata_r        <= {s_axi_stat_qpn_rdata[31:11],s_axi_stat_qpn_rdata_p[10:9],s_axi_stat_qpn_rdata[8:3],
                //                                  s_axi_stat_qpn_rdata_p[2],s_axi_stat_qpn_rdata[1:0]};
                s_axi_curr_sqptr_proc_rdata_r <= s_axi_curr_sqptr_proc_rdata_p;
                s_axi_stat_rq_pi_db_rdata_r   <= s_axi_stat_rq_pi_db_rdata_p;
            end else begin

                s_axi_stat_qpn_rdata_r        <= s_axi_stat_qpn_rdata_p2;
                s_axi_curr_sqptr_proc_rdata_r <= s_axi_curr_sqptr_proc_rdata_p2;
                s_axi_stat_rq_pi_db_rdata_r   <= s_axi_stat_rq_pi_db_rdata_p2;
            end

            if (s_axi_lite_rvalid && s_axi_lite_rready) begin
                rd_req_r1 <= 1'b0;
                rd_req_p1 <= 1'b0;
                rd_req_p2 <= 1'b0;
                rd_req_p3 <= 1'b0;
            end else begin
                rd_req_p1 <= rd_req_r;
                rd_req_p2 <= rd_req_p1;
                rd_req_p3 <= rd_req_p2;
                if (C_CONFIG_MEM_LATENCY == 3) begin
                    rd_req_r1 <= rd_req_p1;
                end else begin
                    rd_req_r1 <= rd_req_p3;
                end
            end
        end
    end

    assign s_axi_lite_reg_1_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_1 : s_axi_lite_reg_1_r;
    assign s_axi_lite_reg_2_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_2 : s_axi_lite_reg_2_r;
    assign s_axi_lite_reg_3_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_3 : s_axi_lite_reg_3_r;
    assign s_axi_lite_reg_4_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_4 : s_axi_lite_reg_4_r;
    assign s_axi_lite_reg_5_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_5 : s_axi_lite_reg_5_r;
    assign s_axi_lite_reg_6_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_6 : s_axi_lite_reg_6_r;
    assign s_axi_lite_reg_7_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_7 : s_axi_lite_reg_7_r;
    assign s_axi_lite_reg_8_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_8 : s_axi_lite_reg_8_r;
    assign s_axi_lite_reg_9_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_9 : s_axi_lite_reg_9_r;

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            s_axi_lite_rvalid <= 1'b0;
            s_axi_lite_rdata  <= 32'd0;
            rvalid_delayed    <= 1'b0;
            rd_addr_wen_r     <= 3'b000;
            axi_lite_rvalid_i <= 1'b0;
            rvalid_delayed_i  <= 1'b0;
            rd_req_mask       <= 1'b0;
        end else begin
            if (rd_req_r) begin
                if (axi_lite_rvalid_i && s_axi_lite_rready) begin
                    axi_lite_rvalid_i <= 1'b0;
                    rvalid_delayed_i  <= 1'b0;
                end else if (rd_req_mask == 1'b0) begin
                    axi_lite_rvalid_i <= rvalid_delayed_i;
                    rvalid_delayed_i  <= 1'b1;
                end
            end

            if (s_axi_lite_rvalid && s_axi_lite_rready) begin
                rd_req_mask <= 1'b0;
            end else if (axi_lite_rvalid_i == 1'b1) begin
                rd_req_mask <= 1'b1;
            end

            if (rd_req_r1) begin
                if (s_axi_lite_rvalid && s_axi_lite_rready) begin
                    s_axi_lite_rvalid <= 1'b0;
                    rvalid_delayed    <= 1'b0;
                end else begin
                    s_axi_lite_rvalid <= rvalid_delayed;
                    rvalid_delayed    <= 1'b1;
                end
                if (~s_axi_lite_rvalid & mux_select == 3'b001) begin
                    rd_addr_wen_r <= 3'b001;
                    case (read_mux)
                        9'b0_0000_0001: s_axi_lite_rdata <= s_axi_lite_reg_1_c;  //Remote offset lsb 32 bits 
                        9'b0_0000_0010: s_axi_lite_rdata <= s_axi_lite_reg_2_c;  //Remote Offset msb 32 bits
                        9'b0_0000_0100: s_axi_lite_rdata <= s_axi_lite_reg_3_c;  //Remote tag 
                        9'b0_0000_1000: s_axi_lite_rdata <= s_axi_lite_reg_4_c;  //Remote tag 
                        9'b0_0001_0000: s_axi_lite_rdata <= s_axi_lite_reg_5_c;
                        9'b0_0010_0000: s_axi_lite_rdata <= s_axi_lite_reg_6_c;
                        9'b0_0100_0000: s_axi_lite_rdata <= s_axi_lite_reg_7_c;
                        9'b0_1000_0000: s_axi_lite_rdata <= s_axi_lite_reg_8_c;
                        9'b1_0000_0000: s_axi_lite_rdata <= s_axi_lite_reg_9_c;
                        default: s_axi_lite_rdata <= 'b0;

                    endcase
                end else if (~s_axi_lite_rvalid & mux_select == 3'b010) begin
                    rd_addr_wen_r <= 3'b010;
                    case (rd_addr_r1[7:0])
                        QP_CONF_QPN:              s_axi_lite_rdata <= s_axi_qp_config_rdata;
                        QP_ADV_CONF_QPN:          s_axi_lite_rdata <= s_axi_qp_adv_config_rdata;
                        RQ_BUF_BA_QPN:            s_axi_lite_rdata <= s_axi_rq_ba_rdata;
                        SQ_BA_QPN:                s_axi_lite_rdata <= s_axi_sq_ba_rdata;
                        CQ_BA_QPN:                s_axi_lite_rdata <= s_axi_cq_ba_rdata;
                        RQ_BUF_BA_MSB_QPN:        s_axi_lite_rdata <= s_axi_rq_ba_msb_rdata;
                        SQ_BA_MSB_QPN:            s_axi_lite_rdata <= s_axi_sq_ba_msb_rdata;
                        CQ_BA_MSB_QPN:            s_axi_lite_rdata <= s_axi_cq_ba_msb_rdata;
                        RQ_WRPTR_DB_ADD_QPN:      s_axi_lite_rdata <= s_axi_rq_wrptrdb_add_rdata;
                        RQ_WRPTR_DB_ADD_QPN_MSB:  s_axi_lite_rdata <= s_axi_rq_wrptrdb_msb_add_rdata;
                        SQ_CMPL_DB_ADD_QPN:       s_axi_lite_rdata <= s_axi_sq_cmpldb_add_rdata;
                        SQ_CMPL_DB_ADD_QPN_MSB:   s_axi_lite_rdata <= s_axi_sq_cmpldb_msb_add_rdata;
                        CQ_HEAD_QPN:              s_axi_lite_rdata <= s_axi_cq_hdptr_rdata;
                        RQ_CI_DB_QPN:             s_axi_lite_rdata <= s_axi_rq_cidb_rdata;
                        SQ_PI_DB_QPN:             s_axi_lite_rdata <= s_axi_sq_pidb_rdata;
                        Q_DEPTH_QPN:              s_axi_lite_rdata <= s_axi_q_depth_rdata;
                        SQ_PSN_QPN:               s_axi_lite_rdata <= s_axi_sq_psn_rdata;
                        LAST_RQ_QPN:              s_axi_lite_rdata <= s_axi_last_rq_rdata;
                        DEST_QP_CONF_QPN:         s_axi_lite_rdata <= s_axi_dest_qpid_rdata;
                        TIMEOUT_QPN:              s_axi_lite_rdata <= s_axi_timeout_rdata;
                        MAC_REMOTE_ADDR_LSB_QPN:  s_axi_lite_rdata <= s_axi_mac_remote_addrl_rdata;
                        MAC_REMOTE_ADDR_MSB_QPN:  s_axi_lite_rdata <= s_axi_mac_remote_addrm_rdata;
                        IP_REMOTE_ADDR1_QPN:      s_axi_lite_rdata <= s_axi_ip_remote_addr1_rdata;
                        IP_REMOTE_ADDR2_QPN:      s_axi_lite_rdata <= s_axi_ip_remote_addr2_rdata;
                        IP_REMOTE_ADDR3_QPN:      s_axi_lite_rdata <= s_axi_ip_remote_addr3_rdata;
                        IP_REMOTE_ADDR4_QPN:      s_axi_lite_rdata <= s_axi_ip_remote_addr4_rdata;
                        STAT_SSN_QPN:             s_axi_lite_rdata <= s_axi_stat_ssn_rdata;
                        STAT_MSN_QPN:             s_axi_lite_rdata <= s_axi_stat_msn_rdata;
                        STAT_QPN:                 s_axi_lite_rdata <= s_axi_stat_qpn_rdata_r;  //check
                        STAT_CURR_SQPTR_PROC_QPN: s_axi_lite_rdata <= s_axi_curr_sqptr_proc_rdata_r;  //check
                        STAT_RESP_PSN_QPN:        s_axi_lite_rdata <= s_axi_stat_resp_psn_rdata;
                        STAT_RQ_BUF_CA:           s_axi_lite_rdata <= s_axi_stat_rq_buf_ca_rdata;
                        STAT_RQ_BUF_CA_MSB:       s_axi_lite_rdata <= s_axi_stat_rq_buf_ca_msb_rdata;
                        STAT_WQE_CNT:             s_axi_lite_rdata <= s_axi_stat_wqe_cnt_rdata;
                        STAT_RQ_PI_DB:            s_axi_lite_rdata <= s_axi_stat_rq_pi_db_rdata_r;  //check
                        STAT_RET_SQ_PSN:          s_axi_lite_rdata <= s_axi_stat_ret_sq_psn_rdata;
                        PD_NUMBER_QPN:            s_axi_lite_rdata <= {8'd0, s_axi_pd_number_rdata[23:0]};
                        default:                  s_axi_lite_rdata <= 'h0;
                    endcase
                end else if (~s_axi_lite_rvalid & mux_select == 3'b011) begin
                    rd_addr_wen_r <= 3'b011;
                    case (rd_addr_r1[7:0])
                        PD_NUMBER_PDN:       s_axi_lite_rdata <= {8'd0, s_axi_pd_pd_num_rdata[23:0]};
                        VIRT_ADDR_0_PDN:     s_axi_lite_rdata <= s_axi_virt_addr_0_rdata;
                        VIRT_ADDR_1_PDN:     s_axi_lite_rdata <= s_axi_virt_addr_1_rdata;
                        BUF_BASE_ADDR_0_PDN: s_axi_lite_rdata <= s_axi_buf_base_addr_0_rdata;
                        BUF_BASE_ADDR_1_PDN: s_axi_lite_rdata <= s_axi_buf_base_addr_1_rdata;
                        BUF_R_KEY_PDN:       s_axi_lite_rdata <= {24'd0, s_axi_buf_r_key_rdata[7:0]};
                        W_R_BUF_LEN_PDN:     s_axi_lite_rdata <= s_axi_w_r_buf_len_rdata;
                        ACCESS_TYPE_PDN:     s_axi_lite_rdata <= {s_axi_access_type_rdata[31:16], 12'd0, s_axi_access_type_rdata[3:0]};
                    endcase
                end else if (s_axi_lite_rready) begin
                    rd_addr_wen_r    <= 3'b000;
                    s_axi_lite_rdata <= 'b0;
                end
            end
        end
    end


    //******************************************************************************
    //AXI Lite write trasaction processing logic. 
    //when a write transaction is received, depending on address bits [5:2] the
    //data is written in to the corresponding register.  
    //The address bits [1:0] are not considred and it is expected that the
    //address is word aligned and writes into entire register.  
    // AXI Lite写事务处理逻辑。
    // 当接收到写事务时，根据地址位[5:2]将数据写入对应寄存器。
    // 地址位[1:0]不予考虑，且要求地址按字对齐并写入整个寄存器。 
    //******************************************************************************
    always @(posedge core_clk) begin
        if (~core_rstn) begin

            xrnic_conf_xrnic_en                <= 'b0;
            xrnic_conf_ipver                   <= 'b0;
            xrnic_conf_tx_ack_gen              <= 'b0;
            xrnic_conf_err_buf_en              <= 'b0;
            xrnic_conf_flow_credits            <= 'b0;
            xrnic_conf_depkt_bypass_en         <= 'b0;
            xrnic_conf_num_qps_enabled         <= 'b0;
            xrnic_conf_udp_src_port            <= 'b0;
            xrnic_adv_conf_sw_override         <= 'b0;
            xrnic_adv_conf_sw_override_qpid    <= 'b0;
            xrnic_adv_conf_errbuf_overwr_en    <= 'b0;
            xrnic_adv_conf_retry_cnt_fatal_dis <= 'b0;
            xrnic_adv_base_cnt                 <= 'd10;
            cnp_timeout                        <= 'h0;
            ecn_timeout                        <= 'h0;

            connect_io_qp_rq_pi_db_wptr        <= 'b0;
            connect_io_qpid                    <= 'b0;
            //connect_io_residual_rq <= 'b0;

            rnr_nack_tval                      <= 'b0;
            retry_cnt                          <= 'b1;
            rnr_retry_cnt                      <= 'b1;
            timeout                            <= 'b0;

            intr_en_pkt_valdn_err              <= 'b0;
            intr_en_wqe_cmpl                   <= 'b0;
            intr_en_mad_pkt_rcvd               <= 'b0;
            intr_en_bypass_pkt_rcvd            <= 'b0;
            intr_en_rnr_nak_gen                <= 'b0;
            intr_en_qp_pkt_rcvd                <= 'b0;
            intr_en_fatal_err                  <= 'b0;
            intr_en_cnp                        <= 'b0;
            intr_en_ill_opc_in_sq              <= 'b0;
            rq_intr_sts_clr_wr_en              <= 'b0;
            rq_intr_sts_clr_wr_addr            <= 'b0;
            cnp_schd_sts_clr_wr_en             <= 'h0;
            cnp_schd_sts_clr_wr_addr           <= 'h0;
            cq_intr_sts_clr_wr_en              <= 'h0;
            cq_intr_sts_clr_wr_addr            <= 'h0;

            cycle_count                        <= 'h5F5E100;
            clear_rd_resp_cnt_per_sec          <= 0;
            clear_rd_req_cnt_per_sec           <= 0;

            wr_req_r1                          <= 1'b0;
            wr_addr_r1                         <= 'd0;
            wr_data_r1                         <= 32'h0;
        end else begin
            if (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r & generic_reg_wr_access) begin
                wr_req_r1  <= 1'b1;
                wr_addr_r1 <= wr_addr_r;
                wr_data_r1 <= s_axi_lite_wdata;

                case (wr_addr_r[10:2])  // 忽略地址位 [1:0]，按字对齐访问

                    XRNIC_CONF_REG[10:2]: begin  // XRNIC Configuration -- 对应手册 0x10_0000，下列均是
                        xrnic_conf_xrnic_en        <= s_axi_lite_wdata[0];
                        xrnic_conf_ipver           <= s_axi_lite_wdata[1];  // Reserved
                        xrnic_conf_depkt_bypass_en <= s_axi_lite_wdata[2];  // Reserved
                        xrnic_conf_tx_ack_gen      <= s_axi_lite_wdata[4:3];
                        xrnic_conf_err_buf_en      <= s_axi_lite_wdata[5];
                        xrnic_conf_flow_credits    <= s_axi_lite_wdata[7:6];  // Reserved
                        xrnic_conf_udp_src_port    <= s_axi_lite_wdata[23:8];
                        // [31:24] Reserved
                    end

                    XRNIC_CONF_QP_EN_REG[10:2]: begin  // XRNIC_CONF_QP_EN
                        xrnic_conf_num_qps_enabled <= s_axi_lite_wdata[C_QP_INDX_WIDTH-1:0];  // [10:0]: Number of QPs enabled
                        // [31:11] Reserved
                    end

                    XRNIC_ADV_CONF_REG[10:2]: begin  // XRNIC Advance Configuration
                        xrnic_adv_conf_sw_override         <= s_axi_lite_wdata[0];
                        xrnic_adv_conf_errbuf_overwr_en    <= s_axi_lite_wdata[1];  // Reserved
                        xrnic_adv_conf_retry_cnt_fatal_dis <= s_axi_lite_wdata[2];
                        // [7:3] Reserved
                        cnp_timeout                        <= s_axi_lite_wdata[11:8];  // Reserved
                        ecn_timeout                        <= s_axi_lite_wdata[15:12];  // Reserved
                        xrnic_adv_base_cnt                 <= s_axi_lite_wdata[19:16];
                        xrnic_adv_conf_sw_override_qpid    <= s_axi_lite_wdata[C_QP_INDX_WIDTH+19 : 20];  //2K QP's max support
                        // [31] Reserved
                    end

                    //  BYPASS_BUF_WRPTR_REG[10:2]: bypass_buf_wrptr <= s_axi_lite_wdata[15:0];


                    INTR_EN_REG[10:2]: begin  // Interrupt Enable (INTEN)
                        intr_en_pkt_valdn_err   <= s_axi_lite_wdata[0];
                        intr_en_mad_pkt_rcvd    <= s_axi_lite_wdata[1];
                        intr_en_bypass_pkt_rcvd <= s_axi_lite_wdata[2];  // Reserved
                        intr_en_rnr_nak_gen     <= s_axi_lite_wdata[3];
                        intr_en_wqe_cmpl        <= s_axi_lite_wdata[4];
                        intr_en_ill_opc_in_sq   <= s_axi_lite_wdata[5];
                        intr_en_qp_pkt_rcvd     <= s_axi_lite_wdata[6];
                        intr_en_fatal_err       <= s_axi_lite_wdata[7];
                        intr_en_cnp             <= s_axi_lite_wdata[8];
                    end

                    // 该寄存器用于在硬件握手刚刚使能时敲响门铃。
                    CNCT_IO_CONF_REG[10:2]: begin  // Connect IP configuration (CON_IO_CONF)
                        connect_io_qp_rq_pi_db_wptr <= s_axi_lite_wdata[12:0];  // Reserved
                        // [15:13] Reserved
                        connect_io_qpid             <= s_axi_lite_wdata[26:16];
                        // [30:27] Reserved
                        //connect_io_residual_rq <= s_axi_lite_wdata[31:16];
                    end

                    RQ_INTR_STS1_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 0-31 (RQINTSTS1)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd0;
                    end
                    RQ_INTR_STS2_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 32-63 (RQINTSTS2)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd1;
                    end
                    RQ_INTR_STS3_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 64-95 (RQINTSTS3)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd2;
                    end
                    RQ_INTR_STS4_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 96-127 (RQINTSTS4)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd3;
                    end
                    RQ_INTR_STS5_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 128-159 (RQINTSTS5)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd4;
                    end
                    RQ_INTR_STS6_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 160-191 (RQINTSTS6)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd5;
                    end
                    RQ_INTR_STS7_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 192-223 (RQINTSTS7)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd6;
                    end
                    RQ_INTR_STS8_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 224-255 (RQINTSTS8)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd7;
                    end
                    RQ_INTR_STS9_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 256-287 (RQINTSTS9)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd8;
                    end
                    RQ_INTR_STS10_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 288-319 (RQINTSTS10)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd9;
                    end
                    RQ_INTR_STS11_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 320-351 (RQINTSTS11)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd10;
                    end
                    RQ_INTR_STS12_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 352-383 (RQINTSTS12)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd11;
                    end
                    RQ_INTR_STS13_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 384-415 (RQINTSTS13)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd12;
                    end
                    RQ_INTR_STS14_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 416-447 (RQINTSTS14)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd13;
                    end
                    RQ_INTR_STS15_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 448-479 (RQINTSTS15)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd14;
                    end
                    RQ_INTR_STS16_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 480-511 (RQINTSTS16)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd15;
                    end
                    RQ_INTR_STS17_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 512-543 (RQINTSTS17)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd16;
                    end
                    RQ_INTR_STS18_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 544-575 (RQINTSTS18)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd17;
                    end
                    RQ_INTR_STS19_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 576-607 (RQINTSTS19)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd18;
                    end
                    RQ_INTR_STS20_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 608-639 (RQINTSTS20)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd19;
                    end
                    RQ_INTR_STS21_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 640-671 (RQINTSTS21)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd20;
                    end
                    RQ_INTR_STS22_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 672-703 (RQINTSTS22)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd21;
                    end
                    RQ_INTR_STS23_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 704-735 (RQINTSTS23)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd22;
                    end
                    RQ_INTR_STS24_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 736-767 (RQINTSTS24)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd23;
                    end
                    RQ_INTR_STS25_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 768-799 (RQINTSTS25)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd24;
                    end
                    RQ_INTR_STS26_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 800-831 (RQINTSTS26)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd25;
                    end
                    RQ_INTR_STS27_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 832-863 (RQINTSTS27)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd26;
                    end
                    RQ_INTR_STS28_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 864-895 (RQINTSTS28)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd27;
                    end
                    RQ_INTR_STS29_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 896-927 (RQINTSTS29)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd28;
                    end
                    RQ_INTR_STS30_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 928-959 (RQINTSTS30)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd29;
                    end
                    RQ_INTR_STS31_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 960-991 (RQINTSTS31)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd30;
                    end
                    RQ_INTR_STS32_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 992-1023 (RQINTSTS32)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd31;
                    end
                    RQ_INTR_STS33_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1024-1055 (RQINTSTS33)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd32;
                    end
                    RQ_INTR_STS34_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1056-1087 (RQINTSTS34)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd33;
                    end
                    RQ_INTR_STS35_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1088-1119 (RQINTSTS35)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd34;
                    end
                    RQ_INTR_STS36_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1120-1151 (RQINTSTS36)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd35;
                    end
                    RQ_INTR_STS37_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1152-1183 (RQINTSTS37)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd36;
                    end
                    RQ_INTR_STS38_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1184-1215 (RQINTSTS38)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd37;
                    end
                    RQ_INTR_STS39_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1216-1247 (RQINTSTS39)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd38;
                    end
                    RQ_INTR_STS40_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1248-1279 (RQINTSTS40)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd39;
                    end
                    RQ_INTR_STS41_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1280-1311 (RQINTSTS41)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd40;
                    end
                    RQ_INTR_STS42_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1312-1343 (RQINTSTS42)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd41;
                    end
                    RQ_INTR_STS43_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1344-1375 (RQINTSTS43)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd42;
                    end
                    RQ_INTR_STS44_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1376-1407 (RQINTSTS44)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd43;
                    end
                    RQ_INTR_STS45_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1408-1439 (RQINTSTS45)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd44;
                    end
                    RQ_INTR_STS46_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1440-1471 (RQINTSTS46)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd45;
                    end
                    RQ_INTR_STS47_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1472-1503 (RQINTSTS47)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd46;
                    end
                    RQ_INTR_STS48_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1504-1535 (RQINTSTS48)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd47;
                    end
                    RQ_INTR_STS49_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1536-1567 (RQINTSTS49)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd48;
                    end
                    RQ_INTR_STS50_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1568-1599 (RQINTSTS50)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd49;
                    end
                    RQ_INTR_STS51_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1600-1631 (RQINTSTS51)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd50;
                    end
                    RQ_INTR_STS52_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1632-1663 (RQINTSTS52)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd51;
                    end
                    RQ_INTR_STS53_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1664-1695 (RQINTSTS53)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd52;
                    end
                    RQ_INTR_STS54_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1696-1727 (RQINTSTS54)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd53;
                    end
                    RQ_INTR_STS55_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1728-1759 (RQINTSTS55)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd54;
                    end
                    RQ_INTR_STS56_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1760-1791 (RQINTSTS56)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd55;
                    end
                    RQ_INTR_STS57_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1792-1823 (RQINTSTS57)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd56;
                    end
                    RQ_INTR_STS58_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1824-1855 (RQINTSTS58)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd57;
                    end
                    RQ_INTR_STS59_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1856-1887 (RQINTSTS59)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd58;
                    end
                    RQ_INTR_STS60_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1888-1919 (RQINTSTS60)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd59;
                    end
                    RQ_INTR_STS61_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1920-1951 (RQINTSTS61)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd60;
                    end
                    RQ_INTR_STS62_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1952-1983 (RQINTSTS62)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd61;
                    end
                    RQ_INTR_STS63_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 1984-2015 (RQINTSTS63)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd62;
                    end
                    RQ_INTR_STS64_REG[10:2]: begin  // RCV Q interrupt status (bitwise) QP 2016-2047 (RQINTSTS64)
                        rq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        rq_intr_sts_clr_wr_addr[10:5] <= 'd63;
                    end


                    // Completion Q interrupt status (bitwise) QP 0-31 (CQINTSTS1) ...
                    CQ_INTR_STS1_REG[10:2]: begin  // Completion Q interrupt status (bitwise) QP 0-31 (CQINTSTS1)
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd0;
                    end
                    CQ_INTR_STS2_REG[10:2]: begin  // Completion Q interrupt status (bitwise) QP 32-63 (CQINTSTS2)
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd1;
                    end
                    CQ_INTR_STS3_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd2;
                    end
                    CQ_INTR_STS4_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd3;
                    end
                    CQ_INTR_STS5_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd4;
                    end
                    CQ_INTR_STS6_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd5;
                    end
                    CQ_INTR_STS7_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd6;
                    end
                    CQ_INTR_STS8_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd7;
                    end
                    CQ_INTR_STS9_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd8;
                    end
                    CQ_INTR_STS10_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd9;
                    end
                    CQ_INTR_STS11_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd10;
                    end
                    CQ_INTR_STS12_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd11;
                    end
                    CQ_INTR_STS13_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd12;
                    end
                    CQ_INTR_STS14_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd13;
                    end
                    CQ_INTR_STS15_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd14;
                    end
                    CQ_INTR_STS16_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd15;
                    end
                    CQ_INTR_STS17_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd16;
                    end
                    CQ_INTR_STS18_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd17;
                    end
                    CQ_INTR_STS19_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd18;
                    end
                    CQ_INTR_STS20_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd19;
                    end
                    CQ_INTR_STS21_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd20;
                    end
                    CQ_INTR_STS22_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd21;
                    end
                    CQ_INTR_STS23_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd22;
                    end
                    CQ_INTR_STS24_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd23;
                    end
                    CQ_INTR_STS25_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd24;
                    end
                    CQ_INTR_STS26_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd25;
                    end
                    CQ_INTR_STS27_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd26;
                    end
                    CQ_INTR_STS28_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd27;
                    end
                    CQ_INTR_STS29_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd28;
                    end
                    CQ_INTR_STS30_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd29;
                    end
                    CQ_INTR_STS31_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd30;
                    end
                    CQ_INTR_STS32_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd31;
                    end
                    CQ_INTR_STS33_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd32;
                    end
                    CQ_INTR_STS34_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd33;
                    end
                    CQ_INTR_STS35_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd34;
                    end
                    CQ_INTR_STS36_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd35;
                    end
                    CQ_INTR_STS37_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd36;
                    end
                    CQ_INTR_STS38_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd37;
                    end
                    CQ_INTR_STS39_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd38;
                    end
                    CQ_INTR_STS40_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd39;
                    end
                    CQ_INTR_STS41_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd40;
                    end
                    CQ_INTR_STS42_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd41;
                    end
                    CQ_INTR_STS43_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd42;
                    end
                    CQ_INTR_STS44_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd43;
                    end
                    CQ_INTR_STS45_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd44;
                    end
                    CQ_INTR_STS46_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd45;
                    end
                    CQ_INTR_STS47_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd46;
                    end
                    CQ_INTR_STS48_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd47;
                    end
                    CQ_INTR_STS49_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd48;
                    end
                    CQ_INTR_STS50_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd49;
                    end
                    CQ_INTR_STS51_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd50;
                    end
                    CQ_INTR_STS52_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd51;
                    end
                    CQ_INTR_STS53_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd52;
                    end
                    CQ_INTR_STS54_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd53;
                    end
                    CQ_INTR_STS55_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd54;
                    end
                    CQ_INTR_STS56_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd55;
                    end
                    CQ_INTR_STS57_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd56;
                    end
                    CQ_INTR_STS58_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd57;
                    end
                    CQ_INTR_STS59_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd58;
                    end
                    CQ_INTR_STS60_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd59;
                    end
                    CQ_INTR_STS61_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd60;
                    end
                    CQ_INTR_STS62_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd61;
                    end
                    CQ_INTR_STS63_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd62;
                    end
                    CQ_INTR_STS64_REG[10:2]: begin
                        cq_intr_sts_clr_wr_en         <= s_axi_lite_wdata[31:0];
                        cq_intr_sts_clr_wr_addr[10:5] <= 'd63;
                    end


                    // CNP Packed scheduled interrupt status (bitwise) QP 0-31 (CNPSCHDSTS1REG) ...
                    CNP_SCHD_STS1_REG[10:2]: begin  // CNP Packed scheduled interrupt status (bitwise) QP 0-31 (CNPSCHDSTS1REG)
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd0;
                    end
                    CNP_SCHD_STS2_REG[10:2]: begin  // CNP Packed scheduled interrupt status (bitwise) QP 32-63 (CNPSCHDSTS2REG)
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd1;
                    end
                    CNP_SCHD_STS3_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd2;
                    end
                    CNP_SCHD_STS4_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd3;
                    end
                    CNP_SCHD_STS5_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd4;
                    end
                    CNP_SCHD_STS6_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd5;
                    end
                    CNP_SCHD_STS7_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd6;
                    end
                    CNP_SCHD_STS8_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd7;
                    end
                    CNP_SCHD_STS9_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd8;
                    end
                    CNP_SCHD_STS10_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd9;
                    end
                    CNP_SCHD_STS11_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd10;
                    end
                    CNP_SCHD_STS12_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd11;
                    end
                    CNP_SCHD_STS13_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd12;
                    end
                    CNP_SCHD_STS14_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd13;
                    end
                    CNP_SCHD_STS15_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd14;
                    end
                    CNP_SCHD_STS16_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd15;
                    end
                    CNP_SCHD_STS17_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd16;
                    end
                    CNP_SCHD_STS18_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd17;
                    end
                    CNP_SCHD_STS19_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd18;
                    end
                    CNP_SCHD_STS20_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd19;
                    end
                    CNP_SCHD_STS21_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd20;
                    end
                    CNP_SCHD_STS22_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd21;
                    end
                    CNP_SCHD_STS23_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd22;
                    end
                    CNP_SCHD_STS24_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd23;
                    end
                    CNP_SCHD_STS25_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd24;
                    end
                    CNP_SCHD_STS26_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd25;
                    end
                    CNP_SCHD_STS27_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd26;
                    end
                    CNP_SCHD_STS28_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd27;
                    end
                    CNP_SCHD_STS29_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd28;
                    end
                    CNP_SCHD_STS30_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd29;
                    end
                    CNP_SCHD_STS31_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd30;
                    end
                    CNP_SCHD_STS32_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd31;
                    end
                    CNP_SCHD_STS33_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd32;
                    end
                    CNP_SCHD_STS34_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd33;
                    end
                    CNP_SCHD_STS35_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd34;
                    end
                    CNP_SCHD_STS36_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd35;
                    end
                    CNP_SCHD_STS37_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd36;
                    end
                    CNP_SCHD_STS38_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd37;
                    end
                    CNP_SCHD_STS39_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd38;
                    end
                    CNP_SCHD_STS40_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd39;
                    end
                    CNP_SCHD_STS41_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd40;
                    end
                    CNP_SCHD_STS42_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd41;
                    end
                    CNP_SCHD_STS43_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd42;
                    end
                    CNP_SCHD_STS44_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd43;
                    end
                    CNP_SCHD_STS45_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd44;
                    end
                    CNP_SCHD_STS46_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd45;
                    end
                    CNP_SCHD_STS47_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd46;
                    end
                    CNP_SCHD_STS48_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd47;
                    end
                    CNP_SCHD_STS49_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd48;
                    end
                    CNP_SCHD_STS50_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd49;
                    end
                    CNP_SCHD_STS51_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd50;
                    end
                    CNP_SCHD_STS52_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd51;
                    end
                    CNP_SCHD_STS53_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd52;
                    end
                    CNP_SCHD_STS54_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd53;
                    end
                    CNP_SCHD_STS55_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd54;
                    end
                    CNP_SCHD_STS56_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd55;
                    end
                    CNP_SCHD_STS57_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd56;
                    end
                    CNP_SCHD_STS58_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd57;
                    end
                    CNP_SCHD_STS59_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd58;
                    end
                    CNP_SCHD_STS60_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd59;
                    end
                    CNP_SCHD_STS61_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd60;
                    end
                    CNP_SCHD_STS62_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd61;
                    end
                    CNP_SCHD_STS63_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd62;
                    end
                    CNP_SCHD_STS64_REG[10:2]: begin
                        cnp_schd_sts_clr_wr_en[31:0]   <= s_axi_lite_wdata[31:0];
                        cnp_schd_sts_clr_wr_addr[10:5] <= 'd63;
                    end

                    // 手册上没有这些寄存器空间及介绍
                    CYCLE_CNT[10:2]:          cycle_count <= s_axi_lite_wdata[31:0];
                    RD_REQ_CNT_PER_SEC[10:2]: clear_rd_req_cnt_per_sec <= !clear_rd_req_cnt_per_sec;
                    RD_RSP_CNT_PER_SEC[10:2]: clear_rd_resp_cnt_per_sec <= !clear_rd_resp_cnt_per_sec;

                endcase
            end else begin
                cnp_schd_sts_clr_wr_en   <= 'd0;
                cnp_schd_sts_clr_wr_addr <= 'd0;
                wr_req_r1                <= 1'b0;
                rq_intr_sts_clr_wr_en    <= 'd0;
                rq_intr_sts_clr_wr_addr  <= 'd0;
                cq_intr_sts_clr_wr_en    <= 'd0;
                cq_intr_sts_clr_wr_addr  <= 'd0;
                //error_corrupt_conf[0] <= 1'b0; 
            end
        end
    end

    // 对初始配置进行额外注册，以尽量减少时序路径
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            mac_xrnic_addr_lsb            <= 'b0;
            mac_xrnic_addr_msb            <= 'b0;
            roce_pause_on_offset_reg      <= 16'd90;
            roce_pause_off_offset_reg     <= 16'd150;
            non_roce_pause_on_offset_reg  <= 16'd90;
            non_roce_pause_off_offset_reg <= 16'd150;
            pfc_en_roce_reg               <= 1'b0;
            pfc_en_non_roce_reg           <= 1'b0;
            roce_priority_reg             <= 4'h0;
            non_roce_priority_reg         <= 4'h0;
            disable_priority_chk_reg      <= 1'b1;

            ipv4_xrnic_addr               <= 'b0;
            ipv6_xrnic_addr1              <= 'b0;
            ipv6_xrnic_addr2              <= 'b0;
            ipv6_xrnic_addr3              <= 'b0;
            ipv6_xrnic_addr4              <= 'b0;

            //tx_hdr_buf_ba          <= 'b0;
            error_corrupt_conf            <= 'b0;
            tx_hdr_buf_sz_num_hdrs        <= 'b0;
            tx_hdr_buf_sz_buf_sz          <= 'b0;
            tx_sgl_buf_ba                 <= 'b0;
            tx_sgl_buf_sz_num_sgls        <= 'b0;
            tx_sgl_buf_sz_buf_sz          <= 'b0;

            bypass_buf_ba                 <= 'b0;
            bypass_buf_sz_num_bufs        <= 'b0;
            bypass_buf_sz_buf_sz          <= 'b0;
            //bypass_buf_wrptr <= 'b0;
            rq_err_pkt_buf_ba             <= 'b0;
            rq_err_pkt_buf_ba_msb         <= 'b0;
            rq_err_pkt_buf_sz_num_bufs    <= 'b0;
            rq_err_pkt_buf_sz_buf_sz      <= 'b0;
            //err_pkt_buf_wrptr <= 'b0;

            data_buf_ba                   <= 'b0;
            data_buf_ba_msb               <= 'b0;
            data_buf_sz_num_bufs          <= 'b0;
            data_buf_sz_buf_sz            <= 'b0;

            resp_err_pkt_buf_ba           <= 'b0;
            resp_err_pkt_buf_msb_ba       <= 'b0;
            resp_err_pkt_buf_sz_num_bufs  <= 'b0;
            resp_err_pkt_buf_sz_buf_sz    <= 'b0;

            rx_pkt_hndl_dbg_ctrl          <= 'b0;
            out_cnp_pkt_cnt               <= 'h0;

            out_errsts_q_ba               <= 'b0;
            out_errsts_q_ba_msb           <= 'b0;
            out_errsts_q_sz               <= 'b0;

            in_errsts_q_ba                <= 'b0;
            in_errsts_q_ba_msb            <= 'b0;
            in_errsts_q_sz                <= 'b0;

        end else begin

            if (wr_req_r1) begin
                case (wr_addr_r1[10:2])
                    // 对应手册 0x10_0010，下列均是
                    MAC_XRNIC_ADDR_LSB_REG[10:2]: mac_xrnic_addr_lsb <= wr_data_r1[31:0];  // MAC XRNIC Address LSB(MACXADDLSB)
                    MAC_XRNIC_ADDR_MSB_REG[10:2]: mac_xrnic_addr_msb <= wr_data_r1[15:0];  // MAC XRNIC Address MSB(MACXADDMSB)
                    XRNIC_BUF_THRESHOLD_ROCE_REG[10:2]: begin  // XRNIC_BUF_THRESHOLD_ROCE
                        roce_pause_on_offset_reg  <= wr_data_r1[15:0];
                        roce_pause_off_offset_reg <= wr_data_r1[31:16];
                    end

                    XRNIC_BUF_THRESHOLD_NON_ROCE_REG[10:2]: begin  // XRNIC_BUF_THRESHOLD_NON_ROCE
                        non_roce_pause_on_offset_reg  <= wr_data_r1[15:0];
                        non_roce_pause_off_offset_reg <= wr_data_r1[31:16];
                    end
                    XRNIC_PAUSE_CONF_REG[10:2]: begin  // XRNIC_PAUSE_CONF
                        pfc_en_roce_reg          <= wr_data_r1[0];
                        pfc_en_non_roce_reg      <= wr_data_r1[1];
                        roce_priority_reg        <= wr_data_r1[7:4];
                        non_roce_priority_reg    <= wr_data_r1[11:8];
                        disable_priority_chk_reg <= wr_data_r1[13];
                    end
                    IPV4_XRNIC_ADDR_REG[10:2]: begin  // IPv4 XRNIC Address(IPv4XADD)
                        ipv4_xrnic_addr <= wr_data_r1[31:0];
                    end
                    IPV6_XRNIC_ADDR1_REG[10:2]:    ipv6_xrnic_addr1 <= wr_data_r1[31:0];    // IPv6 Address 1(IPv6XADD1)
                    IPV6_XRNIC_ADDR2_REG[10:2]:    ipv6_xrnic_addr2 <= wr_data_r1[31:0];    // IPv6 Address 2(IPv6XADD2)
                    IPV6_XRNIC_ADDR3_REG[10:2]:    ipv6_xrnic_addr3 <= wr_data_r1[31:0];    // IPv6 Address 3(IPv6XADD3)
                    IPV6_XRNIC_ADDR4_REG[10:2]:    ipv6_xrnic_addr4 <= wr_data_r1[31:0];    // IPv6 Address 4(IPv6XADD4)
                    //TX_HDR_BUF_BA_REG[10:2]:      tx_hdr_buf_ba        <= wr_data_r1[31:0];
                    TX_ERR_CORRUPT_CONF_REG[10:2]: error_corrupt_conf <= wr_data_r1[31:0];  // 手册上没有这个寄存器介绍

                    TX_IPG_CONF_REG[10:2]: begin  // 手册上没有这个寄存器介绍
                        tx_hdr_buf_sz_num_hdrs <= wr_data_r1[15:0];
                        tx_hdr_buf_sz_buf_sz   <= wr_data_r1[31:16];
                    end

                    // WQE_SPLIT_GRAN_REG[10:2]:      //tx_sgl_buf_ba        <= wr_data_r1[31:0];

                    TX_SGL_BUF_SZ_REG[10:2]: begin  // 手册上没有这个寄存器介绍
                        tx_sgl_buf_sz_num_sgls <= wr_data_r1[15:0];
                        tx_sgl_buf_sz_buf_sz   <= wr_data_r1[31:16];
                    end

                    BYPASS_BUF_BA_REG[10:2]: bypass_buf_ba <= wr_data_r1[31:0];  // 手册上没有这个寄存器介绍

                    BYPASS_BUF_SZ_REG[10:2]: begin  // 手册上没有这个寄存器介绍
                        bypass_buf_sz_num_bufs <= wr_data_r1[15:0];
                        bypass_buf_sz_buf_sz   <= wr_data_r1[31:16];
                    end

                    RQ_ERR_PKT_BUF_BA_REG[10:2]:     rq_err_pkt_buf_ba <= wr_data_r1[31:0];  // Error Buffer Base Address(ERRBUFBA)
                    RQ_ERR_PKT_BUF_BA_REG_MSB[10:2]: rq_err_pkt_buf_ba_msb <= wr_data_r1[31:0];  // Error Buffer Base Address msb(ERRBUFBAMSB)
                    RQ_ERR_PKT_BUF_SZ_REG[10:2]: begin  // Error Buffer Size(ERRBUFSZ)
                        rq_err_pkt_buf_sz_num_bufs <= wr_data_r1[15:0];
                        rq_err_pkt_buf_sz_buf_sz   <= wr_data_r1[31:16];
                    end

                    DATA_BUF_BA_REG[10:2]:     data_buf_ba <= wr_data_r1[31:0];  // Data Buffer Base Address(DATBUFBA)
                    DATA_BUF_BA_MSB_REG[10:2]: data_buf_ba_msb <= wr_data_r1[31:0];  // Data Buffer Base Address MSB(DATBUFBAMSB)
                    DATA_BUF_SZ_REG[10:2]: begin  // Data Buffer Size(DATBUFSZ)
                        data_buf_sz_num_bufs <= wr_data_r1[15:0];
                        data_buf_sz_buf_sz   <= wr_data_r1[31:16];
                    end


                    RESP_ERR_PKT_BUF_BA_REG[10:2]:     resp_err_pkt_buf_ba <= wr_data_r1[31:0];  // Response Error pkt buffer base address(RESPERRPKTBA)
                    RESP_ERR_PKT_BUF_BA_REG_MSB[10:2]: resp_err_pkt_buf_msb_ba <= wr_data_r1[31:0];  // Response Error pkt buffer base address msb(RESPERRPKTBAMSB)
                    RESP_ERR_PKT_BUF_SZ_REG[10:2]: begin  // Response Error pkt buffer  size address(RESPERRSZ)
                        resp_err_pkt_buf_sz_num_bufs <= wr_data_r1[15:0];
                        resp_err_pkt_buf_sz_buf_sz   <= wr_data_r1[31:16];
                    end

                    RX_PKT_HNDL_DBG_CTRL_REG[10:2]: rx_pkt_hndl_dbg_ctrl <= wr_data_r1[31:0];  // 手册上没有这个寄存器介绍
                    OUT_CNP_PKT_CNT[10:2]:          out_cnp_pkt_cnt <= (out_cnp_pkt_cnt + wr_data_r1[15:0]);  // Outgoing CNP packet count(OUTCNPPKTCNT)
                    //   ERR_PKT_BUF_WRPTR_REG[10:2]:    err_pkt_buf_wrptr <= wr_data_r1[15:0];
                    OUT_ERRSTS_Q_BA_REG[10:2]:      out_errsts_q_ba <= wr_data_r1[31:0];  // Outgoing Pkt Error status Queue Base address(OPKTERRQBA)
                    OUT_ERRSTS_Q_BA_REG_MSB[10:2]:  out_errsts_q_ba_msb <= wr_data_r1[31:0];  // Outgoing Pkt Error status Queue Base address(OPKTERRQBAMSB)
                    OUT_ERRSTS_Q_SZ_REG[10:2]:      out_errsts_q_sz <= wr_data_r1[15:0];  // Outgoing Error status Q size(OUTERRSTSQSZ)
                    IN_ERRSTS_Q_BA_REG[10:2]:       in_errsts_q_ba <= wr_data_r1[31:0];  // Incoming Pkt Error Status Queue Base address(IPKTERRQBA)
                    IN_ERRSTS_Q_BA_REG_MSB[10:2]:   in_errsts_q_ba_msb <= wr_data_r1[31:0];  // Incoming Pkt Error Status Queue Base address MSB(IPKTERRQBAMSB)
                    IN_ERRSTS_Q_SZ_REG[10:2]:       in_errsts_q_sz <= wr_data_r1[15:0];  // Incoming Pkt Error Status Queue Size(IPKTERRQSZ)
                    //GLOBAL_DBG_CNTR[10:2] :  global_dbg_cnt <= s_axi_lite_wdata[31:0]; //Already in use 
                endcase
            end
        end
    end


    // 对初始配置进行额外注册，以尽量减少时序路径
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            qp_conf_bram_wen_r               <= 'b0;
            qp_adv_conf_bram_wen_r           <= 'b0;
            qp_rq_ba_bram_wen_r              <= 'b0;
            qp_rq_ba_msb_bram_wen_r          <= 'b0;
            qp_sq_ba_bram_wen_r              <= 'b0;
            qp_sq_ba_msb_bram_wen_r          <= 'b0;
            qp_cq_ba_bram_wen_r              <= 'b0;
            qp_cq_ba_msb_bram_wen_r          <= 'b0;
            qp_rq_wrptrdb_add_bram_wen_r     <= 'b0;
            qp_rq_wrptrdb_add_msb_bram_wen_r <= 'b0;
            qp_sq_cmpldb_add_bram_wen_r      <= 'b0;
            qp_sq_cmpldb_add_msb_bram_wen_r  <= 'b0;
            qp_rq_cidb_bram_wen_r            <= 'b0;
            qp_q_depth_bram_wen_r            <= 'b0;

            qp_last_rq_bram_wen_r            <= 'b0;
            qp_pd_num_bram_wen_r             <= 'b0;
            pd_pd_num_bram_wen_r             <= 'b0;
            pd_virt_addr_0_bram_wen_r        <= 'b0;
            pd_virt_addr_1_bram_wen_r        <= 'b0;
            pd_buf_base_addr_0_bram_wen_r    <= 'b0;
            pd_buf_base_addr_1_bram_wen_r    <= 'b0;
            pd_buf_r_key_bram_wen_r          <= 'b0;
            pd_w_r_buf_len_bram_wen_r        <= 'b0;
            pd_access_type_bram_wen_r        <= 'b0;
            qp_dest_qpid_bram_wen_r          <= 'b0;
            qp_mac_remote_addrl_bram_wen_r   <= 'b0;
            qp_mac_remote_addrm_bram_wen_r   <= 'b0;
            qp_ip_remote_addr1_bram_wen_r    <= 'b0;
            qp_ip_remote_addr2_bram_wen_r    <= 'b0;
            qp_ip_remote_addr3_bram_wen_r    <= 'b0;
            qp_ip_remote_addr4_bram_wen_r    <= 'b0;
            qp_stat_msn_bram_wen_r           <= 'b0;
            qp_stat_ssn_bram_wen_r           <= 'b0;
            qp_timeout_bram_wen_r            <= 'b0;
            wr_addr_wen_r                    <= 'b0;
            wr_addr_mem_r_r                  <= 'b0;
            rd_addr_mem_r_r                  <= 'b0;
            wr_addr_mem_r1_r                 <= 'b0;
            wr_data_mem_r_r                  <= 'b0;
        end else begin

            //wr_addr_mem_r  <= wr_addr_r;
            rd_addr_mem_r_r <= rd_addr_r[8+:C_QP_INDX_WIDTH];
            if (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) begin
                wr_addr_mem_r_r  <= wr_addr_r[8+:C_QP_INDX_WIDTH];
                wr_addr_mem_r1_r <= wr_addr_r[8+:C_QP_INDX_WIDTH];
                wr_data_mem_r_r  <= s_axi_lite_wdata[31:0];
            end else begin
                wr_addr_mem_r_r  <= rd_addr_r[8+:C_QP_INDX_WIDTH];
                wr_addr_mem_r1_r <= rd_addr_r[8+:C_QP_INDX_WIDTH];
            end

            // wr_addr_r[20:19] == 2'b11 表示地址是 0x18 开头，即 0x18_0000 开始
            if ((wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && wr_addr_r[20:19] == 2'b11) begin
                wr_addr_wen_r                    <= 2'b01;
                qp_conf_bram_wen_r               <= 'b0;
                qp_adv_conf_bram_wen_r           <= 'b0;
                qp_rq_ba_bram_wen_r              <= 'b0;
                qp_sq_ba_bram_wen_r              <= 'b0;
                qp_cq_ba_bram_wen_r              <= 'b0;
                qp_rq_ba_msb_bram_wen_r          <= 'b0;
                qp_sq_ba_msb_bram_wen_r          <= 'b0;
                qp_cq_ba_msb_bram_wen_r          <= 'b0;
                qp_rq_wrptrdb_add_bram_wen_r     <= 'b0;
                qp_rq_wrptrdb_add_msb_bram_wen_r <= 'b0;
                qp_sq_cmpldb_add_bram_wen_r      <= 'b0;
                qp_sq_cmpldb_add_msb_bram_wen_r  <= 'b0;
                qp_rq_cidb_bram_wen_r            <= 'b0;
                qp_q_depth_bram_wen_r            <= 'b0;

                qp_last_rq_bram_wen_r            <= 'b0;
                qp_dest_qpid_bram_wen_r          <= 'b0;
                qp_timeout_bram_wen_r            <= 'b0;
                qp_mac_remote_addrl_bram_wen_r   <= 'b0;
                qp_mac_remote_addrm_bram_wen_r   <= 'b0;
                qp_ip_remote_addr1_bram_wen_r    <= 'b0;
                qp_ip_remote_addr2_bram_wen_r    <= 'b0;
                qp_ip_remote_addr3_bram_wen_r    <= 'b0;
                qp_ip_remote_addr4_bram_wen_r    <= 'b0;
                qp_stat_ssn_bram_wen_r           <= 'b0;
                qp_stat_msn_bram_wen_r           <= 'b0;
                qp_pd_num_bram_wen_r             <= 'b0;

                pd_pd_num_bram_wen_r             <= 'b0;
                pd_virt_addr_0_bram_wen_r        <= 'b0;
                pd_virt_addr_1_bram_wen_r        <= 'b0;
                pd_buf_base_addr_0_bram_wen_r    <= 'b0;
                pd_buf_base_addr_1_bram_wen_r    <= 'b0;
                pd_buf_r_key_bram_wen_r          <= 'b0;
                pd_w_r_buf_len_bram_wen_r        <= 'b0;
                pd_access_type_bram_wen_r        <= 'b0;
                case (wr_addr_r[7:0])
                    QP_CONF_QPN:             qp_conf_bram_wen_r <= 1'b1;
                    QP_ADV_CONF_QPN:         qp_adv_conf_bram_wen_r <= 1'b1;
                    RQ_BUF_BA_QPN:           qp_rq_ba_bram_wen_r <= 1'b1;
                    SQ_BA_QPN:               qp_sq_ba_bram_wen_r <= 1'b1;
                    CQ_BA_QPN:               qp_cq_ba_bram_wen_r <= 1'b1;
                    RQ_BUF_BA_MSB_QPN:       qp_rq_ba_msb_bram_wen_r <= 1'b1;
                    SQ_BA_MSB_QPN:           qp_sq_ba_msb_bram_wen_r <= 1'b1;
                    CQ_BA_MSB_QPN:           qp_cq_ba_msb_bram_wen_r <= 1'b1;
                    RQ_WRPTR_DB_ADD_QPN:     qp_rq_wrptrdb_add_bram_wen_r <= 1'b1;
                    RQ_WRPTR_DB_ADD_QPN_MSB: qp_rq_wrptrdb_add_msb_bram_wen_r <= 1'b1;
                    SQ_CMPL_DB_ADD_QPN:      qp_sq_cmpldb_add_bram_wen_r <= 1'b1;
                    SQ_CMPL_DB_ADD_QPN_MSB:  qp_sq_cmpldb_add_msb_bram_wen_r <= 1'b1;
                    RQ_CI_DB_QPN:            qp_rq_cidb_bram_wen_r <= 1'b1;
                    Q_DEPTH_QPN:             qp_q_depth_bram_wen_r <= 1'b1;
                    LAST_RQ_QPN:             qp_last_rq_bram_wen_r <= 1'b1;
                    DEST_QP_CONF_QPN:        qp_dest_qpid_bram_wen_r <= 1'b1;  //shift
                    TIMEOUT_QPN:             qp_timeout_bram_wen_r <= 1'b1;
                    MAC_REMOTE_ADDR_LSB_QPN: qp_mac_remote_addrl_bram_wen_r <= 1'b1;
                    MAC_REMOTE_ADDR_MSB_QPN: qp_mac_remote_addrm_bram_wen_r <= 1'b1;
                    IP_REMOTE_ADDR1_QPN:     qp_ip_remote_addr1_bram_wen_r <= 1'b1;
                    IP_REMOTE_ADDR2_QPN:     qp_ip_remote_addr2_bram_wen_r <= 1'b1;
                    IP_REMOTE_ADDR3_QPN:     qp_ip_remote_addr3_bram_wen_r <= 1'b1;
                    IP_REMOTE_ADDR4_QPN:     qp_ip_remote_addr4_bram_wen_r <= 1'b1;
                    STAT_SSN_QPN:            qp_stat_ssn_bram_wen_r <= 1'b1;
                    STAT_MSN_QPN:            qp_stat_msn_bram_wen_r <= 1'b1;
                    PD_NUMBER_QPN:           qp_pd_num_bram_wen_r <= 1'b1;
                endcase
                //end else if ((wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && (wr_addr_r[17] == 'b0)) begin

                // wr_addr_r[20] == 1'b0 表示地址是 0x00 开头，即 0x00_0000 开始
            end else if ((wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && (wr_addr_r[20] == 1'b0)) begin
                wr_addr_wen_r                    <= 2'b10;
                qp_conf_bram_wen_r               <= 'b0;
                qp_adv_conf_bram_wen_r           <= 'b0;
                qp_rq_ba_bram_wen_r              <= 'b0;
                qp_sq_ba_bram_wen_r              <= 'b0;
                qp_cq_ba_bram_wen_r              <= 'b0;
                qp_rq_ba_msb_bram_wen_r          <= 'b0;
                qp_sq_ba_msb_bram_wen_r          <= 'b0;
                qp_cq_ba_msb_bram_wen_r          <= 'b0;
                qp_rq_wrptrdb_add_bram_wen_r     <= 'b0;
                qp_rq_wrptrdb_add_msb_bram_wen_r <= 'b0;
                qp_sq_cmpldb_add_bram_wen_r      <= 'b0;
                qp_sq_cmpldb_add_msb_bram_wen_r  <= 'b0;
                qp_rq_cidb_bram_wen_r            <= 'b0;
                qp_q_depth_bram_wen_r            <= 'b0;

                qp_last_rq_bram_wen_r            <= 'b0;
                qp_dest_qpid_bram_wen_r          <= 'b0;
                qp_timeout_bram_wen_r            <= 'b0;
                qp_mac_remote_addrl_bram_wen_r   <= 'b0;
                qp_mac_remote_addrm_bram_wen_r   <= 'b0;
                qp_ip_remote_addr1_bram_wen_r    <= 'b0;
                qp_ip_remote_addr2_bram_wen_r    <= 'b0;
                qp_ip_remote_addr3_bram_wen_r    <= 'b0;
                qp_ip_remote_addr4_bram_wen_r    <= 'b0;
                qp_stat_ssn_bram_wen_r           <= 'b0;
                qp_stat_msn_bram_wen_r           <= 'b0;
                qp_pd_num_bram_wen_r             <= 'b0;

                pd_pd_num_bram_wen_r             <= 'b0;
                pd_virt_addr_0_bram_wen_r        <= 'b0;
                pd_virt_addr_1_bram_wen_r        <= 'b0;
                pd_buf_base_addr_0_bram_wen_r    <= 'b0;
                pd_buf_base_addr_1_bram_wen_r    <= 'b0;
                pd_buf_r_key_bram_wen_r          <= 'b0;
                pd_w_r_buf_len_bram_wen_r        <= 'b0;
                pd_access_type_bram_wen_r        <= 'b0;
                case (wr_addr_r[7:0])  // 地址低8位
                    PD_NUMBER_PDN:       pd_pd_num_bram_wen_r <= 1'b1;
                    VIRT_ADDR_0_PDN:     pd_virt_addr_0_bram_wen_r <= 1'b1;
                    VIRT_ADDR_1_PDN:     pd_virt_addr_1_bram_wen_r <= 1'b1;
                    BUF_BASE_ADDR_0_PDN: pd_buf_base_addr_0_bram_wen_r <= 1'b1;
                    BUF_BASE_ADDR_1_PDN: pd_buf_base_addr_1_bram_wen_r <= 1'b1;
                    BUF_R_KEY_PDN:       pd_buf_r_key_bram_wen_r <= 1'b1;
                    W_R_BUF_LEN_PDN:     pd_w_r_buf_len_bram_wen_r <= 1'b1;
                    ACCESS_TYPE_PDN:     pd_access_type_bram_wen_r <= 1'b1;
                endcase
            end else begin
                wr_addr_wen_r                    <= 2'b00;
                qp_conf_bram_wen_r               <= 'b0;
                qp_adv_conf_bram_wen_r           <= 'b0;
                qp_rq_ba_bram_wen_r              <= 'b0;
                qp_rq_ba_msb_bram_wen_r          <= 'b0;
                qp_sq_ba_bram_wen_r              <= 'b0;
                qp_sq_ba_msb_bram_wen_r          <= 'b0;
                qp_cq_ba_bram_wen_r              <= 'b0;
                qp_cq_ba_msb_bram_wen_r          <= 'b0;
                qp_rq_wrptrdb_add_bram_wen_r     <= 'b0;
                qp_rq_wrptrdb_add_msb_bram_wen_r <= 'b0;
                qp_sq_cmpldb_add_bram_wen_r      <= 'b0;
                qp_sq_cmpldb_add_msb_bram_wen_r  <= 'b0;
                qp_rq_cidb_bram_wen_r            <= 'b0;
                qp_q_depth_bram_wen_r            <= 'b0;

                qp_last_rq_bram_wen_r            <= 'b0;
                qp_dest_qpid_bram_wen_r          <= 'b0;
                qp_timeout_bram_wen_r            <= 'b0;
                qp_pd_num_bram_wen_r             <= 'b0;
                qp_mac_remote_addrl_bram_wen_r   <= 'b0;
                qp_mac_remote_addrm_bram_wen_r   <= 'b0;
                qp_ip_remote_addr1_bram_wen_r    <= 'b0;
                qp_ip_remote_addr2_bram_wen_r    <= 'b0;
                qp_ip_remote_addr3_bram_wen_r    <= 'b0;
                qp_ip_remote_addr4_bram_wen_r    <= 'b0;
                qp_stat_msn_bram_wen_r           <= 'b0;
                qp_stat_ssn_bram_wen_r           <= 'b0;

                pd_pd_num_bram_wen_r             <= 'b0;
                pd_virt_addr_0_bram_wen_r        <= 'b0;
                pd_virt_addr_1_bram_wen_r        <= 'b0;
                pd_buf_base_addr_0_bram_wen_r    <= 'b0;
                pd_buf_base_addr_1_bram_wen_r    <= 'b0;
                pd_buf_r_key_bram_wen_r          <= 'b0;
                pd_w_r_buf_len_bram_wen_r        <= 'b0;
                pd_access_type_bram_wen_r        <= 'b0;
            end
        end
    end
    // 对初始配置进行额外注册，以尽量减少时序路径
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            qp_conf_bram_wen_p               <= 'b0;
            qp_adv_conf_bram_wen_p           <= 'b0;
            qp_rq_ba_bram_wen_p              <= 'b0;
            qp_rq_ba_msb_bram_wen_p          <= 'b0;
            qp_sq_ba_bram_wen_p              <= 'b0;
            qp_sq_ba_msb_bram_wen_p          <= 'b0;
            qp_cq_ba_bram_wen_p              <= 'b0;
            qp_cq_ba_msb_bram_wen_p          <= 'b0;
            qp_rq_wrptrdb_add_bram_wen_p     <= 'b0;
            qp_rq_wrptrdb_add_msb_bram_wen_p <= 'b0;
            qp_sq_cmpldb_add_bram_wen_p      <= 'b0;
            qp_sq_cmpldb_add_msb_bram_wen_p  <= 'b0;
            qp_rq_cidb_bram_wen_p            <= 'b0;
            qp_q_depth_bram_wen_p            <= 'b0;

            qp_last_rq_bram_wen_p            <= 'b0;
            qp_pd_num_bram_wen_p             <= 'b0;
            pd_pd_num_bram_wen_p             <= 'b0;
            pd_virt_addr_0_bram_wen_p        <= 'b0;
            pd_virt_addr_1_bram_wen_p        <= 'b0;
            pd_buf_base_addr_0_bram_wen_p    <= 'b0;
            pd_buf_base_addr_1_bram_wen_p    <= 'b0;
            pd_buf_r_key_bram_wen_p          <= 'b0;
            pd_w_r_buf_len_bram_wen_p        <= 'b0;
            pd_access_type_bram_wen_p        <= 'b0;
            qp_dest_qpid_bram_wen_p          <= 'b0;
            qp_mac_remote_addrl_bram_wen_p   <= 'b0;
            qp_mac_remote_addrm_bram_wen_p   <= 'b0;
            qp_ip_remote_addr1_bram_wen_p    <= 'b0;
            qp_ip_remote_addr2_bram_wen_p    <= 'b0;
            qp_ip_remote_addr3_bram_wen_p    <= 'b0;
            qp_ip_remote_addr4_bram_wen_p    <= 'b0;
            qp_stat_msn_bram_wen_p           <= 'b0;
            qp_stat_ssn_bram_wen_p           <= 'b0;
            qp_timeout_bram_wen_p            <= 'b0;
            wr_addr_mem_r_p                  <= 'b0;
            wr_addr_mem_r1_p                 <= 'b0;
            wr_data_mem_r_p                  <= 'b0;
        end else begin
            qp_conf_bram_wen_p               <= qp_conf_bram_wen_r;
            qp_adv_conf_bram_wen_p           <= qp_adv_conf_bram_wen_r;
            qp_rq_ba_bram_wen_p              <= qp_rq_ba_bram_wen_r;
            qp_rq_ba_msb_bram_wen_p          <= qp_rq_ba_msb_bram_wen_r;
            qp_sq_ba_bram_wen_p              <= qp_sq_ba_bram_wen_r;
            qp_sq_ba_msb_bram_wen_p          <= qp_sq_ba_msb_bram_wen_r;
            qp_cq_ba_bram_wen_p              <= qp_cq_ba_bram_wen_r;
            qp_cq_ba_msb_bram_wen_p          <= qp_cq_ba_msb_bram_wen_r;
            qp_rq_wrptrdb_add_bram_wen_p     <= qp_rq_wrptrdb_add_bram_wen_r;
            qp_rq_wrptrdb_add_msb_bram_wen_p <= qp_rq_wrptrdb_add_msb_bram_wen_r;
            qp_sq_cmpldb_add_bram_wen_p      <= qp_sq_cmpldb_add_bram_wen_r;
            qp_sq_cmpldb_add_msb_bram_wen_p  <= qp_sq_cmpldb_add_msb_bram_wen_r;
            qp_rq_cidb_bram_wen_p            <= qp_rq_cidb_bram_wen_r;
            qp_q_depth_bram_wen_p            <= qp_q_depth_bram_wen_r;

            qp_last_rq_bram_wen_p            <= qp_last_rq_bram_wen_r;
            qp_pd_num_bram_wen_p             <= qp_pd_num_bram_wen_r;
            pd_pd_num_bram_wen_p             <= pd_pd_num_bram_wen_r;
            pd_virt_addr_0_bram_wen_p        <= pd_virt_addr_0_bram_wen_r;
            pd_virt_addr_1_bram_wen_p        <= pd_virt_addr_1_bram_wen_r;
            pd_buf_base_addr_0_bram_wen_p    <= pd_buf_base_addr_0_bram_wen_r;
            pd_buf_base_addr_1_bram_wen_p    <= pd_buf_base_addr_1_bram_wen_r;
            pd_buf_r_key_bram_wen_p          <= pd_buf_r_key_bram_wen_r;
            pd_w_r_buf_len_bram_wen_p        <= pd_w_r_buf_len_bram_wen_r;
            pd_access_type_bram_wen_p        <= pd_access_type_bram_wen_r;
            qp_dest_qpid_bram_wen_p          <= qp_dest_qpid_bram_wen_r;
            qp_mac_remote_addrl_bram_wen_p   <= qp_mac_remote_addrl_bram_wen_r;
            qp_mac_remote_addrm_bram_wen_p   <= qp_mac_remote_addrm_bram_wen_r;
            qp_ip_remote_addr1_bram_wen_p    <= qp_ip_remote_addr1_bram_wen_r;
            qp_ip_remote_addr2_bram_wen_p    <= qp_ip_remote_addr2_bram_wen_r;
            qp_ip_remote_addr3_bram_wen_p    <= qp_ip_remote_addr3_bram_wen_r;
            qp_ip_remote_addr4_bram_wen_p    <= qp_ip_remote_addr4_bram_wen_r;
            qp_stat_msn_bram_wen_p           <= qp_stat_msn_bram_wen_r;
            qp_stat_ssn_bram_wen_p           <= qp_stat_ssn_bram_wen_r;
            qp_timeout_bram_wen_p            <= qp_timeout_bram_wen_r;
            wr_addr_mem_r_p                  <= wr_addr_mem_r_r;
            wr_addr_mem_r1_p                 <= wr_addr_mem_r1_r;
            wr_data_mem_r_p                  <= wr_data_mem_r_r;
        end
    end

    assign qp_conf_bram_wen               = (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_bram_wen_r : qp_conf_bram_wen_p;
    assign qp_adv_conf_bram_wen           = (C_CONFIG_MEM_LATENCY == 3) ? qp_adv_conf_bram_wen_r : qp_adv_conf_bram_wen_p;
    assign qp_rq_ba_bram_wen              = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_ba_bram_wen_r : qp_rq_ba_bram_wen_p;
    assign qp_rq_ba_msb_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_ba_msb_bram_wen_r : qp_rq_ba_msb_bram_wen_p;
    assign qp_sq_ba_bram_wen              = (C_CONFIG_MEM_LATENCY == 3) ? qp_sq_ba_bram_wen_r : qp_sq_ba_bram_wen_p;
    assign qp_sq_ba_msb_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? qp_sq_ba_msb_bram_wen_r : qp_sq_ba_msb_bram_wen_p;
    assign qp_cq_ba_bram_wen              = (C_CONFIG_MEM_LATENCY == 3) ? qp_cq_ba_bram_wen_r : qp_cq_ba_bram_wen_p;
    assign qp_cq_ba_msb_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? qp_cq_ba_msb_bram_wen_r : qp_cq_ba_msb_bram_wen_p;
    assign qp_rq_wrptrdb_add_bram_wen     = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_wrptrdb_add_bram_wen_r : qp_rq_wrptrdb_add_bram_wen_p;
    assign qp_rq_wrptrdb_add_msb_bram_wen = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_wrptrdb_add_msb_bram_wen_r : qp_rq_wrptrdb_add_msb_bram_wen_p;
    assign qp_sq_cmpldb_add_bram_wen      = (C_CONFIG_MEM_LATENCY == 3) ? qp_sq_cmpldb_add_bram_wen_r : qp_sq_cmpldb_add_bram_wen_p;
    assign qp_sq_cmpldb_add_msb_bram_wen  = (C_CONFIG_MEM_LATENCY == 3) ? qp_sq_cmpldb_add_msb_bram_wen_r : qp_sq_cmpldb_add_msb_bram_wen_p;
    assign qp_rq_cidb_bram_wen            = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_cidb_bram_wen_r : qp_rq_cidb_bram_wen_p;
    assign qp_q_depth_bram_wen            = (C_CONFIG_MEM_LATENCY == 3) ? qp_q_depth_bram_wen_r : qp_q_depth_bram_wen_p;

    assign qp_last_rq_bram_wen            = (C_CONFIG_MEM_LATENCY == 3) ? qp_last_rq_bram_wen_r : qp_last_rq_bram_wen_p;
    assign qp_pd_num_bram_wen             = (C_CONFIG_MEM_LATENCY == 3) ? qp_pd_num_bram_wen_r : qp_pd_num_bram_wen_p;
    assign pd_pd_num_bram_wen             = (C_CONFIG_MEM_LATENCY == 3) ? pd_pd_num_bram_wen_r : pd_pd_num_bram_wen_p;
    assign pd_virt_addr_0_bram_wen        = (C_CONFIG_MEM_LATENCY == 3) ? pd_virt_addr_0_bram_wen_r : pd_virt_addr_0_bram_wen_p;
    assign pd_virt_addr_1_bram_wen        = (C_CONFIG_MEM_LATENCY == 3) ? pd_virt_addr_1_bram_wen_r : pd_virt_addr_1_bram_wen_p;
    assign pd_buf_base_addr_0_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? pd_buf_base_addr_0_bram_wen_r : pd_buf_base_addr_0_bram_wen_p;
    assign pd_buf_base_addr_1_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? pd_buf_base_addr_1_bram_wen_r : pd_buf_base_addr_1_bram_wen_p;
    assign pd_buf_r_key_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? pd_buf_r_key_bram_wen_r : pd_buf_r_key_bram_wen_p;
    assign pd_w_r_buf_len_bram_wen        = (C_CONFIG_MEM_LATENCY == 3) ? pd_w_r_buf_len_bram_wen_r : pd_w_r_buf_len_bram_wen_p;
    assign pd_access_type_bram_wen        = (C_CONFIG_MEM_LATENCY == 3) ? pd_access_type_bram_wen_r : pd_access_type_bram_wen_p;
    assign qp_dest_qpid_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? qp_dest_qpid_bram_wen_r : qp_dest_qpid_bram_wen_p;
    assign qp_mac_remote_addrl_bram_wen   = (C_CONFIG_MEM_LATENCY == 3) ? qp_mac_remote_addrl_bram_wen_r : qp_mac_remote_addrl_bram_wen_p;

    assign qp_mac_remote_addrm_bram_wen   = (C_CONFIG_MEM_LATENCY == 3) ? qp_mac_remote_addrm_bram_wen_r : qp_mac_remote_addrm_bram_wen_p;

    assign qp_ip_remote_addr1_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? qp_ip_remote_addr1_bram_wen_r : qp_ip_remote_addr1_bram_wen_p;

    assign qp_ip_remote_addr2_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? qp_ip_remote_addr2_bram_wen_r : qp_ip_remote_addr2_bram_wen_p;

    assign qp_ip_remote_addr3_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? qp_ip_remote_addr3_bram_wen_r : qp_ip_remote_addr3_bram_wen_p;

    assign qp_ip_remote_addr4_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? qp_ip_remote_addr4_bram_wen_r : qp_ip_remote_addr4_bram_wen_p;

    assign qp_stat_msn_bram_wen           = (C_CONFIG_MEM_LATENCY == 3) ? qp_stat_msn_bram_wen_r : qp_stat_msn_bram_wen_p;
    assign qp_stat_ssn_bram_wen           = (C_CONFIG_MEM_LATENCY == 3) ? qp_stat_ssn_bram_wen_r : qp_stat_ssn_bram_wen_p;
    assign qp_timeout_bram_wen            = (C_CONFIG_MEM_LATENCY == 3) ? qp_timeout_bram_wen_r : qp_timeout_bram_wen_p;

    assign wr_addr_mem_r                  = (C_CONFIG_MEM_LATENCY == 3) ? wr_addr_mem_r_r : wr_addr_mem_r_p;
    assign wr_addr_mem_r1                 = (C_CONFIG_MEM_LATENCY == 3) ? wr_addr_mem_r1_r : wr_addr_mem_r1_p;
    assign wr_data_mem_r                  = (C_CONFIG_MEM_LATENCY == 3) ? wr_data_mem_r_r : wr_data_mem_r_p;


    assign intr_clr                       = wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r & generic_reg_wr_access & (wr_addr_r[10:2] == INTR_STS_REG[10:2]);

    assign o_intr_clr_pkt_valdn_err       = intr_clr ? s_axi_lite_wdata[0] : 1'b0;
    assign o_intr_clr_mad_pkt_rcvd        = intr_clr ? s_axi_lite_wdata[1] : 1'b0;
    assign o_intr_clr_bypass_pkt_rcvd     = intr_clr ? s_axi_lite_wdata[2] : 1'b0;
    assign o_intr_clr_rnr_nak_gen         = intr_clr ? s_axi_lite_wdata[3] : 1'b0;
    assign o_intr_clr_wqe_cmpl            = intr_clr ? s_axi_lite_wdata[4] : 1'b0;
    assign o_intr_clr_ill_opc_in_sq       = intr_clr ? s_axi_lite_wdata[5] : 1'b0;
    assign o_intr_clr_qp_pkt_rcvd         = intr_clr ? s_axi_lite_wdata[6] : 1'b0;
    //assign o_intr_clr_fatal_err          = intr_clr ? s_axi_lite_wdata[7] : 1'b0;
    assign o_intr_clr_fatal_err           = !qp_fatal_sts && qp_fatal_sts_ff;
    assign o_intr_clr_cnp                 = intr_clr ? s_axi_lite_wdata[8] : 1'b0;

    assign o_rq_intr_sts_clr_wr_en        = rq_intr_sts_clr_wr_en;
    assign o_rq_intr_sts_clr_wr_addr      = rq_intr_sts_clr_wr_addr[C_QP_INDX_WIDTH-1 : 0];
    assign o_cq_intr_sts_clr_wr_en        = cq_intr_sts_clr_wr_en;
    assign o_cq_intr_sts_clr_wr_addr      = cq_intr_sts_clr_wr_addr[C_QP_INDX_WIDTH-1 : 0];
    assign o_cnp_schedule_clr_wr_en       = cnp_schd_sts_clr_wr_en;
    assign o_cnp_schedule_clr_wr_addr     = cnp_schd_sts_clr_wr_addr[C_QP_INDX_WIDTH-1 : 0];
    assign o_cnp_timeout_val              = cnp_timeout;
    assign o_ecn_timeout_val              = ecn_timeout;
    assign qp_fatal_sts                   = (fatal_err_ones_count == 'd0) ? 1'b0 : 1'b1;
    `MSFF_R(qp_fatal_sts_ff, qp_fatal_sts, core_clk, ~core_rstn)

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_COMBO_BYPASS(1)
    ) inst_fatal_err_or_Reduce (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_fatal_err_valid),
        .wr_addr (init_start ? init_count : i_qp_fatal_err_id),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_err_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_fatal_err_id),
        .rd_data (fatal_id_rd_data)
    );

    //osq_empty And_reduce logic
    always @(posedge core_clk) begin : or_reduce_i_fatal_err
        if (~core_rstn) begin
            fatal_err_ones_count <= {C_QP_INDX_WIDTH + 1{1'b0}};
        end else begin
            case ({
                i_qp_fatal_err_valid, i_qp_fatal_err_data, fatal_id_rd_data
            })
                3'b110:  fatal_err_ones_count <= fatal_err_ones_count + 1;
                3'b101:  fatal_err_ones_count <= fatal_err_ones_count - 1;
                default: fatal_err_ones_count <= fatal_err_ones_count;
            endcase
        end
    end


    // wr_addr_r[20:19] == 2'b10 表示地址是 0x10 开头，即 0x10_0000 开始
    assign generic_reg_wr_access = (wr_addr_r[20:19] == 2'b10);

    //********************************************************************************
    //write response channel logic. 
    //This logic will generate BVALID signal for the write transaction. 
    //********************************************************************************
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            s_axi_lite_bvalid <= 1'b0;
        end else begin
            //if(wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) begin
            //s_axi_lite_bvalid should not be asserted when write priority is given to hwhnshake signals
            if (!wr_req_mask_r && wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && (!(rd_req_r || s_axi_lite_arvalid)) && !s_axi_lite_bvalid) begin  //aw/w channel data is sampled; schedule a write response now
                s_axi_lite_bvalid <= 1'b1;
            end else if (s_axi_lite_bready) begin
                s_axi_lite_bvalid <= 1'b0;
            end else begin
                s_axi_lite_bvalid <= s_axi_lite_bvalid;
            end
        end
    end



    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP configuration registers (only 22 bits implemented) -- QP Configuration QPi(QPCONFi)
    // [0]      - QP_EN
    // [1]      - ACK COALSC EN     - 1: 启用聚合 ACK; 0: 启用 Explicit ACK
    //                                1: 把多个可确认事件合并后再回一个 ACK
    //                                0: 对端尽快、明确地回 ACK（可以理解为“该回就回”）
    // [2]      - RQ_INTR_EN
    // [3]      - CQ_INTR_EN
    // [4]      - HW_HNDSHK_DIS
    // [7:4]    - RESERVED - not in BRAN
    // [10:8]   - PMTU
    // [15:11]  - RESERVED - not in BRAM
    // [23:16]  - MAX_RD_OS
    // [31:24]  - RQ_BUF_SZ
    //

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (27 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(27),                    //positive integer
        .READ_DATA_WIDTH_A (27),                    //positive integer
        .BYTE_WRITE_WIDTH_A(27),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(27),               //positive integer
        .READ_DATA_WIDTH_B (27),               //positive integer
        .BYTE_WRITE_WIDTH_B(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_conf_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_conf_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          ({wr_data_mem_r[31:24], wr_data_mem_r[23:16], wr_data_mem_r[10:6], wr_data_mem_r[5:0]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         ({s_axi_qp_config_rdata[31:16], s_axi_qp_config_rdata[10:6], s_axi_qp_config_rdata[5:0]}),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_conf_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (27'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         ({o_qp_conf[31:24], o_qp_conf[23:16], o_qp_conf[10:6], o_qp_conf[5:0]}),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    always @(posedge core_clk) begin
        qp_conf_wr_addr_r  <= qp_conf_wr_addr;
        qp_conf_wr_addr_2r <= qp_conf_wr_addr_r;
        qp_conf_bram_wen_d <= qp_conf_bram_wen;
    end

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            qp_enable_valid           <= 1'b0;
            qp_enable_id              <= {C_QP_INDX_WIDTH{1'b0}};
            qp_enable_data            <= 1'b0;

            o_qp_disable_pulse_valid  <= 1'b0;
            o_qp_disable_pulse_id     <= {C_QP_INDX_WIDTH{1'b0}};
            o_qp_disable_pulse_data   <= 1'b0;

            o_qp_clr_fatal_err_valid  <= 1'b0;
            o_qp_clr_fatal_err_id     <= {C_QP_INDX_WIDTH{1'b0}};
            o_qp_clr_fatal_err_data   <= 1'b0;

            o_qp_under_recovery_valid <= 1'b0;
            o_qp_under_recovery_id    <= {C_QP_INDX_WIDTH{1'b0}};
            o_qp_under_recovery_data  <= 1'b0;
        end else begin
            if (qp_conf_bram_wen == 1'b1) begin
                qp_enable_valid           <= 1'b1;  // 一周期
                qp_enable_id              <= (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_wr_addr_r : qp_conf_wr_addr_2r;
                qp_enable_data            <= wr_data_mem_r[0];  // QPCONFi[0]: QP enable

                o_qp_disable_pulse_valid  <= 1'b1;  // 两周期
                o_qp_disable_pulse_id     <= (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_wr_addr_r : qp_conf_wr_addr_2r;
                o_qp_disable_pulse_data   <= !wr_data_mem_r[0];  // ~QPCONFi[0]: ~QP enable

                o_qp_clr_fatal_err_valid  <= 1'b1;  // 两周期
                o_qp_clr_fatal_err_id     <= (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_wr_addr_r : qp_conf_wr_addr_2r;
                o_qp_clr_fatal_err_data   <= !wr_data_mem_r[0];  // ~QPCONFi[0]: ~QP enable

                o_qp_under_recovery_valid <= 1'b1;  // 两周期
                o_qp_under_recovery_id    <= (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_wr_addr_r : qp_conf_wr_addr_2r;
                o_qp_under_recovery_data  <= wr_data_mem_r[6];  // QPCONFi[6]: QP under recovery
            end else if (qp_conf_bram_wen == 1'b0 && qp_conf_bram_wen_d == 1'b1) begin
                qp_enable_valid           <= 1'b0;
                o_qp_disable_pulse_valid  <= 1'b1;
                o_qp_clr_fatal_err_valid  <= 1'b1;
                o_qp_under_recovery_valid <= 1'b1;
                o_qp_disable_pulse_data   <= 1'b0;
                o_qp_clr_fatal_err_data   <= 1'b0;
                o_qp_under_recovery_data  <= 1'b0;
            end else begin
                qp_enable_valid           <= 1'b0;
                o_qp_disable_pulse_valid  <= 1'b0;
                o_qp_clr_fatal_err_valid  <= 1'b0;
                o_qp_under_recovery_valid <= 1'b0;
            end
        end
    end


    assign o_qp_conf[15:11]             = 'b0;
    assign o_qp_conf_replica[15:11]     = 'b0;
    assign s_axi_qp_config_rdata[15:11] = 'b0;

    assign qp_conf_wr_addr              = wr_addr_r[8+:C_QP_INDX_WIDTH];
    //assign qp_conf_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_conf_bram_addr =  qp_conf_bram_wen ? qp_conf_wr_addr : qp_conf_rd_addr;
    //assign qp_conf_bram_wen_c = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == QP_CONF_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
    assign qp_conf_bram_wen_c           = (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && ((wr_addr_r[7:0] == QP_CONF_QPN) && wr_addr_r[20:19] == 2'b11);

    // REPLICA MEMORY -- 内存复制品
    // 某些内存需要进行复制，因为IPG 0的实现需要两条并行路径，因此需要独立访问某些寄存器（且不存在任何延迟）。
    // 所有写操作都会同时发生在两个内存中。
    // 读操作则保持独立。
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (27 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(27),               //positive integer
        .READ_DATA_WIDTH_A (27),               //positive integer
        .BYTE_WRITE_WIDTH_A(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(27),               //positive integer
        .READ_DATA_WIDTH_B (27),               //positive integer
        .BYTE_WRITE_WIDTH_B(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_replica_qp_conf_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_conf_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          ({wr_data_mem_r[31:24], wr_data_mem_r[23:16], wr_data_mem_r[10:6], wr_data_mem_r[5:0]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_conf_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (27'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         ({o_qp_conf_replica[31:24], o_qp_conf_replica[23:16], o_qp_conf_replica[10:6], o_qp_conf_replica[5:0]}),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP advance configuration registers (only 30 bits implemented)   -QP advanced configuration QPi (QPADVCONFi)寄存器
    // [5:0]    - TRAFFIC CLASS
    // [7:6]    - RESERVED
    // [15:8]   - TIME_TO_LIVE
    // [31:16]  - P_KEY
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (30 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(30),                    //positive integer
        .READ_DATA_WIDTH_A (30),                    //positive integer
        .BYTE_WRITE_WIDTH_A(30),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(30),               //positive integer
        .READ_DATA_WIDTH_B (30),               //positive integer
        .BYTE_WRITE_WIDTH_B(30),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_adv_conf_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_adv_conf_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          ({wr_data_mem_r[31:8], wr_data_mem_r[5:0]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         ({s_axi_qp_adv_config_rdata[31:8], s_axi_qp_adv_config_rdata[5:0]}),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_adv_conf_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (30'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         ({o_qp_adv_conf[31:8], o_qp_adv_conf[5:0]}),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign o_qp_adv_conf[7:6]             = 'b0;
    assign s_axi_qp_adv_config_rdata[7:6] = 'b0;

    //assign qp_adv_conf_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_adv_conf_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_adv_conf_bram_addr =  qp_adv_conf_bram_wen ? qp_adv_conf_wr_addr : qp_adv_conf_rd_addr;
    //assign qp_adv_conf_bram_wen = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == QP_ADV_CONF_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP RQ BUF BASE address registers (only 24 bits implemented) -- RCV Q Buffer base address QPi (RQBAi)
    // [7:0]    - RESERVED
    // [31:8]   - RQ_BUF_BA
    // Check_SDPRAM
    // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供 RDMA 接收队列缓冲区的基地址。
    // [31:8]:接收 Q 缓冲区基地址地址（256 B 对齐）。256 B 是总大小（单个 RQ 缓冲区元素大小）* rq_depth
    // 保存的是“配置类”信息——每个 QP 的 RQ Buffer 基地址（RQ Buffer Base Address）。这些值通常由软件通过 AXI4-Lite 写入，用于告诉硬件接收队列缓冲区的起始位置。

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_rq_ba_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports -- axi_lite read/write port
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_rq_ba_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:8]),      // 只实现了地址的高位部分（代码里把 [31:8] 存在 BRAM 中，低 8 位保留为 0），因为 RQ Buffer 按 256B 对齐。
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_rq_ba_rdata[31:8]),  // 只实现了地址的高位部分（代码里把 [31:8] 存在 BRAM 中，低 8 位保留为 0），因为 RQ Buffer 按 256B 对齐。
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports -- 其他模块 read port
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_rq_ba_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (24'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_rq_ba[23:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_rq_ba_rdata[7:0] = 'b0;

    //assign qp_rq_ba_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH];
    //assign qp_rq_ba_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH];
    //assign qp_rq_ba_bram_addr =  qp_rq_ba_bram_wen ? qp_rq_ba_wr_addr : qp_rq_ba_rd_addr;
    //assign qp_rq_ba_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == RQ_BUF_BA_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_rq_ba_msb
            //assign qp_rq_ba_msb_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH];
            //assign qp_rq_ba_msb_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH];
            //assign qp_rq_ba_msb_bram_addr =  qp_rq_ba_msb_bram_wen ? qp_rq_ba_msb_wr_addr : qp_rq_ba_msb_rd_addr;
            //assign qp_rq_ba_msb_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == RQ_BUF_BA_MSB_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

            // Check_SDPRAM
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_qp_rq_ba_msb_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_rq_ba_msb_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_rq_ba_msb_rdata),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (1'b0),
                .addrb         (i_qp_rq_ba_idx[0+:C_QP_INDX_WIDTH]),
                .dinb          (32'b0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_rq_ba[55:24]),
                .sbiterrb      (),
                .dbiterrb      ()

            );
        end else begin
            assign qp_rq_ba_msb_wr_addr   = 0;
            assign qp_rq_ba_msb_rd_addr   = 0;
            assign qp_rq_ba_msb_bram_addr = 0;
            assign s_axi_rq_ba_msb_rdata  = 0;
        end
    endgenerate
    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP SQ BASE address registers (only 27 bits implemented)
    // [4:0]    - RESERVED
    // [31:5]   - SQ_BA
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (27 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(27),                    //positive integer
        .READ_DATA_WIDTH_A (27),                    //positive integer
        .BYTE_WRITE_WIDTH_A(27),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(27),               //positive integer
        .READ_DATA_WIDTH_B (27),               //positive integer
        .BYTE_WRITE_WIDTH_B(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_ba_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_ba_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:5]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_sq_ba_rdata[31:5]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_sq_ba_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (27'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_sq_ba[31:5]),
        .sbiterrb      (),
        .dbiterrb      ()

    );



    assign o_qp_sq_ba[4:0]        = 'b0;
    assign s_axi_sq_ba_rdata[4:0] = 'b0;

    //assign qp_sq_ba_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_sq_ba_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_sq_ba_bram_addr =  qp_sq_ba_bram_wen ? qp_sq_ba_wr_addr : qp_sq_ba_rd_addr;
    //assign qp_sq_ba_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == SQ_BA_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_sq_ba_msb
            //assign qp_sq_ba_msb_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
            //assign qp_sq_ba_msb_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
            //assign qp_sq_ba_msb_bram_addr =  qp_sq_ba_msb_bram_wen ? qp_sq_ba_msb_wr_addr : qp_sq_ba_msb_rd_addr;
            //assign qp_sq_ba_msb_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == SQ_BA_MSB_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
            // Check_SDPRAM

            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_qp_sq_ba_msb_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_sq_ba_msb_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_sq_ba_msb_rdata),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (1'b0),
                .addrb         (i_qp_sq_ba_idx[0+:C_QP_INDX_WIDTH]),
                .dinb          (32'b0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_sq_ba[63:32]),
                .sbiterrb      (),
                .dbiterrb      ()

            );
        end else begin
            assign qp_sq_ba_msb_wr_addr   = 0;
            assign qp_sq_ba_msb_rd_addr   = 0;
            assign qp_sq_ba_msb_bram_addr = 0;
            assign s_axi_sq_ba_msb_rdata  = 0;
        end
    endgenerate
    //-----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP CQ BASE address registers (only 27 bits implemented)
    // [4:0]    - RESERVED
    // [31:5]   - CQ_BA
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (27 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(27),                    //positive integer
        .READ_DATA_WIDTH_A (27),                    //positive integer
        .BYTE_WRITE_WIDTH_A(27),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(27),               //positive integer
        .READ_DATA_WIDTH_B (27),               //positive integer
        .BYTE_WRITE_WIDTH_B(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_cq_ba_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_cq_ba_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:5]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_cq_ba_rdata[31:5]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_cq_ba_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (27'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_cq_ba[31:5]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign o_qp_cq_ba[4:0]        = 'b0;
    assign s_axi_cq_ba_rdata[4:0] = 'b0;

    //    assign qp_cq_ba_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ; // The repeating registers start at 0x200 so need to normalize
    //    assign qp_cq_ba_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //    assign qp_cq_ba_bram_addr =  qp_cq_ba_bram_wen ? qp_cq_ba_wr_addr : qp_cq_ba_rd_addr;
    //    assign qp_cq_ba_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == CQ_BA_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_cq_ba_msb
            //assign qp_cq_ba_msb_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ; // The repeating registers start at 0x200 so need to normalize
            //assign qp_cq_ba_msb_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
            //assign qp_cq_ba_msb_bram_addr =  qp_cq_ba_msb_bram_wen ? qp_cq_ba_msb_wr_addr : qp_cq_ba_msb_rd_addr;
            //assign qp_cq_ba_msb_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == CQ_BA_MSB_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

            // Check_SDPRAM
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_qp_cq_ba_msb_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_cq_ba_msb_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_cq_ba_msb_rdata),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (1'b0),
                .addrb         (i_qp_cq_ba_idx[0+:C_QP_INDX_WIDTH]),
                .dinb          (32'b0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_cq_ba[63:32]),
                .sbiterrb      (),
                .dbiterrb      ()

            );
        end else begin
            assign qp_cq_ba_msb_wr_addr   = 0;
            assign qp_cq_ba_msb_rd_addr   = 0;
            assign qp_cq_ba_msb_bram_addr = 0;
            assign s_axi_cq_ba_msb_rdata  = 0;
        end
    endgenerate
    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP RQ WRPTR DB address registers -- RCV Q Write pointer DB address QPi(RQWPTRDBADDi)
    // [31:0]   - RQ_WRPTR_DB_ADD
    // Check_SDPRAM
    // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供接收队列门铃寄存器的地址。在接收到新的 RDMA SEND 数据包时，ERNIC IP 会更新寄存器所指向地址中的 RQ 门铃值。


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_rq_wrptrdb_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_rq_wrptrdb_add_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_rq_wrptrdb_add_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_rq_wrptrdb_add_idx[0+:C_QP_INDX_WIDTH]),  // qp索引
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_rq_wrptrdb_add[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    //assign qp_rq_wrptrdb_add_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
    //assign qp_rq_wrptrdb_add_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_rq_wrptrdb_add_bram_addr =  qp_rq_wrptrdb_add_bram_wen ? qp_rq_wrptrdb_add_wr_addr : qp_rq_wrptrdb_add_rd_addr;
    //assign qp_rq_wrptrdb_add_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == RQ_WRPTR_DB_ADD_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_rq_wrptrdb_msb
            //assign qp_rq_wrptrdb_add_msb_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
            //assign qp_rq_wrptrdb_add_msb_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
            //assign qp_rq_wrptrdb_add_msb_bram_addr =  qp_rq_wrptrdb_add_msb_bram_wen ? qp_rq_wrptrdb_add_msb_wr_addr : qp_rq_wrptrdb_add_msb_rd_addr;	
            //assign qp_rq_wrptrdb_add_msb_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == RQ_WRPTR_DB_ADD_QPN_MSB) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
            // Check_SDPRAM
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_QP),        //positive integer
                .MEMORY_PRIMITIVE ("auto"),               //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("independent_clock"),  //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),               //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                   //string;
                .USE_MEM_INIT     (1),                    //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),      //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                    //integer; 0,1
                .ECC_MODE         ("no_ecc"),             //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                    //Do not Change

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_qp_rq_wrptrdb_msb_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_rq_wrptrdb_add_msb_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_rq_wrptrdb_msb_add_rdata),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (1'b0),
                .addrb         (i_qp_rq_wrptrdb_add_idx[0+:C_QP_INDX_WIDTH]),
                .dinb          (32'b0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_rq_wrptrdb_add[63:32]),
                .sbiterrb      (),
                .dbiterrb      ()
            );
        end else begin
            assign qp_rq_wrptrdb_add_msb_wr_addr   = 0;
            assign qp_rq_wrptrdb_add_msb_rd_addr   = 0;
            assign qp_rq_wrptrdb_add_msb_bram_addr = 0;
            assign s_axi_rq_wrptrdb_msb_add_rdata  = 0;
        end
    endgenerate

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP SQ completions DB address registers 
    // [31:0]   - SQ_CMPL_DB_ADD
    // Check_SDPRAM


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_cmpldb_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_cmpldb_add_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_sq_cmpldb_add_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_sq_cmpldb_add_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_sq_cmpldb_add[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    //assign qp_sq_cmpldb_add_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
    //assign qp_sq_cmpldb_add_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_sq_cmpldb_add_bram_addr =  qp_sq_cmpldb_add_bram_wen ? qp_sq_cmpldb_add_wr_addr : qp_sq_cmpldb_add_rd_addr;
    //assign qp_sq_cmpldb_add_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == SQ_CMPL_DB_ADD_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_sq_cmpldb_msb
            //assign qp_sq_cmpldb_add_msb_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
            //assign qp_sq_cmpldb_add_msb_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
            //assign qp_sq_cmpldb_add_msb_bram_addr =  qp_sq_cmpldb_add_msb_bram_wen ? qp_sq_cmpldb_add_msb_wr_addr : qp_sq_cmpldb_add_msb_rd_addr;	
            //assign qp_sq_cmpldb_add_msb_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == SQ_CMPL_DB_ADD_QPN_MSB) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
            // Check_SDPRAM
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_QP),        //positive integer
                .MEMORY_PRIMITIVE ("auto"),               //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("independent_clock"),  //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),               //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                   //string;
                .USE_MEM_INIT     (1),                    //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),      //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                    //integer; 0,1
                .ECC_MODE         ("no_ecc"),             //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                    //Do not Change

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_qp_sq_cmpldb_msb_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_sq_cmpldb_add_msb_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_sq_cmpldb_msb_add_rdata),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (1'b0),
                .addrb         (i_qp_sq_cmpldb_add_idx[0+:C_QP_INDX_WIDTH]),
                .dinb          (32'b0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_sq_cmpldb_add[63:32]),
                .sbiterrb      (),
                .dbiterrb      ()

            );
        end else begin
            assign qp_sq_cmpldb_add_msb_wr_addr   = 0;
            assign qp_sq_cmpldb_add_msb_rd_addr   = 0;
            assign qp_sq_cmpldb_add_msb_bram_addr = 0;
            assign s_axi_sq_cmpldb_msb_add_rdata  = 0;
        end
    endgenerate

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP CQ Head pointer registers 
    // [15:0]   - CQ_HDPTR
    // Check Collission


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_cq_hdptr_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_cq_hdptr_bram_wen),
        .addra         (qp_cq_hdptr_bram_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_cq_hdptr_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_cq_hdptr_wrn & ~i_qp_cq_hdptr_retry_req),
        .addrb         (i_qp_cq_hdptr_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_cq_hdptr),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_cq_hdptr[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_cq_hdptr_rdata[31:16] = 'b0;

    assign qp_cq_hdptr_wr_addr         = wr_addr_r[8+:C_QP_INDX_WIDTH];  // The repeating registers start at 0x200 so need to normalize
    assign qp_cq_hdptr_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_cq_hdptr_bram_addr       = qp_cq_hdptr_bram_wen ? qp_cq_hdptr_wr_addr : qp_cq_hdptr_rd_addr;
    // SW override
    //assign qp_cq_hdptr_bram_wen  = xrnic_adv_conf_sw_override & (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == CQ_HEAD_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
    assign qp_cq_hdptr_bram_wen        = xrnic_adv_conf_sw_override & (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && ((wr_addr_r[7:0] == CQ_HEAD_QPN) && wr_addr_r[20:19] == 2'b11);


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP RQ CI doorbell registers  -- QP RQ CI 门铃寄存器
    // [15:0]   - RQ CI Doorbell
    // Double Check Collission


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_rq_ci_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_rq_cidb_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_rq_cidb_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_rq_cidb_wr_valid_hndshk & ~i_qp_rq_cidb_req),
        .addrb         (rq_cidb_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_rq_cidb_hndshk),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_rq_cidb[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign o_qp_rq_cidb[31:16]        = 'b0;
    assign s_axi_rq_cidb_rdata[31:16] = 'b0;

    assign rq_cidb_idx                = i_qp_rq_cidb_req ? i_qp_rq_cidb_idx : qp_rq_cidb_wr_addr_hndshk;
    assign qp_rq_cidb_wr_addr_hndshk  = i_qp_rq_cidb_wr_addr_hndshk[8+:C_QP_INDX_WIDTH];  // qp索引

    //assign qp_rq_cidb_wr_addr   = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
    //assign qp_rq_cidb_rd_addr   = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_rq_cidb_bram_addr = qp_rq_cidb_bram_wen ? qp_rq_cidb_wr_addr : qp_rq_cidb_rd_addr;
    //assign qp_rq_cidb_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == RQ_CI_DB_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP SQ PI doorbell registers 
    // [15:0]   - SQ PI Doorbell
    // Double Check Collission


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_pidb_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_pidb_bram_wen),
        .addra         (qp_sq_pidb_bram_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_sq_pidb_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (qp_sq_pidb_wen),
        .addrb         (qp_sq_pidb_addr),
        .dinb          (i_qp_sq_pidb_hndshk),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (sq_pidb[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    //    assign o_qp_sq_pidb[31:16]   = 'b0;
    assign s_axi_sq_pidb_rdata[31:16] = 'b0;
    assign o_qp_sq_pidb[15:0]         = sq_pidb[15:0];

    // To_be_Removed
    //assign qp_sq_pidb_addr = i_qp_sq_pidb_req ? i_qp_sq_pidb_idx : (i_qp_sq_pidb_wr_valid_hndshk ? qp_sq_pidb_hndshk_addr : pending_idx);
    assign qp_sq_pidb_addr            = i_qp_sq_pidb_req ? i_qp_sq_pidb_idx : qp_sq_pidb_hndshk_addr;
    assign qp_sq_pidb_wen             = i_qp_sq_pidb_req ? 1'b0 : i_qp_sq_pidb_wr_valid_hndshk;

    assign qp_sq_pidb_hndshk_addr     = i_qp_sq_pidb_wr_addr_hndshk[8+:C_QP_INDX_WIDTH];

    assign qp_sq_pidb_wr_addr         = wr_addr_r[8+:C_QP_INDX_WIDTH];  // The repeating registers start at 0x200 so need to normalize
    assign qp_sq_pidb_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_sq_pidb_bram_addr       = qp_sq_pidb_bram_wen ? qp_sq_pidb_wr_addr : qp_sq_pidb_rd_addr;
    //assign qp_sq_pidb_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == SQ_PI_DB_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
    assign qp_sq_pidb_bram_wen        = (wr_req_mask_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid))) && ((wr_addr_r[7:0] == SQ_PI_DB_QPN) && wr_addr_r[20:19] == 2'b11);


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP Queue depth registers - implemented in 2 different BRAMs as they
    // are required asynchronously
    // [15:0]   - SQ depth
    // [31:16]  - RQ depth
    // Check_SDPRAM


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_depth_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_q_depth_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_q_depth_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (qp_depth_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (16'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_sq_depth[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    // CQ depth is the same as SQ depth
    assign qp_depth_idx  = i_qp_cq_depth_req ? i_qp_cq_depth_idx : i_qp_sq_depth_idx;
    assign o_qp_cq_depth = o_qp_sq_depth;


    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_rq_depth_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_q_depth_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:16]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_q_depth_rdata[31:16]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_rq_depth_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (16'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_rq_depth[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    //assign qp_q_depth_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
    //assign qp_q_depth_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_q_depth_bram_addr =  qp_q_depth_bram_wen ? qp_q_depth_wr_addr : qp_q_depth_rd_addr;
    //assign qp_q_depth_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == Q_DEPTH_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP SQ PSN registers -- SEND Q PSN for QPi (SQPSNi) 寄存器
    // [23:0]   - SQ PSN
    // 该寄存器在连接时由 SW 初始化。此后，硬件会为每个发出的数据包更新该寄存器，而软件则不应更新。该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
    // Check Registering

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_psn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_psn_bram_wen),
        .addra         (qp_sq_psn_bram_addr),
        .dina          (s_axi_lite_wdata[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_sq_psn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (((i_qp_sq_psn_wrn | i_qp_sq_psn_retry_wrn) & ~i_qp_sq_psn_req)),
        .addrb         (muxed_qp_sq_psn_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_sq_psn),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_sq_psn[23:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    //assign muxed_qp_sq_psn_idx = i_qp_sq_psn_req ? i_qp_sq_psn_idx : ((i_qp_sq_psn_retry_req | i_qp_sq_psn_retry_wrn) ? i_qp_sq_psn_retry_idx : i_qp_sq_psn_wqe_idx);
    assign muxed_qp_sq_psn_idx       = i_qp_sq_psn_req ? i_qp_sq_psn_idx : ((i_qp_sq_psn_retry_req | i_qp_sq_psn_retry_wrn) ? i_qp_sq_psn_retry_idx : ((i_qp_sq_psn_wrn | i_qp_sq_psn_wqe_req) ? i_qp_sq_psn_wqe_idx : i_qp_sq_psn_resp_idx));
    assign s_axi_sq_psn_rdata[31:24] = 'b0;

    assign qp_sq_psn_wr_addr         = wr_addr_r[8+:C_QP_INDX_WIDTH];  // The repeating registers start at 0x200 so need to normalize
    assign qp_sq_psn_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_sq_psn_bram_addr       = qp_sq_psn_bram_wen ? qp_sq_psn_wr_addr : qp_sq_psn_rd_addr;
    //assign qp_sq_psn_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == SQ_PSN_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
    assign qp_sq_psn_bram_wen        = (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && ((wr_addr_r[7:0] == SQ_PSN_QPN) && wr_addr_r[20:19] == 2'b11);

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP RQ PSN registers -- Last RQ req for QPi(LSTRQREQi)
    // [23:0]   - RQ PSN
    // [31:24]  - RQ opcode
    // 该寄存器提供最后一个接收到的 RQ 数据包的详细信息。该寄存器根据参数 C_NUM_QP（其中 i = 2 至 C_NUM_QP）生成。QP1 不存在该寄存器。

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_last_rq_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_last_rq_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_last_rq_rdata[31:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_last_rq_wrn),
        .addrb         (i_qp_last_rq_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_last_rq),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_last_rq[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    //assign qp_last_rq_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
    //assign qp_last_rq_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_last_rq_bram_addr =  qp_last_rq_bram_wen ? qp_last_rq_wr_addr : qp_last_rq_rd_addr;
    //assign qp_last_rq_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == LAST_RQ_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP PD number register
    // [23:0]   - PD number

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP PD number register
    // [23:0]   - PD number

    generate
        if (C_EN_INITIATOR_LITE == 1) begin : gen_w_r_buf_len_pd
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
                .WRITE_DATA_WIDTH_A(24),                    //positive integer
                .READ_DATA_WIDTH_A (24),                    //positive integer
                .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(24),               //positive integer
                .READ_DATA_WIDTH_B (24),               //positive integer
                .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_pd_number_qp_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_pd_num_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r[23:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_pd_number_rdata[23:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_qp_pd_num_wrn),
                .addrb         (i_qp_pd_num_idx[0+:C_QP_INDX_WIDTH]),
                .dinb          (i_qp_pd_num),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_pd_num),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign qp_pd_num_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
            //assign qp_pd_num_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
            //assign qp_pd_num_bram_addr =  qp_pd_num_bram_wen ? qp_pd_num_wr_addr : qp_pd_num_rd_addr;
            //assign qp_pd_num_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == PD_NUMBER_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));


            //-----------------------------------------------------------------------------------------------------
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (24 * C_NUM_PD),    //positive integer
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
                .WRITE_DATA_WIDTH_A(24),                    //positive integer
                .READ_DATA_WIDTH_A (24),                    //positive integer
                .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(24),               //positive integer
                .READ_DATA_WIDTH_B (24),               //positive integer
                .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_pd_number_0_pd_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_pd_num_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          (wr_data_mem_r[23:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_pd_pd_num_rdata[23:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_pd_num_wrn),
                .addrb         (i_pd_pd_num_idx),
                .dinb          (i_pd_pd_num),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_pd_num),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign pd_pd_num_wr_addr = wr_addr_r[8+:C_PD_INDX_WIDTH];// The repeating registers start at 0x200 so need to normalize
            //assign pd_pd_num_rd_addr = rd_addr_r[8+:C_PD_INDX_WIDTH];
            //assign pd_pd_num_bram_addr =  pd_pd_num_bram_wen ? pd_pd_num_wr_addr : pd_pd_num_rd_addr;
            //assign pd_pd_num_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == PD_NUMBER_PDN) &&  (wr_addr_r[17] == 'b0));

            //-----------------------------------------------------------------------------------------------------

            //-----------------------------------------------------------------------------------------------------
            // BRAM: QP Virtual Address LSB register -- -- QP 虚拟地址 LSB 寄存器(Virtual address LSB(VIRTADDRLSB))
            // [31:0]   - RQ PSN

            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_PD),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_virt_addr_0_pd_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_virt_addr_0_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          (wr_data_mem_r[31:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_virt_addr_0_rdata[31:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_virt_addr_0_wrn),
                .addrb         (i_pd_virt_addr_0_idx[0+:C_PD_INDX_WIDTH]),
                .dinb          (i_pd_virt_addr_0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_virt_addr_0[31:0]),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign pd_virt_addr_0_wr_addr = wr_addr_r[8+:C_PD_INDX_WIDTH];// The repeating registers start at 0x200 so need to normalize
            //assign pd_virt_addr_0_rd_addr = rd_addr_r[8+:C_PD_INDX_WIDTH];
            //assign pd_virt_addr_0_bram_addr =  pd_virt_addr_0_bram_wen ? pd_virt_addr_0_wr_addr : pd_virt_addr_0_rd_addr;
            //assign pd_virt_addr_0_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == VIRT_ADDR_0_PDN) &&  (wr_addr_r[17] == 'b0));


            //-----------------------------------------------------------------------------------------------------


            //-----------------------------------------------------------------------------------------------------
            // BRAM: QP Virtual Address MSB register -- QP 虚拟地址 MSB 寄存器(Virtual address MSB(VIRTADDRMSB))
            // [31:0]   - RQ PSN


            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_PD),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_virt_addr_1_pd_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_virt_addr_1_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          (wr_data_mem_r[31:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_virt_addr_1_rdata[31:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_virt_addr_1_wrn),
                .addrb         (i_pd_virt_addr_1_idx[0+:C_PD_INDX_WIDTH]),
                .dinb          (i_pd_virt_addr_1),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_virt_addr_1[31:0]),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign pd_virt_addr_1_wr_addr = wr_addr_r[8+:C_PD_INDX_WIDTH];// The repeating registers start at 0x200 so need to normalize
            //assign pd_virt_addr_1_rd_addr = rd_addr_r[8+:C_PD_INDX_WIDTH];
            //assign pd_virt_addr_1_bram_addr =  pd_virt_addr_1_bram_wen ? pd_virt_addr_1_wr_addr : pd_virt_addr_1_rd_addr;
            //assign pd_virt_addr_1_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == VIRT_ADDR_1_PDN) &&  (wr_addr_r[17] == 'b0));


            //-----------------------------------------------------------------------------------------------------

            //-----------------------------------------------------------------------------------------------------
            // BRAM: QP Buffer base address LSB register -- QP 缓冲区基地址 LSB 寄存器(Buffer base address LSB(BUFBASEADDRLSB))
            // [31:0]   - PD BASE ADDR


            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_PD),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_buf_base_addr_0_pd_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_buf_base_addr_0_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          (wr_data_mem_r[31:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_buf_base_addr_0_rdata[31:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_buf_base_addr_0_wrn),
                .addrb         (i_pd_buf_base_addr_0_idx[0+:C_PD_INDX_WIDTH]),
                .dinb          (i_pd_buf_base_addr_0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_buf_base_addr_0[31:0]),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign pd_buf_base_addr_0_wr_addr = wr_addr_r[8+:C_PD_INDX_WIDTH];// The repeating registers start at 0x200 so need to normalize
            //assign pd_buf_base_addr_0_rd_addr = rd_addr_r[8+:C_PD_INDX_WIDTH];
            //assign pd_buf_base_addr_0_bram_addr =  pd_buf_base_addr_0_bram_wen ? pd_buf_base_addr_0_wr_addr : pd_buf_base_addr_0_rd_addr;
            //assign pd_buf_base_addr_0_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == BUF_BASE_ADDR_0_PDN) &&  (wr_addr_r[17] == 'b0));


            //-----------------------------------------------------------------------------------------------------

            //-----------------------------------------------------------------------------------------------------
            // BRAM: QP Buffer base address MSB register -- QP 缓冲区基地址 MSB 寄存器(Buffer base address MSB(BUFBASEADDRMSB))
            // [31:0]   -PD BASE ADDR


            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_PD),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_buf_base_addr_1_pd_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_buf_base_addr_1_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          (wr_data_mem_r[31:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_buf_base_addr_1_rdata[31:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_buf_base_addr_1_wrn),
                .addrb         (i_pd_buf_base_addr_1_idx[0+:C_PD_INDX_WIDTH]),
                .dinb          (i_pd_buf_base_addr_1),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_buf_base_addr_1[31:0]),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign pd_buf_base_addr_1_wr_addr = wr_addr_r[8+:C_PD_INDX_WIDTH];// The repeating registers start at 0x200 so need to normalize
            //assign pd_buf_base_addr_1_rd_addr = rd_addr_r[8+:C_PD_INDX_WIDTH];
            //assign pd_buf_base_addr_1_bram_addr =  pd_buf_base_addr_1_bram_wen ? pd_buf_base_addr_1_wr_addr : pd_buf_base_addr_1_rd_addr;
            //assign pd_buf_base_addr_1_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == BUF_BASE_ADDR_1_PDN) &&  (wr_addr_r[17] == 'b0));


            //-----------------------------------------------------------------------------------------------------

            //-----------------------------------------------------------------------------------------------------
            // BRAM: QP Buffer R Key register -- QP 缓冲区 R Key 寄存器(Buffer R_Key(BUFRKEY))
            localparam C_PD_RKEY_SIZE_L = (C_PD_RKEY_SIZE < 6'd32) ? 6'd32 - C_PD_RKEY_SIZE : 6'd32;
            wire [31:0] s_axi_lite_wdata_rkey = (C_PD_RKEY_SIZE < 6'd32) ? {{C_PD_RKEY_SIZE_L{1'b0}}, wr_data_mem_r[C_PD_RKEY_SIZE-1:0]} : wr_data_mem_r;

            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (C_PD_RKEY_SIZE * C_NUM_PD),  //positive integer
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
                .WRITE_DATA_WIDTH_A(C_PD_RKEY_SIZE),        //positive integer
                .READ_DATA_WIDTH_A (C_PD_RKEY_SIZE),        //positive integer
                .BYTE_WRITE_WIDTH_A(C_PD_RKEY_SIZE),        //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(C_PD_RKEY_SIZE),   //positive integer
                .READ_DATA_WIDTH_B (C_PD_RKEY_SIZE),   //positive integer
                .BYTE_WRITE_WIDTH_B(C_PD_RKEY_SIZE),   //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_buf_r_key_pd_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_buf_r_key_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          (s_axi_lite_wdata_rkey[C_PD_RKEY_SIZE-1 : 0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_buf_r_key_rdata[C_PD_RKEY_SIZE-1:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_buf_r_key_wrn),
                .addrb         (i_pd_buf_r_key_idx[0+:C_PD_INDX_WIDTH]),
                .dinb          (i_pd_buf_r_key),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_buf_r_key),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign pd_buf_r_key_wr_addr = wr_addr_r[8+:C_PD_INDX_WIDTH];// The repeating registers start at 0x200 so need to normalize
            //assign pd_buf_r_key_rd_addr = rd_addr_r[8+:C_PD_INDX_WIDTH];
            //assign pd_buf_r_key_bram_addr =  pd_buf_r_key_bram_wen ? pd_buf_r_key_wr_addr : pd_buf_r_key_rd_addr;
            //assign pd_buf_r_key_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == BUF_R_KEY_PDN) &&  (wr_addr_r[17] == 'b0));


            //-----------------------------------------------------------------------------------------------------

            //-----------------------------------------------------------------------------------------------------
            // BRAM: QP Buffer length register -- 写/读缓冲区长度 LSB 寄存器(Write/Read buffer length(WRRDBUFLEN))
            // Write/Read buffer length LSB
            // [31:0]   - PD BUFFER LENGTH


            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_PD),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_w_r_buf_len_pd_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_w_r_buf_len_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          (wr_data_mem_r[31:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_w_r_buf_len_rdata[31:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_w_r_buf_len_wrn),
                .addrb         (i_pd_w_r_buf_len_idx[0+:C_PD_INDX_WIDTH]),
                .dinb          (i_pd_w_r_buf_len[31:0]),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_w_r_buf_len[31:0]),
                .sbiterrb      (),
                .dbiterrb      ()

            );

            // BRAM: Access Description(ACCESSDESC)[31:16]
            // Write/Read buffer length MSB
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (16 * C_NUM_PD),    //positive integer
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
                .WRITE_DATA_WIDTH_A(16),                    //positive integer
                .READ_DATA_WIDTH_A (16),                    //positive integer
                .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(16),               //positive integer
                .READ_DATA_WIDTH_B (16),               //positive integer
                .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_w_r_buf_len_pd_msb_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_access_type_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          (wr_data_mem_r[31:16]),            // [31:16]: w_r_buf_len_msb。寄存器提供写/读缓冲区长度[47:32］
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_access_type_rdata[31:16]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_w_r_buf_len_wrn),
                .addrb         (i_pd_w_r_buf_len_idx[0+:C_PD_INDX_WIDTH]),
                .dinb          (i_pd_w_r_buf_len[47:32]),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_w_r_buf_len[47:32]),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign pd_w_r_buf_len_wr_addr = wr_addr_r[8+:C_PD_INDX_WIDTH];// The repeating registers start at 0x200 so need to normalize
            //assign pd_w_r_buf_len_rd_addr = rd_addr_r[8+:C_PD_INDX_WIDTH];
            //assign pd_w_r_buf_len_bram_addr =  pd_w_r_buf_len_bram_wen ? pd_w_r_buf_len_wr_addr : pd_w_r_buf_len_rd_addr;
            //assign pd_w_r_buf_len_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == W_R_BUF_LEN_PDN) &&  (wr_addr_r[17] == 'b0));

            //-----------------------------------------------------------------------------------------------------

            //-----------------------------------------------------------------------------------------------------
            // BRAM: PD Access type -- Access Description(ACCESSDESC)[3:0]
            // [3:0]   - ACCESS
            // 4'b0000: READ Only
            // 4'b0001: Write Only
            // 4'b0010: Read and Write
            // Other values: Not supported


            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (4 * C_NUM_PD),     //positive integer
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
                .WRITE_DATA_WIDTH_A(4),                     //positive integer
                .READ_DATA_WIDTH_A (4),                     //positive integer
                .BYTE_WRITE_WIDTH_A(4),                     //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PD_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(4),                //positive integer
                .READ_DATA_WIDTH_B (4),                //positive integer
                .BYTE_WRITE_WIDTH_B(4),                //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_PD_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_access_type_pd_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (pd_access_type_bram_wen),
                .addra         (wr_addr_mem_r1),
                .dina          ({2'h0, wr_data_mem_r[1:0]}),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_access_type_rdata[3:0]),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_pd_access_type_wrn),
                .addrb         (i_pd_access_type_idx[0+:C_PD_INDX_WIDTH]),
                .dinb          (i_pd_access_type),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_pd_access_type[3:0]),
                .sbiterrb      (),
                .dbiterrb      ()

            );


            //assign pd_access_type_wr_addr = wr_addr_r[8+:C_PD_INDX_WIDTH];// The repeating registers start at 0x200 so need to normalize
            //assign pd_access_type_rd_addr = rd_addr_r[8+:C_PD_INDX_WIDTH];
            //assign pd_access_type_bram_addr =  pd_access_type_bram_wen ? pd_access_type_wr_addr : pd_access_type_rd_addr;
            //assign pd_access_type_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == ACCESS_TYPE_PDN) &&  (wr_addr_r[17] == 'b0));
        end
    endgenerate
    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    // BRAM: Dest QP conf registers ---- Destination QP configuration for QPi (DESTQPCONFi) 寄存器
    // [23:0]   - Dest QP ID
    // Check_SDPRAM


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_dest_qpid_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_dest_qpid_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_dest_qpid_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_dest_qpid_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (24'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_dest_qpid[23:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_dest_qpid_rdata[31:24] = 'b0;

    //assign qp_dest_qpid_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;// The repeating registers start at 0x200 so need to normalize
    //assign qp_dest_qpid_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_dest_qpid_bram_addr =  qp_dest_qpid_bram_wen ? qp_dest_qpid_wr_addr : qp_dest_qpid_rd_addr;
    //assign qp_dest_qpid_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == DEST_QP_CONF_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));


    //-----------------------------------------------------------------------------------------------------

    // //-----------------------------------------------------------------------------------------------------
    // BRAM: QP timeout register -- Timeout Configuration Register for QPi(TIMEOUTCONFi)
    // [4:0]   - 超时值
    // [7:5]   - reserved - not implemented
    // [10:8]  - 最大重试次数
    // [13:11] - 最大 RNR 重试次数
    // [15:14] - reserved - not implemented
    // [20:16] - RNR NACK 传出数据包的超时值
    // [31:21] - reserved - not implemented
    // Total 16 bits implemented
    //
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_timeout_reg1 (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_timeout_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          ({wr_data_mem_r[20:16], wr_data_mem_r[13:11], wr_data_mem_r[10:8], wr_data_mem_r[4:0]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         ({s_axi_timeout_rdata[20:16], s_axi_timeout_rdata[13:11], s_axi_timeout_rdata[10:8], s_axi_timeout_rdata[4:0]}),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_timeout_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (16'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         ({o_qp_timeout[20:16], o_qp_timeout[13:11], o_qp_timeout[10:8], o_qp_timeout[4:0]}),
        .sbiterrb      (),
        .dbiterrb      ()

    );



    // These are status bits implemented for STAT_QP register.
    //  stat - [26:24] - CURR_RETRY_CNT
    //  stat - [27]    - REserved - not implemented
    //  stat - [30:28] - CURR_RNR_NACK_CNT

    //xpm_memory_tdpram # (
    //
    //  // Common module parameters
    //  .MEMORY_SIZE        (11*C_NUM_QP),            //positive integer
    //  .MEMORY_PRIMITIVE   ("auto"),          //string; "auto", "distributed", "block" or "ultra";
    //  .CLOCKING_MODE      ("independent_clock"),  //string; "common_clock", "independent_clock" 
    //  .MEMORY_INIT_FILE   ("none"),          //string; "none" or "<filename>.mem" 
    //  .MEMORY_INIT_PARAM  (""    ),          //string;
    //  .USE_MEM_INIT       (1),               //integer; 0,1
    //  .WAKEUP_TIME        ("disable_sleep"), //string; "disable_sleep" or "use_sleep_pin" 
    //  .MESSAGE_CONTROL    (0),               //integer; 0,1
    //  .ECC_MODE           ("no_ecc"),        //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
    //  .AUTO_SLEEP_TIME    (0),               //Do not Change
    //
    //  // Port A module parameters
    //  .WRITE_DATA_WIDTH_A (11),              //positive integer
    //  .READ_DATA_WIDTH_A  (11),              //positive integer
    //  .BYTE_WRITE_WIDTH_A (11),              //integer; 8, 9, or WRITE_DATA_WIDTH_A value
    //  .ADDR_WIDTH_A       (C_QP_INDX_WIDTH),               //positive integer
    //  .READ_RESET_VALUE_A ("0"),             //string
    //  .READ_LATENCY_A     (1),               //non-negative integer
    //  .WRITE_MODE_A       ("read_first"),     //string; "write_first", "read_first", "no_change" 
    //
    //  // Port B module parameters
    //  .WRITE_DATA_WIDTH_B (11),              //positive integer
    //  .READ_DATA_WIDTH_B  (11),              //positive integer
    //  .BYTE_WRITE_WIDTH_B (11),              //integer; 8, 9, or WRITE_DATA_WIDTH_B value
    //  .ADDR_WIDTH_B       (C_QP_INDX_WIDTH),               //positive integer
    //  .READ_RESET_VALUE_B ("0"),             //vector of READ_DATA_WIDTH_B bits
    //  .READ_LATENCY_B     (1),               //non-negative integer
    //  .WRITE_MODE_B       ("read_first")      //string; "write_first", "read_first", "no_change" 
    //
    //) inst_qp_timeout_stat_reg (
    //
    //  // Common module ports
    //  .sleep          (1'b0),
    //
    //  // Port A module ports - used for status reads from AXI lite. No writes
    //  // allowed
    //  .clka           (core_clk),
    //  .rsta           (~core_rstn),
    //  .ena            (1'b1),
    //  .regcea         (1'b1),
    //  .wea            (1'b0),
    //  .addra          (qp_timeout_bram_addr),
    //  .dina           ({s_axi_lite_wdata[30:28],s_axi_lite_wdata[26:24],s_axi_lite_wdata[20:16]}),
    //  .injectsbiterra (1'b0),
    //  .injectdbiterra (1'b0),
    //  .douta          ({s_axi_stat_qpn_rdata[30:28], s_axi_timeout_rdata[26:24]}),
    //  .sbiterra       (),
    //  .dbiterra       (),
    //
    //  // Port B module ports - used for resp handler reads and writes
    //  .clkb           (core_clk),
    //  .rstb           (~core_rstn),
    //  .enb            (1'b1),
    //  .regceb         (1'b1),
    //  .web            (i_qp_timeout_wrn),
    //  .addrb          (i_qp_timeout_idx[0+:C_QP_INDX_WIDTH]),
    //  .dinb           (i_qp_timeout_stat[10:0]),
    //  .injectsbiterrb (1'b0),
    //  .injectdbiterrb (1'b0),
    //  .doutb          ({o_qp_timeout[30:28], o_qp_timeout[26:24], o_qp_timeout[20:16]}),
    //  .sbiterrb       (),
    //  .dbiterrb       ()
    //
    //);


    assign o_qp_timeout[7:5]          = 'b0;
    assign o_qp_timeout[15:14]        = 'b0;
    assign o_qp_timeout[31:21]        = 'b0;
    assign s_axi_timeout_rdata[7:5]   = 'b0;
    assign s_axi_timeout_rdata[15:14] = 'b0;
    assign s_axi_timeout_rdata[31:21] = 'b0;

    //assign qp_timeout_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_timeout_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_timeout_bram_addr =  qp_timeout_bram_wen ? qp_timeout_wr_addr : qp_timeout_rd_addr;
    //assign qp_timeout_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == TIMEOUT_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP MAC REMOTE ADDR LSB register -- MAC destination address LSB QPi(MACDESADDLSBi) 寄存器
    // [31:0]   - MAC REMOTE ADDR VALUE
    // 该寄存器由 SW 在连接时配置，提供与该 QP 连接的远程主机的 MAC 地址。从该 QP 发送的所有数据包都以该 MAC 地址（LSB）作为目标 MAC 地址。
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_mac_remote1_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_mac_remote_addrl_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_mac_remote_addrl_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_mac_remote_addrl_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_mac_remote_addrl[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_mac_remote_addrl_idx = i_qp_mac_remote_addrl_req ? i_qp_mac_remote_addrl_idx : i_qp_mac_remote_addrl_wqe_idx;

    // REPLICA MEMORY
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_mac_remote1_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_mac_remote_addrl_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_mac_remote_addrl_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_mac_remote_addrl_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP MAC DEST ADDR MSB register
    // [15:0]   - MAC DEST ADDR VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_mac_remote2_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_mac_remote_addrm_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_mac_remote_addrm_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_mac_remote_addrm_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (16'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_mac_remote_addrm[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign o_qp_mac_remote_addrm[31:16]        = 'b0;
    assign s_axi_mac_remote_addrm_rdata[31:16] = 'b0;

    assign muxed_qp_mac_remote_addrm_idx       = i_qp_mac_remote_addrm_req ? i_qp_mac_remote_addrm_idx : i_qp_mac_remote_addrm_wqe_idx;
    //assign qp_mac_remote_addrm_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_mac_remote_addrm_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_mac_remote_addrm_bram_addr =  qp_mac_remote_addrm_bram_wen ? qp_mac_remote_addrm_wr_addr : qp_mac_remote_addrm_rd_addr;
    //assign qp_mac_remote_addrm_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == MAC_REMOTE_ADDR_MSB_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
    // SDP Memory No Collission
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),               //positive integer
        .READ_DATA_WIDTH_A (16),               //positive integer
        .BYTE_WRITE_WIDTH_A(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_mac_remote2_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_mac_remote_addrm_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_mac_remote_addrm_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (16'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_mac_remote_addrm_replica[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP IP DEST ADDR1 register
    // [31:0]   - IP DEST ADDR LSB 1 VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote1_regs (


        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr1_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_ip_remote_addr1_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_ip_remote_addr1_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr1[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_ip_remote_addr1_idx = i_qp_ip_remote_addr1_req ? i_qp_ip_remote_addr1_idx : i_qp_ip_remote_addr1_wqe_idx;
    //assign qp_ip_remote_addr1_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_ip_remote_addr1_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_ip_remote_addr1_bram_addr =  qp_ip_remote_addr1_bram_wen ? qp_ip_remote_addr1_wr_addr : qp_ip_remote_addr1_rd_addr;
    //assign qp_ip_remote_addr1_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == IP_REMOTE_ADDR1_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    // REPLICA memory
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote1_replica_regs (


        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr1_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_ip_remote_addr1_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr1_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP IP DEST ADDR2 register
    // [31:0]   - IP DEST ADDR LSB 2 VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote2_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr2_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_ip_remote_addr2_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_ip_remote_addr2_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr2[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_ip_remote_addr2_idx = i_qp_ip_remote_addr2_req ? i_qp_ip_remote_addr2_idx : i_qp_ip_remote_addr2_wqe_idx;
    //assign qp_ip_remote_addr2_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_ip_remote_addr2_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_ip_remote_addr2_bram_addr =  qp_ip_remote_addr2_bram_wen ? qp_ip_remote_addr2_wr_addr : qp_ip_remote_addr2_rd_addr;
    //assign qp_ip_remote_addr2_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == IP_REMOTE_ADDR2_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    // REPLICA MEMORY
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote2_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr2_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_ip_remote_addr2_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr2_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP IP DEST ADDR3 register
    // [31:0]   - IP DEST ADDR MSB 1 VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote3_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr3_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_ip_remote_addr3_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_ip_remote_addr3_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr3[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_ip_remote_addr3_idx = i_qp_ip_remote_addr3_req ? i_qp_ip_remote_addr3_idx : i_qp_ip_remote_addr3_wqe_idx;
    //assign qp_ip_remote_addr3_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_ip_remote_addr3_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_ip_remote_addr3_bram_addr =  qp_ip_remote_addr3_bram_wen ? qp_ip_remote_addr3_wr_addr : qp_ip_remote_addr3_rd_addr;
    //assign qp_ip_remote_addr3_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == IP_REMOTE_ADDR3_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    // REPLICA MEMORY
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote3_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr3_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_ip_remote_addr3_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr3_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP IP DEST ADDR4 register
    // [31:0]   - IP DEST ADDR MSB 2 VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote4_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr4_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_ip_remote_addr4_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_ip_remote_addr4_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr4[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_ip_remote_addr4_idx = i_qp_ip_remote_addr4_req ? i_qp_ip_remote_addr4_idx : i_qp_ip_remote_addr4_wqe_idx;
    //assign qp_ip_remote_addr4_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_ip_remote_addr4_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_ip_remote_addr4_bram_addr =  qp_ip_remote_addr4_bram_wen ? qp_ip_remote_addr4_wr_addr : qp_ip_remote_addr4_rd_addr;
    //assign qp_ip_remote_addr4_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == IP_REMOTE_ADDR4_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));

    // REPLICA MEMORY
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote4_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr4_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_ip_remote_addr4_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr4_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT CURR SQ_PTR_PROC register
    // [15:0]   - STAT_CURR_SQPTR_PROC - Read only
    // Double Check Collission

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_L),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_curr_sqptr_proc_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (stat_curr_sqptr_bram_wen),
        .addra         (stat_curr_sqptr_proc_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_curr_sqptr_proc_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_curr_sqptr_proc_wen),
        .addrb         (i_qp_curr_sqptr_proc_idx),
        .dinb          (i_qp_curr_sqptr_proc),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_curr_sqptr_proc),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    assign s_axi_curr_sqptr_proc_rdata[31:16] = 'b0;

    // The AXI lite interface and the logic that updates the SQ full/empty
    // share the BRAM port A. BRAM port B is only dedicated for the QP manager
    // cache module which reads the current sq_ptr and also updates it
    assign curr_sqptr_proc                    = s_axi_curr_sqptr_proc_rdata[15:0];

    // To_be_Removed
    //assign stat_curr_sqptr_proc_addr      = (axi_curr_sqptr_proc_read | stat_curr_sqptr_bram_wen) ? stat_curr_sqptr_proc_bram_addr : pending_idx; 
    assign stat_curr_sqptr_proc_addr          = stat_curr_sqptr_proc_bram_addr;
    assign stat_curr_sqptr_proc_wr_addr       = wr_addr_r[8+:C_QP_INDX_WIDTH];
    assign stat_curr_sqptr_proc_rd_addr       = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign stat_curr_sqptr_proc_bram_addr     = stat_curr_sqptr_bram_wen ? stat_curr_sqptr_proc_wr_addr : stat_curr_sqptr_proc_rd_addr;
    assign axi_curr_sqptr_proc_read           = ~generic_reg_rd_access & (rd_addr_r[7:0] == STAT_CURR_SQPTR_PROC_QPN) & ~axi_lite_rvalid_i;
    //assign axi_curr_sqptr_proc_read       = ~generic_reg_rd_access & (rd_addr_r[7:0] == STAT_CURR_SQPTR_PROC_QPN) & ~s_axi_lite_rvalid;
    assign stat_curr_sqptr_bram_wen           = xrnic_adv_conf_sw_override && (wr_req_mask_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid))) && ((wr_addr_r[7:0] == STAT_CURR_SQPTR_PROC_QPN) && wr_addr_r[20:19] == 2'b11);

    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT MSN register
    // [23:0]   - STAT CURR MSN

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_stat_msn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_stat_msn_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_msn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_msn_wrn),
        .addrb         (i_qp_stat_msn_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_stat_msn[23:0]),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_msn),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_stat_msn_rdata[31:24] = 'b0;

    //assign qp_stat_msn_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_stat_msn_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_stat_msn_bram_addr = qp_stat_msn_bram_wen ? qp_stat_msn_wr_addr :  qp_stat_msn_rd_addr; // READ ONLY REGISTER
    //assign qp_stat_msn_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && 
    //                               (((wr_addr_r[7:0] == STAT_MSN_QPN) ) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));


    //-----------------------------------------------------------------------------------------------------
    //
    ////-----------------------------------------------------------------------------------------------------
    // BRAM: STAT SSN register -- STATSSNi
    // 该寄存器是根据参数 C_NUM_QP（其中 i = 2 至 C_NUM_QP）生成的。 QP1 不存在该寄存器。
    // [23:0]   - STAT CURR SSN    - 当前传出的 SSN（与传出的 MSN 相同）

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_stat_ssn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_stat_ssn_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_ssn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_ssn_wrn),
        .addrb         (i_qp_stat_ssn_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_stat_ssn[23:0]),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_ssn[23:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_stat_ssn_rdata[31:24] = 'b0;

    //assign qp_stat_ssn_wr_addr = wr_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_stat_ssn_rd_addr = rd_addr_r[8+:C_QP_INDX_WIDTH] ;
    //assign qp_stat_ssn_bram_addr =  qp_stat_ssn_bram_wen ? qp_stat_ssn_wr_addr: qp_stat_ssn_rd_addr; // READ ONLY REGISTER
    //assign qp_stat_ssn_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && 
    //                               (((wr_addr_r[7:0] == STAT_SSN_QPN) ) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));


    //-----------------------------------------------------------------------------------------------------
    //
    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP STAT NAK register -- Status QPi(STATQPi)部分
    // 与 NAK 相关的 QP 状态寄存器中的位已在此 BRAM 中实现。
    // 此外，还包括 STAT_QPn 寄存器的两个字段。
    // 由于 Curr_retry_cnt 由 resp_handler 生成，而其他位由 rx_pkt_handler 生成，因此在两个不同的 BRAM 中实现了这些位。
    // [22:16]  - 收到 NACK 综合征（读/写）(7 bits)
    // [30:28]  - 当前的 RNR nak 计数(计数器计数信息)（读/写），用于接收应答数据包 (3 bits)

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (10 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(10),                    //positive integer
        .READ_DATA_WIDTH_A (10),                    //positive integer
        .BYTE_WRITE_WIDTH_A(10),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_K),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(10),               //positive integer
        .READ_DATA_WIDTH_B (10),               //positive integer
        .BYTE_WRITE_WIDTH_B(10),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_psn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_stat_nak_bram_wen),
        .addra         (qp_stat_nak_bram_addr),
        .dina          ({s_axi_lite_wdata[30:28], s_axi_lite_wdata[22:16]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         ({s_axi_stat_qpn_rdata[30:28], s_axi_stat_qpn_rdata[22:16]}),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_nak_wen),
        .addrb         (i_qp_stat_nak_idx),
        .dinb          (i_qp_stat_nak),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_nak),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    // BRAM: QP STAT NAK register -- Status QPi(STATQPi)部分
    // [26:24]  - 当前重试次数（读/写）
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (3 * C_NUM_QP),     //positive integer
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
        .WRITE_DATA_WIDTH_A(3),                     //positive integer
        .READ_DATA_WIDTH_A (3),                     //positive integer
        .BYTE_WRITE_WIDTH_A(3),                     //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_K),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(3),                //positive integer
        .READ_DATA_WIDTH_B (3),                //positive integer
        .BYTE_WRITE_WIDTH_B(3),                //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_retry_cnt_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_stat_nak_bram_wen),
        .addra         (qp_stat_nak_bram_addr),
        .dina          (s_axi_lite_wdata[26:24]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_qpn_rdata[26:24]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_retry_cnt_wen),
        .addrb         (i_qp_stat_retry_cnt_idx),
        .dinb          (i_qp_stat_retry_cnt),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_retry_cnt),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //#######################################################################################################################################
    //Assumption: Atleast 1 clk gap between consecutive assertion of both i_osq_full_set_valid and i_osq_full_reset_valid
    //#######################################################################################################################################
    always @(posedge core_clk) begin : i_osq_full_proc
        if (!core_rstn) begin
            osq_full_valid         <= 1'b0;
            osq_full_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_full_data          <= 1'b0;
            osq_full_valid_pending <= 1'b0;
            osq_full_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_osq_full_set_valid) begin
                osq_full_valid <= 1'b1;
                osq_full_id    <= i_osq_full_set_id;
                osq_full_data  <= 1'b1;
            end else if (i_osq_full_reset_valid) begin
                osq_full_valid <= 1'b1;
                osq_full_id    <= i_osq_full_reset_id;
                osq_full_data  <= 1'b0;
            end else if (osq_full_valid_pending) begin
                osq_full_valid <= 1'b1;
                osq_full_id    <= osq_full_id_pending;
                osq_full_data  <= 1'b0;
            end else begin
                osq_full_valid <= 1'b0;
            end

            if (i_osq_full_set_valid && i_osq_full_reset_valid) begin  // address collision is avoided at the source of this signal
                osq_full_valid_pending <= 1'b1;
                osq_full_id_pending    <= i_osq_full_reset_id;
            end else begin
                osq_full_valid_pending <= 1'b0;
            end
        end
    end

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_full (
        .core_clk(core_clk),
        .core_rst(!core_rstn),
        .wr_en   (init_start || osq_full_valid),
        .wr_addr (init_start ? init_count : osq_full_id),
        .wr_data (init_start ? 1'b0 : osq_full_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (osq_full_rd_data_int)
    );
    assign osq_full_rd_data = (osq_full_valid_pending && osq_full_id_pending == rd_addr_mem_r_r) ? 1'b0 : osq_full_rd_data_int;

    //#######################################################################################################################################
    //Assumption: Atleast 1 clk gap between consecutive assertion of both i_osq_empty_set_valid and i_osq_empty_reset_valid
    //#######################################################################################################################################
    always @(posedge core_clk) begin : i_osq_empty_proc
        if (!core_rstn) begin
            osq_empty_valid         <= 1'b0;
            osq_empty_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_empty_data          <= 1'b0;
            osq_empty_valid_pending <= 1'b0;
            osq_empty_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_osq_empty_set_valid) begin
                osq_empty_valid <= 1'b1;
                osq_empty_id    <= i_osq_empty_set_id;
                osq_empty_data  <= 1'b1;
            end else if (i_osq_empty_reset_valid) begin
                osq_empty_valid <= 1'b1;
                osq_empty_id    <= i_osq_empty_reset_id;
                osq_empty_data  <= 1'b0;
            end else if (osq_empty_valid_pending) begin
                osq_empty_valid <= 1'b1;
                osq_empty_id    <= osq_empty_id_pending;
                osq_empty_data  <= 1'b0;
            end else begin
                osq_empty_valid <= 1'b0;
            end

            if (i_osq_empty_set_valid && i_osq_empty_reset_valid) begin  // address collision is avoided at the source of this signal
                osq_empty_valid_pending <= 1'b1;
                osq_empty_id_pending    <= i_osq_empty_reset_id;
            end else begin
                osq_empty_valid_pending <= 1'b0;
            end
        end
    end

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_i_osq_empty (
        .core_clk(core_clk),
        .core_rst(!core_rstn),
        .wr_en   (init_start || osq_empty_valid),
        .wr_addr (init_start ? init_count : osq_empty_id),
        .wr_data (init_start ? 1'b1 : osq_empty_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (osq_empty_rd_data_int)
    );
    assign osq_empty_rd_data = (osq_empty_valid_pending && osq_empty_id_pending == rd_addr_mem_r_r) ? 1'b0 : osq_empty_rd_data_int;

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_retried (
        .core_clk   (core_clk),
        .core_rst   (!core_rstn),
        .wr_en      (init_start || i_qp_retried_reset_valid),
        .wr_addr    (init_start ? init_count : i_qp_retried_reset_id),
        .wr_data    (1'b0),
        .wr_en_aux  (i_qp_retried_set_valid),
        .wr_addr_aux(i_qp_retried_set_id),
        .wr_data_aux(1'b1),
        .rd_en      (1'b1),
        .rd_addr    (rd_addr_mem_r_r),
        .rd_data    (qp_retried_rd_data)
    );

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_rq_full (
        .core_clk   (core_clk),
        .core_rst   (!core_rstn),
        .wr_en      (init_start || i_rq_full_valid),
        .wr_addr    (init_start ? init_count : i_rq_full_idx),
        .wr_data    (init_start ? 1'b0 : i_rq_full_data),
        .wr_en_aux  (i_rq_full_reset_valid),
        .wr_addr_aux(i_rq_full_reset_idx),
        .wr_data_aux(i_rq_full_reset_data),
        .rd_en      (1'b1),
        .rd_addr    (rd_addr_mem_r_r),
        .rd_data    (i_rq_full_rd_data)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_fatal_err (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_fatal_err_valid),
        .wr_addr (init_start ? init_count : i_qp_fatal_err_id),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_err_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (fatal_err_rd_data)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_tx_path_congestion (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_tx_path_congestion_valid),
        .wr_addr (init_start ? init_count : i_tx_path_congestion_id),
        .wr_data (init_start ? 1'b0 : i_tx_path_congestion_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (tx_path_congestion_rd_data)
    );

    // 每个 QP 的空/非空状态存放在 inst_dbg_sq_empty_ff 这个 LUT RAM 里，初值 1（假设系统启动时 SQ 为空）
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_dbg_sq_empty_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || dbg_sq_empty_ff_valid),
        .wr_addr (init_start ? init_count : dbg_sq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : dbg_sq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (dbg_sq_empty_ff_rd_data)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_dbg_sq_full_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || dbg_sq_full_ff_valid),
        .wr_addr (init_start ? init_count : dbg_sq_full_ff_id),
        .wr_data (init_start ? 1'b0 : dbg_sq_full_ff_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (dbg_sq_full_ff_rd_data)
    );

    //Add Pipeline
    // Status QPi (STATQPi)寄存器
    assign qp_stat_nak_bram_rd_addr    = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign s_axi_stat_qpn_rdata[0]     = fatal_err_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[1]     = i_rq_full_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[2]     = dbg_sq_full_ff_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[3]     = osq_full_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[4]     = 1'b0;  //i_cq_full[rd_addr_mem_r_r]; // CQ full not implemented
    assign s_axi_stat_qpn_rdata[7:5]   = 'b0;
    assign s_axi_stat_qpn_rdata[8]     = 1'b0;  //i_rq_empty[rd_addr_mem_r_r]; // RQ empty not getting generated
    assign s_axi_stat_qpn_rdata[9]     = dbg_sq_empty_ff_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[10]    = osq_empty_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[11]    = qp_retried_rd_data;
    assign s_axi_stat_qpn_rdata[12]    = tx_path_congestion_rd_data;
    assign s_axi_stat_qpn_rdata[15:13] = 'b0;
    assign s_axi_stat_qpn_rdata[23]    = 'b0;
    assign s_axi_stat_qpn_rdata[27]    = 'b0;
    assign s_axi_stat_qpn_rdata[31]    = 'b0;
    assign qp_stat_nak_bram_addr       = qp_stat_nak_bram_wen ? (wr_addr_r[8+:C_QP_INDX_WIDTH]) : (rd_addr_r[8+:C_QP_INDX_WIDTH]);
    assign qp_stat_nak_bram_wen        = (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && ((wr_addr_r[7:0] == STAT_QPN) && wr_addr_r[20:19] == 2'b11);

    //----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT RESP PSN register
    // [23:0]   - STAT_RESP_PSN - Read only


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_resp_psn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports，初始化时会写入 (发送 Q PSN - 1)的值
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_psn_bram_wen),
        .addra         (qp_sq_psn_bram_addr),
        .dina          ((s_axi_lite_wdata[23:0] - 1'b1)),  // debug: 774470 - 1 = 77446F
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_resp_psn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_resp_psn_wen),
        .addrb         (i_qp_stat_resp_psn_idx),
        .dinb          (i_qp_stat_resp_psn),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_resp_psn),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_stat_resp_psn_rdata[31:24] = 'b0;


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT RQ BUF CA register
    // [31:8]   - STAT_RQ_BUF_CA - Read only
    // 保存的是“状态类”信息——每个 QP 的 RQ Buffer 当前位置（Current Address，CA）。这些值不是软件配置的，而是硬件在运行过程中更新，用来反映接收缓冲区已经写到了哪里，软件或调试逻辑可以读取它来观察当前状态。

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_rq_buf_ca_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports，初始化时会写入RQ buffer基地址
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_rq_ba_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:8]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_rq_buf_ca_rdata[31:8]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports -- 其他模块 write/read port
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_rq_buf_ca_wen),
        .addrb         (i_qp_stat_rq_buf_ca_idx),
        .dinb          (i_qp_stat_rq_buf_ca[23:0]),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_rq_buf_ca[23:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_rq_buf_ca_msb
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
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
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_rq_buf_ca_regs_msb (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_rq_ba_msb_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_stat_rq_buf_ca_msb_rdata),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (i_qp_stat_rq_buf_ca_wen),
                .addrb         (i_qp_stat_rq_buf_ca_idx),
                .dinb          (i_qp_stat_rq_buf_ca[55:24]),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_stat_rq_buf_ca[55:24]),
                .sbiterrb      (),
                .dbiterrb      ()

            );
        end else begin
            assign s_axi_stat_rq_buf_ca_msb_rdata = 0;
        end
    endgenerate

    assign s_axi_stat_rq_buf_ca_rdata[7:0] = 'b0;


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT WQE register
    // [15:0]   - WQE_CNT_POP - Read only


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_wqe_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_wqe_cnt_bram_wen),
        .addra         (qp_wqe_cnt_bram_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_wqe_cnt_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_wqe_cnt_wen),
        .addrb         (i_qp_stat_wqe_cnt_idx),
        .dinb          (i_qp_stat_wqe_cnt),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_wqe_cnt),
        .sbiterrb      (),
        .dbiterrb      ()
    );
    assign s_axi_stat_wqe_cnt_rdata[31:16] = 'b0;
    assign qp_wqe_cnt_bram_addr            = qp_wqe_cnt_bram_wen ? (wr_addr_r[8+:C_QP_INDX_WIDTH]) : (rd_addr_r[8+:C_QP_INDX_WIDTH]);
    assign qp_wqe_cnt_bram_wen             = xrnic_adv_conf_sw_override & (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && (((wr_addr_r[7:0] == STAT_WQE_CNT)) && wr_addr_r[20:19] == 2'b11);
    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT RQ_PI_DB register -- STATRQPIDBi
    // [15:0]   - RQ_PI - Read only
    // Double Check Collission
    //
    // RQ 生产者索引门铃值寄存器
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_L),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_rq_pi_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_stat_rq_pi_db_bram_wen),
        .addra         (qp_stat_rq_pi_db_bram_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_rq_pi_db_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_rq_pi_db_wrn),
        .addrb         (i_qp_rq_pi_db_idx),
        .dinb          (i_qp_rq_pi_db),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_rq_pi_db),      // RQ 写门铃值
        .sbiterrb      (),
        .dbiterrb      ()
    );

    assign s_axi_stat_rq_pi_db_rdata[31:16] = 16'b0;
    assign qp_stat_rq_pi_db_wr_addr         = wr_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_stat_rq_pi_db_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    // This is to support connect on IO command. After the connect IO SW
    // updates are over there may be some residual RQ doorbell for NVmf. They
    // need to be transmitted to NVMf. This is done using the cnct_io_conf
    // register QPID and the address for that doorbell register in NVmf. 
    assign qp_stat_rq_pi_db_bram_addr       = axi_lite_access_to_rqpidb_reg ? (qp_stat_rq_pi_db_bram_wen ? qp_stat_rq_pi_db_wr_addr : qp_stat_rq_pi_db_rd_addr) : connect_io_qpid[0+:C_QP_INDX_WIDTH];  // READ ONLY REGISTER
    assign qp_stat_rq_pi_db_bram_wen        = xrnic_adv_conf_sw_override & (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid) && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r && (((wr_addr_r[7:0] == STAT_RQ_PI_DB)) && wr_addr_r[20:19] == 2'b11);

    assign axi_lite_access_to_rqpidb_reg    = (wr_addr_r[7:0] == STAT_RQ_PI_DB) || (rd_addr_r[7:0] == STAT_RQ_PI_DB);

    always @(posedge core_clk) o_rq_pi_db_hw_hndshk <= (i_waiting_for_rq_update && (i_waiting_for_rq_update_on_q_num == qp_stat_rq_pi_db_bram_addr)) ? (s_axi_stat_rq_pi_db_rdata[15:0] - 1) : s_axi_stat_rq_pi_db_rdata[15:0];
    //assign o_rq_pi_db_hw_hndshk = i_waiting_for_rq_update ? (s_axi_stat_rq_pi_db_rdata[15:0] - 1) : s_axi_stat_rq_pi_db_rdata[15:0];

    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT RET_SQ_PSN register
    // [23:0]   - RETRIED_SQ_PSN - Read only
    //


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_ret_sq_psn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (1'b0),
        .addra         (qp_stat_ret_sq_psn_bram_addr),
        .dina          (s_axi_lite_wdata[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_ret_sq_psn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_ret_sq_psn_wrn),
        .addrb         (i_qp_ret_sq_psn_idx),
        .dinb          (i_qp_ret_sq_psn),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ret_sq_psn),
        .sbiterrb      (),
        .dbiterrb      ()
    );

    assign s_axi_stat_ret_sq_psn_rdata[31:24] = 'b0;
    assign qp_stat_ret_sq_psn_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_stat_ret_sq_psn_bram_addr       = qp_stat_ret_sq_psn_rd_addr;



    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------

    // 生成 SEND Q 满/空/待处理 信号
    // 每当更新 SQ_PI_DB 或 STAT_CURR_SQPTR_PROC 时，"满"/"空"/"待处理"信号都会发生变化，
    // 因为这两个过程都是异步发生的，而且可能同时发生，因此使用以下方法更新 "空"/"满"信号。
    // 每次写入时，都会设置一个待定信号。一旦 fifo 空/满信号被更新，它就会被重置
    //
    // 在写入空闲缓冲区时消除冲突
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            curr_sqptr_enb_r  <= 1'b0;
            curr_sqptr_vld_r  <= 1'b0;
            curr_sqptr_typ_r  <= 1'b0;
            curr_sqptr_addr_r <= 'd0;
            curr_sqptr_data_r <= 'd0;
        end else begin
            /*if (i_qp_curr_sqptr_proc_wen==1'b1 && stat_curr_sqptr_bram_wen==1'b1) begin
         curr_sqptr_enb_r <= 1'b1;
       end else begin 
         curr_sqptr_enb_r <= 1'b0;
       end
       curr_sqptr_vld_r <= curr_sqptr_vld_c;*/
            curr_sqptr_vld_r  <= curr_sqptr_vld_c && !prog_full_db_update_curr_sqptr;
            curr_sqptr_addr_r <= curr_sqptr_addr_c;
            curr_sqptr_data_r <= curr_sqptr_data_c;
            /*if (i_qp_curr_sqptr_proc_wen==1'b0 && stat_curr_sqptr_bram_wen==1'b1) begin
          curr_sqptr_typ_r <= 1'b1;
       end else begin 
          curr_sqptr_typ_r <= curr_sqptr_enb_r;
       end*/

        end
    end

    assign curr_sqptr_vld_c                 = (i_qp_curr_sqptr_proc_wen == 1'b1) ? i_qp_curr_sqptr_proc_wen : stat_curr_sqptr_bram_wen;
    assign curr_sqptr_addr_c                = (i_qp_curr_sqptr_proc_wen == 1'b1) ? i_qp_curr_sqptr_proc_idx : stat_curr_sqptr_proc_wr_addr;
    assign curr_sqptr_data_c                = (i_qp_curr_sqptr_proc_wen == 1'b1) ? i_qp_curr_sqptr_proc : s_axi_lite_wdata[15:0];
    assign o_prog_full_db_update_curr_sqptr = prog_full_db_update_curr_sqptr;

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (16),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (11),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(5)
    ) u_db_update_curr_sqptr (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (curr_sqptr_vld_r),
        .din          (curr_sqptr_addr_r),
        .rd_en        (curr_sqptr_rden_r),
        .empty        (curr_sqptr_fifo_empty),
        .dout         (curr_sqptr_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (curr_sqptr_fifo_full),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (prog_full_db_update_curr_sqptr),
        .rd_data_count(),
        .rd_rst_busy  (),
        .sbiterr      (),
        .underflow    (),
        .wr_ack       (),
        .wr_data_count(curr_sqptr_fifo_dcount),
        .wr_rst_busy  (),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .sleep        (1'b0)
    );


    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),              //positive integer
        .BYTE_WRITE_WIDTH_A(16),              //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (16),               //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //string
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_duplicate_curr_sqptr (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (curr_sqptr_vld_r),
        .addra         (curr_sqptr_addr_r),
        .dina          (curr_sqptr_data_r),
        .injectsbiterra(1'b0),               //do not change
        .injectdbiterra(1'b0),               //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (1'b1),
        .regceb  (1'b1),
        .addrb   (sq_flag_updt_addr),
        .doutb   (curr_sqptr_flag_ptr),
        .sbiterrb(),                     //do not change
        .dbiterrb()                      //do not change

    );

    // 在写入空闲缓冲区时消除冲突
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            sq_pidb_enb_r      <= 1'b0;
            sq_pidb_vld_r      <= 1'b0;
            sq_pidb_typ_r      <= 1'b0;
            sq_pidb_addr_r     <= 'd0;
            sq_pidb_data_r     <= 'd0;
            sq_pidb_req_mask_r <= 1'b1;
        end else begin

            if (i_qp_sq_pidb_wr_valid_hndshk == 1'b1 && o_qp_sq_pidb_wr_rdy == 1'b0 && prog_full_sq_db_update == 1'b0) begin
                sq_pidb_req_mask_r <= 1'b0;
            end else if (o_qp_sq_pidb_wr_rdy == 1'b1) begin
                sq_pidb_req_mask_r <= 1'b1;
            end

            sq_pidb_vld_r  <= sq_pidb_vld_c && !prog_full_sq_db_update;
            sq_pidb_addr_r <= sq_pidb_addr_c;
            sq_pidb_data_r <= sq_pidb_data_c;

        end
    end

    assign sq_pidb_vld_c  = (i_qp_sq_pidb_wr_valid_hndshk == 1'b1 && sq_pidb_req_mask_r == 1'b1) ? 1'b1 : qp_sq_pidb_bram_wen;
    assign sq_pidb_addr_c = (i_qp_sq_pidb_wr_valid_hndshk == 1'b1 && sq_pidb_req_mask_r == 1'b1) ? qp_sq_pidb_hndshk_addr[C_QP_INDX_WIDTH-1:0] : qp_sq_pidb_wr_addr;
    assign sq_pidb_data_c = (i_qp_sq_pidb_wr_valid_hndshk == 1'b1 && sq_pidb_req_mask_r == 1'b1) ? i_qp_sq_pidb_hndshk : s_axi_lite_wdata[15:0];

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (16),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (11),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(5)
    ) u_db_update_sq_pidb (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (sq_pidb_vld_r),
        .din          (sq_pidb_addr_r),
        .rd_en        (sq_pidb_rden_r),
        .empty        (sq_pidb_fifo_empty),
        .dout         (sq_pidb_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (prog_full_sq_db_update),
        .rd_data_count(),
        .rd_rst_busy  (),
        .sbiterr      (),
        .underflow    (),
        .wr_ack       (),
        .wr_data_count(sq_pidb_fifo_dcount),
        .wr_rst_busy  (),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .sleep        (1'b0)
    );

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),              //positive integer
        .BYTE_WRITE_WIDTH_A(16),              //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (16),               //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //string
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_duplicate_sq_pidb (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (sq_pidb_vld_r),
        .addra         (sq_pidb_addr_r),
        .dina          (sq_pidb_data_r),
        .injectsbiterra(1'b0),            //do not change
        .injectdbiterra(1'b0),            //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (1'b1),
        .regceb  (1'b1),
        .addrb   (sq_flag_updt_addr),
        .doutb   (sq_pidb_flag_ptr),
        .sbiterrb(),                   //do not change
        .dbiterrb()                    //do not change

    );

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            flag_updt_cs      <= WAIT_FOR_UPDT;
            curr_sqptr_rden_r <= 1'b0;
            sq_pidb_rden_r    <= 1'b0;
            sq_flag_updt_addr <= 'd0;

            sq_flag_updt_vld  <= 1'b0;
            sq_flag_updt_idx  <= 'b0;
            //clr_wr_req_mask_r      <= 1'b0;
        end else begin
            case (flag_updt_cs)
                WAIT_FOR_UPDT: begin
                    if (curr_sqptr_rden_r == 1'b0 && curr_sqptr_fifo_empty == 1'b0 && (sq_pidb_fifo_empty == 1'b1 || curr_sqptr_fifo_dcount > sq_pidb_fifo_dcount)) begin
                        flag_updt_cs      <= READ_SQ_PTR;
                        curr_sqptr_rden_r <= 1'b1;
                        sq_pidb_rden_r    <= 1'b0;
                        //clr_wr_req_mask_r     <= curr_sqptr_fifo_dout[C_QP_INDX_WIDTH];
                        sq_flag_updt_addr <= curr_sqptr_fifo_dout[C_QP_INDX_WIDTH-1:0];
                    end else if (sq_pidb_rden_r == 1'b0 && sq_pidb_fifo_empty == 1'b0) begin
                        flag_updt_cs      <= READ_SQ_PTR;
                        sq_pidb_rden_r    <= 1'b1;
                        curr_sqptr_rden_r <= 1'b0;
                        //clr_wr_req_mask_r     <= sq_pidb_fifo_dout[C_QP_INDX_WIDTH];
                        sq_flag_updt_addr <= sq_pidb_fifo_dout[C_QP_INDX_WIDTH-1:0];
                    end else begin
                        sq_pidb_rden_r    <= 1'b0;
                        curr_sqptr_rden_r <= 1'b0;
                        //clr_wr_req_mask_r     <= 1'b0;
                        flag_updt_cs      <= WAIT_FOR_UPDT;
                    end
                end
                READ_SQ_PTR: begin
                    flag_updt_cs <= WAIT_FOR_UPDT;
                    if (curr_sqptr_rden_r == 1'b0 && curr_sqptr_fifo_empty == 1'b0) begin
                        curr_sqptr_rden_r <= 1'b1;
                        sq_pidb_rden_r    <= 1'b0;
                        //clr_wr_req_mask_r     <= curr_sqptr_fifo_dout[C_QP_INDX_WIDTH];
                        sq_flag_updt_addr <= curr_sqptr_fifo_dout[C_QP_INDX_WIDTH-1:0];
                    end else if (sq_pidb_rden_r == 1'b0 && sq_pidb_fifo_empty == 1'b0) begin
                        sq_pidb_rden_r    <= 1'b1;
                        curr_sqptr_rden_r <= 1'b0;
                        //clr_wr_req_mask_r     <= sq_pidb_fifo_dout[C_QP_INDX_WIDTH];
                        sq_flag_updt_addr <= sq_pidb_fifo_dout[C_QP_INDX_WIDTH-1:0];
                    end else begin
                        sq_pidb_rden_r    <= 1'b0;
                        curr_sqptr_rden_r <= 1'b0;
                        //clr_wr_req_mask_r     <= 1'b0;
                    end
                end

                default: begin
                    flag_updt_cs <= WAIT_FOR_UPDT;
                end
            endcase

            if (curr_sqptr_rden_r == 1'b1 || sq_pidb_rden_r == 1'b1) begin
                sq_flag_updt_vld <= 1'b1;
            end else begin
                sq_flag_updt_vld <= 1'b0;
            end

            sq_flag_updt_idx <= sq_flag_updt_addr;

        end
    end

    `MSFF_RL(pending_upd_cs, 3'b000, core_clk, ~core_rstn, 3'b000)

    assign dbg_sq_empty = (sq_pidb_flag_ptr == curr_sqptr_flag_ptr) ? 1'b1 : 1'b0;  // 相等时说明 SQ 中没有未处理的 WQE，该 QP 的 SQ 为空
    assign dbg_sq_full  = (sq_pidb_flag_ptr[C_QP_INDX_WIDTH] != curr_sqptr_flag_ptr[C_QP_INDX_WIDTH]) && (sq_pidb_flag_ptr[C_QP_INDX_WIDTH-1:0] == curr_sqptr_flag_ptr[C_QP_INDX_WIDTH-1:0]);   // 不仅比较指针数值，还带着一个额外的 MSB，用来区分“相等但已经绕了一圈”的情况，从而避免把 “满”误判成 “空”。

    always @(posedge core_clk) begin : dbg_sq_empty_ff  //and dbg_sq_full_ff
        if (!core_rstn) begin
            dbg_sq_empty_ff_valid <= 1'b0;
            dbg_sq_empty_ff_id    <= {C_QP_INDX_WIDTH{1'b0}};
            dbg_sq_empty_ff_data  <= 1'b0;
            dbg_sq_full_ff_valid  <= 1'b0;
            dbg_sq_full_ff_id     <= {C_QP_INDX_WIDTH{1'b0}};
            dbg_sq_full_ff_data   <= 1'b0;
        end else begin
            // i_sw_override_per_qp_en[C_QP_INDX_WIDTH] = o_xrnic_adv_conf_sw_override
            // i_sw_override_per_qp_en[C_QP_INDX_WIDTH-1 : 0] = o_xrnic_adv_conf_sw_override_qpid
            if (sq_flag_updt_vld && !(i_sw_override_per_qp_en[C_QP_INDX_WIDTH] && i_sw_override_per_qp_en[C_QP_INDX_WIDTH-1 : 0] == sq_flag_updt_idx)) begin
                dbg_sq_empty_ff_valid <= 1'b1;
                dbg_sq_empty_ff_id    <= sq_flag_updt_idx;
                dbg_sq_empty_ff_data  <= dbg_sq_empty;

                dbg_sq_full_ff_valid  <= 1'b1;
                dbg_sq_full_ff_id     <= sq_flag_updt_idx;
                dbg_sq_full_ff_data   <= dbg_sq_full;
            end else begin
                dbg_sq_empty_ff_valid <= 1'b0;
                dbg_sq_full_ff_valid  <= 1'b0;
            end
        end
    end

    //assign o_sq_full  = dbg_sq_full_ff;
    // 当重新计算出的 dbg_sq_empty_ff_* 与 inst_dbg_sq_empty_ff LUT RAM 里保存的旧值不同，就会把新值写回，并通过以下语句把“该 QP 的 SQ 空状态发生变化”这个事件通知外部模块。
    // 以下只报告“某一个 QP 的 SQ 空状态发生变化”
    assign o_sq_empty_valid = dbg_sq_empty_ff_valid;
    assign o_sq_empty_id    = dbg_sq_empty_ff_id;  // qpid
    assign o_sq_empty_data  = dbg_sq_empty_ff_data;  // 空-1，非空-0

    // ---- 所有 QP 的 SQ 当前是否都为空？ ----
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1),
        .C_COMBO_BYPASS(1)
    ) inst_dbg_sq_empty_ff_and_reduce (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || dbg_sq_empty_ff_valid),
        .wr_addr (init_start ? init_count : dbg_sq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : dbg_sq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (dbg_sq_empty_ff_id),
        .rd_data (dbg_sq_empty_ff_id_rd_data)
    );

    // 检测所有 QP 的 SQ 当前是否都为空
    //i_osq_almost_full And_reduce logic
    always @(posedge core_clk) begin : dbg_sq_empty_ff_and_reduce_proc
        if (~core_rstn) begin
            dbg_sq_empty_ff_zeros_count <= {C_QP_INDX_WIDTH + 1{1'b0}};
        end else begin
            case ({
                dbg_sq_empty_ff_valid, dbg_sq_empty_ff_data, dbg_sq_empty_ff_id_rd_data
            })
                3'b110:  dbg_sq_empty_ff_zeros_count <= dbg_sq_empty_ff_zeros_count - 1;
                3'b101:  dbg_sq_empty_ff_zeros_count <= dbg_sq_empty_ff_zeros_count + 1;
                default: dbg_sq_empty_ff_zeros_count <= dbg_sq_empty_ff_zeros_count;
            endcase
        end
    end
    assign dbg_sq_empty_ff_and_reduce = (dbg_sq_empty_ff_zeros_count == 0) ? 1'b1 : 1'b0;
    // ---- end ----

    generate

        if (C_EN_DEBUG_REGS == 1) begin
            //----Performance debug counters logic-------------------------------//
            //   global_dbg_cnt[0]  --> RWSC, Enable All Counters
            //   global_dbg_cnt[1]  --> RW, Clear All Counters
            //   global_dbg_cnt[15:2]-->RW, Reserved Bits
            //   global_dbg_cnt[31:16] -->RW, Global Count value
            //                         -- 0, counters are enabled all the time
            //                         -- N(non-zero), Counters are enabled for only N clocks

            always @(posedge core_clk) begin
                if (~core_rstn) begin
                    global_dbg_cnt[31:0] <= 32'h1;
                end else if (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r & generic_reg_wr_access & (wr_addr_r[10:2] == GLOBAL_DBG_CNTR[10:2])) begin
                    global_dbg_cnt[31:0] <= s_axi_lite_wdata[31:0];
                end else if ((|global_dbg_cnt[31:16]) && (int_cnt_value == (global_dbg_cnt[31:16] - 1'b1))) begin
                    global_dbg_cnt[0] <= 1'b0;  //This bit will be self cleared after global_dbg_cnt[31:16] of axi_lite_aclk's
                end
            end

            `MSFF_R(int_cnt_value, (global_dbg_cnt[0] ? int_cnt_value + 1 : 0), core_clk, ~core_rstn)



            always @(posedge core_clk) begin : osq_almost_full_proc
                if (!core_rstn) begin
                    osq_almost_full_valid         <= 1'b0;
                    osq_almost_full_id            <= {C_QP_INDX_WIDTH{1'b0}};
                    osq_almost_full_data          <= 1'b0;
                    osq_almost_full_valid_pending <= 1'b0;
                    osq_almost_full_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    if (i_osq_almost_full_set_valid) begin
                        osq_almost_full_valid <= 1'b1;
                        osq_almost_full_id    <= i_osq_almost_full_set_id;
                        osq_almost_full_data  <= 1'b1;
                    end else if (i_osq_almost_full_reset_valid) begin
                        osq_almost_full_valid <= 1'b1;
                        osq_almost_full_id    <= i_osq_almost_full_reset_id;
                        osq_almost_full_data  <= 1'b0;
                    end else if (osq_almost_full_valid_pending) begin
                        osq_almost_full_valid <= 1'b1;
                        osq_almost_full_id    <= osq_almost_full_id_pending;
                        osq_almost_full_data  <= 1'b0;
                    end else begin
                        osq_almost_full_valid <= 1'b0;
                    end

                    if (i_osq_almost_full_set_valid && i_osq_almost_full_reset_valid) begin
                        osq_almost_full_valid_pending <= 1'b1;
                        osq_almost_full_id_pending    <= i_osq_almost_full_reset_id;
                    end else begin
                        osq_almost_full_valid_pending <= 1'b0;
                    end
                end
            end

            sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP),
                .C_COMBO_BYPASS(1)
            ) inst_osq_almost_full_for_reduce (
                .core_clk(core_clk),
                .core_rst(~core_rstn),
                .wr_en   (init_start || osq_almost_full_valid),
                .wr_addr (init_start ? init_count : osq_almost_full_id),
                .wr_data (init_start ? 1'b0 : osq_almost_full_data),
                .rd_en   (1'b1),
                .rd_addr (osq_almost_full_id),
                .rd_data (osq_almost_full_data_rd_data)
            );

            sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP),
                .C_Init_with_1(1),
                .C_COMBO_BYPASS(1)
            ) inst_dbg_sq_empty_ff_for_reduce (
                .core_clk(core_clk),
                .core_rst(~core_rstn),
                .wr_en   (init_start || dbg_sq_empty_ff_valid),
                .wr_addr (init_start ? init_count : dbg_sq_empty_ff_id),
                .wr_data (init_start ? 1'b1 : dbg_sq_empty_ff_data),
                .rd_en   (1'b1),
                .rd_addr (dbg_sq_empty_ff_id),
                .rd_data (dbg_sq_empty_ff_rd_data_1)
            );

            always @(posedge core_clk) begin : or_reduce_for_expr1_2
                //expr1 = |(~dbg_sq_empty_ff & ~i_osq_almost_full) ; expr2 = |(~dbg_sq_empty_ff & i_osq_almost_full)
                if (!core_rstn) begin
                    expr1_num_ones <= {C_QP_INDX_WIDTH + 1{1'b0}};
                    expr2_num_ones <= {C_QP_INDX_WIDTH + 1{1'b0}};
                end else begin
                    if (dbg_sq_empty_ff_valid && osq_almost_full_valid) begin
                        case ({
                            (!dbg_sq_empty_ff_data && !osq_almost_full_data), (!dbg_sq_empty_ff_rd_data_1 && !osq_almost_full_data_rd_data)
                        })
                            2'b10:   expr1_num_ones <= expr1_num_ones + 1;
                            2'b01:   expr1_num_ones <= expr1_num_ones - 1;
                            default: expr1_num_ones <= expr1_num_ones;
                        endcase
                    end else if (dbg_sq_empty_ff_valid) begin
                        case ({
                            (!dbg_sq_empty_ff_data && !osq_almost_full_data_rd_data), (!dbg_sq_empty_ff_rd_data_1 && !osq_almost_full_data_rd_data)
                        })
                            2'b10:   expr1_num_ones <= expr1_num_ones + 1;
                            2'b01:   expr1_num_ones <= expr1_num_ones - 1;
                            default: expr1_num_ones <= expr1_num_ones;
                        endcase
                    end else if (osq_almost_full_valid) begin
                        case ({
                            (!dbg_sq_empty_ff_rd_data_1 && !osq_almost_full_data), (!dbg_sq_empty_ff_rd_data_1 && !osq_almost_full_data_rd_data)
                        })
                            2'b10:   expr1_num_ones <= expr1_num_ones + 1;
                            2'b01:   expr1_num_ones <= expr1_num_ones - 1;
                            default: expr1_num_ones <= expr1_num_ones;
                        endcase
                    end

                    if (dbg_sq_empty_ff_valid && osq_almost_full_valid) begin
                        case ({
                            (!dbg_sq_empty_ff_data && osq_almost_full_data), (!dbg_sq_empty_ff_rd_data_1 && osq_almost_full_data_rd_data)
                        })
                            2'b10:   expr2_num_ones <= expr2_num_ones + 1;
                            2'b01:   expr2_num_ones <= expr2_num_ones - 1;
                            default: expr2_num_ones <= expr2_num_ones;
                        endcase
                    end else if (dbg_sq_empty_ff_valid) begin
                        case ({
                            (!dbg_sq_empty_ff_data && osq_almost_full_data_rd_data), (!dbg_sq_empty_ff_rd_data_1 && osq_almost_full_data_rd_data)
                        })
                            2'b10:   expr2_num_ones <= expr2_num_ones + 1;
                            2'b01:   expr2_num_ones <= expr2_num_ones - 1;
                            default: expr2_num_ones <= expr2_num_ones;
                        endcase
                    end else if (osq_almost_full_valid) begin
                        case ({
                            (!dbg_sq_empty_ff_rd_data_1 && osq_almost_full_data), (!dbg_sq_empty_ff_rd_data_1 && osq_almost_full_data_rd_data)
                        })
                            2'b10:   expr2_num_ones <= expr2_num_ones + 1;
                            2'b01:   expr2_num_ones <= expr2_num_ones - 1;
                            default: expr2_num_ones <= expr2_num_ones;
                        endcase
                    end
                end
            end
            assign expr1_value          = (expr1_num_ones == 0) ? 1'b0 : 1'b1;
            assign expr2_value          = (expr2_num_ones == 0) ? 1'b0 : 1'b1;


            assign sqe_backpressure_cnt = global_dbg_cnt[0] ? ((expr1_value & i_qp_mgr_sts[1]) ? sqe_backpressure_cnt_ff + 1 : sqe_backpressure_cnt_ff) : sqe_backpressure_cnt_ff;
            assign osq_backpressure_cnt = global_dbg_cnt[0] ? (expr2_value ? osq_backpressure_cnt_ff + 1 : osq_backpressure_cnt_ff) : osq_backpressure_cnt_ff;

            assign all_sqes_empty_cnt   = global_dbg_cnt[0] ? (dbg_sq_empty_ff_and_reduce ? all_sqes_empty_cnt_ff + 1 : all_sqes_empty_cnt_ff) : all_sqes_empty_cnt_ff;

        end else begin

            always @(posedge core_clk) global_dbg_cnt <= 32'h0;
            assign sqe_backpressure_cnt = 16'h0;
            assign osq_backpressure_cnt = 16'h0;
            assign all_sqes_empty_cnt   = 16'h0;

        end
    endgenerate


    //---------------- ADDED for DEBUG can be removed later ------------------------
    `MSFF_RL(xrnic_adv_conf_sw_override_ff, xrnic_adv_conf_sw_override, core_clk, ~core_rstn, 3'b000)
    assign posedge_xrnic_sw_override = ~xrnic_adv_conf_sw_override_ff & xrnic_adv_conf_sw_override;
    `MSFF_RL(xrnic_sw_override_cnt, (posedge_xrnic_sw_override ? (xrnic_sw_override_cnt + 1'b1) : xrnic_sw_override_cnt), core_clk, ~core_rstn, 3'b000)
    //------------------------------------------------------------------------------

    assign o_dbg_cnt_override = xrnic_sw_override_cnt;

    //`MSFF_R(hw_hndshk_disable_to_0, (qp_conf_bram_wen_c & ~s_axi_lite_wdata[4]), core_clk, ~core_rstn)
    `MSFF_R(hw_hndshk_disable_to_0, (qp_conf_bram_wen & ~wr_data_mem_r[4]), core_clk, ~core_rstn)

    `MSFF_R(qp_conf_req_rx_pkt_ff, i_qp_conf_req_rx_pkt, core_clk, ~core_rstn)
    `MSFF_R(qp_conf_req_replica_ff, i_qp_conf_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_conf_req_wqe_proc_ff, i_qp_conf_req_wqe_proc & ~i_qp_conf_req_rx_pkt, core_clk, ~core_rstn)
    `MSFF_R(qp_conf_req_resp_hndl_ff, (i_qp_conf_req_resp_hndl & ~i_qp_conf_req_rx_pkt & ~i_qp_conf_req_wqe_proc), core_clk, ~core_rstn)
    `MSFF_R(qp_cfg_retry_cnt_rd_req_ff, (i_qp_cfg_retry_cnt_rd_req & ~i_qp_conf_req_rx_pkt & ~i_qp_conf_req_wqe_proc), core_clk, ~core_rstn)
    `MSFF_R(qp_adv_conf_req_ff, i_qp_adv_conf_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_pi_db_req_ff, i_qp_rq_pi_db_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ret_sq_psn_req_ff, i_qp_ret_sq_psn_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ret_sq_psn_req_int_ff, i_qp_ret_sq_psn_wrn & ~i_qp_ret_sq_psn_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_ba_req_ff, i_qp_rq_ba_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_ba_req_ff, i_qp_sq_ba_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_ba_req_int_ff, (i_qp_sq_ba_req_int & ~i_qp_sq_ba_req), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_ba_req_ff, i_qp_cq_ba_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_wrptrdb_req_ff, i_qp_rq_wrptrdb_add_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_cmpldb_req_ff, i_qp_sq_cmpldb_add_req, core_clk, ~core_rstn)
    `MSFF_R(qp_cq_hdptr_req_ff, (i_qp_cq_hdptr_req & ~i_qp_cq_hdptr_retry_req), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_hdptr_retry_req_ff, i_qp_cq_hdptr_retry_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_cidb_req_ff, i_qp_rq_cidb_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_cidb_req_hndshk_ff, i_qp_rq_cidb_wr_valid_hndshk & ~i_qp_rq_cidb_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_pidb_req_ff, i_qp_sq_pidb_req, core_clk, ~core_rstn)
    //`MSFF_R(qp_sq_pidb_req_hndshk_ff        , i_qp_sq_pidb_wr_valid_hndshk & ~i_qp_sq_pidb_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_pidb_req_hndshk_ff, (i_qp_sq_pidb_wr_valid_hndshk && !i_qp_sq_pidb_req && !prog_full_sq_db_update), core_clk, ~core_rstn)
    `MSFF_R(qp_rq_depth_req_ff, i_qp_rq_depth_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_depth_req_ff, (i_qp_sq_depth_req & ~i_qp_cq_depth_req), core_clk, ~core_rstn)
    // RX 路径的请求
    `MSFF_R(qp_sq_psn_req_ff, i_qp_sq_psn_req, core_clk, ~core_rstn)
    // SQ WQE 路径(qp_mgr 与 wqe_proc)的请求
    // 只有当 SQ WQE 路径请求有效且 RX 路径没有同时请求(读取 SQPSNi 寄存器)时，给 WQE 路径的 valid 才为高
    `MSFF_R(qp_sq_psn_wqe_req_ff, (i_qp_sq_psn_wqe_req & ~i_qp_sq_psn_req), core_clk, ~core_rstn)
    // 重传路径(qp_mgr_retransmit)的请求
    // 只有当 重传路径请求有效且 RX 路径没有同时请求(读取 SQPSNi 寄存器)时，给重传路径的 valid 才为高
    `MSFF_R(qp_sq_psn_retry_req_ff, (i_qp_sq_psn_retry_req & ~i_qp_sq_psn_req), core_clk, ~core_rstn)
    `MSFF_R(qp_last_rq_req_ff, i_qp_last_rq_req, core_clk, ~core_rstn)
    `MSFF_R(pd_virt_addr_0_ff, i_pd_virt_addr_0_req, core_clk, ~core_rstn)
    `MSFF_R(pd_virt_addr_1_ff, i_pd_virt_addr_1_req, core_clk, ~core_rstn)
    `MSFF_R(pd_buf_base_addr_0_ff, i_pd_buf_base_addr_0_req, core_clk, ~core_rstn)
    `MSFF_R(pd_buf_base_addr_1_ff, i_pd_buf_base_addr_1_req, core_clk, ~core_rstn)
    `MSFF_R(pd_buf_r_key_ff, i_pd_buf_r_key_req, core_clk, ~core_rstn)
    `MSFF_R(pd_w_r_buf_len_ff, i_pd_w_r_buf_len_req, core_clk, ~core_rstn)
    `MSFF_R(qp_dest_qpid_req_wqe_proc_ff, i_qp_dest_qpid_req_wqe_proc, core_clk, ~core_rstn)
    `MSFF_R(qp_dest_qpid_req_rx_pkt_ff, i_qp_dest_qpid_req_rx_pkt, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrl_req_ff, i_qp_mac_remote_addrl_req, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrl_req_replica_ff, i_qp_mac_remote_addrl_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrm_req_ff, i_qp_mac_remote_addrm_req, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrm_req_replica_ff, i_qp_mac_remote_addrm_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrl_wqe_req_ff, (i_qp_mac_remote_addrl_wqe_req & ~i_qp_mac_remote_addrl_req), core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrm_wqe_req_ff, (i_qp_mac_remote_addrm_wqe_req & ~i_qp_mac_remote_addrm_req), core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr1_req_ff, i_qp_ip_remote_addr1_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr1_req_replica_ff, i_qp_ip_remote_addr1_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr2_req_ff, i_qp_ip_remote_addr2_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr2_req_replica_ff, i_qp_ip_remote_addr2_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr3_req_ff, i_qp_ip_remote_addr3_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr3_req_replica_ff, i_qp_ip_remote_addr3_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr4_req_ff, i_qp_ip_remote_addr4_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr4_req_replica_ff, i_qp_ip_remote_addr4_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr1_wqe_req_ff, (i_qp_ip_remote_addr1_wqe_req & ~i_qp_ip_remote_addr1_req), core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr2_wqe_req_ff, (i_qp_ip_remote_addr2_wqe_req & ~i_qp_ip_remote_addr2_req), core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr3_wqe_req_ff, (i_qp_ip_remote_addr3_wqe_req & ~i_qp_ip_remote_addr3_req), core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr4_wqe_req_ff, (i_qp_ip_remote_addr4_wqe_req & ~i_qp_ip_remote_addr4_req), core_clk, ~core_rstn)
    `MSFF_R(qp_curr_sqptr_req_ff, i_qp_curr_sqptr_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_resp_psn_req_ff, i_qp_stat_resp_psn_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_msn_req_ff, i_qp_stat_msn_req, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_ssn_req_ff, i_qp_stat_ssn_req, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_nak_req_ff, i_qp_stat_nak_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_rq_buf_ca_rdreq_ff, i_qp_stat_rq_buf_ca_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_wqe_cnt_rdreq_ff, i_qp_stat_wqe_cnt_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_wrptrdb_add_req_ff, i_qp_rq_wrptrdb_add_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_cmpldb_add_req_ff, i_qp_sq_cmpldb_add_req, core_clk, ~core_rstn)
    `MSFF_R(qp_retry_cnt_req_ff, i_qp_stat_retry_cnt_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_retry_cnt_req_2ff, qp_retry_cnt_req_ff, core_clk, ~core_rstn)

    // SQ depth request from cache module has higher priority
    `MSFF_R(qp_cq_depth_req_ff, i_qp_cq_depth_req, core_clk, ~core_rstn)

    //----Performance debug counters FF stages-------------------------------//
    `MSFF_R(sqe_backpressure_cnt_ff, (global_dbg_cnt[1] ? 15'h0 : sqe_backpressure_cnt), core_clk, ~core_rstn)
    `MSFF_R(osq_backpressure_cnt_ff, (global_dbg_cnt[1] ? 15'h0 : osq_backpressure_cnt), core_clk, ~core_rstn)
    `MSFF_R(all_sqes_empty_cnt_ff, (global_dbg_cnt[1] ? 15'h0 : all_sqes_empty_cnt), core_clk, ~core_rstn)

    always @(posedge core_clk)
        if (~core_rstn) o_qp_sq_psn_resp_valid <= 'h0;
        else if (i_qp_sq_psn_req | i_qp_sq_psn_wqe_req | i_qp_sq_psn_retry_req) o_qp_sq_psn_resp_valid <= 'h0;
        else if (i_qp_sq_psn_wrn | i_qp_sq_psn_retry_wrn) o_qp_sq_psn_resp_valid <= 'h0;
        else o_qp_sq_psn_resp_valid <= 'h1;



    assign o_qp_sq_cmpldb_add_valid            = qp_sq_cmpldb_add_req_ff;
    assign o_qp_rq_wrptrdb_add_valid           = qp_rq_wrptrdb_add_req_ff;
    assign o_qp_cq_hdptr_valid                 = qp_cq_hdptr_req_ff;
    assign o_qp_cq_hdptr_valid_retry           = qp_cq_hdptr_retry_req_ff;
    assign o_qp_conf_valid_wqe_proc            = qp_conf_req_wqe_proc_ff;
    assign o_qp_conf_valid_resp_hndl           = qp_conf_req_resp_hndl_ff;
    assign o_qp_cfg_retry_cnt_rd_rdy           = qp_cfg_retry_cnt_rd_req_ff;  //same as valid added for QP_fatal case
    assign o_qp_conf_valid_rx_pkt              = qp_conf_req_rx_pkt_ff;
    assign o_qp_conf_replica_valid             = qp_conf_req_replica_ff;
    assign o_qp_adv_conf_valid                 = qp_adv_conf_req_ff;
    assign o_qp_rq_pi_db_valid                 = qp_rq_pi_db_req_ff;
    assign o_qp_ret_sq_psn_valid               = qp_ret_sq_psn_req_ff;
    assign o_qp_ret_sq_psn_valid_int           = qp_ret_sq_psn_req_int_ff;
    assign o_qp_rq_ba_valid                    = qp_rq_ba_req_ff;
    assign o_qp_sq_ba_valid                    = qp_sq_ba_req_ff;
    assign o_qp_sq_ba_valid_int                = qp_sq_ba_req_int_ff;
    assign o_qp_cq_ba_valid                    = qp_cq_ba_req_ff;
    assign o_qp_rq_wrptrdb_valid               = qp_rq_wrptrdb_req_ff;
    assign o_qp_sq_cmpldb_valid                = qp_sq_cmpldb_req_ff;
    assign o_qp_rq_cidb_valid                  = qp_rq_cidb_req_ff;
    assign o_qp_rq_cidb_wr_rdy                 = qp_rq_cidb_req_hndshk_ff;
    assign o_qp_sq_pidb_valid                  = qp_sq_pidb_req_ff;
    assign o_qp_sq_pidb_wr_rdy                 = qp_sq_pidb_req_hndshk_ff & i_qp_sq_pidb_wr_valid_hndshk;
    assign o_qp_rq_depth_valid                 = qp_rq_depth_req_ff;
    assign o_qp_sq_depth_valid                 = qp_sq_depth_req_ff;
    assign o_qp_cq_depth_valid                 = qp_cq_depth_req_ff;
    assign o_qp_sq_psn_valid                   = qp_sq_psn_req_ff;
    assign o_qp_sq_psn_wqe_valid               = qp_sq_psn_wqe_req_ff;
    assign o_qp_sq_psn_retry_valid             = qp_sq_psn_retry_req_ff & ~i_qp_sq_psn_req;
    assign o_qp_last_rq_valid                  = qp_last_rq_req_ff;
    assign o_pd_virt_addr_0_valid              = pd_virt_addr_0_ff;
    assign o_pd_virt_addr_1_valid              = pd_virt_addr_1_ff;
    assign o_pd_buf_base_addr_0_valid          = pd_buf_base_addr_0_ff;
    assign o_pd_buf_base_addr_1_valid          = pd_buf_base_addr_1_ff;
    assign o_pd_buf_r_key_valid                = pd_buf_r_key_ff;
    assign o_pd_w_r_buf_len_valid              = pd_w_r_buf_len_ff;
    assign o_qp_dest_qpid_valid_wqe_proc       = qp_dest_qpid_req_wqe_proc_ff;
    assign o_qp_dest_qpid_valid_rx_pkt         = qp_dest_qpid_req_rx_pkt_ff;
    assign o_qp_mac_remote_addrl_valid         = qp_mac_remote_addrl_req_ff;
    assign o_qp_mac_remote_addrl_replica_valid = qp_mac_remote_addrl_req_replica_ff;
    assign o_qp_mac_remote_addrm_valid         = qp_mac_remote_addrm_req_ff;
    assign o_qp_mac_remote_addrm_replica_valid = qp_mac_remote_addrm_req_replica_ff;
    assign o_qp_mac_remote_addrl_wqe_valid     = qp_mac_remote_addrl_wqe_req_ff;
    assign o_qp_mac_remote_addrm_wqe_valid     = qp_mac_remote_addrm_wqe_req_ff;
    assign o_qp_ip_remote_addr1_valid          = qp_ip_remote_addr1_req_ff;
    assign o_qp_ip_remote_addr1_replica_valid  = qp_ip_remote_addr1_req_replica_ff;
    assign o_qp_ip_remote_addr2_valid          = qp_ip_remote_addr2_req_ff;
    assign o_qp_ip_remote_addr2_replica_valid  = qp_ip_remote_addr2_req_replica_ff;
    assign o_qp_ip_remote_addr3_valid          = qp_ip_remote_addr3_req_ff;
    assign o_qp_ip_remote_addr3_replica_valid  = qp_ip_remote_addr3_req_replica_ff;
    assign o_qp_ip_remote_addr4_valid          = qp_ip_remote_addr4_req_ff;
    assign o_qp_ip_remote_addr4_replica_valid  = qp_ip_remote_addr4_req_replica_ff;
    assign o_qp_ip_remote_addr1_wqe_valid      = qp_ip_remote_addr1_wqe_req_ff;
    assign o_qp_ip_remote_addr2_wqe_valid      = qp_ip_remote_addr2_wqe_req_ff;
    assign o_qp_ip_remote_addr3_wqe_valid      = qp_ip_remote_addr3_wqe_req_ff;
    assign o_qp_ip_remote_addr4_wqe_valid      = qp_ip_remote_addr4_wqe_req_ff;
    assign o_qp_curr_sqptr_proc_valid          = qp_curr_sqptr_req_ff;
    assign o_qp_stat_resp_psn_valid            = qp_stat_resp_psn_req_ff;
    assign o_qp_stat_rq_buf_ca_valid           = qp_stat_nak_req_ff;
    assign o_qp_stat_nak_valid                 = qp_stat_rq_buf_ca_rdreq_ff;
    assign o_qp_stat_wqe_cnt_valid             = qp_stat_wqe_cnt_rdreq_ff;
    assign o_qp_stat_msn_valid                 = qp_stat_msn_req_ff;
    assign o_qp_stat_ssn_valid                 = qp_stat_ssn_req_ff;
    assign o_qp_stat_retry_cnt_valid           = qp_retry_cnt_req_ff & ~qp_retry_cnt_req_2ff;

    assign o_xrnic_en                          = xrnic_conf_xrnic_en;
    assign o_tx_ack_gen                        = xrnic_conf_tx_ack_gen;
    assign o_err_buf_en                        = xrnic_conf_err_buf_en;
    assign o_flow_credits                      = xrnic_conf_flow_credits;
    assign o_depkt_bypass_en                   = xrnic_conf_depkt_bypass_en;
    assign o_num_qp_en                         = xrnic_conf_num_qps_enabled;
    assign o_xrnic_udp_src_port                = xrnic_conf_udp_src_port;
    assign o_ipv4_xrnic_addr                   = ipv4_xrnic_addr;
    assign o_ipv6_xrnic_addr                   = {ipv6_xrnic_addr4, ipv6_xrnic_addr3, ipv6_xrnic_addr2, ipv6_xrnic_addr1};
    assign o_mac_xrnic_addr                    = {mac_xrnic_addr_msb, mac_xrnic_addr_lsb};
    assign roce_pause_on_offset                = roce_pause_on_offset_reg;
    assign roce_pause_off_offset               = roce_pause_off_offset_reg;
    assign non_roce_pause_on_offset            = non_roce_pause_on_offset_reg;
    assign non_roce_pause_off_offset           = non_roce_pause_off_offset_reg;
    assign pfc_en_roce                         = pfc_en_roce_reg;
    assign pfc_en_non_roce                     = pfc_en_non_roce_reg;
    assign roce_priority                       = roce_priority_reg;
    assign non_roce_priority                   = non_roce_priority_reg;
    assign disable_priority_chk                = disable_priority_chk_reg;
    assign o_timeoutreg                        = {11'b0, rnr_nack_tval, 2'b0, rnr_retry_cnt, retry_cnt, 3'b0, timeout};  // {18'b0, 3'b1, 3'b1, 8'b0}

    assign o_bypass_buf_ba                     = bypass_buf_ba;
    assign o_bypass_buf_sz_num_bufs            = bypass_buf_sz_num_bufs;
    assign o_bypass_buf_sz_buf_sz              = bypass_buf_sz_buf_sz;

    assign o_data_buf_ba                       = {data_buf_ba_msb, data_buf_ba};
    assign o_data_buf_sz_num_bufs              = data_buf_sz_num_bufs;
    assign o_data_buf_sz_buf_sz                = data_buf_sz_buf_sz;

    always @(posedge core_clk) o_connect_io_qp_rq_pi_db_wptr <= connect_io_qp_rq_pi_db_wptr;
    //assign o_connect_io_qp_rq_pi_db_wptr = connect_io_qp_rq_pi_db_wptr;
    assign o_connect_io_qpid              = connect_io_qpid[C_QP_INDX_WIDTH-1 : 0];
    //assign o_connect_io_residual_rq = connect_io_residual_rq;
    assign o_hw_hndshk_disable_to_0       = hw_hndshk_disable_to_0 & xrnic_adv_conf_sw_override;  // 0 & 0

    assign o_rq_err_pkt_buf_ba            = {rq_err_pkt_buf_ba_msb, rq_err_pkt_buf_ba};
    assign o_rq_err_pkt_buf_sz_num_bufs   = rq_err_pkt_buf_sz_num_bufs;
    assign o_rq_err_pkt_buf_sz_buf_sz     = rq_err_pkt_buf_sz_buf_sz;

    assign o_resp_err_pkt_buf_ba          = {resp_err_pkt_buf_msb_ba, resp_err_pkt_buf_ba};
    assign o_resp_err_pkt_buf_sz_num_bufs = resp_err_pkt_buf_sz_num_bufs;
    assign o_resp_err_pkt_buf_sz_buf_sz   = resp_err_pkt_buf_sz_buf_sz;

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            o_error_corrupt_conf <= 32'd0;
        end else begin
            o_error_corrupt_conf[31:1] <= error_corrupt_conf[31:1];
            if (error_corrupt_conf[0]) begin
                o_error_corrupt_conf[0] <= 1'b1;
            end else if (clr_error_corrupt_en) begin
                o_error_corrupt_conf[0] <= 1'b0;
            end
        end
    end

    //assign o_tx_hdr_buf_ba = tx_hdr_buf_ba;
    assign o_tx_hdr_buf_sz_num_hdrs             = tx_hdr_buf_sz_num_hdrs;
    assign o_tx_hdr_buf_sz_buf_sz               = tx_hdr_buf_sz_buf_sz;

    assign o_tx_sgl_buf_ba                      = tx_sgl_buf_ba;
    assign o_tx_sgl_buf_sz_num_sgls             = tx_sgl_buf_sz_num_sgls;
    assign o_tx_sgl_buf_sz_buf_sz               = tx_sgl_buf_sz_buf_sz;

    assign o_xrnic_conf_ipver                   = xrnic_conf_ipver;
    assign o_xrnic_adv_conf_sw_override         = xrnic_adv_conf_sw_override;
    assign o_xrnic_adv_conf_sw_override_qpid    = xrnic_adv_conf_sw_override_qpid;
    assign o_xrnic_adv_conf_errbuf_overwr_en    = xrnic_adv_conf_errbuf_overwr_en;
    assign o_xrnic_adv_conf_retry_cnt_fatal_dis = xrnic_adv_conf_retry_cnt_fatal_dis;
    assign o_xrnic_adv_base_cnt                 = xrnic_adv_base_cnt;

    assign o_intr_en_pkt_valdn_err              = intr_en_pkt_valdn_err;
    assign o_intr_en_wqe_cmpl                   = intr_en_wqe_cmpl;
    assign o_intr_en_mad_pkt_rcvd               = intr_en_mad_pkt_rcvd;
    assign o_intr_en_bypass_pkt_rcvd            = intr_en_bypass_pkt_rcvd;
    assign o_intr_en_rnr_nak_gen                = intr_en_rnr_nak_gen;
    assign o_intr_en_ill_opc_in_sq              = intr_en_ill_opc_in_sq;
    assign o_intr_en_fatal_err                  = intr_en_fatal_err;
    assign o_intr_en_cnp                        = intr_en_cnp;
    assign o_intr_en_qp_pkt_rcvd                = intr_en_qp_pkt_rcvd;


    assign o_out_errsts_q_ba                    = {out_errsts_q_ba_msb, out_errsts_q_ba};
    assign o_out_errsts_q_sz                    = out_errsts_q_sz;

    assign o_in_errsts_q_ba                     = {in_errsts_q_ba_msb, in_errsts_q_ba};
    assign o_in_errsts_q_sz                     = in_errsts_q_sz;

    assign o_rx_pkt_hndl_dbg_ctrl               = rx_pkt_hndl_dbg_ctrl;

    assign o_global_dbg_cnt_value               = global_dbg_cnt[31:16];
    assign o_global_dbg_cnt_clr                 = global_dbg_cnt[1];
    assign o_global_dbg_cnt_en                  = global_dbg_cnt[0];


    //-------------------------------------------------------------------------------------
    // Logic to count the outgoing read response packets per second
    //-------------------------------------------------------------------------------------

    reg  [15:0] rd_resp_pkt_cnt_r1;
    reg         clear_rd_resp_cnt_per_sec_r1;
    wire        first_rd_resp;
    wire        rd_resp_pkt_transmitted;
    reg         count_enable;
    reg  [31:0] tick_counter;

    reg  [15:0] rd_req_pkt_cnt_r1;
    reg         clear_rd_req_cnt_per_sec_r1;
    wire        first_rd_req;
    wire        rd_req_pkt_rcvd;
    reg         rd_req_count_enable;
    reg  [31:0] rd_req_tick_counter;

    always @(posedge core_clk) rd_resp_pkt_cnt_r1 <= i_out_rd_resp_pkt_cnt;

    always @(posedge core_clk) rd_req_pkt_cnt_r1 <= i_good_rd_req_cnt;

    always @(posedge core_clk) clear_rd_resp_cnt_per_sec_r1 <= clear_rd_resp_cnt_per_sec;

    always @(posedge core_clk) clear_rd_req_cnt_per_sec_r1 <= clear_rd_req_cnt_per_sec;

    assign first_rd_resp           = (i_out_rd_resp_pkt_cnt == 15'd1) && (rd_resp_pkt_cnt_r1 == 15'd0);
    assign rd_resp_pkt_transmitted = (i_out_rd_resp_pkt_cnt != rd_resp_pkt_cnt_r1);

    assign first_rd_req            = (i_good_rd_req_cnt == 31'd1) && (rd_req_pkt_cnt_r1 == 31'd0);
    assign rd_req_pkt_rcvd         = (i_good_rd_req_cnt != rd_req_pkt_cnt_r1);


    always @(posedge core_clk) begin
        if (~core_rstn) tick_counter <= 0;
        else if (count_enable) tick_counter <= tick_counter + 1;
    end

    always @(posedge core_clk) begin
        if (~core_rstn) count_enable <= 0;
        else if (first_rd_resp) count_enable <= 1;
        else if (tick_counter == cycle_count) count_enable <= 0;
    end

    always @(posedge core_clk) begin
        if (~core_rstn) rd_resp_pkt_cnt_per_sec <= 0;
        else if (clear_rd_resp_cnt_per_sec_r1 != clear_rd_resp_cnt_per_sec) rd_resp_pkt_cnt_per_sec <= 0;
        else if (count_enable & rd_resp_pkt_transmitted) rd_resp_pkt_cnt_per_sec <= rd_resp_pkt_cnt_per_sec + 1;
    end

    always @(posedge core_clk) begin
        if (~core_rstn) rd_req_tick_counter <= 0;
        else if (rd_req_count_enable) rd_req_tick_counter <= rd_req_tick_counter + 1;
    end

    always @(posedge core_clk) begin
        if (~core_rstn) rd_req_count_enable <= 0;
        else if (first_rd_req) rd_req_count_enable <= 1;
        else if (rd_req_tick_counter == cycle_count) rd_req_count_enable <= 0;
    end

    always @(posedge core_clk) begin
        if (~core_rstn) rd_req_pkt_cnt_per_sec <= 0;
        else if (clear_rd_req_cnt_per_sec_r1 != clear_rd_req_cnt_per_sec) rd_req_pkt_cnt_per_sec <= 0;
        else if (rd_req_count_enable & rd_req_pkt_rcvd) rd_req_pkt_cnt_per_sec <= rd_req_pkt_cnt_per_sec + 1;
    end

    //-------------------------------------------------------------------------------------

endmodule
