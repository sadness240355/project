///////////////////////////////////////////////////////////////////
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
// resp_handler_fsm.v
//-----------------------------------------------------------------------------
//
// Filename        : resp_handler_fsm.v
// Version         : v1.0
// Description     : Module managing the response handler FSM
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
// 管理响应处理程序 FSM 的模块，FSM 只发“我要下一个QP”的请求。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module resp_handler_fsm #(
    parameter C_NUM_QP                                 = 256,
    parameter C_M_AXI_ADDR_WIDTH                       = 32,
    parameter C_QP_INDX_WIDTH                          = 8,
    parameter C_EN_NVMOF_HW_HNDSHK                     = 1,    // Enabled hardware handshake with NVMoF
    parameter C_OS_Q_DEPTH                             = 8,
    parameter IP2BUS_LEN_WIDTH                         = 7,
    parameter C_OSQ_PSN_WIDTH                          = 10,
    //parameter   C_EN_WR_RETRY_DATA_BUF	= 1,
    parameter NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE     = 1,
    parameter NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN   = 0,
    parameter NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING = 0,
    parameter C_OS_Q_INDX_WIDTH                        = 3
) (
    input wire core_clk,  // Module Clock  
    input wire core_rstn, // Active low core reset

    input  wire                         i_acks_to_process,               // 为高表示有OSQ非空，指示当前有 ACK/NACK 相关事件可处理
    input  wire                         i_osq_rnr_nacked,                // QP RNR nacked；当前仲裁 QP 是否处于 RNR NACK 状态
    output reg                          o_clr_timeout_n_nack_set_valid,  // 为 i_osq_timed_out 和 i_osq_nacked 清零
    output reg  [ C_QP_INDX_WIDTH -1:0] o_clr_timeout_n_nack_set_id,     // 为 i_osq_timed_out 和 i_osq_nacked 清零
    output reg                          o_clr_pending_ack_set_valid,
    output reg  [C_QP_INDX_WIDTH-1 : 0] o_clr_pending_ack_set_id,

    output wire                        o_arbitrate,                  // 请求上游仲裁器挑一个 OSQ/QP 供 FSM 处理
    output reg                         fsm_pushback_valid,           // 当前 QP 暂缓处理、要求回推
    input  wire                        i_arbitration_done,           // 仲裁结果有效
    input  wire [C_QP_INDX_WIDTH -1:0] i_arbitrated_indx,            // 仲裁出的QP号
    output wire [C_QP_INDX_WIDTH -1:0] o_sampled_arbitrated_indx,
    output wire [C_QP_INDX_WIDTH -1:0] o_arbitrated_qp_for_intr,     // 当前 CQE 处理 QP(o_qp_cqe_proc_qpid)
    output wire                        o_phase_to_update_post_retry,

    output wire [                31:0] o_resp_hndler_sts,
    output wire [                31:0] o_stat_retry_count,
    input  wire                        i_cfg_retry_cnt_fatal_dis,
    input  wire                        i_bypass_en,
    input  wire                        i_qp_fatal_valid,
    input  wire [C_QP_INDX_WIDTH -1:0] i_qp_fatal_id,
    input  wire                        i_qp_fatal_data,
    input  wire                        i_dma_in_idle,
    input  wire                        i_en_osq_flush_on_qp_fatal,

    input wire                         i_first_outgoing_pkt_valid,
    input wire [C_QP_INDX_WIDTH-1 : 0] i_first_outgoing_pkt_id,
    input wire                         i_first_outgoing_pkt_data,
    input wire                         i_first_outgoing_pkt_valid_pending,
    input wire [C_QP_INDX_WIDTH-1 : 0] i_first_outgoing_pkt_id_pending,
    input wire                         i_first_outgoing_pkt_data_pending,
    input wire [  C_OSQ_PSN_WIDTH-1:0] i_wqe_send_start_psn,
    input wire [  C_QP_INDX_WIDTH-1:0] i_wqe_qpid,                          // QP ID of WQE
    input wire                         i_wqe_push_req,                      // request from WQE Processor to push WQE

    /* 
     * OSQ = Outstanding SQ Queue（每个 QP 的“未完成 SQ 队列”）,它存的是“已经发出去、但还没最终完成/确认”的请求条目
     * resp_handler_fsm 就靠它(OSQ)来决定:
     * 1、当前条目是写完成 CQE，还是要 RETRY
     * 2、何时 pop（把该条目标记处理完）
     * 3、QP 出现 NACK/RNR/TIMEOUT/FATAL 时怎么清队列和回退
     */
    output wire                       o_pop_osq,
    // o_rd_osq / i_rd_osq_vld 触发并等待 OSQ 条目数据有效
    output reg                        o_rd_osq,
    input  wire                       i_rd_osq_vld,
    //output wire                                o_pop_mpsn,
    output reg                        o_flush_mpsn,
    input  wire                       i_osq_empty,
    input  wire                       i_mpsnbuf_flush_done,
    //input  wire                                i_mpsnbuf_empty,
    input  wire                       i_phase_bit_acknack_psn,
    input  wire [                7:0] i_osq_entry_msn,
    input  wire [C_OSQ_PSN_WIDTH-1:0] i_osq_entry_end_psn,
    input  wire [C_OSQ_PSN_WIDTH-1:0] i_osq_entry_start_psn,
    // 和 OSQ 条目(osq_entry) PSN 比较，决定"完成"还是"重传/回推"
    input  wire [               23:0] i_osq_acknacked_psn,
    input  wire [               23:0] i_osq_acknacked_msn,
    input  wire [               23:0] i_osq_last_acknacked_psn,
    // 
    input  wire [                7:0] i_osq_opcode,              // BTH.Opcode of outbound request
    input  wire [               15:0] i_osq_wrid,
    input  wire [               14:0] i_osq_wqe_data_bufid,
    input  wire                       i_osq_wqe_retried,
    input  wire                       i_osq_wqe_fatal_push,
    input  wire                       i_osq_nacked,
    input  wire                       i_osq_wqe_nack_resp,
    output reg  [               23:0] o_psn_to_retry,
    output reg  [               23:0] o_msn_to_retry,
    output reg                        o_freeup_data_buf,         // Freeup RDMA WRITE DATA buffer
    output reg  [               14:0] o_freeup_data_bufid,       // Freeup RDMA WRITE DATA buffer
    // ACK状态读握手接口
    output reg                        acknacked_psn_rd_req,      // FSM 发起读取当前 QP acknacked 状态
    input                             acknacked_psn_rd_req_done, // 读取完成，允许“使用读取结果做决策”。

    // QP details
    output wire                           o_qp_cq_ba_req,
    input  wire [C_M_AXI_ADDR_WIDTH -1:0] i_qp_cq_ba,                 // CQ base address
    input  wire                           i_qp_cq_ba_valid,
    output wire                           o_qp_cq_depth_req,
    input  wire                           i_qp_cq_depth_valid,        // CQ depth
    input  wire [                   15:0] i_qp_cq_depth,
    output wire                           o_qp_sq_cmpldb_addr_req,
    input  wire                           i_qp_sq_cmpldb_addr_valid,  // SQ Completion DB address (points to RDMAif register)
    input  wire [C_M_AXI_ADDR_WIDTH -1:0] i_qp_sq_cmpldb_addr,

    output wire                          o_qp_cq_hdptr_req,
    input  wire                          i_qp_cq_hdptr_valid,  // CQ head pointer
    input  wire [                  15:0] i_qp_cq_hdptr,
    output reg  [                  15:0] o_qp_cq_hdptr,
    output reg                           o_qp_cq_hdptr_wrn,
    output reg  [C_QP_INDX_WIDTH -1 : 0] o_qp_cqe_proc_qpid,   // QP id for this retransmission is required

    output wire        o_qp_conf_req,
    input  wire        i_qp_conf_valid,
    input  wire [31:0] i_qp_conf,

    output wire                           o_qp_sq_ba_req,
    input  wire                           i_qp_sq_ba_valid,
    input  wire [C_M_AXI_ADDR_WIDTH -1:0] i_qp_sq_ba,

    input wire                          i_rd_rsp_wr_done,
    input wire [   C_QP_INDX_WIDTH-1:0] i_rd_rsp_qpid,
    input wire [C_M_AXI_ADDR_WIDTH-1:0] i_rd_rsp_dest_addr,

    // Generic config
    input wire [C_M_AXI_ADDR_WIDTH -1:0] i_data_buf_ba,  // DATBUFBA 寄存器: data buffer base address
    input wire [                   15:0] i_data_buf_sz,  // DATBUFSZ 寄存器[31:16]: data buffer 大小

    // Completion interrupt
    input  wire i_intr_clr_wqe_cmpl,
    output wire o_intr_wqe_cmpl_sts,

    // Interface to QP manager for retrnsmission 
    output reg  [C_QP_INDX_WIDTH -1 : 0] o_retransmit_qpid,     // QP id for this retransmission is required
    output reg                           o_retransmit_reqd,     // Retransmission is required
    input  wire                          i_retransmit_accepted, // Ack from QP manager that the retransmission is initiated

    // Hardware handshake ports for NVMof (only enabled if QP_CONF[4] is set to 0)
    // Direct handshake with RDMAif module
    output reg                            o_send_cq_db_cnt_valid,
    output reg  [C_M_AXI_ADDR_WIDTH -1:0] o_send_cq_db_addr,
    output reg  [                   31:0] o_send_cq_db_cnt,
    input  wire                           i_send_cq_db_rdy,

    //QP STAT register IF
    input  wire [                2:0] i_qp_curr_retry_cnt,
    output wire                       o_qp_curr_retry_cnt_rd_req,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_curr_retry_cnt_qpid,
    input  wire                       i_qp_curr_retry_cnt_rd_rdy,

    output wire       o_qp_curr_retry_cnt_wr_req,
    input  wire       i_qp_curr_retry_cnt_wr_rdy,
    //output wire [C_QP_INDX_WIDTH-1:0]          o_qp_curr_retry_cnt_qpid,
    output wire [2:0] o_qp_curr_retry_cnt_wr,

    output wire                               o_qp_cfg_retry_cnt_rd_req,
    input  wire                               i_qp_cfg_retry_cnt_rd_rdy,
    input  wire [                        2:0] i_qp_cfg_retry_cnt,
    //QP fatal IF for retry limit exceed case
    output wire                               o_qp_retry_fatal_req,
    output wire [        C_QP_INDX_WIDTH-1:0] o_qp_retry_fatal_qpid,
    input  wire                               i_qp_retry_fatal_rdy,
    input  wire [(3*C_QP_INDX_WIDTH+5)-1 : 0] i_reload_retry_cnt_en,
    // Interface to AXI master
    input  wire                               i_bus2ip_data_rdy,
    input  wire                               i_axi_master_done,
    output reg                                o_ip2bus_wen,                              // Write en to AXI master
    output reg  [    C_M_AXI_ADDR_WIDTH -1:0] o_ip2bus_wraddr,                           // Write address
    output reg  [      IP2BUS_LEN_WIDTH -1:0] o_ip2bus_len,
    output reg  [                      511:0] o_ip2bus_data,
    output reg  [                       47:0] o_retransmit_wqe_bram_din,
    input                                     init_start,
    input       [      C_QP_INDX_WIDTH-1 : 0] init_count,
    output wire [      C_QP_INDX_WIDTH-1 : 0] fsm_arbitrated_indx,
    output wire [      C_QP_INDX_WIDTH-1 : 0] fsm_i_arbitrated_indx,
    input                                     osq_timed_out_sticky_fsm_arbitrated_indx,
    input  wire [  2*C_QP_INDX_WIDTH+3-1 : 0] osq_nacked_ff,
    input  wire [  2*C_QP_INDX_WIDTH+3-1 : 0] osq_rnr_nacked_ff,
    input                                     i_qp_under_recovery_wr_en,
    input       [      C_QP_INDX_WIDTH-1 : 0] i_qp_under_recovery_wr_idx,
    input                                     i_qp_under_recovery_wr_data
);

    `include "../../rdma_macros.vh"

    localparam CQE_ENTRY_SIZE = 4;  // Entry size in Bytes

    localparam RDMA_WRITE = 8'b00000000;  // This is the opcode of the WQE
    localparam RDMA_WRITE_IMM = 8'b00000001;  // This is the opcode of the WQE for RDMA WRITE IMMEDIATE
    localparam RDMA_READ = 8'b00000100;  // RC RDMA READ REQUEST opcode

    //-------------------
    // CQE entry 
    // ------------------------------------
    // |BYTE4-|BYTE3-|BYTE2-|BYTE1-|BYTE0-|
    // |------|------|------|-------------|
    // | Flags| Rsvd | OPC  |  WRID       |
    // |------|------|------|-------------|


    localparam IDLE = 5'b00000;
    localparam GET_QP_CONF_DATA = 5'b00001;
    localparam WAIT_VALID_DATA = 5'b00010;
    localparam CHECK_ERROR = 5'b00011;
    localparam WRITE_CQE = 5'b00100;
    localparam INCR_CQ_CNT_UPD_DB = 5'b00101;
    localparam CHECK_OSQ = 5'b00110;
    localparam WRITE_CMPL_CNT = 5'b00111;
    localparam ARBITRATE_OSQ = 5'b01000;
    localparam RETRANSMIT = 5'b01001;
    localparam FLUSH_POP_OSQ = 5'b01010;
    localparam UPD_CQHD = 5'b01011;
    localparam CHECK_FLUSH_OSQ = 5'b01100;
    localparam START_RETRANSMIT = 5'b01101;
    localparam CHECK_FLUSH_MPSNBUF = 5'b01110;
    localparam WAIT_FOR_RDMA_READ_RESP_WRITE = 5'b01111;
    localparam GEN_QP_FATAL = 5'b10000;
    localparam WAIT_DATA_STABLE = 5'b11011;


    localparam QP1 = 'd1;
    localparam QP0 = 'd0;

    localparam SQ_ENTRY_SIZE = 64;  // 64 Bytes

    localparam CQE_DB_SZ_IN_BYTES = 4;  // Bytes
    localparam SQ_REWRITE_SZ_IN_BYTES = 12;  // Bytes

    reg  [                    5:0] resp_h_ns;
    reg  [                    5:0] resp_h_cs;
    reg  [C_M_AXI_ADDR_WIDTH -1:0] qp_cq_ba_ff;  // CQ base address
    reg  [                   15:0] qp_cq_depth_ff;
    reg  [                   15:0] qp_cq_hdptr_ff;
    reg  [                   15:0] rewrite_sq_ptr_ff;
    reg  [C_M_AXI_ADDR_WIDTH -1:0] qp_sq_cmpldb_addr_ff;
    reg  [                   31:0] cq_dbcount_ff;
    reg  [   C_QP_INDX_WIDTH -1:0] arbitrated_indx_ff;
    reg  [   C_QP_INDX_WIDTH -1:0] arbitrated_indx_2ff;
    reg  [ C_QP_INDX_WIDTH+36 : 0] cqe_proc_fifo_data;
    wire [ C_QP_INDX_WIDTH+36 : 0] cqe_proc_fifo_dout;
    reg                            cqe_proc_fifo_wren;
    reg                            cqe_proc_fifo_rden;
    wire                           cqe_proc_fifo_empty;
    wire                           cqe_proc_fifo_full;
    reg  [                 71 : 0] retry_proc_fifo_data;
    wire [                 71 : 0] retry_proc_fifo_dout;
    reg                            retry_proc_fifo_wren;
    reg                            retry_proc_fifo_rden;
    wire                           retry_proc_fifo_empty;
    wire                           retry_proc_fifo_full;
    reg                            osq_empty_ff;
    reg                            osq_empty_2ff;
    reg                            completion_posted;
    reg                            qp_cq_intr_en_ff;
    reg                            qp_hw_hndshk_dis_ff;
    reg                            qp_cqe_write_en_ff;
    reg                            qp_under_recovery_ff;
    reg  [                    2:0] qp_pmtu_ff;
    reg  [                   12:0] pmtu_decoded;
    reg  [                   12:0] pmtu_decoded_r;
    reg  [C_M_AXI_ADDR_WIDTH -1:0] qp_sq_ba_ff;
    reg  [C_M_AXI_ADDR_WIDTH -1:0] data_buf_addr_ff;
    reg  [C_M_AXI_ADDR_WIDTH -1:0] data_buf_addr_2ff;
    reg  [                   23:0] psn_to_retry;
    reg  [                    7:0] msn_to_retry;
    reg  [                   23:0] timeout_psn_to_retry_ff;
    reg  [                   23:0] timeout_psn_to_retry;
    reg                            osq_wqe_nack_resp_ff;
    // During retransmit the phase bit might flip, if the packet that needs to be
    // retried is from a different phase. So, the phase bit ff should be updated
    // just after retrnsmit is initiated.
    reg                            upd_phase_post_retransmit;
    reg                            osq_nacked_sampled;
    reg                            phase_bit_acknack_psn_sampled;
    reg  [                   31:0] retry_count_ff;
    wire [                   23:0] osq_acknacked_psn_plus1;
    wire                           upd_phase_retransmit_pop;
    reg                            pop_osq;
    reg                            osq_rnr_nacked;
    //wire osq_timed_out;
    reg                            osq_timed_out;
    wire                           phase_bit_osq_psn;
    reg                            completion_accepted;
    //wire [C_OSQ_PSN_WIDTH-1:0] last_osq_psn_for_arbitrated_idx;
    wire                           last_osq_psn_for_arbitrated_idx;
    wire [C_M_AXI_ADDR_WIDTH -1:0] data_buf_addr;
    wire [                   15:0] next_rewrite_sq_ptr;
    wire [                   15:0] new_cq_hdptr;
    wire [                   15:0] qp_cq_hdptr_base_0;
    wire [                   15:0] next_sq_ptr;
    wire [                   15:0] sq_ptr_frm_cqhd;
    //reg [31:0] cqe_entry;
    wire [C_M_AXI_ADDR_WIDTH -1:0] cqe_axi_address;
    wire [C_M_AXI_ADDR_WIDTH -1:0] sq_wqe_addr;
    //wire [7:0] error_flag;
    wire                           nacked_psn_rolled_over;
    wire                           osq_psn_rolled_over;
    wire [                   23:0] full_psn_to_retry;
    wire                           phase_mismatch;
    wire                           psn_0_nacked;
    reg                            pending_cqdb_update;
    //wire rdma_rd_resp_done;
    reg                            qp_fatal_flush;
    reg  [                    2:0] qp_curr_retry_cnt_ff;
    reg  [                    2:0] qp_cfg_retry_cnt_ff;
    genvar i, k;

    reg                           qp_fatal;
    reg                           qp_fatal_or_recovery;


    reg  [C_M_AXI_ADDR_WIDTH-1:0] rd_rsp_dest_addr_r1;
    reg  [C_M_AXI_ADDR_WIDTH-1:0] rd_rsp_dest_addr_r2;
    reg  [C_M_AXI_ADDR_WIDTH-1:0] rd_rsp_dest_addr_r3;
    reg                           is_opcode_read;

    reg                           phase_bit_acknack_psn;
    reg  [                   7:0] osq_entry_msn;
    reg  [   C_OSQ_PSN_WIDTH-1:0] osq_entry_end_psn;
    reg  [   C_OSQ_PSN_WIDTH-1:0] osq_entry_start_psn;
    reg  [                  23:0] osq_acknacked_psn;
    reg  [                  23:0] osq_acknacked_msn;
    reg  [                   7:0] osq_opcode;  // BTH.Opcode of outbound request
    reg  [                  15:0] osq_wrid;
    reg  [                  14:0] osq_wqe_data_bufid;
    reg                           osq_wqe_retried;
    reg                           osq_wqe_fatal_push;
    reg                           osq_nacked;
    reg                           osq_wqe_nack_resp;
    reg  [                  14:0] cqe_proc_data_bufid;
    reg  [   C_OSQ_PSN_WIDTH+1:0] completed_dma_len_multiplier;
    wire [                  23:0] completed_dma_len;
    reg  [                  23:0] cqe_proc_completed_dma_len;
    reg  [                  23:0] retry_start_psn;
    reg                           cfg_rd_req;
    reg                           retransmit_en;
    reg                           wr_retry_cnt;
    wire [                  31:0] send_cq_db_data;
    reg i_rd_osq_vld_r1, i_rd_osq_vld_r2;
    reg                          osq_empty;
    reg                          rd_resp_bram_rden;
    reg                          rd_resp_bram_wren;
    reg                          rd_resp_wr_cnt_bram_wren;
    reg  [  C_QP_INDX_WIDTH-1:0] rd_resp_wr_cnt_bram_wr_addr;
    wire [  C_OS_Q_INDX_WIDTH:0] rd_resp_wr_cnt_bram_dout;
    wire [  C_OS_Q_INDX_WIDTH:0] rd_resp_wr_cnt_bram_mirror_dout;
    wire [  C_OS_Q_INDX_WIDTH:0] rd_resp_bram_dout;
    reg                          rd_resp_bram_rden_r1;
    reg                          first_og_trans;
    reg  [  C_QP_INDX_WIDTH-1:0] first_og_trans_qpid;
    wire                         i_first_outgoing_pkt_rd_data_0;
    wire                         i_first_outgoing_pkt_rd_data_2;
    wire                         i_first_outgoing_pkt_rd_data_int_0;
    wire                         i_first_outgoing_pkt_rd_data_int_2;
    reg                          last_osq_psn_valid;
    reg  [C_QP_INDX_WIDTH-1 : 0] last_osq_psn_id;
    reg                          last_osq_psn_data;
    reg                          last_osq_psn_valid_aux;
    reg  [C_QP_INDX_WIDTH-1 : 0] last_osq_psn_id_aux;
    reg                          last_osq_psn_data_aux;
    wire                         last_osq_psn_rd_data;
    reg                          i_reload_retry_cnt_en_rd_data;
    wire                         i_reload_retry_cnt_en_rd_data_int;
    wire                         i_qp_disable_pulse_rd_data_0;
    wire                         i_qp_disable_pulse_rd_data_1;
    wire                         qp_disable_pulse_for_timed_out_rd_data;
    wire                         clr_timeout_n_nack_rd_data;
    wire                         timer_reload_for_acknack_ff_rd_data;
    wire                         clr_pending_ack_timer_reload_for_acknack_rd_data;
    wire                         osq_timed_out_sticky_wire_rd_data;
    wire                         i_qp_fatal_rd_data;
    wire                         osq_nacked_ff_rd_data_int;
    wire                         osq_nacked_ff_rd_data;
    wire                         osq_rnr_nacked_ff_rd_data_int;
    wire                         osq_rnr_nacked_ff_rd_data;
    wire                         osq_nacked_top;
    wire                         osq_nacked_reset_valid;
    wire                         timer_reload_for_wqe_ff_QP1;
    wire                         timer_reload_for_acknack_ff_valid;
    wire                         timer_reload_for_acknack_ff_data;
    wire                         osq_timed_out_sticky_wire_reset_valid;
    wire                         osq_timed_out_sticky_wire_set_valid;
    wire                         clr_timeout_n_nack_set_valid;
    wire                         i_qp_disable_pulse_set_valid;
    wire                         clr_pending_ack_timer_reload_for_acknack_set_valid;
    wire                         clr_pending_ack_timer_reload_for_acknack_reset_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] timer_reload_for_acknack_ff_id;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_wire_reset_id;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_wire_set_id;
    wire [C_QP_INDX_WIDTH-1 : 0] clr_timeout_n_nack_set_id;
    wire [C_QP_INDX_WIDTH-1 : 0] i_qp_disable_pulse_set_id;
    wire [C_QP_INDX_WIDTH-1 : 0] clr_pending_ack_timer_reload_for_acknack_set_id;
    wire [C_QP_INDX_WIDTH-1 : 0] clr_pending_ack_timer_reload_for_acknack_reset_id;
    wire                         osq_nacked_ff_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id;
    wire                         osq_nacked_ff_data;
    wire                         osq_nacked_ff_valid_pending;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id_pending;
    wire                         osq_rnr_nacked_ff_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_rnr_nacked_ff_id;
    wire                         osq_rnr_nacked_ff_data;
    wire                         osq_rnr_nacked_ff_valid_pending;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_rnr_nacked_ff_id_pending;
    reg                          osq_timed_out_rd_data_QP_not_1;
    reg                          osq_timed_out_rd_data_QP_1;
    reg                          rd_resp_wr_cnt_init_valid;
    reg  [C_QP_INDX_WIDTH-1 : 0] rd_resp_wr_cnt_init_id;
    reg                          rd_resp_wr_cnt_init_data;
    reg                          phase_bit_osq_psn_ff_valid;
    reg  [C_QP_INDX_WIDTH-1 : 0] phase_bit_osq_psn_ff_id;
    reg                          phase_bit_osq_psn_ff_data;
    reg                          phase_bit_osq_psn_ff_valid_aux;
    reg  [C_QP_INDX_WIDTH-1 : 0] phase_bit_osq_psn_ff_id_aux;
    reg                          phase_bit_osq_psn_ff_data_aux;
    wire                         phase_bit_osq_psn_ff_rd_data_1;
    wire                         phase_bit_osq_psn_ff_rd_data_0;
    wire                         rd_resp_wr_cnt_init_rd_data;
    wire                         reload_retry_cnt_valid_pending_1;
    wire [C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id_pending_1;
    wire                         reload_retry_cnt_data_pending_1;
    wire                         reload_retry_cnt_valid_pending_0;
    wire [C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id_pending_0;
    wire                         reload_retry_cnt_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id;
    wire                         reload_retry_cnt_data;
    reg                          qp_under_recovery_reg;
    wire                         qp_under_recovery;
    wire [                  3:0] cnt_add;
    wire                         mask_wr_en;


    // 如果需要重传，数据将来自 DDR（每当从 RDMAif 缓冲区读取数据时，都会从 RDMAif 反射回 DDR）。
    // WQE 处理器模块提供缓冲区池 ID，用于标识存储此 RDMA 写事务数据的缓冲区。
    // resp_handler 模块使用此缓冲区 ID 更新 WQE 中的地址，使其指向 DDR 而不是 RDMAif 本地地址。
    // WQE 条目的前 16 位（格式如下）由 resp_handler 重写。
    //
    // WQE format
    //--------|-----------------------------|           -------|-----------------------------|
    // Byte 0 |       WRID/CONTEXT          |           Byte 32|                             |
    // Byte 1 |_____________________________|           Byte 33|                             |
    // Byte 2 |         BUFID/RESERVED   |R_|           Byte 34|                             |
    // Byte 3 |_____________________________|           Byte 35|                             |
    // BYte 4 |                             |           BYte 36|                             |
    // Byte 5 |      LOCAL ADDRESS          |           Byte 37|                             |
    // Byte 6 |                             |           Byte 38|                             |
    // Byte 7 |                             |           Byte 39|       COMPLETTION INFO      |
    // Byte 8 |                             |           Byte 40|                             |
    // Byte 9 |                             |           Byte 41|                             |
    // Byte 10|                             |           Byte 42|                             |
    // Byte 11|_____________________________|           Byte 43|                             |
    // Byte 12|         LENGTH              |           Byte 44|                             |
    // Byte 13|                             |           Byte 45|                             |
    // Byte 14|                             |           Byte 46|                             |
    // Byte 15|_____________________________|           Byte 47|_____________________________|
    // Byte 16|_________OPCODE______________|           Byte 48|                             |
    // Byte 17|        RESERVED             |           Byte 49|                             |
    // BYte 18|                             |           BYte 50|                             |
    // Byte 19|_____________________________|           Byte 51|                             |
    // Byte 20|                             |           Byte 52|                             |
    // Byte 21|      REMOTE OFFSET          |           Byte 53|                             |
    // Byte 22|                             |           Byte 54|                             |
    // Byte 23|                             |           Byte 55|                             |
    // Byte 24|                             |           Byte 56|        RESERVED             |
    // Byte 25|                             |           Byte 57|                             |
    // Byte 26|                             |           Byte 58|                             |
    // Byte 27|_____________________________|           Byte 59|                             |
    // Byte 28|                             |           Byte 60|                             |
    // Byte 29|        REMOTE TAG           |           Byte 61|                             |
    // Byte 30|                             |           Byte 62|                             |
    // Byte 31|_____________________________|           Byte 63|_____________________________|

    // 通常情况下，CQ 门铃值（用于发送 Q 请求）实际上并不使用。
    // curr_sqptr 值与 SQ PI DB 一起使用，以确定 FIFO 是否已满/为空。
    // 但是，如果发生错误需要重传，QP 重传模块会将 curr_sqptr 值更改为 CQ DB中的值。
    // 这种情况预计不会频繁发生。目前，响应处理模块中的逻辑会等待 QP 管理器确认重传，因此流水线效率不高。
    // 如果重传发生的频率很低，这应该不会造成问题。


    /*o_osq_timeout logic formation is common for osq_nacked_top and osq_timed_out*/
    assign {osq_nacked_ff_valid_pending, osq_nacked_ff_id_pending, osq_nacked_ff_valid, osq_nacked_ff_id, osq_nacked_ff_data} = osq_nacked_ff;
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (osq_nacked_ff_rd_data_int)
    );
    assign osq_nacked_ff_rd_data                                                                                                                  = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == arbitrated_indx_ff) ? 1'b1 : osq_nacked_ff_rd_data_int;

    assign {osq_rnr_nacked_ff_valid_pending, osq_rnr_nacked_ff_id_pending, osq_rnr_nacked_ff_valid, osq_rnr_nacked_ff_id, osq_rnr_nacked_ff_data} = osq_rnr_nacked_ff;
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_rnr_nacked_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_rnr_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_rnr_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_rnr_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (osq_rnr_nacked_ff_rd_data_int)
    );
    assign osq_rnr_nacked_ff_rd_data = (osq_rnr_nacked_ff_valid_pending && osq_rnr_nacked_ff_id_pending == arbitrated_indx_ff) ? 1'b0 : osq_rnr_nacked_ff_rd_data_int;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_fatal (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_fatal_valid),
        .wr_addr (init_start ? init_count : i_qp_fatal_id),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (i_qp_fatal_rd_data)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_under_recovery (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_under_recovery_wr_en),
        .wr_addr (init_start ? init_count : i_qp_under_recovery_wr_idx),
        .wr_data (init_start ? 1'b0 : i_qp_under_recovery_wr_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (qp_under_recovery)
    );
    `MSFF_R(qp_under_recovery_reg, qp_under_recovery, core_clk, ~core_rstn)

    assign osq_nacked_top = osq_nacked_ff_rd_data || osq_rnr_nacked_ff_rd_data || osq_timed_out_sticky_fsm_arbitrated_indx;

    //wire current_qp_fatal = qp_fatal | (qp_under_recovery_ff & ~(osq_timed_out || i_osq_nacked || osq_rnr_nacked));
    wire current_qp_fatal = qp_fatal | (qp_under_recovery_reg & ~(osq_timed_out || i_osq_nacked || osq_rnr_nacked));
    `MSFF_R(qp_fatal, (i_qp_fatal_rd_data & !osq_nacked_top), core_clk, ~core_rstn)
    `MSFF_R(qp_fatal_or_recovery, ((i_qp_fatal_rd_data || qp_under_recovery) && !osq_nacked_top), core_clk, ~core_rstn)



    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_rd_resp_wr_cnt_init (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || rd_resp_wr_cnt_init_valid),
        .wr_addr (init_start ? init_count : rd_resp_wr_cnt_init_id),
        .wr_data (init_start ? 1'b0 : rd_resp_wr_cnt_init_data),
        .rd_en   (1'b1),
        .rd_addr (first_og_trans_qpid),
        .rd_data (rd_resp_wr_cnt_init_rd_data)
    );

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            resp_h_cs                    <= IDLE;
            acknacked_psn_rd_req         <= 1'b0;
            phase_bit_acknack_psn        <= 1'b0;
            osq_entry_msn                <= 8'h00;
            osq_entry_end_psn            <= 'h0;
            osq_entry_start_psn          <= 'h0;
            osq_acknacked_psn            <= 24'h0;
            osq_acknacked_msn            <= 24'h0;
            osq_opcode                   <= 8'h00;
            osq_wrid                     <= 16'h0000;
            osq_wqe_data_bufid           <= 15'h0000;
            osq_wqe_retried              <= 1'b0;
            osq_wqe_fatal_push           <= 1'b0;
            osq_nacked                   <= 1'b0;
            osq_wqe_nack_resp            <= 1'b0;
            o_rd_osq                     <= 1'b0;
            cqe_proc_fifo_data           <= 'd00;
            cqe_proc_fifo_wren           <= 1'b0;
            pop_osq                      <= 1'b0;
            o_freeup_data_buf            <= 1'b0;
            o_freeup_data_bufid          <= 15'h0000;
            cfg_rd_req                   <= 1'b0;
            retransmit_en                <= 1'b0;
            wr_retry_cnt                 <= 1'b0;
            o_flush_mpsn                 <= 1'b0;
            cq_dbcount_ff                <= 32'd0;
            i_rd_osq_vld_r1              <= 1'b0;
            i_rd_osq_vld_r2              <= 1'b0;
            pending_cqdb_update          <= 1'b0;
            osq_empty                    <= 1'b0;
            pmtu_decoded                 <= 13'h0000;
            completed_dma_len_multiplier <= 'd0;
            retry_start_psn              <= 24'd0;
            rd_resp_bram_rden            <= 1'b0;
            rd_resp_bram_rden_r1         <= 1'b0;
            rd_resp_bram_wren            <= 1'b0;
            rd_resp_wr_cnt_bram_wren     <= 1'b0;
            rd_resp_wr_cnt_bram_wr_addr  <= 'd0;
            first_og_trans               <= 1'b0;
            first_og_trans_qpid          <= 'd0;
            rd_resp_wr_cnt_init_valid    <= 1'b0;
            rd_resp_wr_cnt_init_id       <= {C_QP_INDX_WIDTH{1'b0}};
            rd_resp_wr_cnt_init_data     <= 1'b0;  // rd_addr first_og_trans_qpid
            fsm_pushback_valid           <= 1'b0;
        end else begin
            rd_resp_wr_cnt_init_valid <= 1'b0;
            if (~rd_resp_wr_cnt_bram_wren && ~i_rd_rsp_wr_done && ~rd_resp_bram_wren) begin
                if (~rd_resp_wr_cnt_init_rd_data && i_first_outgoing_pkt_rd_data_2) begin
                    first_og_trans            <= 1'b1;
                    rd_resp_wr_cnt_init_valid <= 1'b1;
                    rd_resp_wr_cnt_init_id    <= first_og_trans_qpid;
                    rd_resp_wr_cnt_init_data  <= 1'b1;
                end else begin
                    first_og_trans      <= 1'b0;
                    first_og_trans_qpid <= first_og_trans_qpid + {{C_QP_INDX_WIDTH - 1{1'b0}}, 1'b1};
                    if (rd_resp_wr_cnt_init_rd_data && ~i_first_outgoing_pkt_rd_data_2) begin
                        rd_resp_wr_cnt_init_valid <= 1'b1;
                        rd_resp_wr_cnt_init_id    <= first_og_trans_qpid;
                        rd_resp_wr_cnt_init_data  <= 1'b0;
                    end
                end
            end
            i_rd_osq_vld_r1             <= i_rd_osq_vld;
            i_rd_osq_vld_r2             <= i_rd_osq_vld_r1;
            rd_resp_bram_rden_r1        <= rd_resp_bram_rden;
            rd_resp_wr_cnt_bram_wren    <= i_rd_rsp_wr_done && !mask_wr_en;
            rd_resp_wr_cnt_bram_wr_addr <= i_rd_rsp_qpid;
            case (qp_pmtu_ff)
                3'b000: begin
                    pmtu_decoded <= 13'h0100;
                end
                3'b001: begin
                    pmtu_decoded <= 13'h0200;
                end
                3'b010: begin
                    pmtu_decoded <= 13'h0400;
                end
                3'b011: begin
                    pmtu_decoded <= 13'h0800;
                end
                3'b100: begin
                    pmtu_decoded <= 13'h1000;
                end
                default: begin
                    pmtu_decoded <= 13'h1000;
                end
            endcase
            case (resp_h_cs)
                IDLE: begin
                    acknacked_psn_rd_req <= 1'b0;
                    cq_dbcount_ff        <= 32'd0;
                    //if(i_acks_to_process && i_arbitration_done) begin
                    if (i_arbitration_done) begin
                        resp_h_cs  <= GET_QP_CONF_DATA;
                        cfg_rd_req <= 1'b1;
                        o_rd_osq   <= 1'b1;
                    end
                end
                GET_QP_CONF_DATA: begin
                    if (i_qp_conf_valid && i_qp_cfg_retry_cnt_rd_rdy) begin
                        resp_h_cs            <= WAIT_VALID_DATA;
                        cfg_rd_req           <= 1'b0;
                        acknacked_psn_rd_req <= 1'b1;
                    end
                end
                WAIT_VALID_DATA: begin
                    o_rd_osq             <= 1'b0;
                    acknacked_psn_rd_req <= 1'b0;
                    if (acknacked_psn_rd_req_done) begin
                        resp_h_cs             <= CHECK_ERROR;
                        phase_bit_acknack_psn <= i_phase_bit_acknack_psn;
                        osq_entry_msn         <= i_osq_entry_msn;
                        osq_entry_end_psn     <= i_osq_entry_end_psn;
                        osq_entry_start_psn   <= i_osq_entry_start_psn;
                        osq_acknacked_psn     <= i_osq_acknacked_psn;
                        osq_acknacked_msn     <= i_osq_acknacked_msn;
                        osq_opcode            <= i_osq_opcode;
                        osq_wrid              <= i_osq_wrid;
                        osq_wqe_data_bufid    <= i_osq_wqe_data_bufid;
                        osq_wqe_retried       <= i_osq_wqe_retried;
                        osq_wqe_fatal_push    <= i_osq_wqe_fatal_push;
                        osq_nacked            <= i_osq_nacked;
                        osq_wqe_nack_resp     <= i_osq_wqe_nack_resp;
                        osq_empty             <= i_osq_empty || osq_empty_ff || osq_empty_2ff;
                    end else begin
                        resp_h_cs <= WAIT_VALID_DATA;
                    end
                end
                GEN_QP_FATAL: begin
                    if (i_qp_retry_fatal_rdy) begin
                        resp_h_cs <= RETRANSMIT;
                        pop_osq   <= 1'b1;
                    end
                end
                CHECK_ERROR: begin
                    pending_cqdb_update <= 1'b0;
                    if (arbitrated_indx_ff == QP0) begin
                        resp_h_cs <= ARBITRATE_OSQ;
                    end else if (~pending_cqdb_update & ((osq_rnr_nacked & ~osq_timed_out) || ((osq_nacked_sampled | osq_timed_out) & ~osq_wqe_nack_resp_ff))) begin
                        resp_h_cs          <= ARBITRATE_OSQ;
                        fsm_pushback_valid <= 1'b1;
                    end else if ((arbitrated_indx_ff == QP1) || (current_qp_fatal & i_dma_in_idle & i_en_osq_flush_on_qp_fatal) || pending_cqdb_update || (current_qp_fatal & qp_fatal_flush)) begin
                        resp_h_cs <= WRITE_CQE;
                    end else if (current_qp_fatal & ~(i_dma_in_idle & i_en_osq_flush_on_qp_fatal)) begin
                        resp_h_cs          <= ARBITRATE_OSQ;
                        fsm_pushback_valid <= 1'b1;
                    end else if (psn_0_nacked) begin  // See comment below***
                        if ((osq_entry_end_psn < osq_entry_start_psn) | (osq_entry_start_psn == 'b0))  // case 1, case 2
                            resp_h_cs <= START_RETRANSMIT;
                        else  // case 3
                            resp_h_cs <= WRITE_CQE;
                    end else if (((osq_entry_end_psn > (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) & osq_timed_out & ~phase_mismatch) | ((osq_entry_end_psn < (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) & osq_timed_out & phase_mismatch)) resp_h_cs <= START_RETRANSMIT;
                    else if (((osq_entry_end_psn >= (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) & (osq_nacked_sampled | osq_rnr_nacked) & ~phase_mismatch) | ((osq_entry_end_psn <= (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) & (osq_nacked_sampled | osq_rnr_nacked) & phase_mismatch))
                        resp_h_cs <= START_RETRANSMIT;
                    else if (((osq_entry_end_psn > osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0]) && ~phase_mismatch) ||
                        //(i_osq_empty || osq_empty_ff || osq_empty_2ff))
                        (osq_empty))
                        resp_h_cs <= ARBITRATE_OSQ;
                    else if ((osq_entry_end_psn < osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0]) && phase_mismatch) resp_h_cs <= ARBITRATE_OSQ;
                    else resp_h_cs <= WRITE_CQE;
                end
                // ***Nack for PSN 0 is a special case and needs to be handled
                // separately. It is so because of multiple reasons - no OSQ PSN can
                // be greater than 0 - so normal conditions cannot be applied.
                // Secondly when the NACKed psn is 0, the phase bit needs to go back
                // as the last valid acked PSN would be before roll over. 
                // There are 3 different cases for psn 0 nacked
                // SPSN denotes start PSN, EPSN denotes end PSN and "|" denotes
                // rollover point 
                // 
                // (1)SPSN.................| nack0...EPSN         => EPSN is > 0 on the rolled over side
                // (2).....................| nack0/SPSN...EPSN    => SPSN is 0, EPSN > 0
                // (3)SPSN1 EPSN1 SPSN2....| nack0...EPSN2        => EPSN1 needs to be completed, SPSN2 needs to be retried

                WRITE_CQE: begin
                    if (~cqe_proc_fifo_full || ~qp_cqe_write_en_ff) begin
                        cqe_proc_fifo_data  <= {arbitrated_indx_ff, qp_cq_intr_en_ff, qp_cqe_write_en_ff, 3'b000, 7'h00, current_qp_fatal, osq_opcode, osq_wrid};
                        cqe_proc_fifo_wren  <= qp_cqe_write_en_ff;
                        o_freeup_data_buf   <= (~|osq_opcode[7:1]) & ((osq_wqe_retried & osq_wqe_fatal_push) | (~osq_wqe_fatal_push));
                        o_freeup_data_bufid <= osq_wqe_data_bufid;
                        resp_h_cs           <= WAIT_FOR_RDMA_READ_RESP_WRITE;
                        pop_osq             <= 1'b1;
                        rd_resp_bram_rden   <= ~qp_fatal_flush && (osq_opcode == RDMA_READ);
                    end
                end

                WAIT_FOR_RDMA_READ_RESP_WRITE: begin
                    cqe_proc_fifo_wren <= 1'b0;
                    o_freeup_data_buf  <= 1'b0;
                    pop_osq            <= 1'b0;
                    if (qp_fatal_flush || (rd_resp_wr_cnt_bram_mirror_dout != rd_resp_bram_dout && rd_resp_bram_rden_r1) || (osq_opcode != RDMA_READ)) begin
                        resp_h_cs         <= INCR_CQ_CNT_UPD_DB;
                        rd_resp_bram_wren <= ~qp_fatal_flush && (osq_opcode == RDMA_READ);
                        rd_resp_bram_rden <= 1'b0;
                    end
                end

                INCR_CQ_CNT_UPD_DB: begin
                    rd_resp_bram_wren <= 1'b0;
                    if (~cqe_proc_fifo_full || qp_hw_hndshk_dis_ff) begin
                        cqe_proc_fifo_data   <= {arbitrated_indx_ff, qp_cq_intr_en_ff, qp_cqe_write_en_ff, 3'b001, send_cq_db_data};
                        cqe_proc_fifo_wren   <= ~qp_hw_hndshk_dis_ff;
                        cq_dbcount_ff        <= cq_dbcount_ff + 1'b1;
                        resp_h_cs            <= UPD_CQHD;
                        o_rd_osq             <= 1'b1;
                        acknacked_psn_rd_req <= 1'b1;
                    end
                end

                UPD_CQHD: begin
                    o_rd_osq             <= 1'b0;
                    acknacked_psn_rd_req <= 1'b0;
                    cqe_proc_fifo_wren   <= 1'b0;
                    if (i_rd_osq_vld_r2) begin
                        osq_entry_msn       <= i_osq_entry_msn;
                        osq_entry_end_psn   <= i_osq_entry_end_psn;
                        osq_entry_start_psn <= i_osq_entry_start_psn;
                        osq_opcode          <= i_osq_opcode;
                        osq_wrid            <= i_osq_wrid;
                        osq_wqe_data_bufid  <= i_osq_wqe_data_bufid;
                        osq_wqe_retried     <= i_osq_wqe_retried;
                        osq_wqe_fatal_push  <= i_osq_wqe_fatal_push;
                        osq_wqe_nack_resp   <= i_osq_wqe_nack_resp;
                        osq_nacked          <= i_osq_nacked;
                        osq_empty           <= i_osq_empty || osq_empty_ff || osq_empty_2ff;
                    end
                    if (acknacked_psn_rd_req_done) begin
                        resp_h_cs             <= CHECK_OSQ;
                        phase_bit_acknack_psn <= i_phase_bit_acknack_psn;
                        osq_acknacked_psn     <= i_osq_acknacked_psn;
                        osq_acknacked_msn     <= i_osq_acknacked_msn;
                    end
                end

                CHECK_OSQ: begin
                    if ((osq_rnr_nacked & ~osq_timed_out) || (osq_empty) || ((osq_nacked_sampled | osq_timed_out) & ~osq_wqe_nack_resp) || (~qp_fatal_flush & qp_fatal_or_recovery)) resp_h_cs <= WRITE_CMPL_CNT;
                    else if (current_qp_fatal | (arbitrated_indx_ff == QP1 && ~osq_empty)) resp_h_cs <= CHECK_ERROR;

                    // A nack for packet n actually means ack for all packets till
                    // n-1. So, the previous acked packets need to be completed and
                    // the corresponding count needs to be written before starting
                    // retry
                    else if (psn_0_nacked) begin  // See comment above***
                        if ((osq_entry_end_psn < osq_entry_start_psn) | (osq_entry_start_psn == 'b0)) begin  // case 1, case 2
                            resp_h_cs     <= WRITE_CMPL_CNT;  //START_RETRANSMIT;
                            retransmit_en <= 1'b1;
                        end else begin  // case 3
                            resp_h_cs     <= WRITE_CQE;
                            retransmit_en <= 1'b0;
                        end
                    end else if (((osq_entry_end_psn > (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) && osq_timed_out && ~phase_mismatch) | ((osq_entry_end_psn < (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) && osq_timed_out & phase_mismatch)) begin
                        resp_h_cs     <= WRITE_CMPL_CNT;
                        retransmit_en <= 1'b1;
                    end else if (((osq_entry_end_psn >= (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) && (osq_nacked_sampled | osq_rnr_nacked) && ~phase_mismatch) | ((osq_entry_end_psn <= (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) && (osq_nacked_sampled | osq_rnr_nacked) && phase_mismatch)) begin
                        resp_h_cs     <= WRITE_CMPL_CNT;
                        retransmit_en <= 1'b1;
                    end else if ((osq_entry_end_psn > osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0]) && ~phase_mismatch) begin  // no more completed WQEs to process
                        resp_h_cs     <= WRITE_CMPL_CNT;
                        retransmit_en <= 1'b0;
                    end else if ((osq_entry_end_psn < osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0]) && phase_mismatch) begin
                        resp_h_cs     <= WRITE_CMPL_CNT;
                        retransmit_en <= 1'b0;
                    end else begin
                        resp_h_cs           <= CHECK_ERROR;
                        retransmit_en       <= 1'b0;
                        pending_cqdb_update <= qp_hw_hndshk_dis_ff ? 1'b1 : 1'b0;
                    end
                end

                WRITE_CMPL_CNT: begin
                    if (~cqe_proc_fifo_full || !qp_hw_hndshk_dis_ff) begin
                        cqe_proc_fifo_data <= {arbitrated_indx_ff, qp_cq_intr_en_ff, qp_cqe_write_en_ff, 3'b010, cq_dbcount_ff};
                        cqe_proc_fifo_wren <= qp_hw_hndshk_dis_ff;
                        if (retransmit_en) begin
                            resp_h_cs     <= START_RETRANSMIT;
                            retransmit_en <= 1'b0;
                        end else if (qp_fatal_flush) begin
                            resp_h_cs <= CHECK_FLUSH_OSQ;
                        end else begin
                            resp_h_cs <= ARBITRATE_OSQ;
                        end
                    end
                end

                // Before moving to IDLE ask the Arbiter to arbitrate and get the next
                // valid SQ to work on
                ARBITRATE_OSQ: begin
                    cqe_proc_fifo_wren   <= 1'b0;
                    retry_proc_fifo_wren <= 1'b0;
                    resp_h_cs            <= IDLE;
                    fsm_pushback_valid   <= 1'b0;
                end

                START_RETRANSMIT: begin
                    cqe_proc_fifo_wren <= 1'b0;
                    retry_start_psn    <= osq_entry_start_psn;
                    if ((osq_entry_start_psn[C_OSQ_PSN_WIDTH-1] && ~osq_acknacked_psn[C_OSQ_PSN_WIDTH-1]) && (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] < osq_entry_start_psn)) begin
                        //completed_dma_len <= (({1'b1,osq_acknacked_psn[C_OSQ_PSN_WIDTH-1 :0]} - {1'b0,osq_entry_start_psn[C_OSQ_PSN_WIDTH-1:0]} + {23'd0,~(osq_rnr_nacked || osq_nacked_sampled)}) * pmtu_decoded);
                        completed_dma_len_multiplier <= ({1'b1, osq_acknacked_psn[C_OSQ_PSN_WIDTH-1 : 0]} - {1'b0, osq_entry_start_psn[C_OSQ_PSN_WIDTH-1:0]} + {23'd0, ~(osq_rnr_nacked || osq_nacked_sampled)});
                    end else if (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] >= osq_entry_start_psn) begin
                        //completed_dma_len <= (({1'b0,osq_acknacked_psn[C_OSQ_PSN_WIDTH-1 :0]} - {1'b0,osq_entry_start_psn[C_OSQ_PSN_WIDTH-1:0]} + {23'd0,~(osq_rnr_nacked || osq_nacked_sampled)}) * pmtu_decoded);
                        completed_dma_len_multiplier <= ({1'b0, osq_acknacked_psn[C_OSQ_PSN_WIDTH-1 : 0]} - {1'b0, osq_entry_start_psn[C_OSQ_PSN_WIDTH-1:0]} + {23'd0, ~(osq_rnr_nacked || osq_nacked_sampled)});
                    end else begin
                        completed_dma_len_multiplier <= 'd0;
                    end
                    if (i_cfg_retry_cnt_fatal_dis) begin
                        resp_h_cs    <= RETRANSMIT;
                        wr_retry_cnt <= 1'b1;
                        pop_osq      <= 1'b1;
                    end else begin
                        if ((qp_cfg_retry_cnt_ff == 3'h0) & osq_nacked_sampled & !current_qp_fatal) begin
                            resp_h_cs <= GEN_QP_FATAL;
                        end else if ((qp_curr_retry_cnt_ff == 3'h0) & osq_timed_out & !osq_rnr_nacked & !current_qp_fatal) begin
                            resp_h_cs <= GEN_QP_FATAL;
                        end else begin
                            resp_h_cs    <= RETRANSMIT;
                            wr_retry_cnt <= 1'b1;
                            pop_osq      <= 1'b1;
                        end
                    end
                end
                RETRANSMIT: begin
                    wr_retry_cnt <= 1'b0;
                    pop_osq      <= 1'b0;
                    //if (~|(osq_opcode[7:1]) & ~osq_wqe_retried & (C_EN_WR_RETRY_DATA_BUF | NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE)) begin
                    if (~|(osq_opcode[7:1]) & ~osq_wqe_retried & (NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE)) begin
                        if (~cqe_proc_fifo_full) begin
                            cqe_proc_fifo_data <= {arbitrated_indx_ff, qp_cq_intr_en_ff, qp_cqe_write_en_ff, 3'b011, {osq_wqe_data_bufid, 1'b1, osq_wrid}};
                            cqe_proc_fifo_wren <= 1'b1;
                            resp_h_cs          <= FLUSH_POP_OSQ;
                        end
                    end else begin
                        if (~cqe_proc_fifo_full) begin
                            cqe_proc_fifo_data <= {arbitrated_indx_ff, qp_cq_intr_en_ff, qp_cqe_write_en_ff, 3'b101, {osq_wqe_data_bufid, 1'b1, osq_wrid}};
                            cqe_proc_fifo_wren <= 1'b1;
                            resp_h_cs          <= FLUSH_POP_OSQ;
                            o_flush_mpsn       <= 1'b1;
                        end
                    end
                end

                FLUSH_POP_OSQ: begin
                    o_rd_osq           <= 1'b1;
                    cqe_proc_fifo_wren <= 1'b0;
                    resp_h_cs          <= CHECK_FLUSH_OSQ;

                end

                CHECK_FLUSH_OSQ: begin
                    cqe_proc_fifo_wren <= 1'b0;
                    o_rd_osq           <= 1'b0;
                    if (i_osq_empty || qp_fatal_flush) begin
                        resp_h_cs    <= CHECK_FLUSH_MPSNBUF;
                        o_flush_mpsn <= 1'b1;
                    end else begin
                        if (i_rd_osq_vld_r2) begin
                            resp_h_cs             <= RETRANSMIT;
                            pop_osq               <= 1'b1;
                            phase_bit_acknack_psn <= i_phase_bit_acknack_psn;
                            osq_entry_msn         <= i_osq_entry_msn;
                            osq_entry_end_psn     <= i_osq_entry_end_psn;
                            osq_entry_start_psn   <= i_osq_entry_start_psn;
                            osq_opcode            <= i_osq_opcode;
                            osq_wrid              <= i_osq_wrid;
                            osq_wqe_data_bufid    <= i_osq_wqe_data_bufid;
                            osq_wqe_retried       <= i_osq_wqe_retried;
                            osq_wqe_fatal_push    <= i_osq_wqe_fatal_push;
                            osq_nacked            <= i_osq_nacked;
                            osq_wqe_nack_resp     <= i_osq_wqe_nack_resp;
                            osq_empty             <= i_osq_empty || osq_empty_ff || osq_empty_2ff;
                        end
                    end
                end

                CHECK_FLUSH_MPSNBUF: begin
                    if (i_mpsnbuf_flush_done) begin
                        o_flush_mpsn <= 1'b0;
                        resp_h_cs    <= ARBITRATE_OSQ;
                        if (!qp_fatal_flush) begin
                            cqe_proc_fifo_data   <= {arbitrated_indx_ff, qp_cq_intr_en_ff, qp_cqe_write_en_ff, 3'b100, 8'h00, completed_dma_len};
                            cqe_proc_fifo_wren   <= 1'b1;
                            retry_proc_fifo_data <= {retry_start_psn, psn_to_retry, {i_osq_acknacked_msn[23:8], msn_to_retry}};
                            retry_proc_fifo_wren <= 1'b1;
                        end
                    end
                end

            endcase
        end
    end

    always @(posedge core_clk) pmtu_decoded_r <= pmtu_decoded;
    assign completed_dma_len = (completed_dma_len_multiplier * pmtu_decoded_r);

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (32),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (27),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH + 37),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH + 37),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_cqe_proc_fifo (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (cqe_proc_fifo_wren),
        .din          (cqe_proc_fifo_data),
        .rd_en        (cqe_proc_fifo_rden),
        .empty        (cqe_proc_fifo_empty),
        .dout         (cqe_proc_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (cqe_proc_fifo_full),
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
        .READ_DATA_WIDTH    (72),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (72),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_retry_proc_fifo (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (retry_proc_fifo_wren),
        .din          (retry_proc_fifo_data),
        .rd_en        (retry_proc_fifo_rden),
        .empty        (retry_proc_fifo_empty),
        .dout         (retry_proc_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (retry_proc_fifo_full),
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
    reg       cqe_cfg_rd_req;
    reg [2:0] cqe_proc_cs;
    reg hwhs_req, retransmit_req, axi_mm_req;
    reg sq_ptr_cnt_en;
    reg inc_sq_ptr_cnt;
    reg trans_done;
    reg cqe_proc_intr_en;
    localparam CQE_IDLE = 3'b000;
    localparam GET_QP_CONF = 3'b001;
    localparam GEN_TRANS = 3'b011;
    localparam WAIT_TRANS_DONE = 3'b100;
    localparam WAIT_CFG = 3'b010;

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            cqe_proc_fifo_rden             <= 1'b0;
            cqe_proc_data_bufid            <= 15'h00;
            cqe_cfg_rd_req                 <= 1'b0;
            o_qp_cqe_proc_qpid             <= 'h00;
            cqe_proc_cs                    <= CQE_IDLE;
            o_ip2bus_data                  <= 512'h00;
            o_ip2bus_wraddr                <= 'h00;
            o_ip2bus_wen                   <= 1'b0;
            o_ip2bus_len                   <= 'h00;
            o_send_cq_db_cnt_valid         <= 1'b0;
            o_send_cq_db_addr              <= 'h0000;
            o_send_cq_db_cnt               <= 32'd0;
            hwhs_req                       <= 1'b0;
            retransmit_req                 <= 1'b0;
            axi_mm_req                     <= 1'b0;
            o_qp_cq_hdptr_wrn              <= 1'b0;
            o_qp_cq_hdptr                  <= 16'h0000;
            o_retransmit_reqd              <= 1'b0;
            o_retransmit_qpid              <= 'h00;
            completion_posted              <= 1'b0;
            completion_accepted            <= 1'b0;
            sq_ptr_cnt_en                  <= 1'b0;
            o_clr_timeout_n_nack_set_valid <= 1'b0;
            o_clr_timeout_n_nack_set_id    <= {C_QP_INDX_WIDTH{1'b0}};
            trans_done                     <= 1'b0;
            o_msn_to_retry                 <= 23'd0;
            o_psn_to_retry                 <= 23'd0;
            inc_sq_ptr_cnt                 <= 1'b0;
            cqe_proc_intr_en               <= 1'b0;
            cqe_proc_completed_dma_len     <= 24'd0;
            o_retransmit_wqe_bram_din      <= 47'd00;
        end else begin
            case (cqe_proc_cs)
                CQE_IDLE: begin
                    cqe_proc_fifo_rden             <= 1'b0;
                    retry_proc_fifo_rden           <= 1'b0;
                    o_clr_timeout_n_nack_set_valid <= 1'b0;
                    trans_done                     <= 1'b0;
                    if (i_qp_cq_hdptr_valid) begin
                        o_qp_cq_hdptr_wrn <= 1'b0;
                    end
                    if (~cqe_proc_fifo_empty && ~o_qp_cq_hdptr_wrn) begin
                        cqe_proc_data_bufid        <= cqe_proc_fifo_dout[31:17];
                        cqe_proc_cs                <= GET_QP_CONF;
                        cqe_cfg_rd_req             <= 1'b1;
                        o_qp_cqe_proc_qpid         <= cqe_proc_fifo_dout[C_QP_INDX_WIDTH+36 : 37];
                        cqe_proc_intr_en           <= cqe_proc_fifo_dout[36];
                        cqe_proc_completed_dma_len <= cqe_proc_fifo_dout[23:0];
                    end
                end
                GET_QP_CONF: begin
                    if (i_qp_sq_ba_valid && i_qp_cq_ba_valid && i_qp_cq_depth_valid && i_qp_cq_hdptr_valid) begin
                        cqe_proc_cs    <= WAIT_CFG;  //GEN_TRANS;
                        cqe_cfg_rd_req <= 1'b0;
                    end
                end
                WAIT_CFG: begin
                    cqe_proc_cs <= GEN_TRANS;
                end
                GEN_TRANS: begin
                    cqe_proc_cs <= WAIT_TRANS_DONE;
                    if (cqe_proc_fifo_dout[34 : 32] == 3'b000) begin
                        o_ip2bus_data     <= {480'h0, cqe_proc_fifo_dout[31:0]};
                        o_ip2bus_wraddr   <= cqe_axi_address;
                        o_ip2bus_wen      <= 1'b1;
                        o_ip2bus_len      <= CQE_DB_SZ_IN_BYTES;
                        o_qp_cq_hdptr_wrn <= cqe_proc_fifo_dout[35];  //1'b1;
                        o_qp_cq_hdptr     <= new_cq_hdptr;
                        axi_mm_req        <= 1'b1;
                    end else if (cqe_proc_fifo_dout[34 : 32] == 3'b001) begin
                        o_send_cq_db_cnt_valid <= 1'b1;
                        o_send_cq_db_addr      <= qp_sq_cmpldb_addr_ff;
                        o_send_cq_db_cnt       <= cqe_proc_fifo_dout[31:0];
                        o_qp_cq_hdptr_wrn      <= ~cqe_proc_fifo_dout[35];  //1'b1;
                        o_qp_cq_hdptr          <= new_cq_hdptr;
                        hwhs_req               <= 1'b1;
                        completion_posted      <= 1'b1;
                    end else if (cqe_proc_fifo_dout[34 : 32] == 3'b010) begin
                        // Host-CQE mode exports the absolute completed CQ head.
                        // FIFO entries ahead of this marker have already written
                        // their CQEs and committed CQHEAD; GET_QP_CONF rereads it.
                        // cq_dbcount_ff is only an arbitration-local delta (often
                        // 1), so publishing that delta cannot identify completion
                        // of a subsequent software batch. Reuse the registered
                        // 16-bit head: no new adder or state in the write path.
                        // CQE-disabled and sideband users retain count semantics.
                        o_ip2bus_data     <= cqe_proc_fifo_dout[35] ?
                                            {496'h0, qp_cq_hdptr_ff} :
                                            {480'h0, cqe_proc_fifo_dout[31:0]};
                        o_ip2bus_wraddr   <= qp_sq_cmpldb_addr_ff;
                        o_ip2bus_wen      <= 1'b1;
                        o_ip2bus_len      <= CQE_DB_SZ_IN_BYTES;
                        o_qp_cq_hdptr_wrn <= ~cqe_proc_fifo_dout[35];  //1'b1;
                        o_qp_cq_hdptr     <= ((qp_cq_hdptr_ff + cqe_proc_fifo_dout[15:0] > qp_cq_depth_ff)) ? (qp_cq_hdptr_ff + cqe_proc_fifo_dout[15:0] - qp_cq_depth_ff) : (qp_cq_hdptr_ff + cqe_proc_fifo_dout[15:0]);
                        axi_mm_req        <= 1'b1;
                        completion_posted <= 1'b1;
                    end else if (cqe_proc_fifo_dout[34 : 32] == 3'b011) begin
                        o_ip2bus_data   <= (NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE == 1) ? 512'h0 : {416'h0, data_buf_addr_2ff, cqe_proc_fifo_dout[31:0]};
                        o_ip2bus_wraddr <= sq_wqe_addr;
                        o_ip2bus_wen    <= 1'b1;
                        o_ip2bus_len    <= SQ_REWRITE_SZ_IN_BYTES;
                        axi_mm_req      <= 1'b1;
                        sq_ptr_cnt_en   <= 1'b1;
                    end else if (cqe_proc_fifo_dout[34 : 32] == 3'b101) begin
                        o_ip2bus_len       <= SQ_REWRITE_SZ_IN_BYTES;
                        axi_mm_req         <= 1'b1;
                        sq_ptr_cnt_en      <= 1'b1;
                        inc_sq_ptr_cnt     <= 1'b1;
                        cqe_proc_fifo_rden <= 1'b1;
                    end else if (cqe_proc_fifo_dout[34 : 32] == 3'b100) begin
                        o_retransmit_reqd         <= 1'b1;
                        o_msn_to_retry            <= retry_proc_fifo_dout[23:0];  //{i_osq_acknacked_msn[23:8], msn_to_retry};
                        o_psn_to_retry            <= retry_proc_fifo_dout[47:24];  //psn_to_retry;
                        o_retransmit_qpid         <= cqe_proc_fifo_dout[C_QP_INDX_WIDTH+36 : 37];
                        o_retransmit_wqe_bram_din <= {retry_proc_fifo_dout[71:48], cqe_proc_completed_dma_len};
                        retransmit_req            <= 1'b1;
                        sq_ptr_cnt_en             <= 1'b0;
                    end
                end
                WAIT_TRANS_DONE: begin
                    o_ip2bus_wen       <= 1'b0;
                    cqe_proc_fifo_rden <= 1'b0;
                    if (i_qp_cq_hdptr_valid) begin
                        o_qp_cq_hdptr_wrn <= 1'b0;
                    end
                    inc_sq_ptr_cnt <= 1'b0;
                    if (trans_done || inc_sq_ptr_cnt) begin
                        cqe_proc_cs                    <= CQE_IDLE;
                        cqe_proc_fifo_rden             <= 1'b0;
                        retry_proc_fifo_rden           <= 1'b0;
                        //clear bit if it was set in last clcok
                        o_clr_timeout_n_nack_set_valid <= 1'b0;
                        trans_done                     <= 1'b0;
                        completion_accepted            <= 1'b0;
                    end else begin
                        if (hwhs_req && i_send_cq_db_rdy) begin
                            o_send_cq_db_cnt_valid <= 1'b0;
                            //cqe_proc_cs <= CQE_IDLE;
                            trans_done             <= 1'b1;
                            cqe_proc_fifo_rden     <= 1'b1;
                            hwhs_req               <= 1'b0;
                            completion_accepted    <= 1'b0;
                            completion_posted      <= 1'b0;
                        end else if (retransmit_req && i_retransmit_accepted) begin
                            o_retransmit_reqd              <= 1'b0;
                            //cqe_proc_cs <= CQE_IDLE;
                            trans_done                     <= 1'b1;
                            cqe_proc_fifo_rden             <= 1'b1;
                            retransmit_req                 <= 1'b0;
                            retry_proc_fifo_rden           <= 1'b1;
                            o_clr_timeout_n_nack_set_valid <= 1'b1;
                            o_clr_timeout_n_nack_set_id    <= o_qp_cqe_proc_qpid;
                        end else if (axi_mm_req && i_axi_master_done) begin
                            //cqe_proc_cs <= CQE_IDLE; 
                            trans_done          <= 1'b1;
                            cqe_proc_fifo_rden  <= 1'b1;
                            axi_mm_req          <= 1'b0;
                            completion_accepted <= completion_posted;
                            completion_posted   <= 1'b0;
                        end
                    end
                end
            endcase
        end
    end


    `MSFF_R(osq_rnr_nacked, i_osq_rnr_nacked, core_clk, ~core_rstn)
    assign fsm_arbitrated_indx     = arbitrated_indx_ff;

    assign osq_acknacked_psn_plus1 = osq_acknacked_psn[23:0] + 1'b1;

    `MSFF_R(osq_wqe_nack_resp_ff, i_osq_wqe_nack_resp, core_clk, ~core_rstn)

    `MSFF_R(msn_to_retry, (resp_h_cs == START_RETRANSMIT) ? osq_entry_msn : msn_to_retry, core_clk, ~core_rstn)
    `MSFF_R(psn_to_retry, (resp_h_cs == START_RETRANSMIT) ? full_psn_to_retry : psn_to_retry[23:0], core_clk, ~core_rstn)

    // Retry count - saturates at all FFs
    `MSFF_R(retry_count_ff, (((cqe_proc_cs == WAIT_TRANS_DONE) && i_retransmit_accepted && o_retransmit_reqd) ? (~(&retry_count_ff) ? (retry_count_ff + 1'b1) : retry_count_ff) : retry_count_ff), core_clk, ~core_rstn)
    assign o_stat_retry_count = retry_count_ff;


    //--- sample incoming acknack data -----//
    // The reason why these values need to be sampled is to avoid the possibility
    // of an ack coming while the FSM is in progress for the same QP, in which case some values
    // may not be consistent denepding on when the ack was received.
    //


    // The phase bit gets updated one clock cycle after the incoming PSN. So, if
    // the PSN was updated, we need to give one extra clock cycle for the phase to
    // get updated
    `MSFF_R(phase_bit_acknack_psn_sampled, (acknacked_psn_rd_req_done ? i_phase_bit_acknack_psn : phase_bit_acknack_psn_sampled), core_clk, ~core_rstn)

    `MSFF_R(osq_nacked_sampled, (acknacked_psn_rd_req_done ? i_osq_nacked : osq_nacked_sampled), core_clk, ~core_rstn)

    //---------------------------------------

    assign nacked_psn_rolled_over = (((osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] < osq_entry_start_psn))) ? 1'b1 : 1'b0;

    assign osq_psn_rolled_over    = osq_entry_start_psn > osq_entry_end_psn ? 1'b1 : 1'b0;

    // Timeout retry case options
    //********************************
    // PSN      Option1 - OSQ rolled over =0. And PSN Mismatch = 0; so retry from {osq_acknacked_psn_sampled[23:0] , i_osq_entry_start_psn}
    // 8'hfd
    // 8'hfe    
    // 8'hff___
    // 8'h00    osq_start_psn
    // 8'h01    acknacked_psn
    // 8'h02    osq_end_psn
    // *******************************
    //********************************
    // PSN      Option2 - OSQ rolled over =0. And PSN Mismatch = 1; so retry from {osq_acknacked_psn_sampled[23:0] + 1'b1 , i_osq_entry_start_psn}
    // 8'hfd
    // 8'hfe    
    // 8'hff___ acknacked_psn
    // 8'h00    osq_start_psn
    // 8'h01    osq_end_psn
    // 8'h02
    // *******************************
    //********************************
    // PSN      Option3 - OSQ rolled over = 1. And PSN Mismatch = 0; so retry from {osq_acknacked_psn_sampled[23:8] -1'b1, i_osq_entry_start_psn}
    // 8'hfd
    // 8'hfe    osq_start_psn
    // 8'hff___
    // 8'h00    acknacked_psn
    // 8'h01    osq_end_psn
    // 8'h02
    // *******************************
    //********************************
    // PSN      Option4 - OSQ rolled over =1. And PSN Mismatch = 1; so retry from {osq_acknacked_psn_sampled[23:8] , i_osq_entry_start_psn}
    // 8'hfd
    // 8'hfe    osq_start_psn
    // 8'hff___ acknacked_psn
    // 8'h00    
    // 8'h01    osq_end_psn
    // 8'h02
    // *******************************
    always @(*) begin
        case ({
            osq_psn_rolled_over, phase_mismatch
        })
            2'b00: timeout_psn_to_retry = {osq_acknacked_psn[23:C_OSQ_PSN_WIDTH], osq_entry_start_psn};
            2'b01: timeout_psn_to_retry = {osq_acknacked_psn_plus1[23:C_OSQ_PSN_WIDTH], osq_entry_start_psn};
            2'b10: timeout_psn_to_retry = {(osq_acknacked_psn[23:C_OSQ_PSN_WIDTH] - 1'b1), osq_entry_start_psn};
            2'b11: timeout_psn_to_retry = {osq_acknacked_psn[23:C_OSQ_PSN_WIDTH], osq_entry_start_psn};
        endcase
    end

    `MSFF_R(timeout_psn_to_retry_ff, timeout_psn_to_retry, core_clk, ~core_rstn)

    //assign full_psn_to_retry = (nacked_psn_rolled_over && (~osq_timed_out || osq_rnr_nacked)) ? {(osq_acknacked_psn[23:C_OSQ_PSN_WIDTH] -1'b1), osq_entry_start_psn} : 
    //                           (osq_timed_out  ?  timeout_psn_to_retry_ff : {osq_acknacked_psn[23:C_OSQ_PSN_WIDTH], osq_entry_start_psn});
    assign full_psn_to_retry     = (osq_rnr_nacked || osq_nacked_sampled) ? osq_acknacked_psn[23:0] : (osq_acknacked_psn + 24'h000001);

    assign fsm_i_arbitrated_indx = (i_arbitration_done && resp_h_cs == IDLE) ? i_arbitrated_indx : arbitrated_indx_ff;
    assign o_intr_wqe_cmpl_sts   = completion_accepted && cqe_proc_intr_en;

    // Status should be set irrespective of enable and before the completion is
    // acceepted. 				 
    `MSFF_R(osq_timed_out, (acknacked_psn_rd_req_done ? osq_timed_out_sticky_fsm_arbitrated_indx : osq_timed_out), core_clk, ~core_rstn)

    assign psn_0_nacked             = (osq_nacked_sampled || osq_rnr_nacked) && (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] == 'b0);
    assign o_arbitrated_qp_for_intr = o_qp_cqe_proc_qpid;


    `MSFF_R(qp_cq_ba_ff, (i_qp_cq_ba_valid ? i_qp_cq_ba : qp_cq_ba_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_depth_ff, (i_qp_cq_depth_valid ? i_qp_cq_depth : qp_cq_depth_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_hdptr_ff, (i_qp_cq_hdptr_valid ? i_qp_cq_hdptr : qp_cq_hdptr_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_sq_cmpldb_addr_ff, (i_qp_sq_cmpldb_addr_valid ? i_qp_sq_cmpldb_addr : qp_sq_cmpldb_addr_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_intr_en_ff, (i_qp_conf_valid ? i_qp_conf[3] : qp_cq_intr_en_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_hw_hndshk_dis_ff, (i_qp_conf_valid ? i_qp_conf[4] : qp_hw_hndshk_dis_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cqe_write_en_ff, (i_qp_conf_valid ? i_qp_conf[5] : qp_cqe_write_en_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_sq_ba_ff, (i_qp_sq_ba_valid ? i_qp_sq_ba : qp_sq_ba_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_under_recovery_ff, (i_qp_conf_valid ? i_qp_conf[6] : qp_under_recovery_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_pmtu_ff, (i_qp_conf_valid ? i_qp_conf[10:8] : qp_pmtu_ff), core_clk, ~core_rstn)


    // i_arbitrated_indx 被锁存到 arbitrated_indx_ff，作为当前处理 QP 上下文索引。
    `MSFF_R(arbitrated_indx_ff, ((i_arbitration_done && (resp_h_cs == IDLE)) ? i_arbitrated_indx : arbitrated_indx_ff), core_clk, ~core_rstn)

    `MSFF_R(arbitrated_indx_2ff, arbitrated_indx_ff, core_clk, ~core_rstn)


    `MSFF_R(osq_empty_ff, i_osq_empty, core_clk, ~core_rstn)
    `MSFF_R(osq_empty_2ff, osq_empty_ff, core_clk, ~core_rstn)

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_i_first_outgoing_pkt_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_first_outgoing_pkt_valid),
        .wr_addr (init_start ? init_count : i_first_outgoing_pkt_id),
        .wr_data (init_start ? 1'b1 : i_first_outgoing_pkt_data),
        .rd_en   (1'b1),
        .rd_addr (i_wqe_qpid),
        .rd_data (i_first_outgoing_pkt_rd_data_int_0)
    );

    assign i_first_outgoing_pkt_rd_data_0 = (i_first_outgoing_pkt_valid_pending && i_first_outgoing_pkt_id_pending == i_wqe_qpid) ? i_first_outgoing_pkt_data_pending : i_first_outgoing_pkt_rd_data_int_0;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_i_first_outgoing_pkt_2 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_first_outgoing_pkt_valid),
        .wr_addr (init_start ? init_count : i_first_outgoing_pkt_id),
        .wr_data (init_start ? 1'b1 : i_first_outgoing_pkt_data),
        .rd_en   (1'b1),
        .rd_addr (first_og_trans_qpid),
        .rd_data (i_first_outgoing_pkt_rd_data_int_2)
    );

    assign i_first_outgoing_pkt_rd_data_2 = (i_first_outgoing_pkt_valid_pending && i_first_outgoing_pkt_id_pending == first_og_trans_qpid) ? i_first_outgoing_pkt_data_pending : i_first_outgoing_pkt_rd_data_int_2;

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_last_osq_psn (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || last_osq_psn_valid),
        .wr_addr    (init_start ? init_count : last_osq_psn_id),
        .wr_data    (init_start ? 1'b0 : last_osq_psn_data),
        .wr_en_aux  (last_osq_psn_valid_aux),
        .wr_addr_aux(last_osq_psn_id_aux),
        .wr_data_aux(last_osq_psn_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (arbitrated_indx_ff),
        .rd_data    (last_osq_psn_rd_data)
    );

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_phase_bit_osq_psn_ff_0 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || phase_bit_osq_psn_ff_valid),
        .wr_addr    (init_start ? init_count : phase_bit_osq_psn_ff_id),
        .wr_data    (init_start ? 1'b0 : phase_bit_osq_psn_ff_data),
        .wr_en_aux  (phase_bit_osq_psn_ff_valid_aux),
        .wr_addr_aux(phase_bit_osq_psn_ff_id_aux),
        .wr_data_aux(phase_bit_osq_psn_ff_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (arbitrated_indx_ff),
        .rd_data    (phase_bit_osq_psn_ff_rd_data_0)
    );

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_phase_bit_osq_psn_ff_1 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || phase_bit_osq_psn_ff_valid),
        .wr_addr    (init_start ? init_count : phase_bit_osq_psn_ff_id),
        .wr_data    (init_start ? 1'b0 : phase_bit_osq_psn_ff_data),
        .wr_en_aux  (phase_bit_osq_psn_ff_valid_aux),
        .wr_addr_aux(phase_bit_osq_psn_ff_id_aux),
        .wr_data_aux(phase_bit_osq_psn_ff_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (o_qp_cqe_proc_qpid),
        .rd_data    (phase_bit_osq_psn_ff_rd_data_1)
    );

    wire [C_OSQ_PSN_WIDTH-1:0] wqe_send_start_psn_less_one;
    wire [C_OSQ_PSN_WIDTH-1:0] retry_psn_less_one;

    always @(posedge core_clk) begin : last_osq_psn_ff
        if (~core_rstn) begin
            last_osq_psn_valid     <= 1'b0;
            last_osq_psn_id        <= {C_QP_INDX_WIDTH{1'b0}};
            last_osq_psn_data      <= 1'b1;
            last_osq_psn_valid_aux <= 1'b0;
            last_osq_psn_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
            last_osq_psn_data_aux  <= 1'b1;
        end else begin
            if (i_first_outgoing_pkt_rd_data_0 & i_wqe_push_req) begin
                last_osq_psn_valid <= 1'b1;
                last_osq_psn_id    <= i_wqe_qpid;
                last_osq_psn_data  <= wqe_send_start_psn_less_one[C_OSQ_PSN_WIDTH-1];
            end else begin
                last_osq_psn_valid <= 1'b0;
            end

            if (i_first_outgoing_pkt_rd_data_0 & i_wqe_push_req & arbitrated_indx_ff == i_wqe_qpid) begin
                last_osq_psn_valid_aux <= 1'b0;
            end else begin
                if (~(i_osq_empty | osq_empty_ff | osq_empty_2ff) & (i_rd_osq_vld_r2 & ~i_rd_osq_vld_r1)) begin
                    last_osq_psn_valid_aux <= 1'b1;
                    last_osq_psn_id_aux    <= arbitrated_indx_ff;
                    last_osq_psn_data_aux  <= i_osq_entry_end_psn[C_OSQ_PSN_WIDTH-1];
                end else if (resp_h_cs == RETRANSMIT) begin
                    last_osq_psn_valid_aux <= 1'b1;
                    last_osq_psn_id_aux    <= arbitrated_indx_ff;
                    last_osq_psn_data_aux  <= retry_psn_less_one[C_OSQ_PSN_WIDTH-1];
                end else begin
                    last_osq_psn_valid_aux <= 1'b0;
                end
            end
        end
    end

    always @(posedge core_clk) begin : phase_bit_osq_psn_ff
        if (~core_rstn) begin
            phase_bit_osq_psn_ff_valid     <= 1'b0;
            phase_bit_osq_psn_ff_id        <= {C_QP_INDX_WIDTH{1'b0}};
            phase_bit_osq_psn_ff_data      <= 1'b0;
            phase_bit_osq_psn_ff_valid_aux <= 1'b0;
            phase_bit_osq_psn_ff_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
            phase_bit_osq_psn_ff_data_aux  <= 1'b0;
        end else begin
            if (i_first_outgoing_pkt_valid && i_first_outgoing_pkt_data) begin
                phase_bit_osq_psn_ff_valid <= 1'b1;
                phase_bit_osq_psn_ff_id    <= i_first_outgoing_pkt_id;
                phase_bit_osq_psn_ff_data  <= 1'b0;
            end else begin
                phase_bit_osq_psn_ff_valid <= 1'b0;
            end

            if (((i_rd_osq_vld_r2 && ~i_rd_osq_vld_r1) || upd_phase_retransmit_pop) && ~(i_first_outgoing_pkt_valid && i_first_outgoing_pkt_data && i_first_outgoing_pkt_id == arbitrated_indx_ff)) begin
                phase_bit_osq_psn_ff_valid_aux <= 1'b1;
                phase_bit_osq_psn_ff_id_aux    <= arbitrated_indx_ff;
                phase_bit_osq_psn_ff_data_aux  <= phase_bit_osq_psn;
            end else begin
                phase_bit_osq_psn_ff_valid_aux <= 1'b0;
            end
        end
    end

    assign phase_bit_osq_psn               = ((~i_osq_entry_end_psn[C_OSQ_PSN_WIDTH-1] && last_osq_psn_for_arbitrated_idx) & ~(i_osq_empty | osq_empty_ff | osq_empty_2ff)) ? ~phase_bit_osq_psn_ff_rd_data_0 : phase_bit_osq_psn_ff_rd_data_0;

    // Rollover can occur on the incoming ack nacked psn and also the osq psn.
    // When both rollovers occur, they cancel each other. 
    assign last_osq_psn_for_arbitrated_idx = last_osq_psn_rd_data;
    assign wqe_send_start_psn_less_one     = i_wqe_send_start_psn - 1'b1;
    assign retry_psn_less_one              = psn_to_retry - 1'b1;
    assign phase_mismatch                  = (phase_bit_acknack_psn_sampled ^ phase_bit_osq_psn_ff_rd_data_0);

    // This signal is used to update the acknacked_psn post retry. After retry is
    // initiated, the psn of both the osq psn and acknacked_psn should match.
    assign o_phase_to_update_post_retry    = phase_bit_osq_psn_ff_rd_data_1;

    `MSFF_R(upd_phase_post_retransmit, ((resp_h_cs == START_RETRANSMIT) ? 1'b1 : (pop_osq ? 1'b0 : upd_phase_post_retransmit)), core_clk, ~core_rstn)
    assign upd_phase_retransmit_pop = upd_phase_post_retransmit & pop_osq;

    assign new_cq_hdptr             = (qp_cq_hdptr_ff == qp_cq_depth_ff) ? 16'h1 : (qp_cq_hdptr_ff + 1'b1);  // Updated CQ headpointer

    assign qp_cq_hdptr_base_0       = (qp_cq_hdptr_ff == qp_cq_depth_ff) ? 16'h0 : qp_cq_hdptr_ff;
    assign cqe_axi_address          = qp_cq_ba_ff + (qp_cq_hdptr_base_0 * CQE_ENTRY_SIZE);

    // Since the CQ and SQ depths are same and the last completed SQ WQE is
    // signified by the CQ head pointer, the next SQ WQE that needs to be
    // rewritten can be got from the CQ head pointer.
    assign next_rewrite_sq_ptr      = ~sq_ptr_cnt_en ? sq_ptr_frm_cqhd : (((cqe_proc_cs == WAIT_TRANS_DONE) && ((axi_mm_req && i_axi_master_done) || inc_sq_ptr_cnt)) ? next_sq_ptr : rewrite_sq_ptr_ff);
    assign next_sq_ptr              = ((rewrite_sq_ptr_ff + 1'b1) < qp_cq_depth_ff) ? (rewrite_sq_ptr_ff + 1'b1) : 16'b0;
    `MSFF_R(rewrite_sq_ptr_ff, next_rewrite_sq_ptr, core_clk, ~core_rstn)

    // this is to take care of the case when even the first command has not
    // completed. So CQHD is 0 (in no other case the CQHD will be 0). IN this case
    // dont do -1 just use the 0 value.
    assign sq_ptr_frm_cqhd           = (qp_cq_hdptr_ff == qp_cq_depth_ff) ? 16'b0 : qp_cq_hdptr_ff;

    assign o_arbitrate               = (resp_h_cs == ARBITRATE_OSQ) ? 1'b1 : 1'b0;
    assign o_sampled_arbitrated_indx = arbitrated_indx_ff;


    /* always@ (posedge core_clk) begin 
	if (resp_h_cs == CHECK_ERROR) begin
		if (arbitrated_indx_ff == QP0) begin
			fsm_pushback_valid <= 1'b0;
		end else if (~pending_cqdb_update & ((osq_rnr_nacked & ~osq_timed_out) || ((osq_nacked_sampled | osq_timed_out) & ~osq_wqe_nack_resp_ff))) begin
			fsm_pushback_valid <= 1'b1;
		end
	end else begin
		fsm_pushback_valid <= 1'b0;
	end
end */

    always @(posedge core_clk) begin
        rd_rsp_dest_addr_r1 <= i_rd_rsp_dest_addr;
        rd_rsp_dest_addr_r2 <= rd_rsp_dest_addr_r1;
        rd_rsp_dest_addr_r3 <= rd_rsp_dest_addr_r2;
    end

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            is_opcode_read <= 0;
        end else begin
            is_opcode_read <= (osq_opcode == RDMA_READ);
        end
    end
    generate
        begin
            if (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN) begin
                assign send_cq_db_data[31]   = ((resp_h_cs == INCR_CQ_CNT_UPD_DB) & (is_opcode_read == 1)) ? 1'b1 : 1'b0;
                assign send_cq_db_data[30:0] = {rd_rsp_dest_addr_r3[26:12], 16'h1};  //cq_dbcount_ff;
            end else if (NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING) begin
                assign send_cq_db_data[31]    = current_qp_fatal;
                assign send_cq_db_data[30:16] = i_osq_wrid[14:0];
                assign send_cq_db_data[15:0]  = 16'h1;
            end else begin
                assign send_cq_db_data = 32'h1;  //cq_dbcount_ff;
            end
        end
    endgenerate
    //---------------------

    // Generate the retransmitted SQ entry first 12Bytes
    assign sq_wqe_addr   = qp_sq_ba_ff + rewrite_sq_ptr_ff * SQ_ENTRY_SIZE;
    assign data_buf_addr = i_data_buf_ba + cqe_proc_data_bufid * i_data_buf_sz;
    `MSFF_R(data_buf_addr_ff, data_buf_addr, core_clk, ~core_rstn)
    `MSFF_R(data_buf_addr_2ff, data_buf_addr_ff, core_clk, ~core_rstn)
    //----------------------

    assign o_qp_sq_cmpldb_addr_req = cqe_cfg_rd_req;
    assign o_qp_cq_hdptr_req       = cqe_cfg_rd_req || o_qp_cq_hdptr_wrn;
    assign o_qp_cq_depth_req       = cqe_cfg_rd_req;
    assign o_qp_cq_ba_req          = cqe_cfg_rd_req;
    assign o_qp_sq_ba_req          = cqe_cfg_rd_req;
    assign o_qp_conf_req           = cfg_rd_req;
    //-----------------------Re-load the retry counter values-------//
    `MSFF_R(qp_curr_retry_cnt_ff, ((i_qp_curr_retry_cnt_rd_rdy & o_qp_curr_retry_cnt_rd_req) ? i_qp_curr_retry_cnt : qp_curr_retry_cnt_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cfg_retry_cnt_ff, ((i_qp_cfg_retry_cnt_rd_rdy & o_qp_cfg_retry_cnt_rd_req) ? i_qp_cfg_retry_cnt : qp_cfg_retry_cnt_ff), core_clk, ~core_rstn)

    assign o_qp_curr_retry_cnt_rd_req = cfg_rd_req;
    assign o_qp_cfg_retry_cnt_rd_req = cfg_rd_req;
    assign o_qp_curr_retry_cnt_wr_req = ((wr_retry_cnt) | (resp_h_cs == WRITE_CQE) | ((resp_h_cs == WAIT_VALID_DATA) & i_reload_retry_cnt_en_rd_data));
    assign o_qp_curr_retry_cnt_wr = (wr_retry_cnt) ? ((osq_nacked_sampled | osq_rnr_nacked) ? (qp_cfg_retry_cnt_ff - 1'b1) : (qp_curr_retry_cnt_ff - 1)) : qp_cfg_retry_cnt_ff;
    assign o_qp_retry_fatal_qpid = arbitrated_indx_ff;
    assign o_qp_curr_retry_cnt_qpid = arbitrated_indx_ff;
    assign o_qp_retry_fatal_req = (resp_h_cs == GEN_QP_FATAL) ? 1'b1 : 1'b0;
    assign {reload_retry_cnt_valid_pending_1, reload_retry_cnt_id_pending_1, reload_retry_cnt_data_pending_1, reload_retry_cnt_valid_pending_0, reload_retry_cnt_id_pending_0, reload_retry_cnt_valid, reload_retry_cnt_id, reload_retry_cnt_data} = i_reload_retry_cnt_en;
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_i_reload_retry_cnt_en (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || reload_retry_cnt_valid),
        .wr_addr (init_start ? init_count : reload_retry_cnt_id),
        .wr_data (init_start ? 1'b1 : reload_retry_cnt_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (i_reload_retry_cnt_en_rd_data_int)
    );

    always @(*) begin
        if (reload_retry_cnt_valid_pending_0 && reload_retry_cnt_id_pending_0 == arbitrated_indx_ff) begin
            i_reload_retry_cnt_en_rd_data = 1'b0;
        end else if (reload_retry_cnt_valid_pending_1 && reload_retry_cnt_id_pending_1 == arbitrated_indx_ff) begin
            i_reload_retry_cnt_en_rd_data = reload_retry_cnt_data_pending_1;
        end else begin
            i_reload_retry_cnt_en_rd_data = i_reload_retry_cnt_en_rd_data_int;
        end
    end

    //---Once qp_fatal & dma_idle & i_en_osq_flush_on_qp_fatal is detected
    //then proceed error completion preparation of rest of OSQ entries without
    //checking for i_dma_in_idle & i_en_osq_flush_on_qp_fatal
    assign o_pop_osq = pop_osq;

    //POP respective MPSN Buffer when qp_fatal_err is seen pop_osq is triggered.
    //On Qp_fatal flush all OSQ buffer and MPSN buffer entries and do not retry

    //The clear pending ack is added due to the following reason
    //In low performance cases, timeout happens even if an ack is pending to be
    //processed. Keep the timeout logic under reset if the ack is not yet
    //processed. Once processed, timeout can be enabled again



    always @(posedge core_clk) begin : o_clr_pending_ack
        if (~core_rstn) begin
            o_clr_pending_ack_set_valid <= 1'b0;
            o_clr_pending_ack_set_id    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (resp_h_cs == CHECK_ERROR && ~(osq_nacked_sampled & ~osq_wqe_nack_resp_ff) && i_osq_acknacked_psn == osq_acknacked_psn) begin
                o_clr_pending_ack_set_valid <= 1'b1;
                o_clr_pending_ack_set_id    <= arbitrated_indx_ff;
            end else begin
                o_clr_pending_ack_set_valid <= 1'b0;
            end
        end
    end

    //-----------Read Resp Last BResp Count----------//
    // Following change is added to make sure RDMA_READ_RESP data is written to
    // DDR before Read Completion is posted.

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      ((C_OS_Q_INDX_WIDTH + 1) * C_NUM_QP),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                              //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),                      //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                              //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                                  //string;
        .USE_MEM_INIT     (1),                                   //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),                     //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                                   //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(C_OS_Q_INDX_WIDTH + 1),  //positive integer
        .BYTE_WRITE_WIDTH_A(C_OS_Q_INDX_WIDTH + 1),  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),        //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (C_OS_Q_INDX_WIDTH + 1),  //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),        //positive integer
        .READ_RESET_VALUE_B("0"),                    //string
        .READ_LATENCY_B    (1),                      //non-negative integer
        .WRITE_MODE_B      ("read_first")            //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_rd_resp_wr_cnt (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (rd_resp_wr_cnt_bram_wren || first_og_trans),
        .addra         (rd_resp_wr_cnt_bram_wren ? rd_resp_wr_cnt_bram_wr_addr : first_og_trans_qpid),
        .dina          (rd_resp_wr_cnt_bram_wren ? rd_resp_wr_cnt_bram_dout + cnt_add : {C_OS_Q_INDX_WIDTH + 1{1'b0}}),
        .injectsbiterra(1'b0),                                                                                           //do not change
        .injectdbiterra(1'b0),                                                                                           //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (i_rd_rsp_wr_done),
        .regceb  (1'b1),
        .addrb   (i_rd_rsp_qpid),
        .doutb   (rd_resp_wr_cnt_bram_dout),
        //.doutb          (osq_out),
        .sbiterrb(),                          //do not change
        .dbiterrb()                           //do not change

    );
    assign mask_wr_en = i_rd_rsp_wr_done && rd_resp_wr_cnt_bram_wren && (rd_resp_wr_cnt_bram_wr_addr == i_rd_rsp_qpid);
    assign cnt_add    = i_rd_rsp_wr_done && rd_resp_wr_cnt_bram_wren && (rd_resp_wr_cnt_bram_wr_addr == i_rd_rsp_qpid) ? 4'b0010 : 4'b0001;
    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      ((C_OS_Q_INDX_WIDTH + 1) * C_NUM_QP),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                              //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),                      //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                              //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                                  //string;
        .USE_MEM_INIT     (1),                                   //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),                     //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                                   //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(C_OS_Q_INDX_WIDTH + 1),  //positive integer
        .BYTE_WRITE_WIDTH_A(C_OS_Q_INDX_WIDTH + 1),  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),        //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (C_OS_Q_INDX_WIDTH + 1),  //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),        //positive integer
        .READ_RESET_VALUE_B("0"),                    //string
        .READ_LATENCY_B    (1),                      //non-negative integer
        .WRITE_MODE_B      ("read_first")            //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_rd_resp_wr_cnt_mirror (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (rd_resp_wr_cnt_bram_wren || first_og_trans),
        .addra         (rd_resp_wr_cnt_bram_wren ? rd_resp_wr_cnt_bram_wr_addr : first_og_trans_qpid),
        .dina          (rd_resp_wr_cnt_bram_wren ? rd_resp_wr_cnt_bram_dout + cnt_add : {C_OS_Q_INDX_WIDTH + 1{1'b0}}),
        .injectsbiterra(1'b0),                                                                                           //do not change
        .injectdbiterra(1'b0),                                                                                           //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (rd_resp_bram_rden),
        .regceb  (1'b1),
        .addrb   (arbitrated_indx_ff),
        .doutb   (rd_resp_wr_cnt_bram_mirror_dout),
        //.doutb          (osq_out),
        .sbiterrb(),                                 //do not change
        .dbiterrb()                                  //do not change

    );

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      ((C_OS_Q_INDX_WIDTH + 1) * C_NUM_QP),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                              //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),                      //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                              //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                                  //string;
        .USE_MEM_INIT     (1),                                   //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),                     //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                                   //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(C_OS_Q_INDX_WIDTH + 1),  //positive integer
        .BYTE_WRITE_WIDTH_A(C_OS_Q_INDX_WIDTH + 1),  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),        //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (C_OS_Q_INDX_WIDTH + 1),  //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),        //positive integer
        .READ_RESET_VALUE_B("0"),                    //string
        .READ_LATENCY_B    (1),                      //non-negative integer
        .WRITE_MODE_B      ("read_first")            //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_rd_resp_processed_cnt (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (rd_resp_bram_wren || first_og_trans),
        .addra         (rd_resp_bram_wren ? arbitrated_indx_ff : first_og_trans_qpid),
        .dina          (rd_resp_bram_wren ? rd_resp_bram_dout + 4'b0001 : {C_OS_Q_INDX_WIDTH + 1{1'b0}}),
        .injectsbiterra(1'b0),                                                                             //do not change
        .injectdbiterra(1'b0),                                                                             //do not change

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (rd_resp_bram_rden),
        .regceb  (1'b1),
        .addrb   (arbitrated_indx_ff),
        .doutb   (rd_resp_bram_dout),
        //.doutb          (osq_out),
        .sbiterrb(),                    //do not change
        .dbiterrb()                     //do not change

    );




    //-----On QP_fatal, wqe_proc still send wqe_entries to resp_handler for error
    //completions preparation, For these completions FSM should not wait for
    //rd_resp_last_wr_done counter value. qp_fatal_flush will be set 1 to go ahead
    //with error completions preparation 
    always @(posedge core_clk) begin
        if (~core_rstn) qp_fatal_flush <= 'h0;
        else if ((resp_h_cs == CHECK_ERROR) & current_qp_fatal & i_dma_in_idle & i_en_osq_flush_on_qp_fatal) qp_fatal_flush <= 'h1;
        else if (resp_h_cs == ARBITRATE_OSQ) qp_fatal_flush <= 'h0;
    end

    assign o_resp_hndler_sts = {14'h0, i_acks_to_process, resp_h_cs, {(11 - C_QP_INDX_WIDTH) {1'b0}}, arbitrated_indx_ff};

    //-------------------------------------------------------------------------------------
    // Logic to see FSM states in Questa wavedumps (what a pain!!) :-/
    // Non-synthesizable code
    //-------------------------------------------------------------------------------------

`ifdef SIMULATION
    reg [20*8-1:0] resp_h_cs_string = "null";
    reg [20*8-1:0] resp_h_ns_string = "null";

    always @* begin
        case (resp_h_cs)
            IDLE:                          resp_h_cs_string = "IDLE              ";
            CHECK_ERROR:                   resp_h_cs_string = "CHECK_ERROR       ";
            WRITE_CQE:                     resp_h_cs_string = "WRITE_CQE         ";
            INCR_CQ_CNT_UPD_DB:            resp_h_cs_string = "INCR_CQ_CNT_UPD_DB";
            UPD_CQHD:                      resp_h_cs_string = "UPD_CQHD          ";
            WAIT_VALID_DATA:               resp_h_cs_string = "WAIT_VALID_DATA   ";
            CHECK_OSQ:                     resp_h_cs_string = "CHECK_OSQ         ";
            ARBITRATE_OSQ:                 resp_h_cs_string = "ARBITRATE_OSQ     ";
            RETRANSMIT:                    resp_h_cs_string = "RETRANSMIT        ";
            START_RETRANSMIT:              resp_h_cs_string = "START_RETRANSMIT  ";
            WRITE_CMPL_CNT:                resp_h_cs_string = "WRITE_CMPL_CNT    ";
            FLUSH_POP_OSQ:                 resp_h_cs_string = "FLUSH_POP_OSQ ";
            WAIT_DATA_STABLE:              resp_h_cs_string = "WAIT_DATA_STABLE ";
            CHECK_FLUSH_MPSNBUF:           resp_h_cs_string = "CHECK_FLUSH_MPSNBUF ";
            WAIT_FOR_RDMA_READ_RESP_WRITE: resp_h_cs_string = "WAIT_FOR_RDMA_READ_RESP_WRITE";
            //`ifdef QP_FATAL
            GEN_QP_FATAL:                  resp_h_cs_string = "GEN_QP_FATAL";
            GET_QP_CONF_DATA:              resp_h_cs_string = "GET_QP_CONF_DATA";
            //`endif
        endcase
        case (resp_h_ns)
            IDLE:                          resp_h_ns_string = "IDLE              ";
            CHECK_ERROR:                   resp_h_ns_string = "CHECK_ERROR       ";
            WRITE_CQE:                     resp_h_ns_string = "WRITE_CQE         ";
            INCR_CQ_CNT_UPD_DB:            resp_h_ns_string = "INCR_CQ_CNT_UPD_DB";
            UPD_CQHD:                      resp_h_ns_string = "UPD_CQHD          ";
            WAIT_VALID_DATA:               resp_h_ns_string = "WAIT_VALID_DATA   ";
            CHECK_OSQ:                     resp_h_ns_string = "CHECK_OSQ         ";
            ARBITRATE_OSQ:                 resp_h_ns_string = "ARBITRATE_OSQ     ";
            RETRANSMIT:                    resp_h_ns_string = "RETRANSMIT        ";
            START_RETRANSMIT:              resp_h_ns_string = "START_RETRANSMIT  ";
            WRITE_CMPL_CNT:                resp_h_ns_string = "WRITE_CMPL_CNT    ";
            FLUSH_POP_OSQ:                 resp_h_ns_string = "FLUSH_POP_OSQ ";
            WAIT_DATA_STABLE:              resp_h_ns_string = "WAIT_DATA_STABLE ";
            CHECK_FLUSH_MPSNBUF:           resp_h_ns_string = "CHECK_FLUSH_MPSNBUF ";
            WAIT_FOR_RDMA_READ_RESP_WRITE: resp_h_ns_string = "WAIT_FOR_RDMA_READ_RESP_WRITE";
            //`ifdef QP_FATAL
            GEN_QP_FATAL:                  resp_h_ns_string = "GEN_QP_FATAL";
            GET_QP_CONF_DATA:              resp_h_ns_string = "GET_QP_CONF_DATA";
            //`endif
        endcase
    end
`endif

    //-------------------------------------------------------------------------------------

endmodule
