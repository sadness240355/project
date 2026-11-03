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
// wqe_proc_hdr_gen.v
//-----------------------------------------------------------------------------
//
// Filename        : wqe_proc_hdr_gen.v
// Version         : v1.0
// Description     : WQE处理器头部生成模块包含从QP管理器读取WQE并根据寄存器信息生成头部的逻辑。
//                   该模块负责生成和连接这些头部。
//                   1、从 qpm_fifo(SQ WQE FIFO) 取 WQE 并生成头部
//                   2、收到接收侧事件(AETH ACK 事件 & RD 响应请求事件（生成 RD_RESP 包头）)后，生成 ACK/响应头
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
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module wqe_proc_hdr_gen #(
    parameter C_MAX_QP = 2048,
    parameter C_MAX_QID_WIDTH = 11,  // 5
    //parameter C_MAX_WRDATA_BUF_NUM = 128,
    parameter C_OS_Q_INDX_WIDTH = 3,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_EN_DEBUG = 0,
    parameter C_EN_RECV_RD_WR = 1,  // 1，使能接收侧 RD 相关扩展路径
    parameter NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN = 1,  // 1，使能 SEND_INVALIDATE/IMMDT 相关功能
    //parameter C_EN_WR_RETRY_DATA_BUF = 1,
    parameter PSN_based_retransmission = 1
) (
    input core_clk,
    input core_rst,

    // SQ WQE FIFO Interface
    // qpm_fifo 就是 SQ WQE FIFO
    input                               i_qpm_fifo_empty,
    input      [                 511:0] i_qpm_wqe_data,
    output reg                          o_qpm_wqe_pop,
    // i_wqe_halt=1 时，不再从 QPM FIFO 拉取新的 SQ WQE
    input                               i_wqe_halt,         // 暂停请求
    input      [C_MAX_QID_WIDTH -1 : 0] i_wqe_halted_qpid,
    // 模块不会立刻硬停，而是走到安全点后把 o_wqe_halted 拉高，表示“已经停稳，可以认为暂停生效”。
    output reg                          o_wqe_halted,       // 确认暂停

    // 与 Intiator 交互的 QP 配置信息
    output reg                          o_reg_rd_en,            // 读 QP 配置请求
    output reg                          o_reg_rd_en_s,          // 读 QP 配置请求
    input                               i_reg_info_val,         // QP 配置有效
    input                               i_reg_psn_val,          // SQPSNi 寄存器值有效
    output reg                          o_reg_wr_en,            // STATSSNi 和 SQPSNi 的写请求使能，相当于是 QP 寄存器写回正在进行
    output reg [   C_MAX_QID_WIDTH-1:0] o_qp_id,                // qpid, WQE Structure -- i_qpm_wqe_data[136+C_MAX_QID_WIDTH-1 : 136]，即i_qpm_wqe_data[140:136]
    input      [                  15:0] i_reg_pkey,             // o_reg_rd_en 返回 QPADVCONFi[31:16]
    input      [                   2:0] i_reg_pmtu,             // QPCONFi[10:8]
    input      [                  23:0] i_reg_dest_qpid,        // o_reg_rd_en 返回 DESTQPCONFi[23:0]
    input      [                  23:0] i_reg_sq_psn,           // SQPSNi[23:0]
    output reg [                  23:0] o_reg_sq_psn,
    input      [                  23:0] i_reg_sq_msn,           // STATSSNi[23:0]
    output reg [                  23:0] o_reg_sq_msn,
    input      [                   7:0] i_reg_max_rd_atomic,    // QPCONFi[23:16]
    input      [                  47:0] i_reg_mac_dest_addr,    // o_reg_rd_en_s 返回 MACDESADD
    input      [                 127:0] i_reg_ip_dest_addr,     // o_reg_rd_en_s 返回 IPDESADDR
    input      [                  15:0] i_reg_udp_src_port,     // XRNICCONF[23:8]
    input      [                   7:0] i_reg_ttl,              // o_reg_rd_en 返回 QPADVCONFi[15:8]
    input      [                   5:0] i_reg_dscp,             // o_reg_rd_en 返回 QPADVCONFi[5:0]
    input                               i_reg_exp_ack,          // ~QPCONFi[1]
    input      [C_M_AXI_ADDR_WIDTH-1:0] i_reg_wrdata_buf_ba,    // DATBUFBA
    input      [                  15:0] i_reg_wrdata_num_bufs,  // DATBUFSZ[15:0]
    input      [                  15:0] i_reg_wrdata_buf_sz,    // DATBUFSZ[31:16]

    // resp_handler 接口
    input        i_freeup_data_buf,
    input [14:0] i_freeup_data_bufid,
    input [ 4:0] i_reg_qp_to,

    // RNIC 配置信息
    input [ 47:0] i_reg_mac_src_addr,
    input [ 31:0] i_reg_ipv4_src_addr,
    input [127:0] i_reg_ipv6_src_addr,
    input         i_reg_xrnic_en,
    input         i_reg_ip_ver,

    input               i_buf_mngr_rdy,
    output reg [1023:0] o_hdr_data,
    output reg [   7:0] o_hdr_len,
    output reg          o_hdr_valid,
    output reg          o_hdr_ip_ver,

    // 有关 SQ WQE 的信息
    output reg                          o_wqp_req,
    output reg                          o_wqp_req_Read,
    input                               i_wqp_resp,
    output reg [   C_MAX_QID_WIDTH-1:0] o_wqe_qpid,
    output reg [                   7:0] o_wqe_opcode,
    output reg [                  15:0] o_wqe_wrid,
    output reg [                  23:0] o_wqe_send_psn,
    output reg [                  14:0] o_wqe_data_bufid,
    output reg [                   4:0] o_wqe_qp_to,
    output reg                          o_wqe_retried,
    output reg [                  23:0] o_wqe_send_first_psn,
    output reg [                  23:0] o_wqe_send_msn,
    output reg [C_M_AXI_ADDR_WIDTH-1:0] o_wqe_rdma_rd_rcv_ba,
    output reg [                  31:0] o_wqe_rdma_rd_rcv_len,
    output reg                          o_wqe_explicit_ack_req,
    output reg                          o_wqe_fatal_push,

    output reg                       o_hdr_gen_rdy,
    input                            i_aeth_valid,
    input      [               31:0] i_aeth_hdr,
    input      [C_MAX_QID_WIDTH-1:0] i_aeth_qpid,
    input      [               23:0] i_aeth_psn,
    input      [               63:0] i_rd_req_addr,
    input      [               31:0] i_rd_req_len,
    input                            i_rd_req_valid,
    input      [                1:0] i_rd_req_start_with_middle,

    output reg                          wqe_opcode_err,
    output reg [                   7:0] err_opcode,
    output     [                  11:0] wqe_status_out,
    output reg [                  31:0] o_last_out_pkt_info,
    output reg [                  15:0] o_out_send_pkt_cnt,
    output reg [                  15:0] o_out_rw_pkt_cnt,
    output reg [                  15:0] o_out_ack_pkt_cnt,
    output reg [                  15:0] o_out_mad_pkt_cnt,
    output reg [                  15:0] o_out_rd_resp_pkt_cnt,
    input      [ C_OS_Q_INDX_WIDTH : 0] os_num_vld_entries,
    //input [C_MAX_QP - 1 : 0] 	i_osq_nacked,
    input                               i_qp_recovery,
    //output reg [C_MAX_QP - 1 : 0] 	o_osq_nacked_resp,
    output reg                          o_data_buf_fifo_wr_en,
    output reg [C_M_AXI_ADDR_WIDTH-1:0] o_data_buf_fifo_data,
    input                               i_data_buf_fifo_empty,
    input                               dma_in_idle,
    output                              o_dma_in_idle,

    input             i_debug_cnt_en,
    input             i_debug_cnt_clr,
    output reg [15:0] o_wqe_proc_idle_cnt,
    output reg [15:0] o_wqe_proc_rd_wqe_cnt,
    output reg [15:0] o_wqe_proc_rd_q_info_cnt,
    output reg [15:0] o_wqe_proc_wait0_cnt,
    output reg [15:0] o_wqe_proc_ip_chksum_cnt,
    output reg [15:0] o_wqe_proc_hdr_gen_cnt,
    output reg [15:0] o_wqe_proc_hdr_sto_cnt,

    output reg [  C_MAX_QID_WIDTH-1:0] o_wqe_fatal_qpid,
    output reg                         o_wqe_fatal_qp_vld,
    input                              i_wqe_fatal_qp_rdy,
    output reg                         o_wqe_dropped,
    output reg [  C_MAX_QID_WIDTH-1:0] o_wqe_dropped_qpid,
    input                              i_osq_pop,
    input      [  C_MAX_QID_WIDTH-1:0] i_osq_pop_qpid,
    input                              init_start,
    input      [C_MAX_QID_WIDTH-1 : 0] init_count,
    input                              i_qp_fatal_wr_en,
    input      [C_MAX_QID_WIDTH-1 : 0] i_qp_fatal_wr_addr,
    input                              i_qp_fatal_wr_data,
    input                              i_osq_nacked_wr_en,
    input      [C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr,
    input                              i_osq_nacked_wr_data,
    input                              i_osq_nacked_wr_en_aux,
    input      [C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_aux,
    input                              i_osq_nacked_wr_data_aux,
    input      [C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_pending,
    input                              i_osq_nacked_wr_en_pending,
    input      [C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_aux_pending,
    input                              i_osq_nacked_wr_en_aux_pending,
    output reg                         o_osq_nacked_resp_wr_en,
    output reg [C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr,
    output reg                         o_osq_nacked_resp_wr_data,
    output reg                         o_osq_nacked_resp_wr_en_aux,
    output reg [C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr_aux,
    output reg                         o_osq_nacked_resp_wr_data_aux,
    output reg                         o_osq_nacked_resp_wr_en_pending,
    output reg [C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr_pending,
    output reg                         o_osq_nacked_resp_wr_data_pending
);

    `include "../../rdma_macros.vh"

    localparam IDLE = 4'b0000;
    localparam RD_WQE = 4'b0001;
    localparam RD_Q_INFO = 4'b0010;
    localparam WAIT0 = 4'b0011;
    localparam IP_CHKSUM = 4'b0100;
    localparam HDR_GEN = 4'b0101;
    localparam HDR_STO = 4'b0110;
    localparam ILL_OPCODE = 4'b0111;
    localparam DROP_WQE = 4'b1000;
    localparam OS_TH_ENTRIES = 3;
    //localparam DATA_BUF_ID_WIDTH = clog2(C_MAX_WRDATA_BUF_NUM);
    localparam DATA_BUF_ID_WIDTH = 7;
    localparam partial_wqe_bit_pos = 152;
    //localparam partial_wqe_bit_pos = 511;

    reg     [                  111:0] eth_hdr;
    reg     [                  319:0] ip_hdr;
    reg     [                   63:0] udp_hdr;
    reg     [                   95:0] bth_hdr;
    reg     [                  127:0] reth_hdr;
    reg     [                   31:0] aeth_hdr;
    reg     [                    3:0] hdr_gen_cs;
    reg     [                   19:0] ip_hdr_chksum;
    reg     [                   31:0] rem_len;  // WQE Structure -- LENGTH，传输长度
    reg     [                   31:0] dma_len;  // WQE Structure -- LENGTH，传输长度
    reg     [                    7:0] wqe_opcode;  // WQE Structure -- OPCODE，操作码。
    // first_trans 天然就是“每条 WQE 只打一拍事件”，这直接避免了分片/middle/last 多包场景下重复记账
    reg                               first_trans;  // 当前事务首包标志(first/only)，正常发送完首包后清0，不只给 SQ WQE 用，也给 RD 响应（rd_resp）这条路径用
    reg     [                   15:0] payload_len;  // 每包长度，仅有效负载长度，不包括头部长度。(把总长度按 PMTU/状态切片后，当前这一包实际发送的 payload 字节数)
    reg                               reth_present;
    reg     [                   63:0] local_offset;  // WQE Structure -- LADDR，本地地址。操作数据缓冲区的地址
    reg     [                   63:0] remote_offset;  // WQE Structure -- ROFFSET，远端偏移地址
    reg     [                   31:0] r_key;  // WQE Structure -- RTAG，远程 TAG
    reg                               wqe_push_pending;  // WQE 已发起入队(OSQ)请求、但对端还没应答
    reg                               aeth_pkt;
    reg                               chksum_cal_start;
    reg     [                  127:0] nvme_comp_data;  // WQE Structure -- SDATA，RDMA 发送数据。如果要发送的数据小于或等于 16B，则使用该字段来表示数据。
    reg                               completion_data_present;  // 表示这包要携带“内联的小块 SDATA(nvme_comp_data)”
    reg     [                   12:0] max_pmtu;
    reg     [                   23:0] dest_qpid;
    wire    [                   15:0] q_id;
    wire    [                   15:0] ipv6_len;
    reg     [                   17:0] ip_hdr_chksum_add_pipe;
    reg     [                   15:0] ipv4_len;
    reg                               ipv4_len_cal;
    reg     [                   47:0] mac_dest_addr;
    reg     [                  127:0] ip_dest_addr;
    reg                               ip_ver;
    reg                               exp_ack;
    reg                               wqe_os_fifo_full;
    reg     [                   15:0] ipv4_identification;
    reg                               push_to_os_buf;  // 每个 WQE 只在首包(first_trans=1)时 push 一次 OSQ 记录，后续 middle/last 包会置 0
    //Added for performance update. FSM will proceed to the next header generation logic
    //and waits in HDR_GEN state until this signal is de-asserted
    reg                               buf_mgr_busy;
    reg     [                    4:0] reg_qp_to;
    reg                               wait_for_dma_idle;  // 这里固定为1'b0
    reg                               retried;
    wire    [DATA_BUF_ID_WIDTH-1 : 0] data_bufid;
    reg     [DATA_BUF_ID_WIDTH-1 : 0] retried_buf_id;
    reg                               get_data_buf;
    wire    [ C_M_AXI_ADDR_WIDTH-1:0] data_buf_addr;
    wire                              no_data_buf_left;  // 1'b0
    reg                               wqe_pushed_in_fatal;
    integer                           i;
    reg                               partial_wqe;  // 这条 WQE 是否走了 qp_mgr_top 重传/续传路径”的标志位
    reg     [                 23 : 0] partial_wqe_start_psn;
    reg                               wait_for_dma_idle_r;
    reg                               idle_detected;
    reg                               qp_fatal_detected_wr_en;
    reg     [  C_MAX_QID_WIDTH-1 : 0] qp_fatal_detected_wr_addr;
    reg                               qp_fatal_detected_wr_data;
    reg                               qp_fatal_detected_wr_en_aux;
    reg     [  C_MAX_QID_WIDTH-1 : 0] qp_fatal_detected_wr_addr_aux;
    reg                               qp_fatal_detected_wr_data_aux;
    reg                               qp_wqe_opcode_fatal_wr_en;
    reg     [  C_MAX_QID_WIDTH-1 : 0] qp_wqe_opcode_fatal_wr_addr;
    reg                               qp_wqe_opcode_fatal_wr_data;
    reg                               qp_wqe_opcode_fatal_wr_en_aux;
    reg     [  C_MAX_QID_WIDTH-1 : 0] qp_wqe_opcode_fatal_wr_addr_aux;
    reg                               qp_wqe_opcode_fatal_wr_data_aux;
    wire                              qp_fatal_detected_rd_data_0;
    wire                              qp_fatal_detected_rd_data_1;
    wire                              qp_fatal_rd_data_0;
    wire                              qp_fatal_rd_data_1;
    wire                              osq_nacked_rd_data_0;
    wire                              osq_nacked_rd_data_int_1;
    reg                               osq_nacked_rd_data_1;
    wire                              qp_wqe_opcode_fatal_rd_data;
    reg                               osq_nacked_wr_pending;
    reg     [  C_MAX_QID_WIDTH-1 : 0] osq_nacked_wr_id;
    reg                               osq_nacked_wr_data;


    assign q_id           = {{16 - C_MAX_QID_WIDTH{1'b0}}, o_qp_id};
    assign wqe_status_out = {4'h0, buf_mgr_busy, i_data_buf_fifo_empty, no_data_buf_left, wqe_push_pending, hdr_gen_cs};

    //////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Debug counters logic //////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    generate
        if (C_EN_DEBUG == 1) begin : DEBUG_EN
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_wqe_proc_idle_cnt <= 16'h0000;
                end else begin
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_idle_cnt <= 16'h0000;
                    end else if (i_debug_cnt_en && (hdr_gen_cs == IDLE)) begin
                        o_wqe_proc_idle_cnt <= o_wqe_proc_idle_cnt + 1'b1;
                    end
                end
            end
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_wqe_proc_rd_wqe_cnt <= 16'h0000;
                end else begin
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_rd_wqe_cnt <= 16'h0000;
                        //end else if (i_debug_cnt_en && (hdr_gen_cs == RD_WQE) ) begin
                    end else if (i_debug_cnt_en && ((hdr_gen_cs == IDLE) && i_qpm_fifo_empty)) begin
                        o_wqe_proc_rd_wqe_cnt <= o_wqe_proc_rd_wqe_cnt + 1'b1;
                    end
                end
            end
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_wqe_proc_rd_q_info_cnt <= 16'h0000;
                end else begin
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_rd_q_info_cnt <= 16'h0000;
                    end else if (i_debug_cnt_en && (hdr_gen_cs == RD_Q_INFO)) begin
                        o_wqe_proc_rd_q_info_cnt <= o_wqe_proc_rd_q_info_cnt + 1'b1;
                    end
                end
            end
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_wqe_proc_wait0_cnt <= 16'h0000;
                end else begin
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_wait0_cnt <= 16'h0000;
                    end else if (i_debug_cnt_en && (hdr_gen_cs == WAIT0)) begin
                        o_wqe_proc_wait0_cnt <= o_wqe_proc_wait0_cnt + 1'b1;
                    end
                end
            end
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_wqe_proc_ip_chksum_cnt <= 16'h0000;
                end else begin
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_ip_chksum_cnt <= 16'h0000;
                    end else if (i_debug_cnt_en && (hdr_gen_cs == IP_CHKSUM)) begin
                        o_wqe_proc_ip_chksum_cnt <= o_wqe_proc_ip_chksum_cnt + 1'b1;
                    end
                end
            end
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_wqe_proc_hdr_gen_cnt <= 16'h0000;
                end else begin
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_hdr_gen_cnt <= 16'h0000;
                    end else if (i_debug_cnt_en && (hdr_gen_cs == HDR_GEN)) begin
                        o_wqe_proc_hdr_gen_cnt <= o_wqe_proc_hdr_gen_cnt + 1'b1;
                    end
                end
            end
            always @(posedge core_clk) o_wqe_proc_hdr_sto_cnt <= 0;
            //always @(posedge core_clk)
            //begin
            //    if(core_rst) begin
            //        o_wqe_proc_hdr_sto_cnt <= 16'h0000;
            //    end else begin
            //        if(i_debug_cnt_clr) begin
            //            o_wqe_proc_hdr_sto_cnt <= 16'h0000;
            //        //end else if (i_debug_cnt_en && (hdr_gen_cs == HDR_STO)) begin
            //        end else if (i_debug_cnt_en && (hdr_gen_cs == IDLE) && i_wqe_halt) begin
            //            o_wqe_proc_hdr_sto_cnt <= o_wqe_proc_hdr_sto_cnt + 1'b1;
            //        end
            //    end
            //end
        end else begin
            always @(posedge core_clk) begin
                o_wqe_proc_idle_cnt      <= 16'h0000;
                o_wqe_proc_rd_wqe_cnt    <= 16'h0000;
                o_wqe_proc_rd_q_info_cnt <= 16'h0000;
                o_wqe_proc_wait0_cnt     <= 16'h0000;
                o_wqe_proc_ip_chksum_cnt <= 16'h0000;
                o_wqe_proc_hdr_gen_cnt   <= 16'h0000;
                o_wqe_proc_hdr_sto_cnt   <= 16'h0000;
            end
        end
    endgenerate

    // Common LUT RAM Instances for per QP signals

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_i_qp_fatal_1 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || i_qp_fatal_wr_en),
        .wr_addr (init_start ? init_count : i_qp_fatal_wr_addr),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_wr_data),
        .rd_en   (1'b1),
        .rd_addr (o_qp_id),
        .rd_data (qp_fatal_rd_data_1)
    );

    assign osq_nacked_rd_data_0 = 1'b0;

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_i_osq_nacked_1 (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || i_osq_nacked_wr_en),
        .wr_addr    (init_start ? init_count : i_osq_nacked_wr_addr),
        .wr_data    (init_start ? 1'b0 : i_osq_nacked_wr_data),
        .wr_en_aux  (i_osq_nacked_wr_en_aux),
        .wr_addr_aux(i_osq_nacked_wr_addr_aux),
        .wr_data_aux(i_osq_nacked_wr_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (o_qp_id),
        .rd_data    (osq_nacked_rd_data_int_1)
    );
    always @(*) begin
        if (i_osq_nacked_wr_en_pending && i_osq_nacked_wr_addr_pending == o_qp_id) begin
            osq_nacked_rd_data_1 = 1'b1;
        end else if (i_osq_nacked_wr_en_aux_pending && i_osq_nacked_wr_addr_aux_pending == o_qp_id) begin
            osq_nacked_rd_data_1 = 1'b0;
        end else begin
            osq_nacked_rd_data_1 = osq_nacked_rd_data_int_1;
        end
    end

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP),
        .C_Init_with_1(0)
    ) inst_qp_wqe_opcode_fatal (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || qp_wqe_opcode_fatal_wr_en),
        .wr_addr    (init_start ? init_count : qp_wqe_opcode_fatal_wr_addr),
        .wr_data    (init_start ? 1'b0 : qp_wqe_opcode_fatal_wr_data),
        .wr_en_aux  (qp_wqe_opcode_fatal_wr_en_aux),
        .wr_addr_aux(qp_wqe_opcode_fatal_wr_addr_aux),
        .wr_data_aux(qp_wqe_opcode_fatal_wr_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (o_qp_id),
        .rd_data    (qp_wqe_opcode_fatal_rd_data)
    );

    generate
        if (C_EN_RECV_RD_WR == 1 && NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 1) begin : RD_RESP  // current

            reg        rd_resp;
            reg [ 1:0] rd_resp_start_with_middle;
            reg [23:0] rd_resp_psn;
            reg        rd_resp_aeth_present;
            reg [31:0] imm_data;
            reg        imm_data_present;


            assign ipv6_len = (((first_trans && ~|rem_len) || (~first_trans && ~|rem_len)) && (wqe_opcode == 8'h0C || wqe_opcode == 8'h03 || wqe_opcode == 8'h01) && (!aeth_pkt)) ? (8'h18 + payload_len + {reth_present,1'b0,(aeth_pkt || rd_resp_aeth_present),2'b00} + 3'd4) : (8'h18 + payload_len + {reth_present,1'b0,(aeth_pkt || rd_resp_aeth_present),2'b00});
            //assign ipv4_len = (((first_trans && ~|rem_len) || (~first_trans && ~|rem_len)) && (wqe_opcode == 8'h0C || wqe_opcode == 8'h03 || wqe_opcode == 8'h01) && (!aeth_pkt)) ? (8'h2C + payload_len + {reth_present,1'b0,(aeth_pkt || rd_resp_aeth_present),2'b00} + 3'd4) : (8'h2C + payload_len + {reth_present,1'b0,(aeth_pkt || rd_resp_aeth_present),2'b00});

            always @(posedge core_clk) begin
                if (core_rst) begin
                    eth_hdr                           <= 'd0;
                    ip_hdr                            <= 'd0;
                    udp_hdr                           <= 'd0;
                    bth_hdr                           <= 'd0;
                    reth_hdr                          <= 'd0;
                    aeth_hdr                          <= 'd0;
                    o_hdr_data                        <= 'd0;
                    o_hdr_valid                       <= 1'b0;
                    o_hdr_ip_ver                      <= 1'b0;
                    o_hdr_len                         <= 8'h00;
                    o_qpm_wqe_pop                     <= 1'b0;
                    o_qp_id                           <= {C_MAX_QID_WIDTH{1'b0}};
                    o_reg_rd_en                       <= 1'b0;
                    o_reg_rd_en_s                     <= 1'b0;
                    hdr_gen_cs                        <= IDLE;
                    ip_hdr_chksum                     <= 20'd0;
                    rem_len                           <= 32'd0;
                    dma_len                           <= 32'd0;
                    first_trans                       <= 1'b0;
                    payload_len                       <= 16'h0000;
                    reth_present                      <= 1'b0;
                    remote_offset                     <= 64'd00;
                    local_offset                      <= 64'd00;
                    aeth_pkt                          <= 1'b0;
                    wqe_opcode                        <= 8'h00;
                    r_key                             <= 32'd0;
                    chksum_cal_start                  <= 1'b0;
                    nvme_comp_data                    <= 128'd0000;
                    completion_data_present           <= 1'b0;
                    o_reg_wr_en                       <= 1'b0;
                    wqe_opcode_err                    <= 1'b0;
                    err_opcode                        <= 8'h00;
                    dest_qpid                         <= 24'd0;
                    o_hdr_gen_rdy                     <= 1'b0;
                    o_reg_sq_psn                      <= 24'd0;
                    o_wqe_send_first_psn              <= 24'd0;
                    o_reg_sq_msn                      <= 24'd0;
                    o_wqe_wrid                        <= 16'h0000;
                    mac_dest_addr                     <= 'd0;
                    ip_dest_addr                      <= 'd0;
                    ip_ver                            <= 1'b0;
                    exp_ack                           <= 1'b0;
                    o_out_send_pkt_cnt                <= 16'h0000;
                    o_out_rw_pkt_cnt                  <= 16'h0000;
                    o_out_ack_pkt_cnt                 <= 16'h0000;
                    o_out_mad_pkt_cnt                 <= 16'h0000;
                    o_wqe_halted                      <= 1'b0;
                    wqe_os_fifo_full                  <= 1'b0;
                    ipv4_identification               <= 16'h0000;
                    retried                           <= 1'b0;
                    get_data_buf                      <= 1'b0;
                    o_data_buf_fifo_data              <= 'd0;
                    o_data_buf_fifo_wr_en             <= 1'b0;
                    //o_osq_nacked_resp <= 'd0;
                    push_to_os_buf                    <= 1'b0;
                    buf_mgr_busy                      <= 1'b0;
                    reg_qp_to                         <= 5'h00;
                    wait_for_dma_idle                 <= 1'b0;
                    //qp_fatal_detected<= 'h0;
                    retried_buf_id                    <= 'h0;
                    wqe_pushed_in_fatal               <= 1'b0;
                    o_wqe_fatal_qpid                  <= {C_MAX_QID_WIDTH{1'b0}};
                    o_wqe_fatal_qp_vld                <= 1'b0;
                    rd_resp                           <= 1'b0;
                    rd_resp_start_with_middle         <= 2'b00;
                    rd_resp_psn                       <= 24'd0;
                    rd_resp_aeth_present              <= 1'b0;
                    o_out_rd_resp_pkt_cnt             <= 16'h0000;
                    o_wqe_dropped                     <= 1'b0;
                    o_wqe_dropped_qpid                <= {C_MAX_QID_WIDTH{1'b0}};
                    imm_data                          <= 32'd0;
                    imm_data_present                  <= 1'b0;
                    ipv4_len_cal                      <= 1'b0;
                    ip_hdr_chksum_add_pipe            <= 17'd0;
                    ipv4_len                          <= 16'd0;
                    partial_wqe                       <= 1'b0;
                    partial_wqe_start_psn             <= 24'd0;
                    qp_fatal_detected_wr_en           <= 1'b0;
                    qp_fatal_detected_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_fatal_detected_wr_data         <= 1'b0;
                    qp_fatal_detected_wr_en_aux       <= 1'b0;
                    qp_fatal_detected_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_fatal_detected_wr_data_aux     <= 1'b0;
                    qp_wqe_opcode_fatal_wr_en         <= 1'b0;
                    qp_wqe_opcode_fatal_wr_addr       <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_wqe_opcode_fatal_wr_data       <= 1'b0;
                    qp_wqe_opcode_fatal_wr_en_aux     <= 1'b0;
                    qp_wqe_opcode_fatal_wr_addr_aux   <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_wqe_opcode_fatal_wr_data_aux   <= 1'b0;
                    o_osq_nacked_resp_wr_en           <= 1'b0;
                    o_osq_nacked_resp_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data         <= 1'b0;
                    o_osq_nacked_resp_wr_en_aux       <= 1'b0;
                    o_osq_nacked_resp_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data_aux     <= 1'b0;
                    o_osq_nacked_resp_wr_en_pending   <= 1'b0;
                    o_osq_nacked_resp_wr_addr_pending <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data_pending <= 1'b0;
                    osq_nacked_wr_pending             <= 1'b0;
                    osq_nacked_wr_id                  <= {C_MAX_QID_WIDTH{1'b0}};
                    osq_nacked_wr_data                <= 1'b0;
                end else begin

                    if (~wqe_push_pending && (hdr_gen_cs == HDR_STO)) begin
                        o_hdr_valid  <= 1'b1;
                        o_hdr_ip_ver <= ip_ver;
                    end else if (i_buf_mngr_rdy) begin
                        o_hdr_valid <= 1'b0;
                    end

                    if ((hdr_gen_cs == ILL_OPCODE) && ~o_wqe_fatal_qp_vld) begin
                        o_wqe_fatal_qp_vld <= 1'b1;
                    end else if (i_wqe_fatal_qp_rdy) begin
                        o_wqe_fatal_qp_vld <= 1'b0;
                    end

                    if (o_wqe_fatal_qp_vld && i_qp_fatal_wr_en && o_wqe_fatal_qpid == i_qp_fatal_wr_addr) begin
                        qp_wqe_opcode_fatal_wr_en     <= 1'b1;
                        qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                        qp_wqe_opcode_fatal_wr_addr   <= o_wqe_fatal_qpid;
                        qp_wqe_opcode_fatal_wr_data   <= 1'b1;
                    end else begin
                        if (o_wqe_fatal_qp_vld) begin
                            qp_wqe_opcode_fatal_wr_en   <= 1'b1;
                            qp_wqe_opcode_fatal_wr_addr <= o_wqe_fatal_qpid;
                            qp_wqe_opcode_fatal_wr_data <= 1'b1;
                        end else begin
                            qp_wqe_opcode_fatal_wr_en <= 1'b0;
                        end

                        if (i_qp_fatal_wr_en && i_qp_fatal_wr_data) begin
                            qp_wqe_opcode_fatal_wr_en_aux   <= 1'b1;
                            qp_wqe_opcode_fatal_wr_addr_aux <= i_qp_fatal_wr_addr;
                            qp_wqe_opcode_fatal_wr_data_aux <= 1'b0;
                        end else begin
                            qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                        end
                    end

                    if (o_hdr_valid && ~i_buf_mngr_rdy) begin
                        buf_mgr_busy <= 1'b1;
                    end else if (i_buf_mngr_rdy) begin
                        buf_mgr_busy <= 1'b0;
                    end

                    // i_wqe_halt 是外部要求暂停一下
                    // o_wqe_halted 是该模块确认暂停信号，不会再继续推进新的 SQ WQE 处理流程。
                    // 暂停条件就是模块要先把手头这拍/这次必要动作收尾，等到安全点（空闲态、无关键写回）后，才把 o_wqe_halted 拉高
                    if (hdr_gen_cs == IDLE && i_wqe_halt && ~o_reg_wr_en) begin
                        o_wqe_halted <= 1'b1;
                    end else if (~i_wqe_halt) begin
                        o_wqe_halted <= 1'b0;  // i_wqe_halt 拉低后，o_wqe_halted 清零，流程恢复正常推进
                    end

                    if (i_osq_nacked_wr_en && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else if (i_osq_nacked_wr_en && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != 0 && ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~(rd_resp || aeth_pkt))) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else if (i_osq_nacked_wr_en && i_osq_nacked_wr_addr != o_qp_id) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else begin
                        o_osq_nacked_resp_wr_en <= 1'b0;
                    end

                    if (i_osq_nacked_wr_en_aux && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else if (i_osq_nacked_wr_en_aux && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != 0 && ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~(rd_resp || aeth_pkt))) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else if (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux != o_qp_id) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else begin
                        o_osq_nacked_resp_wr_en_aux <= 1'b0;
                    end

                    if ((i_osq_nacked_wr_en && i_osq_nacked_wr_addr == o_qp_id) || (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux == o_qp_id) && hdr_gen_cs != IDLE && hdr_gen_cs != RD_WQE &&
			  ~(hdr_gen_cs == RD_Q_INFO && (i_reg_info_val && o_qp_id != 0 && 
			  ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data)  && ~(rd_resp || aeth_pkt))))) begin
                        osq_nacked_wr_pending <= 1'b1;
                        osq_nacked_wr_id      <= i_osq_nacked_wr_en ? i_osq_nacked_wr_addr : i_osq_nacked_wr_addr_aux;
                        osq_nacked_wr_data    <= i_osq_nacked_wr_en ? i_osq_nacked_wr_data : i_osq_nacked_wr_data_aux;
                    end else if (hdr_gen_cs == IDLE) begin
                        osq_nacked_wr_pending <= 1'b0;
                    end

                    if (hdr_gen_cs == IDLE && osq_nacked_wr_pending) begin
                        o_osq_nacked_resp_wr_en_pending   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_pending <= osq_nacked_wr_id;
                        o_osq_nacked_resp_wr_data_pending <= osq_nacked_wr_data;
                    end else begin
                        o_osq_nacked_resp_wr_en_pending <= 1'b0;
                    end

                    // FSM
                    case (hdr_gen_cs)
                        IDLE: begin
                            push_to_os_buf          <= 1'b0;
                            retried                 <= 1'b0;
                            get_data_buf            <= 1'b0;
                            o_data_buf_fifo_wr_en   <= 1'b0;
                            o_hdr_gen_rdy           <= 1'b0;
                            //o_osq_nacked_resp <= i_osq_nacked;
                            wqe_pushed_in_fatal     <= 1'b0;
                            completion_data_present <= 1'b0;
                            partial_wqe             <= 1'b0;

                            if (i_reg_psn_val) begin
                                o_reg_wr_en <= 1'b0;
                            end

                            if ((i_aeth_valid) && ~o_hdr_gen_rdy && ~o_reg_wr_en) begin
                                hdr_gen_cs    <= RD_Q_INFO;
                                o_qp_id       <= i_aeth_qpid;
                                o_reg_rd_en   <= 1'b1;
                                o_reg_rd_en_s <= 1'b1;
                                aeth_pkt      <= 1'b1;
                                aeth_hdr      <= i_aeth_hdr;
                            end else if (i_rd_req_valid && ~o_hdr_gen_rdy && ~o_reg_wr_en) begin
                                hdr_gen_cs                <= RD_Q_INFO;
                                o_qp_id                   <= i_aeth_qpid;
                                o_reg_rd_en               <= 1'b1;
                                o_reg_rd_en_s             <= 1'b1;
                                rd_resp                   <= 1'b1;
                                rd_resp_start_with_middle <= i_rd_req_start_with_middle;
                                rd_resp_psn               <= i_aeth_psn;
                                aeth_hdr                  <= i_aeth_hdr;
                                dma_len                   <= i_rd_req_len;
                                rem_len                   <= i_rd_req_len;
                                local_offset              <= i_rd_req_addr;
                                first_trans               <= 1'b1;
                                wqe_opcode                <= 8'h00;
                            end else if (~i_qpm_fifo_empty && ~o_reg_wr_en && ~i_wqe_halt && ~wait_for_dma_idle) begin  // 从SQ WQE FIFO取 WQE 判断条件
                                o_qpm_wqe_pop <= 1'b1;
                                hdr_gen_cs    <= RD_WQE;
                            end else begin
                                o_reg_rd_en   <= 1'b0;
                                o_reg_rd_en_s <= 1'b0;
                            end
                        end

                        // 从 SQ WQE FIFO 中取出 WQE
                        RD_WQE: begin
                            o_qpm_wqe_pop <= 1'b0;
                            if (o_qpm_wqe_pop == 1'b0) begin  // 这里为了时序对齐
                                // ============= 手册：WQE Structure =============
                                // WRID: 工作请求 ID。每个 WQE 的唯一标识符
                                o_wqe_wrid     <= i_qpm_wqe_data[15:0];  // WQE Structure -- WRID，工作申请 ID。每个 WQE 的唯一标识符

                                // i_qpm_wqe_data[140:136]
                                o_qp_id        <= i_qpm_wqe_data[136+C_MAX_QID_WIDTH-1 : 136];

                                // LENGTH: 总长度，仅有效负载长度，不包括头部长度。(这条 WQE 要发送的全部 payload 字节数（一次任务的总量）)
                                dma_len        <= i_qpm_wqe_data[127:96];
                                rem_len        <= i_qpm_wqe_data[127:96];

                                // OPCODE: 操作码
                                //  8’h00 -- RDMA WRITE
                                //  8’h01 -- RDMA_WRITE WITH IMMDT
                                //  8’h02 -- RDMA SEND
                                //  8’h03 -- RDMA SEND WITH IMMDT
                                //  8’h04 -- RDMA READ
                                //  8’h0C -- RDMA SEND WITH INVALIDATE
                                //  All other values are reserved.(非法)
                                wqe_opcode     <= i_qpm_wqe_data[135:128];

                                // LADDR: 本地地址。操作数据缓冲区的地址
                                local_offset   <= i_qpm_wqe_data[95:32];

                                // ROFFSET: 远端偏移地址
                                remote_offset  <= i_qpm_wqe_data[223:160];

                                // RTAG: 远程标签
                                r_key          <= i_qpm_wqe_data[255:224];

                                // SDATA: RDMA SEND 数据。如果要发送的数据小于或等于 16B，则使用该字段来表示数据。
                                // ((RDMA SEND || RDMA SEND WITH IMMDT) || RDMA SEND WITH INVALIDATE) ? SDATA : 128'd0000
                                nvme_comp_data <= (i_qpm_wqe_data[129] || (i_qpm_wqe_data[131] && i_qpm_wqe_data[130])) ? i_qpm_wqe_data[383:256] : 128'd0000;

                                if (PSN_based_retransmission == 1) begin  // current
                                    // WQE OPCODE 为 8'h2/3/C && i_qpm_wqe_data[127:96] <= 16 && ~i_qpm_wqe_data[152]
                                    // WQE OPCODE 为 8'h2/3/C 意思是 RDMA SEND 类的
                                    // i_qpm_wqe_data[127:96] <= 16 是为了保证能装进 128bit 的 nvme_comp_data
                                    // 进入 qp_mgr_top 重传分支后，i_qpm_wqe_data[152] = 1
                                    completion_data_present <= (i_qpm_wqe_data[131:128] == 4'b0010 || i_qpm_wqe_data[131:128] == 4'b1100 || i_qpm_wqe_data[131:128] == 4'b0011) && ~(i_qpm_wqe_data[127:96] > 32'h00000010) && ~i_qpm_wqe_data[partial_wqe_bit_pos];
                                end else begin
                                    completion_data_present <= (i_qpm_wqe_data[131:128] == 4'b0010 || i_qpm_wqe_data[131:128] == 4'b1100 || i_qpm_wqe_data[131:128] == 4'b0011) && ~(i_qpm_wqe_data[127:96] > 32'h00000010);
                                end

                                o_reg_rd_en    <= 1'b1;
                                o_reg_rd_en_s  <= 1'b1;
                                hdr_gen_cs     <= RD_Q_INFO;
                                first_trans    <= 1'b1;
                                retried_buf_id <= i_qpm_wqe_data[17+:DATA_BUF_ID_WIDTH];
                                retried        <= i_qpm_wqe_data[16];

                                // IMMDT DATA: 要在 ImmDt 标头中发送的立即数据
                                imm_data       <= i_qpm_wqe_data[415:384];
                                if (PSN_based_retransmission == 1) begin  // current
                                    // 进入 qp_mgr_top 重传分支后，i_qpm_wqe_data[152] = 1
                                    partial_wqe           <= i_qpm_wqe_data[partial_wqe_bit_pos];
                                    partial_wqe_start_psn <= i_qpm_wqe_data[439:416];  // qp_mgr_top retransmit_wqe_bram_dout_b[47:24]
                                end else begin
                                    partial_wqe <= 1'b0;
                                end
                            end
                        end

                        // 拿到 WQE 之后，等 QP 配置读回并做发包前决策
                        RD_Q_INFO: begin
                            if (i_reg_info_val) begin  // QP 配置到了
                                if (o_qp_id == 0) begin  // SQ1
                                    hdr_gen_cs  <= HDR_GEN;
                                    payload_len <= rem_len;
                                    rem_len     <= 32'd0;
                                end //
                                
                                // ？
                                else if ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~(rd_resp || aeth_pkt)) begin
                                    //hdr_gen_cs <= IDLE;
                                    hdr_gen_cs              <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? IDLE : DROP_WQE;
                                    //push_to_os_buf <= (i_qp_fatal[o_qp_id] || i_qp_recovery) ? 1'b1 : 1'b0;
                                    o_wqe_dropped           <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b0 : 1'b1;
                                    o_wqe_dropped_qpid      <= o_qp_id;
                                    push_to_os_buf          <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b1 : 1'b0;
                                    o_reg_rd_en             <= 1'b0;
                                    first_trans             <= 1'b0;
                                    completion_data_present <= 1'b0;
                                    //o_osq_nacked_resp <= i_osq_nacked;
                                    //wqe_pushed_in_fatal <= i_qp_fatal[o_qp_id] || i_qp_recovery;
                                    wqe_pushed_in_fatal     <= (qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1);
                                    //end else if ((i_qp_fatal[o_qp_id] || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) && ((rd_resp || aeth_pkt) && ~(aeth_hdr[30:29] == 2'b11 && |aeth_hdr[26:24]))) begin
                                end //
                                
                                // ？
                                else if ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) && (aeth_pkt && ~(aeth_hdr[30:29] == 2'b11 && |aeth_hdr[26:24]))) begin
                                    hdr_gen_cs                <= IDLE;
                                    o_reg_rd_en               <= 1'b0;
                                    first_trans               <= 1'b0;
                                    o_hdr_gen_rdy             <= 1'b1;
                                    aeth_pkt                  <= 1'b0;
                                    rd_resp                   <= 1'b0;
                                    rd_resp_start_with_middle <= 2'b00;
                                end //
                                
                                // (非法操作码，不在 WQE 白名单范围内) && (aeth_pkt == 1'b0)
                                else if (~(wqe_opcode[7:1] == 7'h00 || wqe_opcode[7:1] == 7'h01 || wqe_opcode == 8'h04 || wqe_opcode == 8'h0C || aeth_pkt)) begin
                                    hdr_gen_cs       <= ILL_OPCODE;
                                    o_wqe_fatal_qpid <= o_qp_id;
                                end //
                                
                                // 其余大部分情况
                                else begin
                                    hdr_gen_cs <= WAIT0;
                                end

                                // 提取 QP && RNIC 配置信息
                                exp_ack              <= i_reg_exp_ack;
                                o_reg_rd_en_s        <= 1'b0;
                                dest_qpid            <= i_reg_dest_qpid;
                                mac_dest_addr        <= i_reg_mac_dest_addr;
                                ip_dest_addr         <= i_reg_ip_dest_addr;
                                ip_ver               <= i_reg_ip_ver;
                                o_reg_sq_psn         <= i_reg_sq_psn;
                                o_wqe_send_first_psn <= partial_wqe ? partial_wqe_start_psn : i_reg_sq_psn;
                                o_reg_sq_msn         <= i_reg_sq_msn + 1'b1;
                                reg_qp_to            <= i_reg_qp_to;
                            end

                            wqe_os_fifo_full <= (os_num_vld_entries >= OS_TH_ENTRIES);

                            // 解析 pmtu
                            case (i_reg_pmtu)
                                3'b000: begin
                                    max_pmtu <= 13'h0100;  // 256
                                end
                                3'b001: begin
                                    max_pmtu <= 13'h0200;  // 512
                                end
                                3'b010: begin
                                    max_pmtu <= 13'h0400;  // 1024
                                end
                                3'b011: begin
                                    max_pmtu <= 13'h0800;  // 2048
                                end
                                3'b100: begin
                                    max_pmtu <= 13'h1000;  // 4096
                                end
                                default: begin
                                    max_pmtu <= 13'h100;  // 256
                                end
                            endcase
                        end

                        // 发包前的分片与头部准备
                        WAIT0: begin
                            get_data_buf          <= 1'b0;
                            o_data_buf_fifo_wr_en <= 1'b0;
                            push_to_os_buf        <= 1'b0;

                            // HDR_STO 状态可能会跳到 WAIT0 状态
                            // 上一包的 PSN/MSN 等状态要写回，因此这里要等待写回完成有效
                            if (i_reg_psn_val) begin
                                o_reg_wr_en <= 1'b0;
                            end

                            if (ip_ver && ~o_reg_wr_en) begin  // ipv6
                                hdr_gen_cs <= HDR_GEN;
                            end else if (~ip_ver && ~o_reg_wr_en) begin  // ipv4
                                hdr_gen_cs          <= IP_CHKSUM;
                                ipv4_identification <= ipv4_identification + 1'b1;
                                ipv4_len_cal        <= 1'b0;
                            end

                            if (~o_reg_wr_en) begin
                                if (aeth_pkt) begin
                                    payload_len  <= 32'd0;
                                    rem_len      <= 32'd0;
                                    reth_present <= 1'b0;
                                end else if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (rd_resp || wqe_opcode[7:1] == 7'h00)) begin
                                    payload_len          <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                                    rem_len              <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                                    reth_present         <= first_trans && ~rd_resp && ~partial_wqe;
                                    rd_resp_aeth_present <= first_trans && rd_resp && ~rd_resp_start_with_middle[0];
                                end else if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (wqe_opcode[7:1] == 7'h01 || wqe_opcode == 8'h0C)) begin
                                    payload_len  <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                                    rem_len      <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                                    reth_present <= 1'b0;
                                end else if (wqe_opcode == 8'h04) begin
                                    payload_len  <= 32'd0;
                                    rem_len      <= rem_len;
                                    reth_present <= 1'b1;
                                end else begin
                                    payload_len          <= rem_len;
                                    rem_len              <= 32'd0;
                                    reth_present         <= 1'b0;
                                    rd_resp_aeth_present <= rd_resp;
                                end
                            end
                        end

                        // 计算 IPv4 头校验的中间变量
                        // 这里都是中间变量，最后头校验 = {~ip_hdr_chksum[7:0], ~ip_hdr_chksum[15:8]}
                        IP_CHKSUM: begin
                            if (~ipv4_len_cal) begin
                                ipv4_len_cal           <= 1'b1;
                                ip_hdr_chksum_add_pipe <= 16'h4501 + ipv4_identification[15:0] + 16'h4000 + ip_dest_addr[31:16] + ip_dest_addr[15:0];
                                if (((first_trans && ~|rem_len) || (~first_trans && ~|rem_len)) && (wqe_opcode == 8'h0C || wqe_opcode == 8'h03 || wqe_opcode == 8'h01) && (!aeth_pkt)) begin
                                    ipv4_len <= 8'h2C + payload_len + {reth_present, 1'b0, (aeth_pkt || rd_resp_aeth_present), 2'b00} + 3'd4;
                                end else begin
                                    ipv4_len <= 8'h2C + payload_len + {reth_present, 1'b0, (aeth_pkt || rd_resp_aeth_present), 2'b00};
                                end
                            end else if (~chksum_cal_start) begin
                                ip_hdr_chksum    <= ip_hdr_chksum_add_pipe + ipv4_len[15:0] + {i_reg_ttl, 8'h11} + i_reg_ipv4_src_addr[31:16] + i_reg_ipv4_src_addr[15:0];
                                chksum_cal_start <= 1'b1;
                            end else begin
                                if (|ip_hdr_chksum[19:16]) begin
                                    ip_hdr_chksum <= ip_hdr_chksum[15:0] + ip_hdr_chksum[19:16];
                                end else begin
                                    chksum_cal_start <= 1'b0;
                                    hdr_gen_cs       <= HDR_GEN;
                                end
                            end
                        end

                        // 头部生成
                        HDR_GEN: begin
                            // wqe_opcode[7:1] != 7'h00，即不为 RDMA WRITE 和 RDMA_WRITE WITH IMMDT
                            if (~no_data_buf_left && ~buf_mgr_busy && ((wqe_opcode[7:1] != 7'h00 || rd_resp) || (~retried || i_data_buf_fifo_empty))) begin
                                wqe_opcode_err <= 1'b0;

                                eth_hdr <= {
                                    (ip_ver ? 16'hDD86 : 16'h0008),
                                    i_reg_mac_src_addr[7:0],
                                    i_reg_mac_src_addr[15:8],
                                    i_reg_mac_src_addr[23:16],
                                    i_reg_mac_src_addr[31:24],
                                    i_reg_mac_src_addr[39:32],
                                    i_reg_mac_src_addr[47:40],
                                    mac_dest_addr[7:0],
                                    mac_dest_addr[15:8],
                                    mac_dest_addr[23:16],
                                    mac_dest_addr[31:24],
                                    mac_dest_addr[39:32],
                                    mac_dest_addr[47:40]
                                };

                                if (ip_ver) begin  // IPv6
                                    ip_hdr <= {
                                        ip_dest_addr[7:0],
                                        ip_dest_addr[15:8],
                                        ip_dest_addr[23:16],
                                        ip_dest_addr[31:24],
                                        ip_dest_addr[39:32],
                                        ip_dest_addr[47:40],
                                        ip_dest_addr[55:48],
                                        ip_dest_addr[63:56],
                                        ip_dest_addr[71:64],
                                        ip_dest_addr[79:72],
                                        ip_dest_addr[87:80],
                                        ip_dest_addr[95:88],
                                        ip_dest_addr[103:96],
                                        ip_dest_addr[111:104],
                                        ip_dest_addr[119:112],
                                        ip_dest_addr[127:120],
                                        i_reg_ipv6_src_addr[7:0],
                                        i_reg_ipv6_src_addr[15:8],
                                        i_reg_ipv6_src_addr[23:16],
                                        i_reg_ipv6_src_addr[31:24],
                                        i_reg_ipv6_src_addr[39:32],
                                        i_reg_ipv6_src_addr[47:40],
                                        i_reg_ipv6_src_addr[55:48],
                                        i_reg_ipv6_src_addr[63:56],
                                        i_reg_ipv6_src_addr[71:64],
                                        i_reg_ipv6_src_addr[79:72],
                                        i_reg_ipv6_src_addr[87:80],
                                        i_reg_ipv6_src_addr[95:88],
                                        i_reg_ipv6_src_addr[103:96],
                                        i_reg_ipv6_src_addr[111:104],
                                        i_reg_ipv6_src_addr[119:112],
                                        i_reg_ipv6_src_addr[127:120],
                                        i_reg_ttl,
                                        8'h11,
                                        ipv6_len[7:0],
                                        ipv6_len[15:8],
                                        q_id[7:0],
                                        q_id[15:8],
                                        {i_reg_dscp[1:0], 2'b01, 4'h0},
                                        {4'h6, i_reg_dscp[5:2]}
                                    };
                                end else begin  // IPv4
                                    ip_hdr[159:96] <= {ip_dest_addr[7:0], ip_dest_addr[15:8], ip_dest_addr[23:16], ip_dest_addr[31:24], i_reg_ipv4_src_addr[7:0], i_reg_ipv4_src_addr[15:8], i_reg_ipv4_src_addr[23:16], i_reg_ipv4_src_addr[31:24]};
                                    ip_hdr[95:80]  <= {~ip_hdr_chksum[7:0], ~ip_hdr_chksum[15:8]};  // IPv4 头校验
                                    ip_hdr[79:0]   <= {8'h11, i_reg_ttl, 16'h0040, ipv4_identification[7:0], ipv4_identification[15:8], ipv4_len[7:0], ipv4_len[15:8], 16'h0145};
                                end

                                udp_hdr <= {16'h0000, ipv6_len[7:0], ipv6_len[15:8], 16'hB712, i_reg_udp_src_port[7:0], i_reg_udp_src_port[15:8]};  //Need to check that UDP destination port is decimal or hexa, considering it as decimal 4791 for now

                                if (aeth_pkt) begin
                                    bth_hdr[95:72] <= {i_aeth_psn[7:0], i_aeth_psn[15:8], i_aeth_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end else if (rd_resp) begin
                                    bth_hdr[95:72] <= {rd_resp_psn[7:0], rd_resp_psn[15:8], rd_resp_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end else begin
                                    //bth_hdr[95:32] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16], (exp_ack || wqe_os_fifo_full), 7'h00,  dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                    bth_hdr[95:72] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end

                                bth_hdr[31:16] <= {i_reg_pkey[7:0], i_reg_pkey[15:8]};
                                bth_hdr[14:8]  <= {1'b0, 2'b00, 4'h0};

                                //SE Bit chnage if((|rem_len && (wqe_opcode == 8'h00 || wqe_opcode == 8'h02)) || aeth_pkt) begin
                                if (~|rem_len && (wqe_opcode[7:1] == 7'h01 || wqe_opcode == 8'h0C)) begin
                                    bth_hdr[15] <= 1'b1;
                                end else begin
                                    bth_hdr[15] <= 1'b0;
                                end

                                if (aeth_pkt) begin
                                    bth_hdr[7:0] <= 8'h11;
                                    bth_hdr[71]  <= 1'b0;
                                end else if (rd_resp) begin
                                    o_out_rd_resp_pkt_cnt <= o_out_rd_resp_pkt_cnt + 1'b1;
                                    bth_hdr[71]           <= 1'b0;
                                    push_to_os_buf        <= 1'b0;
                                    if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= ((rd_resp_start_with_middle == 2'b01) ? 8'h0E : 8'h0D);  //RD_RESP_MIDDLE/RD_RESP_FIRST
                                        aeth_hdr[23:0] <= aeth_hdr[23:0] - 24'd1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0] <= ((rd_resp_start_with_middle == 2'b10) ? 8'h0F : 8'h10);  //RD_RESP_LAST/RD_RESP_FIRST
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0] <= 8'h0E;  //RD_RESP_MIDDLE
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h0F;  //RD_RESP_LAST
                                        aeth_hdr[23:0] <= aeth_hdr[23:0] + 24'd1;
                                    end
                                end else if (wqe_opcode[7:1] == 7'h00) begin  // RDMA WRITE || RDMA_WRITE WITH IMMDT
                                    o_out_rw_pkt_cnt <= o_out_rw_pkt_cnt + 1'b1;
                                    bth_hdr[71]      <= exp_ack || (~|rem_len);
                                    if (PSN_based_retransmission == 1 && partial_wqe == 1'b1) begin
                                        if (|rem_len) begin
                                            bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                            push_to_os_buf <= first_trans;
                                        end else begin
                                            bth_hdr[7:0]     <= {7'h04, wqe_opcode[0]};  //WRITE_LAST, or WRITE_LAST_IMMDT
                                            push_to_os_buf   <= first_trans;
                                            imm_data_present <= wqe_opcode[0];
                                        end
                                    end else if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h06;  //WRITE_FIRST
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]     <= {7'h05, wqe_opcode[0]};  //WRITE_ONLY, or WRITE_ONLY_IMMDT
                                        push_to_os_buf   <= 1'b1;
                                        imm_data_present <= wqe_opcode[0];
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]     <= {7'h04, wqe_opcode[0]};  //WRITE_LAST, or WRITE_LAST_IMMDT
                                        push_to_os_buf   <= 1'b0;
                                        imm_data_present <= wqe_opcode[0];
                                    end
                                end else if (wqe_opcode == 8'h04) begin  // RDMA READ
                                    o_out_rw_pkt_cnt <= o_out_rw_pkt_cnt + 1'b1;
                                    bth_hdr[7:0]     <= 8'h0C;  //RDMA_READ
                                    bth_hdr[71]      <= 1'b1;
                                    push_to_os_buf   <= 1'b1;
                                end else if (wqe_opcode[7:1] == 7'h01 || wqe_opcode == 8'h0C) begin  // RDMA SEND || RDMA SEND WITH IMMDT || RDMA SEND WITH INVALIDATE
                                    o_out_send_pkt_cnt <= o_out_send_pkt_cnt + ((q_id != 16'h0000) ? 1'b1 : 1'b0);
                                    bth_hdr[71]        <= exp_ack || (~|rem_len);
                                    if (PSN_based_retransmission == 1 && partial_wqe == 1'b1) begin
                                        if (|rem_len) begin
                                            bth_hdr[7:0]   <= 8'h01;  //SEND_MIDDLE
                                            push_to_os_buf <= first_trans;
                                        end else if (wqe_opcode == 8'h0C) begin
                                            bth_hdr[7:0]   <= 8'b0001_0110;  // SEND_LAST with Invalidate
                                            push_to_os_buf <= first_trans;
                                        end else begin
                                            bth_hdr[7:0]     <= {7'h01, wqe_opcode[0]};  //SEND_LAST, SEND_LAST_IMMDT
                                            push_to_os_buf   <= first_trans;
                                            imm_data_present <= wqe_opcode[0];
                                        end
                                    end else if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h00;  //SEND_FIRST
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len && wqe_opcode == 8'h0C) begin
                                        bth_hdr[7:0]   <= 8'b0001_0111;  //SEND_ONLY with Invalidate
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]     <= {7'h02, wqe_opcode[0]};  //SEND_ONLY, or SEND_ONLY_IMMDT
                                        push_to_os_buf   <= 1'b1;
                                        imm_data_present <= wqe_opcode[0];
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h01;  //SEND_MIDDLE
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len && wqe_opcode == 8'h0C) begin
                                        bth_hdr[7:0]   <= 8'b0001_0110;  // SEND_LAST with Invalidate
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]     <= {7'h01, wqe_opcode[0]};  //SEND_LAST, SEND_LAST_IMMDT
                                        push_to_os_buf   <= 1'b0;
                                        imm_data_present <= wqe_opcode[0];
                                    end
                                end else begin
                                    bth_hdr[7:0]   <= 8'h04;
                                    bth_hdr[71]    <= 1'b0;
                                    push_to_os_buf <= 1'b0;
                                end

                                if (rd_resp) begin
                                    rd_resp_psn <= rd_resp_psn + 1'b1;
                                end

                                if (wqe_opcode == 8'h04) begin  // RDMA READ
                                    case (max_pmtu[12:8])
                                        5'h01: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:8] + |(rem_len[7:0]);
                                        end
                                        5'h02: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:9] + |(rem_len[8:0]);
                                        end
                                        5'h04: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:10] + |(rem_len[9:0]);
                                        end
                                        5'h08: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:11] + |(rem_len[10:0]);
                                        end
                                        5'h10: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:12] + |(rem_len[11:0]);
                                        end
                                        default: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:8] + |(rem_len[8:0]);
                                        end
                                    endcase
                                end else begin
                                    o_reg_sq_psn <= o_reg_sq_psn + (aeth_pkt ? 1'b0 : 1'b1);
                                end
                                o_out_ack_pkt_cnt <= o_out_ack_pkt_cnt + (aeth_pkt ? 1'b1 : 1'b0);
                                o_out_mad_pkt_cnt <= o_out_mad_pkt_cnt + (q_id == 16'h0000 ? 1'b1 : 1'b0);
                                local_offset <= local_offset + payload_len;
                                remote_offset <= remote_offset + payload_len;
                                reth_hdr <= {
                                    dma_len[7:0],
                                    dma_len[15:8],
                                    dma_len[23:16],
                                    dma_len[31:24],
                                    r_key[7:0],
                                    r_key[15:8],
                                    r_key[23:16],
                                    r_key[31:24],
                                    remote_offset[7:0],
                                    remote_offset[15:8],
                                    remote_offset[23:16],
                                    remote_offset[31:24],
                                    remote_offset[39:32],
                                    remote_offset[47:40],
                                    remote_offset[55:48],
                                    remote_offset[63:56]
                                };

                                // {o_hdr_data[1023:1008], o_hdr_data[1007:944], o_hdr_data[943], o_hdr_data[942:928]}
                                o_hdr_data[1023:928] <= {payload_len, local_offset, completion_data_present, q_id[14:0]};
                                o_hdr_gen_rdy <= aeth_pkt || (rd_resp && ~|rem_len);

                                if (rd_resp || aeth_pkt || (~(i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)))) begin
                                    hdr_gen_cs <= HDR_STO;
                                end else begin
                                    hdr_gen_cs          <= DROP_WQE;  // 上报“这条 WQE 要被撤销/不再继续推进”
                                    o_wqe_dropped       <= first_trans;
                                    o_wqe_dropped_qpid  <= o_qp_id;
                                    ipv4_identification <= ipv4_identification - (ip_ver ? 1'b0 : 1'b1);
                                end
                            end
                        end

                        HDR_STO: begin
                            //get_data_buf <= 1'b0;
                            o_data_buf_fifo_data <= {data_buf_addr[C_M_AXI_ADDR_WIDTH-1:2], partial_wqe, (retried || (q_id == 16'h0000))};
                            o_hdr_gen_rdy        <= 1'b0;
                            if (~wqe_push_pending) begin
                                o_hdr_data[111:0] <= eth_hdr;
                                if (ip_ver) begin
                                    o_hdr_data[431:112] <= ip_hdr;
                                    o_hdr_data[495:432] <= udp_hdr;
                                    o_hdr_data[591:496] <= bth_hdr;
                                    if (reth_present) begin
                                        o_hdr_data[719:592] <= reth_hdr;
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h5E;  //94B header
                                            o_hdr_data[751:720] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                        end else begin
                                            o_hdr_len <= 8'h5A;  //90B header
                                        end
                                    end else if (aeth_pkt || rd_resp_aeth_present) begin
                                        o_hdr_data[623:592] <= {aeth_hdr[7:0], aeth_hdr[15:8], aeth_hdr[23:16], aeth_hdr[31:24]};
                                        o_hdr_len           <= 8'h4E;  //78B header
                                    end else if (bth_hdr[7:1] != 7'b0001_011 && completion_data_present) begin
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h5E;  //94B header
                                            o_hdr_data[623:592] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                            o_hdr_data[751:624] <= nvme_comp_data;
                                        end else begin
                                            o_hdr_data[719:592] <= nvme_comp_data;
                                            o_hdr_len           <= 8'h5A;  //90B header
                                        end
                                    end else if (bth_hdr[7:1] == 7'b0001_011) begin  // Send Invalidate only or send last  Invaliate 
                                        o_hdr_len           <= 8'h4E;  // 78B header
                                        o_hdr_data[623:592] <= {r_key[7:0], r_key[15:8], r_key[23:16], r_key[31:24]};  // 32  biti
                                        if (completion_data_present) begin
                                            o_hdr_data[751:624] <= nvme_comp_data;
                                            o_hdr_len           <= 8'h5E;  // 94B header
                                        end else o_hdr_data[751:624] <= 'b0;
                                    end else begin
                                        o_hdr_data[751:624] <= 128'd0000;
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h4E;  //78B header
                                            o_hdr_data[623:592] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                        end else begin
                                            o_hdr_len           <= 8'h4A;  //74B header
                                            o_hdr_data[623:592] <= 32'd0;
                                        end
                                    end
                                end else begin
                                    o_hdr_data[271:112] <= ip_hdr[159:0];
                                    o_hdr_data[335:272] <= udp_hdr;
                                    o_hdr_data[431:336] <= bth_hdr;
                                    if (reth_present) begin
                                        o_hdr_data[559:432] <= reth_hdr;
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h4A;  //74B header
                                            o_hdr_data[591:560] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                        end else begin
                                            o_hdr_len           <= 8'h46;  //70B header
                                            o_hdr_data[591:560] <= 32'd0;
                                        end
                                    end else if (aeth_pkt || rd_resp_aeth_present) begin
                                        o_hdr_data[463:432] <= {aeth_hdr[7:0], aeth_hdr[15:8], aeth_hdr[23:16], aeth_hdr[31:24]};
                                        o_hdr_len           <= 8'h3A;  //58B header
                                    end else if (bth_hdr[7:1] != 7'b0001_011 && completion_data_present) begin
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h4A;  //74B header
                                            o_hdr_data[463:432] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                            o_hdr_data[591:464] <= nvme_comp_data;
                                        end else begin
                                            o_hdr_data[559:432] <= nvme_comp_data;
                                            o_hdr_len           <= 8'h46;  //70B header
                                        end
                                    end else if (bth_hdr[7:1] == 7'b0001_011) begin  // Send Invalidate only or send last  Invaliate 
                                        o_hdr_len           <= 8'h3A;  // 58B header
                                        o_hdr_data[463:432] <= {r_key[7:0], r_key[15:8], r_key[23:16], r_key[31:24]};  // 32 bit
                                        if (completion_data_present) begin
                                            o_hdr_data[591:464] <= nvme_comp_data;
                                            o_hdr_len           <= 8'h4A;  //74B header
                                        end else o_hdr_data[591:464] <= 'b0;
                                    end else begin
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h3A;  //58B header
                                            o_hdr_data[463:432] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                        end else begin
                                            o_hdr_len           <= 8'h36;  //54B header
                                            o_hdr_data[463:432] <= 32'd0;
                                        end
                                        o_hdr_data[591:464] <= 128'd0000;
                                    end
                                end
                                if (wqe_opcode[7:1] == 7'h00 && first_trans && ~aeth_pkt && ~rd_resp) begin
                                    o_data_buf_fifo_wr_en <= 1'b1;
                                    get_data_buf          <= ~retried;
                                end else begin
                                    o_data_buf_fifo_wr_en <= 1'b0;
                                    get_data_buf          <= 1'b0;
                                end
                                push_to_os_buf   <= 1'b0;
                                first_trans      <= 1'b0;
                                o_reg_wr_en      <= ~(aeth_pkt || rd_resp);  // 既不是aeth_pkt也不是rd_resp
                                imm_data_present <= 1'b0;
                                if ((wqe_opcode[7:1] == 7'h00 || wqe_opcode[7:1] == 7'h01 || wqe_opcode == 8'h0C || rd_resp) && |rem_len) begin
                                    hdr_gen_cs <= WAIT0;
                                end else begin
                                    hdr_gen_cs  <= IDLE;
                                    o_reg_rd_en <= 1'b0;
                                    rd_resp     <= 1'b0;
                                end
                                aeth_pkt                  <= 1'b0;
                                reth_present              <= 1'b0;
                                rd_resp_aeth_present      <= 1'b0;
                                rd_resp_start_with_middle <= 2'b00;
                            end
                        end

                        ILL_OPCODE: begin
                            o_hdr_gen_rdy <= 1'b0;
                            if (~o_wqe_fatal_qp_vld) begin
                                //o_hdr_data[1023:512] <= 512'd0;
                                //o_hdr_data[511:0] <= {128'd0,nvme_comp_data,16'h0000,q_id,r_key,remote_offset,wqe_opcode,dma_len,local_offset,o_wqe_wrid};
                                //o_hdr_len <= 8'h00; 
                                hdr_gen_cs                <= IDLE;
                                o_reg_rd_en               <= 1'b0;
                                o_reg_wr_en               <= ~aeth_pkt;
                                aeth_pkt                  <= 1'b0;
                                rd_resp_start_with_middle <= 2'b00;
                                wqe_opcode_err            <= 1'b1;
                                push_to_os_buf            <= 1'b1;
                            end
                        end

                        // 上报“这条 WQE 要被撤销/不再继续推进”
                        DROP_WQE: begin
                            hdr_gen_cs       <= IDLE;
                            o_wqe_dropped    <= 1'b0;
                            imm_data_present <= 1'b0;
                        end
                        default: begin
                        end
                    endcase
                end
            end

        end
        //   else
        //     begin
        //	always @ (posedge core_clk)
        //	  o_out_rd_resp_pkt_cnt <= 0;
        //     end
    endgenerate  // end of current: C_EN_RECV_RD_WR == 1 && NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 1



    //////////////////////////////////////////////////////////////////////////////SEND_INVALIDATE

    generate
        if (C_EN_RECV_RD_WR == 0 && NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 1) begin : NO_RD_RESP

            reg [31:0] imm_data;  // WQE Structure -- IMMDT DATA。在 ImmDt 标头中发送的即时数据
            reg        imm_data_present;

            assign ipv6_len =  (((first_trans && ~|rem_len) || (~first_trans && ~|rem_len)) && (wqe_opcode == 8'h0C || wqe_opcode == 8'h03 || wqe_opcode == 8'h01) && (!aeth_pkt)) ?  (8'h18 + payload_len + {reth_present,1'b0,aeth_pkt,2'b00} + 3'd4) : (8'h18 + payload_len + {reth_present,1'b0,aeth_pkt,2'b00});
            //assign ipv4_len =   (((first_trans && ~|rem_len) || (~first_trans && ~|rem_len)) && (wqe_opcode == 8'h0C || wqe_opcode == 8'h03 || wqe_opcode == 8'h01) && (!aeth_pkt)) ? (8'h2C + payload_len + {reth_present,1'b0,aeth_pkt,2'b00} + 3'd4) : (8'h2C + payload_len + {reth_present,1'b0,aeth_pkt,2'b00});
            always @(posedge core_clk) begin
                if (core_rst) begin
                    eth_hdr                           <= 'd0;
                    ip_hdr                            <= 'd0;
                    udp_hdr                           <= 'd0;
                    bth_hdr                           <= 'd0;
                    reth_hdr                          <= 'd0;
                    aeth_hdr                          <= 'd0;
                    o_hdr_data                        <= 'd0;
                    o_hdr_valid                       <= 1'b0;
                    o_hdr_ip_ver                      <= 1'b0;
                    o_hdr_len                         <= 8'h00;
                    o_qpm_wqe_pop                     <= 1'b0;
                    o_qp_id                           <= {C_MAX_QID_WIDTH{1'b0}};
                    o_reg_rd_en                       <= 1'b0;
                    o_reg_rd_en_s                     <= 1'b0;
                    hdr_gen_cs                        <= IDLE;
                    ip_hdr_chksum                     <= 20'd0;
                    rem_len                           <= 32'd0;
                    dma_len                           <= 32'd0;
                    first_trans                       <= 1'b0;
                    payload_len                       <= 16'h0000;
                    reth_present                      <= 1'b0;
                    remote_offset                     <= 64'd00;
                    local_offset                      <= 64'd00;
                    aeth_pkt                          <= 1'b0;
                    wqe_opcode                        <= 8'h00;
                    r_key                             <= 32'd0;
                    chksum_cal_start                  <= 1'b0;
                    nvme_comp_data                    <= 128'd0000;
                    completion_data_present           <= 1'b0;
                    o_reg_wr_en                       <= 1'b0;
                    wqe_opcode_err                    <= 1'b0;
                    err_opcode                        <= 8'h00;
                    dest_qpid                         <= 24'd0;
                    o_hdr_gen_rdy                     <= 1'b0;
                    o_reg_sq_psn                      <= 24'd0;
                    o_wqe_send_first_psn              <= 24'd0;
                    o_reg_sq_msn                      <= 24'd0;
                    o_wqe_wrid                        <= 16'h0000;
                    mac_dest_addr                     <= 'd0;
                    ip_dest_addr                      <= 'd0;
                    ip_ver                            <= 1'b0;
                    exp_ack                           <= 1'b0;
                    o_out_send_pkt_cnt                <= 16'h0000;
                    o_out_rw_pkt_cnt                  <= 16'h0000;
                    o_out_ack_pkt_cnt                 <= 16'h0000;
                    o_out_mad_pkt_cnt                 <= 16'h0000;
                    o_wqe_halted                      <= 1'b0;
                    wqe_os_fifo_full                  <= 1'b0;
                    ipv4_identification               <= 16'h0000;
                    retried                           <= 1'b0;
                    get_data_buf                      <= 1'b0;
                    o_data_buf_fifo_data              <= 'd0;
                    o_data_buf_fifo_wr_en             <= 1'b0;
                    //o_osq_nacked_resp <= 'd0;
                    push_to_os_buf                    <= 1'b0;
                    buf_mgr_busy                      <= 1'b0;
                    reg_qp_to                         <= 5'h00;
                    wait_for_dma_idle                 <= 1'b0;
                    //qp_fatal_detected<= 'h0;
                    retried_buf_id                    <= 'h0;
                    wqe_pushed_in_fatal               <= 1'b0;
                    o_wqe_fatal_qpid                  <= {C_MAX_QID_WIDTH{1'b0}};
                    o_wqe_fatal_qp_vld                <= 1'b0;
                    o_wqe_dropped                     <= 1'b0;
                    o_wqe_dropped_qpid                <= {C_MAX_QID_WIDTH{1'b0}};
                    imm_data                          <= 32'd0;
                    imm_data_present                  <= 1'b0;
                    ipv4_len                          <= 16'd0;
                    ip_hdr_chksum_add_pipe            <= 17'd0;
                    ipv4_len_cal                      <= 1'b0;
                    partial_wqe                       <= 1'b0;
                    partial_wqe_start_psn             <= 24'd0;
                    qp_fatal_detected_wr_en           <= 1'b0;
                    qp_fatal_detected_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_fatal_detected_wr_data         <= 1'b0;
                    qp_fatal_detected_wr_en_aux       <= 1'b0;
                    qp_fatal_detected_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_fatal_detected_wr_data_aux     <= 1'b0;
                    qp_wqe_opcode_fatal_wr_en         <= 1'b0;
                    qp_wqe_opcode_fatal_wr_addr       <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_wqe_opcode_fatal_wr_data       <= 1'b0;
                    qp_wqe_opcode_fatal_wr_en_aux     <= 1'b0;
                    qp_wqe_opcode_fatal_wr_addr_aux   <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_wqe_opcode_fatal_wr_data_aux   <= 1'b0;
                    o_osq_nacked_resp_wr_en           <= 1'b0;
                    o_osq_nacked_resp_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data         <= 1'b0;
                    o_osq_nacked_resp_wr_en_aux       <= 1'b0;
                    o_osq_nacked_resp_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data_aux     <= 1'b0;
                    o_osq_nacked_resp_wr_en_pending   <= 1'b0;
                    o_osq_nacked_resp_wr_addr_pending <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data_pending <= 1'b0;
                    osq_nacked_wr_pending             <= 1'b0;
                    osq_nacked_wr_id                  <= {C_MAX_QID_WIDTH{1'b0}};
                    osq_nacked_wr_data                <= 1'b0;
                end else begin
                    //qp_fatal_detected<= (i_qp_fatal & (~i_osq_nacked));
                    /* if (i_qp_fatal_wr_en && i_osq_nacked_wr_en && i_qp_fatal_wr_addr == i_osq_nacked_wr_addr) begin
				qp_fatal_detected_wr_en <= 1'b1;
				qp_fatal_detected_wr_en_aux <= 1'b0;
				qp_fatal_detected_wr_addr <= i_qp_fatal_wr_addr;
				qp_fatal_detected_wr_data <= i_qp_fatal_wr_data & ~i_osq_nacked_wr_data;
			  end else begin
				if (i_qp_fatal_wr_en) begin
					qp_fatal_detected_wr_en <= 1'b1;
					qp_fatal_detected_wr_addr <= i_qp_fatal_wr_addr;
					qp_fatal_detected_wr_data <= i_qp_fatal_wr_data & ~osq_nacked_rd_data_0;
				end else begin
					qp_fatal_detected_wr_en <= 1'b0;
				end
				
				if (i_osq_nacked_wr_en) begin
					qp_fatal_detected_wr_en_aux <= 1'b1;
					qp_fatal_detected_wr_addr_aux <= i_osq_nacked_wr_addr;
					qp_fatal_detected_wr_data_aux <= qp_fatal_rd_data_0 & ~i_osq_nacked_wr_data;
				end else begin
					qp_fatal_detected_wr_en_aux <= 1'b0;
				end
			  end */
                    if (~wqe_push_pending && (hdr_gen_cs == HDR_STO  /*|| hdr_gen_cs == ILL_OPCODE*/)) begin
                        o_hdr_valid  <= 1'b1;
                        o_hdr_ip_ver <= ip_ver;
                    end else if (i_buf_mngr_rdy) begin
                        o_hdr_valid <= 1'b0;
                    end
                    if ((hdr_gen_cs == ILL_OPCODE) && ~o_wqe_fatal_qp_vld) begin
                        o_wqe_fatal_qp_vld <= 1'b1;
                    end else if (i_wqe_fatal_qp_rdy) begin
                        o_wqe_fatal_qp_vld <= 1'b0;
                    end
                    //for (i = 0; i < C_MAX_QP; i = i+1) begin
                    //if(o_wqe_fatal_qpid == i && o_wqe_fatal_qp_vld) begin
                    //           qp_wqe_opcode_fatal[i] <= 1'b1;
                    //end else if (i_qp_fatal[i]) begin
                    //           qp_wqe_opcode_fatal[i] <= 1'b0;
                    //end
                    //end

                    if (o_wqe_fatal_qp_vld && i_qp_fatal_wr_en && o_wqe_fatal_qpid == i_qp_fatal_wr_addr) begin
                        qp_wqe_opcode_fatal_wr_en     <= 1'b1;
                        qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                        qp_wqe_opcode_fatal_wr_addr   <= o_wqe_fatal_qpid;
                        qp_wqe_opcode_fatal_wr_data   <= 1'b1;
                    end else begin
                        if (o_wqe_fatal_qp_vld) begin
                            qp_wqe_opcode_fatal_wr_en   <= 1'b1;
                            qp_wqe_opcode_fatal_wr_addr <= o_wqe_fatal_qpid;
                            qp_wqe_opcode_fatal_wr_data <= 1'b1;
                        end else begin
                            qp_wqe_opcode_fatal_wr_en <= 1'b0;
                        end

                        if (i_qp_fatal_wr_en && i_qp_fatal_wr_data) begin
                            qp_wqe_opcode_fatal_wr_en_aux   <= 1'b1;
                            qp_wqe_opcode_fatal_wr_addr_aux <= i_qp_fatal_wr_addr;
                            qp_wqe_opcode_fatal_wr_data_aux <= 1'b0;
                        end else begin
                            qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                        end
                    end

                    if (o_hdr_valid && ~i_buf_mngr_rdy) begin
                        buf_mgr_busy <= 1'b1;
                    end else if (i_buf_mngr_rdy) begin
                        buf_mgr_busy <= 1'b0;
                    end
                    if (hdr_gen_cs == IDLE && i_wqe_halt && ~o_reg_wr_en) begin
                        o_wqe_halted <= 1'b1;
                    end else if (~i_wqe_halt) begin
                        o_wqe_halted <= 1'b0;
                    end
                    //if( |((i_qp_fatal & ~i_osq_nacked )& ~qp_fatal_detected)) begin
                    /* if (i_qp_fatal_wr_en && i_osq_nacked_wr_en && i_qp_fatal_wr_addr == i_osq_nacked_wr_addr && (i_qp_fatal_wr_data && ~i_osq_nacked_wr_data) && ~qp_fatal_detected_rd_data_0) begin
				 wait_for_dma_idle <= 1'b1;
			  end else if ((i_qp_fatal_wr_en && (i_qp_fatal_wr_data && ~osq_nacked_rd_data_0) && ~qp_fatal_detected_rd_data_0) || (i_osq_nacked_wr_en && (qp_fatal_rd_data_0 && ~i_osq_nacked_wr_data) && ~qp_fatal_detected_rd_data_1)) begin	 
				 wait_for_dma_idle <= 1'b1;
              end else if(dma_in_idle && ~o_hdr_valid && (hdr_gen_cs == IDLE)) begin
				wait_for_dma_idle <= 1'b0;
              end */


                    if (i_osq_nacked_wr_en && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else if (i_osq_nacked_wr_en && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != 0 && ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~aeth_pkt)) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else if (i_osq_nacked_wr_en && i_osq_nacked_wr_addr != o_qp_id) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else begin
                        o_osq_nacked_resp_wr_en <= 1'b0;
                    end

                    if (i_osq_nacked_wr_en_aux && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else if (i_osq_nacked_wr_en_aux && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != 0 && ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~aeth_pkt)) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else if (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux != o_qp_id) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else begin
                        o_osq_nacked_resp_wr_en_aux <= 1'b0;
                    end

                    if (((i_osq_nacked_wr_en && i_osq_nacked_wr_addr == o_qp_id) || (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux == o_qp_id)) && hdr_gen_cs != IDLE && hdr_gen_cs != RD_WQE &&
			  ~(hdr_gen_cs == RD_Q_INFO && (i_reg_info_val && o_qp_id != 0 && 
			  ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data)  && ~aeth_pkt)))) begin
                        osq_nacked_wr_pending <= 1'b1;
                        osq_nacked_wr_id      <= i_osq_nacked_wr_en ? i_osq_nacked_wr_addr : i_osq_nacked_wr_addr_aux;
                        osq_nacked_wr_data    <= i_osq_nacked_wr_en ? i_osq_nacked_wr_data : i_osq_nacked_wr_data_aux;
                    end else if (hdr_gen_cs == IDLE) begin
                        osq_nacked_wr_pending <= 1'b0;
                    end

                    if (hdr_gen_cs == IDLE && osq_nacked_wr_pending) begin
                        o_osq_nacked_resp_wr_en_pending   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_pending <= osq_nacked_wr_id;
                        o_osq_nacked_resp_wr_data_pending <= osq_nacked_wr_data;
                    end else begin
                        o_osq_nacked_resp_wr_en_pending <= 1'b0;
                    end

                    case (hdr_gen_cs)
                        IDLE: begin
                            push_to_os_buf        <= 1'b0;
                            retried               <= 1'b0;
                            get_data_buf          <= 1'b0;
                            o_data_buf_fifo_wr_en <= 1'b0;
                            o_hdr_gen_rdy         <= 1'b0;
                            //o_osq_nacked_resp <= i_osq_nacked;
                            wqe_pushed_in_fatal   <= 1'b0;
                            partial_wqe           <= 1'b0;
                            if (i_reg_psn_val) begin
                                o_reg_wr_en <= 1'b0;
                            end
                            if ((i_aeth_valid) && ~o_reg_wr_en) begin
                                hdr_gen_cs    <= RD_Q_INFO;
                                o_qp_id       <= i_aeth_qpid;
                                o_reg_rd_en   <= 1'b1;
                                o_reg_rd_en_s <= 1'b1;
                                aeth_pkt      <= 1'b1;
                                aeth_hdr      <= i_aeth_hdr;
                            end else if (~i_qpm_fifo_empty && ~o_reg_wr_en && ~i_wqe_halt && ~wait_for_dma_idle) begin
                                o_qpm_wqe_pop <= 1'b1;
                                hdr_gen_cs    <= RD_WQE;
                            end else begin
                                o_reg_rd_en   <= 1'b0;
                                o_reg_rd_en_s <= 1'b0;
                            end
                        end

                        RD_WQE: begin
                            o_qpm_wqe_pop <= 1'b0;
                            if (o_qpm_wqe_pop == 1'b0) begin
                                o_wqe_wrid     <= i_qpm_wqe_data[15:0];
                                o_qp_id        <= i_qpm_wqe_data[136+C_MAX_QID_WIDTH-1 : 136];
                                dma_len        <= i_qpm_wqe_data[127:96];
                                rem_len        <= i_qpm_wqe_data[127:96];
                                wqe_opcode     <= i_qpm_wqe_data[135:128];
                                local_offset   <= i_qpm_wqe_data[95:32];
                                remote_offset  <= i_qpm_wqe_data[223:160];
                                r_key          <= i_qpm_wqe_data[255:224];
                                nvme_comp_data <= (i_qpm_wqe_data[129] || (i_qpm_wqe_data[131] && i_qpm_wqe_data[130])) ? i_qpm_wqe_data[383:256] : 128'd0000;
                                imm_data       <= i_qpm_wqe_data[415:384];
                                if (PSN_based_retransmission == 1) begin
                                    completion_data_present <= (i_qpm_wqe_data[131:128] == 4'b010 || i_qpm_wqe_data[131:128] == 4'b1100 || i_qpm_wqe_data[131:128] == 4'b0011) && ~(i_qpm_wqe_data[127:96] > 32'h00000010) && ~i_qpm_wqe_data[partial_wqe_bit_pos];
                                end else begin
                                    completion_data_present <= (i_qpm_wqe_data[131:128] == 4'b010 || i_qpm_wqe_data[131:128] == 4'b1100 || i_qpm_wqe_data[131:128] == 4'b0011) && ~(i_qpm_wqe_data[127:96] > 32'h00000010);
                                end
                                o_reg_rd_en    <= 1'b1;
                                o_reg_rd_en_s  <= 1'b1;
                                hdr_gen_cs     <= RD_Q_INFO;
                                first_trans    <= 1'b1;
                                retried_buf_id <= i_qpm_wqe_data[17+:DATA_BUF_ID_WIDTH];
                                retried        <= i_qpm_wqe_data[16];
                                if (PSN_based_retransmission == 1) begin
                                    partial_wqe           <= i_qpm_wqe_data[partial_wqe_bit_pos];
                                    partial_wqe_start_psn <= i_qpm_wqe_data[439:416];
                                end else begin
                                    partial_wqe <= 1'b0;
                                end
                            end
                        end

                        RD_Q_INFO: begin
                            if (i_reg_info_val) begin
                                if (o_qp_id == 0) begin
                                    hdr_gen_cs  <= HDR_GEN;
                                    payload_len <= rem_len;
                                    rem_len     <= 32'd0;
                                end else if ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~aeth_pkt) begin
                                    //hdr_gen_cs <= IDLE;
                                    hdr_gen_cs              <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? IDLE : DROP_WQE;
                                    //push_to_os_buf <= (i_qp_fatal[o_qp_id] || i_qp_recovery) ? 1'b1 : 1'b0;
                                    o_wqe_dropped           <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b0 : 1'b1;
                                    o_wqe_dropped_qpid      <= o_qp_id;
                                    push_to_os_buf          <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b1 : 1'b0;
                                    o_reg_rd_en             <= 1'b0;
                                    first_trans             <= 1'b0;
                                    completion_data_present <= 1'b0;
                                    //o_osq_nacked_resp <= i_osq_nacked;
                                    //wqe_pushed_in_fatal <= i_qp_fatal[o_qp_id] || i_qp_recovery;
                                    wqe_pushed_in_fatal     <= (qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1);
                                end else if ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) && (aeth_pkt && ~(aeth_hdr[30:29] == 2'b11 && |aeth_hdr[26:24]))) begin
                                    hdr_gen_cs    <= IDLE;
                                    o_reg_rd_en   <= 1'b0;
                                    first_trans   <= 1'b0;
                                    o_hdr_gen_rdy <= 1'b1;
                                    aeth_pkt      <= 1'b0;
                                end else if (~(wqe_opcode == 8'h00 || wqe_opcode == 8'h02 || wqe_opcode == 8'h04 || wqe_opcode == 8'h0C || wqe_opcode == 8'h01 || wqe_opcode == 8'h03 || aeth_pkt)) begin
                                    hdr_gen_cs       <= ILL_OPCODE;
                                    o_wqe_fatal_qpid <= o_qp_id;
                                end else begin
                                    hdr_gen_cs <= WAIT0;
                                end
                                exp_ack              <= i_reg_exp_ack;
                                o_reg_rd_en_s        <= 1'b0;
                                dest_qpid            <= i_reg_dest_qpid;
                                mac_dest_addr        <= i_reg_mac_dest_addr;
                                ip_dest_addr         <= i_reg_ip_dest_addr;
                                ip_ver               <= i_reg_ip_ver;
                                o_reg_sq_psn         <= i_reg_sq_psn;
                                o_wqe_send_first_psn <= partial_wqe ? partial_wqe_start_psn : i_reg_sq_psn;
                                o_reg_sq_msn         <= i_reg_sq_msn + 1'b1;
                                reg_qp_to            <= i_reg_qp_to;
                            end
                            wqe_os_fifo_full <= (os_num_vld_entries >= OS_TH_ENTRIES);

                            case (i_reg_pmtu)
                                3'b000: begin
                                    max_pmtu <= 13'h0100;
                                end
                                3'b001: begin
                                    max_pmtu <= 13'h0200;
                                end
                                3'b010: begin
                                    max_pmtu <= 13'h0400;
                                end
                                3'b011: begin
                                    max_pmtu <= 13'h0800;
                                end
                                3'b100: begin
                                    max_pmtu <= 13'h1000;
                                end
                                default: begin
                                    max_pmtu <= 13'h100;
                                end
                            endcase
                        end

                        WAIT0: begin
                            get_data_buf          <= 1'b0;
                            o_data_buf_fifo_wr_en <= 1'b0;
                            push_to_os_buf        <= 1'b0;
                            if (i_reg_psn_val) begin
                                o_reg_wr_en <= 1'b0;
                            end
                            if (ip_ver && ~o_reg_wr_en) begin
                                hdr_gen_cs <= HDR_GEN;
                            end else if (~ip_ver && ~o_reg_wr_en) begin
                                hdr_gen_cs          <= IP_CHKSUM;
                                ipv4_identification <= ipv4_identification + 1'b1;
                                ipv4_len_cal        <= 1'b0;
                            end

                            if (~o_reg_wr_en) begin
                                if (aeth_pkt) begin
                                    payload_len  <= 32'd0;
                                    rem_len      <= 32'd0;
                                    reth_present <= 1'b0;
                                end else if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (wqe_opcode[7:1] == 7'h00)) begin
                                    payload_len  <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                                    rem_len      <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                                    reth_present <= first_trans && ~partial_wqe;
                                end else if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (wqe_opcode[7:1] == 8'h01 || wqe_opcode == 8'h0C)) begin
                                    payload_len  <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                                    rem_len      <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                                    reth_present <= 1'b0;
                                end else if (wqe_opcode == 8'h04) begin
                                    payload_len  <= 32'd0;
                                    rem_len      <= rem_len;
                                    reth_present <= 1'b1;
                                end else begin
                                    payload_len  <= rem_len;
                                    rem_len      <= 32'd0;
                                    reth_present <= 1'b0;
                                end
                            end
                        end

                        IP_CHKSUM: begin

                            if (~ipv4_len_cal) begin
                                ipv4_len_cal           <= 1'b1;
                                ip_hdr_chksum_add_pipe <= 16'h4501 + ipv4_identification[15:0] + 16'h4000 + ip_dest_addr[31:16] + ip_dest_addr[15:0];
                                if (((first_trans && ~|rem_len) || (~first_trans && ~|rem_len)) && (wqe_opcode == 8'h0C || wqe_opcode == 8'h03 || wqe_opcode == 8'h01) && (!aeth_pkt)) begin
                                    ipv4_len <= 8'h2C + payload_len + {reth_present, 1'b0, aeth_pkt, 2'b00} + 3'd4;
                                end else begin
                                    ipv4_len <= 8'h2C + payload_len + {reth_present, 1'b0, aeth_pkt, 2'b00};
                                end
                            end else if (~chksum_cal_start) begin
                                //ip_hdr_chksum <= 16'h4501 +  ipv4_len[15:0] + ipv4_identification[15:0] + 16'h4000 + { i_reg_ttl,8'h11} + 
                                //                 i_reg_ipv4_src_addr[31:16] + i_reg_ipv4_src_addr[15:0] + 
                                //                 ip_dest_addr[31:16] + ip_dest_addr[15:0];

                                ip_hdr_chksum    <= ip_hdr_chksum_add_pipe + ipv4_len[15:0] + {i_reg_ttl, 8'h11} + i_reg_ipv4_src_addr[31:16] + i_reg_ipv4_src_addr[15:0];
                                chksum_cal_start <= 1'b1;
                            end else begin
                                if (|ip_hdr_chksum[19:16]) begin
                                    ip_hdr_chksum <= ip_hdr_chksum[15:0] + ip_hdr_chksum[19:16];
                                end else begin
                                    chksum_cal_start <= 1'b0;
                                    hdr_gen_cs       <= HDR_GEN;
                                end
                            end
                        end

                        HDR_GEN: begin
                            if (~no_data_buf_left && ~buf_mgr_busy && ((wqe_opcode[7:1] != 7'h00) || (~retried || i_data_buf_fifo_empty))) begin
                                wqe_opcode_err <= 1'b0;
                                eth_hdr <= {
                                    (ip_ver ? 16'hDD86 : 16'h0008),
                                    i_reg_mac_src_addr[7:0],
                                    i_reg_mac_src_addr[15:8],
                                    i_reg_mac_src_addr[23:16],
                                    i_reg_mac_src_addr[31:24],
                                    i_reg_mac_src_addr[39:32],
                                    i_reg_mac_src_addr[47:40],
                                    mac_dest_addr[7:0],
                                    mac_dest_addr[15:8],
                                    mac_dest_addr[23:16],
                                    mac_dest_addr[31:24],
                                    mac_dest_addr[39:32],
                                    mac_dest_addr[47:40]
                                };
                                if (ip_ver) begin
                                    ip_hdr <= {
                                        ip_dest_addr[7:0],
                                        ip_dest_addr[15:8],
                                        ip_dest_addr[23:16],
                                        ip_dest_addr[31:24],
                                        ip_dest_addr[39:32],
                                        ip_dest_addr[47:40],
                                        ip_dest_addr[55:48],
                                        ip_dest_addr[63:56],
                                        ip_dest_addr[71:64],
                                        ip_dest_addr[79:72],
                                        ip_dest_addr[87:80],
                                        ip_dest_addr[95:88],
                                        ip_dest_addr[103:96],
                                        ip_dest_addr[111:104],
                                        ip_dest_addr[119:112],
                                        ip_dest_addr[127:120],
                                        i_reg_ipv6_src_addr[7:0],
                                        i_reg_ipv6_src_addr[15:8],
                                        i_reg_ipv6_src_addr[23:16],
                                        i_reg_ipv6_src_addr[31:24],
                                        i_reg_ipv6_src_addr[39:32],
                                        i_reg_ipv6_src_addr[47:40],
                                        i_reg_ipv6_src_addr[55:48],
                                        i_reg_ipv6_src_addr[63:56],
                                        i_reg_ipv6_src_addr[71:64],
                                        i_reg_ipv6_src_addr[79:72],
                                        i_reg_ipv6_src_addr[87:80],
                                        i_reg_ipv6_src_addr[95:88],
                                        i_reg_ipv6_src_addr[103:96],
                                        i_reg_ipv6_src_addr[111:104],
                                        i_reg_ipv6_src_addr[119:112],
                                        i_reg_ipv6_src_addr[127:120],
                                        i_reg_ttl,
                                        8'h11,
                                        ipv6_len[7:0],
                                        ipv6_len[15:8],
                                        q_id[7:0],
                                        q_id[15:8],
                                        {i_reg_dscp[1:0], 2'b01, 4'h0},
                                        {4'h6, i_reg_dscp[5:2]}
                                    };
                                end else begin
                                    ip_hdr[159:96] <= {ip_dest_addr[7:0], ip_dest_addr[15:8], ip_dest_addr[23:16], ip_dest_addr[31:24], i_reg_ipv4_src_addr[7:0], i_reg_ipv4_src_addr[15:8], i_reg_ipv4_src_addr[23:16], i_reg_ipv4_src_addr[31:24]};
                                    ip_hdr[95:80]  <= {~ip_hdr_chksum[7:0], ~ip_hdr_chksum[15:8]};
                                    ip_hdr[79:0]   <= {8'h11, i_reg_ttl, 16'h0040, ipv4_identification[7:0], ipv4_identification[15:8], ipv4_len[7:0], ipv4_len[15:8], 16'h0145};
                                end
                                udp_hdr <= {16'h0000, ipv6_len[7:0], ipv6_len[15:8], 16'hB712, i_reg_udp_src_port[7:0], i_reg_udp_src_port[15:8]};  //Need to check that UDP destination port is decimal or hexa, considering it as decimal 4791 for now
                                if (aeth_pkt) begin
                                    bth_hdr[95:72] <= {i_aeth_psn[7:0], i_aeth_psn[15:8], i_aeth_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end else begin
                                    //bth_hdr[95:32] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16], (exp_ack || wqe_os_fifo_full), 7'h00,  dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                    bth_hdr[95:72] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end
                                bth_hdr[31:16] <= {i_reg_pkey[7:0], i_reg_pkey[15:8]};
                                bth_hdr[14:8]  <= {1'b0, 2'b00, 4'h0};
                                //SE Bit chnage if((|rem_len && (wqe_opcode == 8'h00 || wqe_opcode == 8'h02)) || aeth_pkt) begin
                                if (~|rem_len && (wqe_opcode == 8'h02 || wqe_opcode == 8'h0C || wqe_opcode == 8'h03)) begin
                                    bth_hdr[15] <= 1'b1;
                                end else begin
                                    bth_hdr[15] <= 1'b0;
                                end
                                if (aeth_pkt) begin
                                    bth_hdr[7:0] <= 8'h11;
                                    bth_hdr[71]  <= 1'b0;
                                end else if (wqe_opcode[7:1] == 7'h00) begin
                                    o_out_rw_pkt_cnt <= o_out_rw_pkt_cnt + 1'b1;
                                    bth_hdr[71]      <= (~|rem_len);
                                    if (PSN_based_retransmission == 1 && partial_wqe == 1'b1) begin
                                        if (|rem_len) begin
                                            bth_hdr[7:0]   <= 8'h07;  // WRITE_MIDDLE
                                            push_to_os_buf <= first_trans;
                                        end else begin
                                            bth_hdr[7:0]     <= {7'h04, wqe_opcode[0]};  //WRITE_LAST OR WRITE_LAST_IMMDT
                                            push_to_os_buf   <= first_trans;
                                            imm_data_present <= wqe_opcode[0];
                                        end
                                    end else if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h06;  //WRITE_FIRST
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]     <= {7'h05, wqe_opcode[0]};  //WRITE_ONLY, or WRITE_ONLY_IMMDT
                                        push_to_os_buf   <= 1'b1;
                                        imm_data_present <= wqe_opcode[0];
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]     <= {7'h04, wqe_opcode[0]};  //WRITE_LAST, or WRITE_LAST_IMMDT
                                        push_to_os_buf   <= 1'b0;
                                        imm_data_present <= wqe_opcode[0];
                                    end
                                end else if (wqe_opcode == 8'h04) begin
                                    o_out_rw_pkt_cnt <= o_out_rw_pkt_cnt + 1'b1;
                                    bth_hdr[7:0]     <= 8'h0C;  //RDMA_READ
                                    bth_hdr[71]      <= 1'b1;
                                    push_to_os_buf   <= 1'b1;
                                end else if (wqe_opcode[7:1] == 7'h01 || wqe_opcode == 8'h0C) begin
                                    o_out_send_pkt_cnt <= o_out_send_pkt_cnt + ((q_id != 16'h0000) ? 1'b1 : 1'b0);
                                    bth_hdr[71]        <= exp_ack || (~|rem_len);
                                    if (PSN_based_retransmission == 1 && partial_wqe == 1'b1) begin
                                        if (|rem_len) begin
                                            bth_hdr[7:0]   <= 8'h01;  //SEND_MIDDLE
                                            push_to_os_buf <= first_trans;
                                        end else if (wqe_opcode == 8'h0C) begin
                                            bth_hdr[7:0]   <= 8'b0001_0110;  // SEND_LAST with Invalidate
                                            push_to_os_buf <= first_trans;
                                        end else begin
                                            bth_hdr[7:0]     <= {7'h01, wqe_opcode[0]};  //SEND_LAST, or SEND_LAST_IMMDT
                                            push_to_os_buf   <= first_trans;
                                            imm_data_present <= wqe_opcode[0];
                                        end
                                    end else if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h00;  //SEND_FIRST
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len && wqe_opcode == 8'h0C) begin
                                        bth_hdr[7:0]   <= 8'b0001_0111;  //SEND_ONLY with Invalidate
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]     <= {7'h02, wqe_opcode[0]};  //SEND_ONLY, or SEND_ONLY_IMMDT
                                        push_to_os_buf   <= 1'b1;
                                        imm_data_present <= wqe_opcode[0];
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h01;  //SEND_MIDDLE
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len && wqe_opcode == 8'h0C) begin
                                        bth_hdr[7:0]   <= 8'b0001_0110;  // SEND_LAST with Invalidate
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]     <= {7'h01, wqe_opcode[0]};  //SEND_LAST, or SEND_LAST_IMMDT
                                        push_to_os_buf   <= 1'b0;
                                        imm_data_present <= wqe_opcode[0];
                                    end
                                end else begin
                                    bth_hdr[7:0]   <= 8'h04;
                                    bth_hdr[71]    <= 1'b0;
                                    push_to_os_buf <= 1'b0;
                                end
                                if (wqe_opcode == 8'h04) begin
                                    case (max_pmtu[12:8])
                                        5'h01: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:8] + |(rem_len[7:0]);
                                        end
                                        5'h02: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:9] + |(rem_len[8:0]);
                                        end
                                        5'h04: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:10] + |(rem_len[9:0]);
                                        end
                                        5'h08: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:11] + |(rem_len[10:0]);
                                        end
                                        5'h10: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:12] + |(rem_len[11:0]);
                                        end
                                        default: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:8] + |(rem_len[8:0]);
                                        end
                                    endcase
                                end else begin
                                    o_reg_sq_psn <= o_reg_sq_psn + (aeth_pkt ? 1'b0 : 1'b1);
                                end
                                o_out_ack_pkt_cnt <= o_out_ack_pkt_cnt + (aeth_pkt ? 1'b1 : 1'b0);
                                o_out_mad_pkt_cnt <= o_out_mad_pkt_cnt + (q_id == 16'h0000 ? 1'b1 : 1'b0);
                                local_offset <= local_offset + payload_len;
                                remote_offset <= remote_offset + payload_len;
                                reth_hdr <= {
                                    dma_len[7:0],
                                    dma_len[15:8],
                                    dma_len[23:16],
                                    dma_len[31:24],
                                    r_key[7:0],
                                    r_key[15:8],
                                    r_key[23:16],
                                    r_key[31:24],
                                    remote_offset[7:0],
                                    remote_offset[15:8],
                                    remote_offset[23:16],
                                    remote_offset[31:24],
                                    remote_offset[39:32],
                                    remote_offset[47:40],
                                    remote_offset[55:48],
                                    remote_offset[63:56]
                                };
                                o_hdr_data[1023:928] <= {payload_len, local_offset, completion_data_present, q_id[14:0]};
                                o_hdr_gen_rdy <= aeth_pkt;
                                if (aeth_pkt || (~(i_wqe_halt && (i_wqe_halted_qpid == o_qp_id))  /* && ~(i_osq_nacked[o_qp_id])*/)) begin
                                    hdr_gen_cs <= HDR_STO;
                                    //if(wqe_opcode == 8'h00 && first_trans && ~aeth_pkt) begin
                                    //    get_data_buf <= ~retried;
                                    //end else begin
                                    //    get_data_buf <= 1'b0;
                                    //end
                                end else begin
                                    hdr_gen_cs          <= DROP_WQE;
                                    o_wqe_dropped       <= first_trans;
                                    o_wqe_dropped_qpid  <= o_qp_id;
                                    ipv4_identification <= ipv4_identification - (ip_ver ? 1'b0 : 1'b1);
                                end
                            end
                        end

                        HDR_STO: begin
                            //get_data_buf <= 1'b0;
                            o_data_buf_fifo_data <= {data_buf_addr[C_M_AXI_ADDR_WIDTH-1:2], partial_wqe, (retried || (q_id == 16'h0000))};
                            o_hdr_gen_rdy        <= 1'b0;
                            if (~wqe_push_pending) begin
                                o_hdr_data[111:0] <= eth_hdr;
                                if (ip_ver) begin
                                    o_hdr_data[431:112] <= ip_hdr;
                                    o_hdr_data[495:432] <= udp_hdr;
                                    o_hdr_data[591:496] <= bth_hdr;
                                    if (reth_present) begin
                                        o_hdr_data[719:592] <= reth_hdr;
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h5E;  //94B header
                                            o_hdr_data[751:720] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                        end else begin
                                            o_hdr_len <= 8'h5A;  //90B header
                                        end
                                    end else if (aeth_pkt) begin
                                        o_hdr_data[623:592] <= {aeth_hdr[7:0], aeth_hdr[15:8], aeth_hdr[23:16], aeth_hdr[31:24]};
                                        o_hdr_len           <= 8'h4E;  //78B header
                                    end else if (bth_hdr[7:1] != 7'b0001_011 && completion_data_present) begin
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h5E;  //94B header
                                            o_hdr_data[623:592] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                            o_hdr_data[751:624] <= nvme_comp_data;
                                        end else begin
                                            o_hdr_len           <= 8'h5A;  //90B header
                                            o_hdr_data[719:592] <= nvme_comp_data;
                                        end
                                    end else if (bth_hdr[7:1] == 7'b0001_011) begin  // Send Invalidate only or send last  Invaliate 
                                        o_hdr_data[623:592] <= {r_key[7:0], r_key[15:8], r_key[23:16], r_key[31:24]};  // 32  bit
                                        if (completion_data_present) begin
                                            o_hdr_data[751:624] <= nvme_comp_data;
                                            o_hdr_len           <= 8'h5E;  // 94B header
                                        end else begin
                                            o_hdr_data[751:624] <= 'b0;
                                            o_hdr_len           <= 8'h4E;  // 78B header
                                        end
                                    end else begin
                                        o_hdr_data[751:624] <= 128'd0000;
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h4E;  //78B header
                                            o_hdr_data[623:592] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                        end else begin
                                            o_hdr_len           <= 8'h4A;  //74B header
                                            o_hdr_data[623:592] <= 32'd0;
                                        end
                                    end
                                end else begin
                                    o_hdr_data[271:112] <= ip_hdr[159:0];
                                    o_hdr_data[335:272] <= udp_hdr;
                                    o_hdr_data[431:336] <= bth_hdr;
                                    if (reth_present) begin
                                        o_hdr_data[559:432] <= reth_hdr;
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h4A;  //74B header
                                            o_hdr_data[591:560] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                        end else begin
                                            o_hdr_len <= 8'h46;  //70B header
                                        end
                                    end else if (aeth_pkt) begin
                                        o_hdr_data[463:432] <= {aeth_hdr[7:0], aeth_hdr[15:8], aeth_hdr[23:16], aeth_hdr[31:24]};
                                        o_hdr_len           <= 8'h3A;  //58B header
                                    end else if (bth_hdr[7:1] != 7'b0001_011 && completion_data_present) begin
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h4A;  //74B header
                                            o_hdr_data[463:432] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                            o_hdr_data[591:464] <= nvme_comp_data;
                                        end else begin
                                            o_hdr_len           <= 8'h46;  //70B header
                                            o_hdr_data[559:432] <= nvme_comp_data;
                                        end
                                    end else if (bth_hdr[7:1] == 7'b0001_011) begin  // Send Invalidate only or send last  Invaliate 
                                        o_hdr_data[463:432] <= {r_key[7:0], r_key[15:8], r_key[23:16], r_key[31:24]};  // 32 bit
                                        if (completion_data_present) begin
                                            o_hdr_data[591:464] <= nvme_comp_data;
                                            o_hdr_len           <= 8'h4A;  //74B header
                                        end else begin
                                            o_hdr_len           <= 8'h3A;  // 58B header
                                            o_hdr_data[591:464] <= 'b0;
                                        end
                                    end else begin
                                        o_hdr_data[591:464] <= 128'd0000;
                                        if (imm_data_present) begin
                                            o_hdr_len           <= 8'h3A;  //58B header
                                            o_hdr_data[463:432] <= {imm_data[7:0], imm_data[15:8], imm_data[23:16], imm_data[31:24]};
                                        end else begin
                                            o_hdr_len           <= 8'h36;  //54B header
                                            o_hdr_data[463:432] <= 32'd0;
                                        end
                                    end
                                end
                                //if(wqe_opcode[7:1] == 7'h00 && first_trans && ~aeth_pkt && ~partial_wqe) begin
                                if (wqe_opcode[7:1] == 7'h00 && first_trans && ~aeth_pkt) begin
                                    o_data_buf_fifo_wr_en <= 1'b1;
                                    get_data_buf          <= ~retried;
                                end else begin
                                    o_data_buf_fifo_wr_en <= 1'b0;
                                    get_data_buf          <= 1'b0;
                                end
                                push_to_os_buf   <= 1'b0;
                                first_trans      <= 1'b0;
                                o_reg_wr_en      <= ~aeth_pkt;
                                imm_data_present <= 1'b0;
                                if (wqe_opcode[7:1] == 7'h00 && |rem_len) begin
                                    hdr_gen_cs <= WAIT0;
                                end else if ((wqe_opcode[7:1] == 8'h01 || wqe_opcode == 8'h0C) && |rem_len) begin
                                    hdr_gen_cs <= WAIT0;
                                end else begin
                                    hdr_gen_cs  <= IDLE;
                                    o_reg_rd_en <= 1'b0;
                                end
                                aeth_pkt     <= 1'b0;
                                reth_present <= 1'b0;
                            end
                        end

                        ILL_OPCODE: begin
                            o_hdr_gen_rdy <= 1'b0;
                            if (~o_wqe_fatal_qp_vld) begin
                                //o_hdr_data[1023:512] <= 512'd0;
                                //o_hdr_data[511:0] <= {128'd0,nvme_comp_data,16'h0000,q_id,r_key,remote_offset,wqe_opcode,dma_len,local_offset,o_wqe_wrid};
                                //o_hdr_len <= 8'h00; 
                                hdr_gen_cs     <= IDLE;
                                o_reg_rd_en    <= 1'b0;
                                o_reg_wr_en    <= ~aeth_pkt;
                                aeth_pkt       <= 1'b0;
                                wqe_opcode_err <= 1'b1;
                                push_to_os_buf <= 1'b1;
                            end
                        end

                        DROP_WQE: begin
                            //if(~(i_osq_pop && (i_osq_pop_qpid == o_qp_id))) begin
                            hdr_gen_cs       <= IDLE;
                            o_wqe_dropped    <= 1'b0;
                            imm_data_present <= 1'b0;
                            //end
                        end

                        default: begin
                        end
                    endcase
                end
            end

        end
    endgenerate  // end of C_EN_RECV_RD_WR == 0 && NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 1



    //////////////////////////////////////////////////////////////////////////////SEND_INVALIDATE



    generate
        if (C_EN_RECV_RD_WR == 0 && NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 0) begin : no_rd_resp_ONLY

            assign ipv6_len = (8'h18 + payload_len + {reth_present, 1'b0, aeth_pkt, 2'b00});
            //assign ipv4_len = (8'h2C + payload_len + {reth_present,1'b0,aeth_pkt,2'b00});
            always @(posedge core_clk) begin
                if (core_rst) begin
                    eth_hdr                           <= 'd0;
                    ip_hdr                            <= 'd0;
                    udp_hdr                           <= 'd0;
                    bth_hdr                           <= 'd0;
                    reth_hdr                          <= 'd0;
                    aeth_hdr                          <= 'd0;
                    o_hdr_data                        <= 'd0;
                    o_hdr_valid                       <= 1'b0;
                    o_hdr_ip_ver                      <= 1'b0;
                    o_hdr_len                         <= 8'h00;
                    o_qpm_wqe_pop                     <= 1'b0;
                    o_qp_id                           <= {C_MAX_QID_WIDTH{1'b0}};
                    o_reg_rd_en                       <= 1'b0;
                    o_reg_rd_en_s                     <= 1'b0;
                    hdr_gen_cs                        <= IDLE;
                    ip_hdr_chksum                     <= 20'd0;
                    rem_len                           <= 32'd0;
                    dma_len                           <= 32'd0;
                    first_trans                       <= 1'b0;
                    payload_len                       <= 16'h0000;
                    reth_present                      <= 1'b0;
                    remote_offset                     <= 64'd00;
                    local_offset                      <= 64'd00;
                    aeth_pkt                          <= 1'b0;
                    wqe_opcode                        <= 8'h00;
                    r_key                             <= 32'd0;
                    chksum_cal_start                  <= 1'b0;
                    nvme_comp_data                    <= 128'd0000;
                    completion_data_present           <= 1'b0;
                    o_reg_wr_en                       <= 1'b0;
                    wqe_opcode_err                    <= 1'b0;
                    err_opcode                        <= 8'h00;
                    dest_qpid                         <= 24'd0;
                    o_hdr_gen_rdy                     <= 1'b0;
                    o_reg_sq_psn                      <= 24'd0;
                    o_wqe_send_first_psn              <= 24'd0;
                    o_reg_sq_msn                      <= 24'd0;
                    o_wqe_wrid                        <= 16'h0000;
                    mac_dest_addr                     <= 'd0;
                    ip_dest_addr                      <= 'd0;
                    ip_ver                            <= 1'b0;
                    exp_ack                           <= 1'b0;
                    o_out_send_pkt_cnt                <= 16'h0000;
                    o_out_rw_pkt_cnt                  <= 16'h0000;
                    o_out_ack_pkt_cnt                 <= 16'h0000;
                    o_out_mad_pkt_cnt                 <= 16'h0000;
                    o_wqe_halted                      <= 1'b0;
                    wqe_os_fifo_full                  <= 1'b0;
                    ipv4_identification               <= 16'h0000;
                    retried                           <= 1'b0;
                    get_data_buf                      <= 1'b0;
                    o_data_buf_fifo_data              <= 32'd0;
                    o_data_buf_fifo_wr_en             <= 1'b0;
                    //o_osq_nacked_resp <= 'd0;
                    push_to_os_buf                    <= 1'b0;
                    buf_mgr_busy                      <= 1'b0;
                    reg_qp_to                         <= 5'h00;
                    wait_for_dma_idle                 <= 1'b0;
                    //qp_fatal_detected<= 'h0;
                    retried_buf_id                    <= 'h0;
                    wqe_pushed_in_fatal               <= 1'b0;
                    o_wqe_fatal_qpid                  <= {C_MAX_QID_WIDTH{1'b0}};
                    o_wqe_fatal_qp_vld                <= 1'b0;
                    o_wqe_dropped                     <= 1'b0;
                    o_wqe_dropped_qpid                <= {C_MAX_QID_WIDTH{1'b0}};
                    ipv4_len_cal                      <= 1'b0;
                    ip_hdr_chksum_add_pipe            <= 17'd0;
                    ipv4_len                          <= 16'd0;
                    partial_wqe                       <= 1'b0;
                    partial_wqe_start_psn             <= 24'd0;
                    qp_fatal_detected_wr_en           <= 1'b0;
                    qp_fatal_detected_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_fatal_detected_wr_data         <= 1'b0;
                    qp_fatal_detected_wr_en_aux       <= 1'b0;
                    qp_fatal_detected_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_fatal_detected_wr_data_aux     <= 1'b0;
                    qp_wqe_opcode_fatal_wr_en         <= 1'b0;
                    qp_wqe_opcode_fatal_wr_addr       <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_wqe_opcode_fatal_wr_data       <= 1'b0;
                    qp_wqe_opcode_fatal_wr_en_aux     <= 1'b0;
                    qp_wqe_opcode_fatal_wr_addr_aux   <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_wqe_opcode_fatal_wr_data_aux   <= 1'b0;
                    o_osq_nacked_resp_wr_en           <= 1'b0;
                    o_osq_nacked_resp_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data         <= 1'b0;
                    o_osq_nacked_resp_wr_en_aux       <= 1'b0;
                    o_osq_nacked_resp_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data_aux     <= 1'b0;
                    o_osq_nacked_resp_wr_en_pending   <= 1'b0;
                    o_osq_nacked_resp_wr_addr_pending <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data_pending <= 1'b0;
                    osq_nacked_wr_pending             <= 1'b0;
                    osq_nacked_wr_id                  <= {C_MAX_QID_WIDTH{1'b0}};
                    osq_nacked_wr_data                <= 1'b0;
                end else begin
                    //qp_fatal_detected<= (i_qp_fatal & (~i_osq_nacked));
                    /*if (i_qp_fatal_wr_en && i_osq_nacked_wr_en && i_qp_fatal_wr_addr == i_osq_nacked_wr_addr) begin
				qp_fatal_detected_wr_en <= 1'b1;
				qp_fatal_detected_wr_en_aux <= 1'b0;
				qp_fatal_detected_wr_addr <= i_qp_fatal_wr_addr;
				qp_fatal_detected_wr_data <= i_qp_fatal_wr_data & ~i_osq_nacked_wr_data;
			  end else begin
				 if (i_qp_fatal_wr_en) begin
					qp_fatal_detected_wr_en <= 1'b1;
					qp_fatal_detected_wr_addr <= i_qp_fatal_wr_addr;
					qp_fatal_detected_wr_data <= i_qp_fatal_wr_data & ~osq_nacked_rd_data_0;
				end else begin
					qp_fatal_detected_wr_en <= 1'b0;
				end
				
				if (i_osq_nacked_wr_en) begin
					qp_fatal_detected_wr_en_aux <= 1'b1;
					qp_fatal_detected_wr_addr_aux <= i_osq_nacked_wr_addr;
					qp_fatal_detected_wr_data_aux <= qp_fatal_rd_data_0 & ~i_osq_nacked_wr_data;
				end else begin
					qp_fatal_detected_wr_en_aux <= 1'b0;
				end
			  end */
                    if (~wqe_push_pending && (hdr_gen_cs == HDR_STO  /*|| hdr_gen_cs == ILL_OPCODE*/)) begin
                        o_hdr_valid  <= 1'b1;
                        o_hdr_ip_ver <= ip_ver;
                    end else if (i_buf_mngr_rdy) begin
                        o_hdr_valid <= 1'b0;
                    end
                    if ((hdr_gen_cs == ILL_OPCODE) && ~o_wqe_fatal_qp_vld) begin
                        o_wqe_fatal_qp_vld <= 1'b1;
                    end else if (i_wqe_fatal_qp_rdy) begin
                        o_wqe_fatal_qp_vld <= 1'b0;
                    end
                    if (o_wqe_fatal_qp_vld && i_qp_fatal_wr_en && o_wqe_fatal_qpid == i_qp_fatal_wr_addr) begin
                        qp_wqe_opcode_fatal_wr_en     <= 1'b1;
                        qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                        qp_wqe_opcode_fatal_wr_addr   <= o_wqe_fatal_qpid;
                        qp_wqe_opcode_fatal_wr_data   <= 1'b1;
                    end else begin
                        if (o_wqe_fatal_qp_vld) begin
                            qp_wqe_opcode_fatal_wr_en   <= 1'b1;
                            qp_wqe_opcode_fatal_wr_addr <= o_wqe_fatal_qpid;
                            qp_wqe_opcode_fatal_wr_data <= 1'b1;
                        end else begin
                            qp_wqe_opcode_fatal_wr_en <= 1'b0;
                        end

                        if (i_qp_fatal_wr_en && i_qp_fatal_wr_data) begin
                            qp_wqe_opcode_fatal_wr_en_aux   <= 1'b1;
                            qp_wqe_opcode_fatal_wr_addr_aux <= i_qp_fatal_wr_addr;
                            qp_wqe_opcode_fatal_wr_data_aux <= 1'b0;
                        end else begin
                            qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                        end
                    end
                    if (o_hdr_valid && ~i_buf_mngr_rdy) begin
                        buf_mgr_busy <= 1'b1;
                    end else if (i_buf_mngr_rdy) begin
                        buf_mgr_busy <= 1'b0;
                    end
                    if (hdr_gen_cs == IDLE && i_wqe_halt && ~o_reg_wr_en) begin
                        o_wqe_halted <= 1'b1;
                    end else if (~i_wqe_halt) begin
                        o_wqe_halted <= 1'b0;
                    end
                    //if( |((i_qp_fatal & ~i_osq_nacked )& ~qp_fatal_detected)) begin
                    /* if (i_qp_fatal_wr_en && i_osq_nacked_wr_en && i_qp_fatal_wr_addr == i_osq_nacked_wr_addr && (i_qp_fatal_wr_data && ~i_osq_nacked_wr_data) && ~qp_fatal_detected_rd_data_0) begin
				 wait_for_dma_idle <= 1'b1;
			  end else if ((i_qp_fatal_wr_en && (i_qp_fatal_wr_data && ~osq_nacked_rd_data_0) && ~qp_fatal_detected_rd_data_0) || (i_osq_nacked_wr_en && (qp_fatal_rd_data_0 && ~i_osq_nacked_wr_data) && ~qp_fatal_detected_rd_data_1)) begin	 
				 wait_for_dma_idle <= 1'b1;
              end else if(dma_in_idle && ~o_hdr_valid && (hdr_gen_cs == IDLE)) begin
				wait_for_dma_idle <= 1'b0;
              end */

                    if (i_osq_nacked_wr_en && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else if (i_osq_nacked_wr_en && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != 0 && ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~aeth_pkt)) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else if (i_osq_nacked_wr_en && i_osq_nacked_wr_addr != o_qp_id) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else begin
                        o_osq_nacked_resp_wr_en <= 1'b0;
                    end

                    if (i_osq_nacked_wr_en_aux && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else if (i_osq_nacked_wr_en_aux && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != 0 && ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~aeth_pkt)) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else if (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux != o_qp_id) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else begin
                        o_osq_nacked_resp_wr_en_aux <= 1'b0;
                    end

                    if ((i_osq_nacked_wr_en && i_osq_nacked_wr_addr == o_qp_id) || (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux == o_qp_id) && hdr_gen_cs != IDLE && hdr_gen_cs != RD_WQE &&
			  ~(hdr_gen_cs == RD_Q_INFO && (i_reg_info_val && o_qp_id != 0 && 
			  ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data)  && ~aeth_pkt)))) begin
                        osq_nacked_wr_pending <= 1'b1;
                        osq_nacked_wr_id      <= i_osq_nacked_wr_en ? i_osq_nacked_wr_addr : i_osq_nacked_wr_addr_aux;
                        osq_nacked_wr_data    <= i_osq_nacked_wr_en ? i_osq_nacked_wr_data : i_osq_nacked_wr_data_aux;
                    end else if (hdr_gen_cs == IDLE) begin
                        osq_nacked_wr_pending <= 1'b0;
                    end

                    if (hdr_gen_cs == IDLE && osq_nacked_wr_pending) begin
                        o_osq_nacked_resp_wr_en_pending   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_pending <= osq_nacked_wr_id;
                        o_osq_nacked_resp_wr_data_pending <= osq_nacked_wr_data;
                    end else begin
                        o_osq_nacked_resp_wr_en_pending <= 1'b0;
                    end

                    case (hdr_gen_cs)
                        IDLE: begin
                            push_to_os_buf        <= 1'b0;
                            retried               <= 1'b0;
                            get_data_buf          <= 1'b0;
                            o_data_buf_fifo_wr_en <= 1'b0;
                            o_hdr_gen_rdy         <= 1'b0;
                            //o_osq_nacked_resp <= i_osq_nacked;
                            wqe_pushed_in_fatal   <= 1'b0;
                            partial_wqe           <= 1'b0;
                            if (i_reg_psn_val) begin
                                o_reg_wr_en <= 1'b0;
                            end
                            if ((i_aeth_valid) && ~o_reg_wr_en) begin
                                hdr_gen_cs    <= RD_Q_INFO;
                                o_qp_id       <= i_aeth_qpid;
                                o_reg_rd_en   <= 1'b1;
                                o_reg_rd_en_s <= 1'b1;
                                aeth_pkt      <= 1'b1;
                                aeth_hdr      <= i_aeth_hdr;
                            end else if (~i_qpm_fifo_empty && ~o_reg_wr_en && ~i_wqe_halt && ~wait_for_dma_idle) begin
                                o_qpm_wqe_pop <= 1'b1;
                                hdr_gen_cs    <= RD_WQE;
                            end else begin
                                o_reg_rd_en   <= 1'b0;
                                o_reg_rd_en_s <= 1'b0;
                            end
                        end
                        RD_WQE: begin
                            o_qpm_wqe_pop <= 1'b0;
                            if (o_qpm_wqe_pop == 1'b0) begin
                                o_wqe_wrid     <= i_qpm_wqe_data[15:0];
                                o_qp_id        <= i_qpm_wqe_data[136+C_MAX_QID_WIDTH-1 : 136];
                                dma_len        <= i_qpm_wqe_data[127:96];
                                rem_len        <= i_qpm_wqe_data[127:96];
                                wqe_opcode     <= i_qpm_wqe_data[135:128];
                                local_offset   <= i_qpm_wqe_data[95:32];
                                remote_offset  <= i_qpm_wqe_data[223:160];
                                r_key          <= i_qpm_wqe_data[255:224];
                                nvme_comp_data <= i_qpm_wqe_data[129] ? i_qpm_wqe_data[383:256] : 128'd0000;
                                if (PSN_based_retransmission == 1) begin
                                    completion_data_present <= (i_qpm_wqe_data[130:128] == 3'b010) && ~(i_qpm_wqe_data[127:96] > 32'h00000010) && ~i_qpm_wqe_data[partial_wqe_bit_pos];
                                end else begin
                                    completion_data_present <= (i_qpm_wqe_data[130:128] == 3'b010) && ~(i_qpm_wqe_data[127:96] > 32'h00000010);
                                end
                                o_reg_rd_en    <= 1'b1;
                                o_reg_rd_en_s  <= 1'b1;
                                hdr_gen_cs     <= RD_Q_INFO;
                                first_trans    <= 1'b1;
                                retried_buf_id <= i_qpm_wqe_data[17+:DATA_BUF_ID_WIDTH];
                                retried        <= i_qpm_wqe_data[16];
                                if (PSN_based_retransmission == 1) begin
                                    partial_wqe           <= i_qpm_wqe_data[partial_wqe_bit_pos];
                                    partial_wqe_start_psn <= i_qpm_wqe_data[439:416];
                                end else begin
                                    partial_wqe <= 1'b0;
                                end
                            end
                        end
                        RD_Q_INFO: begin
                            if (i_reg_info_val) begin
                                if (o_qp_id == 0) begin
                                    hdr_gen_cs  <= HDR_GEN;
                                    payload_len <= rem_len;
                                    rem_len     <= 32'd0;
                                end else if ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~aeth_pkt) begin
                                    //hdr_gen_cs <= IDLE;
                                    hdr_gen_cs              <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? IDLE : DROP_WQE;
                                    //push_to_os_buf <= (i_qp_fatal[o_qp_id] || i_qp_recovery) ? 1'b1 : 1'b0;
                                    o_wqe_dropped           <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b0 : 1'b1;
                                    o_wqe_dropped_qpid      <= o_qp_id;
                                    push_to_os_buf          <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b1 : 1'b0;
                                    o_reg_rd_en             <= 1'b0;
                                    first_trans             <= 1'b0;
                                    completion_data_present <= 1'b0;
                                    //o_osq_nacked_resp <= i_osq_nacked;
                                    //wqe_pushed_in_fatal <= i_qp_fatal[o_qp_id] || i_qp_recovery;
                                    wqe_pushed_in_fatal     <= (qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1);
                                end else if ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) && (aeth_pkt && ~(aeth_hdr[30:29] == 2'b11 && |aeth_hdr[26:24]))) begin
                                    hdr_gen_cs    <= IDLE;
                                    o_reg_rd_en   <= 1'b0;
                                    first_trans   <= 1'b0;
                                    o_hdr_gen_rdy <= 1'b1;
                                    aeth_pkt      <= 1'b0;
                                end else if (~(wqe_opcode == 8'h00 || wqe_opcode == 8'h02 || wqe_opcode == 8'h04 || aeth_pkt)) begin
                                    hdr_gen_cs       <= ILL_OPCODE;
                                    o_wqe_fatal_qpid <= o_qp_id;
                                end else begin
                                    hdr_gen_cs <= WAIT0;
                                end
                                exp_ack              <= i_reg_exp_ack;
                                o_reg_rd_en_s        <= 1'b0;
                                dest_qpid            <= i_reg_dest_qpid;
                                mac_dest_addr        <= i_reg_mac_dest_addr;
                                ip_dest_addr         <= i_reg_ip_dest_addr;
                                ip_ver               <= i_reg_ip_ver;
                                o_reg_sq_psn         <= i_reg_sq_psn;
                                o_wqe_send_first_psn <= partial_wqe ? partial_wqe_start_psn : i_reg_sq_psn;
                                o_reg_sq_msn         <= i_reg_sq_msn + 1'b1;
                                reg_qp_to            <= i_reg_qp_to;
                            end
                            wqe_os_fifo_full <= (os_num_vld_entries >= OS_TH_ENTRIES);

                            case (i_reg_pmtu)
                                3'b000: begin
                                    max_pmtu <= 13'h0100;
                                end
                                3'b001: begin
                                    max_pmtu <= 13'h0200;
                                end
                                3'b010: begin
                                    max_pmtu <= 13'h0400;
                                end
                                3'b011: begin
                                    max_pmtu <= 13'h0800;
                                end
                                3'b100: begin
                                    max_pmtu <= 13'h1000;
                                end
                                default: begin
                                    max_pmtu <= 13'h100;
                                end
                            endcase
                        end

                        WAIT0: begin
                            get_data_buf          <= 1'b0;
                            o_data_buf_fifo_wr_en <= 1'b0;
                            push_to_os_buf        <= 1'b0;
                            if (i_reg_psn_val) begin
                                o_reg_wr_en <= 1'b0;
                            end
                            if (ip_ver && ~o_reg_wr_en) begin
                                hdr_gen_cs <= HDR_GEN;
                            end else if (~ip_ver && ~o_reg_wr_en) begin
                                hdr_gen_cs          <= IP_CHKSUM;
                                ipv4_identification <= ipv4_identification + 1'b1;
                                ipv4_len_cal        <= 1'b0;
                            end

                            if (~o_reg_wr_en) begin
                                if (aeth_pkt) begin
                                    payload_len  <= 32'd0;
                                    rem_len      <= 32'd0;
                                    reth_present <= 1'b0;
                                end else if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (wqe_opcode == 8'h00)) begin
                                    payload_len  <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                                    rem_len      <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                                    reth_present <= first_trans && ~partial_wqe;
                                end else if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (wqe_opcode == 8'h02)) begin
                                    payload_len  <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                                    rem_len      <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                                    reth_present <= 1'b0;
                                end else if (wqe_opcode == 8'h04) begin
                                    payload_len  <= 32'd0;
                                    rem_len      <= rem_len;
                                    reth_present <= 1'b1;
                                end else begin
                                    payload_len  <= rem_len;
                                    rem_len      <= 32'd0;
                                    reth_present <= 1'b0;
                                end
                            end
                        end
                        IP_CHKSUM: begin

                            if (~ipv4_len_cal) begin
                                ipv4_len_cal           <= 1'b1;
                                ip_hdr_chksum_add_pipe <= 16'h4501 + ipv4_identification[15:0] + 16'h4000 + ip_dest_addr[31:16] + ip_dest_addr[15:0];
                                ipv4_len               <= 8'h2C + payload_len + {reth_present, 1'b0, aeth_pkt, 2'b00};
                            end else if (~chksum_cal_start) begin
                                ip_hdr_chksum    <= ip_hdr_chksum_add_pipe + ipv4_len[15:0] + {i_reg_ttl, 8'h11} + i_reg_ipv4_src_addr[31:16] + i_reg_ipv4_src_addr[15:0];
                                //ip_hdr_chksum <= 16'h4501 +  ipv4_len[15:0] + ipv4_identification[15:0] + 16'h4000 + { i_reg_ttl,8'h11} + 
                                //                 i_reg_ipv4_src_addr[31:16] + i_reg_ipv4_src_addr[15:0] + 
                                //                 ip_dest_addr[31:16] + ip_dest_addr[15:0];
                                chksum_cal_start <= 1'b1;
                            end else begin
                                if (|ip_hdr_chksum[19:16]) begin
                                    ip_hdr_chksum <= ip_hdr_chksum[15:0] + ip_hdr_chksum[19:16];
                                end else begin
                                    chksum_cal_start <= 1'b0;
                                    hdr_gen_cs       <= HDR_GEN;
                                end
                            end
                        end
                        HDR_GEN: begin
                            if (~no_data_buf_left && ~buf_mgr_busy && ((wqe_opcode[7:1] != 7'h00) || (~retried || i_data_buf_fifo_empty))) begin
                                wqe_opcode_err <= 1'b0;
                                eth_hdr <= {
                                    (ip_ver ? 16'hDD86 : 16'h0008),
                                    i_reg_mac_src_addr[7:0],
                                    i_reg_mac_src_addr[15:8],
                                    i_reg_mac_src_addr[23:16],
                                    i_reg_mac_src_addr[31:24],
                                    i_reg_mac_src_addr[39:32],
                                    i_reg_mac_src_addr[47:40],
                                    mac_dest_addr[7:0],
                                    mac_dest_addr[15:8],
                                    mac_dest_addr[23:16],
                                    mac_dest_addr[31:24],
                                    mac_dest_addr[39:32],
                                    mac_dest_addr[47:40]
                                };
                                if (ip_ver) begin
                                    ip_hdr <= {
                                        ip_dest_addr[7:0],
                                        ip_dest_addr[15:8],
                                        ip_dest_addr[23:16],
                                        ip_dest_addr[31:24],
                                        ip_dest_addr[39:32],
                                        ip_dest_addr[47:40],
                                        ip_dest_addr[55:48],
                                        ip_dest_addr[63:56],
                                        ip_dest_addr[71:64],
                                        ip_dest_addr[79:72],
                                        ip_dest_addr[87:80],
                                        ip_dest_addr[95:88],
                                        ip_dest_addr[103:96],
                                        ip_dest_addr[111:104],
                                        ip_dest_addr[119:112],
                                        ip_dest_addr[127:120],
                                        i_reg_ipv6_src_addr[7:0],
                                        i_reg_ipv6_src_addr[15:8],
                                        i_reg_ipv6_src_addr[23:16],
                                        i_reg_ipv6_src_addr[31:24],
                                        i_reg_ipv6_src_addr[39:32],
                                        i_reg_ipv6_src_addr[47:40],
                                        i_reg_ipv6_src_addr[55:48],
                                        i_reg_ipv6_src_addr[63:56],
                                        i_reg_ipv6_src_addr[71:64],
                                        i_reg_ipv6_src_addr[79:72],
                                        i_reg_ipv6_src_addr[87:80],
                                        i_reg_ipv6_src_addr[95:88],
                                        i_reg_ipv6_src_addr[103:96],
                                        i_reg_ipv6_src_addr[111:104],
                                        i_reg_ipv6_src_addr[119:112],
                                        i_reg_ipv6_src_addr[127:120],
                                        i_reg_ttl,
                                        8'h11,
                                        ipv6_len[7:0],
                                        ipv6_len[15:8],
                                        q_id[7:0],
                                        q_id[15:8],
                                        {i_reg_dscp[1:0], 2'b01, 4'h0},
                                        {4'h6, i_reg_dscp[5:2]}
                                    };
                                end else begin
                                    ip_hdr[159:96] <= {ip_dest_addr[7:0], ip_dest_addr[15:8], ip_dest_addr[23:16], ip_dest_addr[31:24], i_reg_ipv4_src_addr[7:0], i_reg_ipv4_src_addr[15:8], i_reg_ipv4_src_addr[23:16], i_reg_ipv4_src_addr[31:24]};
                                    ip_hdr[95:80]  <= {~ip_hdr_chksum[7:0], ~ip_hdr_chksum[15:8]};
                                    ip_hdr[79:0]   <= {8'h11, i_reg_ttl, 16'h0040, ipv4_identification[7:0], ipv4_identification[15:8], ipv4_len[7:0], ipv4_len[15:8], 16'h0145};
                                end
                                udp_hdr <= {16'h0000, ipv6_len[7:0], ipv6_len[15:8], 16'hB712, i_reg_udp_src_port[7:0], i_reg_udp_src_port[15:8]};  //Need to check that UDP destination port is decimal or hexa, considering it as decimal 4791 for now
                                if (aeth_pkt) begin
                                    bth_hdr[95:72] <= {i_aeth_psn[7:0], i_aeth_psn[15:8], i_aeth_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end else begin
                                    //bth_hdr[95:32] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16], (exp_ack || wqe_os_fifo_full), 7'h00,  dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                    bth_hdr[95:72] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end
                                bth_hdr[31:16] <= {i_reg_pkey[7:0], i_reg_pkey[15:8]};
                                bth_hdr[14:8]  <= {1'b0, 2'b00, 4'h0};
                                //SE Bit chnage if((|rem_len && (wqe_opcode == 8'h00 || wqe_opcode == 8'h02)) || aeth_pkt) begin
                                if (~|rem_len && wqe_opcode == 8'h02) begin
                                    bth_hdr[15] <= 1'b1;
                                end else begin
                                    bth_hdr[15] <= 1'b0;
                                end
                                if (aeth_pkt) begin
                                    bth_hdr[7:0] <= 8'h11;
                                    bth_hdr[71]  <= 1'b0;
                                end else if (wqe_opcode == 8'h00) begin
                                    o_out_rw_pkt_cnt <= o_out_rw_pkt_cnt + 1'b1;
                                    bth_hdr[71]      <= exp_ack || (~|rem_len);
                                    if (PSN_based_retransmission == 1 && partial_wqe == 1'b1) begin
                                        if (|rem_len) begin
                                            bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                            push_to_os_buf <= first_trans;
                                        end else begin
                                            bth_hdr[7:0]   <= 8'h08;  //WRITE_LAST
                                            push_to_os_buf <= first_trans;
                                        end
                                    end else if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h06;  //WRITE_FIRST
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h0A;  //WRITE_ONLY
                                        push_to_os_buf <= 1'b1;
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h08;  //WRITE_LAST
                                        push_to_os_buf <= 1'b0;
                                    end
                                end else if (wqe_opcode == 8'h04) begin
                                    o_out_rw_pkt_cnt <= o_out_rw_pkt_cnt + 1'b1;
                                    bth_hdr[7:0]     <= 8'h0C;  //RDMA_READ
                                    bth_hdr[71]      <= 1'b1;
                                    push_to_os_buf   <= 1'b1;
                                end else if (wqe_opcode == 8'h02) begin
                                    o_out_send_pkt_cnt <= o_out_send_pkt_cnt + ((q_id != 16'h0000) ? 1'b1 : 1'b0);
                                    bth_hdr[71]        <= exp_ack || (~|rem_len);
                                    if (PSN_based_retransmission == 1 && partial_wqe == 1'b1) begin
                                        if (|rem_len) begin
                                            bth_hdr[7:0]   <= 8'h01;  //SEND_MIDDLE
                                            push_to_os_buf <= first_trans;
                                        end else begin
                                            bth_hdr[7:0]   <= 8'h02;  //SEND_LAST
                                            push_to_os_buf <= first_trans;
                                        end
                                    end else if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h00;  //SEND_FIRST
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h04;  //SEND_ONLY
                                        push_to_os_buf <= 1'b1;
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h01;  //SEND_MIDDLE
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h02;  //SEND_LAST
                                        push_to_os_buf <= 1'b0;
                                    end
                                end else begin
                                    bth_hdr[7:0]   <= 8'h04;
                                    bth_hdr[71]    <= 1'b0;
                                    push_to_os_buf <= 1'b0;
                                end
                                if (wqe_opcode == 8'h04) begin
                                    case (max_pmtu[12:8])
                                        5'h01: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:8] + |(rem_len[7:0]);
                                        end
                                        5'h02: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:9] + |(rem_len[8:0]);
                                        end
                                        5'h04: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:10] + |(rem_len[9:0]);
                                        end
                                        5'h08: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:11] + |(rem_len[10:0]);
                                        end
                                        5'h10: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:12] + |(rem_len[11:0]);
                                        end
                                        default: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:8] + |(rem_len[8:0]);
                                        end
                                    endcase
                                end else begin
                                    o_reg_sq_psn <= o_reg_sq_psn + (aeth_pkt ? 1'b0 : 1'b1);
                                end
                                o_out_ack_pkt_cnt <= o_out_ack_pkt_cnt + (aeth_pkt ? 1'b1 : 1'b0);
                                o_out_mad_pkt_cnt <= o_out_mad_pkt_cnt + (q_id == 16'h0000 ? 1'b1 : 1'b0);
                                local_offset <= local_offset + payload_len;
                                remote_offset <= remote_offset + payload_len;
                                reth_hdr <= {
                                    dma_len[7:0],
                                    dma_len[15:8],
                                    dma_len[23:16],
                                    dma_len[31:24],
                                    r_key[7:0],
                                    r_key[15:8],
                                    r_key[23:16],
                                    r_key[31:24],
                                    remote_offset[7:0],
                                    remote_offset[15:8],
                                    remote_offset[23:16],
                                    remote_offset[31:24],
                                    remote_offset[39:32],
                                    remote_offset[47:40],
                                    remote_offset[55:48],
                                    remote_offset[63:56]
                                };
                                o_hdr_data[1023:928] <= {payload_len, local_offset, completion_data_present, q_id[14:0]};
                                o_hdr_gen_rdy <= aeth_pkt;
                                if (aeth_pkt || (~(i_wqe_halt && (i_wqe_halted_qpid == o_qp_id))  /* && ~(i_osq_nacked[o_qp_id])*/)) begin
                                    hdr_gen_cs <= HDR_STO;
                                    //if(wqe_opcode == 8'h00 && first_trans && ~aeth_pkt) begin
                                    //    get_data_buf <= ~retried;
                                    //end else begin
                                    //    get_data_buf <= 1'b0;
                                    //end
                                end else begin
                                    //hdr_gen_cs <= IDLE;
                                    hdr_gen_cs          <= DROP_WQE;
                                    o_wqe_dropped       <= first_trans;
                                    o_wqe_dropped_qpid  <= o_qp_id;
                                    //ipv4_identification <= ipv4_identification - 1'b1;
                                    ipv4_identification <= ipv4_identification - (ip_ver ? 1'b0 : 1'b1);
                                end
                            end
                        end
                        HDR_STO: begin
                            //get_data_buf <= 1'b0;
                            o_data_buf_fifo_data <= {data_buf_addr[C_M_AXI_ADDR_WIDTH-1:2], partial_wqe, (retried || (q_id == 16'h0000))};
                            o_hdr_gen_rdy        <= 1'b0;
                            if (~wqe_push_pending) begin
                                o_hdr_data[111:0] <= eth_hdr;
                                if (ip_ver) begin
                                    o_hdr_data[431:112] <= ip_hdr;
                                    o_hdr_data[495:432] <= udp_hdr;
                                    o_hdr_data[591:496] <= bth_hdr;
                                    if (reth_present) begin
                                        o_hdr_data[719:592] <= reth_hdr;
                                        o_hdr_len           <= 8'h5A;  //90B header
                                    end else if (aeth_pkt) begin
                                        o_hdr_data[623:592] <= {aeth_hdr[7:0], aeth_hdr[15:8], aeth_hdr[23:16], aeth_hdr[31:24]};
                                        o_hdr_len           <= 8'h4E;  //78B header
                                    end else if (completion_data_present) begin
                                        o_hdr_data[719:592] <= nvme_comp_data;
                                        o_hdr_len           <= 8'h5A;  //90B header
                                    end else begin
                                        o_hdr_data[719:592] <= 128'd0000;
                                        o_hdr_len           <= 8'h4A;  //74B header
                                    end
                                end else begin
                                    o_hdr_data[271:112] <= ip_hdr[159:0];
                                    o_hdr_data[335:272] <= udp_hdr;
                                    o_hdr_data[431:336] <= bth_hdr;
                                    if (reth_present) begin
                                        o_hdr_data[559:432] <= reth_hdr;
                                        o_hdr_len           <= 8'h46;  //70B header
                                    end else if (aeth_pkt) begin
                                        o_hdr_data[463:432] <= {aeth_hdr[7:0], aeth_hdr[15:8], aeth_hdr[23:16], aeth_hdr[31:24]};
                                        o_hdr_len           <= 8'h3A;  //58B header
                                    end else if (completion_data_present) begin
                                        o_hdr_data[559:432] <= nvme_comp_data;
                                        o_hdr_len           <= 8'h46;  //70B header
                                    end else begin
                                        o_hdr_data[559:432] <= 128'd0000;
                                        o_hdr_len           <= 8'h36;  //54B header
                                    end
                                end
                                //if(wqe_opcode == 8'h00 && first_trans && ~aeth_pkt && ~partial_wqe) begin
                                if (wqe_opcode == 8'h00 && first_trans && ~aeth_pkt) begin
                                    o_data_buf_fifo_wr_en <= 1'b1;
                                    get_data_buf          <= ~retried;
                                end else begin
                                    o_data_buf_fifo_wr_en <= 1'b0;
                                    get_data_buf          <= 1'b0;
                                end
                                push_to_os_buf <= 1'b0;
                                first_trans    <= 1'b0;
                                o_reg_wr_en    <= ~aeth_pkt;
                                if (wqe_opcode == 8'h00 && |rem_len) begin
                                    hdr_gen_cs <= WAIT0;
                                end else if (wqe_opcode == 8'h02 && |rem_len) begin
                                    hdr_gen_cs <= WAIT0;
                                end else begin
                                    hdr_gen_cs  <= IDLE;
                                    o_reg_rd_en <= 1'b0;
                                end
                                aeth_pkt     <= 1'b0;
                                reth_present <= 1'b0;
                            end
                        end
                        ILL_OPCODE: begin
                            o_hdr_gen_rdy <= 1'b0;
                            if (~o_wqe_fatal_qp_vld) begin
                                //o_hdr_data[1023:512] <= 512'd0;
                                //o_hdr_data[511:0] <= {128'd0,nvme_comp_data,16'h0000,q_id,r_key,remote_offset,wqe_opcode,dma_len,local_offset,o_wqe_wrid};
                                //o_hdr_len <= 8'h00; 
                                hdr_gen_cs     <= IDLE;
                                o_reg_rd_en    <= 1'b0;
                                o_reg_wr_en    <= ~aeth_pkt;
                                aeth_pkt       <= 1'b0;
                                wqe_opcode_err <= 1'b1;
                                push_to_os_buf <= 1'b1;
                            end
                        end
                        DROP_WQE: begin
                            //if(~(i_osq_pop && (i_osq_pop_qpid == o_qp_id))) begin
                            hdr_gen_cs    <= IDLE;
                            o_wqe_dropped <= 1'b0;
                            //end
                        end
                        default: begin
                        end
                    endcase
                end
            end

        end
    endgenerate  // end of C_EN_RECV_RD_WR == 0 && NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 0



    generate
        if (C_EN_RECV_RD_WR == 1'b0) always @(posedge core_clk) o_out_rd_resp_pkt_cnt <= 0;
    endgenerate



    generate
        if (C_EN_RECV_RD_WR == 1 && NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 1'b0) begin : RD_RESP_ONLY

            reg        rd_resp;
            reg [ 1:0] rd_resp_start_with_middle;
            reg [23:0] rd_resp_psn;
            reg        rd_resp_aeth_present;

            assign ipv6_len = (8'h18 + payload_len + {reth_present, 1'b0, (aeth_pkt || rd_resp_aeth_present), 2'b00});
            //assign ipv4_len = (8'h2C + payload_len + {reth_present,1'b0,(aeth_pkt || rd_resp_aeth_present),2'b00});

            always @(posedge core_clk) begin
                if (core_rst) begin
                    eth_hdr                           <= 'd0;
                    ip_hdr                            <= 'd0;
                    udp_hdr                           <= 'd0;
                    bth_hdr                           <= 'd0;
                    reth_hdr                          <= 'd0;
                    aeth_hdr                          <= 'd0;
                    o_hdr_data                        <= 'd0;
                    o_hdr_valid                       <= 1'b0;
                    o_hdr_ip_ver                      <= 1'b0;
                    o_hdr_len                         <= 8'h00;
                    o_qpm_wqe_pop                     <= 1'b0;
                    o_qp_id                           <= {C_MAX_QID_WIDTH{1'b0}};
                    o_reg_rd_en                       <= 1'b0;
                    o_reg_rd_en_s                     <= 1'b0;
                    hdr_gen_cs                        <= IDLE;
                    ip_hdr_chksum                     <= 20'd0;
                    rem_len                           <= 32'd0;
                    dma_len                           <= 32'd0;
                    first_trans                       <= 1'b0;
                    payload_len                       <= 16'h0000;
                    reth_present                      <= 1'b0;
                    remote_offset                     <= 64'd00;
                    local_offset                      <= 64'd00;
                    aeth_pkt                          <= 1'b0;
                    wqe_opcode                        <= 8'h00;
                    r_key                             <= 32'd0;
                    chksum_cal_start                  <= 1'b0;
                    nvme_comp_data                    <= 128'd0000;
                    completion_data_present           <= 1'b0;
                    o_reg_wr_en                       <= 1'b0;
                    wqe_opcode_err                    <= 1'b0;
                    err_opcode                        <= 8'h00;
                    dest_qpid                         <= 24'd0;
                    o_hdr_gen_rdy                     <= 1'b0;
                    o_reg_sq_psn                      <= 24'd0;
                    o_wqe_send_first_psn              <= 24'd0;
                    o_reg_sq_msn                      <= 24'd0;
                    o_wqe_wrid                        <= 16'h0000;
                    mac_dest_addr                     <= 'd0;
                    ip_dest_addr                      <= 'd0;
                    ip_ver                            <= 1'b0;
                    exp_ack                           <= 1'b0;
                    o_out_send_pkt_cnt                <= 16'h0000;
                    o_out_rw_pkt_cnt                  <= 16'h0000;
                    o_out_ack_pkt_cnt                 <= 16'h0000;
                    o_out_mad_pkt_cnt                 <= 16'h0000;
                    o_wqe_halted                      <= 1'b0;
                    wqe_os_fifo_full                  <= 1'b0;
                    ipv4_identification               <= 16'h0000;
                    retried                           <= 1'b0;
                    get_data_buf                      <= 1'b0;
                    o_data_buf_fifo_data              <= 32'd0;
                    o_data_buf_fifo_wr_en             <= 1'b0;
                    //o_osq_nacked_resp <= 'd0;
                    push_to_os_buf                    <= 1'b0;
                    buf_mgr_busy                      <= 1'b0;
                    reg_qp_to                         <= 5'h00;
                    wait_for_dma_idle                 <= 1'b0;
                    //qp_fatal_detected<= 'h0;
                    retried_buf_id                    <= 'h0;
                    wqe_pushed_in_fatal               <= 1'b0;
                    o_wqe_fatal_qpid                  <= {C_MAX_QID_WIDTH{1'b0}};
                    o_wqe_fatal_qp_vld                <= 1'b0;
                    rd_resp                           <= 1'b0;
                    rd_resp_start_with_middle         <= 2'b00;
                    rd_resp_psn                       <= 24'd0;
                    rd_resp_aeth_present              <= 1'b0;
                    o_out_rd_resp_pkt_cnt             <= 16'h0000;
                    o_wqe_dropped                     <= 1'b0;
                    o_wqe_dropped_qpid                <= {C_MAX_QID_WIDTH{1'b0}};
                    ipv4_len_cal                      <= 1'b0;
                    ip_hdr_chksum_add_pipe            <= 17'd0;
                    ipv4_len                          <= 16'd0;
                    partial_wqe                       <= 1'b0;
                    partial_wqe_start_psn             <= 24'd0;
                    qp_fatal_detected_wr_en           <= 1'b0;
                    qp_fatal_detected_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_fatal_detected_wr_data         <= 1'b0;
                    qp_fatal_detected_wr_en_aux       <= 1'b0;
                    qp_fatal_detected_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_fatal_detected_wr_data_aux     <= 1'b0;
                    qp_wqe_opcode_fatal_wr_en         <= 1'b0;
                    qp_wqe_opcode_fatal_wr_addr       <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_wqe_opcode_fatal_wr_data       <= 1'b0;
                    qp_wqe_opcode_fatal_wr_en_aux     <= 1'b0;
                    qp_wqe_opcode_fatal_wr_addr_aux   <= {C_MAX_QID_WIDTH{1'b0}};
                    qp_wqe_opcode_fatal_wr_data_aux   <= 1'b0;
                    o_osq_nacked_resp_wr_en           <= 1'b0;
                    o_osq_nacked_resp_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data         <= 1'b0;
                    o_osq_nacked_resp_wr_en_aux       <= 1'b0;
                    o_osq_nacked_resp_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data_aux     <= 1'b0;
                    o_osq_nacked_resp_wr_en_pending   <= 1'b0;
                    o_osq_nacked_resp_wr_addr_pending <= {C_MAX_QID_WIDTH{1'b0}};
                    o_osq_nacked_resp_wr_data_pending <= 1'b0;
                    osq_nacked_wr_pending             <= 1'b0;
                    osq_nacked_wr_id                  <= {C_MAX_QID_WIDTH{1'b0}};
                    osq_nacked_wr_data                <= 1'b0;
                end else begin
                    //qp_fatal_detected<= (i_qp_fatal & (~i_osq_nacked));
                    /* if (i_qp_fatal_wr_en && i_osq_nacked_wr_en && i_qp_fatal_wr_addr == i_osq_nacked_wr_addr) begin
				qp_fatal_detected_wr_en <= 1'b1;
				qp_fatal_detected_wr_en_aux <= 1'b0;
				qp_fatal_detected_wr_addr <= i_qp_fatal_wr_addr;
				qp_fatal_detected_wr_data <= i_qp_fatal_wr_data & ~i_osq_nacked_wr_data;
			  end else begin
				if (i_qp_fatal_wr_en) begin
					qp_fatal_detected_wr_en <= 1'b1;
					qp_fatal_detected_wr_addr <= i_qp_fatal_wr_addr;
					qp_fatal_detected_wr_data <= i_qp_fatal_wr_data & ~osq_nacked_rd_data_0;
				end else begin
					qp_fatal_detected_wr_en <= 1'b0;
				end
				
				if (i_osq_nacked_wr_en) begin
					qp_fatal_detected_wr_en_aux <= 1'b1;
					qp_fatal_detected_wr_addr_aux <= i_osq_nacked_wr_addr;
					qp_fatal_detected_wr_data_aux <= qp_fatal_rd_data_0 & ~i_osq_nacked_wr_data;
				end else begin
					qp_fatal_detected_wr_en_aux <= 1'b0;
				end
			  end */
                    if (~wqe_push_pending && (hdr_gen_cs == HDR_STO  /*|| hdr_gen_cs == ILL_OPCODE*/)) begin
                        o_hdr_valid  <= 1'b1;
                        o_hdr_ip_ver <= ip_ver;
                    end else if (i_buf_mngr_rdy) begin
                        o_hdr_valid <= 1'b0;
                    end
                    if ((hdr_gen_cs == ILL_OPCODE) && ~o_wqe_fatal_qp_vld) begin
                        o_wqe_fatal_qp_vld <= 1'b1;
                    end else if (i_wqe_fatal_qp_rdy) begin
                        o_wqe_fatal_qp_vld <= 1'b0;
                    end
                    if (o_wqe_fatal_qp_vld && i_qp_fatal_wr_en && o_wqe_fatal_qpid == i_qp_fatal_wr_addr) begin
                        qp_wqe_opcode_fatal_wr_en     <= 1'b1;
                        qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                        qp_wqe_opcode_fatal_wr_addr   <= o_wqe_fatal_qpid;
                        qp_wqe_opcode_fatal_wr_data   <= 1'b1;
                    end else begin
                        if (o_wqe_fatal_qp_vld) begin
                            qp_wqe_opcode_fatal_wr_en   <= 1'b1;
                            qp_wqe_opcode_fatal_wr_addr <= o_wqe_fatal_qpid;
                            qp_wqe_opcode_fatal_wr_data <= 1'b1;
                        end else begin
                            qp_wqe_opcode_fatal_wr_en <= 1'b0;
                        end

                        if (i_qp_fatal_wr_en && i_qp_fatal_wr_data) begin
                            qp_wqe_opcode_fatal_wr_en_aux   <= 1'b1;
                            qp_wqe_opcode_fatal_wr_addr_aux <= i_qp_fatal_wr_addr;
                            qp_wqe_opcode_fatal_wr_data_aux <= 1'b0;
                        end else begin
                            qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                        end
                    end
                    if (o_hdr_valid && ~i_buf_mngr_rdy) begin
                        buf_mgr_busy <= 1'b1;
                    end else if (i_buf_mngr_rdy) begin
                        buf_mgr_busy <= 1'b0;
                    end
                    if (hdr_gen_cs == IDLE && i_wqe_halt && ~o_reg_wr_en) begin
                        o_wqe_halted <= 1'b1;
                    end else if (~i_wqe_halt) begin
                        o_wqe_halted <= 1'b0;
                    end
                    //if( |((i_qp_fatal & ~i_osq_nacked )& ~qp_fatal_detected)) begin
                    /* if (i_qp_fatal_wr_en && i_osq_nacked_wr_en && i_qp_fatal_wr_addr == i_osq_nacked_wr_addr && (i_qp_fatal_wr_data && ~i_osq_nacked_wr_data) && ~qp_fatal_detected_rd_data_0) begin
				 wait_for_dma_idle <= 1'b1;
			  end else if ((i_qp_fatal_wr_en && (i_qp_fatal_wr_data && ~osq_nacked_rd_data_0) && ~qp_fatal_detected_rd_data_0) || (i_osq_nacked_wr_en && (qp_fatal_rd_data_0 && ~i_osq_nacked_wr_data) && ~qp_fatal_detected_rd_data_1)) begin	 
				 wait_for_dma_idle <= 1'b1;
              end else if(dma_in_idle && ~o_hdr_valid && (hdr_gen_cs == IDLE)) begin
				wait_for_dma_idle <= 1'b0;
              end */

                    if (i_osq_nacked_wr_en && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else if (i_osq_nacked_wr_en && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != 0 && ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~(rd_resp || aeth_pkt))) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else if (i_osq_nacked_wr_en && i_osq_nacked_wr_addr != o_qp_id) begin
                        o_osq_nacked_resp_wr_en   <= 1'b1;
                        o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                        o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
                    end else begin
                        o_osq_nacked_resp_wr_en <= 1'b0;
                    end

                    if (i_osq_nacked_wr_en_aux && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else if (i_osq_nacked_wr_en_aux && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != 0 && ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~(rd_resp || aeth_pkt))) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else if (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux != o_qp_id) begin
                        o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                        o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
                    end else begin
                        o_osq_nacked_resp_wr_en_aux <= 1'b0;
                    end

                    if ((i_osq_nacked_wr_en && i_osq_nacked_wr_addr == o_qp_id) || (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux == o_qp_id) && hdr_gen_cs != IDLE && hdr_gen_cs != RD_WQE &&
			  ~(hdr_gen_cs == RD_Q_INFO && (i_reg_info_val && o_qp_id != 0 && 
			  ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data)  && ~(rd_resp || aeth_pkt))))) begin
                        osq_nacked_wr_pending <= 1'b1;
                        osq_nacked_wr_id      <= i_osq_nacked_wr_en ? i_osq_nacked_wr_addr : i_osq_nacked_wr_addr_aux;
                        osq_nacked_wr_data    <= i_osq_nacked_wr_en ? i_osq_nacked_wr_data : i_osq_nacked_wr_data_aux;
                    end else if (hdr_gen_cs == IDLE) begin
                        osq_nacked_wr_pending <= 1'b0;
                    end

                    if (hdr_gen_cs == IDLE && osq_nacked_wr_pending) begin
                        o_osq_nacked_resp_wr_en_pending   <= 1'b1;
                        o_osq_nacked_resp_wr_addr_pending <= osq_nacked_wr_id;
                        o_osq_nacked_resp_wr_data_pending <= osq_nacked_wr_data;
                    end else begin
                        o_osq_nacked_resp_wr_en_pending <= 1'b0;
                    end

                    case (hdr_gen_cs)
                        IDLE: begin
                            push_to_os_buf          <= 1'b0;
                            retried                 <= 1'b0;
                            get_data_buf            <= 1'b0;
                            o_data_buf_fifo_wr_en   <= 1'b0;
                            o_hdr_gen_rdy           <= 1'b0;
                            //o_osq_nacked_resp <= i_osq_nacked;
                            wqe_pushed_in_fatal     <= 1'b0;
                            completion_data_present <= 1'b0;
                            partial_wqe             <= 1'b0;
                            if (i_reg_psn_val) begin
                                o_reg_wr_en <= 1'b0;
                            end
                            if ((i_aeth_valid) && ~o_hdr_gen_rdy && ~o_reg_wr_en) begin
                                hdr_gen_cs    <= RD_Q_INFO;
                                o_qp_id       <= i_aeth_qpid;
                                o_reg_rd_en   <= 1'b1;
                                o_reg_rd_en_s <= 1'b1;
                                aeth_pkt      <= 1'b1;
                                aeth_hdr      <= i_aeth_hdr;
                            end else if (i_rd_req_valid && ~o_hdr_gen_rdy && ~o_reg_wr_en) begin
                                hdr_gen_cs                <= RD_Q_INFO;
                                o_qp_id                   <= i_aeth_qpid;
                                o_reg_rd_en               <= 1'b1;
                                o_reg_rd_en_s             <= 1'b1;
                                rd_resp                   <= 1'b1;
                                rd_resp_start_with_middle <= i_rd_req_start_with_middle;
                                rd_resp_psn               <= i_aeth_psn;
                                aeth_hdr                  <= i_aeth_hdr;
                                dma_len                   <= i_rd_req_len;
                                rem_len                   <= i_rd_req_len;
                                local_offset              <= i_rd_req_addr;
                                first_trans               <= 1'b1;
                                wqe_opcode                <= 8'h00;
                            end else if (~i_qpm_fifo_empty && ~o_reg_wr_en && ~i_wqe_halt && ~wait_for_dma_idle) begin
                                o_qpm_wqe_pop <= 1'b1;
                                hdr_gen_cs    <= RD_WQE;
                            end else begin
                                o_reg_rd_en   <= 1'b0;
                                o_reg_rd_en_s <= 1'b0;
                            end
                        end
                        RD_WQE: begin
                            o_qpm_wqe_pop <= 1'b0;
                            if (o_qpm_wqe_pop == 1'b0) begin
                                o_wqe_wrid     <= i_qpm_wqe_data[15:0];
                                o_qp_id        <= i_qpm_wqe_data[136+C_MAX_QID_WIDTH-1 : 136];
                                dma_len        <= i_qpm_wqe_data[127:96];
                                rem_len        <= i_qpm_wqe_data[127:96];
                                wqe_opcode     <= i_qpm_wqe_data[135:128];
                                local_offset   <= i_qpm_wqe_data[95:32];
                                remote_offset  <= i_qpm_wqe_data[223:160];
                                r_key          <= i_qpm_wqe_data[255:224];
                                nvme_comp_data <= i_qpm_wqe_data[129] ? i_qpm_wqe_data[383:256] : 128'd0000;
                                if (PSN_based_retransmission == 1) begin
                                    completion_data_present <= (i_qpm_wqe_data[130:128] == 3'b010) && ~(i_qpm_wqe_data[127:96] > 32'h00000010) && ~i_qpm_wqe_data[partial_wqe_bit_pos];
                                end else begin
                                    completion_data_present <= (i_qpm_wqe_data[130:128] == 3'b010) && ~(i_qpm_wqe_data[127:96] > 32'h00000010);
                                end
                                o_reg_rd_en    <= 1'b1;
                                o_reg_rd_en_s  <= 1'b1;
                                hdr_gen_cs     <= RD_Q_INFO;
                                first_trans    <= 1'b1;
                                retried_buf_id <= i_qpm_wqe_data[17+:DATA_BUF_ID_WIDTH];
                                retried        <= i_qpm_wqe_data[16];
                                if (PSN_based_retransmission == 1) begin
                                    partial_wqe           <= i_qpm_wqe_data[partial_wqe_bit_pos];
                                    partial_wqe_start_psn <= i_qpm_wqe_data[439:416];
                                end else begin
                                    partial_wqe <= 1'b0;
                                end
                            end
                        end
                        RD_Q_INFO: begin
                            if (i_reg_info_val) begin
                                if (o_qp_id == 0) begin
                                    hdr_gen_cs  <= HDR_GEN;
                                    payload_len <= rem_len;
                                    rem_len     <= 32'd0;
                                end else if ((osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) && ~(rd_resp || aeth_pkt)) begin
                                    //hdr_gen_cs <= IDLE;
                                    hdr_gen_cs              <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? IDLE : DROP_WQE;
                                    o_wqe_dropped           <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b0 : 1'b1;
                                    o_wqe_dropped_qpid      <= o_qp_id;
                                    //push_to_os_buf <= (i_qp_fatal[o_qp_id] || i_qp_recovery) ? 1'b1 : 1'b0;
                                    push_to_os_buf          <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b1 : 1'b0;
                                    o_reg_rd_en             <= 1'b0;
                                    first_trans             <= 1'b0;
                                    completion_data_present <= 1'b0;
                                    //o_osq_nacked_resp <= i_osq_nacked;
                                    //wqe_pushed_in_fatal <= i_qp_fatal[o_qp_id] || i_qp_recovery;
                                    wqe_pushed_in_fatal     <= (qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1);
                                    //end else if ((i_qp_fatal[o_qp_id] || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) && ((rd_resp || aeth_pkt) && ~(aeth_hdr[30:29] == 2'b11 && |aeth_hdr[26:24]))) begin
                                end else if ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) && (aeth_pkt && ~(aeth_hdr[30:29] == 2'b11 && |aeth_hdr[26:24]))) begin
                                    hdr_gen_cs                <= IDLE;
                                    o_reg_rd_en               <= 1'b0;
                                    first_trans               <= 1'b0;
                                    o_hdr_gen_rdy             <= 1'b1;
                                    aeth_pkt                  <= 1'b0;
                                    rd_resp                   <= 1'b0;
                                    rd_resp_start_with_middle <= 2'b00;
                                end else if (~(wqe_opcode == 8'h00 || wqe_opcode == 8'h02 || wqe_opcode == 8'h04 || aeth_pkt)) begin
                                    hdr_gen_cs       <= ILL_OPCODE;
                                    o_wqe_fatal_qpid <= o_qp_id;
                                end else begin
                                    hdr_gen_cs <= WAIT0;
                                end
                                exp_ack              <= i_reg_exp_ack;
                                o_reg_rd_en_s        <= 1'b0;
                                dest_qpid            <= i_reg_dest_qpid;
                                mac_dest_addr        <= i_reg_mac_dest_addr;
                                ip_dest_addr         <= i_reg_ip_dest_addr;
                                ip_ver               <= i_reg_ip_ver;
                                o_reg_sq_psn         <= i_reg_sq_psn;
                                o_wqe_send_first_psn <= partial_wqe ? partial_wqe_start_psn : i_reg_sq_psn;
                                o_reg_sq_msn         <= i_reg_sq_msn + 1'b1;
                                reg_qp_to            <= i_reg_qp_to;
                            end
                            wqe_os_fifo_full <= (os_num_vld_entries >= OS_TH_ENTRIES);

                            case (i_reg_pmtu)
                                3'b000: begin
                                    max_pmtu <= 13'h0100;
                                end
                                3'b001: begin
                                    max_pmtu <= 13'h0200;
                                end
                                3'b010: begin
                                    max_pmtu <= 13'h0400;
                                end
                                3'b011: begin
                                    max_pmtu <= 13'h0800;
                                end
                                3'b100: begin
                                    max_pmtu <= 13'h1000;
                                end
                                default: begin
                                    max_pmtu <= 13'h100;
                                end
                            endcase
                        end

                        WAIT0: begin
                            get_data_buf          <= 1'b0;
                            o_data_buf_fifo_wr_en <= 1'b0;
                            push_to_os_buf        <= 1'b0;
                            if (i_reg_psn_val) begin
                                o_reg_wr_en <= 1'b0;
                            end
                            if (ip_ver && ~o_reg_wr_en) begin
                                hdr_gen_cs <= HDR_GEN;
                            end else if (~ip_ver && ~o_reg_wr_en) begin
                                hdr_gen_cs          <= IP_CHKSUM;
                                ipv4_identification <= ipv4_identification + 1'b1;
                                ipv4_len_cal        <= 1'b0;
                            end

                            if (~o_reg_wr_en) begin
                                if (aeth_pkt) begin
                                    payload_len  <= 32'd0;
                                    rem_len      <= 32'd0;
                                    reth_present <= 1'b0;
                                end else if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (rd_resp || wqe_opcode == 8'h00)) begin
                                    payload_len          <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                                    rem_len              <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                                    reth_present         <= first_trans && ~rd_resp && ~partial_wqe;
                                    rd_resp_aeth_present <= first_trans && rd_resp && ~rd_resp_start_with_middle[0];
                                end else if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (wqe_opcode == 8'h02)) begin
                                    payload_len  <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                                    rem_len      <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                                    reth_present <= 1'b0;
                                end else if (wqe_opcode == 8'h04) begin
                                    payload_len  <= 32'd0;
                                    rem_len      <= rem_len;
                                    reth_present <= 1'b1;
                                end else begin
                                    payload_len          <= rem_len;
                                    rem_len              <= 32'd0;
                                    reth_present         <= 1'b0;
                                    rd_resp_aeth_present <= rd_resp;
                                end
                            end
                        end
                        IP_CHKSUM: begin

                            if (~ipv4_len_cal) begin
                                ipv4_len_cal           <= 1'b1;
                                ip_hdr_chksum_add_pipe <= 16'h4501 + ipv4_identification[15:0] + 16'h4000 + ip_dest_addr[31:16] + ip_dest_addr[15:0];
                                ipv4_len               <= 8'h2C + payload_len + {reth_present, 1'b0, (aeth_pkt || rd_resp_aeth_present), 2'b00};
                            end else if (~chksum_cal_start) begin
                                ip_hdr_chksum    <= ip_hdr_chksum_add_pipe + ipv4_len[15:0] + {i_reg_ttl, 8'h11} + i_reg_ipv4_src_addr[31:16] + i_reg_ipv4_src_addr[15:0];
                                //ip_hdr_chksum <= 16'h4501 +  ipv4_len[15:0] + ipv4_identification[15:0] + 16'h4000 + { i_reg_ttl,8'h11} + 
                                //                 i_reg_ipv4_src_addr[31:16] + i_reg_ipv4_src_addr[15:0] + 
                                //                 ip_dest_addr[31:16] + ip_dest_addr[15:0];
                                chksum_cal_start <= 1'b1;
                            end else begin
                                if (|ip_hdr_chksum[19:16]) begin
                                    ip_hdr_chksum <= ip_hdr_chksum[15:0] + ip_hdr_chksum[19:16];
                                end else begin
                                    chksum_cal_start <= 1'b0;
                                    hdr_gen_cs       <= HDR_GEN;
                                end
                            end
                        end
                        HDR_GEN: begin
                            if (~no_data_buf_left && ~buf_mgr_busy && ((wqe_opcode[7:1] != 7'h00 || rd_resp) || (~retried || i_data_buf_fifo_empty))) begin
                                wqe_opcode_err <= 1'b0;
                                eth_hdr <= {
                                    (ip_ver ? 16'hDD86 : 16'h0008),
                                    i_reg_mac_src_addr[7:0],
                                    i_reg_mac_src_addr[15:8],
                                    i_reg_mac_src_addr[23:16],
                                    i_reg_mac_src_addr[31:24],
                                    i_reg_mac_src_addr[39:32],
                                    i_reg_mac_src_addr[47:40],
                                    mac_dest_addr[7:0],
                                    mac_dest_addr[15:8],
                                    mac_dest_addr[23:16],
                                    mac_dest_addr[31:24],
                                    mac_dest_addr[39:32],
                                    mac_dest_addr[47:40]
                                };
                                if (ip_ver) begin
                                    ip_hdr <= {
                                        ip_dest_addr[7:0],
                                        ip_dest_addr[15:8],
                                        ip_dest_addr[23:16],
                                        ip_dest_addr[31:24],
                                        ip_dest_addr[39:32],
                                        ip_dest_addr[47:40],
                                        ip_dest_addr[55:48],
                                        ip_dest_addr[63:56],
                                        ip_dest_addr[71:64],
                                        ip_dest_addr[79:72],
                                        ip_dest_addr[87:80],
                                        ip_dest_addr[95:88],
                                        ip_dest_addr[103:96],
                                        ip_dest_addr[111:104],
                                        ip_dest_addr[119:112],
                                        ip_dest_addr[127:120],
                                        i_reg_ipv6_src_addr[7:0],
                                        i_reg_ipv6_src_addr[15:8],
                                        i_reg_ipv6_src_addr[23:16],
                                        i_reg_ipv6_src_addr[31:24],
                                        i_reg_ipv6_src_addr[39:32],
                                        i_reg_ipv6_src_addr[47:40],
                                        i_reg_ipv6_src_addr[55:48],
                                        i_reg_ipv6_src_addr[63:56],
                                        i_reg_ipv6_src_addr[71:64],
                                        i_reg_ipv6_src_addr[79:72],
                                        i_reg_ipv6_src_addr[87:80],
                                        i_reg_ipv6_src_addr[95:88],
                                        i_reg_ipv6_src_addr[103:96],
                                        i_reg_ipv6_src_addr[111:104],
                                        i_reg_ipv6_src_addr[119:112],
                                        i_reg_ipv6_src_addr[127:120],
                                        i_reg_ttl,
                                        8'h11,
                                        ipv6_len[7:0],
                                        ipv6_len[15:8],
                                        q_id[7:0],
                                        q_id[15:8],
                                        {i_reg_dscp[1:0], 2'b01, 4'h0},
                                        {4'h6, i_reg_dscp[5:2]}
                                    };
                                end else begin
                                    ip_hdr[159:96] <= {ip_dest_addr[7:0], ip_dest_addr[15:8], ip_dest_addr[23:16], ip_dest_addr[31:24], i_reg_ipv4_src_addr[7:0], i_reg_ipv4_src_addr[15:8], i_reg_ipv4_src_addr[23:16], i_reg_ipv4_src_addr[31:24]};
                                    ip_hdr[95:80]  <= {~ip_hdr_chksum[7:0], ~ip_hdr_chksum[15:8]};
                                    ip_hdr[79:0]   <= {8'h11, i_reg_ttl, 16'h0040, ipv4_identification[7:0], ipv4_identification[15:8], ipv4_len[7:0], ipv4_len[15:8], 16'h0145};
                                end
                                udp_hdr <= {16'h0000, ipv6_len[7:0], ipv6_len[15:8], 16'hB712, i_reg_udp_src_port[7:0], i_reg_udp_src_port[15:8]};  //Need to check that UDP destination port is decimal or hexa, considering it as decimal 4791 for now
                                if (aeth_pkt) begin
                                    bth_hdr[95:72] <= {i_aeth_psn[7:0], i_aeth_psn[15:8], i_aeth_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end else if (rd_resp) begin
                                    bth_hdr[95:72] <= {rd_resp_psn[7:0], rd_resp_psn[15:8], rd_resp_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end else begin
                                    //bth_hdr[95:32] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16], (exp_ack || wqe_os_fifo_full), 7'h00,  dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                    bth_hdr[95:72] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16]};
                                    bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};
                                end
                                bth_hdr[31:16] <= {i_reg_pkey[7:0], i_reg_pkey[15:8]};
                                bth_hdr[14:8]  <= {1'b0, 2'b00, 4'h0};
                                //SE Bit chnage if((|rem_len && (wqe_opcode == 8'h00 || wqe_opcode == 8'h02)) || aeth_pkt) begin
                                if (~|rem_len && wqe_opcode == 8'h02) begin
                                    bth_hdr[15] <= 1'b1;
                                end else begin
                                    bth_hdr[15] <= 1'b0;
                                end
                                if (aeth_pkt) begin
                                    bth_hdr[7:0] <= 8'h11;
                                    bth_hdr[71]  <= 1'b0;
                                end else if (rd_resp) begin
                                    o_out_rd_resp_pkt_cnt <= o_out_rd_resp_pkt_cnt + 1'b1;
                                    bth_hdr[71]           <= 1'b0;
                                    push_to_os_buf        <= 1'b0;
                                    if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= ((rd_resp_start_with_middle == 2'b01) ? 8'h0E : 8'h0D);  //RD_RESP_MIDDLE/RD_RESP_FIRST
                                        aeth_hdr[23:0] <= aeth_hdr[23:0] - 24'd1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0] <= ((rd_resp_start_with_middle == 2'b10) ? 8'h0F : 8'h10);  //RD_RESP_LAST/RD_RESP_ONLY
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0] <= 8'h0E;  //RD_RESP_MIDDLE
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h0F;  //RD_RESP_LAST
                                        aeth_hdr[23:0] <= aeth_hdr[23:0] + 24'd1;
                                    end
                                end else if (wqe_opcode == 8'h00) begin
                                    o_out_rw_pkt_cnt <= o_out_rw_pkt_cnt + 1'b1;
                                    bth_hdr[71]      <= exp_ack || (~|rem_len);
                                    if (PSN_based_retransmission == 1 && partial_wqe == 1'b1) begin
                                        if (|rem_len) begin
                                            bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                            push_to_os_buf <= first_trans;
                                        end else begin
                                            bth_hdr[7:0]   <= 8'h08;  //WRITE_LAST
                                            push_to_os_buf <= first_trans;
                                        end
                                    end else if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h06;  //WRITE_FIRST
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h0A;  //WRITE_ONLY
                                        push_to_os_buf <= 1'b1;
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h08;  //WRITE_LAST
                                        push_to_os_buf <= 1'b0;
                                    end
                                end else if (wqe_opcode == 8'h04) begin
                                    o_out_rw_pkt_cnt <= o_out_rw_pkt_cnt + 1'b1;
                                    bth_hdr[7:0]     <= 8'h0C;  //RDMA_READ
                                    bth_hdr[71]      <= 1'b1;
                                    push_to_os_buf   <= 1'b1;
                                end else if (wqe_opcode == 8'h02) begin
                                    o_out_send_pkt_cnt <= o_out_send_pkt_cnt + ((q_id != 16'h0000) ? 1'b1 : 1'b0);
                                    bth_hdr[71]        <= exp_ack || (~|rem_len);
                                    if (PSN_based_retransmission == 1 && partial_wqe == 1'b1) begin
                                        if (|rem_len) begin
                                            bth_hdr[7:0]   <= 8'h01;  //SEND_MIDDLE
                                            push_to_os_buf <= first_trans;
                                        end else begin
                                            bth_hdr[7:0]   <= 8'h02;  //SEND_LAST
                                            push_to_os_buf <= first_trans;
                                        end
                                    end else if (first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h00;  //SEND_FIRST
                                        push_to_os_buf <= 1'b1;
                                    end else if (first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h04;  //SEND_ONLY
                                        push_to_os_buf <= 1'b1;
                                    end else if (~first_trans && |rem_len) begin
                                        bth_hdr[7:0]   <= 8'h01;  //SEND_MIDDLE
                                        push_to_os_buf <= 1'b0;
                                    end else if (~first_trans && ~|rem_len) begin
                                        bth_hdr[7:0]   <= 8'h02;  //SEND_LAST
                                        push_to_os_buf <= 1'b0;
                                    end
                                end else begin
                                    bth_hdr[7:0]   <= 8'h04;
                                    bth_hdr[71]    <= 1'b0;
                                    push_to_os_buf <= 1'b0;
                                end
                                if (rd_resp) begin
                                    rd_resp_psn <= rd_resp_psn + 1'b1;
                                end
                                if (wqe_opcode == 8'h04) begin
                                    case (max_pmtu[12:8])
                                        5'h01: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:8] + |(rem_len[7:0]);
                                        end
                                        5'h02: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:9] + |(rem_len[8:0]);
                                        end
                                        5'h04: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:10] + |(rem_len[9:0]);
                                        end
                                        5'h08: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:11] + |(rem_len[10:0]);
                                        end
                                        5'h10: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:12] + |(rem_len[11:0]);
                                        end
                                        default: begin
                                            o_reg_sq_psn <= o_reg_sq_psn + rem_len[31:8] + |(rem_len[8:0]);
                                        end
                                    endcase
                                end else begin
                                    o_reg_sq_psn <= o_reg_sq_psn + (aeth_pkt ? 1'b0 : 1'b1);
                                end
                                o_out_ack_pkt_cnt <= o_out_ack_pkt_cnt + (aeth_pkt ? 1'b1 : 1'b0);
                                o_out_mad_pkt_cnt <= o_out_mad_pkt_cnt + (q_id == 16'h0000 ? 1'b1 : 1'b0);
                                local_offset <= local_offset + payload_len;
                                remote_offset <= remote_offset + payload_len;
                                reth_hdr <= {
                                    dma_len[7:0],
                                    dma_len[15:8],
                                    dma_len[23:16],
                                    dma_len[31:24],
                                    r_key[7:0],
                                    r_key[15:8],
                                    r_key[23:16],
                                    r_key[31:24],
                                    remote_offset[7:0],
                                    remote_offset[15:8],
                                    remote_offset[23:16],
                                    remote_offset[31:24],
                                    remote_offset[39:32],
                                    remote_offset[47:40],
                                    remote_offset[55:48],
                                    remote_offset[63:56]
                                };
                                o_hdr_data[1023:928] <= {payload_len, local_offset, completion_data_present, q_id[14:0]};
                                o_hdr_gen_rdy <= aeth_pkt || (rd_resp && ~|rem_len);
                                if (rd_resp || aeth_pkt || (~(i_wqe_halt && (i_wqe_halted_qpid == o_qp_id))  /* && ~(i_osq_nacked[o_qp_id])*/)) begin
                                    hdr_gen_cs <= HDR_STO;
                                    //if(wqe_opcode == 8'h00 && first_trans && ~aeth_pkt) begin
                                    //    get_data_buf <= ~retried;
                                    //end else begin
                                    //    get_data_buf <= 1'b0;
                                    //end
                                end else begin
                                    //hdr_gen_cs <= IDLE;
                                    hdr_gen_cs          <= DROP_WQE;
                                    o_wqe_dropped       <= first_trans;
                                    o_wqe_dropped_qpid  <= o_qp_id;
                                    //ipv4_identification <= ipv4_identification - 1'b1;
                                    ipv4_identification <= ipv4_identification - (ip_ver ? 1'b0 : 1'b1);
                                end
                            end
                        end
                        HDR_STO: begin
                            //get_data_buf <= 1'b0;
                            o_data_buf_fifo_data <= {data_buf_addr[C_M_AXI_ADDR_WIDTH-1:2], partial_wqe, (retried || (q_id == 16'h0000))};
                            o_hdr_gen_rdy        <= 1'b0;
                            if (~wqe_push_pending) begin
                                o_hdr_data[111:0] <= eth_hdr;
                                if (ip_ver) begin
                                    o_hdr_data[431:112] <= ip_hdr;
                                    o_hdr_data[495:432] <= udp_hdr;
                                    o_hdr_data[591:496] <= bth_hdr;
                                    if (reth_present) begin
                                        o_hdr_data[719:592] <= reth_hdr;
                                        o_hdr_len           <= 8'h5A;  //90B header
                                    end else if (aeth_pkt || rd_resp_aeth_present) begin
                                        o_hdr_data[623:592] <= {aeth_hdr[7:0], aeth_hdr[15:8], aeth_hdr[23:16], aeth_hdr[31:24]};
                                        o_hdr_len           <= 8'h4E;  //78B header
                                    end else if (completion_data_present) begin
                                        o_hdr_data[719:592] <= nvme_comp_data;
                                        o_hdr_len           <= 8'h5A;  //90B header
                                    end else begin
                                        o_hdr_data[719:592] <= 128'd0000;
                                        o_hdr_len           <= 8'h4A;  //74B header
                                    end
                                end else begin
                                    o_hdr_data[271:112] <= ip_hdr[159:0];
                                    o_hdr_data[335:272] <= udp_hdr;
                                    o_hdr_data[431:336] <= bth_hdr;
                                    if (reth_present) begin
                                        o_hdr_data[559:432] <= reth_hdr;
                                        o_hdr_len           <= 8'h46;  //70B header
                                    end else if (aeth_pkt || rd_resp_aeth_present) begin
                                        o_hdr_data[463:432] <= {aeth_hdr[7:0], aeth_hdr[15:8], aeth_hdr[23:16], aeth_hdr[31:24]};
                                        o_hdr_len           <= 8'h3A;  //58B header
                                    end else if (completion_data_present) begin
                                        o_hdr_data[559:432] <= nvme_comp_data;
                                        o_hdr_len           <= 8'h46;  //70B header
                                    end else begin
                                        o_hdr_data[559:432] <= 128'd0000;
                                        o_hdr_len           <= 8'h36;  //54B header
                                    end
                                end
                                //if(wqe_opcode == 8'h00 && first_trans && ~aeth_pkt && ~rd_resp && ~partial_wqe) begin
                                if (wqe_opcode == 8'h00 && first_trans && ~aeth_pkt && ~rd_resp) begin
                                    o_data_buf_fifo_wr_en <= 1'b1;
                                    get_data_buf          <= ~retried;
                                end else begin
                                    o_data_buf_fifo_wr_en <= 1'b0;
                                    get_data_buf          <= 1'b0;
                                end
                                push_to_os_buf <= 1'b0;
                                first_trans    <= 1'b0;
                                o_reg_wr_en    <= ~(aeth_pkt || rd_resp);
                                if ((wqe_opcode == 8'h00 || wqe_opcode == 8'h02 || rd_resp) && |rem_len) begin
                                    hdr_gen_cs <= WAIT0;
                                end else begin
                                    hdr_gen_cs  <= IDLE;
                                    o_reg_rd_en <= 1'b0;
                                    rd_resp     <= 1'b0;
                                end
                                aeth_pkt                  <= 1'b0;
                                reth_present              <= 1'b0;
                                rd_resp_aeth_present      <= 1'b0;
                                rd_resp_start_with_middle <= 2'b00;
                            end
                        end
                        ILL_OPCODE: begin
                            o_hdr_gen_rdy <= 1'b0;
                            if (~o_wqe_fatal_qp_vld) begin
                                //o_hdr_data[1023:512] <= 512'd0;
                                //o_hdr_data[511:0] <= {128'd0,nvme_comp_data,16'h0000,q_id,r_key,remote_offset,wqe_opcode,dma_len,local_offset,o_wqe_wrid};
                                //o_hdr_len <= 8'h00; 
                                hdr_gen_cs                <= IDLE;
                                o_reg_rd_en               <= 1'b0;
                                o_reg_wr_en               <= ~aeth_pkt;
                                aeth_pkt                  <= 1'b0;
                                rd_resp_start_with_middle <= 2'b00;
                                wqe_opcode_err            <= 1'b1;
                                push_to_os_buf            <= 1'b1;
                            end
                        end
                        DROP_WQE: begin
                            //   if(~(i_osq_pop && (i_osq_pop_qpid == o_qp_id))) begin
                            hdr_gen_cs    <= IDLE;
                            o_wqe_dropped <= 1'b0;
                            //   end
                        end
                        default: begin
                        end
                    endcase
                end
            end

        end
        //   else
        //     begin
        //	always @ (posedge core_clk)
        //	  o_out_rd_resp_pkt_cnt <= 0;
        //     end
    endgenerate  // end of C_EN_RECV_RD_WR == 1 && NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 1'b0


    generate
        begin
            if (NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 0) begin  // NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 0
                always @(posedge core_clk) begin
                    if (core_rst) begin
                        o_wqp_req              <= 1'b0;
                        o_wqp_req_Read         <= 1'b0;
                        o_wqe_qpid             <= {C_MAX_QID_WIDTH{1'b0}};
                        wqe_push_pending       <= 1'b0;
                        o_wqe_opcode           <= 8'h00;
                        o_wqe_send_psn         <= 24'd0;
                        o_wqe_send_msn         <= 24'd0;
                        o_wqe_rdma_rd_rcv_ba   <= {C_M_AXI_ADDR_WIDTH{1'b0}};
                        o_wqe_rdma_rd_rcv_len  <= 32'd0;
                        o_wqe_explicit_ack_req <= 1'b0;
                        o_wqe_retried          <= 1'b0;
                        o_last_out_pkt_info    <= 32'd0;
                        o_wqe_data_bufid       <= 15'h0000;
                        o_wqe_qp_to            <= 5'h00;
                        o_wqe_fatal_push       <= 1'b0;
                    end else begin
                        if (hdr_gen_cs == RD_Q_INFO) begin
                            if (retried) o_wqe_data_bufid <= retried_buf_id;
                            else o_wqe_data_bufid <= {{(15 - DATA_BUF_ID_WIDTH) {1'b0}}, data_bufid};
                        end

                        if (push_to_os_buf && ~wqe_push_pending && ~(i_wqe_halt && (i_wqe_halted_qpid == o_qp_id))) begin
                            o_wqp_req              <= ~aeth_pkt && push_to_os_buf;
                            o_wqp_req_Read         <= (wqe_opcode == 8'h04) ? (~aeth_pkt && push_to_os_buf) : 1'b0;  //(wqe_opcode == 8'h04) means RDMA READ
                            wqe_push_pending       <= 1'b1;
                            o_wqe_qpid             <= o_qp_id;
                            o_wqe_qp_to            <= reg_qp_to;
                            o_wqe_retried          <= retried;
                            o_wqe_opcode           <= wqe_opcode;  //bth_hdr[7:0];
                            o_wqe_explicit_ack_req <= bth_hdr[71];
                            o_wqe_fatal_push       <= wqe_pushed_in_fatal;
                            //Work request is getting pushed to outstanding buffer while sending the first transaction 
                            //WRITE_ONLY, WRITE_FIRST, SEND_ONLY, or SEND_FIRST 
                            //hence need to calculate the last PSN used for this work request depending on PMTU value
                            if (wqe_opcode == 8'h00 || wqe_opcode == 8'h02) begin
                                case (max_pmtu[12:8])
                                    5'h01: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:8] + |(dma_len[7:0]) - 'h2;
                                    end
                                    5'h02: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:9] + |(dma_len[8:0]) - 'h2;
                                    end
                                    5'h04: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:10] + |(dma_len[9:0]) - 'h2;
                                    end
                                    5'h08: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:11] + |(dma_len[10:0]) - 'h2;
                                    end
                                    5'h10: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:12] + |(dma_len[11:0]) - 'h2;
                                    end
                                    default: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:8] + |(dma_len[8:0]) - 'h2;
                                    end
                                endcase
                            end else begin
                                o_wqe_send_psn <= o_reg_sq_psn - 1'b1;
                            end
                            o_wqe_send_msn      <= o_reg_sq_msn;
                            o_last_out_pkt_info <= ~aeth_pkt ? {{(12 - C_MAX_QID_WIDTH) {1'b0}}, o_qp_id, (o_reg_sq_psn[11:0] - 1'b1), wqe_opcode} : o_last_out_pkt_info;
                        end else if (i_wqp_resp) begin
                            wqe_push_pending <= 1'b0;
                            o_wqp_req        <= 1'b0;
                            o_wqp_req_Read   <= 1'b0;
                        end
                        if (wqe_opcode == 8'h04) begin
                            o_wqe_rdma_rd_rcv_ba  <= local_offset;
                            o_wqe_rdma_rd_rcv_len <= dma_len;
                        end
                    end
                end
            end // end of NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 0

            else begin  // current: NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 1
                ///////////////////
                always @(posedge core_clk) begin
                    if (core_rst) begin
                        o_wqp_req              <= 1'b0;
                        o_wqp_req_Read         <= 1'b0;
                        o_wqe_qpid             <= {C_MAX_QID_WIDTH{1'b0}};
                        wqe_push_pending       <= 1'b0;
                        o_wqe_opcode           <= 8'h00;
                        o_wqe_send_psn         <= 24'd0;
                        o_wqe_send_msn         <= 24'd0;
                        o_wqe_rdma_rd_rcv_ba   <= {C_M_AXI_ADDR_WIDTH{1'b0}};
                        o_wqe_rdma_rd_rcv_len  <= 32'd0;
                        o_wqe_explicit_ack_req <= 1'b0;
                        o_wqe_retried          <= 1'b0;
                        o_last_out_pkt_info    <= 32'd0;
                        o_wqe_data_bufid       <= 15'h0000;
                        o_wqe_qp_to            <= 5'h00;
                        o_wqe_fatal_push       <= 1'b0;
                    end else begin
                        if (hdr_gen_cs == RD_Q_INFO) begin
                            if (retried) o_wqe_data_bufid <= retried_buf_id;
                            else o_wqe_data_bufid <= {{(15 - DATA_BUF_ID_WIDTH) {1'b0}}, data_bufid};
                        end

                        if (push_to_os_buf && ~wqe_push_pending && ~(i_wqe_halt && (i_wqe_halted_qpid == o_qp_id))) begin
                            o_wqp_req              <= ~aeth_pkt && push_to_os_buf;
                            o_wqp_req_Read         <= (wqe_opcode == 8'h04) ? (~aeth_pkt && push_to_os_buf) : 1'b0;  //(wqe_opcode == 8'h04) means RDMA READ
                            wqe_push_pending       <= 1'b1;
                            o_wqe_qpid             <= o_qp_id;
                            o_wqe_qp_to            <= reg_qp_to;
                            o_wqe_retried          <= retried;
                            o_wqe_opcode           <= wqe_opcode;  //bth_hdr[7:0];
                            o_wqe_explicit_ack_req <= bth_hdr[71];
                            o_wqe_fatal_push       <= wqe_pushed_in_fatal;
                            //Work request is getting pushed to outstanding buffer while sending the first transaction 
                            //WRITE_ONLY, WRITE_FIRST, SEND_ONLY, or SEND_FIRST 
                            //hence need to calculate the last PSN used for this work request depending on PMTU value
                            if (wqe_opcode[7:1] == 7'h00 || wqe_opcode[7:1] == 7'h01 || wqe_opcode == 8'h0C) begin
                                case (max_pmtu[12:8])
                                    5'h01: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:8] + |(dma_len[7:0]) - 'h2;
                                    end
                                    5'h02: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:9] + |(dma_len[8:0]) - 'h2;
                                    end
                                    5'h04: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:10] + |(dma_len[9:0]) - 'h2;
                                    end
                                    5'h08: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:11] + |(dma_len[10:0]) - 'h2;
                                    end
                                    5'h10: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:12] + |(dma_len[11:0]) - 'h2;
                                    end
                                    default: begin
                                        o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:8] + |(dma_len[8:0]) - 'h2;
                                    end
                                endcase
                            end else begin
                                o_wqe_send_psn <= o_reg_sq_psn - 1'b1;
                            end
                            o_wqe_send_msn      <= o_reg_sq_msn;
                            o_last_out_pkt_info <= ~aeth_pkt ? {{(12 - C_MAX_QID_WIDTH) {1'b0}}, o_qp_id, (o_reg_sq_psn[11:0] - 1'b1), wqe_opcode} : o_last_out_pkt_info;
                        end else if (i_wqp_resp) begin
                            wqe_push_pending <= 1'b0;
                            o_wqp_req        <= 1'b0;
                            o_wqp_req_Read   <= 1'b0;
                        end
                        if (wqe_opcode == 8'h04) begin
                            o_wqe_rdma_rd_rcv_ba  <= local_offset;
                            o_wqe_rdma_rd_rcv_len <= dma_len;
                        end
                    end
                end
                /////////////////////
            end
        end
    endgenerate



    assign no_data_buf_left = 1'b0;
    assign data_buf_addr    = 'd0;
    assign data_bufid       = {DATA_BUF_ID_WIDTH{1'b0}};
    assign o_dma_in_idle    = dma_in_idle;

endmodule
