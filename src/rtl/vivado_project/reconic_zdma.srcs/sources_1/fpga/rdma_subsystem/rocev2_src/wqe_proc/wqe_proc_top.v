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
// wqe_proc_top.v
//-----------------------------------------------------------------------------
//
// Filename        : wqe_proc_top.v
// Version         : v1.0
// Description     : WQE processor top module contains logic to stitch 
//		             axi master and slave block, header generation, and buffer
//		             manager blocks
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
`timescale 1ps / 1ps
module wqe_proc_top #(
    parameter C_MAX_QP = 8,
    parameter C_MAX_QID_WIDTH = 3,  // 5
    parameter C_M_AXI_DATA_WIDTH = 512,
    parameter C_M_AXI_ADDR_WIDTH = 32,  //64
    parameter C_M_AXI_ID_WIDTH = 4,
    parameter C_AXIS_DATA_WIDTH = 512,
    parameter C_MAX_SGL_DEPTH = 128,
    parameter C_MAX_HDR_DEPTH = 512,
    //parameter C_MAX_WRDATA_BUF_NUM = 128,
    //parameter C_EN_WR_RETRY_DATA_BUF = 1, //0
    parameter C_OS_Q_INDX_WIDTH = 3,  //4
    parameter C_VIVADO_VER = 1,
    parameter C_EN_DEBUG = 0,
    parameter C_MAX_RD_OS = 16,
    parameter C_EN_RECV_RD_WR = 1,
    parameter NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN = 0,  //1
    parameter C_EN_RD_RESP_MIDDLE = 1
) (
    input core_clk,
    input core_rst,

    // wqe_proc_top_m_axi_* 信号组
    output [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_awid,
    output [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output [                     7:0] m_axi_awlen,
    output [                     2:0] m_axi_awsize,
    output [                     1:0] m_axi_awburst,
    output [                     3:0] m_axi_awcache,
    output [                     2:0] m_axi_awprot,
    output                            m_axi_awvalid,
    input                             m_axi_awready,
    output [  C_M_AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output [C_M_AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output                            m_axi_wlast,
    output                            m_axi_wvalid,
    input                             m_axi_wready,
    output                            m_axi_awlock,
    input  [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_bid,
    input  [                     1:0] m_axi_bresp,
    input                             m_axi_bvalid,
    output                            m_axi_bready,
    output [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_arid,
    output [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output [                     7:0] m_axi_arlen,
    output [                     2:0] m_axi_arsize,
    output [                     1:0] m_axi_arburst,
    output [                     3:0] m_axi_arcache,
    output [                     2:0] m_axi_arprot,
    output                            m_axi_arvalid,
    input                             m_axi_arready,
    input  [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_rid,
    input  [  C_M_AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  [                     1:0] m_axi_rresp,
    input                             m_axi_rlast,
    input                             m_axi_rvalid,
    output                            m_axi_rready,
    output                            m_axi_arlock,

    // wqe_proc_wr_ddr_m_axi_* 信号组
    output [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_wr_ddr_awid,
    output [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_wr_ddr_awaddr,
    output [                     7:0] m_axi_wr_ddr_awlen,
    output [                     2:0] m_axi_wr_ddr_awsize,
    output [                     1:0] m_axi_wr_ddr_awburst,
    output [                     3:0] m_axi_wr_ddr_awcache,
    output [                     2:0] m_axi_wr_ddr_awprot,
    output                            m_axi_wr_ddr_awvalid,
    input                             m_axi_wr_ddr_awready,
    output [  C_M_AXI_DATA_WIDTH-1:0] m_axi_wr_ddr_wdata,
    output [C_M_AXI_DATA_WIDTH/8-1:0] m_axi_wr_ddr_wstrb,
    output                            m_axi_wr_ddr_wlast,
    output                            m_axi_wr_ddr_wvalid,
    input                             m_axi_wr_ddr_wready,
    output                            m_axi_wr_ddr_awlock,
    input  [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_wr_ddr_bid,
    input  [                     1:0] m_axi_wr_ddr_bresp,
    input                             m_axi_wr_ddr_bvalid,
    output                            m_axi_wr_ddr_bready,
    output [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_wr_ddr_arid,
    output [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_wr_ddr_araddr,
    output [                     7:0] m_axi_wr_ddr_arlen,
    output [                     2:0] m_axi_wr_ddr_arsize,
    output [                     1:0] m_axi_wr_ddr_arburst,
    output [                     3:0] m_axi_wr_ddr_arcache,
    output [                     2:0] m_axi_wr_ddr_arprot,
    output                            m_axi_wr_ddr_arvalid,
    input                             m_axi_wr_ddr_arready,
    input  [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_wr_ddr_rid,
    input  [  C_M_AXI_DATA_WIDTH-1:0] m_axi_wr_ddr_rdata,
    input  [                     1:0] m_axi_wr_ddr_rresp,
    input                             m_axi_wr_ddr_rlast,
    input                             m_axi_wr_ddr_rvalid,
    output                            m_axi_wr_ddr_rready,
    output                            m_axi_wr_ddr_arlock,

    //AXI streaming interface signals
    output [C_AXIS_DATA_WIDTH-1 : 0] m_axis_tdata,
    output [C_AXIS_DATA_WIDTH/8-1:0] m_axis_tkeep,
    output                           m_axis_tvalid,
    input                            m_axis_tready,
    output                           m_axis_tlast,


    input                           i_qpm_fifo_empty,
    input  [                 511:0] i_qpm_wqe_data,
    output                          o_qpm_wqe_pop,
    input                           i_wqe_halt,
    input  [C_MAX_QID_WIDTH -1 : 0] i_wqe_halted_qpid,
    output                          o_wqe_halted,

    output                       o_reg_rd_en,
    output                       o_reg_rd_en_s,
    input                        i_reg_rdy,
    input                        i_reg_psn_val,
    output                       o_reg_wr_en,
    output [C_MAX_QID_WIDTH-1:0] o_qp_id,
    input  [               15:0] i_reg_pkey,
    input  [                2:0] i_reg_pmtu,
    input  [               23:0] i_reg_dest_qpid,
    input  [               23:0] i_reg_sq_psn,
    output [               23:0] o_reg_sq_psn,
    input  [               23:0] i_reg_sq_msn,
    output [               23:0] o_reg_sq_msn,
    input                        i_freeup_data_buf,
    input  [               14:0] i_freeup_data_bufid,
    input  [                4:0] i_reg_qp_to,
    input  [                7:0] i_reg_max_rd_atomic,
    input  [               47:0] i_reg_mac_dest_addr,
    input  [              127:0] i_reg_ip_dest_addr,
    input  [               15:0] i_reg_udp_src_port,
    input  [                7:0] i_reg_ttl,
    input  [                5:0] i_reg_dscp,
    input                        i_reg_exp_ack,

    input [                  47:0] i_reg_mac_src_addr,
    input [                  31:0] i_reg_ipv4_src_addr,
    input [                 127:0] i_reg_ipv6_src_addr,
    input                          i_reg_xrnic_en,
    input [                   1:0] i_reg_ack_resp_strategy,
    input                          i_reg_ip_ver,
    input [C_M_AXI_ADDR_WIDTH-1:0] i_reg_wrdata_buf_ba,
    input [                  15:0] i_reg_wrdata_num_bufs,
    input [                  15:0] i_reg_wrdata_buf_sz,

    // 响应处理程序 ACK 接口
    input                        i_bres_valid,
    input                        i_bres_exp_ack,
    input  [C_MAX_QID_WIDTH-1:0] i_bres_dest_qpid,
    output                       o_bres_fifo_full,
    input  [               23:0] i_res_psn,
    input  [                6:0] i_res_aeth_syndrome,
    input  [               23:0] i_res_msn,
    input  [                1:0] i_req_type,
    input  [               63:0] i_rd_req_phy_addr,
    input  [               31:0] i_rd_payload_len,
    input  [               12:0] i_rd_req_pmtu,
    input  [               23:0] i_rsp_last_psn,

    output                          o_wqp_req,
    output                          o_wqp_req_Read,
    input                           i_wqp_resp,
    output [   C_MAX_QID_WIDTH-1:0] o_wqe_qpid,
    output [                   7:0] o_wqe_opcode,
    output [                  15:0] o_wqe_wrid,
    output [                  23:0] o_wqe_send_psn,
    output [                  23:0] o_wqe_send_first_psn,
    output [                  23:0] o_wqe_send_msn,
    output [                  14:0] o_wqe_data_bufid,
    output [                   4:0] o_wqe_qp_to,
    output                          o_wqe_retried,
    output [C_M_AXI_ADDR_WIDTH-1:0] o_wqe_rdma_rd_rcv_ba,
    output [                  31:0] o_wqe_rdma_rd_rcv_len,
    output                          o_wqe_explicit_ack_req,
    output                          o_wqe_fatal_push,
    output [                  31:0] o_wqe_status_out,
    output [                  31:0] o_last_out_pkt_info,
    output [                  15:0] o_out_send_pkt_cnt,
    output [                  15:0] o_out_rw_pkt_cnt,
    output [                  15:0] o_out_ack_pkt_cnt,
    output [                  15:0] o_out_mad_pkt_cnt,
    output [                  15:0] o_out_rd_resp_pkt_cnt,
    output [                  15:0] o_wqe_proc_idle_cnt,
    output [                  15:0] o_wqe_proc_rd_wqe_cnt,
    output [                  15:0] o_wqe_proc_rd_q_info_cnt,
    output [                  15:0] o_wqe_proc_wait0_cnt,
    output [                  15:0] o_wqe_proc_ip_chksum_cnt,
    output [                  15:0] o_wqe_proc_hdr_gen_cnt,
    output [                  15:0] o_wqe_proc_hdr_sto_cnt,
    output [                  15:0] o_wqe_proc_hdr_sgl_buf_full_cnt,
    output [                  15:0] o_wqe_proc_wr_retry_buf_full_cnt,
    input                           i_debug_cnt_en,
    input                           i_debug_cnt_clr,

    input  [                     31:0] i_out_errsts_q_ba,
    input  [                     15:0] i_out_errsts_q_sz,
    output [                     15:0] o_out_errsts_q_wrptr,
    input                              i_intr_en_ill_opc_in_sq,
    input                              i_intr_clr_ill_opc_in_sq,
    output                             o_ill_opc_in_sq_intr,
    input  [    C_OS_Q_INDX_WIDTH : 0] os_num_vld_entries,
    //input      [C_MAX_QP - 1 : 0]               i_osq_nacked,
    output                             o_dma_in_idle,
    input                              i_qp_recovery,
    //output     [C_MAX_QP - 1 : 0]               o_osq_nacked_resp,
    output [      C_MAX_QID_WIDTH-1:0] o_wqe_fatal_qpid,
    output                             o_wqe_fatal_qp_vld,
    input                              i_wqe_fatal_qp_rdy,
    output                             o_wqe_dropped,
    output [      C_MAX_QID_WIDTH-1:0] o_wqe_dropped_qpid,
    input                              i_osq_pop,
    input  [      C_MAX_QID_WIDTH-1:0] i_osq_pop_qpid,
    input  [    C_MAX_QID_WIDTH-1 : 0] i_qp_enable_idx,
    input                              i_qp_enable_vld,
    input                              i_qp_enable_data,
    input  [      C_MAX_QID_WIDTH-1:0] i_qp_fatal_idx,
    input                              i_qp_fatal_vld,
    input                              i_qp_fatal_data,
    input  [(4*C_MAX_QID_WIDTH+4)-1:0] i_osq_nacked,
    output                             o_rd_req_fifo_full_wr_en_0,
    output [    C_MAX_QID_WIDTH-1 : 0] o_rd_req_fifo_full_wr_addr_0,
    output                             o_rd_req_fifo_full_wr_en_1,
    output [    C_MAX_QID_WIDTH-1 : 0] o_rd_req_fifo_full_wr_addr_1,
    output                             o_osq_nacked_resp_wr_en,
    output [    C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr,
    output                             o_osq_nacked_resp_wr_data,
    output                             o_osq_nacked_resp_wr_en_aux,
    output [    C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr_aux,
    output                             o_osq_nacked_resp_wr_data_aux,
    output                             o_osq_nacked_resp_wr_en_pending,
    output [    C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr_pending,
    output                             o_osq_nacked_resp_wr_data_pending,
    input                              init_start,
    input  [    C_MAX_QID_WIDTH-1 : 0] init_count,
    input                              i_qp_under_recovery_wr_en,
    input  [    C_MAX_QID_WIDTH-1 : 0] i_qp_under_recovery_wr_id,
    input                              i_qp_under_recovery_wr_data
);
    `include "../../rdma_macros.vh"

    localparam HDR_BRAM_ADDR_WIDTH = clog2(C_MAX_HDR_DEPTH);

    wire                            buf_mngr_rdy;
    wire [                  1023:0] hdr_data;
    wire [                     7:0] hdr_len;
    wire                            hdr_valid;
    wire                            hdr_gen_rdy;
    wire                            aeth_valid;
    wire [                    31:0] aeth_hdr;
    wire [     C_MAX_QID_WIDTH-1:0] aeth_qpid;
    wire [                    23:0] aeth_psn;

    //buffer manager to MAXI signals
    wire [C_M_AXI_ADDR_WIDTH-1 : 0] maxi_addr;
    wire                            maxi_wr_rdn;
    wire                            maxi_en;
    wire [                     7:0] maxi_len;
    wire [  C_M_AXI_DATA_WIDTH-1:0] maxi_rdata;
    wire [  C_M_AXI_DATA_WIDTH-1:0] maxi_wdata;
    wire                            maxi_rvalid;
    wire                            maxi_done;
    wire                            maxi_busy;
    wire                            maxi_error;
    wire                            maxi_wdata_rdy;

    //SGL Buffer signals
    wire [                    15:0] sgl_head_ptr;
    wire [                    14:0] sgl_bram_rd_addr;
    wire                            sgl_bram_rd_en;
    wire [                    82:0] sgl_bram_rd_data;
    //Header buffer signals
    wire [                    14:0] hdr_bram_rd_addr;
    wire                            hdr_bram_rd_en;
    wire [                   511:0] hdr_bram_rd_data;
    wire                            hdr_bram_pop;

    wire [ C_AXIS_DATA_WIDTH-1 : 0] axis_tdata_int;
    wire [ C_AXIS_DATA_WIDTH/8-1:0] axis_tkeep_int;
    wire                            axis_tvalid_int;
    wire                            axis_tlast_int;
    wire                            axis_tready_int;
    wire [ C_AXIS_DATA_WIDTH-1 : 0] m_axis_dma_tdata;
    wire [ C_AXIS_DATA_WIDTH/8-1:0] m_axis_dma_tkeep;
    wire                            m_axis_dma_tvalid;
    wire                            m_axis_dma_tlast;
    wire                            m_axis_dma_tready;
    wire [ C_AXIS_DATA_WIDTH-1 : 0] m_axis_dma_raw_tdata;
    wire [ C_AXIS_DATA_WIDTH/8-1:0] m_axis_dma_raw_tkeep;
    wire                            m_axis_dma_raw_tvalid;
    wire                            m_axis_dma_raw_tlast;
    wire                            m_axis_dma_raw_tready;
    wire                            wqe_opcode_err;
    wire                            data_ptr_fifo_wr_en;  //Data buffer write enable to fifo in data re-align logic. 
    wire [  C_M_AXI_ADDR_WIDTH-1:0] data_ptr_fifo_data;
    wire                            data_buf_fifo_empty;

    //write retry logic to MAXI signals(no use)
    wire [C_M_AXI_ADDR_WIDTH-1 : 0] maxi_wr_ddr_addr;
    wire                            maxi_wr_ddr_wr_rdn;
    wire                            maxi_wr_ddr_en;
    wire [                    15:0] maxi_wr_ddr_len;
    wire [  C_M_AXI_DATA_WIDTH-1:0] maxi_wr_ddr_wdata;
    wire                            maxi_wr_ddr_done;
    wire                            maxi_wr_ddr_busy;
    wire                            maxi_wr_ddr_bvalid;
    wire                            maxi_wr_ddr_error;
    wire                            maxi_wr_ddr_wdata_rdy;

    wire [                    63:0] rd_req_addr;
    wire [                    31:0] rd_req_len;
    wire                            rd_req_valid;
    wire [                     1:0] rd_req_start_with_middle;
    wire                            m_axis_tvalid_s;
    reg                             m_axis_tvalid_reg;
    wire [                     8:0] ctl_rx_pause_ack_s;
    wire                            i_osq_nacked_wr_en;
    wire [   C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr;
    wire                            i_osq_nacked_wr_en_pending;
    wire [   C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_pending;
    wire                            i_osq_nacked_wr_en_aux;
    wire [   C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_aux;
    wire                            i_osq_nacked_wr_en_aux_pending;
    wire [   C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_aux_pending;

    assign m_axis_tvalid = m_axis_tvalid_s;



    //////////////////////////////////////////////////////////////////////////////////
    /////// WQE CRC wrapper block instance ///////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    wqe_proc_crc_wrap crc_wrap_inst (  // CRC Logic 模块
        .core_clk     (core_clk),
        .core_rst     (core_rst),
        .m_axis_tdata (m_axis_tdata),
        .m_axis_tkeep (m_axis_tkeep),
        .m_axis_tvalid(m_axis_tvalid_s),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast (m_axis_tlast),

        .s_axis_tdata (axis_tdata_int),
        .s_axis_tkeep (axis_tkeep_int),
        .s_axis_tvalid(axis_tvalid_int),
        .s_axis_tlast (axis_tlast_int),
        .s_axis_tready(axis_tready_int)
    );



    //////////////////////////////////////////////////////////////////////////////////
    /////// WQE data re-alignment block instance /////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    // The DMA source is an UltraRAM-backed AXI stream.  Register it locally
    // before the wide DRE mux so the URAM output no longer crosses the DRE
    // shift/control cone in one 250 MHz cycle.  The two-entry skid form keeps
    // full one-beat-per-cycle throughput under continuous ready/valid traffic.
    axis_register #(
        .DATA_WIDTH (C_AXIS_DATA_WIDTH),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH (C_AXIS_DATA_WIDTH/8),
        .LAST_ENABLE(1),
        .ID_ENABLE   (0),
        .ID_WIDTH    (1),
        .DEST_ENABLE (0),
        .DEST_WIDTH  (1),
        .USER_ENABLE (0),
        .USER_WIDTH  (1),
        .REG_TYPE    (2)
    ) dma_to_dre_skid_inst (
        .clk          (core_clk),
        .rst          (core_rst),
        .s_axis_tdata (m_axis_dma_raw_tdata),
        .s_axis_tkeep (m_axis_dma_raw_tkeep),
        .s_axis_tvalid(m_axis_dma_raw_tvalid),
        .s_axis_tready(m_axis_dma_raw_tready),
        .s_axis_tlast (m_axis_dma_raw_tlast),
        .s_axis_tid   (1'b0),
        .s_axis_tdest (1'b0),
        .s_axis_tuser (1'b0),
        .m_axis_tdata (m_axis_dma_tdata),
        .m_axis_tkeep (m_axis_dma_tkeep),
        .m_axis_tvalid(m_axis_dma_tvalid),
        .m_axis_tready(m_axis_dma_tready),
        .m_axis_tlast (m_axis_dma_tlast),
        .m_axis_tid   (),
        .m_axis_tdest (),
        .m_axis_tuser ()
    );

    wqe_proc_dre #(
        .C_AXIS_DATA_WIDTH(512),
        .C_M_AXI_DATA_WIDTH(C_M_AXI_DATA_WIDTH),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_ID_WIDTH(C_M_AXI_ID_WIDTH),
        .IP2BUS_LEN_WIDTH(16),
        //.C_MAX_WRDATA_BUF_NUM(C_MAX_WRDATA_BUF_NUM),
        //.C_EN_WR_RETRY_DATA_BUF(C_EN_WR_RETRY_DATA_BUF),
        .C_EN_DEBUG(C_EN_DEBUG)
    ) dre_inst (  // WQE 数据重新对齐块实例
        .core_clk     (core_clk),
        .core_rst     (core_rst),
        .m_axis_tdata (axis_tdata_int),
        .m_axis_tkeep (axis_tkeep_int),
        .m_axis_tvalid(axis_tvalid_int),
        .m_axis_tready(axis_tready_int),
        .m_axis_tlast (axis_tlast_int),

        .s_axis_tdata (m_axis_dma_tdata),
        .s_axis_tkeep (m_axis_dma_tkeep),
        .s_axis_tvalid(m_axis_dma_tvalid),
        .s_axis_tlast (m_axis_dma_tlast),
        .s_axis_tready(m_axis_dma_tready),

        // no use
        .i_data_ptr_fifo_wr_en(data_ptr_fifo_wr_en),
        .i_data_ptr_fifo_data (data_ptr_fifo_data),
        .o_data_buf_fifo_empty(data_buf_fifo_empty),
        .i_blk_base_addr      (i_reg_wrdata_buf_ba),

        // no use
        .maxi_addr     (maxi_wr_ddr_addr),
        .maxi_wr_rdn   (maxi_wr_ddr_wr_rdn),
        .maxi_en       (maxi_wr_ddr_en),
        .maxi_len      (maxi_wr_ddr_len),
        .maxi_wdata_rdy(maxi_wr_ddr_wdata_rdy),
        .maxi_wdata    (maxi_wr_ddr_wdata),
        .maxi_done     (maxi_wr_ddr_done),
        .maxi_busy     (maxi_wr_ddr_busy),
        .maxi_bvalid   (maxi_wr_ddr_bvalid),
        .maxi_error    (maxi_wr_ddr_error),

        .m_axi_awid   (m_axi_wr_ddr_awid),
        .m_axi_awaddr (m_axi_wr_ddr_awaddr),
        .m_axi_awlen  (m_axi_wr_ddr_awlen),
        .m_axi_awsize (m_axi_wr_ddr_awsize),
        .m_axi_awburst(m_axi_wr_ddr_awburst),
        .m_axi_awcache(m_axi_wr_ddr_awcache),
        .m_axi_awprot (m_axi_wr_ddr_awprot),
        .m_axi_awlock (m_axi_wr_ddr_awlock),
        .m_axi_awready(m_axi_wr_ddr_awready),
        .m_axi_awvalid(m_axi_wr_ddr_awvalid),
        .m_axi_wready (m_axi_wr_ddr_wready),
        .m_axi_wvalid (m_axi_wr_ddr_wvalid),
        .m_axi_wdata  (m_axi_wr_ddr_wdata),
        .m_axi_wlast  (m_axi_wr_ddr_wlast),
        .m_axi_wstrb  (m_axi_wr_ddr_wstrb),
        .m_axi_bready (m_axi_wr_ddr_bready),
        .m_axi_bvalid (m_axi_wr_ddr_bvalid),

        .i_debug_cnt_en                  (i_debug_cnt_en),
        .i_debug_cnt_clr                 (i_debug_cnt_clr),
        .o_wqe_proc_wr_retry_buf_full_cnt(o_wqe_proc_wr_retry_buf_full_cnt)
    );


    //////////////////////////////////////////////////////////////////////////////////
    /////// WQE Header Generation block instance /////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    wqe_proc_hdr_gen #(
        .NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN(NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN),
        .C_MAX_QP(C_MAX_QP),
        .C_MAX_QID_WIDTH(C_MAX_QID_WIDTH),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_OS_Q_INDX_WIDTH(C_OS_Q_INDX_WIDTH),
        .C_EN_DEBUG(C_EN_DEBUG),
        //.C_MAX_WRDATA_BUF_NUM(C_MAX_WRDATA_BUF_NUM),
        //.C_EN_WR_RETRY_DATA_BUF(C_EN_WR_RETRY_DATA_BUF),
        .C_EN_RECV_RD_WR(C_EN_RECV_RD_WR)
    ) hdr_gen_inst (
        .core_clk(core_clk),
        .core_rst(core_rst),

        .i_qpm_fifo_empty (i_qpm_fifo_empty),
        .i_qpm_wqe_data   (i_qpm_wqe_data),
        .o_qpm_wqe_pop    (o_qpm_wqe_pop),
        .i_wqe_halt       (i_wqe_halt),
        .i_wqe_halted_qpid(i_wqe_halted_qpid),
        .o_wqe_halted     (o_wqe_halted),

        .o_reg_rd_en        (o_reg_rd_en),
        .o_reg_rd_en_s      (o_reg_rd_en_s),
        .i_reg_info_val     (i_reg_rdy),
        .i_reg_psn_val      (i_reg_psn_val),
        .o_reg_wr_en        (o_reg_wr_en),
        .o_qp_id            (o_qp_id),
        .i_reg_pkey         (i_reg_pkey),
        .i_reg_qp_to        (i_reg_qp_to),
        .i_reg_pmtu         (i_reg_pmtu),
        .i_reg_dest_qpid    (i_reg_dest_qpid),
        .i_reg_sq_psn       (i_reg_sq_psn),
        .o_reg_sq_psn       (o_reg_sq_psn),
        .i_reg_sq_msn       (i_reg_sq_msn),
        .o_reg_sq_msn       (o_reg_sq_msn),
        .i_reg_max_rd_atomic(i_reg_max_rd_atomic),
        .i_reg_mac_dest_addr(i_reg_mac_dest_addr),
        .i_reg_ip_dest_addr (i_reg_ip_dest_addr),
        .i_reg_udp_src_port (i_reg_udp_src_port),
        .i_reg_ttl          (i_reg_ttl),
        .i_reg_dscp         (i_reg_dscp),
        .i_reg_exp_ack      (i_reg_exp_ack),

        .i_reg_mac_src_addr   (i_reg_mac_src_addr),
        .i_reg_ipv4_src_addr  (i_reg_ipv4_src_addr),
        .i_reg_ipv6_src_addr  (i_reg_ipv6_src_addr),
        .i_reg_xrnic_en       (i_reg_xrnic_en),
        .i_reg_ip_ver         (i_reg_ip_ver),
        .i_reg_wrdata_buf_ba  (i_reg_wrdata_buf_ba),
        .i_reg_wrdata_num_bufs(i_reg_wrdata_num_bufs),
        .i_reg_wrdata_buf_sz  (i_reg_wrdata_buf_sz),

        .i_buf_mngr_rdy(buf_mngr_rdy),
        .o_hdr_data    (hdr_data),
        .o_hdr_len     (hdr_len),
        .o_hdr_valid   (hdr_valid),
        .o_hdr_ip_ver  (hdr_ip_ver),

        .o_wqp_req             (o_wqp_req),
        .o_wqp_req_Read        (o_wqp_req_Read),
        .i_wqp_resp            (i_wqp_resp),
        .o_wqe_qpid            (o_wqe_qpid),
        .o_wqe_opcode          (o_wqe_opcode),
        .o_wqe_wrid            (o_wqe_wrid),
        .o_wqe_send_psn        (o_wqe_send_psn),
        .o_wqe_send_first_psn  (o_wqe_send_first_psn),
        .o_wqe_send_msn        (o_wqe_send_msn),
        .o_wqe_rdma_rd_rcv_ba  (o_wqe_rdma_rd_rcv_ba),
        .o_wqe_rdma_rd_rcv_len (o_wqe_rdma_rd_rcv_len),
        .o_wqe_explicit_ack_req(o_wqe_explicit_ack_req),
        .o_wqe_fatal_push      (o_wqe_fatal_push),
        .o_wqe_data_bufid      (o_wqe_data_bufid),
        .o_wqe_qp_to           (o_wqe_qp_to),
        .o_wqe_retried         (o_wqe_retried),
        .i_freeup_data_buf     (i_freeup_data_buf),
        .i_freeup_data_bufid   (i_freeup_data_bufid),

        .o_data_buf_fifo_wr_en(data_ptr_fifo_wr_en),
        .o_data_buf_fifo_data (data_ptr_fifo_data),
        .i_data_buf_fifo_empty(data_buf_fifo_empty),

        .o_hdr_gen_rdy                    (hdr_gen_rdy),
        .i_aeth_valid                     (aeth_valid),
        .i_aeth_hdr                       (aeth_hdr),
        .i_aeth_qpid                      (aeth_qpid),
        .i_aeth_psn                       (aeth_psn),
        .i_rd_req_addr                    (rd_req_addr),
        .i_rd_req_len                     (rd_req_len),
        .i_rd_req_valid                   (rd_req_valid),
        .i_rd_req_start_with_middle       (rd_req_start_with_middle),
        .wqe_opcode_err                   (wqe_opcode_err),
        .wqe_status_out                   (o_wqe_status_out[11:0]),
        .o_last_out_pkt_info              (o_last_out_pkt_info),
        .o_out_send_pkt_cnt               (o_out_send_pkt_cnt),
        .o_out_rw_pkt_cnt                 (o_out_rw_pkt_cnt),
        .o_out_ack_pkt_cnt                (o_out_ack_pkt_cnt),
        .o_out_mad_pkt_cnt                (o_out_mad_pkt_cnt),
        .o_out_rd_resp_pkt_cnt            (o_out_rd_resp_pkt_cnt),
        .os_num_vld_entries               (os_num_vld_entries),
        .i_debug_cnt_en                   (i_debug_cnt_en),
        .i_debug_cnt_clr                  (i_debug_cnt_clr),
        .o_wqe_proc_idle_cnt              (o_wqe_proc_idle_cnt),
        .o_wqe_proc_rd_wqe_cnt            (o_wqe_proc_rd_wqe_cnt),
        .o_wqe_proc_rd_q_info_cnt         (o_wqe_proc_rd_q_info_cnt),
        .o_wqe_proc_wait0_cnt             (o_wqe_proc_wait0_cnt),
        .o_wqe_proc_ip_chksum_cnt         (o_wqe_proc_ip_chksum_cnt),
        .o_wqe_proc_hdr_gen_cnt           (o_wqe_proc_hdr_gen_cnt),
        .o_wqe_proc_hdr_sto_cnt           (),
        .i_qp_recovery                    (i_qp_recovery),
        .o_dma_in_idle                    (o_dma_in_idle),
        .dma_in_idle                      (dma_in_idle),
        .o_wqe_fatal_qpid                 (o_wqe_fatal_qpid),
        .o_wqe_fatal_qp_vld               (o_wqe_fatal_qp_vld),
        .i_wqe_fatal_qp_rdy               (i_wqe_fatal_qp_rdy),
        .i_osq_pop                        (i_osq_pop),
        .i_osq_pop_qpid                   (i_osq_pop_qpid),
        .o_wqe_dropped                    (o_wqe_dropped),
        .o_wqe_dropped_qpid               (o_wqe_dropped_qpid),
        .init_start                       (init_start),
        .init_count                       (init_count),
        .i_qp_fatal_wr_en                 (i_qp_fatal_vld),
        .i_qp_fatal_wr_addr               (i_qp_fatal_idx),
        .i_qp_fatal_wr_data               (i_qp_fatal_data),
        .i_osq_nacked_wr_en               (i_osq_nacked_wr_en),
        .i_osq_nacked_wr_addr             (i_osq_nacked_wr_addr),
        .i_osq_nacked_wr_data             (1'b1),
        .i_osq_nacked_wr_en_pending       (i_osq_nacked_wr_en_pending),
        .i_osq_nacked_wr_addr_pending     (i_osq_nacked_wr_addr_pending),
        .i_osq_nacked_wr_en_aux           (i_osq_nacked_wr_en_aux),
        .i_osq_nacked_wr_addr_aux         (i_osq_nacked_wr_addr_aux),
        .i_osq_nacked_wr_data_aux         (1'b0),
        .i_osq_nacked_wr_en_aux_pending   (i_osq_nacked_wr_en_aux_pending),
        .i_osq_nacked_wr_addr_aux_pending (i_osq_nacked_wr_addr_aux_pending),
        .o_osq_nacked_resp_wr_en          (o_osq_nacked_resp_wr_en),
        .o_osq_nacked_resp_wr_addr        (o_osq_nacked_resp_wr_addr),
        .o_osq_nacked_resp_wr_data        (o_osq_nacked_resp_wr_data),
        .o_osq_nacked_resp_wr_en_aux      (o_osq_nacked_resp_wr_en_aux),
        .o_osq_nacked_resp_wr_addr_aux    (o_osq_nacked_resp_wr_addr_aux),
        .o_osq_nacked_resp_wr_data_aux    (o_osq_nacked_resp_wr_data_aux),
        .o_osq_nacked_resp_wr_en_pending  (o_osq_nacked_resp_wr_en_pending),
        .o_osq_nacked_resp_wr_addr_pending(o_osq_nacked_resp_wr_addr_pending),
        .o_osq_nacked_resp_wr_data_pending(o_osq_nacked_resp_wr_data_pending)
    );

    assign {i_osq_nacked_wr_en, i_osq_nacked_wr_addr, i_osq_nacked_wr_en_pending, i_osq_nacked_wr_addr_pending, i_osq_nacked_wr_en_aux, i_osq_nacked_wr_addr_aux, i_osq_nacked_wr_en_aux_pending, i_osq_nacked_wr_addr_aux_pending} = i_osq_nacked;
    //////////////////////////////////////////////////////////////////////////////////
    /////// WQE buffer Management block instance /////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    wqe_proc_buf_mgr #(
        .C_MAX_QP(C_MAX_QP),
        .C_MAX_QID_WIDTH(C_MAX_QID_WIDTH),
        .C_EN_DEBUG(C_EN_DEBUG),
        .C_MAX_SGL_DEPTH(C_MAX_SGL_DEPTH),
        .NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN(NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN),
        .C_MAX_HDR_DEPTH(C_MAX_HDR_DEPTH)
    ) buf_mgr_inst (
        .core_clk(core_clk),
        .core_rst(core_rst),

        .o_buf_mngr_rdy(buf_mngr_rdy),
        .i_hdr_data    (hdr_data),
        .i_hdr_len     (hdr_len),
        .i_hdr_valid   (hdr_valid),
        .i_reg_ip_ver  (hdr_ip_ver),

        .sgl_head_ptr    (sgl_head_ptr),
        .sgl_bram_rd_addr(sgl_bram_rd_addr),
        .sgl_bram_rd_en  (sgl_bram_rd_en),
        .sgl_bram_rd_data(sgl_bram_rd_data),

        .hdr_bram_rd_addr               (hdr_bram_rd_addr),
        .hdr_bram_rd_en                 (hdr_bram_rd_en),
        .hdr_bram_pop                   (hdr_bram_pop),
        .hdr_bram_rd_data               (hdr_bram_rd_data),
        .buf_mgr_ptr                    (o_wqe_status_out[31:12]),
        .i_m_axis_bkp_pressure          (m_axis_tvalid && !(m_axis_tready)),
        .i_debug_cnt_en                 (i_debug_cnt_en),
        .i_debug_cnt_clr                (i_debug_cnt_clr),
        .o_wqe_proc_hdr_sgl_buf_full_cnt(o_wqe_proc_hdr_sgl_buf_full_cnt),
        .o_wqe_proc_maxis_bk_pressure   (o_wqe_proc_hdr_sto_cnt)
    );


    //////////////////////////////////////////////////////////////////////////////////
    /////// WQE DMA logic ////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    wqe_proc_dma #(
        .C_M_AXI_DATA_WIDTH(C_M_AXI_DATA_WIDTH),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_ID_WIDTH(C_M_AXI_ID_WIDTH),
        .C_AXIS_DATA_WIDTH(C_AXIS_DATA_WIDTH),
        .C_MAX_SGL_DEPTH(C_MAX_SGL_DEPTH),
        .C_SGL_DATA_WIDTH(83),  //C_SGL_DATA_WIDTH),
        .C_HDR_BUF_ADDR_WIDTH(HDR_BRAM_ADDR_WIDTH),
        .C_HDR_BUF_DEPTH(C_MAX_HDR_DEPTH),
        .C_VIVADO_VER(C_VIVADO_VER)
    ) dma_inst (

        .core_clk(core_clk),
        .core_rst(core_rst),

        .m_axi_dma_arid   (m_axi_arid),
        .m_axi_dma_araddr (m_axi_araddr),
        .m_axi_dma_arlen  (m_axi_arlen),
        .m_axi_dma_arsize (m_axi_arsize),
        .m_axi_dma_arburst(m_axi_arburst),
        .m_axi_dma_arcache(m_axi_arcache),
        .m_axi_dma_arprot (m_axi_arprot),
        .m_axi_dma_arvalid(m_axi_arvalid),
        .m_axi_dma_arready(m_axi_arready),
        .m_axi_dma_rid    (m_axi_rid),
        .m_axi_dma_rdata  (m_axi_rdata),
        .m_axi_dma_rresp  (m_axi_rresp),
        .m_axi_dma_rlast  (m_axi_rlast),
        .m_axi_dma_rvalid (m_axi_rvalid),
        .m_axi_dma_rready (m_axi_rready),
        .m_axi_dma_arlock (m_axi_arlock),

        .m_axis_dma_tdata (m_axis_dma_raw_tdata),
        .m_axis_dma_tkeep (m_axis_dma_raw_tkeep),
        .m_axis_dma_tvalid(m_axis_dma_raw_tvalid),
        .m_axis_dma_tready(m_axis_dma_raw_tready),
        .m_axis_dma_tlast (m_axis_dma_raw_tlast),

        .sgl_head_ptr    (sgl_head_ptr),
        .sgl_bram_rd_addr(sgl_bram_rd_addr),
        .sgl_bram_rd_en  (sgl_bram_rd_en),
        .sgl_bram_rd_data(sgl_bram_rd_data),

        .hdr_bram_rd_addr(hdr_bram_rd_addr),
        .hdr_bram_rd_en  (hdr_bram_rd_en),
        .hdr_bram_rd_data(hdr_bram_rd_data),
        .hdr_bram_pop    (hdr_bram_pop),

        .o_dma_in_idle(dma_in_idle)
    );



    //////////////////////////////////////////////////////////////////////////////////
    /////// WQE Acknowledge buffer block instance：WQE 确认缓冲区块实例 /////////////////
    //////////////////////////////////////////////////////////////////////////////////
    wqe_proc_ack_buf #(
        .C_MAX_QID_WIDTH(C_MAX_QID_WIDTH),
        .C_MAX_QP(C_MAX_QP),
        .C_EN_RECV_RD_WR(C_EN_RECV_RD_WR),
        .C_EN_RD_RESP_MIDDLE(C_EN_RD_RESP_MIDDLE),
        .C_MAX_RD_OS(C_MAX_RD_OS)
    ) ack_buf_inst (
        .core_clk                    (core_clk),
        .core_rst                    (core_rst),
        .i_bres_valid                (i_bres_valid),
        .i_bres_exp_ack              (i_bres_exp_ack),
        .i_bres_dest_qpid            (i_bres_dest_qpid),
        .o_bres_fifo_full            (o_bres_fifo_full),
        .i_res_psn                   (i_res_psn),
        .i_res_aeth_syndrome         (i_res_aeth_syndrome),
        .i_res_msn                   (i_res_msn),
        .i_req_type                  (i_req_type),
        .i_rd_req_phy_addr           (i_rd_req_phy_addr),
        .i_rd_payload_len            (i_rd_payload_len),
        .i_rd_req_pmtu               (i_rd_req_pmtu),
        .i_rsp_last_psn              (i_rsp_last_psn),
        //.i_qp_enable(i_qp_enable),
        //.o_rd_req_fifo_full(o_rd_req_fifo_full),
        .i_hdr_gen_rdy               (hdr_gen_rdy),
        .o_aeth_valid                (aeth_valid),
        .o_aeth_hdr                  (aeth_hdr),
        .o_aeth_qpid                 (aeth_qpid),
        .o_aeth_psn                  (aeth_psn),
        .i_reg_xrnic_en              (i_reg_xrnic_en),
        .ack_resp_strategy           (i_reg_ack_resp_strategy),
        //.i_qp_fatal(i_qp_fatal),
        .o_rd_req_addr               (rd_req_addr),
        .o_rd_req_len                (rd_req_len),
        .o_rd_req_valid              (rd_req_valid),
        .o_rd_req_start_with_middle  (rd_req_start_with_middle),
        .init_start                  (init_start),
        .init_count                  (init_count),
        .i_qp_enable_idx             (i_qp_enable_idx),
        .i_qp_enable_vld             (i_qp_enable_vld),
        .i_qp_enable_data            (i_qp_enable_data),
        .i_qp_fatal_idx              (i_qp_fatal_idx),
        .i_qp_fatal_vld              (i_qp_fatal_vld),
        .i_qp_fatal_data             (i_qp_fatal_data),
        .rd_req_fifo_full_reset_valid(o_rd_req_fifo_full_wr_en_0),
        .rd_req_fifo_full_reset_id   (o_rd_req_fifo_full_wr_addr_0),
        .rd_req_fifo_full_set_valid  (o_rd_req_fifo_full_wr_en_1),
        .rd_req_fifo_full_set_id     (o_rd_req_fifo_full_wr_addr_1),
        .i_qp_under_recovery_wr_en   (i_qp_under_recovery_wr_en),
        .i_qp_under_recovery_wr_id   (i_qp_under_recovery_wr_id),
        .i_qp_under_recovery_wr_data (i_qp_under_recovery_wr_data)
    );


    assign m_axi_wr_ddr_arid    = {C_M_AXI_ID_WIDTH{1'b0}};
    assign m_axi_wr_ddr_araddr  = {C_M_AXI_ADDR_WIDTH{1'b0}};
    assign m_axi_wr_ddr_arlen   = 8'h00;
    assign m_axi_wr_ddr_arsize  = 3'b000;
    assign m_axi_wr_ddr_arburst = 2'b00;
    assign m_axi_wr_ddr_arcache = 4'h0;
    assign m_axi_wr_ddr_arprot  = 3'b000;
    assign m_axi_wr_ddr_arvalid = 1'b0;
    assign m_axi_wr_ddr_rready  = 1'b1;
    assign m_axi_wr_ddr_arlock  = 1'b0;
    assign m_axi_awid           = {C_M_AXI_ID_WIDTH{1'b0}};
    assign m_axi_awaddr         = {C_M_AXI_ADDR_WIDTH{1'b0}};
    assign m_axi_awlen          = 8'h00;
    assign m_axi_awsize         = 3'b000;
    assign m_axi_awburst        = 2'b00;
    assign m_axi_awcache        = 4'h0;
    assign m_axi_awprot         = 3'b000;
    assign m_axi_awvalid        = 1'b0;
    assign m_axi_wdata          = {C_M_AXI_DATA_WIDTH{1'b0}};
    assign m_axi_wstrb          = {C_M_AXI_DATA_WIDTH / 8{1'b0}};
    assign m_axi_wlast          = 1'b0;
    assign m_axi_wvalid         = 1'b0;
    assign m_axi_awlock         = 1'b0;
    assign m_axi_bready         = 1'b1;

endmodule
