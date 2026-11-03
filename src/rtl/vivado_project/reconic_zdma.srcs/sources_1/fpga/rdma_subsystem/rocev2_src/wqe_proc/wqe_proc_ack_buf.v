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
// wqe_proc_ack_buf.v
//-----------------------------------------------------------------------------
//
// Filename        : wqe_proc_ack_buf.v
// Version         : v1.0
// Description     : WQE processor acknowledge buffer block implements logic to  
//		             store the AETH information coming from the RX block.
//		             this block handsover the AETH information to the Header
//		             generation block. 
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
module wqe_proc_ack_buf #(
    parameter C_MAX_QID_WIDTH = 11,
    parameter C_MAX_QP = 2048,
    parameter C_MAX_RD_OS = 16,
    parameter C_EN_RECV_RD_WR = 1,  // 1
    parameter C_EN_RD_RESP_MIDDLE = 1  // 1
) (
    input core_clk,
    input core_rst,

    //Response handler ACK interface
    input                          i_bres_valid,
    input                          i_bres_exp_ack,
    input  [  C_MAX_QID_WIDTH-1:0] i_bres_dest_qpid,
    output                         o_bres_fifo_full,
    input  [                 23:0] i_res_psn,
    input  [                  6:0] i_res_aeth_syndrome,
    input  [                 23:0] i_res_msn,
    input  [                  1:0] i_req_type,
    input  [                 63:0] i_rd_req_phy_addr,
    input  [                 31:0] i_rd_payload_len,
    input  [                 12:0] i_rd_req_pmtu,
    input  [                 23:0] i_rsp_last_psn,
    //output reg [C_MAX_QP - 1 : 0]               o_rd_req_fifo_full,
    input  [C_MAX_QID_WIDTH-1 : 0] i_qp_enable_idx,
    input                          i_qp_enable_vld,
    input                          i_qp_enable_data,

    input                              i_hdr_gen_rdy,
    output reg                         o_aeth_valid,
    output reg [                 31:0] o_aeth_hdr,
    output reg [  C_MAX_QID_WIDTH-1:0] o_aeth_qpid,
    output reg [                 23:0] o_aeth_psn,
    input                              i_reg_xrnic_en,
    input      [                  1:0] ack_resp_strategy,
    input                              init_start,
    input      [C_MAX_QID_WIDTH-1 : 0] init_count,
    input      [C_MAX_QID_WIDTH-1 : 0] i_qp_fatal_idx,
    input                              i_qp_fatal_vld,
    input                              i_qp_fatal_data,
    output reg [                 63:0] o_rd_req_addr,
    output reg [                 31:0] o_rd_req_len,
    output reg                         o_rd_req_valid,
    output reg [                  1:0] o_rd_req_start_with_middle,
    output                             rd_req_fifo_full_set_valid,
    output     [C_MAX_QID_WIDTH-1 : 0] rd_req_fifo_full_set_id,
    output                             rd_req_fifo_full_reset_valid,
    output     [C_MAX_QID_WIDTH-1 : 0] rd_req_fifo_full_reset_id,
    input                              i_qp_under_recovery_wr_en,
    input      [C_MAX_QID_WIDTH-1 : 0] i_qp_under_recovery_wr_id,
    input                              i_qp_under_recovery_wr_data


);
    `include "../../rdma_macros.vh"

    localparam RD_REQ_PTR_WIDTH = clog2(C_MAX_RD_OS);

    reg     [                                    55 : 0] aeth_bram_wr_data;
    reg                                                  aeth_bram_wr_en;
    wire    [                                    55 : 0] aeth_bram_rd_data;
    reg                                                  aeth_bram_rd_en;
    reg                                                  aeth_bram_rd_en_r;
    reg                                                  aeth_bram_rd_en_r1;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] aeth_bram_wr_addr;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] aeth_bram_rd_addr;
    reg     [                                      15:0] ack_ign_cnt;
    reg                                                  arb_in_progress;

    reg                                                  rd_req_bram_wr_en;
    reg                                                  rd_req_bram_rd_en;
    wire                                                 rd_req_bram_rd_en_ored;
    reg                                                  rd_req_bram_rd_en_r;
    reg                                                  rd_req_bram_rd_en_r1;
    reg     [C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH - 1 : 0] rd_req_bram_wr_addr;
    reg     [C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH - 1 : 0] rd_req_bram_rd_addr;
    reg     [                                   175 : 0] rd_req_bram_wr_data;
    reg     [                                   175 : 0] rd_req_bram_wr_data_r1;
    wire    [                                   175 : 0] rd_req_bram_rd_data;

    reg                                                  dup_rd_req_bram_wr_en;
    reg                                                  dup_rd_req_bram_rd_en;
    wire                                                 dup_rd_req_bram_rd_en_ored;
    reg                                                  dup_rd_req_bram_rd_en_r;
    reg                                                  dup_rd_req_en;
    reg     [                                       1:0] start_with_middle_r;
    reg     [                                       1:0] start_with_middle_r1;
    reg                                                  dup_rd_req_bram_rd_en_r1;
    reg     [C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH - 1 : 0] dup_rd_req_bram_wr_addr;
    reg     [C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH - 1 : 0] dup_rd_req_bram_rd_addr;
    reg     [                                   159 : 0] dup_rd_req_bram_wr_data;
    reg     [                                   159 : 0] dup_rd_req_bram_wr_data_r1;
    wire    [                                   159 : 0] dup_rd_req_bram_rd_data;
    reg                                                  bres_exp_ack;
    reg                                                  rd_req_valid_r;
    reg                                                  rd_req_valid_r1;
    reg                                                  rd_req_valid_r2;
    reg                                                  aeth_valid_r;
    reg                                                  aeth_valid_r1;
    reg                                                  aeth_valid_r2;

    reg     [                                      63:0] rd_req_addr_r;
    reg     [                                      31:0] rd_req_len_r;

    reg                                                  discard_rd_req;
    reg                                                  discard_rd_req_r;
    integer                                              i;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] i_bres_dest_qpid_r;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] i_bres_dest_qpid_2r;
    reg                                                  i_bres_valid_r;
    reg                                                  qp_ack_pending_wr_en;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] qp_ack_pending_wr_addr;
    reg                                                  qp_ack_pending_wr_data;
    reg                                                  qp_nack_pending_wr_en;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] qp_nack_pending_wr_addr;
    reg                                                  qp_nack_pending_wr_data;
    reg                                                  qp_nacked_wr_en;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] qp_nacked_wr_addr;
    reg                                                  qp_nacked_wr_data;
    reg                                                  qp_ack_entry_wr_en;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] qp_ack_entry_wr_addr;
    reg                                                  qp_ack_entry_wr_data;
    wire                                                 qp_ack_pending_rd_data_0;
    wire    [                     C_MAX_QID_WIDTH-1 : 0] qp_ack_pending_rd_addr_2;
    wire                                                 qp_ack_pending_rd_data_2;
    wire    [                     C_MAX_QID_WIDTH-1 : 0] qp_nack_pending_rd_addr_0;
    wire                                                 qp_nack_pending_rd_data_0;
    wire    [                     C_MAX_QID_WIDTH-1 : 0] qp_nack_pending_rd_addr_1;
    wire                                                 qp_nack_pending_rd_data_1;
    wire    [                     C_MAX_QID_WIDTH-1 : 0] qp_nack_pending_rd_addr_2;
    wire                                                 qp_nack_pending_rd_data_2;
    wire    [                     C_MAX_QID_WIDTH-1 : 0] qp_nacked_rd_addr_0;
    wire                                                 qp_nacked_rd_data_0;
    wire    [                     C_MAX_QID_WIDTH-1 : 0] qp_nacked_rd_addr_1;
    wire                                                 qp_nacked_rd_data_1;
    reg     [                                       1:0] i_req_type_r;
    wire                                                 qp_ack_pending_rd_data_1;

    reg                                                  rd_req_fifo_empty_wr_en_0;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] rd_req_fifo_empty_wr_addr_0;
    reg                                                  rd_req_fifo_empty_wr_data_0;
    reg                                                  rd_req_fifo_empty_wr_en_1;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] rd_req_fifo_empty_wr_addr_1;
    reg                                                  rd_req_fifo_empty_wr_data_1;
    wire                                                 rd_req_fifo_empty_rd_data;
    reg                                                  dup_rd_req_fifo_empty_wr_en_0;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] dup_rd_req_fifo_empty_wr_addr_0;
    reg                                                  dup_rd_req_fifo_empty_wr_data_0;
    reg                                                  dup_rd_req_fifo_empty_wr_en_1;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] dup_rd_req_fifo_empty_wr_addr_1;
    reg                                                  dup_rd_req_fifo_empty_wr_data_1;
    wire                                                 dup_rd_req_fifo_empty_rd_data;
    wire                                                 rd_req_fifo_full_rd_data;
    reg                                                  dup_rd_req_fifo_full_wr_en_0;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] dup_rd_req_fifo_full_wr_addr_0;
    reg                                                  dup_rd_req_fifo_full_wr_data_0;
    reg                                                  dup_rd_req_fifo_full_wr_en_1;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] dup_rd_req_fifo_full_wr_addr_1;
    reg                                                  dup_rd_req_fifo_full_wr_data_1;
    wire                                                 dup_rd_req_fifo_full_rd_data;
    reg                                                  core_rst_r;
    wire    [                     C_MAX_QID_WIDTH-1 : 0] arbitrated_Qpid;  // 
    wire                                                 arbitration_done;
    reg                                                  send_response;
    reg                                                  o_rd_req_valid_r;
    reg                                                  o_aeth_valid_r;
    reg                                                  o_rd_req_valid_2r;
    reg                                                  o_aeth_valid_2r;
    reg                                                  start_check;
    reg                                                  arbitration_done_r;
    reg                                                  arbitration_done_2r;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] pushback_Qpid;
    reg                                                  pushback_valid;
    reg                                                  entry_dropped;
    reg                                                  aeth_bram_wr_en_r;
    wire                                                 fifo_busy;
    reg                                                  pushback_done;
    reg                                                  hdr_gen_rdy_asserted;
    wire                                                 fetch_next_id;
    wire                                                 qp_enable_rd_data_1;
    wire                                                 qp_fatal_rd_data_0;
    wire                                                 qp_under_recovery;
    wire                                                 qp_under_recovery_rd_data;
    wire                                                 qp_fatal_rd_data_1;
    wire                                                 qp_fatal_rd_data_2;
    wire                                                 qp_fatal_rd_data_3;
    reg                                                  fetch_next_id_r;
    reg                                                  send_response_r;
    wire                                                 qp_is_enabled;
    reg     [                       C_MAX_QID_WIDTH : 0] Qpid_rst_buf_wr_addr;
    reg     [                       C_MAX_QID_WIDTH : 0] Qpid_rst_buf_rd_addr;
    wire    [                     C_MAX_QID_WIDTH-1 : 0] rst_qpid;
    wire                                                 rst_fifo_empty;
    wire                                                 rst_fifo_full;
    reg                                                  wr_ptr_buf_wren_init;
    reg                                                  wr_ptr_buf_wren_init_r;
    wire                                                 rst_fifo_rd_en;
    reg                                                  rst_fifo_rd_en_r;
    wire                                                 ptr_init_done;
    wire                                                 status_lut_init_done;
    reg                                                  ptr_init_done_flag;
    reg                                                  status_lut_init_done_flag;
    reg                                                  i_qp_under_recovery_wr_en_r;
    reg                                                  rd_req_fifo_full_wr_en_0;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] rd_req_fifo_full_wr_addr_0;
    reg                                                  rd_req_fifo_full_wr_en_1;
    reg     [                     C_MAX_QID_WIDTH-1 : 0] rd_req_fifo_full_wr_addr_1;



    always @(posedge core_clk) begin
        o_rd_req_valid_r    <= o_rd_req_valid;
        o_aeth_valid_r      <= o_aeth_valid;
        o_rd_req_valid_2r   <= o_rd_req_valid_r;
        o_aeth_valid_2r     <= o_aeth_valid_r;
        arbitration_done_r  <= arbitration_done;
        arbitration_done_2r <= arbitration_done_r;
        i_bres_dest_qpid_r  <= i_bres_dest_qpid;
        i_bres_dest_qpid_2r <= i_bres_dest_qpid_r;
        i_bres_valid_r      <= i_bres_valid;
        i_req_type_r        <= i_req_type;
        aeth_bram_wr_en_r   <= aeth_bram_wr_en;
    end

    always @(posedge core_clk) begin
        if (core_rst) begin
            send_response  <= 1'b0;
            pushback_valid <= 1'b0;
            pushback_Qpid  <= {C_MAX_QID_WIDTH{1'b0}};
            pushback_done  <= 1'b0;
            start_check    <= 1'b0;
        end else begin
            if (qp_is_enabled && arbitration_done && ~arbitration_done_r && !qp_fatal_rd_data_0 && !qp_under_recovery && !(rd_req_fifo_empty_rd_data && dup_rd_req_fifo_empty_rd_data)) begin
                send_response <= 1'b1;  //rd_response
            end else if (qp_is_enabled && arbitration_done && ~arbitration_done_r && qp_nack_pending_rd_data_2) begin
                send_response <= 1'b1;  //Nack
            end else if (qp_is_enabled && arbitration_done && ~arbitration_done_r && ack_resp_strategy != 2'b10 && !qp_fatal_rd_data_0 && !qp_under_recovery && qp_ack_pending_rd_data_1) begin
                send_response <= 1'b1;  //ack
            end else begin
                send_response <= 1'b0;
            end

            if ((rd_req_valid_r && !rd_req_valid_r1) || (aeth_valid_r && !aeth_valid_r1)) begin
                pushback_valid <= (((!(rd_req_fifo_empty_rd_data && dup_rd_req_fifo_empty_rd_data)) || qp_ack_pending_rd_data_1) && !qp_fatal_rd_data_0 && !qp_under_recovery) || qp_nack_pending_rd_data_2;
                pushback_Qpid  <= arbitrated_Qpid;
            end else if (~fifo_busy) begin
                pushback_valid <= 1'b0;
            end

            if ((rd_req_valid_r1 && !rd_req_valid_r2) || (aeth_valid_r1 && !aeth_valid_r2)) begin
                start_check <= 1'b1;
                if (~pushback_valid) begin
                    pushback_done <= 1'b1;
                end else if (pushback_valid && ~fifo_busy) begin
                    pushback_done <= 1'b1;
                end
            end else if (start_check && pushback_valid && ~fifo_busy) begin
                pushback_done <= 1'b1;
            end else if (fetch_next_id) begin
                pushback_done <= 1'b0;
                start_check   <= 1'b0;
            end

            if (arbitration_done_r && ~arbitration_done_2r && ~send_response) begin  //No_etries left for arb_fifo
                entry_dropped <= 1'b1;
            end else begin
                entry_dropped <= 1'b0;
            end

        end
    end


    always @(posedge core_clk) begin
        if (core_rst) begin
            hdr_gen_rdy_asserted <= 1'b0;
        end else begin
            if (i_hdr_gen_rdy && ~pushback_done) begin  //if ready came before pushback_done
                hdr_gen_rdy_asserted <= 1'b1;
            end else if (fetch_next_id) begin
                hdr_gen_rdy_asserted <= 1'b0;
            end
        end
    end

    assign fetch_next_id = (i_hdr_gen_rdy || hdr_gen_rdy_asserted) && pushback_done;

    // i_bres_dest_qpid_2r（延迟 2 拍对齐）被推入 FIFO_QPID_Arbiter
    // Arbiter 做轮询仲裁，输出 arbitrated_Qpid 表示"下一个该响应谁"
    FIFO_QPID_Arbiter #(
        .C_MAX_QP(C_MAX_QP),
        .C_MAX_QID_WIDTH(C_MAX_QID_WIDTH)
    ) inst_ack_rd_resp_Arbiter (
        .core_clk        (core_clk),
        .core_rst        (core_rst),
        .init_start      (init_start),
        .init_count      (init_count),
        .valid_in        ((aeth_bram_wr_en_r || rd_req_bram_wr_en || dup_rd_req_bram_wr_en)),
        .Qpid_in         (i_bres_dest_qpid_2r),
        .fetch_next_id   (fetch_next_id || entry_dropped),
        .pushback_Qpid   (pushback_Qpid),
        .pushback_valid  (pushback_valid),
        .arbitration_done(arbitration_done),
        .arbitrated_Qpid (arbitrated_Qpid),
        .fifo_busy       (fifo_busy)
    );


    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP),
        .C_Init_with_1(1)
    ) inst_rd_req_fifo_empty_0 (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || rd_req_fifo_empty_wr_en_0),
        .wr_addr    (init_start ? init_count : rd_req_fifo_empty_wr_addr_0),
        .wr_data    (init_start ? 1'b1 : rd_req_fifo_empty_wr_data_0),
        .wr_en_aux  (rd_req_fifo_empty_wr_en_1),
        .wr_addr_aux(rd_req_fifo_empty_wr_addr_1),
        .wr_data_aux(rd_req_fifo_empty_wr_data_1),
        .rd_en      (1'b1),
        .rd_addr    (arbitrated_Qpid),
        .rd_data    (rd_req_fifo_empty_rd_data)

    );

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP),
        .C_Init_with_1(1)
    ) inst_dup_rd_req_fifo_empty_0 (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || dup_rd_req_fifo_empty_wr_en_0),
        .wr_addr    (init_start ? init_count : dup_rd_req_fifo_empty_wr_addr_0),
        .wr_data    (init_start ? 1'b1 : dup_rd_req_fifo_empty_wr_data_0),
        .wr_en_aux  (dup_rd_req_fifo_empty_wr_en_1),
        .wr_addr_aux(dup_rd_req_fifo_empty_wr_addr_1),
        .wr_data_aux(dup_rd_req_fifo_empty_wr_data_1),
        .rd_en      (1'b1),
        .rd_addr    (arbitrated_Qpid),
        .rd_data    (dup_rd_req_fifo_empty_rd_data)

    );

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP),
        .C_Init_with_1(0)
    ) inst_rd_req_fifo_full_0 (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || rd_req_fifo_full_wr_en_0),
        .wr_addr    (init_start ? init_count : rd_req_fifo_full_wr_addr_0),
        .wr_data    (1'b0),
        .wr_en_aux  (rd_req_fifo_full_wr_en_1),
        .wr_addr_aux(rd_req_fifo_full_wr_addr_1),
        .wr_data_aux(1'b1),
        .rd_en      (1'b1),
        .rd_addr    (i_bres_dest_qpid),
        .rd_data    (rd_req_fifo_full_rd_data)

    );

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP),
        .C_Init_with_1(0)
    ) inst_dup_rd_req_fifo_full_0 (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || dup_rd_req_fifo_full_wr_en_0),
        .wr_addr    (init_start ? init_count : dup_rd_req_fifo_full_wr_addr_0),
        .wr_data    (init_start ? 1'b0 : dup_rd_req_fifo_full_wr_data_0),
        .wr_en_aux  (dup_rd_req_fifo_full_wr_en_1),
        .wr_addr_aux(dup_rd_req_fifo_full_wr_addr_1),
        .wr_data_aux(dup_rd_req_fifo_full_wr_data_1),
        .rd_en      (1'b1),
        .rd_addr    (i_bres_dest_qpid),
        .rd_data    (dup_rd_req_fifo_full_rd_data)

    );


    assign o_bres_fifo_full = 1'b0;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_ack_pending_0 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || qp_ack_pending_wr_en),
        .wr_addr (init_start ? init_count : qp_ack_pending_wr_addr),
        .wr_data (init_start ? 1'b0 : qp_ack_pending_wr_data),
        .rd_en   (1'b1),
        .rd_addr (i_bres_dest_qpid_r),
        .rd_data (qp_ack_pending_rd_data_0)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_ack_pending_1 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || qp_ack_pending_wr_en),
        .wr_addr (init_start ? init_count : qp_ack_pending_wr_addr),
        .wr_data (init_start ? 1'b0 : qp_ack_pending_wr_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_Qpid),
        .rd_data (qp_ack_pending_rd_data_1)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_nack_pending_0 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || qp_nack_pending_wr_en),
        .wr_addr (init_start ? init_count : qp_nack_pending_wr_addr),
        .wr_data (init_start ? 1'b0 : qp_nack_pending_wr_data),
        .rd_en   (1'b1),
        .rd_addr (qp_nack_pending_rd_addr_0),
        .rd_data (qp_nack_pending_rd_data_0)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_nack_pending_1 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || qp_nack_pending_wr_en),
        .wr_addr (init_start ? init_count : qp_nack_pending_wr_addr),
        .wr_data (init_start ? 1'b0 : qp_nack_pending_wr_data),
        .rd_en   (1'b1),
        .rd_addr (qp_nack_pending_rd_addr_1),
        .rd_data (qp_nack_pending_rd_data_1)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_nack_pending_2 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || qp_nack_pending_wr_en),
        .wr_addr (init_start ? init_count : qp_nack_pending_wr_addr),
        .wr_data (init_start ? 1'b0 : qp_nack_pending_wr_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_Qpid),
        .rd_data (qp_nack_pending_rd_data_2)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_nacked_0 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || qp_nacked_wr_en),
        .wr_addr (init_start ? init_count : qp_nacked_wr_addr),
        .wr_data (init_start ? 1'b0 : qp_nacked_wr_data),
        .rd_en   (1'b1),
        .rd_addr (qp_nacked_rd_addr_0),
        .rd_data (qp_nacked_rd_data_0)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_nacked_1 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || qp_nacked_wr_en),
        .wr_addr (init_start ? init_count : qp_nacked_wr_addr),
        .wr_data (init_start ? 1'b0 : qp_nacked_wr_data),
        .rd_en   (1'b1),
        .rd_addr (qp_nacked_rd_addr_1),
        .rd_data (qp_nacked_rd_data_1)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_enable_0 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || i_qp_enable_vld),
        .wr_addr (init_start ? init_count : i_qp_enable_idx),
        .wr_data (init_start ? 1'b0 : i_qp_enable_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_Qpid),
        .rd_data (qp_is_enabled)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_fatal_0 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || i_qp_fatal_vld),
        .wr_addr (init_start ? init_count : i_qp_fatal_idx),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_Qpid),
        .rd_data (qp_fatal_rd_data_0)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_under_recovery (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || (i_qp_under_recovery_wr_en && !i_qp_under_recovery_wr_en_r)),
        .wr_addr (init_start ? init_count : i_qp_under_recovery_wr_id),
        .wr_data (init_start ? 1'b0 : i_qp_under_recovery_wr_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_Qpid),
        .rd_data (qp_under_recovery)
    );

    always @(posedge core_clk) begin
        i_qp_under_recovery_wr_en_r <= i_qp_under_recovery_wr_en;
    end

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_fatal_2 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || i_qp_fatal_vld),
        .wr_addr (init_start ? init_count : i_qp_fatal_idx),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_data),
        .rd_en   (1'b1),
        .rd_addr (i_bres_dest_qpid),
        .rd_data (qp_fatal_rd_data_2)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qp_under_recovery_1 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || (i_qp_under_recovery_wr_en && !i_qp_under_recovery_wr_en_r)),
        .wr_addr (init_start ? init_count : i_qp_under_recovery_wr_id),
        .wr_data (init_start ? 1'b0 : i_qp_under_recovery_wr_data),
        .rd_en   (1'b1),
        .rd_addr (i_bres_dest_qpid),
        .rd_data (qp_under_recovery_rd_data)
    );


    assign qp_nack_pending_rd_addr_0 = aeth_bram_wr_en ? aeth_bram_wr_addr : i_bres_dest_qpid_2r;
    assign qp_nacked_rd_addr_0       = aeth_bram_wr_en ? aeth_bram_wr_addr : i_bres_dest_qpid_2r;

    // 
    always @(posedge core_clk) begin
        if (core_rst) begin
            qp_ack_pending_wr_en    <= 1'b0;
            qp_ack_pending_wr_addr  <= {C_MAX_QID_WIDTH{1'b0}};
            qp_ack_pending_wr_data  <= 1'b0;

            qp_nack_pending_wr_en   <= 1'b0;
            qp_nack_pending_wr_addr <= {C_MAX_QID_WIDTH{1'b0}};
            qp_nack_pending_wr_data <= 1'b0;

            qp_nacked_wr_en         <= 1'b0;
            qp_nacked_wr_addr       <= {C_MAX_QID_WIDTH{1'b0}};
            qp_nacked_wr_data       <= 1'b0;
        end else begin
            if (i_reg_xrnic_en && aeth_bram_wr_en) begin
                qp_ack_pending_wr_en    <= 1'b1;
                qp_ack_pending_wr_addr  <= aeth_bram_wr_addr;
                qp_ack_pending_wr_data  <= 1'b1;

                qp_nack_pending_wr_en   <= 1'b1;
                qp_nack_pending_wr_addr <= aeth_bram_wr_addr;
                qp_nack_pending_wr_data <= (|aeth_bram_wr_data[30:29]) || bres_exp_ack || qp_nack_pending_rd_data_0;

                qp_nacked_wr_en         <= 1'b1;
                qp_nacked_wr_addr       <= aeth_bram_wr_addr;
                qp_nacked_wr_data       <= (|aeth_bram_wr_data[30:29]) || qp_nacked_rd_data_0;
            end else if (i_reg_xrnic_en && rd_req_bram_wr_en) begin
                qp_ack_pending_wr_en   <= 1'b1;
                qp_ack_pending_wr_addr <= i_bres_dest_qpid_2r;
                qp_ack_pending_wr_data <= 1'b0;
                if (C_EN_RD_RESP_MIDDLE == 1) begin
                    qp_nack_pending_wr_en   <= 1'b1;
                    qp_nack_pending_wr_addr <= i_bres_dest_qpid_2r;
                    qp_nack_pending_wr_data <= 1'b0;

                    qp_nacked_wr_en         <= 1'b1;
                    qp_nacked_wr_addr       <= i_bres_dest_qpid_2r;
                    qp_nacked_wr_data       <= 1'b0;
                end else begin
                    qp_nack_pending_wr_en   <= 1'b1;
                    qp_nack_pending_wr_addr <= i_bres_dest_qpid_2r;
                    qp_nack_pending_wr_data <= qp_nacked_rd_data_0 ? qp_nack_pending_rd_data_0 : 1'b0;
                end
            end else if (aeth_bram_rd_en) begin
                qp_ack_pending_wr_en    <= 1'b1;
                qp_ack_pending_wr_addr  <= aeth_bram_rd_addr;
                qp_ack_pending_wr_data  <= 1'b0;

                qp_nack_pending_wr_en   <= 1'b1;
                qp_nack_pending_wr_addr <= aeth_bram_rd_addr;
                qp_nack_pending_wr_data <= 1'b0;

                qp_nacked_wr_en         <= 1'b1;
                qp_nacked_wr_addr       <= aeth_bram_rd_addr;
                qp_nacked_wr_data       <= 1'b0;
            end else if (dup_rd_req_bram_wr_en && (C_EN_RD_RESP_MIDDLE == 1)) begin
                qp_ack_pending_wr_en    <= 1'b1;
                qp_ack_pending_wr_addr  <= dup_rd_req_bram_wr_addr[C_MAX_QID_WIDTH+RD_REQ_PTR_WIDTH-1 : RD_REQ_PTR_WIDTH];
                qp_ack_pending_wr_data  <= 1'b0;

                qp_nack_pending_wr_en   <= 1'b1;
                qp_nack_pending_wr_addr <= dup_rd_req_bram_wr_addr[C_MAX_QID_WIDTH+RD_REQ_PTR_WIDTH-1 : RD_REQ_PTR_WIDTH];
                qp_nack_pending_wr_data <= 1'b0;

                qp_nacked_wr_en         <= 1'b1;
                qp_nacked_wr_addr       <= dup_rd_req_bram_wr_addr[C_MAX_QID_WIDTH+RD_REQ_PTR_WIDTH-1 : RD_REQ_PTR_WIDTH];
                qp_nacked_wr_data       <= 1'b0;
            end else if (rst_fifo_rd_en_r && !status_lut_init_done_flag) begin
                qp_ack_pending_wr_en    <= 1'b1;
                qp_ack_pending_wr_addr  <= rst_qpid;
                qp_ack_pending_wr_data  <= 1'b0;

                qp_nack_pending_wr_en   <= 1'b1;
                qp_nack_pending_wr_addr <= rst_qpid;
                qp_nack_pending_wr_data <= 1'b0;

                qp_nacked_wr_en         <= 1'b1;
                qp_nacked_wr_addr       <= rst_qpid;
                qp_nacked_wr_data       <= 1'b0;
            end else begin
                qp_ack_pending_wr_en  <= 1'b0;
                qp_nack_pending_wr_en <= 1'b0;
                qp_nacked_wr_en       <= 1'b0;
            end
        end
    end

    assign status_lut_init_done = (rst_fifo_rd_en_r && !aeth_bram_wr_en && !rd_req_bram_wr_en && !aeth_bram_rd_en && !dup_rd_req_bram_wr_en) ? 1'b1 : 1'b0;
    assign qp_nacked_rd_addr_1  = i_bres_dest_qpid;

    // 用 i_bres_dest_qpid 作为地址，将 {PSN, Syndrome, MSN} 写入 AETH BRAM
    always @(posedge core_clk) begin
        if (core_rst) begin
            aeth_bram_wr_data <= 'd0;
            aeth_bram_wr_en   <= 1'b0;
            aeth_bram_wr_addr <= {C_MAX_QID_WIDTH{1'b0}};
            bres_exp_ack      <= 1'b0;
        end else begin
            aeth_bram_wr_data <= {i_res_psn, 1'b0, i_res_aeth_syndrome, i_res_msn};
            aeth_bram_wr_addr <= i_bres_dest_qpid;
            bres_exp_ack      <= i_bres_exp_ack;
            if (i_reg_xrnic_en && i_req_type == 2'b00) begin
                aeth_bram_wr_en <= i_bres_valid && ~qp_nacked_rd_data_1;
            end else begin
                aeth_bram_wr_en <= 1'b0;
            end
        end
    end


    always @(posedge core_clk) aeth_valid_r1 <= aeth_valid_r;
    always @(posedge core_clk) aeth_valid_r2 <= aeth_valid_r1;

    always @(posedge core_clk) begin
        if (core_rst) begin
            o_aeth_valid <= 0;
        end else begin
            if (aeth_valid_r & !aeth_valid_r1) o_aeth_valid <= 1;
            else if (i_hdr_gen_rdy) o_aeth_valid <= 0;
        end
    end

    generate
        if (C_EN_RECV_RD_WR == 1) begin  // current
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_rd_req_valid             <= 0;
                    o_rd_req_start_with_middle <= 2'b00;
                end else begin
                    if (rd_req_valid_r & !rd_req_valid_r1) begin
                        o_rd_req_valid             <= 1;
                        o_rd_req_start_with_middle <= start_with_middle_r1;
                    end else if (i_hdr_gen_rdy) begin
                        o_rd_req_valid             <= 0;
                        o_rd_req_start_with_middle <= 2'b00;
                    end
                end
            end
        end
    endgenerate

    always @(posedge core_clk) begin
        rd_req_valid_r2 <= rd_req_valid_r1;
        rd_req_valid_r1 <= rd_req_valid_r;
        fetch_next_id_r <= fetch_next_id;
        send_response_r <= send_response;
    end

    assign rd_req_bram_rd_en_ored     = rd_req_bram_rd_en;
    assign dup_rd_req_bram_rd_en_ored = dup_rd_req_bram_rd_en;
    assign qp_nack_pending_rd_addr_1  = aeth_bram_rd_addr;

    always @(posedge core_clk) begin
        if (core_rst) begin
            aeth_bram_rd_en   <= 1'b0;
            aeth_bram_rd_en_r <= 1'b0;
            aeth_valid_r      <= 1'b0;
            aeth_bram_rd_addr <= {C_MAX_QID_WIDTH{1'b0}};
            arb_in_progress   <= 1'b1;
        end else begin
            //aeth_bram_rd_en_r <= aeth_bram_rd_en & (~qp_fatal_rd_data_0 | qp_nack_pending_rd_data_1);
            aeth_bram_rd_en_r <= aeth_bram_rd_en;
            if(~arb_in_progress && ~aeth_valid_r && ~aeth_bram_rd_en && ~aeth_bram_rd_en_r && ~rd_req_valid_r && dup_rd_req_fifo_empty_rd_data && rd_req_fifo_empty_rd_data && ~rd_req_bram_rd_en_ored && 
			 ~dup_rd_req_bram_rd_en_ored && ~rd_req_bram_rd_en_r && ~dup_rd_req_bram_rd_en_r && ~dup_rd_req_en && ~(i_bres_valid && i_req_type == 2'b00 && ~qp_nacked_rd_data_1) && 
			 ~(i_bres_valid_r && i_req_type_r != 2'b00)) begin
                aeth_bram_rd_en <= 1'b1;
            end else begin
                aeth_bram_rd_en <= 1'b0;
            end

            if ((send_response && ~fetch_next_id) || (fetch_next_id_r && send_response_r)) begin
                aeth_bram_rd_addr <= arbitrated_Qpid;
            end

            if (fetch_next_id) begin
                arb_in_progress <= 1'b1;
            end else if (send_response) begin
                arb_in_progress <= 1'b0;
                //if both pulse come at the same clock
            end else if (fetch_next_id_r && send_response_r) begin
                arb_in_progress <= 1'b0;
            end

            if (C_EN_RECV_RD_WR == 1) begin
                if (aeth_bram_rd_en_r) begin
                    aeth_valid_r <= 1'b1;
                end else if (rd_req_bram_rd_en_r && ~dup_rd_req_bram_rd_en_r) begin
                    aeth_valid_r <= 1'b0;
                end else if (~rd_req_bram_rd_en_r && dup_rd_req_bram_rd_en_r) begin
                    aeth_valid_r <= 1'b0;
                end else if (rd_req_bram_rd_en_r && dup_rd_req_bram_rd_en_r) begin
                    aeth_valid_r <= 1'b0;
                end else if (i_hdr_gen_rdy) begin
                    aeth_valid_r <= 1'b0;
                end
            end else begin
                if (aeth_bram_rd_en_r) begin
                    aeth_valid_r <= 1'b1;
                end else if (i_hdr_gen_rdy) begin
                    aeth_valid_r <= 1'b0;
                end
            end

        end
    end

    always @(posedge core_clk) begin
        if (C_EN_RECV_RD_WR == 1) begin  // current
            if (core_rst) begin
                o_aeth_hdr  <= 32'd0;
                o_aeth_qpid <= 'd0;
                o_aeth_psn  <= 'd0;
            end else begin
                if (aeth_bram_rd_en_r) begin
                    o_aeth_hdr  <= aeth_bram_rd_data[31:0];
                    o_aeth_qpid <= aeth_bram_rd_addr;
                    o_aeth_psn  <= aeth_bram_rd_data[55:32];
                end else if (rd_req_bram_rd_en_r1 && ~dup_rd_req_bram_rd_en_r1) begin
                    o_aeth_hdr  <= rd_req_bram_rd_data[31:0];
                    o_aeth_qpid <= aeth_bram_rd_addr;
                    o_aeth_psn  <= rd_req_bram_rd_data[55:32];
                end else if (~rd_req_bram_rd_en_r1 && dup_rd_req_bram_rd_en_r1) begin
                    o_aeth_hdr  <= dup_rd_req_bram_rd_data[31:0];
                    o_aeth_qpid <= aeth_bram_rd_addr;
                    o_aeth_psn  <= dup_rd_req_bram_rd_data[55:32];
                end else if (rd_req_bram_rd_en_r1 && dup_rd_req_bram_rd_en_r1) begin
                    o_aeth_qpid <= aeth_bram_rd_addr;
                    if (dup_rd_req_bram_rd_data[55:32] < rd_req_bram_rd_data[55:32]) begin
                        o_aeth_hdr <= dup_rd_req_bram_rd_data[31:0];
                        o_aeth_psn <= dup_rd_req_bram_rd_data[55:32];
                    end else begin
                        o_aeth_hdr <= rd_req_bram_rd_data[31:0];
                        o_aeth_psn <= rd_req_bram_rd_data[55:32];
                    end
                end
            end
        end else begin  // C_EN_RECV_RD_WR != 1
            if (core_rst) begin
                o_aeth_hdr  <= 32'd0;
                o_aeth_qpid <= 'd0;
                o_aeth_psn  <= 'd0;
            end else begin
                if (aeth_bram_rd_en_r) begin
                    o_aeth_hdr  <= aeth_bram_rd_data[31:0];
                    o_aeth_qpid <= aeth_bram_rd_addr;
                    o_aeth_psn  <= aeth_bram_rd_data[55:32];
                end
            end
        end
    end

    always @(posedge core_clk) begin
        rd_req_bram_rd_en_r1     <= rd_req_bram_rd_en_r;
        dup_rd_req_bram_rd_en_r1 <= dup_rd_req_bram_rd_en_r;
        start_with_middle_r1     <= start_with_middle_r;
        aeth_bram_rd_en_r1       <= aeth_bram_rd_en_r;
    end

    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////// Block RAM for Header pointer and buffer address /////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (56 * C_MAX_QP),    //positive integer
        .MEMORY_PRIMITIVE ("ultra"),          //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                //Do not Change

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(56),              //positive integer
        .BYTE_WRITE_WIDTH_A(8),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_MAX_QID_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (56),               //positive integer
        .ADDR_WIDTH_B      (C_MAX_QID_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //string
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) hdr_ptr_bram (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (aeth_bram_wr_en),
        .wea           ({7{1'b1}}),
        .addra         (aeth_bram_wr_addr),
        .dina          (aeth_bram_wr_data),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (core_rst),
        .enb     (aeth_bram_rd_en),
        .regceb  (1'b1),
        .addrb   (aeth_bram_rd_addr),
        .doutb   (aeth_bram_rd_data),
        .sbiterrb(),
        .dbiterrb()

    );


    /******************* FIFO to store pointer Initialization requests for disabled QPs *************************************************/
    xpm_memory_sdpram #(
        .MEMORY_SIZE       (C_MAX_QID_WIDTH * C_MAX_QP),
        .MEMORY_PRIMITIVE  ("auto"),
        .CLOCKING_MODE     ("common_clock"),
        .MEMORY_INIT_FILE  ("none"),
        .MEMORY_INIT_PARAM (""),
        .USE_MEM_INIT      (0),
        .WAKEUP_TIME       ("disable_sleep"),
        .MESSAGE_CONTROL   (0),
        .ECC_MODE          ("no_ecc"),
        .AUTO_SLEEP_TIME   (0),
        .WRITE_DATA_WIDTH_A(C_MAX_QID_WIDTH),
        .BYTE_WRITE_WIDTH_A(C_MAX_QID_WIDTH),
        .ADDR_WIDTH_A      (C_MAX_QID_WIDTH),

        .READ_DATA_WIDTH_B (C_MAX_QID_WIDTH),
        .ADDR_WIDTH_B      (C_MAX_QID_WIDTH),
        .READ_RESET_VALUE_B("0"),
        .READ_LATENCY_B    (1),
        .WRITE_MODE_B      ("read_first")
    ) inst_rst_QPids (
        .clka  (core_clk),
        .ena   (i_qp_enable_vld && !i_qp_enable_data),
        .wea   (1'b1),
        .addra (Qpid_rst_buf_wr_addr[C_MAX_QID_WIDTH-1 : 0]),
        .dina  (i_qp_enable_idx),
        .clkb  (core_clk),
        .rstb  (core_rst),
        .regceb(1'b1),
        .enb   (rst_fifo_rd_en),
        .addrb (Qpid_rst_buf_rd_addr[C_MAX_QID_WIDTH-1 : 0]),
        .doutb (rst_qpid)
    );

    assign rst_fifo_empty = (Qpid_rst_buf_wr_addr == Qpid_rst_buf_rd_addr) ? 1'b1 : 1'b0;
    assign rst_fifo_full  = (Qpid_rst_buf_wr_addr[C_MAX_QID_WIDTH] != Qpid_rst_buf_rd_addr[C_MAX_QID_WIDTH] && Qpid_rst_buf_wr_addr[C_MAX_QID_WIDTH-1 : 0] == Qpid_rst_buf_rd_addr[C_MAX_QID_WIDTH-1 : 0]) ? 1'b1 : 1'b0;
    assign rst_fifo_rd_en = !rst_fifo_empty && !(ptr_init_done_flag && status_lut_init_done_flag);

    always @(posedge core_clk) begin
        wr_ptr_buf_wren_init_r <= wr_ptr_buf_wren_init;
        rst_fifo_rd_en_r       <= rst_fifo_rd_en;
    end

    always @(posedge core_clk) begin : QP_reset_pointer_Init_handling
        if (core_rst) begin
            Qpid_rst_buf_wr_addr      <= 'd0;
            Qpid_rst_buf_rd_addr      <= 'd0;
            ptr_init_done_flag        <= 1'b0;
            status_lut_init_done_flag <= 1'b0;
        end else begin
            if (i_qp_enable_vld && !i_qp_enable_data) begin
                Qpid_rst_buf_wr_addr <= Qpid_rst_buf_wr_addr + 1;
            end

            if (ptr_init_done_flag && status_lut_init_done_flag) begin
                Qpid_rst_buf_rd_addr      <= Qpid_rst_buf_rd_addr + 1;
                ptr_init_done_flag        <= 1'b0;
                status_lut_init_done_flag <= 1'b0;
            end else begin
                if (ptr_init_done) begin
                    ptr_init_done_flag <= 1'b1;
                end

                if (status_lut_init_done) begin
                    status_lut_init_done_flag <= 1'b1;
                end
            end
        end
    end


    generate
        if (C_EN_RECV_RD_WR == 1 && C_EN_RD_RESP_MIDDLE == 1) begin : EN_RD_WR_MIDDLE  // current

            genvar j;

            /////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////// Read request buffer pointers logic ///////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////


            reg                          dup_rd_req_compare_done;
            reg  [                  1:0] dup_rd_req_compare_cs;
            reg  [                  1:0] start_with_middle;
            reg  [   RD_REQ_PTR_WIDTH:0] dup_rd_req_compare_cnt;
            reg  [ RD_REQ_PTR_WIDTH-1:0] dup_rd_req_compare_match_ptr;
            reg  [                 23:0] prev_rd_req_psn;
            reg                          dup_rd_req_compare_wait;
            reg                          dup_rd_req_compare_wait_r;

            reg  [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_wr_addr;
            wire [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_wr_addr_mux;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_wr_ptr_buf_wr_addr_mux;
            reg  [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_wr_addr_init;
            wire [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_rd_addr;
            wire [ RD_REQ_PTR_WIDTH : 0] wr_ptr_buf_wr_data;
            wire [ RD_REQ_PTR_WIDTH : 0] wr_ptr_buf_rd_data;
            reg  [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_wr_addr;
            reg  [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_wr_addr_init;
            wire [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_rd_addr;
            reg  [ RD_REQ_PTR_WIDTH : 0] rd_ptr_buf_wr_data;
            wire [ RD_REQ_PTR_WIDTH : 0] rd_ptr_buf_rd_data;
            wire [ RD_REQ_PTR_WIDTH : 0] mirror_wr_ptr_buf_rd_data;
            wire [ RD_REQ_PTR_WIDTH : 0] mirror_rd_ptr_buf_rd_data;
            reg                          wr_ptr_buf_wren;
            reg                          wr_ptr_buf_wren_r;
            wire                         wr_ptr_buf_rden;
            reg                          rd_ptr_buf_wren;
            reg                          rd_ptr_buf_wren_init;
            reg  [ RD_REQ_PTR_WIDTH : 0] rd_ptr_buf_rd_data_sampled;
            reg  [C_MAX_QID_WIDTH-1 : 0] rd_init_qpid;

            reg  [C_MAX_QID_WIDTH-1 : 0] dup_wr_ptr_buf_wr_addr;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_wr_ptr_buf_rd_addr;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_wr_ptr_buf_wr_data;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_wr_ptr_buf_rd_data;
            reg  [C_MAX_QID_WIDTH-1 : 0] dup_rd_ptr_buf_wr_addr;
            reg  [C_MAX_QID_WIDTH-1 : 0] dup_rd_ptr_buf_wr_addr_init;
            reg  [ RD_REQ_PTR_WIDTH : 0] dup_rd_ptr_buf_wr_data;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_rd_ptr_buf_rd_data;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_mirror_wr_ptr_buf_rd_data;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_mirror_rd_ptr_buf_rd_data;
            reg                          dup_wr_ptr_buf_wren;
            reg                          dup_wr_ptr_buf_wren_r;
            wire                         dup_wr_ptr_buf_rden;
            reg                          dup_rd_ptr_buf_wren;
            wire                         dup_rd_ptr_buf_rden;
            reg  [ RD_REQ_PTR_WIDTH : 0] dup_rd_ptr_buf_rd_data_sampled;
            reg  [ RD_REQ_PTR_WIDTH : 0] dup_mirror_wr_ptr_buf_rd_data_sampled;
            reg                          ignore_full_check;
            reg                          ignore_dup_full_check;
            wire [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_addr_A;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_wr_ptr_buf_addr_A;
            wire [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_addr_A;
            wire [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_addr_B;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_rd_ptr_buf_addr_A;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_rd_ptr_buf_addr_B;
            reg                          last_fatal_status;
            reg                          last_full_status;
            reg                          last_dup_full_status;


            assign wr_ptr_buf_addr_A      = wr_ptr_buf_rden ? wr_ptr_buf_rd_addr : wr_ptr_buf_wr_addr_mux;
            assign dup_wr_ptr_buf_addr_A  = dup_wr_ptr_buf_rden ? dup_wr_ptr_buf_rd_addr : dup_wr_ptr_buf_wr_addr_mux;
            assign rd_ptr_buf_addr_A      = arb_in_progress ? rd_ptr_buf_rd_addr : rd_ptr_buf_wr_addr;
            assign dup_rd_ptr_buf_addr_A  = arb_in_progress ? rd_ptr_buf_rd_addr : dup_rd_ptr_buf_wr_addr;
            assign rd_ptr_buf_addr_B      = i_bres_valid ? i_bres_dest_qpid : wr_ptr_buf_wr_addr_init;
            assign dup_rd_ptr_buf_addr_B  = i_bres_valid ? i_bres_dest_qpid : wr_ptr_buf_wr_addr_init;

            assign wr_ptr_buf_rd_addr     = i_bres_dest_qpid;
            assign wr_ptr_buf_rden        = i_bres_valid;
            assign dup_wr_ptr_buf_rd_addr = i_bres_dest_qpid;
            assign dup_wr_ptr_buf_rden    = i_bres_valid;
            assign rd_ptr_buf_rd_addr     = arbitrated_Qpid;

            always @(posedge core_clk) begin
                if (core_rst) begin
                    rd_req_fifo_empty_wr_en_0   <= 1'b0;
                    rd_req_fifo_empty_wr_addr_0 <= {C_MAX_QID_WIDTH{1'b0}};
                    rd_req_fifo_empty_wr_en_1   <= 1'b0;
                    rd_req_fifo_empty_wr_addr_1 <= {C_MAX_QID_WIDTH{1'b0}};
                    rd_req_fifo_full_wr_en_0    <= 1'b0;
                    rd_req_fifo_full_wr_addr_0  <= {C_MAX_QID_WIDTH{1'b0}};
                    rd_req_fifo_full_wr_en_1    <= 1'b0;
                    rd_req_fifo_full_wr_addr_1  <= {C_MAX_QID_WIDTH{1'b0}};
                end else begin
                    if (wr_ptr_buf_wren) begin
                        rd_req_fifo_empty_wr_en_0   <= 1'b1;
                        rd_req_fifo_empty_wr_addr_0 <= wr_ptr_buf_wr_addr;
                        rd_req_fifo_empty_wr_data_0 <= 1'b0;
                    end else if (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) begin
                        rd_req_fifo_empty_wr_en_0   <= 1'b1;
                        rd_req_fifo_empty_wr_addr_0 <= wr_ptr_buf_wr_addr_init;
                        rd_req_fifo_empty_wr_data_0 <= 1'b1;
                    end else begin
                        rd_req_fifo_empty_wr_en_0 <= 1'b0;
                    end
                    if (rd_ptr_buf_wren && rd_ptr_buf_wr_data == mirror_wr_ptr_buf_rd_data && ~((wr_ptr_buf_wren || wr_ptr_buf_wren_r) && wr_ptr_buf_wr_addr == rd_ptr_buf_wr_addr)) begin
                        rd_req_fifo_empty_wr_en_1   <= 1'b1;
                        rd_req_fifo_empty_wr_addr_1 <= rd_ptr_buf_wr_addr;
                        rd_req_fifo_empty_wr_data_1 <= 1'b1;
                    end else begin
                        rd_req_fifo_empty_wr_en_1 <= 1'b0;
                    end


                    if (~ignore_full_check && ~(rd_ptr_buf_wren && (wr_ptr_buf_wr_addr == rd_ptr_buf_wr_addr)) && wr_ptr_buf_wren && 
			(wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] == mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]) && (wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH] != mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH])) begin
                        rd_req_fifo_full_wr_en_1   <= 1'b1;
                        rd_req_fifo_full_wr_addr_1 <= wr_ptr_buf_wr_addr;
                    end else begin
                        rd_req_fifo_full_wr_en_1 <= 1'b0;
                    end

                    if (rd_ptr_buf_wren && ~(wr_ptr_buf_wren && (rd_ptr_buf_wr_addr == wr_ptr_buf_wr_addr))) begin
                        rd_req_fifo_full_wr_en_0   <= 1'b1;
                        rd_req_fifo_full_wr_addr_0 <= rd_ptr_buf_wr_addr;
                        // when QP is re-initialized, full should be reset for that QP
                    end else if (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) begin
                        rd_req_fifo_full_wr_en_0   <= 1'b1;
                        rd_req_fifo_full_wr_addr_0 <= wr_ptr_buf_wr_addr_init;
                    end else begin
                        rd_req_fifo_full_wr_en_0 <= 1'b0;
                    end
                end
            end

            //CR-1112580 fix requires overflow info on the just next clock
            assign rd_req_fifo_full_set_valid   = (!ignore_full_check && !(rd_ptr_buf_wren && (wr_ptr_buf_wr_addr == rd_ptr_buf_wr_addr)) && wr_ptr_buf_wren && 
			                          (wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] == mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]) && 
									  (wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH] != mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH])) ? 1'b1 : 1'b0;
            assign rd_req_fifo_full_set_id = wr_ptr_buf_wr_addr;
            assign rd_req_fifo_full_reset_valid = ((rd_ptr_buf_wren && !(wr_ptr_buf_wren && (rd_ptr_buf_wr_addr == wr_ptr_buf_wr_addr))) || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)) ? 1'b1 : 1'b0;
            assign rd_req_fifo_full_reset_id = (rd_ptr_buf_wren && !(wr_ptr_buf_wren && (rd_ptr_buf_wr_addr == wr_ptr_buf_wr_addr))) ? rd_ptr_buf_wr_addr : wr_ptr_buf_wr_addr_init;



            // To reduce a clk delay between rd_en and wr_en for these brams 
            assign wr_ptr_buf_wr_data = (wr_ptr_buf_wren) ? (wr_ptr_buf_rd_data + {{RD_REQ_PTR_WIDTH{1'b0}}, 1'b1}) : ({RD_REQ_PTR_WIDTH + 1{1'b0}});
            assign dup_wr_ptr_buf_wr_data = (dup_wr_ptr_buf_wren) ? (dup_wr_ptr_buf_rd_data + {{RD_REQ_PTR_WIDTH{1'b0}}, 1'b1}) : ({RD_REQ_PTR_WIDTH + 1{1'b0}});
            assign wr_ptr_buf_wr_addr_mux = (wr_ptr_buf_wren) ? wr_ptr_buf_wr_addr : wr_ptr_buf_wr_addr_init;
            assign dup_wr_ptr_buf_wr_addr_mux = (dup_wr_ptr_buf_wren) ? dup_wr_ptr_buf_wr_addr : wr_ptr_buf_wr_addr_init;
            assign ptr_init_done = (wr_ptr_buf_wren == 1'b0 && wr_ptr_buf_wren_init == 1'b1 && rd_ptr_buf_wren == 1'b0 && dup_rd_ptr_buf_wren == 1'b0 && dup_wr_ptr_buf_wren == 1'b0) ? 1'b1 : 1'b0;

            always @(posedge core_clk) begin
                if (core_rst) begin
                    wr_ptr_buf_wr_addr      <= {C_MAX_QID_WIDTH{1'b0}};
                    wr_ptr_buf_wr_addr_init <= {C_MAX_QID_WIDTH{1'b0}};
                    wr_ptr_buf_wren         <= 1'b0;
                    wr_ptr_buf_wren_r       <= 1'b0;
                    dup_wr_ptr_buf_wren_r   <= 1'b0;
                    dup_wr_ptr_buf_wr_addr  <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_wr_ptr_buf_wren     <= 1'b0;
                    ignore_full_check       <= 1'b0;
                    ignore_dup_full_check   <= 1'b0;
                    wr_ptr_buf_wren_init    <= 1'b0;
                end else begin
                    wr_ptr_buf_wren_r     <= wr_ptr_buf_wren;
                    dup_wr_ptr_buf_wren_r <= dup_wr_ptr_buf_wren;
                    if (rd_ptr_buf_wren && (i_bres_dest_qpid == rd_ptr_buf_addr_A) && i_bres_valid && i_reg_xrnic_en && (i_req_type == 2'b01) && ~(qp_fatal_rd_data_2 || qp_under_recovery_rd_data) && ~rd_req_fifo_full_rd_data) begin
                        ignore_full_check <= 1'b1;
                    end else begin
                        ignore_full_check <= 1'b0;
                    end

                    if (i_bres_valid && i_reg_xrnic_en && (i_req_type == 2'b01) && ~(qp_fatal_rd_data_2 || qp_under_recovery_rd_data) && ~rd_req_fifo_full_rd_data) begin
                        wr_ptr_buf_wr_addr <= i_bres_dest_qpid;
                        wr_ptr_buf_wren    <= 1'b1;
                    end else begin
                        wr_ptr_buf_wren <= 1'b0;
                        if (!wr_ptr_buf_wren_init && rst_fifo_rd_en_r && !ptr_init_done_flag && !rd_req_bram_rd_en_r && !(dup_rd_req_bram_rd_en_r || discard_rd_req)) begin
                            wr_ptr_buf_wr_addr_init <= rst_qpid;
                            wr_ptr_buf_wren_init    <= 1'b1;
                            // Init to both write and read ptr brams should be simultaneously; write will happen when all bram ports are not busy and write is possible; else wait.. 
                        end else if (wr_ptr_buf_wren == 1'b0 && wr_ptr_buf_wren_init == 1'b1 && rd_ptr_buf_wren == 1'b0 && dup_rd_ptr_buf_wren == 1'b0 && dup_wr_ptr_buf_wren == 1'b0) begin
                            wr_ptr_buf_wren_init <= 1'b0;
                        end
                    end

                    if (dup_rd_ptr_buf_wren && (i_bres_dest_qpid == dup_rd_ptr_buf_addr_A) && i_reg_xrnic_en && i_bres_valid && (i_req_type == 2'b10) && ~dup_rd_req_fifo_full_rd_data) begin
                        ignore_dup_full_check <= 1'b1;
                    end else begin
                        ignore_dup_full_check <= 1'b0;
                    end

                    if (i_reg_xrnic_en && i_bres_valid && (i_req_type == 2'b10) && ~dup_rd_req_fifo_full_rd_data) begin
                        dup_wr_ptr_buf_wren    <= 1'b1;
                        dup_wr_ptr_buf_wr_addr <= i_bres_dest_qpid;
                    end else begin
                        dup_wr_ptr_buf_wren <= 1'b0;
                    end
                end
            end




            /***************************************************************************
            *                                                                          *
            * 									      *
            * 			TDP memory -- for pointer storage		      *
            * 									      *
            * 									      *
            ***************************************************************************/
            xpm_memory_tdpram #(
                .ADDR_WIDTH_A           (C_MAX_QID_WIDTH),
                .ADDR_WIDTH_B           (C_MAX_QID_WIDTH),
                .AUTO_SLEEP_TIME        (0),
                .BYTE_WRITE_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .BYTE_WRITE_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .CASCADE_HEIGHT         (0),
                .CLOCKING_MODE          ("common_clock"),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_MAX_QP * (RD_REQ_PTR_WIDTH + 1)),
                .MESSAGE_CONTROL        (0),
                .READ_DATA_WIDTH_A      (RD_REQ_PTR_WIDTH + 1),
                .READ_DATA_WIDTH_B      (RD_REQ_PTR_WIDTH + 1),
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
                .WRITE_DATA_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_DATA_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_MODE_A           ("read_first"),
                .WRITE_MODE_B           ("read_first")
            ) inst_tdp_wr_ptr_buf (
                .clka          (core_clk),
                .rsta          (core_rst),
                .regcea        (1'b1),
                .ena           (wr_ptr_buf_wren || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) || wr_ptr_buf_rden),
                .wea           ((wr_ptr_buf_wren || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)) && (~wr_ptr_buf_rden)),
                .addra         (wr_ptr_buf_addr_A),
                .dina          (wr_ptr_buf_wr_data),
                .douta         (wr_ptr_buf_rd_data),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .regceb        (1'b1),
                .enb           (~arb_in_progress),
                .web           (1'b0),
                .addrb         (aeth_bram_rd_addr),
                .dinb          (wr_ptr_buf_wr_data),
                .doutb         (mirror_wr_ptr_buf_rd_data),
                .injectdbiterra(1'b0),
                .injectdbiterrb(1'b0),
                .injectsbiterra(1'b0),
                .injectsbiterrb(1'b0),
                .sleep         (1'b0),
                .sbiterra      (),
                .sbiterrb      (),
                .dbiterra      (),
                .dbiterrb      ()
            );


            xpm_memory_tdpram #(
                .ADDR_WIDTH_A           (C_MAX_QID_WIDTH),
                .ADDR_WIDTH_B           (C_MAX_QID_WIDTH),
                .AUTO_SLEEP_TIME        (0),
                .BYTE_WRITE_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .BYTE_WRITE_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .CASCADE_HEIGHT         (0),
                .CLOCKING_MODE          ("common_clock"),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_MAX_QP * (RD_REQ_PTR_WIDTH + 1)),
                .MESSAGE_CONTROL        (0),
                .READ_DATA_WIDTH_A      (RD_REQ_PTR_WIDTH + 1),
                .READ_DATA_WIDTH_B      (RD_REQ_PTR_WIDTH + 1),
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
                .WRITE_DATA_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_DATA_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_MODE_A           ("read_first"),
                .WRITE_MODE_B           ("read_first")
            ) inst_tdp_rd_ptr_buf (
                .clka          (core_clk),
                .rsta          (core_rst),
                .regcea        (1'b1),
                .ena           (rd_ptr_buf_wren || arb_in_progress),
                .wea           (rd_ptr_buf_wren && (~arb_in_progress)),
                .addra         (rd_ptr_buf_addr_A),
                .dina          (rd_ptr_buf_wr_data),
                .douta         (rd_ptr_buf_rd_data),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .regceb        (1'b1),
                .enb           (i_bres_valid || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)),     //To avoid write conflict on both ports
                .web           ((~i_bres_valid) && (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)),
                .addrb         (rd_ptr_buf_addr_B),
                .dinb          ({RD_REQ_PTR_WIDTH + 1{1'b0}}),
                .doutb         (mirror_rd_ptr_buf_rd_data),
                .injectdbiterra(1'b0),
                .injectdbiterrb(1'b0),
                .injectsbiterra(1'b0),
                .injectsbiterrb(1'b0),
                .sleep         (1'b0),
                .sbiterra      (),
                .sbiterrb      (),
                .dbiterra      (),
                .dbiterrb      ()
            );


            xpm_memory_tdpram #(
                .ADDR_WIDTH_A           (C_MAX_QID_WIDTH),
                .ADDR_WIDTH_B           (C_MAX_QID_WIDTH),
                .AUTO_SLEEP_TIME        (0),
                .BYTE_WRITE_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .BYTE_WRITE_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .CASCADE_HEIGHT         (0),
                .CLOCKING_MODE          ("common_clock"),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_MAX_QP * (RD_REQ_PTR_WIDTH + 1)),
                .MESSAGE_CONTROL        (0),
                .READ_DATA_WIDTH_A      (RD_REQ_PTR_WIDTH + 1),
                .READ_DATA_WIDTH_B      (RD_REQ_PTR_WIDTH + 1),
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
                .WRITE_DATA_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_DATA_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_MODE_A           ("read_first"),
                .WRITE_MODE_B           ("read_first")
            ) inst_dup_tdp_wr_ptr_buf (
                .clka          (core_clk),
                .rsta          (core_rst),
                .regcea        (1'b1),
                .ena           (dup_wr_ptr_buf_wren || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) || dup_wr_ptr_buf_rden),
                .wea           ((dup_wr_ptr_buf_wren || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)) && (~dup_wr_ptr_buf_rden)),
                .addra         (dup_wr_ptr_buf_addr_A),
                .dina          (dup_wr_ptr_buf_wr_data),
                .douta         (dup_wr_ptr_buf_rd_data),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .regceb        (1'b1),
                .enb           (~arb_in_progress),
                .web           (1'b0),
                .addrb         (aeth_bram_rd_addr),
                .dinb          (dup_wr_ptr_buf_wr_data),
                .doutb         (dup_mirror_wr_ptr_buf_rd_data),
                .injectdbiterra(1'b0),
                .injectdbiterrb(1'b0),
                .injectsbiterra(1'b0),
                .injectsbiterrb(1'b0),
                .sleep         (1'b0),
                .sbiterra      (),
                .sbiterrb      (),
                .dbiterra      (),
                .dbiterrb      ()
            );

            xpm_memory_tdpram #(
                .ADDR_WIDTH_A           (C_MAX_QID_WIDTH),
                .ADDR_WIDTH_B           (C_MAX_QID_WIDTH),
                .AUTO_SLEEP_TIME        (0),
                .BYTE_WRITE_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .BYTE_WRITE_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .CASCADE_HEIGHT         (0),
                .CLOCKING_MODE          ("common_clock"),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_MAX_QP * (RD_REQ_PTR_WIDTH + 1)),
                .MESSAGE_CONTROL        (0),
                .READ_DATA_WIDTH_A      (RD_REQ_PTR_WIDTH + 1),
                .READ_DATA_WIDTH_B      (RD_REQ_PTR_WIDTH + 1),
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
                .WRITE_DATA_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_DATA_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_MODE_A           ("read_first"),
                .WRITE_MODE_B           ("read_first")
            ) inst_dup_tdp_rd_ptr_buf (
                .clka          (core_clk),
                .rsta          (core_rst),
                .regcea        (1'b1),
                .ena           (dup_rd_ptr_buf_wren || arb_in_progress),
                .wea           (dup_rd_ptr_buf_wren && (~arb_in_progress)),
                .addra         (dup_rd_ptr_buf_addr_A),
                .dina          (dup_rd_ptr_buf_wr_data),
                .douta         (dup_rd_ptr_buf_rd_data),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .regceb        (1'b1),
                .enb           (i_bres_valid || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)),
                .web           ((~i_bres_valid) && (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)),
                .addrb         (dup_rd_ptr_buf_addr_B),
                .dinb          ({RD_REQ_PTR_WIDTH + 1{1'b0}}),
                .doutb         (dup_mirror_rd_ptr_buf_rd_data),
                .injectdbiterra(1'b0),
                .injectdbiterrb(1'b0),
                .injectsbiterra(1'b0),
                .injectsbiterrb(1'b0),
                .sleep         (1'b0),
                .sbiterra      (),
                .sbiterrb      (),
                .dbiterra      (),
                .dbiterrb      ()
            );


            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////// END read response data write logic using BRAM for Write pointer /////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////


            always @(posedge core_clk) begin
                if (core_rst) begin
                    rd_req_bram_wr_en            <= 1'b0;
                    rd_req_bram_rd_en            <= 1'b0;
                    rd_req_bram_rd_en_r          <= 1'b0;
                    rd_req_bram_wr_addr          <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                    rd_req_bram_rd_addr          <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                    rd_req_bram_wr_data          <= 'd0;
                    rd_req_bram_wr_data_r1       <= 'd0;
                    rd_req_valid_r               <= 1'b0;
                    start_with_middle            <= 2'b00;
                    discard_rd_req               <= 1'b0;
                    discard_rd_req_r             <= 1'b0;
                    dup_rd_req_compare_cs        <= 2'b00;
                    prev_rd_req_psn              <= 24'd0;
                    dup_rd_req_compare_done      <= 1'b0;
                    dup_rd_req_compare_cnt       <= {RD_REQ_PTR_WIDTH + 1{1'b0}};
                    dup_rd_req_compare_match_ptr <= {RD_REQ_PTR_WIDTH{1'b0}};
                    dup_rd_req_compare_wait      <= 1'b0;
                    dup_rd_req_compare_wait_r    <= 1'b0;
                    rd_ptr_buf_rd_data_sampled   <= 'd0;
                    //mirror_wr_ptr_buf_rd_data_sampled <= 'd0;
                    rd_ptr_buf_wren              <= 1'b0;
                    rd_init_qpid                 <= {C_MAX_QID_WIDTH{1'b0}};
                    rd_ptr_buf_wren_init         <= 1'b0;
                    rd_ptr_buf_wr_addr_init      <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_rd_ptr_buf_wr_addr_init  <= {C_MAX_QID_WIDTH{1'b0}};
                    last_fatal_status            <= 1'b0;
                    last_full_status             <= 1'b0;
                    //last_wr_ignore <= 1'b0;
                end else begin
                    discard_rd_req_r          <= discard_rd_req;
                    dup_rd_req_compare_wait_r <= dup_rd_req_compare_wait;
                    rd_req_bram_wr_data_r1    <= {i_rsp_last_psn, i_rd_req_phy_addr, i_rd_payload_len, i_res_psn, 1'b0, i_res_aeth_syndrome, i_res_msn};
                    rd_req_bram_wr_data       <= rd_req_bram_wr_data_r1;
                    last_fatal_status         <= qp_fatal_rd_data_2 || qp_under_recovery_rd_data;
                    last_full_status          <= rd_req_fifo_full_rd_data;

                    if (i_bres_valid_r && i_reg_xrnic_en && (i_req_type_r == 2'b01) && ~last_fatal_status && ~last_full_status) begin
                        rd_req_bram_wr_en   <= 1'b1;
                        rd_req_bram_wr_addr <= ((C_MAX_RD_OS * i_bres_dest_qpid_r) + wr_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]);
                    end else begin
                        rd_req_bram_wr_en <= 1'b0;
                    end
                    rd_req_bram_rd_en_r <= rd_req_bram_rd_en && ~dup_rd_req_en && ~dup_rd_req_bram_rd_en;
                    if (~arb_in_progress && ~aeth_valid_r && ~dup_rd_req_en && ~dup_rd_req_bram_rd_en_r && ~dup_rd_req_bram_rd_en && ~rd_req_bram_rd_en_r && ~rd_req_bram_rd_en && ~rd_req_valid_r && ~rd_req_fifo_empty_rd_data && ~discard_rd_req_r && ~aeth_bram_rd_en && ~aeth_bram_rd_en_r) begin
                        rd_req_bram_rd_en          <= 1'b1;
                        rd_req_bram_rd_addr        <= (aeth_bram_rd_addr * C_MAX_RD_OS) + rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0];
                        rd_ptr_buf_rd_data_sampled <= rd_ptr_buf_rd_data;
                    end else if (dup_rd_req_en && ~dup_rd_req_compare_done) begin
                        rd_req_bram_rd_en   <= 1'b1;
                        rd_req_bram_rd_addr <= (aeth_bram_rd_addr * C_MAX_RD_OS) + dup_rd_req_compare_cnt[RD_REQ_PTR_WIDTH-1:0];
                    end else begin
                        rd_req_bram_rd_en <= 1'b0;
                    end

                    prev_rd_req_psn <= rd_req_bram_rd_data[55:32];
                    case (dup_rd_req_compare_cs)
                        2'b00: begin
                            dup_rd_req_compare_done <= 1'b0;
                            dup_rd_req_compare_wait <= 1'b0;
                            dup_rd_req_compare_cnt  <= {RD_REQ_PTR_WIDTH + 1{1'b0}};
                            discard_rd_req          <= 1'b0;
                            start_with_middle       <= 2'b00;
                            if (dup_rd_req_en && ~dup_rd_req_compare_done) begin
                                dup_rd_req_compare_cs <= 2'b01;
                            end
                        end
                        2'b01: begin
                            if (dup_rd_req_compare_cnt == 'd2) begin
                                if (rd_req_bram_rd_data[55:32] == (dup_rd_req_bram_rd_data[55:32])) begin
                                    start_with_middle       <= 2'b00;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else if (rd_req_bram_rd_data[175:152] == dup_rd_req_bram_rd_data[55:32]) begin
                                    start_with_middle       <= 2'b10;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else if((rd_req_bram_rd_data[55:32] < dup_rd_req_bram_rd_data[55:32] && rd_req_bram_rd_data[175:152] > dup_rd_req_bram_rd_data[55:32] && rd_req_bram_rd_data[55:32] < rd_req_bram_rd_data[175:152])  || 
                                   ((rd_req_bram_rd_data[55:32] < dup_rd_req_bram_rd_data[55:32] || rd_req_bram_rd_data[175:152] > dup_rd_req_bram_rd_data[55:32]) && rd_req_bram_rd_data[55:32] > rd_req_bram_rd_data[175:152]))begin
                                    start_with_middle       <= 2'b01;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else begin
                                    dup_rd_req_compare_cs <= 2'b10;
                                    start_with_middle     <= 2'b00;
                                end
                            end
                            dup_rd_req_compare_cnt       <= dup_rd_req_compare_cnt + {{RD_REQ_PTR_WIDTH{1'b0}}, 1'b1};
                            dup_rd_req_compare_match_ptr <= {RD_REQ_PTR_WIDTH{1'b0}};
                        end
                        2'b10: begin
                            dup_rd_req_compare_match_ptr <= dup_rd_req_compare_match_ptr + {{RD_REQ_PTR_WIDTH - 1{1'b0}}, 1'b1};
                            if (!dup_rd_req_compare_cnt[RD_REQ_PTR_WIDTH]) begin
                                if (rd_req_bram_rd_data[55:32] == (dup_rd_req_bram_rd_data[55:32])) begin
                                    start_with_middle       <= 2'b00;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else if (rd_req_bram_rd_data[175:152] == dup_rd_req_bram_rd_data[55:32]) begin
                                    start_with_middle       <= 2'b10;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else if((rd_req_bram_rd_data[55:32] < dup_rd_req_bram_rd_data[55:32] && rd_req_bram_rd_data[175:152] > dup_rd_req_bram_rd_data[55:32] && rd_req_bram_rd_data[55:32] < rd_req_bram_rd_data[175:152])  || 
                                   ((rd_req_bram_rd_data[55:32] < dup_rd_req_bram_rd_data[55:32] || rd_req_bram_rd_data[175:152] > dup_rd_req_bram_rd_data[55:32]) && rd_req_bram_rd_data[55:32] > rd_req_bram_rd_data[175:152]))begin

                                    start_with_middle       <= 2'b01;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else begin
                                    dup_rd_req_compare_cs  <= 2'b10;
                                    start_with_middle      <= 2'b00;
                                    dup_rd_req_compare_cnt <= dup_rd_req_compare_cnt + {{RD_REQ_PTR_WIDTH{1'b0}}, 1'b1};
                                end
                            end else begin
                                if (rd_req_bram_rd_data[55:32] == (dup_rd_req_bram_rd_data[55:32])) begin
                                    start_with_middle       <= 2'b00;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else if (rd_req_bram_rd_data[175:152] == dup_rd_req_bram_rd_data[55:32]) begin
                                    start_with_middle       <= 2'b10;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else if((rd_req_bram_rd_data[55:32] < dup_rd_req_bram_rd_data[55:32] && rd_req_bram_rd_data[175:152] > dup_rd_req_bram_rd_data[55:32] && rd_req_bram_rd_data[55:32] < rd_req_bram_rd_data[175:152])  || 
                                   ((rd_req_bram_rd_data[55:32] < dup_rd_req_bram_rd_data[55:32] || rd_req_bram_rd_data[175:152] > dup_rd_req_bram_rd_data[55:32]) && rd_req_bram_rd_data[55:32] > rd_req_bram_rd_data[175:152]))begin

                                    start_with_middle       <= 2'b01;
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    dup_rd_req_compare_done <= 1'b1;
                                end else begin
                                    dup_rd_req_compare_cs <= 2'b11;
                                    start_with_middle     <= 2'b00;
                                end
                            end
                        end
                        2'b11: begin
                            dup_rd_req_compare_match_ptr <= dup_rd_req_compare_match_ptr + {{RD_REQ_PTR_WIDTH - 1{1'b0}}, 1'b1};
                            if (~dup_rd_req_compare_wait && ~dup_rd_req_compare_wait_r) begin
                                dup_rd_req_compare_wait <= 1'b1;
                            end else begin
                                dup_rd_req_compare_wait <= 1'b0;
                            end
                            dup_rd_req_compare_cnt <= dup_rd_req_compare_cnt + {{RD_REQ_PTR_WIDTH{1'b0}}, 1'b1};
                            if (rd_req_bram_rd_data[55:32] == (dup_rd_req_bram_rd_data[55:32])) begin
                                start_with_middle       <= 2'b00;
                                dup_rd_req_compare_cs   <= 2'b00;
                                dup_rd_req_compare_done <= 1'b1;
                            end else if (rd_req_bram_rd_data[175:152] == dup_rd_req_bram_rd_data[55:32]) begin
                                start_with_middle       <= 2'b10;
                                dup_rd_req_compare_cs   <= 2'b00;
                                dup_rd_req_compare_done <= 1'b1;
                            end else if((rd_req_bram_rd_data[55:32] < dup_rd_req_bram_rd_data[55:32] && rd_req_bram_rd_data[175:152] > dup_rd_req_bram_rd_data[55:32] && rd_req_bram_rd_data[55:32] < rd_req_bram_rd_data[175:152])  || 
                                   ((rd_req_bram_rd_data[55:32] < dup_rd_req_bram_rd_data[55:32] || rd_req_bram_rd_data[175:152] > dup_rd_req_bram_rd_data[55:32]) && rd_req_bram_rd_data[55:32] > rd_req_bram_rd_data[175:152]))begin

                                start_with_middle       <= 2'b01;
                                dup_rd_req_compare_cs   <= 2'b00;
                                dup_rd_req_compare_done <= 1'b1;
                            end else begin
                                if (dup_rd_req_compare_wait_r == 1'b1) begin
                                    dup_rd_req_compare_cs   <= 2'b00;
                                    discard_rd_req          <= 1'b1;
                                    dup_rd_req_compare_done <= 1'b1;
                                end
                            end
                        end
                        default: begin
                            dup_rd_req_compare_cs   <= 2'b00;
                            start_with_middle       <= 1'b0;
                            discard_rd_req          <= 1'b0;
                            dup_rd_req_compare_done <= 1'b1;
                        end
                    endcase


                    if (rd_req_bram_rd_en_r || dup_rd_req_bram_rd_en_r) begin
                        rd_req_valid_r <= 1'b1;
                    end else if (i_hdr_gen_rdy) begin
                        rd_req_valid_r <= 1'b0;
                    end


                    if (rd_req_bram_rd_en_r && ~dup_rd_req_bram_rd_en_r) begin
                        if (rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH-1:0] == (C_MAX_RD_OS - 1)) begin
                            rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] <= 'd0;
                            rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH]     <= ~rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH];
                        end else begin
                            rd_ptr_buf_wr_data <= rd_ptr_buf_rd_data_sampled + 1'b1;
                        end
                        // For Sudden QP reset no internal process should update pointers (otherwise QP pointer reset valuse may be overwritten by internally driven values)
                        rd_ptr_buf_wren    <= qp_is_enabled;
                        rd_ptr_buf_wr_addr <= aeth_bram_rd_addr;
                    end else if (~rd_req_bram_rd_en_r && (dup_rd_req_bram_rd_en_r || discard_rd_req)) begin
                        if (dup_rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH-1:0] == (C_MAX_RD_OS - 1)) begin
                            dup_rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] <= 'd0;
                            dup_rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH]     <= ~dup_rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH];
                        end else begin
                            dup_rd_ptr_buf_wr_data <= dup_rd_ptr_buf_rd_data_sampled + 1'b1;
                        end
                        dup_rd_ptr_buf_wren    <= qp_is_enabled;
                        dup_rd_ptr_buf_wr_addr <= aeth_bram_rd_addr;
                        if (~discard_rd_req) begin
                            rd_ptr_buf_wr_data <= dup_rd_req_bram_rd_data[153+RD_REQ_PTR_WIDTH : 152];
                            rd_ptr_buf_wren    <= qp_is_enabled;
                            rd_ptr_buf_wr_addr <= aeth_bram_rd_addr;
                        end
                    end else begin
                        rd_ptr_buf_wren     <= 1'b0;
                        dup_rd_ptr_buf_wren <= 1'b0;
                    end
                end
            end




            always @(posedge core_clk) begin
                if (core_rst) begin
                    rd_req_addr_r <= 0;
                    rd_req_len_r  <= 0;
                end else begin
                    rd_req_addr_r <= o_rd_req_addr;
                    rd_req_len_r  <= o_rd_req_len;
                end
            end

            always @(*) begin
                if (rd_req_bram_rd_en_r1 && ~dup_rd_req_bram_rd_en_r1) begin
                    o_rd_req_addr = rd_req_bram_rd_data[151:88];
                    o_rd_req_len  = rd_req_bram_rd_data[87:56];
                end else if (~rd_req_bram_rd_en_r1 && dup_rd_req_bram_rd_en_r1) begin
                    o_rd_req_addr = dup_rd_req_bram_rd_data[151:88];
                    o_rd_req_len  = dup_rd_req_bram_rd_data[87:56];
                end else if (rd_req_bram_rd_en_r1 && dup_rd_req_bram_rd_en_r1) begin
                    if (dup_rd_req_bram_rd_data[55:32] < rd_req_bram_rd_data[55:32]) begin
                        o_rd_req_addr = dup_rd_req_bram_rd_data[151:88];
                        o_rd_req_len  = dup_rd_req_bram_rd_data[87:56];
                    end else if (dup_rd_req_bram_rd_data[55:32] == rd_req_bram_rd_data[55:32]) begin
                        o_rd_req_addr = rd_req_bram_rd_data[151:88];
                        o_rd_req_len  = rd_req_bram_rd_data[87:56];
                    end else begin
                        o_rd_req_addr = rd_req_bram_rd_data[151:88];
                        o_rd_req_len  = rd_req_bram_rd_data[87:56];
                    end
                end else begin
                    o_rd_req_addr = rd_req_addr_r;
                    o_rd_req_len  = rd_req_len_r;
                end

            end
            /////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////// Duplicate read request buffer pointers logic /////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////

            always @(posedge core_clk) begin
                if (core_rst) begin
                    dup_rd_req_fifo_empty_wr_en_0   <= 1'b0;
                    dup_rd_req_fifo_empty_wr_addr_0 <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_rd_req_fifo_empty_wr_en_1   <= 1'b0;
                    dup_rd_req_fifo_empty_wr_addr_1 <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_rd_req_fifo_full_wr_en_0    <= 1'b0;
                    dup_rd_req_fifo_full_wr_addr_0  <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_rd_req_fifo_full_wr_en_1    <= 1'b0;
                    dup_rd_req_fifo_full_wr_addr_1  <= {C_MAX_QID_WIDTH{1'b0}};
                end else begin

                    if (dup_wr_ptr_buf_wren) begin
                        dup_rd_req_fifo_empty_wr_en_0   <= 1'b1;
                        dup_rd_req_fifo_empty_wr_addr_0 <= dup_wr_ptr_buf_wr_addr;
                        dup_rd_req_fifo_empty_wr_data_0 <= 1'b0;
                    end else if (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) begin
                        dup_rd_req_fifo_empty_wr_en_0   <= 1'b1;
                        dup_rd_req_fifo_empty_wr_addr_0 <= wr_ptr_buf_wr_addr_init;
                        dup_rd_req_fifo_empty_wr_data_0 <= 1'b1;
                    end else begin
                        dup_rd_req_fifo_empty_wr_en_0 <= 1'b0;
                    end

                    if (dup_rd_ptr_buf_wren && dup_rd_ptr_buf_wr_data == dup_mirror_wr_ptr_buf_rd_data && ~((dup_wr_ptr_buf_wren || dup_wr_ptr_buf_wren_r) && dup_wr_ptr_buf_wr_addr == dup_rd_ptr_buf_wr_addr)) begin
                        dup_rd_req_fifo_empty_wr_en_1   <= 1'b1;
                        dup_rd_req_fifo_empty_wr_addr_1 <= dup_rd_ptr_buf_wr_addr;
                        dup_rd_req_fifo_empty_wr_data_1 <= 1'b1;
                    end else begin
                        dup_rd_req_fifo_empty_wr_en_1 <= 1'b0;
                    end



                    if (~ignore_dup_full_check && ~(dup_rd_ptr_buf_wren && (dup_wr_ptr_buf_wr_addr == dup_rd_ptr_buf_wr_addr)) && dup_wr_ptr_buf_wren && 
			(dup_wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] == dup_mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]) && 
			(dup_wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH] != dup_mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH])) begin
                        dup_rd_req_fifo_full_wr_en_1   <= 1'b1;
                        dup_rd_req_fifo_full_wr_addr_1 <= dup_wr_ptr_buf_wr_addr;
                        dup_rd_req_fifo_full_wr_data_1 <= 1'b0;
                    end else begin
                        dup_rd_req_fifo_full_wr_en_1 <= 1'b0;
                    end

                    if (dup_rd_ptr_buf_wren && ~(dup_wr_ptr_buf_wren && (dup_rd_ptr_buf_wr_addr == dup_wr_ptr_buf_wr_addr))) begin
                        dup_rd_req_fifo_full_wr_en_0   <= 1'b1;
                        dup_rd_req_fifo_full_wr_addr_0 <= dup_rd_ptr_buf_wr_addr;
                        dup_rd_req_fifo_full_wr_data_0 <= 1'b0;
                        // when QP is re-initialized, full should be reset for that QP
                    end else if (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) begin
                        dup_rd_req_fifo_full_wr_en_0   <= 1'b1;
                        dup_rd_req_fifo_full_wr_addr_0 <= wr_ptr_buf_wr_addr_init;
                        dup_rd_req_fifo_full_wr_data_0 <= 1'b0;
                    end else begin
                        dup_rd_req_fifo_full_wr_en_0 <= 1'b0;
                    end
                end
            end


            always @(posedge core_clk) begin
                if (core_rst) begin
                    dup_rd_req_bram_wr_en                 <= 1'b0;
                    dup_rd_req_bram_rd_en                 <= 1'b0;
                    dup_rd_req_bram_rd_en_r               <= 1'b0;
                    dup_rd_req_en                         <= 1'b0;
                    dup_rd_req_bram_wr_addr               <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                    dup_rd_req_bram_rd_addr               <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                    dup_rd_req_bram_wr_data               <= 'd0;
                    dup_rd_req_bram_wr_data_r1            <= 'd0;
                    start_with_middle_r                   <= 2'b00;
                    dup_rd_ptr_buf_rd_data_sampled        <= 'd0;
                    dup_mirror_wr_ptr_buf_rd_data_sampled <= 'd0;
                    last_dup_full_status                  <= 1'b0;
                end else begin
                    dup_rd_req_bram_wr_data_r1[151:0] <= {i_rd_req_phy_addr, i_rd_payload_len, i_res_psn, 1'b0, i_res_aeth_syndrome, i_res_msn};
                    last_dup_full_status              <= dup_rd_req_fifo_full_rd_data;
                    if (i_reg_xrnic_en && i_bres_valid_r && (i_req_type_r == 2'b10) && ~last_dup_full_status) begin
                        dup_rd_req_bram_wr_en <= 1'b1;
                        if ((i_bres_dest_qpid_r == dup_wr_ptr_buf_wr_addr) && i_bres_valid_r && dup_wr_ptr_buf_wren_r) begin
                            if (dup_wr_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0] == C_MAX_RD_OS - 1) begin
                                dup_rd_req_bram_wr_addr <= ((C_MAX_RD_OS * i_bres_dest_qpid_r));
                            end else begin
                                dup_rd_req_bram_wr_addr <= ((C_MAX_RD_OS * i_bres_dest_qpid_r) + dup_wr_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]) + {{RD_REQ_PTR_WIDTH - 1{1'b0}}, 1'b1};
                            end
                        end else begin
                            dup_rd_req_bram_wr_addr <= ((C_MAX_RD_OS * i_bres_dest_qpid_r) + dup_wr_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]);
                        end
                    end else begin
                        dup_rd_req_bram_wr_en <= 1'b0;
                    end
                    dup_rd_req_bram_wr_data[153+RD_REQ_PTR_WIDTH : 152] <= wr_ptr_buf_rd_data;
                    dup_rd_req_bram_wr_data[154+RD_REQ_PTR_WIDTH]       <= qp_ack_pending_rd_data_0;
                    dup_rd_req_bram_wr_data[151:0]                      <= dup_rd_req_bram_wr_data_r1[151:0];
                    dup_rd_req_bram_rd_en_r                             <= dup_rd_req_compare_done && ~discard_rd_req;
                    start_with_middle_r                                 <= start_with_middle;
                    if (dup_rd_req_bram_rd_en) begin
                        dup_rd_req_en <= 1'b1;
                    end else if (dup_rd_req_compare_done) begin
                        dup_rd_req_en <= 1'b0;
                    end

                    if (~arb_in_progress && ~aeth_valid_r && ~rd_req_bram_rd_en_r && ~rd_req_bram_rd_en && ~dup_rd_req_en && ~dup_rd_req_bram_rd_en_r && ~dup_rd_req_bram_rd_en && ~rd_req_valid_r && ~discard_rd_req_r && ~dup_rd_req_fifo_empty_rd_data) begin
                        dup_rd_req_bram_rd_en                 <= 1'b1;
                        dup_rd_req_bram_rd_addr               <= ((C_MAX_RD_OS * aeth_bram_rd_addr) + dup_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]);
                        dup_rd_ptr_buf_rd_data_sampled        <= dup_rd_ptr_buf_rd_data;
                        dup_mirror_wr_ptr_buf_rd_data_sampled <= dup_mirror_wr_ptr_buf_rd_data;
                    end else begin
                        dup_rd_req_bram_rd_en <= 1'b0;
                    end
                end
            end
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////// Read request buffers for QP's ////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            xpm_memory_sdpram #(

                // Common module parameters
                .MEMORY_SIZE      (176 * C_MAX_QP * C_MAX_RD_OS),  //positive integer
                .MEMORY_PRIMITIVE ("ultra"),                       //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),                //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),                        //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                            //string;
                .USE_MEM_INIT     (1),                             //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),               //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                             //integer; 0,1
                .ECC_MODE         ("no_ecc"),                      //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                             //Do not Change

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(176),                                //positive integer
                .BYTE_WRITE_WIDTH_A(8),                                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH), //positive integer

                // Port B module parameters
                .READ_DATA_WIDTH_B (176),                                 //positive integer
                .ADDR_WIDTH_B      (C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),                                 //string
                .READ_LATENCY_B    (2),                                   //non-negative integer
                .WRITE_MODE_B      ("read_first")                         //string; "write_first", "read_first", "no_change" 

            ) rd_req_bram (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (rd_req_bram_wr_en),
                .wea           ({22{1'b1}}),
                .addra         (rd_req_bram_wr_addr),
                .dina          (rd_req_bram_wr_data),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (rd_req_bram_rd_en),
                .regceb  (1'b1),
                .addrb   (rd_req_bram_rd_addr),
                .doutb   (rd_req_bram_rd_data),
                .sbiterrb(),
                .dbiterrb()

            );

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////// Duplicate read request buffers for QP's //////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            xpm_memory_sdpram #(

                // Common module parameters
                .MEMORY_SIZE      (160 * C_MAX_QP * C_MAX_RD_OS),  //positive integer
                .MEMORY_PRIMITIVE ("ultra"),                       //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),                //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),                        //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                            //string;
                .USE_MEM_INIT     (1),                             //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),               //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                             //integer; 0,1
                .ECC_MODE         ("no_ecc"),                      //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                             //Do not Change

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(160),                                //positive integer
                .BYTE_WRITE_WIDTH_A(8),                                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH), //positive integer

                // Port B module parameters
                .READ_DATA_WIDTH_B (160),                                 //positive integer
                .ADDR_WIDTH_B      (C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),                                 //string
                .READ_LATENCY_B    (2),                                   //non-negative integer
                .WRITE_MODE_B      ("read_first")                         //string; "write_first", "read_first", "no_change" 

            ) dup_rd_req_bram (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (dup_rd_req_bram_wr_en),
                .wea           ({20{1'b1}}),
                .addra         (dup_rd_req_bram_wr_addr),
                .dina          (dup_rd_req_bram_wr_data),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (dup_rd_req_bram_rd_en),
                .regceb  (1'b1),
                .addrb   (dup_rd_req_bram_rd_addr),
                .doutb   (dup_rd_req_bram_rd_data),
                .sbiterrb(),
                .dbiterrb()

            );
        end
    endgenerate

    generate
        if (C_EN_RECV_RD_WR == 1 && C_EN_RD_RESP_MIDDLE == 0) begin : EN_RD_WR_FIRST

            reg  [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_wr_addr;
            reg  [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_wr_addr_init;
            wire [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_wr_addr_mux;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_wr_ptr_buf_wr_addr_mux;
            wire [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_rd_addr;
            wire [ RD_REQ_PTR_WIDTH : 0] wr_ptr_buf_wr_data;
            wire [ RD_REQ_PTR_WIDTH : 0] wr_ptr_buf_rd_data;
            reg  [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_wr_addr;
            wire [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_addr_B;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_rd_ptr_buf_addr_B;
            wire [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_rd_addr;
            reg  [ RD_REQ_PTR_WIDTH : 0] rd_ptr_buf_wr_data;
            wire [ RD_REQ_PTR_WIDTH : 0] rd_ptr_buf_rd_data;
            wire [ RD_REQ_PTR_WIDTH : 0] mirror_wr_ptr_buf_rd_data;
            wire [ RD_REQ_PTR_WIDTH : 0] mirror_rd_ptr_buf_rd_data;
            reg                          wr_ptr_buf_wren;
            reg                          wr_ptr_buf_wren_r;
            wire                         wr_ptr_buf_rden;
            reg                          rd_ptr_buf_wren;
            reg                          rd_ptr_buf_wren_init;
            reg  [ RD_REQ_PTR_WIDTH : 0] rd_ptr_buf_rd_data_sampled;
            //reg  [RD_REQ_PTR_WIDTH : 0] mirror_wr_ptr_buf_rd_data_sampled;
            reg  [C_MAX_QID_WIDTH-1 : 0] rd_init_qpid;

            reg  [C_MAX_QID_WIDTH-1 : 0] dup_wr_ptr_buf_wr_addr;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_wr_ptr_buf_rd_addr;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_wr_ptr_buf_wr_data;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_wr_ptr_buf_rd_data;
            reg  [C_MAX_QID_WIDTH-1 : 0] dup_rd_ptr_buf_wr_addr;
            reg  [ RD_REQ_PTR_WIDTH : 0] dup_rd_ptr_buf_wr_data;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_rd_ptr_buf_rd_data;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_mirror_wr_ptr_buf_rd_data;
            wire [ RD_REQ_PTR_WIDTH : 0] dup_mirror_rd_ptr_buf_rd_data;
            reg                          dup_wr_ptr_buf_wren;
            reg                          dup_wr_ptr_buf_wren_r;
            wire                         dup_wr_ptr_buf_rden;
            reg                          dup_rd_ptr_buf_wren;
            wire                         dup_rd_ptr_buf_rden;
            reg  [ RD_REQ_PTR_WIDTH : 0] dup_rd_ptr_buf_rd_data_sampled;
            reg  [ RD_REQ_PTR_WIDTH : 0] dup_mirror_wr_ptr_buf_rd_data_sampled;
            reg                          ignore_full_check;
            reg                          ignore_dup_full_check;
            wire [C_MAX_QID_WIDTH-1 : 0] wr_ptr_buf_addr_A;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_wr_ptr_buf_addr_A;
            wire [C_MAX_QID_WIDTH-1 : 0] rd_ptr_buf_addr_A;
            wire [C_MAX_QID_WIDTH-1 : 0] dup_rd_ptr_buf_addr_A;
            reg                          last_fatal_status;
            reg                          last_full_status;
            reg                          last_dup_full_status;

            assign wr_ptr_buf_addr_A     = wr_ptr_buf_rden ? wr_ptr_buf_rd_addr : wr_ptr_buf_wr_addr_mux;
            assign dup_wr_ptr_buf_addr_A = dup_wr_ptr_buf_rden ? dup_wr_ptr_buf_rd_addr : dup_wr_ptr_buf_wr_addr_mux;
            assign rd_ptr_buf_addr_A     = arb_in_progress ? rd_ptr_buf_rd_addr : rd_ptr_buf_wr_addr;
            assign dup_rd_ptr_buf_addr_A = arb_in_progress ? rd_ptr_buf_rd_addr : dup_rd_ptr_buf_wr_addr;
            assign rd_ptr_buf_addr_B     = i_bres_valid ? i_bres_dest_qpid : wr_ptr_buf_wr_addr_init;
            assign dup_rd_ptr_buf_addr_B = i_bres_valid ? i_bres_dest_qpid : wr_ptr_buf_wr_addr_init;


            genvar j;
            always @(posedge core_clk) begin
                start_with_middle_r <= 2'b00;
                dup_rd_req_en       <= 1'b0;
                discard_rd_req      <= 1'b0;
                discard_rd_req_r    <= 1'b0;
            end

            assign wr_ptr_buf_rd_addr     = i_bres_dest_qpid;
            assign wr_ptr_buf_rden        = i_bres_valid;
            assign dup_wr_ptr_buf_rd_addr = i_bres_dest_qpid;
            assign dup_wr_ptr_buf_rden    = i_bres_valid;
            assign rd_ptr_buf_rd_addr     = arbitrated_Qpid;


            always @(posedge core_clk) begin
                if (core_rst) begin
                    rd_req_fifo_empty_wr_en_0   <= 1'b0;
                    rd_req_fifo_empty_wr_addr_0 <= {C_MAX_QID_WIDTH{1'b0}};
                    rd_req_fifo_empty_wr_en_1   <= 1'b0;
                    rd_req_fifo_empty_wr_addr_1 <= {C_MAX_QID_WIDTH{1'b0}};
                    rd_req_fifo_full_wr_en_0    <= 1'b0;
                    rd_req_fifo_full_wr_addr_0  <= {C_MAX_QID_WIDTH{1'b0}};
                    rd_req_fifo_full_wr_en_1    <= 1'b0;
                    rd_req_fifo_full_wr_addr_1  <= {C_MAX_QID_WIDTH{1'b0}};
                end else begin
                    if (wr_ptr_buf_wren) begin
                        rd_req_fifo_empty_wr_en_0   <= 1'b1;
                        rd_req_fifo_empty_wr_addr_0 <= wr_ptr_buf_wr_addr;
                        rd_req_fifo_empty_wr_data_0 <= 1'b0;
                    end else if (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) begin
                        rd_req_fifo_empty_wr_en_0   <= 1'b1;
                        rd_req_fifo_empty_wr_addr_0 <= wr_ptr_buf_wr_addr_init;
                        rd_req_fifo_empty_wr_data_0 <= 1'b1;
                    end else begin
                        rd_req_fifo_empty_wr_en_0 <= 1'b0;
                    end

                    if (rd_ptr_buf_wren && rd_ptr_buf_wr_data == mirror_wr_ptr_buf_rd_data && ~((wr_ptr_buf_wren || wr_ptr_buf_wren_r) && wr_ptr_buf_wr_addr == rd_ptr_buf_wr_addr)) begin
                        rd_req_fifo_empty_wr_en_1   <= 1'b1;
                        rd_req_fifo_empty_wr_addr_1 <= rd_ptr_buf_wr_addr;
                        rd_req_fifo_empty_wr_data_1 <= 1'b1;
                    end else begin
                        rd_req_fifo_empty_wr_en_1 <= 1'b0;
                    end


                    if (~ignore_full_check && ~(rd_ptr_buf_wren && (wr_ptr_buf_wr_addr == rd_ptr_buf_wr_addr)) && wr_ptr_buf_wren && 
			(wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] == mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]) && (wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH] != mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH])) begin
                        rd_req_fifo_full_wr_en_1   <= 1'b1;
                        rd_req_fifo_full_wr_addr_1 <= wr_ptr_buf_wr_addr;
                    end else begin
                        rd_req_fifo_full_wr_en_1 <= 1'b0;
                    end

                    if (rd_ptr_buf_wren && ~(wr_ptr_buf_wren && (rd_ptr_buf_wr_addr == wr_ptr_buf_wr_addr))) begin
                        rd_req_fifo_full_wr_en_0   <= 1'b1;
                        rd_req_fifo_full_wr_addr_0 <= rd_ptr_buf_wr_addr;
                        // when QP is re-initialized, full should be reset for that QP
                    end else if (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) begin
                        rd_req_fifo_full_wr_en_0   <= 1'b1;
                        rd_req_fifo_full_wr_addr_0 <= wr_ptr_buf_wr_addr_init;
                    end else begin
                        rd_req_fifo_full_wr_en_0 <= 1'b0;
                    end
                end
            end

            //CR-1112580 fix requires overflow info on the just next clock
            assign rd_req_fifo_full_set_valid   = (!ignore_full_check && !(rd_ptr_buf_wren && (wr_ptr_buf_wr_addr == rd_ptr_buf_wr_addr)) && wr_ptr_buf_wren && 
			                          (wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] == mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]) && 
									  (wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH] != mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH])) ? 1'b1 : 1'b0;
            assign rd_req_fifo_full_set_id = wr_ptr_buf_wr_addr;
            assign rd_req_fifo_full_reset_valid = ((rd_ptr_buf_wren && !(wr_ptr_buf_wren && (rd_ptr_buf_wr_addr == wr_ptr_buf_wr_addr))) || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)) ? 1'b1 : 1'b0;
            assign rd_req_fifo_full_reset_id = (rd_ptr_buf_wren && !(wr_ptr_buf_wren && (rd_ptr_buf_wr_addr == wr_ptr_buf_wr_addr))) ? rd_ptr_buf_wr_addr : wr_ptr_buf_wr_addr_init;


            // Moving en,addr and data of inst_wr_ptr_buf and inst_dup_wr_ptr_buf to combinational processor
            // To reduce a clk delay between rd_en and wr_en for these brams
            assign wr_ptr_buf_wr_data = (wr_ptr_buf_wren) ? (wr_ptr_buf_rd_data + {{RD_REQ_PTR_WIDTH{1'b0}}, 1'b1}) : ({RD_REQ_PTR_WIDTH + 1{1'b0}});
            assign dup_wr_ptr_buf_wr_data = (dup_wr_ptr_buf_wren) ? (dup_wr_ptr_buf_rd_data + {{RD_REQ_PTR_WIDTH{1'b0}}, 1'b1}) : ({RD_REQ_PTR_WIDTH + 1{1'b0}});
            assign wr_ptr_buf_wr_addr_mux = (wr_ptr_buf_wren) ? wr_ptr_buf_wr_addr : wr_ptr_buf_wr_addr_init;
            assign dup_wr_ptr_buf_wr_addr_mux = (dup_wr_ptr_buf_wren) ? dup_wr_ptr_buf_wr_addr : wr_ptr_buf_wr_addr_init;
            assign ptr_init_done = (wr_ptr_buf_wren == 1'b0 && wr_ptr_buf_wren_init == 1'b1 && rd_ptr_buf_wren == 1'b0 && dup_rd_ptr_buf_wren == 1'b0 && dup_wr_ptr_buf_wren == 1'b0) ? 1'b1 : 1'b0;

            always @(posedge core_clk) begin
                if (core_rst) begin
                    wr_ptr_buf_wr_addr      <= {C_MAX_QID_WIDTH{1'b0}};
                    wr_ptr_buf_wr_addr_init <= {C_MAX_QID_WIDTH{1'b0}};
                    wr_ptr_buf_wren         <= 1'b0;
                    wr_ptr_buf_wren_init    <= 1'b0;
                    wr_ptr_buf_wren_r       <= 1'b0;
                    dup_wr_ptr_buf_wren_r   <= 1'b0;
                    dup_wr_ptr_buf_wr_addr  <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_wr_ptr_buf_wren     <= 1'b0;
                    ignore_full_check       <= 1'b0;
                    ignore_dup_full_check   <= 1'b0;
                end else begin
                    wr_ptr_buf_wren_r     <= wr_ptr_buf_wren;
                    dup_wr_ptr_buf_wren_r <= dup_wr_ptr_buf_wren;
                    if (rd_ptr_buf_wren && (i_bres_dest_qpid == rd_ptr_buf_addr_A) && i_bres_valid && i_reg_xrnic_en && (i_req_type == 2'b01) && ~(qp_fatal_rd_data_2 || qp_under_recovery_rd_data) && ~rd_req_fifo_full_rd_data) begin
                        ignore_full_check <= 1'b1;
                    end else begin
                        ignore_full_check <= 1'b0;
                    end

                    if (i_bres_valid && i_reg_xrnic_en && (i_req_type == 2'b01) && ~(qp_fatal_rd_data_2 || qp_under_recovery_rd_data) && ~rd_req_fifo_full_rd_data) begin
                        wr_ptr_buf_wr_addr <= i_bres_dest_qpid;
                        wr_ptr_buf_wren    <= 1'b1;
                    end else begin
                        wr_ptr_buf_wren <= 1'b0;
                        if (!wr_ptr_buf_wren_init && rst_fifo_rd_en_r && !ptr_init_done_flag && !rd_req_bram_rd_en_r && !dup_rd_req_bram_rd_en_r) begin
                            wr_ptr_buf_wr_addr_init <= rst_qpid;
                            wr_ptr_buf_wren_init    <= 1'b1;
                            // Init to both write and read ptr brams should be simultaneously; write will happen when all bram ports are not busy and write is possible; else wait.. 
                        end else if (wr_ptr_buf_wren == 1'b0 && wr_ptr_buf_wren_init == 1'b1 && rd_ptr_buf_wren == 1'b0 && dup_rd_ptr_buf_wren == 1'b0 && dup_wr_ptr_buf_wren == 1'b0) begin
                            wr_ptr_buf_wren_init <= 1'b0;
                        end
                    end


                    if (dup_rd_ptr_buf_wren && (i_bres_dest_qpid == dup_rd_ptr_buf_addr_A) && i_reg_xrnic_en && i_bres_valid && (i_req_type == 2'b10) && ~dup_rd_req_fifo_full_rd_data) begin
                        ignore_dup_full_check <= 1'b1;
                    end else begin
                        ignore_dup_full_check <= 1'b0;
                    end

                    if (i_reg_xrnic_en && i_bres_valid && (i_req_type == 2'b10) && ~dup_rd_req_fifo_full_rd_data) begin
                        dup_wr_ptr_buf_wren    <= 1'b1;
                        dup_wr_ptr_buf_wr_addr <= i_bres_dest_qpid;
                    end else begin
                        dup_wr_ptr_buf_wren <= 1'b0;
                    end
                end
            end
            /***************************************************************************
             *                                                                         *
             * 									                                       *
             * 			TDP memory -- for pointer storage		                       *
             * 									                                       *
             * 									                                       *
             ***************************************************************************/
            xpm_memory_tdpram #(
                .ADDR_WIDTH_A           (C_MAX_QID_WIDTH),
                .ADDR_WIDTH_B           (C_MAX_QID_WIDTH),
                .AUTO_SLEEP_TIME        (0),
                .BYTE_WRITE_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .BYTE_WRITE_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .CASCADE_HEIGHT         (0),
                .CLOCKING_MODE          ("common_clock"),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_MAX_QP * (RD_REQ_PTR_WIDTH + 1)),
                .MESSAGE_CONTROL        (0),
                .READ_DATA_WIDTH_A      (RD_REQ_PTR_WIDTH + 1),
                .READ_DATA_WIDTH_B      (RD_REQ_PTR_WIDTH + 1),
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
                .WRITE_DATA_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_DATA_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_MODE_A           ("read_first"),
                .WRITE_MODE_B           ("read_first")
            ) inst_tdp_wr_ptr_buf (
                .clka          (core_clk),
                .rsta          (core_rst),
                .regcea        (1'b1),
                .ena           (wr_ptr_buf_wren || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) || wr_ptr_buf_rden),
                .wea           ((wr_ptr_buf_wren || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)) && (~wr_ptr_buf_rden)),
                .addra         (wr_ptr_buf_addr_A),
                .dina          (wr_ptr_buf_wr_data),
                .douta         (wr_ptr_buf_rd_data),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .regceb        (1'b1),
                .enb           (~arb_in_progress),
                .web           (1'b0),
                .addrb         (aeth_bram_rd_addr),
                .dinb          (wr_ptr_buf_wr_data),
                .doutb         (mirror_wr_ptr_buf_rd_data),
                .injectdbiterra(1'b0),
                .injectdbiterrb(1'b0),
                .injectsbiterra(1'b0),
                .injectsbiterrb(1'b0),
                .sleep         (1'b0),
                .sbiterra      (),
                .sbiterrb      (),
                .dbiterra      (),
                .dbiterrb      ()
            );


            xpm_memory_tdpram #(
                .ADDR_WIDTH_A           (C_MAX_QID_WIDTH),
                .ADDR_WIDTH_B           (C_MAX_QID_WIDTH),
                .AUTO_SLEEP_TIME        (0),
                .BYTE_WRITE_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .BYTE_WRITE_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .CASCADE_HEIGHT         (0),
                .CLOCKING_MODE          ("common_clock"),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_MAX_QP * (RD_REQ_PTR_WIDTH + 1)),
                .MESSAGE_CONTROL        (0),
                .READ_DATA_WIDTH_A      (RD_REQ_PTR_WIDTH + 1),
                .READ_DATA_WIDTH_B      (RD_REQ_PTR_WIDTH + 1),
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
                .WRITE_DATA_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_DATA_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_MODE_A           ("read_first"),
                .WRITE_MODE_B           ("read_first")
            ) inst_tdp_rd_ptr_buf (
                .clka          (core_clk),
                .rsta          (core_rst),
                .regcea        (1'b1),
                .ena           (rd_ptr_buf_wren || arb_in_progress),
                .wea           (rd_ptr_buf_wren && (~arb_in_progress)),
                .addra         (rd_ptr_buf_addr_A),
                .dina          (rd_ptr_buf_wr_data),
                .douta         (rd_ptr_buf_rd_data),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .regceb        (1'b1),
                .enb           (i_bres_valid || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)),
                .web           ((~i_bres_valid) && (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)),
                .addrb         (rd_ptr_buf_addr_B),
                .dinb          ({RD_REQ_PTR_WIDTH + 1{1'b0}}),
                .doutb         (mirror_rd_ptr_buf_rd_data),
                .injectdbiterra(1'b0),
                .injectdbiterrb(1'b0),
                .injectsbiterra(1'b0),
                .injectsbiterrb(1'b0),
                .sleep         (1'b0),
                .sbiterra      (),
                .sbiterrb      (),
                .dbiterra      (),
                .dbiterrb      ()
            );


            xpm_memory_tdpram #(
                .ADDR_WIDTH_A           (C_MAX_QID_WIDTH),
                .ADDR_WIDTH_B           (C_MAX_QID_WIDTH),
                .AUTO_SLEEP_TIME        (0),
                .BYTE_WRITE_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .BYTE_WRITE_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .CASCADE_HEIGHT         (0),
                .CLOCKING_MODE          ("common_clock"),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_MAX_QP * (RD_REQ_PTR_WIDTH + 1)),
                .MESSAGE_CONTROL        (0),
                .READ_DATA_WIDTH_A      (RD_REQ_PTR_WIDTH + 1),
                .READ_DATA_WIDTH_B      (RD_REQ_PTR_WIDTH + 1),
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
                .WRITE_DATA_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_DATA_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_MODE_A           ("read_first"),
                .WRITE_MODE_B           ("read_first")
            ) inst_dup_tdp_wr_ptr_buf (
                .clka          (core_clk),
                .rsta          (core_rst),
                .regcea        (1'b1),
                .ena           (dup_wr_ptr_buf_wren || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) || dup_wr_ptr_buf_rden),
                .wea           ((dup_wr_ptr_buf_wren || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)) && (~dup_wr_ptr_buf_rden)),
                .addra         (dup_wr_ptr_buf_addr_A),
                .dina          (dup_wr_ptr_buf_wr_data),
                .douta         (dup_wr_ptr_buf_rd_data),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .regceb        (1'b1),
                .enb           (~arb_in_progress),
                .web           (1'b0),
                .addrb         (aeth_bram_rd_addr),
                .dinb          (dup_wr_ptr_buf_wr_data),
                .doutb         (dup_mirror_wr_ptr_buf_rd_data),
                .injectdbiterra(1'b0),
                .injectdbiterrb(1'b0),
                .injectsbiterra(1'b0),
                .injectsbiterrb(1'b0),
                .sleep         (1'b0),
                .sbiterra      (),
                .sbiterrb      (),
                .dbiterra      (),
                .dbiterrb      ()
            );


            xpm_memory_tdpram #(
                .ADDR_WIDTH_A           (C_MAX_QID_WIDTH),
                .ADDR_WIDTH_B           (C_MAX_QID_WIDTH),
                .AUTO_SLEEP_TIME        (0),
                .BYTE_WRITE_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .BYTE_WRITE_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .CASCADE_HEIGHT         (0),
                .CLOCKING_MODE          ("common_clock"),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_MAX_QP * (RD_REQ_PTR_WIDTH + 1)),
                .MESSAGE_CONTROL        (0),
                .READ_DATA_WIDTH_A      (RD_REQ_PTR_WIDTH + 1),
                .READ_DATA_WIDTH_B      (RD_REQ_PTR_WIDTH + 1),
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
                .WRITE_DATA_WIDTH_A     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_DATA_WIDTH_B     (RD_REQ_PTR_WIDTH + 1),
                .WRITE_MODE_A           ("read_first"),
                .WRITE_MODE_B           ("read_first")
            ) inst_dup_tdp_rd_ptr_buf (
                .clka          (core_clk),
                .rsta          (core_rst),
                .regcea        (1'b1),
                .ena           (dup_rd_ptr_buf_wren || arb_in_progress),
                .wea           (dup_rd_ptr_buf_wren && (~arb_in_progress)),
                .addra         (dup_rd_ptr_buf_addr_A),
                .dina          (dup_rd_ptr_buf_wr_data),
                .douta         (dup_rd_ptr_buf_rd_data),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .regceb        (1'b1),
                .enb           (i_bres_valid || (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)),
                .web           ((~i_bres_valid) && (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren)),
                .addrb         (dup_rd_ptr_buf_addr_B),
                .dinb          ({RD_REQ_PTR_WIDTH + 1{1'b0}}),
                .doutb         (dup_mirror_rd_ptr_buf_rd_data),
                .injectdbiterra(1'b0),
                .injectdbiterrb(1'b0),
                .injectsbiterra(1'b0),
                .injectsbiterrb(1'b0),
                .sleep         (1'b0),
                .sbiterra      (),
                .sbiterrb      (),
                .dbiterra      (),
                .dbiterrb      ()
            );


            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////// END read response data write logic using BRAM for Write pointer /////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            always @(posedge core_clk) begin
                if (core_rst) begin
                    rd_req_bram_wr_en          <= 1'b0;
                    rd_req_bram_rd_en          <= 1'b0;
                    rd_req_bram_rd_en_r        <= 1'b0;
                    rd_req_bram_wr_addr        <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                    rd_req_bram_rd_addr        <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                    rd_req_bram_wr_data        <= 'd0;
                    rd_req_bram_wr_data_r1     <= 'd0;
                    //o_rd_req_addr <= 64'd00;
                    //o_rd_req_len <= 32'd0;
                    rd_req_valid_r             <= 1'b0;
                    rd_ptr_buf_rd_data_sampled <= 'd0;
                    //mirror_wr_ptr_buf_rd_data_sampled <= 'd0;
                    rd_ptr_buf_wren            <= 1'b0;
                    rd_ptr_buf_wren_init       <= 1'b0;
                    rd_init_qpid               <= {C_MAX_QID_WIDTH{1'b0}};
                    last_fatal_status          <= 1'b0;
                    last_full_status           <= 1'b0;
                end else begin
                    rd_req_bram_wr_data_r1 <= {i_rd_req_phy_addr, i_rd_payload_len, i_res_psn, 1'b0, i_res_aeth_syndrome, i_res_msn};
                    rd_req_bram_wr_data    <= rd_req_bram_wr_data_r1;
                    last_fatal_status      <= qp_fatal_rd_data_2 || qp_under_recovery_rd_data;
                    last_full_status       <= rd_req_fifo_full_rd_data;

                    if (i_bres_valid_r && i_reg_xrnic_en && (i_req_type_r == 2'b01) && ~last_fatal_status && ~last_full_status) begin
                        rd_req_bram_wr_en   <= 1'b1;
                        rd_req_bram_wr_addr <= ((C_MAX_RD_OS * i_bres_dest_qpid_r) + wr_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]);
                    end else begin
                        rd_req_bram_wr_en <= 1'b0;
                    end
                    rd_req_bram_rd_en_r <= rd_req_bram_rd_en;
                    if (~arb_in_progress && ~aeth_valid_r && ~dup_rd_req_bram_rd_en_r && ~dup_rd_req_bram_rd_en && ~rd_req_bram_rd_en_r && ~rd_req_bram_rd_en && ~rd_req_valid_r && ~rd_req_fifo_empty_rd_data && ~aeth_bram_rd_en && ~aeth_bram_rd_en_r) begin
                        rd_req_bram_rd_en          <= 1'b1;
                        rd_req_bram_rd_addr        <= (aeth_bram_rd_addr * C_MAX_RD_OS) + rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0];
                        rd_ptr_buf_rd_data_sampled <= rd_ptr_buf_rd_data;
                    end else begin
                        rd_req_bram_rd_en <= 1'b0;
                    end

                    if (rd_req_bram_rd_en_r || dup_rd_req_bram_rd_en_r) begin
                        rd_req_valid_r <= 1'b1;
                    end else if (i_hdr_gen_rdy) begin
                        rd_req_valid_r <= 1'b0;
                    end


                    if (rd_req_bram_rd_en_r && ~dup_rd_req_bram_rd_en_r) begin
                        if (rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH-1:0] == (C_MAX_RD_OS - 1)) begin
                            rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] <= 'd0;
                            rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH]     <= ~rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH];
                        end else begin
                            rd_ptr_buf_wr_data <= rd_ptr_buf_rd_data_sampled + 1'b1;
                        end
                        rd_ptr_buf_wren    <= qp_is_enabled;
                        rd_ptr_buf_wr_addr <= aeth_bram_rd_addr;
                    end else if (~rd_req_bram_rd_en_r && dup_rd_req_bram_rd_en_r) begin
                        if (dup_rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH-1:0] == (C_MAX_RD_OS - 1)) begin
                            dup_rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] <= 'd0;
                            dup_rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH]     <= ~dup_rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH];
                        end else begin
                            dup_rd_ptr_buf_wr_data <= dup_rd_ptr_buf_rd_data_sampled + 1'b1;
                        end
                        dup_rd_ptr_buf_wren    <= qp_is_enabled;
                        dup_rd_ptr_buf_wr_addr <= aeth_bram_rd_addr;
                    end else if (rd_req_bram_rd_en_r && dup_rd_req_bram_rd_en_r) begin
                        if (dup_rd_req_bram_rd_data[55:32] < rd_req_bram_rd_data[55:32]) begin
                            if (dup_rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH-1:0] == (C_MAX_RD_OS - 1)) begin
                                dup_rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] <= 'd0;
                                dup_rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH]     <= ~dup_rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH];
                            end else begin
                                dup_rd_ptr_buf_wr_data <= dup_rd_ptr_buf_rd_data_sampled + 1'b1;
                            end
                            dup_rd_ptr_buf_wren    <= qp_is_enabled;
                            dup_rd_ptr_buf_wr_addr <= aeth_bram_rd_addr;
                        end else if (dup_rd_req_bram_rd_data[55:32] == rd_req_bram_rd_data[55:32]) begin
                            if (dup_rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH-1:0] == (C_MAX_RD_OS - 1)) begin
                                dup_rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] <= 'd0;
                                dup_rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH]     <= ~dup_rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH];
                            end else begin
                                dup_rd_ptr_buf_wr_data <= dup_rd_ptr_buf_rd_data_sampled + 1'b1;
                            end
                            if (rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH-1:0] == (C_MAX_RD_OS - 1)) begin
                                rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] <= 'd0;
                                rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH]     <= ~rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH];
                            end else begin
                                rd_ptr_buf_wr_data <= rd_ptr_buf_rd_data_sampled + 1'b1;
                            end
                            dup_rd_ptr_buf_wren    <= qp_is_enabled;
                            dup_rd_ptr_buf_wr_addr <= aeth_bram_rd_addr;
                            rd_ptr_buf_wren        <= qp_is_enabled;
                            rd_ptr_buf_wr_addr     <= aeth_bram_rd_addr;
                        end else begin
                            if (rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH-1:0] == (C_MAX_RD_OS - 1)) begin
                                rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] <= 'd0;
                                rd_ptr_buf_wr_data[RD_REQ_PTR_WIDTH]     <= ~rd_ptr_buf_rd_data_sampled[RD_REQ_PTR_WIDTH];
                            end else begin
                                rd_ptr_buf_wr_data <= rd_ptr_buf_rd_data_sampled + 1'b1;
                            end
                            rd_ptr_buf_wren    <= qp_is_enabled;
                            rd_ptr_buf_wr_addr <= aeth_bram_rd_addr;
                        end
                    end else begin
                        rd_ptr_buf_wren     <= 1'b0;
                        dup_rd_ptr_buf_wren <= 1'b0;
                    end
                end
            end



            always @(posedge core_clk) begin
                if (core_rst) begin
                    rd_req_addr_r <= 0;
                    rd_req_len_r  <= 0;
                end else begin
                    rd_req_addr_r <= o_rd_req_addr;
                    rd_req_len_r  <= o_rd_req_len;
                end
            end

            always @(*) begin
                if (rd_req_bram_rd_en_r1 && ~dup_rd_req_bram_rd_en_r1) begin
                    o_rd_req_addr = rd_req_bram_rd_data[151:88];
                    o_rd_req_len  = rd_req_bram_rd_data[87:56];
                end else if (~rd_req_bram_rd_en_r1 && dup_rd_req_bram_rd_en_r1) begin
                    o_rd_req_addr = dup_rd_req_bram_rd_data[151:88];
                    o_rd_req_len  = dup_rd_req_bram_rd_data[87:56];
                end else if (rd_req_bram_rd_en_r1 && dup_rd_req_bram_rd_en_r1) begin
                    if (dup_rd_req_bram_rd_data[55:32] < rd_req_bram_rd_data[55:32]) begin
                        o_rd_req_addr = dup_rd_req_bram_rd_data[151:88];
                        o_rd_req_len  = dup_rd_req_bram_rd_data[87:56];
                    end else if (dup_rd_req_bram_rd_data[55:32] == rd_req_bram_rd_data[55:32]) begin
                        o_rd_req_addr = rd_req_bram_rd_data[151:88];
                        o_rd_req_len  = rd_req_bram_rd_data[87:56];
                    end else begin
                        o_rd_req_addr = rd_req_bram_rd_data[151:88];
                        o_rd_req_len  = rd_req_bram_rd_data[87:56];
                    end
                end else begin
                    o_rd_req_addr = rd_req_addr_r;
                    o_rd_req_len  = rd_req_len_r;
                end

            end
            /////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////// Duplicate read request buffer pointers logic /////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////

            always @(posedge core_clk) begin
                if (core_rst) begin
                    dup_rd_req_fifo_empty_wr_en_0   <= 1'b0;
                    dup_rd_req_fifo_empty_wr_addr_0 <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_rd_req_fifo_empty_wr_en_1   <= 1'b0;
                    dup_rd_req_fifo_empty_wr_addr_1 <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_rd_req_fifo_full_wr_en_0    <= 1'b0;
                    dup_rd_req_fifo_full_wr_addr_0  <= {C_MAX_QID_WIDTH{1'b0}};
                    dup_rd_req_fifo_full_wr_en_1    <= 1'b0;
                    dup_rd_req_fifo_full_wr_addr_1  <= {C_MAX_QID_WIDTH{1'b0}};
                end else begin

                    if (dup_wr_ptr_buf_wren) begin
                        dup_rd_req_fifo_empty_wr_en_0   <= 1'b1;
                        dup_rd_req_fifo_empty_wr_addr_0 <= dup_wr_ptr_buf_wr_addr;
                        dup_rd_req_fifo_empty_wr_data_0 <= 1'b0;
                    end else if (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) begin
                        dup_rd_req_fifo_empty_wr_en_0   <= 1'b1;
                        dup_rd_req_fifo_empty_wr_addr_0 <= wr_ptr_buf_wr_addr_init;
                        dup_rd_req_fifo_empty_wr_data_0 <= 1'b1;
                    end else begin
                        dup_rd_req_fifo_empty_wr_en_0 <= 1'b0;
                    end

                    if (dup_rd_ptr_buf_wren && dup_rd_ptr_buf_wr_data == dup_mirror_wr_ptr_buf_rd_data && ~((dup_wr_ptr_buf_wren || dup_wr_ptr_buf_wren_r) && dup_wr_ptr_buf_wr_addr == dup_rd_ptr_buf_wr_addr)) begin
                        dup_rd_req_fifo_empty_wr_en_1   <= 1'b1;
                        dup_rd_req_fifo_empty_wr_addr_1 <= dup_rd_ptr_buf_wr_addr;
                        dup_rd_req_fifo_empty_wr_data_1 <= 1'b1;
                    end else begin
                        dup_rd_req_fifo_empty_wr_en_1 <= 1'b0;
                    end

                    if (~ignore_dup_full_check && ~(dup_rd_ptr_buf_wren && (dup_wr_ptr_buf_wr_addr == dup_rd_ptr_buf_wr_addr)) && dup_wr_ptr_buf_wren && 
			(dup_wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH-1:0] == dup_mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]) && 
			(dup_wr_ptr_buf_wr_data[RD_REQ_PTR_WIDTH] != dup_mirror_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH])) begin
                        dup_rd_req_fifo_full_wr_en_1   <= 1'b1;
                        dup_rd_req_fifo_full_wr_addr_1 <= dup_wr_ptr_buf_wr_addr;
                        dup_rd_req_fifo_full_wr_data_1 <= 1'b0;
                    end else begin
                        dup_rd_req_fifo_full_wr_en_1 <= 1'b0;
                    end

                    if (dup_rd_ptr_buf_wren && ~(dup_wr_ptr_buf_wren && (dup_rd_ptr_buf_wr_addr == dup_wr_ptr_buf_wr_addr))) begin
                        dup_rd_req_fifo_full_wr_en_0   <= 1'b1;
                        dup_rd_req_fifo_full_wr_addr_0 <= dup_rd_ptr_buf_wr_addr;
                        dup_rd_req_fifo_full_wr_data_0 <= 1'b0;
                        // when QP is re-initialized, full should be reset for that QP
                    end else if (wr_ptr_buf_wren_init && ~rd_ptr_buf_wren && ~dup_rd_ptr_buf_wren && ~wr_ptr_buf_wren && ~dup_wr_ptr_buf_wren) begin
                        dup_rd_req_fifo_full_wr_en_0   <= 1'b1;
                        dup_rd_req_fifo_full_wr_addr_0 <= wr_ptr_buf_wr_addr_init;
                        dup_rd_req_fifo_full_wr_data_0 <= 1'b0;
                    end else begin
                        dup_rd_req_fifo_full_wr_en_0 <= 1'b0;
                    end
                end
            end

            always @(posedge core_clk) begin
                if (core_rst) begin
                    dup_rd_req_bram_wr_en                 <= 1'b0;
                    dup_rd_req_bram_rd_en                 <= 1'b0;
                    dup_rd_req_bram_rd_en_r               <= 1'b0;
                    dup_rd_req_bram_wr_addr               <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                    dup_rd_req_bram_rd_addr               <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                    dup_rd_req_bram_wr_data               <= 'd0;
                    dup_rd_req_bram_wr_data_r1            <= 'd0;
                    start_with_middle_r                   <= 2'b00;
                    dup_rd_ptr_buf_rd_data_sampled        <= 'd0;
                    dup_mirror_wr_ptr_buf_rd_data_sampled <= 'd0;
                    last_dup_full_status                  <= 1'b0;
                end else begin
                    dup_rd_req_bram_wr_data_r1[151:0] <= {i_rd_req_phy_addr, i_rd_payload_len, i_res_psn, 1'b0, i_res_aeth_syndrome, i_res_msn};
                    last_dup_full_status              <= dup_rd_req_fifo_full_rd_data;
                    if (i_reg_xrnic_en && i_bres_valid_r && (i_req_type_r == 2'b10) && ~last_dup_full_status) begin
                        dup_rd_req_bram_wr_en <= 1'b1;
                        if ((i_bres_dest_qpid_r == dup_wr_ptr_buf_wr_addr) && i_bres_valid_r && dup_wr_ptr_buf_wren_r) begin
                            if (dup_wr_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0] == C_MAX_RD_OS - 1) begin
                                dup_rd_req_bram_wr_addr <= ((C_MAX_RD_OS * i_bres_dest_qpid_r));
                            end else begin
                                dup_rd_req_bram_wr_addr <= ((C_MAX_RD_OS * i_bres_dest_qpid_r) + dup_wr_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]) + {{RD_REQ_PTR_WIDTH - 1{1'b0}}, 1'b1};
                            end
                        end else begin
                            dup_rd_req_bram_wr_addr <= ((C_MAX_RD_OS * i_bres_dest_qpid_r) + dup_wr_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]);
                        end
                    end else begin
                        dup_rd_req_bram_wr_en <= 1'b0;
                    end
                    dup_rd_req_bram_wr_data_r1[153+RD_REQ_PTR_WIDTH : 152] <= wr_ptr_buf_rd_data;
                    dup_rd_req_bram_wr_data_r1[154+RD_REQ_PTR_WIDTH]       <= 1'b0;
                    dup_rd_req_bram_wr_data                                <= dup_rd_req_bram_wr_data_r1;
                    dup_rd_req_bram_rd_en_r                                <= dup_rd_req_bram_rd_en;
                    if (~arb_in_progress && ~aeth_valid_r && ~rd_req_bram_rd_en_r && ~rd_req_bram_rd_en && ~dup_rd_req_bram_rd_en_r && ~dup_rd_req_bram_rd_en && ~rd_req_valid_r && ~dup_rd_req_fifo_empty_rd_data) begin
                        dup_rd_req_bram_rd_en                 <= 1'b1;
                        dup_rd_req_bram_rd_addr               <= ((C_MAX_RD_OS * aeth_bram_rd_addr) + dup_rd_ptr_buf_rd_data[RD_REQ_PTR_WIDTH-1:0]);
                        dup_rd_ptr_buf_rd_data_sampled        <= dup_rd_ptr_buf_rd_data;
                        dup_mirror_wr_ptr_buf_rd_data_sampled <= dup_mirror_wr_ptr_buf_rd_data;
                    end else begin
                        dup_rd_req_bram_rd_en <= 1'b0;
                    end
                end
            end
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////// Read request buffers for QP's ////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            xpm_memory_sdpram #(

                // Common module parameters
                .MEMORY_SIZE      (152 * C_MAX_QP * C_MAX_RD_OS),  //positive integer
                .MEMORY_PRIMITIVE ("ultra"),                       //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),                //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),                        //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                            //string;
                .USE_MEM_INIT     (1),                             //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),               //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                             //integer; 0,1
                .ECC_MODE         ("no_ecc"),                      //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                             //Do not Change

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(152),                                //positive integer
                .BYTE_WRITE_WIDTH_A(8),                                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH), //positive integer

                // Port B module parameters
                .READ_DATA_WIDTH_B (152),                                 //positive integer
                .ADDR_WIDTH_B      (C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),                                 //string
                .READ_LATENCY_B    (2),                                   //non-negative integer
                .WRITE_MODE_B      ("read_first")                         //string; "write_first", "read_first", "no_change" 

            ) rd_req_bram (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (|rd_req_bram_wr_en),
                .wea           ({19{1'b1}}),
                .addra         (rd_req_bram_wr_addr),
                .dina          (rd_req_bram_wr_data[151:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (rd_req_bram_rd_en),
                .regceb  (1'b1),
                .addrb   (rd_req_bram_rd_addr),
                .doutb   (rd_req_bram_rd_data[151:0]),
                .sbiterrb(),
                .dbiterrb()

            );
            assign rd_req_bram_rd_data[175:152] = 8'h00;

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////// Duplicate read request buffers for QP's //////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            xpm_memory_sdpram #(

                // Common module parameters
                .MEMORY_SIZE      (152 * C_MAX_QP * C_MAX_RD_OS),  //positive integer
                .MEMORY_PRIMITIVE ("ultra"),                       //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),                //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),                        //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                            //string;
                .USE_MEM_INIT     (1),                             //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),               //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                             //integer; 0,1
                .ECC_MODE         ("no_ecc"),                      //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                             //Do not Change

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(152),                                //positive integer
                .BYTE_WRITE_WIDTH_A(8),                                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH), //positive integer

                // Port B module parameters
                .READ_DATA_WIDTH_B (152),                                 //positive integer
                .ADDR_WIDTH_B      (C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),                                 //string
                .READ_LATENCY_B    (2),                                   //non-negative integer
                .WRITE_MODE_B      ("read_first")                         //string; "write_first", "read_first", "no_change" 

            ) dup_rd_req_bram (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (dup_rd_req_bram_wr_en),
                .wea           ({19{1'b1}}),
                .addra         (dup_rd_req_bram_wr_addr),
                .dina          (dup_rd_req_bram_wr_data[151:0]),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (dup_rd_req_bram_rd_en),
                .regceb  (1'b1),
                .addrb   (dup_rd_req_bram_rd_addr),
                .doutb   (dup_rd_req_bram_rd_data[151:0]),
                .sbiterrb(),
                .dbiterrb()

            );
            assign dup_rd_req_bram_rd_data[159:152] = 8'h00;
        end  //EN_DUP_RD_FIRST
    endgenerate

    generate
        if (C_EN_RECV_RD_WR == 0) begin : NO_RD_RESP
            always @(posedge core_clk) begin
                start_with_middle_r             <= 2'b00;
                //rd_req_bram_wr_en <= {C_MAX_QP{1'b0}};
                //rd_req_bram_rd_en <= {C_MAX_QP{1'b0}};
                rd_req_bram_wr_en               <= 1'b0;
                rd_req_bram_rd_en               <= 1'b0;
                rd_req_bram_rd_en_r             <= 1'b0;
                rd_req_bram_wr_addr             <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                rd_req_bram_rd_addr             <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                rd_req_bram_wr_data             <= 'd0;
                o_rd_req_addr                   <= 64'd00;
                o_rd_req_len                    <= 32'd0;
                o_rd_req_valid                  <= 1'b0;
                o_rd_req_start_with_middle      <= 2'b00;
                dup_rd_req_bram_wr_en           <= 1'b0;
                dup_rd_req_bram_rd_en           <= 1'b0;
                dup_rd_req_bram_rd_en_r         <= 1'b0;
                dup_rd_req_en                   <= 1'b0;
                rd_req_valid_r                  <= 1'b0;
                dup_rd_req_bram_wr_addr         <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                dup_rd_req_bram_rd_addr         <= {C_MAX_QID_WIDTH + RD_REQ_PTR_WIDTH{1'b0}};
                dup_rd_req_bram_wr_data         <= 'd0;
                discard_rd_req                  <= 1'b0;
                discard_rd_req_r                <= 1'b0;
                rd_req_fifo_empty_wr_en_0       <= 1'b0;
                rd_req_fifo_empty_wr_en_1       <= 1'b0;
                rd_req_fifo_full_wr_en_0        <= 1'b0;
                rd_req_fifo_full_wr_en_1        <= 1'b0;
                dup_rd_req_fifo_empty_wr_en_0   <= 1'b0;
                dup_rd_req_fifo_empty_wr_en_1   <= 1'b0;
                dup_rd_req_fifo_full_wr_en_0    <= 1'b0;
                dup_rd_req_fifo_full_wr_en_1    <= 1'b0;
                rd_req_fifo_full_wr_en_0        <= 1'b0;
                rd_req_fifo_full_wr_en_1        <= 1'b0;
                wr_ptr_buf_wren_init            <= 1'b0;
                rd_req_fifo_empty_wr_addr_0     <= 'd0;
                rd_req_fifo_empty_wr_data_0     <= 1'b0;
                rd_req_fifo_empty_wr_addr_1     <= 'd0;
                rd_req_fifo_empty_wr_data_1     <= 1'b0;
                dup_rd_req_fifo_empty_wr_addr_0 <= 'd0;
                dup_rd_req_fifo_empty_wr_data_0 <= 1'b0;
                dup_rd_req_fifo_empty_wr_addr_1 <= 'd0;
                dup_rd_req_fifo_empty_wr_data_1 <= 1'b0;
                rd_req_fifo_full_wr_addr_0      <= 'd0;
                rd_req_fifo_full_wr_addr_1      <= 'd0;
                dup_rd_req_fifo_full_wr_addr_0  <= 'd0;
                dup_rd_req_fifo_full_wr_data_0  <= 1'b0;
                dup_rd_req_fifo_full_wr_addr_1  <= 'd0;
                dup_rd_req_fifo_full_wr_data_1  <= 1'b0;
            end
            assign rd_req_bram_rd_data          = 'd0;
            assign dup_rd_req_bram_rd_data      = 'd0;
            assign ptr_init_done                = 1'b0;
            assign rd_req_fifo_full_set_valid   = 1'b0;
            assign rd_req_fifo_full_set_id      = 'd0;
            assign rd_req_fifo_full_reset_valid = 1'b0;
            assign rd_req_fifo_full_reset_id    = 'd0;
        end
    endgenerate



endmodule
