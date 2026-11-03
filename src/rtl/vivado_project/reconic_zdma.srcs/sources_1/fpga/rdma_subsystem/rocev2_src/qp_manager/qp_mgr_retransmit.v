/////////////////////////////////////////////////////////////////
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
// Filename        : qp_mgr_retransmit.v
// Version         : v1.0
// Description     : QP manager retrnsmission logic. 
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- qp mgr top
//                     -- qp_mgr_retransmit.v
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
module qp_mgr_retransmit #(
    parameter C_NUM_QP             = 128,
    parameter C_WQE_FIFO_IDX_WIDTH = 3,
    parameter C_QP_INDX_WIDTH      = 7
) (
    input wire core_clk,
    input wire core_rstn,

    // Configuration details
    input  wire                          i_retransmit_reqd,
    input  wire [C_QP_INDX_WIDTH -1 : 0] i_retransmit_qpid,          // QP id for this retransmission is required
    input  wire [                47 : 0] i_retransmit_wqe_bram_din,
    output wire                          o_retransmit_accepted,
    input  wire [                  23:0] i_psn_to_retry,
    input  wire [                  23:0] i_ssn_to_retry,

    input wire [C_WQE_FIFO_IDX_WIDTH : 0] i_num_valid_entries,

    output wire                        o_qp_cq_hdptr_req,
    input  wire                        i_qp_cq_hdptr_valid,
    input  wire [                15:0] i_qp_cq_hdptr,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_cq_hdptr_idx,    // Index for reading Q depth

    // configuration that is updated
    output wire [                15:0] o_qp_curr_sqptr_proc,
    output wire                        o_qp_curr_sqptr_proc_wen,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_curr_sqptr_proc_idx,  // Index for writing curent SQ pointer

    output wire [                23:0] o_qp_stat_ret_sq_psn,
    output wire                        o_qp_stat_ret_sq_psn_wen,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_stat_ret_sq_psn_idx,
    input  wire                        i_qp_stat_ret_sq_psn_valid,

    output wire                        o_qp_sq_psn_req,
    input  wire                        i_qp_sq_psn_valid,
    input  wire [                23:0] i_qp_sq_psn,
    output wire                        o_qp_sq_psn_wrn,
    output wire [                23:0] o_qp_sq_psn,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_sq_psn_idx,    // Index for reading Q depth

    output wire [C_QP_INDX_WIDTH -1:0] o_qp_stat_ssn_idx,   // Index for reading/writing MSN
    output wire [                23:0] o_qp_stat_ssn,       // Expected MSN for Qp incoming messages
    input  wire [                23:0] i_qp_stat_ssn,       // Expected MSN for Qp incoming messages write data from hdr validation
    output wire                        o_qp_stat_ssn_req,
    output wire                        o_qp_stat_ssn_wrn,
    input  wire                        i_qp_stat_ssn_valid,

    // Interface to SQ WQE FIFO
    input  wire [511:0] i_wqe,
    output wire [511:0] o_wqe,
    output wire         o_wqe_fifo_push,
    output wire         o_wqe_fifo_pop,

    output wire                          o_halt,         // 重传流程占用中，暂停该 QP 的正常发送路径
    output wire [C_QP_INDX_WIDTH -1 : 0] o_halted_qpid,  // 重传 QP 号
    input  wire                          i_halted,

    input  wire                         i_osq_pop,
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_osq_pop_qpid,
    output reg                          o_wqe_dropped,
    output wire [C_QP_INDX_WIDTH-1 : 0] o_wqe_dropped_qpid,

    output reg  [                 48:0] o_retransmit_wqe_bram_din,
    input  wire [                 48:0] i_retransmit_wqe_bram_dout,
    output reg                          o_retransmit_wqe_bram_wen,
    output reg  [C_QP_INDX_WIDTH-1 : 0] o_retransmit_wqe_bram_idx,
    input                               i_prog_full_db_update_curr_sqptr
);
    `include "../../rdma_macros.vh"


    localparam IDLE = 4'b0000;
    localparam HALT_PROCESS = 4'b0001;
    localparam GET_CONF = 4'b0010;
    localparam UPD_CONF = 4'b0011;
    localparam WAIT_FOR_PROG_FULL = 4'b1001;
    localparam CLEAN_SWQE_FIFO = 4'b0100;
    localparam POP_SWQE = 4'b0101;
    localparam PUSH_SWQE = 4'b0110;
    localparam CHECK_SWQE = 4'b0111;
    localparam WQE_DROPPED = 4'b1000;


    reg [                     3:0] retry_ns;
    reg [                     3:0] retry_cs;
    reg [C_WQE_FIFO_IDX_WIDTH : 0] sq_fifo_unclean_entries_ff;
    reg [C_WQE_FIFO_IDX_WIDTH : 0] sq_fifo_unclean_entries;
    reg [                    15:0] qp_cq_hdptr_ff;
    reg [                    23:0] qp_sq_psn_ff;
    reg [                    23:0] qp_stat_ssn_ff;
    reg [  C_QP_INDX_WIDTH -1 : 0] qp_retry_idx;
    reg [                   511:0] wqe_ff;
    reg [                    23:0] psn_to_retry_ff;
    reg [                    23:0] ssn_to_retry_ff;
    reg [                    47:0] retransmit_wqe_bram_din1;

    assign o_wqe_dropped_qpid = qp_retry_idx;
    //always @ (posedge core_clk or negedge core_rstn)
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            retry_cs                  <= IDLE;
            o_wqe_dropped             <= 1'b0;
            o_retransmit_wqe_bram_din <= 'd0;
            o_retransmit_wqe_bram_idx <= 'd0;
            o_retransmit_wqe_bram_wen <= 1'b0;
            retransmit_wqe_bram_din1  <= 'd0;
            //o_wqe_dropped_qpid <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            retry_cs <= retry_ns;
            if (retry_cs == IDLE && i_retransmit_reqd) begin
                retransmit_wqe_bram_din1  <= i_retransmit_wqe_bram_din;
                o_retransmit_wqe_bram_idx <= i_retransmit_qpid;
            end
            if (retry_cs == CLEAN_SWQE_FIFO && (sq_fifo_unclean_entries_ff == 'b0)) begin
                o_retransmit_wqe_bram_din[48]    <= ~|retransmit_wqe_bram_din1[23:0] ? i_retransmit_wqe_bram_dout[48] : ~i_retransmit_wqe_bram_dout[48];
                o_retransmit_wqe_bram_din[23:0]  <= retransmit_wqe_bram_din1[23:0];
                o_retransmit_wqe_bram_din[47:24] <= retransmit_wqe_bram_din1[47:24];
                o_retransmit_wqe_bram_wen        <= 1'b1;
            end else begin
                o_retransmit_wqe_bram_wen <= 1'b0;
            end
            if ((retry_cs == WQE_DROPPED) && ~(i_osq_pop && (i_osq_pop_qpid == qp_retry_idx))) begin
                o_wqe_dropped <= 1'b1;
            end else begin
                o_wqe_dropped <= 1'b0;
            end
        end
    end


    always @(*) begin
        retry_ns                = retry_cs;
        sq_fifo_unclean_entries = sq_fifo_unclean_entries_ff;
        case (retry_cs)

            IDLE: begin
                if (i_retransmit_reqd) begin
                    retry_ns = HALT_PROCESS;
                end
            end

            HALT_PROCESS: begin
                if (i_halted) retry_ns = GET_CONF;
            end

            GET_CONF: begin
                sq_fifo_unclean_entries = i_num_valid_entries;
                if (i_qp_cq_hdptr_valid & i_qp_sq_psn_valid & i_qp_stat_ssn_valid) retry_ns = UPD_CONF;
            end

            UPD_CONF: begin
                if (i_qp_sq_psn_valid & i_qp_stat_ret_sq_psn_valid) retry_ns = !i_prog_full_db_update_curr_sqptr ? CLEAN_SWQE_FIFO : WAIT_FOR_PROG_FULL;
            end

            WAIT_FOR_PROG_FULL: begin
                if (!i_prog_full_db_update_curr_sqptr) retry_ns = CLEAN_SWQE_FIFO;
            end

            CLEAN_SWQE_FIFO: begin
                if (sq_fifo_unclean_entries_ff != 'b0) begin
                    retry_ns = POP_SWQE;
                end else begin
                    retry_ns = IDLE;
                end
            end

            POP_SWQE: begin
                retry_ns = CHECK_SWQE;
            end

            CHECK_SWQE: begin
                sq_fifo_unclean_entries = sq_fifo_unclean_entries_ff - 1'b1;
                if (i_wqe[136+:C_QP_INDX_WIDTH] == qp_retry_idx) begin  // location of the inserted QP ID
                    //retry_ns = CLEAN_SWQE_FIFO;
                    retry_ns = WQE_DROPPED;
                end else retry_ns = PUSH_SWQE;
            end
            PUSH_SWQE: retry_ns = CLEAN_SWQE_FIFO;

            WQE_DROPPED:
            if (~(i_osq_pop && (i_osq_pop_qpid == qp_retry_idx))) begin
                retry_ns = CLEAN_SWQE_FIFO;
            end

        endcase
    end


    `MSFF_R(sq_fifo_unclean_entries_ff, sq_fifo_unclean_entries, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_psn_ff, (((retry_cs == GET_CONF) & i_qp_sq_psn_valid) ? i_qp_sq_psn : qp_sq_psn_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_stat_ssn_ff, (i_qp_stat_ssn_valid ? i_qp_stat_ssn : qp_stat_ssn_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_hdptr_ff, (i_qp_cq_hdptr_valid ? i_qp_cq_hdptr : qp_cq_hdptr_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_retry_idx, (retry_cs == IDLE ? i_retransmit_qpid : qp_retry_idx), core_clk, ~core_rstn)

    `MSFF_R(wqe_ff, i_wqe, core_clk, ~core_rstn)

    assign o_wqe                = wqe_ff;
    assign o_qp_stat_ret_sq_psn = qp_sq_psn_ff;

    `MSFF_R(psn_to_retry_ff, ((i_retransmit_reqd & (retry_cs == IDLE)) ? i_psn_to_retry : psn_to_retry_ff), core_clk, ~core_rstn)
    `MSFF_R(ssn_to_retry_ff, ((i_retransmit_reqd & (retry_cs == IDLE)) ? i_ssn_to_retry : ssn_to_retry_ff), core_clk, ~core_rstn)

    assign o_qp_sq_psn              = psn_to_retry_ff;  //{qp_sq_psn_ff[23:8], (i_psn_to_retry)};
    assign o_qp_stat_ssn            = ssn_to_retry_ff;  //{qp_stat_ssn_ff[23:8], (i_ssn_to_retry)};
    assign o_qp_curr_sqptr_proc     = qp_cq_hdptr_ff;  //(qp_cq_hdptr_ff == 16'b0) ? 16'h1 : qp_cq_hdptr_ff; //- 1'b1;

    assign o_qp_sq_psn_req          = ((retry_cs == GET_CONF) || (retry_cs == UPD_CONF)) ? 1'b1 : 1'b0;
    assign o_qp_stat_ssn_req        = (retry_cs == GET_CONF) ? 1'b1 : 1'b0;
    assign o_qp_cq_hdptr_req        = (retry_cs == GET_CONF) ? 1'b1 : 1'b0;

    assign o_qp_sq_psn_wrn          = (retry_cs == UPD_CONF) ? 1'b1 : 1'b0;
    assign o_qp_stat_ret_sq_psn_wen = (retry_cs == UPD_CONF) ? 1'b1 : 1'b0;
    assign o_qp_stat_ssn_wrn        = (retry_cs == UPD_CONF) ? 1'b1 : 1'b0;
    assign o_qp_curr_sqptr_proc_wen = (retry_cs == UPD_CONF || retry_cs == WAIT_FOR_PROG_FULL) ? 1'b1 : 1'b0;

    assign o_wqe_fifo_push          = (retry_cs == PUSH_SWQE) ? 1'b1 : 1'b0;
    assign o_wqe_fifo_pop           = (retry_cs == POP_SWQE) ? 1'b1 : 1'b0;

    assign o_halt                   = (retry_cs != IDLE) ? 1'b1 : 1'b0;
    assign o_halted_qpid            = qp_retry_idx;
    assign o_qp_stat_ssn_idx        = qp_retry_idx;
    assign o_qp_sq_psn_idx          = qp_retry_idx;
    assign o_qp_stat_ret_sq_psn_idx = qp_retry_idx;
    assign o_qp_curr_sqptr_proc_idx = qp_retry_idx;
    assign o_qp_cq_hdptr_idx        = qp_retry_idx;

    //assign o_retransmit_done = ((retry_cs == CLEAN_SWQE_FIFO) && (sq_fifo_unclean_entries_ff == 'd0)) ? 1'b1 : 1'b0;
    assign o_retransmit_accepted    = (retry_cs == IDLE) ? 1'b1 : 1'b0;



    //-------------------------------------------------------------------------------------
    // Logic to see FSM states in Questa wavedumps (what a pain!!) :-/
    // Non-synthesizable code
    //-------------------------------------------------------------------------------------

`ifdef SIMULATION
    reg [20*8-1:0] retry_cs_string = "null";
    reg [20*8-1:0] retry_ns_string = "null";

    always @* begin
        case (retry_cs)
            IDLE:               retry_cs_string = "IDLE           ";
            HALT_PROCESS:       retry_cs_string = "HALT_PROCESS   ";
            GET_CONF:           retry_cs_string = "GET_CONF       ";
            UPD_CONF:           retry_cs_string = "UPD_CONF       ";
            WAIT_FOR_PROG_FULL: retry_cs_string = "WAIT_FOR_PROG_FULL       ";
            CLEAN_SWQE_FIFO:    retry_cs_string = "CLEAN_SWQE_FIFO";
            POP_SWQE:           retry_cs_string = "POP_SWQE       ";
            PUSH_SWQE:          retry_cs_string = "PUSH_SWQE      ";
            CHECK_SWQE:         retry_cs_string = "CHECK_SWQE     ";
        endcase
        case (retry_ns)
            IDLE:               retry_ns_string = "IDLE           ";
            HALT_PROCESS:       retry_ns_string = "HALT_PROCESS   ";
            GET_CONF:           retry_ns_string = "GET_CONF       ";
            UPD_CONF:           retry_ns_string = "UPD_CONF       ";
            WAIT_FOR_PROG_FULL: retry_cs_string = "WAIT_FOR_PROG_FULL       ";
            CLEAN_SWQE_FIFO:    retry_ns_string = "CLEAN_SWQE_FIFO";
            POP_SWQE:           retry_ns_string = "POP_SWQE       ";
            PUSH_SWQE:          retry_ns_string = "PUSH_SWQE      ";
            CHECK_SWQE:         retry_ns_string = "CHECK_SWQE     ";
        endcase
    end
`endif

    //-------------------------------------------------------------------------------------

endmodule
