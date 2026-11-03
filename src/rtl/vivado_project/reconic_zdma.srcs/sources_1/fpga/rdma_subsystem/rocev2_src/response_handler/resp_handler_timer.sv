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
// resp_handler_timer.sv
//-----------------------------------------------------------------------------
//
// Filename        : resp_handler_timer.sv
// Version         : v1.0
// Description     : Module handling timeout generation
// Verilog-Standard: System Verilog
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- resp_handler_top.v
//                     -- resp_handler_timer.sv
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

// (* DowngradeIPIdentifiedWarnings="yes" *)
module resp_handler_timer #(
    parameter C_NUM_QP           = 256,
    parameter C_QP_INDX_WIDTH    = 8,
    parameter C_NUM_CYCLES_4US   = 512,
    parameter C_OS_Q_INDX_WIDTH  = 3,
    parameter LOG2_NUM_QP        = $clog2(C_NUM_QP),
    parameter C_TIMER_VAL_32_7us = 6554,
    parameter C_TIMER_VAL_4_1us  = 820,
    parameter SUBTRACT_VAL       = 1
) (
    // clocks and resets
    input wire core_clk,
    input wire core_rstn,

    input  wire  [                    3:0] i_cfg_base_cnt,
    output logic                           o_q_timed_out_valid,
    output logic [      LOG2_NUM_QP-1 : 0] o_q_timed_out_wr_addr,
    //Due to per QP based signals input interface has to be changed
    input  wire  [(2*LOG2_NUM_QP+3)-1 : 0] osq_empty,
    input  wire  [(2*LOG2_NUM_QP+4)-1 : 0] timer_disabled_per_qp,
    input  wire  [(2*LOG2_NUM_QP+2)-1 : 0] all_resp_received,
    input  wire  [(2*LOG2_NUM_QP+3)-1 : 0] osq_rnr_nacked_ff,
    input  wire  [  (LOG2_NUM_QP+2)-1 : 0] i_qp_fatal,
    input  wire                            i_reload_timer_wqe,
    input  wire  [      LOG2_NUM_QP - 1:0] i_reload_timer_wqe_qpid,
    input  wire                            i_reload_timer_acknack,
    input  wire  [      LOG2_NUM_QP - 1:0] i_reload_timer_acknack_qpid,
    input  wire                            i_rnr_nak_rcvd,
    input  wire  [      LOG2_NUM_QP - 1:0] i_rnr_nak_qpid,
    input  wire  [                    4:0] i_rnr_nak_timer,
    input  wire  [                    4:0] i_reload_value_wqe,
    input  wire  [                    4:0] i_reload_value_acknack,
    input  wire                            init_start,
    input  wire  [      LOG2_NUM_QP-1 : 0] init_count,
    input  wire                            osq_nacked_timer_mem_rdaddr,
    output wire  [      LOG2_NUM_QP-1 : 0] o_timer_mem_rdaddr,
    input  wire                            osq_nacked_ff_rd_data,
    input  wire                            i_qp_under_recovery_wr_en,
    input  wire  [      LOG2_NUM_QP-1 : 0] i_qp_under_recovery_wr_idx,
    input  wire                            i_qp_under_recovery_wr_data
);

    `include "../../rdma_macros.vh"
    //==========================================================================
    // local parameters
    //==========================================================================
    localparam LUTRAM_WIDTH = 5 + 5;
    localparam W_4_1us_CNT = 4;
    localparam INPUT_FIFO_WIDTH = LOG2_NUM_QP + 5 + W_4_1us_CNT;
    localparam NUM_REQS = 2;
    localparam GNT_RNR = 0;
    localparam GNT_WQE = 1;
    localparam GNT_RST = 2;
    localparam ARB_GNT_RNR = 1;
    localparam ARB_GNT_WQE = 2;
    localparam ARB_GNT_RST = 4;
    //localparam MEM_DATA_WIDTH   = 64;
    localparam MEM_DATA_WIDTH = 43;
    localparam C_RES_VAL = 4'b1001;

    //==========================================================================
    // structures
    //==========================================================================
    typedef struct packed {
        logic [W_4_1us_CNT-1:0] trigger_count;
        //logic [31:0] init_value;
        logic [4:0]             init_value;
        logic                   cntr_value_0;
        logic                   final_count;
        logic [31:0]            cntr_value;
    } memdata;

    memdata mem_a_wr_data, mem_b_wr_data, mem_b_rd_data;



    //==========================================================================
    // registers and wire declarations
    //==========================================================================

    logic                        timer_en_fifo_empty;
    logic                        timer_en_fifo_rden;
    logic [   LOG2_NUM_QP - 1:0] timer_en_fifo_qpid_out;
    logic                        rst_en_fifo_empty;
    logic                        rst_en_fifo_rden;
    logic [   LOG2_NUM_QP - 1:0] rst_en_fifo_qpid_out;
    logic                        wqe_reload_fifo_empty;
    logic                        wqe_reload_fifo_rden;
    logic [   LOG2_NUM_QP - 1:0] wqe_reload_fifo_qpid_out;
    logic [                 4:0] wqe_reload_fifo_cntr_out;
    logic                        reload_rnr_fifo_wren;
    logic [   LOG2_NUM_QP - 1:0] reload_rnr_fifo_qpid_in;
    logic [                 4:0] reload_rnr_fifo_cntr_in;
    logic                        reload_rnr_fifo_empty;
    logic                        reload_rnr_fifo_rden;
    logic [   LOG2_NUM_QP - 1:0] reload_rnr_fifo_qpid_out;
    logic [                 4:0] reload_rnr_fifo_cntr_out;
    logic [      NUM_REQS - 1:0] arb_reqs;
    logic [      NUM_REQS - 1:0] arb_gnt;
    logic                        arb_done;
    logic [   LOG2_NUM_QP - 1:0] qpid_to_update;
    //logic [31:0] 		  cntr_to_update;
    logic [               4 : 0] cntr_to_update;
    logic                        timer_enable;
    logic [                12:0] free_running_timer_32_7us;
    logic [                 9:0] free_running_timer_4_1_us;
    logic [     W_4_1us_CNT-1:0] trigger_count_4_1us;
    logic [     W_4_1us_CNT-1:0] trigger_count_4_1us_out;
    logic [     W_4_1us_CNT-1:0] trigger_count_4_1us_rnr_out;
    logic [   LOG2_NUM_QP - 1:0] timer_en_qpid;
    logic [   LOG2_NUM_QP - 1:0] rst_timer_qpid;
    logic [   LOG2_NUM_QP - 1:0] wqe_timer_qpid;
    logic [   LOG2_NUM_QP - 1:0] reload_timer_qpid;
    logic [   LOG2_NUM_QP - 1:0] nak_rcvd_qpid;
    logic                        timer_mem_wren;
    logic [   LOG2_NUM_QP - 1:0] timer_mem_wraddr;
    logic [MEM_DATA_WIDTH - 1:0] timer_mem_wrdata;
    logic                        terminal_count;
    logic                        terminal_count_4_1;
    logic                        qpid_proc_done;
    logic [   LOG2_NUM_QP - 1:0] qpid_proc;
    logic                        timer_mem_rden;
    logic [   LOG2_NUM_QP - 1:0] timer_mem_rdaddr;
    reg   [   LOG2_NUM_QP - 1:0] timer_mem_rdaddr_comb;
    logic [MEM_DATA_WIDTH - 1:0] timer_mem_rddata;
    logic [MEM_DATA_WIDTH - 1:0] timer_mem_mod_rddata;
    logic                        timer_mem_mod_wren;
    logic [                 4:0] rnr_nak_reload_val;
    logic                        rd_qp_incr;

    logic [41:0] global_timer, global_timer_ff, global_timer_toggled;
    logic                         collison_wr_rd;
    wire                          osq_nacked_reset_valid;
    wire                          timer_reload_for_wqe_ff_QP1;
    wire                          timer_reload_for_acknack_ff_valid;
    wire                          timer_reload_for_acknack_ff_data;
    wire                          osq_timed_out_sticky_wire_reset_valid;
    wire                          osq_timed_out_sticky_wire_set_valid;
    wire                          clr_timeout_n_nack_set_valid;
    wire                          i_qp_disable_pulse_set_valid;
    wire                          osq_nacked_ff_valid_pending;
    wire                          osq_nacked_ff_valid;
    wire                          osq_nacked_ff_data;
    wire                          osq_rnr_nacked_ff_valid_pending;
    wire                          osq_rnr_nacked_ff_valid;
    wire                          osq_rnr_nacked_ff_data;
    wire                          clr_pending_ack_timer_reload_for_acknack_set_valid;
    wire                          clr_pending_ack_timer_reload_for_acknack_reset_valid;
    wire  [C_QP_INDX_WIDTH-1 : 0] timer_reload_for_acknack_ff_id;
    wire  [C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_wire_reset_id;
    wire  [C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_wire_set_id;
    wire  [C_QP_INDX_WIDTH-1 : 0] clr_timeout_n_nack_set_id;
    wire  [C_QP_INDX_WIDTH-1 : 0] i_qp_disable_pulse_set_id;
    wire  [C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id;
    wire  [C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id_pending;
    wire  [C_QP_INDX_WIDTH-1 : 0] osq_rnr_nacked_ff_id;
    wire  [C_QP_INDX_WIDTH-1 : 0] osq_rnr_nacked_ff_id_pending;
    wire  [C_QP_INDX_WIDTH-1 : 0] clr_pending_ack_timer_reload_for_acknack_set_id;
    wire  [C_QP_INDX_WIDTH-1 : 0] clr_pending_ack_timer_reload_for_acknack_reset_id;
    reg                           osq_timed_out_rd_data_QP_1;
    reg                           osq_timed_out_rd_data_QP_not_1;
    reg                           timer_en;
    reg                           timer_reset;
    wire                          timer_reload_pending_int;
    wire                          timer_reload_pending;
    wire                          osq_rnr_nacked_ff_rd_data_int;
    wire                          osq_rnr_nacked_ff_rd_data;
    wire                          osq_empty_rd_data_int_0;
    wire                          osq_empty_rd_data_0;
    wire                          timer_disabled_per_qp_rd_data_int;
    wire                          timer_disabled_per_qp_rd_data;
    reg                           timeout_mask_set_valid;
    reg   [    LOG2_NUM_QP-1 : 0] timeout_mask_set_id;
    reg                           timeout_mask_reset_valid;
    reg   [    LOG2_NUM_QP-1 : 0] timeout_mask_reset_id;
    reg                           timeout_mask_set_valid_pending;
    reg   [    LOG2_NUM_QP-1 : 0] timeout_mask_set_id_pending;
    wire                          reset_timer_osq_nacked;
    wire                          qp_under_recovery_rd_data;
    wire                          qp_fatal_rd_data;
    logic [    W_4_1us_CNT-1 : 0] trigger_count;


    //==========================================================================
    // fsm states
    //==========================================================================
    typedef enum logic [2:0] {
        RD_CNTR,
        WAIT_RD_CNTR,
        MOD_CNTR,
        WR_CNTR,
        WAIT_WR_CNTR
    } states_t;
    states_t curr_st, next_st;

    `MSFF_R(global_timer, (global_timer + 1'b1), core_clk, ~core_rstn)
    `MSFF_R(global_timer_ff, (global_timer), core_clk, ~core_rstn)

    // Single cycle pulse
    assign global_timer_toggled = global_timer ^ global_timer_ff;


    assign timer_mem_wrdata     = mem_a_wr_data;
    assign mem_b_rd_data        = timer_mem_rddata;
    assign timer_mem_mod_rddata = mem_b_wr_data;

    // free running timer @each 32.768 us
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            free_running_timer_32_7us <= 13'h0;
        end else if (free_running_timer_32_7us == C_TIMER_VAL_32_7us - 1) begin
            free_running_timer_32_7us <= 13'h0;
        end else begin
            free_running_timer_32_7us <= free_running_timer_32_7us + 1'b1;
        end
    end

    // terminal count is when timer hits its max value
    assign terminal_count = (free_running_timer_32_7us == C_TIMER_VAL_32_7us - 1);

    // free running timer @each 4.1 us
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            free_running_timer_4_1_us <= 10'h0;
        end else if ((free_running_timer_4_1_us == C_TIMER_VAL_4_1us - 1) || terminal_count) begin
            free_running_timer_4_1_us <= 10'h0;
        end else begin
            free_running_timer_4_1_us <= free_running_timer_4_1_us + 1'b1;
        end
    end

    // terminal count is when timer hits its max value
    assign terminal_count_4_1 = (free_running_timer_4_1_us == C_TIMER_VAL_4_1us - 1);

    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            trigger_count_4_1us <= 'd0;
        end else if (terminal_count) begin
            trigger_count_4_1us <= 'd0;
        end else if (terminal_count_4_1) begin
            trigger_count_4_1us <= trigger_count_4_1us + 1;
        end
    end

    /***************************************************************************
   *                                                                          *
   * 									      *
   *     	 FIFO based logic to arbitrate between requests		      *
   * 									      *
   * 									      *
   ***************************************************************************/

    //==========================================================================
    // WQE timer reload enable FIFO 
    //==========================================================================

    assign wqe_timer_qpid = i_reload_timer_wqe_qpid;

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
        .READ_DATA_WIDTH    (INPUT_FIFO_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (INPUT_FIFO_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_wqe_reload (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (i_reload_timer_wqe),
        .din          ({trigger_count_4_1us, i_reload_value_wqe, wqe_timer_qpid}),
        .rd_en        (wqe_reload_fifo_rden),
        .empty        (wqe_reload_fifo_empty),
        .dout         ({trigger_count_4_1us_out, wqe_reload_fifo_cntr_out, wqe_reload_fifo_qpid_out}),
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

    //==========================================================================
    // Reload acknack/RNR NAK received FIFO 
    //==========================================================================

    // one of it only will be loaded at any given time. not both at the same time
    // according to Ravi B
    assign reload_timer_qpid    = i_reload_timer_acknack_qpid;
    assign nak_rcvd_qpid        = i_rnr_nak_qpid;

    assign reload_rnr_fifo_wren = (i_reload_timer_acknack) | (i_rnr_nak_rcvd);
    always_comb begin
        if (|i_reload_timer_acknack) begin
            reload_rnr_fifo_qpid_in = reload_timer_qpid;
            reload_rnr_fifo_cntr_in = i_reload_value_acknack;
        end else begin
            reload_rnr_fifo_qpid_in = nak_rcvd_qpid;
            reload_rnr_fifo_cntr_in = rnr_nak_reload_val;
        end
    end

    always_comb begin
        case (i_rnr_nak_timer)
            5'b00001:           rnr_nak_reload_val = 5'b00010;
            5'b00010, 5'b00011: rnr_nak_reload_val = 5'b00011;
            5'b00100, 5'b00101: rnr_nak_reload_val = 5'b00100;
            5'b00110, 5'b00111: rnr_nak_reload_val = 5'b00101;
            5'b01000, 5'b01001: rnr_nak_reload_val = 5'b00110;
            5'b01010, 5'b01011: rnr_nak_reload_val = 5'b00111;
            5'b01100, 5'b01101: rnr_nak_reload_val = 5'b01000;
            5'b01110, 5'b01111: rnr_nak_reload_val = 5'b01001;
            5'b10000, 5'b10001: rnr_nak_reload_val = 5'b01010;
            5'b10010, 5'b10011: rnr_nak_reload_val = 5'b01011;
            5'b10100, 5'b10101: rnr_nak_reload_val = 5'b01100;
            5'b10110, 5'b10111: rnr_nak_reload_val = 5'b01101;
            5'b11000, 5'b11001: rnr_nak_reload_val = 5'b01110;
            5'b11010, 5'b11011: rnr_nak_reload_val = 5'b01111;
            5'b11100, 5'b11101: rnr_nak_reload_val = 5'b10000;
            5'b11111, 5'b11110: rnr_nak_reload_val = 5'b10001;
            // 5'b0_0000
            default:            rnr_nak_reload_val = 5'b10010;
        endcase
    end


    //CR-1104670 Fix (timer should not expire if reload req is rcvd bcz there is fifo dealy bw actual reload and reload req rcvd)  
    always_ff @(posedge core_clk) begin : timeout_mask
        if (!core_rstn) begin
            timeout_mask_set_valid         <= 1'b0;
            timeout_mask_set_id            <= {LOG2_NUM_QP{1'b0}};
            timeout_mask_reset_valid       <= 1'b0;
            timeout_mask_reset_id          <= {LOG2_NUM_QP{1'b0}};
            timeout_mask_set_valid_pending <= 1'b0;
            timeout_mask_set_id_pending    <= {LOG2_NUM_QP{1'b0}};
        end else begin
            if (i_reload_timer_wqe) begin
                timeout_mask_set_valid <= 1'b1;
                timeout_mask_set_id    <= i_reload_timer_wqe_qpid;
            end else if (i_reload_timer_acknack || i_rnr_nak_rcvd) begin
                timeout_mask_set_valid <= 1'b1;
                timeout_mask_set_id    <= i_reload_timer_acknack ? i_reload_timer_acknack_qpid : i_rnr_nak_qpid;
            end else if (timeout_mask_set_valid_pending && !(timer_mem_wren && timer_mem_wraddr == timeout_mask_set_id_pending)) begin
                timeout_mask_set_valid <= 1'b1;
                timeout_mask_set_id    <= timeout_mask_set_id_pending;
            end else begin
                timeout_mask_set_valid <= 1'b0;
            end

            if (i_reload_timer_wqe && i_reload_timer_acknack && i_reload_timer_wqe_qpid != i_reload_timer_acknack_qpid) begin
                timeout_mask_set_valid_pending <= 1'b1;
                timeout_mask_set_id_pending    <= i_reload_timer_acknack_qpid;
            end else if (i_reload_timer_wqe && i_rnr_nak_rcvd && i_reload_timer_wqe_qpid != i_rnr_nak_qpid) begin
                timeout_mask_set_valid_pending <= 1'b1;
                timeout_mask_set_id_pending    <= i_rnr_nak_qpid;
            end else begin
                timeout_mask_set_valid_pending <= 1'b0;
            end

            if (timer_mem_wren && !(i_reload_timer_wqe && timer_mem_wraddr == i_reload_timer_wqe_qpid) && !(i_reload_timer_acknack && timer_mem_wraddr == i_reload_timer_acknack_qpid) && !(i_rnr_nak_rcvd && timer_mem_wraddr == i_rnr_nak_qpid)) begin
                timeout_mask_reset_valid <= 1'b1;
                timeout_mask_reset_id    <= timer_mem_wraddr;
            end else begin
                timeout_mask_reset_valid <= 1'b0;
            end
        end
    end

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_reload_timer_wqe (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || timeout_mask_reset_valid),
        .wr_addr    (init_start ? init_count : timeout_mask_reset_id),
        .wr_data    (1'b0),
        .wr_en_aux  (timeout_mask_set_valid),
        .wr_addr_aux(timeout_mask_set_id),
        .wr_data_aux(1'b1),
        .rd_en      (1'b1),
        .rd_addr    (timer_mem_rdaddr),
        .rd_data    (timer_reload_pending_int)
    );
    assign timer_reload_pending = (timeout_mask_set_valid_pending && timeout_mask_set_id_pending == timer_mem_rdaddr) ? 1'b1 : timer_reload_pending_int;

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
        .READ_DATA_WIDTH    (INPUT_FIFO_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (INPUT_FIFO_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_rnr_reload (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (reload_rnr_fifo_wren),
        .din          ({trigger_count_4_1us, reload_rnr_fifo_cntr_in, reload_rnr_fifo_qpid_in}),
        .rd_en        (reload_rnr_fifo_rden),
        .empty        (reload_rnr_fifo_empty),
        .dout         ({trigger_count_4_1us_rnr_out, reload_rnr_fifo_cntr_out, reload_rnr_fifo_qpid_out}),
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

    /***************************************************************************
   *                                                                          *
   * 									      *
   * 		Arbiter to arbitrate between interfaces			      *
   * 									      *
   * 									      *
   ***************************************************************************/

    assign arb_reqs = {!wqe_reload_fifo_empty, !reload_rnr_fifo_empty};
    assign arb_done = 1'b1;

    // Arbiter module
    arbiter #(
        .NUM_REQS(NUM_REQS),
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

    // read enables to corresponding FIFOs
    assign reload_rnr_fifo_rden = arb_gnt[GNT_RNR];
    assign wqe_reload_fifo_rden = arb_gnt[GNT_WQE];
    //assign rst_en_fifo_rden     = arb_gnt[GNT_RST];

    // value to be updated to which QPID depends on what is arbitrated
    always_comb begin
        case (arb_gnt)
            ARB_GNT_RNR: begin
                qpid_to_update = reload_rnr_fifo_qpid_out;
                //cntr_to_update = (reload_rnr_fifo_cntr_out == 5'h0) ? 32'h0 : (1 << reload_rnr_fifo_cntr_out);
                cntr_to_update = reload_rnr_fifo_cntr_out;
                trigger_count  = trigger_count_4_1us_rnr_out;
            end
            default: begin
                qpid_to_update = wqe_reload_fifo_qpid_out;
                //cntr_to_update = (wqe_reload_fifo_cntr_out == 5'h0) ? 32'h0 : (1 << wqe_reload_fifo_cntr_out);
                cntr_to_update = wqe_reload_fifo_cntr_out;
                trigger_count  = trigger_count_4_1us_out;
            end
        endcase
    end


    // load data calculated into the correct QPID
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            timer_mem_wren   <= 1'b0;
            timer_mem_wraddr <= {LOG2_NUM_QP{1'b0}};
            //timer_mem_wrdata <= {MEM_DATA_WIDTH {1'b0}};
            mem_a_wr_data    <= {MEM_DATA_WIDTH{1'b0}};
        end else if (|arb_gnt) begin
            timer_mem_wren              <= 1'b1;
            timer_mem_wraddr            <= qpid_to_update;
            mem_a_wr_data.init_value    <= cntr_to_update;
            mem_a_wr_data.cntr_value_0  <= (cntr_to_update == 5'h0);
            mem_a_wr_data.final_count   <= 1'b0;
            mem_a_wr_data.cntr_value    <= (1 << cntr_to_update);
            mem_a_wr_data.trigger_count <= trigger_count;
        end else begin
            timer_mem_wren   <= 1'b0;
            timer_mem_wraddr <= {LOG2_NUM_QP{1'b0}};
            mem_a_wr_data    <= {MEM_DATA_WIDTH{1'b0}};
            //timer_mem_wrdata <= {MEM_DATA_WIDTH {1'b0}};
        end
    end

    // counter memory
    xpm_memory_tdpram #(
        .ADDR_WIDTH_A           (LOG2_NUM_QP),
        .ADDR_WIDTH_B           (LOG2_NUM_QP),
        .AUTO_SLEEP_TIME        (0),
        .BYTE_WRITE_WIDTH_A     (MEM_DATA_WIDTH),
        .BYTE_WRITE_WIDTH_B     (MEM_DATA_WIDTH),
        .CASCADE_HEIGHT         (0),
        .CLOCKING_MODE          ("common_clock"),
        .ECC_MODE               ("no_ecc"),
        .MEMORY_INIT_FILE       ("none"),
        .MEMORY_INIT_PARAM      ("0"),
        .MEMORY_OPTIMIZATION    ("true"),
        .MEMORY_PRIMITIVE       ("auto"),
        .MEMORY_SIZE            (C_NUM_QP * MEM_DATA_WIDTH),
        .MESSAGE_CONTROL        (0),
        .READ_DATA_WIDTH_A      (MEM_DATA_WIDTH),
        .READ_DATA_WIDTH_B      (MEM_DATA_WIDTH),
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
        .WRITE_DATA_WIDTH_A     (MEM_DATA_WIDTH),
        .WRITE_DATA_WIDTH_B     (MEM_DATA_WIDTH),
        .WRITE_MODE_A           ("no_change"),
        .WRITE_MODE_B           ("no_change")
    ) u_per_qp_counter (
        .clka          (core_clk),
        .rsta          (!core_rstn),
        .ena           (timer_mem_wren),
        .addra         (timer_mem_wraddr),
        .wea           (timer_mem_wren),
        .dina          (timer_mem_wrdata),
        .douta         (),                                     // NOT USED on this side
        .clkb          (core_clk),
        .rstb          (!core_rstn),
        .web           (timer_mem_mod_wren),
        .regcea        (1'b1),
        .addrb         (timer_mem_rdaddr),
        .regceb        (1'b1),
        .doutb         (timer_mem_rddata),
        .dinb          (timer_mem_mod_rddata),
        .enb           (timer_mem_rden | timer_mem_mod_wren),
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

    // current state flops
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            curr_st <= RD_CNTR;
        end else begin
            curr_st <= next_st;
        end
    end

    // during the two cycles of grant, it's better to stop the read
    assign collison_wr_rd = ((|arb_gnt) & (qpid_to_update == qpid_proc) & (qpid_to_update != 'h0)) | (timer_mem_wren & (timer_mem_wraddr == qpid_proc) & (timer_mem_wraddr != 'h0));


    // process every QPID till all QPIDs are done. when there's a simultaneous access of write to the same QPID
    // from the input side, pause the subtraction logic and then move on
    always_comb begin
        case (curr_st)
            RD_CNTR: begin
                if (terminal_count | !qpid_proc_done) begin
                    // when there's a write at the same time to the same QPID, there's
                    // no need to do a read-modify-write
                    if (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) begin
                        next_st = RD_CNTR;
                    end else begin
                        next_st = MOD_CNTR;
                    end
                end else begin
                    next_st = RD_CNTR;
                end
            end
            MOD_CNTR: begin
                next_st = (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) ? RD_CNTR : WR_CNTR;
            end
            default: begin  // WR_CNTR
                next_st = RD_CNTR;
            end
        endcase
    end

    always_comb begin
        case (curr_st)
            RD_CNTR: begin
                // one cycle after terminal count is set qpid_proc_done is reset and stays
                // reset till all the QPs have been processed
                if (terminal_count | !qpid_proc_done) begin
                    // when there's a write at the same time to the same QPID, there's
                    // no need to do a read-modify-write
                    if (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) begin
                        rd_qp_incr = 1'b1;
                    end else begin
                        rd_qp_incr = 1'b0;
                    end
                end else begin
                    rd_qp_incr = 1'b0;
                end
            end
            MOD_CNTR: begin
                rd_qp_incr = (timer_mem_wren & (timer_mem_wraddr == qpid_proc));
            end
            default: begin  // WR_CNTR:
                rd_qp_incr = 1'b1;
            end
        endcase
    end

    // QPID processing is done. after the first queue is processed, this bit is reset.
    // when last queue is processed, then this bit is set
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            qpid_proc_done <= 1'b1;
        end  // reset proc done when terminal count is set
        else if (terminal_count) begin
            qpid_proc_done <= 1'b0;
        end  // when the last QPID is processed, set qpid_proc_done
        else if (qpid_proc == (C_NUM_QP - 1)) begin
            case (curr_st)
                RD_CNTR: begin
                    if (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) begin
                        qpid_proc_done <= 1'b1;
                    end else begin
                        qpid_proc_done <= 1'b0;
                    end
                end
                MOD_CNTR: begin
                    qpid_proc_done <= (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) ? 1'b1 : 1'b0;
                end
                default: begin
                    qpid_proc_done <= 1'b1;
                end
            endcase
        end
    end

    // processed QPID
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            qpid_proc <= {{(LOG2_NUM_QP - 2) {1'b0}}, 2'b10};
        end else if (rd_qp_incr) begin
            if (qpid_proc == C_NUM_QP - 1) begin
                qpid_proc <= {{(LOG2_NUM_QP - 2) {1'b0}}, 2'b10};
            end else begin
                qpid_proc <= qpid_proc + 1'b1;
            end
        end
    end

    // read when the write side is not using the memory
    assign timer_mem_rden   = ((curr_st == RD_CNTR) & (terminal_count | !qpid_proc_done) & !(timer_mem_wren & (timer_mem_wraddr == qpid_proc)));
    assign timer_mem_rdaddr = qpid_proc;

    always_comb begin
        if (rd_qp_incr && qpid_proc == C_NUM_QP - 1) begin
            timer_mem_rdaddr_comb = {{(LOG2_NUM_QP - 2) {1'b0}}, 2'b10};
        end else if (rd_qp_incr) begin
            timer_mem_rdaddr_comb = qpid_proc + 1;
        end else begin
            timer_mem_rdaddr_comb = qpid_proc;
        end
    end

    //timer_en formation logic 
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty[C_QP_INDX_WIDTH+1]),
        .wr_addr (init_start ? init_count : osq_empty[C_QP_INDX_WIDTH : 1]),
        .wr_data (init_start ? 1'b1 : osq_empty[0]),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr_comb),
        .rd_data (osq_empty_rd_data_int_0)
    );
    assign osq_empty_rd_data_0 = (osq_empty[2*C_QP_INDX_WIDTH+2] && osq_empty[2*C_QP_INDX_WIDTH+1 : C_QP_INDX_WIDTH+2] == timer_mem_rdaddr_comb) ? 1'b0 : osq_empty_rd_data_int_0;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_timer_disable_per_qp (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || timer_disabled_per_qp[2*C_QP_INDX_WIDTH+3]),
        .wr_addr (init_start ? init_count : timer_disabled_per_qp[2*C_QP_INDX_WIDTH+2 : C_QP_INDX_WIDTH+3]),
        .wr_data (init_start ? 1'b0 : timer_disabled_per_qp[C_QP_INDX_WIDTH+2]),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr_comb),
        .rd_data (timer_disabled_per_qp_rd_data_int)
    );
    assign timer_disabled_per_qp_rd_data = (timer_disabled_per_qp[C_QP_INDX_WIDTH+1] && timer_disabled_per_qp[C_QP_INDX_WIDTH : 1] == timer_mem_rdaddr_comb) ? timer_disabled_per_qp[0] : timer_disabled_per_qp_rd_data_int;

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_all_resp_received (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || all_resp_received[2*C_QP_INDX_WIDTH+1]),
        .wr_addr    (init_start ? init_count : all_resp_received[2*C_QP_INDX_WIDTH : C_QP_INDX_WIDTH+1]),
        .wr_data    (1'b0),
        .wr_en_aux  (all_resp_received[C_QP_INDX_WIDTH]),
        .wr_addr_aux(all_resp_received[C_QP_INDX_WIDTH-1 : 0]),
        .wr_data_aux(1'b1),
        .rd_en      (1'b1),
        .rd_addr    (timer_mem_rdaddr_comb),
        .rd_data    (all_resp_received_rd_data)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_rnr_nacked_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_rnr_nacked_ff[C_QP_INDX_WIDTH+1]),
        .wr_addr (init_start ? init_count : osq_rnr_nacked_ff[C_QP_INDX_WIDTH : 1]),
        .wr_data (init_start ? 1'b0 : osq_rnr_nacked_ff[0]),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr_comb),
        .rd_data (osq_rnr_nacked_ff_rd_data_int)
    );
    assign osq_rnr_nacked_ff_rd_data = (osq_rnr_nacked_ff[2*C_QP_INDX_WIDTH+2] && osq_rnr_nacked_ff[2*C_QP_INDX_WIDTH+1 : C_QP_INDX_WIDTH+2] == timer_mem_rdaddr_comb) ? 1'b0 : osq_rnr_nacked_ff_rd_data_int;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_fatal (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_fatal[C_QP_INDX_WIDTH+1]),
        .wr_addr (init_start ? init_count : i_qp_fatal[C_QP_INDX_WIDTH : 1]),
        .wr_data (init_start ? 1'b0 : i_qp_fatal[0]),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr),
        .rd_data (qp_fatal_rd_data)
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
        .rd_addr (timer_mem_rdaddr),
        .rd_data (qp_under_recovery_rd_data)
    );

    //timer_en      =     ((~osq_empty_ff & ~timer_disabled_per_qp & ~all_resp_received) | osq_rnr_nacked_ff);
    `MSFF_R(timer_en, ((!osq_empty_rd_data_0 && !timer_disabled_per_qp_rd_data && !all_resp_received_rd_data) || osq_rnr_nacked_ff_rd_data), core_clk, ~core_rstn)

    assign o_timer_mem_rdaddr     = timer_mem_rdaddr_comb;

    //reset_timer_osq_nacked = (osq_nacked_ff | osq_empty | (i_qp_fatal & ~o_osq_nacked)),
    assign reset_timer_osq_nacked = (timer_reset || ((qp_fatal_rd_data || qp_under_recovery_rd_data) && !osq_nacked_timer_mem_rdaddr));
    `MSFF_R(timer_reset, (osq_nacked_ff_rd_data || osq_empty_rd_data_0), core_clk, !core_rstn)

    // when timer is enabled and reset is de-asserted, timer enabled is set
    assign timer_enable       = timer_en && !reset_timer_osq_nacked;

    // timer value write back
    assign timer_mem_mod_wren = ((curr_st == WR_CNTR) & !(timer_mem_wren & (timer_mem_wraddr == qpid_proc))) & (timer_enable & !mem_b_rd_data.cntr_value_0);

    // modify counter. read it and write back one less than the read out value
    (*USE_DSP48 = "yes"*)

    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            mem_b_wr_data <= {MEM_DATA_WIDTH{1'b0}};
        end else if (curr_st == MOD_CNTR) begin
            // if terminal count indicator is set to 1, then reload the
            // counter value to initial value
            if (mem_b_rd_data.final_count) begin
                mem_b_wr_data.init_value    <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0  <= 1'b0;
                mem_b_wr_data.final_count   <= 1'b0;
                mem_b_wr_data.cntr_value    <= (1 << mem_b_rd_data.init_value);
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end  // if the last count is reached, then terminal count is set to
            // 1 and count value is set to 0
            else if (timer_enable && mem_b_rd_data.trigger_count == C_RES_VAL && (mem_b_rd_data.cntr_value <= (SUBTRACT_VAL << 3))) begin
                mem_b_wr_data.init_value    <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0  <= 1'b0;
                mem_b_wr_data.final_count   <= 1'b1;
                mem_b_wr_data.cntr_value    <= 32'h0;
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end else if (timer_enable && mem_b_rd_data.trigger_count != C_RES_VAL && trigger_count_4_1us > mem_b_rd_data.trigger_count && mem_b_rd_data.cntr_value <= (SUBTRACT_VAL * (trigger_count_4_1us - mem_b_rd_data.trigger_count))) begin
                mem_b_wr_data.init_value    <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0  <= 1'b0;
                mem_b_wr_data.final_count   <= 1'b1;
                mem_b_wr_data.cntr_value    <= 32'h0;
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end else if (timer_enable && mem_b_rd_data.trigger_count != C_RES_VAL && mem_b_rd_data.cntr_value <= (SUBTRACT_VAL * (8 + trigger_count_4_1us - mem_b_rd_data.trigger_count))) begin
                mem_b_wr_data.init_value    <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0  <= 1'b0;
                mem_b_wr_data.final_count   <= 1'b1;
                mem_b_wr_data.cntr_value    <= 32'h0;
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end  // in all other conditions, it's subtracted by a value of SUBTRACT_VAL
            else begin
                mem_b_wr_data.init_value   <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0 <= 1'b0;
                mem_b_wr_data.final_count  <= 1'b0;
                if (mem_b_rd_data.trigger_count != C_RES_VAL) begin  //when a reload happened within 32.768 us
                    if (trigger_count_4_1us > mem_b_rd_data.trigger_count) begin
                        mem_b_wr_data.cntr_value <= mem_b_rd_data.cntr_value - (SUBTRACT_VAL * (trigger_count_4_1us - mem_b_rd_data.trigger_count));
                    end else begin
                        mem_b_wr_data.cntr_value <= mem_b_rd_data.cntr_value - (SUBTRACT_VAL * (8 + trigger_count_4_1us - mem_b_rd_data.trigger_count));
                    end
                end else begin
                    mem_b_wr_data.cntr_value <= mem_b_rd_data.cntr_value - (SUBTRACT_VAL << 3);
                end
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end
        end
    end

    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            o_q_timed_out_valid   <= 1'b0;
            o_q_timed_out_wr_addr <= {LOG2_NUM_QP{1'b0}};
        end  // when the timer is enabled and the initial value is written back then the timer
        // is said to have expired
        else if (timer_enable & timer_mem_mod_wren & !mem_b_wr_data.final_count & (mem_b_wr_data.cntr_value == (1 << mem_b_wr_data.init_value))) begin
            o_q_timed_out_valid   <= !timer_reload_pending;
            //o_q_timed_out_valid <= 1'b1;
            o_q_timed_out_wr_addr <= timer_mem_rdaddr;
        end else begin
            o_q_timed_out_valid <= 1'b0;
        end
    end

endmodule  // resp_handler_timer
