//==========================================================================
// Change history
//==========================================================================
// 16-Jul-2020 : first version with basic verification
// 20-Jul-2020 : not making read data go to 0, if the timer is reset at that
//               point. only ensuring that QP timeout doesn't get set when
//               reset timer is received and not having any effect on the
//               timer being reloaded
// 21-Jul-2020 : added one more bit for terminal count indicator. when terminal
//               count is set and counter bits are 0 only there should be a
//               timeout. previously a value of 1 would generate timeout for
//               every 4.1us instead of for 8.2us. fixed that now
// 10-Aug-2020 : added another bit
// 19-Aug-2020 : added logic to reload the counter with initial value after
//               it times out
// 20-Aug-2020 : when there's a read-modify-write and a new reload happens
//               to that queue being processed, then the queue shouldn't be
//               read out or modified or written back
// 22-Aug-2020 : fixed qpid_proc_done logic so that it's not reset in the wrong
//               time


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
// Filename        : resp_handler_psn.sv
// Version         : v1.0
// Description     : 处理来自 RX pkt 处理程序的响应数据包的模块
// Verilog-Standard: System Verilog
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- resp_handler_top.v
//                     -- resp_handler_psn.sv
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
module resp_handler_psn #(
    parameter DATA_WIDTH      = 24,
    parameter NUM_WR_REQS     = 3,
    parameter WR_FIFO_EN      = 1,
    parameter RD_FIFO_EN      = 1,
    parameter NUM_RD_REQS     = 3,
    parameter C_NUM_QP        = 256,
    parameter C_QP_INDX_WIDTH = $clog2(C_NUM_QP)
) (
    // clocks and resets
    input core_clk,
    input core_rstn,

    // 写请求 FIFO接口
    input        [    NUM_WR_REQS - 1:0] wr_reqs,
    input        [C_QP_INDX_WIDTH - 1:0] wr_req_qpid[NUM_WR_REQS],
    input        [     DATA_WIDTH - 1:0] wr_req_data[NUM_WR_REQS],
    output logic [    NUM_WR_REQS - 1:0] wr_req_done,

    // 读请求 FIFO接口
    input        [    NUM_RD_REQS - 1:0] rd_reqs,
    input        [C_QP_INDX_WIDTH - 1:0] rd_req_qpid[NUM_RD_REQS],
    output logic [     DATA_WIDTH - 1:0] rd_req_data[NUM_RD_REQS],
    output logic [    NUM_RD_REQS - 1:0] rd_req_done
);

    `include "../../rdma_macros.vh"

    //==========================================================================
    // local parameters
    //==========================================================================
    localparam LOG2_NUM_WR_REQS = $clog2(NUM_WR_REQS);
    localparam LOG2_NUM_RD_REQS = $clog2(NUM_RD_REQS);
    localparam WR_FIFO_WIDTH = DATA_WIDTH + C_QP_INDX_WIDTH;
    localparam RD_FIFO_WIDTH = C_QP_INDX_WIDTH;

    //==========================================================================
    // wires and registers
    //==========================================================================
    logic [     NUM_WR_REQS - 1:0] wrreq_fifo_empty;
    logic [     NUM_WR_REQS - 1:0] wrreq_fifo_rden;
    logic [      DATA_WIDTH - 1:0] wrreq_fifo_dout_data[NUM_WR_REQS];
    logic [ C_QP_INDX_WIDTH - 1:0] wrreq_fifo_dout_qpid[NUM_WR_REQS];
    logic [     NUM_RD_REQS - 1:0] rdreq_fifo_empty;
    logic [     NUM_RD_REQS - 1:0] rdreq_fifo_rden;
    logic [ C_QP_INDX_WIDTH - 1:0] rdreq_fifo_dout_qpid[NUM_RD_REQS];
    logic [     NUM_WR_REQS - 1:0] wr_req_arb_reqs;
    logic [     NUM_WR_REQS - 1:0] wr_req_arb_gnt;
    logic                          wr_req_arb_done;
    logic [LOG2_NUM_WR_REQS - 1:0] wr_req_num;
    logic [     NUM_RD_REQS - 1:0] rd_req_arb_reqs;
    logic [     NUM_RD_REQS - 1:0] rd_req_arb_gnt;
    logic [     NUM_RD_REQS - 1:0] rd_req_arb_gnt_q;
    logic [     NUM_RD_REQS - 1:0] rd_req_arb_gnt_2q;
    logic                          rd_req_arb_done;
    logic [LOG2_NUM_RD_REQS - 1:0] rd_req_num;
    logic                          mem_wren_nxt;
    logic [ C_QP_INDX_WIDTH - 1:0] mem_wraddr_nxt;
    logic [      DATA_WIDTH - 1:0] mem_wrdata_nxt;
    logic                          mem_wren;
    logic [ C_QP_INDX_WIDTH - 1:0] mem_wraddr;
    logic [      DATA_WIDTH - 1:0] mem_wrdata;
    logic                          mem_rden_nxt;
    logic [ C_QP_INDX_WIDTH - 1:0] mem_rdaddr_nxt;
    logic [      DATA_WIDTH - 1:0] mem_rddata;
    logic                          mem_rden;
    logic [ C_QP_INDX_WIDTH - 1:0] mem_rdaddr;
    logic                          mem_rddata_valid;

    /***************************************************************************
   *                                                                          *
   *                                                                          *
   *                  Write arbitration and memory write logic                *
   *                                                                          *
   *                                                                          *
   ***************************************************************************/

    generate
        if (NUM_WR_REQS > 1) begin : wr_arb_inst
            if (WR_FIFO_EN == 1) begin : inputs_in_fifo
                for (genvar i = 0; i < NUM_WR_REQS; i++) begin : wrreq_fifo_insts
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
                        .READ_DATA_WIDTH    (WR_FIFO_WIDTH),
                        .READ_MODE          ("fwft"),
                        .SIM_ASSERT_CHK     (0),
                        .USE_ADV_FEATURES   ("0707"),
                        .WAKEUP_TIME        (0),
                        .WRITE_DATA_WIDTH   (WR_FIFO_WIDTH),
                        .WR_DATA_COUNT_WIDTH(1)
                    ) u_in_wrreq_fifo (
                        .wr_clk       (core_clk),
                        .rst          (!core_rstn),
                        .wr_en        (wr_reqs[i]),
                        .din          ({wr_req_data[i], wr_req_qpid[i]}),
                        .rd_en        (wrreq_fifo_rden[i]),
                        .empty        (wrreq_fifo_empty[i]),
                        .dout         ({wrreq_fifo_dout_data[i], wrreq_fifo_dout_qpid[i]}),
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

                    assign wr_req_arb_reqs[i] = !wrreq_fifo_empty[i];
                    assign wrreq_fifo_rden[i] = !wrreq_fifo_empty[i] & wr_req_arb_gnt[i];

                end
            end // inputs_in_fifo
	    else
	    begin : inputs_direct
                assign wr_req_arb_reqs = wr_reqs;
                for (genvar i = 0; i < NUM_WR_REQS; i++) begin : direct_data_qpid
                    assign wrreq_fifo_dout_qpid[i] = wr_req_qpid[i];
                    assign wrreq_fifo_dout_data[i] = wr_req_data[i];
                end
            end  // inputs_direct

            // arbiter instance to arbitrate between various requests
            arbiter #(
                .NUM_REQS(NUM_WR_REQS),
                .BUS_HOLD(0),
                .NEW_ARB (1),
                .SPLIT   (0)
            ) u_wr_req_arb (
                .clk_arb  (core_clk),
                .rst_n_arb(core_rstn),
                .reqs     (wr_req_arb_reqs),
                .arb_done (wr_req_arb_done),
                .gnt      (wr_req_arb_gnt),
                .gnt_valid()
            );

            //==========================================================================
            // function to identify the qp number to update the BRAM
            //==========================================================================
            function [LOG2_NUM_WR_REQS - 1:0] wr_gnt_num_idfy(input [NUM_WR_REQS - 1:0] grant_bit);
                begin : wr_gnt_num_idfy_blk
                    for (int i = 0; i < NUM_WR_REQS; i++) begin : calc_block
                        if (grant_bit[i]) begin
                            wr_gnt_num_idfy = i;
                            disable wr_gnt_num_idfy_blk;
                        end
                    end
                end
            endfunction  // wr_gnt_num_idfy

            // identify the address and data to be written based on grant
            assign wr_req_num     = wr_gnt_num_idfy(wr_req_arb_gnt);
            assign mem_wraddr_nxt = wrreq_fifo_dout_qpid[wr_req_num];
            assign mem_wrdata_nxt = wrreq_fifo_dout_data[wr_req_num];
            assign mem_wren_nxt   = |wr_req_arb_gnt;

            // flops to ensure timing clean design
            `MSFF_RN(mem_wraddr, mem_wraddr_nxt, core_clk, core_rstn)
            `MSFF_RN(mem_wrdata, mem_wrdata_nxt, core_clk, core_rstn)
            `MSFF_RN(mem_wren, mem_wren_nxt, core_clk, core_rstn)

            // writes can arbitrate every cycle
            assign wr_req_arb_done = 1'b1;
            assign wr_req_done     = wr_req_arb_gnt;
        end else begin : no_wr_arb
            assign mem_wraddr     = wr_req_qpid[0];
            assign mem_wrdata     = wr_req_data[0];
            assign mem_wren       = wr_reqs[0];
            assign wr_req_done[0] = mem_wren;
        end
    endgenerate

    /***************************************************************************
   *                                                                          *
   *                                                                          *
   *                  Read arbitration and memory read logic                  *
   *                                                                          *
   *                                                                          *
   ***************************************************************************/

    generate
        if (NUM_RD_REQS > 1) begin : rd_arb_inst
            if (RD_FIFO_EN == 1) begin : rd_inputs_in_fifo
                for (genvar i = 0; i < NUM_RD_REQS; i++) begin : wrreq_fifo_insts
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
                        .READ_DATA_WIDTH    (RD_FIFO_WIDTH),
                        .READ_MODE          ("fwft"),
                        .SIM_ASSERT_CHK     (0),
                        .USE_ADV_FEATURES   ("0707"),
                        .WAKEUP_TIME        (0),
                        .WRITE_DATA_WIDTH   (RD_FIFO_WIDTH),
                        .WR_DATA_COUNT_WIDTH(1)
                    ) u_in_rdreq_fifo (
                        .wr_clk       (core_clk),
                        .rst          (!core_rstn),
                        .wr_en        (rd_reqs[i]),
                        .din          (rd_req_qpid[i]),
                        .rd_en        (rdreq_fifo_rden[i]),
                        .empty        (rdreq_fifo_empty[i]),
                        .dout         (rdreq_fifo_dout_qpid[i]),
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

                    assign rd_req_arb_reqs[i] = !rdreq_fifo_empty[i];
                    assign rdreq_fifo_rden[i] = !rdreq_fifo_empty[i] & rd_req_arb_gnt[i] & rd_req_arb_done;

                end
            end // inputs_in_fifo
	   else
	     begin : rd_inputs_direct
                assign rd_req_arb_reqs = wr_reqs;
                for (genvar i = 0; i < NUM_WR_REQS; i++) begin : rd_direct_data_qpid
                    assign rdreq_fifo_dout_qpid[i] = rd_req_qpid[i];
                end
            end  // rd_inputs_direct

            // arbiter instance to arbitrate between various requests
            arbiter #(
                .NUM_REQS(NUM_RD_REQS),
                .BUS_HOLD(0),
                .NEW_ARB (1),
                .SPLIT   (0)
            ) u_rd_req_arb (
                .clk_arb  (core_clk),
                .rst_n_arb(core_rstn),
                .reqs     (rd_req_arb_reqs),
                .arb_done (rd_req_arb_done),
                .gnt      (rd_req_arb_gnt),
                .gnt_valid()
            );
            //==========================================================================
            // function to identify the qp number to update the BRAM
            //==========================================================================
            function [LOG2_NUM_RD_REQS - 1:0] rd_gnt_num_idfy(input [NUM_RD_REQS - 1:0] grant_bit);
                begin : rd_gnt_num_idfy_blk
                    for (int i = 0; i < NUM_RD_REQS; i++) begin : calc_block
                        if (grant_bit[i]) begin
                            rd_gnt_num_idfy = i;
                            disable rd_gnt_num_idfy_blk;
                        end
                    end
                end
            endfunction  // rd_gnt_num_idfy

            // identify the address and data to be read out based on grant
            assign rd_req_num     = rd_gnt_num_idfy(rd_req_arb_gnt);
            assign mem_rdaddr_nxt = rd_req_qpid[rd_req_num];
            assign mem_rden_nxt   = (mem_wren_nxt & (mem_wraddr_nxt == mem_rdaddr_nxt)) ? 1'b0 : |rd_req_arb_gnt;

            // flops to ensure timing clean design
            `MSFF_RN(mem_rdaddr, mem_rdaddr_nxt, core_clk, core_rstn)
            `MSFF_RN(mem_rden, mem_rden_nxt, core_clk, core_rstn)
            `MSFF_RN(mem_rddata_valid, mem_rden, core_clk, core_rstn)

            assign rd_req_arb_done = !(mem_wren_nxt & (mem_wraddr_nxt == mem_rdaddr_nxt));
            //`MSFF_RN (rd_req_done, ({NUM_RD_REQS {mem_rddata_valid}} & rd_req_arb_gnt_q), core_clk, core_rstn);
            assign rd_req_done     = ({NUM_RD_REQS{mem_rddata_valid}} & rd_req_arb_gnt_2q);

            // delay flops
            `MSFF_RN(rd_req_arb_gnt_q, rd_req_arb_gnt, core_clk, core_rstn)
            `MSFF_RN(rd_req_arb_gnt_2q, rd_req_arb_gnt_q, core_clk, core_rstn)

            for (genvar i = 0; i < NUM_RD_REQS; i++) begin : rd_data_gen
                // based on what request was granted, data is put on that and all others are
                // driven 0s
                assign rd_req_data[i] = rd_req_arb_gnt_2q[i] ? mem_rddata : {DATA_WIDTH{1'b0}};
            end  // rd_data_gen
        end else begin : no_rd_arb
            assign mem_rdaddr = rd_req_qpid[0];
            assign mem_rden   = (mem_wren & (mem_wraddr == mem_rdaddr)) ? 1'b0 : rd_reqs[0];
            `MSFF_RN(rd_req_done[0], mem_rden, core_clk, core_rstn)
            assign rd_req_data[0] = mem_rddata;
        end
    endgenerate

    /***************************************************************************
   *                                                                          *
   *                                                                          *
   *                  Memory instantiation to store data                      *
   *                                                                          *
   *                                                                          *
   ***************************************************************************/

    xpm_memory_sdpram #(  // "A" side parameters
        .ADDR_WIDTH_A           (C_QP_INDX_WIDTH),
        .RST_MODE_A             ("SYNC"),
        .BYTE_WRITE_WIDTH_A     (DATA_WIDTH),
        .WRITE_DATA_WIDTH_A     (DATA_WIDTH),
        // "B" side parameters
        .ADDR_WIDTH_B           (C_QP_INDX_WIDTH),
        .RST_MODE_B             ("SYNC"),
        .READ_RESET_VALUE_B     ("0"),
        .READ_LATENCY_B         (1),
        .READ_DATA_WIDTH_B      (DATA_WIDTH),
        .WRITE_MODE_B           ("read_first"),
        // Common parameters
        .CLOCKING_MODE          ("common_clock"),
        .CASCADE_HEIGHT         (0),
        .ECC_MODE               ("no_ecc"),
        .MEMORY_INIT_FILE       ("none"),
        .MEMORY_INIT_PARAM      ("0"),
        .MEMORY_OPTIMIZATION    ("true"),
        .MEMORY_PRIMITIVE       ("auto"),
        .MEMORY_SIZE            (C_NUM_QP * DATA_WIDTH),
        .MESSAGE_CONTROL        (0),
        .AUTO_SLEEP_TIME        (0),
        .SIM_ASSERT_CHK         (0),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT           (1),
        .WAKEUP_TIME            ("disable_sleep")
    ) u_data_memory (
        .clka          (core_clk),
        .ena           (mem_wren),
        .wea           (mem_wren),
        .addra         (mem_wraddr),
        .dina          (mem_wrdata),
        .injectdbiterra(1'b0),
        .injectsbiterra(1'b0),
        .clkb          (core_clk),
        .rstb          (!core_rstn),
        .enb           (mem_rden),
        .addrb         (mem_rdaddr),
        .regceb        (1'b1),
        .doutb         (mem_rddata),
        .sleep         (1'b0),
        .dbiterrb      (),
        .sbiterrb      ()
    );

endmodule  // resp_handler_psn
