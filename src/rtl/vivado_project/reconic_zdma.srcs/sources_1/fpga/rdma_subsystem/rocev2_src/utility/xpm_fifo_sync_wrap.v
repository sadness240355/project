////////////////////////////////////////////////////////////////////
// (c) Copyright 2013 - 2019 Xilinx, Inc. All rights reserved.   
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
// xpm_fifo_sync_wrap.v
//-----------------------------------------------------------------------------
//
// Filename        : xpm_fifo_sync_wrap
// Version         : v1.0
// Description     : Interface to CMAC module
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
`timescale 1 ps / 1 ps
module xpm_fifo_sync_wrap #(

    // Common module parameters
    parameter FIFO_MEMORY_TYPE = "auto",
    parameter ECC_MODE         = "no_ecc",

    parameter integer FIFO_WRITE_DEPTH    = 2048,
    parameter integer WRITE_DATA_WIDTH    = 32,
    parameter integer WR_DATA_COUNT_WIDTH = 1,
    parameter integer PROG_FULL_THRESH    = 10,
    parameter integer FULL_RESET_VALUE    = 0,
    parameter         USE_ADV_FEATURES    = "0707",

    parameter         READ_MODE           = "std",
    parameter integer FIFO_READ_LATENCY   = 1,
    parameter integer READ_DATA_WIDTH     = WRITE_DATA_WIDTH,
    parameter integer RD_DATA_COUNT_WIDTH = 1,
    parameter integer PROG_EMPTY_THRESH   = 10,
    parameter         DOUT_RESET_VALUE    = "0",

    parameter WAKEUP_TIME = 0

) (

    // Common module ports
    input wire sleep,
    input wire rst,

    // Write Domain ports
    input  wire                           wr_clk,
    input  wire                           wr_en,
    input  wire [   WRITE_DATA_WIDTH-1:0] din,
    output wire                           full,
    output wire                           prog_full,
    output wire [WR_DATA_COUNT_WIDTH-1:0] wr_data_count,
    output wire                           overflow,
    output wire                           wr_rst_busy,
    output wire                           almost_full,
    output wire                           wr_ack,

    // Read Domain ports
    input  wire                           rd_en,
    output wire [    READ_DATA_WIDTH-1:0] dout,
    output wire                           empty,
    output wire                           prog_empty,
    output wire [RD_DATA_COUNT_WIDTH-1:0] rd_data_count,
    output wire                           underflow,
    output wire                           rd_rst_busy,
    output wire                           almost_empty,
    output wire                           data_valid,

    // ECC Related ports
    input  wire injectsbiterr,
    input  wire injectdbiterr,
    output wire sbiterr,
    output wire dbiterr
);

    xpm_fifo_sync #(
        .FIFO_MEMORY_TYPE   (FIFO_MEMORY_TYPE),     //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           (ECC_MODE),             //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (FIFO_WRITE_DEPTH),     //positive integer									
        .WRITE_DATA_WIDTH   (WRITE_DATA_WIDTH),     //positive integer									
        .WR_DATA_COUNT_WIDTH(WR_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_FULL_THRESH   (PROG_FULL_THRESH),     //positive integer									
        .USE_ADV_FEATURES   (USE_ADV_FEATURES),
        .FULL_RESET_VALUE   (FULL_RESET_VALUE),     //positive integer; 0 or 1									
        .READ_MODE          (READ_MODE),            //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (FIFO_READ_LATENCY),    //positive integer;									
        .READ_DATA_WIDTH    (READ_DATA_WIDTH),      //positive integer									
        .RD_DATA_COUNT_WIDTH(RD_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_EMPTY_THRESH  (PROG_EMPTY_THRESH),    //positive integer									
        .DOUT_RESET_VALUE   (DOUT_RESET_VALUE),     //string									
        .WAKEUP_TIME        (WAKEUP_TIME)           //positive integer; 0 or 2;									            
    ) XPM_FIFO_SYNC (
        .sleep        (sleep),
        .rst          (rst),
        .wr_clk       (wr_clk),
        .wr_en        (wr_en),
        .din          (din),
        .full         (full),
        .prog_full    (prog_full),
        .wr_data_count(wr_data_count),
        .overflow     (overflow),
        .wr_rst_busy  (wr_rst_busy),
        .rd_en        (rd_en),
        .dout         (dout),
        .data_valid   (data_valid),
        .almost_empty (almost_empty),
        .almost_full  (almost_full),
        .wr_ack       (wr_ack),
        .empty        (empty),
        .prog_empty   (prog_empty),
        .rd_data_count(rd_data_count),
        .underflow    (underflow),
        .rd_rst_busy  (rd_rst_busy),
        .injectsbiterr(injectsbiterr),
        .injectdbiterr(injectdbiterr),
        .sbiterr      (sbiterr),
        .dbiterr      (dbiterr)
    );

endmodule
