////////////////////////////////////////////////////////////////////
// (c) Copyright 2013 - 2015 Xilinx, Inc. All rights reserved.   
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
// reg_fifo_sync.v
//-----------------------------------------------------------------------------
//
// Filename        : reg_fifo_sync.v
// Version         : v1.0
// Description     : Synchronous FIFO implemented with registers only：仅用寄存器实现同步 FIFO
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

module reg_fifo_sync #(
    parameter [0:0] C_READ_MODE = 1'b0,
    parameter C_FIFO_DEPTH = 8,
    parameter C_FIFO_WRCNTR_WIDTH = 4,
    parameter C_FIFO_DATA_WIDTH = 32
) (
    input  wire                         clk,
    input  wire                         reset,
    input  wire                         wr_en,
    input  wire [C_FIFO_DATA_WIDTH-1:0] din,
    output wire                         full,
    output wire                         almost_full,
    input  wire                         rd_en,
    output wire [C_FIFO_DATA_WIDTH-1:0] dout,
    output wire                         empty
);
    reg  [C_FIFO_WRCNTR_WIDTH-1:0] i;

    reg  [  C_FIFO_DATA_WIDTH-1:0] fifo_reg  [C_FIFO_DEPTH-1:0];
    reg  [C_FIFO_WRCNTR_WIDTH-1:0] wrptr;
    reg  [C_FIFO_WRCNTR_WIDTH-1:0] rdptr;

    wire [  C_FIFO_DATA_WIDTH-1:0] dout_i;
    reg  [  C_FIFO_DATA_WIDTH-1:0] dout_r;

    wire [C_FIFO_WRCNTR_WIDTH-1:0] nxt_wrptr;

    assign nxt_wrptr = wrptr + 'd1;

    always @(posedge clk)
        if (reset) wrptr <= {(C_FIFO_WRCNTR_WIDTH) {1'b0}};
        else if (wr_en) wrptr <= nxt_wrptr;

    always @(posedge clk)
        if (reset) rdptr <= {(C_FIFO_WRCNTR_WIDTH) {1'b0}};
        else if (rd_en) rdptr <= rdptr + 'd1;

    always @(posedge clk)
        for (i = 0; i < C_FIFO_DEPTH; i = i + 1) begin
            if (reset) fifo_reg[i] <= {(C_FIFO_DATA_WIDTH) {1'b0}};
            else if ((i[C_FIFO_WRCNTR_WIDTH-2:0] == wrptr[C_FIFO_WRCNTR_WIDTH-2:0]) && wr_en) fifo_reg[i] <= din;
        end

    assign dout_i = fifo_reg[rdptr[C_FIFO_WRCNTR_WIDTH-2:0]];

    always @(posedge clk)
        if (reset) dout_r <= {(C_FIFO_DATA_WIDTH) {1'b0}};
        else if (rd_en) dout_r <= dout_i;

    assign dout        = C_READ_MODE ? dout_i : dout_r;

    assign full        = ({~wrptr[C_FIFO_WRCNTR_WIDTH-1], wrptr[C_FIFO_WRCNTR_WIDTH-2:0]} == rdptr) ? 1'b1 : 1'b0;
    assign almost_full = ({~nxt_wrptr[C_FIFO_WRCNTR_WIDTH-1], nxt_wrptr[C_FIFO_WRCNTR_WIDTH-2:0]} == rdptr) ? 1'b1 : 1'b0;
    assign empty       = (wrptr == rdptr) ? 1'b1 : 1'b0;

endmodule
