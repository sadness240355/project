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
// queue.v
//-----------------------------------------------------------------------------
//
// Filename        : q_mgr_queue.v
// Version         : v1.0
// Description     : bram_q_manager top file. This is a generic queue manager
//		     file. This can be instantiated for any types of queues 
//		     like submission queue, completion queue or admin queue
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//--                  -- q_mgr_queue.v
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
// 该模块实现了一个基于指针的环形FIFO管理机制。
// 目前用于管理 SQ FIFO
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module q_mgr_queue #(
    parameter C_Q_DEPTH            = 32,  // 每个队列中的条目数，4
    parameter C_ENTRY_SIZE         = 64,  // 每个条目的大小（字节）。SQ 大小为 64B，512
    parameter C_PTR_WIDTH          = 5,   // 访问队列深度所需的指针位宽，2
    parameter C_THRESHOLD          = 4,   // 产生几乎满载的阈值--用于流量控制，4
    parameter C_ECN_THRESHOLD      = 4,   // ECN阈值----------用于流量控制，4
    parameter C_PUSH_POP_1         = 1,   // 是否只支持单元素推入/弹出，1
    parameter C_ADDR_WIDTH         = 32,  // 系统地址总线宽度，32
    parameter C_FULL_DEPEND_ON_ECN = 0    // full信号是否依赖ECN阈值，0
) (
    // Clock and Reset
    input wire core_clk,  // Escape Mode Transmit Clock  
    input wire core_rst,  // Active high core reset

    // 队列指针和状态
    output wire [C_ADDR_WIDTH -1:0] o_rd_addr,            // 读地址（已考虑基地址）堆叠在一条总线上
    output wire [C_ADDR_WIDTH -1:0] o_wr_addr,            // 写地址堆叠在一条总线上
    output wire [ C_PTR_WIDTH -1:0] o_rd_ptr,             // 读指针
    output wire [ C_PTR_WIDTH -1:0] o_wr_ptr,             // 写指针
    output wire                     o_q_empty,            // 队列空标志
    output wire                     o_q_full,             // 队列满标志
    output wire                     o_q_almost_full,      // 队列几乎满标志
    output wire [  C_PTR_WIDTH : 0] o_num_valid_entries,  // 队列中有效条目的数量
    output wire [  C_PTR_WIDTH : 0] o_num_free_entries,   // 队列中的空闲条目数

    // 配置信号
    input wire [C_ADDR_WIDTH -1:0] i_base_addr,       // 队列基地址，32'b0
    input wire [  C_PTR_WIDTH : 0] i_q_depth,         // 队列深度，WQE_FIFO_DEPTH，4
    input wire                     i_init,            // 初始化信号，1'b0
    input wire                     i_threshold_cntrl, // 阈值控制，1'b0

    // 推入/弹出操作
    input wire                   i_q_push,           // 启用队列的推送功能
    input wire [C_PTR_WIDTH : 0] i_push_num_entries, // 要推送的条目数，3'b1

    input wire                   i_q_pop,           // 队列的弹出启用
    input wire [C_PTR_WIDTH : 0] i_pop_num_entries  // 要弹出的条目数，3'b1

);
    `include "../../rdma_macros.vh"

    reg                    threshold_cntrl;
    reg  [  C_PTR_WIDTH:0] diff_in_entries;
    reg  [C_PTR_WIDTH : 0] rd_ptr_ff;  // Read pointer with roll over bit--带滚动位的读取指针
    wire [C_PTR_WIDTH : 0] rd_ptr_nxt;  // Read pointer with roll over bit--带滚动位的读取指针

    reg  [C_PTR_WIDTH : 0] wr_ptr;  // Write pointer with roll over bit--带滚动位的写入指针

    reg  [C_PTR_WIDTH : 0] num_valid_entries;  // 队列中有效条目的数量
    reg  [C_PTR_WIDTH : 0] num_free_entries;  // 队列中的空闲条目数
    wire                   fifo_full;
    wire                   fifo_almost_full;
    wire                   fifo_empty;
    wire                   valid_pop;
    wire                   valid_push;

    // 地址偏移计算
    /*
    功能: 根据条目大小计算地址偏移位数
        1bit → 偏移0位
        4bit → 偏移2位
        64bit → 偏移6位
        512bit → 偏移9位
    */
    // localparam ADDR_SHIFT = C_ENTRY_SIZE == 1 ? 0 : (C_ENTRY_SIZE == 4 ? 2 : (C_ENTRY_SIZE == 8 ? 3 : (C_ENTRY_SIZE == 16 ? 4 : (C_ENTRY_SIZE == 32 ? 5 : (C_ENTRY_SIZE == 64 ? 6 : (C_ENTRY_SIZE == 128 ? 7 : 8))))));  // 8
    localparam ADDR_SHIFT = C_ENTRY_SIZE == 1 ? 0 : (C_ENTRY_SIZE == 4 ? 2 : (C_ENTRY_SIZE == 8 ? 3 : (C_ENTRY_SIZE == 16 ? 4 : (C_ENTRY_SIZE == 32 ? 5 : (C_ENTRY_SIZE == 64 ? 6 : (C_ENTRY_SIZE == 128 ? 7 : (C_ENTRY_SIZE == 256 ? 8 : 9)))))));  // 9

    // 读指针管理(环形指针)
    assign rd_ptr_nxt[C_PTR_WIDTH-1:0] = valid_pop ? ((rd_ptr_ff[C_PTR_WIDTH-1:0] + i_pop_num_entries < i_q_depth) ? rd_ptr_ff[C_PTR_WIDTH-1:0] + i_pop_num_entries : (rd_ptr_ff[C_PTR_WIDTH-1:0] + i_pop_num_entries - i_q_depth)) : rd_ptr_ff[C_PTR_WIDTH-1:0];
    assign rd_ptr_nxt[C_PTR_WIDTH]     = valid_pop ? ((rd_ptr_ff[C_PTR_WIDTH-1:0] + i_pop_num_entries < i_q_depth) ? rd_ptr_ff[C_PTR_WIDTH] : ~rd_ptr_ff[C_PTR_WIDTH]) : rd_ptr_ff[C_PTR_WIDTH];

    `MSFF_R(rd_ptr_ff, rd_ptr_nxt, core_clk, core_rst)  // rd_ptr_ff <= rd_ptr_nxt

    // 队列有效性检查
    // 弹出时检查队列非空且请求数量不超过有效条目数
    assign valid_pop  = i_q_pop && !fifo_empty && i_pop_num_entries <= num_valid_entries;
    // 推入时检查队列非满且请求数量不超过空闲条目数
    assign valid_push = i_q_push && !fifo_full && i_push_num_entries <= num_free_entries;

    // 写指针管理
    generate
        if (~C_PUSH_POP_1) begin  // 多元素推入/弹出
            always @(posedge core_clk) begin
                if (core_rst) begin
                    wr_ptr <= 'b0;
                end else begin
                    if (valid_push) begin
                        if (wr_ptr[C_PTR_WIDTH-1:0] + i_push_num_entries < i_q_depth) begin
                            wr_ptr[C_PTR_WIDTH-1 : 0] <= wr_ptr[C_PTR_WIDTH-1:0] + i_push_num_entries;
                        end else begin  // roll over after i_q_depth
                            wr_ptr[C_PTR_WIDTH-1 : 0] <= wr_ptr[C_PTR_WIDTH-1:0] + i_push_num_entries - i_q_depth;
                            wr_ptr[C_PTR_WIDTH]       <= ~wr_ptr[C_PTR_WIDTH];
                        end
                    end
                end
            end
        end else begin  // current：单元素推入/弹出 - 每次只操作1个元素
            always @(posedge core_clk) begin
                if (core_rst) begin
                    wr_ptr <= 'b0;
                end else begin
                    if (valid_push) begin
                        if (wr_ptr[C_PTR_WIDTH-1:0] + 1'b1 < i_q_depth) begin
                            wr_ptr[C_PTR_WIDTH-1 : 0] <= wr_ptr[C_PTR_WIDTH-1:0] + 1'b1;
                        end else begin  // 回绕到队列开始
                            wr_ptr[C_PTR_WIDTH-1 : 0] <= 'b0;
                            wr_ptr[C_PTR_WIDTH]       <= ~wr_ptr[C_PTR_WIDTH];
                        end
                    end
                end
            end
        end
    endgenerate

    // 条目数量管理
    generate
        if (~C_PUSH_POP_1) begin  // 多元素操作
            always @(posedge core_clk) begin
                if (core_rst) begin
                    num_valid_entries <= 'b0;
                    num_free_entries  <= C_Q_DEPTH;
                end else begin
                    if (i_init) begin
                        num_free_entries  <= i_q_depth;
                        num_valid_entries <= 'b0;
                    end else begin
                        case ({
                            valid_pop, valid_push
                        })
                            2'b01: begin  // 仅推入
                                num_valid_entries <= num_valid_entries + i_push_num_entries;
                                num_free_entries  <= num_free_entries - i_push_num_entries;
                            end
                            2'b10: begin  // 仅弹出
                                num_valid_entries <= num_valid_entries - i_pop_num_entries;
                                num_free_entries  <= num_free_entries + i_pop_num_entries;
                            end
                            2'b11: begin  // 同时推入和弹出
                                num_valid_entries <= num_valid_entries - i_pop_num_entries + i_push_num_entries;
                                num_free_entries  <= num_free_entries + i_pop_num_entries - i_push_num_entries;
                            end
                            2'b00: begin  // 无操作
                                num_valid_entries <= num_valid_entries;
                                num_free_entries  <= num_free_entries;
                            end
                        endcase
                    end

                end
            end
        end else begin  // current：单元素操作-每次只操作1个元素
            always @(posedge core_clk) begin
                if (core_rst) begin
                    num_valid_entries <= 'b0;
                    num_free_entries  <= C_Q_DEPTH;
                end else begin
                    if (i_init) begin
                        num_free_entries  <= i_q_depth;
                        num_valid_entries <= 'b0;
                    end else begin
                        case ({
                            valid_pop, valid_push
                        })
                            2'b01: begin
                                num_valid_entries <= num_valid_entries + 1'b1;
                                num_free_entries  <= num_free_entries - 1'b1;
                            end
                            2'b10: begin
                                num_valid_entries <= num_valid_entries - 1'b1;
                                num_free_entries  <= num_free_entries + 1'b1;
                            end
                            2'b11, 2'b00: begin
                                num_valid_entries <= num_valid_entries;
                                num_free_entries  <= num_free_entries;
                            end
                        endcase
                    end

                end
            end
        end
    endgenerate

    // 空标志：读写指针完全相等（包括溢出位）
    assign fifo_empty       = (wr_ptr == rd_ptr_ff) ? 1'b1 : 1'b0;

    // 满标志：溢出位不同但指针位置相同
    assign fifo_full        = (wr_ptr[C_PTR_WIDTH] != rd_ptr_ff[C_PTR_WIDTH]) && (wr_ptr[C_PTR_WIDTH-1:0] == rd_ptr_ff[C_PTR_WIDTH-1:0]);

    // 当深度 -1 的位置已被写入时，会生成 fifo almost full。
    // 这主要用于不允许写入所有位置的 NVMe SQ 和 CQ
    // 根据控制信号选择不同阈值
    assign fifo_almost_full = threshold_cntrl ? (diff_in_entries <= C_ECN_THRESHOLD) : (diff_in_entries <= C_THRESHOLD);

    ////////////////////////////////For timing fix in LV devices/////////////////////
    // 时序优化：将阈值控制和差值计算寄存
    always @(posedge core_clk) begin
        if (core_rst) threshold_cntrl <= 1'b0;
        else threshold_cntrl <= i_threshold_cntrl;
    end

    always @(posedge core_clk) begin
        if (core_rst) diff_in_entries <= {(C_PTR_WIDTH + 1) {1'b0}};
        else diff_in_entries <= i_q_depth - num_valid_entries;
    end
    ///////////////////////////////////////////////////////////////////////////

    // 地址生成: 基地址 + (指针位置 × 条目大小)
    assign o_wr_addr = i_base_addr + (wr_ptr[C_PTR_WIDTH-1:0] << ADDR_SHIFT);
    assign o_rd_addr = i_base_addr + (rd_ptr_ff[C_PTR_WIDTH-1:0] << ADDR_SHIFT);
    assign o_wr_ptr  = wr_ptr[C_PTR_WIDTH-1:0];
    assign o_rd_ptr  = rd_ptr_ff[C_PTR_WIDTH-1:0];
    generate
        if (C_FULL_DEPEND_ON_ECN == 1) begin
            assign o_q_full = threshold_cntrl ? fifo_almost_full : fifo_full;
        end else begin
            assign o_q_full = fifo_full;
        end
    endgenerate
    assign o_q_almost_full     = fifo_almost_full;
    assign o_q_empty           = fifo_empty;
    assign o_num_valid_entries = num_valid_entries;
    assign o_num_free_entries  = num_free_entries;


endmodule
