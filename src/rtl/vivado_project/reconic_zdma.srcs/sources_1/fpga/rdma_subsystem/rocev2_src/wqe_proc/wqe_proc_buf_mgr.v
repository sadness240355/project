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
// wqe_proc_buf_mgr.v
//-----------------------------------------------------------------------------
//
// Filename        : wqe_proc_buf_mgr.v
// Version         : v1.0
// Description     : WQE 处理器缓冲区管理模块实现将 128B 包头写入包头模块并生成 SGL 的逻辑。
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
// 接收上游有效头数据并写入 HDR BRAM，生成并写入对应 SGL 描述，维护 head/tail 指针、判断满/空风险，
// 给上游返回完成握手 o_buf_mngr_rdy，提供下游读口（hdr_bram_rd_*、sgl_bram_rd_*）和 pop 推进
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module wqe_proc_buf_mgr #(
    parameter C_MAX_QP = 8,
    parameter C_MAX_QID_WIDTH = 3,
    parameter C_MAX_HDR_DEPTH = 128,
    parameter C_EN_DEBUG = 0,
    parameter NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN = 0,  // 1
    parameter C_MAX_SGL_DEPTH = 32
) (
    input core_clk,
    input core_rst,

    // 与 hdr_gen 的接口
    output reg          o_buf_mngr_rdy,  // 本次写入ram流程完成，常规数据包路径下通常是 hdr+sgl 都完成后拉高；特殊分支下不一定两者都写
    input      [1023:0] i_hdr_data,      // 头
    input      [   7:0] i_hdr_len,       // 头长度信息
    input               i_hdr_valid,     // 头有效

    input i_reg_ip_ver,

    output     [ 19:0] buf_mgr_ptr,
    //SGL Buffer signals
    output reg [ 15:0] sgl_head_ptr,
    input      [ 14:0] sgl_bram_rd_addr,
    input              sgl_bram_rd_en,
    output     [ 82:0] sgl_bram_rd_data,
    //Header buffer signals
    input      [ 14:0] hdr_bram_rd_addr,
    input              hdr_bram_rd_en,
    output     [511:0] hdr_bram_rd_data,
    input              hdr_bram_pop,

    input             i_debug_cnt_en,
    input             i_debug_cnt_clr,
    output reg [15:0] o_wqe_proc_hdr_sgl_buf_full_cnt,
    output reg [15:0] o_wqe_proc_maxis_bk_pressure,
    input             i_m_axis_bkp_pressure
);
    `include "../../rdma_macros.vh"

    localparam SGL_BRAM_ADDR_WIDTH = clog2(C_MAX_SGL_DEPTH);
    localparam HDR_BRAM_ADDR_WIDTH = clog2(C_MAX_HDR_DEPTH);
    localparam IDLE = 3'b001;
    localparam EN_DMA = 3'b000;
    localparam WR_HDR = 3'b010;
    localparam WR_DMA_TAIL = 3'b011;
    localparam WAIT_ON_AXI = 3'b100;
    localparam RST_DMA = 3'b101;

    reg  [                    2:0] buf_mgr_cs;
    reg  [                   15:0] hdr_head_ptr;
    reg  [                   15:0] hdr_tail_ptr;
    reg  [                   15:0] sgl_head_ptr_prev;
    reg  [                   15:0] sgl_tail_ptr;
    reg  [                   82:0] sgl_bram_wr_data;
    wire [SGL_BRAM_ADDR_WIDTH-1:0] sgl_bram_wr_addr;
    reg                            sgl_bram_wr_en;
    reg                            sgl_bram_rd_en_r;
    reg                            hdr_bram_rd_en_r;
    reg                            sgl_wr_done;
    reg                            sgl_wr_wait;  // 写两条SGL时的节拍对齐/间隔控制位
    reg  [                  511:0] hdr_bram_wr_data;
    reg  [  HDR_BRAM_ADDR_WIDTH:0] hdr_bram_wr_addr;
    reg                            hdr_bram_wr_en;
    reg                            sgl_buf_full;
    reg                            dma_error;
    reg                            hdr_buf_full;
    reg  [SGL_BRAM_ADDR_WIDTH-1:0] sgl_wr_cnt;
    reg                            curr_desc_done;
    reg                            dma_en_done;
    reg                            dma_cr_done;
    reg                            m_axis_bkp_pressure;

    //////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Debug counters logic //////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    generate
        if (C_EN_DEBUG == 1) begin : DEBUG_EN
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_wqe_proc_hdr_sgl_buf_full_cnt <= 16'h0000;
                    m_axis_bkp_pressure             <= 1'b0;
                    o_wqe_proc_maxis_bk_pressure    <= 16'h0000;
                end else begin
                    if (i_m_axis_bkp_pressure) begin
                        m_axis_bkp_pressure <= 1'b1;
                    end
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_hdr_sgl_buf_full_cnt <= 16'h0000;
                    end else if (i_debug_cnt_en && (sgl_buf_full || hdr_buf_full)) begin
                        o_wqe_proc_hdr_sgl_buf_full_cnt <= o_wqe_proc_hdr_sgl_buf_full_cnt + 1'b1;
                    end
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_maxis_bk_pressure <= 16'h0000;
                    end else if (i_debug_cnt_en && i_m_axis_bkp_pressure) begin
                        o_wqe_proc_maxis_bk_pressure <= o_wqe_proc_maxis_bk_pressure + 1'b1;
                    end
                end
            end
        end else begin
            always @(posedge core_clk) begin
                o_wqe_proc_hdr_sgl_buf_full_cnt <= 16'h0000;
                m_axis_bkp_pressure             <= 1'b0;
                o_wqe_proc_maxis_bk_pressure    <= 16'h0000;
            end
        end
    endgenerate


    assign sgl_bram_wr_addr = sgl_head_ptr[SGL_BRAM_ADDR_WIDTH-1:0];
    assign buf_mgr_ptr      = {sgl_head_ptr[7:0], sgl_tail_ptr[7:0], buf_mgr_cs, m_axis_bkp_pressure};

    generate
        begin
            if (NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN) begin  // current: NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 1

                always @(posedge core_clk) begin
                    if (core_rst) begin
                        o_buf_mngr_rdy   <= 1'b0;
                        hdr_head_ptr     <= 16'h0000;
                        sgl_bram_wr_en   <= 1'b0;
                        sgl_bram_wr_data <= 83'd000;
                        sgl_wr_done      <= 1'b1;
                        sgl_wr_wait      <= 1'b0;
                        buf_mgr_cs       <= IDLE;
                        curr_desc_done   <= 1'b0;
                        dma_en_done      <= 1'b0;
                        dma_cr_done      <= 1'b0;
                        hdr_bram_wr_addr <= {HDR_BRAM_ADDR_WIDTH + 1{1'b0}};
                    end else begin
                        case (buf_mgr_cs)

                            IDLE: begin
                                o_buf_mngr_rdy <= 1'b0;
                                // q_id[14:0] != 15'h0001, 正常路径处理
                                if (~sgl_buf_full && ~hdr_buf_full && (i_hdr_valid && ~o_buf_mngr_rdy) && (i_hdr_data[943:928] != 16'h0001)) begin
                                    sgl_bram_wr_data[82]   <= 1'b0;
                                    hdr_bram_wr_en         <= 1'b1;
                                    hdr_bram_wr_addr       <= {hdr_head_ptr[HDR_BRAM_ADDR_WIDTH-1:0], 1'b0};
                                    hdr_bram_wr_data       <= i_hdr_data[511:0];
                                    buf_mgr_cs             <= WR_HDR;
                                    sgl_bram_wr_en         <= 1'b1;
                                    sgl_bram_wr_data[79:0] <= {8'h00, i_hdr_len, 32'd0, (32'h00000000 + (hdr_head_ptr[14:0] * 16'h80))};
                                    sgl_bram_wr_data[80]   <= 1'b1;
                                    // 8'h0C : Read request     8'h04: SEND_only with data
                                    //  Send Last with immediate
                                    // i_hdr_data[943] 为 hdr_gen completion_data_present
                                    if (i_reg_ip_ver) begin  // IPv6
                                        if (i_hdr_data[503:496] == 8'h0C || ((i_hdr_data[503:497] == 7'h02 && i_hdr_data[943]) || (i_hdr_data[503:496] == 8'b10111 && i_hdr_data[943])) || i_hdr_data[503:496] == 8'h11 || ~|i_hdr_data[1023:1008]) begin
                                            sgl_bram_wr_data[81] <= 1'b1;
                                            sgl_wr_done          <= 1'b1;
                                        end else begin
                                            sgl_bram_wr_data[81] <= 1'b0;
                                            sgl_wr_done          <= 1'b0;
                                            sgl_wr_wait          <= 1'b1;
                                        end
                                    end else begin  // IPv4
                                        // i_hdr_data[343:336] 是 WQE OPCODE字段
                                        // i_hdr_data[943] 为 hdr_gen completion_data_present
                                        if (i_hdr_data[343:336] == 8'h0C || ((i_hdr_data[343:337] == 7'h02 && i_hdr_data[943]) || (i_hdr_data[343:336] == 8'b10111 && i_hdr_data[943])) || i_hdr_data[343:336] == 8'h11 || ~|i_hdr_data[1023:1008]) begin
                                            sgl_bram_wr_data[81] <= 1'b1;
                                            sgl_wr_done          <= 1'b1;
                                        end else begin
                                            sgl_bram_wr_data[81] <= 1'b0;
                                            sgl_wr_done          <= 1'b0;
                                            sgl_wr_wait          <= 1'b1;
                                        end
                                    end
                                end //

                                // q_id[14:0] == 15'h0001(说明是从 SQ1 取出 WQE 执行的), 特殊处理
                                else if (~sgl_buf_full && ~hdr_buf_full && (i_hdr_valid && ~o_buf_mngr_rdy) && (i_hdr_data[942:928] == 15'h0001)) begin
                                    sgl_bram_wr_data[82]    <= 1'b1;  // QP1_HDR_VAL
                                    sgl_bram_wr_en          <= 1'b1;
                                    sgl_wr_done             <= 1'b1;
                                    sgl_bram_wr_data[81:80] <= 2'b11;
                                    sgl_bram_wr_data[79:0]  <= {i_hdr_data[1023:1008], i_hdr_data[1007:944]};  // hdr_gen 的{payload_len, local_offset}
                                    buf_mgr_cs              <= WR_HDR;
                                    hdr_bram_wr_en          <= 1'b0;
                                end else begin
                                    hdr_bram_wr_en <= 1'b0;
                                    sgl_bram_wr_en <= 1'b0;
                                end
                            end

                            WR_HDR: begin
                                // i_hdr_data[942:928] 对应 hdr_gen 模块 q_id[14:0]信号
                                hdr_bram_wr_en   <= (i_hdr_data[942:928] != 15'h0001);  // 不是 SQ1，才把 hdr 写入 ram
                                hdr_bram_wr_addr <= {hdr_head_ptr[HDR_BRAM_ADDR_WIDTH-1:0], 1'b1};
                                hdr_bram_wr_data <= i_hdr_data[1023:512];

                                // SQ1 的处理: 不把 hdr 写入 hdr ram，只写 sgl ram(hdr 头指针不 +1)
                                // 其余 SQ 的处理: 正常 hdr头指针 +1
                                if (sgl_wr_done || (i_hdr_data[942:928] == 15'h0001)) begin
                                    o_buf_mngr_rdy <= 1'b1;
                                    buf_mgr_cs     <= IDLE;
                                    if ((i_hdr_data[942:928] != 15'h0001) && (hdr_head_ptr[14:0] == C_MAX_HDR_DEPTH - 1'b1)) begin
                                        hdr_head_ptr[14:0] <= 15'h0000;
                                        hdr_head_ptr[15]   <= ~hdr_head_ptr[15];
                                    end else if ((i_hdr_data[942:928] != 15'h0001)) begin
                                        hdr_head_ptr[14:0] <= hdr_head_ptr[14:0] + 1'b1;
                                    end
                                end
                                if (sgl_wr_wait) begin
                                    sgl_wr_wait    <= 1'b0;
                                    sgl_bram_wr_en <= 1'b0;
                                end else if (~sgl_wr_done && ~((sgl_tail_ptr[14:0] == sgl_head_ptr[14:0]) && (sgl_tail_ptr[15] != sgl_head_ptr[15]))) begin
                                    sgl_bram_wr_en          <= 1'b1;
                                    sgl_wr_done             <= 1'b1;
                                    sgl_bram_wr_data[81:80] <= 2'b10;
                                    sgl_bram_wr_data[79:0]  <= {i_hdr_data[1023:1008], i_hdr_data[1007:944]};
                                end else begin
                                    sgl_bram_wr_en <= 1'b0;
                                end
                            end

                        endcase
                    end
                end

            end else begin  // NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN == 0

                always @(posedge core_clk) begin
                    if (core_rst) begin
                        o_buf_mngr_rdy   <= 1'b0;
                        hdr_head_ptr     <= 16'h0000;
                        sgl_bram_wr_en   <= 1'b0;
                        sgl_bram_wr_data <= 83'd000;
                        sgl_wr_done      <= 1'b1;
                        sgl_wr_wait      <= 1'b0;
                        buf_mgr_cs       <= IDLE;
                        curr_desc_done   <= 1'b0;
                        dma_en_done      <= 1'b0;
                        dma_cr_done      <= 1'b0;
                        hdr_bram_wr_addr <= {HDR_BRAM_ADDR_WIDTH + 1{1'b0}};
                    end else begin
                        case (buf_mgr_cs)
                            IDLE: begin
                                o_buf_mngr_rdy <= 1'b0;
                                if (~sgl_buf_full && ~hdr_buf_full && (i_hdr_valid && ~o_buf_mngr_rdy) && (i_hdr_data[943:928] != 16'h0001)) begin
                                    sgl_bram_wr_data[82]   <= 1'b0;
                                    hdr_bram_wr_en         <= 1'b1;
                                    hdr_bram_wr_addr       <= {hdr_head_ptr[HDR_BRAM_ADDR_WIDTH-1:0], 1'b0};
                                    hdr_bram_wr_data       <= i_hdr_data[511:0];
                                    buf_mgr_cs             <= WR_HDR;
                                    sgl_bram_wr_en         <= 1'b1;
                                    sgl_bram_wr_data[79:0] <= {8'h00, i_hdr_len, 32'd0, (32'h00000000 + (hdr_head_ptr[14:0] * 16'h80))};
                                    sgl_bram_wr_data[80]   <= 1'b1;
                                    if (i_reg_ip_ver) begin
                                        if (i_hdr_data[503:496] == 8'h0C || (i_hdr_data[503:496] == 8'h04 && i_hdr_data[943]) || i_hdr_data[503:496] == 8'h11 || ~|i_hdr_data[1023:1008]) begin
                                            sgl_bram_wr_data[81] <= 1'b1;
                                            sgl_wr_done          <= 1'b1;
                                        end else begin
                                            sgl_bram_wr_data[81] <= 1'b0;
                                            sgl_wr_done          <= 1'b0;
                                            sgl_wr_wait          <= 1'b1;
                                        end
                                    end else begin
                                        if (i_hdr_data[343:336] == 8'h0C || (i_hdr_data[343:336] == 8'h04 && i_hdr_data[943]) || i_hdr_data[343:336] == 8'h11 || ~|i_hdr_data[1023:1008]) begin
                                            sgl_bram_wr_data[81] <= 1'b1;
                                            sgl_wr_done          <= 1'b1;
                                        end else begin
                                            sgl_bram_wr_data[81] <= 1'b0;
                                            sgl_wr_done          <= 1'b0;
                                            sgl_wr_wait          <= 1'b1;
                                        end
                                    end
                                end else if (~sgl_buf_full && ~hdr_buf_full && (i_hdr_valid && ~o_buf_mngr_rdy) && (i_hdr_data[942:928] == 15'h0001)) begin
                                    sgl_bram_wr_data[82]    <= 1'b1;
                                    sgl_bram_wr_en          <= 1'b1;
                                    sgl_wr_done             <= 1'b1;
                                    sgl_bram_wr_data[81:80] <= 2'b11;
                                    sgl_bram_wr_data[79:0]  <= {i_hdr_data[1023:1008], i_hdr_data[1007:944]};
                                    buf_mgr_cs              <= WR_HDR;
                                    hdr_bram_wr_en          <= 1'b0;
                                end else begin
                                    hdr_bram_wr_en <= 1'b0;
                                    sgl_bram_wr_en <= 1'b0;
                                end
                            end
                            WR_HDR: begin
                                hdr_bram_wr_en   <= (i_hdr_data[942:928] != 15'h0001);
                                hdr_bram_wr_addr <= {hdr_head_ptr[HDR_BRAM_ADDR_WIDTH-1:0], 1'b1};
                                hdr_bram_wr_data <= i_hdr_data[1023:512];
                                if (sgl_wr_done || (i_hdr_data[942:928] == 15'h0001)) begin
                                    o_buf_mngr_rdy <= 1'b1;
                                    buf_mgr_cs     <= IDLE;
                                    if ((i_hdr_data[942:928] != 15'h0001) && (hdr_head_ptr[14:0] == C_MAX_HDR_DEPTH - 1'b1)) begin
                                        hdr_head_ptr[14:0] <= 15'h0000;
                                        hdr_head_ptr[15]   <= ~hdr_head_ptr[15];
                                    end else if ((i_hdr_data[942:928] != 15'h0001)) begin
                                        hdr_head_ptr[14:0] <= hdr_head_ptr[14:0] + 1'b1;
                                    end
                                end
                                if (sgl_wr_wait) begin
                                    sgl_wr_wait    <= 1'b0;
                                    sgl_bram_wr_en <= 1'b0;
                                end else if (~sgl_wr_done && ~((sgl_tail_ptr[14:0] == sgl_head_ptr[14:0]) && (sgl_tail_ptr[15] != sgl_head_ptr[15]))) begin
                                    sgl_bram_wr_en          <= 1'b1;
                                    sgl_wr_done             <= 1'b1;
                                    sgl_bram_wr_data[81:80] <= 2'b10;
                                    sgl_bram_wr_data[79:0]  <= {i_hdr_data[1023:1008], i_hdr_data[1007:944]};
                                end else begin
                                    sgl_bram_wr_en <= 1'b0;
                                end
                            end
                        endcase
                    end
                end

            end
        end
    endgenerate

    // ======== hdr 和 sgl ram 指针(地址)管理 ========
    always @(posedge core_clk) begin
        if (core_rst) begin
            sgl_head_ptr      <= 16'h0000;
            sgl_head_ptr_prev <= 16'h0000;
            sgl_tail_ptr      <= 16'h0000;
            sgl_buf_full      <= 1'b0;
            hdr_buf_full      <= 1'b0;
            hdr_tail_ptr      <= 16'h0000;
            sgl_bram_rd_en_r  <= 1'b0;
            hdr_bram_rd_en_r  <= 1'b0;
            dma_error         <= 1'b0;
            sgl_wr_cnt        <= {SGL_BRAM_ADDR_WIDTH{1'b0}};
        end else begin
            sgl_buf_full <= (sgl_tail_ptr[14:0] == sgl_head_ptr[14:0]) && (sgl_tail_ptr[15] != sgl_head_ptr[15]);
            hdr_buf_full <= (hdr_tail_ptr[14:0] == hdr_head_ptr[14:0]) && (hdr_tail_ptr[15] != hdr_head_ptr[15]);
            if (sgl_bram_wr_en) begin  // write后，头指针+1
                sgl_head_ptr_prev <= sgl_head_ptr;
                if (sgl_head_ptr[14:0] == C_MAX_SGL_DEPTH - 1'b1) begin
                    sgl_head_ptr[14:0] <= 15'h0000;
                    sgl_head_ptr[15]   <= ~sgl_head_ptr[15];
                end else begin
                    sgl_head_ptr[14:0] <= sgl_head_ptr[14:0] + 1'b1;
                end
            end
            sgl_bram_rd_en_r <= sgl_bram_rd_en;
            hdr_bram_rd_en_r <= hdr_bram_rd_en;

            if (sgl_bram_rd_en) begin  // read后，尾指针+1
                if (sgl_tail_ptr[14:0] == C_MAX_SGL_DEPTH - 1'b1) begin
                    sgl_tail_ptr[14:0] <= 15'h0000;
                    sgl_tail_ptr[15]   <= ~sgl_tail_ptr[15];
                end else begin
                    sgl_tail_ptr[14:0] <= sgl_tail_ptr[14:0] + 1'b1;
                end
            end

            if (hdr_bram_pop) begin  // pop后，尾指针+1
                if (hdr_tail_ptr[14:0] == C_MAX_HDR_DEPTH - 1'b1) begin
                    hdr_tail_ptr[14:0] <= 15'h0000;
                    hdr_tail_ptr[15]   <= ~hdr_tail_ptr[15];
                end else begin
                    hdr_tail_ptr[14:0] <= hdr_tail_ptr[14:0] + 1'b1;
                end
            end
        end
    end

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (83 * C_MAX_SGL_DEPTH),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),        //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                    //string;
        .USE_MEM_INIT     (1),                     //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),       //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                     //integer; 0,1
        .ECC_MODE         ("no_ecc"),              //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                     //Do not Change

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(83),                  //positive integer
        .BYTE_WRITE_WIDTH_A(83),                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (SGL_BRAM_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (83),                   //positive integer
        .ADDR_WIDTH_B      (SGL_BRAM_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                  //string
        .READ_LATENCY_B    (1),                    //non-negative integer
        .WRITE_MODE_B      ("read_first")          //string; "write_first", "read_first", "no_change" 

    ) sgl_buffer_xpm (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (sgl_bram_wr_en),
        .wea           (1'b1),
        .addra         (sgl_bram_wr_addr),
        .dina          (sgl_bram_wr_data),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (core_rst),
        .enb     (sgl_bram_rd_en),
        .regceb  (1'b1),
        .addrb   (sgl_bram_rd_addr[SGL_BRAM_ADDR_WIDTH-1:0]),
        .doutb   (sgl_bram_rd_data),
        .sbiterrb(),
        .dbiterrb()

    );

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (512 * 2 * C_MAX_HDR_DEPTH),  //positive integer
        /* 512 x (2*C_MAX_HDR_DEPTH) is a deep, very wide buffer.  With the
         * configured 512 headers it needs fifteen BRAM primitives but only
         * eight width-banked UltraRAMs.  Depth and one-cycle read behavior are
         * unchanged. */
        .MEMORY_PRIMITIVE ("ultra"),                    //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),             //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                     //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                         //string;
        .USE_MEM_INIT     (1),                          //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),            //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                          //integer; 0,1
        .ECC_MODE         ("no_ecc"),                   //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                          //Do not Change

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(512),                     //positive integer
        .BYTE_WRITE_WIDTH_A(512),                     //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (HDR_BRAM_ADDR_WIDTH + 1), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (512),                      //positive integer
        .ADDR_WIDTH_B      (HDR_BRAM_ADDR_WIDTH + 1),  //positive integer
        .READ_RESET_VALUE_B("0"),                      //string
        .READ_LATENCY_B    (1),                        //non-negative integer
        .WRITE_MODE_B      ("read_first")              //string; "write_first", "read_first", "no_change" 

    ) hdr_buffer_xpm (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (hdr_bram_wr_en),
        .wea           (1'b1),
        .addra         (hdr_bram_wr_addr),
        .dina          (hdr_bram_wr_data),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (core_rst),
        .enb     (hdr_bram_rd_en),                           //bus2ip_rd_req),
        .regceb  (1'b1),
        .addrb   (hdr_bram_rd_addr[HDR_BRAM_ADDR_WIDTH:0]),
        .doutb   (hdr_bram_rd_data),
        .sbiterrb(),
        .dbiterrb()

    );

    //-------------------------------------------------------------------------------------------------

endmodule
