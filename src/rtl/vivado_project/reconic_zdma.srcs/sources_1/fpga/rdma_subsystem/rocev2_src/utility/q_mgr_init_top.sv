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
// q_mgr_init_top.v
//-----------------------------------------------------------------------------
//
// Filename        : q_mgr_init_top.sv
// Version         : v1.0
// Description     : bram_q_manager init top file. This is a generic queue manager
//		     file. This can be instantiated for any types of queues 
//		     like submission queue, completion queue or admin queue
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- q_mgr_init_top.sv 
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
// bram_q_manager init top 文件。这是一个通用队列管理器文件。它可以为任何类型的队列（如提交队列、完成队列或管理队列）实例化
// 弹出后，
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module q_mgr_init_top #(
    parameter C_NUM_QUEUES         = 256,                   // inst_osq_manager: 32
    parameter C_Q_DEPTH            = 32,                    // inst_osq_manager: 16
    parameter C_ENTRY_SIZE         = 64,                    // inst_osq_manager: 1
    parameter C_PTR_WIDTH          = 5,                     // inst_osq_manager: 4
    parameter C_THRESHOLD          = 4,                     // inst_osq_manager: 5
    parameter C_ECN_THRESHOLD      = 4,                     // inst_osq_manager: 12
    parameter C_PUSH_POP_1         = 1,
    parameter C_ADDR_WIDTH         = 32,                    // inst_osq_manager: 9
    parameter C_FULL_DEPEND_ON_ECN = 0,
    parameter C_ENABLE_WR_ADDR     = 1,
    parameter C_ENABLE_RD_ADDR     = 1,
    parameter C_RDADDR_INTF        = 0,
    parameter LOG2_NUM_QUEUES      = $clog2(C_NUM_QUEUES),
    parameter RD_POP_QPID_SAME     = 1
) (
    // Clock and Reset
    input core_clk,  // Module Clock  
    input core_rst,  // Active high core reset

    // 推送接口
    input                                i_q_push,       // 启用队列的推送功能
    input        [LOG2_NUM_QUEUES - 1:0] i_q_push_qpid,  // 队列ID
    output logic                         o_q_push_idle,  // 推送空闲状态，为高表示可以接收新的推送

    input                                i_wr_addr_rd_en,
    input        [LOG2_NUM_QUEUES - 1:0] i_wr_addr_qnum,
    input        [   C_ADDR_WIDTH - 1:0] i_wr_base_addr,      // base address of where the Q exists
    output logic                         o_wr_addr_rd_valid,  // i_wr_addr_rd_en 拉高后两周期输出 o_wr_addr_rd_valid
    output logic [    C_ADDR_WIDTH -1:0] o_wr_addr,           // Write address stacked on one bus, i_wr_addr_rd_en 拉高后下一周期输出 o_wr_addr

    // 弹出接口
    input                                i_q_pop,       // Pop enable for a queue
    input        [LOG2_NUM_QUEUES - 1:0] i_q_pop_qpid,
    output logic                         o_q_pop_idle,

    input                                i_rd_addr_rd_en,
    input        [LOG2_NUM_QUEUES - 1:0] i_rd_addr_qnum,
    input        [   C_ADDR_WIDTH - 1:0] i_rd_base_addr,
    output logic                         o_rd_addr_rd_valid,  // i_rd_addr_rd_en 拉高后两周期输出 o_rd_addr_rd_valid
    output logic [    C_ADDR_WIDTH -1:0] o_rd_addr,           // Read address stacked on one bus, i_rd_addr_rd_en 拉高后下一周期输出 o_rd_addr

    // queue depth and threshold control for almost full
    input [C_PTR_WIDTH:0] i_q_depth,  // 可配置的 q 深度（小于参数定义的最大值）

    // flush inputs
    // 把某个队列(QPID=i_q_flush_qpid)强制恢复初始状态：wr_ptr=0, rd_ptr=0, valid_entries=0, free_entries=i_q_depth。
    input                                i_q_flush,       // 刷新使能
    input        [LOG2_NUM_QUEUES - 1:0] i_q_flush_qpid,
    output logic                         o_q_flush_done,

    // 不使用
    input                                vld_free_entry_rden,     // 1'b0
    input        [LOG2_NUM_QUEUES - 1:0] vld_free_entry_qnum,     // {C_QP_INDX_WIDTH{1'b0}}
    output logic                         vld_free_entry_data_vld, // 空接口

    output logic [C_PTR_WIDTH:0] o_num_valid_entries,  // 队列中有效条目的数量
    output logic [C_PTR_WIDTH:0] o_num_free_entries,   // 队列中的空闲条目数

    //Lut ram based signls output
    input                         init_start,
    input [LOG2_NUM_QUEUES-1 : 0] init_count,

    // 
    input                         i_threshold_cntrl_valid,
    input [LOG2_NUM_QUEUES-1 : 0] i_threshold_cntrl_id,
    input                         i_threshold_cntrl_data,

    // 只要有推送，等待操作结束(回到PUSH_RD状态)，就会输出o_q_empty_reset_*信号，说明队列空状态重置，不空了
    output logic                         o_q_empty_reset_valid,
    output logic [LOG2_NUM_QUEUES-1 : 0] o_q_empty_reset_id,
    // 弹操作 / 刷新操作 结束，队列为空信号
    output logic                         o_q_empty_set_valid,
    output logic [LOG2_NUM_QUEUES-1 : 0] o_q_empty_set_id,

    // 推操作结束，队列为满信号
    output logic                         o_q_full_set_valid,
    output logic [LOG2_NUM_QUEUES-1 : 0] o_q_full_set_id,
    // 弹操作 / 刷新操作 结束，就会输出o_q_full_reset_*信号，说明队列满状态重置，不满了
    output logic                         o_q_full_reset_valid,
    output logic [LOG2_NUM_QUEUES-1 : 0] o_q_full_reset_id,

    // 在 push 写入后剩余空间 (i_q_depth - valid_entries) 小于等于阈值 (ECN 或普通)
    output logic                         o_q_almost_full_set_valid,
    output logic [LOG2_NUM_QUEUES-1 : 0] o_q_almost_full_set_id,
    // 在 pop 写入后剩余空间重新大于阈值 (ECN 或普通)，或 flush 操作 结束
    output logic                         o_q_almost_full_reset_valid,
    output logic [LOG2_NUM_QUEUES-1 : 0] o_q_almost_full_reset_id

);
    `include "../../rdma_macros.vh"

    //==========================================================================
    // local parameters
    //==========================================================================
    localparam C_WRADDR_NOSTORE = 0;
    localparam MEM_DATA_WIDTH = (4 * (C_PTR_WIDTH + 1));
    localparam MEM_ADDR_WIDTH = $clog2(C_NUM_QUEUES);
    localparam MEM_SIZE = C_NUM_QUEUES * MEM_DATA_WIDTH;
    //localparam LOG2_NUM_QUEUES = $clog2(C_NUM_QUEUES);
    localparam ADDR_SHIFT = $clog2(C_ENTRY_SIZE);  // 地址偏移量，向上取整

    //==========================================================================
    // fsm states
    //==========================================================================
    typedef enum logic [1:0] {
        INIT_IDLE,
        INIT_WR,
        INIT_INCR
    } init_t;
    init_t init_curr_st, init_next_st;
    typedef enum logic [1:0] {
        PUSH_RD,
        PUSH_GEN_WR_ADDR,
        PUSH_MOD,
        PUSH_WR
    } push_t;
    push_t push_curr_st, push_next_st;
    typedef enum logic [1:0] {
        POP_RD,
        POP_MOD,
        POP_GEN_RD_ADDR,
        POP_WR
    } pop_t;
    pop_t pop_curr_st, pop_next_st;

    //==========================================================================
    // structures
    //==========================================================================
    typedef struct packed {
        logic [C_PTR_WIDTH:0] rd_ptr;
        logic [C_PTR_WIDTH:0] wr_ptr;
        logic [C_PTR_WIDTH:0] free_entries;
        logic [C_PTR_WIDTH:0] valid_entries;
    } ptrmem;

    ptrmem ptrmem_init_wrdata, push_wrdata, push_rddata, pop_wrdata, pop_rddata, noread_push_data, noread_pop_data, flush_data;

    //==========================================================================
    // registers and wires
    //==========================================================================
    logic                         initialize;
    logic [      C_PTR_WIDTH : 0] q_depth_ff;
    logic [LOG2_NUM_QUEUES - 1:0] init_q_num;
    logic                         init_mem_wren;
    logic [LOG2_NUM_QUEUES - 1:0] init_mem_wraddr;
    logic [ MEM_DATA_WIDTH - 1:0] init_mem_wrdata;
    logic [   C_ADDR_WIDTH - 1:0] init_mem_addr_wrdata;
    logic                         init_done;
    logic                         same_q_push_pop;
    logic                         same_q_push_pop_q;
    logic                         push_mem_rden;
    logic [LOG2_NUM_QUEUES - 1:0] push_mem_rdaddr;
    logic [ MEM_DATA_WIDTH - 1:0] push_mem_rddata;
    logic                         push_mem_wren;
    logic [LOG2_NUM_QUEUES - 1:0] push_mem_wraddr;
    logic [ MEM_DATA_WIDTH - 1:0] push_mem_wrdata;
    logic [LOG2_NUM_QUEUES - 1:0] push_q_num;
    logic [LOG2_NUM_QUEUES - 1:0] push_q_num_q;
    logic [    C_PTR_WIDTH - 1:0] push_ptr_cmpr_wrptr;
    logic [    C_PTR_WIDTH - 1:0] push_ptr_cmpr_rdptr;
    logic                         push_ptr_cmpr_wrptr_msb;
    logic                         push_ptr_cmpr_rdptr_msb;
    logic                         pop_mem_rden;
    logic [LOG2_NUM_QUEUES - 1:0] pop_mem_rdaddr;
    logic [ MEM_DATA_WIDTH - 1:0] pop_mem_rddata;
    logic                         pop_mem_wren;
    logic [LOG2_NUM_QUEUES - 1:0] pop_mem_wraddr;
    logic [ MEM_DATA_WIDTH - 1:0] pop_mem_wrdata;
    logic [LOG2_NUM_QUEUES - 1:0] pop_q_num;
    logic [LOG2_NUM_QUEUES - 1:0] pop_q_num_q;
    logic [    C_PTR_WIDTH - 1:0] pop_ptr_cmpr_wrptr;
    logic [    C_PTR_WIDTH - 1:0] pop_ptr_cmpr_rdptr;
    logic                         pop_ptr_cmpr_wrptr_msb;
    logic                         pop_ptr_cmpr_rdptr_msb;
    logic                         mem_wren_a;
    logic                         mem_rden_a;
    logic [LOG2_NUM_QUEUES - 1:0] mem_addr_a;
    logic [ MEM_DATA_WIDTH - 1:0] mem_wrdata_a;
    logic                         mem_wren_b;
    logic                         mem_rden_b;
    logic [LOG2_NUM_QUEUES - 1:0] mem_addr_b;
    logic [ MEM_DATA_WIDTH - 1:0] mem_wrdata_b;
    logic                         vld_free_entry_rden_stky;
    logic                         stat_rd_en;
    logic [LOG2_NUM_QUEUES - 1:0] stat_rd_addr;
    logic                         pop_mem_wren_q;
    logic [LOG2_NUM_QUEUES - 1:0] pop_mem_wraddr_q;
    logic [   C_ADDR_WIDTH - 1:0] rd_addr_in;
    logic                         push_mem_wren_q;
    logic [LOG2_NUM_QUEUES - 1:0] push_mem_wraddr_q;
    logic [   C_ADDR_WIDTH - 1:0] wr_addr_in;
    logic [   C_ADDR_WIDTH - 1:0] rd_addr_int;
    logic [   C_ADDR_WIDTH - 1:0] wr_addr_int;
    logic                         i_wr_addr_rd_en_q;
    logic                         i_rd_addr_rd_en_q;
    logic [   C_ADDR_WIDTH - 1:0] i_wr_addr_qnum_q;
    logic [   C_ADDR_WIDTH - 1:0] i_rd_addr_qnum_q;
    logic [LOG2_NUM_QUEUES - 1:0] push_qpid;
    logic [LOG2_NUM_QUEUES - 1:0] pop_qpid;
    logic                         wr_addr_mem_wren;
    logic [LOG2_NUM_QUEUES - 1:0] wr_addr_mem_qpid;
    logic [   C_ADDR_WIDTH - 1:0] wr_addr_mem_wrdata;
    logic                         rd_addr_mem_wren;
    logic                         rd_addr_mem_wren_q;
    logic [LOG2_NUM_QUEUES - 1:0] rd_addr_mem_qpid;
    logic [   C_ADDR_WIDTH - 1:0] rd_addr_mem_wrdata;
    logic [   C_ADDR_WIDTH - 1:0] rd_addr_mem_wrdata_q;
    logic [    C_PTR_WIDTH - 1:0] rd_addr_mem_push_data;
    logic                         push_t1_pop_t0;
    logic                         push_t0_pop_t1;
    logic                         no_read_for_push;
    logic                         no_read_for_pop;
    logic [   C_ADDR_WIDTH - 1:0] in_rd_base_addr;
    logic [   C_ADDR_WIDTH - 1:0] in_wr_base_addr;
    wire                          i_threshold_cntrl_rd_data_0;  // 若=1 采用 C_ECN_THRESHOLD，否则用 C_THRESHOLD
    wire                          i_threshold_cntrl_rd_data_1;  // 若=1 用 C_ECN_THRESHOLD 作恢复门限判定，否则用 C_THRESHOLD。

    /***************************************************************************
   *                                          *
   * 									      *
   * 			初始化逻辑    		            *
   * 									      *
   * 									      *
   ***************************************************************************/

    // 当深度发生变化时，初始化 FSM 开始，即 initialize 拉高
    // 延迟 i_q_depth 以了解配置的变化
    `MSFF_R(q_depth_ff, i_q_depth, core_clk, core_rst)
    assign initialize = |(i_q_depth ^ q_depth_ff);  // 当前周期深度 异或 上周期深度，只要有一位不同，则initialize就拉高

    // current state flops
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            init_curr_st <= INIT_IDLE;
        end else begin
            init_curr_st <= init_next_st;
        end
    end

    always_comb begin
        case (init_curr_st)
            INIT_IDLE: begin
                init_next_st = initialize ? INIT_WR : INIT_IDLE;
            end
            INIT_WR: begin
                init_next_st = INIT_INCR;
            end
            default: begin  // INIT_INCR
                init_next_st = (init_q_num == C_NUM_QUEUES - 1) ? INIT_IDLE : INIT_WR;
            end
        endcase
    end

    // FSM初始化完成指示器
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            init_done <= 1'b0;
        end else if ((init_curr_st == INIT_INCR) & (init_q_num == C_NUM_QUEUES - 1)) begin
            init_done <= 1'b1;
        end
    end

    // 当 fsm 处于递增状态时，队列编号递增
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            init_q_num <= {LOG2_NUM_QUEUES{1'b0}};
        end else if (init_curr_st == INIT_INCR) begin
            init_q_num <= init_q_num + 1'b1;  // init_q_num 变化，这时 init_curr_st = INIT_WR
        end
    end

    //==========================================================================
    // 初始化更新逻辑: 将 {'b0, 'b0, i_q_depth, 'b0} 写入每个队列的指针内存
    //==========================================================================
    assign init_mem_wren                    = (init_curr_st == INIT_WR);
    assign init_mem_wraddr                  = init_q_num;

    assign ptrmem_init_wrdata.wr_ptr        = {(C_PTR_WIDTH + 1) {1'b0}};
    assign ptrmem_init_wrdata.rd_ptr        = {(C_PTR_WIDTH + 1) {1'b0}};
    // CK : isn't the queue depth same across all queues? if so, this can be
    // simply tied to one value
    assign ptrmem_init_wrdata.free_entries  = i_q_depth;
    assign ptrmem_init_wrdata.valid_entries = {(C_PTR_WIDTH + 1) {1'b0}};

    assign init_mem_wrdata                  = ptrmem_init_wrdata;  // {'b0, 'b0, i_q_depth, 'b0}
    assign init_mem_addr_wrdata             = {C_ADDR_WIDTH{1'b0}};

    /***************************************************************************
   *                                                                          *
   * 									      *
   * 		       Common logic for push and pop	                      *
   * 									      *
   * 									      *
   ***************************************************************************/


    // 当推送和弹出发生在同一队列时设置。
    // 内容无需更改
    assign same_q_push_pop                  = i_q_push & i_q_pop & (i_q_push_qpid == i_q_pop_qpid);

    // delay flops
    `MSFF_R(same_q_push_pop_q, same_q_push_pop, core_clk, core_rst)

    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            o_num_valid_entries <= {(C_PTR_WIDTH + 1) {1'b0}};
            o_num_free_entries  <= {(C_PTR_WIDTH + 1) {1'b0}};
        end else if (vld_free_entry_data_vld) begin
            o_num_valid_entries <= pop_rddata.valid_entries;
            o_num_free_entries  <= pop_rddata.free_entries;
        end
    end

    // flush data resets all pointers
    assign flush_data.valid_entries = {(C_PTR_WIDTH + 1) {1'b0}};
    assign flush_data.free_entries  = i_q_depth;
    assign flush_data.wr_ptr        = {(C_PTR_WIDTH + 1) {1'b0}};
    assign flush_data.rd_ptr        = {(C_PTR_WIDTH + 1) {1'b0}};

    // 根据是否正在进行初始化或推/弹工作，init fsms 或 push/pop fsms 成为主站
    always_comb begin
        if (init_curr_st != INIT_IDLE) begin  // init_curr_st == INIT_WR || init_curr_st == INIT_INCR: 正在进行初始化
            mem_wren_a   = init_mem_wren;
            mem_rden_a   = 1'b0;
            mem_addr_a   = init_mem_wraddr;
            mem_wrdata_a = init_mem_wrdata;  // {5'b0, 5'b0, i_q_depth, 5'b0}
            mem_wren_b   = 1'b0;
            mem_rden_b   = 1'b0;
            mem_addr_b   = {LOG2_NUM_QUEUES{1'b0}};
            mem_wrdata_b = {MEM_DATA_WIDTH{1'b0}};
        end else begin  // init_curr_st == INIT_IDLE: 初始化完成，可以进行推/弹工作
            // push operation
            mem_wren_a   = push_mem_wren;
            mem_rden_a   = push_mem_rden;
            mem_addr_a   = push_mem_rden ? push_mem_rdaddr : push_mem_wraddr;
            mem_wrdata_a = push_mem_wrdata;
            mem_rden_b   = pop_mem_rden;

            // 只有在 Push 与 Pop 两个 FSM 都处于空闲读状态且当前无新 push/pop 操作
            if ((pop_curr_st == POP_RD) & (push_curr_st == PUSH_RD) & i_q_flush & !i_q_pop & !i_q_push) begin  // flush operation
                mem_wren_b   = 1'b1;
                mem_addr_b   = i_q_flush_qpid;
                mem_wrdata_b = flush_data;
            end else begin  // pop operation
                mem_wren_b   = pop_mem_wren;
                mem_addr_b   = pop_mem_rden ? pop_mem_rdaddr : pop_mem_wraddr;
                mem_wrdata_b = pop_mem_wrdata;
            end
        end
    end

    // data valid for valid/free entry value
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            vld_free_entry_data_vld <= 1'b0;
        end else if (stat_rd_en & ((pop_curr_st == POP_RD) & !(!same_q_push_pop & (i_q_pop)))) begin
            vld_free_entry_data_vld <= 1'b1;
        end else begin
            vld_free_entry_data_vld <= 1'b0;
        end
    end

    /***************************************************************************
   *                                                                          *
   * 									      *
   * 			      Push update logic		                      *
   * 									      *
   * 									      *
   ***************************************************************************/

    // store QPID for comparison
    `MSFF_R(push_qpid, (i_q_push ? i_q_push_qpid : push_qpid), core_clk, core_rst)

    // current state flops
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            push_curr_st <= PUSH_RD;
        end else begin
            push_curr_st <= push_next_st;
        end
    end

    // 设置为表示可以进行新的推送
    assign o_q_push_idle = (push_curr_st == PUSH_RD);

    always_comb begin
        case (push_curr_st)
            PUSH_RD: begin
                if (i_q_push) begin
                    if ((pop_curr_st == POP_MOD) & (i_q_push_qpid == pop_qpid)) begin
                        // 如果 pop 已经在读取相同的 QPID 值，那么只需向写地址内存写入数据，
                        // 而不要从推送端向指针内存写入数据，因为 pop 端会写入正确的值。
                        push_next_st = PUSH_GEN_WR_ADDR;
                    end else begin
                        push_next_st = PUSH_MOD;
                    end
                end else begin
                    push_next_st = PUSH_RD;
                end
            end
            PUSH_MOD: begin
                push_next_st = PUSH_WR;
            end
            default: begin  // PUSH_GEN_WR_ADDR, PUSH_WR
                push_next_st = PUSH_RD;
            end
        endcase
    end

    // 当没有向同一队列发送 POP 请求时，此操作为只读
    assign push_mem_rden = i_q_push & !((pop_curr_st == POP_MOD) & (i_q_push_qpid == pop_qpid)) & !(pop_mem_wren & (pop_mem_wraddr == i_q_push_qpid));
    // no read
    `MSFF_R(no_read_for_push, (i_q_push & (pop_mem_wren & (pop_mem_wraddr == i_q_push_qpid))), core_clk, core_rst)
    assign push_q_num = i_q_push_qpid;
    `MSFF_R(push_q_num_q, push_q_num, core_clk, core_rst)
    assign push_mem_rdaddr = push_q_num;
    `MSFF_R(noread_push_data, pop_wrdata, core_clk, core_rst)
    assign push_rddata = (no_read_for_push) ? noread_push_data : push_mem_rddata;

    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            push_wrdata     <= {MEM_DATA_WIDTH{1'b0}};
            push_mem_wraddr <= {LOG2_NUM_QUEUES{1'b0}};
            push_mem_wren   <= 1'b0;
        end else if (push_curr_st == PUSH_MOD) begin  // PUSH_MOD 状态: 计算新的写指针、读指针、有效条目和空闲条目，并保存至 push_wrdata
            // 如果对同一个队列同时执行入队和出队操作，则递增写入和读取指针，并且不要更改空闲和有效的条目。
            push_wrdata.valid_entries <= (same_q_push_pop_q | (i_q_pop & (i_q_pop_qpid == push_qpid))) ? push_rddata.valid_entries : (push_rddata.valid_entries + 1'b1);
            push_wrdata.free_entries  <= (same_q_push_pop_q | (i_q_pop & (i_q_pop_qpid == push_qpid))) ? push_rddata.free_entries : (push_rddata.free_entries - 1'b1);
            if ((push_rddata.wr_ptr[C_PTR_WIDTH-1:0] + 1'b1) < i_q_depth) begin
                push_wrdata.wr_ptr[C_PTR_WIDTH-1:0] <= push_rddata.wr_ptr[C_PTR_WIDTH-1:0] + 1'b1;
                push_wrdata.wr_ptr[C_PTR_WIDTH]     <= push_rddata.wr_ptr[C_PTR_WIDTH];
            end else begin
                push_wrdata.wr_ptr[C_PTR_WIDTH-1:0] <= push_rddata.wr_ptr[C_PTR_WIDTH-1:0] + 1'b1 - i_q_depth;
                push_wrdata.wr_ptr[C_PTR_WIDTH]     <= ~push_rddata.wr_ptr[C_PTR_WIDTH];
            end
            if (same_q_push_pop_q | (i_q_pop & (i_q_pop_qpid == push_qpid))) begin
                if ((push_rddata.rd_ptr[C_PTR_WIDTH-1:0] + 1'b1) < i_q_depth) begin
                    push_wrdata.rd_ptr[C_PTR_WIDTH-1:0] <= push_rddata.rd_ptr[C_PTR_WIDTH-1:0] + 1'b1;
                    push_wrdata.rd_ptr[C_PTR_WIDTH]     <= push_rddata.rd_ptr[C_PTR_WIDTH];
                end else begin
                    push_wrdata.rd_ptr[C_PTR_WIDTH-1:0] <= push_rddata.rd_ptr[C_PTR_WIDTH-1:0] + 1'b1 - i_q_depth;
                    push_wrdata.rd_ptr[C_PTR_WIDTH]     <= ~push_rddata.rd_ptr[C_PTR_WIDTH];
                end
            end else begin
                push_wrdata.rd_ptr <= push_rddata.rd_ptr;
            end
            push_mem_wraddr <= push_qpid;
            push_mem_wren   <= 1'b1;
        end else begin
            push_mem_wraddr <= {LOG2_NUM_QUEUES{1'b0}};
            push_mem_wren   <= 1'b0;
        end
    end

    // set this bit when push is in T1 and pop is in T0
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            push_t1_pop_t0 <= 1'b0;
        end else if (push_curr_st == PUSH_GEN_WR_ADDR) begin
            push_t1_pop_t0 <= 1'b1;
        end else begin
            push_t1_pop_t0 <= 1'b0;
        end
    end

    // push operation write data
    assign push_mem_wrdata         = push_wrdata;

    // values for comparison
    assign push_ptr_cmpr_wrptr     = push_wrdata.wr_ptr[C_PTR_WIDTH-1:0];  // push写指针低位
    assign push_ptr_cmpr_rdptr     = push_wrdata.rd_ptr[C_PTR_WIDTH-1:0];  // push读指针低位
    assign push_ptr_cmpr_wrptr_msb = push_wrdata.wr_ptr[C_PTR_WIDTH];  // push写指针高位
    assign push_ptr_cmpr_rdptr_msb = push_wrdata.rd_ptr[C_PTR_WIDTH];  // push读指针高位


    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QUEUES)
    ) inst_i_threshold_cntrl_0 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || i_threshold_cntrl_valid),
        .wr_addr (init_start ? init_count : i_threshold_cntrl_id),
        .wr_data (init_start ? 1'b0 : i_threshold_cntrl_data),
        .rd_en   (1'b1),
        .rd_addr (push_mem_rdaddr),
        .rd_data (i_threshold_cntrl_rd_data_0)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QUEUES)
    ) inst_i_threshold_cntrl_1 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || i_threshold_cntrl_valid),
        .wr_addr (init_start ? init_count : i_threshold_cntrl_id),
        .wr_data (init_start ? 1'b0 : i_threshold_cntrl_data),
        .rd_en   (1'b1),
        .rd_addr (pop_mem_rdaddr),
        .rd_data (i_threshold_cntrl_rd_data_1)
    );

    always_ff @(posedge core_clk) begin : o_q_full_proc
        if (core_rst) begin
            o_q_full_set_valid   <= 1'b0;
            o_q_full_set_id      <= {LOG2_NUM_QUEUES{1'b0}};
            o_q_full_reset_valid <= 1'b0;
            o_q_full_reset_id    <= {LOG2_NUM_QUEUES{1'b0}};
        end else begin
            // 本次 push 更新后，写指针“追上”读指针（低位相等），且绕圈标志翻转（MSB 不同），说明刚好占满了所有条目 → 队列进入满状态
            if ((push_mem_wren & (push_ptr_cmpr_wrptr == push_ptr_cmpr_rdptr) & (push_ptr_cmpr_wrptr_msb ^ push_ptr_cmpr_rdptr_msb))) begin
                o_q_full_set_valid <= 1'b1;
                o_q_full_set_id    <= push_mem_wraddr;
            end else begin
                o_q_full_set_valid <= 1'b0;
            end
            // 在 RTL 中，同一时钟下的同一地址不会发生推送和弹出。
            if (pop_mem_wren) begin  // 刚完成一次弹出(Pop)写回，该队列的占用减少，若之前为满需要复位满状态
                o_q_full_reset_valid <= 1'b1;
                o_q_full_reset_id    <= pop_mem_wraddr;
            end else if ((pop_curr_st == POP_RD) & (push_curr_st == PUSH_RD) & i_q_flush & !i_q_pop & !i_q_push) begin  // Push/Pop 两个 FSM 都在空闲读态，当前无新 push/pop，且收到 flush 请求。此时执行队列强制清零，满状态也要复位
                o_q_full_reset_valid <= !o_q_flush_done;
                o_q_full_reset_id    <= i_q_flush_qpid;
            end else begin
                o_q_full_reset_valid <= 1'b0;
            end
        end
    end

    always_ff @(posedge core_clk) begin : o_q_almost_full_proc
        if (core_rst) begin
            o_q_almost_full_set_valid   <= 1'b0;
            o_q_almost_full_set_id      <= {LOG2_NUM_QUEUES{1'b0}};
            o_q_almost_full_reset_valid <= 1'b0;
            o_q_almost_full_reset_id    <= {LOG2_NUM_QUEUES{1'b0}};
        end else begin
            // flush、push 和 pop 值相互排斥，因此不检查 lut ram 信号的写入冲突条件
            if (push_mem_wren && i_threshold_cntrl_rd_data_0 && ((i_q_depth - push_wrdata.valid_entries) <= C_ECN_THRESHOLD)) begin
                o_q_almost_full_set_valid <= 1'b1;
                o_q_almost_full_set_id    <= push_mem_wraddr;
            end else if (push_mem_wren && ((i_q_depth - push_wrdata.valid_entries) <= C_THRESHOLD)) begin
                o_q_almost_full_set_valid <= 1'b1;
                o_q_almost_full_set_id    <= push_mem_wraddr;
            end else begin
                o_q_almost_full_set_valid <= 1'b0;
            end

            if ((pop_curr_st == POP_RD) & (push_curr_st == PUSH_RD) & i_q_flush & !i_q_pop & !i_q_push) begin
                o_q_almost_full_reset_valid <= !o_q_flush_done;
                o_q_almost_full_reset_id    <= i_q_flush_qpid;
            end else if (pop_mem_wren && i_threshold_cntrl_rd_data_1 && (((i_q_depth - pop_wrdata.valid_entries) > C_ECN_THRESHOLD))) begin
                o_q_almost_full_reset_valid <= 1'b1;
                o_q_almost_full_reset_id    <= pop_mem_wraddr;
            end else if (pop_mem_wren && (((i_q_depth - pop_wrdata.valid_entries) > C_THRESHOLD))) begin
                o_q_almost_full_reset_valid <= 1'b1;
                o_q_almost_full_reset_id    <= pop_mem_wraddr;
            end else begin
                o_q_almost_full_reset_valid <= 1'b0;
            end

        end
    end


    /***************************************************************************
   *                                                                          *
   * 									      *
   * 			      Pop update logic		                      *
   * 									      *
   * 									      *
   ***************************************************************************/

    // store QPID for comparison
    `MSFF_R(pop_qpid, (i_q_pop ? i_q_pop_qpid : pop_qpid), core_clk, core_rst)

    // current state flops
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            pop_curr_st <= POP_RD;
        end else begin
            pop_curr_st <= pop_next_st;
        end
    end

    // set to indicate a new pop can be taken
    assign o_q_pop_idle = (pop_curr_st == POP_RD);

    always_comb begin
        case (pop_curr_st)
            POP_RD: begin
                if (i_q_pop & !same_q_push_pop) begin
                    if ((push_curr_st == PUSH_MOD) & (i_q_pop_qpid == push_qpid)) begin
                        pop_next_st = POP_GEN_RD_ADDR;
                    end else begin
                        pop_next_st = POP_MOD;
                    end
                end else begin
                    pop_next_st = POP_RD;
                end
            end
            POP_MOD: begin
                pop_next_st = POP_WR;
            end
            default: begin  // POP_WR
                pop_next_st = POP_RD;
            end
        endcase
    end

    // read only when there's no POP to the same queue
    assign pop_mem_rden = i_q_pop & !((push_curr_st == PUSH_MOD) & (i_q_pop_qpid == push_qpid)) & !(push_mem_wren & (push_mem_wraddr == i_q_pop_qpid));
    // no read
    `MSFF_R(no_read_for_pop, (i_q_pop & (push_mem_wren & (push_mem_wraddr == i_q_pop_qpid))), core_clk, core_rst)
    assign pop_q_num = i_q_pop_qpid;
    `MSFF_R(pop_q_num_q, pop_q_num, core_clk, core_rst)
    assign pop_mem_rdaddr = pop_q_num;
    `MSFF_R(noread_pop_data, push_wrdata, core_clk, core_rst)
    assign pop_rddata = (no_read_for_pop) ? noread_pop_data : pop_mem_rddata;

    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            pop_wrdata     <= {MEM_DATA_WIDTH{1'b0}};
            pop_mem_wraddr <= {LOG2_NUM_QUEUES{1'b0}};
            pop_mem_wren   <= 1'b0;
        end else if (pop_curr_st == POP_MOD) begin  // POP_MOD 状态: 计算新的写指针、读指针、有效条目和空闲条目，并保存至 pop_wrdata
            // 如果 push 和 pop 操作同时发生，则根本不会执行 pop 操作。
            // 如果 push 操作发生在 T0，pop 操作发生在 T1，则从 pop 端也不会对内存进行任何写入操作。
            if (i_q_push & (i_q_push_qpid == pop_qpid)) begin
                pop_wrdata.valid_entries <= pop_rddata.valid_entries;
                pop_wrdata.free_entries  <= pop_rddata.free_entries;
                if ((pop_rddata.wr_ptr[C_PTR_WIDTH-1:0] + 1'b1) < i_q_depth) begin
                    pop_wrdata.wr_ptr[C_PTR_WIDTH-1:0] <= pop_rddata.wr_ptr[C_PTR_WIDTH-1:0] + 1'b1;
                    pop_wrdata.wr_ptr[C_PTR_WIDTH]     <= pop_rddata.wr_ptr[C_PTR_WIDTH];
                end else begin
                    pop_wrdata.wr_ptr[C_PTR_WIDTH-1:0] <= pop_rddata.wr_ptr[C_PTR_WIDTH-1:0] + 1'b1 - i_q_depth;
                    pop_wrdata.wr_ptr[C_PTR_WIDTH]     <= ~pop_rddata.wr_ptr[C_PTR_WIDTH];
                end
            end else begin
                pop_wrdata.wr_ptr        <= pop_rddata.wr_ptr;
                pop_wrdata.valid_entries <= pop_rddata.valid_entries - 1'b1;
                pop_wrdata.free_entries  <= pop_rddata.free_entries + 1'b1;
            end
            if ((pop_rddata.rd_ptr[C_PTR_WIDTH-1:0] + 1'b1) < i_q_depth) begin
                pop_wrdata.rd_ptr[C_PTR_WIDTH-1:0] <= pop_rddata.rd_ptr[C_PTR_WIDTH-1:0] + 1'b1;
                pop_wrdata.rd_ptr[C_PTR_WIDTH]     <= pop_rddata.rd_ptr[C_PTR_WIDTH];
            end else begin
                pop_wrdata.rd_ptr[C_PTR_WIDTH-1:0] <= pop_rddata.rd_ptr[C_PTR_WIDTH-1:0] + 1'b1 - i_q_depth;
                pop_wrdata.rd_ptr[C_PTR_WIDTH]     <= ~pop_rddata.rd_ptr[C_PTR_WIDTH];
            end
            pop_mem_wraddr <= pop_qpid;
            pop_mem_wren   <= 1'b1;
        end else begin
            pop_mem_wraddr <= {LOG2_NUM_QUEUES{1'b0}};
            pop_mem_wren   <= 1'b0;
        end
    end

    // when flush is accepted, this bit is set to indicate that flush operation
    // is completed
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            o_q_flush_done <= 1'b0;
        end else if ((pop_curr_st == POP_RD) & (push_curr_st == PUSH_RD) & i_q_flush & !i_q_pop & !i_q_push) begin
            o_q_flush_done <= 1'b1;
        end else begin
            o_q_flush_done <= 1'b0;
        end
    end

    // pop operation write data
    assign pop_mem_wrdata         = pop_wrdata;

    assign pop_ptr_cmpr_wrptr     = pop_wrdata.wr_ptr[C_PTR_WIDTH-1:0];
    assign pop_ptr_cmpr_rdptr     = pop_wrdata.rd_ptr[C_PTR_WIDTH-1:0];
    assign pop_ptr_cmpr_wrptr_msb = pop_wrdata.wr_ptr[C_PTR_WIDTH];
    assign pop_ptr_cmpr_rdptr_msb = pop_wrdata.rd_ptr[C_PTR_WIDTH];

    always_ff @(posedge core_clk) begin : Empty_gen_proc
        if (core_rst) begin
            o_q_empty_reset_valid <= 1'b0;
            o_q_empty_reset_id    <= {LOG2_NUM_QUEUES{1'b0}};
            o_q_empty_set_valid   <= 1'b0;
            o_q_empty_set_id      <= {LOG2_NUM_QUEUES{1'b0}};
        end else begin
            if (push_mem_wren) begin  // 当有推送时，队列的 empty 会被去断言
                o_q_empty_reset_valid <= 1'b1;
                o_q_empty_reset_id    <= push_mem_wraddr;
            end else begin
                o_q_empty_reset_valid <= 1'b0;
            end

            if (pop_mem_wren && pop_wrdata.wr_ptr == pop_wrdata.rd_ptr) begin  // 指针完全匹配时设置为空
                o_q_empty_set_valid <= 1'b1;
                o_q_empty_set_id    <= pop_mem_wraddr;
            end else if ((pop_curr_st == POP_RD) & (push_curr_st == PUSH_RD) & i_q_flush & !i_q_pop & !i_q_push) begin
                o_q_empty_set_valid <= !o_q_flush_done;
                o_q_empty_set_id    <= i_q_flush_qpid;
            end else begin
                o_q_empty_set_valid <= 1'b0;
            end
        end
    end

    // this bit is set when there's a read to the free/valid entries
    always_ff @(posedge core_clk) begin
        if (core_rst) begin
            stat_rd_en <= 1'b0;
        end else if (vld_free_entry_rden) begin
            stat_rd_en <= 1'b1;
        end else if (vld_free_entry_data_vld) begin
            stat_rd_en <= 1'b0;
        end
    end

    /***************************************************************************
   *                                                                          *
   * 									      *
   * 			TDP 内存 -- 用于指针存储		      *
   * 									      *
   * 									      *
   ***************************************************************************/
    xpm_memory_tdpram #(
        .ADDR_WIDTH_A           (MEM_ADDR_WIDTH),
        .ADDR_WIDTH_B           (MEM_ADDR_WIDTH),
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
        .MEMORY_SIZE            (MEM_SIZE),
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
    ) u_pointer_memory (
        // 端口 A: 用于 push
        .clka  (core_clk),
        .rsta  (core_rst),
        .regcea(1'b1),
        .ena   (mem_wren_a | mem_rden_a),
        .wea   (mem_wren_a),
        .addra (mem_addr_a),
        .dina  (mem_wrdata_a),             // push_wrdata
        .douta (push_mem_rddata),

        // 端口 B: 用于 pop
        .clkb  (core_clk),
        .rstb  (core_rst),
        .regceb(1'b1),
        .enb   (mem_wren_b | mem_rden_b),
        .web   (mem_wren_b),
        .addrb (mem_addr_b),
        .dinb  (mem_wrdata_b),             // flush_data / pop_wrdata
        .doutb (pop_mem_rddata),

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

`ifdef SIMULATION
    // dina
    ptrmem dbg_mem_wrdata_a;
    assign dbg_mem_wrdata_a.rd_ptr        = mem_wren_a ? mem_wrdata_a[19:15] : 'b0;
    assign dbg_mem_wrdata_a.wr_ptr        = mem_wren_a ? mem_wrdata_a[14:10] : 'b0;
    assign dbg_mem_wrdata_a.free_entries  = mem_wren_a ? mem_wrdata_a[9:5] : 'b0;
    assign dbg_mem_wrdata_a.valid_entries = mem_wren_a ? mem_wrdata_a[4:0] : 'b0;

    // douta
    ptrmem dbg_push_mem_rddata;
    assign dbg_push_mem_rddata.rd_ptr        = mem_rden_a ? push_mem_rddata[19:15] : 'b0;
    assign dbg_push_mem_rddata.wr_ptr        = mem_rden_a ? push_mem_rddata[14:10] : 'b0;
    assign dbg_push_mem_rddata.free_entries  = mem_rden_a ? push_mem_rddata[9:5] : 'b0;
    assign dbg_push_mem_rddata.valid_entries = mem_rden_a ? push_mem_rddata[4:0] : 'b0;

    // dinb
    ptrmem dbg_mem_wrdata_b;
    assign dbg_mem_wrdata_b.rd_ptr        = mem_wren_b ? mem_wrdata_b[19:15] : 'b0;
    assign dbg_mem_wrdata_b.wr_ptr        = mem_wren_b ? mem_wrdata_b[14:10] : 'b0;
    assign dbg_mem_wrdata_b.free_entries  = mem_wren_b ? mem_wrdata_b[9:5] : 'b0;
    assign dbg_mem_wrdata_b.valid_entries = mem_wren_b ? mem_wrdata_b[4:0] : 'b0;

    // doutb
    ptrmem dbg_pop_mem_rddata;
    assign dbg_pop_mem_rddata.rd_ptr        = mem_rden_b ? pop_mem_rddata[19:15] : 'b0;
    assign dbg_pop_mem_rddata.wr_ptr        = mem_rden_b ? pop_mem_rddata[14:10] : 'b0;
    assign dbg_pop_mem_rddata.free_entries  = mem_rden_b ? pop_mem_rddata[9:5] : 'b0;
    assign dbg_pop_mem_rddata.valid_entries = mem_rden_b ? pop_mem_rddata[4:0] : 'b0;
`endif

    /***************************************************************************
   *                                                                          *
   * 									      *
   *       Individual SDP memory for write/read -- for address storage	      *
   * 									      *
   * 									      *
   ***************************************************************************/

    generate
        if (C_ENABLE_WR_ADDR == 1) begin : enable_wr_addr

            assign in_wr_base_addr = {C_ADDR_WIDTH{1'b0}};

            always_comb begin
                if (!init_done) begin
                    wr_addr_mem_wren   = init_mem_wren;
                    wr_addr_mem_qpid   = init_mem_wraddr;
                    wr_addr_mem_wrdata = init_mem_addr_wrdata;
                end else begin
                    case (push_curr_st)
                        PUSH_RD: begin
                            if (i_q_flush & !i_q_pop & !i_q_push & (pop_curr_st == POP_RD)) begin
                                wr_addr_mem_wren   = 1'b1;
                                wr_addr_mem_qpid   = i_q_flush_qpid;
                                wr_addr_mem_wrdata = in_wr_base_addr;
                            end else begin
                                wr_addr_mem_wren   = 1'b0;
                                wr_addr_mem_qpid   = {LOG2_NUM_QUEUES{1'b0}};
                                wr_addr_mem_wrdata = in_wr_base_addr;
                            end
                        end
                        // 正常推送：推送时写入
                        PUSH_WR: begin  // 保存更新后的写指针，若是push了一个条目，则写指针会递增，指向下一个push的位置
                            wr_addr_mem_wren   = 1'b1;
                            wr_addr_mem_qpid   = push_mem_wraddr;
                            wr_addr_mem_wrdata = ((push_wrdata.wr_ptr[C_PTR_WIDTH-1:0] << ADDR_SHIFT) + in_wr_base_addr);
                        end
                        PUSH_GEN_WR_ADDR: begin
                            // 如果在 T0 中有 pop 操作，在 T1 中有 push 操作，因此本次 push 没有自行读取指针内存，写指针的最新值已经由 pop 流水线算好并写在 pop_wrdata 中。
                            wr_addr_mem_wren   = 1'b1;
                            wr_addr_mem_qpid   = push_qpid;
                            wr_addr_mem_wrdata = ((pop_wrdata.wr_ptr[C_PTR_WIDTH-1:0] << ADDR_SHIFT) + in_wr_base_addr);
                        end
                        default: begin
                            wr_addr_mem_wren   = 1'b0;
                            wr_addr_mem_qpid   = {LOG2_NUM_QUEUES{1'b0}};
                            wr_addr_mem_wrdata = in_wr_base_addr;
                        end
                    endcase
                end
            end

            // 保存最新写指针
            xpm_memory_sdpram #(
                .ADDR_WIDTH_A           (MEM_ADDR_WIDTH),
                .RST_MODE_A             ("SYNC"),
                .BYTE_WRITE_WIDTH_A     (C_ADDR_WIDTH),
                .WRITE_DATA_WIDTH_A     (C_ADDR_WIDTH),
                // "B" side parameters
                .ADDR_WIDTH_B           (MEM_ADDR_WIDTH),
                .RST_MODE_B             ("SYNC"),
                .READ_RESET_VALUE_B     ("0"),
                .READ_LATENCY_B         (1),
                .READ_DATA_WIDTH_B      (C_ADDR_WIDTH),
                .WRITE_MODE_B           ("read_first"),
                // Common parameters
                .CLOCKING_MODE          ("common_clock"),
                .CASCADE_HEIGHT         (0),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_NUM_QUEUES * C_ADDR_WIDTH),
                .MESSAGE_CONTROL        (0),
                .AUTO_SLEEP_TIME        (0),
                .SIM_ASSERT_CHK         (0),
                .USE_EMBEDDED_CONSTRAINT(0),
                .USE_MEM_INIT           (1),
                .WAKEUP_TIME            ("disable_sleep")
            ) u_wr_addr_mem (
                .clka          (core_clk),
                .ena           (wr_addr_mem_wren),
                .wea           (wr_addr_mem_wren),
                .addra         (wr_addr_mem_qpid),
                .dina          (wr_addr_mem_wrdata),
                .injectdbiterra(1'b0),
                .injectsbiterra(1'b0),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .addrb         (i_wr_addr_qnum),
                .enb           (i_wr_addr_rd_en),
                .regceb        (1'b1),
                .doutb         (wr_addr_int),
                .sleep         (1'b0),
                .dbiterrb      (),
                .sbiterrb      ()
            );
            `MSFF_R(i_wr_addr_rd_en_q, i_wr_addr_rd_en, core_clk, core_rst)
            `MSFF_R(o_wr_addr_rd_valid, i_wr_addr_rd_en_q, core_clk, core_rst)
            `MSFF_R(i_wr_addr_qnum_q, i_wr_addr_qnum, core_clk, core_rst)
            `MSFF_R(o_wr_addr, (wr_addr_int + i_wr_base_addr), core_clk, core_rst)
        end else begin : no_enable_wr_addr
            assign o_wr_addr          = {C_ADDR_WIDTH{1'b0}};
            assign o_wr_addr_rd_valid = 1'b0;
        end

        if (C_ENABLE_RD_ADDR == 1) begin : enable_rd_addr

            assign rd_addr_mem_push_data = (push_rddata.rd_ptr[C_PTR_WIDTH-1:0] + 1'b1);
            assign in_rd_base_addr       = {C_ADDR_WIDTH{1'b0}};

            always_comb begin
                if (!init_done) begin
                    rd_addr_mem_wren   = init_mem_wren;
                    rd_addr_mem_qpid   = init_mem_wraddr;
                    rd_addr_mem_wrdata = init_mem_addr_wrdata;
                end else if (same_q_push_pop_q) begin
                    rd_addr_mem_wren   = 1'b1;
                    rd_addr_mem_qpid   = push_qpid;
                    rd_addr_mem_wrdata = ((rd_addr_mem_push_data << ADDR_SHIFT) + in_rd_base_addr);
                end else begin
                    case (pop_curr_st)
                        // normal pop : write when there's a pop
                        POP_RD: begin
                            if (i_q_flush & !i_q_pop & !i_q_push & (push_curr_st == PUSH_RD)) begin
                                rd_addr_mem_wren   = 1'b1;
                                rd_addr_mem_qpid   = i_q_flush_qpid;
                                rd_addr_mem_wrdata = ({C_ADDR_WIDTH{1'b0}} + in_rd_base_addr);
                            end else begin
                                rd_addr_mem_wren   = 1'b0;
                                rd_addr_mem_qpid   = {LOG2_NUM_QUEUES{1'b0}};
                                rd_addr_mem_wrdata = rd_addr_mem_wrdata_q;
                            end
                        end
                        POP_WR: begin  // 保存更新后的读指针，若是pop了一个条目，则读指针会递增，指向下一个pop的位置
                            rd_addr_mem_wren   = 1'b1;
                            rd_addr_mem_qpid   = pop_mem_wraddr;
                            rd_addr_mem_wrdata = ((pop_wrdata.rd_ptr[C_PTR_WIDTH-1:0] << ADDR_SHIFT) + in_rd_base_addr);
                        end
                        POP_GEN_RD_ADDR: begin
                            // if in T0, there's push and in T1 there's pop, then depend
                            // on push memory read to generate this
                            rd_addr_mem_wren   = 1'b1;
                            rd_addr_mem_qpid   = pop_qpid;
                            rd_addr_mem_wrdata = ((push_wrdata.rd_ptr[C_PTR_WIDTH-1:0] << ADDR_SHIFT) + in_rd_base_addr);
                        end
                        default: begin
                            rd_addr_mem_wren   = 1'b0;
                            rd_addr_mem_qpid   = {LOG2_NUM_QUEUES{1'b0}};
                            rd_addr_mem_wrdata = rd_addr_mem_wrdata_q;
                        end
                    endcase
                end
            end

            `MSFF_R(rd_addr_mem_wrdata_q, rd_addr_mem_wrdata, core_clk, core_rst)

            // 保存读指针
            xpm_memory_sdpram #(
                .ADDR_WIDTH_A           (MEM_ADDR_WIDTH),
                .RST_MODE_A             ("SYNC"),
                .BYTE_WRITE_WIDTH_A     (C_ADDR_WIDTH),
                .WRITE_DATA_WIDTH_A     (C_ADDR_WIDTH),
                // "B" side parameters
                .ADDR_WIDTH_B           (MEM_ADDR_WIDTH),
                .RST_MODE_B             ("SYNC"),
                .READ_RESET_VALUE_B     ("0"),
                .READ_LATENCY_B         (1),
                .READ_DATA_WIDTH_B      (C_ADDR_WIDTH),
                .WRITE_MODE_B           ("read_first"),
                // Common parameters
                .CLOCKING_MODE          ("common_clock"),
                .CASCADE_HEIGHT         (0),
                .ECC_MODE               ("no_ecc"),
                .MEMORY_INIT_FILE       ("none"),
                .MEMORY_INIT_PARAM      ("0"),
                .MEMORY_OPTIMIZATION    ("true"),
                .MEMORY_PRIMITIVE       ("auto"),
                .MEMORY_SIZE            (C_NUM_QUEUES * C_ADDR_WIDTH),
                .MESSAGE_CONTROL        (0),
                .AUTO_SLEEP_TIME        (0),
                .SIM_ASSERT_CHK         (0),
                .USE_EMBEDDED_CONSTRAINT(0),
                .USE_MEM_INIT           (1),
                .WAKEUP_TIME            ("disable_sleep")
            ) u_rd_addr_mem (
                .clka          (core_clk),
                .ena           (rd_addr_mem_wren),
                .wea           (rd_addr_mem_wren),
                .addra         (rd_addr_mem_qpid),
                .dina          (rd_addr_mem_wrdata),
                .injectdbiterra(1'b0),
                .injectsbiterra(1'b0),
                .clkb          (core_clk),
                .rstb          (core_rst),
                .addrb         (i_rd_addr_qnum),
                .enb           (i_rd_addr_rd_en),
                .regceb        (1'b1),
                .doutb         (rd_addr_int),
                .sleep         (1'b0),
                .dbiterrb      (),
                .sbiterrb      ()
            );

            `MSFF_R(i_rd_addr_rd_en_q, i_rd_addr_rd_en, core_clk, core_rst)
            `MSFF_R(o_rd_addr_rd_valid, i_rd_addr_rd_en_q, core_clk, core_rst)
            `MSFF_R(i_rd_addr_qnum_q, i_rd_addr_qnum, core_clk, core_rst)
            `MSFF_R(rd_addr_mem_wren_q, rd_addr_mem_wren, core_clk, core_rst)
            if (C_RDADDR_INTF == 0) begin  // current
                // 保证是最新的读地址
                `MSFF_R(o_rd_addr, ((rd_addr_mem_wren | rd_addr_mem_wren_q) ? (i_rd_base_addr + rd_addr_mem_wrdata) : (i_rd_base_addr + rd_addr_int)), core_clk, core_rst)
            end else begin
                `MSFF_R(o_rd_addr, (i_rd_base_addr + rd_addr_int), core_clk, core_rst)
            end
            //`MSFF_R (o_rd_addr,          (rd_addr_int + i_rd_base_addr), core_clk, core_rst)
        end else begin : no_enable_rd_addr
            assign o_rd_addr          = {C_ADDR_WIDTH{1'b0}};
            assign o_rd_addr_rd_valid = 1'b0;
        end
    endgenerate

endmodule  // q_mgr_init_top
