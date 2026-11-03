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
// rnic_config_reg.v
//-----------------------------------------------------------------------------
//
// Filename        : qp_mgr_cache.v
// Version         : v1.0
// Description     : QP manager caching logic. Fetches the Send WQE from DDR
//                   QP 管理器缓存逻辑。从 DDR 抓取发送 WQE
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- qp mgr top
//                     -- qp_mgr_cache.v
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
// qp_mgr_cache 负责控制 axi master(qp_mgr_axi_master模块) 从 DDR 读取仲裁出的 QP 的 SQ 条目，并在读出后更新 QP 的 SQ 状态
// 
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module qp_mgr_cache #(
    parameter C_NUM_QP           = 128,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter WQE_FIFO_IDX_WIDTH = 2,
    parameter WQE_FIFO_DEPTH     = 4,
    parameter C_QP_INDX_WIDTH    = 7
) (
    input wire core_clk,
    input wire core_rstn,

    output wire [ 2:0] o_fsm_status,
    output wire [15:0] o_stat_num_wqe_cnt, // 处理的 WQE 数量

    // Configuration details
    input  wire                          i_sw_override_en,
    input  wire                          i_xrnic_en,
    input  wire                          i_sq_empty_and_reduce,          // 所有 QP 的 SQ 当前是否都为空
    input  wire                          i_osq_almost_full_set_valid,
    input  wire [C_QP_INDX_WIDTH -1 : 0] i_osq_almost_full_set_id,
    input  wire                          i_osq_almost_full_reset_valid,
    input  wire [C_QP_INDX_WIDTH -1 : 0] i_osq_almost_full_reset_id,
    input  wire [C_QP_INDX_WIDTH -1 : 0] i_qp_sq_arbitrated_idx,         // 仲裁后 QP 号
    input  wire                          i_wqe_fifo_full,                // SQ WQE FIFO满信号
    input  wire [  WQE_FIFO_IDX_WIDTH:0] i_num_valid_entries,            // SQ WQE FIFO中有效条目的数量
    input  wire                          i_arbitration_done,             // 仲裁完成
    output wire                          o_arbitrate,                    // 发起仲裁请求
    output wire                          o_sample_arbitrated_sq,         // 允许采样仲裁结果

    output wire o_status_upd_needed,

    // SQ 基地址
    output wire                          o_qp_sq_ba_req,
    input  wire                          i_qp_sq_ba_valid,
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_qp_sq_ba,

    // SQ 深度
    output wire        o_qp_sq_depth_req,
    input  wire        i_qp_sq_depth_valid,
    input  wire [15:0] i_qp_sq_depth,

    // SQ PI DB
    output wire        o_qp_sq_pidb_req,
    input  wire        i_qp_sq_pidb_valid,
    input  wire [15:0] i_qp_sq_pidb,

    // 已经决定要取这个 QP 的一个 SQE，并且 AXI 地址握手也完成了，这里就要更新相应状态了
    input  wire        i_qp_curr_sqptr_valid,
    output wire        o_qp_curr_sqptr_rdreq,
    input  wire [15:0] i_qp_curr_sqptr_proc,
    output wire        o_update_curr_sqptr,    // sq 消费指针更新标志
    output wire [15:0] o_qp_curr_sqptr,        // sq 消费指针

    input  wire        i_qp_stat_wqe_cnt_valid,
    output wire        o_qp_stat_wqe_cnt_rdreq,
    input  wire [15:0] i_qp_stat_wqe_cnt,
    output wire        o_update_qp_stat_wqe_cnt,
    output wire [15:0] o_qp_stat_wqe_cnt,

    // Halt asserted for retransmission -- 断言停止重传
    input  wire i_halt,
    output wire o_halted,

    // AXI master module interface
    // 读 DDR 中 SQ 的 WQE
    output wire                           o_axi_ren_valid,     // 告诉 AXI master“现在可以对当前仲裁 QP 发一次 DDR 读”
    output wire [    C_QP_INDX_WIDTH-1:0] o_wqe_qpid,          // 这次要读的 WQE 属于哪个 QP
    output wire [C_M_AXI_ADDR_WIDTH -1:0] o_axi_read_addr,     // 本次 WQE 的 DDR 地址（请求地址）
    input  wire                           i_wqe_addr_latched,  // AXI AR 通道这次请求地址已经被 DDR 侧接受，即一次 AXI 读地址握手成功，就是发出了一个“在途读”
    input  wire                           i_wqe_dataread,      // AXI R 通道有一拍有效数据返回（本工程里 WQE 64B，通常单拍 512b），就是“在途读”完成返回

    input wire                       init_start,
    input wire [C_QP_INDX_WIDTH-1:0] init_count,
    input                            i_prog_full_db_update_curr_sqptr  // 用于 PUSH_SQ_UPD_CURR_SQPTR 状态的回压信号（下游几乎满/暂不可接收）

);
    `include "../../rdma_macros.vh"


    localparam SQ_ENTRY_SIZE = 64;  // 64 Bytes


    localparam IDLE = 3'b000;
    localparam ARBITRATE_SQ = 3'b001;
    localparam FETCH_SQ_ENTRY = 3'b010;
    localparam PUSH_SQ_UPD_CURR_SQPTR = 3'b011;
    localparam GET_ELEM_ADDR = 3'b100;
    localparam GET_ARBITRATED_SQ = 3'b101;
    localparam HALTED = 3'b110;
    localparam CHECK_SQ_EMPTY = 3'b111;

    reg  [                    2:0] cache_ns;
    reg  [                    2:0] cache_cs;
    reg  [                    2:0] cache_cs_r;
    reg  [                   15:0] qp_sq_depth_ff;
    reg  [                   15:0] qp_sq_pidb_ff;
    reg  [                   15:0] qp_stat_wqe_cnt_ff;
    reg  [ C_M_AXI_ADDR_WIDTH-1:0] qp_sq_ba_ff;
    reg  [                   15:0] qp_curr_sqptr_proc_ff;
    reg  [                   15:0] num_global_wqe_cnt;

    reg  [                    2:0] new_fsm_cs;
    reg  [                    2:0] new_fsm_ns;

    wire [                   15:0] new_qp_stat_wqe_cnt;
    wire [                   15:0] new_curr_sqptr_proc;
    wire [                   15:0] curr_sqptr_base_0;
    wire [C_M_AXI_ADDR_WIDTH -1:0] sq_read_addr_to_axi;
    wire                           sq_empty;
    reg  [   WQE_FIFO_IDX_WIDTH:0] wqe_reads_in_pipe;  // 已经发出读地址且被AXI Master接收，但 WQE 数据还没回来的 WQE 读请求数量
    wire [   WQE_FIFO_IDX_WIDTH:0] total_wqes_in_system;  // 系统当前“占位中的 WQE 总数估计”
    wire                           i_osq_almost_full_rd_data;
    wire                           i_osq_almost_full_rd_data_int;
    reg                            osq_almost_full_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_almost_full_id;
    reg                            osq_almost_full_data;
    reg                            osq_almost_full_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_almost_full_id_pending;

    // i_num_valid_entries 是 SQ WQE FIFO 里已缓存但未消费的条目数
    assign total_wqes_in_system = wqe_reads_in_pipe + i_num_valid_entries;

    // 通常情况下，CQ 门铃值（用于 SQ）并不真正使用。
    // curr_sqptr 值与 SQ PI DB 一起用于确定 FIFO 满/空状态。
    // 但是，如果出现错误，需要重新传输，QP 缓存模块会将 curr_sqptr 值移动到 CQ DB 值中。
    // 预计这种情况不会经常发生。
    // 目前，响应处理程序模块中的逻辑会等待缓存管理器接受重传，因此流水线功能不强。
    // 如果重传发生的频率很低，理想情况下这应该不是问题。

    //############################################################################################
    //假设：i_osq_almost_full_set_valid 和 i_osq_almost_full_reset_valid 连续断言之间至少有 1 个时钟间隔
    //############################################################################################
    always @(posedge core_clk) begin
        if (!core_rstn) begin
            osq_almost_full_valid         <= 1'b0;
            osq_almost_full_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_almost_full_data          <= 1'b0;
            osq_almost_full_valid_pending <= 1'b0;
            osq_almost_full_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_osq_almost_full_set_valid) begin  // 置1
                osq_almost_full_valid <= 1'b1;
                osq_almost_full_id    <= i_osq_almost_full_set_id;
                osq_almost_full_data  <= 1'b1;
            end else if (i_osq_almost_full_reset_valid) begin  // 清0
                osq_almost_full_valid <= 1'b1;
                osq_almost_full_id    <= i_osq_almost_full_reset_id;
                osq_almost_full_data  <= 1'b0;
            end else if (osq_almost_full_valid_pending) begin
                osq_almost_full_valid <= 1'b1;
                osq_almost_full_id    <= osq_almost_full_id_pending;
                osq_almost_full_data  <= 1'b0;
            end else begin
                osq_almost_full_valid <= 1'b0;
            end

            if (i_osq_almost_full_set_valid && i_osq_almost_full_reset_valid) begin  // 同拍冲突消化
                osq_almost_full_valid_pending <= 1'b1;
                osq_almost_full_id_pending    <= i_osq_almost_full_reset_id;
            end else begin
                osq_almost_full_valid_pending <= 1'b0;
            end
        end
    end

    // osq 将满状态 lut sdpram
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_almost_full (
        .core_clk(core_clk),
        .core_rst(!core_rstn),
        .wr_en   (init_start || osq_almost_full_valid),
        .wr_addr (init_start ? init_count : osq_almost_full_id),
        .wr_data (init_start ? 1'b0 : osq_almost_full_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_sq_arbitrated_idx),
        .rd_data (i_osq_almost_full_rd_data_int)
    );
    // 如果当前有 pending reset，且读的正好是这个 QP，就强制输出 0，避免“RAM 还没写回，下游先看到旧的 1”这个一拍误判
    assign i_osq_almost_full_rd_data = (osq_almost_full_valid_pending && osq_almost_full_id_pending == i_qp_sq_arbitrated_idx) ? 1'b0 : i_osq_almost_full_rd_data_int;



    always @(posedge core_clk) begin
        if (~core_rstn) begin
            cache_cs <= IDLE;
        end else if (!i_xrnic_en) begin
            cache_cs <= IDLE;
        end else begin
            cache_cs <= cache_ns;
        end
    end

    always @(posedge core_clk) begin
        if (!core_rstn) begin
            cache_cs_r <= IDLE;
        end else begin
            cache_cs_r <= cache_cs;
        end
    end

    always @(*) begin
        cache_ns = cache_cs;
        case (cache_cs)

            IDLE: begin
                if (i_sw_override_en) begin
                    cache_ns = IDLE;
                end//
                // 

                else if (i_halt) begin
                    cache_ns = HALTED;
                end// 

                // 所有 QP 有 SQ 不为空 && SQ WQE FIFO 不满 && 系统估计WQE数量<SQ WQE FIFO最大深度
                else if (!(i_sq_empty_and_reduce) && !i_wqe_fifo_full && (total_wqes_in_system < WQE_FIFO_DEPTH)) begin
                    cache_ns = GET_ARBITRATED_SQ;  // 选取 SQ 仲裁
                end
                // Q：为什么有“系统估计WQE数量<SQ WQE FIFO最大深度”？
                // A：因为取出来的WQE要放在SQ WQE FIFO中，而SQ WQE FIFO的深度是有限的
            end

            // 默认情况下，仲裁器将工作并获得有效的 SQ，因此无需请求仲裁，如果所有 SQ 都为空，则返回 IDLE 状态。
            GET_ARBITRATED_SQ: begin
                if (i_sw_override_en) begin
                    cache_ns = IDLE;
                end else if (i_halt) begin
                    cache_ns = HALTED;
                end else if (i_arbitration_done) begin
                    cache_ns = ARBITRATE_SQ;
                end else if (i_sq_empty_and_reduce) begin
                    cache_ns = IDLE;
                end
            end

            // 在进入下一步之前，请仲裁器进行仲裁，并获取下一个有效的 SQ，以便在后台进行仲裁。
            // assign o_arbitrate = (cache_cs == ARBITRATE_SQ) ? 1'b1 : 1'b0;
            ARBITRATE_SQ: begin  // 此时 o_arbitrate 拉高
                if (i_osq_almost_full_rd_data) cache_ns = IDLE;
                else cache_ns = GET_ELEM_ADDR;
            end

            GET_ELEM_ADDR: begin  // 此时 i_arbitration_done 拉高
                if (i_qp_sq_ba_valid & i_qp_curr_sqptr_valid & i_qp_sq_depth_valid & i_qp_sq_pidb_valid & i_qp_stat_wqe_cnt_valid) begin
                    cache_ns = CHECK_SQ_EMPTY;
                end
            end

            // 由于 PI 和当前指针更新与 FIFO 空信号更新之间存在延迟，因此需要检查 SQ 是否为空
            CHECK_SQ_EMPTY: begin
                if (sq_empty) cache_ns = IDLE;
                else cache_ns = FETCH_SQ_ENTRY;
            end

            FETCH_SQ_ENTRY: begin
                //cache_ns = IDLE;
                //if (i_wqe_dataread) begin // data read back from DDR
                if (i_wqe_addr_latched) begin
                    cache_ns = PUSH_SQ_UPD_CURR_SQPTR;
                end
                //end
            end

            // 将 SQ WQE 推入 SQ WQE FIFO 并更新当前 SQ 消费指针
            PUSH_SQ_UPD_CURR_SQPTR: begin  // 提交本次取数结果并推进 SQ 消费指针
                cache_ns = i_prog_full_db_update_curr_sqptr ? PUSH_SQ_UPD_CURR_SQPTR : IDLE;  // 等待 fifo 可用
            end

            HALTED: begin
                if (~i_halt) cache_ns = IDLE;
            end
        endcase
    end

    always @(posedge core_clk) begin
        if (!core_rstn) begin
            wqe_reads_in_pipe <= 0;
        end else begin
            if (!i_wqe_dataread & i_wqe_addr_latched) begin
                wqe_reads_in_pipe <= wqe_reads_in_pipe + 1;
            end else if (i_wqe_dataread & !i_wqe_addr_latched) begin
                wqe_reads_in_pipe <= wqe_reads_in_pipe - 1;
            end else begin
                wqe_reads_in_pipe <= wqe_reads_in_pipe;
            end
        end
    end

    always @(posedge core_clk) begin
        if (!core_rstn) begin
            new_fsm_cs <= IDLE;
        end else begin
            new_fsm_cs <= new_fsm_ns;
        end
    end

    always @(*) begin
        new_fsm_ns = new_fsm_cs;
        case (new_fsm_cs)
            IDLE:
            if (i_wqe_dataread) begin
                new_fsm_ns = PUSH_SQ_UPD_CURR_SQPTR;
            end
            PUSH_SQ_UPD_CURR_SQPTR: new_fsm_ns = IDLE;
            default: new_fsm_ns = IDLE;
        endcase
    end




    `MSFF_R(qp_sq_ba_ff, (i_qp_sq_ba_valid ? i_qp_sq_ba : qp_sq_ba_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_sq_depth_ff, (i_qp_sq_depth_valid ? i_qp_sq_depth : qp_sq_depth_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_curr_sqptr_proc_ff, (i_qp_curr_sqptr_valid ? i_qp_curr_sqptr_proc : qp_curr_sqptr_proc_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_sq_pidb_ff, (i_qp_sq_pidb_valid ? i_qp_sq_pidb : qp_sq_pidb_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_stat_wqe_cnt_ff, (i_qp_stat_wqe_cnt_valid ? i_qp_stat_wqe_cnt : new_qp_stat_wqe_cnt), core_clk, ~core_rstn)


    `MSFF_R(num_global_wqe_cnt, ((cache_cs == PUSH_SQ_UPD_CURR_SQPTR && cache_cs_r != PUSH_SQ_UPD_CURR_SQPTR) ? (num_global_wqe_cnt + 1'b1) : num_global_wqe_cnt), core_clk, ~core_rstn)

    assign new_curr_sqptr_proc      = (qp_curr_sqptr_proc_ff == qp_sq_depth_ff) ? 16'b1 : (qp_curr_sqptr_proc_ff + 1'b1);

    assign new_qp_stat_wqe_cnt      = ((cache_cs == FETCH_SQ_ENTRY) & i_wqe_dataread) ? qp_stat_wqe_cnt_ff + 1'b1 : qp_stat_wqe_cnt_ff;
    //assign new_qp_stat_wqe_cnt = ((new_fsm_cs == IDLE) & i_wqe_dataread) ? qp_stat_wqe_cnt_ff + 1'b1 : qp_stat_wqe_cnt_ff;

    // 计算要取的 SQ WQE DDR地址
    assign curr_sqptr_base_0        = (qp_curr_sqptr_proc_ff == qp_sq_depth_ff) ? 16'b0 : qp_curr_sqptr_proc_ff;
    assign sq_read_addr_to_axi      = qp_sq_ba_ff + (curr_sqptr_base_0 * SQ_ENTRY_SIZE);

    assign sq_empty                 = (qp_sq_pidb_ff == qp_curr_sqptr_proc_ff) ? 1'b1 : 1'b0;
    assign o_qp_stat_wqe_cnt        = qp_stat_wqe_cnt_ff;

    // 有时，如果在更新后的一个时钟周期内发生清除操作，状态更新会被覆盖。
    // 请确保如果 SQ 为空状态不正确，则应将其显示为待处理状态。
    assign o_status_upd_needed      = 1'b0;  //(cache_cs == CHECK_SQ_EMPTY); 

    assign o_axi_read_addr          = sq_read_addr_to_axi;
    assign o_qp_curr_sqptr          = new_curr_sqptr_proc;

    assign o_axi_ren_valid          = ((cache_cs == CHECK_SQ_EMPTY) & ~sq_empty) ? 1'b1 : 1'b0;
    assign o_qp_sq_ba_req           = (cache_cs == GET_ELEM_ADDR) ? 1'b1 : 1'b0;
    assign o_qp_sq_depth_req        = (cache_cs == GET_ELEM_ADDR) ? 1'b1 : 1'b0;
    assign o_qp_sq_pidb_req         = (cache_cs == GET_ELEM_ADDR) ? 1'b1 : 1'b0;
    assign o_qp_curr_sqptr_rdreq    = (cache_cs == GET_ELEM_ADDR) ? 1'b1 : 1'b0;
    assign o_qp_stat_wqe_cnt_rdreq  = (cache_cs == GET_ELEM_ADDR) ? 1'b1 : 1'b0;
    assign o_update_curr_sqptr      = (cache_cs == PUSH_SQ_UPD_CURR_SQPTR) ? 1'b1 : 1'b0;
    assign o_update_qp_stat_wqe_cnt = (cache_cs == PUSH_SQ_UPD_CURR_SQPTR && cache_cs_r != PUSH_SQ_UPD_CURR_SQPTR) ? 1'b1 : 1'b0;
    assign o_arbitrate              = (cache_cs == ARBITRATE_SQ) ? 1'b1 : 1'b0;
    assign o_sample_arbitrated_sq   = (cache_cs == GET_ARBITRATED_SQ) ? 1'b1 : 1'b0;
    assign o_halted                 = (cache_cs == HALTED && wqe_reads_in_pipe == 0) ? 1'b1 : 1'b0;
    assign o_fsm_status             = cache_cs;
    assign o_stat_num_wqe_cnt       = num_global_wqe_cnt;
    assign o_wqe_qpid               = i_qp_sq_arbitrated_idx;

    //-------------------------------------------------------------------------------------
    // 功能：查看FSM状态
    // 不可综合的代码
    //-------------------------------------------------------------------------------------

`ifdef SIMULATION
    reg [20*8-1:0] cache_cs_string = "null";
    reg [20*8-1:0] cache_ns_string = "null";

    always @* begin
        case (cache_cs)
            IDLE:                   cache_cs_string = "IDLE";
            ARBITRATE_SQ:           cache_cs_string = "ARBITRATE_SQ";
            GET_ARBITRATED_SQ:      cache_cs_string = "GET_ARBITRATED_SQ";
            FETCH_SQ_ENTRY:         cache_cs_string = "FETCH_SQ_ENTRY";
            CHECK_SQ_EMPTY:         cache_cs_string = "CHECK_SQ_EMPTY";
            PUSH_SQ_UPD_CURR_SQPTR: cache_cs_string = "PUSH_SQ_UPD_CURR_SQPTR";
            GET_ELEM_ADDR:          cache_cs_string = "GET_ELEM_ADDR";
        endcase
        case (cache_ns)
            IDLE:                   cache_ns_string = "IDLE";
            ARBITRATE_SQ:           cache_ns_string = "ARBITRATE_SQ";
            GET_ARBITRATED_SQ:      cache_ns_string = "GET_ARBITRATED_SQ";
            FETCH_SQ_ENTRY:         cache_ns_string = "FETCH_SQ_ENTRY";
            CHECK_SQ_EMPTY:         cache_ns_string = "CHECK_SQ_EMPTY";
            PUSH_SQ_UPD_CURR_SQPTR: cache_ns_string = "PUSH_SQ_UPD_CURR_SQPTR";
            GET_ELEM_ADDR:          cache_ns_string = "GET_ELEM_ADDR";
        endcase
    end
`endif

    //-------------------------------------------------------------------------------------

endmodule
