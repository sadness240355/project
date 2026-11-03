`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_qp_wqe_cache — QP 管理器缓存控制核心模块
//
// 【模块概述】
// 负责协调从 DDR 读取 WQE 的完整流程，是 QP 管理器中连接仲裁器与 AXI Master
// 的桥梁。控制 lrdma_qp_mgr_axi_master 从 DDR 读取仲裁出的 QP 的 SQ 条目，
// 并在读出后更新 QP 的 SQ 消费指针等状态。
//
// 【主 FSM 工作流程】
// IDLE → GET_ARBITRATED_SQ → ARBITRATE_SQ → GET_ELEM_ADDR →
// CHECK_SQ_EMPTY → FETCH_SQ_ENTRY → PUSH_SQ_UPD_CURR_SQPTR → IDLE
// 另有 HALTED 状态用于重传场景暂停。
//
// 【详细流程】
// 1. IDLE：检测 SQ 非空、WQE FIFO 未满、WQE 在途数未超限
// 2. GET_ARBITRATED_SQ：等待轮询仲裁器完成
// 3. ARBITRATE_SQ：触发下一次后台仲裁（预取优化）
// 4. GET_ELEM_ADDR：请求 QP 的 SQ 基地址、深度、PI 门铃、消费指针等配置
// 5. CHECK_SQ_EMPTY：二次检查 PI 门铃 vs 消费指针，排除延迟导致的假非空
// 6. FETCH_SQ_ENTRY：驱动 AXI Master 发起读请求（地址 = SQ 基地址 + 消费指针 × 64）
// 7. PUSH_SQ_UPD_CURR_SQPTR：更新消费指针（环绕归零）和 WQE 计数
//
// 【流控机制】
// - wqe_reads_in_pipe 计数器跟踪在途读请求数
// - i_halt / o_halted 握手支持重传场景暂停正常取数
// 
// OSQ full 状态检查通过 LUT RAM 查询。
// lrdma_qp_mgr_top 接入 inst_osq_manager 的 osq_full_set/reset 事件，
// 这里维护的是 per-QP OSQ full 状态，用于仲裁结果进入配置读取前的二次检查。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_qp_wqe_cache #(
    parameter C_NUM_QP           = 128,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter WQE_FIFO_IDX_WIDTH = 2,
    parameter WQE_FIFO_DEPTH     = 4,
    parameter C_QP_INDX_WIDTH    = 7,
    parameter C_ENABLE_SAME_SQ_FAST_FETCH = 1
) (
    input core_clk,
    input core_rstn,

    output [ 2:0] o_fsm_status,
    output [15:0] o_stat_num_wqe_cnt, // 处理的 WQE 数量

    // Configuration details
    input                           i_sw_override_en,
    input                           i_lrdma_en,
    input                           i_sq_empty_and_reduce,   // 所有 QP 的 SQ 是否都为空
    input                           i_osq_full_set_valid,
    input  [C_QP_INDX_WIDTH -1 : 0] i_osq_full_set_id,
    input                           i_osq_full_reset_valid,
    input  [C_QP_INDX_WIDTH -1 : 0] i_osq_full_reset_id,
    input  [C_QP_INDX_WIDTH -1 : 0] i_qp_sq_arbitrated_idx,  // 仲裁后 QP 号
    input                           i_wqe_fifo_full,         // SQ WQE FIFO满信号
    input  [  WQE_FIFO_IDX_WIDTH:0] i_num_valid_entries,     // SQ WQE FIFO中有效条目的数量
    input                           i_arbitration_done,      // 仲裁完成
    output                          o_arbitrate,             // 发起仲裁请求
    output                          o_sample_arbitrated_sq,  // 允许采样仲裁结果

    // SQ 基地址
    output                          o_qp_sq_ba_req,
    input                           i_qp_sq_ba_valid,
    input  [C_M_AXI_ADDR_WIDTH-1:0] i_qp_sq_ba,

    // SQ 深度
    output        o_qp_sq_depth_req,
    input         i_qp_sq_depth_valid,
    input  [15:0] i_qp_sq_depth,

    // SQ PI DB
    output        o_qp_sq_pidb_req,
    input         i_qp_sq_pidb_valid,
    input  [15:0] i_qp_sq_pidb,

    // 已经决定要取这个 QP 的一个 SQE，并且 AXI 地址握手也完成了，这里就要更新相应状态了
    input         i_qp_curr_sqptr_valid,
    output        o_qp_curr_sqptr_rdreq,
    input  [15:0] i_qp_curr_sqptr_proc,
    output        o_update_curr_sqptr,    // sq 消费指针更新标志
    output [15:0] o_qp_curr_sqptr,        // sq 消费指针

    input         i_qp_stat_wqe_cnt_valid,
    output        o_qp_stat_wqe_cnt_rdreq,
    input  [15:0] i_qp_stat_wqe_cnt,
    output        o_update_qp_stat_wqe_cnt,
    output [15:0] o_qp_stat_wqe_cnt,

    // 触发 i_halt → 重传
    input  i_halt,
    output o_halted,

    // AXI master module interface
    // 读 DDR 中 SQ 的 WQE
    output                           o_axi_ren_valid,     // 通知 AXI master 可对仲裁 QP 发一次 DDR 读
    output [    C_QP_INDX_WIDTH-1:0] o_wqe_qpid,          // 这次要读的 WQE 属于哪个 QP
    output [C_M_AXI_ADDR_WIDTH -1:0] o_axi_read_addr,     // 本次 WQE 的 DDR 地址（请求地址）
    input                            i_wqe_addr_latched,  // AXI AR 通道这次请求地址已经被 DDR 侧接受，即一次 AXI 读地址握手成功，就是发出了一个“在途读”
    input                            i_wqe_dataread,      // AXI R 通道有一拍有效数据返回，就是“在途读”完成返回，即data read back from DDR

    input                       init_start,
    input [C_QP_INDX_WIDTH-1:0] init_count,
    input                       i_prog_full_db_update_curr_sqptr  // 用于 PUSH_SQ_UPD_CURR_SQPTR 状态的回压信号（下游几乎满或正在处理）

);
    `include "lrdma_defs.vh"


    localparam SQ_ENTRY_SIZE = 64;  // 64 Bytes


    //==========================================================================
    // 主 FSM 状态（3-bit 普通二进制编码）
    //
    // 时间线一览（一次正常取数）：
    //   IDLE → GET_ARBITRATED_SQ → ARBITRATE_SQ → GET_ELEM_ADDR
    //        → CHECK_SQ_EMPTY → FETCH_SQ_ENTRY → PUSH_SQ_UPD_CURR_SQPTR → IDLE
    // 重传分支：IDLE 收到 i_halt → HALTED → 等 i_halt 撤销 → IDLE
    //==========================================================================
    localparam IDLE = 3'b000;  // 空闲。三大门槛都通过（SQ 非空 + WQE FIFO 不满 + 系统在途+缓存条目数 < 深度）才往下走
    localparam ARBITRATE_SQ = 3'b001;  // 拉 o_arbitrate=1 触发"后台仲裁"，预先选好下一轮要读的 QP；同时检查 OSQ 是否满
    localparam FETCH_SQ_ENTRY = 3'b010;  // 等 lrdma_qp_mgr_axi_master 接住 AR 地址（i_wqe_addr_latched=1）；AR 握手后进入推进指针阶段
    localparam PUSH_SQ_UPD_CURR_SQPTR = 3'b011;  // 提交本次取数：消费指针 +1（到 depth 时回 1）、全局 WQE 计数 +1；prog_full 期间挂起不过连
    localparam GET_ELEM_ADDR = 3'b100;  // 一次性请求仲裁 QP 的 SQ_BA / depth / PI doorbell / 消费指针 / wqe_cnt，等所有 valid 到齐
    localparam GET_ARBITRATED_SQ = 3'b101;  // 等仲裁器给出 i_arbitration_done，同拍采样 i_qp_sq_arbitrated_idx
    localparam HALTED = 3'b110;  // 重传场景：i_halt=1 时驻留这里，不发新读请求；i_halt 撤销后回 IDLE
    localparam CHECK_SQ_EMPTY = 3'b111;  // 用最新读到的 PI、sqptr 多拍后复核一次 SQ 是否为空；同拍拉 o_axi_ren_valid 让 lrdma_qp_mgr_axi_master 发 AR，“假非空”则回 IDLE

    reg  [                    2:0] cache_ns;
    reg  [                    2:0] cache_cs;
    reg  [                    2:0] cache_cs_r;
    reg  [                   15:0] qp_sq_depth_ff;
    reg  [                   15:0] qp_sq_pidb_ff;
    reg  [                   15:0] qp_stat_wqe_cnt_ff;
    reg  [ C_M_AXI_ADDR_WIDTH-1:0] qp_sq_ba_ff;
    reg  [                   15:0] qp_curr_sqptr_proc_ff;
    reg  [                   15:0] num_global_wqe_cnt;

    wire [                   15:0] new_qp_stat_wqe_cnt;
    wire [                   15:0] new_curr_sqptr_proc;
    wire [                   15:0] curr_sqptr_base_0;
    wire [C_M_AXI_ADDR_WIDTH -1:0] sq_read_addr_to_axi;
    wire                           sq_empty;
    wire                           osq_full_block;
    wire                           same_sq_has_next_wqe;
    wire                           same_sq_fast_fetch_ok;
    reg  [   WQE_FIFO_IDX_WIDTH:0] wqe_reads_in_pipe;  // 已经发出读地址且被AXI Master接收，但 WQE 数据还没回来的 WQE 读请求数量
    wire [   WQE_FIFO_IDX_WIDTH:0] total_wqes_in_system;  // 系统内已占用 WQE 条目数估计
    wire                           i_osq_full_rd_data;
    wire                           i_osq_full_rd_data_int;
    reg                            osq_full_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_full_id;
    reg                            osq_full_data;
    reg                            osq_full_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_full_id_pending;

    // i_num_valid_entries 是 SQ WQE FIFO 里已缓存但未消费的条目数
    assign total_wqes_in_system = wqe_reads_in_pipe + i_num_valid_entries;

    // 通常情况下，CQ 门铃值（用于 SQ）并不真正使用。
    // curr_sqptr 值与 SQ PI DB 一起用于确定 FIFO 满/空状态。
    // 但是，如果出现错误，需要重新传输，QP 缓存模块会将 curr_sqptr 值移动到 CQ DB 值中。
    // 预计这种情况不会经常发生。
    // ACK 管理器等待缓存管理器接受重传请求，等待期间会限制后续取数流水线推进。

    //############################################################################################
    //假设：i_osq_full_set_valid 和 i_osq_full_reset_valid 连续断言之间至少有 1 个时钟间隔
    //############################################################################################
    always @(posedge core_clk) begin
        if (!core_rstn) begin
            osq_full_valid         <= 1'b0;
            osq_full_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_full_data          <= 1'b0;
            osq_full_valid_pending <= 1'b0;
            osq_full_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_osq_full_set_valid) begin  // 置1
                osq_full_valid <= 1'b1;
                osq_full_id    <= i_osq_full_set_id;
                osq_full_data  <= 1'b1;
            end else if (i_osq_full_reset_valid) begin  // 清0
                osq_full_valid <= 1'b1;
                osq_full_id    <= i_osq_full_reset_id;
                osq_full_data  <= 1'b0;
            end else if (osq_full_valid_pending) begin
                osq_full_valid <= 1'b1;
                osq_full_id    <= osq_full_id_pending;
                osq_full_data  <= 1'b0;
            end else begin
                osq_full_valid <= 1'b0;
            end

            if (i_osq_full_set_valid && i_osq_full_reset_valid) begin  // 同拍冲突消化
                osq_full_valid_pending <= 1'b1;
                osq_full_id_pending    <= i_osq_full_reset_id;
            end else begin
                osq_full_valid_pending <= 1'b0;
            end
        end
    end

    // osq 满状态 lut sdpram
    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_full (
        .core_clk(core_clk),
        .core_rst(!core_rstn),
        .wr_en   (init_start || osq_full_valid),
        .wr_addr (init_start ? init_count : osq_full_id),
        .wr_data (init_start ? 1'b0 : osq_full_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_sq_arbitrated_idx),
        .rd_data (i_osq_full_rd_data_int)
    );
    // 如果存在 pending reset，且读的正好是这个 QP，就强制输出 0，避免“RAM 还没写回，下游先看到旧的 1”这个一拍误判
    assign i_osq_full_rd_data = (osq_full_valid_pending && osq_full_id_pending == i_qp_sq_arbitrated_idx) ? 1'b0 : i_osq_full_rd_data_int;



    always @(posedge core_clk) begin
        if (~core_rstn) begin
            cache_cs <= IDLE;
        end else if (!i_lrdma_en) begin
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
                end else if (i_halt) begin
                    cache_ns = HALTED;
                end else if (!(i_sq_empty_and_reduce) && !i_wqe_fifo_full && (total_wqes_in_system < WQE_FIFO_DEPTH)) begin  // 所有 QP 有 SQ 不为空 && SQ WQE FIFO 不满 && 系统估计WQE数量<SQ WQE FIFO最大深度
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
                end else if (i_arbitration_done) begin  // 这里会采样仲裁结果
                    cache_ns = ARBITRATE_SQ;
                end else if (i_sq_empty_and_reduce) begin
                    cache_ns = IDLE;
                end
            end

            // 进入下一步前，请仲裁器进行仲裁，并获取下一个有效的 SQ，以便在后台完成下一轮仲裁。
            // assign o_arbitrate = (cache_cs == ARBITRATE_SQ) ? 1'b1 : 1'b0;
            ARBITRATE_SQ: begin  // 此时 o_arbitrate 拉高
                if (osq_full_block) cache_ns = IDLE;
                else cache_ns = GET_ELEM_ADDR;  // 获取 QP 相应配置
            end

            GET_ELEM_ADDR: begin  // 此时 i_arbitration_done 拉高
                if (i_qp_sq_ba_valid & i_qp_curr_sqptr_valid & i_qp_sq_depth_valid & i_qp_sq_pidb_valid & i_qp_stat_wqe_cnt_valid) begin
                    cache_ns = CHECK_SQ_EMPTY;
                end
            end

            // 由于 PI 和消费指针更新与 FIFO 空信号更新之间存在延迟，因此需要检查 SQ 是否为空
            CHECK_SQ_EMPTY: begin
                if (sq_empty || osq_full_block) cache_ns = IDLE;
                else cache_ns = FETCH_SQ_ENTRY;
            end

            FETCH_SQ_ENTRY: begin
                if (i_wqe_addr_latched) begin
                    cache_ns = PUSH_SQ_UPD_CURR_SQPTR;
                end
            end

            // 将 SQ WQE 推入 SQ WQE FIFO 并更新 SQ 消费指针
            PUSH_SQ_UPD_CURR_SQPTR: begin  // 提交本次取数结果并推进 SQ 消费指针
                if (i_prog_full_db_update_curr_sqptr) begin
                    cache_ns = PUSH_SQ_UPD_CURR_SQPTR;
                end else if (same_sq_fast_fetch_ok) begin
                    cache_ns = CHECK_SQ_EMPTY;
                end else begin
                    cache_ns = IDLE;
                end
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



    `MSFF_R(qp_sq_ba_ff, (i_qp_sq_ba_valid ? i_qp_sq_ba : qp_sq_ba_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_sq_depth_ff, (i_qp_sq_depth_valid ? i_qp_sq_depth : qp_sq_depth_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_sq_pidb_ff, (i_qp_sq_pidb_valid ? i_qp_sq_pidb : qp_sq_pidb_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_stat_wqe_cnt_ff, (i_qp_stat_wqe_cnt_valid ? i_qp_stat_wqe_cnt : new_qp_stat_wqe_cnt), core_clk, ~core_rstn)

    always @(posedge core_clk) begin
        if (!core_rstn) begin
            qp_curr_sqptr_proc_ff <= 16'd0;
        end else if (i_qp_curr_sqptr_valid) begin
            qp_curr_sqptr_proc_ff <= i_qp_curr_sqptr_proc;
        end else if ((cache_cs == PUSH_SQ_UPD_CURR_SQPTR) && !i_prog_full_db_update_curr_sqptr) begin
            qp_curr_sqptr_proc_ff <= new_curr_sqptr_proc;
        end
    end


    `MSFF_R(num_global_wqe_cnt, ((cache_cs == PUSH_SQ_UPD_CURR_SQPTR && cache_cs_r != PUSH_SQ_UPD_CURR_SQPTR) ? (num_global_wqe_cnt + 1'b1) : num_global_wqe_cnt), core_clk, ~core_rstn)

    assign new_curr_sqptr_proc      = (qp_curr_sqptr_proc_ff == qp_sq_depth_ff) ? 16'b1 : (qp_curr_sqptr_proc_ff + 1'b1);

    assign new_qp_stat_wqe_cnt      = (cache_cs == PUSH_SQ_UPD_CURR_SQPTR && cache_cs_r != PUSH_SQ_UPD_CURR_SQPTR) ? qp_stat_wqe_cnt_ff + 1'b1 : qp_stat_wqe_cnt_ff;

    // 计算要取的 SQ WQE DDR地址
    assign curr_sqptr_base_0        = (qp_curr_sqptr_proc_ff == qp_sq_depth_ff) ? 16'b0 : qp_curr_sqptr_proc_ff;
    assign sq_read_addr_to_axi      = qp_sq_ba_ff + (curr_sqptr_base_0 * SQ_ENTRY_SIZE);

    assign sq_empty                 = (qp_sq_pidb_ff == qp_curr_sqptr_proc_ff) ? 1'b1 : 1'b0;
    assign osq_full_block           = i_osq_full_rd_data ||
                                      (i_osq_full_set_valid && (i_osq_full_set_id == i_qp_sq_arbitrated_idx));
    assign same_sq_has_next_wqe     = (qp_sq_pidb_ff != new_curr_sqptr_proc);
    assign same_sq_fast_fetch_ok    = (C_ENABLE_SAME_SQ_FAST_FETCH != 0) &&
                                      same_sq_has_next_wqe && !i_sw_override_en && !i_halt &&
                                      !i_wqe_fifo_full && (total_wqes_in_system < WQE_FIFO_DEPTH) &&
                                      !osq_full_block;

    assign o_qp_stat_wqe_cnt        = new_qp_stat_wqe_cnt;  // 写回BRAM时用已递增的值

    // 有时，如果在更新后的一个时钟周期内发生清除操作，状态更新会被覆盖。
    // 请确保如果 SQ 为空状态不正确，则应将其显示为待处理状态。

    assign o_axi_read_addr          = sq_read_addr_to_axi;
    assign o_qp_curr_sqptr          = new_curr_sqptr_proc;

    assign o_axi_ren_valid          = ((cache_cs == CHECK_SQ_EMPTY) & ~sq_empty & ~osq_full_block) ? 1'b1 : 1'b0;
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

endmodule
