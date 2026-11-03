`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_qp_retransmit_ctrl — LRDMA QP 管理器重传控制核心 FSM
//
// 【模块角色】
// 驻留在 lrdma_qp_mgr_top 内部。当 lrdma_ack_mgr_fsm 检测到某 QP 需要 Go-Back-N
// 重传（NAK 或 Timeout）时，通过 i_retransmit_reqd + i_retransmit_qpid 启动本
// FSM，完成以下三件事：
//   1. 回退该 QP 的发送侧上下文：SQ PSN → i_psn_to_retry，SSN → i_ssn_to_retry，
//      SQ 消费指针 curr_sqptr → CQ head pointer（最后一次成功完成的 WQE 位置）
//   2. 清理已经进入 WQE FIFO 但尚未送入封装/发送的"在飞"WQE：逐条 pop → 若属于
//      重传 QP 则 DROP（并让 OSQ pop 同步释放条目），否则 push 回 FIFO 尾部
//   3. 写 retransmit_wqe_bram：更新 {toggle, retry_start_psn, completed_dma_len}，
//      供 TX 引擎侧在后续重传取 WQE 时知道“从哪个 PSN 开始、已经完成多少字节”
//
// 【主 FSM 状态转换】
//   IDLE
//    └─(i_retransmit_reqd)→ HALT_PROCESS
//                            └─(i_halted)→ GET_CONF   (读 CQ_HD / SQ_PSN / SSN)
//                                           └→ UPD_CONF (写回回退后的 PSN/SSN/sqptr)
//                                               ├─(prog_full=0)→ CLEAN_SWQE_FIFO
//                                               └─(prog_full=1)→ WAIT_FOR_PROG_FULL → CLEAN_SWQE_FIFO
//
// CLEAN_SWQE_FIFO 循环：
//   unclean != 0 → POP_SWQE → CHECK_SWQE
//   若 qpid 匹配：丢弃该 WQE，并通过 o_wqe_dropped 向 QP 管理器的 OSQ 缓冲
//   补一个 pop 事件。
//   若同拍 ACK 管理器已经对同一 QP 发出 i_osq_pop，则延后一拍产生
//   o_wqe_dropped，避免同一 QP 同拍双 pop。
//   若 qpid 不匹配：PUSH_SWQE → CLEAN_SWQE_FIFO。
//
// 【与 OSQ 的同步】
// WQE_DROPPED 必须等 i_osq_pop && i_osq_pop_qpid==qp_retry_idx 才离开，
// 确保"WQE FIFO 中丢弃一条"与"OSQ 弹出对应条目"一一对应，防止 OSQ/WQE FIFO
// 状态失配。
//
// 【与 lrdma_qp_wqe_cache 的同步】
// o_halt 在整个重传周期（非 IDLE）拉高，阻止 lrdma_qp_wqe_cache 继续往下游送 WQE；
// i_halted 表示 cache 已彻底停下，WQE FIFO 不再增加新条目。
//
// 【retransmit_wqe_bram 布局（57-bit）】
//   [56]     toggle：部分 WQE 跨重传周期的相位翻转标记
//                    当本次 completed_dma_len==0（retransmit_wqe_bram_din1[31:0]==0）
//                    意味着"上次重传点还没完成任何字节"，toggle 保持不变；
//                    否则 toggle 翻转，提示后续重新取回的 WQE 需要应用部分重传修正
//   [55:32]  retry_start_psn：这个字段保存的是“这个 WQE/OSQ 条目最早从哪个 PSN 开始”
//   [31:0]   completed_dma_len：已 ACK 部分完成的字节数（按 PMTU 精确计算）
//             completed_dma_len==0 时 TX 会按完整 WRITE 首包重发并携带 RETH；
//             completed_dma_len>0 时 QP 管理器标记 retransmit_wqe，TX 发 WRITE_MIDDLE/LAST 且不带 RETH。
//
// 【输入 i_retransmit_wqe_bram_din 为 56-bit（未含 toggle），本模块读出旧 toggle
//   决定是否翻转，再组合 57-bit 写回。】
///////////////////////////////////////////////////////////////////////////////

`timescale 1ps / 1ps
module lrdma_qp_retransmit_ctrl #(
    parameter C_NUM_QP             = 128,  // 支持的 QP 总数
    parameter C_WQE_FIFO_IDX_WIDTH = 3,    // WQE FIFO 索引位宽（depth = 2^N，含水位计数多 1 bit）
    parameter C_QP_INDX_WIDTH      = 7     // QP 索引位宽 = log2(C_NUM_QP)
) (
    input core_clk,  // 核心时钟
    input core_rstn, // 低有效同步复位

    //==========================================================================
    // 来自 lrdma_ack_mgr_fsm 的重传请求
    //==========================================================================
    input                           i_retransmit_reqd,          // 重传请求脉冲（需保持到 o_retransmit_accepted=1）
    input  [C_QP_INDX_WIDTH -1 : 0] i_retransmit_qpid,          // 需要重传的 QP ID
    input  [                55 : 0] i_retransmit_wqe_bram_din,  // {retry_start_psn[23:0]@[55:32], completed_dma_len[31:0]}
    output                          o_retransmit_accepted,      // = (retry_cs == IDLE)；空闲时可接收请求，流程结束后重新拉高
    input  [                  23:0] i_psn_to_retry,             // SQ PSN 回退目标（ACK 管理器计算：NAK=acknacked；Timeout=acknacked+1）
    input  [                  23:0] i_ssn_to_retry,             // SSN（Send Sequence Number）回退目标

    //==========================================================================
    // 与 WQE FIFO 的水位观测
    //==========================================================================
    input [C_WQE_FIFO_IDX_WIDTH : 0] i_num_valid_entries,  // WQE FIFO 有效条目数（比 IDX_WIDTH 多 1 bit，用于全满表示）

    //==========================================================================
    // CQ head pointer 读接口（决定回退 curr_sqptr 到哪）
    //==========================================================================
    output                        o_qp_cq_hdptr_req,    // 读请求
    input                         i_qp_cq_hdptr_valid,  // 读数据有效
    input  [                15:0] i_qp_cq_hdptr,        // CQ head pointer = 最后一次完成的 WQE 之后的位置
    output [C_QP_INDX_WIDTH -1:0] o_qp_cq_hdptr_idx,    // 读地址（目标 QP）

    //==========================================================================
    // 回退后的 SQ 消费指针写回
    //==========================================================================
    output [                15:0] o_qp_curr_sqptr_proc,      // 要写入的 SQ 消费指针 = qp_cq_hdptr_ff
    output                        o_qp_curr_sqptr_proc_wen,  // 在 UPD_CONF / WAIT_FOR_PROG_FULL 拉高
    output [C_QP_INDX_WIDTH -1:0] o_qp_curr_sqptr_proc_idx,  // 写地址

    //==========================================================================
    // 回退后的统计 PSN 写回（stat 寄存器，供软件观察）
    //==========================================================================
    output [                23:0] o_qp_stat_ret_sq_psn,       // = qp_sq_psn_ff（回退前的原 PSN，作为"已重传前的最后 PSN"统计）
    output                        o_qp_stat_ret_sq_psn_wen,
    output [C_QP_INDX_WIDTH -1:0] o_qp_stat_ret_sq_psn_idx,
    input                         i_qp_stat_ret_sq_psn_valid, // 写完成确认

    //==========================================================================
    // SQ PSN 读/写接口
    //==========================================================================
    output                        o_qp_sq_psn_req,    // 读请求（GET_CONF 读 SQ PSN 用于 stat；UPD_CONF 兼作写地址保持）
    input                         i_qp_sq_psn_valid,  // 读返回有效
    input  [                23:0] i_qp_sq_psn,        // SQ PSN 读返回值
    output                        o_qp_sq_psn_wrn,    // 写使能
    output [                23:0] o_qp_sq_psn,        // 要写入的 PSN = psn_to_retry_ff
    output [C_QP_INDX_WIDTH -1:0] o_qp_sq_psn_idx,    // 读/写地址

    //==========================================================================
    // SSN（Send Sequence Number）读/写接口
    //==========================================================================
    output [C_QP_INDX_WIDTH -1:0] o_qp_stat_ssn_idx,   // 读/写地址
    output [                23:0] o_qp_stat_ssn,       // 要写入的 SSN = ssn_to_retry_ff
    input  [                23:0] i_qp_stat_ssn,       // 发送侧 SSN 读返回
    output                        o_qp_stat_ssn_req,   // 读请求
    output                        o_qp_stat_ssn_wrn,   // 写使能
    input                         i_qp_stat_ssn_valid,

    //==========================================================================
    // WQE FIFO 接口（用于清洗在飞 WQE）
    //==========================================================================
    input  [511:0] i_wqe,            // FIFO 队头 WQE（pop 后一拍可用）
    output [511:0] o_wqe,            // 推回 FIFO 尾部的 WQE（= wqe_ff，延一拍）
    output         o_wqe_fifo_push,  // PUSH_SWQE 态拉高
    output         o_wqe_fifo_pop,   // POP_SWQE 态拉高

    //==========================================================================
    // 与 lrdma_qp_wqe_cache 的 halt 握手
    //==========================================================================
    output                          o_halt,         // 重传进行中 → 拉高，阻止 cache 继续送 WQE
    output [C_QP_INDX_WIDTH -1 : 0] o_halted_qpid,  // halt 的目标 QP（cache 需据此清自己的 ready 信号）
    input                           i_halted,       // cache 已彻底停下（管线排空）

    //==========================================================================
    // 与 OSQ（Outstanding SQ Queue）的同步
    //==========================================================================
    input                              i_osq_pop,          // OSQ 本拍 pop 脉冲
    input      [C_QP_INDX_WIDTH-1 : 0] i_osq_pop_qpid,     // pop 的 QP ID
    output reg                         o_wqe_dropped,      // 本 FSM 丢弃了一条 WQE → 通知 OSQ 同步 pop
    output     [C_QP_INDX_WIDTH-1 : 0] o_wqe_dropped_qpid, // = qp_retry_idx

    //==========================================================================
    // retransmit_wqe_bram（per-QP）读写接口
    //   写：unclean == 0（清洗完成） 时写入一次，更新 toggle + retry_start_psn + completed_dma_len
    //   读：i_retransmit_wqe_bram_dout[56] 旧 toggle，根据 completed_dma_len 决定是否翻转
    //==========================================================================
    output reg [                 56:0] o_retransmit_wqe_bram_din,   // 写数据 {toggle, retry_start_psn[23:0], completed_dma_len[31:0]}
    input      [                 56:0] i_retransmit_wqe_bram_dout,  // 读数据（同地址）
    output reg                         o_retransmit_wqe_bram_wen,   // 写使能（1 拍）
    output reg [C_QP_INDX_WIDTH-1 : 0] o_retransmit_wqe_bram_idx,   // 地址 = qp_retry_idx

    //==========================================================================
    // prog_full 联锁（防止 DB 更新 curr_sqptr 与回退写冲突）
    //==========================================================================
    input i_prog_full_db_update_curr_sqptr  // 1：有 doorbell 正在更新 curr_sqptr，需等待
);
    `include "lrdma_defs.vh"

    //==========================================================================
    // 主 FSM 状态（4-bit 普通二进制编码，没有特殊编码诉求）
    //
    // 时间线一览（一次重传从头到尾会按这个顺序走一遍）：
    //   IDLE → HALT_PROCESS → GET_CONF → UPD_CONF
    //        → [WAIT_FOR_PROG_FULL?] → CLEAN_SWQE_FIFO
    //        → ( POP_SWQE → CHECK_SWQE → {PUSH_SWQE | WQE_DROPPED} )*
    //        → CLEAN_SWQE_FIFO（unclean=0 时写 retransmit_wqe_bram）→ IDLE
    //==========================================================================
    localparam IDLE = 4'b0000;  // 空闲。等待 ACK 管理器拉起 i_retransmit_reqd 启动一次重传
    localparam HALT_PROCESS = 4'b0001;  // 通知 lrdma_qp_wqe_cache 暂停送 WQE，等它确认 i_halted=1（管线已排空）
    localparam GET_CONF = 4'b0010;  // 同时发出 3 路读：CQ head pointer / SQ PSN / SSN，等 3 个 valid 都回来
    localparam UPD_CONF = 4'b0011;  // 把回退值写回去：SQ PSN←psn_to_retry、SSN←ssn_to_retry、curr_sqptr←CQ_HD，并存档 stat_ret_sq_psn
    localparam WAIT_FOR_PROG_FULL = 4'b1001;  // 软件 doorbell 的 curr_sqptr 更新队列还没排空，先挂着等它消化，wen 维持高不变
    localparam CLEAN_SWQE_FIFO = 4'b0100;  // 清洗 WQE FIFO 的循环入口：还有未检查条目就去 POP，没了就写 bram 然后回 IDLE
    localparam POP_SWQE = 4'b0101;  // 从 WQE FIFO pop 一条（1 拍脉冲）
    localparam CHECK_SWQE = 4'b0111;  // 看队头 WQE 的 qpid（藏在 i_wqe[136+:C_QP_INDX_WIDTH]）；unclean 计数减 1
    localparam PUSH_SWQE = 4'b0110;  // qpid 不是要重传的目标 → 把这条 WQE 原样推回 FIFO 尾部（wqe_ff 已经打了 1 拍）
    localparam WQE_DROPPED = 4'b1000;  // qpid 命中目标 → 直接丢弃；并要等 OSQ 也对应 pop 一次，确保两边条目数同步

    //==========================================================================
    // 内部状态/上下文寄存器
    //==========================================================================
    reg [                     3:0] retry_ns;  // 下一状态（组合）
    reg [                     3:0] retry_cs;  // 状态寄存器
    reg [C_WQE_FIFO_IDX_WIDTH : 0] sq_fifo_unclean_entries_ff;  // 剩余未清洗 WQE 计数（寄存）
    reg [C_WQE_FIFO_IDX_WIDTH : 0] sq_fifo_unclean_entries;  // 下一拍值（组合）
    reg [                    15:0] qp_cq_hdptr_ff;  // 锁存的 CQ head pointer
    reg [                    23:0] qp_sq_psn_ff;  // 锁存的 回退前 SQ PSN（写入 stat_ret_sq_psn）
    reg [  C_QP_INDX_WIDTH -1 : 0] qp_retry_idx;  // 重传目标 QP id（整个流程的上下文索引）
    reg [                   511:0] wqe_ff;  // 打 1 拍的 WQE（用于 PUSH_SWQE 推回）
    reg [                    23:0] psn_to_retry_ff;  // 锁存的 i_psn_to_retry（IDLE→HALT 边沿）
    reg [                    23:0] ssn_to_retry_ff;  // 锁存的 i_ssn_to_retry
    reg [                    55:0] retransmit_wqe_bram_din1;  // 锁存的 56-bit 写入载荷（toggle 另算）

    // 状态观测信号
    reg [                    23:0] qp_stat_ssn_ff;  // 锁存的 回退前 SSN

    assign o_wqe_dropped_qpid = qp_retry_idx;

    //==========================================================================
    // 时序块：状态/输出寄存器 + retransmit_wqe_bram 写驱动 + o_wqe_dropped 产生
    //==========================================================================
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            retry_cs                  <= IDLE;
            o_wqe_dropped             <= 1'b0;
            o_retransmit_wqe_bram_din <= 'd0;
            o_retransmit_wqe_bram_idx <= 'd0;
            o_retransmit_wqe_bram_wen <= 1'b0;
            retransmit_wqe_bram_din1  <= 'd0;
        end else begin
            retry_cs <= retry_ns;

            // --- 入口锁存：IDLE 态下一拍将进 HALT_PROCESS 时，把 bram 写载荷 + 目标 QP 索引锁定
            if (retry_cs == IDLE && i_retransmit_reqd) begin
                retransmit_wqe_bram_din1  <= i_retransmit_wqe_bram_din;  // {retry_start_psn[23:0], completed_dma_len[31:0]}
                o_retransmit_wqe_bram_idx <= i_retransmit_qpid;
            end

            // --- 收尾写 retransmit_wqe_bram：所有 WQE 清洗完（unclean==0）的那一拍
            //   toggle 规则：
            //     completed_dma_len==0 → 保持旧 toggle（上一重传起点未推进字节，不翻转）
            //     否则                 → 翻转旧 toggle（标记"部分完成 WQE"已切换实例）
            //   din[55:32] = retry_start_psn，din[31:0] = completed_dma_len
            if (retry_cs == CLEAN_SWQE_FIFO && (sq_fifo_unclean_entries_ff == 'b0)) begin
                o_retransmit_wqe_bram_din[56]    <= ~|retransmit_wqe_bram_din1[31:0] ? i_retransmit_wqe_bram_dout[56] : ~i_retransmit_wqe_bram_dout[56];
                o_retransmit_wqe_bram_din[31:0]  <= retransmit_wqe_bram_din1[31:0];
                o_retransmit_wqe_bram_din[55:32] <= retransmit_wqe_bram_din1[55:32];
                o_retransmit_wqe_bram_wen        <= 1'b1;
            end else begin
                o_retransmit_wqe_bram_wen <= 1'b0;
            end

            // --- 当进入 WQE_DROPPED 态且 OSQ 尚未同步 pop 时拉高 o_wqe_dropped
            //     与 retry_ns 的转移条件互补（一旦 OSQ pop 命中，下一拍离开 WQE_DROPPED，此处置 0）
            if ((retry_cs == WQE_DROPPED) && ~(i_osq_pop && (i_osq_pop_qpid == qp_retry_idx))) begin
                o_wqe_dropped <= 1'b1;
            end else begin
                o_wqe_dropped <= 1'b0;
            end
        end
    end

    //==========================================================================
    // FSM 组合块：状态转移 + unclean 递减
    //==========================================================================
    always @(*) begin
        retry_ns                = retry_cs;
        sq_fifo_unclean_entries = sq_fifo_unclean_entries_ff;
        case (retry_cs)

            // ======================= WQE FIFO 清理 =======================
            // 等请求
            IDLE: begin
                if (i_retransmit_reqd) retry_ns = HALT_PROCESS;
            end

            // 拉 o_halt，等 cache 把取数管线排空
            HALT_PROCESS: begin
                if (i_halted) retry_ns = GET_CONF;
            end

            // 发三路读请求；同时采样 WQE FIFO 深度作为"需清洗条目数"
            GET_CONF: begin
                sq_fifo_unclean_entries = i_num_valid_entries;
                if (i_qp_cq_hdptr_valid & i_qp_sq_psn_valid & i_qp_stat_ssn_valid) retry_ns = UPD_CONF;
            end

            // 写 PSN/SSN/sqptr/stat_ret_sq_psn；根据 prog_full 决定是否挂起
            UPD_CONF: begin
                if (i_qp_sq_psn_valid & i_qp_stat_ret_sq_psn_valid) retry_ns = !i_prog_full_db_update_curr_sqptr ? CLEAN_SWQE_FIFO : WAIT_FOR_PROG_FULL;
            end

            // doorbell 正在写 curr_sqptr，等释放；o_qp_curr_sqptr_proc_wen 仍保持
            WAIT_FOR_PROG_FULL: begin
                if (!i_prog_full_db_update_curr_sqptr) retry_ns = CLEAN_SWQE_FIFO;
            end



            // ======================= WQE FIFO 清理 =======================
            // 还有条目→POP；否则回 IDLE（同时那一拍会写 bram）
            CLEAN_SWQE_FIFO: begin
                if (sq_fifo_unclean_entries_ff != 'b0) retry_ns = POP_SWQE;
                else retry_ns = IDLE;
            end

            // pop 1 拍
            POP_SWQE: retry_ns = CHECK_SWQE;

            // 检查队头 WQE 的 qpid（藏在 WQE bit[136+:C_QP_INDX_WIDTH]）
            //   匹配 → DROP；unclean-- 在此处完成
            //   不匹配 → PUSH 回队尾
            CHECK_SWQE: begin
                sq_fifo_unclean_entries = sq_fifo_unclean_entries_ff - 1'b1;
                if (i_wqe[136+:C_QP_INDX_WIDTH] == qp_retry_idx) retry_ns = WQE_DROPPED;
                else retry_ns = PUSH_SWQE;
            end

            // 推回一条 WQE → 回循环顶
            PUSH_SWQE: retry_ns = CLEAN_SWQE_FIFO;

            // WQE_DROPPED：为本次丢弃的 WQE 产生一个 QP-side OSQ pop 事件。
            // 如果同一拍 ACK 管理器已经对同一 QP 发出 i_osq_pop，
            // 则本拍不产生 o_wqe_dropped，并停留在本状态等待下一拍再发，
            // 避免 QP 管理器内部 OSQ 缓冲对同一 QP 同拍双 pop。
            WQE_DROPPED: begin
                if (~(i_osq_pop && (i_osq_pop_qpid == qp_retry_idx))) retry_ns = CLEAN_SWQE_FIFO;
            end
            // ======================= WQE FIFO 清理 end =======================

            default: retry_ns = IDLE;

        endcase
    end

    //==========================================================================
    // 状态上下文寄存器：unclean 计数 / QP cfg / 目标索引 / WQE 打拍
    //==========================================================================
    `MSFF_R(sq_fifo_unclean_entries_ff, sq_fifo_unclean_entries, core_clk, ~core_rstn)

    // GET_CONF 时捕获 SQ PSN（用于 stat_ret_sq_psn 快照）
    `MSFF_R(qp_sq_psn_ff, (((retry_cs == GET_CONF) & i_qp_sq_psn_valid) ? i_qp_sq_psn : qp_sq_psn_ff), core_clk, ~core_rstn)

    // SSN 和 CQ head pointer 在对应 valid 时刻始终捕获（多处都会触发）
    `MSFF_R(qp_stat_ssn_ff, (i_qp_stat_ssn_valid ? i_qp_stat_ssn : qp_stat_ssn_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_hdptr_ff, (i_qp_cq_hdptr_valid ? i_qp_cq_hdptr : qp_cq_hdptr_ff), core_clk, ~core_rstn)

    // 目标 QP 索引：IDLE 态跟随 i_retransmit_qpid；其它状态锁定
    `MSFF_R(qp_retry_idx, (retry_cs == IDLE ? i_retransmit_qpid : qp_retry_idx), core_clk, ~core_rstn)

    // WQE 打 1 拍：POP 后下一拍 i_wqe 有效，再下一拍 wqe_ff 送给 o_wqe 给 PUSH_SWQE 用
    `MSFF_R(wqe_ff, i_wqe, core_clk, ~core_rstn)

    assign o_wqe                = wqe_ff;
    assign o_qp_stat_ret_sq_psn = qp_sq_psn_ff;  // 写 stat：重传前的 SQ PSN 快照

    //==========================================================================
    // 回退目标 PSN/SSN 的锁存：进入本 FSM 的那一拍捕获 ACK 管理器给出的值
    //==========================================================================
    `MSFF_R(psn_to_retry_ff, ((i_retransmit_reqd & (retry_cs == IDLE)) ? i_psn_to_retry : psn_to_retry_ff), core_clk, ~core_rstn)
    `MSFF_R(ssn_to_retry_ff, ((i_retransmit_reqd & (retry_cs == IDLE)) ? i_ssn_to_retry : ssn_to_retry_ff), core_clk, ~core_rstn)

    //==========================================================================
    // 向 QP 上下文写回的实际数据
    //==========================================================================
    assign o_qp_sq_psn              = psn_to_retry_ff;  // SQ PSN 回退值
    assign o_qp_stat_ssn            = ssn_to_retry_ff;  // SSN 回退值
    assign o_qp_curr_sqptr_proc     = qp_cq_hdptr_ff;  // SQ 消费指针 = 最后完成的位置（CQ head pointer）

    //==========================================================================
    // 读/写请求脉冲
    //==========================================================================
    // SQ PSN：GET_CONF 发读、UPD_CONF 持续保持地址（wr 同拍）
    assign o_qp_sq_psn_req          = ((retry_cs == GET_CONF) || (retry_cs == UPD_CONF)) ? 1'b1 : 1'b0;
    assign o_qp_stat_ssn_req        = (retry_cs == GET_CONF) ? 1'b1 : 1'b0;
    assign o_qp_cq_hdptr_req        = (retry_cs == GET_CONF) ? 1'b1 : 1'b0;

    assign o_qp_sq_psn_wrn          = (retry_cs == UPD_CONF) ? 1'b1 : 1'b0;  // 写 SQ PSN
    assign o_qp_stat_ret_sq_psn_wen = (retry_cs == UPD_CONF) ? 1'b1 : 1'b0;  // 写 stat 历史快照
    assign o_qp_stat_ssn_wrn        = (retry_cs == UPD_CONF) ? 1'b1 : 1'b0;  // 写 SSN
    // curr_sqptr 在 UPD_CONF 写一次；若撞上 prog_full，WAIT_FOR_PROG_FULL 期间继续保持 wen
    assign o_qp_curr_sqptr_proc_wen = (retry_cs == UPD_CONF || retry_cs == WAIT_FOR_PROG_FULL) ? 1'b1 : 1'b0;

    //==========================================================================
    // WQE FIFO push/pop 控制（仅在对应状态拉 1 拍）
    //==========================================================================
    assign o_wqe_fifo_push          = (retry_cs == PUSH_SWQE) ? 1'b1 : 1'b0;
    assign o_wqe_fifo_pop           = (retry_cs == POP_SWQE) ? 1'b1 : 1'b0;

    //==========================================================================
    // Halt 握手：非 IDLE 期间全程拉高
    //==========================================================================
    assign o_halt                   = (retry_cs != IDLE) ? 1'b1 : 1'b0;
    assign o_halted_qpid            = qp_retry_idx;

    //==========================================================================
    // 所有 per-QP 读写地址统一用 qp_retry_idx
    //==========================================================================
    assign o_qp_stat_ssn_idx        = qp_retry_idx;
    assign o_qp_sq_psn_idx          = qp_retry_idx;
    assign o_qp_stat_ret_sq_psn_idx = qp_retry_idx;
    assign o_qp_curr_sqptr_proc_idx = qp_retry_idx;
    assign o_qp_cq_hdptr_idx        = qp_retry_idx;

    //==========================================================================
    // 对外握手：IDLE 时表示可接收/已完成一次重传准备流程，ACK 管理器据此释放 retransmit_reqd
    //==========================================================================
    assign o_retransmit_accepted    = (retry_cs == IDLE) ? 1'b1 : 1'b0;

endmodule
