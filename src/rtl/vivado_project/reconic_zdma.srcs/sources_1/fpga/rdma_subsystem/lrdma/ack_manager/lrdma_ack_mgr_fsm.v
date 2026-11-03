`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_ack_mgr_fsm — ACK 管理器核心状态机
//
// 【模块功能】
// 本模块负责一个已仲裁 QP 的完成确认、重传触发和 fatal 处理。顶层先根据
// ACK/NAK、timeout、fatal 以及 pushback 事件选择待处理 QP；本模块再读取该 QP
// 的配置、OSQ 队头、ACK/NAK PSN/MSN、NAK 标志和 timeout sticky，判断 OSQ
// 条目完成、等待、进入重传，或在 fatal flush 中生成带 fatal 标记的完成事务。
//
// 【完成和重传的输出方式】
// 正常完成且 cqe_write_en=1 时，主 FSM 向 cqe_proc_fifo 写入 3'b000 事务，
// 由 CQE 子 FSM 写 4B CQE 到 DDR 并推进 CQ head pointer。cqe_write_en=0 时，
// 主 FSM 不再把完成事务送入 cqe_proc_fifo，而是等待 CQE 子 FSM 空闲后只推进
// CQ head pointer。重传路径在 OSQ flush 结束后写入 3'b100 事务，同时把
// {retry_start_psn, psn_to_retry, msn_to_retry} 写入 retry_proc_fifo，由子 FSM 向
// QP 管理器发起重传请求。ACK 管理器不直接改写 DDR 中的原始 SQ WQE。
//
// 【主 FSM 主线】
// IDLE → GET_QP_CONF_DATA → WAIT_VALID_DATA → CHECK_ERROR。
// 正常完成时，FSM 写 CQE 事务、pop OSQ 条目，再继续读取下一条 OSQ。若比较
// 发现 NACK 或 timeout 覆盖到该条目，则先结束已完成条目的提交，再进入
// START_RETRANSMIT。重传时先 pop/flush 旧 OSQ 条目；OSQ 清空后再向 QP 管理器
// 发送重传请求。fatal flush 路径只清理 OSQ，并为被清理条目生成带 fatal 标记的完成事务，
// 不再发起普通重传请求。
//
// 【CQE 子 FSM】
// cqe_proc_cs 与主 FSM 解耦，从 cqe_proc_fifo 中读取事务：
//   - 3'b000：按 QP 的 CQ 配置写 CQE，并推进 CQ head pointer；
//   - 3'b100：读取 retry_proc_fifo，向 QP 管理器发起重传请求；
//   - 其它编码：异常保护分支，直接弹出事务，避免子 FSM 卡住。
// 主 FSM 正常只产生 3'b000 和 3'b100 两类事务。
//
// 【PSN 回绕处理】
// OSQ 只保存低 C_OSQ_PSN_WIDTH 位 PSN，因此比较 ACK/NAK PSN 与 OSQ start/end PSN
// 时必须结合 phase bit。phase bit 用于区分 PSN 回绕前后的两侧，psn_0_nacked
// 负责处理 NACK 指向低位 PSN 0 的特殊边界。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_ack_mgr_fsm #(
    parameter C_NUM_QP           = 256,  // 支持的 QP 总数（决定 per-QP BRAM 深度）
    parameter C_M_AXI_ADDR_WIDTH = 32,   // ACK 管理器 AXI 写地址位宽，用于 CQE 写 DDR
    parameter C_QP_INDX_WIDTH    = 8,    // QP 索引位宽 = log2(C_NUM_QP)
    parameter IP2BUS_LEN_WIDTH   = 7,    // AXI master 写长度字段位宽（Byte）
    parameter C_OSQ_PSN_WIDTH    = 16    // OSQ 内 PSN 低位位宽（比较 start/end_psn 用），PSN 总 24 bit
) (
    input core_clk,  // 核心时钟（通常 250/322 MHz，与 lrdma 整个 subsystem 同步）
    input core_rstn, // 低有效同步复位

    //==========================================================================
    // 仲裁器接口 / 待处理标志
    //==========================================================================
    input i_acks_to_process,  // 汇总信号：至少一个 QP 的 OSQ 非空，FSM 可以请求仲裁检查是否有完成/重传事件

    // 清除"已处理完的 timeout/nack"状态（给上游 timeout/nack 追踪 FIFO 返回处理回执）
    output reg                        o_clr_timeout_n_nack_set_valid,  // 脉冲：本拍清除对应 QP 的 timeout/nack sticky
    output reg [C_QP_INDX_WIDTH -1:0] o_clr_timeout_n_nack_set_id,     // 要被清的 QP ID

    //==========================================================================
    // QP 仲裁握手
    //==========================================================================
    output                            o_arbitrate,                  // FSM 请求仲裁器给一个可处理的 QP（在 ARBITRATE_OSQ 状态拉高）
    output reg                        fsm_pushback_valid,           // 本次 QP 暂缓处理：通知仲裁器把该 QP 重新放回待处理集
    input                             i_arbitration_done,           // 仲裁结果有效（i_arbitrated_indx 可采样）
    input      [C_QP_INDX_WIDTH -1:0] i_arbitrated_indx,            // 被选中的 QP 索引
    output     [C_QP_INDX_WIDTH -1:0] o_sampled_arbitrated_indx,    // 已锁存的处理 QP = arbitrated_indx_ff（供外部观察）
    output     [C_QP_INDX_WIDTH -1:0] o_arbitrated_qp_for_intr,     // CQE 处理子 FSM 正在服务的 QP（= o_qp_cqe_proc_qpid）
    output                            o_phase_to_update_post_retry, // 重传后写回 acknacked_psn 时使用的 phase bit（用于 PSN 回绕跨相位比较）

    //==========================================================================
    // 状态/统计寄存器
    //==========================================================================
    output [                31:0] o_ack_mgr_sts,              // 状态寄存器：{14'h0, i_acks_to_process, ack_mgr_h_cs, pad, arbitrated_indx_ff}
    output [                31:0] o_stat_retry_count,         // 累计重传次数（saturate 饱和）
    input                         i_cfg_retry_cnt_fatal_dis,  // 1：禁用"重试耗尽→QP fatal"，无限重试；0：正常耗尽后生成 fatal
    input                         i_qp_fatal_valid,           // 外部写 per-QP qp_fatal BRAM 的使能
    input  [C_QP_INDX_WIDTH -1:0] i_qp_fatal_id,              // 写 qp_fatal 的 QP ID
    input                         i_qp_fatal_data,            // 写 qp_fatal 的数据
    input                         i_dma_in_idle,              // DMA 子系统空闲（QP fatal flush 必须 DMA idle 后才启动）
    input                         i_en_osq_flush_on_qp_fatal, // 使能：fatal 时扫 OSQ 生成错误 CQE

    //==========================================================================
    // 首包 PSN/相位跟踪（用于 PSN wrap 跨相位比较）
    //==========================================================================
    input                         i_first_outgoing_pkt_valid,          // 首包写 BRAM 使能
    input [C_QP_INDX_WIDTH-1 : 0] i_first_outgoing_pkt_id,             // 首包的 QP ID
    input                         i_first_outgoing_pkt_data,           // 首包 phase bit 初值（通常 1）
    input                         i_first_outgoing_pkt_valid_pending,  // pending 写队列项 valid（forward path 补偿）
    input [C_QP_INDX_WIDTH-1 : 0] i_first_outgoing_pkt_id_pending,
    input                         i_first_outgoing_pkt_data_pending,
    input [  C_OSQ_PSN_WIDTH-1:0] i_wqe_send_start_psn,                // 新 push WQE 的 start PSN（用于 last_osq_psn phase 记录）
    input [  C_QP_INDX_WIDTH-1:0] i_wqe_qpid,                          // 新 push WQE 所属 QP
    input                         i_wqe_push_req,                      // TX 引擎 推入新 WQE 的请求脉冲

    //==========================================================================
    // OSQ (Outstanding SQ Queue) 接口
    // OSQ 存"已发出但未最终完成/确认"的 WQE 条目；FSM 逐条扫描决定完成/重传/回退
    //==========================================================================
    output                           o_pop_osq,                // pop OSQ 队头（处理完成/进入重传流水线）
    output reg                       o_rd_osq,                 // 请求读取 OSQ 队头条目
    input                            i_rd_osq_vld,             // OSQ 条目数据有效（带 1~2 拍 latency）
    input                            i_osq_empty,              // OSQ 为空
    input                            i_phase_bit_acknack_psn,  // 已 ACK/NAK 的 PSN phase bit（受 rd_req_done 门控后才稳定）
    input      [                7:0] i_osq_entry_msn,          // 条目 MSN（消息序列号）
    input      [C_OSQ_PSN_WIDTH-1:0] i_osq_entry_end_psn,      // 条目结束 PSN 的低 C_OSQ_PSN_WIDTH 位
    input      [C_OSQ_PSN_WIDTH-1:0] i_osq_entry_start_psn,    // 条目起始 PSN
    input      [               23:0] i_osq_acknacked_psn,      // QP 最近的 ack/nack PSN（24 bit 完整）
    input      [               23:0] i_osq_acknacked_msn,      // QP 最近的 ack/nack MSN
    input      [                7:0] i_osq_opcode,             // OSQ 条目保存的 BTH.Opcode，普通数据路径登记 RDMA WRITE
    input      [               15:0] i_osq_wrid,               // 条目 WRID（用户标识，写入 CQE）
    input                            i_osq_wqe_retried,        // 条目已被重试过
    input                            i_osq_wqe_fatal_push,     // 条目在 QP fatal 状态下被 flush-push 的
    input                            i_osq_nacked,             // 条目被 NACK（对端明确拒绝）
    input                            i_osq_wqe_nack_resp,      // QP 的 NACK/异常响应处理完成标志；未完成时 FSM 会 pushback 等待

    output reg [23:0] o_psn_to_retry,  // 重传起始 PSN（送给 QP 管理器）
    output reg [23:0] o_msn_to_retry,  // 重传起始 MSN（高 16 位来自 i_osq_acknacked_msn）

    // Acknacked PSN 读握手：避免 FSM 决策时 ack 正在更新
    output reg acknacked_psn_rd_req,      // FSM 发起 acknacked 上下文读请求
    input      acknacked_psn_rd_req_done, // 读完成，acknacked_psn/phase/osq_nacked 可采样

    //==========================================================================
    // QP 配置上下文读接口（per-QP BRAM/寄存器）
    //==========================================================================
    output                           o_qp_cq_ba_req,       // CQ base address 读请求
    input  [C_M_AXI_ADDR_WIDTH -1:0] i_qp_cq_ba,
    input                            i_qp_cq_ba_valid,
    output                           o_qp_cq_depth_req,    // CQ 深度读请求
    input                            i_qp_cq_depth_valid,
    input  [                   15:0] i_qp_cq_depth,

    output                              o_qp_cq_hdptr_req,    // CQ head pointer 读/写请求
    input                               i_qp_cq_hdptr_valid,
    input      [                  15:0] i_qp_cq_hdptr,
    output reg [                  15:0] o_qp_cq_hdptr,        // 更新后的 CQ head pointer（写回）
    output reg                          o_qp_cq_hdptr_wrn,    // CQ head pointer 写使能（写回时拉高）
    output reg [C_QP_INDX_WIDTH -1 : 0] o_qp_cqe_proc_qpid,   // CQE 处理子 FSM 的 QP ID（与主 FSM 解耦）

    output        o_qp_conf_req,    // QP CONF 寄存器读请求
    input         i_qp_conf_valid,
    input  [31:0] i_qp_conf,        // QPCONF: [3]=cq_intr_en, [5]=cqe_write_en, [6]=under_recovery, [10:8]=pmtu

    //==========================================================================
    // 完成中断
    //==========================================================================
    input  i_intr_clr_wqe_cmpl,  // 中断清除（上层软件写寄存器）
    output o_intr_wqe_cmpl_sts,  // WQE 完成中断状态（CQE 成功写入 + intr_en 开启时拉高）

    //==========================================================================
    // QP 管理器重传接口
    //==========================================================================
    output reg [C_QP_INDX_WIDTH -1 : 0] o_retransmit_qpid,     // 需要重传的 QP ID
    output reg                          o_retransmit_reqd,     // 重传请求（直到 i_retransmit_accepted 为止）
    input                               i_retransmit_accepted, // QP 管理器已接收重传请求

    //==========================================================================
    // Retry 计数器接口（per-QP）
    //==========================================================================
    input  [                2:0] i_qp_curr_retry_cnt,         // 剩余重试次数
    output                       o_qp_curr_retry_cnt_rd_req,  // 读剩余重试次数
    output [C_QP_INDX_WIDTH-1:0] o_qp_curr_retry_cnt_qpid,    // 读/写目标 QP
    input                        i_qp_curr_retry_cnt_rd_rdy,

    output       o_qp_curr_retry_cnt_wr_req,  // 写回剩余重试次数
    input        i_qp_curr_retry_cnt_wr_rdy,
    output [2:0] o_qp_curr_retry_cnt_wr,      // NACK: cfg-1（reset-to-max-1）；Timeout: curr-1（递减）

    output       o_qp_cfg_retry_cnt_rd_req,  // 读 QP 配置的 max retry 次数
    input        i_qp_cfg_retry_cnt_rd_rdy,
    input  [2:0] i_qp_cfg_retry_cnt,

    // QP fatal 接口（重试耗尽时触发）
    output                               o_qp_retry_fatal_req,   // 向 QP 管理器 发起 fatal（GEN_QP_FATAL 状态）
    output [        C_QP_INDX_WIDTH-1:0] o_qp_retry_fatal_qpid,
    input                                i_qp_retry_fatal_rdy,
    input  [(3*C_QP_INDX_WIDTH+5)-1 : 0] i_reload_retry_cnt_en,  // 重载 retry 计数使能的分层结构
                                                                 // {valid_pending_1, id_pending_1, data_pending_1,
                                                                 //  valid_pending_0, id_pending_0,
                                                                 //  valid, id, data}

    //==========================================================================
    // AXI Master 数据路径（用于 CQE 写入 DDR）
    //==========================================================================
    input                                i_bus2ip_data_rdy,         // AXI master 就绪
    input                                i_lrdma_axi_master_done,   // AXI 写完成
    output reg                           o_ip2bus_wen,              // AXI 写使能脉冲
    output reg [C_M_AXI_ADDR_WIDTH -1:0] o_ip2bus_wraddr,           // 写地址
    output reg [  IP2BUS_LEN_WIDTH -1:0] o_ip2bus_len,              // 写长度（Byte）
    output reg [                  511:0] o_ip2bus_data,             // CQE 写数据，高位补 0，有效写入长度为 4B
    output reg [                   55:0] o_retransmit_wqe_bram_din, // 重传 BRAM 写入：{retry_start_psn[23:0], completed_dma_len[31:0]}

    //==========================================================================
    // 复位初始化扫描（给所有 per-QP BRAM 在 init_start 期间清 0）
    //==========================================================================
    input                          init_start,
    input  [C_QP_INDX_WIDTH-1 : 0] init_count,
    output [C_QP_INDX_WIDTH-1 : 0] fsm_arbitrated_indx,   // = arbitrated_indx_ff
    output [C_QP_INDX_WIDTH-1 : 0] fsm_i_arbitrated_indx, // 当 IDLE 且 arbitration_done 时透传 i_arbitrated_indx，否则 ff 值

    // OSQ timeout sticky 读出（已按 fsm_arbitrated_indx 索引）
    input osq_timed_out_sticky_fsm_arbitrated_indx,

    // per-QP 状态 BRAM 写入的分层结构
    // osq_nacked_ff = {valid_pending, id_pending, valid, id, data}
    input [2*C_QP_INDX_WIDTH+3-1 : 0] osq_nacked_ff,

    // QP under recovery BRAM 外部写口（在 ACK 管理器顶层中由 QP 管理器的 fatal clear 驱动）
    input                         i_qp_under_recovery_wr_en,
    input [C_QP_INDX_WIDTH-1 : 0] i_qp_under_recovery_wr_idx,
    input                         i_qp_under_recovery_wr_data
);

    `include "lrdma_defs.vh"

    localparam CQE_ENTRY_SIZE = 4;  // 单个 CQE 写入大小（Byte）

    //-------------------
    // CQE entry 
    // ------------------------------------
    // |BYTE4-|BYTE3-|BYTE2-|BYTE1-|BYTE0-|
    // |------|------|------|-------------|
    // |Rsrv+ | OPC  |  WRID              |
    // |Fatal |      |                    |
    // |------|------|--------------------|


    //==========================================================================
    // 主 FSM 状态（5 位编码）
    //
    // 这个 FSM 由“仲裁器选中某个 QP”触发（i_arbitration_done=1），任务是
    // 把该 QP 的 OSQ（Outstanding SQ Queue，已发出但还没完成确认的 WQE
    // 条目队列）逐条扫描，决定每条 WQE 的命运：
    //   - 已被对端 ACK 了 → 提交 CQE 写事务，推进 CQ head pointer，并弹出 OSQ
    //   - 还没被 ACK     → 留在 OSQ，结束这一轮扫描
    //   - 落入 NAK/超时窗口 → 转入重传分支
    //
    // 时间线一览（一次正常 ACK 扫描）：
    //   IDLE → GET_QP_CONF_DATA → WAIT_VALID_DATA → CHECK_ERROR
    //        → ( WRITE_CQE → WAIT_DATA_STABLE → RD_NEXT_OSQ_AFTER_CQE → UPD_CQHD
    //            → CHECK_OSQ )*
    //        → FINISH_OSQ_BATCH → ARBITRATE_OSQ → IDLE
    //
    // 重传分支（CHECK_OSQ 发现需要重传，或 CHECK_ERROR 直接进重传）：
    //        → START_RETRANSMIT → [GEN_QP_FATAL?] → RETRANSMIT
    //        → FLUSH_POP_OSQ → CHECK_FLUSH_OSQ
    //        → ( RETRANSMIT → FLUSH_POP_OSQ → CHECK_FLUSH_OSQ )*
    //        → ARBITRATE_OSQ → IDLE
    //
    // 注意：状态编码不连续（如 WAIT_DATA_STABLE=5'b11011），是历史演进留下
    // 来的，没有特殊综合诉求；FSM 本身按描述顺序读即可。
    //==========================================================================
    localparam IDLE = 5'b00000;  // 等待仲裁；收到 i_arbitration_done 后锁存 QP，发起 QP CONF / OSQ 读
    localparam GET_QP_CONF_DATA = 5'b00001;  // 等待 QP CONF 数据 + cfg_retry_cnt 到达；满足后发起 acknacked_psn 读
    localparam WAIT_VALID_DATA = 5'b00010;  // 等待 acknacked 读完成；读到后锁存所有 OSQ 条目/ack 上下文，进入 CHECK_ERROR
    localparam CHECK_ERROR = 5'b00011;  // 核心决策：MAD 跳过 / NACK 未处理回推 / qp_fatal 完成 / PSN 比较决定完成 or 重传
    localparam WRITE_CQE = 5'b00100;  // 完成 OSQ 条目：cqe_write_en=1 时入 CQE FIFO，否则转 CQ head pointer bypass
    localparam BYPASS_WAIT_CQE_IDLE = 5'b01110;  // cqe_write_en=0 路径等待 CQE 子 FSM 空闲，避免共用 CQ 配置端口冲突
    localparam BYPASS_CQHDPTR_WAIT = 5'b01111;  // cqe_write_en=0 路径读取 CQ depth/head pointer
    localparam BYPASS_CQHDPTR_WR = 5'b10001;  // cqe_write_en=0 路径只写回 CQ head pointer，并 pop OSQ
    localparam WAIT_DATA_STABLE = 5'b11011;  // 撤销 wren/pop，给后续 OSQ 读和 FIFO 读侧留出一拍间隔
    localparam RD_NEXT_OSQ_AFTER_CQE = 5'b00101;  // 发起下一条 OSQ 读，并重新读取 ACK/NAK 上下文
    localparam UPD_CQHD = 5'b01011;  // 等 OSQ/acknacked 下一条读到位，锁存后进入 CHECK_OSQ
    localparam CHECK_OSQ = 5'b00110;  // 二级 PSN 比较：决定还需完成更多 / 需要重传 / 扫描结束
    localparam FINISH_OSQ_BATCH = 5'b00111;  // 本轮 OSQ/CQE 扫描收尾，根据 retransmit_en / qp_fatal_flush 分流
    localparam START_RETRANSMIT = 5'b01101;  // 捕获 retry_start_psn/completed_dma_len_multiplier，检查 retry 计数
    localparam RETRANSMIT = 5'b01001;  // 重传 flush 过渡状态，清 pop_osq 后继续读取下一条 OSQ
    localparam FLUSH_POP_OSQ = 5'b01010;  // 发起下一条 OSQ 读
    localparam CHECK_FLUSH_OSQ = 5'b01100;  // 重传/fatal flush 收尾：OSQ 未空则继续 pop；非 fatal 重传且 OSQ 空后写 3'b100 + retry_proc_fifo
    localparam GEN_QP_FATAL = 5'b10000;  // 重试次数耗尽，先向 QP 管理器报 fatal；握手完成后进入 RETRANSMIT/flush 收尾
    localparam ARBITRATE_OSQ = 5'b01000;  // 清各 FIFO 写使能和 pushback_valid，回 IDLE 等下一轮仲裁



    //==========================================================================
    // CQE 处理子 FSM（cqe_proc_cs）
    //   3'b000 事务：CQE_IDLE → GET_QP_CONF → GEN_TRANS → WAIT_TRANS_DONE → CQE_IDLE
    //   其它事务：CQE_IDLE → GEN_TRANS → WAIT_TRANS_DONE → CQE_IDLE
    //
    // 子 FSM 从 cqe_proc_fifo 中取事务，并和主 FSM 的 OSQ 扫描解耦。
    // 正常事务只有两类：
    //   3'b000：读取 CQ base/depth/head pointer，必要时向 DDR 写 4B CQE，并推进 CQ head pointer；
    //   3'b100：读取 retry_proc_fifo，向 QP 管理器拉起重传请求，不访问 DDR。
    // 其它 sub_opcode 只作为异常保护分支处理，直接弹出 FIFO 项。
    //==========================================================================
    localparam CQE_IDLE = 3'b000;  // 空闲。监视 cqe_proc_fifo；3'b000 事务需要读取 CQ 配置，其它事务可直接进入 GEN_TRANS
    localparam GET_QP_CONF = 3'b001;  // 保持 QP 配置读请求，等待 CQ base/depth/head pointer 返回
    localparam GEN_TRANS = 3'b011;  // 根据 sub_opcode 生成 CQE AXI 写、触发重传请求，或弹出异常事务
    localparam WAIT_TRANS_DONE = 3'b100;  // 等 CQE AXI 写完成、重传请求被接受，或异常事务弹出完成；之后回 CQE_IDLE

    localparam QP1 = 'd0;  // MAD QP 内部索引（软件约定 QP1 不走 RDMA 流程）

    localparam CQE_SZ_IN_BYTES = 4;  // CQE 写入长度（Byte）

    //==========================================================================
    // 主 FSM 状态寄存器 & 锁存的 QP 配置
    //==========================================================================
    reg  [                    4:0] ack_mgr_h_cs;  // 主 FSM 状态
    reg  [C_M_AXI_ADDR_WIDTH -1:0] qp_cq_ba_ff;  // 锁存的 CQ 基地址
    reg  [                   15:0] qp_cq_depth_ff;  // 锁存的 CQ 深度
    reg  [                   15:0] qp_cq_hdptr_ff;  // 锁存的 CQ head pointer
    reg  [   C_QP_INDX_WIDTH -1:0] arbitrated_indx_ff;  // 处理 QP id（主索引）
    reg  [   C_QP_INDX_WIDTH -1:0] arbitrated_indx_2ff;  // 再延一拍，供某些时序对齐

    //--- CQE 处理 FIFO（主 FSM 推入，CQE 子 FSM 弹出）
    // 数据格式：{qp_id[C_QP_INDX_WIDTH], intr_en[1], cqe_write_en[1], sub_opcode[3], payload[32]}
    //   sub_opcode = 3'b000：写 CQE entry。
    //       payload[31:0] = {7'h0, fatal_err, opcode[7:0], wrid[15:0]}。
    //   sub_opcode = 3'b100：重传 flush 已结束。
    //       payload[31:0] = completed_dma_len；子 FSM 同时读取 retry_proc_fifo，
    //       生成 QP 管理器重传请求和重传 WQE BRAM 写入数据。
    // 主 FSM 正常只生成 3'b000 和 3'b100。其它 sub_opcode 只作为异常保护分支，
    // 子 FSM 会直接弹出该事务，防止异常 FIFO 项导致状态机停住。
    reg  [ C_QP_INDX_WIDTH+36 : 0] cqe_proc_fifo_data;
    wire [ C_QP_INDX_WIDTH+36 : 0] cqe_proc_fifo_dout;
    reg                            cqe_proc_fifo_wren;
    reg                            cqe_proc_fifo_rden;
    wire                           cqe_proc_fifo_empty;
    wire                           cqe_proc_fifo_full;

    //--- 重传信息 FIFO（主 FSM 在 CHECK_FLUSH_OSQ 写；子 FSM 遇到 3'b100 时读）
    // 数据格式(72b)：{retry_start_psn[23:0], psn_to_retry[23:0], i_osq_acknacked_msn[23:8], msn_to_retry[7:0]}
    reg  [                 71 : 0] retry_proc_fifo_data;
    wire [                 71 : 0] retry_proc_fifo_dout;
    reg                            retry_proc_fifo_wren;
    reg                            retry_proc_fifo_rden;
    wire                           retry_proc_fifo_empty;
    wire                           retry_proc_fifo_full;

    reg                            osq_empty_ff;  // i_osq_empty 打 1 拍
    reg                            osq_empty_2ff;  // 再打 1 拍（OSQ empty 需合成 3 拍窗口防脉冲丢失）
    reg                            completion_posted;  // CQE AXI 写请求已发出，等待 AXI done
    reg                            qp_cq_intr_en_ff;  // QPCONF[3]：CQ 中断使能
    reg                            qp_cqe_write_en_ff;  // QPCONF[5]：CQE 写使能（0 则不写实际 CQE）
    reg  [                    2:0] qp_pmtu_ff;  // QPCONF[10:8]：PMTU 编码
    reg  [                   12:0] pmtu_decoded;  // pmtu 解码字节数（256/512/1024/2048/4096）
    reg  [                   12:0] pmtu_decoded_r;  // 寄一拍用于 completed_dma_len 计算

    //--- 重传参数（捕获于 START_RETRANSMIT，消费于 RETRANSMIT→CHECK_FLUSH_OSQ 闭环）
    reg  [                   23:0] psn_to_retry;  // 重传起始 PSN
    reg  [                    7:0] msn_to_retry;  // 重传起始 MSN（低 8 位）
    reg                            upd_phase_post_retransmit;  // "START_RETRANSMIT 之后需更新 phase bit" 标记，pop_osq 时一拍写入 BRAM
    reg                            osq_nacked_sampled;  // acknacked 读完成窗口采样 i_osq_nacked（防决策中被新 ack 污染）
    reg                            phase_bit_acknack_psn_sampled;  // 同样窗口采样 phase bit
    reg  [                   31:0] retry_count_ff;  // 累计重传次数（饱和）
    wire                           upd_phase_retransmit_pop;  // = upd_phase_post_retransmit & pop_osq（phase 写 BRAM 窗口）
    reg                            pop_osq;  // 向 OSQ 发 pop 脉冲
    reg                            osq_timed_out;  // osq_timed_out_sticky 打窗口寄存（与 osq_nacked_sampled 同策略）
    wire                           phase_bit_osq_psn;  // 依据 last_osq_psn / OSQ end_psn 推算的相位比较结果
    reg                            completion_accepted;  // 一次 CQE 写完成（供中断判定）
    wire                           last_osq_psn_for_arbitrated_idx;  // per-QP last osq psn bit（BRAM 读）
    wire [                   15:0] new_cq_hdptr;  // CQ head pointer +1（含 wrap）
    wire [                   15:0] qp_cq_hdptr_base_0;  // CQ head pointer 的 wrap-base-0 表示
    wire [C_M_AXI_ADDR_WIDTH -1:0] cqe_axi_address;  // CQE 写入地址 = cq_ba + hdptr_base*4
    wire [                   23:0] full_psn_to_retry;  // 重传起始 PSN 的源：NAK=acknacked；Timeout=acknacked+1
    wire                           phase_mismatch;  // ack 相位 ^ osq 相位（CHECK_ERROR 比较用）
    wire                           psn_0_nacked;  // psn_0_nacked 表示 NAK 指向的低位 PSN 为 0。在 OSQ 低位 PSN 资源优化下，它用于处理低位 PSN 回绕边界，不表示完整 24-bit PSN 一定为 0。
    reg                            pending_cqdb_update;  // CHECK_OSQ 发现条目也要生成 CQE 时置位，CHECK_ERROR 下一拍消费
    reg                            qp_fatal_flush;  // qp_fatal 且 DMA idle 且 en_flush → 扫 OSQ flush 所有条目
    reg  [                    2:0] qp_curr_retry_cnt_ff;  // 锁存的剩余重试次数
    reg  [                    2:0] qp_cfg_retry_cnt_ff;  // 锁存的 QP 配置重试次数上限
    reg                            qp_fatal;  // 本轮决策中 qp 是否 fatal（= qp_fatal_rd & !osq_nacked_top）
    reg                            qp_fatal_or_recovery;  // qp_fatal | under_recovery（决策时一并考虑）

    //--- 采样/锁存的 OSQ 条目（CHECK_ERROR/CHECK_OSQ 决策用）
    reg                            phase_bit_acknack_psn;
    reg  [                    7:0] osq_entry_msn;
    reg  [    C_OSQ_PSN_WIDTH-1:0] osq_entry_end_psn;
    reg  [    C_OSQ_PSN_WIDTH-1:0] osq_entry_start_psn;
    reg  [                   23:0] osq_acknacked_psn;
    reg  [                    7:0] osq_opcode;
    reg  [                   15:0] osq_wrid;
    reg                            osq_wqe_retried;
    reg                            osq_wqe_fatal_push;
    reg                            osq_wqe_nack_resp;
    reg                            osq_wqe_nack_resp_ff;

    //--- CQE 子 FSM 用
    reg  [    C_OSQ_PSN_WIDTH+1:0] completed_dma_len_multiplier;  // 完成 PSN 数（乘 PMTU = 字节数）
    wire [                   31:0] completed_dma_len;  // = multiplier * pmtu_decoded_r
    reg  [                   31:0] cqe_proc_completed_dma_len;  // 子 FSM 从 FIFO 取出后锁存
    reg  [                   23:0] retry_start_psn;  // 重传起始 PSN（OSQ 条目的 start_psn）
    reg                            cfg_rd_req;  // QP CONF 读请求（IDLE→GET_QP_CONF_DATA）
    reg                            retransmit_en;  // CHECK_OSQ → FINISH_OSQ_BATCH 路径上需不需要后续重传
    reg                            wr_retry_cnt;  // 触发 retry 计数器写回
    reg i_rd_osq_vld_r1, i_rd_osq_vld_r2;  // i_rd_osq_vld 两级打拍（捕获新条目数据）
    reg                          osq_empty;  // 组合性 OSQ 空（主 FSM 内比较用）

    //--- 首包/相位 BRAM 附属信号
    wire                         i_first_outgoing_pkt_rd_data_0;
    wire                         i_first_outgoing_pkt_rd_data_int_0;
    reg                          last_osq_psn_valid;
    reg  [C_QP_INDX_WIDTH-1 : 0] last_osq_psn_id;
    reg                          last_osq_psn_data;
    reg                          last_osq_psn_valid_aux;
    reg  [C_QP_INDX_WIDTH-1 : 0] last_osq_psn_id_aux;
    reg                          last_osq_psn_data_aux;
    wire                         last_osq_psn_rd_data;
    reg                          i_reload_retry_cnt_en_rd_data;  // 组合选择：pending0/pending1/BRAM 读出
    wire                         i_reload_retry_cnt_en_rd_data_int;

    //--- per-QP 状态 BRAM 读出/forward 合成
    wire                         i_qp_fatal_rd_data;
    wire                         osq_nacked_ff_rd_data_int;
    wire                         osq_nacked_ff_rd_data;  // forward 后的 osq_nacked
    wire                         osq_nacked_top;  // osq_nacked | timed_out sticky
    wire                         osq_nacked_ff_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id;
    wire                         osq_nacked_ff_data;
    wire                         osq_nacked_ff_valid_pending;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id_pending;

    // phase_bit_osq_psn BRAM 双写口（正常写 + 重传完成后的 aux 写）
    reg                          phase_bit_osq_psn_ff_valid;
    reg  [C_QP_INDX_WIDTH-1 : 0] phase_bit_osq_psn_ff_id;
    reg                          phase_bit_osq_psn_ff_data;
    reg                          phase_bit_osq_psn_ff_valid_aux;
    reg  [C_QP_INDX_WIDTH-1 : 0] phase_bit_osq_psn_ff_id_aux;
    reg                          phase_bit_osq_psn_ff_data_aux;
    wire                         phase_bit_osq_psn_ff_rd_data_1;  // 由 o_qp_cqe_proc_qpid 索引（子 FSM 用）
    wire                         phase_bit_osq_psn_ff_rd_data_0;  // 由 arbitrated_indx_ff 索引（主 FSM 用）

    // reload_retry_cnt 分层 forward
    wire                         reload_retry_cnt_valid_pending_1;
    wire [C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id_pending_1;
    wire                         reload_retry_cnt_data_pending_1;
    wire                         reload_retry_cnt_valid_pending_0;
    wire [C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id_pending_0;
    wire                         reload_retry_cnt_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id;
    wire                         reload_retry_cnt_data;
    reg                          qp_under_recovery_reg;  // qp_under_recovery 打一拍（BRAM 读有 1 拍 latency）
    wire                         qp_under_recovery;

    reg                          main_cqhdptr_rd_req;  // cqe_write_en=0 bypass 路径发起 CQ depth/head pointer 读请求
    reg                          cqe_cfg_rd_req;
    reg  [                  2:0] cqe_proc_cs;
    reg retransmit_req, axi_mm_req;
    reg trans_done;
    reg cqe_proc_intr_en;
    reg cqhdptr_only_req;


    // ACK 管理器不直接改写 DDR 中的 SQ WQE。
    // 部分重传需要的地址和长度修正由 QP 管理器完成：
    //   1) 本模块在 CHECK_FLUSH_OSQ 结束时写入 retry_proc_fifo，并通过 3'b100 事务
    //      向 QP 管理器发送 psn_to_retry、msn_to_retry、retry_start_psn 和 completed_dma_len；
    //   2) QP 管理器重传控制回退 SQPSN、STATSSN 和 STATCURSQPTR，并写重传 WQE BRAM；
    //   3) WQE 重新从 DDR SQ 读回后，QP 管理器返回侧根据重传 WQE BRAM 对内部 WQE
    //      数据执行 retransmit_wqe 修正。

    /*o_osq_timeout logic formation is common for osq_nacked_top and osq_timed_out*/
    assign {osq_nacked_ff_valid_pending, osq_nacked_ff_id_pending, osq_nacked_ff_valid, osq_nacked_ff_id, osq_nacked_ff_data} = osq_nacked_ff;
    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (osq_nacked_ff_rd_data_int)
    );

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_fatal (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_fatal_valid),
        .wr_addr (init_start ? init_count : i_qp_fatal_id),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (i_qp_fatal_rd_data)
    );

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_under_recovery (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_under_recovery_wr_en),
        .wr_addr (init_start ? init_count : i_qp_under_recovery_wr_idx),
        .wr_data (init_start ? 1'b0 : i_qp_under_recovery_wr_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (qp_under_recovery)
    );
    `MSFF_R(qp_under_recovery_reg, qp_under_recovery, core_clk, ~core_rstn)

    assign osq_nacked_ff_rd_data = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == arbitrated_indx_ff) ? 1'b1 : osq_nacked_ff_rd_data_int;

    assign osq_nacked_top        = osq_nacked_ff_rd_data || osq_timed_out_sticky_fsm_arbitrated_indx;

    wire current_qp_fatal = qp_fatal | (qp_under_recovery_reg & ~(osq_timed_out || i_osq_nacked));
    `MSFF_R(qp_fatal, (i_qp_fatal_rd_data & !osq_nacked_top), core_clk, ~core_rstn)
    `MSFF_R(qp_fatal_or_recovery, ((i_qp_fatal_rd_data || qp_under_recovery) && !osq_nacked_top), core_clk, ~core_rstn)


    always @(posedge core_clk) begin
        if (~core_rstn) begin
            ack_mgr_h_cs                 <= IDLE;
            acknacked_psn_rd_req         <= 1'b0;
            phase_bit_acknack_psn        <= 1'b0;
            osq_entry_msn                <= 8'h00;
            osq_entry_end_psn            <= 'h0;
            osq_entry_start_psn          <= 'h0;
            osq_acknacked_psn            <= 24'h0;
            osq_opcode                   <= 8'h00;
            osq_wrid                     <= 16'h0000;
            osq_wqe_retried              <= 1'b0;
            osq_wqe_fatal_push           <= 1'b0;
            osq_wqe_nack_resp            <= 1'b0;
            o_rd_osq                     <= 1'b0;
            cqe_proc_fifo_data           <= 'd00;
            cqe_proc_fifo_wren           <= 1'b0;
            retry_proc_fifo_data         <= 72'd0;
            retry_proc_fifo_wren         <= 1'b0;
            pop_osq                      <= 1'b0;
            cfg_rd_req                   <= 1'b0;
            retransmit_en                <= 1'b0;
            wr_retry_cnt                 <= 1'b0;
            i_rd_osq_vld_r1              <= 1'b0;
            i_rd_osq_vld_r2              <= 1'b0;
            pending_cqdb_update          <= 1'b0;
            osq_empty                    <= 1'b0;
            pmtu_decoded                 <= 13'h0000;
            completed_dma_len_multiplier <= 'd0;
            retry_start_psn              <= 24'd0;
            fsm_pushback_valid           <= 1'b0;
            main_cqhdptr_rd_req          <= 1'b0;
        end else begin

            i_rd_osq_vld_r1 <= i_rd_osq_vld;
            i_rd_osq_vld_r2 <= i_rd_osq_vld_r1;

            case (qp_pmtu_ff)
                3'b000: begin
                    pmtu_decoded <= 13'h0100;
                end
                3'b001: begin
                    pmtu_decoded <= 13'h0200;
                end
                3'b010: begin
                    pmtu_decoded <= 13'h0400;
                end
                3'b011: begin
                    pmtu_decoded <= 13'h0800;
                end
                3'b100: begin
                    pmtu_decoded <= 13'h1000;
                end
                default: begin
                    pmtu_decoded <= 13'h1000;
                end
            endcase

            // 默认不请求 CQ head pointer bypass；需要保持请求的状态会重新拉高。
            main_cqhdptr_rd_req <= 1'b0;

            case (ack_mgr_h_cs)

                // -------- IDLE：等仲裁 --------
                // 收到 i_arbitration_done → 锁存 arbitrated_indx_ff (通过 MSFF_R)，
                // 同时拉起 cfg_rd_req（读 QP CONF/cfg_retry_cnt）和 o_rd_osq（读 OSQ 队头）
                IDLE: begin
                    acknacked_psn_rd_req <= 1'b0;

                    if (i_arbitration_done) begin
                        ack_mgr_h_cs <= GET_QP_CONF_DATA;
                        cfg_rd_req   <= 1'b1;
                        o_rd_osq     <= 1'b1;
                    end
                end

                // -------- GET_QP_CONF_DATA：等 QP 配置 --------
                // 同时等待 i_qp_conf_valid（QPCONF）和 i_qp_cfg_retry_cnt_rd_rdy（重试上限）
                // 就绪后关 cfg_rd_req，开 acknacked_psn_rd_req（读 ack 上下文）
                GET_QP_CONF_DATA: begin
                    if (i_qp_conf_valid && i_qp_cfg_retry_cnt_rd_rdy) begin
                        ack_mgr_h_cs         <= WAIT_VALID_DATA;
                        cfg_rd_req           <= 1'b0;
                        acknacked_psn_rd_req <= 1'b1;
                    end
                end

                // -------- WAIT_VALID_DATA：等 ack 上下文 --------
                // acknacked_psn_rd_req_done=1 → 一拍锁存所有 OSQ 条目字段 + ack 上下文，
                // 随后进 CHECK_ERROR。此时 osq_nacked_sampled/phase_bit_acknack_psn_sampled
                // 也在对应 MSFF_R 里被同窗口采样，避免后续 ack 翻新污染决策。
                WAIT_VALID_DATA: begin
                    o_rd_osq             <= 1'b0;
                    acknacked_psn_rd_req <= 1'b0;
                    if (acknacked_psn_rd_req_done) begin
                        ack_mgr_h_cs          <= CHECK_ERROR;
                        phase_bit_acknack_psn <= i_phase_bit_acknack_psn;
                        osq_entry_msn         <= i_osq_entry_msn;
                        osq_entry_end_psn     <= i_osq_entry_end_psn;
                        osq_entry_start_psn   <= i_osq_entry_start_psn;
                        osq_acknacked_psn     <= i_osq_acknacked_psn;
                        osq_opcode            <= i_osq_opcode;
                        osq_wrid              <= i_osq_wrid;
                        osq_wqe_retried       <= i_osq_wqe_retried;
                        osq_wqe_fatal_push    <= i_osq_wqe_fatal_push;
                        osq_wqe_nack_resp     <= i_osq_wqe_nack_resp;
                        osq_empty             <= i_osq_empty || osq_empty_ff || osq_empty_2ff;
                    end else begin
                        ack_mgr_h_cs <= WAIT_VALID_DATA;
                    end
                end

                // -------- CHECK_ERROR：主决策 --------
                // 决策优先级（先命中先执行）：
                //   1. MAD QP (QP1) → 跳过（ARBITRATE_OSQ）
                //   2. 存在未处理的 NAK 或 timeout 且响应尚未完成 → pushback 到仲裁器
                //   3. qp_fatal/under_recovery 且 DMA idle 且 en_flush → 转 WRITE_CQE 做错误完成
                //   4. qp_fatal/under_recovery 其它 → pushback 等 DMA idle
                //   5. psn_0_nacked（NAK 的 PSN==0，回绕特殊情况）：
                //        osq_end<start 或 start==0 → START_RETRANSMIT
                //        否则 → WRITE_CQE（先完成上一相位条目）
                //   6. Timeout 且 end>ack (相位正常) 或 end<ack (相位翻转) → START_RETRANSMIT
                //   7. NAK 且 end>=ack（相位正常）或 end<=ack（相位翻转）→ START_RETRANSMIT
                //   8. end 仍在 ack 之后（未完成） 或 OSQ 空 → ARBITRATE_OSQ
                //   9. 默认 → WRITE_CQE（正常完成）
                // pending_cqdb_update：上一次 CHECK_OSQ 发现 OSQ 条目也需要生成 CQE 时置位，
                //                      这里读到后强制进入 WRITE_CQE，然后再继续决策。
                CHECK_ERROR: begin
                    pending_cqdb_update <= 1'b0;
                    if (arbitrated_indx_ff == QP1) begin
                        ack_mgr_h_cs <= ARBITRATE_OSQ;
                    end else if (~pending_cqdb_update & ((osq_nacked_sampled | osq_timed_out) & ~osq_wqe_nack_resp_ff)) begin
                        ack_mgr_h_cs       <= ARBITRATE_OSQ;
                        fsm_pushback_valid <= 1'b1;
                    end else if ((current_qp_fatal & i_dma_in_idle & i_en_osq_flush_on_qp_fatal) || pending_cqdb_update || (current_qp_fatal & qp_fatal_flush)) begin
                        ack_mgr_h_cs <= WRITE_CQE;
                    end else if (current_qp_fatal & ~(i_dma_in_idle & i_en_osq_flush_on_qp_fatal)) begin
                        ack_mgr_h_cs       <= ARBITRATE_OSQ;
                        fsm_pushback_valid <= 1'b1;
                    end else if (psn_0_nacked) begin  // NACK 低位 PSN 为 0 时的回绕边界处理
                        if ((osq_entry_end_psn < osq_entry_start_psn) | (osq_entry_start_psn == 'b0))  // 情况 1/2：OSQ 条目跨回绕点，或从低位 0 开始
                            ack_mgr_h_cs <= START_RETRANSMIT;
                        else  // 情况 3：OSQ 条目未跨回绕点
                            ack_mgr_h_cs <= WRITE_CQE;
                    end else if (((osq_entry_end_psn > (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) & osq_timed_out & ~phase_mismatch) | ((osq_entry_end_psn < (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) & osq_timed_out & phase_mismatch)) ack_mgr_h_cs <= START_RETRANSMIT;
                    else if (((osq_entry_end_psn >= (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) & osq_nacked_sampled & ~phase_mismatch) | ((osq_entry_end_psn <= (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) & osq_nacked_sampled & phase_mismatch)) ack_mgr_h_cs <= START_RETRANSMIT;
                    else if (((osq_entry_end_psn > osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0]) && ~phase_mismatch) ||
                        //(i_osq_empty || osq_empty_ff || osq_empty_2ff))
                        (osq_empty))
                        ack_mgr_h_cs <= ARBITRATE_OSQ;
                    else if ((osq_entry_end_psn < osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0]) && phase_mismatch) ack_mgr_h_cs <= ARBITRATE_OSQ;
                    else ack_mgr_h_cs <= WRITE_CQE;
                end

                // NAK 的 PSN 为 0 是 PSN 回绕边界上的特殊情况，需要单独判断。
                // 这是因为低位 PSN 在回绕后重新从 0 开始，普通大小比较不能直接表示
                // 低位 PSN 回绕边界两侧的 PSN 先后关系。
                // 下面分三类处理，其中 SPSN 表示 OSQ 条目的 start_psn，EPSN 表示 end_psn，
                // “|”表示 PSN 回绕点：
                //   1) SPSN ............. | nack0 ... EPSN       ：该条跨过回绕点，NACK 指向新一轮 PSN 0；
                //   2) .................. | nack0/SPSN ... EPSN  ：该条从 PSN 0 开始；
                //   3) SPSN1 EPSN1 SPSN2. | nack0 ... EPSN2      ：回绕前条目已完成，回绕后条目需要重传。

                // -------- WRITE_CQE：完成 OSQ 条目 --------
                // cqe_write_en=1：提交 3'b000 事务到 cqe_proc_fifo，CQE 子 FSM 写 DDR CQE 并推进 CQ head pointer。
                // cqe_write_en=0：不写 DDR CQE，也不占用 cqe_proc_fifo；后续 bypass 状态只推进 CQ head pointer。
                // payload[31:0] = {rsvd[6:0], fatal_err, opcode[7:0], wrid[15:0]}
                WRITE_CQE: begin
                    if (qp_cqe_write_en_ff) begin
                        if (~cqe_proc_fifo_full) begin
                            cqe_proc_fifo_data <= {arbitrated_indx_ff, qp_cq_intr_en_ff, qp_cqe_write_en_ff, 3'b000, 7'h00, current_qp_fatal, osq_opcode, osq_wrid};
                            cqe_proc_fifo_wren <= 1'b1;
                            ack_mgr_h_cs       <= WAIT_DATA_STABLE;
                            pop_osq            <= 1'b1;
                        end
                    end else begin
                        cqe_proc_fifo_wren <= 1'b0;
                        pop_osq            <= 1'b0;
                        ack_mgr_h_cs       <= BYPASS_WAIT_CQE_IDLE;
                    end
                end

                // -------- BYPASS_WAIT_CQE_IDLE：等待共用 CQ 配置端口空闲 --------
                // CQ head pointer 读写地址使用 o_qp_cqe_proc_qpid。为了不打断正在处理的 CQE/重传事务，
                // bypass 路径必须等 CQE 子 FSM 空闲且 cqe_proc_fifo 没有待处理项后再使用该端口。
                BYPASS_WAIT_CQE_IDLE: begin
                    cqe_proc_fifo_wren <= 1'b0;
                    pop_osq            <= 1'b0;
                    if ((cqe_proc_cs == CQE_IDLE) && cqe_proc_fifo_empty && !cqe_cfg_rd_req && !o_qp_cq_hdptr_wrn) begin
                        main_cqhdptr_rd_req <= 1'b1;
                        ack_mgr_h_cs        <= BYPASS_CQHDPTR_WAIT;
                    end
                end

                // -------- BYPASS_CQHDPTR_WAIT：读取 CQ depth/head pointer --------
                // depth 和 head pointer 返回后，先等寄存器打一拍更新 qp_cq_depth_ff/qp_cq_hdptr_ff，
                // 下一拍再用 new_cq_hdptr 写回，避免同拍使用旧值。
                BYPASS_CQHDPTR_WAIT: begin
                    main_cqhdptr_rd_req <= 1'b1;
                    cqe_proc_fifo_wren  <= 1'b0;
                    pop_osq             <= 1'b0;
                    if (i_qp_cq_depth_valid && i_qp_cq_hdptr_valid) begin
                        main_cqhdptr_rd_req <= 1'b0;
                        ack_mgr_h_cs        <= BYPASS_CQHDPTR_WR;
                    end
                end

                // -------- BYPASS_CQHDPTR_WR：只推进 CQ head pointer --------
                // 该状态不写 CQE DDR、不产生完成中断，只打一拍 CQ head pointer 写使能并 pop OSQ。
                BYPASS_CQHDPTR_WR: begin
                    cqe_proc_fifo_wren <= 1'b0;
                    pop_osq            <= 1'b1;
                    ack_mgr_h_cs       <= WAIT_DATA_STABLE;
                end

                // -------- WAIT_DATA_STABLE：撤销本拍写使能并隔离后续读请求 --------
                WAIT_DATA_STABLE: begin
                    cqe_proc_fifo_wren <= 1'b0;
                    pop_osq            <= 1'b0;
                    ack_mgr_h_cs       <= RD_NEXT_OSQ_AFTER_CQE;
                end

                // -------- RD_NEXT_OSQ_AFTER_CQE：继续扫描下一条 OSQ --------
                // CQE 事务已经提交给子 FSM。这里请求下一条 OSQ，并重新发起 ACK/NAK 状态读。
                RD_NEXT_OSQ_AFTER_CQE: begin
                    cqe_proc_fifo_wren   <= 1'b0;
                    ack_mgr_h_cs         <= UPD_CQHD;
                    o_rd_osq             <= 1'b1;
                    acknacked_psn_rd_req <= 1'b1;
                end

                // -------- UPD_CQHD：等下一条 OSQ + ack 上下文，锁存后进 CHECK_OSQ --------
                UPD_CQHD: begin
                    o_rd_osq             <= 1'b0;
                    acknacked_psn_rd_req <= 1'b0;
                    cqe_proc_fifo_wren   <= 1'b0;
                    if (i_rd_osq_vld_r2) begin
                        osq_entry_msn       <= i_osq_entry_msn;
                        osq_entry_end_psn   <= i_osq_entry_end_psn;
                        osq_entry_start_psn <= i_osq_entry_start_psn;
                        osq_opcode          <= i_osq_opcode;
                        osq_wrid            <= i_osq_wrid;
                        osq_wqe_retried     <= i_osq_wqe_retried;
                        osq_wqe_fatal_push  <= i_osq_wqe_fatal_push;
                        osq_wqe_nack_resp   <= i_osq_wqe_nack_resp;
                        osq_empty           <= i_osq_empty || osq_empty_ff || osq_empty_2ff;
                    end
                    if (acknacked_psn_rd_req_done) begin
                        ack_mgr_h_cs          <= CHECK_OSQ;
                        phase_bit_acknack_psn <= i_phase_bit_acknack_psn;
                        osq_acknacked_psn     <= i_osq_acknacked_psn;
                    end
                end

                // -------- CHECK_OSQ：二级决策 --------
                // 在一次仲裁窗口中连续完成多个 WQE 的扫描路径。
                // 决策相较 CHECK_ERROR 差异：
                //   - 发现需重传 → 先结束完成批次，再进 START_RETRANSMIT；
                //   - 若条目也已完成 → 回 CHECK_ERROR，并通过 pending_cqdb_update 触发下一拍 WRITE_CQE。
                CHECK_OSQ: begin
                    if ((osq_empty) || ((osq_nacked_sampled | osq_timed_out) & ~osq_wqe_nack_resp) || (~qp_fatal_flush & qp_fatal_or_recovery)) ack_mgr_h_cs <= FINISH_OSQ_BATCH;
                    else if (current_qp_fatal | (arbitrated_indx_ff == QP1 && ~osq_empty)) ack_mgr_h_cs <= CHECK_ERROR;

                    // NACK 指向 packet n 时，PSN 小于 n 的 packet 已经被确认。
                    // 因此，进入重传前要先把已经完成覆盖到的 OSQ 条目提交 CQE 并 pop。
                    else if (psn_0_nacked) begin  // NACK 低位 PSN 为 0 时的回绕边界处理
                        if ((osq_entry_end_psn < osq_entry_start_psn) | (osq_entry_start_psn == 'b0)) begin  // 情况 1/2：OSQ 条目跨回绕点，或从低位 0 开始
                            ack_mgr_h_cs  <= FINISH_OSQ_BATCH;  // 先结束完成批次，再进入重传
                            retransmit_en <= 1'b1;
                        end else begin  // 情况 3：OSQ 条目未跨回绕点
                            ack_mgr_h_cs  <= WRITE_CQE;
                            retransmit_en <= 1'b0;
                        end
                    end else if (((osq_entry_end_psn > (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) && osq_timed_out && ~phase_mismatch) | ((osq_entry_end_psn < (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) && osq_timed_out & phase_mismatch)) begin
                        ack_mgr_h_cs  <= FINISH_OSQ_BATCH;
                        retransmit_en <= 1'b1;
                    end else if (((osq_entry_end_psn >= (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) && osq_nacked_sampled && ~phase_mismatch) | ((osq_entry_end_psn <= (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0])) && osq_nacked_sampled && phase_mismatch)) begin
                        ack_mgr_h_cs  <= FINISH_OSQ_BATCH;
                        retransmit_en <= 1'b1;
                    end else if ((osq_entry_end_psn > osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0]) && ~phase_mismatch) begin  // no more completed WQEs to process
                        ack_mgr_h_cs  <= FINISH_OSQ_BATCH;
                        retransmit_en <= 1'b0;
                    end else if ((osq_entry_end_psn < osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0]) && phase_mismatch) begin
                        ack_mgr_h_cs  <= FINISH_OSQ_BATCH;
                        retransmit_en <= 1'b0;
                    end else begin
                        ack_mgr_h_cs        <= CHECK_ERROR;
                        retransmit_en       <= 1'b0;
                        pending_cqdb_update <= 1'b1;
                    end
                end

                // -------- FINISH_OSQ_BATCH：本轮扫描收尾 --------
                // 该状态不写 SQ 完成 Doorbell，只根据 retransmit_en / qp_fatal_flush 分流：
                //   retransmit_en=1 → START_RETRANSMIT
                //   qp_fatal_flush=1 → CHECK_FLUSH_OSQ
                //   否则 → ARBITRATE_OSQ
                FINISH_OSQ_BATCH: begin
                    cqe_proc_fifo_wren <= 1'b0;

                    if (retransmit_en) begin
                        ack_mgr_h_cs  <= START_RETRANSMIT;
                        retransmit_en <= 1'b0;
                    end else if (qp_fatal_flush) begin
                        ack_mgr_h_cs <= CHECK_FLUSH_OSQ;
                    end else begin
                        ack_mgr_h_cs <= ARBITRATE_OSQ;
                    end
                end

                // 短暂进入 ARBITRATE_OSQ，拉高 o_arbitrate，请仲裁器重新选择下一个可处理 QP。
                // -------- ARBITRATE_OSQ：清扫描类写使能，回 IDLE --------
                ARBITRATE_OSQ: begin
                    cqe_proc_fifo_wren   <= 1'b0;
                    retry_proc_fifo_wren <= 1'b0;
                    ack_mgr_h_cs         <= IDLE;
                    fsm_pushback_valid   <= 1'b0;
                end

                // -------- START_RETRANSMIT：捕获重传参数 + 重试次数检查 --------
                // 1) retry_start_psn = osq_entry_start_psn（被重传的第一条 WQE 起点）
                // 2) completed_dma_len_multiplier = (acknacked_psn - start_psn) + {不含/含 NACK 包}
                //    跨相位：acknacked 在高 bit=1 而 start 在 bit=0 时用 {1,ack}-{0,start}
                //    NAK 时 +0（NAK 指向的包未成功，不计入已完成）
                //    Timeout 时 +1（所有 <= ack 的都已完成）
                // 3) 若 fatal_dis → 直接转 RETRANSMIT
                //    否则：NACK 且 cfg_retry_cnt==0 → GEN_QP_FATAL
                //          Timeout 且 curr_retry_cnt==0 → GEN_QP_FATAL
                //          否则 → RETRANSMIT，wr_retry_cnt=1（触发计数器写回）
                START_RETRANSMIT: begin
                    cqe_proc_fifo_wren <= 1'b0;
                    retry_start_psn    <= osq_entry_start_psn;
                    if ((osq_entry_start_psn[C_OSQ_PSN_WIDTH-1] && ~osq_acknacked_psn[C_OSQ_PSN_WIDTH-1]) && (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] < osq_entry_start_psn)) begin
                        completed_dma_len_multiplier <= ({1'b1, osq_acknacked_psn[C_OSQ_PSN_WIDTH-1 : 0]} - {1'b0, osq_entry_start_psn[C_OSQ_PSN_WIDTH-1:0]} + {23'd0, ~osq_nacked_sampled});
                    end else if (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] >= osq_entry_start_psn) begin
                        completed_dma_len_multiplier <= ({1'b0, osq_acknacked_psn[C_OSQ_PSN_WIDTH-1 : 0]} - {1'b0, osq_entry_start_psn[C_OSQ_PSN_WIDTH-1:0]} + {23'd0, ~osq_nacked_sampled});
                    end else begin
                        completed_dma_len_multiplier <= 'd0;
                    end
                    if (i_cfg_retry_cnt_fatal_dis) begin
                        ack_mgr_h_cs <= RETRANSMIT;
                        wr_retry_cnt <= 1'b1;
                        pop_osq      <= 1'b1;
                    end else begin
                        if ((qp_cfg_retry_cnt_ff == 3'h0) & osq_nacked_sampled & !current_qp_fatal) ack_mgr_h_cs <= GEN_QP_FATAL;
                        else if ((qp_curr_retry_cnt_ff == 3'h0) & osq_timed_out & !current_qp_fatal) ack_mgr_h_cs <= GEN_QP_FATAL;
                        else begin
                            ack_mgr_h_cs <= RETRANSMIT;
                            wr_retry_cnt <= 1'b1;
                            pop_osq      <= 1'b1;
                        end
                    end
                end

                RETRANSMIT: begin
                    wr_retry_cnt         <= 1'b0;
                    pop_osq              <= 1'b0;
                    cqe_proc_fifo_wren   <= 1'b0;
                    retry_proc_fifo_wren <= 1'b0;
                    ack_mgr_h_cs         <= FLUSH_POP_OSQ;
                end

                // -------- FLUSH_POP_OSQ：请求读下一条 OSQ --------
                FLUSH_POP_OSQ: begin
                    o_rd_osq           <= 1'b1;
                    cqe_proc_fifo_wren <= 1'b0;
                    ack_mgr_h_cs       <= CHECK_FLUSH_OSQ;

                end

                // -------- CHECK_FLUSH_OSQ：重传前清理真实 OSQ --------
                // OSQ 未空时继续 pop 后续旧 OSQ 条目，保证同一 QP 的旧 outstanding 记录被清空。
                // OSQ 为空后，写 cqe_proc_fifo 的 3'b100 事务，并同步写 retry_proc_fifo；
                // 子 FSM 处理 3'b100 时向 QP 管理器发起重传请求。
                CHECK_FLUSH_OSQ: begin
                    cqe_proc_fifo_wren   <= 1'b0;
                    retry_proc_fifo_wren <= 1'b0;
                    o_rd_osq             <= 1'b0;

                    if (i_osq_empty || qp_fatal_flush) begin
                        if (qp_fatal_flush) begin
                            ack_mgr_h_cs <= ARBITRATE_OSQ;
                        end else if (!cqe_proc_fifo_full && !retry_proc_fifo_full) begin
                            cqe_proc_fifo_data   <= {arbitrated_indx_ff, qp_cq_intr_en_ff, qp_cqe_write_en_ff, 3'b100, completed_dma_len};
                            cqe_proc_fifo_wren   <= 1'b1;

                            retry_proc_fifo_data <= {retry_start_psn, psn_to_retry, {i_osq_acknacked_msn[23:8], msn_to_retry}};
                            retry_proc_fifo_wren <= 1'b1;

                            ack_mgr_h_cs         <= ARBITRATE_OSQ;
                        end else begin
                            ack_mgr_h_cs <= CHECK_FLUSH_OSQ;
                        end
                    end else begin
                        if (i_rd_osq_vld_r2) begin
                            ack_mgr_h_cs <= RETRANSMIT;
                            pop_osq      <= 1'b1;
                            osq_empty    <= i_osq_empty || osq_empty_ff || osq_empty_2ff;
                        end
                    end
                end

                // -------- GEN_QP_FATAL：重试耗尽 --------
                // 等 i_qp_retry_fatal_rdy 握手后 pop OSQ 条目，再转入 flush 流程。
                // 该状态本身只负责 fatal 请求握手；后续 flush/重传事务是否产生由 CHECK_FLUSH_OSQ 和 qp_fatal_flush 条件决定。
                GEN_QP_FATAL: begin
                    if (i_qp_retry_fatal_rdy) begin
                        ack_mgr_h_cs <= RETRANSMIT;
                        pop_osq      <= 1'b1;
                    end
                end

                default: ack_mgr_h_cs <= IDLE;

            endcase
        end
    end

    always @(posedge core_clk) pmtu_decoded_r <= pmtu_decoded;
    assign completed_dma_len = (completed_dma_len_multiplier * pmtu_decoded_r);

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (32),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (27),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH + 37),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH + 37),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_cqe_proc_fifo (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (cqe_proc_fifo_wren),
        .din          (cqe_proc_fifo_data),
        .rd_en        (cqe_proc_fifo_rden),
        .empty        (cqe_proc_fifo_empty),
        .dout         (cqe_proc_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (cqe_proc_fifo_full),
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
        .READ_DATA_WIDTH    (72),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (72),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_retry_proc_fifo (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (retry_proc_fifo_wren),
        .din          (retry_proc_fifo_data),
        .rd_en        (retry_proc_fifo_rden),
        .empty        (retry_proc_fifo_empty),
        .dout         (retry_proc_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (retry_proc_fifo_full),
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


    always @(posedge core_clk) begin
        if (~core_rstn) begin
            cqe_proc_fifo_rden             <= 1'b0;
            cqe_cfg_rd_req                 <= 1'b0;
            o_qp_cqe_proc_qpid             <= 'h00;
            cqe_proc_cs                    <= CQE_IDLE;
            cqhdptr_only_req               <= 1'b0;
            o_ip2bus_data                  <= 512'h00;
            o_ip2bus_wraddr                <= 'h00;
            o_ip2bus_wen                   <= 1'b0;
            o_ip2bus_len                   <= 'h00;
            retransmit_req                 <= 1'b0;
            axi_mm_req                     <= 1'b0;
            o_qp_cq_hdptr_wrn              <= 1'b0;
            o_qp_cq_hdptr                  <= 16'h0000;
            o_retransmit_reqd              <= 1'b0;
            o_retransmit_qpid              <= 'h00;
            completion_posted              <= 1'b0;
            completion_accepted            <= 1'b0;
            o_clr_timeout_n_nack_set_valid <= 1'b0;
            o_clr_timeout_n_nack_set_id    <= {C_QP_INDX_WIDTH{1'b0}};
            trans_done                     <= 1'b0;
            o_msn_to_retry                 <= 24'd0;
            o_psn_to_retry                 <= 24'd0;
            cqe_proc_intr_en               <= 1'b0;
            cqe_proc_completed_dma_len     <= 32'd0;
            o_retransmit_wqe_bram_din      <= 56'd0;
            retry_proc_fifo_rden           <= 1'b0;
        end else begin
            case (cqe_proc_cs)

                CQE_IDLE: begin
                    cqe_proc_fifo_rden             <= 1'b0;
                    retry_proc_fifo_rden           <= 1'b0;
                    o_clr_timeout_n_nack_set_valid <= 1'b0;
                    trans_done                     <= 1'b0;
                    o_qp_cq_hdptr_wrn              <= 1'b0;

                    if (ack_mgr_h_cs == BYPASS_WAIT_CQE_IDLE && cqe_proc_fifo_empty) begin
                        // cqe_write_en=0 bypass 路径使用同一个 CQ 配置索引端口。
                        // 只有子 FSM 空闲且 FIFO 为空时才切换 qpid，避免影响正在处理的 CQE/重传事务。
                        o_qp_cqe_proc_qpid <= arbitrated_indx_ff;
                        cqe_cfg_rd_req     <= 1'b0;
                    end else if (ack_mgr_h_cs == BYPASS_CQHDPTR_WR) begin
                        // bypass 路径只推进 CQ head pointer，不访问 AXI，也不弹 cqe_proc_fifo。
                        o_qp_cq_hdptr_wrn <= 1'b1;
                        o_qp_cq_hdptr     <= new_cq_hdptr;
                        cqe_cfg_rd_req    <= 1'b0;
                    end else if (~cqe_proc_fifo_empty && ~o_qp_cq_hdptr_wrn) begin
                        o_qp_cqe_proc_qpid         <= cqe_proc_fifo_dout[C_QP_INDX_WIDTH+36 : 37];
                        cqe_proc_intr_en           <= cqe_proc_fifo_dout[36];
                        cqe_proc_completed_dma_len <= cqe_proc_fifo_dout[31:0];

                        if (cqe_proc_fifo_dout[34:32] == 3'b000) begin
                            cqe_proc_cs    <= GET_QP_CONF;
                            cqe_cfg_rd_req <= 1'b1;
                        end else begin
                            cqe_proc_cs    <= GEN_TRANS;
                            cqe_cfg_rd_req <= 1'b0;
                        end
                    end
                end

                GET_QP_CONF: begin
                    if (i_qp_cq_ba_valid && i_qp_cq_depth_valid && i_qp_cq_hdptr_valid) begin
                        cqe_proc_cs    <= GEN_TRANS;
                        cqe_cfg_rd_req <= 1'b0;
                    end
                end

                GEN_TRANS: begin
                    cqe_proc_cs <= WAIT_TRANS_DONE;
                    if (cqe_proc_fifo_dout[34 : 32] == 3'b000) begin
                        // 完成指针始终推进。o_qp_cq_hdptr_wrn 是写脉冲，
                        // i_qp_cq_hdptr_valid 只表示读返回有效，不能作为写完成应答等待。
                        o_qp_cq_hdptr_wrn <= 1'b1;
                        o_qp_cq_hdptr     <= new_cq_hdptr;

                        if (cqe_proc_fifo_dout[35]) begin
                            // cqe_write_en=1：写 CQE 到 DDR
                            o_ip2bus_data     <= {480'h0, cqe_proc_fifo_dout[31:0]};
                            o_ip2bus_wraddr   <= cqe_axi_address;
                            o_ip2bus_wen      <= 1'b1;
                            o_ip2bus_len      <= CQE_SZ_IN_BYTES;
                            axi_mm_req        <= 1'b1;
                            cqhdptr_only_req  <= 1'b0;
                            completion_posted <= cqe_proc_fifo_dout[36];
                        end else begin
                            // cqe_write_en=0：不写 CQE DDR，只推进完成指针。
                            // 该路径不需要等待 AXI 完成，写指针打一拍后立即弹出事务，避免 CQE 子 FSM 成为吞吐瓶颈。
                            o_ip2bus_wen       <= 1'b0;
                            axi_mm_req         <= 1'b0;
                            cqhdptr_only_req   <= 1'b0;
                            completion_posted  <= 1'b0;
                            cqe_proc_fifo_rden <= 1'b1;
                            trans_done         <= 1'b0;
                            cqe_proc_cs        <= CQE_IDLE;
                        end
                    end else if (cqe_proc_fifo_dout[34 : 32] == 3'b100) begin
                        if (!retry_proc_fifo_empty) begin
                            o_retransmit_reqd         <= 1'b1;
                            o_msn_to_retry            <= retry_proc_fifo_dout[23:0];
                            o_psn_to_retry            <= retry_proc_fifo_dout[47:24];
                            o_retransmit_qpid         <= cqe_proc_fifo_dout[C_QP_INDX_WIDTH+36 : 37];
                            o_retransmit_wqe_bram_din <= {retry_proc_fifo_dout[71:48], cqe_proc_completed_dma_len};
                            retransmit_req            <= 1'b1;
                        end else begin
                            cqe_proc_cs <= GEN_TRANS;  // 或保持等待
                        end
                    end else begin
                        // 未定义 sub-opcode，直接丢弃，避免 cqe_proc 卡死
                        o_ip2bus_wen       <= 1'b0;
                        axi_mm_req         <= 1'b0;
                        completion_posted  <= 1'b0;
                        o_qp_cq_hdptr_wrn  <= 1'b0;

                        cqe_proc_fifo_rden <= 1'b1;
                        trans_done         <= 1'b1;
                    end
                end

                WAIT_TRANS_DONE: begin
                    o_ip2bus_wen       <= 1'b0;
                    cqe_proc_fifo_rden <= 1'b0;

                    // CQ head pointer 写口没有独立完成应答。写使能只打一拍，
                    // 后续等待 AXI CQE 写完成或 cqhdptr_only_req 自身完成。
                    o_qp_cq_hdptr_wrn  <= 1'b0;

                    if (trans_done) begin
                        cqe_proc_cs                    <= CQE_IDLE;
                        cqe_proc_fifo_rden             <= 1'b0;
                        retry_proc_fifo_rden           <= 1'b0;
                        o_clr_timeout_n_nack_set_valid <= 1'b0;
                        trans_done                     <= 1'b0;
                        completion_accepted            <= 1'b0;
                        cqhdptr_only_req               <= 1'b0;
                    end else begin
                        if (retransmit_req && i_retransmit_accepted) begin
                            o_retransmit_reqd              <= 1'b0;
                            trans_done                     <= 1'b1;
                            cqe_proc_fifo_rden             <= 1'b1;
                            retransmit_req                 <= 1'b0;
                            retry_proc_fifo_rden           <= 1'b1;
                            o_clr_timeout_n_nack_set_valid <= 1'b1;
                            o_clr_timeout_n_nack_set_id    <= o_qp_cqe_proc_qpid;
                        end else if (cqhdptr_only_req) begin
                            // 不写 CQE DDR，只推进完成指针；写脉冲已在上一拍发出。
                            trans_done          <= 1'b1;
                            cqe_proc_fifo_rden  <= 1'b1;
                            cqhdptr_only_req    <= 1'b0;
                            completion_accepted <= 1'b0;
                        end else if (axi_mm_req && i_lrdma_axi_master_done) begin
                            // 写 CQE DDR 完成后再弹 FIFO；CQ head pointer 写脉冲已发出。
                            trans_done          <= 1'b1;
                            cqe_proc_fifo_rden  <= 1'b1;
                            axi_mm_req          <= 1'b0;
                            completion_accepted <= completion_posted;
                            completion_posted   <= 1'b0;
                        end
                    end
                end

                default: cqe_proc_cs <= CQE_IDLE;

            endcase
        end
    end

    assign fsm_arbitrated_indx = arbitrated_indx_ff;

    `MSFF_R(osq_wqe_nack_resp_ff, i_osq_wqe_nack_resp, core_clk, ~core_rstn)

    `MSFF_R(msn_to_retry, (ack_mgr_h_cs == START_RETRANSMIT) ? osq_entry_msn : msn_to_retry, core_clk, ~core_rstn)
    `MSFF_R(psn_to_retry, (ack_mgr_h_cs == START_RETRANSMIT) ? full_psn_to_retry : psn_to_retry[23:0], core_clk, ~core_rstn)

    // Retry count - saturates at all FFs
    `MSFF_R(retry_count_ff, (((cqe_proc_cs == WAIT_TRANS_DONE) && i_retransmit_accepted && o_retransmit_reqd) ? (~(&retry_count_ff) ? (retry_count_ff + 1'b1) : retry_count_ff) : retry_count_ff), core_clk, ~core_rstn)
    assign o_stat_retry_count = retry_count_ff;


    //--- ACK/NAK 输入采样 -----------------------------------------------------
    // FSM 处理某个 QP 时，RX 侧可能又收到同一个 QP 的新 ACK/NAK。
    // 为避免主 FSM 在一个决策窗口里看到前后不一致的 PSN、phase bit 和 nacked 状态，
    // 这里在 acknacked_psn_rd_req_done 有效时统一采样。
    // phase bit 的更新比 PSN 晚一个周期，因此 FSM 需要等待读完成窗口后再使用采样值。
    `MSFF_R(phase_bit_acknack_psn_sampled, (acknacked_psn_rd_req_done ? i_phase_bit_acknack_psn : phase_bit_acknack_psn_sampled), core_clk, ~core_rstn)

    `MSFF_R(osq_nacked_sampled, (acknacked_psn_rd_req_done ? i_osq_nacked : osq_nacked_sampled), core_clk, ~core_rstn)

    //---------------------------------------

    // Timeout 重传起点选择：根据 OSQ 条目是否跨低位回绕点、ACK/NAK PSN phase 是否不一致决定高位拼接。
    // 情况 1：OSQ 未跨回绕点，phase 一致，重传起点 = {osq_acknacked_psn_sampled[23:0], i_osq_entry_start_psn}
    // 8'hfd
    // 8'hfe    
    // 8'hff___
    // 8'h00    osq_start_psn
    // 8'h01    acknacked_psn
    // 8'h02    osq_end_psn
    // 情况 2：OSQ 未跨回绕点，phase 不一致，重传起点 = {osq_acknacked_psn_sampled[23:0] + 1'b1, i_osq_entry_start_psn}
    // 8'hfd
    // 8'hfe    
    // 8'hff___ acknacked_psn
    // 8'h00    osq_start_psn
    // 8'h01    osq_end_psn
    // 8'h02
    // 情况 3：OSQ 跨回绕点，phase 一致，重传起点 = {osq_acknacked_psn_sampled[23:8] - 1'b1, i_osq_entry_start_psn}
    // 8'hfd
    // 8'hfe    osq_start_psn
    // 8'hff___
    // 8'h00    acknacked_psn
    // 8'h01    osq_end_psn
    // 8'h02
    // 情况 4：OSQ 跨回绕点，phase 不一致，重传起点 = {osq_acknacked_psn_sampled[23:8], i_osq_entry_start_psn}
    // 8'hfd
    // 8'hfe    osq_start_psn
    // 8'hff___ acknacked_psn
    // 8'h00    
    // 8'h01    osq_end_psn
    // 8'h02

    //-----------------------------------------------------------------
    // full_psn_to_retry：重传起始 PSN 的源
    //   NAK → ack 处直接重传（NAK 指向的包本身未成功）
    //   Timeout  → ack+1 处重传（PSN <= ack 的 packet 都已完成，Go-Back-N 从下一个开始）
    //-----------------------------------------------------------------
    assign full_psn_to_retry     = osq_nacked_sampled ? osq_acknacked_psn[23:0] : (osq_acknacked_psn + 24'h000001);

    // fsm_i_arbitrated_indx：IDLE 且仲裁完成时透传 i_arbitrated_indx，其余拍用 ff（供上游 forward 合成）
    assign fsm_i_arbitrated_indx = (i_arbitration_done && ack_mgr_h_cs == IDLE) ? i_arbitrated_indx : arbitrated_indx_ff;

    // 中断：CQE AXI 写完成，并且该 QP 允许完成中断
    assign o_intr_wqe_cmpl_sts   = completion_accepted && cqe_proc_intr_en;

    // osq_timed_out 在 acknacked_psn 读完成窗口采样，与 osq_nacked_sampled 保持同一决策时点。
    // osq_timed_out 采样：与 osq_nacked_sampled 同策略，用 acknacked 读完成窗口	 
    `MSFF_R(osq_timed_out, (acknacked_psn_rd_req_done ? osq_timed_out_sticky_fsm_arbitrated_indx : osq_timed_out), core_clk, ~core_rstn)

    // psn_0_nacked：NAK 的 PSN 恰好为 0（PSN 回绕特殊情况）
    assign psn_0_nacked             = osq_nacked_sampled && (osq_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] == 'b0);
    assign o_arbitrated_qp_for_intr = o_qp_cqe_proc_qpid;


    `MSFF_R(qp_cq_ba_ff, (i_qp_cq_ba_valid ? i_qp_cq_ba : qp_cq_ba_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_depth_ff, (i_qp_cq_depth_valid ? i_qp_cq_depth : qp_cq_depth_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_hdptr_ff, (i_qp_cq_hdptr_valid ? i_qp_cq_hdptr : qp_cq_hdptr_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_intr_en_ff, (i_qp_conf_valid ? i_qp_conf[3] : qp_cq_intr_en_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cqe_write_en_ff, (i_qp_conf_valid ? i_qp_conf[5] : qp_cqe_write_en_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_pmtu_ff, (i_qp_conf_valid ? i_qp_conf[10:8] : qp_pmtu_ff), core_clk, ~core_rstn)


    // i_arbitrated_indx 被锁存到 arbitrated_indx_ff，作为处理 QP 上下文索引。
    `MSFF_R(arbitrated_indx_ff, ((i_arbitration_done && (ack_mgr_h_cs == IDLE)) ? i_arbitrated_indx : arbitrated_indx_ff), core_clk, ~core_rstn)

    `MSFF_R(arbitrated_indx_2ff, arbitrated_indx_ff, core_clk, ~core_rstn)


    `MSFF_R(osq_empty_ff, i_osq_empty, core_clk, ~core_rstn)
    `MSFF_R(osq_empty_2ff, osq_empty_ff, core_clk, ~core_rstn)

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_i_first_outgoing_pkt_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_first_outgoing_pkt_valid),
        .wr_addr (init_start ? init_count : i_first_outgoing_pkt_id),
        .wr_data (init_start ? 1'b1 : i_first_outgoing_pkt_data),
        .rd_en   (1'b1),
        .rd_addr (i_wqe_qpid),
        .rd_data (i_first_outgoing_pkt_rd_data_int_0)
    );

    assign i_first_outgoing_pkt_rd_data_0 = (i_first_outgoing_pkt_valid_pending && i_first_outgoing_pkt_id_pending == i_wqe_qpid) ? i_first_outgoing_pkt_data_pending : i_first_outgoing_pkt_rd_data_int_0;

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_last_osq_psn (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || last_osq_psn_valid),
        .wr_addr    (init_start ? init_count : last_osq_psn_id),
        .wr_data    (init_start ? 1'b0 : last_osq_psn_data),
        .wr_en_aux  (last_osq_psn_valid_aux),
        .wr_addr_aux(last_osq_psn_id_aux),
        .wr_data_aux(last_osq_psn_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (arbitrated_indx_ff),
        .rd_data    (last_osq_psn_rd_data)
    );

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_phase_bit_osq_psn_ff_0 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || phase_bit_osq_psn_ff_valid),
        .wr_addr    (init_start ? init_count : phase_bit_osq_psn_ff_id),
        .wr_data    (init_start ? 1'b0 : phase_bit_osq_psn_ff_data),
        .wr_en_aux  (phase_bit_osq_psn_ff_valid_aux),
        .wr_addr_aux(phase_bit_osq_psn_ff_id_aux),
        .wr_data_aux(phase_bit_osq_psn_ff_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (arbitrated_indx_ff),
        .rd_data    (phase_bit_osq_psn_ff_rd_data_0)
    );

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_phase_bit_osq_psn_ff_1 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || phase_bit_osq_psn_ff_valid),
        .wr_addr    (init_start ? init_count : phase_bit_osq_psn_ff_id),
        .wr_data    (init_start ? 1'b0 : phase_bit_osq_psn_ff_data),
        .wr_en_aux  (phase_bit_osq_psn_ff_valid_aux),
        .wr_addr_aux(phase_bit_osq_psn_ff_id_aux),
        .wr_data_aux(phase_bit_osq_psn_ff_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (o_qp_cqe_proc_qpid),
        .rd_data    (phase_bit_osq_psn_ff_rd_data_1)
    );

    wire [C_OSQ_PSN_WIDTH-1:0] wqe_send_start_psn_less_one;
    wire [C_OSQ_PSN_WIDTH-1:0] retry_psn_less_one;

    always @(posedge core_clk) begin : last_osq_psn_ff
        if (~core_rstn) begin
            last_osq_psn_valid     <= 1'b0;
            last_osq_psn_id        <= {C_QP_INDX_WIDTH{1'b0}};
            last_osq_psn_data      <= 1'b1;
            last_osq_psn_valid_aux <= 1'b0;
            last_osq_psn_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
            last_osq_psn_data_aux  <= 1'b1;
        end else begin
            if (i_first_outgoing_pkt_rd_data_0 & i_wqe_push_req) begin
                last_osq_psn_valid <= 1'b1;
                last_osq_psn_id    <= i_wqe_qpid;
                last_osq_psn_data  <= wqe_send_start_psn_less_one[C_OSQ_PSN_WIDTH-1];
            end else begin
                last_osq_psn_valid <= 1'b0;
            end

            if (i_first_outgoing_pkt_rd_data_0 & i_wqe_push_req & arbitrated_indx_ff == i_wqe_qpid) begin
                last_osq_psn_valid_aux <= 1'b0;
            end else begin
                if (~(i_osq_empty | osq_empty_ff | osq_empty_2ff) & (i_rd_osq_vld_r2 & ~i_rd_osq_vld_r1)) begin
                    last_osq_psn_valid_aux <= 1'b1;
                    last_osq_psn_id_aux    <= arbitrated_indx_ff;
                    last_osq_psn_data_aux  <= i_osq_entry_end_psn[C_OSQ_PSN_WIDTH-1];
                end else if (ack_mgr_h_cs == RETRANSMIT) begin
                    last_osq_psn_valid_aux <= 1'b1;
                    last_osq_psn_id_aux    <= arbitrated_indx_ff;
                    last_osq_psn_data_aux  <= retry_psn_less_one[C_OSQ_PSN_WIDTH-1];
                end else begin
                    last_osq_psn_valid_aux <= 1'b0;
                end
            end
        end
    end

    always @(posedge core_clk) begin : phase_bit_osq_psn_ff
        if (~core_rstn) begin
            phase_bit_osq_psn_ff_valid     <= 1'b0;
            phase_bit_osq_psn_ff_id        <= {C_QP_INDX_WIDTH{1'b0}};
            phase_bit_osq_psn_ff_data      <= 1'b0;
            phase_bit_osq_psn_ff_valid_aux <= 1'b0;
            phase_bit_osq_psn_ff_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
            phase_bit_osq_psn_ff_data_aux  <= 1'b0;
        end else begin
            if (i_first_outgoing_pkt_valid && i_first_outgoing_pkt_data) begin
                phase_bit_osq_psn_ff_valid <= 1'b1;
                phase_bit_osq_psn_ff_id    <= i_first_outgoing_pkt_id;
                phase_bit_osq_psn_ff_data  <= 1'b0;
            end else begin
                phase_bit_osq_psn_ff_valid <= 1'b0;
            end

            if (((i_rd_osq_vld_r2 && ~i_rd_osq_vld_r1) || upd_phase_retransmit_pop) && ~(i_first_outgoing_pkt_valid && i_first_outgoing_pkt_data && i_first_outgoing_pkt_id == arbitrated_indx_ff)) begin
                phase_bit_osq_psn_ff_valid_aux <= 1'b1;
                phase_bit_osq_psn_ff_id_aux    <= arbitrated_indx_ff;
                phase_bit_osq_psn_ff_data_aux  <= phase_bit_osq_psn;
            end else begin
                phase_bit_osq_psn_ff_valid_aux <= 1'b0;
            end
        end
    end

    // phase_bit_osq_psn：推导 OSQ 条目的 phase bit
    //   条件：条目 end_psn 最高 bit 为 0 且 last_osq_psn 最高 bit 为 1 且 OSQ 非空 → 相位翻转
    //   否则沿用 BRAM 记录的上一相位
    assign phase_bit_osq_psn               = ((~i_osq_entry_end_psn[C_OSQ_PSN_WIDTH-1] && last_osq_psn_for_arbitrated_idx) & ~(i_osq_empty | osq_empty_ff | osq_empty_2ff)) ? ~phase_bit_osq_psn_ff_rd_data_0 : phase_bit_osq_psn_ff_rd_data_0;

    // phase_mismatch：ACK 相位与 OSQ 相位不一致（PSN 已跨 rollover 点）
    assign phase_mismatch                  = (phase_bit_acknack_psn_sampled ^ phase_bit_osq_psn_ff_rd_data_0);

    // ACK/NAK PSN 和 OSQ PSN 都可能跨过低位 PSN 回绕点。
    // 如果两边同时跨回绕点，phase 差异会相互抵消。
    assign last_osq_psn_for_arbitrated_idx = last_osq_psn_rd_data;
    assign wqe_send_start_psn_less_one     = i_wqe_send_start_psn - 1'b1;
    assign retry_psn_less_one              = psn_to_retry - 1'b1;

    // 重传请求被接受后，顶层用该 phase bit 回写 acknacked_psn 表，
    // 使重传后的 acknacked_psn 相位与 OSQ 相位保持一致。
    assign o_phase_to_update_post_retry    = phase_bit_osq_psn_ff_rd_data_1;

    // 重传后更新 phase：1 拍窗口（RETRANSMIT 态且 pop_osq=1）
    `MSFF_R(upd_phase_post_retransmit, ((ack_mgr_h_cs == START_RETRANSMIT) ? 1'b1 : (pop_osq ? 1'b0 : upd_phase_post_retransmit)), core_clk, ~core_rstn)
    assign upd_phase_retransmit_pop  = upd_phase_post_retransmit & pop_osq;

    // CQ head pointer 递增 + wrap
    assign new_cq_hdptr              = (qp_cq_hdptr_ff == qp_cq_depth_ff) ? 16'h1 : (qp_cq_hdptr_ff + 1'b1);  // Updated CQ headpointer
    assign qp_cq_hdptr_base_0        = (qp_cq_hdptr_ff == qp_cq_depth_ff) ? 16'h0 : qp_cq_hdptr_ff;
    assign cqe_axi_address           = qp_cq_ba_ff + (qp_cq_hdptr_base_0 * CQE_ENTRY_SIZE);

    assign o_arbitrate               = (ack_mgr_h_cs == ARBITRATE_OSQ) ? 1'b1 : 1'b0;
    assign o_sampled_arbitrated_indx = arbitrated_indx_ff;

    //----------------------

    assign o_qp_cq_hdptr_req         = cqe_cfg_rd_req || main_cqhdptr_rd_req || o_qp_cq_hdptr_wrn;
    assign o_qp_cq_depth_req         = cqe_cfg_rd_req || main_cqhdptr_rd_req;
    assign o_qp_cq_ba_req            = cqe_cfg_rd_req;
    assign o_qp_conf_req             = cfg_rd_req;
    //-----------------------Re-load the retry counter values-------//
    `MSFF_R(qp_curr_retry_cnt_ff, ((i_qp_curr_retry_cnt_rd_rdy & o_qp_curr_retry_cnt_rd_req) ? i_qp_curr_retry_cnt : qp_curr_retry_cnt_ff), core_clk, ~core_rstn)
    `MSFF_R(qp_cfg_retry_cnt_ff, ((i_qp_cfg_retry_cnt_rd_rdy & o_qp_cfg_retry_cnt_rd_req) ? i_qp_cfg_retry_cnt : qp_cfg_retry_cnt_ff), core_clk, ~core_rstn)

    //-----------------------------------------------------------------
    // Retry 计数器接口
    //   读：cfg_rd_req 同时拉两个 rd_req
    //   写：1) wr_retry_cnt=1 时写回（START_RETRANSMIT→RETRANSMIT 拍）
    //          NAK → cfg_retry_cnt-1（reset-to-max-1，对端响应说明 peer 在，重置计数）
    //          Timeout  → curr_retry_cnt-1（peer 沉默，累减）
    //       2) WRITE_CQE 时写回 cfg_retry_cnt（正常完成后清回上限）
    //       3) WAIT_VALID_DATA + reload_en=1 时写 cfg_retry_cnt（per-QP 重载使能）
    //-----------------------------------------------------------------
    assign o_qp_curr_retry_cnt_rd_req = cfg_rd_req;
    assign o_qp_cfg_retry_cnt_rd_req  = cfg_rd_req;

    wire retry_cnt_reload_on_success;
    assign retry_cnt_reload_on_success = (ack_mgr_h_cs == WRITE_CQE) && !osq_timed_out && !osq_nacked_sampled && !current_qp_fatal;
    assign o_qp_curr_retry_cnt_wr_req = wr_retry_cnt | retry_cnt_reload_on_success | ((ack_mgr_h_cs == WAIT_VALID_DATA) & i_reload_retry_cnt_en_rd_data);
    assign o_qp_curr_retry_cnt_wr = wr_retry_cnt ? (osq_nacked_sampled ? (qp_cfg_retry_cnt_ff - 1'b1) : (qp_curr_retry_cnt_ff - 1'b1)) : qp_cfg_retry_cnt_ff;

    assign o_qp_retry_fatal_qpid = arbitrated_indx_ff;
    assign o_qp_curr_retry_cnt_qpid = arbitrated_indx_ff;
    assign o_qp_retry_fatal_req = (ack_mgr_h_cs == GEN_QP_FATAL) ? 1'b1 : 1'b0;
    assign {reload_retry_cnt_valid_pending_1, reload_retry_cnt_id_pending_1, reload_retry_cnt_data_pending_1, reload_retry_cnt_valid_pending_0, reload_retry_cnt_id_pending_0, reload_retry_cnt_valid, reload_retry_cnt_id, reload_retry_cnt_data} = i_reload_retry_cnt_en;
    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_i_reload_retry_cnt_en (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || reload_retry_cnt_valid),
        .wr_addr (init_start ? init_count : reload_retry_cnt_id),
        .wr_data (init_start ? 1'b1 : reload_retry_cnt_data),
        .rd_en   (1'b1),
        .rd_addr (arbitrated_indx_ff),
        .rd_data (i_reload_retry_cnt_en_rd_data_int)
    );

    //-----------------------------------------------------------------
    // reload_retry_cnt_en 分层 forward：
    //   优先 pending_0 → pending_1 → BRAM 读出（解决同拍写读竞争）
    //-----------------------------------------------------------------
    always @(*) begin
        if (reload_retry_cnt_valid_pending_0 && reload_retry_cnt_id_pending_0 == arbitrated_indx_ff) begin
            i_reload_retry_cnt_en_rd_data = 1'b0;
        end else if (reload_retry_cnt_valid_pending_1 && reload_retry_cnt_id_pending_1 == arbitrated_indx_ff) begin
            i_reload_retry_cnt_en_rd_data = reload_retry_cnt_data_pending_1;
        end else begin
            i_reload_retry_cnt_en_rd_data = i_reload_retry_cnt_en_rd_data_int;
        end
    end

    //---Once qp_fatal & dma_idle & i_en_osq_flush_on_qp_fatal is detected
    //then proceed error completion preparation of rest of OSQ entries without
    //checking for i_dma_in_idle & i_en_osq_flush_on_qp_fatal
    assign o_pop_osq = pop_osq;

    // qp_fatal_flush 锁存 fatal flush 模式。进入该模式后，本轮扫描持续生成错误完成并清理 OSQ，
    // 不再每一拍重复检查 dma_in_idle 和 i_en_osq_flush_on_qp_fatal。
    //-----------------------------------------------------------------
    // qp_fatal_flush：CHECK_ERROR 命中本 QP fatal、DMA 空闲且允许 OSQ fatal 冲刷时置位
    // 就锁存 1，CHECK_OSQ/CHECK_ERROR 后续循环中忽略 dma_in_idle 条件持续扫 OSQ，
    // 直到 ARBITRATE_OSQ 清 0（本 QP 扫完）
    //-----------------------------------------------------------------
    always @(posedge core_clk) begin
        if (~core_rstn) qp_fatal_flush <= 'h0;
        else if ((ack_mgr_h_cs == CHECK_ERROR) & current_qp_fatal & i_dma_in_idle & i_en_osq_flush_on_qp_fatal) qp_fatal_flush <= 'h1;
        else if (ack_mgr_h_cs == ARBITRATE_OSQ) qp_fatal_flush <= 'h0;
    end

    // 状态寄存器：|14'h0|acks_to_process|ack_mgr_h_cs[4:0]|pad|arbitrated_indx[C_QP_INDX_WIDTH-1:0]|
    assign o_ack_mgr_sts = {14'h0, i_acks_to_process, ack_mgr_h_cs, {(11 - C_QP_INDX_WIDTH) {1'b0}}, arbitrated_indx_ff};

endmodule
