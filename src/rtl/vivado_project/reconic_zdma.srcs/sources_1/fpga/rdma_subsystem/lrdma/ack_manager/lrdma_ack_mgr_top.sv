`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_ack_mgr_top — ACK 管理器顶层
//
// 【模块职责】
// ACK 管理器连接 TX 发送路径、RX 响应包处理路径、QP 管理器和 AXI Master。TX 每发送
// 一条 WQE 后在 OSQ 中登记 outstanding 条目；RX 收到 ACK/NAK 后更新对应 QP 的
// ACK/NAK PSN/MSN 和 NACK 状态；retry timer 在超时后写入 timeout sticky。本顶层把
// 这些事件汇聚到仲裁器，再交给 lrdma_ack_mgr_fsm 完成 OSQ 扫描、完成提交、重传触发
// 和 fatal flush。
//
// 【完成写回方式】
// ACK 管理器 AXI Master 向 DDR 中的 CQ 写 4B CQE，不写 SQ completion
// Doorbell。CQ head pointer 由 FSM 通过 QP 配置接口写回，保证下一条
// CQE 写入正确位置。
//
// 【主要数据结构】
// 1) OSQ（Outstanding SQ Queue）：按 QP 记录已发出但尚未完成确认的 WQE。entry 保存
//    start_psn、end_psn、MSN、opcode、WRID、retried 和 fatal_push。
// 2) ACK/NAK PSN/MSN 表：PSN 表为 26 bit，格式为 {phase_bit, osq_nacked, psn[23:0]}；
//    MSN 表为 24 bit。两张表都通过 lrdma_multiport_qp_state_table 做多源读写仲裁。
// 3) lrdma_common_qpid_fifo_arbiter：汇聚 ACK/NAK、QP1 特殊事件、timeout/fatal pushback 和 FSM
//    pushback，选择下一个需要处理的 QP。
// 4) lrdma_ack_retry_timer：按 QP 维护重试超时计数，超时后把 QP 推入处理队列。
//
// 【中断路径】
// CQE DDR 写完成且该事务允许中断时，FSM 输出完成状态；顶层按 QP 写 per-QP
// 中断状态表，再 OR-reduce 成 o_intr_wqe_cmpl。若 QP 配置关闭 CQE DDR 写，则只推进
// CQ head pointer，不产生完成中断。软件清除寄存器会清对应 QP 的中断状态。
//
// 【AXI 接口】
// 本模块的 AXI Master 发起 CQE 写 DDR 事务；读通道请求常驱 0。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps

module lrdma_ack_mgr_top #(
    parameter C_M_AXI_ADDR_WIDTH = 64,    // AXI master 地址位宽，用于 CQE 写 DDR
    parameter C_NUM_QP           = 2048,  // 支持的最大 QP 数
    parameter C_QP_INDX_WIDTH    = 11,    // QP 索引位宽 = log2(C_NUM_QP)
    parameter C_OS_Q_DEPTH       = 16,    // 每个 QP 的 OSQ 最大深度（并发 in-flight WQE 数上限）
    parameter C_OS_Q_INDX_WIDTH  = 4,     // OSQ 索引位宽 = log2(C_OS_Q_DEPTH)
    parameter C_OSQ_PSN_WIDTH    = 16,    // OSQ 内存放的 PSN 位宽（低 16 位，完整 PSN 仍为 24）
    parameter RESP_WIDTH         = 2
) (
    input wire core_clk,  // 核心时钟
    input wire core_rstn, // 低有效同步复位

    //==========================================================================
    // TX 引擎 → OSQ push 接口
    // 每发送一条 WQE，TX 引擎会把元数据推入本模块的 OSQ，用于后续 ACK/NAK 匹配
    //==========================================================================
    input  wire [C_QP_INDX_WIDTH-1:0] i_wqe_qpid,            // WQE 所属 QP ID
    input  wire [                7:0] i_wqe_opcode,          // 发送侧 WQE opcode；普通数据路径只登记 RDMA WRITE
    input  wire [               15:0] i_wqe_wrid,            // 用户 WRID（写入 CQE 用）
    input  wire                       i_wqe_retried,         // 此 WQE 是重试请求
    input  wire                       i_wqe_fatal_push,      // QP 处于 fatal 状态时 flush-push 的 WQE
    input  wire [               23:0] i_wqe_send_end_psn,    // 此 WQE 最后一个 WRITE 分片包的 PSN
    // i_wqe_send_start_psn:
    //   正常发送：QP SQPSN，即本 WQE 首包 BTH PSN。
    //   retransmit 重传：来自 QP 管理器写入的 retransmit_wqe_bram[55:32]，
    //   即 ACK 管理器的 retry_start_psn。该字段用于 OSQ start_psn 低位记录
    //   和后续 completed_dma_len 计算；真正线上 BTH PSN 来自回退后的 SQPSN，
    //   也就是 psn_to_retry，不是该字段。
    input  wire [               23:0] i_wqe_send_start_psn,  // 本次发送（首发/重传）的首包 PSN
    input  wire [               23:0] i_wqe_send_msn,        // 本 WQE 的 MSN（Message Seq Num）
    input  wire                       i_wqe_push_req,        // 推入 OSQ 请求
    output wire                       o_wqe_push_rdy,        // 恒为 1；OSQ full 状态通过 o_osq_full_* 反馈给发送调度侧

    //==========================================================================
    // OSQ full/empty 状态广播 → TX 引擎和 QP 管理器据此暂停该 QP 的新 WQE 发送
    //==========================================================================
    output wire                        o_osq_full_set_valid,     // OSQ 由非满变满 → 置位
    output wire [C_QP_INDX_WIDTH -1:0] o_osq_full_set_id,
    output wire                        o_osq_full_reset_valid,   // OSQ 由满变非满 → 清除
    output wire [C_QP_INDX_WIDTH -1:0] o_osq_full_reset_id,
    output wire                        o_osq_empty_set_valid,    // OSQ 变空 → 置位
    output wire [C_QP_INDX_WIDTH -1:0] o_osq_empty_set_id,
    output wire                        o_osq_empty_reset_valid,  // OSQ 变非空 → 清除
    output wire [C_QP_INDX_WIDTH -1:0] o_osq_empty_reset_id,

    //==========================================================================
    // TX 引擎 → 通告"QP 的 NACK 响应已处理完毕"（多级 forward 合成）
    //==========================================================================
    input wire                        i_osq_wqe_nack_resp_valid,          // 正常写口
    input wire [C_QP_INDX_WIDTH -1:0] i_osq_wqe_nack_resp_id,
    input wire                        i_osq_wqe_nack_resp_data,
    input wire                        i_osq_wqe_nack_resp_valid_aux,      // dual-write 辅助写口
    input wire [C_QP_INDX_WIDTH -1:0] i_osq_wqe_nack_resp_id_aux,
    input wire                        i_osq_wqe_nack_resp_data_aux,
    input wire                        i_osq_wqe_nack_resp_valid_pending,  // forward 解同拍竞争
    input wire [C_QP_INDX_WIDTH -1:0] i_osq_wqe_nack_resp_id_pending,
    input wire                        i_osq_wqe_nack_resp_data_pending,

    //==========================================================================
    // 重试/错误恢复配置
    //==========================================================================
    input wire                        i_cfg_retry_cnt_fatal_dis,  // 1：禁用"重试耗尽→fatal"（无限重试）
    input wire                        i_qp_fatal_valid,           // 外部（QP 管理器）写 QP fatal 标志
    input wire [C_QP_INDX_WIDTH -1:0] i_qp_fatal_id,
    input wire                        i_qp_fatal_data,

    //==========================================================================
    // 状态观测
    //==========================================================================
    output wire [(4*C_QP_INDX_WIDTH+4)-1:0] o_osq_nacked,  // timeout/NACK 状态事件输出
                                                           // {set_valid,set_id,set_pending,set_pending_id,reset_valid,reset_id,reset_pending,reset_pending_id}

    //==========================================================================
    // QP disable（QP 管理器 通告 QP 即将禁用）
    //==========================================================================
    input wire                        i_qp_disable_pulse_set_valid,
    input wire [C_QP_INDX_WIDTH -1:0] i_qp_disable_pulse_set_id,

    //==========================================================================
    // 状态/统计输出
    //==========================================================================
    output wire [31:0] o_ack_mgr_sts,      // FSM 状态寄存器
    output wire [31:0] o_stat_retry_count, // 累计重传次数（饱和）

    //==========================================================================
    // 完成中断
    //==========================================================================
    input  wire                         i_intr_en_wqe_cmpl,                   // 全局中断使能
    input  wire                         i_intr_clr_wqe_cmpl,                  // 全局清除
    output wire                         o_intr_wqe_cmpl,                      // 中断请求（一般 = or-reduce & en）
    output wire                         o_intr_wqe_cmpl_sts_or_reduce,        // per-QP 中断状态的或归约
    output wire [               31 : 0] o_intr_wqe_cmpl_sts_wr_en,            // 写入 per-QP 中断状态位组
    output wire [               31 : 0] o_intr_wqe_cmpl_sts_wr_data,
    output wire [C_QP_INDX_WIDTH-1 : 0] o_intr_wqe_cmpl_sts_wr_addr,
    output wire [               31 : 0] o_intr_wqe_cmpl_sts_wr_en_pending,    // forward 支路
    output wire [C_QP_INDX_WIDTH-1 : 0] o_intr_wqe_cmpl_sts_wr_addr_pending,
    input  wire [                 31:0] i_clr_wqe_cmpl_sts_set_valid,         // 软件清除（按 QP bit）
    input  wire [C_QP_INDX_WIDTH-1 : 0] i_clr_wqe_cmpl_sts_set_id,

    //==========================================================================
    // RX 响应包处理器 → ACK/NACK 更新接口
    //==========================================================================
    input wire [C_QP_INDX_WIDTH -1 : 0] i_qp_index_acknack,  // 响应包所属 QP ID
    input wire                          i_acknack_valid,     // acked PSN/MSN 有效
    input wire [                  23:0] i_acknacked_psn,     // 被 ACK/NACK 的 PSN
    input wire [                  23:0] i_acknacked_msn,     // 被 ACK/NACK 的 MSN
    input wire [                   7:0] i_pkt_nacksyndrome,  // NACK syndrome（i_pkt_nack 时有效）

    //==========================================================================
    // QP 管理器重传接口（触发侧）
    //==========================================================================
    output wire [C_QP_INDX_WIDTH -1 : 0] o_retransmit_qpid,          // 需要重传的 QP
    output wire                          o_retransmit_reqd,          // 重传请求（保持到 i_retransmit_accepted）
    input  wire                          i_retransmit_accepted,      // QP 管理器已接收重传请求
    output reg  [                  55:0] o_retransmit_wqe_bram_din,  // {retry_start_psn[23:0]@[55:32], completed_dma_len[31:0]}
    output wire [                  23:0] o_psn_to_retry,             // 回退 PSN 目标
    output wire [                  23:0] o_ssn_to_retry,             // 回退 SSN 目标
    input  wire                          i_halt,                     // QP 管理器正在 halt 某 QP；该 QP 的 NACK set 事件在本模块内被屏蔽
    input  wire [C_QP_INDX_WIDTH -1 : 0] i_halted_qpid,

    //==========================================================================
    // QP 配置上下文读（从 QP 管理器 的 per-QP 寄存器块）
    //==========================================================================
    output wire [   C_QP_INDX_WIDTH -1:0] o_qp_conf_qp_idx,     // 读地址（本模块选定的 QP）
    output wire                           o_qp_cq_ba_req,       // CQ 基址读请求
    input  wire [C_M_AXI_ADDR_WIDTH -1:0] i_qp_cq_ba,
    input  wire                           i_qp_cq_ba_valid,
    output wire                           o_qp_cq_depth_req,    // CQ 深度读请求
    input  wire                           i_qp_cq_depth_valid,
    input  wire [                   15:0] i_qp_cq_depth,

    output wire        o_qp_conf_req,    // QP CONF 寄存器读（包含 pmtu / cqe_wr_en / intr_en 等）
    input  wire        i_qp_conf_valid,
    input  wire [31:0] i_qp_conf,

    // SQ PSN 响应接口；FSM 未直接使用该读返回
    input  wire                       i_qp_sq_psn_resp_valid,
    input  wire [               23:0] i_qp_sq_psn,
    output wire                       o_qp_sq_psn_resp_req,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_sq_psn_resp_idx,

    // CQ head pointer 读/写接口
    output wire                          o_qp_cq_hdptr_req,
    input  wire                          i_qp_cq_hdptr_valid,
    input  wire [                  15:0] i_qp_cq_hdptr,
    output wire [                  15:0] o_qp_cq_hdptr,        // 更新后写回
    output wire                          o_qp_cq_hdptr_wrn,
    output wire [C_QP_INDX_WIDTH -1 : 0] o_qp_cqe_proc_qpid,   // CQE 子 FSM 处理的 QP

    input wire [C_QP_INDX_WIDTH:0] i_num_elements_enabled,     // 使能的 QP 数（影响 timer 扫描范围）
    input wire [                4:0] i_timer_loadval_wqe,        // 首 WQE 发出后的 timeout 装载值
    input wire [                4:0] i_timer_loadval_acknack,    // 收到 ACK/NACK 后的 timeout 装载值
    input wire                       i_dma_in_idle,              // DMA 子系统空闲（fatal flush 前置条件）
    input wire                       i_en_osq_flush_on_qp_fatal, // 使能 fatal 时扫 OSQ 生成错误 CQE

    //==========================================================================
    // 向 QP 管理器 汇报 "该 QP 已 retried" 状态（置/清）
    //==========================================================================
    output wire                        o_qp_retried_set_valid,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_retried_set_id,
    output wire                        o_qp_retried_reset_valid,
    output wire [C_QP_INDX_WIDTH -1:0] o_qp_retried_reset_id,

    //==========================================================================
    // Retry 计数器接口（QP STAT 寄存器块）
    //==========================================================================
    input  wire [                2:0] i_qp_curr_retry_cnt,         // 剩余重试次数
    output wire                       o_qp_curr_retry_cnt_rd_req,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_curr_retry_cnt_qpid,
    input  wire                       i_qp_curr_retry_cnt_rd_rdy,

    output wire       o_qp_curr_retry_cnt_wr_req,  // 写回
    input  wire       i_qp_curr_retry_cnt_wr_rdy,
    output wire [2:0] o_qp_curr_retry_cnt,         // NACK:cfg-1 Timeout:curr-1 正常完成:cfg

    output wire       o_qp_cfg_retry_cnt_rd_req,  // 读 QP 配置上限
    input  wire       i_qp_cfg_retry_cnt_rd_rdy,
    input  wire [2:0] i_qp_cfg_retry_cnt,

    //==========================================================================
    // Retry 耗尽 → QP fatal 请求
    //==========================================================================
    output wire                       o_qp_retry_fatal_req,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_retry_fatal_qpid,
    input  wire                       i_qp_retry_fatal_rdy,

    //==========================================================================
    // 统计输出：NACK 计数（每收到一次 NACK 累加）
    //==========================================================================
    output wire [15:0] o_inc_nack_cnt,

    //==========================================================================
    // AXI Master 写通道（用于 CQE 写入 DDR）
    //  读通道请求常驱 0
    //==========================================================================
    // AW
    output wire [                   0:0] m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [                   7:0] m_axi_awlen,
    output wire [                   2:0] m_axi_awsize,
    output wire [                   1:0] m_axi_awburst,
    output wire [                   3:0] m_axi_awcache,
    output wire [                   2:0] m_axi_awprot,
    output wire                          m_axi_awvalid,
    input  wire                          m_axi_awready,
    // W
    output wire [                 511:0] m_axi_wdata,
    output wire [             512/8-1:0] m_axi_wstrb,
    output wire                          m_axi_wlast,
    output wire                          m_axi_wvalid,
    input  wire                          m_axi_wready,
    output wire                          m_axi_awlock,
    // B
    input  wire [                 0 : 0] m_axi_bid,
    input  wire [        RESP_WIDTH-1:0] m_axi_bresp,
    input  wire                          m_axi_bvalid,
    output wire                          m_axi_bready,
    // AR（读请求常驱 0）
    output wire [                   0:0] m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [                   7:0] m_axi_arlen,
    output wire [                   2:0] m_axi_arsize,
    output wire [                   1:0] m_axi_arburst,
    output wire [                   3:0] m_axi_arcache,
    output wire [                   2:0] m_axi_arprot,
    output wire                          m_axi_arvalid,
    input  wire                          m_axi_arready,
    // R（无读请求）
    input  wire [                   0:0] m_axi_rid,
    input  wire [                 511:0] m_axi_rdata,
    input  wire [        RESP_WIDTH-1:0] m_axi_rresp,
    input  wire                          m_axi_rlast,
    input  wire                          m_axi_rvalid,
    output wire                          m_axi_rready,
    output wire                          m_axi_arlock,

    //==========================================================================
    // OSQ pop 广播（WQE 完成/重传丢弃时拉 1 拍，供 lrdma_qp_retransmit_ctrl 同步）
    //==========================================================================
    output wire                         o_osq_pop,
    output wire [C_QP_INDX_WIDTH - 1:0] o_osq_pop_qpid,

    //==========================================================================
    // 超时事件 → 写入 timed_out sticky BRAM / pushback FIFO
    //==========================================================================
    output wire                         o_osq_timeout_set_valid,
    output wire [C_QP_INDX_WIDTH-1 : 0] o_osq_timeout_set_id,

    //==========================================================================
    // 复位初始化扫描：依次对所有 per-QP BRAM 清 0
    //==========================================================================
    input wire                         init_start,
    input wire [C_QP_INDX_WIDTH-1 : 0] init_count,

    //==========================================================================
    // QP under recovery / enable 状态外部写口
    //==========================================================================
    input wire                         i_qp_under_recovery_wr_en,    // QP 管理器 在 fatal clear 时写
    input wire [C_QP_INDX_WIDTH-1 : 0] i_qp_under_recovery_wr_idx,
    input wire                         i_qp_under_recovery_wr_data,
    input wire                         i_qp_enable_valid,            // QP enable 事件
    input wire [C_QP_INDX_WIDTH-1 : 0] i_qp_enable_id,
    input wire                         i_qp_enable_data
);
    `include "lrdma_defs.vh"

    //==========================================================================
    // 本地常量
    //==========================================================================
    localparam QP1 = 'd0;  // 内部 qpid0，对应软件 QP1/MAD 管理路径
    localparam FLOPS_TO_MEM = 1;  // OSQ/ACK BRAM 使用 BRAM 实现（非 FLOP）
    localparam IP2BUS_LEN_WIDTH = 7;  // AXI master 写长度位宽
    localparam CQE_DB_SIZE_IN_BYTES = 7'd4;  // 单个 CQE 写入大小（Byte）

    //-------------------
    // Outstanding buffer
    //---------------------------------------------------------
    // OSQ entry bit layout:
    //   { start_psn[C_OSQ_PSN_WIDTH-1:0],
    //     fatal_push, retried,
    //     msn[7:0],
    //     end_psn[C_OSQ_PSN_WIDTH-1:0],
    //     opcode[7:0],
    //     wrid[15:0] }
    // Total = 44 + 2 × C_OSQ_PSN_WIDTH bits；C_OSQ_PSN_WIDTH=16 时为 66 bits。
    localparam OSBUF_ENTRY_SIZE = 34 + C_OSQ_PSN_WIDTH + C_OSQ_PSN_WIDTH;  // bits

    // 同时保存 start_psn 和 end_psn：
    // end_psn 用于判断该 WQE 是否已经被 ACK/NAK 覆盖；start_psn 用于计算重传上下文，
    // 并在 retransmit 时作为内部 WQE 的 retransmit_wqe_start_psn 来源之一。
    //-------------------

    localparam OSBUF_MEM_SZ = C_OS_Q_DEPTH * OSBUF_ENTRY_SIZE * C_NUM_QP;  // memory size in bits

    localparam OS_Q_ADDR_WIDTH = (clog2(C_OS_Q_DEPTH * C_NUM_QP));  // OSQ 地址位宽

    localparam NAK_SEQ_ERR = 8'b01100000;
    localparam NO_ERROR = 8'b00000000;

    wire                           bus2ip_data_rdy;
    wire                           lrdma_axi_master_done;

    wire [ C_QP_INDX_WIDTH -1 : 0] retransmit_qpid;  // 重传路径选中的 QP id
    wire                           phase_to_update_post_retry;
    wire                           nak_seq_error;
    wire                           nack_for_psn_0;
    wire                           retransmit_reqd;
    wire                           wqe_push_for_non_nacked_qp;
    wire [                   31:0] ack_mgr_sts;
    wire                           osq_full_reset_valid;
    wire [  C_QP_INDX_WIDTH-1 : 0] osq_full_reset_id;
    wire                           osq_full_set_valid;
    wire [  C_QP_INDX_WIDTH-1 : 0] osq_full_set_id;
    wire                           intr_wqe_cmpl_sts;
    wire [                   15:0] osq_wrid;
    wire [                    7:0] osq_opcode;
    wire [    C_OSQ_PSN_WIDTH-1:0] osq_start_psn;
    wire [    C_OSQ_PSN_WIDTH-1:0] osq_end_psn;
    wire [                    7:0] osq_msn;
    wire [                   23:0] psn_to_retry;
    wire [                   23:0] msn_to_retry;
    wire [  OS_Q_ADDR_WIDTH - 1:0] osq_wr_addr_new;
    wire [  OS_Q_ADDR_WIDTH - 1:0] osq_rd_addr_new;

    // 状态观测信号
    reg  [  C_OS_Q_INDX_WIDTH : 0] selected_num_valid_osq_entries;  // Number of valid OSQ entries

    wire [                  511:0] ip2bus_data;
    wire [C_M_AXI_ADDR_WIDTH -1:0] ip2bus_wraddr;
    wire                           ip2bus_wen;
    wire                           arbitrate;
    reg                            arbitration_done;
    wire [   C_QP_INDX_WIDTH -1:0] osq_arbitrated_idx;
    wire [   C_QP_INDX_WIDTH -1:0] osq_sampled_arbitrated_idx;
    wire [   C_QP_INDX_WIDTH -1:0] arbitrated_idx_for_intr;
    wire                           pop_osq_from_fsm;
    wire [                   23:0] acknacked_psn_for_arbitrated_qp;
    wire [                   23:0] acknacked_msn_for_arbitrated_qp;
    wire [   C_QP_INDX_WIDTH -1:0] clr_timeout_n_nack_set_id;  // Clear for i_osq_timed_out
    wire                           clr_timeout_n_nack_set_valid;
    wire                           osq_nacked_cur_qp;
    wire                           osq_nacked_ff_comb;
    wire [  OS_Q_ADDR_WIDTH - 1:0] osq_base_addr_2d                                                                 [0:C_NUM_QP - 1];

    wire                           osq_wqe_retried;
    wire                           osq_wqe_fatal_push;
    wire [  IP2BUS_LEN_WIDTH -1:0] ip2bus_len;
    wire                           phase_bit_acknack_psn;
    wire [    C_OSQ_PSN_WIDTH-1:0] curr_acknacked_psn_for_curr_qp;
    reg                            qp_sq_psn_resp_valid_ff;
    reg  [                   23:0] qp_sq_psn_ff;

    reg                            nak_seq_error_ff;
    reg  [                   15:0] inc_nack_cnt;
    reg  [ C_QP_INDX_WIDTH -1 : 0] qp_index_acknack_ff;  // QP ID for ack_nack packet PSN
    reg                            acknack_valid_ff;

    reg                            dma_in_idle_ff;

    wire [   OSBUF_ENTRY_SIZE-1:0] osq_in;

    reg  [ OSBUF_ENTRY_SIZE - 1:0] osq_in_q;
    reg  [ OSBUF_ENTRY_SIZE - 1:0] osq_in_2q;

    wire                           osq_wr_addr_rd_valid;
    wire                           osq_rd_addr_rd_valid;
    wire                           rd_osq;
    wire                           acknacked_psn_rd_req_from_fsm;
    reg                            acknacked_psn_rd_req_done_to_fsm;
    reg                            osq_nacked_to_fsm;
    reg                            QP1_acknack_valid;
    wire                           osq_timed_out_set_valid;
    wire [  C_QP_INDX_WIDTH-1 : 0] osq_timed_out_set_id;
    wire [  C_QP_INDX_WIDTH-1 : 0] fatal_timeout_qpid;
    reg                            pushback_timeout_fatal_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] pushback_timeout_fatal_id;
    reg                            arbiter_valid_in;
    reg  [  C_QP_INDX_WIDTH-1 : 0] arbiter_Qpid_in;
    wire                           arbitration_done_int;
    wire                           no_OS_pending;
    reg                            no_OS_pending_r;
    reg                            arbitration_done_int_r;
    reg  [  C_QP_INDX_WIDTH-1 : 0] pushback_Qpid;
    reg                            pushback_valid;
    wire                           fifo_busy;
    wire                           fsm_pushback_valid;
    wire                           osq_empty_reset_valid;
    wire [  C_QP_INDX_WIDTH-1 : 0] osq_empty_reset_id;
    wire                           osq_empty_set_valid;
    wire [  C_QP_INDX_WIDTH-1 : 0] osq_empty_set_id;
    reg                            reload_retry_cnt_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id;
    reg                            reload_retry_cnt_data;
    reg                            reload_retry_cnt_valid_pending_1;
    reg  [  C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id_pending_1;
    reg                            reload_retry_cnt_data_pending_1;
    reg                            reload_retry_cnt_valid_pending_0;
    reg  [  C_QP_INDX_WIDTH-1 : 0] reload_retry_cnt_id_pending_0;
    reg  [                  1 : 0] condition_reload_retry;
    wire                           first_outgoing_pkt_ff_rd_data_int;
    wire                           first_outgoing_pkt_ff_rd_data;

    // QP 已发出的未完成请求被响应覆盖到 SQ PSN - 1
    reg                            all_outstanding_resp_received_set_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] all_outstanding_resp_received_set_id;
    reg                            all_outstanding_resp_received_reset_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] all_outstanding_resp_received_reset_id;
    wire                           all_outstanding_resp_received_rd_data;

    reg                            first_outstanding_pkt_ff_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_id;
    reg                            first_outstanding_pkt_ff_data;
    reg                            first_outstanding_pkt_ff_valid_aux;
    reg  [  C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_id_aux;
    reg                            first_outstanding_pkt_ff_data_aux;
    reg  [  C_QP_INDX_WIDTH-1 : 0] first_outstanding_pkt_ff_id_pending;
    reg                            first_outstanding_pkt_ff_valid_pending;
    wire                           first_outstanding_pkt_ff_rd_data_int;
    wire                           first_outstanding_pkt_ff_rd_data;

    reg  [                   31:0] wqe_cmpl_intr_sts_valid;
    reg  [                   31:0] wqe_cmpl_intr_sts_valid_int;
    reg  [                   31:0] wqe_cmpl_intr_sts_data;
    reg  [  C_QP_INDX_WIDTH-1 : 0] wqe_cmpl_intr_sts_id;
    reg  [                   31:0] wqe_cmpl_intr_sts_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] wqe_cmpl_intr_sts_id_pending;
    wire                           phase_bit_acknack_psn_ff_rd_data;
    wire [  C_QP_INDX_WIDTH-1 : 0] fsm_arbitrated_indx;
    wire [  C_QP_INDX_WIDTH-1 : 0] fsm_i_arbitrated_indx;
    wire                           timer_reload_for_wqe_ff_set;
    wire                           timer_reload_for_wqe_for_i_wqe_qpid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] i_wqe_qpid_r;
    reg                            acknack_psn_for_phase_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] acknack_psn_for_phase_id;
    reg                            acknack_psn_for_phase_data;
    reg                            acknack_psn_for_phase_valid_aux;
    reg  [  C_QP_INDX_WIDTH-1 : 0] acknack_psn_for_phase_id_aux;
    reg                            acknack_psn_for_phase_data_aux;
    reg                            acknack_psn_for_phase_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] acknack_psn_for_phase_id_pending;
    reg                            acknack_psn_for_phase_data_pending;
    wire                           acknack_psn_for_phase_rd_data_int;
    wire                           acknack_psn_for_phase_rd_data;
    wire                           i_osq_wqe_nack_resp_rd_data_0;
    wire                           i_osq_wqe_nack_resp_rd_data_1;
    wire                           osq_nacked_ff_rd_data_0;
    wire                           osq_nacked_ff_rd_data_1;
    wire                           osq_nacked_ff_rd_data_2;
    wire                           osq_nacked_ff_rd_data_3;
    wire                           osq_nacked_ff_rd_data_int_0;
    wire                           osq_nacked_ff_rd_data_int_1;
    wire                           osq_nacked_ff_rd_data_int_2;
    wire                           osq_nacked_ff_rd_data_int_3;
    reg                            qp_retried_ff_reset_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] qp_retried_ff_reset_id;
    reg                            qp_retried_ff_set_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] qp_retried_ff_set_id;
    reg                            first_outgoing_pkt_2ff_set_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] first_outgoing_pkt_2ff_set_id;
    wire                           qp_under_fatal_or_recovery_rd_data;
    reg                            i_qp_under_recovery_wr_en_r;
    wire [                   31:0] wqe_cmpl_intr_sts_rd_data;
    reg                            timer_disabled_per_qp_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] timer_disabled_per_qp_id;
    reg                            timer_disabled_per_qp_data;
    reg                            timer_disabled_per_qp_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] timer_disabled_per_qp_id_pending;
    reg                            timer_disabled_per_qp_data_pending;
    wire                           osq_empty_rd_data_0;
    wire                           osq_empty_rd_data_1;
    wire                           osq_empty_rd_data_2;
    wire                           osq_empty_rd_data_3;
    wire                           osq_empty_rd_data_int_0;
    wire                           osq_empty_rd_data_int_1;
    wire                           osq_empty_rd_data_int_2;
    wire                           osq_empty_rd_data_int_3;
    reg                            first_outgoing_pkt_ff_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] first_outgoing_pkt_ff_id;
    reg                            first_outgoing_pkt_ff_data;
    reg                            first_outgoing_pkt_ff_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] first_outgoing_pkt_ff_id_pending;
    reg                            phase_bit_acknack_psn_2ff_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_2ff_id;
    reg                            phase_bit_acknack_psn_2ff_data;
    reg                            phase_bit_acknack_psn_2ff_valid_pending_0;
    reg  [  C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_2ff_id_pending_0;
    reg                            phase_bit_acknack_psn_2ff_data_pending_0;
    reg                            phase_bit_acknack_psn_2ff_valid_pending_1;
    reg  [  C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_2ff_id_pending_1;
    reg                            phase_bit_acknack_psn_2ff_data_pending_1;
    reg                            phase_bit_acknack_psn_ff_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] phase_bit_acknack_psn_ff_id;
    reg                            phase_bit_acknack_psn_ff_data;
    reg                            wqe_cmpl_intr_sts_or_reduce;
    reg                            any_ones_in_cmpl_intr_lut;
    reg  [    C_QP_INDX_WIDTH : 0] osq_empty_zeros_count;
    wire                           osq_empty_ff_id_rd_data;
    wire                           ack_nack_to_process;
    reg                            i_qp_disable_pulse_set_valid_r;
    reg                            osq_nacked_ff_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id;
    reg                            osq_nacked_ff_data;
    reg                            osq_nacked_ff_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_nacked_ff_id_pending;
    reg                            osq_empty_ff_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_empty_ff_id;
    reg                            osq_empty_ff_data;
    reg                            osq_empty_ff_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_empty_ff_id_pending;
    wire                           i_qp_disable_pulse_set_valid_edge;  // qp_disable_pulse_set_valid 上升沿检测
    reg                            i_qp_disable_pulse_set_valid_edge_r;
    reg  [                    1:0] condition_phase_bit_acknack_psn_2ff;
    reg  [  C_QP_INDX_WIDTH-1 : 0] i_qp_disable_pulse_set_id_r;
    reg                            osq_timed_out_sticky_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_id;
    reg                            osq_timed_out_sticky_data;
    reg                            osq_timed_out_sticky_valid_aux;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_id_aux;
    reg                            osq_timed_out_sticky_data_aux;
    reg                            osq_timed_out_sticky_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_id_pending;
    reg                            osq_timed_out_sticky_valid_aux_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] osq_timed_out_sticky_id_aux_pending;
    wire                           osq_timed_out_sticky_rd_data_int_0;
    reg                            osq_timed_out_sticky_rd_data_0;
    reg                            osq_timed_out_sticky_rd_data_0_ff;
    wire                           osq_timed_out_sticky_fsm_arbitrated_indx;
    wire                           osq_timed_out_sticky_rd_data_int_1;
    reg                            osq_timed_out_sticky_rd_data_1;
    wire                           osq_nacked_timer_mem_rdaddr;
    reg                            osq_nacked_timer_mem_rdaddr_i;
    wire                           osq_nacked_ff_rd_data_comp_int;
    wire                           osq_nacked_ff_rd_data_comp;
    reg                            osq_nacked_ff_rd_data_comp_ff;
    wire [  C_QP_INDX_WIDTH-1 : 0] timer_mem_rdaddr;
    reg                            o_osq_nacked_set_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] o_osq_nacked_set_id;
    reg                            o_osq_nacked_reset_valid;
    reg  [  C_QP_INDX_WIDTH-1 : 0] o_osq_nacked_reset_id;
    reg                            o_osq_nacked_set_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] o_osq_nacked_set_id_pending;
    reg                            o_osq_nacked_reset_valid_pending;
    reg  [  C_QP_INDX_WIDTH-1 : 0] o_osq_nacked_reset_id_pending;
    reg                            i_osq_nacked_wr_pending;
    wire                           i_osq_wqe_nack_resp_rd_data_int_0;
    wire                           i_osq_wqe_nack_resp_rd_data_int_1;
    reg                            push_wr_en_A;
    reg  [  C_QP_INDX_WIDTH-1 : 0] push_wr_id_A;
    reg                            push_wr_en_B;
    reg  [  C_QP_INDX_WIDTH-1 : 0] push_wr_id_B;
    reg  [  C_QP_INDX_WIDTH-1 : 0] buffer_addr_B;
    reg  [    C_QP_INDX_WIDTH : 0] Qpid_buffer_wr_ptr;
    reg  [    C_QP_INDX_WIDTH : 0] Qpid_buffer_rd_ptr;
    reg                            pop_wr_en_B;
    wire                           fifo_empty;
    wire                           fifo_full;
    reg                            push_wr_en_A_r;
    reg                            push_wr_en_B_r;
    reg                            pop_wr_en_B_r;
    reg                            push_pending;
    reg  [    C_QP_INDX_WIDTH-1:0] push_id_pending;


    always_ff @(posedge core_clk) begin : i_osq_nackded
        if (!core_rstn) begin
            i_osq_nacked_wr_pending <= 1'b0;
        end else begin
            if (i_osq_wqe_nack_resp_valid_aux && i_osq_wqe_nack_resp_valid_pending && i_osq_wqe_nack_resp_id_aux != i_osq_wqe_nack_resp_id_pending) begin
                i_osq_nacked_wr_pending <= 1'b1;
            end else if (i_osq_nacked_wr_pending && (!i_osq_wqe_nack_resp_valid_aux || (i_osq_wqe_nack_resp_valid_aux && i_osq_wqe_nack_resp_id_aux == i_osq_wqe_nack_resp_id_pending) || (i_osq_wqe_nack_resp_valid && i_osq_wqe_nack_resp_id == i_osq_wqe_nack_resp_id_pending))) begin
                i_osq_nacked_wr_pending <= 1'b0;
            end
        end
    end

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_wqe_nack_resp_0 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || i_osq_wqe_nack_resp_valid),
        .wr_addr    (init_start ? init_count : i_osq_wqe_nack_resp_id),
        .wr_data    (init_start ? 1'b0 : i_osq_wqe_nack_resp_data),
        .wr_en_aux  (i_osq_wqe_nack_resp_valid_aux || i_osq_wqe_nack_resp_valid_pending || i_osq_nacked_wr_pending),
        .wr_addr_aux(i_osq_wqe_nack_resp_valid_aux ? i_osq_wqe_nack_resp_id_aux : i_osq_wqe_nack_resp_id_pending),
        .wr_data_aux(i_osq_wqe_nack_resp_valid_aux ? i_osq_wqe_nack_resp_data_aux : i_osq_wqe_nack_resp_data_pending),
        .rd_en      (1'b1),
        .rd_addr    (i_wqe_qpid),
        .rd_data    (i_osq_wqe_nack_resp_rd_data_int_0)
    );
    assign i_osq_wqe_nack_resp_rd_data_0 = ((i_osq_wqe_nack_resp_valid_pending || i_osq_nacked_wr_pending) && i_wqe_qpid == i_osq_wqe_nack_resp_id_pending) ? i_osq_wqe_nack_resp_data_pending : i_osq_wqe_nack_resp_rd_data_int_0;

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_wqe_nack_resp_1 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || i_osq_wqe_nack_resp_valid),
        .wr_addr    (init_start ? init_count : i_osq_wqe_nack_resp_id),
        .wr_data    (init_start ? 1'b0 : i_osq_wqe_nack_resp_data),
        .wr_en_aux  (i_osq_wqe_nack_resp_valid_aux || i_osq_wqe_nack_resp_valid_pending || i_osq_nacked_wr_pending),
        .wr_addr_aux(i_osq_wqe_nack_resp_valid_aux ? i_osq_wqe_nack_resp_id_aux : i_osq_wqe_nack_resp_id_pending),
        .wr_data_aux(i_osq_wqe_nack_resp_valid_aux ? i_osq_wqe_nack_resp_data_aux : i_osq_wqe_nack_resp_data_pending),
        .rd_en      (1'b1),
        .rd_addr    (osq_sampled_arbitrated_idx),
        .rd_data    (i_osq_wqe_nack_resp_rd_data_int_1)
    );
    assign i_osq_wqe_nack_resp_rd_data_1 = ((i_osq_wqe_nack_resp_valid_pending || i_osq_nacked_wr_pending) && osq_sampled_arbitrated_idx == i_osq_wqe_nack_resp_id_pending) ? i_osq_wqe_nack_resp_data_pending : i_osq_wqe_nack_resp_rd_data_int_1;



    `MSFF_R(dma_in_idle_ff, i_dma_in_idle, core_clk, ~core_rstn)


    // RX 响应包处理路径送入 ACK/NAK PSN 后，本模块按 QP 更新 acknacked_psn 表。
    // 重传被接受后，表中 PSN 回退到 psn_to_retry-1，也就是最后确认成功的 PSN，
    // 避免把 NACK 指向的包误认为已经完成。

    `MSFF_R(acknack_valid_ff, i_acknack_valid, core_clk, ~core_rstn)
    `MSFF_R(qp_index_acknack_ff, i_qp_index_acknack, core_clk, ~core_rstn)

    // RDMA WRITE 多包场景下，WRITE FIRST/MIDDLE 的 ACK 会推进 last_acknacked_psn；
    // 因此重传时需要把 acknacked_psn 回退到真正的重传起点前一拍。

    `MSFF_R(nak_seq_error_ff, nak_seq_error, core_clk, ~core_rstn)

    generate
        if (FLOPS_TO_MEM == 1) begin : ackanacked_psn_move_to_mem

            //==========================================================================
            // acknack_psn_ff
            //==========================================================================
            wire [                  2:0] acknacked_psn_wr_req;
            wire [C_QP_INDX_WIDTH - 1:0] acknacked_psn_wr_req_qpid               [0:2];
            wire [                 25:0] acknacked_psn_wr_req_data               [0:2];

            wire [                  1:0] acknacked_psn_rd_req;
            wire [C_QP_INDX_WIDTH - 1:0] acknacked_psn_rd_req_qpid               [0:1];
            wire [                 25:0] acknacked_psn_rd_req_data               [0:1];
            wire [                  1:0] acknacked_psn_rd_req_done;

            reg  [C_OSQ_PSN_WIDTH - 1:0] curr_acknacked_psn_for_curr_qp_q;
            reg  [                 23:0] acknacked_psn_for_arbitrated_qp_q;
            wire                         phase_bit_acknack_psn_2ff_rd_data_int_0;
            reg                          phase_bit_acknack_psn_2ff_rd_data_0;
            wire                         phase_bit_acknack_psn_2ff_rd_data_int_1;


            reg                          phase_bit_acknack_psn_2ff_rd_data_1;


            reg  [                 23:0] acknacked_psn_r1;
            reg  [                 23:0] acknacked_psn_r2;
            reg  [                 23:0] acknacked_psn_r3;
            reg                          qp_id_valid_acknack_ff;

            // 3 个写请求
            // WQE 首次进入 OSQ 时初始化最后已 ACK 的 PSN
            assign acknacked_psn_wr_req[0]      = i_wqe_push_req & first_outgoing_pkt_ff_rd_data;
            assign acknacked_psn_wr_req_qpid[0] = i_wqe_qpid;
            assign acknacked_psn_wr_req_data[0] = {2'b00, (i_wqe_send_start_psn - 1'b1)};

            // 收到 ACK/NAK 后更新 per-QP ACK/NAK PSN 状态
            assign acknacked_psn_wr_req[1]      = qp_id_valid_acknack_ff;
            assign acknacked_psn_wr_req_qpid[1] = qp_index_acknack_ff;
            assign acknacked_psn_wr_req_data[1] = {phase_bit_acknack_psn_2ff_rd_data_1, osq_nacked_ff_rd_data_0, acknacked_psn_r1[23:0]};

            // 重传处理后把最后已 ACK 的 PSN 回退到 psn_to_retry - 1
            assign acknacked_psn_wr_req[2]      = clr_timeout_n_nack_set_valid;
            assign acknacked_psn_wr_req_qpid[2] = o_qp_cqe_proc_qpid;
            assign acknacked_psn_wr_req_data[2] = {phase_to_update_post_retry, 1'b0, (psn_to_retry - 1'b1)};

            // 2 个读请求
            // 用于生成 acknacked_psn_for_arbitrated_qp
            // FSM 主读，用来得到仲裁 QP 的 ACK/NAK PSN 和 osq_nacked，并把 rd_done[0] 送给 FSM
            assign acknacked_psn_rd_req[0]      = acknacked_psn_rd_req_from_fsm;
            assign acknacked_psn_rd_req_qpid[0] = osq_sampled_arbitrated_idx;

            // i_acknack_valid 时读取该 QP 的旧 ACK/NAK PSN，并锁存到 curr_acknacked_psn_for_curr_qp_q
            assign acknacked_psn_rd_req[1]      = i_acknack_valid;
            assign acknacked_psn_rd_req_qpid[1] = i_qp_index_acknack;

            `MSFF_R(acknacked_psn_r1, i_acknacked_psn, core_clk, !core_rstn)
            `MSFF_R(acknacked_psn_r2, acknacked_psn_r1, core_clk, !core_rstn)
            `MSFF_R(acknacked_psn_r3, acknacked_psn_r2, core_clk, !core_rstn)
            `MSFF_R(qp_id_valid_acknack_ff, i_acknack_valid, core_clk, !core_rstn)

            always @(posedge core_clk) begin
                if (!core_rstn) begin
                    acknacked_psn_rd_req_done_to_fsm <= 1'b0;
                    osq_nacked_to_fsm                <= 1'b0;
                end else begin
                    if (acknacked_psn_rd_req_done[0]) begin
                        osq_nacked_to_fsm <= acknacked_psn_rd_req_data[0][24];
                    end
                    acknacked_psn_rd_req_done_to_fsm <= acknacked_psn_rd_req_done[0];
                end
            end

            lrdma_multiport_qp_state_table #(
                .DATA_WIDTH     (26),
                .NUM_WR_REQS    (3),
                .WR_FIFO_EN     (1),
                .NUM_RD_REQS    (2),
                .RD_FIFO_EN     (1),
                .C_NUM_QP       (C_NUM_QP),
                .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH)
            ) u_acknacked_psn_table (
                .core_clk   (core_clk),
                .core_rstn  (core_rstn),
                .wr_reqs    (acknacked_psn_wr_req),
                .wr_req_qpid(acknacked_psn_wr_req_qpid),
                .wr_req_data(acknacked_psn_wr_req_data),
                .wr_req_done(),                           // 写请求完成指示未接出
                .rd_reqs    (acknacked_psn_rd_req),
                .rd_req_qpid(acknacked_psn_rd_req_qpid),
                .rd_req_data(acknacked_psn_rd_req_data),
                .rd_req_done(acknacked_psn_rd_req_done)
            );

            // ACK/NAK 到达时读取并锁存该 QP 更新前的 ACK/NAK PSN。
            `MSFF_R(curr_acknacked_psn_for_curr_qp_q, (acknacked_psn_rd_req_done[1] ? acknacked_psn_rd_req_data[1][C_OSQ_PSN_WIDTH-1:0] : {C_OSQ_PSN_WIDTH{1'b0}}), core_clk, !core_rstn)
            assign curr_acknacked_psn_for_curr_qp = curr_acknacked_psn_for_curr_qp_q;

            assign nack_for_psn_0                 = i_acknack_valid & (i_acknacked_psn[C_OSQ_PSN_WIDTH-1:0] == 'h0) & osq_nacked_cur_qp;
            `MSFF_R(acknacked_psn_for_arbitrated_qp_q, (acknacked_psn_rd_req_done[0] ? acknacked_psn_rd_req_data[0][23:0] : acknacked_psn_for_arbitrated_qp_q), core_clk, !core_rstn)
            assign acknacked_psn_for_arbitrated_qp = acknacked_psn_for_arbitrated_qp_q;

            assign o_qp_sq_psn_resp_idx            = i_qp_index_acknack;
            assign o_qp_sq_psn_resp_req            = i_acknack_valid;

            reg qp_sq_psn_req_valid;
            `MSFF_R(qp_sq_psn_req_valid, acknack_valid_ff ? 1'b1 : (qp_sq_psn_resp_valid_ff ? 1'b0 : qp_sq_psn_req_valid), core_clk, ~core_rstn)

            always_ff @(posedge core_clk) begin : all_outstanding_resp_received_flop_to_mem
                if (~core_rstn) begin
                    all_outstanding_resp_received_reset_valid <= 1'b0;
                    all_outstanding_resp_received_reset_id    <= {C_QP_INDX_WIDTH{1'b0}};
                    all_outstanding_resp_received_set_valid   <= 1'b0;
                    all_outstanding_resp_received_set_id      <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    if (i_wqe_push_req) begin
                        all_outstanding_resp_received_reset_valid <= 1'b1;
                        all_outstanding_resp_received_reset_id    <= i_wqe_qpid;
                    end else begin
                        all_outstanding_resp_received_reset_valid <= 1'b0;
                    end

                    if (qp_sq_psn_resp_valid_ff && qp_sq_psn_req_valid && ((qp_sq_psn_ff - 1'b1) == acknacked_psn_r1) && ~(i_wqe_push_req && i_wqe_qpid == qp_index_acknack_ff)) begin
                        all_outstanding_resp_received_set_valid <= 1'b1;
                        all_outstanding_resp_received_set_id    <= qp_index_acknack_ff;
                    end else begin
                        all_outstanding_resp_received_set_valid <= 1'b0;
                    end
                end
            end

            lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_all_outstanding_resp_received (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || all_outstanding_resp_received_reset_valid),
                .wr_addr    (init_start ? init_count : all_outstanding_resp_received_reset_id),
                .wr_data    (1'b0),
                .wr_en_aux  (all_outstanding_resp_received_set_valid),
                .wr_addr_aux(all_outstanding_resp_received_set_id),
                .wr_data_aux(1'b1),
                .rd_en      (1'b1),
                .rd_addr    (i_wqe_qpid),
                .rd_data    (all_outstanding_resp_received_rd_data)
            );


            wire [23:0] wqe_send_start_psn_less_one;
            wire [23:0] psn_to_retry_less_one;
            assign wqe_send_start_psn_less_one = (i_wqe_send_start_psn - 1'b1);
            assign psn_to_retry_less_one       = (psn_to_retry - 1'b1);

            // NACK 指向低位 PSN 0 时不能直接翻转 phase bit。此时 ACK/NAK 表示的是
            // 回绕边界另一侧的 PSN；只有 NACK 指向非 0 低位 PSN 时，才说明后续 ACK/NAK
            // 已进入回绕后的新一侧。
            assign phase_bit_acknack_psn       = (((!i_acknacked_psn[C_OSQ_PSN_WIDTH-1] && acknack_psn_for_phase_rd_data) & ~nack_for_psn_0)) ? ~phase_bit_acknack_psn_2ff_rd_data_0 : phase_bit_acknack_psn_2ff_rd_data_0;

            // acknack_psn_for_phase 记录用于判断 ACK/NAK PSN 是否跨过低位 PSN 回绕点的历史最高位。
            // WQE 首包、ACK/NAK 到达、重传 clear 都可能更新该表。

            always_ff @(posedge core_clk) begin : acknack_psn_for_phase
                if (~core_rstn) begin
                    acknack_psn_for_phase_valid     <= 1'b0;
                    acknack_psn_for_phase_id        <= {C_QP_INDX_WIDTH{1'b0}};
                    acknack_psn_for_phase_data      <= 1'b0;
                    acknack_psn_for_phase_valid_aux <= 1'b0;
                    acknack_psn_for_phase_id_aux    <= {C_QP_INDX_WIDTH{1'b0}};
                    acknack_psn_for_phase_data_aux  <= 1'b0;
                end else begin
                    if (first_outgoing_pkt_ff_rd_data && i_wqe_push_req) begin
                        acknack_psn_for_phase_valid <= 1'b1;
                        acknack_psn_for_phase_id    <= i_wqe_qpid;
                        acknack_psn_for_phase_data  <= wqe_send_start_psn_less_one[C_OSQ_PSN_WIDTH-1];
                    end else if (i_acknack_valid) begin
                        acknack_psn_for_phase_valid <= 1'b1;
                        acknack_psn_for_phase_id    <= i_qp_index_acknack;
                        acknack_psn_for_phase_data  <= i_acknacked_psn[C_OSQ_PSN_WIDTH-1];
                    end else if (acknack_psn_for_phase_valid_pending && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == acknack_psn_for_phase_id_pending)) begin
                        acknack_psn_for_phase_valid <= 1'b1;
                        acknack_psn_for_phase_id    <= acknack_psn_for_phase_id_pending;
                        acknack_psn_for_phase_data  <= acknack_psn_for_phase_data_pending;
                    end else begin
                        acknack_psn_for_phase_valid <= 1'b0;
                    end

                    if (clr_timeout_n_nack_set_valid && !(i_acknack_valid && i_qp_index_acknack == clr_timeout_n_nack_set_id) && !(first_outgoing_pkt_ff_rd_data && i_wqe_push_req && i_wqe_qpid == clr_timeout_n_nack_set_id)) begin
                        acknack_psn_for_phase_valid_aux <= 1'b1;
                        acknack_psn_for_phase_id_aux    <= clr_timeout_n_nack_set_id;
                        acknack_psn_for_phase_data_aux  <= psn_to_retry_less_one[C_OSQ_PSN_WIDTH-1];
                    end else begin
                        acknack_psn_for_phase_valid_aux <= 1'b0;
                    end
                end
            end

            always_ff @(posedge core_clk) begin
                if (~core_rstn) begin
                    acknack_psn_for_phase_valid_pending <= 1'b0;
                    acknack_psn_for_phase_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                    acknack_psn_for_phase_data_pending  <= 1'b0;
                end else begin
                    if (first_outgoing_pkt_ff_rd_data && i_wqe_push_req && i_acknack_valid && i_qp_index_acknack != i_wqe_qpid) begin
                        acknack_psn_for_phase_valid_pending <= 1'b1;
                        acknack_psn_for_phase_id_pending    <= i_qp_index_acknack;
                        acknack_psn_for_phase_data_pending  <= i_acknacked_psn[C_OSQ_PSN_WIDTH-1];
                    end else begin
                        acknack_psn_for_phase_valid_pending <= 1'b0;
                    end
                end
            end

            lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_acknack_psn_for_phase (
                .core_clk   (core_clk),
                .core_rst   (~core_rstn),
                .wr_en      (init_start || acknack_psn_for_phase_valid),
                .wr_addr    (init_start ? init_count : acknack_psn_for_phase_id),
                .wr_data    (init_start ? 1'b0 : acknack_psn_for_phase_data),
                .wr_en_aux  (acknack_psn_for_phase_valid_aux),
                .wr_addr_aux(acknack_psn_for_phase_id_aux),
                .wr_data_aux(acknack_psn_for_phase_data_aux),
                .rd_en      (1'b1),
                .rd_addr    (i_qp_index_acknack),
                .rd_data    (acknack_psn_for_phase_rd_data_int)
            );

            assign acknack_psn_for_phase_rd_data       = (acknack_psn_for_phase_valid_pending && acknack_psn_for_phase_id_pending == i_qp_index_acknack) ? acknack_psn_for_phase_data_pending : acknack_psn_for_phase_rd_data_int;

            assign timer_reload_for_wqe_ff_set         = (first_outstanding_pkt_ff_rd_data || all_outstanding_resp_received_rd_data) && i_wqe_push_req;

            assign timer_reload_for_wqe_for_i_wqe_qpid = (first_outstanding_pkt_ff_rd_data || all_outstanding_resp_received_rd_data) && i_wqe_push_req;

            // phase_bit_acknack_psn_2ff 保存每个 QP 最近一次 ACK/NAK PSN 对应的 phase bit。
            // disable 清 0，ACK/NAK 写新 phase，重传接受后的 clear 使用 phase_to_update_post_retry。
            // 同拍不同 QP 冲突通过 pending_0/pending_1 串行补写。
            //`MSFF_R(phase_bit_acknack_psn_2ff[j], (i_qp_disable_pulse[j] ? 1'b0: (((j == i_qp_index_acknack) & i_acknack_valid) ? phase_bit_acknack_psn : 
            //(clr_timeout_n_nack[j] ? phase_to_update_post_retry : phase_bit_acknack_psn_2ff[j]))), core_clk, ~core_rstn)

            assign i_qp_disable_pulse_set_valid_edge   = i_qp_disable_pulse_set_valid && !i_qp_disable_pulse_set_valid_r;
            `MSFF_R(i_qp_disable_pulse_set_valid_edge_r, i_qp_disable_pulse_set_valid_edge, core_clk, !core_rstn)
            always_ff @(posedge core_clk) begin : phase_bit_acknack_psn_2ff
                if (~core_rstn) begin
                    phase_bit_acknack_psn_2ff_valid           <= 1'b0;
                    phase_bit_acknack_psn_2ff_id              <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_2ff_data            <= 1'b0;
                    phase_bit_acknack_psn_2ff_valid_pending_0 <= 1'b0;
                    phase_bit_acknack_psn_2ff_id_pending_0    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_2ff_data_pending_0  <= 1'b0;
                    phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b0;
                    phase_bit_acknack_psn_2ff_id_pending_1    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_2ff_data_pending_1  <= 1'b0;
                    condition_phase_bit_acknack_psn_2ff       <= 2'b00;
                end else begin
                    if (i_qp_disable_pulse_set_valid_edge) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= i_qp_disable_pulse_set_id;
                        phase_bit_acknack_psn_2ff_data  <= 1'b0;
                    end else if (i_acknack_valid) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= i_qp_index_acknack;
                        phase_bit_acknack_psn_2ff_data  <= phase_bit_acknack_psn;
                    end else if (clr_timeout_n_nack_set_valid) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= clr_timeout_n_nack_set_id;
                        phase_bit_acknack_psn_2ff_data  <= phase_to_update_post_retry;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_0) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= phase_bit_acknack_psn_2ff_id_pending_0;
                        phase_bit_acknack_psn_2ff_data  <= phase_bit_acknack_psn_2ff_data_pending_0;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_1) begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b1;
                        phase_bit_acknack_psn_2ff_id    <= phase_bit_acknack_psn_2ff_id_pending_1;
                        phase_bit_acknack_psn_2ff_data  <= phase_bit_acknack_psn_2ff_data_pending_1;
                    end else begin
                        phase_bit_acknack_psn_2ff_valid <= 1'b0;
                    end

                    if (i_qp_disable_pulse_set_valid_edge && i_acknack_valid && i_qp_index_acknack != i_qp_disable_pulse_set_id) begin
                        phase_bit_acknack_psn_2ff_valid_pending_0 <= 1'b1;
                        phase_bit_acknack_psn_2ff_id_pending_0    <= i_qp_index_acknack;
                        phase_bit_acknack_psn_2ff_data_pending_0  <= phase_bit_acknack_psn;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_0 && (!clr_timeout_n_nack_set_valid || (clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == phase_bit_acknack_psn_2ff_id_pending_0))) begin
                        phase_bit_acknack_psn_2ff_valid_pending_0 <= 1'b0;
                    end

                    if (i_qp_disable_pulse_set_valid_edge && i_acknack_valid && clr_timeout_n_nack_set_valid && !(clr_timeout_n_nack_set_id == i_qp_disable_pulse_set_id) && !(clr_timeout_n_nack_set_id == i_qp_index_acknack)) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b1;
                        phase_bit_acknack_psn_2ff_id_pending_1    <= clr_timeout_n_nack_set_id;
                        phase_bit_acknack_psn_2ff_data_pending_1  <= phase_to_update_post_retry;
                        condition_phase_bit_acknack_psn_2ff       <= 2'b00;  // 三路更新冲突时保持本拍不直接写，交给 pending 路径处理
                    end else if (!i_acknack_valid && i_qp_disable_pulse_set_valid_edge && clr_timeout_n_nack_set_valid && !(clr_timeout_n_nack_set_id == i_qp_disable_pulse_set_id)) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b1;
                        phase_bit_acknack_psn_2ff_id_pending_1    <= clr_timeout_n_nack_set_id;
                        phase_bit_acknack_psn_2ff_data_pending_1  <= phase_to_update_post_retry;
                        condition_phase_bit_acknack_psn_2ff       <= 2'b01;
                    end else if (!i_qp_disable_pulse_set_valid_edge && i_acknack_valid && clr_timeout_n_nack_set_valid && !(clr_timeout_n_nack_set_id == i_qp_index_acknack)) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b1;
                        phase_bit_acknack_psn_2ff_id_pending_1    <= clr_timeout_n_nack_set_id;
                        phase_bit_acknack_psn_2ff_data_pending_1  <= phase_to_update_post_retry;
                        condition_phase_bit_acknack_psn_2ff       <= 2'b10;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && condition_phase_bit_acknack_psn_2ff == 2'b00 && (!phase_bit_acknack_psn_2ff_valid_pending_0)) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b0;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && condition_phase_bit_acknack_psn_2ff == 2'b01 && (!i_acknack_valid || (i_acknack_valid && i_qp_index_acknack == phase_bit_acknack_psn_2ff_id_pending_1))) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b0;
                    end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && condition_phase_bit_acknack_psn_2ff == 2'b10 && (!i_qp_disable_pulse_set_valid_edge || (i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == phase_bit_acknack_psn_2ff_id_pending_1))) begin
                        phase_bit_acknack_psn_2ff_valid_pending_1 <= 1'b0;
                    end
                end
            end

            always_ff @(posedge core_clk) begin
                i_qp_disable_pulse_set_valid_r <= i_qp_disable_pulse_set_valid;
                i_qp_disable_pulse_set_id_r    <= i_qp_disable_pulse_set_id;
            end

            lrdma_common_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_2ff_0 (
                .core_clk(core_clk),
                .core_rst(~core_rstn),
                .wr_en   (init_start || phase_bit_acknack_psn_2ff_valid),
                .wr_addr (init_start ? init_count : phase_bit_acknack_psn_2ff_id),
                .wr_data (init_start ? 1'b0 : phase_bit_acknack_psn_2ff_data),
                .rd_en   (1'b1),
                .rd_addr (i_qp_index_acknack),
                .rd_data (phase_bit_acknack_psn_2ff_rd_data_int_0)
            );

            always_comb begin
                if (phase_bit_acknack_psn_2ff_valid_pending_0 && i_qp_index_acknack == phase_bit_acknack_psn_2ff_id_pending_0) begin
                    phase_bit_acknack_psn_2ff_rd_data_0 = phase_bit_acknack_psn_2ff_data_pending_0;
                end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && i_qp_index_acknack == phase_bit_acknack_psn_2ff_id_pending_1) begin
                    phase_bit_acknack_psn_2ff_rd_data_0 = phase_bit_acknack_psn_2ff_data_pending_1;
                end else begin
                    phase_bit_acknack_psn_2ff_rd_data_0 = phase_bit_acknack_psn_2ff_rd_data_int_0;
                end
            end

            lrdma_common_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_2ff_1 (
                .core_clk(core_clk),
                .core_rst(~core_rstn),
                .wr_en   (init_start || phase_bit_acknack_psn_2ff_valid),
                .wr_addr (init_start ? init_count : phase_bit_acknack_psn_2ff_id),
                .wr_data (init_start ? 1'b0 : phase_bit_acknack_psn_2ff_data),
                .rd_en   (1'b1),
                .rd_addr (qp_index_acknack_ff),
                .rd_data (phase_bit_acknack_psn_2ff_rd_data_int_1)
            );

            always_comb begin
                if (phase_bit_acknack_psn_2ff_valid_pending_0 && qp_index_acknack_ff == phase_bit_acknack_psn_2ff_id_pending_0) begin
                    phase_bit_acknack_psn_2ff_rd_data_1 = phase_bit_acknack_psn_2ff_data_pending_0;
                end else if (phase_bit_acknack_psn_2ff_valid_pending_1 && qp_index_acknack_ff == phase_bit_acknack_psn_2ff_id_pending_1) begin
                    phase_bit_acknack_psn_2ff_rd_data_1 = phase_bit_acknack_psn_2ff_data_pending_1;
                end else begin
                    phase_bit_acknack_psn_2ff_rd_data_1 = phase_bit_acknack_psn_2ff_rd_data_int_1;
                end
            end

            always_ff @(posedge core_clk) begin : phase_bit_acknack_psn_ff
                if (~core_rstn) begin
                    phase_bit_acknack_psn_ff_valid <= 1'b0;
                    phase_bit_acknack_psn_ff_id    <= {C_QP_INDX_WIDTH{1'b0}};
                    phase_bit_acknack_psn_ff_data  <= 1'b0;
                end else begin
                    phase_bit_acknack_psn_ff_valid <= acknacked_psn_rd_req_done[0];
                    phase_bit_acknack_psn_ff_id    <= osq_sampled_arbitrated_idx;
                    phase_bit_acknack_psn_ff_data  <= acknacked_psn_rd_req_data[0][25];
                end
            end

            lrdma_common_sdp_lut_ram_wrapper #(
                .C_RAM_WORD_WIDTH(1),
                .C_RAM_DEPTH(C_NUM_QP)
            ) inst_phase_bit_acknack_psn_ff (
                .core_clk(core_clk),
                .core_rst(~core_rstn),
                .wr_en   (init_start || phase_bit_acknack_psn_ff_valid),
                .wr_addr (init_start ? init_count : phase_bit_acknack_psn_ff_id),
                .wr_data (init_start ? 1'b0 : phase_bit_acknack_psn_ff_data),
                .rd_en   (1'b1),
                .rd_addr (osq_sampled_arbitrated_idx),
                .rd_data (phase_bit_acknack_psn_ff_rd_data)
            );


            //==========================================================================
            // acknack_msn
            //==========================================================================
            wire [C_QP_INDX_WIDTH - 1:0] acknacked_msn_wr_req_qpid[0:1];
            wire [                  1:0] acknacked_msn_wr_req;
            wire [                 23:0] acknacked_msn_wr_req_data[0:1];

            assign acknacked_msn_wr_req[0]      = i_acknack_valid;
            assign acknacked_msn_wr_req_qpid[0] = i_qp_index_acknack;
            assign acknacked_msn_wr_req_data[0] = i_acknacked_msn[23:0];

            assign acknacked_msn_wr_req[1]      = clr_timeout_n_nack_set_valid;
            assign acknacked_msn_wr_req_qpid[1] = o_qp_cqe_proc_qpid;
            assign acknacked_msn_wr_req_data[1] = (msn_to_retry - 1'b1);

            wire                         acknacked_msn_rd_req = acknacked_psn_rd_req_from_fsm;
            wire [C_QP_INDX_WIDTH - 1:0] acknacked_msn_rd_req_qpid                            [0:0];
            assign acknacked_msn_rd_req_qpid[0] = osq_sampled_arbitrated_idx;
            wire        acknacked_msn_rd_req_done;
            wire [23:0] acknacked_msn_rd_req_data   [0:0];
            reg  [23:0] acknacked_msn_rd_req_data_q;


            lrdma_multiport_qp_state_table #(
                .DATA_WIDTH     (24),
                .NUM_WR_REQS    (2),
                .WR_FIFO_EN     (1),
                .NUM_RD_REQS    (1),
                .RD_FIFO_EN     (0),
                .C_NUM_QP       (C_NUM_QP),
                .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH)
            ) u_acknacked_msn_table (
                .core_clk   (core_clk),
                .core_rstn  (core_rstn),
                .wr_reqs    (acknacked_msn_wr_req),
                .wr_req_qpid(acknacked_msn_wr_req_qpid),
                .wr_req_data(acknacked_msn_wr_req_data),
                .wr_req_done(),                           // 写请求完成指示未接出
                .rd_reqs    (acknacked_msn_rd_req),
                .rd_req_qpid(acknacked_msn_rd_req_qpid),
                .rd_req_data(acknacked_msn_rd_req_data),
                .rd_req_done(acknacked_msn_rd_req_done)
            );

            `MSFF_R(acknacked_msn_rd_req_data_q, (acknacked_msn_rd_req_done ? acknacked_msn_rd_req_data[0] : acknacked_msn_rd_req_data_q), core_clk, !core_rstn)
            assign acknacked_msn_for_arbitrated_qp = acknacked_msn_rd_req_data_q;

        end  // ackanacked_psn_move_to_mem 

    endgenerate  // end of FLOPS_TO_MEM == 1

    always_ff @(posedge core_clk) begin : qp_retried_ff
        if (~core_rstn) begin
            qp_retried_ff_reset_valid <= 1'b0;
            qp_retried_ff_reset_id    <= {C_QP_INDX_WIDTH{1'b0}};
            qp_retried_ff_set_valid   <= 1'b0;
            qp_retried_ff_set_id      <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (retransmit_reqd & i_retransmit_accepted) begin
                qp_retried_ff_set_valid <= 1'b1;
                qp_retried_ff_set_id    <= retransmit_qpid;
            end else begin
                qp_retried_ff_set_valid <= 1'b0;
            end

            if (i_acknack_valid && ~((retransmit_reqd & i_retransmit_accepted) && i_qp_index_acknack == retransmit_qpid)) begin
                qp_retried_ff_reset_valid <= 1'b1;
                qp_retried_ff_reset_id    <= i_qp_index_acknack;
            end else begin
                qp_retried_ff_reset_valid <= 1'b0;
            end
        end
    end

    always_ff @(posedge core_clk) begin : first_outgoing_pkt_2ff
        if (~core_rstn) begin
            first_outgoing_pkt_2ff_set_valid <= 1'b0;
            first_outgoing_pkt_2ff_set_id    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            first_outgoing_pkt_2ff_set_valid <= first_outgoing_pkt_ff_valid && first_outgoing_pkt_ff_data;
            first_outgoing_pkt_2ff_set_id    <= first_outgoing_pkt_ff_id;
        end
    end

    //###############################################################################################################################################
    // first_outgoing_pkt_ff 标记某 QP 的下一次 WQE push 是否是该 QP 重新启用/清空后的首个 outgoing packet。
    //###############################################################################################################################################
    always_ff @(posedge core_clk) begin : first_outgoing_pkt_ff
        if (~core_rstn) begin
            first_outgoing_pkt_ff_valid         <= 1'b0;
            first_outgoing_pkt_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            first_outgoing_pkt_ff_data          <= 1'b0;
            first_outgoing_pkt_ff_valid_pending <= 1'b0;
            first_outgoing_pkt_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_qp_disable_pulse_set_valid_edge) begin
                first_outgoing_pkt_ff_valid <= 1'b1;
                first_outgoing_pkt_ff_id    <= i_qp_disable_pulse_set_id;
                first_outgoing_pkt_ff_data  <= 1'b1;
            end else if (i_wqe_push_req) begin
                first_outgoing_pkt_ff_valid <= 1'b1;
                first_outgoing_pkt_ff_id    <= i_wqe_qpid;
                first_outgoing_pkt_ff_data  <= 1'b0;
            end else if (first_outgoing_pkt_ff_valid_pending) begin
                first_outgoing_pkt_ff_valid <= 1'b1;
                first_outgoing_pkt_ff_id    <= first_outgoing_pkt_ff_id_pending;
                first_outgoing_pkt_ff_data  <= 1'b0;
            end else begin
                first_outgoing_pkt_ff_valid <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge && i_wqe_push_req && i_qp_disable_pulse_set_id != i_wqe_qpid) begin
                first_outgoing_pkt_ff_valid_pending <= 1'b1;
                first_outgoing_pkt_ff_id_pending    <= i_wqe_qpid;
            end else begin
                first_outgoing_pkt_ff_valid_pending <= 1'b0;
            end
        end
    end



    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_first_outgoing_pkt_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || first_outgoing_pkt_ff_valid),
        .wr_addr (init_start ? init_count : first_outgoing_pkt_ff_id),
        .wr_data (init_start ? 1'b1 : first_outgoing_pkt_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_wqe_qpid),
        .rd_data (first_outgoing_pkt_ff_rd_data_int)
    );

    assign first_outgoing_pkt_ff_rd_data = (first_outgoing_pkt_ff_valid_pending && first_outgoing_pkt_ff_id_pending == i_wqe_qpid) ? 1'b0 : first_outgoing_pkt_ff_rd_data_int;

    //###############################################################################################################################################
    // first_outstanding_pkt_ff 标记 QP 是否还没有未完成响应；disable 和 clear 都会把它置回 1，WQE push 会清 0。
    //###############################################################################################################################################
    //`MSFF_RL(first_outstanding_pkt_ff[j], ((clr_timeout_n_nack[j] | i_qp_disable_pulse[j]) ? 1'b1: ((i_wqe_push_req && (i_wqe_qpid == j)) ? 1'b0 : 
    // first_outstanding_pkt_ff[j])), core_clk, ~core_rstn, 1'b1)

    always_ff @(posedge core_clk) begin : first_outstanding_pkt_ff
        if (~core_rstn) begin
            first_outstanding_pkt_ff_valid         <= 1'b0;
            first_outstanding_pkt_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            first_outstanding_pkt_ff_data          <= 1'b0;
            first_outstanding_pkt_ff_valid_aux     <= 1'b0;
            first_outstanding_pkt_ff_id_aux        <= {C_QP_INDX_WIDTH{1'b0}};
            first_outstanding_pkt_ff_data_aux      <= 1'b0;
            first_outstanding_pkt_ff_valid_pending <= 1'b0;
            first_outstanding_pkt_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_wqe_push_req && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == i_wqe_qpid) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_wqe_qpid)) begin
                first_outstanding_pkt_ff_valid <= 1'b1;
                first_outstanding_pkt_ff_id    <= i_wqe_qpid;
                first_outstanding_pkt_ff_data  <= 1'b0;
            end else begin
                first_outstanding_pkt_ff_valid <= 1'b0;
            end

            if (clr_timeout_n_nack_set_valid) begin
                first_outstanding_pkt_ff_valid_aux <= 1'b1;
                first_outstanding_pkt_ff_id_aux    <= clr_timeout_n_nack_set_id;
                first_outstanding_pkt_ff_data_aux  <= 1'b1;
            end else if (i_qp_disable_pulse_set_valid_edge) begin
                first_outstanding_pkt_ff_valid_aux <= 1'b1;
                first_outstanding_pkt_ff_id_aux    <= i_qp_disable_pulse_set_id;
                first_outstanding_pkt_ff_data_aux  <= 1'b1;
            end else if (first_outstanding_pkt_ff_valid_pending && !(i_wqe_push_req && i_wqe_qpid == first_outstanding_pkt_ff_id_pending)) begin
                first_outstanding_pkt_ff_valid_aux <= 1'b1;
                first_outstanding_pkt_ff_id_aux    <= first_outstanding_pkt_ff_id_pending;
                first_outstanding_pkt_ff_data_aux  <= 1'b1;
            end else begin
                first_outstanding_pkt_ff_valid_aux <= 1'b0;
            end

            if (clr_timeout_n_nack_set_valid && i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id != clr_timeout_n_nack_set_id) begin
                first_outstanding_pkt_ff_valid_pending <= 1'b1;
                first_outstanding_pkt_ff_id_pending    <= i_qp_disable_pulse_set_id;
            end else begin
                first_outstanding_pkt_ff_valid_pending <= 1'b0;
            end
        end
    end

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_first_outstanding_pkt_ff (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || first_outstanding_pkt_ff_valid_aux),
        .wr_addr    (init_start ? init_count : first_outstanding_pkt_ff_id_aux),
        .wr_data    (1'b1),
        .wr_en_aux  (first_outstanding_pkt_ff_valid),
        .wr_addr_aux(first_outstanding_pkt_ff_id),
        .wr_data_aux(first_outstanding_pkt_ff_data),
        .rd_en      (1'b1),
        .rd_addr    (i_wqe_qpid),
        .rd_data    (first_outstanding_pkt_ff_rd_data_int)
    );

    assign first_outstanding_pkt_ff_rd_data = (first_outstanding_pkt_ff_valid_pending && first_outstanding_pkt_ff_id_pending == i_wqe_qpid) ? 1'b1 : first_outstanding_pkt_ff_rd_data_int;

    always_ff @(posedge core_clk) begin
        i_wqe_qpid_r <= i_wqe_qpid;
    end

    assign osq_nacked_cur_qp  = (i_acknack_valid && nak_seq_error && ~(i_halt && i_halted_qpid == i_qp_index_acknack)) ? 1'b1 : osq_nacked_ff_rd_data_3;

    assign osq_nacked_ff_comb = (clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_qp_index_acknack) ? 1'b0 : ((i_acknack_valid && nak_seq_error && ~(i_halt && i_halted_qpid == i_qp_index_acknack)) | osq_nacked_ff_rd_data_3);

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (qp_index_acknack_ff),
        .rd_data (osq_nacked_ff_rd_data_int_0)
    );
    // 读口 0：ACK/NAK 更新流水线使用，读 qp_index_acknack_ff 对应的 NACK sticky。
    assign osq_nacked_ff_rd_data_0 = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == qp_index_acknack_ff) ? 1'b1 : osq_nacked_ff_rd_data_int_0;

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_1 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_wqe_qpid),
        .rd_data (osq_nacked_ff_rd_data_int_1)
    );
    // 读口 1：TX WQE push 路径使用，用于阻止仍在 NACK 处理中的 QP 继续 push 新 OSQ。
    assign osq_nacked_ff_rd_data_1 = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == i_wqe_qpid) ? 1'b1 : osq_nacked_ff_rd_data_int_1;

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_2 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (osq_sampled_arbitrated_idx),
        .rd_data (osq_nacked_ff_rd_data_int_2)
    );
    // 读口 2：FSM 仲裁 QP 使用，作为 CHECK_ERROR/CHECK_OSQ 的 NACK 状态输入。
    assign osq_nacked_ff_rd_data_2 = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == osq_sampled_arbitrated_idx) ? 1'b1 : osq_nacked_ff_rd_data_int_2;

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_3 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_index_acknack),
        .rd_data (osq_nacked_ff_rd_data_int_3)
    );
    // 读口 3：ACK/NAK 入口使用，用于和本拍 NAK 合成 osq_nacked_cur_qp/osq_nacked_ff_comb。
    assign osq_nacked_ff_rd_data_3 = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == i_qp_index_acknack) ? 1'b1 : osq_nacked_ff_rd_data_int_3;

    //###############################################################################################################################################
    // clr_timeout_n_nack_set_valid 与 i_acknack_valid 最好错开至少 1 拍，避免同一 QP 的 set/clear 关系难以判断。
    //###############################################################################################################################################
    always_ff @(posedge core_clk) begin : osq_nacked_ff
        if (~core_rstn) begin
            osq_nacked_ff_valid         <= 1'b0;
            osq_nacked_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_nacked_ff_data          <= 1'b0;
            osq_nacked_ff_valid_pending <= 1'b0;
            osq_nacked_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (clr_timeout_n_nack_set_valid) begin
                osq_nacked_ff_valid <= 1'b1;
                osq_nacked_ff_id    <= clr_timeout_n_nack_set_id;
                osq_nacked_ff_data  <= 1'b0;
            end else if (i_acknack_valid && nak_seq_error && ~(i_halt && i_halted_qpid == i_qp_index_acknack)) begin
                osq_nacked_ff_valid <= 1'b1;
                osq_nacked_ff_id    <= i_qp_index_acknack;
                osq_nacked_ff_data  <= 1'b1;
            end else if (osq_nacked_ff_valid_pending) begin
                osq_nacked_ff_valid <= 1'b1;
                osq_nacked_ff_id    <= osq_nacked_ff_id_pending;
                osq_nacked_ff_data  <= 1'b1;
            end else begin
                osq_nacked_ff_valid <= 1'b0;
            end

            if (clr_timeout_n_nack_set_valid && i_acknack_valid && nak_seq_error && !(i_halt && i_halted_qpid == i_qp_index_acknack) && clr_timeout_n_nack_set_id != i_qp_index_acknack) begin
                osq_nacked_ff_valid_pending <= 1'b1;
                osq_nacked_ff_id_pending    <= i_qp_index_acknack;
            end else begin
                osq_nacked_ff_valid_pending <= 1'b0;
            end
        end
    end

    //###############################################################################################################################################
    // reload_retry_cnt_ff 记录“下一次 FSM 读取该 QP 时是否需要把 retry 计数恢复到配置值”。
    // disable 置 1，timeout 置 0，普通 ACK 置 1，NACK 置 0；同拍不同 QP 冲突通过 pending 补写。
    //###############################################################################################################################################
    always_ff @(posedge core_clk) begin : reload_retry_cnt_ff
        if (!core_rstn) begin
            reload_retry_cnt_valid           <= 1'b0;
            reload_retry_cnt_id              <= {C_QP_INDX_WIDTH{1'b0}};
            reload_retry_cnt_data            <= 1'b0;
            reload_retry_cnt_valid_pending_1 <= 1'b0;
            reload_retry_cnt_id_pending_1    <= {C_QP_INDX_WIDTH{1'b0}};
            reload_retry_cnt_data_pending_1  <= 1'b0;
            reload_retry_cnt_valid_pending_0 <= 1'b0;
            reload_retry_cnt_id_pending_0    <= {C_QP_INDX_WIDTH{1'b0}};
            condition_reload_retry           <= 2'b00;
        end else begin
            if (i_qp_disable_pulse_set_valid_edge) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= i_qp_disable_pulse_set_id;
                reload_retry_cnt_data  <= 1'b1;
            end else if (osq_timed_out_set_valid) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= osq_timed_out_set_id;
                reload_retry_cnt_data  <= 1'b0;
            end else if (i_acknack_valid) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= i_qp_index_acknack;
                reload_retry_cnt_data  <= !nak_seq_error;
            end else if (reload_retry_cnt_valid_pending_0) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= reload_retry_cnt_id_pending_0;
                reload_retry_cnt_data  <= 1'b0;
            end else if (reload_retry_cnt_valid_pending_1) begin
                reload_retry_cnt_valid <= 1'b1;
                reload_retry_cnt_id    <= reload_retry_cnt_id_pending_1;
                reload_retry_cnt_data  <= reload_retry_cnt_data_pending_1;
            end else begin
                reload_retry_cnt_valid <= 1'b0;
            end


            if (i_qp_disable_pulse_set_valid_edge && osq_timed_out_set_valid && i_qp_disable_pulse_set_id != osq_timed_out_set_id) begin
                reload_retry_cnt_valid_pending_0 <= 1'b1;
                reload_retry_cnt_id_pending_0    <= osq_timed_out_set_id;
            end else begin
                reload_retry_cnt_valid_pending_0 <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge && osq_timed_out_set_valid && i_acknack_valid && i_qp_disable_pulse_set_id != i_qp_index_acknack && osq_timed_out_set_id != i_qp_index_acknack) begin
                reload_retry_cnt_valid_pending_1 <= 1'b1;
                reload_retry_cnt_id_pending_1    <= i_qp_index_acknack;
                reload_retry_cnt_data_pending_1  <= !nak_seq_error;
                condition_reload_retry           <= 2'b00;
            end else if (!i_qp_disable_pulse_set_valid_edge && osq_timed_out_set_valid && i_acknack_valid && osq_timed_out_set_id != i_qp_index_acknack) begin
                reload_retry_cnt_valid_pending_1 <= 1'b1;
                reload_retry_cnt_id_pending_1    <= i_qp_index_acknack;
                reload_retry_cnt_data_pending_1  <= !nak_seq_error;
                condition_reload_retry           <= 2'b01;
            end else if (!osq_timed_out_set_valid && i_qp_disable_pulse_set_valid_edge && i_acknack_valid && i_qp_disable_pulse_set_id != i_qp_index_acknack) begin
                reload_retry_cnt_valid_pending_1 <= 1'b1;
                reload_retry_cnt_id_pending_1    <= i_qp_index_acknack;
                reload_retry_cnt_data_pending_1  <= !nak_seq_error;
                condition_reload_retry           <= 2'b10;
            end else if (reload_retry_cnt_valid_pending_1 && condition_reload_retry == 2'b00 && !reload_retry_cnt_valid_pending_0) begin
                reload_retry_cnt_valid_pending_1 <= 1'b0;
            end else if (reload_retry_cnt_valid_pending_1 && condition_reload_retry == 2'b01 && (!i_qp_disable_pulse_set_valid_edge || (i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == reload_retry_cnt_id_pending_1))) begin
                reload_retry_cnt_valid_pending_1 <= 1'b0;
            end else if (reload_retry_cnt_valid_pending_1 && condition_reload_retry == 2'b10 && (!osq_timed_out_set_valid || (osq_timed_out_set_valid && osq_timed_out_set_id == reload_retry_cnt_id_pending_1))) begin
                reload_retry_cnt_valid_pending_1 <= 1'b0;
            end
        end
    end


    assign o_qp_retried_set_valid   = qp_retried_ff_set_valid;
    assign o_qp_retried_set_id      = qp_retried_ff_set_id;
    assign o_qp_retried_reset_valid = qp_retried_ff_reset_valid;
    assign o_qp_retried_reset_id    = qp_retried_ff_reset_id;

    assign nak_seq_error            = i_pkt_nacksyndrome == NAK_SEQ_ERR ? 1'b1 : 1'b0;
    `MSFF_R(inc_nack_cnt, (i_acknack_valid & nak_seq_error ? (inc_nack_cnt + 1'b1) : inc_nack_cnt), core_clk, ~core_rstn)
    `MSFF_R(qp_sq_psn_resp_valid_ff, i_qp_sq_psn_resp_valid, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_psn_ff, i_qp_sq_psn, core_clk, ~core_rstn)

    assign o_inc_nack_cnt = inc_nack_cnt;


    // ACK 管理器使用 OSQ 记录已发出但尚未完成的 WQE。
    // 每条 OSQ entry 保存 start/end PSN、MSN、opcode 和 WRID 等完成/重传判断需要的信息。
    // 完成通知通过 CQE 写入 CQ，不向 SQ 完成 Doorbell 写累计完成计数。

    assign osq_in         = {i_wqe_send_start_psn[C_OSQ_PSN_WIDTH-1:0], i_wqe_fatal_push, i_wqe_retried, i_wqe_send_msn[7:0], i_wqe_send_end_psn[C_OSQ_PSN_WIDTH-1:0], i_wqe_opcode, i_wqe_wrid};
    `MSFF_R(osq_in_q, osq_in, core_clk, !core_rstn)
    `MSFF_R(osq_in_2q, osq_in_q, core_clk, !core_rstn)


    //--------------------------------------------------------------------------
    // Outstanding SQ Queue 存储
    //  布局 per entry:
    //    { start_psn, fatal_push, retried,
    //      msn, end_psn, opcode, wrid }
    //  读写由 lrdma_queue_mgr 维护 per-QP 读写指针 / 水位 / full / empty
    //--------------------------------------------------------------------------
    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (OSBUF_MEM_SZ),     //positive integer
        .MEMORY_PRIMITIVE ("ultra"),          //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(OSBUF_ENTRY_SIZE),  //positive integer
        .BYTE_WRITE_WIDTH_A(OSBUF_ENTRY_SIZE),  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (OS_Q_ADDR_WIDTH),   //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (OSBUF_ENTRY_SIZE),  //positive integer
        .ADDR_WIDTH_B      (OS_Q_ADDR_WIDTH),   //positive integer
        .READ_RESET_VALUE_B("0"),               //string
        .READ_LATENCY_B    (2),                 //non-negative integer
        .WRITE_MODE_B      ("read_first")       //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_osq (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (osq_wr_addr_rd_valid),  //(i_wqe_push_req),
        .addra         (osq_wr_addr_new),
        .dina          (osq_in_2q),
        .injectsbiterra(1'b0),                  // A 端不注入单比特错误
        .injectdbiterra(1'b0),                  // A 端不注入双比特错误

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (1'b1),
        .regceb  (1'b1),
        .addrb   (osq_rd_addr_new),
        .doutb   ({osq_start_psn, osq_wqe_fatal_push, osq_wqe_retried, osq_msn, osq_end_psn, osq_opcode, osq_wrid}),
        //.doutb          (osq_out),
        .sbiterrb(),                                                                                                  // B 端单比特错误状态未接出
        .dbiterrb()                                                                                                   // B 端双比特错误状态未接出

    );

    assign o_psn_to_retry = psn_to_retry;
    assign o_ssn_to_retry = msn_to_retry;

    // End of xpm_memory_sdpram instance declaration


    // OSQ 队列管理器只维护每个 QP 的读写指针、水位、full 和 empty 状态。
    // 真正的 OSQ entry 数据存放在上面的 XPM RAM 中。这里的 OSQ 是硬件侧 outstanding
    // 发送队列，不是软件提交的原始 SQ。
    lrdma_queue_mgr #(
        .C_NUM_QUEUES(C_NUM_QP),           // Number of queues
        .C_Q_DEPTH   (C_OS_Q_DEPTH),       // Number of entries in each queue
        .C_ENTRY_SIZE(1),                  // 队列管理器内部地址步进单位，保持原配置
        .C_THRESHOLD (5),                  // almost_full 阈值参数；本顶层未接出 almost_full 结果
        .C_PTR_WIDTH (C_OS_Q_INDX_WIDTH),  // Pointer size reqd for accessing the Q depth 
        .C_ADDR_WIDTH(OS_Q_ADDR_WIDTH)
        //* C_ENTRY_SIZE - This parameter is used to generate the address for the
        //memory which is likely in DDR and generates a BYTE address. However
        //since we are directly using this address for he BRAM which houses the
        //OSQ the address is entry address (as each entry houses the entire OSQ
        //content for one valida transaction). hence a size of 1 is given so that
        //the address maps to the correct entry.
    ) inst_osq_manager (
        // Clock and Reset
        .core_clk(core_clk),   // Module Clock  
        .core_rst(~core_rstn), // Active high core reset

        // Queue pointers
        .i_wr_base_addr    (osq_base_addr_2d[i_wqe_qpid]),
        .i_wr_addr_qnum    (i_wqe_qpid),
        .i_wr_addr_rd_en   (wqe_push_for_non_nacked_qp),    // 读取写地址使能，这里和 push 信号同步，表示这次 push 的地址
        .o_wr_addr         (osq_wr_addr_new),               // 最新写地址
        .o_wr_addr_rd_valid(osq_wr_addr_rd_valid),          // i_wr_addr_rd_en 拉高后两周期输出 o_wr_addr_rd_valid

        .i_rd_base_addr    (osq_base_addr_2d[osq_sampled_arbitrated_idx]),
        .i_rd_addr_qnum    (osq_sampled_arbitrated_idx),
        .i_rd_addr_rd_en   (rd_osq),                                        // 读取读地址使能
        .o_rd_addr         (osq_rd_addr_new),                               // 最新读地址
        .o_rd_addr_rd_valid(osq_rd_addr_rd_valid),                          // i_rd_addr_rd_en 拉高后两周期输出 o_rd_addr_rd_valid

        // 推操作结束，队列为满信号
        .o_q_full_set_valid  (osq_full_set_valid),
        .o_q_full_set_id     (osq_full_set_id),
        // 弹操作 / 刷新操作 结束，就会输出o_q_full_reset_*信号，说明队列满状态重置，不满了
        .o_q_full_reset_valid(osq_full_reset_valid),
        .o_q_full_reset_id   (osq_full_reset_id),

        // almost_full 端口未接出，外部流控使用 full/empty 事件。
        .o_q_almost_full_set_valid  (),
        .o_q_almost_full_set_id     (),
        .o_q_almost_full_reset_valid(),
        .o_q_almost_full_reset_id   (),

        // 只要有推送，等待操作结束(回到PUSH_RD状态)，就会输出o_q_empty_reset_*信号，说明队列空状态重置，不空了
        .o_q_empty_reset_valid(osq_empty_reset_valid),
        .o_q_empty_reset_id   (osq_empty_reset_id),
        // 弹操作 / 刷新操作 结束，队列为空信号
        .o_q_empty_set_valid  (osq_empty_set_valid),
        .o_q_empty_set_id     (osq_empty_set_id),

        .vld_free_entry_rden    (1'b0),
        .vld_free_entry_qnum    ({C_QP_INDX_WIDTH{1'b0}}),
        .vld_free_entry_data_vld(),
        .o_num_valid_entries    (selected_num_valid_osq_entries),
        .o_num_free_entries     (),
        .i_q_flush              (1'b0),
        .i_q_flush_qpid         ({C_QP_INDX_WIDTH{1'b0}}),
        .o_q_flush_done         (),

        .i_q_depth(C_OS_Q_DEPTH[C_OS_Q_INDX_WIDTH:0]),

        // OSQ push/pop：
        // push 来自 TX 引擎登记的新 outstanding WQE；
        // pop 来自 FSM 对仲裁 QP 的完成处理或重传 flush 处理。
        .i_q_push     (wqe_push_for_non_nacked_qp),
        .i_q_push_qpid(i_wqe_qpid),
        .i_q_pop      (pop_osq_from_fsm & !osq_empty_rd_data_0),
        .i_q_pop_qpid (osq_sampled_arbitrated_idx),
        .o_q_pop_idle (),
        .o_q_push_idle(),
        .init_start   (init_start),
        .init_count   (init_count)
    );

    always_ff @(posedge core_clk) begin : osq_empty_ff
        if (!core_rstn) begin
            osq_empty_ff_valid         <= 1'b0;
            osq_empty_ff_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_empty_ff_data          <= 1'b0;
            osq_empty_ff_valid_pending <= 1'b0;
            osq_empty_ff_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (osq_empty_set_valid) begin  // 空
                osq_empty_ff_valid <= 1'b1;
                osq_empty_ff_id    <= osq_empty_set_id;
                osq_empty_ff_data  <= 1'b1;
            end else if (osq_empty_reset_valid) begin  // 不空
                osq_empty_ff_valid <= 1'b1;
                osq_empty_ff_id    <= osq_empty_reset_id;
                osq_empty_ff_data  <= 1'b0;
            end else if (osq_empty_ff_valid_pending) begin  // 不空
                osq_empty_ff_valid <= 1'b1;
                osq_empty_ff_id    <= osq_empty_ff_id_pending;
                osq_empty_ff_data  <= 1'b0;
            end else begin
                osq_empty_ff_valid <= 1'b0;
            end

            // 同时发生空和非空事件
            // 一个队列刚被 push（从空变非空）→ reset 脉冲。另一个队列同周期 pop 后变空 → set 脉冲。
            if (osq_empty_set_valid && osq_empty_reset_valid) begin
                osq_empty_ff_valid_pending <= 1'b1;
                osq_empty_ff_id_pending    <= osq_empty_reset_id;  // 非空的队列 ID
            end else begin
                osq_empty_ff_valid_pending <= 1'b0;
            end
        end
    end

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (osq_sampled_arbitrated_idx),
        .rd_data (osq_empty_rd_data_int_0)
    );
    // 如果 osq_sampled_arbitrated_idx 恰好就是这个 pending 的 reset 队列，真实状态已经“非空”，但 RAM 还没更新。为了不给 FSM 造成“误判为空而跳过处理”，用条件覆盖直接输出 0（非空），实现提前可见。
    assign osq_empty_rd_data_0 = (osq_empty_ff_valid_pending && osq_empty_ff_id_pending == osq_sampled_arbitrated_idx) ? 1'b0 : osq_empty_rd_data_int_0;

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_1 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (o_osq_pop_qpid),
        .rd_data (osq_empty_rd_data_int_1)
    );
    // 如果 o_osq_pop_qpid 恰好就是这个 pending 的 reset 队列，真实状态已经“非空”，但 RAM 还没更新。为了不给 FSM 造成“误判为空而跳过处理”，用条件覆盖直接输出 0（非空），实现提前可见。
    assign osq_empty_rd_data_1 = (osq_empty_ff_valid_pending && osq_empty_ff_id_pending == o_osq_pop_qpid) ? 1'b0 : osq_empty_rd_data_int_1;

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_2 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (osq_arbitrated_idx),
        .rd_data (osq_empty_rd_data_int_2)
    );
    // 如果 osq_arbitrated_idx 恰好就是这个 pending 的 reset 队列，真实状态已经“非空”，但 RAM 还没更新。为了不给 FSM 造成“误判为空而跳过处理”，用条件覆盖直接输出 0（非空），实现提前可见。
    assign osq_empty_rd_data_2 = (osq_empty_ff_valid_pending && osq_empty_ff_id_pending == osq_arbitrated_idx) ? 1'b0 : osq_empty_rd_data_int_2;

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_3 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_fatal_id),
        .rd_data (osq_empty_rd_data_int_3)
    );
    // 如果 i_qp_fatal_id 恰好就是这个 pending 的 reset 队列，真实状态已经“非空”，但 RAM 还没更新。为了不给 FSM 造成“误判为空而跳过处理”，用条件覆盖直接输出 0（非空），实现提前可见。
    assign osq_empty_rd_data_3 = (osq_empty_ff_valid_pending && osq_empty_ff_id_pending == i_qp_fatal_id) ? 1'b0 : osq_empty_rd_data_int_3;

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1),
        .C_COMBO_BYPASS(1)
    ) inst_osq_empty_4 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty_ff_valid),
        .wr_addr (init_start ? init_count : osq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : osq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (osq_empty_ff_id),
        .rd_data (osq_empty_ff_id_rd_data)
    );

    // osq_empty And_reduce logic
    // 检测所有OSQ是否都为空
    always_ff @(posedge core_clk) begin : and_reduce_osq_empty
        if (~core_rstn) begin
            osq_empty_zeros_count <= {C_QP_INDX_WIDTH + 1{1'b0}};
        end else begin
            case ({
                osq_empty_ff_valid, osq_empty_ff_data, osq_empty_ff_id_rd_data
            })
                3'b110:  osq_empty_zeros_count <= osq_empty_zeros_count - 1;
                3'b101:  osq_empty_zeros_count <= osq_empty_zeros_count + 1;
                default: osq_empty_zeros_count <= osq_empty_zeros_count;
            endcase
        end
    end
    assign ack_nack_to_process        = (osq_empty_zeros_count != 'd0) ? 1'b1 : 1'b0;  // 1-表示至少有一个 OSQ 非空，0-表示所有 OSQ 都为空

    assign o_wqe_push_rdy             = 1'b1;  // 可稍后更改，以增加延迟

    assign o_osq_empty_set_valid      = osq_empty_set_valid;
    assign o_osq_empty_set_id         = osq_empty_set_id;
    assign o_osq_empty_reset_valid    = osq_empty_reset_valid;
    assign o_osq_empty_reset_id       = osq_empty_reset_id;

    assign wqe_push_for_non_nacked_qp = (osq_nacked_ff_rd_data_1 & i_osq_wqe_nack_resp_rd_data_0) ? 1'b0 : i_wqe_push_req;


    //###############################################################################################################################################
    //`MSFF_R(osq_timed_out_sticky[j], ((i_qp_disable_pulse[j] | clr_timeout_n_nack[j] | timer_reload_for_acknack_ff[j])? 1'b0 : osq_timed_out_sticky_wire[j]),
    // core_clk, ~core_rstn)

    always_ff @(posedge core_clk) begin : osq_timed_out_sticky
        if (!core_rstn) begin
            osq_timed_out_sticky_valid             <= 1'b0;
            osq_timed_out_sticky_id                <= {C_QP_INDX_WIDTH{1'b0}};
            osq_timed_out_sticky_data              <= 1'b0;
            osq_timed_out_sticky_valid_aux         <= 1'b0;
            osq_timed_out_sticky_id_aux            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_timed_out_sticky_data_aux          <= 1'b0;
            osq_timed_out_sticky_valid_pending     <= 1'b0;
            osq_timed_out_sticky_id_pending        <= {C_QP_INDX_WIDTH{1'b0}};
            osq_timed_out_sticky_valid_aux_pending <= 1'b0;
            osq_timed_out_sticky_id_aux_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_qp_disable_pulse_set_valid_edge) begin
                osq_timed_out_sticky_valid <= 1'b1;
                osq_timed_out_sticky_id    <= i_qp_disable_pulse_set_id;
                osq_timed_out_sticky_data  <= 1'b0;
            end else if (clr_timeout_n_nack_set_valid) begin
                osq_timed_out_sticky_valid <= 1'b1;
                osq_timed_out_sticky_id    <= clr_timeout_n_nack_set_id;
                osq_timed_out_sticky_data  <= 1'b0;
            end else if (osq_timed_out_sticky_valid_pending && !((i_acknack_valid || osq_nacked_ff_comb) && i_qp_index_acknack == osq_timed_out_sticky_id_pending) && !(osq_timed_out_set_valid && osq_timed_out_set_id == osq_timed_out_sticky_id_pending)) begin
                osq_timed_out_sticky_valid <= 1'b1;
                osq_timed_out_sticky_id    <= osq_timed_out_sticky_id_pending;
                osq_timed_out_sticky_data  <= 1'b0;
            end else begin
                osq_timed_out_sticky_valid <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge && clr_timeout_n_nack_set_valid && i_qp_disable_pulse_set_id != clr_timeout_n_nack_set_id) begin
                osq_timed_out_sticky_valid_pending <= 1'b1;
                osq_timed_out_sticky_id_pending    <= clr_timeout_n_nack_set_id;
            end else begin
                osq_timed_out_sticky_valid_pending <= 1'b0;
            end

            if ((i_acknack_valid || osq_nacked_ff_comb) && !(i_qp_disable_pulse_set_valid_edge && i_qp_index_acknack == i_qp_disable_pulse_set_id) && !(clr_timeout_n_nack_set_valid && i_qp_index_acknack == clr_timeout_n_nack_set_id)) begin
                osq_timed_out_sticky_valid_aux <= 1'b1;
                osq_timed_out_sticky_id_aux    <= i_qp_index_acknack;
                osq_timed_out_sticky_data_aux  <= 1'b0;
            end else if (osq_timed_out_set_valid && !(i_qp_disable_pulse_set_valid_edge && osq_timed_out_set_id == i_qp_disable_pulse_set_id) && !(clr_timeout_n_nack_set_valid && osq_timed_out_set_id == clr_timeout_n_nack_set_id)) begin
                osq_timed_out_sticky_valid_aux <= 1'b1;
                osq_timed_out_sticky_id_aux    <= osq_timed_out_set_id;
                osq_timed_out_sticky_data_aux  <= 1'b1;
            end else if (osq_timed_out_sticky_valid_aux_pending && !(i_qp_disable_pulse_set_valid_edge && osq_timed_out_sticky_id_aux_pending == i_qp_disable_pulse_set_id) && !(clr_timeout_n_nack_set_valid && osq_timed_out_sticky_id_aux_pending == clr_timeout_n_nack_set_id)) begin
                osq_timed_out_sticky_valid_aux <= 1'b1;
                osq_timed_out_sticky_id_aux    <= osq_timed_out_sticky_id_aux_pending;
                osq_timed_out_sticky_data_aux  <= 1'b1;
            end else begin
                osq_timed_out_sticky_valid_aux <= 1'b0;
            end

            if (osq_timed_out_set_valid && (i_acknack_valid || osq_nacked_ff_comb) && i_qp_index_acknack != osq_timed_out_set_id && !(i_qp_disable_pulse_set_valid_edge 
		&& (osq_timed_out_set_id == i_qp_disable_pulse_set_id || i_qp_index_acknack == i_qp_disable_pulse_set_id)) && !(clr_timeout_n_nack_set_valid && (osq_timed_out_set_id == clr_timeout_n_nack_set_id
		|| i_qp_index_acknack == clr_timeout_n_nack_set_id))) begin
                osq_timed_out_sticky_valid_aux_pending <= 1'b1;
                osq_timed_out_sticky_id_aux_pending    <= osq_timed_out_set_id;
            end else begin
                osq_timed_out_sticky_valid_aux_pending <= 1'b0;
            end

        end
    end

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_timed_out_0 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || osq_timed_out_sticky_valid),
        .wr_addr    (init_start ? init_count : osq_timed_out_sticky_id),
        .wr_data    (init_start ? 1'b0 : osq_timed_out_sticky_data),
        .wr_en_aux  (osq_timed_out_sticky_valid_aux),
        .wr_addr_aux(osq_timed_out_sticky_id_aux),
        .wr_data_aux(osq_timed_out_sticky_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (fsm_i_arbitrated_indx),
        .rd_data    (osq_timed_out_sticky_rd_data_int_0)
    );
    always_comb begin
        if (osq_timed_out_sticky_valid_pending && osq_timed_out_sticky_id_pending == fsm_i_arbitrated_indx) begin
            osq_timed_out_sticky_rd_data_0 = 1'b0;
        end else if (osq_timed_out_sticky_valid_aux_pending && osq_timed_out_sticky_id_aux_pending == fsm_i_arbitrated_indx) begin
            osq_timed_out_sticky_rd_data_0 = 1'b1;
        end else begin
            osq_timed_out_sticky_rd_data_0 = osq_timed_out_sticky_rd_data_int_0;
        end
    end
    `MSFF_R(osq_timed_out_sticky_rd_data_0_ff, osq_timed_out_sticky_rd_data_0, core_clk, !core_rstn)

    assign osq_timed_out_sticky_fsm_arbitrated_indx = osq_timed_out_sticky_rd_data_0_ff;


    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_timed_out_1 (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || osq_timed_out_sticky_valid),
        .wr_addr    (init_start ? init_count : osq_timed_out_sticky_id),
        .wr_data    (init_start ? 1'b0 : osq_timed_out_sticky_data),
        .wr_en_aux  (osq_timed_out_sticky_valid_aux),
        .wr_addr_aux(osq_timed_out_sticky_id_aux),
        .wr_data_aux(osq_timed_out_sticky_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (timer_mem_rdaddr),
        .rd_data    (osq_timed_out_sticky_rd_data_int_1)
    );
    always_comb begin
        if (osq_timed_out_sticky_valid_pending && osq_timed_out_sticky_id_pending == timer_mem_rdaddr) begin
            osq_timed_out_sticky_rd_data_1 = 1'b0;
        end else if (osq_timed_out_sticky_valid_aux_pending && osq_timed_out_sticky_id_aux_pending == timer_mem_rdaddr) begin
            osq_timed_out_sticky_rd_data_1 = 1'b1;
        end else begin
            osq_timed_out_sticky_rd_data_1 = osq_timed_out_sticky_rd_data_int_1;
        end
    end

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_osq_nacked_ff_comp (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_nacked_ff_valid),
        .wr_addr (init_start ? init_count : osq_nacked_ff_id),
        .wr_data (init_start ? 1'b0 : osq_nacked_ff_data),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr),
        .rd_data (osq_nacked_ff_rd_data_comp_int)
    );
    assign osq_nacked_ff_rd_data_comp = (osq_nacked_ff_valid_pending && osq_nacked_ff_id_pending == timer_mem_rdaddr) ? 1'b1 : osq_nacked_ff_rd_data_comp_int;
    `MSFF_R(osq_nacked_ff_rd_data_comp_ff, osq_nacked_ff_rd_data_comp, core_clk, !core_rstn)

    `MSFF_R(osq_nacked_timer_mem_rdaddr_i, osq_timed_out_sticky_rd_data_1, core_clk, !core_rstn)
    assign osq_nacked_timer_mem_rdaddr = osq_nacked_timer_mem_rdaddr_i || osq_nacked_ff_rd_data_comp_ff;

    // o_osq_nacked 同时上报 set/reset 事件及其 pending 项。set 来源包括 timeout 和有效 NACK；
    // reset 来源包括 QP disable 和 FSM 在重传请求被接受后的 clear。
    // 打包顺序：{set_valid,set_id,set_valid_pending,set_id_pending,reset_valid,reset_id,reset_valid_pending,reset_id_pending}
    assign o_osq_nacked                = {o_osq_nacked_set_valid, o_osq_nacked_set_id, o_osq_nacked_set_valid_pending, o_osq_nacked_set_id_pending, o_osq_nacked_reset_valid, o_osq_nacked_reset_id, o_osq_nacked_reset_valid_pending, o_osq_nacked_reset_id_pending};

    always_ff @(posedge core_clk) begin : o_osq_nacked_proc
        if (!core_rstn) begin
            o_osq_nacked_set_valid           <= 1'b0;
            o_osq_nacked_set_id              <= {C_QP_INDX_WIDTH{1'b0}};
            o_osq_nacked_reset_valid         <= 1'b0;
            o_osq_nacked_reset_id            <= {C_QP_INDX_WIDTH{1'b0}};
            o_osq_nacked_set_valid_pending   <= 1'b0;
            o_osq_nacked_set_id_pending      <= {C_QP_INDX_WIDTH{1'b0}};
            o_osq_nacked_reset_valid_pending <= 1'b0;
            o_osq_nacked_reset_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (osq_timed_out_set_valid && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == osq_timed_out_set_id) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == osq_timed_out_set_id) && !(i_acknack_valid && i_qp_index_acknack == osq_timed_out_set_id)) begin
                o_osq_nacked_set_valid <= 1'b1;  //timeout
                o_osq_nacked_set_id    <= osq_timed_out_set_id;
            end else if (i_acknack_valid && nak_seq_error && !(i_halt && i_halted_qpid == i_qp_index_acknack) && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == i_qp_index_acknack) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == i_qp_index_acknack)) begin
                o_osq_nacked_set_valid <= 1'b1;  //nack
                o_osq_nacked_set_id    <= i_qp_index_acknack;
            end else if (o_osq_nacked_set_valid_pending && !(i_qp_disable_pulse_set_valid_edge && i_qp_disable_pulse_set_id == o_osq_nacked_set_id_pending) && !(clr_timeout_n_nack_set_valid && clr_timeout_n_nack_set_id == o_osq_nacked_set_id_pending)) begin
                o_osq_nacked_set_valid <= 1'b1;
                o_osq_nacked_set_id    <= o_osq_nacked_set_id_pending;
            end else begin
                o_osq_nacked_set_valid <= 1'b0;
            end

            if ((i_qp_disable_pulse_set_valid_edge && (i_qp_disable_pulse_set_id == osq_timed_out_set_id || i_qp_disable_pulse_set_id == i_qp_index_acknack)) || (clr_timeout_n_nack_set_valid && 
		(clr_timeout_n_nack_set_id == osq_timed_out_set_id || clr_timeout_n_nack_set_id == i_qp_index_acknack))) begin
                o_osq_nacked_set_valid_pending <= 1'b0;
            end else if (osq_timed_out_set_valid && i_acknack_valid && (nak_seq_error && !(i_halt && i_halted_qpid == i_qp_index_acknack)) && osq_timed_out_set_id != i_qp_index_acknack) begin
                o_osq_nacked_set_valid_pending <= 1'b1;
                o_osq_nacked_set_id_pending    <= i_qp_index_acknack;
            end else begin
                o_osq_nacked_set_valid_pending <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge) begin
                o_osq_nacked_reset_valid <= 1'b1;  //qp_reset
                o_osq_nacked_reset_id    <= i_qp_disable_pulse_set_id;
            end else if (clr_timeout_n_nack_set_valid) begin
                o_osq_nacked_reset_valid <= 1'b1;  //fsm clear req
                o_osq_nacked_reset_id    <= clr_timeout_n_nack_set_id;
            end else if (o_osq_nacked_reset_valid_pending && !(osq_timed_out_set_valid && !(i_acknack_valid && i_qp_index_acknack == osq_timed_out_set_id) && osq_timed_out_set_id
		== o_osq_nacked_reset_id_pending) && !(i_acknack_valid && (nak_seq_error && !(i_halt && i_halted_qpid == i_qp_index_acknack)) && i_qp_index_acknack
		== o_osq_nacked_reset_id_pending)) begin
                o_osq_nacked_reset_valid <= 1'b1;
                o_osq_nacked_reset_id    <= o_osq_nacked_reset_id_pending;
            end else begin
                o_osq_nacked_reset_valid <= 1'b0;
            end

            if (i_qp_disable_pulse_set_valid_edge && clr_timeout_n_nack_set_valid && i_qp_disable_pulse_set_id != clr_timeout_n_nack_set_id) begin
                o_osq_nacked_reset_valid_pending <= 1'b1;
                o_osq_nacked_reset_id_pending    <= clr_timeout_n_nack_set_id;
            end else begin
                o_osq_nacked_reset_valid_pending <= 1'b0;
            end
        end
    end

    genvar i;
    generate
        for (i = 0; i < C_NUM_QP; i = i + 1) begin : gen_osq_param
            assign osq_base_addr_2d[i] = (i * C_OS_Q_DEPTH);
        end
    endgenerate


    assign o_osq_pop_qpid = osq_sampled_arbitrated_idx;
    assign o_osq_pop      = pop_osq_from_fsm & ~osq_empty_rd_data_1;

    //`MSFF_R(wqe_cmpl_intr_sts[n], (i_clr_wqe_cmpl_sts[n] ? 1'b0 : ((arbitrated_idx_for_intr == n) ? (intr_wqe_cmpl_sts || wqe_cmpl_intr_sts[n]) : 
    //                                                                    wqe_cmpl_intr_sts[n])) , core_clk, ~core_rstn)

    genvar k;
    generate
        if (C_NUM_QP > 32) begin : wqe_cmpl_intr_sts
            reg [(clog2(C_NUM_QP/32)) : 0] num_ones_in_cmpl_intr_lut[32];
            // 软件一次按 32 bit 清完成中断状态。若清除和新的完成事件同拍到达，
            // pending 路径保存新的完成事件，避免被清除操作覆盖。
            always_ff @(posedge core_clk) begin : wqe_cmpl_intr_sts_proc
                if (~core_rstn) begin
                    wqe_cmpl_intr_sts_valid         <= {32{1'b0}};
                    wqe_cmpl_intr_sts_data          <= {32{1'b0}};
                    wqe_cmpl_intr_sts_valid_int     <= {32{1'b0}};
                    wqe_cmpl_intr_sts_id            <= {C_QP_INDX_WIDTH{1'b0}};
                    wqe_cmpl_intr_sts_valid_pending <= {32{1'b0}};
                    wqe_cmpl_intr_sts_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    if (|i_clr_wqe_cmpl_sts_set_valid) begin
                        wqe_cmpl_intr_sts_valid     <= i_clr_wqe_cmpl_sts_set_valid;
                        wqe_cmpl_intr_sts_id        <= i_clr_wqe_cmpl_sts_set_id;
                        wqe_cmpl_intr_sts_valid_int <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data      <= {32{1'b0}};
                    end else if (intr_wqe_cmpl_sts) begin
                        wqe_cmpl_intr_sts_valid_int[arbitrated_idx_for_intr[4:0]] <= 1'b1;
                        wqe_cmpl_intr_sts_id                                      <= {arbitrated_idx_for_intr[C_QP_INDX_WIDTH-1 : 5], 5'd0};
                        wqe_cmpl_intr_sts_valid                                   <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data                                    <= {32{1'b1}};
                    end else if (|wqe_cmpl_intr_sts_valid_pending) begin
                        wqe_cmpl_intr_sts_valid_int <= wqe_cmpl_intr_sts_valid_pending;
                        wqe_cmpl_intr_sts_id        <= wqe_cmpl_intr_sts_id_pending;
                        wqe_cmpl_intr_sts_valid     <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data      <= {32{1'b1}};
                    end else begin
                        wqe_cmpl_intr_sts_valid     <= {32{1'b0}};
                        wqe_cmpl_intr_sts_valid_int <= {32{1'b0}};
                    end

                    if (intr_wqe_cmpl_sts && !(i_clr_wqe_cmpl_sts_set_valid[arbitrated_idx_for_intr[4:0]] == 1'b1 && arbitrated_idx_for_intr[C_QP_INDX_WIDTH-1:5] == wqe_cmpl_intr_sts_id[C_QP_INDX_WIDTH-1:5])) begin
                        wqe_cmpl_intr_sts_valid_pending[arbitrated_idx_for_intr[4:0]] <= 1'b1;
                        wqe_cmpl_intr_sts_id_pending                                  <= {arbitrated_idx_for_intr[C_QP_INDX_WIDTH-1 : 5], 5'd0};
                    end else begin
                        wqe_cmpl_intr_sts_valid_pending <= {32{1'b0}};
                    end
                end
            end

            for (k = 0; k <= 31; k = k + 1) begin
                lrdma_common_sdp_lut_ram_wrapper #(
                    .C_RAM_WORD_WIDTH(1),
                    .C_COMBO_BYPASS(1),
                    .C_RAM_DEPTH(C_NUM_QP / 32)
                ) inst_wqe_cmpl_intr_sts (
                    .core_clk(core_clk),
                    .core_rst(!core_rstn),
                    .wr_en   (init_start || wqe_cmpl_intr_sts_valid[k] || wqe_cmpl_intr_sts_valid_int[k]),
                    .wr_addr (init_start ? init_count[C_QP_INDX_WIDTH-1 : 5] : wqe_cmpl_intr_sts_id[C_QP_INDX_WIDTH-1 : 5]),
                    .wr_data (init_start ? 1'b0 : (wqe_cmpl_intr_sts_valid[k] ? 1'b0 : 1'b1)),
                    .rd_en   (1'b1),
                    .rd_addr ((wqe_cmpl_intr_sts_id[C_QP_INDX_WIDTH-1 : 5])),
                    .rd_data (wqe_cmpl_intr_sts_rd_data[k])
                );

                always_ff @(posedge core_clk) begin
                    if (!core_rstn) begin
                        num_ones_in_cmpl_intr_lut[k] <= 'd0;
                    end else begin
                        //case({wqe_cmpl_intr_sts_valid[k],wqe_cmpl_intr_sts_valid_int[k],wqe_cmpl_intr_sts_rd_data[k]})
                        case ({
                            (wqe_cmpl_intr_sts_valid[k] || wqe_cmpl_intr_sts_valid_int[k]), wqe_cmpl_intr_sts_data[k], wqe_cmpl_intr_sts_rd_data[k]
                        })
                            3'b101:  num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k] - 1;
                            3'b110:  num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k] + 1;
                            default: num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k];
                        endcase
                    end
                end
            end  //loop

            always_comb begin
                any_ones_in_cmpl_intr_lut = 0;
                for (integer i = 0; i <= 31; i = i + 1) begin
                    any_ones_in_cmpl_intr_lut = any_ones_in_cmpl_intr_lut || (|num_ones_in_cmpl_intr_lut[i]);
                end
            end
        end // wqe_cmpl_intr_sts

        else begin : wqe_cmpl_intr_sts_less_than_32
            // 软件一次按 32 bit 清完成中断状态。若清除和新的完成事件同拍到达，
            // pending 路径保存新的完成事件，避免被清除操作覆盖。
            always_ff @(posedge core_clk) begin : wqe_cmpl_intr_sts_proc
                if (~core_rstn) begin
                    wqe_cmpl_intr_sts_valid         <= {32{1'b0}};
                    wqe_cmpl_intr_sts_data          <= {32{1'b0}};
                    wqe_cmpl_intr_sts_valid_int     <= {32{1'b0}};
                    wqe_cmpl_intr_sts_id            <= {C_QP_INDX_WIDTH{1'b0}};
                    wqe_cmpl_intr_sts_valid_pending <= {32{1'b0}};
                    wqe_cmpl_intr_sts_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
                end else begin
                    if (|i_clr_wqe_cmpl_sts_set_valid) begin
                        wqe_cmpl_intr_sts_valid     <= i_clr_wqe_cmpl_sts_set_valid;
                        wqe_cmpl_intr_sts_id        <= i_clr_wqe_cmpl_sts_set_id;
                        wqe_cmpl_intr_sts_valid_int <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data      <= {32{1'b0}};
                    end else if (intr_wqe_cmpl_sts) begin
                        wqe_cmpl_intr_sts_valid_int[arbitrated_idx_for_intr] <= 1'b1;
                        wqe_cmpl_intr_sts_id                                 <= arbitrated_idx_for_intr;
                        wqe_cmpl_intr_sts_valid                              <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data                               <= {32{1'b1}};
                    end else if (|wqe_cmpl_intr_sts_valid_pending) begin
                        wqe_cmpl_intr_sts_valid_int <= wqe_cmpl_intr_sts_valid_pending;
                        wqe_cmpl_intr_sts_id        <= wqe_cmpl_intr_sts_id_pending;
                        wqe_cmpl_intr_sts_valid     <= {32{1'b0}};
                        wqe_cmpl_intr_sts_data      <= {32{1'b1}};
                    end else begin
                        wqe_cmpl_intr_sts_valid     <= {32{1'b0}};
                        wqe_cmpl_intr_sts_valid_int <= {32{1'b0}};
                    end

                    if (intr_wqe_cmpl_sts && !(i_clr_wqe_cmpl_sts_set_valid[arbitrated_idx_for_intr] == 1'b1 && arbitrated_idx_for_intr == wqe_cmpl_intr_sts_id)) begin
                        wqe_cmpl_intr_sts_valid_pending[arbitrated_idx_for_intr] <= 1'b1;
                        wqe_cmpl_intr_sts_id_pending                             <= arbitrated_idx_for_intr;
                    end else begin
                        wqe_cmpl_intr_sts_valid_pending <= {32{1'b0}};
                    end
                end
            end

            reg [31 : 0] num_ones_in_cmpl_intr_lut;
            reg [  31:0] wqe_cmpl_intr_sts_rd_data_reg;



            for (k = 0; k <= 31; k = k + 1) begin
                always_ff @(posedge core_clk) begin
                    if (!core_rstn) begin
                        wqe_cmpl_intr_sts_rd_data_reg[k] <= 1'b0;
                    end else begin
                        if (wqe_cmpl_intr_sts_valid_int[k] || wqe_cmpl_intr_sts_valid[k]) begin
                            wqe_cmpl_intr_sts_rd_data_reg[k] <= wqe_cmpl_intr_sts_data[k];
                        end
                    end
                end

                always_ff @(posedge core_clk) begin
                    if (!core_rstn) begin
                        num_ones_in_cmpl_intr_lut[k] <= 1'b0;
                    end else begin
                        case ({
                            (wqe_cmpl_intr_sts_valid[k] || wqe_cmpl_intr_sts_valid_int[k]), wqe_cmpl_intr_sts_data[k], wqe_cmpl_intr_sts_rd_data_reg[k]
                        })
                            3'b101:  num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k] - 1;
                            3'b110:  num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k] + 1;
                            default: num_ones_in_cmpl_intr_lut[k] <= num_ones_in_cmpl_intr_lut[k];
                        endcase
                    end
                end
            end

            always_comb begin
                any_ones_in_cmpl_intr_lut = |num_ones_in_cmpl_intr_lut;
            end

        end  // wqe_cmpl_intr_sts_less_than_32
    endgenerate  // C_NUM_QP <= 32



    `MSFF_R(wqe_cmpl_intr_sts_or_reduce, any_ones_in_cmpl_intr_lut, core_clk, !core_rstn)

    assign o_intr_wqe_cmpl_sts_wr_en           = wqe_cmpl_intr_sts_valid | wqe_cmpl_intr_sts_valid_int;
    assign o_intr_wqe_cmpl_sts_wr_addr         = wqe_cmpl_intr_sts_id;
    assign o_intr_wqe_cmpl_sts_wr_data         = wqe_cmpl_intr_sts_data;
    assign o_intr_wqe_cmpl_sts_wr_en_pending   = wqe_cmpl_intr_sts_valid_pending;
    assign o_intr_wqe_cmpl_sts_wr_addr_pending = wqe_cmpl_intr_sts_id_pending;

    assign o_intr_wqe_cmpl                     = (wqe_cmpl_intr_sts_or_reduce != 1'b0) && i_intr_en_wqe_cmpl;
    assign o_intr_wqe_cmpl_sts_or_reduce       = wqe_cmpl_intr_sts_or_reduce;

    //----------------------------------------------------------------------------------


    //----------------------------------------------------------------------------------
    // QP 处理仲裁入口。ACK/NAK 直接按响应包 QP 入队；QP1 没有普通 ACK，因此由 WQE push
    // 触发一次特殊处理；timeout/fatal 以及 FSM pushback 也会重新进入仲裁。
    always_ff @(posedge core_clk) begin
        if (~core_rstn) begin
            QP1_acknack_valid <= 1'b0;
            pushback_valid    <= 1'b0;
            arbiter_valid_in  <= 1'b0;
        end else begin
            if (wqe_push_for_non_nacked_qp && i_wqe_qpid == {{C_QP_INDX_WIDTH - 1{1'b0}}, QP1}) begin
                QP1_acknack_valid <= 1'b1;
            end else if (~i_acknack_valid) begin
                QP1_acknack_valid <= 1'b0;
            end

            arbiter_valid_in <= i_acknack_valid || QP1_acknack_valid || pushback_timeout_fatal_valid;
            if (i_acknack_valid) begin
                arbiter_Qpid_in <= i_qp_index_acknack;
            end else if (QP1_acknack_valid) begin
                arbiter_Qpid_in <= {{C_QP_INDX_WIDTH - 1{1'b0}}, QP1};
            end else if (pushback_timeout_fatal_valid) begin
                arbiter_Qpid_in <= pushback_timeout_fatal_id;
            end

            // FSM 处理 NACK、timeout 或 fatal QP 时可能需要多轮仲裁。pushback_valid 表示该 QP
            // 还不能完成处理，需要重新放回待处理队列。
            //if (arbitrate && (fsm_pushback_valid || (!osq_empty_rd_data_2 && i_qp_fatal_rd_data_1))) begin
            if (arbitrate && fsm_pushback_valid) begin
                pushback_valid <= 1'b1;
                pushback_Qpid  <= osq_arbitrated_idx;
            end else if (~fifo_busy) begin
                pushback_valid <= 1'b0;
            end
        end
    end

    // 如果仲裁出的 QP OSQ 已空，则跳过该结果，避免 FSM 空转。
    always_ff @(posedge core_clk) begin
        no_OS_pending_r        <= no_OS_pending;
        arbitration_done_int_r <= arbitration_done_int;
        if (arbitrate) begin
            arbitration_done <= 1'b0;
        end else begin
            arbitration_done <= arbitration_done_int && ~no_OS_pending && ~no_OS_pending_r;
        end

    end
    assign no_OS_pending = osq_empty_rd_data_2 && arbitration_done_int && ~arbitration_done_int_r;

    lrdma_common_qpid_fifo_arbiter #(
        .C_MAX_QP(C_NUM_QP),
        .C_MAX_QID_WIDTH(C_QP_INDX_WIDTH)
    ) inst_ack_nack_resp_Arbiter (
        .core_clk        (core_clk),
        .core_rst        (~core_rstn),                 //Arbiter reset is active high
        .init_start      (init_start),                 //lut ram seq_reset interface 
        .init_count      (init_count),                 //lut ram seq_reset interface 
        .valid_in        (arbiter_valid_in),
        .Qpid_in         (arbiter_Qpid_in),
        .fetch_next_id   (arbitrate | no_OS_pending),
        .pushback_Qpid   (pushback_Qpid),
        .pushback_valid  (pushback_valid),
        .arbitration_done(arbitration_done_int),
        .arbitrated_Qpid (osq_arbitrated_idx),
        .fifo_busy       (fifo_busy)
    );


    /******************* 缓存 timeout / fatal / under_recovery QP，之后作为 pushback 事件重新进入仲裁 *************************************************/
    wire                         push_is_avail_A;
    wire                         push_is_avail_B;
    reg                          i_qp_enable_valid_r;
    reg                          osq_empty_reset_valid_r;
    reg  [C_QP_INDX_WIDTH-1 : 0] osq_empty_reset_id_r;

    xpm_memory_tdpram #(
        .ADDR_WIDTH_A           (C_QP_INDX_WIDTH),
        .ADDR_WIDTH_B           (C_QP_INDX_WIDTH),
        .AUTO_SLEEP_TIME        (0),
        .BYTE_WRITE_WIDTH_A     (C_QP_INDX_WIDTH),
        .BYTE_WRITE_WIDTH_B     (C_QP_INDX_WIDTH),
        .CASCADE_HEIGHT         (0),
        .CLOCKING_MODE          ("common_clock"),
        .ECC_MODE               ("no_ecc"),
        .MEMORY_INIT_FILE       ("none"),
        .MEMORY_INIT_PARAM      ("0"),
        .MEMORY_OPTIMIZATION    ("true"),
        .MEMORY_PRIMITIVE       ("auto"),
        .MEMORY_SIZE            (C_QP_INDX_WIDTH * C_NUM_QP),
        .MESSAGE_CONTROL        (0),
        .READ_DATA_WIDTH_A      (C_QP_INDX_WIDTH),
        .READ_DATA_WIDTH_B      (C_QP_INDX_WIDTH),
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
        .WRITE_DATA_WIDTH_A     (C_QP_INDX_WIDTH),
        .WRITE_DATA_WIDTH_B     (C_QP_INDX_WIDTH),
        .WRITE_MODE_A           ("read_first"),
        .WRITE_MODE_B           ("read_first")
    ) inst_fatal_timedout_qpid (
        .clka          (core_clk),
        .rsta          (!core_rstn),
        .regcea        (1'b1),
        .ena           (push_wr_en_A && push_is_avail_A),
        .wea           (push_wr_en_A && push_is_avail_A),
        .addra         (Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH-1 : 0]),
        .dina          (push_wr_id_A),
        .douta         (),
        .clkb          (core_clk),
        .rstb          (!core_rstn),
        .regceb        (1'b1),
        .enb           ((push_wr_en_B && push_is_avail_B) || pop_wr_en_B),
        .web           ((push_wr_en_B && push_is_avail_B) && !pop_wr_en_B),
        .addrb         (buffer_addr_B),
        .dinb          (push_wr_id_B),
        .doutb         (fatal_timeout_qpid),
        .sleep         (1'b0),
        .injectdbiterra(1'b0),
        .injectsbiterra(1'b0),
        .dbiterra      (),
        .sbiterra      (),
        .injectdbiterrb(1'b0),
        .injectsbiterrb(1'b0),
        .dbiterrb      (),
        .sbiterrb      ()
    );

    assign fifo_empty = (Qpid_buffer_wr_ptr == Qpid_buffer_rd_ptr) ? 1'b1 : 1'b0;
    assign fifo_full  = (Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH] != Qpid_buffer_rd_ptr[C_QP_INDX_WIDTH] && Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH-1 : 0] == Qpid_buffer_rd_ptr[C_QP_INDX_WIDTH-1 : 0]) ? 1'b1 : 1'b0;

    always_ff @(posedge core_clk) begin
        pop_wr_en_B_r           <= pop_wr_en_B;
        push_wr_en_A_r          <= push_wr_en_A;
        push_wr_en_B_r          <= push_wr_en_B;
        osq_empty_reset_valid_r <= osq_empty_reset_valid;
        osq_empty_reset_id_r    <= osq_empty_reset_id;
        i_qp_enable_valid_r     <= i_qp_enable_valid;
    end

    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            push_wr_en_A                 <= 1'b0;
            push_wr_id_A                 <= 'd0;
            push_wr_en_B                 <= 1'b0;
            push_wr_id_B                 <= 'd0;
            pushback_timeout_fatal_valid <= 1'b0;
            buffer_addr_B                <= 'd0;
            Qpid_buffer_wr_ptr           <= 'd0;
            Qpid_buffer_rd_ptr           <= 'd0;
            pushback_timeout_fatal_id    <= 'd0;
            push_pending                 <= 1'b0;
            push_id_pending              <= 'd0;
        end else begin
            if (osq_timed_out_set_valid) begin
                push_wr_en_A <= 1'b1;
                push_wr_id_A <= osq_timed_out_set_id;
            end else if (osq_empty_reset_valid_r && qp_under_fatal_or_recovery_rd_data) begin
                push_wr_en_A <= 1'b1;
                push_wr_id_A <= osq_empty_reset_id_r;
            end else if (push_pending) begin
                push_wr_en_A <= 1'b1;
                push_wr_id_A <= push_id_pending;
            end else begin
                push_wr_en_A <= 1'b0;
            end

            if (osq_timed_out_set_valid && (osq_empty_reset_valid_r && qp_under_fatal_or_recovery_rd_data) && osq_timed_out_set_id != osq_empty_reset_id_r) begin
                push_pending    <= 1'b1;
                push_id_pending <= osq_empty_reset_id_r;
            end else begin
                push_pending <= 1'b0;
            end

            if ((i_qp_fatal_valid && i_qp_fatal_data && !osq_empty_rd_data_3) && (osq_timed_out_set_valid || (osq_empty_reset_valid_r && qp_under_fatal_or_recovery_rd_data) || push_pending)) begin
                push_wr_en_B  <= 1'b1;
                push_wr_id_B  <= i_qp_fatal_id;
                buffer_addr_B <= Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH-1 : 0] + 1;
            end else if (i_qp_fatal_valid && i_qp_fatal_data && !osq_empty_rd_data_3) begin
                push_wr_en_B  <= 1'b1;
                push_wr_id_B  <= i_qp_fatal_id;
                buffer_addr_B <= Qpid_buffer_wr_ptr[C_QP_INDX_WIDTH-1 : 0];
            end else begin
                push_wr_en_B  <= 1'b0;
                buffer_addr_B <= Qpid_buffer_rd_ptr[C_QP_INDX_WIDTH-1 : 0];
            end

            if (i_qp_fatal_valid && i_qp_fatal_data && !osq_empty_rd_data_3) begin
                pop_wr_en_B <= 1'b0;
            end else if (!fifo_empty && !pop_wr_en_B && !pop_wr_en_B_r && !pushback_timeout_fatal_valid) begin
                pop_wr_en_B <= 1'b1;
            end else begin
                pop_wr_en_B <= 1'b0;
            end

            if (push_wr_en_A && push_wr_en_B) begin
                Qpid_buffer_wr_ptr <= Qpid_buffer_wr_ptr + 2;
            end else if (push_wr_en_A || push_wr_en_B) begin
                Qpid_buffer_wr_ptr <= Qpid_buffer_wr_ptr + 1;
            end

            if (pop_wr_en_B) begin
                Qpid_buffer_rd_ptr <= Qpid_buffer_rd_ptr + 1;
            end

            if (pop_wr_en_B_r) begin
                pushback_timeout_fatal_valid <= 1'b1;
                pushback_timeout_fatal_id    <= fatal_timeout_qpid;
            end else if (!i_acknack_valid && !QP1_acknack_valid) begin
                pushback_timeout_fatal_valid <= 1'b0;
            end
        end
    end

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_duplicate_push_protection_0 (
        .core_clk   (core_clk),
        .core_rst   (!core_rstn),
        .wr_en      (init_start || (i_qp_fatal_valid && i_qp_fatal_data)),
        .wr_addr    (init_start ? init_count : i_qp_fatal_id),
        .wr_data    (init_start ? 1'b0 : 1'b1),
        .wr_en_aux  (i_qp_enable_valid && !i_qp_enable_valid_r && !i_qp_enable_data),
        .wr_addr_aux(i_qp_enable_id),
        .wr_data_aux(1'b0),
        .rd_en      (1'b1),
        .rd_addr    (osq_empty_reset_id_r),
        .rd_data    (qp_under_fatal_or_recovery_rd_data)
    );

    always_ff @(posedge core_clk) begin
        i_qp_under_recovery_wr_en_r <= i_qp_under_recovery_wr_en;
    end

    assign push_is_avail_A  = 1'b1;
    assign push_is_avail_B  = 1'b1;

    assign o_qp_conf_qp_idx = osq_sampled_arbitrated_idx;


    //----------------------------------------------------------------------------------

    // ACK 管理器 FSM 实例 -----------------------------------------
    //
    lrdma_ack_mgr_fsm #(
        .C_NUM_QP          (C_NUM_QP),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_QP_INDX_WIDTH   (C_QP_INDX_WIDTH),
        .IP2BUS_LEN_WIDTH  (IP2BUS_LEN_WIDTH),
        .C_OSQ_PSN_WIDTH   (C_OSQ_PSN_WIDTH)
    ) inst_ack_mgr_fsm (
        .core_clk                    (core_clk),                    // Module Clock  
        .core_rstn                   (core_rstn),                   // Active low core reset
        .i_acks_to_process           (ack_nack_to_process),
        .o_arbitrate                 (arbitrate),
        .fsm_pushback_valid          (fsm_pushback_valid),
        .i_arbitration_done          (arbitration_done),
        .i_arbitrated_indx           (osq_arbitrated_idx),
        .o_sampled_arbitrated_indx   (osq_sampled_arbitrated_idx),
        .o_arbitrated_qp_for_intr    (arbitrated_idx_for_intr),
        .o_ack_mgr_sts               (ack_mgr_sts),
        .o_stat_retry_count          (o_stat_retry_count),
        .o_phase_to_update_post_retry(phase_to_update_post_retry),
        .i_cfg_retry_cnt_fatal_dis   (i_cfg_retry_cnt_fatal_dis),
        .i_qp_fatal_valid            (i_qp_fatal_valid),
        .i_qp_fatal_id               (i_qp_fatal_id),
        .i_qp_fatal_data             (i_qp_fatal_data),

        .i_dma_in_idle             (i_dma_in_idle),
        .i_en_osq_flush_on_qp_fatal(i_en_osq_flush_on_qp_fatal),

        .i_first_outgoing_pkt_valid        (first_outgoing_pkt_ff_valid),
        .i_first_outgoing_pkt_id           (first_outgoing_pkt_ff_id),
        .i_first_outgoing_pkt_data         (first_outgoing_pkt_ff_data),
        .i_first_outgoing_pkt_valid_pending(first_outgoing_pkt_ff_valid_pending),
        .i_first_outgoing_pkt_id_pending   (first_outgoing_pkt_ff_id_pending),
        .i_first_outgoing_pkt_data_pending (1'b0),
        .i_wqe_qpid                        (i_wqe_qpid),
        .i_wqe_send_start_psn              (i_wqe_send_start_psn[C_OSQ_PSN_WIDTH-1:0]),
        .i_wqe_push_req                    (i_wqe_push_req),

        .o_pop_osq              (pop_osq_from_fsm),
        .o_rd_osq               (rd_osq),
        .i_rd_osq_vld           (osq_rd_addr_rd_valid),
        .i_osq_empty            (osq_empty_rd_data_0),
        .i_phase_bit_acknack_psn(phase_bit_acknack_psn_ff_rd_data),
        .i_osq_entry_msn        (osq_msn),
        .i_osq_entry_end_psn    (osq_end_psn),
        .i_osq_entry_start_psn  (osq_start_psn),
        .i_osq_acknacked_psn    (acknacked_psn_for_arbitrated_qp),
        .i_osq_acknacked_msn    (acknacked_msn_for_arbitrated_qp),

        .o_clr_timeout_n_nack_set_valid(clr_timeout_n_nack_set_valid),
        .o_clr_timeout_n_nack_set_id   (clr_timeout_n_nack_set_id),
        .i_osq_nacked                  (osq_nacked_to_fsm),
        .i_osq_wqe_nack_resp           (i_osq_wqe_nack_resp_rd_data_1),
        .i_osq_opcode                  (osq_opcode),                     // BTH.Opcode of outbound request
        .i_osq_wrid                    (osq_wrid),
        .o_psn_to_retry                (psn_to_retry),
        .o_msn_to_retry                (msn_to_retry),

        .i_osq_wqe_retried   (osq_wqe_retried),
        .i_osq_wqe_fatal_push(osq_wqe_fatal_push),

        .acknacked_psn_rd_req     (acknacked_psn_rd_req_from_fsm),
        .acknacked_psn_rd_req_done(acknacked_psn_rd_req_done_to_fsm),

        .o_qp_cq_ba_req     (o_qp_cq_ba_req),
        .i_qp_cq_ba         (i_qp_cq_ba),           // CQ base address
        .i_qp_cq_ba_valid   (i_qp_cq_ba_valid),
        .o_qp_cq_depth_req  (o_qp_cq_depth_req),
        .i_qp_cq_depth_valid(i_qp_cq_depth_valid),  // CQ depth
        .i_qp_cq_depth      (i_qp_cq_depth),

        .o_qp_conf_req  (o_qp_conf_req),
        .i_qp_conf_valid(i_qp_conf_valid),
        .i_qp_conf      (i_qp_conf),

        .o_qp_cq_hdptr_req  (o_qp_cq_hdptr_req),
        .i_qp_cq_hdptr_valid(i_qp_cq_hdptr_valid),  // CQ head pointer
        .i_qp_cq_hdptr      (i_qp_cq_hdptr),
        .o_qp_cq_hdptr      (o_qp_cq_hdptr),
        .o_qp_cq_hdptr_wrn  (o_qp_cq_hdptr_wrn),
        .o_qp_cqe_proc_qpid (o_qp_cqe_proc_qpid),

        .i_intr_clr_wqe_cmpl(i_intr_clr_wqe_cmpl),
        .o_intr_wqe_cmpl_sts(intr_wqe_cmpl_sts),

        // QP 管理器重传接口
        .o_retransmit_qpid    (retransmit_qpid),       // 重传路径选中的 QP id
        .o_retransmit_reqd    (retransmit_reqd),       // Retransmission is required
        .i_retransmit_accepted(i_retransmit_accepted), // QP 管理器已接收重传请求

        .i_qp_curr_retry_cnt       (i_qp_curr_retry_cnt),
        .o_qp_curr_retry_cnt_rd_req(o_qp_curr_retry_cnt_rd_req),
        .o_qp_curr_retry_cnt_qpid  (o_qp_curr_retry_cnt_qpid),
        .i_qp_curr_retry_cnt_rd_rdy(i_qp_curr_retry_cnt_rd_rdy),

        .o_qp_curr_retry_cnt_wr_req(o_qp_curr_retry_cnt_wr_req),
        .i_qp_curr_retry_cnt_wr_rdy(i_qp_curr_retry_cnt_wr_rdy),
        //.o_qp_curr_retry_cnt_qpid (o_qp_curr_retry_cnt_qpid   ),
        .o_qp_curr_retry_cnt_wr    (o_qp_curr_retry_cnt),

        .o_qp_cfg_retry_cnt_rd_req(o_qp_cfg_retry_cnt_rd_req),
        .i_qp_cfg_retry_cnt_rd_rdy(i_qp_cfg_retry_cnt_rd_rdy),
        .i_qp_cfg_retry_cnt       (i_qp_cfg_retry_cnt),

        .o_qp_retry_fatal_req(o_qp_retry_fatal_req),
        .o_qp_retry_fatal_qpid(o_qp_retry_fatal_qpid),
        .i_qp_retry_fatal_rdy(i_qp_retry_fatal_rdy),
        .i_reload_retry_cnt_en({reload_retry_cnt_valid_pending_1, reload_retry_cnt_id_pending_1, reload_retry_cnt_data_pending_1, reload_retry_cnt_valid_pending_0, reload_retry_cnt_id_pending_0, reload_retry_cnt_valid, reload_retry_cnt_id, reload_retry_cnt_data}),
        .i_bus2ip_data_rdy(bus2ip_data_rdy),
        .i_lrdma_axi_master_done(lrdma_axi_master_done),
        .o_ip2bus_wen(ip2bus_wen),  // Write en to AXI master
        .o_ip2bus_len(ip2bus_len),
        .o_ip2bus_wraddr(ip2bus_wraddr),  // Write address
        .o_ip2bus_data(ip2bus_data),
        .o_retransmit_wqe_bram_din(o_retransmit_wqe_bram_din),
        .fsm_arbitrated_indx(fsm_arbitrated_indx),
        .fsm_i_arbitrated_indx(fsm_i_arbitrated_indx),
        .osq_timed_out_sticky_fsm_arbitrated_indx(osq_timed_out_sticky_fsm_arbitrated_indx),
        .init_start(init_start),
        .init_count(init_count),
        .osq_nacked_ff({osq_nacked_ff_valid_pending, osq_nacked_ff_id_pending, osq_nacked_ff_valid, osq_nacked_ff_id, osq_nacked_ff_data}),
        .i_qp_under_recovery_wr_en(i_qp_under_recovery_wr_en && !i_qp_under_recovery_wr_en_r),
        .i_qp_under_recovery_wr_idx(i_qp_under_recovery_wr_idx),
        .i_qp_under_recovery_wr_data(i_qp_under_recovery_wr_data)
    );

    //------------------------------------------------------------


    assign o_retransmit_reqd      = retransmit_reqd;
    assign o_osq_full_set_valid   = osq_full_set_valid;
    assign o_osq_full_set_id      = osq_full_set_id;
    assign o_osq_full_reset_valid = osq_full_reset_valid;
    assign o_osq_full_reset_id    = osq_full_reset_id;
    assign o_retransmit_qpid      = retransmit_qpid;
    assign o_ack_mgr_sts          = ack_mgr_sts;


    //----------------------------------------------------------------------------

    lrdma_ack_retry_timer #(
        .C_NUM_QP       (C_NUM_QP),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .C_TIMER_VAL_32_7us(8192),.C_TIMER_VAL_4_1us(1024)
    ) inst_ack_retry_timer (
        .core_clk                     (core_clk),                                                                                                                                                                                        // Module Clock  
        .core_rstn                    (core_rstn),                                                                                                                                                                                       // Active low core reset
        .o_q_timed_out_valid          (osq_timed_out_set_valid),
        .o_q_timed_out_wr_addr        (osq_timed_out_set_id),
        .osq_empty                    ({osq_empty_ff_valid_pending, osq_empty_ff_id_pending, osq_empty_ff_valid, osq_empty_ff_id, osq_empty_ff_data}),
        .timer_disabled_per_qp        ({timer_disabled_per_qp_valid, timer_disabled_per_qp_id, timer_disabled_per_qp_data, timer_disabled_per_qp_valid_pending, timer_disabled_per_qp_id_pending, timer_disabled_per_qp_data_pending}),
        .all_outstanding_resp_received({all_outstanding_resp_received_reset_valid, all_outstanding_resp_received_reset_id, all_outstanding_resp_received_set_valid, all_outstanding_resp_received_set_id}),
        .i_qp_fatal                   ({i_qp_fatal_valid, i_qp_fatal_id, i_qp_fatal_data}),
        .i_reload_timer_wqe           (timer_reload_for_wqe_for_i_wqe_qpid),
        .i_reload_timer_wqe_qpid      (i_wqe_qpid),

        .i_reload_timer_acknack     (i_acknack_valid),
        .i_reload_timer_acknack_qpid(i_qp_index_acknack),

        .i_reload_value_wqe         (i_timer_loadval_wqe),                                        //re-load value when 1st WQE goes out
        .i_reload_value_acknack     (i_timer_loadval_acknack),                                    // 收到 ACK/NAK 以及相关事件时使用的定时器重装载值
        .init_start                 (init_start),
        .init_count                 (init_count),
        .osq_nacked_timer_mem_rdaddr(osq_nacked_timer_mem_rdaddr),
        .o_timer_mem_rdaddr         (timer_mem_rdaddr),
        .osq_nacked_ff_rd_data      (osq_nacked_ff_rd_data_comp),
        .i_qp_under_recovery_wr_en  (i_qp_under_recovery_wr_en && !i_qp_under_recovery_wr_en_r),
        .i_qp_under_recovery_wr_idx (i_qp_under_recovery_wr_idx),
        .i_qp_under_recovery_wr_data(i_qp_under_recovery_wr_data)

    );

    assign o_osq_timeout_set_valid = osq_timed_out_set_valid;
    assign o_osq_timeout_set_id    = osq_timed_out_set_id;

    //###############################################################################################################################################
    // timer_disabled_per_qp 同拍只能优先写 ACK/NAK；若 ACK/NAK 与 WQE push 指向不同 QP，WQE push 侧通过 pending 补写。
    //###############################################################################################################################################
    always_ff @(posedge core_clk) begin : timer_disabled_per_qp
        if (~core_rstn) begin
            timer_disabled_per_qp_valid         <= 1'b0;
            timer_disabled_per_qp_id            <= {C_QP_INDX_WIDTH{1'b0}};
            timer_disabled_per_qp_data          <= 1'b0;
            timer_disabled_per_qp_valid_pending <= 1'b0;
            timer_disabled_per_qp_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
            timer_disabled_per_qp_data_pending  <= 1'b0;
        end else begin
            if (i_acknack_valid) begin
                timer_disabled_per_qp_valid <= 1'b1;
                timer_disabled_per_qp_id    <= i_qp_index_acknack;
                timer_disabled_per_qp_data  <= ~(|i_timer_loadval_acknack);
            end else if (i_wqe_push_req) begin
                timer_disabled_per_qp_valid <= 1'b1;
                timer_disabled_per_qp_id    <= i_wqe_qpid;
                timer_disabled_per_qp_data  <= ~(|i_timer_loadval_wqe);
            end else if (timer_disabled_per_qp_valid_pending) begin
                timer_disabled_per_qp_valid <= 1'b1;
                timer_disabled_per_qp_id    <= timer_disabled_per_qp_id_pending;
                timer_disabled_per_qp_data  <= timer_disabled_per_qp_data_pending;
            end else begin
                timer_disabled_per_qp_valid <= 1'b0;
            end

            if (i_acknack_valid && i_wqe_push_req && i_qp_index_acknack != i_wqe_qpid) begin
                timer_disabled_per_qp_valid_pending <= 1'b1;
                timer_disabled_per_qp_id_pending    <= i_wqe_qpid;
                timer_disabled_per_qp_data_pending  <= ~(|i_timer_loadval_wqe);
            end else begin
                timer_disabled_per_qp_valid_pending <= 1'b0;
            end
        end
    end

    //----------------------------------------------------------------------------
    // AXI master 用于把 CQE 写入 DDR；doorbell 写回路径无写请求。
    lrdma_axi_master #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_DATA_WIDTH(512),
        .C_M_AXI_THREAD_ID_WIDTH(1),
        .IP2BUS_LEN_WIDTH(IP2BUS_LEN_WIDTH)
    ) inst_ack_mgr_axi_master (
        // AXI System signals
        .m_axi_aclk   (core_clk),
        .m_axi_aresetn(core_rstn),
        // AXI Master signals
        .m_axi_awid   (m_axi_awid),
        .m_axi_awaddr (m_axi_awaddr),
        .m_axi_awlen  (m_axi_awlen),
        .m_axi_awsize (m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_awprot (m_axi_awprot),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata  (m_axi_wdata),
        .m_axi_wstrb  (m_axi_wstrb),
        .m_axi_wlast  (m_axi_wlast),
        .m_axi_wvalid (m_axi_wvalid),
        .m_axi_wready (m_axi_wready),
        .m_axi_awlock (m_axi_awlock),
        .m_axi_bid    (m_axi_bid),
        .m_axi_bresp  (m_axi_bresp),
        .m_axi_bvalid (m_axi_bvalid),
        .m_axi_bready (m_axi_bready),
        .m_axi_arid   (m_axi_arid),
        .m_axi_araddr (m_axi_araddr),
        .m_axi_arlen  (m_axi_arlen),
        .m_axi_arsize (m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arcache(m_axi_arcache),
        .m_axi_arprot (m_axi_arprot),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid    (m_axi_rid),
        .m_axi_rdata  (m_axi_rdata),
        .m_axi_rresp  (m_axi_rresp),
        .m_axi_rlast  (m_axi_rlast),
        .m_axi_rvalid (m_axi_rvalid),
        .m_axi_rready (m_axi_rready),
        .m_axi_arlock (m_axi_arlock),

        // Master attachment to IP core side interface. 
        .bus2ip_byte_en (),
        .bus2ip_data    (),
        .bus2ip_dvalid  (),                 // Only used for writes
        .ip2bus_data    (ip2bus_data),
        .bus2ip_data_rdy(bus2ip_data_rdy),
        .axi_m_en       (ip2bus_wen),
        .wr_rdn         (1'b1),             //1 = write; 0 = read
        .ip2bus_addr    (ip2bus_wraddr),
        .ip2bus_len     (ip2bus_len),       //length in bytes

        // Status indicators to the core IP 
        .lrdma_axi_master_done  (lrdma_axi_master_done),
        .lrdma_axi_master_busy  (),
        .lrdma_axi_master_bvalid(),
        .lrdma_axi_master_error ()

    );

endmodule
