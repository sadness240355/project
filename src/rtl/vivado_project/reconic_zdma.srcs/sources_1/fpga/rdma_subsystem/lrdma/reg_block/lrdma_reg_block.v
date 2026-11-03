`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_reg_block — LRDMA 配置/状态寄存器集合模块
//
// 【模块概述】
// 实现完整的 AXI4-Lite 从接口，供上位机软件配置和监控 LRDMA 。
// 管理两大类寄存器：全局寄存器和 Per-QP 寄存器。
//
// 【全局寄存器】（地址空间 0x10_0000 区域）
// - LRDMACONF（使能）、LRDMAADCONF（高级配置：retry_cnt_fatal_dis 等）
// - MAC 地址、IPv4/IPv6 地址
// - 数据缓冲区基地址/大小、错误缓冲区配置、响应错误缓冲区配置
// - QP 使能数量、中断使能/状态（INTEN/INTSTS，4 种中断源）
// - PFC 配置（暂停优先级、XON/XOFF 阈值）、ACK 生成策略、流信用
//
// 【Per-QP 寄存器】（地址空间 0x18_0000 区域，每 QP 固定偏移）
// - QPCONF（QP 使能位、RC/UD 类型、中断使能等）
// - QPADVCONF（DSCP、TTL、PKEY）
// - RQBA/SQBA/CQBA（RQ/SQ/CQ 基地址）、QDEPTH（队列深度）
// - SQPSN（SQ PSN）、SQPI（SQ 生产者索引门铃）
// - CQHEAD（CQ 头指针）、RQCI（RQ 消费者索引）
// - TIMEOUT 配置、目标 QP ID、远端 MAC/IP 地址等数十种配置
//
// 【存储架构】
// 每种 Per-QP 配置使用独立的双端口 BRAM（simple dual-port）：
// - A 端口：AXI-Lite 读写（"慢路径"，供 CPU 访问）
// - B 端口：内部模块实时读写（"快路径"，供硬件逻辑访问）
//
// 【AXI-Lite 状态机】
// - wr_req_r / rd_req_r 锁存写/读请求
// - 地址解码区分全局寄存器区（0x10_xxxx）和 Per-QP 区（0x18_xxxx）
//
// 【附加功能】
// - 中断管理：4 种中断源独立使能/清除，按位寄存器支持最多 256 QP 独立中断跟踪
// - QP 配置写入时触发 fatal 清除和 recovery 通知（qp_conf_bram_wen 上升沿）
// - SQ 空状态聚合：sq_empty_ff_and_reduce 合并所有 QP 的 SQ PI vs 消费指针比较结果
// - 状态计数器：发送/接收包数、WQE 计数、全局计数使能/清除
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_reg_block #(
    parameter C_S_AXI_LITE_ADDR_WIDTH = 18,
    parameter C_S_AXI_LITE_DATA_WIDTH = 32,
    parameter C_M_AXI_ADDR_WIDTH      = 32,
    parameter C_NUM_QP                = 256,
    parameter C_QP_INDX_WIDTH         = 8,
    parameter STB_WIDTH               = 4
) (
    input s_axi_lite_aclk,
    input s_axi_lite_aresetn,

    input      [C_S_AXI_LITE_ADDR_WIDTH-1:0] s_axi_lite_awaddr,
    output                                   s_axi_lite_awready,
    input                                    s_axi_lite_awvalid,
    input      [C_S_AXI_LITE_ADDR_WIDTH-1:0] s_axi_lite_araddr,
    output                                   s_axi_lite_arready,
    input                                    s_axi_lite_arvalid,
    input      [C_S_AXI_LITE_DATA_WIDTH-1:0] s_axi_lite_wdata,
    input      [              STB_WIDTH-1:0] s_axi_lite_wstrb,
    output                                   s_axi_lite_wready,
    input                                    s_axi_lite_wvalid,
    output reg [C_S_AXI_LITE_DATA_WIDTH-1:0] s_axi_lite_rdata,
    output     [                        1:0] s_axi_lite_rresp,
    input                                    s_axi_lite_rready,
    output reg                               s_axi_lite_rvalid,
    output     [                        1:0] s_axi_lite_bresp,
    input                                    s_axi_lite_bready,
    output reg                               s_axi_lite_bvalid,

    input core_clk,  // Module Clock  
    input core_rstn, // Active high core reset

    input                       i_waiting_for_rq_update,
    input [C_QP_INDX_WIDTH-1:0] i_waiting_for_rq_update_on_q_num,

    // 来自/前往头部验证模块的中断信号
    input i_pkt_valdn_err_intr,
    input i_mad_pkt_rcvd_intr,

    // Interrupt Status 寄存器
    output o_intr_clr_pkt_valdn_err,  // INTSTS[0]: 传入数据包验证错误中断
    output o_intr_clr_mad_pkt_rcvd,   // INTSTS[1]: 收到传入的 MAD 数据包中断
    output o_intr_clr_wqe_cmpl,       // INTSTS[2]: WQE 完成中断（对 QPCONF[3] 位被设置的 QP 断言）
    output o_intr_clr_fatal_err,      // qp致命错误中断，只要有一个QP发生致命错误就会触发（）

    // Interrupt Enable 寄存器
    output o_intr_en_pkt_valdn_err,  // INTEN[0]: 传入数据包验证错误中断启用
    output o_intr_en_mad_pkt_rcvd,   // INTEN[1]: 接收到 MAD 数据包中断启用
    output o_intr_en_wqe_cmpl,       // INTEN[2]: WQE 完成中断启用
    output o_intr_en_fatal_err,      // INTEN[3]: 收到致命错误中断启用

    input                             i_qp_fatal_err_valid,
    input      [C_QP_INDX_WIDTH -1:0] i_qp_fatal_err_id,
    input                             i_qp_fatal_err_data,
    output reg                        o_qp_clr_fatal_err_valid,
    output reg [C_QP_INDX_WIDTH -1:0] o_qp_clr_fatal_err_id,
    output reg                        o_qp_clr_fatal_err_data,
    output reg                        o_qp_under_recovery_valid,
    output reg [C_QP_INDX_WIDTH -1:0] o_qp_under_recovery_id,
    output reg                        o_qp_under_recovery_data,

    input                         i_qp_wq_cmpl_intr_or_reduce,
    input  [                31:0] i_qp_wq_cmpl_intr_wr_en,
    input  [                31:0] i_qp_wq_cmpl_intr_wr_data,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_wq_cmpl_intr_wr_addr,
    input  [                31:0] i_qp_wq_cmpl_intr_wr_en_pending,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_wq_cmpl_intr_wr_addr_pending,
    output [                31:0] o_cq_intr_sts_clr_wr_en,            // Completion Q interrupt status (bitwise)寄存器
    output [ C_QP_INDX_WIDTH-1:0] o_cq_intr_sts_clr_wr_addr,          // Completion Q interrupt status (bitwise) address 寄存器(addr为0, 表示QP0-31, addr为1, 表示QP32-63, 以此类推)

    input                         i_rq_full_valid,
    input [C_QP_INDX_WIDTH-1 : 0] i_rq_full_idx,
    input                         i_rq_full_data,
    input                         i_rq_full_reset_valid,
    input [C_QP_INDX_WIDTH-1 : 0] i_rq_full_reset_idx,
    input                         i_rq_full_reset_data,
    input                         i_osq_full_set_valid,      // Outstanding Q full for QPn
    input [C_QP_INDX_WIDTH-1 : 0] i_osq_full_set_id,
    input                         i_osq_full_reset_valid,
    input [C_QP_INDX_WIDTH-1 : 0] i_osq_full_reset_id,
    input                         i_osq_empty_set_valid,     // Outstanding Q empty for QPn
    input [C_QP_INDX_WIDTH-1 : 0] i_osq_empty_set_id,
    input                         i_osq_empty_reset_valid,
    input [C_QP_INDX_WIDTH-1 : 0] i_osq_empty_reset_id,
    input                         i_qp_retried_set_valid,    // QP retried for QPn
    input [C_QP_INDX_WIDTH-1 : 0] i_qp_retried_set_id,
    input                         i_qp_retried_reset_valid,
    input [C_QP_INDX_WIDTH-1 : 0] i_qp_retried_reset_id,

    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_ssn_idx,   // STATSSNi 的 QP 索引
    input  [                23:0] i_qp_stat_ssn,       // 写入的发送侧 SSN
    output [                23:0] o_qp_stat_ssn,       // STATSSNi[23:0]：发送侧 SSN
    input                         i_qp_stat_ssn_req,
    input                         i_qp_stat_ssn_wrn,
    output                        o_qp_stat_ssn_valid,

    input  [C_QP_INDX_WIDTH -1:0] i_qp_timeout_idx,  // TIMEOUTCONFi 的 QP 索引
    output [                31:0] o_qp_timeout,      // TIMEOUTCONFi：超时与最大重试配置

    input  [C_QP_INDX_WIDTH -1:0] i_qp_rq_pi_db_idx,   // STATRQPIDBi 的 QP 索引
    input  [                15:0] i_qp_rq_pi_db,
    output [                15:0] o_qp_rq_pi_db,       // Status RQ Producer index DB QPi (STATRQPIDBi)寄存器
    input                         i_qp_rq_pi_db_req,
    input                         i_qp_rq_pi_db_wrn,
    output                        o_qp_rq_pi_db_valid,

    input  [C_QP_INDX_WIDTH -1:0] i_qp_ret_sq_psn_idx,       // STATRET_SQ_PSN 的 QP 索引
    input  [                23:0] i_qp_ret_sq_psn,
    output [                23:0] o_qp_ret_sq_psn,
    input                         i_qp_ret_sq_psn_req,
    input                         i_qp_ret_sq_psn_wrn,
    output                        o_qp_ret_sq_psn_valid,
    output                        o_qp_ret_sq_psn_valid_int,

    // 入站信息统计
    input [15:0] i_inc_ack_pkt_cnt,
    input [15:0] i_inc_mad_pkt_cnt,
    input [15:0] i_inc_inv_pkt_cnt,
    input [31:0] i_inc_all_dropped_cnt,
    input [15:0] i_inc_nack_cnt,

    // 出站信息统计
    input [15:0] i_out_wr_pkt_cnt,
    input [15:0] i_out_mad_pkt_cnt,
    input [31:0] i_ack_mgr_sts,
    input [31:0] i_stat_retry_cnt,

    // TX 引擎状态
    input [31:0] i_tx_engine_sts,
    input [31:0] i_qp_mgr_sts,

    // SQ（发送队列）仲裁的满/空状态
    output                          o_sq_empty_valid,
    output [C_QP_INDX_WIDTH -1 : 0] o_sq_empty_id,
    output                          o_sq_empty_data,   // 某个 QP 的 SQ（发送队列）是否为空

    // 输出到其它模块的通用配置
    output                       o_lrdma_en,                            // LRDMA Configuration寄存器 -- [0] LRDMA Enable
    output                       o_lrdma_adv_conf_sw_override,          // LRDMA Advance Configuration寄存器 -- [0] SW override enable
    output [C_QP_INDX_WIDTH-1:0] o_lrdma_adv_conf_sw_override_qpid,     // LRDMA Advance Configuration寄存器 -- [30:20] Software Override QP Number
    output                       o_lrdma_adv_conf_retry_cnt_fatal_dis,  // LRDMA Advance Configuration寄存器 -- [2] retry_cnt_fatal_dis
    input  [  C_QP_INDX_WIDTH:0] i_sw_override_per_qp_en,               // {o_lrdma_adv_conf_sw_override, o_lrdma_adv_conf_sw_override_qpid}
    output                       o_err_buf_en,                          // LRDMA Configuration寄存器 -- [5] Error buffer enable
    output [                1:0] o_flow_credits,                        // LRDMA Configuration寄存器 -- [7:6] 无功能定义
    output [               15:0] o_lrdma_udp_src_port,                  // LRDMA Configuration寄存器 -- [23:8] UDP source port for outgoing packets
    output [C_QP_INDX_WIDTH:0] o_num_qp_en,                           // 启用的 QP 数量 -- LRDMA_CONF_QP_EN寄存器 -- [10:0] Number of QPs enabled
    output [               47:0] o_mac_lrdma_addr,                      // 本地 LRDMA MAC 地址 -- MAC LRDMA Address寄存器
    output [               31:0] o_ipv4_lrdma_addr,                     // 本地 LRDMA IPv4 地址-- IPv4 LRDMA Address寄存器
    output [              127:0] o_ipv6_lrdma_addr,                     // 本地 LRDMA IPv6 地址 -- IPv6 Address寄存器

    // Error Buffer: IP 会根据未通过验证的数据包更新这些缓冲区。
    //               通过向 LRDMACONF[5]写 1，可启用向该缓冲区写入所有验证错误信息。
    //               如果禁用该位，则只有导致 QP 进入 FATAL 状态的数据包才会写入错误缓冲区。
    output [C_M_AXI_ADDR_WIDTH-1:0] o_rq_err_pkt_buf_ba,           // Error Buffer Base Address寄存器
    output [                  15:0] o_rq_err_pkt_buf_sz_num_bufs,  // Error Buffer Size寄存器 -- [15:0] Number of Error buffers
    output [                  15:0] o_rq_err_pkt_buf_sz_buf_sz,    // Error Buffer Size寄存器 -- [31:16] Size of each Error buffer，单位为字节
    input  [                  15:0] i_rq_err_pkt_buf_wrptr,

    output reg                        qp_enable_valid,
    output reg [C_QP_INDX_WIDTH -1:0] qp_enable_id,
    output reg                        qp_enable_data,

    output reg                          o_qp_disable_pulse_valid,
    output reg [C_QP_INDX_WIDTH -1 : 0] o_qp_disable_pulse_id,
    output reg                          o_qp_disable_pulse_data,

    output [C_M_AXI_ADDR_WIDTH-1:0] o_in_errsts_q_ba,    // Incoming Pkt Error Status Queue Base address寄存器
    output [                  15:0] o_in_errsts_q_sz,    // Incoming Pkt Error Status Queue Size寄存器 -- [15:0] Number of incoming error pkt status queue entries
    input  [                  15:0] i_in_errsts_q_wrptr,

    //Configuration outputs going to other modules (PER QP)
    input                         i_qp_conf_req_ack_mgr,
    input                         i_qp_conf_req_tx_engine,
    input                         i_qp_conf_req_rx_engine,
    output                        o_qp_conf_valid_ack_mgr,
    output                        o_qp_conf_valid_tx_engine,
    output                        o_qp_conf_valid_rx_engine,
    output [                31:0] o_qp_conf,                  // QP Configuration QPi(QPCONFi) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_conf_idx,              // 读取 QPCONFi 的 QP 索引
    output                        o_qp_cfg_retry_cnt_rd_rdy,  // TIMEOUTCONFi 重试次数配置读响应有效
    input                         i_qp_cfg_retry_cnt_rd_req,  // ACK 管理器读取 TIMEOUTCONFi 重试次数配置的请求

    output [                31:0] o_qp_conf_replica,       // QP configuration QPi(QPCONFi) REPLICA -- QP 配置 REPLICA
    input  [C_QP_INDX_WIDTH -1:0] i_qp_conf_replica_idx,   // 读取 QPCONFi 复制寄存器的 QP 索引
    input                         i_qp_conf_replica_req,
    output                        o_qp_conf_replica_valid,

    input                         i_qp_adv_conf_req,
    output                        o_qp_adv_conf_valid,
    output [                31:0] o_qp_adv_conf,        // QP advanced configuration QPi (QPADVCONFi) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_adv_conf_idx,    // 读取 QPADVCONFi 的 QP 索引

    input                           i_qp_rq_ba_req,
    output                          o_qp_rq_ba_valid,
    output [C_M_AXI_ADDR_WIDTH-9:0] o_qp_rq_ba,        // RCV Q Buffer base address QPi (RQBAi) 寄存器
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_rq_ba_idx,    // 读取 RQ 缓冲区基地址的 QP 索引

    input                           i_qp_sq_ba_req_int,
    output                          o_qp_sq_ba_valid_int,
    output [C_M_AXI_ADDR_WIDTH-1:0] o_qp_sq_ba,            // SQ（发送队列）基地址 QPi (SQBAi) 寄存器
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_sq_ba_idx,        // 读取 SQ 基地址的 QP 索引

    input                           i_qp_cq_ba_req,
    output                          o_qp_cq_ba_valid,
    output [C_M_AXI_ADDR_WIDTH-1:0] o_qp_cq_ba,        // CQ base address QPi (CQBAi) 寄存器
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_cq_ba_idx,    // 读取 CQ 基地址的 QP 索引

    input                           i_qp_rq_wrptrdb_add_req,
    output                          o_qp_rq_wrptrdb_add_valid,
    output [C_M_AXI_ADDR_WIDTH-1:0] o_qp_rq_wrptrdb_add,        // RCV Q Write pointer DB address QPi (RQWPTRDBADDi) 寄存器
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_rq_wrptrdb_add_idx,    // 读取 RQ 写门铃地址的 QP 索引

    input                         i_qp_cq_hdptr_req,
    input                         i_qp_cq_hdptr_retry_req,
    output                        o_qp_cq_hdptr_valid,
    output                        o_qp_cq_hdptr_valid_retry,
    output [                15:0] o_qp_cq_hdptr,              // CQ head pointer QPi (CQHEADi) 寄存器
    input  [                15:0] i_qp_cq_hdptr,
    input                         i_qp_cq_hdptr_wrn,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_cq_hdptr_idx,          // 读取 CQ head pointer 的 QP 索引

    input                         i_qp_rq_cidb_req,
    output                        o_qp_rq_cidb_valid,
    output [                31:0] o_qp_rq_cidb,        // RQ Consumer Index QPi (RQCIi) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_rq_cidb_idx,    // 读取 RQ CI doorbell 的 QP 索引

    output [                15:0] o_qp_sq_pidb,       // SQ Producer index QPi (SQPIi) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_sq_pidb_idx,   // 读取 SQ PI doorbell 的 QP 索引
    input                         i_qp_sq_pidb_req,
    output                        o_qp_sq_pidb_valid,
    //-------------------------------  

    input                         i_qp_rq_depth_req,
    output                        o_qp_rq_depth_valid,
    output [                15:0] o_qp_rq_depth,        // Q Depth QPi (QDEPTHi) 寄存器 -- [31:16]: Receive Q depth
    input  [C_QP_INDX_WIDTH -1:0] i_qp_rq_depth_idx,    // 读取 RQ 深度的 QP 索引

    input                         i_qp_sq_depth_req,
    output                        o_qp_sq_depth_valid,
    output [                15:0] o_qp_sq_depth,        // Q Depth QPi (QDEPTHi) 寄存器 -- [15:0]: SQ 深度，CQ 使用相同深度
    input  [C_QP_INDX_WIDTH -1:0] i_qp_sq_depth_idx,    // 读取 SQ 深度的 QP 索引

    // 提供给 ACK 管理器的 CQ 深度信息。 
    // CQ 深度等于 SQ 深度，该信息由 QP cache 路径共享。
    // QP 管理器内部更新与 ACK 管理器更新共享该路径，cache manager 优先。
    input                         i_qp_cq_depth_req,
    output                        o_qp_cq_depth_valid,
    output [                15:0] o_qp_cq_depth,        // Q Depth QPi (QDEPTHi) 寄存器 -- [15:0]: CQ 深度，与 SQ 深度一致
    input  [C_QP_INDX_WIDTH -1:0] i_qp_cq_depth_idx,    // QDEPTHi/CQ depth 读取的 QP 索引

    input                             i_qp_sq_psn_req,          // RX 路径的请求
    input                             i_qp_sq_psn_wqe_req,      // SQ WQE 路径的请求
    input                             i_qp_sq_psn_retry_req,
    output                            o_qp_sq_psn_valid,
    output                            o_qp_sq_psn_wqe_valid,
    output reg                        o_qp_sq_psn_resp_valid,
    output                            o_qp_sq_psn_retry_valid,
    output     [                23:0] o_qp_sq_psn,              // SQ PSN for QPi (SQPSNi) 寄存器
    input                             i_qp_sq_psn_wrn,
    input                             i_qp_sq_psn_retry_wrn,
    input      [                23:0] i_qp_sq_psn,
    input      [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_idx,          // RX response 路径访问 SQPSNi 的 QP 索引
    input      [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_wqe_idx,      // TX WQE 路径访问 SQPSNi 的 QP 索引
    input      [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_retry_idx,    // 重传路径访问 SQPSNi 的 QP 索引
    input      [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_resp_idx,     // ACK 管理器响应完成检查访问 SQPSNi 的 QP 索引

    output [                31:0] o_qp_last_req,       // Last req for QPi (LSTREQi) 寄存器
    input  [                31:0] i_qp_last_req,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_last_req_idx,   // LSTREQi 的 QP 索引
    input                         i_qp_last_req_req,
    input                         i_qp_last_req_wrn,
    output                        o_qp_last_req_valid,

    input                         i_qp_dest_qpid_req_tx_engine,
    input                         i_qp_dest_qpid_req_rx_engine,
    output                        o_qp_dest_qpid_valid_tx_engine,
    output                        o_qp_dest_qpid_valid_rx_engine,
    output [                23:0] o_qp_dest_qpid,                  // Destination QP configuration for QPi (DESTQPCONFi) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_dest_qpid_idx,              // DESTQPCONFi 的 QP 索引

    input                         i_qp_mac_remote_addrl_req,
    input                         i_qp_mac_remote_addrl_wqe_req,
    output                        o_qp_mac_remote_addrl_valid,
    output                        o_qp_mac_remote_addrl_wqe_valid,
    output [                31:0] o_qp_mac_remote_addrl,            // MAC destination address LSB QPi (MACDESADDLSBi) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_idx,        // MACDESADDLSBi 读取的 QP 索引
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_wqe_idx,    // TX WQE 路径读取 MACDESADDLSBi 的 QP 索引

    input                         i_qp_mac_remote_addrl_replica_req,
    output [                31:0] o_qp_mac_remote_addrl_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_replica_idx,   // MACDESADDLSBi replica 读取的 QP 索引
    output                        o_qp_mac_remote_addrl_replica_valid,

    input                         i_qp_mac_remote_addrm_req,
    input                         i_qp_mac_remote_addrm_wqe_req,
    output                        o_qp_mac_remote_addrm_valid,
    output                        o_qp_mac_remote_addrm_wqe_valid,
    output [                31:0] o_qp_mac_remote_addrm,            // MAC destination address MSB QPi (MACDESADDMSBi) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_idx,        // MACDESADDMSBi 读取的 QP 索引
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_wqe_idx,    // TX WQE 路径读取 MACDESADDMSBi 的 QP 索引

    output [                31:0] o_qp_mac_remote_addrm_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_replica_idx,   // MACDESADDMSBi replica 读取的 QP 索引
    input                         i_qp_mac_remote_addrm_replica_req,
    output                        o_qp_mac_remote_addrm_replica_valid,

    input                         i_qp_ip_remote_addr1_req,
    output                        o_qp_ip_remote_addr1_valid,
    input                         i_qp_ip_remote_addr1_wqe_req,
    output                        o_qp_ip_remote_addr1_wqe_valid,
    output [                31:0] o_qp_ip_remote_addr1,            // IP destination address 1 QPi (IPDESADDR1i) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_idx,        // IPDESADDR1i 读取的 QP 索引
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_wqe_idx,    // TX WQE 路径读取 IPDESADDR1i 的 QP 索引

    output [                31:0] o_qp_ip_remote_addr1_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_replica_idx,   // IPDESADDR1i replica 读取的 QP 索引
    input                         i_qp_ip_remote_addr1_replica_req,
    output                        o_qp_ip_remote_addr1_replica_valid,

    input                         i_qp_ip_remote_addr2_req,
    input                         i_qp_ip_remote_addr2_wqe_req,
    output                        o_qp_ip_remote_addr2_valid,
    output                        o_qp_ip_remote_addr2_wqe_valid,
    output [                31:0] o_qp_ip_remote_addr2,            // IP destination address 2 QPi (IPDESADDR2i) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_idx,        // IPDESADDR2i 读取的 QP 索引
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_wqe_idx,    // TX WQE 路径读取 IPDESADDR2i 的 QP 索引

    output [                31:0] o_qp_ip_remote_addr2_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_replica_idx,   // IPDESADDR2i replica 读取的 QP 索引
    input                         i_qp_ip_remote_addr2_replica_req,
    output                        o_qp_ip_remote_addr2_replica_valid,

    input                         i_qp_ip_remote_addr3_req,
    output                        o_qp_ip_remote_addr3_valid,
    input                         i_qp_ip_remote_addr3_wqe_req,
    output                        o_qp_ip_remote_addr3_wqe_valid,
    output [                31:0] o_qp_ip_remote_addr3,            // IP destination address 3 QPi (IPDESADDR3i) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_idx,        // IPDESADDR3i 读取的 QP 索引
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_wqe_idx,    // TX WQE 路径读取 IPDESADDR3i 的 QP 索引

    input                         i_qp_ip_remote_addr3_replica_req,
    output                        o_qp_ip_remote_addr3_replica_valid,
    output [                31:0] o_qp_ip_remote_addr3_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_replica_idx,    // IPDESADDR3i replica 读取的 QP 索引

    input                         i_qp_ip_remote_addr4_req,
    output                        o_qp_ip_remote_addr4_valid,
    input                         i_qp_ip_remote_addr4_wqe_req,
    output                        o_qp_ip_remote_addr4_wqe_valid,
    output [                31:0] o_qp_ip_remote_addr4,            // IP destination address 4 QPi (IPDESADDR4i) 寄存器
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_idx,        // IPDESADDR4i 读取的 QP 索引
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_wqe_idx,    // TX WQE 路径读取 IPDESADDR4i 的 QP 索引

    input                         i_qp_ip_remote_addr4_replica_req,
    output                        o_qp_ip_remote_addr4_replica_valid,
    output [                31:0] o_qp_ip_remote_addr4_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_replica_idx,    // IPDESADDR4i replica 读取的 QP 索引

    input  [                15:0] i_qp_curr_sqptr_proc,
    input                         i_qp_curr_sqptr_proc_wen,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_curr_sqptr_proc_idx,    // 写回 SQ pointer 的 QP 索引
    output                        o_qp_curr_sqptr_proc_valid,
    input                         i_qp_curr_sqptr_rdreq,
    output [                15:0] o_qp_curr_sqptr_proc,        // Status Current SQ ptr under process (STATCURSQPTRi) 寄存器

    // Response PSN status register. Read interface to rx engine
    input  [                23:0] i_qp_stat_resp_psn,
    input                         i_qp_stat_resp_psn_wen,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_resp_psn_idx,
    output                        o_qp_stat_resp_psn_valid,
    input                         i_qp_stat_resp_psn_rdreq,
    output [                23:0] o_qp_stat_resp_psn,

    // Status NAK details from RX Engine
    // 来自 RX 包处理程序的状态 NAK 详情
    input  [                 6:0] i_qp_stat_nak,
    input                         i_qp_stat_nak_wen,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_nak_idx,
    output                        o_qp_stat_nak_valid,
    input                         i_qp_stat_nak_rdreq,
    output [                 6:0] o_qp_stat_nak,

    // 来自 ACK 管理器的 NAK 状态信息
    // 来自 ACK 管理器的 NAK 状态信息
    input  [                 2:0] i_qp_stat_retry_cnt,
    input                         i_qp_stat_retry_cnt_wen,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_retry_cnt_idx,
    output                        o_qp_stat_retry_cnt_valid,
    input                         i_qp_stat_retry_cnt_rdreq,
    output [                 2:0] o_qp_stat_retry_cnt,

    // RQ 缓冲区写入地址
    // RQ 缓冲区写入地址
    input  [C_M_AXI_ADDR_WIDTH-9:0] i_qp_stat_rq_buf_ca,
    input                           i_qp_stat_rq_buf_ca_wen,
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_stat_rq_buf_ca_idx,
    output                          o_qp_stat_rq_buf_ca_valid,
    input                           i_qp_stat_rq_buf_ca_rdreq,
    output [C_M_AXI_ADDR_WIDTH-9:0] o_qp_stat_rq_buf_ca,

    // WQE push status interface
    input  [                15:0] i_qp_stat_wqe_cnt,
    input                         i_qp_stat_wqe_cnt_wen,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_wqe_cnt_idx,
    output                        o_qp_stat_wqe_cnt_valid,
    input                         i_qp_stat_wqe_cnt_rdreq,
    output [                15:0] o_qp_stat_wqe_cnt,

    // PFC 配置
    output [ 3:0] roce_priority,            // LRDMA_PAUSE_CONF -- [7:4] Pause priority for RoCE (Binary encoding)
    output [ 3:0] non_roce_priority,        // LRDMA_PAUSE_CONF -- [11:8] Pause priority for non-RoCE (Binary encoding)
    output        disable_priority_chk,     // LRDMA_PAUSE_CONF -- [13] Disable priority check (applicable only for TX data path)
    output        pfc_en_roce,              // LRDMA_PAUSE_CONF -- [0] Pause enable RoCE
    output        pfc_en_non_roce,          // LRDMA_PAUSE_CONF -- [1] Pause enable non-RoCE
    output [15:0] roce_pause_on_threshold,  // LRDMA_BUF_THRESHOLD_ROCE -- [15:0] XON Value for RoCE
    output [15:0] roce_pause_off_threshold, // LRDMA_BUF_THRESHOLD_ROCE -- [31:16] XOFF Value for RoCE

    input                         init_start,
    input [C_QP_INDX_WIDTH-1 : 0] init_count,

    // 状态观测寄存器
    input  [ 7:0] i_axi_rd_err_cnt,
    input  [31:0] i_last_in_pkt_info,
    input  [31:0] i_last_out_pkt_info,
    output        sq_empty_ff_and_reduce,           // 所有 QP 的 SQ 是否都为空
    output        o_prog_full_db_update_curr_sqptr

);
    `include "lrdma_defs.vh"


    // AXI lite operation registers
    reg  [                  20:0] wr_addr_r;
    reg  [                  20:0] rd_addr_r;
    reg                           wr_req_r;
    reg                           wr_req_mask_r;
    reg                           rd_req_r;
    reg                           reset_released_r;
    reg                           rvalid_delayed;
    reg                           rvalid_delayed_i;
    reg                           rd_req_mask;
    reg                           axi_lite_rvalid_i;

    reg                           lrdma_conf_lrdma_en;
    reg                           lrdma_conf_err_buf_en;
    reg  [                   1:0] lrdma_conf_flow_credits;
    reg  [C_QP_INDX_WIDTH:0] lrdma_conf_num_qps_enabled;
    reg  [                  15:0] lrdma_conf_udp_src_port;

    reg  [                  15:0] lrdma_sw_override_cnt;
    reg                           lrdma_adv_conf_sw_override;  // 上位机设置为0
    reg  [   C_QP_INDX_WIDTH-1:0] lrdma_adv_conf_sw_override_qpid;
    wire                          posedge_lrdma_sw_override;
    reg                           lrdma_adv_conf_sw_override_ff;
    reg                           lrdma_adv_conf_retry_cnt_fatal_dis;

    reg  [                  31:0] mac_lrdma_addr_lsb;
    reg  [                  15:0] mac_lrdma_addr_msb;

    reg  [                  15:0] roce_pause_on_threshold_reg;
    reg  [                  15:0] roce_pause_off_threshold_reg;
    reg                           pfc_en_roce_reg;
    reg                           pfc_en_non_roce_reg;
    reg  [                   3:0] roce_priority_reg;
    reg  [                   3:0] non_roce_priority_reg;
    reg                           disable_priority_chk_reg;

    reg  [                  31:0] ipv4_lrdma_addr;
    reg  [                  31:0] ipv6_lrdma_addr1;
    reg  [                  31:0] ipv6_lrdma_addr2;
    reg  [                  31:0] ipv6_lrdma_addr3;
    reg  [                  31:0] ipv6_lrdma_addr4;

    reg  [                  31:0] rq_err_pkt_buf_ba;
    reg  [                  31:0] rq_err_pkt_buf_ba_msb;
    reg  [                  15:0] rq_err_pkt_buf_sz_num_bufs;
    reg  [                  15:0] rq_err_pkt_buf_sz_buf_sz;

    reg  [C_QP_INDX_WIDTH -1 : 0] qp_conf_wr_addr_r;
    reg  [C_QP_INDX_WIDTH -1 : 0] qp_conf_wr_addr_2r;
    reg                           qp_conf_bram_wen_d;

    reg  [                  31:0] in_errsts_q_ba;
    reg  [                  31:0] in_errsts_q_ba_msb;
    reg  [                  15:0] in_errsts_q_sz;


    reg                           intr_en_pkt_valdn_err;
    reg                           intr_en_wqe_cmpl;
    reg                           intr_en_mad_pkt_rcvd;
    reg                           intr_en_fatal_err;


    reg                           qp_cq_hdptr_req_ff;
    reg                           qp_cq_hdptr_retry_req_ff;
    reg                           qp_conf_req_tx_engine_ff;
    reg                           qp_conf_req_ack_mgr_ff;
    reg                           qp_cfg_retry_cnt_rd_req_ff;
    reg                           qp_conf_req_rx_engine_ff;
    reg                           qp_conf_req_replica_ff;
    reg                           qp_adv_conf_req_ff;
    reg                           qp_rq_pi_db_req_ff;
    reg                           qp_ret_sq_psn_req_ff;
    reg                           qp_ret_sq_psn_req_int_ff;
    reg                           qp_rq_ba_req_ff;
    reg                           qp_sq_ba_req_int_ff;
    reg                           qp_cq_ba_req_ff;
    // reg                           qp_rq_wrptrdb_req_ff;
    // reg                           qp_sq_cmpldb_req_ff;
    reg                           qp_rq_cidb_req_ff;
    reg                           qp_sq_pidb_req_ff;
    reg                           qp_rq_depth_req_ff;
    reg                           qp_sq_depth_req_ff;
    reg                           qp_cq_depth_req_ff;
    reg                           qp_sq_psn_req_ff;
    reg                           qp_sq_psn_wqe_req_ff;
    reg                           qp_sq_psn_retry_req_ff;
    reg                           qp_last_req_ff;
    reg                           qp_dest_qpid_req_tx_engine_ff;
    reg                           qp_dest_qpid_req_rx_engine_ff;
    reg                           qp_mac_remote_addrl_req_ff;
    reg                           qp_mac_remote_addrl_req_replica_ff;
    reg                           qp_mac_remote_addrm_req_ff;
    reg                           qp_mac_remote_addrm_req_replica_ff;
    reg                           qp_mac_remote_addrl_wqe_req_ff;
    reg                           qp_mac_remote_addrm_wqe_req_ff;
    reg                           qp_ip_remote_addr1_req_ff;
    reg                           qp_ip_remote_addr2_req_ff;
    reg                           qp_ip_remote_addr3_req_ff;
    reg                           qp_ip_remote_addr4_req_ff;
    reg                           qp_ip_remote_addr1_req_replica_ff;
    reg                           qp_ip_remote_addr2_req_replica_ff;
    reg                           qp_ip_remote_addr3_req_replica_ff;
    reg                           qp_ip_remote_addr4_req_replica_ff;
    reg                           qp_ip_remote_addr1_wqe_req_ff;
    reg                           qp_ip_remote_addr2_wqe_req_ff;
    reg                           qp_ip_remote_addr3_wqe_req_ff;
    reg                           qp_ip_remote_addr4_wqe_req_ff;
    reg                           qp_curr_sqptr_req_ff;
    reg                           qp_stat_resp_psn_req_ff;
    reg                           qp_stat_nak_req_ff;
    reg                           qp_stat_rq_buf_ca_rdreq_ff;
    reg                           qp_stat_wqe_cnt_rdreq_ff;
    reg                           qp_stat_ssn_req_ff;
    reg                           qp_retry_cnt_req_ff;
    reg                           qp_retry_cnt_req_2ff;
    reg                           qp_rq_wrptrdb_add_req_ff;

    genvar i;

    wire [  C_QP_INDX_WIDTH -1:0] rq_cidb_idx;  // 读取 RQ CI doorbell 的 QP 索引

    wire [C_QP_INDX_WIDTH -1 : 0] qp_conf_wr_addr;
    reg                           qp_conf_bram_wen_r;
    reg                           qp_conf_bram_wen_p;
    wire                          qp_conf_bram_wen;

    reg                           qp_adv_conf_bram_wen_r;
    reg                           qp_adv_conf_bram_wen_p;
    wire                          qp_adv_conf_bram_wen;

    reg                           qp_rq_ba_bram_wen_r;
    reg                           qp_rq_ba_bram_wen_p;
    wire                          qp_rq_ba_bram_wen;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_ba_msb_bram_addr;
    reg                           qp_rq_ba_msb_bram_wen_r;
    reg                           qp_rq_ba_msb_bram_wen_p;
    wire                          qp_rq_ba_msb_bram_wen;

    reg                           qp_sq_ba_bram_wen_r;
    reg                           qp_sq_ba_bram_wen_p;
    wire                          qp_sq_ba_bram_wen;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_ba_msb_bram_addr;
    reg                           qp_sq_ba_msb_bram_wen_r;
    reg                           qp_sq_ba_msb_bram_wen_p;
    wire                          qp_sq_ba_msb_bram_wen;

    reg                           qp_cq_ba_bram_wen_r;
    reg                           qp_cq_ba_bram_wen_p;
    wire                          qp_cq_ba_bram_wen;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_ba_msb_bram_addr;
    reg                           qp_cq_ba_msb_bram_wen_r;
    reg                           qp_cq_ba_msb_bram_wen_p;
    wire                          qp_cq_ba_msb_bram_wen;

    reg                           qp_rq_wrptrdb_add_bram_wen_r;
    reg                           qp_rq_wrptrdb_add_bram_wen_p;
    wire                          qp_rq_wrptrdb_add_bram_wen;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_msb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_msb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_rq_wrptrdb_add_msb_bram_addr;
    reg                           qp_rq_wrptrdb_add_msb_bram_wen_r;
    reg                           qp_rq_wrptrdb_add_msb_bram_wen_p;
    wire                          qp_rq_wrptrdb_add_msb_bram_wen;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_hdptr_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_hdptr_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_cq_hdptr_bram_addr;
    wire                          qp_cq_hdptr_bram_wen;

    reg                           qp_rq_cidb_bram_wen_r;
    reg                           qp_rq_cidb_bram_wen_p;
    wire                          qp_rq_cidb_bram_wen;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_pidb_bram_addr;
    wire                          qp_sq_pidb_bram_wen;

    reg                           qp_q_depth_bram_wen_r;
    reg                           qp_q_depth_bram_wen_p;
    wire                          qp_q_depth_bram_wen;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_psn_wr_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_psn_rd_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_sq_psn_bram_addr;
    wire                          qp_sq_psn_bram_wen;

    reg                           qp_last_rq_bram_wen_r;
    reg                           qp_last_rq_bram_wen_p;
    wire                          qp_last_req_bram_wen;










    reg                           qp_dest_qpid_bram_wen_r;
    reg                           qp_dest_qpid_bram_wen_p;
    wire                          qp_dest_qpid_bram_wen;

    reg                           qp_mac_remote_addrl_bram_wen_r;
    reg                           qp_mac_remote_addrl_bram_wen_p;
    wire                          qp_mac_remote_addrl_bram_wen;

    reg                           qp_mac_remote_addrm_bram_wen_r;
    reg                           qp_mac_remote_addrm_bram_wen_p;
    wire                          qp_mac_remote_addrm_bram_wen;

    reg                           qp_ip_remote_addr1_bram_wen_r;
    reg                           qp_ip_remote_addr1_bram_wen_p;
    wire                          qp_ip_remote_addr1_bram_wen;

    reg                           qp_ip_remote_addr2_bram_wen_r;
    reg                           qp_ip_remote_addr2_bram_wen_p;
    wire                          qp_ip_remote_addr2_bram_wen;

    reg                           qp_ip_remote_addr3_bram_wen_r;
    reg                           qp_ip_remote_addr3_bram_wen_p;
    wire                          qp_ip_remote_addr3_bram_wen;

    reg                           qp_ip_remote_addr4_bram_wen_r;
    reg                           qp_ip_remote_addr4_bram_wen_p;
    wire                          qp_ip_remote_addr4_bram_wen;

    reg                           qp_stat_ssn_bram_wen_r;
    reg                           qp_stat_ssn_bram_wen_p;
    wire                          qp_stat_ssn_bram_wen;

    wire [  C_QP_INDX_WIDTH -1:0] stat_curr_sqptr_proc_addr;
    wire [  C_QP_INDX_WIDTH -1:0] stat_curr_sqptr_proc_wr_addr;
    wire [  C_QP_INDX_WIDTH -1:0] stat_curr_sqptr_proc_rd_addr;
    wire [  C_QP_INDX_WIDTH -1:0] stat_curr_sqptr_proc_bram_addr;
    wire                          axi_curr_sqptr_proc_read;
    wire                          stat_curr_sqptr_bram_wen;
    wire                          qp_stat_rq_pi_db_bram_wen;
    wire [                  15:0] curr_sqptr_proc;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_nak_bram_addr;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_nak_bram_rd_addr;
    wire                          qp_stat_nak_bram_wen;

    wire [C_QP_INDX_WIDTH -1 : 0] qp_wqe_cnt_bram_addr;
    wire                          qp_wqe_cnt_bram_wen;
    wire [C_QP_INDX_WIDTH -1 : 0] qp_stat_rq_pi_db_bram_addr;

    wire [  C_QP_INDX_WIDTH -1:0] qp_depth_idx;  // QDEPTHi 读取的 QP 索引
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_sq_psn_idx;

    wire [                  31:0] s_axi_qp_config_rdata;
    wire [                  31:0] s_axi_qp_adv_config_rdata;
    wire [                  31:0] s_axi_rq_ba_rdata;
    wire [                  31:0] s_axi_sq_ba_rdata;
    wire [                  31:0] s_axi_cq_ba_rdata;
    wire [                  31:0] s_axi_rq_ba_msb_rdata;
    wire [                  31:0] s_axi_sq_ba_msb_rdata;
    wire [                  31:0] s_axi_cq_ba_msb_rdata;
    wire [                  31:0] s_axi_rq_wrptrdb_add_rdata;
    wire [                  31:0] s_axi_rq_wrptrdb_msb_add_rdata;
    wire [                  31:0] s_axi_cq_hdptr_rdata;
    wire [                  31:0] s_axi_rq_cidb_rdata;
    wire [                  31:0] s_axi_sq_pidb_rdata;
    wire [                  31:0] s_axi_q_depth_rdata;
    wire [                  31:0] s_axi_sq_psn_rdata;
    wire [                  31:0] s_axi_last_req_rdata;
    wire [                  31:0] s_axi_dest_qpid_rdata;
    wire [                  31:0] s_axi_timeout_rdata;
    wire [                  31:0] s_axi_mac_remote_addrl_rdata;
    wire [                  31:0] s_axi_mac_remote_addrm_rdata;
    wire [                  31:0] s_axi_ip_remote_addr1_rdata;
    wire [                  31:0] s_axi_ip_remote_addr2_rdata;
    wire [                  31:0] s_axi_ip_remote_addr3_rdata;
    wire [                  31:0] s_axi_ip_remote_addr4_rdata;
    wire [                  31:0] s_axi_curr_sqptr_proc_rdata;
    wire [                  31:0] s_axi_stat_resp_psn_rdata;
    wire [                  31:0] s_axi_stat_rq_buf_ca_rdata;
    wire [                  31:0] s_axi_stat_rq_buf_ca_msb_rdata;
    wire [                  31:0] s_axi_stat_wqe_cnt_rdata;
    wire [                  31:0] s_axi_stat_rq_pi_db_rdata;
    wire [                  31:0] s_axi_stat_ret_sq_psn_rdata;
    wire [                  31:0] s_axi_stat_qpn_rdata;
    wire [                  31:0] s_axi_stat_ssn_rdata;
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_mac_remote_addrl_idx;  // MACDESADDLSBi 多读源仲裁后的 QP 索引
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_mac_remote_addrm_idx;  // MACDESADDMSBi 多读源仲裁后的 QP 索引
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_ip_remote_addr1_idx;  // IPDESADDR1i 多读源仲裁后的 QP 索引
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_ip_remote_addr2_idx;  // IPDESADDR2i 多读源仲裁后的 QP 索引
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_ip_remote_addr3_idx;  // IPDESADDR3i 多读源仲裁后的 QP 索引
    wire [  C_QP_INDX_WIDTH -1:0] muxed_qp_ip_remote_addr4_idx;  // IPDESADDR4i 多读源仲裁后的 QP 索引


    wire                          generic_reg_rd_access;
    wire                          generic_reg_wr_access;  // 地址是否0x10开头(0x10_0000)？1-是，0-不是
    wire                          intr_clr;
    wire [                  15:0] sq_pidb;

    wire [   C_QP_INDX_WIDTH-1:0] qp_stat_ret_sq_psn_rd_addr;

    wire [   C_QP_INDX_WIDTH-1:0] qp_stat_rq_pi_db_wr_addr;
    wire [   C_QP_INDX_WIDTH-1:0] qp_stat_rq_pi_db_rd_addr;

    reg                           qp_timeout_bram_wen_r;
    reg                           qp_timeout_bram_wen_p;
    wire                          qp_timeout_bram_wen;

    wire [   C_QP_INDX_WIDTH-1:0] qp_stat_ret_sq_psn_bram_addr;
    reg  [                   2:0] pending_upd_cs;

    wire                          sq_empty;
    wire                          sq_full;
    reg                           sq_empty_ff_valid;
    reg  [  C_QP_INDX_WIDTH -1:0] sq_empty_ff_id;
    reg                           sq_empty_ff_data;
    wire                          sq_empty_ff_id_rd_data;
    reg                           sq_full_ff_valid;
    reg  [  C_QP_INDX_WIDTH -1:0] sq_full_ff_id;
    reg                           sq_full_ff_data;


    reg                           curr_sqptr_vld_r;
    reg  [   C_QP_INDX_WIDTH-1:0] curr_sqptr_addr_r;
    reg  [                  15:0] curr_sqptr_data_r;
    wire                          curr_sqptr_vld_c;
    wire [   C_QP_INDX_WIDTH-1:0] curr_sqptr_addr_c;
    wire [                  15:0] curr_sqptr_data_c;
    reg                           curr_sqptr_rden_r;
    wire                          curr_sqptr_fifo_empty;
    wire                          curr_sqptr_fifo_full;
    wire [   C_QP_INDX_WIDTH-1:0] curr_sqptr_fifo_dout;
    wire [                   4:0] curr_sqptr_fifo_dcount;
    wire [                  15:0] curr_sqptr_flag_ptr;  // 正在被硬件消费的 SQ 条目指针

    reg                           sq_pidb_vld_r;
    reg  [   C_QP_INDX_WIDTH-1:0] sq_pidb_addr_r;
    reg  [                  15:0] sq_pidb_data_r;
    wire                          sq_pidb_vld_c;
    wire [   C_QP_INDX_WIDTH-1:0] sq_pidb_addr_c;
    wire [                  15:0] sq_pidb_data_c;
    reg                           sq_pidb_rden_r;
    wire                          sq_pidb_fifo_empty;
    wire [   C_QP_INDX_WIDTH-1:0] sq_pidb_fifo_dout;
    wire [                   4:0] sq_pidb_fifo_dcount;
    wire [                  15:0] sq_pidb_flag_ptr;  // 软件通过 SQ 门铃写入的最新生产者指针（“写指针”）？


    reg                           flag_updt_cs;
    reg  [   C_QP_INDX_WIDTH-1:0] sq_flag_updt_addr;
    reg                           sq_flag_updt_vld;
    reg  [   C_QP_INDX_WIDTH-1:0] sq_flag_updt_idx;

    reg  [                  31:0] cq_intr_sts_clr_wr_en;
    reg  [                  10:0] cq_intr_sts_clr_wr_addr;
    wire                          qp_fatal_sts;
    reg                           qp_fatal_sts_ff;
    reg  [                   2:0] mux_select;
    reg  [                  31:0] s_axi_lite_reg_1;
    reg  [                  31:0] s_axi_lite_reg_2;
    reg  [                  31:0] s_axi_lite_reg_3;
    reg  [                  31:0] s_axi_lite_reg_4;
    reg  [                  31:0] s_axi_lite_reg_5;
    reg  [                  31:0] s_axi_lite_reg_6;
    reg  [                  31:0] s_axi_lite_reg_1_r;
    reg  [                  31:0] s_axi_lite_reg_2_r;
    reg  [                  31:0] s_axi_lite_reg_3_r;
    reg  [                  31:0] s_axi_lite_reg_4_r;
    reg  [                  31:0] s_axi_lite_reg_5_r;
    reg  [                  31:0] s_axi_lite_reg_6_r;
    wire [                  31:0] s_axi_lite_reg_1_c;
    wire [                  31:0] s_axi_lite_reg_2_c;
    wire [                  31:0] s_axi_lite_reg_3_c;
    wire [                  31:0] s_axi_lite_reg_4_c;
    wire [                  31:0] s_axi_lite_reg_5_c;
    wire [                  31:0] s_axi_lite_reg_6_c;
    reg  [                   5:0] read_mux;

    reg  [                  31:0] s_axi_stat_qpn_rdata_r;
    reg  [                  31:0] s_axi_curr_sqptr_proc_rdata_r;
    reg  [                  31:0] s_axi_stat_rq_pi_db_rdata_r;

    reg  [                  31:0] s_axi_stat_qpn_rdata_p;
    reg  [                  31:0] s_axi_curr_sqptr_proc_rdata_p;
    reg  [                  31:0] s_axi_stat_rq_pi_db_rdata_p;

    reg  [                  31:0] s_axi_stat_qpn_rdata_p1;
    reg  [                  31:0] s_axi_curr_sqptr_proc_rdata_p1;
    reg  [                  31:0] s_axi_stat_rq_pi_db_rdata_p1;

    reg  [                  31:0] s_axi_stat_qpn_rdata_p2;
    reg  [                  31:0] s_axi_curr_sqptr_proc_rdata_p2;
    reg  [                  31:0] s_axi_stat_rq_pi_db_rdata_p2;
    wire [                  20:0] rd_addr_c1;
    reg  [                  20:0] rd_addr_p1;
    reg  [                  20:0] rd_addr_p2;
    reg  [                  20:0] rd_addr_r1;
    reg  [                   2:0] rd_addr_wen_r;
    reg                           rd_req_p1;
    reg                           rd_req_p2;
    reg                           rd_req_p3;
    reg                           rd_req_r1;
    reg  [                   1:0] wr_addr_wen_r;
    reg  [                  20:0] wr_addr_r1;
    wire [  C_QP_INDX_WIDTH -1:0] wr_addr_mem_r;
    wire [                  31:0] wr_data_mem_r;
    reg  [  C_QP_INDX_WIDTH -1:0] wr_addr_mem_r_r;
    reg  [  C_QP_INDX_WIDTH -1:0] rd_addr_mem_r_r;
    reg  [                  31:0] wr_data_mem_r_r;
    reg  [  C_QP_INDX_WIDTH -1:0] wr_addr_mem_r_p;
    reg  [                  31:0] wr_data_mem_r_p;
    reg                           wr_req_r1;
    reg  [                  31:0] wr_data_r1;
    reg                           osq_full_valid;
    reg  [ C_QP_INDX_WIDTH-1 : 0] osq_full_id;
    reg                           osq_full_data;
    reg                           osq_full_valid_pending;
    reg  [ C_QP_INDX_WIDTH-1 : 0] osq_full_id_pending;
    wire                          osq_full_rd_data;
    wire                          osq_full_rd_data_int;
    wire                          i_rq_full_rd_data;
    wire                          fatal_err_rd_data;
    wire                          sq_empty_ff_rd_data;
    wire                          sq_full_ff_rd_data;
    reg                           osq_empty_valid;
    reg  [ C_QP_INDX_WIDTH-1 : 0] osq_empty_id;
    reg                           osq_empty_data;
    reg                           osq_empty_valid_pending;
    reg  [ C_QP_INDX_WIDTH-1 : 0] osq_empty_id_pending;
    wire                          osq_empty_rd_data;
    wire                          osq_empty_rd_data_int;
    wire                          qp_retried_rd_data;
    reg  [   C_QP_INDX_WIDTH : 0] fatal_err_ones_count;
    wire                          fatal_id_rd_data;
    wire [                  31:0] w_qp_wq_cmpl_intr;
    wire [                  31:0] w_qp_wq_cmpl_intr_int;
    reg  [   C_QP_INDX_WIDTH : 0] sq_empty_ff_zeros_count;
    reg                           s_axi_lite_wready_i;
    reg                           w_channel_hwshake_before_aw;
    reg                           aw_window_active;
    wire                          prog_full_sq_db_update;
    wire                          prog_full_db_update_curr_sqptr;


    // 将 QP 分成 8 个相等的部分。每块称为 QP_CHUNK
    // 这样做是为了在只有几个 QP 被激活的情况下更快地更新 sq_empty/full 信号
    localparam QP_CHUNK = (C_NUM_QP < 8) ? 1 : (C_NUM_QP / 8);  //C_NUM_QP/8;

    // 以下两个参数的 C_NUM_QP 比较值必须保持一致
    localparam C_CONFIG_MEM_LATENCY = (C_NUM_QP <= 256) ? 3 : 4;  // 3
    localparam CONFIG_MEM_LATENCY_H = C_CONFIG_MEM_LATENCY;  // 3
    localparam CONFIG_MEM_LATENCY_M = C_CONFIG_MEM_LATENCY - 1;  // 2
    localparam CONFIG_MEM_LATENCY_L = 1;
    localparam CONFIG_MEM_LATENCY_K = CONFIG_MEM_LATENCY_L + 1;

    localparam IDLE = 3'b000;
    localparam FIND_PEND_UPD = 3'b001;
    localparam WAIT_BRAM = 3'b010;
    localparam INCR_MSB_CNT = 3'b011;
    localparam INCR_LSB_CNT = 3'b100;
    localparam UPDATE_PEND_STAT = 3'b101;

    localparam WAIT_FOR_UPDT = 1'b0;
    localparam READ_SQ_PTR = 1'b1;

    // 空洞地址段

    // 全局寄存器偏移
    localparam LRDMA_CONF_REG = 21'h10_0000;
    localparam LRDMA_ADV_CONF_REG = 21'h10_0004;

    localparam LRDMA_BUF_THRESHOLD_ROCE_REG = 21'h10_0008;
    localparam LRDMA_PAUSE_CONF_REG = 21'h10_000C;

    localparam MAC_LRDMA_ADDR_LSB_REG = 21'h10_0010;
    localparam MAC_LRDMA_ADDR_MSB_REG = 21'h10_0014;

    localparam IPV4_LRDMA_ADDR_REG = 21'h100070;
    localparam IPV6_LRDMA_ADDR1_REG = 21'h100020;
    localparam IPV6_LRDMA_ADDR2_REG = 21'h100024;
    localparam IPV6_LRDMA_ADDR3_REG = 21'h100028;
    localparam IPV6_LRDMA_ADDR4_REG = 21'h10002c;

    localparam LRDMA_CONF_QP_EN_REG = 21'h100044;

    localparam RQ_ERR_PKT_BUF_BA_REG = 21'h100060;
    localparam RQ_ERR_PKT_BUF_BA_REG_MSB = 21'h100064;
    localparam RQ_ERR_PKT_BUF_SZ_REG = 21'h100068;
    localparam RQ_ERR_PKT_BUF_WRPTR_REG = 21'h10006c;

    localparam IN_ERRSTS_Q_BA_REG = 21'h100088;
    localparam IN_ERRSTS_Q_BA_REG_MSB = 21'h10008c;
    localparam IN_ERRSTS_Q_SZ_REG = 21'h100090;
    localparam IN_ERRSTS_Q_WRPTR_REG = 21'h100094;

    // 包计数和状态计数
    localparam INC_MA_PKT_CNT_REG = 21'h10_0104;  // {mad count, ack count}
    localparam OUT_WR_PKT_CNT_REG = 21'h10_0108;
    localparam OUT_MAD_PKT_CNT_REG = 21'h10_010C;  // {mad count, 16'd0}

    localparam LAST_IN_PKT_REG = 21'h10_0110;
    localparam LAST_OUT_PKT_REG = 21'h10_0114;
    localparam INV_DUP_PKT_CNT_REG = 21'h10_0118;
    localparam TX_ENGINE_STS_REG = 21'h10_011C;

    localparam QP_MGR_STS_REG = 21'h10_0120;
    localparam INC_ALL_DROPPED_CNT_REG = 21'h10_0124;
    localparam INC_NACK_CNT_REG = 21'h10_0128;
    localparam ACK_MGR_STS_REG = 21'h10_0130;
    localparam RETRY_CNT_STS_REG = 21'h10_0134;
    localparam DBG_CNT_OVERRIDE_REG = 21'h10_0138;

    // 中断
    localparam INTR_EN_REG = 21'h10_0140;
    localparam INTR_STS_REG = 21'h10_0144;


    // 假设所有RQ、CQ中断偏移量必须是连续的；否则 LUT RAM 读取寻址将会失败。
    localparam CQ_INTR_STS1_REG = 21'h10_0200;
    localparam CQ_INTR_STS2_REG = 21'h10_0204;
    localparam CQ_INTR_STS3_REG = 21'h10_0208;
    localparam CQ_INTR_STS4_REG = 21'h10_020C;

    localparam CQ_INTR_STS5_REG = 21'h10_0210;
    localparam CQ_INTR_STS6_REG = 21'h10_0214;
    localparam CQ_INTR_STS7_REG = 21'h10_0218;
    localparam CQ_INTR_STS8_REG = 21'h10_021C;

    localparam CQ_INTR_STS9_REG = 21'h10_0220;
    localparam CQ_INTR_STS10_REG = 21'h10_0224;
    localparam CQ_INTR_STS11_REG = 21'h10_0228;
    localparam CQ_INTR_STS12_REG = 21'h10_022C;

    localparam CQ_INTR_STS13_REG = 21'h10_0230;
    localparam CQ_INTR_STS14_REG = 21'h10_0234;
    localparam CQ_INTR_STS15_REG = 21'h10_0238;
    localparam CQ_INTR_STS16_REG = 21'h10_023C;

    localparam CQ_INTR_STS17_REG = 21'h10_0240;
    localparam CQ_INTR_STS18_REG = 21'h10_0244;
    localparam CQ_INTR_STS19_REG = 21'h10_0248;
    localparam CQ_INTR_STS20_REG = 21'h10_024C;

    localparam CQ_INTR_STS21_REG = 21'h10_0250;
    localparam CQ_INTR_STS22_REG = 21'h10_0254;
    localparam CQ_INTR_STS23_REG = 21'h10_0258;
    localparam CQ_INTR_STS24_REG = 21'h10_025C;

    localparam CQ_INTR_STS25_REG = 21'h10_0260;
    localparam CQ_INTR_STS26_REG = 21'h10_0264;
    localparam CQ_INTR_STS27_REG = 21'h10_0268;
    localparam CQ_INTR_STS28_REG = 21'h10_026C;

    localparam CQ_INTR_STS29_REG = 21'h10_0270;
    localparam CQ_INTR_STS30_REG = 21'h10_0274;
    localparam CQ_INTR_STS31_REG = 21'h10_0278;
    localparam CQ_INTR_STS32_REG = 21'h10_027C;

    localparam CQ_INTR_STS33_REG = 21'h10_0280;
    localparam CQ_INTR_STS34_REG = 21'h10_0284;
    localparam CQ_INTR_STS35_REG = 21'h10_0288;
    localparam CQ_INTR_STS36_REG = 21'h10_028C;

    localparam CQ_INTR_STS37_REG = 21'h10_0290;
    localparam CQ_INTR_STS38_REG = 21'h10_0294;
    localparam CQ_INTR_STS39_REG = 21'h10_0298;
    localparam CQ_INTR_STS40_REG = 21'h10_029C;

    localparam CQ_INTR_STS41_REG = 21'h10_02A0;
    localparam CQ_INTR_STS42_REG = 21'h10_02A4;
    localparam CQ_INTR_STS43_REG = 21'h10_02A8;
    localparam CQ_INTR_STS44_REG = 21'h10_02AC;

    localparam CQ_INTR_STS45_REG = 21'h10_02B0;
    localparam CQ_INTR_STS46_REG = 21'h10_02B4;
    localparam CQ_INTR_STS47_REG = 21'h10_02B8;
    localparam CQ_INTR_STS48_REG = 21'h10_02BC;

    localparam CQ_INTR_STS49_REG = 21'h10_02C0;
    localparam CQ_INTR_STS50_REG = 21'h10_02C4;
    localparam CQ_INTR_STS51_REG = 21'h10_02C8;
    localparam CQ_INTR_STS52_REG = 21'h10_02CC;

    localparam CQ_INTR_STS53_REG = 21'h10_02D0;
    localparam CQ_INTR_STS54_REG = 21'h10_02D4;
    localparam CQ_INTR_STS55_REG = 21'h10_02D8;
    localparam CQ_INTR_STS56_REG = 21'h10_02DC;

    localparam CQ_INTR_STS57_REG = 21'h10_02E0;
    localparam CQ_INTR_STS58_REG = 21'h10_02E4;
    localparam CQ_INTR_STS59_REG = 21'h10_02E8;
    localparam CQ_INTR_STS60_REG = 21'h10_02EC;

    localparam CQ_INTR_STS61_REG = 21'h10_02F0;
    localparam CQ_INTR_STS62_REG = 21'h10_02F4;
    localparam CQ_INTR_STS63_REG = 21'h10_02F8;
    localparam CQ_INTR_STS64_REG = 21'h10_02FC;

    localparam integer C_CQ_INTR_STS_BANKS = (C_NUM_QP + 31) / 32;

    // 映射到相应的 CQ 中断状态寄存器
    function [31:0] cq_intr_sts_read_bank;  // 读 cq_intr_sts
        input [5:0] bank_id;
        begin
            if (bank_id < C_CQ_INTR_STS_BANKS) begin
                cq_intr_sts_read_bank = w_qp_wq_cmpl_intr;
            end else begin
                cq_intr_sts_read_bank = 32'd0;
            end
        end
    endfunction
    task cq_intr_sts_clear_bank;  // 写 cq_intr_sts
        input [5:0] bank_id;
        begin
            if (bank_id < C_CQ_INTR_STS_BANKS) begin
                cq_intr_sts_clr_wr_en   <= s_axi_lite_wdata[31:0];
                cq_intr_sts_clr_wr_addr <= {bank_id, 5'd0};
            end else begin
                cq_intr_sts_clr_wr_en   <= 32'd0;
                cq_intr_sts_clr_wr_addr <= 11'd0;
            end
        end
    endtask


    // Per-QP 寄存器偏移
    localparam QP_CONF_QPN = 8'h00;
    localparam QP_ADV_CONF_QPN = 8'h04;

    localparam RQ_BUF_BA_QPN = 8'h08;
    localparam RQ_BUF_BA_MSB_QPN = 8'hc0;
    localparam STAT_RQ_BUF_CA = 8'h94;
    localparam STAT_RQ_BUF_CA_MSB = 8'hd8;
    localparam RQ_WRPTR_DB_ADD_QPN = 8'h20;
    localparam RQ_WRPTR_DB_ADD_QPN_MSB = 8'h24;
    localparam STAT_RQ_PI_DB = 8'h9c;
    localparam RQ_CI_DB_QPN = 8'h34;

    localparam SQ_BA_QPN = 8'h10;
    localparam SQ_BA_MSB_QPN = 8'hc8;
    localparam SQ_PI_DB_QPN = 8'h38;
    localparam STAT_CURR_SQPTR_PROC_QPN = 8'h8c;
    localparam SQ_PSN_QPN = 8'h40;
    localparam STAT_RET_SQ_PSN = 8'hf0;

    localparam CQ_BA_QPN = 8'h18;
    localparam CQ_BA_MSB_QPN = 8'hd0;
    localparam CQ_HEAD_QPN = 8'h30;

    localparam Q_DEPTH_QPN = 8'h3c;
    localparam DEST_QP_CONF_QPN = 8'h48;
    localparam TIMEOUT_QPN = 8'h4c;

    localparam MAC_REMOTE_ADDR_LSB_QPN = 8'h50;
    localparam MAC_REMOTE_ADDR_MSB_QPN = 8'h54;

    localparam IP_REMOTE_ADDR1_QPN = 8'h60;
    localparam IP_REMOTE_ADDR2_QPN = 8'h64;
    localparam IP_REMOTE_ADDR3_QPN = 8'h68;
    localparam IP_REMOTE_ADDR4_QPN = 8'h6c;

    localparam STAT_SSN_QPN = 8'h80;
    localparam STAT_QPN = 8'h88;
    localparam STAT_WQE_CNT = 8'h98;
    localparam STAT_RESP_PSN_QPN = 8'h90;
    localparam LAST_REQ_QPN = 8'h44;

    localparam LOG_NUM_QP = clog2(C_NUM_QP);

    //***************************************************************************
    // 只有在没有待处理的读取或写入事务时，才会接受写地址阶段。
    // 当读取和写入事务发生在同一时钟上时，读取事务将获得最高优先级并优先处理。
    //***************************************************************************
    //assign s_axi_lite_awready = ((~wr_req_r) && (~wr_req_mask_r) && (!(rd_req_r || s_axi_lite_arvalid))) && reset_released_r;
    assign s_axi_lite_awready = (~wr_req_r) && reset_released_r;
    assign s_axi_lite_bresp   = 2'b00;
    assign s_axi_lite_rresp   = 2'b00;
    //assign s_axi_lite_wready  = wr_req_r && ~s_axi_lite_bvalid;
    //assign s_axi_lite_wready  = wr_req_r && ~s_axi_lite_bvalid && ~(i_qp_sq_pidb_wr_valid_hndshk && sq_pidb_req_mask_r);
    //assign s_axi_lite_arready = ~rd_req_r && ~wr_req_r && reset_released_r;
    assign s_axi_lite_wready  = s_axi_lite_wready_i;
    assign s_axi_lite_arready = ~rd_req_r && reset_released_r;


    //***************************************************************************
    // 复位释放后，AWREADY 和 ARREADY 至少保持低 1 拍。
    // reset_released_r 与 s_axi_lite_awready / s_axi_lite_arready 共同门控，
    // 确保复位期间输出为 0。
    //***************************************************************************
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            reset_released_r            <= 1'b0;
            s_axi_lite_wready_i         <= 1'b0;
            w_channel_hwshake_before_aw <= 1'b0;
            aw_window_active            <= 1'b0;
        end else begin
            reset_released_r <= 1'b1;
            if ((s_axi_lite_wready && s_axi_lite_wvalid) || (s_axi_lite_awvalid && s_axi_lite_awready)) begin
                s_axi_lite_wready_i <= 1'b0;
            end else if (!wr_req_mask_r && wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && (!(rd_req_r || s_axi_lite_arvalid)) && !s_axi_lite_bvalid) begin
                s_axi_lite_wready_i <= 1'b1;
            end

            if ((s_axi_lite_wready && s_axi_lite_wvalid) && !((s_axi_lite_awvalid && s_axi_lite_awready) || aw_window_active)) begin
                w_channel_hwshake_before_aw <= 1'b1;  // in case w channel data comes first; need to wait for aw channel
            end else if (!wr_req_mask_r && wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && (!(rd_req_r || s_axi_lite_arvalid)) && !s_axi_lite_bvalid) begin
                w_channel_hwshake_before_aw <= 1'b0;
            end

            if (s_axi_lite_awvalid && s_axi_lite_awready) begin
                aw_window_active <= 1'b1;
            end else if (!wr_req_r) begin
                aw_window_active <= 1'b0;
            end
        end
    end

    //***************************************************************************
    // AXI-Lite 事务译码和地址锁存逻辑。
    // master 拉高 s_axi_lite_a*valid 后锁存地址，并保持 wr_req_r 或 rd_req_r，
    // 直到对应数据阶段完成。
    //***************************************************************************


    always @(posedge core_clk) begin
        if (~core_rstn) begin
            wr_req_r      <= 1'b0;
            wr_req_mask_r <= 1'b0;
            rd_req_r      <= 1'b0;
            wr_addr_r     <= 'd0;
            rd_addr_r     <= 'd0;
        end else begin
            if (s_axi_lite_awvalid && s_axi_lite_awready) begin
                wr_req_r  <= 1'b1;
                wr_addr_r <= s_axi_lite_awaddr[20:0];
                //end else if (s_axi_lite_bvalid && s_axi_lite_bready) begin
            end else if (!wr_req_mask_r && wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && (!(rd_req_r || s_axi_lite_arvalid)) && !s_axi_lite_bvalid) begin
                wr_req_r  <= 1'b0;
                wr_addr_r <= 'd0;
            end else begin
                wr_req_r  <= wr_req_r;
                wr_addr_r <= wr_addr_r;
            end

            if (qp_sq_pidb_bram_wen && !prog_full_sq_db_update) begin
                wr_req_mask_r <= 1'b0;
            end else if (!i_qp_curr_sqptr_proc_wen && stat_curr_sqptr_bram_wen && !prog_full_db_update_curr_sqptr) begin
                wr_req_mask_r <= 1'b0;
                // 0x18_008C: Status Current SQ ptr under process
            end else if (s_axi_lite_awvalid && s_axi_lite_awready && (s_axi_lite_awaddr[7:0] == STAT_CURR_SQPTR_PROC_QPN && s_axi_lite_awaddr[20:19] == 2'b11)) begin
                wr_req_mask_r <= 1'b1;
                // 0x18_0038: SQ Producer index QPi
            end else if (s_axi_lite_awvalid && s_axi_lite_awready && (s_axi_lite_awaddr[7:0] == SQ_PI_DB_QPN && s_axi_lite_awaddr[20:19] == 2'b11)) begin
                wr_req_mask_r <= 1'b1;
            end

            if (s_axi_lite_arvalid && s_axi_lite_arready) begin
                rd_req_r  <= 1'b1;
                rd_addr_r <= s_axi_lite_araddr[20:0];
            end else if (s_axi_lite_rvalid && s_axi_lite_rready) begin
                rd_req_r  <= 1'b0;
                rd_addr_r <= rd_addr_r;
            end else begin
                rd_req_r  <= rd_req_r;
                rd_addr_r <= rd_addr_r;
            end
        end
    end


    generate
        if (C_NUM_QP > 32) begin : Intr_sts_handling  // C_NUM_QP > 32
            integer k;

            for (i = 0; i <= 31; i = i + 1) begin
                lrdma_common_sdp_lut_ram_wrapper #(
                    .C_RAM_WORD_WIDTH(1),
                    .C_RAM_DEPTH(C_NUM_QP / 32)
                ) inst_i_qp_wq_cmpl_intr (
                    .core_clk(core_clk),
                    .core_rst(!core_rstn),
                    .wr_en   (init_start || i_qp_wq_cmpl_intr_wr_en[i]),
                    .wr_addr (init_start ? init_count[C_QP_INDX_WIDTH-1 : 5] : i_qp_wq_cmpl_intr_wr_addr[C_QP_INDX_WIDTH-1 : 5]),
                    .wr_data (init_start ? 1'b0 : i_qp_wq_cmpl_intr_wr_data[i]),
                    .rd_en   (1'b1),
                    .rd_addr (rd_addr_c1[C_QP_INDX_WIDTH-4 : 2] - CQ_INTR_STS1_REG[C_QP_INDX_WIDTH-4:2]),
                    .rd_data (w_qp_wq_cmpl_intr_int[i])
                );
                assign w_qp_wq_cmpl_intr[i] = (i_qp_wq_cmpl_intr_wr_en_pending[i] && i_qp_wq_cmpl_intr_wr_addr_pending[C_QP_INDX_WIDTH-1:5] == rd_addr_c1[C_QP_INDX_WIDTH-4 : 2] - CQ_INTR_STS1_REG[C_QP_INDX_WIDTH-4:2]) ? 1'b1 : w_qp_wq_cmpl_intr_int[i];
            end  // loop

        end // if (C_NUM_QP > 32)

        else begin : Intr_sts_handling_less_than_32 // C_NUM_QP <= 32
            reg     [31:0] w_qp_wq_cmpl_intr_r;
            integer        k;

            always @(posedge core_clk) begin
                if (~core_rstn) begin
                    w_qp_wq_cmpl_intr_r <= 32'd0;
                end else begin
                    for (k = 0; k < 32; k = k + 1) begin
                        if (i_qp_wq_cmpl_intr_wr_en[k]) begin
                            w_qp_wq_cmpl_intr_r[k] <= i_qp_wq_cmpl_intr_wr_data[k];
                        end
                    end
                end
            end
            assign w_qp_wq_cmpl_intr_int = w_qp_wq_cmpl_intr_r;
            for (i = 0; i <= 31; i = i + 1) begin
                assign w_qp_wq_cmpl_intr[i] = (i_qp_wq_cmpl_intr_wr_en_pending[i]) ? 1'b1 : w_qp_wq_cmpl_intr_int[i];
            end

        end  // C_NUM_QP <= 32
    endgenerate

    //***************************************************************************
    // AXI-Lite 读事务处理逻辑。
    // 收到读事务后，根据地址位 [13:2] 选择读数据，并通过 s_axi_lite_rdata
    // 与 s_axi_lite_rvalid 返回。地址位 [1:0] 不参与译码，访问应为字对齐。
    //***************************************************************************

    //assign generic_reg_rd_access = (rd_addr_r[C_AXIS_DATA_WIDTH-1:9] == 'b0);
    assign generic_reg_rd_access = (rd_addr_r[20:19] == 2'b10);

    assign rd_addr_c1            = (C_CONFIG_MEM_LATENCY == 3) ? rd_addr_p1 : rd_addr_p2;

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            mux_select                     <= 3'b000;
            s_axi_lite_reg_1               <= 'd0;
            s_axi_lite_reg_2               <= 'd0;
            s_axi_lite_reg_3               <= 'd0;
            s_axi_lite_reg_4               <= 'd0;
            s_axi_lite_reg_5               <= 'd0;
            s_axi_lite_reg_6               <= 'd0;
            s_axi_lite_reg_1_r             <= 'd0;
            s_axi_lite_reg_2_r             <= 'd0;
            s_axi_lite_reg_3_r             <= 'd0;
            s_axi_lite_reg_4_r             <= 'd0;
            s_axi_lite_reg_5_r             <= 'd0;
            s_axi_lite_reg_6_r             <= 'd0;
            read_mux                       <= 9'b0_0000_0000;

            s_axi_stat_qpn_rdata_p         <= 'd0;
            s_axi_curr_sqptr_proc_rdata_p  <= 'd0;
            s_axi_stat_rq_pi_db_rdata_p    <= 'd0;

            s_axi_stat_qpn_rdata_p2        <= 'd0;
            s_axi_curr_sqptr_proc_rdata_p2 <= 'd0;
            s_axi_stat_rq_pi_db_rdata_p2   <= 'd0;

            s_axi_stat_qpn_rdata_r         <= 'd0;
            s_axi_curr_sqptr_proc_rdata_r  <= 'd0;
            s_axi_stat_rq_pi_db_rdata_r    <= 'd0;
            rd_addr_p1                     <= 'd0;
            rd_addr_p2                     <= 'd0;
            rd_addr_r1                     <= 'd0;
            rd_req_r1                      <= 1'b0;
            rd_req_p1                      <= 1'b0;
            rd_req_p2                      <= 1'b0;
            rd_req_p3                      <= 1'b0;
        end else begin

            rd_addr_p1 <= rd_addr_r;
            rd_addr_p2 <= rd_addr_p1;
            if (C_CONFIG_MEM_LATENCY == 3) begin
                rd_addr_r1 <= rd_addr_p1;
            end else begin
                rd_addr_r1 <= rd_addr_p2;
            end

            if (rd_addr_c1[20:19] == 2'b10) begin  //global registers
                mux_select <= 3'b001;
            end else if (rd_addr_c1[20:19] == 2'b11) begin  // per QP registers
                mux_select <= 3'b010;
            end else begin
                mux_select <= 3'b011;
            end

            case (rd_addr_c1[10:2])
                LRDMA_CONF_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {8'h00, lrdma_conf_udp_src_port, lrdma_conf_flow_credits, lrdma_conf_err_buf_en, 2'b00, 1'b0, 1'b0, lrdma_conf_lrdma_en};
                    read_mux[0]      <= 1'b1;
                end
                LRDMA_ADV_CONF_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {{1'b0, {(11 - C_QP_INDX_WIDTH) {1'b0}}, lrdma_adv_conf_sw_override_qpid}, 17'h0, lrdma_adv_conf_retry_cnt_fatal_dis, 1'b0, lrdma_adv_conf_sw_override};
                    read_mux[0]      <= 1'b1;
                end
                LRDMA_BUF_THRESHOLD_ROCE_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {roce_pause_off_threshold_reg, roce_pause_on_threshold_reg};
                    read_mux[0]      <= 1'b1;
                end
                LRDMA_PAUSE_CONF_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {18'h0, disable_priority_chk_reg, 1'b0, non_roce_priority_reg, roce_priority_reg, 2'b0, pfc_en_non_roce_reg, pfc_en_roce_reg};
                    read_mux[0]      <= 1'b1;
                end
                MAC_LRDMA_ADDR_LSB_REG[10:2]: begin
                    s_axi_lite_reg_1 <= mac_lrdma_addr_lsb;
                    read_mux[0]      <= 1'b1;
                end
                MAC_LRDMA_ADDR_MSB_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, mac_lrdma_addr_msb};
                    read_mux[0]      <= 1'b1;
                end
                IPV6_LRDMA_ADDR1_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv6_lrdma_addr1;
                    read_mux[0]      <= 1'b1;
                end
                IPV6_LRDMA_ADDR2_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv6_lrdma_addr2;
                    read_mux[0]      <= 1'b1;
                end
                IPV6_LRDMA_ADDR3_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv6_lrdma_addr3;
                    read_mux[0]      <= 1'b1;
                end
                IPV6_LRDMA_ADDR4_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv6_lrdma_addr4;
                    read_mux[0]      <= 1'b1;
                end
                RQ_ERR_PKT_BUF_BA_REG[10:2]: begin
                    s_axi_lite_reg_1 <= rq_err_pkt_buf_ba;
                    read_mux[0]      <= 1'b1;
                end
                RQ_ERR_PKT_BUF_BA_REG_MSB[10:2]: begin
                    s_axi_lite_reg_1 <= rq_err_pkt_buf_ba_msb;
                    read_mux[0]      <= 1'b1;
                end
                RQ_ERR_PKT_BUF_SZ_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {rq_err_pkt_buf_sz_buf_sz, rq_err_pkt_buf_sz_num_bufs};
                    read_mux[0]      <= 1'b1;
                end
                RQ_ERR_PKT_BUF_WRPTR_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, i_rq_err_pkt_buf_wrptr};
                    read_mux[0]      <= 1'b1;
                end
                IPV4_LRDMA_ADDR_REG[10:2]: begin
                    s_axi_lite_reg_1 <= ipv4_lrdma_addr;
                    read_mux[0]      <= 1'b1;
                end
                IN_ERRSTS_Q_BA_REG[10:2]: begin
                    s_axi_lite_reg_1 <= in_errsts_q_ba;
                    read_mux[0]      <= 1'b1;
                end
                IN_ERRSTS_Q_BA_REG_MSB[10:2]: begin
                    s_axi_lite_reg_1 <= in_errsts_q_ba_msb;
                    read_mux[0]      <= 1'b1;
                end
                IN_ERRSTS_Q_SZ_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, in_errsts_q_sz};
                    read_mux[0]      <= 1'b1;
                end
                IN_ERRSTS_Q_WRPTR_REG[10:2]: begin
                    s_axi_lite_reg_1 <= {16'b0, i_in_errsts_q_wrptr};
                    read_mux[0]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_1 <= 'd0;
                    read_mux[0]      <= 1'b0;
                end
            endcase

            case (rd_addr_c1[10:2])
                INC_MA_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {i_inc_mad_pkt_cnt, i_inc_ack_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                OUT_WR_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {16'b0, i_out_wr_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                OUT_MAD_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {i_out_mad_pkt_cnt, 16'd0};
                    read_mux[1]      <= 1'b1;
                end
                LAST_IN_PKT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_last_in_pkt_info;  //{last_in_pkt_psn_lsb, last_in_pkt_qpid, last_in_pkt_opcode};
                    read_mux[1]      <= 1'b1;
                end
                LAST_OUT_PKT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_last_out_pkt_info;  //{last_out_pkt_psn_lsb, last_out_pkt_qpid, last_out_pkt_opcode};
                    read_mux[1]      <= 1'b1;
                end
                INV_DUP_PKT_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {16'd0, i_inc_inv_pkt_cnt};
                    read_mux[1]      <= 1'b1;
                end
                TX_ENGINE_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_tx_engine_sts;
                    read_mux[1]      <= 1'b1;
                end
                QP_MGR_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_qp_mgr_sts;
                    read_mux[1]      <= 1'b1;
                end
                INC_ALL_DROPPED_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_inc_all_dropped_cnt;
                    read_mux[1]      <= 1'b1;
                end
                INC_NACK_CNT_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {16'h0, i_inc_nack_cnt};
                    read_mux[1]      <= 1'b1;
                end
                ACK_MGR_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_ack_mgr_sts;
                    read_mux[1]      <= 1'b1;
                end
                RETRY_CNT_STS_REG[10:2]: begin
                    s_axi_lite_reg_2 <= i_stat_retry_cnt;
                    read_mux[1]      <= 1'b1;
                end
                DBG_CNT_OVERRIDE_REG[10:2]: begin
                    s_axi_lite_reg_2 <= {8'h0, i_axi_rd_err_cnt, lrdma_sw_override_cnt};
                    read_mux[1]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_2 <= 'd0;
                    read_mux[1]      <= 1'b0;
                end
            endcase

            case (rd_addr_c1[10:2])
                INTR_EN_REG[10:2]: begin
                    s_axi_lite_reg_3 <= {28'b0, intr_en_fatal_err, intr_en_wqe_cmpl, intr_en_mad_pkt_rcvd, intr_en_pkt_valdn_err};
                    read_mux[2]      <= 1'b1;
                end
                INTR_STS_REG[10:2]: begin
                    s_axi_lite_reg_3 <= {28'b0, qp_fatal_sts, i_qp_wq_cmpl_intr_or_reduce, i_mad_pkt_rcvd_intr, i_pkt_valdn_err_intr};
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS1_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd0);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS2_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd1);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS3_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd2);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS4_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd3);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS5_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd4);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS6_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd5);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS7_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd6);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS8_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd7);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS9_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd8);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS10_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd9);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS11_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd10);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS12_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd11);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS13_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd12);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS14_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd13);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS15_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd14);
                    read_mux[2]      <= 1'b1;
                end
                CQ_INTR_STS16_REG[10:2]: begin
                    s_axi_lite_reg_3 <= cq_intr_sts_read_bank(6'd15);
                    read_mux[2]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_3 <= 'd0;
                    read_mux[2]      <= 1'b0;
                end
            endcase

            case (rd_addr_c1[10:2])
                CQ_INTR_STS17_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd16);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS18_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd17);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS19_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd18);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS20_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd19);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS21_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd20);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS22_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd21);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS23_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd22);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS24_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd23);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS25_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd24);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS26_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd25);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS27_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd26);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS28_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd27);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS29_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd28);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS30_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd29);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS31_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd30);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS32_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd31);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS33_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd32);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS34_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd33);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS35_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd34);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS36_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd35);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS37_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd36);
                    read_mux[3]      <= 1'b1;
                end
                CQ_INTR_STS38_REG[10:2]: begin
                    s_axi_lite_reg_4 <= cq_intr_sts_read_bank(6'd37);
                    read_mux[3]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_4 <= 'd0;
                    read_mux[3]      <= 1'b0;
                end
            endcase

            // case for additional global interrupt registers 
            case (rd_addr_c1[10:2])
                CQ_INTR_STS39_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd38);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS40_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd39);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS41_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd40);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS42_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd41);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS43_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd42);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS44_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd43);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS45_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd44);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS46_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd45);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS47_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd46);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS48_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd47);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS49_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd48);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS50_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd49);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS51_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd50);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS52_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd51);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS53_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd52);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS54_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd53);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS55_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd54);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS56_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd55);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS57_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd56);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS58_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd57);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS59_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd58);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS60_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd59);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS61_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd60);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS62_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd61);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS63_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd62);
                    read_mux[4]      <= 1'b1;
                end
                CQ_INTR_STS64_REG[10:2]: begin
                    s_axi_lite_reg_5 <= cq_intr_sts_read_bank(6'd63);
                    read_mux[4]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_5 <= 'd0;
                    read_mux[4]      <= 1'b0;
                end

            endcase

            case (rd_addr_c1[10:2])
                LRDMA_CONF_QP_EN_REG[10:2]: begin
                    s_axi_lite_reg_6 <= {{(31 - C_QP_INDX_WIDTH) {1'b0}}, lrdma_conf_num_qps_enabled};
                    read_mux[5]      <= 1'b1;
                end
                default: begin
                    s_axi_lite_reg_6 <= 'd0;
                    read_mux[5]      <= 1'b0;
                end

            endcase

            s_axi_lite_reg_1_r             <= s_axi_lite_reg_1;
            s_axi_lite_reg_2_r             <= s_axi_lite_reg_2;
            s_axi_lite_reg_3_r             <= s_axi_lite_reg_3;
            s_axi_lite_reg_4_r             <= s_axi_lite_reg_4;
            s_axi_lite_reg_5_r             <= s_axi_lite_reg_5;
            s_axi_lite_reg_6_r             <= s_axi_lite_reg_6;

            s_axi_curr_sqptr_proc_rdata_p  <= s_axi_curr_sqptr_proc_rdata;
            s_axi_stat_rq_pi_db_rdata_p    <= s_axi_stat_rq_pi_db_rdata;

            s_axi_stat_qpn_rdata_p1        <= s_axi_stat_qpn_rdata;
            s_axi_curr_sqptr_proc_rdata_p1 <= s_axi_curr_sqptr_proc_rdata_p;
            s_axi_stat_rq_pi_db_rdata_p1   <= s_axi_stat_rq_pi_db_rdata_p;

            s_axi_stat_qpn_rdata_p2        <= s_axi_stat_qpn_rdata_p1;
            s_axi_curr_sqptr_proc_rdata_p2 <= s_axi_curr_sqptr_proc_rdata_p1;
            s_axi_stat_rq_pi_db_rdata_p2   <= s_axi_stat_rq_pi_db_rdata_p1;
            if (C_CONFIG_MEM_LATENCY == 3) begin  // 配置读数据延迟为 3 拍时，输出再打一拍

                s_axi_stat_qpn_rdata_r        <= s_axi_stat_qpn_rdata;
                s_axi_curr_sqptr_proc_rdata_r <= s_axi_curr_sqptr_proc_rdata_p;
                s_axi_stat_rq_pi_db_rdata_r   <= s_axi_stat_rq_pi_db_rdata_p;
            end else begin

                s_axi_stat_qpn_rdata_r        <= s_axi_stat_qpn_rdata_p2;
                s_axi_curr_sqptr_proc_rdata_r <= s_axi_curr_sqptr_proc_rdata_p2;
                s_axi_stat_rq_pi_db_rdata_r   <= s_axi_stat_rq_pi_db_rdata_p2;
            end

            if (s_axi_lite_rvalid && s_axi_lite_rready) begin
                rd_req_r1 <= 1'b0;
                rd_req_p1 <= 1'b0;
                rd_req_p2 <= 1'b0;
                rd_req_p3 <= 1'b0;
            end else begin
                rd_req_p1 <= rd_req_r;
                rd_req_p2 <= rd_req_p1;
                rd_req_p3 <= rd_req_p2;
                if (C_CONFIG_MEM_LATENCY == 3) begin
                    rd_req_r1 <= rd_req_p1;
                end else begin
                    rd_req_r1 <= rd_req_p3;
                end
            end
        end
    end

    assign s_axi_lite_reg_1_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_1 : s_axi_lite_reg_1_r;
    assign s_axi_lite_reg_2_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_2 : s_axi_lite_reg_2_r;
    assign s_axi_lite_reg_3_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_3 : s_axi_lite_reg_3_r;
    assign s_axi_lite_reg_4_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_4 : s_axi_lite_reg_4_r;
    assign s_axi_lite_reg_5_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_5 : s_axi_lite_reg_5_r;
    assign s_axi_lite_reg_6_c = (C_CONFIG_MEM_LATENCY == 3) ? s_axi_lite_reg_6 : s_axi_lite_reg_6_r;

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            s_axi_lite_rvalid <= 1'b0;
            s_axi_lite_rdata  <= 32'd0;
            rvalid_delayed    <= 1'b0;
            rd_addr_wen_r     <= 3'b000;
            axi_lite_rvalid_i <= 1'b0;
            rvalid_delayed_i  <= 1'b0;
            rd_req_mask       <= 1'b0;
        end else begin
            if (rd_req_r) begin
                if (axi_lite_rvalid_i && s_axi_lite_rready) begin
                    axi_lite_rvalid_i <= 1'b0;
                    rvalid_delayed_i  <= 1'b0;
                end else if (rd_req_mask == 1'b0) begin
                    axi_lite_rvalid_i <= rvalid_delayed_i;
                    rvalid_delayed_i  <= 1'b1;
                end
            end

            if (s_axi_lite_rvalid && s_axi_lite_rready) begin
                rd_req_mask <= 1'b0;
            end else if (axi_lite_rvalid_i == 1'b1) begin
                rd_req_mask <= 1'b1;
            end

            if (rd_req_r1) begin
                if (s_axi_lite_rvalid && s_axi_lite_rready) begin
                    s_axi_lite_rvalid <= 1'b0;
                    rvalid_delayed    <= 1'b0;
                end else begin
                    s_axi_lite_rvalid <= rvalid_delayed;
                    rvalid_delayed    <= 1'b1;
                end
                if (~s_axi_lite_rvalid & mux_select == 3'b001) begin
                    rd_addr_wen_r <= 3'b001;
                    case (read_mux)
                        6'b000_001: s_axi_lite_rdata <= s_axi_lite_reg_1_c;  //Remote offset lsb 32 bits 
                        6'b000_010: s_axi_lite_rdata <= s_axi_lite_reg_2_c;  //Remote Offset msb 32 bits
                        6'b000_100: s_axi_lite_rdata <= s_axi_lite_reg_3_c;  //Remote tag 
                        6'b001_000: s_axi_lite_rdata <= s_axi_lite_reg_4_c;  //Remote tag 
                        6'b010_000: s_axi_lite_rdata <= s_axi_lite_reg_5_c;
                        6'b100_000: s_axi_lite_rdata <= s_axi_lite_reg_6_c;
                        default: s_axi_lite_rdata <= 'b0;
                    endcase
                end else if (~s_axi_lite_rvalid & mux_select == 3'b010) begin
                    rd_addr_wen_r <= 3'b010;
                    case (rd_addr_r1[7:0])
                        QP_CONF_QPN:              s_axi_lite_rdata <= s_axi_qp_config_rdata;
                        QP_ADV_CONF_QPN:          s_axi_lite_rdata <= s_axi_qp_adv_config_rdata;
                        RQ_BUF_BA_QPN:            s_axi_lite_rdata <= s_axi_rq_ba_rdata;
                        SQ_BA_QPN:                s_axi_lite_rdata <= s_axi_sq_ba_rdata;
                        CQ_BA_QPN:                s_axi_lite_rdata <= s_axi_cq_ba_rdata;
                        RQ_BUF_BA_MSB_QPN:        s_axi_lite_rdata <= s_axi_rq_ba_msb_rdata;
                        SQ_BA_MSB_QPN:            s_axi_lite_rdata <= s_axi_sq_ba_msb_rdata;
                        CQ_BA_MSB_QPN:            s_axi_lite_rdata <= s_axi_cq_ba_msb_rdata;
                        RQ_WRPTR_DB_ADD_QPN:      s_axi_lite_rdata <= s_axi_rq_wrptrdb_add_rdata;
                        RQ_WRPTR_DB_ADD_QPN_MSB:  s_axi_lite_rdata <= s_axi_rq_wrptrdb_msb_add_rdata;
                        CQ_HEAD_QPN:              s_axi_lite_rdata <= s_axi_cq_hdptr_rdata;
                        RQ_CI_DB_QPN:             s_axi_lite_rdata <= s_axi_rq_cidb_rdata;
                        SQ_PI_DB_QPN:             s_axi_lite_rdata <= s_axi_sq_pidb_rdata;
                        Q_DEPTH_QPN:              s_axi_lite_rdata <= s_axi_q_depth_rdata;
                        SQ_PSN_QPN:               s_axi_lite_rdata <= s_axi_sq_psn_rdata;
                        LAST_REQ_QPN:             s_axi_lite_rdata <= s_axi_last_req_rdata;
                        DEST_QP_CONF_QPN:         s_axi_lite_rdata <= s_axi_dest_qpid_rdata;
                        TIMEOUT_QPN:              s_axi_lite_rdata <= s_axi_timeout_rdata;
                        MAC_REMOTE_ADDR_LSB_QPN:  s_axi_lite_rdata <= s_axi_mac_remote_addrl_rdata;
                        MAC_REMOTE_ADDR_MSB_QPN:  s_axi_lite_rdata <= s_axi_mac_remote_addrm_rdata;
                        IP_REMOTE_ADDR1_QPN:      s_axi_lite_rdata <= s_axi_ip_remote_addr1_rdata;
                        IP_REMOTE_ADDR2_QPN:      s_axi_lite_rdata <= s_axi_ip_remote_addr2_rdata;
                        IP_REMOTE_ADDR3_QPN:      s_axi_lite_rdata <= s_axi_ip_remote_addr3_rdata;
                        IP_REMOTE_ADDR4_QPN:      s_axi_lite_rdata <= s_axi_ip_remote_addr4_rdata;
                        STAT_SSN_QPN:             s_axi_lite_rdata <= s_axi_stat_ssn_rdata;
                        STAT_QPN:                 s_axi_lite_rdata <= s_axi_stat_qpn_rdata_r;  //check
                        STAT_CURR_SQPTR_PROC_QPN: s_axi_lite_rdata <= s_axi_curr_sqptr_proc_rdata_r;  //check
                        STAT_RESP_PSN_QPN:        s_axi_lite_rdata <= s_axi_stat_resp_psn_rdata;
                        STAT_RQ_BUF_CA:           s_axi_lite_rdata <= s_axi_stat_rq_buf_ca_rdata;
                        STAT_RQ_BUF_CA_MSB:       s_axi_lite_rdata <= s_axi_stat_rq_buf_ca_msb_rdata;
                        STAT_WQE_CNT:             s_axi_lite_rdata <= s_axi_stat_wqe_cnt_rdata;
                        STAT_RQ_PI_DB:            s_axi_lite_rdata <= s_axi_stat_rq_pi_db_rdata_r;  //check
                        STAT_RET_SQ_PSN:          s_axi_lite_rdata <= s_axi_stat_ret_sq_psn_rdata;
                        default:                  s_axi_lite_rdata <= 'h0;
                    endcase
                end else if (~s_axi_lite_rvalid & mux_select == 3'b011) begin
                    rd_addr_wen_r    <= 3'b011;
                    s_axi_lite_rdata <= 32'd0;
                end else if (s_axi_lite_rready) begin
                    rd_addr_wen_r    <= 3'b000;
                    s_axi_lite_rdata <= 'b0;
                end
            end
        end
    end


    //***************************************************************************
    // AXI-Lite 写事务处理逻辑。
    // 收到写事务后，根据地址位 [5:2] 选择目标寄存器并写入数据。
    // 地址位 [1:0] 不参与译码，访问应为字对齐。
    //***************************************************************************
    always @(posedge core_clk) begin
        if (~core_rstn) begin

            lrdma_conf_lrdma_en                <= 'b0;
            lrdma_conf_err_buf_en              <= 'b0;
            lrdma_conf_flow_credits            <= 'b0;
            lrdma_conf_num_qps_enabled         <= 'b0;
            lrdma_conf_udp_src_port            <= 'b0;
            lrdma_adv_conf_sw_override         <= 'b0;
            lrdma_adv_conf_sw_override_qpid    <= 'b0;
            lrdma_adv_conf_retry_cnt_fatal_dis <= 'b0;

            intr_en_pkt_valdn_err              <= 'b0;
            intr_en_wqe_cmpl                   <= 'b0;
            intr_en_mad_pkt_rcvd               <= 'b0;
            intr_en_fatal_err                  <= 'b0;
            cq_intr_sts_clr_wr_en              <= 'h0;
            cq_intr_sts_clr_wr_addr            <= 'h0;

            wr_req_r1                          <= 1'b0;
            wr_addr_r1                         <= 'd0;
            wr_data_r1                         <= 32'h0;
        end else begin
            if (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r & generic_reg_wr_access) begin
                wr_req_r1  <= 1'b1;
                wr_addr_r1 <= wr_addr_r;
                wr_data_r1 <= s_axi_lite_wdata;

                case (wr_addr_r[10:2])  // 忽略地址位 [1:0]，按字对齐访问

                    LRDMA_CONF_REG[10:2]: begin  // LRDMA Configuration -- 对应手册 0x10_0000，下列均是
                        lrdma_conf_lrdma_en     <= s_axi_lite_wdata[0];
                        lrdma_conf_err_buf_en   <= s_axi_lite_wdata[5];
                        lrdma_conf_flow_credits <= s_axi_lite_wdata[7:6];  // LRDMACONF[7:6] 读写位，无功能使用
                        lrdma_conf_udp_src_port <= s_axi_lite_wdata[23:8];
                        // [31:24] 无功能定义
                    end

                    LRDMA_CONF_QP_EN_REG[10:2]: begin  // LRDMA_CONF_QP_EN
                        lrdma_conf_num_qps_enabled <= s_axi_lite_wdata[C_QP_INDX_WIDTH:0];  // [10:0]: Number of QPs enabled
                        // [31:11] 无功能定义
                    end

                    LRDMA_ADV_CONF_REG[10:2]: begin  // LRDMA Advance Configuration
                        lrdma_adv_conf_sw_override         <= s_axi_lite_wdata[0];
                        // [1] 无功能定义
                        lrdma_adv_conf_retry_cnt_fatal_dis <= s_axi_lite_wdata[2];
                        // [19:3] 无功能定义
                        lrdma_adv_conf_sw_override_qpid    <= s_axi_lite_wdata[C_QP_INDX_WIDTH+19 : 20];  //2K QP's max support
                        // [31] 无功能定义
                    end

                    INTR_EN_REG[10:2]: begin  // Interrupt Enable (INTEN)
                        intr_en_pkt_valdn_err <= s_axi_lite_wdata[0];
                        intr_en_mad_pkt_rcvd  <= s_axi_lite_wdata[1];
                        intr_en_wqe_cmpl      <= s_axi_lite_wdata[2];
                        intr_en_fatal_err     <= s_axi_lite_wdata[3];
                    end

                    // Completion Q interrupt status (bitwise) QP 0-31 (CQINTSTS1) ...
                    CQ_INTR_STS1_REG[10:2]: begin  // Completion Q interrupt status (bitwise) QP 0-31 (CQINTSTS1)
                        cq_intr_sts_clear_bank(6'd0);
                    end
                    CQ_INTR_STS2_REG[10:2]: begin  // Completion Q interrupt status (bitwise) QP 32-63 (CQINTSTS2)
                        cq_intr_sts_clear_bank(6'd1);
                    end
                    CQ_INTR_STS3_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd2);
                    end
                    CQ_INTR_STS4_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd3);
                    end
                    CQ_INTR_STS5_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd4);
                    end
                    CQ_INTR_STS6_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd5);
                    end
                    CQ_INTR_STS7_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd6);
                    end
                    CQ_INTR_STS8_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd7);
                    end
                    CQ_INTR_STS9_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd8);
                    end
                    CQ_INTR_STS10_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd9);
                    end
                    CQ_INTR_STS11_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd10);
                    end
                    CQ_INTR_STS12_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd11);
                    end
                    CQ_INTR_STS13_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd12);
                    end
                    CQ_INTR_STS14_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd13);
                    end
                    CQ_INTR_STS15_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd14);
                    end
                    CQ_INTR_STS16_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd15);
                    end
                    CQ_INTR_STS17_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd16);
                    end
                    CQ_INTR_STS18_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd17);
                    end
                    CQ_INTR_STS19_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd18);
                    end
                    CQ_INTR_STS20_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd19);
                    end
                    CQ_INTR_STS21_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd20);
                    end
                    CQ_INTR_STS22_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd21);
                    end
                    CQ_INTR_STS23_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd22);
                    end
                    CQ_INTR_STS24_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd23);
                    end
                    CQ_INTR_STS25_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd24);
                    end
                    CQ_INTR_STS26_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd25);
                    end
                    CQ_INTR_STS27_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd26);
                    end
                    CQ_INTR_STS28_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd27);
                    end
                    CQ_INTR_STS29_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd28);
                    end
                    CQ_INTR_STS30_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd29);
                    end
                    CQ_INTR_STS31_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd30);
                    end
                    CQ_INTR_STS32_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd31);
                    end
                    CQ_INTR_STS33_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd32);
                    end
                    CQ_INTR_STS34_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd33);
                    end
                    CQ_INTR_STS35_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd34);
                    end
                    CQ_INTR_STS36_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd35);
                    end
                    CQ_INTR_STS37_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd36);
                    end
                    CQ_INTR_STS38_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd37);
                    end
                    CQ_INTR_STS39_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd38);
                    end
                    CQ_INTR_STS40_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd39);
                    end
                    CQ_INTR_STS41_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd40);
                    end
                    CQ_INTR_STS42_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd41);
                    end
                    CQ_INTR_STS43_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd42);
                    end
                    CQ_INTR_STS44_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd43);
                    end
                    CQ_INTR_STS45_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd44);
                    end
                    CQ_INTR_STS46_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd45);
                    end
                    CQ_INTR_STS47_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd46);
                    end
                    CQ_INTR_STS48_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd47);
                    end
                    CQ_INTR_STS49_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd48);
                    end
                    CQ_INTR_STS50_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd49);
                    end
                    CQ_INTR_STS51_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd50);
                    end
                    CQ_INTR_STS52_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd51);
                    end
                    CQ_INTR_STS53_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd52);
                    end
                    CQ_INTR_STS54_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd53);
                    end
                    CQ_INTR_STS55_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd54);
                    end
                    CQ_INTR_STS56_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd55);
                    end
                    CQ_INTR_STS57_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd56);
                    end
                    CQ_INTR_STS58_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd57);
                    end
                    CQ_INTR_STS59_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd58);
                    end
                    CQ_INTR_STS60_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd59);
                    end
                    CQ_INTR_STS61_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd60);
                    end
                    CQ_INTR_STS62_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd61);
                    end
                    CQ_INTR_STS63_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd62);
                    end
                    CQ_INTR_STS64_REG[10:2]: begin
                        cq_intr_sts_clear_bank(6'd63);
                    end
                endcase
            end else begin
                wr_req_r1               <= 1'b0;
                cq_intr_sts_clr_wr_en   <= 'd0;
                cq_intr_sts_clr_wr_addr <= 'd0;
            end
        end
    end

    // 对初始配置进行额外注册，以尽量减少时序路径
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            mac_lrdma_addr_lsb           <= 'b0;
            mac_lrdma_addr_msb           <= 'b0;
            roce_pause_on_threshold_reg  <= 16'd90;
            roce_pause_off_threshold_reg <= 16'd150;
            pfc_en_roce_reg              <= 1'b0;
            pfc_en_non_roce_reg          <= 1'b0;
            roce_priority_reg            <= 4'h0;
            non_roce_priority_reg        <= 4'h0;
            disable_priority_chk_reg     <= 1'b1;

            ipv4_lrdma_addr              <= 'b0;
            ipv6_lrdma_addr1             <= 'b0;
            ipv6_lrdma_addr2             <= 'b0;
            ipv6_lrdma_addr3             <= 'b0;
            ipv6_lrdma_addr4             <= 'b0;

            rq_err_pkt_buf_ba            <= 'b0;
            rq_err_pkt_buf_ba_msb        <= 'b0;
            rq_err_pkt_buf_sz_num_bufs   <= 'b0;
            rq_err_pkt_buf_sz_buf_sz     <= 'b0;

            in_errsts_q_ba               <= 'b0;
            in_errsts_q_ba_msb           <= 'b0;
            in_errsts_q_sz               <= 'b0;

        end else begin

            if (wr_req_r1) begin
                case (wr_addr_r1[10:2])
                    // 对应手册 0x10_0010，下列均是
                    MAC_LRDMA_ADDR_LSB_REG[10:2]: mac_lrdma_addr_lsb <= wr_data_r1[31:0];  // MAC LRDMA Address LSB(MACXADDLSB)
                    MAC_LRDMA_ADDR_MSB_REG[10:2]: mac_lrdma_addr_msb <= wr_data_r1[15:0];  // MAC LRDMA Address MSB(MACXADDMSB)
                    LRDMA_BUF_THRESHOLD_ROCE_REG[10:2]: begin  // LRDMA_BUF_THRESHOLD_ROCE
                        roce_pause_on_threshold_reg  <= wr_data_r1[15:0];
                        roce_pause_off_threshold_reg <= wr_data_r1[31:16];
                    end

                    LRDMA_PAUSE_CONF_REG[10:2]: begin  // LRDMA_PAUSE_CONF
                        pfc_en_roce_reg          <= wr_data_r1[0];  // 默认0
                        pfc_en_non_roce_reg      <= wr_data_r1[1];  // 默认0
                        roce_priority_reg        <= wr_data_r1[7:4];  // 默认0
                        non_roce_priority_reg    <= wr_data_r1[11:8];  // 默认0
                        disable_priority_chk_reg <= wr_data_r1[13];  // 默认1
                    end

                    IPV4_LRDMA_ADDR_REG[10:2]: begin  // IPv4 LRDMA Address(IPv4XADD)
                        ipv4_lrdma_addr <= wr_data_r1[31:0];
                    end

                    IPV6_LRDMA_ADDR1_REG[10:2]: ipv6_lrdma_addr1 <= wr_data_r1[31:0];  // IPv6 Address 1(IPv6XADD1)
                    IPV6_LRDMA_ADDR2_REG[10:2]: ipv6_lrdma_addr2 <= wr_data_r1[31:0];  // IPv6 Address 2(IPv6XADD2)
                    IPV6_LRDMA_ADDR3_REG[10:2]: ipv6_lrdma_addr3 <= wr_data_r1[31:0];  // IPv6 Address 3(IPv6XADD3)
                    IPV6_LRDMA_ADDR4_REG[10:2]: ipv6_lrdma_addr4 <= wr_data_r1[31:0];  // IPv6 Address 4(IPv6XADD4)

                    RQ_ERR_PKT_BUF_BA_REG[10:2]:     rq_err_pkt_buf_ba <= wr_data_r1[31:0];  // Error Buffer Base Address(ERRBUFBA)
                    RQ_ERR_PKT_BUF_BA_REG_MSB[10:2]: rq_err_pkt_buf_ba_msb <= wr_data_r1[31:0];  // Error Buffer Base Address msb(ERRBUFBAMSB)
                    RQ_ERR_PKT_BUF_SZ_REG[10:2]: begin  // Error Buffer Size(ERRBUFSZ)
                        rq_err_pkt_buf_sz_num_bufs <= wr_data_r1[15:0];
                        rq_err_pkt_buf_sz_buf_sz   <= wr_data_r1[31:16];
                    end

                    IN_ERRSTS_Q_BA_REG[10:2]:     in_errsts_q_ba <= wr_data_r1[31:0];  // Incoming Pkt Error Status Queue Base address(IPKTERRQBA)
                    IN_ERRSTS_Q_BA_REG_MSB[10:2]: in_errsts_q_ba_msb <= wr_data_r1[31:0];  // Incoming Pkt Error Status Queue Base address MSB(IPKTERRQBAMSB)
                    IN_ERRSTS_Q_SZ_REG[10:2]:     in_errsts_q_sz <= wr_data_r1[15:0];  // Incoming Pkt Error Status Queue Size(IPKTERRQSZ)
                endcase
            end
        end
    end


    // 对初始配置进行额外注册，以尽量减少时序路径
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            qp_conf_bram_wen_r               <= 'b0;
            qp_adv_conf_bram_wen_r           <= 'b0;
            qp_rq_ba_bram_wen_r              <= 'b0;
            qp_rq_ba_msb_bram_wen_r          <= 'b0;
            qp_sq_ba_bram_wen_r              <= 'b0;
            qp_sq_ba_msb_bram_wen_r          <= 'b0;
            qp_cq_ba_bram_wen_r              <= 'b0;
            qp_cq_ba_msb_bram_wen_r          <= 'b0;
            qp_rq_wrptrdb_add_bram_wen_r     <= 'b0;
            qp_rq_wrptrdb_add_msb_bram_wen_r <= 'b0;
            qp_rq_cidb_bram_wen_r            <= 'b0;
            qp_q_depth_bram_wen_r            <= 'b0;

            qp_last_rq_bram_wen_r            <= 'b0;
            qp_dest_qpid_bram_wen_r          <= 'b0;
            qp_mac_remote_addrl_bram_wen_r   <= 'b0;
            qp_mac_remote_addrm_bram_wen_r   <= 'b0;
            qp_ip_remote_addr1_bram_wen_r    <= 'b0;
            qp_ip_remote_addr2_bram_wen_r    <= 'b0;
            qp_ip_remote_addr3_bram_wen_r    <= 'b0;
            qp_ip_remote_addr4_bram_wen_r    <= 'b0;
            qp_stat_ssn_bram_wen_r           <= 'b0;
            qp_timeout_bram_wen_r            <= 'b0;
            wr_addr_wen_r                    <= 'b0;
            wr_addr_mem_r_r                  <= 'b0;
            rd_addr_mem_r_r                  <= 'b0;
            wr_data_mem_r_r                  <= 'b0;
        end else begin

            //wr_addr_mem_r  <= wr_addr_r;
            rd_addr_mem_r_r <= rd_addr_r[8+:C_QP_INDX_WIDTH];
            if (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) begin
                wr_addr_mem_r_r <= wr_addr_r[8+:C_QP_INDX_WIDTH];
                wr_data_mem_r_r <= s_axi_lite_wdata[31:0];
            end else begin
                wr_addr_mem_r_r <= rd_addr_r[8+:C_QP_INDX_WIDTH];
            end

            // wr_addr_r[20:19] == 2'b11 表示地址是 0x18 开头，即 0x18_0000 开始
            if ((wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && wr_addr_r[20:19] == 2'b11) begin
                wr_addr_wen_r                    <= 2'b01;
                qp_conf_bram_wen_r               <= 'b0;
                qp_adv_conf_bram_wen_r           <= 'b0;
                qp_rq_ba_bram_wen_r              <= 'b0;
                qp_sq_ba_bram_wen_r              <= 'b0;
                qp_cq_ba_bram_wen_r              <= 'b0;
                qp_rq_ba_msb_bram_wen_r          <= 'b0;
                qp_sq_ba_msb_bram_wen_r          <= 'b0;
                qp_cq_ba_msb_bram_wen_r          <= 'b0;
                qp_rq_wrptrdb_add_bram_wen_r     <= 'b0;
                qp_rq_wrptrdb_add_msb_bram_wen_r <= 'b0;
                qp_rq_cidb_bram_wen_r            <= 'b0;
                qp_q_depth_bram_wen_r            <= 'b0;

                qp_last_rq_bram_wen_r            <= 'b0;
                qp_dest_qpid_bram_wen_r          <= 'b0;
                qp_timeout_bram_wen_r            <= 'b0;
                qp_mac_remote_addrl_bram_wen_r   <= 'b0;
                qp_mac_remote_addrm_bram_wen_r   <= 'b0;
                qp_ip_remote_addr1_bram_wen_r    <= 'b0;
                qp_ip_remote_addr2_bram_wen_r    <= 'b0;
                qp_ip_remote_addr3_bram_wen_r    <= 'b0;
                qp_ip_remote_addr4_bram_wen_r    <= 'b0;
                qp_stat_ssn_bram_wen_r           <= 'b0;

                case (wr_addr_r[7:0])
                    QP_CONF_QPN:             qp_conf_bram_wen_r <= 1'b1;
                    QP_ADV_CONF_QPN:         qp_adv_conf_bram_wen_r <= 1'b1;
                    RQ_BUF_BA_QPN:           qp_rq_ba_bram_wen_r <= 1'b1;
                    SQ_BA_QPN:               qp_sq_ba_bram_wen_r <= 1'b1;
                    CQ_BA_QPN:               qp_cq_ba_bram_wen_r <= 1'b1;
                    RQ_BUF_BA_MSB_QPN:       qp_rq_ba_msb_bram_wen_r <= 1'b1;
                    SQ_BA_MSB_QPN:           qp_sq_ba_msb_bram_wen_r <= 1'b1;
                    CQ_BA_MSB_QPN:           qp_cq_ba_msb_bram_wen_r <= 1'b1;
                    RQ_WRPTR_DB_ADD_QPN:     qp_rq_wrptrdb_add_bram_wen_r <= 1'b1;
                    RQ_WRPTR_DB_ADD_QPN_MSB: qp_rq_wrptrdb_add_msb_bram_wen_r <= 1'b1;
                    RQ_CI_DB_QPN:            qp_rq_cidb_bram_wen_r <= 1'b1;
                    Q_DEPTH_QPN:             qp_q_depth_bram_wen_r <= 1'b1;
                    LAST_REQ_QPN:            qp_last_rq_bram_wen_r <= 1'b1;
                    DEST_QP_CONF_QPN:        qp_dest_qpid_bram_wen_r <= 1'b1;  //shift
                    TIMEOUT_QPN:             qp_timeout_bram_wen_r <= 1'b1;
                    MAC_REMOTE_ADDR_LSB_QPN: qp_mac_remote_addrl_bram_wen_r <= 1'b1;
                    MAC_REMOTE_ADDR_MSB_QPN: qp_mac_remote_addrm_bram_wen_r <= 1'b1;
                    IP_REMOTE_ADDR1_QPN:     qp_ip_remote_addr1_bram_wen_r <= 1'b1;
                    IP_REMOTE_ADDR2_QPN:     qp_ip_remote_addr2_bram_wen_r <= 1'b1;
                    IP_REMOTE_ADDR3_QPN:     qp_ip_remote_addr3_bram_wen_r <= 1'b1;
                    IP_REMOTE_ADDR4_QPN:     qp_ip_remote_addr4_bram_wen_r <= 1'b1;
                    STAT_SSN_QPN:            qp_stat_ssn_bram_wen_r <= 1'b1;
                endcase
                //end else if ((wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && (wr_addr_r[17] == 'b0)) begin

                // wr_addr_r[20] == 1'b0 表示地址是 0x00 开头，即 0x00_0000 开始
            end else if ((wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && (wr_addr_r[20] == 1'b0)) begin
                wr_addr_wen_r                    <= 2'b10;
                qp_conf_bram_wen_r               <= 'b0;
                qp_adv_conf_bram_wen_r           <= 'b0;
                qp_rq_ba_bram_wen_r              <= 'b0;
                qp_sq_ba_bram_wen_r              <= 'b0;
                qp_cq_ba_bram_wen_r              <= 'b0;
                qp_rq_ba_msb_bram_wen_r          <= 'b0;
                qp_sq_ba_msb_bram_wen_r          <= 'b0;
                qp_cq_ba_msb_bram_wen_r          <= 'b0;
                qp_rq_wrptrdb_add_bram_wen_r     <= 'b0;
                qp_rq_wrptrdb_add_msb_bram_wen_r <= 'b0;
                qp_rq_cidb_bram_wen_r            <= 'b0;
                qp_q_depth_bram_wen_r            <= 'b0;

                qp_last_rq_bram_wen_r            <= 'b0;
                qp_dest_qpid_bram_wen_r          <= 'b0;
                qp_timeout_bram_wen_r            <= 'b0;
                qp_mac_remote_addrl_bram_wen_r   <= 'b0;
                qp_mac_remote_addrm_bram_wen_r   <= 'b0;
                qp_ip_remote_addr1_bram_wen_r    <= 'b0;
                qp_ip_remote_addr2_bram_wen_r    <= 'b0;
                qp_ip_remote_addr3_bram_wen_r    <= 'b0;
                qp_ip_remote_addr4_bram_wen_r    <= 'b0;
                qp_stat_ssn_bram_wen_r           <= 'b0;

                // Removed register writes are intentionally no-ops.
            end else begin
                wr_addr_wen_r                    <= 2'b00;
                qp_conf_bram_wen_r               <= 'b0;
                qp_adv_conf_bram_wen_r           <= 'b0;
                qp_rq_ba_bram_wen_r              <= 'b0;
                qp_rq_ba_msb_bram_wen_r          <= 'b0;
                qp_sq_ba_bram_wen_r              <= 'b0;
                qp_sq_ba_msb_bram_wen_r          <= 'b0;
                qp_cq_ba_bram_wen_r              <= 'b0;
                qp_cq_ba_msb_bram_wen_r          <= 'b0;
                qp_rq_wrptrdb_add_bram_wen_r     <= 'b0;
                qp_rq_wrptrdb_add_msb_bram_wen_r <= 'b0;
                qp_rq_cidb_bram_wen_r            <= 'b0;
                qp_q_depth_bram_wen_r            <= 'b0;

                qp_last_rq_bram_wen_r            <= 'b0;
                qp_dest_qpid_bram_wen_r          <= 'b0;
                qp_timeout_bram_wen_r            <= 'b0;
                qp_mac_remote_addrl_bram_wen_r   <= 'b0;
                qp_mac_remote_addrm_bram_wen_r   <= 'b0;
                qp_ip_remote_addr1_bram_wen_r    <= 'b0;
                qp_ip_remote_addr2_bram_wen_r    <= 'b0;
                qp_ip_remote_addr3_bram_wen_r    <= 'b0;
                qp_ip_remote_addr4_bram_wen_r    <= 'b0;
                qp_stat_ssn_bram_wen_r           <= 'b0;

            end
        end
    end
    // 对初始配置进行额外注册，以尽量减少时序路径
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            qp_conf_bram_wen_p               <= 'b0;
            qp_adv_conf_bram_wen_p           <= 'b0;
            qp_rq_ba_bram_wen_p              <= 'b0;
            qp_rq_ba_msb_bram_wen_p          <= 'b0;
            qp_sq_ba_bram_wen_p              <= 'b0;
            qp_sq_ba_msb_bram_wen_p          <= 'b0;
            qp_cq_ba_bram_wen_p              <= 'b0;
            qp_cq_ba_msb_bram_wen_p          <= 'b0;
            qp_rq_wrptrdb_add_bram_wen_p     <= 'b0;
            qp_rq_wrptrdb_add_msb_bram_wen_p <= 'b0;
            qp_rq_cidb_bram_wen_p            <= 'b0;
            qp_q_depth_bram_wen_p            <= 'b0;

            qp_last_rq_bram_wen_p            <= 'b0;
            qp_dest_qpid_bram_wen_p          <= 'b0;
            qp_mac_remote_addrl_bram_wen_p   <= 'b0;
            qp_mac_remote_addrm_bram_wen_p   <= 'b0;
            qp_ip_remote_addr1_bram_wen_p    <= 'b0;
            qp_ip_remote_addr2_bram_wen_p    <= 'b0;
            qp_ip_remote_addr3_bram_wen_p    <= 'b0;
            qp_ip_remote_addr4_bram_wen_p    <= 'b0;
            qp_stat_ssn_bram_wen_p           <= 'b0;
            qp_timeout_bram_wen_p            <= 'b0;
            wr_addr_mem_r_p                  <= 'b0;
            wr_data_mem_r_p                  <= 'b0;
        end else begin
            qp_conf_bram_wen_p               <= qp_conf_bram_wen_r;
            qp_adv_conf_bram_wen_p           <= qp_adv_conf_bram_wen_r;
            qp_rq_ba_bram_wen_p              <= qp_rq_ba_bram_wen_r;
            qp_rq_ba_msb_bram_wen_p          <= qp_rq_ba_msb_bram_wen_r;
            qp_sq_ba_bram_wen_p              <= qp_sq_ba_bram_wen_r;
            qp_sq_ba_msb_bram_wen_p          <= qp_sq_ba_msb_bram_wen_r;
            qp_cq_ba_bram_wen_p              <= qp_cq_ba_bram_wen_r;
            qp_cq_ba_msb_bram_wen_p          <= qp_cq_ba_msb_bram_wen_r;
            qp_rq_wrptrdb_add_bram_wen_p     <= qp_rq_wrptrdb_add_bram_wen_r;
            qp_rq_wrptrdb_add_msb_bram_wen_p <= qp_rq_wrptrdb_add_msb_bram_wen_r;
            qp_rq_cidb_bram_wen_p            <= qp_rq_cidb_bram_wen_r;
            qp_q_depth_bram_wen_p            <= qp_q_depth_bram_wen_r;

            qp_last_rq_bram_wen_p            <= qp_last_rq_bram_wen_r;
            qp_dest_qpid_bram_wen_p          <= qp_dest_qpid_bram_wen_r;
            qp_mac_remote_addrl_bram_wen_p   <= qp_mac_remote_addrl_bram_wen_r;
            qp_mac_remote_addrm_bram_wen_p   <= qp_mac_remote_addrm_bram_wen_r;
            qp_ip_remote_addr1_bram_wen_p    <= qp_ip_remote_addr1_bram_wen_r;
            qp_ip_remote_addr2_bram_wen_p    <= qp_ip_remote_addr2_bram_wen_r;
            qp_ip_remote_addr3_bram_wen_p    <= qp_ip_remote_addr3_bram_wen_r;
            qp_ip_remote_addr4_bram_wen_p    <= qp_ip_remote_addr4_bram_wen_r;
            qp_stat_ssn_bram_wen_p           <= qp_stat_ssn_bram_wen_r;
            qp_timeout_bram_wen_p            <= qp_timeout_bram_wen_r;
            wr_addr_mem_r_p                  <= wr_addr_mem_r_r;
            wr_data_mem_r_p                  <= wr_data_mem_r_r;
        end
    end

    assign qp_conf_bram_wen               = (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_bram_wen_r : qp_conf_bram_wen_p;
    assign qp_adv_conf_bram_wen           = (C_CONFIG_MEM_LATENCY == 3) ? qp_adv_conf_bram_wen_r : qp_adv_conf_bram_wen_p;
    assign qp_rq_ba_bram_wen              = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_ba_bram_wen_r : qp_rq_ba_bram_wen_p;
    assign qp_rq_ba_msb_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_ba_msb_bram_wen_r : qp_rq_ba_msb_bram_wen_p;
    assign qp_sq_ba_bram_wen              = (C_CONFIG_MEM_LATENCY == 3) ? qp_sq_ba_bram_wen_r : qp_sq_ba_bram_wen_p;
    assign qp_sq_ba_msb_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? qp_sq_ba_msb_bram_wen_r : qp_sq_ba_msb_bram_wen_p;
    assign qp_cq_ba_bram_wen              = (C_CONFIG_MEM_LATENCY == 3) ? qp_cq_ba_bram_wen_r : qp_cq_ba_bram_wen_p;
    assign qp_cq_ba_msb_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? qp_cq_ba_msb_bram_wen_r : qp_cq_ba_msb_bram_wen_p;
    assign qp_rq_wrptrdb_add_bram_wen     = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_wrptrdb_add_bram_wen_r : qp_rq_wrptrdb_add_bram_wen_p;
    assign qp_rq_wrptrdb_add_msb_bram_wen = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_wrptrdb_add_msb_bram_wen_r : qp_rq_wrptrdb_add_msb_bram_wen_p;
    assign qp_rq_cidb_bram_wen            = (C_CONFIG_MEM_LATENCY == 3) ? qp_rq_cidb_bram_wen_r : qp_rq_cidb_bram_wen_p;
    assign qp_q_depth_bram_wen            = (C_CONFIG_MEM_LATENCY == 3) ? qp_q_depth_bram_wen_r : qp_q_depth_bram_wen_p;

    assign qp_last_req_bram_wen           = (C_CONFIG_MEM_LATENCY == 3) ? qp_last_rq_bram_wen_r : qp_last_rq_bram_wen_p;
    assign qp_dest_qpid_bram_wen          = (C_CONFIG_MEM_LATENCY == 3) ? qp_dest_qpid_bram_wen_r : qp_dest_qpid_bram_wen_p;
    assign qp_mac_remote_addrl_bram_wen   = (C_CONFIG_MEM_LATENCY == 3) ? qp_mac_remote_addrl_bram_wen_r : qp_mac_remote_addrl_bram_wen_p;

    assign qp_mac_remote_addrm_bram_wen   = (C_CONFIG_MEM_LATENCY == 3) ? qp_mac_remote_addrm_bram_wen_r : qp_mac_remote_addrm_bram_wen_p;

    assign qp_ip_remote_addr1_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? qp_ip_remote_addr1_bram_wen_r : qp_ip_remote_addr1_bram_wen_p;

    assign qp_ip_remote_addr2_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? qp_ip_remote_addr2_bram_wen_r : qp_ip_remote_addr2_bram_wen_p;

    assign qp_ip_remote_addr3_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? qp_ip_remote_addr3_bram_wen_r : qp_ip_remote_addr3_bram_wen_p;

    assign qp_ip_remote_addr4_bram_wen    = (C_CONFIG_MEM_LATENCY == 3) ? qp_ip_remote_addr4_bram_wen_r : qp_ip_remote_addr4_bram_wen_p;

    assign qp_stat_ssn_bram_wen           = (C_CONFIG_MEM_LATENCY == 3) ? qp_stat_ssn_bram_wen_r : qp_stat_ssn_bram_wen_p;
    assign qp_timeout_bram_wen            = (C_CONFIG_MEM_LATENCY == 3) ? qp_timeout_bram_wen_r : qp_timeout_bram_wen_p;

    assign wr_addr_mem_r                  = (C_CONFIG_MEM_LATENCY == 3) ? wr_addr_mem_r_r : wr_addr_mem_r_p;
    assign wr_data_mem_r                  = (C_CONFIG_MEM_LATENCY == 3) ? wr_data_mem_r_r : wr_data_mem_r_p;


    assign intr_clr                       = wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r & generic_reg_wr_access & (wr_addr_r[10:2] == INTR_STS_REG[10:2]);

    assign o_intr_clr_pkt_valdn_err       = intr_clr ? s_axi_lite_wdata[0] : 1'b0;
    assign o_intr_clr_mad_pkt_rcvd        = intr_clr ? s_axi_lite_wdata[1] : 1'b0;
    assign o_intr_clr_wqe_cmpl            = intr_clr ? s_axi_lite_wdata[2] : 1'b0;
    //assign o_intr_clr_fatal_err          = intr_clr ? s_axi_lite_wdata[3] : 1'b0;
    assign o_intr_clr_fatal_err           = !qp_fatal_sts && qp_fatal_sts_ff;

    assign o_cq_intr_sts_clr_wr_en        = cq_intr_sts_clr_wr_en;
    assign o_cq_intr_sts_clr_wr_addr      = cq_intr_sts_clr_wr_addr[C_QP_INDX_WIDTH-1 : 0];
    assign qp_fatal_sts                   = (fatal_err_ones_count == 'd0) ? 1'b0 : 1'b1;
    `MSFF_R(qp_fatal_sts_ff, qp_fatal_sts, core_clk, ~core_rstn)

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_COMBO_BYPASS(1)
    ) inst_fatal_err_or_Reduce (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_fatal_err_valid),
        .wr_addr (init_start ? init_count : i_qp_fatal_err_id),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_err_data),
        .rd_en   (1'b1),
        .rd_addr (i_qp_fatal_err_id),
        .rd_data (fatal_id_rd_data)
    );

    //osq_empty And_reduce logic
    always @(posedge core_clk) begin : or_reduce_i_fatal_err
        if (~core_rstn) begin
            fatal_err_ones_count <= {C_QP_INDX_WIDTH + 1{1'b0}};
        end else begin
            case ({
                i_qp_fatal_err_valid, i_qp_fatal_err_data, fatal_id_rd_data
            })
                3'b110:  fatal_err_ones_count <= fatal_err_ones_count + 1;
                3'b101:  fatal_err_ones_count <= fatal_err_ones_count - 1;
                default: fatal_err_ones_count <= fatal_err_ones_count;
            endcase
        end
    end


    // wr_addr_r[20:19] == 2'b10 表示地址是 0x10 开头，即 0x10_0000 开始
    assign generic_reg_wr_access = (wr_addr_r[20:19] == 2'b10);

    //*****************************************************************************
    // 写响应通道逻辑：为 AXI-Lite 写事务产生 BVALID。
    //*****************************************************************************
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            s_axi_lite_bvalid <= 1'b0;
        end else begin
            //if(wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) begin
            //s_axi_lite_bvalid should not be asserted when write priority is given to hwhnshake signals
            if (!wr_req_mask_r && wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && (!(rd_req_r || s_axi_lite_arvalid)) && !s_axi_lite_bvalid) begin  //aw/w channel data is sampled; schedule a write response now
                s_axi_lite_bvalid <= 1'b1;
            end else if (s_axi_lite_bready) begin
                s_axi_lite_bvalid <= 1'b0;
            end else begin
                s_axi_lite_bvalid <= s_axi_lite_bvalid;
            end
        end
    end



    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP configuration registers (only 22 bits implemented) -- QP Configuration QPi(QPCONFi)
    // [0]      - QP_EN
    // [1]      - 无功能定义
    // [2]      - RQ_INTR_EN
    // [3]      - CQ_INTR_EN
    // [4]      - 无功能定义
    // [7:5]    - 无功能定义
    // [10:8]   - PMTU
    // [15:11]  - 无功能定义
    // [23:16]  - MAX_RD_OS
    // [31:24]  - RQ_BUF_SZ
    //

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (27 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(27),                    //positive integer
        .READ_DATA_WIDTH_A (27),                    //positive integer
        .BYTE_WRITE_WIDTH_A(27),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(27),               //positive integer
        .READ_DATA_WIDTH_B (27),               //positive integer
        .BYTE_WRITE_WIDTH_B(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_conf_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_conf_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          ({wr_data_mem_r[31:24], wr_data_mem_r[23:16], wr_data_mem_r[10:6], wr_data_mem_r[5:0]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         ({s_axi_qp_config_rdata[31:16], s_axi_qp_config_rdata[10:6], s_axi_qp_config_rdata[5:0]}),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_conf_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (27'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         ({o_qp_conf[31:24], o_qp_conf[23:16], o_qp_conf[10:6], o_qp_conf[5:0]}),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    always @(posedge core_clk) begin
        qp_conf_wr_addr_r  <= qp_conf_wr_addr;
        qp_conf_wr_addr_2r <= qp_conf_wr_addr_r;
        qp_conf_bram_wen_d <= qp_conf_bram_wen;
    end

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            qp_enable_valid           <= 1'b0;
            qp_enable_id              <= {C_QP_INDX_WIDTH{1'b0}};
            qp_enable_data            <= 1'b0;

            o_qp_disable_pulse_valid  <= 1'b0;
            o_qp_disable_pulse_id     <= {C_QP_INDX_WIDTH{1'b0}};
            o_qp_disable_pulse_data   <= 1'b0;

            o_qp_clr_fatal_err_valid  <= 1'b0;
            o_qp_clr_fatal_err_id     <= {C_QP_INDX_WIDTH{1'b0}};
            o_qp_clr_fatal_err_data   <= 1'b0;

            o_qp_under_recovery_valid <= 1'b0;
            o_qp_under_recovery_id    <= {C_QP_INDX_WIDTH{1'b0}};
            o_qp_under_recovery_data  <= 1'b0;
        end else begin
            if (qp_conf_bram_wen == 1'b1) begin
                qp_enable_valid           <= 1'b1;  // 一周期
                qp_enable_id              <= (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_wr_addr_r : qp_conf_wr_addr_2r;
                qp_enable_data            <= wr_data_mem_r[0];  // QPCONFi[0]: QP enable

                o_qp_disable_pulse_valid  <= 1'b1;  // 两周期
                o_qp_disable_pulse_id     <= (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_wr_addr_r : qp_conf_wr_addr_2r;
                o_qp_disable_pulse_data   <= !wr_data_mem_r[0];  // ~QPCONFi[0]: ~QP enable

                o_qp_clr_fatal_err_valid  <= 1'b1;  // 两周期
                o_qp_clr_fatal_err_id     <= (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_wr_addr_r : qp_conf_wr_addr_2r;
                o_qp_clr_fatal_err_data   <= !wr_data_mem_r[0];  // ~QPCONFi[0]: ~QP enable

                o_qp_under_recovery_valid <= 1'b1;  // 两周期
                o_qp_under_recovery_id    <= (C_CONFIG_MEM_LATENCY == 3) ? qp_conf_wr_addr_r : qp_conf_wr_addr_2r;
                o_qp_under_recovery_data  <= wr_data_mem_r[6];  // QPCONFi[6]: QP under recovery
            end else if (qp_conf_bram_wen == 1'b0 && qp_conf_bram_wen_d == 1'b1) begin
                qp_enable_valid           <= 1'b0;
                o_qp_disable_pulse_valid  <= 1'b1;
                o_qp_clr_fatal_err_valid  <= 1'b1;
                o_qp_under_recovery_valid <= 1'b1;
                o_qp_disable_pulse_data   <= 1'b0;
                o_qp_clr_fatal_err_data   <= 1'b0;
                o_qp_under_recovery_data  <= 1'b0;
            end else begin
                qp_enable_valid           <= 1'b0;
                o_qp_disable_pulse_valid  <= 1'b0;
                o_qp_clr_fatal_err_valid  <= 1'b0;
                o_qp_under_recovery_valid <= 1'b0;
            end
        end
    end


    assign o_qp_conf[15:11]             = 'b0;
    assign o_qp_conf_replica[15:11]     = 'b0;
    assign s_axi_qp_config_rdata[15:11] = 'b0;

    assign qp_conf_wr_addr              = wr_addr_r[8+:C_QP_INDX_WIDTH];

    // REPLICA MEMORY -- 内存复制品
    // 某些内存需要进行复制，因为IPG 0的实现需要两条并行路径，因此需要独立访问某些寄存器（且不存在任何延迟）。
    // 所有写操作都会同时发生在两个内存中。
    // 读操作则保持独立。
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (27 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(27),               //positive integer
        .READ_DATA_WIDTH_A (27),               //positive integer
        .BYTE_WRITE_WIDTH_A(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(27),               //positive integer
        .READ_DATA_WIDTH_B (27),               //positive integer
        .BYTE_WRITE_WIDTH_B(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_replica_qp_conf_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_conf_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          ({wr_data_mem_r[31:24], wr_data_mem_r[23:16], wr_data_mem_r[10:6], wr_data_mem_r[5:0]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_conf_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (27'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         ({o_qp_conf_replica[31:24], o_qp_conf_replica[23:16], o_qp_conf_replica[10:6], o_qp_conf_replica[5:0]}),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP advance configuration registers (only 30 bits implemented)   -QP advanced configuration QPi (QPADVCONFi)寄存器
    // [5:0]    - TRAFFIC CLASS
    // [7:6]    - RESERVED
    // [15:8]   - TIME_TO_LIVE
    // [31:16]  - P_KEY
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (30 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(30),                    //positive integer
        .READ_DATA_WIDTH_A (30),                    //positive integer
        .BYTE_WRITE_WIDTH_A(30),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(30),               //positive integer
        .READ_DATA_WIDTH_B (30),               //positive integer
        .BYTE_WRITE_WIDTH_B(30),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_adv_conf_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_adv_conf_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          ({wr_data_mem_r[31:8], wr_data_mem_r[5:0]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         ({s_axi_qp_adv_config_rdata[31:8], s_axi_qp_adv_config_rdata[5:0]}),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_adv_conf_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (30'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         ({o_qp_adv_conf[31:8], o_qp_adv_conf[5:0]}),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign o_qp_adv_conf[7:6]             = 'b0;
    assign s_axi_qp_adv_config_rdata[7:6] = 'b0;

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP RQ BUF BASE address registers (only 24 bits implemented) -- RCV Q Buffer base address QPi (RQBAi)
    // [7:0]    - RESERVED
    // [31:8]   - RQ_BUF_BA
    // Check_SDPRAM
    // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供 RDMA 接收队列缓冲区的基地址。
    // [31:8]: RQ（接收队列）缓冲区基地址（256 B 对齐）。256 B 是总大小（单个 RQ 缓冲区元素大小）* rq_depth
    // 保存的是“配置类”信息——每个 QP 的 RQ Buffer 基地址（RQ Buffer Base Address）。这些值通常由软件通过 AXI4-Lite 写入，用于告诉硬件接收队列缓冲区的起始位置。

    // 仅 QP1（idx==0）使用 RQ 配置；其余 QP 静默忽略写、读返回 0。
    lrdma_common_xpm_memory_tdpram_qp1 #(
        .WIDTH     (24),
        .ADDR_WIDTH(C_QP_INDX_WIDTH),
        .LATENCY_A (CONFIG_MEM_LATENCY_M),
        .LATENCY_B (1)
    ) inst_qp_rq_ba_regs (
        .clk  (core_clk),
        .rst  (~core_rstn),
        .wea  (qp_rq_ba_bram_wen),
        .addra(wr_addr_mem_r),
        .dina (wr_data_mem_r[31:8]),
        .douta(s_axi_rq_ba_rdata[31:8]),

        .web  (1'b0),
        .addrb(i_qp_rq_ba_idx[0+:C_QP_INDX_WIDTH]),
        .dinb (24'b0),
        .doutb(o_qp_rq_ba[23:0])
    );


    assign s_axi_rq_ba_rdata[7:0] = 'b0;

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_rq_ba_msb
            lrdma_common_xpm_memory_tdpram_qp1 #(
                .WIDTH     (32),
                .ADDR_WIDTH(C_QP_INDX_WIDTH),
                .LATENCY_A (CONFIG_MEM_LATENCY_M),
                .LATENCY_B (1)
            ) inst_qp_rq_ba_msb_regs (
                .clk  (core_clk),
                .rst  (~core_rstn),
                .wea  (qp_rq_ba_msb_bram_wen),
                .addra(wr_addr_mem_r),
                .dina (wr_data_mem_r),
                .douta(s_axi_rq_ba_msb_rdata),

                .web  (1'b0),
                .addrb(i_qp_rq_ba_idx[0+:C_QP_INDX_WIDTH]),
                .dinb (32'b0),
                .doutb(o_qp_rq_ba[55:24])
            );
        end else begin
            assign qp_rq_ba_msb_wr_addr   = 0;
            assign qp_rq_ba_msb_rd_addr   = 0;
            assign qp_rq_ba_msb_bram_addr = 0;
            assign s_axi_rq_ba_msb_rdata  = 0;
        end
    endgenerate

    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP SQ BASE address registers (only 27 bits implemented)
    // [4:0]    - RESERVED
    // [31:5]   - SQ_BA
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (27 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(27),                    //positive integer
        .READ_DATA_WIDTH_A (27),                    //positive integer
        .BYTE_WRITE_WIDTH_A(27),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(27),               //positive integer
        .READ_DATA_WIDTH_B (27),               //positive integer
        .BYTE_WRITE_WIDTH_B(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_ba_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_ba_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:5]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_sq_ba_rdata[31:5]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_sq_ba_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (27'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_sq_ba[31:5]),
        .sbiterrb      (),
        .dbiterrb      ()

    );



    assign o_qp_sq_ba[4:0]        = 'b0;
    assign s_axi_sq_ba_rdata[4:0] = 'b0;

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_sq_ba_msb
            // Check_SDPRAM

            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
                .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),               //string;
                .USE_MEM_INIT     (1),                //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                //integer; 0,1
                .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_qp_sq_ba_msb_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_sq_ba_msb_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_sq_ba_msb_rdata),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (1'b0),
                .addrb         (i_qp_sq_ba_idx[0+:C_QP_INDX_WIDTH]),
                .dinb          (32'b0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_sq_ba[63:32]),
                .sbiterrb      (),
                .dbiterrb      ()

            );
        end else begin
            assign qp_sq_ba_msb_wr_addr   = 0;
            assign qp_sq_ba_msb_rd_addr   = 0;
            assign qp_sq_ba_msb_bram_addr = 0;
            assign s_axi_sq_ba_msb_rdata  = 0;
        end
    endgenerate
    //-----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP CQ BASE address registers (only 27 bits implemented)
    // [4:0]    - RESERVED
    // [31:5]   - CQ_BA
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (27 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(27),                    //positive integer
        .READ_DATA_WIDTH_A (27),                    //positive integer
        .BYTE_WRITE_WIDTH_A(27),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(27),               //positive integer
        .READ_DATA_WIDTH_B (27),               //positive integer
        .BYTE_WRITE_WIDTH_B(27),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_cq_ba_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_cq_ba_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:5]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_cq_ba_rdata[31:5]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_cq_ba_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (27'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_cq_ba[31:5]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    assign o_qp_cq_ba[4:0]        = 'b0;
    assign s_axi_cq_ba_rdata[4:0] = 'b0;

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_cq_ba_msb
            // Check_SDPRAM
            xpm_memory_tdpram #(

                // Common module parameters
                .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
                .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),               //string;
                .USE_MEM_INIT     (1),                //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                //integer; 0,1
                .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(32),                    //positive integer
                .READ_DATA_WIDTH_A (32),                    //positive integer
                .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
                .READ_RESET_VALUE_A("0"),                   //string
                .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
                .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

                // Port B module parameters
                .WRITE_DATA_WIDTH_B(32),               //positive integer
                .READ_DATA_WIDTH_B (32),               //positive integer
                .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
                .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
                .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
                .READ_LATENCY_B    (1),                //non-negative integer
                .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

            ) inst_qp_cq_ba_msb_regs (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .rsta          (~core_rstn),
                .ena           (1'b1),
                .regcea        (1'b1),
                .wea           (qp_cq_ba_msb_bram_wen),
                .addra         (wr_addr_mem_r),
                .dina          (wr_data_mem_r),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),
                .douta         (s_axi_cq_ba_msb_rdata),
                .sbiterra      (),
                .dbiterra      (),

                // Port B module ports
                .clkb          (core_clk),
                .rstb          (~core_rstn),
                .enb           (1'b1),
                .regceb        (1'b1),
                .web           (1'b0),
                .addrb         (i_qp_cq_ba_idx[0+:C_QP_INDX_WIDTH]),
                .dinb          (32'b0),
                .injectsbiterrb(1'b0),
                .injectdbiterrb(1'b0),
                .doutb         (o_qp_cq_ba[63:32]),
                .sbiterrb      (),
                .dbiterrb      ()

            );
        end else begin
            assign qp_cq_ba_msb_wr_addr   = 0;
            assign qp_cq_ba_msb_rd_addr   = 0;
            assign qp_cq_ba_msb_bram_addr = 0;
            assign s_axi_cq_ba_msb_rdata  = 0;
        end
    endgenerate
    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP RQ WRPTR DB address registers -- RCV Q Write pointer DB address QPi(RQWPTRDBADDi)
    // [31:0]   - RQ_WRPTR_DB_ADD
    // Check_SDPRAM
    // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供接收队列门铃寄存器的地址。在接收到 MAD（QP1 SEND_ONLY）并消耗 RQ WQE 时，LRDMA IP 会更新寄存器所指向地址中的 RQ 门铃值。


    lrdma_common_xpm_memory_tdpram_qp1 #(
        .WIDTH     (32),
        .ADDR_WIDTH(C_QP_INDX_WIDTH),
        .LATENCY_A (CONFIG_MEM_LATENCY_M),
        .LATENCY_B (1)
    ) inst_qp_rq_wrptrdb_regs (
        .clk  (core_clk),
        .rst  (~core_rstn),
        .wea  (qp_rq_wrptrdb_add_bram_wen),
        .addra(wr_addr_mem_r),
        .dina (wr_data_mem_r[31:0]),
        .douta(s_axi_rq_wrptrdb_add_rdata),

        .web  (1'b0),
        .addrb(i_qp_rq_wrptrdb_add_idx[0+:C_QP_INDX_WIDTH]),
        .dinb (32'b0),
        .doutb(o_qp_rq_wrptrdb_add[31:0])
    );
    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_qp_rq_wrptrdb_msb
            lrdma_common_xpm_memory_tdpram_qp1 #(
                .WIDTH     (32),
                .ADDR_WIDTH(C_QP_INDX_WIDTH),
                .LATENCY_A (CONFIG_MEM_LATENCY_M),
                .LATENCY_B (1)
            ) inst_qp_rq_wrptrdb_msb_regs (
                .clk  (core_clk),
                .rst  (~core_rstn),
                .wea  (qp_rq_wrptrdb_add_msb_bram_wen),
                .addra(wr_addr_mem_r),
                .dina (wr_data_mem_r),
                .douta(s_axi_rq_wrptrdb_msb_add_rdata),

                .web  (1'b0),
                .addrb(i_qp_rq_wrptrdb_add_idx[0+:C_QP_INDX_WIDTH]),
                .dinb (32'b0),
                .doutb(o_qp_rq_wrptrdb_add[63:32])
            );
        end else begin
            assign qp_rq_wrptrdb_add_msb_wr_addr   = 0;
            assign qp_rq_wrptrdb_add_msb_rd_addr   = 0;
            assign qp_rq_wrptrdb_add_msb_bram_addr = 0;
            assign s_axi_rq_wrptrdb_msb_add_rdata  = 0;
        end
    endgenerate

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP SQ completions DB address registers 
    // [31:0]   - SQ_CMPL_DB_ADD
    // Check_SDPRAM


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP CQ Head pointer registers 
    // [15:0]   - CQ_HDPTR
    // CQHEADi: 该状态寄存器提供发送完成队列门铃值并提供有关已完成的发送 WQE 的信息。这是写入 CQDBADDi 寄存器指向的地址的门铃值


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_cq_hdptr_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_cq_hdptr_bram_wen),
        .addra         (qp_cq_hdptr_bram_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_cq_hdptr_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_cq_hdptr_wrn),
        .addrb         (i_qp_cq_hdptr_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_cq_hdptr),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_cq_hdptr[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_cq_hdptr_rdata[31:16] = 'b0;

    assign qp_cq_hdptr_wr_addr         = wr_addr_r[8+:C_QP_INDX_WIDTH];  // The repeating registers start at 0x200 so need to normalize
    assign qp_cq_hdptr_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_cq_hdptr_bram_addr       = qp_cq_hdptr_bram_wen ? qp_cq_hdptr_wr_addr : qp_cq_hdptr_rd_addr;
    // SW override
    //assign qp_cq_hdptr_bram_wen  = lrdma_adv_conf_sw_override & (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == CQ_HEAD_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
    assign qp_cq_hdptr_bram_wen        = lrdma_adv_conf_sw_override & (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && ((wr_addr_r[7:0] == CQ_HEAD_QPN) && wr_addr_r[20:19] == 2'b11);


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP RQ CI doorbell registers  -- QP RQ CI 门铃寄存器
    // [15:0]   - RQ CI Doorbell
    // 双写端口碰撞处理


    lrdma_common_xpm_memory_tdpram_qp1 #(
        .WIDTH     (16),
        .ADDR_WIDTH(C_QP_INDX_WIDTH),
        .LATENCY_A (CONFIG_MEM_LATENCY_M),
        .LATENCY_B (1)
    ) inst_qp_rq_ci_regs (
        .clk  (core_clk),
        .rst  (~core_rstn),
        .wea  (qp_rq_cidb_bram_wen),
        .addra(wr_addr_mem_r),
        .dina (wr_data_mem_r[15:0]),
        .douta(s_axi_rq_cidb_rdata[15:0]),

        .web  (1'b0),
        .addrb(rq_cidb_idx[0+:C_QP_INDX_WIDTH]),
        .dinb (16'd0),
        .doutb(o_qp_rq_cidb[15:0])
    );


    assign o_qp_rq_cidb[31:16]        = 'b0;
    assign s_axi_rq_cidb_rdata[31:16] = 'b0;

    assign rq_cidb_idx                = i_qp_rq_cidb_idx;


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP SQ PI doorbell registers 
    // [15:0]   - SQ PI Doorbell
    // 双写端口碰撞处理


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_pidb_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_pidb_bram_wen),
        .addra         (qp_sq_pidb_bram_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_sq_pidb_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (qp_sq_pidb_addr),
        .dinb          (16'd0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (sq_pidb[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    //    assign o_qp_sq_pidb[31:16]   = 'b0;
    assign s_axi_sq_pidb_rdata[31:16] = 'b0;
    assign o_qp_sq_pidb[15:0]         = sq_pidb[15:0];


    assign qp_sq_pidb_addr            = i_qp_sq_pidb_idx;

    assign qp_sq_pidb_wr_addr         = wr_addr_r[8+:C_QP_INDX_WIDTH];  // The repeating registers start at 0x200 so need to normalize
    assign qp_sq_pidb_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_sq_pidb_bram_addr       = qp_sq_pidb_bram_wen ? qp_sq_pidb_wr_addr : qp_sq_pidb_rd_addr;
    //assign qp_sq_pidb_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == SQ_PI_DB_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
    assign qp_sq_pidb_bram_wen        = (wr_req_mask_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid))) && ((wr_addr_r[7:0] == SQ_PI_DB_QPN) && wr_addr_r[20:19] == 2'b11);


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP Queue depth registers - implemented in 2 different BRAMs as they
    // are required asynchronously
    // [15:0]   - SQ depth
    // [31:16]  - RQ depth
    // Check_SDPRAM


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_depth_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_q_depth_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_q_depth_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (qp_depth_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (16'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_sq_depth[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    // CQ depth is the same as SQ depth
    assign qp_depth_idx  = i_qp_cq_depth_req ? i_qp_cq_depth_idx : i_qp_sq_depth_idx;
    assign o_qp_cq_depth = o_qp_sq_depth;


    lrdma_common_xpm_memory_tdpram_qp1 #(
        .WIDTH     (16),
        .ADDR_WIDTH(C_QP_INDX_WIDTH),
        .LATENCY_A (CONFIG_MEM_LATENCY_M),
        .LATENCY_B (1)
    ) inst_qp_rq_depth_regs (
        .clk  (core_clk),
        .rst  (~core_rstn),
        .wea  (qp_q_depth_bram_wen),
        .addra(wr_addr_mem_r),
        .dina (wr_data_mem_r[31:16]),
        .douta(s_axi_q_depth_rdata[31:16]),

        .web  (1'b0),
        .addrb(i_qp_rq_depth_idx[0+:C_QP_INDX_WIDTH]),
        .dinb (16'b0),
        .doutb(o_qp_rq_depth[15:0])
    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP SQ PSN registers -- SQ PSN for QPi (SQPSNi) 寄存器
    // [23:0]   - SQ PSN
    // 该寄存器在连接时由 SW 初始化。此后，硬件会为每个发出的数据包更新该寄存器，而软件则不应更新。该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
    // Check Registering

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_sq_psn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_psn_bram_wen),
        .addra         (qp_sq_psn_bram_addr),
        .dina          (s_axi_lite_wdata[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_sq_psn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (((i_qp_sq_psn_wrn | i_qp_sq_psn_retry_wrn) & ~i_qp_sq_psn_req)),
        .addrb         (muxed_qp_sq_psn_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_sq_psn),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_sq_psn[23:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    //assign muxed_qp_sq_psn_idx = i_qp_sq_psn_req ? i_qp_sq_psn_idx : ((i_qp_sq_psn_retry_req | i_qp_sq_psn_retry_wrn) ? i_qp_sq_psn_retry_idx : i_qp_sq_psn_wqe_idx);
    assign muxed_qp_sq_psn_idx       = i_qp_sq_psn_req ? i_qp_sq_psn_idx : ((i_qp_sq_psn_retry_req | i_qp_sq_psn_retry_wrn) ? i_qp_sq_psn_retry_idx : ((i_qp_sq_psn_wrn | i_qp_sq_psn_wqe_req) ? i_qp_sq_psn_wqe_idx : i_qp_sq_psn_resp_idx));
    assign s_axi_sq_psn_rdata[31:24] = 'b0;

    assign qp_sq_psn_wr_addr         = wr_addr_r[8+:C_QP_INDX_WIDTH];  // The repeating registers start at 0x200 so need to normalize
    assign qp_sq_psn_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_sq_psn_bram_addr       = qp_sq_psn_bram_wen ? qp_sq_psn_wr_addr : qp_sq_psn_rd_addr;
    //assign qp_sq_psn_bram_wen  = (wr_req_r && s_axi_lite_wvalid && ~s_axi_lite_bvalid) && ((wr_addr_r[7:0] == SQ_PSN_QPN) &&  (wr_addr_r[17] == 'b1) && (wr_addr_r[16:9] != 'b0));
    assign qp_sq_psn_bram_wen        = (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && ((wr_addr_r[7:0] == SQ_PSN_QPN) && wr_addr_r[20:19] == 2'b11);

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP RQ PSN registers -- Last RQ req for QPi(LSTRQREQi)
    // [23:0]   - RQ PSN
    // [31:24]  - RQ opcode
    // 该寄存器提供最后一个接收到的 RQ 数据包的详细信息。该寄存器根据参数 C_NUM_QP（其中 i = 2 至 C_NUM_QP）生成。QP1 不存在该寄存器。

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_last_req_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_last_req_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_last_req_rdata[31:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_last_req_wrn),
        .addrb         (i_qp_last_req_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_last_req),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_last_req[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    // BRAM: Dest QP conf registers ---- Destination QP configuration for QPi (DESTQPCONFi) 寄存器
    // [23:0]   - Dest QP ID
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_dest_qpid_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_dest_qpid_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_dest_qpid_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_dest_qpid_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (24'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_dest_qpid[23:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_dest_qpid_rdata[31:24] = 'b0;

    //-----------------------------------------------------------------------------------------------------

    // //-----------------------------------------------------------------------------------------------------
    // BRAM: QP timeout register -- Timeout Configuration Register for QPi(TIMEOUTCONFi)
    // [4:0]   - 超时值
    // [7:5]   - 无功能定义
    // [10:8]  - 最大重试次数
    // [31:11] - 无功能定义
    // Total 8 bits implemented
    //
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (8 * C_NUM_QP),     //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(8),                     //positive integer
        .READ_DATA_WIDTH_A (8),                     //positive integer
        .BYTE_WRITE_WIDTH_A(8),                     //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(8),                //positive integer
        .READ_DATA_WIDTH_B (8),                //positive integer
        .BYTE_WRITE_WIDTH_B(8),                //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_timeout_reg1 (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (init_start || qp_timeout_bram_wen),
        .addra         (init_start ? init_count : wr_addr_mem_r),
        .dina          (init_start ? {3'd7,5'd14} : {wr_data_mem_r[10:8], wr_data_mem_r[4:0]}),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         ({s_axi_timeout_rdata[10:8], s_axi_timeout_rdata[4:0]}),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_timeout_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (8'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         ({o_qp_timeout[10:8], o_qp_timeout[4:0]}),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    assign o_qp_timeout[7:5]          = 'b0;
    assign o_qp_timeout[31:11]        = 'b0;
    assign s_axi_timeout_rdata[7:5]   = 'b0;
    assign s_axi_timeout_rdata[31:11] = 'b0;

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP MAC REMOTE ADDR LSB register -- MAC destination address LSB QPi(MACDESADDLSBi) 寄存器
    // [31:0]   - MAC REMOTE ADDR VALUE
    // 该寄存器由 SW 在连接时配置，提供与该 QP 连接的远程主机的 MAC 地址。从该 QP 发送的所有数据包都以该 MAC 地址（LSB）作为目标 MAC 地址。
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_mac_remote1_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_mac_remote_addrl_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_mac_remote_addrl_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_mac_remote_addrl_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_mac_remote_addrl[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_mac_remote_addrl_idx = i_qp_mac_remote_addrl_req ? i_qp_mac_remote_addrl_idx : i_qp_mac_remote_addrl_wqe_idx;

    // REPLICA MEMORY
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_mac_remote1_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_mac_remote_addrl_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_mac_remote_addrl_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_mac_remote_addrl_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP MAC DEST ADDR MSB register
    // [15:0]   - MAC DEST ADDR VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_mac_remote2_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_mac_remote_addrm_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_mac_remote_addrm_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_mac_remote_addrm_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (16'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_mac_remote_addrm[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign o_qp_mac_remote_addrm[31:16]        = 'b0;
    assign s_axi_mac_remote_addrm_rdata[31:16] = 'b0;

    assign muxed_qp_mac_remote_addrm_idx       = i_qp_mac_remote_addrm_req ? i_qp_mac_remote_addrm_idx : i_qp_mac_remote_addrm_wqe_idx;

    // SDP Memory No Collission
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),               //positive integer
        .READ_DATA_WIDTH_A (16),               //positive integer
        .BYTE_WRITE_WIDTH_A(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_mac_remote2_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_mac_remote_addrm_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_mac_remote_addrm_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (16'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_mac_remote_addrm_replica[15:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP IP DEST ADDR1 register
    // [31:0]   - IP DEST ADDR LSB 1 VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote1_regs (


        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr1_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_ip_remote_addr1_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_ip_remote_addr1_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr1[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_ip_remote_addr1_idx = i_qp_ip_remote_addr1_req ? i_qp_ip_remote_addr1_idx : i_qp_ip_remote_addr1_wqe_idx;
    // REPLICA memory
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote1_replica_regs (


        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr1_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_ip_remote_addr1_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr1_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP IP DEST ADDR2 register
    // [31:0]   - IP DEST ADDR LSB 2 VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote2_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr2_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_ip_remote_addr2_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_ip_remote_addr2_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr2[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_ip_remote_addr2_idx = i_qp_ip_remote_addr2_req ? i_qp_ip_remote_addr2_idx : i_qp_ip_remote_addr2_wqe_idx;

    // REPLICA MEMORY
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote2_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr2_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_ip_remote_addr2_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr2_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP IP DEST ADDR3 register
    // [31:0]   - IP DEST ADDR MSB 1 VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote3_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr3_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_ip_remote_addr3_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_ip_remote_addr3_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr3[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_ip_remote_addr3_idx = i_qp_ip_remote_addr3_req ? i_qp_ip_remote_addr3_idx : i_qp_ip_remote_addr3_wqe_idx;
    // REPLICA MEMORY
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote3_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr3_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_ip_remote_addr3_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr3_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP IP DEST ADDR4 register
    // [31:0]   - IP DEST ADDR MSB 2 VALUE
    // Check_SDPRAM

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                    //positive integer
        .READ_DATA_WIDTH_A (32),                    //positive integer
        .BYTE_WRITE_WIDTH_A(32),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote4_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr4_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_ip_remote_addr4_rdata),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (muxed_qp_ip_remote_addr4_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr4[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign muxed_qp_ip_remote_addr4_idx = i_qp_ip_remote_addr4_req ? i_qp_ip_remote_addr4_idx : i_qp_ip_remote_addr4_wqe_idx;
    // REPLICA MEMORY
    // Some memories need to be replicated since IPG 0 implementation requires
    // 2 parallel paths and hence needs independent access to some registers
    // (without any delay). Any writes happen to both memories. Reads are
    // independent
    // SDP Memory No Collission
    // Check_SDPRAM
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (32 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),               //positive integer
        .READ_DATA_WIDTH_A (32),               //positive integer
        .BYTE_WRITE_WIDTH_A(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("read_first"),     //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(32),               //positive integer
        .READ_DATA_WIDTH_B (32),               //positive integer
        .BYTE_WRITE_WIDTH_B(32),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_ip_remote4_replica_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_ip_remote_addr4_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[31:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (i_qp_ip_remote_addr4_replica_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (32'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ip_remote_addr4_replica[31:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //-----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT CURR SQ_PTR_PROC register
    // [15:0]   - STAT_CURR_SQPTR_PROC - Read only
    // 双写端口碰撞处理

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_L),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_curr_sqptr_proc_regs (
        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (stat_curr_sqptr_bram_wen),
        .addra         (stat_curr_sqptr_proc_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_curr_sqptr_proc_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_curr_sqptr_proc_wen),
        .addrb         (i_qp_curr_sqptr_proc_idx),
        .dinb          (i_qp_curr_sqptr_proc),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_curr_sqptr_proc),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    assign s_axi_curr_sqptr_proc_rdata[31:16] = 'b0;

    // The AXI lite interface and the logic that updates the SQ full/empty
    // Port A 由 AXI-Lite 接口和 SQ full/empty 更新逻辑共享。
    // Port B 专用于 QP 管理器 cache 模块，用于读取和更新 SQ 指针。
    assign curr_sqptr_proc                    = s_axi_curr_sqptr_proc_rdata[15:0];


    assign stat_curr_sqptr_proc_addr          = stat_curr_sqptr_proc_bram_addr;
    assign stat_curr_sqptr_proc_wr_addr       = wr_addr_r[8+:C_QP_INDX_WIDTH];
    assign stat_curr_sqptr_proc_rd_addr       = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign stat_curr_sqptr_proc_bram_addr     = stat_curr_sqptr_bram_wen ? stat_curr_sqptr_proc_wr_addr : stat_curr_sqptr_proc_rd_addr;
    assign axi_curr_sqptr_proc_read           = ~generic_reg_rd_access & (rd_addr_r[7:0] == STAT_CURR_SQPTR_PROC_QPN) & ~axi_lite_rvalid_i;
    //assign axi_curr_sqptr_proc_read       = ~generic_reg_rd_access & (rd_addr_r[7:0] == STAT_CURR_SQPTR_PROC_QPN) & ~s_axi_lite_rvalid;
    assign stat_curr_sqptr_bram_wen           = lrdma_adv_conf_sw_override && (wr_req_mask_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid))) && ((wr_addr_r[7:0] == STAT_CURR_SQPTR_PROC_QPN) && wr_addr_r[20:19] == 2'b11);

    //-----------------------------------------------------------------------------------------------------
    //
    ////-----------------------------------------------------------------------------------------------------
    // BRAM: STAT SSN register -- STATSSNi
    // 该寄存器是根据参数 C_NUM_QP（其中 i = 2 至 C_NUM_QP）生成的。 QP1 不存在该寄存器。
    // [23:0]   - STAT CURR SSN    - 发送侧 SSN（与发出的 MSN 相同）

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_M),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_qp_stat_ssn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_stat_ssn_bram_wen),
        .addra         (wr_addr_mem_r),
        .dina          (wr_data_mem_r[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_ssn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_ssn_wrn),
        .addrb         (i_qp_stat_ssn_idx[0+:C_QP_INDX_WIDTH]),
        .dinb          (i_qp_stat_ssn[23:0]),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_ssn[23:0]),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_stat_ssn_rdata[31:24] = 'b0;

    //-----------------------------------------------------------------------------------------------------
    //
    //-----------------------------------------------------------------------------------------------------
    // BRAM: QP STAT NAK register -- Status QPi(STATQPi)部分
    // 与 NAK 相关的 QP 状态寄存器中的位已在此 BRAM 中实现。
    // 此外，还包括 STAT_QPn 寄存器的两个字段。
    // 由于 Curr_retry_cnt 由 ACK 管理器生成，而其他状态位由 RX 引擎生成，因此这些位分别在不同 BRAM 中实现。
    // [22:16]  - 收到 NACK 综合征（读/写）(7 bits)

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (7 * C_NUM_QP),     //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(7),                     //positive integer
        .READ_DATA_WIDTH_A (7),                     //positive integer
        .BYTE_WRITE_WIDTH_A(7),                     //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_K),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(7),                //positive integer
        .READ_DATA_WIDTH_B (7),                //positive integer
        .BYTE_WRITE_WIDTH_B(7),                //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_psn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_stat_nak_bram_wen),
        .addra         (qp_stat_nak_bram_addr),
        .dina          (s_axi_lite_wdata[22:16]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_qpn_rdata[22:16]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_nak_wen),
        .addrb         (i_qp_stat_nak_idx),
        .dinb          (i_qp_stat_nak),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_nak),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    // BRAM: QP STAT NAK register -- Status QPi(STATQPi)部分
    // [26:24]  - 重试次数（读/写）
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (3 * C_NUM_QP),     //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(3),                     //positive integer
        .READ_DATA_WIDTH_A (3),                     //positive integer
        .BYTE_WRITE_WIDTH_A(3),                     //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_K),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(3),                //positive integer
        .READ_DATA_WIDTH_B (3),                //positive integer
        .BYTE_WRITE_WIDTH_B(3),                //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_retry_cnt_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_stat_nak_bram_wen),
        .addra         (qp_stat_nak_bram_addr),
        .dina          (s_axi_lite_wdata[26:24]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_qpn_rdata[26:24]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_retry_cnt_wen),
        .addrb         (i_qp_stat_retry_cnt_idx),
        .dinb          (i_qp_stat_retry_cnt),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_retry_cnt),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    //#######################################################################################################################################
    // i_osq_full_set_valid 与 i_osq_full_reset_valid 连续同时拉高之间至少间隔 1 个 core_clk 周期
    //#######################################################################################################################################
    always @(posedge core_clk) begin : i_osq_full_proc
        if (!core_rstn) begin
            osq_full_valid         <= 1'b0;
            osq_full_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_full_data          <= 1'b0;
            osq_full_valid_pending <= 1'b0;
            osq_full_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_osq_full_set_valid) begin
                osq_full_valid <= 1'b1;
                osq_full_id    <= i_osq_full_set_id;
                osq_full_data  <= 1'b1;
            end else if (i_osq_full_reset_valid) begin
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

            if (i_osq_full_set_valid && i_osq_full_reset_valid) begin  // address collision is avoided at the source of this signal
                osq_full_valid_pending <= 1'b1;
                osq_full_id_pending    <= i_osq_full_reset_id;
            end else begin
                osq_full_valid_pending <= 1'b0;
            end
        end
    end

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
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (osq_full_rd_data_int)
    );
    assign osq_full_rd_data = (osq_full_valid_pending && osq_full_id_pending == rd_addr_mem_r_r) ? 1'b0 : osq_full_rd_data_int;

    //#######################################################################################################################################
    // i_osq_empty_set_valid 与 i_osq_empty_reset_valid 连续同时拉高之间至少间隔 1 个 core_clk 周期
    //#######################################################################################################################################
    always @(posedge core_clk) begin : i_osq_empty_proc
        if (!core_rstn) begin
            osq_empty_valid         <= 1'b0;
            osq_empty_id            <= {C_QP_INDX_WIDTH{1'b0}};
            osq_empty_data          <= 1'b0;
            osq_empty_valid_pending <= 1'b0;
            osq_empty_id_pending    <= {C_QP_INDX_WIDTH{1'b0}};
        end else begin
            if (i_osq_empty_set_valid) begin
                osq_empty_valid <= 1'b1;
                osq_empty_id    <= i_osq_empty_set_id;
                osq_empty_data  <= 1'b1;
            end else if (i_osq_empty_reset_valid) begin
                osq_empty_valid <= 1'b1;
                osq_empty_id    <= i_osq_empty_reset_id;
                osq_empty_data  <= 1'b0;
            end else if (osq_empty_valid_pending) begin
                osq_empty_valid <= 1'b1;
                osq_empty_id    <= osq_empty_id_pending;
                osq_empty_data  <= 1'b0;
            end else begin
                osq_empty_valid <= 1'b0;
            end

            if (i_osq_empty_set_valid && i_osq_empty_reset_valid) begin  // address collision is avoided at the source of this signal
                osq_empty_valid_pending <= 1'b1;
                osq_empty_id_pending    <= i_osq_empty_reset_id;
            end else begin
                osq_empty_valid_pending <= 1'b0;
            end
        end
    end

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_i_osq_empty (
        .core_clk(core_clk),
        .core_rst(!core_rstn),
        .wr_en   (init_start || osq_empty_valid),
        .wr_addr (init_start ? init_count : osq_empty_id),
        .wr_data (init_start ? 1'b1 : osq_empty_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (osq_empty_rd_data_int)
    );
    assign osq_empty_rd_data = (osq_empty_valid_pending && osq_empty_id_pending == rd_addr_mem_r_r) ? 1'b0 : osq_empty_rd_data_int;

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_retried (
        .core_clk   (core_clk),
        .core_rst   (!core_rstn),
        .wr_en      (init_start || i_qp_retried_reset_valid),
        .wr_addr    (init_start ? init_count : i_qp_retried_reset_id),
        .wr_data    (1'b0),
        .wr_en_aux  (i_qp_retried_set_valid),
        .wr_addr_aux(i_qp_retried_set_id),
        .wr_data_aux(1'b1),
        .rd_en      (1'b1),
        .rd_addr    (rd_addr_mem_r_r),
        .rd_data    (qp_retried_rd_data)
    );

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_rq_full (
        .core_clk   (core_clk),
        .core_rst   (!core_rstn),
        .wr_en      (init_start || i_rq_full_valid),
        .wr_addr    (init_start ? init_count : i_rq_full_idx),
        .wr_data    (init_start ? 1'b0 : i_rq_full_data),
        .wr_en_aux  (i_rq_full_reset_valid),
        .wr_addr_aux(i_rq_full_reset_idx),
        .wr_data_aux(i_rq_full_reset_data),
        .rd_en      (1'b1),
        .rd_addr    (rd_addr_mem_r_r),
        .rd_data    (i_rq_full_rd_data)
    );

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_fatal_err (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_fatal_err_valid),
        .wr_addr (init_start ? init_count : i_qp_fatal_err_id),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_err_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (fatal_err_rd_data)
    );

    // 每个 QP 的空/非空状态存放在 inst_sq_empty_ff 这个 LUT RAM 里，初值 1（假设系统启动时 SQ 为空）
    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_sq_empty_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || sq_empty_ff_valid),
        .wr_addr (init_start ? init_count : sq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : sq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (sq_empty_ff_rd_data)
    );

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_sq_full_ff (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || sq_full_ff_valid),
        .wr_addr (init_start ? init_count : sq_full_ff_id),
        .wr_data (init_start ? 1'b0 : sq_full_ff_data),
        .rd_en   (1'b1),
        .rd_addr (rd_addr_mem_r_r),
        .rd_data (sq_full_ff_rd_data)
    );

    //Add Pipeline
    // Status QPi (STATQPi)寄存器
    assign qp_stat_nak_bram_rd_addr    = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign s_axi_stat_qpn_rdata[0]     = fatal_err_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[1]     = i_rq_full_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[2]     = sq_full_ff_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[3]     = osq_full_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[8:4]   = 'b0;
    assign s_axi_stat_qpn_rdata[9]     = sq_empty_ff_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[10]    = osq_empty_rd_data;  // bram address is the QP index
    assign s_axi_stat_qpn_rdata[11]    = qp_retried_rd_data;
    assign s_axi_stat_qpn_rdata[15:12] = 'b0;
    // s_axi_stat_qpn_rdata[22:16] used
    assign s_axi_stat_qpn_rdata[23]    = 'b0;
    // s_axi_stat_qpn_rdata[26:24] used
    assign s_axi_stat_qpn_rdata[31:27] = 'b0;
    assign qp_stat_nak_bram_addr       = qp_stat_nak_bram_wen ? (wr_addr_r[8+:C_QP_INDX_WIDTH]) : (rd_addr_r[8+:C_QP_INDX_WIDTH]);
    assign qp_stat_nak_bram_wen        = (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && ((wr_addr_r[7:0] == STAT_QPN) && wr_addr_r[20:19] == 2'b11);

    //----------------------------------------------------------------------------------------------------


    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT RESP PSN register
    // [23:0]   - STAT_RESP_PSN - Read only
    // STATRESPSNi: 最后一个已确认/已接收的响应 PSN

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_resp_psn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports，初始化时会写入(SQ PSN - 1)的值
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_sq_psn_bram_wen),
        .addra         (qp_sq_psn_bram_addr),
        .dina          ((s_axi_lite_wdata[23:0] - 1'b1)),  // SQPSNi 初始化写入软件值减 1
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_resp_psn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_resp_psn_wen),
        .addrb         (i_qp_stat_resp_psn_idx),
        .dinb          (i_qp_stat_resp_psn),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_resp_psn),
        .sbiterrb      (),
        .dbiterrb      ()

    );


    assign s_axi_stat_resp_psn_rdata[31:24] = 'b0;


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT RQ BUF CA register
    // [31:8]   - STAT_RQ_BUF_CA - Read only
    // 保存每个 QP 的 RQ Buffer 写入地址（Current Address，CA）；硬件更新该值，用于反映接收缓冲区写入位置。

    lrdma_common_xpm_memory_tdpram_qp1 #(
        .WIDTH     (24),
        .ADDR_WIDTH(C_QP_INDX_WIDTH),
        .LATENCY_A (CONFIG_MEM_LATENCY_M),
        .LATENCY_B (1)
    ) inst_rq_buf_ca_regs (
        .clk(core_clk),
        .rst(~core_rstn),

        .wea  (qp_rq_ba_bram_wen),
        .addra(wr_addr_mem_r),
        .dina (wr_data_mem_r[31:8]),
        .douta(s_axi_stat_rq_buf_ca_rdata[31:8]),

        .web  (i_qp_stat_rq_buf_ca_wen),
        .addrb(i_qp_stat_rq_buf_ca_idx),
        .dinb (i_qp_stat_rq_buf_ca[23:0]),
        .doutb(o_qp_stat_rq_buf_ca[23:0])
    );

    generate
        if (C_M_AXI_ADDR_WIDTH == 64) begin : gen_rq_buf_ca_msb
            lrdma_common_xpm_memory_tdpram_qp1 #(
                .WIDTH     (32),
                .ADDR_WIDTH(C_QP_INDX_WIDTH),
                .LATENCY_A (CONFIG_MEM_LATENCY_M),
                .LATENCY_B (1)
            ) inst_rq_buf_ca_regs_msb (
                .clk(core_clk),
                .rst(~core_rstn),

                .wea  (qp_rq_ba_msb_bram_wen),
                .addra(wr_addr_mem_r),
                .dina (wr_data_mem_r),
                .douta(s_axi_stat_rq_buf_ca_msb_rdata),

                .web  (i_qp_stat_rq_buf_ca_wen),
                .addrb(i_qp_stat_rq_buf_ca_idx),
                .dinb (i_qp_stat_rq_buf_ca[55:24]),
                .doutb(o_qp_stat_rq_buf_ca[55:24])
            );
        end else begin
            assign s_axi_stat_rq_buf_ca_msb_rdata = 0;
        end
    endgenerate

    assign s_axi_stat_rq_buf_ca_rdata[7:0] = 'b0;


    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT WQE register
    // [15:0]   - WQE_CNT_POP - Read only


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),                    //positive integer
        .READ_DATA_WIDTH_A (16),                    //positive integer
        .BYTE_WRITE_WIDTH_A(16),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(16),               //positive integer
        .READ_DATA_WIDTH_B (16),               //positive integer
        .BYTE_WRITE_WIDTH_B(16),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_wqe_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (qp_wqe_cnt_bram_wen),
        .addra         (qp_wqe_cnt_bram_addr),
        .dina          (s_axi_lite_wdata[15:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_wqe_cnt_rdata[15:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_stat_wqe_cnt_wen),
        .addrb         (i_qp_stat_wqe_cnt_idx),
        .dinb          (i_qp_stat_wqe_cnt),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_stat_wqe_cnt),
        .sbiterrb      (),
        .dbiterrb      ()
    );
    assign s_axi_stat_wqe_cnt_rdata[31:16] = 'b0;
    assign qp_wqe_cnt_bram_addr            = qp_wqe_cnt_bram_wen ? (wr_addr_r[8+:C_QP_INDX_WIDTH]) : (rd_addr_r[8+:C_QP_INDX_WIDTH]);
    assign qp_wqe_cnt_bram_wen             = lrdma_adv_conf_sw_override & (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r) && (((wr_addr_r[7:0] == STAT_WQE_CNT)) && wr_addr_r[20:19] == 2'b11);
    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT RQ_PI_DB register -- STATRQPIDBi
    // [15:0]   - RQ_PI - Read only
    // 双写端口碰撞处理
    //
    // RQ 生产者索引门铃值寄存器
    lrdma_common_xpm_memory_tdpram_qp1 #(
        .WIDTH     (16),
        .ADDR_WIDTH(C_QP_INDX_WIDTH),
        .LATENCY_A (CONFIG_MEM_LATENCY_L),
        .LATENCY_B (1)
    ) inst_stat_rq_pi_regs (
        .clk(core_clk),
        .rst(~core_rstn),

        .wea  (qp_stat_rq_pi_db_bram_wen),
        .addra(qp_stat_rq_pi_db_bram_addr),
        .dina (s_axi_lite_wdata[15:0]),
        .douta(s_axi_stat_rq_pi_db_rdata[15:0]),

        .web  (i_qp_rq_pi_db_wrn),
        .addrb(i_qp_rq_pi_db_idx),
        .dinb (i_qp_rq_pi_db),
        .doutb(o_qp_rq_pi_db)
    );

    assign s_axi_stat_rq_pi_db_rdata[31:16] = 16'b0;
    assign qp_stat_rq_pi_db_wr_addr         = wr_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_stat_rq_pi_db_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_stat_rq_pi_db_bram_addr       = qp_stat_rq_pi_db_bram_wen ? qp_stat_rq_pi_db_wr_addr : qp_stat_rq_pi_db_rd_addr;
    assign qp_stat_rq_pi_db_bram_wen        = lrdma_adv_conf_sw_override & (wr_req_r && (s_axi_lite_wvalid || w_channel_hwshake_before_aw) && ~s_axi_lite_bvalid) && (!(rd_req_r || s_axi_lite_arvalid)) && !wr_req_mask_r && (((wr_addr_r[7:0] == STAT_RQ_PI_DB)) && wr_addr_r[20:19] == 2'b11);

    //-----------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------
    // BRAM: STAT RET_SQ_PSN register
    // [23:0]   - RETRIED_SQ_PSN - Read only
    //


    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (24 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1
        .ECC_MODE         ("no_ecc"),         //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(24),                    //positive integer
        .READ_DATA_WIDTH_A (24),                    //positive integer
        .BYTE_WRITE_WIDTH_A(24),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),       //positive integer
        .READ_RESET_VALUE_A("0"),                   //string
        .READ_LATENCY_A    (CONFIG_MEM_LATENCY_H),  //non-negative integer
        .WRITE_MODE_A      ("read_first"),          //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(24),               //positive integer
        .READ_DATA_WIDTH_B (24),               //positive integer
        .BYTE_WRITE_WIDTH_B(24),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_stat_ret_sq_psn_regs (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (1'b0),
        .addra         (qp_stat_ret_sq_psn_bram_addr),
        .dina          (s_axi_lite_wdata[23:0]),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (s_axi_stat_ret_sq_psn_rdata[23:0]),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (1'b1),
        .regceb        (1'b1),
        .web           (i_qp_ret_sq_psn_wrn),
        .addrb         (i_qp_ret_sq_psn_idx),
        .dinb          (i_qp_ret_sq_psn),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (o_qp_ret_sq_psn),
        .sbiterrb      (),
        .dbiterrb      ()
    );

    assign s_axi_stat_ret_sq_psn_rdata[31:24] = 'b0;
    assign qp_stat_ret_sq_psn_rd_addr         = rd_addr_r[8+:C_QP_INDX_WIDTH];
    assign qp_stat_ret_sq_psn_bram_addr       = qp_stat_ret_sq_psn_rd_addr;



    //-----------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------

    // 生成 SQ（发送队列）满/空/待处理信号
    // 每当更新 SQ_PI_DB 或 STAT_CURR_SQPTR_PROC 时，"满"/"空"/"待处理"信号都会发生变化，
    // 因为这两个过程都是异步发生的，而且可能同时发生，因此使用以下方法更新 "空"/"满"信号。
    // 每次写入时，都会设置一个待定信号。一旦 lrdma_common_fifo 空/满信号被更新，它就会被重置
    //
    // 在写入空闲缓冲区时消除冲突
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            curr_sqptr_vld_r  <= 1'b0;
            curr_sqptr_addr_r <= 'd0;
            curr_sqptr_data_r <= 'd0;
        end else begin
            curr_sqptr_vld_r  <= curr_sqptr_vld_c && !prog_full_db_update_curr_sqptr;
            curr_sqptr_addr_r <= curr_sqptr_addr_c;
            curr_sqptr_data_r <= curr_sqptr_data_c;
        end
    end

    assign curr_sqptr_vld_c                 = (i_qp_curr_sqptr_proc_wen == 1'b1) ? i_qp_curr_sqptr_proc_wen : stat_curr_sqptr_bram_wen;
    assign curr_sqptr_addr_c                = (i_qp_curr_sqptr_proc_wen == 1'b1) ? i_qp_curr_sqptr_proc_idx : stat_curr_sqptr_proc_wr_addr;
    assign curr_sqptr_data_c                = (i_qp_curr_sqptr_proc_wen == 1'b1) ? i_qp_curr_sqptr_proc : s_axi_lite_wdata[15:0];
    assign o_prog_full_db_update_curr_sqptr = prog_full_db_update_curr_sqptr;

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (16),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (11),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(5)
    ) u_db_update_curr_sqptr (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (curr_sqptr_vld_r),
        .din          (curr_sqptr_addr_r),
        .rd_en        (curr_sqptr_rden_r),
        .empty        (curr_sqptr_fifo_empty),
        .dout         (curr_sqptr_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (curr_sqptr_fifo_full),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (prog_full_db_update_curr_sqptr),
        .rd_data_count(),
        .rd_rst_busy  (),
        .sbiterr      (),
        .underflow    (),
        .wr_ack       (),
        .wr_data_count(curr_sqptr_fifo_dcount),
        .wr_rst_busy  (),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .sleep        (1'b0)
    );


    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),              //positive integer
        .BYTE_WRITE_WIDTH_A(16),              //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (16),               //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //string
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_duplicate_curr_sqptr (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (curr_sqptr_vld_r),
        .addra         (curr_sqptr_addr_r),
        .dina          (curr_sqptr_data_r),
        .injectsbiterra(1'b0),               // A 端不注入单比特错误
        .injectdbiterra(1'b0),               // A 端不注入双比特错误

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (1'b1),
        .regceb  (1'b1),
        .addrb   (sq_flag_updt_addr),
        .doutb   (curr_sqptr_flag_ptr),
        .sbiterrb(),                     // B 端单比特错误状态未接出
        .dbiterrb()                      // B 端双比特错误状态未接出

    );

    // 在写入空闲缓冲区时消除冲突
    always @(posedge core_clk) begin
        if (~core_rstn) begin
            sq_pidb_vld_r  <= 1'b0;
            sq_pidb_addr_r <= 'd0;
            sq_pidb_data_r <= 'd0;
        end else begin

            sq_pidb_vld_r  <= sq_pidb_vld_c && !prog_full_sq_db_update;
            sq_pidb_addr_r <= sq_pidb_addr_c;
            sq_pidb_data_r <= sq_pidb_data_c;

        end
    end

    assign sq_pidb_vld_c  = qp_sq_pidb_bram_wen;
    assign sq_pidb_addr_c = qp_sq_pidb_wr_addr;
    assign sq_pidb_data_c = s_axi_lite_wdata[15:0];

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("auto"),
        .FIFO_READ_LATENCY  (1),
        .FIFO_WRITE_DEPTH   (16),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (11),
        .RD_DATA_COUNT_WIDTH(1),
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(5)
    ) u_db_update_sq_pidb (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (sq_pidb_vld_r),
        .din          (sq_pidb_addr_r),
        .rd_en        (sq_pidb_rden_r),
        .empty        (sq_pidb_fifo_empty),
        .dout         (sq_pidb_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (prog_full_sq_db_update),
        .rd_data_count(),
        .rd_rst_busy  (),
        .sbiterr      (),
        .underflow    (),
        .wr_ack       (),
        .wr_data_count(sq_pidb_fifo_dcount),
        .wr_rst_busy  (),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .sleep        (1'b0)
    );

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (16 * C_NUM_QP),    //positive integer
        .MEMORY_PRIMITIVE ("auto"),           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(16),              //positive integer
        .BYTE_WRITE_WIDTH_A(16),              //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (16),               //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //string
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_xpm_duplicate_sq_pidb (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (sq_pidb_vld_r),
        .addra         (sq_pidb_addr_r),
        .dina          (sq_pidb_data_r),
        .injectsbiterra(1'b0),            // A 端不注入单比特错误
        .injectdbiterra(1'b0),            // A 端不注入双比特错误

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (1'b1),
        .regceb  (1'b1),
        .addrb   (sq_flag_updt_addr),
        .doutb   (sq_pidb_flag_ptr),
        .sbiterrb(),                   // B 端单比特错误状态未接出
        .dbiterrb()                    // B 端双比特错误状态未接出

    );

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            flag_updt_cs      <= WAIT_FOR_UPDT;
            curr_sqptr_rden_r <= 1'b0;
            sq_pidb_rden_r    <= 1'b0;
            sq_flag_updt_addr <= 'd0;

            sq_flag_updt_vld  <= 1'b0;
            sq_flag_updt_idx  <= 'b0;
        end else begin
            case (flag_updt_cs)
                WAIT_FOR_UPDT: begin
                    if (curr_sqptr_rden_r == 1'b0 && curr_sqptr_fifo_empty == 1'b0 && (sq_pidb_fifo_empty == 1'b1 || curr_sqptr_fifo_dcount > sq_pidb_fifo_dcount)) begin
                        flag_updt_cs      <= READ_SQ_PTR;
                        curr_sqptr_rden_r <= 1'b1;
                        sq_pidb_rden_r    <= 1'b0;
                        sq_flag_updt_addr <= curr_sqptr_fifo_dout[C_QP_INDX_WIDTH-1:0];
                    end else if (sq_pidb_rden_r == 1'b0 && sq_pidb_fifo_empty == 1'b0) begin
                        flag_updt_cs      <= READ_SQ_PTR;
                        sq_pidb_rden_r    <= 1'b1;
                        curr_sqptr_rden_r <= 1'b0;
                        sq_flag_updt_addr <= sq_pidb_fifo_dout[C_QP_INDX_WIDTH-1:0];
                    end else begin
                        sq_pidb_rden_r    <= 1'b0;
                        curr_sqptr_rden_r <= 1'b0;
                        flag_updt_cs      <= WAIT_FOR_UPDT;
                    end
                end
                READ_SQ_PTR: begin
                    flag_updt_cs <= WAIT_FOR_UPDT;
                    if (curr_sqptr_rden_r == 1'b0 && curr_sqptr_fifo_empty == 1'b0) begin
                        curr_sqptr_rden_r <= 1'b1;
                        sq_pidb_rden_r    <= 1'b0;
                        sq_flag_updt_addr <= curr_sqptr_fifo_dout[C_QP_INDX_WIDTH-1:0];
                    end else if (sq_pidb_rden_r == 1'b0 && sq_pidb_fifo_empty == 1'b0) begin
                        sq_pidb_rden_r    <= 1'b1;
                        curr_sqptr_rden_r <= 1'b0;
                        sq_flag_updt_addr <= sq_pidb_fifo_dout[C_QP_INDX_WIDTH-1:0];
                    end else begin
                        sq_pidb_rden_r    <= 1'b0;
                        curr_sqptr_rden_r <= 1'b0;
                    end
                end

                default: begin
                    flag_updt_cs <= WAIT_FOR_UPDT;
                end
            endcase

            if (curr_sqptr_rden_r == 1'b1 || sq_pidb_rden_r == 1'b1) begin
                sq_flag_updt_vld <= 1'b1;
            end else begin
                sq_flag_updt_vld <= 1'b0;
            end

            sq_flag_updt_idx <= sq_flag_updt_addr;

        end
    end

    `MSFF_RL(pending_upd_cs, 3'b000, core_clk, ~core_rstn, 3'b000)

    assign sq_empty = (sq_pidb_flag_ptr == curr_sqptr_flag_ptr) ? 1'b1 : 1'b0;  // 相等时说明 SQ 中没有未处理的 WQE，该 QP 的 SQ 为空
    assign sq_full  = (sq_pidb_flag_ptr[C_QP_INDX_WIDTH] != curr_sqptr_flag_ptr[C_QP_INDX_WIDTH]) && (sq_pidb_flag_ptr[C_QP_INDX_WIDTH-1:0] == curr_sqptr_flag_ptr[C_QP_INDX_WIDTH-1:0]);   // 不仅比较指针数值，还带着一个额外的 MSB，用来区分“相等但已经绕了一圈”的情况，从而避免把 “满”误判成 “空”。

    always @(posedge core_clk) begin : sq_empty_ff  //and sq_full_ff
        if (!core_rstn) begin
            sq_empty_ff_valid <= 1'b0;
            sq_empty_ff_id    <= {C_QP_INDX_WIDTH{1'b0}};
            sq_empty_ff_data  <= 1'b0;
            sq_full_ff_valid  <= 1'b0;
            sq_full_ff_id     <= {C_QP_INDX_WIDTH{1'b0}};
            sq_full_ff_data   <= 1'b0;
        end else begin
            // i_sw_override_per_qp_en[C_QP_INDX_WIDTH] = o_lrdma_adv_conf_sw_override
            // i_sw_override_per_qp_en[C_QP_INDX_WIDTH-1 : 0] = o_lrdma_adv_conf_sw_override_qpid
            if (sq_flag_updt_vld && !(i_sw_override_per_qp_en[C_QP_INDX_WIDTH] && i_sw_override_per_qp_en[C_QP_INDX_WIDTH-1 : 0] == sq_flag_updt_idx)) begin
                sq_empty_ff_valid <= 1'b1;
                sq_empty_ff_id    <= sq_flag_updt_idx;
                sq_empty_ff_data  <= sq_empty;

                sq_full_ff_valid  <= 1'b1;
                sq_full_ff_id     <= sq_flag_updt_idx;
                sq_full_ff_data   <= sq_full;
            end else begin
                sq_empty_ff_valid <= 1'b0;
                sq_full_ff_valid  <= 1'b0;
            end
        end
    end

    //assign o_sq_full  = sq_full_ff;
    // 当重新计算出的 sq_empty_ff_* 与 inst_sq_empty_ff LUT RAM 里保存的旧值不同，就会把新值写回，并通过以下语句把“该 QP 的 SQ 空状态发生变化”这个事件通知外部模块。
    // 以下只报告“某一个 QP 的 SQ 空状态发生变化”
    assign o_sq_empty_valid = sq_empty_ff_valid;
    assign o_sq_empty_id    = sq_empty_ff_id;  // qpid
    assign o_sq_empty_data  = sq_empty_ff_data;  // 空-1，非空-0

    // ---- 所有 QP 的 SQ 是否都为空？ ----
    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1),
        .C_COMBO_BYPASS(1)
    ) inst_sq_empty_ff_and_reduce (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || sq_empty_ff_valid),
        .wr_addr (init_start ? init_count : sq_empty_ff_id),
        .wr_data (init_start ? 1'b1 : sq_empty_ff_data),
        .rd_en   (1'b1),
        .rd_addr (sq_empty_ff_id),
        .rd_data (sq_empty_ff_id_rd_data)
    );

    // 检测所有 QP 的 SQ 是否都为空
    //i_osq_almost_full And_reduce logic
    always @(posedge core_clk) begin : sq_empty_ff_and_reduce_proc
        if (~core_rstn) begin
            sq_empty_ff_zeros_count <= {C_QP_INDX_WIDTH + 1{1'b0}};
        end else begin
            case ({
                sq_empty_ff_valid, sq_empty_ff_data, sq_empty_ff_id_rd_data
            })
                3'b110:  sq_empty_ff_zeros_count <= sq_empty_ff_zeros_count - 1;
                3'b101:  sq_empty_ff_zeros_count <= sq_empty_ff_zeros_count + 1;
                default: sq_empty_ff_zeros_count <= sq_empty_ff_zeros_count;
            endcase
        end
    end
    assign sq_empty_ff_and_reduce = (sq_empty_ff_zeros_count == 0) ? 1'b1 : 1'b0;
    // ---- end ----


    `MSFF_RL(lrdma_adv_conf_sw_override_ff, lrdma_adv_conf_sw_override, core_clk, ~core_rstn, 3'b000)
    assign posedge_lrdma_sw_override = ~lrdma_adv_conf_sw_override_ff & lrdma_adv_conf_sw_override;
    `MSFF_RL(lrdma_sw_override_cnt, (posedge_lrdma_sw_override ? (lrdma_sw_override_cnt + 1'b1) : lrdma_sw_override_cnt), core_clk, ~core_rstn, 3'b000)
    //------------------------------------------------------------------------------

    `MSFF_R(qp_conf_req_rx_engine_ff, i_qp_conf_req_rx_engine, core_clk, ~core_rstn)
    `MSFF_R(qp_conf_req_replica_ff, i_qp_conf_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_conf_req_tx_engine_ff, i_qp_conf_req_tx_engine & ~i_qp_conf_req_rx_engine, core_clk, ~core_rstn)
    `MSFF_R(qp_conf_req_ack_mgr_ff, (i_qp_conf_req_ack_mgr & ~i_qp_conf_req_rx_engine & ~i_qp_conf_req_tx_engine), core_clk, ~core_rstn)
    `MSFF_R(qp_cfg_retry_cnt_rd_req_ff, (i_qp_cfg_retry_cnt_rd_req & ~i_qp_conf_req_rx_engine & ~i_qp_conf_req_tx_engine), core_clk, ~core_rstn)
    `MSFF_R(qp_adv_conf_req_ff, i_qp_adv_conf_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_pi_db_req_ff, i_qp_rq_pi_db_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ret_sq_psn_req_ff, i_qp_ret_sq_psn_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ret_sq_psn_req_int_ff, i_qp_ret_sq_psn_wrn & ~i_qp_ret_sq_psn_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_ba_req_ff, i_qp_rq_ba_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_ba_req_int_ff, i_qp_sq_ba_req_int, core_clk, ~core_rstn)
    `MSFF_R(qp_cq_ba_req_ff, i_qp_cq_ba_req, core_clk, ~core_rstn)
    `MSFF_R(qp_cq_hdptr_req_ff, (i_qp_cq_hdptr_req & ~i_qp_cq_hdptr_retry_req & ~i_qp_cq_hdptr_wrn), core_clk, ~core_rstn)
    `MSFF_R(qp_cq_hdptr_retry_req_ff, i_qp_cq_hdptr_retry_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_cidb_req_ff, i_qp_rq_cidb_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_pidb_req_ff, i_qp_sq_pidb_req, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_depth_req_ff, i_qp_rq_depth_req, core_clk, ~core_rstn)
    `MSFF_R(qp_sq_depth_req_ff, (i_qp_sq_depth_req & ~i_qp_cq_depth_req), core_clk, ~core_rstn)
    // RX 路径的请求
    `MSFF_R(qp_sq_psn_req_ff, i_qp_sq_psn_req, core_clk, ~core_rstn)
    // SQ WQE 路径（QP 管理器与 TX 引擎）的请求
    // 只有当 SQ WQE 路径请求有效且 RX 路径没有同时请求(读取 SQPSNi 寄存器)时，给 WQE 路径的 valid 才为高
    `MSFF_R(qp_sq_psn_wqe_req_ff, (i_qp_sq_psn_wqe_req & ~i_qp_sq_psn_req), core_clk, ~core_rstn)
    // 重传路径(lrdma_qp_retransmit_ctrl)的请求
    // 只有当 重传路径请求有效且 RX 路径没有同时请求(读取 SQPSNi 寄存器)时，给重传路径的 valid 才为高
    `MSFF_R(qp_sq_psn_retry_req_ff, (i_qp_sq_psn_retry_req & ~i_qp_sq_psn_req), core_clk, ~core_rstn)
    `MSFF_R(qp_last_req_ff, i_qp_last_req, core_clk, ~core_rstn)
    `MSFF_R(qp_dest_qpid_req_tx_engine_ff, i_qp_dest_qpid_req_tx_engine, core_clk, ~core_rstn)
    `MSFF_R(qp_dest_qpid_req_rx_engine_ff, i_qp_dest_qpid_req_rx_engine, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrl_req_ff, i_qp_mac_remote_addrl_req, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrl_req_replica_ff, i_qp_mac_remote_addrl_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrm_req_ff, i_qp_mac_remote_addrm_req, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrm_req_replica_ff, i_qp_mac_remote_addrm_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrl_wqe_req_ff, (i_qp_mac_remote_addrl_wqe_req & ~i_qp_mac_remote_addrl_req), core_clk, ~core_rstn)
    `MSFF_R(qp_mac_remote_addrm_wqe_req_ff, (i_qp_mac_remote_addrm_wqe_req & ~i_qp_mac_remote_addrm_req), core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr1_req_ff, i_qp_ip_remote_addr1_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr1_req_replica_ff, i_qp_ip_remote_addr1_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr2_req_ff, i_qp_ip_remote_addr2_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr2_req_replica_ff, i_qp_ip_remote_addr2_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr3_req_ff, i_qp_ip_remote_addr3_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr3_req_replica_ff, i_qp_ip_remote_addr3_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr4_req_ff, i_qp_ip_remote_addr4_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr4_req_replica_ff, i_qp_ip_remote_addr4_replica_req, core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr1_wqe_req_ff, (i_qp_ip_remote_addr1_wqe_req & ~i_qp_ip_remote_addr1_req), core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr2_wqe_req_ff, (i_qp_ip_remote_addr2_wqe_req & ~i_qp_ip_remote_addr2_req), core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr3_wqe_req_ff, (i_qp_ip_remote_addr3_wqe_req & ~i_qp_ip_remote_addr3_req), core_clk, ~core_rstn)
    `MSFF_R(qp_ip_remote_addr4_wqe_req_ff, (i_qp_ip_remote_addr4_wqe_req & ~i_qp_ip_remote_addr4_req), core_clk, ~core_rstn)
    `MSFF_R(qp_curr_sqptr_req_ff, i_qp_curr_sqptr_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_resp_psn_req_ff, i_qp_stat_resp_psn_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_ssn_req_ff, i_qp_stat_ssn_req, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_nak_req_ff, i_qp_stat_nak_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_rq_buf_ca_rdreq_ff, i_qp_stat_rq_buf_ca_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_stat_wqe_cnt_rdreq_ff, i_qp_stat_wqe_cnt_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_rq_wrptrdb_add_req_ff, i_qp_rq_wrptrdb_add_req, core_clk, ~core_rstn)
    `MSFF_R(qp_retry_cnt_req_ff, i_qp_stat_retry_cnt_rdreq, core_clk, ~core_rstn)
    `MSFF_R(qp_retry_cnt_req_2ff, qp_retry_cnt_req_ff, core_clk, ~core_rstn)

    // SQ depth request from cache module has higher priority
    `MSFF_R(qp_cq_depth_req_ff, i_qp_cq_depth_req, core_clk, ~core_rstn)

    always @(posedge core_clk)
        if (~core_rstn) o_qp_sq_psn_resp_valid <= 'h0;
        else if (i_qp_sq_psn_req | i_qp_sq_psn_wqe_req | i_qp_sq_psn_retry_req) o_qp_sq_psn_resp_valid <= 'h0;
        else if (i_qp_sq_psn_wrn | i_qp_sq_psn_retry_wrn) o_qp_sq_psn_resp_valid <= 'h0;
        else o_qp_sq_psn_resp_valid <= 'h1;

    assign o_qp_rq_wrptrdb_add_valid            = qp_rq_wrptrdb_add_req_ff;
    assign o_qp_cq_hdptr_valid                  = qp_cq_hdptr_req_ff;
    assign o_qp_cq_hdptr_valid_retry            = qp_cq_hdptr_retry_req_ff;
    assign o_qp_conf_valid_tx_engine            = qp_conf_req_tx_engine_ff;
    assign o_qp_conf_valid_ack_mgr              = qp_conf_req_ack_mgr_ff;
    assign o_qp_cfg_retry_cnt_rd_rdy            = qp_cfg_retry_cnt_rd_req_ff;  // TIMEOUTCONFi 重试次数读请求延后一拍作为有效响应
    assign o_qp_conf_valid_rx_engine            = qp_conf_req_rx_engine_ff;
    assign o_qp_conf_replica_valid              = qp_conf_req_replica_ff;
    assign o_qp_adv_conf_valid                  = qp_adv_conf_req_ff;
    assign o_qp_rq_pi_db_valid                  = qp_rq_pi_db_req_ff;
    assign o_qp_ret_sq_psn_valid                = qp_ret_sq_psn_req_ff;
    assign o_qp_ret_sq_psn_valid_int            = qp_ret_sq_psn_req_int_ff;
    assign o_qp_rq_ba_valid                     = qp_rq_ba_req_ff;
    assign o_qp_sq_ba_valid_int                 = qp_sq_ba_req_int_ff;
    assign o_qp_cq_ba_valid                     = qp_cq_ba_req_ff;
    // assign o_qp_rq_wrptrdb_valid               = qp_rq_wrptrdb_req_ff;
    // assign o_qp_sq_cmpldb_valid                = qp_sq_cmpldb_req_ff;
    assign o_qp_rq_cidb_valid                   = qp_rq_cidb_req_ff;
    assign o_qp_sq_pidb_valid                   = qp_sq_pidb_req_ff;
    assign o_qp_rq_depth_valid                  = qp_rq_depth_req_ff;
    assign o_qp_sq_depth_valid                  = qp_sq_depth_req_ff;
    assign o_qp_cq_depth_valid                  = qp_cq_depth_req_ff;
    assign o_qp_sq_psn_valid                    = qp_sq_psn_req_ff;
    assign o_qp_sq_psn_wqe_valid                = qp_sq_psn_wqe_req_ff;
    assign o_qp_sq_psn_retry_valid              = qp_sq_psn_retry_req_ff & ~i_qp_sq_psn_req;
    assign o_qp_last_req_valid                  = qp_last_req_ff;
    assign o_qp_dest_qpid_valid_tx_engine       = qp_dest_qpid_req_tx_engine_ff;
    assign o_qp_dest_qpid_valid_rx_engine       = qp_dest_qpid_req_rx_engine_ff;
    assign o_qp_mac_remote_addrl_valid          = qp_mac_remote_addrl_req_ff;
    assign o_qp_mac_remote_addrl_replica_valid  = qp_mac_remote_addrl_req_replica_ff;
    assign o_qp_mac_remote_addrm_valid          = qp_mac_remote_addrm_req_ff;
    assign o_qp_mac_remote_addrm_replica_valid  = qp_mac_remote_addrm_req_replica_ff;
    assign o_qp_mac_remote_addrl_wqe_valid      = qp_mac_remote_addrl_wqe_req_ff;
    assign o_qp_mac_remote_addrm_wqe_valid      = qp_mac_remote_addrm_wqe_req_ff;
    assign o_qp_ip_remote_addr1_valid           = qp_ip_remote_addr1_req_ff;
    assign o_qp_ip_remote_addr1_replica_valid   = qp_ip_remote_addr1_req_replica_ff;
    assign o_qp_ip_remote_addr2_valid           = qp_ip_remote_addr2_req_ff;
    assign o_qp_ip_remote_addr2_replica_valid   = qp_ip_remote_addr2_req_replica_ff;
    assign o_qp_ip_remote_addr3_valid           = qp_ip_remote_addr3_req_ff;
    assign o_qp_ip_remote_addr3_replica_valid   = qp_ip_remote_addr3_req_replica_ff;
    assign o_qp_ip_remote_addr4_valid           = qp_ip_remote_addr4_req_ff;
    assign o_qp_ip_remote_addr4_replica_valid   = qp_ip_remote_addr4_req_replica_ff;
    assign o_qp_ip_remote_addr1_wqe_valid       = qp_ip_remote_addr1_wqe_req_ff;
    assign o_qp_ip_remote_addr2_wqe_valid       = qp_ip_remote_addr2_wqe_req_ff;
    assign o_qp_ip_remote_addr3_wqe_valid       = qp_ip_remote_addr3_wqe_req_ff;
    assign o_qp_ip_remote_addr4_wqe_valid       = qp_ip_remote_addr4_wqe_req_ff;
    assign o_qp_curr_sqptr_proc_valid           = qp_curr_sqptr_req_ff;
    assign o_qp_stat_resp_psn_valid             = qp_stat_resp_psn_req_ff;
    assign o_qp_stat_rq_buf_ca_valid            = qp_stat_rq_buf_ca_rdreq_ff;
    assign o_qp_stat_nak_valid                  = qp_stat_nak_req_ff;
    assign o_qp_stat_wqe_cnt_valid              = qp_stat_wqe_cnt_rdreq_ff;
    assign o_qp_stat_ssn_valid                  = qp_stat_ssn_req_ff;
    assign o_qp_stat_retry_cnt_valid            = qp_retry_cnt_req_ff & ~qp_retry_cnt_req_2ff;

    assign o_lrdma_en                           = lrdma_conf_lrdma_en;
    assign o_err_buf_en                         = lrdma_conf_err_buf_en;
    assign o_flow_credits                       = lrdma_conf_flow_credits;
    assign o_num_qp_en                          = lrdma_conf_num_qps_enabled;
    assign o_lrdma_udp_src_port                 = lrdma_conf_udp_src_port;
    assign o_ipv4_lrdma_addr                    = ipv4_lrdma_addr;
    assign o_ipv6_lrdma_addr                    = {ipv6_lrdma_addr4, ipv6_lrdma_addr3, ipv6_lrdma_addr2, ipv6_lrdma_addr1};
    assign o_mac_lrdma_addr                     = {mac_lrdma_addr_msb, mac_lrdma_addr_lsb};
    assign roce_pause_on_threshold              = roce_pause_on_threshold_reg;
    assign roce_pause_off_threshold             = roce_pause_off_threshold_reg;
    assign pfc_en_roce                          = pfc_en_roce_reg;
    assign pfc_en_non_roce                      = pfc_en_non_roce_reg;
    assign roce_priority                        = roce_priority_reg;
    assign non_roce_priority                    = non_roce_priority_reg;
    assign disable_priority_chk                 = disable_priority_chk_reg;

    assign o_rq_err_pkt_buf_ba                  = {rq_err_pkt_buf_ba_msb, rq_err_pkt_buf_ba};
    assign o_rq_err_pkt_buf_sz_num_bufs         = rq_err_pkt_buf_sz_num_bufs;
    assign o_rq_err_pkt_buf_sz_buf_sz           = rq_err_pkt_buf_sz_buf_sz;

    assign o_lrdma_adv_conf_sw_override         = lrdma_adv_conf_sw_override;
    assign o_lrdma_adv_conf_sw_override_qpid    = lrdma_adv_conf_sw_override_qpid;
    assign o_lrdma_adv_conf_retry_cnt_fatal_dis = lrdma_adv_conf_retry_cnt_fatal_dis;

    assign o_intr_en_pkt_valdn_err              = intr_en_pkt_valdn_err;
    assign o_intr_en_wqe_cmpl                   = intr_en_wqe_cmpl;
    assign o_intr_en_mad_pkt_rcvd               = intr_en_mad_pkt_rcvd;
    assign o_intr_en_fatal_err                  = intr_en_fatal_err;

    assign o_in_errsts_q_ba                     = {in_errsts_q_ba_msb, in_errsts_q_ba};
    assign o_in_errsts_q_sz                     = in_errsts_q_sz;

    //-------------------------------------------------------------------------------------

endmodule
