`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_qp_mgr_top — LRDMA QP 管理器顶层模块
//
// 【模块概述】
// 将所有 QP 管理子模块集成在一起，是连接 AXI-Lite 配置接口、DDR AXI Master 接口、
// TX 引擎接口和重传逻辑的核心枢纽。
//
// 【内部子模块实例化】
// 1. lrdma_reg_block (配置寄存器集)：
//    通过 AXI-Lite 提供全局和 Per-QP 配置寄存器的读写访问
// 2. lrdma_qp_rr_arbiter (轮询仲裁器)：
//    在多个 QP 的 SQ 之间进行公平仲裁
// 3. lrdma_qp_wqe_cache (缓存控制器)：
//    协调 SQ WQE 的取数流程（仲裁→配置读取→DDR 读→指针更新）
// 4. lrdma_qp_mgr_axi_master (AXI 主接口)：
//    执行实际的 DDR 读操作获取 WQE 数据
// 5. lrdma_qp_retransmit_ctrl (重传控制器)：
//    处理重传场景下的 FIFO 清理和 PSN/SSN 回退
//
// 【顶层逻辑】
// - WQE FIFO：缓存从 DDR 读回的 WQE 数据，QP ID 嵌入 WQE 的 bit[136+] 位置
// - retransmit BRAM：存储每个 QP 的部分 WQE 状态（retransmit_wqe 翻转标志）
// - osq_pop / wqe_dropped FIFO：辅助重传清理流程
// - SQ PSN 读写端口在正常路径和重传路径之间优先级复用
// - qp_disable_retrans_clear：QP 禁用时清除 retransmit BRAM 中的过时状态
//
// 【对外接口】
// - AXI-Lite Slave：CPU 配置寄存器读写
// - AXI4 Master：DDR SQ/WQE 读取
// - TX 引擎：o_wqe[511:0], i_wqe_pop, o_wqe_empty
// - 中断管理：8 类中断使能/清除/状态
// - PFC 配置：优先级、暂停偏移、阈值
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_qp_mgr_top #(
    parameter C_S_AXI_LITE_ADDR_WIDTH     = 64,   // 32
    parameter C_S_AXI_LITE_DATA_WIDTH     = 32,   // 32
    parameter C_M_AXI_ADDR_WIDTH          = 64,   // 64
    parameter C_NUM_QP                    = 256,  // 32
    parameter C_QP_INDX_WIDTH             = 8,    // 5
    parameter STB_WIDTH                   = 4,    // 4, AXI-Lite WSTRB 位宽
    parameter C_OS_Q_DEPTH                = 16,   // 16
    parameter C_OS_Q_INDX_WIDTH           = 4,    // 4
    parameter C_ENABLE_FAST_WQE_AXI_EN    = 1,    // 1: issue WQE AXI read immediately; 0: legacy one-cycle delayed enable
    parameter C_ENABLE_WQE_BYPASS         = 1,    // 1: bypass freshly returned normal WQE around the FIFO read latency
    parameter C_ENABLE_SAME_SQ_FAST_FETCH = 1     // 1: reuse cached same-SQ context for consecutive WQEs
) (
    input s_axi_lite_aclk,
    input s_axi_lite_aresetn,

    input  [C_S_AXI_LITE_ADDR_WIDTH-1:0] s_axi_lite_awaddr,
    output                               s_axi_lite_awready,
    input                                s_axi_lite_awvalid,
    input  [C_S_AXI_LITE_ADDR_WIDTH-1:0] s_axi_lite_araddr,
    output                               s_axi_lite_arready,
    input                                s_axi_lite_arvalid,
    input  [C_S_AXI_LITE_DATA_WIDTH-1:0] s_axi_lite_wdata,
    input  [              STB_WIDTH-1:0] s_axi_lite_wstrb,
    output                               s_axi_lite_wready,
    input                                s_axi_lite_wvalid,
    output [C_S_AXI_LITE_DATA_WIDTH-1:0] s_axi_lite_rdata,
    output [                        1:0] s_axi_lite_rresp,
    input                                s_axi_lite_rready,
    output                               s_axi_lite_rvalid,
    output [                        1:0] s_axi_lite_bresp,
    input                                s_axi_lite_bready,
    output                               s_axi_lite_bvalid,

    input core_clk,  // Module Clock  
    input core_rstn, // Active low core reset

    // Interface to TX Engine for arbitrated WQ entries
    // 用于仲裁 WQE 的 TX 引擎接口
    output [                 511:0] o_wqe,
    input                           i_wqe_pop,
    output                          o_wqe_empty,
    output                          o_wqe_fast_valid,
    output                          o_halt,
    output [C_QP_INDX_WIDTH -1 : 0] o_halted_qpid,
    input                           i_wqe_halted,

    // Interrupt signals coming from Header validation module
    // 来自头部验证模块的中断信号
    input i_pkt_valdn_err_intr,
    input i_mad_pkt_rcvd_intr,

    output o_intr_clr_pkt_valdn_err,
    output o_intr_clr_wqe_cmpl,
    output o_intr_clr_mad_pkt_rcvd,
    output o_intr_clr_fatal_err,

    output o_intr_en_pkt_valdn_err,
    output o_intr_en_wqe_cmpl,
    output o_intr_en_mad_pkt_rcvd,
    output o_intr_en_fatal_err,

    // Status inputs from Header validation module
    // 来自头部验证模块的状态输入
    input                         i_rq_full_valid,           // Rcv Q full for QPn
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

    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_ssn_idx,
    input  [                23:0] i_qp_stat_ssn,
    output [                23:0] o_qp_stat_ssn,
    input                         i_qp_stat_ssn_req,
    input                         i_qp_stat_ssn_wrn,
    output                        o_qp_stat_ssn_valid,

    input  [C_QP_INDX_WIDTH -1:0] i_qp_timeout_idx,  // TIMEOUTCONFi 的 QP 索引
    output [                31:0] o_qp_timeout,      // TIMEOUTCONFi 配置值

    input  [C_QP_INDX_WIDTH -1:0] i_qp_rq_pi_db_idx,   // STATRQPIDBi 的 QP 索引
    input  [                15:0] i_qp_rq_pi_db,
    output [                15:0] o_qp_rq_pi_db,
    input                         i_qp_rq_pi_db_req,
    input                         i_qp_rq_pi_db_wrn,
    output                        o_qp_rq_pi_db_valid,

    // 重传前 SQ PSN 快照寄存器，供软件读取状态
    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_ret_sq_psn_idx,   // STATRET_SQ_PSN 的 QP 索引
    output [                23:0] o_qp_stat_ret_sq_psn,
    input                         i_qp_stat_ret_sq_psn_req,
    output                        o_qp_stat_ret_sq_psn_valid,


    // Status NAK details from RX Engine {NAK_SYNDRM_RCVD}
    input  [                 6:0] i_qp_stat_nak,
    input                         i_qp_stat_nak_wen,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_nak_idx,
    output                        o_qp_stat_nak_valid,
    input                         i_qp_stat_nak_rdreq,
    output [                 6:0] o_qp_stat_nak,

    // 来自 ACK 管理器的 NAK 状态信息
    input  [                 2:0] i_qp_stat_retry_cnt,
    input                         i_qp_stat_retry_cnt_wen,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_retry_cnt_idx,
    output                        o_qp_stat_retry_cnt_valid,
    input                         i_qp_stat_retry_cnt_rdreq,
    output [                 2:0] o_qp_stat_retry_cnt,
    // Status from tx engine
    input  [                31:0] i_tx_engine_sts,

    //Configuration outputs going to other modules (generic)
    output                       o_lrdma_en,
    output                       o_lrdma_adv_conf_retry_cnt_fatal_dis,
    output                       o_err_buf_en,
    output [                1:0] o_flow_credits,
    output [               15:0] o_lrdma_udp_src_port,
    output [C_QP_INDX_WIDTH:0] o_num_qp_en,                           // Number of QPs enabled
    output [               47:0] o_mac_lrdma_addr,                      // Ethernet MAC destination (own) address from LRDMA registers
    output [               31:0] o_ipv4_lrdma_addr,                     // IPv4 destination (own) address from LRDMA registers
    output [              127:0] o_ipv6_lrdma_addr,                     // IPv6 destination (own) address form LRDMA registers

    output [C_M_AXI_ADDR_WIDTH-1:0] o_rq_err_pkt_buf_ba,
    output [                  15:0] o_rq_err_pkt_buf_sz_num_bufs,
    output [                  15:0] o_rq_err_pkt_buf_sz_buf_sz,
    input  [                  15:0] i_rq_err_pkt_buf_wrptr,

    input [15:0] i_inc_ack_pkt_cnt,
    input [15:0] i_inc_mad_pkt_cnt,
    input [15:0] i_inc_inv_pkt_cnt,
    input [31:0] i_inc_all_dropped_cnt,
    input [15:0] i_inc_nack_cnt,

    input [15:0] i_out_wr_pkt_cnt,
    input [15:0] i_out_mad_pkt_cnt,
    input [31:0] i_ack_mgr_sts,
    input [31:0] i_stat_retry_cnt,

    output                        o_qp_enable_valid,
    output [C_QP_INDX_WIDTH -1:0] o_qp_enable_id,
    output                        o_qp_enable_data,


    //Configuration outputs going to other modules (PER QP)
    input                         i_qp_conf_req_ack_mgr,
    input                         i_qp_conf_req_tx_engine,
    input                         i_qp_conf_req_rx_engine,
    output                        o_qp_conf_valid_ack_mgr,
    output                        o_qp_conf_valid_tx_engine,
    output                        o_qp_conf_valid_rx_engine,
    output [                31:0] o_qp_conf,                  // QP configuration
    input  [C_QP_INDX_WIDTH -1:0] i_qp_conf_idx,              // 读取 QPCONFi 的 QP 索引

    input                         i_qp_cfg_retry_cnt_rd_req,
    output                        o_qp_cfg_retry_cnt_rd_rdy,
    output [                31:0] o_qp_conf_replica,          // QP configuration REPLICA
    input  [C_QP_INDX_WIDTH -1:0] i_qp_conf_replica_idx,      // 读取 QPCONFi 复制寄存器的 QP 索引
    input                         i_qp_conf_replica_req,
    output                        o_qp_conf_replica_valid,

    input                         i_qp_adv_conf_req,
    output                        o_qp_adv_conf_valid,
    output [                31:0] o_qp_adv_conf,        // QP advance configuration
    input  [C_QP_INDX_WIDTH -1:0] i_qp_adv_conf_idx,    // 读取 QPADVCONFi 的 QP 索引

    input                           i_qp_rq_ba_req,
    output                          o_qp_rq_ba_valid,
    output [C_M_AXI_ADDR_WIDTH-9:0] o_qp_rq_ba,
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_rq_ba_idx,    // 读取 RQ 缓冲区基地址的 QP 索引

    input                           i_qp_cq_ba_req,
    output                          o_qp_cq_ba_valid,
    output [C_M_AXI_ADDR_WIDTH-1:0] o_qp_cq_ba,
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_cq_ba_idx,    // 读取 CQ 基地址的 QP 索引

    input                           i_qp_rq_wrptrdb_add_req,
    output                          o_qp_rq_wrptrdb_add_valid,
    output [C_M_AXI_ADDR_WIDTH-1:0] o_qp_rq_wrptrdb_add,
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_rq_wrptrdb_add_idx,    // 读取 RQ 写门铃地址的 QP 索引

    input                         i_qp_cq_hdptr_req,
    output                        o_qp_cq_hdptr_valid,
    output [                15:0] o_qp_cq_hdptr,
    input  [                15:0] i_qp_cq_hdptr,
    input                         i_qp_cq_hdptr_wrn,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_cq_hdptr_idx,    // 读取 CQ head pointer 的 QP 索引

    input                         i_qp_rq_cidb_req,
    output                        o_qp_rq_cidb_valid,
    output [                31:0] o_qp_rq_cidb,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_rq_cidb_idx,    // 读取 RQ CI doorbell 的 QP 索引

    // Response PSN status register. Read interface to rx engine
    input  [                23:0] i_qp_stat_resp_psn,
    input                         i_qp_stat_resp_psn_wen,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_stat_resp_psn_idx,
    output                        o_qp_stat_resp_psn_valid,
    input                         i_qp_stat_resp_psn_rdreq,
    output [                23:0] o_qp_stat_resp_psn,

    // RQ 缓冲区写入地址
    input  [C_M_AXI_ADDR_WIDTH-9:0] i_qp_stat_rq_buf_ca,
    input                           i_qp_stat_rq_buf_ca_wen,
    input  [  C_QP_INDX_WIDTH -1:0] i_qp_stat_rq_buf_ca_idx,
    output                          o_qp_stat_rq_buf_ca_valid,
    input                           i_qp_stat_rq_buf_ca_rdreq,
    output [C_M_AXI_ADDR_WIDTH-9:0] o_qp_stat_rq_buf_ca,

    input                         i_qp_rq_depth_req,
    output                        o_qp_rq_depth_valid,
    output [                15:0] o_qp_rq_depth,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_rq_depth_idx,

    // 提供给 ACK 管理器的 CQ 深度信息。 
    // The CQ depth is equal to SQ depth. This information is shared by QP cache
    // QP 管理器内部更新与 ACK 管理器更新共享该路径，cache manager 优先。

    input                         i_qp_cq_depth_req,
    output                        o_qp_cq_depth_valid,
    output [                15:0] o_qp_cq_depth,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_cq_depth_idx,

    input                         i_qp_sq_psn_req,
    input                         i_qp_sq_psn_wqe_req,
    input                         i_qp_sq_psn_resp_req,
    output                        o_qp_sq_psn_valid,
    output                        o_qp_sq_psn_wqe_valid,
    output                        o_qp_sq_psn_resp_valid,
    output [                23:0] o_qp_sq_psn,
    input                         i_qp_sq_psn_wrn,
    input  [                23:0] i_qp_sq_psn,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_idx,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_wqe_idx,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_sq_psn_resp_idx,

    output [                31:0] o_qp_last_req,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_last_req_idx,
    input  [                31:0] i_qp_last_req,
    input                         i_qp_last_req_req,
    input                         i_qp_last_req_wrn,
    output                        o_qp_last_req_valid,

    input                         i_qp_dest_qpid_req_tx_engine,
    input                         i_qp_dest_qpid_req_rx_engine,
    output                        o_qp_dest_qpid_valid_tx_engine,
    output                        o_qp_dest_qpid_valid_rx_engine,
    output [                23:0] o_qp_dest_qpid,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_dest_qpid_idx,

    output                        o_qp_disable_pulse_valid,
    output [C_QP_INDX_WIDTH -1:0] o_qp_disable_pulse_id,
    output                        o_qp_disable_pulse_data,

    output [C_M_AXI_ADDR_WIDTH -1:0] o_in_errsts_q_ba,
    output [                   15:0] o_in_errsts_q_sz,
    input  [                   15:0] i_in_errsts_q_wrptr,

    input                         i_qp_mac_remote_addrl_req,
    input                         i_qp_mac_remote_addrl_wqe_req,
    output                        o_qp_mac_remote_addrl_valid,
    output                        o_qp_mac_remote_addrl_wqe_valid,
    output [                31:0] o_qp_mac_remote_addrl,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_idx,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_wqe_idx,    // TX WQE 路径读取 MACDESADDLSBi 的 QP 索引

    input                         i_qp_mac_remote_addrl_replica_req,
    output [                31:0] o_qp_mac_remote_addrl_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrl_replica_idx,   // MACDESADDLSBi replica 读取的 QP 索引
    output                        o_qp_mac_remote_addrl_replica_valid,

    input                         i_qp_mac_remote_addrm_req,
    input                         i_qp_mac_remote_addrm_wqe_req,
    output                        o_qp_mac_remote_addrm_valid,
    output                        o_qp_mac_remote_addrm_wqe_valid,
    output [                31:0] o_qp_mac_remote_addrm,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_idx,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_wqe_idx,    // TX WQE 路径读取 MACDESADDMSBi 的 QP 索引

    output [                31:0] o_qp_mac_remote_addrm_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_mac_remote_addrm_replica_idx,   // MACDESADDMSBi replica 读取的 QP 索引
    input                         i_qp_mac_remote_addrm_replica_req,
    output                        o_qp_mac_remote_addrm_replica_valid,

    input                         i_qp_ip_remote_addr1_req,
    output                        o_qp_ip_remote_addr1_valid,
    input                         i_qp_ip_remote_addr1_wqe_req,
    output                        o_qp_ip_remote_addr1_wqe_valid,
    output [                31:0] o_qp_ip_remote_addr1,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_idx,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_wqe_idx,    // TX WQE 路径读取 IPDESADDR1i 的 QP 索引

    output [                31:0] o_qp_ip_remote_addr1_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr1_replica_idx,   // IPDESADDR1i replica 读取的 QP 索引
    input                         i_qp_ip_remote_addr1_replica_req,
    output                        o_qp_ip_remote_addr1_replica_valid,

    input                         i_qp_ip_remote_addr2_req,
    input                         i_qp_ip_remote_addr2_wqe_req,
    output                        o_qp_ip_remote_addr2_valid,
    output                        o_qp_ip_remote_addr2_wqe_valid,
    output [                31:0] o_qp_ip_remote_addr2,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_idx,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_wqe_idx,    // TX WQE 路径读取 IPDESADDR2i 的 QP 索引

    output [                31:0] o_qp_ip_remote_addr2_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr2_replica_idx,   // IPDESADDR2i replica 读取的 QP 索引
    input                         i_qp_ip_remote_addr2_replica_req,
    output                        o_qp_ip_remote_addr2_replica_valid,

    input                         i_qp_ip_remote_addr3_req,
    output                        o_qp_ip_remote_addr3_valid,
    input                         i_qp_ip_remote_addr3_wqe_req,
    output                        o_qp_ip_remote_addr3_wqe_valid,
    output [                31:0] o_qp_ip_remote_addr3,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_idx,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_wqe_idx,    // TX WQE 路径读取 IPDESADDR3i 的 QP 索引

    input                         i_qp_ip_remote_addr3_replica_req,
    output                        o_qp_ip_remote_addr3_replica_valid,
    output [                31:0] o_qp_ip_remote_addr3_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr3_replica_idx,    // IPDESADDR3i replica 读取的 QP 索引

    input                         i_qp_ip_remote_addr4_req,
    output                        o_qp_ip_remote_addr4_valid,
    input                         i_qp_ip_remote_addr4_wqe_req,
    output                        o_qp_ip_remote_addr4_wqe_valid,
    output [                31:0] o_qp_ip_remote_addr4,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_idx,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_wqe_idx,    // TX WQE 路径读取 IPDESADDR4i 的 QP 索引

    input                         i_qp_ip_remote_addr4_replica_req,
    output                        o_qp_ip_remote_addr4_replica_valid,
    output [                31:0] o_qp_ip_remote_addr4_replica,
    input  [C_QP_INDX_WIDTH -1:0] i_qp_ip_remote_addr4_replica_idx,    // IPDESADDR4i replica 读取的 QP 索引

    input                       i_waiting_for_rq_update,
    input [C_QP_INDX_WIDTH-1:0] i_waiting_for_rq_update_on_q_num,


    input [31:0] i_last_in_pkt_info,
    input [31:0] i_last_out_pkt_info,

    // 用于重传逻辑的 ACK 管理器接口
    input  [C_QP_INDX_WIDTH -1 : 0] i_retransmit_qpid,          // 重传路径选中的 QP id
    input                           i_retransmit_reqd,          // Retransmission is required
    input  [                  55:0] i_retransmit_wqe_bram_din,
    output                          o_retransmit_accepted,      // QP 管理器已接收重传请求
    input  [                  23:0] i_psn_to_retry,
    input  [                  23:0] i_ssn_to_retry,

    input                       i_qp_fatal_err_valid,  // QPs those are stalled due to fatal error
    input [C_QP_INDX_WIDTH-1:0] i_qp_fatal_err_id,
    input                       i_qp_fatal_err_data,

    output                         o_qp_clr_fatal_err_valid,
    output [ C_QP_INDX_WIDTH -1:0] o_qp_clr_fatal_err_id,
    output                         o_qp_clr_fatal_err_data,
    output                         o_qp_under_recovery_valid,
    output [ C_QP_INDX_WIDTH -1:0] o_qp_under_recovery_id,
    output                         o_qp_under_recovery_data,
    input                          i_qp_wq_cmpl_intr_or_reduce,
    input  [                 31:0] i_qp_wq_cmpl_intr_wr_en,
    input  [                 31:0] i_qp_wq_cmpl_intr_wr_data,
    input  [C_QP_INDX_WIDTH-1 : 0] i_qp_wq_cmpl_intr_wr_addr,
    input  [                 31:0] i_qp_wq_cmpl_intr_wr_en_pending,
    input  [C_QP_INDX_WIDTH-1 : 0] i_qp_wq_cmpl_intr_wr_addr_pending,
    output [                 31:0] o_cq_intr_sts_clr_wr_en,
    output [  C_QP_INDX_WIDTH-1:0] o_cq_intr_sts_clr_wr_addr,

    //Pause_Offset
    output [ 3:0] roce_priority,
    output [ 3:0] non_roce_priority,
    output        pfc_en_roce,
    output        pfc_en_non_roce,
    output [15:0] roce_pause_on_threshold,
    output [15:0] roce_pause_off_threshold,
    output        disable_priority_chk,



    // AXI Master 信号组
    // ----------------------------------------------------------
    // Write address channel
    output [                   0:0] m_axi_awid,
    output [C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output [                   7:0] m_axi_awlen,
    output [                   2:0] m_axi_awsize,
    output [                   1:0] m_axi_awburst,
    output [                   3:0] m_axi_awcache,
    output [                   2:0] m_axi_awprot,
    output                          m_axi_awvalid,
    input                           m_axi_awready,
    // Write data channel
    output [                 511:0] m_axi_wdata,
    output [             512/8-1:0] m_axi_wstrb,
    output                          m_axi_wlast,
    output                          m_axi_wvalid,
    input                           m_axi_wready,
    output                          m_axi_awlock,
    // Write Response channel
    input  [                 0 : 0] m_axi_bid,
    input  [                   1:0] m_axi_bresp,
    input                           m_axi_bvalid,
    output                          m_axi_bready,
    // Read address channel
    output [                   0:0] m_axi_arid,
    output [C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output [                   7:0] m_axi_arlen,
    output [                   2:0] m_axi_arsize,
    output [                   1:0] m_axi_arburst,
    output [                   3:0] m_axi_arcache,
    output [                   2:0] m_axi_arprot,
    output                          m_axi_arvalid,
    input                           m_axi_arready,
    // Read data/response channel
    input  [                   0:0] m_axi_rid,
    input  [                 511:0] m_axi_rdata,
    input  [                   1:0] m_axi_rresp,
    input                           m_axi_rlast,
    input                           m_axi_rvalid,
    output                          m_axi_rready,
    output                          m_axi_arlock,

    input                         i_osq_pop,
    input [C_QP_INDX_WIDTH - 1:0] i_osq_pop_qpid,
    input                         i_wqe_dropped,
    input [C_QP_INDX_WIDTH - 1:0] i_wqe_dropped_qpid,
    input                         init_start,
    input [C_QP_INDX_WIDTH-1 : 0] init_count

);

    `include "lrdma_defs.vh"


    localparam WQE_SIZE = 14'd512;  // WQE 条目大小为 64 Bytes，即 512 bits
    localparam WQE_SIZE_IN_BYTES = WQE_SIZE / 8;
    localparam WQE_FIFO_DEPTH = 3'd4;  //不要超过 4。参见下文*。与 TX 引擎接口的 FIFO 深度
    localparam WQE_FIFO_IDX_WIDTH = 2;
    localparam OS_Q_ADDR_WIDTH = (clog2(C_OS_Q_DEPTH * C_NUM_QP));

    // SQ WQE FIFO 向 TX 引擎提供 WQE 数据。该缓存用于支持 WQE 的连续处理。
    // 由于采用 FIFO 机制，若某条 WQE 因 ACK 管理器中该 QP 的 OSQ 已满而无法被 TX 引擎继续处理，则后续 WQE 条目也会被阻塞。
    // 因此仲裁模块会接收osq_full信号，当QP触发该信号时将被排除在仲裁之外。
    // 为确保该逻辑始终正常运行，当FIFO中存在足够空间容纳所有WQE时，应断言almost_full信号。
    // OSQ 深度为 16；若 WQE FIFO 深度超过 4，在仅连接单个 QP 的情况下，OSQ可能严重闲置。

    wire                             halt;
    wire                             sw_override_en;
    wire [      C_QP_INDX_WIDTH-1:0] sw_override_qpid;
    wire [                     15:0] qp_cq_hdptr_from_config;
    wire [   WQE_FIFO_IDX_WIDTH : 0] num_valid_entries;  // SQ WQE FIFO中有效条目的数量
    wire [WQE_FIFO_IDX_WIDTH -1 : 0] wqe_fifo_rd_ptr;  // 从0开始
    wire [WQE_FIFO_IDX_WIDTH -1 : 0] wqe_fifo_wr_ptr;  // 从0开始
    wire                             wqe_full;  // SQ WQE FIFO满信号
    wire                             sq_empty_valid;
    wire [   C_QP_INDX_WIDTH -1 : 0] sq_empty_id;
    wire                             sq_empty_data;
    wire                             arbitrate;  // 仲裁请求
    wire                             sample_arbitrated_sq;
    wire                             arbitration_done;  // 仲裁完成
    wire [                      2:0] cache_fsm_status;
    wire [                     15:0] stat_wqe_cnt;
    wire [                    511:0] original_wqe_data;  // 刚从 DDR 读取的 SQ WQE 数据
    wire                             original_wqe_dvalid;
    reg  [                    511:0] original_wqe_data_r1;
    reg                              original_wqe_dvalid_r1;
    reg                              original_wqe_dvalid_r2;
    wire                             original_wqe_arready;
    wire [   C_M_AXI_ADDR_WIDTH-1:0] qp_sq_ba_int;
    wire                             axi_ren_valid;
    wire [   C_QP_INDX_WIDTH -1 : 0] qp_sq_arbitrated_idx;  // 通过仲裁选择出来的QP SQ索引
    wire [                     15:0] qp_sq_pidb;
    wire                             qp_curr_sqptr_req;
    wire                             qp_sq_depth_req;
    wire                             qp_sq_depth_valid;
    wire                             qp_sq_ba_req_int;
    wire                             qp_sq_ba_valid_int;
    wire [                     15:0] new_curr_sqptr_proc;
    wire [                     15:0] qp_curr_sqptr_proc;
    wire [                     15:0] qp_sq_depth;
    wire [  C_M_AXI_ADDR_WIDTH -1:0] sq_read_addr_to_axi;
    reg  [                    511:0] wqe_qid_inserted;
    wire                             qp_cq_hdptr_req_retry;
    wire                             qp_cq_hdptr_valid_retry;
    wire [     C_QP_INDX_WIDTH -1:0] qp_cq_hdptr_idx_retry;
    wire [                     15:0] qp_curr_sqptr_proc_retry;
    wire                             qp_curr_sqptr_proc_wen_retry;
    wire [     C_QP_INDX_WIDTH -1:0] qp_curr_sqptr_proc_idx_retry;  // 重传路径写回 SQ pointer 的 QP 索引
    wire                             qp_sq_psn_req_retry;
    wire                             qp_sq_psn_wrn_retry;
    wire [                     23:0] qp_sq_psn_retry;
    wire [     C_QP_INDX_WIDTH -1:0] qp_sq_psn_idx_retry;  // 重传路径访问 SQPSNi 的 QP 索引
    wire [     C_QP_INDX_WIDTH -1:0] qp_stat_ssn_idx_retry;  // 重传路径访问 STATSSNi 的 QP 索引
    wire [                     23:0] qp_stat_ssn_retry;  // 重传路径写回的发送侧 SSN
    wire                             qp_stat_ssn_req_retry;
    wire                             qp_stat_ssn_wrn_retry;
    wire [                    511:0] wqe_from_fifo;
    wire [                    511:0] wqe_to_fifo;
    wire [                    511:0] cached_wqe;
    wire                             wqe_fifo_push_retry;
    wire                             wqe_fifo_pop_retry;
    wire                             cache_halted;
    wire                             qp_sq_psn_retry_valid;
    wire                             qp_stat_wqe_cnt_valid;
    wire                             qp_stat_wqe_cnt_rdreq;
    wire [                     15:0] qp_stat_wqe_cnt;
    wire [                     15:0] new_qp_stat_wqe_cnt;

    wire [                     56:0] retransmit_wqe_bram_din_a;
    wire [                     56:0] retransmit_wqe_bram_dout_a;
    wire                             retransmit_wqe_bram_wen_a;
    wire [      C_QP_INDX_WIDTH-1:0] retransmit_wqe_bram_idx_a;
    wire [                     56:0] retransmit_wqe_bram_dout_b;

    // QP disable 时清零重传 BRAM 状态，防止过期 retransmit_wqe 翻转标志被使用
    wire                             qp_disable_retrans_clear;
    wire                             retransmit_wqe_bram_wen_mux;
    wire [      C_QP_INDX_WIDTH-1:0] retransmit_wqe_bram_idx_mux;
    wire [                     56:0] retransmit_wqe_bram_din_mux;

    reg  [                      7:0] axi_rd_err_cnt_ff;
    reg  [   C_QP_INDX_WIDTH -1 : 0] qp_sq_arbitrated_idx_ff;
    reg                              axi_ren_valid_ff;
    wire                             wqe_axi_m_en;
    reg                              wqe_empty_ff;
    reg  [                    511:0] wqe_bypass_data;
    reg                              wqe_bypass_valid;
    reg                              wqe_bypass_pop_seen;
    wire                             wqe_bypass_fire;
    wire                             wqe_bypass_active;
    wire                             wqe_bypass_retry_ctx;
    localparam WQE_RETRANSMIT_BIT = 152;
    localparam WQE_RETRIED_BIT = 16;

    wire [                 23:0] qp_stat_ret_sq_psn_int;
    wire [ C_QP_INDX_WIDTH -1:0] qp_stat_ret_sq_psn_idx_int;

    wire [  C_QP_INDX_WIDTH-1:0] wqe_qpid_from_qp_cache;
    wire [  C_QP_INDX_WIDTH-1:0] original_wqe_qid;  // 仲裁后的 SQ QPID
    reg  [  C_QP_INDX_WIDTH-1:0] original_wqe_qid_r1;

    wire                         retransmit_wqe_dropped;
    wire [C_QP_INDX_WIDTH - 1:0] retransmit_wqe_dropped_qpid;


    wire [                  2:0] arb_reqs;
    wire [                  2:0] arb_gnt;
    wire                         arb_done;

    wire                         osq_pop_fifo_empty;
    wire                         osq_pop_fifo_rden;
    wire [C_QP_INDX_WIDTH - 1:0] osq_pop_fifo_dout;

    wire                         in_wqe_dropped_fifo_empty;
    wire                         in_wqe_dropped_fifo_rden;
    wire [C_QP_INDX_WIDTH - 1:0] in_wqe_dropped_fifo_dout;

    wire                         retransmit_wqe_fifo_empty;
    wire                         retransmit_wqe_fifo_rden;
    wire [C_QP_INDX_WIDTH - 1:0] retransmit_wqe_fifo_dout;

    wire                         arb_gnt_popen;
    reg  [C_QP_INDX_WIDTH - 1:0] arb_gnt_pop_qpid;

    wire                         retrans_check_done;
    reg                          retrans_check_update;
    wire [                 63:0] retransmit_local_addr;
    wire [                 63:0] retransmit_remote_addr;
    wire [                 31:0] retransmit_dma_len;
    wire                         osq_full_set_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_full_set_id;
    wire                         osq_full_reset_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_full_reset_id;
    wire                         sq_empty_and_reduce;  // 所有 QP 的 SQ（发送队列）是否都为空
    wire                         prog_full_db_update_curr_sqptr;

    wire                         update_qp_stat_wqe_cnt;
    wire                         qp_stat_ret_sq_psn_wen_int;
    wire                         qp_stat_ret_sq_psn_valid_int;

    wire                         qp_sq_pidb_req;
    wire                         qp_sq_pidb_valid;

    wire                         update_curr_sqptr;
    wire                         qp_curr_sqptr_proc_valid;
    wire                         wqe_empty;

    wire                         lrdma_axi_master_done;
    wire                         lrdma_axi_master_error_nc;

    // 如果 ACK 管理器正在写 CQ head pointer，本拍不给重传控制器读权限。
    // 否则重传控制器可以使用 CQ head pointer 端口。
    wire                         qp_cq_hdptr_retry_grant;
    assign qp_cq_hdptr_retry_grant = qp_cq_hdptr_req_retry & ~i_qp_cq_hdptr_wrn;

    // 配置寄存器
    lrdma_reg_block #(
        .C_S_AXI_LITE_ADDR_WIDTH(C_S_AXI_LITE_ADDR_WIDTH),
        .C_S_AXI_LITE_DATA_WIDTH(C_S_AXI_LITE_DATA_WIDTH),
        .C_M_AXI_ADDR_WIDTH     (C_M_AXI_ADDR_WIDTH),
        .C_NUM_QP               (C_NUM_QP),
        .C_QP_INDX_WIDTH        (C_QP_INDX_WIDTH),
        .STB_WIDTH              (STB_WIDTH)
    ) inst_reg_block (
        // AXI Lite 信号组
        // ----------------------------------------------------------
        .s_axi_lite_aclk   (s_axi_lite_aclk),
        .s_axi_lite_aresetn(s_axi_lite_aresetn),

        .s_axi_lite_awaddr (s_axi_lite_awaddr),
        .s_axi_lite_awready(s_axi_lite_awready),
        .s_axi_lite_awvalid(s_axi_lite_awvalid),

        .s_axi_lite_araddr (s_axi_lite_araddr),
        .s_axi_lite_arready(s_axi_lite_arready),
        .s_axi_lite_arvalid(s_axi_lite_arvalid),

        .s_axi_lite_wdata (s_axi_lite_wdata),
        .s_axi_lite_wstrb (s_axi_lite_wstrb),
        .s_axi_lite_wready(s_axi_lite_wready),
        .s_axi_lite_wvalid(s_axi_lite_wvalid),

        .s_axi_lite_rdata (s_axi_lite_rdata),
        .s_axi_lite_rresp (s_axi_lite_rresp),
        .s_axi_lite_rready(s_axi_lite_rready),
        .s_axi_lite_rvalid(s_axi_lite_rvalid),

        .s_axi_lite_bresp (s_axi_lite_bresp),
        .s_axi_lite_bready(s_axi_lite_bready),
        .s_axi_lite_bvalid(s_axi_lite_bvalid),
        // ----------------------------------------------------------

        // 来自/前往头部验证模块的中断信号
        .i_pkt_valdn_err_intr(i_pkt_valdn_err_intr),
        .i_mad_pkt_rcvd_intr (i_mad_pkt_rcvd_intr),

        // Interrupt Status 寄存器
        // 该状态寄存器提供断言中断的原因。
        .o_intr_clr_pkt_valdn_err(o_intr_clr_pkt_valdn_err),  // INTSTS[0]: 传入数据包验证错误中断
        .o_intr_clr_mad_pkt_rcvd (o_intr_clr_mad_pkt_rcvd),   // INTSTS[1]: 收到传入的 MAD 数据包中断
        .o_intr_clr_wqe_cmpl     (o_intr_clr_wqe_cmpl),       // INTSTS[2]: WQE 完成中断（对 QPCONF[3] 位被设置的 QP 断言）
        .o_intr_clr_fatal_err    (o_intr_clr_fatal_err),      // INTSTS[3]: qp致命错误中断，只要有一个QP发生致命错误就会触发

        // Interrupt Enable 寄存器
        // 该寄存器为 LRDMA IP 产生的中断提供配置控制。
        .o_intr_en_pkt_valdn_err(o_intr_en_pkt_valdn_err),  // INTEN[0]: 传入数据包验证错误中断启用
        .o_intr_en_mad_pkt_rcvd (o_intr_en_mad_pkt_rcvd),   // INTEN[1]: 接收到 MAD 数据包中断启用
        .o_intr_en_wqe_cmpl     (o_intr_en_wqe_cmpl),       // INTEN[2]: WQE 完成中断启用
        .o_intr_en_fatal_err    (o_intr_en_fatal_err),      // INTEN[3]: 收到致命错误中断启用

        // Completion Q interrupt status (bitwise) 寄存器（IP核写，上位机读）
        .i_qp_wq_cmpl_intr_or_reduce      (i_qp_wq_cmpl_intr_or_reduce),
        .i_qp_wq_cmpl_intr_wr_en          (i_qp_wq_cmpl_intr_wr_en),
        .i_qp_wq_cmpl_intr_wr_addr        (i_qp_wq_cmpl_intr_wr_addr),
        .i_qp_wq_cmpl_intr_wr_data        (i_qp_wq_cmpl_intr_wr_data),
        .i_qp_wq_cmpl_intr_wr_en_pending  (i_qp_wq_cmpl_intr_wr_en_pending),
        .i_qp_wq_cmpl_intr_wr_addr_pending(i_qp_wq_cmpl_intr_wr_addr_pending),
        // Completion Q interrupt status (bitwise) 寄存器（上位机写，IP核读）
        .o_cq_intr_sts_clr_wr_en          (o_cq_intr_sts_clr_wr_en),            // CQINTSTSn[31:0]: 该寄存器提供 QP 0 至 31 的 CQ 中断状态。位[i] 提供 CQi 的中断状态，其中 i=0 至 31。
        .o_cq_intr_sts_clr_wr_addr        (o_cq_intr_sts_clr_wr_addr),          // CQINTSTSn中的n，n=1说明 o_cq_intr_sts_clr_wr_en 是 CQINTSTS1 的值，表明对应 QP0-31 的 CQ 中断状态。

        .i_qp_fatal_err_valid(i_qp_fatal_err_valid),
        .i_qp_fatal_err_id   (i_qp_fatal_err_id),
        .i_qp_fatal_err_data (i_qp_fatal_err_data),

        .o_qp_clr_fatal_err_valid(o_qp_clr_fatal_err_valid),  // qp_conf_bram_wen == 1'b1时，o_qp_clr_fatal_err_valid <= 1'b1，保持两个周期
        .o_qp_clr_fatal_err_id   (o_qp_clr_fatal_err_id),     // qp idx
        .o_qp_clr_fatal_err_data (o_qp_clr_fatal_err_data),   // ~QPCONFi[0]: ~QP enable

        .o_qp_under_recovery_valid(o_qp_under_recovery_valid),  // qp_conf_bram_wen == 1'b1时，o_qp_under_recovery_valid <= 1'b1，保持两个周期
        .o_qp_under_recovery_id   (o_qp_under_recovery_id),     // qp idx
        .o_qp_under_recovery_data (o_qp_under_recovery_data),   // QPCONFi[6]: QP under recovery

        .i_rq_full_valid         (i_rq_full_valid),
        .i_rq_full_idx           (i_rq_full_idx),
        .i_rq_full_data          (i_rq_full_data),
        .i_rq_full_reset_valid   (i_rq_full_reset_valid),
        .i_rq_full_reset_idx     (i_rq_full_reset_idx),
        .i_rq_full_reset_data    (i_rq_full_reset_data),
        .i_osq_full_set_valid    (i_osq_full_set_valid),
        .i_osq_full_set_id       (i_osq_full_set_id),
        .i_osq_full_reset_valid  (i_osq_full_reset_valid),
        .i_osq_full_reset_id     (i_osq_full_reset_id),
        .i_osq_empty_set_valid   (i_osq_empty_set_valid),
        .i_osq_empty_set_id      (i_osq_empty_set_id),
        .i_osq_empty_reset_valid (i_osq_empty_reset_valid),
        .i_osq_empty_reset_id    (i_osq_empty_reset_id),
        .i_qp_retried_set_valid  (i_qp_retried_set_valid),
        .i_qp_retried_set_id     (i_qp_retried_set_id),
        .i_qp_retried_reset_valid(i_qp_retried_reset_valid),
        .i_qp_retried_reset_id   (i_qp_retried_reset_id),
        .i_stat_retry_cnt        (i_stat_retry_cnt),

        // Status Sender sequence number QPi 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
        .i_qp_stat_ssn_idx  ((qp_stat_ssn_req_retry | qp_stat_ssn_wrn_retry) ? qp_stat_ssn_idx_retry : i_qp_stat_ssn_idx),  // addrb
        .i_qp_stat_ssn      (qp_stat_ssn_wrn_retry ? qp_stat_ssn_retry : i_qp_stat_ssn),                                    // STATSSNi[23:0]: 发送侧 SSN（与发出的 MSN 相同）
        .o_qp_stat_ssn      (o_qp_stat_ssn),                                                                                // STATSSNi[23:0]: 发送侧 SSN（与发出的 MSN 相同）
        .i_qp_stat_ssn_req  (i_qp_stat_ssn_req | qp_stat_ssn_req_retry),                                                    // 可在 WQE 停止时进行 OR 运算
        .i_qp_stat_ssn_wrn  (i_qp_stat_ssn_wrn | qp_stat_ssn_wrn_retry),                                                    // web, 可在 WQE 停止时进行 OR 运算
        .o_qp_stat_ssn_valid(o_qp_stat_ssn_valid),                                                                          // valid 由 req 延后一拍产生

        // Timeout Configuration Register for QPi (TIMEOUTCONFi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成(其中 i = 2 至 C_NUM_QP）。它提供 QPi 的超时配置。QP1 不存在该寄存器。
        .i_qp_timeout_idx(i_qp_timeout_idx),  // addrb
        .o_qp_timeout    (o_qp_timeout),      // [10:8]: 最大重试次数；[4:0]: 超时值；其余位无功能定义

        // Status RQ Producer index DB QPi (STATRQPIDBi) 寄存器 (Read only)
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。
        .i_qp_rq_pi_db_idx  (i_qp_rq_pi_db_idx),   // addrb
        .i_qp_rq_pi_db      (i_qp_rq_pi_db),       // [15:0]: RCV Q 生产者索引 DB
        .o_qp_rq_pi_db      (o_qp_rq_pi_db),       // [15:0]: RCV Q 生产者索引 DB
        .i_qp_rq_pi_db_req  (i_qp_rq_pi_db_req),
        .i_qp_rq_pi_db_wrn  (i_qp_rq_pi_db_wrn),   // web
        .o_qp_rq_pi_db_valid(o_qp_rq_pi_db_valid), // valid 由 req 延后一拍产生

        // Status of WQEs posted to QPi (STATWQEi) 寄存器 (Read only)
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。
        .i_qp_stat_wqe_cnt      (new_qp_stat_wqe_cnt),      // [15:0]: QP MGR 为该 QP 推送的 WQE 计数
        .i_qp_stat_wqe_cnt_wen  (update_qp_stat_wqe_cnt),   // web
        .i_qp_stat_wqe_cnt_idx  (qp_sq_arbitrated_idx_ff),  // addrb
        .o_qp_stat_wqe_cnt_valid(qp_stat_wqe_cnt_valid),    // valid 由 req 延后一拍产生
        .i_qp_stat_wqe_cnt_rdreq(qp_stat_wqe_cnt_rdreq),
        .o_qp_stat_wqe_cnt      (qp_stat_wqe_cnt),          // [15:0]: QP MGR 为该 QP 推送的 WQE 计数

        .i_qp_ret_sq_psn_idx      (i_qp_stat_ret_sq_psn_req ? i_qp_stat_ret_sq_psn_idx : qp_stat_ret_sq_psn_idx_int),
        .i_qp_ret_sq_psn          (qp_stat_ret_sq_psn_int),
        .o_qp_ret_sq_psn          (o_qp_stat_ret_sq_psn),
        .i_qp_ret_sq_psn_req      (i_qp_stat_ret_sq_psn_req),
        .i_qp_ret_sq_psn_wrn      (i_qp_stat_ret_sq_psn_req ? 1'b0 : qp_stat_ret_sq_psn_wen_int),
        .o_qp_ret_sq_psn_valid    (o_qp_stat_ret_sq_psn_valid),
        .o_qp_ret_sq_psn_valid_int(qp_stat_ret_sq_psn_valid_int),                                                      // internal from QPM

        // LRDMA Configuration (LRDMACONF) 寄存器
        // 该寄存器为 LRDMA IP 提供基本的全局（非 QP 专用）配置控制。
        .o_lrdma_en          (o_lrdma_en),           // LRDMACONF[0]: LRDMA 启用
        .o_err_buf_en        (o_err_buf_en),         // LRDMACONF[5]: 启用错误缓冲区
        .o_flow_credits      (o_flow_credits),       // LRDMACONF[7:6]: 无功能定义
        .o_lrdma_udp_src_port(o_lrdma_udp_src_port), // LRDMACONF[23:8]: 外发数据包的 UDP 源端口，即 LRDMA UDP 输出端口

        // LRDMA Advance Configuration (LRDMAADCONF) 寄存器
        .o_lrdma_adv_conf_sw_override        (sw_override_en),                       // LRDMAADCONF[0]: 允许 SW 写以下只读寄存器 - CQHEADi, STATCURSQPTRi 和 STATRQPIDBi (其中 i 为 QP 索引)
        .o_lrdma_adv_conf_sw_override_qpid   (sw_override_qpid),                     // LRDMAADCONF[30:20]: 软件覆盖 QP 编号
        .i_sw_override_per_qp_en             ({sw_override_en, sw_override_qpid}),
        .o_lrdma_adv_conf_retry_cnt_fatal_dis(o_lrdma_adv_conf_retry_cnt_fatal_dis), // LRDMAADCONF[2]: retry_cnt_fatal_dis

        .o_num_qp_en      (o_num_qp_en),        // LRDMA_CONF_QP_EN[10:0]: 启用的 QP 数量；该寄存器包含多少个 QP 将被激活的信息。
        .o_mac_lrdma_addr (o_mac_lrdma_addr),   // MACXADD[47:0]: 本地 (LRDMA) 设备的 MAC 地址，应先配置该地址再置位 LRDMACONF[0]。
        .o_ipv4_lrdma_addr(o_ipv4_lrdma_addr),  // IPv4XADD[31:0]: 本地 (LRDMA) 设备的 IPv4 地址。
        .o_ipv6_lrdma_addr(o_ipv6_lrdma_addr),  // IPv6XADD[127:0]: 本地（LRDMA）设备的 IPv6 地址。应先配置该地址再置位 LRDMACONF[0]。

        // rq_err_pkt_buf: 错误缓冲区
        // LRDMA IP 会根据未通过验证的数据包更新这些缓冲区。通过向 LRDMACONF[5]写 1，可启用向该缓冲区写入所有验证错误信息。如果禁用该位，则只有导致 QP 进入 FATAL 状态的数据包才会写入错误缓冲区。
        .o_rq_err_pkt_buf_ba         (o_rq_err_pkt_buf_ba),           // ERRBUFBA[63:0]: 错误缓冲区基地址
        // 该寄存器提供错误缓冲区的大小
        .o_rq_err_pkt_buf_sz_num_bufs(o_rq_err_pkt_buf_sz_num_bufs),  // ERRBUFSZ[15:0]: 错误缓冲区数量
        .o_rq_err_pkt_buf_sz_buf_sz  (o_rq_err_pkt_buf_sz_buf_sz),    // ERRBUFSZ[31:16]: 每个错误缓冲区的大小
        .i_rq_err_pkt_buf_wrptr      (i_rq_err_pkt_buf_wrptr),        // 错误缓冲区写指针，指向下一个可用的错误缓冲区

        .i_inc_ack_pkt_cnt    (i_inc_ack_pkt_cnt),
        .i_inc_mad_pkt_cnt    (i_inc_mad_pkt_cnt),
        .i_inc_inv_pkt_cnt    (i_inc_inv_pkt_cnt),
        .i_inc_all_dropped_cnt(i_inc_all_dropped_cnt),
        .i_inc_nack_cnt       (i_inc_nack_cnt),

        .i_out_wr_pkt_cnt (i_out_wr_pkt_cnt),
        .i_out_mad_pkt_cnt(i_out_mad_pkt_cnt),
        .i_ack_mgr_sts    (i_ack_mgr_sts),

        .qp_enable_valid(o_qp_enable_valid),  // qp_conf_bram_wen == 1'b1时，o_qp_enable_valid <= 1'b1，保持一个周期
        .qp_enable_id   (o_qp_enable_id),     // qpid
        .qp_enable_data (o_qp_enable_data),   // QPCONFi[0]: QP enable

        // LRDMA_PAUSE_CONF 寄存器
        // 该寄存器提供 PFC 配置。
        .pfc_en_roce         (pfc_en_roce),          // LRDMA_PAUSE_CONF[0]: 暂停启用 RoCE
        .pfc_en_non_roce     (pfc_en_non_roce),      // LRDMA_PAUSE_CONF[1]: 暂停启用非 RoCE
        .roce_priority       (roce_priority),        // LRDMA_PAUSE_CONF[7:4]: RoCE 的暂停优先级（二进制编码）
        .non_roce_priority   (non_roce_priority),    // LRDMA_PAUSE_CONF[11:8]: 非 RoCE（二进制编码）的暂停优先级
        .disable_priority_chk(disable_priority_chk), // LRDMA_PAUSE_CONF[13]: 禁用优先级检查（仅适用于 TX 数据路径）

        // LRDMA_BUF_THRESHOLD_ROCE 寄存器
        // 该寄存器为 RoCE 流量提供 XON 和 XOFF 配置。
        .roce_pause_on_threshold (roce_pause_on_threshold),  // LRDMA_BUF_THRESHOLD_ROCE[15:0]: RoCE 的 XON 值
        .roce_pause_off_threshold(roce_pause_off_threshold), // LRDMA_BUF_THRESHOLD_ROCE[31:16]: RoCE 的 XOFF 值

        // sq_empty_*报告某一个 QP 的 SQ 空状态发生变化
        .o_sq_empty_valid      (sq_empty_valid),      // sq_empty_*有效信号
        .o_sq_empty_id         (sq_empty_id),         // qpid
        .o_sq_empty_data       (sq_empty_data),       // 1-空，0-非空
        .sq_empty_ff_and_reduce(sq_empty_and_reduce), // 所有 QP 的 SQ 是否都为空

        .core_clk (core_clk),  // Module Clock  
        .core_rstn(core_rstn), // Active high core reset

        .i_waiting_for_rq_update         (i_waiting_for_rq_update),          // 等待更新 RQ 
        .i_waiting_for_rq_update_on_q_num(i_waiting_for_rq_update_on_q_num), // 等待更新 RQ 的 QP 编号

        .i_qp_conf_req_tx_engine  (i_qp_conf_req_tx_engine),    // TX 引擎获取 QP 配置请求
        .i_qp_conf_req_ack_mgr    (i_qp_conf_req_ack_mgr),      // ACK 管理器获取 QP 配置请求
        .i_qp_conf_req_rx_engine  (i_qp_conf_req_rx_engine),    // RX 引擎获取 QP 配置请求
        .o_qp_conf_valid_tx_engine(o_qp_conf_valid_tx_engine),  // 响应 TX 引擎的 QP 配置请求
        .o_qp_conf_valid_ack_mgr  (o_qp_conf_valid_ack_mgr),    // 响应 ACK 管理器的 QP 配置请求
        .o_qp_conf_valid_rx_engine(o_qp_conf_valid_rx_engine),  // 响应 RX 引擎的 QP 配置请求
        // QP Configuration QPi (QPCONFi) 寄存器
        // 该寄存器定义 QP 的基本配置。该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。每个 QP 的寄存器偏移量为 0x18_0000、0x18_0004，随后的偏移量依此类推。
        .o_qp_conf                (o_qp_conf),
        .i_qp_conf_idx            (i_qp_conf_idx),              // 读取 Qp 配置的索引

        .i_qp_cfg_retry_cnt_rd_req(i_qp_cfg_retry_cnt_rd_req),  // ACK 管理器读取 TIMEOUTCONFi 重试次数配置的请求
        .o_qp_cfg_retry_cnt_rd_rdy(o_qp_cfg_retry_cnt_rd_rdy),  // TIMEOUTCONFi 重试次数配置读响应有效

        // QP configuration QPi (QPCONFi) 寄存器复制寄存器
        .o_qp_conf_replica      (o_qp_conf_replica),       // QP configuration REPLICA
        .i_qp_conf_replica_idx  (i_qp_conf_replica_idx),   // 读取 QPCONFi 复制寄存器的 QP 索引
        .i_qp_conf_replica_req  (i_qp_conf_replica_req),
        .o_qp_conf_replica_valid(o_qp_conf_replica_valid), // valid 由 req 延后一拍产生

        // QP advanced configuration QPi (QPADVCONFi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
        .i_qp_adv_conf_req  (i_qp_adv_conf_req),
        .o_qp_adv_conf_valid(o_qp_adv_conf_valid),  // valid 由 req 延后一拍产生
        .o_qp_adv_conf      (o_qp_adv_conf),        // QP advanced configuration; [5:0]: 流量类别dscp-（保持默认重置值）；[15:8]: 生存时间ttl；[31:16]: 分区密钥pkey
        .i_qp_adv_conf_idx  (i_qp_adv_conf_idx),    // 读取 QPADVCONFi 的 QP 索引

        // RCV Q Buffer base  address QPi (RQBAi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供 RDMA 接收队列缓冲区的基地址。
        .i_qp_rq_ba_req  (i_qp_rq_ba_req),
        .o_qp_rq_ba_valid(o_qp_rq_ba_valid),  // valid 由 req 延后一拍产生
        .o_qp_rq_ba      (o_qp_rq_ba),        // RQBAi[63:8]: RQ（接收队列）缓冲区基地址（256 B 对齐）。256 B 是总大小（单个 RQ 缓冲区元素大小）* rq_depth
        .i_qp_rq_ba_idx  (i_qp_rq_ba_idx),    // 读取 RQ 缓冲区基地址的 QP 索引

        // SQ（发送队列）基地址 QPi (SQBAi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供 RDMA 发送队列缓冲区的基地址。
        .i_qp_sq_ba_req_int  (qp_sq_ba_req_int),        // QP 管理器 cache 路径使用
        .o_qp_sq_ba_valid_int(qp_sq_ba_valid_int),      // QP 管理器 cache 路径使用
        .o_qp_sq_ba          (qp_sq_ba_int),            // SQBAi[63:5]: SQ 基地址（32 B 对齐）
        .i_qp_sq_ba_idx      (qp_sq_arbitrated_idx_ff), // 读取 SQ 基地址的 QP 索引

        // CQ base address QPi (CQBAi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（i = 1 至 C_NUM_QP）生成。该寄存器提供 RDMA 完成队列缓冲区的基地址。
        .i_qp_cq_ba_req  (i_qp_cq_ba_req),
        .o_qp_cq_ba_valid(o_qp_cq_ba_valid),
        .o_qp_cq_ba      (o_qp_cq_ba),        // CQBAi[63:5]: 发送 CQ 基地址（32 B 对齐）
        .i_qp_cq_ba_idx  (i_qp_cq_ba_idx),    // 读取 CQ 基地址的 QP 索引

        // RCV Q Write pointer DB address QPi (RQWPTRDBADDi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器提供接收队列门铃寄存器的地址。在接收到 MAD（QP1 SEND_ONLY）并消耗 RQ WQE 时，LRDMA IP 会更新寄存器所指向地址中的 RQ 门铃值。
        .i_qp_rq_wrptrdb_add_req  (i_qp_rq_wrptrdb_add_req),
        .o_qp_rq_wrptrdb_add_valid(o_qp_rq_wrptrdb_add_valid),
        .o_qp_rq_wrptrdb_add      (o_qp_rq_wrptrdb_add),        // RQWPTRDBADDi[63:0]: RCV Q 写指针门铃地址
        .i_qp_rq_wrptrdb_add_idx  (i_qp_rq_wrptrdb_add_idx),    // 读取 RQ 写门铃地址的 QP 索引

        // CQ head pointer QPi (CQHEADi) 寄存器
        // 这里做了处理：ACK 写 CQ head pointer 优先，retry 读等待；
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。该状态寄存器给出发送完成队列的门铃值，并提供已完成发送 WQE 的信息。这是 LRDMA IP 写入 CQDBADDi 寄存器所指向地址的门铃值。
        .i_qp_cq_hdptr_req        (i_qp_cq_hdptr_req),                                                                                           // 与 ACK 管理器 halt 请求进行 OR
        .i_qp_cq_hdptr_retry_req  (qp_cq_hdptr_retry_grant),                                                                                     // 与 ACK 管理器 halt 请求进行 OR
        .o_qp_cq_hdptr_valid      (o_qp_cq_hdptr_valid),
        .o_qp_cq_hdptr_valid_retry(qp_cq_hdptr_valid_retry),
        .o_qp_cq_hdptr            (qp_cq_hdptr_from_config),                                                                                     // CQHEADi[15:0]: CQ head pointer
        .i_qp_cq_hdptr            (i_qp_cq_hdptr),                                                                                               // CQHEADi[15:0]: CQ head pointer
        .i_qp_cq_hdptr_wrn        (i_qp_cq_hdptr_wrn),                                                                                           // web
        .i_qp_cq_hdptr_idx        (i_qp_cq_hdptr_wrn ? i_qp_cq_hdptr_idx : qp_cq_hdptr_retry_grant ? qp_cq_hdptr_idx_retry : i_qp_cq_hdptr_idx), // 读写 CQ head pointer 的 QP 索引

        // RQ Consumer Index QPi (RQCIi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。目标应用在消耗一个新的接收队列条目时更新该寄存器。一旦消耗完毕，LRDMA IP 就可以重新写入 RQ 条目。RQCI 索引也可通过侧带接口更新。
        .i_qp_rq_cidb_req  (i_qp_rq_cidb_req),
        .o_qp_rq_cidb_valid(o_qp_rq_cidb_valid),  // valid 由 req 延后一拍产生
        .o_qp_rq_cidb      (o_qp_rq_cidb),        // RQCIi[15:0]: RCV Q 消费者索引门铃
        .i_qp_rq_cidb_idx  (i_qp_rq_cidb_idx),    // 读取 RQ CI doorbell 的 QP 索引

        // SQ Producer index QPi (SQPIi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。目标应用在发布新的发送工作队列条目时更新该寄存器。SQPI 也可通过侧带接口更新。发送队列是一个循环缓冲区。相应 QP 的 CQHEAD 寄存器提供有关已完成的工作队列条目的信息，目标应用程序可对其进行重写。
        .o_qp_sq_pidb      (qp_sq_pidb),               // SQPIi[15:0]: SQ（发送队列）生产者索引门铃
        .i_qp_sq_pidb_idx  (qp_sq_arbitrated_idx_ff),  // 读取 SQ PI doorbell 的 QP 索引
        .i_qp_sq_pidb_req  (qp_sq_pidb_req),
        .o_qp_sq_pidb_valid(qp_sq_pidb_valid),         // valid 由 req 延后一拍产生

        // Q Depth QPi (QDEPTHi) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。该寄存器定义发送、完成和接收队列的队列深度。
        // RQ
        .i_qp_rq_depth_req  (i_qp_rq_depth_req),
        .o_qp_rq_depth_valid(o_qp_rq_depth_valid),      // valid 由 req 延后一拍产生
        .o_qp_rq_depth      (o_qp_rq_depth),            // QDEPTHi[31:16]: RQ 深度
        .i_qp_rq_depth_idx  (i_qp_rq_depth_idx),
        // SQ
        .i_qp_sq_depth_req  (qp_sq_depth_req),
        .o_qp_sq_depth_valid(qp_sq_depth_valid),        // valid 由 req 延后一拍产生
        .o_qp_sq_depth      (qp_sq_depth),              // QDEPTHi[15:0]: SQ 深度 (Send CQ will of the same depth)
        .i_qp_sq_depth_idx  (qp_sq_arbitrated_idx_ff),
        // CQ
        .i_qp_cq_depth_req  (i_qp_cq_depth_req),
        .o_qp_cq_depth_valid(o_qp_cq_depth_valid),      // valid 由 req 延后一拍产生
        .o_qp_cq_depth      (o_qp_cq_depth),            // QDEPTHi[15:0]: CQ 深度
        .i_qp_cq_depth_idx  (i_qp_cq_depth_idx),

        // SQ PSN for QPi (SQPSNi) 寄存器
        // 该寄存器在连接时由 SW 初始化。此后，硬件会为每个发出的数据包更新该寄存器，而软件则不应更新。该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。QP1 不存在该寄存器。
        .i_qp_sq_psn_req        (i_qp_sq_psn_req),                                      // RX 响应路径的 SQPSN 请求，优先级最高
        .i_qp_sq_psn_wqe_req    (i_qp_sq_psn_wqe_req),                                  // WQE 停止期间可与其它 SQPSN 请求仲裁
        .i_qp_sq_psn_retry_req  (qp_sq_psn_req_retry),                                  // 重传路径 SQPSN 读请求，WQE 停止期间可仲裁
        .o_qp_sq_psn_valid      (o_qp_sq_psn_valid),
        .o_qp_sq_psn_wqe_valid  (o_qp_sq_psn_wqe_valid),
        .o_qp_sq_psn_resp_valid (o_qp_sq_psn_resp_valid),
        .o_qp_sq_psn_retry_valid(qp_sq_psn_retry_valid),
        .o_qp_sq_psn            (o_qp_sq_psn),                                          // SQPSNi[23:0]: SQ（发送队列）PSN
        .i_qp_sq_psn            (qp_sq_psn_wrn_retry ? qp_sq_psn_retry : i_qp_sq_psn),  // SQPSNi[23:0]: SQ（发送队列）PSN
        .i_qp_sq_psn_wrn        (i_qp_sq_psn_wrn),                                      // SQPSNi 写使能，WQE 停止期间可仲裁
        .i_qp_sq_psn_retry_wrn  (qp_sq_psn_wrn_retry),                                  // 重传路径 SQPSNi 写使能
        .i_qp_sq_psn_idx        (i_qp_sq_psn_idx),                                      // RX 响应路径访问 SQPSNi 的 QP 索引
        .i_qp_sq_psn_wqe_idx    (i_qp_sq_psn_wqe_idx),                                  // TX WQE 路径访问 SQPSNi 的 QP 索引
        .i_qp_sq_psn_retry_idx  (qp_sq_psn_idx_retry),                                  // 重传路径访问 SQPSNi 的 QP 索引
        .i_qp_sq_psn_resp_idx   (i_qp_sq_psn_resp_idx),                                 // ACK 管理器响应完成检查访问 SQPSNi 的 QP 索引

        // Last RQ req for QPi (LSTREQi) 寄存器
        // 该寄存器提供最后一个接收到的 RQ 数据包的详细信息。该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。
        .o_qp_last_req      (o_qp_last_req),       // LSTREQi[31:24]: last req opcode; LSTREQi[23:0]: last req PSN
        .i_qp_last_req_idx  (i_qp_last_req_idx),
        .i_qp_last_req      (i_qp_last_req),       // LSTREQi[31:24]: last req opcode; LSTREQi[23:0]: last req PSN
        .i_qp_last_req_req  (i_qp_last_req_req),
        .i_qp_last_req_wrn  (i_qp_last_req_wrn),   // web
        .o_qp_last_req_valid(o_qp_last_req_valid), // valid 由 req 延后一拍产生

        // Destination QP configuration for QPi (DESTQPCONFi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。该寄存器由 SW 在连接时配置，提供与该 QP 连接的远程 QPID。从该 QP 发送的所有数据包都以该 QPID 作为目标 QPID。
        .i_qp_dest_qpid_req_tx_engine  (i_qp_dest_qpid_req_tx_engine),
        .i_qp_dest_qpid_req_rx_engine  (i_qp_dest_qpid_req_rx_engine),
        .o_qp_dest_qpid_valid_tx_engine(o_qp_dest_qpid_valid_tx_engine),
        .o_qp_dest_qpid_valid_rx_engine(o_qp_dest_qpid_valid_rx_engine),
        .o_qp_dest_qpid                (o_qp_dest_qpid),                  // DESTQPCONFi[23:0]: 连接到目的地的 QPID
        .i_qp_dest_qpid_idx            (i_qp_dest_qpid_idx),              // DESTQPCONFi 的 QP 索引

        .o_qp_disable_pulse_valid(o_qp_disable_pulse_valid),  // qp_conf_bram_wen == 1'b1时，o_qp_disable_pulse_valid <= 1'b1，保持两个周期
        .o_qp_disable_pulse_id   (o_qp_disable_pulse_id),     // qp idx
        .o_qp_disable_pulse_data (o_qp_disable_pulse_data),   // ~QPCONFi[0]: ~QP enable

        // Incoming Pkt Error Status Queue Base address (IPKTERRQBA) 寄存器
        // 该寄存器用于配置传入数据包错误状态队列的基地址。有关传入数据包错误状态队列的详情，请参阅 LRDMA RX Path。
        .o_in_errsts_q_ba   (o_in_errsts_q_ba),    // IPKTERRQBA[63:0]: 传入数据包错误状态队列基地址
        .o_in_errsts_q_sz   (o_in_errsts_q_sz),    // IPKTERRQSZ[15:0]: 收到的错误信息包状态队列条目数。
        .i_in_errsts_q_wrptr(i_in_errsts_q_wrptr), // 传入数据包错误状态队列写指针，指向下一个可写入位置

        // MAC destination address LSB QPi (MACDESADDLSBi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。该寄存器由 SW 在连接时配置，提供与该 QP 连接的远程主机的 MAC 地址。从该 QP 发送的所有数据包都以该 MAC 地址（LSB）作为目标 MAC 地址。
        .i_qp_mac_remote_addrl_req          (i_qp_mac_remote_addrl_req),
        .o_qp_mac_remote_addrl_valid        (o_qp_mac_remote_addrl_valid),
        .i_qp_mac_remote_addrl_wqe_req      (i_qp_mac_remote_addrl_wqe_req),
        .o_qp_mac_remote_addrl_wqe_valid    (o_qp_mac_remote_addrl_wqe_valid),
        .o_qp_mac_remote_addrl              (o_qp_mac_remote_addrl),               // MACDESADDLSBi[31:0]: 目标 MAC 地址 LSB
        .i_qp_mac_remote_addrl_idx          (i_qp_mac_remote_addrl_idx),           // MACDESADDLSBi 读取的 QP 索引
        .i_qp_mac_remote_addrl_wqe_idx      (i_qp_mac_remote_addrl_wqe_idx),       // TX WQE 路径读取 MACDESADDLSBi 的 QP 索引
        // MAC destination address LSB QPi (MACDESADDLSBi) 寄存器 复制寄存器
        .o_qp_mac_remote_addrl_replica      (o_qp_mac_remote_addrl_replica),
        .i_qp_mac_remote_addrl_replica_idx  (i_qp_mac_remote_addrl_replica_idx),   // MACDESADDLSBi replica 读取的 QP 索引
        .i_qp_mac_remote_addrl_replica_req  (i_qp_mac_remote_addrl_replica_req),
        .o_qp_mac_remote_addrl_replica_valid(o_qp_mac_remote_addrl_replica_valid),

        // MAC destination address MSB QPi (MACDESADDMSBi) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。该寄存器由 SW 在连接时配置，提供与该 QP 连接的远程主机的 MAC 地址。从该 QP 发送的所有数据包都以该 MAC 地址（MSB）作为目标 MAC 地址。
        .i_qp_mac_remote_addrm_req          (i_qp_mac_remote_addrm_req),
        .o_qp_mac_remote_addrm_valid        (o_qp_mac_remote_addrm_valid),
        .i_qp_mac_remote_addrm_wqe_req      (i_qp_mac_remote_addrm_wqe_req),
        .o_qp_mac_remote_addrm_wqe_valid    (o_qp_mac_remote_addrm_wqe_valid),
        .o_qp_mac_remote_addrm              (o_qp_mac_remote_addrm),               // MACDESADDMSBi[15:0]: 目标 MAC 地址 MSB
        .i_qp_mac_remote_addrm_idx          (i_qp_mac_remote_addrm_idx),           // MACDESADDMSBi 读取的 QP 索引
        .i_qp_mac_remote_addrm_wqe_idx      (i_qp_mac_remote_addrm_wqe_idx),       // TX WQE 路径读取 MACDESADDMSBi 的 QP 索引
        // MAC destination address MSB QPi (MACDESADDMSBi) 寄存器 复制寄存器
        .o_qp_mac_remote_addrm_replica      (o_qp_mac_remote_addrm_replica),
        .i_qp_mac_remote_addrm_replica_idx  (i_qp_mac_remote_addrm_replica_idx),   // MACDESADDMSBi replica 读取的 QP 索引
        .i_qp_mac_remote_addrm_replica_req  (i_qp_mac_remote_addrm_replica_req),
        .o_qp_mac_remote_addrm_replica_valid(o_qp_mac_remote_addrm_replica_valid),

        // IP destination address 1 QPi (IPDESADDR1i) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。如果该 QP 的远程主机使用 IPv4 协议配置，则该寄存器指示 IPv4 地址。如果远程主机使用 IPv6 协议配置，寄存器则指示 IPv6 目标地址。
        .i_qp_ip_remote_addr1_req          (i_qp_ip_remote_addr1_req),
        .o_qp_ip_remote_addr1_valid        (o_qp_ip_remote_addr1_valid),
        .i_qp_ip_remote_addr1_wqe_req      (i_qp_ip_remote_addr1_wqe_req),
        .o_qp_ip_remote_addr1_wqe_valid    (o_qp_ip_remote_addr1_wqe_valid),
        .o_qp_ip_remote_addr1              (o_qp_ip_remote_addr1),               // IPDESADDR1i[31:0]: 目标 IP 地址 LSB1
        .i_qp_ip_remote_addr1_idx          (i_qp_ip_remote_addr1_idx),           // IPDESADDR1i 读取的 QP 索引
        .i_qp_ip_remote_addr1_wqe_idx      (i_qp_ip_remote_addr1_wqe_idx),       // TX WQE 路径读取 IPDESADDR1i 的 QP 索引
        // IP destination address 1 QPi (IPDESADDR1i) 寄存器 复制寄存器
        .o_qp_ip_remote_addr1_replica      (o_qp_ip_remote_addr1_replica),
        .i_qp_ip_remote_addr1_replica_idx  (i_qp_ip_remote_addr1_replica_idx),   // IPDESADDR1i replica 读取的 QP 索引
        .i_qp_ip_remote_addr1_replica_req  (i_qp_ip_remote_addr1_replica_req),
        .o_qp_ip_remote_addr1_replica_valid(o_qp_ip_remote_addr1_replica_valid),

        // IP destination address 2 QPi (IPDESADDR2i) 寄存器
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。IPv6 配置时指示目标地址 [63:32]；IPv4 配置下无地址字段含义。
        .i_qp_ip_remote_addr2_req          (i_qp_ip_remote_addr2_req),
        .o_qp_ip_remote_addr2_valid        (o_qp_ip_remote_addr2_valid),
        .i_qp_ip_remote_addr2_wqe_req      (i_qp_ip_remote_addr2_wqe_req),
        .o_qp_ip_remote_addr2_wqe_valid    (o_qp_ip_remote_addr2_wqe_valid),
        .o_qp_ip_remote_addr2              (o_qp_ip_remote_addr2),               // IPDESADDR2i[31:0]: 目标 IP 地址 LSB2
        .i_qp_ip_remote_addr2_idx          (i_qp_ip_remote_addr2_idx),           // IPDESADDR2i 读取的 QP 索引
        .i_qp_ip_remote_addr2_wqe_idx      (i_qp_ip_remote_addr2_wqe_idx),       // TX WQE 路径读取 IPDESADDR2i 的 QP 索引
        // IP destination address 2 QPi (IPDESADDR2i) 寄存器 复制寄存器
        .o_qp_ip_remote_addr2_replica      (o_qp_ip_remote_addr2_replica),
        .i_qp_ip_remote_addr2_replica_idx  (i_qp_ip_remote_addr2_replica_idx),   // IPDESADDR2i replica 读取的 QP 索引
        .i_qp_ip_remote_addr2_replica_req  (i_qp_ip_remote_addr2_replica_req),
        .o_qp_ip_remote_addr2_replica_valid(o_qp_ip_remote_addr2_replica_valid),

        // IP destination address 3 QPi (IPDESADDR3i) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。IPv6 配置时指示目标地址 [95:64]；IPv4 配置下无地址字段含义。
        .i_qp_ip_remote_addr3_req          (i_qp_ip_remote_addr3_req),
        .o_qp_ip_remote_addr3_valid        (o_qp_ip_remote_addr3_valid),
        .i_qp_ip_remote_addr3_wqe_req      (i_qp_ip_remote_addr3_wqe_req),
        .o_qp_ip_remote_addr3_wqe_valid    (o_qp_ip_remote_addr3_wqe_valid),
        .o_qp_ip_remote_addr3              (o_qp_ip_remote_addr3),               // IPDESADDR3i[31:0]: 目标 IP 地址 MSB1
        .i_qp_ip_remote_addr3_idx          (i_qp_ip_remote_addr3_idx),           // IPDESADDR3i 读取的 QP 索引
        .i_qp_ip_remote_addr3_wqe_idx      (i_qp_ip_remote_addr3_wqe_idx),       // TX WQE 路径读取 IPDESADDR3i 的 QP 索引
        // IP destination address 3 QPi (IPDESADDR3i) 寄存器 复制寄存器
        .o_qp_ip_remote_addr3_replica      (o_qp_ip_remote_addr3_replica),
        .i_qp_ip_remote_addr3_replica_idx  (i_qp_ip_remote_addr3_replica_idx),   // IPDESADDR3i replica 读取的 QP 索引
        .i_qp_ip_remote_addr3_replica_req  (i_qp_ip_remote_addr3_replica_req),
        .o_qp_ip_remote_addr3_replica_valid(o_qp_ip_remote_addr3_replica_valid),

        // IP destination address 4 QPi (IPDESADDR4i) 寄存器
        // 该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。IPv6 配置时指示目标地址 [127:96]；IPv4 配置下无地址字段含义。
        .i_qp_ip_remote_addr4_req          (i_qp_ip_remote_addr4_req),
        .o_qp_ip_remote_addr4_valid        (o_qp_ip_remote_addr4_valid),
        .i_qp_ip_remote_addr4_wqe_req      (i_qp_ip_remote_addr4_wqe_req),
        .o_qp_ip_remote_addr4_wqe_valid    (o_qp_ip_remote_addr4_wqe_valid),
        .o_qp_ip_remote_addr4              (o_qp_ip_remote_addr4),               // IPDESADDR4i[31:0]: 目标 IP 地址 MSB2
        .i_qp_ip_remote_addr4_idx          (i_qp_ip_remote_addr4_idx),           // IPDESADDR4i 读取的 QP 索引
        .i_qp_ip_remote_addr4_wqe_idx      (i_qp_ip_remote_addr4_wqe_idx),       // TX WQE 路径读取 IPDESADDR4i 的 QP 索引
        // IP destination address 4 QPi (IPDESADDR4i) 寄存器 复制寄存器
        .o_qp_ip_remote_addr4_replica      (o_qp_ip_remote_addr4_replica),
        .i_qp_ip_remote_addr4_replica_idx  (i_qp_ip_remote_addr4_replica_idx),   // IPDESADDR4i replica 读取的 QP 索引
        .i_qp_ip_remote_addr4_replica_req  (i_qp_ip_remote_addr4_replica_req),
        .o_qp_ip_remote_addr4_replica_valid(o_qp_ip_remote_addr4_replica_valid),

        // Status Current SQ ptr under process (STATCURSQPTRi) 寄存器 (Read only)
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。
        .i_qp_curr_sqptr_proc            (qp_curr_sqptr_proc_wen_retry ? qp_curr_sqptr_proc_retry : new_curr_sqptr_proc),
        .i_qp_curr_sqptr_rdreq           (qp_curr_sqptr_req),
        .i_qp_curr_sqptr_proc_wen        (qp_curr_sqptr_proc_wen_retry | update_curr_sqptr),                                       // 由于 QP 缓存已停止，因此可进行 OR 运算
        .o_prog_full_db_update_curr_sqptr(prog_full_db_update_curr_sqptr),                                                         // DB 更新队列接近满状态
        .i_qp_curr_sqptr_proc_idx        (qp_curr_sqptr_proc_wen_retry ? qp_curr_sqptr_proc_idx_retry : qp_sq_arbitrated_idx_ff),
        .o_qp_curr_sqptr_proc_valid      (qp_curr_sqptr_proc_valid),
        .o_qp_curr_sqptr_proc            (qp_curr_sqptr_proc),                                                                     // STATCURSQPTRi[15:0]: SQ 消费指针，指向等待远端响应的 WQE 队列位置。

        // Status Response PSN for QPi (STATRESPSNi)
        .i_qp_stat_resp_psn      (i_qp_stat_resp_psn),
        .i_qp_stat_resp_psn_wen  (i_qp_stat_resp_psn_wen),
        .i_qp_stat_resp_psn_idx  (i_qp_stat_resp_psn_idx),
        .o_qp_stat_resp_psn_valid(o_qp_stat_resp_psn_valid),
        .i_qp_stat_resp_psn_rdreq(i_qp_stat_resp_psn_rdreq),
        .o_qp_stat_resp_psn      (o_qp_stat_resp_psn),        // STATRESPSNi[23:0]: 预期响应的 PSN

        // QPi 的 RQ 缓冲区写入地址状态寄存器（STATRQBUFCAi，只读）
        // 该寄存器根据参数 C_NUM_QP 生成（其中 i = 2 至 C_NUM_QP）。
        .i_qp_stat_rq_buf_ca      (i_qp_stat_rq_buf_ca),        // STATRQBUFCAi[31:8]: RQ（接收队列）缓冲区写入地址（256 B 对齐）
        .i_qp_stat_rq_buf_ca_wen  (i_qp_stat_rq_buf_ca_wen),
        .i_qp_stat_rq_buf_ca_idx  (i_qp_stat_rq_buf_ca_idx),
        .o_qp_stat_rq_buf_ca_valid(o_qp_stat_rq_buf_ca_valid),
        .i_qp_stat_rq_buf_ca_rdreq(i_qp_stat_rq_buf_ca_rdreq),
        .o_qp_stat_rq_buf_ca      (o_qp_stat_rq_buf_ca),        // STATRQBUFCAi[31:8]: RQ（接收队列）缓冲区写入地址（256 B 对齐）

        // Status QPi (STATQPi) 寄存器
        // 该寄存器提供每个 QP 的 QP 状态。该寄存器根据参数 C_NUM_QP（其中 i = 1 至 C_NUM_QP）生成。
        // STATQPi[30:28]: 无功能定义
        // STATQPi[22:16]: 收到 NACK 综合征（读/写）
        .i_qp_stat_nak      (i_qp_stat_nak),        // STATQPi[22:16]
        .i_qp_stat_nak_wen  (i_qp_stat_nak_wen),
        .i_qp_stat_nak_idx  (i_qp_stat_nak_idx),
        .o_qp_stat_nak_valid(o_qp_stat_nak_valid),
        .i_qp_stat_nak_rdreq(i_qp_stat_nak_rdreq),
        .o_qp_stat_nak      (o_qp_stat_nak),        // STATQPi[22:16]

        .i_qp_stat_retry_cnt      (i_qp_stat_retry_cnt),        // STATQPi[26:24]: 重试次数（读/写）
        .i_qp_stat_retry_cnt_wen  (i_qp_stat_retry_cnt_wen),
        .i_qp_stat_retry_cnt_idx  (i_qp_stat_retry_cnt_idx),
        .o_qp_stat_retry_cnt_valid(o_qp_stat_retry_cnt_valid),
        .i_qp_stat_retry_cnt_rdreq(i_qp_stat_retry_cnt_rdreq),
        .o_qp_stat_retry_cnt      (o_qp_stat_retry_cnt),        // STATQPi[26:24]: 重试次数（读/写）

        .init_start(init_start),
        .init_count(init_count),

        // 来自 tx engine 的状态
        .i_tx_engine_sts(i_tx_engine_sts),
        .i_qp_mgr_sts   ({stat_wqe_cnt, 8'b0, 1'b0, cache_fsm_status, 2'b0, wqe_empty, wqe_full}), // 赋值给 QPMSTS 寄存器，cache_fsm_status 没用到

        // 状态观测寄存器
        .i_axi_rd_err_cnt   (axi_rd_err_cnt_ff),
        .i_last_in_pkt_info (i_last_in_pkt_info),
        .i_last_out_pkt_info(i_last_out_pkt_info)
    );

    assign o_qp_cq_hdptr = qp_cq_hdptr_from_config;

    reg  [C_NUM_QP-1 : 0] sq_empty;  // 每个QP的SQ是否为空，sq_empty[0]对应QP0的SQ是否为空
    reg  [C_NUM_QP-1 : 0] osq_full;
    wire [  C_NUM_QP-1:0] sw_override_per_qp_en;

    always @(posedge core_clk) begin
        if (!core_rstn) begin
            sq_empty <= {C_NUM_QP{1'b1}};
            osq_full <= {C_NUM_QP{1'b0}};
        end else begin
            // SQ 空状态
            if (sq_empty_valid) begin
                sq_empty[sq_empty_id] <= sq_empty_data;
            end

            // OSQ 满状态
            if (osq_full_set_valid) begin
                osq_full[osq_full_set_id] <= 1'b1;
            end
            if (osq_full_reset_valid) begin
                osq_full[osq_full_reset_id] <= 1'b0;
            end
        end
    end

    // QP RR轮询仲裁器：选取某个有用QP进行处理
    // 在多个有 WQE 待发的 QP 间公平调度
    lrdma_qp_rr_arbiter #(
        .C_NUM_ELMNTS_TO_ARB(C_NUM_QP),        // 需要仲裁元素的最大数量
        .C_ELMNT_PTR_WIDTH  (C_QP_INDX_WIDTH)  // SSD编号指针宽度
    ) inst_qp_rr_arbiter (
        // Clock and Reset
        .core_clk(core_clk),   // Module Clock  
        .core_rst(~core_rstn), // Active high core reset

        // Configurations driven from register interface    
        // Intreface to sq_arbiter
        .i_arbitrate          (arbitrate),                                   // 仲裁请求
        .i_enable             (1'b1),                                        // 启用该模块是为了进行仲裁
        .o_arbitration_done   (arbitration_done),                            // 仲裁完成，相当于 arbitration_done <= arbitrate
        .o_elmnt_index_one_hot(),                                            // 获胜元素的一个热门索引
        .o_elmnt_index        (qp_sq_arbitrated_idx),                        // i_elmnt_unavail 输入后，3周期后输出结果 o_elmnt_index。tips: 需要 SQPIDB 的索引
        .i_elmnt_unavail      (sq_empty | osq_full | sw_override_per_qp_en)  // qp不可用向量
    );

    //----------------------------------------------------------------------------
    // cache module 
    lrdma_qp_wqe_cache #(
        .C_NUM_QP                   (C_NUM_QP),
        .C_M_AXI_ADDR_WIDTH         (C_M_AXI_ADDR_WIDTH),
        .WQE_FIFO_IDX_WIDTH         (WQE_FIFO_IDX_WIDTH),
        .WQE_FIFO_DEPTH             (WQE_FIFO_DEPTH),
        .C_QP_INDX_WIDTH            (C_QP_INDX_WIDTH),
        .C_ENABLE_SAME_SQ_FAST_FETCH(C_ENABLE_SAME_SQ_FAST_FETCH)
    ) inst_qp_wqe_cache (
        .core_clk (core_clk),
        .core_rstn(core_rstn),

        .o_fsm_status      (cache_fsm_status),
        .o_stat_num_wqe_cnt(stat_wqe_cnt),

        // Configuration details
        .i_sq_empty_and_reduce (sq_empty_and_reduce),      // 所有 QP 的 SQ 是否都为空？
        .i_osq_full_set_valid  (osq_full_set_valid),
        .i_osq_full_set_id     (osq_full_set_id),
        .i_osq_full_reset_valid(osq_full_reset_valid),
        .i_osq_full_reset_id   (osq_full_reset_id),
        .i_qp_sq_arbitrated_idx(qp_sq_arbitrated_idx_ff),
        .i_wqe_fifo_full       (wqe_full),
        .i_num_valid_entries   (num_valid_entries),
        .i_arbitration_done    (arbitration_done),
        .o_arbitrate           (arbitrate),
        .o_sample_arbitrated_sq(sample_arbitrated_sq),

        // SQ 基地址配置信息
        .o_qp_sq_ba_req  (qp_sq_ba_req_int),
        .i_qp_sq_ba_valid(qp_sq_ba_valid_int),
        .i_qp_sq_ba      (qp_sq_ba_int),

        // SQ 深度配置信息
        .o_qp_sq_depth_req  (qp_sq_depth_req),
        .i_qp_sq_depth_valid(qp_sq_depth_valid),
        .i_qp_sq_depth      (qp_sq_depth),

        // SQ PIDB 配置信息
        .o_qp_sq_pidb_req  (qp_sq_pidb_req),
        .i_qp_sq_pidb_valid(qp_sq_pidb_valid),
        .i_qp_sq_pidb      (qp_sq_pidb),

        // SQ 消费指针信息，指向等待远端响应的 WQE 队列位置。
        .o_qp_curr_sqptr_rdreq           (qp_curr_sqptr_req),
        .i_qp_curr_sqptr_valid           (qp_curr_sqptr_proc_valid),
        .i_qp_curr_sqptr_proc            (qp_curr_sqptr_proc),
        .o_qp_curr_sqptr                 (new_curr_sqptr_proc),
        .o_update_curr_sqptr             (update_curr_sqptr),
        .i_prog_full_db_update_curr_sqptr(prog_full_db_update_curr_sqptr),

        .i_sw_override_en(sw_override_en),
        .i_lrdma_en      (o_lrdma_en),

        .i_halt                  (halt),
        .o_halted                (cache_halted),
        .i_qp_stat_wqe_cnt_valid (qp_stat_wqe_cnt_valid),
        .o_qp_stat_wqe_cnt_rdreq (qp_stat_wqe_cnt_rdreq),
        .i_qp_stat_wqe_cnt       (qp_stat_wqe_cnt),
        .o_update_qp_stat_wqe_cnt(update_qp_stat_wqe_cnt),
        .o_qp_stat_wqe_cnt       (new_qp_stat_wqe_cnt),

        .o_axi_ren_valid   (axi_ren_valid),
        .o_wqe_qpid        (wqe_qpid_from_qp_cache),
        .o_axi_read_addr   (sq_read_addr_to_axi),
        .i_wqe_addr_latched(original_wqe_arready),
        .i_wqe_dataread    (original_wqe_dvalid_r2),
        .init_start        (init_start),
        .init_count        (init_count)

    );

    assign o_halt = halt;
    `MSFF_R(axi_ren_valid_ff, axi_ren_valid, core_clk, ~core_rstn)
    assign wqe_axi_m_en = (C_ENABLE_FAST_WQE_AXI_EN != 0) ? axi_ren_valid : axi_ren_valid_ff;

    //------------------------------------------------------------------
    // SQ WQE FIFO

    // SQ WQE FIFO 控制逻辑
    lrdma_pointer_queue_mgr #(
        .C_Q_DEPTH   (WQE_FIFO_DEPTH),     // 每个队列中的条目数
        .C_ENTRY_SIZE(WQE_SIZE),           // 每个条目的大小（字节）。SQ 大小为 64B
        .C_PTR_WIDTH (WQE_FIFO_IDX_WIDTH)  // 访问 Q 深度所需的指针大小
    ) inst_pointer_queue_mgr (
        // Clock and Reset
        .core_clk(core_clk),   // Clock  
        .core_rst(~core_rstn), // Active high core reset

        // Queue pointers
        .o_rd_addr          (),                   // 读地址堆叠在一条总线上
        .o_wr_addr          (),                   // 写地址堆叠在一条总线上
        .o_rd_ptr           (wqe_fifo_rd_ptr),    // FIFO 读指针，弹出+1
        .o_wr_ptr           (wqe_fifo_wr_ptr),    // FIFO 写指针，推入+1
        .o_num_valid_entries(num_valid_entries),  // FIFO 有效条目数，给 cache 做“是否还能继续从 DDR 取 WQE”的门控
        .o_num_free_entries (),                   // Number of free entries stacked

        // 空满状态。o_wqe_empty 还额外延长 1 拍，防止 BRAM 读延迟导致误判
        .o_q_empty      (wqe_empty),
        .o_q_full       (wqe_full),
        .o_q_almost_full(),           // Q is full -1

        // 配置信号
        .i_base_addr(32'b0),           // Base address of queues
        .i_q_depth  (WQE_FIFO_DEPTH),
        .i_init     (1'b0),

        // 推入操作
        // 有新WQE从DDR过来，或者重传模块回推WQE时，入队
        .i_q_push          (original_wqe_dvalid_r2 | wqe_fifo_push_retry),  // Push enable for a queue
        .i_push_num_entries({{(WQE_FIFO_IDX_WIDTH) {1'b0}}, 1'b1}),         // Number of entries to be pushed

        // 弹出操作
        // TX 引擎消费一条 WQE，或重传模块清理/重排时弹出
        .i_q_pop          (i_wqe_pop | wqe_fifo_pop_retry),        // Pop enable for a queue
        .i_pop_num_entries({{(WQE_FIFO_IDX_WIDTH) {1'b0}}, 1'b1})  // Number of entries to be popped
    );

    // 正常 WQE 可以走 bypass 并进入 TX/OSQ，之后若收到 NAK 或 timeout，仍然可以触发重传。
    // 但重传回放 WQE 本身必须排除在普通 bypass 之外，否则会绕过 retry reorder/BRAM
    // 上下文保护，导致 PSN、payload address 或 OSQ 窗口与重传上下文错位。
    assign wqe_bypass_retry_ctx = wqe_fifo_push_retry | wqe_fifo_pop_retry | wqe_qid_inserted[WQE_RETRANSMIT_BIT] | wqe_qid_inserted[WQE_RETRIED_BIT];
    assign wqe_bypass_fire      = (C_ENABLE_WQE_BYPASS != 0) & original_wqe_dvalid_r2 & wqe_empty & wqe_empty_ff & ~halt & ~wqe_bypass_valid & ~wqe_bypass_retry_ctx;
    assign wqe_bypass_active    = wqe_bypass_fire | wqe_bypass_valid;

    assign o_wqe_empty          = wqe_bypass_active ? 1'b0 : (wqe_empty | wqe_empty_ff);  // extending the WQE empty assertion for one more cycle until BRAm data is stable
    assign o_wqe_fast_valid     = wqe_bypass_valid;
    `MSFF_R(wqe_empty_ff, wqe_empty, core_clk, ~core_rstn)

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            wqe_bypass_data     <= 512'd0;
            wqe_bypass_valid    <= 1'b0;
            wqe_bypass_pop_seen <= 1'b0;
        end else begin
            if (wqe_bypass_valid && halt && !i_wqe_pop) begin
                wqe_bypass_valid    <= 1'b0;
                wqe_bypass_pop_seen <= 1'b0;
            end else if (wqe_bypass_fire) begin
                wqe_bypass_data     <= wqe_qid_inserted;
                wqe_bypass_valid    <= 1'b1;
                wqe_bypass_pop_seen <= 1'b0;
            end else if (wqe_bypass_valid && wqe_bypass_pop_seen) begin
                wqe_bypass_valid    <= 1'b0;
                wqe_bypass_pop_seen <= 1'b0;
            end else if (wqe_bypass_valid && i_wqe_pop) begin
                wqe_bypass_pop_seen <= 1'b1;
            end
        end
    end

    // SQ WQE FIFO 存储逻辑
    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (WQE_SIZE * WQE_FIFO_DEPTH),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                     //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),             //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                     //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                         //string;
        .USE_MEM_INIT     (1),                          //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),            //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                          //integer; 0,1
        .ECC_MODE         ("no_ecc"),                   //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                          // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(WQE_SIZE),            //positive integer
        .READ_DATA_WIDTH_A (WQE_SIZE),            //positive integer
        .BYTE_WRITE_WIDTH_A(WQE_SIZE),            //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (WQE_FIFO_IDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),                 //string
        .READ_LATENCY_A    (1),                   //non-negative integer
        .WRITE_MODE_A      ("write_first"),       //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(WQE_SIZE),            //positive integer
        .READ_DATA_WIDTH_B (WQE_SIZE),            //positive integer
        .BYTE_WRITE_WIDTH_B(WQE_SIZE),            //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (WQE_FIFO_IDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                 //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                   //non-negative integer
        .WRITE_MODE_B      ("write_first")        //string; "write_first", "read_first", "no_change" 

    ) inst_wqe_fifo (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (wqe_fifo_push_retry | original_wqe_dvalid_r2),
        .addra         (wqe_fifo_wr_ptr),                                       // 从0开始
        .dina          (wqe_fifo_push_retry ? wqe_to_fifo : wqe_qid_inserted),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (~wqe_empty),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (wqe_fifo_rd_ptr),   // 从0开始
        .dinb          ({WQE_SIZE{1'b0}}),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (cached_wqe),
        .sbiterrb      (),
        .dbiterrb      ()

    );
    // SQ WQE FIFO end
    //------------------------------------------------------------------


    assign o_wqe = wqe_bypass_active ? (wqe_bypass_valid ? wqe_bypass_data : wqe_qid_inserted) : cached_wqe;

    `MSFF_R(qp_sq_arbitrated_idx_ff, ((arbitration_done & sample_arbitrated_sq) ? qp_sq_arbitrated_idx : qp_sq_arbitrated_idx_ff), core_clk, ~core_rstn)



    //----------------------------------------------------------------------------
    // 重传模块

    lrdma_qp_retransmit_ctrl #(
        .C_NUM_QP            (C_NUM_QP),
        .C_WQE_FIFO_IDX_WIDTH(WQE_FIFO_IDX_WIDTH),
        .C_QP_INDX_WIDTH     (C_QP_INDX_WIDTH)
    ) inst_qp_retransmit_ctrl (
        .core_clk (core_clk),
        .core_rstn(core_rstn),

        // Configuration details
        .i_retransmit_reqd        (i_retransmit_reqd),
        .i_retransmit_qpid        (i_retransmit_qpid),          // 重传路径选中的 QP id
        .i_retransmit_wqe_bram_din(i_retransmit_wqe_bram_din),  // {retry_start_psn[23:0], completed_dma_len[31:0]}
        .o_retransmit_accepted    (o_retransmit_accepted),
        .i_psn_to_retry           (i_psn_to_retry),
        .i_ssn_to_retry           (i_ssn_to_retry),

        .i_num_valid_entries(num_valid_entries),

        .o_qp_cq_hdptr_req  (qp_cq_hdptr_req_retry),
        .i_qp_cq_hdptr_valid(qp_cq_hdptr_valid_retry),
        .i_qp_cq_hdptr      (qp_cq_hdptr_from_config),
        .o_qp_cq_hdptr_idx  (qp_cq_hdptr_idx_retry),

        // configuration that is updated
        .o_qp_curr_sqptr_proc    (qp_curr_sqptr_proc_retry),
        .o_qp_curr_sqptr_proc_wen(qp_curr_sqptr_proc_wen_retry),
        .o_qp_curr_sqptr_proc_idx(qp_curr_sqptr_proc_idx_retry),  // 重传路径写回 SQ pointer 的 QP 索引

        .o_qp_stat_ret_sq_psn      (qp_stat_ret_sq_psn_int),
        .o_qp_stat_ret_sq_psn_wen  (qp_stat_ret_sq_psn_wen_int),
        .o_qp_stat_ret_sq_psn_idx  (qp_stat_ret_sq_psn_idx_int),
        .i_qp_stat_ret_sq_psn_valid(qp_stat_ret_sq_psn_valid_int),

        .o_qp_sq_psn_req  (qp_sq_psn_req_retry),
        .i_qp_sq_psn_valid(qp_sq_psn_retry_valid),
        .i_qp_sq_psn      (o_qp_sq_psn),
        .o_qp_sq_psn_wrn  (qp_sq_psn_wrn_retry),
        .o_qp_sq_psn      (qp_sq_psn_retry),
        .o_qp_sq_psn_idx  (qp_sq_psn_idx_retry),    // 重传路径访问 SQPSNi 的 QP 索引

        .o_qp_stat_ssn_idx               (qp_stat_ssn_idx_retry),           // 重传路径访问 STATSSNi 的 QP 索引
        .o_qp_stat_ssn                   (qp_stat_ssn_retry),               // 重传路径写回的发送侧 SSN
        .i_qp_stat_ssn                   (o_qp_stat_ssn),                   // 发送侧 SSN 读返回
        .o_qp_stat_ssn_req               (qp_stat_ssn_req_retry),
        .i_prog_full_db_update_curr_sqptr(prog_full_db_update_curr_sqptr),
        .o_qp_stat_ssn_wrn               (qp_stat_ssn_wrn_retry),
        .i_qp_stat_ssn_valid             (o_qp_stat_ssn_valid),

        // Interface to SQ WQE FIFO
        .i_wqe          (wqe_from_fifo),
        .o_wqe          (wqe_to_fifo),
        .o_wqe_fifo_push(wqe_fifo_push_retry),
        .o_wqe_fifo_pop (wqe_fifo_pop_retry),

        .o_halt       (halt),
        .o_halted_qpid(o_halted_qpid),
        .i_halted     (i_wqe_halted & cache_halted),

        .i_osq_pop         (i_osq_pop),
        .i_osq_pop_qpid    (i_osq_pop_qpid),
        .o_wqe_dropped     (retransmit_wqe_dropped),
        .o_wqe_dropped_qpid(retransmit_wqe_dropped_qpid),

        .o_retransmit_wqe_bram_din (retransmit_wqe_bram_din_a),
        .i_retransmit_wqe_bram_dout(retransmit_wqe_bram_dout_a),
        .o_retransmit_wqe_bram_wen (retransmit_wqe_bram_wen_a),
        .o_retransmit_wqe_bram_idx (retransmit_wqe_bram_idx_a)
    );

    ///////////////////////////////////////////////////////////////////////////
    //////////////// Retransmit WQE BRAM instance /////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    assign qp_disable_retrans_clear    = o_qp_disable_pulse_valid & o_qp_disable_pulse_data;
    assign retransmit_wqe_bram_wen_mux = qp_disable_retrans_clear | retransmit_wqe_bram_wen_a;
    assign retransmit_wqe_bram_idx_mux = qp_disable_retrans_clear ? o_qp_disable_pulse_id : retransmit_wqe_bram_idx_a;
    assign retransmit_wqe_bram_din_mux = qp_disable_retrans_clear ? 57'd0 : retransmit_wqe_bram_din_a;

    xpm_memory_tdpram #(

        // Common module parameters
        .MEMORY_SIZE      (57 * C_NUM_QP),    //positive integer
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
        .WRITE_DATA_WIDTH_A(57),               //positive integer
        .READ_DATA_WIDTH_A (57),               //positive integer
        .BYTE_WRITE_WIDTH_A(57),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_A("0"),              //string
        .READ_LATENCY_A    (1),                //non-negative integer
        .WRITE_MODE_A      ("write_first"),    //string; "write_first", "read_first", "no_change" 

        // Port B module parameters
        .WRITE_DATA_WIDTH_B(57),               //positive integer
        .READ_DATA_WIDTH_B (57),               //positive integer
        .BYTE_WRITE_WIDTH_B(57),               //integer; 8, 9, or WRITE_DATA_WIDTH_B value
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //vector of READ_DATA_WIDTH_B bits
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("write_first")     //string; "write_first", "read_first", "no_change" 

    ) inst_retransmit_wqe_bram (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .rsta          (~core_rstn),
        .ena           (1'b1),
        .regcea        (1'b1),
        .wea           (retransmit_wqe_bram_wen_mux),
        .addra         (retransmit_wqe_bram_idx_mux),
        .dina          (retransmit_wqe_bram_din_mux),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .douta         (retransmit_wqe_bram_dout_a),
        .sbiterra      (),
        .dbiterra      (),

        // Port B module ports
        .clkb          (core_clk),
        .rstb          (~core_rstn),
        .enb           (original_wqe_dvalid),
        .regceb        (1'b1),
        .web           (1'b0),
        .addrb         (original_wqe_qid),
        .dinb          ({57{1'b0}}),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .doutb         (retransmit_wqe_bram_dout_b),
        .sbiterrb      (),
        .dbiterrb      ()

    );

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (1 * C_NUM_QP),     //positive integer
        .MEMORY_PRIMITIVE ("distributed"),    //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),   //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),               //string;
        .USE_MEM_INIT     (1),                //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                //integer; 0,1

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(1),               //positive integer
        .BYTE_WRITE_WIDTH_A(1),               //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_QP_INDX_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (1),                //positive integer
        .ADDR_WIDTH_B      (C_QP_INDX_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),              //string
        .READ_LATENCY_B    (1),                //non-negative integer
        .WRITE_MODE_B      ("read_first")      //string; "write_first", "read_first", "no_change" 

    ) inst_retrans_check_done (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (1'b1),
        .wea           (original_wqe_dvalid_r1 | qp_disable_retrans_clear),
        .addra         (qp_disable_retrans_clear ? o_qp_disable_pulse_id : original_wqe_qid_r1),
        .dina          (qp_disable_retrans_clear ? 1'b0 : retransmit_wqe_bram_dout_b[56]),
        .injectsbiterra(1'b0),                                                                    // A 端不注入单比特错误
        .injectdbiterra(1'b0),                                                                    // A 端不注入双比特错误

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (~core_rstn),
        .enb     (1'b1),
        .regceb  (1'b1),
        .addrb   (original_wqe_qid),
        .doutb   (retrans_check_done),
        .sbiterrb(),                    // B 端单比特错误状态未接出
        .dbiterrb()                     // B 端双比特错误状态未接出

    );

    assign retransmit_local_addr  = original_wqe_data_r1[95:32] + retransmit_wqe_bram_dout_b[31:0];
    assign retransmit_remote_addr = original_wqe_data_r1[223:160] + retransmit_wqe_bram_dout_b[31:0];
    assign retransmit_dma_len     = original_wqe_data_r1[127:96] - retransmit_wqe_bram_dout_b[31:0];

    always @(posedge core_clk) begin
        if (~core_rstn) begin
            original_wqe_dvalid_r1 <= 1'b0;
            original_wqe_dvalid_r2 <= 1'b0;
            original_wqe_data_r1   <= 512'd00000000;
            original_wqe_qid_r1    <= 'd0;
            wqe_qid_inserted       <= 512'd00000000;
        end else begin
            original_wqe_dvalid_r1 <= original_wqe_dvalid;
            original_wqe_dvalid_r2 <= original_wqe_dvalid_r1;
            original_wqe_data_r1   <= original_wqe_data;
            original_wqe_qid_r1    <= original_wqe_qid;

            // 重传分支
            if ((retransmit_wqe_bram_dout_b[56] != retrans_check_done) && original_wqe_dvalid_r1 && ~(original_wqe_dvalid && original_wqe_qid == original_wqe_qid_r1)) begin
                retrans_check_update <= 1'b1;
                // [511:440]                     ← original_wqe_data_r1[511:440]
                // [439:416]                     ← retransmit_wqe_bram_dout_b[55:32]
                // [415:224]                     ← original_wqe_data_r1[415:224]
                // [223:160]                     ← retransmit_remote_addr
                // [159:152]                     ← 8'h01，置位 TX 侧 retransmit_wqe 标志，表示部分重传不再携带 RETH
                // [151:147]                     ← 5'h00
                // [146:(136 + C_QP_INDX_WIDTH)] ← {(11 - C_QP_INDX_WIDTH){1'b0}}
                // [(135 + C_QP_INDX_WIDTH):136] ← original_wqe_qid_r1
                // [135:128]                     ← original_wqe_data_r1[135:128]
                // [127:96]                      ← retransmit_dma_len
                // [95:32]                       ← retransmit_local_addr
                // [31:0]                        ← original_wqe_data_r1[31:0]
                wqe_qid_inserted <= {
                    original_wqe_data_r1[511 : 440],
                    retransmit_wqe_bram_dout_b[55:32],
                    original_wqe_data_r1[415:224],
                    retransmit_remote_addr,
                    8'h01,
                    5'h00,
                    {(11 - C_QP_INDX_WIDTH) {1'b0}},
                    original_wqe_qid_r1,
                    original_wqe_data_r1[135:128],
                    retransmit_dma_len,
                    retransmit_local_addr,
                    original_wqe_data_r1[31:0]
                };
            end//
            
            // 普通分支，从DDR中取出SQ WQE执行的分支
            else begin
                // 在手册 WQE 格式的[146:136]插入了 original_wqe_qid_r1，其余保持不变
                wqe_qid_inserted     <= {original_wqe_data_r1[511 : 153], 1'b0, 5'h00, {(11 - C_QP_INDX_WIDTH) {1'b0}}, original_wqe_qid_r1, original_wqe_data_r1[135:0]};
                retrans_check_update <= 1'b0;
            end
        end
    end
    assign wqe_from_fifo = cached_wqe;

    //----------------------------------------------------------------------------
    // 从 DDR 中的 SQ 中抓取 WQE
    lrdma_qp_mgr_axi_master #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_DATA_WIDTH(512),
        .C_M_AXI_THREAD_ID_WIDTH(1),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .IP2BUS_LEN_WIDTH(14)
    ) inst_qp_mgr_axi_master (
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
        .i_qp_sq_arbitrated_idx(wqe_qpid_from_qp_cache),  // input，仲裁后的 SQ QPID
        .original_wqe_byte_en  (),
        .original_wqe_data     (original_wqe_data),
        .o_qp_sq_idx           (original_wqe_qid),        // i_qp_sq_arbitrated_idx 进入 FIFO 后出来 o_qp_sq_idx, 也就是仲裁后的 SQ QPID
        .original_wqe_dvalid   (original_wqe_dvalid),     // m_axi_rvalid && m_axi_rready
        .original_wqe_arready  (original_wqe_arready),    // m_axi_arready & m_axi_arvalid
        .original_wqe_data_rdy (),
        .axi_m_en              (wqe_axi_m_en),            // input，sq_read_addr_to_axi 有效时拉高，使能 AXI 事务
        .ip2bus_addr           (sq_read_addr_to_axi),     // input，地址
        .ip2bus_len            (WQE_SIZE_IN_BYTES[13:0]), // input [13:0], 长度（以字节为单位）

        // Status indicators to the core IP 
        .lrdma_qp_mgr_axi_master_done  (lrdma_axi_master_done),
        .lrdma_qp_mgr_axi_master_busy  (),
        .lrdma_qp_mgr_axi_master_bvalid(),
        .lrdma_qp_mgr_axi_master_error (lrdma_axi_master_error_nc)

    );

    // 将 sw_override_qpid 信号扩展到每位对应一个 QP
    // sw_override_qpid 单指一个 QP
    genvar k0;
    generate
        for (k0 = 0; k0 < C_NUM_QP; k0 = k0 + 1) begin : sw_override_per_qp_loop
            assign sw_override_per_qp_en[k0] = sw_override_en & ((sw_override_qpid == k0) || (sw_override_qpid == 0));
        end
    endgenerate

    //--------状态观测寄存器逻辑结束---------------//
    `MSFF_R(axi_rd_err_cnt_ff, ((lrdma_axi_master_error_nc & lrdma_axi_master_done) ? ((axi_rd_err_cnt_ff == 8'hFF) ? axi_rd_err_cnt_ff : axi_rd_err_cnt_ff + 1) : axi_rd_err_cnt_ff), core_clk, ~core_rstn)


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 该队列管理器跟踪每个 QP 的 OSQ 满/空状态。
    // OSQ full 用于门控 SQ 仲裁，因此具有 full OSQ 的 QP 不会发出更多 WQE。
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    /***************************************************************************
   *                                                                          * 
   *                                                                          *
   *            FIFOs and arbitration to decide pop                           *
   *                                                                          *
   *                                                                          *
   ***************************************************************************/

    // i_pop_osq FIFO
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
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_i_osq_pop (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (i_osq_pop),
        .din          (i_osq_pop_qpid),
        .rd_en        (osq_pop_fifo_rden),
        .empty        (osq_pop_fifo_empty),
        .dout         (osq_pop_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (),
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

    // i_wqe_dropped FIFO
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
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_i_wqe_dropped (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (i_wqe_dropped),
        .din          (i_wqe_dropped_qpid),
        .rd_en        (in_wqe_dropped_fifo_rden),
        .empty        (in_wqe_dropped_fifo_empty),
        .dout         (in_wqe_dropped_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (),
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

    // retransmit_wqe_dropped FIFO
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
        .READ_DATA_WIDTH    (C_QP_INDX_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (C_QP_INDX_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_retransmit_wqe_droppped (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (retransmit_wqe_dropped),
        .din          (retransmit_wqe_dropped_qpid),
        .rd_en        (retransmit_wqe_fifo_rden),
        .empty        (retransmit_wqe_fifo_empty),
        .dout         (retransmit_wqe_fifo_dout),
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .full         (),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (),
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


    // lrdma_common_arbiter to arbitrate between the above 3 FIFOs
    assign arb_reqs = {!retransmit_wqe_fifo_empty, !in_wqe_dropped_fifo_empty, !osq_pop_fifo_empty};
    //assign arb_done = 1'b1;

    // Arbiter module
    lrdma_common_arbiter #(
        .NUM_REQS(3),
        .BUS_HOLD(0),
        .NEW_ARB (1),
        .SPLIT   (0)
    ) u_fifo_arbiter (
        .clk_arb  (core_clk),
        .rst_n_arb(core_rstn),
        .reqs     (arb_reqs),
        .arb_done (arb_done),
        .gnt      (arb_gnt),
        .gnt_valid()
    );

    assign osq_pop_fifo_rden        = arb_gnt[0] & arb_done;
    assign in_wqe_dropped_fifo_rden = arb_gnt[1] & arb_done;
    assign retransmit_wqe_fifo_rden = arb_gnt[2] & arb_done;

    // based on lrdma_common_arbiter output, one of the pop is served
    assign arb_gnt_popen            = |arb_gnt & arb_done;

    always @(*) begin
        if (arb_done) begin
            case (arb_gnt)
                3'b001: begin
                    arb_gnt_pop_qpid = osq_pop_fifo_dout;
                end
                3'b010: begin
                    arb_gnt_pop_qpid = in_wqe_dropped_fifo_dout;
                end
                3'b100: begin
                    arb_gnt_pop_qpid = retransmit_wqe_fifo_dout;
                end
                default: begin
                    arb_gnt_pop_qpid = {C_QP_INDX_WIDTH{1'b0}};
                end
            endcase
        end else begin
            arb_gnt_pop_qpid = {C_QP_INDX_WIDTH{1'b0}};
        end
    end

    // QP OSQ Buffer 管理
    // 不存放具体上下文，只有 ACK 管理器才存放具体 OSQ 上下文
    lrdma_queue_mgr #(
        .C_NUM_QUEUES    (C_NUM_QP),           // 32 个 QP，每个 QP 对应一个深度为 C_OS_Q_DEPTH 的 OSQ Buffer
        .C_Q_DEPTH       (C_OS_Q_DEPTH),
        .C_ENTRY_SIZE    (1),
        .C_THRESHOLD     (5),
        .C_PTR_WIDTH     (C_OS_Q_INDX_WIDTH),  // 4
        .C_ADDR_WIDTH    (OS_Q_ADDR_WIDTH),
        .C_ENABLE_WR_ADDR(0),
        .C_ENABLE_RD_ADDR(0)
    ) inst_osq_manager (
        .core_clk                   (core_clk),
        .core_rst                   (~core_rstn),
        .o_rd_addr                  (),
        .o_wr_addr                  (),
        .i_wr_addr_rd_en            (1'b0),
        .i_wr_addr_qnum             ({C_QP_INDX_WIDTH{1'b0}}),
        .i_wr_base_addr             ({OS_Q_ADDR_WIDTH{1'b0}}),
        .o_wr_addr_rd_valid         (),
        .i_rd_addr_rd_en            (1'b0),
        .i_rd_addr_qnum             ({C_QP_INDX_WIDTH{1'b0}}),
        .i_rd_base_addr             ({OS_Q_ADDR_WIDTH{1'b0}}),
        .o_rd_addr_rd_valid         (),
        .o_q_full_set_valid         (osq_full_set_valid),
        .o_q_full_set_id            (osq_full_set_id),
        .o_q_full_reset_valid       (osq_full_reset_valid),
        .o_q_full_reset_id          (osq_full_reset_id),
        .o_q_almost_full_set_valid  (),
        .o_q_almost_full_set_id     (),
        .o_q_almost_full_reset_valid(),
        .o_q_almost_full_reset_id   (),
        .o_q_empty_set_valid        (),
        .o_q_empty_set_id           (),
        .o_q_empty_reset_valid      (),
        .o_q_empty_reset_id         (),
        .vld_free_entry_rden        (1'b0),
        .vld_free_entry_qnum        ({C_QP_INDX_WIDTH{1'b0}}),
        .vld_free_entry_data_vld    (),
        .o_num_valid_entries        (),
        .o_num_free_entries         (),
        .i_q_flush                  (1'b0),
        .i_q_flush_qpid             ({C_QP_INDX_WIDTH{1'b0}}),
        .o_q_flush_done             (),
        .i_q_depth                  (C_OS_Q_DEPTH[C_OS_Q_INDX_WIDTH:0]),
        // push
        .i_q_push                   (wqe_axi_m_en),
        .i_q_push_qpid              (wqe_qpid_from_qp_cache),
        // pop
        .i_q_pop                    (arb_gnt_popen),
        .i_q_pop_qpid               (arb_gnt_pop_qpid),
        .o_q_push_idle              (),
        .o_q_pop_idle               (arb_done),
        .init_start                 (init_start),
        .init_count                 (init_count)
    );


endmodule
