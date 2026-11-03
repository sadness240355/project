`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_rx_engine_top — RX 引擎顶层封装模块
//
// 【模块概述】
// 将来自 CMAC 的 512 位以太网帧进行完整的 RoCEv2 协议处理。
// 采用"分类→验证→写入"的三阶段流水线设计。
//
// 【第一阶段：接收分类（rx_cmac_intf）】
// - 接收 AXI-Stream 数据，通过 EtherType 和 BTH 操作码分为请求/响应包
// - 分别写入独立 SDPRAM 缓冲区（请求包数据缓冲区为 32KB；请求头缓冲区为 32KB；响应头缓冲区为 32KB）
// - 并行执行 FCS/IPv4 校验和/ICRC 校验，结果打包到描述符 FIFO
//
// 【第二阶段：头部验证（并行处理）】
// - rx_req_hdr_val（请求路径）：三级流水线完成 MAC/IP/UDP/BTH 公共验证；
//   处理 QP1/MAD；普通 RC 请求按 unsupported/error 路径处理
// - rx_rsp_hdr_val（响应路径）：解析 AETH Syndrome，区分 ACK、SEQ-ERR-NAK 和 Fatal-NAK
//
// 【第三阶段：数据写入（rx_req_axi_master）】
// 通过 AXI4 写通道将验证通过的包数据搬移到 DDR 内存
//
// 【内部 SDPRAM 缓冲区（5 个）】
// - xpm_req_pkt_buf (512b×512)：请求包数据
// - xpm_req_hdr_buf (512b×512)：请求头部（两拍×512b = 128B 完整头）
// - xpm_req_hdr_buf_qpid (32b×256)：请求头 QP ID
// - xpm_rsp_hdr_buf (512b×512)：响应头部
//
// 【附加功能】
// - QP Fatal 管理：合并 lrdma_rx_req_hdr_validator 和 lrdma_rx_resp_hdr_validator 的 fatal 信号，
//   通过 LUTRAM 写使能管理 per-QP fatal 状态
// - Pause Threshold：根据缓冲区使用量生成 PFC 流控信号
// - 中断控制器（rx_intr_ctrl）：汇总包头验证错误、MAD 接收和 fatal 错误事件
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module lrdma_rx_engine_top #(
    parameter C_M_AXI_ADDR_WIDTH = 32,  // 32
    parameter C_M_AXI_AWUSER_WIDTH = 16,  // 32
    parameter C_NUM_QP = 256,  // 32
    parameter C_QP_INDX_WIDTH = 8  // 5
) (
    // System Clock & Reset
    input wire core_clk,
    input wire core_rst_n,

    // 以太网控制器接口，AXI-Stream数据包输入
    input wire         s_axis_tvalid,
    input wire [511:0] s_axis_tdata,
    input wire [ 63:0] s_axis_tkeep,
    input wire         s_axis_tlast,
    input wire [  0:0] s_axis_tuser,

    // LRDMA register interface
    input wire                       i_lrdma_en,       // LRDMA interface enable from LRDMA registers
    input wire [C_QP_INDX_WIDTH:0] i_num_qp_enabled, // 活动的 QP 数量

    input wire [ 47:0] i_mac_node_addr,   // 本地 (LRDMA) 设备的 MAC 地址
    input wire [ 31:0] i_ipv4_node_addr,  // 本地 (LRDMA) 设备的 IPv4 地址
    input wire [127:0] i_ipv6_node_addr,  // 本地 (LRDMA) 设备的 IPv6 地址

    input wire [1:0] i_flow_credits,  // 2'b00传入

    // 传入req数据包错误缓冲区配置
    input  wire                          i_req_err_buf_en,      // LRDMA Configuration寄存器 -- [5] Error buffer enable
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_req_err_pkt_buf_ba,  // Error logging queue base address  // tip:错误缓冲区的基地址
    input  wire [                  15:0] i_req_err_num_bufs,    // Error logging queue depth // tip:错误缓冲区的数量
    input  wire [                  15:0] i_req_err_buffer_sz,   // Error logging queue element buffer size in bytes // tip:每个错误缓冲区的大小
    output wire [                  15:0] o_req_err_buf_wrptr,   // 错误包队列写指针


    // 传入数据包的错误状态队列配置
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_in_errsts_q_ba,    // 传入数据包错误状态队列的基地址
    input  wire [                  15:0] i_in_errsts_q_sz,    // 传入数据包错误状态队列的大小
    output wire [                  15:0] o_in_errsts_q_wrptr,

    // 各数据包统计
    output wire [15:0] o_inc_acknak_cnt,    // 传入 ACK/NAK 响应包计数
    output wire [15:0] o_inc_mad_cnt,       // 传入 QP1 数据包计数，即 MAD 数据包计数
    output reg  [15:0] o_inc_inv_pkt_cnt,   // 传入无效数据包计数
    output wire [15:0] o_inc_nack_pkt_cnt,  // 传入 NAK 数据包计数
    output wire [31:0] pkt_all_cnt,         // 收到的所有数据包计数，丢弃包也计数
    output wire [31:0] pkt_disc_cnt,        // discarded packet count：丢弃数据包计数

    //input wire  [C_NUM_QP-1:0]                 i_qp_enable,
    input wire                       i_qp_enable_wr_en,
    input wire [C_QP_INDX_WIDTH-1:0] i_qp_enable_wr_idx,
    input wire                       i_qp_enable_wr_data,

    output wire [ 7:0] o_last_in_req_pkt_opcode,
    output wire [ 7:0] o_last_in_req_pkt_qpid,
    output wire [15:0] o_last_in_req_pkt_psn,

    output wire [ 7:0] o_last_in_rsp_pkt_opcode,
    output wire [ 7:0] o_last_in_rsp_pkt_qpid,
    output wire [15:0] o_last_in_rsp_pkt_psn,

    output wire [                7:0] o_last_in_pkt_opcode,
    output wire [C_QP_INDX_WIDTH-1:0] o_last_in_pkt_qpid,
    output wire [               15:0] o_last_in_pkt_psn,

    //output wire [C_NUM_QP-1:0]                 o_rq_full,                   // Receive request packet queue full
    output wire [C_QP_INDX_WIDTH-1:0] o_rq_full_idx,
    output wire                       o_rq_full_wr_en,
    output wire                       o_rq_full_val,
    output wire [C_QP_INDX_WIDTH-1:0] o_rq_full_reset_idx,
    output wire                       o_rq_full_reset_wr_en,
    output wire                       o_rq_full_reset_val,
    //output wire [C_NUM_QP-1:0]                 o_qp_fatal,                  // QPs those are stalled due to fatal error
    output wire                       o_qp_fatal_wr_en,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_fatal_wr_idx,
    output wire                       o_qp_fatal_wr_data,
    //input  wire [C_NUM_QP-1:0]                 i_clr_qp_fatal,              // removes stall condition on QPs
    input  wire                       i_clr_qp_fatal,         // removes stall condition on QPs
    input  wire [C_QP_INDX_WIDTH-1:0] i_clr_qp_fatal_idx,     // removes stall condition on QPs

    input wire                       qp_under_recovery_wr_en,
    input wire [C_QP_INDX_WIDTH-1:0] qp_under_recovery_wr_idx,
    input wire                       qp_under_recovery_wr_data,

    input wire [15:0] i_lrdma_pause_on_threshold,
    input wire [15:0] i_lrdma_pause_off_threshold,

    // Interrupt controller interface
    input  wire i_pkt_valdn_err_intr_en,   // Enable for Inbound packet header validation interrupt   
    output wire o_pkt_valdn_err_intr,      // Inbound packet header validation error interrupt
    output wire o_pkt_valdn_err_intr_sts,  // Inbound packet header validation error interrupt status flag
    input  wire i_clr_pkt_valdn_err_intr,  // Clear Inbound packet header validation error interrupt

    input  wire i_mad_pkt_rcvd_intr_en,   // Enable to MAD packet received interrupt  
    output wire o_mad_pkt_rcvd_intr,      // MAD packet received interrupt
    output wire o_mad_pkt_rcvd_intr_sts,  // MAD packet received interrupt status flag
    input  wire i_clr_mad_pkt_rcvd_intr,  // Clear MAD packet received interrupt

    input  wire i_fatal_err_intr_en,
    output wire o_fatal_err_intr,
    output wire o_fatal_err_intr_sts,
    input  wire i_clr_fatal_err_intr,

    //input  wire [C_NUM_QP-1:0]                 i_qp_retried,  
    input wire                       i_qp_retried_set_valid,
    input wire [C_QP_INDX_WIDTH-1:0] i_qp_retried_set_id,
    input wire                       i_qp_retried_reset_valid,
    input wire [C_QP_INDX_WIDTH-1:0] i_qp_retried_reset_id,

    output wire [C_QP_INDX_WIDTH-1:0] o_qp_index_acknack,    // QP Id for which ACKNOWLEDGE received
    output wire                       o_acknack_valid,       // valid pulse for this group of signals
    output wire [               23:0] o_acknacked_psn,       // ACK/NAK 响应携带的确认 PSN（送 ACK 管理器做完成/重传判断）
    output wire [                7:0] o_acknacked_syndrome,  // AETH.Syndrome in ACKNOWLEDGE  
    output wire [               23:0] o_acknacked_msn,       // AETH.MSN in ACKNOWLEDGE 
    output wire [               31:0] o_qp_timeout_per_qp,

    //input  wire [C_NUM_QP -1: 0]               i_osq_empty,
    input wire                                 i_osq_empty_set_valid,
    input wire [          C_QP_INDX_WIDTH-1:0] i_osq_empty_set_id,
    input wire                                 i_osq_empty_reset_valid,
    input wire [          C_QP_INDX_WIDTH-1:0] i_osq_empty_reset_id,
    //input  wire [C_NUM_QP -1: 0]               i_osq_nacked,
    input wire [(4*C_QP_INDX_WIDTH+4) - 1 : 0] i_osq_nacked,

    // TX 引擎 interface -1 
    input  wire [C_QP_INDX_WIDTH-1:0] i_qp_fatal_tx_engine,
    input  wire                       i_qp_fatal_tx_engine_vld,
    output wire                       o_qp_fatal_tx_engine_rdy,

    // ACK 管理器接口 -2 
    input  wire [C_QP_INDX_WIDTH-1:0] i_qp_fatal_ack_mgr,
    input  wire                       i_qp_fatal_ack_mgr_vld,
    output wire                       o_qp_fatal_ack_mgr_rdy,

    // QP configuration
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_req_conf_idx,  // qp1就是0

    input  wire [31:0] i_qp_req_conf,
    input  wire        qp_req_conf_vld,
    output wire        qp_req_conf_en,

    input  wire [C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_ba,
    output wire                          rq_buf_ba_en,

    input  wire [C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_ca,
    output wire [C_M_AXI_ADDR_WIDTH-9:0] o_rq_buf_ca,
    output wire                          rq_buf_ca_en,
    output wire                          rq_buf_ca_we,

    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_rq_wrptr_db_addr,
    output wire                          rq_wrptr_db_addr_en,

    input  wire [15:0] i_q_depth,
    output wire        q_depth_en,

    input  wire [15:0] i_rq_ci_db,
    output wire        rq_ci_db_en,

    input  wire [31:0] i_last_req,
    output wire [31:0] o_last_req,
    output wire        last_req_en,
    output wire        last_req_we,

    input  wire [47:0] i_qp_req_remote_qp_mac_addr,
    output wire        qp_req_remote_qp_mac_addr_en,

    input  wire [127:0] i_qp_req_remote_qp_ip_addr,
    output wire         qp_req_remote_qp_ip_addr_en,

    input  wire [15:0] i_rq_wrptr_db,
    output wire [15:0] o_rq_wrptr_db,
    output wire        rq_wrptr_db_en,
    output wire        rq_wrptr_db_we,

    output wire [C_QP_INDX_WIDTH-1:0] o_qp_rsp_conf_idx,

    input  wire [31:0] i_qp_rsp_conf,
    input  wire        qp_rsp_conf_vld,
    output wire        qp_rsp_conf_en,

    input  wire [6:0] i_stat_nak,
    output wire       stat_nak_en,
    output wire       stat_nak_we,
    output wire [6:0] o_stat_nak,

    input  wire [23:0] i_stat_resp_psn,
    output wire        stat_resp_psn_en,
    output wire        stat_resp_psn_we,
    output wire [23:0] o_stat_resp_psn,

    input  wire [23:0] i_sq_psn,
    output wire        sq_psn_en,

    input  wire [47:0] i_qp_rsp_remote_qp_mac_addr,
    output wire        qp_rsp_remote_qp_mac_addr_en,

    input  wire [127:0] i_qp_rsp_remote_qp_ip_addr,
    output wire         qp_rsp_remote_qp_ip_addr_en,

    input wire [31:0] i_qp_timeout,

    output reg o_qp_fatal_timer_idle,

    // RX 请求路径 AXI 写接口，用于 MAD/RQ 相关写回
    // rx_req_axi_mstr 模块传出的 AXI 接口
    output wire [                     0:0] rx_req_axi_awid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] rx_req_axi_awaddr,
    output wire [C_M_AXI_AWUSER_WIDTH-1:0] rx_req_axi_awuser,
    output wire [                     7:0] rx_req_axi_awlen,
    output wire [                     2:0] rx_req_axi_awsize,
    output wire [                     1:0] rx_req_axi_awburst,
    output wire [                     3:0] rx_req_axi_awcache,
    output wire [                     2:0] rx_req_axi_awprot,
    output wire                            rx_req_axi_awvalid,
    input  wire                            rx_req_axi_awready,
    output wire [                   511:0] rx_req_axi_wdata,
    output wire [                    63:0] rx_req_axi_wstrb,
    output wire                            rx_req_axi_wlast,
    output wire                            rx_req_axi_wvalid,
    input  wire                            rx_req_axi_wready,
    output wire                            rx_req_axi_awlock,
    input  wire [                     0:0] rx_req_axi_bid,
    input  wire [                     1:0] rx_req_axi_bresp,
    input  wire                            rx_req_axi_bvalid,
    output wire                            rx_req_axi_bready,
    output wire [                     0:0] rx_req_axi_arid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] rx_req_axi_araddr,
    output wire [                     7:0] rx_req_axi_arlen,
    output wire [                     2:0] rx_req_axi_arsize,
    output wire [                     1:0] rx_req_axi_arburst,
    output wire [                     3:0] rx_req_axi_arcache,
    output wire [                     2:0] rx_req_axi_arprot,
    output wire                            rx_req_axi_arvalid,
    input  wire                            rx_req_axi_arready,
    input  wire [                     0:0] rx_req_axi_rid,
    input  wire [                   511:0] rx_req_axi_rdata,
    input  wire [                     1:0] rx_req_axi_rresp,
    input  wire                            rx_req_axi_rlast,
    input  wire                            rx_req_axi_rvalid,
    output wire                            rx_req_axi_rready,
    output wire                            rx_req_axi_arlock,

    output wire                       waiting_for_rq_update,
    output wire [C_QP_INDX_WIDTH-1:0] waiting_for_rq_update_on_qp_num,

    output wire                       tx_pause_req_on,
    input  wire                       init_start,
    input  wire [C_QP_INDX_WIDTH-1:0] init_count
);

    // Local parameters
    // 数据位宽均为 512 bits = 64 Bytes, 即每个单元: 512位 = 64字节
    // 地址位宽计算空间: 总容量 = 2^地址位宽 * 数据位宽/8
    // 例: REQ_BUF总容量 = 2^9 * 64 = 2^5 KBytes = 32 KBytes
    localparam C_REQ_BUF_ADDR_WIDTH = 9;  // 32 KBytes, approx.
    localparam C_HDR_BUF_ADDR_WIDTH = 9;  // 32 KBytes, 256 packets 
    localparam C_CFG_MEM_ADDR_WIDTH = C_HDR_BUF_ADDR_WIDTH - 1;  // 32 KBytes, 256 packets；8

    localparam PKT_DESC_FIFO_DATA_WIDTH = C_REQ_BUF_ADDR_WIDTH + 16;  // 25
    localparam REQ_BUF_MEM_SZ_IN_BITS = (2 ** C_REQ_BUF_ADDR_WIDTH) * 512;
    localparam HDR_BUF_MEM_SZ_IN_BITS = (2 ** C_HDR_BUF_ADDR_WIDTH) * 512;
    localparam REQ_HDR_BUF_QPID_MEM_SZ_IN_BITS = (2 ** C_CFG_MEM_ADDR_WIDTH) * 32;

    // 原 'd70 硬编码 = 最大 PMTU 4096B / 64B beat (= 64 beats) + 6 beat 安全余量。
    // PMTU 上限放宽时须同步调整此 localparam。
    localparam REQ_BUF_ALMOST_FULL_MARGIN = (4096 / 64) + 6;  // 70

    wire                                core_rst;

    wire                                incr_inv_req_pkt_cnt;

    wire [         C_QP_INDX_WIDTH-1:0] qp_req_fatal_idx;
    wire                                set_qp_req_fatal;
    wire [         C_QP_INDX_WIDTH-1:0] qp_rsp_fatal_idx;
    wire                                set_qp_rsp_fatal;

    reg                                 set_qp_req_fatal_r;
    reg                                 set_qp_rsp_fatal_r;
    reg                                 clr_qp_fatal_r;
    reg  [         C_QP_INDX_WIDTH-1:0] req_fatal_idx_r;
    reg  [         C_QP_INDX_WIDTH-1:0] rsp_fatal_idx_r;
    reg  [         C_QP_INDX_WIDTH-1:0] clr_qp_fatal_idx_r;



    wire                                set_req_pkt_valdn_err_intr;
    wire                                set_pkt_valdn_err_intr;
    wire                                set_mad_pkt_rcvd_intr;
    wire                                set_fatal_err_intr;

    wire [                        31:0] req_pkt_id;
    wire [         C_QP_INDX_WIDTH-1:0] req_qpid;
    wire [                         3:0] req_pkt_type;
    wire                                req_pkt_ip_ver;
    wire [                        15:0] req_rq_pi_db;
    wire [      C_M_AXI_ADDR_WIDTH-1:0] req_dest_addr;
    wire [    C_REQ_BUF_ADDR_WIDTH-1:0] req_buf_addr_offset;
    wire [                        12:0] req_pkt_len;
    wire [                        43:0] req_imm_data;
    wire                                req_tnfr_vld;
    wire                                rdy_to_req_tnfr;

    wire [                        31:0] o_req_pkt_id;
    wire [                         3:0] o_req_pkt_type;
    wire                                o_req_pkt_ip_ver;
    wire [                        15:0] o_req_rq_pi_db;
    wire [         C_QP_INDX_WIDTH-1:0] o_req_qpid;
    wire [      C_M_AXI_ADDR_WIDTH-1:0] o_req_dest_addr;
    wire [    C_REQ_BUF_ADDR_WIDTH-1:0] o_req_buf_addr_offset;
    wire [                        12:0] o_req_pkt_len;
    wire [                        43:0] o_req_imm_data;
    wire                                o_req_tnfr_vld;
    wire                                i_rdy_to_req_tnfr;


    wire [                        31:0] o_req_err_pkt_id;
    // 请求错误路径只输出 pkt_id、imm_data 和 valid，pkt_type 不接出
    wire [                        31:0] o_req_err_imm_data;
    wire                                o_req_err_tnfr_vld;
    wire                                i_rdy_to_req_err_tnfr;

    wire                                req_err_tnfr_vld;
    wire                                rdy_to_req_err_tnfr;

    // lrdma_rx_resp_hdr_validator outputs used by err FIFO
    wire [                        31:0] rsp_pkt_id;
    wire [                        31:0] rsp_imm_data;

    wire [    C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addra;
    wire                                req_pkt_buf_ena;
    wire [                        63:0] req_pkt_buf_wea;
    wire [                       511:0] req_pkt_buf_dina;
    wire [    C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addrb;
    wire                                req_pkt_buf_enb;
    wire [                       511:0] req_pkt_buf_doutb;

    wire [    C_HDR_BUF_ADDR_WIDTH-1:0] req_hdr_buf_addra;
    wire                                req_hdr_buf_ena;
    wire [                        63:0] req_hdr_buf_wea;
    wire [                       511:0] req_hdr_buf_dina;
    wire [    C_HDR_BUF_ADDR_WIDTH-1:0] req_hdr_buf_addrb;
    wire                                req_hdr_buf_enb;
    wire [                       511:0] req_hdr_buf_doutb;


    wire [                         3:0] req_hdr_qpid_buf_wea;
    wire [                        23:0] req_hdr_qpid_buf_dina;
    wire [                         7:0] bth_opcode;
    wire [                        31:0] req_hdr_qpid_buf_doutb;


    wire [                        15:0] ether_type_ipv4;
    reg                                 ipv6_valid;

    wire [    C_HDR_BUF_ADDR_WIDTH-1:0] rsp_hdr_buf_addra;
    wire                                rsp_hdr_buf_ena;
    wire [                        63:0] rsp_hdr_buf_wea;
    wire [                       511:0] rsp_hdr_buf_dina;
    wire [    C_HDR_BUF_ADDR_WIDTH-1:0] rsp_hdr_buf_addrb;
    wire                                rsp_hdr_buf_enb;
    wire [                       511:0] rsp_hdr_buf_doutb;

    // req 包描述符 lrdma_common_fifo 接口
    wire [PKT_DESC_FIFO_DATA_WIDTH-1:0] req_pkt_desc_fifo_dout;
    wire                                req_pkt_desc_fifo_empty;
    reg                                 req_pkt_desc_fifo_empty_r1;
    wire                                rd_req_pkt_desc_fifo;

    // rsp 包描述符 lrdma_common_fifo 接口
    wire [PKT_DESC_FIFO_DATA_WIDTH-1:0] rsp_pkt_desc_fifo_dout;
    wire                                rsp_pkt_desc_fifo_empty;
    wire                                rd_rsp_pkt_desc_fifo;

    wire [      C_REQ_BUF_ADDR_WIDTH:0] req_pkt_buf_sz;  // req_pkt_buf 总单元数
    wire [      C_HDR_BUF_ADDR_WIDTH:0] rsp_hdr_buf_sz;  // rsp_hdr_buf 总单元数
    wire [      C_HDR_BUF_ADDR_WIDTH:0] req_hdr_buf_sz;  // req_hdr_buf 总单元数

    reg                                 req_pkt_buf_almost_full;

    reg  [                         5:0] lrdma_en_reg;
    wire                                lrdma_en_pulse;

    reg  [                        31:0] free_run_cnt;

    wire                                load_qp_fatal_timer;
    wire                                qp_fatal_timer_idle;
    wire                                clr_qp_fatal_timer;

    wire                                rsp_fifos_empty;

    reg  [                         9:0] qp_fatal_timer;

    reg                                 rsp_hdr_buf_pause_on;
    reg                                 req_pkt_buf_pause_on;
    reg                                 req_hdr_buf_pause_on;
    reg  [                         4:0] assert_cnt;
    reg                                 cnt_en;

    // Other miscellanous logic & internal signals
    assign core_rst = ~core_rst_n;

    always @(posedge core_clk)
        if (core_rst) o_inc_inv_pkt_cnt <= 16'd0;
        else o_inc_inv_pkt_cnt <= o_inc_inv_pkt_cnt + incr_inv_req_pkt_cnt;

    //////////////////////////////////////////////////////////////////////////////////////////////
    // set_qp_req_fatal、set_qp_rsp_fatal 与 clr_qp_fatal 均为单拍脉冲；
    // 相邻脉冲间隔至少 4 个 core_clk 周期。
    //////////////////////////////////////////////////////////////////////////////////////////////

    always @(posedge core_clk) begin
        if (core_rst) begin
            set_qp_req_fatal_r <= 1'b0;
            set_qp_rsp_fatal_r <= 1'b0;
            clr_qp_fatal_r     <= 1'b0;
            req_fatal_idx_r    <= 'd0;
            rsp_fatal_idx_r    <= 'd0;
        end else begin
            set_qp_req_fatal_r <= set_qp_req_fatal;
            req_fatal_idx_r    <= qp_req_fatal_idx;
            if (~(set_qp_req_fatal_r && set_qp_rsp_fatal_r)) begin
                set_qp_rsp_fatal_r <= set_qp_rsp_fatal;
                rsp_fatal_idx_r    <= qp_rsp_fatal_idx;
            end
            if (~((set_qp_req_fatal_r || set_qp_rsp_fatal_r) && clr_qp_fatal_r)) begin
                clr_qp_fatal_r     <= i_clr_qp_fatal;
                clr_qp_fatal_idx_r <= i_clr_qp_fatal_idx;
            end
        end
    end

    assign o_qp_fatal_wr_en    = set_qp_req_fatal_r || set_qp_rsp_fatal_r || clr_qp_fatal_r;
    assign o_qp_fatal_wr_idx   = set_qp_req_fatal_r ? req_fatal_idx_r : (set_qp_rsp_fatal_r ? rsp_fatal_idx_r : clr_qp_fatal_idx_r);
    assign o_qp_fatal_wr_data  = (set_qp_req_fatal_r || set_qp_rsp_fatal_r) ? 1'b1 : 1'b0;


    //assign load_qp_fatal_timer = set_qp_req_fatal | set_qp_rsp_fatal | (|qp_under_recovery);
    assign load_qp_fatal_timer = set_qp_req_fatal | set_qp_rsp_fatal | (qp_under_recovery_wr_en && qp_under_recovery_wr_data);

    // 响应侧 AXI 写主站无实例，响应 FIFO 维持空状态，无需 drain 延迟
    assign rsp_fifos_empty     = 1'b1;

    always @(posedge core_clk)
        if (core_rst) qp_fatal_timer <= 10'd0;
        else if (load_qp_fatal_timer) qp_fatal_timer <= 10'h3FF;
        else if (rsp_fifos_empty) qp_fatal_timer <= 10'd0;

    assign qp_fatal_timer_idle = (qp_fatal_timer == 10'd0) ? 1'b1 : 1'b0;

    assign clr_qp_fatal_timer  = rsp_fifos_empty | qp_fatal_timer_idle;

    always @(posedge core_clk)
        if (core_rst) o_qp_fatal_timer_idle <= 1'b1;
        else if (load_qp_fatal_timer || clr_qp_fatal_timer) o_qp_fatal_timer_idle <= ~load_qp_fatal_timer;

    assign set_pkt_valdn_err_intr = set_req_pkt_valdn_err_intr;

    assign req_pkt_buf_sz         = {1'b1, {(C_REQ_BUF_ADDR_WIDTH) {1'b0}}};  // 2^9 = 512个单元, 即可以包含 512 个 512bit(64B) 的数据
    assign req_hdr_buf_sz         = {1'b1, {(C_HDR_BUF_ADDR_WIDTH) {1'b0}}};
    assign rsp_hdr_buf_sz         = {1'b1, {(C_HDR_BUF_ADDR_WIDTH) {1'b0}}};

    // 检测 req_pkt_buf 空闲空间单元是否 < REQ_BUF_ALMOST_FULL_MARGIN
    always @(posedge core_clk)
        if (core_rst) req_pkt_buf_almost_full <= 1'b0;
        else if (req_pkt_buf_addra >= req_pkt_buf_addrb) begin
            if ((req_pkt_buf_sz - req_pkt_buf_addra + req_pkt_buf_addrb) < REQ_BUF_ALMOST_FULL_MARGIN) req_pkt_buf_almost_full <= 1'b1;
            else req_pkt_buf_almost_full <= 1'b0;
        end else if (req_pkt_buf_addrb - req_pkt_buf_addra < REQ_BUF_ALMOST_FULL_MARGIN) req_pkt_buf_almost_full <= 1'b1;
        else req_pkt_buf_almost_full <= 1'b0;


    //Pause threshold generation
    assign tx_pause_req_on = (rsp_hdr_buf_pause_on | req_pkt_buf_pause_on | req_hdr_buf_pause_on);

    always @(posedge core_clk)
        if (core_rst) cnt_en <= 1'b0;
        else if (tx_pause_req_on) cnt_en <= 1'b1;
        else if (assert_cnt == 5'h1e) cnt_en <= 1'b0;

    always @(posedge core_clk)
        if (core_rst) assert_cnt <= 5'h0;
        else if (cnt_en || tx_pause_req_on) assert_cnt <= assert_cnt + 1;
        else if (assert_cnt == 5'h1f) assert_cnt <= assert_cnt;


    always @(posedge core_clk)
        if (core_rst) req_pkt_buf_pause_on <= 1'b0;
        else if (req_pkt_buf_addra >= req_pkt_buf_addrb) begin
            if ((req_pkt_buf_sz - req_pkt_buf_addra + req_pkt_buf_addrb) < i_lrdma_pause_on_threshold) req_pkt_buf_pause_on <= 1'b1;
            else if (((req_pkt_buf_sz - req_pkt_buf_addra + req_pkt_buf_addrb) > i_lrdma_pause_off_threshold) && assert_cnt == 5'hf) req_pkt_buf_pause_on <= 1'b0;
        end else if ((req_pkt_buf_addrb - req_pkt_buf_addra) < i_lrdma_pause_on_threshold) req_pkt_buf_pause_on <= 1'b1;
        else if (((req_pkt_buf_addrb - req_pkt_buf_addra) > i_lrdma_pause_off_threshold) && assert_cnt == 5'hf) req_pkt_buf_pause_on <= 1'b0;

    always @(posedge core_clk)
        if (core_rst) req_hdr_buf_pause_on <= 1'b0;
        else if (req_hdr_buf_addra >= req_hdr_buf_addrb) begin
            if ((req_hdr_buf_sz - req_hdr_buf_addra + req_hdr_buf_addrb) < i_lrdma_pause_on_threshold) req_hdr_buf_pause_on <= 1'b1;
            else if (((req_hdr_buf_sz - req_hdr_buf_addra + req_hdr_buf_addrb) > i_lrdma_pause_off_threshold) && assert_cnt == 5'hf) req_hdr_buf_pause_on <= 1'b0;
        end else if ((req_hdr_buf_addrb - req_hdr_buf_addra) < i_lrdma_pause_on_threshold) req_hdr_buf_pause_on <= 1'b1;
        else if (((req_hdr_buf_addrb - req_hdr_buf_addra) > i_lrdma_pause_off_threshold) && assert_cnt == 5'hf) req_hdr_buf_pause_on <= 1'b0;

    always @(posedge core_clk)
        if (core_rst) rsp_hdr_buf_pause_on <= 1'b0;
        else if (rsp_hdr_buf_addra >= rsp_hdr_buf_addrb) begin
            if ((rsp_hdr_buf_sz - rsp_hdr_buf_addra + rsp_hdr_buf_addrb) < i_lrdma_pause_on_threshold) rsp_hdr_buf_pause_on <= 1'b1;
            else if (((rsp_hdr_buf_sz - rsp_hdr_buf_addra + rsp_hdr_buf_addrb) > i_lrdma_pause_off_threshold) && assert_cnt == 5'hf) rsp_hdr_buf_pause_on <= 1'b0;
        end else if ((rsp_hdr_buf_addrb - rsp_hdr_buf_addra) < i_lrdma_pause_on_threshold) rsp_hdr_buf_pause_on <= 1'b1;
        else if (((rsp_hdr_buf_addrb - rsp_hdr_buf_addra) > i_lrdma_pause_off_threshold) && assert_cnt == 5'hf) rsp_hdr_buf_pause_on <= 1'b0;


    // lrdma_en开始一直拉高时，下下个时钟周期lrdma_en_pulse拉高，且保持4个时钟周期，随后拉低
    always @(posedge core_clk)
        if (core_rst) lrdma_en_reg <= {6{1'b0}};
        else lrdma_en_reg <= {lrdma_en_reg[5:0], i_lrdma_en};
    assign lrdma_en_pulse = ~lrdma_en_reg[5] & lrdma_en_reg[1];  // 当i_lrdma_en拉高时，下下个时钟周期lrdma_en_pulse拉高，且保持4个时钟周期

    always @(posedge core_clk)
        if (core_rst) free_run_cnt <= 32'd0;
        else free_run_cnt <= free_run_cnt + 'd1;

    // 1、包分类：解析以太网、IP、BTH 等协议头并进行 FCS、IPv4 头校验和、ICRC 校验，以区分请求包和响应包
    // 2、缓冲区管理：根据包类型 (req | rsp) 将包头、完整包 (不含 FCS)、包描述符写入不同的缓冲区；若完整包缓冲区已满则丢弃该包
    // 3、统计信息：统计接收的包总数、丢弃包数、最后接收包的操作码、QP ID、PSN
    lrdma_rx_cmac_intf #(
        .C_NUM_QP(C_NUM_QP),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .PKT_DESC_FIFO_DATA_WIDTH(PKT_DESC_FIFO_DATA_WIDTH),
        .C_REQ_BUF_ADDR_WIDTH(C_REQ_BUF_ADDR_WIDTH),
        .C_HDR_BUF_ADDR_WIDTH(C_HDR_BUF_ADDR_WIDTH)
    ) u_rx_cmac_intf (
        .clk          (core_clk),
        .reset        (core_rst),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tdata (s_axis_tdata),
        .s_axis_tkeep (s_axis_tkeep),
        .s_axis_tlast (s_axis_tlast),
        .s_axis_tuser (s_axis_tuser),

        .lrdma_en(i_lrdma_en),

        .o_inc_all_pkt_cnt   (pkt_all_cnt),
        .o_inc_drop_pkt_cnt  (pkt_disc_cnt),
        .o_last_in_pkt_opcode(o_last_in_pkt_opcode),
        .o_last_in_pkt_qpid  (o_last_in_pkt_qpid),
        .o_last_in_pkt_psn   (o_last_in_pkt_psn),

        .req_pkt_buf_almost_full(req_pkt_buf_almost_full),  // 请求包缓冲区快满标志：剩余可用 beat 数 < REQ_BUF_ALMOST_FULL_MARGIN，即 70

        .req_pkt_desc_fifo_dout(req_pkt_desc_fifo_dout),
        .rd_req_pkt_desc_fifo  (rd_req_pkt_desc_fifo),

        .rsp_pkt_desc_fifo_dout(rsp_pkt_desc_fifo_dout),
        .rd_rsp_pkt_desc_fifo  (rd_rsp_pkt_desc_fifo),

        // 包
        .req_pkt_buf_addra(req_pkt_buf_addra),
        .req_pkt_buf_ena  (req_pkt_buf_ena),
        .req_pkt_buf_wea  (req_pkt_buf_wea),
        .req_pkt_buf_dina (req_pkt_buf_dina),
        // 包头
        .req_hdr_buf_addra(req_hdr_buf_addra),
        .req_hdr_buf_ena  (req_hdr_buf_ena),
        .req_hdr_buf_wea  (req_hdr_buf_wea),
        .req_hdr_buf_dina (req_hdr_buf_dina),

        // 包头
        .rsp_hdr_buf_addra(rsp_hdr_buf_addra),
        .rsp_hdr_buf_ena  (rsp_hdr_buf_ena),
        .rsp_hdr_buf_wea  (rsp_hdr_buf_wea),
        .rsp_hdr_buf_dina (rsp_hdr_buf_dina),

        .req_pkt_desc_fifo_empty(req_pkt_desc_fifo_empty),
        .rsp_pkt_desc_fifo_empty(rsp_pkt_desc_fifo_empty)
    );

    always @(posedge core_clk) begin
        if (core_rst) req_pkt_desc_fifo_empty_r1 <= 1;
        else req_pkt_desc_fifo_empty_r1 <= req_pkt_desc_fifo_empty;
    end

    // ----------------------------------------------------------------------------------------------

    // ======================== 请求包缓冲区 sdpram ========================
    xpm_memory_sdpram #(  // Common module parameters
        .MEMORY_SIZE      (REQ_BUF_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                  //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),          //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                // 关闭 ECC
        .MEMORY_INIT_FILE ("none"),                  //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                      //string;
        .WAKEUP_TIME      ("disable_sleep"),         //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(512),                  //positive integer
        .BYTE_WRITE_WIDTH_A(8),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_REQ_BUF_ADDR_WIDTH), //positive integer

        // Port B module parameters                   
        .READ_DATA_WIDTH_B (512),                   //positive integer
        .ADDR_WIDTH_B      (C_REQ_BUF_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 
    ) xpm_req_pkt_buf (
        // Common module ports
        .sleep(1'b0),  // sleep 固定关闭

        // Port A module ports
        .clka          (core_clk),
        .ena           (req_pkt_buf_ena),
        .wea           (req_pkt_buf_wea),
        .addra         (req_pkt_buf_addra),
        .dina          (req_pkt_buf_dina),
        .injectsbiterra(1'b0),               // A 端不注入单比特错误
        .injectdbiterra(1'b0),               // A 端不注入双比特错误

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (req_pkt_buf_enb),
        .regceb  (1'b1),
        .addrb   (req_pkt_buf_addrb),
        .doutb   (req_pkt_buf_doutb),
        .sbiterrb(),                   // B 端单比特错误状态未接出
        .dbiterrb()                    // B 端双比特错误状态未接出
    );

    // ======================== 请求包头缓冲区 sdpram ========================
    xpm_memory_sdpram #(
        // Common module parameters
        .MEMORY_SIZE      (HDR_BUF_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                  //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),          //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                // 关闭 ECC
        .MEMORY_INIT_FILE ("none"),                  //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                      //string;
        .WAKEUP_TIME      ("disable_sleep"),         //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(512),                  //positive integer
        .BYTE_WRITE_WIDTH_A(512),                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_HDR_BUF_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (512),                   //positive integer
        .ADDR_WIDTH_B      (C_HDR_BUF_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer，读数据输出延迟一个周期
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 

    ) xpm_req_hdr_buf (
        // Common module ports
        .sleep(1'b0),  // sleep 固定关闭

        // Port A module ports
        .clka          (core_clk),
        .ena           (req_hdr_buf_ena),
        .wea           (req_hdr_buf_wea[0]),
        .addra         (req_hdr_buf_addra),
        .dina          (req_hdr_buf_dina),
        .injectsbiterra(1'b0),                // A 端不注入单比特错误
        .injectdbiterra(1'b0),                // A 端不注入双比特错误

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (req_hdr_buf_enb),
        .regceb  (1'b1),
        .addrb   (req_hdr_buf_addrb),
        .doutb   (req_hdr_buf_doutb),
        .sbiterrb(),                   // B 端单比特错误状态未接出
        .dbiterrb()                    // B 端双比特错误状态未接出
    );

    assign ether_type_ipv4 = {req_hdr_buf_dina[12*8+:8], req_hdr_buf_dina[13*8+:8]};
    assign bth_opcode      = (ether_type_ipv4 == 16'h0800) ? req_hdr_buf_dina[42*8+:8] : req_hdr_buf_dina[62*8+:8];

    always @(posedge core_clk) begin
        if (core_rst) begin
            ipv6_valid <= 1'b0;
        end else begin
            if (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b0 && ether_type_ipv4 != 16'h0800) ipv6_valid <= 1'b1;
            else if (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b1) ipv6_valid <= 1'b0;

        end
    end

    // ======================== 请求包头 {BTH头部 bth_opcode 字段, BTH头部 Dest QP 字段} sdpram ========================
    // 包第一拍或 IPv6 包时，QPID 字段三个字节均需写入，掩码全 1；否则全 0
    assign req_hdr_qpid_buf_wea[2:0] = (req_hdr_buf_ena == 1'b1 && (req_hdr_buf_addra[0] == 1'b0 || ipv6_valid == 1'b1)) ? 3'h7 : 3'h0;
    // 仅在包第一拍写入 opcode 字段，掩码为 1；否则为 0
    assign req_hdr_qpid_buf_wea[3] = (req_hdr_buf_ena == 1'b1 && req_hdr_buf_addra[0] == 1'b0) ? 1'b1 : 1'b0;
    // req_hdr_buf_addra[0] == 1'b0 表示包第一拍，req_hdr_buf_addra[0] == 1'b1 表示包第二拍
    // {req_hdr_buf_dina[47*8+:8], req_hdr_buf_dina[48*8+:8], req_hdr_buf_dina[49*8+:8]}是bth头部的QP字段
    assign req_hdr_qpid_buf_dina     = (req_hdr_buf_addra[0] == 1'b0) ? 
                                        {{req_hdr_buf_dina[47*8+:8], req_hdr_buf_dina[48*8+:8], req_hdr_buf_dina[49*8+:8]} >= 24'd1 ? {req_hdr_buf_dina[47*8+:8], req_hdr_buf_dina[48*8+:8], req_hdr_buf_dina[49*8+:8]} - 24'd1 : 24'd0} 
                                        : {{req_hdr_buf_dina[3*8+:8], req_hdr_buf_dina[4*8+:8], req_hdr_buf_dina[5*8+:8]} >= 24'd1 ? {req_hdr_buf_dina[3*8+:8], req_hdr_buf_dina[4*8+:8], req_hdr_buf_dina[5*8+:8]} - 24'd1 : 24'd0};
    xpm_memory_sdpram #(
        // Common module parameters
        .MEMORY_SIZE      (REQ_HDR_BUF_QPID_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                           //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),                   //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                         // 关闭 ECC
        .MEMORY_INIT_FILE ("none"),                           //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                               //string;
        .WAKEUP_TIME      ("disable_sleep"),                  //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(32),                   //positive integer
        .BYTE_WRITE_WIDTH_A(8),                    //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_CFG_MEM_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (32),                    //positive integer
        .ADDR_WIDTH_B      (C_CFG_MEM_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 

    ) xpm_req_hdr_buf_qpid (
        // Common module ports
        .sleep(1'b0),  // sleep 固定关闭

        // Port A module ports
        .clka(core_clk),
        .ena (req_hdr_buf_ena),
        .wea (req_hdr_qpid_buf_wea),

        // 与包头索引对齐，若第2个包头（两拍）（包头索引为1）存在0x02和0x03中，即第3个、第4个单元，
        // 则{bth_opcode, req_hdr_qpid_buf_dina}存在第1个单元（与包头索引对齐），即0x01中
        .addra         (req_hdr_buf_addra[C_HDR_BUF_ADDR_WIDTH-1:1]),
        .dina          ({bth_opcode, req_hdr_qpid_buf_dina}),          // 高8位存opcode，低24位存qpid
        .injectsbiterra(1'b0),                                         // A 端不注入单比特错误
        .injectdbiterra(1'b0),                                         // A 端不注入双比特错误


        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (req_hdr_buf_enb),
        .regceb  (1'b1),
        .addrb   (req_hdr_buf_addrb[C_HDR_BUF_ADDR_WIDTH-1:1]),
        .doutb   (req_hdr_qpid_buf_doutb),                       // {bth_opcode, qpid}
        .sbiterrb(),                                             // B 端单比特错误状态未接出
        .dbiterrb()                                              // B 端双比特错误状态未接出
    );


    // ======================== 响应包头缓冲区 sdpram ========================
    xpm_memory_sdpram #(
        // Common module parameters
        .MEMORY_SIZE      (HDR_BUF_MEM_SZ_IN_BITS),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                  //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),          //string; "common_clock", "independent_clock" 
        .ECC_MODE         ("no_ecc"),                // 关闭 ECC
        .MEMORY_INIT_FILE ("none"),                  //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                      //string;
        .WAKEUP_TIME      ("disable_sleep"),         //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(512),                  //positive integer
        .BYTE_WRITE_WIDTH_A(512),                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (C_HDR_BUF_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (512),                   //positive integer
        .ADDR_WIDTH_B      (C_HDR_BUF_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                   //string
        .READ_LATENCY_B    (1),                     //non-negative integer
        .WRITE_MODE_B      ("read_first")           //string; "write_first", "read_first", "no_change" 

    ) xpm_rsp_hdr_buf (
        // Common module ports
        .sleep(1'b0),  // sleep 固定关闭

        // Port A module ports
        .clka          (core_clk),
        .ena           (rsp_hdr_buf_ena),
        .wea           (rsp_hdr_buf_wea[0]),  // 因为 BYTE_WRITE_WIDTH_A = 512，WRITE_DATA_WIDTH_A = 512，所以wea宽度=1
        .addra         (rsp_hdr_buf_addra),   // 两拍对齐，包头存两拍
        .dina          (rsp_hdr_buf_dina),
        .injectsbiterra(1'b0),                // A 端不注入单比特错误
        .injectdbiterra(1'b0),                // A 端不注入双比特错误

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (1'b0),
        .enb     (rsp_hdr_buf_enb),
        .regceb  (1'b1),
        .addrb   (rsp_hdr_buf_addrb),
        .doutb   (rsp_hdr_buf_doutb),
        .sbiterrb(),                   // B 端单比特错误状态未接出
        .dbiterrb()                    // B 端双比特错误状态未接出
    );

    lrdma_rx_req_hdr_validator #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .C_NUM_QP(C_NUM_QP),
        .PKT_DESC_FIFO_DATA_WIDTH(PKT_DESC_FIFO_DATA_WIDTH),
        .C_HDR_BUF_ADDR_WIDTH(C_HDR_BUF_ADDR_WIDTH),
        .C_REQ_BUF_ADDR_WIDTH(C_REQ_BUF_ADDR_WIDTH)
    ) u_rx_req_hdr_validator (
        .clk                     (core_clk),
        .reset                   (core_rst),
        .req_pkt_desc_fifo_dout  (req_pkt_desc_fifo_dout),
        .req_pkt_desc_fifo_empty (req_pkt_desc_fifo_empty),
        .rd_req_pkt_desc_fifo    (rd_req_pkt_desc_fifo),
        .req_hdr_buf_addrb       (req_hdr_buf_addrb),
        .req_hdr_buf_enb         (req_hdr_buf_enb),
        .req_hdr_buf_doutb       (req_hdr_buf_doutb),
        .req_hdr_qpid_buf_doutb  (req_hdr_qpid_buf_doutb),
        .i_err_buf_en            (i_req_err_buf_en),
        .i_mac_node_addr         (i_mac_node_addr),
        .i_ipv4_node_addr        (i_ipv4_node_addr),
        .i_ipv6_node_addr        (i_ipv6_node_addr),
        .i_num_qp_enabled        (i_num_qp_enabled),
        .i_flow_credits          (i_flow_credits),
        .o_last_in_req_pkt_opcode(o_last_in_req_pkt_opcode),
        .o_last_in_req_pkt_qpid  (o_last_in_req_pkt_qpid),
        .o_last_in_req_pkt_psn   (o_last_in_req_pkt_psn),
        //.o_rq_full               (o_rq_full),
        .o_rq_full_idx           (o_rq_full_idx),
        .o_rq_full_wr_en         (o_rq_full_wr_en),
        .o_rq_full_val           (o_rq_full_val),
        .o_rq_full_reset_idx     (o_rq_full_reset_idx),
        .o_rq_full_reset_wr_en   (o_rq_full_reset_wr_en),
        .o_rq_full_reset_val     (o_rq_full_reset_val),
        //.i_rd_req_fifo_full      (i_rd_req_fifo_full),
        .o_qp_conf_idx           (o_qp_req_conf_idx),
        .i_qp_conf               (i_qp_req_conf),
        .qp_conf_vld             (qp_req_conf_vld),
        .qp_conf_en              (qp_req_conf_en),
        .i_rq_buf_ba             (i_rq_buf_ba),
        .rq_buf_ba_en            (rq_buf_ba_en),
        .i_rq_buf_ca             (i_rq_buf_ca),
        .o_rq_buf_ca             (o_rq_buf_ca),
        .rq_buf_ca_en            (rq_buf_ca_en),
        .rq_buf_ca_we            (rq_buf_ca_we),
        .i_rq_wrptr_db_addr      (i_rq_wrptr_db_addr),
        .rq_wrptr_db_addr_en     (rq_wrptr_db_addr_en),
        .i_q_depth               (i_q_depth),
        .q_depth_en              (q_depth_en),
        .i_rq_ci_db              (i_rq_ci_db),
        .rq_ci_db_en             (rq_ci_db_en),
        .i_last_req              (i_last_req),
        .o_last_req              (o_last_req),
        .last_req_en             (last_req_en),
        .last_req_we             (last_req_we),
        .i_remote_qp_mac_addr    (i_qp_req_remote_qp_mac_addr),
        .remote_qp_mac_addr_en   (qp_req_remote_qp_mac_addr_en),
        .i_remote_qp_ip_addr     (i_qp_req_remote_qp_ip_addr),
        .remote_qp_ip_addr_en    (qp_req_remote_qp_ip_addr_en),
        .i_rq_wrptr_db           (i_rq_wrptr_db),
        .o_rq_wrptr_db           (o_rq_wrptr_db),
        .rq_wrptr_db_en          (rq_wrptr_db_en),
        .rq_wrptr_db_we          (rq_wrptr_db_we),
        //.i_qp_fatal              (qp_fatal),
        .i_qp_fatal_wr_en        (o_qp_fatal_wr_en),
        .i_qp_fatal_wr_idx       (o_qp_fatal_wr_idx),
        .i_qp_fatal_wr_data      (o_qp_fatal_wr_data),
        .o_qp_fatal_idx          (qp_req_fatal_idx),
        .set_qp_fatal            (set_qp_req_fatal),
        .rsp_qp_fatal_idx        (qp_rsp_fatal_idx),
        .set_rsp_qp_fatal        (set_qp_rsp_fatal),
        .i_qp_fatal_tx_engine    (i_qp_fatal_tx_engine),
        .i_qp_fatal_tx_engine_vld(i_qp_fatal_tx_engine_vld),
        .o_qp_fatal_tx_engine_rdy(o_qp_fatal_tx_engine_rdy),
        .i_qp_fatal_ack_mgr      (i_qp_fatal_ack_mgr),
        .i_qp_fatal_ack_mgr_vld  (i_qp_fatal_ack_mgr_vld),
        .o_qp_fatal_ack_mgr_rdy  (o_qp_fatal_ack_mgr_rdy),
        .req_pkt_id              (req_pkt_id),
        .req_qpid                (req_qpid),
        .req_pkt_type            (req_pkt_type),
        .req_pkt_ip_ver          (req_pkt_ip_ver),
        .req_rq_pi_db            (req_rq_pi_db),
        .req_dest_addr           (req_dest_addr),
        .req_buf_addr_offset     (req_buf_addr_offset),
        .req_pkt_len             (req_pkt_len),
        .req_imm_data            (req_imm_data),
        .req_tnfr_vld            (req_tnfr_vld),
        .rdy_to_req_tnfr         (rdy_to_req_tnfr),
        //.i_qp_enable             (i_qp_enable), 
        .i_qp_enable_wr_en       (i_qp_enable_wr_en),
        .i_qp_enable_wr_idx      (i_qp_enable_wr_idx),
        .i_qp_enable_wr_data     (i_qp_enable_wr_data),

        .waiting_for_rq_update          (waiting_for_rq_update),
        .waiting_for_rq_update_on_qp_num(waiting_for_rq_update_on_qp_num),

        .init_start               (init_start),
        .init_count               (init_count),
        .qp_under_recovery_wr_en  (qp_under_recovery_wr_en),
        .qp_under_recovery_wr_idx (qp_under_recovery_wr_idx),
        .qp_under_recovery_wr_data(qp_under_recovery_wr_data)
    );

    lrdma_common_reg_pipe #(
        .C_DATA_WIDTH(C_QP_INDX_WIDTH + C_M_AXI_ADDR_WIDTH + C_REQ_BUF_ADDR_WIDTH + 16 + 89 + 1 + 4)
    )  //16 for req_rq_pi_db
        u_reg_pipe_1 (
        .clk    (core_clk),
        .reset  (core_rst),
        .din    ({req_qpid, req_pkt_id, req_pkt_type, req_pkt_ip_ver, req_rq_pi_db, req_dest_addr, req_buf_addr_offset, req_pkt_len, req_imm_data}),
        .vld_in (req_tnfr_vld),
        .rdy_out(rdy_to_req_tnfr),
        .dout   ({o_req_qpid, o_req_pkt_id, o_req_pkt_type, o_req_pkt_ip_ver, o_req_rq_pi_db, o_req_dest_addr, o_req_buf_addr_offset, o_req_pkt_len, o_req_imm_data}),
        .vld_out(o_req_tnfr_vld),
        .rdy_in (i_rdy_to_req_tnfr)
    );

    lrdma_common_reg_pipe #(
        .C_DATA_WIDTH(64)
    ) u_reg_pipe_err_data (
        .clk    (core_clk),
        .reset  (core_rst),
        .din    ({rsp_pkt_id, rsp_imm_data}),
        .vld_in (req_err_tnfr_vld),
        .rdy_out(rdy_to_req_err_tnfr),
        .dout   ({o_req_err_pkt_id, o_req_err_imm_data}),
        .vld_out(o_req_err_tnfr_vld),
        .rdy_in (i_rdy_to_req_err_tnfr)
    );

    lrdma_rx_req_axi_master #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_AWUSER_WIDTH(C_M_AXI_AWUSER_WIDTH),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .C_NUM_QP(C_NUM_QP),
        .C_REQ_BUF_ADDR_WIDTH(C_REQ_BUF_ADDR_WIDTH)
    ) u_rx_req_axi_master (
        .clk                   (core_clk),
        .reset                 (core_rst),
        .req_pkt_id            (o_req_pkt_id),
        .req_pkt_type          (o_req_pkt_type),
        .req_pkt_ip_ver        (o_req_pkt_ip_ver),
        .req_rq_pi_db          (o_req_rq_pi_db),
        .req_dest_addr         (o_req_dest_addr),
        .req_qpid              (o_req_qpid),
        .req_buf_addr_offset   (o_req_buf_addr_offset),
        .req_pkt_len           (o_req_pkt_len),
        .req_imm_data          (o_req_imm_data),
        .req_tnfr_vld          (o_req_tnfr_vld),
        .rdy_to_req_tnfr       (i_rdy_to_req_tnfr),
        .rsp_pkt_id            (o_req_err_pkt_id),
        .rsp_imm_data          (o_req_err_imm_data),
        .rsp_tnfr_vld          (o_req_err_tnfr_vld),
        .rdy_to_rsp_tnfr       (i_rdy_to_req_err_tnfr),
        .i_err_buf_ba          (i_req_err_pkt_buf_ba),
        .i_err_buf_sz          (i_req_err_buffer_sz),
        .i_err_num_bufs        (i_req_err_num_bufs),
        .o_err_buf_ca          (),
        .o_err_buf_wrptr       (o_req_err_buf_wrptr),
        .i_in_err_sts_q_ba     (i_in_errsts_q_ba),
        .i_in_err_sts_q_sz     (16'd8),
        .i_in_err_sts_q_depth  (i_in_errsts_q_sz),
        .o_in_err_sts_q_ca     (),
        .o_in_err_sts_q_wrptr  (o_in_errsts_q_wrptr),
        .incr_inv_pkt_cnt      (incr_inv_req_pkt_cnt),
        .o_mad_pkt_cnt         (o_inc_mad_cnt),
        .lrdma_en_pulse        (lrdma_en_pulse),
        .set_pkt_valdn_err_intr(set_req_pkt_valdn_err_intr),
        .set_mad_pkt_rcvd_intr (set_mad_pkt_rcvd_intr),
        .set_qp_fatal_intr     (set_fatal_err_intr),
        .req_pkt_buf_addra     (req_pkt_buf_addra),
        .req_pkt_buf_enb       (req_pkt_buf_enb),
        .req_pkt_buf_addrb     (req_pkt_buf_addrb),
        .req_pkt_buf_doutb     (req_pkt_buf_doutb),
        .m_axi_awid            (rx_req_axi_awid),
        .m_axi_awaddr          (rx_req_axi_awaddr),
        .m_axi_awuser          (rx_req_axi_awuser),
        .m_axi_awlen           (rx_req_axi_awlen),
        .m_axi_awsize          (rx_req_axi_awsize),
        .m_axi_awburst         (rx_req_axi_awburst),
        .m_axi_awcache         (rx_req_axi_awcache),
        .m_axi_awprot          (rx_req_axi_awprot),
        .m_axi_awvalid         (rx_req_axi_awvalid),
        .m_axi_awready         (rx_req_axi_awready),
        .m_axi_wdata           (rx_req_axi_wdata),
        .m_axi_wstrb           (rx_req_axi_wstrb),
        .m_axi_wlast           (rx_req_axi_wlast),
        .m_axi_wvalid          (rx_req_axi_wvalid),
        .m_axi_wready          (rx_req_axi_wready),
        .m_axi_awlock          (rx_req_axi_awlock),
        .m_axi_bid             (rx_req_axi_bid),
        .m_axi_bresp           (rx_req_axi_bresp),
        .m_axi_bvalid          (rx_req_axi_bvalid),
        .m_axi_bready          (rx_req_axi_bready),
        .m_axi_arid            (rx_req_axi_arid),
        .m_axi_araddr          (rx_req_axi_araddr),
        .m_axi_arlen           (rx_req_axi_arlen),
        .m_axi_arsize          (rx_req_axi_arsize),
        .m_axi_arburst         (rx_req_axi_arburst),
        .m_axi_arcache         (rx_req_axi_arcache),
        .m_axi_arprot          (rx_req_axi_arprot),
        .m_axi_arvalid         (rx_req_axi_arvalid),
        .m_axi_arready         (rx_req_axi_arready),
        .m_axi_rid             (rx_req_axi_rid),
        .m_axi_rdata           (rx_req_axi_rdata),
        .m_axi_rresp           (rx_req_axi_rresp),
        .m_axi_rlast           (rx_req_axi_rlast),
        .m_axi_rvalid          (rx_req_axi_rvalid),
        .m_axi_rready          (rx_req_axi_rready),
        .m_axi_arlock          (rx_req_axi_arlock),

        .free_run_cnt(free_run_cnt)
    );

    lrdma_rx_resp_hdr_validator #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),
        .C_NUM_QP(C_NUM_QP),
        .PKT_DESC_FIFO_DATA_WIDTH(PKT_DESC_FIFO_DATA_WIDTH),
        .C_HDR_BUF_ADDR_WIDTH(C_HDR_BUF_ADDR_WIDTH)
    ) u_rx_resp_hdr_validator (
        .clk                      (core_clk),
        .reset                    (core_rst),
        .rsp_pkt_desc_fifo_dout   (rsp_pkt_desc_fifo_dout),
        .rsp_pkt_desc_fifo_empty  (rsp_pkt_desc_fifo_empty),
        .rd_rsp_pkt_desc_fifo     (rd_rsp_pkt_desc_fifo),
        .rsp_hdr_buf_addrb        (rsp_hdr_buf_addrb),
        .rsp_hdr_buf_enb          (rsp_hdr_buf_enb),
        .rsp_hdr_buf_doutb        (rsp_hdr_buf_doutb),
        .i_mac_node_addr          (i_mac_node_addr),
        .i_ipv4_node_addr         (i_ipv4_node_addr),
        .i_ipv6_node_addr         (i_ipv6_node_addr),
        .i_num_qp_enabled         (i_num_qp_enabled),
        .o_acknack_pkt_cnt        (o_inc_acknak_cnt),
        .o_inc_nack_pkt_cnt       (o_inc_nack_pkt_cnt),
        .o_last_in_rsp_pkt_opcode (o_last_in_rsp_pkt_opcode),
        .o_last_in_rsp_pkt_qpid   (o_last_in_rsp_pkt_qpid),
        .o_last_in_rsp_pkt_psn    (o_last_in_rsp_pkt_psn),
        .o_qp_conf_idx            (o_qp_rsp_conf_idx),
        .i_qp_conf                (i_qp_rsp_conf),
        .qp_conf_vld              (qp_rsp_conf_vld),
        .qp_conf_en               (qp_rsp_conf_en),
        .i_stat_nak               (i_stat_nak),
        .stat_nak_en              (stat_nak_en),
        .stat_nak_we              (stat_nak_we),
        .o_stat_nak               (o_stat_nak),
        .i_stat_resp_psn          (i_stat_resp_psn),
        .stat_resp_psn_en         (stat_resp_psn_en),
        .stat_resp_psn_we         (stat_resp_psn_we),
        .o_stat_resp_psn          (o_stat_resp_psn),
        .i_sq_psn                 (i_sq_psn),
        .sq_psn_en                (sq_psn_en),
        .i_remote_qp_mac_addr     (i_qp_rsp_remote_qp_mac_addr),
        .remote_qp_mac_addr_en    (qp_rsp_remote_qp_mac_addr_en),
        .i_remote_qp_ip_addr      (i_qp_rsp_remote_qp_ip_addr),
        .remote_qp_ip_addr_en     (qp_rsp_remote_qp_ip_addr_en),
        .i_qp_timeout             (i_qp_timeout),
        //.i_qp_fatal              (qp_fatal),
        .i_qp_fatal_wr_en         (o_qp_fatal_wr_en),
        .i_qp_fatal_wr_idx        (o_qp_fatal_wr_idx),
        .i_qp_fatal_wr_data       (o_qp_fatal_wr_data),
        .o_qp_fatal_idx           (qp_rsp_fatal_idx),
        .set_qp_fatal             (set_qp_rsp_fatal),
        .req_qp_fatal_idx         (qp_req_fatal_idx),
        .set_req_qp_fatal         (set_qp_req_fatal),
        //.i_qp_retried            (i_qp_retried),
        .i_qp_retried_set_valid   (i_qp_retried_set_valid),
        .i_qp_retried_set_id      (i_qp_retried_set_id),
        .i_qp_retried_reset_valid (i_qp_retried_reset_valid),
        .i_qp_retried_reset_id    (i_qp_retried_reset_id),
        //.i_osq_empty             (i_osq_empty),
        .i_osq_empty_set_valid    (i_osq_empty_set_valid),
        .i_osq_empty_set_id       (i_osq_empty_set_id),
        .i_osq_empty_reset_valid  (i_osq_empty_reset_valid),
        .i_osq_empty_reset_id     (i_osq_empty_reset_id),
        .i_osq_nacked             (i_osq_nacked),
        .o_qp_acknack_idx         (o_qp_index_acknack),
        .o_acknack_vld            (o_acknack_valid),
        .o_acknack_psn            (o_acknacked_psn),
        .o_acknack_syndr          (o_acknacked_syndrome),
        .o_acknack_msn            (o_acknacked_msn),
        .o_qp_timeout_per_qp      (o_qp_timeout_per_qp),
        .rsp_pkt_id               (rsp_pkt_id),
        .rsp_imm_data             (rsp_imm_data),
        .req_tnfr_vld             (req_err_tnfr_vld),
        .rdy_to_req_tnfr          (rdy_to_req_err_tnfr),
        .qp_under_recovery_wr_en  (qp_under_recovery_wr_en),
        .qp_under_recovery_wr_idx (qp_under_recovery_wr_idx),
        .qp_under_recovery_wr_data(qp_under_recovery_wr_data),
        .init_start               (init_start),
        .init_count               (init_count)
    );

    lrdma_rx_intr_ctrl u_rx_intr_ctrl (
        .clk          (core_clk),
        .rst_n        (core_rst_n),
        .lrdma_enabled(lrdma_en_pulse),

        .pkt_valdn_err_intr_en (i_pkt_valdn_err_intr_en),
        .pkt_valdn_err_intr    (o_pkt_valdn_err_intr),
        .pkt_valdn_err_intr_sts(o_pkt_valdn_err_intr_sts),
        .clr_pkt_valdn_err_intr(i_clr_pkt_valdn_err_intr),

        .mad_pkt_rcvd_intr_en (i_mad_pkt_rcvd_intr_en),
        .mad_pkt_rcvd_intr    (o_mad_pkt_rcvd_intr),
        .mad_pkt_rcvd_intr_sts(o_mad_pkt_rcvd_intr_sts),
        .clr_mad_pkt_rcvd_intr(i_clr_mad_pkt_rcvd_intr),

        .fatal_err_intr_en (i_fatal_err_intr_en),
        .fatal_err_intr    (o_fatal_err_intr),
        .fatal_err_intr_sts(o_fatal_err_intr_sts),
        .clr_fatal_err_intr(i_clr_fatal_err_intr),

        .set_pkt_valdn_err_intr(set_pkt_valdn_err_intr),
        .set_mad_pkt_rcvd_intr (set_mad_pkt_rcvd_intr),
        .set_fatal_err_intr    (set_fatal_err_intr)
    );

endmodule
