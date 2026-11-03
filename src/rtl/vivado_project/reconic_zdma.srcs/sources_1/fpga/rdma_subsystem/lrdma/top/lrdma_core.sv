`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_core — LRDMA IP 用户级顶层封装模块
//
// 【模块概述】
// 本模块是 LRDMA 用户级顶层，将所有核心子系统
// 集成为一个完整的硬件 RDMA 引擎。支持最多 C_NUM_QP 个队列对（QP），内部 QP 索引从 0 开始
// 编号，但实际 QP 编号需加 1（即 qpid=0 对应 QP1，qpid=31 对应 QP32）。
//
// 【内部子系统架构】
// 1. RX 引擎（rx_engine）：
//    - 接收 CMAC 传入的 512 位 RoCEv2 AXI-Stream 数据流
//    - 执行入站包头验证（ETH/IP/UDP/BTH）、QP 分流、PSN 序号验证
//    - 检测重复/乱序包
//    - 统计各类计数器（发送包数、ACK 包数、MAD 包数、无效包数、重复包数等）
//    - 检测协议违规时设置 per-QP 的 fatal 标志
//
// 2. ACK 管理器（ack_mgr）：
//    - 管理未完成发送队列（OSQ），追踪每个已发出但尚未确认的工作请求
//    - 接收 RX 模块解析的 ACK/NAK 信息，执行 PSN 匹配判断 WQE 完成状态
//    - 处理 NAK/超时重传，通过 QP 管理器接口请求重传仲裁
//    - 管理重试计数器，耗尽时设置 per-QP fatal（可通过 LRDMA_ADV_CONF.retry_cnt_fatal_dis 禁用）
//    - 通过 AXI Master 将 CQE 写入 DDR 中的完成队列（CQ），产生 WQE 完成中断
//
// 3. QP 管理器（qp_mgr）：
//    - 配置中枢：通过 AXI-Lite 从接口（经 lrdma_common_xpm_fifo_axil_wrap 跨时钟域）接收 CPU 寄存器配置
//    - 管理所有 per-QP BRAM 配置参数（QP 配置、RQ/CQ/SQ 基地址、远端 MAC/IP、超时、PSN、MSN 等）
//    - 维护全局配置（LRDMA 使能、本地 MAC/IP、数据缓冲区参数等）
//    - WQE 仲裁：从 SQ 取出 WQE，并通过 FIFO 分发给 TX 引擎
//    - 管理中断使能/清除、重传请求接受、QP 使能/禁用脉冲、统计计数器回读
//
// 4. TX 引擎（tx_engine）：
//    - 从 QP 管理器的 WQE FIFO 取出工作队列条目
//    - 读取 per-QP 配置（目标 QPID、MAC/IP、PSN、PMTU 等）
//    - 生成完整 RDMA WRITE/MAD 发送数据包头（ETH+IP+UDP+BTH+可选 RETH）
//    - 通过 AXI Master 从 DDR 读取 RDMA WRITE 有效载荷数据
//    - 维护 PSN/SSN 递增更新，管理 SGL 缓冲区和写重试数据缓冲区
//    - 发送完成后向 OSQ push 条目，供 ACK 管理器追踪
//
// 5. 流控管理器（flow_ctrl_mgr）：
//    - 将 TX 引擎的 RoCE TX 流量和外部 DMA 的非 RoCE 流量仲裁合并到 CMAC TX 通道
//    - 实现 IEEE 802.1Qbb PFC 优先级流控，按暂停请求控制发送速率
//
// 6. 寄存器切片（lrdma_common_reg_slice）：
//    - 在 CMAC TX 时钟域对输出数据路径做一级流水线寄存器切片，改善时序
//
// 【辅助逻辑】
// - LUT RAM 初始化 FSM：硬复位后遍历所有 QP 索引，广播 init_start/init_count 清零各子模块 LUT RAM
// - TX 数据包间隔延迟 FSM：每个输出包的 tlast 后插入可配置周期数延迟控制发送速率
// - 错误注入逻辑：可对指定数据包首 beat 的 MSB 做位翻转用于测试
// - 中断聚合逻辑：将 4 个中断源 OR 合并为单一 lrdma_intr 输出
// - PFC sideband：stat_rx_pause_req 已由 packet_adapter 同步到 cmac_clk 域
//
// 【支持特性】
// - IPv4/IPv6 双协议栈
// - 可选 SGL 深度、写重试数据缓冲区
//
// 【时钟域】
// - m_axi_aclk：核心/AXI Master 时钟域（LRDMA 内部主时钟）
// - cmac_clk：CMAC 时钟域
// - s_axi_lite_aclk：AXI-Lite 配置时钟域
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module lrdma_core #(
    parameter C_NUM_QP                   = 32,
    parameter C_M_AXI_ADDR_WIDTH         = 64,
    parameter C_M_AXI_AWUSER_WIDTH       = 32,
    parameter C_M_AXI_ID_WIDTH           = 1,
    parameter C_S_AXI_LITE_ADDR_WIDTH    = 32,
    parameter C_MAX_SGL_DEPTH            = 128,
    parameter C_OSQ_PSN_WIDTH            = 16,
    // LRDMA 低延迟总开关:
    // 1 = optimized mode，启用面向轻量化单向数据采集的发送侧低延迟组合优化：
    //     - QP manager 在 SQ 读地址有效时立即发起 WQE AXI AR 请求，减少 1 拍等待；
    //     - 普通 WQE 返回且内部 FIFO 为空时，旁路 WQE FIFO 直接送 TX engine；
    //     - 同一 SQ 连续 WQE 复用已缓存上下文，降低多 WQE batch 启动间隔；
    //     - RoCE/LRDMA TX 包在 flow_ctrl_mgr 中绕过 legacy CMAC packet FIFO；
    //     - TX buffer manager 写完 header 高拍和 payload SGL 后同拍释放 ready；
    //     - 对确认来自 WQE bypass 寄存器的 WQE，hdr_gen 提前 1 拍解析。
    //     优化只压缩发送侧空转拍数，不改变 WQE 地址/长度/顺序、PSN/MSN 更新、
    //     ACK/NAK/timeout 重传语义；QDMA non-RoCE 仍保留原 FIFO 路径，避免破坏旁路流量。
    // 0 = baseline/legacy mode，关闭上述优化点，保持原始 RTL 时序路径，便于
    //     与优化模式做单 WQE、多 WQE、重传和 QDMA non-RoCE 对比实验。
    parameter C_ENABLE_LRDMA_LOW_LATENCY = 1
) (
    input m_axi_aclk,
    input m_axi_aresetn,
    input [7:0] m_axi_aresetn_shards,

    input s_axi_lite_aclk,
    input s_axi_lite_aresetn,

    input cmac_clk,
    input cmac_rstn,

    // RX 引擎使用该接口将传入的 MAD、错误状态和 RQ 门铃更新写入 DDR。
    // 只用到了写通道，没用到读通道和响应通道
    output wire [    C_M_AXI_ID_WIDTH-1:0] m_axi_rdma_send_write_payload_store_awid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_rdma_send_write_payload_store_awaddr,
    output wire [C_M_AXI_AWUSER_WIDTH-1:0] m_axi_rdma_send_write_payload_store_awuser,
    output wire [                     7:0] m_axi_rdma_send_write_payload_store_awlen,
    output wire [                     2:0] m_axi_rdma_send_write_payload_store_awsize,
    output wire [                     1:0] m_axi_rdma_send_write_payload_store_awburst,
    output wire [                     3:0] m_axi_rdma_send_write_payload_store_awcache,
    output wire [                     2:0] m_axi_rdma_send_write_payload_store_awprot,
    output wire                            m_axi_rdma_send_write_payload_store_awvalid,
    input  wire                            m_axi_rdma_send_write_payload_store_awready,
    output wire [                   511:0] m_axi_rdma_send_write_payload_store_wdata,
    output wire [                    63:0] m_axi_rdma_send_write_payload_store_wstrb,
    output wire                            m_axi_rdma_send_write_payload_store_wlast,
    output wire                            m_axi_rdma_send_write_payload_store_wvalid,
    input  wire                            m_axi_rdma_send_write_payload_store_wready,
    output wire                            m_axi_rdma_send_write_payload_store_awlock,
    input  wire [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_rdma_send_write_payload_store_bid,
    input  wire [                     1:0] m_axi_rdma_send_write_payload_store_bresp,
    input  wire                            m_axi_rdma_send_write_payload_store_bvalid,
    output wire                            m_axi_rdma_send_write_payload_store_bready,
    output wire [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_rdma_send_write_payload_store_arid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_rdma_send_write_payload_store_araddr,
    output wire [                     7:0] m_axi_rdma_send_write_payload_store_arlen,
    output wire [                     2:0] m_axi_rdma_send_write_payload_store_arsize,
    output wire [                     1:0] m_axi_rdma_send_write_payload_store_arburst,
    output wire [                     3:0] m_axi_rdma_send_write_payload_store_arcache,
    output wire [                     2:0] m_axi_rdma_send_write_payload_store_arprot,
    output wire                            m_axi_rdma_send_write_payload_store_arvalid,
    input  wire                            m_axi_rdma_send_write_payload_store_arready,
    input  wire [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_rdma_send_write_payload_store_rid,
    input  wire [                   511:0] m_axi_rdma_send_write_payload_store_rdata,
    input  wire [                     1:0] m_axi_rdma_send_write_payload_store_rresp,
    input  wire                            m_axi_rdma_send_write_payload_store_rlast,
    input  wire                            m_axi_rdma_send_write_payload_store_rvalid,
    output wire                            m_axi_rdma_send_write_payload_store_rready,
    output wire                            m_axi_rdma_send_write_payload_store_arlock,

    // 该接口将网络接口的 RoCE 数据包提供给LRDMA。
    input wire         roce_cmac_s_axis_tvalid,
    input wire [511:0] roce_cmac_s_axis_tdata,
    input wire [ 63:0] roce_cmac_s_axis_tkeep,
    input wire         roce_cmac_s_axis_tlast,
    input wire         roce_cmac_s_axis_tuser,

    // 这是一条从 QDMA 模块到流控管理器的传入非 RDMA 路径，即 H2C 路径
    input  wire         non_roce_qdma_s_axis_tvalid,
    input  wire [511:0] non_roce_qdma_s_axis_tdata,
    input  wire [ 63:0] non_roce_qdma_s_axis_tkeep,
    input  wire         non_roce_qdma_s_axis_tlast,
    output wire         non_roce_qdma_s_axis_tready,

    // TX 引擎 AXI Master 接口
    // 此接口被 TX Engine 用于从 DDR 中读取待发送 RDMA WRITE 的 payload。
    // 只用到了读通道，没用到写通道和响应通道
    output wire [  C_M_AXI_ID_WIDTH-1:0] m_axi_pktgen_get_payload_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_pktgen_get_payload_awaddr,
    output wire [                   7:0] m_axi_pktgen_get_payload_awlen,
    output wire [                   2:0] m_axi_pktgen_get_payload_awsize,
    output wire [                   1:0] m_axi_pktgen_get_payload_awburst,
    output wire [                   3:0] m_axi_pktgen_get_payload_awcache,
    output wire [                   2:0] m_axi_pktgen_get_payload_awprot,
    output wire                          m_axi_pktgen_get_payload_awvalid,
    input  wire                          m_axi_pktgen_get_payload_awready,
    output wire [                 511:0] m_axi_pktgen_get_payload_wdata,
    output wire [                  63:0] m_axi_pktgen_get_payload_wstrb,
    output wire                          m_axi_pktgen_get_payload_wlast,
    output wire                          m_axi_pktgen_get_payload_wvalid,
    input  wire                          m_axi_pktgen_get_payload_wready,
    output wire                          m_axi_pktgen_get_payload_awlock,
    input  wire [C_M_AXI_ID_WIDTH-1 : 0] m_axi_pktgen_get_payload_bid,
    input  wire [                   1:0] m_axi_pktgen_get_payload_bresp,
    input  wire                          m_axi_pktgen_get_payload_bvalid,
    output wire                          m_axi_pktgen_get_payload_bready,
    output wire [C_M_AXI_ID_WIDTH-1 : 0] m_axi_pktgen_get_payload_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_pktgen_get_payload_araddr,
    output wire [                   7:0] m_axi_pktgen_get_payload_arlen,
    output wire [                   2:0] m_axi_pktgen_get_payload_arsize,
    output wire [                   1:0] m_axi_pktgen_get_payload_arburst,
    output wire [                   3:0] m_axi_pktgen_get_payload_arcache,
    output wire [                   2:0] m_axi_pktgen_get_payload_arprot,
    output wire                          m_axi_pktgen_get_payload_arvalid,
    input  wire                          m_axi_pktgen_get_payload_arready,
    input  wire [C_M_AXI_ID_WIDTH-1 : 0] m_axi_pktgen_get_payload_rid,
    input  wire [                 511:0] m_axi_pktgen_get_payload_rdata,
    input  wire [                   1:0] m_axi_pktgen_get_payload_rresp,
    input  wire                          m_axi_pktgen_get_payload_rlast,
    input  wire                          m_axi_pktgen_get_payload_rvalid,
    output wire                          m_axi_pktgen_get_payload_rready,
    output wire                          m_axi_pktgen_get_payload_arlock,

    // AXI streaming interface signals to and from tx engine CRC process engine
    // 往返于 tx engine CRC process engine 的 AXI 流媒体接口信号
    output wire [511 : 0] cmac_m_axis_tdata,
    output wire [   63:0] cmac_m_axis_tkeep,
    output wire           cmac_m_axis_tvalid,
    input  wire           cmac_m_axis_tready,
    output wire           cmac_m_axis_tlast,

    // 来自 ACK 管理器的 AXI 主站信号
    // 此接口由 ACK 管理器用于将完成信息（CQE）写入 DDR 中的 CQ
    // Write address channel
    output wire [                   0:0] m_axi_write_completion_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_write_completion_awaddr,
    output wire [                   7:0] m_axi_write_completion_awlen,
    output wire [                   2:0] m_axi_write_completion_awsize,
    output wire [                   1:0] m_axi_write_completion_awburst,
    output wire [                   3:0] m_axi_write_completion_awcache,
    output wire [                   2:0] m_axi_write_completion_awprot,
    output wire                          m_axi_write_completion_awvalid,
    input  wire                          m_axi_write_completion_awready,
    // Write data channel
    output wire [                 511:0] m_axi_write_completion_wdata,
    output wire [             512/8-1:0] m_axi_write_completion_wstrb,
    output wire                          m_axi_write_completion_wlast,
    output wire                          m_axi_write_completion_wvalid,
    input  wire                          m_axi_write_completion_wready,
    output wire                          m_axi_write_completion_awlock,
    // Write Response channel
    input  wire [                 0 : 0] m_axi_write_completion_bid,
    input  wire [                   1:0] m_axi_write_completion_bresp,
    input  wire                          m_axi_write_completion_bvalid,
    output wire                          m_axi_write_completion_bready,
    // Read address channel
    output wire [                   0:0] m_axi_write_completion_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_write_completion_araddr,
    output wire [                   7:0] m_axi_write_completion_arlen,
    output wire [                   2:0] m_axi_write_completion_arsize,
    output wire [                   1:0] m_axi_write_completion_arburst,
    output wire [                   3:0] m_axi_write_completion_arcache,
    output wire [                   2:0] m_axi_write_completion_arprot,
    output wire                          m_axi_write_completion_arvalid,
    input  wire                          m_axi_write_completion_arready,
    // Read data/response channel
    input  wire [                   0:0] m_axi_write_completion_rid,
    input  wire [                 511:0] m_axi_write_completion_rdata,
    input  wire [                   1:0] m_axi_write_completion_rresp,
    input  wire                          m_axi_write_completion_rlast,
    input  wire                          m_axi_write_completion_rvalid,
    output wire                          m_axi_write_completion_rready,
    output wire                          m_axi_write_completion_arlock,

    // AXI lite interface
    // Write address channel
    input  wire [C_S_AXI_LITE_ADDR_WIDTH-1:0] s_axi_lite_awaddr,
    output wire                               s_axi_lite_awready,
    input  wire                               s_axi_lite_awvalid,
    // Read address channel
    input  wire [C_S_AXI_LITE_ADDR_WIDTH-1:0] s_axi_lite_araddr,
    output wire                               s_axi_lite_arready,
    input  wire                               s_axi_lite_arvalid,
    // Write data channel
    input  wire [                       31:0] s_axi_lite_wdata,
    input  wire [                        3:0] s_axi_lite_wstrb,
    output wire                               s_axi_lite_wready,
    input  wire                               s_axi_lite_wvalid,
    // Read data channel
    output wire [                       31:0] s_axi_lite_rdata,
    output wire [                        1:0] s_axi_lite_rresp,
    input  wire                               s_axi_lite_rready,
    output wire                               s_axi_lite_rvalid,
    // Write Response channel
    output wire [                        1:0] s_axi_lite_bresp,
    input  wire                               s_axi_lite_bready,
    output wire                               s_axi_lite_bvalid,

    // AXI Master signals
    // 只用到了读通道
    // Write address channel
    output wire [                   0:0] m_axi_qp_get_wqe_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_qp_get_wqe_awaddr,
    output wire [                   7:0] m_axi_qp_get_wqe_awlen,
    output wire [                   2:0] m_axi_qp_get_wqe_awsize,
    output wire [                   1:0] m_axi_qp_get_wqe_awburst,
    output wire [                   3:0] m_axi_qp_get_wqe_awcache,
    output wire [                   2:0] m_axi_qp_get_wqe_awprot,
    output wire                          m_axi_qp_get_wqe_awvalid,
    input  wire                          m_axi_qp_get_wqe_awready,
    // Write data channel
    output wire [                 511:0] m_axi_qp_get_wqe_wdata,
    output wire [             512/8-1:0] m_axi_qp_get_wqe_wstrb,
    output wire                          m_axi_qp_get_wqe_wlast,
    output wire                          m_axi_qp_get_wqe_wvalid,
    input  wire                          m_axi_qp_get_wqe_wready,
    output wire                          m_axi_qp_get_wqe_awlock,
    // Write Response channel
    input  wire [                 0 : 0] m_axi_qp_get_wqe_bid,
    input  wire [                   1:0] m_axi_qp_get_wqe_bresp,
    input  wire                          m_axi_qp_get_wqe_bvalid,
    output wire                          m_axi_qp_get_wqe_bready,
    // Read address channel
    output wire [                   0:0] m_axi_qp_get_wqe_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_qp_get_wqe_araddr,
    output wire [                   7:0] m_axi_qp_get_wqe_arlen,
    output wire [                   2:0] m_axi_qp_get_wqe_arsize,
    output wire [                   1:0] m_axi_qp_get_wqe_arburst,
    output wire [                   3:0] m_axi_qp_get_wqe_arcache,
    output wire [                   2:0] m_axi_qp_get_wqe_arprot,
    output wire                          m_axi_qp_get_wqe_arvalid,
    input  wire                          m_axi_qp_get_wqe_arready,
    // Read data/response channel
    input  wire [                   0:0] m_axi_qp_get_wqe_rid,
    input  wire [                 511:0] m_axi_qp_get_wqe_rdata,
    input  wire [                   1:0] m_axi_qp_get_wqe_rresp,
    input  wire                          m_axi_qp_get_wqe_rlast,
    input  wire                          m_axi_qp_get_wqe_rvalid,
    output wire                          m_axi_qp_get_wqe_rready,
    output wire                          m_axi_qp_get_wqe_arlock,

    // ---------------------------------------------------------------------------------------------------------------------------------------------------

    output reg lrdma_intr,  // RNIC 中断

    // Prioroty flow control
    // 优先流量控制
    input  wire [8:0] stat_rx_pause_req,  // CMAC IP核已接收到暂停包
    output wire [8:0] ctl_tx_pause_req,

    output wire compat_resetn_done,
    output wire [63:0] compat_credit_awaddr,
    output wire compat_credit_awvalid,
    input wire compat_credit_awready,
    output wire [511:0] compat_credit_wdata,
    output wire compat_credit_wvalid,
    input wire compat_credit_wready,
    input wire [1:0] compat_credit_bresp,
    input wire compat_credit_bvalid,
    output wire compat_credit_bready
);
    reg compat_in_frame,compat_credit_frame;
    wire compat_credit_first=roce_cmac_s_axis_tdata[42*8+:8]==8'h0a;
    wire native_rx_valid=roce_cmac_s_axis_tvalid &&
      !(compat_in_frame ? compat_credit_frame : compat_credit_first);
    always @(posedge m_axi_aclk) begin
      if(!m_axi_aresetn)begin compat_in_frame<=0;compat_credit_frame<=0;end
      else if(roce_cmac_s_axis_tvalid)begin
        compat_in_frame<=!roce_cmac_s_axis_tlast;
        if(!compat_in_frame)compat_credit_frame<=compat_credit_first;
      end
    end

wire [                   0:0] cq_core_awid;
wire [C_M_AXI_ADDR_WIDTH-1:0] cq_core_awaddr;
wire [                   7:0] cq_core_awlen;
wire [                   2:0] cq_core_awsize;
wire [                   1:0] cq_core_awburst;
wire [                   3:0] cq_core_awcache;
wire [                   2:0] cq_core_awprot;
wire cq_core_awvalid;
wire cq_core_awready;
wire [                 511:0] cq_core_wdata;
wire [             512/8-1:0] cq_core_wstrb;
wire cq_core_wlast;
wire cq_core_wvalid;
wire cq_core_wready;
wire cq_core_awlock;
wire [                 0 : 0] cq_core_bid;
wire [                   1:0] cq_core_bresp;
wire cq_core_bvalid;
wire cq_core_bready;
wire [511 : 0] primary_tdata;
wire [   63:0] primary_tkeep;
wire primary_tvalid;
wire primary_tlast;
wire primary_tready;

    `include "lrdma_defs.vh"


    localparam C_QP_INDX_WIDTH = clog2(C_NUM_QP);  // C_NUM_QP = 32, C_QP_INDX_WIDTH = 5
    localparam OS_Q_DEPTH = 16;
    localparam OS_Q_INDX_WIDTH = 4;
    localparam C_ENABLE_FAST_WQE_AXI_EN = (C_ENABLE_LRDMA_LOW_LATENCY != 0);
    localparam C_ENABLE_WQE_BYPASS = (C_ENABLE_LRDMA_LOW_LATENCY != 0);
    localparam C_ENABLE_SAME_SQ_FAST_FETCH = (C_ENABLE_LRDMA_LOW_LATENCY != 0);
    localparam C_ENABLE_LRDMA_TX_LOW_LATENCY_BYPASS = (C_ENABLE_LRDMA_LOW_LATENCY != 0);
    localparam C_ENABLE_TX_BUF_MGR_EARLY_RELEASE = (C_ENABLE_LRDMA_LOW_LATENCY != 0);
    localparam C_ENABLE_FAST_WQE_CONSUME = (C_ENABLE_LRDMA_LOW_LATENCY != 0);
    ////////////////////////////////- Constants ////////////////////////////////////

    wire                                 qp_mgr_o_lrdma_en;  // LRDMA Configuration -- [0] LRDMA Enable
    wire                                 qp_mgr_o_lrdma_adv_conf_retry_cnt_fatal_dis;
    wire                                 qp_mgr_o_err_buf_en;  // Enable validation error packets to go to error buffer
    wire [C_QP_INDX_WIDTH:0] qp_mgr_o_num_qp_en;  // Number of QPs enabled
    wire [                         47:0] qp_mgr_o_mac_src_addr;  // LRDMA 寄存器中的本地/源以太网 MAC 地址
    wire [                         31:0] qp_mgr_o_ipv4_src_addr;  // Local/source IPv4 address from LRDMA registers
    wire [                        127:0] qp_mgr_o_ipv6_src_addr;  // Local/source IPv6 address from LRDMA registers

    wire                                 rx_engine_o_ib_pkt_hdr_valdn_err_intr;  // Inbound packet header validation error interrupt
    wire                                 rx_engine_o_mad_pkt_rcvd_intr;  // MAD packet received interrupt

    wire                                 qp_mgr_o_intr_clr_pkt_valdn_err;
    wire                                 qp_mgr_o_intr_clr_mad_pkt_rcvd;
    wire [          C_QP_INDX_WIDTH-1:0] rx_engine_o_rq_full_idx;
    wire                                 rx_engine_o_rq_full_wr_en;
    wire                                 rx_engine_o_rq_full_val;
    wire [          C_QP_INDX_WIDTH-1:0] rx_engine_o_rq_full_reset_idx;
    wire                                 rx_engine_o_rq_full_reset_wr_en;
    wire                                 rx_engine_o_rq_full_reset_val;
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_err_pkt_buf_ba;
    wire [                         15:0] qp_mgr_o_err_pkt_buf_sz_num_bufs;
    wire [                         15:0] qp_mgr_o_err_pkt_buf_sz_buf_sz;
    wire [                         15:0] rx_engine_o_err_pkt_buf_wrptr;
    wire [                         15:0] rx_engine_o_ib_ack_pkt_cnt;  // 传入 ACK/NAK 响应包计数
    wire [                         15:0] rx_engine_o_ib_qp1_pkt_cnt;  // 传入 QP1/MAD 包计数
    wire [                         15:0] rx_engine_o_ib_inv_pkt_cnt;  // 传入无效包计数
    wire [                          7:0] rx_engine_o_last_ib_pkt_opcode;  // Last incoming packet OPCODE
    wire [          C_QP_INDX_WIDTH-1:0] rx_engine_o_last_ib_pkt_dest_qpid;  // Last incoming packet DEST_QP
    wire [                         15:0] rx_engine_o_last_ib_pkt_psn;  // Last incoming packet PSN  
    wire [                         15:0] qp_mgr_o_lrdma_udp_src_port;

    wire                                 qp_mgr_o_qp_conf_valid_tx_engine;
    wire                                 qp_mgr_o_qp_conf_valid_rx_engine;
    wire [                         31:0] qp_mgr_o_qp_conf;  // QP configuration
    wire [          C_QP_INDX_WIDTH-1:0] qp_mgr_qp_conf_idx;
    wire                                 tx_engine_qp_info_ready;

    wire                                 rx_engine_o_qpm_req_rq_buf_ba;
    wire                                 qp_mgr_o_qp_rq_ba_valid;
    wire [       C_M_AXI_ADDR_WIDTH-9:0] qp_mgr_o_qp_rq_ba;

    wire                                 ack_mgr_o_qp_cq_ba_req;
    wire                                 qp_mgr_o_qp_cq_ba_valid;
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_qp_cq_ba;

    wire                                 rx_engine_o_qpm_req_rq_wrptr_db_add;
    wire                                 qp_mgr_o_qp_rq_wrptrdb_add_valid;
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_qp_rq_wrptrdb_add;

    wire                                 ack_mgr_o_qp_cq_hdptr_req;
    wire                                 qp_mgr_o_qp_cq_hdptr_valid;
    wire [                         15:0] qp_mgr_o_qp_cq_hdptr;
    wire [                         15:0] ack_mgr_o_qp_cq_hdptr;
    wire                                 ack_mgr_o_qp_cq_hdptr_wrn;
    wire [       C_QP_INDX_WIDTH -1 : 0] ack_mgr_o_qp_cqe_proc_qpid;

    wire                                 rx_engine_o_qpm_req_rq_ci_db;
    wire                                 qp_mgr_o_qp_rq_cidb_valid;
    wire [                         31:0] qp_mgr_o_qp_rq_cidb;

    wire                                 rx_engine_o_qpm_req_q_depth;
    wire                                 qp_mgr_o_qp_rq_depth_valid;
    wire [                         15:0] qp_mgr_o_qp_rq_depth;

    wire                                 rx_engine_o_qpm_req_last_req;
    wire                                 rx_engine_o_qpm_rw_last_req;
    wire [                         31:0] rx_engine_o_last_req;
    wire [                         31:0] qp_mgr_o_qp_last_req;
    wire                                 qp_mgr_o_qp_last_req_valid;


    wire                                 qp_mgr_o_intr_clr_wqe_cmpl;
    wire                                 qp_mgr_o_intr_clr_fatal_err;
    wire                                 qp_mgr_o_intr_en_wqe_cmpl;
    wire                                 ack_mgr_o_intr_wqe_cmpl;

    wire                                 rx_engine_o_qpm_req_dest_qp_conf;
    wire                                 qp_mgr_o_qp_dest_qpid_valid_tx_engine;
    wire                                 qp_mgr_o_qp_dest_qpid_valid_rx_engine;
    wire [                         23:0] qp_mgr_o_qp_dest_qpid;

    wire [                         31:0] qp_mgr_o_qp_timeout_per_qp;

    wire                                 qpm_wqe_fifo_empty;
    wire                                 qpm_wqe_pop;
    wire [                        511:0] qpm_wqe_data;  //64B of WQE 
    wire                                 qpm_wqe_fast_valid;


    wire                                 qp_mgr_o_qp_mac_remote_addrl_valid;

    wire                                 qp_mgr_o_qp_ip_remote_addr1_valid;

    wire                                 qp_mgr_o_qp_mac_dest_addrl_valid;
    wire [                         31:0] qp_mgr_o_qp_mac_dest_addrl;
    wire                                 qp_mgr_o_qp_mac_dest_addrm_valid;
    wire [                         31:0] qp_mgr_o_qp_mac_dest_addrm;
    wire                                 qp_mgr_o_qp_ip_dest_addr1_valid;
    wire [                         31:0] qp_mgr_o_qp_ip_dest_addr1;
    wire                                 qp_mgr_o_qp_ip_dest_addr2_valid;
    wire [                         31:0] qp_mgr_o_qp_ip_dest_addr2;
    wire                                 qp_mgr_o_qp_ip_dest_addr3_valid;
    wire [                         31:0] qp_mgr_o_qp_ip_dest_addr3;
    wire                                 qp_mgr_o_qp_ip_dest_addr4_valid;
    wire [                         31:0] qp_mgr_o_qp_ip_dest_addr4;
    wire                                 tx_engine_o_reg_rd_en;
    wire                                 tx_engine_o_reg_rd_en_s;
    wire                                 tx_engine_o_reg_wr_en;

    wire                                 qp_mgr_o_qp_adv_conf_valid;
    wire [                         31:0] qp_mgr_o_qp_adv_conf;  // QP advance configuration
    wire                                 qp_mgr_o_qp_sq_psn_valid;
    wire                                 qp_mgr_o_qp_sq_psn_wqe_valid;
    wire                                 qp_mgr_o_qp_stat_ssn_valid;
    wire [                         23:0] qp_mgr_o_qp_sq_psn;
    wire [                         23:0] tx_engine_reg_sq_psn;
    wire [                         23:0] qp_mgr_o_qp_sq_ssn;
    wire [                         23:0] tx_engine_reg_sq_ssn;
    wire                                 rx_engine_o_qpm_req_sq_psn;

    wire [          C_QP_INDX_WIDTH-1:0] tx_engine_os_wqe_qpid;  // QP ID of WQE
    wire [                          7:0] tx_engine_os_wqe_opcode;  // BTH.Opcode of outbound request
    wire [                          4:0] tx_engine_o_wqe_qp_to;  // BTH.Opcode of outbound request
    wire [                         15:0] tx_engine_os_wqe_wrid;  // Work request ID of outbound request
    wire                                 tx_engine_os_wqe_retried;  // Retried work request
    wire                                 tx_engine_os_wqe_fatal_push;  // Retried work request
    wire [                         23:0] tx_engine_os_wqe_send_psn;  // PSN of most recent request
    wire [                         23:0] tx_engine_os_wqe_send_first_psn;  // PSN of most recent request
    wire [                         23:0] tx_engine_os_wqe_send_msn;  // PSN of most recent request
    wire                                 tx_engine_os_wqe_explicit_ack_req;  // Explicit acknowledgement requirement on outbound requests
    wire                                 tx_engine_os_wqp_req;  // request from TX 引擎 to push WQE
    wire                                 tx_engine_os_wqp_rdy;  // response to TX 引擎; 0 - busy; 1 - completed
    wire [                         31:0] ack_mgr_o_ack_mgr_sts;
    wire [                         31:0] ack_mgr_o_stat_retry_count;

    wire [         C_QP_INDX_WIDTH -1:0] tx_engine_o_qp_id;  // index to be replaced during tx engine integration 
    wire                                 ack_mgr_o_qp_cq_depth_req;
    wire                                 qp_mgr_o_qp_cq_depth_valid;
    wire [                         15:0] qp_mgr_o_qp_cq_depth;
    wire [         C_QP_INDX_WIDTH -1:0] ack_mgr_o_qp_conf_qp_idx;
    wire [         C_QP_INDX_WIDTH -1:0] ack_mgr_o_qp_sq_psn_resp_idx;
    wire [(4*C_QP_INDX_WIDTH+4) - 1 : 0] o_osq_nacked;

    wire [       C_QP_INDX_WIDTH -1 : 0] ack_mgr_o_retransmit_qpid;  // 重传路径选中的 QP id
    wire [                       55 : 0] ack_mgr_o_retransmit_wqe_bram_din;
    wire                                 ack_mgr_o_retransmit_reqd;  // Retransmission is required
    wire                                 qp_mgr_o_retransmit_accepted;
    wire [                         23:0] ack_mgr_o_psn_to_retry;
    wire [                         23:0] ack_mgr_o_ssn_to_retry;

    wire                                 rx_engine_o_qpm_req_rq_buf_ca;
    wire                                 rx_engine_o_qpm_rw_rq_buf_ca;
    wire [       C_M_AXI_ADDR_WIDTH-9:0] qp_mgr_o_qp_stat_rq_buf_ca;
    wire                                 qp_mgr_o_qp_stat_rq_buf_ca_valid;
    wire [       C_M_AXI_ADDR_WIDTH-9:0] rx_engine_o_rq_buf_ca;

    wire [                         15:0] ack_mgr_o_inc_nack_cnt;

    wire [          C_QP_INDX_WIDTH-1:0] rx_engine_o_qp_index_acknack;
    wire                                 rx_engine_o_acknack_valid;
    wire [                          7:0] rx_engine_o_acknacked_syndrome;
    wire [                         23:0] rx_engine_o_acknacked_msn;
    wire [                         23:0] rx_engine_o_acknacked_psn;

    wire                                 qp_mgr_o_intr_en_pkt_valdn_err;
    wire                                 qp_mgr_o_intr_en_mad_pkt_rcvd;

    wire                                 rx_engine_o_qpm_req_stat_nak;
    wire                                 rx_engine_o_qpm_rw_stat_nak;
    wire [                          6:0] rx_engine_o_stat_nak;
    wire                                 ack_mgr_o_qp_stat_nak_valid;
    wire [                          6:0] ack_mgr_o_qp_stat_nak;

    wire [                          2:0] ack_mgr_o_qp_retry_cnt_new;
    wire [                          2:0] ack_mgr_i_qp_retry_cnt_curr;
    wire [          C_QP_INDX_WIDTH-1:0] ack_mgr_o_qp_retry_cnt_qpid;
    wire                                 ack_mgr_o_qp_retry_cnt_req;
    wire                                 ack_mgr_o_qp_retry_cnt_wen;

    wire                                 ack_mgr_o_qp_retry_fatal_req;
    wire [          C_QP_INDX_WIDTH-1:0] ack_mgr_o_qp_retry_fatal_qpid;
    wire                                 ack_mgr_i_qp_retry_fatal_rdy;

    wire                                 qp_mgr_o_halt;
    wire [       C_QP_INDX_WIDTH -1 : 0] qp_mgr_o_halted_qpid;
    wire                                 tx_engine_i_wqe_halted;

    wire                                 rx_engine_o_qpm_req_stat_resp_psn;
    wire                                 rx_engine_o_qpm_rw_stat_resp_psn;
    wire [                         23:0] qp_mgr_o_qp_stat_resp_psn;
    wire [                         23:0] rx_engine_o_qp_stat_resp_psn;
    wire                                 qp_mgr_o_qp_stat_resp_psn_valid;
    wire                                 rx_engine_o_qp_fatal_wr_en;
    wire [          C_QP_INDX_WIDTH-1:0] rx_engine_o_qp_fatal_wr_idx;
    wire                                 rx_engine_o_qp_fatal_wr_data;
    wire                                 clr_qp_fatal_err;
    wire [          C_QP_INDX_WIDTH-1:0] clr_qp_fatal_idx;
    wire                                 qp_under_recovery_wr_en;
    wire [          C_QP_INDX_WIDTH-1:0] qp_under_recovery_wr_idx;
    wire                                 qp_under_recovery_wr_data;

    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_in_errsts_q_ba;
    wire [                         15:0] qp_mgr_in_errsts_q_sz;
    wire [                         15:0] rx_engine_in_errsts_q_wrptr;


    wire [                         15:0] wqe_out_wr_pkt_cnt;
    wire [                         15:0] wqe_out_mad_pkt_cnt;

    wire                                 rx_engine_o_fatal_err_intr;
    wire                                 qp_mgr_o_intr_en_fatal_err;

    wire                                 ack_mgr_o_qp_conf_req;
    wire                                 qp_mgr_o_qp_conf_valid_ack_mgr;

    wire [                         31:0] tx_engine_status_out;
    wire [                         31:0] last_out_pkt_info;

    wire                                 intr_wqe_cmpl_sts_or_reduce;
    wire [                       31 : 0] intr_wqe_cmpl_sts_wr_en;
    wire [                       31 : 0] intr_wqe_cmpl_sts_wr_data;
    wire [        C_QP_INDX_WIDTH-1 : 0] intr_wqe_cmpl_sts_wr_addr;
    wire [                       31 : 0] intr_wqe_cmpl_sts_wr_en_pending;
    wire [        C_QP_INDX_WIDTH-1 : 0] intr_wqe_cmpl_sts_wr_addr_pending;
    wire [                          1:0] qp_mgr_o_flow_credits;

    wire [                         31:0] rx_engine_o_pkt_all_cnt;
    wire [                         31:0] rx_engine_o_pkt_disc_cnt;

    wire [                         15:0] rx_engine_o_inc_nack_cnt;

    wire                                 rx_engine_o_qpm_req_rq_wrptr_db;
    wire                                 rx_engine_o_qpm_rw_rq_wrptr_db;
    wire [                         15:0] rx_engine_o_rq_wrptr_db;
    wire [                         15:0] qp_mgr_o_rq_pi_db;
    wire                                 qp_mgr_o_qp_rq_pi_db_valid;

    wire [                         23:0] qp_mgr_o_qp_stat_ret_sq_psn;
    wire                                 qp_mgr_o_qp_stat_ret_sq_psn_valid;


    wire [          C_QP_INDX_WIDTH-1:0] rx_engine_o_qp_req_conf_idx;
    wire [          C_QP_INDX_WIDTH-1:0] rx_engine_o_qp_rsp_conf_idx;

    wire [                         31:0] qp_mgr_o_qp_conf_replica;
    wire                                 qp_mgr_o_qp_conf_replica_valid;
    wire                                 rx_engine_o_qp_req_conf_en;

    wire                                 rx_engine_o_qp_rsp_conf_en;

    wire                                 rx_engine_req_remote_qp_mac_addr_en;
    wire                                 rx_engine_req_remote_qp_ip_addr_en;

    wire                                 rx_engine_rsp_remote_qp_mac_addr_en;
    wire                                 rx_engine_rsp_remote_qp_ip_addr_en;

    wire [                         31:0] qp_mgr_o_qp_mac_remote_addrl_replica;
    wire [                         31:0] qp_mgr_o_qp_mac_remote_addrm_replica;


    wire [                         31:0] qp_mgr_o_qp_ip_remote_addr1_replica;
    wire [                         31:0] qp_mgr_o_qp_ip_remote_addr2_replica;
    wire [                         31:0] qp_mgr_o_qp_ip_remote_addr3_replica;
    wire [                         31:0] qp_mgr_o_qp_ip_remote_addr4_replica;

    wire [                         31:0] rx_engine_o_qp_timeout_per_qp;

    wire                                 tx_engine_dma_in_idle;

    wire                                 rx_engine_o_qp_fatal_timer_idle;

    wire [          C_QP_INDX_WIDTH-1:0] wqe_fatal_qpid;
    wire                                 wqe_fatal_qp_vld;
    wire                                 wqe_fatal_qp_rdy;

    wire                                 rx_engine_waiting_for_rq_update;
    wire [          C_QP_INDX_WIDTH-1:0] rx_engine_waiting_for_rq_update_on_qp_num;

    wire [  C_S_AXI_LITE_ADDR_WIDTH-1:0] int_w_axi_lite_awaddr;
    wire                                 int_w_axi_lite_awready;
    wire                                 int_w_axi_lite_awvalid;
    wire [  C_S_AXI_LITE_ADDR_WIDTH-1:0] int_w_axi_lite_araddr;
    wire                                 int_w_axi_lite_arready;
    wire                                 int_w_axi_lite_arvalid;
    wire [                         31:0] int_w_axi_lite_wdata;
    wire [                          3:0] int_w_axi_lite_wstrb;
    wire                                 int_w_axi_lite_wready;
    wire                                 int_w_axi_lite_wvalid;
    wire [                         31:0] int_w_axi_lite_rdata;
    wire [                          1:0] int_w_axi_lite_rresp;
    wire                                 int_w_axi_lite_rready;
    wire                                 int_w_axi_lite_rvalid;
    wire [                          1:0] int_w_axi_lite_bresp;
    wire                                 int_w_axi_lite_bready;
    wire                                 int_w_axi_lite_bvalid;

    wire                                 lrdma_intr0;
    wire                                 lrdma_intr1;
    wire                                 lrdma_intr2;
    wire                                 lrdma_intr3;

    wire                                 ack_mgr_osq_pop;
    wire [        C_QP_INDX_WIDTH - 1:0] ack_mgr_osq_pop_qpid;
    wire                                 wqe_dropped;
    wire [        C_QP_INDX_WIDTH - 1:0] wqe_dropped_qpid;

    wire                                 tx_pause_req_on;
    wire [                        511:0] tx_engine_m_axis_tdata;
    wire                                 tx_engine_m_axis_tvalid;
    wire                                 tx_engine_m_axis_tlast;
    wire                                 tx_engine_m_axis_tready;
    wire [                         63:0] tx_engine_m_axis_tkeep;

    wire [                          3:0] roce_priority;
    wire [                          3:0] non_roce_priority;
    wire [                         15:0] roce_pause_on_threshold;
    wire [                         15:0] roce_pause_off_threshold;

    wire                                 pfc_en_roce;
    wire                                 pfc_en_non_roce;
    wire                                 disable_priority_chk;

    wire [                        511:0] cmac_m_axis_tdata_out;
    wire [                         63:0] cmac_m_axis_tkeep_out;
    wire                                 cmac_m_axis_tlast_out;
    wire                                 cmac_m_axis_tready_out;
    wire                                 cmac_m_axis_tvalid_out;

    wire                                 rx_engine_o_ib_pkt_hdr_valdn_err_intr_sts;
    wire                                 rx_engine_o_mad_pkt_rcvd_intr_sts;
    wire                                 rx_engine_fatal_err_intr_sts;

    wire                                 qp_mgr_o_qp_sq_psn_resp_valid;
    wire                                 ack_mgr_o_qp_sq_psn_resp_req;

    wire                                 ack_mgr_i_qp_retry_cnt_rdy;
    wire                                 ack_mgr_o_qp_cfg_retry_cnt_rd_req;
    wire                                 ack_mgr_i_qp_cfg_retry_cnt_rd_rdy;

    wire [                        576:0] cmac_reg_out;

    reg                                  init_start;  // 1 - 初始化状态; 0 - 非初始化状态
    reg  [        C_QP_INDX_WIDTH-1 : 0] init_count;  // 初始化计数器
    reg                                  core_rst_n_r;
    wire                                 osq_nacked_resp_wr_en;
    wire [        C_QP_INDX_WIDTH-1 : 0] osq_nacked_resp_wr_addr;
    wire                                 osq_nacked_resp_wr_data;
    wire                                 osq_nacked_resp_wr_en_aux;
    wire [        C_QP_INDX_WIDTH-1 : 0] osq_nacked_resp_wr_addr_aux;
    wire                                 osq_nacked_resp_wr_data_aux;
    wire                                 osq_nacked_resp_wr_en_pending;
    wire [        C_QP_INDX_WIDTH-1 : 0] osq_nacked_resp_wr_addr_pending;
    wire                                 osq_nacked_resp_wr_data_pending;
    wire                                 osq_full_set_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] osq_full_set_id;
    wire                                 osq_full_reset_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] osq_full_reset_id;
    wire                                 osq_timeout_set_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] osq_timeout_set_id;
    wire                                 osq_empty_set_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] osq_empty_set_id;
    wire                                 osq_empty_reset_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] osq_empty_reset_id;
    wire                                 qp_retried_set_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] qp_retried_set_id;
    wire                                 qp_retried_reset_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] qp_retried_reset_id;
    wire [                         31:0] clr_wqe_cmpl_sts_set_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] clr_wqe_cmpl_sts_set_id;
    wire                                 qp_disable_pulse_valid;
    wire [        C_QP_INDX_WIDTH-1 : 0] qp_disable_pulse_id;
    wire                                 qp_disable_pulse_data;
    wire                                 qp_enable_wr_en;
    wire [        C_QP_INDX_WIDTH-1 : 0] qp_enable_wr_addr;
    wire                                 qp_enable_wr_data;

    assign lrdma_intr0 = rx_engine_o_ib_pkt_hdr_valdn_err_intr;
    assign lrdma_intr1 = rx_engine_o_mad_pkt_rcvd_intr;
    assign lrdma_intr2 = ack_mgr_o_intr_wqe_cmpl;
    assign lrdma_intr3 = rx_engine_o_fatal_err_intr;
    assign qp_mgr_qp_conf_idx = rx_engine_o_qp_rsp_conf_en ? rx_engine_o_qp_rsp_conf_idx : (tx_engine_o_reg_rd_en_s ? tx_engine_o_qp_id : ack_mgr_o_qp_conf_qp_idx);
    assign tx_engine_qp_info_ready = qp_mgr_o_qp_conf_valid_tx_engine &
                                     qp_mgr_o_qp_adv_conf_valid &
                                     qp_mgr_o_qp_sq_psn_wqe_valid &
                                     qp_mgr_o_qp_stat_ssn_valid &
                                     qp_mgr_o_qp_dest_qpid_valid_tx_engine &
                                     qp_mgr_o_qp_mac_dest_addrl_valid &
                                     qp_mgr_o_qp_mac_dest_addrm_valid &
                                     qp_mgr_o_qp_ip_dest_addr1_valid &
                                     qp_mgr_o_qp_ip_dest_addr2_valid &
                                     qp_mgr_o_qp_ip_dest_addr3_valid &
                                     qp_mgr_o_qp_ip_dest_addr4_valid;

    always @(posedge m_axi_aclk) begin
        if (!m_axi_aresetn) lrdma_intr <= 0;
        else lrdma_intr <= lrdma_intr0 | lrdma_intr1 | lrdma_intr2 | lrdma_intr3;
    end

    lrdma_tx_flow_ctrl_mgr #(
        .C_ENABLE_LRDMA_TX_LOW_LATENCY_BYPASS(C_ENABLE_LRDMA_TX_LOW_LATENCY_BYPASS)
    ) inst_tx_flow_ctrl_mgr (
        .m_axi_aclk   (m_axi_aclk),
        .m_axi_aresetn(m_axi_aresetn_shards[0]),

        .cmac_clk (cmac_clk),
        .cmac_rstn(cmac_rstn),

        .cmac_m_axis_tdata (cmac_m_axis_tdata_out),
        .cmac_m_axis_tkeep (cmac_m_axis_tkeep_out),
        .cmac_m_axis_tvalid(cmac_m_axis_tvalid_out),
        .cmac_m_axis_tready(cmac_m_axis_tready_out),
        .cmac_m_axis_tlast (cmac_m_axis_tlast_out),
        .cmac_m_axis_tuser (),

        .qdma_s_axis_tvalid(non_roce_qdma_s_axis_tvalid),
        .qdma_s_axis_tdata (non_roce_qdma_s_axis_tdata),
        .qdma_s_axis_tkeep (non_roce_qdma_s_axis_tkeep),
        .qdma_s_axis_tlast (non_roce_qdma_s_axis_tlast),
        .qdma_s_axis_tuser (1'b0),
        .qdma_s_axis_tready(non_roce_qdma_s_axis_tready),

        .lrdma_s_axis_tvalid(tx_engine_m_axis_tvalid),
        .lrdma_s_axis_tdata (tx_engine_m_axis_tdata),
        .lrdma_s_axis_tkeep (tx_engine_m_axis_tkeep),
        .lrdma_s_axis_tlast (tx_engine_m_axis_tlast),
        .lrdma_s_axis_tready(tx_engine_m_axis_tready),

        // Prioroty flow control
        .stat_rx_pause_req     (stat_rx_pause_req),
        .ctl_tx_pause_req      (ctl_tx_pause_req),
        .roce_priority_s       (roce_priority),
        .non_roce_priority_s   (non_roce_priority),
        .roce_rx_buf_full_s    (tx_pause_req_on),
        .pfc_en_roce_s         (pfc_en_roce),
        .pfc_en_non_roce_s     (pfc_en_non_roce),
        .disable_priority_chk_s(disable_priority_chk)
    );


    lrdma_common_reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_inst (
        .aclk  (m_axi_aclk),
        .areset(~m_axi_aresetn_shards[0]),
        .aclken(1'b1),

        .S_PAYLOAD_DATA({cmac_m_axis_tlast_out, cmac_m_axis_tkeep_out, cmac_m_axis_tdata_out}),
        .S_VALID       (cmac_m_axis_tvalid_out),
        .S_READY       (cmac_m_axis_tready_out),

        .M_PAYLOAD_DATA(cmac_reg_out),
        .M_VALID       (primary_tvalid),
        .M_READY       (primary_tready)
    );

    assign primary_tdata = cmac_reg_out[511:0];
    assign primary_tkeep = cmac_reg_out[575:512];
    assign primary_tlast = cmac_reg_out[576];

    lrdma_rx_engine_top #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),  // 32
        .C_M_AXI_AWUSER_WIDTH(C_M_AXI_AWUSER_WIDTH),  // 32
        .C_NUM_QP(C_NUM_QP),  // 32
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH)  // 5
    ) inst_rx_engine (
        .core_clk  (m_axi_aclk),
        .core_rst_n(m_axi_aresetn_shards[1]),

        .s_axis_tvalid(native_rx_valid),
        .s_axis_tdata (roce_cmac_s_axis_tdata),
        .s_axis_tkeep (roce_cmac_s_axis_tkeep),
        .s_axis_tlast (roce_cmac_s_axis_tlast),
        .s_axis_tuser (~roce_cmac_s_axis_tuser),

        .i_lrdma_en      (qp_mgr_o_lrdma_en),       // LRDMA Configuration -- [0] LRDMA Enable
        .i_num_qp_enabled(qp_mgr_o_num_qp_en),
        .i_mac_node_addr (qp_mgr_o_mac_src_addr),
        .i_ipv4_node_addr(qp_mgr_o_ipv4_src_addr),
        .i_ipv6_node_addr(qp_mgr_o_ipv6_src_addr),
        .i_flow_credits  (qp_mgr_o_flow_credits),

        .i_req_err_buf_en    (qp_mgr_o_err_buf_en),
        .i_req_err_pkt_buf_ba(qp_mgr_o_err_pkt_buf_ba),
        .i_req_err_num_bufs  (qp_mgr_o_err_pkt_buf_sz_num_bufs),
        .i_req_err_buffer_sz (qp_mgr_o_err_pkt_buf_sz_buf_sz),
        .o_req_err_buf_wrptr (rx_engine_o_err_pkt_buf_wrptr),

        .i_in_errsts_q_ba         (qp_mgr_in_errsts_q_ba),
        .i_in_errsts_q_sz         (qp_mgr_in_errsts_q_sz),
        .o_in_errsts_q_wrptr      (rx_engine_in_errsts_q_wrptr),
        .o_inc_acknak_cnt         (rx_engine_o_ib_ack_pkt_cnt),
        .o_inc_mad_cnt            (rx_engine_o_ib_qp1_pkt_cnt),
        .o_inc_inv_pkt_cnt        (rx_engine_o_ib_inv_pkt_cnt),
        .o_inc_nack_pkt_cnt       (rx_engine_o_inc_nack_cnt),
        .pkt_all_cnt              (rx_engine_o_pkt_all_cnt),
        .pkt_disc_cnt             (rx_engine_o_pkt_disc_cnt),
        .i_qp_enable_wr_en        (qp_enable_wr_en),
        .i_qp_enable_wr_idx       (qp_enable_wr_addr),
        .i_qp_enable_wr_data      (qp_enable_wr_data),
        .o_last_in_req_pkt_opcode (),
        .o_last_in_req_pkt_qpid   (),
        .o_last_in_req_pkt_psn    (),
        .o_last_in_rsp_pkt_opcode (),
        .o_last_in_rsp_pkt_qpid   (),
        .o_last_in_rsp_pkt_psn    (),
        .o_last_in_pkt_opcode     (rx_engine_o_last_ib_pkt_opcode),
        .o_last_in_pkt_qpid       (rx_engine_o_last_ib_pkt_dest_qpid),
        .o_last_in_pkt_psn        (rx_engine_o_last_ib_pkt_psn),
        .o_rq_full_idx            (rx_engine_o_rq_full_idx),
        .o_rq_full_wr_en          (rx_engine_o_rq_full_wr_en),
        .o_rq_full_val            (rx_engine_o_rq_full_val),
        .o_rq_full_reset_idx      (rx_engine_o_rq_full_reset_idx),
        .o_rq_full_reset_wr_en    (rx_engine_o_rq_full_reset_wr_en),
        .o_rq_full_reset_val      (rx_engine_o_rq_full_reset_val),
        .o_qp_fatal_wr_en         (rx_engine_o_qp_fatal_wr_en),
        .o_qp_fatal_wr_idx        (rx_engine_o_qp_fatal_wr_idx),
        .o_qp_fatal_wr_data       (rx_engine_o_qp_fatal_wr_data),
        .i_clr_qp_fatal           (clr_qp_fatal_err),
        .i_clr_qp_fatal_idx       (clr_qp_fatal_idx),
        .qp_under_recovery_wr_en  (qp_under_recovery_wr_en),
        .qp_under_recovery_wr_idx (qp_under_recovery_wr_idx),
        .qp_under_recovery_wr_data(qp_under_recovery_wr_data),
        .i_pkt_valdn_err_intr_en  (qp_mgr_o_intr_en_pkt_valdn_err),
        .o_pkt_valdn_err_intr     (rx_engine_o_ib_pkt_hdr_valdn_err_intr),
        .o_pkt_valdn_err_intr_sts (rx_engine_o_ib_pkt_hdr_valdn_err_intr_sts),
        .i_clr_pkt_valdn_err_intr (qp_mgr_o_intr_clr_pkt_valdn_err),
        .i_mad_pkt_rcvd_intr_en   (qp_mgr_o_intr_en_mad_pkt_rcvd),
        .o_mad_pkt_rcvd_intr      (rx_engine_o_mad_pkt_rcvd_intr),
        .o_mad_pkt_rcvd_intr_sts  (rx_engine_o_mad_pkt_rcvd_intr_sts),
        .i_clr_mad_pkt_rcvd_intr  (qp_mgr_o_intr_clr_mad_pkt_rcvd),
        .i_fatal_err_intr_en      (qp_mgr_o_intr_en_fatal_err),
        .o_fatal_err_intr         (rx_engine_o_fatal_err_intr),
        .o_fatal_err_intr_sts     (rx_engine_fatal_err_intr_sts),
        .i_clr_fatal_err_intr     (qp_mgr_o_intr_clr_fatal_err),

        .i_qp_retried_set_valid  (qp_retried_set_valid),
        .i_qp_retried_set_id     (qp_retried_set_id),
        .i_qp_retried_reset_valid(qp_retried_reset_valid),
        .i_qp_retried_reset_id   (qp_retried_reset_id),
        .o_qp_index_acknack      (rx_engine_o_qp_index_acknack),
        .o_acknack_valid         (rx_engine_o_acknack_valid),
        .o_acknacked_psn         (rx_engine_o_acknacked_psn),
        .o_acknacked_syndrome    (rx_engine_o_acknacked_syndrome),
        .o_acknacked_msn         (rx_engine_o_acknacked_msn),
        .o_qp_timeout_per_qp     (rx_engine_o_qp_timeout_per_qp),
        //.i_osq_empty                   (ack_mgr_o_osq_empty           ),
        .i_osq_empty_set_valid   (osq_empty_set_valid),
        .i_osq_empty_set_id      (osq_empty_set_id),
        .i_osq_empty_reset_valid (osq_empty_reset_valid),
        .i_osq_empty_reset_id    (osq_empty_reset_id),
        .i_osq_nacked            (o_osq_nacked),
        .i_qp_fatal_tx_engine    (wqe_fatal_qpid),
        .i_qp_fatal_tx_engine_vld(wqe_fatal_qp_vld),
        .o_qp_fatal_tx_engine_rdy(wqe_fatal_qp_rdy),
        .i_qp_fatal_ack_mgr      (ack_mgr_o_qp_retry_fatal_qpid),
        .i_qp_fatal_ack_mgr_vld  (ack_mgr_o_qp_retry_fatal_req),
        .o_qp_fatal_ack_mgr_rdy  (ack_mgr_i_qp_retry_fatal_rdy),

        //.i_rd_req_fifo_full            (rd_req_fifo_full),
        .o_qp_req_conf_idx           (rx_engine_o_qp_req_conf_idx),
        .i_qp_req_conf               (qp_mgr_o_qp_conf_replica),                                                                                                                              // RX 请求/MAD 路径读取的 QPCONFi 复制数据
        .qp_req_conf_vld             (qp_mgr_o_qp_conf_replica_valid),                                                                                                                        // QPCONFi 复制数据有效
        .qp_req_conf_en              (rx_engine_o_qp_req_conf_en),                                                                                                                            // RX 请求/MAD 路径发起 QPCONFi 读取
        .i_rq_buf_ba                 (qp_mgr_o_qp_rq_ba),
        .rq_buf_ba_en                (rx_engine_o_qpm_req_rq_buf_ba),
        .i_rq_buf_ca                 (qp_mgr_o_qp_stat_rq_buf_ca),
        .o_rq_buf_ca                 (rx_engine_o_rq_buf_ca),
        .rq_buf_ca_en                (rx_engine_o_qpm_req_rq_buf_ca),
        .rq_buf_ca_we                (rx_engine_o_qpm_rw_rq_buf_ca),
        .i_rq_wrptr_db_addr          (qp_mgr_o_qp_rq_wrptrdb_add),
        .rq_wrptr_db_addr_en         (rx_engine_o_qpm_req_rq_wrptr_db_add),
        .i_q_depth                   (qp_mgr_o_qp_rq_depth),
        .q_depth_en                  (rx_engine_o_qpm_req_q_depth),
        .i_rq_ci_db                  (qp_mgr_o_qp_rq_cidb[15:0]),
        .rq_ci_db_en                 (rx_engine_o_qpm_req_rq_ci_db),
        .i_last_req                  (qp_mgr_o_qp_last_req),
        .o_last_req                  (rx_engine_o_last_req),
        .last_req_en                 (rx_engine_o_qpm_req_last_req),
        .last_req_we                 (rx_engine_o_qpm_rw_last_req),
        .i_qp_req_remote_qp_mac_addr ({qp_mgr_o_qp_mac_remote_addrm_replica[15:0], qp_mgr_o_qp_mac_remote_addrl_replica}),                                                                    // RX 请求/MAD 路径读取的远端 QP MAC 地址
        .qp_req_remote_qp_mac_addr_en(rx_engine_req_remote_qp_mac_addr_en),                                                                                                                   // RX 请求/MAD 路径发起远端 QP MAC 地址读取
        .i_qp_req_remote_qp_ip_addr  ({qp_mgr_o_qp_ip_remote_addr4_replica, qp_mgr_o_qp_ip_remote_addr3_replica, qp_mgr_o_qp_ip_remote_addr2_replica, qp_mgr_o_qp_ip_remote_addr1_replica}),
        .qp_req_remote_qp_ip_addr_en (rx_engine_req_remote_qp_ip_addr_en),
        .i_rq_wrptr_db               (qp_mgr_o_rq_pi_db),
        .o_rq_wrptr_db               (rx_engine_o_rq_wrptr_db),
        .rq_wrptr_db_en              (rx_engine_o_qpm_req_rq_wrptr_db),
        .rq_wrptr_db_we              (rx_engine_o_qpm_rw_rq_wrptr_db),


        .o_qp_rsp_conf_idx           (rx_engine_o_qp_rsp_conf_idx),
        .i_qp_rsp_conf               (qp_mgr_o_qp_conf),
        .qp_rsp_conf_vld             (qp_mgr_o_qp_conf_valid_rx_engine),
        .qp_rsp_conf_en              (rx_engine_o_qp_rsp_conf_en),
        .i_stat_nak                  (ack_mgr_o_qp_stat_nak),
        .stat_nak_en                 (rx_engine_o_qpm_req_stat_nak),
        .stat_nak_we                 (rx_engine_o_qpm_rw_stat_nak),
        .o_stat_nak                  (rx_engine_o_stat_nak),
        .i_stat_resp_psn             (qp_mgr_o_qp_stat_resp_psn),
        .stat_resp_psn_en            (rx_engine_o_qpm_req_stat_resp_psn),
        .stat_resp_psn_we            (rx_engine_o_qpm_rw_stat_resp_psn),
        .o_stat_resp_psn             (rx_engine_o_qp_stat_resp_psn),
        .i_sq_psn                    (qp_mgr_o_qp_sq_psn),
        .sq_psn_en                   (rx_engine_o_qpm_req_sq_psn),
        .i_qp_rsp_remote_qp_mac_addr ({qp_mgr_o_qp_mac_dest_addrm[15:0], qp_mgr_o_qp_mac_dest_addrl}),
        .qp_rsp_remote_qp_mac_addr_en(rx_engine_rsp_remote_qp_mac_addr_en),
        .i_qp_rsp_remote_qp_ip_addr  ({qp_mgr_o_qp_ip_dest_addr4, qp_mgr_o_qp_ip_dest_addr3, qp_mgr_o_qp_ip_dest_addr2, qp_mgr_o_qp_ip_dest_addr1}),
        .qp_rsp_remote_qp_ip_addr_en (rx_engine_rsp_remote_qp_ip_addr_en),
        .o_qp_fatal_timer_idle       (rx_engine_o_qp_fatal_timer_idle),
        .i_qp_timeout                (qp_mgr_o_qp_timeout_per_qp),
        .rx_req_axi_awid             (m_axi_rdma_send_write_payload_store_awid),
        .rx_req_axi_awaddr           (m_axi_rdma_send_write_payload_store_awaddr),
        .rx_req_axi_awuser           (m_axi_rdma_send_write_payload_store_awuser),
        .rx_req_axi_awlen            (m_axi_rdma_send_write_payload_store_awlen),
        .rx_req_axi_awsize           (m_axi_rdma_send_write_payload_store_awsize),
        .rx_req_axi_awburst          (m_axi_rdma_send_write_payload_store_awburst),
        .rx_req_axi_awcache          (m_axi_rdma_send_write_payload_store_awcache),
        .rx_req_axi_awprot           (m_axi_rdma_send_write_payload_store_awprot),
        .rx_req_axi_awvalid          (m_axi_rdma_send_write_payload_store_awvalid),
        .rx_req_axi_awready          (m_axi_rdma_send_write_payload_store_awready),
        .rx_req_axi_wdata            (m_axi_rdma_send_write_payload_store_wdata),
        .rx_req_axi_wstrb            (m_axi_rdma_send_write_payload_store_wstrb),
        .rx_req_axi_wlast            (m_axi_rdma_send_write_payload_store_wlast),
        .rx_req_axi_wvalid           (m_axi_rdma_send_write_payload_store_wvalid),
        .rx_req_axi_wready           (m_axi_rdma_send_write_payload_store_wready),
        .rx_req_axi_awlock           (m_axi_rdma_send_write_payload_store_awlock),
        .rx_req_axi_bid              (m_axi_rdma_send_write_payload_store_bid),
        .rx_req_axi_bresp            (m_axi_rdma_send_write_payload_store_bresp),
        .rx_req_axi_bvalid           (m_axi_rdma_send_write_payload_store_bvalid),
        .rx_req_axi_bready           (m_axi_rdma_send_write_payload_store_bready),
        .rx_req_axi_arid             (m_axi_rdma_send_write_payload_store_arid),
        .rx_req_axi_araddr           (m_axi_rdma_send_write_payload_store_araddr),
        .rx_req_axi_arlen            (m_axi_rdma_send_write_payload_store_arlen),
        .rx_req_axi_arsize           (m_axi_rdma_send_write_payload_store_arsize),
        .rx_req_axi_arburst          (m_axi_rdma_send_write_payload_store_arburst),
        .rx_req_axi_arcache          (m_axi_rdma_send_write_payload_store_arcache),
        .rx_req_axi_arprot           (m_axi_rdma_send_write_payload_store_arprot),
        .rx_req_axi_arvalid          (m_axi_rdma_send_write_payload_store_arvalid),
        .rx_req_axi_arready          (m_axi_rdma_send_write_payload_store_arready),
        .rx_req_axi_rid              (m_axi_rdma_send_write_payload_store_rid),
        .rx_req_axi_rdata            (m_axi_rdma_send_write_payload_store_rdata),
        .rx_req_axi_rresp            (m_axi_rdma_send_write_payload_store_rresp),
        .rx_req_axi_rlast            (m_axi_rdma_send_write_payload_store_rlast),
        .rx_req_axi_rvalid           (m_axi_rdma_send_write_payload_store_rvalid),
        .rx_req_axi_rready           (m_axi_rdma_send_write_payload_store_rready),
        .rx_req_axi_arlock           (m_axi_rdma_send_write_payload_store_arlock),

        .waiting_for_rq_update          (rx_engine_waiting_for_rq_update),
        .waiting_for_rq_update_on_qp_num(rx_engine_waiting_for_rq_update_on_qp_num),

        .i_lrdma_pause_on_threshold (roce_pause_on_threshold),
        .i_lrdma_pause_off_threshold(roce_pause_off_threshold),
        .tx_pause_req_on            (tx_pause_req_on),

        .init_start(init_start),
        .init_count(init_count)
    );

    lrdma_ack_mgr_top #(
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_NUM_QP          (C_NUM_QP),
        .C_QP_INDX_WIDTH   (C_QP_INDX_WIDTH),
        .C_OS_Q_DEPTH      (OS_Q_DEPTH),
        .C_OS_Q_INDX_WIDTH (OS_Q_INDX_WIDTH),
        .C_OSQ_PSN_WIDTH   (C_OSQ_PSN_WIDTH),
        .RESP_WIDTH        (2)
    ) inst_ack_mgr (
        .core_clk (m_axi_aclk),    // Module Clock  
        .core_rstn(m_axi_aresetn_shards[2]), // Active low core reset

        // Interface to the TX Engine to the outstanding buffers
        // TX 引擎与未完成队列（OSQ）的接口
        .i_wqe_qpid                       (tx_engine_os_wqe_qpid),            // QP ID of WQE
        .i_wqe_opcode                     (tx_engine_os_wqe_opcode),          // BTH.Opcode of outbound request
        .i_wqe_wrid                       (tx_engine_os_wqe_wrid),
        .i_wqe_retried                    (tx_engine_os_wqe_retried),
        .i_wqe_fatal_push                 (tx_engine_os_wqe_fatal_push),
        .i_wqe_send_end_psn               (tx_engine_os_wqe_send_psn),        // PSN of most recent request
        .i_wqe_send_start_psn             (tx_engine_os_wqe_send_first_psn),  // PSN of most recent request
        .i_wqe_send_msn                   (tx_engine_os_wqe_send_msn),
        .i_wqe_push_req                   (tx_engine_os_wqp_req),             // request from TX 引擎 to push WQE
        .o_wqe_push_rdy                   (tx_engine_os_wqp_rdy),             // response to TX 引擎; 0 - busy; 1 - completed  
        .i_osq_wqe_nack_resp_valid        (osq_nacked_resp_wr_en),
        .i_osq_wqe_nack_resp_id           (osq_nacked_resp_wr_addr),
        .i_osq_wqe_nack_resp_data         (osq_nacked_resp_wr_data),
        .i_osq_wqe_nack_resp_valid_aux    (osq_nacked_resp_wr_en_aux),
        .i_osq_wqe_nack_resp_id_aux       (osq_nacked_resp_wr_addr_aux),
        .i_osq_wqe_nack_resp_data_aux     (osq_nacked_resp_wr_data_aux),
        .i_osq_wqe_nack_resp_valid_pending(osq_nacked_resp_wr_en_pending),
        .i_osq_wqe_nack_resp_id_pending   (osq_nacked_resp_wr_addr_pending),
        .i_osq_wqe_nack_resp_data_pending (osq_nacked_resp_wr_data_pending),
        .o_osq_full_set_valid             (osq_full_set_valid),
        .o_osq_full_set_id                (osq_full_set_id),
        .o_osq_full_reset_valid           (osq_full_reset_valid),
        .o_osq_full_reset_id              (osq_full_reset_id),

        .o_ack_mgr_sts     (ack_mgr_o_ack_mgr_sts),
        .o_stat_retry_count(ack_mgr_o_stat_retry_count),

        //.i_qp_disable_pulse               (qp_mgr_o_qp_disable_pulse), // whenever QP_enable bit is written this pulse is generated and used for first_outgoing_pkt_ff signal generation 
        .i_qp_disable_pulse_set_valid       (qp_disable_pulse_valid && qp_disable_pulse_data),
        .i_qp_disable_pulse_set_id          (qp_disable_pulse_id),
        .i_cfg_retry_cnt_fatal_dis          (qp_mgr_o_lrdma_adv_conf_retry_cnt_fatal_dis),
        .i_qp_fatal_valid                   (rx_engine_o_qp_fatal_wr_en),
        .i_qp_fatal_id                      (rx_engine_o_qp_fatal_wr_idx),
        .i_qp_fatal_data                    (rx_engine_o_qp_fatal_wr_data),
        .i_dma_in_idle                      (tx_engine_dma_in_idle),
        .i_en_osq_flush_on_qp_fatal         (rx_engine_o_qp_fatal_timer_idle),
        .o_osq_nacked                       (o_osq_nacked),
        // Completion interrupt
        .i_intr_en_wqe_cmpl                 (qp_mgr_o_intr_en_wqe_cmpl),
        .i_intr_clr_wqe_cmpl                (qp_mgr_o_intr_clr_wqe_cmpl),
        .o_intr_wqe_cmpl                    (ack_mgr_o_intr_wqe_cmpl),
        .o_intr_wqe_cmpl_sts_or_reduce      (intr_wqe_cmpl_sts_or_reduce),
        .o_intr_wqe_cmpl_sts_wr_en          (intr_wqe_cmpl_sts_wr_en),
        .o_intr_wqe_cmpl_sts_wr_addr        (intr_wqe_cmpl_sts_wr_addr),
        .o_intr_wqe_cmpl_sts_wr_data        (intr_wqe_cmpl_sts_wr_data),
        .o_intr_wqe_cmpl_sts_wr_en_pending  (intr_wqe_cmpl_sts_wr_en_pending),
        .o_intr_wqe_cmpl_sts_wr_addr_pending(intr_wqe_cmpl_sts_wr_addr_pending),
        .i_clr_wqe_cmpl_sts_set_valid       (clr_wqe_cmpl_sts_set_valid),
        .i_clr_wqe_cmpl_sts_set_id          (clr_wqe_cmpl_sts_set_id),

        // RX 引擎到 ACK 管理器的 ACK/NAK 处理事件接口
        .i_qp_index_acknack(rx_engine_o_qp_index_acknack),   // QP ID for ack_nack packet PSN
        .i_acknack_valid   (rx_engine_o_acknack_valid),      // Valid signal qualifying the qp_index and acked PSN
        .i_acknacked_psn   (rx_engine_o_acknacked_psn),      // PSN for the packet being acked/nacked
        .i_acknacked_msn   (rx_engine_o_acknacked_msn),      // PSN for the packet being acked/nacked
        .i_pkt_nacksyndrome(rx_engine_o_acknacked_syndrome), // NACK Syndrome. Valid when the i_pkt_nack is asserted

        // Status output
        .o_inc_nack_cnt(ack_mgr_o_inc_nack_cnt),

        // ---- 与 QP 管理器的接口，用于重传
        .o_retransmit_qpid        (ack_mgr_o_retransmit_qpid),          // 本次重传需要 QP ID
        .o_retransmit_reqd        (ack_mgr_o_retransmit_reqd),          // Retransmission is required
        .o_retransmit_wqe_bram_din(ack_mgr_o_retransmit_wqe_bram_din),
        .i_retransmit_accepted    (qp_mgr_o_retransmit_accepted),       // QP 管理器已接收重传请求
        .o_psn_to_retry           (ack_mgr_o_psn_to_retry),
        .o_ssn_to_retry           (ack_mgr_o_ssn_to_retry),
        //.o_osq_empty                      (ack_mgr_o_osq_empty),
        .o_osq_empty_set_valid    (osq_empty_set_valid),
        .o_osq_empty_set_id       (osq_empty_set_id),
        .o_osq_empty_reset_valid  (osq_empty_reset_valid),
        .o_osq_empty_reset_id     (osq_empty_reset_id),
        .o_osq_timeout_set_valid  (osq_timeout_set_valid),
        .o_osq_timeout_set_id     (osq_timeout_set_id),
        .o_qp_retried_set_valid   (qp_retried_set_valid),
        .o_qp_retried_set_id      (qp_retried_set_id),
        .o_qp_retried_reset_valid (qp_retried_reset_valid),
        .o_qp_retried_reset_id    (qp_retried_reset_id),
        .i_halt                   (qp_mgr_o_halt),
        .i_halted_qpid            (qp_mgr_o_halted_qpid),


        // QP 管理器配置接口  
        .o_qp_conf_qp_idx   (ack_mgr_o_qp_conf_qp_idx),
        .o_qp_cq_ba_req     (ack_mgr_o_qp_cq_ba_req),
        .i_qp_cq_ba         (qp_mgr_o_qp_cq_ba),           // CQ base address
        .i_qp_cq_ba_valid   (qp_mgr_o_qp_cq_ba_valid),
        .o_qp_cq_depth_req  (ack_mgr_o_qp_cq_depth_req),
        .i_qp_cq_depth_valid(qp_mgr_o_qp_cq_depth_valid),  // CQ depth
        .i_qp_cq_depth      (qp_mgr_o_qp_cq_depth),

        .i_qp_sq_psn_resp_valid(qp_mgr_o_qp_sq_psn_resp_valid),
        .i_qp_sq_psn           (qp_mgr_o_qp_sq_psn),
        .o_qp_sq_psn_resp_req  (ack_mgr_o_qp_sq_psn_resp_req),
        .o_qp_sq_psn_resp_idx  (ack_mgr_o_qp_sq_psn_resp_idx),

        .o_qp_conf_req  (ack_mgr_o_qp_conf_req),
        .i_qp_conf_valid(qp_mgr_o_qp_conf_valid_ack_mgr),
        .i_qp_conf      (qp_mgr_o_qp_conf),

        .o_qp_cq_hdptr_req  (ack_mgr_o_qp_cq_hdptr_req),
        .i_qp_cq_hdptr_valid(qp_mgr_o_qp_cq_hdptr_valid),  // CQ head pointer
        .i_qp_cq_hdptr      (qp_mgr_o_qp_cq_hdptr),
        .o_qp_cq_hdptr      (ack_mgr_o_qp_cq_hdptr),
        .o_qp_cq_hdptr_wrn  (ack_mgr_o_qp_cq_hdptr_wrn),
        .o_qp_cqe_proc_qpid (ack_mgr_o_qp_cqe_proc_qpid),

        .i_num_elements_enabled (qp_mgr_o_num_qp_en),
        .i_timer_loadval_wqe    (tx_engine_o_wqe_qp_to),
        .i_timer_loadval_acknack(rx_engine_o_qp_timeout_per_qp[4:0]),

        //QP STAT register IF
        .i_qp_curr_retry_cnt       (ack_mgr_i_qp_retry_cnt_curr),
        .o_qp_curr_retry_cnt_rd_req(ack_mgr_o_qp_retry_cnt_req),
        .o_qp_curr_retry_cnt_qpid  (ack_mgr_o_qp_retry_cnt_qpid),
        .i_qp_curr_retry_cnt_rd_rdy(ack_mgr_i_qp_retry_cnt_rdy),

        .o_qp_curr_retry_cnt_wr_req(ack_mgr_o_qp_retry_cnt_wen),
        .i_qp_curr_retry_cnt_wr_rdy(ack_mgr_i_qp_retry_cnt_rdy),
        //.o_qp_curr_retry_cnt_qpid	(),
        .o_qp_curr_retry_cnt       (ack_mgr_o_qp_retry_cnt_new),

        .o_qp_cfg_retry_cnt_rd_req(ack_mgr_o_qp_cfg_retry_cnt_rd_req),
        .i_qp_cfg_retry_cnt_rd_rdy(ack_mgr_i_qp_cfg_retry_cnt_rd_rdy),
        .i_qp_cfg_retry_cnt       (qp_mgr_o_qp_timeout_per_qp[10:8]),

        //QP fatal IF for retry limit exceed case
        .o_qp_retry_fatal_req (ack_mgr_o_qp_retry_fatal_req),
        .o_qp_retry_fatal_qpid(ack_mgr_o_qp_retry_fatal_qpid),
        .i_qp_retry_fatal_rdy (ack_mgr_i_qp_retry_fatal_rdy),

        // AXI Master signals
        // Write address channel
        .m_axi_awid   (cq_core_awid),
        .m_axi_awaddr (cq_core_awaddr),
        .m_axi_awlen  (cq_core_awlen),
        .m_axi_awsize (cq_core_awsize),
        .m_axi_awburst(cq_core_awburst),
        .m_axi_awcache(cq_core_awcache),
        .m_axi_awprot (cq_core_awprot),
        .m_axi_awvalid(cq_core_awvalid),
        .m_axi_awready(cq_core_awready),

        .m_axi_wdata (cq_core_wdata),
        .m_axi_wstrb (cq_core_wstrb),
        .m_axi_wlast (cq_core_wlast),
        .m_axi_wvalid(cq_core_wvalid),
        .m_axi_wready(cq_core_wready),
        .m_axi_awlock(cq_core_awlock),

        .m_axi_bid   (cq_core_bid),
        .m_axi_bresp (cq_core_bresp),
        .m_axi_bvalid(cq_core_bvalid),
        .m_axi_bready(cq_core_bready),

        .m_axi_arid   (m_axi_write_completion_arid),
        .m_axi_araddr (m_axi_write_completion_araddr),
        .m_axi_arlen  (m_axi_write_completion_arlen),
        .m_axi_arsize (m_axi_write_completion_arsize),
        .m_axi_arburst(m_axi_write_completion_arburst),
        .m_axi_arcache(m_axi_write_completion_arcache),
        .m_axi_arprot (m_axi_write_completion_arprot),
        .m_axi_arvalid(m_axi_write_completion_arvalid),
        .m_axi_arready(m_axi_write_completion_arready),

        .m_axi_rid   (m_axi_write_completion_rid),
        .m_axi_rdata (m_axi_write_completion_rdata),
        .m_axi_rresp (m_axi_write_completion_rresp),
        .m_axi_rlast (m_axi_write_completion_rlast),
        .m_axi_rvalid(m_axi_write_completion_rvalid),
        .m_axi_rready(m_axi_write_completion_rready),
        .m_axi_arlock(m_axi_write_completion_arlock),

        .o_osq_pop                  (ack_mgr_osq_pop),
        .o_osq_pop_qpid             (ack_mgr_osq_pop_qpid),
        .init_start                 (init_start),
        .init_count                 (init_count),
        .i_qp_under_recovery_wr_en  (qp_under_recovery_wr_en),
        .i_qp_under_recovery_wr_idx (qp_under_recovery_wr_idx),
        .i_qp_under_recovery_wr_data(qp_under_recovery_wr_data),
        .i_qp_enable_valid          (qp_enable_wr_en),
        .i_qp_enable_id             (qp_enable_wr_addr),
        .i_qp_enable_data           (qp_enable_wr_data)
    );

    assign rx_engine_o_qpm_req_dest_qp_conf = 1'b0;  // RX 请求路径不读取 dest_qp_conf，固定为 0

    // axil lrdma_common_fifo
    lrdma_common_xpm_fifo_axil_wrap #(
        .AXI_ADDR_WIDTH          (C_S_AXI_LITE_ADDR_WIDTH),  // DECIMAL
        .AXI_DATA_WIDTH          (32),                       // DECIMAL
        .CDC_SYNC_STAGES         (2),                        // DECIMAL
        .CLOCKING_MODE           ("independent"),
        .ECC_MODE_RDCH           ("no_ecc"),                 // String
        .ECC_MODE_WDCH           ("no_ecc"),                 // String
        .FIFO_DEPTH_RACH         (16),                       // DECIMAL
        .FIFO_DEPTH_RDCH         (16),                       // DECIMAL
        .FIFO_DEPTH_WACH         (16),                       // DECIMAL
        .FIFO_DEPTH_WDCH         (16),                       // DECIMAL
        .FIFO_DEPTH_WRCH         (16),                       // DECIMAL
        .FIFO_MEMORY_TYPE_RACH   ("auto"),                   // String
        .FIFO_MEMORY_TYPE_RDCH   ("auto"),                   // String
        .FIFO_MEMORY_TYPE_WACH   ("auto"),                   // String
        .FIFO_MEMORY_TYPE_WDCH   ("auto"),                   // String
        .FIFO_MEMORY_TYPE_WRCH   ("auto"),                   // String
        .PROG_EMPTY_THRESH_RDCH  (5),                        // DECIMAL
        .PROG_EMPTY_THRESH_WDCH  (5),                        // DECIMAL
        .PROG_FULL_THRESH_RDCH   (5),                        // DECIMAL
        .PROG_FULL_THRESH_WDCH   (5),                        // DECIMAL
        .RD_DATA_COUNT_WIDTH_RDCH(5),                        // DECIMAL To reflect the correct value, the width should be log2(FIFO_DEPTH)+1
        .RD_DATA_COUNT_WIDTH_WDCH(5),                        // DECIMAL To reflect the correct value, the width should be log2(FIFO_DEPTH)+1
        .SIM_ASSERT_CHK          (0),                        // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .USE_ADV_FEATURES_RDCH   ("1000"),                   // String
        .USE_ADV_FEATURES_WDCH   ("1000"),                   // String
        .WR_DATA_COUNT_WIDTH_RDCH(5),                        // DECIMAL To reflect the correct value, the width should be log2(FIFO_DEPTH)+1
        .WR_DATA_COUNT_WIDTH_WDCH(5)                         // DECIMAL To reflect the correct value, the width should be log2(FIFO_DEPTH)+1
    ) u_xpm_fifo_axil_wrap (
        .dbiterr_rdch      (),                        // 1-bit output: Double Bit Error- Indicates that the ECC
        .dbiterr_wdch      (),                        // 1-bit output: Double Bit Error- Indicates that the ECC
        .m_axi_araddr      (int_w_axi_lite_araddr),   // AXI_ADDR_WIDTH-bit output: ARADDR: The read address bus gives
        .m_axi_arprot      (),                        // 2-bit output: ARPROT: Indicates the normal, privileged, or
        .m_axi_arvalid     (int_w_axi_lite_arvalid),  // 1-bit output: ARVALID: Indicates that the master is driving a
        .m_axi_awaddr      (int_w_axi_lite_awaddr),   // AXI_ADDR_WIDTH-bit output: AWADDR: The write address bus
        .m_axi_awprot      (),                        // 2-bit output: AWPROT: Indicates the normal, privileged, or
        .m_axi_awvalid     (int_w_axi_lite_awvalid),  // 1-bit output: AWVALID: Indicates that the master is driving a
        .m_axi_bready      (int_w_axi_lite_bready),   // 1-bit output: BREADY: Indicates that the master can accept a
        .m_axi_rready      (int_w_axi_lite_rready),   // 1-bit output: RREADY: Indicates that the master can accept a
        .m_axi_wdata       (int_w_axi_lite_wdata),    // AXI_DATA_WIDTH-bit output: WDATA: The primary payload that is
        .m_axi_wstrb       (int_w_axi_lite_wstrb),    // AXI_DATA_WIDTH-bit output: WSTRB: The byte qualifier that
        .m_axi_wvalid      (int_w_axi_lite_wvalid),   // 1-bit output: WVALID: Indicates that the master is driving a
        .prog_empty_rdch   (),                        // 1-bit output: Programmable Empty- This signal is asserted
        .prog_empty_wdch   (),                        // 1-bit output: Programmable Empty- This signal is asserted
        .prog_full_rdch    (),                        // 1-bit output: Programmable Full: This signal is asserted when
        .prog_full_wdch    (),                        // 1-bit output: Programmable Full: This signal is asserted when
        .rd_data_count_rdch(),                        // RD_DATA_COUNT_WIDTH_RDCH-bit output: Read Data Count- This
        .rd_data_count_wdch(),                        // RD_DATA_COUNT_WIDTH_WDCH-bit output: Read Data Count- This
        .s_axi_arready     (s_axi_lite_arready),      // 1-bit output: ARREADY: Indicates that the slave can accept a
        .s_axi_awready     (s_axi_lite_awready),      // 1-bit output: AWREADY: Indicates that the slave can accept a
        .s_axi_bresp       (s_axi_lite_bresp),        // 2-bit output: BRESP: Write Response. Indicates the status of
        .s_axi_bvalid      (s_axi_lite_bvalid),       // 1-bit output: BVALID: Indicates that the master is driving a
        .s_axi_rdata       (s_axi_lite_rdata),        // AXI_DATA_WIDTH-bit output: RDATA: The primary payload that is
        .s_axi_rresp       (s_axi_lite_rresp),        // 2-bit output: RRESP: Indicates the status of the read
        .s_axi_rvalid      (s_axi_lite_rvalid),       // 1-bit output: RVALID: Indicates that the master is driving a
        .s_axi_wready      (s_axi_lite_wready),       // 1-bit output: WREADY: Indicates that the slave can accept a
        .sbiterr_rdch      (),                        // 1-bit output: Single Bit Error (ECC disabled, left unconnected)
        .sbiterr_wdch      (),                        // 1-bit output: Single Bit Error (ECC disabled, left unconnected)
        .wr_data_count_rdch(),                        // WR_DATA_COUNT_WIDTH_RDCH-bit output: Write Data Count: This
        .wr_data_count_wdch(),                        // WR_DATA_COUNT_WIDTH_WDCH-bit output: Write Data Count: This
        .injectdbiterr_rdch(1'b0),                    // 1-bit input: Double Bit Error Injection- Injects a double bit
        .injectdbiterr_wdch(1'b0),                    // 1-bit input: Double Bit Error Injection- Injects a double bit
        .injectsbiterr_rdch(1'b0),                    // 1-bit input: Single Bit Error Injection- Injects a single bit
        .injectsbiterr_wdch(1'b0),                    // 1-bit input: Single Bit Error Injection- Injects a single bit

        .m_aclk       (m_axi_aclk),              // 1-bit input: Master Interface Clock: All signals on master
        .m_axi_arready(int_w_axi_lite_arready),  // 1-bit input: ARREADY: Indicates that the master can accept a
        .m_axi_awready(int_w_axi_lite_awready),  // 1-bit input: AWREADY: Indicates that the master can accept a
        .m_axi_bresp  (int_w_axi_lite_bresp),    // 2-bit input: BRESP: Write Response. Indicates the status of
        .m_axi_bvalid (int_w_axi_lite_bvalid),   // 1-bit input: BVALID: Indicates that the master is driving a
        .m_axi_rdata  (int_w_axi_lite_rdata),    // AXI_DATA_WIDTH-bit input: RDATA: The primary payload that is
        .m_axi_rresp  (int_w_axi_lite_rresp),    // 2-bit input: RRESP: Indicates the status of the read
        .m_axi_rvalid (int_w_axi_lite_rvalid),   // 1-bit input: RVALID: Indicates that the master is driving a
        .m_axi_wready (int_w_axi_lite_wready),   // 1-bit input: WREADY: Indicates that the master can accept a

        .s_aclk       (s_axi_lite_aclk),     // 1-bit input: Slave Interface Clock: All signals on slave
        .s_aresetn    (s_axi_lite_aresetn),  // 1-bit input: Active low asynchronous reset.
        .s_axi_araddr (s_axi_lite_araddr),   // AXI_ADDR_WIDTH-bit input: ARADDR: The read address bus gives
        .s_axi_arprot (3'b0),                // 3-bit input: ARPROT: Indicates the normal, privileged, or
        .s_axi_arvalid(s_axi_lite_arvalid),  // 1-bit input: ARVALID: Indicates that the master is driving a
        .s_axi_awaddr (s_axi_lite_awaddr),   // AXI_ADDR_WIDTH-bit input: AWADDR: The write address bus gives
        .s_axi_awprot (3'b0),                // 3-bit input: AWPROT: Indicates the normal, privileged, or
        .s_axi_awvalid(s_axi_lite_awvalid),  // 1-bit input: AWVALID: Indicates that the master is driving a
        .s_axi_bready (s_axi_lite_bready),   // 1-bit input: BREADY: Indicates that the slave can accept a
        .s_axi_rready (s_axi_lite_rready),   // 1-bit input: RREADY: Indicates that the slave can accept a
        .s_axi_wdata  (s_axi_lite_wdata),    // AXI_DATA_WIDTH-bit input: WDATA: The primary payload that is
        .s_axi_wstrb  (s_axi_lite_wstrb),    // AXI_DATA_WIDTH-bit input: WSTRB: The byte qualifier that
        .s_axi_wvalid (s_axi_lite_wvalid)    // 1-bit input: WVALID: Indicates that the master is driving a
    );

    lrdma_qp_mgr_top #(
        .C_S_AXI_LITE_ADDR_WIDTH    (C_S_AXI_LITE_ADDR_WIDTH),
        .C_S_AXI_LITE_DATA_WIDTH    (32),
        .C_M_AXI_ADDR_WIDTH         (C_M_AXI_ADDR_WIDTH),
        .C_NUM_QP                   (C_NUM_QP),
        .C_QP_INDX_WIDTH            (C_QP_INDX_WIDTH),
        .STB_WIDTH                  (4),
        .C_OS_Q_DEPTH               (OS_Q_DEPTH),
        .C_OS_Q_INDX_WIDTH          (OS_Q_INDX_WIDTH),
        .C_ENABLE_FAST_WQE_AXI_EN   (C_ENABLE_FAST_WQE_AXI_EN),
        .C_ENABLE_WQE_BYPASS        (C_ENABLE_WQE_BYPASS),
        .C_ENABLE_SAME_SQ_FAST_FETCH(C_ENABLE_SAME_SQ_FAST_FETCH)
    ) inst_qp_mgr (
        .s_axi_lite_aclk   (m_axi_aclk),
        .s_axi_lite_aresetn(m_axi_aresetn_shards[3]),

        .s_axi_lite_awaddr (int_w_axi_lite_awaddr),
        .s_axi_lite_awready(int_w_axi_lite_awready),
        .s_axi_lite_awvalid(int_w_axi_lite_awvalid),
        .s_axi_lite_araddr (int_w_axi_lite_araddr),
        .s_axi_lite_arready(int_w_axi_lite_arready),
        .s_axi_lite_arvalid(int_w_axi_lite_arvalid),
        .s_axi_lite_wdata  (int_w_axi_lite_wdata),
        .s_axi_lite_wstrb  (int_w_axi_lite_wstrb),
        .s_axi_lite_wready (int_w_axi_lite_wready),
        .s_axi_lite_wvalid (int_w_axi_lite_wvalid),
        .s_axi_lite_rdata  (int_w_axi_lite_rdata),
        .s_axi_lite_rresp  (int_w_axi_lite_rresp),
        .s_axi_lite_rready (int_w_axi_lite_rready),
        .s_axi_lite_rvalid (int_w_axi_lite_rvalid),
        .s_axi_lite_bresp  (int_w_axi_lite_bresp),
        .s_axi_lite_bready (int_w_axi_lite_bready),
        .s_axi_lite_bvalid (int_w_axi_lite_bvalid),

        .core_clk (m_axi_aclk),    // Module Clock  
        .core_rstn(m_axi_aresetn_shards[4]), // Active high core reset

        // Interface to TX Engine for arbitrated WQ entries
        // 用于仲裁 WQE 的 TX 引擎接口
        .o_wqe           (qpm_wqe_data),           // 输出给 TX 引擎的 SQ WQE 数据
        .i_wqe_pop       (qpm_wqe_pop),
        .o_wqe_empty     (qpm_wqe_fifo_empty),
        .o_wqe_fast_valid(qpm_wqe_fast_valid),
        .o_halt          (qp_mgr_o_halt),
        .o_halted_qpid   (qp_mgr_o_halted_qpid),
        .i_wqe_halted    (tx_engine_i_wqe_halted),

        // Interrupt signals coming from Header validation module
        // 来自包头验证模块的中断信号
        .i_pkt_valdn_err_intr(rx_engine_o_ib_pkt_hdr_valdn_err_intr_sts),
        .i_mad_pkt_rcvd_intr (rx_engine_o_mad_pkt_rcvd_intr_sts),
        .i_qp_fatal_err_valid(rx_engine_o_qp_fatal_wr_en),
        .i_qp_fatal_err_id   (rx_engine_o_qp_fatal_wr_idx),
        .i_qp_fatal_err_data (rx_engine_o_qp_fatal_wr_data),

        .o_intr_clr_pkt_valdn_err (qp_mgr_o_intr_clr_pkt_valdn_err),
        .o_intr_clr_mad_pkt_rcvd  (qp_mgr_o_intr_clr_mad_pkt_rcvd),
        .o_intr_clr_wqe_cmpl      (qp_mgr_o_intr_clr_wqe_cmpl),
        .o_intr_clr_fatal_err     (qp_mgr_o_intr_clr_fatal_err),
        .o_qp_clr_fatal_err_valid (clr_qp_fatal_err),
        .o_qp_clr_fatal_err_id    (clr_qp_fatal_idx),
        .o_qp_clr_fatal_err_data  (),
        .o_qp_under_recovery_valid(qp_under_recovery_wr_en),
        .o_qp_under_recovery_id   (qp_under_recovery_wr_idx),
        .o_qp_under_recovery_data (qp_under_recovery_wr_data),
        .o_intr_en_wqe_cmpl       (qp_mgr_o_intr_en_wqe_cmpl),
        .o_intr_en_pkt_valdn_err  (qp_mgr_o_intr_en_pkt_valdn_err),
        .o_intr_en_mad_pkt_rcvd   (qp_mgr_o_intr_en_mad_pkt_rcvd),
        .o_intr_en_fatal_err      (qp_mgr_o_intr_en_fatal_err),

        .i_qp_wq_cmpl_intr_or_reduce      (intr_wqe_cmpl_sts_or_reduce),
        .i_qp_wq_cmpl_intr_wr_en          (intr_wqe_cmpl_sts_wr_en),
        .i_qp_wq_cmpl_intr_wr_addr        (intr_wqe_cmpl_sts_wr_addr),
        .i_qp_wq_cmpl_intr_wr_data        (intr_wqe_cmpl_sts_wr_data),
        .i_qp_wq_cmpl_intr_wr_en_pending  (intr_wqe_cmpl_sts_wr_en_pending),
        .i_qp_wq_cmpl_intr_wr_addr_pending(intr_wqe_cmpl_sts_wr_addr_pending),
        //.o_cq_intr_sts_clr                (qp_mgr_o_cq_intr_sts_clr),
        .o_cq_intr_sts_clr_wr_en          (clr_wqe_cmpl_sts_set_valid),
        .o_cq_intr_sts_clr_wr_addr        (clr_wqe_cmpl_sts_set_id),

        // Timeout register：{无功能定义位, retry_cnt[2:0], 无功能定义位, timeout[4:0]}
        .o_qp_disable_pulse_valid(qp_disable_pulse_valid),
        .o_qp_disable_pulse_id   (qp_disable_pulse_id),
        .o_qp_disable_pulse_data (qp_disable_pulse_data),
        .i_qp_timeout_idx        (qp_mgr_qp_conf_idx),         // TIMEOUTCONFi QP index, aligned with the arbitrated QPCONF read index
        .o_qp_timeout            (qp_mgr_o_qp_timeout_per_qp), // TIMEOUTCONFi 配置值

        .o_in_errsts_q_ba   (qp_mgr_in_errsts_q_ba),
        .o_in_errsts_q_sz   (qp_mgr_in_errsts_q_sz),
        .i_in_errsts_q_wrptr(rx_engine_in_errsts_q_wrptr),

        // Status inputs from Header validation module
        .i_rq_full_valid         (rx_engine_o_rq_full_wr_en),        // Rcv Q full for QPn
        .i_rq_full_idx           (rx_engine_o_rq_full_idx),
        .i_rq_full_data          (rx_engine_o_rq_full_val),
        .i_rq_full_reset_valid   (rx_engine_o_rq_full_reset_wr_en),
        .i_rq_full_reset_idx     (rx_engine_o_rq_full_reset_idx),
        .i_rq_full_reset_data    (rx_engine_o_rq_full_reset_val),
        .i_osq_full_set_valid    (osq_full_set_valid),               // Outstanding Q full for QPn
        .i_osq_full_set_id       (osq_full_set_id),                  // Outstanding Q full for QPn
        .i_osq_full_reset_valid  (osq_full_reset_valid),             // Outstanding Q full for QPn
        .i_osq_full_reset_id     (osq_full_reset_id),                // Outstanding Q full for QPn
        .i_osq_empty_set_valid   (osq_empty_set_valid),              // Outstanding Q empty for QPn
        .i_osq_empty_set_id      (osq_empty_set_id),                 // Outstanding Q empty for QPn
        .i_osq_empty_reset_valid (osq_empty_reset_valid),            // Outstanding Q empty for QPn
        .i_osq_empty_reset_id    (osq_empty_reset_id),               // Outstanding Q empty for QPn
        .i_qp_retried_set_valid  (qp_retried_set_valid),             // QP retried for QPn
        .i_qp_retried_set_id     (qp_retried_set_id),
        .i_qp_retried_reset_valid(qp_retried_reset_valid),
        .i_qp_retried_reset_id   (qp_retried_reset_id),

        .i_qp_rq_pi_db_idx  (rx_engine_o_qp_req_conf_idx),      // STATRQPIDBi 的 QP 索引
        .i_qp_rq_pi_db      (rx_engine_o_rq_wrptr_db),          // 写回的 RQ producer index doorbell
        .o_qp_rq_pi_db      (qp_mgr_o_rq_pi_db),                // RQ producer index doorbell
        .i_qp_rq_pi_db_req  (rx_engine_o_qpm_req_rq_wrptr_db),
        .i_qp_rq_pi_db_wrn  (rx_engine_o_qpm_rw_rq_wrptr_db),
        .o_qp_rq_pi_db_valid(qp_mgr_o_qp_rq_pi_db_valid),

        // 重传前 SQ PSN 快照寄存器，供软件/调试读取
        .i_qp_stat_ret_sq_psn_idx  (rx_engine_o_qp_rsp_conf_idx),       // STATRET_SQ_PSN 的 QP 索引
        .o_qp_stat_ret_sq_psn      (qp_mgr_o_qp_stat_ret_sq_psn),
        .i_qp_stat_ret_sq_psn_req  (rx_engine_o_qp_rsp_conf_en),
        .o_qp_stat_ret_sq_psn_valid(qp_mgr_o_qp_stat_ret_sq_psn_valid),

        // Status input from other modules
        .i_tx_engine_sts (tx_engine_status_out),
        .i_ack_mgr_sts   (ack_mgr_o_ack_mgr_sts),
        .i_stat_retry_cnt(ack_mgr_o_stat_retry_count),


        // Configuration outputs going to other modules (generic)
        // 连接其他模块的配置输出（通用）
        .o_lrdma_en                          (qp_mgr_o_lrdma_en),                                                // LRDMA Configuration -- [0] LRDMA Enable
        .o_lrdma_adv_conf_retry_cnt_fatal_dis(qp_mgr_o_lrdma_adv_conf_retry_cnt_fatal_dis),
        .o_err_buf_en                        (qp_mgr_o_err_buf_en),
        .o_flow_credits                      (qp_mgr_o_flow_credits),
        .o_num_qp_en                         (qp_mgr_o_num_qp_en),                                               // Number of QPs enabled
        .o_mac_lrdma_addr                    (qp_mgr_o_mac_src_addr),                                            // 本地 LRDMA 以太网 MAC 地址
        .o_ipv4_lrdma_addr                   (qp_mgr_o_ipv4_src_addr),                                           // IPv4 destination (own) address
        .o_ipv6_lrdma_addr                   (qp_mgr_o_ipv6_src_addr),                                           // IPv6 destination (own) address
        .o_rq_err_pkt_buf_ba                 (qp_mgr_o_err_pkt_buf_ba),
        .o_rq_err_pkt_buf_sz_num_bufs        (qp_mgr_o_err_pkt_buf_sz_num_bufs),
        .o_rq_err_pkt_buf_sz_buf_sz          (qp_mgr_o_err_pkt_buf_sz_buf_sz),
        .i_rq_err_pkt_buf_wrptr              (rx_engine_o_err_pkt_buf_wrptr),
        .i_inc_ack_pkt_cnt                   (rx_engine_o_ib_ack_pkt_cnt),
        .i_inc_mad_pkt_cnt                   (rx_engine_o_ib_qp1_pkt_cnt),
        .i_inc_inv_pkt_cnt                   (rx_engine_o_ib_inv_pkt_cnt),
        .i_inc_all_dropped_cnt               ({rx_engine_o_pkt_all_cnt[15:0], rx_engine_o_pkt_disc_cnt[15:0]}),
        .i_inc_nack_cnt                      (rx_engine_o_inc_nack_cnt),
        .o_qp_enable_valid                   (qp_enable_wr_en),
        .o_qp_enable_id                      (qp_enable_wr_addr),
        .o_qp_enable_data                    (qp_enable_wr_data),
        .o_lrdma_udp_src_port                (qp_mgr_o_lrdma_udp_src_port),

        .pfc_en_roce             (pfc_en_roce),
        .pfc_en_non_roce         (pfc_en_non_roce),
        .roce_priority           (roce_priority),
        .non_roce_priority       (non_roce_priority),
        .roce_pause_on_threshold (roce_pause_on_threshold),
        .roce_pause_off_threshold(roce_pause_off_threshold),
        .disable_priority_chk    (disable_priority_chk),

        .i_out_wr_pkt_cnt (wqe_out_wr_pkt_cnt),
        .i_out_mad_pkt_cnt(wqe_out_mad_pkt_cnt),

        .i_qp_cfg_retry_cnt_rd_req(ack_mgr_o_qp_cfg_retry_cnt_rd_req),
        //.i_qp_cfg_retry_cnt_qpid          (), //already connected
        .o_qp_cfg_retry_cnt_rd_rdy(ack_mgr_i_qp_cfg_retry_cnt_rd_rdy),

        //Configuration outputs going to other modules (PER QP)
        .i_qp_conf_req_ack_mgr    (ack_mgr_o_qp_conf_req),
        .i_qp_conf_req_tx_engine  (tx_engine_o_reg_rd_en_s),
        .i_qp_conf_req_rx_engine  (rx_engine_o_qp_rsp_conf_en),
        .o_qp_conf_valid_ack_mgr  (qp_mgr_o_qp_conf_valid_ack_mgr),
        .o_qp_conf_valid_tx_engine(qp_mgr_o_qp_conf_valid_tx_engine),
        .o_qp_conf_valid_rx_engine(qp_mgr_o_qp_conf_valid_rx_engine),
        .o_qp_conf                (qp_mgr_o_qp_conf),                  // QP configuration
        .i_qp_conf_idx            (qp_mgr_qp_conf_idx),                // QPCONFi QP index

        .o_qp_conf_replica      (qp_mgr_o_qp_conf_replica),       // QP configuration REPLICA
        .i_qp_conf_replica_idx  (rx_engine_o_qp_req_conf_idx),    // 读取 QPCONFi 复制寄存器的 QP 索引
        .i_qp_conf_replica_req  (rx_engine_o_qp_req_conf_en),
        .o_qp_conf_replica_valid(qp_mgr_o_qp_conf_replica_valid),

        .i_qp_adv_conf_req  (tx_engine_o_reg_rd_en),       // TX 引擎发起 QPADVCONFi 读取请求
        .o_qp_adv_conf_valid(qp_mgr_o_qp_adv_conf_valid),  // QPADVCONFi 读取数据有效
        .o_qp_adv_conf      (qp_mgr_o_qp_adv_conf),        // QP advance configuration
        .i_qp_adv_conf_idx  (tx_engine_o_qp_id),           // TX 引擎读取 QPADVCONFi 的 QP 索引

        .i_qp_rq_ba_req  (rx_engine_o_qpm_req_rq_buf_ba),
        .o_qp_rq_ba_valid(qp_mgr_o_qp_rq_ba_valid),
        .o_qp_rq_ba      (qp_mgr_o_qp_rq_ba),
        .i_qp_rq_ba_idx  (rx_engine_o_qp_req_conf_idx),    // 读取 RQ 缓冲区基地址的 QP 索引

        .i_qp_cq_ba_req  (ack_mgr_o_qp_cq_ba_req),
        .o_qp_cq_ba_valid(qp_mgr_o_qp_cq_ba_valid),
        .o_qp_cq_ba      (qp_mgr_o_qp_cq_ba),
        .i_qp_cq_ba_idx  (ack_mgr_o_qp_cqe_proc_qpid), // 读取 CQ 基地址的 QP 索引

        // rq_wrptr_db -- RQWPTRDBADDi 寄存器指向 rq_wrptr_db 的地址。
        // RQWPTRDBADDi 寄存器:
        //      该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。
        //      该寄存器提供接收队列门铃寄存器的地址。
        //      在接收到 MAD（QP1 SEND_ONLY）数据包消耗 1 个 RQ WQE 时，LRDMA IP 会更新该寄存器所指向地址中的 RQ 门铃值。
        .i_qp_rq_wrptrdb_add_req  (rx_engine_o_qpm_req_rq_wrptr_db_add),
        .o_qp_rq_wrptrdb_add_valid(qp_mgr_o_qp_rq_wrptrdb_add_valid),
        .o_qp_rq_wrptrdb_add      (qp_mgr_o_qp_rq_wrptrdb_add),
        .i_qp_rq_wrptrdb_add_idx  (rx_engine_o_qp_req_conf_idx),          // 读取 RQ 写门铃地址的 QP 索引

        .i_qp_cq_hdptr_req  (ack_mgr_o_qp_cq_hdptr_req),
        .o_qp_cq_hdptr_valid(qp_mgr_o_qp_cq_hdptr_valid),
        .o_qp_cq_hdptr      (qp_mgr_o_qp_cq_hdptr),
        .i_qp_cq_hdptr_idx  (ack_mgr_o_qp_cqe_proc_qpid),  // 读取 CQ head pointer 的 QP 索引
        .i_qp_cq_hdptr      (ack_mgr_o_qp_cq_hdptr),
        .i_qp_cq_hdptr_wrn  (ack_mgr_o_qp_cq_hdptr_wrn),

        .i_qp_rq_cidb_req  (rx_engine_o_qpm_req_rq_ci_db),
        .o_qp_rq_cidb_valid(qp_mgr_o_qp_rq_cidb_valid),
        .o_qp_rq_cidb      (qp_mgr_o_qp_rq_cidb),
        .i_qp_rq_cidb_idx  (rx_engine_o_qp_req_conf_idx),   // 读取 RQ CI doorbell 的 QP 索引

        .i_qp_rq_depth_req  (rx_engine_o_qpm_req_q_depth),
        .o_qp_rq_depth_valid(qp_mgr_o_qp_rq_depth_valid),
        .o_qp_rq_depth      (qp_mgr_o_qp_rq_depth),
        .i_qp_rq_depth_idx  (rx_engine_o_qp_req_conf_idx),  // RX 请求/MAD 路径读取 RQ 深度的 QP 索引

        .i_qp_sq_psn_req       (rx_engine_o_qpm_req_sq_psn),                       // RX 响应路径发起 SQPSNi 读取请求
        .i_qp_sq_psn_wqe_req   (tx_engine_o_reg_rd_en_s | tx_engine_o_reg_wr_en),  // TX WQE 路径发起 SQPSNi 读写请求
        .i_qp_sq_psn_resp_req  (ack_mgr_o_qp_sq_psn_resp_req),                     // ACK 管理器发起 SQPSNi 读取请求
        .o_qp_sq_psn_valid     (qp_mgr_o_qp_sq_psn_valid),                         // RX 响应路径 SQPSNi 读取数据有效
        .o_qp_sq_psn_wqe_valid (qp_mgr_o_qp_sq_psn_wqe_valid),                     // TX WQE 路径 SQPSNi 读取数据有效
        .o_qp_sq_psn_resp_valid(qp_mgr_o_qp_sq_psn_resp_valid),                    // ACK 管理器 SQPSNi 读取数据有效
        .o_qp_sq_psn           (qp_mgr_o_qp_sq_psn),
        .i_qp_sq_psn_wrn       (tx_engine_o_reg_wr_en),
        .i_qp_sq_psn           (tx_engine_reg_sq_psn),
        .i_qp_sq_psn_idx       (rx_engine_o_qp_rsp_conf_idx),                      // RX response 路径访问 SQPSNi 的 QP 索引
        .i_qp_sq_psn_wqe_idx   (tx_engine_o_qp_id),                                // TX WQE 路径访问 SQPSNi 的 QP 索引
        .i_qp_sq_psn_resp_idx  (ack_mgr_o_qp_sq_psn_resp_idx),                     // ACK 管理器响应完成检查访问 SQPSNi 的 QP 索引

        .o_qp_last_req      (qp_mgr_o_qp_last_req),
        .i_qp_last_req_idx  (rx_engine_o_qp_req_conf_idx),   // RX 请求/MAD 路径读写 LASTREQi 的 QP 索引
        .i_qp_last_req      (rx_engine_o_last_req),
        .i_qp_last_req_req  (rx_engine_o_qpm_req_last_req),
        .i_qp_last_req_wrn  (rx_engine_o_qpm_rw_last_req),
        .o_qp_last_req_valid(qp_mgr_o_qp_last_req_valid),

        .i_qp_dest_qpid_req_tx_engine  (tx_engine_o_reg_rd_en),                  // req from tx engine
        .i_qp_dest_qpid_req_rx_engine  (1'b0),                                   // req from rx engine
        .o_qp_dest_qpid_valid_tx_engine(qp_mgr_o_qp_dest_qpid_valid_tx_engine),
        .o_qp_dest_qpid_valid_rx_engine(qp_mgr_o_qp_dest_qpid_valid_rx_engine),
        .o_qp_dest_qpid                (qp_mgr_o_qp_dest_qpid),
        .i_qp_dest_qpid_idx            (tx_engine_o_qp_id),                      // 读取目标 QPID 的 QP 索引

        .i_qp_stat_resp_psn      (rx_engine_o_qp_stat_resp_psn),
        .i_qp_stat_resp_psn_wen  (rx_engine_o_qpm_rw_stat_resp_psn),
        .i_qp_stat_resp_psn_idx  (rx_engine_o_qp_rsp_conf_idx),
        .o_qp_stat_resp_psn_valid(qp_mgr_o_qp_stat_resp_psn_valid),
        .i_qp_stat_resp_psn_rdreq(rx_engine_o_qpm_req_stat_resp_psn),
        .o_qp_stat_resp_psn      (qp_mgr_o_qp_stat_resp_psn),

        .i_qp_stat_rq_buf_ca      (rx_engine_o_rq_buf_ca),
        .i_qp_stat_rq_buf_ca_wen  (rx_engine_o_qpm_rw_rq_buf_ca),
        .i_qp_stat_rq_buf_ca_idx  (rx_engine_o_qp_req_conf_idx),
        .o_qp_stat_rq_buf_ca_valid(qp_mgr_o_qp_stat_rq_buf_ca_valid),
        .i_qp_stat_rq_buf_ca_rdreq(rx_engine_o_qpm_req_rq_buf_ca),
        .o_qp_stat_rq_buf_ca      (qp_mgr_o_qp_stat_rq_buf_ca),

        .i_qp_stat_ssn_idx  (tx_engine_o_qp_id),
        .i_qp_stat_ssn      (tx_engine_reg_sq_ssn),
        .o_qp_stat_ssn      (qp_mgr_o_qp_sq_ssn),
        .i_qp_stat_ssn_req  (tx_engine_o_reg_rd_en | tx_engine_o_reg_wr_en),
        .i_qp_stat_ssn_wrn  (tx_engine_o_reg_wr_en),
        .o_qp_stat_ssn_valid(qp_mgr_o_qp_stat_ssn_valid),

        .i_waiting_for_rq_update         (rx_engine_waiting_for_rq_update),
        .i_waiting_for_rq_update_on_q_num(rx_engine_waiting_for_rq_update_on_qp_num),

        // Status NAK details from RX Engine
        .i_qp_stat_nak      (rx_engine_o_stat_nak),
        .i_qp_stat_nak_wen  (rx_engine_o_qpm_rw_stat_nak),
        .i_qp_stat_nak_idx  (rx_engine_o_qp_rsp_conf_idx),
        .o_qp_stat_nak_valid(ack_mgr_o_qp_stat_nak_valid),
        .i_qp_stat_nak_rdreq(rx_engine_o_qpm_req_stat_nak),
        .o_qp_stat_nak      (ack_mgr_o_qp_stat_nak),         // Status QPi(STATQPi)寄存器[22:16]

        // 来自 ACK 管理器的 NAK 状态信息
        .i_qp_stat_retry_cnt      (ack_mgr_o_qp_retry_cnt_new),
        .i_qp_stat_retry_cnt_wen  (ack_mgr_o_qp_retry_cnt_wen),
        .i_qp_stat_retry_cnt_idx  (ack_mgr_o_qp_retry_cnt_qpid),
        .o_qp_stat_retry_cnt_valid(ack_mgr_i_qp_retry_cnt_rdy),
        .i_qp_stat_retry_cnt_rdreq(ack_mgr_o_qp_retry_cnt_req),
        .o_qp_stat_retry_cnt      (ack_mgr_i_qp_retry_cnt_curr),  // Status QPi(STATQPi)寄存器[26:24]

        .i_qp_mac_remote_addrl_req      (rx_engine_rsp_remote_qp_mac_addr_en),
        .o_qp_mac_remote_addrl_valid    (qp_mgr_o_qp_mac_remote_addrl_valid),
        .i_qp_mac_remote_addrl_wqe_req  (tx_engine_o_reg_rd_en_s),
        .o_qp_mac_remote_addrl_wqe_valid(qp_mgr_o_qp_mac_dest_addrl_valid),
        .o_qp_mac_remote_addrl          (qp_mgr_o_qp_mac_dest_addrl),
        .i_qp_mac_remote_addrl_idx      (rx_engine_o_qp_rsp_conf_idx),          // RX 响应路径读取远端 MAC LSB 的 QP 索引
        .i_qp_mac_remote_addrl_wqe_idx  (tx_engine_o_qp_id),                    // TX WQE 路径读取远端 MAC LSB 的 QP 索引

        .i_qp_mac_remote_addrl_replica_req  (rx_engine_req_remote_qp_mac_addr_en),
        .o_qp_mac_remote_addrl_replica      (qp_mgr_o_qp_mac_remote_addrl_replica),
        .i_qp_mac_remote_addrl_replica_idx  (rx_engine_o_qp_req_conf_idx),
        .o_qp_mac_remote_addrl_replica_valid(),

        .i_qp_mac_remote_addrm_req      (rx_engine_rsp_remote_qp_mac_addr_en),
        .o_qp_mac_remote_addrm_valid    (),
        .i_qp_mac_remote_addrm_wqe_req  (tx_engine_o_reg_rd_en_s),
        .o_qp_mac_remote_addrm_wqe_valid(qp_mgr_o_qp_mac_dest_addrm_valid),
        .o_qp_mac_remote_addrm          (qp_mgr_o_qp_mac_dest_addrm),
        .i_qp_mac_remote_addrm_idx      (rx_engine_o_qp_rsp_conf_idx),          // RX 响应路径读取远端 MAC MSB 的 QP 索引
        .i_qp_mac_remote_addrm_wqe_idx  (tx_engine_o_qp_id),                    // TX WQE 路径读取远端 MAC MSB 的 QP 索引

        .o_qp_mac_remote_addrm_replica      (qp_mgr_o_qp_mac_remote_addrm_replica),
        .i_qp_mac_remote_addrm_replica_idx  (rx_engine_o_qp_req_conf_idx),
        .i_qp_mac_remote_addrm_replica_req  (rx_engine_req_remote_qp_mac_addr_en),
        .o_qp_mac_remote_addrm_replica_valid(),

        .i_qp_ip_remote_addr1_req      (rx_engine_rsp_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr1_valid    (qp_mgr_o_qp_ip_remote_addr1_valid),
        .i_qp_ip_remote_addr1_wqe_req  (tx_engine_o_reg_rd_en_s),
        .o_qp_ip_remote_addr1_wqe_valid(qp_mgr_o_qp_ip_dest_addr1_valid),
        .o_qp_ip_remote_addr1          (qp_mgr_o_qp_ip_dest_addr1),
        .i_qp_ip_remote_addr1_idx      (rx_engine_o_qp_rsp_conf_idx),         // RX 响应路径读取 IPDESADDR1i 的 QP 索引
        .i_qp_ip_remote_addr1_wqe_idx  (tx_engine_o_qp_id),                   // TX WQE 路径读取 IPDESADDR1i 的 QP 索引

        .o_qp_ip_remote_addr1_replica      (qp_mgr_o_qp_ip_remote_addr1_replica),
        .i_qp_ip_remote_addr1_replica_idx  (rx_engine_o_qp_req_conf_idx),
        .i_qp_ip_remote_addr1_replica_req  (rx_engine_req_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr1_replica_valid(),

        .i_qp_ip_remote_addr2_req      (rx_engine_rsp_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr2_valid    (),
        .i_qp_ip_remote_addr2_wqe_req  (tx_engine_o_reg_rd_en_s),
        .o_qp_ip_remote_addr2_wqe_valid(qp_mgr_o_qp_ip_dest_addr2_valid),
        .o_qp_ip_remote_addr2          (qp_mgr_o_qp_ip_dest_addr2),
        .i_qp_ip_remote_addr2_idx      (rx_engine_o_qp_rsp_conf_idx),         // RX 响应路径读取 IPDESADDR2i 的 QP 索引
        .i_qp_ip_remote_addr2_wqe_idx  (tx_engine_o_qp_id),                   // TX WQE 路径读取 IPDESADDR2i 的 QP 索引

        .o_qp_ip_remote_addr2_replica      (qp_mgr_o_qp_ip_remote_addr2_replica),
        .i_qp_ip_remote_addr2_replica_idx  (rx_engine_o_qp_req_conf_idx),
        .i_qp_ip_remote_addr2_replica_req  (rx_engine_req_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr2_replica_valid(),

        .i_qp_ip_remote_addr3_req      (rx_engine_rsp_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr3_valid    (),
        .i_qp_ip_remote_addr3_wqe_req  (tx_engine_o_reg_rd_en_s),
        .o_qp_ip_remote_addr3_wqe_valid(qp_mgr_o_qp_ip_dest_addr3_valid),
        .o_qp_ip_remote_addr3          (qp_mgr_o_qp_ip_dest_addr3),
        .i_qp_ip_remote_addr3_idx      (rx_engine_o_qp_rsp_conf_idx),         // RX 响应路径读取 IPDESADDR3i 的 QP 索引
        .i_qp_ip_remote_addr3_wqe_idx  (tx_engine_o_qp_id),                   // TX WQE 路径读取 IPDESADDR3i 的 QP 索引

        .o_qp_ip_remote_addr3_replica      (qp_mgr_o_qp_ip_remote_addr3_replica),
        .i_qp_ip_remote_addr3_replica_idx  (rx_engine_o_qp_req_conf_idx),
        .i_qp_ip_remote_addr3_replica_req  (rx_engine_req_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr3_replica_valid(),

        .i_qp_ip_remote_addr4_req      (rx_engine_rsp_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr4_valid    (),
        .i_qp_ip_remote_addr4_wqe_req  (tx_engine_o_reg_rd_en_s),
        .o_qp_ip_remote_addr4_wqe_valid(qp_mgr_o_qp_ip_dest_addr4_valid),
        .o_qp_ip_remote_addr4          (qp_mgr_o_qp_ip_dest_addr4),
        .i_qp_ip_remote_addr4_idx      (rx_engine_o_qp_rsp_conf_idx),         // RX 响应路径读取 IPDESADDR4i 的 QP 索引
        .i_qp_ip_remote_addr4_wqe_idx  (tx_engine_o_qp_id),                   // TX WQE 路径读取 IPDESADDR4i 的 QP 索引

        .o_qp_ip_remote_addr4_replica      (qp_mgr_o_qp_ip_remote_addr4_replica),
        .i_qp_ip_remote_addr4_replica_idx  (rx_engine_o_qp_req_conf_idx),
        .i_qp_ip_remote_addr4_replica_req  (rx_engine_req_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr4_replica_valid(),

        .i_qp_cq_depth_req  (ack_mgr_o_qp_cq_depth_req),
        .o_qp_cq_depth_valid(qp_mgr_o_qp_cq_depth_valid),
        .o_qp_cq_depth      (qp_mgr_o_qp_cq_depth),
        .i_qp_cq_depth_idx  (ack_mgr_o_qp_cqe_proc_qpid),

        .i_last_in_pkt_info ({1'b0, {(11 - C_QP_INDX_WIDTH) {1'b0}}, rx_engine_o_last_ib_pkt_dest_qpid, rx_engine_o_last_ib_pkt_psn[11:0], rx_engine_o_last_ib_pkt_opcode}),
        .i_last_out_pkt_info(last_out_pkt_info),

        .i_retransmit_qpid        (ack_mgr_o_retransmit_qpid),          // 重传路径选中的 QP id
        .i_retransmit_reqd        (ack_mgr_o_retransmit_reqd),          // Retransmission is required
        .i_retransmit_wqe_bram_din(ack_mgr_o_retransmit_wqe_bram_din),
        .o_retransmit_accepted    (qp_mgr_o_retransmit_accepted),       // QP 管理器已接收重传请求
        .i_psn_to_retry           (ack_mgr_o_psn_to_retry),
        .i_ssn_to_retry           (ack_mgr_o_ssn_to_retry),

        // AXI Master signals
        .m_axi_awid   (m_axi_qp_get_wqe_awid),
        .m_axi_awaddr (m_axi_qp_get_wqe_awaddr),
        .m_axi_awlen  (m_axi_qp_get_wqe_awlen),
        .m_axi_awsize (m_axi_qp_get_wqe_awsize),
        .m_axi_awburst(m_axi_qp_get_wqe_awburst),
        .m_axi_awcache(m_axi_qp_get_wqe_awcache),
        .m_axi_awprot (m_axi_qp_get_wqe_awprot),
        .m_axi_awvalid(m_axi_qp_get_wqe_awvalid),
        .m_axi_awready(m_axi_qp_get_wqe_awready),

        .m_axi_wdata (m_axi_qp_get_wqe_wdata),
        .m_axi_wstrb (m_axi_qp_get_wqe_wstrb),
        .m_axi_wlast (m_axi_qp_get_wqe_wlast),
        .m_axi_wvalid(m_axi_qp_get_wqe_wvalid),
        .m_axi_wready(m_axi_qp_get_wqe_wready),
        .m_axi_awlock(m_axi_qp_get_wqe_awlock),

        .m_axi_bid   (m_axi_qp_get_wqe_bid),
        .m_axi_bresp (m_axi_qp_get_wqe_bresp),
        .m_axi_bvalid(m_axi_qp_get_wqe_bvalid),
        .m_axi_bready(m_axi_qp_get_wqe_bready),

        .m_axi_arid   (m_axi_qp_get_wqe_arid),
        .m_axi_araddr (m_axi_qp_get_wqe_araddr),
        .m_axi_arlen  (m_axi_qp_get_wqe_arlen),
        .m_axi_arsize (m_axi_qp_get_wqe_arsize),
        .m_axi_arburst(m_axi_qp_get_wqe_arburst),
        .m_axi_arcache(m_axi_qp_get_wqe_arcache),
        .m_axi_arprot (m_axi_qp_get_wqe_arprot),
        .m_axi_arvalid(m_axi_qp_get_wqe_arvalid),
        .m_axi_arready(m_axi_qp_get_wqe_arready),

        .m_axi_rid   (m_axi_qp_get_wqe_rid),
        .m_axi_rdata (m_axi_qp_get_wqe_rdata),
        .m_axi_rresp (m_axi_qp_get_wqe_rresp),
        .m_axi_rlast (m_axi_qp_get_wqe_rlast),
        .m_axi_rvalid(m_axi_qp_get_wqe_rvalid),
        .m_axi_rready(m_axi_qp_get_wqe_rready),
        .m_axi_arlock(m_axi_qp_get_wqe_arlock),

        .i_osq_pop         (ack_mgr_osq_pop),
        .i_osq_pop_qpid    (ack_mgr_osq_pop_qpid),
        .i_wqe_dropped     (wqe_dropped),
        .i_wqe_dropped_qpid(wqe_dropped_qpid),
        .init_start        (init_start),
        .init_count        (init_count)

    );



    lrdma_tx_engine_top #(
        .C_MAX_QP(C_NUM_QP),
        .C_MAX_QID_WIDTH(C_QP_INDX_WIDTH),
        .C_OS_Q_INDX_WIDTH(OS_Q_INDX_WIDTH),
        .C_M_AXI_DATA_WIDTH(512),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_ID_WIDTH(C_M_AXI_ID_WIDTH),
        .C_MAX_SGL_DEPTH(C_MAX_SGL_DEPTH),
        .C_ENABLE_TX_BUF_MGR_EARLY_RELEASE(C_ENABLE_TX_BUF_MGR_EARLY_RELEASE),
        .C_ENABLE_FAST_WQE_CONSUME(C_ENABLE_FAST_WQE_CONSUME)
    ) inst_tx_engine (
        .core_clk(m_axi_aclk),
        .core_rst(~m_axi_aresetn_shards[5]),

        .m_axi_awid   (m_axi_pktgen_get_payload_awid),
        .m_axi_awaddr (m_axi_pktgen_get_payload_awaddr),
        .m_axi_awlen  (m_axi_pktgen_get_payload_awlen),
        .m_axi_awsize (m_axi_pktgen_get_payload_awsize),
        .m_axi_awburst(m_axi_pktgen_get_payload_awburst),
        .m_axi_awcache(m_axi_pktgen_get_payload_awcache),
        .m_axi_awprot (m_axi_pktgen_get_payload_awprot),
        .m_axi_awvalid(m_axi_pktgen_get_payload_awvalid),
        .m_axi_awready(m_axi_pktgen_get_payload_awready),
        .m_axi_wdata  (m_axi_pktgen_get_payload_wdata),
        .m_axi_wstrb  (m_axi_pktgen_get_payload_wstrb),
        .m_axi_wlast  (m_axi_pktgen_get_payload_wlast),
        .m_axi_wvalid (m_axi_pktgen_get_payload_wvalid),
        .m_axi_wready (m_axi_pktgen_get_payload_wready),
        .m_axi_awlock (m_axi_pktgen_get_payload_awlock),
        .m_axi_bid    (m_axi_pktgen_get_payload_bid),
        .m_axi_bresp  (m_axi_pktgen_get_payload_bresp),
        .m_axi_bvalid (m_axi_pktgen_get_payload_bvalid),
        .m_axi_bready (m_axi_pktgen_get_payload_bready),
        .m_axi_arid   (m_axi_pktgen_get_payload_arid),
        .m_axi_araddr (m_axi_pktgen_get_payload_araddr),
        .m_axi_arlen  (m_axi_pktgen_get_payload_arlen),
        .m_axi_arsize (m_axi_pktgen_get_payload_arsize),
        .m_axi_arburst(m_axi_pktgen_get_payload_arburst),
        .m_axi_arcache(m_axi_pktgen_get_payload_arcache),
        .m_axi_arprot (m_axi_pktgen_get_payload_arprot),
        .m_axi_arvalid(m_axi_pktgen_get_payload_arvalid),
        .m_axi_arready(m_axi_pktgen_get_payload_arready),
        .m_axi_rid    (m_axi_pktgen_get_payload_rid),
        .m_axi_rdata  (m_axi_pktgen_get_payload_rdata),
        .m_axi_rresp  (m_axi_pktgen_get_payload_rresp),
        .m_axi_rlast  (m_axi_pktgen_get_payload_rlast),
        .m_axi_rvalid (m_axi_pktgen_get_payload_rvalid),
        .m_axi_rready (m_axi_pktgen_get_payload_rready),
        .m_axi_arlock (m_axi_pktgen_get_payload_arlock),

        .m_axis_tdata (tx_engine_m_axis_tdata),
        .m_axis_tkeep (tx_engine_m_axis_tkeep),
        .m_axis_tvalid(tx_engine_m_axis_tvalid),
        .m_axis_tready(tx_engine_m_axis_tready),
        .m_axis_tlast (tx_engine_m_axis_tlast),

        .i_qpm_fifo_empty                 (qpm_wqe_fifo_empty),
        .i_wqe_halt                       (qp_mgr_o_halt),
        .i_wqe_halted_qpid                (qp_mgr_o_halted_qpid),
        .o_wqe_halted                     (tx_engine_i_wqe_halted),
        .i_qpm_wqe_data                   (qpm_wqe_data),
        .i_qpm_wqe_fast_valid             (qpm_wqe_fast_valid),
        .o_qpm_wqe_pop                    (qpm_wqe_pop),
        .o_reg_rd_en                      (tx_engine_o_reg_rd_en),
        .o_reg_rd_en_s                    (tx_engine_o_reg_rd_en_s),
        .i_reg_rdy                        (tx_engine_qp_info_ready),
        .i_reg_psn_val                    (qp_mgr_o_qp_sq_psn_wqe_valid),
        //.i_reg_mac_addr_rdy(qp_mgr_o_qp_mac_dest_addrm_valid),                                                      
        .o_reg_wr_en                      (tx_engine_o_reg_wr_en),
        .o_qp_id                          (tx_engine_o_qp_id),
        .i_reg_pkey                       (qp_mgr_o_qp_adv_conf[31:16]),
        .i_reg_pmtu                       (qp_mgr_o_qp_conf[10:8]),                                                                                        // use relevant qp_conf field
        .i_reg_dest_qpid                  (qp_mgr_o_qp_dest_qpid),                                                                                         // pls add ready and connect ~rx_engine_o_qpm_req_dest_qp_conf
        .i_reg_sq_psn                     (qp_mgr_o_qp_sq_psn),
        .o_reg_sq_psn                     (tx_engine_reg_sq_psn),
        .i_reg_sq_msn                     (qp_mgr_o_qp_sq_ssn),
        .o_reg_sq_msn                     (tx_engine_reg_sq_ssn),
        .i_reg_mac_dest_addr              ({qp_mgr_o_qp_mac_dest_addrm[15:0], qp_mgr_o_qp_mac_dest_addrl}),
        .i_reg_ip_dest_addr               ({qp_mgr_o_qp_ip_dest_addr4, qp_mgr_o_qp_ip_dest_addr3, qp_mgr_o_qp_ip_dest_addr2, qp_mgr_o_qp_ip_dest_addr1}),
        .i_reg_udp_src_port               (qp_mgr_o_lrdma_udp_src_port),                                                                                   // this is a static value for RoCE
        .i_reg_qp_to                      (qp_mgr_o_qp_timeout_per_qp[4:0]),
        .i_reg_ttl                        (qp_mgr_o_qp_adv_conf[15:8]),
        .i_reg_dscp                       (qp_mgr_o_qp_adv_conf[5:0]),
        .i_reg_mac_src_addr               (qp_mgr_o_mac_src_addr),
        .i_reg_ipv4_src_addr              (qp_mgr_o_ipv4_src_addr),                                                                                        // need ipv4/ipv6
        .i_reg_ipv6_src_addr              (qp_mgr_o_ipv6_src_addr),
        .i_reg_lrdma_en                   (qp_mgr_o_lrdma_en),
        .i_reg_ip_ver                     (qp_mgr_o_qp_conf[7]),
        .o_wqp_req                        (tx_engine_os_wqp_req),
        .i_wqp_resp                       (tx_engine_os_wqp_rdy),
        .o_wqe_qpid                       (tx_engine_os_wqe_qpid),
        .o_wqe_opcode                     (tx_engine_os_wqe_opcode),
        .o_wqe_qp_to                      (tx_engine_o_wqe_qp_to),
        .o_wqe_wrid                       (tx_engine_os_wqe_wrid),
        .o_wqe_retried                    (tx_engine_os_wqe_retried),
        .o_wqe_fatal_push                 (tx_engine_os_wqe_fatal_push),
        .o_wqe_send_psn                   (tx_engine_os_wqe_send_psn),
        .o_wqe_send_first_psn             (tx_engine_os_wqe_send_first_psn),
        .o_wqe_send_msn                   (tx_engine_os_wqe_send_msn),
        .o_wqe_explicit_ack_req           (tx_engine_os_wqe_explicit_ack_req),
        .o_wqe_status_out                 (tx_engine_status_out),
        .o_last_out_pkt_info              (last_out_pkt_info),
        .o_out_wr_pkt_cnt                 (wqe_out_wr_pkt_cnt),
        .o_out_mad_pkt_cnt                (wqe_out_mad_pkt_cnt),
        .i_qp_recovery                    (qp_mgr_o_qp_conf[6]),
        .o_dma_in_idle                    (tx_engine_dma_in_idle),
        .o_wqe_fatal_qpid                 (wqe_fatal_qpid),
        .o_wqe_fatal_qp_vld               (wqe_fatal_qp_vld),
        .i_wqe_fatal_qp_rdy               (wqe_fatal_qp_rdy),
        .i_osq_pop                        (ack_mgr_osq_pop),
        .o_wqe_dropped                    (wqe_dropped),
        .o_wqe_dropped_qpid               (wqe_dropped_qpid),
        .i_qp_enable_idx                  (qp_enable_wr_addr),
        .i_qp_enable_vld                  (qp_enable_wr_en),
        .i_qp_enable_data                 (qp_enable_wr_data),
        .i_qp_fatal_idx                   (rx_engine_o_qp_fatal_wr_idx),
        .i_qp_fatal_vld                   (rx_engine_o_qp_fatal_wr_en),
        .i_qp_fatal_data                  (rx_engine_o_qp_fatal_wr_data),
        .i_osq_nacked                     (o_osq_nacked),
        .o_osq_nacked_resp_wr_en          (osq_nacked_resp_wr_en),
        .o_osq_nacked_resp_wr_addr        (osq_nacked_resp_wr_addr),
        .o_osq_nacked_resp_wr_data        (osq_nacked_resp_wr_data),
        .o_osq_nacked_resp_wr_en_aux      (osq_nacked_resp_wr_en_aux),
        .o_osq_nacked_resp_wr_addr_aux    (osq_nacked_resp_wr_addr_aux),
        .o_osq_nacked_resp_wr_data_aux    (osq_nacked_resp_wr_data_aux),
        .o_osq_nacked_resp_wr_en_pending  (osq_nacked_resp_wr_en_pending),
        .o_osq_nacked_resp_wr_addr_pending(osq_nacked_resp_wr_addr_pending),
        .o_osq_nacked_resp_wr_data_pending(osq_nacked_resp_wr_data_pending),
        .init_start                       (init_start),
        .init_count                       (init_count),
        .i_qp_under_recovery_wr_en        (qp_under_recovery_wr_en),
        .i_qp_under_recovery_wr_id        (qp_under_recovery_wr_idx),
        .i_qp_under_recovery_wr_data      (qp_under_recovery_wr_data),
        // 这些端口固定接 0，不参与有效路径传输
        .i_osq_pop_qpid                   (ack_mgr_osq_pop_qpid)
    );

    /**********************************************************/
    //All lut Ram contents has to be re-initialized manually when a sudden hard reset is encountered
    //当遇到突然的硬重置时，所有lut Ram内容都必须手动重新初始化。
    always @(posedge m_axi_aclk) begin : global_lut_ram_init_process
        core_rst_n_r <= m_axi_aresetn;
        if (~core_rst_n_r) begin
            init_start <= 1'b1;
        end else if (init_count == C_NUM_QP - 1) begin
            init_start <= 1'b0;
        end

        if (!m_axi_aresetn) begin
            init_count <= 0;
        end else if (init_start && init_count != C_NUM_QP - 1) begin  //sudden hard reset -- 突然硬重启
            init_count <= init_count + 1;
        end else if (init_start && init_count == C_NUM_QP - 1) begin
            init_count <= init_count;
        end else begin
            init_count <= {C_QP_INDX_WIDTH{1'b0}};
        end

    end


    assign compat_resetn_done = m_axi_aresetn && !init_start;
    wire [4:0] compat_rx_qp;
    wire compat_enable,compat_psn_load;
    wire [47:0] compat_local_mac,compat_remote_mac;
    wire [31:0] compat_local_ip,compat_remote_ip,compat_rkey,compat_access,compat_len,compat_qp_conf;
    wire [63:0] compat_va,compat_pa,compat_db;
    wire [23:0] compat_remote_qpn,compat_last_psn,compat_psn_value;
    wire [15:0] compat_pkey;
    wire [4:0] compat_psn_qp;
    wire [511:0] credit_tx_data;
    wire [63:0] credit_tx_keep;
    wire credit_tx_valid,credit_tx_ready;
    wire [31:0] credit_committed,credit_dropped,credit_duplicates,credit_write_errors;
    lrdma_compat_config compat_config (
      .clk(m_axi_aclk),.rstn(m_axi_aresetn_shards[6]),
      .awaddr(int_w_axi_lite_awaddr),.awvalid(int_w_axi_lite_awvalid),.awready(int_w_axi_lite_awready),
      .wdata(int_w_axi_lite_wdata),.wstrb(int_w_axi_lite_wstrb),.wvalid(int_w_axi_lite_wvalid),.wready(int_w_axi_lite_wready),
      .rx_qp(compat_rx_qp),.db_qp(ack_mgr_o_qp_cqe_proc_qpid),
      .enabled(compat_enable),.local_mac(compat_local_mac),.remote_mac(compat_remote_mac),
      .local_ip(compat_local_ip),.remote_ip(compat_remote_ip),.rkey(compat_rkey),.access(compat_access),
      .virtual_base(compat_va),.dma_base(compat_pa),.db_base(compat_db),.region_len(compat_len),.qp_conf(compat_qp_conf),
      .remote_qpn(compat_remote_qpn),.last_psn(compat_last_psn),.pkey(compat_pkey),
      .psn_load(compat_psn_load),.psn_load_qp(compat_psn_qp),.psn_load_value(compat_psn_value)
    );
    lrdma_credit_rx compat_credit (
      .clk(m_axi_aclk),.rstn(m_axi_aresetn_shards[6]),.rx_data(roce_cmac_s_axis_tdata),.rx_keep(roce_cmac_s_axis_tkeep),
      .rx_valid(roce_cmac_s_axis_tvalid),.rx_last(roce_cmac_s_axis_tlast),.rx_error(roce_cmac_s_axis_tuser),
      .lookup_qp(compat_rx_qp),.enabled(compat_enable),.local_mac(compat_local_mac),.remote_mac(compat_remote_mac),
      .local_ip(compat_local_ip),.remote_ip(compat_remote_ip),.qp_conf(compat_qp_conf),.remote_qpn(compat_remote_qpn),.pkey(compat_pkey),
      .virtual_base(compat_va),.dma_base(compat_pa),.rkey(compat_rkey),.region_len(compat_len),.access(compat_access),
      .psn_load(compat_psn_load),.psn_load_qp(compat_psn_qp),.psn_load_value(compat_psn_value),
      .awaddr(compat_credit_awaddr),.awvalid(compat_credit_awvalid),.awready(compat_credit_awready),
      .wdata(compat_credit_wdata),.wvalid(compat_credit_wvalid),.wready(compat_credit_wready),
      .bresp(compat_credit_bresp),.bvalid(compat_credit_bvalid),.bready(compat_credit_bready),
      .tx_data(credit_tx_data),.tx_keep(credit_tx_keep),.tx_valid(credit_tx_valid),.tx_ready(credit_tx_ready),
      .committed(credit_committed),.dropped(credit_dropped),.duplicates(credit_duplicates),.write_errors(credit_write_errors)
    );
    lrdma_packet_mux compat_tx_mux (
      .clk(m_axi_aclk),.rstn(m_axi_aresetn_shards[7]),
      .a_data(primary_tdata),.a_keep(primary_tkeep),.a_valid(primary_tvalid),.a_last(primary_tlast),.a_ready(primary_tready),
      .b_data(credit_tx_data),.b_keep(credit_tx_keep),.b_valid(credit_tx_valid),.b_last(1'b1),.b_ready(credit_tx_ready),
      .m_data(cmac_m_axis_tdata),.m_keep(cmac_m_axis_tkeep),.m_valid(cmac_m_axis_tvalid),.m_last(cmac_m_axis_tlast),.m_ready(cmac_m_axis_tready)
    );

 lrdma_cq_commit compat_cq_commit(.clk(m_axi_aclk),.rstn(m_axi_aresetn_shards[7]),.db_addr(compat_db),.head(ack_mgr_o_qp_cq_hdptr),
.s_awaddr(cq_core_awaddr),.m_awaddr(m_axi_write_completion_awaddr),
.s_awlen(cq_core_awlen),.m_awlen(m_axi_write_completion_awlen),
.s_awsize(cq_core_awsize),.m_awsize(m_axi_write_completion_awsize),
.s_awburst(cq_core_awburst),.m_awburst(m_axi_write_completion_awburst),
.s_awcache(cq_core_awcache),.m_awcache(m_axi_write_completion_awcache),
.s_awprot(cq_core_awprot),.m_awprot(m_axi_write_completion_awprot),
.s_awvalid(cq_core_awvalid),.m_awvalid(m_axi_write_completion_awvalid),
.s_awready(cq_core_awready),.m_awready(m_axi_write_completion_awready),
.s_wdata(cq_core_wdata),.m_wdata(m_axi_write_completion_wdata),
.s_wstrb(cq_core_wstrb),.m_wstrb(m_axi_write_completion_wstrb),
.s_wlast(cq_core_wlast),.m_wlast(m_axi_write_completion_wlast),
.s_wvalid(cq_core_wvalid),.m_wvalid(m_axi_write_completion_wvalid),
.s_wready(cq_core_wready),.m_wready(m_axi_write_completion_wready),
.s_bresp(cq_core_bresp),.m_bresp(m_axi_write_completion_bresp),
.s_bvalid(cq_core_bvalid),.m_bvalid(m_axi_write_completion_bvalid),
.s_bready(cq_core_bready),.m_bready(m_axi_write_completion_bready));
assign m_axi_write_completion_awid=cq_core_awid;
assign m_axi_write_completion_awlock=cq_core_awlock;
assign cq_core_bid=m_axi_write_completion_bid;

endmodule
