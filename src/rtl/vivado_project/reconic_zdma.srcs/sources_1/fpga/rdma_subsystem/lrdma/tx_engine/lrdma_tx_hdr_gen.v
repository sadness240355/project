`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_tx_hdr_gen — TX 引擎头部生成模块
//
// 【模块概述】
// 发送路径最核心、最复杂的模块，从 WQE 到完整 RoCEv2 数据包头部的全部生成逻辑。
// 功能：
//  1. 从 SQ WQE FIFO 取出 WQE，解析操作码（RDMA WRITE）和各字段
//  2. 读取 QP 配置（目的 MAC/IP/QP、PMTU、PSN 等）
//  3. 生成 RoCEv2 报文头（ETH + IPv4/IPv6 + UDP + BTH + 可选 RETH）
//  4. PMTU 分片（大于 PMTU 的传输切为 FIRST/MIDDLE/LAST 多包）
//  5. PSN/MSN 递增管理，支持重传
//
// 【主 FSM（9 个状态）】
// IDLE ──(FIFO 非空)──> RD_WQE ──> RD_Q_INFO
// RD_Q_INFO ──┬─(QP1/MAD)──> HDR_GEN
//             ├─(nack/fatal)──> DROP_WQE
//             ├─(非法 opcode)──> ILL_OPCODE
//             └─(正常)──> WAIT0 ──┬─(IPv6)─> HDR_GEN ─> HDR_STO ─> IDLE/WAIT0
//                                 └─(IPv4)─> IP_CHKSUM ─> HDR_GEN
//
// 【WQE 字段解析（RD_WQE 状态）】
// WRID、QP ID、操作码、总传输长度(LENGTH)、本地地址(LADDR)、
// 远端偏移(ROFFSET)、远程标签(RTAG)
//
// 【PMTU 分片（WAIT0 状态）】
// 按 PMTU（256~4096B）切片，确定 payload_len 和 rem_len，
// 选择 BTH opcode（WRITE_FIRST/MIDDLE/LAST/ONLY）
// 首发/完整重发的首包需要 RETH；部分重传 WQE 从 WRITE_MIDDLE/LAST 开始，不带 RETH。
// 大于 PMTU 时 HDR_STO 完成后回 WAIT0 继续下一分片。
//
// 【IPv4 校验和（IP_CHKSUM 状态）】
// 两级流水线：WAIT0 预计算大部分分量，IP_CHKSUM 完成进位折叠
//
// 【头部拼装（HDR_GEN + HDR_STO 状态）】
// 拼装 ETH 14B + IPv4/IPv6 + UDP + BTH + 可选 RETH
// 打包到 o_hdr_data[1023:0]（高 96-bit 元信息 + 低 928-bit 头部字节）
// 通过 o_hdr_valid 握手传递给 buf_mgr
//
// 【OSQ Push】
// 首包时发起 OSQ push 请求，记录 {start_psn, end_psn, msn, opcode, wrid, bufid}
//
// 【per-QP 状态管理】
// LUTRAM 存储 osq_nacked 和 qp_fatal 标志，处理多源写入冲突
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 笔记：
//  1、QP1（MAD）的 TX 数据流是 DMA 从 DDR 直读整帧, 读到此处计算并添加 icrc 后进行发送。
//     MAD TX 的正确做法是：软件在 DDR 预拼好完整帧（ETH+IP+UDP+BTH(UD 0x64)+DETH+Payload），硬件只管 DMA 读出 → 附加 ICRC → 发送。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_tx_hdr_gen #(
    parameter C_MAX_QP = 2048,
    parameter C_MAX_QID_WIDTH = 11,  // 5
    parameter C_OS_Q_INDX_WIDTH = 3,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_ENABLE_FAST_WQE_CONSUME = 1,
    parameter PSN_based_retransmission = 1  // 基于 PSN 的重传模式
) (
    input core_clk,
    input core_rst,

    // SQ WQE FIFO 接口（来自 QP 管理器）
    // qpm_fifo 就是 SQ WQE FIFO
    input                               i_qpm_fifo_empty,   // SQ WQE FIFO 为空
    input      [                 511:0] i_qpm_wqe_data,
    input                               i_qpm_wqe_fast_valid,
    output reg                          o_qpm_wqe_pop,
    input                               i_wqe_halt,         // 暂停请求，不再从 QPM/SQ FIFO 拉取新的 SQ WQE
    input      [C_MAX_QID_WIDTH -1 : 0] i_wqe_halted_qpid,  // 要暂停的 QP ID
    // 模块不会立刻硬停，而是走到安全点后把 o_wqe_halted 拉高，表示“已经停稳，可以认为暂停生效”。
    output reg                          o_wqe_halted,       // 确认已暂停

    // QP 配置读写接口（与 Initiator 交互）
    output reg                       o_reg_rd_en,          // 发起 QP 配置读请求
    output reg                       o_reg_rd_en_s,        // 发起 QP 配置读请求（第二组）
    input                            i_reg_info_val,       // QP 配置有效
    input                            i_reg_psn_val,        // SQPSNi 寄存器值有效
    output reg                       o_reg_wr_en,          // STATSSNi 和 SQPSNi 的写请求使能，相当于是 QP 寄存器写回正在进行
    output reg [C_MAX_QID_WIDTH-1:0] o_qp_id,              // qpid, WQE Structure -- i_qpm_wqe_data[136+C_MAX_QID_WIDTH-1 : 136]
    input      [               15:0] i_reg_pkey,           // o_reg_rd_en 返回 QPADVCONFi[31:16]
    input      [                2:0] i_reg_pmtu,           // QPCONFi[10:8]
    input      [               23:0] i_reg_dest_qpid,      // o_reg_rd_en 返回 DESTQPCONFi[23:0]
    input      [               23:0] i_reg_sq_psn,         // SQPSNi[23:0]
    output reg [               23:0] o_reg_sq_psn,
    input      [               23:0] i_reg_sq_msn,         // STATSSNi[23:0]
    output reg [               23:0] o_reg_sq_msn,
    input      [               47:0] i_reg_mac_dest_addr,  // o_reg_rd_en_s 返回 MACDESADD
    input      [              127:0] i_reg_ip_dest_addr,   // o_reg_rd_en_s 返回 IPDESADDR
    input      [               15:0] i_reg_udp_src_port,   // LRDMACONF[23:8]
    input      [                7:0] i_reg_ttl,            // o_reg_rd_en 返回 QPADVCONFi[15:8]
    input      [                5:0] i_reg_dscp,           // o_reg_rd_en 返回 QPADVCONFi[5:0]

    // ACK 管理器接口
    input [4:0] i_reg_qp_to,

    // RNIC 全局配置
    input [ 47:0] i_reg_mac_src_addr,
    input [ 31:0] i_reg_ipv4_src_addr,
    input [127:0] i_reg_ipv6_src_addr,
    input         i_reg_lrdma_en,
    input         i_reg_ip_ver,

    //o_hdr_data 高 96 bit 是元信息，低 928 bit 是实际头部:
    //  o_hdr_data[1023:1008] = payload_len（16-bit）
    //  o_hdr_data[1007:944] = local_offset（64-bit，DMA 源地址）
    //  o_hdr_data[943] = 1'b0
    //  o_hdr_data[942:928] = q_id[14:0]
    //=========================================================
    // 头部输出接口（→ tx_engine_buf_mgr）
    input               i_buf_mngr_rdy,  // buf_mgr 准备好接收头部
    output reg [1023:0] o_hdr_data,      // 拼好的头部数据
    output reg [   7:0] o_hdr_len,       // 头部长度（字节）(所有头部)，包含 SDATA
    output reg          o_hdr_valid,
    output reg          o_hdr_ip_ver,    // 0-ipv4; 1-ipv6

    // 有关 SQ WQE 的信息
    output reg                       o_wqp_req,               // 请求向 OSQ 登记一条普通 WRITE WQE
    input                            i_wqp_resp,              // OSQ push 完成应答
    output reg [C_MAX_QID_WIDTH-1:0] o_wqe_qpid,              // 推入 OSQ 的 QP ID
    output reg [                7:0] o_wqe_opcode,            // WQE 操作码
    output reg [               15:0] o_wqe_wrid,              // WQE 的 WRID
    output reg [               23:0] o_wqe_send_psn,          // 该 WQE 最后一个分片包的 PSN
    output reg [                4:0] o_wqe_qp_to,             // TIMEOUTCONFi[4:0], 超时值
    output reg                       o_wqe_retried,           // 是否重传
    output reg [               23:0] o_wqe_send_first_psn,    // OSQ/重传上下文中的 WQE 起始 PSN；部分重传时不等于线上首包 BTH PSN
    output reg [               23:0] o_wqe_send_msn,
    output reg                       o_wqe_explicit_ack_req,  // BTH 的 AckReq 位
    output reg                       o_wqe_fatal_push,

    output reg        wqe_opcode_err,
    output     [11:0] wqe_status_out,
    output reg [31:0] o_last_out_pkt_info,
    output reg [15:0] o_out_wr_pkt_cnt,
    output reg [15:0] o_out_mad_pkt_cnt,
    input             i_qp_recovery,

    input  dma_in_idle,
    output o_dma_in_idle,

    output reg [  C_MAX_QID_WIDTH-1:0] o_wqe_fatal_qpid,
    output reg                         o_wqe_fatal_qp_vld,
    input                              i_wqe_fatal_qp_rdy,
    output reg                         o_wqe_dropped,
    output reg [  C_MAX_QID_WIDTH-1:0] o_wqe_dropped_qpid,
    input                              i_osq_pop,
    input      [  C_MAX_QID_WIDTH-1:0] i_osq_pop_qpid,
    input                              init_start,
    input      [C_MAX_QID_WIDTH-1 : 0] init_count,
    input                              i_qp_fatal_wr_en,
    input      [C_MAX_QID_WIDTH-1 : 0] i_qp_fatal_wr_addr,
    input                              i_qp_fatal_wr_data,

    input                         i_osq_nacked_wr_en,
    input [C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr,
    input                         i_osq_nacked_wr_data,
    input                         i_osq_nacked_wr_en_aux,
    input [C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_aux,
    input                         i_osq_nacked_wr_data_aux,
    input [C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_pending,
    input                         i_osq_nacked_wr_en_pending,
    input [C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_aux_pending,
    input                         i_osq_nacked_wr_en_aux_pending,

    output reg                         o_osq_nacked_resp_wr_en,
    output reg [C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr,
    output reg                         o_osq_nacked_resp_wr_data,
    output reg                         o_osq_nacked_resp_wr_en_aux,
    output reg [C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr_aux,
    output reg                         o_osq_nacked_resp_wr_data_aux,
    output reg                         o_osq_nacked_resp_wr_en_pending,
    output reg [C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr_pending,
    output reg                         o_osq_nacked_resp_wr_data_pending
);

    `include "lrdma_defs.vh"
    localparam QP1 = 'd0;

    //==========================================================================
    // 主 FSM 状态（4-bit 普通二进制编码）
    //
    // 这个 FSM 是 TX 路径的“包头工厂”：从 SQ FIFO 拿一条 WQE，按 QP 配置和
    // 操作码生成对应 RoCEv2 包头（ETH+IP+UDP+BTH 必选，按需追加 RETH），
    // 并按 PMTU 切片成多个连续包；payload 由下游 lrdma_tx_dma_read_engine 取。
    //
    // 时间线一览（一次正常 RDMA WRITE，多包切片）：
    //   IDLE → RD_WQE → RD_Q_INFO → WAIT0 → [IP_CHKSUM] → HDR_GEN → HDR_STO
    //        → ( WAIT0 → [IP_CHKSUM] → HDR_GEN → HDR_STO )*    // 切片中间包
    //        → IDLE
    //
    // 异常分支：
    //   RD_WQE 解析到非法 opcode    → ILL_OPCODE（fatal 上报）→ IDLE
    //   RD_Q_INFO 发现 QP 处于 NACK/recovery/halt → DROP_WQE → IDLE
    //   QP1(MAD) 路径：RD_Q_INFO 直接跳 HDR_GEN（MAD 包头由软件已组好）
    //==========================================================================
    localparam IDLE = 4'b0000;  // 空闲。等 SQ FIFO 弹出 WQE 触发新一轮发包
    localparam RD_WQE = 4'b0001;  // 从 SQ FIFO 拿一条 WQE，解析操作码、长度、SGL 起址等字段
    localparam RD_Q_INFO = 4'b0010;  // 等 QP 配置 BRAM 返回，做发包前决策；QP1(MAD) 跳 HDR_GEN
    localparam WAIT0 = 4'b0011;  // 按 PMTU 切片：算本包 payload_len / 剩余 rem_len，置 reth_present 等位
    localparam IP_CHKSUM = 4'b0100;  // 计算 IPv4 头校验和。IPv6 路径不需要这一步，会被跳过
    localparam HDR_GEN = 4'b0101;  // 把 ETH/IP/UDP/BTH 以及可选 RETH 拼到各分段寄存器
    localparam HDR_STO = 4'b0110;  // 把拼好的头部按字节序写入 o_hdr_data[1023:0]，并设 o_hdr_len
    localparam ILL_OPCODE = 4'b0111;  // WQE 解析出非法 op-code，向上层报 fatal，停在这里直到复位
    localparam DROP_WQE = 4'b1000;  // QP 被 NACK/recovery/halt，本条 WQE 不发包，直接丢弃后回 IDLE
    localparam retransmit_wqe_bit_pos = 152;

    reg  [                111:0] eth_hdr;
    reg  [                319:0] ip_hdr;
    reg  [                 63:0] udp_hdr;
    reg  [                 95:0] bth_hdr;
    reg  [                127:0] reth_hdr;
    reg  [                  3:0] hdr_gen_cs;
    reg  [                 19:0] ip_hdr_chksum;
    reg  [                 31:0] rem_len;  // WQE Structure -- LENGTH，传输长度
    reg  [                 31:0] dma_len;  // WQE Structure -- LENGTH，传输长度
    reg  [                  7:0] wqe_opcode;  // WQE Structure -- OPCODE，操作码。
    // first_trans 天然就是“每条 WQE 只打一拍事件”，这直接避免了分片/middle/last 多包场景下重复记账
    reg                          first_trans;  // first/only flag, cleared after first packet sent
    reg  [                 15:0] payload_len;  // 每包有效负载长度，不包括头部长度；由总长度按 PMTU/状态切片得到
    reg                          reth_present;  // 本分片是否携带 RETH；部分重传 retransmit_wqe=1 时强制不带
    reg  [                 63:0] local_offset;  // WQE Structure -- LADDR，本地地址。操作数据缓冲区的地址
    reg  [                 63:0] remote_offset;  // WQE Structure -- ROFFSET，远端偏移地址
    reg  [                 31:0] r_key;  // WQE Structure -- RTAG，远程 TAG
    reg                          wqe_push_pending;
    reg                          chksum_cal_start;
    reg                          ipv4_len_cal;  // IP_CHKSUM 两拍流水的第一拍完成标志
    reg  [                 12:0] max_pmtu;
    reg  [                 23:0] dest_qpid;
    wire [                 15:0] q_id;
    wire [                 15:0] ipv6_len;
    reg  [                 17:0] ip_hdr_chksum_add_pipe;
    reg  [                 17:0] ip_hdr_chksum_src_pipe;
    reg  [                 15:0] ipv4_len;
    reg  [                 47:0] mac_dest_addr;
    reg  [                127:0] ip_dest_addr;
    reg                          ip_ver;
    reg  [                  5:0] dscp;
    reg  [                  7:0] ttl;
    reg  [                 15:0] ipv4_identification;
    reg                          push_to_os_buf;  // 仅 first_trans=1 时置1（每条 WQE 只 push 一次 OSQ），后续 middle/last 包会置 0
    //Added for performance update. FSM will proceed to the next header generation logic
    //and waits in HDR_GEN state until this signal is de-asserted
    reg                          buf_mgr_busy;  // buf_mgr 忙标志
    reg  [                  4:0] reg_qp_to;
    reg                          retried;  // 锁存 WQE bit[16] 的重传标记
    reg                          wqe_pushed_in_fatal;
    reg                          retransmit_wqe;  // 这条 WQE 是否走了 lrdma_qp_mgr_top 的部分重传路径
    reg  [               23 : 0] retransmit_wqe_start_psn;
    reg                          qp_fatal_detected_wr_en;
    reg  [C_MAX_QID_WIDTH-1 : 0] qp_fatal_detected_wr_addr;
    reg                          qp_fatal_detected_wr_data;
    reg                          qp_fatal_detected_wr_en_aux;
    reg  [C_MAX_QID_WIDTH-1 : 0] qp_fatal_detected_wr_addr_aux;
    reg                          qp_fatal_detected_wr_data_aux;
    reg                          qp_wqe_opcode_fatal_wr_en;
    reg  [C_MAX_QID_WIDTH-1 : 0] qp_wqe_opcode_fatal_wr_addr;
    reg                          qp_wqe_opcode_fatal_wr_data;
    reg                          qp_wqe_opcode_fatal_wr_en_aux;
    reg  [C_MAX_QID_WIDTH-1 : 0] qp_wqe_opcode_fatal_wr_addr_aux;
    reg                          qp_wqe_opcode_fatal_wr_data_aux;
    wire                         qp_fatal_rd_data_1;  // 该 QP 被标记为 fatal
    wire                         osq_nacked_rd_data_int_1;
    reg                          osq_nacked_rd_data_1;  // 该 QP 收到 NACK
    wire                         qp_wqe_opcode_fatal_rd_data;  // 该 QP 曾发过非法 opcode
    reg                          osq_nacked_wr_pending;
    reg  [C_MAX_QID_WIDTH-1 : 0] osq_nacked_wr_id;
    reg                          osq_nacked_wr_data;


    assign q_id           = {{16 - C_MAX_QID_WIDTH{1'b0}}, o_qp_id};
    assign wqe_status_out = {4'h0, buf_mgr_busy, 2'b0, wqe_push_pending, hdr_gen_cs};


    // Common LUT RAM Instances for per QP signals

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_i_qp_fatal_1 (
        .core_clk(core_clk),
        .core_rst(core_rst),
        .wr_en   (init_start || i_qp_fatal_wr_en),
        .wr_addr (init_start ? init_count : i_qp_fatal_wr_addr),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_wr_data),
        .rd_en   (1'b1),
        .rd_addr (o_qp_id),
        .rd_data (qp_fatal_rd_data_1)
    );

    // osq_nacked LUT RAM 是一个 per-QP 的 1-bit 标志，记录该 QP 是否收到了 NACK（需要重传）
    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_i_osq_nacked_1 (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || i_osq_nacked_wr_en),
        .wr_addr    (init_start ? init_count : i_osq_nacked_wr_addr),
        .wr_data    (init_start ? 1'b0 : i_osq_nacked_wr_data),
        .wr_en_aux  (i_osq_nacked_wr_en_aux),
        .wr_addr_aux(i_osq_nacked_wr_addr_aux),
        .wr_data_aux(i_osq_nacked_wr_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (o_qp_id),
        .rd_data    (osq_nacked_rd_data_int_1)
    );
    always @(*) begin
        if (i_osq_nacked_wr_en_pending && i_osq_nacked_wr_addr_pending == o_qp_id) begin
            osq_nacked_rd_data_1 = 1'b1;
        end else if (i_osq_nacked_wr_en_aux_pending && i_osq_nacked_wr_addr_aux_pending == o_qp_id) begin
            osq_nacked_rd_data_1 = 1'b0;
        end else begin
            osq_nacked_rd_data_1 = osq_nacked_rd_data_int_1;
        end
    end

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_MAX_QP),
        .C_Init_with_1(0)
    ) inst_qp_wqe_opcode_fatal (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || qp_wqe_opcode_fatal_wr_en),
        .wr_addr    (init_start ? init_count : qp_wqe_opcode_fatal_wr_addr),
        .wr_data    (init_start ? 1'b0 : qp_wqe_opcode_fatal_wr_data),
        .wr_en_aux  (qp_wqe_opcode_fatal_wr_en_aux),
        .wr_addr_aux(qp_wqe_opcode_fatal_wr_addr_aux),
        .wr_data_aux(qp_wqe_opcode_fatal_wr_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (o_qp_id),
        .rd_data    (qp_wqe_opcode_fatal_rd_data)
    );

    // IPv4 头长度组合逻辑（用于在 WAIT0 → IP_CHKSUM 转换时同拍计算 ipv4_len，省 1 周期）
    wire [1:0] payload_pad = 2'd0 - payload_len[1:0];
    wire [15:0] ipv4_len_comb = 8'h2C + payload_len + {14'd0,payload_pad} + (reth_present ? 16'd16 : 16'd0);

    // IPv4 checksum 双级 carry fold (组合逻辑, 供 IP_CHKSUM cycle 2 使用)
    wire [16:0] chksum_fold1 = {1'b0, ip_hdr_chksum[15:0]} + {13'b0, ip_hdr_chksum[19:16]};
    wire [15:0] chksum_fold2 = chksum_fold1[15:0] + {15'b0, chksum_fold1[16]};
    wire        fast_wqe_consume_valid = (C_ENABLE_FAST_WQE_CONSUME != 0) && i_qpm_wqe_fast_valid;

    assign ipv6_len = 8'h18 + payload_len + {14'd0,payload_pad} + (reth_present ? 16'd16 : 16'd0);

    always @(posedge core_clk) begin
        if (core_rst) begin
            eth_hdr                           <= 'd0;
            ip_hdr                            <= 'd0;
            udp_hdr                           <= 'd0;
            bth_hdr                           <= 'd0;
            reth_hdr                          <= 'd0;
            o_hdr_data                        <= 'd0;
            o_hdr_valid                       <= 1'b0;
            o_hdr_ip_ver                      <= 1'b0;
            o_hdr_len                         <= 8'h00;
            o_qpm_wqe_pop                     <= 1'b0;
            o_qp_id                           <= {C_MAX_QID_WIDTH{1'b0}};
            o_reg_rd_en                       <= 1'b0;
            o_reg_rd_en_s                     <= 1'b0;
            hdr_gen_cs                        <= IDLE;
            ip_hdr_chksum                     <= 20'd0;
            rem_len                           <= 32'd0;
            dma_len                           <= 32'd0;
            first_trans                       <= 1'b0;
            payload_len                       <= 16'h0000;
            reth_present                      <= 1'b0;
            remote_offset                     <= 64'd00;
            local_offset                      <= 64'd00;
            wqe_opcode                        <= 8'h00;
            r_key                             <= 32'd0;
            chksum_cal_start                  <= 1'b0;
            ipv4_len_cal                      <= 1'b0;

            o_reg_wr_en                       <= 1'b0;
            wqe_opcode_err                    <= 1'b0;
            dest_qpid                         <= 24'd0;
            o_reg_sq_psn                      <= 24'd0;
            o_wqe_send_first_psn              <= 24'd0;
            o_reg_sq_msn                      <= 24'd0;
            o_wqe_wrid                        <= 16'h0000;
            mac_dest_addr                     <= 'd0;
            ip_dest_addr                      <= 'd0;
            ip_ver                            <= 1'b0;
            o_out_wr_pkt_cnt                  <= 16'h0000;
            o_out_mad_pkt_cnt                 <= 16'h0000;
            o_wqe_halted                      <= 1'b0;
            ipv4_identification               <= 16'h0000;
            retried                           <= 1'b0;
            push_to_os_buf                    <= 1'b0;
            buf_mgr_busy                      <= 1'b0;
            reg_qp_to                         <= 5'h00;
            wqe_pushed_in_fatal               <= 1'b0;
            o_wqe_fatal_qpid                  <= {C_MAX_QID_WIDTH{1'b0}};
            o_wqe_fatal_qp_vld                <= 1'b0;
            o_wqe_dropped                     <= 1'b0;
            o_wqe_dropped_qpid                <= {C_MAX_QID_WIDTH{1'b0}};
            ip_hdr_chksum_add_pipe            <= 17'd0;
            ip_hdr_chksum_src_pipe            <= 18'd0;
            ipv4_len                          <= 16'd0;
            retransmit_wqe                    <= 1'b0;
            retransmit_wqe_start_psn          <= 24'd0;
            qp_fatal_detected_wr_en           <= 1'b0;
            qp_fatal_detected_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
            qp_fatal_detected_wr_data         <= 1'b0;
            qp_fatal_detected_wr_en_aux       <= 1'b0;
            qp_fatal_detected_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
            qp_fatal_detected_wr_data_aux     <= 1'b0;
            qp_wqe_opcode_fatal_wr_en         <= 1'b0;
            qp_wqe_opcode_fatal_wr_addr       <= {C_MAX_QID_WIDTH{1'b0}};
            qp_wqe_opcode_fatal_wr_data       <= 1'b0;
            qp_wqe_opcode_fatal_wr_en_aux     <= 1'b0;
            qp_wqe_opcode_fatal_wr_addr_aux   <= {C_MAX_QID_WIDTH{1'b0}};
            qp_wqe_opcode_fatal_wr_data_aux   <= 1'b0;
            o_osq_nacked_resp_wr_en           <= 1'b0;
            o_osq_nacked_resp_wr_addr         <= {C_MAX_QID_WIDTH{1'b0}};
            o_osq_nacked_resp_wr_data         <= 1'b0;
            o_osq_nacked_resp_wr_en_aux       <= 1'b0;
            o_osq_nacked_resp_wr_addr_aux     <= {C_MAX_QID_WIDTH{1'b0}};
            o_osq_nacked_resp_wr_data_aux     <= 1'b0;
            o_osq_nacked_resp_wr_en_pending   <= 1'b0;
            o_osq_nacked_resp_wr_addr_pending <= {C_MAX_QID_WIDTH{1'b0}};
            o_osq_nacked_resp_wr_data_pending <= 1'b0;
            osq_nacked_wr_pending             <= 1'b0;
            osq_nacked_wr_id                  <= {C_MAX_QID_WIDTH{1'b0}};
            osq_nacked_wr_data                <= 1'b0;
        end else begin

            // 头部有效握手
            // 进入 HDR_STO 状态 且 OSQ push 已完成 → 拉高 o_hdr_valid，通知 lrdma_tx_buffer_mgr "头部数据准备好了
            if (~wqe_push_pending && (hdr_gen_cs == HDR_STO)) begin
                o_hdr_valid  <= 1'b1;
                o_hdr_ip_ver <= ip_ver;
            end //
            // buf_mgr 回复 i_buf_mngr_rdy=1 → 清零 o_hdr_valid，握手完成
            else if (i_buf_mngr_rdy) begin
                o_hdr_valid <= 1'b0;
            end

            // FSM 进入 ILL_OPCODE → 拉高 o_wqe_fatal_qp_vld，向外部上报"QP o_wqe_fatal_qpid 遇到非法 WQE opcode"
            if ((hdr_gen_cs == ILL_OPCODE) && ~o_wqe_fatal_qp_vld) begin
                o_wqe_fatal_qp_vld <= 1'b1;
            end //
            // 外部确认 i_wqe_fatal_qp_rdy=1 → 清零，握手完成
            else if (i_wqe_fatal_qp_rdy) begin
                o_wqe_fatal_qp_vld <= 1'b0;
            end

            // qp_wqe_opcode_fatal LUT RAM 写入仲裁
            if (o_wqe_fatal_qp_vld && i_qp_fatal_wr_en && o_wqe_fatal_qpid == i_qp_fatal_wr_addr) begin
                // 冲突：本模块要写 opcode_fatal=1，外部同时也在写同一个 QP 的 qp_fatal
                qp_wqe_opcode_fatal_wr_en     <= 1'b1;  // 本模块写入优先
                qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;  // 抑制 aux 端口
                qp_wqe_opcode_fatal_wr_addr   <= o_wqe_fatal_qpid;
                qp_wqe_opcode_fatal_wr_data   <= 1'b1;  // 标记 fatal
            end else begin
                // 无冲突，两个写源各自独立
                if (o_wqe_fatal_qp_vld) begin
                    // 本模块需要写 opcode_fatal=1
                    qp_wqe_opcode_fatal_wr_en   <= 1'b1;
                    qp_wqe_opcode_fatal_wr_addr <= o_wqe_fatal_qpid;
                    qp_wqe_opcode_fatal_wr_data <= 1'b1;
                end else begin
                    qp_wqe_opcode_fatal_wr_en <= 1'b0;
                end

                // 外部（ACK 管理器等）写入 qp_fatal=1 → 通过 aux 端口清零 opcode_fatal
                if (i_qp_fatal_wr_en && i_qp_fatal_wr_data) begin
                    qp_wqe_opcode_fatal_wr_en_aux   <= 1'b1;
                    qp_wqe_opcode_fatal_wr_addr_aux <= i_qp_fatal_wr_addr;
                    qp_wqe_opcode_fatal_wr_data_aux <= 1'b0;  // 清零
                end else begin
                    qp_wqe_opcode_fatal_wr_en_aux <= 1'b0;
                end
            end

            // 头部已发出 (o_hdr_valid=1) 但 buf_mgr 还没 ready → 标记 busy
            if (o_hdr_valid && ~i_buf_mngr_rdy) begin
                buf_mgr_busy <= 1'b1;
            end //
            // buf_mgr ready → 清除 busy
            else if (i_buf_mngr_rdy) begin
                buf_mgr_busy <= 1'b0;
            end

            // i_wqe_halt 是外部要求暂停一下
            // o_wqe_halted 是该模块的暂停确认信号，表示不会再继续推进新的 SQ WQE 发送处理流程。
            // 暂停条件就是模块要先把手头这拍/这次必要动作收尾，等到安全点（空闲态、无关键写回）后，才把 o_wqe_halted 拉高
            if (hdr_gen_cs == IDLE && i_wqe_halt && ~o_reg_wr_en) begin
                o_wqe_halted <= 1'b1;
            end else if (~i_wqe_halt) begin
                o_wqe_halted <= 1'b0;  // i_wqe_halt 拉低后，o_wqe_halted 清零，流程恢复正常推进
            end

            // 这段管理 osq_nacked 的 per-QP LUT RAM 写入: 
            //      因为写入来自外部 ACK 管理器（i_osq_nacked_wr_en/addr/data），
            //      但这个 LUT RAM 的 读端口 正被 FSM 使用（o_qp_id 做地址），
            //      如果写入目标和 FSM 正在处理的 QP 重合，就会产生读写冲突
            // 情况 A: FSM 在 IDLE/RD_WQE → 安全放行，直接转发写入
            if (i_osq_nacked_wr_en && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                o_osq_nacked_resp_wr_en   <= 1'b1;
                o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
            end //
            // 情况 B: FSM 在 RD_Q_INFO 且本拍确认要丢弃 WQE → 也安全放行
            else if (i_osq_nacked_wr_en && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != QP1 && (osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data)) begin
                o_osq_nacked_resp_wr_en   <= 1'b1;
                o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
            end //
            // 情况 C: 目标 QP 和 FSM 处理的 QP 不同 → 无冲突，直接转发
            else if (i_osq_nacked_wr_en && i_osq_nacked_wr_addr != o_qp_id) begin
                o_osq_nacked_resp_wr_en   <= 1'b1;
                o_osq_nacked_resp_wr_addr <= i_osq_nacked_wr_addr;
                o_osq_nacked_resp_wr_data <= i_osq_nacked_wr_data;
            end // 
            // 情况 D: 以上都不满足 → 不转发
            else begin
                o_osq_nacked_resp_wr_en <= 1'b0;
            end

            // 这段管理 osq_nacked_aux 的 per-QP LUT RAM 写入
            if (i_osq_nacked_wr_en_aux && (hdr_gen_cs == IDLE || hdr_gen_cs == RD_WQE)) begin
                o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
            end else if (i_osq_nacked_wr_en_aux && hdr_gen_cs == RD_Q_INFO && i_reg_info_val && o_qp_id != QP1 && (osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data)) begin
                o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
            end else if (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux != o_qp_id) begin
                o_osq_nacked_resp_wr_en_aux   <= 1'b1;
                o_osq_nacked_resp_wr_addr_aux <= i_osq_nacked_wr_addr_aux;
                o_osq_nacked_resp_wr_data_aux <= i_osq_nacked_wr_data_aux;
            end else begin
                o_osq_nacked_resp_wr_en_aux <= 1'b0;
            end

            // 挂起逻辑，即发生冲突，等待补写
            // 当写入请求的目标 QP 等于 FSM 正在处理的 QP，且 FSM 不在安全状态 → 无法立即转发，暂存到 osq_nacked_wr_pending
            if (((i_osq_nacked_wr_en && i_osq_nacked_wr_addr == o_qp_id) || (i_osq_nacked_wr_en_aux && i_osq_nacked_wr_addr_aux == o_qp_id)) && hdr_gen_cs != IDLE && hdr_gen_cs != RD_WQE &&
			  ~(hdr_gen_cs == RD_Q_INFO && (i_reg_info_val && o_qp_id != QP1 && 
			  (osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data)))) begin
                osq_nacked_wr_pending <= 1'b1;
                osq_nacked_wr_id      <= i_osq_nacked_wr_en ? i_osq_nacked_wr_addr : i_osq_nacked_wr_addr_aux;
                osq_nacked_wr_data    <= i_osq_nacked_wr_en ? i_osq_nacked_wr_data : i_osq_nacked_wr_data_aux;
            end else if (hdr_gen_cs == IDLE) begin
                osq_nacked_wr_pending <= 1'b0;
            end
            // 等 FSM 回到 IDLE 后，通过 pending 通道补转发
            if (hdr_gen_cs == IDLE && osq_nacked_wr_pending) begin
                o_osq_nacked_resp_wr_en_pending   <= 1'b1;
                o_osq_nacked_resp_wr_addr_pending <= osq_nacked_wr_id;
                o_osq_nacked_resp_wr_data_pending <= osq_nacked_wr_data;
            end else begin
                o_osq_nacked_resp_wr_en_pending <= 1'b0;
            end

            // FSM
            case (hdr_gen_cs)

                // 等待事件或 WQE
                IDLE: begin
                    push_to_os_buf      <= 1'b0;
                    retried             <= 1'b0;
                    wqe_pushed_in_fatal <= 1'b0;
                    retransmit_wqe      <= 1'b0;

                    if (i_reg_psn_val) begin
                        o_reg_wr_en <= 1'b0;
                    end

                    // 从SQ WQE FIFO取 WQE: SQ FIFO 非空 && 上一轮 PSN/MSN 写回已完成 && 没有暂停请求
                    if (~i_qpm_fifo_empty && ~o_reg_wr_en && ~i_wqe_halt) begin
                        o_qpm_wqe_pop <= 1'b1;
                        hdr_gen_cs    <= RD_WQE;
                    end //
                    else begin
                        o_reg_rd_en   <= 1'b0;
                        o_reg_rd_en_s <= 1'b0;
                    end
                end

                // 从 SQ WQE FIFO 中取出 WQE 并解析所有字段
                RD_WQE: begin
                    o_qpm_wqe_pop <= 1'b0;
                    if ((o_qpm_wqe_pop == 1'b0) || fast_wqe_consume_valid) begin
                        // 普通 BRAM FIFO 路径仍等待 o_qpm_wqe_pop 清零，保证 cached_wqe
                        // 经过 READ_LATENCY_B=1 后稳定。WQE bypass 路径由
                        // i_qpm_wqe_fast_valid 标记，数据来自 QP manager 的旁路寄存器，
                        // 可在 pop 后第一拍直接解析，减少 1 个 core_clk 周期。
                        // 注意: 不在 IDLE 同拍解析旁路 WQE。正常旁路 WQE 发出后仍可能
                        // 进入 OSQ 并由 NAK/timeout 触发重传，RD_WQE 这一拍用于保持
                        // pop、OSQ、PSN 和重传上下文的相对时序不被提前打乱。
                        // ============= 手册：WQE Structure =============
                        // WRID: 工作请求 ID。每个 WQE 的唯一标识符
                        o_wqe_wrid    <= i_qpm_wqe_data[15:0];  // WQE Structure -- WRID，工作申请 ID。每个 WQE 的唯一标识符

                        // i_qpm_wqe_data[140:136]
                        o_qp_id       <= i_qpm_wqe_data[136+C_MAX_QID_WIDTH-1 : 136];

                        // LENGTH: 总长度，仅有效负载长度，不包括头部长度。(这条 WQE 要发送的全部 payload 字节数（一次任务的总量）)
                        dma_len       <= i_qpm_wqe_data[127:96];
                        rem_len       <= i_qpm_wqe_data[127:96];

                        // OPCODE: 操作码
                        //  8’h00 -- RDMA WRITE
                        //  其他 opcode 进入 ILL_OPCODE
                        wqe_opcode    <= i_qpm_wqe_data[135:128];

                        // LADDR: 本地地址。操作数据缓冲区的地址
                        local_offset  <= i_qpm_wqe_data[95:32];

                        // ROFFSET: 远端偏移地址
                        remote_offset <= i_qpm_wqe_data[223:160];

                        // RTAG: 远程标签
                        r_key         <= i_qpm_wqe_data[255:224];

                        o_reg_rd_en   <= 1'b1;
                        o_reg_rd_en_s <= 1'b1;
                        hdr_gen_cs    <= RD_Q_INFO;
                        first_trans   <= 1'b1;

                        retried       <= i_qpm_wqe_data[16];

                        if (PSN_based_retransmission == 1) begin
                            // 进入 lrdma_qp_mgr_top 重传分支后，i_qpm_wqe_data[152] = 1
                            retransmit_wqe           <= i_qpm_wqe_data[retransmit_wqe_bit_pos];
                            // retransmit_wqe_start_psn 来自 QP 管理器插入的 retransmit_wqe_bram_dout_b[55:32]，
                            // 对应 ACK 管理器的 retry_start_psn。它用于 OSQ/重传上下文的 start_psn 记录，
                            // retransmit 重传真正发出的 BTH PSN 仍由 o_reg_sq_psn 生成。
                            retransmit_wqe_start_psn <= i_qpm_wqe_data[439:416];
                        end else begin
                            retransmit_wqe <= 1'b0;
                        end
                    end
                end

                // 等 QP 配置返回 (i_reg_info_val=1)，做发包前决策
                RD_Q_INFO: begin
                    if (i_reg_info_val) begin  // QP 配置到了
                        if (o_qp_id == QP1) begin  // SQ1，快速路径
                            // tips: 虽然会生成头部，但 lrdma_tx_buffer_mgr 把 QP1 的 QP1_HDR_VAL=1，
                            //       lrdma_tx_dma_read_engine 对 QP1 跳过头部 BRAM 读取 → 这里生成的头部实际不会被使用，帧内容全靠 DDR DMA
                            hdr_gen_cs  <= HDR_GEN;
                            payload_len <= rem_len;
                            rem_len     <= 32'd0;
                        end //
                        // NACK / Fatal / Recovery / Halt 检测
                        else if (osq_nacked_rd_data_1 || qp_fatal_rd_data_1 || i_qp_recovery || (i_wqe_halt && (i_wqe_halted_qpid == o_qp_id)) || qp_wqe_opcode_fatal_rd_data) begin
                            // Fatal/Recovery 优先且无 NACK → 跳 IDLE，push_to_os_buf=1（让 OSQ 记录这条被丢弃的 WQE）
                            // 有 NACK → 跳 DROP_WQE，o_wqe_dropped=1
                            hdr_gen_cs          <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? IDLE : DROP_WQE;
                            o_wqe_dropped       <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b0 : 1'b1;
                            o_wqe_dropped_qpid  <= o_qp_id;
                            push_to_os_buf      <= ((qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1)) ? 1'b1 : 1'b0;
                            o_reg_rd_en         <= 1'b0;
                            first_trans         <= 1'b0;

                            wqe_pushed_in_fatal <= (qp_fatal_rd_data_1 || i_qp_recovery || qp_wqe_opcode_fatal_rd_data) & (~osq_nacked_rd_data_1);
                        end //
                        else if (wqe_opcode != 8'h00) begin
                            hdr_gen_cs       <= ILL_OPCODE;
                            o_wqe_fatal_qpid <= o_qp_id;
                        end //
                        // 正常
                        else begin
                            hdr_gen_cs <= WAIT0;
                        end

                        // 锁存 QP && RNIC 配置信息
                        o_reg_rd_en_s        <= 1'b0;
                        dest_qpid            <= i_reg_dest_qpid;
                        mac_dest_addr        <= i_reg_mac_dest_addr;
                        ip_dest_addr         <= i_reg_ip_dest_addr;
                        ip_ver               <= i_reg_ip_ver;
                        dscp                 <= i_reg_dscp;
                        ttl                  <= i_reg_ttl;
                        o_reg_sq_psn         <= i_reg_sq_psn;
                        // retransmit_wqe 时，o_wqe_send_first_psn 是 OSQ/重传上下文的 start_psn，
                        // 不是本次重传线上首包的 BTH PSN。
                        o_wqe_send_first_psn <= retransmit_wqe ? retransmit_wqe_start_psn : i_reg_sq_psn;
                        o_reg_sq_msn         <= i_reg_sq_msn + 1'b1;
                        reg_qp_to            <= i_reg_qp_to;
                    end

                    // PMTU 解码
                    case (i_reg_pmtu)
                        3'b000: begin
                            max_pmtu <= 13'h0100;  // 256
                        end
                        3'b001: begin
                            max_pmtu <= 13'h0200;  // 512
                        end
                        3'b010: begin
                            max_pmtu <= 13'h0400;  // 1024
                        end
                        3'b011: begin
                            max_pmtu <= 13'h0800;  // 2048
                        end
                        3'b100: begin
                            max_pmtu <= 13'h1000;  // 4096
                        end
                        default: begin
                            max_pmtu <= 13'h100;  // 256
                        end
                    endcase
                end

                // PMTU 分片计算 + 决定 reth_present
                WAIT0: begin
                    push_to_os_buf <= 1'b0;

                    // HDR_STO 状态可能会跳到 WAIT0 状态
                    // 上一包的 PSN/MSN 等状态要写回，因此这里要等待写回完成有效
                    if (i_reg_psn_val) begin
                        o_reg_wr_en <= 1'b0;
                    end

                    // 优化：在 WAIT0 每个周期预计算 ip_hdr_chksum_add_pipe（不依赖 payload_len/reth_present）
                    // 使用 (ipv4_identification + 1) 因为实际递增在下方 IP_CHKSUM 转换分支
                    if (~ip_ver) begin
                        ip_hdr_chksum_add_pipe <= {8'h45, dscp[5:0], 2'b00} + (ipv4_identification + 1'b1) + 16'h4000 + ip_dest_addr[31:16] + ip_dest_addr[15:0];
                        ip_hdr_chksum_src_pipe <= {ttl, 8'h11} + i_reg_ipv4_src_addr[31:16] + i_reg_ipv4_src_addr[15:0];
                    end

                    // ack 同拍即允许前进 (避免 NBA 造成 off-by-one 每包多 1 拍 stall)
                    if (ip_ver && (~o_reg_wr_en || i_reg_psn_val)) begin  // ipv6
                        hdr_gen_cs <= HDR_GEN;
                    end else if (~ip_ver && (~o_reg_wr_en || i_reg_psn_val)) begin  // ipv4
                        hdr_gen_cs          <= IP_CHKSUM;
                        ipv4_identification <= ipv4_identification + 1'b1;
                    end

                    if (~o_reg_wr_en || i_reg_psn_val) begin
                        if ((({19'd0, max_pmtu} < rem_len) || first_trans) && (wqe_opcode == 8'h00)) begin
                            payload_len  <= ({19'd0, max_pmtu} < rem_len) ? max_pmtu : rem_len;
                            rem_len      <= ({19'd0, max_pmtu} < rem_len) ? rem_len - {19'd0, max_pmtu} : 32'd0;
                            reth_present <= first_trans && ~retransmit_wqe;
                        end else begin
                            payload_len  <= rem_len;
                            rem_len      <= 32'd0;
                            reth_present <= 1'b0;
                        end
                    end
                end

                // IPv4 头校验和 (优化为 2 拍固定流水)
                //   Cycle 1: full_sum = add_pipe + src_pipe + ipv4_len_comb (3 项, ≤20bit)
                //   Cycle 2: 双级 carry fold → 16-bit one's complement sum
                // 最后头校验 = {~ip_hdr_chksum[7:0], ~ip_hdr_chksum[15:8]}
                IP_CHKSUM: begin
                    if (~chksum_cal_start) begin
                        // Cycle 1: 全量求和 — add_pipe(5 项) + src_pipe(3 项) + ipv4_len_comb(Word1)
                        // ipv4_len_comb 此时已反映 WAIT0 NBA 更新后的 payload_len/reth_present
                        ip_hdr_chksum    <= ip_hdr_chksum_add_pipe + ip_hdr_chksum_src_pipe + ipv4_len_comb;
                        ipv4_len         <= ipv4_len_comb;
                        chksum_cal_start <= 1'b1;
                    end else begin
                        // Cycle 2: 双级 carry fold (9 × 16-bit 之和 ≤ 0x8FFF7, 保证 2 级折叠无残余进位)
                        ip_hdr_chksum    <= {4'b0, chksum_fold2};
                        chksum_cal_start <= 1'b0;
                        ipv4_len_cal     <= 1'b0;
                        hdr_gen_cs       <= HDR_GEN;
                    end
                end

                // 拼装所有协议头寄存器 + BTH 操作码选择 + PSN 管理
                HDR_GEN: begin
                    if (~buf_mgr_busy) begin
                        wqe_opcode_err <= 1'b0;

                        eth_hdr <= {
                            (ip_ver ? 16'hDD86 : 16'h0008),
                            i_reg_mac_src_addr[7:0],
                            i_reg_mac_src_addr[15:8],
                            i_reg_mac_src_addr[23:16],
                            i_reg_mac_src_addr[31:24],
                            i_reg_mac_src_addr[39:32],
                            i_reg_mac_src_addr[47:40],
                            mac_dest_addr[7:0],
                            mac_dest_addr[15:8],
                            mac_dest_addr[23:16],
                            mac_dest_addr[31:24],
                            mac_dest_addr[39:32],
                            mac_dest_addr[47:40]
                        };

                        if (ip_ver) begin  // IPv6
                            ip_hdr <= {
                                ip_dest_addr[7:0],
                                ip_dest_addr[15:8],
                                ip_dest_addr[23:16],
                                ip_dest_addr[31:24],
                                ip_dest_addr[39:32],
                                ip_dest_addr[47:40],
                                ip_dest_addr[55:48],
                                ip_dest_addr[63:56],
                                ip_dest_addr[71:64],
                                ip_dest_addr[79:72],
                                ip_dest_addr[87:80],
                                ip_dest_addr[95:88],
                                ip_dest_addr[103:96],
                                ip_dest_addr[111:104],
                                ip_dest_addr[119:112],
                                ip_dest_addr[127:120],
                                i_reg_ipv6_src_addr[7:0],
                                i_reg_ipv6_src_addr[15:8],
                                i_reg_ipv6_src_addr[23:16],
                                i_reg_ipv6_src_addr[31:24],
                                i_reg_ipv6_src_addr[39:32],
                                i_reg_ipv6_src_addr[47:40],
                                i_reg_ipv6_src_addr[55:48],
                                i_reg_ipv6_src_addr[63:56],
                                i_reg_ipv6_src_addr[71:64],
                                i_reg_ipv6_src_addr[79:72],
                                i_reg_ipv6_src_addr[87:80],
                                i_reg_ipv6_src_addr[95:88],
                                i_reg_ipv6_src_addr[103:96],
                                i_reg_ipv6_src_addr[111:104],
                                i_reg_ipv6_src_addr[119:112],
                                i_reg_ipv6_src_addr[127:120],
                                ttl,
                                8'h11,
                                ipv6_len[7:0],
                                ipv6_len[15:8],
                                q_id[7:0],
                                q_id[15:8],
                                {dscp[1:0], 2'b00, 4'h0},
                                {4'h6, dscp[5:2]}
                            };
                        end else begin  // IPv4
                            ip_hdr[159:96] <= {ip_dest_addr[7:0], ip_dest_addr[15:8], ip_dest_addr[23:16], ip_dest_addr[31:24], i_reg_ipv4_src_addr[7:0], i_reg_ipv4_src_addr[15:8], i_reg_ipv4_src_addr[23:16], i_reg_ipv4_src_addr[31:24]};
                            ip_hdr[95:80]  <= {~ip_hdr_chksum[7:0], ~ip_hdr_chksum[15:8]};  // IPv4 头校验
                            ip_hdr[79:0]   <= {8'h11, ttl, 16'h0040, ipv4_identification[7:0], ipv4_identification[15:8], ipv4_len[7:0], ipv4_len[15:8], dscp[5:0], 2'b00, 8'h45};
                        end

                        udp_hdr <= {16'h0000, ipv6_len[7:0], ipv6_len[15:8], 16'hB712, i_reg_udp_src_port[7:0], i_reg_udp_src_port[15:8]};  //UDP destination port 4791 (0x12B7), stored in little-endian byte order as 16'hB712

                        bth_hdr[95:72] <= {o_reg_sq_psn[7:0], o_reg_sq_psn[15:8], o_reg_sq_psn[23:16]};
                        bth_hdr[70:32] <= {7'h00, dest_qpid[7:0], dest_qpid[15:8], dest_qpid[23:16], 8'h00};

                        bth_hdr[31:16] <= {i_reg_pkey[7:0], i_reg_pkey[15:8]};
                        bth_hdr[14:8]  <= {1'b0, payload_pad, 4'h0};

                        bth_hdr[15]    <= 1'b0;

                        if (wqe_opcode == 8'h00) begin  // RDMA WRITE
                            o_out_wr_pkt_cnt <= o_out_wr_pkt_cnt + 1'b1;

                            bth_hdr[71]      <= (~|rem_len);
                            if (PSN_based_retransmission == 1 && retransmit_wqe == 1'b1) begin
                                if (|rem_len) begin
                                    bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                    push_to_os_buf <= first_trans;
                                end else begin
                                    bth_hdr[7:0]   <= 8'h08;  //WRITE_LAST
                                    push_to_os_buf <= first_trans;
                                end
                            end else if (first_trans && |rem_len) begin
                                bth_hdr[7:0]   <= 8'h06;  //WRITE_FIRST
                                push_to_os_buf <= 1'b1;
                            end else if (first_trans && ~|rem_len) begin
                                bth_hdr[7:0]   <= 8'h0A;  //WRITE_ONLY
                                push_to_os_buf <= 1'b1;
                            end else if (~first_trans && |rem_len) begin
                                bth_hdr[7:0]   <= 8'h07;  //WRITE_MIDDLE
                                push_to_os_buf <= 1'b0;
                            end else if (~first_trans && ~|rem_len) begin
                                bth_hdr[7:0]   <= 8'h08;  //WRITE_LAST
                                push_to_os_buf <= 1'b0;
                            end
                        end else begin
                            if (o_qp_id != QP1) begin
                                // 防御：不应到达此分支（RD_Q_INFO 已拦截非法 opcode）
                                // QP1除外
                                hdr_gen_cs       <= ILL_OPCODE;
                                o_wqe_fatal_qpid <= o_qp_id;
                            end
                        end

                        o_reg_sq_psn <= o_reg_sq_psn + 1'b1;

                        o_out_mad_pkt_cnt <= o_out_mad_pkt_cnt + (q_id == QP1 ? 1'b1 : 1'b0);

                        local_offset <= local_offset + payload_len;
                        remote_offset <= remote_offset + payload_len;
                        reth_hdr <= {
                            dma_len[7:0],
                            dma_len[15:8],
                            dma_len[23:16],
                            dma_len[31:24],
                            r_key[7:0],
                            r_key[15:8],
                            r_key[23:16],
                            r_key[31:24],
                            remote_offset[7:0],
                            remote_offset[15:8],
                            remote_offset[23:16],
                            remote_offset[31:24],
                            remote_offset[39:32],
                            remote_offset[47:40],
                            remote_offset[55:48],
                            remote_offset[63:56]
                        };

                        // {o_hdr_data[1023:1008], o_hdr_data[1007:944], o_hdr_data[943], o_hdr_data[942:928]}
                        o_hdr_data[1023:928] <= {payload_len, local_offset, 1'b0, q_id[14:0]};

                        // halt 检查: 仅 QP1/合法 WRITE 路径进入 HDR_STO/DROP_WQE，else 分支产生 ILL_OPCODE
                        if ((o_qp_id == QP1) || (wqe_opcode == 8'h00)) begin
                            if (~(i_wqe_halt && (i_wqe_halted_qpid == o_qp_id))) begin
                                hdr_gen_cs <= HDR_STO;
                            end else begin
                                hdr_gen_cs          <= DROP_WQE;
                                o_wqe_dropped       <= first_trans;
                                o_wqe_dropped_qpid  <= o_qp_id;
                                ipv4_identification <= ipv4_identification - (ip_ver ? 1'b0 : 1'b1);
                            end
                        end
                    end
                end

                // 等 wqe_push_pending 清零后，将寄存器→o_hdr_data，设 o_hdr_len
                HDR_STO: begin

                    // 必须等 ~wqe_push_pending（OSQ push 完成）才执行
                    if (~wqe_push_pending) begin
                        o_hdr_data[111:0] <= eth_hdr;
                        if (ip_ver) begin
                            o_hdr_data[431:112] <= ip_hdr;
                            o_hdr_data[495:432] <= udp_hdr;
                            o_hdr_data[591:496] <= bth_hdr;
                            if (reth_present) begin
                                o_hdr_data[719:592] <= reth_hdr;
                                o_hdr_len           <= 8'h5A;  //90B header
                            end else begin
                                o_hdr_data[751:624] <= 128'd0000;
                                o_hdr_len           <= 8'h4A;  //74B header
                                o_hdr_data[623:592] <= 32'd0;
                            end
                        end else begin
                            o_hdr_data[271:112] <= ip_hdr[159:0];
                            o_hdr_data[335:272] <= udp_hdr;
                            o_hdr_data[431:336] <= bth_hdr;
                            if (reth_present) begin
                                o_hdr_data[559:432] <= reth_hdr;
                                o_hdr_len           <= 8'h46;  //70B header
                                o_hdr_data[591:560] <= 32'd0;
                            end else begin
                                o_hdr_len           <= 8'h36;  //54B header
                                o_hdr_data[463:432] <= 32'd0;
                                o_hdr_data[591:464] <= 128'd0000;
                            end
                        end
                        push_to_os_buf <= 1'b0;
                        first_trans    <= 1'b0;  // 后续包不再是 first
                        o_reg_wr_en    <= 1'b1;  // 正常 SQ WQE → 写回 PSN/MSN

                        // rem_len > 0 说明还有分片
                        if ((wqe_opcode == 8'h00) && |rem_len) begin
                            hdr_gen_cs <= WAIT0;
                        end else begin
                            hdr_gen_cs  <= IDLE;
                            o_reg_rd_en <= 1'b0;
                        end
                        reth_present <= 1'b0;
                    end
                end

                // 当 WQE opcode 不在白名单时进入
                ILL_OPCODE: begin
                    if (~o_wqe_fatal_qp_vld) begin
                        hdr_gen_cs     <= IDLE;
                        o_reg_rd_en    <= 1'b0;
                        o_reg_wr_en    <= 1'b1;
                        wqe_opcode_err <= 1'b1;
                        push_to_os_buf <= 1'b1;
                    end
                end

                // 当 QP 被 NACK / halt 时丢弃 WQE
                DROP_WQE: begin
                    hdr_gen_cs    <= IDLE;
                    o_wqe_dropped <= 1'b0;
                end
                default: begin
                    hdr_gen_cs <= IDLE;
                end
            endcase
        end
    end


    // OSQ Push 元数据
    always @(posedge core_clk) begin
        if (core_rst) begin
            o_wqp_req              <= 1'b0;
            o_wqe_qpid             <= {C_MAX_QID_WIDTH{1'b0}};
            wqe_push_pending       <= 1'b0;
            o_wqe_opcode           <= 8'h00;
            o_wqe_send_psn         <= 24'd0;
            o_wqe_send_msn         <= 24'd0;
            o_wqe_explicit_ack_req <= 1'b0;
            o_wqe_retried          <= 1'b0;
            o_last_out_pkt_info    <= 32'd0;
            o_wqe_qp_to            <= 5'h00;
            o_wqe_fatal_push       <= 1'b0;
        end else begin

            if (push_to_os_buf && ~wqe_push_pending && ~(i_wqe_halt && (i_wqe_halted_qpid == o_qp_id))) begin
                o_wqp_req              <= push_to_os_buf;
                wqe_push_pending       <= 1'b1;
                o_wqe_qpid             <= o_qp_id;
                o_wqe_qp_to            <= reg_qp_to;
                o_wqe_retried          <= retried;
                o_wqe_opcode           <= wqe_opcode;  //bth_hdr[7:0];
                o_wqe_explicit_ack_req <= bth_hdr[71];
                o_wqe_fatal_push       <= wqe_pushed_in_fatal;

                if (wqe_opcode == 8'h00) begin
                    case (max_pmtu[12:8])
                        5'h01: begin  // 256
                            o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:8] + |(dma_len[7:0]) - 'h2;
                        end
                        5'h02: begin  // 512
                            o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:9] + |(dma_len[8:0]) - 'h2;
                        end
                        5'h04: begin  // 1024
                            o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:10] + |(dma_len[9:0]) - 'h2;
                        end
                        5'h08: begin  // 2048
                            o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:11] + |(dma_len[10:0]) - 'h2;
                        end
                        5'h10: begin  // 4096
                            o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:12] + |(dma_len[11:0]) - 'h2;
                        end
                        default: begin  // 256
                            o_wqe_send_psn <= o_reg_sq_psn + dma_len[31:8] + |(dma_len[7:0]) - 'h2;
                        end
                    endcase
                end else begin
                    o_wqe_send_psn <= o_reg_sq_psn - 1'b1;
                end
                o_wqe_send_msn      <= o_reg_sq_msn;
                o_last_out_pkt_info <= {{(12 - C_MAX_QID_WIDTH) {1'b0}}, o_qp_id, (o_reg_sq_psn[11:0] - 1'b1), wqe_opcode};
            end else if (i_wqp_resp) begin
                wqe_push_pending <= 1'b0;
                o_wqp_req        <= 1'b0;
            end
        end
    end
    /////////////////////



    assign o_dma_in_idle = dma_in_idle;

endmodule
