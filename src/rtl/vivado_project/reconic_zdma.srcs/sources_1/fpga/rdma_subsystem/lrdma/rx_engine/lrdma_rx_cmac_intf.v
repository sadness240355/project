`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_rx_cmac_intf — RX 引擎数据包过滤模块
//
// 【模块概述】
// 直接连接 CMAC（100G 以太网 MAC）AXI-Stream 输出，对每个接收帧执行
// 协议分类、多级校验和缓冲区分配三项关键操作。
//
// 【协议分类】
// - 第一拍提取 EtherType 判断 IPv4（0x0800）/ IPv6（0x86DD）
// - 从 BTH 解析 Transport Type（高 3 位）和 Opcode（低 5 位）
// - 请求包筛出 MAD 候选 → 请求缓冲区路径
// - 响应包（RC ACK，opcode=0x11）→ 响应缓冲区路径
// - 其他包类型标记丢弃
//
// 【三重校验（并行执行）】
// 1. FCS 校验：由 CMAC tuser 信号携带
// 2. IPv4 头部校验和：内部累加器流水线计算，支持 20B 标准头
// 3. ICRC：实例化 lrdma_rx_icrc_rocev2_512b 子模块 CRC32 计算
// 三项结果通过各自 FIFO 同步后合并到描述符
//
// 【缓冲区管理】
// - 请求/响应分别维护独立的包数据缓冲区和头部缓冲区
// - 头部提取 FSM（3 状态）：每包前两拍捕获 128B 完整协议头
// - 同时提取 BTH 目标 QP 号存入窄位宽缓冲区（优化查找延迟）
//
// 【包描述符格式（25-bit）】
// {pkt_buf_offset[24:16], pkt_len[15:3],
//  FCS_err[2], IPv4_chksum_err[1], ICRC_err[0]}
//
// 【内部 FIFO】
// - FIFO1（lrdma_common_reg_fifo_sync, 深度16）：基础包信息（pkt_id, 长度, IP版本, req/rsp类型）
// - FIFO2（lrdma_common_reg_fifo_sync, 深度16）：IPv4 校验和结果
// - FIFO3（lrdma_common_reg_fifo_sync，深度16）：ICRC 校验结果
// - FIFO4-req/rsp：最终描述符，传给 lrdma_rx_req_hdr_validator / lrdma_rx_resp_hdr_validator
///////////////////////////////////////////////////////////////////////////////

`timescale 1 ps / 1 ps

module lrdma_rx_cmac_intf #(
    parameter C_NUM_QP                 = 256,  // 32
    parameter C_QP_INDX_WIDTH          = 8,    // 5
    parameter PKT_DESC_FIFO_DATA_WIDTH = 25,   // 包描述符 FIFO 数据位宽：{pkt_buf_offset, pkt_len, fcs_err, ipv4_chksum_err, icrc_err}
    parameter C_REQ_BUF_ADDR_WIDTH     = 9,    // 请求缓冲区地址位宽(512个位置)-9
    parameter C_HDR_BUF_ADDR_WIDTH     = 9     // 包头缓冲区地址位宽-9
) (
    // System interface
    input clk,
    input reset,

    // CMAC interface -- CMAC数据流接口
    // ------------------------------
    input         s_axis_tvalid,
    input [511:0] s_axis_tdata,
    input [ 63:0] s_axis_tkeep,
    input         s_axis_tlast,
    input [  0:0] s_axis_tuser,   // CMAC 帧有效标志：1 表示该帧无 FCS 错误，0 表示该帧错误；仅在 tlast 有效时使用

    // LRDMA configuration -- LRDMA配置接口
    // ------------------------------
    input lrdma_en,  // RDMA网卡使能

    // 统计信息
    // ------------------------------
    output reg [31:0] o_inc_all_pkt_cnt,  // 总包计数
    output reg [31:0] o_inc_drop_pkt_cnt, // 丢包计数

    output reg [                7:0] o_last_in_pkt_opcode,  // 最后一个包的操作码
    output reg [C_QP_INDX_WIDTH-1:0] o_last_in_pkt_qpid,    // 最后一个包的QP ID
    output reg [               15:0] o_last_in_pkt_psn,     // 最后一个包的psn

    // 包缓冲区反压信号
    // ------------------------------
    input req_pkt_buf_almost_full,  // 请求包缓冲区快满

    // req_pkt_desc_fifo interface -- 请求包包描述符FIFO接口（这些FIFO都在该模块中）
    // 请求包描述符
    // ------------------------------
    output [PKT_DESC_FIFO_DATA_WIDTH-1:0] req_pkt_desc_fifo_dout,
    input                                 rd_req_pkt_desc_fifo,    // 读使能

    // rsp_pkt_desc_fifo interface -- 响应包包描述符FIFO接口（这些FIFO都在该模块中）
    // 响应包描述符
    // ------------------------------
    output [PKT_DESC_FIFO_DATA_WIDTH-1:0] rsp_pkt_desc_fifo_dout,
    input                                 rd_rsp_pkt_desc_fifo,    // 读使能

    // req_pkt_buf interface -- 数据缓冲区接口-端口A（RAM 在 RX 引擎中）
    // 请求包缓冲区
    // ------------------------------
    output reg [C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addra,  // 地址从0开始写
    output                                req_pkt_buf_ena,    // 使能端口a写
    output     [                    63:0] req_pkt_buf_wea,    // 写掩码（字节为单位）
    output     [                   511:0] req_pkt_buf_dina,   // 写数据

    // req_hdr_buf interface -- 请求包头缓冲区接口-端口A（RAM 在 RX 引擎中）
    // 请求包头缓冲区
    // ------------------------------
    output reg [C_HDR_BUF_ADDR_WIDTH-1:0] req_hdr_buf_addra,
    output reg                            req_hdr_buf_ena,
    output     [                    63:0] req_hdr_buf_wea,
    output     [                   511:0] req_hdr_buf_dina,

    // rsp_hdr_buf interface -- 响应包头缓冲区接口-端口A（RAM 在 RX 引擎中）
    // 响应包头缓冲区
    // ------------------------------
    output reg [C_HDR_BUF_ADDR_WIDTH-1:0] rsp_hdr_buf_addra,
    output reg                            rsp_hdr_buf_ena,
    output     [                    63:0] rsp_hdr_buf_wea,
    output     [                   511:0] rsp_hdr_buf_dina,

    output req_pkt_desc_fifo_empty,
    output rsp_pkt_desc_fifo_empty
);
    localparam QP1 = 24'd0;

    // 以太网类型定义
    localparam [15:0] ETH_TYPE_IPV4 = 16'h0800;  // IPv4以太网类型
    localparam [15:0] ETH_TYPE_IPV6 = 16'h86DD;  // IPv6以太网类型

    localparam [7:0] RDMA_UD_SEND_ONLY = 8'b011_00100;

    // 包类型定义
    localparam PKT_TYPE_REQ = 1'b0;  // 请求包
    localparam PKT_TYPE_RSP = 1'b1;  // 响应包

    // FIFO深度和位宽定义
    localparam PKT_DESC_FIFO_1_DEPTH = 16;
    localparam PKT_DESC_FIFO_1_DATA_WIDTH = C_REQ_BUF_ADDR_WIDTH + 15;  // 9+15=24
    localparam PKT_DESC_FIFO_1_WRCNTR_WIDTH = 5;

    localparam PKT_DESC_FIFO_2_DEPTH = 16;
    localparam PKT_DESC_FIFO_2_DATA_WIDTH = 1;
    localparam PKT_DESC_FIFO_2_WRCNTR_WIDTH = 5;

    localparam PKT_DESC_FIFO_3_DEPTH = 16;
    localparam PKT_DESC_FIFO_3_DATA_WIDTH = 1;
    localparam PKT_DESC_FIFO_3_WRCNTR_WIDTH = 5;

    localparam PKT_DESC_FIFO_DEPTH = 256;
    localparam PKT_DESC_FIFO_WRCNTR_WIDTH = 9;

    localparam PIPELINE_MODE = 0;
    localparam DESC_FIFO_PROG_FULL_TH = PIPELINE_MODE ? 6 : 4;


    integer                                  i;

    reg                                      s_axis_tidle;
    wire                                     s_axis_tfirst;
    reg     [                           6:0] s_axis_tbytes;

    reg                                      tvalid;
    reg     [                         511:0] tdata;
    reg     [                          63:0] tkeep;
    reg     [                           0:0] tuser;
    reg                                      tfirst;
    reg                                      tlast;

    wire    [                           7:0] ipv4_bth_opcode;
    wire    [                           7:0] ipv6_bth_opcode;
    wire    [                          15:0] eth_type;
    wire    [                           7:0] bth_opcode;
    wire    [                          23:0] ipv4_bth_qp_dest_addr;
    wire    [                          23:0] ipv6_bth_qp_dest_addr;
    wire    [                          23:0] bth_qp_dest_addr;
    reg     [           C_QP_INDX_WIDTH-1:0] bth_qp_dest_addr_r;
    wire    [                          23:0] ipv4_bth_psn;
    wire    [                          23:0] ipv6_bth_psn;
    wire    [                          23:0] bth_psn;

    reg                                      ipv6;

    wire                                     rsp_pkt_rcvd;
    wire                                     rdma_mad_pkt;
    wire                                     rdma_mad_opcode;

    reg                                      pkt_discarded;  // 本包是否丢弃：1-丢弃，0-接收

    reg                                      pkt_type;  // 包类型：0-请求包，1-响应包
    reg     [                          12:0] pkt_len;  // 包总长度，不包括FCS

    reg     [      C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_offset;
    wire    [      C_REQ_BUF_ADDR_WIDTH-1:0] pkt_buf_offset;  // 缓冲区偏移，例如某数据包突发传输 3 拍后偏移累加为 3，表示下一个数据包存储的起始地址

    reg     [PKT_DESC_FIFO_1_DATA_WIDTH-1:0] pkt_desc_fifo_1_din;
    reg     [PKT_DESC_FIFO_1_DATA_WIDTH-1:0] pkt_desc_fifo_1_din_r;
    wire    [PKT_DESC_FIFO_1_DATA_WIDTH-1:0] pkt_desc_fifo_1_dout;
    reg                                      wr_pkt_desc_fifo_1;
    reg                                      wr_pkt_desc_fifo_1_r;
    wire                                     rd_pkt_desc_fifo_1;
    wire                                     pkt_desc_fifo_1_empty;
    wire                                     pkt_desc_fifo_1_full;
    wire                                     pkt_desc_fifo_1_almost_full;

    reg     [                          17:0] ipv4_hdr_chksum_i1;
    reg     [                          17:0] ipv4_hdr_chksum_i2;
    reg     [                          17:0] ipv4_hdr_chksum_i3;

    reg     [                          19:0] ipv4_hdr_chksum_ii1;
    wire    [                          16:0] ipv4_hdr_chksum_ii2;
    wire                                     ipv4_hdr_chksum_err;  // 校验和检查标志：错误-1，正确-0

    wire                                     ipv4_hdr_chk_en;
    reg     [                           1:0] ipv4_hdr_chk_st;

    wire    [PKT_DESC_FIFO_2_DATA_WIDTH-1:0] pkt_desc_fifo_2_din;
    wire    [PKT_DESC_FIFO_2_DATA_WIDTH-1:0] pkt_desc_fifo_2_dout;
    wire                                     wr_pkt_desc_fifo_2;
    wire                                     rd_pkt_desc_fifo_2;

    wire                                     pkt_crc_err;
    wire                                     crc_calc_done;

    reg     [PKT_DESC_FIFO_3_DATA_WIDTH-1:0] pkt_desc_fifo_3_din;
    wire    [PKT_DESC_FIFO_3_DATA_WIDTH-1:0] pkt_desc_fifo_3_dout;
    reg                                      wr_pkt_desc_fifo_3;
    wire                                     rd_pkt_desc_fifo_3;
    wire                                     pkt_desc_fifo_3_empty;

    wire    [  PKT_DESC_FIFO_DATA_WIDTH-1:0] pkt_desc_fifo_din;
    reg     [  PKT_DESC_FIFO_DATA_WIDTH-1:0] pkt_desc_fifo_din_r;
    reg     [  PKT_DESC_FIFO_DATA_WIDTH-1:0] pkt_desc_fifo_din_p;

    wire                                     wr_req_pkt_desc_fifo;
    reg                                      wr_req_pkt_desc_fifo_r;
    reg                                      wr_req_pkt_desc_fifo_p;
    wire                                     req_pkt_desc_fifo_full;

    wire                                     wr_rsp_pkt_desc_fifo;
    reg                                      wr_rsp_pkt_desc_fifo_r;
    reg                                      wr_rsp_pkt_desc_fifo_p;
    wire                                     rsp_pkt_desc_fifo_full;

    reg     [                           1:0] req_hdr_buf_fsm_cs;
    reg     [                           1:0] req_hdr_buf_fsm_ns;
    reg     [                           1:0] req_hdr_buf_addra_incr;

    reg     [                           1:0] rsp_hdr_buf_fsm_cs;
    reg     [                           1:0] rsp_hdr_buf_fsm_ns;
    reg     [                           1:0] rsp_hdr_buf_addra_incr;

    // ============================= 第一拍检测 =============================
    always @(posedge clk)
        if (reset) s_axis_tidle <= 1'b1;
        else if (s_axis_tvalid) s_axis_tidle <= s_axis_tlast;  // 本拍是包末拍 (tlast=1) 时，下一拍 s_axis_tidle 拉高，标记新包到来
    // 包开始检测
    assign s_axis_tfirst = s_axis_tidle & s_axis_tvalid;  // s_axis_tfirst为高表示包的第一拍

    // ============================= 数据清0处理 =============================
    // valid信号延迟一拍，以便与数据对齐
    always @(posedge clk)
        if (reset) tvalid <= 1'b0;
        else tvalid <= s_axis_tvalid;

    // 数据清零处理(无效字节置0)，1周期执行完成
    always @(posedge clk)
        for (i = 'd0; i < 'd64; i = i + 1)
            if (s_axis_tvalid && s_axis_tkeep[i]) tdata[i*8+:8] <= s_axis_tdata[i*8+:8];
            else tdata[i*8+:8] <= 8'h00;

    // 其余控制信号也延迟一拍，以便与数据对齐
    always @(posedge clk)
        if (s_axis_tvalid) begin
            tkeep  <= s_axis_tkeep;
            tuser  <= s_axis_tuser;
            tfirst <= s_axis_tidle;
            tlast  <= s_axis_tlast;
        end else begin
            tkeep  <= {64{1'b0}};
            tuser  <= 1'b0;
            tfirst <= 1'b0;
            tlast  <= 1'b0;
        end

    // ============================= 协议字段提取 =============================
    // BTH操作码提取 (Base Transport Header)
    assign ipv4_bth_opcode       = s_axis_tdata[42*8+:8];
    assign ipv6_bth_opcode       = s_axis_tdata[62*8+:8];
    assign eth_type              = {s_axis_tdata[12*8+:8], s_axis_tdata[13*8+:8]};  // 以太网类型字段
    assign bth_opcode            = (eth_type == ETH_TYPE_IPV4) ? ipv4_bth_opcode : ipv6_bth_opcode;
    // 目地QP号提取
    assign ipv4_bth_qp_dest_addr = {s_axis_tdata[47*8+:8], s_axis_tdata[48*8+:8], s_axis_tdata[49*8+:8]} >= 24'd1 ? {s_axis_tdata[47*8+:8], s_axis_tdata[48*8+:8], s_axis_tdata[49*8+:8]} - 24'd1 : 24'd0;
    assign ipv6_bth_qp_dest_addr = {s_axis_tdata[3*8+:8], s_axis_tdata[4*8+:8], s_axis_tdata[5*8+:8]} >= 24'd1 ? {s_axis_tdata[3*8+:8], s_axis_tdata[4*8+:8], s_axis_tdata[5*8+:8]} - 24'd1 : 24'd0;
    assign bth_qp_dest_addr      = (eth_type == ETH_TYPE_IPV4) ? ipv4_bth_qp_dest_addr : ipv6_bth_qp_dest_addr;
    // 包序列号(PSN)提取
    assign ipv4_bth_psn          = {s_axis_tdata[51*8+:8], s_axis_tdata[52*8+:8], s_axis_tdata[53*8+:8]};
    assign ipv6_bth_psn          = {s_axis_tdata[7*8+:8], s_axis_tdata[8*8+:8], s_axis_tdata[9*8+:8]};
    assign bth_psn               = (eth_type == ETH_TYPE_IPV4) ? ipv4_bth_psn : ipv6_bth_psn;

    // ============================= 包类型判断 =============================
    // 响应包检测 (操作码范围01101-10001) ACKNOWLEDGE也是响应包
    assign rsp_pkt_rcvd          = ((bth_opcode >= 8'b000_01101) && (bth_opcode <= 8'b000_10001)) ? 1'b1 : 1'b0;
    assign rdma_mad_opcode       = (bth_opcode == RDMA_UD_SEND_ONLY) ? 1'b1 : 1'b0;
    assign rdma_mad_pkt          = (eth_type == ETH_TYPE_IPV4) ? (rdma_mad_opcode && (bth_qp_dest_addr == QP1)) : rdma_mad_opcode;

    // 包类型设置
    always @(posedge clk)
        if (s_axis_tfirst) begin
            if (rsp_pkt_rcvd) pkt_type <= PKT_TYPE_RSP;  // 响应包
            else pkt_type <= PKT_TYPE_REQ;  // 请求包
        end

    // ============================= 包ID和长度计算 =============================
    // 计算单次传输的有效字节数
    always @(*) begin
        s_axis_tbytes = 7'd1;
        for (i = 'd0; i < 'd63; i = i + 1) s_axis_tbytes = s_axis_tbytes + s_axis_tkeep[i+1];
    end

    // 累计整个数据包的总有效字节数（包括ICRC，不包括FCS）
    always @(posedge clk) if (s_axis_tvalid) pkt_len <= s_axis_tidle ? s_axis_tbytes : (pkt_len + s_axis_tbytes);

    // ============================= 丢包逻辑 =============================
    always @(posedge clk)
        if (reset) begin
            pkt_discarded        <= 1'b1;
            o_inc_drop_pkt_cnt   <= 32'd0;
            o_last_in_pkt_opcode <= 8'h02;
            o_last_in_pkt_qpid   <= {C_QP_INDX_WIDTH{1'b0}};
            o_last_in_pkt_psn    <= 16'h0000;
            ipv6                 <= 1'b0;
        end else if (s_axis_tfirst) begin
            if (!lrdma_en) begin  // 未使能IP
                pkt_discarded <= 1'b1;
            end else if (rsp_pkt_rcvd && bth_opcode != 8'b000_10001) begin  // 响应包不是 ACK, 丢弃
                pkt_discarded <= 1'b1;
            end else if (!rsp_pkt_rcvd && !rdma_mad_pkt) begin  // 请求包不是 MAD 包, 丢弃
                pkt_discarded <= 1'b1;
            end else if (pkt_desc_fifo_1_almost_full || pkt_desc_fifo_1_full) begin  // FIFO1满
                pkt_discarded      <= 1'b1;
                o_inc_drop_pkt_cnt <= o_inc_drop_pkt_cnt + 'd1;
            end else if (rsp_pkt_rcvd && rsp_pkt_desc_fifo_full) begin  // 响应描述符缓冲区满
                pkt_discarded      <= 1'b1;
                o_inc_drop_pkt_cnt <= o_inc_drop_pkt_cnt + 'd1;
            end else if (!rsp_pkt_rcvd && (req_pkt_buf_almost_full | req_pkt_desc_fifo_full)) begin  // 请求包缓冲区将满 | 请求描述符缓冲区满
                pkt_discarded      <= 1'b1;
                o_inc_drop_pkt_cnt <= o_inc_drop_pkt_cnt + 'd1;
            end else begin  // 正常接收
                pkt_discarded        <= 1'b0;
                // 记录包信息
                o_last_in_pkt_opcode <= bth_opcode;
                if (eth_type == ETH_TYPE_IPV4) begin
                    o_last_in_pkt_qpid <= ipv4_bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];
                    o_last_in_pkt_psn  <= ipv4_bth_psn[15:0];
                end else ipv6 <= 1'b1;
            end
        end else if (ipv6 && s_axis_tvalid) begin
            ipv6               <= 1'b0;
            // 记录包信息
            o_last_in_pkt_qpid <= ipv6_bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];
            o_last_in_pkt_psn  <= ipv6_bth_psn[15:0];
        end

    // 总包计数
    always @(posedge clk)
        if (reset) o_inc_all_pkt_cnt <= 32'd0;
        else if (lrdma_en && tlast) o_inc_all_pkt_cnt <= o_inc_all_pkt_cnt + 'd1;

    // ============================= 包缓冲区地址偏移管理 =============================
    // 包缓冲区地址递增
    always @(posedge clk)
        if (reset) begin
            req_pkt_buf_offset <= {(C_REQ_BUF_ADDR_WIDTH) {1'b0}};
        end else if (!pkt_discarded && tlast && pkt_type == PKT_TYPE_REQ) begin
            req_pkt_buf_offset <= req_pkt_buf_addra + 'd1;
        end

    // 缓冲区偏移，例如某数据包突发传输 3 拍后 pkt_buf_offset = 3，按拍累加；响应包共享首地址，故偏移恒为 0
    assign pkt_buf_offset = (pkt_type == PKT_TYPE_RSP) ? {(C_REQ_BUF_ADDR_WIDTH) {1'b0}} : req_pkt_buf_offset;
    // ============================ FIFO =============================
    // FIFO 1: 基本包信息
    always @(posedge clk) begin
        if (reset) begin
            pkt_desc_fifo_1_din <= 0;
        end else begin
            pkt_desc_fifo_1_din <= {
                pkt_type,  // 1
                pkt_buf_offset,  // C_REQ_BUF_ADDR_WIDTH, 9
                pkt_len,  // 13
                tuser[0]  // 1
            };
        end
    end

    always @(posedge clk) begin
        if (reset) wr_pkt_desc_fifo_1 <= 0;
        else wr_pkt_desc_fifo_1 <= ~pkt_discarded & tlast;
    end

    always @(posedge clk) begin
        pkt_desc_fifo_1_din_r <= pkt_desc_fifo_1_din;
        wr_pkt_desc_fifo_1_r  <= wr_pkt_desc_fifo_1;
        // IPv4: BTH DestQP 在 beat 0 (字节47-49) 直接可用，首拍锁存
        // IPv6: BTH DestQP 在 beat 1 (字节67-69) 才可用，需延迟到 ipv6 标志有效时锁存
        if (s_axis_tfirst && eth_type == ETH_TYPE_IPV4) bth_qp_dest_addr_r <= ipv4_bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];
        else if (ipv6 && s_axis_tvalid) bth_qp_dest_addr_r <= ipv6_bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];
    end

    //=====================================
    // pkt_desc_fifo_1 (FWFT), Default depth = 16
    //=====================================
    // FIFO 1：存储该包基本信息(pkt_type, pkt buffer offset, pkt_len, tuser)
    // bit[23]      pkt_type
    // bit[22:14]   pkt buffer offset
    // bit[13:1]    pkt_len
    // bit[0]       tuser
    lrdma_common_reg_fifo_sync #(
        .C_READ_MODE(1'b1),
        .C_FIFO_DEPTH(PKT_DESC_FIFO_1_DEPTH),
        .C_FIFO_WRCNTR_WIDTH(PKT_DESC_FIFO_1_WRCNTR_WIDTH),
        .C_FIFO_DATA_WIDTH(PKT_DESC_FIFO_1_DATA_WIDTH)
    ) pkt_desc_fifo_1 (
        .clk        (clk),
        .reset      (reset),
        .wr_en      (wr_pkt_desc_fifo_1_r),         // 这时基本信息才正确完整
        .din        (pkt_desc_fifo_1_din_r),        // 这时基本信息才正确完整
        .full       (pkt_desc_fifo_1_full),
        .almost_full(pkt_desc_fifo_1_almost_full),
        .rd_en      (rd_pkt_desc_fifo_1),
        .dout       (pkt_desc_fifo_1_dout),
        .empty      (pkt_desc_fifo_1_empty)
    );

    // IPv4头校验和计算
    always @(posedge clk) begin
        ipv4_hdr_chksum_i1  <= {tdata[14*8+:8], tdata[15*8+:8]} + {tdata[16*8+:8], tdata[17*8+:8]} + {tdata[18*8+:8], tdata[19*8+:8]} + {tdata[20*8+:8], tdata[21*8+:8]};
        ipv4_hdr_chksum_i2  <= {tdata[22*8+:8], tdata[23*8+:8]} + {tdata[24*8+:8], tdata[25*8+:8]} + {tdata[26*8+:8], tdata[27*8+:8]} + {tdata[28*8+:8], tdata[29*8+:8]};
        ipv4_hdr_chksum_i3  <= {tdata[30*8+:8], tdata[31*8+:8]} + {tdata[32*8+:8], tdata[33*8+:8]};

        ipv4_hdr_chksum_ii1 <= ipv4_hdr_chksum_i1 + ipv4_hdr_chksum_i2 + ipv4_hdr_chksum_i3;
    end

    assign ipv4_hdr_chksum_ii2 = ipv4_hdr_chksum_ii1[15:0] + ipv4_hdr_chksum_ii1[19:16];
    assign ipv4_hdr_chksum_err = (ipv4_hdr_chksum_ii2 != 17'h0_FFFF) ? 1'b1 : 1'b0;  // 校验和检查标志：错误-1，正确-0

    assign ipv4_hdr_chk_en     = ~pkt_discarded & tfirst;

    always @(posedge clk)
        if (reset) ipv4_hdr_chk_st <= 2'b00;
        else ipv4_hdr_chk_st <= {ipv4_hdr_chk_st[0], ipv4_hdr_chk_en};

    // FIFO 2: IPv4头校验和结果
    assign pkt_desc_fifo_2_din = ipv4_hdr_chksum_err;

    assign wr_pkt_desc_fifo_2  = ipv4_hdr_chk_st[1];

    //=====================================
    // pkt_desc_fifo_2 (FWFT), DEPTH = 16
    //===================================== 
    // FIFO 2：存储该包IPv4头校验和检查标志--错误-1，正确-0
    lrdma_common_reg_fifo_sync #(
        .C_READ_MODE(1'b1),
        .C_FIFO_DEPTH(PKT_DESC_FIFO_2_DEPTH),
        .C_FIFO_WRCNTR_WIDTH(PKT_DESC_FIFO_2_WRCNTR_WIDTH),
        .C_FIFO_DATA_WIDTH(PKT_DESC_FIFO_2_DATA_WIDTH)
    ) pkt_desc_fifo_2 (
        .clk        (clk),
        .reset      (reset),
        .wr_en      (wr_pkt_desc_fifo_2),    // 这时校验和检查标志才正确
        .din        (pkt_desc_fifo_2_din),   // 校验和检查标志
        .full       (),
        .almost_full(),
        .rd_en      (rd_pkt_desc_fifo_2),
        .dout       (pkt_desc_fifo_2_dout),
        .empty      ()
    );

    // ICRC校验
    lrdma_rx_icrc_rocev2_512b u_icrc_rocev2_512b_rx (
        .clk            (clk),
        .reset          (reset),
        .i_pkt_discarded(pkt_discarded),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tkeep   (s_axis_tkeep),
        .s_axis_tlast   (s_axis_tlast),
        .o_crc_calc_done(crc_calc_done),
        .o_crc_out      (),
        .o_crc_err      (pkt_crc_err)
    );

    always @(posedge clk)
        if (reset) wr_pkt_desc_fifo_3 <= 1'b0;
        else wr_pkt_desc_fifo_3 <= crc_calc_done;

    // FIFO 3: ICRC校验结果
    always @(posedge clk) pkt_desc_fifo_3_din <= pkt_crc_err;

    //=====================================
    // pkt_desc_fifo_3 (FWFT), DEPTH = 16
    //=====================================  
    // FIFO 3：存储该包ICRC校验标志--错误-1，正确-0
    lrdma_common_reg_fifo_sync #(
        .C_READ_MODE(1'b1),
        .C_FIFO_DEPTH(PKT_DESC_FIFO_3_DEPTH),
        .C_FIFO_WRCNTR_WIDTH(PKT_DESC_FIFO_3_WRCNTR_WIDTH),
        .C_FIFO_DATA_WIDTH(PKT_DESC_FIFO_3_DATA_WIDTH)
    ) pkt_desc_fifo_3 (
        .clk        (clk),
        .reset      (reset),
        .wr_en      (wr_pkt_desc_fifo_3),
        .din        (pkt_desc_fifo_3_din),
        .full       (),
        .almost_full(),
        .rd_en      (rd_pkt_desc_fifo_3),
        .dout       (pkt_desc_fifo_3_dout),
        .empty      (pkt_desc_fifo_3_empty)
    );

    // 最终包描述符组装
    // -----------------------------------------------------------------------------
    // 包描述符格式
    // bit[24:16]   pkt buffer offset
    // bit[15:3]    pkt_len
    // bit[2]       FCS error -- 0-no error, 1-error
    // bit[1]       IPv4 Header checksum error
    // bit[0]       ICRC error
    assign pkt_desc_fifo_din    = {pkt_desc_fifo_1_dout[14+:C_REQ_BUF_ADDR_WIDTH],  // pkt buffer offset
 pkt_desc_fifo_1_dout[13:1],  // pkt_len
 ~pkt_desc_fifo_1_dout[0],  // FCS error
 pkt_desc_fifo_2_dout[0],  // IPv4 Header checksum error
 pkt_desc_fifo_3_dout[0]  // ICRC error
};

    // ========================== 包分流，将包描述符按不同包类型存储进相应的FIFO中 ==========================
    // 请求包和响应包分别写入不同FIFO
    assign wr_req_pkt_desc_fifo = !pkt_desc_fifo_3_empty && !req_pkt_desc_fifo_full && ((pkt_desc_fifo_1_dout[C_REQ_BUF_ADDR_WIDTH+14] == PKT_TYPE_REQ) ? 1'b1 : 1'b0);

    assign wr_rsp_pkt_desc_fifo = !pkt_desc_fifo_3_empty && !rsp_pkt_desc_fifo_full && ((pkt_desc_fifo_1_dout[C_REQ_BUF_ADDR_WIDTH+14] == PKT_TYPE_RSP) ? 1'b1 : 1'b0);

    assign rd_pkt_desc_fifo_1   = wr_req_pkt_desc_fifo | wr_rsp_pkt_desc_fifo;
    assign rd_pkt_desc_fifo_2   = rd_pkt_desc_fifo_1;
    assign rd_pkt_desc_fifo_3   = rd_pkt_desc_fifo_1;


    generate
        if (PIPELINE_MODE == 0) begin  // PIPELINE_MODE = 0 分支

            always @(*) begin
                pkt_desc_fifo_din_r    = 'd0;
                pkt_desc_fifo_din_p    = pkt_desc_fifo_din;
                wr_req_pkt_desc_fifo_r = 1'b0;
                wr_req_pkt_desc_fifo_p = wr_req_pkt_desc_fifo;
                wr_rsp_pkt_desc_fifo_r = 1'b0;
                wr_rsp_pkt_desc_fifo_p = wr_rsp_pkt_desc_fifo;
            end

        end else begin  // PIPELINE_MODE = 1

            always @(posedge clk) begin
                if (reset) begin
                    pkt_desc_fifo_din_r    <= 'd0;
                    pkt_desc_fifo_din_p    <= 'd0;
                    wr_req_pkt_desc_fifo_r <= 1'b0;
                    wr_req_pkt_desc_fifo_p <= 1'b0;
                    wr_rsp_pkt_desc_fifo_r <= 1'b0;
                    wr_rsp_pkt_desc_fifo_p <= 1'b0;
                end else begin
                    pkt_desc_fifo_din_r    <= pkt_desc_fifo_din;
                    pkt_desc_fifo_din_p    <= pkt_desc_fifo_din_r;
                    wr_req_pkt_desc_fifo_r <= wr_req_pkt_desc_fifo;
                    wr_req_pkt_desc_fifo_p <= wr_req_pkt_desc_fifo_r;
                    wr_rsp_pkt_desc_fifo_r <= wr_rsp_pkt_desc_fifo;
                    wr_rsp_pkt_desc_fifo_p <= wr_rsp_pkt_desc_fifo_r;
                end
            end

        end
    endgenerate

    //=====================================
    // req_pkt_desc_fifo (STD), DEPTH = 256
    //===================================== 
    // FIFO 4-req_pkt：存储该请求包描述符(pkt buffer offset, pkt_len, FCS error, IPv4 Header checksum error, ICRC error)
    lrdma_common_xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),                                                                //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),                                                              //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (PKT_DESC_FIFO_DEPTH),                                                   //positive integer									
        .WRITE_DATA_WIDTH   (PKT_DESC_FIFO_DATA_WIDTH),                                              //positive integer									
        .WR_DATA_COUNT_WIDTH(PKT_DESC_FIFO_WRCNTR_WIDTH),                                            //positive integer									
        .PROG_FULL_THRESH   (PKT_DESC_FIFO_DEPTH - PKT_DESC_FIFO_1_DEPTH - DESC_FIFO_PROG_FULL_TH),  //positive integer									
        .FULL_RESET_VALUE   (0),                                                                     //positive integer; 0 or 1									
        .READ_MODE          ("fwft"),                                                                //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (1),                                                                     //positive integer;									
        .READ_DATA_WIDTH    (PKT_DESC_FIFO_DATA_WIDTH),                                              //positive integer									
        .RD_DATA_COUNT_WIDTH(PKT_DESC_FIFO_WRCNTR_WIDTH),                                            //positive integer									
        .PROG_EMPTY_THRESH  (10),                                                                    //positive integer									
        .DOUT_RESET_VALUE   ("0"),                                                                   //string									
        .WAKEUP_TIME        (0)                                                                      //positive integer; 0 or 2;									            
    ) req_pkt_desc_fifo (
        .sleep        (1'b0),
        .rst          (reset),
        .wr_clk       (clk),
        .wr_en        (wr_req_pkt_desc_fifo_p),
        .din          (pkt_desc_fifo_din_p),
        .full         (),
        .prog_full    (req_pkt_desc_fifo_full),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (rd_req_pkt_desc_fifo),
        .dout         (req_pkt_desc_fifo_dout),
        .empty        (req_pkt_desc_fifo_empty),  // 随着FIFO中最后一个数据输出而拉高
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   ()
    );

    //=====================================
    // rsp_pkt_desc_fifo (STD), DEPTH = 256
    //===================================== 
    // FIFO 4-rsp_pkt：存储该响应包描述符(pkt buffer offset, pkt_len, FCS error, IPv4 Header checksum error, ICRC error)
    lrdma_common_xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),                                                                //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),                                                              //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (PKT_DESC_FIFO_DEPTH),                                                   //positive integer									
        .WRITE_DATA_WIDTH   (PKT_DESC_FIFO_DATA_WIDTH),                                              //positive integer									
        .WR_DATA_COUNT_WIDTH(PKT_DESC_FIFO_WRCNTR_WIDTH),                                            //positive integer									
        .PROG_FULL_THRESH   (PKT_DESC_FIFO_DEPTH - PKT_DESC_FIFO_1_DEPTH - DESC_FIFO_PROG_FULL_TH),  //positive integer									
        .FULL_RESET_VALUE   (0),                                                                     //positive integer; 0 or 1									
        .READ_MODE          ("fwft"),                                                                //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (1),                                                                     //positive integer;									
        .READ_DATA_WIDTH    (PKT_DESC_FIFO_DATA_WIDTH),                                              //positive integer									
        .RD_DATA_COUNT_WIDTH(PKT_DESC_FIFO_WRCNTR_WIDTH),                                            //positive integer									
        .PROG_EMPTY_THRESH  (10),                                                                    //positive integer									
        .DOUT_RESET_VALUE   ("0"),                                                                   //string									
        .WAKEUP_TIME        (0)                                                                      //positive integer; 0 or 2;									             
    ) rsp_pkt_desc_fifo (
        .sleep        (1'b0),
        .rst          (reset),
        .wr_clk       (clk),
        .wr_en        (wr_rsp_pkt_desc_fifo_p),
        .din          (pkt_desc_fifo_din_p),
        .full         (),
        .prog_full    (rsp_pkt_desc_fifo_full),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (rd_rsp_pkt_desc_fifo),
        .dout         (rsp_pkt_desc_fifo_dout),
        .empty        (rsp_pkt_desc_fifo_empty),  // 随着最后一个数据输出而输出
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   ()
    );

    // ========================== 包与包头数据缓冲区写入控制（这些缓冲区在 RX 引擎中） ==========================
    // ======================== 请求包数据缓冲区写入控制 ========================
    assign req_pkt_buf_ena = (!pkt_discarded && (pkt_type == PKT_TYPE_REQ) && tvalid) ? 1'b1 : 1'b0;  // 该包是请求包且未丢弃--1-是（写），0-否（不写）
    // 数据缓冲区地址递增
    always @(posedge clk)
        if (reset) req_pkt_buf_addra <= {(C_REQ_BUF_ADDR_WIDTH) {1'b0}};
        else if (req_pkt_buf_ena) req_pkt_buf_addra <= req_pkt_buf_addra + 'd1;

    assign req_pkt_buf_dina = tdata;
    assign req_pkt_buf_wea  = {64{1'b1}};

    // ======================== 请求包包头缓冲区写入状态机(只存储前2个64字节)，包头有用的数据仅存在前2拍 ========================
    always @(*) begin
        req_hdr_buf_ena        = 1'b0;
        req_hdr_buf_addra_incr = 2'd1;  // 正常情况地址递增1
        req_hdr_buf_fsm_ns     = req_hdr_buf_fsm_cs;
        if (!pkt_discarded && (pkt_type == PKT_TYPE_REQ) && tvalid) begin
            case (req_hdr_buf_fsm_cs)
                2'b00: begin  // 第一拍
                    req_hdr_buf_ena = 1'b1;
                    if (tlast) req_hdr_buf_addra_incr = 2'd2;  // 短包, 地址多增1, 跳过“第二拍”，保证两两拍存储
                    else req_hdr_buf_fsm_ns = 2'b01;
                end
                2'b01: begin  // 第二拍
                    req_hdr_buf_ena = 1'b1;
                    if (tlast) req_hdr_buf_fsm_ns = 2'b00;
                    else req_hdr_buf_fsm_ns = 2'b10;
                end
                2'b10: begin  // 后续拍不写入
                    if (tlast) req_hdr_buf_fsm_ns = 2'b00;
                end
                default: begin
                    req_hdr_buf_fsm_ns = 2'b00;
                end
            endcase
        end
    end

    always @(posedge clk)
        if (reset) req_hdr_buf_fsm_cs <= 2'b00;
        else req_hdr_buf_fsm_cs <= req_hdr_buf_fsm_ns;

    always @(posedge clk)
        if (reset) req_hdr_buf_addra <= {(C_HDR_BUF_ADDR_WIDTH) {1'b0}};
        else if (req_hdr_buf_ena) req_hdr_buf_addra <= req_hdr_buf_addra + req_hdr_buf_addra_incr;

    assign req_hdr_buf_dina = tdata;
    assign req_hdr_buf_wea  = {64{1'b1}};

    // ======================== 响应包包头缓冲区写入状态机(只存储前2个64字节) ========================
    always @(*) begin
        rsp_hdr_buf_ena        = 1'b0;
        rsp_hdr_buf_addra_incr = 2'd1;
        rsp_hdr_buf_fsm_ns     = rsp_hdr_buf_fsm_cs;
        if (!pkt_discarded && (pkt_type == PKT_TYPE_RSP) && tvalid) begin
            case (rsp_hdr_buf_fsm_cs)
                2'b00: begin
                    rsp_hdr_buf_ena = 1'b1;
                    if (tlast) rsp_hdr_buf_addra_incr = 2'd2;  // 短包, 地址多增1, 跳过“第二拍”，保证两两拍存储
                    else rsp_hdr_buf_fsm_ns = 2'b01;
                end
                2'b01: begin
                    rsp_hdr_buf_ena = 1'b1;
                    if (tlast) rsp_hdr_buf_fsm_ns = 2'b00;
                    else rsp_hdr_buf_fsm_ns = 2'b10;
                end
                2'b10: begin
                    if (tlast) rsp_hdr_buf_fsm_ns = 2'b00;
                end
                default: begin
                    rsp_hdr_buf_fsm_ns = 2'b00;
                end
            endcase
        end
    end

    always @(posedge clk)
        if (reset) rsp_hdr_buf_fsm_cs <= 2'b00;
        else rsp_hdr_buf_fsm_cs <= rsp_hdr_buf_fsm_ns;

    always @(posedge clk)
        if (reset) rsp_hdr_buf_addra <= {(C_HDR_BUF_ADDR_WIDTH) {1'b0}};
        else if (rsp_hdr_buf_ena) rsp_hdr_buf_addra <= rsp_hdr_buf_addra + rsp_hdr_buf_addra_incr;

    assign rsp_hdr_buf_dina = tdata;
    assign rsp_hdr_buf_wea  = {64{1'b1}};

endmodule
