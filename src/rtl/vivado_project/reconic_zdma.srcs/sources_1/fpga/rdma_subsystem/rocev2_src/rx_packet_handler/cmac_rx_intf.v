///////////////////////////////////////////////////////////////////////////////
// Module Declaration
// 1、包分类：解析以太网、IP、BTH等协议头 和 进行FCS、IPv4头校验和、ICRC校验 以 区分请求包和响应包
// 2、缓冲区管理：根据包类型（req | rsp）将包头、完整包（不含FCS）、包描述符写入不同的缓冲区，若是包缓冲区满则丢弃该包；包描述符存储在该模块内的FIFO中，包/包头存储在rx_pkt_handler模块内的FIFO中
// 3、统计信息：统计接收的包总数、丢弃包数、最后接收包的操作码、QP ID、PSN
///////////////////////////////////////////////////////////////////////////////
// FIFO1存储包基本信息（rnr_window_active, pkt_type, pkt buffer offset, pkt_len, cmac ip核的tuser信号）
// FIFO2存储IPv4头校验和结果
// FIFO3存储ICRC校验结果
// FIFO4-req_pkt存储最终请求包描述符，FIFO4-rsp_pkt存储最终响应包描述符
//      包描述符格式：
//              bit[26]      rnr_window_active
//              bit[25:16]   pkt buffer offset
//              bit[15:3]    pkt_len
//              bit[2]       FCS error -- 0-no error, 1-error(即~tuser)
//              bit[1]       IPv4 Header checksum error
//              bit[0]       ICRC error

`timescale 1 ps / 1 ps

module cmac_rx_intf #(
    parameter C_SIM_DEBUG              = 0,    // 0
    parameter C_NUM_QP                 = 256,  // 32
    parameter C_QP_INDX_WIDTH          = 8,    // 5
    parameter PKT_DESC_FIFO_DATA_WIDTH = 26,   // 包描述符FIFO数据位宽-27
    parameter C_REQ_BUF_ADDR_WIDTH     = 9,    // 请求缓冲区地址位宽(512个位置)-9
    parameter C_RSP_BUF_ADDR_WIDTH     = 10,   // 响应缓冲区地址位宽(1024个位置)-10
    parameter C_HDR_BUF_ADDR_WIDTH     = 9     // 包头缓冲区地址位宽-9
) (
    // System interface
    input wire clk,
    input wire reset,

    // CMAC interface -- CMAC数据流接口
    // ------------------------------
    input wire         s_axis_tvalid,
    input wire [511:0] s_axis_tdata,
    input wire [ 63:0] s_axis_tkeep,
    input wire         s_axis_tlast,
    input wire [  0:0] s_axis_tuser,   // 数据包错误标志：1-有效，0-错误，与CMAC IP核定义相反，tlast为高时有效

    // XRNIC configuration -- XRNIC配置接口
    // ------------------------------
    input wire xrnic_en,  // RDMA网卡使能
    input wire bypass_en, // 旁路模式使能

    // 统计信息
    // ------------------------------
    output reg [31:0] o_inc_all_pkt_cnt,  // 总包计数
    output reg [31:0] o_inc_drop_pkt_cnt, // 丢包计数

    output reg [                7:0] o_last_in_pkt_opcode,  // 最后一个包的操作码
    output reg [C_QP_INDX_WIDTH-1:0] o_last_in_pkt_qpid,    // 最后一个包的QP ID
    output reg [               15:0] o_last_in_pkt_psn,     // 最后一个包的psn

    // miscellanous inputs/outputs
    // 包缓冲区反压信号
    // ------------------------------
    input  wire req_pkt_buf_almost_full,  // 请求包缓冲区快满
    input  wire rsp_pkt_buf_almost_full,  // 响应包缓冲区快满
    output wire pkt_discarded_to_req_hv,  // 包丢弃标志

    // 配置接口
    // ------------------------------
    input wire [C_NUM_QP-1:0] rnr_window_flag_reg,  // RNR窗口标志，相应位为1表示这个 QP 当前处于“丢包后 RNR 窗口期”，影响最终 AETH syndrome（决定发哪种 RNR NAK）


    // req_pkt_desc_fifo interface -- 请求包包描述符FIFO接口（这些FIFO都在该模块中）
    // 请求包描述符
    // ------------------------------
    output wire [PKT_DESC_FIFO_DATA_WIDTH-1:0] req_pkt_desc_fifo_dout,
    input  wire                                rd_req_pkt_desc_fifo,    // 读使能

    // rsp_pkt_desc_fifo interface -- 响应包包描述符FIFO接口（这些FIFO都在该模块中）
    // 响应包描述符
    // ------------------------------
    output wire [PKT_DESC_FIFO_DATA_WIDTH-1:0] rsp_pkt_desc_fifo_dout,
    input  wire                                rd_rsp_pkt_desc_fifo,    // 读使能

    // req_pkt_buf interface -- 数据缓冲区接口-端口A（RAM在rx_pkt_handler模块中）
    // 请求包缓冲区
    // ------------------------------
    output reg  [C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addra,  // 地址从0开始写
    output wire                            req_pkt_buf_ena,    // 使能端口a写
    output wire [                    63:0] req_pkt_buf_wea,    // 写掩码（字节为单位）
    output wire [                   511:0] req_pkt_buf_dina,   // 写数据

    // req_hdr_buf interface -- 请求包头缓冲区接口-端口A（RAM在rx_pkt_handler模块中）
    // 请求包头缓冲区
    // ------------------------------
    output reg  [C_HDR_BUF_ADDR_WIDTH-1:0] req_hdr_buf_addra,
    output reg                             req_hdr_buf_ena,
    output wire [                    63:0] req_hdr_buf_wea,
    output wire [                   511:0] req_hdr_buf_dina,

    // rsp_pkt_buf interface -- 响应包缓冲区接口-端口A（RAM在rx_pkt_handler模块中）
    // 响应包缓冲区
    // ------------------------------
    output reg  [C_RSP_BUF_ADDR_WIDTH-1:0] rsp_pkt_buf_addra,
    output wire                            rsp_pkt_buf_ena,
    output wire [                    63:0] rsp_pkt_buf_wea,
    output wire [                   511:0] rsp_pkt_buf_dina,

    // rsp_hdr_buf interface -- 响应包头缓冲区接口-端口A（RAM在rx_pkt_handler模块中）
    // 响应包头缓冲区
    // ------------------------------
    output reg  [C_HDR_BUF_ADDR_WIDTH-1:0] rsp_hdr_buf_addra,
    output reg                             rsp_hdr_buf_ena,
    output wire [                    63:0] rsp_hdr_buf_wea,
    output wire [                   511:0] rsp_hdr_buf_dina,

    // Debug signals -- 调试信号
    // ------------------------------
    input  wire        i_clr_debug_sts,
    output reg         pkt_dropped_sticky,
    output reg         req_pkt_buf_almost_full_sticky,
    output reg         rsp_pkt_buf_almost_full_sticky,
    output reg  [15:0] pkt_desc_fifo_1_unoccupied_r,
    output reg         pkt_desc_fifo_1_full_sticky,
    output reg  [15:0] req_pkt_desc_fifo_unoccupied_r,
    output reg         req_pkt_desc_fifo_full_sticky,
    output reg  [15:0] rsp_pkt_desc_fifo_unoccupied_r,
    output reg         rsp_pkt_desc_fifo_full_sticky,
    output wire        pkt_desc_fifo_3_empty,
    output wire        req_pkt_desc_fifo_empty,
    output wire        rsp_pkt_desc_fifo_empty,
    output reg         pkt_fcs_err_sticky,
    output reg         pkt_crc_err_sticky
);

    // 以太网类型定义
    localparam [15:0] ETH_TYPE_IPV4 = 16'h0800;  // IPv4以太网类型
    localparam [15:0] ETH_TYPE_IPV6 = 16'h86DD;  // IPv6以太网类型

    // 包类型定义
    localparam PKT_TYPE_REQ = 1'b0;  // 请求包
    localparam PKT_TYPE_RSP = 1'b1;  // 响应包

    // FIFO深度和位宽定义
    localparam PKT_DESC_FIFO_1_DEPTH = 16;
    localparam PKT_DESC_FIFO_1_DATA_WIDTH = C_SIM_DEBUG ? (C_RSP_BUF_ADDR_WIDTH + 48) : (C_RSP_BUF_ADDR_WIDTH + 16);
    localparam PKT_DESC_FIFO_1_WRCNTR_WIDTH = 5;

    localparam PKT_DESC_FIFO_2_DEPTH = 16;
    localparam PKT_DESC_FIFO_2_DATA_WIDTH = C_SIM_DEBUG ? 33 : 1;
    localparam PKT_DESC_FIFO_2_WRCNTR_WIDTH = 5;

    localparam PKT_DESC_FIFO_3_DEPTH = 16;
    localparam PKT_DESC_FIFO_3_DATA_WIDTH = C_SIM_DEBUG ? 33 : 1;
    localparam PKT_DESC_FIFO_3_WRCNTR_WIDTH = 5;

    localparam PKT_DESC_FIFO_DEPTH = 256;
    localparam PKT_DESC_FIFO_WRCNTR_WIDTH = 9;

    localparam PIPELINE_MODE = 0;
    localparam DESC_FIFO_PROG_FULL_TH = PIPELINE_MODE ? 6 : 4;


    integer i;
    genvar gi;

    reg                                   s_axis_tidle;
    wire                                  s_axis_tfirst;
    reg  [                           6:0] s_axis_tbytes;

    reg                                   tvalid;
    reg  [                         511:0] tdata;
    reg  [                          63:0] tkeep;
    reg  [                           0:0] tuser;
    reg                                   tfirst;
    reg                                   tlast;

    wire [                           7:0] ipv4_bth_opcode;
    wire [                           7:0] ipv6_bth_opcode;
    wire [                          15:0] eth_type;
    wire [                           7:0] bth_opcode;
    wire [                          23:0] ipv4_bth_qp_dest_addr;
    wire [                          23:0] ipv6_bth_qp_dest_addr;
    wire [                          23:0] bth_qp_dest_addr;
    reg  [                          23:0] bth_qp_dest_addr_r;
    wire [                          23:0] ipv4_bth_psn;
    wire [                          23:0] ipv6_bth_psn;
    wire [                          23:0] bth_psn;

    reg                                   ipv6;

    wire                                  rsp_pkt_rcvd;

    reg                                   pkt_discarded;  // 当前包是否丢弃：1-丢弃，0-接收

    reg                                   pkt_type;  // 包类型：0-请求包，1-响应包
    reg  [                          31:0] pkt_id_1;
    reg  [                          12:0] pkt_len;  // 包总长度，不包括FCS
    wire                                  rnr_window_active;

    reg  [      C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_offset;
    reg  [      C_RSP_BUF_ADDR_WIDTH-1:0] rsp_pkt_buf_offset;
    wire [      C_RSP_BUF_ADDR_WIDTH-1:0] pkt_buf_offset;  // 缓冲区偏移，若是数据包突发传输3次，则偏移为3，一直累加；表示下一个数据包存储的起始地址

    wire [PKT_DESC_FIFO_1_DATA_WIDTH-1:0] pkt_desc_fifo_1_din_w;
    reg  [PKT_DESC_FIFO_1_DATA_WIDTH-1:0] pkt_desc_fifo_1_din;
    reg  [PKT_DESC_FIFO_1_DATA_WIDTH-1:0] pkt_desc_fifo_1_din_r;
    wire [PKT_DESC_FIFO_1_DATA_WIDTH-1:0] pkt_desc_fifo_1_dout;
    reg                                   wr_pkt_desc_fifo_1;
    reg                                   wr_pkt_desc_fifo_1_r;
    wire                                  rd_pkt_desc_fifo_1;
    wire                                  pkt_desc_fifo_1_empty;
    wire                                  pkt_desc_fifo_1_full;
    wire                                  pkt_desc_fifo_1_almost_full;

    reg  [                          17:0] ipv4_hdr_chksum_i1;
    reg  [                          17:0] ipv4_hdr_chksum_i2;
    reg  [                          17:0] ipv4_hdr_chksum_i3;

    reg  [                          19:0] ipv4_hdr_chksum_ii1;
    wire [                          16:0] ipv4_hdr_chksum_ii2;
    wire                                  ipv4_hdr_chksum_err;  // 校验和检查标志：错误-1，正确-0

    wire                                  ipv4_hdr_chk_en;
    reg  [                           1:0] ipv4_hdr_chk_st;

    reg  [                          31:0] pkt_id_2_reg                                                                                                                                        [ 1:0];
    wire [                          31:0] pkt_id_2;

    wire [PKT_DESC_FIFO_2_DATA_WIDTH-1:0] pkt_desc_fifo_2_din;
    wire [PKT_DESC_FIFO_2_DATA_WIDTH-1:0] pkt_desc_fifo_2_dout;
    wire                                  wr_pkt_desc_fifo_2;
    wire                                  rd_pkt_desc_fifo_2;

    reg  [                         511:0] crc_data_in;

    wire                                  tvalid_crc;
    wire                                  tfirst_crc;
    wire                                  tlast_crc;
    reg                                   tvalid_crc_d1;
    reg                                   tvalid_crc_d2;
    reg                                   tfirst_crc_d1;
    reg                                   tfirst_crc_d2;
    reg                                   tlast_crc_d1;
    reg                                   tlast_crc_d2;
    reg  [                          31:0] pkt_id_crc_d1;
    reg  [                          31:0] pkt_id_crc_d2;

    reg  [                           5:0] lshift_bytes;
    reg  [                         511:0] crc_data_in_la;
    reg  [                           5:0] invld_crc_bytes;
    reg  [                           2:0] crc_corr_sel;
    reg  [                           5:0] invld_crc_bytes_d1;
    reg  [                           5:0] invld_crc_bytes_d2;
    wire [                          31:0] dword_crc32                                                                                                                                         [0:15];
    reg  [                          31:0] crc_corr;
    reg  [                          31:0] line_crc32;
    reg  [                          31:0] nxt_line_crc32;
    wire [                          31:0] crc_data_out;

    wire                                  pkt_crc_err;
    wire                                  crc_calc_done;

    reg  [PKT_DESC_FIFO_3_DATA_WIDTH-1:0] pkt_desc_fifo_3_din;
    wire [PKT_DESC_FIFO_3_DATA_WIDTH-1:0] pkt_desc_fifo_3_dout;
    reg                                   wr_pkt_desc_fifo_3;
    wire                                  rd_pkt_desc_fifo_3;

    wire [                          31:0] pkt_id_crc;

    wire [  PKT_DESC_FIFO_DATA_WIDTH-1:0] pkt_desc_fifo_din;
    reg  [  PKT_DESC_FIFO_DATA_WIDTH-1:0] pkt_desc_fifo_din_r;
    reg  [  PKT_DESC_FIFO_DATA_WIDTH-1:0] pkt_desc_fifo_din_p;

    wire                                  wr_req_pkt_desc_fifo;
    reg                                   wr_req_pkt_desc_fifo_r;
    reg                                   wr_req_pkt_desc_fifo_p;
    wire                                  req_pkt_desc_fifo_full;

    wire                                  wr_rsp_pkt_desc_fifo;
    reg                                   wr_rsp_pkt_desc_fifo_r;
    reg                                   wr_rsp_pkt_desc_fifo_p;
    wire                                  rsp_pkt_desc_fifo_full;

    reg  [                           1:0] req_hdr_buf_fsm_cs;
    reg  [                           1:0] req_hdr_buf_fsm_ns;
    reg  [                           1:0] req_hdr_buf_addra_incr;

    reg  [                           1:0] rsp_hdr_buf_fsm_cs;
    reg  [                           1:0] rsp_hdr_buf_fsm_ns;
    reg  [                           1:0] rsp_hdr_buf_addra_incr;

    // ============================= 第一拍检测 =============================
    always @(posedge clk)
        if (reset) s_axis_tidle <= 1'b1;
        else if (s_axis_tvalid) s_axis_tidle <= s_axis_tlast;  // 有效tlast，如果 s_axis_tlast 为高，因此下一个周期 s_axis_tidle 变为 1'b1
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
    // QP目标地址提取
    assign ipv4_bth_qp_dest_addr = {s_axis_tdata[47*8+:8], s_axis_tdata[48*8+:8], s_axis_tdata[49*8+:8]} >= 24'd1 ? {s_axis_tdata[47*8+:8], s_axis_tdata[48*8+:8], s_axis_tdata[49*8+:8]} - 24'd1 : 24'd0;
    assign ipv6_bth_qp_dest_addr = {s_axis_tdata[3*8+:8], s_axis_tdata[4*8+:8], s_axis_tdata[5*8+:8]} >= 24'd1 ? {s_axis_tdata[3*8+:8], s_axis_tdata[4*8+:8], s_axis_tdata[5*8+:8]} - 24'd1 : 24'd0;
    assign bth_qp_dest_addr      = (eth_type == ETH_TYPE_IPV4) ? ipv4_bth_qp_dest_addr : ipv6_bth_qp_dest_addr;
    // 包序列号(PSN)提取
    assign ipv4_bth_psn          = {s_axis_tdata[51*8+:8], s_axis_tdata[52*8+:8], s_axis_tdata[53*8+:8]};
    assign ipv6_bth_psn          = {s_axis_tdata[7*8+:8], s_axis_tdata[8*8+:8], s_axis_tdata[9*8+:8]};
    assign bth_psn               = (eth_type == ETH_TYPE_IPV4) ? ipv4_bth_psn : ipv6_bth_psn;

    // ============================= 包类型判断 =============================
    // 响应包检测？ (操作码范围01101-10001) ACKNOWLEDGE也是响应包
    assign rsp_pkt_rcvd          = (!bypass_en && (bth_opcode >= 8'b000_01101) && (bth_opcode <= 8'b000_10001)) ? 1'b1 : 1'b0;
    // 只检测ACKNOWLEDGE操作码，不检测读响应
    // assign rsp_pkt_rcvd          = (!bypass_en && (bth_opcode == 8'b000_10001)) ? 1'b1 : 1'b0;
    // 包类型设置
    always @(posedge clk)
        if (s_axis_tfirst) begin
            if (rsp_pkt_rcvd) pkt_type <= PKT_TYPE_RSP;  // 响应包
            else pkt_type <= PKT_TYPE_REQ;  // 请求包
        end

    // ============================= 包ID和长度计算 =============================
    // 包ID生成(递增计数器)
    always @(posedge clk)
        if (reset) pkt_id_1 <= 32'hFFFF_FFFF;
        else if (s_axis_tfirst) pkt_id_1 <= pkt_id_1 + 'd1;  // 第一个包ID为0

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
            if (!xrnic_en) begin  // 未使能IP
                pkt_discarded <= 1'b1;
            end else if (pkt_desc_fifo_1_almost_full || pkt_desc_fifo_1_full) begin  // FIFO1满
                pkt_discarded      <= 1'b1;
                o_inc_drop_pkt_cnt <= o_inc_drop_pkt_cnt + 'd1;
            end else if (rsp_pkt_rcvd && (rsp_pkt_buf_almost_full | rsp_pkt_desc_fifo_full)) begin  // 响应包缓冲区满
                pkt_discarded      <= 1'b1;
                o_inc_drop_pkt_cnt <= o_inc_drop_pkt_cnt + 'd1;
            end else if (!rsp_pkt_rcvd && (req_pkt_buf_almost_full | req_pkt_desc_fifo_full)) begin  // 请求包缓冲区满
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
    // 丢包标志
    assign pkt_discarded_to_req_hv = xrnic_en & tfirst & pkt_discarded;

    // 总包计数
    always @(posedge clk)
        if (reset) o_inc_all_pkt_cnt <= 32'd0;
        else if (xrnic_en && tlast) o_inc_all_pkt_cnt <= o_inc_all_pkt_cnt + 'd1;

    // ============================= 包缓冲区地址偏移管理 =============================
    // 包缓冲区地址递增
    always @(posedge clk)
        if (reset) begin
            req_pkt_buf_offset <= {(C_REQ_BUF_ADDR_WIDTH) {1'b0}};
            rsp_pkt_buf_offset <= {(C_RSP_BUF_ADDR_WIDTH) {1'b0}};
        end else if (!pkt_discarded && tlast) begin
            if (pkt_type == PKT_TYPE_REQ) req_pkt_buf_offset <= req_pkt_buf_addra + 'd1;
            else rsp_pkt_buf_offset <= rsp_pkt_buf_addra + 'd1;
        end

    // 缓冲区偏移，若是数据包突发传输3次，则偏移为3，一直累加
    assign pkt_buf_offset = (pkt_type == PKT_TYPE_RSP) ? rsp_pkt_buf_offset : {{(C_RSP_BUF_ADDR_WIDTH - C_REQ_BUF_ADDR_WIDTH) {1'b0}}, req_pkt_buf_offset};
    // ============================ FIFO =============================
    // FIFO 1: 基本包信息
    generate
        if (C_SIM_DEBUG) begin  // C_SIM_DEBUG = 1
            always @(posedge clk) begin
                if (reset) begin
                    pkt_desc_fifo_1_din <= 0;
                end else begin
                    pkt_desc_fifo_1_din <= {
                        pkt_id_1,  // 32-包ID
                        1'b0,  // 1 (激活 rnr 窗口的占位符)-RNR窗口占位符
                        pkt_type,  // 1-包类型
                        pkt_buf_offset,  // C_RSP_BUF_ADDR_WIDTH-10-缓冲区偏移
                        pkt_len,  // 13-包长度
                        tuser[0]  // 1-包错误标志
                    };
                end
            end
        end else begin  // current: C_SIM_DEBUG = 0
            always @(posedge clk) begin
                if (reset) begin
                    pkt_desc_fifo_1_din <= 0;
                end else begin
                    pkt_desc_fifo_1_din <= {
                        1'b0,  // 1 (激活 rnr 窗口的占位符)
                        pkt_type,  // 1
                        pkt_buf_offset,  // C_RSP_BUF_ADDR_WIDTH-10
                        pkt_len,  // 13
                        tuser[0]  // 1
                    };
                end
            end
        end
    endgenerate

    always @(posedge clk) begin
        if (reset) wr_pkt_desc_fifo_1 <= 0;
        else wr_pkt_desc_fifo_1 <= ~pkt_discarded & tlast;
    end
    //assign wr_pkt_desc_fifo_1 = ~pkt_discarded & tlast;

    assign rnr_window_active = rnr_window_flag_reg[bth_qp_dest_addr_r];  // for (i = 0; i < C_NUM_QP; i = i + 1)  if (pkt_discarded_r) rnr_window_flag_reg[i] <= 1'b1;
    always @(posedge clk) begin
        pkt_desc_fifo_1_din_r <= pkt_desc_fifo_1_din;
        wr_pkt_desc_fifo_1_r  <= wr_pkt_desc_fifo_1;
        bth_qp_dest_addr_r    <= bth_qp_dest_addr;
    end


    generate
        if (C_SIM_DEBUG) begin  // C_SIM_DEBUG = 1
            assign pkt_desc_fifo_1_din_w = {
                pkt_desc_fifo_1_din_r[(C_RSP_BUF_ADDR_WIDTH+16)+:32],  // pkt_id
                rnr_window_active,  //rnr_window_active
                pkt_desc_fifo_1_din_r[15+C_RSP_BUF_ADDR_WIDTH-1:0]  // pkt_type, pkt buffer offset, pkt_len, tuser
            };
        end else begin  // current: C_SIM_DEBUG = 0
            assign pkt_desc_fifo_1_din_w = {
                rnr_window_active,  // 激活 rnr 窗口
                pkt_desc_fifo_1_din_r[15+C_RSP_BUF_ADDR_WIDTH-1:0]  // pkt_type, pkt buffer offset, pkt_len, tuser
            };
        end
    endgenerate

    //=====================================
    // pkt_desc_fifo_1 (FWFT), Default depth = 8
    //=====================================
    // FIFO 1：存储该包基本信息(rnr_window_active, pkt_type, pkt buffer offset, pkt_len, tuser)
    // C_SIM_DEBUG = 0: 
    // bit[25]      rnr_window_active
    // bit[24]      pkt_type
    // bit[23:14]   pkt buffer offset
    // bit[13:1]    pkt_len
    // bit[0]       tuser
    reg_fifo_sync #(
        .C_READ_MODE(1'b1),
        .C_FIFO_DEPTH(PKT_DESC_FIFO_1_DEPTH),
        .C_FIFO_WRCNTR_WIDTH(PKT_DESC_FIFO_1_WRCNTR_WIDTH),
        .C_FIFO_DATA_WIDTH(PKT_DESC_FIFO_1_DATA_WIDTH)
    ) pkt_desc_fifo_1 (
        .clk        (clk),
        .reset      (reset),
        .wr_en      (wr_pkt_desc_fifo_1_r),         // 这时基本信息才正确完整
        .din        (pkt_desc_fifo_1_din_w),        // 这时基本信息才正确完整
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

    always @(posedge clk) begin
        pkt_id_2_reg[1] <= pkt_id_2_reg[0];
        pkt_id_2_reg[0] <= pkt_id_1;
    end

    assign pkt_id_2 = pkt_id_2_reg[1];

    // FIFO 2: IPv4头校验和结果
    generate
        if (C_SIM_DEBUG) assign pkt_desc_fifo_2_din = {pkt_id_2, ipv4_hdr_chksum_err};
        else assign pkt_desc_fifo_2_din = ipv4_hdr_chksum_err;
    endgenerate

    assign wr_pkt_desc_fifo_2 = ipv4_hdr_chk_st[1];

    //=====================================
    // pkt_desc_fifo_2 (FWFT), DEPTH = 8
    //===================================== 
    // FIFO 2：存储该包IPv4头校验和检查标志--错误-1，正确-0
    reg_fifo_sync #(
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
    icrc_rocev2_512b_rx u_icrc_rocev2_512b_rx (
        .clk            (clk),
        .reset          (reset),
        .i_pkt_discarded(pkt_discarded),
        .i_pkt_id       (pkt_id_1),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tkeep   (s_axis_tkeep),
        .s_axis_tlast   (s_axis_tlast),
        .o_crc_calc_done(crc_calc_done),
        .o_pkt_id       (pkt_id_crc),
        .o_crc_out      (),
        .o_crc_err      (pkt_crc_err)
    );

    always @(posedge clk)
        if (reset) wr_pkt_desc_fifo_3 <= 1'b0;
        else wr_pkt_desc_fifo_3 <= crc_calc_done;

    // FIFO 3: ICRC校验结果
    generate
        if (C_SIM_DEBUG) always @(posedge clk) pkt_desc_fifo_3_din <= {pkt_id_crc, pkt_crc_err};
        else always @(posedge clk) pkt_desc_fifo_3_din <= pkt_crc_err;
    endgenerate

    //=====================================
    // pkt_desc_fifo_3 (FWFT), DEPTH = 8
    //=====================================  
    // FIFO 3：存储该包ICRC校验标志--错误-1，正确-0
    reg_fifo_sync #(
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
    // C_SIM_DEBUG = 0: 
    // bit[26]      rnr_window_active
    // bit[25:16]   pkt buffer offset
    // bit[15:3]    pkt_len
    // bit[2]       FCS error -- 0-no error, 1-error
    // bit[1]       IPv4 Header checksum error
    // bit[0]       ICRC error
    generate
        if (C_SIM_DEBUG)  // C_SIM_DEBUG = 1
            assign pkt_desc_fifo_din = {
                pkt_desc_fifo_1_dout[(C_RSP_BUF_ADDR_WIDTH+16)+:32],  // pkt_id
                pkt_desc_fifo_1_dout[C_RSP_BUF_ADDR_WIDTH+15],  // rnr_window_active
                pkt_desc_fifo_1_dout[14+:C_RSP_BUF_ADDR_WIDTH],  // pkt buffer offset
                pkt_desc_fifo_1_dout[13:1],  // pkt_len
                ~pkt_desc_fifo_1_dout[0],  // FCS error
                pkt_desc_fifo_2_dout[0],  // IPv4 Header checksum error
                pkt_desc_fifo_3_dout[0]  // ICRC error
            };
        else  // current: C_SIM_DEBUG = 0
            assign pkt_desc_fifo_din = {
                pkt_desc_fifo_1_dout[C_RSP_BUF_ADDR_WIDTH+15],  // rnr_window_active
                pkt_desc_fifo_1_dout[14+:C_RSP_BUF_ADDR_WIDTH],  // pkt buffer offset
                pkt_desc_fifo_1_dout[13:1],  // pkt_len
                ~pkt_desc_fifo_1_dout[0],  // FCS error
                pkt_desc_fifo_2_dout[0],  // IPv4 Header checksum error
                pkt_desc_fifo_3_dout[0]  // ICRC error
            };
    endgenerate

    // ========================== 包分流，将包描述符按不同包类型存储进相应的FIFO中 ==========================
    // 请求包和响应包分别写入不同FIFO
    assign wr_req_pkt_desc_fifo = !pkt_desc_fifo_3_empty && !req_pkt_desc_fifo_full && ((pkt_desc_fifo_1_dout[C_RSP_BUF_ADDR_WIDTH+14] == PKT_TYPE_REQ) ? 1'b1 : 1'b0);

    assign wr_rsp_pkt_desc_fifo = !pkt_desc_fifo_3_empty && !rsp_pkt_desc_fifo_full && ((pkt_desc_fifo_1_dout[C_RSP_BUF_ADDR_WIDTH+14] == PKT_TYPE_RSP) ? 1'b1 : 1'b0);

    assign rd_pkt_desc_fifo_1   = wr_req_pkt_desc_fifo | wr_rsp_pkt_desc_fifo;
    assign rd_pkt_desc_fifo_2   = rd_pkt_desc_fifo_1;
    assign rd_pkt_desc_fifo_3   = rd_pkt_desc_fifo_1;


    generate
        if (PIPELINE_MODE == 0) begin  // current: PIPELINE_MODE = 0

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
    // req_pkt_desc_fifo (STD), DEPTH = 512
    //===================================== 
    // FIFO 4-req_pkt：存储该请求包描述符(rnr_window_active, pkt buffer offset, pkt_len, FCS error, IPv4 Header checksum error, ICRC error)
    xpm_fifo_sync_wrap #(
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
        .dbiterr      ()
    );

    //=====================================
    // rsp_pkt_desc_fifo (STD), DEPTH = 512
    //===================================== 
    // FIFO 4-rsp_pkt：存储该响应包描述符(rnr_window_active, pkt buffer offset, pkt_len, FCS error, IPv4 Header checksum error, ICRC error)
    xpm_fifo_sync_wrap #(
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
        .dbiterr      ()
    );

    // ========================== 包 & 包头 数据缓冲区写入控制(这些缓冲区在rx_pkt_handler模块中) ===========================
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
        if (!pkt_discarded && (pkt_type == PKT_TYPE_REQ) && tvalid && !bypass_en) begin
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
                    req_hdr_buf_fsm_ns = 2'bx;
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

    // ======================== 响应包数据缓冲区写入控制 ========================
    assign rsp_pkt_buf_ena  = (!pkt_discarded && (pkt_type == PKT_TYPE_RSP) && tvalid) ? 1'b1 : 1'b0;  // 该包是响应包且未丢弃--1-是，0-否

    always @(posedge clk)
        if (reset) rsp_pkt_buf_addra <= {(C_RSP_BUF_ADDR_WIDTH) {1'b0}};
        else if (rsp_pkt_buf_ena) rsp_pkt_buf_addra <= rsp_pkt_buf_addra + 'd1;

    assign rsp_pkt_buf_dina = tdata;
    assign rsp_pkt_buf_wea  = {64{1'b1}};

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
                    rsp_hdr_buf_fsm_ns = 2'bx;
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

    // =====================================================================
    // Debug Logic -- 调试逻辑
    // =====================================================================
    reg  [14:0] pkt_desc_fifo_1_wr_cnt;
    reg  [14:0] pkt_desc_fifo_1_rd_cnt;
    wire [15:0] pkt_desc_fifo_1_occupied;
    wire [15:0] pkt_desc_fifo_1_unoccupied;

    reg  [14:0] req_pkt_desc_fifo_wr_cnt;
    reg  [14:0] req_pkt_desc_fifo_rd_cnt;
    wire [15:0] req_pkt_desc_fifo_occupied;
    wire [15:0] req_pkt_desc_fifo_unoccupied;

    reg  [14:0] rsp_pkt_desc_fifo_wr_cnt;
    reg  [14:0] rsp_pkt_desc_fifo_rd_cnt;
    wire [15:0] rsp_pkt_desc_fifo_occupied;
    wire [15:0] rsp_pkt_desc_fifo_unoccupied;

    // 丢包粘性标志
    always @(posedge clk)
        if (reset) pkt_dropped_sticky <= 1'b0;
        else if (i_clr_debug_sts) pkt_dropped_sticky <= 1'b0;
        else if (xrnic_en && pkt_discarded && tfirst) pkt_dropped_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) req_pkt_buf_almost_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) req_pkt_buf_almost_full_sticky <= 1'b0;
        else if (req_pkt_buf_almost_full) req_pkt_buf_almost_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) rsp_pkt_buf_almost_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) rsp_pkt_buf_almost_full_sticky <= 1'b0;
        else if (rsp_pkt_buf_almost_full) rsp_pkt_buf_almost_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) pkt_desc_fifo_1_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) pkt_desc_fifo_1_wr_cnt <= 15'd0;
        else if (wr_pkt_desc_fifo_1_r) pkt_desc_fifo_1_wr_cnt <= pkt_desc_fifo_1_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) pkt_desc_fifo_1_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) pkt_desc_fifo_1_rd_cnt <= 15'd0;
        else if (rd_pkt_desc_fifo_1) pkt_desc_fifo_1_rd_cnt <= pkt_desc_fifo_1_rd_cnt + 'd1;

    // FIFO占用率统计
    assign pkt_desc_fifo_1_occupied   = (pkt_desc_fifo_1_rd_cnt > pkt_desc_fifo_1_wr_cnt) ? ({1'b1, pkt_desc_fifo_1_wr_cnt} - pkt_desc_fifo_1_rd_cnt) : (pkt_desc_fifo_1_wr_cnt - pkt_desc_fifo_1_rd_cnt);

    assign pkt_desc_fifo_1_unoccupied = PKT_DESC_FIFO_1_DEPTH - pkt_desc_fifo_1_occupied;

    always @(posedge clk)
        if (reset) pkt_desc_fifo_1_unoccupied_r <= PKT_DESC_FIFO_1_DEPTH;
        else if (i_clr_debug_sts) pkt_desc_fifo_1_unoccupied_r <= PKT_DESC_FIFO_1_DEPTH;
        else if (pkt_desc_fifo_1_unoccupied_r > pkt_desc_fifo_1_unoccupied) pkt_desc_fifo_1_unoccupied_r <= pkt_desc_fifo_1_unoccupied;

    always @(posedge clk)
        if (reset) pkt_desc_fifo_1_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) pkt_desc_fifo_1_full_sticky <= 1'b0;
        else if (pkt_desc_fifo_1_full) pkt_desc_fifo_1_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) req_pkt_desc_fifo_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) req_pkt_desc_fifo_wr_cnt <= 15'd0;
        else if (wr_req_pkt_desc_fifo) req_pkt_desc_fifo_wr_cnt <= req_pkt_desc_fifo_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) req_pkt_desc_fifo_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) req_pkt_desc_fifo_rd_cnt <= 15'd0;
        else if (rd_req_pkt_desc_fifo) req_pkt_desc_fifo_rd_cnt <= req_pkt_desc_fifo_rd_cnt + 'd1;

    assign req_pkt_desc_fifo_occupied   = (req_pkt_desc_fifo_rd_cnt > req_pkt_desc_fifo_wr_cnt) ? ({1'b1, req_pkt_desc_fifo_wr_cnt} - req_pkt_desc_fifo_rd_cnt) : (req_pkt_desc_fifo_wr_cnt - req_pkt_desc_fifo_rd_cnt);

    assign req_pkt_desc_fifo_unoccupied = PKT_DESC_FIFO_DEPTH - req_pkt_desc_fifo_occupied;

    always @(posedge clk)
        if (reset) req_pkt_desc_fifo_unoccupied_r <= PKT_DESC_FIFO_DEPTH;
        else if (i_clr_debug_sts) req_pkt_desc_fifo_unoccupied_r <= PKT_DESC_FIFO_DEPTH;
        else if (req_pkt_desc_fifo_unoccupied_r > req_pkt_desc_fifo_unoccupied) req_pkt_desc_fifo_unoccupied_r <= req_pkt_desc_fifo_unoccupied;

    always @(posedge clk)
        if (reset) req_pkt_desc_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) req_pkt_desc_fifo_full_sticky <= 1'b0;
        else if (req_pkt_desc_fifo_full) req_pkt_desc_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) rsp_pkt_desc_fifo_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) rsp_pkt_desc_fifo_wr_cnt <= 15'd0;
        else if (wr_rsp_pkt_desc_fifo) rsp_pkt_desc_fifo_wr_cnt <= rsp_pkt_desc_fifo_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) rsp_pkt_desc_fifo_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) rsp_pkt_desc_fifo_rd_cnt <= 15'd0;
        else if (rd_rsp_pkt_desc_fifo) rsp_pkt_desc_fifo_rd_cnt <= rsp_pkt_desc_fifo_rd_cnt + 'd1;

    assign rsp_pkt_desc_fifo_occupied   = (rsp_pkt_desc_fifo_rd_cnt > rsp_pkt_desc_fifo_wr_cnt) ? ({1'b1, rsp_pkt_desc_fifo_wr_cnt} - rsp_pkt_desc_fifo_rd_cnt) : (rsp_pkt_desc_fifo_wr_cnt - rsp_pkt_desc_fifo_rd_cnt);

    assign rsp_pkt_desc_fifo_unoccupied = PKT_DESC_FIFO_DEPTH - rsp_pkt_desc_fifo_occupied;

    always @(posedge clk)
        if (reset) rsp_pkt_desc_fifo_unoccupied_r <= PKT_DESC_FIFO_DEPTH;
        else if (i_clr_debug_sts) rsp_pkt_desc_fifo_unoccupied_r <= PKT_DESC_FIFO_DEPTH;
        else if (rsp_pkt_desc_fifo_unoccupied_r > rsp_pkt_desc_fifo_unoccupied) rsp_pkt_desc_fifo_unoccupied_r <= rsp_pkt_desc_fifo_unoccupied;

    always @(posedge clk)
        if (reset) rsp_pkt_desc_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) rsp_pkt_desc_fifo_full_sticky <= 1'b0;
        else if (rsp_pkt_desc_fifo_full) rsp_pkt_desc_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) pkt_fcs_err_sticky <= 1'b0;
        else if (i_clr_debug_sts) pkt_fcs_err_sticky <= 1'b0;
        else if (rd_pkt_desc_fifo_1 && !pkt_desc_fifo_1_dout[0]) pkt_fcs_err_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) pkt_crc_err_sticky <= 1'b0;
        else if (i_clr_debug_sts) pkt_crc_err_sticky <= 1'b0;
        else if (rd_pkt_desc_fifo_3 && pkt_desc_fifo_3_dout[0]) pkt_crc_err_sticky <= 1'b1;

endmodule
