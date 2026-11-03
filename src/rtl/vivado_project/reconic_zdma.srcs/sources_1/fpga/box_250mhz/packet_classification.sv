//==============================================================================
// Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
// SPDX-License-Identifier: MIT
//
//==============================================================================
//  packet_classification
//    - filter packets and generate metadata(过滤数据包并生成元数据)
//    - meta data(元数据): 263-bit
//        index       : 32-bit, index of an incoming packet, used for debug purpose(接收到的数据包的索引，用于调试)
//        ip_src      : 32-bit, IP source address(IP 源地址)
//        ip_dst      : 32-bit, IP destination address(IP 目标地址)
//        udp_sport   : 16-bit, UDP source port(UDP 源端口)
//        udp_dport   : 16-bit, UDP destination port(UDP 目的地端口)
//        opcode      : 5-bit, opcode(操作码)
//        pktlen      : 16-bit, packet length(数据包长度)
//        dma_length  : 32-bit, length in bytes of the DMA operation(以字节为单位的 DMA 操作长度)
//        r_key       : 32-bit, remote key(rkey)
//        se          : 1-bit, solicited event()
//        psn         : 24-bit, packet sequence number(数据包序列号)
//        msn         : 24-bit, message sequence number(消息序列号)
//        is_rdma     : 1-bit, indicates that this packet is a rdma packet(表示该数据包是一个 rdma 数据包)
//==============================================================================
`timescale 1ns / 1ps

module packet_classification #(
    parameter METADATA_WIDTH  = 263,
    parameter AXIS_DATA_WIDTH = 512,
    parameter AXIS_KEEP_WIDTH = 64,
    parameter AXIS_USER_WIDTH = 16
) (
    // Metadata
    input  [METADATA_WIDTH-1:0] user_metadata_in,
    input                       user_metadata_in_valid,
    output [METADATA_WIDTH-1:0] user_metadata_out,
    output                      user_metadata_out_valid,

    // Packet input in axi-streaming format
    // axi-streaming格式数据包输入
    input                        s_axis_tvalid,
    input  [AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input  [AXIS_KEEP_WIDTH-1:0] s_axis_tkeep,
    input                        s_axis_tlast,
    output                       s_axis_tready,

    // RoCEv2 packet output in axi-streaming format
    // axi-streaming格式的RoCEv2数据包输出
    output                       m_axis_roce_tvalid,
    output [AXIS_DATA_WIDTH-1:0] m_axis_roce_tdata,
    output [AXIS_KEEP_WIDTH-1:0] m_axis_roce_tkeep,
    output [AXIS_USER_WIDTH-1:0] m_axis_roce_tuser_size,
    output                       m_axis_roce_tlast,
    input                        m_axis_roce_tready,

    // non-RoCEv2 packet output in axi-streaming format
    // axi-streaming格式的非RoCEv2数据包输出
    output                       m_axis_non_roce_tvalid,
    output [AXIS_DATA_WIDTH-1:0] m_axis_non_roce_tdata,
    output [AXIS_KEEP_WIDTH-1:0] m_axis_non_roce_tkeep,
    output [AXIS_USER_WIDTH-1:0] m_axis_non_roce_tuser_size,
    output                       m_axis_non_roce_tlast,
    input                        m_axis_non_roce_tready,

    // Table configuration using AXI-lite interface(不使用)
    input         s_axil_ctrl_awvalid,
    input  [12:0] s_axil_ctrl_awaddr,
    output        s_axil_ctrl_awready,
    input         s_axil_ctrl_wvalid,
    input  [31:0] s_axil_ctrl_wdata,
    output        s_axil_ctrl_wready,
    output        s_axil_ctrl_bvalid,
    output [ 1:0] s_axil_ctrl_bresp,
    input         s_axil_ctrl_bready,
    input         s_axil_ctrl_arvalid,
    input  [12:0] s_axil_ctrl_araddr,
    output        s_axil_ctrl_arready,
    output        s_axil_ctrl_rvalid,
    output [31:0] s_axil_ctrl_rdata,
    output [ 1:0] s_axil_ctrl_rresp,
    input         s_axil_ctrl_rready,

    output [1:0] pkt_filter_err,

    input axis_aclk,
    input axis_rstn,
    input axil_aclk,
    input axil_rstn
);

    // Output data of the parser
    logic [ METADATA_WIDTH-1:0] parser_out_metadata;
    logic                       parser_out_metadata_valid;

    logic                       parser_out_tvalid;
    logic [AXIS_DATA_WIDTH-1:0] parser_out_tdata;
    logic [AXIS_KEEP_WIDTH-1:0] parser_out_tkeep;
    logic [AXIS_USER_WIDTH-1:0] parser_out_tuser_size;
    logic                       parser_out_tlast;
    logic                       parser_out_tready;

    // Hold parser packet size for the whole packet so downstream logic sees a
    // stable *_tuser_size on every beat.
    logic [AXIS_USER_WIDTH-1:0] parser_pkt_size_reg;

    // Packet Classification module
    packet_parser packet_parser_inst (
        .s_axis_aclk   (axis_aclk),
        .s_axis_aresetn(axis_rstn),
        .s_axil_aclk   (axil_aclk),
        .s_axil_aresetn(axil_rstn),

        // Metadata输入
        .user_metadata_in       (user_metadata_in),          // 完整元数据全0传入(除了pc_in_pktlen)
        .user_metadata_in_valid (user_metadata_in_valid),    // 第一次突发传输时才为1
        // 输出 --> packet_filter模块
        .user_metadata_out      (parser_out_metadata),
        .user_metadata_out_valid(parser_out_metadata_valid),

        // Slave AXI-lite interface(不使用)
        .s_axil_awaddr (s_axil_ctrl_awaddr),
        .s_axil_awvalid(s_axil_ctrl_awvalid),
        .s_axil_awready(s_axil_ctrl_awready),
        .s_axil_wdata  (s_axil_ctrl_wdata),
        .s_axil_wvalid (s_axil_ctrl_wvalid),
        .s_axil_wready (s_axil_ctrl_wready),
        .s_axil_bresp  (s_axil_ctrl_bresp),
        .s_axil_bvalid (s_axil_ctrl_bvalid),
        .s_axil_bready (s_axil_ctrl_bready),
        .s_axil_araddr (s_axil_ctrl_araddr),
        .s_axil_arvalid(s_axil_ctrl_arvalid),
        .s_axil_arready(s_axil_ctrl_arready),
        .s_axil_rdata  (s_axil_ctrl_rdata),
        .s_axil_rvalid (s_axil_ctrl_rvalid),
        .s_axil_rready (s_axil_ctrl_rready),
        .s_axil_rresp  (s_axil_ctrl_rresp),

        // AXI Master port
        .m_axis_tdata (parser_out_tdata),
        .m_axis_tkeep (parser_out_tkeep),
        .m_axis_tvalid(parser_out_tvalid),
        .m_axis_tready(parser_out_tready),
        .m_axis_tlast (parser_out_tlast),

        // AXI Slave port
        .s_axis_tdata (s_axis_tdata),
        .s_axis_tkeep (s_axis_tkeep),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tlast (s_axis_tlast),
        .s_axis_tready(s_axis_tready)
    );

    always_ff @(posedge axis_aclk) begin
        if (~axis_rstn) begin
            parser_pkt_size_reg <= {AXIS_USER_WIDTH{1'b0}};
        end else if (parser_out_tvalid && parser_out_tready) begin
            if (parser_out_metadata_valid) begin
                parser_pkt_size_reg <= parser_out_metadata[129:114];
            end
            if (parser_out_tlast) begin
                parser_pkt_size_reg <= {AXIS_USER_WIDTH{1'b0}};
            end
        end
    end

    assign parser_out_tuser_size = parser_out_tvalid ?
                                   (parser_out_metadata_valid ? parser_out_metadata[129:114] : parser_pkt_size_reg) :
                                   {AXIS_USER_WIDTH{1'b0}};

    packet_filter #(
        .METADATA_WIDTH (METADATA_WIDTH),
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_KEEP_WIDTH(AXIS_KEEP_WIDTH)
    ) packet_filter_inst (
        .metadata_in      (parser_out_metadata),
        .metadata_in_valid(parser_out_metadata_valid),

        // Packet input in axi-streaming format
        .s_axis_tvalid    (parser_out_tvalid),
        .s_axis_tdata     (parser_out_tdata),
        .s_axis_tkeep     (parser_out_tkeep),
        .s_axis_tuser_size(parser_out_tuser_size),
        .s_axis_tlast     (parser_out_tlast),
        .s_axis_tready    (parser_out_tready),

        // RoCEv2 packet output in axi-streaming format
        .m_axis_roce_tvalid    (m_axis_roce_tvalid),
        .m_axis_roce_tdata     (m_axis_roce_tdata),
        .m_axis_roce_tkeep     (m_axis_roce_tkeep),
        .m_axis_roce_tuser_size(m_axis_roce_tuser_size),
        .m_axis_roce_tlast     (m_axis_roce_tlast),
        .m_axis_roce_tready    (m_axis_roce_tready),

        // non-RoCEv2 packet output in axi-streaming format
        .m_axis_non_roce_tvalid    (m_axis_non_roce_tvalid),
        .m_axis_non_roce_tdata     (m_axis_non_roce_tdata),
        .m_axis_non_roce_tkeep     (m_axis_non_roce_tkeep),
        .m_axis_non_roce_tuser_size(m_axis_non_roce_tuser_size),
        .m_axis_non_roce_tlast     (m_axis_non_roce_tlast),
        .m_axis_non_roce_tready    (m_axis_non_roce_tready),

        .metadata_out      (user_metadata_out),
        .metadata_out_valid(user_metadata_out_valid),

        .pkt_filter_err(pkt_filter_err),

        .axis_aclk(axis_aclk),
        .axis_rstn(axis_rstn)
    );

endmodule : packet_classification
