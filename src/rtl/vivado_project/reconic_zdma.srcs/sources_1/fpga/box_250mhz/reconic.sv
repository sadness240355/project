//==============================================================================
// Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
// SPDX-License-Identifier: MIT
//
//==============================================================================
`timescale 1ns / 1ps

module reconic #(
    parameter AXIL_REG_ADDR_WIDTH  = 12,
    parameter AXIL_CTRL_ADDR_WIDTH = 13,
    parameter AXIL_DATA_WIDTH      = 32,
    parameter AXIS_DATA_WIDTH      = 512,
    parameter AXIS_KEEP_WIDTH      = 64,
    parameter AXIS_USER_WIDTH      = 16
) (
    // Table control interface（不使用）
    input         s_axil_ctrl_awvalid,
    input  [31:0] s_axil_ctrl_awaddr,
    output        s_axil_ctrl_awready,
    input         s_axil_ctrl_wvalid,
    input  [31:0] s_axil_ctrl_wdata,
    output        s_axil_ctrl_wready,
    output        s_axil_ctrl_bvalid,
    output [ 1:0] s_axil_ctrl_bresp,
    input         s_axil_ctrl_bready,
    input         s_axil_ctrl_arvalid,
    input  [31:0] s_axil_ctrl_araddr,
    output        s_axil_ctrl_arready,
    output        s_axil_ctrl_rvalid,
    output [31:0] s_axil_ctrl_rdata,
    output [ 1:0] s_axil_ctrl_rresp,
    input         s_axil_ctrl_rready,

    // Register control interface
    input         s_axil_reg_awvalid,
    input  [31:0] s_axil_reg_awaddr,
    output        s_axil_reg_awready,
    input         s_axil_reg_wvalid,
    input  [31:0] s_axil_reg_wdata,
    output        s_axil_reg_wready,
    output        s_axil_reg_bvalid,
    output [ 1:0] s_axil_reg_bresp,
    input         s_axil_reg_bready,
    input         s_axil_reg_arvalid,
    input  [31:0] s_axil_reg_araddr,
    output        s_axil_reg_arready,
    output        s_axil_reg_rvalid,
    output [31:0] s_axil_reg_rdata,
    output [ 1:0] s_axil_reg_rresp,
    input         s_axil_reg_rready,


    // Receive packets from CMAC RX path
    // 从 CMAC RX 路径接收数据包
    input          s_axis_cmac_rx_tvalid,
    input  [511:0] s_axis_cmac_rx_tdata,
    input  [ 63:0] s_axis_cmac_rx_tkeep,
    input          s_axis_cmac_rx_tlast,
    input  [ 15:0] s_axis_cmac_rx_tuser_size,  // 突发传输的包大小（字节数）
`ifdef DEBUG
    input  [ 31:0] s_axis_cmac_rx_tuser_idx,
`endif
    output         s_axis_cmac_rx_tready,

    // Send roce packets to rdma rx path
    output         m_axis_user2rdma_roce_from_cmac_rx_tvalid,
    output [511:0] m_axis_user2rdma_roce_from_cmac_rx_tdata,
    output [ 63:0] m_axis_user2rdma_roce_from_cmac_rx_tkeep,
    output         m_axis_user2rdma_roce_from_cmac_rx_tlast,
    input          m_axis_user2rdma_roce_from_cmac_rx_tready,

    // Send non-roce packets to QDMA rx path
    output         m_axis_qdma_c2h_tvalid,
    output [511:0] m_axis_qdma_c2h_tdata,
    output [ 63:0] m_axis_qdma_c2h_tkeep,
    output         m_axis_qdma_c2h_tlast,
    output [ 15:0] m_axis_qdma_c2h_tuser_size,
    input          m_axis_qdma_c2h_tready,

    // Get non-roce packets from QDMA tx path
    input          s_axis_qdma_h2c_tvalid,
    input  [511:0] s_axis_qdma_h2c_tdata,
    input  [ 63:0] s_axis_qdma_h2c_tkeep,
    input          s_axis_qdma_h2c_tlast,
    input  [ 15:0] s_axis_qdma_h2c_tuser_size,
    output         s_axis_qdma_h2c_tready,

    // Send non-roce packets to rdma tx path
    output         m_axis_user2rdma_from_qdma_tx_tvalid,
    output [511:0] m_axis_user2rdma_from_qdma_tx_tdata,
    output [ 63:0] m_axis_user2rdma_from_qdma_tx_tkeep,
    output         m_axis_user2rdma_from_qdma_tx_tlast,
    input          m_axis_user2rdma_from_qdma_tx_tready,

    // Get roce packets from rdma tx path
    input          s_axis_rdma2user_to_cmac_tx_tvalid,
    input  [511:0] s_axis_rdma2user_to_cmac_tx_tdata,
    input  [ 63:0] s_axis_rdma2user_to_cmac_tx_tkeep,
    input          s_axis_rdma2user_to_cmac_tx_tlast,
    output         s_axis_rdma2user_to_cmac_tx_tready,

    // Send packets to CMAC tx path
    output         m_axis_cmac_tx_tvalid,
    output [511:0] m_axis_cmac_tx_tdata,
    output [ 63:0] m_axis_cmac_tx_tkeep,
    output         m_axis_cmac_tx_tlast,
    output [ 15:0] m_axis_cmac_tx_tuser_size,
    input          m_axis_cmac_tx_tready,


    input axil_aclk,
    input axil_rstn,
    input axis_aclk,
    input axis_rstn
);

    localparam METADATA_WIDTH = 263;
    localparam FIFO_WRITE_DEPTH = 512;

    /* 
     * Metadata contains debug information and control data
     * Total size   : 263 bits
     * @index       : 32-bit, index of an incoming packet, used for debug purpose
     * @ip_src      : 32-bit, IP source address
     * @ip_dst      : 32-bit, IP destination address
     * @udp_sport   : 16-bit, UDP source port
     * @udp_dport   : 16-bit, UDP destination port
     * @opcode      : 5-bit, opcode
     * @pktlen      : 16-bit, packet length
     * @dma_length  : 32-bit, length in bytes of the DMA operation
     * @r_key       : 32-bit, remote key
     * @se          : 1-bit, solicited event
     * @psn         : 24-bit, packet sequence number
     * @msn         : 24-bit, message sequence number
     * @is_rdma     : 1-bit, indicates that this packet is a rdma packet
     */
    logic [METADATA_WIDTH-1:0] pc_in_metadata;
    logic                      pc_in_metadata_valid;
    logic                      set_pc_in_metadata_valid_low;
    logic [METADATA_WIDTH-1:0] pc_out_metadata;
    logic                      pc_out_metadata_valid;

    // Metadata input
    logic [              31:0] pc_in_index;
    logic [              31:0] pc_in_ip_src;
    logic [              31:0] pc_in_ip_dst;
    logic [              15:0] pc_in_udp_sport;
    logic [              15:0] pc_in_udp_dport;
    logic [               4:0] pc_in_opcode;
    logic [              15:0] pc_in_pktlen;
    logic [              31:0] pc_in_dma_length;
    logic [              31:0] pc_in_r_key;
    logic                      pc_in_se;
    logic [              23:0] pc_in_psn;
    logic [              23:0] pc_in_msn;
    logic                      pc_in_is_rdma;

    // Metadata output
    logic [              31:0] pc_out_index;
    logic [              31:0] pc_out_ip_src;
    logic [              31:0] pc_out_ip_dst;
    logic [              15:0] pc_out_udp_sport;
    logic [              15:0] pc_out_udp_dport;
    logic [               4:0] pc_out_opcode;
    logic [              15:0] pc_out_pktlen;
    logic [              31:0] pc_out_dma_length;
    logic [              31:0] pc_out_r_key;
    logic                      pc_out_se;
    logic [              23:0] pc_out_psn;
    logic [              23:0] pc_out_msn;
    logic                      pc_out_is_rdma;

    logic                      roce_pkt_recved;
    logic                      non_roce_pkt_recved;
    logic                      packet_filter_err;
    logic                      roce_pkt_recved_at_axis_clk;
    logic                      non_roce_pkt_recved_at_axis_clk;
    logic [              31:0] fatal_err;

    // Declaration for buffer_mac_rx_data
    logic                      wr_en;
    logic                      rd_en;
    logic                      buffer_full;
    logic                      buff_empty;
    logic                      buffer_vld;
    logic [             511:0] buffer_out_tdata;
    logic [              63:0] buffer_out_tkeep;
    logic [              15:0] buffer_out_tuser_size;
    logic                      buffer_out_tlast;
`ifdef DEBUG
    logic [31:0] buffer_out_tuser_idx;
`endif

    logic         pc_in_tvalid;
    logic [511:0] pc_in_tdata;
    logic [ 63:0] pc_in_tkeep;
    logic         pc_in_tlast;
    logic         pc_in_tready;

    logic         pc_non_roce_out_tvalid;
    logic [511:0] pc_non_roce_out_tdata;
    logic [ 63:0] pc_non_roce_out_tkeep;
    logic [ 15:0] pc_non_roce_out_tuser_size;
    logic         pc_non_roce_out_tlast;
    logic         pc_non_roce_out_tready;

    logic         pc_roce_out_tvalid;
    logic [511:0] pc_roce_out_tdata;
    logic [ 63:0] pc_roce_out_tkeep;
    logic [ 15:0] pc_roce_out_tuser_size;
    logic         pc_roce_out_tlast;
    logic         pc_roce_out_tready;

    logic [  1:0] pkt_filter_err;

    // rn_reg_control only uses fatal_err[5:0]. When changing fatal_err, 
    // please make sure that you also change fatal_err in rn_reg_control
    // fatal_err[1]: RDMA FIFO in packet_filter is overflowed
    // fatal_err[0]: non-RDMA FIFO in packet_filter is overflowed
    // ****************************************************************
    // rn_reg_control 只使用 fatal_err[5:0]。更改 fatal_err 时,
    // 请确保同时更改 rn_reg_control 中的 fatal_err。
    // fatal_err[1]：packet_filter 中的 RDMA FIFO 已溢出
    // fatal_err[0]：packet_filter 中的非 RDMA FIFO 已溢出
    assign fatal_err = {27'd0, 3'd0, packet_filter_err};

    // Register control
    // 寄存器控制
    rn_reg_control #(
        .AXIL_ADDR_WIDTH(AXIL_REG_ADDR_WIDTH),
        .AXIL_DATA_WIDTH(AXIL_DATA_WIDTH)
    ) rn_reg_control_inst (
        .s_axil_reg_awvalid(s_axil_reg_awvalid),
        .s_axil_reg_awaddr (s_axil_reg_awaddr[AXIL_REG_ADDR_WIDTH-1:0]),
        .s_axil_reg_awready(s_axil_reg_awready),
        .s_axil_reg_wvalid (s_axil_reg_wvalid),
        .s_axil_reg_wdata  (s_axil_reg_wdata),
        .s_axil_reg_wready (s_axil_reg_wready),
        .s_axil_reg_bvalid (s_axil_reg_bvalid),
        .s_axil_reg_bresp  (s_axil_reg_bresp),
        .s_axil_reg_bready (s_axil_reg_bready),
        .s_axil_reg_arvalid(s_axil_reg_arvalid),
        .s_axil_reg_araddr (s_axil_reg_araddr[AXIL_REG_ADDR_WIDTH-1:0]),
        .s_axil_reg_arready(s_axil_reg_arready),
        .s_axil_reg_rvalid (s_axil_reg_rvalid),
        .s_axil_reg_rdata  (s_axil_reg_rdata),
        .s_axil_reg_rresp  (s_axil_reg_rresp),
        .s_axil_reg_rready (s_axil_reg_rready),

        // input
        .roce_pkt_recved    (roce_pkt_recved),
        .non_roce_pkt_recved(non_roce_pkt_recved),
        .fatal_err          (fatal_err),

        .axil_aclk (axil_aclk),
        .axil_arstn(axil_rstn)
    );

    // Buffer data coming from the MAC rx path
    // 缓冲来自 MAC Rx 路径的数据
`ifdef DEBUG
    xpm_fifo_sync #(
        .DOUT_RESET_VALUE ("0"),
        .ECC_MODE         ("no_ecc"),
        .FIFO_MEMORY_TYPE ("auto"),
        .FIFO_READ_LATENCY(1),
        .FIFO_WRITE_DEPTH (FIFO_WRITE_DEPTH),
        .READ_DATA_WIDTH  (AXIS_DATA_WIDTH + AXIS_KEEP_WIDTH + AXIS_USER_WIDTH + 32 + 1),
        .READ_MODE        ("fwft"),
        .WRITE_DATA_WIDTH (AXIS_DATA_WIDTH + AXIS_KEEP_WIDTH + AXIS_USER_WIDTH + 32 + 1)
    ) buffer_mac_rx_data (
        .wr_en     (wr_en),
        .din       ({s_axis_cmac_rx_tdata, s_axis_cmac_rx_tkeep, s_axis_cmac_rx_tuser_size, s_axis_cmac_rx_tuser_idx, s_axis_cmac_rx_tlast}),
        .wr_ack    (),
        .rd_en     (rd_en),
        .data_valid(buffer_vld),
        .dout      ({buffer_out_tdata, buffer_out_tkeep, buffer_out_tuser_size, buffer_out_tuser_idx, buffer_out_tlast}),

        .wr_data_count(),
        .rd_data_count(),

        .empty       (buff_empty),
        .full        (buffer_full),
        .almost_empty(),
        .almost_full (),
        .overflow    (),
        .underflow   (),
        .prog_empty  (),
        .prog_full   (),
        .sleep       (1'b0),

        .sbiterr      (),
        .dbiterr      (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),

        .wr_clk     (axis_aclk),
        .rst        (~axis_rstn),
        .rd_rst_busy(),
        .wr_rst_busy()
    );
`else  // ！DEBUG
    xpm_fifo_sync #(
        .DOUT_RESET_VALUE ("0"),
        .ECC_MODE         ("no_ecc"),
        .FIFO_MEMORY_TYPE ("auto"),
        .FIFO_READ_LATENCY(1),
        .FIFO_WRITE_DEPTH (FIFO_WRITE_DEPTH),
        .READ_DATA_WIDTH  (AXIS_DATA_WIDTH + AXIS_KEEP_WIDTH + AXIS_USER_WIDTH + 1),
        .READ_MODE        ("fwft"),
        .WRITE_DATA_WIDTH (AXIS_DATA_WIDTH + AXIS_KEEP_WIDTH + AXIS_USER_WIDTH + 1)
    ) buffer_mac_rx_data (
        .wr_en     (wr_en),
        .din       ({s_axis_cmac_rx_tdata, s_axis_cmac_rx_tkeep, s_axis_cmac_rx_tuser_size, s_axis_cmac_rx_tlast}),
        .wr_ack    (),
        .rd_en     (rd_en),
        .data_valid(buffer_vld),
        .dout      ({buffer_out_tdata, buffer_out_tkeep, buffer_out_tuser_size, buffer_out_tlast}),

        .wr_data_count(),
        .rd_data_count(),

        .empty       (buff_empty),
        .full        (buffer_full),
        .almost_empty(),
        .almost_full (),
        .overflow    (),
        .underflow   (),
        .prog_empty  (),
        .prog_full   (),
        .sleep       (1'b0),

        .sbiterr      (),
        .dbiterr      (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),

        .wr_clk     (axis_aclk),
        .rst        (~axis_rstn),
        .rd_rst_busy(),
        .wr_rst_busy()
    );
`endif

    assign s_axis_cmac_rx_tready = !buffer_full;  // RX路径的fifo不满，表示还能接收数据
    assign wr_en                 = s_axis_cmac_rx_tready && s_axis_cmac_rx_tvalid;  // 写入使能：若是fifo不满且数据有效，则写入使能为1
    assign rd_en                 = pc_in_tready && !buff_empty;  // 读出使能：若是pc_in_tready为1且fifo不空，则读出使能为1
    assign pc_in_tvalid          = rd_en;  // 当读使能为1，则pc_in_tvalid为1，说明读出数据有效
    assign pc_in_tdata           = rd_en ? buffer_out_tdata : 512'd0;  // 读出数据
    assign pc_in_tkeep           = rd_en ? buffer_out_tkeep : 64'd0;  // 读出keep位
    assign pc_in_tlast           = rd_en ? buffer_out_tlast : 1'b0;  // 读出最后一个数据标志位

    // Metadata input（RX路径）
    // 元数据输入，全部为0
`ifdef DEBUG
    assign pc_in_index = buffer_out_tuser_idx;
`else
    assign pc_in_index = 32'd0;
`endif




    assign pc_in_ip_src     = 32'd0;
    assign pc_in_ip_dst     = 32'd0;
    assign pc_in_udp_sport  = 16'd0;
    assign pc_in_udp_dport  = 16'd0;
    assign pc_in_opcode     = 8'd0;
    assign pc_in_pktlen     = buffer_out_tuser_size;  // RX路径上，一个数据包的总字节数
    assign pc_in_dma_length = 32'd0;
    assign pc_in_r_key      = 6'd0;
    assign pc_in_se         = 2'd0;
    assign pc_in_psn        = 12'd0;
    assign pc_in_msn        = 24'd0;
    assign pc_in_is_rdma    = 1'b0;
    // 元数据pc_in_metadata
    assign pc_in_metadata   = {pc_in_index, pc_in_ip_src, pc_in_ip_dst, pc_in_udp_sport, pc_in_udp_dport, pc_in_opcode, pc_in_pktlen, pc_in_dma_length, pc_in_r_key, pc_in_se, pc_in_psn, pc_in_msn, pc_in_is_rdma};

    // 判断是否为单beat包（<=64字节）
    logic is_single_beat_packet;
    assign is_single_beat_packet = (buffer_out_tuser_size <= 16'd64);

    // 元数据有效逻辑：对于单beat包在tlast时有效，对于多beat包在第一拍时有效
    assign pc_in_metadata_valid  = pc_in_tvalid && pc_in_tready && (is_single_beat_packet ? pc_in_tlast : !set_pc_in_metadata_valid_low);

    // Metadata output, valid signal is pc_out_metadata_valid
    // 元数据输出，有效信号为 pc_out_metadata_valid
    assign pc_out_index          = pc_out_metadata_valid ? pc_out_metadata[262:231] : 0;
    assign pc_out_ip_src         = pc_out_metadata_valid ? pc_out_metadata[230:199] : 0;
    assign pc_out_ip_dst         = pc_out_metadata_valid ? pc_out_metadata[198:167] : 0;
    assign pc_out_udp_sport      = pc_out_metadata_valid ? pc_out_metadata[166:151] : 0;
    assign pc_out_udp_dport      = pc_out_metadata_valid ? pc_out_metadata[150:135] : 0;
    assign pc_out_opcode         = pc_out_metadata_valid ? pc_out_metadata[134:130] : 0;
    assign pc_out_pktlen         = pc_out_metadata_valid ? pc_out_metadata[129:114] : 0;
    assign pc_out_dma_length     = pc_out_metadata_valid ? pc_out_metadata[113:82] : 0;
    assign pc_out_r_key          = pc_out_metadata_valid ? pc_out_metadata[81 : 50] : 0;
    assign pc_out_se             = pc_out_metadata_valid ? pc_out_metadata[49 : 49] : 0;
    assign pc_out_psn            = pc_out_metadata_valid ? pc_out_metadata[48 : 25] : 0;
    assign pc_out_msn            = pc_out_metadata_valid ? pc_out_metadata[24 : 1] : 0;
    assign pc_out_is_rdma        = pc_out_metadata_valid ? pc_out_metadata[0 : 0] : 0;

    // 这一段逻辑用于设置 set_pc_in_metadata_valid_low 信号  
    // 只对于多beat包：只有第一拍时元数据有效，其余拍无效
    always @(posedge axis_aclk) begin
        if (~axis_rstn) begin
            set_pc_in_metadata_valid_low <= 1'b0;  // 复位后为0，第一个包的第一拍元数据有效
        end else begin
            if (pc_in_tvalid && pc_in_tready) begin
                // 只要有数据传输就拉高信号，使得从第二拍开始元数据无效
                // 在最后一拍时重新拉低，为下个包做准备
                if (pc_in_tlast) begin
                    set_pc_in_metadata_valid_low <= 1'b0;  // 最后一拍：为下个包的第一拍准备
                end else begin
                    set_pc_in_metadata_valid_low <= 1'b1;  // 非最后拍：拉高使元数据无效
                end
            end
        end
    end

    // Packet Classification module
    // 数据包分类模块
    packet_classification #(
        .METADATA_WIDTH(METADATA_WIDTH)
    ) pacekt_classification_inst (
        // Metadata
        // 传入packet_parser模块
        .user_metadata_in       (pc_in_metadata),
        .user_metadata_in_valid (pc_in_metadata_valid),
        // packet_filter模块传出，且只有当数据包是 RoCEv2 数据包时，元数据输出才有效
        .user_metadata_out      (pc_out_metadata),
        .user_metadata_out_valid(pc_out_metadata_valid),

        // Packet input in axi-streaming format
        // axi-streaming格式数据包输入
        .s_axis_tdata (pc_in_tdata),
        .s_axis_tkeep (pc_in_tkeep),
        .s_axis_tvalid(pc_in_tvalid),
        .s_axis_tlast (pc_in_tlast),
        .s_axis_tready(pc_in_tready),  // packet_classification模块准备好接收数据

        // RoCEv2 packet output in axi-streaming format
        // 以 axi-streaming 格式输出 RoCEv2 数据包
        .m_axis_roce_tdata     (pc_roce_out_tdata),
        .m_axis_roce_tkeep     (pc_roce_out_tkeep),
        .m_axis_roce_tuser_size(pc_roce_out_tuser_size),
        .m_axis_roce_tvalid    (pc_roce_out_tvalid),
        .m_axis_roce_tready    (pc_roce_out_tready),
        .m_axis_roce_tlast     (pc_roce_out_tlast),

        // non-RoCEv2 packet output in axi-streaming format
        // 以 axi-streaming 格式输出非 RoCEv2 数据包
        .m_axis_non_roce_tdata     (pc_non_roce_out_tdata),
        .m_axis_non_roce_tkeep     (pc_non_roce_out_tkeep),
        .m_axis_non_roce_tuser_size(pc_non_roce_out_tuser_size),
        .m_axis_non_roce_tvalid    (pc_non_roce_out_tvalid),
        .m_axis_non_roce_tready    (pc_non_roce_out_tready),
        .m_axis_non_roce_tlast     (pc_non_roce_out_tlast),

        // Slave AXI-lite interface(不使用)
        .s_axil_ctrl_awaddr (s_axil_ctrl_awaddr[AXIL_CTRL_ADDR_WIDTH-1:0]),
        .s_axil_ctrl_awvalid(s_axil_ctrl_awvalid),
        .s_axil_ctrl_awready(s_axil_ctrl_awready),
        .s_axil_ctrl_wdata  (s_axil_ctrl_wdata),
        .s_axil_ctrl_wvalid (s_axil_ctrl_wvalid),
        .s_axil_ctrl_wready (s_axil_ctrl_wready),
        .s_axil_ctrl_bresp  (s_axil_ctrl_bresp),
        .s_axil_ctrl_bvalid (s_axil_ctrl_bvalid),
        .s_axil_ctrl_bready (s_axil_ctrl_bready),
        .s_axil_ctrl_araddr (s_axil_ctrl_araddr[AXIL_CTRL_ADDR_WIDTH-1:0]),
        .s_axil_ctrl_arvalid(s_axil_ctrl_arvalid),
        .s_axil_ctrl_arready(s_axil_ctrl_arready),
        .s_axil_ctrl_rdata  (s_axil_ctrl_rdata),
        .s_axil_ctrl_rvalid (s_axil_ctrl_rvalid),
        .s_axil_ctrl_rready (s_axil_ctrl_rready),
        .s_axil_ctrl_rresp  (s_axil_ctrl_rresp),

        .pkt_filter_err(pkt_filter_err),  // packet_filter模块传出，用于调试

        .axis_aclk(axis_aclk),
        .axis_rstn(axis_rstn),
        .axil_aclk(axil_aclk),
        .axil_rstn(axil_rstn)
    );


    assign m_axis_user2rdma_roce_from_cmac_rx_tvalid = pc_roce_out_tvalid;
    assign m_axis_user2rdma_roce_from_cmac_rx_tdata  = pc_roce_out_tdata;
    assign m_axis_user2rdma_roce_from_cmac_rx_tkeep  = pc_roce_out_tkeep;
    assign m_axis_user2rdma_roce_from_cmac_rx_tlast  = pc_roce_out_tlast;
    assign pc_roce_out_tready                        = m_axis_user2rdma_roce_from_cmac_rx_tready;

    assign m_axis_qdma_c2h_tvalid                    = pc_non_roce_out_tvalid;
    assign m_axis_qdma_c2h_tdata                     = pc_non_roce_out_tdata;
    assign m_axis_qdma_c2h_tkeep                     = pc_non_roce_out_tkeep;
    assign m_axis_qdma_c2h_tlast                     = pc_non_roce_out_tlast;
    assign m_axis_qdma_c2h_tuser_size                = pc_non_roce_out_tuser_size;
    assign pc_non_roce_out_tready                    = m_axis_qdma_c2h_tready;

    assign m_axis_cmac_tx_tvalid                     = s_axis_rdma2user_to_cmac_tx_tvalid;
    assign m_axis_cmac_tx_tdata                      = s_axis_rdma2user_to_cmac_tx_tdata;
    assign m_axis_cmac_tx_tkeep                      = s_axis_rdma2user_to_cmac_tx_tkeep;
    assign m_axis_cmac_tx_tlast                      = s_axis_rdma2user_to_cmac_tx_tlast;
    assign m_axis_cmac_tx_tuser_size                 = 16'd0;
    assign s_axis_rdma2user_to_cmac_tx_tready        = m_axis_cmac_tx_tready;

    assign m_axis_user2rdma_from_qdma_tx_tvalid      = s_axis_qdma_h2c_tvalid;
    assign m_axis_user2rdma_from_qdma_tx_tdata       = s_axis_qdma_h2c_tdata;
    assign m_axis_user2rdma_from_qdma_tx_tkeep       = s_axis_qdma_h2c_tkeep;
    assign m_axis_user2rdma_from_qdma_tx_tlast       = s_axis_qdma_h2c_tlast;
    assign s_axis_qdma_h2c_tready                    = m_axis_user2rdma_from_qdma_tx_tready;

    // Get number of roce and non-roce packets received
    // 获取收到的 ROCE 和非 ROCE 数据包数量
    assign roce_pkt_recved_at_axis_clk               = pc_out_metadata_valid;
    assign non_roce_pkt_recved_at_axis_clk           = pc_non_roce_out_tready && pc_non_roce_out_tvalid && pc_non_roce_out_tlast;

    // Convert signals @ axis_aclk to axil_aclk
    xpm_cdc_single #(
        .DEST_SYNC_FF  (4),  // DECIMAL; range: 2-10
        .INIT_SYNC_FF  (0),  // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
        .SIM_ASSERT_CHK(1),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .SRC_INPUT_REG (1)   // DECIMAL; 0=do not register input, 1=register input
    ) roce_pkt_recved_cdc (
        .dest_out(roce_pkt_recved),
        .dest_clk(axil_aclk),
        .src_clk (axis_aclk),
        .src_in  (roce_pkt_recved_at_axis_clk)
    );

    xpm_cdc_single #(
        .DEST_SYNC_FF  (4),  // DECIMAL; range: 2-10
        .INIT_SYNC_FF  (0),  // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
        .SIM_ASSERT_CHK(1),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .SRC_INPUT_REG (1)   // DECIMAL; 0=do not register input, 1=register input
    ) non_roce_pkt_recved_cdc (
        .dest_out(non_roce_pkt_recved),
        .dest_clk(axil_aclk),
        .src_clk (axis_aclk),
        .src_in  (non_roce_pkt_recved_at_axis_clk)
    );

    xpm_cdc_array_single #(
        .DEST_SYNC_FF  (4),  // DECIMAL; range: 2-10
        .INIT_SYNC_FF  (0),  // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
        .SIM_ASSERT_CHK(1),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .SRC_INPUT_REG (1),  // DECIMAL; 0=do not register input, 1=register input
        .WIDTH         (2)   // DECIMAL; range: 1-1024
    ) pkt_filter_err_cdc (
        .dest_out(packet_filter_err),  // WIDTH-bit output: src_in synchronized to the destination clock 
                                       // domain. This output is registered.

        .dest_clk(axil_aclk),      // 1-bit input: Clock signal for the destination clock domain.
        .src_clk (axis_aclk),      // 1-bit input: optional; required when SRC_INPUT_REG = 1
        .src_in  (pkt_filter_err)  // WIDTH-bit input: Input single-bit array to be synchronized to destination 
                                   //  clock domain. It is assumed that each bit of the array is unrelated to 
                                   //  the others. This is reflected in the constraints applied to this macro. 
                                   //  To transfer a binary value losslessly across the two clock domains, use 
                                   //  the XPM_CDC_GRAY macro instead.
    );

//     reg         s_axis_cmac_rx_tvalid_r1;
//     reg         s_axis_cmac_rx_tlast_r1;
//     reg         m_axis_user2rdma_roce_from_cmac_rx_tvalid_r1;
//     reg         m_axis_user2rdma_roce_from_cmac_rx_tlast_r1;
//     reg         m_axis_qdma_c2h_tvalid_r1;
//     reg         m_axis_qdma_c2h_tlast_r1;
//     reg         roce_pkt_recved_at_axis_clk_r1;
// always@(posedge axis_aclk)begin
//         if (!axis_rstn) begin
//             s_axis_cmac_rx_tvalid_r1 <= 1'd0;
//             s_axis_cmac_rx_tlast_r1 <= 1'd0;
//             m_axis_user2rdma_roce_from_cmac_rx_tvalid_r1 <= 1'd0;
//             m_axis_user2rdma_roce_from_cmac_rx_tlast_r1 <= 1'd0;
//             m_axis_qdma_c2h_tvalid_r1 <= 1'd0;
//             m_axis_qdma_c2h_tlast_r1 <= 1'd0;
//             roce_pkt_recved_at_axis_clk_r1 <= 1'd0;
//         end else begin
//             s_axis_cmac_rx_tvalid_r1 <= s_axis_cmac_rx_tvalid;
//             s_axis_cmac_rx_tlast_r1 <= s_axis_cmac_rx_tlast;
//             m_axis_user2rdma_roce_from_cmac_rx_tvalid_r1 <= m_axis_user2rdma_roce_from_cmac_rx_tvalid;
//             m_axis_user2rdma_roce_from_cmac_rx_tlast_r1 <= m_axis_user2rdma_roce_from_cmac_rx_tlast;
//             m_axis_qdma_c2h_tvalid_r1 <= m_axis_qdma_c2h_tvalid;
//             m_axis_qdma_c2h_tlast_r1 <= m_axis_qdma_c2h_tlast;
//             roce_pkt_recved_at_axis_clk_r1 <= roce_pkt_recved_at_axis_clk;
//         end
//     end

// ila_5 buffer_debug_ila (
//     .clk(axis_aclk),
//     .probe0(s_axis_cmac_rx_tvalid_r1),
//     .probe1(s_axis_cmac_rx_tlast_r1),
//     .probe2(m_axis_user2rdma_roce_from_cmac_rx_tvalid_r1),
//     .probe3(m_axis_user2rdma_roce_from_cmac_rx_tlast_r1),
//     .probe4(m_axis_qdma_c2h_tvalid_r1),
//     .probe5(m_axis_qdma_c2h_tlast_r1),
//     .probe6(roce_pkt_recved_at_axis_clk_r1)
// );

endmodule : reconic
