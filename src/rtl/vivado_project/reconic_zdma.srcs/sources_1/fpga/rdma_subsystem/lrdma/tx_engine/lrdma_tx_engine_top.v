`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_tx_engine_top — TX 引擎顶层模块
//
// 【模块概述】
// 将发送路径所有子模块串联，实现"WQE 入→以太网帧出"完整流水线。
//
// 【实例化层次与数据流】
// SQ WQE FIFO → hdr_gen → buf_mgr → (HDR BRAM + SGL BRAM) → dma → dre → crc_wrap → m_axis
//                 ↑                                            ↑
//            ack_buf ← RX path                          AXI4 Master (DDR)
//
// 【子模块】
// 1. lrdma_tx_hdr_gen：核心 FSM + 头部生成
// 2. lrdma_tx_buffer_mgr：缓冲管理（HDR BRAM + SGL BRAM）
// 3. lrdma_tx_dma_read_engine：DMA 引擎（AXI4 Master 读 + AXI-Stream 输出）
// 4. lrdma_tx_data_realign：数据重对齐（消除头部/payload 空隙）
// 5. lrdma_tx_icrc_wrap：ICRC 校验码附加
//
// 【AXI4 接口】
// 写通道全部硬连线为无效（仅使用读通道进行 DMA 数据搬运）
//
// 【支持操作】
// RDMA WRITE、MAD（QP1）管理帧
// 基于 PMTU 分片、基于 PSN 重传、PFC 反压处理
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_tx_engine_top #(
    parameter C_MAX_QP = 8,
    parameter C_MAX_QID_WIDTH = 3,  // 5
    parameter C_M_AXI_DATA_WIDTH = 512,
    parameter C_M_AXI_ADDR_WIDTH = 32,  //64
    parameter C_M_AXI_ID_WIDTH = 4,
    parameter C_AXIS_DATA_WIDTH = 512,
    parameter C_MAX_SGL_DEPTH = 128,
    parameter C_MAX_HDR_DEPTH = 512,
    parameter C_OS_Q_INDX_WIDTH = 3,  //4
    parameter C_ENABLE_TX_BUF_MGR_EARLY_RELEASE = 1,
    parameter C_ENABLE_FAST_WQE_CONSUME = 1
) (
    input core_clk,
    input core_rst,

    // tx_engine_m_axi_* 信号组
    // 写通道和写响应通道固定拉低，TX DMA 只发起读事务
    output [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_awid,
    output [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output [                     7:0] m_axi_awlen,
    output [                     2:0] m_axi_awsize,
    output [                     1:0] m_axi_awburst,
    output [                     3:0] m_axi_awcache,
    output [                     2:0] m_axi_awprot,
    output                            m_axi_awvalid,
    input                             m_axi_awready,
    output [  C_M_AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output [C_M_AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output                            m_axi_wlast,
    output                            m_axi_wvalid,
    input                             m_axi_wready,
    output                            m_axi_awlock,
    input  [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_bid,
    input  [                     1:0] m_axi_bresp,
    input                             m_axi_bvalid,
    output                            m_axi_bready,
    // 只用到了读通道
    output [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_arid,
    output [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output [                     7:0] m_axi_arlen,
    output [                     2:0] m_axi_arsize,
    output [                     1:0] m_axi_arburst,
    output [                     3:0] m_axi_arcache,
    output [                     2:0] m_axi_arprot,
    output                            m_axi_arvalid,
    input                             m_axi_arready,
    input  [  C_M_AXI_ID_WIDTH-1 : 0] m_axi_rid,
    input  [  C_M_AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  [                     1:0] m_axi_rresp,
    input                             m_axi_rlast,
    input                             m_axi_rvalid,
    output                            m_axi_rready,
    output                            m_axi_arlock,

    //AXI streaming interface signals
    output [C_AXIS_DATA_WIDTH-1 : 0] m_axis_tdata,
    output [C_AXIS_DATA_WIDTH/8-1:0] m_axis_tkeep,
    output                           m_axis_tvalid,
    input                            m_axis_tready,
    output                           m_axis_tlast,


    input                           i_qpm_fifo_empty,
    input  [                 511:0] i_qpm_wqe_data,
    input                           i_qpm_wqe_fast_valid,
    output                          o_qpm_wqe_pop,
    input                           i_wqe_halt,
    input  [C_MAX_QID_WIDTH -1 : 0] i_wqe_halted_qpid,
    output                          o_wqe_halted,

    output                       o_reg_rd_en,
    output                       o_reg_rd_en_s,
    input                        i_reg_rdy,
    input                        i_reg_psn_val,
    output                       o_reg_wr_en,
    output [C_MAX_QID_WIDTH-1:0] o_qp_id,
    input  [               15:0] i_reg_pkey,
    input  [                2:0] i_reg_pmtu,
    input  [               23:0] i_reg_dest_qpid,
    input  [               23:0] i_reg_sq_psn,
    output [               23:0] o_reg_sq_psn,
    input  [               23:0] i_reg_sq_msn,
    output [               23:0] o_reg_sq_msn,
    input  [                4:0] i_reg_qp_to,
    input  [               47:0] i_reg_mac_dest_addr,
    input  [              127:0] i_reg_ip_dest_addr,
    input  [               15:0] i_reg_udp_src_port,
    input  [                7:0] i_reg_ttl,
    input  [                5:0] i_reg_dscp,

    input [ 47:0] i_reg_mac_src_addr,
    input [ 31:0] i_reg_ipv4_src_addr,
    input [127:0] i_reg_ipv6_src_addr,
    input         i_reg_lrdma_en,
    input         i_reg_ip_ver,

    output                       o_wqp_req,
    input                        i_wqp_resp,
    output [C_MAX_QID_WIDTH-1:0] o_wqe_qpid,
    output [                7:0] o_wqe_opcode,
    output [               15:0] o_wqe_wrid,
    output [               23:0] o_wqe_send_psn,
    output [               23:0] o_wqe_send_first_psn,
    output [               23:0] o_wqe_send_msn,
    output [                4:0] o_wqe_qp_to,
    output                       o_wqe_retried,
    output                       o_wqe_explicit_ack_req,
    output                       o_wqe_fatal_push,
    output [               31:0] o_wqe_status_out,
    output [               31:0] o_last_out_pkt_info,
    output [               15:0] o_out_wr_pkt_cnt,
    output [               15:0] o_out_mad_pkt_cnt,

    output                             o_dma_in_idle,
    input                              i_qp_recovery,
    output [      C_MAX_QID_WIDTH-1:0] o_wqe_fatal_qpid,
    output                             o_wqe_fatal_qp_vld,
    input                              i_wqe_fatal_qp_rdy,
    output                             o_wqe_dropped,
    output [      C_MAX_QID_WIDTH-1:0] o_wqe_dropped_qpid,
    input                              i_osq_pop,
    input  [      C_MAX_QID_WIDTH-1:0] i_osq_pop_qpid,
    input  [    C_MAX_QID_WIDTH-1 : 0] i_qp_enable_idx,
    input                              i_qp_enable_vld,
    input                              i_qp_enable_data,
    input  [      C_MAX_QID_WIDTH-1:0] i_qp_fatal_idx,
    input                              i_qp_fatal_vld,
    input                              i_qp_fatal_data,
    input  [(4*C_MAX_QID_WIDTH+4)-1:0] i_osq_nacked,
    output                             o_osq_nacked_resp_wr_en,
    output [    C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr,
    output                             o_osq_nacked_resp_wr_data,
    output                             o_osq_nacked_resp_wr_en_aux,
    output [    C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr_aux,
    output                             o_osq_nacked_resp_wr_data_aux,
    output                             o_osq_nacked_resp_wr_en_pending,
    output [    C_MAX_QID_WIDTH-1 : 0] o_osq_nacked_resp_wr_addr_pending,
    output                             o_osq_nacked_resp_wr_data_pending,
    input                              init_start,
    input  [    C_MAX_QID_WIDTH-1 : 0] init_count,
    input                              i_qp_under_recovery_wr_en,
    input  [    C_MAX_QID_WIDTH-1 : 0] i_qp_under_recovery_wr_id,
    input                              i_qp_under_recovery_wr_data
);
    `include "lrdma_defs.vh"

    localparam HDR_BRAM_ADDR_WIDTH = clog2(C_MAX_HDR_DEPTH);

    wire                           buf_mngr_rdy;
    wire [                 1023:0] hdr_data;
    wire [                    7:0] hdr_len;
    wire                           hdr_valid;
    wire                           hdr_ip_ver;

    //SGL Buffer signals
    wire [                   15:0] sgl_head_ptr;
    wire [                   14:0] sgl_bram_rd_addr;
    wire                           sgl_bram_rd_en;
    wire [                   82:0] sgl_bram_rd_data;
    //Header buffer signals
    wire [                   14:0] hdr_bram_rd_addr;
    wire                           hdr_bram_rd_en;
    wire [                  511:0] hdr_bram_rd_data;
    wire                           hdr_bram_pop;

    wire [C_AXIS_DATA_WIDTH-1 : 0] axis_tdata_int;
    wire [C_AXIS_DATA_WIDTH/8-1:0] axis_tkeep_int;
    wire                           axis_tvalid_int;
    wire                           axis_tlast_int;
    wire                           axis_tready_int;
    wire [C_AXIS_DATA_WIDTH-1 : 0] m_axis_dma_tdata;
    wire [C_AXIS_DATA_WIDTH/8-1:0] m_axis_dma_tkeep;
    wire                           m_axis_dma_tvalid;
    wire                           m_axis_dma_tlast;
    wire                           m_axis_dma_tready;
    wire                           wqe_opcode_err;

    wire                           m_axis_tvalid_s;
    wire                           i_osq_nacked_wr_en;
    wire [  C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr;
    wire                           i_osq_nacked_wr_en_pending;
    wire [  C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_pending;
    wire                           i_osq_nacked_wr_en_aux;
    wire [  C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_aux;
    wire                           i_osq_nacked_wr_en_aux_pending;
    wire [  C_MAX_QID_WIDTH-1 : 0] i_osq_nacked_wr_addr_aux_pending;

    wire                           dma_in_idle;

    assign m_axis_tvalid = m_axis_tvalid_s;



    wire [511:0] padded_data;wire [63:0] padded_keep;
    wire padded_valid,padded_last,padded_ready;
    lrdma_tx_padding tx_padding_inst(.clk(core_clk),.rst(core_rst),
     .s_data(axis_tdata_int),.s_keep(axis_tkeep_int),.s_valid(axis_tvalid_int),.s_last(axis_tlast_int),.s_ready(axis_tready_int),
     .m_data(padded_data),.m_keep(padded_keep),.m_valid(padded_valid),.m_last(padded_last),.m_ready(padded_ready));
    lrdma_tx_icrc_wrap tx_icrc_wrap_inst (  // CRC Logic 模块
        .core_clk     (core_clk),
        .core_rst     (core_rst),
        .m_axis_tdata (m_axis_tdata),
        .m_axis_tkeep (m_axis_tkeep),
        .m_axis_tvalid(m_axis_tvalid_s),
        .m_axis_tready(m_axis_tready),
        .m_axis_tlast (m_axis_tlast),

        .s_axis_tdata (padded_data),
        .s_axis_tkeep (padded_keep),
        .s_axis_tvalid(padded_valid),
        .s_axis_tlast (padded_last),
        .s_axis_tready(padded_ready)
    );



    lrdma_tx_data_realign #(
        .C_AXIS_DATA_WIDTH (512),
        .C_M_AXI_DATA_WIDTH(C_M_AXI_DATA_WIDTH),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_ID_WIDTH  (C_M_AXI_ID_WIDTH),
        .IP2BUS_LEN_WIDTH  (16)
    ) tx_data_realign_inst (  // WQE 数据重新对齐块实例
        .core_clk(core_clk),
        .core_rst(core_rst),

        .m_axis_tdata (axis_tdata_int),
        .m_axis_tkeep (axis_tkeep_int),
        .m_axis_tvalid(axis_tvalid_int),
        .m_axis_tready(axis_tready_int),
        .m_axis_tlast (axis_tlast_int),

        .s_axis_tdata (m_axis_dma_tdata),
        .s_axis_tkeep (m_axis_dma_tkeep),
        .s_axis_tvalid(m_axis_dma_tvalid),
        .s_axis_tlast (m_axis_dma_tlast),
        .s_axis_tready(m_axis_dma_tready)
    );

    // 消费 WQE，组装完整 RoCEv2 协议头部。包含 PMTU 切片、PSN 递增、OSQ 管理。
    lrdma_tx_hdr_gen #(
        .C_MAX_QP(C_MAX_QP),
        .C_MAX_QID_WIDTH(C_MAX_QID_WIDTH),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_OS_Q_INDX_WIDTH(C_OS_Q_INDX_WIDTH),
        .C_ENABLE_FAST_WQE_CONSUME(C_ENABLE_FAST_WQE_CONSUME)
    ) tx_hdr_gen_inst (
        .core_clk(core_clk),
        .core_rst(core_rst),

        .i_qpm_fifo_empty (i_qpm_fifo_empty),
        .i_qpm_wqe_data   (i_qpm_wqe_data),
        .i_qpm_wqe_fast_valid(i_qpm_wqe_fast_valid),
        .o_qpm_wqe_pop    (o_qpm_wqe_pop),
        .i_wqe_halt       (i_wqe_halt),
        .i_wqe_halted_qpid(i_wqe_halted_qpid),
        .o_wqe_halted     (o_wqe_halted),

        .o_reg_rd_en        (o_reg_rd_en),
        .o_reg_rd_en_s      (o_reg_rd_en_s),
        .i_reg_info_val     (i_reg_rdy),
        .i_reg_psn_val      (i_reg_psn_val),
        .o_reg_wr_en        (o_reg_wr_en),
        .o_qp_id            (o_qp_id),
        .i_reg_pkey         (i_reg_pkey),
        .i_reg_qp_to        (i_reg_qp_to),
        .i_reg_pmtu         (i_reg_pmtu),
        .i_reg_dest_qpid    (i_reg_dest_qpid),
        .i_reg_sq_psn       (i_reg_sq_psn),
        .o_reg_sq_psn       (o_reg_sq_psn),
        .i_reg_sq_msn       (i_reg_sq_msn),
        .o_reg_sq_msn       (o_reg_sq_msn),
        .i_reg_mac_dest_addr(i_reg_mac_dest_addr),
        .i_reg_ip_dest_addr (i_reg_ip_dest_addr),
        .i_reg_udp_src_port (i_reg_udp_src_port),
        .i_reg_ttl          (i_reg_ttl),
        .i_reg_dscp         (i_reg_dscp),

        .i_reg_mac_src_addr (i_reg_mac_src_addr),
        .i_reg_ipv4_src_addr(i_reg_ipv4_src_addr),
        .i_reg_ipv6_src_addr(i_reg_ipv6_src_addr),
        .i_reg_lrdma_en     (i_reg_lrdma_en),
        .i_reg_ip_ver       (i_reg_ip_ver),

        .i_buf_mngr_rdy(buf_mngr_rdy),
        .o_hdr_data    (hdr_data),
        .o_hdr_len     (hdr_len),
        .o_hdr_valid   (hdr_valid),
        .o_hdr_ip_ver  (hdr_ip_ver),

        .o_wqp_req             (o_wqp_req),
        .i_wqp_resp            (i_wqp_resp),
        .o_wqe_qpid            (o_wqe_qpid),
        .o_wqe_opcode          (o_wqe_opcode),
        .o_wqe_wrid            (o_wqe_wrid),
        .o_wqe_send_psn        (o_wqe_send_psn),
        .o_wqe_send_first_psn  (o_wqe_send_first_psn),
        .o_wqe_send_msn        (o_wqe_send_msn),
        .o_wqe_explicit_ack_req(o_wqe_explicit_ack_req),
        .o_wqe_fatal_push      (o_wqe_fatal_push),
        .o_wqe_qp_to           (o_wqe_qp_to),
        .o_wqe_retried         (o_wqe_retried),

        .wqe_opcode_err     (wqe_opcode_err),
        .wqe_status_out     (o_wqe_status_out[11:0]),
        .o_last_out_pkt_info(o_last_out_pkt_info),
        .o_out_wr_pkt_cnt   (o_out_wr_pkt_cnt),
        .o_out_mad_pkt_cnt  (o_out_mad_pkt_cnt),
        .i_qp_recovery      (i_qp_recovery),
        .o_dma_in_idle      (o_dma_in_idle),
        .dma_in_idle        (dma_in_idle),
        .o_wqe_fatal_qpid   (o_wqe_fatal_qpid),
        .o_wqe_fatal_qp_vld (o_wqe_fatal_qp_vld),
        .i_wqe_fatal_qp_rdy (i_wqe_fatal_qp_rdy),
        .i_osq_pop          (i_osq_pop),
        .i_osq_pop_qpid     (i_osq_pop_qpid),
        .o_wqe_dropped      (o_wqe_dropped),
        .o_wqe_dropped_qpid (o_wqe_dropped_qpid),
        .init_start         (init_start),
        .init_count         (init_count),
        .i_qp_fatal_wr_en   (i_qp_fatal_vld),
        .i_qp_fatal_wr_addr (i_qp_fatal_idx),
        .i_qp_fatal_wr_data (i_qp_fatal_data),

        // input
        // osq nacked
        .i_osq_nacked_wr_en              (i_osq_nacked_wr_en),
        .i_osq_nacked_wr_addr            (i_osq_nacked_wr_addr),
        .i_osq_nacked_wr_data            (1'b1),
        // osq nacked aux
        .i_osq_nacked_wr_en_aux          (i_osq_nacked_wr_en_aux),
        .i_osq_nacked_wr_addr_aux        (i_osq_nacked_wr_addr_aux),
        .i_osq_nacked_wr_data_aux        (1'b0),
        // osq nacked pending
        .i_osq_nacked_wr_en_pending      (i_osq_nacked_wr_en_pending),
        .i_osq_nacked_wr_addr_pending    (i_osq_nacked_wr_addr_pending),
        // osq nacked aux pending
        .i_osq_nacked_wr_en_aux_pending  (i_osq_nacked_wr_en_aux_pending),
        .i_osq_nacked_wr_addr_aux_pending(i_osq_nacked_wr_addr_aux_pending),

        // output
        // osq nacked 
        .o_osq_nacked_resp_wr_en          (o_osq_nacked_resp_wr_en),
        .o_osq_nacked_resp_wr_addr        (o_osq_nacked_resp_wr_addr),
        .o_osq_nacked_resp_wr_data        (o_osq_nacked_resp_wr_data),
        // osq nacked aux
        .o_osq_nacked_resp_wr_en_aux      (o_osq_nacked_resp_wr_en_aux),
        .o_osq_nacked_resp_wr_addr_aux    (o_osq_nacked_resp_wr_addr_aux),
        .o_osq_nacked_resp_wr_data_aux    (o_osq_nacked_resp_wr_data_aux),
        // osq nacked pending
        .o_osq_nacked_resp_wr_en_pending  (o_osq_nacked_resp_wr_en_pending),
        .o_osq_nacked_resp_wr_addr_pending(o_osq_nacked_resp_wr_addr_pending),
        .o_osq_nacked_resp_wr_data_pending(o_osq_nacked_resp_wr_data_pending)
    );

    // osq_nacked 信号解包
    assign {i_osq_nacked_wr_en, i_osq_nacked_wr_addr, i_osq_nacked_wr_en_pending, i_osq_nacked_wr_addr_pending, i_osq_nacked_wr_en_aux, i_osq_nacked_wr_addr_aux, i_osq_nacked_wr_en_aux_pending, i_osq_nacked_wr_addr_aux_pending} = i_osq_nacked;


    lrdma_tx_buffer_mgr #(
        .C_MAX_QP(C_MAX_QP),
        .C_MAX_QID_WIDTH(C_MAX_QID_WIDTH),
        .C_MAX_SGL_DEPTH(C_MAX_SGL_DEPTH),
        .C_MAX_HDR_DEPTH(C_MAX_HDR_DEPTH),
        .C_ENABLE_EARLY_BUF_MGR_READY(C_ENABLE_TX_BUF_MGR_EARLY_RELEASE)
    ) tx_buf_mgr_inst (
        .core_clk(core_clk),
        .core_rst(core_rst),

        .o_buf_mngr_rdy(buf_mngr_rdy),
        .i_hdr_data    (hdr_data),
        .i_hdr_len     (hdr_len),
        .i_hdr_valid   (hdr_valid),
        .i_reg_ip_ver  (hdr_ip_ver),

        .sgl_head_ptr    (sgl_head_ptr),
        .sgl_bram_rd_addr(sgl_bram_rd_addr),
        .sgl_bram_rd_en  (sgl_bram_rd_en),
        .sgl_bram_rd_data(sgl_bram_rd_data),

        .hdr_bram_rd_addr(hdr_bram_rd_addr),
        .hdr_bram_rd_en  (hdr_bram_rd_en),
        .hdr_bram_pop    (hdr_bram_pop),
        .hdr_bram_rd_data(hdr_bram_rd_data),
        .buf_mgr_ptr     (o_wqe_status_out[31:12])
    );


    lrdma_tx_dma_read_engine #(
        .C_M_AXI_DATA_WIDTH(C_M_AXI_DATA_WIDTH),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_ID_WIDTH(C_M_AXI_ID_WIDTH),
        .C_AXIS_DATA_WIDTH(C_AXIS_DATA_WIDTH),
        .C_MAX_SGL_DEPTH(C_MAX_SGL_DEPTH),
        .C_SGL_DATA_WIDTH(83),  //C_SGL_DATA_WIDTH),
        .C_HDR_BUF_ADDR_WIDTH(HDR_BRAM_ADDR_WIDTH),
        .C_HDR_BUF_DEPTH(C_MAX_HDR_DEPTH)
    ) tx_dma_read_engine_inst (

        .core_clk(core_clk),
        .core_rst(core_rst),

        .m_axi_dma_arid   (m_axi_arid),
        .m_axi_dma_araddr (m_axi_araddr),
        .m_axi_dma_arlen  (m_axi_arlen),
        .m_axi_dma_arsize (m_axi_arsize),
        .m_axi_dma_arburst(m_axi_arburst),
        .m_axi_dma_arcache(m_axi_arcache),
        .m_axi_dma_arprot (m_axi_arprot),
        .m_axi_dma_arvalid(m_axi_arvalid),
        .m_axi_dma_arready(m_axi_arready),
        .m_axi_dma_rid    (m_axi_rid),
        .m_axi_dma_rdata  (m_axi_rdata),
        .m_axi_dma_rresp  (m_axi_rresp),
        .m_axi_dma_rlast  (m_axi_rlast),
        .m_axi_dma_rvalid (m_axi_rvalid),
        .m_axi_dma_rready (m_axi_rready),
        .m_axi_dma_arlock (m_axi_arlock),

        .m_axis_dma_tdata (m_axis_dma_tdata),
        .m_axis_dma_tkeep (m_axis_dma_tkeep),
        .m_axis_dma_tvalid(m_axis_dma_tvalid),
        .m_axis_dma_tready(m_axis_dma_tready),
        .m_axis_dma_tlast (m_axis_dma_tlast),

        .sgl_head_ptr    (sgl_head_ptr),
        .sgl_bram_rd_addr(sgl_bram_rd_addr),
        .sgl_bram_rd_en  (sgl_bram_rd_en),
        .sgl_bram_rd_data(sgl_bram_rd_data),

        .hdr_bram_rd_addr(hdr_bram_rd_addr),
        .hdr_bram_rd_en  (hdr_bram_rd_en),
        .hdr_bram_rd_data(hdr_bram_rd_data),
        .hdr_bram_pop    (hdr_bram_pop),

        .o_dma_in_idle(dma_in_idle)
    );


    assign m_axi_awid    = {C_M_AXI_ID_WIDTH{1'b0}};
    assign m_axi_awaddr  = {C_M_AXI_ADDR_WIDTH{1'b0}};
    assign m_axi_awlen   = 8'h00;
    assign m_axi_awsize  = 3'b000;
    assign m_axi_awburst = 2'b00;
    assign m_axi_awcache = 4'h0;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awvalid = 1'b0;
    assign m_axi_wdata   = {C_M_AXI_DATA_WIDTH{1'b0}};
    assign m_axi_wstrb   = {C_M_AXI_DATA_WIDTH / 8{1'b0}};
    assign m_axi_wlast   = 1'b0;
    assign m_axi_wvalid  = 1'b0;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_bready  = 1'b1;

endmodule
