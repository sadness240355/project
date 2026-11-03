`default_nettype wire
//-----------------------------------------------------------------------------
// xrnic.v
//-----------------------------------------------------------------------------
//
// Filename        : xrnic.v
// Version         : v1.0
// Description     : XRNIC IP top level file 
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
// Naming Conventions:
//      active low signals:                     "*_n"
//      clock signals:                          "clk", "clk_div#", "clk_#x" 
//      reset signals:                          "rst", "rst_n" 
//      parameters:                             "C_*" 
//      user defined types:                     "*_TYPE" 
//      state machine next state:               "*_ns" 
//      state machine current state:            "*_cs" 
//      combinatorial signals:                  "*_com" 
//      pipelined or register delay signals:    "*_d#" 
//      counter signals:                        "*cnt*"
//      clock enable signals:                   "*_ce" 
//      internal version of output port         "*_i"
//      device pins:                            "*_pin" 
//      component instantiations:               "<MODULE>I_<#|FUNC>
//-----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// tips:
// 1、该ip核里面涉及到的所有qp index / qpid，实际上的QP号(i)要在其基础上加上1。例: qpid = 0时，实际QP号/i = 1；qpid = 31时，实际QP号/i = 32。
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module rdma_core_usr #(
    parameter C_NUM_QP = 32,
    parameter C_EN_FLOW_CTRL = 0,
    parameter C_NUM_PD = 2048,
    parameter C_M_AXI_ADDR_WIDTH = 64,
    parameter C_M_AXI_AWUSER_WIDTH = 32,
    parameter C_M_AXI_ID_WIDTH = 1,
    parameter C_S_AXI_LITE_ADDR_WIDTH = 32,
    parameter C_EN_DEBUG_PORTS = 0,
    parameter C_EN_NVMOF_HW_HNDSHK = 0,  // Enabled hardware handshake with NVMoF
    parameter C_MAX_SGL_DEPTH = 128,
    parameter C_MAX_WR_RETRY_DATA_BUF_DEPTH = 128,
    parameter C_EN_WR_RETRY_DATA_BUF = 0,
    parameter C_ECN_THRESHOLD = 12,
    parameter C_OSQ_PSN_WIDTH = 16,
    parameter C_EN_INITIATOR_LITE = 1,
    parameter C_EN_RD_RESP_MIDDLE = 1,
    parameter C_MAX_RD_OS = 16,
    parameter NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN = 0,
    parameter NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE = 0,
    parameter NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING = 0,
    parameter C_EN_IETH_IMMDT = 1,
    parameter SIM_REGS_EN = 0
    // parameter C_HWTIMEOUT = 1
) (
    input  m_axi_aclk,
    input  cmac_rx_clk,
    input  cmac_rx_rst,
    input  cmac_tx_clk,
    input  cmac_tx_rst,
    input  m_axi_aresetn,
    input  s_axi_lite_aclk,
    input  s_axi_lite_aresetn,
    output system_resetn,

    // AXI Master I/F for rx packet handler for DDR
    // RX 数据包处理模块使用该接口将传入的 MAD、SEND 和 RDMA WRITE 数据包中的数据推送到 DDR 中的 RQ 缓冲区。
    // 仅支持 64B 对齐的事务。
    output wire [    C_M_AXI_ID_WIDTH-1:0] rx_pkt_hndler_ddr_m_axi_awid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] rx_pkt_hndler_ddr_m_axi_awaddr,
    output wire [C_M_AXI_AWUSER_WIDTH-1:0] rx_pkt_hndler_ddr_m_axi_awuser,
    output wire [                     7:0] rx_pkt_hndler_ddr_m_axi_awlen,
    output wire [                     2:0] rx_pkt_hndler_ddr_m_axi_awsize,
    output wire [                     1:0] rx_pkt_hndler_ddr_m_axi_awburst,
    output wire [                     3:0] rx_pkt_hndler_ddr_m_axi_awcache,
    output wire [                     2:0] rx_pkt_hndler_ddr_m_axi_awprot,
    output wire                            rx_pkt_hndler_ddr_m_axi_awvalid,
    input  wire                            rx_pkt_hndler_ddr_m_axi_awready,
    output wire [                   511:0] rx_pkt_hndler_ddr_m_axi_wdata,
    output wire [                    63:0] rx_pkt_hndler_ddr_m_axi_wstrb,
    output wire                            rx_pkt_hndler_ddr_m_axi_wlast,
    output wire                            rx_pkt_hndler_ddr_m_axi_wvalid,
    input  wire                            rx_pkt_hndler_ddr_m_axi_wready,
    output wire                            rx_pkt_hndler_ddr_m_axi_awlock,
    input  wire [  C_M_AXI_ID_WIDTH-1 : 0] rx_pkt_hndler_ddr_m_axi_bid,
    input  wire [                     1:0] rx_pkt_hndler_ddr_m_axi_bresp,
    input  wire                            rx_pkt_hndler_ddr_m_axi_bvalid,
    output wire                            rx_pkt_hndler_ddr_m_axi_bready,
    output wire [  C_M_AXI_ID_WIDTH-1 : 0] rx_pkt_hndler_ddr_m_axi_arid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] rx_pkt_hndler_ddr_m_axi_araddr,
    output wire [                     7:0] rx_pkt_hndler_ddr_m_axi_arlen,
    output wire [                     2:0] rx_pkt_hndler_ddr_m_axi_arsize,
    output wire [                     1:0] rx_pkt_hndler_ddr_m_axi_arburst,
    output wire [                     3:0] rx_pkt_hndler_ddr_m_axi_arcache,
    output wire [                     2:0] rx_pkt_hndler_ddr_m_axi_arprot,
    output wire                            rx_pkt_hndler_ddr_m_axi_arvalid,
    input  wire                            rx_pkt_hndler_ddr_m_axi_arready,
    input  wire [  C_M_AXI_ID_WIDTH-1 : 0] rx_pkt_hndler_ddr_m_axi_rid,
    input  wire [                   511:0] rx_pkt_hndler_ddr_m_axi_rdata,
    input  wire [                     1:0] rx_pkt_hndler_ddr_m_axi_rresp,
    input  wire                            rx_pkt_hndler_ddr_m_axi_rlast,
    input  wire                            rx_pkt_hndler_ddr_m_axi_rvalid,
    output wire                            rx_pkt_hndler_ddr_m_axi_rready,
    output wire                            rx_pkt_hndler_ddr_m_axi_arlock,

    // AXI Master I/F for rx packet handler 
    // RX 数据包处理程序使用该接口将读响应（对方传来的响应）中的有效负载数据写入 DDR。
    output wire [  C_M_AXI_ID_WIDTH-1:0] rx_pkt_hndler_rdrsp_m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] rx_pkt_hndler_rdrsp_m_axi_awaddr,
    output wire [                   7:0] rx_pkt_hndler_rdrsp_m_axi_awlen,
    output wire [                   2:0] rx_pkt_hndler_rdrsp_m_axi_awsize,
    output wire [                   1:0] rx_pkt_hndler_rdrsp_m_axi_awburst,
    output wire [                   3:0] rx_pkt_hndler_rdrsp_m_axi_awcache,
    output wire [                   2:0] rx_pkt_hndler_rdrsp_m_axi_awprot,
    output wire                          rx_pkt_hndler_rdrsp_m_axi_awvalid,
    input  wire                          rx_pkt_hndler_rdrsp_m_axi_awready,
    output wire [                 511:0] rx_pkt_hndler_rdrsp_m_axi_wdata,
    output wire [                  63:0] rx_pkt_hndler_rdrsp_m_axi_wstrb,
    output wire                          rx_pkt_hndler_rdrsp_m_axi_wlast,
    output wire                          rx_pkt_hndler_rdrsp_m_axi_wvalid,
    input  wire                          rx_pkt_hndler_rdrsp_m_axi_wready,
    output wire                          rx_pkt_hndler_rdrsp_m_axi_awlock,
    input  wire [C_M_AXI_ID_WIDTH-1 : 0] rx_pkt_hndler_rdrsp_m_axi_bid,
    input  wire [                   1:0] rx_pkt_hndler_rdrsp_m_axi_bresp,
    input  wire                          rx_pkt_hndler_rdrsp_m_axi_bvalid,
    output wire                          rx_pkt_hndler_rdrsp_m_axi_bready,
    output wire [C_M_AXI_ID_WIDTH-1 : 0] rx_pkt_hndler_rdrsp_m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] rx_pkt_hndler_rdrsp_m_axi_araddr,
    output wire [                   7:0] rx_pkt_hndler_rdrsp_m_axi_arlen,
    output wire [                   2:0] rx_pkt_hndler_rdrsp_m_axi_arsize,
    output wire [                   1:0] rx_pkt_hndler_rdrsp_m_axi_arburst,
    output wire [                   3:0] rx_pkt_hndler_rdrsp_m_axi_arcache,
    output wire [                   2:0] rx_pkt_hndler_rdrsp_m_axi_arprot,
    output wire                          rx_pkt_hndler_rdrsp_m_axi_arvalid,
    input  wire                          rx_pkt_hndler_rdrsp_m_axi_arready,
    input  wire [C_M_AXI_ID_WIDTH-1 : 0] rx_pkt_hndler_rdrsp_m_axi_rid,
    input  wire [                 511:0] rx_pkt_hndler_rdrsp_m_axi_rdata,
    input  wire [                   1:0] rx_pkt_hndler_rdrsp_m_axi_rresp,
    input  wire                          rx_pkt_hndler_rdrsp_m_axi_rlast,
    input  wire                          rx_pkt_hndler_rdrsp_m_axi_rvalid,
    output wire                          rx_pkt_hndler_rdrsp_m_axi_rready,
    output wire                          rx_pkt_hndler_rdrsp_m_axi_arlock,


    // Ethernet controller I/F                                                  
    // AXI streaming master feeding to the RX packet handler
    // 该接口将网络接口的 RoCE 数据包提供给XRNIC。
    input wire         roce_cmac_s_axis_tvalid,
    input wire [511:0] roce_cmac_s_axis_tdata,
    input wire [ 63:0] roce_cmac_s_axis_tkeep,
    input wire         roce_cmac_s_axis_tlast,
    input wire         roce_cmac_s_axis_tuser,   // 

    // AXI streaming master feeding to the non Roce buffer
    // 该接口将网络接口的非 RoCE 数据包提供给XRNIC。
    // tip: 延迟了几拍后输出到 non_roce_dma_s_axis 接口
    input wire         non_roce_cmac_s_axis_tvalid,
    input wire [511:0] non_roce_cmac_s_axis_tdata,
    input wire [ 63:0] non_roce_cmac_s_axis_tkeep,
    input wire         non_roce_cmac_s_axis_tlast,
    input wire         non_roce_cmac_s_axis_tuser,

    // AXI streaming master from DMA to non_roce buffer
    // 这是一条从 DMA 模块到 flow control manager 的传入非 RDMA 路径
    input  wire         non_roce_dma_s_axis_tvalid,
    input  wire [511:0] non_roce_dma_s_axis_tdata,
    input  wire [ 63:0] non_roce_dma_s_axis_tkeep,
    input  wire         non_roce_dma_s_axis_tlast,
    output wire         non_roce_dma_s_axis_tready,

    // AXI Master I/F for wqe_proc engine
    // 此接口被 WQE processor engine 用于从 DDR 中读取传出的 RDMA SEND/WRITE 和 READ responses中要发送的数据。
    output wire [  C_M_AXI_ID_WIDTH-1:0] wqe_proc_top_m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] wqe_proc_top_m_axi_awaddr,
    output wire [                   7:0] wqe_proc_top_m_axi_awlen,
    output wire [                   2:0] wqe_proc_top_m_axi_awsize,
    output wire [                   1:0] wqe_proc_top_m_axi_awburst,
    output wire [                   3:0] wqe_proc_top_m_axi_awcache,
    output wire [                   2:0] wqe_proc_top_m_axi_awprot,
    output wire                          wqe_proc_top_m_axi_awvalid,
    input  wire                          wqe_proc_top_m_axi_awready,
    output wire [                 511:0] wqe_proc_top_m_axi_wdata,
    output wire [                  63:0] wqe_proc_top_m_axi_wstrb,
    output wire                          wqe_proc_top_m_axi_wlast,
    output wire                          wqe_proc_top_m_axi_wvalid,
    input  wire                          wqe_proc_top_m_axi_wready,
    output wire                          wqe_proc_top_m_axi_awlock,
    input  wire [C_M_AXI_ID_WIDTH-1 : 0] wqe_proc_top_m_axi_bid,
    input  wire [                   1:0] wqe_proc_top_m_axi_bresp,
    input  wire                          wqe_proc_top_m_axi_bvalid,
    output wire                          wqe_proc_top_m_axi_bready,
    output wire [C_M_AXI_ID_WIDTH-1 : 0] wqe_proc_top_m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] wqe_proc_top_m_axi_araddr,
    output wire [                   7:0] wqe_proc_top_m_axi_arlen,
    output wire [                   2:0] wqe_proc_top_m_axi_arsize,
    output wire [                   1:0] wqe_proc_top_m_axi_arburst,
    output wire [                   3:0] wqe_proc_top_m_axi_arcache,
    output wire [                   2:0] wqe_proc_top_m_axi_arprot,
    output wire                          wqe_proc_top_m_axi_arvalid,
    input  wire                          wqe_proc_top_m_axi_arready,
    input  wire [C_M_AXI_ID_WIDTH-1 : 0] wqe_proc_top_m_axi_rid,
    input  wire [                 511:0] wqe_proc_top_m_axi_rdata,
    input  wire [                   1:0] wqe_proc_top_m_axi_rresp,
    input  wire                          wqe_proc_top_m_axi_rlast,
    input  wire                          wqe_proc_top_m_axi_rvalid,
    output wire                          wqe_proc_top_m_axi_rready,
    output wire                          wqe_proc_top_m_axi_arlock,

    // AXI Master I/F for wqe_proc engine
    // WQE processor engine 使用该接口将传出的写数据包的数据写入写重试缓冲区。
    output wire [  C_M_AXI_ID_WIDTH-1:0] wqe_proc_wr_ddr_m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] wqe_proc_wr_ddr_m_axi_awaddr,
    output wire [                   7:0] wqe_proc_wr_ddr_m_axi_awlen,
    output wire [                   2:0] wqe_proc_wr_ddr_m_axi_awsize,
    output wire [                   1:0] wqe_proc_wr_ddr_m_axi_awburst,
    output wire [                   3:0] wqe_proc_wr_ddr_m_axi_awcache,
    output wire [                   2:0] wqe_proc_wr_ddr_m_axi_awprot,
    output wire                          wqe_proc_wr_ddr_m_axi_awvalid,
    input  wire                          wqe_proc_wr_ddr_m_axi_awready,
    output wire [                 511:0] wqe_proc_wr_ddr_m_axi_wdata,
    output wire [                  63:0] wqe_proc_wr_ddr_m_axi_wstrb,
    output wire                          wqe_proc_wr_ddr_m_axi_wlast,
    output wire                          wqe_proc_wr_ddr_m_axi_wvalid,
    input  wire                          wqe_proc_wr_ddr_m_axi_wready,
    output wire                          wqe_proc_wr_ddr_m_axi_awlock,
    input  wire [C_M_AXI_ID_WIDTH-1 : 0] wqe_proc_wr_ddr_m_axi_bid,
    input  wire [                   1:0] wqe_proc_wr_ddr_m_axi_bresp,
    input  wire                          wqe_proc_wr_ddr_m_axi_bvalid,
    output wire                          wqe_proc_wr_ddr_m_axi_bready,
    output wire [C_M_AXI_ID_WIDTH-1 : 0] wqe_proc_wr_ddr_m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] wqe_proc_wr_ddr_m_axi_araddr,
    output wire [                   7:0] wqe_proc_wr_ddr_m_axi_arlen,
    output wire [                   2:0] wqe_proc_wr_ddr_m_axi_arsize,
    output wire [                   1:0] wqe_proc_wr_ddr_m_axi_arburst,
    output wire [                   3:0] wqe_proc_wr_ddr_m_axi_arcache,
    output wire [                   2:0] wqe_proc_wr_ddr_m_axi_arprot,
    output wire                          wqe_proc_wr_ddr_m_axi_arvalid,
    input  wire                          wqe_proc_wr_ddr_m_axi_arready,
    input  wire [C_M_AXI_ID_WIDTH-1 : 0] wqe_proc_wr_ddr_m_axi_rid,
    input  wire [                 511:0] wqe_proc_wr_ddr_m_axi_rdata,
    input  wire [                   1:0] wqe_proc_wr_ddr_m_axi_rresp,
    input  wire                          wqe_proc_wr_ddr_m_axi_rlast,
    input  wire                          wqe_proc_wr_ddr_m_axi_rvalid,
    output wire                          wqe_proc_wr_ddr_m_axi_rready,
    output wire                          wqe_proc_wr_ddr_m_axi_arlock,

    // AXI streaming interface signals to and from WQE proc CRC process engine
    // 往返于 WQE proc CRC process engine 的 AXI 流媒体接口信号
    output wire [511 : 0] cmac_m_axis_tdata,
    output wire [   63:0] cmac_m_axis_tkeep,
    output wire           cmac_m_axis_tvalid,
    input  wire           cmac_m_axis_tready,
    output wire           cmac_m_axis_tlast,

    // AXI streaming master non roce buffer to DMA
    // AXI streaming 从 IP 到 DMA 模块的传出非 RDMA 路径。
    output wire [511 : 0] non_roce_dma_m_axis_tdata,
    output wire [   63:0] non_roce_dma_m_axis_tkeep,
    output wire           non_roce_dma_m_axis_tvalid,
    input  wire           non_roce_dma_m_axis_tready,
    output wire           non_roce_dma_m_axis_tlast,

    //input  wire [C_AXIS_DATA_WIDTH-1 : 0]      wqe_proc_top_s_axis_tdata,
    //input  wire [C_AXIS_DATA_WIDTH/8-1:0]      wqe_proc_top_s_axis_tkeep,
    //input  wire                                wqe_proc_top_s_axis_tvalid,
    //input  wire                                wqe_proc_top_s_axis_tlast,
    //output wire                                wqe_proc_top_s_axis_tready,

    // 来自 resp_handler 模块的 AXI 主站信号
    // 此接口由 response handler 用于将完成信息写入 DDR 中存在的完成队列。
    // Write address channel
    output wire [                   0:0] resp_hndler_m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] resp_hndler_m_axi_awaddr,
    output wire [                   7:0] resp_hndler_m_axi_awlen,
    output wire [                   2:0] resp_hndler_m_axi_awsize,
    output wire [                   1:0] resp_hndler_m_axi_awburst,
    output wire [                   3:0] resp_hndler_m_axi_awcache,
    output wire [                   2:0] resp_hndler_m_axi_awprot,
    output wire                          resp_hndler_m_axi_awvalid,
    input  wire                          resp_hndler_m_axi_awready,
    // Write data channel
    output wire [                 511:0] resp_hndler_m_axi_wdata,
    output wire [             512/8-1:0] resp_hndler_m_axi_wstrb,
    output wire                          resp_hndler_m_axi_wlast,
    output wire                          resp_hndler_m_axi_wvalid,
    input  wire                          resp_hndler_m_axi_wready,
    output wire                          resp_hndler_m_axi_awlock,
    // Write Response channel
    input  wire [                 0 : 0] resp_hndler_m_axi_bid,
    input  wire [                   1:0] resp_hndler_m_axi_bresp,
    input  wire                          resp_hndler_m_axi_bvalid,
    output wire                          resp_hndler_m_axi_bready,
    // Read address channel
    output wire [                   0:0] resp_hndler_m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] resp_hndler_m_axi_araddr,
    output wire [                   7:0] resp_hndler_m_axi_arlen,
    output wire [                   2:0] resp_hndler_m_axi_arsize,
    output wire [                   1:0] resp_hndler_m_axi_arburst,
    output wire [                   3:0] resp_hndler_m_axi_arcache,
    output wire [                   2:0] resp_hndler_m_axi_arprot,
    output wire                          resp_hndler_m_axi_arvalid,
    input  wire                          resp_hndler_m_axi_arready,
    // Read data/response channel
    input  wire [                   0:0] resp_hndler_m_axi_rid,
    input  wire [                 511:0] resp_hndler_m_axi_rdata,
    input  wire [                   1:0] resp_hndler_m_axi_rresp,
    input  wire                          resp_hndler_m_axi_rlast,
    input  wire                          resp_hndler_m_axi_rvalid,
    output wire                          resp_hndler_m_axi_rready,
    output wire                          resp_hndler_m_axi_arlock,

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
    // Write address channel
    output wire [                   0:0] qp_mgr_m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_m_axi_awaddr,
    output wire [                   7:0] qp_mgr_m_axi_awlen,
    output wire [                   2:0] qp_mgr_m_axi_awsize,
    output wire [                   1:0] qp_mgr_m_axi_awburst,
    output wire [                   3:0] qp_mgr_m_axi_awcache,
    output wire [                   2:0] qp_mgr_m_axi_awprot,
    output wire                          qp_mgr_m_axi_awvalid,
    input  wire                          qp_mgr_m_axi_awready,
    // Write data channel
    output wire [                 511:0] qp_mgr_m_axi_wdata,
    output wire [             512/8-1:0] qp_mgr_m_axi_wstrb,
    output wire                          qp_mgr_m_axi_wlast,
    output wire                          qp_mgr_m_axi_wvalid,
    input  wire                          qp_mgr_m_axi_wready,
    output wire                          qp_mgr_m_axi_awlock,
    // Write Response channel
    input  wire [                 0 : 0] qp_mgr_m_axi_bid,
    input  wire [                   1:0] qp_mgr_m_axi_bresp,
    input  wire                          qp_mgr_m_axi_bvalid,
    output wire                          qp_mgr_m_axi_bready,
    // Read address channel
    output wire [                   0:0] qp_mgr_m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_m_axi_araddr,
    output wire [                   7:0] qp_mgr_m_axi_arlen,
    output wire [                   2:0] qp_mgr_m_axi_arsize,
    output wire [                   1:0] qp_mgr_m_axi_arburst,
    output wire [                   3:0] qp_mgr_m_axi_arcache,
    output wire [                   2:0] qp_mgr_m_axi_arprot,
    output wire                          qp_mgr_m_axi_arvalid,
    input  wire                          qp_mgr_m_axi_arready,
    // Read data/response channel
    input  wire [                   0:0] qp_mgr_m_axi_rid,
    input  wire [                 511:0] qp_mgr_m_axi_rdata,
    input  wire [                   1:0] qp_mgr_m_axi_rresp,
    input  wire                          qp_mgr_m_axi_rlast,
    input  wire                          qp_mgr_m_axi_rvalid,
    output wire                          qp_mgr_m_axi_rready,
    output wire                          qp_mgr_m_axi_arlock,

    // Hardware handshake ports for NVMof (only enabled if C_EN_NVMOF_HW_HNDSHK is set to 1): NVMof 的硬件握手端口（仅当 C_EN_NVMOF_HW_HNDSHK 设置为 1 时才启用）
    // Direct handshake with RDMAif module: 与 RDMAif 模块直接握手
    // ---------------------------------------------------------------------------------------------------------------------------------------------------
    // 用于 CQ 门铃的硬件握手端口
    output wire        resp_hndler_o_send_cq_db_cnt_valid,
    //output wire [C_M_AXI_ADDR_WIDTH -1:0]      resp_hndler_o_send_cq_db_addr,
    output wire [12:0] resp_hndler_o_send_cq_db_addr,
    output wire [31:0] resp_hndler_o_send_cq_db_cnt,
    input  wire        resp_hndler_i_send_cq_db_rdy,

    //-------------------------------  
    // 用于 RQ CI 门铃的硬件握手端口
    input  wire [15:0] i_qp_rq_cidb_hndshk,
    input  wire [31:0] i_qp_rq_cidb_wr_addr_hndshk,   // 要保证i_qp_rq_cidb_wr_addr_hndshk[8+:C_QP_INDX_WIDTH]为qp索引
    input  wire        i_qp_rq_cidb_wr_valid_hndshk,
    output wire        o_qp_rq_cidb_wr_rdy,
    //-------------------------------  

    // 用于 SQ PI 门铃的硬件握手端口
    input  wire [15:0] i_qp_sq_pidb_hndshk,
    input  wire [31:0] i_qp_sq_pidb_wr_addr_hndshk,   // 
    input  wire        i_qp_sq_pidb_wr_valid_hndshk,
    output wire        o_qp_sq_pidb_wr_rdy,


    // 对接rx_pkt_hndler模块接口
    // 用于 RQ PI 门铃的硬件握手端口
    // tip: 在正常接收到新的 RDMA SEND 数据包时RNIC IP 会更新 rx_pkt_hndler_o_rq_db_addr(rq_wrptr_db_addr) 指向的 rx_pkt_hndler_o_rq_db_data(rq_wrptr_db)。
    output wire [31:0] rx_pkt_hndler_o_rq_db_data,
    output wire [12:0] rx_pkt_hndler_o_rq_db_addr,
    output wire        rx_pkt_hndler_o_rq_db_data_valid,
    input  wire        rx_pkt_hndler_i_rq_db_rdy,
    // ---------------------------------------------------------------------------------------------------------------------------------------------------

    output reg rnic_intr,  // RNIC 中断

    // Prioroty flow control
    // 优先流量控制
    input  wire [8:0] stat_rx_pause_req,
    output wire [8:0] ctl_tx_pause_req,
    output wire       ctl_tx_resend_pause,

    // IETH or IMMDT axi streaming interface  
    // IETH 或 IMMDT axi 流媒体接口 
    output wire        ieth_immdt_axis_tvalid,
    output wire        ieth_immdt_axis_tlast,
    output wire [63:0] ieth_immdt_axis_tdata,
    input  wire        ieth_immdt_axis_trdy,

    // Debug counter enablement signals
    // 调试计数器启用信号
    output wire o_global_dbg_cnt_en,
    output wire o_global_dbg_cnt_clr
);
    `include "../../rdma_macros.vh"


    localparam C_QP_INDX_WIDTH = clog2(C_NUM_QP);  // C_NUM_QP = 32, C_QP_INDX_WIDTH = 5
    localparam C_PD_INDX_WIDTH = clog2(C_NUM_PD);  // C_NUM_PD = 2048, C_PD_INDX_WIDTH = 11
    localparam OS_Q_DEPTH = 16;
    localparam OS_Q_INDX_WIDTH = 4;
    localparam NUM_CYCLES_4US = 1024;  // Number of clock cycles that make 4 us (for 125Mhz clk period it is 512)：4 us 的时钟周期数（125Mhz 时钟周期为 512 个时钟周期）
    ////////////////////////////////- Constants ////////////////////////////////////

    // xlpp eval_constant ernic@2022.10 begin
    parameter C_IS_EVAL = 1;  // Hardware time out circuit：硬件超时电路
    // xlpp eval_constant end
    wire                                 qp_mgr_o_xrnic_en;  // XRNIC Configuration -- [0] ERNIC Enable
    wire                                 qp_mgr_o_xrnic_adv_conf_errbuf_overwr_en;
    wire                                 qp_mgr_o_xrnic_adv_conf_retry_cnt_fatal_dis;
    wire [                          3:0] qp_mgr_o_xrnic_adv_base_cnt;  // For timeout count for 4us consideration 
    wire                                 qp_mgr_o_err_buf_en;  // Enable validaion error packets to go to error buffer 
    wire [                          1:0] qp_mgr_o_tx_ack_gen;  // TX side ack generation setting 
    wire                                 qp_mgr_o_depkt_bypass_en;  // XRNIC interface bypass feature enable
    wire [          C_QP_INDX_WIDTH-1:0] qp_mgr_o_num_qp_en;  // Number of QPs enabled
    wire [                         47:0] qp_mgr_o_mac_src_addr;  // Ethernet MAC destination (own) address from XRNIC registers
    wire [                         31:0] qp_mgr_o_ipv4_src_addr;  // IPv4 destination (own) address from XRNIC registers
    wire [                        127:0] qp_mgr_o_ipv6_src_addr;  // IPv6 destination (own) address form XRNIC registers

    /*
     * The evaluation reset used to be a single combinational net feeding more
     * than sixteen thousand reset/enable pins across the RDMA core.  In the
     * integrated design that net, rather than a payload datapath, was the
     * worst 250 MHz setup path.  Register separate, logically identical reset
     * branches so placement can keep each driver with its consumer hierarchy.
     * MAX_FANOUT also permits synthesis to replicate a branch inside a large
     * hierarchy.  The extra cycle affects reset entry/exit only; no traffic
     * valid/ready path passes through these registers.
     */
    wire s_axi_lite_aresetn_sync1;
    wire s_axi_lite_aresetn_1;
    wire s_axi_lite_aresetn_eval;

    (* KEEP = "TRUE", MAX_FANOUT = 256 *) reg core_rstn_local_reg;
    (* KEEP = "TRUE", MAX_FANOUT = 256 *) reg core_rstn_rx_reg;
    (* KEEP = "TRUE", MAX_FANOUT = 256 *) reg core_rstn_resp_reg;
    (* KEEP = "TRUE", MAX_FANOUT = 256 *) reg core_rstn_qp_reg;
    (* KEEP = "TRUE", MAX_FANOUT = 256 *) reg core_rstn_wqe_reg;

    always @(posedge m_axi_aclk) begin : core_reset_branch_pipeline
        if (!m_axi_aresetn) begin
            core_rstn_local_reg <= 1'b0;
            core_rstn_rx_reg    <= 1'b0;
            core_rstn_resp_reg  <= 1'b0;
            core_rstn_qp_reg    <= 1'b0;
            core_rstn_wqe_reg   <= 1'b0;
        end else begin
            core_rstn_local_reg <= ~s_axi_lite_aresetn_sync1;
            core_rstn_rx_reg    <= ~s_axi_lite_aresetn_sync1;
            core_rstn_resp_reg  <= ~s_axi_lite_aresetn_sync1;
            core_rstn_qp_reg    <= ~s_axi_lite_aresetn_sync1;
            core_rstn_wqe_reg   <= ~s_axi_lite_aresetn_sync1;
        end
    end

    /* Local copies for the two distant consumers.  The configuration value is
     * static while traffic is enabled; registering it locally removes a
     * 128-bit cross-region route without changing datapath throughput. */
    (* keep = "true" *) logic [127:0] ipv6_src_addr_rx_reg;
    (* keep = "true" *) logic [127:0] ipv6_src_addr_wqe_reg;

    always_ff @(posedge m_axi_aclk) begin
        if (!core_rstn_local_reg) begin
            ipv6_src_addr_rx_reg  <= 128'd0;
            ipv6_src_addr_wqe_reg <= 128'd0;
        end else begin
            ipv6_src_addr_rx_reg  <= qp_mgr_o_ipv6_src_addr;
            ipv6_src_addr_wqe_reg <= qp_mgr_o_ipv6_src_addr;
        end
    end
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_data_buf_ba;  // Write retry buffer base address
    wire [                         15:0] qp_mgr_o_data_buf_sz_num_bufs;  // number of write retry buffers allocated
    wire [                         15:0] qp_mgr_o_data_buf_sz_buf_sz;  // each write retry buffer size

    wire                                 rx_pkt_hndler_o_ib_pkt_hdr_valdn_err_intr;  // Inbound packet header validation error interrupt
    wire                                 rx_pkt_hndler_o_mad_pkt_rcvd_intr;  // MAD packet received interrupt
    wire                                 rx_pkt_hndler_o_bypass_pkt_rcvd_intr;  // Incoming bypass packet received interrupt
    wire                                 rx_pkt_hndler_o_rnr_nack_gen_intr;
    wire                                 rx_pkt_hndler_o_ob_rsp_rnr_nak_gen_intr;  // RNR-NAK generated interrupt (due receive buffer overflow)
    wire                                 rx_pkt_hndler_o_ob_req_nak_intr;  // NAK/no response for outbound request interrupt
    wire                                 qp_mgr_o_intr_clr_pkt_valdn_err;
    wire                                 qp_mgr_o_intr_clr_mad_pkt_rcvd;
    wire                                 qp_mgr_o_intr_clr_bypass_pkt_rcvd;
    wire                                 qp_mgr_o_intr_clr_rnr_nak_gen;
    wire                                 qp_mgr_o_intr_clr_nack_in;
    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_rq_full_idx;
    wire                                 rx_pkt_hndler_o_rq_full_wr_en;
    wire                                 rx_pkt_hndler_o_rq_full_val;
    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_rq_full_reset_idx;
    wire                                 rx_pkt_hndler_o_rq_full_reset_wr_en;
    wire                                 rx_pkt_hndler_o_rq_full_reset_val;
    wire [                 C_NUM_QP-1:0] rx_pkt_hndler_o_cq_full;  // Completion Q full for each QP
    wire [                 C_NUM_QP-1:0] rx_pkt_hndler_o_rq_empty;  // Receive Q empty for each QP
    reg  [                 C_NUM_QP-1:0] resp_hndler_o_qp_retried;  // QP retried for each QP
    wire [                         31:0] qp_mgr_o_bypass_buf_ba;
    wire [                         15:0] qp_mgr_o_bypass_buf_sz_num_bufs;
    wire [                         15:0] qp_mgr_o_bypass_buf_sz_buf_sz;
    wire [                         15:0] rx_pkt_hndler_o_bypass_buf_wrptr;
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_err_pkt_buf_ba;
    wire [                         15:0] qp_mgr_o_err_pkt_buf_sz_num_bufs;
    wire [                         15:0] qp_mgr_o_err_pkt_buf_sz_buf_sz;
    wire [                         15:0] rx_pkt_hndler_o_err_pkt_buf_wrptr;
    wire [                         15:0] rx_pkt_hndler_o_ib_send_pkt_cnt;  // Incoming send request packets count
    wire [                         15:0] rx_pkt_hndler_o_ib_ack_pkt_cnt;  // Incoming acknowledgement packets count
    wire [                         15:0] rx_pkt_hndler_o_ib_rres_pkt_cnt;  // Incoming read response packets count
    wire [                         15:0] rx_pkt_hndler_o_ib_qp1_pkt_cnt;  // Incoming QP1 packet count
    wire [                         15:0] rx_pkt_hndler_o_ib_inv_pkt_cnt;  // Incoming invalid packet count
    wire [                         15:0] rx_pkt_hndler_o_ib_dupl_pkt_cnt;  // Incoming duplicate packet count  
    wire [                          7:0] rx_pkt_hndler_o_last_ib_pkt_opcode;  // Last incoming packet OPCODE
    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_last_ib_pkt_dest_qpid;  // Last incoming packet DEST_QP
    wire [                         15:0] rx_pkt_hndler_o_last_ib_pkt_psn;  // Last incoming packet PSN  
    wire [                 C_NUM_QP-1:0] resp_hndler_o_osq_full;  // Outstanding entry Q is full for each QP
    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_addr;
    wire [                         31:0] rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_data;
    wire [                         31:0] rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_en;
    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_query_qpid;  // Index of QP for which information queried
    //wire [31:0]                         qp_mgr_o_tx_hdr_buf_ba;
    wire [                         31:0] qp_mgr_o_error_corrupt_conf;
    wire [                         15:0] qp_mgr_o_error_corrupt_cnt;
    wire                                 qp_mgr_o_error_corrupt_en;
    wire [                         15:0] qp_mgr_o_tx_hdr_buf_sz_num_hdrs;
    wire [                         15:0] qp_mgr_o_tx_hdr_buf_sz_buf_sz;
    wire [                         31:0] qp_mgr_o_tx_sgl_buf_ba;
    wire [                         15:0] qp_mgr_o_tx_sgl_buf_sz_num_sgls;
    wire [                         15:0] qp_mgr_o_tx_sgl_buf_sz_buf_sz;
    wire                                 qp_mgr_o_xrnic_conf_ipver;
    wire [                         15:0] qp_mgr_o_xrnic_udp_src_port;

    wire                                 rx_pkt_hndler_o_qpm_req_qp_conf;
    wire                                 qp_mgr_o_qp_conf_valid_wqe_proc;
    wire                                 qp_mgr_o_qp_conf_valid_rx_pkt;
    wire [                         31:0] qp_mgr_o_qp_conf;  // QP configuration

    wire                                 rx_pkt_hndler_o_qpm_req_rq_buf_ba;
    wire                                 qp_mgr_o_qp_rq_ba_valid;
    wire [       C_M_AXI_ADDR_WIDTH-9:0] qp_mgr_o_qp_rq_ba;

    wire                                 resp_hndler_o_qp_cq_ba_req;
    wire                                 qp_mgr_o_qp_cq_ba_valid;
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_qp_cq_ba;

    wire                                 rx_pkt_hndler_o_qpm_req_rq_wrptr_db_add;
    wire                                 qp_mgr_o_qp_rq_wrptrdb_add_valid;
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_qp_rq_wrptrdb_add;

    wire                                 resp_hndler_o_qp_cq_hdptr_req;
    wire                                 qp_mgr_o_qp_cq_hdptr_valid;
    wire [                         15:0] qp_mgr_o_qp_cq_hdptr;
    wire [                         15:0] resp_hndler_o_qp_cq_hdptr;
    wire                                 resp_hndler_o_qp_cq_hdptr_wrn;
    wire [       C_QP_INDX_WIDTH -1 : 0] resp_hndler_o_qp_cqe_proc_qpid;

    wire                                 rx_pkt_hndler_o_qpm_req_rq_ci_db;
    wire                                 qp_mgr_o_qp_rq_cidb_valid;
    wire [                         31:0] qp_mgr_o_qp_rq_cidb;

    wire                                 rx_pkt_hndler_o_qpm_req_q_depth;
    wire                                 qp_mgr_o_qp_rq_depth_valid;
    wire [                         15:0] qp_mgr_o_qp_rq_depth;

    wire                                 rx_pkt_hndler_o_qpm_req_last_rq_req;
    wire                                 rx_pkt_hndler_o_qpm_rw_last_rq_req;
    wire                                 rx_pkt_hndler_o_qpm_req_qp_pd_num;
    wire [                         31:0] rx_pkt_hndler_o_last_rq_req;
    wire [                         31:0] qp_mgr_o_qp_last_rq;
    wire                                 qp_mgr_o_qp_last_rq_valid;

    wire                                 rx_pkt_hndler_o_qpm_req_pd_num;
    wire                                 rx_pkt_hndler_o_qpm_rw_pd_num;
    wire [                         31:0] rx_pkt_hndler_o_pd_num;
    wire [                         23:0] qp_mgr_o_qp_pd_num;
    wire                                 qp_mgr_o_qp_pd_num_valid;

    wire                                 qp_mgr_o_intr_clr_wqe_cmpl;
    wire                                 qp_mgr_o_intr_clr_fatal_err;
    wire                                 qp_mgr_o_intr_en_wqe_cmpl;
    wire                                 resp_hndler_o_intr_wqe_cmpl;

    wire                                 rx_pkt_hndler_o_qpm_req_dest_qp_conf;
    wire                                 qp_mgr_o_qp_dest_qpid_valid_wqe_proc;
    wire                                 qp_mgr_o_qp_dest_qpid_valid_rx_pkt;
    wire [                         23:0] qp_mgr_o_qp_dest_qpid;

    wire                                 rx_pkt_hndler_o_qpm_req_stat_qp_msn;
    wire                                 rx_pkt_hndler_o_qpm_rw_stat_qp_msn;
    wire [                         23:0] rx_pkt_hndler_o_stat_qp_msn;  // last valid inbound request
    wire [                         23:0] qp_mgr_o_qp_stat_msn;  // Expected MSN for Qp incoming messages write data from hdr validation
    wire                                 qp_mgr_o_qp_stat_msn_valid;

    wire                                 rx_pkt_hndler_o_qpm_req_timeout;
    wire                                 rx_pkt_hndler_o_qpm_rw_timeout;
    wire [                         31:0] rx_pkt_hndler_o_timeout;  // inbound response current RNR-NAK count, timeout value,
    wire                                 qp_mgr_o_qp_timeout_valid;
    wire [                         31:0] qp_mgr_o_qp_timeout_per_qp;

    wire                                 qpm_wqe_fifo_empty;
    wire                                 qpm_wqe_pop;
    wire [                        511:0] qpm_wqe_data;  //64B of WQE 


    wire                                 rx_pkt_hndler_o_qpm_req_mac_dest_addr;
    wire                                 qp_mgr_o_qp_mac_remote_addrl_valid;

    wire                                 rx_pkt_hndler_o_qpm_req_ip_dest_addr;
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
    wire                                 wqe_proc_top_o_reg_rd_en;
    wire                                 wqe_proc_top_o_reg_rd_en_s;
    wire                                 wqe_proc_top_o_reg_wr_en;

    wire                                 qp_mgr_o_qp_adv_conf_valid;
    wire [                         31:0] qp_mgr_o_qp_adv_conf;  // QP advance configuration
    wire                                 qp_mgr_o_qp_sq_psn_valid;
    wire                                 qp_mgr_o_qp_sq_psn_wqe_valid;
    wire [                         23:0] qp_mgr_o_qp_sq_psn;
    wire [                         23:0] wqe_proc_top_reg_sq_psn;
    wire [                         23:0] qp_mgr_o_qp_sq_ssn;
    wire [                         23:0] wqe_proc_top_reg_sq_ssn;
    wire                                 rx_pkt_hndler_o_qpm_req_sq_psn;

    // WQE Processor I/F                                                        
    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_wqe_proc_ob_rsp_qpid;  // 响应数据包的目标 QP ID -- Destination QP ID for which response packet is targetted
    wire [                         23:0] rx_pkt_wqe_proc_ob_rsp_psn;  // PSN field value in response packet BTH Header
    wire [                          1:0] rx_pkt_wqe_proc_ob_rsp_type;  // 2'b00 - ACK, 2'b01 - RNR-NAK, 2'b10 - rsvd, 2'b11 - NAK
    wire [                          4:0] rx_pkt_wqe_proc_ob_rsp_crdt_cnt;  // Credit count used to implement End-to-End flow control
    wire [                          4:0] rx_pkt_wqe_proc_ob_rsp_rnr_nak_tval;  // If resp_type is RNR-NAK, associated time-out vale
    wire [                          4:0] rx_pkt_wqe_proc_ob_rsp_nak_code;  // If resp_type is NAK, associated NAK code, see table.44
    wire [                         23:0] rx_pkt_wqe_proc_ob_rsp_msn;  // MSN field in AETH Header of response packet
    wire                                 rx_pkt_wqe_proc_ob_rsp_req;  // Level to WQE Processor to indicates that new response to schedule 
    wire                                 rx_pkt_wqe_proc_ob_rsp_expl_ack;  // Level to WQE Processor to indicates that new response to schedule 
    wire                                 wqe_proc_o_bres_fifo_full;  // Acknowledgement that new response in schedule queue

    wire [                          7:0] rx_pkt_wqe_proc_aeth_syndrome;

    wire [          C_QP_INDX_WIDTH-1:0] wqe_proc_os_wqe_qpid;  // QP ID of WQE
    wire [                          7:0] wqe_proc_os_wqe_opcode;  // BTH.Opcode of outbound request
    wire [                          4:0] wqe_proc_o_wqe_qp_to;  // BTH.Opcode of outbound request
    wire [                         15:0] wqe_proc_os_wqe_wrid;  // Work request ID of outbound request
    wire [                         14:0] wqe_proc_os_wqe_data_bufid;  // Data buffer ID for RDMA WRITES
    wire                                 wqe_proc_os_wqe_retried;  // Retried work request
    wire                                 wqe_proc_os_wqe_fatal_push;  // Retried work request
    wire [                         23:0] wqe_proc_os_wqe_send_psn;  // PSN of most recent request
    wire [                         23:0] wqe_proc_os_wqe_send_first_psn;  // PSN of most recent request
    wire [                         23:0] wqe_proc_os_wqe_send_msn;  // PSN of most recent request
    wire [       C_M_AXI_ADDR_WIDTH-1:0] wqe_proc_os_wqe_rdma_rd_rcv_ba;  // If request is RDMA READ, receive buffer base address
    wire [                         31:0] wqe_proc_os_wqe_rdma_rd_res_len;  // If request is RDMA READ, response length in bytes
    wire                                 wqe_proc_os_wqe_explicit_ack_req;  // Explicit acknowledgement requirement on outbound requests
    wire                                 wqe_proc_os_wqp_req;  // request from WQE Processor to push WQE
    wire                                 wqe_proc_os_wqp_req_Read;  // request from WQE Processor to push WQE
    wire                                 wqe_proc_os_wqp_rdy;  // response to WQE Processor; 0 - busy; 1 - completed  
    reg  [                C_NUM_QP -1:0] wqe_proc_os_fifo_full;  //FIFO full signal from outstanding fifo
    wire                                 resp_hndler_freeup_data_buf;  // Freeup RDMA WRITE DATA buffer
    wire [                         14:0] resp_hndler_freeup_data_bufid;  // BUffer ID to be freed up
    wire [                         31:0] resp_hndler_o_resp_hndler_sts;
    wire [                         31:0] resp_hndler_o_stat_retry_count;

    wire [         C_QP_INDX_WIDTH -1:0] wqe_proc_top_o_qp_id;  // index to be replaced during WQE proc integration 
    wire [          OS_Q_INDX_WIDTH : 0] os_num_vld_entries;
    wire                                 resp_hndler_o_qp_cq_depth_req;
    wire                                 qp_mgr_o_qp_cq_depth_valid;
    wire [                         15:0] qp_mgr_o_qp_cq_depth;
    wire [         C_QP_INDX_WIDTH -1:0] resp_hndler_o_qp_conf_qp_idx;
    wire [         C_QP_INDX_WIDTH -1:0] resp_hndler_o_qp_sq_psn_resp_idx;
    wire [(4*C_QP_INDX_WIDTH+4) - 1 : 0] o_osq_nacked;
    wire                                 resp_handler_o_qp_sq_ba_req;
    wire                                 qp_mgr_o_qp_sq_ba_valid;
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_qp_sq_ba;

    wire                                 resp_hndler_o_qp_sq_cmpldb_addr_req;
    wire                                 qp_mgr_o_qp_sq_cmpldb_add_valid;
    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_qp_sq_cmpldb_add;

    wire [       C_QP_INDX_WIDTH -1 : 0] resp_hndler_o_retransmit_qpid;  // QP id for this retransmission is required
    wire [                       47 : 0] resp_hndler_o_retransmit_wqe_bram_din;
    wire                                 resp_hndler_o_retransmit_reqd;  // Retransmission is required
    wire                                 qp_mgr_o_retransmit_done;
    wire [                         23:0] resp_hndler_o_psn_to_retry;
    wire [                         23:0] resp_hndler_o_ssn_to_retry;
    reg  [                C_NUM_QP -1:0] resp_hndler_o_osq_almost_full;

    wire [         C_QP_INDX_WIDTH -1:0] qp_idx_TBD;

    wire                                 rx_pkt_hndler_o_qpm_req_rq_buf_ca;
    wire                                 rx_pkt_hndler_o_qpm_rw_rq_buf_ca;
    wire [       C_M_AXI_ADDR_WIDTH-9:0] qp_mgr_o_qp_stat_rq_buf_ca;
    wire                                 qp_mgr_o_qp_stat_rq_buf_ca_valid;
    wire [       C_M_AXI_ADDR_WIDTH-9:0] rx_pkt_hndler_o_rq_buf_ca;

    wire [         C_QP_INDX_WIDTH -1:0] rx_pkt_hndler_o_qp_index_mpsnbuf;

    wire                                 rx_pkt_hndler_o_max_epsn_req;
    wire                                 resp_hndler_o_mpsn_buf_valid;


    wire [                 C_NUM_QP-1:0] resp_hndler_o_mpsnbuf_empty;
    wire                                 rx_pkt_hndler_o_mpsnbuf_pop;

    wire [          C_OSQ_PSN_WIDTH-1:0] resp_hndler_o_max_epsn;
    wire [                         63:0] resp_hndler_o_local_addr;
    wire [                         23:0] resp_hndler_o_trnsfr_length;
    wire [                         15:0] resp_hndler_o_inc_nack_cnt;

    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_qp_index_acknack;
    wire                                 rx_pkt_hndler_o_acknack_valid;
    wire [                          7:0] rx_pkt_hndler_o_acknacked_syndrome;
    wire [                         23:0] rx_pkt_hndler_o_acknacked_msn;
    wire [                          1:0] rx_pkt_hndler_o_acknacked_opcode;
    wire [                         23:0] rx_pkt_hndler_o_acknacked_psn;
    wire [          C_QP_INDX_WIDTH-1:0] resp_hndler_o_ob_rsp_qpid;
    wire [                         23:0] resp_hndler_o_ob_rsp_psn;
    wire [                          4:0] resp_hndler_o_ob_rsp_aeth_syndrome;
    wire [                         23:0] resp_hndler_o_ob_rsp_aeth_msn;
    wire                                 resp_hndler_o_ob_rsp_vld;

    wire                                 qp_mgr_o_intr_en_pkt_valdn_err;
    wire                                 qp_mgr_o_intr_en_mad_pkt_rcvd;
    wire                                 qp_mgr_o_intr_en_bypass_pkt_rcvd;
    wire                                 qp_mgr_o_intr_clr_qp_pkt_rcvd;
    wire                                 qp_mgr_o_intr_en_rnr_nak_gen;

    wire                                 rx_pkt_hndler_o_qpm_req_stat_nak;
    wire                                 rx_pkt_hndler_o_qpm_rw_stat_nak;
    wire [                          9:0] rx_pkt_hndler_o_stat_nak;
    wire                                 resp_hndler_o_qp_stat_nak_valid;
    wire [                          9:0] resp_hndler_o_qp_stat_nak;

    wire [                          2:0] resp_handler_o_qp_retry_cnt_new;
    wire [                          2:0] resp_handler_i_qp_retry_cnt_curr;
    wire [          C_QP_INDX_WIDTH-1:0] resp_handler_o_qp_retry_cnt_qpid;
    wire                                 resp_handler_o_qp_retry_cnt_req;
    wire                                 resp_handler_o_qp_retry_cnt_wen;
    wire                                 resp_handler_o_qp_retry_cnt_rdy;

    wire                                 resp_handler_o_qp_retry_fatal_req;
    wire [          C_QP_INDX_WIDTH-1:0] resp_handler_o_qp_retry_fatal_qpid;
    wire                                 resp_handler_i_qp_retry_fatal_rdy;

    wire                                 qp_mgr_o_halt;
    wire [       C_QP_INDX_WIDTH -1 : 0] qp_mgr_o_halted_qpid;
    wire                                 wqe_proc_i_wqe_halted;

    wire                                 rx_pkt_hndler_o_qpm_req_stat_resp_psn;
    wire                                 rx_pkt_hndler_o_qpm_rw_stat_resp_psn;
    wire [                         23:0] qp_mgr_o_qp_stat_resp_psn;
    wire [                         23:0] rx_pkt_hndler_o_qp_stat_resp_psn;
    wire                                 qp_mgr_o_qp_stat_resp_psn_valid;
    wire                                 rx_pkt_hndler_o_qp_fatal_wr_en;
    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_qp_fatal_wr_idx;
    wire                                 rx_pkt_hndler_o_qp_fatal_wr_data;
    wire                                 rx_pkt_hndler_o_qp_pkt_rcvd_intr;
    wire                                 clr_qp_fatal_err;
    wire [          C_QP_INDX_WIDTH-1:0] clr_qp_fatal_idx;
    wire                                 qp_under_recovery_wr_en;
    wire [          C_QP_INDX_WIDTH-1:0] qp_under_recovery_wr_idx;
    wire                                 qp_under_recovery_wr_data;

    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_in_errsts_q_ba;
    wire [                         15:0] qp_mgr_in_errsts_q_sz;
    wire [                         15:0] rx_pkt_hndler_in_errsts_q_wrptr;

    wire [       C_M_AXI_ADDR_WIDTH-1:0] out_errsts_q_ba;
    wire [                         15:0] out_errsts_q_sz;
    wire [                         15:0] out_errsts_q_wrptr;
    wire                                 intr_en_ill_opc_in_sq;
    wire                                 intr_clr_ill_opc_in_sq;
    wire                                 ill_opc_in_sq_intr;
    wire [                         15:0] wqe_out_rdwr_pkt_cnt;
    wire [                         15:0] wqe_out_send_pkt_cnt;
    wire [                         15:0] wqe_out_ack_pkt_cnt;
    wire [                         15:0] wqe_out_mad_pkt_cnt;
    wire [                         15:0] wqe_out_rd_resp_pkt_cnt;

    wire                                 rx_pkt_hndler_o_fatal_err_intr;
    wire                                 qp_mgr_o_intr_en_fatal_err;

    wire                                 resp_hndler_o_qp_conf_req;
    wire                                 qp_mgr_o_qp_conf_valid_resp_hndl;

    wire [                         31:0] qp_mgr_o_rx_pkt_hndl_dbg_ctrl;
    wire [                         31:0] rx_pkt_hndler_o_debug_bus;
    wire [                         31:0] wqe_proc_status_out;
    wire [                         31:0] last_out_pkt_info;

    reg  [                C_NUM_QP -1:0] resp_hndlr_i_qp_wq_cmpl_intr_sts;
    wire                                 intr_wqe_cmpl_sts_or_reduce;
    wire [                       31 : 0] intr_wqe_cmpl_sts_wr_en;
    wire [                       31 : 0] intr_wqe_cmpl_sts_wr_data;
    wire [        C_QP_INDX_WIDTH-1 : 0] intr_wqe_cmpl_sts_wr_addr;
    wire [                       31 : 0] intr_wqe_cmpl_sts_wr_en_pending;
    wire [        C_QP_INDX_WIDTH-1 : 0] intr_wqe_cmpl_sts_wr_addr_pending;
    wire [                         31:0] clr_qp_pkt_rcvd_intr;
    wire [          C_QP_INDX_WIDTH-1:0] clr_qp_pkt_rcvd_intr_wr_addr;
    wire [                          1:0] qp_mgr_o_flow_credits;

    wire [                         31:0] rx_pkt_hndler_o_pkt_all_cnt;
    wire [                         31:0] rx_pkt_hndler_o_pkt_disc_cnt;

    wire [                         15:0] rx_pkt_hndler_o_inc_nack_cnt;
    wire [                         15:0] rx_pkt_hndler_o_out_nack_cnt;

    // wire [31:0]                         rx_pkt_hndler_o_rq_db_data;      
    wire [          C_QP_INDX_WIDTH+1:0] rx_pkt_hndler_o_rq_db_addr_temp;
    // wire                                rx_pkt_hndler_o_rq_db_data_valid;

    wire                                 rx_pkt_hndler_o_qpm_req_rq_wrptr_db;
    wire                                 rx_pkt_hndler_o_qpm_rw_rq_wrptr_db;
    wire [                         15:0] rx_pkt_hndler_o_rq_wrptr_db;
    wire [                         15:0] qp_mgr_o_rq_pi_db;
    wire                                 qp_mgr_o_qp_rq_pi_db_valid;

    //wire [15:0] qp_mgr_o_connect_io_residual_rq;       
    wire [                         15:0] qp_mgr_o_rq_pi_db_hw_hndshk;
    wire [                         12:0] qp_mgr_o_connect_io_qp_rq_pi_db_wptr;
    wire [          C_QP_INDX_WIDTH-1:0] qp_mgr_o_connect_io_qpid;
    wire                                 qp_mgr_o_hw_hndshk_disable_to_0;

    wire [                         23:0] qp_mgr_o_qp_stat_ret_sq_psn;
    wire                                 rx_pkt_hndler_o_qpm_req_qp_stat_ret_sq_psn;
    wire                                 qp_mgr_o_qp_stat_ret_sq_psn_valid;


    wire [                         31:0] rx_pkt_hndler_o_ipg_cntr_min;
    wire [                         31:0] rx_pkt_hndler_o_ipg_0_5_cntr;
    wire [                         31:0] rx_pkt_hndler_o_ipg_6_10_cntr;
    wire [                         31:0] rx_pkt_hndler_o_ipg_11_15_cntr;
    wire [                         31:0] rx_pkt_hndler_o_ipg_16_20_cntr;

    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_qp_req_conf_idx;
    wire [          C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_qp_rsp_conf_idx;
    wire [          C_PD_INDX_WIDTH-1:0] rx_pkt_hndler_o_pd_req_conf_idx;

    wire [                         31:0] qp_mgr_o_qp_conf_replica;
    wire                                 qp_mgr_o_qp_conf_replica_valid;
    wire                                 rx_pkt_hndler_o_qp_req_conf_en;
    wire                                 rx_pkt_hndler_o_pd_req_conf_en;

    wire                                 rx_pkt_hndler_o_qp_rsp_conf_en;

    wire                                 rx_pkt_hndler_req_remote_qp_mac_addr_en;
    wire                                 rx_pkt_hndler_req_remote_qp_ip_addr_en;

    wire                                 rx_pkt_hndler_rsp_remote_qp_mac_addr_en;
    wire                                 rx_pkt_hndler_rsp_remote_qp_ip_addr_en;

    wire [                         31:0] qp_mgr_o_qp_mac_remote_addrl_replica;
    wire [                         31:0] qp_mgr_o_qp_mac_remote_addrm_replica;


    wire [                         31:0] qp_mgr_o_qp_ip_remote_addr1_replica;
    wire [                         31:0] qp_mgr_o_qp_ip_remote_addr2_replica;
    wire [                         31:0] qp_mgr_o_qp_ip_remote_addr3_replica;
    wire [                         31:0] qp_mgr_o_qp_ip_remote_addr4_replica;

    wire [       C_M_AXI_ADDR_WIDTH-1:0] qp_mgr_o_resp_err_pkt_buf_ba;
    wire [                         15:0] qp_mgr_o_resp_err_pkt_buf_sz_num_bufs;
    wire [                         15:0] qp_mgr_o_resp_err_pkt_buf_sz_buf_sz;
    wire [                         15:0] rx_pkt_hndler_o_rsp_err_buf_wrptr;

    wire [                         31:0] rx_pkt_hndler_o_qp_timeout_per_qp;

    wire                                 rx_pkt_hndler_o_connect_io_qp_rq_pi_db_rdy;

    //Debug counter signals
    wire [                         15:0] wqe_proc_idle_cnt;
    wire [                         15:0] wqe_proc_rd_wqe_cnt;
    wire [                         15:0] wqe_proc_rd_q_info_cnt;
    wire [                         15:0] wqe_proc_wait0_cnt;
    wire [                         15:0] wqe_proc_ip_chksum_cnt;
    wire [                         15:0] wqe_proc_hdr_gen_cnt;
    wire [                         15:0] wqe_proc_hdr_sto_cnt;
    wire [                         15:0] wqe_proc_hdr_sgl_buf_full_cnt;
    wire [                         15:0] wqe_proc_wr_retry_buf_full_cnt;

    wire                                 wqe_proc_dma_in_idle;

    wire [      C_M_AXI_ADDR_WIDTH -1:0] resp_hndler_o_send_cq_db_addr_int;
    assign resp_hndler_o_send_cq_db_addr = resp_hndler_o_send_cq_db_addr_int[12:0];

    wire                               rx_pkt_hndler_o_rd_rsp_wr_cmpltd;
    wire [        C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_o_rsp_qp_id;
    wire [     C_M_AXI_ADDR_WIDTH-1:0] rx_pkt_hndler_o_rsp_dest_addr;
    wire                               rx_pkt_hndler_o_qp_fatal_timer_idle;
    // CNP related signals
    wire                               rx_pkt_hndlr_i_cnp_sched_intr_en;
    wire                               rx_pkt_hndlr_o_cnp_sched_intr;
    wire                               rx_pkt_hndlr_o_cnp_sched_intr_sts;
    wire                               rx_pkt_hndlr_i_clr_cnp_sched_intr;

    //wire  [C_NUM_QP-1:0]                 rx_pkt_hndlr_o_cnp_sched_reg;
    //reg  [C_NUM_QP-1:0]                 rx_pkt_hndlr_o_cnp_sched_reg;
    wire [        C_QP_INDX_WIDTH-1:0] rx_pkt_hndlr_o_cnp_sched_wr_addr;
    wire [                       31:0] rx_pkt_hndlr_o_cnp_sched_wr_data;
    wire [                       31:0] rx_pkt_hndlr_o_cnp_sched_wr_en;
    //wire [C_NUM_QP-1:0]                 rx_pkt_hndlr_i_clr_cnp_sched;
    //reg  [C_NUM_QP-1:0]                 rx_pkt_hndlr_i_clr_cnp_sched_r;
    wire [                       31:0] clr_cnp_sched;
    wire [        C_QP_INDX_WIDTH-1:0] clr_cnp_sched_wr_addr;

    wire [                        3:0] rx_pkt_hndlr_i_cnp_timeout_reg_1;
    wire [                        3:0] rx_pkt_hndlr_i_cnp_timeout_reg_2;
    wire [                       15:0] rx_pkt_hndlr_o_cnp_rx_pkt_cnt;

    wire [        C_QP_INDX_WIDTH-1:0] wqe_fatal_qpid;
    wire                               wqe_fatal_qp_vld;
    wire                               wqe_fatal_qp_rdy;

    wire [                       23:0] pd_pd_num;
    wire [                       31:0] pd_virt_addr_0;
    wire [                       31:0] pd_virt_addr_1;
    wire [                       31:0] pd_buf_base_addr_0;
    wire [                       31:0] pd_buf_base_addr_1;
    wire [                        7:0] pd_buf_r_key;
    wire [                       47:0] pd_w_r_buf_len;
    wire [                        3:0] pd_access_type;

    wire [                        1:0] req_indicator;
    wire [                       63:0] physical_addr;
    wire [                       31:0] rd_dma_len;
    wire [                       12:0] rd_req_pmtu;
    wire [                       23:0] rsp_last_psn;
    //wire [C_NUM_QP-1 : 0]               rd_req_fifo_full;
    //reg  [C_NUM_QP-1 : 0]               rd_req_fifo_full;


    wire [                       31:0] o_all_rd_req_cnt;
    wire [                       31:0] o_good_rd_req_cnt;
    wire [                       31:0] o_dup_rd_req_cnt;

    wire                               rx_pkt_hndler_waiting_for_rq_update;
    wire [        C_QP_INDX_WIDTH-1:0] rx_pkt_hndler_waiting_for_rq_update_on_qp_num;

    wire [C_S_AXI_LITE_ADDR_WIDTH-1:0] int_w_axi_lite_awaddr;
    wire                               int_w_axi_lite_awready;
    wire                               int_w_axi_lite_awvalid;

    wire [C_S_AXI_LITE_ADDR_WIDTH-1:0] int_w_axi_lite_araddr;
    wire                               int_w_axi_lite_arready;
    wire                               int_w_axi_lite_arvalid;

    wire [                       31:0] int_w_axi_lite_wdata;
    wire [                        3:0] int_w_axi_lite_wstrb;
    wire                               int_w_axi_lite_wready;
    wire                               int_w_axi_lite_wvalid;

    wire [                       31:0] int_w_axi_lite_rdata;
    wire [                       15:0] o_xrnic_pause_on;
    wire [                        8:0] o_xrnic_pause_priority;
    wire                               o_xrnic_pause_en;
    wire [                       15:0] o_xrnic_pause_off;
    wire [                        1:0] int_w_axi_lite_rresp;
    wire                               int_w_axi_lite_rready;
    wire                               int_w_axi_lite_rvalid;
    wire [                        1:0] int_w_axi_lite_bresp;
    wire                               int_w_axi_lite_bready;
    wire                               int_w_axi_lite_bvalid;

    wire                               xrnic_intr0;
    wire                               xrnic_intr1;
    wire                               xrnic_intr2;
    wire                               xrnic_intr3;
    wire                               xrnic_intr4;
    wire                               xrnic_intr5;
    wire                               xrnic_intr6;
    wire                               xrnic_intr7;
    wire                               xrnic_intr8;
    wire                               resp_handler_osq_pop;
    wire [      C_QP_INDX_WIDTH - 1:0] resp_handler_osq_pop_qpid;
    wire                               wqe_dropped;
    wire [      C_QP_INDX_WIDTH - 1:0] wqe_dropped_qpid;

    wire                               tx_pause_req_on;
    wire [                      511:0] wqe_proc_top_m_axis_tdata;
    wire                               wqe_proc_top_m_axis_tvalid;
    wire                               wqe_proc_top_m_axis_tlast;
    wire                               wqe_proc_top_m_axis_tready;
    wire [                       63:0] wqe_proc_top_m_axis_tkeep;

    wire                               fifo_empty_pause_off;
    wire                               fifo_prog_full;
    reg                                drop_non_roce_pkt;
    reg                                tlast_strech;

    wire [                        8:0] rx_pause_req;
    wire [                        8:0] tx_pause_req;

    //  wire [8:0]                           ctl_tx_pause_req_out;
    wire [                        3:0] roce_priority;
    wire [                        3:0] non_roce_priority;
    wire [                       15:0] roce_pause_on_offset;
    wire [                       15:0] roce_pause_off_offset;
    wire [                       15:0] non_roce_pause_on_offset;
    wire [                       15:0] non_roce_pause_off_offset;
    wire [                        8:0] fifo_data_count;
    wire                               non_roce_rx_fifo_full;

    wire [                      511:0] cmac_m_axis_tdata_out;
    wire [                       63:0] cmac_m_axis_tkeep_out;
    wire                               cmac_m_axis_tlast_out;
    wire                               cmac_m_axis_tready_out;
    wire                               cmac_m_axis_tvalid_out;
    wire                               cmac_m_axis_tuser_out;

    wire [                      511:0] non_roce_cmac_s_axis_tdata_d;
    wire [                      511:0] non_roce_cmac_s_axis_tdata_2d;
    wire [                      511:0] non_roce_cmac_s_axis_tdata_3d;
    wire [                       63:0] non_roce_cmac_s_axis_tkeep_d;
    wire [                       63:0] non_roce_cmac_s_axis_tkeep_2d;
    wire [                       63:0] non_roce_cmac_s_axis_tkeep_3d;
    wire                               non_roce_cmac_s_axis_tlast_d;
    wire                               non_roce_cmac_s_axis_tlast_2d;
    wire                               non_roce_cmac_s_axis_tlast_3d;
    wire                               non_roce_cmac_s_axis_tvalid_d;
    wire                               non_roce_cmac_s_axis_tvalid_2d;
    wire                               non_roce_cmac_s_axis_tvalid_3d;
    wire                               non_roce_cmac_s_axis_tuser_d;
    wire                               non_roce_cmac_s_axis_tuser_2d;
    wire                               non_roce_cmac_s_axis_tuser_3d;
    wire                               roce_reg_tvalid_out;
    wire [                      511:0] roce_reg_tdata_out;
    wire [                       63:0] roce_reg_tkeep_out;
    wire                               roce_reg_tlast_out;
    wire [                        0:0] roce_reg_tuser_out;


    wire [                      577:0] roce_reg_out;
    wire [                      577:0] non_roce_cmac_reg;
    wire [                      577:0] non_roce_cmac_reg_d;
    wire [                      577:0] non_roce_cmac_reg_2d;


    wire [                      576:0] cmac_reg_out;

    localparam [0:0] IDLE = 1'b0;
    localparam [0:0] PKT_BNDRY_START = 1'b1;
    reg                                   state;
    reg                                   pkt_boundry;
    wire                                  sop;
    reg                                   pkt_boundry_d;

    reg                                   init_start;  // 1 - 初始化状态; 0 - 非初始化状态
    reg     [      C_QP_INDX_WIDTH-1 : 0] init_count;  // 初始化计数器
    reg                                   core_rst_n_r;
    wire                                  lut_ram_reset_busy;
    wire                                  rd_req_fifo_full_wr_en_0;
    wire    [      C_QP_INDX_WIDTH-1 : 0] rd_req_fifo_full_wr_addr_0;
    wire                                  rd_req_fifo_full_wr_en_1;
    wire    [      C_QP_INDX_WIDTH-1 : 0] rd_req_fifo_full_wr_addr_1;
    wire                                  osq_nacked_resp_wr_en;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_nacked_resp_wr_addr;
    wire                                  osq_nacked_resp_wr_data;
    wire                                  osq_nacked_resp_wr_en_aux;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_nacked_resp_wr_addr_aux;
    wire                                  osq_nacked_resp_wr_data_aux;
    wire                                  osq_nacked_resp_wr_en_pending;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_nacked_resp_wr_addr_pending;
    wire                                  osq_nacked_resp_wr_data_pending;
    wire                                  mpsnbuf_empty_set_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] mpsnbuf_empty_set_id;
    wire                                  mpsnbuf_empty_reset_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] mpsnbuf_empty_reset_id;
    reg     [             C_NUM_QP-1 : 0] resp_hndler_o_mpsnbuf_empty_0;
    reg     [             C_NUM_QP-1 : 0] resp_hndler_o_mpsnbuf_empty_ff;
    reg     [             C_NUM_QP-1 : 0] resp_hndler_o_mpsnbuf_empty_2ff;
    wire                                  osq_full_set_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_full_set_id;
    wire                                  osq_full_reset_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_full_reset_id;
    wire                                  osq_timeout_set_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_timeout_set_id;
    wire                                  osq_empty_set_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_empty_set_id;
    wire                                  osq_empty_reset_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_empty_reset_id;
    wire                                  osq_almost_full_set_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_almost_full_set_id;
    wire                                  osq_almost_full_reset_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] osq_almost_full_reset_id;
    wire    [(2*C_QP_INDX_WIDTH+4)-1 : 0] osq_rnr_nacked;
    wire                                  qp_retried_set_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] qp_retried_set_id;
    wire                                  qp_retried_reset_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] qp_retried_reset_id;
    wire    [                       31:0] clr_wqe_cmpl_sts_set_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] clr_wqe_cmpl_sts_set_id;
    wire                                  qp_disable_pulse_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] qp_disable_pulse_id;
    wire                                  qp_disable_pulse_data;
    wire                                  i_tx_path_congestion_valid;
    wire    [      C_QP_INDX_WIDTH-1 : 0] i_tx_path_congestion_id;
    wire                                  i_tx_path_congestion_data;
    wire                                  qp_enable_wr_en;
    wire    [      C_QP_INDX_WIDTH-1 : 0] qp_enable_wr_addr;
    wire                                  qp_enable_wr_data;
    //reg      [C_NUM_QP-1 : 0]                   o_osq_nacked_sim;
    integer                               i;
    integer                               k;


    // assign rx_pkt_hndler_o_rq_db_addr = C_QP_INDX_WIDTH == 8 ? rx_pkt_hndler_o_rq_db_addr_temp : {{(8-C_QP_INDX_WIDTH){1'b0}},rx_pkt_hndler_o_rq_db_addr_temp};

    //  assign xrnic_intr = {24'b0 ,rx_pkt_hndler_o_fatal_err_intr, rx_pkt_hndler_o_qp_pkt_rcvd_intr,ill_opc_in_sq_intr, resp_hndler_o_intr_wqe_cmpl, 1'b0, rx_pkt_hndler_o_bypass_pkt_rcvd_intr, rx_pkt_hndler_o_mad_pkt_rcvd_intr, rx_pkt_hndler_o_ib_pkt_hdr_valdn_err_intr};

    assign xrnic_intr0                = rx_pkt_hndler_o_ib_pkt_hdr_valdn_err_intr;
    assign xrnic_intr1                = rx_pkt_hndler_o_mad_pkt_rcvd_intr;
    assign xrnic_intr2                = rx_pkt_hndler_o_bypass_pkt_rcvd_intr;
    assign xrnic_intr3                = rx_pkt_hndler_o_rnr_nack_gen_intr;
    assign xrnic_intr4                = resp_hndler_o_intr_wqe_cmpl;
    assign xrnic_intr5                = 1'b0;  //ill_opc_in_sq_intr;              
    assign xrnic_intr6                = rx_pkt_hndler_o_qp_pkt_rcvd_intr;
    assign xrnic_intr7                = rx_pkt_hndler_o_fatal_err_intr;
    assign xrnic_intr8                = rx_pkt_hndlr_o_cnp_sched_intr;

    assign qp_mgr_o_error_corrupt_cnt = qp_mgr_o_error_corrupt_conf[31:16];
    assign qp_mgr_o_error_corrupt_en  = qp_mgr_o_error_corrupt_conf[0];
    reg  [ 15:0] num_op_pkts;
    wire [511:0] wqe_proc_top_m_axis_tdata_int;
    wire         wqe_proc_top_m_axis_tvalid_d;
    wire         wqe_proc_top_m_axis_tready_d;
    reg          wqe_proc_maxis_delay;
    reg  [ 15:0] delay_cnt;
    reg          delay_cs;

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            delay_cs             <= 1'b0;
            wqe_proc_maxis_delay <= 1'b0;
            delay_cnt            <= 16'h0000;
        end else begin
            case (delay_cs)
                1'b0: begin
                    if (wqe_proc_top_m_axis_tvalid_d && wqe_proc_top_m_axis_tlast && wqe_proc_top_m_axis_tready_d && (|qp_mgr_o_tx_hdr_buf_sz_num_hdrs)) begin
                        delay_cs             <= 1'b1;
                        wqe_proc_maxis_delay <= 1'b1;
                    end
                end
                1'b1: begin
                    if (delay_cnt == qp_mgr_o_tx_hdr_buf_sz_num_hdrs) begin
                        delay_cs             <= 1'b0;
                        wqe_proc_maxis_delay <= 1'b0;
                        delay_cnt            <= 16'h0000;
                    end else begin
                        delay_cnt <= delay_cnt + 16'h0001;
                    end
                end
            endcase
        end
    end

    assign wqe_proc_top_m_axis_tvalid   = wqe_proc_top_m_axis_tvalid_d && ~wqe_proc_maxis_delay;
    assign wqe_proc_top_m_axis_tready_d = wqe_proc_top_m_axis_tready && ~wqe_proc_maxis_delay;

    reg tfirst;
    reg clr_error_corrupt_en;

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            num_op_pkts <= 16'h0000;
            tfirst      <= 1'b0;
        end else begin
            if (qp_mgr_o_error_corrupt_en) begin
                if (wqe_proc_top_m_axis_tvalid && wqe_proc_top_m_axis_tlast && wqe_proc_top_m_axis_tready) begin
                    num_op_pkts <= (num_op_pkts == qp_mgr_o_error_corrupt_cnt + 16'h0003) ? num_op_pkts : num_op_pkts + 8'h01;
                    tfirst      <= 1'b1;
                end else if (wqe_proc_top_m_axis_tvalid && wqe_proc_top_m_axis_tready) begin
                    tfirst <= 1'b0;
                end
            end else begin
                num_op_pkts <= 16'h0000;
                tfirst      <= 1'b0;
            end

            if (num_op_pkts == qp_mgr_o_error_corrupt_cnt) begin
                clr_error_corrupt_en <= 1'b1;
            end else if (!qp_mgr_o_error_corrupt_en) begin
                clr_error_corrupt_en <= 1'b0;
            end
        end
    end
    assign wqe_proc_top_m_axis_tdata = ((num_op_pkts == qp_mgr_o_error_corrupt_cnt) && tfirst) ? {~wqe_proc_top_m_axis_tdata_int[511], wqe_proc_top_m_axis_tdata_int[510:0]} : wqe_proc_top_m_axis_tdata_int;



    generate
        if (C_EN_FLOW_CTRL == 1) begin
            assign ctl_tx_pause_req = tx_pause_req;

            // xpm_cdc_array_single: Single-bit Array Synchronizer
            // Xilinx Parameterized Macro, version 2018.1
            xpm_cdc_array_single #(
                .DEST_SYNC_FF(4),  // DECIMAL; range: 2-10
                .INIT_SYNC_FF(0),  // DECIMAL; integer; 0=disable simulation init values, 1=enable simulation init
                // values
                .SIM_ASSERT_CHK(0),  // DECIMAL; integer; 0=disable simulation messages, 1=enable simulation messages
                .SRC_INPUT_REG(1),  // DECIMAL; 0=do not register input, 1=register input
                .WIDTH(9)  // DECIMAL; range: 1-1024
            ) xpm_cdc_array_single_inst (
                .dest_out(rx_pause_req),      // WIDTH-bit output: src_in synchronized to the destination clock domain. This
                // output is registered.
                .dest_clk(cmac_tx_clk),       // 1-bit input: Clock signal for the destination clock domain.
                .src_clk (cmac_rx_clk),       // 1-bit input: optional; required when SRC_INPUT_REG = 1
                .src_in  (stat_rx_pause_req)  // WIDTH-bit input: Input single-bit array to be synchronized to destination clock
                // domain. It is assumed that each bit of the array is unrelated to the others. This
                // is reflected in the constraints applied to this macro. To transfer a binary value
                // losslessly across the two clock domains, use the XPM_CDC_GRAY macro instead.
            );
            // End of xpm_cdc_array_single_inst instantiation
            ///////////////////////////////////////////////////////////////
            // xpm_cdc_array_single: Single-bit Array Synchronizer
            // Xilinx Parameterized Macro, version 2018.1
            // xpm_cdc_array_single #(
            // .DEST_SYNC_FF(4), // DECIMAL; range: 2-10
            // .INIT_SYNC_FF(0), // DECIMAL; integer; 0=disable simulation init values, 1=enable simulation init
            // // values
            // .SIM_ASSERT_CHK(0), // DECIMAL; integer; 0=disable simulation messages, 1=enable simulation messages
            // .SRC_INPUT_REG(1), // DECIMAL; 0=do not register input, 1=register input
            // .WIDTH(9) // DECIMAL; range: 1-1024
            // )
            // xpm_cdc_array_single_inst_1 (
            // .dest_out(tx_pause_req), // WIDTH-bit output: src_in synchronized to the destination clock domain. This
            // // output is registered.
            // .dest_clk(cmac_tx_clk), // 1-bit input: Clock signal for the destination clock domain.
            // .src_clk(m_axi_aclk), // 1-bit input: optional; required when SRC_INPUT_REG = 1
            // .src_in(ctl_tx_pause_req_out) // WIDTH-bit input: Input single-bit array to be synchronized to destination clock
            // // domain. It is assumed that each bit of the array is unrelated to the others. This
            // // is reflected in the constraints applied to this macro. To transfer a binary value
            // // losslessly across the two clock domains, use the XPM_CDC_GRAY macro instead.
            // );
            // // End of xpm_cdc_array_single_inst instantiation
            ///////////////////////////////////////////////////////////////	
        end else begin
            assign ctl_tx_pause_req = 9'h000;


            assign rx_pause_req     = 9'h000;


        end
    endgenerate

    always @(posedge m_axi_aclk) begin
        if (!m_axi_aresetn) rnic_intr <= 0;
        else rnic_intr <= xrnic_intr0 | xrnic_intr1 | xrnic_intr2 | xrnic_intr3 | xrnic_intr4 | xrnic_intr5 | xrnic_intr6 | xrnic_intr7 | xrnic_intr8;
    end

    //EVALUATION LOGIC----- himani
    arbit #(
        .C_HWTIMEOUT(C_IS_EVAL)
    ) arbitrationloss (
        .s_axi_lite_aclk         (s_axi_lite_aclk),
        .m_axi_aclk              (m_axi_aclk),
        .s_axi_lite_aresetn      (s_axi_lite_aresetn),
        .s_axi_lite_aresetn_sync1(s_axi_lite_aresetn_sync1),
        .s_axi_lite_aresetn_1    (s_axi_lite_aresetn_1)
        //   .TXE_TX_EN     (TXE_TX_EN),
        //   .TXE_ARB_LOST  (TXE_ARB_LOST)  
    );

    assign s_axi_lite_aresetn_eval = ~s_axi_lite_aresetn_1 & s_axi_lite_aresetn;
    //assign s_axi_lite_aresetn_eval = s_axi_lite_aresetn;

    flow_ctrl_mgr inst_flow_ctrl_mgr (
        .m_axi_aclk               (m_axi_aclk),
        .m_axi_aresetn            (m_axi_aresetn),
        .cmac_rx_clk              (cmac_rx_clk),
        .cmac_rx_rst              (cmac_rx_rst),
        .cmac_tx_clk              (cmac_tx_clk),
        .cmac_tx_rst              (cmac_tx_rst),
        .cmac_m_axis_tdata        (cmac_m_axis_tdata_out),
        .cmac_m_axis_tkeep        (cmac_m_axis_tkeep_out),
        .cmac_m_axis_tvalid       (cmac_m_axis_tvalid_out),
        .cmac_m_axis_tready       (cmac_m_axis_tready_out),
        .cmac_m_axis_tlast        (cmac_m_axis_tlast_out),
        .cmac_m_axis_tuser        (cmac_m_axis_tuser_out),
        .dma_s_axis_tvalid        (non_roce_dma_s_axis_tvalid),
        .dma_s_axis_tdata         (non_roce_dma_s_axis_tdata),
        .dma_s_axis_tkeep         (non_roce_dma_s_axis_tkeep),
        .dma_s_axis_tlast         (non_roce_dma_s_axis_tlast),
        .dma_s_axis_tuser         (1'b0),
        .dma_s_axis_tready        (non_roce_dma_s_axis_tready),
        .ernic_s_axis_tvalid      (wqe_proc_top_m_axis_tvalid),
        .ernic_s_axis_tdata       (wqe_proc_top_m_axis_tdata),
        .ernic_s_axis_tkeep       (wqe_proc_top_m_axis_tkeep),
        .ernic_s_axis_tlast       (wqe_proc_top_m_axis_tlast),
        .ernic_s_axis_tready      (wqe_proc_top_m_axis_tready),
        // Prioroty flow control
        .stat_rx_pause_req        (rx_pause_req),                // 暂停请求信号。CMAC IP 在有效量子周期内断言该信号。
        .ctl_tx_pause_req         (tx_pause_req),                // 当缓冲区阈值介于 XON 和 XOFF 之间时，该总线被断言，优先级位通过优先级寄存器设置。
        .ctl_tx_resend_pause      (ctl_tx_resend_pause),         // 该信号硬接线为 0，可连接 CMAC IP 信号。
        .roce_priority_s          (roce_priority),
        .non_roce_priority_s      (non_roce_priority),
        .roce_rx_buf_full         (tx_pause_req_on),
        .non_roce_rx_buf_count    (fifo_data_count),
        .non_roce_rx_fifo_full    (non_roce_rx_fifo_full),
        .pfc_en_roce_s            (pfc_en_roce),
        .pfc_en_non_roce_s        (pfc_en_non_roce),
        .non_roce_pause_off_offset(non_roce_pause_off_offset),
        .disable_priority_chk_s   (disable_priority_chk),
        .non_roce_pause_on_offset (non_roce_pause_on_offset)
    );

    reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_inst (
        .aclk          (cmac_tx_clk),
        .areset        (cmac_tx_rst),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({cmac_m_axis_tlast_out, cmac_m_axis_tkeep_out, cmac_m_axis_tdata_out}),
        .S_VALID       (cmac_m_axis_tvalid_out),
        .S_READY       (cmac_m_axis_tready_out),

        .M_PAYLOAD_DATA(cmac_reg_out),
        .M_VALID       (cmac_m_axis_tvalid),
        .M_READY       (cmac_m_axis_tready)
    );

    assign cmac_m_axis_tdata = cmac_reg_out[511:0];
    assign cmac_m_axis_tkeep = cmac_reg_out[575:512];
    assign cmac_m_axis_tlast = cmac_reg_out[576];

    // xpm_fifo_axis: AXI Stream FIFO
    // Xilinx Parameterized Macro, version 2020.1
    xpm_fifo_axis #(
        .CDC_SYNC_STAGES(2),  // DECIMAL
        .CLOCKING_MODE("common_clock"),  // String
        .ECC_MODE("no_ecc"),  // String
        .FIFO_DEPTH(256),  // DECIMAL
        .FIFO_MEMORY_TYPE("auto"),  // String
        .PACKET_FIFO("true"),  // String，这里使能Packet模式，表示下游的处理模块接收Stream数据时要求数据以帧为单位，且接收一帧数据时，数据要连续有效传输，直到一个完整的数据帧结束。
        // 启用Packet模式后，会自动在输入输出端口加上last信号，此时AXI4-Stream Data FIFO IP核在缓存数据时，会在接收到有效的last信号后（或者内部FIFO存满后），再将数据发给下游模块。
        // 若是非Packet模式时，IP核不会等待last信号，当接收到数据后紧接着就开始输出数据。
        .PROG_EMPTY_THRESH(10),  // DECIMAL
        .PROG_FULL_THRESH(256 - 66),  // DECIMAL
        .RD_DATA_COUNT_WIDTH(9),  // DECIMAL
        .RELATED_CLOCKS(0),  // DECIMAL
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .TDATA_WIDTH(512),  // DECIMAL
        .TDEST_WIDTH(1),  // DECIMAL
        .TID_WIDTH(1),  // DECIMAL
        .TUSER_WIDTH(1),  // DECIMAL
        .USE_ADV_FEATURES("1F0F"),  // String
        .WR_DATA_COUNT_WIDTH(9)  // DECIMAL
    ) non_roce_rx_xpm_fifo_axis_inst (
        .almost_empty_axis (),                                                       // 1-bit output: Almost Empty : When asserted, this signal
        // indicates that only one more read can be performed before the
        // FIFO goes to empty.
        .almost_full_axis  (non_roce_rx_fifo_full),                                  // 1-bit output: Almost Full: When asserted, this signal
        // indicates that only one more write can be performed before
        // the FIFO is full.
        .dbiterr_axis      (),                                                       // 1-bit output: Double Bit Error- Indicates that the ECC
        // decoder detected a double-bit error and data in the FIFO core
        // is corrupted.
        .m_axis_tdata      (non_roce_dma_m_axis_tdata),                              // TDATA_WIDTH-bit output: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .m_axis_tdest      (),                                                       // TDEST_WIDTH-bit output: TDEST: Provides routing information
        // for the data stream.
        .m_axis_tid        (),                                                       // TID_WIDTH-bit output: TID: The data stream identifier that
        // indicates different streams of data.
        .m_axis_tkeep      (non_roce_dma_m_axis_tkeep),                              // TDATA_WIDTH/8-bit output: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // and can be removed from the data stream. For a 64-bit DATA,
        // bit 0 corresponds to the least significant byte on DATA, and
        // bit 7 corresponds to the most significant byte. For example:
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .m_axis_tlast      (non_roce_dma_m_axis_tlast),                              // 1-bit output: TLAST: Indicates the boundary of a packet.
        .m_axis_tstrb      (),                                                       // TDATA_WIDTH/8-bit output: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .m_axis_tuser      (),                                                       // TUSER_WIDTH-bit output: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .m_axis_tvalid     (non_roce_dma_m_axis_tvalid),                             // 1-bit output: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
        .prog_empty_axis   (fifo_empty_pause_off),                                   // 1-bit output: Programmable Empty- This signal is asserted
        // when the number of words in the FIFO is less than or equal to
        // the programmable empty threshold value. It is de-asserted
        // when the number of words in the FIFO exceeds the programmable
        // empty threshold value.
        .prog_full_axis    (fifo_prog_full),                                         // 1-bit output: Programmable Full: This signal is asserted when
                                                                                     // the number of words in the FIFO is greater than or equal to
                                                                                     // the programmable full threshold value. It is de-asserted when
                                                                                     // the number of words in the FIFO is less than the programmable
                                                                                     // full threshold value.
        .rd_data_count_axis(),                                                       // RD_DATA_COUNT_WIDTH-bit output: Read Data Count- This bus
        // indicates the number of words available for reading in the
        // FIFO.
        .s_axis_tready     (),                                                       // 1-bit output: TREADY: Indicates that the slave can accept a
        // transfer in the current cycle.
        .sbiterr_axis      (),                                                       // 1-bit output: Single Bit Error- Indicates that the ECC
        // decoder detected and fixed a single-bit error.
        .wr_data_count_axis(fifo_data_count),                                        // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus
        // indicates the number of words written into the FIFO.
        .injectdbiterr_axis(1'b0),                                                   // 1-bit input: Double Bit Error Injection- Injects a double bit
        // error if the ECC feature is used.
        .injectsbiterr_axis(1'b0),                                                   // 1-bit input: Single Bit Error Injection- Injects a single bit
        // error if the ECC feature is used.
        .m_aclk            (m_axi_aclk),                                             // 1-bit input: Master Interface Clock: All signals on master
        // interface are sampled on the rising edge of this clock.
        .m_axis_tready     (non_roce_dma_m_axis_tready),                             // 1-bit input: TREADY: Indicates that the slave can accept a
        // transfer in the current cycle.
        .s_aclk            (m_axi_aclk),                                             // 1-bit input: Slave Interface Clock: All signals on slave
        // interface are sampled on the rising edge of this clock.
        .s_aresetn         (m_axi_aresetn),                                          // 1-bit input: Active low asynchronous reset.
        .s_axis_tdata      (non_roce_cmac_s_axis_tdata_3d),                          // TDATA_WIDTH-bit input: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .s_axis_tdest      (1'b0),                                                   // TDEST_WIDTH-bit input: TDEST: Provides routing information
        // for the data stream.
        .s_axis_tid        (1'b0),                                                   // TID_WIDTH-bit input: TID: The data stream identifier that
        // indicates different streams of data.
        .s_axis_tkeep      (non_roce_cmac_s_axis_tkeep_3d),                          // TDATA_WIDTH/8-bit input: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // and can be removed from the data stream. For a 64-bit DATA,
        // bit 0 corresponds to the least significant byte on DATA, and
        // bit 7 corresponds to the most significant byte. For example:
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .s_axis_tlast      (non_roce_cmac_s_axis_tlast_3d && !(drop_non_roce_pkt)),  // 1-bit input: TLAST: Indicates the boundary of a packet.
        .s_axis_tstrb      ({64{1'b1}}),                                             // TDATA_WIDTH/8-bit input: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .s_axis_tuser      (non_roce_cmac_s_axis_tuser_3d),                          // TUSER_WIDTH-bit input: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .s_axis_tvalid     (non_roce_cmac_s_axis_tvalid_3d && !(drop_non_roce_pkt))  // 1-bit input: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
    );
    // End of xpm_fifo_axis_inst instantiation 

    // 打一拍
    reg_slice #(
        .C_DATA_WIDTH(578)
    ) reg_slice_inst_2 (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({non_roce_cmac_s_axis_tuser, non_roce_cmac_s_axis_tlast, non_roce_cmac_s_axis_tkeep, non_roce_cmac_s_axis_tdata}),
        .S_VALID       (non_roce_cmac_s_axis_tvalid),
        .S_READY       (non_roce_cmac_s_axis_tready),

        .M_PAYLOAD_DATA(non_roce_cmac_reg),
        .M_VALID       (non_roce_cmac_s_axis_tvalid_d),
        .M_READY       (1'b1)
    );
    // 打两拍
    reg_slice #(
        .C_DATA_WIDTH(578)
    ) reg_slice_inst_3 (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA(non_roce_cmac_reg),
        .S_VALID       (non_roce_cmac_s_axis_tvalid_d),
        .S_READY       (),

        .M_PAYLOAD_DATA(non_roce_cmac_reg_d),
        .M_VALID       (non_roce_cmac_s_axis_tvalid_2d),
        .M_READY       (1'b1)
    );
    // 打两拍的数据
    assign non_roce_cmac_s_axis_tdata_2d = non_roce_cmac_reg_d[511:0];
    assign non_roce_cmac_s_axis_tkeep_2d = non_roce_cmac_reg_d[575:512];
    assign non_roce_cmac_s_axis_tlast_2d = non_roce_cmac_reg_d[576];
    assign non_roce_cmac_s_axis_tuser_2d = non_roce_cmac_reg_d[577];

    // 打一拍的数据
    assign non_roce_cmac_s_axis_tdata_d  = non_roce_cmac_reg[511:0];
    assign non_roce_cmac_s_axis_tkeep_d  = non_roce_cmac_reg[575:512];
    assign non_roce_cmac_s_axis_tlast_d  = non_roce_cmac_reg[576];
    assign non_roce_cmac_s_axis_tuser_d  = non_roce_cmac_reg[577];

    // 打三拍
    reg_slice #(
        .C_DATA_WIDTH(578)
    ) reg_slice_inst_4 (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA(non_roce_cmac_reg_d),
        .S_VALID       (non_roce_cmac_s_axis_tvalid_2d),
        .S_READY       (),
        .M_PAYLOAD_DATA(non_roce_cmac_reg_2d),
        .M_VALID       (non_roce_cmac_s_axis_tvalid_3d),
        .M_READY       (1'b1)
    );
    // 打三拍的数据
    assign non_roce_cmac_s_axis_tdata_3d = non_roce_cmac_reg_2d[511:0];
    assign non_roce_cmac_s_axis_tkeep_3d = non_roce_cmac_reg_2d[575:512];
    assign non_roce_cmac_s_axis_tlast_3d = non_roce_cmac_reg_2d[576];
    assign non_roce_cmac_s_axis_tuser_3d = non_roce_cmac_reg_2d[577];


    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) pkt_boundry <= 1'b1;
        else if (non_roce_cmac_s_axis_tvalid_2d) pkt_boundry <= non_roce_cmac_s_axis_tlast_2d;
    end

    // 第2拍的 最后一拍且有效 信号
    assign sop = pkt_boundry && non_roce_cmac_s_axis_tvalid_2d;

    // non_roce_rx_xpm_fifo_axis_inst满了，该包就丢掉（不存入fifo也不输出）
    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) drop_non_roce_pkt <= 1'b0;
        else if (sop) begin
            if (fifo_prog_full) drop_non_roce_pkt <= 1'b1;
            else drop_non_roce_pkt <= 1'b0;
        end
    end

    // always @(posedge m_axi_aclk) begin
    // if (~m_axi_aresetn) begin
    // state <= IDLE;
    // pkt_boundry <= 1'b0;
    // end else begin
    // case (state)
    // IDLE:

    // if (non_roce_cmac_s_axis_tvalid_d && non_roce_cmac_s_axis_tlast_d) begin
    // state <= PKT_BNDRY_START;
    // pkt_boundry <= 1'b1;
    // end else begin
    // state <= IDLE;
    // pkt_boundry <= 1'b0;
    // end

    // PKT_BNDRY_START:
    // if (non_roce_cmac_s_axis_tvalid_d) begin
    // state <= IDLE;
    // pkt_boundry <= 1'b0;
    // end else begin
    // state <= PKT_BNDRY_START;
    // pkt_boundry <= 1'b1;
    // end
    // default: state <= IDLE;
    // endcase
    // end
    // end


    // always @(posedge m_axi_aclk)
    // begin
    // pkt_boundry_d <= pkt_boundry;
    // end

    // assign sop = !pkt_boundry & (pkt_boundry_d);

    // always @(posedge m_axi_aclk) begin
    // if (~m_axi_aresetn) begin
    // drop_non_roce_pkt <= 1'b0;
    // end else begin
    // if (sop && fifo_prog_full) 
    // drop_non_roce_pkt <= 1'b1;
    // else if (non_roce_cmac_s_axis_tlast_3d)
    // drop_non_roce_pkt <= 1'b0;
    // end
    // end


    //assign drop_non_roce_pkt = tlast_strech && fifo_prog_full;

    // reg_slice #(
    // .C_DATA_WIDTH (578)
    // )
    // reg_slice_roce_in_inst (
    // .aclk (m_axi_aclk),
    // .areset (~m_axi_aresetn),
    // .aclken (1'b1),
    // .S_PAYLOAD_DATA ({roce_cmac_s_axis_tuser,roce_cmac_s_axis_tlast,roce_cmac_s_axis_tkeep,roce_cmac_s_axis_tdata}),
    // .S_VALID (roce_cmac_s_axis_tvalid),
    // .S_READY (roce_cmac_s_axis_tready),
    // .M_PAYLOAD_DATA (roce_reg_out),
    // .M_VALID (roce_reg_tvalid_out),
    // .M_READY (1'b1)
    // );
    // assign roce_reg_tdata_out = roce_reg_out[511:0];
    // assign roce_reg_tkeep_out = roce_reg_out[575:512];
    // assign roce_reg_tlast_out = roce_reg_out[576];
    // assign roce_reg_tuser_out = roce_reg_out[577];

    rx_pkt_handler #(
        .NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN(NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN | NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING),  // 0
        .C_SIM_DEBUG(0),  // 0
        .C_EN_INITIATOR_LITE(C_EN_INITIATOR_LITE),  // 1
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),  // 32
        .C_M_AXI_AWUSER_WIDTH(C_M_AXI_AWUSER_WIDTH),  // 32
        .C_NUM_QP(C_NUM_QP),  // 32
        .C_QP_INDX_WIDTH(C_QP_INDX_WIDTH),  // 5
        .C_NUM_PD(C_NUM_PD),  // 2048
        .C_PD_INDX_WIDTH(C_PD_INDX_WIDTH),  // 11
        .C_OSQ_PSN_WIDTH(C_OSQ_PSN_WIDTH),  // 16
        .C_EN_DEBUG_PORTS(C_EN_DEBUG_PORTS),  // 0
        .C_EN_IETH_IMMDT(C_EN_IETH_IMMDT)  // 1
    ) u_rx_pkt_handler (
        .core_clk  (m_axi_aclk),
        .core_rst_n(core_rstn_rx_reg),

        .s_axis_tvalid(roce_cmac_s_axis_tvalid),
        .s_axis_tdata (roce_cmac_s_axis_tdata),
        .s_axis_tkeep (roce_cmac_s_axis_tkeep),
        .s_axis_tlast (roce_cmac_s_axis_tlast),
        .s_axis_tuser (roce_cmac_s_axis_tuser),

        .i_xrnic_en       (qp_mgr_o_xrnic_en),                  // XRNIC Configuration -- [0] XRNIC Enable
        .i_num_qp_enabled (qp_mgr_o_num_qp_en),
        .i_mac_node_addr  (qp_mgr_o_mac_src_addr),
        .i_ipv4_node_addr (qp_mgr_o_ipv4_src_addr),
        .i_ipv6_node_addr (ipv6_src_addr_rx_reg),
        .i_rnr_nak_tval   (qp_mgr_o_qp_timeout_per_qp[20:16]),  // RNR NACK 传出数据包的超时值
        .i_rnr_nak_rst_val(qp_mgr_o_qp_timeout_per_qp[13:11]),  // 最大 RNR 重试次数
        .i_flow_credits   (qp_mgr_o_flow_credits),

        .i_depkt_bypass_en (qp_mgr_o_depkt_bypass_en),
        .i_bypass_buf_ba   ({{C_M_AXI_ADDR_WIDTH - 32{1'b0}}, qp_mgr_o_bypass_buf_ba}),
        .i_bypass_num_bufs (qp_mgr_o_bypass_buf_sz_num_bufs),
        .i_bypass_buffer_sz(qp_mgr_o_bypass_buf_sz_buf_sz),
        .o_bypass_buf_wrptr(rx_pkt_hndler_o_bypass_buf_wrptr),

        .i_req_err_buf_en       (qp_mgr_o_err_buf_en),
        .i_req_err_buf_ovr_wr_en(qp_mgr_o_xrnic_adv_conf_errbuf_overwr_en),
        .i_req_err_pkt_buf_ba   (qp_mgr_o_err_pkt_buf_ba),
        .i_req_err_num_bufs     (qp_mgr_o_err_pkt_buf_sz_num_bufs),
        .i_req_err_buffer_sz    (qp_mgr_o_err_pkt_buf_sz_buf_sz),
        .o_req_err_buf_wrptr    (rx_pkt_hndler_o_err_pkt_buf_wrptr),

        .i_rsp_err_buf_en       (qp_mgr_o_err_buf_en),
        .i_rsp_err_buf_ovr_wr_en(qp_mgr_o_xrnic_adv_conf_errbuf_overwr_en),
        .i_rsp_err_pkt_buf_ba   (qp_mgr_o_resp_err_pkt_buf_ba),
        .i_rsp_err_num_bufs     (qp_mgr_o_resp_err_pkt_buf_sz_num_bufs),
        .i_rsp_err_buffer_sz    (qp_mgr_o_resp_err_pkt_buf_sz_buf_sz),
        .o_rsp_err_buf_wrptr    (rx_pkt_hndler_o_rsp_err_buf_wrptr),

        .i_in_errsts_q_ba              (qp_mgr_in_errsts_q_ba),
        .i_in_errsts_q_sz              (qp_mgr_in_errsts_q_sz),
        .o_in_errsts_q_wrptr           (rx_pkt_hndler_in_errsts_q_wrptr),
        .o_inc_send_cnt                (rx_pkt_hndler_o_ib_send_pkt_cnt),
        .o_inc_acknak_cnt              (rx_pkt_hndler_o_ib_ack_pkt_cnt),
        .o_inc_rresp_cnt               (rx_pkt_hndler_o_ib_rres_pkt_cnt),
        .o_inc_mad_cnt                 (rx_pkt_hndler_o_ib_qp1_pkt_cnt),
        .o_inc_inv_pkt_cnt             (rx_pkt_hndler_o_ib_inv_pkt_cnt),
        .o_inc_dup_pkt_cnt             (rx_pkt_hndler_o_ib_dupl_pkt_cnt),
        .o_inc_nack_pkt_cnt            (rx_pkt_hndler_o_inc_nack_cnt),
        .o_out_nack_pkt_cnt            (rx_pkt_hndler_o_out_nack_cnt),
        .pkt_all_cnt                   (rx_pkt_hndler_o_pkt_all_cnt),
        .pkt_disc_cnt                  (rx_pkt_hndler_o_pkt_disc_cnt),
        //.i_qp_enable                   (qp_mgr_o_qp_enable),
        .i_qp_enable_wr_en             (qp_enable_wr_en),
        .i_qp_enable_wr_idx            (qp_enable_wr_addr),
        .i_qp_enable_wr_data           (qp_enable_wr_data),
        .o_last_in_req_pkt_opcode      (),
        .o_last_in_req_pkt_qpid        (),
        .o_last_in_req_pkt_psn         (),
        .o_last_in_rsp_pkt_opcode      (),
        .o_last_in_rsp_pkt_qpid        (),
        .o_last_in_rsp_pkt_psn         (),
        .o_last_in_pkt_opcode          (rx_pkt_hndler_o_last_ib_pkt_opcode),
        .o_last_in_pkt_qpid            (rx_pkt_hndler_o_last_ib_pkt_dest_qpid),
        .o_last_in_pkt_psn             (rx_pkt_hndler_o_last_ib_pkt_psn),
        //.o_rq_full                     (rx_pkt_hndler_o_rq_full          ),
        .o_rq_full_idx                 (rx_pkt_hndler_o_rq_full_idx),
        .o_rq_full_wr_en               (rx_pkt_hndler_o_rq_full_wr_en),
        .o_rq_full_val                 (rx_pkt_hndler_o_rq_full_val),
        .o_rq_full_reset_idx           (rx_pkt_hndler_o_rq_full_reset_idx),
        .o_rq_full_reset_wr_en         (rx_pkt_hndler_o_rq_full_reset_wr_en),
        .o_rq_full_reset_val           (rx_pkt_hndler_o_rq_full_reset_val),
        .o_qp_fatal_wr_en              (rx_pkt_hndler_o_qp_fatal_wr_en),
        .o_qp_fatal_wr_idx             (rx_pkt_hndler_o_qp_fatal_wr_idx),
        .o_qp_fatal_wr_data            (rx_pkt_hndler_o_qp_fatal_wr_data),
        //.i_clr_qp_fatal                (qp_mgr_o_qp_clr_fatal_err        ),
        .i_clr_qp_fatal                (clr_qp_fatal_err),
        .i_clr_qp_fatal_idx            (clr_qp_fatal_idx),
        .i_rq_pi_db_hw_hndshk          (qp_mgr_o_rq_pi_db_hw_hndshk),
        .i_connect_io_qp_rq_pi_db_wptr ({1'b0, qp_mgr_o_connect_io_qp_rq_pi_db_wptr}),
        .i_connect_io_qpid             (qp_mgr_o_connect_io_qpid),
        .i_hw_hndshk_disable_to_0      (qp_mgr_o_hw_hndshk_disable_to_0),
        .o_connect_io_qp_rq_pi_db_rdy  (rx_pkt_hndler_o_connect_io_qp_rq_pi_db_rdy),
        //.qp_under_recovery             (qp_mgr_o_qp_under_recovery),
        .qp_under_recovery_wr_en       (qp_under_recovery_wr_en),
        .qp_under_recovery_wr_idx      (qp_under_recovery_wr_idx),
        .qp_under_recovery_wr_data     (qp_under_recovery_wr_data),
        .o_rq_db_data                  (rx_pkt_hndler_o_rq_db_data),
        .o_rq_db_addr                  (rx_pkt_hndler_o_rq_db_addr),
        .o_rq_db_data_valid            (rx_pkt_hndler_o_rq_db_data_valid),
        .i_rq_db_data_ack              (rx_pkt_hndler_i_rq_db_rdy),
        .i_pkt_valdn_err_intr_en       (qp_mgr_o_intr_en_pkt_valdn_err),
        .o_pkt_valdn_err_intr          (rx_pkt_hndler_o_ib_pkt_hdr_valdn_err_intr),
        .o_pkt_valdn_err_intr_sts      (rx_pkt_hndler_o_ib_pkt_hdr_valdn_err_intr_sts),
        .i_clr_pkt_valdn_err_intr      (qp_mgr_o_intr_clr_pkt_valdn_err),
        .i_mad_pkt_rcvd_intr_en        (qp_mgr_o_intr_en_mad_pkt_rcvd),
        .o_mad_pkt_rcvd_intr           (rx_pkt_hndler_o_mad_pkt_rcvd_intr),
        .o_mad_pkt_rcvd_intr_sts       (rx_pkt_hndler_o_mad_pkt_rcvd_intr_sts),
        .i_clr_mad_pkt_rcvd_intr       (qp_mgr_o_intr_clr_mad_pkt_rcvd),
        .i_bypass_pkt_rcvd_intr_en     (qp_mgr_o_intr_en_bypass_pkt_rcvd),
        .o_bypass_pkt_rcvd_intr        (rx_pkt_hndler_o_bypass_pkt_rcvd_intr),
        .o_bypass_pkt_rcvd_intr_sts    (rx_pkt_hndler_o_bypass_pkt_rcvd_intr_sts),
        .i_clr_bypass_pkt_rcvd_intr    (qp_mgr_o_intr_clr_bypass_pkt_rcvd),
        .i_rnr_nack_gen_intr_en        (qp_mgr_o_intr_en_rnr_nak_gen),
        .o_rnr_nack_gen_intr           (rx_pkt_hndler_o_rnr_nack_gen_intr),
        .o_rnr_nack_gen_intr_sts       (rx_pkt_hndler_o_rnr_nack_gen_intr_sts),
        .i_clr_rnr_nack_gen_intr       (qp_mgr_o_intr_clr_rnr_nak_gen),
        .i_fatal_err_intr_en           (qp_mgr_o_intr_en_fatal_err),
        .o_fatal_err_intr              (rx_pkt_hndler_o_fatal_err_intr),
        .o_fatal_err_intr_sts          (rx_pkt_hndler_fatal_err_intr_sts),
        .i_clr_fatal_err_intr          (qp_mgr_o_intr_clr_fatal_err),
        .i_qp_pkt_rcvd_intr_en         (qp_mgr_o_intr_en_qp_pkt_rcvd),
        .o_qp_pkt_rcvd_intr            (rx_pkt_hndler_o_qp_pkt_rcvd_intr),
        //.o_qp_pkt_rcvd_intr_sts        (rx_pkt_hndler_o_qp_pkt_rcvd_intr_sts ),
        .o_qp_pkt_rcvd_intr_sts        (rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_data),
        .o_qp_pkt_rcvd_intr_sts_wr_addr(rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_addr),
        .o_qp_pkt_rcvd_intr_sts_wr_en  (rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_en),
        //.i_clr_qp_pkt_rcvd_intr        (qp_mgr_o_rq_intr_sts_clr         ),
        .i_clr_qp_pkt_rcvd_intr        (clr_qp_pkt_rcvd_intr),
        .i_clr_qp_pkt_rcvd_intr_wr_addr(clr_qp_pkt_rcvd_intr_wr_addr),
        .i_cnp_sched_intr_en           (rx_pkt_hndlr_i_cnp_sched_intr_en),
        .o_cnp_sched_intr              (rx_pkt_hndlr_o_cnp_sched_intr),
        .o_cnp_sched_intr_sts          (rx_pkt_hndlr_o_cnp_sched_intr_sts),
        .i_clr_cnp_sched_intr          (rx_pkt_hndlr_i_clr_cnp_sched_intr),

        //.o_cnp_sched_reg          (rx_pkt_hndlr_o_cnp_sched_reg     ),
        .o_cnp_sched_wr_addr    (rx_pkt_hndlr_o_cnp_sched_wr_addr),
        .o_cnp_sched_wr_data    (rx_pkt_hndlr_o_cnp_sched_wr_data),
        .o_cnp_sched_wr_en      (rx_pkt_hndlr_o_cnp_sched_wr_en),
        //.i_clr_cnp_sched          (rx_pkt_hndlr_i_clr_cnp_sched     ),
        .i_clr_cnp_sched        (clr_cnp_sched),
        .i_clr_cnp_sched_wr_addr(clr_cnp_sched_wr_addr),

        .i_cnp_timeout_reg_1(rx_pkt_hndlr_i_cnp_timeout_reg_1),  // Reserved
        .i_cnp_timeout_reg_2(rx_pkt_hndlr_i_cnp_timeout_reg_2),  // Reserved

        .o_cnp_pkt_rcvd_wr_idx (i_tx_path_congestion_id),
        .o_cnp_pkt_rcvd_wr_en  (i_tx_path_congestion_valid),
        .o_cnp_pkt_rcvd_wr_data(i_tx_path_congestion_data),
        .o_cnp_rx_pkt_cnt      (rx_pkt_hndlr_o_cnp_rx_pkt_cnt),

        .o_qp_index_mpsnbuf           (rx_pkt_hndler_o_qp_index_mpsnbuf),
        .o_max_epsn_req               (rx_pkt_hndler_o_max_epsn_req),
        .i_max_epsn_vld               (resp_hndler_o_mpsn_buf_valid),
        //.i_mpsnbuf_empty               (resp_hndler_o_mpsnbuf_empty      ),
        .i_mpsnbuf_empty_set_valid    (mpsnbuf_empty_set_valid),
        .i_mpsnbuf_empty_set_id       (mpsnbuf_empty_set_id),
        .i_mpsnbuf_empty_reset_valid  (mpsnbuf_empty_reset_valid),
        .i_mpsnbuf_empty_reset_id     (mpsnbuf_empty_reset_id),
        //.i_qp_retried                  (resp_hndler_o_qp_retried         ),
        .i_qp_retried_set_valid       (qp_retried_set_valid),
        .i_qp_retried_set_id          (qp_retried_set_id),
        .i_qp_retried_reset_valid     (qp_retried_reset_valid),
        .i_qp_retried_reset_id        (qp_retried_reset_id),
        .o_mpsnbuf_pop                (rx_pkt_hndler_o_mpsnbuf_pop),
        .i_os_rdma_rd_resp_psn_7_0    (resp_hndler_o_max_epsn),
        .i_os_rdma_rd_resp_dest_addr  (resp_hndler_o_local_addr[C_M_AXI_ADDR_WIDTH-1:0]),
        .i_os_rdma_rd_resp_length     ({8'h00, resp_hndler_o_trnsfr_length[23:0]}),
        .o_qp_index_acknack           (rx_pkt_hndler_o_qp_index_acknack),
        .o_acknack_valid              (rx_pkt_hndler_o_acknack_valid),
        .o_acknacked_psn              (rx_pkt_hndler_o_acknacked_psn),
        .o_acknacked_syndrome         (rx_pkt_hndler_o_acknacked_syndrome),
        .o_acknacked_msn              (rx_pkt_hndler_o_acknacked_msn),
        .o_acknacked_opcode           (rx_pkt_hndler_o_acknacked_opcode),
        .o_qp_timeout_per_qp          (rx_pkt_hndler_o_qp_timeout_per_qp),
        //.i_osq_empty                   (resp_hndler_o_osq_empty           ),
        .i_osq_empty_set_valid        (osq_empty_set_valid),
        .i_osq_empty_set_id           (osq_empty_set_id),
        .i_osq_empty_reset_valid      (osq_empty_reset_valid),
        .i_osq_empty_reset_id         (osq_empty_reset_id),
        .i_osq_nacked                 (o_osq_nacked),
        .o_rd_rsp_wr_cmpltd           (rx_pkt_hndler_o_rd_rsp_wr_cmpltd),
        .i_qp_fatal_wqe_proc          (wqe_fatal_qpid),
        .i_qp_fatal_wqe_proc_vld      (wqe_fatal_qp_vld),
        .o_qp_fatal_wqe_proc_rdy      (wqe_fatal_qp_rdy),
        .i_qp_fatal_rsp_hndl          (resp_handler_o_qp_retry_fatal_qpid),
`ifdef RETRY_CNT_QP_FATAL_DIS
        .i_qp_fatal_rsp_hndl_vld      (1'b0),                                              //TODO resp_handler_o_qp_retry_fatal_req),
`else
        .i_qp_fatal_rsp_hndl_vld      (resp_handler_o_qp_retry_fatal_req),
`endif
        .o_qp_fatal_rsp_hndl_rdy      (resp_handler_i_qp_retry_fatal_rdy),
        .o_rsp_qp_id                  (rx_pkt_hndler_o_rsp_qp_id),
        .o_rsp_dest_addr_for_hw_hndshk(rx_pkt_hndler_o_rsp_dest_addr),
        .o_ob_rsp_qpid                (rx_pkt_wqe_proc_ob_rsp_qpid),
        .o_ob_rsp_psn                 (rx_pkt_wqe_proc_ob_rsp_psn),
        .o_ob_rsp_aeth_syndrome       (rx_pkt_wqe_proc_aeth_syndrome),
        .o_ob_rsp_aeth_msn            (rx_pkt_wqe_proc_ob_rsp_msn),
        .o_ob_rsp_expl_ack            (rx_pkt_wqe_proc_ob_rsp_expl_ack),
        .o_ob_rsp_vld                 (rx_pkt_wqe_proc_ob_rsp_req),
        .ob_rsp_fifo_full             (wqe_proc_o_bres_fifo_full),

        .o_req_indicator             (req_indicator),
        .o_physical_addr             (physical_addr),
        .o_dma_len                   (rd_dma_len),
        .o_rd_req_pmtu               (rd_req_pmtu),
        .o_rsp_last_psn              (rsp_last_psn),
        //.i_rd_req_fifo_full            (rd_req_fifo_full),
        .i_rd_req_fifo_full_set_idx  (rd_req_fifo_full_wr_addr_1),
        .i_rd_req_fifo_full_set      (rd_req_fifo_full_wr_en_1),
        .i_rd_req_fifo_full_reset_idx(rd_req_fifo_full_wr_addr_0),
        .i_rd_req_fifo_full_reset    (rd_req_fifo_full_wr_en_0),
        .i_pd_pd_num                 (pd_pd_num),
        .i_pd_virt_addr_0            (pd_virt_addr_0),
        .i_pd_virt_addr_1            (pd_virt_addr_1),
        .i_pd_buf_base_addr_0        (pd_buf_base_addr_0),
        .i_pd_buf_base_addr_1        (pd_buf_base_addr_1),
        .i_pd_buf_r_key              (pd_buf_r_key),
        .i_pd_w_r_buf_len            (pd_w_r_buf_len),
        .i_pd_access_type            (pd_access_type),


        .o_qp_req_conf_idx           (rx_pkt_hndler_o_qp_req_conf_idx),
        .o_pd_req_conf_idx           (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_qp_req_conf               (qp_mgr_o_qp_conf_replica),                                                                                                                              // need revisit
        .qp_req_conf_vld             (qp_mgr_o_qp_conf_replica_valid),                                                                                                                        // need revisit
        .qp_req_conf_en              (rx_pkt_hndler_o_qp_req_conf_en),                                                                                                                        // need revisit
        .pd_req_conf_en              (rx_pkt_hndler_o_pd_req_conf_en),                                                                                                                        // need revisit
        .i_rq_buf_ba                 (qp_mgr_o_qp_rq_ba),
        .rq_buf_ba_en                (rx_pkt_hndler_o_qpm_req_rq_buf_ba),
        .i_rq_buf_ca                 (qp_mgr_o_qp_stat_rq_buf_ca),
        .o_rq_buf_ca                 (rx_pkt_hndler_o_rq_buf_ca),
        .rq_buf_ca_en                (rx_pkt_hndler_o_qpm_req_rq_buf_ca),
        .rq_buf_ca_we                (rx_pkt_hndler_o_qpm_rw_rq_buf_ca),
        .i_rq_wrptr_db_addr          (qp_mgr_o_qp_rq_wrptrdb_add),
        .rq_wrptr_db_addr_en         (rx_pkt_hndler_o_qpm_req_rq_wrptr_db_add),
        .i_q_depth                   (qp_mgr_o_qp_rq_depth),
        .q_depth_en                  (rx_pkt_hndler_o_qpm_req_q_depth),
        .i_rq_ci_db                  (qp_mgr_o_qp_rq_cidb[15:0]),
        .rq_ci_db_en                 (rx_pkt_hndler_o_qpm_req_rq_ci_db),
        .i_last_rq_req               (qp_mgr_o_qp_last_rq),
        .o_last_rq_req               (rx_pkt_hndler_o_last_rq_req),
        .last_rq_req_en              (rx_pkt_hndler_o_qpm_req_last_rq_req),
        .last_rq_req_we              (rx_pkt_hndler_o_qpm_rw_last_rq_req),
        .i_qp_pd_num                 (qp_mgr_o_qp_pd_num),
        .qp_pd_num_en                (rx_pkt_hndler_o_qpm_req_qp_pd_num),
        .i_stat_qp_msn               (qp_mgr_o_qp_stat_msn),
        .o_stat_qp_msn               (rx_pkt_hndler_o_stat_qp_msn),
        .stat_qp_msn_en              (rx_pkt_hndler_o_qpm_req_stat_qp_msn),
        .stat_qp_msn_we              (rx_pkt_hndler_o_qpm_rw_stat_qp_msn),
        .i_qp_req_remote_qp_mac_addr ({qp_mgr_o_qp_mac_remote_addrm_replica[15:0], qp_mgr_o_qp_mac_remote_addrl_replica}),                                                                    // need revisit
        .qp_req_remote_qp_mac_addr_en(rx_pkt_hndler_req_remote_qp_mac_addr_en),                                                                                                               // need revisit
        .i_qp_req_remote_qp_ip_addr  ({qp_mgr_o_qp_ip_remote_addr4_replica, qp_mgr_o_qp_ip_remote_addr3_replica, qp_mgr_o_qp_ip_remote_addr2_replica, qp_mgr_o_qp_ip_remote_addr1_replica}),  // need revisit
        .qp_req_remote_qp_ip_addr_en (rx_pkt_hndler_req_remote_qp_ip_addr_en),
        .i_rq_wrptr_db               (qp_mgr_o_rq_pi_db),
        .o_rq_wrptr_db               (rx_pkt_hndler_o_rq_wrptr_db),
        .rq_wrptr_db_en              (rx_pkt_hndler_o_qpm_req_rq_wrptr_db),
        .rq_wrptr_db_we              (rx_pkt_hndler_o_qpm_rw_rq_wrptr_db),


        .o_qp_rsp_conf_idx           (rx_pkt_hndler_o_qp_rsp_conf_idx),
        .i_qp_rsp_conf               (qp_mgr_o_qp_conf),
        .qp_rsp_conf_vld             (qp_mgr_o_qp_conf_valid_rx_pkt),
        .qp_rsp_conf_en              (rx_pkt_hndler_o_qp_rsp_conf_en),
        .i_stat_nak                  (resp_hndler_o_qp_stat_nak[9:0]),
        .stat_nak_en                 (rx_pkt_hndler_o_qpm_req_stat_nak),
        .stat_nak_we                 (rx_pkt_hndler_o_qpm_rw_stat_nak),
        .o_stat_nak                  (rx_pkt_hndler_o_stat_nak),
        .i_stat_resp_psn             (qp_mgr_o_qp_stat_resp_psn),
        .stat_resp_psn_en            (rx_pkt_hndler_o_qpm_req_stat_resp_psn),
        .stat_resp_psn_we            (rx_pkt_hndler_o_qpm_rw_stat_resp_psn),
        .o_stat_resp_psn             (rx_pkt_hndler_o_qp_stat_resp_psn),
        .i_sq_psn                    (qp_mgr_o_qp_sq_psn),
        .sq_psn_en                   (rx_pkt_hndler_o_qpm_req_sq_psn),
        .i_qp_rsp_remote_qp_mac_addr ({qp_mgr_o_qp_mac_dest_addrm[15:0], qp_mgr_o_qp_mac_dest_addrl}),
        .qp_rsp_remote_qp_mac_addr_en(rx_pkt_hndler_rsp_remote_qp_mac_addr_en),
        .i_qp_rsp_remote_qp_ip_addr  ({qp_mgr_o_qp_ip_dest_addr4, qp_mgr_o_qp_ip_dest_addr3, qp_mgr_o_qp_ip_dest_addr2, qp_mgr_o_qp_ip_dest_addr1}),
        .qp_rsp_remote_qp_ip_addr_en (rx_pkt_hndler_rsp_remote_qp_ip_addr_en),
        .o_qp_fatal_timer_idle       (rx_pkt_hndler_o_qp_fatal_timer_idle),
        .i_qp_timeout                (qp_mgr_o_qp_timeout_per_qp),
        .rx_req_axi_awid             (rx_pkt_hndler_ddr_m_axi_awid),
        .rx_req_axi_awaddr           (rx_pkt_hndler_ddr_m_axi_awaddr),
        .rx_req_axi_awuser           (rx_pkt_hndler_ddr_m_axi_awuser),
        .rx_req_axi_awlen            (rx_pkt_hndler_ddr_m_axi_awlen),
        .rx_req_axi_awsize           (rx_pkt_hndler_ddr_m_axi_awsize),
        .rx_req_axi_awburst          (rx_pkt_hndler_ddr_m_axi_awburst),
        .rx_req_axi_awcache          (rx_pkt_hndler_ddr_m_axi_awcache),
        .rx_req_axi_awprot           (rx_pkt_hndler_ddr_m_axi_awprot),
        .rx_req_axi_awvalid          (rx_pkt_hndler_ddr_m_axi_awvalid),
        .rx_req_axi_awready          (rx_pkt_hndler_ddr_m_axi_awready),
        .rx_req_axi_wdata            (rx_pkt_hndler_ddr_m_axi_wdata),
        .rx_req_axi_wstrb            (rx_pkt_hndler_ddr_m_axi_wstrb),
        .rx_req_axi_wlast            (rx_pkt_hndler_ddr_m_axi_wlast),
        .rx_req_axi_wvalid           (rx_pkt_hndler_ddr_m_axi_wvalid),
        .rx_req_axi_wready           (rx_pkt_hndler_ddr_m_axi_wready),
        .rx_req_axi_awlock           (rx_pkt_hndler_ddr_m_axi_awlock),
        .rx_req_axi_bid              (rx_pkt_hndler_ddr_m_axi_bid),
        .rx_req_axi_bresp            (rx_pkt_hndler_ddr_m_axi_bresp),
        .rx_req_axi_bvalid           (rx_pkt_hndler_ddr_m_axi_bvalid),
        .rx_req_axi_bready           (rx_pkt_hndler_ddr_m_axi_bready),
        .rx_req_axi_arid             (rx_pkt_hndler_ddr_m_axi_arid),
        .rx_req_axi_araddr           (rx_pkt_hndler_ddr_m_axi_araddr),
        .rx_req_axi_arlen            (rx_pkt_hndler_ddr_m_axi_arlen),
        .rx_req_axi_arsize           (rx_pkt_hndler_ddr_m_axi_arsize),
        .rx_req_axi_arburst          (rx_pkt_hndler_ddr_m_axi_arburst),
        .rx_req_axi_arcache          (rx_pkt_hndler_ddr_m_axi_arcache),
        .rx_req_axi_arprot           (rx_pkt_hndler_ddr_m_axi_arprot),
        .rx_req_axi_arvalid          (rx_pkt_hndler_ddr_m_axi_arvalid),
        .rx_req_axi_arready          (rx_pkt_hndler_ddr_m_axi_arready),
        .rx_req_axi_rid              (rx_pkt_hndler_ddr_m_axi_rid),
        .rx_req_axi_rdata            (rx_pkt_hndler_ddr_m_axi_rdata),
        .rx_req_axi_rresp            (rx_pkt_hndler_ddr_m_axi_rresp),
        .rx_req_axi_rlast            (rx_pkt_hndler_ddr_m_axi_rlast),
        .rx_req_axi_rvalid           (rx_pkt_hndler_ddr_m_axi_rvalid),
        .rx_req_axi_rready           (rx_pkt_hndler_ddr_m_axi_rready),
        .rx_req_axi_arlock           (rx_pkt_hndler_ddr_m_axi_arlock),
        .rx_rsp_axi_awid             (rx_pkt_hndler_rdrsp_m_axi_awid),
        .rx_rsp_axi_awaddr           (rx_pkt_hndler_rdrsp_m_axi_awaddr),
        .rx_rsp_axi_awlen            (rx_pkt_hndler_rdrsp_m_axi_awlen),
        .rx_rsp_axi_awsize           (rx_pkt_hndler_rdrsp_m_axi_awsize),
        .rx_rsp_axi_awburst          (rx_pkt_hndler_rdrsp_m_axi_awburst),
        .rx_rsp_axi_awcache          (rx_pkt_hndler_rdrsp_m_axi_awcache),
        .rx_rsp_axi_awprot           (rx_pkt_hndler_rdrsp_m_axi_awprot),
        .rx_rsp_axi_awvalid          (rx_pkt_hndler_rdrsp_m_axi_awvalid),
        .rx_rsp_axi_awready          (rx_pkt_hndler_rdrsp_m_axi_awready),
        .rx_rsp_axi_wdata            (rx_pkt_hndler_rdrsp_m_axi_wdata),
        .rx_rsp_axi_wstrb            (rx_pkt_hndler_rdrsp_m_axi_wstrb),
        .rx_rsp_axi_wlast            (rx_pkt_hndler_rdrsp_m_axi_wlast),
        .rx_rsp_axi_wvalid           (rx_pkt_hndler_rdrsp_m_axi_wvalid),
        .rx_rsp_axi_wready           (rx_pkt_hndler_rdrsp_m_axi_wready),
        .rx_rsp_axi_awlock           (rx_pkt_hndler_rdrsp_m_axi_awlock),
        .rx_rsp_axi_bid              (rx_pkt_hndler_rdrsp_m_axi_bid),
        .rx_rsp_axi_bresp            (rx_pkt_hndler_rdrsp_m_axi_bresp),
        .rx_rsp_axi_bvalid           (rx_pkt_hndler_rdrsp_m_axi_bvalid),
        .rx_rsp_axi_bready           (rx_pkt_hndler_rdrsp_m_axi_bready),
        .rx_rsp_axi_arid             (rx_pkt_hndler_rdrsp_m_axi_arid),
        .rx_rsp_axi_araddr           (rx_pkt_hndler_rdrsp_m_axi_araddr),
        .rx_rsp_axi_arlen            (rx_pkt_hndler_rdrsp_m_axi_arlen),
        .rx_rsp_axi_arsize           (rx_pkt_hndler_rdrsp_m_axi_arsize),
        .rx_rsp_axi_arburst          (rx_pkt_hndler_rdrsp_m_axi_arburst),
        .rx_rsp_axi_arcache          (rx_pkt_hndler_rdrsp_m_axi_arcache),
        .rx_rsp_axi_arprot           (rx_pkt_hndler_rdrsp_m_axi_arprot),
        .rx_rsp_axi_arvalid          (rx_pkt_hndler_rdrsp_m_axi_arvalid),
        .rx_rsp_axi_arready          (rx_pkt_hndler_rdrsp_m_axi_arready),
        .rx_rsp_axi_rid              (rx_pkt_hndler_rdrsp_m_axi_rid),
        .rx_rsp_axi_rdata            (rx_pkt_hndler_rdrsp_m_axi_rdata),
        .rx_rsp_axi_rresp            (rx_pkt_hndler_rdrsp_m_axi_rresp),
        .rx_rsp_axi_rlast            (rx_pkt_hndler_rdrsp_m_axi_rlast),
        .rx_rsp_axi_rvalid           (rx_pkt_hndler_rdrsp_m_axi_rvalid),
        .rx_rsp_axi_rready           (rx_pkt_hndler_rdrsp_m_axi_rready),
        .rx_rsp_axi_arlock           (rx_pkt_hndler_rdrsp_m_axi_arlock),

        .waiting_for_rq_update          (rx_pkt_hndler_waiting_for_rq_update),
        .waiting_for_rq_update_on_qp_num(rx_pkt_hndler_waiting_for_rq_update_on_qp_num),
        .o_all_rd_req_cnt               (o_all_rd_req_cnt),
        .o_good_rd_req_cnt              (o_good_rd_req_cnt),
        .o_dup_rd_req_cnt               (o_dup_rd_req_cnt),
        .debug_ctrl_in                  (qp_mgr_o_rx_pkt_hndl_dbg_ctrl),
        .debug_sts_out                  (rx_pkt_hndler_o_debug_bus),
        .i_global_dbg_cnt_clr           (o_global_dbg_cnt_clr),
        .i_global_dbg_cnt_en            (o_global_dbg_cnt_en),
        .i_xrnic_pause_on               (roce_pause_on_offset),
        .i_xrnic_pause_off              (roce_pause_off_offset),
        .tx_pause_req_on                (tx_pause_req_on),
        .ieth_immdt_axis_tvalid         (ieth_immdt_axis_tvalid),
        .ieth_immdt_axis_tlast          (ieth_immdt_axis_tlast),
        .ieth_immdt_axis_tdata          (ieth_immdt_axis_tdata),
        .ieth_immdt_axis_trdy           (ieth_immdt_axis_trdy),
        .init_start                     (init_start),
        .init_count                     (init_count)
    );

    resp_handler_top #(
        .NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN  (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN),
        .NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING(NVMOF_SYSTEM_ONLY_FEATURE_FATAL_HANDLING),
        .NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE    (NVMOF_SYSTEM_ONLY_DDRLESS_RETRY_MODE),
        .C_M_AXI_ADDR_WIDTH                      (C_M_AXI_ADDR_WIDTH),
        .C_NUM_QP                                (C_NUM_QP),
        .C_QP_INDX_WIDTH                         (C_QP_INDX_WIDTH),
        .C_OS_Q_DEPTH                            (OS_Q_DEPTH),
        .C_OS_Q_INDX_WIDTH                       (OS_Q_INDX_WIDTH),
        .C_NUM_CYCLES_4US                        (NUM_CYCLES_4US),                            // Number of clock cycles that make 4 us (for 125Mhz clk period it is 512)
        .C_EN_NVMOF_HW_HNDSHK                    (C_EN_NVMOF_HW_HNDSHK),
        .STB_WIDTH                               (4),
        .C_OSQ_PSN_WIDTH                         (C_OSQ_PSN_WIDTH),
        .C_ECN_THRESHOLD                         (C_ECN_THRESHOLD),
        //.C_EN_WR_RETRY_DATA_BUF	    (C_EN_WR_RETRY_DATA_BUF),
        .RESP_WIDTH                              (2)
    ) inst_resp_handler (
        .core_clk (m_axi_aclk),         // Module Clock  
        .core_rstn(core_rstn_resp_reg), // Active low core reset

        // Interface to the WQE processor to the outstanding buffers
        // WQE 处理器 与 未完成缓冲区(OSQ) 的接口
        .i_wqe_qpid                       (wqe_proc_os_wqe_qpid),              // QP ID of WQE
        .i_wqe_opcode                     (wqe_proc_os_wqe_opcode),            // BTH.Opcode of outbound request
        .i_wqe_wrid                       (wqe_proc_os_wqe_wrid),
        .i_wqe_data_bufid                 (wqe_proc_os_wqe_data_bufid),
        .i_wqe_retried                    (wqe_proc_os_wqe_retried),
        .i_wqe_fatal_push                 (wqe_proc_os_wqe_fatal_push),
        .i_wqe_send_end_psn               (wqe_proc_os_wqe_send_psn),          // PSN of most recent request
        .i_wqe_send_start_psn             (wqe_proc_os_wqe_send_first_psn),    // PSN of most recent request
        .i_wqe_send_msn                   (wqe_proc_os_wqe_send_msn),
        .i_wqe_rdma_rd_rcv_ba             (wqe_proc_os_wqe_rdma_rd_rcv_ba),    // If request is RDMA READ, receive buffer base address
        .i_wqe_rdma_rd_res_len            (wqe_proc_os_wqe_rdma_rd_res_len),   // If request is RDMA READ, response length in bytes
        .i_wqe_push_req                   (wqe_proc_os_wqp_req),               // request from WQE Processor to push WQE
        .i_wqe_push_req_Read              (wqe_proc_os_wqp_req_Read),
        .o_wqe_push_rdy                   (wqe_proc_os_wqp_rdy),               // response to WQE Processor; 0 - busy; 1 - completed  
        .i_wqe_exp_ack_set                (wqe_proc_os_wqe_explicit_ack_req),
        .i_osq_wqe_nack_resp_valid        (osq_nacked_resp_wr_en),
        .i_osq_wqe_nack_resp_id           (osq_nacked_resp_wr_addr),
        .i_osq_wqe_nack_resp_data         (osq_nacked_resp_wr_data),
        .i_osq_wqe_nack_resp_valid_aux    (osq_nacked_resp_wr_en_aux),
        .i_osq_wqe_nack_resp_id_aux       (osq_nacked_resp_wr_addr_aux),
        .i_osq_wqe_nack_resp_data_aux     (osq_nacked_resp_wr_data_aux),
        .i_osq_wqe_nack_resp_valid_pending(osq_nacked_resp_wr_en_pending),
        .i_osq_wqe_nack_resp_id_pending   (osq_nacked_resp_wr_addr_pending),
        .i_osq_wqe_nack_resp_data_pending (osq_nacked_resp_wr_data_pending),
        //.o_osq_full                       (wqe_proc_os_fifo_full),                     // Oustanding Q full - 1 bit for each QP
        .o_osq_full_set_valid             (osq_full_set_valid),
        .o_osq_full_set_id                (osq_full_set_id),
        .o_osq_full_reset_valid           (osq_full_reset_valid),
        .o_osq_full_reset_id              (osq_full_reset_id),
        .o_freeup_data_buf                (resp_hndler_freeup_data_buf),       // Freeup RDMA WRITE DATA buffer
        .o_freeup_data_bufid              (resp_hndler_freeup_data_bufid),     // BUffer ID to be freed up

        .o_resp_hndler_sts (resp_hndler_o_resp_hndler_sts),
        .o_stat_retry_count(resp_hndler_o_stat_retry_count),

        //.i_qp_disable_pulse               (qp_mgr_o_qp_disable_pulse), // whenever QP_enable bit is written this pulse is generated and used for first_outgoing_pkt_ff signal generation 
        .i_qp_disable_pulse_set_valid       (qp_disable_pulse_valid && qp_disable_pulse_data),
        .i_qp_disable_pulse_set_id          (qp_disable_pulse_id),
        .i_cfg_retry_cnt_fatal_dis          (qp_mgr_o_xrnic_adv_conf_retry_cnt_fatal_dis),
        .i_bypass_en                        (qp_mgr_o_depkt_bypass_en),
        .i_qp_fatal_valid                   (rx_pkt_hndler_o_qp_fatal_wr_en),
        .i_qp_fatal_id                      (rx_pkt_hndler_o_qp_fatal_wr_idx),
        .i_qp_fatal_data                    (rx_pkt_hndler_o_qp_fatal_wr_data),
        .i_dma_in_idle                      (wqe_proc_dma_in_idle),
        .i_en_osq_flush_on_qp_fatal         (rx_pkt_hndler_o_qp_fatal_timer_idle),              //TODO, For fixing the completion before Read Resp data write to DDR.
        .i_cfg_base_cnt                     (qp_mgr_o_xrnic_adv_base_cnt),
        .o_num_valid_osq_entries            (os_num_vld_entries),
        .i_qpid_valid_entries               (wqe_proc_top_o_qp_id),
        .o_osq_nacked                       (o_osq_nacked),
        .o_osq_rnr_nacked                   (osq_rnr_nacked),
        // Completion interrupt
        .i_intr_en_wqe_cmpl                 (qp_mgr_o_intr_en_wqe_cmpl),
        .i_intr_clr_wqe_cmpl                (qp_mgr_o_intr_clr_wqe_cmpl),
        .o_intr_wqe_cmpl                    (resp_hndler_o_intr_wqe_cmpl),
        //.o_intr_wqe_cmpl_sts              (resp_hndlr_i_qp_wq_cmpl_intr_sts),
        .o_intr_wqe_cmpl_sts_or_reduce      (intr_wqe_cmpl_sts_or_reduce),
        .o_intr_wqe_cmpl_sts_wr_en          (intr_wqe_cmpl_sts_wr_en),
        .o_intr_wqe_cmpl_sts_wr_addr        (intr_wqe_cmpl_sts_wr_addr),
        .o_intr_wqe_cmpl_sts_wr_data        (intr_wqe_cmpl_sts_wr_data),
        .o_intr_wqe_cmpl_sts_wr_en_pending  (intr_wqe_cmpl_sts_wr_en_pending),
        .o_intr_wqe_cmpl_sts_wr_addr_pending(intr_wqe_cmpl_sts_wr_addr_pending),
        .i_clr_wqe_cmpl_sts_set_valid       (clr_wqe_cmpl_sts_set_valid),
        .i_clr_wqe_cmpl_sts_set_id          (clr_wqe_cmpl_sts_set_id),
        .i_tx_path_congestion_valid         (i_tx_path_congestion_valid),
        .i_tx_path_congestion_id            (i_tx_path_congestion_id),
        .i_tx_path_congestion_data          (i_tx_path_congestion_data),

        // Interface to the RX packet handler - MAX PSN buffer 
        .i_qp_index_mpsnbuf         (rx_pkt_hndler_o_qp_index_mpsnbuf),  // QP ID to read the max PSN buffer top entry
        .i_mpsn_buf_req             (rx_pkt_hndler_o_max_epsn_req),
        .o_mpsn_buf_valid           (resp_hndler_o_mpsn_buf_valid),
        .o_max_epsn                 (resp_hndler_o_max_epsn),            // Max expected response PSN
        .o_trnsfr_length            (resp_hndler_o_trnsfr_length),       // Transfer length for the WQE
        .o_local_addr               (resp_hndler_o_local_addr),          // Local address - relevant only for RDMA READ requests
        .i_mpsnbuf_pop              (rx_pkt_hndler_o_mpsnbuf_pop),       // Pop the MAX PSN buffer
        //.o_mpsnbuf_empty                  (resp_hndler_o_mpsnbuf_empty),               // OSQ is empty - one hot
        .o_mpsnbuf_empty_set_valid  (mpsnbuf_empty_set_valid),
        .o_mpsnbuf_empty_set_id     (mpsnbuf_empty_set_id),
        .o_mpsnbuf_empty_reset_valid(mpsnbuf_empty_reset_valid),
        .o_mpsnbuf_empty_reset_id   (mpsnbuf_empty_reset_id),

        // ACKED 数据包的 RX 数据包处理程序的接口
        .i_qp_index_acknack(rx_pkt_hndler_o_qp_index_acknack),   // QP ID for ack_nack packet PSN
        .i_acknack_valid   (rx_pkt_hndler_o_acknack_valid),      // Valid signal qualifying the qp_index and acked PSN
        .i_acknacked_psn   (rx_pkt_hndler_o_acknacked_psn),      // PSN for the packet being acked/nacked
        .i_acknacked_msn   (rx_pkt_hndler_o_acknacked_msn),      // PSN for the packet being acked/nacked
        .i_acknacked_opcode(rx_pkt_hndler_o_acknacked_opcode),   // Asserted when the packet is nacked 
        .i_pkt_nacksyndrome(rx_pkt_hndler_o_acknacked_syndrome), // NACK Syndrome. Valid when the i_pkt_nack is asserted

        .i_rd_rsp_wr_done  (rx_pkt_hndler_o_rd_rsp_wr_cmpltd),
        .i_rd_rsp_qpid     (rx_pkt_hndler_o_rsp_qp_id),
        .i_rd_rsp_dest_addr(rx_pkt_hndler_o_rsp_dest_addr),
        // Status output
        .o_inc_nack_cnt    (resp_hndler_o_inc_nack_cnt),

        // ---- 与QP管理器的接口，用于重新传输
        .o_retransmit_qpid            (resp_hndler_o_retransmit_qpid),          // 本次重传需要 QP ID
        .o_retransmit_reqd            (resp_hndler_o_retransmit_reqd),          // Retransmission is required
        .o_retransmit_wqe_bram_din    (resp_hndler_o_retransmit_wqe_bram_din),
        .i_retransmit_accepted        (qp_mgr_o_retransmit_accepted),           // Ack from QP manager that the retransmission is initiated
        .o_psn_to_retry               (resp_hndler_o_psn_to_retry),
        .o_ssn_to_retry               (resp_hndler_o_ssn_to_retry),
        //.o_osq_almost_full                (resp_hndler_o_osq_almost_full),
        .o_osq_almost_full_set_valid  (osq_almost_full_set_valid),
        .o_osq_almost_full_set_id     (osq_almost_full_set_id),
        .o_osq_almost_full_reset_valid(osq_almost_full_reset_valid),
        .o_osq_almost_full_reset_id   (osq_almost_full_reset_id),
        //.o_osq_empty                      (resp_hndler_o_osq_empty),
        .o_osq_empty_set_valid        (osq_empty_set_valid),
        .o_osq_empty_set_id           (osq_empty_set_id),
        .o_osq_empty_reset_valid      (osq_empty_reset_valid),
        .o_osq_empty_reset_id         (osq_empty_reset_id),
        .o_osq_timeout_set_valid      (osq_timeout_set_valid),
        .o_osq_timeout_set_id         (osq_timeout_set_id),
        //.o_qp_retried                     (resp_hndler_o_qp_retried),
        .o_qp_retried_set_valid       (qp_retried_set_valid),
        .o_qp_retried_set_id          (qp_retried_set_id),
        .o_qp_retried_reset_valid     (qp_retried_reset_valid),
        .o_qp_retried_reset_id        (qp_retried_reset_id),
        .i_halt                       (qp_mgr_o_halt),
        .i_halted_qpid                (qp_mgr_o_halted_qpid),


        // INterface to QP manager for QP configirations  
        .o_qp_conf_qp_idx         (resp_hndler_o_qp_conf_qp_idx),
        .o_qp_cq_ba_req           (resp_hndler_o_qp_cq_ba_req),
        .i_qp_cq_ba               (qp_mgr_o_qp_cq_ba),                    // CQ base address
        .i_qp_cq_ba_valid         (qp_mgr_o_qp_cq_ba_valid),
        .o_qp_cq_depth_req        (resp_hndler_o_qp_cq_depth_req),
        .i_qp_cq_depth_valid      (qp_mgr_o_qp_cq_depth_valid),           // CQ depth
        .i_qp_cq_depth            (qp_mgr_o_qp_cq_depth),
        .o_qp_sq_cmpldb_addr_req  (resp_hndler_o_qp_sq_cmpldb_addr_req),
        .i_qp_sq_cmpldb_addr_valid(qp_mgr_o_qp_sq_cmpldb_add_valid),      // SQ Completion DB address (points to RDMAif register)
        .i_qp_sq_cmpldb_addr      (qp_mgr_o_qp_sq_cmpldb_add),

        .i_qp_sq_psn_resp_valid(qp_mgr_o_qp_sq_psn_resp_valid),
        .i_qp_sq_psn           (qp_mgr_o_qp_sq_psn),
        .o_qp_sq_psn_resp_req  (resp_hndler_o_qp_sq_psn_resp_req),
        .o_qp_sq_psn_resp_idx  (resp_hndler_o_qp_sq_psn_resp_idx),

        .o_qp_conf_req  (resp_hndler_o_qp_conf_req),
        .i_qp_conf_valid(qp_mgr_o_qp_conf_valid_resp_hndl),
        .i_qp_conf      (qp_mgr_o_qp_conf),

        .o_qp_cq_hdptr_req  (resp_hndler_o_qp_cq_hdptr_req),
        .i_qp_cq_hdptr_valid(qp_mgr_o_qp_cq_hdptr_valid),     // CQ head pointer
        .i_qp_cq_hdptr      (qp_mgr_o_qp_cq_hdptr),
        .o_qp_cq_hdptr      (resp_hndler_o_qp_cq_hdptr),
        .o_qp_cq_hdptr_wrn  (resp_hndler_o_qp_cq_hdptr_wrn),
        .o_qp_cqe_proc_qpid (resp_hndler_o_qp_cqe_proc_qpid),

        .i_num_elements_enabled (qp_mgr_o_num_qp_en),
        .i_timer_loadval_wqe    (wqe_proc_o_wqe_qp_to),
        .i_timer_loadval_acknack(rx_pkt_hndler_o_qp_timeout_per_qp[4:0]),

        .o_send_cq_db_cnt_valid(resp_hndler_o_send_cq_db_cnt_valid),
        .o_send_cq_db_addr     (resp_hndler_o_send_cq_db_addr_int),
        .o_send_cq_db_cnt      (resp_hndler_o_send_cq_db_cnt),
        .i_send_cq_db_rdy      (resp_hndler_i_send_cq_db_rdy),

        //QP STAT register IF
        .i_qp_curr_retry_cnt       (resp_handler_i_qp_retry_cnt_curr),
        .o_qp_curr_retry_cnt_rd_req(resp_handler_o_qp_retry_cnt_req),
        .o_qp_curr_retry_cnt_qpid  (resp_handler_o_qp_retry_cnt_qpid),
        .i_qp_curr_retry_cnt_rd_rdy(resp_handler_i_qp_retry_cnt_rdy),

        .o_qp_curr_retry_cnt_wr_req(resp_handler_o_qp_retry_cnt_wen),
        .i_qp_curr_retry_cnt_wr_rdy(resp_handler_i_qp_retry_cnt_rdy),
        //.o_qp_curr_retry_cnt_qpid	(),
        .o_qp_curr_retry_cnt       (resp_handler_o_qp_retry_cnt_new),

        .o_qp_cfg_retry_cnt_rd_req(resp_handler_o_qp_cfg_retry_cnt_rd_req),
        .i_qp_cfg_retry_cnt_rd_rdy(resp_handler_i_qp_cfg_retry_cnt_rd_rdy),
        .i_qp_cfg_retry_cnt       (qp_mgr_o_qp_timeout_per_qp[10:8]),

        //QP fatal IF for retry limit exceed case
        .o_qp_retry_fatal_req (resp_handler_o_qp_retry_fatal_req),
        .o_qp_retry_fatal_qpid(resp_handler_o_qp_retry_fatal_qpid),
`ifdef RETRY_CNT_QP_FATAL_DIS
        .i_qp_retry_fatal_rdy (1'b1),                                //TODO resp_handler_i_qp_retry_fatal_rdy ),
`else
        .i_qp_retry_fatal_rdy (resp_handler_i_qp_retry_fatal_rdy),
`endif
        .o_qp_sq_ba_req       (resp_handler_o_qp_sq_ba_req),
        .i_qp_sq_ba_valid     (qp_mgr_o_qp_sq_ba_valid),
        .i_qp_sq_ba           (qp_mgr_o_qp_sq_ba),

        .i_data_buf_ba(qp_mgr_o_data_buf_ba),
        .i_data_buf_sz(qp_mgr_o_data_buf_sz_buf_sz),

        // AXI Master signals
        // Write address channel
        .m_axi_awid   (resp_hndler_m_axi_awid),
        .m_axi_awaddr (resp_hndler_m_axi_awaddr),
        .m_axi_awlen  (resp_hndler_m_axi_awlen),
        .m_axi_awsize (resp_hndler_m_axi_awsize),
        .m_axi_awburst(resp_hndler_m_axi_awburst),
        .m_axi_awcache(resp_hndler_m_axi_awcache),
        .m_axi_awprot (resp_hndler_m_axi_awprot),
        .m_axi_awvalid(resp_hndler_m_axi_awvalid),
        .m_axi_awready(resp_hndler_m_axi_awready),

        .m_axi_wdata (resp_hndler_m_axi_wdata),
        .m_axi_wstrb (resp_hndler_m_axi_wstrb),
        .m_axi_wlast (resp_hndler_m_axi_wlast),
        .m_axi_wvalid(resp_hndler_m_axi_wvalid),
        .m_axi_wready(resp_hndler_m_axi_wready),
        .m_axi_awlock(resp_hndler_m_axi_awlock),

        .m_axi_bid   (resp_hndler_m_axi_bid),
        .m_axi_bresp (resp_hndler_m_axi_bresp),
        .m_axi_bvalid(resp_hndler_m_axi_bvalid),
        .m_axi_bready(resp_hndler_m_axi_bready),

        .m_axi_arid   (resp_hndler_m_axi_arid),
        .m_axi_araddr (resp_hndler_m_axi_araddr),
        .m_axi_arlen  (resp_hndler_m_axi_arlen),
        .m_axi_arsize (resp_hndler_m_axi_arsize),
        .m_axi_arburst(resp_hndler_m_axi_arburst),
        .m_axi_arcache(resp_hndler_m_axi_arcache),
        .m_axi_arprot (resp_hndler_m_axi_arprot),
        .m_axi_arvalid(resp_hndler_m_axi_arvalid),
        .m_axi_arready(resp_hndler_m_axi_arready),

        .m_axi_rid   (resp_hndler_m_axi_rid),
        .m_axi_rdata (resp_hndler_m_axi_rdata),
        .m_axi_rresp (resp_hndler_m_axi_rresp),
        .m_axi_rlast (resp_hndler_m_axi_rlast),
        .m_axi_rvalid(resp_hndler_m_axi_rvalid),
        .m_axi_rready(resp_hndler_m_axi_rready),
        .m_axi_arlock(resp_hndler_m_axi_arlock),

        .o_osq_pop                  (resp_handler_osq_pop),
        .o_osq_pop_qpid             (resp_handler_osq_pop_qpid),
        .init_start                 (init_start),
        .init_count                 (init_count),
        .i_qp_under_recovery_wr_en  (qp_under_recovery_wr_en),
        .i_qp_under_recovery_wr_idx (qp_under_recovery_wr_idx),
        .i_qp_under_recovery_wr_data(qp_under_recovery_wr_data),
        .i_qp_enable_valid          (qp_enable_wr_en),
        .i_qp_enable_id             (qp_enable_wr_addr),
        .i_qp_enable_data           (qp_enable_wr_data)
    );

    assign rx_pkt_hndler_o_qpm_req_dest_qp_conf = 1'b0;  // rx_pkt_handler never need of dest_qp_conf

    // axil fifo
    xpm_fifo_axil_wrap #(
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
        .dbiterr_rdch      (),                         // 1-bit output: Double Bit Error- Indicates that the ECC
        .dbiterr_wdch      (),                         // 1-bit output: Double Bit Error- Indicates that the ECC
        .m_axi_araddr      (int_w_axi_lite_araddr),    // AXI_ADDR_WIDTH-bit output: ARADDR: The read address bus gives
        .m_axi_arprot      (),                         // 2-bit output: ARPROT: Indicates the normal, privileged, or
        .m_axi_arvalid     (int_w_axi_lite_arvalid),   // 1-bit output: ARVALID: Indicates that the master is driving a
        .m_axi_awaddr      (int_w_axi_lite_awaddr),    // AXI_ADDR_WIDTH-bit output: AWADDR: The write address bus
        .m_axi_awprot      (),                         // 2-bit output: AWPROT: Indicates the normal, privileged, or
        .m_axi_awvalid     (int_w_axi_lite_awvalid),   // 1-bit output: AWVALID: Indicates that the master is driving a
        .m_axi_bready      (int_w_axi_lite_bready),    // 1-bit output: BREADY: Indicates that the master can accept a
        .m_axi_rready      (int_w_axi_lite_rready),    // 1-bit output: RREADY: Indicates that the master can accept a
        .m_axi_wdata       (int_w_axi_lite_wdata),     // AXI_DATA_WIDTH-bit output: WDATA: The primary payload that is
        .m_axi_wstrb       (int_w_axi_lite_wstrb),     // AXI_DATA_WIDTH-bit output: WSTRB: The byte qualifier that
        .m_axi_wvalid      (int_w_axi_lite_wvalid),    // 1-bit output: WVALID: Indicates that the master is driving a
        .prog_empty_rdch   (),                         // 1-bit output: Programmable Empty- This signal is asserted
        .prog_empty_wdch   (),                         // 1-bit output: Programmable Empty- This signal is asserted
        .prog_full_rdch    (),                         // 1-bit output: Programmable Full: This signal is asserted when
        .prog_full_wdch    (),                         // 1-bit output: Programmable Full: This signal is asserted when
        .rd_data_count_rdch(),                         // RD_DATA_COUNT_WIDTH_RDCH-bit output: Read Data Count- This
        .rd_data_count_wdch(),                         // RD_DATA_COUNT_WIDTH_WDCH-bit output: Read Data Count- This
        .s_axi_arready     (s_axi_lite_arready),       // 1-bit output: ARREADY: Indicates that the slave can accept a
        .s_axi_awready     (s_axi_lite_awready),       // 1-bit output: AWREADY: Indicates that the slave can accept a
        .s_axi_bresp       (s_axi_lite_bresp),         // 2-bit output: BRESP: Write Response. Indicates the status of
        .s_axi_bvalid      (s_axi_lite_bvalid),        // 1-bit output: BVALID: Indicates that the master is driving a
        .s_axi_rdata       (s_axi_lite_rdata),         // AXI_DATA_WIDTH-bit output: RDATA: The primary payload that is
        .s_axi_rresp       (s_axi_lite_rresp),         // 2-bit output: RRESP: Indicates the status of the read
        .s_axi_rvalid      (s_axi_lite_rvalid),        // 1-bit output: RVALID: Indicates that the master is driving a
        .s_axi_wready      (s_axi_lite_wready),        // 1-bit output: WREADY: Indicates that the slave can accept a
        .sbiterr_rdch      (sbiterr_rdch),             // 1-bit output: Single Bit Error- Indicates that the ECC
        .sbiterr_wdch      (sbiterr_wdch),             // 1-bit output: Single Bit Error- Indicates that the ECC
        .wr_data_count_rdch(),                         // WR_DATA_COUNT_WIDTH_RDCH-bit output: Write Data Count: This
        .wr_data_count_wdch(),                         // WR_DATA_COUNT_WIDTH_WDCH-bit output: Write Data Count: This
        .injectdbiterr_rdch(0),                        // 1-bit input: Double Bit Error Injection- Injects a double bit
        .injectdbiterr_wdch(0),                        // 1-bit input: Double Bit Error Injection- Injects a double bit
        .injectsbiterr_rdch(0),                        // 1-bit input: Single Bit Error Injection- Injects a single bit
        .injectsbiterr_wdch(0),                        // 1-bit input: Single Bit Error Injection- Injects a single bit
        .m_aclk            (m_axi_aclk),               // 1-bit input: Master Interface Clock: All signals on master
        .m_axi_arready     (int_w_axi_lite_arready),   // 1-bit input: ARREADY: Indicates that the master can accept a
        .m_axi_awready     (int_w_axi_lite_awready),   // 1-bit input: AWREADY: Indicates that the master can accept a
        .m_axi_bresp       (int_w_axi_lite_bresp),     // 2-bit input: BRESP: Write Response. Indicates the status of
        .m_axi_bvalid      (int_w_axi_lite_bvalid),    // 1-bit input: BVALID: Indicates that the master is driving a
        .m_axi_rdata       (int_w_axi_lite_rdata),     // AXI_DATA_WIDTH-bit input: RDATA: The primary payload that is
        .m_axi_rresp       (int_w_axi_lite_rresp),     // 2-bit input: RRESP: Indicates the status of the read
        .m_axi_rvalid      (int_w_axi_lite_rvalid),    // 1-bit input: RVALID: Indicates that the master is driving a
        .m_axi_wready      (int_w_axi_lite_wready),    // 1-bit input: WREADY: Indicates that the master can accept a
        .s_aclk            (s_axi_lite_aclk),          // 1-bit input: Slave Interface Clock: All signals on slave
        .s_aresetn         (s_axi_lite_aresetn_eval),  // 1-bit input: Active low asynchronous reset.
        .s_axi_araddr      (s_axi_lite_araddr),        // AXI_ADDR_WIDTH-bit input: ARADDR: The read address bus gives
        .s_axi_arprot      (2'b0),                     // 2-bit input: ARPROT: Indicates the normal, privileged, or
        .s_axi_arvalid     (s_axi_lite_arvalid),       // 1-bit input: ARVALID: Indicates that the master is driving a
        .s_axi_awaddr      (s_axi_lite_awaddr),        // AXI_ADDR_WIDTH-bit input: AWADDR: The write address bus gives
        .s_axi_awprot      (2'b0),                     // 2-bit input: AWPROT: Indicates the normal, privileged, or
        .s_axi_awvalid     (s_axi_lite_awvalid),       // 1-bit input: AWVALID: Indicates that the master is driving a
        .s_axi_bready      (s_axi_lite_bready),        // 1-bit input: BREADY: Indicates that the slave can accept a
        .s_axi_rready      (s_axi_lite_rready),        // 1-bit input: RREADY: Indicates that the slave can accept a
        .s_axi_wdata       (s_axi_lite_wdata),         // AXI_DATA_WIDTH-bit input: WDATA: The primary payload that is
        .s_axi_wstrb       (s_axi_lite_wstrb),         // AXI_DATA_WIDTH-bit input: WSTRB: The byte qualifier that
        .s_axi_wvalid      (s_axi_lite_wvalid)         // 1-bit input: WVALID: Indicates that the master is driving a
    );

    qp_mgr_top #(
        .C_S_AXI_LITE_ADDR_WIDTH(C_S_AXI_LITE_ADDR_WIDTH),
        .C_S_AXI_LITE_DATA_WIDTH(32),
        .C_M_AXI_ADDR_WIDTH     (C_M_AXI_ADDR_WIDTH),
        .C_NUM_QP               (C_NUM_QP),
        .C_QP_INDX_WIDTH        (C_QP_INDX_WIDTH),
        .C_NUM_PD               (C_NUM_PD),
        .C_PD_INDX_WIDTH        (C_PD_INDX_WIDTH),
        .STB_WIDTH              (4),
        .RESP_WIDTH             (2),
        .C_EN_DEBUG_REGS        (C_EN_DEBUG_PORTS),
        .C_OS_Q_DEPTH           (OS_Q_DEPTH),
        .C_OS_Q_INDX_WIDTH      (OS_Q_INDX_WIDTH),
        .C_ECN_THRESHOLD        (C_ECN_THRESHOLD)
    ) inst_qp_mgr (
        .s_axi_lite_aclk   (m_axi_aclk),
        .s_axi_lite_aresetn(core_rstn_qp_reg),

        .s_axi_lite_awaddr (int_w_axi_lite_awaddr),
        .s_axi_lite_awready(int_w_axi_lite_awready),
        .s_axi_lite_awvalid(int_w_axi_lite_awvalid),

        .s_axi_lite_araddr (int_w_axi_lite_araddr),
        .s_axi_lite_arready(int_w_axi_lite_arready),
        .s_axi_lite_arvalid(int_w_axi_lite_arvalid),

        .s_axi_lite_wdata (int_w_axi_lite_wdata),
        .s_axi_lite_wstrb (int_w_axi_lite_wstrb),
        .s_axi_lite_wready(int_w_axi_lite_wready),
        .s_axi_lite_wvalid(int_w_axi_lite_wvalid),

        .s_axi_lite_rdata (int_w_axi_lite_rdata),
        .s_axi_lite_rresp (int_w_axi_lite_rresp),
        .s_axi_lite_rready(int_w_axi_lite_rready),
        .s_axi_lite_rvalid(int_w_axi_lite_rvalid),

        .s_axi_lite_bresp (int_w_axi_lite_bresp),
        .s_axi_lite_bready(int_w_axi_lite_bready),
        .s_axi_lite_bvalid(int_w_axi_lite_bvalid),

        .core_clk (m_axi_aclk),         // Module Clock  
        .core_rstn(core_rstn_qp_reg), // Active high core reset

        // Interface to WQE processor for arbitrated WQ entries
        // 用于仲裁 WQE 的 WQE 处理器接口
        .o_wqe        (qpm_wqe_data),          // TBD
        .i_wqe_pop    (qpm_wqe_pop),
        .o_wqe_empty  (qpm_wqe_fifo_empty),
        .o_halt       (qp_mgr_o_halt),
        .o_halted_qpid(qp_mgr_o_halted_qpid),
        .i_wqe_halted (wqe_proc_i_wqe_halted),

        // Interrupt signals coming from Header validation module
        // 来自接头验证模块的中断信号
        .i_pkt_valdn_err_intr      (rx_pkt_hndler_o_ib_pkt_hdr_valdn_err_intr_sts),
        .i_mad_pkt_rcvd_intr       (rx_pkt_hndler_o_mad_pkt_rcvd_intr_sts),
        .i_bypass_pkt_rcvd_intr    (rx_pkt_hndler_o_bypass_pkt_rcvd_intr_sts),
        .i_rnr_nack_gen_intr       (rx_pkt_hndler_o_rnr_nack_gen_intr_sts),
        .i_ill_opc_in_sq_intr      (1'b0),                                           //ill_opc_in_sq_intr),
        .i_qp_pkt_rcvd_intr_wr_en  (rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_en),
        .i_qp_pkt_rcvd_intr_wr_addr(rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_addr),
        .i_qp_pkt_rcvd_intr_wr_data(rx_pkt_hndlr_o_qp_pkt_rcvd_intr_sts_wr_data),
        .i_qp_fatal_err_valid      (rx_pkt_hndler_o_qp_fatal_wr_en),
        .i_qp_fatal_err_id         (rx_pkt_hndler_o_qp_fatal_wr_idx),
        .i_qp_fatal_err_data       (rx_pkt_hndler_o_qp_fatal_wr_data),
        .i_qp_rnr_nacked           (osq_rnr_nacked),

        .o_intr_clr_pkt_valdn_err  (qp_mgr_o_intr_clr_pkt_valdn_err),
        .o_intr_clr_mad_pkt_rcvd   (qp_mgr_o_intr_clr_mad_pkt_rcvd),
        .o_intr_clr_bypass_pkt_rcvd(qp_mgr_o_intr_clr_bypass_pkt_rcvd),
        .o_intr_clr_rnr_nak_gen    (qp_mgr_o_intr_clr_rnr_nak_gen),
        .o_intr_clr_ill_opc_in_sq  (intr_clr_ill_opc_in_sq),
        .o_intr_clr_qp_pkt_rcvd    (qp_mgr_o_intr_clr_qp_pkt_rcvd),
        .o_intr_clr_wqe_cmpl       (qp_mgr_o_intr_clr_wqe_cmpl),
        .o_intr_clr_fatal_err      (qp_mgr_o_intr_clr_fatal_err),
        .o_qp_clr_fatal_err_valid  (clr_qp_fatal_err),
        .o_qp_clr_fatal_err_id     (clr_qp_fatal_idx),
        .o_qp_clr_fatal_err_data   (),
        .o_qp_under_recovery_valid (qp_under_recovery_wr_en),
        .o_qp_under_recovery_id    (qp_under_recovery_wr_idx),
        .o_qp_under_recovery_data  (qp_under_recovery_wr_data),
        .o_intr_en_wqe_cmpl        (qp_mgr_o_intr_en_wqe_cmpl),
        .o_intr_en_pkt_valdn_err   (qp_mgr_o_intr_en_pkt_valdn_err),
        .o_intr_en_mad_pkt_rcvd    (qp_mgr_o_intr_en_mad_pkt_rcvd),
        .o_intr_en_bypass_pkt_rcvd (qp_mgr_o_intr_en_bypass_pkt_rcvd),
        .o_intr_en_rnr_nak_gen     (qp_mgr_o_intr_en_rnr_nak_gen),
        .o_intr_en_ill_opc_in_sq   (intr_en_ill_opc_in_sq),
        .o_intr_en_fatal_err       (qp_mgr_o_intr_en_fatal_err),
        .o_intr_en_qp_pkt_rcvd     (qp_mgr_o_intr_en_qp_pkt_rcvd),

        //.i_qp_wq_cmpl_intr                (resp_hndlr_i_qp_wq_cmpl_intr_sts),
        .i_qp_wq_cmpl_intr_or_reduce      (intr_wqe_cmpl_sts_or_reduce),
        .i_qp_wq_cmpl_intr_wr_en          (intr_wqe_cmpl_sts_wr_en),
        .i_qp_wq_cmpl_intr_wr_addr        (intr_wqe_cmpl_sts_wr_addr),
        .i_qp_wq_cmpl_intr_wr_data        (intr_wqe_cmpl_sts_wr_data),
        .i_qp_wq_cmpl_intr_wr_en_pending  (intr_wqe_cmpl_sts_wr_en_pending),
        .i_qp_wq_cmpl_intr_wr_addr_pending(intr_wqe_cmpl_sts_wr_addr_pending),
        //.o_rq_intr_sts_clr                (qp_mgr_o_rq_intr_sts_clr),
        .o_rq_intr_sts_clr_wr_en          (clr_qp_pkt_rcvd_intr),
        .o_rq_intr_sts_clr_wr_addr        (clr_qp_pkt_rcvd_intr_wr_addr),
        //.o_cq_intr_sts_clr                (qp_mgr_o_cq_intr_sts_clr),
        .o_cq_intr_sts_clr_wr_en          (clr_wqe_cmpl_sts_set_valid),
        .o_cq_intr_sts_clr_wr_addr        (clr_wqe_cmpl_sts_set_id),

        // Timeout register {11'b0,rnr_nack_tval,5'b0,retry_cnt,3'b0,timeout}
        .o_timeoutreg            (),                                 //qp_mgr_o_qp_timeout),
        .o_qp_disable_pulse_valid(qp_disable_pulse_valid),
        .o_qp_disable_pulse_id   (qp_disable_pulse_id),
        .o_qp_disable_pulse_data (qp_disable_pulse_data),
        .i_qp_timeout_idx        (rx_pkt_hndler_o_qp_rsp_conf_idx),  // Index for reading/writing MSN
        .o_qp_timeout            (qp_mgr_o_qp_timeout_per_qp),       // Expected MSN for Qp incoming messages write data from hdr validation

        // CNP Config on RX path
        //.i_cnp_schedule_info               (rx_pkt_hndlr_o_cnp_sched_reg),
        .i_cnp_schedule_info_wr_en  (rx_pkt_hndlr_o_cnp_sched_wr_en),
        .i_cnp_schedule_info_wr_addr(rx_pkt_hndlr_o_cnp_sched_wr_addr),
        .i_cnp_schedule_info_wr_data(rx_pkt_hndlr_o_cnp_sched_wr_data),
        //.o_cnp_schedule_clr                (rx_pkt_hndlr_i_clr_cnp_sched),
        .o_cnp_schedule_clr_wr_en   (clr_cnp_sched),
        .o_cnp_schedule_clr_wr_addr (clr_cnp_sched_wr_addr),
        .o_cnp_timeout_val          (rx_pkt_hndlr_i_cnp_timeout_reg_1),
        .i_cnp_sched_intr_sts       (rx_pkt_hndlr_o_cnp_sched_intr_sts),
        .o_intr_en_cnp              (rx_pkt_hndlr_i_cnp_sched_intr_en),
        .o_intr_clr_cnp             (rx_pkt_hndlr_i_clr_cnp_sched_intr),
        // CNP Config on TX path
        .i_tx_path_congestion_valid (i_tx_path_congestion_valid),
        .i_tx_path_congestion_id    (i_tx_path_congestion_id),
        .i_tx_path_congestion_data  (i_tx_path_congestion_data),
        .o_ecn_timeout_val          (rx_pkt_hndlr_i_cnp_timeout_reg_2),

        .o_out_errsts_q_ba   (out_errsts_q_ba),
        .o_out_errsts_q_sz   (out_errsts_q_sz),
        .i_out_errsts_q_wrptr(16'h0000),         //out_errsts_q_wrptr),

        .o_in_errsts_q_ba   (qp_mgr_in_errsts_q_ba),
        .o_in_errsts_q_sz   (qp_mgr_in_errsts_q_sz),
        .i_in_errsts_q_wrptr(rx_pkt_hndler_in_errsts_q_wrptr),

        // Status inputs from Header validation module
        .i_rq_full_valid         (rx_pkt_hndler_o_rq_full_wr_en),        // Rcv Q full for QPn
        .i_rq_full_idx           (rx_pkt_hndler_o_rq_full_idx),
        .i_rq_full_data          (rx_pkt_hndler_o_rq_full_val),
        .i_rq_full_reset_valid   (rx_pkt_hndler_o_rq_full_reset_wr_en),
        .i_rq_full_reset_idx     (rx_pkt_hndler_o_rq_full_reset_idx),
        .i_rq_full_reset_data    (rx_pkt_hndler_o_rq_full_reset_val),
        .i_osq_full_set_valid    (osq_full_set_valid),                   // Outstanding Q full for QPn
        .i_osq_full_set_id       (osq_full_set_id),                      // Outstanding Q full for QPn
        .i_osq_full_reset_valid  (osq_full_reset_valid),                 // Outstanding Q full for QPn
        .i_osq_full_reset_id     (osq_full_reset_id),                    // Outstanding Q full for QPn
        .i_osq_empty_set_valid   (osq_empty_set_valid),                  // Outstanding Q empty for QPn
        .i_osq_empty_set_id      (osq_empty_set_id),                     // Outstanding Q empty for QPn
        .i_osq_empty_reset_valid (osq_empty_reset_valid),                // Outstanding Q empty for QPn
        .i_osq_empty_reset_id    (osq_empty_reset_id),                   // Outstanding Q empty for QPn
        .i_qp_retried_set_valid  (qp_retried_set_valid),                 // QP retried for QPn
        .i_qp_retried_set_id     (qp_retried_set_id),
        .i_qp_retried_reset_valid(qp_retried_reset_valid),
        .i_qp_retried_reset_id   (qp_retried_reset_id),

        .i_qp_stat_msn_idx  (rx_pkt_hndler_o_qp_req_conf_idx),      // Index for reading/writing MSN
        .i_qp_stat_msn      (rx_pkt_hndler_o_stat_qp_msn),          // Expected MSN for Qp incoming messages
        .o_qp_stat_msn      (qp_mgr_o_qp_stat_msn),                 // Expected MSN for Qp incoming messages write data from hdr validation
        .i_qp_stat_msn_req  (rx_pkt_hndler_o_qpm_req_stat_qp_msn),
        .i_qp_stat_msn_wrn  (rx_pkt_hndler_o_qpm_rw_stat_qp_msn),
        .o_qp_stat_msn_valid(qp_mgr_o_qp_stat_msn_valid),

        .i_qp_rq_pi_db_idx  (rx_pkt_hndler_o_qp_req_conf_idx),      // Index for reading/writing MSN
        .i_qp_rq_pi_db      (rx_pkt_hndler_o_rq_wrptr_db),          // Expected MSN for Qp incoming messages
        .o_qp_rq_pi_db      (qp_mgr_o_rq_pi_db),                    // Expected MSN for Qp incoming messages write data from hdr validation
        .i_qp_rq_pi_db_req  (rx_pkt_hndler_o_qpm_req_rq_wrptr_db),
        .i_qp_rq_pi_db_wrn  (rx_pkt_hndler_o_qpm_rw_rq_wrptr_db),
        .o_qp_rq_pi_db_valid(qp_mgr_o_qp_rq_pi_db_valid),

        // Retried SQ PSN for Rx packet handler
        // 为 Rx 数据包处理程序 重试 SQ PSN
        .i_qp_stat_ret_sq_psn_idx  (rx_pkt_hndler_o_qp_rsp_conf_idx),   // Index for reading/writing RQ PI DB
        .o_qp_stat_ret_sq_psn      (qp_mgr_o_qp_stat_ret_sq_psn),
        .i_qp_stat_ret_sq_psn_req  (rx_pkt_hndler_o_qp_rsp_conf_en),
        .o_qp_stat_ret_sq_psn_valid(qp_mgr_o_qp_stat_ret_sq_psn_valid),

        // Status input from other modules

        .i_wqe_proc_sts        (wqe_proc_status_out),
        .o_rx_pkt_hndl_dbg_ctrl(qp_mgr_o_rx_pkt_hndl_dbg_ctrl),
        .i_rx_pkt_vld_sts      (rx_pkt_hndler_o_debug_bus),
        .i_resp_hndler_sts     (resp_hndler_o_resp_hndler_sts),
        .i_stat_retry_cnt      (resp_hndler_o_stat_retry_count),
        .i_min_ipg_stat        (rx_pkt_hndler_o_ipg_cntr_min),
        //  .i_ipg_0_4_cnt                    (rx_pkt_hndler_o_ipg_0_5_cntr),
        //  .i_ipg_5_9_cnt                    (rx_pkt_hndler_o_ipg_6_10_cntr),
        //  .i_ipg_10_14_cnt                  (rx_pkt_hndler_o_ipg_11_15_cntr),
        //  .i_ipg_15_19_cnt                  (rx_pkt_hndler_o_ipg_16_20_cntr),

        .i_ipg_0_4_cnt  (32'h0),
        .i_ipg_5_9_cnt  (32'h0),
        .i_ipg_10_14_cnt(32'h0),
        .i_ipg_15_19_cnt(32'h0),


        // Configuration outputs going to other modules (generic)
        // 连接其他模块的配置输出（通用）
        .o_xrnic_en                          (qp_mgr_o_xrnic_en),                                                        // XRNIC Configuration -- [0] ERNIC Enable
        .o_xrnic_adv_conf_errbuf_overwr_en   (qp_mgr_o_xrnic_adv_conf_errbuf_overwr_en),
        .o_xrnic_adv_conf_retry_cnt_fatal_dis(qp_mgr_o_xrnic_adv_conf_retry_cnt_fatal_dis),
        .o_xrnic_adv_base_cnt                (qp_mgr_o_xrnic_adv_base_cnt),
        .o_err_buf_en                        (qp_mgr_o_err_buf_en),
        .o_flow_credits                      (qp_mgr_o_flow_credits),
        .o_tx_ack_gen                        (qp_mgr_o_tx_ack_gen),
        .o_depkt_bypass_en                   (qp_mgr_o_depkt_bypass_en),                                                 // XRNIC interface bypass feature enable
        .o_num_qp_en                         (qp_mgr_o_num_qp_en),                                                       // Number of QPs enabled
        .o_mac_xrnic_addr                    (qp_mgr_o_mac_src_addr),                                                    // Ethernet MAC destination (own) address from XRNIC registers
        .o_ipv4_xrnic_addr                   (qp_mgr_o_ipv4_src_addr),                                                   // IPv4 destination (own) address from XRNIC registers
        .o_ipv6_xrnic_addr                   (qp_mgr_o_ipv6_src_addr),                                                   // IPv6 destination (own) address form XRNIC registers
        .o_data_buf_ba                       (qp_mgr_o_data_buf_ba),
        .o_data_buf_sz_num_bufs              (qp_mgr_o_data_buf_sz_num_bufs),
        .o_data_buf_sz_buf_sz                (qp_mgr_o_data_buf_sz_buf_sz),
        .o_bypass_buf_ba                     (qp_mgr_o_bypass_buf_ba),
        .o_bypass_buf_sz_num_bufs            (qp_mgr_o_bypass_buf_sz_num_bufs),
        .o_bypass_buf_sz_buf_sz              (qp_mgr_o_bypass_buf_sz_buf_sz),
        .i_bypass_buf_wrptr                  (rx_pkt_hndler_o_bypass_buf_wrptr),
        .o_rq_err_pkt_buf_ba                 (qp_mgr_o_err_pkt_buf_ba),
        .o_rq_err_pkt_buf_sz_num_bufs        (qp_mgr_o_err_pkt_buf_sz_num_bufs),
        .o_rq_err_pkt_buf_sz_buf_sz          (qp_mgr_o_err_pkt_buf_sz_buf_sz),
        .i_rq_err_pkt_buf_wrptr              (rx_pkt_hndler_o_err_pkt_buf_wrptr),
        .o_resp_err_pkt_buf_ba               (qp_mgr_o_resp_err_pkt_buf_ba),
        .o_resp_err_pkt_buf_sz_num_bufs      (qp_mgr_o_resp_err_pkt_buf_sz_num_bufs),
        .o_resp_err_pkt_buf_sz_buf_sz        (qp_mgr_o_resp_err_pkt_buf_sz_buf_sz),
        .i_resp_err_pkt_buf_wrptr            (rx_pkt_hndler_o_rsp_err_buf_wrptr),
        .i_inc_rresp_pkt_cnt                 (rx_pkt_hndler_o_ib_rres_pkt_cnt),
        .i_inc_send_pkt_cnt                  (rx_pkt_hndler_o_ib_send_pkt_cnt),
        .i_inc_ack_pkt_cnt                   (rx_pkt_hndler_o_ib_ack_pkt_cnt),
        .i_inc_mad_pkt_cnt                   (rx_pkt_hndler_o_ib_qp1_pkt_cnt),
        .i_inc_inv_pkt_cnt                   (rx_pkt_hndler_o_ib_inv_pkt_cnt),
        .i_inc_dup_pkt_cnt                   (rx_pkt_hndler_o_ib_dupl_pkt_cnt),
        .i_inc_all_dropped_cnt               ({rx_pkt_hndler_o_pkt_all_cnt[15:0], rx_pkt_hndler_o_pkt_disc_cnt[15:0]}),  // TBD
        .i_inc_nack_cnt                      (rx_pkt_hndler_o_inc_nack_cnt),                                             // TBD
        .i_inc_cnp_pkt_cnt                   (rx_pkt_hndlr_o_cnp_rx_pkt_cnt),
        .o_error_corrupt_conf                (qp_mgr_o_error_corrupt_conf),
        .clr_error_corrupt_en                (clr_error_corrupt_en),
        .o_tx_hdr_buf_sz_num_hdrs            (qp_mgr_o_tx_hdr_buf_sz_num_hdrs),
        .o_tx_hdr_buf_sz_buf_sz              (qp_mgr_o_tx_hdr_buf_sz_buf_sz),
        .o_tx_sgl_buf_ba                     (qp_mgr_o_tx_sgl_buf_ba),
        .o_tx_sgl_buf_sz_num_sgls            (qp_mgr_o_tx_sgl_buf_sz_num_sgls),
        .o_tx_sgl_buf_sz_buf_sz              (qp_mgr_o_tx_sgl_buf_sz_buf_sz),
        .o_xrnic_conf_ipver                  (qp_mgr_o_xrnic_conf_ipver),
        .o_qp_enable_valid                   (qp_enable_wr_en),
        .o_qp_enable_id                      (qp_enable_wr_addr),
        .o_qp_enable_data                    (qp_enable_wr_data),
        .o_xrnic_udp_src_port                (qp_mgr_o_xrnic_udp_src_port),

        //.o_connect_io_residual_rq         (qp_mgr_o_connect_io_residual_rq     ),
        .o_rq_pi_db_hw_hndshk         (qp_mgr_o_rq_pi_db_hw_hndshk),
        .o_connect_io_qp_rq_pi_db_wptr(qp_mgr_o_connect_io_qp_rq_pi_db_wptr),
        .o_connect_io_qpid            (qp_mgr_o_connect_io_qpid),
        .i_connect_io_qp_rq_pi_db_rdy (rx_pkt_hndler_o_connect_io_qp_rq_pi_db_rdy),
        .o_hw_hndshk_disable_to_0     (qp_mgr_o_hw_hndshk_disable_to_0),
        .pfc_en_roce                  (pfc_en_roce),
        .pfc_en_non_roce              (pfc_en_non_roce),
        .roce_priority                (roce_priority),
        .non_roce_priority            (non_roce_priority),
        .roce_pause_on_offset         (roce_pause_on_offset),
        .roce_pause_off_offset        (roce_pause_off_offset),
        .non_roce_pause_on_offset     (non_roce_pause_on_offset),
        .non_roce_pause_off_offset    (non_roce_pause_off_offset),
        .disable_priority_chk         (disable_priority_chk),

        //.i_out_nack_cnt                   (resp_hndler_o_inc_nack_cnt), 
        .i_out_nack_cnt       (rx_pkt_hndler_o_out_nack_cnt[15:0]),
        .i_out_rdwr_pkt_cnt   (wqe_out_rdwr_pkt_cnt),
        .i_out_send_pkt_cnt   (wqe_out_send_pkt_cnt),
        .i_out_mad_pkt_cnt    (wqe_out_mad_pkt_cnt),
        .i_out_rd_resp_pkt_cnt(wqe_out_rd_resp_pkt_cnt),
        .i_out_ack_pkt_cnt    (wqe_out_ack_pkt_cnt),
        .i_all_rd_req_cnt     (o_all_rd_req_cnt),
        .i_good_rd_req_cnt    (o_good_rd_req_cnt),
        .i_dup_rd_req_cnt     (o_dup_rd_req_cnt),

        .i_qp_cfg_retry_cnt_rd_req(resp_handler_o_qp_cfg_retry_cnt_rd_req),
        //.i_qp_cfg_retry_cnt_qpid          (), //already connected
        .o_qp_cfg_retry_cnt_rd_rdy(resp_handler_i_qp_cfg_retry_cnt_rd_rdy),

        //Configuration outputs going to other modules (PER QP)
        .i_qp_conf_req_resp_hndl  (resp_hndler_o_qp_conf_req),
        .i_qp_conf_req_wqe_proc   (wqe_proc_top_o_reg_rd_en_s),
        .i_qp_conf_req_rx_pkt     (rx_pkt_hndler_o_qp_rsp_conf_en),
        .o_qp_conf_valid_resp_hndl(qp_mgr_o_qp_conf_valid_resp_hndl),
        .o_qp_conf_valid_wqe_proc (qp_mgr_o_qp_conf_valid_wqe_proc),
        .o_qp_conf_valid_rx_pkt   (qp_mgr_o_qp_conf_valid_rx_pkt),
        .o_qp_conf                (qp_mgr_o_qp_conf),                                                                                                                                      // QP configuration
        .i_qp_conf_idx            (rx_pkt_hndler_o_qp_rsp_conf_en ? rx_pkt_hndler_o_qp_rsp_conf_idx : (wqe_proc_top_o_reg_rd_en_s ? wqe_proc_top_o_qp_id : resp_hndler_o_qp_conf_qp_idx)), // Index for reading Qp configuration

        .o_qp_conf_replica      (qp_mgr_o_qp_conf_replica),         // QP configuration REPLICA
        .i_qp_conf_replica_idx  (rx_pkt_hndler_o_qp_req_conf_idx),  // Index for reading Qp configuration REPLICA
        .i_qp_conf_replica_req  (rx_pkt_hndler_o_qp_req_conf_en),
        .o_qp_conf_replica_valid(qp_mgr_o_qp_conf_replica_valid),

        .i_qp_adv_conf_req  (wqe_proc_top_o_reg_rd_en),    // TBD
        .o_qp_adv_conf_valid(qp_mgr_o_qp_adv_conf_valid),  // TBD
        .o_qp_adv_conf      (qp_mgr_o_qp_adv_conf),        // QP advance configuration
        .i_qp_adv_conf_idx  (wqe_proc_top_o_qp_id),        // TBD

        .i_qp_rq_ba_req  (rx_pkt_hndler_o_qpm_req_rq_buf_ba),
        .o_qp_rq_ba_valid(qp_mgr_o_qp_rq_ba_valid),
        .o_qp_rq_ba      (qp_mgr_o_qp_rq_ba),
        .i_qp_rq_ba_idx  (rx_pkt_hndler_o_qp_req_conf_idx),    // Index for reading Qp RQ Buffer BA

        .i_qp_cq_ba_req  (resp_hndler_o_qp_cq_ba_req),
        .o_qp_cq_ba_valid(qp_mgr_o_qp_cq_ba_valid),
        .o_qp_cq_ba      (qp_mgr_o_qp_cq_ba),
        .i_qp_cq_ba_idx  (resp_hndler_o_qp_cqe_proc_qpid), // Index for reading Qp CQ BA

        // rq_wrptr_db -- RQWPTRDBADDi 寄存器指向 rq_wrptr_db 的地址。
        // RQWPTRDBADDi 寄存器:
        //      该寄存器根据参数 C_NUM_QP 生成（其中 i = 1 至 C_NUM_QP）。
        //      该寄存器提供接收队列门铃寄存器的地址。
        //      在接收到新的 RDMA SEND 数据包时，ERNIC IP 会更新该寄存器所指向地址中的 RQ 门铃值。
        .i_qp_rq_wrptrdb_add_req  (rx_pkt_hndler_o_qpm_req_rq_wrptr_db_add),
        .o_qp_rq_wrptrdb_add_valid(qp_mgr_o_qp_rq_wrptrdb_add_valid),
        .o_qp_rq_wrptrdb_add      (qp_mgr_o_qp_rq_wrptrdb_add),
        .i_qp_rq_wrptrdb_add_idx  (rx_pkt_hndler_o_qp_req_conf_idx),          // Index for reading Qp SQ BA

        .i_qp_sq_cmpldb_add_req  (resp_hndler_o_qp_sq_cmpldb_addr_req),
        .o_qp_sq_cmpldb_add_valid(qp_mgr_o_qp_sq_cmpldb_add_valid),
        .o_qp_sq_cmpldb_add      (qp_mgr_o_qp_sq_cmpldb_add),
        .i_qp_sq_cmpldb_add_idx  (resp_hndler_o_qp_cqe_proc_qpid),       // Index for reading SQ Completion DB address

        .i_qp_cq_hdptr_req  (resp_hndler_o_qp_cq_hdptr_req),
        .o_qp_cq_hdptr_valid(qp_mgr_o_qp_cq_hdptr_valid),
        .o_qp_cq_hdptr      (qp_mgr_o_qp_cq_hdptr),
        .i_qp_cq_hdptr_idx  (resp_hndler_o_qp_cqe_proc_qpid),  // Index for reading CQ head pointer
        .i_qp_cq_hdptr      (resp_hndler_o_qp_cq_hdptr),
        .i_qp_cq_hdptr_wrn  (resp_hndler_o_qp_cq_hdptr_wrn),

        .i_qp_rq_cidb_req  (rx_pkt_hndler_o_qpm_req_rq_ci_db),
        .o_qp_rq_cidb_valid(qp_mgr_o_qp_rq_cidb_valid),
        .o_qp_rq_cidb      (qp_mgr_o_qp_rq_cidb),
        .i_qp_rq_cidb_idx  (rx_pkt_hndler_o_qp_req_conf_idx),   // Index for reading RQ CI DB

        .i_qp_rq_depth_req  (rx_pkt_hndler_o_qpm_req_q_depth),
        .o_qp_rq_depth_valid(qp_mgr_o_qp_rq_depth_valid),
        .o_qp_rq_depth      (qp_mgr_o_qp_rq_depth),
        .i_qp_rq_depth_idx  (rx_pkt_hndler_o_qp_req_conf_idx),  // Index for reading Q depth

        .i_qp_sq_psn_req       (rx_pkt_hndler_o_qpm_req_sq_psn),                         // TBD
        .i_qp_sq_psn_wqe_req   (wqe_proc_top_o_reg_rd_en_s | wqe_proc_top_o_reg_wr_en),  // TBD
        .i_qp_sq_psn_resp_req  (resp_hndler_o_qp_sq_psn_resp_req),                       // TBD
        .o_qp_sq_psn_valid     (qp_mgr_o_qp_sq_psn_valid),                               // TBD
        .o_qp_sq_psn_wqe_valid (qp_mgr_o_qp_sq_psn_wqe_valid),                           // TBD
        .o_qp_sq_psn_resp_valid(qp_mgr_o_qp_sq_psn_resp_valid),                          // TBD
        .o_qp_sq_psn           (qp_mgr_o_qp_sq_psn),
        .i_qp_sq_psn_wrn       (wqe_proc_top_o_reg_wr_en),
        .i_qp_sq_psn           (wqe_proc_top_reg_sq_psn),
        .i_qp_sq_psn_idx       (rx_pkt_hndler_o_qp_rsp_conf_idx),                        // Index for reading Q depth
        .i_qp_sq_psn_wqe_idx   (wqe_proc_top_o_qp_id),                                   // Index for reading Q depth
        .i_qp_sq_psn_resp_idx  (resp_hndler_o_qp_sq_psn_resp_idx),                       // Index for reading Q depth

        .o_qp_last_rq      (qp_mgr_o_qp_last_rq),
        .i_qp_last_rq_idx  (rx_pkt_hndler_o_qp_req_conf_idx),      // Index for reading Q depth
        .i_qp_last_rq      (rx_pkt_hndler_o_last_rq_req),
        .i_qp_last_rq_req  (rx_pkt_hndler_o_qpm_req_last_rq_req),
        .i_qp_last_rq_wrn  (rx_pkt_hndler_o_qpm_rw_last_rq_req),
        .o_qp_last_rq_valid(qp_mgr_o_qp_last_rq_valid),

        .i_qp_pd_num_wrn  (1'b0),
        .o_qp_pd_num      (qp_mgr_o_qp_pd_num),
        .i_qp_pd_num      (24'd0),
        .o_qp_pd_num_valid(),
        .i_qp_pd_num_idx  (rx_pkt_hndler_o_qp_req_conf_idx),
        .i_qp_pd_num_req  (rx_pkt_hndler_o_qpm_req_qp_pd_num),

        .i_pd_pd_num_wrn  (1'b0),
        .o_pd_pd_num      (pd_pd_num),
        .i_pd_pd_num      (24'd0),
        .o_pd_pd_num_valid(),
        .i_pd_pd_num_idx  (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_pd_pd_num_req  (rx_pkt_hndler_o_pd_req_conf_en),

        .i_pd_virt_addr_0_wrn  (1'b0),
        .o_pd_virt_addr_0      (pd_virt_addr_0),
        .i_pd_virt_addr_0      (32'd0),
        .o_pd_virt_addr_0_valid(),
        .i_pd_virt_addr_0_idx  (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_pd_virt_addr_0_req  (rx_pkt_hndler_o_pd_req_conf_en),

        .i_pd_virt_addr_1_wrn  (1'b0),
        .o_pd_virt_addr_1      (pd_virt_addr_1),
        .i_pd_virt_addr_1      (32'd0),
        .o_pd_virt_addr_1_valid(),
        .i_pd_virt_addr_1_idx  (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_pd_virt_addr_1_req  (rx_pkt_hndler_o_pd_req_conf_en),

        .i_pd_buf_base_addr_0_wrn  (1'b0),
        .o_pd_buf_base_addr_0      (pd_buf_base_addr_0),
        .i_pd_buf_base_addr_0      (32'd0),
        .o_pd_buf_base_addr_0_valid(),
        .i_pd_buf_base_addr_0_idx  (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_pd_buf_base_addr_0_req  (rx_pkt_hndler_o_pd_req_conf_en),

        .i_pd_buf_base_addr_1_wrn  (1'b0),
        .o_pd_buf_base_addr_1      (pd_buf_base_addr_1),
        .i_pd_buf_base_addr_1      (32'd0),
        .o_pd_buf_base_addr_1_valid(),
        .i_pd_buf_base_addr_1_idx  (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_pd_buf_base_addr_1_req  (rx_pkt_hndler_o_pd_req_conf_en),

        .i_pd_buf_r_key_wrn  (1'b0),
        .o_pd_buf_r_key      (pd_buf_r_key),
        .i_pd_buf_r_key      (8'd0),
        .o_pd_buf_r_key_valid(),
        .i_pd_buf_r_key_idx  (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_pd_buf_r_key_req  (rx_pkt_hndler_o_pd_req_conf_en),

        .i_pd_w_r_buf_len_wrn  (1'b0),
        .o_pd_w_r_buf_len      (pd_w_r_buf_len),
        .i_pd_w_r_buf_len      (48'd0),
        .o_pd_w_r_buf_len_valid(),
        .i_pd_w_r_buf_len_idx  (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_pd_w_r_buf_len_req  (rx_pkt_hndler_o_pd_req_conf_en),

        .i_pd_access_type_wrn  (1'b0),
        .o_pd_access_type      (pd_access_type),
        .i_pd_access_type      (4'd0),
        .o_pd_access_type_valid(),
        .i_pd_access_type_idx  (rx_pkt_hndler_o_pd_req_conf_idx),
        .i_pd_access_type_req  (rx_pkt_hndler_o_pd_req_conf_en),

        .i_qp_dest_qpid_req_wqe_proc  (wqe_proc_top_o_reg_rd_en),              // req from WQE PROC
        .i_qp_dest_qpid_req_rx_pkt    (1'b0),                                  // req from RX pkt handler
        .o_qp_dest_qpid_valid_wqe_proc(qp_mgr_o_qp_dest_qpid_valid_wqe_proc),
        .o_qp_dest_qpid_valid_rx_pkt  (qp_mgr_o_qp_dest_qpid_valid_rx_pkt),
        .o_qp_dest_qpid               (qp_mgr_o_qp_dest_qpid),
        .i_qp_dest_qpid_idx           (wqe_proc_top_o_qp_id),                  // Index for reading dest QPID 

        .i_qp_stat_resp_psn      (rx_pkt_hndler_o_qp_stat_resp_psn),
        .i_qp_stat_resp_psn_wen  (rx_pkt_hndler_o_qpm_rw_stat_resp_psn),
        .i_qp_stat_resp_psn_idx  (rx_pkt_hndler_o_qp_rsp_conf_idx),
        .o_qp_stat_resp_psn_valid(qp_mgr_o_qp_stat_resp_psn_valid),
        .i_qp_stat_resp_psn_rdreq(rx_pkt_hndler_o_qpm_req_stat_resp_psn),
        .o_qp_stat_resp_psn      (qp_mgr_o_qp_stat_resp_psn),

        .i_qp_stat_rq_buf_ca      (rx_pkt_hndler_o_rq_buf_ca),
        .i_qp_stat_rq_buf_ca_wen  (rx_pkt_hndler_o_qpm_rw_rq_buf_ca),
        .i_qp_stat_rq_buf_ca_idx  (rx_pkt_hndler_o_qp_req_conf_idx),
        .o_qp_stat_rq_buf_ca_valid(qp_mgr_o_qp_stat_rq_buf_ca_valid),
        .i_qp_stat_rq_buf_ca_rdreq(rx_pkt_hndler_o_qpm_req_rq_buf_ca),
        .o_qp_stat_rq_buf_ca      (qp_mgr_o_qp_stat_rq_buf_ca),

        .i_qp_stat_ssn_idx  (wqe_proc_top_o_qp_id),
        .i_qp_stat_ssn      (wqe_proc_top_reg_sq_ssn),
        .o_qp_stat_ssn      (qp_mgr_o_qp_sq_ssn),
        .i_qp_stat_ssn_req  (wqe_proc_top_o_reg_rd_en | wqe_proc_top_o_reg_wr_en),
        .i_qp_stat_ssn_wrn  (wqe_proc_top_o_reg_wr_en),
        .o_qp_stat_ssn_valid(),

        .i_waiting_for_rq_update         (rx_pkt_hndler_waiting_for_rq_update),
        .i_waiting_for_rq_update_on_q_num(rx_pkt_hndler_waiting_for_rq_update_on_qp_num),

        // HW handshake for RQ CI doorbell
        .i_qp_rq_cidb_hndshk         (i_qp_rq_cidb_hndshk),
        .i_qp_rq_cidb_wr_addr_hndshk (i_qp_rq_cidb_wr_addr_hndshk),
        .i_qp_rq_cidb_wr_valid_hndshk(i_qp_rq_cidb_wr_valid_hndshk),
        .o_qp_rq_cidb_wr_rdy         (o_qp_rq_cidb_wr_rdy),
        //-------------------------------  

        .i_qp_sq_pidb_hndshk         (i_qp_sq_pidb_hndshk),
        .i_qp_sq_pidb_wr_addr_hndshk (i_qp_sq_pidb_wr_addr_hndshk),
        .i_qp_sq_pidb_wr_valid_hndshk(i_qp_sq_pidb_wr_valid_hndshk),
        .o_qp_sq_pidb_wr_rdy         (o_qp_sq_pidb_wr_rdy),

        // Status NAK details from RX paket handler
        // 
        //.i_qp_stat_nak                    ({3'b0, rx_pkt_hndler_o_stat_nak}),
        .i_qp_stat_nak      (rx_pkt_hndler_o_stat_nak[9:0]),
        .i_qp_stat_nak_wen  (rx_pkt_hndler_o_qpm_rw_stat_nak),
        .i_qp_stat_nak_idx  (rx_pkt_hndler_o_qp_rsp_conf_idx),
        .o_qp_stat_nak_valid(resp_hndler_o_qp_stat_nak_valid),
        .i_qp_stat_nak_rdreq(rx_pkt_hndler_o_qpm_req_stat_nak),
        .o_qp_stat_nak      (resp_hndler_o_qp_stat_nak),         // Status QPi(STATQPi)寄存器{[30:28], [22:16]}

        // Status NAK details from resp handler
        .i_qp_stat_retry_cnt      (resp_handler_o_qp_retry_cnt_new),
        .i_qp_stat_retry_cnt_wen  (resp_handler_o_qp_retry_cnt_wen),
        .i_qp_stat_retry_cnt_idx  (resp_handler_o_qp_retry_cnt_qpid),
        .o_qp_stat_retry_cnt_valid(resp_handler_i_qp_retry_cnt_rdy),
        .i_qp_stat_retry_cnt_rdreq(resp_handler_o_qp_retry_cnt_req),
        .o_qp_stat_retry_cnt      (resp_handler_i_qp_retry_cnt_curr),  // Status QPi(STATQPi)寄存器[26:24]

        .i_qp_mac_remote_addrl_req      (rx_pkt_hndler_rsp_remote_qp_mac_addr_en),
        .o_qp_mac_remote_addrl_valid    (qp_mgr_o_qp_mac_remote_addrl_valid),
        .i_qp_mac_remote_addrl_wqe_req  (wqe_proc_top_o_reg_rd_en_s),
        .o_qp_mac_remote_addrl_wqe_valid(qp_mgr_o_qp_mac_dest_addrl_valid),
        .o_qp_mac_remote_addrl          (qp_mgr_o_qp_mac_dest_addrl),
        .i_qp_mac_remote_addrl_idx      (rx_pkt_hndler_o_qp_rsp_conf_idx),          // Index for reading Q depth
        .i_qp_mac_remote_addrl_wqe_idx  (wqe_proc_top_o_qp_id),                     // Index for reading Q depth

        .i_qp_mac_remote_addrl_replica_req  (rx_pkt_hndler_req_remote_qp_mac_addr_en),
        .o_qp_mac_remote_addrl_replica      (qp_mgr_o_qp_mac_remote_addrl_replica),
        .i_qp_mac_remote_addrl_replica_idx  (rx_pkt_hndler_o_qp_req_conf_idx),
        .o_qp_mac_remote_addrl_replica_valid(),

        .i_qp_mac_remote_addrm_req      (rx_pkt_hndler_rsp_remote_qp_mac_addr_en),
        .o_qp_mac_remote_addrm_valid    (),
        .i_qp_mac_remote_addrm_wqe_req  (wqe_proc_top_o_reg_rd_en_s),
        .o_qp_mac_remote_addrm_wqe_valid(qp_mgr_o_qp_mac_dest_addrm_valid),
        .o_qp_mac_remote_addrm          (qp_mgr_o_qp_mac_dest_addrm),
        .i_qp_mac_remote_addrm_idx      (rx_pkt_hndler_o_qp_rsp_conf_idx),          // Index for reading Q depth
        .i_qp_mac_remote_addrm_wqe_idx  (wqe_proc_top_o_qp_id),                     // Index for reading Q depth

        .o_qp_mac_remote_addrm_replica      (qp_mgr_o_qp_mac_remote_addrm_replica),
        .i_qp_mac_remote_addrm_replica_idx  (rx_pkt_hndler_o_qp_req_conf_idx),
        .i_qp_mac_remote_addrm_replica_req  (rx_pkt_hndler_req_remote_qp_mac_addr_en),
        .o_qp_mac_remote_addrm_replica_valid(),

        .i_qp_ip_remote_addr1_req      (rx_pkt_hndler_rsp_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr1_valid    (qp_mgr_o_qp_ip_remote_addr1_valid),
        .i_qp_ip_remote_addr1_wqe_req  (wqe_proc_top_o_reg_rd_en_s),
        .o_qp_ip_remote_addr1_wqe_valid(qp_mgr_o_qp_ip_dest_addr1_valid),
        .o_qp_ip_remote_addr1          (qp_mgr_o_qp_ip_dest_addr1),
        .i_qp_ip_remote_addr1_idx      (rx_pkt_hndler_o_qp_rsp_conf_idx),         // Index for reading IP ADDR1
        .i_qp_ip_remote_addr1_wqe_idx  (wqe_proc_top_o_qp_id),                    // Index for reading IP ADDR1

        .o_qp_ip_remote_addr1_replica      (qp_mgr_o_qp_ip_remote_addr1_replica),
        .i_qp_ip_remote_addr1_replica_idx  (rx_pkt_hndler_o_qp_req_conf_idx),
        .i_qp_ip_remote_addr1_replica_req  (rx_pkt_hndler_req_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr1_replica_valid(),

        .i_qp_ip_remote_addr2_req      (rx_pkt_hndler_rsp_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr2_valid    (),
        .i_qp_ip_remote_addr2_wqe_req  (wqe_proc_top_o_reg_rd_en_s),
        .o_qp_ip_remote_addr2_wqe_valid(qp_mgr_o_qp_ip_dest_addr2_valid),
        .o_qp_ip_remote_addr2          (qp_mgr_o_qp_ip_dest_addr2),
        .i_qp_ip_remote_addr2_idx      (rx_pkt_hndler_o_qp_rsp_conf_idx),         // Index for reading IP ADDR2 
        .i_qp_ip_remote_addr2_wqe_idx  (wqe_proc_top_o_qp_id),                    // Index for reading IP ADDR2 

        .o_qp_ip_remote_addr2_replica      (qp_mgr_o_qp_ip_remote_addr2_replica),
        .i_qp_ip_remote_addr2_replica_idx  (rx_pkt_hndler_o_qp_req_conf_idx),
        .i_qp_ip_remote_addr2_replica_req  (rx_pkt_hndler_req_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr2_replica_valid(),

        .i_qp_ip_remote_addr3_req      (rx_pkt_hndler_rsp_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr3_valid    (),
        .i_qp_ip_remote_addr3_wqe_req  (wqe_proc_top_o_reg_rd_en_s),
        .o_qp_ip_remote_addr3_wqe_valid(qp_mgr_o_qp_ip_dest_addr3_valid),
        .o_qp_ip_remote_addr3          (qp_mgr_o_qp_ip_dest_addr3),
        .i_qp_ip_remote_addr3_idx      (rx_pkt_hndler_o_qp_rsp_conf_idx),         // Index for reading IP ADDR3
        .i_qp_ip_remote_addr3_wqe_idx  (wqe_proc_top_o_qp_id),                    // Index for reading IP ADDR3

        .o_qp_ip_remote_addr3_replica      (qp_mgr_o_qp_ip_remote_addr3_replica),
        .i_qp_ip_remote_addr3_replica_idx  (rx_pkt_hndler_o_qp_req_conf_idx),
        .i_qp_ip_remote_addr3_replica_req  (rx_pkt_hndler_req_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr3_replica_valid(),

        .i_qp_sq_ba_req  (resp_handler_o_qp_sq_ba_req),
        .o_qp_sq_ba_valid(qp_mgr_o_qp_sq_ba_valid),
        .o_qp_sq_ba      (qp_mgr_o_qp_sq_ba),
        .i_qp_sq_ba_idx  (resp_hndler_o_qp_cqe_proc_qpid),

        .i_qp_ip_remote_addr4_req      (rx_pkt_hndler_rsp_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr4_valid    (),
        .i_qp_ip_remote_addr4_wqe_req  (wqe_proc_top_o_reg_rd_en_s),
        .o_qp_ip_remote_addr4_wqe_valid(qp_mgr_o_qp_ip_dest_addr4_valid),
        .o_qp_ip_remote_addr4          (qp_mgr_o_qp_ip_dest_addr4),
        .i_qp_ip_remote_addr4_idx      (rx_pkt_hndler_o_qp_rsp_conf_idx),         // Index for reading IP ADDR4
        .i_qp_ip_remote_addr4_wqe_idx  (wqe_proc_top_o_qp_id),                    // Index for reading IP ADDR4

        .o_qp_ip_remote_addr4_replica      (qp_mgr_o_qp_ip_remote_addr4_replica),
        .i_qp_ip_remote_addr4_replica_idx  (rx_pkt_hndler_o_qp_req_conf_idx),
        .i_qp_ip_remote_addr4_replica_req  (rx_pkt_hndler_req_remote_qp_ip_addr_en),
        .o_qp_ip_remote_addr4_replica_valid(),

        .i_qp_cq_depth_req  (resp_hndler_o_qp_cq_depth_req),
        .o_qp_cq_depth_valid(qp_mgr_o_qp_cq_depth_valid),
        .o_qp_cq_depth      (qp_mgr_o_qp_cq_depth),
        .i_qp_cq_depth_idx  (resp_hndler_o_qp_cqe_proc_qpid),

        .o_global_dbg_cnt_value     (),                                //o_global_dbg_cnt_value     ),      
        .o_global_dbg_cnt_clr       (o_global_dbg_cnt_clr),
        .o_global_dbg_cnt_en        (o_global_dbg_cnt_en),
        .i_wqe_fsm_idle_cnt         (wqe_proc_idle_cnt),
        .i_hdr_backpressure_cnt     (wqe_proc_hdr_sgl_buf_full_cnt),
        .i_retry_tx_backpressure_cnt(wqe_proc_wr_retry_buf_full_cnt),
        .i_wqe_proc_rd_wqe_cnt      (wqe_proc_rd_wqe_cnt),
        .i_wqe_proc_rd_q_info_cnt   (wqe_proc_rd_q_info_cnt),
        .i_wqe_proc_wait0_cnt       (wqe_proc_wait0_cnt),
        .i_wqe_proc_ip_chksum_cnt   (wqe_proc_ip_chksum_cnt),
        .i_wqe_proc_hdr_gen_cnt     (wqe_proc_hdr_gen_cnt),
        .i_wqe_proc_hdr_sto_cnt     (wqe_proc_hdr_sto_cnt),

        .i_last_in_pkt_info ({1'b0, {(11 - C_QP_INDX_WIDTH) {1'b0}}, rx_pkt_hndler_o_last_ib_pkt_dest_qpid, rx_pkt_hndler_o_last_ib_pkt_psn[11:0], rx_pkt_hndler_o_last_ib_pkt_opcode}),
        .i_last_out_pkt_info(last_out_pkt_info),

        .i_retransmit_qpid            (resp_hndler_o_retransmit_qpid),          // QP id for this retransmission is required
        .i_retransmit_reqd            (resp_hndler_o_retransmit_reqd),          // Retransmission is required
        .i_retransmit_wqe_bram_din    (resp_hndler_o_retransmit_wqe_bram_din),
        .o_retransmit_accepted        (qp_mgr_o_retransmit_accepted),           // Ack from QP manager that the retransmission is initiated
        .i_psn_to_retry               (resp_hndler_o_psn_to_retry),
        .i_ssn_to_retry               (resp_hndler_o_ssn_to_retry),
        .i_osq_almost_full_set_valid  (osq_almost_full_set_valid),
        .i_osq_almost_full_set_id     (osq_almost_full_set_id),
        .i_osq_almost_full_reset_valid(osq_almost_full_reset_valid),
        .i_osq_almost_full_reset_id   (osq_almost_full_reset_id),

        // AXI Master signals
        .m_axi_awid   (qp_mgr_m_axi_awid),
        .m_axi_awaddr (qp_mgr_m_axi_awaddr),
        .m_axi_awlen  (qp_mgr_m_axi_awlen),
        .m_axi_awsize (qp_mgr_m_axi_awsize),
        .m_axi_awburst(qp_mgr_m_axi_awburst),
        .m_axi_awcache(qp_mgr_m_axi_awcache),
        .m_axi_awprot (qp_mgr_m_axi_awprot),
        .m_axi_awvalid(qp_mgr_m_axi_awvalid),
        .m_axi_awready(qp_mgr_m_axi_awready),

        .m_axi_wdata (qp_mgr_m_axi_wdata),
        .m_axi_wstrb (qp_mgr_m_axi_wstrb),
        .m_axi_wlast (qp_mgr_m_axi_wlast),
        .m_axi_wvalid(qp_mgr_m_axi_wvalid),
        .m_axi_wready(qp_mgr_m_axi_wready),
        .m_axi_awlock(qp_mgr_m_axi_awlock),

        .m_axi_bid   (qp_mgr_m_axi_bid),
        .m_axi_bresp (qp_mgr_m_axi_bresp),
        .m_axi_bvalid(qp_mgr_m_axi_bvalid),
        .m_axi_bready(qp_mgr_m_axi_bready),

        .m_axi_arid   (qp_mgr_m_axi_arid),
        .m_axi_araddr (qp_mgr_m_axi_araddr),
        .m_axi_arlen  (qp_mgr_m_axi_arlen),
        .m_axi_arsize (qp_mgr_m_axi_arsize),
        .m_axi_arburst(qp_mgr_m_axi_arburst),
        .m_axi_arcache(qp_mgr_m_axi_arcache),
        .m_axi_arprot (qp_mgr_m_axi_arprot),
        .m_axi_arvalid(qp_mgr_m_axi_arvalid),
        .m_axi_arready(qp_mgr_m_axi_arready),

        .m_axi_rid   (qp_mgr_m_axi_rid),
        .m_axi_rdata (qp_mgr_m_axi_rdata),
        .m_axi_rresp (qp_mgr_m_axi_rresp),
        .m_axi_rlast (qp_mgr_m_axi_rlast),
        .m_axi_rvalid(qp_mgr_m_axi_rvalid),
        .m_axi_rready(qp_mgr_m_axi_rready),
        .m_axi_arlock(qp_mgr_m_axi_arlock),

        //.i_osq_pop                        (resp_handler_osq_pop | wqe_dropped),
        .i_osq_pop         (resp_handler_osq_pop),
        .i_osq_pop_qpid    (resp_handler_osq_pop_qpid),
        .i_wqe_dropped     (wqe_dropped),
        .i_wqe_dropped_qpid(wqe_dropped_qpid),
        .init_start        (init_start),
        .init_count        (init_count)

    );



    assign qp_idx_TBD = 'b0;

    wqe_proc_top #(
        .NVMOF_SYSTEM_ONLY_FEATURE_SEND_INVALIDATE_EN(C_EN_IETH_IMMDT),
        .C_MAX_QP(C_NUM_QP),
        .C_MAX_QID_WIDTH(C_QP_INDX_WIDTH),
        .C_OS_Q_INDX_WIDTH(OS_Q_INDX_WIDTH),
        //.C_MAX_WRDATA_BUF_NUM(C_MAX_WR_RETRY_DATA_BUF_DEPTH),
        //.C_EN_WR_RETRY_DATA_BUF(C_EN_WR_RETRY_DATA_BUF),                                                
        .C_M_AXI_DATA_WIDTH(512),
        .C_M_AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .C_M_AXI_ID_WIDTH(C_M_AXI_ID_WIDTH),
        //.C_S_AXI_DATA_WIDTH(512),
        //.C_S_AXI_ADDR_WIDTH(32),
        //.C_S_AXI_ID_WIDTH(1),
        //.C_AXI_DMA_ADDR(32'h50040000),
        .C_MAX_SGL_DEPTH(C_MAX_SGL_DEPTH),
        .C_VIVADO_VER(1),
        .C_EN_DEBUG(C_EN_DEBUG_PORTS),
        .C_EN_RECV_RD_WR(C_EN_INITIATOR_LITE),
        .C_EN_RD_RESP_MIDDLE(C_EN_RD_RESP_MIDDLE),
        .C_MAX_RD_OS(C_MAX_RD_OS)
    ) wqe_proc_top_inst (
        .core_clk                         (m_axi_aclk),
        .core_rst                         (~core_rstn_wqe_reg),
        .m_axi_awid                       (wqe_proc_top_m_axi_awid),
        .m_axi_awaddr                     (wqe_proc_top_m_axi_awaddr),
        .m_axi_awlen                      (wqe_proc_top_m_axi_awlen),
        .m_axi_awsize                     (wqe_proc_top_m_axi_awsize),
        .m_axi_awburst                    (wqe_proc_top_m_axi_awburst),
        .m_axi_awcache                    (wqe_proc_top_m_axi_awcache),
        .m_axi_awprot                     (wqe_proc_top_m_axi_awprot),
        .m_axi_awvalid                    (wqe_proc_top_m_axi_awvalid),
        .m_axi_awready                    (wqe_proc_top_m_axi_awready),
        .m_axi_wdata                      (wqe_proc_top_m_axi_wdata),
        .m_axi_wstrb                      (wqe_proc_top_m_axi_wstrb),
        .m_axi_wlast                      (wqe_proc_top_m_axi_wlast),
        .m_axi_wvalid                     (wqe_proc_top_m_axi_wvalid),
        .m_axi_wready                     (wqe_proc_top_m_axi_wready),
        .m_axi_awlock                     (wqe_proc_top_m_axi_awlock),
        .m_axi_bid                        (wqe_proc_top_m_axi_bid),
        .m_axi_bresp                      (wqe_proc_top_m_axi_bresp),
        .m_axi_bvalid                     (wqe_proc_top_m_axi_bvalid),
        .m_axi_bready                     (wqe_proc_top_m_axi_bready),
        .m_axi_arid                       (wqe_proc_top_m_axi_arid),
        .m_axi_araddr                     (wqe_proc_top_m_axi_araddr),
        .m_axi_arlen                      (wqe_proc_top_m_axi_arlen),
        .m_axi_arsize                     (wqe_proc_top_m_axi_arsize),
        .m_axi_arburst                    (wqe_proc_top_m_axi_arburst),
        .m_axi_arcache                    (wqe_proc_top_m_axi_arcache),
        .m_axi_arprot                     (wqe_proc_top_m_axi_arprot),
        .m_axi_arvalid                    (wqe_proc_top_m_axi_arvalid),
        .m_axi_arready                    (wqe_proc_top_m_axi_arready),
        .m_axi_rid                        (wqe_proc_top_m_axi_rid),
        .m_axi_rdata                      (wqe_proc_top_m_axi_rdata),
        .m_axi_rresp                      (wqe_proc_top_m_axi_rresp),
        .m_axi_rlast                      (wqe_proc_top_m_axi_rlast),
        .m_axi_rvalid                     (wqe_proc_top_m_axi_rvalid),
        .m_axi_rready                     (wqe_proc_top_m_axi_rready),
        .m_axi_arlock                     (wqe_proc_top_m_axi_arlock),
        .m_axi_wr_ddr_awid                (wqe_proc_wr_ddr_m_axi_awid),
        .m_axi_wr_ddr_awaddr              (wqe_proc_wr_ddr_m_axi_awaddr),
        .m_axi_wr_ddr_awlen               (wqe_proc_wr_ddr_m_axi_awlen),
        .m_axi_wr_ddr_awsize              (wqe_proc_wr_ddr_m_axi_awsize),
        .m_axi_wr_ddr_awburst             (wqe_proc_wr_ddr_m_axi_awburst),
        .m_axi_wr_ddr_awcache             (wqe_proc_wr_ddr_m_axi_awcache),
        .m_axi_wr_ddr_awprot              (wqe_proc_wr_ddr_m_axi_awprot),
        .m_axi_wr_ddr_awvalid             (wqe_proc_wr_ddr_m_axi_awvalid),
        .m_axi_wr_ddr_awready             (wqe_proc_wr_ddr_m_axi_awready),
        .m_axi_wr_ddr_wdata               (wqe_proc_wr_ddr_m_axi_wdata),
        .m_axi_wr_ddr_wstrb               (wqe_proc_wr_ddr_m_axi_wstrb),
        .m_axi_wr_ddr_wlast               (wqe_proc_wr_ddr_m_axi_wlast),
        .m_axi_wr_ddr_wvalid              (wqe_proc_wr_ddr_m_axi_wvalid),
        .m_axi_wr_ddr_wready              (wqe_proc_wr_ddr_m_axi_wready),
        .m_axi_wr_ddr_awlock              (wqe_proc_wr_ddr_m_axi_awlock),
        .m_axi_wr_ddr_bid                 (wqe_proc_wr_ddr_m_axi_bid),
        .m_axi_wr_ddr_bresp               (wqe_proc_wr_ddr_m_axi_bresp),
        .m_axi_wr_ddr_bvalid              (wqe_proc_wr_ddr_m_axi_bvalid),
        .m_axi_wr_ddr_bready              (wqe_proc_wr_ddr_m_axi_bready),
        .m_axi_wr_ddr_arid                (wqe_proc_wr_ddr_m_axi_arid),
        .m_axi_wr_ddr_araddr              (wqe_proc_wr_ddr_m_axi_araddr),
        .m_axi_wr_ddr_arlen               (wqe_proc_wr_ddr_m_axi_arlen),
        .m_axi_wr_ddr_arsize              (wqe_proc_wr_ddr_m_axi_arsize),
        .m_axi_wr_ddr_arburst             (wqe_proc_wr_ddr_m_axi_arburst),
        .m_axi_wr_ddr_arcache             (wqe_proc_wr_ddr_m_axi_arcache),
        .m_axi_wr_ddr_arprot              (wqe_proc_wr_ddr_m_axi_arprot),
        .m_axi_wr_ddr_arvalid             (wqe_proc_wr_ddr_m_axi_arvalid),
        .m_axi_wr_ddr_arready             (wqe_proc_wr_ddr_m_axi_arready),
        .m_axi_wr_ddr_rid                 (wqe_proc_wr_ddr_m_axi_rid),
        .m_axi_wr_ddr_rdata               (wqe_proc_wr_ddr_m_axi_rdata),
        .m_axi_wr_ddr_rresp               (wqe_proc_wr_ddr_m_axi_rresp),
        .m_axi_wr_ddr_rlast               (wqe_proc_wr_ddr_m_axi_rlast),
        .m_axi_wr_ddr_rvalid              (wqe_proc_wr_ddr_m_axi_rvalid),
        .m_axi_wr_ddr_rready              (wqe_proc_wr_ddr_m_axi_rready),
        .m_axi_wr_ddr_arlock              (wqe_proc_wr_ddr_m_axi_arlock),
        //.s_axi_awid(wqe_proc_top_s_axi_awid),
        //.s_axi_awaddr(wqe_proc_top_s_axi_awaddr),
        //.s_axi_awlen(wqe_proc_top_s_axi_awlen),
        //.s_axi_awsize(wqe_proc_top_s_axi_awsize),
        //.s_axi_awburst(wqe_proc_top_s_axi_awburst),
        //.s_axi_awcache(wqe_proc_top_s_axi_awcache),
        //.s_axi_awprot(wqe_proc_top_s_axi_awprot),
        //.s_axi_awvalid(wqe_proc_top_s_axi_awvalid),
        //.s_axi_awready(wqe_proc_top_s_axi_awready),
        //.s_axi_wdata(wqe_proc_top_s_axi_wdata),
        //.s_axi_wstrb(wqe_proc_top_s_axi_wstrb),
        //.s_axi_wlast(wqe_proc_top_s_axi_wlast),
        //.s_axi_wvalid(wqe_proc_top_s_axi_wvalid),
        //.s_axi_wready(wqe_proc_top_s_axi_wready),
        //.s_axi_awlock(wqe_proc_top_s_axi_awlock),
        //.s_axi_bid(wqe_proc_top_s_axi_bid),
        //.s_axi_bresp(wqe_proc_top_s_axi_bresp),
        //.s_axi_bvalid(wqe_proc_top_s_axi_bvalid),
        //.s_axi_bready(wqe_proc_top_s_axi_bready),
        //.s_axi_arid(wqe_proc_top_s_axi_arid),
        //.s_axi_araddr(wqe_proc_top_s_axi_araddr),
        //.s_axi_arlen(wqe_proc_top_s_axi_arlen),
        //.s_axi_arsize(wqe_proc_top_s_axi_arsize),
        //.s_axi_arburst(wqe_proc_top_s_axi_arburst),
        //.s_axi_arcache(wqe_proc_top_s_axi_arcache),
        //.s_axi_arprot(wqe_proc_top_s_axi_arprot),
        //.s_axi_arvalid(wqe_proc_top_s_axi_arvalid),
        //.s_axi_arready(wqe_proc_top_s_axi_arready),
        //.s_axi_rid(wqe_proc_top_s_axi_rid),
        //.s_axi_rdata(wqe_proc_top_s_axi_rdata),
        //.s_axi_rresp(wqe_proc_top_s_axi_rresp),
        //.s_axi_rlast(wqe_proc_top_s_axi_rlast),
        //.s_axi_rvalid(wqe_proc_top_s_axi_rvalid),
        //.s_axi_rready(wqe_proc_top_s_axi_rready),
        //.s_axi_arlock(wqe_proc_top_s_axi_arlock),
        .m_axis_tdata                     (wqe_proc_top_m_axis_tdata_int),
        .m_axis_tkeep                     (wqe_proc_top_m_axis_tkeep),
        .m_axis_tvalid                    (wqe_proc_top_m_axis_tvalid_d),
        .m_axis_tready                    (wqe_proc_top_m_axis_tready_d),
        .m_axis_tlast                     (wqe_proc_top_m_axis_tlast),
        //.s_axis_tdata(wqe_proc_top_s_axis_tdata),
        //.s_axis_tkeep(wqe_proc_top_s_axis_tkeep),
        //.s_axis_tvalid(wqe_proc_top_s_axis_tvalid),
        //.s_axis_tlast(wqe_proc_top_s_axis_tlast),
        //.s_axis_tready(wqe_proc_top_s_axis_tready),
        .i_qpm_fifo_empty                 (qpm_wqe_fifo_empty),
        .i_wqe_halt                       (qp_mgr_o_halt),
        .i_wqe_halted_qpid                (qp_mgr_o_halted_qpid),
        .o_wqe_halted                     (wqe_proc_i_wqe_halted),
        .i_qpm_wqe_data                   (qpm_wqe_data),
        .o_qpm_wqe_pop                    (qpm_wqe_pop),
        .o_reg_rd_en                      (wqe_proc_top_o_reg_rd_en),
        .o_reg_rd_en_s                    (wqe_proc_top_o_reg_rd_en_s),
        .i_reg_rdy                        (qp_mgr_o_qp_sq_psn_wqe_valid & qp_mgr_o_qp_mac_dest_addrm_valid),
        .i_reg_psn_val                    (qp_mgr_o_qp_sq_psn_wqe_valid),
        //.i_reg_mac_addr_rdy(qp_mgr_o_qp_mac_dest_addrm_valid),                                                      
        .o_reg_wr_en                      (wqe_proc_top_o_reg_wr_en),
        .o_qp_id                          (wqe_proc_top_o_qp_id),
        .i_reg_pkey                       (qp_mgr_o_qp_adv_conf[31:16]),
        .i_reg_pmtu                       (qp_mgr_o_qp_conf[10:8]),                                                                                        // use relevant qp_conf field
        .i_reg_dest_qpid                  (qp_mgr_o_qp_dest_qpid),                                                                                         // pls add ready and connect ~rx_pkt_hndler_o_qpm_req_dest_qp_conf
        .i_reg_sq_psn                     (qp_mgr_o_qp_sq_psn),
        .o_reg_sq_psn                     (wqe_proc_top_reg_sq_psn),
        .i_reg_sq_msn                     (qp_mgr_o_qp_sq_ssn),
        .o_reg_sq_msn                     (wqe_proc_top_reg_sq_ssn),
        .i_reg_max_rd_atomic              (qp_mgr_o_qp_conf[23:16]),                                                                                       // use relevant qp_conf field
        .i_reg_mac_dest_addr              ({qp_mgr_o_qp_mac_dest_addrm[15:0], qp_mgr_o_qp_mac_dest_addrl}),
        .i_reg_ip_dest_addr               ({qp_mgr_o_qp_ip_dest_addr4, qp_mgr_o_qp_ip_dest_addr3, qp_mgr_o_qp_ip_dest_addr2, qp_mgr_o_qp_ip_dest_addr1}),
        .i_reg_udp_src_port               (qp_mgr_o_xrnic_udp_src_port),                                                                                   // this is a static value for RoCE
        .i_reg_qp_to                      (qp_mgr_o_qp_timeout_per_qp[4:0]),
        .i_reg_ttl                        (qp_mgr_o_qp_adv_conf[15:8]),
        .i_reg_dscp                       (qp_mgr_o_qp_adv_conf[5:0]),
        .i_reg_exp_ack                    (~qp_mgr_o_qp_conf[1]),                                                                                          //needs a register bit
        .i_reg_ack_resp_strategy          (qp_mgr_o_tx_ack_gen),
        .i_reg_mac_src_addr               (qp_mgr_o_mac_src_addr),
        .i_reg_ipv4_src_addr              (qp_mgr_o_ipv4_src_addr),                                                                                        // need ipv4/ipv6
        .i_reg_ipv6_src_addr              (ipv6_src_addr_wqe_reg),                                                                                         // need ipv4/ipv6
        //.i_reg_tx_hdr_buf_ba(qp_mgr_o_tx_hdr_buf_ba),
        //.i_reg_tx_hdr_buf_depth(qp_mgr_o_tx_hdr_buf_sz_num_hdrs),
        //.i_reg_tx_hdr_buf_size(qp_mgr_o_tx_hdr_buf_sz_buf_sz),
        //.i_reg_tx_sgl_buf_ba(qp_mgr_o_tx_sgl_buf_ba),
        //.i_reg_tx_sgl_buf_depth(qp_mgr_o_tx_sgl_buf_sz_num_sgls),
        //.i_reg_tx_sgl_buf_size(qp_mgr_o_tx_sgl_buf_sz_buf_sz),
        .i_reg_xrnic_en                   (qp_mgr_o_xrnic_en),
        .i_reg_ip_ver                     (qp_mgr_o_qp_conf[7]),
        .i_reg_wrdata_buf_ba              (qp_mgr_o_data_buf_ba),
        .i_reg_wrdata_num_bufs            (qp_mgr_o_data_buf_sz_num_bufs),
        .i_reg_wrdata_buf_sz              (qp_mgr_o_data_buf_sz_buf_sz),
        .i_bres_valid                     (rx_pkt_wqe_proc_ob_rsp_req),
        .i_bres_exp_ack                   (rx_pkt_wqe_proc_ob_rsp_expl_ack),
        .i_bres_dest_qpid                 (rx_pkt_wqe_proc_ob_rsp_qpid),
        .o_bres_fifo_full                 (wqe_proc_o_bres_fifo_full),
        .i_res_psn                        (rx_pkt_wqe_proc_ob_rsp_psn),
        .i_res_aeth_syndrome              (rx_pkt_wqe_proc_aeth_syndrome[6:0]),
        .i_res_msn                        (rx_pkt_wqe_proc_ob_rsp_msn),
        .i_req_type                       (req_indicator),                                                                                                 //i_req_type),
        .i_rd_req_phy_addr                (physical_addr),                                                                                                 //i_phy_addr_lsb),
        .i_rd_payload_len                 (rd_dma_len),                                                                                                    //i_rd_payload_len),
        .i_rd_req_pmtu                    (rd_req_pmtu),                                                                                                   //i_rd_payload_len),
        .i_rsp_last_psn                   (rsp_last_psn),                                                                                                  //i_rd_payload_len),
        //.o_rd_req_fifo_full(rd_req_fifo_full),  //o_rd_req_fifo_full),
        .i_freeup_data_buf                (resp_hndler_freeup_data_buf),                                                                                   // Freeup RDMA WRITE DATA buffer
        .i_freeup_data_bufid              (resp_hndler_freeup_data_bufid),                                                                                 // BUffer ID to be freed up
        .o_wqp_req                        (wqe_proc_os_wqp_req),
        .o_wqp_req_Read                   (wqe_proc_os_wqp_req_Read),
        .i_wqp_resp                       (wqe_proc_os_wqp_rdy),
        .o_wqe_qpid                       (wqe_proc_os_wqe_qpid),
        .o_wqe_opcode                     (wqe_proc_os_wqe_opcode),
        .o_wqe_qp_to                      (wqe_proc_o_wqe_qp_to),
        .o_wqe_wrid                       (wqe_proc_os_wqe_wrid),
        .o_wqe_retried                    (wqe_proc_os_wqe_retried),
        .o_wqe_fatal_push                 (wqe_proc_os_wqe_fatal_push),
        .o_wqe_data_bufid                 (wqe_proc_os_wqe_data_bufid),
        .o_wqe_send_psn                   (wqe_proc_os_wqe_send_psn),
        .o_wqe_send_first_psn             (wqe_proc_os_wqe_send_first_psn),
        .o_wqe_send_msn                   (wqe_proc_os_wqe_send_msn),
        .o_wqe_rdma_rd_rcv_ba             (wqe_proc_os_wqe_rdma_rd_rcv_ba),
        .o_wqe_rdma_rd_rcv_len            (wqe_proc_os_wqe_rdma_rd_res_len),
        .o_wqe_explicit_ack_req           (wqe_proc_os_wqe_explicit_ack_req),
        .o_wqe_status_out                 (wqe_proc_status_out),
        .o_last_out_pkt_info              (last_out_pkt_info),
        .o_out_send_pkt_cnt               (wqe_out_send_pkt_cnt),
        .o_out_rw_pkt_cnt                 (wqe_out_rdwr_pkt_cnt),
        .o_out_ack_pkt_cnt                (wqe_out_ack_pkt_cnt),
        .o_out_mad_pkt_cnt                (wqe_out_mad_pkt_cnt),
        .o_out_rd_resp_pkt_cnt            (wqe_out_rd_resp_pkt_cnt),
        //.i_out_errsts_q_ba(out_errsts_q_ba),
        //.i_out_errsts_q_sz(out_errsts_q_sz),
        //.o_out_errsts_q_wrptr(out_errsts_q_wrptr),
        //.i_intr_en_ill_opc_in_sq(intr_en_ill_opc_in_sq),
        //.i_intr_clr_ill_opc_in_sq(intr_clr_ill_opc_in_sq),
        //.o_ill_opc_in_sq_intr(ill_opc_in_sq_intr),
        .os_num_vld_entries               (os_num_vld_entries),
        .o_wqe_proc_idle_cnt              (wqe_proc_idle_cnt),
        .o_wqe_proc_rd_wqe_cnt            (wqe_proc_rd_wqe_cnt),
        .o_wqe_proc_rd_q_info_cnt         (wqe_proc_rd_q_info_cnt),
        .o_wqe_proc_wait0_cnt             (wqe_proc_wait0_cnt),
        .o_wqe_proc_ip_chksum_cnt         (wqe_proc_ip_chksum_cnt),
        .o_wqe_proc_hdr_gen_cnt           (wqe_proc_hdr_gen_cnt),
        .o_wqe_proc_hdr_sto_cnt           (wqe_proc_hdr_sto_cnt),
        .o_wqe_proc_hdr_sgl_buf_full_cnt  (wqe_proc_hdr_sgl_buf_full_cnt),
        .o_wqe_proc_wr_retry_buf_full_cnt (wqe_proc_wr_retry_buf_full_cnt),
        .i_debug_cnt_en                   (o_global_dbg_cnt_en),
        .i_debug_cnt_clr                  (o_global_dbg_cnt_clr),
        .i_qp_recovery                    (qp_mgr_o_qp_conf[6]),
        .o_dma_in_idle                    (wqe_proc_dma_in_idle),
        .o_wqe_fatal_qpid                 (wqe_fatal_qpid),
        .o_wqe_fatal_qp_vld               (wqe_fatal_qp_vld),
        .i_wqe_fatal_qp_rdy               (wqe_fatal_qp_rdy),
        .i_osq_pop                        (resp_handler_osq_pop),
        .o_wqe_dropped                    (wqe_dropped),
        .o_wqe_dropped_qpid               (wqe_dropped_qpid),
        .i_qp_enable_idx                  (qp_enable_wr_addr),
        .i_qp_enable_vld                  (qp_enable_wr_en),
        .i_qp_enable_data                 (qp_enable_wr_data),
        .i_qp_fatal_idx                   (rx_pkt_hndler_o_qp_fatal_wr_idx),
        .i_qp_fatal_vld                   (rx_pkt_hndler_o_qp_fatal_wr_en),
        .i_qp_fatal_data                  (rx_pkt_hndler_o_qp_fatal_wr_data),
        //.i_osq_nacked_wr_en(osq_nacked_wr_en),
        //.i_osq_nacked_wr_addr(osq_nacked_wr_addr),
        //.i_osq_nacked_wr_data(osq_nacked_wr_data),
        .i_osq_nacked                     (o_osq_nacked),
        .o_rd_req_fifo_full_wr_en_0       (rd_req_fifo_full_wr_en_0),
        .o_rd_req_fifo_full_wr_addr_0     (rd_req_fifo_full_wr_addr_0),
        .o_rd_req_fifo_full_wr_en_1       (rd_req_fifo_full_wr_en_1),
        .o_rd_req_fifo_full_wr_addr_1     (rd_req_fifo_full_wr_addr_1),
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
        .i_qp_under_recovery_wr_data      (qp_under_recovery_wr_data)
    );

    /**********************************************************/
    //All lut Ram contents has to be re-initialized manually when a sudden hard reset is encountered
    //当遇到突然的硬重置时，所有lut Ram内容都必须手动重新初始化。
    always @(posedge m_axi_aclk) begin : global_lut_ram_init_process
        core_rst_n_r <= core_rstn_local_reg;
        if (~core_rst_n_r) begin
            init_start <= 1'b1;
        end else if (init_count == C_NUM_QP - 1) begin
            init_start <= 1'b0;
        end

        if (init_start && init_count != C_NUM_QP - 1) begin  //sudden hard reset -- 突然硬重启
            init_count <= init_count + 1;
        end else if (init_start && init_count == C_NUM_QP - 1) begin
            init_count <= init_count;
        end else begin
            init_count <= {C_QP_INDX_WIDTH{1'b0}};
        end

    end

    assign lut_ram_reset_busy = init_start && init_count != (C_NUM_QP - 1);



    /**********************************************************/

    // resp_hndler_o_osq_empty, rx_pkt_hndler_o_rq_full and o_q_timed_out are used by verification
    // these 3 reg will be placed under a parameter control for verification only (not used by design)
    reg [C_NUM_QP-1 : 0] o_q_timed_out;
    reg [C_NUM_QP-1 : 0] rx_pkt_hndler_o_rq_full;
    reg [C_NUM_QP-1 : 0] resp_hndler_o_osq_empty;
    generate
        if (SIM_REGS_EN == 1) begin : Verification_only
            always @(*) begin
                if (~core_rstn_local_reg) begin
                    resp_hndler_o_osq_empty = {C_NUM_QP{1'b1}};
                end else begin
                    if (osq_empty_set_valid) begin
                        resp_hndler_o_osq_empty[osq_empty_set_id] = 1'b1;
                    end

                    if (osq_empty_reset_valid) begin
                        resp_hndler_o_osq_empty[osq_empty_reset_id] = 1'b0;
                    end
                end
            end

            always @(*) begin
                if (~core_rstn_local_reg) begin
                    o_q_timed_out = {C_NUM_QP{1'b0}};
                end else begin
                    if (osq_timeout_set_valid) begin
                        o_q_timed_out[osq_timeout_set_id] = 1'b1;
                    end else begin
                        o_q_timed_out = {C_NUM_QP{1'b0}};
                    end
                end
            end

            always @(*) begin
                if (~core_rstn_local_reg) begin
                    rx_pkt_hndler_o_rq_full = {C_NUM_QP{1'b0}};
                end else begin
                    if (rx_pkt_hndler_o_rq_full_reset_wr_en) begin
                        rx_pkt_hndler_o_rq_full[rx_pkt_hndler_o_rq_full_reset_idx] = rx_pkt_hndler_o_rq_full_reset_val;
                    end

                    if (rx_pkt_hndler_o_rq_full_wr_en) begin
                        rx_pkt_hndler_o_rq_full[rx_pkt_hndler_o_rq_full_idx] = rx_pkt_hndler_o_rq_full_val;
                    end
                end
            end

        end
    endgenerate
    //always@(posedge m_axi_aclk) begin
    //	rx_pkt_hndlr_i_clr_cnp_sched_r <= rx_pkt_hndlr_i_clr_cnp_sched;
    //end


    /*         generate if(C_NUM_QP < 32) begin
            always@(*) begin
                clr_cnp_sched = rx_pkt_hndlr_i_clr_cnp_sched;
                clr_cnp_sched_wr_addr = 'd0;
            end
        end else begin
            always@(*) begin
		if (rx_pkt_hndlr_i_clr_cnp_sched != rx_pkt_hndlr_i_clr_cnp_sched_r) begin
                        for (i=0; i<C_NUM_QP/32; i=i+1) begin
                            for(k=0; k < 32; k=k+1) begin
                                if(rx_pkt_hndlr_i_clr_cnp_sched[(i*32)+k] != rx_pkt_hndlr_i_clr_cnp_sched_r[(i*32)+k]) begin
                                    clr_cnp_sched[k] = rx_pkt_hndlr_i_clr_cnp_sched[(i*32)+k];
                                    clr_cnp_sched_wr_addr = (i*32);
                                end
                            end 
                        end
                end else begin
                     clr_cnp_sched = 'd0;
                     clr_cnp_sched_wr_addr = 'd0;
                end
            end
        end
        endgenerate */


    /* 	always@(*) begin
		if (~m_axi_aresetn_eval) begin
			wqe_proc_os_fifo_full = {C_NUM_QP{1'b0}};
		end else begin
		    if (osq_full_set_valid) begin
		    	wqe_proc_os_fifo_full[osq_full_set_id] = 1'b1;
		    end
		    if (osq_full_reset_valid) begin
		    	wqe_proc_os_fifo_full[osq_full_reset_id] = 1'b0;
		    end
		end
	end */

    /* 	always@(*) begin
		if (~m_axi_aresetn_eval) begin
			resp_hndler_o_osq_almost_full = {C_NUM_QP{1'b0}};
		end else begin
		    if (osq_almost_full_set_valid) begin
		    	resp_hndler_o_osq_almost_full[osq_almost_full_set_id] = 1'b1;
		    end
		    if (osq_almost_full_reset_valid) begin
		    	resp_hndler_o_osq_almost_full[osq_almost_full_reset_id] = 1'b0;
		    end
		end
	end */

    /* 	always@(*) begin
		if (~m_axi_aresetn_eval) begin
			resp_hndler_o_qp_retried = {C_NUM_QP{1'b0}};
		end else begin
		    if (qp_retried_set_valid) begin
		    	resp_hndler_o_qp_retried[qp_retried_set_id] = 1'b1;
		    end
		    if (qp_retried_reset_valid) begin
		    	resp_hndler_o_qp_retried[qp_retried_reset_id] = 1'b0;
		    end
		end
	end */

    //wire                                        o_osq_nacked_set_valid;
    //wire [C_QP_INDX_WIDTH-1 : 0]                o_osq_nacked_set_id;
    //wire                                        o_osq_nacked_set_valid_pending;
    //wire [C_QP_INDX_WIDTH-1 : 0]                o_osq_nacked_set_id_pending;
    //wire                                        o_osq_nacked_reset_valid;
    //wire [C_QP_INDX_WIDTH-1 : 0]                o_osq_nacked_reset_id;
    //wire                                        o_osq_nacked_reset_valid_pending;
    //wire [C_QP_INDX_WIDTH-1 : 0]                o_osq_nacked_reset_id_pending;	

    //assign {o_osq_nacked_set_valid,o_osq_nacked_set_id,o_osq_nacked_set_valid_pending,o_osq_nacked_set_id_pending,o_osq_nacked_reset_valid,o_osq_nacked_reset_id,
    //                       o_osq_nacked_reset_valid_pending,o_osq_nacked_reset_id_pending} = o_osq_nacked;

    //	always@(*) begin
    //		if (!m_axi_aresetn_eval) begin
    //			o_osq_nacked_sim = {C_QP_INDX_WIDTH{1'b0}};
    //		end else begin
    //			if (o_osq_nacked_set_valid) begin
    //				o_osq_nacked_sim[o_osq_nacked_set_id] = 1'b1;
    //			end
    //			
    //			if (o_osq_nacked_set_valid_pending) begin
    //				o_osq_nacked_sim[o_osq_nacked_set_id_pending] = 1'b1;
    //			end
    //			
    //			if (o_osq_nacked_reset_valid) begin
    //				o_osq_nacked_sim[o_osq_nacked_reset_id] = 1'b0;
    //			end
    //			
    //			if (o_osq_nacked_reset_valid_pending) begin
    //				o_osq_nacked_sim[o_osq_nacked_reset_id_pending] = 1'b0;
    //			end
    //		end
    //	end

    // 真实QP号
`ifdef SIMULATION
    wire [C_QP_INDX_WIDTH:0] dbg_qp_idx;
    assign dbg_qp_idx = rx_pkt_hndler_o_qp_rsp_conf_idx + 1'b1;
`endif

endmodule
