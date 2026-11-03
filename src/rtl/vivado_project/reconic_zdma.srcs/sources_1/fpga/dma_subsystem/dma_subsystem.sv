// SPDX-License-Identifier: MIT
// Minimal sgdma-facing sgdma subsystem shim.
//
// This file intentionally removes the OpenNIC/sgdma physical-function,
// address-map, RSS/hash, function-register, H2C splitter, and C2H arbiter
// hierarchy.  The remaining logic only adapts the sgdma wrapper streams to the
// surrounding 512-bit network datapath and terminates legacy sgdma control
// interfaces that are no longer implemented by sgdma.
`timescale 1ns / 1ps

`ifndef OPEN_NIC_SHELL_SIM
`ifndef __synthesis__
`define __synthesis__
`endif
`endif

module dma_subsystem #(
    parameter int MIN_PKT_LEN   = 64,
    parameter int MAX_PKT_LEN   = 1518,
    parameter int USE_PHYS_FUNC = 1,
    parameter int NUM_PHYS_FUNC = 1,
    parameter int NUM_QUEUE     = 512
) (
    input         s_axil_awvalid,
    input  [31:0] s_axil_awaddr,
    output        s_axil_awready,
    input         s_axil_wvalid,
    input  [31:0] s_axil_wdata,
    output        s_axil_wready,
    output        s_axil_bvalid,
    output [ 1:0] s_axil_bresp,
    input         s_axil_bready,
    input         s_axil_arvalid,
    input  [31:0] s_axil_araddr,
    output        s_axil_arready,
    output        s_axil_rvalid,
    output [31:0] s_axil_rdata,
    output [ 1:0] s_axil_rresp,
    input         s_axil_rready,

    output [    NUM_PHYS_FUNC-1:0] m_axis_h2c_tvalid,
    output [512*NUM_PHYS_FUNC-1:0] m_axis_h2c_tdata,
    output [ 64*NUM_PHYS_FUNC-1:0] m_axis_h2c_tkeep,
    output [    NUM_PHYS_FUNC-1:0] m_axis_h2c_tlast,
    output [ 16*NUM_PHYS_FUNC-1:0] m_axis_h2c_tuser_size,
    output [ 16*NUM_PHYS_FUNC-1:0] m_axis_h2c_tuser_src,
    output [ 16*NUM_PHYS_FUNC-1:0] m_axis_h2c_tuser_dst,
    input  [    NUM_PHYS_FUNC-1:0] m_axis_h2c_tready,

    input  [    NUM_PHYS_FUNC-1:0] s_axis_c2h_tvalid,
    input  [512*NUM_PHYS_FUNC-1:0] s_axis_c2h_tdata,
    input  [ 64*NUM_PHYS_FUNC-1:0] s_axis_c2h_tkeep,
    input  [    NUM_PHYS_FUNC-1:0] s_axis_c2h_tlast,
    input  [ 16*NUM_PHYS_FUNC-1:0] s_axis_c2h_tuser_size,
    input  [ 16*NUM_PHYS_FUNC-1:0] s_axis_c2h_tuser_src,
    input  [ 16*NUM_PHYS_FUNC-1:0] s_axis_c2h_tuser_dst,
    output [    NUM_PHYS_FUNC-1:0] s_axis_c2h_tready,

    // sgdma DMA Engine - AXI MM interface
    input          m_axi_awready,
    input          m_axi_wready,
    input  [  3:0] m_axi_bid,
    input  [  1:0] m_axi_bresp,
    input          m_axi_bvalid,
    input          m_axi_arready,
    input  [  3:0] m_axi_rid,
    input  [511:0] m_axi_rdata,
    input  [  1:0] m_axi_rresp,
    input          m_axi_rlast,
    input          m_axi_rvalid,
    output [  3:0] m_axi_awid,
    output [ 63:0] m_axi_awaddr,
    output [ 31:0] m_axi_awuser,
    output [  7:0] m_axi_awlen,
    output [  2:0] m_axi_awsize,
    output [  1:0] m_axi_awburst,
    output [  2:0] m_axi_awprot,
    output         m_axi_awvalid,
    output         m_axi_awlock,
    output [  3:0] m_axi_awcache,
    output [511:0] m_axi_wdata,
    output [ 63:0] m_axi_wuser,
    output [ 63:0] m_axi_wstrb,
    output         m_axi_wlast,
    output         m_axi_wvalid,
    output         m_axi_bready,
    output [  3:0] m_axi_arid,
    output [ 63:0] m_axi_araddr,
    output [ 31:0] m_axi_aruser,
    output [  7:0] m_axi_arlen,
    output [  2:0] m_axi_arsize,
    output [  1:0] m_axi_arburst,
    output [  2:0] m_axi_arprot,
    output         m_axi_arvalid,
    output         m_axi_arlock,
    output [  3:0] m_axi_arcache,
    output         m_axi_rready,

    input          camera_frame_irq,

`ifdef __synthesis__
    input  [7:0] pcie_rxp,
    input  [7:0] pcie_rxn,
    output [7:0] pcie_txp,
    output [7:0] pcie_txn,

    // BAR2-mapped master AXI-Lite feeding into system configuration block
    output        m_axil_pcie_awvalid,
    output [31:0] m_axil_pcie_awaddr,
    input         m_axil_pcie_awready,
    output        m_axil_pcie_wvalid,
    output [31:0] m_axil_pcie_wdata,
    input         m_axil_pcie_wready,
    input         m_axil_pcie_bvalid,
    input  [ 1:0] m_axil_pcie_bresp,
    output        m_axil_pcie_bready,
    output        m_axil_pcie_arvalid,
    output [31:0] m_axil_pcie_araddr,
    input         m_axil_pcie_arready,
    input         m_axil_pcie_rvalid,
    input  [31:0] m_axil_pcie_rdata,
    input  [ 1:0] m_axil_pcie_rresp,
    output        m_axil_pcie_rready,

    input  pcie_refclk_p,
    input  pcie_refclk_n,
    input  pcie_rstn,
    output user_lnk_up,
    output phy_ready,

    // This reset signal serves as a power-up reset for the entire system.  It is
    // routed into the `system_config` submodule to generate proper reset signals
    // for each submodule.
    output         powerup_rstn,
`else  // !`ifdef __synthesis__
    input          s_axis_sgdma_h2c_tvalid,
    input  [511:0] s_axis_sgdma_h2c_tdata,
    input  [ 31:0] s_axis_sgdma_h2c_tcrc,
    input          s_axis_sgdma_h2c_tlast,
    input  [ 10:0] s_axis_sgdma_h2c_tuser_qid,
    input  [  2:0] s_axis_sgdma_h2c_tuser_port_id,
    input          s_axis_sgdma_h2c_tuser_err,
    input  [ 31:0] s_axis_sgdma_h2c_tuser_mdata,
    input  [  5:0] s_axis_sgdma_h2c_tuser_mty,
    input          s_axis_sgdma_h2c_tuser_zero_byte,
    output         s_axis_sgdma_h2c_tready,

    output         m_axis_sgdma_c2h_tvalid,
    output [511:0] m_axis_sgdma_c2h_tdata,
    output [ 31:0] m_axis_sgdma_c2h_tcrc,
    output         m_axis_sgdma_c2h_tlast,
    output         m_axis_sgdma_c2h_ctrl_marker,
    output [  2:0] m_axis_sgdma_c2h_ctrl_port_id,
    output [  6:0] m_axis_sgdma_c2h_ctrl_ecc,
    output [ 15:0] m_axis_sgdma_c2h_ctrl_len,
    output [ 10:0] m_axis_sgdma_c2h_ctrl_qid,
    output         m_axis_sgdma_c2h_ctrl_has_cmpt,
    output [  5:0] m_axis_sgdma_c2h_mty,
    input          m_axis_sgdma_c2h_tready,

    output         m_axis_sgdma_cpl_tvalid,
    output [511:0] m_axis_sgdma_cpl_tdata,
    output [  1:0] m_axis_sgdma_cpl_size,
    output [ 15:0] m_axis_sgdma_cpl_dpar,
    output [ 10:0] m_axis_sgdma_cpl_ctrl_qid,
    output [  1:0] m_axis_sgdma_cpl_ctrl_cmpt_type,
    output [ 15:0] m_axis_sgdma_cpl_ctrl_wait_pld_pkt_id,
    output [  2:0] m_axis_sgdma_cpl_ctrl_port_id,
    output         m_axis_sgdma_cpl_ctrl_marker,
    output         m_axis_sgdma_cpl_ctrl_user_trig,
    output [  2:0] m_axis_sgdma_cpl_ctrl_col_idx,
    output [  2:0] m_axis_sgdma_cpl_ctrl_err_idx,
    output         m_axis_sgdma_cpl_ctrl_no_wrb_marker,
    input          m_axis_sgdma_cpl_tready,
`endif

    // sgdma bridge slave interface
    input  [  3:0] s_axib_awid,
    input  [ 63:0] s_axib_awaddr,
    input  [  3:0] s_axib_awregion,
    input  [  7:0] s_axib_awlen,
    input  [  2:0] s_axib_awsize,
    input  [  1:0] s_axib_awburst,
    input          s_axib_awvalid,
    input  [511:0] s_axib_wdata,
    input  [ 63:0] s_axib_wstrb,
    input          s_axib_wlast,
    input          s_axib_wvalid,
    input  [ 63:0] s_axib_wuser,
    output [ 63:0] s_axib_ruser,
    input          s_axib_bready,
    input  [  3:0] s_axib_arid,
    input  [ 63:0] s_axib_araddr,
    input  [ 11:0] s_axib_aruser,
    input  [ 11:0] s_axib_awuser,
    input  [  3:0] s_axib_arregion,
    input  [  7:0] s_axib_arlen,
    input  [  2:0] s_axib_arsize,
    input  [  1:0] s_axib_arburst,
    input          s_axib_arvalid,
    input          s_axib_rready,
    output         s_axib_awready,
    output         s_axib_wready,
    output [  3:0] s_axib_bid,
    output [  1:0] s_axib_bresp,
    output         s_axib_bvalid,
    output         s_axib_arready,
    output [  3:0] s_axib_rid,
    output [511:0] s_axib_rdata,
    output [  1:0] s_axib_rresp,
    output         s_axib_rlast,
    output         s_axib_rvalid,

    input  mod_rstn,
    output mod_rst_done,

`ifdef __synthesis__
    output     axil_aclk,
    output     axis_aclk,
    output     cmlk_ref_clk,
    output     cmlk_idelay_ref_clk,
    output     cmlk_idelay_ref_locked
`else
    output reg axil_aclk,
    output reg axis_aclk,
    output reg cmlk_ref_clk,
    output reg cmlk_idelay_ref_clk,
    output reg cmlk_idelay_ref_locked
`endif
); 

    localparam [15:0] DEFAULT_RX_QBASE = 16'd32;
    localparam [15:0] DEFAULT_RX_NUMQ  = 16'd32;

    wire         axis_sgdma_h2c_tvalid;
    wire [511:0] axis_sgdma_h2c_tdata;
    wire [ 31:0] axis_sgdma_h2c_tcrc;
    wire         axis_sgdma_h2c_tlast;
    wire [ 10:0] axis_sgdma_h2c_tuser_qid;
    wire [  2:0] axis_sgdma_h2c_tuser_port_id;
    wire         axis_sgdma_h2c_tuser_err;
    wire [ 31:0] axis_sgdma_h2c_tuser_mdata;
    wire [  5:0] axis_sgdma_h2c_tuser_mty;
    wire         axis_sgdma_h2c_tuser_zero_byte;
    wire         axis_sgdma_h2c_tready;

    wire         axis_sgdma_c2h_tvalid;
    wire [511:0] axis_sgdma_c2h_tdata;
    wire [ 31:0] axis_sgdma_c2h_tcrc;
    wire         axis_sgdma_c2h_tlast;
    wire         axis_sgdma_c2h_ctrl_marker;
    wire [  2:0] axis_sgdma_c2h_ctrl_port_id;
    wire [  6:0] axis_sgdma_c2h_ctrl_ecc;
    wire [ 15:0] axis_sgdma_c2h_ctrl_len;
    wire [ 10:0] axis_sgdma_c2h_ctrl_qid;
    wire         axis_sgdma_c2h_ctrl_has_cmpt;
    wire [  5:0] axis_sgdma_c2h_mty;
    wire         axis_sgdma_c2h_tready;

    wire         axis_sgdma_cpl_tvalid;
    wire [511:0] axis_sgdma_cpl_tdata;
    wire [  1:0] axis_sgdma_cpl_size;
    wire [ 15:0] axis_sgdma_cpl_dpar;
    wire [ 10:0] axis_sgdma_cpl_ctrl_qid;
    wire [  1:0] axis_sgdma_cpl_ctrl_cmpt_type;
    wire [ 15:0] axis_sgdma_cpl_ctrl_wait_pld_pkt_id;
    wire [  2:0] axis_sgdma_cpl_ctrl_port_id;
    wire         axis_sgdma_cpl_ctrl_marker;
    wire         axis_sgdma_cpl_ctrl_user_trig;
    wire [  2:0] axis_sgdma_cpl_ctrl_col_idx;
    wire [  2:0] axis_sgdma_cpl_ctrl_err_idx;
    wire         axis_sgdma_cpl_ctrl_no_wrb_marker;
    wire         axis_sgdma_cpl_tready;

    wire axil_resetn;
    wire axis_resetn;


    wire pcie_refclk_gt;
    wire pcie_refclk;

    IBUFDS_GTE4 pcie_refclk_buf (
        .CEB  (1'b0),
        .I    (pcie_refclk_p),
        .IB   (pcie_refclk_n),
        .O    (pcie_refclk_gt),
        .ODIV2(pcie_refclk)
    );


    dma_subsystem_wrapper dma_wrapper_inst (
        .camera_frame_irq(camera_frame_irq),

        .pcie_rxp(pcie_rxp),
        .pcie_rxn(pcie_rxn),
        .pcie_txp(pcie_txp),
        .pcie_txn(pcie_txn),

        .m_axil_awvalid(m_axil_pcie_awvalid),
        .m_axil_awaddr (m_axil_pcie_awaddr),
        .m_axil_awready(m_axil_pcie_awready),
        .m_axil_wvalid (m_axil_pcie_wvalid),
        .m_axil_wdata  (m_axil_pcie_wdata),
        .m_axil_wready (m_axil_pcie_wready),
        .m_axil_bvalid (m_axil_pcie_bvalid),
        .m_axil_bresp  (m_axil_pcie_bresp),
        .m_axil_bready (m_axil_pcie_bready),
        .m_axil_arvalid(m_axil_pcie_arvalid),
        .m_axil_araddr (m_axil_pcie_araddr),
        .m_axil_arready(m_axil_pcie_arready),
        .m_axil_rvalid (m_axil_pcie_rvalid),
        .m_axil_rdata  (m_axil_pcie_rdata),
        .m_axil_rresp  (m_axil_pcie_rresp),
        .m_axil_rready (m_axil_pcie_rready),

        .m_axi_awready(m_axi_awready),
        .m_axi_wready (m_axi_wready),
        .m_axi_bid    (m_axi_bid),
        .m_axi_bresp  (m_axi_bresp),
        .m_axi_bvalid (m_axi_bvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid    (m_axi_rid),
        .m_axi_rdata  (m_axi_rdata),
        .m_axi_rresp  (m_axi_rresp),
        .m_axi_rlast  (m_axi_rlast),
        .m_axi_rvalid (m_axi_rvalid),
        .m_axi_awid   (m_axi_awid),
        .m_axi_awaddr (m_axi_awaddr),
        .m_axi_awuser (m_axi_awuser),
        .m_axi_awlen  (m_axi_awlen),
        .m_axi_awsize (m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awprot (m_axi_awprot),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awlock (m_axi_awlock),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_wdata  (m_axi_wdata),
        .m_axi_wuser  (m_axi_wuser),
        .m_axi_wstrb  (m_axi_wstrb),
        .m_axi_wlast  (m_axi_wlast),
        .m_axi_wvalid (m_axi_wvalid),
        .m_axi_bready (m_axi_bready),
        .m_axi_arid   (m_axi_arid),
        .m_axi_araddr (m_axi_araddr),
        .m_axi_aruser (m_axi_aruser),
        .m_axi_arlen  (m_axi_arlen),
        .m_axi_arsize (m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arprot (m_axi_arprot),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arlock (m_axi_arlock),
        .m_axi_arcache(m_axi_arcache),
        .m_axi_rready (m_axi_rready),

        .m_axis_h2c_tvalid         (axis_sgdma_h2c_tvalid),
        .m_axis_h2c_tdata          (axis_sgdma_h2c_tdata),
        .m_axis_h2c_tcrc           (axis_sgdma_h2c_tcrc),
        .m_axis_h2c_tlast          (axis_sgdma_h2c_tlast),
        .m_axis_h2c_tuser_qid      (axis_sgdma_h2c_tuser_qid),
        .m_axis_h2c_tuser_port_id  (axis_sgdma_h2c_tuser_port_id),
        .m_axis_h2c_tuser_err      (axis_sgdma_h2c_tuser_err),
        .m_axis_h2c_tuser_mdata    (axis_sgdma_h2c_tuser_mdata),
        .m_axis_h2c_tuser_mty      (axis_sgdma_h2c_tuser_mty),
        .m_axis_h2c_tuser_zero_byte(axis_sgdma_h2c_tuser_zero_byte),
        .m_axis_h2c_tready         (axis_sgdma_h2c_tready),

        .s_axis_c2h_tvalid       (axis_sgdma_c2h_tvalid),
        .s_axis_c2h_tdata        (axis_sgdma_c2h_tdata),
        .s_axis_c2h_tcrc         (axis_sgdma_c2h_tcrc),
        .s_axis_c2h_tlast        (axis_sgdma_c2h_tlast),
        .s_axis_c2h_ctrl_marker  (axis_sgdma_c2h_ctrl_marker),
        .s_axis_c2h_ctrl_port_id (axis_sgdma_c2h_ctrl_port_id),
        .s_axis_c2h_ctrl_ecc     (axis_sgdma_c2h_ctrl_ecc),
        .s_axis_c2h_ctrl_len     (axis_sgdma_c2h_ctrl_len),
        .s_axis_c2h_ctrl_qid     (axis_sgdma_c2h_ctrl_qid),
        .s_axis_c2h_ctrl_has_cmpt(axis_sgdma_c2h_ctrl_has_cmpt),
        .s_axis_c2h_mty          (axis_sgdma_c2h_mty),
        .s_axis_c2h_tready       (axis_sgdma_c2h_tready),

        .s_axis_cpl_tvalid              (axis_sgdma_cpl_tvalid),
        .s_axis_cpl_tdata               (axis_sgdma_cpl_tdata),
        .s_axis_cpl_size                (axis_sgdma_cpl_size),
        .s_axis_cpl_dpar                (axis_sgdma_cpl_dpar),
        .s_axis_cpl_ctrl_qid            (axis_sgdma_cpl_ctrl_qid),
        .s_axis_cpl_ctrl_cmpt_type      (axis_sgdma_cpl_ctrl_cmpt_type),
        .s_axis_cpl_ctrl_wait_pld_pkt_id(axis_sgdma_cpl_ctrl_wait_pld_pkt_id),
        .s_axis_cpl_ctrl_port_id        (axis_sgdma_cpl_ctrl_port_id),
        .s_axis_cpl_ctrl_marker         (axis_sgdma_cpl_ctrl_marker),
        .s_axis_cpl_ctrl_user_trig      (axis_sgdma_cpl_ctrl_user_trig),
        .s_axis_cpl_ctrl_col_idx        (axis_sgdma_cpl_ctrl_col_idx),
        .s_axis_cpl_ctrl_err_idx        (axis_sgdma_cpl_ctrl_err_idx),
        .s_axis_cpl_ctrl_no_wrb_marker  (axis_sgdma_cpl_ctrl_no_wrb_marker),
        .s_axis_cpl_tready              (axis_sgdma_cpl_tready),


        .s_axib_awid    (s_axib_awid),
        .s_axib_awaddr  (s_axib_awaddr),
        .s_axib_awregion(s_axib_awregion),
        .s_axib_awlen   (s_axib_awlen),
        .s_axib_awsize  (s_axib_awsize),
        .s_axib_awburst (s_axib_awburst),
        .s_axib_awvalid (s_axib_awvalid),
        .s_axib_wdata   (s_axib_wdata),
        .s_axib_wstrb   (s_axib_wstrb),
        .s_axib_wlast   (s_axib_wlast),
        .s_axib_wvalid  (s_axib_wvalid),
        .s_axib_wuser   (s_axib_wuser),
        .s_axib_ruser   (s_axib_ruser),
        .s_axib_bready  (s_axib_bready),
        .s_axib_arid    (s_axib_arid),
        .s_axib_araddr  (s_axib_araddr),
        .s_axib_aruser  (s_axib_aruser),
        .s_axib_awuser  (s_axib_awuser),
        .s_axib_arregion(s_axib_arregion),
        .s_axib_arlen   (s_axib_arlen),
        .s_axib_arsize  (s_axib_arsize),
        .s_axib_arburst (s_axib_arburst),
        .s_axib_arvalid (s_axib_arvalid),
        .s_axib_rready  (s_axib_rready),
        .s_axib_awready (s_axib_awready),
        .s_axib_wready  (s_axib_wready),
        .s_axib_bid     (s_axib_bid),
        .s_axib_bresp   (s_axib_bresp),
        .s_axib_bvalid  (s_axib_bvalid),
        .s_axib_arready (s_axib_arready),
        .s_axib_rid     (s_axib_rid),
        .s_axib_rdata   (s_axib_rdata),
        .s_axib_rresp   (s_axib_rresp),
        .s_axib_rlast   (s_axib_rlast),
        .s_axib_rvalid  (s_axib_rvalid),

        .pcie_refclk   (pcie_refclk),
        .pcie_refclk_gt(pcie_refclk_gt),
        .pcie_rstn     (pcie_rstn),
        .user_lnk_up   (user_lnk_up),
        .phy_ready     (phy_ready),

        .soft_reset_n(mod_rstn),

        .axil_aclk(axil_aclk),
        .axis_aclk(axis_aclk),
        .cmlk_ref_clk(cmlk_ref_clk),
        .cmlk_idelay_ref_clk(cmlk_idelay_ref_clk),
        .cmlk_idelay_ref_locked(cmlk_idelay_ref_locked),
        .aresetn  (powerup_rstn)
    );

`ifdef __synthesis__
    assign axil_resetn = powerup_rstn;
    assign axis_resetn = powerup_rstn;
    assign mod_rst_done = powerup_rstn;
`else
    assign axil_resetn = mod_rstn;
    assign axis_resetn = mod_rstn;
    assign mod_rst_done = mod_rstn;
`endif

    assign axis_sgdma_cpl_tvalid               = 1'b0;
    assign axis_sgdma_cpl_tdata                = 512'd0;
    assign axis_sgdma_cpl_size                 = 2'd0;
    assign axis_sgdma_cpl_dpar                 = 16'd0;
    assign axis_sgdma_cpl_ctrl_qid             = 11'd0;
    assign axis_sgdma_cpl_ctrl_cmpt_type       = 2'd0;
    assign axis_sgdma_cpl_ctrl_wait_pld_pkt_id = 16'd0;
    assign axis_sgdma_cpl_ctrl_port_id         = 3'd0;
    assign axis_sgdma_cpl_ctrl_marker          = 1'b0;
    assign axis_sgdma_cpl_ctrl_user_trig       = 1'b0;
    assign axis_sgdma_cpl_ctrl_col_idx         = 3'd0;
    assign axis_sgdma_cpl_ctrl_err_idx         = 3'd0;
    assign axis_sgdma_cpl_ctrl_no_wrb_marker   = 1'b0;

    function automatic [63:0] keep_from_mty(input [5:0] mty);
        begin
            if (mty == 6'd0) begin
                keep_from_mty = 64'hffff_ffff_ffff_ffff;
            end else begin
                keep_from_mty = 64'hffff_ffff_ffff_ffff >> mty;
            end
        end
    endfunction

    function automatic [5:0] mty_from_len(input [15:0] len);
        reg [5:0] rem;
        reg [6:0] empty;
        begin
            rem = len[5:0];
            empty = 7'd64 - {1'b0, rem};
            mty_from_len = (rem == 6'd0) ? 6'd0 : empty[5:0];
        end
    endfunction

    /* ----------------------------------------------------------------------
     * Minimal AXI-Lite register terminator.
     *   0x000: RX queue config {qbase[15:0], numq[15:0]}
     *   0x800/0x804: H2C packet counter low/high
     *   0x808/0x80c: H2C byte counter low/high
     *   0x900/0x904: C2H packet counter low/high
     *   0x908/0x90c: C2H byte counter low/high
     * Writes to old RSS table/hash addresses are acknowledged but ignored.
     * ---------------------------------------------------------------------- */
    localparam [11:0] REG_QCONF   = 12'h000;
    localparam [11:0] REG_H2C_PKTL = 12'h800;
    localparam [11:0] REG_H2C_PKTH = 12'h804;
    localparam [11:0] REG_H2C_BYTL = 12'h808;
    localparam [11:0] REG_H2C_BYTH = 12'h80c;
    localparam [11:0] REG_C2H_PKTL = 12'h900;
    localparam [11:0] REG_C2H_PKTH = 12'h904;
    localparam [11:0] REG_C2H_BYTL = 12'h908;
    localparam [11:0] REG_C2H_BYTH = 12'h90c;

    reg        axil_aw_seen_reg = 1'b0;
    reg [31:0] axil_awaddr_reg  = 32'd0;
    reg        axil_w_seen_reg  = 1'b0;
    reg [31:0] axil_wdata_reg   = 32'd0;
    reg        axil_bvalid_reg  = 1'b0;
    reg        axil_rvalid_reg  = 1'b0;
    reg [31:0] axil_rdata_reg   = 32'd0;
    reg [15:0] rx_qbase_reg     = DEFAULT_RX_QBASE;
    reg [15:0] rx_numq_reg      = DEFAULT_RX_NUMQ;

    wire axil_aw_fire = s_axil_awvalid & s_axil_awready;
    wire axil_w_fire  = s_axil_wvalid  & s_axil_wready;
    wire axil_wr_fire = (axil_aw_seen_reg | axil_aw_fire) & (axil_w_seen_reg | axil_w_fire) & ~axil_bvalid_reg;
    wire [31:0] axil_wr_addr = axil_aw_fire ? s_axil_awaddr : axil_awaddr_reg;
    wire [31:0] axil_wr_data = axil_w_fire  ? s_axil_wdata  : axil_wdata_reg;

    assign s_axil_awready = ~axil_aw_seen_reg & ~axil_bvalid_reg;
    assign s_axil_wready  = ~axil_w_seen_reg  & ~axil_bvalid_reg;
    assign s_axil_bvalid  = axil_bvalid_reg;
    assign s_axil_bresp   = 2'b00;
    assign s_axil_arready = ~axil_rvalid_reg;
    assign s_axil_rvalid  = axil_rvalid_reg;
    assign s_axil_rdata   = axil_rdata_reg;
    assign s_axil_rresp   = 2'b00;

    reg [47:0] h2c_pkt_cnt = 48'd0;
    reg [47:0] h2c_byte_cnt = 48'd0;
    reg [47:0] c2h_pkt_cnt = 48'd0;
    reg [47:0] c2h_byte_cnt = 48'd0;

    wire [31:0] axil_read_data =
        (s_axil_araddr[11:0] == REG_QCONF)   ? {rx_qbase_reg, rx_numq_reg} :
        (s_axil_araddr[11:0] == REG_H2C_PKTL) ? h2c_pkt_cnt[31:0] :
        (s_axil_araddr[11:0] == REG_H2C_PKTH) ? {16'd0, h2c_pkt_cnt[47:32]} :
        (s_axil_araddr[11:0] == REG_H2C_BYTL) ? h2c_byte_cnt[31:0] :
        (s_axil_araddr[11:0] == REG_H2C_BYTH) ? {16'd0, h2c_byte_cnt[47:32]} :
        (s_axil_araddr[11:0] == REG_C2H_PKTL) ? c2h_pkt_cnt[31:0] :
        (s_axil_araddr[11:0] == REG_C2H_PKTH) ? {16'd0, c2h_pkt_cnt[47:32]} :
        (s_axil_araddr[11:0] == REG_C2H_BYTL) ? c2h_byte_cnt[31:0] :
        (s_axil_araddr[11:0] == REG_C2H_BYTH) ? {16'd0, c2h_byte_cnt[47:32]} :
                                                 32'd0;

    always @(posedge axil_aclk) begin
        if (!axil_resetn) begin
            axil_aw_seen_reg <= 1'b0;
            axil_awaddr_reg  <= 32'd0;
            axil_w_seen_reg  <= 1'b0;
            axil_wdata_reg   <= 32'd0;
            axil_bvalid_reg  <= 1'b0;
            axil_rvalid_reg  <= 1'b0;
            axil_rdata_reg   <= 32'd0;
            rx_qbase_reg     <= DEFAULT_RX_QBASE;
            rx_numq_reg      <= DEFAULT_RX_NUMQ;
        end else begin
            if (axil_aw_fire) begin
                axil_aw_seen_reg <= 1'b1;
                axil_awaddr_reg  <= s_axil_awaddr;
            end
            if (axil_w_fire) begin
                axil_w_seen_reg <= 1'b1;
                axil_wdata_reg  <= s_axil_wdata;
            end

            if (axil_wr_fire) begin
                axil_aw_seen_reg <= 1'b0;
                axil_w_seen_reg  <= 1'b0;
                axil_bvalid_reg  <= 1'b1;
                if (axil_wr_addr[11:0] == REG_QCONF) begin
                    // sgdma queue layout reserves qid 0-31 for TX and 32-63 for RX.
                    // Clamp legacy qbase=0 software writes to the sgdma RX base.
                    rx_qbase_reg <= (axil_wr_data[31:16] < DEFAULT_RX_QBASE) ?
                                    DEFAULT_RX_QBASE : axil_wr_data[31:16];
                    rx_numq_reg  <= (axil_wr_data[15:0] == 16'd0) ? 16'd1 : axil_wr_data[15:0];
                end
            end else if (axil_bvalid_reg && s_axil_bready) begin
                axil_bvalid_reg <= 1'b0;
            end

            if (s_axil_arvalid && s_axil_arready) begin
                axil_rvalid_reg <= 1'b1;
                axil_rdata_reg  <= axil_read_data;
            end else if (axil_rvalid_reg && s_axil_rready) begin
                axil_rvalid_reg <= 1'b0;
            end
        end
    end

    /* ----------------------------------------------------------------------
     * H2C/TX: sgdma wrapper 512-bit stream -> network TX stream.  No PF
     * filtering, no qid filtering, no function table.  sgdma already selects
     * the host TX queue and supplies packet length/mty.
     * ---------------------------------------------------------------------- */
    wire        h2c_out_valid = axis_sgdma_h2c_tvalid;
    wire [15:0] h2c_out_size  = axis_sgdma_h2c_tuser_mdata[31:16];
    wire [63:0] h2c_out_keep  = axis_sgdma_h2c_tlast ? keep_from_mty(axis_sgdma_h2c_tuser_mty) : 64'hffff_ffff_ffff_ffff;

    assign axis_sgdma_h2c_tready = m_axis_h2c_tready[0];

    generate
        for (genvar pf = 0; pf < NUM_PHYS_FUNC; pf = pf + 1) begin : g_pf_ports
            if (pf == 0) begin : g_pf0
                assign m_axis_h2c_tvalid[pf]              = h2c_out_valid;
                assign m_axis_h2c_tdata[512*pf +: 512]   = axis_sgdma_h2c_tdata;
                assign m_axis_h2c_tkeep[ 64*pf +:  64]   = h2c_out_keep;
                assign m_axis_h2c_tlast[pf]               = axis_sgdma_h2c_tlast;
                assign m_axis_h2c_tuser_size[16*pf +: 16] = h2c_out_size;
                assign m_axis_h2c_tuser_src[ 16*pf +: 16] = 16'd0;
                assign m_axis_h2c_tuser_dst[ 16*pf +: 16] = 16'd0;
            end else begin : g_unused_pf
                assign m_axis_h2c_tvalid[pf]              = 1'b0;
                assign m_axis_h2c_tdata[512*pf +: 512]   = 512'd0;
                assign m_axis_h2c_tkeep[ 64*pf +:  64]   = 64'd0;
                assign m_axis_h2c_tlast[pf]               = 1'b0;
                assign m_axis_h2c_tuser_size[16*pf +: 16] = 16'd0;
                assign m_axis_h2c_tuser_src[ 16*pf +: 16] = 16'd0;
                assign m_axis_h2c_tuser_dst[ 16*pf +: 16] = 16'd0;
            end
        end
    endgenerate

    /* ----------------------------------------------------------------------
     * C2H/RX: network RX stream -> sgdma wrapper C2H stream.  The old RSS hash
     * module is replaced with a small round-robin qid selector over the RX
     * qbase/numq range.  This preserves multi-RX-queue operation without the
     * OpenNIC function/RSS hierarchy.
     * ---------------------------------------------------------------------- */
    reg [15:0] rx_qbase_sync1 = DEFAULT_RX_QBASE;
    reg [15:0] rx_qbase_sync2 = DEFAULT_RX_QBASE;
    reg [15:0] rx_numq_sync1  = DEFAULT_RX_NUMQ;
    reg [15:0] rx_numq_sync2  = DEFAULT_RX_NUMQ;
    reg [15:0] rx_rr_reg      = 16'd0;
    reg [10:0] c2h_active_qid_reg = 11'd0;
    reg        c2h_in_pkt_reg = 1'b0;

    always @(posedge axis_aclk) begin
        if (!axis_resetn) begin
            rx_qbase_sync1 <= DEFAULT_RX_QBASE;
            rx_qbase_sync2 <= DEFAULT_RX_QBASE;
            rx_numq_sync1  <= DEFAULT_RX_NUMQ;
            rx_numq_sync2  <= DEFAULT_RX_NUMQ;
        end else begin
            rx_qbase_sync1 <= rx_qbase_reg;
            rx_qbase_sync2 <= rx_qbase_sync1;
            rx_numq_sync1  <= (rx_numq_reg == 16'd0) ? 16'd1 : rx_numq_reg;
            rx_numq_sync2  <= rx_numq_sync1;
        end
    end

    wire        c2h_in_valid = s_axis_c2h_tvalid[0];
    wire [511:0] c2h_in_data = s_axis_c2h_tdata[0 +: 512];
    wire        c2h_in_last  = s_axis_c2h_tlast[0];
    wire [15:0] c2h_in_size  = s_axis_c2h_tuser_size[0 +: 16];
    wire        c2h_fire     = c2h_in_valid & axis_sgdma_c2h_tready;
    wire [15:0] rx_numq_eff  = (rx_numq_sync2 == 16'd0) ? 16'd1 : rx_numq_sync2;
    wire [16:0] c2h_qid_sum  = {1'b0, rx_qbase_sync2} + {1'b0, rx_rr_reg};
    wire [10:0] c2h_next_qid = c2h_qid_sum[10:0];
    wire [10:0] c2h_out_qid  = c2h_in_pkt_reg ? c2h_active_qid_reg : c2h_next_qid;

    assign axis_sgdma_c2h_tvalid        = c2h_in_valid;
    assign axis_sgdma_c2h_tdata         = c2h_in_data;
    assign axis_sgdma_c2h_tcrc          = 32'd0;
    assign axis_sgdma_c2h_tlast         = c2h_in_last;
    assign axis_sgdma_c2h_ctrl_marker   = 1'b0;
    assign axis_sgdma_c2h_ctrl_port_id  = 3'd0;
    assign axis_sgdma_c2h_ctrl_ecc      = 7'd0;
    assign axis_sgdma_c2h_ctrl_len      = c2h_in_size;
    assign axis_sgdma_c2h_ctrl_qid      = c2h_out_qid;
    assign axis_sgdma_c2h_ctrl_has_cmpt = 1'b0;
    assign axis_sgdma_c2h_mty           = c2h_in_last ? mty_from_len(c2h_in_size) : 6'd0;

    generate
        for (genvar rxpf = 0; rxpf < NUM_PHYS_FUNC; rxpf = rxpf + 1) begin : g_c2h_ready
            if (rxpf == 0) begin : g_c2h_pf0
                assign s_axis_c2h_tready[rxpf] = axis_sgdma_c2h_tready;
            end else begin : g_c2h_unused_pf
                assign s_axis_c2h_tready[rxpf] = 1'b1;
            end
        end
    endgenerate

    always @(posedge axis_aclk) begin
        if (!axis_resetn) begin
            rx_rr_reg          <= 16'd0;
            c2h_active_qid_reg <= DEFAULT_RX_QBASE[10:0];
            c2h_in_pkt_reg     <= 1'b0;
        end else if (c2h_fire) begin
            if (!c2h_in_pkt_reg) begin
                c2h_active_qid_reg <= c2h_next_qid;
            end

            if (c2h_in_last) begin
                c2h_in_pkt_reg <= 1'b0;
                if (rx_rr_reg >= rx_numq_eff - 16'd1) begin
                    rx_rr_reg <= 16'd0;
                end else begin
                    rx_rr_reg <= rx_rr_reg + 16'd1;
                end
            end else begin
                c2h_in_pkt_reg <= 1'b1;
            end
        end
    end

    always @(posedge axis_aclk) begin
        if (!axis_resetn) begin
            h2c_pkt_cnt  <= 48'd0;
            h2c_byte_cnt <= 48'd0;
            c2h_pkt_cnt  <= 48'd0;
            c2h_byte_cnt <= 48'd0;
        end else begin
            if (axis_sgdma_h2c_tvalid && axis_sgdma_h2c_tready && axis_sgdma_h2c_tlast) begin
                h2c_pkt_cnt  <= h2c_pkt_cnt + 48'd1;
                h2c_byte_cnt <= h2c_byte_cnt + {32'd0, h2c_out_size};
            end
            if (axis_sgdma_c2h_tvalid && axis_sgdma_c2h_tready && axis_sgdma_c2h_tlast) begin
                c2h_pkt_cnt  <= c2h_pkt_cnt + 48'd1;
                c2h_byte_cnt <= c2h_byte_cnt + {32'd0, c2h_in_size};
            end
        end
    end


endmodule : dma_subsystem
