// *************************************************************************
//
// Copyright 2020 Xilinx, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// *************************************************************************
// This module wraps up the sgdma IP.  It creates clock domains for the 250MHz
// data path, 125MHz AXI-Lite path, 200MHz CameraLink logic reference, and a
// 400MHz IDELAYCTRL calibration reference.
`timescale 1ns / 1ps
module dma_subsystem_wrapper (
    input  [7:0] pcie_rxp,
    input  [7:0] pcie_rxn,
    output [7:0] pcie_txp,
    output [7:0] pcie_txn,

    output        m_axil_awvalid,
    output [31:0] m_axil_awaddr,
    input         m_axil_awready,
    output        m_axil_wvalid,
    output [31:0] m_axil_wdata,
    input         m_axil_wready,
    input         m_axil_bvalid,
    input  [ 1:0] m_axil_bresp,
    output        m_axil_bready,
    output        m_axil_arvalid,
    output [31:0] m_axil_araddr,
    input         m_axil_arready,
    input         m_axil_rvalid,
    input  [31:0] m_axil_rdata,
    input  [ 1:0] m_axil_rresp,
    output        m_axil_rready,

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
    output [ 31:0] m_axi_wuser,
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

    output         m_axis_h2c_tvalid,
    output [511:0] m_axis_h2c_tdata,
    output [ 31:0] m_axis_h2c_tcrc,
    output         m_axis_h2c_tlast,
    output [ 10:0] m_axis_h2c_tuser_qid,
    output [  2:0] m_axis_h2c_tuser_port_id,
    output         m_axis_h2c_tuser_err,
    output [ 31:0] m_axis_h2c_tuser_mdata,
    output [  5:0] m_axis_h2c_tuser_mty,
    output         m_axis_h2c_tuser_zero_byte,
    input          m_axis_h2c_tready,

    input          s_axis_c2h_tvalid,
    input  [511:0] s_axis_c2h_tdata,
    input  [ 31:0] s_axis_c2h_tcrc,
    input          s_axis_c2h_tlast,
    input          s_axis_c2h_ctrl_marker,
    input  [  2:0] s_axis_c2h_ctrl_port_id,
    input  [  6:0] s_axis_c2h_ctrl_ecc,
    input  [ 15:0] s_axis_c2h_ctrl_len,
    input  [ 10:0] s_axis_c2h_ctrl_qid,
    input          s_axis_c2h_ctrl_has_cmpt,
    input  [  5:0] s_axis_c2h_mty,
    output         s_axis_c2h_tready,

    input          s_axis_cpl_tvalid,
    input  [511:0] s_axis_cpl_tdata,
    input  [  1:0] s_axis_cpl_size,
    input  [ 15:0] s_axis_cpl_dpar,
    input  [ 10:0] s_axis_cpl_ctrl_qid,
    input  [  1:0] s_axis_cpl_ctrl_cmpt_type,
    input  [ 15:0] s_axis_cpl_ctrl_wait_pld_pkt_id,
    input  [  2:0] s_axis_cpl_ctrl_port_id,
    input          s_axis_cpl_ctrl_marker,
    input          s_axis_cpl_ctrl_user_trig,
    input  [  2:0] s_axis_cpl_ctrl_col_idx,
    input  [  2:0] s_axis_cpl_ctrl_err_idx,
    input          s_axis_cpl_ctrl_no_wrb_marker,
    output         s_axis_cpl_tready,


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
    input  [ 31:0] s_axib_wuser,
    output [ 31:0] s_axib_ruser,
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

    input camera_frame_irq,
    input pcie_refclk,
    input pcie_refclk_gt,
    input pcie_rstn,

    output user_lnk_up,
    output phy_ready,
    input  soft_reset_n,

    output axis_aclk,
    output axil_aclk,
    output cmlk_ref_clk,
    output cmlk_idelay_ref_clk,
    output cmlk_idelay_ref_locked,
    output aresetn
);

    // sgdma replaces the generated sgdma IP, but this wrapper intentionally keeps
    // the old sgdma_subsystem port contract.  The surrounding OpenNIC-style
    // modules still see 512-bit H2C/C2H streams, while the internal sgdma core
    // runs on the 256-bit PCIe user interface.

    localparam AXIS_PCIE_DATA_WIDTH = 256;
    localparam AXIS_PCIE_KEEP_WIDTH = AXIS_PCIE_DATA_WIDTH / 32;
    localparam AXIS_PCIE_RC_USER_WIDTH = 75;
    localparam AXIS_PCIE_RQ_USER_WIDTH = 62;
    localparam AXIS_PCIE_CQ_USER_WIDTH = 88;
    localparam AXIS_PCIE_CC_USER_WIDTH = 33;
    localparam AXI_256_DATA_WIDTH = 256;
    localparam AXI_256_STRB_WIDTH = AXI_256_DATA_WIDTH / 8;
    localparam AXI_256_ADDR_WIDTH = 64;
    localparam AXI_256_ID_WIDTH = 4;
    localparam AXIL_DATA_WIDTH = 32;
    localparam AXIL_ADDR_WIDTH = 32;
    localparam AXIL_STRB_WIDTH = AXIL_DATA_WIDTH / 8;
    localparam DMA_QID_WIDTH = 9;
    localparam DMA_LEN_WIDTH = 24;
    localparam ST_AXIS_USER_WIDTH = 32;
    localparam ST_AXIS_MTY_WIDTH = 6;
    localparam RQ_SEQ_NUM_WIDTH = AXIS_PCIE_RQ_USER_WIDTH == 60 ? 4 : 6;
    localparam AXI_DATA_WIDTH = AXIS_PCIE_DATA_WIDTH;
    localparam AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8;
    localparam PCIE_ADDR_WIDTH = 64;
    localparam AXI_ADDR_WIDTH = 64;
    localparam AXI_ID_WIDTH = 4;
    localparam ST_AXIS_KEEP_WIDTH = AXIS_PCIE_DATA_WIDTH / 8;







    wire       aclk_250mhz;
    wire       pcie_user_reset;
    wire       aresetn_250mhz;
    wire       datapath_aresetn_250mhz;
    reg  [1:0] aresetn_sync = 2'b11;

    assign axis_aclk               = aclk_250mhz;

    // Keep the PCIe completer/sgdma control path out of the shell soft-reset loop.
    // BAR0/BAR2 MMIO requests are completed by sgdma; if sgdma is reset by
    // soft_reset_n while the shell waits for mod_rst_done, host MMIO reads can
    // time out with PCIe AER CmpltTO.  Therefore the exported aresetn/powerup
    // reset follows only the PCIe hard-IP user reset.  The optional shell soft
    // reset is limited to packet datapath adapters below.
    assign aresetn_250mhz          = ~pcie_user_reset;
    assign datapath_aresetn_250mhz = soft_reset_n & aresetn_250mhz;

    // The old wrapper exported AXI user sidebands, but the sgdma payload path
    // does not use them.  Keep them deterministic for downstream logic.
    assign m_axi_awuser            = 32'd0;
    assign m_axi_wuser             = 32'd0;
    assign m_axi_aruser            = 32'd0;

    // Generate 125 MHz AXI-Lite clock and 200 MHz CameraLink reference clock
    // from the PCIe user clock.  IDELAYCTRL uses a separate 400 MHz calibration
    // reference because the UltraScale IDELAYCTRL primitive requires >=300 MHz.
    sgdma_subsystem_clk_div clk_div_inst (
        .clk_in1 (axis_aclk),
        .clk_out1(axil_aclk),
        .clk_out2(cmlk_ref_clk),
        .locked  ()
    );

    cmlk_idelay_refclk_gen cmlk_idelay_refclk_gen_inst (
        .clk_in (axis_aclk),
        .clk_out(cmlk_idelay_ref_clk),
        .locked (cmlk_idelay_ref_locked)
    );

    // Generate reset w.r.t. the 125 MHz AXI-Lite clock.
    assign aresetn = aresetn_sync[1];
    always @(posedge axil_aclk) begin
        aresetn_sync[0] <= aresetn_250mhz;
        aresetn_sync[1] <= aresetn_sync[0];
    end

    /* ---------------- PCIe hard IP user AXIS signals ---------------- */

    wire [   AXIS_PCIE_DATA_WIDTH-1:0] axis_rq_tdata;
    wire [   AXIS_PCIE_KEEP_WIDTH-1:0] axis_rq_tkeep;
    wire                               axis_rq_tlast;
    wire                               axis_rq_tready;
    wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis_rq_tuser;
    wire                               axis_rq_tvalid;

    /* Posted Memory Write requests do not return RC completions.  The sgdma
     * payload write engine therefore uses the UltraScale+ RQ sequence-number
     * feedback from the PCIe hard block to retire D2H/MM-write operations and
     * generate descriptor status.  Leaving these outputs unconnected makes
     * MM D2H descriptors issue on RQ but prevents write-status completion, so
     * software observes a timeout with no error completion.
     */
    wire [       RQ_SEQ_NUM_WIDTH-1:0] axis_rq_seq_num0;
    wire                               axis_rq_seq_num_vld0;
    wire [       RQ_SEQ_NUM_WIDTH-1:0] axis_rq_seq_num1;
    wire                               axis_rq_seq_num_vld1;

    wire [   AXIS_PCIE_DATA_WIDTH-1:0] axis_rc_tdata;
    wire [   AXIS_PCIE_KEEP_WIDTH-1:0] axis_rc_tkeep;
    wire                               axis_rc_tlast;
    wire                               axis_rc_tready;
    wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis_rc_tuser;
    wire                               axis_rc_tvalid;

    wire [   AXIS_PCIE_DATA_WIDTH-1:0] axis_cq_tdata;
    wire [   AXIS_PCIE_KEEP_WIDTH-1:0] axis_cq_tkeep;
    wire                               axis_cq_tlast;
    wire                               axis_cq_tready;
    wire [AXIS_PCIE_CQ_USER_WIDTH-1:0] axis_cq_tuser;
    wire                               axis_cq_tvalid;

    wire [   AXIS_PCIE_DATA_WIDTH-1:0] axis_cc_tdata;
    wire [   AXIS_PCIE_KEEP_WIDTH-1:0] axis_cc_tkeep;
    wire                               axis_cc_tlast;
    wire                               axis_cc_tready;
    wire [AXIS_PCIE_CC_USER_WIDTH-1:0] axis_cc_tuser;
    wire                               axis_cc_tvalid;

    wire [                        2:0] cfg_max_payload;
    wire [                        2:0] cfg_max_read_req;
    wire [                        7:0] cfg_fc_ph;
    wire [                       11:0] cfg_fc_pd;
    wire [                        7:0] cfg_fc_nph;
    wire [                       11:0] cfg_fc_npd;

    wire [                        9:0] cfg_mgmt_addr;
    wire [                        7:0] cfg_mgmt_function_number;
    wire                               cfg_mgmt_write;
    wire [                       31:0] cfg_mgmt_write_data;
    wire [                        3:0] cfg_mgmt_byte_enable;
    wire                               cfg_mgmt_read;
    wire [                       31:0] cfg_mgmt_read_data;
    wire                               cfg_mgmt_read_write_done;

    wire                               cfg_interrupt_msi_sent;
    wire                               cfg_interrupt_msi_fail;
    wire [                        7:0] cfg_interrupt_msi_function_number;
    wire [                        3:0] cfg_interrupt_msix_enable;
    wire [                        3:0] cfg_interrupt_msix_mask;
    wire [                       63:0] cfg_interrupt_msix_address;
    wire [                       31:0] cfg_interrupt_msix_data;
    wire                               cfg_interrupt_msix_int;
    wire [                        1:0] cfg_interrupt_msix_vec_pending;
    wire                               cfg_interrupt_msix_vec_pending_status;

    wire                               status_error_cor;
    wire                               status_error_uncor;

    pcie4_uscale_plus_0 pcie4_uscale_plus_inst (
        .pci_exp_txn(pcie_txn),
        .pci_exp_txp(pcie_txp),
        .pci_exp_rxn(pcie_rxn),
        .pci_exp_rxp(pcie_rxp),
        .user_clk   (aclk_250mhz),
        .user_reset (pcie_user_reset),
        .user_lnk_up(user_lnk_up),

        .s_axis_rq_tdata (axis_rq_tdata),
        .s_axis_rq_tkeep (axis_rq_tkeep),
        .s_axis_rq_tlast (axis_rq_tlast),
        .s_axis_rq_tready(axis_rq_tready),
        .s_axis_rq_tuser (axis_rq_tuser),
        .s_axis_rq_tvalid(axis_rq_tvalid),

        .m_axis_rc_tdata (axis_rc_tdata),
        .m_axis_rc_tkeep (axis_rc_tkeep),
        .m_axis_rc_tlast (axis_rc_tlast),
        .m_axis_rc_tready(axis_rc_tready),
        .m_axis_rc_tuser (axis_rc_tuser),
        .m_axis_rc_tvalid(axis_rc_tvalid),

        .m_axis_cq_tdata (axis_cq_tdata),
        .m_axis_cq_tkeep (axis_cq_tkeep),
        .m_axis_cq_tlast (axis_cq_tlast),
        .m_axis_cq_tready(axis_cq_tready),
        .m_axis_cq_tuser (axis_cq_tuser),
        .m_axis_cq_tvalid(axis_cq_tvalid),

        .s_axis_cc_tdata (axis_cc_tdata),
        .s_axis_cc_tkeep (axis_cc_tkeep),
        .s_axis_cc_tlast (axis_cc_tlast),
        .s_axis_cc_tready(axis_cc_tready),
        .s_axis_cc_tuser (axis_cc_tuser),
        .s_axis_cc_tvalid(axis_cc_tvalid),

        .pcie_rq_seq_num0    (axis_rq_seq_num0),
        .pcie_rq_seq_num_vld0(axis_rq_seq_num_vld0),
        .pcie_rq_seq_num1    (axis_rq_seq_num1),
        .pcie_rq_seq_num_vld1(axis_rq_seq_num_vld1),
        .pcie_rq_tag0        (),
        .pcie_rq_tag1        (),
        .pcie_rq_tag_av      (),
        .pcie_rq_tag_vld0    (),
        .pcie_rq_tag_vld1    (),
        .pcie_tfc_nph_av     (),
        .pcie_tfc_npd_av     (),
        .pcie_cq_np_req      (2'b01),
        .pcie_cq_np_req_count(),

        .cfg_phy_link_down       (),
        .cfg_phy_link_status     (),
        .cfg_negotiated_width    (),
        .cfg_current_speed       (),
        .cfg_max_payload         (cfg_max_payload),
        .cfg_max_read_req        (cfg_max_read_req),
        .cfg_function_status     (),
        .cfg_function_power_state(),
        .cfg_vf_status           (),
        .cfg_vf_power_state      (),
        .cfg_link_power_state    (),

        .cfg_mgmt_addr           (cfg_mgmt_addr),
        .cfg_mgmt_function_number(cfg_mgmt_function_number),
        .cfg_mgmt_write          (cfg_mgmt_write),
        .cfg_mgmt_write_data     (cfg_mgmt_write_data),
        .cfg_mgmt_byte_enable    (cfg_mgmt_byte_enable),
        .cfg_mgmt_read           (cfg_mgmt_read),
        .cfg_mgmt_read_data      (cfg_mgmt_read_data),
        .cfg_mgmt_read_write_done(cfg_mgmt_read_write_done),
        .cfg_mgmt_debug_access   (1'b0),

        .cfg_err_cor_out            (),
        .cfg_err_nonfatal_out       (),
        .cfg_err_fatal_out          (),
        .cfg_local_error_valid      (),
        .cfg_local_error_out        (),
        .cfg_ltssm_state            (),
        .cfg_rx_pm_state            (),
        .cfg_tx_pm_state            (),
        .cfg_rcb_status             (),
        .cfg_obff_enable            (),
        .cfg_pl_status_change       (),
        .cfg_tph_requester_enable   (),
        .cfg_tph_st_mode            (),
        .cfg_vf_tph_requester_enable(),
        .cfg_vf_tph_st_mode         (),

        .cfg_msg_received     (),
        .cfg_msg_received_data(),
        .cfg_msg_received_type(),
        .cfg_msg_transmit     (1'b0),
        .cfg_msg_transmit_type(3'd0),
        .cfg_msg_transmit_data(32'd0),
        .cfg_msg_transmit_done(),

        .cfg_fc_ph                            (cfg_fc_ph),
        .cfg_fc_pd                            (cfg_fc_pd),
        .cfg_fc_nph                           (cfg_fc_nph),
        .cfg_fc_npd                           (cfg_fc_npd),
        .cfg_fc_cplh                          (),
        .cfg_fc_cpld                          (),
        // sgdma_core gates requester TLP issue on transmit FC credits.
        // Match the standalone zdma top: expose transmit user credits.
        .cfg_fc_sel                           (3'd4),
        .cfg_dsn                              (64'd0),
        .cfg_bus_number                       (),
        .cfg_power_state_change_ack           (1'b1),
        .cfg_power_state_change_interrupt     (),
        .cfg_err_cor_in                       (status_error_cor),
        .cfg_err_uncor_in                     (status_error_uncor),
        .cfg_flr_in_process                   (),
        .cfg_flr_done                         (4'd0),
        .cfg_vf_flr_in_process                (),
        .cfg_vf_flr_func_num                  (8'd0),
        .cfg_vf_flr_done                      (8'd0),
        .cfg_link_training_enable             (1'b1),
        .cfg_interrupt_int                    (4'd0),
        .cfg_interrupt_pending                (4'd0),
        .cfg_interrupt_sent                   (),
        .cfg_interrupt_msi_sent               (cfg_interrupt_msi_sent),
        .cfg_interrupt_msi_fail               (cfg_interrupt_msi_fail),
        .cfg_interrupt_msi_function_number    (cfg_interrupt_msi_function_number),
        .cfg_interrupt_msix_enable            (cfg_interrupt_msix_enable),
        .cfg_interrupt_msix_mask              (cfg_interrupt_msix_mask),
        .cfg_interrupt_msix_vf_enable         (),
        .cfg_interrupt_msix_vf_mask           (),
        .cfg_interrupt_msix_address           (cfg_interrupt_msix_address),
        .cfg_interrupt_msix_data              (cfg_interrupt_msix_data),
        .cfg_interrupt_msix_int               (cfg_interrupt_msix_int),
        .cfg_interrupt_msix_vec_pending       (cfg_interrupt_msix_vec_pending),
        .cfg_interrupt_msix_vec_pending_status(cfg_interrupt_msix_vec_pending_status),
        .cfg_pm_aspm_l1_entry_reject          (1'b0),
        .cfg_pm_aspm_tx_l0s_entry_disable     (1'b0),
        .cfg_hot_reset_out                    (),
        .cfg_config_space_enable              (1'b1),
        .cfg_req_pm_transition_l23_ready      (1'b0),
        .cfg_hot_reset_in                     (1'b0),
        .cfg_ds_port_number                   (8'd0),
        .cfg_ds_bus_number                    (8'd0),
        .cfg_ds_device_number                 (5'd0),
        .sys_clk                              (pcie_refclk),
        .sys_clk_gt                           (pcie_refclk_gt),
        .sys_reset                            (pcie_rstn),
        .phy_rdy_out                          (phy_ready)
    );

    /* ---------------- sgdma <-> legacy sgdma-stream adaptation ---------------- */

    wire                     sgdma_st_rx_tvalid;
    wire [            255:0] sgdma_st_rx_tdata;
    wire                     sgdma_st_rx_tlast;
    wire [             10:0] sgdma_st_rx_qid_wide;
    wire [             15:0] sgdma_st_rx_len16;
    wire [              5:0] sgdma_st_rx_mty;
    wire                     sgdma_st_rx_tready;

    wire                     sgdma_st_tx_tvalid;
    wire [            255:0] sgdma_st_tx_tdata;
    wire                     sgdma_st_tx_tlast;
    wire [              5:0] sgdma_st_tx_mty;
    wire [             31:0] sgdma_st_tx_tuser;
    wire [DMA_QID_WIDTH-1:0] sgdma_st_tx_tqid;
    wire [DMA_LEN_WIDTH-1:0] sgdma_st_tx_tlen;
    wire                     sgdma_st_tx_tready;

    wire                     c2h_256_tvalid;
    wire [            255:0] c2h_256_tdata;
    wire [             31:0] c2h_256_tcrc;
    wire                     c2h_256_tlast;
    wire                     c2h_256_ctrl_marker;
    wire [              2:0] c2h_256_ctrl_port_id;
    wire [              6:0] c2h_256_ctrl_ecc;
    wire [             15:0] c2h_256_ctrl_len;
    wire [             10:0] c2h_256_ctrl_qid;
    wire                     c2h_256_ctrl_has_cmpt;
    wire [              5:0] c2h_256_mty;
    wire                     c2h_256_tready;

    // Network RX / C2H: legacy 512-bit stream -> sgdma ST D2H/RX 256-bit stream.
    axis_dwconv_c2h_512_to_256 u_c2h_dwconv (
        .aclk   (aclk_250mhz),
        .aresetn(datapath_aresetn_250mhz),

        .s_axis_tvalid       (s_axis_c2h_tvalid),
        .s_axis_tdata        (s_axis_c2h_tdata),
        .s_axis_tcrc         (s_axis_c2h_tcrc),
        .s_axis_tlast        (s_axis_c2h_tlast),
        .s_axis_ctrl_marker  (s_axis_c2h_ctrl_marker),
        .s_axis_ctrl_port_id (s_axis_c2h_ctrl_port_id),
        .s_axis_ctrl_ecc     (s_axis_c2h_ctrl_ecc),
        .s_axis_ctrl_len     (s_axis_c2h_ctrl_len),
        .s_axis_ctrl_qid     (s_axis_c2h_ctrl_qid),
        .s_axis_ctrl_has_cmpt(s_axis_c2h_ctrl_has_cmpt),
        .s_axis_mty          (s_axis_c2h_mty),
        .s_axis_tready       (s_axis_c2h_tready),

        .m_axis_tvalid       (c2h_256_tvalid),
        .m_axis_tdata        (c2h_256_tdata),
        .m_axis_tcrc         (c2h_256_tcrc),
        .m_axis_tlast        (c2h_256_tlast),
        .m_axis_ctrl_marker  (c2h_256_ctrl_marker),
        .m_axis_ctrl_port_id (c2h_256_ctrl_port_id),
        .m_axis_ctrl_ecc     (c2h_256_ctrl_ecc),
        .m_axis_ctrl_len     (c2h_256_ctrl_len),
        .m_axis_ctrl_qid     (c2h_256_ctrl_qid),
        .m_axis_ctrl_has_cmpt(c2h_256_ctrl_has_cmpt),
        .m_axis_mty          (c2h_256_mty),
        .m_axis_tready       (c2h_256_tready)
    );

    assign sgdma_st_rx_tvalid   = c2h_256_tvalid;
    assign sgdma_st_rx_tdata    = c2h_256_tdata;
    assign sgdma_st_rx_tlast    = c2h_256_tlast;
    assign sgdma_st_rx_qid_wide = c2h_256_ctrl_qid;
    assign sgdma_st_rx_len16    = c2h_256_ctrl_len;
    assign sgdma_st_rx_mty      = c2h_256_mty;
    assign c2h_256_tready       = sgdma_st_rx_tready;

    // sgdma owns ST completion writeback; the old sgdma C2H completion stream is
    // consumed to avoid backpressure but is not forwarded anywhere.
    assign s_axis_cpl_tready    = 1'b1;

    // Network TX / H2C: sgdma ST H2D/TX 256-bit stream -> legacy 512-bit stream.
    axis_dwconv_h2c_256_to_512 u_h2c_dwconv (
        .aclk   (aclk_250mhz),
        .aresetn(datapath_aresetn_250mhz),

        .s_axis_tvalid         (sgdma_st_tx_tvalid),
        .s_axis_tdata          (sgdma_st_tx_tdata),
        .s_axis_tcrc           (32'd0),
        .s_axis_tlast          (sgdma_st_tx_tlast),
        .s_axis_tuser_qid      ({2'd0, sgdma_st_tx_tqid}),
        .s_axis_tuser_port_id  (3'd0),
        .s_axis_tuser_err      (1'b0),
        .s_axis_tuser_mdata    ({sgdma_st_tx_tlen[15:0], 16'd0}),
        .s_axis_tuser_mty      (sgdma_st_tx_mty),
        .s_axis_tuser_zero_byte(1'b0),
        .s_axis_tready         (sgdma_st_tx_tready),

        .m_axis_tvalid         (m_axis_h2c_tvalid),
        .m_axis_tdata          (m_axis_h2c_tdata),
        .m_axis_tcrc           (m_axis_h2c_tcrc),
        .m_axis_tlast          (m_axis_h2c_tlast),
        .m_axis_tuser_qid      (m_axis_h2c_tuser_qid),
        .m_axis_tuser_port_id  (m_axis_h2c_tuser_port_id),
        .m_axis_tuser_err      (m_axis_h2c_tuser_err),
        .m_axis_tuser_mdata    (m_axis_h2c_tuser_mdata),
        .m_axis_tuser_mty      (m_axis_h2c_tuser_mty),
        .m_axis_tuser_zero_byte(m_axis_h2c_tuser_zero_byte),
        .m_axis_tready         (m_axis_h2c_tready)
    );

    /* ---------------- Payload AXI width conversion: sgdma 256 -> shell 512 ---------------- */

    wire         axi_256_awready;
    wire         axi_256_wready;
    wire [  3:0] axi_256_bid;
    wire [  1:0] axi_256_bresp;
    wire         axi_256_bvalid;
    wire         axi_256_arready;
    wire [  3:0] axi_256_rid;
    wire [255:0] axi_256_rdata;
    wire [  1:0] axi_256_rresp;
    wire         axi_256_rlast;
    wire         axi_256_rvalid;
    wire [  3:0] axi_256_awid;
    wire [ 63:0] axi_256_awaddr;
    wire [  7:0] axi_256_awlen;
    wire [  2:0] axi_256_awsize;
    wire [  1:0] axi_256_awburst;
    wire [  2:0] axi_256_awprot;
    wire         axi_256_awvalid;
    wire         axi_256_awlock;
    wire [  3:0] axi_256_awcache;
    wire [255:0] axi_256_wdata;
    wire [ 31:0] axi_256_wstrb;
    wire         axi_256_wlast;
    wire         axi_256_wvalid;
    wire         axi_256_bready;
    wire [  3:0] axi_256_arid;
    wire [ 63:0] axi_256_araddr;
    wire [  7:0] axi_256_arlen;
    wire [  2:0] axi_256_arsize;
    wire [  1:0] axi_256_arburst;
    wire [  2:0] axi_256_arprot;
    wire         axi_256_arvalid;
    wire         axi_256_arlock;
    wire [  3:0] axi_256_arcache;
    wire         axi_256_rready;

    wire         s_axi_256_awready;
    wire         s_axi_256_wready;
    wire [  3:0] s_axi_256_bid;
    wire [  1:0] s_axi_256_bresp;
    wire         s_axi_256_bvalid;
    wire         s_axi_256_arready;
    wire [  3:0] s_axi_256_rid;
    wire [255:0] s_axi_256_rdata;
    wire [  1:0] s_axi_256_rresp;
    wire         s_axi_256_rlast;
    wire         s_axi_256_rvalid;
    wire [  3:0] s_axi_256_awid;
    wire [ 63:0] s_axi_256_awaddr;
    wire [  7:0] s_axi_256_awlen;
    wire [  2:0] s_axi_256_awsize;
    wire [  1:0] s_axi_256_awburst;
    wire [  2:0] s_axi_256_awprot;
    wire         s_axi_256_awvalid;
    wire         s_axi_256_awlock;
    wire [  3:0] s_axi_256_awcache;
    wire [255:0] s_axi_256_wdata;
    wire [ 31:0] s_axi_256_wstrb;
    wire         s_axi_256_wlast;
    wire         s_axi_256_wvalid;
    wire         s_axi_256_bready;
    wire [  3:0] s_axi_256_arid;
    wire [ 63:0] s_axi_256_araddr;
    wire [  7:0] s_axi_256_arlen;
    wire [  2:0] s_axi_256_arsize;
    wire [  1:0] s_axi_256_arburst;
    wire [  2:0] s_axi_256_arprot;
    wire         s_axi_256_arvalid;
    wire         s_axi_256_arlock;
    wire [  3:0] s_axi_256_arcache;
    wire         s_axi_256_rready;


    axi_width_conv_256_to_512 #(
        .OUTSTANDING(32),
        .RDATA_FIFO_DEPTH(512)
    ) axi_converter_256to512 (
        .aclk   (aclk_250mhz),
        .aresetn(aresetn_250mhz),

        .s_awaddr  ({32'hA3500000, axi_256_awaddr[31:0]}),
        .s_awlen   (axi_256_awlen),
        .s_awsize  (axi_256_awsize),
        .s_awburst (axi_256_awburst),
        .s_awlock  (axi_256_awlock),
        .s_awcache (axi_256_awcache),
        .s_awprot  (axi_256_awprot),
        .s_awregion(4'd0),
        .s_awqos   (4'd0),
        .s_awid    (axi_256_awid),
        .s_awvalid (axi_256_awvalid),
        .s_awready (axi_256_awready),
        .s_wdata   (axi_256_wdata),
        .s_wstrb   (axi_256_wstrb),
        .s_wlast   (axi_256_wlast),
        .s_wvalid  (axi_256_wvalid),
        .s_wready  (axi_256_wready),
        .s_bresp   (axi_256_bresp),
        .s_bvalid  (axi_256_bvalid),
        .s_bid     (axi_256_bid),
        .s_bready  (axi_256_bready),
        .s_araddr  ({32'hA3500000, axi_256_araddr[31:0]}),
        .s_arlen   (axi_256_arlen),
        .s_arsize  (axi_256_arsize),
        .s_arburst (axi_256_arburst),
        .s_arlock  (axi_256_arlock),
        .s_arcache (axi_256_arcache),
        .s_arprot  (axi_256_arprot),
        .s_arregion(4'd0),
        .s_arqos   (4'd0),
        .s_arid    (axi_256_arid),
        .s_arvalid (axi_256_arvalid),
        .s_arready (axi_256_arready),
        .s_rdata   (axi_256_rdata),
        .s_rresp   (axi_256_rresp),
        .s_rlast   (axi_256_rlast),
        .s_rvalid  (axi_256_rvalid),
        .s_rready  (axi_256_rready),
        .s_rid     (axi_256_rid),

        .m_awaddr  (m_axi_awaddr),
        .m_awid    (m_axi_awid),
        .m_awlen   (m_axi_awlen),
        .m_awsize  (m_axi_awsize),
        .m_awburst (m_axi_awburst),
        .m_awlock  (m_axi_awlock),
        .m_awcache (m_axi_awcache),
        .m_awprot  (m_axi_awprot),
        .m_awregion(),
        .m_awqos   (),
        .m_awvalid (m_axi_awvalid),
        .m_awready (m_axi_awready),
        .m_wdata   (m_axi_wdata),
        .m_wstrb   (m_axi_wstrb),
        .m_wlast   (m_axi_wlast),
        .m_wvalid  (m_axi_wvalid),
        .m_wready  (m_axi_wready),
        .m_bresp   (m_axi_bresp),
        .m_bid     (m_axi_bid),
        .m_bvalid  (m_axi_bvalid),
        .m_bready  (m_axi_bready),
        .m_araddr  (m_axi_araddr),
        .m_arid    (m_axi_arid),
        .m_arlen   (m_axi_arlen),
        .m_arsize  (m_axi_arsize),
        .m_arburst (m_axi_arburst),
        .m_arlock  (m_axi_arlock),
        .m_arcache (m_axi_arcache),
        .m_arprot  (m_axi_arprot),
        .m_arregion(),
        .m_arqos   (),
        .m_arvalid (m_axi_arvalid),
        .m_arready (m_axi_arready),
        .m_rdata   (m_axi_rdata),
        .m_rresp   (m_axi_rresp),
        .m_rid     (m_axi_rid),
        .m_rlast   (m_axi_rlast),
        .m_rvalid  (m_axi_rvalid),
        .m_rready  (m_axi_rready)
    );

    axi4_dw_512_to_256 axi_converter_512to256 (
        .aclk   (aclk_250mhz),
        .aresetn(aresetn_250mhz),

        .s_axi_awaddr (s_axib_awaddr),
        .s_axi_awlen  (s_axib_awlen),
        .s_axi_awsize (s_axib_awsize),
        .s_axi_awburst(s_axib_awburst),
        .s_axi_awid   (s_axib_awid),
        .s_axi_awvalid(s_axib_awvalid),
        .s_axi_awready(s_axib_awready),
        .s_axi_wdata  (s_axib_wdata),
        .s_axi_wstrb  (s_axib_wstrb),
        .s_axi_wlast  (s_axib_wlast),
        .s_axi_wvalid (s_axib_wvalid),
        .s_axi_wready (s_axib_wready),
        .s_axi_bresp  (s_axib_bresp),
        .s_axi_bvalid (s_axib_bvalid),
        .s_axi_bid    (s_axib_bid),
        .s_axi_bready (s_axib_bready),
        .s_axi_araddr (s_axib_araddr),
        .s_axi_arlen  (s_axib_arlen),
        .s_axi_arsize (s_axib_arsize),
        .s_axi_arburst(s_axib_arburst),
        .s_axi_arid   (s_axib_arid),
        .s_axi_arvalid(s_axib_arvalid),
        .s_axi_arready(s_axib_arready),
        .s_axi_rdata  (s_axib_rdata),
        .s_axi_rresp  (s_axib_rresp),
        .s_axi_rlast  (s_axib_rlast),
        .s_axi_rvalid (s_axib_rvalid),
        .s_axi_rready (s_axib_rready),
        .s_axi_rid    (s_axib_rid),

        .m_axi_awaddr (s_axi_256_awaddr),
        .m_axi_awid   (s_axi_256_awid),
        .m_axi_awlen  (s_axi_256_awlen),
        .m_axi_awsize (s_axi_256_awsize),
        .m_axi_awburst(s_axi_256_awburst),
        .m_axi_awvalid(s_axi_256_awvalid),
        .m_axi_awready(s_axi_256_awready),
        .m_axi_wdata  (s_axi_256_wdata),
        .m_axi_wstrb  (s_axi_256_wstrb),
        .m_axi_wlast  (s_axi_256_wlast),
        .m_axi_wvalid (s_axi_256_wvalid),
        .m_axi_wready (s_axi_256_wready),
        .m_axi_bresp  (s_axi_256_bresp),
        .m_axi_bid    (s_axi_256_bid),
        .m_axi_bvalid (s_axi_256_bvalid),
        .m_axi_bready (s_axi_256_bready),
        .m_axi_araddr (s_axi_256_araddr),
        .m_axi_arid   (s_axi_256_arid),
        .m_axi_arlen  (s_axi_256_arlen),
        .m_axi_arsize (s_axi_256_arsize),
        .m_axi_arburst(s_axi_256_arburst),
        .m_axi_arvalid(s_axi_256_arvalid),
        .m_axi_arready(s_axi_256_arready),
        .m_axi_rdata  (s_axi_256_rdata),
        .m_axi_rresp  (s_axi_256_rresp),
        .m_axi_rid    (s_axi_256_rid),
        .m_axi_rlast  (s_axi_256_rlast),
        .m_axi_rvalid (s_axi_256_rvalid),
        .m_axi_rready (s_axi_256_rready)
    );


    /* ---------------- BAR2 AXI-Lite CDC: sgdma user_clk -> shell axil_aclk ---------------- */

    wire        sgdma_axil_awvalid;
    wire [31:0] sgdma_axil_awaddr;
    wire [ 2:0] sgdma_axil_awprot;
    wire        sgdma_axil_awready;
    wire        sgdma_axil_wvalid;
    wire [31:0] sgdma_axil_wdata;
    wire [ 3:0] sgdma_axil_wstrb;
    wire        sgdma_axil_wready;
    wire        sgdma_axil_bvalid;
    wire [ 1:0] sgdma_axil_bresp;
    wire        sgdma_axil_bready;
    wire        sgdma_axil_arvalid;
    wire [31:0] sgdma_axil_araddr;
    wire [ 2:0] sgdma_axil_arprot;
    wire        sgdma_axil_arready;
    wire        sgdma_axil_rvalid;
    wire [31:0] sgdma_axil_rdata;
    wire [ 1:0] sgdma_axil_rresp;
    wire        sgdma_axil_rready;

    sgdma_subsystem_axi_cdc axi_cdc_inst (
        .s_axi_awvalid(sgdma_axil_awvalid),
        .s_axi_awaddr (sgdma_axil_awaddr),
        .s_axi_awprot (sgdma_axil_awprot),
        .s_axi_awready(sgdma_axil_awready),
        .s_axi_wvalid (sgdma_axil_wvalid),
        .s_axi_wdata  (sgdma_axil_wdata),
        .s_axi_wstrb  (sgdma_axil_wstrb),
        .s_axi_wready (sgdma_axil_wready),
        .s_axi_bvalid (sgdma_axil_bvalid),
        .s_axi_bresp  (sgdma_axil_bresp),
        .s_axi_bready (sgdma_axil_bready),
        .s_axi_arvalid(sgdma_axil_arvalid),
        .s_axi_araddr (sgdma_axil_araddr),
        .s_axi_arprot (sgdma_axil_arprot),
        .s_axi_arready(sgdma_axil_arready),
        .s_axi_rvalid (sgdma_axil_rvalid),
        .s_axi_rdata  (sgdma_axil_rdata),
        .s_axi_rresp  (sgdma_axil_rresp),
        .s_axi_rready (sgdma_axil_rready),

        .m_axi_awvalid(m_axil_awvalid),
        .m_axi_awaddr (m_axil_awaddr),
        .m_axi_awprot (),
        .m_axi_awready(m_axil_awready),
        .m_axi_wvalid (m_axil_wvalid),
        .m_axi_wdata  (m_axil_wdata),
        .m_axi_wstrb  (),
        .m_axi_wready (m_axil_wready),
        .m_axi_bvalid (m_axil_bvalid),
        .m_axi_bresp  (m_axil_bresp),
        .m_axi_bready (m_axil_bready),
        .m_axi_arvalid(m_axil_arvalid),
        .m_axi_araddr (m_axil_araddr),
        .m_axi_arprot (),
        .m_axi_arready(m_axil_arready),
        .m_axi_rvalid (m_axil_rvalid),
        .m_axi_rdata  (m_axil_rdata),
        .m_axi_rresp  (m_axil_rresp),
        .m_axi_rready (m_axil_rready),

        .s_axi_aclk   (axis_aclk),
        .s_axi_aresetn(aresetn_250mhz),
        .m_axi_aclk   (axil_aclk),
        .m_axi_aresetn(aresetn)
    );

    /* ---------------- SGDMA core instance ---------------- */


    sgdma_core #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
        .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
        .AXIS_PCIE_CQ_USER_WIDTH(AXIS_PCIE_CQ_USER_WIDTH),
        .AXIS_PCIE_CC_USER_WIDTH(AXIS_PCIE_CC_USER_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
        .PCIE_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXIL_DATA_WIDTH(AXIL_DATA_WIDTH),
        .AXIL_ADDR_WIDTH(AXIL_ADDR_WIDTH),
        .AXIL_STRB_WIDTH(AXIL_STRB_WIDTH),
        .ST_AXIS_USER_WIDTH(ST_AXIS_USER_WIDTH),
        .DMA_QID_WIDTH(DMA_QID_WIDTH),
        .DMA_LEN_WIDTH(DMA_LEN_WIDTH)

    ) sgdma_inst (
        .clk                                  (aclk_250mhz),
        // sgdma owns the PCIe CQ/CC completer for host BAR accesses; do not
        // reset it with shell soft_reset_n, otherwise config/probe MMIO reads
        // can lose completions and trigger host CmpltTO.
        .rst                                  (~aresetn_250mhz),
        .camera_frame_irq                     (camera_frame_irq),
        .s_axis_st_rx_tdata                   (sgdma_st_rx_tdata),
        .s_axis_st_rx_mty                     (sgdma_st_rx_mty),
        .s_axis_st_rx_tvalid                  (sgdma_st_rx_tvalid),
        .s_axis_st_rx_tready                  (sgdma_st_rx_tready),
        .s_axis_st_rx_tlast                   (sgdma_st_rx_tlast),
        .s_axis_st_rx_tuser                   (32'd0),
        .s_axis_st_rx_tqid                    (sgdma_st_rx_qid_wide[DMA_QID_WIDTH-1:0]),
        .s_axis_st_rx_tlen                    ({8'd0, sgdma_st_rx_len16}),
        .m_axis_st_tx_tdata                   (sgdma_st_tx_tdata),
        .m_axis_st_tx_mty                     (sgdma_st_tx_mty),
        .m_axis_st_tx_tvalid                  (sgdma_st_tx_tvalid),
        .m_axis_st_tx_tready                  (sgdma_st_tx_tready),
        .m_axis_st_tx_tlast                   (sgdma_st_tx_tlast),
        .m_axis_st_tx_tuser                   (sgdma_st_tx_tuser),
        .m_axis_st_tx_tqid                    (sgdma_st_tx_tqid),
        .m_axis_st_tx_tlen                    (sgdma_st_tx_tlen),
        .m_axis_rq_tdata                      (axis_rq_tdata),
        .m_axis_rq_tkeep                      (axis_rq_tkeep),
        .m_axis_rq_tlast                      (axis_rq_tlast),
        .m_axis_rq_tready                     (axis_rq_tready),
        .m_axis_rq_tuser                      (axis_rq_tuser),
        .m_axis_rq_tvalid                     (axis_rq_tvalid),
        .s_axis_rq_seq_num_0                  (axis_rq_seq_num0),
        .s_axis_rq_seq_num_valid_0            (axis_rq_seq_num_vld0),
        .s_axis_rq_seq_num_1                  (axis_rq_seq_num1),
        .s_axis_rq_seq_num_valid_1            (axis_rq_seq_num_vld1),
        .s_axis_rc_tdata                      (axis_rc_tdata),
        .s_axis_rc_tkeep                      (axis_rc_tkeep),
        .s_axis_rc_tlast                      (axis_rc_tlast),
        .s_axis_rc_tready                     (axis_rc_tready),
        .s_axis_rc_tuser                      (axis_rc_tuser),
        .s_axis_rc_tvalid                     (axis_rc_tvalid),
        .s_axis_cq_tdata                      (axis_cq_tdata),
        .s_axis_cq_tkeep                      (axis_cq_tkeep),
        .s_axis_cq_tlast                      (axis_cq_tlast),
        .s_axis_cq_tready                     (axis_cq_tready),
        .s_axis_cq_tuser                      (axis_cq_tuser),
        .s_axis_cq_tvalid                     (axis_cq_tvalid),
        .m_axis_cc_tdata                      (axis_cc_tdata),
        .m_axis_cc_tkeep                      (axis_cc_tkeep),
        .m_axis_cc_tlast                      (axis_cc_tlast),
        .m_axis_cc_tready                     (axis_cc_tready),
        .m_axis_cc_tuser                      (axis_cc_tuser),
        .m_axis_cc_tvalid                     (axis_cc_tvalid),
        .cfg_max_payload                      (cfg_max_payload),
        .cfg_max_read_req                     (cfg_max_read_req),
        .pcie_tx_fc_nph_av                    (cfg_fc_nph),
        .pcie_tx_fc_ph_av                     (cfg_fc_ph),
        .pcie_tx_fc_pd_av                     (cfg_fc_pd),
        .cfg_mgmt_addr                        (cfg_mgmt_addr),
        .cfg_mgmt_function_number             (cfg_mgmt_function_number),
        .cfg_mgmt_write                       (cfg_mgmt_write),
        .cfg_mgmt_write_data                  (cfg_mgmt_write_data),
        .cfg_mgmt_byte_enable                 (cfg_mgmt_byte_enable),
        .cfg_mgmt_read                        (cfg_mgmt_read),
        .cfg_mgmt_read_data                   (cfg_mgmt_read_data),
        .cfg_mgmt_read_write_done             (cfg_mgmt_read_write_done),
        .cfg_interrupt_msix_enable            (cfg_interrupt_msix_enable),
        .cfg_interrupt_msix_mask              (cfg_interrupt_msix_mask),
        .cfg_interrupt_msix_address           (cfg_interrupt_msix_address),
        .cfg_interrupt_msix_data              (cfg_interrupt_msix_data),
        .cfg_interrupt_msix_int               (cfg_interrupt_msix_int),
        .cfg_interrupt_msix_vec_pending       (cfg_interrupt_msix_vec_pending),
        .cfg_interrupt_msix_vec_pending_status(cfg_interrupt_msix_vec_pending_status),
        .cfg_interrupt_msi_sent               (cfg_interrupt_msi_sent),
        .cfg_interrupt_msi_fail               (cfg_interrupt_msi_fail),
        .cfg_interrupt_msi_function_number    (cfg_interrupt_msi_function_number),
        .status_error_cor                     (status_error_cor),
        .status_error_uncor                   (status_error_uncor),
        .m_axi_awid                           (axi_256_awid),
        .m_axi_awaddr                         (axi_256_awaddr),
        .m_axi_awlen                          (axi_256_awlen),
        .m_axi_awsize                         (axi_256_awsize),
        .m_axi_awburst                        (axi_256_awburst),
        .m_axi_awlock                         (axi_256_awlock),
        .m_axi_awcache                        (axi_256_awcache),
        .m_axi_awprot                         (axi_256_awprot),
        .m_axi_awvalid                        (axi_256_awvalid),
        .m_axi_awready                        (axi_256_awready),
        .m_axi_wdata                          (axi_256_wdata),
        .m_axi_wstrb                          (axi_256_wstrb),
        .m_axi_wlast                          (axi_256_wlast),
        .m_axi_wvalid                         (axi_256_wvalid),
        .m_axi_wready                         (axi_256_wready),
        .m_axi_bid                            (axi_256_bid),
        .m_axi_bresp                          (axi_256_bresp),
        .m_axi_bvalid                         (axi_256_bvalid),
        .m_axi_bready                         (axi_256_bready),
        .m_axi_arid                           (axi_256_arid),
        .m_axi_araddr                         (axi_256_araddr),
        .m_axi_arlen                          (axi_256_arlen),
        .m_axi_arsize                         (axi_256_arsize),
        .m_axi_arburst                        (axi_256_arburst),
        .m_axi_arlock                         (axi_256_arlock),
        .m_axi_arcache                        (axi_256_arcache),
        .m_axi_arprot                         (axi_256_arprot),
        .m_axi_arvalid                        (axi_256_arvalid),
        .m_axi_arready                        (axi_256_arready),
        .m_axi_rid                            (axi_256_rid),
        .m_axi_rdata                          (axi_256_rdata),
        .m_axi_rresp                          (axi_256_rresp),
        .m_axi_rlast                          (axi_256_rlast),
        .m_axi_rvalid                         (axi_256_rvalid),
        .m_axi_rready                         (axi_256_rready),
        .m_axil_ext_awaddr                    (sgdma_axil_awaddr),
        .m_axil_ext_awprot                    (sgdma_axil_awprot),
        .m_axil_ext_awvalid                   (sgdma_axil_awvalid),
        .m_axil_ext_awready                   (sgdma_axil_awready),
        .m_axil_ext_wdata                     (sgdma_axil_wdata),
        .m_axil_ext_wstrb                     (sgdma_axil_wstrb),
        .m_axil_ext_wvalid                    (sgdma_axil_wvalid),
        .m_axil_ext_wready                    (sgdma_axil_wready),
        .m_axil_ext_bresp                     (sgdma_axil_bresp),
        .m_axil_ext_bvalid                    (sgdma_axil_bvalid),
        .m_axil_ext_bready                    (sgdma_axil_bready),
        .m_axil_ext_araddr                    (sgdma_axil_araddr),
        .m_axil_ext_arprot                    (sgdma_axil_arprot),
        .m_axil_ext_arvalid                   (sgdma_axil_arvalid),
        .m_axil_ext_arready                   (sgdma_axil_arready),
        .m_axil_ext_rdata                     (sgdma_axil_rdata),
        .m_axil_ext_rresp                     (sgdma_axil_rresp),
        .m_axil_ext_rvalid                    (sgdma_axil_rvalid),
        .m_axil_ext_rready                    (sgdma_axil_rready),
        .s_axi_pcie_awid                      (s_axi_256_awid),
        .s_axi_pcie_awaddr                    (s_axi_256_awaddr),
        .s_axi_pcie_awlen                     (s_axi_256_awlen),
        .s_axi_pcie_awsize                    (s_axi_256_awsize),
        .s_axi_pcie_awburst                   (s_axi_256_awburst),
        .s_axi_pcie_awvalid                   (s_axi_256_awvalid),
        .s_axi_pcie_awready                   (s_axi_256_awready),
        .s_axi_pcie_wdata                     (s_axi_256_wdata),
        .s_axi_pcie_wstrb                     (s_axi_256_wstrb),
        .s_axi_pcie_wlast                     (s_axi_256_wlast),
        .s_axi_pcie_wvalid                    (s_axi_256_wvalid),
        .s_axi_pcie_wready                    (s_axi_256_wready),
        .s_axi_pcie_bid                       (s_axi_256_bid),
        .s_axi_pcie_bresp                     (s_axi_256_bresp),
        .s_axi_pcie_bvalid                    (s_axi_256_bvalid),
        .s_axi_pcie_bready                    (s_axi_256_bready),
        .s_axi_pcie_arid                      (s_axi_256_arid),
        .s_axi_pcie_araddr                    (s_axi_256_araddr),
        .s_axi_pcie_arlen                     (s_axi_256_arlen),
        .s_axi_pcie_arsize                    (s_axi_256_arsize),
        .s_axi_pcie_arburst                   (s_axi_256_arburst),
        .s_axi_pcie_arvalid                   (s_axi_256_arvalid),
        .s_axi_pcie_arready                   (s_axi_256_arready),
        .s_axi_pcie_rid                       (s_axi_256_rid),
        .s_axi_pcie_rdata                     (s_axi_256_rdata),
        .s_axi_pcie_rresp                     (s_axi_256_rresp),
        .s_axi_pcie_rlast                     (s_axi_256_rlast),
        .s_axi_pcie_rvalid                    (s_axi_256_rvalid),
        .s_axi_pcie_rready                    (s_axi_256_rready)
    );

    //     reg        m_axil_r_awvalid;
    //     reg [31:0] m_axil_r_awaddr;
    //     reg        m_axil_r_awready;
    //     reg        m_axil_r_wvalid;
    //     reg [31:0] m_axil_r_wdata;
    //     reg        m_axil_r_wready;
    //     reg        m_axil_r_bvalid;
    //     reg [ 1:0] m_axil_r_bresp;
    //     reg        m_axil_r_bready;
    //     reg        m_axil_r_arvalid;
    //     reg [31:0] m_axil_r_araddr;
    //     reg        m_axil_r_arready;
    //     reg        m_axil_r_rvalid;
    //     reg [31:0] m_axil_r_rdata;
    //     reg [ 1:0] m_axil_r_rresp;
    //     reg        m_axil_r_rready;

    // always @(posedge axil_aclk) begin
    //     if(!aresetn)begin
    //         m_axil_r_awvalid <= 1'b0;
    //         m_axil_r_awaddr  <= 32'd0;
    //         m_axil_r_awready <= 1'b0;
    //         m_axil_r_wvalid  <= 1'b0;
    //         m_axil_r_wdata   <= 32'd0;
    //         m_axil_r_wready  <= 1'b0;
    //         m_axil_r_bvalid  <= 1'b0;
    //         m_axil_r_bresp  <= 2'b00;
    //         m_axil_r_bready <= 1'b0;
    //         m_axil_r_arvalid <= 1'b0;
    //         m_axil_r_araddr <= 32'd0;
    //         m_axil_r_arready <= 1'b0;
    //         m_axil_r_rvalid <= 1'b0;
    //         m_axil_r_rdata <= 32'd0;
    //         m_axil_r_rresp <= 2'b00;
    //         m_axil_r_rready <= 1'b0;
    //     end
    //     m_axil_r_awvalid <= m_axil_awvalid;
    //     m_axil_r_awaddr  <= m_axil_awaddr;
    //     m_axil_r_awready <= m_axil_awready;
    //     m_axil_r_wvalid  <= m_axil_wvalid;
    //     m_axil_r_wdata   <= m_axil_wdata;
    //     m_axil_r_wready  <= m_axil_wready;
    //     m_axil_r_bvalid  <= m_axil_bvalid;
    //     m_axil_r_bresp   <= m_axil_bresp;
    //     m_axil_r_bready  <= m_axil_bready;
    //     m_axil_r_arvalid <= m_axil_arvalid;
    //     m_axil_r_araddr  <= m_axil_araddr;
    //     m_axil_r_arready <= m_axil_arready;
    //     m_axil_r_rvalid  <= m_axil_rvalid;
    //     m_axil_r_rdata   <= m_axil_rdata;
    //     m_axil_r_rresp   <= m_axil_rresp;
    //     m_axil_r_rready  <= m_axil_rready;
    // end

    // ila_5 axil_ila(
    //     .clk(axil_aclk),
    //     .probe0(m_axil_r_awvalid),
    //     .probe1(m_axil_r_awaddr),
    //     .probe2(m_axil_r_awready),
    //     .probe3(m_axil_r_wvalid),
    //     .probe4(m_axil_r_wdata),
    //     .probe5(m_axil_r_wready),
    //     .probe6(m_axil_r_bvalid),
    //     .probe7(m_axil_r_bresp),
    //     .probe8(m_axil_r_bready),
    //     .probe9(m_axil_r_arvalid),
    //     .probe10(m_axil_r_araddr),
    //     .probe11(m_axil_r_arready),
    //     .probe12(m_axil_r_rvalid),
    //     .probe13(m_axil_r_rdata),
    //     .probe14(m_axil_r_rresp),
    //     .probe15(m_axil_r_rready)
    // );



    // AXI and stream observability cores were removed from the production datapath.

endmodule : dma_subsystem_wrapper

module cmlk_idelay_refclk_gen (
    input  wire clk_in,
    output wire clk_out,
    output wire locked
);

    wire        clkfb;
    wire        clkfb_buf;
    wire        clkout0;
    wire        locked_int;
    wire [15:0] do_unused;
    wire        drdy_unused;
    wire        psdone_unused;
    wire        clkinstopped_unused;
    wire        clkfbstopped_unused;
    wire        clkfboutb_unused;
    wire        clkout0b_unused;
    wire        clkout1_unused;
    wire        clkout1b_unused;
    wire        clkout2_unused;
    wire        clkout2b_unused;
    wire        clkout3_unused;
    wire        clkout3b_unused;
    wire        clkout4_unused;
    wire        clkout5_unused;
    wire        clkout6_unused;

    MMCME4_ADV #(
        .BANDWIDTH           ("OPTIMIZED"),
        .CLKOUT4_CASCADE     ("FALSE"),
        .COMPENSATION        ("AUTO"),
        .STARTUP_WAIT        ("FALSE"),
        .DIVCLK_DIVIDE       (5),
        .CLKFBOUT_MULT_F     (24.000),
        .CLKFBOUT_PHASE      (0.000),
        .CLKFBOUT_USE_FINE_PS("FALSE"),
        .CLKOUT0_DIVIDE_F    (3.000),
        .CLKOUT0_PHASE       (0.000),
        .CLKOUT0_DUTY_CYCLE  (0.500),
        .CLKOUT0_USE_FINE_PS ("FALSE"),
        .CLKIN1_PERIOD       (4.000)
    ) mmcme4_adv_inst (
        .CLKFBOUT    (clkfb),
        .CLKFBOUTB   (clkfboutb_unused),
        .CLKOUT0     (clkout0),
        .CLKOUT0B    (clkout0b_unused),
        .CLKOUT1     (clkout1_unused),
        .CLKOUT1B    (clkout1b_unused),
        .CLKOUT2     (clkout2_unused),
        .CLKOUT2B    (clkout2b_unused),
        .CLKOUT3     (clkout3_unused),
        .CLKOUT3B    (clkout3b_unused),
        .CLKOUT4     (clkout4_unused),
        .CLKOUT5     (clkout5_unused),
        .CLKOUT6     (clkout6_unused),
        .CLKFBIN     (clkfb_buf),
        .CLKIN1      (clk_in),
        .CLKIN2      (1'b0),
        .CLKINSEL    (1'b1),
        .DADDR       (7'h0),
        .DCLK        (1'b0),
        .DEN         (1'b0),
        .DI          (16'h0),
        .DO          (do_unused),
        .DRDY        (drdy_unused),
        .DWE         (1'b0),
        .CDDCDONE    (),
        .CDDCREQ     (1'b0),
        .PSCLK       (1'b0),
        .PSEN        (1'b0),
        .PSINCDEC    (1'b0),
        .PSDONE      (psdone_unused),
        .LOCKED      (locked_int),
        .CLKINSTOPPED(clkinstopped_unused),
        .CLKFBSTOPPED(clkfbstopped_unused),
        .PWRDWN      (1'b0),
        .RST         (1'b0)
    );

    BUFG clkfb_buf_inst (
        .I(clkfb),
        .O(clkfb_buf)
    );

    BUFG clkout_buf_inst (
        .I(clkout0),
        .O(clk_out)
    );

    assign locked = locked_int;

endmodule
