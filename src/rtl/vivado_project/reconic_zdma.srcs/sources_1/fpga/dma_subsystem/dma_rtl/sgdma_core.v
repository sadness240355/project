/* SPDX-License-Identifier: MIT
 * Refactored PCIe core wrapper for industrial-camera SGDMA.
 *
 * Replacement for the original test fpga_core.v:
 *   - BAR0 -> direct-CQ SGDMA control/register/ring frontend.
 *   - BAR2 -> external AXI-Lite master for camera/ISP/user registers.
 *   - BAR4 -> direct-CQ MSI-X Table/PBA frontend.
 *   - Payload DMA AXI4 master is exposed to real DDR/image-buffer logic.
 *   - Internal test axi_ram instances are removed.
 */

`resetall `timescale 1ns / 1ps `default_nettype none

`include "edev_sgdma_defs.vh"

module sgdma_core #(
    parameter AXIS_PCIE_DATA_WIDTH = 256,
    parameter AXIS_PCIE_KEEP_WIDTH = (AXIS_PCIE_DATA_WIDTH / 32),
    parameter AXIS_PCIE_RC_USER_WIDTH = 75,
    parameter AXIS_PCIE_RQ_USER_WIDTH = 62,
    parameter AXIS_PCIE_CQ_USER_WIDTH = 88,
    parameter AXIS_PCIE_CC_USER_WIDTH = 33,
    parameter RQ_SEQ_NUM_WIDTH = AXIS_PCIE_RQ_USER_WIDTH == 60 ? 4 : 6,
    parameter PCIE_ADDR_WIDTH = 64,
    parameter AXIL_DATA_WIDTH = 32,
    parameter AXIL_ADDR_WIDTH = 32,
    parameter AXIL_STRB_WIDTH = AXIL_DATA_WIDTH / 8,
    parameter AXI_DATA_WIDTH = AXIS_PCIE_DATA_WIDTH,
    parameter AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8,
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_ID_WIDTH = 8,
    parameter DMA_LEN_WIDTH = 24,
    parameter DMA_TAG_WIDTH = 12,
    parameter DMA_OUTSTANDING_OP_COUNT = 16,
    parameter DMA_DESC_FIFO_DEPTH = DMA_OUTSTANDING_OP_COUNT,
    parameter DMA_DESC_FIFO_ADDR_WIDTH = $clog2(DMA_DESC_FIFO_DEPTH),
    parameter RING_INDEX_WIDTH = 16,
    parameter DMA_QUEUE_COUNT = 512,
    parameter DMA_QID_WIDTH = 9,
    // H2D uses PCIe Memory Read requests and therefore consumes requester
    // completion receive-buffer space in the PCIe hard block.  Keep the
    // requester-tag map within the same 6-bit Extended-Tag profile used by
    // zdma_core.  A wider proven PCIe requester can override this parameter
    // together with the static tag windows below.
    parameter PCIE_REQUESTER_TAG_LIMIT = 64,
    // Payload H2D live MRd tag pool.  The default 32-tag payload window leaves
    // disjoint low-tag windows for ST TX, the AXI bridge, and hostmem traffic.
    parameter DMA_PCIE_TAG_COUNT = 32,
    // Payload H2D live MRd tag limit.  The effective value is clipped to the
    // payload tag window before being passed into pcie_us_axi_dma.
    parameter DMA_READ_REQ_TAG_LIMIT = DMA_PCIE_TAG_COUNT,
    // Payload RC completion FIFO depth in PCIe beats.  This FIFO decouples the
    // PCIe hard-block RC interface from temporary AXI payload-memory stalls, so
    // H2D MRd completions do not block hostmem/BAR/completion traffic under
    // multi-queue burst load.
    parameter DMA_RC_FIFO_DEPTH = 512,
    // Descriptor/writeback RC completion FIFO depth.  Hostmem descriptor fetch
    // and writeback-fence completions are latency-sensitive; buffering this
    // path prevents the hostmem parser from head-of-line blocking payload RC.
    parameter HMEM_RC_FIFO_DEPTH = 128,
    // Final shared requester-request FIFO depth in PCIe beats.  All RQ sources
    // are arbitrated before this FIFO.  Packet mode holds each complete TLP
    // until TLAST has entered the FIFO, so an elastic D2H producer cannot
    // expose an AXI-R data gap as an illegal TVALID gap inside a PCIe MWr TLP.
    parameter DMA_RQ_FIFO_DEPTH = 512,
    // Payload D2H AXI read-data FIFO depth in AXI beats.  This isolates DDR
    // RREADY from PCIe RQ header/credit/backpressure bubbles. The 16 KiB
    // budget admits four 4 KiB read bursts before PCIe consumption, hiding
    // shared-DDR service latency without changing MPS or link clocks.
    parameter DMA_AXI_RDATA_FIFO_DEPTH = 512,
    // Default IRQ profile for Tegra/root ports: collapse all queue completion
    // events onto MSI-X vector 0 and let the driver reclaim all active queues.
    // This avoids per-queue MSI-X table traffic under high DMA rate while
    // preserving multi-queue DMA parallelism.  Set to 0 only with a matching
    // driver loaded with irq_vectors=DMA_QUEUE_COUNT.
    parameter MSIX_SINGLE_VECTOR = 1,
    // Disable host readback fences before MSI-X by default.  The WB MWr is
    // accepted before irq_valid is raised; PCIe posted-write ordering keeps the
    // WB ahead of the MSI-X MWr without issuing extra non-posted fence MRds.
    parameter IRQ_READBACK_FENCE_ENABLE = 0,
    // Scheduler high-performance/stable profile: deeper active context and
    // descriptor FIFOs, block descriptor fetch, and explicit FF pipeline stages.
    parameter REQUESTER_TX_FC_ENABLE = 1,
    parameter DMA_SCHED_OP_FIFO_DEPTH = 32,
    parameter DMA_SCHED_OP_FIFO_ADDR_WIDTH = 5,
    parameter DMA_SCHED_PATH_FIFO_DEPTH = 32,
    parameter DMA_SCHED_PATH_FIFO_ADDR_WIDTH = 5,
    parameter DMA_SCHED_WB_FIFO_DEPTH = 32,
    parameter DMA_SCHED_WB_FIFO_ADDR_WIDTH = 5,
    parameter DMA_SCHED_ST_STAT_FIFO_DEPTH = 16,
    parameter DMA_SCHED_ST_STAT_FIFO_ADDR_WIDTH = 4,
    parameter DMA_DESC_FETCH_MAX_COUNT = 8,
    parameter ST_TX_OUTSTANDING_READS = 2,
    /* High-performance external/client-tag map.
     * Keep this layout identical to zdma_core:
     *
     * Payload MM DMA read tags : 0x00..0x1f
     * ST H2D/TX read tags      : 0x20..0x27
     * AXI bridge read tags     : 0x28..0x2f
     * Hostmem descriptor reads : 0x30..0x3f
     */
    parameter ST_TX_PCIE_TAG = 8'h20,
    parameter ST_TX_PCIE_TAG_MASK = 8'hf8,
    parameter HMEM_PCIE_TAG = 8'h30,
    parameter HMEM_PCIE_TAG_MASK = 8'hf0,
    parameter AXI_BRIDGE_PCIE_TAG = 8'h28,
    parameter AXI_BRIDGE_PCIE_TAG_MASK = 8'hf8,
    /* Cap posted Memory Write TLP payload size.  The PCIe Device Control MPS
     * value is still read from the hard block, but limiting generated posted
     * writes to 256 B avoids fatal MalfTLP on hosts whose upstream path is not
     * safe for 512 B MWr.  Set to 3'd0 for conservative 128 B bring-up, or raise
     * only after the driver verifies the whole path MPS.
     */
    parameter [2:0] POSTED_MPS_LIMIT = 3'd1,
    parameter ST_AXIS_USER_WIDTH = 32,
    parameter ST_AXIS_MTY_WIDTH = 6
) (
    input wire clk,
    input wire rst,
    // Asynchronous level request from the 125 MHz camera AXI-Lite domain.
    // It is synchronized and converted to one event per empty-to-nonempty DDR
    // ring transition below.
    input wire camera_frame_irq,

    /* External streaming DMA ports */
    input  wire [AXIS_PCIE_DATA_WIDTH-1:0] s_axis_st_rx_tdata,
    input  wire [   ST_AXIS_MTY_WIDTH-1:0] s_axis_st_rx_mty,
    input  wire                            s_axis_st_rx_tvalid,
    output wire                            s_axis_st_rx_tready,
    input  wire                            s_axis_st_rx_tlast,
    input  wire [  ST_AXIS_USER_WIDTH-1:0] s_axis_st_rx_tuser,
    input  wire [       DMA_QID_WIDTH-1:0] s_axis_st_rx_tqid,
    input  wire [       DMA_LEN_WIDTH-1:0] s_axis_st_rx_tlen,

    output wire [AXIS_PCIE_DATA_WIDTH-1:0] m_axis_st_tx_tdata,
    output wire [   ST_AXIS_MTY_WIDTH-1:0] m_axis_st_tx_mty,
    output wire                            m_axis_st_tx_tvalid,
    input  wire                            m_axis_st_tx_tready,
    output wire                            m_axis_st_tx_tlast,
    output wire [  ST_AXIS_USER_WIDTH-1:0] m_axis_st_tx_tuser,
    output wire [       DMA_QID_WIDTH-1:0] m_axis_st_tx_tqid,
    output wire [       DMA_LEN_WIDTH-1:0] m_axis_st_tx_tlen,

    /* PCIe AXI-stream interfaces */
    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_rq_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_rq_tkeep,
    output wire                               m_axis_rq_tlast,
    input  wire                               m_axis_rq_tready,
    output wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] m_axis_rq_tuser,
    output wire                               m_axis_rq_tvalid,

    input wire [RQ_SEQ_NUM_WIDTH-1:0] s_axis_rq_seq_num_0,
    input wire                        s_axis_rq_seq_num_valid_0,
    input wire [RQ_SEQ_NUM_WIDTH-1:0] s_axis_rq_seq_num_1,
    input wire                        s_axis_rq_seq_num_valid_1,

    input  wire [   AXIS_PCIE_DATA_WIDTH-1:0] s_axis_rc_tdata,
    input  wire [   AXIS_PCIE_KEEP_WIDTH-1:0] s_axis_rc_tkeep,
    input  wire                               s_axis_rc_tlast,
    output wire                               s_axis_rc_tready,
    input  wire [AXIS_PCIE_RC_USER_WIDTH-1:0] s_axis_rc_tuser,
    input  wire                               s_axis_rc_tvalid,

    input  wire [   AXIS_PCIE_DATA_WIDTH-1:0] s_axis_cq_tdata,
    input  wire [   AXIS_PCIE_KEEP_WIDTH-1:0] s_axis_cq_tkeep,
    input  wire                               s_axis_cq_tlast,
    output wire                               s_axis_cq_tready,
    input  wire [AXIS_PCIE_CQ_USER_WIDTH-1:0] s_axis_cq_tuser,
    input  wire                               s_axis_cq_tvalid,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_cc_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_cc_tkeep,
    output wire                               m_axis_cc_tlast,
    input  wire                               m_axis_cc_tready,
    output wire [AXIS_PCIE_CC_USER_WIDTH-1:0] m_axis_cc_tuser,
    output wire                               m_axis_cc_tvalid,

    input wire [2:0] cfg_max_payload,
    input wire [2:0] cfg_max_read_req,

    /* PCIe hard-block transmit flow-control credits.
     * cfg_fc_sel must select transmit user credits (3'd4) in the shell.
     */
    input wire [ 7:0] pcie_tx_fc_nph_av,
    input wire [ 7:0] pcie_tx_fc_ph_av,
    input wire [11:0] pcie_tx_fc_pd_av,

    output wire [ 9:0] cfg_mgmt_addr,
    output wire [ 7:0] cfg_mgmt_function_number,
    output wire        cfg_mgmt_write,
    output wire [31:0] cfg_mgmt_write_data,
    output wire [ 3:0] cfg_mgmt_byte_enable,
    output wire        cfg_mgmt_read,
    input  wire [31:0] cfg_mgmt_read_data,
    input  wire        cfg_mgmt_read_write_done,

    input  wire [ 3:0] cfg_interrupt_msix_enable,
    input  wire [ 3:0] cfg_interrupt_msix_mask,
    output wire [63:0] cfg_interrupt_msix_address,
    output wire [31:0] cfg_interrupt_msix_data,
    output wire        cfg_interrupt_msix_int,
    output wire [ 1:0] cfg_interrupt_msix_vec_pending,
    input  wire        cfg_interrupt_msix_vec_pending_status,
    input  wire        cfg_interrupt_msi_sent,
    input  wire        cfg_interrupt_msi_fail,
    output wire [ 7:0] cfg_interrupt_msi_function_number,



    output wire                       status_error_cor,
    output wire                       status_error_uncor,
    /* AXI4 slave window translated to PCIe requester memory transactions */
    input  wire [   AXI_ID_WIDTH-1:0] s_axi_pcie_awid,
    input  wire [PCIE_ADDR_WIDTH-1:0] s_axi_pcie_awaddr,
    input  wire [                7:0] s_axi_pcie_awlen,
    input  wire [                2:0] s_axi_pcie_awsize,
    input  wire [                1:0] s_axi_pcie_awburst,
    input  wire                       s_axi_pcie_awvalid,
    output wire                       s_axi_pcie_awready,
    input  wire [ AXI_DATA_WIDTH-1:0] s_axi_pcie_wdata,
    input  wire [ AXI_STRB_WIDTH-1:0] s_axi_pcie_wstrb,
    input  wire                       s_axi_pcie_wlast,
    input  wire                       s_axi_pcie_wvalid,
    output wire                       s_axi_pcie_wready,
    output wire [   AXI_ID_WIDTH-1:0] s_axi_pcie_bid,
    output wire [                1:0] s_axi_pcie_bresp,
    output wire                       s_axi_pcie_bvalid,
    input  wire                       s_axi_pcie_bready,
    input  wire [   AXI_ID_WIDTH-1:0] s_axi_pcie_arid,
    input  wire [PCIE_ADDR_WIDTH-1:0] s_axi_pcie_araddr,
    input  wire [                7:0] s_axi_pcie_arlen,
    input  wire [                2:0] s_axi_pcie_arsize,
    input  wire [                1:0] s_axi_pcie_arburst,
    input  wire                       s_axi_pcie_arvalid,
    output wire                       s_axi_pcie_arready,
    output wire [   AXI_ID_WIDTH-1:0] s_axi_pcie_rid,
    output wire [ AXI_DATA_WIDTH-1:0] s_axi_pcie_rdata,
    output wire [                1:0] s_axi_pcie_rresp,
    output wire                       s_axi_pcie_rlast,
    output wire                       s_axi_pcie_rvalid,
    input  wire                       s_axi_pcie_rready,

    /* External AXI4 master: connect to DDR/image frame memory */
    output wire [  AXI_ID_WIDTH-1:0] m_axi_awid,
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [               7:0] m_axi_awlen,
    output wire [               2:0] m_axi_awsize,
    output wire [               1:0] m_axi_awburst,
    output wire                      m_axi_awlock,
    output wire [               3:0] m_axi_awcache,
    output wire [               2:0] m_axi_awprot,
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,
    output wire [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,
    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,
    input  wire [  AXI_ID_WIDTH-1:0] m_axi_bid,
    input  wire [               1:0] m_axi_bresp,
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,
    output wire [  AXI_ID_WIDTH-1:0] m_axi_arid,
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [               7:0] m_axi_arlen,
    output wire [               2:0] m_axi_arsize,
    output wire [               1:0] m_axi_arburst,
    output wire                      m_axi_arlock,
    output wire [               3:0] m_axi_arcache,
    output wire [               2:0] m_axi_arprot,
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    input  wire [  AXI_ID_WIDTH-1:0] m_axi_rid,
    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [               1:0] m_axi_rresp,
    input  wire                      m_axi_rlast,
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,

    /* External AXI-Lite master behind BAR2: camera/ISP/user registers */
    output wire [AXIL_ADDR_WIDTH-1:0] m_axil_ext_awaddr,
    output wire [                2:0] m_axil_ext_awprot,
    output wire                       m_axil_ext_awvalid,
    input  wire                       m_axil_ext_awready,
    output wire [AXIL_DATA_WIDTH-1:0] m_axil_ext_wdata,
    output wire [AXIL_STRB_WIDTH-1:0] m_axil_ext_wstrb,
    output wire                       m_axil_ext_wvalid,
    input  wire                       m_axil_ext_wready,
    input  wire [                1:0] m_axil_ext_bresp,
    input  wire                       m_axil_ext_bvalid,
    output wire                       m_axil_ext_bready,
    output wire [AXIL_ADDR_WIDTH-1:0] m_axil_ext_araddr,
    output wire [                2:0] m_axil_ext_arprot,
    output wire                       m_axil_ext_arvalid,
    input  wire                       m_axil_ext_arready,
    input  wire [AXIL_DATA_WIDTH-1:0] m_axil_ext_rdata,
    input  wire [                1:0] m_axil_ext_rresp,
    input  wire                       m_axil_ext_rvalid,
    output wire                       m_axil_ext_rready
);

    /* ---------------- CQ BAR demux and CC mux ---------------- */

    wire [AXIS_PCIE_DATA_WIDTH-1:0] axis_cq_tdata_bar_0, axis_cq_tdata_bar_2, axis_cq_tdata_bar_4;
    wire [AXIS_PCIE_KEEP_WIDTH-1:0] axis_cq_tkeep_bar_0, axis_cq_tkeep_bar_2, axis_cq_tkeep_bar_4;
    wire axis_cq_tvalid_bar_0, axis_cq_tready_bar_0, axis_cq_tlast_bar_0;
    wire axis_cq_tvalid_bar_2, axis_cq_tready_bar_2, axis_cq_tlast_bar_2;
    wire axis_cq_tvalid_bar_4, axis_cq_tready_bar_4, axis_cq_tlast_bar_4;
    wire [AXIS_PCIE_CQ_USER_WIDTH-1:0] axis_cq_tuser_bar_0, axis_cq_tuser_bar_2, axis_cq_tuser_bar_4;

    wire [AXIS_PCIE_DATA_WIDTH-1:0] axis_cc_tdata_bar_0, axis_cc_tdata_bar_2, axis_cc_tdata_bar_4;
    wire [AXIS_PCIE_KEEP_WIDTH-1:0] axis_cc_tkeep_bar_0, axis_cc_tkeep_bar_2, axis_cc_tkeep_bar_4;
    wire axis_cc_tvalid_bar_0, axis_cc_tready_bar_0, axis_cc_tlast_bar_0;
    wire axis_cc_tvalid_bar_2, axis_cc_tready_bar_2, axis_cc_tlast_bar_2;
    wire axis_cc_tvalid_bar_4, axis_cc_tready_bar_4, axis_cc_tlast_bar_4;
    wire [AXIS_PCIE_CC_USER_WIDTH-1:0] axis_cc_tuser_bar_0, axis_cc_tuser_bar_2, axis_cc_tuser_bar_4;

    wire [2:0] bar_id;
    wire [2:0] cq_select;
    wire [3:0] desc_resp_error;
    assign cq_select[0] = (bar_id == 3'd0);
    assign cq_select[1] = (bar_id == 3'd2);
    assign cq_select[2] = (bar_id == 3'd4);

    pcie_us_axis_cq_demux #(
        .M_COUNT(3),
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_CQ_USER_WIDTH(AXIS_PCIE_CQ_USER_WIDTH)
    ) cq_demux_inst (
        .clk             (clk),
        .rst             (rst),
        .s_axis_cq_tdata (s_axis_cq_tdata),
        .s_axis_cq_tkeep (s_axis_cq_tkeep),
        .s_axis_cq_tvalid(s_axis_cq_tvalid),
        .s_axis_cq_tready(s_axis_cq_tready),
        .s_axis_cq_tlast (s_axis_cq_tlast),
        .s_axis_cq_tuser (s_axis_cq_tuser),
        .m_axis_cq_tdata ({axis_cq_tdata_bar_4, axis_cq_tdata_bar_2, axis_cq_tdata_bar_0}),
        .m_axis_cq_tkeep ({axis_cq_tkeep_bar_4, axis_cq_tkeep_bar_2, axis_cq_tkeep_bar_0}),
        .m_axis_cq_tvalid({axis_cq_tvalid_bar_4, axis_cq_tvalid_bar_2, axis_cq_tvalid_bar_0}),
        .m_axis_cq_tready({axis_cq_tready_bar_4, axis_cq_tready_bar_2, axis_cq_tready_bar_0}),
        .m_axis_cq_tlast ({axis_cq_tlast_bar_4, axis_cq_tlast_bar_2, axis_cq_tlast_bar_0}),
        .m_axis_cq_tuser ({axis_cq_tuser_bar_4, axis_cq_tuser_bar_2, axis_cq_tuser_bar_0}),
        .req_type        (),
        .target_function (),
        .bar_id          (bar_id),
        .msg_code        (),
        .msg_routing     (),
        .enable          (1'b1),
        .drop            (1'b0),
        .select          (cq_select)
    );

    axis_arb_mux #(
        .S_COUNT(3),
        .DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .ID_ENABLE(0),
        .DEST_ENABLE(0),
        .USER_ENABLE(1),
        .USER_WIDTH(AXIS_PCIE_CC_USER_WIDTH),
        .ARB_TYPE_ROUND_ROBIN(1),
        .ARB_LSB_HIGH_PRIORITY(1)
    ) cc_mux_inst (
        .clk          (clk),
        .rst          (rst),
        .s_axis_tdata ({axis_cc_tdata_bar_4, axis_cc_tdata_bar_2, axis_cc_tdata_bar_0}),
        .s_axis_tkeep ({axis_cc_tkeep_bar_4, axis_cc_tkeep_bar_2, axis_cc_tkeep_bar_0}),
        .s_axis_tvalid({axis_cc_tvalid_bar_4, axis_cc_tvalid_bar_2, axis_cc_tvalid_bar_0}),
        .s_axis_tready({axis_cc_tready_bar_4, axis_cc_tready_bar_2, axis_cc_tready_bar_0}),
        .s_axis_tlast ({axis_cc_tlast_bar_4, axis_cc_tlast_bar_2, axis_cc_tlast_bar_0}),
        .s_axis_tid   (24'd0),
        .s_axis_tdest (24'd0),
        .s_axis_tuser ({axis_cc_tuser_bar_4, axis_cc_tuser_bar_2, axis_cc_tuser_bar_0}),
        .m_axis_tdata (m_axis_cc_tdata),
        .m_axis_tkeep (m_axis_cc_tkeep),
        .m_axis_tvalid(m_axis_cc_tvalid),
        .m_axis_tready(m_axis_cc_tready),
        .m_axis_tlast (m_axis_cc_tlast),
        .m_axis_tid   (),
        .m_axis_tdest (),
        .m_axis_tuser (m_axis_cc_tuser)
    );

    /* ---------------- BAR0/BAR4 direct CQ register blocks, BAR2 external AXI-Lite ---------------- */

    wire status_error_cor_bar0;
    wire status_error_uncor_bar0;
    wire status_error_cor_bar2;
    wire status_error_uncor_bar2;
    wire status_error_cor_bar4;
    wire status_error_uncor_bar4;

    /* BAR0 and BAR4 are internal DMA-engine spaces and are handled directly on CQ/CC.
     * Only BAR2 remains behind the generic CQ-to-AXI-Lite bridge.
     */

    pcie_us_axil_master #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_CQ_USER_WIDTH(AXIS_PCIE_CQ_USER_WIDTH),
        .AXIS_PCIE_CC_USER_WIDTH(AXIS_PCIE_CC_USER_WIDTH),
        .AXI_DATA_WIDTH(AXIL_DATA_WIDTH),
        .AXI_ADDR_WIDTH(AXIL_ADDR_WIDTH),
        .ENABLE_PARITY(0)
    ) bar2_axil_master_inst (
        .clk                (clk),
        .rst                (rst),
        .s_axis_cq_tdata    (axis_cq_tdata_bar_2),
        .s_axis_cq_tkeep    (axis_cq_tkeep_bar_2),
        .s_axis_cq_tvalid   (axis_cq_tvalid_bar_2),
        .s_axis_cq_tready   (axis_cq_tready_bar_2),
        .s_axis_cq_tlast    (axis_cq_tlast_bar_2),
        .s_axis_cq_tuser    (axis_cq_tuser_bar_2),
        .m_axis_cc_tdata    (axis_cc_tdata_bar_2),
        .m_axis_cc_tkeep    (axis_cc_tkeep_bar_2),
        .m_axis_cc_tvalid   (axis_cc_tvalid_bar_2),
        .m_axis_cc_tready   (axis_cc_tready_bar_2),
        .m_axis_cc_tlast    (axis_cc_tlast_bar_2),
        .m_axis_cc_tuser    (axis_cc_tuser_bar_2),
        .m_axil_awaddr      (m_axil_ext_awaddr),
        .m_axil_awprot      (m_axil_ext_awprot),
        .m_axil_awvalid     (m_axil_ext_awvalid),
        .m_axil_awready     (m_axil_ext_awready),
        .m_axil_wdata       (m_axil_ext_wdata),
        .m_axil_wstrb       (m_axil_ext_wstrb),
        .m_axil_wvalid      (m_axil_ext_wvalid),
        .m_axil_wready      (m_axil_ext_wready),
        .m_axil_bresp       (m_axil_ext_bresp),
        .m_axil_bvalid      (m_axil_ext_bvalid),
        .m_axil_bready      (m_axil_ext_bready),
        .m_axil_araddr      (m_axil_ext_araddr),
        .m_axil_arprot      (m_axil_ext_arprot),
        .m_axil_arvalid     (m_axil_ext_arvalid),
        .m_axil_arready     (m_axil_ext_arready),
        .m_axil_rdata       (m_axil_ext_rdata),
        .m_axil_rresp       (m_axil_ext_rresp),
        .m_axil_rvalid      (m_axil_ext_rvalid),
        .m_axil_rready      (m_axil_ext_rready),
        .completer_id       ({8'd0, 5'd0, 3'd0}),
        .completer_id_enable(1'b0),
        .status_error_cor   (status_error_cor_bar2),
        .status_error_uncor (status_error_uncor_bar2)
    );

    /* ---------------- SGDMA software-queued register file and single scheduler ---------------- */

    localparam DMA_ACTIVE_CTX_WIDTH = 11 * 32;
    localparam DMA_CTX_DATA_WIDTH = DMA_QID_WIDTH + DMA_ACTIVE_CTX_WIDTH;
    localparam DMA_CTX_UPDATE_WIDTH = 4 + DMA_QID_WIDTH + RING_INDEX_WIDTH * 2 + 32 * 6;

    wire global_enable, global_reset, global_pause;
    wire [           31:0] global_mode;
    wire [           31:0] global_irq_enable;
    wire [DMA_QID_WIDTH:0] enabled_queue_count;
    wire [            3:0] axi_bridge_table_enable;
    wire [       4*64-1:0] axi_bridge_table_axi_base;
    wire [       4*64-1:0] axi_bridge_table_pcie_base;
    wire [       4*64-1:0] axi_bridge_table_mask;

    wire ctx_valid, ctx_ready;
    wire [  DMA_CTX_DATA_WIDTH-1:0] ctx_data;

    wire                            ctx_update_valid;
    wire [DMA_CTX_UPDATE_WIDTH-1:0] ctx_update_data;
    wire                            sched_busy;
    wire [       DMA_QID_WIDTH-1:0] sched_active_qid;
    wire [31:0] scheduler_irq_status_set;
    wire        scheduler_irq_valid;
    wire [DMA_QID_WIDTH-1:0] scheduler_irq_qid;
    wire [31:0] irq_status_set, irq_status;
    wire                     irq_valid;
    wire [DMA_QID_WIDTH-1:0] irq_qid;
    // Two XPM stages contain metastability and carry their implementation/CDC
    // constraints with the netlist; one local flip-flop performs edge detect.
    wire camera_irq_sync;
    reg  camera_irq_sync_d = 1'b0;

    xpm_cdc_single #(
        .DEST_SYNC_FF  (2),
        .INIT_SYNC_FF  (0),
        .SIM_ASSERT_CHK(0),
        .SRC_INPUT_REG (0)
    ) camera_frame_irq_cdc_inst (
        .src_clk (1'b0),
        .src_in  (camera_frame_irq),
        .dest_clk(clk),
        .dest_out(camera_irq_sync)
    );

    always @(posedge clk) begin
        if (rst)
            camera_irq_sync_d <= 1'b0;
        else
            camera_irq_sync_d <= camera_irq_sync;
    end

    wire camera_irq_rise = camera_irq_sync && !camera_irq_sync_d;
    wire camera_irq_fire = camera_irq_rise && global_enable &&
                           global_mode[`EDEV_MODE_IRQ] &&
                           global_irq_enable[`EDEV_IRQ_CAMERA_FRAME];

    // The default design uses one MSI-X vector, so simultaneous scheduler and
    // camera events share one posted interrupt while preserving both sticky
    // status bits.  In multi-vector builds a simultaneous scheduler event keeps
    // its queue vector; the ISR still reads the same global status register.
    assign irq_status_set = scheduler_irq_status_set |
                            (camera_irq_fire ?
                             (32'b1 << `EDEV_IRQ_CAMERA_FRAME) : 32'b0);
    assign irq_valid = scheduler_irq_valid | camera_irq_fire;
    assign irq_qid = scheduler_irq_valid ? scheduler_irq_qid :
                     {DMA_QID_WIDTH{1'b0}};



    edev_sgdma_regs #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_CQ_USER_WIDTH(AXIS_PCIE_CQ_USER_WIDTH),
        .AXIS_PCIE_CC_USER_WIDTH(AXIS_PCIE_CC_USER_WIDTH),
        .RING_INDEX_WIDTH(RING_INDEX_WIDTH),
        .MAX_QUEUES(DMA_QUEUE_COUNT),
        .QID_WIDTH(DMA_QID_WIDTH),
        .PENDING_FIFO_ADDR_WIDTH(DMA_QID_WIDTH),
        .CAP_VALUE(32'h0051_1000),
        .CTX_DATA_WIDTH(DMA_CTX_DATA_WIDTH),
        .CTX_UPDATE_WIDTH(DMA_CTX_UPDATE_WIDTH)
    ) sgdma_regs_inst (
        .clk                       (clk),
        .rst                       (rst),
        .s_axis_cq_tdata           (axis_cq_tdata_bar_0),
        .s_axis_cq_tkeep           (axis_cq_tkeep_bar_0),
        .s_axis_cq_tvalid          (axis_cq_tvalid_bar_0),
        .s_axis_cq_tready          (axis_cq_tready_bar_0),
        .s_axis_cq_tlast           (axis_cq_tlast_bar_0),
        .s_axis_cq_tuser           (axis_cq_tuser_bar_0),
        .m_axis_cc_tdata           (axis_cc_tdata_bar_0),
        .m_axis_cc_tkeep           (axis_cc_tkeep_bar_0),
        .m_axis_cc_tvalid          (axis_cc_tvalid_bar_0),
        .m_axis_cc_tready          (axis_cc_tready_bar_0),
        .m_axis_cc_tlast           (axis_cc_tlast_bar_0),
        .m_axis_cc_tuser           (axis_cc_tuser_bar_0),
        .completer_id              ({8'd0, 5'd0, 3'd0}),
        .completer_id_enable       (1'b0),
        .global_enable             (global_enable),
        .global_reset              (global_reset),
        .global_pause              (global_pause),
        .global_mode               (global_mode),
        .global_irq_enable         (global_irq_enable),
        .enabled_queue_count       (enabled_queue_count),
        .ctx_valid                 (ctx_valid),
        .ctx_ready                 (ctx_ready),
        .ctx_data                  (ctx_data),
        .ctx_update_valid          (ctx_update_valid),
        .ctx_update_data           (ctx_update_data),
        .sched_busy                (sched_busy),
        .sched_active_qid          (sched_active_qid),
        .irq_status_set            (irq_status_set),
        .irq_status                (irq_status),
        .axi_bridge_table_enable   (axi_bridge_table_enable),
        .axi_bridge_table_axi_base (axi_bridge_table_axi_base),
        .axi_bridge_table_pcie_base(axi_bridge_table_pcie_base),
        .axi_bridge_table_mask     (axi_bridge_table_mask),
        .status_error_cor          (status_error_cor_bar0),
        .status_error_uncor        (status_error_uncor_bar0)
    );

    wire                       sgdma_rst = rst | global_reset;

    wire [PCIE_ADDR_WIDTH-1:0] desc_fetch_addr;
    wire [DMA_TAG_WIDTH-1:0] desc_fetch_tag, desc_resp_tag;
    wire desc_fetch_valid, desc_fetch_ready;
    wire         desc_fetch_is_st;
    wire [  4:0] desc_fetch_count;
    wire [255:0] desc_resp_data;
    wire desc_resp_valid, desc_resp_ready;
    wire [PCIE_ADDR_WIDTH-1:0] cmpl_write_addr;
    wire [              255:0] cmpl_write_data;
    wire cmpl_write_valid, cmpl_write_fence, cmpl_write_ready, cmpl_write_done;


    wire [PCIE_ADDR_WIDTH-1:0] dma_read_desc_pcie_addr;
    wire [ AXI_ADDR_WIDTH-1:0] dma_read_desc_axi_addr;
    wire [  DMA_LEN_WIDTH-1:0] dma_read_desc_len;
    wire [  DMA_TAG_WIDTH-1:0] dma_read_desc_tag;
    wire [  DMA_QID_WIDTH-1:0] dma_read_desc_qid;
    wire dma_read_desc_valid, dma_read_desc_ready;
    wire [  DMA_TAG_WIDTH-1:0] dma_read_status_tag;
    wire [                3:0] dma_read_status_error;
    wire                       dma_read_status_valid;

    wire [PCIE_ADDR_WIDTH-1:0] dma_write_desc_pcie_addr;
    wire [ AXI_ADDR_WIDTH-1:0] dma_write_desc_axi_addr;
    wire [  DMA_LEN_WIDTH-1:0] dma_write_desc_len;
    wire [  DMA_TAG_WIDTH-1:0] dma_write_desc_tag;
    wire [  DMA_QID_WIDTH-1:0] dma_write_desc_qid;
    wire dma_write_desc_valid, dma_write_desc_ready;
    wire [   DMA_TAG_WIDTH-1:0] dma_write_status_tag;
    wire [                 3:0] dma_write_status_error;
    wire                        dma_write_status_valid;

    wire [ PCIE_ADDR_WIDTH-1:0] st_tx_desc_host_addr;
    wire [   DMA_LEN_WIDTH-1:0] st_tx_desc_len;
    wire [   DMA_TAG_WIDTH-1:0] st_tx_desc_tag;
    wire [   DMA_QID_WIDTH-1:0] st_tx_desc_qid;
    wire [RING_INDEX_WIDTH-1:0] st_tx_desc_cidx_after;
    wire [                15:0] st_tx_desc_flags;
    wire [                15:0] st_tx_desc_swtag;
    wire [                31:0] st_tx_desc_user;
    wire st_tx_desc_valid, st_tx_desc_ready;
    wire [   DMA_TAG_WIDTH-1:0] st_tx_status_tag;
    wire [   DMA_QID_WIDTH-1:0] st_tx_status_qid;
    wire [RING_INDEX_WIDTH-1:0] st_tx_status_cidx_after;
    wire [                31:0] st_tx_status_bytes;
    wire [                31:0] st_tx_status_error_code;
    wire [                15:0] st_tx_status_swtag;
    wire [                31:0] st_tx_status_user;
    wire st_tx_status_valid, st_tx_status_ready;

    wire [ PCIE_ADDR_WIDTH-1:0] st_rx_desc_host_addr;
    wire [   DMA_LEN_WIDTH-1:0] st_rx_desc_len;
    wire [   DMA_TAG_WIDTH-1:0] st_rx_desc_tag;
    wire [   DMA_QID_WIDTH-1:0] st_rx_desc_qid;
    wire [RING_INDEX_WIDTH-1:0] st_rx_desc_cidx_after;
    wire [                15:0] st_rx_desc_flags;
    wire [                15:0] st_rx_desc_swtag;
    wire [                31:0] st_rx_desc_user;
    wire [ PCIE_ADDR_WIDTH-1:0] st_rx_desc_wb_base;
    wire [                31:0] st_rx_desc_irq_enable;
    wire st_rx_desc_valid, st_rx_desc_ready;
    wire [   DMA_TAG_WIDTH-1:0] st_rx_status_tag;
    wire [   DMA_QID_WIDTH-1:0] st_rx_status_qid;
    wire [RING_INDEX_WIDTH-1:0] st_rx_status_cidx_after;
    wire [                31:0] st_rx_status_bytes;
    wire [                31:0] st_rx_status_error_code;
    wire [                15:0] st_rx_status_swtag;
    wire [                31:0] st_rx_status_user;
    wire [ PCIE_ADDR_WIDTH-1:0] st_rx_status_wb_base;
    wire [                31:0] st_rx_status_irq_enable;
    wire st_rx_status_valid, st_rx_status_ready;
    edev_sgdma_scheduler #(
        .PCIE_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .LEN_WIDTH(DMA_LEN_WIDTH),
        .TAG_WIDTH(DMA_TAG_WIDTH),
        .RING_INDEX_WIDTH(RING_INDEX_WIDTH),
        .QID_WIDTH(DMA_QID_WIDTH),
        .IRQ_VECTOR_COUNT(DMA_QUEUE_COUNT),
        .DESC_WIDTH(256),
        .CMPL_WIDTH(256),
        .MAX_QUEUES(DMA_QUEUE_COUNT),
        // The Vivado-safe scheduler implementation is internally specialized
        // for 16 active slots / 16-entry path and WB FIFOs.  Pass those values
        // explicitly so top-level parameter overrides cannot accidentally ask
        // Vivado to elaborate a 32-slot wrapper around a 16-slot static bank.
        .OP_FIFO_DEPTH(16),
        .OP_FIFO_ADDR_WIDTH(4),
        .OUTSTANDING_OP_COUNT(DMA_OUTSTANDING_OP_COUNT),
        .DESC_FETCH_MAX_COUNT(DMA_DESC_FETCH_MAX_COUNT),
        .PATH_FIFO_DEPTH(16),
        .PATH_FIFO_ADDR_WIDTH(4),
        .WB_FIFO_DEPTH(16),
        .WB_FIFO_ADDR_WIDTH(4),
        .ST_STAT_FIFO_DEPTH(8),
        .ST_STAT_FIFO_ADDR_WIDTH(3),
        .IRQ_READBACK_FENCE_ENABLE(IRQ_READBACK_FENCE_ENABLE),
        .CTX_DATA_WIDTH(DMA_CTX_DATA_WIDTH),
        .CTX_UPDATE_WIDTH(DMA_CTX_UPDATE_WIDTH)
    ) sgdma_scheduler_inst (
        .clk                     (clk),
        .rst                     (sgdma_rst),
        .global_enable           (global_enable),
        .global_pause            (global_pause),
        .global_mode             (global_mode),
        .max_read_request_size   (cfg_max_read_req),
        .ctx_valid               (ctx_valid),
        .ctx_ready               (ctx_ready),
        .ctx_data                (ctx_data),
        .ctx_update_valid        (ctx_update_valid),
        .ctx_update_data         (ctx_update_data),
        .sched_busy              (sched_busy),
        .sched_active_qid        (sched_active_qid),
        .irq_status_set          (scheduler_irq_status_set),
        .irq_valid               (scheduler_irq_valid),
        .irq_qid                 (scheduler_irq_qid),
        .desc_fetch_addr         (desc_fetch_addr),
        .desc_fetch_tag          (desc_fetch_tag),
        .desc_fetch_count        (desc_fetch_count),
        .desc_fetch_is_st        (desc_fetch_is_st),
        .desc_fetch_valid        (desc_fetch_valid),
        .desc_fetch_ready        (desc_fetch_ready),
        .desc_resp_tag           (desc_resp_tag),
        .desc_resp_data          (desc_resp_data),
        .desc_resp_error         (desc_resp_error),
        .desc_resp_valid         (desc_resp_valid),
        .desc_resp_ready         (desc_resp_ready),
        .cmpl_write_addr         (cmpl_write_addr),
        .cmpl_write_data         (cmpl_write_data),
        .cmpl_write_valid        (cmpl_write_valid),
        .cmpl_write_fence        (cmpl_write_fence),
        .cmpl_write_ready        (cmpl_write_ready),
        .cmpl_write_done         (cmpl_write_done),
        .dma_read_desc_pcie_addr (dma_read_desc_pcie_addr),
        .dma_read_desc_axi_addr  (dma_read_desc_axi_addr),
        .dma_read_desc_len       (dma_read_desc_len),
        .dma_read_desc_tag       (dma_read_desc_tag),
        .dma_read_desc_qid       (dma_read_desc_qid),
        .dma_read_desc_valid     (dma_read_desc_valid),
        .dma_read_desc_ready     (dma_read_desc_ready),
        .dma_read_status_tag     (dma_read_status_tag),
        .dma_read_status_error   (dma_read_status_error),
        .dma_read_status_valid   (dma_read_status_valid),
        .dma_write_desc_pcie_addr(dma_write_desc_pcie_addr),
        .dma_write_desc_axi_addr (dma_write_desc_axi_addr),
        .dma_write_desc_len      (dma_write_desc_len),
        .dma_write_desc_tag      (dma_write_desc_tag),
        .dma_write_desc_qid      (dma_write_desc_qid),
        .dma_write_desc_valid    (dma_write_desc_valid),
        .dma_write_desc_ready    (dma_write_desc_ready),
        .dma_write_status_tag    (dma_write_status_tag),
        .dma_write_status_error  (dma_write_status_error),
        .dma_write_status_valid  (dma_write_status_valid),
        .st_tx_desc_host_addr    (st_tx_desc_host_addr),
        .st_tx_desc_len          (st_tx_desc_len),
        .st_tx_desc_tag          (st_tx_desc_tag),
        .st_tx_desc_qid          (st_tx_desc_qid),
        .st_tx_desc_cidx_after   (st_tx_desc_cidx_after),
        .st_tx_desc_flags        (st_tx_desc_flags),
        .st_tx_desc_swtag        (st_tx_desc_swtag),
        .st_tx_desc_user         (st_tx_desc_user),
        .st_tx_desc_valid        (st_tx_desc_valid),
        .st_tx_desc_ready        (st_tx_desc_ready),
        .st_tx_status_tag        (st_tx_status_tag),
        .st_tx_status_qid        (st_tx_status_qid),
        .st_tx_status_cidx_after (st_tx_status_cidx_after),
        .st_tx_status_bytes      (st_tx_status_bytes),
        .st_tx_status_error_code (st_tx_status_error_code),
        .st_tx_status_swtag      (st_tx_status_swtag),
        .st_tx_status_user       (st_tx_status_user),
        .st_tx_status_valid      (st_tx_status_valid),
        .st_tx_status_ready      (st_tx_status_ready),
        .st_rx_desc_host_addr    (st_rx_desc_host_addr),
        .st_rx_desc_len          (st_rx_desc_len),
        .st_rx_desc_tag          (st_rx_desc_tag),
        .st_rx_desc_qid          (st_rx_desc_qid),
        .st_rx_desc_cidx_after   (st_rx_desc_cidx_after),
        .st_rx_desc_flags        (st_rx_desc_flags),
        .st_rx_desc_swtag        (st_rx_desc_swtag),
        .st_rx_desc_user         (st_rx_desc_user),
        .st_rx_desc_wb_base      (st_rx_desc_wb_base),
        .st_rx_desc_irq_enable   (st_rx_desc_irq_enable),
        .st_rx_desc_valid        (st_rx_desc_valid),
        .st_rx_desc_ready        (st_rx_desc_ready),
        .st_rx_status_tag        (st_rx_status_tag),
        .st_rx_status_qid        (st_rx_status_qid),
        .st_rx_status_cidx_after (st_rx_status_cidx_after),
        .st_rx_status_bytes      (st_rx_status_bytes),
        .st_rx_status_error_code (st_rx_status_error_code),
        .st_rx_status_swtag      (st_rx_status_swtag),
        .st_rx_status_user       (st_rx_status_user),
        .st_rx_status_wb_base    (st_rx_status_wb_base),
        .st_rx_status_irq_enable (st_rx_status_irq_enable),
        .st_rx_status_valid      (st_rx_status_valid),
        .st_rx_status_ready      (st_rx_status_ready)
    );


    wire [PCIE_ADDR_WIDTH-1:0] mm_read_desc_pcie_addr = dma_read_desc_pcie_addr;
    wire [ AXI_ADDR_WIDTH-1:0] mm_read_desc_axi_addr = dma_read_desc_axi_addr;
    wire [  DMA_LEN_WIDTH-1:0] mm_read_desc_len = dma_read_desc_len;
    wire [  DMA_TAG_WIDTH-1:0] mm_read_desc_tag = dma_read_desc_tag;
    wire                       mm_read_desc_valid = dma_read_desc_valid;
    wire                       mm_read_desc_ready;
    assign dma_read_desc_ready = mm_read_desc_ready;

    wire [PCIE_ADDR_WIDTH-1:0] mm_write_desc_pcie_addr = dma_write_desc_pcie_addr;
    wire [ AXI_ADDR_WIDTH-1:0] mm_write_desc_axi_addr = dma_write_desc_axi_addr;
    wire [  DMA_LEN_WIDTH-1:0] mm_write_desc_len = dma_write_desc_len;
    wire [  DMA_TAG_WIDTH-1:0] mm_write_desc_tag = dma_write_desc_tag;
    wire                       mm_write_desc_valid = dma_write_desc_valid;
    wire                       mm_write_desc_ready;
    assign dma_write_desc_ready = mm_write_desc_ready;

    wire                     unused_dma_desc_qid = ^{dma_read_desc_qid, dma_write_desc_qid};

    wire [DMA_TAG_WIDTH-1:0] mm_read_status_tag;
    wire [              3:0] mm_read_status_error;
    wire                     mm_read_status_valid;
    wire [DMA_TAG_WIDTH-1:0] mm_write_status_tag;
    wire [              3:0] mm_write_status_error;
    wire                     mm_write_status_valid;

    localparam integer DMA_PCIE_TAG_COUNT_LIMITED =
        (DMA_PCIE_TAG_COUNT > PCIE_REQUESTER_TAG_LIMIT) ?
        PCIE_REQUESTER_TAG_LIMIT : DMA_PCIE_TAG_COUNT;
    localparam integer DMA_PCIE_TAG_COUNT_EFFECTIVE =
        (DMA_PCIE_TAG_COUNT_LIMITED > ST_TX_PCIE_TAG) ?
        ST_TX_PCIE_TAG : DMA_PCIE_TAG_COUNT_LIMITED;
    localparam integer DMA_READ_REQ_TAG_LIMIT_EFFECTIVE =
        (DMA_READ_REQ_TAG_LIMIT > DMA_PCIE_TAG_COUNT_EFFECTIVE) ?
        DMA_PCIE_TAG_COUNT_EFFECTIVE : DMA_READ_REQ_TAG_LIMIT;
    localparam [7:0] ST_TX_PCIE_TAG_LAST =
        ST_TX_PCIE_TAG | (8'hff ^ ST_TX_PCIE_TAG_MASK);
    localparam [7:0] AXI_BRIDGE_PCIE_TAG_LAST =
        AXI_BRIDGE_PCIE_TAG | (8'hff ^ AXI_BRIDGE_PCIE_TAG_MASK);
    localparam [7:0] HMEM_PCIE_TAG_LAST =
        HMEM_PCIE_TAG | (8'hff ^ HMEM_PCIE_TAG_MASK);

    assign dma_read_status_tag    = mm_read_status_tag;
    assign dma_read_status_error  = mm_read_status_error;
    assign dma_read_status_valid  = mm_read_status_valid;
    assign dma_write_status_tag   = mm_write_status_tag;
    assign dma_write_status_error = mm_write_status_error;
    assign dma_write_status_valid = mm_write_status_valid;
    /* Sanity check for the static client-tag partition.  In client-tag mode the
     * PCIe core does not allocate tags for us, so all requester engines must
     * occupy disjoint tag ranges.
     */
    initial begin
        if (PCIE_REQUESTER_TAG_LIMIT != 32 &&
            PCIE_REQUESTER_TAG_LIMIT != 64 &&
            PCIE_REQUESTER_TAG_LIMIT != 256) begin
            $error("PCIE_REQUESTER_TAG_LIMIT must be 32, 64, or 256");
        end
        if (DMA_PCIE_TAG_COUNT_EFFECTIVE < 1) begin
            $error("Effective payload DMA PCIe tag count must be at least 1");
        end
        if (DMA_READ_REQ_TAG_LIMIT_EFFECTIVE < 1 ||
            DMA_READ_REQ_TAG_LIMIT_EFFECTIVE > DMA_PCIE_TAG_COUNT_EFFECTIVE) begin
            $error("Effective H2D read-request tag limit is outside payload DMA tag range");
        end
        if (ST_TX_OUTSTANDING_READS >
            ((8'hff ^ ST_TX_PCIE_TAG_MASK) + 1)) begin
            $error("ST_TX_OUTSTANDING_READS exceeds ST_TX tag-demux window");
        end
        if (DMA_PCIE_TAG_COUNT_EFFECTIVE > ST_TX_PCIE_TAG) begin
            $error("Payload DMA tag range overlaps ST_TX tag range");
        end
        if (ST_TX_PCIE_TAG_LAST >= PCIE_REQUESTER_TAG_LIMIT ||
            AXI_BRIDGE_PCIE_TAG_LAST >= PCIE_REQUESTER_TAG_LIMIT ||
            HMEM_PCIE_TAG_LAST >= PCIE_REQUESTER_TAG_LIMIT) begin
            $error("one or more reserved PCIe requester tag windows exceed PCIE_REQUESTER_TAG_LIMIT");
        end
        if (((HMEM_PCIE_TAG & HMEM_PCIE_TAG_MASK) <
             DMA_PCIE_TAG_COUNT_EFFECTIVE) ||
            (((HMEM_PCIE_TAG & HMEM_PCIE_TAG_MASK) &
              ST_TX_PCIE_TAG_MASK) ==
             (ST_TX_PCIE_TAG & ST_TX_PCIE_TAG_MASK))) begin
            $error("HMEM tag range overlaps payload DMA or ST_TX tag range");
        end
        if (AXI_BRIDGE_PCIE_TAG < DMA_PCIE_TAG_COUNT_EFFECTIVE) begin
            $error("AXI bridge tag range overlaps payload DMA tag range");
        end
        if ((AXI_BRIDGE_PCIE_TAG & ST_TX_PCIE_TAG_MASK) == (ST_TX_PCIE_TAG & ST_TX_PCIE_TAG_MASK)) begin
            $error("AXI bridge tag range overlaps ST_TX tag range");
        end
        if (((HMEM_PCIE_TAG & HMEM_PCIE_TAG_MASK) & AXI_BRIDGE_PCIE_TAG_MASK) == (AXI_BRIDGE_PCIE_TAG & AXI_BRIDGE_PCIE_TAG_MASK)) begin
            $error("HMEM tag range overlaps AXI bridge tag range");
        end
    end

    /* ---------------- PCIe configuration helper ---------------- */

    wire       ext_tag_enable;
    wire [2:0] cfg_max_payload_posted = (cfg_max_payload > POSTED_MPS_LIMIT) ? POSTED_MPS_LIMIT : cfg_max_payload;

    pcie_us_cfg #(
        .PF_COUNT(1),
        .VF_COUNT(0),
        .VF_OFFSET(4),
        .PCIE_CAP_OFFSET(12'h070)
    ) pcie_us_cfg_inst (
        .clk                     (clk),
        .rst                     (rst),
        .ext_tag_enable          (ext_tag_enable),
        .max_read_request_size   (),
        .max_payload_size        (),
        .cfg_mgmt_addr           (cfg_mgmt_addr),
        .cfg_mgmt_function_number(cfg_mgmt_function_number),
        .cfg_mgmt_write          (cfg_mgmt_write),
        .cfg_mgmt_write_data     (cfg_mgmt_write_data),
        .cfg_mgmt_byte_enable    (cfg_mgmt_byte_enable),
        .cfg_mgmt_read           (cfg_mgmt_read),
        .cfg_mgmt_read_data      (cfg_mgmt_read_data),
        .cfg_mgmt_read_write_done(cfg_mgmt_read_write_done)
    );

    /* ---------------- RC demux: payload DMA vs descriptor fetch ---------------- */

    wire [AXIS_PCIE_DATA_WIDTH-1:0] axis_rc_tdata_r;
    wire [AXIS_PCIE_KEEP_WIDTH-1:0] axis_rc_tkeep_r;
    wire axis_rc_tlast_r, axis_rc_tready_r, axis_rc_tvalid_r;
    wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis_rc_tuser_r;

    axis_register #(
        .DATA_WIDTH (AXIS_PCIE_DATA_WIDTH),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH (AXIS_PCIE_KEEP_WIDTH),
        .LAST_ENABLE(1),
        .ID_ENABLE  (0),
        .DEST_ENABLE(0),
        .USER_ENABLE(1),
        .USER_WIDTH (AXIS_PCIE_RC_USER_WIDTH)
    ) rc_reg_inst (
        .clk          (clk),
        .rst          (sgdma_rst),
        .s_axis_tdata (s_axis_rc_tdata),
        .s_axis_tkeep (s_axis_rc_tkeep),
        .s_axis_tvalid(s_axis_rc_tvalid),
        .s_axis_tready(s_axis_rc_tready),
        .s_axis_tlast (s_axis_rc_tlast),
        .s_axis_tid   (1'b0),
        .s_axis_tdest (1'b0),
        .s_axis_tuser (s_axis_rc_tuser),
        .m_axis_tdata (axis_rc_tdata_r),
        .m_axis_tkeep (axis_rc_tkeep_r),
        .m_axis_tvalid(axis_rc_tvalid_r),
        .m_axis_tready(axis_rc_tready_r),
        .m_axis_tlast (axis_rc_tlast_r),
        .m_axis_tid   (),
        .m_axis_tdest (),
        .m_axis_tuser (axis_rc_tuser_r)
    );

    wire [AXIS_PCIE_DATA_WIDTH-1:0] axis_rc_tdata_dma_demux, axis_rc_tdata_dma;
    wire [AXIS_PCIE_DATA_WIDTH-1:0] axis_rc_tdata_hmem_demux, axis_rc_tdata_hmem;
    wire [AXIS_PCIE_DATA_WIDTH-1:0] axis_rc_tdata_st_tx, axis_rc_tdata_axi_bridge;
    wire [AXIS_PCIE_KEEP_WIDTH-1:0] axis_rc_tkeep_dma_demux, axis_rc_tkeep_dma;
    wire [AXIS_PCIE_KEEP_WIDTH-1:0] axis_rc_tkeep_hmem_demux, axis_rc_tkeep_hmem;
    wire [AXIS_PCIE_KEEP_WIDTH-1:0] axis_rc_tkeep_st_tx, axis_rc_tkeep_axi_bridge;
    wire axis_rc_tvalid_dma_demux, axis_rc_tready_dma_demux, axis_rc_tlast_dma_demux;
    wire axis_rc_tvalid_dma, axis_rc_tready_dma, axis_rc_tlast_dma;
    wire axis_rc_tvalid_hmem_demux, axis_rc_tready_hmem_demux, axis_rc_tlast_hmem_demux;
    wire axis_rc_tvalid_hmem, axis_rc_tready_hmem, axis_rc_tlast_hmem;
    wire axis_rc_tvalid_st_tx, axis_rc_tready_st_tx, axis_rc_tlast_st_tx;
    wire axis_rc_tvalid_axi_bridge, axis_rc_tready_axi_bridge, axis_rc_tlast_axi_bridge;
    wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis_rc_tuser_dma_demux, axis_rc_tuser_dma;
    wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis_rc_tuser_hmem_demux, axis_rc_tuser_hmem;
    wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis_rc_tuser_st_tx, axis_rc_tuser_axi_bridge;
    localparam integer DMA_RC_FIFO_COUNT_WIDTH = $clog2(DMA_RC_FIFO_DEPTH) + 1;
    localparam integer HMEM_RC_FIFO_COUNT_WIDTH = $clog2(HMEM_RC_FIFO_DEPTH) + 1;
    localparam [DMA_RC_FIFO_COUNT_WIDTH-1:0] DMA_RC_FIFO_SOFT_LIMIT = (DMA_RC_FIFO_DEPTH > 520) ? (DMA_RC_FIFO_DEPTH - 512) : (DMA_RC_FIFO_DEPTH - 8);
    localparam [HMEM_RC_FIFO_COUNT_WIDTH-1:0] HMEM_RC_FIFO_SOFT_LIMIT = (HMEM_RC_FIFO_DEPTH > 72) ? (HMEM_RC_FIFO_DEPTH - 64) : (HMEM_RC_FIFO_DEPTH - 8);
    wire dma_rc_fifo_almost_full, dma_rc_fifo_prog_full;
    wire hmem_rc_fifo_almost_full, hmem_rc_fifo_prog_full;
    wire [ DMA_RC_FIFO_COUNT_WIDTH-1:0] dma_rc_fifo_wr_count;
    wire [HMEM_RC_FIFO_COUNT_WIDTH-1:0] hmem_rc_fifo_wr_count;
    wire                                dma_rc_fifo_soft_full = dma_rc_fifo_prog_full || (dma_rc_fifo_wr_count >= DMA_RC_FIFO_SOFT_LIMIT);
    wire                                hmem_rc_fifo_soft_full = hmem_rc_fifo_prog_full || (hmem_rc_fifo_wr_count >= HMEM_RC_FIFO_SOFT_LIMIT);
    wire                                payload_read_request_enable = !dma_rc_fifo_soft_full && !hmem_rc_fifo_soft_full;

    edev_axis_rc_tag_demux #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
        .HMEM_TAG(HMEM_PCIE_TAG),
        .HMEM_TAG_MASK(HMEM_PCIE_TAG_MASK),
        .ST_TX_TAG(ST_TX_PCIE_TAG),
        .ST_TX_TAG_MASK(ST_TX_PCIE_TAG_MASK),
        .AXI_BRIDGE_TAG(AXI_BRIDGE_PCIE_TAG),
        .AXI_BRIDGE_TAG_MASK(AXI_BRIDGE_PCIE_TAG_MASK)
    ) rc_tag_demux_inst (
        .clk                     (clk),
        .rst                     (sgdma_rst),
        .s_axis_tdata            (axis_rc_tdata_r),
        .s_axis_tkeep            (axis_rc_tkeep_r),
        .s_axis_tvalid           (axis_rc_tvalid_r),
        .s_axis_tready           (axis_rc_tready_r),
        .s_axis_tlast            (axis_rc_tlast_r),
        .s_axis_tuser            (axis_rc_tuser_r),
        .m_axis_dma_tdata        (axis_rc_tdata_dma_demux),
        .m_axis_dma_tkeep        (axis_rc_tkeep_dma_demux),
        .m_axis_dma_tvalid       (axis_rc_tvalid_dma_demux),
        .m_axis_dma_tready       (axis_rc_tready_dma_demux),
        .m_axis_dma_tlast        (axis_rc_tlast_dma_demux),
        .m_axis_dma_tuser        (axis_rc_tuser_dma_demux),
        .m_axis_hmem_tdata       (axis_rc_tdata_hmem_demux),
        .m_axis_hmem_tkeep       (axis_rc_tkeep_hmem_demux),
        .m_axis_hmem_tvalid      (axis_rc_tvalid_hmem_demux),
        .m_axis_hmem_tready      (axis_rc_tready_hmem_demux),
        .m_axis_hmem_tlast       (axis_rc_tlast_hmem_demux),
        .m_axis_hmem_tuser       (axis_rc_tuser_hmem_demux),
        .m_axis_st_tx_tdata      (axis_rc_tdata_st_tx),
        .m_axis_st_tx_tkeep      (axis_rc_tkeep_st_tx),
        .m_axis_st_tx_tvalid     (axis_rc_tvalid_st_tx),
        .m_axis_st_tx_tready     (axis_rc_tready_st_tx),
        .m_axis_st_tx_tlast      (axis_rc_tlast_st_tx),
        .m_axis_st_tx_tuser      (axis_rc_tuser_st_tx),
        .m_axis_axi_bridge_tdata (axis_rc_tdata_axi_bridge),
        .m_axis_axi_bridge_tkeep (axis_rc_tkeep_axi_bridge),
        .m_axis_axi_bridge_tvalid(axis_rc_tvalid_axi_bridge),
        .m_axis_axi_bridge_tready(axis_rc_tready_axi_bridge),
        .m_axis_axi_bridge_tlast (axis_rc_tlast_axi_bridge),
        .m_axis_axi_bridge_tuser (axis_rc_tuser_axi_bridge)
    );

    /* Payload RC isolation FIFO.  The payload H2D read engine can temporarily
     * stop accepting CplD beats while the external AXI memory target deasserts
     * WREADY or delays B responses.  Without an elastic buffer here, that
     * backpressure reaches the PCIe hard block RC interface and can starve
     * unrelated completion/BAR traffic until the root port reports CmpltTO.
     */
    axis_xpm_fifo #(
        .DATA_WIDTH      (AXIS_PCIE_DATA_WIDTH),
        .KEEP_ENABLE     (1),
        .KEEP_WIDTH      (AXIS_PCIE_KEEP_WIDTH),
        .LAST_ENABLE     (1),
        .USER_ENABLE     (1),
        .USER_WIDTH      (AXIS_PCIE_RC_USER_WIDTH),
        .FIFO_DEPTH      (DMA_RC_FIFO_DEPTH),
        .PROG_FULL_MARGIN(512),
        .FIFO_MEMORY_TYPE("block")
    ) dma_rc_fifo_inst (
        .clk          (clk),
        .rst          (sgdma_rst),
        .s_axis_tdata (axis_rc_tdata_dma_demux),
        .s_axis_tkeep (axis_rc_tkeep_dma_demux),
        .s_axis_tvalid(axis_rc_tvalid_dma_demux),
        .s_axis_tready(axis_rc_tready_dma_demux),
        .s_axis_tlast (axis_rc_tlast_dma_demux),
        .s_axis_tuser (axis_rc_tuser_dma_demux),
        .m_axis_tdata (axis_rc_tdata_dma),
        .m_axis_tkeep (axis_rc_tkeep_dma),
        .m_axis_tvalid(axis_rc_tvalid_dma),
        .m_axis_tready(axis_rc_tready_dma),
        .m_axis_tlast (axis_rc_tlast_dma),
        .m_axis_tuser (axis_rc_tuser_dma),
        .almost_full  (dma_rc_fifo_almost_full),
        .prog_full    (dma_rc_fifo_prog_full),
        .wr_data_count(dma_rc_fifo_wr_count)
    );

    /* Hostmem RC isolation FIFO.  The hostmem engine parses descriptor-fetch
     * completions and writeback-fence completions with a compact FSM that can
     * deassert ready for several cycles while assembling descriptor words.
     * Buffering this output prevents those short stalls from blocking payload
     * CplD TLPs at the RC demux head.
     */
    axis_xpm_fifo #(
        .DATA_WIDTH      (AXIS_PCIE_DATA_WIDTH),
        .KEEP_ENABLE     (1),
        .KEEP_WIDTH      (AXIS_PCIE_KEEP_WIDTH),
        .LAST_ENABLE     (1),
        .USER_ENABLE     (1),
        .USER_WIDTH      (AXIS_PCIE_RC_USER_WIDTH),
        .FIFO_DEPTH      (HMEM_RC_FIFO_DEPTH),
        .PROG_FULL_MARGIN(64),
        .FIFO_MEMORY_TYPE("block")
    ) hmem_rc_fifo_inst (
        .clk          (clk),
        .rst          (sgdma_rst),
        .s_axis_tdata (axis_rc_tdata_hmem_demux),
        .s_axis_tkeep (axis_rc_tkeep_hmem_demux),
        .s_axis_tvalid(axis_rc_tvalid_hmem_demux),
        .s_axis_tready(axis_rc_tready_hmem_demux),
        .s_axis_tlast (axis_rc_tlast_hmem_demux),
        .s_axis_tuser (axis_rc_tuser_hmem_demux),
        .m_axis_tdata (axis_rc_tdata_hmem),
        .m_axis_tkeep (axis_rc_tkeep_hmem),
        .m_axis_tvalid(axis_rc_tvalid_hmem),
        .m_axis_tready(axis_rc_tready_hmem),
        .m_axis_tlast (axis_rc_tlast_hmem),
        .m_axis_tuser (axis_rc_tuser_hmem),
        .almost_full  (hmem_rc_fifo_almost_full),
        .prog_full    (hmem_rc_fifo_prog_full),
        .wr_data_count(hmem_rc_fifo_wr_count)
    );

    /* ---------------- RQ mux: payload DMA and host-memory micro-engine ---------------- */

    localparam integer RQ_SEQ_IGNORE_BIT = (AXIS_PCIE_RQ_USER_WIDTH == 137) ? 66 : ((AXIS_PCIE_RQ_USER_WIDTH == 62) ? 61 : 27);
    localparam [AXIS_PCIE_RQ_USER_WIDTH-1:0] RQ_SEQ_IGNORE_MASK = ({{(AXIS_PCIE_RQ_USER_WIDTH - 1) {1'b0}}, 1'b1} << RQ_SEQ_IGNORE_BIT);

    wire [AXIS_PCIE_DATA_WIDTH-1:0] axis_rq_tdata_dma, axis_rq_tdata_hmem, axis_rq_tdata_st_rx, axis_rq_tdata_st_tx, axis_rq_tdata_axi_bridge;
    wire [AXIS_PCIE_KEEP_WIDTH-1:0] axis_rq_tkeep_dma, axis_rq_tkeep_hmem, axis_rq_tkeep_st_rx, axis_rq_tkeep_st_tx, axis_rq_tkeep_axi_bridge;
    wire axis_rq_tvalid_dma, axis_rq_tready_dma, axis_rq_tlast_dma;
    wire axis_rq_tvalid_hmem, axis_rq_tready_hmem, axis_rq_tlast_hmem;
    wire axis_rq_tvalid_st_rx, axis_rq_tready_st_rx, axis_rq_tlast_st_rx;
    wire axis_rq_tvalid_st_tx, axis_rq_tready_st_tx, axis_rq_tlast_st_tx;
    wire axis_rq_tvalid_axi_bridge, axis_rq_tready_axi_bridge, axis_rq_tlast_axi_bridge;
    wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis_rq_tuser_dma, axis_rq_tuser_hmem, axis_rq_tuser_st_rx, axis_rq_tuser_st_tx, axis_rq_tuser_axi_bridge;
    wire [AXIS_PCIE_DATA_WIDTH-1:0] axis_rq_tdata_mux;
    wire [AXIS_PCIE_KEEP_WIDTH-1:0] axis_rq_tkeep_mux;
    wire axis_rq_tvalid_mux, axis_rq_tready_mux, axis_rq_tlast_mux;
    wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis_rq_tuser_mux;
    wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis_rq_tuser_hmem_seq = axis_rq_tuser_hmem | RQ_SEQ_IGNORE_MASK;
    wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis_rq_tuser_st_rx_seq = axis_rq_tuser_st_rx | RQ_SEQ_IGNORE_MASK;
    wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis_rq_tuser_st_tx_seq = axis_rq_tuser_st_tx | RQ_SEQ_IGNORE_MASK;
    wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis_rq_tuser_axi_bridge_seq = axis_rq_tuser_axi_bridge | RQ_SEQ_IGNORE_MASK;

    axis_arb_mux #(
        .S_COUNT(5),
        .DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .ID_ENABLE(0),
        .DEST_ENABLE(0),
        .USER_ENABLE(1),
        .USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
        .ARB_TYPE_ROUND_ROBIN(1),
        .ARB_LSB_HIGH_PRIORITY(1)
    ) rq_mux_inst (
        .clk          (clk),
        .rst          (sgdma_rst),
        .s_axis_tdata ({axis_rq_tdata_axi_bridge, axis_rq_tdata_dma, axis_rq_tdata_st_tx, axis_rq_tdata_st_rx, axis_rq_tdata_hmem}),
        .s_axis_tkeep ({axis_rq_tkeep_axi_bridge, axis_rq_tkeep_dma, axis_rq_tkeep_st_tx, axis_rq_tkeep_st_rx, axis_rq_tkeep_hmem}),
        .s_axis_tvalid({axis_rq_tvalid_axi_bridge, axis_rq_tvalid_dma, axis_rq_tvalid_st_tx, axis_rq_tvalid_st_rx, axis_rq_tvalid_hmem}),
        .s_axis_tready({axis_rq_tready_axi_bridge, axis_rq_tready_dma, axis_rq_tready_st_tx, axis_rq_tready_st_rx, axis_rq_tready_hmem}),
        .s_axis_tlast ({axis_rq_tlast_axi_bridge, axis_rq_tlast_dma, axis_rq_tlast_st_tx, axis_rq_tlast_st_rx, axis_rq_tlast_hmem}),
        .s_axis_tid   (40'd0),
        .s_axis_tdest (40'd0),
        .s_axis_tuser ({axis_rq_tuser_axi_bridge_seq, axis_rq_tuser_dma, axis_rq_tuser_st_tx_seq, axis_rq_tuser_st_rx_seq, axis_rq_tuser_hmem_seq}),
        .m_axis_tdata (axis_rq_tdata_mux),
        .m_axis_tkeep (axis_rq_tkeep_mux),
        .m_axis_tvalid(axis_rq_tvalid_mux),
        .m_axis_tready(axis_rq_tready_mux),
        .m_axis_tlast (axis_rq_tlast_mux),
        .m_axis_tid   (),
        .m_axis_tdest (),
        .m_axis_tuser (axis_rq_tuser_mux)
    );

    /* Final shared RQ packet FIFO.
     *
     * The PCIe RQ interface does not permit TVALID bubbles after a packet has
     * started and before TLAST is accepted.  The MM D2H write engine is fed by
     * an elastic AXI read channel, so a temporary AXI-R gap can otherwise
     * propagate through the packet-locking arbiter.  XPM packet mode stores the
     * complete TLP before exposing its first beat to the PCIe hard block.
     *
     * This FIFO is intentionally placed after the five-source RQ arbiter:
     *   - every source sees ordinary AXI backpressure;
     *   - only one BRAM FIFO is required;
     *   - no dynamic FIFO-level comparison is added to the D2H critical path;
     *   - packet boundaries and sequence-number sideband data remain aligned.
     */
    axis_xpm_fifo #(
        .DATA_WIDTH         (AXIS_PCIE_DATA_WIDTH),
        .KEEP_ENABLE        (1),
        .KEEP_WIDTH         (AXIS_PCIE_KEEP_WIDTH),
        .LAST_ENABLE        (1),
        .USER_ENABLE        (1),
        .USER_WIDTH         (AXIS_PCIE_RQ_USER_WIDTH),
        .FIFO_DEPTH         (DMA_RQ_FIFO_DEPTH),
        .PROG_FULL_MARGIN   (8),
        .FIFO_MEMORY_TYPE   ("block"),
        .PACKET_FIFO_ENABLE ("true")
    ) dma_rq_fifo_inst (
        .clk          (clk),
        .rst          (sgdma_rst),
        .s_axis_tdata (axis_rq_tdata_mux),
        .s_axis_tkeep (axis_rq_tkeep_mux),
        .s_axis_tvalid(axis_rq_tvalid_mux),
        .s_axis_tready(axis_rq_tready_mux),
        .s_axis_tlast (axis_rq_tlast_mux),
        .s_axis_tuser (axis_rq_tuser_mux),
        .m_axis_tdata (m_axis_rq_tdata),
        .m_axis_tkeep (m_axis_rq_tkeep),
        .m_axis_tvalid(m_axis_rq_tvalid),
        .m_axis_tready(m_axis_rq_tready),
        .m_axis_tlast (m_axis_rq_tlast),
        .m_axis_tuser (m_axis_rq_tuser),
        .almost_full  (),
        .prog_full    (),
        .wr_data_count()
    );

`ifndef SYNTHESIS
    /* Check the signal actually presented to the PCIe hard block.  Internal
     * producers may remain elastic behind the packet FIFO, but the external RQ
     * stream must stay valid continuously from the accepted first beat through
     * the accepted TLAST beat.
     */
    reg rq_packet_open_sim = 1'b0;
    always @(posedge clk) begin
        if (sgdma_rst) begin
            rq_packet_open_sim <= 1'b0;
        end else begin
            if (rq_packet_open_sim && !m_axis_rq_tvalid)
                $error("PCIe RQ TVALID gap detected before TLAST");

            if (m_axis_rq_tvalid && m_axis_rq_tready) begin
                if (m_axis_rq_tlast)
                    rq_packet_open_sim <= 1'b0;
                else
                    rq_packet_open_sim <= 1'b1;
            end
        end
    end
`endif


    /* ---------------- ST C2H/RX target ---------------- */

    wire        status_busy_st_rx;
    wire [31:0] st_rx_pkt_count;
    wire [31:0] st_rx_byte_count;
    wire [31:0] st_rx_trunc_count;
    wire [31:0] st_rx_drop_count;

    edev_st_d2h_rx_dma_core #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_ST_MTY_WIDTH(ST_AXIS_MTY_WIDTH),
        .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
        .PCIE_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .LEN_WIDTH(DMA_LEN_WIDTH),
        .TAG_WIDTH(DMA_TAG_WIDTH),
        .QID_WIDTH(DMA_QID_WIDTH),
        .RING_INDEX_WIDTH(RING_INDEX_WIDTH),
        .FIFO_DEPTH(32),
        .FIFO_ADDR_WIDTH(5),
        .DESC_POOL_DEPTH(DMA_QUEUE_COUNT)
    ) st_d2h_rx_dma_inst (
        .clk                     (clk),
        .rst                     (sgdma_rst),
        .enable                  (global_enable),
        .requester_id            ({8'd0, 5'd0, 3'd0}),
        .requester_id_enable     (1'b0),
        .cfg_max_payload         (cfg_max_payload_posted),
        .s_axis_desc_host_addr   (st_rx_desc_host_addr),
        .s_axis_desc_len         (st_rx_desc_len),
        .s_axis_desc_tag         (st_rx_desc_tag),
        .s_axis_desc_qid         (st_rx_desc_qid),
        .s_axis_desc_cidx_after  (st_rx_desc_cidx_after),
        .s_axis_desc_flags       (st_rx_desc_flags),
        .s_axis_desc_swtag       (st_rx_desc_swtag),
        .s_axis_desc_user        (st_rx_desc_user),
        .s_axis_desc_wb_base     (st_rx_desc_wb_base),
        .s_axis_desc_irq_enable  (st_rx_desc_irq_enable),
        .s_axis_desc_valid       (st_rx_desc_valid),
        .s_axis_desc_ready       (st_rx_desc_ready),
        .s_axis_st_rx_tdata      (s_axis_st_rx_tdata),
        .s_axis_st_rx_mty        (s_axis_st_rx_mty),
        .s_axis_st_rx_tvalid     (s_axis_st_rx_tvalid),
        .s_axis_st_rx_tready     (s_axis_st_rx_tready),
        .s_axis_st_rx_tlast      (s_axis_st_rx_tlast),
        .s_axis_st_rx_tuser      (s_axis_st_rx_tuser[31:0]),
        .s_axis_st_rx_tqid       (s_axis_st_rx_tqid),
        .s_axis_st_rx_tlen       (s_axis_st_rx_tlen),
        .m_axis_rq_tdata         (axis_rq_tdata_st_rx),
        .m_axis_rq_tkeep         (axis_rq_tkeep_st_rx),
        .m_axis_rq_tvalid        (axis_rq_tvalid_st_rx),
        .m_axis_rq_tready        (axis_rq_tready_st_rx),
        .m_axis_rq_tlast         (axis_rq_tlast_st_rx),
        .m_axis_rq_tuser         (axis_rq_tuser_st_rx),
        .m_axis_status_tag       (st_rx_status_tag),
        .m_axis_status_qid       (st_rx_status_qid),
        .m_axis_status_cidx_after(st_rx_status_cidx_after),
        .m_axis_status_bytes     (st_rx_status_bytes),
        .m_axis_status_error_code(st_rx_status_error_code),
        .m_axis_status_swtag     (st_rx_status_swtag),
        .m_axis_status_user      (st_rx_status_user),
        .m_axis_status_wb_base   (st_rx_status_wb_base),
        .m_axis_status_irq_enable(st_rx_status_irq_enable),
        .m_axis_status_valid     (st_rx_status_valid),
        .m_axis_status_ready     (st_rx_status_ready),
        .status_busy             (status_busy_st_rx),
        .stat_rx_packets         (st_rx_pkt_count),
        .stat_rx_bytes           (st_rx_byte_count),
        .stat_trunc_packets      (st_rx_trunc_count),
        .stat_drop_packets       (st_rx_drop_count)
    );

    /* ---------------- ST H2D/TX target ---------------- */

    wire        status_busy_st_tx;
    wire [31:0] st_tx_pkt_count;
    wire [31:0] st_tx_byte_count;
    wire [31:0] st_tx_rc_err_count;
    wire [31:0] st_tx_desc_err_count;

    edev_st_h2d_tx_dma_core #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
        .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
        .AXIS_ST_MTY_WIDTH(ST_AXIS_MTY_WIDTH),
        .PCIE_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .LEN_WIDTH(DMA_LEN_WIDTH),
        .TAG_WIDTH(DMA_TAG_WIDTH),
        .QID_WIDTH(DMA_QID_WIDTH),
        .RING_INDEX_WIDTH(RING_INDEX_WIDTH),
        /* 512 x 32-byte beats = 16 KiB: enough headroom for one complete
         * 9600-byte jumbo frame during downstream backpressure. */
        .FIFO_ADDR_WIDTH(9),
        .ST_TX_PCIE_TAG(ST_TX_PCIE_TAG),
        .MAX_READ_REQ_BYTES(512),
        .OUTSTANDING_READS(ST_TX_OUTSTANDING_READS)
    ) st_h2d_tx_dma_inst (
        .clk                     (clk),
        .rst                     (sgdma_rst),
        .enable                  (global_enable),
        .requester_id            ({8'd0, 5'd0, 3'd0}),
        .requester_id_enable     (1'b0),
        .max_read_request_size   (cfg_max_read_req),
        .s_axis_desc_host_addr   (st_tx_desc_host_addr),
        .s_axis_desc_len         (st_tx_desc_len),
        .s_axis_desc_tag         (st_tx_desc_tag),
        .s_axis_desc_qid         (st_tx_desc_qid),
        .s_axis_desc_cidx_after  (st_tx_desc_cidx_after),
        .s_axis_desc_flags       (st_tx_desc_flags),
        .s_axis_desc_swtag       (st_tx_desc_swtag),
        .s_axis_desc_user        (st_tx_desc_user),
        .s_axis_desc_valid       (st_tx_desc_valid),
        .s_axis_desc_ready       (st_tx_desc_ready),
        .m_axis_rq_tdata         (axis_rq_tdata_st_tx),
        .m_axis_rq_tkeep         (axis_rq_tkeep_st_tx),
        .m_axis_rq_tvalid        (axis_rq_tvalid_st_tx),
        .m_axis_rq_tready        (axis_rq_tready_st_tx),
        .m_axis_rq_tlast         (axis_rq_tlast_st_tx),
        .m_axis_rq_tuser         (axis_rq_tuser_st_tx),
        .s_axis_rc_tdata         (axis_rc_tdata_st_tx),
        .s_axis_rc_tkeep         (axis_rc_tkeep_st_tx),
        .s_axis_rc_tvalid        (axis_rc_tvalid_st_tx),
        .s_axis_rc_tready        (axis_rc_tready_st_tx),
        .s_axis_rc_tlast         (axis_rc_tlast_st_tx),
        .s_axis_rc_tuser         (axis_rc_tuser_st_tx),
        .m_axis_st_tx_tdata      (m_axis_st_tx_tdata),
        .m_axis_st_tx_mty        (m_axis_st_tx_mty),
        .m_axis_st_tx_tvalid     (m_axis_st_tx_tvalid),
        .m_axis_st_tx_tready     (m_axis_st_tx_tready),
        .m_axis_st_tx_tlast      (m_axis_st_tx_tlast),
        .m_axis_st_tx_tuser      (m_axis_st_tx_tuser),
        .m_axis_st_tx_tqid       (m_axis_st_tx_tqid),
        .m_axis_st_tx_tlen       (m_axis_st_tx_tlen),
        .m_axis_status_tag       (st_tx_status_tag),
        .m_axis_status_qid       (st_tx_status_qid),
        .m_axis_status_cidx_after(st_tx_status_cidx_after),
        .m_axis_status_bytes     (st_tx_status_bytes),
        .m_axis_status_error_code(st_tx_status_error_code),
        .m_axis_status_swtag     (st_tx_status_swtag),
        .m_axis_status_user      (st_tx_status_user),
        .m_axis_status_valid     (st_tx_status_valid),
        .m_axis_status_ready     (st_tx_status_ready),
        .status_busy             (status_busy_st_tx),
        .stat_tx_packets         (st_tx_pkt_count),
        .stat_tx_bytes           (st_tx_byte_count),
        .stat_tx_rc_errors       (st_tx_rc_err_count),
        .stat_tx_desc_errors     (st_tx_desc_err_count)
    );

    /* ---------------- Host memory micro-engine ---------------- */

    wire status_error_cor_hmem;
    wire status_error_uncor_hmem;

    edev_hostmem_pcie_us #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
        .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
        .PCIE_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .TAG_WIDTH(DMA_TAG_WIDTH),
        .TX_FC_ENABLE(REQUESTER_TX_FC_ENABLE),
        .DESC_WIDTH(256),
        .CMPL_WIDTH(256),
        .DESC_FETCH_PCIE_TAG(HMEM_PCIE_TAG)
    ) hostmem_engine_inst (
        .clk                (clk),
        .rst                (sgdma_rst),
        .desc_fetch_addr    (desc_fetch_addr),
        .desc_fetch_tag     (desc_fetch_tag),
        .desc_fetch_count   (desc_fetch_count),
        .desc_fetch_is_st   (desc_fetch_is_st),
        .desc_fetch_valid   (desc_fetch_valid),
        .desc_fetch_ready   (desc_fetch_ready),
        .desc_resp_tag      (desc_resp_tag),
        .desc_resp_data     (desc_resp_data),
        .desc_resp_error    (desc_resp_error),
        .desc_resp_valid    (desc_resp_valid),
        .desc_resp_ready    (desc_resp_ready),
        .cmpl_write_addr    (cmpl_write_addr),
        .cmpl_write_data    (cmpl_write_data),
        .cmpl_write_valid   (cmpl_write_valid),
        .cmpl_write_fence   (cmpl_write_fence),
        .cmpl_write_ready   (cmpl_write_ready),
        .cmpl_write_done    (cmpl_write_done),
        .s_axis_rc_tdata    (axis_rc_tdata_hmem),
        .s_axis_rc_tkeep    (axis_rc_tkeep_hmem),
        .s_axis_rc_tvalid   (axis_rc_tvalid_hmem),
        .s_axis_rc_tready   (axis_rc_tready_hmem),
        .s_axis_rc_tlast    (axis_rc_tlast_hmem),
        .s_axis_rc_tuser    (axis_rc_tuser_hmem),
        .m_axis_rq_tdata    (axis_rq_tdata_hmem),
        .m_axis_rq_tkeep    (axis_rq_tkeep_hmem),
        .m_axis_rq_tvalid   (axis_rq_tvalid_hmem),
        .m_axis_rq_tready   (axis_rq_tready_hmem),
        .m_axis_rq_tlast    (axis_rq_tlast_hmem),
        .m_axis_rq_tuser    (axis_rq_tuser_hmem),
        .pcie_tx_fc_nph_av  (pcie_tx_fc_nph_av),
        .pcie_tx_fc_ph_av   (pcie_tx_fc_ph_av),
        .pcie_tx_fc_pd_av   (pcie_tx_fc_pd_av),
        .requester_id       ({8'd0, 5'd0, 3'd0}),
        .requester_id_enable(1'b0),
        .status_error_cor   (status_error_cor_hmem),
        .status_error_uncor (status_error_uncor_hmem)
    );

    /* ---------------- AXI-MM slave to PCIe requester bridge ---------------- */

    wire status_busy_axi_bridge;
    wire status_error_cor_axi_bridge;
    wire status_error_uncor_axi_bridge;

    edev_axi_pcie_bridge #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
        .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
        .AXI_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .PCIE_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .TABLE_ENTRIES(4),
        .READ_TAG_BASE(AXI_BRIDGE_PCIE_TAG),
        .READ_TAG_COUNT(8),
        .READ_REQ_BYTES(512),
        .READ_WINDOW_BYTES(4096),
        .WRITE_TLP_BYTES(32)
    ) axi_pcie_bridge_inst (
        .clk                  (clk),
        .rst                  (sgdma_rst),
        .enable               (global_enable),
        .table_enable         (axi_bridge_table_enable),
        .table_axi_base       (axi_bridge_table_axi_base),
        .table_pcie_base      (axi_bridge_table_pcie_base),
        .table_mask           (axi_bridge_table_mask),
        .requester_id         ({8'd0, 5'd0, 3'd0}),
        .requester_id_enable  (1'b0),
        .ext_tag_enable       (ext_tag_enable),
        .max_read_request_size(cfg_max_read_req),
        .s_axi_awid           (s_axi_pcie_awid),
        .s_axi_awaddr         (s_axi_pcie_awaddr),
        .s_axi_awlen          (s_axi_pcie_awlen),
        .s_axi_awsize         (s_axi_pcie_awsize),
        .s_axi_awburst        (s_axi_pcie_awburst),
        .s_axi_awvalid        (s_axi_pcie_awvalid),
        .s_axi_awready        (s_axi_pcie_awready),
        .s_axi_wdata          (s_axi_pcie_wdata),
        .s_axi_wstrb          (s_axi_pcie_wstrb),
        .s_axi_wlast          (s_axi_pcie_wlast),
        .s_axi_wvalid         (s_axi_pcie_wvalid),
        .s_axi_wready         (s_axi_pcie_wready),
        .s_axi_bid            (s_axi_pcie_bid),
        .s_axi_bresp          (s_axi_pcie_bresp),
        .s_axi_bvalid         (s_axi_pcie_bvalid),
        .s_axi_bready         (s_axi_pcie_bready),
        .s_axi_arid           (s_axi_pcie_arid),
        .s_axi_araddr         (s_axi_pcie_araddr),
        .s_axi_arlen          (s_axi_pcie_arlen),
        .s_axi_arsize         (s_axi_pcie_arsize),
        .s_axi_arburst        (s_axi_pcie_arburst),
        .s_axi_arvalid        (s_axi_pcie_arvalid),
        .s_axi_arready        (s_axi_pcie_arready),
        .s_axi_rid            (s_axi_pcie_rid),
        .s_axi_rdata          (s_axi_pcie_rdata),
        .s_axi_rresp          (s_axi_pcie_rresp),
        .s_axi_rlast          (s_axi_pcie_rlast),
        .s_axi_rvalid         (s_axi_pcie_rvalid),
        .s_axi_rready         (s_axi_pcie_rready),
        .m_axis_rq_tdata      (axis_rq_tdata_axi_bridge),
        .m_axis_rq_tkeep      (axis_rq_tkeep_axi_bridge),
        .m_axis_rq_tvalid     (axis_rq_tvalid_axi_bridge),
        .m_axis_rq_tready     (axis_rq_tready_axi_bridge),
        .m_axis_rq_tlast      (axis_rq_tlast_axi_bridge),
        .m_axis_rq_tuser      (axis_rq_tuser_axi_bridge),
        .s_axis_rc_tdata      (axis_rc_tdata_axi_bridge),
        .s_axis_rc_tkeep      (axis_rc_tkeep_axi_bridge),
        .s_axis_rc_tvalid     (axis_rc_tvalid_axi_bridge),
        .s_axis_rc_tready     (axis_rc_tready_axi_bridge),
        .s_axis_rc_tlast      (axis_rc_tlast_axi_bridge),
        .s_axis_rc_tuser      (axis_rc_tuser_axi_bridge),
        .status_busy          (status_busy_axi_bridge),
        .status_error_cor     (status_error_cor_axi_bridge),
        .status_error_uncor   (status_error_uncor_axi_bridge)
    );

    /* ---------------- Payload PCIe AXI DMA ---------------- */

    wire status_error_cor_dma;
    wire status_error_uncor_dma;
    wire status_rd_busy;
    wire status_wr_busy;

    pcie_us_axi_dma #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
        .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXI_MAX_BURST_LEN(256),
        .PCIE_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .PCIE_TAG_COUNT(DMA_PCIE_TAG_COUNT_EFFECTIVE),
        .READ_REQUEST_TAG_LIMIT(DMA_READ_REQ_TAG_LIMIT_EFFECTIVE),
        .LEN_WIDTH(DMA_LEN_WIDTH),
        .TAG_WIDTH(DMA_TAG_WIDTH),
        .RQ_SEQ_NUM_ENABLE(1),
        .READ_TX_FC_ENABLE(REQUESTER_TX_FC_ENABLE),
        .WRITE_TX_FC_ENABLE(REQUESTER_TX_FC_ENABLE),
        .WRITE_AXI_RDATA_FIFO_DEPTH(DMA_AXI_RDATA_FIFO_DEPTH)
    ) payload_dma_inst (
        .clk                           (clk),
        .rst                           (sgdma_rst),
        .s_axis_rc_tdata               (axis_rc_tdata_dma),
        .s_axis_rc_tkeep               (axis_rc_tkeep_dma),
        .s_axis_rc_tvalid              (axis_rc_tvalid_dma),
        .s_axis_rc_tready              (axis_rc_tready_dma),
        .s_axis_rc_tlast               (axis_rc_tlast_dma),
        .s_axis_rc_tuser               (axis_rc_tuser_dma),
        .m_axis_rq_tdata               (axis_rq_tdata_dma),
        .m_axis_rq_tkeep               (axis_rq_tkeep_dma),
        .m_axis_rq_tvalid              (axis_rq_tvalid_dma),
        .m_axis_rq_tready              (axis_rq_tready_dma),
        .m_axis_rq_tlast               (axis_rq_tlast_dma),
        .m_axis_rq_tuser               (axis_rq_tuser_dma),
        .s_axis_rq_seq_num_0           (s_axis_rq_seq_num_0),
        .s_axis_rq_seq_num_valid_0     (s_axis_rq_seq_num_valid_0),
        .s_axis_rq_seq_num_1           (s_axis_rq_seq_num_1),
        .s_axis_rq_seq_num_valid_1     (s_axis_rq_seq_num_valid_1),
        .pcie_tx_fc_nph_av             (pcie_tx_fc_nph_av),
        .pcie_tx_fc_ph_av              (pcie_tx_fc_ph_av),
        .pcie_tx_fc_pd_av              (pcie_tx_fc_pd_av),
        .s_axis_read_desc_pcie_addr    (mm_read_desc_pcie_addr),
        .s_axis_read_desc_axi_addr     (mm_read_desc_axi_addr),
        .s_axis_read_desc_len          (mm_read_desc_len),
        .s_axis_read_desc_tag          (mm_read_desc_tag),
        .s_axis_read_desc_valid        (mm_read_desc_valid),
        .s_axis_read_desc_ready        (mm_read_desc_ready),
        .m_axis_read_desc_status_tag   (mm_read_status_tag),
        .m_axis_read_desc_status_error (mm_read_status_error),
        .m_axis_read_desc_status_valid (mm_read_status_valid),
        .s_axis_write_desc_pcie_addr   (mm_write_desc_pcie_addr),
        .s_axis_write_desc_axi_addr    (mm_write_desc_axi_addr),
        .s_axis_write_desc_len         (mm_write_desc_len),
        .s_axis_write_desc_tag         (mm_write_desc_tag),
        .s_axis_write_desc_valid       (mm_write_desc_valid),
        .s_axis_write_desc_ready       (mm_write_desc_ready),
        .m_axis_write_desc_status_tag  (mm_write_status_tag),
        .m_axis_write_desc_status_error(mm_write_status_error),
        .m_axis_write_desc_status_valid(mm_write_status_valid),
        .m_axi_awid                    (m_axi_awid),
        .m_axi_awaddr                  (m_axi_awaddr),
        .m_axi_awlen                   (m_axi_awlen),
        .m_axi_awsize                  (m_axi_awsize),
        .m_axi_awburst                 (m_axi_awburst),
        .m_axi_awlock                  (m_axi_awlock),
        .m_axi_awcache                 (m_axi_awcache),
        .m_axi_awprot                  (m_axi_awprot),
        .m_axi_awvalid                 (m_axi_awvalid),
        .m_axi_awready                 (m_axi_awready),
        .m_axi_wdata                   (m_axi_wdata),
        .m_axi_wstrb                   (m_axi_wstrb),
        .m_axi_wlast                   (m_axi_wlast),
        .m_axi_wvalid                  (m_axi_wvalid),
        .m_axi_wready                  (m_axi_wready),
        .m_axi_bid                     (m_axi_bid),
        .m_axi_bresp                   (m_axi_bresp),
        .m_axi_bvalid                  (m_axi_bvalid),
        .m_axi_bready                  (m_axi_bready),
        .m_axi_arid                    (m_axi_arid),
        .m_axi_araddr                  (m_axi_araddr),
        .m_axi_arlen                   (m_axi_arlen),
        .m_axi_arsize                  (m_axi_arsize),
        .m_axi_arburst                 (m_axi_arburst),
        .m_axi_arlock                  (m_axi_arlock),
        .m_axi_arcache                 (m_axi_arcache),
        .m_axi_arprot                  (m_axi_arprot),
        .m_axi_arvalid                 (m_axi_arvalid),
        .m_axi_arready                 (m_axi_arready),
        .m_axi_rid                     (m_axi_rid),
        .m_axi_rdata                   (m_axi_rdata),
        .m_axi_rresp                   (m_axi_rresp),
        .m_axi_rlast                   (m_axi_rlast),
        .m_axi_rvalid                  (m_axi_rvalid),
        .m_axi_rready                  (m_axi_rready),
        .read_enable                   (1'b1),
        .write_enable                  (1'b1),
        .read_request_enable           (payload_read_request_enable),
        .ext_tag_enable                (ext_tag_enable),
        .requester_id                  ({8'd0, 5'd0, 3'd0}),
        .requester_id_enable           (1'b0),
        .max_read_request_size         (cfg_max_read_req),
        .max_payload_size              (cfg_max_payload_posted),
        .status_rd_busy                (status_rd_busy),
        .status_wr_busy                (status_wr_busy),
        .status_error_cor              (status_error_cor_dma),
        .status_error_uncor            (status_error_uncor_dma)
    );

    /* ---------------- Error and MSI generation ---------------- */

    wire [4:0] cor_pulses = {status_error_cor_bar4, status_error_cor_hmem, status_error_cor_dma, status_error_cor_bar2, status_error_cor_bar0};
    wire [4:0] uncor_pulses = {status_error_uncor_bar4, status_error_uncor_hmem, status_error_uncor_dma, status_error_uncor_bar2, status_error_uncor_bar0};

    pulse_merge #(
        .INPUT_WIDTH(5),
        .COUNT_WIDTH(4)
    ) status_error_cor_pm_inst (
        .clk      (clk),
        .rst      (rst),
        .pulse_in (cor_pulses),
        .count_out(),
        .pulse_out(status_error_cor)
    );

    pulse_merge #(
        .INPUT_WIDTH(5),
        .COUNT_WIDTH(4)
    ) status_error_uncor_pm_inst (
        .clk      (clk),
        .rst      (rst),
        .pulse_in (uncor_pulses),
        .count_out(),
        .pulse_out(status_error_uncor)
    );

    /* ---------------- MSI-X external-table interrupt generation ---------------- */

    wire [8:0] msix_irq_vector = MSIX_SINGLE_VECTOR ? 9'd0 : irq_qid;
    wire [9:0] msix_active_vector_count = MSIX_SINGLE_VECTOR ? 10'd1 : enabled_queue_count;

    pcie_us_msix #(
        .VECTOR_COUNT(DMA_QUEUE_COUNT),
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_CQ_USER_WIDTH(AXIS_PCIE_CQ_USER_WIDTH),
        .AXIS_PCIE_CC_USER_WIDTH(AXIS_PCIE_CC_USER_WIDTH),
        .BAR4_ADDR_WIDTH(16),
        .PBA_OFFSET(32'h0000_8000)
    ) pcie_us_msix_inst (
        .clk                                  (clk),
        .rst                                  (rst),
        .msix_irq_valid                       (irq_valid),
        .msix_irq_qid                         (msix_irq_vector),
        .active_vector_count                  (msix_active_vector_count),
        .s_axis_cq_tdata                      (axis_cq_tdata_bar_4),
        .s_axis_cq_tkeep                      (axis_cq_tkeep_bar_4),
        .s_axis_cq_tvalid                     (axis_cq_tvalid_bar_4),
        .s_axis_cq_tready                     (axis_cq_tready_bar_4),
        .s_axis_cq_tlast                      (axis_cq_tlast_bar_4),
        .s_axis_cq_tuser                      (axis_cq_tuser_bar_4),
        .m_axis_cc_tdata                      (axis_cc_tdata_bar_4),
        .m_axis_cc_tkeep                      (axis_cc_tkeep_bar_4),
        .m_axis_cc_tvalid                     (axis_cc_tvalid_bar_4),
        .m_axis_cc_tready                     (axis_cc_tready_bar_4),
        .m_axis_cc_tlast                      (axis_cc_tlast_bar_4),
        .m_axis_cc_tuser                      (axis_cc_tuser_bar_4),
        .completer_id                         ({8'd0, 5'd0, 3'd0}),
        .completer_id_enable                  (1'b0),
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
        .status_error_cor                     (status_error_cor_bar4),
        .status_error_uncor                   (status_error_uncor_bar4)
    );



endmodule

`resetall
