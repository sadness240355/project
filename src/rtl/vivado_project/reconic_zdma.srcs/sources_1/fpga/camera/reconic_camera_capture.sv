`timescale 1ns / 1ps
`default_nettype none

// AXI-Lite control is the OpenNIC 125 MHz domain; AXI/DDR remains the
// independent high-throughput data clock domain.
module reconic_camera_capture #(
    parameter integer AXI_DATA_WIDTH = 512,
    parameter integer AXI_ADDR_WIDTH = 32,
    parameter integer AXI_ID_WIDTH   = 3,
    // Camera Link has no READY. The CDC BRAM holds 32,768 pixel records;
    // bounded PCIe-host bursts guarantee recurring arbitration opportunities.
    parameter integer FIFO_DEPTH     = 32768,
    // Match the 4 KiB DDR burst to reduce switches on the shared write port.
    // Always split at AXI's 4 KiB boundary, including nonzero ring offsets.
    parameter integer AXI_BURST_BEATS = 64,
    parameter integer AXI_BURST_FIFO_DEPTH = 256,
    parameter integer AXI_CMD_FIFO_DEPTH = 64,
    parameter integer AXI_MAX_OUTSTANDING = 4,
    // DDR video and the credit-gated direct host branch have separate queues.
    parameter bit DIRECT_HOST_PRESENT = 1'b1,
    parameter integer AXIL_CLK_HZ = 125000000,
    parameter integer CAMERA_UART_BAUD = 9600,
    // HLS ISP defaults. Gains and CCM entries are Q4.12. Start from a neutral,
    // shadow-preserving profile: board software programs calibrated values
    // while capture is stopped, before the HLS core is released from reset.
    parameter logic [3:0]  ISP_ENABLE      = 4'b1111,
    parameter logic [7:0]  ISP_BLACK_LEVEL = 8'd0,
    parameter logic [15:0] ISP_WB_GAIN_R   = 16'h1000,
    parameter logic [15:0] ISP_WB_GAIN_G   = 16'h1000,
    parameter logic [15:0] ISP_WB_GAIN_B   = 16'h1000,
    parameter logic signed [15:0] ISP_CCM_00 = 16'sh1000,
    parameter logic signed [15:0] ISP_CCM_01 = 16'sh0000,
    parameter logic signed [15:0] ISP_CCM_02 = 16'sh0000,
    parameter logic signed [15:0] ISP_CCM_10 = 16'sh0000,
    parameter logic signed [15:0] ISP_CCM_11 = 16'sh1000,
    parameter logic signed [15:0] ISP_CCM_12 = 16'sh0000,
    parameter logic signed [15:0] ISP_CCM_20 = 16'sh0000,
    parameter logic signed [15:0] ISP_CCM_21 = 16'sh0000,
    parameter logic signed [15:0] ISP_CCM_22 = 16'sh1000
) (
    input  wire                       cmlk_clk_p,
    input  wire                       cmlk_clk_n,
    input  wire [11:0]                cmlk_data_p,
    input  wire [11:0]                cmlk_data_n,
    input  wire                       cmlk_ref_clk,
    input  wire                       cmlk_idelay_ref_clk,
    input  wire                       cmlk_idelay_ref_locked,
    input  wire                       cmlk_reset,
    input  wire                       cmlk_ser_rx,
    output wire                       cmlk_ser_tx,
    output wire [3:0]                 cmlk_cc,
    // Level request: asserted while at least one committed DDR frame is
    // pending and software has enabled frame-ready interrupts.  The SGDMA
    // clock domain performs the CDC and edge detection.
    output wire                       camera_frame_irq,

    input  wire                       axil_aclk,
    input  wire                       axil_aresetn,
    input  wire                       s_axil_awvalid,
    input  wire [31:0]                s_axil_awaddr,
    output logic                      s_axil_awready,
    input  wire                       s_axil_wvalid,
    input  wire [31:0]                s_axil_wdata,
    output logic                      s_axil_wready,
    output logic                      s_axil_bvalid,
    output logic [1:0]                s_axil_bresp,
    input  wire                       s_axil_bready,
    input  wire                       s_axil_arvalid,
    input  wire [31:0]                s_axil_araddr,
    output logic                      s_axil_arready,
    output logic                      s_axil_rvalid,
    output logic [31:0]               s_axil_rdata,
    output logic [1:0]                s_axil_rresp,
    input  wire                       s_axil_rready,

    input  wire                       axi_aclk,
    input  wire                       axi_aresetn,
    output logic [AXI_ID_WIDTH-1:0]   m_axi_awid,
    output logic [AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output logic [7:0]                m_axi_awlen,
    output logic [2:0]                m_axi_awsize,
    output logic [1:0]                m_axi_awburst,
    output logic                      m_axi_awlock,
    output logic [3:0]                m_axi_awcache,
    output logic [2:0]                m_axi_awprot,
    output logic [3:0]                m_axi_awqos,
    output logic [3:0]                m_axi_awregion,
    output logic                      m_axi_awvalid,
    input  wire                       m_axi_awready,
    output logic [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output logic [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output logic                      m_axi_wlast,
    output logic                      m_axi_wvalid,
    input  wire                       m_axi_wready,
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid,
    input  wire [1:0]                 m_axi_bresp,
    input  wire                       m_axi_bvalid,
    output logic                      m_axi_bready,
    output wire [63:0] m_host_awaddr, output wire [7:0] m_host_awlen,
    output wire m_host_awvalid, input wire m_host_awready,
    output wire [511:0] m_host_wdata, output wire [63:0] m_host_wstrb,
    output wire m_host_wlast, m_host_wvalid, input wire m_host_wready,
    input wire [1:0] m_host_bresp, input wire m_host_bvalid,
    output wire m_host_bready
);

    localparam integer AXI_KEEP_WIDTH = AXI_DATA_WIDTH/8;
    localparam integer WORD_BYTES = 10;
    localparam integer BURST_COUNT_WIDTH = $clog2(AXI_BURST_BEATS+1);
    localparam integer OUTSTANDING_COUNT_WIDTH = $clog2(AXI_MAX_OUTSTANDING+1);
    localparam integer CAMERA_RING_SLOTS = 32;
    localparam [31:0] CAMERA_RING_SLOTS_U32 = 32'd32;
    localparam integer BEAT_FIFO_WIDTH = AXI_DATA_WIDTH + AXI_KEEP_WIDTH;
    localparam integer CMD_META_WIDTH = 128;
    localparam integer CMD_FIFO_WIDTH = 5 + 1 + 1 + CMD_META_WIDTH + AXI_ADDR_WIDTH + 8;
    localparam [AXI_ADDR_WIDTH-1:0] AXI_KEEP_WIDTH_ADDR = AXI_KEEP_WIDTH;
    localparam [OUTSTANDING_COUNT_WIDTH-1:0] AXI_MAX_OUTSTANDING_COUNT =
        AXI_MAX_OUTSTANDING;
    localparam logic [BURST_COUNT_WIDTH-1:0] AXI_BURST_BEATS_L = AXI_BURST_BEATS;

    localparam [31:0] REG_CONTROL      = 32'h00;
    localparam [31:0] REG_STATUS       = 32'h04;
    localparam [31:0] REG_IRQ_STATUS   = 32'h08;
    localparam [31:0] REG_IRQ_ENABLE   = 32'h0c;
    localparam [31:0] REG_RING_BASE_LO = 32'h10;
    localparam [31:0] REG_RING_BASE_HI = 32'h14;
    localparam [31:0] REG_FRAME_STRIDE = 32'h18;
    localparam [31:0] REG_RING_SLOTS   = 32'h1c;
    localparam [31:0] REG_CONSUMER_IDX = 32'h20;
    localparam [31:0] REG_PRODUCER_IDX = 32'h24;
    localparam [31:0] REG_WIDTH        = 32'h28;
    localparam [31:0] REG_HEIGHT       = 32'h2c;
    localparam [31:0] REG_FRAME_BYTES  = 32'h30;
    localparam [31:0] REG_FRAME_SEQ    = 32'h34;
    localparam [31:0] REG_DROP_COUNT   = 32'h38;
    localparam [31:0] REG_OUTPUT_MODE = 32'h3c; // signature 0x42590000; bit0=RAW BayerRG8
    wire hls_core_input_ready, raw_input_ready;
    logic raw_output_reg = 1'b0;
    wire raw_output_axi;
    xpm_cdc_single #(.DEST_SYNC_FF(2),.INIT_SYNC_FF(0),.SIM_ASSERT_CHK(0),.SRC_INPUT_REG(0)) raw_output_cdc (
        .src_clk(axil_aclk),.src_in(raw_output_reg),.dest_clk(axi_aclk),.dest_out(raw_output_axi));
    localparam [31:0] REG_DIAG_FVAL_EDGES      = 32'h40;
    localparam [31:0] REG_DIAG_LVAL_EDGES      = 32'h44;
    localparam [31:0] REG_DIAG_DVAL_EDGES      = 32'h48;
    localparam [31:0] REG_DIAG_PIX_VALID_WORDS = 32'h4c;
    localparam [31:0] REG_DIAG_BAD_FRAMES      = 32'h50;
    localparam [31:0] REG_DIAG_LAST_LINE_BYTES = 32'h54;
    localparam [31:0] REG_EXPECTED_LINE_BYTES  = 32'h58;
    localparam [31:0] REG_EXPECTED_FRAME_BYTES = 32'h5c;
    localparam [31:0] REG_DIAG_DROP_FRAME_BYTES = 32'h60;
    localparam [31:0] REG_DIAG_DROP_REASON      = 32'h64;
    localparam [31:0] REG_EXPECTED_HEIGHT       = 32'h68;
    // Build/diagnostic CSRs. Software uses BUILD_ID to reject stale bitstreams
    // that still contain the historical double-publication path.
    localparam [31:0] REG_BUILD_ID              = 32'h6c;
    localparam [31:0] REG_BUILD_VERSION         = 32'h70;
    localparam [31:0] REG_DUP_COMMIT_COUNT      = 32'h74;
    localparam [31:0] REG_HLS_STALL_CYCLES      = 32'h78;
    localparam [31:0] REG_AXI_STALL_CYCLES      = 32'h7c;
    // Camera Link group-integrity diagnostics. In 80-bit Deca mode the three
    // Channel Link groups physically replicate LVAL only; cmlk_decode maps
    // those copies into the logical sync vectors used by this monitor.
    localparam [31:0] REG_CMLK_SYNC_MISMATCH_CYCLES = 32'hb0;
    localparam [31:0] REG_CMLK_SYNC_MISMATCH_EVENTS = 32'hb4;
    localparam [31:0] REG_CMLK_SYNC_MISMATCH_MASK   = 32'hb8;
    localparam [31:0] REG_CMLK_ALIGN_LOSS_COUNT     = 32'hbc;
    localparam [31:0] REG_HLS_RESYNC_COUNT          = 32'hc0;
    localparam [31:0] REG_ISP_ENABLE                = 32'hc4;
    localparam [31:0] REG_ISP_BLACK_LEVEL           = 32'hc8;
    localparam [31:0] REG_ISP_WB_GAIN_R             = 32'hcc;
    localparam [31:0] REG_ISP_WB_GAIN_G             = 32'hd0;
    localparam [31:0] REG_ISP_WB_GAIN_B             = 32'hd4;
    localparam [31:0] REG_ISP_CCM_00                = 32'hd8;
    localparam [31:0] REG_ISP_CCM_01                = 32'hdc;
    localparam [31:0] REG_ISP_CCM_02                = 32'he0;
    localparam [31:0] REG_ISP_CCM_10                = 32'he4;
    localparam [31:0] REG_ISP_CCM_11                = 32'he8;
    localparam [31:0] REG_ISP_CCM_12                = 32'hec;
    localparam [31:0] REG_ISP_CCM_20                = 32'hf0;
    localparam [31:0] REG_ISP_CCM_21                = 32'hf4;
    localparam [31:0] REG_ISP_CCM_22                = 32'hf8;
    localparam [31:0] CAMERA_BUILD_ID            = 32'h5247_4233; // "RGB3"
    // 3.5: continuously validate the Camera Link clock word after startup,
    // automatically retrain after eight consecutive mismatches, and reject
    // the interrupted frame instead of publishing a horizontally shifted one.
    // 3.6: after reset/retrain, wait for FVAL low and the next real rising edge
    // before accepting pixels, so a mid-frame reset cannot become a false SOF.
    // 3.7: register HLS backpressure with a no-bubble skid buffer to break the
    // long 250 MHz capture-control path without changing stream throughput.
    // 3.8: add fixed-point BLC, Bayer-site white balance, 3x3 CCM and a
    // 12-bank dual-port sRGB LUT while preserving eight pixels/clock.
    // 3.9: preserve the next SOF on a short input frame, pad and reject only
    // the damaged frame, then resume on the genuine frame boundary. Also make
    // ISP enable/BLC programmable while capture is stopped.
    // 3.10: synchronize IDELAYCTRL reset release in its own 400 MHz reference
    // domain. This removes the routed asynchronous recovery violation without
    // inserting latency into the eight-pixel-per-cycle capture data path.
    // 3.11: use a conservative IMX250 daylight WB/CCM starting profile.
    // 3.12: make WB and all nine CCM coefficients stopped-capture CSRs. The
    // complete profile is synchronized and snapshotted before HLS start. A
    // frame containing any replicated X/Y/Z sync disagreement is marked bad
    // and never published. The eight-pixel/cycle path and II are unchanged.
    // 3.13: initial port-H bit-2 repair using the nearest same-colour pixel F.
    // 3.14: replace that unconditional copy with a D/F/H same-colour majority
    // vote. Raw H is preserved at D/F bit transitions, removing the artificial
    // edge/dark-region noise while retaining isolated-bit correction. The vote
    // is one LUT before the existing register; framing and 8-pixel/cycle
    // throughput are unchanged.
    // 3.15: make the existing software capture reset also retrain the Camera
    // Link PLL, IDELAY and 8-to-7 gearbox. This establishes a deterministic
    // word phase on every application start and prevents horizontal rotation
    // with a wrapped/stale-looking right edge after a camera restart.
    // 3.16: decode Camera Link 2.0 80-bit/1X10_1Y ports A..J and trim the
    // invalid taps in each final line word. A fixed 10-byte to 8-byte gearbox
    // in the 250 MHz domain preserves the existing HLS interface and II=1.
    // 3.17: store and transport compact RGB888. The HLS block emits 24 bytes
    // for eight pixels and a timing-isolated byte packer feeds 512-bit DDR AXI.
    // 3.18: make every HLS SOF an unconditional RGB888 packer byte-phase
    // boundary. If an upstream frame is aborted without TLAST, its partial
    // group can no longer join the next frame and form a stale right edge.
    // This changes only collection control; the normal 8-group/3-beat
    // schedule and TREADY timing are unchanged.
    // 3.24: isolate the live ISP CSR window and provide full-word write strobes.
    // 3.25: reserve registered FIFO writes before asserting stream READY.
    // Real overflow is sticky and rejects the affected DDR frame.
    // 0x0003001e: ISP split to independent host memory and DDR/RDMA readers.
    // 0x0003001f: shared camera AW/W/B timing slices, same frame/ownership ABI.
    // 0x00030020: physical-frame input loss, recover without clearing diagnostics.
    // 0x30024: elastic AXI bridge and host FIFO timing pipelines; frame-stop ABI retained.
    // 0x30027: DDR remains full rate; host admits complete frames by credit.
    // 0x30028: DDR-first burst scheduling and early abort of rejected host frames.
    // 0x30029: continuous full-frame ISP->PCIe host preview; DDR remains RDMA-only.
    // 0x3002b: camera input backpressure and overflow diagnostics.
    // 0x3002e: PCIe bridge packs full AXI beats into 32B MWr and prefetches W.
    // 0x3002f: explicit stopped-source abort drains unpublished host tail safely.
    // 0x30030: bounded PCIe host bursts and BRAM-only camera/host buffering.
    localparam [31:0] CAMERA_BUILD_VERSION       = 32'h0003_0030;

    wire host_direct_enabled,host_stream_ready,host_direct_drained,host_tail_abort;
    wire host_frame_credit, host_meta_take, host_meta_bad;
    wire [31:0] host_csr_rdata;
    wire host_read_select=s_axil_araddr[11:0]>=12'ha00 && s_axil_araddr[11:0]<12'hb80;
    wire [607:0] drop_hist_axi,drop_hist_axil;
    wire [1023:0] ddr_frame_ids_axi,ddr_frame_ids_axil;

    logic        pix_clk;
    logic        pix_rst;
    logic        cmlk_link_ready;
    logic [31:0] cmlk_align_loss_count;
    logic [31:0] cmlk_align_loss_count_axi;
    logic [31:0] cmlk_align_loss_count_axil;
    logic        cmlk_link_ready_axil;
    logic [95:0] cmlk_diag_cdc_dst;
    wire [31:0] diag_cmlk_sync_mismatch_cycles_axil =
        cmlk_diag_cdc_dst[95 -: 32];
    wire [31:0] diag_cmlk_sync_mismatch_events_axil =
        cmlk_diag_cdc_dst[63 -: 32];
    wire [31:0] diag_cmlk_sync_mismatch_mask_axil =
        cmlk_diag_cdc_dst[31 -: 32];
    logic        cmlk_ref_domain_rst;
    logic        pix_domain_rst_base;
    logic        axi_domain_rst_base;
    logic        pix_domain_rst;
    logic        axi_domain_rst;
    logic        soft_reset_req;
    logic        soft_clear_req;
    logic        soft_reset_pix;
    logic        soft_reset_axi;
    logic        soft_clear_pix;
    logic        soft_clear_axi;
    logic        f_val;
    logic        l_val;
    logic        d_val;
     logic [2:0] cmlk_sync_x;
     logic [2:0] cmlk_sync_y;
     logic [2:0] cmlk_sync_z;
    logic [7:0]  port_a;
    logic [7:0]  port_b;
    logic [7:0]  port_c;
    logic [7:0]  port_d;
    logic [7:0]  port_e;
    logic [7:0]  port_f;
    logic [7:0]  port_g;
    logic [7:0]  port_h;
    logic [7:0]  port_i;
    logic [7:0]  port_j;

    xpm_cdc_async_rst #(
        .DEST_SYNC_FF    (4),
        .INIT_SYNC_FF    (0),
        .RST_ACTIVE_HIGH (1)
    ) cmlk_ref_reset_cdc_inst (
        // Retrain PLL/IDELAY/gearbox for every software capture reset. A
        // camera stop or transport reconfiguration can otherwise leave the
        // previous 8-to-7 word phase alive across sessions, producing a
        // horizontally rotated frame with a wrap seam at the right edge.
        .src_arst  (cmlk_reset || soft_reset_req),
        .dest_clk  (cmlk_ref_clk),
        .dest_arst (cmlk_ref_domain_rst)
    );

    cmlk_decode cmlk_decode_inst (
        .clkin2_p (cmlk_clk_p),
        .clkin2_n (cmlk_clk_n),
        .datain2_p(cmlk_data_p),
        .datain2_n(cmlk_data_n),
        .rx_reset (cmlk_ref_domain_rst),
        .ref_clk  (cmlk_idelay_ref_clk),
        .ref_clk_locked(cmlk_idelay_ref_locked),
        .pix_clk  (pix_clk),
        .pix_rst  (pix_rst),
        .link_ready(cmlk_link_ready),
        .align_loss_count(cmlk_align_loss_count),
        .sync_x   (cmlk_sync_x),
        .sync_y   (cmlk_sync_y),
        .sync_z   (cmlk_sync_z),
        .l_val    (l_val),
        .f_val    (f_val),
        .d_val    (d_val),
        .port_a   (port_a),
        .port_b   (port_b),
        .port_c   (port_c),
        .port_d   (port_d),
        .port_e   (port_e),
        .port_f   (port_f),
        .port_g   (port_g),
        .port_h   (port_h),
        .port_i   (port_i),
        .port_j   (port_j)
    );

    xpm_cdc_async_rst #(
        .DEST_SYNC_FF    (4),
        .INIT_SYNC_FF    (0),
        .RST_ACTIVE_HIGH (1)
    ) pix_reset_cdc_inst (
        .src_arst  (pix_rst || cmlk_reset || !axi_aresetn),
        .dest_clk  (pix_clk),
        .dest_arst (pix_domain_rst_base)
    );

    xpm_cdc_async_rst #(
        .DEST_SYNC_FF    (4),
        .INIT_SYNC_FF    (0),
        .RST_ACTIVE_HIGH (1)
    ) axi_reset_cdc_inst (
        .src_arst  (!axi_aresetn || cmlk_reset),
        .dest_clk  (axi_aclk),
        .dest_arst (axi_domain_rst_base)
    );

    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0)
    ) soft_reset_pix_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (soft_reset_req),
        .dest_clk (pix_clk),
        .dest_out (soft_reset_pix)
    );

    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0)
    ) soft_reset_axi_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (soft_reset_req),
        .dest_clk (axi_aclk),
        .dest_out (soft_reset_axi)
    );

    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0)
    ) soft_clear_pix_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (soft_clear_req),
        .dest_clk (pix_clk),
        .dest_out (soft_clear_pix)
    );

    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0)
    ) soft_clear_axi_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (soft_clear_req),
        .dest_clk (axi_aclk),
        .dest_out (soft_clear_axi)
    );

    assign pix_domain_rst = pix_domain_rst_base || soft_reset_pix;
    // Keep the AXI reset request registered near the camera pipeline.
    // Capture is disabled before software asserts this held reset request;
    // an extra AXI clock keeps every AXI-side block in the same reset epoch.
    (* max_fanout = 64 *) logic soft_reset_axi_local = 1'b1;
    always_ff @(posedge axi_aclk) begin
        if (axi_domain_rst_base) soft_reset_axi_local <= 1'b1;
        else soft_reset_axi_local <= soft_reset_axi;
    end
    assign axi_domain_rst = axi_domain_rst_base || soft_reset_axi_local;

    logic        cap_enable;
    logic        cap_enable_axi;
    // CONTROL[4] gates only a new pixel-domain SOF. Keep HLS, FIFOs and
    // AXI writers enabled until software proves their complete drain.
    logic        frame_stop_req;
    wire         frame_stop_pix;
    logic        frame_stop_ack_pix;
    wire         frame_stop_ack_axil;
    wire         frame_stop_ack_axi;
    logic        use_dval_reg;
    logic [31:0] ring_base_reg;
    logic [31:0] frame_stride_reg;
    logic [31:0] consumer_idx_reg;
    logic [31:0] expected_line_bytes_reg;
    logic [31:0] expected_frame_bytes_reg;
    logic [31:0] expected_height_reg;
    logic [3:0]  isp_enable_reg;
    logic [7:0]  isp_black_level_reg;
    logic [15:0] isp_wb_gain_r_reg;
    logic [15:0] isp_wb_gain_g_reg;
    logic [15:0] isp_wb_gain_b_reg;
    logic signed [15:0] isp_ccm_00_reg;
    logic signed [15:0] isp_ccm_01_reg;
    logic signed [15:0] isp_ccm_02_reg;
    logic signed [15:0] isp_ccm_10_reg;
    logic signed [15:0] isp_ccm_11_reg;
    logic signed [15:0] isp_ccm_12_reg;
    logic signed [15:0] isp_ccm_20_reg;
    logic signed [15:0] isp_ccm_21_reg;
    logic signed [15:0] isp_ccm_22_reg;
    logic [31:0] producer_idx_reg;
    logic [31:0] frame_width_reg;
    logic [31:0] frame_height_reg;
    logic [31:0] frame_bytes_reg;
    logic [31:0] frame_seq_reg;
    logic [31:0] drop_count_reg;
    logic [31:0] irq_enable_reg;
    logic        fifo_overflow_reg;
    logic        output_fifo_fault_reg;
    logic        meta_overflow_reg;
    logic        axi_error_reg;
    logic        cfg_error_reg;
    logic        frame_size_error_reg;
    logic [31:0] drop_frame_bytes_reg;
    logic [31:0] drop_reason_reg;
    logic [31:0] duplicate_commit_count_reg;
    logic [31:0] hls_stall_cycles_reg;
    logic [31:0] axi_stall_cycles_reg;
    logic [31:0] hls_resync_count_reg;
    logic        frame_seq_valid_reg;
    logic        hls_cfg_error_reg;
    logic        hls_input_error_reg;
    logic        cmlk_align_loss_reg;
    logic [31:0] cmlk_align_loss_baseline_reg;
    logic        cfg_error_axil_reg;

    logic [31:0] producer_idx_axil;
    logic [31:0] frame_width_axil;
    logic [31:0] frame_height_axil;
    logic [31:0] frame_bytes_axil;
    logic [31:0] frame_seq_axil;
    logic [31:0] drop_count_axil;
    logic        fifo_overflow_axil;
    logic        meta_overflow_axil;
    logic        axi_error_axil;
    logic        cfg_error_axil;
    logic        frame_size_error_axil;
    logic [31:0] drop_frame_bytes_axil;
    logic [31:0] drop_reason_axil;
    logic [31:0] duplicate_commit_count_axil;
    logic [31:0] hls_stall_cycles_axil;
    logic [31:0] axi_stall_cycles_axil;
    logic [31:0] hls_resync_count_axil;
    logic [388:0] axi_status_cdc_src;
    logic [388:0] axi_status_cdc_dst;

    assign axi_status_cdc_src = {
        producer_idx_reg,
        frame_width_reg,
        frame_height_reg,
        frame_bytes_reg,
        frame_seq_reg,
        drop_count_reg,
        drop_frame_bytes_reg,
        drop_reason_reg,
        duplicate_commit_count_reg,
        hls_stall_cycles_reg,
        axi_stall_cycles_reg,
        hls_resync_count_reg,
        fifo_overflow_reg,
        meta_overflow_reg,
        axi_error_reg,
        (cfg_error_reg | hls_cfg_error_reg),
        (frame_size_error_reg | hls_input_error_reg | cmlk_align_loss_reg)
    };

    assign {
        producer_idx_axil,
        frame_width_axil,
        frame_height_axil,
        frame_bytes_axil,
        frame_seq_axil,
        drop_count_axil,
        drop_frame_bytes_axil,
        drop_reason_axil,
        duplicate_commit_count_axil,
        hls_stall_cycles_axil,
        axi_stall_cycles_axil,
        hls_resync_count_axil,
        fifo_overflow_axil,
        meta_overflow_axil,
        axi_error_axil,
        cfg_error_axil,
        frame_size_error_axil
    } = axi_status_cdc_dst;

    camera_status_snapshot_cdc #(.WIDTH(1414)) status_cdc_inst (
        .src_clk  (axi_aclk),
        .src_in   ({uip_drained_source,ddr_frame_ids_axi,axi_status_cdc_src}),
        .dest_clk (axil_aclk),
        .dest_out ({uip_drained_axil,ddr_frame_ids_axil,axi_status_cdc_dst})
    );


    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (1)
    ) link_ready_axil_cdc_inst (
        .src_clk  (pix_clk),
        .src_in   (cmlk_link_ready),
        .dest_clk (axil_aclk),
        .dest_out (cmlk_link_ready_axil)
    );

    logic [31:0] axil_write_addr;
    logic        axil_write_pending;
    logic [31:0] camctrl_rdata;
    // UIP1 window at Camera CSR + 0x200. Legacy 8-bit decoding is excluded.
    wire uip_wr_select=axil_write_addr[11:0]>=12'h200 && axil_write_addr[11:0]<12'ha00;
    wire uip_rd_select=s_axil_araddr[11:0]>=12'h200 && s_axil_araddr[11:0]<12'ha00;
    wire uip_error,uip_ready,uip_lut_we,uip_drained_axi,uip_drained_axil;
    // Drain and producer/frame IDs publish together, preventing early idle.
    wire [1:0] uip_drain_sync={2{uip_drained_axil}};
    reg uip_drained_source = 1'b0;
    always_ff @(posedge axi_aclk)
      if(axi_domain_rst) uip_drained_source <= 1'b0;
      else uip_drained_source <= uip_drained_axi;

    wire [31:0] uip_rdata,uip_generation;wire [6:0] uip_mask;
    wire [7:0] uip_lut_addr,uip_lut_data;wire [271:0] uip_config;
    uip_regs common_isp_regs (
      .clk(axil_aclk),.rst(!axil_aresetn),.idle(!cap_enable && uip_drain_sync[1]),
      .wr_en(s_axil_wready && s_axil_wvalid && uip_wr_select),
      .wr_addr(axil_write_addr[10:0]-11'h200),.wr_data(s_axil_wdata),.wr_error(uip_error),
      .rd_addr(s_axil_araddr[10:0]-11'h200),.rd_data(uip_rdata),
      .pixel_clk(axi_aclk),.pixel_rst(axi_domain_rst_base),.pixel_config(uip_config),
      .lut_we(uip_lut_we),.lut_addr(uip_lut_addr),.lut_data(uip_lut_data),
      .ready(uip_ready),.generation(uip_generation),.active_mask(uip_mask));

    // Separate online profile window; legacy stopped-capture CSRs stay protected.
    wire live_wr_select=axil_write_addr[11:8]==4'h1;
    wire live_rd_select=s_axil_araddr[11:8]==4'h1;
    wire [31:0] live_rdata;
    wire live_error, live_pending;
    wire [447:0] live_profile;
    logic live_done, live_failed;
    // This CSR slave has a full-word-only AXI-Lite interface (no WSTRB port).
    // Every accepted control write therefore carries all four valid bytes.
    camera_online_isp_mailbox live_isp_regs (
      .clk(axil_aclk),.rst(!axil_aresetn),.allowed(cap_enable && !raw_output_reg),
      .wr_en(s_axil_wready && s_axil_wvalid && live_wr_select),
      .wr_addr(axil_write_addr[7:0]),.wr_data(s_axil_wdata),.wr_strb(4'hf),
      .wr_error(live_error),.rd_addr(s_axil_araddr[7:0]),.rd_data(live_rdata),
      .pixel_clk(axi_aclk),.pixel_rst(axi_domain_rst_base),
      .pixel_allowed(cap_enable_axi && !raw_output_axi && !axi_domain_rst),
      .pending(live_pending),.profile(live_profile),.done(live_done),.failed(live_failed));

    wire         camctrl_wr_en = s_axil_wready && s_axil_wvalid && axil_write_addr[11:8]==0 &&
                                  (axil_write_addr[7:4] >= 4'h8) &&
                                  (axil_write_addr[7:4] <= 4'ha);
    wire         camctrl_rd_en = s_axil_arready && s_axil_arvalid && s_axil_araddr[11:8]==0 &&
                                  (s_axil_araddr[7:4] >= 4'h8) &&
                                  (s_axil_araddr[7:4] <= 4'ha);

    camera_control_subsystem #(
        .CLK_HZ        (AXIL_CLK_HZ),
        .UART_BAUD     (CAMERA_UART_BAUD),
        .RX_FIFO_DEPTH (512),
        .INIT_GAP_CLKS (AXIL_CLK_HZ / 20)
    ) camera_ctrl_inst (
        .clk     (axil_aclk),
        .rst     (!axil_aresetn || cmlk_reset),
        .wr_en   (camctrl_wr_en),
        .wr_addr (axil_write_addr[7:0]),
        .wr_data (s_axil_wdata),
        .rd_en   (camctrl_rd_en),
        .rd_addr (s_axil_araddr[7:0]),
        .rd_data (camctrl_rdata),
        .ser_rx  (cmlk_ser_rx),
        .ser_tx  (cmlk_ser_tx),
        .cc      (cmlk_cc)
    );

    always_ff @(posedge axil_aclk) begin
        if (!axil_aresetn) begin
            cap_enable         <= 1'b0;
            frame_stop_req     <= 1'b0;
            raw_output_reg     <= 1'b0;
            use_dval_reg       <= 1'b0;
            ring_base_reg      <= 32'd0;
            frame_stride_reg   <= 32'd0;
            consumer_idx_reg   <= 32'd0;
            expected_line_bytes_reg  <= 32'd0;
            expected_frame_bytes_reg <= 32'd0;
            expected_height_reg      <= 32'd0;
            isp_enable_reg      <= ISP_ENABLE;
            isp_black_level_reg <= ISP_BLACK_LEVEL;
            isp_wb_gain_r_reg   <= ISP_WB_GAIN_R;
            isp_wb_gain_g_reg   <= ISP_WB_GAIN_G;
            isp_wb_gain_b_reg   <= ISP_WB_GAIN_B;
            isp_ccm_00_reg      <= ISP_CCM_00;
            isp_ccm_01_reg      <= ISP_CCM_01;
            isp_ccm_02_reg      <= ISP_CCM_02;
            isp_ccm_10_reg      <= ISP_CCM_10;
            isp_ccm_11_reg      <= ISP_CCM_11;
            isp_ccm_12_reg      <= ISP_CCM_12;
            isp_ccm_20_reg      <= ISP_CCM_20;
            isp_ccm_21_reg      <= ISP_CCM_21;
            isp_ccm_22_reg      <= ISP_CCM_22;
            irq_enable_reg     <= 32'd0;
            cfg_error_axil_reg <= 1'b0;
            soft_reset_req     <= 1'b0;
            soft_clear_req     <= 1'b0;
            s_axil_awready     <= 1'b0;
            s_axil_wready      <= 1'b0;
            s_axil_bvalid      <= 1'b0;
            s_axil_bresp       <= 2'b00;
            axil_write_pending <= 1'b0;
            axil_write_addr    <= 32'd0;
        end else begin
            s_axil_awready <= !axil_write_pending && !s_axil_bvalid && s_axil_awvalid;
            s_axil_wready  <= axil_write_pending && !s_axil_bvalid && s_axil_wvalid;

            if (s_axil_awready && s_axil_awvalid) begin
                axil_write_pending <= 1'b1;
                axil_write_addr    <= s_axil_awaddr;
            end

            if (s_axil_wready && s_axil_wvalid) begin
                // Decode the complete CSR window: LISP (0x100) must never alias
                // legacy 8-bit registers while capture is enabled.
                if(axil_write_addr[11:8] == 4'h0) begin
                case (axil_write_addr[7:0])
                    REG_OUTPUT_MODE[7:0]: begin
                        if (!cap_enable && (!s_axil_wdata[0] || !(|(uip_mask & 7'h68)))) raw_output_reg <= s_axil_wdata[0];
                        else if(!cap_enable)cfg_error_axil_reg<=1'b1;
                        else if (cap_enable) cfg_error_axil_reg <= 1'b1;
                    end
                    REG_CONTROL[7:0]: begin
                        if (s_axil_wdata[1]) begin
                            cap_enable         <= 1'b0;
                            frame_stop_req     <= 1'b0;
                            use_dval_reg       <= 1'b0;
                            soft_reset_req     <= 1'b1;
                            soft_clear_req     <= 1'b0;
                            cfg_error_axil_reg <= 1'b0;
                        end else begin
                            // A running frame must survive STOP_REQUEST/clear
                            // even if an ISP mailbox is temporarily not ready.
                            // Admission validation applies only to a new start.
                            if (cap_enable && s_axil_wdata[0])
                                cap_enable <= 1'b1;
                            else
                                cap_enable <= s_axil_wdata[0] && uip_ready &&
                                              !(raw_output_reg && |(uip_mask & 7'h68));
                            if(s_axil_wdata[0] && !cap_enable &&
                               (!uip_ready || (raw_output_reg && |(uip_mask & 7'h68))))
                                cfg_error_axil_reg<=1'b1;
                            frame_stop_req <= s_axil_wdata[4] && s_axil_wdata[0];
                            use_dval_reg   <= s_axil_wdata[3];
                            soft_reset_req <= 1'b0;
                            soft_clear_req <= s_axil_wdata[2];
                            if (s_axil_wdata[2])
                                cfg_error_axil_reg <= 1'b0;
                        end
                    end
                    REG_IRQ_ENABLE[7:0]: irq_enable_reg <= s_axil_wdata;
                    REG_RING_BASE_LO[7:0]: begin
                        if (!cap_enable) ring_base_reg <= s_axil_wdata;
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_RING_BASE_HI[7:0]: begin end
                    REG_FRAME_STRIDE[7:0]: begin
                        if (!cap_enable) frame_stride_reg <= s_axil_wdata;
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_RING_SLOTS[7:0]: begin
                        if (s_axil_wdata != CAMERA_RING_SLOTS_U32)
                            cfg_error_axil_reg <= 1'b1;
                    end
                    REG_CONSUMER_IDX[7:0]: consumer_idx_reg <= s_axil_wdata;
                    REG_EXPECTED_LINE_BYTES[7:0]: begin
                        if (!cap_enable) expected_line_bytes_reg <= s_axil_wdata;
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_EXPECTED_FRAME_BYTES[7:0]: begin
                        if (!cap_enable) expected_frame_bytes_reg <= s_axil_wdata;
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_EXPECTED_HEIGHT[7:0]: begin
                        if (!cap_enable) expected_height_reg <= s_axil_wdata;
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_ENABLE[7:0]: begin
                        if (!cap_enable)
                            isp_enable_reg <= s_axil_wdata[3:0];
                        else
                            cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_BLACK_LEVEL[7:0]: begin
                        if (!cap_enable)
                            isp_black_level_reg <= s_axil_wdata[7:0];
                        else
                            cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_WB_GAIN_R[7:0]: begin
                        if (!cap_enable) isp_wb_gain_r_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_WB_GAIN_G[7:0]: begin
                        if (!cap_enable) isp_wb_gain_g_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_WB_GAIN_B[7:0]: begin
                        if (!cap_enable) isp_wb_gain_b_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_00[7:0]: begin
                        if (!cap_enable) isp_ccm_00_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_01[7:0]: begin
                        if (!cap_enable) isp_ccm_01_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_02[7:0]: begin
                        if (!cap_enable) isp_ccm_02_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_10[7:0]: begin
                        if (!cap_enable) isp_ccm_10_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_11[7:0]: begin
                        if (!cap_enable) isp_ccm_11_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_12[7:0]: begin
                        if (!cap_enable) isp_ccm_12_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_20[7:0]: begin
                        if (!cap_enable) isp_ccm_20_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_21[7:0]: begin
                        if (!cap_enable) isp_ccm_21_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    REG_ISP_CCM_22[7:0]: begin
                        if (!cap_enable) isp_ccm_22_reg <= s_axil_wdata[15:0];
                        else cfg_error_axil_reg <= 1'b1;
                    end
                    default: begin end
                endcase
                end
                s_axil_bvalid      <= 1'b1;
                s_axil_bresp       <= ((uip_wr_select && uip_error) || (live_wr_select && live_error)) ? 2'b10:2'b00;
                axil_write_pending <= 1'b0;
            end

            if (s_axil_bvalid && s_axil_bready)
                s_axil_bvalid <= 1'b0;
        end
    end

    wire [36:0] axil_ring_end_ext = {5'd0, ring_base_reg} +
                                            ({5'd0, frame_stride_reg} << 5);
    logic [15:0] axil_expected_line_d1, axil_expected_height_d1;
    logic [15:0] axil_expected_line_d2, axil_expected_height_d2;
    (* use_dsp = "yes" *) logic [31:0] axil_expected_raw_mreg;
    (* use_dsp = "yes" *) logic [31:0] axil_expected_raw_preg;
    logic [1:0] axil_expected_raw_valid;
    always_ff @(posedge axil_aclk) begin
        if (!axil_aresetn) begin
            axil_expected_line_d1 <= 0; axil_expected_height_d1 <= 0;
            axil_expected_line_d2 <= 0; axil_expected_height_d2 <= 0;
            axil_expected_raw_mreg <= 0; axil_expected_raw_preg <= 0;
            axil_expected_raw_valid <= 0;
        end else begin
            axil_expected_line_d1 <= expected_line_bytes_reg[15:0];
            axil_expected_height_d1 <= expected_height_reg[15:0];
            axil_expected_line_d2 <= axil_expected_line_d1;
            axil_expected_height_d2 <= axil_expected_height_d1;
            axil_expected_raw_mreg <= expected_line_bytes_reg[15:0] * expected_height_reg[15:0];
            axil_expected_raw_preg <= axil_expected_raw_mreg;
            axil_expected_raw_valid <= {axil_expected_raw_valid[0], 1'b1};
        end
    end
    wire axil_geometry_product_valid = axil_expected_raw_valid[1] &&
        expected_line_bytes_reg[15:0] == axil_expected_line_d2 &&
        expected_height_reg[15:0] == axil_expected_height_d2;
    wire [33:0] axil_expected_rgb_ext =
                    {2'd0, expected_frame_bytes_reg} +
                    ({2'd0, expected_frame_bytes_reg} << 1);
    wire        cfg_geometry_axil_ok = axil_geometry_product_valid &&
                    (expected_line_bytes_reg != 32'd0) &&
                    (expected_height_reg != 32'd0) &&
                    (expected_frame_bytes_reg != 32'd0) &&
                    (expected_line_bytes_reg <= 32'd4096) &&
                    (expected_height_reg <= 32'd4096) &&
                    (expected_line_bytes_reg[2:0] == 3'd0) &&
                    (expected_frame_bytes_reg[2:0] == 3'd0) &&
                    (expected_frame_bytes_reg == axil_expected_raw_preg);
    wire        cfg_memory_axil_ok =
                    (frame_stride_reg != 32'd0) &&
                    (ring_base_reg[5:0] == 6'd0) &&
                    (frame_stride_reg[5:0] == 6'd0) &&
                    (axil_ring_end_ext[36:32] == 5'd0) &&
                    (axil_expected_rgb_ext[33:32] == 2'd0) &&
                    (frame_stride_reg >= axil_expected_rgb_ext[31:0]);
    // DDR address zero is valid.  Width/height may be changed between runs,
    // but capture becomes ready only after a complete, self-consistent geometry
    // and RGB frame stride have been programmed.
    wire        cfg_ready_axil_local = cap_enable &&
                                       cfg_geometry_axil_ok &&
                                       cfg_memory_axil_ok;
    logic [191:0] diag_cdc_dst;
    wire [31:0] diag_fval_edges_axil      = diag_cdc_dst[191 -: 32];
    wire [31:0] diag_lval_edges_axil      = diag_cdc_dst[159 -: 32];
    wire [31:0] diag_dval_edges_axil      = diag_cdc_dst[127 -: 32];
    wire [31:0] diag_pix_valid_words_axil = diag_cdc_dst[95  -: 32];
    wire [31:0] diag_bad_frames_axil      = diag_cdc_dst[63  -: 32];
    wire [31:0] diag_last_line_bytes_axil = diag_cdc_dst[31  -: 32];

    logic fval_seen_axil;
    logic lval_seen_axil;
    logic dval_seen_axil;

    // Keep the interrupt source off the high-throughput AXI/CameraLink paths:
    // it depends only on already-synchronized AXI-Lite-domain ring indices.
    // A level request is intentional.  It cannot be lost across the 125 MHz to
    // 250 MHz CDC, and it deasserts after software advances consumer_idx.
    reg camera_frame_irq_reg = 1'b0;
    assign camera_frame_irq = camera_frame_irq_reg;
    always_ff @(posedge axil_aclk) begin
        if (!axil_aresetn) camera_frame_irq_reg <= 1'b0;
        else camera_frame_irq_reg <= irq_enable_reg[0] &&
                                     (producer_idx_axil != consumer_idx_reg);
    end

    always_ff @(posedge axil_aclk) begin
        if (!axil_aresetn) begin
            s_axil_arready <= 1'b0;
            s_axil_rvalid  <= 1'b0;
            s_axil_rdata   <= 32'd0;
            s_axil_rresp   <= 2'b00;
        end else begin
            s_axil_arready <= !s_axil_rvalid && s_axil_arvalid;
            if (s_axil_arready && s_axil_arvalid) begin
                s_axil_rvalid <= 1'b1;
                s_axil_rresp  <= 2'b00;
                if(host_read_select)s_axil_rdata<=host_csr_rdata;
                else if(s_axil_araddr[11:0]>=12'hc00 && s_axil_araddr[11:0]<12'hc4c)
                    s_axil_rdata<=drop_hist_axil[((s_axil_araddr[11:0]-12'hc00)>>2)*32 +:32];
                else if(s_axil_araddr[11:0]>=12'hd00 && s_axil_araddr[11:0]<12'hd80)
                    s_axil_rdata<=ddr_frame_ids_axil[((s_axil_araddr[11:0]-12'hd00)>>2)*32 +:32];
                else if(live_rd_select) s_axil_rdata<=live_rdata;
                else if(uip_rd_select) s_axil_rdata<=uip_rdata;
                else if (s_axil_araddr[11:8]==0 && (s_axil_araddr[7:4] >= 4'h8) && (s_axil_araddr[7:4] <= 4'ha)) begin
                    s_axil_rdata <= camctrl_rdata;
                end else begin
                case (s_axil_araddr[7:0])
                    REG_OUTPUT_MODE[7:0]: s_axil_rdata <= 32'h42590000 | {31'd0,raw_output_reg};
                    REG_CONTROL[7:0]:      s_axil_rdata <= {26'd0, frame_stop_ack_axil, frame_stop_req, use_dval_reg, 1'b0, soft_reset_req, cap_enable};
                    REG_STATUS[7:0]:       s_axil_rdata <= {19'd0, frame_size_error_axil, dval_seen_axil, lval_seen_axil, fval_seen_axil, cfg_ready_axil_local, (frame_bytes_axil != 32'd0), cmlk_link_ready_axil, (cfg_error_axil | cfg_error_axil_reg), axi_error_axil, meta_overflow_axil, fifo_overflow_axil, 2'd0};
                    REG_IRQ_STATUS[7:0]:   s_axil_rdata <= {31'd0, (producer_idx_axil != consumer_idx_reg)};
                    REG_IRQ_ENABLE[7:0]:   s_axil_rdata <= irq_enable_reg;
                    REG_RING_BASE_LO[7:0]: s_axil_rdata <= ring_base_reg;
                    REG_RING_BASE_HI[7:0]: s_axil_rdata <= 32'd0;
                    REG_FRAME_STRIDE[7:0]: s_axil_rdata <= frame_stride_reg;
                    REG_RING_SLOTS[7:0]:   s_axil_rdata <= CAMERA_RING_SLOTS_U32;
                    REG_CONSUMER_IDX[7:0]: s_axil_rdata <= consumer_idx_reg;
                    REG_PRODUCER_IDX[7:0]: s_axil_rdata <= producer_idx_axil;
                    REG_WIDTH[7:0]:        s_axil_rdata <= frame_width_axil;
                    REG_HEIGHT[7:0]:       s_axil_rdata <= frame_height_axil;
                    REG_FRAME_BYTES[7:0]:  s_axil_rdata <= frame_bytes_axil;
                    REG_FRAME_SEQ[7:0]:    s_axil_rdata <= frame_seq_axil;
                    REG_DROP_COUNT[7:0]:   s_axil_rdata <= drop_count_axil;
                    REG_DIAG_FVAL_EDGES[7:0]:      s_axil_rdata <= diag_fval_edges_axil;
                    REG_DIAG_LVAL_EDGES[7:0]:      s_axil_rdata <= diag_lval_edges_axil;
                    REG_DIAG_DVAL_EDGES[7:0]:      s_axil_rdata <= diag_dval_edges_axil;
                    REG_DIAG_PIX_VALID_WORDS[7:0]: s_axil_rdata <= diag_pix_valid_words_axil;
                    REG_DIAG_BAD_FRAMES[7:0]:      s_axil_rdata <= diag_bad_frames_axil;
                    REG_DIAG_LAST_LINE_BYTES[7:0]: s_axil_rdata <= diag_last_line_bytes_axil;
                    REG_EXPECTED_LINE_BYTES[7:0]:  s_axil_rdata <= expected_line_bytes_reg;
                    REG_EXPECTED_FRAME_BYTES[7:0]: s_axil_rdata <= expected_frame_bytes_reg;
                    REG_DIAG_DROP_FRAME_BYTES[7:0]: s_axil_rdata <= drop_frame_bytes_axil;
                    REG_DIAG_DROP_REASON[7:0]:      s_axil_rdata <= drop_reason_axil;
                    REG_EXPECTED_HEIGHT[7:0]:         s_axil_rdata <= expected_height_reg;
                    REG_BUILD_ID[7:0]:                s_axil_rdata <= CAMERA_BUILD_ID;
                    REG_BUILD_VERSION[7:0]:           s_axil_rdata <= CAMERA_BUILD_VERSION;
                    REG_DUP_COMMIT_COUNT[7:0]:        s_axil_rdata <= duplicate_commit_count_axil;
                    REG_HLS_STALL_CYCLES[7:0]:        s_axil_rdata <= hls_stall_cycles_axil;
                    REG_AXI_STALL_CYCLES[7:0]:        s_axil_rdata <= axi_stall_cycles_axil;
                    REG_CMLK_SYNC_MISMATCH_CYCLES[7:0]: s_axil_rdata <= diag_cmlk_sync_mismatch_cycles_axil;
                    REG_CMLK_SYNC_MISMATCH_EVENTS[7:0]: s_axil_rdata <= diag_cmlk_sync_mismatch_events_axil;
                    REG_CMLK_SYNC_MISMATCH_MASK[7:0]:   s_axil_rdata <= diag_cmlk_sync_mismatch_mask_axil;
                    REG_CMLK_ALIGN_LOSS_COUNT[7:0]:     s_axil_rdata <= cmlk_align_loss_count_axil;
                    REG_HLS_RESYNC_COUNT[7:0]:          s_axil_rdata <= hls_resync_count_axil;
                    REG_ISP_ENABLE[7:0]:                s_axil_rdata <= {28'd0, isp_enable_reg};
                    REG_ISP_BLACK_LEVEL[7:0]:           s_axil_rdata <= {24'd0, isp_black_level_reg};
                    REG_ISP_WB_GAIN_R[7:0]:             s_axil_rdata <= {16'd0, isp_wb_gain_r_reg};
                    REG_ISP_WB_GAIN_G[7:0]:             s_axil_rdata <= {16'd0, isp_wb_gain_g_reg};
                    REG_ISP_WB_GAIN_B[7:0]:             s_axil_rdata <= {16'd0, isp_wb_gain_b_reg};
                    REG_ISP_CCM_00[7:0]:                s_axil_rdata <= {{16{isp_ccm_00_reg[15]}}, isp_ccm_00_reg};
                    REG_ISP_CCM_01[7:0]:                s_axil_rdata <= {{16{isp_ccm_01_reg[15]}}, isp_ccm_01_reg};
                    REG_ISP_CCM_02[7:0]:                s_axil_rdata <= {{16{isp_ccm_02_reg[15]}}, isp_ccm_02_reg};
                    REG_ISP_CCM_10[7:0]:                s_axil_rdata <= {{16{isp_ccm_10_reg[15]}}, isp_ccm_10_reg};
                    REG_ISP_CCM_11[7:0]:                s_axil_rdata <= {{16{isp_ccm_11_reg[15]}}, isp_ccm_11_reg};
                    REG_ISP_CCM_12[7:0]:                s_axil_rdata <= {{16{isp_ccm_12_reg[15]}}, isp_ccm_12_reg};
                    REG_ISP_CCM_20[7:0]:                s_axil_rdata <= {{16{isp_ccm_20_reg[15]}}, isp_ccm_20_reg};
                    REG_ISP_CCM_21[7:0]:                s_axil_rdata <= {{16{isp_ccm_21_reg[15]}}, isp_ccm_21_reg};
                    REG_ISP_CCM_22[7:0]:                s_axil_rdata <= {{16{isp_ccm_22_reg[15]}}, isp_ccm_22_reg};
                    default:               s_axil_rdata <= 32'd0;
                endcase
                end
            end
            if (s_axil_rvalid && s_axil_rready)
                s_axil_rvalid <= 1'b0;
        end
    end

    logic        pix_f_d;
    logic        pix_l_d;
    logic        pix_valid;
    logic        pix_valid_d;
    logic [79:0] pix_data_d;
    logic [9:0]  pix_keep_d;
    logic [3:0]  pix_word_bytes_d;
    logic        cap_enable_pix;
    logic [31:0] expected_line_bytes_pix;
    logic        use_dval_pix;
    logic        pix_f_seen;
    logic        pix_l_seen;
    logic        pix_d_seen;
    logic        pix_d_d;
    logic [2:0]  pix_seen_cdc_dst;
    logic        first_word;
    logic        pix_capture_armed;
    logic        pix_frame_active;
    logic        pix_frame_sync_error;
    logic        pix_frame_fifo_error;
    logic [31:0] line_bytes;
    logic [31:0] max_width;
    logic [31:0] height_cnt;
    logic [31:0] byte_cnt;
    logic [31:0] pix_frame_seq;
    logic [90:0] data_fifo_din;
    logic        data_fifo_wr_en;
    logic        data_fifo_full;
    wire         data_fifo_overflow_event_pix;
    logic        data_fifo_overflow_latched_pix;
    logic        data_fifo_overflow_axi;
    logic [127:0] meta_fifo_din;
    logic        meta_fifo_wr_en;
    logic        meta_fifo_full;
    wire         meta_fifo_overflow_event_pix;
    logic        meta_fifo_overflow_latched_pix;
    logic        meta_fifo_overflow_axi;
    logic [31:0] diag_fval_edges_pix;
    logic [31:0] diag_lval_edges_pix;
    logic [31:0] diag_dval_edges_pix;
    logic [31:0] diag_pix_valid_words_pix;
    logic [31:0] diag_bad_frames_pix;
    logic [31:0] diag_last_line_bytes_pix;
    logic [31:0] diag_cmlk_sync_mismatch_cycles_pix;
    logic [31:0] diag_cmlk_sync_mismatch_events_pix;
    logic [2:0]  diag_cmlk_sync_mismatch_mask_pix;
    logic        cmlk_sync_mismatch_d_pix;
     wire [2:0] cmlk_sync_mismatch_bits =
        (cmlk_sync_x ^ cmlk_sync_y) | (cmlk_sync_x ^ cmlk_sync_z);
     wire cmlk_sync_mismatch =
        cmlk_link_ready && (|cmlk_sync_mismatch_bits);
    wire [191:0] diag_cdc_src = {
        diag_fval_edges_pix,
        diag_lval_edges_pix,
        diag_dval_edges_pix,
        diag_pix_valid_words_pix,
        diag_bad_frames_pix,
        diag_last_line_bytes_pix
    };

    camera_status_snapshot_cdc #(.WIDTH(192)) diag_status_cdc_inst (
        .src_clk  (pix_clk),
        .src_in   (diag_cdc_src),
        .dest_clk (axil_aclk),
        .dest_out (diag_cdc_dst)
    );

    wire [95:0] cmlk_diag_cdc_src = {
        diag_cmlk_sync_mismatch_cycles_pix,
        diag_cmlk_sync_mismatch_events_pix,
        {29'd0, diag_cmlk_sync_mismatch_mask_pix}
    };

    camera_status_snapshot_cdc #(.WIDTH(96)) cmlk_diag_status_cdc_inst (
        .src_clk  (pix_clk),
        .src_in   (cmlk_diag_cdc_src),
        .dest_clk (axil_aclk),
        .dest_out (cmlk_diag_cdc_dst)
    );

    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0)
    ) cap_enable_pix_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (cap_enable),
        .dest_clk (pix_clk),
        .dest_out (cap_enable_pix)
    );

    // One-bit vendor CDCs; no clock change, no wide data/READY reset path.
    xpm_cdc_single #(
        .DEST_SYNC_FF(2), .INIT_SYNC_FF(0),
        .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0)
    ) frame_stop_request_cdc_inst (
        .src_clk(axil_aclk), .src_in(frame_stop_req),
        .dest_clk(pix_clk), .dest_out(frame_stop_pix)
    );
    xpm_cdc_single #(
        .DEST_SYNC_FF(2), .INIT_SYNC_FF(0),
        .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0)
    ) frame_stop_ack_cdc_inst (
        .src_clk(pix_clk), .src_in(frame_stop_ack_pix),
        .dest_clk(axil_aclk), .dest_out(frame_stop_ack_axil)
    );
    xpm_cdc_single #(.DEST_SYNC_FF(2),.INIT_SYNC_FF(0),.SIM_ASSERT_CHK(0),.SRC_INPUT_REG(0)) frame_stop_ack_axi_cdc_inst (
        .src_clk(pix_clk),.src_in(frame_stop_ack_pix),
        .dest_clk(axi_aclk),.dest_out(frame_stop_ack_axi));

    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0)
    ) use_dval_pix_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (use_dval_reg),
        .dest_clk (pix_clk),
        .dest_out (use_dval_pix)
    );

    xpm_cdc_array_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0),
        .WIDTH          (32)
    ) expected_line_bytes_pix_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (expected_line_bytes_reg),
        .dest_clk (pix_clk),
        .dest_out (expected_line_bytes_pix)
    );

    xpm_cdc_array_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0),
        .WIDTH          (3)
    ) pix_seen_status_cdc_inst (
        .src_clk  (pix_clk),
        .src_in   ({pix_d_seen, pix_l_seen, pix_f_seen}),
        .dest_clk (axil_aclk),
        .dest_out (pix_seen_cdc_dst)
    );

    assign {dval_seen_axil, lval_seen_axil, fval_seen_axil} = pix_seen_cdc_dst;

    assign pix_valid = f_val && l_val && (!use_dval_pix || d_val);

    wire pix_f_rise = !pix_f_d && f_val;
    wire pix_f_fall = pix_f_d && !f_val;
    wire pix_begin_frame = cap_enable_pix && pix_capture_armed &&
                           pix_f_rise && !frame_stop_pix;
    // ACK is input admission quiescence, not permission to free DMA.
    // Software must additionally wait for HOST_STATUS.IDLE and DDR ownership.
    wire pix_count_active = pix_frame_active || pix_begin_frame;

    function automatic [9:0] keep10_from_count(input logic [3:0] count);
        begin
            case (count)
                4'd1: keep10_from_count = 10'b0000000001;
                4'd2: keep10_from_count = 10'b0000000011;
                4'd3: keep10_from_count = 10'b0000000111;
                4'd4: keep10_from_count = 10'b0000001111;
                4'd5: keep10_from_count = 10'b0000011111;
                4'd6: keep10_from_count = 10'b0000111111;
                4'd7: keep10_from_count = 10'b0001111111;
                4'd8: keep10_from_count = 10'b0011111111;
                4'd9: keep10_from_count = 10'b0111111111;
                4'd10: keep10_from_count = 10'b1111111111;
                default: keep10_from_count = 10'b0000000000;
            endcase
        end
    endfunction

    wire [31:0] pix_line_remaining =
        (expected_line_bytes_pix > line_bytes) ?
        (expected_line_bytes_pix - line_bytes) : 32'd0;
    wire [3:0] pix_word_bytes_now = !pix_valid ? 4'd0 :
        ((pix_line_remaining >= 32'd10) ? 4'd10 :
         pix_line_remaining[3:0]);

    wire [31:0] pix_frame_width_now = (line_bytes > max_width) ? line_bytes : max_width;
    wire [31:0] pix_frame_height_now = height_cnt + ((line_bytes != 0) ? 32'd1 : 32'd0);
    wire        pix_frame_has_payload = (byte_cnt != 32'd0) &&
                                        (pix_frame_width_now != 32'd0) &&
                                        (pix_frame_height_now != 32'd0);

    // The pixel sample is already registered. Enqueue it on the same edge
    // that tests FULL, so a queued WR_EN cannot lose SOF one cycle later.
    // FULL and all operands here belong to pix_clk; no new AXI READY path.
    wire pix_fifo_word = pix_valid_d && cap_enable_pix && pix_frame_active &&
                         (pix_word_bytes_d != 0) && !pix_domain_rst;
    wire pix_fifo_loss = pix_fifo_word && data_fifo_full;
    wire pix_frame_fifo_bad = pix_frame_fifo_error || pix_fifo_loss ||
                              data_fifo_overflow_event_pix;
    assign data_fifo_din = {first_word, pix_keep_d, pix_data_d};
    assign data_fifo_wr_en = pix_fifo_word && !data_fifo_full;

    always_ff @(posedge pix_clk) begin
        if (pix_domain_rst) frame_stop_ack_pix <= 1'b0;
        else frame_stop_ack_pix <= frame_stop_pix && !pix_frame_active &&
                                   !pix_fifo_word;
    end

    always_ff @(posedge pix_clk) begin
        if (pix_domain_rst) begin
            pix_f_d         <= 1'b0;
            pix_l_d         <= 1'b0;
            pix_d_d         <= 1'b0;
            pix_valid_d     <= 1'b0;
            pix_data_d      <= 80'd0;
            pix_keep_d      <= 10'd0;
            pix_word_bytes_d <= 4'd0;
            pix_f_seen      <= 1'b0;
            pix_l_seen      <= 1'b0;
            pix_d_seen      <= 1'b0;
            first_word      <= 1'b0;
            pix_capture_armed <= 1'b0;
            pix_frame_active  <= 1'b0;
            pix_frame_sync_error <= 1'b0;
            pix_frame_fifo_error <= 1'b0;
            line_bytes      <= 32'd0;
            max_width       <= 32'd0;
            height_cnt      <= 32'd0;
            byte_cnt        <= 32'd0;
            pix_frame_seq   <= 32'd0;
            meta_fifo_wr_en <= 1'b0;
            data_fifo_overflow_latched_pix <= 1'b0;
            meta_fifo_overflow_latched_pix <= 1'b0;
            diag_fval_edges_pix      <= 32'd0;
            diag_lval_edges_pix      <= 32'd0;
            diag_dval_edges_pix      <= 32'd0;
            diag_pix_valid_words_pix <= 32'd0;
            diag_bad_frames_pix      <= 32'd0;
            diag_last_line_bytes_pix <= 32'd0;
            diag_cmlk_sync_mismatch_cycles_pix <= 32'd0;
            diag_cmlk_sync_mismatch_events_pix <= 32'd0;
            diag_cmlk_sync_mismatch_mask_pix   <= 3'd0;
            cmlk_sync_mismatch_d_pix           <= 1'b0;
        end else begin
            pix_f_d         <= f_val;
            pix_l_d         <= l_val;
            pix_d_d         <= d_val;
            pix_valid_d      <= pix_valid;
            pix_data_d       <= {port_j, port_i, port_h, port_g, port_f,
                                 port_e, port_d, port_c, port_b, port_a};
            pix_keep_d       <= keep10_from_count(pix_word_bytes_now);
            pix_word_bytes_d <= pix_word_bytes_now;
            cmlk_sync_mismatch_d_pix <= cmlk_sync_mismatch;
            if (soft_clear_pix) begin
                pix_f_seen <= f_val;
                pix_l_seen <= l_val;
                pix_d_seen <= d_val;
                diag_fval_edges_pix      <= 32'd0;
                diag_lval_edges_pix      <= 32'd0;
                diag_dval_edges_pix      <= 32'd0;
                diag_pix_valid_words_pix <= 32'd0;
                diag_bad_frames_pix      <= 32'd0;
                diag_last_line_bytes_pix <= 32'd0;
                diag_cmlk_sync_mismatch_cycles_pix <= 32'd0;
                diag_cmlk_sync_mismatch_events_pix <= 32'd0;
                diag_cmlk_sync_mismatch_mask_pix   <= 3'd0;
                cmlk_sync_mismatch_d_pix           <= 1'b0;
            end else begin
                if (f_val)
                    pix_f_seen <= 1'b1;
                if (l_val)
                    pix_l_seen <= 1'b1;
                if (d_val)
                    pix_d_seen <= 1'b1;
                if (cmlk_sync_mismatch) begin
                    diag_cmlk_sync_mismatch_cycles_pix <=
                        diag_cmlk_sync_mismatch_cycles_pix + 1'b1;
                    diag_cmlk_sync_mismatch_mask_pix <=
                        diag_cmlk_sync_mismatch_mask_pix | cmlk_sync_mismatch_bits;
                    if (!cmlk_sync_mismatch_d_pix)
                        diag_cmlk_sync_mismatch_events_pix <=
                            diag_cmlk_sync_mismatch_events_pix + 1'b1;
                    if (pix_frame_active)
                        pix_frame_sync_error <= 1'b1;
                end
            end
            meta_fifo_wr_en <= 1'b0;

            /*
             * Reset deassertion while FVAL is high is not a real frame edge.
             * Capture may be enabled during a camera frame or immediately
             * after link retraining. Arm only after observing the inter-frame
             * low level, then accept the next genuine FVAL rising edge.
             */
            if (!cap_enable_pix) begin
                pix_capture_armed <= 1'b0;
                pix_frame_active  <= 1'b0;
                pix_frame_sync_error <= 1'b0;
                pix_frame_fifo_error <= 1'b0;
                first_word        <= 1'b0;
                line_bytes        <= 32'd0;
                max_width         <= 32'd0;
                height_cnt        <= 32'd0;
                byte_cnt          <= 32'd0;
            end else if (!pix_capture_armed && !pix_frame_active) begin
                pix_frame_active <= 1'b0;
                pix_frame_sync_error <= 1'b0;
                pix_frame_fifo_error <= 1'b0;
                first_word       <= 1'b0;
                if (!f_val && !pix_f_d)
                    pix_capture_armed <= 1'b1;
            end

            if (soft_clear_pix) begin
                data_fifo_overflow_latched_pix <= 1'b0;
                meta_fifo_overflow_latched_pix <= 1'b0;
            end else begin
                // Retain unexpected XPM write errors as diagnostics. Normal
                // enqueue tests FULL on the FIFO's own write edge.
                if (data_fifo_overflow_event_pix)
                    data_fifo_overflow_latched_pix <= 1'b1;
                if (meta_fifo_overflow_event_pix)
                    meta_fifo_overflow_latched_pix <= 1'b1;
            end

            if (pix_f_rise) begin
                diag_fval_edges_pix <= diag_fval_edges_pix + 1'b1;
                if (pix_begin_frame) begin
                    // Consume this arm token. A subsequent frame is accepted
                    // only after at least one complete FVAL-low clock.
                    pix_capture_armed <= 1'b0;
                    pix_frame_active <= 1'b1;
                    pix_frame_sync_error <= cmlk_sync_mismatch;
                    pix_frame_fifo_error <= 1'b0;
                    first_word       <= 1'b1;
                    line_bytes       <= 32'd0;
                    max_width        <= 32'd0;
                    height_cnt       <= 32'd0;
                    byte_cnt         <= 32'd0;
                    pix_frame_seq    <= pix_frame_seq + 1'b1;
                end
            end

            /* FIFO full is only backpressure information.  It becomes an
             * overflow error only when an active camera word actually arrives
             * while no FIFO entry is available.  The previous implementation
             * latched the full level itself and therefore reported false data
             * loss during harmless transient occupancy peaks.
             */
            if (data_fifo_wr_en)
                first_word <= 1'b0;
            if (pix_fifo_loss || data_fifo_overflow_event_pix) begin
                pix_frame_fifo_error <= 1'b1;
                data_fifo_overflow_latched_pix <= 1'b1;
            end

            if (!pix_d_d && d_val) begin
                diag_dval_edges_pix <= diag_dval_edges_pix + 1'b1;
            end

            if (pix_valid) begin
                diag_pix_valid_words_pix <= diag_pix_valid_words_pix + 1'b1;
                if (pix_count_active && pix_word_bytes_now != 0) begin
                    line_bytes <= line_bytes + pix_word_bytes_now;
                    byte_cnt   <= byte_cnt + pix_word_bytes_now;
                end
            end

            if (pix_l_d && !l_val && pix_f_d) begin
                diag_lval_edges_pix <= diag_lval_edges_pix + 1'b1;
                if (pix_frame_active) begin
                    diag_last_line_bytes_pix <= line_bytes;
                    if (line_bytes > max_width)
                        max_width <= line_bytes;
                    if (line_bytes != 0)
                        height_cnt <= height_cnt + 1'b1;
                    line_bytes <= 32'd0;
                end
            end

            if (pix_f_fall) begin
                if (cap_enable_pix && pix_frame_active) begin
                    if (!pix_frame_has_payload || pix_frame_sync_error ||
                        cmlk_sync_mismatch || pix_frame_fifo_bad)
                        diag_bad_frames_pix <= diag_bad_frames_pix + 1'b1;
                    // Preserve FIFO/HLS frame pairing, but force the metadata
                    // validator to reject a frame whose replicated LVAL bits
                    // disagreed. The following physical SOF remains aligned.
                    meta_fifo_din <= {
                        pix_frame_seq,
                        pix_frame_width_now,
                        pix_frame_height_now,
                        {pix_frame_sync_error || cmlk_sync_mismatch,
                         pix_frame_fifo_bad, byte_cnt[29:0]}
                    };
                    // No accepted SOF means no matching HLS invocation. Do not
                    // enqueue orphan metadata for a wholly lost physical frame.
                    if (pix_frame_has_payload && (!first_word || data_fifo_wr_en)) begin
                        if (!meta_fifo_full) begin
                            meta_fifo_wr_en <= 1'b1;
                        end else begin
                            meta_fifo_overflow_latched_pix <= 1'b1;
                        end
                    end
                end
                pix_frame_active <= 1'b0;
                first_word       <= 1'b0;
            end
        end
    end

    logic [90:0] camera_fifo_dout;
    logic        camera_fifo_empty;
    logic        camera_fifo_valid;
    logic        camera_fifo_rd_en;
    wire [90:0]  camera_fifo_raw_dout;
    wire         camera_fifo_raw_valid, camera_fifo_raw_rd_en;
    wire         camera_fifo_read_busy;
    wire [90:0]  camera_cdc_axis_data;
    wire         camera_cdc_axis_valid, camera_cdc_axis_ready;
    wire         camera_cdc_prefetch_pending;
    wire         camera_fifo_prefetch_ready;
    wire  [64:0] data_fifo_dout;
    wire         data_fifo_valid;
    logic        data_fifo_rd_en;
    logic [127:0] meta_fifo_dout;
    logic        meta_fifo_empty;
    logic        meta_fifo_valid;
    logic        meta_fifo_rd_en;

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE    ("block"),
        // Standard reads permit RAM output pipelining. Bound cascade depth
        // instead of routing many unregistered BRAMs into one output stage.
        .CASCADE_HEIGHT      (1),
        .FIFO_WRITE_DEPTH    (FIFO_DEPTH),
        .WRITE_DATA_WIDTH    (91),
        .READ_DATA_WIDTH     (91),
        .READ_MODE           ("std"),
        .FIFO_READ_LATENCY   (3),
        .CDC_SYNC_STAGES     (2),
        .DOUT_RESET_VALUE    ("0"),
        .ECC_MODE            ("no_ecc"),
        .RELATED_CLOCKS      (0),
        .USE_ADV_FEATURES    ("1100")
    ) data_fifo_inst (
        .rst          (pix_domain_rst),
        .wr_clk       (pix_clk),
        .rd_clk       (axi_aclk),
        .din          (data_fifo_din),
        .wr_en        (data_fifo_wr_en),
        .rd_en        (camera_fifo_raw_rd_en),
        .dout         (camera_fifo_raw_dout),
        .full         (data_fifo_full),
        .empty        (camera_fifo_empty),
        .data_valid   (camera_fifo_raw_valid),
        .overflow     (data_fifo_overflow_event_pix),
        .underflow    (),
        .wr_rst_busy  (),
        .rd_rst_busy  (camera_fifo_read_busy),
        .sleep        (1'b0),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .wr_data_count(),
        .rd_data_count(),
        .prog_full    (),
        .prog_empty   (),
        .almost_full  (),
        .almost_empty (),
        .wr_ack       ()
    );

    /*
     * 80-to-64 byte gearbox in the 250 MHz AXI/HLS domain. It deliberately
     * alternates between one fixed-position append and one fixed 64-bit pop;
     * four ten-byte inputs need nine AXI clocks (111.1 M input words/s), above
     * the Camera Link maximum of 85 M words/s. No variable barrel shifter is
     * inferred on the HLS-ready path.
     */
    function automatic [3:0] count_keep10(input logic [9:0] keep);
        begin
            // The pixel domain emits contiguous low-bit masks only. Decode
            // them directly so the 250 MHz FIFO-to-gearbox path contains no
            // ten-input population-count adder chain.
            case (keep)
                10'b0000000000: count_keep10 = 4'd0;
                10'b0000000001: count_keep10 = 4'd1;
                10'b0000000011: count_keep10 = 4'd2;
                10'b0000000111: count_keep10 = 4'd3;
                10'b0000001111: count_keep10 = 4'd4;
                10'b0000011111: count_keep10 = 4'd5;
                10'b0000111111: count_keep10 = 4'd6;
                10'b0001111111: count_keep10 = 4'd7;
                10'b0011111111: count_keep10 = 4'd8;
                10'b0111111111: count_keep10 = 4'd9;
                10'b1111111111: count_keep10 = 4'd10;
                default:        count_keep10 = 4'd0;
            endcase
        end
    endfunction

    // The small read adapter reserves all XPM standard-read responses before
    // issuing them. Its output is a one-word/cycle AXI-clock stream.
    camera_input_fifo_read_adapter #(.DATA_WIDTH(91),.DEPTH(8))
    camera_cdc_prefetch (
        .clk(axi_aclk),.rst(axi_domain_rst),
        .fifo_busy(camera_fifo_read_busy),.fifo_empty(camera_fifo_empty),
        .fifo_rd_en(camera_fifo_raw_rd_en),.fifo_data(camera_fifo_raw_dout),
        .fifo_data_valid(camera_fifo_raw_valid),
        .m_axis_data(camera_cdc_axis_data),
        .m_axis_valid(camera_cdc_axis_valid),
        .m_axis_ready(camera_cdc_axis_ready),
        .pending_out(camera_cdc_prefetch_pending)
    );

    // The CDC adapter already reserves every outstanding BRAM read response.
    // Feed its accepted records directly into the registered gearbox input.
    // This removes the 65,536-record UltraRAM absorb-and-prefetch stage.
    assign camera_cdc_axis_ready = camera_fifo_prefetch_ready;

    // Register READY between the small response queue and the byte gearbox.
    // The 91-bit payload includes physical SOF.
    // It accepts one word per cycle and holds data/SOF stable under stalls.
    axis_register #(
        .DATA_WIDTH(91),.KEEP_ENABLE(0),.KEEP_WIDTH(1),.LAST_ENABLE(0),
        .ID_ENABLE(0),.ID_WIDTH(1),.DEST_ENABLE(0),.DEST_WIDTH(1),
        .USER_ENABLE(0),.USER_WIDTH(1),.REG_TYPE(2)
    ) camera_input_timing_slice (
        .clk(axi_aclk),.rst(axi_domain_rst),
        .s_axis_tdata(camera_cdc_axis_data),.s_axis_tkeep(1'b1),
        .s_axis_tvalid(camera_cdc_axis_valid),.s_axis_tready(camera_fifo_prefetch_ready),
        .s_axis_tlast(1'b1),.s_axis_tid(1'b0),.s_axis_tdest(1'b0),.s_axis_tuser(1'b0),
        .m_axis_tdata(camera_fifo_dout),.m_axis_tkeep(),
        .m_axis_tvalid(camera_fifo_valid),.m_axis_tready(camera_fifo_rd_en),
        .m_axis_tlast(),.m_axis_tid(),.m_axis_tdest(),.m_axis_tuser()
    );

    logic [143:0] repack_data;
    logic [4:0]   repack_count;
    logic         repack_sof_pending;
    wire [3:0]    camera_fifo_bytes = count_keep10(camera_fifo_dout[89:80]);
    wire          repack_pop = data_fifo_valid && data_fifo_rd_en;

    assign data_fifo_valid = (repack_count >= 5'd8);
    assign data_fifo_dout = {repack_sof_pending, repack_data[63:0]};
    assign camera_fifo_rd_en = camera_fifo_valid && (repack_count < 5'd8);

    always_ff @(posedge axi_aclk) begin
        if (axi_domain_rst) begin
            repack_data        <= 144'd0;
            repack_count       <= 5'd0;
            repack_sof_pending <= 1'b0;
        end else if (repack_pop) begin
            repack_data  <= {64'd0, repack_data[143:64]};
            repack_count <= repack_count - 5'd8;
            if (repack_sof_pending)
                repack_sof_pending <= 1'b0;
        end else if (camera_fifo_rd_en) begin
            // A genuine SOF with residual bytes means the preceding malformed
            // frame ended off the programmed eight-byte HLS boundary. Flush
            // only that residue; the new SOF remains explicit downstream.
            if (camera_fifo_dout[90] && repack_count != 0) begin
                repack_data  <= {64'd0, camera_fifo_dout[79:0]};
                repack_count <= camera_fifo_bytes;
            end else begin
                case (repack_count[2:0])
                    3'd0: repack_data[79:0]   <= camera_fifo_dout[79:0];
                    3'd1: repack_data[87:8]   <= camera_fifo_dout[79:0];
                    3'd2: repack_data[95:16]  <= camera_fifo_dout[79:0];
                    3'd3: repack_data[103:24] <= camera_fifo_dout[79:0];
                    3'd4: repack_data[111:32] <= camera_fifo_dout[79:0];
                    3'd5: repack_data[119:40] <= camera_fifo_dout[79:0];
                    3'd6: repack_data[127:48] <= camera_fifo_dout[79:0];
                    3'd7: repack_data[135:56] <= camera_fifo_dout[79:0];
                endcase
                repack_count <= repack_count + camera_fifo_bytes;
            end
            if (camera_fifo_dout[90])
                repack_sof_pending <= 1'b1;
        end
    end

    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0)
    ) data_fifo_overflow_axi_cdc_inst (
        .src_clk  (pix_clk),
        .src_in   (data_fifo_overflow_latched_pix),
        .dest_clk (axi_aclk),
        .dest_out (data_fifo_overflow_axi)
    );

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE    ("auto"),
        .FIFO_WRITE_DEPTH    (64),
        .WRITE_DATA_WIDTH    (128),
        .READ_DATA_WIDTH     (128),
        .READ_MODE           ("fwft"),
        .FIFO_READ_LATENCY   (0),
        .CDC_SYNC_STAGES     (2),
        .DOUT_RESET_VALUE    ("0"),
        .ECC_MODE            ("no_ecc"),
        .RELATED_CLOCKS      (0),
        .USE_ADV_FEATURES    ("1100")
    ) meta_fifo_inst (
        .rst          (pix_domain_rst),
        .wr_clk       (pix_clk),
        .rd_clk       (axi_aclk),
        .din          (meta_fifo_din),
        .wr_en        (meta_fifo_wr_en),
        .rd_en        (meta_fifo_rd_en),
        .dout         (meta_fifo_dout),
        .full         (meta_fifo_full),
        .empty        (meta_fifo_empty),
        .data_valid   (meta_fifo_valid),
        .overflow     (meta_fifo_overflow_event_pix),
        .underflow    (),
        .wr_rst_busy  (),
        .rd_rst_busy  (),
        .sleep        (1'b0),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .wr_data_count(),
        .rd_data_count(),
        .prog_full    (),
        .prog_empty   (),
        .almost_full  (),
        .almost_empty (),
        .wr_ack       ()
    );

    xpm_cdc_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0)
    ) meta_fifo_overflow_axi_cdc_inst (
        .src_clk  (pix_clk),
        .src_in   (meta_fifo_overflow_latched_pix),
        .dest_clk (axi_aclk),
        .dest_out (meta_fifo_overflow_axi)
    );

    typedef enum logic [2:0] {
        CAP_IDLE,
        CAP_COLLECT,
        CAP_DROP,
        CAP_WAIT_META
    } cap_state_t;

    typedef enum logic [1:0] {
        WR_IDLE,
        WR_AW,
        WR_W
    } wr_state_t;

    typedef enum logic [2:0] {
        HLS_CFG_WAIT,
        HLS_CFG_AW,
        HLS_CFG_W,
        HLS_CFG_B,
        HLS_CFG_DONE,
        HLS_CFG_POLL_AR,
        HLS_CFG_POLL_R
    } hls_cfg_state_t;

    cap_state_t cap_state;
    wr_state_t  wr_state;
    hls_cfg_state_t hls_cfg_state;

    logic [31:0]               bytes_seen;
    logic [31:0]               meta_seq;
    logic [31:0]               meta_width;
    logic [31:0]               meta_height;
    logic [31:0]               meta_bytes;
    logic                      meta_loaded;
    logic [31:0]               drop_bytes_seen;
    logic                      drop_output_done;
    logic [4:0]                stream_drop_reason;
    logic [AXI_ADDR_WIDTH-1:0] next_slot_addr;
    logic [4:0]                next_slot_pos;
    logic [AXI_ADDR_WIDTH-1:0] burst_start_addr;
    logic [AXI_ADDR_WIDTH-1:0] next_beat_addr;
    logic [BURST_COUNT_WIDTH-1:0] burst_beat_count;
    logic [AXI_ADDR_WIDTH-1:0] final_burst_addr;
    logic [7:0]                final_burst_len;
    // Frames whose final AXI burst has been issued but whose final BRESP has
    // not yet published the corresponding DDR slot. This contributes to ring
    // occupancy, but it does not stall the following HLS frame.
    logic [5:0]                reserved_frame_count;

    logic [BEAT_FIFO_WIDTH-1:0] beat_fifo_din;
    logic [BEAT_FIFO_WIDTH-1:0] beat_fifo_dout;
    logic                       beat_fifo_wr_en;
    wire                        beat_fifo_rd_en;
    logic                       beat_fifo_full;
    wire                        beat_fifo_backpressure;
    wire                        beat_fifo_overflow_event;
    logic                       beat_fifo_valid;
    wire [AXI_DATA_WIDTH-1:0]   beat_fifo_data = beat_fifo_dout[0 +: AXI_DATA_WIDTH];
    wire [AXI_KEEP_WIDTH-1:0]   beat_fifo_strb = beat_fifo_dout[AXI_DATA_WIDTH +: AXI_KEEP_WIDTH];

    logic [CMD_FIFO_WIDTH-1:0]  cmd_fifo_din;
    logic [CMD_FIFO_WIDTH-1:0]  cmd_fifo_dout;
    logic                       cmd_fifo_wr_en;
    wire                        cmd_fifo_rd_en;
    logic                       cmd_fifo_full;
    wire                        cmd_fifo_backpressure;
    wire                        cmd_fifo_overflow_event;
    logic                       cmd_fifo_valid;
    wire [7:0]                  cmd_fifo_len = cmd_fifo_dout[0 +: 8];
    wire [AXI_ADDR_WIDTH-1:0]   cmd_fifo_addr = cmd_fifo_dout[8 +: AXI_ADDR_WIDTH];
    wire [CMD_META_WIDTH-1:0]   cmd_fifo_meta = cmd_fifo_dout[8+AXI_ADDR_WIDTH +: CMD_META_WIDTH];
    wire                        cmd_fifo_final = cmd_fifo_dout[8+AXI_ADDR_WIDTH+CMD_META_WIDTH];
    // The upper command bit marks a frame that must be discarded after its
    // final BRESP. It is not a zero-length commit command.
    wire                        cmd_fifo_drop = cmd_fifo_dout[8+AXI_ADDR_WIDTH+CMD_META_WIDTH+1];
    wire [4:0]                  cmd_fifo_reason = cmd_fifo_dout[8+AXI_ADDR_WIDTH+CMD_META_WIDTH+2 +: 5];

    logic [AXI_ADDR_WIDTH-1:0]  active_cmd_addr;
    logic [7:0]                 active_cmd_len;
    logic                       active_cmd_final;
    logic                       active_cmd_drop;
    logic [4:0]                 active_cmd_reason;
    logic [31:0]                active_cmd_seq;
    logic [31:0]                active_cmd_width;
    logic [31:0]                active_cmd_height;
    logic [31:0]                active_cmd_bytes;
    logic [7:0]                 write_beat_idx;

    logic [OUTSTANDING_COUNT_WIDTH-1:0] outstanding_count;
    logic [AXI_MAX_OUTSTANDING-1:0]     resp_final;
    logic [AXI_MAX_OUTSTANDING-1:0]     resp_drop;
    logic [4:0]                         resp_reason[0:AXI_MAX_OUTSTANDING-1];
    logic [31:0]                        resp_seq   [0:AXI_MAX_OUTSTANDING-1];
    logic [31:0]                        resp_width [0:AXI_MAX_OUTSTANDING-1];
    logic [31:0]                        resp_height[0:AXI_MAX_OUTSTANDING-1];
    logic [31:0]                        resp_bytes [0:AXI_MAX_OUTSTANDING-1];

    logic        use_dval_axi;
    logic [AXI_ADDR_WIDTH-1:0] ring_base_axi;
    logic [31:0] frame_stride_axi;
    logic [31:0] consumer_idx_axi;
    logic [31:0] expected_line_bytes_axi;
    logic [31:0] expected_frame_bytes_axi;
    logic [31:0] expected_height_axi;
    logic [3:0]  isp_enable_axi;
    logic [7:0]  isp_black_level_axi;
    logic [15:0] isp_wb_gain_r_axi;
    logic [15:0] isp_wb_gain_g_axi;
    logic [15:0] isp_wb_gain_b_axi;
    logic signed [15:0] isp_ccm_00_axi;
    logic signed [15:0] isp_ccm_01_axi;
    logic signed [15:0] isp_ccm_02_axi;
    logic signed [15:0] isp_ccm_10_axi;
    logic signed [15:0] isp_ccm_11_axi;
    logic signed [15:0] isp_ccm_12_axi;
    logic signed [15:0] isp_ccm_20_axi;
    logic signed [15:0] isp_ccm_21_axi;
    logic signed [15:0] isp_ccm_22_axi;
    logic [191:0] isp_cfg_cdc_src;
    logic [191:0] isp_cfg_cdc_dst;
    logic [205:0] axil_cfg_cdc_src;
    logic [205:0] axil_cfg_cdc_dst;

    assign axil_cfg_cdc_src = {
        cap_enable,
        use_dval_reg,
        ring_base_reg,
        frame_stride_reg,
        consumer_idx_reg,
        expected_line_bytes_reg,
        expected_frame_bytes_reg,
        expected_height_reg,
        isp_enable_reg,
        isp_black_level_reg
    };

    assign {
        cap_enable_axi,
        use_dval_axi,
        ring_base_axi,
        frame_stride_axi,
        consumer_idx_axi,
        expected_line_bytes_axi,
        expected_frame_bytes_axi,
        expected_height_axi,
        isp_enable_axi,
        isp_black_level_axi
    } = axil_cfg_cdc_dst;

    xpm_cdc_array_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0),
        .WIDTH          (206)
    ) cfg_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (axil_cfg_cdc_src),
        .dest_clk (axi_aclk),
        .dest_out (axil_cfg_cdc_dst)
    );

    // These scalar controls are stable while capture is enabled: AXI-Lite
    // rejects profile writes when cap_enable=1. Keeping them in a separate CDC
    // prevents the geometry/control bundle from acquiring extra fanout.
    assign isp_cfg_cdc_src = {
        isp_wb_gain_r_reg, isp_wb_gain_g_reg, isp_wb_gain_b_reg,
        isp_ccm_00_reg, isp_ccm_01_reg, isp_ccm_02_reg,
        isp_ccm_10_reg, isp_ccm_11_reg, isp_ccm_12_reg,
        isp_ccm_20_reg, isp_ccm_21_reg, isp_ccm_22_reg
    };
    assign {
        isp_wb_gain_r_axi, isp_wb_gain_g_axi, isp_wb_gain_b_axi,
        isp_ccm_00_axi, isp_ccm_01_axi, isp_ccm_02_axi,
        isp_ccm_10_axi, isp_ccm_11_axi, isp_ccm_12_axi,
        isp_ccm_20_axi, isp_ccm_21_axi, isp_ccm_22_axi
    } = isp_cfg_cdc_dst;

    xpm_cdc_array_single #(
        .DEST_SYNC_FF   (2),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (0),
        .WIDTH          (192)
    ) isp_cfg_cdc_inst (
        .src_clk  (axil_aclk),
        .src_in   (isp_cfg_cdc_src),
        .dest_clk (axi_aclk),
        .dest_out (isp_cfg_cdc_dst)
    );

    // The receiver counter changes only after eight consecutive bad clock
    // words and then remains stable. Gray-code synchronizers make it
    // available to the AXI frame guard and to software diagnostics.
    xpm_cdc_gray #(.DEST_SYNC_FF(2),.INIT_SYNC_FF(1),.REG_OUTPUT(1),
        .SIM_ASSERT_CHK(0),.SIM_LOSSLESS_GRAY_CHK(0),.WIDTH(32)) cmlk_align_loss_axi_cdc_inst (
        .src_clk  (pix_clk),
        .src_in_bin   (cmlk_align_loss_count),
        .dest_clk (axi_aclk),
        .dest_out_bin (cmlk_align_loss_count_axi)
    );

    xpm_cdc_gray #(.DEST_SYNC_FF(2),.INIT_SYNC_FF(1),.REG_OUTPUT(1),
        .SIM_ASSERT_CHK(0),.SIM_LOSSLESS_GRAY_CHK(0),.WIDTH(32)) cmlk_align_loss_axil_cdc_inst (
        .src_clk  (pix_clk),
        .src_in_bin   (cmlk_align_loss_count),
        .dest_clk (axil_aclk),
        .dest_out_bin (cmlk_align_loss_count_axil)
    );

    wire ring_full, ring_decision_valid;
    camera_ring_guard ring_guard_inst (
        .clk(axi_aclk),.rst(axi_domain_rst || !cap_enable_axi),
        .idle(cap_state == CAP_IDLE),.producer(producer_idx_reg),
        .consumer(consumer_idx_axi),.reserved(reserved_frame_count),
        .full(ring_full),.decision_valid(ring_decision_valid)
    );
    wire cfg_static_ok;
    // Only the registered decision reaches HLS/repacker VALID/READY/CE.
    // Geometry is programmed while stopped; the HLS startup waits 16 cycles.
    camera_config_validator #(.ADDR_WIDTH(AXI_ADDR_WIDTH)) cfg_validator_inst (
        .clk(axi_aclk), .rst(axi_domain_rst),
        .ring_base(ring_base_axi), .frame_stride(frame_stride_axi),
        .line_bytes(expected_line_bytes_axi),
        .frame_bytes(expected_frame_bytes_axi), .height(expected_height_axi),
        .cfg_ok(cfg_static_ok)
    );

    /* ---------------------------------------------------------------------
     * HLS BayerRG8 -> RGB888 converter.
     *
     * The HLS block is reset whenever capture is disabled.  On each enable the
     * small internal AXI-Lite master programs raw Bayer width/height, then sets
     * AP_START for one frame (control value 0x01). A small poll FSM rearms each
     * completed invocation; pending ISP profiles are written only at AP_IDLE.
     * Capture, DDR DMA and RDMA are never reset by an online ISP update.  The source stream remains
     * stalled in the asynchronous FIFO until this sequence has completed.
     * Runtime geometry changes use disable -> program -> enable; disabling
     * capture resets the HLS core and guarantees a clean reconfiguration.
     * ------------------------------------------------------------------ */
    logic [7:0]  hls_axil_awaddr;
    logic        hls_axil_awvalid;
    wire         hls_axil_awready;
    logic [31:0] hls_axil_wdata;
    logic [3:0]  hls_axil_wstrb;
    logic        hls_axil_wvalid;
    wire         hls_axil_wready;
    wire [1:0]   hls_axil_bresp;
    wire         hls_axil_bvalid;
    logic        hls_axil_bready;
    logic [7:0]  hls_axil_araddr;
    logic        hls_axil_arvalid;
    wire         hls_axil_arready;
    wire [31:0]  hls_axil_rdata;
    wire [1:0]   hls_axil_rresp;
    wire         hls_axil_rvalid;
    logic        hls_axil_rready;
    wire         hls_interrupt;

    wire         hls_s_axis_bayer_tvalid;
    wire         hls_s_axis_bayer_tready;
    wire [63:0]  hls_s_axis_bayer_tdata;
    wire [0:0]   hls_s_axis_bayer_tdest;
    wire [7:0]   hls_s_axis_bayer_tkeep;
    wire [7:0]   hls_s_axis_bayer_tstrb;
    wire [0:0]   hls_s_axis_bayer_tuser;
    wire [0:0]   hls_s_axis_bayer_tlast;
    wire [0:0]   hls_s_axis_bayer_tid;

    wire hls_m_axis_rgb_raw_tvalid, hls_m_axis_rgb_raw_tready;
    wire [191:0] hls_m_axis_rgb_raw_tdata;
    wire [0:0] hls_m_axis_rgb_raw_tdest, hls_m_axis_rgb_raw_tuser;
    wire [0:0] hls_m_axis_rgb_raw_tlast, hls_m_axis_rgb_raw_tid;
    wire [23:0] hls_m_axis_rgb_raw_tkeep, hls_m_axis_rgb_raw_tstrb;
    wire hls_m_axis_rgb_pack_tvalid, hls_m_axis_rgb_pack_tready;
    wire [511:0] hls_m_axis_rgb_pack_tdata;
    wire [63:0] hls_m_axis_rgb_pack_tkeep;
    wire hls_m_axis_rgb_pack_tuser, hls_m_axis_rgb_pack_tlast;
    wire hls_m_axis_rgb_tvalid;
    wire hls_m_axis_rgb_tready;
    wire fork_in_valid, fork_in_ready, fork_in_last;
    wire [511:0] fork_in_data;
    wire [63:0] fork_in_keep;
    wire [7:0] fork_in_sideband;
    wire [511:0] host_fork_data;
    wire [63:0] host_fork_keep;
    wire host_fork_valid,host_fork_last,host_fork_sof,host_fork_pending;
    wire fork_drained;
    wire [511:0] hls_m_axis_rgb_tdata;
    wire [63:0] hls_m_axis_rgb_tkeep, hls_m_axis_rgb_tstrb;
    wire [0:0] hls_m_axis_rgb_tuser, hls_m_axis_rgb_tlast;
    wire [7:0] hls_m_axis_rgb_sideband;
    wire [6:0] hls_m_axis_rgb_beat_bytes;

    logic [3:0] hls_cfg_wait_count;
    localparam logic [4:0] HLS_CFG_LAST = 5'd16;
    logic [4:0]  hls_cfg_index;
    logic [7:0]  hls_cfg_addr;
    logic [31:0] hls_cfg_data;
    logic [3:0]  hls_isp_enable_cfg;
    logic [7:0]  hls_isp_black_level_cfg;
    logic [15:0] hls_isp_wb_gain_r_cfg;
    logic [15:0] hls_isp_wb_gain_g_cfg;
    logic [15:0] hls_isp_wb_gain_b_cfg;
    logic signed [15:0] hls_isp_ccm_00_cfg;
    logic signed [15:0] hls_isp_ccm_01_cfg;
    logic signed [15:0] hls_isp_ccm_02_cfg;
    logic signed [15:0] hls_isp_ccm_10_cfg;
    logic signed [15:0] hls_isp_ccm_11_cfg;
    logic signed [15:0] hls_isp_ccm_12_cfg;
    logic signed [15:0] hls_isp_ccm_20_cfg;
    logic signed [15:0] hls_isp_ccm_21_cfg;
    logic signed [15:0] hls_isp_ccm_22_cfg;
    logic [5:0] hls_poll_wait;
    logic hls_live_programming;
    logic hls_seen_busy, hls_seen_done;
    logic       hls_cfg_done;
    logic       hls_input_in_frame;
    logic [31:0] hls_input_word_index;
    logic       hls_pad_active;
    logic       hls_drop_active;
    logic [4:0] hls_stop_quiet_count;
    // Require all CDC and prefetch stages to be empty for 16 AXI clocks
    // after frame-stop ACK before padding a physically short last HLS frame.
    wire hls_stop_front_empty = frame_stop_ack_axi &&
        camera_fifo_empty && !camera_fifo_raw_valid &&
        !camera_cdc_prefetch_pending && !camera_cdc_axis_valid &&
        !camera_fifo_valid && !data_fifo_valid;

    // axi_domain_rst already contains the synchronized external reset. Do
    // not bypass that synchronizer with a raw control-domain reset here.
    wire hls_ap_rst_n = !axi_domain_rst && cap_enable_axi;
    wire hls_stream_enable = hls_ap_rst_n && hls_cfg_done && cfg_static_ok;
    wire [31:0] hls_expected_words = expected_frame_bytes_axi >> 3;
    wire hls_input_head_valid = hls_input_in_frame || data_fifo_dout[64];
    wire hls_unexpected_sof = hls_stream_enable && data_fifo_valid &&
                              hls_input_in_frame && data_fifo_dout[64];
    wire hls_pad_word = hls_pad_active || hls_unexpected_sof;
    wire hls_drop_preamble = hls_stream_enable && data_fifo_valid &&
                             !hls_input_in_frame && !data_fifo_dout[64];
    wire hls_input_fire = hls_s_axis_bayer_tvalid && hls_s_axis_bayer_tready;

    assign hls_s_axis_bayer_tvalid = hls_stream_enable &&
                                     (hls_pad_word ||
                                      (data_fifo_valid && hls_input_head_valid));
    assign hls_s_axis_bayer_tdata  = hls_pad_word ? 64'd0 : data_fifo_dout[63:0];
    assign hls_s_axis_bayer_tkeep  = 8'hff;
    assign hls_s_axis_bayer_tstrb  = 8'hff;
    assign hls_s_axis_bayer_tuser  = !hls_input_in_frame;
    assign hls_s_axis_bayer_tlast  = (!hls_input_in_frame) ?
                                     (hls_expected_words == 32'd1) :
                                     (hls_input_word_index == (hls_expected_words - 1'b1));
    assign hls_s_axis_bayer_tid    = 1'b0;
    assign hls_s_axis_bayer_tdest  = 1'b0;
    // Padding closes only the damaged HLS invocation. The real SOF remains at
    // the FWFT FIFO head and starts the following invocation without phase
    // drift. This recovery path is inactive for well-formed frames.
    assign data_fifo_rd_en = (hls_input_fire && !hls_pad_word) ||
                             hls_drop_preamble;

    always_comb begin
        // The table is deliberately data-driven so adding a scalar HLS control
        // does not require three new FSM states (AW/W/B) per register.
        hls_cfg_addr = 8'h00;
        hls_cfg_data = 32'd0;
        case (hls_cfg_index)
            5'd0:  begin hls_cfg_addr = 8'h10; hls_cfg_data = {16'd0, expected_line_bytes_axi[15:0]}; end
            5'd1:  begin hls_cfg_addr = 8'h18; hls_cfg_data = {16'd0, expected_height_axi[15:0]}; end
            5'd2:  begin hls_cfg_addr = 8'h20; hls_cfg_data = {28'd0, hls_isp_enable_cfg}; end
            5'd3:  begin hls_cfg_addr = 8'h28; hls_cfg_data = {24'd0, hls_isp_black_level_cfg}; end
            5'd4:  begin hls_cfg_addr = 8'h30; hls_cfg_data = {16'd0, hls_isp_wb_gain_r_cfg}; end
            5'd5:  begin hls_cfg_addr = 8'h38; hls_cfg_data = {16'd0, hls_isp_wb_gain_g_cfg}; end
            5'd6:  begin hls_cfg_addr = 8'h40; hls_cfg_data = {16'd0, hls_isp_wb_gain_b_cfg}; end
            5'd7:  begin hls_cfg_addr = 8'h48; hls_cfg_data = {{16{hls_isp_ccm_00_cfg[15]}}, hls_isp_ccm_00_cfg}; end
            5'd8:  begin hls_cfg_addr = 8'h50; hls_cfg_data = {{16{hls_isp_ccm_01_cfg[15]}}, hls_isp_ccm_01_cfg}; end
            5'd9:  begin hls_cfg_addr = 8'h58; hls_cfg_data = {{16{hls_isp_ccm_02_cfg[15]}}, hls_isp_ccm_02_cfg}; end
            5'd10: begin hls_cfg_addr = 8'h60; hls_cfg_data = {{16{hls_isp_ccm_10_cfg[15]}}, hls_isp_ccm_10_cfg}; end
            5'd11: begin hls_cfg_addr = 8'h68; hls_cfg_data = {{16{hls_isp_ccm_11_cfg[15]}}, hls_isp_ccm_11_cfg}; end
            5'd12: begin hls_cfg_addr = 8'h70; hls_cfg_data = {{16{hls_isp_ccm_12_cfg[15]}}, hls_isp_ccm_12_cfg}; end
            5'd13: begin hls_cfg_addr = 8'h78; hls_cfg_data = {{16{hls_isp_ccm_20_cfg[15]}}, hls_isp_ccm_20_cfg}; end
            5'd14: begin hls_cfg_addr = 8'h80; hls_cfg_data = {{16{hls_isp_ccm_21_cfg[15]}}, hls_isp_ccm_21_cfg}; end
            5'd15: begin hls_cfg_addr = 8'h88; hls_cfg_data = {{16{hls_isp_ccm_22_cfg[15]}}, hls_isp_ccm_22_cfg}; end
            // Single frame invocation: do not auto-restart before the next profile is installed.
            5'd16: begin hls_cfg_addr = 8'h00; hls_cfg_data = 32'h0000_0001; end
            default: begin hls_cfg_addr = 8'h00; hls_cfg_data = 32'd0; end
        endcase

        hls_axil_awaddr  = hls_cfg_addr;
        hls_axil_awvalid = (hls_cfg_state == HLS_CFG_AW);
        hls_axil_wdata   = hls_cfg_data;
        hls_axil_wstrb   = 4'hf;
        hls_axil_wvalid  = (hls_cfg_state == HLS_CFG_W);
        hls_axil_bready  = 1'b1;
        hls_axil_araddr  = 8'd0;
        hls_axil_arvalid = (hls_cfg_state == HLS_CFG_POLL_AR);
        hls_axil_rready  = 1'b1;
    end

    always_ff @(posedge axi_aclk) begin
        if (!hls_ap_rst_n) begin
            hls_poll_wait<=0;hls_live_programming<=0;hls_seen_busy<=0;hls_seen_done<=0;live_done<=0;live_failed<=0;
            hls_cfg_state      <= HLS_CFG_WAIT;
            hls_cfg_index      <= 5'd0;
            hls_cfg_wait_count <= 4'd0;
            hls_cfg_done       <= 1'b0;
            hls_cfg_error_reg  <= 1'b0;
            hls_isp_enable_cfg      <= ISP_ENABLE;
            hls_isp_black_level_cfg <= ISP_BLACK_LEVEL;
            hls_isp_wb_gain_r_cfg   <= ISP_WB_GAIN_R;
            hls_isp_wb_gain_g_cfg   <= ISP_WB_GAIN_G;
            hls_isp_wb_gain_b_cfg   <= ISP_WB_GAIN_B;
            hls_isp_ccm_00_cfg      <= ISP_CCM_00;
            hls_isp_ccm_01_cfg      <= ISP_CCM_01;
            hls_isp_ccm_02_cfg      <= ISP_CCM_02;
            hls_isp_ccm_10_cfg      <= ISP_CCM_10;
            hls_isp_ccm_11_cfg      <= ISP_CCM_11;
            hls_isp_ccm_12_cfg      <= ISP_CCM_12;
            hls_isp_ccm_20_cfg      <= ISP_CCM_20;
            hls_isp_ccm_21_cfg      <= ISP_CCM_21;
            hls_isp_ccm_22_cfg      <= ISP_CCM_22;
        end else begin
            live_done<=0;
            if (soft_clear_axi)
                hls_cfg_error_reg <= 1'b0;
            case (hls_cfg_state)
                HLS_CFG_WAIT: begin
                    hls_cfg_done <= 1'b0;
                    // Capture one coherent stopped-capture profile. These
                    // values have the full wait interval to settle after CDC.
                    hls_isp_enable_cfg      <= isp_enable_axi & ~{uip_config[3],uip_config[6],uip_config[1],uip_config[0]};
                    hls_isp_black_level_cfg <= isp_black_level_axi;
                    hls_isp_wb_gain_r_cfg   <= isp_wb_gain_r_axi;
                    hls_isp_wb_gain_g_cfg   <= isp_wb_gain_g_axi;
                    hls_isp_wb_gain_b_cfg   <= isp_wb_gain_b_axi;
                    hls_isp_ccm_00_cfg      <= isp_ccm_00_axi;
                    hls_isp_ccm_01_cfg      <= isp_ccm_01_axi;
                    hls_isp_ccm_02_cfg      <= isp_ccm_02_axi;
                    hls_isp_ccm_10_cfg      <= isp_ccm_10_axi;
                    hls_isp_ccm_11_cfg      <= isp_ccm_11_axi;
                    hls_isp_ccm_12_cfg      <= isp_ccm_12_axi;
                    hls_isp_ccm_20_cfg      <= isp_ccm_20_axi;
                    hls_isp_ccm_21_cfg      <= isp_ccm_21_axi;
                    hls_isp_ccm_22_cfg      <= isp_ccm_22_axi;
                    if (hls_cfg_wait_count != 4'hf)
                        hls_cfg_wait_count <= hls_cfg_wait_count + 1'b1;
                    else if (cfg_static_ok) begin
                        hls_cfg_index <= 5'd0;
                        hls_cfg_state <= HLS_CFG_AW;
                    end
                end
                HLS_CFG_AW:
                    if (hls_axil_awready)
                        hls_cfg_state <= HLS_CFG_W;
                HLS_CFG_W:
                    if (hls_axil_wready)
                        hls_cfg_state <= HLS_CFG_B;
                HLS_CFG_B:
                    if (hls_axil_bvalid) begin
                        if (hls_axil_bresp != 2'b00)
                            hls_cfg_error_reg <= 1'b1;
                        if (hls_cfg_index == HLS_CFG_LAST) begin
                            hls_cfg_done <= !(hls_cfg_error_reg ||
                                              (hls_axil_bresp != 2'b00));
                            hls_cfg_state <= HLS_CFG_DONE;
                            hls_poll_wait<=0;hls_seen_busy<=0;hls_seen_done<=0;
                            if(hls_live_programming)begin
                                live_done<=1;live_failed<=hls_cfg_error_reg || hls_axil_bresp!=0;
                                hls_live_programming<=0;
                            end
                        end else begin
                            hls_cfg_index <= hls_cfg_index + 1'b1;
                            hls_cfg_state <= HLS_CFG_AW;
                        end
                    end
                HLS_CFG_DONE: begin
                    hls_cfg_done <= !hls_cfg_error_reg;
                    if(!hls_cfg_error_reg)begin
                        if(hls_poll_wait==6'd63)begin hls_poll_wait<=0;hls_cfg_state<=HLS_CFG_POLL_AR;end
                        else hls_poll_wait<=hls_poll_wait+1'b1;
                    end
                end
                HLS_CFG_POLL_AR:if(hls_axil_arready)hls_cfg_state<=HLS_CFG_POLL_R;
                HLS_CFG_POLL_R:if(hls_axil_rvalid)begin
                    hls_cfg_state<=HLS_CFG_DONE;
                    if(hls_axil_rresp!=0)hls_cfg_error_reg<=1;
                    else begin
                        if(!hls_axil_rdata[2])hls_seen_busy<=1;
                        if(hls_axil_rdata[1])hls_seen_done<=1;
                        // AP_IDLE means the prior invocation no longer uses scalar controls.
                        // Sticky AP_DONE / seen-busy avoids treating stale reset idle as completion.
                        if(hls_axil_rdata[2] && (hls_seen_busy || hls_seen_done || hls_axil_rdata[1]))begin
                            hls_cfg_index<=HLS_CFG_LAST;
                            if(live_pending)begin
                            hls_isp_enable_cfg<=live_profile[0*32+:4] & ~{uip_config[3],uip_config[6],uip_config[1],uip_config[0]};
                            hls_isp_black_level_cfg<=live_profile[1*32+:8];
                            hls_isp_wb_gain_r_cfg<=live_profile[2*32+:16];
                            hls_isp_wb_gain_g_cfg<=live_profile[3*32+:16];
                            hls_isp_wb_gain_b_cfg<=live_profile[4*32+:16];
                            hls_isp_ccm_00_cfg<=live_profile[5*32+:16];
                            hls_isp_ccm_01_cfg<=live_profile[6*32+:16];
                            hls_isp_ccm_02_cfg<=live_profile[7*32+:16];
                            hls_isp_ccm_10_cfg<=live_profile[8*32+:16];
                            hls_isp_ccm_11_cfg<=live_profile[9*32+:16];
                            hls_isp_ccm_12_cfg<=live_profile[10*32+:16];
                            hls_isp_ccm_20_cfg<=live_profile[11*32+:16];
                            hls_isp_ccm_21_cfg<=live_profile[12*32+:16];
                            hls_isp_ccm_22_cfg<=live_profile[13*32+:16];
                                hls_cfg_index<=5'd2;hls_live_programming<=1;
                            end
                            hls_cfg_state<=HLS_CFG_AW;
                        end
                    end
                end
                default: hls_cfg_state <= HLS_CFG_WAIT;
            endcase
        end
    end

    always_ff @(posedge axi_aclk) begin
        if (!hls_ap_rst_n) begin
            hls_input_in_frame   <= 1'b0;
            hls_input_word_index <= 32'd0;
            hls_input_error_reg  <= 1'b0;
            hls_pad_active       <= 1'b0;
            hls_drop_active      <= 1'b0;
            hls_stop_quiet_count <= 5'd0;
            hls_resync_count_reg <= 32'd0;
        end else begin
            if (soft_clear_axi) begin
                hls_input_error_reg  <= 1'b0;
            end

            if (!hls_stop_front_empty)
                hls_stop_quiet_count <= 5'd0;
            else if (hls_stop_quiet_count != 5'd31)
                hls_stop_quiet_count <= hls_stop_quiet_count + 1'b1;
            if (hls_stop_quiet_count == 5'd16 &&
                hls_input_in_frame && !hls_pad_active) begin
                hls_pad_active <= 1'b1;
                if (!soft_clear_axi && hls_resync_count_reg != 32'hffff_ffff)
                    hls_resync_count_reg <= hls_resync_count_reg + 1'b1;
            end

            // A SOF before the programmed word count means that the preceding
            // physical frame was short. Keep that SOF in the FIFO and feed
            // neutral padding until the current HLS invocation reaches TLAST.
            if (hls_unexpected_sof && !hls_pad_active) begin
                hls_pad_active <= 1'b1;
                if (!soft_clear_axi && hls_resync_count_reg != 32'hffff_ffff)
                    hls_resync_count_reg <= hls_resync_count_reg + 1'b1;
            end

            // Conversely, words after the programmed frame size are discarded
            // until the next SOF. Count one resync episode, not every word.
            if (hls_drop_preamble) begin
                if (!hls_drop_active && !soft_clear_axi &&
                    hls_resync_count_reg != 32'hffff_ffff)
                    hls_resync_count_reg <= hls_resync_count_reg + 1'b1;
                hls_drop_active <= 1'b1;
            end else if (data_fifo_valid && data_fifo_dout[64]) begin
                hls_drop_active <= 1'b0;
            end

            if (hls_input_fire) begin
                if (!hls_input_in_frame) begin
                    hls_input_word_index <= 32'd1;
                    if (hls_expected_words <= 32'd1) begin
                        hls_input_in_frame   <= 1'b0;
                        hls_input_word_index <= 32'd0;
                    end else begin
                        hls_input_in_frame <= 1'b1;
                    end
                end else begin
                    if (hls_input_word_index >= (hls_expected_words - 1'b1)) begin
                        hls_input_in_frame   <= 1'b0;
                        hls_input_word_index <= 32'd0;
                        hls_pad_active       <= 1'b0;
                    end else begin
                        hls_input_word_index <= hls_input_word_index + 1'b1;
                    end
                end
            end
        end
    end

    wire pp_raw_valid,pp_raw_ready,pp_raw_sof,pp_raw_last;
    wire [63:0] pp_raw_data;
    wire pp_rgb_valid,pp_rgb_ready,pp_rgb_sof,pp_rgb_last;
    wire [191:0] pp_rgb_data;
    wire [271:0] raw_config={uip_config[271:7],2'b0,uip_config[4],1'b0,uip_config[2:0]};
    wire [271:0] rgb_config={uip_config[271:7],uip_config[6:5],1'b0,uip_config[3],3'b0};
    camera_isp_stream #(.LANES(8),.CHANNELS(1)) common_raw_isp (
      .clk(axi_aclk),.rst(!hls_ap_rst_n),.width(expected_line_bytes_axi[15:0]),.bayer(1'b1),
      .s_valid(hls_s_axis_bayer_tvalid),.s_ready(hls_s_axis_bayer_tready),.s_data(hls_s_axis_bayer_tdata),
      .s_sof(hls_s_axis_bayer_tuser[0]),.s_last(hls_s_axis_bayer_tlast[0]),
      .m_valid(pp_raw_valid),.m_ready(pp_raw_ready),.m_data(pp_raw_data),.m_sof(pp_raw_sof),.m_last(pp_raw_last),
      .config_data(raw_config),.config_clk(axil_aclk),.lut_we(uip_lut_we),.lut_addr(uip_lut_addr),.lut_data(uip_lut_data));
    camera_isp_stream #(.LANES(8),.CHANNELS(3)) common_rgb_isp (
      .clk(axi_aclk),.rst(!hls_ap_rst_n),.width(expected_line_bytes_axi[15:0]),.bayer(1'b0),
      .s_valid(hls_m_axis_rgb_raw_tvalid),.s_ready(hls_m_axis_rgb_raw_tready),.s_data(hls_m_axis_rgb_raw_tdata),
      .s_sof(hls_m_axis_rgb_raw_tuser[0]),.s_last(hls_m_axis_rgb_raw_tlast[0]),
      .m_valid(pp_rgb_valid),.m_ready(pp_rgb_ready),.m_data(pp_rgb_data),.m_sof(pp_rgb_sof),.m_last(pp_rgb_last),
      .config_data(rgb_config),.config_clk(axil_aclk),.lut_we(uip_lut_we),.lut_addr(uip_lut_addr),.lut_data(uip_lut_data));

    bayer_rg8_to_rgba8888_0 bayer_rg8_to_rgba8888_inst (
        .s_axi_control_AWADDR  (hls_axil_awaddr),
        .s_axi_control_AWVALID (hls_axil_awvalid),
        .s_axi_control_AWREADY (hls_axil_awready),
        .s_axi_control_WDATA   (hls_axil_wdata),
        .s_axi_control_WSTRB   (hls_axil_wstrb),
        .s_axi_control_WVALID  (hls_axil_wvalid),
        .s_axi_control_WREADY  (hls_axil_wready),
        .s_axi_control_BRESP   (hls_axil_bresp),
        .s_axi_control_BVALID  (hls_axil_bvalid),
        .s_axi_control_BREADY  (hls_axil_bready),
        .s_axi_control_ARADDR  (hls_axil_araddr),
        .s_axi_control_ARVALID (hls_axil_arvalid),
        .s_axi_control_ARREADY (hls_axil_arready),
        .s_axi_control_RDATA   (hls_axil_rdata),
        .s_axi_control_RRESP   (hls_axil_rresp),
        .s_axi_control_RVALID  (hls_axil_rvalid),
        .s_axi_control_RREADY  (hls_axil_rready),
        .ap_clk                (axi_aclk),
        .ap_rst_n              (hls_ap_rst_n),
        .interrupt             (hls_interrupt),
        .s_axis_bayer_TVALID   (pp_raw_valid && !raw_output_axi),
        .s_axis_bayer_TREADY   (hls_core_input_ready),
        .s_axis_bayer_TDATA    (pp_raw_data),
        .s_axis_bayer_TDEST    (hls_s_axis_bayer_tdest),
        .s_axis_bayer_TKEEP    (hls_s_axis_bayer_tkeep),
        .s_axis_bayer_TSTRB    (hls_s_axis_bayer_tstrb),
        .s_axis_bayer_TUSER    (pp_raw_sof),
        .s_axis_bayer_TLAST    (pp_raw_last),
        .s_axis_bayer_TID      (hls_s_axis_bayer_tid),
        .m_axis_rgb_TVALID    (hls_m_axis_rgb_raw_tvalid),
        .m_axis_rgb_TREADY    (hls_m_axis_rgb_raw_tready),
        .m_axis_rgb_TDATA     (hls_m_axis_rgb_raw_tdata),
        .m_axis_rgb_TDEST     (hls_m_axis_rgb_raw_tdest),
        .m_axis_rgb_TKEEP     (hls_m_axis_rgb_raw_tkeep),
        .m_axis_rgb_TSTRB     (hls_m_axis_rgb_raw_tstrb),
        .m_axis_rgb_TUSER     (hls_m_axis_rgb_raw_tuser),
        .m_axis_rgb_TLAST     (hls_m_axis_rgb_raw_tlast),
        .m_axis_rgb_TID       (hls_m_axis_rgb_raw_tid)
    );

    // RAW bypass consumes the exact guarded eight Bayer bytes before HLS.
    // Mode is writable only while capture is disabled and crosses two FFs;
    // normal startup's configuration delay precedes either stream's VALID.
    wire [511:0] raw_pack_data;
    wire [63:0] raw_pack_keep;
    wire raw_pack_valid, raw_pack_ready, raw_pack_user, raw_pack_last;
    assign pp_raw_ready = raw_output_axi ? raw_input_ready : hls_core_input_ready;
    camera_raw64_to_axis512 raw_bayer_packer (
        .clk(axi_aclk),.rst(!hls_ap_rst_n),
        .s_data(pp_raw_data),
        .s_valid(pp_raw_valid && raw_output_axi),.s_ready(raw_input_ready),
        .s_user(pp_raw_sof),.s_last(pp_raw_last),
        .m_data(raw_pack_data),.m_keep(raw_pack_keep),.m_valid(raw_pack_valid),
        .m_ready(raw_pack_ready),.m_user(raw_pack_user),.m_last(raw_pack_last));
    wire selected_ready;
    assign raw_pack_ready = selected_ready && raw_output_axi;
    assign hls_m_axis_rgb_pack_tready = selected_ready && !raw_output_axi;
    wire [511:0] selected_data = raw_output_axi ? raw_pack_data : hls_m_axis_rgb_pack_tdata;
    wire [63:0] selected_keep = raw_output_axi ? raw_pack_keep : hls_m_axis_rgb_pack_tkeep;
    wire selected_valid = raw_output_axi ? raw_pack_valid : hls_m_axis_rgb_pack_tvalid;
    wire selected_user = raw_output_axi ? raw_pack_user : hls_m_axis_rgb_pack_tuser;
    wire selected_last = raw_output_axi ? raw_pack_last : hls_m_axis_rgb_pack_tlast;

    // Compact eight-pixel RGB beats are repacked byte-contiguously before DDR.
    // The packer has more than 2x headroom at 250 MHz for 2432x2048@140.
    camera_rgb192_to_axis512 camera_rgb_byte_packer_inst (
        .clk          (axi_aclk),
        .rst          (!hls_ap_rst_n),
        .s_axis_tdata (pp_rgb_data),
        .s_axis_tvalid(pp_rgb_valid),
        .s_axis_tready(pp_rgb_ready),
        .s_axis_tuser (pp_rgb_sof),
        .s_axis_tlast (pp_rgb_last),
        .m_axis_tdata (hls_m_axis_rgb_pack_tdata),
        .m_axis_tkeep (hls_m_axis_rgb_pack_tkeep),
        .m_axis_tvalid(hls_m_axis_rgb_pack_tvalid),
        .m_axis_tready(hls_m_axis_rgb_pack_tready),
        .m_axis_tuser (hls_m_axis_rgb_pack_tuser),
        .m_axis_tlast (hls_m_axis_rgb_pack_tlast)
    );

    // Register format selection separately from the existing byte-count stage.
    // Both skid stages use registered READY and sustain one beat per cycle.
    // The new 512-bit mux does not extend the old TKEEP popcount timing path.
    wire [511:0] selected_reg_data;
    wire [63:0] selected_reg_keep;
    wire selected_reg_valid, selected_reg_ready, selected_reg_user, selected_reg_last;
    axis_register #(
        .DATA_WIDTH(512),.KEEP_ENABLE(1),.KEEP_WIDTH(64),.LAST_ENABLE(1),
        .ID_ENABLE(0),.ID_WIDTH(1),.DEST_ENABLE(0),.DEST_WIDTH(1),
        .USER_ENABLE(1),.USER_WIDTH(1),.REG_TYPE(2)
    ) camera_format_skid_inst (
        .clk(axi_aclk),.rst(!hls_ap_rst_n),
        .s_axis_tdata(selected_data),.s_axis_tkeep(selected_keep),
        .s_axis_tvalid(selected_valid),.s_axis_tready(selected_ready),
        .s_axis_tlast(selected_last),.s_axis_tid(1'b0),.s_axis_tdest(1'b0),
        .s_axis_tuser(selected_user),
        .m_axis_tdata(selected_reg_data),.m_axis_tkeep(selected_reg_keep),
        .m_axis_tvalid(selected_reg_valid),.m_axis_tready(selected_reg_ready),
        .m_axis_tlast(selected_reg_last),.m_axis_tid(),.m_axis_tdest(),
        .m_axis_tuser(selected_reg_user)
    );

    function automatic [6:0] count_keep_bytes(input [63:0] keep);
        integer k;
        begin
            count_keep_bytes = 7'd0;
            for (k = 0; k < 64; k = k + 1)
                count_keep_bytes = count_keep_bytes + keep[k];
        end
    endfunction

    // A two-entry skid buffer registers TREADY back toward the HLS IP.  The
    // previous one-entry elastic register propagated CAP/ring/FIFO decisions
    // combinationally to the HLS output register CE, which produced the large
    // 250 MHz timing-failure cluster.  REG_TYPE=2 accepts one beat per cycle
    // without bubbles, including across a one-cycle downstream stall.
    //
    // TKEEP&TSTRB is the actual byte-valid mask.  Carry the popcount beside
    // TUSER so the capture FSM does not acquire a new 64-bit popcount path.
    axis_register #(
        .DATA_WIDTH (512),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH (64),
        .LAST_ENABLE(1),
        .ID_ENABLE   (0),
        .ID_WIDTH    (1),
        .DEST_ENABLE (0),
        .DEST_WIDTH  (1),
        .USER_ENABLE (1),
        .USER_WIDTH  (8),
        .REG_TYPE    (2)
    ) camera_hls_rgb_skid_inst (
        .clk          (axi_aclk),
        .rst          (!hls_ap_rst_n),
        .s_axis_tdata (selected_reg_data),
        .s_axis_tkeep (selected_reg_keep),
        .s_axis_tvalid(selected_reg_valid),
        .s_axis_tready(selected_reg_ready),
        .s_axis_tlast (selected_reg_last),
        .s_axis_tid   (1'b0),
        .s_axis_tdest (1'b0),
        .s_axis_tuser ({count_keep_bytes(selected_reg_keep),
                        selected_reg_user}),
        .m_axis_tdata (fork_in_data),
        .m_axis_tkeep (fork_in_keep),
        .m_axis_tvalid(fork_in_valid),
        .m_axis_tready(fork_in_ready),
        .m_axis_tlast (fork_in_last),
        .m_axis_tid   (),
        .m_axis_tdest (),
        .m_axis_tuser (fork_in_sideband)
    );

    // Host admission is per frame. Host FIFO pressure never enters DDR READY.
    camera_isp_video_fork #(.FIFO_DEPTH(512),.HOST_FIFO_DEPTH(8192),.HOST_BRANCH(DIRECT_HOST_PRESENT)) video_fork_inst (
        .clk(axi_aclk),.rst(axi_domain_rst),.host_enabled(DIRECT_HOST_PRESENT && host_direct_enabled),
        .host_credit(host_frame_credit),.tail_abort(host_tail_abort),.meta_valid(host_meta_event),
        .meta_take(host_meta_take),.meta_bad(host_meta_bad),
        .s_data(fork_in_data),.s_keep(fork_in_keep),.s_user(fork_in_sideband),
        .s_last(fork_in_last),.s_valid(fork_in_valid),.s_ready(fork_in_ready),
        .d_data(hls_m_axis_rgb_tdata),.d_keep(hls_m_axis_rgb_tkeep),
        .d_user(hls_m_axis_rgb_sideband),.d_last(hls_m_axis_rgb_tlast[0]),
        .d_valid(hls_m_axis_rgb_tvalid),.d_ready(hls_m_axis_rgb_tready),
        .h_data(host_fork_data),.h_keep(host_fork_keep),
        .h_sof(host_fork_sof),.h_last(host_fork_last),
        .h_valid(host_fork_valid),.h_ready(host_stream_ready),
        .h_pending(host_fork_pending),.drained(fork_drained)
    );

    assign hls_m_axis_rgb_tstrb      = hls_m_axis_rgb_tkeep;
    assign hls_m_axis_rgb_tuser[0]   = hls_m_axis_rgb_sideband[0];
    assign hls_m_axis_rgb_beat_bytes = hls_m_axis_rgb_sideband[7:1];

    wire [63:0] rgb_valid_keep = hls_m_axis_rgb_tkeep & hls_m_axis_rgb_tstrb;
    wire [6:0] rgb_beat_bytes = hls_m_axis_rgb_beat_bytes;
    wire [31:0] rgb_beat_bytes_u32 = {25'd0, rgb_beat_bytes};
    wire [31:0] rgb_next_bytes = bytes_seen + rgb_beat_bytes_u32;
    wire [BURST_COUNT_WIDTH-1:0] burst_beats_after = burst_beat_count + 1'b1;
    wire rgb_frame_last = hls_m_axis_rgb_tvalid &&
                           hls_m_axis_rgb_tlast[0];
    // Only six registered address bits enter this comparison. No extra
    // 32/64-bit carry chain or downstream READY path is added.
    wire rgb_page_end = (cap_state == CAP_IDLE) ?
                          (&next_slot_addr[11:6]) : (&next_beat_addr[11:6]);
    wire rgb_burst_full = (burst_beats_after == AXI_BURST_BEATS_L) || rgb_page_end;
    // A full non-final burst needs a command immediately. The final burst is
    // held in the beat FIFO until pixel-domain metadata arrives, then issued
    // as the sole frame-final command carrying the publication metadata.
    wire rgb_need_cmd_now = rgb_burst_full && !rgb_frame_last;
    wire [7:0] rgb_cmd_len =
        {{(8-BURST_COUNT_WIDTH){1'b0}}, burst_beats_after} - 8'd1;
    wire rgb_fire = hls_m_axis_rgb_tvalid && hls_m_axis_rgb_tready;

    // Raw Bayer frames are <=4096x4096. Bits 31/30 carry frame-local
    // sync/loss flags atomically in the existing metadata FIFO, without CDC
    // sampling a historical sticky flag from a different physical frame.
    wire meta_cmlk_sync_bad = meta_loaded && meta_bytes[31];
    wire meta_input_fifo_bad = meta_loaded && meta_bytes[30];
    wire [31:0] meta_payload_bytes = {2'b0, meta_bytes[29:0]};
    wire [31:0] meta_rgb_width = raw_output_axi ? meta_width : meta_width + (meta_width << 1);
    wire [31:0] meta_rgb_bytes = raw_output_axi ? meta_payload_bytes : meta_payload_bytes + (meta_payload_bytes << 1);
    wire [31:0] expected_rgb_width = raw_output_axi ? expected_line_bytes_axi : expected_line_bytes_axi +
                                     (expected_line_bytes_axi << 1);
    wire [31:0] expected_rgb_bytes = raw_output_axi ? expected_frame_bytes_axi : expected_frame_bytes_axi +
                                     (expected_frame_bytes_axi << 1);

    wire meta_too_large = meta_loaded && (meta_rgb_bytes > frame_stride_axi);
    wire meta_line_mismatch = meta_loaded &&
                    (expected_line_bytes_axi != 32'd0) &&
                    (meta_width != expected_line_bytes_axi);
    wire meta_height_mismatch = meta_loaded &&
                    (expected_height_axi != 32'd0) &&
                    (meta_height != expected_height_axi);
    wire meta_frame_mismatch = meta_loaded &&
                    (expected_frame_bytes_axi != 32'd0) &&
                    (meta_payload_bytes != expected_frame_bytes_axi);
    wire meta_reject = meta_cmlk_sync_bad || meta_too_large ||
                       meta_line_mismatch || meta_height_mismatch || meta_frame_mismatch ||
                       meta_input_fifo_bad || output_fifo_fault_reg || meta_overflow_reg ||
                       hls_input_error_reg || hls_cfg_error_reg ||
                       cmlk_align_loss_reg || axi_error_reg;
    wire [31:0] meta_reject_reason = meta_cmlk_sync_bad ? 32'd13 :
                                      meta_too_large ? 32'd2 :
                                      meta_line_mismatch ? 32'd3 :
                                      meta_frame_mismatch ? 32'd4 :
                                      meta_height_mismatch ? 32'd5 :
                                      (meta_input_fifo_bad || output_fifo_fault_reg) ? 32'd6 :
                                      hls_input_error_reg ? 32'd7 :
                                      hls_cfg_error_reg ? 32'd8 :
                                      meta_overflow_reg ? 32'd10 :
                                      axi_error_reg ? 32'd11 :
                                      cmlk_align_loss_reg ? 32'd12 : 32'd0;

    assign uip_drained_axi=fork_drained && (!host_direct_enabled || host_direct_drained) && cap_state==CAP_IDLE && wr_state==WR_IDLE && outstanding_count==0 && !cmd_fifo_valid && !m_axi_awvalid && !m_axi_wvalid;
    wire [1:0] writer_bresp;
    wire writer_bvalid;
    logic writer_bready;
    axis_register #(
        .DATA_WIDTH(2),.KEEP_ENABLE(0),.KEEP_WIDTH(1),.LAST_ENABLE(0),
        .ID_ENABLE(0),.ID_WIDTH(1),.DEST_ENABLE(0),.DEST_WIDTH(1),
        .USER_ENABLE(0),.USER_WIDTH(1),.REG_TYPE(2)
    ) camera_ddr_b_timing_slice (
        .clk(axi_aclk),.rst(axi_domain_rst),
        .s_axis_tdata(m_axi_bresp),.s_axis_tkeep(1'b1),
        .s_axis_tvalid(m_axi_bvalid),.s_axis_tready(m_axi_bready),
        .s_axis_tlast(1'b1),.s_axis_tid(1'b0),.s_axis_tdest(1'b0),.s_axis_tuser(1'b0),
        .m_axis_tdata(writer_bresp),.m_axis_tkeep(),
        .m_axis_tvalid(writer_bvalid),.m_axis_tready(writer_bready),
        .m_axis_tlast(),.m_axis_tid(),.m_axis_tdest(),.m_axis_tuser()
    );
    wire b_fire = writer_bvalid && writer_bready;
    wire aw_fire = m_axi_awvalid && m_axi_awready;
    wire w_fire = m_axi_wvalid && m_axi_wready;
    wire final_cmd_enqueue = (cap_state == CAP_WAIT_META) &&
                             meta_loaded && !cmd_fifo_backpressure;
    wire final_cmd_drop = meta_reject || (bytes_seen != meta_rgb_bytes);
    wire final_cmd_reserve = final_cmd_enqueue && !final_cmd_drop;
    wire final_resp_fire = b_fire && resp_final[0];
    wire final_resp_release = final_resp_fire && !resp_drop[0];
    wire reject_response = final_resp_fire && (resp_drop[0] || writer_bresp != 0);
    wire reject_stream = cap_state == CAP_DROP && cap_enable_axi &&
                         meta_loaded && drop_output_done;
    wire [4:0] reject_response_reason = writer_bresp != 0 ? 5'd11 : resp_reason[0];
    wire cmd_data_ready = (wr_state == WR_IDLE) && cmd_fifo_valid &&
                          (outstanding_count < AXI_MAX_OUTSTANDING_COUNT);
    assign cmd_fifo_rd_en = cmd_data_ready;
    // FWFT FIFOs must be popped in the same cycle in which their current DOUT
    // is captured. Registering RD_EN duplicates the first beat of every burst
    // and discards its final beat.
    wire beat_fifo_pop = (wr_state == WR_W) && beat_fifo_valid &&
                         (!m_axi_wvalid || (w_fire && !m_axi_wlast));
    assign beat_fifo_rd_en = beat_fifo_pop;
    wire [OUTSTANDING_COUNT_WIDTH-1:0] resp_push_index =
                    outstanding_count - (b_fire ? {{(OUTSTANDING_COUNT_WIDTH-1){1'b0}}, 1'b1} :
                                                   {OUTSTANDING_COUNT_WIDTH{1'b0}});

    logic ddr_stream_ready;
    always_comb begin
        ddr_stream_ready = 1'b0;
        case (cap_state)
            CAP_IDLE: begin
                if (hls_cfg_done && cfg_static_ok && ring_decision_valid) begin
                    if (ring_full)
                        ddr_stream_ready = 1'b1;
                    else if (!beat_fifo_backpressure &&
                             (!rgb_need_cmd_now || !cmd_fifo_backpressure))
                        ddr_stream_ready = 1'b1;
                end
            end
            CAP_COLLECT: begin
                if (!beat_fifo_backpressure &&
                    (!rgb_need_cmd_now || !cmd_fifo_backpressure))
                    ddr_stream_ready = 1'b1;
            end
            CAP_DROP: ddr_stream_ready = !drop_output_done;
            default: ddr_stream_ready = 1'b0;
        endcase
    end

    assign hls_m_axis_rgb_tready=ddr_stream_ready;

    xpm_fifo_sync #(
        .FIFO_MEMORY_TYPE    ("block"),
        .FIFO_WRITE_DEPTH    (AXI_BURST_FIFO_DEPTH),
        // FULL is too late for the registered WR_EN. Allow flag latency
        // and the already accepted write before stopping the stream.
        .PROG_FULL_THRESH    (AXI_BURST_FIFO_DEPTH-8),
        .WRITE_DATA_WIDTH    (BEAT_FIFO_WIDTH),
        .READ_DATA_WIDTH     (BEAT_FIFO_WIDTH),
        .READ_MODE           ("fwft"),
        .FIFO_READ_LATENCY   (0),
        .DOUT_RESET_VALUE    ("0"),
        .ECC_MODE            ("no_ecc"),
        .USE_ADV_FEATURES    ("1102")
    ) beat_fifo_inst (
        .rst          (axi_domain_rst),
        .wr_clk       (axi_aclk),
        .din          (beat_fifo_din),
        .wr_en        (beat_fifo_wr_en),
        .rd_en        (beat_fifo_rd_en),
        .dout         (beat_fifo_dout),
        .full         (beat_fifo_full),
        .empty        (),
        .data_valid   (beat_fifo_valid),
        .overflow     (beat_fifo_overflow_event),
        .underflow    (),
        .wr_rst_busy  (),
        .rd_rst_busy  (),
        .sleep        (1'b0),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .wr_data_count(),
        .rd_data_count(),
        .prog_full    (beat_fifo_backpressure),
        .prog_empty   (),
        .almost_full  (),
        .almost_empty (),
        .wr_ack       ()
    );

    xpm_fifo_sync #(
        .FIFO_MEMORY_TYPE    ("distributed"),
        .FIFO_WRITE_DEPTH    (AXI_CMD_FIFO_DEPTH),
        // FULL is too late for the registered WR_EN. Allow flag latency
        // and the already accepted write before stopping the stream.
        .PROG_FULL_THRESH    (AXI_CMD_FIFO_DEPTH-8),
        .WRITE_DATA_WIDTH    (CMD_FIFO_WIDTH),
        .READ_DATA_WIDTH     (CMD_FIFO_WIDTH),
        .READ_MODE           ("fwft"),
        .FIFO_READ_LATENCY   (0),
        .DOUT_RESET_VALUE    ("0"),
        .ECC_MODE            ("no_ecc"),
        .USE_ADV_FEATURES    ("1102")
    ) cmd_fifo_inst (
        .rst          (axi_domain_rst),
        .wr_clk       (axi_aclk),
        .din          (cmd_fifo_din),
        .wr_en        (cmd_fifo_wr_en),
        .rd_en        (cmd_fifo_rd_en),
        .dout         (cmd_fifo_dout),
        .full         (cmd_fifo_full),
        .empty        (),
        .data_valid   (cmd_fifo_valid),
        .overflow     (cmd_fifo_overflow_event),
        .underflow    (),
        .wr_rst_busy  (),
        .rd_rst_busy  (),
        .sleep        (1'b0),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .wr_data_count(),
        .rd_data_count(),
        .prog_full    (cmd_fifo_backpressure),
        .prog_empty   (),
        .almost_full  (),
        .almost_empty (),
        .wr_ack       ()
    );

    always_comb begin
        meta_fifo_rd_en = 1'b0;
        if ((cap_state == CAP_COLLECT || cap_state == CAP_DROP ||
             cap_state == CAP_WAIT_META) && !meta_loaded && meta_fifo_valid)
            meta_fifo_rd_en = 1'b1;
    end

    always_ff @(posedge axi_aclk) begin
        integer i;
        if (axi_domain_rst) begin
            cap_state        <= CAP_IDLE;
            wr_state         <= WR_IDLE;
            beat_fifo_wr_en  <= 1'b0;
            cmd_fifo_wr_en   <= 1'b0;
            m_axi_awvalid    <= 1'b0;
            m_axi_wvalid     <= 1'b0;
            writer_bready     <= 1'b0;
            m_axi_awaddr     <= '0;
            m_axi_awlen      <= 8'd0;
            m_axi_wdata      <= '0;
            m_axi_wstrb      <= '0;
            m_axi_wlast      <= 1'b1;
            bytes_seen       <= 32'd0;
            meta_loaded      <= 1'b0;
            meta_seq         <= 32'd0;
            meta_width       <= 32'd0;
            meta_height      <= 32'd0;
            meta_bytes       <= 32'd0;
            drop_bytes_seen  <= 32'd0;
            drop_output_done <= 1'b0;
            stream_drop_reason <= 5'd0;
            next_slot_addr   <= '0;
            next_slot_pos    <= 5'd0;
            burst_start_addr <= '0;
            next_beat_addr   <= '0;
            burst_beat_count <= '0;
            final_burst_addr <= '0;
            final_burst_len  <= 8'd0;
            reserved_frame_count <= 6'd0;
            producer_idx_reg <= 32'd0;
            frame_width_reg  <= 32'd0;
            frame_height_reg <= 32'd0;
            frame_bytes_reg  <= 32'd0;
            frame_seq_reg    <= 32'd0;
            drop_count_reg   <= 32'd0;
            fifo_overflow_reg <= 1'b0;
            output_fifo_fault_reg <= 1'b0;
            meta_overflow_reg <= 1'b0;
            axi_error_reg     <= 1'b0;
            cfg_error_reg     <= 1'b0;
            frame_size_error_reg <= 1'b0;
            drop_frame_bytes_reg <= 32'd0;
            drop_reason_reg      <= 32'd0;
            duplicate_commit_count_reg <= 32'd0;
            hls_stall_cycles_reg <= 32'd0;
            axi_stall_cycles_reg <= 32'd0;
            frame_seq_valid_reg  <= 1'b0;
            cmlk_align_loss_reg <= 1'b0;
            cmlk_align_loss_baseline_reg <= 32'd0;
            active_cmd_addr   <= '0;
            active_cmd_len    <= 8'd0;
            active_cmd_final  <= 1'b0;
            active_cmd_drop   <= 1'b0;
            active_cmd_reason <= 5'd0;
            active_cmd_seq    <= 32'd0;
            active_cmd_width  <= 32'd0;
            active_cmd_height <= 32'd0;
            active_cmd_bytes  <= 32'd0;
            write_beat_idx    <= 8'd0;
            outstanding_count <= '0;
            resp_final        <= '0;
            resp_drop         <= '0;
            beat_fifo_din     <= '0;
            cmd_fifo_din      <= '0;
            for (i = 0; i < AXI_MAX_OUTSTANDING; i = i + 1) begin
                resp_seq[i]    <= 32'd0;
                resp_width[i]  <= 32'd0;
                resp_height[i] <= 32'd0;
                resp_bytes[i]  <= 32'd0;
                resp_reason[i] <= 5'd0;
            end
        end else begin
            beat_fifo_wr_en <= 1'b0;
            cmd_fifo_wr_en  <= 1'b0;
            // One AXI ID is used and BRESPs are consumed in issue order.
            writer_bready    <= 1'b1;

            if (soft_clear_axi) begin
                fifo_overflow_reg <= 1'b0;
                output_fifo_fault_reg <= 1'b0;
                meta_overflow_reg <= 1'b0;
                axi_error_reg     <= 1'b0;
                cfg_error_reg     <= 1'b0;
                frame_size_error_reg <= 1'b0;
                drop_frame_bytes_reg <= 32'd0;
                drop_reason_reg      <= 32'd0;
                drop_count_reg    <= 32'd0;
                duplicate_commit_count_reg <= 32'd0;
                hls_stall_cycles_reg <= 32'd0;
                axi_stall_cycles_reg <= 32'd0;
                cmlk_align_loss_reg <= 1'b0;
                cmlk_align_loss_baseline_reg <= cmlk_align_loss_count_axi;
            end else begin
                // CLEAR_STATUS remains asserted across the pixel flag CDC;
                // outside that interval retain errors from both clock domains.
                // Stream FIFOs have backpressure headroom; if any unexpected
                // overflow still occurs, never silently publish shifted pixels.
                fifo_overflow_reg <= fifo_overflow_reg ||
                                     data_fifo_overflow_axi ||
                                     beat_fifo_overflow_event;
                // Unexpected DDR beat loss has no recoverable byte boundary.
                // Keep it fatal; only Camera Link input loss is frame-local.
                output_fifo_fault_reg <= output_fifo_fault_reg ||
                                         beat_fifo_overflow_event;
                meta_overflow_reg <= meta_overflow_reg ||
                                     meta_fifo_overflow_axi || cmd_fifo_overflow_event;
                if (cap_enable_axi && hls_cfg_wait_count == 4'hf && !cfg_static_ok)
                    cfg_error_reg <= 1'b1;
            end

            // Baseline while disabled so an event from a previous session is
            // diagnostic only. During capture, any new alignment loss is sticky
            // and makes the affected frame fail the existing strict guard.
            if (!cap_enable_axi) begin
                cmlk_align_loss_reg <= 1'b0;
                cmlk_align_loss_baseline_reg <= cmlk_align_loss_count_axi;
            end else if (!soft_clear_axi &&
                         cmlk_align_loss_count_axi !=
                         cmlk_align_loss_baseline_reg) begin
                cmlk_align_loss_reg <= 1'b1;
            end

            if (!soft_clear_axi && (reject_response || reject_stream))
                drop_count_reg <= drop_count_reg + {31'd0,reject_response} +
                                  {31'd0,reject_stream};

            // These counters identify downstream backpressure without changing
            // any FIFO or ring depth. They are diagnostic only and intentionally
            // saturate rather than wrap. CLEAR_STATUS wins over counting.
            if (!soft_clear_axi) begin
                if (cap_enable_axi && hls_m_axis_rgb_tvalid &&
                    !hls_m_axis_rgb_tready && hls_stall_cycles_reg != 32'hffff_ffff)
                    hls_stall_cycles_reg <= hls_stall_cycles_reg + 1'b1;
                // Count only actual channel backpressure. Waiting for a
                // BRESP while useful W traffic is still progressing is normal
                // AXI latency and must not be reported as a stall.
                if (((m_axi_awvalid && !m_axi_awready) ||
                     (m_axi_wvalid && !m_axi_wready)) &&
                    axi_stall_cycles_reg != 32'hffff_ffff)
                    axi_stall_cycles_reg <= axi_stall_cycles_reg + 1'b1;
            end

            if (meta_fifo_rd_en) begin
                {meta_seq, meta_width, meta_height, meta_bytes} <= meta_fifo_dout;
                meta_loaded <= 1'b1;
            end

            if (b_fire) begin
                if (writer_bresp != 2'b00)
                    axi_error_reg <= 1'b1;

                // All writes use one AXI ID, so BRESP order is the AW issue
                // order. The response associated with the frame-final burst is
                // the publication fence for that DDR slot. This removes the
                // frame-wide "wait for all BRESPs" barrier that previously
                // stalled the HLS output between every pair of frames.
                if (resp_final[0]) begin
                    if ((writer_bresp != 2'b00) || resp_drop[0]) begin
                        // Counted centrally: response and stream rejections can coincide.
                        drop_frame_bytes_reg <= resp_bytes[0];
                        drop_reason_reg <= {27'd0,reject_response_reason};
                    end else if (frame_seq_valid_reg &&
                                 (resp_seq[0] == frame_seq_reg)) begin
                        if (duplicate_commit_count_reg != 32'hffff_ffff)
                            duplicate_commit_count_reg <=
                                duplicate_commit_count_reg + 1'b1;
                    end else begin
                        frame_width_reg  <= resp_width[0];
                        frame_height_reg <= resp_height[0];
                        frame_bytes_reg  <= resp_bytes[0];
                        frame_seq_reg    <= resp_seq[0];
                        frame_seq_valid_reg <= 1'b1;
                        producer_idx_reg <= producer_idx_reg + 1'b1;
                    end
                end

                for (i = 0; i < AXI_MAX_OUTSTANDING-1; i = i + 1) begin
                    resp_final[i] <= resp_final[i+1];
                    resp_drop[i] <= resp_drop[i+1];
                    resp_reason[i] <= resp_reason[i+1];
                    resp_seq[i] <= resp_seq[i+1];
                    resp_width[i] <= resp_width[i+1];
                    resp_height[i] <= resp_height[i+1];
                    resp_bytes[i] <= resp_bytes[i+1];
                end
                resp_final[AXI_MAX_OUTSTANDING-1] <= 1'b0;
                resp_drop[AXI_MAX_OUTSTANDING-1] <= 1'b0;
                resp_reason[AXI_MAX_OUTSTANDING-1] <= 5'd0;
            end

            case (cap_state)
                CAP_IDLE: begin
                    bytes_seen       <= 32'd0;
                    drop_bytes_seen  <= 32'd0;
                    drop_output_done <= 1'b0;
                    meta_loaded      <= 1'b0;
                    burst_beat_count <= '0;
                    if (!cfg_static_ok || !hls_cfg_done) begin
                        next_slot_addr <= ring_base_axi;
                        next_slot_pos  <= 5'd0;
                    end else if (rgb_fire) begin
                        if (!hls_m_axis_rgb_tuser[0] || ring_full ||
                            (rgb_beat_bytes == 0)) begin
                            if (!hls_m_axis_rgb_tuser[0] ||
                                (rgb_beat_bytes == 0))
                                frame_size_error_reg <= 1'b1;
                            drop_bytes_seen  <= rgb_beat_bytes_u32;
                            drop_output_done <= hls_m_axis_rgb_tlast[0];
                            drop_reason_reg  <= ring_full ? 32'd16 : 32'd18;
                            stream_drop_reason <= ring_full ? 5'd16 : 5'd18;
                            cap_state        <= CAP_DROP;
                        end else begin
                            beat_fifo_din   <= {rgb_valid_keep,
                                                hls_m_axis_rgb_tdata};
                            beat_fifo_wr_en <= 1'b1;
                            bytes_seen      <= rgb_beat_bytes_u32;
                            burst_start_addr <= next_slot_addr;
                            next_beat_addr   <= next_slot_addr +
                                                AXI_KEEP_WIDTH_ADDR;

                            if (rgb_frame_last) begin
                                final_burst_addr <= next_slot_addr;
                                final_burst_len  <= rgb_cmd_len;
                                burst_beat_count <= burst_beats_after;
                                cap_state <= CAP_WAIT_META;
                            end else if (rgb_need_cmd_now) begin
                                cmd_fifo_din <= {
                                    5'd0, 1'b0, 1'b0,
                                    32'd0, expected_rgb_width,
                                    expected_height_axi, expected_rgb_bytes,
                                    next_slot_addr, rgb_cmd_len
                                };
                                cmd_fifo_wr_en   <= 1'b1;
                                burst_beat_count <= '0;
                                burst_start_addr <= next_slot_addr +
                                                    AXI_KEEP_WIDTH_ADDR;
                                cap_state <= CAP_COLLECT;
                            end else begin
                                burst_beat_count <=
                                    {{(BURST_COUNT_WIDTH-1){1'b0}}, 1'b1};
                                cap_state <= CAP_COLLECT;
                            end
                        end
                    end
                end

                CAP_COLLECT: begin
                    if (rgb_fire) begin
                        if ((rgb_beat_bytes == 0) ||
                            (rgb_next_bytes > frame_stride_axi)) begin
                            frame_size_error_reg <= 1'b1;
                            drop_frame_bytes_reg <= rgb_next_bytes;
                            drop_reason_reg <= (rgb_beat_bytes == 0) ?
                                               32'd18 : 32'd1;
                            stream_drop_reason <= (rgb_beat_bytes == 0) ? 5'd18 : 5'd1;
                            drop_bytes_seen  <= rgb_next_bytes;
                            drop_output_done <= hls_m_axis_rgb_tlast[0];
                            cap_state        <= CAP_DROP;
                        end else begin
                            beat_fifo_din   <= {rgb_valid_keep,
                                                hls_m_axis_rgb_tdata};
                            beat_fifo_wr_en <= 1'b1;
                            bytes_seen      <= rgb_next_bytes;
                            next_beat_addr  <= next_beat_addr +
                                                AXI_KEEP_WIDTH_ADDR;

                            if (rgb_frame_last) begin
                                final_burst_addr <= burst_start_addr;
                                final_burst_len  <= rgb_cmd_len;
                                burst_beat_count <= burst_beats_after;
                                cap_state <= CAP_WAIT_META;
                            end else if (rgb_need_cmd_now) begin
                                cmd_fifo_din <= {
                                    5'd0, 1'b0, 1'b0,
                                    32'd0, expected_rgb_width,
                                    expected_height_axi, expected_rgb_bytes,
                                    burst_start_addr, rgb_cmd_len
                                };
                                cmd_fifo_wr_en   <= 1'b1;
                                burst_beat_count <= '0;
                                burst_start_addr <= next_beat_addr +
                                                    AXI_KEEP_WIDTH_ADDR;
                            end else begin
                                burst_beat_count <= burst_beats_after;
                            end
                        end
                    end
                end

                CAP_WAIT_META: begin
                    // The final burst remains in beat_fifo until the matching
                    // pixel-domain frame metadata is available. It is then
                    // issued as a normal AXI write command with final=1.
                    // Publication occurs on this burst's BRESP, so the next
                    // frame can enter the pipeline without a frame-wide gap.
                    if (final_cmd_enqueue) begin
                        cmd_fifo_din <= {
                            (meta_reject ? meta_reject_reason[4:0] : 5'd9),
                            final_cmd_drop, 1'b1,
                            meta_seq, meta_rgb_width,
                            meta_height, meta_rgb_bytes,
                            final_burst_addr, final_burst_len
                        };
                        cmd_fifo_wr_en <= 1'b1;

                        if (final_cmd_drop) begin
                            frame_size_error_reg <= 1'b1;
                            drop_frame_bytes_reg <= bytes_seen;
                            drop_reason_reg <= meta_reject ?
                                               meta_reject_reason : 32'd9;
                        end else if (next_slot_pos == 5'd31) begin
                            next_slot_pos  <= 5'd0;
                            next_slot_addr <= ring_base_axi;
                        end else begin
                            next_slot_pos  <= next_slot_pos + 1'b1;
                            next_slot_addr <= next_slot_addr +
                                              frame_stride_axi[
                                                  AXI_ADDR_WIDTH-1:0];
                        end
                        cap_state <= CAP_IDLE;
                    end
                end

                CAP_DROP: begin
                    if (!cap_enable_axi) begin
                        cap_state <= CAP_IDLE;
                    end else begin
                        if (rgb_fire) begin
                            drop_bytes_seen <= drop_bytes_seen +
                                               rgb_beat_bytes_u32;
                            if (hls_m_axis_rgb_tlast[0])
                                drop_output_done <= 1'b1;
                        end
                        if (meta_loaded && drop_output_done) begin
                            // Counted centrally: response and stream rejections can coincide.
                            drop_frame_bytes_reg <= drop_bytes_seen;
                            cap_state <= CAP_IDLE;
                        end
                    end
                end

                default: cap_state <= CAP_IDLE;
            endcase

            case (wr_state)
                WR_IDLE: begin
                    if (cmd_data_ready) begin
                        active_cmd_addr   <= cmd_fifo_addr;
                        active_cmd_len    <= cmd_fifo_len;
                        active_cmd_final  <= cmd_fifo_final;
                        active_cmd_drop   <= cmd_fifo_drop;
                        active_cmd_reason <= cmd_fifo_reason;
                        active_cmd_seq    <= cmd_fifo_meta[127:96];
                        active_cmd_width  <= cmd_fifo_meta[95:64];
                        active_cmd_height <= cmd_fifo_meta[63:32];
                        active_cmd_bytes  <= cmd_fifo_meta[31:0];
                        m_axi_awaddr      <= cmd_fifo_addr;
                        m_axi_awlen       <= cmd_fifo_len;
                        m_axi_awvalid     <= 1'b1;
                        wr_state          <= WR_AW;
                    end
                end

                WR_AW: begin
                    if (aw_fire) begin
                        m_axi_awvalid <= 1'b0;
                        resp_final[resp_push_index] <= active_cmd_final;
                        resp_drop[resp_push_index] <= active_cmd_drop;
                        resp_reason[resp_push_index] <= active_cmd_reason;
                        resp_seq[resp_push_index] <= active_cmd_seq;
                        resp_width[resp_push_index] <= active_cmd_width;
                        resp_height[resp_push_index] <= active_cmd_height;
                        resp_bytes[resp_push_index] <= active_cmd_bytes;
                        write_beat_idx <= 8'd0;
                        wr_state <= WR_W;
                    end
                end

                WR_W: begin
                    // beat_fifo is FWFT. beat_fifo_pop is combinational and
                    // asserted in the exact cycle its current DOUT is captured.
                    // This permits one accepted W beat per clock without
                    // duplicating the first beat or discarding the last beat.
                    if (w_fire && m_axi_wlast) begin
                        m_axi_wvalid   <= 1'b0;
                        m_axi_wlast    <= 1'b0;
                        write_beat_idx <= 8'd0;
                        wr_state       <= WR_IDLE;
                    end else if (beat_fifo_pop) begin
                        m_axi_wdata  <= beat_fifo_data;
                        m_axi_wstrb  <= beat_fifo_strb;
                        m_axi_wlast  <= (write_beat_idx == active_cmd_len);
                        m_axi_wvalid <= 1'b1;
                        if (write_beat_idx != active_cmd_len)
                            write_beat_idx <= write_beat_idx + 1'b1;
                    end else if (w_fire) begin
                        // The current non-final beat was accepted but the next
                        // FIFO word is not available yet. Drop WVALID until the
                        // FWFT output becomes valid; data remains stable during
                        // every WREADY stall.
                        m_axi_wvalid <= 1'b0;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase

            case ({final_cmd_reserve, final_resp_release})
                2'b10: begin
                    if (reserved_frame_count < 6'd32)
                        reserved_frame_count <= reserved_frame_count + 1'b1;
                    else
                        cfg_error_reg <= 1'b1;
                end
                2'b01: begin
                    if (reserved_frame_count != 0)
                        reserved_frame_count <= reserved_frame_count - 1'b1;
                    else
                        cfg_error_reg <= 1'b1;
                end
                default: begin end
            endcase

            if (aw_fire || b_fire)
                outstanding_count <= outstanding_count +
                                     (aw_fire ? {{(OUTSTANDING_COUNT_WIDTH-1){1'b0}}, 1'b1} :
                                                {OUTSTANDING_COUNT_WIDTH{1'b0}}) -
                                     (b_fire ? {{(OUTSTANDING_COUNT_WIDTH-1){1'b0}}, 1'b1} :
                                               {OUTSTANDING_COUNT_WIDTH{1'b0}});
        end
    end

    assign m_axi_awsize = 3'd6;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awlock = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot = 3'b000;
    assign m_axi_awqos = 4'd0;
    assign m_axi_awregion = 4'd0;
    assign m_axi_awid = '0;

    wire host_meta_event=final_cmd_enqueue ||
        (cap_state==CAP_DROP && meta_loaded && drop_output_done && cap_enable_axi);
    wire [31:0] host_meta_flags=(meta_reject || host_meta_bad ? 32'd0 : 32'd1) |
        ((final_cmd_enqueue && !final_cmd_drop) ? 32'd2 : 32'd0);
    wire [31:0] host_ddr_index=producer_idx_reg+{26'd0,reserved_frame_count};
    generate if (DIRECT_HOST_PRESENT) begin : g_direct_host
    // The upstream host fork now absorbs 512 KiB of transient PCIe stalls in
    // UltraRAM; this downstream FIFO remains shallow for the AXI burst writer.
    // DDR and host frame admission remain independent.
    camera_direct_host_writer #(.FIFO_DEPTH(256),.DEMAND_GATED(1'b0)) host_direct_inst (
        .clk(axi_aclk),.rst(axi_domain_rst),
        .axil_clk(axil_aclk),.axil_rstn(axil_aresetn),.cap_enabled(cap_enable),
        .stop_ack(frame_stop_ack_axi),.tail_abort(host_tail_abort),
        .csr_wr(s_axil_wready&&s_axil_wvalid),.csr_addr(axil_write_addr[11:0]),
        .csr_data(s_axil_wdata),.csr_rd_addr(s_axil_araddr[11:0]),.csr_rdata(host_csr_rdata),
        .frame_bytes(expected_rgb_bytes),.width_pixels(expected_line_bytes_axi),
        .height_pixels(expected_height_axi),.pixel_format(raw_output_axi ? 32'h42474752 : 32'h33424752),
        .s_data(host_fork_data),.s_keep(host_fork_keep),
        .s_valid(host_fork_valid && host_stream_ready),
        .s_last(host_fork_last),.s_sof(host_fork_sof),
        .s_pending(host_fork_pending),
        .stream_ready(host_stream_ready),.enabled(host_direct_enabled),.drained_out(host_direct_drained),
        .frame_credit(host_frame_credit),
        .meta_valid(host_meta_event && host_meta_take),.meta_data({meta_seq,meta_width,meta_height,meta_payload_bytes}),
        .meta_status(host_meta_flags),.meta_ddr_idx(host_ddr_index),
        .awaddr(m_host_awaddr),.awlen(m_host_awlen),.awvalid(m_host_awvalid),.awready(m_host_awready),
        .wdata(m_host_wdata),.wstrb(m_host_wstrb),.wlast(m_host_wlast),.wvalid(m_host_wvalid),
        .wready(m_host_wready),.bresp(m_host_bresp),.bvalid(m_host_bvalid),.bready(m_host_bready)
    );
    end else begin : g_ddr_host
        assign host_direct_enabled = 1'b0;
        assign host_stream_ready = 1'b1;
        assign host_direct_drained = 1'b1;
        assign host_tail_abort = 1'b0;
        assign host_frame_credit = 1'b0;
        assign host_csr_rdata = 32'd0;
        assign m_host_awaddr = 64'd0;
        assign m_host_awlen = 8'd0;
        assign m_host_awvalid = 1'b0;
        assign m_host_wdata = 512'd0;
        assign m_host_wstrb = 64'd0;
        assign m_host_wlast = 1'b0;
        assign m_host_wvalid = 1'b0;
        assign m_host_bready = 1'b1;
    end endgenerate
    // Rejection reasons travel with outstanding frames. Counting the last
    // reason register would misattribute a response to a following frame.
    logic [31:0] drop_hist [0:18];
    genvar dh;
    generate for(dh=0;dh<19;dh=dh+1)begin:drop_hist_pack
        assign drop_hist_axi[dh*32 +:32]=drop_hist[dh];
    end endgenerate
    camera_status_snapshot_cdc #(.WIDTH(608)) drop_hist_cdc (.src_clk(axi_aclk),.src_in(drop_hist_axi),
                      .dest_clk(axil_aclk),.dest_out(drop_hist_axil));
    integer hist_i;
    always_ff @(posedge axi_aclk)begin
        if(axi_domain_rst || soft_clear_axi)begin
            for(hist_i=0;hist_i<19;hist_i=hist_i+1)drop_hist[hist_i]<=0;
        end else begin
            for(hist_i=0;hist_i<19;hist_i=hist_i+1)begin
                if((reject_response && reject_response_reason==hist_i) ||
                   (reject_stream && stream_drop_reason==hist_i))
                    drop_hist[hist_i]<=drop_hist[hist_i] +
                        {31'd0,(reject_response && reject_response_reason==hist_i)} +
                        {31'd0,(reject_stream && stream_drop_reason==hist_i)};
            end
        end
    end
    // Stable per-DDR-slot frame IDs, needed when host and video finish independently.
    logic [31:0] ddr_frame_ids [0:31];
    genvar di;
    generate for(di=0;di<32;di=di+1)begin:frame_id_pack
        assign ddr_frame_ids_axi[di*32 +:32]=ddr_frame_ids[di];
    end endgenerate
    // Per-slot frame IDs are published in the same snapshot as producer_idx
    // by status_cdc_inst; independent mailboxes could expose ownership first.
    // Decode the producer slot ahead of the wide frame-ID write enable.
    // Publication and frame ID still occur together on a valid final BRESP.
    wire ddr_id_commit = final_resp_release && writer_bresp==0 &&
                        !(frame_seq_valid_reg && resp_seq[0]==frame_seq_reg);
    logic [31:0] ddr_id_slot_onehot = 32'd1;
    always_ff @(posedge axi_aclk) begin
        if (axi_domain_rst) ddr_id_slot_onehot <= 32'd1;
        else if (ddr_id_commit)
            ddr_id_slot_onehot <= {ddr_id_slot_onehot[30:0],ddr_id_slot_onehot[31]};
    end
    generate for (di=0;di<32;di=di+1) begin:frame_id_write
        always_ff @(posedge axi_aclk) begin
            if (axi_domain_rst) ddr_frame_ids[di] <= 0;
            else if (ddr_id_commit && ddr_id_slot_onehot[di])
                ddr_frame_ids[di] <= resp_seq[0];
        end
    end endgenerate


`ifndef SYNTHESIS
    initial begin
        if (AXI_DATA_WIDTH != 512)
            $fatal(1, "RGB capture requires AXI_DATA_WIDTH=512");
        if (AXI_BURST_BEATS < 1 || AXI_BURST_BEATS > 256)
            $fatal(1, "AXI_BURST_BEATS must be in 1..256");
        if (FIFO_DEPTH < 16 || (FIFO_DEPTH & (FIFO_DEPTH-1)) != 0)
            $fatal(1, "Camera input FIFO depth must be a power of two");
        if (AXI_BURST_FIFO_DEPTH <= AXI_BURST_BEATS+8)
            $fatal(1, "beat FIFO must hold a full burst plus flag/WR_EN margin");
        if (AXI_MAX_OUTSTANDING < 1)
            $fatal(1, "AXI_MAX_OUTSTANDING must be positive");
    end
`endif

endmodule

// Standard-read FIFO to a backpressure-safe stream. Each issued RAM read
// reserves one entry until the stream consumes its response.
module camera_input_fifo_read_adapter #(
    parameter integer DATA_WIDTH=91,
    parameter integer DEPTH=8
) (
    input wire clk,rst,
    input wire fifo_busy,fifo_empty,
    output wire fifo_rd_en,
    input wire [DATA_WIDTH-1:0] fifo_data,
    input wire fifo_data_valid,
    output wire [DATA_WIDTH-1:0] m_axis_data,
    output wire m_axis_valid,
    input wire m_axis_ready,
    output wire pending_out
);
    localparam integer PTR_WIDTH=$clog2(DEPTH);
    localparam integer COUNT_WIDTH=$clog2(DEPTH+1);
    localparam [COUNT_WIDTH-1:0] CAPACITY=DEPTH;
    (* ram_style="distributed" *) reg [DATA_WIDTH-1:0] payload [0:DEPTH-1];
    reg [PTR_WIDTH-1:0] write_ptr=0,read_ptr=0;
    reg [COUNT_WIDTH-1:0] reserved=0,queued=0;
    wire reset_active=rst || fifo_busy;
    wire returned=fifo_data_valid && !reset_active;
    wire consumed=m_axis_valid && m_axis_ready;
    assign fifo_rd_en=!reset_active && !fifo_empty && reserved<CAPACITY;
    assign m_axis_valid=!reset_active && queued!=0;
    assign m_axis_data=payload[read_ptr];
    assign pending_out=(reserved!=0);
    always @(posedge clk) begin
        // Valid/credits define ownership, not the contents of inactive RAM.
        if (fifo_data_valid) payload[write_ptr] <= fifo_data;
    end
    always @(posedge clk) begin
        if (reset_active) begin
            write_ptr<=0;read_ptr<=0;reserved<=0;queued<=0;
        end else begin
            if (returned) write_ptr<=write_ptr+1'b1;
            if (consumed) read_ptr<=read_ptr+1'b1;
            case ({fifo_rd_en,consumed})
                2'b10: reserved<=reserved+1'b1;
                2'b01: reserved<=reserved-1'b1;
                default: begin end
            endcase
            case ({returned,consumed})
                2'b10: queued<=queued+1'b1;
                2'b01: queued<=queued-1'b1;
                default: begin end
            endcase
        end
    end
    initial begin
        if (DATA_WIDTH<1 || DEPTH<8 || (DEPTH & (DEPTH-1))!=0)
            $fatal(1,"camera input adapter needs power-of-two DEPTH >= 8");
    end
`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (!reset_active) begin
            if (reserved>CAPACITY || queued>reserved)
                $fatal(1,"camera FIFO response-credit invariant");
            if (returned && reserved==0)
                $fatal(1,"camera FIFO return without reserved read");
            if (returned && queued==CAPACITY && !consumed)
                $fatal(1,"camera FIFO response queue overflow");
        end
    end
`endif
endmodule



// Packs groups of eight RGB888 pixels (24 bytes) into contiguous 64-byte AXI
// beats. Collection and emission are deliberately separated: eight input
// groups take eight clocks and emit in three clocks, for 181.8 Mgroup/s at
// 250 MHz. This is above the 87.2 Mgroup/s required by 2432x2048 at 140 fps,
// while avoiding a 1536-bit variable barrel shifter on the HLS TREADY path.
module camera_rgb192_to_axis512 (
    input  wire         clk,
    input  wire         rst,
    input  wire [191:0] s_axis_tdata,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire         s_axis_tuser,
    input  wire         s_axis_tlast,
    output wire [511:0] m_axis_tdata,
    output wire [63:0]  m_axis_tkeep,
    output wire         m_axis_tvalid,
    input  wire         m_axis_tready,
    output wire         m_axis_tuser,
    output wire         m_axis_tlast
);
    logic [1535:0] data_buffer;
    logic [7:0] bytes_remaining;
    logic [2:0] group_index;
    logic       emitting;
    logic       batch_last;
    logic       batch_user;
    logic       first_output;

    function automatic [7:0] bytes_for_groups(input logic [2:0] index);
        begin
            case (index)
                3'd0: bytes_for_groups = 8'd24;
                3'd1: bytes_for_groups = 8'd48;
                3'd2: bytes_for_groups = 8'd72;
                3'd3: bytes_for_groups = 8'd96;
                3'd4: bytes_for_groups = 8'd120;
                3'd5: bytes_for_groups = 8'd144;
                3'd6: bytes_for_groups = 8'd168;
                default: bytes_for_groups = 8'd192;
            endcase
        end
    endfunction

    function automatic [63:0] keep_for_remaining(input logic [7:0] count);
        begin
            case (count)
                8'd8:  keep_for_remaining = 64'h00000000000000ff;
                8'd16: keep_for_remaining = 64'h000000000000ffff;
                8'd24: keep_for_remaining = 64'h0000000000ffffff;
                8'd32: keep_for_remaining = 64'h00000000ffffffff;
                8'd40: keep_for_remaining = 64'h000000ffffffffff;
                8'd48: keep_for_remaining = 64'h0000ffffffffffff;
                8'd56: keep_for_remaining = 64'h00ffffffffffffff;
                default: keep_for_remaining = 64'hffffffffffffffff;
            endcase
        end
    endfunction

    assign s_axis_tready = ~emitting;
    assign m_axis_tdata  = data_buffer[511:0];
    assign m_axis_tkeep  = keep_for_remaining(bytes_remaining);
    assign m_axis_tvalid = emitting;
    assign m_axis_tuser  = emitting && first_output && batch_user;
    assign m_axis_tlast  = emitting && batch_last &&
                           (bytes_remaining <= 8'd64);

    always_ff @(posedge clk) begin
        if (rst) begin
            data_buffer     <= 1536'd0;
            bytes_remaining <= 8'd0;
            group_index     <= 3'd0;
            emitting        <= 1'b0;
            batch_last      <= 1'b0;
            batch_user      <= 1'b0;
            first_output    <= 1'b0;
        end else if (!emitting) begin
            if (s_axis_tvalid && s_axis_tready) begin
                case (s_axis_tuser ? 3'd0 : group_index)
                    3'd0: data_buffer[191:0]      <= s_axis_tdata;
                    3'd1: data_buffer[383:192]    <= s_axis_tdata;
                    3'd2: data_buffer[575:384]    <= s_axis_tdata;
                    3'd3: data_buffer[767:576]    <= s_axis_tdata;
                    3'd4: data_buffer[959:768]    <= s_axis_tdata;
                    3'd5: data_buffer[1151:960]   <= s_axis_tdata;
                    3'd6: data_buffer[1343:1152]  <= s_axis_tdata;
                    default: data_buffer[1535:1344] <= s_axis_tdata;
                endcase

                if (s_axis_tuser || (group_index == 3'd0))
                    batch_user <= s_axis_tuser;

                if (s_axis_tlast ||
                    (!s_axis_tuser && (group_index == 3'd7))) begin
                    bytes_remaining <= bytes_for_groups(
                        s_axis_tuser ? 3'd0 : group_index);
                    group_index  <= 3'd0;
                    emitting     <= 1'b1;
                    batch_last   <= s_axis_tlast;
                    first_output <= 1'b1;
                end else begin
                    group_index <= s_axis_tuser ? 3'd1 : group_index + 1'b1;
                end
            end
        end else if (m_axis_tvalid && m_axis_tready) begin
            if (bytes_remaining <= 8'd64) begin
                data_buffer     <= 1536'd0;
                bytes_remaining <= 8'd0;
                emitting        <= 1'b0;
                batch_last      <= 1'b0;
                batch_user      <= 1'b0;
                first_output    <= 1'b0;
            end else begin
                data_buffer     <= {512'd0, data_buffer[1535:512]};
                bytes_remaining <= bytes_remaining - 8'd64;
                first_output    <= 1'b0;
            end
        end
    end
endmodule


// Configuration-only pipeline. No pixel data or AXIS handshake is delayed.
// Arithmetic, comparisons and the final enable occupy separate clock stages.
module camera_config_validator #(
    parameter integer ADDR_WIDTH = 32
) (
    input wire clk, rst,
    input wire [ADDR_WIDTH-1:0] ring_base,
    input wire [31:0] frame_stride, line_bytes, frame_bytes, height,
    output logic cfg_ok
);
    wire [36:0] ring_end_ext = {{(37-ADDR_WIDTH){1'b0}}, ring_base} + ({5'd0, frame_stride} << 5);
    wire        cfg_base_aligned = (ring_base[5:0] == 6'd0);
    wire        cfg_stride_aligned = (frame_stride[5:0] == 6'd0);
    logic [15:0] expected_line_bytes_d1, expected_height_d1;
    logic [15:0] expected_line_bytes_d2, expected_height_d2;
    logic [15:0] expected_line_bytes_d3, expected_height_d3;
    (* use_dsp = "yes" *) logic [31:0] expected_raw_frame_mreg;
    (* use_dsp = "yes" *) logic [31:0] expected_raw_frame_preg;
    logic [2:0] expected_raw_frame_valid;
    always_ff @(posedge clk) begin
        if (rst) begin
            expected_line_bytes_d1 <= 0; expected_height_d1 <= 0;
            expected_line_bytes_d2 <= 0; expected_height_d2 <= 0;
            expected_line_bytes_d3 <= 0; expected_height_d3 <= 0;
            expected_raw_frame_mreg <= 0; expected_raw_frame_preg <= 0;
            expected_raw_frame_valid <= 0;
        end else begin
            expected_line_bytes_d1 <= line_bytes[15:0];
            expected_height_d1 <= height[15:0];
            expected_line_bytes_d2 <= expected_line_bytes_d1;
            expected_height_d2 <= expected_height_d1;
            expected_line_bytes_d3 <= expected_line_bytes_d2;
            expected_height_d3 <= expected_height_d2;
            expected_raw_frame_mreg <= expected_line_bytes_d1 * expected_height_d1;
            expected_raw_frame_preg <= expected_raw_frame_mreg;
            expected_raw_frame_valid <= {expected_raw_frame_valid[1:0], 1'b1};
        end
    end
    wire cfg_geometry_product_valid = expected_raw_frame_valid[2] &&
        line_bytes[15:0] == expected_line_bytes_d3 &&
        height[15:0] == expected_height_d3;
    wire        cfg_input_geometry_ok = cfg_geometry_product_valid &&
                    (line_bytes != 32'd0) &&
                    (height != 32'd0) &&
                    (frame_bytes != 32'd0) &&
                    (line_bytes <= 32'd4096) &&
                    (height <= 32'd4096) &&
                    (line_bytes[2:0] == 3'd0) &&
                    (frame_bytes[2:0] == 3'd0) &&
                    (frame_bytes == expected_raw_frame_preg);
    (* KEEP = "TRUE" *) logic [33:0] rgb_bytes_s1;
    (* KEEP = "TRUE" *) logic [36:0] ring_end_s1;
    logic [31:0] stride_s1;
    logic geometry_ok_s1, aligned_s1, programmed_s1;
    // Stage 1 ends the multiply-by-three and ring-end carry chains.
    always_ff @(posedge clk) begin
        if (rst) begin
            rgb_bytes_s1 <= 0; ring_end_s1 <= 0; stride_s1 <= 0;
            geometry_ok_s1 <= 0; aligned_s1 <= 0; programmed_s1 <= 0;
            cfg_ok <= 0;
        end else begin
            rgb_bytes_s1 <= {2'd0, frame_bytes} + ({2'd0, frame_bytes} << 1);
            ring_end_s1 <= ring_end_ext;
            stride_s1 <= frame_stride;
            geometry_ok_s1 <= cfg_input_geometry_ok;
            aligned_s1 <= cfg_base_aligned && cfg_stride_aligned;
            programmed_s1 <= (frame_stride != 0);
            // Stage 2 ends the size comparison before downstream flow control.
            cfg_ok <= geometry_ok_s1 && aligned_s1 && programmed_s1 &&
                      (ring_end_s1[36:32] == 0) &&
                      (rgb_bytes_s1[33:32] == 0) &&
                      (stride_s1 >= rgb_bytes_s1[31:0]);
        end
    end
endmodule

// Whole eight-byte samples; fixed lane writes avoid a wide variable shifter.
// Eight input cycles + one output cycle per full beat; no READY comb path
// from DDR through this packer. Output and sidebands stay stable on stalls.
module camera_raw64_to_axis512 (
 input wire clk,rst,
 input wire [63:0] s_data,input wire s_valid,output wire s_ready,
 input wire s_user,s_last,
 output reg [511:0] m_data,output reg [63:0] m_keep,
 output reg m_valid,input wire m_ready,output reg m_user,m_last
);
 reg [2:0] lane;
 wire [2:0] at_lane = s_user ? 3'd0 : lane;
 assign s_ready = !m_valid;
 always @(posedge clk) begin
  if(rst) begin lane<=0;m_keep<=0;m_valid<=0;m_user<=0;m_last<=0;end
  else if(m_valid) begin
   if(m_ready) begin m_valid<=0;m_last<=0;m_user<=0;end
  end else if(s_valid && s_ready) begin
   if(at_lane==0) m_user<=s_user;
   if(s_last || at_lane==7) begin
    m_valid<=1;m_last<=s_last;lane<=0;
    case(at_lane)
     0:m_keep<=64'h00000000000000ff;1:m_keep<=64'h000000000000ffff;
     2:m_keep<=64'h0000000000ffffff;3:m_keep<=64'h00000000ffffffff;
     4:m_keep<=64'h000000ffffffffff;5:m_keep<=64'h0000ffffffffffff;
     6:m_keep<=64'h00ffffffffffffff;7:m_keep<=64'hffffffffffffffff;
    endcase
   end else lane<=at_lane+1'b1;
  end
 end
 // A valid beat has overwritten every lane enabled by m_keep.
 // Do not distribute the global reset to the 512 payload flip-flops.
 always @(posedge clk) begin
  if(s_valid && s_ready) begin
   case(at_lane)
    0:m_data[63:0]<=s_data;1:m_data[127:64]<=s_data;
    2:m_data[191:128]<=s_data;3:m_data[255:192]<=s_data;
    4:m_data[319:256]<=s_data;5:m_data[383:320]<=s_data;
    6:m_data[447:384]<=s_data;7:m_data[511:448]<=s_data;
   endcase
  end
 end
endmodule


// Admission-only pipeline; the payload path still accepts a beat per clock.
// Wait three idle edges after the final command reserves a slot. B responses
// transfer reservation to producer (sum unchanged), or release on error.
// Consumer advances only release space. Thus a stale full result is conservative.
module camera_ring_guard (
    input wire clk, rst, idle,
    input wire [31:0] producer, consumer,
    input wire [5:0] reserved,
    output reg full,
    output wire decision_valid
);
    initial full = 1'b1;
    reg [31:0] fill_reg = 0;
    reg [5:0] reserved_reg = 0;
    reg [1:0] idle_age = 0;
    wire [32:0] total = {1'b0,fill_reg} + {27'd0,reserved_reg};
    assign decision_valid = (idle_age == 2'd3) && idle && !rst;
    always @(posedge clk) begin
        if (rst) begin
            fill_reg <= 0; reserved_reg <= 0; full <= 1'b1; idle_age <= 0;
        end else begin
            fill_reg <= producer - consumer;
            reserved_reg <= reserved;
            full <= total >= 33'd32;
            if (!idle) idle_age <= 0;
            else if (idle_age != 2'd3) idle_age <= idle_age + 1'b1;
        end
    end
endmodule

// Online base-ISP control mailbox. Payload is immutable until destination ACK.
// 0x00 ID, 0x04 ABI, 0x08 capabilities, 0x0c busy/error/allowed,
// 0x10 COMMIT, 0x14 applied generation, 0x18 applied enable mask,
// 0x40..0x74 shadow profile; 0x80..0xb4 last successfully applied profile.
module camera_online_isp_mailbox (
 input wire clk, rst, allowed,
 input wire wr_en, input wire [7:0] wr_addr,
 input wire [31:0] wr_data, input wire [3:0] wr_strb,
 output wire wr_error, input wire [7:0] rd_addr, output reg [31:0] rd_data,
 input wire pixel_clk, pixel_rst, pixel_allowed,
 output wire pending, output wire [447:0] profile,
 input wire done, failed
);
 reg [31:0] shadow[0:13];
 reg [447:0] mailbox, active_profile;
 reg request, completed, sticky_error;
 reg [31:0] generation;
 reg acknowledge, result_failed;
 // Six request stages allow the constrained bundled payload to settle first.
 (* ASYNC_REG="TRUE" *) reg [5:0] request_sync;
 (* ASYNC_REG="TRUE" *) reg [2:0] ack_sync;
 (* ASYNC_REG="TRUE" *) reg [1:0] failed_sync;
 (* ASYNC_REG="TRUE" *) reg [447:0] payload_meta, payload_sync;
 wire busy=(request!=ack_sync[2]) || (request!=completed);
 wire shadow_sel=wr_addr>=8'h40 && wr_addr<=8'h74 && wr_addr[1:0]==0;
 wire commit_sel=wr_addr==8'h10 && wr_data==1;
 wire clear_sel=wr_addr==8'h0c && wr_data==2;
 assign wr_error=wr_en && (wr_strb!=4'hf || busy ||
                  !(shadow_sel || clear_sel || (commit_sel && allowed)));
 assign pending=request_sync[5]!=acknowledge;
 assign profile=payload_sync;
 integer i;
 always @(posedge clk) begin
  if(rst) begin
   request<=0;completed<=0;sticky_error<=0;generation<=0;mailbox<=0;active_profile<=0;
   ack_sync<=0;failed_sync<=0;
   for(i=0;i<14;i=i+1)shadow[i]<=0;
  end else begin
   ack_sync<={ack_sync[1:0],acknowledge};failed_sync<={failed_sync[0],result_failed};
   if(request==ack_sync[2] && request!=completed) begin
    completed<=request;
    if(failed_sync[1])sticky_error<=1;
    else begin generation<=generation+1'b1;active_profile<=mailbox;end
   end
   if(wr_en) begin
    if(wr_error)sticky_error<=1;
    else if(shadow_sel)shadow[(wr_addr-8'h40)>>2]<=wr_data;
    else if(clear_sel)sticky_error<=0;
    else if(commit_sel) begin
     for(i=0;i<14;i=i+1)mailbox[i*32+:32]<=shadow[i];
     request<=~request;sticky_error<=0;
    end
   end
  end
 end
 always @(posedge pixel_clk) begin
  if(pixel_rst) begin request_sync<=0;payload_meta<=0;payload_sync<=0;acknowledge<=0;result_failed<=0;end
  else begin
   request_sync<={request_sync[4:0],request};payload_meta<=mailbox;payload_sync<=payload_meta;
   if(pending && (!pixel_allowed || done)) begin
    acknowledge<=request_sync[5];result_failed<=!pixel_allowed || failed;
   end
  end
 end
 always @* begin
  rd_data=0;
  case(rd_addr)
   8'h00:rd_data=32'h4c495350;
   8'h04:rd_data=32'h00010000;
   8'h08:rd_data=32'h0000000f;
   8'h0c:rd_data={29'd0,allowed,sticky_error,busy};
   8'h10:rd_data={31'd0,request};
   8'h14:rd_data=generation;
   8'h18:rd_data=active_profile[31:0];
   default:begin
    if(rd_addr>=8'h40 && rd_addr<=8'h74 && rd_addr[1:0]==0)rd_data=shadow[(rd_addr-8'h40)>>2];
    if(rd_addr>=8'h80 && rd_addr<=8'hb4 && rd_addr[1:0]==0)rd_data=active_profile[((rd_addr-8'h80)>>2)*32+:32];
   end
  endcase
 end
endmodule

`default_nettype wire


// Optional ISP output -> host IOVA path. Normal builds use only the DDR queue;
// HOST_BRANCH=0 removes the host FIFO at elaboration rather than relying on
// synthesis to prune a powered-down wide memory. The DDR queue still absorbs
// burst scheduling skew and preserves its original handshake and timing cut.
module camera_isp_video_fork #(
    parameter integer FIFO_DEPTH=512,
    parameter integer HOST_FIFO_DEPTH=FIFO_DEPTH,
    parameter bit HOST_BRANCH=1'b1
) (
    input wire clk,rst,host_enabled,host_credit,tail_abort,meta_valid,
    output wire meta_take,meta_bad,
    input wire [511:0] s_data,input wire [63:0] s_keep,
    input wire [7:0] s_user,input wire s_last,s_valid,output wire s_ready,
    output wire [511:0] d_data,output wire [63:0] d_keep,
    output wire [7:0] d_user,output wire d_last,d_valid,input wire d_ready,
    output wire [511:0] h_data,output wire [63:0] h_keep,
    output wire h_sof,h_last,h_valid,input wire h_ready,
    output wire h_pending,drained
);
    localparam integer WIDTH=585;
    wire [WIDTH-1:0] in_word={s_last,s_user,s_keep,s_data};
    wire [WIDTH-1:0] d_word,h_word;
    wire d_full,d_empty,d_prog_full,d_wr_busy,d_rd_busy;
    wire h_full,h_empty,h_prog_full,h_wr_busy,h_rd_busy;
    wire d_overflow,h_overflow;
    wire push=s_valid && s_ready;
    // Only the DDR queue controls upstream READY. A stalled PCIe host can
    // discard its own complete frame, never the DDR/RDMA video frame.
    assign s_ready=!d_prog_full && !d_full && !d_wr_busy && !d_rd_busy;
    assign {d_last,d_user,d_keep,d_data}=d_word;
    assign {h_last,h_sof,h_keep,h_data}={h_word[584],h_word[576],h_word[575:0]};
    xpm_fifo_sync #(.FIFO_MEMORY_TYPE("block"),.FIFO_WRITE_DEPTH(FIFO_DEPTH),
        .WRITE_DATA_WIDTH(WIDTH),.READ_DATA_WIDTH(WIDTH),.READ_MODE("fwft"),
        .FIFO_READ_LATENCY(0),.PROG_FULL_THRESH(FIFO_DEPTH-8),
        .USE_ADV_FEATURES("1707")) ddr_queue (
        .rst(rst),.wr_clk(clk),.din(in_word),.wr_en(push),
        .rd_en(d_valid&&d_ready),.dout(d_word),
        .data_valid(d_valid),.empty(d_empty),.full(d_full),
        .prog_full(d_prog_full),.overflow(d_overflow),
        .wr_rst_busy(d_wr_busy),.rd_rst_busy(d_rd_busy),
        .sleep(1'b0),.injectsbiterr(1'b0),.injectdbiterr(1'b0),
        .almost_empty(),.almost_full(),.prog_empty(),.underflow(),.wr_ack(),
        .rd_data_count(),.wr_data_count(),.sbiterr(),.dbiterr());
    generate if (HOST_BRANCH) begin : g_host_queue
        // One decision per ISP frame, retired by the corresponding frame
        // metadata. The physical input FIFO allows at most 32 committed DDR
        // frames, so 64 decision entries are sufficient with pipeline slack.
        reg [63:0] admit_bits=0, bad_bits=0;
        reg [6:0] admit_wr=0, admit_rd=0;
        reg [5:0] active_slot=0;
        reg frame_selected=0;
        wire decision_full=(admit_wr-admit_rd)==7'd64;
        wire host_space=!h_prog_full && !h_full && !h_wr_busy && !h_rd_busy;
        wire select_sof=host_enabled && host_credit && host_space && !decision_full && !tail_abort;
        wire selected_beat=!tail_abort && (s_user[0] ? select_sof : frame_selected);
        wire h_push=push && selected_beat && host_space;
        wire abandon=push && selected_beat && !host_space;
        assign meta_take=(admit_wr!=admit_rd) && admit_bits[admit_rd[5:0]];
        assign meta_bad=(admit_wr!=admit_rd) && bad_bits[admit_rd[5:0]];
        assign h_pending=frame_selected || !h_empty || h_valid;
        assign drained=d_empty && h_empty && !s_valid &&
                       !frame_selected && admit_wr==admit_rd;
        always @(posedge clk) begin
            if(rst) begin
                admit_bits<=0;bad_bits<=0;admit_wr<=0;admit_rd<=0;
                active_slot<=0;frame_selected<=0;
            end else begin
                if(push && s_user[0]) begin
                    admit_bits[admit_wr[5:0]]<=select_sof;
                    bad_bits[admit_wr[5:0]]<=1'b0;
                    active_slot<=admit_wr[5:0];
                    admit_wr<=admit_wr+1'b1;
                    frame_selected<=select_sof;
                end
                if(abandon) begin
                    bad_bits[s_user[0] ? admit_wr[5:0] : active_slot]<=1'b1;
                    frame_selected<=1'b0;
                end
                if(push && s_last)frame_selected<=1'b0;
                if(meta_valid && admit_wr!=admit_rd)
                    admit_rd<=admit_rd+1'b1;
                // Only software may request this after STOP_ACK and all complete
                // DDR/host frames are consumed. Drop the unpublished tail decision.
                if(tail_abort)begin frame_selected<=1'b0;admit_rd<=admit_wr;end
            end
        end
        // URAM retains the full host queue capacity. Reserve response space
        // before each standard read; unread and in-flight words both keep
        // h_empty low so frame-boundary drain cannot release DMA too early.
        wire [WIDTH-1:0] h_raw_word;
        wire h_raw_empty,h_raw_valid,h_raw_rd,h_read_pending;
        assign h_empty=h_raw_empty && !h_read_pending;
        camera_input_fifo_read_adapter #(.DATA_WIDTH(WIDTH),.DEPTH(8)) host_read_adapter (
            .clk(clk),.rst(rst),.fifo_busy(h_wr_busy || h_rd_busy),
            .fifo_empty(h_raw_empty),.fifo_rd_en(h_raw_rd),
            .fifo_data(h_raw_word),.fifo_data_valid(h_raw_valid),
            .m_axis_data(h_word),.m_axis_valid(h_valid),.m_axis_ready(h_ready),
            .pending_out(h_read_pending));
        xpm_fifo_sync #(.FIFO_MEMORY_TYPE("ultra"),.CASCADE_HEIGHT(1),
            .FIFO_WRITE_DEPTH(HOST_FIFO_DEPTH),
            .WRITE_DATA_WIDTH(WIDTH),.READ_DATA_WIDTH(WIDTH),.READ_MODE("std"),
            .FIFO_READ_LATENCY(3),.PROG_FULL_THRESH(HOST_FIFO_DEPTH-8),
            .USE_ADV_FEATURES("1707")) host_queue (
            .rst(rst),.wr_clk(clk),.din(in_word),.wr_en(h_push),
            .rd_en(h_raw_rd),.dout(h_raw_word),
            .data_valid(h_raw_valid),.empty(h_raw_empty),.full(h_full),
            .prog_full(h_prog_full),.overflow(h_overflow),
            .wr_rst_busy(h_wr_busy),.rd_rst_busy(h_rd_busy),
            .sleep(1'b0),.injectsbiterr(1'b0),.injectdbiterr(1'b0),
            .almost_empty(),.almost_full(),.prog_empty(),.underflow(),.wr_ack(),
            .rd_data_count(),.wr_data_count(),.sbiterr(),.dbiterr());
    end else begin : g_no_host_queue
        assign h_word={WIDTH{1'b0}};
        assign h_empty=1'b1;assign h_prog_full=1'b0;assign h_full=1'b0;
        assign h_wr_busy=1'b0;assign h_rd_busy=1'b0;
        assign h_overflow=1'b0;assign h_valid=1'b0;
        assign meta_take=1'b0;assign meta_bad=1'b0;
        assign h_pending=1'b0;assign drained=d_empty && !s_valid;
    end endgenerate
endmodule

module camera_direct_host_writer #(
    parameter integer FIFO_DEPTH = 8192,
    parameter bit DEMAND_GATED = 1'b0
) (
    input wire clk, rst, input wire axil_clk, axil_rstn, cap_enabled, stop_ack,
    output wire tail_abort,
    input wire csr_wr, input wire [11:0] csr_addr, input wire [31:0] csr_data,
    input wire [11:0] csr_rd_addr, output logic [31:0] csr_rdata,
    input wire [31:0] frame_bytes, width_pixels, height_pixels, pixel_format,
    input wire [511:0] s_data, input wire [63:0] s_keep,
    input wire s_valid, s_last, s_sof, s_pending, output wire stream_ready,
    output wire enabled, drained_out, frame_credit,
    input wire meta_valid, input wire [127:0] meta_data,
    input wire [31:0] meta_status, meta_ddr_idx,
    output wire [63:0] awaddr, output wire [7:0] awlen,
    output wire awvalid, input wire awready,
    output wire [511:0] wdata, output wire [63:0] wstrb,
    output wire wlast, wvalid, input wire wready,
    input wire [1:0] bresp, input wire bvalid, output wire bready
);
    logic enable_reg = 0;
    logic abort_reg = 0;
    wire abort_axi;
    logic [5:0] slots_reg = 0;
    logic [31:0] consumer_reg = 0, demand_reg = 0;
    logic [63:0] addresses_reg [0:31];
    wire [2054:0] cfg_src, cfg_dst;
    wire [31:0] consumer_axi, demand_axi;
    wire [5:0] slots_axi;
    wire [2047:0] address_axi;
    wire enable_axi;
    assign cfg_src = {enable_reg, slots_reg,
        addresses_reg[31],addresses_reg[30],addresses_reg[29],addresses_reg[28],
        addresses_reg[27],addresses_reg[26],addresses_reg[25],addresses_reg[24],
        addresses_reg[23],addresses_reg[22],addresses_reg[21],addresses_reg[20],
        addresses_reg[19],addresses_reg[18],addresses_reg[17],addresses_reg[16],
        addresses_reg[15],addresses_reg[14],addresses_reg[13],addresses_reg[12],
        addresses_reg[11],addresses_reg[10],addresses_reg[9],addresses_reg[8],
        addresses_reg[7],addresses_reg[6],addresses_reg[5],addresses_reg[4],
        addresses_reg[3],addresses_reg[2],addresses_reg[1],addresses_reg[0]};
    assign {enable_axi,slots_axi,address_axi}=cfg_dst;
    assign enabled=enable_axi;
    xpm_cdc_array_single #(.DEST_SYNC_FF(2),.SRC_INPUT_REG(0),.WIDTH(7))
        cfg_control_cdc(.src_clk(axil_clk),.src_in(cfg_src[2054:2048]),.dest_clk(clk),.dest_out(cfg_dst[2054:2048]));
    xpm_cdc_array_single #(.DEST_SYNC_FF(2),.SRC_INPUT_REG(0),.WIDTH(1024))
        cfg_addr_low_cdc(.src_clk(axil_clk),.src_in(cfg_src[1023:0]),.dest_clk(clk),.dest_out(cfg_dst[1023:0]));
    xpm_cdc_array_single #(.DEST_SYNC_FF(2),.SRC_INPUT_REG(0),.WIDTH(1024))
        cfg_addr_high_cdc(.src_clk(axil_clk),.src_in(cfg_src[2047:1024]),.dest_clk(clk),.dest_out(cfg_dst[2047:1024]));
    // Gray CDC includes registered decode and the vendor CDC constraints.
    // Software advances the producer-consumer credit one slot per CSR write.
    xpm_cdc_gray #(.DEST_SYNC_FF(2),.INIT_SYNC_FF(0),.REG_OUTPUT(1),
        .SIM_ASSERT_CHK(0),.SIM_LOSSLESS_GRAY_CHK(0),.WIDTH(32)) consumer_cdc(
        .src_clk(axil_clk),.src_in_bin(consumer_reg),
        .dest_clk(clk),.dest_out_bin(consumer_axi));
    xpm_cdc_gray #(.DEST_SYNC_FF(2),.INIT_SYNC_FF(0),.REG_OUTPUT(1),
        .SIM_ASSERT_CHK(0),.SIM_LOSSLESS_GRAY_CHK(0),.WIDTH(32)) demand_cdc(
        .src_clk(axil_clk),.src_in_bin(demand_reg),
        .dest_clk(clk),.dest_out_bin(demand_axi));
    xpm_cdc_single #(.DEST_SYNC_FF(2),.SRC_INPUT_REG(0)) abort_cdc(
        .src_clk(axil_clk),.src_in(abort_reg),.dest_clk(clk),.dest_out(abort_axi));
    assign tail_abort=abort_axi && stop_ack;
    integer a;
    always_ff @(posedge axil_clk) begin
        if(!axil_rstn)begin
            enable_reg<=0; abort_reg<=0; slots_reg<=0; consumer_reg<=0; demand_reg<=0;
            for(a=0;a<32;a=a+1)addresses_reg[a]<=0;
        end else if(csr_wr)begin
            if(csr_addr==12'ha0c)begin
                consumer_reg<=csr_data;
            end else if(csr_addr==12'ha24)begin
                demand_reg<=csr_data;
            end else if(csr_addr==12'ha28)begin
                abort_reg<=csr_data[0];
            end else if(!cap_enabled)begin
                if(csr_addr==12'ha00)enable_reg<=csr_data[0];
                if(csr_addr==12'ha04)slots_reg<=csr_data[5:0];
                if(csr_addr>=12'ha80 && csr_addr<12'hb80)begin
                    if(csr_addr[2])addresses_reg[(csr_addr-12'ha80)>>3][63:32]<=csr_data;
                    else addresses_reg[(csr_addr-12'ha80)>>3][31:0]<=csr_data;
                end
            end
        end
    end

    wire [577:0] fifo_dout, pixel_fifo_raw_data;
    wire fifo_valid, fifo_empty, fifo_full, fifo_prog_full, fifo_overflow;
    wire fifo_rd, pixel_fifo_raw_empty, pixel_fifo_raw_valid, pixel_fifo_raw_rd;
    wire pixel_fifo_wr_busy, pixel_fifo_rd_busy, pixel_fifo_adapter_pending;
    wire [$clog2(FIFO_DEPTH):0] pixel_fifo_raw_count;
    // A registered write enable/payload removes cfg/ISP handshake logic from
    // the fanout to every BRAM bank. PROG_FULL leaves eight beats of headroom.
    (* max_fanout=16 *) logic pixel_write_valid=0;
    logic [577:0] pixel_write_data;
    logic [$clog2(FIFO_DEPTH)+1:0] pixel_fifo_count=0;
    wire [191:0] meta_dout;
    wire meta_available, meta_empty, meta_prog_full, meta_overflow;
    wire meta_rd;
    assign stream_ready=!fifo_prog_full && !meta_prog_full &&
        !pixel_fifo_wr_busy && !pixel_fifo_rd_busy;
    always_ff @(posedge clk) begin
        if (rst) pixel_write_valid<=0;
        else pixel_write_valid<=s_valid && enable_axi;
        if (s_valid && enable_axi) pixel_write_data<={s_sof,s_last,s_keep,s_data};
        // Count every accepted word until the host writer consumes it,
        // including the write register, BRAM read pipeline and prefetch queue.
        if (rst) pixel_fifo_count<=0;
        else case ({s_valid && enable_axi,fifo_rd})
            2'b10: pixel_fifo_count<=pixel_fifo_count+1'b1;
            2'b01: pixel_fifo_count<=pixel_fifo_count-1'b1;
            default: begin end
        endcase
    end
    xpm_fifo_sync #(.FIFO_MEMORY_TYPE("block"),.FIFO_WRITE_DEPTH(FIFO_DEPTH),
        .CASCADE_HEIGHT(1),
        .WRITE_DATA_WIDTH(578),.READ_DATA_WIDTH(578),.READ_MODE("std"),
        .RD_DATA_COUNT_WIDTH($clog2(FIFO_DEPTH)+1),.WR_DATA_COUNT_WIDTH($clog2(FIFO_DEPTH)+1),
        .FIFO_READ_LATENCY(3),.PROG_FULL_THRESH(FIFO_DEPTH-8),
        .USE_ADV_FEATURES("1707")) pixels_fifo (
        .rst(rst),.wr_clk(clk),.din(pixel_write_data),
        .wr_en(pixel_write_valid),.rd_en(pixel_fifo_raw_rd),.dout(pixel_fifo_raw_data),
        .data_valid(pixel_fifo_raw_valid),.empty(pixel_fifo_raw_empty),.full(fifo_full),
        .prog_full(fifo_prog_full),.overflow(fifo_overflow),.sleep(1'b0),
        .injectsbiterr(1'b0),.injectdbiterr(1'b0),.rd_data_count(pixel_fifo_raw_count),.wr_data_count(),
        .almost_empty(),.almost_full(),.prog_empty(),.underflow(),.wr_ack(),
        .sbiterr(),.dbiterr(),.rd_rst_busy(pixel_fifo_rd_busy),.wr_rst_busy(pixel_fifo_wr_busy));
    camera_input_fifo_read_adapter #(.DATA_WIDTH(578),.DEPTH(8)) host_read_adapter (
        .clk(clk),.rst(rst),.fifo_busy(pixel_fifo_rd_busy),
        .fifo_empty(pixel_fifo_raw_empty),.fifo_data(pixel_fifo_raw_data),
        .fifo_data_valid(pixel_fifo_raw_valid),.fifo_rd_en(pixel_fifo_raw_rd),
        .m_axis_data(fifo_dout),.m_axis_valid(fifo_valid),.m_axis_ready(fifo_rd),
        .pending_out(pixel_fifo_adapter_pending));
    assign fifo_empty=pixel_fifo_raw_empty && !pixel_fifo_adapter_pending &&
        !pixel_write_valid && pixel_fifo_count==0;
    xpm_fifo_sync #(.FIFO_MEMORY_TYPE("distributed"),.FIFO_WRITE_DEPTH(64),
        .WRITE_DATA_WIDTH(192),.READ_DATA_WIDTH(192),.READ_MODE("fwft"),
        .FIFO_READ_LATENCY(0),.PROG_FULL_THRESH(56),.USE_ADV_FEATURES("1707"))
        metadata_fifo (
        .rst(rst),.wr_clk(clk),.din({meta_ddr_idx,meta_status,meta_data}),
        .wr_en(meta_valid&&enable_axi),.rd_en(meta_rd),.dout(meta_dout),
        .data_valid(meta_available),.empty(meta_empty),.full(),
        .prog_full(meta_prog_full),.overflow(meta_overflow),.sleep(1'b0),
        .injectsbiterr(1'b0),.injectdbiterr(1'b0),.rd_data_count(),.wr_data_count(),
        .almost_empty(),.almost_full(),.prog_empty(),.underflow(),.wr_ack(),
        .sbiterr(),.dbiterr(),.rd_rst_busy(),.wr_rst_busy());

    localparam IDLE=0, AW=1, W=2, B=3, DRAIN=4, META=5,
               FOOT_AW=6, FOOT_W=7, FOOT_B=8, FAULT=9, ABORT_DRAIN=10,
               YIELD=11;
    logic [3:0] state=IDLE;
    logic [31:0] producer=0, admitted_frames=0, stall_cycles=0, error_code=0;
    // Reserve the host slot at SOF, before its footer increments producer.
    // This credit crosses only a frame-boundary admission decision, never READY.
    // Reserve credit one clock ahead.  Include a SOF accepted on this edge
    // so even back-to-back one-beat frames cannot consume a stale host slot.
    // The CSR consumer is monotonically advanced while capture is active;
    // its one-clock CDC observation delay can only defer a newly freed slot.
    logic host_slot_room_q = 1'b0;
    wire [31:0] admitted_next = admitted_frames + (s_valid && s_sof);
    always_ff @(posedge clk) begin
        if (rst) host_slot_room_q <= 1'b0;
        else host_slot_room_q <= slots_axi != 0 &&
            (admitted_next-consumer_axi) < {26'd0,slots_axi};
    end
    assign frame_credit=enable_axi && !tail_abort && host_slot_room_q &&
        state!=FAULT && stream_ready &&
        (!DEMAND_GATED || (admitted_frames != demand_axi));
    logic [63:0] base=0, address_cursor=0, offset=0;
    logic [39:0] footer_addr=0;
    logic [20:0] footer_low_sum=0;
    wire [31:0] frame_beats=(frame_bytes+32'd63)>>6;
    wire [39:0] aligned_frame_bytes={2'd0,frame_beats,6'd0};
    logic [31:0] remaining=0;
    logic [6:0] burst_beats=0, beats_left=0;
    logic saw_last=0, pad_tail=0, bad_frame=0;
    // Payload requests may remain in flight while later 1 KiB bursts issue.
    // The footer is an ordered publication fence after every payload BRESP.
    // Limit the high-priority PCIe path to sixteen queued 1-KiB transactions.
    // This retains a 16-KiB response-latency window, one quarter of the
    // former sixteen queued 4-KiB transactions.
    // A four-cycle AW pause after each burst gives lower-priority requesters
    // a bounded arbitration window before the next high-priority grant.
    localparam logic [4:0] MAX_PAYLOAD_OUTSTANDING=5'd16;
    localparam logic [6:0] HOST_BURST_BEATS=7'd16;
    logic [1:0] yield_cycles=0;
    logic [4:0] outstanding_writes=0;
    wire aw_fire=awvalid && awready;
    wire b_fire=bvalid && bready;
    logic [511:0] footer=0;
    wire payload_wfire=state==W && wvalid && wready;
    wire unexpected_sof=fifo_valid && fifo_dout[577] && offset!=0;
    // DDR metadata may lead the last ISP pixels. An empty local FIFO is
    // not end-of-frame while the upstream fork still owns pixel data.
    wire source_closed=meta_available && fifo_empty && !s_pending;
    wire no_source=tail_abort || pad_tail || unexpected_sof ||
        (state==W && source_closed);
    // Never lock the shared W channel while waiting for future ISP pixels.
    // Otherwise the DDR beat FIFO could fill before a 4KiB host burst finishes.
    // Do not reserve the shared AXI W channel on early metadata alone.
    // A full burst can proceed; a genuinely closed source may be padded.
    wire burst_buffered=pixel_fifo_count>=burst_beats ||
        (meta_available && !s_pending) || pad_tail;
    assign awvalid=(state==AW && !tail_abort && burst_buffered &&
                    outstanding_writes<MAX_PAYLOAD_OUTSTANDING) ||
                   (state==FOOT_AW && outstanding_writes==0);
    assign awaddr=address_cursor;
    assign awlen=state==FOOT_AW ? 8'd0 : {1'b0,burst_beats}-8'd1;
    assign wvalid=state==FOOT_W || (state==W && (no_source || fifo_valid));
    assign wdata=state==FOOT_W ? footer : no_source ? 512'd0 : fifo_dout[511:0];
    assign wstrb=state==FOOT_W || no_source ? ~64'd0 : fifo_dout[575:512];
    assign wlast=state==FOOT_W || (state==W && beats_left==1);
    assign bready=outstanding_writes!=0;
    assign fifo_rd=(payload_wfire&&!no_source) || (state==DRAIN&&fifo_valid&&!fifo_dout[577]) ||
        (state==ABORT_DRAIN&&fifo_valid);
    assign meta_rd=((state==META) || (state==ABORT_DRAIN)) && meta_available;
    wire drained=((state==IDLE) || (state==ABORT_DRAIN)) && outstanding_writes==0 &&
        fifo_empty && meta_empty && !s_pending &&
        !pixel_fifo_wr_busy && !pixel_fifo_rd_busy;
    assign drained_out=drained;
    wire [31:0] debug_fifo_count = pixel_fifo_count;
    wire [31:0] writer_debug = {8'd0,debug_fifo_count[15:0],
        meta_available,fifo_valid,meta_empty,fifo_empty,state};
    wire [128:0] status_src={drained,writer_debug,producer,stall_cycles,error_code};
    wire [128:0] status_dst;
    wire drained_axil=status_dst[128];
    camera_status_snapshot_cdc #(.WIDTH(129)) status_cdc (.src_clk(clk),.src_in(status_src),.dest_clk(axil_clk),.dest_out(status_dst));
    // Drain acknowledgement shares the producer snapshot, including aborts.
    always_comb begin
        csr_rdata=0;
        case(csr_rd_addr)
            12'ha00:csr_rdata={31'd0,enable_reg};
            12'ha04:csr_rdata={26'd0,slots_reg};
            12'ha0c:csr_rdata=consumer_reg;
            12'ha10:csr_rdata={30'd0,(status_dst[31:0]!=0),drained_axil};
            12'ha14:csr_rdata=status_dst[95:64];
            12'ha18:csr_rdata=status_dst[63:32];
            12'ha1c:csr_rdata=status_dst[31:0];
            12'ha20:csr_rdata=status_dst[127:96];
            12'ha24:csr_rdata=demand_reg;
            12'ha28:csr_rdata={31'd0,abort_reg};
            default:begin
                if(csr_rd_addr>=12'ha80 && csr_rd_addr<12'hb80)
                    csr_rdata=csr_rd_addr[2] ?
                        addresses_reg[(csr_rd_addr-12'ha80)>>3][63:32] :
                        addresses_reg[(csr_rd_addr-12'ha80)>>3][31:0];
            end
        endcase
    end
    always_ff @(posedge clk) begin
        if(rst)begin
            state<=IDLE;producer<=0;admitted_frames<=0;stall_cycles<=0;error_code<=0;
            base<=0;address_cursor<=0;offset<=0;footer_addr<=0;footer_low_sum<=0;
            remaining<=0;burst_beats<=0;beats_left<=0;
            saw_last<=0;pad_tail<=0;bad_frame<=0;footer<=0;
            outstanding_writes<=0;yield_cycles<=0;
        end else begin
            if(s_valid && s_sof)admitted_frames<=admitted_frames+1'b1;
            // Pipeline the footer target over the first AW/W burst. Neither
            // the B-state abort decision nor the host AW request sees a
            // 40-bit address adder on its combinational timing path.
            if(state==AW && offset==0)
                footer_low_sum<={1'b0,base[19:0]}+
                                {1'b0,aligned_frame_bytes[19:0]};
            if(state==W && offset==0 && beats_left==burst_beats)begin
                footer_addr[19:0]<=footer_low_sum[19:0];
                footer_addr[39:20]<=base[39:20]+aligned_frame_bytes[39:20]+
                                     footer_low_sum[20];
            end
            case ({aw_fire,b_fire})
                2'b10:outstanding_writes<=outstanding_writes+1'b1;
                2'b01:outstanding_writes<=outstanding_writes-1'b1;
                default:begin end
            endcase
            if(enable_axi && s_pending && !stream_ready && stall_cycles!=32'hffffffff)
                stall_cycles<=stall_cycles+1;
            if(fifo_overflow || meta_overflow)begin error_code<=1;state<=FAULT;end
            else if(b_fire && bresp!=2'b00)begin
                error_code<=(state==FOOT_B) ? 32'd4 : 32'd3;
                state<=FAULT;
            end else case(state)
            IDLE:if(tail_abort)state<=ABORT_DRAIN;
            else if(fifo_valid && slots_axi!=0 && (producer-consumer_axi)<{26'd0,slots_axi})begin
                if(slots_axi>32 || (slots_axi&(slots_axi-1))!=0 || frame_bytes==0)begin
                    error_code<=2;state<=FAULT;
                end else begin
                    base<=address_axi[(producer[4:0]&(slots_axi[4:0]-5'd1))*64 +: 64];
                    address_cursor<=address_axi[(producer[4:0]&(slots_axi[4:0]-5'd1))*64 +: 64];
                    offset<=0;remaining<=frame_beats;
                    burst_beats<=frame_beats>HOST_BURST_BEATS ? HOST_BURST_BEATS : frame_beats[6:0];
                    beats_left<=frame_beats>HOST_BURST_BEATS ? HOST_BURST_BEATS : frame_beats[6:0];
                    saw_last<=0;pad_tail<=0;bad_frame<=!fifo_dout[577];state<=AW;
                end
            end
            AW:if(tail_abort)state<=ABORT_DRAIN;
            else if(aw_fire)state<=W;
            W:if(payload_wfire)begin
                if(no_source)begin bad_frame<=1;pad_tail<=1;saw_last<=1;end
                if(!no_source && fifo_dout[576])begin
                    saw_last<=1;
                    if(remaining!=burst_beats || beats_left!=1)begin bad_frame<=1;pad_tail<=1;end
                end
                beats_left<=beats_left-1'b1;
                if(beats_left==1)state<=B;
            end
            // Advance to the next payload burst without waiting for BRESP.
            // The outstanding counter and ordered footer fence enforce safety.
            B:if(tail_abort)state<=ABORT_DRAIN;
            else begin
                // A rejected optional host frame must not consume the whole
                // full-resolution PCIe payload as zero padding. Finish only
                // the already accepted AXI burst, then publish a bad footer.
                // The DDR branch and its frame ownership are unaffected.
                if(pad_tail)begin
                    address_cursor<={24'd0,footer_addr};
                    state<=saw_last ? META : DRAIN;
                end else if(remaining>burst_beats)begin
                    remaining<=remaining-burst_beats;
                    offset<=offset+{51'd0,burst_beats,6'd0};
                    address_cursor<={24'd0,address_cursor[39:0]+{27'd0,burst_beats,6'd0}};
                    burst_beats<=remaining-burst_beats>HOST_BURST_BEATS ? HOST_BURST_BEATS : remaining-burst_beats;
                    beats_left<=remaining-burst_beats>HOST_BURST_BEATS ? HOST_BURST_BEATS : remaining-burst_beats;
                    yield_cycles<=2'd3;
                    state<=YIELD;
                end else begin
                    offset<={26'd0,frame_beats,6'd0};
                    address_cursor<={24'd0,address_cursor[39:0]+{27'd0,burst_beats,6'd0}};
                    state<=saw_last ? META : DRAIN;
                end
            end
            YIELD:if(tail_abort)state<=ABORT_DRAIN;
            else if(yield_cycles!=0)yield_cycles<=yield_cycles-1'b1;
            else state<=AW;
            DRAIN:if(tail_abort)state<=ABORT_DRAIN;
            else begin
                bad_frame<=1;
                // A forked host frame can end without TLAST after host FIFO
                // admission is revoked. Its metadata closes the frame even
                // when no next host SOF follows.
                if(source_closed)state<=META;
                else if(fifo_valid && fifo_dout[577])state<=META;
                else if(fifo_rd && fifo_dout[576])state<=META;
            end
            META:if(meta_available)begin
                // 64-byte little-endian footer, token at byte 56.
                footer<=0;
                footer[31:0]<=32'h31485349;footer[63:32]<=32'd1;
                footer[127:64]<={32'd0,meta_dout[127:96]};
                footer[159:128]<=frame_bytes;footer[191:160]<=width_pixels;
                footer[223:192]<=height_pixels;
                footer[255:224]<=bad_frame ? 32'd0 : meta_dout[159:128];
                footer[287:256]<=meta_dout[191:160];
                footer[319:288]<=pixel_format;
                footer[511:448]<={32'd0,producer}+64'd1;
                state<=FOOT_AW;
            end
            FOOT_AW:if(aw_fire)state<=FOOT_W;
            FOOT_W:if(wready)state<=FOOT_B;
            FOOT_B:if(b_fire)begin producer<=producer+1;state<=IDLE;end
            ABORT_DRAIN:begin
                // No new AW is issued. Drain staged pixels and B responses;
                // the uncommitted host slot is never published to software.
            end
            default:begin end
            endcase
        end
    end
endmodule

// Burst arbitration with ordered response tags. The existing Xilinx S06 has
// THREAD_ID_WIDTH=0 and returns BID=0 for both targets. Do not use input IDs
// to demultiplex B: all requests use the legacy ID and tags follow AXI order.
module camera_dual_write_mux (
    input wire clk,rst,
    input wire [63:0] d_awaddr,h_awaddr,input wire [7:0] d_awlen,h_awlen,
    input wire d_awvalid,h_awvalid,output wire d_awready,h_awready,
    input wire [511:0] d_wdata,h_wdata,input wire [63:0] d_wstrb,h_wstrb,
    input wire d_wlast,h_wlast,d_wvalid,h_wvalid,output wire d_wready,h_wready,
    output wire [1:0] d_bresp,h_bresp,output wire d_bvalid,h_bvalid,
    input wire d_bready,h_bready,
    output wire [2:0] m_awid,output wire [63:0] m_awaddr,output wire [7:0] m_awlen,
    output wire m_awvalid,input wire m_awready,
    output wire [511:0] m_wdata,output wire [63:0] m_wstrb,
    output wire m_wlast,m_wvalid,input wire m_wready,
    input wire [2:0] m_bid,input wire [1:0] m_bresp,input wire m_bvalid,
    output wire m_bready
);
    // Register both directions at the shared camera AXI boundary. This cuts
    // BRAM -> DDR clock-converter data paths and PCIe/crossbar READY -> FIFO
    // pop paths, while retaining burst/response order at the acceptance point.
    wire [63:0] arb_awaddr;
    wire [7:0] arb_awlen;
    wire arb_awvalid,arb_awready;
    wire [511:0] arb_wdata;
    wire [63:0] arb_wstrb;
    wire arb_wlast,arb_wvalid,arb_wready;
    wire [1:0] arb_bresp;
    wire arb_bvalid,arb_bready;

    logic active=0,route=0,prefer_host=0,aw_locked=0,aw_route=0;
    // A continuously asserted DDR AW previously starved the host writer for
    // an entire frame.  Alternate 4-KiB bursts when both paths request;
    // each branch receives a bounded service interval without coupling their
    // frame admission or adding another pixel FIFO.
    wire select_host=aw_locked ? aw_route : h_awvalid && (!d_awvalid || prefer_host);
    logic [31:0] response_routes=0;
    logic [4:0] response_rd=0,response_wr=0;
    logic [5:0] response_count=0;
    wire response_full=response_count==6'd32;
    wire response_host=response_routes[response_rd];
    wire response_available=response_count!=0;
    wire aw_fire=arb_awvalid&&arb_awready;
    wire b_fire=arb_bvalid&&arb_bready;
    assign m_awid=3'd0;
    assign arb_awaddr=select_host ? h_awaddr : d_awaddr;
    assign arb_awlen=select_host ? h_awlen : d_awlen;
    assign arb_awvalid=!active && !response_full && (select_host ? h_awvalid : d_awvalid);
    assign d_awready=!active && !response_full && !select_host && arb_awready;
    assign h_awready=!active && !response_full && select_host && arb_awready;
    assign arb_wdata=route ? h_wdata : d_wdata;
    assign arb_wstrb=route ? h_wstrb : d_wstrb;
    assign arb_wlast=route ? h_wlast : d_wlast;
    assign arb_wvalid=active && (route ? h_wvalid : d_wvalid);
    assign d_wready=active && !route && arb_wready;
    assign h_wready=active && route && arb_wready;
    assign d_bresp=arb_bresp;assign h_bresp=arb_bresp;
    assign d_bvalid=arb_bvalid && response_available && !response_host;
    assign h_bvalid=arb_bvalid && response_available && response_host;
    assign arb_bready=response_available && (response_host ? h_bready : d_bready);
    always_ff @(posedge clk)begin
        if(rst)begin
            active<=0;route<=0;prefer_host<=0;aw_locked<=0;aw_route<=0;
            response_routes<=0;response_rd<=0;response_wr<=0;response_count<=0;
        end
        else begin
            if(aw_fire)begin
                response_routes[response_wr]<=select_host;
                prefer_host<=!select_host;
                response_wr<=response_wr+1'b1;
            end
            if(b_fire)response_rd<=response_rd+1'b1;
            case({aw_fire,b_fire})
                2'b10:response_count<=response_count+1'b1;
                2'b01:response_count<=response_count-1'b1;
                default:begin end
            endcase
            if(arb_awvalid&&!arb_awready)begin aw_locked<=1;aw_route<=select_host;end
            if(arb_awvalid&&arb_awready)begin active<=1;route<=select_host;aw_locked<=0;end
            if(arb_wvalid&&arb_wready&&arb_wlast)active<=0;
        end
    end

    axis_register #(
        .DATA_WIDTH(72),.KEEP_ENABLE(0),.KEEP_WIDTH(1),.LAST_ENABLE(0),
        .ID_ENABLE(0),.ID_WIDTH(1),.DEST_ENABLE(0),.DEST_WIDTH(1),
        .USER_ENABLE(0),.USER_WIDTH(1),.REG_TYPE(2)
    ) aw_timing_slice (
        .clk(clk),.rst(rst),
        .s_axis_tdata({arb_awaddr,arb_awlen}),.s_axis_tkeep(1'b1),
        .s_axis_tvalid(arb_awvalid),.s_axis_tready(arb_awready),
        .s_axis_tlast(1'b1),.s_axis_tid(1'b0),.s_axis_tdest(1'b0),.s_axis_tuser(1'b0),
        .m_axis_tdata({m_awaddr,m_awlen}),.m_axis_tkeep(),
        .m_axis_tvalid(m_awvalid),.m_axis_tready(m_awready),
        .m_axis_tlast(),.m_axis_tid(),.m_axis_tdest(),.m_axis_tuser()
    );
    axis_register #(
        .DATA_WIDTH(512),.KEEP_ENABLE(1),.KEEP_WIDTH(64),.LAST_ENABLE(1),
        .ID_ENABLE(0),.ID_WIDTH(1),.DEST_ENABLE(0),.DEST_WIDTH(1),
        .USER_ENABLE(0),.USER_WIDTH(1),.REG_TYPE(2)
    ) w_timing_slice (
        .clk(clk),.rst(rst),
        .s_axis_tdata(arb_wdata),.s_axis_tkeep(arb_wstrb),
        .s_axis_tvalid(arb_wvalid),.s_axis_tready(arb_wready),
        .s_axis_tlast(arb_wlast),.s_axis_tid(1'b0),.s_axis_tdest(1'b0),.s_axis_tuser(1'b0),
        .m_axis_tdata(m_wdata),.m_axis_tkeep(m_wstrb),
        .m_axis_tvalid(m_wvalid),.m_axis_tready(m_wready),
        .m_axis_tlast(m_wlast),.m_axis_tid(),.m_axis_tdest(),.m_axis_tuser()
    );
    axis_register #(
        .DATA_WIDTH(2),.KEEP_ENABLE(0),.KEEP_WIDTH(1),.LAST_ENABLE(0),
        .ID_ENABLE(0),.ID_WIDTH(1),.DEST_ENABLE(0),.DEST_WIDTH(1),
        .USER_ENABLE(0),.USER_WIDTH(1),.REG_TYPE(2)
    ) b_timing_slice (
        .clk(clk),.rst(rst),
        .s_axis_tdata(m_bresp),.s_axis_tkeep(1'b1),
        .s_axis_tvalid(m_bvalid),.s_axis_tready(m_bready),
        .s_axis_tlast(1'b1),.s_axis_tid(1'b0),.s_axis_tdest(1'b0),.s_axis_tuser(1'b0),
        .m_axis_tdata(arb_bresp),.m_axis_tkeep(),
        .m_axis_tvalid(arb_bvalid),.m_axis_tready(arb_bready),
        .m_axis_tlast(),.m_axis_tid(),.m_axis_tdest(),.m_axis_tuser()
    );

endmodule

// Periodic coherent snapshots. Registered bundled data is held for the entire
// four-phase handshake. Only its request/acknowledgement crosses through XPM;
// timing.xdc bounds mailbox-to-publication delay/skew. Functional counter resets
// change data only and never cancel an in-flight snapshot transaction.
module camera_status_snapshot_cdc #(parameter integer WIDTH=32)(
    input wire src_clk, input wire [WIDTH-1:0] src_in,
    input wire dest_clk, output wire [WIDTH-1:0] dest_out
);
    reg send=1'b0, ack=1'b0;
    wire received, requested;
    reg [WIDTH-1:0] mailbox={WIDTH{1'b0}};
    reg [WIDTH-1:0] published={WIDTH{1'b0}};
    assign dest_out=published;
    always @(posedge src_clk) begin
        if (send) begin
            if (received) send<=1'b0;
        end else if (!received) begin
            mailbox<=src_in;
            send<=1'b1;
        end
    end
    always @(posedge dest_clk) begin
        if (!ack) begin
            if (requested) begin
                published<=mailbox;
                ack<=1'b1;
            end
        end else if (!requested) ack<=1'b0;
    end
    xpm_cdc_handshake #(.WIDTH(1),.DEST_EXT_HSK(1),.DEST_SYNC_FF(3),
        .SRC_SYNC_FF(3),.INIT_SYNC_FF(1),.SIM_ASSERT_CHK(1)) control (
        .src_clk(src_clk),.src_in(1'b1),.src_send(send),.src_rcv(received),
        .dest_clk(dest_clk),.dest_out(),.dest_req(requested),.dest_ack(ack)
    );
`ifndef SYNTHESIS
    reg previous_send=1'b0, previous_ack=1'b0, previous_busy=1'b0;
    reg [WIDTH-1:0] previous_mailbox={WIDTH{1'b0}}, previous_publication={WIDTH{1'b0}};
    always @(posedge src_clk) begin
        if(send !== previous_send) begin
            if(send && received !== 1'b0) $fatal(1,"snapshot request before idle: %m");
            if(!send && received !== 1'b1) $fatal(1,"snapshot request released before receipt: %m");
        end
        if((send || received) && previous_busy && mailbox !== previous_mailbox)
            $fatal(1,"snapshot source changed an owned mailbox: %m");
        previous_send<=send;previous_busy<=send || received;previous_mailbox<=mailbox;
    end
    always @(posedge dest_clk) begin
        if(ack !== previous_ack) begin
            if(ack && requested !== 1'b1) $fatal(1,"snapshot ack before request: %m");
            if(!ack && requested !== 1'b0) $fatal(1,"snapshot ack before request idle: %m");
        end
        if(ack && previous_ack && published !== previous_publication)
            $fatal(1,"snapshot destination changed an acknowledged publication: %m");
        previous_ack<=ack;previous_publication<=published;
    end
`endif
endmodule
