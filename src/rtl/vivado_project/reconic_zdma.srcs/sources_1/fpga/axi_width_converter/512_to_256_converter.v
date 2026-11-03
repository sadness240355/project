`timescale 1ns / 1ps

module axi4_dw_512_to_256 #(
    parameter integer ADDR_W      = 64,
    parameter integer ID_W        = 4,
    parameter integer OUTSTANDING = 3
) (
    input wire aclk,
    input wire aresetn,

    // =========================================================
    // SI : 512-bit AXI4 slave-side interface
    // Supported subset on SI:
    //   - INCR burst only
    //   - full-width AWSIZE/ARSIZE=6, 64B aligned
    //   - also single-beat AWSIZE=2, 4B aligned CQE/doorbell writes
    //   - single ID thread
    // =========================================================
    input  wire [  ID_W-1:0] s_axi_awid,
    input  wire [ADDR_W-1:0] s_axi_awaddr,
    input  wire [       7:0] s_axi_awlen,
    input  wire [       2:0] s_axi_awsize,
    input  wire [       1:0] s_axi_awburst,
    input  wire              s_axi_awvalid,
    output wire              s_axi_awready,

    input  wire [511:0] s_axi_wdata,
    input  wire [ 63:0] s_axi_wstrb,
    input  wire         s_axi_wlast,
    input  wire         s_axi_wvalid,
    output wire         s_axi_wready,

    output reg  [ID_W-1:0] s_axi_bid,
    output reg  [     1:0] s_axi_bresp,
    output reg             s_axi_bvalid,
    input  wire            s_axi_bready,

    input  wire [  ID_W-1:0] s_axi_arid,
    input  wire [ADDR_W-1:0] s_axi_araddr,
    input  wire [       7:0] s_axi_arlen,
    input  wire [       2:0] s_axi_arsize,
    input  wire [       1:0] s_axi_arburst,
    input  wire              s_axi_arvalid,
    output wire              s_axi_arready,

    output reg  [ID_W-1:0] s_axi_rid,
    output reg  [   511:0] s_axi_rdata,
    output reg  [     1:0] s_axi_rresp,
    output reg             s_axi_rlast,
    output reg             s_axi_rvalid,
    input  wire            s_axi_rready,

    // =========================================================
    // MI : 256-bit AXI4 master-side interface
    // Generated MI transactions:
    //   - INCR burst only
    //   - full-width AWSIZE/ARSIZE=5; narrow writes preserve AWSIZE=2
    // =========================================================
    output wire [  ID_W-1:0] m_axi_awid,
    output wire [ADDR_W-1:0] m_axi_awaddr,
    output wire [       7:0] m_axi_awlen,
    output wire [       2:0] m_axi_awsize,
    output wire [       1:0] m_axi_awburst,
    output wire              m_axi_awvalid,
    input  wire              m_axi_awready,

    output wire [255:0] m_axi_wdata,
    output wire [ 31:0] m_axi_wstrb,
    output wire         m_axi_wlast,
    output wire         m_axi_wvalid,
    input  wire         m_axi_wready,

    input  wire [ID_W-1:0] m_axi_bid,
    input  wire [     1:0] m_axi_bresp,
    input  wire            m_axi_bvalid,
    output wire            m_axi_bready,

    output wire [  ID_W-1:0] m_axi_arid,
    output wire [ADDR_W-1:0] m_axi_araddr,
    output wire [       7:0] m_axi_arlen,
    output wire [       2:0] m_axi_arsize,
    output wire [       1:0] m_axi_arburst,
    output wire              m_axi_arvalid,
    input  wire              m_axi_arready,

    input  wire [ID_W-1:0] m_axi_rid,
    input  wire [   255:0] m_axi_rdata,
    input  wire [     1:0] m_axi_rresp,
    input  wire            m_axi_rlast,
    input  wire            m_axi_rvalid,
    output wire            m_axi_rready
);

    localparam integer MAX_MI_BURST = 256;  // AXI4 max beats
    localparam [8:0] MAX_SI_SEG_BEATS = 9'd128;  // 128 * 2 = 256 MI beats
    localparam integer WRSEG_DEPTH = (OUTSTANDING < 1) ? 2 : (OUTSTANDING * 2);

    localparam integer AW_CMD_W = ID_W + ADDR_W + 8 + 1; // narrow flag
    localparam integer AR_CMD_W = ID_W + ADDR_W + 8;
    localparam integer B_CMD_W = ID_W + 1;  // {id, split_flag}
    localparam integer R_CMD_W = ID_W + 8;  // {id, len}
    localparam integer SEG_W = 10; // {narrow, upper_half, SI beats[7:0]}

    // =========================================================
    // Functions
    // =========================================================
    function [1:0] resp_rank;
        input [1:0] resp;
        begin
            case (resp)
                2'b11:   resp_rank = 2'd3;  // DECERR
                2'b10:   resp_rank = 2'd2;  // SLVERR
                2'b00:   resp_rank = 2'd1;  // OKAY
                2'b01:   resp_rank = 2'd0;  // EXOKAY
                default: resp_rank = 2'd3;
            endcase
        end
    endfunction

    function [1:0] worst_resp;
        input [1:0] a;
        input [1:0] b;
        begin
            worst_resp = (resp_rank(a) >= resp_rank(b)) ? a : b;
        end
    endfunction

    // =========================================================
    // Supported-request checks
    // =========================================================
    wire s_aw_supported;
    wire s_ar_supported;

    wire s_aw_narrow = (s_axi_awsize == 3'd2) && (s_axi_awlen == 0) &&
                       (s_axi_awaddr[1:0] == 0);
    assign s_aw_supported = (s_axi_awburst == 2'b01) &&
        (((s_axi_awsize == 3'd6) && (s_axi_awaddr[5:0] == 0)) || s_aw_narrow);

    assign s_ar_supported = (s_axi_arsize == 3'd6) && (s_axi_arburst == 2'b01) && (s_axi_araddr[5:0] == 6'd0);

    // =========================================================
    // FIFO I/O wires
    // =========================================================
    wire [AW_CMD_W-1:0] aw_cmd_in_data;
    wire [AR_CMD_W-1:0] ar_cmd_in_data;
    wire [ B_CMD_W-1:0] b_cmd_in_data;
    wire [ R_CMD_W-1:0] r_cmd_in_data;

    wire [AW_CMD_W-1:0] aw_cmd_out_data;
    wire                aw_cmd_out_valid;
    wire                aw_cmd_out_ready;
    wire                aw_cmd_in_ready;
    wire                aw_cmd_in_valid;

    wire [AR_CMD_W-1:0] ar_cmd_out_data;
    wire                ar_cmd_out_valid;
    wire                ar_cmd_out_ready;
    wire                ar_cmd_in_ready;
    wire                ar_cmd_in_valid;

    wire [ B_CMD_W-1:0] b_cmd_out_data;
    wire                b_cmd_out_valid;
    wire                b_cmd_out_ready;
    wire                b_cmd_in_ready;
    wire                b_cmd_in_valid;

    wire [ R_CMD_W-1:0] r_cmd_out_data;
    wire                r_cmd_out_valid;
    wire                r_cmd_out_ready;
    wire                r_cmd_in_ready;
    wire                r_cmd_in_valid;

    wire [   SEG_W-1:0] wrseg_out_data;
    wire                wrseg_out_valid;
    wire                wrseg_out_ready;
    wire                wrseg_in_ready;
    wire                wrseg_in_valid;
    wire [   SEG_W-1:0] wrseg_in_data;

    wire [         8:0] s_aw_total_beats;
    wire [         8:0] s_ar_total_beats;
    wire                s_aw_split;
    wire                s_ar_split;

    assign aw_cmd_in_data   = {s_aw_narrow, s_axi_awid, s_axi_awaddr, s_axi_awlen};
    assign ar_cmd_in_data   = {s_axi_arid, s_axi_araddr, s_axi_arlen};
    assign s_aw_total_beats = {1'b0, s_axi_awlen} + 9'd1;
    assign s_ar_total_beats = {1'b0, s_axi_arlen} + 9'd1;
    assign s_aw_split       = (s_aw_total_beats > MAX_SI_SEG_BEATS);
    assign s_ar_split       = (s_ar_total_beats > MAX_SI_SEG_BEATS);
    assign b_cmd_in_data    = {s_axi_awid, s_aw_split};
    assign r_cmd_in_data    = {s_axi_arid, s_axi_arlen};

    sync_fifo_simple #(
        .WIDTH(AW_CMD_W),
        .DEPTH(OUTSTANDING)
    ) u_aw_cmd_fifo (
        .clk      (aclk),
        .rstn     (aresetn),
        .in_data  (aw_cmd_in_data),
        .in_valid (aw_cmd_in_valid),
        .in_ready (aw_cmd_in_ready),
        .out_data (aw_cmd_out_data),
        .out_valid(aw_cmd_out_valid),
        .out_ready(aw_cmd_out_ready)
    );

    sync_fifo_simple #(
        .WIDTH(AR_CMD_W),
        .DEPTH(OUTSTANDING)
    ) u_ar_cmd_fifo (
        .clk      (aclk),
        .rstn     (aresetn),
        .in_data  (ar_cmd_in_data),
        .in_valid (ar_cmd_in_valid),
        .in_ready (ar_cmd_in_ready),
        .out_data (ar_cmd_out_data),
        .out_valid(ar_cmd_out_valid),
        .out_ready(ar_cmd_out_ready)
    );

    sync_fifo_simple #(
        .WIDTH(B_CMD_W),
        .DEPTH(OUTSTANDING)
    ) u_b_cmd_fifo (
        .clk      (aclk),
        .rstn     (aresetn),
        .in_data  (b_cmd_in_data),
        .in_valid (b_cmd_in_valid),
        .in_ready (b_cmd_in_ready),
        .out_data (b_cmd_out_data),
        .out_valid(b_cmd_out_valid),
        .out_ready(b_cmd_out_ready)
    );

    sync_fifo_simple #(
        .WIDTH(R_CMD_W),
        .DEPTH(OUTSTANDING)
    ) u_r_cmd_fifo (
        .clk      (aclk),
        .rstn     (aresetn),
        .in_data  (r_cmd_in_data),
        .in_valid (r_cmd_in_valid),
        .in_ready (r_cmd_in_ready),
        .out_data (r_cmd_out_data),
        .out_valid(r_cmd_out_valid),
        .out_ready(r_cmd_out_ready)
    );

    sync_fifo_simple #(
        .WIDTH(SEG_W),
        .DEPTH(WRSEG_DEPTH)
    ) u_wrseg_fifo (
        .clk      (aclk),
        .rstn     (aresetn),
        .in_data  (wrseg_in_data),
        .in_valid (wrseg_in_valid),
        .in_ready (wrseg_in_ready),
        .out_data (wrseg_out_data),
        .out_valid(wrseg_out_valid),
        .out_ready(wrseg_out_ready)
    );

    assign s_axi_awready = s_aw_supported && aw_cmd_in_ready && b_cmd_in_ready;
    assign s_axi_arready = s_ar_supported && ar_cmd_in_ready && r_cmd_in_ready;

    // =========================================================
    // Write address issue engine (SI AW -> split MI AW)
    // =========================================================
    wire [       7:0] awf_len;
    wire [ADDR_W-1:0] awf_addr;
    wire [  ID_W-1:0] awf_id;

    reg               wr_aw_act;
    reg               wr_aw_narrow;
    reg  [  ID_W-1:0] wr_aw_id;
    reg  [ADDR_W-1:0] wr_aw_addr_base;
    reg  [       7:0] wr_aw_seg0_beats;
    reg  [       7:0] wr_aw_seg1_beats;
    reg               wr_aw_seg_idx;  // 0 -> seg0, 1 -> seg1

    wire [       7:0] wr_aw_curr_seg_beats;
    wire              wr_aw_last_seg;
    wire [ADDR_W-1:0] wr_aw_curr_addr;
    wire [       8:0] wr_aw_mi_len_m1;

    assign awf_len              = aw_cmd_out_data[7:0];
    assign awf_addr             = aw_cmd_out_data[8+:ADDR_W];
    assign awf_id               = aw_cmd_out_data[8+ADDR_W+:ID_W];

    assign wr_aw_curr_seg_beats = wr_aw_seg_idx ? wr_aw_seg1_beats : wr_aw_seg0_beats;
    assign wr_aw_last_seg       = wr_aw_seg_idx ? 1'b1 : (wr_aw_seg1_beats == 8'd0);

    assign wr_aw_curr_addr      = wr_aw_addr_base + (wr_aw_seg_idx ? {{(ADDR_W - 14) {1'b0}}, wr_aw_seg0_beats, 6'b0} : {ADDR_W{1'b0}});

    assign wr_aw_mi_len_m1      = {wr_aw_curr_seg_beats, 1'b0} - 9'd1;

    assign m_axi_awid           = wr_aw_id;
    assign m_axi_awaddr         = wr_aw_curr_addr;
    assign m_axi_awlen          = wr_aw_narrow ? 8'd0 : wr_aw_mi_len_m1[7:0];
    assign m_axi_awsize         = wr_aw_narrow ? 3'd2 : 3'd5;
    assign m_axi_awburst        = 2'b01;  // INCR
    assign m_axi_awvalid        = wr_aw_act && wrseg_in_ready;

    wire m_aw_fire;
    assign m_aw_fire = m_axi_awvalid && m_axi_awready;

    // =========================================================
    // Write data serializer (512 -> 2 x 256)
    // =========================================================
    reg          wr_seg_act;
    reg          wr_seg_narrow;
    reg          wr_seg_upper;
    reg  [  7:0] wr_seg_beats_left;  // in SI beats

    reg          wr_buf_valid;
    reg          wr_buf_half;  // 0: low256 pending, 1: high256 pending
    reg  [511:0] wr_buf_data;
    reg  [ 63:0] wr_buf_strb;

    wire         s_w_fire;
    wire         m_w_fire;

    // Refill on the last half of a non-final full-width beat: sustain
    // two MI beats per SI beat without an extra empty-buffer cycle.
    assign s_axi_wready = wr_seg_act && (!wr_buf_valid ||
        (m_axi_wready && wr_buf_half && !wr_seg_narrow && wr_seg_beats_left > 8'd1));

    assign m_axi_wvalid = wr_buf_valid;
    assign m_axi_wdata  = wr_buf_half ? wr_buf_data[511:256] : wr_buf_data[255:0];
    assign m_axi_wstrb  = wr_buf_half ? wr_buf_strb[63:32] : wr_buf_strb[31:0];
    assign m_axi_wlast  = wr_buf_valid && (wr_buf_half || wr_seg_narrow) && (wr_seg_beats_left == 8'd1);

    assign s_w_fire     = s_axi_wvalid && s_axi_wready;
    assign m_w_fire     = m_axi_wvalid && m_axi_wready;

    // =========================================================
    // Write response coalescer
    // =========================================================
    wire            bcf_split;
    wire [ID_W-1:0] bcf_id;

    reg             b_rsp_act;
    reg  [ID_W-1:0] b_rsp_id;
    reg  [     1:0] b_rsp_remain;
    reg  [     1:0] b_rsp_agg;

    wire            b_final_resp;
    wire            m_b_fire;

    assign bcf_split    = b_cmd_out_data[0];
    assign bcf_id       = b_cmd_out_data[1+:ID_W];

    assign b_final_resp = (b_rsp_remain == 2'd1);
    assign m_axi_bready = b_rsp_act && (!b_final_resp || !s_axi_bvalid || s_axi_bready);
    assign m_b_fire     = m_axi_bvalid && m_axi_bready;

    // =========================================================
    // Read address issue engine (SI AR -> split MI AR)
    // =========================================================
    wire [       7:0] arf_len;
    wire [ADDR_W-1:0] arf_addr;
    wire [  ID_W-1:0] arf_id;

    reg               rd_ar_act;
    reg  [  ID_W-1:0] rd_ar_id;
    reg  [ADDR_W-1:0] rd_ar_addr_base;
    reg  [       7:0] rd_ar_seg0_beats;
    reg  [       7:0] rd_ar_seg1_beats;
    reg               rd_ar_seg_idx;

    wire [       7:0] rd_ar_curr_seg_beats;
    wire              rd_ar_last_seg;
    wire [ADDR_W-1:0] rd_ar_curr_addr;
    wire [       8:0] rd_ar_mi_len_m1;
    wire              m_ar_fire;

    assign arf_len              = ar_cmd_out_data[7:0];
    assign arf_addr             = ar_cmd_out_data[8+:ADDR_W];
    assign arf_id               = ar_cmd_out_data[8+ADDR_W+:ID_W];

    assign rd_ar_curr_seg_beats = rd_ar_seg_idx ? rd_ar_seg1_beats : rd_ar_seg0_beats;
    assign rd_ar_last_seg       = rd_ar_seg_idx ? 1'b1 : (rd_ar_seg1_beats == 8'd0);

    assign rd_ar_curr_addr      = rd_ar_addr_base + (rd_ar_seg_idx ? {{(ADDR_W - 14) {1'b0}}, rd_ar_seg0_beats, 6'b0} : {ADDR_W{1'b0}});

    assign rd_ar_mi_len_m1      = {rd_ar_curr_seg_beats, 1'b0} - 9'd1;

    assign m_axi_arid           = rd_ar_id;
    assign m_axi_araddr         = rd_ar_curr_addr;
    assign m_axi_arlen          = rd_ar_mi_len_m1[7:0];
    assign m_axi_arsize         = 3'd5;  // 32 bytes
    assign m_axi_arburst        = 2'b01;  // INCR
    assign m_axi_arvalid        = rd_ar_act;

    assign m_ar_fire            = m_axi_arvalid && m_axi_arready;

    // =========================================================
    // Read response merger (2 x 256 -> 512)
    // =========================================================
    wire [     7:0] rcf_len;
    wire [ID_W-1:0] rcf_id;

    reg             r_rsp_act;
    reg  [ID_W-1:0] r_rsp_id;
    reg  [     8:0] r_rsp_beats_left;  // in SI beats

    reg             r_half_valid;
    reg  [   255:0] r_half_data;
    reg  [     1:0] r_half_resp;

    wire            m_r_fire;

    assign rcf_len      = r_cmd_out_data[7:0];
    assign rcf_id       = r_cmd_out_data[8+:ID_W];

    assign m_axi_rready = r_rsp_act && (!r_half_valid || (r_half_valid && (!s_axi_rvalid || s_axi_rready)));

    assign m_r_fire     = m_axi_rvalid && m_axi_rready;

    // =========================================================
    // Temporaries
    // =========================================================
    wire [8:0] awf_beats = {1'b0, awf_len} + 9'd1;
    wire [8:0] arf_beats = {1'b0, arf_len} + 9'd1;

    // FIFO data and VALID/READY belong to the SAME clock edge. Delaying
    // VALID while leaving input data live captures the next/cleared request.
    assign aw_cmd_in_valid = s_axi_awvalid && s_axi_awready;
    assign b_cmd_in_valid = aw_cmd_in_valid;
    assign ar_cmd_in_valid = s_axi_arvalid && s_axi_arready;
    assign r_cmd_in_valid = ar_cmd_in_valid;
    assign aw_cmd_out_ready = !wr_aw_act;
    assign ar_cmd_out_ready = !rd_ar_act;
    assign b_cmd_out_ready = !b_rsp_act && !s_axi_bvalid;
    assign r_cmd_out_ready = !r_rsp_act;
    assign wrseg_out_ready = !wr_seg_act;
    assign wrseg_in_valid = m_aw_fire;
    assign wrseg_in_data = {wr_aw_narrow, wr_aw_addr_base[5], wr_aw_curr_seg_beats};

    // =========================================================
    // Sequential
    // =========================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wr_aw_act         <= 1'b0;
            wr_aw_narrow      <= 1'b0;
            wr_aw_id          <= {ID_W{1'b0}};
            wr_aw_addr_base   <= {ADDR_W{1'b0}};
            wr_aw_seg0_beats  <= 8'd0;
            wr_aw_seg1_beats  <= 8'd0;
            wr_aw_seg_idx     <= 1'b0;

            wr_seg_act        <= 1'b0;
            wr_seg_narrow     <= 1'b0;
            wr_seg_upper      <= 1'b0;
            wr_seg_beats_left <= 8'd0;
            wr_buf_valid      <= 1'b0;
            wr_buf_half       <= 1'b0;
            wr_buf_data       <= 512'd0;
            wr_buf_strb       <= 64'd0;

            b_rsp_act         <= 1'b0;
            b_rsp_id          <= {ID_W{1'b0}};
            b_rsp_remain      <= 2'd0;
            b_rsp_agg         <= 2'b00;
            s_axi_bvalid      <= 1'b0;
            s_axi_bid         <= {ID_W{1'b0}};
            s_axi_bresp       <= 2'b00;

            rd_ar_act         <= 1'b0;
            rd_ar_id          <= {ID_W{1'b0}};
            rd_ar_addr_base   <= {ADDR_W{1'b0}};
            rd_ar_seg0_beats  <= 8'd0;
            rd_ar_seg1_beats  <= 8'd0;
            rd_ar_seg_idx     <= 1'b0;

            r_rsp_act         <= 1'b0;
            r_rsp_id          <= {ID_W{1'b0}};
            r_rsp_beats_left  <= 9'd0;
            r_half_valid      <= 1'b0;
            r_half_data       <= 256'd0;
            r_half_resp       <= 2'b00;

            s_axi_rvalid      <= 1'b0;
            s_axi_rid         <= {ID_W{1'b0}};
            s_axi_rdata       <= 512'd0;
            s_axi_rresp       <= 2'b00;
            s_axi_rlast       <= 1'b0;

        end else begin
            // -------------------------------------------------
            // Write AW issue engine
            // -------------------------------------------------
            if (!wr_aw_act && aw_cmd_out_valid) begin
                wr_aw_act        <= 1'b1;
                wr_aw_id         <= awf_id;
                wr_aw_narrow     <= aw_cmd_out_data[AW_CMD_W-1];
                wr_aw_addr_base  <= awf_addr;
                wr_aw_seg_idx    <= 1'b0;

                if (awf_beats > MAX_SI_SEG_BEATS) begin
                    wr_aw_seg0_beats <= 8'd128;
                    wr_aw_seg1_beats <= awf_beats[7:0] - 8'd128;
                end else begin
                    wr_aw_seg0_beats <= awf_beats[7:0];
                    wr_aw_seg1_beats <= 8'd0;
                end
            end else if (m_aw_fire) begin
                if (wr_aw_last_seg) begin
                    wr_aw_act <= 1'b0;
                end else begin
                    wr_aw_seg_idx <= 1'b1;
                end
            end

            // -------------------------------------------------
            // Write segment FIFO -> W serializer segment context
            // -------------------------------------------------
            if (!wr_seg_act && wrseg_out_valid) begin
                wr_seg_act        <= 1'b1;
                wr_seg_beats_left <= wrseg_out_data[7:0];
                wr_seg_narrow     <= wrseg_out_data[9];
                wr_seg_upper      <= wrseg_out_data[8];
            end

            // emit 2 MI W beats
            if (m_w_fire) begin
                if (!wr_buf_half && !wr_seg_narrow) begin
                    wr_buf_half <= 1'b1;
                end else begin
                    wr_buf_valid <= 1'b0;
                    wr_buf_half  <= 1'b0;

                    if (wr_seg_beats_left == 8'd1) begin
                        wr_seg_act        <= 1'b0;
                        wr_seg_beats_left <= 8'd0;
                    end else begin
                        wr_seg_beats_left <= wr_seg_beats_left - 8'd1;
                    end
                end
            end

            // accept one SI W beat into local 512-bit buffer
            if (s_w_fire) begin
                wr_buf_valid <= 1'b1;
                wr_buf_half  <= wr_seg_narrow && wr_seg_upper;
                wr_buf_data  <= s_axi_wdata;
                wr_buf_strb  <= s_axi_wstrb;
            end

            // -------------------------------------------------
            // B command loader
            // -------------------------------------------------
            if (!b_rsp_act && !s_axi_bvalid && b_cmd_out_valid) begin
                b_rsp_act       <= 1'b1;
                b_rsp_id        <= bcf_id;
                b_rsp_remain    <= bcf_split ? 2'd2 : 2'd1;
                b_rsp_agg       <= 2'b00;  // OKAY
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (m_b_fire) begin
                if (b_rsp_remain == 2'd1) begin
                    s_axi_bvalid <= 1'b1;
                    s_axi_bid    <= b_rsp_id;
                    s_axi_bresp  <= worst_resp(b_rsp_agg, m_axi_bresp);

                    b_rsp_act    <= 1'b0;
                    b_rsp_remain <= 2'd0;
                end else begin
                    b_rsp_agg    <= worst_resp(b_rsp_agg, m_axi_bresp);
                    b_rsp_remain <= b_rsp_remain - 2'd1;
                end
            end

            // -------------------------------------------------
            // Read AR issue engine
            // -------------------------------------------------
            if (!rd_ar_act && ar_cmd_out_valid) begin
                rd_ar_act        <= 1'b1;
                rd_ar_id         <= arf_id;
                rd_ar_addr_base  <= arf_addr;
                rd_ar_seg_idx    <= 1'b0;

                if (arf_beats > MAX_SI_SEG_BEATS) begin
                    rd_ar_seg0_beats <= 8'd128;
                    rd_ar_seg1_beats <= arf_beats[7:0] - 8'd128;
                end else begin
                    rd_ar_seg0_beats <= arf_beats[7:0];
                    rd_ar_seg1_beats <= 8'd0;
                end
            end else if (m_ar_fire) begin
                if (rd_ar_last_seg) begin
                    rd_ar_act <= 1'b0;
                end else begin
                    rd_ar_seg_idx <= 1'b1;
                end
            end

            // -------------------------------------------------
            // R command loader
            // -------------------------------------------------
            if (!r_rsp_act && r_cmd_out_valid) begin
                r_rsp_act        <= 1'b1;
                r_rsp_id         <= rcf_id;
                r_rsp_beats_left <= {1'b0, rcf_len} + 9'd1;
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end

            // Merge two 256b MI R beats into one 512b SI R beat
            if (m_r_fire) begin
                if (!r_half_valid) begin
                    r_half_valid <= 1'b1;
                    r_half_data  <= m_axi_rdata;
                    r_half_resp  <= m_axi_rresp;
                end else begin
                    r_half_valid <= 1'b0;

                    s_axi_rvalid <= 1'b1;
                    s_axi_rid    <= r_rsp_id;
                    s_axi_rdata  <= {m_axi_rdata, r_half_data};
                    s_axi_rresp  <= worst_resp(r_half_resp, m_axi_rresp);
                    s_axi_rlast  <= (r_rsp_beats_left == 9'd1);

                    if (r_rsp_beats_left == 9'd1) begin
                        r_rsp_act        <= 1'b0;
                        r_rsp_beats_left <= 9'd0;
                    end else begin
                        r_rsp_beats_left <= r_rsp_beats_left - 9'd1;
                    end
                end
            end
        end
    end

endmodule

// ============================================================================
// Simple synchronous FIFO
// - Vivado friendly
// - Small depth command FIFOs / segment FIFOs
// ============================================================================
module sync_fifo_simple #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 4
) (
    input  wire             clk,
    input  wire             rstn,
    input  wire [WIDTH-1:0] in_data,
    input  wire             in_valid,
    output wire             in_ready,
    output wire [WIDTH-1:0] out_data,
    output wire             out_valid,
    input  wire             out_ready
);
    localparam integer AW = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
    localparam integer CW = $clog2(DEPTH + 1);

    localparam [CW-1:0] DEPTH_COUNT = DEPTH[CW-1:0];
    localparam integer LAST_INDEX = DEPTH-1;
    localparam [AW-1:0] LAST_PTR = LAST_INDEX[AW-1:0];

    reg  [WIDTH-1:0] mem    [0:DEPTH-1];
    reg  [   AW-1:0] wr_ptr;
    reg  [   AW-1:0] rd_ptr;
    reg  [   CW-1:0] count;

    wire             push;
    wire             pop;

    assign push      = in_valid && in_ready;
    assign pop       = out_valid && out_ready;

    assign in_ready  = (count < DEPTH_COUNT);
    assign out_valid = (count != 0);
    assign out_data  = mem[rd_ptr];

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            wr_ptr <= {AW{1'b0}};
            rd_ptr <= {AW{1'b0}};
            count  <= {CW{1'b0}};
        end else begin
            case ({
                push, pop
            })
                2'b10: begin
                    mem[wr_ptr] <= in_data;
                    if (wr_ptr == LAST_PTR) wr_ptr <= {AW{1'b0}};
                    else wr_ptr <= wr_ptr + {{(AW - 1) {1'b0}}, 1'b1};
                    count <= count + {{(CW - 1) {1'b0}}, 1'b1};
                end

                2'b01: begin
                    if (rd_ptr == LAST_PTR) rd_ptr <= {AW{1'b0}};
                    else rd_ptr <= rd_ptr + {{(AW - 1) {1'b0}}, 1'b1};
                    count <= count - {{(CW - 1) {1'b0}}, 1'b1};
                end

                2'b11: begin
                    mem[wr_ptr] <= in_data;

                    if (wr_ptr == LAST_PTR) wr_ptr <= {AW{1'b0}};
                    else wr_ptr <= wr_ptr + {{(AW - 1) {1'b0}}, 1'b1};

                    if (rd_ptr == LAST_PTR) rd_ptr <= {AW{1'b0}};
                    else rd_ptr <= rd_ptr + {{(AW - 1) {1'b0}}, 1'b1};

                    count <= count;
                end

                default: begin
                    count <= count;
                end
            endcase
        end
    end
endmodule
