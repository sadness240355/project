/* SPDX-License-Identifier: MIT
 * Timing-optimized AXI4-MM slave to PCIe requester bridge for UltraScale+ PCIe RQ/RC.
 *
 * This version keeps the external protocol and transaction policy of the uploaded
 * bridge, but removes helper subroutines and variable-index accesses from the
 * per-tag state table.  The long RC path is split by an input pipeline:
 * RC payload placement is derived from the completion descriptor dword count and
 * the fixed 12-byte RC descriptor offset.  The bridge no longer depends on
 * m_axis_rc_tuser byte_en for the middle data path; byte_en is often a long
 * timing path and is not needed for the full-width aligned AXI reads used here.
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module edev_axi_pcie_bridge #(
    parameter AXIS_PCIE_DATA_WIDTH = 256,
    parameter AXIS_PCIE_KEEP_WIDTH = AXIS_PCIE_DATA_WIDTH / 32,
    parameter AXIS_PCIE_RC_USER_WIDTH = 75,
    parameter AXIS_PCIE_RQ_USER_WIDTH = 62,
    parameter AXI_DATA_WIDTH = AXIS_PCIE_DATA_WIDTH,
    parameter AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8,
    parameter AXI_ADDR_WIDTH = 64,
    parameter AXI_ID_WIDTH = 4,
    parameter PCIE_ADDR_WIDTH = 64,
    parameter TABLE_ENTRIES = 4,
    parameter TABLE_ADDR_WIDTH = TABLE_ENTRIES * PCIE_ADDR_WIDTH,
    parameter READ_TAG_BASE = 8'h88,
    parameter READ_TAG_COUNT = 8,
    parameter READ_REQ_BYTES = 512,
    parameter READ_WINDOW_BYTES = 4096,
    parameter WRITE_TLP_BYTES = 16
) (
    input wire clk,
    input wire rst,
    input wire enable,

    input wire [   TABLE_ENTRIES-1:0] table_enable,
    input wire [TABLE_ADDR_WIDTH-1:0] table_axi_base,
    input wire [TABLE_ADDR_WIDTH-1:0] table_pcie_base,
    input wire [TABLE_ADDR_WIDTH-1:0] table_mask,

    input wire [15:0] requester_id,
    input wire        requester_id_enable,
    input wire        ext_tag_enable,
    input wire [ 2:0] max_read_request_size,

    input  wire [  AXI_ID_WIDTH-1:0] s_axi_awid,
    input  wire [AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire [               7:0] s_axi_awlen,
    input  wire [               2:0] s_axi_awsize,
    input  wire [               1:0] s_axi_awburst,
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,
    input  wire [AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input  wire [AXI_STRB_WIDTH-1:0] s_axi_wstrb,
    input  wire                      s_axi_wlast,
    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,
    output wire [  AXI_ID_WIDTH-1:0] s_axi_bid,
    output wire [               1:0] s_axi_bresp,
    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,

    input  wire [  AXI_ID_WIDTH-1:0] s_axi_arid,
    input  wire [AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire [               7:0] s_axi_arlen,
    input  wire [               2:0] s_axi_arsize,
    input  wire [               1:0] s_axi_arburst,
    input  wire                      s_axi_arvalid,
    output wire                      s_axi_arready,
    output wire [  AXI_ID_WIDTH-1:0] s_axi_rid,
    output wire [AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output wire [               1:0] s_axi_rresp,
    output wire                      s_axi_rlast,
    output wire                      s_axi_rvalid,
    input  wire                      s_axi_rready,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_rq_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_rq_tkeep,
    output wire                               m_axis_rq_tvalid,
    input  wire                               m_axis_rq_tready,
    output wire                               m_axis_rq_tlast,
    output wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] m_axis_rq_tuser,

    input  wire [   AXIS_PCIE_DATA_WIDTH-1:0] s_axis_rc_tdata,
    input  wire [   AXIS_PCIE_KEEP_WIDTH-1:0] s_axis_rc_tkeep,
    input  wire                               s_axis_rc_tvalid,
    output wire                               s_axis_rc_tready,
    input  wire                               s_axis_rc_tlast,
    input  wire [AXIS_PCIE_RC_USER_WIDTH-1:0] s_axis_rc_tuser,

    output wire status_busy,
    output wire status_error_cor,
    output wire status_error_uncor
);

    // Elastic slices cut payload and backwards READY propagation.
    wire [AXI_ID_WIDTH-1:0] core_awid;
    wire [AXI_ADDR_WIDTH-1:0] core_awaddr;
    wire [8-1:0] core_awlen;
    wire [3-1:0] core_awsize;
    wire [2-1:0] core_awburst;
    wire [AXI_DATA_WIDTH-1:0] core_wdata;
    wire [AXI_STRB_WIDTH-1:0] core_wstrb;
    wire [1-1:0] core_wlast;
    wire [AXI_ID_WIDTH-1:0] core_bid;
    wire [2-1:0] core_bresp;
    wire [AXI_ID_WIDTH-1:0] core_arid;
    wire [AXI_ADDR_WIDTH-1:0] core_araddr;
    wire [8-1:0] core_arlen;
    wire [3-1:0] core_arsize;
    wire [2-1:0] core_arburst;
    wire [AXI_ID_WIDTH-1:0] core_rid;
    wire [AXI_DATA_WIDTH-1:0] core_rdata;
    wire [2-1:0] core_rresp;
    wire [1-1:0] core_rlast;
    wire core_awvalid, core_awready, aw_pending;
    wire core_wvalid, core_wready, w_pending;
    wire core_bvalid, core_bready, b_pending;
    wire core_arvalid, core_arready, ar_pending;
    wire core_rvalid, core_rready, r_pending;
    wire core_busy;
    assign status_busy = core_busy | aw_pending | w_pending | b_pending | ar_pending | r_pending;
    wire aw_ready_internal;
    assign s_axi_awready = enable && aw_ready_internal;
    edev_bridge_channel_slice #(.WIDTH(AXI_ID_WIDTH + AXI_ADDR_WIDTH + 8 + 3 + 2)) aw_slice (
        .clk(clk), .rst(rst), .s_data({s_axi_awid, s_axi_awaddr, s_axi_awlen, s_axi_awsize, s_axi_awburst}), .s_valid(enable && s_axi_awvalid), .s_ready(aw_ready_internal),
        .m_data({core_awid, core_awaddr, core_awlen, core_awsize, core_awburst}), .m_valid(core_awvalid), .m_ready(core_awready), .pending(aw_pending));
    edev_bridge_channel_slice #(.WIDTH(AXI_DATA_WIDTH + AXI_STRB_WIDTH + 1)) w_slice (
        .clk(clk), .rst(rst), .s_data({s_axi_wdata, s_axi_wstrb, s_axi_wlast}), .s_valid(s_axi_wvalid), .s_ready(s_axi_wready),
        .m_data({core_wdata, core_wstrb, core_wlast}), .m_valid(core_wvalid), .m_ready(core_wready), .pending(w_pending));
    edev_bridge_channel_slice #(.WIDTH(AXI_ID_WIDTH + 2)) b_slice (
        .clk(clk), .rst(rst), .s_data({core_bid, core_bresp}), .s_valid(core_bvalid), .s_ready(core_bready),
        .m_data({s_axi_bid, s_axi_bresp}), .m_valid(s_axi_bvalid), .m_ready(s_axi_bready), .pending(b_pending));
    wire ar_ready_internal;
    assign s_axi_arready = enable && ar_ready_internal;
    edev_bridge_channel_slice #(.WIDTH(AXI_ID_WIDTH + AXI_ADDR_WIDTH + 8 + 3 + 2)) ar_slice (
        .clk(clk), .rst(rst), .s_data({s_axi_arid, s_axi_araddr, s_axi_arlen, s_axi_arsize, s_axi_arburst}), .s_valid(enable && s_axi_arvalid), .s_ready(ar_ready_internal),
        .m_data({core_arid, core_araddr, core_arlen, core_arsize, core_arburst}), .m_valid(core_arvalid), .m_ready(core_arready), .pending(ar_pending));
    edev_bridge_channel_slice #(.WIDTH(AXI_ID_WIDTH + AXI_DATA_WIDTH + 2 + 1)) r_slice (
        .clk(clk), .rst(rst), .s_data({core_rid, core_rdata, core_rresp, core_rlast}), .s_valid(core_rvalid), .s_ready(core_rready),
        .m_data({s_axi_rid, s_axi_rdata, s_axi_rresp, s_axi_rlast}), .m_valid(s_axi_rvalid), .m_ready(s_axi_rready), .pending(r_pending));

    edev_axi_pcie_bridge_core #(
        .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
        .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
        .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
        .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_STRB_WIDTH(AXI_STRB_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .PCIE_ADDR_WIDTH(PCIE_ADDR_WIDTH),
        .TABLE_ENTRIES(TABLE_ENTRIES),
        .TABLE_ADDR_WIDTH(TABLE_ADDR_WIDTH),
        .READ_TAG_BASE(READ_TAG_BASE),
        .READ_TAG_COUNT(READ_TAG_COUNT),
        .READ_REQ_BYTES(READ_REQ_BYTES),
        .READ_WINDOW_BYTES(READ_WINDOW_BYTES),
        .WRITE_TLP_BYTES(WRITE_TLP_BYTES)
    ) core_inst (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .table_enable(table_enable),
        .table_axi_base(table_axi_base),
        .table_pcie_base(table_pcie_base),
        .table_mask(table_mask),
        .requester_id(requester_id),
        .requester_id_enable(requester_id_enable),
        .ext_tag_enable(ext_tag_enable),
        .max_read_request_size(max_read_request_size),
        .s_axi_awid(core_awid),
        .s_axi_awaddr(core_awaddr),
        .s_axi_awlen(core_awlen),
        .s_axi_awsize(core_awsize),
        .s_axi_awburst(core_awburst),
        .s_axi_awvalid(core_awvalid),
        .s_axi_awready(core_awready),
        .s_axi_wdata(core_wdata),
        .s_axi_wstrb(core_wstrb),
        .s_axi_wlast(core_wlast),
        .s_axi_wvalid(core_wvalid),
        .s_axi_wready(core_wready),
        .s_axi_bid(core_bid),
        .s_axi_bresp(core_bresp),
        .s_axi_bvalid(core_bvalid),
        .s_axi_bready(core_bready),
        .s_axi_arid(core_arid),
        .s_axi_araddr(core_araddr),
        .s_axi_arlen(core_arlen),
        .s_axi_arsize(core_arsize),
        .s_axi_arburst(core_arburst),
        .s_axi_arvalid(core_arvalid),
        .s_axi_arready(core_arready),
        .s_axi_rid(core_rid),
        .s_axi_rdata(core_rdata),
        .s_axi_rresp(core_rresp),
        .s_axi_rlast(core_rlast),
        .s_axi_rvalid(core_rvalid),
        .s_axi_rready(core_rready),
        .m_axis_rq_tdata(m_axis_rq_tdata),
        .m_axis_rq_tkeep(m_axis_rq_tkeep),
        .m_axis_rq_tvalid(m_axis_rq_tvalid),
        .m_axis_rq_tready(m_axis_rq_tready),
        .m_axis_rq_tlast(m_axis_rq_tlast),
        .m_axis_rq_tuser(m_axis_rq_tuser),
        .s_axis_rc_tdata(s_axis_rc_tdata),
        .s_axis_rc_tkeep(s_axis_rc_tkeep),
        .s_axis_rc_tvalid(s_axis_rc_tvalid),
        .s_axis_rc_tready(s_axis_rc_tready),
        .s_axis_rc_tlast(s_axis_rc_tlast),
        .s_axis_rc_tuser(s_axis_rc_tuser),
        .status_busy(core_busy),
        .status_error_cor(status_error_cor),
        .status_error_uncor(status_error_uncor)
    );
endmodule

// Two-entry registered-READY elastic slice. Policy matches REG_TYPE=2 in
// the project's existing axis_register: one beat/cycle with safe skid capture.
module edev_bridge_channel_slice #(parameter WIDTH=1) (
    input wire clk, rst,
    input wire [WIDTH-1:0] s_data, input wire s_valid, output wire s_ready,
    output wire [WIDTH-1:0] m_data, output wire m_valid, input wire m_ready,
    output wire pending
);
    reg ready_reg=0, valid_reg=0, temp_valid_reg=0;
    reg [WIDTH-1:0] data_reg, temp_data_reg;
    reg valid_next, temp_valid_next, store_input, store_temp, move_temp;
    wire ready_next=m_ready || (!temp_valid_reg && (!valid_reg || !s_valid));
    assign s_ready=ready_reg; assign m_valid=valid_reg; assign m_data=data_reg;
    assign pending=valid_reg || temp_valid_reg;
    always @* begin
        valid_next=valid_reg; temp_valid_next=temp_valid_reg;
        store_input=0;store_temp=0;move_temp=0;
        if (ready_reg) begin
            if (m_ready || !valid_reg) begin valid_next=s_valid;store_input=1;end
            else begin temp_valid_next=s_valid;store_temp=1;end
        end else if (m_ready) begin
            valid_next=temp_valid_reg;temp_valid_next=0;move_temp=1;
        end
    end
    always @(posedge clk) begin
        if (rst) begin ready_reg<=0;valid_reg<=0;temp_valid_reg<=0;end
        else begin ready_reg<=ready_next;valid_reg<=valid_next;temp_valid_reg<=temp_valid_next;end
        if (store_input) data_reg<=s_data;
        else if (move_temp) data_reg<=temp_data_reg;
        if (store_temp) temp_data_reg<=s_data;
    end
endmodule

module edev_axi_pcie_bridge_core #(
    parameter AXIS_PCIE_DATA_WIDTH = 256,
    parameter AXIS_PCIE_KEEP_WIDTH = AXIS_PCIE_DATA_WIDTH / 32,
    parameter AXIS_PCIE_RC_USER_WIDTH = 75,
    parameter AXIS_PCIE_RQ_USER_WIDTH = 62,
    parameter AXI_DATA_WIDTH = AXIS_PCIE_DATA_WIDTH,
    parameter AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8,
    parameter AXI_ADDR_WIDTH = 64,
    parameter AXI_ID_WIDTH = 4,
    parameter PCIE_ADDR_WIDTH = 64,
    parameter TABLE_ENTRIES = 4,
    parameter TABLE_ADDR_WIDTH = TABLE_ENTRIES * PCIE_ADDR_WIDTH,
    parameter READ_TAG_BASE = 8'h88,
    parameter READ_TAG_COUNT = 8,
    parameter READ_REQ_BYTES = 512,
    parameter READ_WINDOW_BYTES = 4096,
    parameter WRITE_TLP_BYTES = 16
) (
    input wire clk,
    input wire rst,
    input wire enable,

    input wire [   TABLE_ENTRIES-1:0] table_enable,
    input wire [TABLE_ADDR_WIDTH-1:0] table_axi_base,
    input wire [TABLE_ADDR_WIDTH-1:0] table_pcie_base,
    input wire [TABLE_ADDR_WIDTH-1:0] table_mask,

    input wire [15:0] requester_id,
    input wire        requester_id_enable,
    input wire        ext_tag_enable,
    input wire [ 2:0] max_read_request_size,

    input  wire [  AXI_ID_WIDTH-1:0] s_axi_awid,
    input  wire [AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire [               7:0] s_axi_awlen,
    input  wire [               2:0] s_axi_awsize,
    input  wire [               1:0] s_axi_awburst,
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,
    input  wire [AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input  wire [AXI_STRB_WIDTH-1:0] s_axi_wstrb,
    input  wire                      s_axi_wlast,
    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,
    output wire [  AXI_ID_WIDTH-1:0] s_axi_bid,
    output wire [               1:0] s_axi_bresp,
    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,

    input  wire [  AXI_ID_WIDTH-1:0] s_axi_arid,
    input  wire [AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire [               7:0] s_axi_arlen,
    input  wire [               2:0] s_axi_arsize,
    input  wire [               1:0] s_axi_arburst,
    input  wire                      s_axi_arvalid,
    output wire                      s_axi_arready,
    output wire [  AXI_ID_WIDTH-1:0] s_axi_rid,
    output wire [AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output wire [               1:0] s_axi_rresp,
    output wire                      s_axi_rlast,
    output wire                      s_axi_rvalid,
    input  wire                      s_axi_rready,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_rq_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_rq_tkeep,
    output wire                               m_axis_rq_tvalid,
    input  wire                               m_axis_rq_tready,
    output wire                               m_axis_rq_tlast,
    output wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] m_axis_rq_tuser,

    input  wire [   AXIS_PCIE_DATA_WIDTH-1:0] s_axis_rc_tdata,
    input  wire [   AXIS_PCIE_KEEP_WIDTH-1:0] s_axis_rc_tkeep,
    input  wire                               s_axis_rc_tvalid,
    output wire                               s_axis_rc_tready,
    input  wire                               s_axis_rc_tlast,
    input  wire [AXIS_PCIE_RC_USER_WIDTH-1:0] s_axis_rc_tuser,

    output wire status_busy,
    output wire status_error_cor,
    output wire status_error_uncor
);

    initial begin
        if (AXIS_PCIE_DATA_WIDTH != 256) begin
            $error("edev_axi_pcie_bridge currently requires 256-bit PCIe AXI-ST (instance %m)");
            $finish;
        end
        if (AXI_DATA_WIDTH != 256) begin
            $error("edev_axi_pcie_bridge currently requires 256-bit AXI-MM slave data (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_RQ_USER_WIDTH != 60 && AXIS_PCIE_RQ_USER_WIDTH != 62) begin
            $error("edev_axi_pcie_bridge supports UltraScale+ 256-bit RQ tuser width 60/62 only (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_RC_USER_WIDTH != 75) begin
            $error("edev_axi_pcie_bridge supports UltraScale+ 256-bit RC tuser width 75 only (instance %m)");
            $finish;
        end
        if (READ_TAG_COUNT != 8 || READ_REQ_BYTES != 512 || READ_WINDOW_BYTES != 4096) begin
            $error("edev_axi_pcie_bridge requires 8 tags, max 512B MRd and 4KiB windows (instance %m)");
            $finish;
        end
        if (TABLE_ENTRIES != 4) begin
            $error("edev_axi_pcie_bridge expects four translation entries (instance %m)");
            $finish;
        end
        if (WRITE_TLP_BYTES != 16 && WRITE_TLP_BYTES != 32) begin
            $error("edev_axi_pcie_bridge supports 16B or 32B writes (instance %m)");
            $finish;
        end
    end

    localparam [3:0] REQ_MEM_READ = 4'b0000;
    localparam [3:0] REQ_MEM_WRITE = 4'b0001;
    localparam [1:0] AXI_RESP_OKAY = 2'b00;
    localparam [1:0] AXI_RESP_SLVERR = 2'b10;
    localparam [1:0] AXI_BURST_INCR = 2'b01;

    localparam integer AXI_BYTE_WIDTH = AXI_STRB_WIDTH;
    localparam integer AXI_SIZE_INT = $clog2(AXI_STRB_WIDTH);
    localparam [2:0] AXI_SIZE_VALUE = AXI_SIZE_INT[2:0];
    localparam [11:0] READ_REQ_BYTES_VALUE = READ_REQ_BYTES;
    localparam integer READ_WINDOW_BEATS = READ_WINDOW_BYTES / AXI_BYTE_WIDTH;

    localparam [7:0] TAG0 = READ_TAG_BASE + 8'd0;
    localparam [7:0] TAG1 = READ_TAG_BASE + 8'd1;
    localparam [7:0] TAG2 = READ_TAG_BASE + 8'd2;
    localparam [7:0] TAG3 = READ_TAG_BASE + 8'd3;
    localparam [7:0] TAG4 = READ_TAG_BASE + 8'd4;
    localparam [7:0] TAG5 = READ_TAG_BASE + 8'd5;
    localparam [7:0] TAG6 = READ_TAG_BASE + 8'd6;
    localparam [7:0] TAG7 = READ_TAG_BASE + 8'd7;

    localparam [4:0] S_IDLE = 5'd0;
    localparam [4:0] S_WR_LOOKUP = 5'd1;
    localparam [4:0] S_RD_LOOKUP = 5'd2;
    localparam [4:0] S_WR_DROP = 5'd3;
    localparam [4:0] S_WR_W = 5'd4;
    localparam [4:0] S_WR_RQ = 5'd5;
    localparam [4:0] S_WR_RESP = 5'd6;
    localparam [4:0] S_RD_NEXT = 5'd7;
    localparam [4:0] S_RD_WAVE = 5'd8;
    localparam [4:0] S_RD_ISSUE = 5'd9;
    localparam [4:0] S_RD_WAIT = 5'd10;
    localparam [4:0] S_RD_SEND = 5'd11;
    localparam [4:0] S_RD_RESP_ERR = 5'd12;
    localparam [4:0] S_LOOKUP_RESULT = 5'd13, S_LOOKUP_OFFSET = 5'd14, S_LOOKUP_TRANSLATE = 5'd15;

    wire [        PCIE_ADDR_WIDTH-1:0] table_axi_base0 = table_axi_base[PCIE_ADDR_WIDTH*1-1:PCIE_ADDR_WIDTH*0];
    wire [        PCIE_ADDR_WIDTH-1:0] table_axi_base1 = table_axi_base[PCIE_ADDR_WIDTH*2-1:PCIE_ADDR_WIDTH*1];
    wire [        PCIE_ADDR_WIDTH-1:0] table_axi_base2 = table_axi_base[PCIE_ADDR_WIDTH*3-1:PCIE_ADDR_WIDTH*2];
    wire [        PCIE_ADDR_WIDTH-1:0] table_axi_base3 = table_axi_base[PCIE_ADDR_WIDTH*4-1:PCIE_ADDR_WIDTH*3];
    wire [        PCIE_ADDR_WIDTH-1:0] table_pcie_base0 = table_pcie_base[PCIE_ADDR_WIDTH*1-1:PCIE_ADDR_WIDTH*0];
    wire [        PCIE_ADDR_WIDTH-1:0] table_pcie_base1 = table_pcie_base[PCIE_ADDR_WIDTH*2-1:PCIE_ADDR_WIDTH*1];
    wire [        PCIE_ADDR_WIDTH-1:0] table_pcie_base2 = table_pcie_base[PCIE_ADDR_WIDTH*3-1:PCIE_ADDR_WIDTH*2];
    wire [        PCIE_ADDR_WIDTH-1:0] table_pcie_base3 = table_pcie_base[PCIE_ADDR_WIDTH*4-1:PCIE_ADDR_WIDTH*3];
    wire [        PCIE_ADDR_WIDTH-1:0] table_mask0 = table_mask[PCIE_ADDR_WIDTH*1-1:PCIE_ADDR_WIDTH*0];
    wire [        PCIE_ADDR_WIDTH-1:0] table_mask1 = table_mask[PCIE_ADDR_WIDTH*2-1:PCIE_ADDR_WIDTH*1];
    wire [        PCIE_ADDR_WIDTH-1:0] table_mask2 = table_mask[PCIE_ADDR_WIDTH*3-1:PCIE_ADDR_WIDTH*2];
    wire [        PCIE_ADDR_WIDTH-1:0] table_mask3 = table_mask[PCIE_ADDR_WIDTH*4-1:PCIE_ADDR_WIDTH*3];

    reg  [                        4:0] state_reg = S_IDLE;

    reg  [           AXI_ID_WIDTH-1:0] op_id_reg = {AXI_ID_WIDTH{1'b0}};
    reg  [         AXI_ADDR_WIDTH-1:0] op_axi_addr_reg = {AXI_ADDR_WIDTH{1'b0}};
    reg  [        PCIE_ADDR_WIDTH-1:0] op_pcie_addr_reg = {PCIE_ADDR_WIDTH{1'b0}};
    reg  [        PCIE_ADDR_WIDTH-1:0] trans_pcie_base_reg = {PCIE_ADDR_WIDTH{1'b0}};
    reg  [        PCIE_ADDR_WIDTH-1:0] trans_axi_base_reg = {PCIE_ADDR_WIDTH{1'b0}};
    reg  [                       15:0] op_beats_left_reg = 16'd0;
    reg  [                       15:0] op_total_beats_reg = 16'd0;
    reg  [                       15:0] op_return_beats_left_reg = 16'd0;
    reg  [                        1:0] op_resp_reg = AXI_RESP_OKAY;

    reg  [                        1:0] lookup_idx_reg = 2'd0;
    reg                                lookup_is_read_reg = 1'b0;

    reg  [         AXI_DATA_WIDTH-1:0] wr_data_reg = {AXI_DATA_WIDTH{1'b0}};
    reg                                wr_data_valid_reg = 1'b0;
    reg                                wr_half_reg = 1'b0;
    reg [31:0] wr_strb_reg = 32'd0;
    reg [2:0] wr_dw_reg = 3'd0;
    reg wr_sparse_reg = 1'b0;
    reg                                wr_last_reg = 1'b0;
    reg  [                       15:0] wr_beat_count_reg = 16'd0;
    // Registered W lookahead avoids a PCIe request bubble per 32-byte beat.
    reg  [AXI_DATA_WIDTH-1:0] wr_next_data_reg;
    reg  [AXI_STRB_WIDTH-1:0] wr_next_strb_reg;
    reg                       wr_next_last_reg = 1'b0;
    reg                       wr_next_valid_reg = 1'b0;

    reg  [           AXI_ID_WIDTH-1:0] b_id_reg = {AXI_ID_WIDTH{1'b0}};
    reg  [                        1:0] b_resp_reg = AXI_RESP_OKAY;
    reg                                b_valid_reg = 1'b0;

    reg  [           AXI_ID_WIDTH-1:0] r_id_reg = {AXI_ID_WIDTH{1'b0}};
    reg  [         AXI_DATA_WIDTH-1:0] r_data_reg;
    reg                                r_zero_reg = 1'b0;
    reg  [                        1:0] r_resp_reg = AXI_RESP_OKAY;
    reg                                r_last_reg = 1'b0;
    reg                                r_valid_reg = 1'b0;

    reg  [   AXIS_PCIE_DATA_WIDTH-1:0] rq_tdata_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg  [   AXIS_PCIE_KEEP_WIDTH-1:0] rq_tkeep_reg = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
    reg                                rq_tvalid_reg = 1'b0;
    reg                                rq_tlast_reg = 1'b0;
    reg  [AXIS_PCIE_RQ_USER_WIDTH-1:0] rq_tuser_reg = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};

    (* ram_style = "block" *)reg  [         AXI_DATA_WIDTH-1:0] rd_buf_mem                                                                [0:READ_WINDOW_BEATS-1];
    reg                                rd_buf_wr_en_reg = 1'b0;
    reg  [                        6:0] rd_buf_wr_addr_reg = 7'd0;
    reg  [         AXI_DATA_WIDTH-1:0] rd_buf_wr_data_reg = {AXI_DATA_WIDTH{1'b0}};
    reg                                rd_buf_tail_wr_pending_reg = 1'b0;
    reg  [                        6:0] rd_buf_tail_wr_addr_reg = 7'd0;
    reg  [         AXI_DATA_WIDTH-1:0] rd_buf_tail_wr_data_reg = {AXI_DATA_WIDTH{1'b0}};
    reg  [                       12:0] rd_window_bytes_reg = 13'd0;
    /* A full 4 KiB window contains 128 32-byte AXI beats.  The count needs
     * eight bits even though the RAM index itself only needs seven. */
    reg  [                        7:0] rd_window_beats_reg = 8'd0;
    reg  [                        6:0] rd_return_idx_reg = 7'd0;
    reg  [                        2:0] rd_issue_idx_reg = 3'd0;
    reg  [                        3:0] rd_issue_count_reg = 4'd0;
    reg  [                        7:0] rd_issue_mask_reg = 8'd0;
    reg  [                        7:0] rd_done_mask_reg = 8'd0;
    reg  [                       15:0] rd_global_returned_beats_reg = 16'd0;
    reg  [                       11:0] rd_req_bytes_reg = READ_REQ_BYTES_VALUE;
    reg  [                       12:0] rd_window_issued_bytes_reg = 13'd0;
    reg  [                       12:0] rd_wave_bytes_reg = 13'd0;
    reg  [                        6:0] rd_wave_base_beat_reg = 7'd0;

    reg [11:0] tag0_req_bytes_reg = 12'd0, tag1_req_bytes_reg = 12'd0, tag2_req_bytes_reg = 12'd0, tag3_req_bytes_reg = 12'd0;
    reg [11:0] tag4_req_bytes_reg = 12'd0, tag5_req_bytes_reg = 12'd0, tag6_req_bytes_reg = 12'd0, tag7_req_bytes_reg = 12'd0;
    reg [11:0] tag0_recv_bytes_reg = 12'd0, tag1_recv_bytes_reg = 12'd0, tag2_recv_bytes_reg = 12'd0, tag3_recv_bytes_reg = 12'd0;
    reg [11:0] tag4_recv_bytes_reg = 12'd0, tag5_recv_bytes_reg = 12'd0, tag6_recv_bytes_reg = 12'd0, tag7_recv_bytes_reg = 12'd0;
    reg [6:0] tag0_buf_base_reg = 7'd0, tag1_buf_base_reg = 7'd0, tag2_buf_base_reg = 7'd0, tag3_buf_base_reg = 7'd0;
    reg [6:0] tag4_buf_base_reg = 7'd0, tag5_buf_base_reg = 7'd0, tag6_buf_base_reg = 7'd0, tag7_buf_base_reg = 7'd0;
    reg  [  7:0] tag_error_reg = 8'd0;

    reg          rc_cpl_active_reg = 1'b0;
    reg  [  2:0] rc_cpl_tag_idx_reg = 3'd0;
    reg  [  3:0] rc_cpl_error_reg = 4'd0;
    reg  [  6:0] rc_cpl_word_base_reg = 7'd0;
    reg  [  6:0] rc_cpl_write_word_reg = 7'd0;
    reg  [  2:0] rc_cpl_carry_dw_reg = 3'd0;
    reg  [255:0] rc_cpl_carry_data_reg = 256'd0;
    reg  [ 11:0] rc_cpl_payload_bytes_reg = 12'd0;
    reg  [ 11:0] rc_cpl_recv_bytes_reg = 12'd0;

    reg          rc_s0_valid_reg = 1'b0;
    reg          rc_s0_tlast_reg = 1'b0;
    reg          rc_s0_sof_reg = 1'b0;
    reg          rc_s0_discontinue_reg = 1'b0;
    reg  [  7:0] rc_s0_tag_reg = 8'd0;
    reg  [  3:0] rc_s0_error_reg = 4'd0;
    reg          rc_s0_completed_reg = 1'b0;
    reg  [ 12:0] rc_s0_byte_count_reg = 13'd0;
    reg  [ 12:0] rc_s0_desc_byte_count_reg = 13'd0;
    reg  [  1:0] rc_s0_lower_addr_reg = 2'd0;
    reg  [255:0] rc_s0_payload_reg = 256'd0;

    /* RC decode stage 1.  The expensive descriptor-dword-count path is
     * terminated here: dword_count/lower_addr/byte_count are converted into
     * completion-local payload byte count and the request-window word base.
     * The following packer stage no longer reads descriptor dword_count, which
     * removes the long path from the RC descriptor register to rd_buf_mem/tag
     * completion state updates while still accepting one RC beat per cycle.
     */
    reg          rc_s1_valid_reg = 1'b0;
    reg          rc_s1_tlast_reg = 1'b0;
    reg          rc_s1_pkt_first_reg = 1'b0;
    reg          rc_s1_discontinue_reg = 1'b0;
    reg          rc_s1_sel_valid_reg = 1'b0;
    reg  [  2:0] rc_s1_tag_idx_reg = 3'd0;
    reg  [  3:0] rc_s1_error_reg = 4'd0;
    reg  [ 11:0] rc_s1_sel_recv_bytes_reg = 12'd0;
    reg  [ 11:0] rc_s1_payload_bytes_reg = 12'd0;
    reg  [  6:0] rc_s1_word_base_reg = 7'd0;
    reg  [255:0] rc_s1_payload_reg = 256'd0;

    reg          status_error_cor_reg = 1'b0;
    reg          status_error_uncor_reg = 1'b0;

    wire         rq_fire = rq_tvalid_reg && m_axis_rq_tready;
    wire         rq_available = !rq_tvalid_reg || rq_fire;
    wire         r_fire = r_valid_reg && s_axi_rready;
    wire         b_fire = b_valid_reg && s_axi_bready;
    wire         rc_accept = s_axis_rc_tvalid && s_axis_rc_tready;
    wire         cfg_ext_tag_ok = (READ_TAG_BASE < 8'd32) || ext_tag_enable;

    assign s_axi_awready      = enable && (state_reg == S_IDLE) && !b_valid_reg && !r_valid_reg && !rq_tvalid_reg;
    assign s_axi_arready      = enable && (state_reg == S_IDLE) && !b_valid_reg && !r_valid_reg && !rq_tvalid_reg && !s_axi_awvalid;
    assign s_axi_wready = (state_reg == S_WR_DROP) ||
        ((state_reg == S_WR_W) && !wr_next_valid_reg && !wr_data_valid_reg) ||
        ((WRITE_TLP_BYTES == 32) && (state_reg == S_WR_RQ) &&
         wr_data_valid_reg && !wr_next_valid_reg && !wr_last_reg &&
         (op_beats_left_reg > 16'd1));

    assign s_axi_bid          = b_id_reg;
    assign s_axi_bresp        = b_resp_reg;
    assign s_axi_bvalid       = b_valid_reg;
    assign s_axi_rid          = r_id_reg;
    assign s_axi_rdata        = r_zero_reg ? {AXI_DATA_WIDTH{1'b0}} : r_data_reg;
    assign s_axi_rresp        = r_resp_reg;
    assign s_axi_rlast        = r_last_reg;
    assign s_axi_rvalid       = r_valid_reg;

    assign m_axis_rq_tdata    = rq_tdata_reg;
    assign m_axis_rq_tkeep    = rq_tkeep_reg;
    assign m_axis_rq_tvalid   = rq_tvalid_reg;
    assign m_axis_rq_tlast    = rq_tlast_reg;
    assign m_axis_rq_tuser    = rq_tuser_reg;

    assign s_axis_rc_tready   = (state_reg == S_RD_ISSUE || state_reg == S_RD_WAIT) && !rd_buf_tail_wr_pending_reg;
    assign status_busy        = state_reg != S_IDLE || rq_tvalid_reg || r_valid_reg || b_valid_reg || rc_s0_valid_reg || rc_s1_valid_reg || rc_cpl_active_reg || rd_buf_wr_en_reg || rd_buf_tail_wr_pending_reg;
    assign status_error_cor   = status_error_cor_reg;
    assign status_error_uncor = status_error_uncor_reg;

    reg [              127:0] rq_hdr_tmp;
    reg [PCIE_ADDR_WIDTH-1:0] tbl_axi_base_tmp;
    reg [PCIE_ADDR_WIDTH-1:0] tbl_pcie_base_tmp;
    reg [PCIE_ADDR_WIDTH-1:0] tbl_mask_tmp;
    reg [PCIE_ADDR_WIDTH-1:0] translated_tmp;
    reg                       lookup_hit_tmp;
    reg lookup_hit_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0] lookup_axi_base_reg=0, lookup_pcie_base_reg=0, lookup_offset_reg=0;
    reg [               15:0] rd_bytes_left_tmp;
    reg [               15:0] page_left_tmp;
    reg [               15:0] calc16_tmp;
    reg [               12:0] win_bytes_tmp;
    reg [                7:0] win_beats_tmp;
    reg [               12:0] rd_wave_left_tmp;
    reg [               12:0] rd_wave_max_tmp;
    reg [               12:0] rd_wave_next_tmp;
    reg [               12:0] rd_wave_done_tmp;
    reg [               12:0] rd_issue_offset_tmp;
    reg [PCIE_ADDR_WIDTH-1:0] rd_issue_addr_tmp;
    reg [               12:0] rd_issue_left_tmp;
    reg [               11:0] seg_bytes_tmp;
    reg [               12:0] tag_count_adj_tmp;
    reg [                6:0] word_index_tmp;
    reg [PCIE_ADDR_WIDTH-1:0] wr_addr_tmp;

    reg                       rc_pkt_first_tmp;
    reg [                2:0] rc_sel_tag_idx_tmp;
    reg                       rc_sel_valid_tmp;
    reg [               11:0] rc_sel_req_bytes_tmp;
    reg [               11:0] rc_sel_recv_bytes_tmp;
    reg [                6:0] rc_sel_buf_base_tmp;
    reg [               12:0] rc_payload_capacity_tmp;
    reg [               11:0] rc_cpl_payload_bytes_tmp;
    reg [               12:0] rc_cpl_start_byte_tmp;
    reg [                6:0] rc_proc_word_base_tmp;
    reg [                6:0] rc_proc_write_word_tmp;
    reg [                2:0] rc_proc_carry_dw_tmp;
    reg [              255:0] rc_proc_carry_data_tmp;
    reg [               11:0] rc_proc_payload_bytes_tmp;
    reg [               11:0] rc_proc_recv_bytes_tmp;
    reg [              511:0] rc_combined_dw_tmp;
    reg [                3:0] rc_total_dw_tmp;
    reg [               11:0] rc_recv_inc_tmp;
    reg [                6:0] rc_next_write_word_tmp;
    reg [                2:0] rc_next_carry_dw_tmp;
    reg [              255:0] rc_next_carry_data_tmp;
    reg [               11:0] rc_next_recv_bytes_tmp;
    reg [               12:0] rc_tag_recv_total_tmp;
    reg [                3:0] rc_proc_error_tmp;
    reg [               11:0] rc_payload_left_bytes_tmp;
    reg [               10:0] rc_payload_left_dw_tmp;
    reg [                3:0] rc_payload_dw_count_tmp;
    reg [              255:0] rc_payload_aligned_tmp;
    reg                       rc_full_word_written_tmp;

    // One synchronous RAM read port with one address/enable expression.
    // Keeping error-data selection and the FSM outside this port permits the
    // read register to be absorbed into block RAM instead of a wide LUTRAM mux.
    // READY-low holds the RAM output.  A consumed beat prefetches the next beat
    // on the same edge, preserving one 256-bit AXI R beat per cycle.
    wire rd_buf_read_next = r_valid_reg && r_fire &&
        (op_return_beats_left_reg != 16'd1) &&
        ({1'b0, rd_return_idx_reg} + 8'd1 < rd_window_beats_reg);
    wire rd_buf_read_en = !rst && (state_reg == S_RD_SEND) &&
        (!r_valid_reg || rd_buf_read_next);
    wire [6:0] rd_buf_read_addr = r_valid_reg ?
        (rd_return_idx_reg + 7'd1) : rd_return_idx_reg;
    wire rd_buf_write_en = rd_buf_wr_en_reg || rd_buf_tail_wr_pending_reg;
    wire [6:0] rd_buf_write_addr = rd_buf_wr_en_reg ?
        rd_buf_wr_addr_reg : rd_buf_tail_wr_addr_reg;
    wire [AXI_DATA_WIDTH-1:0] rd_buf_write_data = rd_buf_wr_en_reg ?
        rd_buf_wr_data_reg : rd_buf_tail_wr_data_reg;

    always @(posedge clk) begin
        if (rd_buf_write_en)
            rd_buf_mem[rd_buf_write_addr] <= rd_buf_write_data;
        if (rd_buf_read_en)
            r_data_reg <= rd_buf_mem[rd_buf_read_addr];
    end

    always @(posedge clk) begin
        status_error_cor_reg   <= 1'b0;
        status_error_uncor_reg <= 1'b0;
        rc_s0_valid_reg        <= 1'b0;

        if (rq_fire) begin
            rq_tvalid_reg <= 1'b0;
        end
        if (b_fire) begin
            b_valid_reg <= 1'b0;
        end
        if (r_fire) begin
            r_valid_reg <= 1'b0;
        end

        if (!rd_buf_wr_en_reg && rd_buf_tail_wr_pending_reg)
            rd_buf_tail_wr_pending_reg <= 1'b0;
        rd_buf_wr_en_reg <= 1'b0;

        /* RC stage 0: capture the incoming beat.  Keep this stage narrow and
         * avoid any descriptor arithmetic here so the PCIe RC pins only feed
         * local registers in this cycle.
         */
        if (rc_accept) begin
            rc_s0_valid_reg           <= 1'b1;
            rc_s0_tlast_reg           <= s_axis_rc_tlast;
            rc_s0_sof_reg             <= s_axis_rc_tuser[32];
            rc_s0_discontinue_reg     <= s_axis_rc_tuser[42];
            rc_s0_tag_reg             <= s_axis_rc_tdata[71:64];
            rc_s0_error_reg           <= s_axis_rc_tdata[15:12];
            rc_s0_completed_reg       <= s_axis_rc_tdata[30];
            rc_s0_byte_count_reg      <= s_axis_rc_tdata[28:16];
            rc_s0_desc_byte_count_reg <= {s_axis_rc_tdata[42:32], 2'b00};
            rc_s0_lower_addr_reg      <= s_axis_rc_tdata[1:0];
            rc_s0_payload_reg         <= s_axis_rc_tdata;
        end

        /* RC stage 1: descriptor-local decode.  This stage is the only place
         * that consumes the descriptor byte-count register.  The result is registered as
         * rc_s1_payload_bytes_reg and rc_s1_word_base_reg before the data packer
         * and tag completion logic, cutting the former high-fanout long path.
         */
        rc_s1_valid_reg <= rc_s0_valid_reg;
        if (rc_s0_valid_reg) begin
            rc_s1_tlast_reg       <= rc_s0_tlast_reg;
            rc_s1_pkt_first_reg   <= rc_s0_sof_reg;
            rc_s1_discontinue_reg <= rc_s0_discontinue_reg;
            rc_s1_error_reg       <= rc_s0_error_reg;
            rc_s1_payload_reg     <= rc_s0_sof_reg ? {96'd0, rc_s0_payload_reg[255:96]} : rc_s0_payload_reg;

            rc_sel_tag_idx_tmp    = 3'd0;
            rc_sel_req_bytes_tmp  = 12'd0;
            rc_sel_recv_bytes_tmp = 12'd0;
            rc_sel_buf_base_tmp   = 7'd0;
            rc_sel_valid_tmp      = 1'b0;

            if (rc_s0_sof_reg) begin
                case (rc_s0_tag_reg)
                    TAG0: begin
                        rc_sel_tag_idx_tmp    = 3'd0;
                        rc_sel_req_bytes_tmp  = tag0_req_bytes_reg;
                        rc_sel_recv_bytes_tmp = tag0_recv_bytes_reg;
                        rc_sel_buf_base_tmp   = tag0_buf_base_reg;
                        rc_sel_valid_tmp      = 1'b1;
                    end
                    TAG1: begin
                        rc_sel_tag_idx_tmp    = 3'd1;
                        rc_sel_req_bytes_tmp  = tag1_req_bytes_reg;
                        rc_sel_recv_bytes_tmp = tag1_recv_bytes_reg;
                        rc_sel_buf_base_tmp   = tag1_buf_base_reg;
                        rc_sel_valid_tmp      = 1'b1;
                    end
                    TAG2: begin
                        rc_sel_tag_idx_tmp    = 3'd2;
                        rc_sel_req_bytes_tmp  = tag2_req_bytes_reg;
                        rc_sel_recv_bytes_tmp = tag2_recv_bytes_reg;
                        rc_sel_buf_base_tmp   = tag2_buf_base_reg;
                        rc_sel_valid_tmp      = 1'b1;
                    end
                    TAG3: begin
                        rc_sel_tag_idx_tmp    = 3'd3;
                        rc_sel_req_bytes_tmp  = tag3_req_bytes_reg;
                        rc_sel_recv_bytes_tmp = tag3_recv_bytes_reg;
                        rc_sel_buf_base_tmp   = tag3_buf_base_reg;
                        rc_sel_valid_tmp      = 1'b1;
                    end
                    TAG4: begin
                        rc_sel_tag_idx_tmp    = 3'd4;
                        rc_sel_req_bytes_tmp  = tag4_req_bytes_reg;
                        rc_sel_recv_bytes_tmp = tag4_recv_bytes_reg;
                        rc_sel_buf_base_tmp   = tag4_buf_base_reg;
                        rc_sel_valid_tmp      = 1'b1;
                    end
                    TAG5: begin
                        rc_sel_tag_idx_tmp    = 3'd5;
                        rc_sel_req_bytes_tmp  = tag5_req_bytes_reg;
                        rc_sel_recv_bytes_tmp = tag5_recv_bytes_reg;
                        rc_sel_buf_base_tmp   = tag5_buf_base_reg;
                        rc_sel_valid_tmp      = 1'b1;
                    end
                    TAG6: begin
                        rc_sel_tag_idx_tmp    = 3'd6;
                        rc_sel_req_bytes_tmp  = tag6_req_bytes_reg;
                        rc_sel_recv_bytes_tmp = tag6_recv_bytes_reg;
                        rc_sel_buf_base_tmp   = tag6_buf_base_reg;
                        rc_sel_valid_tmp      = 1'b1;
                    end
                    TAG7: begin
                        rc_sel_tag_idx_tmp    = 3'd7;
                        rc_sel_req_bytes_tmp  = tag7_req_bytes_reg;
                        rc_sel_recv_bytes_tmp = tag7_recv_bytes_reg;
                        rc_sel_buf_base_tmp   = tag7_buf_base_reg;
                        rc_sel_valid_tmp      = 1'b1;
                    end
                    default: begin
                        rc_sel_tag_idx_tmp    = 3'd0;
                        rc_sel_req_bytes_tmp  = 12'd0;
                        rc_sel_recv_bytes_tmp = 12'd0;
                        rc_sel_buf_base_tmp   = 7'd0;
                        rc_sel_valid_tmp      = 1'b0;
                    end
                endcase

                rc_payload_capacity_tmp = rc_s0_desc_byte_count_reg - {11'd0, rc_s0_lower_addr_reg};
                if (rc_s0_byte_count_reg > rc_payload_capacity_tmp) begin
                    rc_cpl_payload_bytes_tmp = rc_payload_capacity_tmp[11:0];
                end else begin
                    rc_cpl_payload_bytes_tmp = rc_s0_byte_count_reg[11:0];
                end

                rc_cpl_start_byte_tmp = {1'b0, rc_sel_buf_base_tmp, 5'd0} + ({1'b0, rc_sel_req_bytes_tmp} - rc_s0_byte_count_reg);
                rc_s1_tag_idx_reg        <= rc_sel_tag_idx_tmp;
                rc_s1_sel_valid_reg      <= rc_sel_valid_tmp;
                rc_s1_sel_recv_bytes_reg <= rc_sel_recv_bytes_tmp;
                rc_s1_payload_bytes_reg  <= rc_cpl_payload_bytes_tmp;
                rc_s1_word_base_reg      <= rc_cpl_start_byte_tmp[11:5];
            end else begin
                rc_s1_tag_idx_reg        <= 3'd0;
                rc_s1_sel_valid_reg      <= 1'b0;
                rc_s1_sel_recv_bytes_reg <= 12'd0;
                rc_s1_payload_bytes_reg  <= 12'd0;
                rc_s1_word_base_reg      <= 7'd0;
            end
        end

        /* RC stage 2: payload pack/write and per-tag completion update.  All
         * inputs here come from the stage-1 registers or the multi-beat
         * completion context, so descriptor dword_count is no longer on the
         * BRAM/tag-update timing path.
         */
        if (rc_s1_valid_reg) begin
            rc_pkt_first_tmp = rc_s1_pkt_first_reg;
            if (rc_pkt_first_tmp) begin
                rc_sel_tag_idx_tmp        = rc_s1_tag_idx_reg;
                rc_sel_recv_bytes_tmp     = rc_s1_sel_recv_bytes_reg;
                rc_sel_valid_tmp          = rc_s1_sel_valid_reg;
                rc_proc_word_base_tmp     = rc_s1_word_base_reg;
                rc_proc_write_word_tmp    = 7'd0;
                rc_proc_carry_dw_tmp      = 3'd0;
                rc_proc_carry_data_tmp    = 256'd0;
                rc_proc_payload_bytes_tmp = rc_s1_payload_bytes_reg;
                rc_proc_recv_bytes_tmp    = 12'd0;
                rc_proc_error_tmp         = rc_s1_error_reg;
            end else begin
                rc_sel_tag_idx_tmp        = rc_cpl_tag_idx_reg;
                rc_sel_recv_bytes_tmp     = 12'd0;
                rc_sel_valid_tmp          = 1'b1;
                rc_proc_word_base_tmp     = rc_cpl_word_base_reg;
                rc_proc_write_word_tmp    = rc_cpl_write_word_reg;
                rc_proc_carry_dw_tmp      = rc_cpl_carry_dw_reg;
                rc_proc_carry_data_tmp    = rc_cpl_carry_data_reg;
                rc_proc_payload_bytes_tmp = rc_cpl_payload_bytes_reg;
                rc_proc_recv_bytes_tmp    = rc_cpl_recv_bytes_reg;
                rc_proc_error_tmp         = rc_cpl_error_reg;
            end

            rc_next_write_word_tmp    = rc_proc_write_word_tmp;
            rc_next_carry_dw_tmp      = rc_proc_carry_dw_tmp;
            rc_next_carry_data_tmp    = rc_proc_carry_data_tmp;
            rc_next_recv_bytes_tmp    = rc_proc_recv_bytes_tmp;
            rc_full_word_written_tmp  = 1'b0;

            rc_payload_left_bytes_tmp = rc_proc_payload_bytes_tmp - rc_proc_recv_bytes_tmp;
            rc_payload_left_dw_tmp    = (rc_payload_left_bytes_tmp[11:2] + (|rc_payload_left_bytes_tmp[1:0] ? 11'd1 : 11'd0));
            rc_payload_aligned_tmp    = rc_s1_payload_reg;
            if (rc_pkt_first_tmp) begin
                rc_payload_dw_count_tmp = (rc_payload_left_dw_tmp > 11'd5) ? 4'd5 : rc_payload_left_dw_tmp[3:0];
            end else begin
                rc_payload_dw_count_tmp = (rc_payload_left_dw_tmp > 11'd8) ? 4'd8 : rc_payload_left_dw_tmp[3:0];
            end

            if (rc_payload_dw_count_tmp != 4'd0 && rc_proc_recv_bytes_tmp < rc_proc_payload_bytes_tmp) begin
                case (rc_proc_carry_dw_tmp)
                    3'd0: rc_combined_dw_tmp = {256'd0, rc_payload_aligned_tmp};
                    3'd1: rc_combined_dw_tmp = {224'd0, rc_payload_aligned_tmp, rc_proc_carry_data_tmp[31:0]};
                    3'd2: rc_combined_dw_tmp = {192'd0, rc_payload_aligned_tmp, rc_proc_carry_data_tmp[63:0]};
                    3'd3: rc_combined_dw_tmp = {160'd0, rc_payload_aligned_tmp, rc_proc_carry_data_tmp[95:0]};
                    3'd4: rc_combined_dw_tmp = {128'd0, rc_payload_aligned_tmp, rc_proc_carry_data_tmp[127:0]};
                    3'd5: rc_combined_dw_tmp = {96'd0, rc_payload_aligned_tmp, rc_proc_carry_data_tmp[159:0]};
                    3'd6: rc_combined_dw_tmp = {64'd0, rc_payload_aligned_tmp, rc_proc_carry_data_tmp[191:0]};
                    3'd7: rc_combined_dw_tmp = {32'd0, rc_payload_aligned_tmp, rc_proc_carry_data_tmp[223:0]};
                    default: rc_combined_dw_tmp = {256'd0, rc_payload_aligned_tmp};
                endcase

                rc_total_dw_tmp = {1'b0, rc_proc_carry_dw_tmp} + rc_payload_dw_count_tmp;
                rc_recv_inc_tmp = {8'd0, rc_payload_dw_count_tmp} << 2;
                if ({1'b0, rc_proc_recv_bytes_tmp} + {1'b0, rc_recv_inc_tmp} > {1'b0, rc_proc_payload_bytes_tmp}) begin
                    rc_recv_inc_tmp = rc_proc_payload_bytes_tmp - rc_proc_recv_bytes_tmp;
                end

                if (rc_total_dw_tmp >= 4'd8) begin
                    word_index_tmp = rc_proc_word_base_tmp + rc_proc_write_word_tmp;
                    rd_buf_wr_en_reg   <= 1'b1;
                    rd_buf_wr_addr_reg <= word_index_tmp;
                    rd_buf_wr_data_reg <= rc_combined_dw_tmp[255:0];
                    rc_full_word_written_tmp = 1'b1;
                    rc_next_write_word_tmp   = rc_proc_write_word_tmp + 7'd1;
                    rc_next_carry_data_tmp   = rc_combined_dw_tmp[511:256];
                    rc_next_carry_dw_tmp     = rc_total_dw_tmp[2:0];
                end else begin
                    rc_next_carry_data_tmp = rc_combined_dw_tmp[255:0];
                    rc_next_carry_dw_tmp   = rc_total_dw_tmp[2:0];
                end

                rc_next_recv_bytes_tmp = rc_proc_recv_bytes_tmp + rc_recv_inc_tmp;
            end

            if (!rc_s1_tlast_reg) begin
                rc_cpl_active_reg <= 1'b1;
                if (rc_pkt_first_tmp) begin
                    rc_cpl_tag_idx_reg <= rc_s1_tag_idx_reg;
                    rc_cpl_error_reg   <= rc_s1_error_reg;
                end
                rc_cpl_word_base_reg     <= rc_proc_word_base_tmp;
                rc_cpl_write_word_reg    <= rc_next_write_word_tmp;
                rc_cpl_carry_dw_reg      <= rc_next_carry_dw_tmp;
                rc_cpl_carry_data_reg    <= rc_next_carry_data_tmp;
                rc_cpl_payload_bytes_reg <= rc_proc_payload_bytes_tmp;
                rc_cpl_recv_bytes_reg    <= rc_next_recv_bytes_tmp;
            end else begin
                if (rc_next_carry_dw_tmp != 3'd0) begin
                    word_index_tmp = rc_proc_word_base_tmp + rc_next_write_word_tmp;
                    if (rc_full_word_written_tmp) begin
                        rd_buf_tail_wr_pending_reg <= 1'b1;
                        rd_buf_tail_wr_addr_reg    <= word_index_tmp;
                        rd_buf_tail_wr_data_reg    <= rc_next_carry_data_tmp;
                    end else begin
                        rd_buf_wr_en_reg   <= 1'b1;
                        rd_buf_wr_addr_reg <= word_index_tmp;
                        rd_buf_wr_data_reg <= rc_next_carry_data_tmp;
                    end
                end
                rc_cpl_active_reg     <= 1'b0;
                rc_cpl_write_word_reg <= 7'd0;
                rc_cpl_carry_dw_reg   <= 3'd0;
                rc_cpl_recv_bytes_reg <= 12'd0;

                if (rc_pkt_first_tmp) begin
                    rc_tag_recv_total_tmp = {1'b0, rc_s1_sel_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                end else begin
                    case (rc_cpl_tag_idx_reg)
                        3'd0: rc_tag_recv_total_tmp = {1'b0, tag0_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                        3'd1: rc_tag_recv_total_tmp = {1'b0, tag1_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                        3'd2: rc_tag_recv_total_tmp = {1'b0, tag2_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                        3'd3: rc_tag_recv_total_tmp = {1'b0, tag3_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                        3'd4: rc_tag_recv_total_tmp = {1'b0, tag4_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                        3'd5: rc_tag_recv_total_tmp = {1'b0, tag5_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                        3'd6: rc_tag_recv_total_tmp = {1'b0, tag6_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                        3'd7: rc_tag_recv_total_tmp = {1'b0, tag7_recv_bytes_reg} + {1'b0, rc_proc_payload_bytes_tmp};
                        default: rc_tag_recv_total_tmp = {1'b0, rc_proc_payload_bytes_tmp};
                    endcase
                end

                case (rc_pkt_first_tmp ? rc_s1_tag_idx_reg : rc_cpl_tag_idx_reg)
                    3'd0: begin
                        tag0_recv_bytes_reg <= rc_tag_recv_total_tmp[11:0];
                        if (rc_proc_error_tmp != 4'd0 || rc_s1_discontinue_reg) begin
                            tag_error_reg[0]    <= 1'b1;
                            rd_done_mask_reg[0] <= 1'b1;
                        end else if (rc_tag_recv_total_tmp >= {1'b0, tag0_req_bytes_reg}) begin
                            rd_done_mask_reg[0] <= 1'b1;
                        end
                    end
                    3'd1: begin
                        tag1_recv_bytes_reg <= rc_tag_recv_total_tmp[11:0];
                        if (rc_proc_error_tmp != 4'd0 || rc_s1_discontinue_reg) begin
                            tag_error_reg[1]    <= 1'b1;
                            rd_done_mask_reg[1] <= 1'b1;
                        end else if (rc_tag_recv_total_tmp >= {1'b0, tag1_req_bytes_reg}) begin
                            rd_done_mask_reg[1] <= 1'b1;
                        end
                    end
                    3'd2: begin
                        tag2_recv_bytes_reg <= rc_tag_recv_total_tmp[11:0];
                        if (rc_proc_error_tmp != 4'd0 || rc_s1_discontinue_reg) begin
                            tag_error_reg[2]    <= 1'b1;
                            rd_done_mask_reg[2] <= 1'b1;
                        end else if (rc_tag_recv_total_tmp >= {1'b0, tag2_req_bytes_reg}) begin
                            rd_done_mask_reg[2] <= 1'b1;
                        end
                    end
                    3'd3: begin
                        tag3_recv_bytes_reg <= rc_tag_recv_total_tmp[11:0];
                        if (rc_proc_error_tmp != 4'd0 || rc_s1_discontinue_reg) begin
                            tag_error_reg[3]    <= 1'b1;
                            rd_done_mask_reg[3] <= 1'b1;
                        end else if (rc_tag_recv_total_tmp >= {1'b0, tag3_req_bytes_reg}) begin
                            rd_done_mask_reg[3] <= 1'b1;
                        end
                    end
                    3'd4: begin
                        tag4_recv_bytes_reg <= rc_tag_recv_total_tmp[11:0];
                        if (rc_proc_error_tmp != 4'd0 || rc_s1_discontinue_reg) begin
                            tag_error_reg[4]    <= 1'b1;
                            rd_done_mask_reg[4] <= 1'b1;
                        end else if (rc_tag_recv_total_tmp >= {1'b0, tag4_req_bytes_reg}) begin
                            rd_done_mask_reg[4] <= 1'b1;
                        end
                    end
                    3'd5: begin
                        tag5_recv_bytes_reg <= rc_tag_recv_total_tmp[11:0];
                        if (rc_proc_error_tmp != 4'd0 || rc_s1_discontinue_reg) begin
                            tag_error_reg[5]    <= 1'b1;
                            rd_done_mask_reg[5] <= 1'b1;
                        end else if (rc_tag_recv_total_tmp >= {1'b0, tag5_req_bytes_reg}) begin
                            rd_done_mask_reg[5] <= 1'b1;
                        end
                    end
                    3'd6: begin
                        tag6_recv_bytes_reg <= rc_tag_recv_total_tmp[11:0];
                        if (rc_proc_error_tmp != 4'd0 || rc_s1_discontinue_reg) begin
                            tag_error_reg[6]    <= 1'b1;
                            rd_done_mask_reg[6] <= 1'b1;
                        end else if (rc_tag_recv_total_tmp >= {1'b0, tag6_req_bytes_reg}) begin
                            rd_done_mask_reg[6] <= 1'b1;
                        end
                    end
                    3'd7: begin
                        tag7_recv_bytes_reg <= rc_tag_recv_total_tmp[11:0];
                        if (rc_proc_error_tmp != 4'd0 || rc_s1_discontinue_reg) begin
                            tag_error_reg[7]    <= 1'b1;
                            rd_done_mask_reg[7] <= 1'b1;
                        end else if (rc_tag_recv_total_tmp >= {1'b0, tag7_req_bytes_reg}) begin
                            rd_done_mask_reg[7] <= 1'b1;
                        end
                    end
                    default: begin
                        tag_error_reg[0]    <= 1'b1;
                        rd_done_mask_reg[0] <= 1'b1;
                    end
                endcase

                if ((rc_proc_error_tmp != 4'd0) || rc_s1_discontinue_reg || (rc_pkt_first_tmp && !rc_s1_sel_valid_reg)) begin
                    op_resp_reg          <= AXI_RESP_SLVERR;
                    status_error_cor_reg <= 1'b1;
                end
            end
        end

        if (s_axi_wvalid && s_axi_wready && state_reg == S_WR_RQ) begin
            wr_next_data_reg  <= s_axi_wdata;
            wr_next_strb_reg  <= s_axi_wstrb;
            wr_next_last_reg  <= s_axi_wlast;
            wr_next_valid_reg <= 1'b1;
        end

        case (state_reg)
            S_IDLE: begin
                lookup_idx_reg    <= 2'd0;
                wr_data_valid_reg <= 1'b0;
                wr_half_reg       <= 1'b0;

                if (enable && s_axi_awvalid && s_axi_awready) begin
                    op_id_reg          <= s_axi_awid;
                    b_id_reg           <= s_axi_awid;
                    op_axi_addr_reg    <= {s_axi_awaddr[AXI_ADDR_WIDTH-1:5], 5'd0};
                    op_beats_left_reg  <= {8'd0, s_axi_awlen} + 16'd1;
                    op_total_beats_reg <= {8'd0, s_axi_awlen} + 16'd1;
                    wr_beat_count_reg  <= 16'd0;
                    op_resp_reg        <= ((s_axi_awburst == AXI_BURST_INCR) &&
                        (((s_axi_awsize == AXI_SIZE_VALUE) && (s_axi_awaddr[4:0] == 5'd0)) ||
                         ((s_axi_awsize == 3'd2) && (s_axi_awlen == 8'd0) && (s_axi_awaddr[1:0] == 2'd0)))) ? AXI_RESP_OKAY : AXI_RESP_SLVERR;
                    lookup_is_read_reg <= 1'b0;
                    state_reg          <= S_WR_LOOKUP;
                end else if (enable && s_axi_arvalid && s_axi_arready) begin
                    op_id_reg                    <= s_axi_arid;
                    r_id_reg                     <= s_axi_arid;
                    op_axi_addr_reg              <= s_axi_araddr;
                    op_beats_left_reg            <= {8'd0, s_axi_arlen} + 16'd1;
                    op_total_beats_reg           <= {8'd0, s_axi_arlen} + 16'd1;
                    op_return_beats_left_reg     <= {8'd0, s_axi_arlen} + 16'd1;
                    rd_global_returned_beats_reg <= 16'd0;
                    case (max_read_request_size)
                        3'd0: rd_req_bytes_reg <= 12'd128;
                        3'd1: rd_req_bytes_reg <= 12'd256;
                        default: rd_req_bytes_reg <= READ_REQ_BYTES_VALUE;
                    endcase
                    op_resp_reg        <= ((s_axi_arburst == AXI_BURST_INCR) && (s_axi_arsize == AXI_SIZE_VALUE) && (s_axi_araddr[4:0] == 5'd0) && cfg_ext_tag_ok) ? AXI_RESP_OKAY : AXI_RESP_SLVERR;
                    lookup_is_read_reg <= 1'b1;
                    state_reg          <= S_RD_LOOKUP;
                end
            end

            S_WR_LOOKUP, S_RD_LOOKUP: begin
                tbl_axi_base_tmp  = table_axi_base0;
                tbl_pcie_base_tmp = table_pcie_base0;
                tbl_mask_tmp      = table_mask0;
                lookup_hit_tmp    = 1'b0;
                case (lookup_idx_reg)
                    2'd0: begin
                        tbl_axi_base_tmp  = table_axi_base0;
                        tbl_pcie_base_tmp = table_pcie_base0;
                        tbl_mask_tmp      = table_mask0;
                        lookup_hit_tmp    = table_enable[0] && ((op_axi_addr_reg[PCIE_ADDR_WIDTH-1:0] & table_mask0) == (table_axi_base0 & table_mask0));
                    end
                    2'd1: begin
                        tbl_axi_base_tmp  = table_axi_base1;
                        tbl_pcie_base_tmp = table_pcie_base1;
                        tbl_mask_tmp      = table_mask1;
                        lookup_hit_tmp    = table_enable[1] && ((op_axi_addr_reg[PCIE_ADDR_WIDTH-1:0] & table_mask1) == (table_axi_base1 & table_mask1));
                    end
                    2'd2: begin
                        tbl_axi_base_tmp  = table_axi_base2;
                        tbl_pcie_base_tmp = table_pcie_base2;
                        tbl_mask_tmp      = table_mask2;
                        lookup_hit_tmp    = table_enable[2] && ((op_axi_addr_reg[PCIE_ADDR_WIDTH-1:0] & table_mask2) == (table_axi_base2 & table_mask2));
                    end
                    default: begin
                        tbl_axi_base_tmp  = table_axi_base3;
                        tbl_pcie_base_tmp = table_pcie_base3;
                        tbl_mask_tmp      = table_mask3;
                        lookup_hit_tmp    = table_enable[3] && ((op_axi_addr_reg[PCIE_ADDR_WIDTH-1:0] & table_mask3) == (table_axi_base3 & table_mask3));
                    end
                endcase

                // Register masked match before deciding the state. Preserve
                // table priority, arbitrary masks and full-width arithmetic.
                lookup_hit_reg <= lookup_hit_tmp;
                lookup_axi_base_reg <= tbl_axi_base_tmp;
                lookup_pcie_base_reg <= tbl_pcie_base_tmp;
                state_reg <= S_LOOKUP_RESULT;
            end
            S_LOOKUP_RESULT: begin
                if (lookup_hit_reg) begin
                    trans_axi_base_reg <= lookup_axi_base_reg;
                    trans_pcie_base_reg <= lookup_pcie_base_reg;
                    state_reg <= S_LOOKUP_OFFSET;
                end else if (lookup_idx_reg == 2'd3) begin
                    op_resp_reg <= AXI_RESP_SLVERR;
                    status_error_cor_reg <= 1'b1;
                    state_reg <= lookup_is_read_reg ? S_RD_RESP_ERR : S_WR_DROP;
                end else begin
                    lookup_idx_reg <= lookup_idx_reg + 2'd1;
                    state_reg <= lookup_is_read_reg ? S_RD_LOOKUP : S_WR_LOOKUP;
                end
            end
            S_LOOKUP_OFFSET: begin
                lookup_offset_reg <= op_axi_addr_reg[PCIE_ADDR_WIDTH-1:0] - trans_axi_base_reg;
                state_reg <= S_LOOKUP_TRANSLATE;
            end
            S_LOOKUP_TRANSLATE: begin
                op_pcie_addr_reg <= trans_pcie_base_reg + lookup_offset_reg;
                if (lookup_is_read_reg) begin
                    if (op_resp_reg == AXI_RESP_OKAY) state_reg <= S_RD_NEXT;
                    else begin
                        status_error_cor_reg <= 1'b1;
                        state_reg <= S_RD_RESP_ERR;
                    end
                end else state_reg <= op_resp_reg == AXI_RESP_OKAY ? S_WR_W : S_WR_DROP;
            end

            S_WR_DROP: begin
                if (s_axi_wvalid && s_axi_wready) begin
                    if (op_beats_left_reg != 0) begin
                        op_beats_left_reg <= op_beats_left_reg - 16'd1;
                    end
                    if (s_axi_wlast || op_beats_left_reg <= 16'd1) begin
                        b_resp_reg  <= AXI_RESP_SLVERR;
                        b_valid_reg <= 1'b1;
                        state_reg   <= S_WR_RESP;
                    end
                end
            end

            S_WR_W: begin
                if (wr_next_valid_reg) begin
                    wr_data_reg       <= wr_next_data_reg;
                    wr_strb_reg       <= wr_next_strb_reg;
                    wr_sparse_reg     <= (wr_next_strb_reg != 32'hffffffff);
                    wr_dw_reg         <= 3'd0;
                    wr_data_valid_reg <= 1'b1;
                    wr_half_reg       <= 1'b0;
                    wr_last_reg       <= wr_next_last_reg || (op_beats_left_reg <= 16'd1);
                    wr_next_valid_reg <= 1'b0;
                    state_reg         <= S_WR_RQ;
                end else if (s_axi_wvalid && s_axi_wready) begin
                    wr_data_reg       <= s_axi_wdata;
                    wr_strb_reg       <= s_axi_wstrb;
                    wr_sparse_reg     <= (s_axi_wstrb != 32'hffffffff);
                    wr_dw_reg         <= 3'd0;
                    wr_data_valid_reg <= 1'b1;
                    wr_half_reg       <= 1'b0;
                    wr_last_reg       <= s_axi_wlast || (op_beats_left_reg <= 16'd1);
                    state_reg         <= S_WR_RQ;
                end
            end

            S_WR_RQ: begin
                if (wr_data_valid_reg && rq_available) begin
                    rq_hdr_tmp         = 128'd0;
                    rq_hdr_tmp[1:0]    = 2'b00;
                    rq_hdr_tmp[74:64]  = 11'd4;
                    rq_hdr_tmp[78:75]  = REQ_MEM_WRITE;
                    rq_hdr_tmp[95:80]  = requester_id;
                    rq_hdr_tmp[103:96] = 8'd0;
                    rq_hdr_tmp[120]    = requester_id_enable;

                    // op_pcie_addr_reg is the current aligned write beat address.
                    wr_addr_tmp = op_pcie_addr_reg;
                    if (wr_sparse_reg) begin
                        /* CQEs/doorbells contain only four valid bytes. A
                         * 16-byte MWr cannot encode holes in interior DWs.
                         * Shift by a constant, emit one enabled DW at a time;
                         * no priority encoder or variable 256-bit mux in RQ.
                         * Full beats use one 32B TLP in video mode; sparse writes stay DW-sized. */
                        wr_addr_tmp = {op_pcie_addr_reg[PCIE_ADDR_WIDTH-1:5], wr_dw_reg, 2'b00};
                        rq_hdr_tmp[63:2] = wr_addr_tmp[PCIE_ADDR_WIDTH-1:2];
                        rq_hdr_tmp[74:64] = 11'd1;
                        if (wr_strb_reg[3:0] != 4'd0) begin
                            rq_tdata_reg <= {96'd0, wr_data_reg[31:0], rq_hdr_tmp};
                            rq_tkeep_reg <= 8'h1f;
                            rq_tlast_reg <= 1'b1;
                            rq_tuser_reg <= {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                            rq_tuser_reg[3:0] <= wr_strb_reg[3:0];
                            // A one-DW TLP requires Last BE = 0.
                            rq_tvalid_reg <= 1'b1;
                        end
                        wr_data_reg <= {32'd0, wr_data_reg[255:32]};
                        wr_strb_reg <= {4'd0, wr_strb_reg[31:4]};
                        wr_dw_reg <= wr_dw_reg + 3'd1;
                        if (wr_dw_reg == 3'd7 || wr_strb_reg[31:4] == 28'd0) begin
                            wr_data_valid_reg <= 1'b0;
                            wr_beat_count_reg <= wr_beat_count_reg + 16'd1;
                            op_pcie_addr_reg <= {op_pcie_addr_reg[PCIE_ADDR_WIDTH-1:5] + {{(PCIE_ADDR_WIDTH-6){1'b0}}, 1'b1}, 5'd0};
                            op_beats_left_reg <= op_beats_left_reg - 16'd1;
                            if (wr_last_reg) begin
                                b_resp_reg <= op_resp_reg;
                                b_valid_reg <= 1'b1;
                                state_reg <= S_WR_RESP;
                            end else if (wr_next_valid_reg) begin
                                wr_data_reg       <= wr_next_data_reg;
                                wr_strb_reg       <= wr_next_strb_reg;
                                wr_sparse_reg     <= (wr_next_strb_reg != 32'hffffffff);
                                wr_dw_reg         <= 3'd0;
                                wr_data_valid_reg <= 1'b1;
                                wr_last_reg       <= wr_next_last_reg || (op_beats_left_reg <= 16'd2);
                                wr_next_valid_reg <= 1'b0;
                                state_reg         <= S_WR_RQ;
                            end else begin
                                state_reg <= S_WR_W;
                            end
                        end
                    end else if (!wr_half_reg) begin
                        rq_hdr_tmp[63:2] = wr_addr_tmp[PCIE_ADDR_WIDTH-1:2];
                        if (WRITE_TLP_BYTES == 32) rq_hdr_tmp[74:64] = 11'd8;
                        rq_tdata_reg      <= {wr_data_reg[127:0], rq_hdr_tmp};
                        rq_tkeep_reg      <= 8'hff;
                        rq_tlast_reg      <= (WRITE_TLP_BYTES != 32);
                        rq_tuser_reg      <= {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                        rq_tuser_reg[3:0] <= 4'hf;
                        rq_tuser_reg[7:4] <= 4'hf;
                        rq_tvalid_reg     <= 1'b1;
                        wr_half_reg       <= 1'b1;
                    end else begin
                        wr_addr_tmp = {op_pcie_addr_reg[PCIE_ADDR_WIDTH-1:5], 1'b1, 4'd0};
                        rq_hdr_tmp[63:2] = wr_addr_tmp[PCIE_ADDR_WIDTH-1:2];
                        rq_tdata_reg      <= (WRITE_TLP_BYTES == 32) ?
                                             {128'd0, wr_data_reg[255:128]} :
                                             {wr_data_reg[255:128], rq_hdr_tmp};
                        rq_tkeep_reg      <= (WRITE_TLP_BYTES == 32) ? 8'h0f : 8'hff;
                        rq_tlast_reg      <= 1'b1;
                        rq_tuser_reg      <= {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                        rq_tuser_reg[3:0] <= 4'hf;
                        rq_tuser_reg[7:4] <= 4'hf;
                        rq_tvalid_reg     <= 1'b1;

                        wr_half_reg       <= 1'b0;
                        wr_data_valid_reg <= 1'b0;
                        wr_beat_count_reg <= wr_beat_count_reg + 16'd1;
                        op_pcie_addr_reg <= {op_pcie_addr_reg[PCIE_ADDR_WIDTH-1:5] + {{(PCIE_ADDR_WIDTH-6){1'b0}}, 1'b1}, 5'd0};
                        if (op_beats_left_reg != 0) begin
                            op_beats_left_reg <= op_beats_left_reg - 16'd1;
                        end
                        if (wr_last_reg) begin
                            b_resp_reg  <= op_resp_reg;
                            b_valid_reg <= 1'b1;
                            state_reg   <= S_WR_RESP;
                        end else if (wr_next_valid_reg) begin
                            wr_data_reg       <= wr_next_data_reg;
                            wr_strb_reg       <= wr_next_strb_reg;
                            wr_sparse_reg     <= (wr_next_strb_reg != 32'hffffffff);
                            wr_dw_reg         <= 3'd0;
                            wr_data_valid_reg <= 1'b1;
                            wr_last_reg       <= wr_next_last_reg || (op_beats_left_reg <= 16'd2);
                            wr_next_valid_reg <= 1'b0;
                            state_reg         <= S_WR_RQ;
                        end else begin
                            state_reg <= S_WR_W;
                        end
                    end
                end
            end

            S_WR_RESP: begin
                if (!b_valid_reg) begin
                    state_reg <= S_IDLE;
                end
            end

            S_RD_NEXT: begin
                if (!rq_tvalid_reg && !r_valid_reg) begin
                    rd_bytes_left_tmp = op_return_beats_left_reg << 5;
                    page_left_tmp     = 16'd4096 - {4'b0000, op_pcie_addr_reg[11:0]};
                    calc16_tmp        = rd_bytes_left_tmp;
                    if (calc16_tmp > page_left_tmp) begin
                        calc16_tmp = page_left_tmp;
                    end
                    win_bytes_tmp = calc16_tmp[12:0];
                    win_beats_tmp = win_bytes_tmp[12:5] + (|win_bytes_tmp[4:0] ? 8'd1 : 8'd0);
                    rd_window_bytes_reg        <= win_bytes_tmp;
                    rd_window_beats_reg        <= win_beats_tmp;
                    rd_window_issued_bytes_reg <= 13'd0;
                    rd_return_idx_reg          <= 7'd0;
                    state_reg                  <= S_RD_WAVE;
                end
            end

            S_RD_WAVE: begin
                if (!rq_tvalid_reg && !r_valid_reg) begin
                    rd_wave_left_tmp = rd_window_bytes_reg - rd_window_issued_bytes_reg;
                    rd_wave_max_tmp  = {1'b0, rd_req_bytes_reg} << 3;
                    if (rd_wave_left_tmp > rd_wave_max_tmp) begin
                        rd_wave_next_tmp = rd_wave_max_tmp;
                    end else begin
                        rd_wave_next_tmp = rd_wave_left_tmp;
                    end
                    rd_wave_bytes_reg     <= rd_wave_next_tmp;
                    rd_wave_base_beat_reg <= rd_window_issued_bytes_reg[11:5];
                    case (rd_req_bytes_reg)
                        12'd128: begin
                            tag_count_adj_tmp = rd_wave_next_tmp + 13'd127;
                            rd_issue_count_reg <= tag_count_adj_tmp[10:7];
                        end
                        12'd256: begin
                            tag_count_adj_tmp = rd_wave_next_tmp + 13'd255;
                            rd_issue_count_reg <= tag_count_adj_tmp[11:8];
                        end
                        default: begin
                            tag_count_adj_tmp = rd_wave_next_tmp + 13'd511;
                            rd_issue_count_reg <= tag_count_adj_tmp[12:9];
                        end
                    endcase
                    rd_issue_idx_reg      <= 3'd0;
                    rd_issue_mask_reg     <= 8'd0;
                    rd_done_mask_reg      <= 8'd0;
                    tag_error_reg         <= 8'd0;
                    rc_cpl_active_reg     <= 1'b0;
                    rc_cpl_write_word_reg <= 7'd0;
                    rc_cpl_carry_dw_reg   <= 3'd0;
                    rc_cpl_recv_bytes_reg <= 12'd0;
                    tag0_recv_bytes_reg   <= 12'd0;
                    tag0_req_bytes_reg    <= 12'd0;
                    tag0_buf_base_reg     <= 7'd0;
                    tag1_recv_bytes_reg   <= 12'd0;
                    tag1_req_bytes_reg    <= 12'd0;
                    tag1_buf_base_reg     <= 7'd0;
                    tag2_recv_bytes_reg   <= 12'd0;
                    tag2_req_bytes_reg    <= 12'd0;
                    tag2_buf_base_reg     <= 7'd0;
                    tag3_recv_bytes_reg   <= 12'd0;
                    tag3_req_bytes_reg    <= 12'd0;
                    tag3_buf_base_reg     <= 7'd0;
                    tag4_recv_bytes_reg   <= 12'd0;
                    tag4_req_bytes_reg    <= 12'd0;
                    tag4_buf_base_reg     <= 7'd0;
                    tag5_recv_bytes_reg   <= 12'd0;
                    tag5_req_bytes_reg    <= 12'd0;
                    tag5_buf_base_reg     <= 7'd0;
                    tag6_recv_bytes_reg   <= 12'd0;
                    tag6_req_bytes_reg    <= 12'd0;
                    tag6_buf_base_reg     <= 7'd0;
                    tag7_recv_bytes_reg   <= 12'd0;
                    tag7_req_bytes_reg    <= 12'd0;
                    tag7_buf_base_reg     <= 7'd0;
                    state_reg             <= S_RD_ISSUE;
                end
            end

            S_RD_ISSUE: begin
                if (rq_available) begin
                    case (rd_req_bytes_reg)
                        12'd128: rd_issue_offset_tmp = {3'd0, rd_issue_idx_reg, 7'd0};
                        12'd256: rd_issue_offset_tmp = {2'd0, rd_issue_idx_reg, 8'd0};
                        default: rd_issue_offset_tmp = {1'd0, rd_issue_idx_reg, 9'd0};
                    endcase

                    rd_issue_addr_tmp = op_pcie_addr_reg + {{(PCIE_ADDR_WIDTH-13){1'b0}}, rd_window_issued_bytes_reg} + {{(PCIE_ADDR_WIDTH-13){1'b0}}, rd_issue_offset_tmp};
                    rd_issue_left_tmp = rd_wave_bytes_reg - rd_issue_offset_tmp;
                    calc16_tmp        = {3'd0, rd_issue_left_tmp};
                    /* S_RD_NEXT already clips rd_window_bytes_reg at the 4 KB
                     * boundary, and every wave/segment is a subset of that
                     * window.  Recomputing page_left from the issued address
                     * here only duplicates the boundary check on the one-MRd-
                     * per-cycle path. */
                    if (calc16_tmp > {4'd0, rd_req_bytes_reg}) begin
                        calc16_tmp = {4'd0, rd_req_bytes_reg};
                    end
                    seg_bytes_tmp     = calc16_tmp[11:0];

                    rq_hdr_tmp        = 128'd0;
                    rq_hdr_tmp[1:0]   = 2'b00;
                    rq_hdr_tmp[63:2]  = rd_issue_addr_tmp[PCIE_ADDR_WIDTH-1:2];
                    rq_hdr_tmp[74:64] = {1'b0, seg_bytes_tmp[11:2]};
                    rq_hdr_tmp[78:75] = REQ_MEM_READ;
                    rq_hdr_tmp[95:80] = requester_id;
                    case (rd_issue_idx_reg)
                        3'd0: rq_hdr_tmp[103:96] = TAG0;
                        3'd1: rq_hdr_tmp[103:96] = TAG1;
                        3'd2: rq_hdr_tmp[103:96] = TAG2;
                        3'd3: rq_hdr_tmp[103:96] = TAG3;
                        3'd4: rq_hdr_tmp[103:96] = TAG4;
                        3'd5: rq_hdr_tmp[103:96] = TAG5;
                        3'd6: rq_hdr_tmp[103:96] = TAG6;
                        default: rq_hdr_tmp[103:96] = TAG7;
                    endcase
                    rq_hdr_tmp[120] = requester_id_enable;

                    rq_tdata_reg      <= {128'd0, rq_hdr_tmp};
                    rq_tkeep_reg      <= 8'h0f;
                    rq_tlast_reg      <= 1'b1;
                    rq_tuser_reg      <= {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                    rq_tuser_reg[3:0] <= 4'hf;
                    rq_tuser_reg[7:4] <= (seg_bytes_tmp == 12'd4) ? 4'h0 : 4'hf;
                    rq_tvalid_reg     <= 1'b1;

                    case (rd_issue_idx_reg)
                        3'd0: begin
                            tag0_req_bytes_reg   <= seg_bytes_tmp;
                            tag0_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[0] <= 1'b1;
                        end
                        3'd1: begin
                            tag1_req_bytes_reg   <= seg_bytes_tmp;
                            tag1_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[1] <= 1'b1;
                        end
                        3'd2: begin
                            tag2_req_bytes_reg   <= seg_bytes_tmp;
                            tag2_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[2] <= 1'b1;
                        end
                        3'd3: begin
                            tag3_req_bytes_reg   <= seg_bytes_tmp;
                            tag3_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[3] <= 1'b1;
                        end
                        3'd4: begin
                            tag4_req_bytes_reg   <= seg_bytes_tmp;
                            tag4_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[4] <= 1'b1;
                        end
                        3'd5: begin
                            tag5_req_bytes_reg   <= seg_bytes_tmp;
                            tag5_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[5] <= 1'b1;
                        end
                        3'd6: begin
                            tag6_req_bytes_reg   <= seg_bytes_tmp;
                            tag6_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[6] <= 1'b1;
                        end
                        3'd7: begin
                            tag7_req_bytes_reg   <= seg_bytes_tmp;
                            tag7_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[7] <= 1'b1;
                        end
                        default: begin
                            tag7_req_bytes_reg   <= seg_bytes_tmp;
                            tag7_buf_base_reg    <= rd_wave_base_beat_reg + rd_issue_offset_tmp[11:5];
                            rd_issue_mask_reg[7] <= 1'b1;
                        end
                    endcase

                    if ({1'b0, rd_issue_idx_reg} + 4'd1 >= rd_issue_count_reg) begin
                        state_reg <= S_RD_WAIT;
                    end else begin
                        rd_issue_idx_reg <= rd_issue_idx_reg + 3'd1;
                    end
                end
            end

            S_RD_WAIT: begin
                if ((rd_done_mask_reg & rd_issue_mask_reg) == rd_issue_mask_reg && !r_valid_reg && !rc_s0_valid_reg && !rc_s1_valid_reg && !rc_cpl_active_reg && !rd_buf_wr_en_reg && !rd_buf_tail_wr_pending_reg) begin
                    rd_wave_done_tmp = rd_window_issued_bytes_reg + rd_wave_bytes_reg;
                    if (rd_wave_done_tmp < rd_window_bytes_reg) begin
                        rd_window_issued_bytes_reg <= rd_wave_done_tmp;
                        state_reg                  <= S_RD_WAVE;
                    end else begin
                        rd_return_idx_reg <= 7'd0;
                        state_reg         <= S_RD_SEND;
                    end
                end
            end

            S_RD_SEND: begin
                if (!r_valid_reg) begin
                    r_zero_reg  <= 1'b0;
                    r_resp_reg  <= op_resp_reg;
                    r_last_reg  <= (op_return_beats_left_reg == 16'd1);
                    r_valid_reg <= 1'b1;
                end else if (r_fire) begin
                    rd_return_idx_reg            <= rd_return_idx_reg + 7'd1;
                    rd_global_returned_beats_reg <= rd_global_returned_beats_reg + 16'd1;
                    op_return_beats_left_reg     <= op_return_beats_left_reg - 16'd1;
                    if (op_return_beats_left_reg == 16'd1) begin
                        state_reg <= S_IDLE;
                    end else if ({1'b0, rd_return_idx_reg} + 8'd1 >= rd_window_beats_reg) begin
                        op_pcie_addr_reg <= op_pcie_addr_reg + {{(PCIE_ADDR_WIDTH-13){1'b0}}, rd_window_beats_reg, 5'd0};
                        state_reg        <= S_RD_NEXT;
                    end else begin
                        /* The separate RAM port reads the following AXI beat
                         * on this edge.  Reassert VALID over the generic r_fire
                         * clear, keeping READY-high traffic at one beat/cycle. */
                        r_zero_reg  <= 1'b0;
                        r_resp_reg  <= op_resp_reg;
                        r_last_reg  <= (op_return_beats_left_reg == 16'd2);
                        r_valid_reg <= 1'b1;
                    end
                end
            end

            S_RD_RESP_ERR: begin
                if (!r_valid_reg) begin
                    r_zero_reg  <= 1'b1;
                    r_resp_reg  <= AXI_RESP_SLVERR;
                    r_last_reg  <= (op_return_beats_left_reg == 16'd1);
                    r_valid_reg <= 1'b1;
                end else if (r_fire) begin
                    op_return_beats_left_reg <= op_return_beats_left_reg - 16'd1;
                    if (op_return_beats_left_reg == 16'd1) begin
                        state_reg <= S_IDLE;
                    end
                end
            end

            default: begin
                state_reg <= S_IDLE;
            end
        endcase

        if (rst) begin
            r_zero_reg                <= 1'b0;
            state_reg                  <= S_IDLE;
            rq_tvalid_reg              <= 1'b0;
            r_valid_reg                <= 1'b0;
            b_valid_reg                <= 1'b0;
            wr_data_valid_reg          <= 1'b0;
            wr_next_valid_reg          <= 1'b0;
            status_error_cor_reg       <= 1'b0;
            status_error_uncor_reg     <= 1'b0;
            rc_s0_valid_reg            <= 1'b0;
            rc_s1_valid_reg            <= 1'b0;
            rc_cpl_active_reg          <= 1'b0;
            rc_cpl_write_word_reg      <= 7'd0;
            rc_cpl_carry_dw_reg        <= 3'd0;
            rc_cpl_recv_bytes_reg      <= 12'd0;
            rd_buf_wr_en_reg           <= 1'b0;
            rd_buf_wr_addr_reg         <= 7'd0;
            rd_buf_wr_data_reg         <= {AXI_DATA_WIDTH{1'b0}};
            rd_buf_tail_wr_pending_reg <= 1'b0;
            rd_buf_tail_wr_addr_reg    <= 7'd0;
            rd_buf_tail_wr_data_reg    <= {AXI_DATA_WIDTH{1'b0}};
            rd_done_mask_reg           <= 8'd0;
            rd_issue_mask_reg          <= 8'd0;
            tag_error_reg              <= 8'd0;
        end
    end

endmodule



`resetall
