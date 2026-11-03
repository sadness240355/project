/* SPDX-License-Identifier: MIT
 * RC stream demultiplexer by PCIe completion tag.
 *
 * Output 0: normal payload DMA completions.
 * Output 1: host-memory micro-engine completions, selected by HMEM_TAG.
 * Output 2: ST_H2D/TX completions, selected by ST_TX_TAG.
 * Output 3: AXI-MM to PCIe bridge completions, selected by AXI_BRIDGE_TAG.
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module edev_axis_rc_tag_demux #(
    parameter AXIS_PCIE_DATA_WIDTH = 256,
    parameter AXIS_PCIE_KEEP_WIDTH = AXIS_PCIE_DATA_WIDTH/32,
    parameter AXIS_PCIE_RC_USER_WIDTH = 75,
    parameter HMEM_TAG = 8'hf0,
    parameter HMEM_TAG_MASK = 8'hf0,
    parameter ST_TX_TAG = 8'h80,
    parameter ST_TX_TAG_MASK = 8'hf0,
    parameter AXI_BRIDGE_TAG = 8'h90,
    parameter AXI_BRIDGE_TAG_MASK = 8'hf8
)(
    input  wire                               clk,
    input  wire                               rst,

    input  wire [   AXIS_PCIE_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire [   AXIS_PCIE_KEEP_WIDTH-1:0] s_axis_tkeep,
    input  wire                               s_axis_tvalid,
    output wire                               s_axis_tready,
    input  wire                               s_axis_tlast,
    input  wire [AXIS_PCIE_RC_USER_WIDTH-1:0] s_axis_tuser,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_dma_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_dma_tkeep,
    output wire                               m_axis_dma_tvalid,
    input  wire                               m_axis_dma_tready,
    output wire                               m_axis_dma_tlast,
    output wire [AXIS_PCIE_RC_USER_WIDTH-1:0] m_axis_dma_tuser,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_hmem_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_hmem_tkeep,
    output wire                               m_axis_hmem_tvalid,
    input  wire                               m_axis_hmem_tready,
    output wire                               m_axis_hmem_tlast,
    output wire [AXIS_PCIE_RC_USER_WIDTH-1:0] m_axis_hmem_tuser,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_st_tx_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_st_tx_tkeep,
    output wire                               m_axis_st_tx_tvalid,
    input  wire                               m_axis_st_tx_tready,
    output wire                               m_axis_st_tx_tlast,
    output wire [AXIS_PCIE_RC_USER_WIDTH-1:0] m_axis_st_tx_tuser,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_axi_bridge_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_axi_bridge_tkeep,
    output wire                               m_axis_axi_bridge_tvalid,
    input  wire                               m_axis_axi_bridge_tready,
    output wire                               m_axis_axi_bridge_tlast,
    output wire [AXIS_PCIE_RC_USER_WIDTH-1:0] m_axis_axi_bridge_tuser
);

    localparam DEST_DMA = 2'd0;
    localparam DEST_HMEM = 2'd1;
    localparam DEST_ST_TX = 2'd2;
    localparam DEST_AXI_BRIDGE = 2'd3;

    reg [1:0] select_reg = DEST_DMA;
    reg frame_active_reg = 1'b0;

    wire first_beat = !frame_active_reg;
    wire [7:0] rc_tag = s_axis_tdata[71:64];
    wire match_hmem = ((rc_tag & HMEM_TAG_MASK[7:0]) == (HMEM_TAG[7:0] & HMEM_TAG_MASK[7:0]));
    wire match_axi_bridge = ((rc_tag & AXI_BRIDGE_TAG_MASK[7:0]) == (AXI_BRIDGE_TAG[7:0] & AXI_BRIDGE_TAG_MASK[7:0]));
    wire match_st_tx = ((rc_tag & ST_TX_TAG_MASK[7:0]) == (ST_TX_TAG[7:0] & ST_TX_TAG_MASK[7:0]));

    /* Decode the most-specific AXI-bridge tag window before the wider ST_TX
     * window.  This keeps the module safe even if a future top-level places a
     * bridge tag subrange inside the ST_TX class.
     */
    wire [1:0] select_now = match_hmem ? DEST_HMEM :
                            match_axi_bridge ? DEST_AXI_BRIDGE :
                            match_st_tx ? DEST_ST_TX : DEST_DMA;
    wire [1:0] select = first_beat ? select_now : select_reg;

    wire selected_ready = (select == DEST_HMEM)  ? m_axis_hmem_tready :
                          (select == DEST_ST_TX) ? m_axis_st_tx_tready :
                          (select == DEST_AXI_BRIDGE) ? m_axis_axi_bridge_tready : m_axis_dma_tready;

    assign s_axis_tready = selected_ready;

    assign m_axis_dma_tdata  = s_axis_tdata;
    assign m_axis_dma_tkeep  = s_axis_tkeep;
    assign m_axis_dma_tvalid = s_axis_tvalid && select == DEST_DMA;
    assign m_axis_dma_tlast  = s_axis_tlast;
    assign m_axis_dma_tuser  = s_axis_tuser;

    assign m_axis_hmem_tdata  = s_axis_tdata;
    assign m_axis_hmem_tkeep  = s_axis_tkeep;
    assign m_axis_hmem_tvalid = s_axis_tvalid && select == DEST_HMEM;
    assign m_axis_hmem_tlast  = s_axis_tlast;
    assign m_axis_hmem_tuser  = s_axis_tuser;

    assign m_axis_st_tx_tdata  = s_axis_tdata;
    assign m_axis_st_tx_tkeep  = s_axis_tkeep;
    assign m_axis_st_tx_tvalid = s_axis_tvalid && select == DEST_ST_TX;
    assign m_axis_st_tx_tlast  = s_axis_tlast;
    assign m_axis_st_tx_tuser  = s_axis_tuser;

    assign m_axis_axi_bridge_tdata  = s_axis_tdata;
    assign m_axis_axi_bridge_tkeep  = s_axis_tkeep;
    assign m_axis_axi_bridge_tvalid = s_axis_tvalid && select == DEST_AXI_BRIDGE;
    assign m_axis_axi_bridge_tlast  = s_axis_tlast;
    assign m_axis_axi_bridge_tuser  = s_axis_tuser;

    always @(posedge clk) begin
        if (s_axis_tvalid && s_axis_tready) begin
            if (first_beat) begin
                select_reg <= select_now;
                frame_active_reg <= !s_axis_tlast;
            end else if (s_axis_tlast) begin
                frame_active_reg <= 1'b0;
            end
        end

        if (rst) begin
            select_reg <= DEST_DMA;
            frame_active_reg <= 1'b0;
        end
    end

endmodule

`resetall
