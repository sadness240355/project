/* SPDX-License-Identifier: MIT
 * AXI4-Stream synchronous FIFO wrapper around Xilinx XPM_FIFO_AXIS.
 *
 * This wrapper is intentionally small and contains no RTL RAM indexing.  The
 * storage is implemented by Vivado's XPM FIFO macro so the RC completion path
 * can absorb AXI-payload backpressure without building large LUT muxes in the
 * SGDMA fabric.
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module axis_xpm_fifo #(
    parameter DATA_WIDTH = 256,
    parameter KEEP_ENABLE = 1,
    parameter KEEP_WIDTH = (DATA_WIDTH/32),
    parameter LAST_ENABLE = 1,
    parameter USER_ENABLE = 1,
    parameter USER_WIDTH = 75,
    parameter FIFO_DEPTH = 512,
    parameter PROG_FULL_MARGIN = (FIFO_DEPTH > 128) ? 64 : 8,
    parameter PROG_FULL_THRESH_VALUE = (FIFO_DEPTH > PROG_FULL_MARGIN + 8) ?
                                      (FIFO_DEPTH-PROG_FULL_MARGIN) :
                                      (FIFO_DEPTH-8),
    parameter COUNT_WIDTH = $clog2(FIFO_DEPTH)+1,
    parameter FIFO_MEMORY_TYPE = "block",
    // When enabled, XPM withholds a packet until its TLAST has entered the
    // FIFO.  This is used on the D2H RQ path so elastic AXI-R gaps cannot be
    // exposed as illegal TVALID gaps inside a PCIe Memory Write TLP.
    parameter PACKET_FIFO_ENABLE = "false"
) (
    input  wire                  clk,
    input  wire                  rst,

    input  wire [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0] s_axis_tkeep,
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire                  s_axis_tlast,
    input  wire [USER_WIDTH-1:0] s_axis_tuser,

    output wire [DATA_WIDTH-1:0] m_axis_tdata,
    output wire [KEEP_WIDTH-1:0] m_axis_tkeep,
    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire                  m_axis_tlast,
    output wire [USER_WIDTH-1:0] m_axis_tuser,

    output wire                  almost_full,
    output wire                  prog_full,
    output wire [COUNT_WIDTH-1:0] wr_data_count
);

    wire [KEEP_WIDTH-1:0] xpm_m_axis_tstrb;
    wire                  unused_almost_empty_axis;
    wire                  almost_full_axis;
    wire                  unused_dbiterr_axis;
    wire                  unused_prog_empty_axis;
    wire                  prog_full_axis;
    wire                  unused_sbiterr_axis;
    wire [COUNT_WIDTH-1:0] unused_rd_data_count_axis;
    wire [COUNT_WIDTH-1:0] wr_data_count_axis;

    assign almost_full   = almost_full_axis;
    assign prog_full     = prog_full_axis;
    assign wr_data_count = wr_data_count_axis;

    xpm_fifo_axis #(
        .CDC_SYNC_STAGES    (2),
        .CLOCKING_MODE      ("common_clock"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_DEPTH         (FIFO_DEPTH),
        .FIFO_MEMORY_TYPE   (FIFO_MEMORY_TYPE),
        .PACKET_FIFO        (PACKET_FIFO_ENABLE),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (PROG_FULL_THRESH_VALUE),
        .RD_DATA_COUNT_WIDTH(COUNT_WIDTH),
        .RELATED_CLOCKS     (0),
        .SIM_ASSERT_CHK     (0),
        .TDATA_WIDTH        (DATA_WIDTH),
        .TDEST_WIDTH        (1),
        .TID_WIDTH          (1),
        .TUSER_WIDTH        (USER_ENABLE ? USER_WIDTH : 1),
        .USE_ADV_FEATURES   ("000E"),
        .WR_DATA_COUNT_WIDTH(COUNT_WIDTH)
    ) xpm_fifo_axis_inst (
        .almost_empty_axis  (unused_almost_empty_axis),
        .almost_full_axis   (almost_full_axis),
        .dbiterr_axis       (unused_dbiterr_axis),
        .injectdbiterr_axis (1'b0),
        .injectsbiterr_axis (1'b0),
        .m_aclk             (clk),
        .m_axis_tdata       (m_axis_tdata),
        .m_axis_tdest       (),
        .m_axis_tid         (),
        .m_axis_tkeep       (m_axis_tkeep),
        .m_axis_tlast       (m_axis_tlast),
        .m_axis_tready      (m_axis_tready),
        .m_axis_tstrb       (xpm_m_axis_tstrb),
        .m_axis_tuser       (m_axis_tuser),
        .m_axis_tvalid      (m_axis_tvalid),
        .prog_empty_axis    (unused_prog_empty_axis),
        .prog_full_axis     (prog_full_axis),
        .rd_data_count_axis (unused_rd_data_count_axis),
        .s_aclk             (clk),
        .s_aresetn          (!rst),
        .s_axis_tdata       (s_axis_tdata),
        .s_axis_tdest       (1'b0),
        .s_axis_tid         (1'b0),
        .s_axis_tkeep       (KEEP_ENABLE ? s_axis_tkeep : {KEEP_WIDTH{1'b1}}),
        .s_axis_tlast       (LAST_ENABLE ? s_axis_tlast : 1'b1),
        .s_axis_tready      (s_axis_tready),
        .s_axis_tstrb       (KEEP_ENABLE ? s_axis_tkeep : {KEEP_WIDTH{1'b1}}),
        .s_axis_tuser       (USER_ENABLE ? s_axis_tuser : {USER_WIDTH{1'b0}}),
        .s_axis_tvalid      (s_axis_tvalid),
        .sbiterr_axis       (unused_sbiterr_axis),
        .wr_data_count_axis (wr_data_count_axis)
    );

endmodule

`resetall
