///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_common_xpm_fifo_sync_wrap — Original Vendor XPM 同步 FIFO 薄包装
//
// 【模块概述】
// 对 Original Vendor xpm_fifo_sync 原语的直接包装。
// 单时钟域同步 FIFO，支持可配置的存储类型、ECC、读模式、
// 编程式空满阈值等参数。
//
// 【关键参数】
// - FIFO_MEMORY_TYPE：auto/block/distributed/ultra
// - READ_MODE：std（标准）/ fwft（First-Word-Fall-Through）
// - FIFO_READ_LATENCY：读延迟周期数
// - PROG_FULL/EMPTY_THRESH：编程式满/空阈值
//
// 【包装目的】
// 统一实例化接口，便于全局参数管理和替换
///////////////////////////////////////////////////////////////////////////////
`default_nettype wire `timescale 1 ps / 1 ps
module lrdma_common_xpm_fifo_sync_wrap #(

    // Common module parameters
    parameter FIFO_MEMORY_TYPE = "auto",
    parameter ECC_MODE         = "no_ecc",

    parameter integer FIFO_WRITE_DEPTH    = 2048,
    parameter integer WRITE_DATA_WIDTH    = 32,
    parameter integer WR_DATA_COUNT_WIDTH = 1,
    parameter integer PROG_FULL_THRESH    = 10,
    parameter integer FULL_RESET_VALUE    = 0,
    parameter         USE_ADV_FEATURES    = "0707",

    parameter         READ_MODE           = "std",
    parameter integer FIFO_READ_LATENCY   = 1,
    parameter integer READ_DATA_WIDTH     = WRITE_DATA_WIDTH,
    parameter integer RD_DATA_COUNT_WIDTH = 1,
    parameter integer PROG_EMPTY_THRESH   = 10,
    parameter         DOUT_RESET_VALUE    = "0",

    parameter WAKEUP_TIME = 0

) (

    // Common module ports
    input wire sleep,
    input wire rst,

    // Write Domain ports
    input  wire                           wr_clk,
    input  wire                           wr_en,
    input  wire [   WRITE_DATA_WIDTH-1:0] din,
    output wire                           full,
    output wire                           prog_full,
    output wire [WR_DATA_COUNT_WIDTH-1:0] wr_data_count,
    output wire                           overflow,
    output wire                           wr_rst_busy,
    output wire                           almost_full,
    output wire                           wr_ack,

    // Read Domain ports
    input  wire                           rd_en,
    output wire [    READ_DATA_WIDTH-1:0] dout,
    output wire                           empty,
    output wire                           prog_empty,
    output wire [RD_DATA_COUNT_WIDTH-1:0] rd_data_count,
    output wire                           underflow,
    output wire                           rd_rst_busy,
    output wire                           almost_empty,
    output wire                           data_valid,

    // ECC Related ports
    input  wire injectsbiterr,
    input  wire injectdbiterr,
    output wire sbiterr,
    output wire dbiterr
);

    xpm_fifo_sync #(
        .FIFO_MEMORY_TYPE   (FIFO_MEMORY_TYPE),     //string; "auto", "block", "distributed", or "ultra";									
        .USE_ADV_FEATURES   (USE_ADV_FEATURES),
        .ECC_MODE           (ECC_MODE),             //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (FIFO_WRITE_DEPTH),     //positive integer									
        .WRITE_DATA_WIDTH   (WRITE_DATA_WIDTH),     //positive integer									
        .WR_DATA_COUNT_WIDTH(WR_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_FULL_THRESH   (PROG_FULL_THRESH),     //positive integer									
        .FULL_RESET_VALUE   (FULL_RESET_VALUE),     //positive integer; 0 or 1									
        .READ_MODE          (READ_MODE),            //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (FIFO_READ_LATENCY),    //positive integer;									
        .READ_DATA_WIDTH    (READ_DATA_WIDTH),      //positive integer									
        .RD_DATA_COUNT_WIDTH(RD_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_EMPTY_THRESH  (PROG_EMPTY_THRESH),    //positive integer									
        .DOUT_RESET_VALUE   (DOUT_RESET_VALUE),     //string									
        .WAKEUP_TIME        (WAKEUP_TIME)           //positive integer; 0 or 2;									            
    ) XPM_FIFO_SYNC (
        .sleep        (sleep),
        .rst          (rst),
        .wr_clk       (wr_clk),
        .wr_en        (wr_en),
        .din          (din),
        .full         (full),
        .prog_full    (prog_full),
        .wr_data_count(wr_data_count),
        .overflow     (overflow),
        .wr_rst_busy  (wr_rst_busy),
        .almost_full  (almost_full),
        .wr_ack       (wr_ack),
        .rd_en        (rd_en),
        .dout         (dout),
        .empty        (empty),
        .prog_empty   (prog_empty),
        .rd_data_count(rd_data_count),
        .underflow    (underflow),
        .rd_rst_busy  (rd_rst_busy),
        .almost_empty (almost_empty),
        .data_valid   (data_valid),
        .injectsbiterr(injectsbiterr),
        .injectdbiterr(injectdbiterr),
        .sbiterr      (sbiterr),
        .dbiterr      (dbiterr)
    );

endmodule
