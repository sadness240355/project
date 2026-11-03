///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_common_reg_fifo_sync — 纯寄存器实现的轻量级同步 FIFO
//
// 【模块概述】
// 不实例化 BRAM/分布式 RAM，纯 fifo_reg 寄存器数组实现，
// 适用于深度较浅（8 或 16）的场景。
//
// 【两种读模式】
// - C_READ_MODE=1：组合直通（fall-through），零延迟
// - C_READ_MODE=0：寄存器输出，rd_en 有效时锁存新值
//
// 【空满判断】
// 读写指针 + MSB 翻转位机制
// almost_full 通过 nxt_wrptr 预判（提前一拍满预警）
///////////////////////////////////////////////////////////////////////////////
`default_nettype wire `timescale 1 ps / 1 ps

module lrdma_common_reg_fifo_sync #(
    parameter [0:0] C_READ_MODE = 1'b0,
    parameter C_FIFO_DEPTH = 8,
    parameter C_FIFO_WRCNTR_WIDTH = 4,
    parameter C_FIFO_DATA_WIDTH = 32
) (
    input                          clk,
    input                          reset,
    input                          wr_en,
    input  [C_FIFO_DATA_WIDTH-1:0] din,
    output                         full,
    output                         almost_full,
    input                          rd_en,
    output [C_FIFO_DATA_WIDTH-1:0] dout,
    output                         empty
);
    reg  [C_FIFO_WRCNTR_WIDTH-1:0] i;

    reg  [  C_FIFO_DATA_WIDTH-1:0] fifo_reg  [C_FIFO_DEPTH-1:0];
    reg  [C_FIFO_WRCNTR_WIDTH-1:0] wrptr;
    reg  [C_FIFO_WRCNTR_WIDTH-1:0] rdptr;

    wire [  C_FIFO_DATA_WIDTH-1:0] dout_i;
    reg  [  C_FIFO_DATA_WIDTH-1:0] dout_r;

    wire [C_FIFO_WRCNTR_WIDTH-1:0] nxt_wrptr;

    assign nxt_wrptr = wrptr + 'd1;

    always @(posedge clk)
        if (reset) wrptr <= {(C_FIFO_WRCNTR_WIDTH) {1'b0}};
        else if (wr_en) wrptr <= nxt_wrptr;

    always @(posedge clk)
        if (reset) rdptr <= {(C_FIFO_WRCNTR_WIDTH) {1'b0}};
        else if (rd_en) rdptr <= rdptr + 'd1;

    always @(posedge clk)
        for (i = 0; i < C_FIFO_DEPTH; i = i + 1) begin
            if (reset) fifo_reg[i] <= {(C_FIFO_DATA_WIDTH) {1'b0}};
            else if ((i[C_FIFO_WRCNTR_WIDTH-2:0] == wrptr[C_FIFO_WRCNTR_WIDTH-2:0]) && wr_en) fifo_reg[i] <= din;
        end

    assign dout_i = fifo_reg[rdptr[C_FIFO_WRCNTR_WIDTH-2:0]];

    always @(posedge clk)
        if (reset) dout_r <= {(C_FIFO_DATA_WIDTH) {1'b0}};
        else if (rd_en) dout_r <= dout_i;

    assign dout        = C_READ_MODE ? dout_i : dout_r;

    assign full        = ({~wrptr[C_FIFO_WRCNTR_WIDTH-1], wrptr[C_FIFO_WRCNTR_WIDTH-2:0]} == rdptr) ? 1'b1 : 1'b0;
    assign almost_full = ({~nxt_wrptr[C_FIFO_WRCNTR_WIDTH-1], nxt_wrptr[C_FIFO_WRCNTR_WIDTH-2:0]} == rdptr) ? 1'b1 : 1'b0;
    assign empty       = (wrptr == rdptr) ? 1'b1 : 1'b0;

endmodule
