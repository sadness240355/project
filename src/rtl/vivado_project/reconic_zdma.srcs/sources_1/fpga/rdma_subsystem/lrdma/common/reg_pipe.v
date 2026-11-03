///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_common_reg_pipe — 单级 valid-ready 流水线寄存器
//
// 【模块概述】
// 极简的单拍延迟寄存器，用于数据通路中改善时序。
// rdy_out = rdy_in | ~vld_out：下游能接收或寄存器为空时接受上游
// 零气泡传输——通路畅通时每周期传递一拍数据。
//
// 【注意】
// 仅对 valid/data 路径做寄存器切断，ready 路径存在组合逻辑
// （从 rdy_in 到 rdy_out），适用于前向路径是时序瓶颈的场景。
///////////////////////////////////////////////////////////////////////////////
`default_nettype wire `timescale 1 ps / 1 ps

module lrdma_common_reg_pipe #(
    parameter C_DATA_WIDTH = 32
) (
    input                         clk,
    input                         reset,
    input      [C_DATA_WIDTH-1:0] din,
    input                         vld_in,
    output                        rdy_out,
    output reg [C_DATA_WIDTH-1:0] dout,
    output reg                    vld_out,
    input                         rdy_in
);

    assign rdy_out = rdy_in | ~vld_out;

    always @(posedge clk)
        if (reset) vld_out <= 1'b0;
        else if (rdy_out) vld_out <= vld_in;

    always @(posedge clk) if (rdy_out) dout <= din;

endmodule
