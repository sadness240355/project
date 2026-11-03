`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_rx_intr_ctrl — RX 引擎中断控制器
//
// 【模块概述】
// 汇总 RX 引擎中产生的多类中断事件，向 QP 管理器的中断寄存器提供
// 中断请求信号。
//
// 【中断源类型】
// 1. pkt_valdn_err：包验证错误中断
// 2. mad_rcvd：MAD 管理报文接收中断
// 3. fatal_err：QP Fatal 错误中断
///////////////////////////////////////////////////////////////////////////////
module lrdma_rx_intr_ctrl (
    // System level inputs & outputs
    input wire clk,
    input wire rst_n,

    input wire lrdma_enabled,

    input  wire pkt_valdn_err_intr_en,
    output wire pkt_valdn_err_intr,
    output reg  pkt_valdn_err_intr_sts,
    input  wire clr_pkt_valdn_err_intr,

    input  wire mad_pkt_rcvd_intr_en,
    output wire mad_pkt_rcvd_intr,
    output reg  mad_pkt_rcvd_intr_sts,
    input  wire clr_mad_pkt_rcvd_intr,

    input  wire fatal_err_intr_en,
    output wire fatal_err_intr,
    output reg  fatal_err_intr_sts,
    input  wire clr_fatal_err_intr,

    input wire set_pkt_valdn_err_intr,
    input wire set_mad_pkt_rcvd_intr,
    input wire set_fatal_err_intr
);

    wire pkt_valdn_err_intr_sts_en;
    wire mad_pkt_rcvd_intr_sts_en;
    wire fatal_err_intr_sts_en;

    assign pkt_valdn_err_intr_sts_en = set_pkt_valdn_err_intr | clr_pkt_valdn_err_intr;
    assign mad_pkt_rcvd_intr_sts_en  = set_mad_pkt_rcvd_intr | clr_mad_pkt_rcvd_intr;
    assign fatal_err_intr_sts_en     = set_fatal_err_intr | clr_fatal_err_intr;

    always @(posedge clk)
        if (!rst_n || lrdma_enabled) begin
            pkt_valdn_err_intr_sts <= 1'b0;
            mad_pkt_rcvd_intr_sts  <= 1'b0;
            fatal_err_intr_sts     <= 1'b0;
        end else begin
            if (pkt_valdn_err_intr_sts_en) pkt_valdn_err_intr_sts <= set_pkt_valdn_err_intr;
            if (mad_pkt_rcvd_intr_sts_en) mad_pkt_rcvd_intr_sts <= set_mad_pkt_rcvd_intr;
            if (fatal_err_intr_sts_en) fatal_err_intr_sts <= set_fatal_err_intr;
        end

    assign pkt_valdn_err_intr = pkt_valdn_err_intr_en & pkt_valdn_err_intr_sts;
    assign mad_pkt_rcvd_intr  = mad_pkt_rcvd_intr_en & mad_pkt_rcvd_intr_sts;
    assign fatal_err_intr     = fatal_err_intr_en & fatal_err_intr_sts;

endmodule
