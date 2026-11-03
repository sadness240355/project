`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_tx_data_realign — TX 引擎数据重对齐模块
//
// 【模块概述】
// 消除 AXI-Stream 数据流中协议头部和 payload 之间的空隙（gap）。
// 位于 DMA 输出和 CRC 封装之间。
//
// 【空隙产生原因】
// HDR BRAM 和 AXI4 读通道的数据独立对齐，包头长度（54B/58B/70B/74B/78B/90B/94B）
// 不是 64B 的整数倍，导致 tkeep 中出现中间空洞。
//
// 【空隙检测】
// 首拍通过 tkeep[63]/[54]/[58] 确定偏移位置（byte_cnt）：
// 0, 6, 10, 14, 26, 54, 58 字节固定模式
//
// 【桶形移位重对齐】
// 两级数据寄存器（data_d1 暂存前拍溢出，data_d2 为输出）：
// 例 byte_cnt=6：data_d2 = {s_axis_tdata[463:0], data_d1[47:0]}
// 例 byte_cnt=58：data_d2 = {s_axis_tdata[47:0], data_d1[463:0]}
//
// 【extra_cycle】
// 移位后有残余数据时产生额外一拍输出
//
// 【设计目标】
// 确保输出数据流每拍有效数据从 bit[0] 开始连续排列，无中间空洞
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_tx_data_realign #(
    parameter C_AXIS_DATA_WIDTH  = 512,
    parameter C_M_AXI_DATA_WIDTH = 512,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_M_AXI_ID_WIDTH   = 1,
    parameter IP2BUS_LEN_WIDTH   = 8
) (
    input core_clk,
    input core_rst,

    //AXI streaming interface signals
    output     [C_AXIS_DATA_WIDTH-1 : 0] m_axis_tdata,
    output reg [C_AXIS_DATA_WIDTH/8-1:0] m_axis_tkeep,
    output                               m_axis_tvalid,
    input                                m_axis_tready,
    output reg                           m_axis_tlast,

    input  [C_AXIS_DATA_WIDTH-1 : 0] s_axis_tdata,
    input  [C_AXIS_DATA_WIDTH/8-1:0] s_axis_tkeep,
    input                            s_axis_tvalid,
    input                            s_axis_tlast,
    output                           s_axis_tready
);
    wire                           fifo_full;
    reg                            pkt_start;
    reg  [C_AXIS_DATA_WIDTH-1 : 0] data_d1;
    reg  [C_AXIS_DATA_WIDTH-1 : 0] data_d2;
    reg                            fifo_wr_en;
    reg  [                    6:0] byte_cnt;
    reg                            extra_cycle;
    reg                            extra_cycle_1;
    reg  [                   63:0] last_tkeep;

    assign fifo_full     = fifo_wr_en && ~m_axis_tready;
    assign m_axis_tdata  = data_d2;
    assign m_axis_tvalid = fifo_wr_en;
    assign s_axis_tready = (~extra_cycle && ~extra_cycle_1 && ~fifo_full);


    always @(posedge core_clk) begin
        if (core_rst) begin
            data_d1       <= 'd0;
            data_d2       <= 'd0;
            pkt_start     <= 1'b0;
            m_axis_tkeep  <= 'd0;
            fifo_wr_en    <= 1'b0;
            m_axis_tlast  <= 1'b0;
            extra_cycle   <= 1'b0;
            extra_cycle_1 <= 1'b0;
            last_tkeep    <= 64'd00;
            byte_cnt      <= 7'h00;
        end else begin
            if (~pkt_start && s_axis_tvalid && ~fifo_full && ~extra_cycle_1) begin
                if (s_axis_tlast) begin
                    data_d2      <= {s_axis_tdata};
                    fifo_wr_en   <= 1'b1;
                    m_axis_tkeep <= s_axis_tkeep;
                    m_axis_tlast <= 1'b1;
                end else begin
                    m_axis_tlast <= 1'b0;
                    pkt_start    <= 1'b1;
                    if (s_axis_tkeep[63]) begin
                        fifo_wr_en   <= 1'b1;
                        byte_cnt     <= 'd0;
                        data_d2      <= {s_axis_tdata};
                        m_axis_tkeep <= s_axis_tkeep;
                    end else if (~s_axis_tkeep[54] || ~s_axis_tkeep[58]) begin
                        fifo_wr_en <= 1'b0;
                        byte_cnt   <= ~s_axis_tkeep[54] ? 'd54 : 'd58;
                        last_tkeep <= s_axis_tkeep;
                        data_d1    <= {s_axis_tdata};
                    end else begin
                        fifo_wr_en <= 1'b0;
                    end
                end
            end else if (((pkt_start && s_axis_tvalid) || extra_cycle || extra_cycle_1) && ~fifo_full) begin
                if (s_axis_tlast || extra_cycle || extra_cycle_1) begin
                    if (byte_cnt != 'd0 && ~extra_cycle && ~extra_cycle_1) begin
                        case (byte_cnt)
                            'd6: begin
                                extra_cycle   <= s_axis_tkeep[58];
                                extra_cycle_1 <= ~m_axis_tready && ~s_axis_tkeep[58];
                                m_axis_tlast  <= ~s_axis_tkeep[58];
                                pkt_start     <= (~m_axis_tready && ~s_axis_tkeep[58]) || s_axis_tkeep[58];
                            end
                            'd10: begin
                                extra_cycle   <= s_axis_tkeep[54];
                                extra_cycle_1 <= ~m_axis_tready && ~s_axis_tkeep[54];
                                m_axis_tlast  <= ~s_axis_tkeep[54];
                                pkt_start     <= (~m_axis_tready && ~s_axis_tkeep[54]) || s_axis_tkeep[54];
                            end
                            'd14: begin
                                extra_cycle   <= s_axis_tkeep[50];
                                extra_cycle_1 <= ~m_axis_tready && ~s_axis_tkeep[50];
                                m_axis_tlast  <= ~s_axis_tkeep[50];
                                pkt_start     <= (~m_axis_tready && ~s_axis_tkeep[50]) || s_axis_tkeep[50];
                            end
                            'd26: begin
                                extra_cycle   <= s_axis_tkeep[38];
                                extra_cycle_1 <= ~m_axis_tready && ~s_axis_tkeep[38];
                                m_axis_tlast  <= ~s_axis_tkeep[38];
                                pkt_start     <= (~m_axis_tready && ~s_axis_tkeep[38]) || s_axis_tkeep[38];
                            end
                            'd54: begin
                                extra_cycle   <= s_axis_tkeep[10];
                                extra_cycle_1 <= ~m_axis_tready && ~s_axis_tkeep[10];
                                m_axis_tlast  <= ~s_axis_tkeep[10];
                                pkt_start     <= (~m_axis_tready && ~s_axis_tkeep[10]) || s_axis_tkeep[10];
                            end
                            'd58: begin
                                extra_cycle   <= s_axis_tkeep[6];
                                extra_cycle_1 <= ~m_axis_tready && ~s_axis_tkeep[6];
                                m_axis_tlast  <= ~s_axis_tkeep[6];
                                pkt_start     <= (~m_axis_tready && ~s_axis_tkeep[6]) || s_axis_tkeep[6];
                            end
                            default: begin
                                extra_cycle  <= 1'b0;
                                m_axis_tlast <= 1'b0;
                                pkt_start    <= 1'b0;
                            end
                        endcase
                    end else begin
                        pkt_start     <= ~m_axis_tready && ~extra_cycle_1;  //1'b0;
                        extra_cycle   <= extra_cycle ? ~m_axis_tready : 1'b0;  //~m_axis_tready && ~extra_cycle_1;  // 1'b0;
                        m_axis_tlast  <= ~extra_cycle_1;
                        extra_cycle_1 <= ~m_axis_tready && ~m_axis_tvalid && s_axis_tready && s_axis_tvalid && s_axis_tlast;  //1'b0;
                    end
                end
                case (byte_cnt)
                    'd0: begin
                        data_d1 <= {s_axis_tdata};
                        if (~s_axis_tkeep[6] && ~s_axis_tlast) begin
                            byte_cnt   <= 'd6;
                            fifo_wr_en <= 1'b0;
                            last_tkeep <= s_axis_tkeep;
                        end else if (~s_axis_tkeep[10] && ~s_axis_tlast) begin
                            byte_cnt   <= 'd10;
                            fifo_wr_en <= 1'b0;
                            last_tkeep <= s_axis_tkeep;
                        end else if (~s_axis_tkeep[14] && ~s_axis_tlast) begin
                            byte_cnt   <= 'd14;
                            fifo_wr_en <= 1'b0;
                            last_tkeep <= s_axis_tkeep;
                        end else if (~s_axis_tkeep[26] && ~s_axis_tlast) begin
                            byte_cnt   <= 'd26;
                            fifo_wr_en <= 1'b0;
                            last_tkeep <= s_axis_tkeep;
                        end else begin
                            //fifo_wr_en <= 1'b1;
                            //fifo_wr_en <= ~extra_cycle_1;
                            fifo_wr_en   <= ~(m_axis_tvalid && m_axis_tready && m_axis_tlast);
                            data_d2      <= {s_axis_tdata};
                            m_axis_tkeep <= s_axis_tkeep;
                        end
                    end
                    'd6: begin
                        data_d2[511:48] <= s_axis_tdata[463:0];
                        data_d2[47:0]   <= data_d1[47:0];
                        data_d1[47:0]   <= s_axis_tdata[511:464];
                        //fifo_wr_en <= 1'b1;
                        fifo_wr_en      <= ~(m_axis_tvalid && m_axis_tlast);  //~extra_cycle_1;
                        m_axis_tkeep    <= extra_cycle ? last_tkeep : {s_axis_tkeep[57:0], last_tkeep[5:0]};
                        last_tkeep[5:0] <= s_axis_tkeep[63:58];
                    end
                    'd10: begin
                        data_d2[511:80] <= s_axis_tdata[431:0];
                        data_d2[79:0]   <= data_d1[79:0];
                        data_d1[79:0]   <= s_axis_tdata[511:432];
                        //fifo_wr_en <= 1'b1;
                        fifo_wr_en      <= ~(m_axis_tvalid && m_axis_tlast);  //~extra_cycle_1;
                        m_axis_tkeep    <= extra_cycle ? last_tkeep : {s_axis_tkeep[53:0], last_tkeep[9:0]};
                        last_tkeep[9:0] <= s_axis_tkeep[63:54];
                    end
                    'd14: begin
                        data_d2[511:112] <= s_axis_tdata[399:0];
                        data_d2[111:0]   <= data_d1[111:0];
                        data_d1[111:0]   <= s_axis_tdata[511:400];
                        //fifo_wr_en <= 1'b1;
                        fifo_wr_en       <= ~(m_axis_tvalid && m_axis_tlast);  //~extra_cycle_1;
                        m_axis_tkeep     <= extra_cycle ? last_tkeep : {s_axis_tkeep[49:0], last_tkeep[13:0]};
                        last_tkeep[13:0] <= s_axis_tkeep[63:50];
                    end
                    'd26: begin
                        data_d2[511:208] <= s_axis_tdata[303:0];
                        data_d2[207:0]   <= data_d1[207:0];
                        data_d1[207:0]   <= s_axis_tdata[511:304];
                        //fifo_wr_en <= 1'b1;
                        fifo_wr_en       <= ~(m_axis_tvalid && m_axis_tlast);  // ~extra_cycle_1;
                        m_axis_tkeep     <= extra_cycle ? last_tkeep : {s_axis_tkeep[37:0], last_tkeep[25:0]};
                        last_tkeep[25:0] <= s_axis_tkeep[63:38];
                    end
                    'd54: begin
                        data_d2[511:432] <= s_axis_tdata[79:0];
                        data_d2[431:0]   <= data_d1[431:0];
                        data_d1[431:0]   <= s_axis_tdata[511:80];
                        //fifo_wr_en <= 1'b1;
                        fifo_wr_en       <= ~(m_axis_tvalid && m_axis_tlast);  //~extra_cycle_1 && m_axis_tlast;
                        m_axis_tkeep     <= extra_cycle ? last_tkeep : {s_axis_tkeep[9:0], last_tkeep[53:0]};
                        last_tkeep[53:0] <= s_axis_tkeep[63:10];
                    end
                    'd58: begin
                        data_d2[511:464] <= s_axis_tdata[47:0];
                        data_d2[463:0]   <= data_d1[463:0];
                        data_d1[463:0]   <= s_axis_tdata[511:48];
                        //fifo_wr_en <= 1'b1;
                        fifo_wr_en       <= ~(m_axis_tvalid && m_axis_tlast);  //~extra_cycle_1 && m_axis_tlast;
                        m_axis_tkeep     <= extra_cycle ? last_tkeep : {s_axis_tkeep[5:0], last_tkeep[57:0]};
                        last_tkeep[57:0] <= s_axis_tkeep[63:6];
                    end
                    default: begin
                        fifo_wr_en <= 1'b0;
                    end
                endcase
            end else begin
                if (m_axis_tready) begin
                    fifo_wr_en    <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    pkt_start     <= (m_axis_tvalid && m_axis_tlast) ? 1'b0 : pkt_start;
                    extra_cycle_1 <= 1'b0;
                end else if (~extra_cycle_1) begin
                    extra_cycle_1 <= (m_axis_tvalid && m_axis_tlast) ? 1'b1 : extra_cycle_1;
                end
            end
        end
    end

endmodule
