//-----------------------------------------------------------------------------
// wqe_proc_dre.v
//-----------------------------------------------------------------------------
//
// Filename        : wqe_proc_dre.v
// Version         : v1.0
// Description     : WQE处理器数据重新对齐逻辑实现了消除数据包头与数据之间空隙的逻辑。 
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
// Naming Conventions:
//      active low signals:                     "*_n"
//      clock signals:                          "clk", "clk_div#", "clk_#x" 
//      reset signals:                          "rst", "rst_n" 
//      parameters:                             "C_*" 
//      user defined types:                     "*_TYPE" 
//      state machine next state:               "*_ns" 
//      state machine current state:            "*_cs" 
//      combinatorial signals:                  "*_com" 
//      pipelined or register delay signals:    "*_d#" 
//      counter signals:                        "*cnt*"
//      clock enable signals:                   "*_ce" 
//      internal version of output port         "*_i"
//      device pins:                            "*_pin" 
//      component instantiations:               "<MODULE>I_<#|FUNC>
//-----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// Module Declaration
// WQE处理器数据重对齐逻辑，根据keep信号移除数据包头部和数据之间的空隙，实现数据流的紧凑重排。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module wqe_proc_dre #(
    parameter C_AXIS_DATA_WIDTH = 512,
    parameter C_M_AXI_DATA_WIDTH = 512,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_M_AXI_ID_WIDTH = 1,
    parameter IP2BUS_LEN_WIDTH = 8,
    //parameter C_MAX_WRDATA_BUF_NUM = 128,
    //parameter C_EN_WR_RETRY_DATA_BUF = 1,
    parameter C_EN_DEBUG = 0
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
    output                           s_axis_tready,

    // no use
    input                           i_data_ptr_fifo_wr_en,
    input  [C_M_AXI_ADDR_WIDTH-1:0] i_data_ptr_fifo_data,
    output                          o_data_buf_fifo_empty,

    //Axi master interface（no use）
    output reg [C_M_AXI_ADDR_WIDTH -1 : 0] maxi_addr,
    output                                 maxi_wr_rdn,
    output reg                             maxi_en,
    output reg [     IP2BUS_LEN_WIDTH-1:0] maxi_len,
    input                                  maxi_wdata_rdy,
    output     [   C_M_AXI_DATA_WIDTH-1:0] maxi_wdata,
    input                                  maxi_done,
    input                                  maxi_busy,
    input                                  maxi_bvalid,
    input                                  maxi_error,

    output [  C_M_AXI_ID_WIDTH-1:0] m_axi_awid,
    output [C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output [                   7:0] m_axi_awlen,
    output [                   2:0] m_axi_awsize,
    output [                   1:0] m_axi_awburst,
    output [                   3:0] m_axi_awcache,
    output [                   2:0] m_axi_awprot,
    output                          m_axi_awlock,
    input                           m_axi_awready,
    output                          m_axi_awvalid,
    output [                 511:0] m_axi_wdata,
    output                          m_axi_wlast,
    output [                  63:0] m_axi_wstrb,
    input                           m_axi_wready,
    output                          m_axi_wvalid,
    output                          m_axi_bready,
    input                           m_axi_bvalid,

    input                               i_debug_cnt_en,
    input                               i_debug_cnt_clr,
    output reg [                  15:0] o_wqe_proc_wr_retry_buf_full_cnt,
    input      [C_M_AXI_ADDR_WIDTH-1:0] i_blk_base_addr

);

    //localparam DATA_PTR_FIFO_DEPTH = C_MAX_WRDATA_BUF_NUM;
    localparam [15:0] ETH_TYPE_IPv4 = 16'h0008;
    localparam [15:0] ETH_TYPE_IPv6 = 16'hdd86;
    localparam RDMA_WRITE_FIRST = 8'b00000110;
    localparam RDMA_WRITE_MIDDLE = 8'b00000111;
    localparam RDMA_WRITE_LAST = 7'b0000100;  //Bit-0 indicates if immediate data is present or not. Only upper bits are compared  
    localparam RDMA_WRITE_LAST_IMM = 8'b00001001;
    localparam RDMA_WRITE_ONLY = 7'b0000101;  //Bit-0 indicates if immediate data is present or not. Only upper bits are compared 
    localparam IDLE = 2'b00;
    localparam START_WR = 2'b01;
    localparam WAIT_ON_AXI_DONE = 2'b10;
    localparam AW_FIFO_DWIDTH = 32 + C_M_AXI_ADDR_WIDTH + 7;
    localparam W_FIFO_DWIDTH = 32 + 7;

    wire                              s_axis_stall;
    reg  [                     511:0] data_buf_din;
    reg                               rdma_write;
    reg                               rdma_write_in_progress;
    reg                               data_buf_wr_en;
    reg                               data_buf_rd_en;
    wire [                     511:0] data_buf_dout;
    reg  [                      15:0] pkt_len;
    reg  [                       1:0] ddr_wr_cs;
    reg                               data_ptr_fifo_rd_en;
    wire [C_M_AXI_ADDR_WIDTH - 1 : 0] data_ptr_fifo_rd_data;
    reg  [C_M_AXI_ADDR_WIDTH - 1 : 0] curr_wr_ddr_addr;
    reg                               retried_pkt;
    reg  [                       7:0] pkt_wr_cnt;
    reg  [                       7:0] pkt_rd_cnt;
    reg  [                       7:0] pkt_bvalid_cnt;

    reg                               pkt_start;
    reg  [   C_AXIS_DATA_WIDTH-1 : 0] data_d1;
    reg  [   C_AXIS_DATA_WIDTH-1 : 0] data_d2;
    reg  [   C_AXIS_DATA_WIDTH/8-1:0] tkeep_d1;
    reg  [   C_AXIS_DATA_WIDTH/8-1:0] tkeep_d2;
    reg                               fifo_wr_en;
    reg  [                       6:0] byte_cnt;
    reg                               extra_cycle;
    reg                               extra_cycle_1;
    reg  [                      63:0] last_tkeep;
    wire                              data_buf_fifo_empty;
    wire                              data_ptr_fifo_empty;
    wire                              data_ptr_fifo_full;
    reg  [                      31:0] tnfr_id;
    wire [        AW_FIFO_DWIDTH-1:0] awfifo_din;
    wire [        AW_FIFO_DWIDTH-1:0] awfifo_dout;
    wire [                       6:0] awlen;
    wire [                       6:0] wlen;
    wire [         W_FIFO_DWIDTH-1:0] wfifo_din;
    wire [         W_FIFO_DWIDTH-1:0] wfifo_dout;
    wire [                      31:0] aw_tnfr_id;
    wire                              wvalid_i;
    reg                               wchnl_ns;
    reg                               wchnl_cs;
    reg                               wlast_i;
    reg                               wfifo_rd_en;
    reg  [                       6:0] nxt_wcnt;
    reg                               data_buf_rd_en2;
    wire [                      31:0] w_tnfr_id;
    reg                               wfifo_wr_en;
    wire                              s_axis_stall_3;
    wire                              s_axis_stall_2;
    wire                              s_axis_stall_1;
    wire                              wfifo_empty;
    wire                              awfifo_empty;
    wire                              awfifo_rd_en;
    reg                               data_buf_wr_en2;
    wire [                     511:0] data_buf_dout2;
    wire                              data_buf_fifo_empty2;
    reg                               awfifo_wr_en;
    reg  [                       6:0] wcnt;
    wire                              wready_i;
    reg  [                     511:0] m_axi_wdata_int;
    reg                               m_axi_wlast_int;
    reg                               m_axi_wvalid_int;
    wire [    C_M_AXI_ADDR_WIDTH-1:0] i_data_ptr_fifo_data_d_addr;
    reg  [    C_M_AXI_ADDR_WIDTH-1:0] i_data_ptr_fifo_data_d;
    reg                               i_data_ptr_fifo_wr_en_d;


    assign fifo_full     = s_axis_stall || (fifo_wr_en && ~m_axis_tready);
    assign m_axis_tdata  = data_d2;
    assign m_axis_tvalid = fifo_wr_en;
    assign s_axis_tready = (~extra_cycle && ~extra_cycle_1 && ~(s_axis_stall || (fifo_wr_en && ~m_axis_tready)));

    //////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Debug counters logic //////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    generate
        if (C_EN_DEBUG == 1) begin : DEBUG_EN
            always @(posedge core_clk) begin
                if (core_rst) begin
                    o_wqe_proc_wr_retry_buf_full_cnt <= 16'h0000;
                end else begin
                    if (i_debug_cnt_clr) begin
                        o_wqe_proc_wr_retry_buf_full_cnt <= 16'h0000;
                    end else if (i_debug_cnt_en && s_axis_stall) begin
                        o_wqe_proc_wr_retry_buf_full_cnt <= o_wqe_proc_wr_retry_buf_full_cnt + 1'b1;
                    end
                end
            end
        end else begin
            always @(posedge core_clk) begin
                o_wqe_proc_wr_retry_buf_full_cnt <= 16'h0000;
            end
        end
    endgenerate


    always @(posedge core_clk) begin
        if (core_rst) begin
            data_d1       <= 'd0;
            data_d2       <= 'd0;
            tkeep_d1      <= 'd0;
            tkeep_d2      <= 'd0;
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
                        tkeep_d1   <= {s_axis_tkeep};
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
                            'd30: begin
                                extra_cycle   <= s_axis_tkeep[34];
                                extra_cycle_1 <= ~m_axis_tready && ~s_axis_tkeep[34];
                                m_axis_tlast  <= ~s_axis_tkeep[34];
                                pkt_start     <= (~m_axis_tready && ~s_axis_tkeep[34]) || s_axis_tkeep[34];
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
                        end else if (~s_axis_tkeep[30] && ~s_axis_tlast) begin
                            byte_cnt   <= 'd30;
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
                    'd30: begin
                        data_d2[511:240] <= s_axis_tdata[271:0];
                        data_d2[239:0]   <= data_d1[239:0];
                        data_d1[239:0]   <= s_axis_tdata[511:272];
                        //fifo_wr_en <= 1'b1;
                        fifo_wr_en       <= ~(m_axis_tvalid && m_axis_tlast);  // ~extra_cycle_1;
                        m_axis_tkeep     <= extra_cycle ? last_tkeep : {s_axis_tkeep[33:0], last_tkeep[29:0]};
                        last_tkeep[29:0] <= s_axis_tkeep[63:34];
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

    assign o_data_buf_fifo_empty = 1'b1;
    assign s_axis_stall          = 1'b0;
    always @(posedge core_clk) begin
        maxi_en <= 1'b0;
    end
    assign m_axi_awid    = {C_M_AXI_ID_WIDTH{1'b0}};
    assign m_axi_awaddr  = {C_M_AXI_ADDR_WIDTH{1'b0}};
    assign m_axi_awlen   = 8'h00;
    assign m_axi_awsize  = 3'b000;
    assign m_axi_awburst = 2'b00;
    assign m_axi_awcache = 4'h0;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awvalid = 1'b0;
    assign m_axi_wdata   = {C_M_AXI_DATA_WIDTH{1'b0}};
    assign m_axi_wstrb   = {C_M_AXI_DATA_WIDTH / 8{1'b0}};
    assign m_axi_wlast   = 1'b0;
    assign m_axi_wvalid  = 1'b0;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_bready  = 1'b1;
    //end
    //endgenerate

endmodule
