`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_tx_dma_read_engine — TX 引擎 DMA 读取模块
//
// 【模块概述】
// 将 SGL 描述符转化为实际数据搬运动作，是 buf_mgr 和下游 AXI-Stream 的桥梁。
// 从 HDR BRAM 读取包头、通过 AXI4 Master 从 DDR 搬运 payload，合并成数据流。
//
// 【SGL 消费逻辑】
// 监控 sgl_head/tail_ptr，有新 SGL 时读出并按类型分发到三个 FIFO：
// - hdr_req_fifo（SOF=1 的 SGL）：触发 HDR BRAM 读取
// - axi_req_fifo（SOF=0 或 QP1）：触发 AXI4 Master DDR 读事务
// - axi_len_fifo：记录字节偏移用于 tkeep 生成
//
// 【核心 FSM（axis_data_ctrl_ps）】
// - WRITE_HDR_1：读 HDR BRAM 低 512b（包头前 64B）
// - WRITE_HDR_2：读 HDR BRAM 高 512b（包头后 64B）
// - WRITE_DATA：从 AXI4 读回的内存数据逐拍推入 axis_data_fifo
//
// 【内部 FIFO】
// - axi_req_fifo（C_M_AXI_ADDR_WIDTH+13 bit × 32）：AXI 读地址 + 读长度/控制字段；默认 32bit 地址时为 45bit
// - axis_data_fifo（609b×512 URAM）：{pkt_count, tlast, tkeep, tdata}
// - axi_len_fifo（7b×32, FWFT）：末拍字节偏移+EOF
// - hdr_req_fifo（~43b×32, FWFT）：头部 BRAM 地址+长度+QP1 标志
//
// 【AXI4 Master 读接口】
// INCR burst，arsize=3'b110（64B/拍），单笔≤64 拍（4KB 边界）
//
// 【反压】
// axis_data_fifo_prog_full（≥448）时暂停 AXI 读取和头部写入，保留 64 拍余量
// o_dma_in_idle：所有 SGL 消耗完且 FIFO 清空后拉高
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_tx_dma_read_engine #(
    parameter C_M_AXI_DATA_WIDTH = 512,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_M_AXI_ID_WIDTH = 4,
    parameter C_AXIS_DATA_WIDTH = 512,
    parameter C_MAX_SGL_DEPTH = 128,
    parameter C_SGL_DATA_WIDTH = 82,
    parameter C_HDR_BUF_ADDR_WIDTH = 32,
    parameter C_HDR_BUF_DEPTH = 128
) (
    input core_clk,
    input core_rst,

    // AXI-4 Master signals
    output     [C_M_AXI_ID_WIDTH-1 : 0] m_axi_dma_arid,
    output reg [C_M_AXI_ADDR_WIDTH-1:0] m_axi_dma_araddr,
    output reg [                   7:0] m_axi_dma_arlen,
    output     [                   2:0] m_axi_dma_arsize,
    output     [                   1:0] m_axi_dma_arburst,
    output     [                   3:0] m_axi_dma_arcache,
    output     [                   2:0] m_axi_dma_arprot,
    output                              m_axi_dma_arvalid,
    input                               m_axi_dma_arready,
    input      [C_M_AXI_ID_WIDTH-1 : 0] m_axi_dma_rid,
    input      [C_M_AXI_DATA_WIDTH-1:0] m_axi_dma_rdata,
    input      [                   1:0] m_axi_dma_rresp,
    input                               m_axi_dma_rlast,
    input                               m_axi_dma_rvalid,
    output                              m_axi_dma_rready,
    output                              m_axi_dma_arlock,

    //AXI-4 Stream signals
    output reg [C_AXIS_DATA_WIDTH-1 : 0] m_axis_dma_tdata,
    output reg [C_AXIS_DATA_WIDTH/8-1:0] m_axis_dma_tkeep,
    output                               m_axis_dma_tvalid,
    input                                m_axis_dma_tready,
    output reg                           m_axis_dma_tlast,

    //SGL Buffer signals
    input  [                15:0] sgl_head_ptr,
    output [                14:0] sgl_bram_rd_addr,
    output                        sgl_bram_rd_en,
    input  [C_SGL_DATA_WIDTH-1:0] sgl_bram_rd_data,
    //Header buffer signals
    output [                14:0] hdr_bram_rd_addr,
    output                        hdr_bram_rd_en,
    input  [               511:0] hdr_bram_rd_data,

    //From Config space
    output reg o_dma_in_idle,
    output reg hdr_bram_pop
);

    `include "lrdma_defs.vh"
    //AXIS header and data write control FSM states
    localparam WRITE_HDR_1 = 2'b00;  // 输出包头的前 64 字节。用 hdr_req_fifo 弹出的地址读 HDR BRAM 的低 512 bit
                                     //   如果该 SGL 是零 payload 的“纯头部包”且头部 ≤ 64B，
                                     //   这拍就是最后一拍，下一拍直接回 WRITE_HDR_1 处理下一个 SGL
    localparam WRITE_HDR_2 = 2'b01;  // 输出包头的后 64 字节（读 HDR BRAM 高 512 bit）。
                                     //   头部 > 64B 时才需要；然后根据 SGL.EOF 决定继续 WRITE_DATA 还是回 WRITE_HDR_1
    localparam WRITE_DATA = 2'b10;  // 输出 payload。AXI4 读回的 DDR 数据逐拍推进 axis_data_fifo，
                                    //   遇到 m_axi_dma_rlast 表示本条 SGL 的数据部分送完，回 WRITE_HDR_1
    /////////////////////////
    localparam SGL_TAIL_PTR_WIDTH = 16;
    localparam SGL_BRAM_ADDR_WIDTH = clog2(C_MAX_SGL_DEPTH);



    localparam SGL_ADDR_WIDTH = 64;
    localparam SGL_ADDR_LSB = 0;
    localparam SGL_LEN_WIDTH = 16;
    localparam SGL_LEN_LSB = SGL_ADDR_WIDTH;
    localparam SGL_SOF_LSB = SGL_LEN_LSB + SGL_LEN_WIDTH;
    localparam SGL_EOF_LSB = SGL_SOF_LSB + 1;
    localparam SGL_QP1_HDR_VAL_LSB = 82;

    ///////////////////////////////////////////////
    //      AXI stream lrdma_common_fifo bit arrangements     //
    // tlast    tkeep      tdata                 //
    //  576     [575:512]  [511:0]               //
    ///////////////////////////////////////////////
    localparam AXIS_FIFO_TDATA_LSB = 0;
    localparam AXIS_FIFO_TDATA_W = 512;
    localparam AXIS_FIFO_TKEEP_LSB = 512;
    localparam AXIS_FIFO_TKEEP_W = 64;
    localparam AXIS_FIFO_TLAST_LSB = 576;
    localparam AXIS_FIFO_TLAST_W = 1;
    localparam integer AXIS_FIFO_DEPTH = 512;
    localparam [11:0] AXIS_FIFO_STOP = 12'd448;

    // AXI REQ FIFO Parameters
    localparam AXI_REQ_FIFO_WIDTH = C_M_AXI_ADDR_WIDTH + 13;  // 77

    //AXIS DATA FIFO Parameters  
    //Width = 512(DATA)+64(TKEEP)+1(TLAST) = 577
    localparam AXIS_DATA_FIFO_WIDTH = 512 + 64 + 1;

    //AXI LEN FIFO Parameters  
    //width = 6(LEN) + 1(EOF)= 7 
    localparam AXI_LEN_FIFO_WIDTH = 7;
    localparam AXI_LEN_FIFO_EOF_LSB = 6;

    //HDR REQ FIFO Parameters
    // C_HDR_BUF_ADDR_WIDTH + 6(LEN) + 1(len>64) + 1(len[6]) +1(EOF) + 1(QP1_HDR_VAL)
    localparam HDR_REQ_FIFO_LEN_W = 6;
    localparam HDR_REQ_FIFO_LEN_LSB = (C_HDR_BUF_ADDR_WIDTH + 1);
    localparam HDR_REQ_FIFO_EOF_LSB = (C_HDR_BUF_ADDR_WIDTH + 1) + HDR_REQ_FIFO_LEN_W + 2;
    localparam HDR_QP1_HDR_VAL_LSB = (C_HDR_BUF_ADDR_WIDTH + 1) + HDR_REQ_FIFO_LEN_W + 3;
    localparam HDR_REQ_FIFO_WIDTH = (C_HDR_BUF_ADDR_WIDTH + 1) + HDR_REQ_FIFO_LEN_W + 1 + 1 + 1 + 1;

    ////////
    localparam VALID_DIS = 1'b0;
    localparam VALID_EN = 1'b1;

    localparam TVALID_DIS = 1'b0;
    localparam TVALID_EN = 1'b1;
    //Declarations
    reg  [     SGL_TAIL_PTR_WIDTH-1:0] sgl_tail_ptr;
    wire                               sgl_hd_ptr_neq_tail_ptr;  // sgl头指针 != sgl尾指针
    reg                                sgl_data_push;

    wire [     AXI_REQ_FIFO_WIDTH-1:0] axi_req_fifo_data_in;
    wire [     AXI_LEN_FIFO_WIDTH-1:0] axi_len_fifo_data_in;
    wire [     HDR_REQ_FIFO_WIDTH-1:0] hdr_req_fifo_data_in;
    wire [AXIS_DATA_FIFO_WIDTH-1+32:0] axis_data_fifo_data_in;

    wire [     AXI_REQ_FIFO_WIDTH-1:0] axi_req_fifo_data_out;
    wire [     AXI_LEN_FIFO_WIDTH-1:0] axi_len_fifo_data_out;
    wire [     HDR_REQ_FIFO_WIDTH-1:0] hdr_req_fifo_data_out;
    wire [AXIS_DATA_FIFO_WIDTH-1+32:0] axis_data_fifo_data_out;

    wire                               axi_req_fifo_push;
    wire                               axi_len_fifo_push;
    wire                               hdr_req_fifo_push;
    wire                               axis_data_fifo_push;

    reg                                axi_req_fifo_pop;
    wire                               axi_len_fifo_pop;
    wire                               hdr_req_fifo_pop;
    wire                               axis_data_fifo_pop;

    wire                               axi_req_fifo_full;
    wire                               axi_len_fifo_full;
    wire                               hdr_req_fifo_full;
    wire                               axis_data_fifo_prog_full;
    reg  [                       11:0] data_count;



    wire                               axi_req_fifo_empty;
    wire                               axi_len_fifo_empty;
    wire                               hdr_req_fifo_empty;
    wire                               axis_data_fifo_empty;

    wire                               axi_req_fifo_wr_rst_busy;
    wire                               axi_len_fifo_wr_rst_busy;
    wire                               hdr_req_fifo_wr_rst_busy;
    wire                               axis_data_fifo_wr_rst_busy;

    wire                               axi_req_fifo_rd_rst_busy;
    wire                               axi_len_fifo_rd_rst_busy;
    wire                               hdr_req_fifo_rd_rst_busy;
    wire                               axis_data_fifo_rd_rst_busy;

    wire                               hdr_wr_en;
    wire                               hdr_write_active;
    reg                                axis_hdr_push;
    wire [                       63:0] axis_hdr_data_tkeep;
    wire [                       63:0] axis_payld_data_tkeep;
    wire                               axis_hdr_data_tlast;
    wire                               axis_payld_data_tlast;
    reg  [                        1:0] axis_data_ctrl_ps;
    reg  [                        1:0] axis_data_ctrl_ns;
    wire [       HDR_REQ_FIFO_LEN_W:0] hdr_len;
    wire                               hdr_len_or_bit_7_8;
    wire [       HDR_REQ_FIFO_LEN_W:0] hdr_len_r;
    wire                               hdr_len_or_bit_7_8_r;
    reg                                hdr_data_ps;
    reg                                hdr_data_ns;
    reg  [                        8:0] hdr_req_fifo_data_out_r;
    wire                               axi_dma_rlast_int;
    wire                               m_axis_dma_tvalid_int;
    reg                                m_axi_dma_arvalid_int;

    // R 通道 AXI register slice (Full Registered, 1+data 位宽)
    //  - 作用: 把 m_axi_dma_rready 从组合逻辑 (~prog_full & ~hdr_wr_en) 变为 lrdma_common_reg_slice 内部寄存输出,
    //          切断到 PCIe/QDMA 的长组合反压路径, 同时解耦 hdr_wr_en 瞬时门控死区.
    //  - S 侧: 接 AXI-R (m_axi_dma_rvalid/ready/rdata/rlast)
    //  - M 侧: 接下游消费 (r_buf_ready = ~prog_full & ~hdr_wr_en)
    wire                               r_buf_valid;
    wire                               r_buf_ready;
    wire                               r_buf_rlast;
    wire [     C_M_AXI_DATA_WIDTH-1:0] r_buf_rdata;
    wire                               payload_push_now;
    wire                               payload_rlast_now;
    wire [     C_M_AXI_DATA_WIDTH-1:0] payload_rdata_now;
    wire [                      576:0] hdr_data;
    reg  [                       31:0] pkt_count;
    reg  [                        3:0] wr_count;

    /////////////////////////////////////
    //          SGL Format             //
    // EOF   SOF    LEN       ADDR     //
    // 81    80     [79:64]   [63:0]   //
    /////////////////////////////////////

    always @(posedge core_clk) begin
        if (core_rst) wr_count <= 4'h0;
        else begin
            case ({
                hdr_req_fifo_pop, hdr_req_fifo_push
            })
                2'b01:   wr_count <= wr_count + 1'b1;
                2'b10:   wr_count <= wr_count - 1'b1;
                default: wr_count <= wr_count;
            endcase  // case ({hdr_req_fifo_pop,hdr_req_fifo_push})
        end
    end

    assign hdr_req_fifo_full = (wr_count >= 4'hD);

    //tail pointer logic
    always @(posedge core_clk) begin
        if (core_rst) sgl_tail_ptr <= {SGL_TAIL_PTR_WIDTH{1'b0}};
        else if (sgl_bram_rd_en) begin
            if (sgl_tail_ptr[(SGL_TAIL_PTR_WIDTH-2):0] == (C_MAX_SGL_DEPTH - 1'b1)) begin
                sgl_tail_ptr[(SGL_TAIL_PTR_WIDTH-2):0] <= {(SGL_TAIL_PTR_WIDTH - 1) {1'b0}};
                sgl_tail_ptr[SGL_TAIL_PTR_WIDTH-1]     <= ~sgl_tail_ptr[SGL_TAIL_PTR_WIDTH-1];
            end else sgl_tail_ptr[(SGL_TAIL_PTR_WIDTH-2):0] <= sgl_tail_ptr + {{(SGL_TAIL_PTR_WIDTH - 2) {1'b0}}, 1'b1};
        end
    end



    // =========================== SGL BRAM 信号分配 ===========================
    // sgl头指针 != sgl尾指针
    assign sgl_hd_ptr_neq_tail_ptr = (sgl_tail_ptr != sgl_head_ptr);
    // sgl 读使能
    assign sgl_bram_rd_en          = sgl_hd_ptr_neq_tail_ptr & (~axi_req_fifo_full) & (~axi_len_fifo_full) & (~hdr_req_fifo_full);
    // sgl 读地址 
    assign sgl_bram_rd_addr        = sgl_tail_ptr[SGL_BRAM_ADDR_WIDTH-1:0];
    // BRAM 读取延迟一个周期，以作为读取延迟为 1 的 BRAM 数据的有效读取方式
    always @(posedge core_clk) begin
        if (core_rst) sgl_data_push <= 1'b0;
        else sgl_data_push <= sgl_bram_rd_en;
    end
    //axi request lrdma_common_fifo push
    assign axi_req_fifo_push = sgl_data_push & ((~sgl_bram_rd_data[SGL_SOF_LSB]) | (sgl_bram_rd_data[SGL_QP1_HDR_VAL_LSB]));
    //axi length lrdma_common_fifo push
    assign axi_len_fifo_push = axi_req_fifo_push;
    //header lrdma_common_fifo push
    assign hdr_req_fifo_push = sgl_data_push & sgl_bram_rd_data[SGL_SOF_LSB];

    //axi request lrdma_common_fifo data
    assign axi_req_fifo_data_in = {sgl_bram_rd_data[(SGL_LEN_LSB+SGL_LEN_WIDTH-4):(SGL_LEN_LSB)], sgl_bram_rd_data[SGL_ADDR_LSB+:C_M_AXI_ADDR_WIDTH]};

    //axi length lrdma_common_fifo data
    assign axi_len_fifo_data_in = {sgl_bram_rd_data[SGL_EOF_LSB], sgl_bram_rd_data[SGL_LEN_LSB+:6]};

    //header lrdma_common_fifo data
    assign hdr_req_fifo_data_in = {sgl_bram_rd_data[SGL_QP1_HDR_VAL_LSB], sgl_bram_rd_data[SGL_EOF_LSB], (|sgl_bram_rd_data[(SGL_LEN_LSB+8):(SGL_LEN_LSB+7)]), sgl_bram_rd_data[SGL_LEN_LSB+:7], sgl_bram_rd_data[(SGL_ADDR_LSB+6)+:(C_HDR_BUF_ADDR_WIDTH+1)]};


    //axi request lrdma_common_fifo pop
    //  assign axi_req_fifo_pop = m_axi_dma_arready && (!axi_req_fifo_empty);
    //axi length lrdma_common_fifo pop
    assign axi_len_fifo_pop = (axis_data_ctrl_ps == WRITE_DATA) && axi_dma_rlast_int && (!axis_data_fifo_prog_full) && (!hdr_wr_en);
    //header lrdma_common_fifo pop
    assign hdr_req_fifo_pop = (!hdr_req_fifo_empty)    ? (((axis_data_ctrl_ps == WRITE_HDR_1) && (( hdr_len <= 7'd64) && (!hdr_len_or_bit_7_8) 
                                                      && hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB]) && (!axis_data_fifo_prog_full)
                                                      && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB]))
                                                     ||((axis_data_ctrl_ps == WRITE_HDR_2) && (!axis_data_fifo_prog_full) 
                                                        && hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB])
                                                     ||((axis_data_ctrl_ps == WRITE_DATA) && axi_dma_rlast_int && (!hdr_wr_en) && (!axis_data_fifo_prog_full))) 
                                                     : 1'b0;

    assign hdr_len = hdr_req_fifo_data_out[HDR_REQ_FIFO_LEN_LSB+:(HDR_REQ_FIFO_LEN_W+1)];
    assign hdr_len_or_bit_7_8 = hdr_req_fifo_data_out[HDR_REQ_FIFO_LEN_LSB+HDR_REQ_FIFO_LEN_W+1];

    always @(posedge core_clk) begin
        if (core_rst) begin
            data_count <= 12'd0;
        end else begin
            case ({
                axis_data_fifo_pop, axis_data_fifo_push
            })
                2'b01:   data_count <= data_count + 12'd1;
                2'b10:   data_count <= data_count - 12'd1;
                default: data_count <= data_count;
            endcase  // case (axis_data_fifo_pop,axis_data_fifo_push)
        end
    end


    assign axis_data_fifo_prog_full = data_count >= AXIS_FIFO_STOP;


    always @(posedge core_clk)
        if (core_rst) hdr_req_fifo_data_out_r <= 9'd0;
        else hdr_req_fifo_data_out_r <= hdr_req_fifo_data_out[HDR_REQ_FIFO_LEN_LSB+:(HDR_REQ_FIFO_LEN_W+3)];

    assign hdr_len_r            = hdr_req_fifo_data_out_r[0+:7];
    assign hdr_len_or_bit_7_8_r = hdr_req_fifo_data_out_r[7];

    // Axi-4 read interface signals
    // static assignments
    assign m_axi_dma_arid       = {C_M_AXI_ID_WIDTH{1'b1}};
    assign m_axi_dma_arsize     = 3'b110;
    assign m_axi_dma_arlock     = 1'b0;
    assign m_axi_dma_arburst    = 2'b01;
    assign m_axi_dma_arcache    = 4'h3;
    assign m_axi_dma_arprot     = 3'b000;

    // ============ R 通道 lrdma_common_reg_slice (Full Registered 双缓冲) ============
    lrdma_common_reg_slice #(
        .C_DATA_WIDTH(1 + C_M_AXI_DATA_WIDTH)
    ) u_r_chan_reg_slice (
        .aclk          (core_clk),
        .areset        (core_rst),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({m_axi_dma_rlast, m_axi_dma_rdata}),
        .S_VALID       (m_axi_dma_rvalid),
        .S_READY       (m_axi_dma_rready),
        .M_PAYLOAD_DATA({r_buf_rlast, r_buf_rdata}),
        .M_VALID       (r_buf_valid),
        .M_READY       (r_buf_ready)
    );

    // Accept payload only when the output FIFO has space and no header beat is being staged.
    assign r_buf_ready       = (~axis_data_fifo_prog_full) & (~hdr_wr_en);
    assign payload_push_now  = r_buf_valid & r_buf_ready;
    assign payload_rlast_now = r_buf_rlast;
    assign payload_rdata_now = r_buf_rdata;

    // axi_dma_rlast_int: 本拍实际推入 axis_data_fifo 的 last beat
    // (语义等价原版 m_axi_dma_rvalid ? m_axi_dma_rlast, 但仅在 ready 时才置1)
    assign axi_dma_rlast_int = payload_push_now & payload_rlast_now;


    // Dynamic assignments
    always @(*) begin
        m_axi_dma_araddr = axi_req_fifo_data_out[SGL_ADDR_LSB+:C_M_AXI_ADDR_WIDTH];
        m_axi_dma_arlen  = {1'b0, axi_req_fifo_data_out[(C_M_AXI_ADDR_WIDTH+6)+:7]} - {{7{1'b0}}, (~|axi_req_fifo_data_out[C_M_AXI_ADDR_WIDTH+:6])};
    end

    always @(*) begin
        if (((!m_axi_dma_arvalid_int) && (!axi_req_fifo_empty))) axi_req_fifo_pop = 1'b1;
        else if (m_axi_dma_arvalid_int && m_axi_dma_arready && (!axi_req_fifo_empty)) axi_req_fifo_pop = 1'b1;
        else axi_req_fifo_pop = 1'b0;
    end

    always @(posedge core_clk) begin
        if (core_rst) begin
            m_axi_dma_arvalid_int <= 1'b0;
        end else if ((!axi_req_fifo_empty) && ((!m_axi_dma_arvalid_int) || (m_axi_dma_arvalid_int && m_axi_dma_arready))) begin
            m_axi_dma_arvalid_int <= 1'b1;
        end else if (m_axi_dma_arvalid_int && m_axi_dma_arready) begin
            m_axi_dma_arvalid_int <= 1'b0;
        end
    end
    assign m_axi_dma_arvalid = m_axi_dma_arvalid_int;


    //Axi stream signal assignments
    ///////////////////////////////////////////////
    //      AXI stream lrdma_common_fifo bit arrangements     //
    // tlast    tkeep      tdata                 //
    //  576     [575:512]  [511:0]               //
    ///////////////////////////////////////////////

    // URAM standard-read response is buffered by an elastic credit adapter.
    // The FIFO may return a beat several clocks after RD_EN; the adapter
    // reserves space before issuing the read and holds AXIS stable on stalls.
    wire [AXIS_DATA_FIFO_WIDTH+31:0] axis_data_stream_data;
    wire axis_data_fifo_data_valid;
    always @(*) begin
        m_axis_dma_tdata = axis_data_stream_data[AXIS_FIFO_TDATA_LSB+:AXIS_FIFO_TDATA_W];
        m_axis_dma_tkeep = axis_data_stream_data[AXIS_FIFO_TKEEP_LSB+:AXIS_FIFO_TKEEP_W];
        m_axis_dma_tlast = axis_data_stream_data[AXIS_FIFO_TLAST_LSB];
    end
    lrdma_fifo_read_elastic #(
        .DATA_WIDTH(AXIS_DATA_FIFO_WIDTH+32),.DEPTH(8)
    ) axis_data_read_pipe_inst (
        .clk(core_clk),.rst(core_rst),
        .fifo_busy(axis_data_fifo_rd_rst_busy || axis_data_fifo_wr_rst_busy),
        .fifo_empty(axis_data_fifo_empty),.fifo_rd_en(axis_data_fifo_pop),
        .fifo_data(axis_data_fifo_data_out),
        .fifo_data_valid(axis_data_fifo_data_valid),
        .m_axis_data(axis_data_stream_data),
        .m_axis_valid(m_axis_dma_tvalid_int),.m_axis_ready(m_axis_dma_tready)
    );

    assign m_axis_dma_tvalid = m_axis_dma_tvalid_int;

    // ================================= AXI 读事务 FIFO =================================

    //Fifo comprising of axi address and length
    //32(ADDR)+8(LEN)+SOF_EOF= 42

    lrdma_common_xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),              //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),            //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (32),                  //positive integer									
        .WRITE_DATA_WIDTH   (AXI_REQ_FIFO_WIDTH),  //positive integer									
        .WR_DATA_COUNT_WIDTH(6),                   //positive integer									
        .PROG_FULL_THRESH   (27),                  //positive integer									
        .FULL_RESET_VALUE   (0),                   //positive integer; 0 or 1									
        .READ_MODE          ("std"),               //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (1),                   //positive integer;									
        .READ_DATA_WIDTH    (AXI_REQ_FIFO_WIDTH),  //positive integer									
        .RD_DATA_COUNT_WIDTH(6),                   //positive integer									
        .PROG_EMPTY_THRESH  (5),                   //positive integer									
        .DOUT_RESET_VALUE   ("0"),                 //string									
        .WAKEUP_TIME        (0)                    //positive integer; 0 or 2;									
    ) axi_req_fifo (
        .sleep        (1'b0),
        .rst          (core_rst),
        .wr_clk       (core_clk),
        .wr_en        (axi_req_fifo_push),
        .din          (axi_req_fifo_data_in),
        .full         (axi_req_fifo_full),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (axi_req_fifo_wr_rst_busy),
        .rd_en        (axi_req_fifo_pop),
        .dout         (axi_req_fifo_data_out),
        .empty        (axi_req_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (axi_req_fifo_rd_rst_busy),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   ()
    );


    // ================================= AXI 读数据 FIFO =================================

    // AXI 读数据 FIFO：深度 512，位宽 609bit。AXI R 通道可反压，停止线留出 64 拍。
    // 数据格式：{pkt_count[31:0], tlast, tkeep[63:0], tdata[511:0]}

    lrdma_common_xpm_fifo_sync_wrap #(
        .USE_ADV_FEATURES   ("1707"), // enable data_valid for the elastic read adapter
        .FIFO_MEMORY_TYPE   ("ultra"),                    //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),                   //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (AXIS_FIFO_DEPTH),            //positive integer									
        .WRITE_DATA_WIDTH   (AXIS_DATA_FIFO_WIDTH + 32),  //positive integer									
        .WR_DATA_COUNT_WIDTH(10),                         //positive integer									
        .PROG_FULL_THRESH   (AXIS_FIFO_STOP),             //positive integer									
        .FULL_RESET_VALUE   (0),                          //positive integer; 0 or 1									
        .READ_MODE          ("std"),                      //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (3),                          //positive integer;									
        .READ_DATA_WIDTH    (AXIS_DATA_FIFO_WIDTH + 32),  //positive integer									
        .RD_DATA_COUNT_WIDTH(10),                         //positive integer									
        .PROG_EMPTY_THRESH  (12),                         //positive integer									
        .DOUT_RESET_VALUE   ("0"),                        //string									
        .WAKEUP_TIME        (0)                           //positive integer; 0 or 2;									
    ) axis_data_fifo (
        .sleep        (1'b0),
        .rst          (core_rst),
        .wr_clk       (core_clk),
        .wr_en        (axis_data_fifo_push),
        .din          (axis_data_fifo_data_in),
        .full         (),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (axis_data_fifo_wr_rst_busy),
        .rd_en        (axis_data_fifo_pop),
        .dout         (axis_data_fifo_data_out),
        .empty        (axis_data_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (axis_data_fifo_rd_rst_busy),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   (axis_data_fifo_data_valid)
    );


    // ================================= AXI LEN FIFO =================================

    // AXI Stream 接口逻辑在 AXI-4 事务的最后一个节拍（beat）上，根据剩余长度推导 tkeep 信号。

    // 该 FIFO 包含一个用于存储 AXI 长度的字段，该字段将用于计算 AXI-4 事务中最后一个节拍（beat）的 AXIS 接口 tkeep 信号。
    // 该 AXI 长度字段的位宽为 6 位，因为在 512 位宽的数据总线配置下，若要确保整个事务的数据量处于 4K 字节的范围内，最多只能包含 64 个节拍。
    // 该长度字段的 [5:0] 位将表示最后一个节拍中包含的有效数据字节数，进而用于生成相应的 TKEEP 信号。

    //width = 6(LEN) + 1(EOF)= 7 

    lrdma_common_xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),              //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),            //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (32),                  //positive integer									
        .WRITE_DATA_WIDTH   (AXI_LEN_FIFO_WIDTH),  //positive integer									
        .WR_DATA_COUNT_WIDTH(6),                   //positive integer									
        .PROG_FULL_THRESH   (27),                  //positive integer									
        .FULL_RESET_VALUE   (0),                   //positive integer; 0 or 1									
        .READ_MODE          ("fwft"),              //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (0),                   //positive integer;									
        .READ_DATA_WIDTH    (AXI_LEN_FIFO_WIDTH),  //positive integer									
        .RD_DATA_COUNT_WIDTH(6),                   //positive integer									
        .PROG_EMPTY_THRESH  (5),                   //positive integer									
        .DOUT_RESET_VALUE   ("0"),                 //string									
        .WAKEUP_TIME        (0)                    //positive integer; 0 or 2;									
    ) axi_len_fifo (
        .sleep        (1'b0),
        .rst          (core_rst),
        .wr_clk       (core_clk),
        .wr_en        (axi_len_fifo_push),
        .din          (axi_len_fifo_data_in),
        .full         (axi_len_fifo_full),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (axi_len_fifo_wr_rst_busy),
        .rd_en        (axi_len_fifo_pop),
        .dout         (axi_len_fifo_data_out),
        .empty        (axi_len_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (axi_len_fifo_rd_rst_busy),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   ()
    );


    // ================================= 头部请求 FIFO =================================

    // 用于存储头部地址和长度的头部请求 FIFO
    // C_HDR_BUF_ADDR_WIDTH + 8(LEN) + 1(EOF)

    lrdma_common_xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),              //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),            //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (32),                  //positive integer									
        .WRITE_DATA_WIDTH   (HDR_REQ_FIFO_WIDTH),  //positive integer									
        .WR_DATA_COUNT_WIDTH(6),                   //positive integer									
        .PROG_FULL_THRESH   (27),                  //positive integer									
        .FULL_RESET_VALUE   (0),                   //positive integer; 0 or 1									
        .READ_MODE          ("fwft"),              //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (0),                   //positive integer;									
        .READ_DATA_WIDTH    (HDR_REQ_FIFO_WIDTH),  //positive integer									
        .RD_DATA_COUNT_WIDTH(6),                   //positive integer									
        .PROG_EMPTY_THRESH  (5),                   //positive integer									
        .DOUT_RESET_VALUE   ("0"),                 //string									
        .WAKEUP_TIME        (0)                    //positive integer; 0 or 2;									
    ) hdr_req_fifo (
        .sleep        (1'b0),
        .rst          (core_rst),
        .wr_clk       (core_clk),
        .wr_en        (hdr_req_fifo_push),
        .din          (hdr_req_fifo_data_in),
        .full         (),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (hdr_req_fifo_wr_rst_busy),
        .rd_en        (hdr_req_fifo_pop),
        .dout         (hdr_req_fifo_data_out),
        .empty        (hdr_req_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (hdr_req_fifo_rd_rst_busy),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   ()
    );


    //AXIS data write control FSM

    always @(posedge core_clk) begin
        if (core_rst) axis_data_ctrl_ps <= WRITE_HDR_1;
        else if ((!axis_data_fifo_prog_full)) axis_data_ctrl_ps <= axis_data_ctrl_ns;
    end

    always @(*) begin
        case (axis_data_ctrl_ps)
            // 从 HDR BRAM 读低 512b（包头前 64 字节），推入 axis_data_fifo
            WRITE_HDR_1: begin
                if (!hdr_req_fifo_empty) begin
                    // 如果 hdr_len > 64B → 进 HDR_2
                    if (((hdr_len > 7'd64) || hdr_len_or_bit_7_8) && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB])) axis_data_ctrl_ns = WRITE_HDR_2;

                    // 如果 hdr_len ≤ 64B 且 EOF=1 → 只有包头，直接回 HDR_1
                    else if (hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB] && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB])) axis_data_ctrl_ns = WRITE_HDR_1;

                    else axis_data_ctrl_ns = WRITE_DATA;

                end else axis_data_ctrl_ns = WRITE_HDR_1;
            end

            // 从 HDR BRAM 读高 512b（包头后 64 字节），推入 axis_data_fifo
            WRITE_HDR_2: begin
                // EOF=1 → 回 HDR_1（无 payload）
                if (hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB]) axis_data_ctrl_ns = WRITE_HDR_1;

                // EOF=0 → 进 WRITE_DATA
                else
                    axis_data_ctrl_ns = WRITE_DATA;
            end

            // 从 AXI4 Master 读回的内存数据推入 axis_data_fifo
            WRITE_DATA: begin
                // rlast 到来 → 回 HDR_1，开始下一个包
                if (axi_dma_rlast_int && (!hdr_wr_en)) begin
                    axis_data_ctrl_ns = WRITE_HDR_1;
                end //
                
                // m_axi_dma_rvalid 的每一拍数据写入 FIFO
                else
                    axis_data_ctrl_ns = WRITE_DATA;
            end
            default: begin
                axis_data_ctrl_ns = WRITE_HDR_1;
            end
        endcase  // case (axis_data_ctrl_ps)
    end

    always @(posedge core_clk) begin
        if (core_rst) axis_hdr_push <= 1'b0;
        else axis_hdr_push <= hdr_bram_rd_en;
    end

    assign axis_data_fifo_push = payload_push_now | axis_hdr_push;

    //HEADER BRAM interface Signals

    assign hdr_bram_rd_en      = (!axis_data_fifo_prog_full) && (!hdr_req_fifo_empty) && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB]) && ((axis_data_ctrl_ps == WRITE_HDR_1) || (axis_data_ctrl_ps == WRITE_HDR_2));  //|| ((axis_data_ctrl_ps == WRITE_DATA) && axi_dma_rlast_int && (!hdr_wr_en)));
    assign hdr_write_active    = hdr_bram_rd_en | axis_hdr_push;
    assign hdr_wr_en           = hdr_write_active;

    assign hdr_bram_rd_addr    = {hdr_req_fifo_data_out[C_HDR_BUF_ADDR_WIDTH:1], (axis_data_ctrl_ps == WRITE_HDR_2)};

    always @(posedge core_clk) begin
        if (core_rst) begin
            hdr_bram_pop <= 1'b0;
        end else begin
            hdr_bram_pop <= hdr_bram_rd_en && ((axis_data_ctrl_ps == WRITE_HDR_2) || ((hdr_len <= 7'd64) && (!hdr_len_or_bit_7_8)));
        end
    end

    //Header buffering logic to save the extra rd_en given to header bram when the lrdma_common_fifo is full.

    always @(posedge core_clk)
        if (core_rst) hdr_data_ps <= 1'b0;
        else hdr_data_ps <= hdr_data_ns;

    always @(*) begin
        case (hdr_data_ps)
            1'b0: begin
                if (axis_hdr_push && ((hdr_len_r > 7'd64) || hdr_len_or_bit_7_8_r)) hdr_data_ns = 1'b1;
                else hdr_data_ns = 1'b0;
            end
            1'b1: begin
                if (axis_hdr_push) hdr_data_ns = 1'b0;
                else hdr_data_ns = 1'b1;
            end
        endcase  // case (hdr_data_ps)
    end

    always @(posedge core_clk)
        if (core_rst) pkt_count <= 32'd0;
        else if (axis_data_fifo_push) pkt_count <= pkt_count + 1'b1;

    genvar i;

    generate
        for (i = 0; i < 64; i = i + 1) begin : loop1
            assign axis_hdr_data_tkeep[i]   = ((!hdr_data_ps) && ((hdr_len_r > 7'd64) || hdr_len_or_bit_7_8_r)) ? 1'b1 : ((hdr_len_r[5:0] >= (i + 1)) || (~|hdr_len_r[5:0])) ? 1'b1 : 1'b0;
            assign axis_payld_data_tkeep[i] = axi_dma_rlast_int ? ((axi_len_fifo_data_out[5:0] >= (i + 1)) || (~|axi_len_fifo_data_out[5:0])) ? 1'b1 : 1'b0 : 1'b1;
        end
    endgenerate

    assign hdr_data               = {axis_hdr_data_tlast, axis_hdr_data_tkeep, hdr_bram_rd_data};

    assign axis_hdr_data_tlast    = (((!hdr_data_ps) && (!((hdr_len_r > 7'd64) || hdr_len_or_bit_7_8_r))) || hdr_data_ps) ? hdr_req_fifo_data_out_r[8] : 1'b0;
    assign axis_payld_data_tlast  = axi_len_fifo_data_out[AXI_LEN_FIFO_EOF_LSB] & axi_dma_rlast_int;
    assign axis_data_fifo_data_in = (hdr_wr_en) ? {pkt_count, hdr_data} : {pkt_count, axis_payld_data_tlast, axis_payld_data_tkeep, payload_rdata_now};


    always @(posedge core_clk) begin
        if (core_rst) o_dma_in_idle <= 1'b1;
        else if (sgl_hd_ptr_neq_tail_ptr) o_dma_in_idle <= 1'b0;
        else if (!sgl_hd_ptr_neq_tail_ptr && hdr_req_fifo_empty && axi_req_fifo_empty) o_dma_in_idle <= 1'b1;
    end


endmodule  // lrdma_tx_dma_read_engine


module lrdma_fifo_read_elastic #(
    parameter integer DATA_WIDTH = 609,
    parameter integer DEPTH      = 8
) (
    input  wire                  clk,
    input  wire                  rst,

    input  wire                  fifo_busy,
    input  wire                  fifo_empty,
    output wire                  fifo_rd_en,
    input  wire [DATA_WIDTH-1:0]  fifo_data,
    input  wire                  fifo_data_valid,

    output wire [DATA_WIDTH-1:0]  m_axis_data,
    output wire                  m_axis_valid,
    input  wire                  m_axis_ready
);
    localparam integer PTR_WIDTH = $clog2(DEPTH);
    localparam integer CNT_WIDTH = $clog2(DEPTH + 1);
    localparam [CNT_WIDTH-1:0] CAPACITY = DEPTH;

    (* ram_style = "distributed" *)
    reg [DATA_WIDTH-1:0] payload [0:DEPTH-1];
    reg [PTR_WIDTH-1:0] wr_ptr, rd_ptr;
    reg [CNT_WIDTH-1:0] reserved, queued;

    wire reset_active = rst || fifo_busy;
    wire returned = fifo_data_valid && !reset_active;
    wire consumed = m_axis_valid && m_axis_ready;

    // No combinational READY path back to the memory request port.
    assign fifo_rd_en = !reset_active && !fifo_empty && (reserved < CAPACITY);
    assign m_axis_valid = !reset_active && (queued != 0);
    assign m_axis_data = payload[rd_ptr];

    always @(posedge clk) begin
        // Reset flushes ownership/credits; stale writes during reset are unobservable.
        // Keep the global reset out of the wide payload RAM write enable.
        if (fifo_data_valid) payload[wr_ptr] <= fifo_data;
    end

    always @(posedge clk) begin
        if (reset_active) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            reserved <= 0;
            queued <= 0;
        end else begin
            if (returned) wr_ptr <= wr_ptr + 1'b1;
            if (consumed) rd_ptr <= rd_ptr + 1'b1;
            case ({fifo_rd_en, consumed})
                2'b10: reserved <= reserved + 1'b1;
                2'b01: reserved <= reserved - 1'b1;
                default: begin end
            endcase
            case ({returned, consumed})
                2'b10: queued <= queued + 1'b1;
                2'b01: queued <= queued - 1'b1;
                default: begin end
            endcase
        end
    end

    // Elaboration/simulation guards only; no timing exceptions or fake ready.
    initial begin
        if (DATA_WIDTH < 1 || DEPTH < 4 || (DEPTH & (DEPTH-1)) != 0)
            $fatal(1, "lrdma_fifo_read_elastic requires power-of-two DEPTH >= 4");
    end
`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (!reset_active) begin
            if (reserved > CAPACITY || queued > reserved)
                $fatal(1, "FIFO credit invariant failed");
            if (returned && (reserved == 0))
                $fatal(1, "FIFO response without reserved credit");
            if (returned && queued == CAPACITY && !consumed)
                $fatal(1, "FIFO response overflow");
        end
    end
`endif
endmodule
