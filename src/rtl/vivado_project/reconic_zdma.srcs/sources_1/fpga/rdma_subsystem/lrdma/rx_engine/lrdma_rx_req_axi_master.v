`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_rx_req_axi_master — RX 引擎请求包数据负载 AXI4 写主机模块
//
// 【模块概述】
// 根据 RX 引擎请求路径输出的元数据，通过 AXI4 写通道完成 payload、错误状态、MAD、DB update 等写入。
// 支持多种包类型的不同写入目标。
//
// 【写入目标（按 pkt_type 编码）】
// - 0 (DISCARD)：丢弃，不写入
// - 1 (ERROR)：写入错误缓冲区
// - 2 (MAD)：MAD 管理报文，写入 MAD 专用缓冲区
// - 3 (DB_UPDATE)：RQ 门铃更新（更新 RQ 写指针到 host 指定的门铃地址）
// - 4 (IN_ERR_STS)：Fatal 错误状态写入
//
// 【内部 FIFO（3 个）】
// - AW FIFO：AXI 写地址通道元数据（地址、burst 长度、类型）
// - W FIFO：写数据通道元数据（burst 长度、缓冲起始地址、wstrb）
// - Post-AXI FIFO：传输完成后的后处理信息（中断、计数、门铃更新）
//
// 【写数据 FSM（3 状态）】
// - 2'd0 IDLE/INIT：从 W FIFO 读取传输元数据，初始化 burst 参数
// - 2'd1 EXTRA_READ：对头部带偏移的包类型额外读一拍以完成跨拍数据对齐
// - 2'd2 BURST_WRITE：逐拍从缓冲区读 512 位数据并通过 AXI W 通道写出
//
// 【Payload 拼接】
// 根据包类型（IPv4/IPv6）执行数据对齐，
// 头部带偏移时跨 512 位边界拼接相邻两拍
// wstrb 精确到字节粒度
//
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module lrdma_rx_req_axi_master #(
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_M_AXI_AWUSER_WIDTH = 16,
    parameter C_QP_INDX_WIDTH = 8,
    parameter C_REQ_BUF_ADDR_WIDTH = 9,
    parameter C_NUM_QP = 32
) (
    // System level interface
    input clk,
    input reset,

    // 通过这些接口将接收到的请求包元数据存放进 FIFO 中，以下就是请求包元数据
    // req_pkt_hdr_val 接口
    input  [                    31:0] req_pkt_id,
    input  [                     3:0] req_pkt_type,
    input                             req_pkt_ip_ver,
    input  [                    15:0] req_rq_pi_db,
    input  [     C_QP_INDX_WIDTH-1:0] req_qpid,
    input  [  C_M_AXI_ADDR_WIDTH-1:0] req_dest_addr,
    input  [C_REQ_BUF_ADDR_WIDTH-1:0] req_buf_addr_offset,  // lrdma_rx_cmac_intf pkt_buf_offset
    input  [                    12:0] req_pkt_len,
    // req_imm_data字段：
    // 发生错误时：对应手册 Packet Validation Error Syndrome 或者 Error Status Format
    // 正常包时：{1bit 硬件握手标志, 1bit MAD标志, 1bit RQ中断使能, 14bit RQ门铃地址, (11-C_QP_INDX_WIDTH) bit 高位填 0, C_QP_INDX_WIDTH位 QP号, 16bit 门铃值}
    input  [                    43:0] req_imm_data,
    input                             req_tnfr_vld,
    output                            rdy_to_req_tnfr,      // ready to request transfer

    // 通过这些接口将接收到的响应包元数据存放进 FIFO 中，以下就是响应包元数据
    // rsp_pkt_hdr_val 接口
    input  [31:0] rsp_pkt_id,
    input  [31:0] rsp_imm_data,    // rsp_imm_data字段，与 rx_rsp_hdr_val模块 配合
    input         rsp_tnfr_vld,
    output        rdy_to_rsp_tnfr, // ready to response transfer

    // LRDMA register interface
    // LRDMA 寄存器接口
    input      [C_M_AXI_ADDR_WIDTH-1:0] i_err_buf_ba,
    input      [                  15:0] i_err_buf_sz,
    input      [                  15:0] i_err_num_bufs,
    output reg [C_M_AXI_ADDR_WIDTH-1:0] o_err_buf_ca,
    output reg [                  15:0] o_err_buf_wrptr,
    input      [C_M_AXI_ADDR_WIDTH-1:0] i_in_err_sts_q_ba,
    input      [                  15:0] i_in_err_sts_q_sz,
    input      [                  15:0] i_in_err_sts_q_depth,
    output reg [C_M_AXI_ADDR_WIDTH-1:0] o_in_err_sts_q_ca,
    output reg [                  15:0] o_in_err_sts_q_wrptr,
    output reg                          incr_inv_pkt_cnt,
    output reg [                  15:0] o_mad_pkt_cnt,

    input lrdma_en_pulse,

    // 至中断控制器
    output reg set_pkt_valdn_err_intr,
    output     set_mad_pkt_rcvd_intr,
    output reg set_qp_fatal_intr,

    // 请求包缓冲区接口，有包头
    input      [C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addra,
    output reg                            req_pkt_buf_enb,
    output reg [C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addrb,
    input      [                   511:0] req_pkt_buf_doutb,

    // AXI Master interface to DDR
    // 仅支持 AXI4 写通道（写地址&写数据）和 响应通道
    // tip: 写的都是有效载荷，不包含任何协议头
    // 只用到了写通道
    output [                     0:0] m_axi_awid,
    output [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output [C_M_AXI_AWUSER_WIDTH-1:0] m_axi_awuser,   // AWUSER[31:16] = rq_pi_db, [15:12] = pkt_type, [11:0] = qpid
    output [                     7:0] m_axi_awlen,
    output [                     2:0] m_axi_awsize,
    output [                     1:0] m_axi_awburst,
    output [                     3:0] m_axi_awcache,
    output [                     2:0] m_axi_awprot,
    output                            m_axi_awvalid,
    input                             m_axi_awready,
    output [                   511:0] m_axi_wdata,
    output [                    63:0] m_axi_wstrb,
    output                            m_axi_wlast,
    output                            m_axi_wvalid,
    input                             m_axi_wready,
    output                            m_axi_awlock,
    // AXI 读请求常驱 0；写响应 B 通道用于写事务完成与错误响应检查
    input  [                     0:0] m_axi_bid,
    input  [                     1:0] m_axi_bresp,
    input                             m_axi_bvalid,
    output                            m_axi_bready,
    output [                     0:0] m_axi_arid,
    output [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output [                     7:0] m_axi_arlen,
    output [                     2:0] m_axi_arsize,
    output [                     1:0] m_axi_arburst,
    output [                     3:0] m_axi_arcache,
    output [                     2:0] m_axi_arprot,
    output                            m_axi_arvalid,
    input                             m_axi_arready,
    input  [                     0:0] m_axi_rid,
    input  [                   511:0] m_axi_rdata,
    input  [                     1:0] m_axi_rresp,
    input                             m_axi_rlast,
    input                             m_axi_rvalid,
    output                            m_axi_rready,
    output                            m_axi_arlock,

    input [31:0] free_run_cnt
);
    localparam QP1 = 'd0;

    // Local Parameters
    // 传输包类型定义，与 lrdma_rx_req_hdr_validator 模块 对应
    localparam [3:0] REQ_PKT_TYPE_DISCARD = 4'd0;
    localparam [3:0] REQ_PKT_TYPE_ERROR = 4'd1;
    localparam [3:0] REQ_PKT_TYPE_MAD = 4'd2;
    localparam [3:0] REQ_DB_UPDATE = 4'd3;
    localparam [3:0] IN_ERR_STS = 4'd4;

    localparam IPv4 = 1'b0;
    localparam IPv6 = 1'b1;

    localparam AWCHNL_FIFO_DEPTH = 512;
    localparam AWCHNL_FIFO_DATA_WIDTH = C_QP_INDX_WIDTH + C_M_AXI_ADDR_WIDTH + 7 + 4 + 16;  //4 bits is for packet type 16 bits for req_pi_db
    localparam AWCHNL_DATA_COUNT_WIDTH = 10;  // log2(AWCHNL_FIFO_DEPTH) + 1

    localparam WCHNL_FIFO_DEPTH = 512;
    //Added one more bit for increasing REQ_PKT_TYPE to 4 bits
    localparam WCHNL_FIFO_DATA_WIDTH = C_REQ_BUF_ADDR_WIDTH + 66;  // 75
    localparam WCHNL_DATA_COUNT_WIDTH = 10;  // log2(WCHNL_FIFO_DEPTH) + 1  

    localparam POST_AXI_TNFR_FIFO_DEPTH = 512;
    localparam POST_AXI_TNFR_FIFO_DATA_WIDTH = 58;
    localparam POST_AXI_TNFR_DATA_COUNT_WIDTH = 10;  // log2(POST_AXI_TNFR_FIFO_DEPTH) + 1  

    wire [                             43:0] imm_data;
    wire [                              3:0] pkt_type;  // req_pkt_type
    wire                                     pkt_ip_ver;
    wire                                     req_tnfr_vld_to_axi;
    reg                                      mad_pkt_ip_ver;
    reg                                      current_mad_pkt_ip_ver;
    reg                                      set_first_cyc;
    reg                                      first_cyc;
    wire [              C_QP_INDX_WIDTH+3:0] qpid;  // {4bit pkt_type, C_QP_INDX_WIDTH-bit qpid}

    reg  [           C_M_AXI_ADDR_WIDTH-1:0] err_buf_ca;
    reg  [                             15:0] err_buf_wrptr;

    reg  [           C_M_AXI_ADDR_WIDTH-1:0] in_err_sts_q_ca;
    reg  [                             15:0] in_err_sts_q_wrptr;

    reg  [           C_M_AXI_ADDR_WIDTH-1:0] dest_addr;
    reg  [                             12:0] pyld_len;
    wire [                              6:0] awlen;
    wire [                              3:0] lshift_by;

    wire [       AWCHNL_FIFO_DATA_WIDTH-1:0] axi_awchnl_fifo_din;
    wire [       AWCHNL_FIFO_DATA_WIDTH-1:0] axi_awchnl_fifo_dout;
    wire                                     wr_axi_awchnl_fifo;
    wire                                     rd_axi_awchnl_fifo;
    wire                                     axi_awchnl_fifo_full;
    wire                                     axi_awchnl_fifo_empty;

    wire [                             15:0] fifo_out_req_rq_pi_db;
    wire [              C_QP_INDX_WIDTH-1:0] fifo_out_qpid;
    wire [                              3:0] fifo_out_pkt_type;
    wire [           C_M_AXI_ADDR_WIDTH-1:0] fifo_out_dest_addr;
    wire [                              6:0] fifo_out_awlen;

    wire                                     wr_axi_wchnl_fifo_1;

    wire [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_din_1;

    wire [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_din;
    wire [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_dout;
    wire                                     wr_axi_wchnl_fifo;
    reg                                      rd_axi_wchnl_fifo;
    wire                                     axi_wchnl_fifo_full;
    wire                                     axi_wchnl_fifo_empty;
    wire [                              3:0] wchnl_req_pkt_type;
    wire                                     wchnl_req_pkt_ip_ver;
    wire [         C_REQ_BUF_ADDR_WIDTH-1:0] wchnl_req_buf_addr_offset;
    wire [                             12:0] wchnl_pyld_len;
    wire [                             43:0] wchnl_imm_data;
    wire [                              3:0] wchnl_lshift_by;

    reg  [                              6:0] wcnt;
    reg  [                              6:0] nxt_wcnt;

    wire [                              6:0] wchnl_beats;

    reg                                      async_db_update;

    reg  [         C_REQ_BUF_ADDR_WIDTH-1:0] prev_req_pkt_buf_addrb;
    reg                                      req_pkt_buf_enb_dummy;
    reg                                      cpy_imm_data;
    reg                                      set_last_cyc;
    reg                                      last_cyc;
    reg                                      set_single_beat_only;
    reg                                      single_beat_only;
    reg  [                            511:0] prev_req_pkt_buf_doutb;  // req_pkt_buf_doutb上一个周期读出的数据

    wire [                              3:0] wchnl_req_pkt_type_to_data_mux;

    reg                                      m_axi_wvalid_i;
    reg  [                            511:0] m_axi_wdata_i;
    reg  [                             63:0] m_axi_wstrb_i;
    reg                                      m_axi_wlast_i;
    wire                                     m_axi_wready_i;

    wire                                     st0_exit_cond;  // 写数据状态机在 state 0 的“允许推进条件”标志
    wire                                     axi_mstr_idle;

    reg  [                              1:0] axi_wchnl_cs;
    reg  [                              1:0] axi_wchnl_ns;

    reg  [                              3:0] wchnl_req_pkt_type_r;
    reg  [                             43:0] wchnl_imm_data_r;

    wire                                     rd_post_axi_tnfr_fifo_1;
    wire                                     rd_post_axi_tnfr_fifo_2;

    wire [POST_AXI_TNFR_FIFO_DATA_WIDTH-1:0] post_axi_tnfr_fifo_din;
    wire [POST_AXI_TNFR_FIFO_DATA_WIDTH-1:0] post_axi_tnfr_fifo_dout;
    wire                                     wr_post_axi_tnfr_fifo;
    wire                                     rd_post_axi_tnfr_fifo;
    wire                                     post_axi_tnfr_fifo_full;
    wire                                     post_axi_tnfr_fifo_empty;

    wire [                              3:0] req_pkt_type_post_tnfr;
    wire [                             43:0] imm_data_post_tnfr;

    reg                                      incr_mad_pkt_cnt_1;
    reg                                      incr_in_err_sts_q_wrptr;

    reg                                      set_mad_pkt_rcvd_intr_1;

    reg                                      m_axi_bvalid_d;

    assign req_tnfr_vld_to_axi = req_tnfr_vld;

    // 准备条件为：没有响应传输有效 & AXI写地址通道的FIFO没满 & AXI写数据通道的FIFO没满
    assign rdy_to_req_tnfr     = (!rsp_tnfr_vld && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full) ? 1'b1 : 1'b0;

    assign rdy_to_rsp_tnfr     = (!axi_awchnl_fifo_full && !axi_wchnl_fifo_full) ? 1'b1 : 1'b0;

    assign wr_axi_awchnl_fifo  = (!axi_awchnl_fifo_full && !axi_wchnl_fifo_full && (rsp_tnfr_vld || (req_tnfr_vld_to_axi && (req_pkt_type != REQ_PKT_TYPE_DISCARD)))) ? 1'b1 : 1'b0;

    assign imm_data            = rsp_tnfr_vld ? {12'h000, rsp_imm_data} : req_imm_data;
    assign pkt_type            = rsp_tnfr_vld ? IN_ERR_STS : req_pkt_type;
    assign pkt_ip_ver          = rsp_tnfr_vld ? IPv6 : req_pkt_ip_ver;
    assign qpid                = rsp_tnfr_vld ? {IN_ERR_STS, {C_QP_INDX_WIDTH{1'b0}}} : {pkt_type, req_qpid};

    // ================ 环形队列写地址&写指针管理 ================
    // 错误缓冲区/队列
    always @(posedge clk)
        if (reset) begin
            err_buf_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            err_buf_wrptr <= 16'd0;
        end else if (lrdma_en_pulse) begin
            err_buf_ca    <= i_err_buf_ba;
            err_buf_wrptr <= 16'd0;
        end else if (req_tnfr_vld_to_axi && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full && (pkt_type == REQ_PKT_TYPE_ERROR)) begin
            if (err_buf_wrptr == i_err_num_bufs - 'd1) begin
                err_buf_ca    <= i_err_buf_ba;
                err_buf_wrptr <= 16'd0;
            end else begin
                err_buf_ca    <= err_buf_ca + i_err_buf_sz;
                err_buf_wrptr <= err_buf_wrptr + 'd1;
            end
        end

    // 传入数据包的错误状态缓冲区/队列
    always @(posedge clk)
        if (reset) begin
            in_err_sts_q_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            in_err_sts_q_wrptr <= 16'd0;
        end else if (lrdma_en_pulse) begin
            in_err_sts_q_ca    <= i_in_err_sts_q_ba;
            in_err_sts_q_wrptr <= 16'd0;
        end else if ((req_tnfr_vld_to_axi || rsp_tnfr_vld) && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full && (pkt_type == IN_ERR_STS)) begin
            if (in_err_sts_q_wrptr == i_in_err_sts_q_depth - 'd1) begin
                in_err_sts_q_ca    <= i_in_err_sts_q_ba;
                in_err_sts_q_wrptr <= 16'd0;
            end else begin
                in_err_sts_q_ca    <= in_err_sts_q_ca + i_in_err_sts_q_sz;
                in_err_sts_q_wrptr <= in_err_sts_q_wrptr + 'd1;
            end
        end
    // ================ 环形队列写指针管理 end ================

    // 对于这三类特殊包类型，DDR地址不取req_dest_addr
    always @(*) begin
        case (pkt_type)
            REQ_PKT_TYPE_ERROR: dest_addr = err_buf_ca;
            IN_ERR_STS: dest_addr = in_err_sts_q_ca;
            default: dest_addr = req_dest_addr;
        endcase
    end


    always @(*) begin
        case (pkt_type)
            REQ_PKT_TYPE_DISCARD: pyld_len = req_pkt_len;

            REQ_PKT_TYPE_ERROR: pyld_len = (i_err_buf_sz < (req_pkt_len + 'd4)) ? i_err_buf_sz : (req_pkt_len + 'd4);

            REQ_PKT_TYPE_MAD: begin
                if (pkt_ip_ver == IPv6) pyld_len = req_pkt_len - 'd18;
                else pyld_len = req_pkt_len - 'd18 + 'd20;
            end

            REQ_DB_UPDATE: pyld_len = 13'd1;

            IN_ERR_STS: pyld_len = 13'd1;

            default: pyld_len = 13'd0;
        endcase
    end

    assign awlen               = (pyld_len[5:0] == 6'd0) ? (pyld_len[12:6] - 7'd1) : pyld_len[12:6];  // 计算 AxiLen，表示一次突发传输中包含的数据传输数量，实际的长度值为 AxiLen + 1，为 0 表示突发传输 1 次
    assign lshift_by           = (pkt_type == IN_ERR_STS) ? in_err_sts_q_ca[5:2] : req_dest_addr[5:2];

    assign axi_awchnl_fifo_din = {req_rq_pi_db,  //16
 qpid,  // {4bit pkt_type, C_QP_INDX_WIDTH-bit qpid}
 dest_addr,  // C_M_AXI_ADDR_WIDTH
 awlen  // 7
};

    //==================================
    // AW FIFO
    // 地址元数据 FIFO { req_rq_pi_db, qpid, dest_addr, awlen }
    //==================================  
    lrdma_common_xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE("auto"),  //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE("no_ecc"),  //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH(AWCHNL_FIFO_DEPTH),  //positive integer									
        .WRITE_DATA_WIDTH(AWCHNL_FIFO_DATA_WIDTH),  //positive integer									
        .WR_DATA_COUNT_WIDTH(AWCHNL_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_FULL_THRESH(10),  //positive integer									
        .FULL_RESET_VALUE(0),  //positive integer; 0 or 1									
        .READ_MODE("fwft"),  //string; "std" or "fwft";									
        .FIFO_READ_LATENCY(0),  //positive integer;									
        .READ_DATA_WIDTH(AWCHNL_FIFO_DATA_WIDTH),  //positive integer									
        .RD_DATA_COUNT_WIDTH(AWCHNL_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_EMPTY_THRESH(10),  //positive integer									
        .DOUT_RESET_VALUE("0"),  //string									
        .WAKEUP_TIME(0)  //positive integer; 0 or 2;	  
    ) axi_awchnl_fifo (
        .sleep        (1'b0),
        .rst          (reset),
        .wr_clk       (clk),
        .wr_en        (wr_axi_awchnl_fifo),
        .din          (axi_awchnl_fifo_din),
        .full         (axi_awchnl_fifo_full),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (rd_axi_awchnl_fifo),
        .dout         (axi_awchnl_fifo_dout),
        .empty        (axi_awchnl_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   ()
    );
    //==================================    

    assign rd_axi_awchnl_fifo                                                                            = ~axi_awchnl_fifo_empty & m_axi_awready;

    // 地址元数据各字段
    assign {fifo_out_req_rq_pi_db, fifo_out_pkt_type, fifo_out_qpid, fifo_out_dest_addr, fifo_out_awlen} = axi_awchnl_fifo_dout;

    assign m_axi_awid                                                                                    = 1'b1;
    assign m_axi_awaddr                                                                                  = axi_awchnl_fifo_empty ? {(C_M_AXI_ADDR_WIDTH) {1'b0}} : axi_awchnl_fifo_dout[7+:C_M_AXI_ADDR_WIDTH];  // AW FIFO dest_addr 字段

    assign m_axi_awuser[31:16]                                                                           = fifo_out_req_rq_pi_db;  // AW FIFO req_rq_pi_db字段
    assign m_axi_awuser[15:12]                                                                           = fifo_out_pkt_type;  // pkt_type
    assign m_axi_awuser[11:0]                                                                            = {{(12 - C_QP_INDX_WIDTH) {1'b0}}, fifo_out_qpid[C_QP_INDX_WIDTH-1:0]};  // QPID
    assign m_axi_awlen                                                                                   = axi_awchnl_fifo_empty ? 8'b0 : {1'b0, axi_awchnl_fifo_dout[6:0]};  // AW FIFO awlen字段
    assign m_axi_awsize                                                                                  = 3'b110;
    assign m_axi_awburst                                                                                 = 2'b01;
    assign m_axi_awcache                                                                                 = 4'b0011;
    assign m_axi_awprot                                                                                  = 3'b000;
    assign m_axi_awvalid                                                                                 = ~axi_awchnl_fifo_empty;
    assign m_axi_awlock                                                                                  = 1'b0;

    // 当 AXI写地址通道的FIFO没满 & AXI写数据通道的FIFO没满 & (有响应传输有效 || 有请求传输有效)，使用fifo_1数据，反之使用fifo_2数据
    assign wr_axi_wchnl_fifo_1                                                                           = (!axi_awchnl_fifo_full && !axi_wchnl_fifo_full && (rsp_tnfr_vld || req_tnfr_vld_to_axi)) ? 1'b1 : 1'b0;

    // { pkt_ip_ver, pkt_type, req_buf_addr_offset, pyld_len, imm_data, lshift_by }
    assign axi_wchnl_fifo_din_1                                                                          = {pkt_ip_ver,  // 1
 pkt_type,  // 4
 req_buf_addr_offset,  // C_REQ_BUF_ADDR_WIDTH
 pyld_len,  // 13
 imm_data,  // 44
 lshift_by  // 4
};

    assign axi_wchnl_fifo_din                                                                            = axi_wchnl_fifo_din_1;
    assign wr_axi_wchnl_fifo                                                                             = wr_axi_wchnl_fifo_1;

    //=================================
    // W FIFO
    // 写数据元数据通道 FIFO
    //=================================
    lrdma_common_xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE("auto"),  //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE("no_ecc"),  //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH(WCHNL_FIFO_DEPTH),  //positive integer									
        .WRITE_DATA_WIDTH(WCHNL_FIFO_DATA_WIDTH),  //positive integer									
        .WR_DATA_COUNT_WIDTH(WCHNL_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_FULL_THRESH(10),  //positive integer									
        .FULL_RESET_VALUE(0),  //positive integer; 0 or 1									
        .READ_MODE("fwft"),  //string; "std" or "fwft";									
        .FIFO_READ_LATENCY(0),  //positive integer;									
        .READ_DATA_WIDTH(WCHNL_FIFO_DATA_WIDTH),  //positive integer									
        .RD_DATA_COUNT_WIDTH(WCHNL_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_EMPTY_THRESH(10),  //positive integer									
        .DOUT_RESET_VALUE("0"),  //string									
        .WAKEUP_TIME(0)  //positive integer; 0 or 2;	  
    ) axi_wchnl_fifo (
        .sleep        (1'b0),
        .rst          (reset),
        .wr_clk       (clk),
        .wr_en        (wr_axi_wchnl_fifo),
        .din          (axi_wchnl_fifo_din),
        .full         (axi_wchnl_fifo_full),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (rd_axi_wchnl_fifo),
        .dout         (axi_wchnl_fifo_dout),
        .empty        (axi_wchnl_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   ()
    );
    //================================= 

    assign wchnl_req_pkt_ip_ver      = axi_wchnl_fifo_dout[(C_REQ_BUF_ADDR_WIDTH+65)+:1];
    assign wchnl_req_pkt_type        = axi_wchnl_fifo_dout[(C_REQ_BUF_ADDR_WIDTH+61)+:4];
    assign wchnl_req_buf_addr_offset = axi_wchnl_fifo_dout[61+:C_REQ_BUF_ADDR_WIDTH];  // req_buf_addr_offset
    assign wchnl_pyld_len            = axi_wchnl_fifo_dout[48+:13];
    assign wchnl_imm_data            = axi_wchnl_fifo_dout[4+:44];
    assign wchnl_lshift_by           = axi_wchnl_fifo_dout[0+:4];

    // ~axi_wchnl_fifo_empty：W 元数据 FIFO 里必须有待处理项，否则 state0 不该动。
    // ~post_axi_tnfr_fifo_full：后级 post FIFO 不能满；因为本次事务完成后要往里写完成信息，满了就先背压
    // (!last_cyc | m_axi_wready_i)：如果不是“尾拍待发送”就可推进；如果是尾拍，则必须等 WREADY 握手成功，防止上一包最后一拍还没被接收就读下一条元数据。
    assign st0_exit_cond             = ~axi_wchnl_fifo_empty & ~post_axi_tnfr_fifo_full & (!last_cyc | m_axi_wready_i);
    assign axi_mstr_idle             = axi_wchnl_fifo_empty & ~m_axi_wvalid_i;

    assign wchnl_beats               = |wchnl_pyld_len[5:0] ? wchnl_pyld_len[12:6] : (wchnl_pyld_len[12:6] - 'd1);


    always @(*) begin
        nxt_wcnt              = wcnt;
        req_pkt_buf_addrb     = prev_req_pkt_buf_addrb;
        req_pkt_buf_enb       = 1'b0;
        req_pkt_buf_enb_dummy = 1'b0;
        rd_axi_wchnl_fifo     = 1'b0;
        cpy_imm_data          = 1'b0;
        set_last_cyc          = 1'b0;
        set_first_cyc         = 1'b0;
        set_single_beat_only  = 1'b0;
        m_axi_wvalid_i        = 1'b0;
        m_axi_wlast_i         = 1'b0;
        m_axi_wstrb_i         = {64{1'b0}};
        axi_wchnl_ns          = axi_wchnl_cs;
        async_db_update       = 1'b0;
        mad_pkt_ip_ver        = current_mad_pkt_ip_ver;

        case (axi_wchnl_cs)
            2'd0: begin
                if (last_cyc) begin
                    m_axi_wvalid_i = 1'b1;
                    m_axi_wstrb_i  = {64{1'b1}};
                    m_axi_wlast_i  = 1'b1;
                end

                if (st0_exit_cond) begin
                    case (wchnl_req_pkt_type)
                        REQ_PKT_TYPE_DISCARD: begin
                            if (!last_cyc) begin
                                rd_axi_wchnl_fifo = 1'b1;
                                async_db_update   = 1'b1;
                            end
                        end

                        REQ_PKT_TYPE_ERROR: begin
                            req_pkt_buf_addrb = wchnl_req_buf_addr_offset;
                            req_pkt_buf_enb   = 1'b1;
                            cpy_imm_data      = 1'b1;

                            if (wchnl_pyld_len <= 13'd64) begin
                                nxt_wcnt          = 7'd0;
                                set_last_cyc      = 1'b1;
                                rd_axi_wchnl_fifo = 1'b1;
                            end else begin
                                nxt_wcnt     = wchnl_beats;
                                axi_wchnl_ns = 2'd2;
                            end
                        end

                        REQ_PKT_TYPE_MAD: begin
                            req_pkt_buf_addrb = wchnl_req_buf_addr_offset;
                            req_pkt_buf_enb   = 1'b1;
                            mad_pkt_ip_ver    = wchnl_req_pkt_ip_ver;
                            set_first_cyc     = 1'b1;

                            if (wchnl_pyld_len <= 13'd50) begin
                                nxt_wcnt          = 7'd0;
                                set_last_cyc      = 1'b1;
                                rd_axi_wchnl_fifo = 1'b1;
                            end else begin
                                nxt_wcnt = wchnl_beats;
                                if (wchnl_req_pkt_ip_ver == IPv6) axi_wchnl_ns = 2'd1;
                                else axi_wchnl_ns = 2'd2;
                            end
                        end

                        REQ_DB_UPDATE: begin
                            nxt_wcnt = 7'd0;
                            if (!last_cyc) begin
                                m_axi_wvalid_i = 1'b1;
                                m_axi_wstrb_i  = {{62{1'b0}}, {2{1'b1}}} << {wchnl_lshift_by, {2{1'b0}}};
                                m_axi_wlast_i  = 1'b1;
                                if (m_axi_wready_i) rd_axi_wchnl_fifo = 1'b1;
                            end
                        end

                        IN_ERR_STS: begin
                            nxt_wcnt = 7'd0;
                            if (!last_cyc) begin
                                m_axi_wvalid_i = 1'b1;
                                m_axi_wstrb_i  = {{60{1'b0}}, {4{1'b1}}} << {wchnl_lshift_by, {2{1'b0}}};
                                m_axi_wlast_i  = 1'b1;
                                if (m_axi_wready_i) rd_axi_wchnl_fifo = 1'b1;
                            end
                        end

                        default: begin
                            if (!last_cyc) rd_axi_wchnl_fifo = 1'b1;
                        end
                    endcase
                end
            end

            2'd1: begin
                req_pkt_buf_addrb = prev_req_pkt_buf_addrb + 'd1;
                req_pkt_buf_enb   = 1'b1;

                if (wcnt == 7'd0) begin
                    set_last_cyc      = 1'b1;
                    rd_axi_wchnl_fifo = 1'b1;
                    axi_wchnl_ns      = 2'd0;
                end else begin
                    axi_wchnl_ns = 2'd2;
                end
            end

            2'd2: begin
                m_axi_wvalid_i = 1'b1;
                m_axi_wstrb_i  = {64{1'b1}};
                set_first_cyc  = 1'b0;

                if (m_axi_wready_i) begin
                    nxt_wcnt          = wcnt - 'd1;
                    req_pkt_buf_addrb = prev_req_pkt_buf_addrb + 'd1;
                    req_pkt_buf_enb   = 1'b1;

                    if (wcnt == 7'd1) begin
                        set_last_cyc      = 1'b1;
                        rd_axi_wchnl_fifo = 1'b1;
                        axi_wchnl_ns      = 2'd0;
                    end
                end
            end

            default: begin
                axi_wchnl_ns = 2'd0;
            end
        endcase
    end

    always @(posedge clk)
        if (reset) axi_wchnl_cs <= 2'b00;
        else axi_wchnl_cs <= axi_wchnl_ns;

    // last_cyc / set_last_cyc 标记“该包是否进入最后一个 beat 的发送阶段”
    always @(posedge clk)
        if (reset) last_cyc <= 1'b0;
        else if (set_last_cyc || m_axi_wready_i) last_cyc <= set_last_cyc;

    // first_cyc / set_first_cyc 标记“是否是该包首拍”，主要给特殊数据拼接用(MAD 包)
    always @(posedge clk)
        if (reset) first_cyc <= 1'b0;
        else if (set_first_cyc || m_axi_wready_i) first_cyc <= set_first_cyc;

    always @(posedge clk)
        if (reset) current_mad_pkt_ip_ver <= IPv6;
        else current_mad_pkt_ip_ver <= mad_pkt_ip_ver;

    always @(posedge clk) begin
        if (reset) begin
            single_beat_only <= 1'b0;
        end else begin
            if (set_single_beat_only || m_axi_wready_i) single_beat_only <= set_single_beat_only;
        end
    end

    always @(posedge clk) wcnt <= nxt_wcnt;

    always @(posedge clk)
        if (req_pkt_buf_enb) prev_req_pkt_buf_addrb <= req_pkt_buf_addrb;
        else if (axi_mstr_idle) prev_req_pkt_buf_addrb <= req_pkt_buf_addra;

    always @(posedge clk)
        if (cpy_imm_data) prev_req_pkt_buf_doutb <= {wchnl_imm_data[31:0], req_pkt_buf_doutb[479:0]};  // 对错误状态写入类型，将 imm_data 放入首拍低地址位置，并拼接原缓冲区剩余 payload 数据
        else if (req_pkt_buf_enb) prev_req_pkt_buf_doutb <= req_pkt_buf_doutb;

    assign wchnl_req_pkt_type_to_data_mux = last_cyc ? wchnl_req_pkt_type_r : wchnl_req_pkt_type;

    always @(*) begin
        m_axi_wdata_i = 512'd0;

        if (m_axi_wvalid_i) begin
            case (wchnl_req_pkt_type_to_data_mux)
                REQ_PKT_TYPE_ERROR: m_axi_wdata_i = {req_pkt_buf_doutb[60*8-1:0], prev_req_pkt_buf_doutb[64*8-1:60*8]};

                REQ_PKT_TYPE_MAD: begin
                    if (mad_pkt_ip_ver == IPv6) begin
                        if (single_beat_only) m_axi_wdata_i[50*8-1:0] = req_pkt_buf_doutb[64*8-1:14*8];
                        else m_axi_wdata_i = {req_pkt_buf_doutb[14*8-1:0], prev_req_pkt_buf_doutb[64*8-1:14*8]};
                    end else begin
                        if (single_beat_only) m_axi_wdata_i[6*8-1:0] = req_pkt_buf_doutb[64*8-1:58*8];
                        else if (first_cyc) m_axi_wdata_i = {req_pkt_buf_doutb[58*8-1:14*8], {20 * 8{1'b0}}};
                        else m_axi_wdata_i = {req_pkt_buf_doutb[58*8-1:0], prev_req_pkt_buf_doutb[64*8-1:58*8]};
                    end
                end

                REQ_DB_UPDATE: m_axi_wdata_i = {{496{1'b0}}, wchnl_imm_data[15:0]} << {wchnl_lshift_by, {5{1'b0}}};

                IN_ERR_STS: m_axi_wdata_i = {{480{1'b0}}, wchnl_imm_data[31:0]} << {wchnl_lshift_by, {5{1'b0}}};

                default: m_axi_wdata_i = 512'd0;
            endcase
        end
    end

    lrdma_common_reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_inst (
        .aclk          (clk),
        .aclken        (1'b1),
        .areset        (reset),
        .S_PAYLOAD_DATA({m_axi_wdata_i, m_axi_wstrb_i, m_axi_wlast_i}),
        .S_VALID       (m_axi_wvalid_i),
        .S_READY       (m_axi_wready_i),
        .M_PAYLOAD_DATA({m_axi_wdata, m_axi_wstrb, m_axi_wlast}),
        .M_VALID       (m_axi_wvalid),
        .M_READY       (m_axi_wready)
    );

    assign m_axi_arid    = 1'b1;
    assign m_axi_araddr  = {(C_M_AXI_ADDR_WIDTH) {1'b0}};
    assign m_axi_arlen   = 8'h00;
    assign m_axi_arsize  = 3'b110;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arvalid = 1'b0;
    assign m_axi_rready  = 1'b1;
    assign m_axi_arlock  = 1'b0;

    always @(posedge clk)
        if (reset) begin
            wchnl_req_pkt_type_r <= 4'h0;
            wchnl_imm_data_r     <= 44'd00;
        end else begin
            if (rd_axi_wchnl_fifo) begin
                wchnl_req_pkt_type_r <= wchnl_req_pkt_type;
                wchnl_imm_data_r     <= wchnl_imm_data;
            end
        end

    assign post_axi_tnfr_fifo_din = last_cyc ? {free_run_cnt[9:0],  // 10
 wchnl_req_pkt_type_r,  // 4
 wchnl_imm_data_r  // 44
} : {free_run_cnt[9:0],  // 10
 wchnl_req_pkt_type,  // 4 
 wchnl_imm_data  // 44      
};

    assign wr_post_axi_tnfr_fifo  = (m_axi_wlast_i & m_axi_wready_i) | async_db_update;

    // =====================================================================
    // POST_AXI_TNFR_FIFO (FWFT)
    // post_axi_tnfr_fifo 是“写完成事件队列”，确保只有在写事务真正完成后，才执行门铃/统计/中断这些副作用逻辑
    // =====================================================================


    lrdma_common_xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE("auto"),  //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE("no_ecc"),  //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH(POST_AXI_TNFR_FIFO_DEPTH),  //positive integer									
        .WRITE_DATA_WIDTH(POST_AXI_TNFR_FIFO_DATA_WIDTH),  //positive integer									
        .WR_DATA_COUNT_WIDTH(POST_AXI_TNFR_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_FULL_THRESH(POST_AXI_TNFR_FIFO_DEPTH - 8),  //positive integer									
        .FULL_RESET_VALUE(0),  //positive integer; 0 or 1									
        .READ_MODE("fwft"),  //string; "std" or "fwft";									
        .FIFO_READ_LATENCY(0),  //positive integer;									
        .READ_DATA_WIDTH(POST_AXI_TNFR_FIFO_DATA_WIDTH),  //positive integer									
        .RD_DATA_COUNT_WIDTH(POST_AXI_TNFR_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_EMPTY_THRESH(10),  //positive integer									
        .DOUT_RESET_VALUE("0"),  //string									
        .WAKEUP_TIME(0)  //positive integer; 0 or 2;	  
    ) post_axi_tnfr_fifo (
        .sleep        (1'b0),
        .rst          (reset),
        .wr_clk       (clk),
        .wr_en        (wr_post_axi_tnfr_fifo),
        .din          (post_axi_tnfr_fifo_din),
        .full         (),
        .prog_full    (post_axi_tnfr_fifo_full),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (rd_post_axi_tnfr_fifo),
        .dout         (post_axi_tnfr_fifo_dout),
        .empty        (post_axi_tnfr_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      (),
        .almost_full  (),
        .wr_ack       (),
        .almost_empty (),
        .data_valid   ()
    );

    // =====================================================================  

    assign req_pkt_type_post_tnfr = post_axi_tnfr_fifo_dout[47:44];
    assign imm_data_post_tnfr     = post_axi_tnfr_fifo_dout[43:0];

    always @(posedge clk)
        if (reset) begin
            m_axi_bvalid_d <= 1'b0;
        end else begin
            m_axi_bvalid_d <= m_axi_bvalid && m_axi_bready;
        end

    assign rd_post_axi_tnfr_fifo_1 = (!post_axi_tnfr_fifo_empty && (req_pkt_type_post_tnfr != REQ_PKT_TYPE_DISCARD) && m_axi_bvalid_d) ? 1'b1 : 1'b0;
    assign rd_post_axi_tnfr_fifo_2 = (!post_axi_tnfr_fifo_empty && (req_pkt_type_post_tnfr == REQ_PKT_TYPE_DISCARD)) ? 1'b1 : 1'b0;
    assign rd_post_axi_tnfr_fifo   = rd_post_axi_tnfr_fifo_1 | rd_post_axi_tnfr_fifo_2;

    assign m_axi_bready            = (!post_axi_tnfr_fifo_empty && (req_pkt_type_post_tnfr == REQ_PKT_TYPE_DISCARD)) ? 1'b0 : 1'b1;

    always @(*) begin
        incr_inv_pkt_cnt        = 1'b0;
        set_pkt_valdn_err_intr  = 1'b0;
        incr_mad_pkt_cnt_1      = 1'b0;
        set_mad_pkt_rcvd_intr_1 = 1'b0;
        set_qp_fatal_intr       = 1'b0;
        incr_in_err_sts_q_wrptr = 1'b0;

        if (rd_post_axi_tnfr_fifo) begin
            case (req_pkt_type_post_tnfr)
                REQ_PKT_TYPE_DISCARD: begin
                end

                REQ_PKT_TYPE_ERROR: begin
                    incr_inv_pkt_cnt       = 1'b1;
                    set_pkt_valdn_err_intr = 1'b1;
                end

                REQ_PKT_TYPE_MAD: begin
                end

                REQ_DB_UPDATE: begin
                    if (imm_data_post_tnfr[16+:C_QP_INDX_WIDTH] == QP1) begin
                        incr_mad_pkt_cnt_1      = 1'b1;
                        set_mad_pkt_rcvd_intr_1 = 1'b1;
                    end
                end

                IN_ERR_STS: begin
                    set_qp_fatal_intr       = 1'b1;
                    incr_in_err_sts_q_wrptr = 1'b1;
                end

                default: begin
                end
            endcase
        end
    end

    always @(posedge clk)
        if (reset) begin
            o_err_buf_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            o_err_buf_wrptr <= 16'd0;
        end else if (lrdma_en_pulse) begin
            o_err_buf_ca    <= i_err_buf_ba;
            o_err_buf_wrptr <= 16'd0;
        end else if (incr_inv_pkt_cnt) begin
            if (o_err_buf_wrptr == i_err_num_bufs - 'd1) begin
                o_err_buf_ca    <= i_err_buf_ba;
                o_err_buf_wrptr <= 16'd0;
            end else begin
                o_err_buf_ca    <= o_err_buf_ca + i_err_buf_sz;
                o_err_buf_wrptr <= o_err_buf_wrptr + 'd1;
            end
        end

    always @(posedge clk)
        if (reset) begin
            o_in_err_sts_q_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            o_in_err_sts_q_wrptr <= 16'd0;
        end else if (lrdma_en_pulse) begin
            o_in_err_sts_q_ca    <= i_in_err_sts_q_ba;
            o_in_err_sts_q_wrptr <= 16'd0;
        end else if (incr_in_err_sts_q_wrptr) begin
            if (o_in_err_sts_q_wrptr == i_in_err_sts_q_depth - 'd1) begin
                o_in_err_sts_q_ca    <= i_in_err_sts_q_ba;
                o_in_err_sts_q_wrptr <= 16'd0;
            end else begin
                o_in_err_sts_q_ca    <= o_in_err_sts_q_ca + i_in_err_sts_q_sz;
                o_in_err_sts_q_wrptr <= o_in_err_sts_q_wrptr + 'd1;
            end
        end

    always @(posedge clk)
        if (reset) o_mad_pkt_cnt <= 16'd0;
        else if (incr_mad_pkt_cnt_1) o_mad_pkt_cnt <= o_mad_pkt_cnt + 'd1;

    assign set_mad_pkt_rcvd_intr = set_mad_pkt_rcvd_intr_1;

endmodule
