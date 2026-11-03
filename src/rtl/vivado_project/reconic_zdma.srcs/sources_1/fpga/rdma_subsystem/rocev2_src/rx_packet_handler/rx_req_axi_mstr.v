//-----------------------------------------------------------------------------
// rx_req_axi_mstr.v
//-----------------------------------------------------------------------------
//
// Filename        : rx_req_axi_mstr.v
// Version         : v1.0
// Description     : Writes the incoming request packets to system memory：将收到的请求数据包写入系统内存
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
// rx_req_axi_mstr 模块把接收到的请求包/响应包 元数据 存放进 FIFO，按包类型计算目标写地址和有效长度，
// 通过 AXI 写通道把 包 或 门铃/错误信息 写入 DDR，处理 IETH/IMMDT 侧带数据，
// 并在写完成后根据 req_imm_data 更新门铃 FIFO、统计计数和触发相关中断。
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module rx_req_axi_mstr #(
    parameter C_SIM_DEBUG = 0,
    parameter C_EN_INITIATOR_LITE = 0,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_M_AXI_AWUSER_WIDTH = 16,
    parameter C_QP_INDX_WIDTH = 8,
    parameter C_REQ_BUF_ADDR_WIDTH = 9,
    parameter C_NUM_QP = 32,
    parameter C_EN_IETH_IMMDT = 1
) (
    // System level interface
    input wire clk,
    input wire reset,

    // 通过这些接口将接收到的请求包元数据存放进 FIFO 中，以下就是请求包元数据
    // req_pkt_hdr_val 接口
    input  wire [                    31:0] req_pkt_id,
    input  wire [                     3:0] req_pkt_type,
    input  wire                            req_pkt_ip_ver,
    input  wire [                    15:0] req_rq_pi_db,
    input  wire [     C_QP_INDX_WIDTH-1:0] req_qpid,
    input  wire [  C_M_AXI_ADDR_WIDTH-1:0] req_dest_addr,
    input  wire [C_REQ_BUF_ADDR_WIDTH-1:0] req_buf_addr_offset,      // cmac_rx_intf pkt_buf_offset
    input  wire [                    12:0] req_pkt_len,
    // req_imm_data字段：
    // 发生错误时：对应手册 Packet Validation Error Syndrome 或者 Error Status Format
    // 正常包时：{1bit 硬件握手标志, 1bit MAD标志, 1bit RQ中断使能, 14bit RQ门铃地址, (11-C_QP_INDX_WIDTH) bit 保留字段, C_QP_INDX_WIDTH位 QP号, 16bit 门铃值}
    input  wire [                    43:0] req_imm_data,
    input  wire                            req_tnfr_vld,
    input  wire                            req_immdt_present,        // Immdt头部标志
    input  wire                            req_ieth_present,
    input  wire [                    63:0] ieth_immdt_fifo_wr_data,
    input  wire                            ieth_immdt_fifo_wr_en,
    output wire                            rdy_to_req_tnfr,          // ready to request transfer

    // 通过这些接口将接收到的响应包元数据存放进 FIFO 中，以下就是响应包元数据
    // rsp_pkt_hdr_val 接口
    input  wire [31:0] rsp_pkt_id,
    input  wire [31:0] rsp_imm_data,    // rsp_imm_data字段，与 rx_rsp_hdr_val模块 配合
    input  wire        rsp_tnfr_vld,
    output wire        rdy_to_rsp_tnfr, // ready to response transfer

    // XRNIC register interface
    // XRNIC 寄存器接口
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_bypass_buf_ba,
    input  wire [                  15:0] i_bypass_buf_sz,
    input  wire [                  15:0] i_bypass_num_bufs,
    output reg  [C_M_AXI_ADDR_WIDTH-1:0] o_bypass_buf_ca,
    output reg  [                  15:0] o_bypass_buf_wrptr,
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_err_buf_ba,
    input  wire [                  15:0] i_err_buf_sz,
    input  wire [                  15:0] i_err_num_bufs,
    output reg  [C_M_AXI_ADDR_WIDTH-1:0] o_err_buf_ca,
    output reg  [                  15:0] o_err_buf_wrptr,
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_in_err_sts_q_ba,
    input  wire [                  15:0] i_in_err_sts_q_sz,
    input  wire [                  15:0] i_in_err_sts_q_depth,
    output reg  [C_M_AXI_ADDR_WIDTH-1:0] o_in_err_sts_q_ca,
    output reg  [                  15:0] o_in_err_sts_q_wrptr,
    output reg  [                  15:0] o_bypass_pkt_cnt,
    output reg                           incr_inv_pkt_cnt,
    output reg  [                  15:0] o_mad_pkt_cnt,
    output reg  [                  15:0] o_send_req_cnt,
    output reg  [                  15:0] o_send_pkt_cnt,

    input wire xrnic_en_pulse,

    // 至中断控制器
    output reg                        set_bypass_pkt_rcvd_intr,
    output reg                        set_pkt_valdn_err_intr,
    output wire                       set_mad_pkt_rcvd_intr,
    output reg                        set_qp_req_rcvd_intr_1,
    output wire [C_QP_INDX_WIDTH-1:0] intr_qp_indx_1,
    output wire                       set_qp_req_rcvd_intr_2,
    output wire [C_QP_INDX_WIDTH-1:0] intr_qp_indx_2,
    output reg                        set_qp_fatal_intr,

    // 请求包缓冲区接口，有包头
    input  wire [C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addra,
    output reg                             req_pkt_buf_enb,
    output reg  [C_REQ_BUF_ADDR_WIDTH-1:0] req_pkt_buf_addrb,
    input  wire [                   511:0] req_pkt_buf_doutb,

    // Sideband RQ doorbell interface
    // 侧带 RQ 门铃接口
    output reg         rq_db_data_valid,
    output wire [12:0] rq_db_addr,
    output wire [15:0] rq_db_data,
    input  wire        rq_db_data_ack,    // rdy

    // AXI Master interface to DDR
    // 仅支持 AXI4 写通道（写地址&写数据）和 响应通道
    // tip: 写的都是有效载荷，不包含任何协议头
    output wire [                     0:0] m_axi_awid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [C_M_AXI_AWUSER_WIDTH-1:0] m_axi_awuser,
    output wire [                     7:0] m_axi_awlen,
    output wire [                     2:0] m_axi_awsize,
    output wire [                     1:0] m_axi_awburst,
    output wire [                     3:0] m_axi_awcache,
    output wire [                     2:0] m_axi_awprot,
    output wire                            m_axi_awvalid,
    input  wire                            m_axi_awready,
    output wire [                   511:0] m_axi_wdata,
    output wire [                    63:0] m_axi_wstrb,
    output wire                            m_axi_wlast,
    output wire                            m_axi_wvalid,
    input  wire                            m_axi_wready,
    output wire                            m_axi_awlock,
    input  wire [                     0:0] m_axi_bid,
    input  wire [                     1:0] m_axi_bresp,
    input  wire                            m_axi_bvalid,
    output wire                            m_axi_bready,
    output wire [                     0:0] m_axi_arid,
    output wire [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [                     7:0] m_axi_arlen,
    output wire [                     2:0] m_axi_arsize,
    output wire [                     1:0] m_axi_arburst,
    output wire [                     3:0] m_axi_arcache,
    output wire [                     2:0] m_axi_arprot,
    output wire                            m_axi_arvalid,
    input  wire                            m_axi_arready,
    input  wire [                     0:0] m_axi_rid,
    input  wire [                   511:0] m_axi_rdata,
    input  wire [                     1:0] m_axi_rresp,
    input  wire                            m_axi_rlast,
    input  wire                            m_axi_rvalid,
    output wire                            m_axi_rready,
    output wire                            m_axi_arlock,

    // Asynchronous DB update command during connect IO
    // 连接 IO 期间异步 DB 更新命令
    input  wire [               15:0] i_rq_pi_db_hw_hndshk,           // RQ PI 门铃
    input  wire [               13:0] i_connect_io_qp_rq_pi_db_wptr,  // RQ PI 门铃地址
    input  wire [C_QP_INDX_WIDTH-1:0] i_connect_io_qpid,              // RQ PI 门铃 qpid
    input  wire                       i_hw_hndshk_disable_to_0,       // pulse indicates HWHNDSHK_DIS set 0，已设置为0
    output wire                       o_connect_io_qp_rq_pi_db_rdy,   // Ready indicates can accept another

    input wire rq_wrptr_db_we,

    // Debug signals
    input wire [31:0] free_run_cnt,

    input  wire        i_clr_debug_sts,
    output reg  [15:0] axi_awchnl_fifo_unoccupied_r,
    output reg  [15:0] axi_awchnl_max_wait_st,
    output reg  [15:0] axi_wchnl_fifo_unoccupied_r,
    output reg  [15:0] axi_wchnl_max_wait_st,
    output reg  [15:0] post_axi_tnfr_fifo_unoccupied_r,
    output reg  [15:0] db_fifo_unoccupied_r,
    output reg  [15:0] axi_bresp_max_wait_st,
    output wire        axi_awchnl_fifo_empty,
    output reg         axi_awchnl_fifo_full_sticky,
    output wire        axi_wchnl_fifo_empty,
    output reg         axi_wchnl_fifo_full_sticky,
    output wire        post_axi_tnfr_fifo_empty,
    output reg         post_axi_tnfr_fifo_full_sticky,
    output wire        db_fifo_empty,
    output reg         db_fifo_full_sticky,

    // IETH or IMMDT axi streaming interface
    output wire        ieth_immdt_axis_tvalid,
    output wire        ieth_immdt_axis_tlast,
    output wire [63:0] ieth_immdt_axis_tdata,
    input  wire        ieth_immdt_axis_trdy
);

    // Local Parameters
    // 传输包类型定义，与 rx_req_hdr_val 模块 对应
    localparam [3:0] REQ_PKT_TYPE_DISCARD = 4'd0;
    localparam [3:0] REQ_PKT_TYPE_ERROR = 4'd1;
    localparam [3:0] REQ_PKT_TYPE_BYPASS = 4'd2;
    localparam [3:0] REQ_PKT_TYPE_MAD = 4'd3;
    localparam [3:0] REQ_PKT_TYPE_IPV4_SEND = 4'd4;
    localparam [3:0] REQ_PKT_TYPE_IPV6_SEND = 4'd5;
    localparam [3:0] REQ_DB_UPDATE = 4'd6;
    localparam [3:0] IN_ERR_STS = 4'd7;
    localparam [3:0] REQ_PKT_TYPE_IPV4_WR_REQ = 4'd12;
    localparam [3:0] REQ_PKT_TYPE_IPV6_WR_REQ = 4'd13;
    localparam [3:0] REQ_PKT_TYPE_IPV4_WR_REQ_RETH = 4'd14;
    localparam [3:0] REQ_PKT_TYPE_IPV6_WR_REQ_RETH = 4'd15;

    localparam IPv4 = 1'b0;
    localparam IPv6 = 1'b1;

    localparam AWCHNL_FIFO_DEPTH = 512;
    localparam AWCHNL_FIFO_DATA_WIDTH = C_SIM_DEBUG ? (C_QP_INDX_WIDTH + C_M_AXI_ADDR_WIDTH + 39 + 4 + 16) : (C_QP_INDX_WIDTH + C_M_AXI_ADDR_WIDTH + 7 + 4 + 16);  //4 bits is for packet type 16 bits for req_pi_db
    localparam AWCHNL_DATA_COUNT_WIDTH = 10;  // log2(AWCHNL_FIFO_DEPTH) + 1

    localparam WCHNL_FIFO_DEPTH = 512;
    //Added one more bit for increasing REQ_PKT_TYPE to 4 bits
    //localparam WCHNL_FIFO_DATA_WIDTH    = C_SIM_DEBUG ? (C_EN_IETH_IMMDT ? (C_REQ_BUF_ADDR_WIDTH + 96) : (C_REQ_BUF_ADDR_WIDTH + 94)) : (C_EN_IETH_IMMDT ? (C_REQ_BUF_ADDR_WIDTH + 64) : (C_REQ_BUF_ADDR_WIDTH + 62));
    localparam WCHNL_FIFO_DATA_WIDTH = C_SIM_DEBUG ? (C_EN_IETH_IMMDT ? (C_REQ_BUF_ADDR_WIDTH + 100) : (C_REQ_BUF_ADDR_WIDTH + 98)) : (C_EN_IETH_IMMDT ? (C_REQ_BUF_ADDR_WIDTH + 68) : (C_REQ_BUF_ADDR_WIDTH + 66));
    localparam WCHNL_DATA_COUNT_WIDTH = 10;  // log2(WCHNL_FIFO_DEPTH) + 1  

    localparam POST_AXI_TNFR_FIFO_DEPTH = 512;
    localparam POST_AXI_TNFR_FIFO_DATA_WIDTH = C_SIM_DEBUG ? 90 : 58;
    localparam POST_AXI_TNFR_DATA_COUNT_WIDTH = 10;  // log2(POST_AXI_TNFR_FIFO_DEPTH) + 1  

    localparam DB_FIFO_DEPTH = 8;
    localparam DB_FIFO_DATA_WIDTH = C_SIM_DEBUG ? (C_QP_INDX_WIDTH + 64) : (C_QP_INDX_WIDTH + 32);
    localparam DB_FIFO_DATA_COUNT_WIDTH = 4;  // log2(DB_FIFO_DEPTH) + 1    

    integer                                     i;

    wire    [                             31:0] pkt_id;
    wire    [                             43:0] imm_data;
    wire    [                              3:0] pkt_type;  // req_pkt_type
    wire                                        pkt_ip_ver;
    wire                                        immdt_present;
    wire                                        ieth_present;
    reg                                         mad_pkt_ip_ver;
    reg                                         current_mad_pkt_ip_ver;
    reg                                         set_first_cyc;
    reg                                         first_cyc;
    wire    [              C_QP_INDX_WIDTH+3:0] qpid;  //{4 bitpkt_type + QPID}

    reg     [           C_M_AXI_ADDR_WIDTH-1:0] err_buf_ca;
    reg     [                             15:0] err_buf_wrptr;

    reg     [           C_M_AXI_ADDR_WIDTH-1:0] bypass_buf_ca;
    reg     [                             15:0] bypass_buf_wrptr;

    reg     [           C_M_AXI_ADDR_WIDTH-1:0] in_err_sts_q_ca;
    reg     [                             15:0] in_err_sts_q_wrptr;

    reg     [           C_M_AXI_ADDR_WIDTH-1:0] dest_addr;
    reg     [                             12:0] pyld_len;
    wire    [                              6:0] awlen;
    wire    [                              3:0] lshift_by;

    wire    [       AWCHNL_FIFO_DATA_WIDTH-1:0] axi_awchnl_fifo_din;
    wire    [       AWCHNL_FIFO_DATA_WIDTH-1:0] axi_awchnl_fifo_dout;
    wire                                        wr_axi_awchnl_fifo;
    wire                                        rd_axi_awchnl_fifo;
    wire                                        axi_awchnl_fifo_full;

    // Separate BRAM FWFT outputs from AXI address decode/arbitration.
    wire    [       AWCHNL_FIFO_DATA_WIDTH-1:0] axi_awchnl_fifo_raw_dout;
    wire                                        axi_awchnl_fifo_raw_empty;
    wire                                        rd_axi_awchnl_fifo_raw;
    wire                                        axi_awchnl_desc_ready;
    wire                                        axi_awchnl_desc_valid;

    wire    [                             31:0] fifo_out_pkt_id;
    wire    [                             15:0] fifo_out_req_rq_pi_db;
    wire    [              C_QP_INDX_WIDTH-1:0] fifo_out_qpid;
    wire    [                              3:0] fifo_out_pkt_type;
    wire    [           C_M_AXI_ADDR_WIDTH-1:0] fifo_out_dest_addr;
    wire    [                              6:0] fifo_out_awlen;
    wire    [                             31:0] awchnl_pkt_id;

    wire                                        set_wr_axi_wchnl_fifo_2;
    wire                                        clr_wr_axi_wchnl_fifo_2;

    wire                                        wr_axi_wchnl_fifo_1;
    reg                                         wr_axi_wchnl_fifo_2;

    wire    [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_din_1;
    wire    [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_din_2;

    wire    [                             13:0] rq_pi_db_addr_async;
    wire    [                             15:0] rq_pi_db_val_async;

    wire    [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_din;
    wire    [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_dout;
    wire                                        wr_axi_wchnl_fifo;
    reg                                         rd_axi_wchnl_fifo;
    wire                                        axi_wchnl_fifo_full;

    // Keep a complete descriptor stable until the W FSM consumes it.
    wire    [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_raw_dout;
    wire                                        axi_wchnl_fifo_raw_empty;
    wire                                        rd_axi_wchnl_fifo_raw;
    wire                                        axi_wchnl_desc_ready;
    wire                                        axi_wchnl_desc_valid;

    wire    [                             31:0] wchnl_pkt_id;
    wire    [                              3:0] wchnl_req_pkt_type;
    wire                                        wchnl_req_pkt_ip_ver;
    wire    [         C_REQ_BUF_ADDR_WIDTH-1:0] wchnl_req_buf_addr_offset;
    wire    [                             12:0] wchnl_pyld_len;
    wire    [                             43:0] wchnl_imm_data;
    wire    [                              3:0] wchnl_lshift_by;
    wire                                        wchnl_req_pkt_ieth_present;
    wire                                        wchnl_req_pkt_immdt_present;
    reg                                         wchnl_req_pkt_ieth_present_r;
    reg                                         wchnl_req_pkt_immdt_present_r;
    reg                                         set_ieth_or_immdt_present;
    reg                                         ieth_or_immdt_present;

    reg     [                              6:0] wcnt;
    reg     [                              6:0] nxt_wcnt;

    wire    [                              6:0] wchnl_beats;

    reg                                         async_db_update;

    wire                                        stale_db_update;

    reg     [         C_REQ_BUF_ADDR_WIDTH-1:0] prev_req_pkt_buf_addrb;
    reg                                         req_pkt_buf_enb_dummy;
    reg                                         cpy_imm_data;
    reg                                         set_last_cyc;
    reg                                         last_cyc;
    reg                                         set_single_beat_only;
    reg                                         single_beat_only;
    reg     [                            511:0] prev_req_pkt_buf_doutb;  // req_pkt_buf_doutb上一个周期读出的数据

    wire    [                              3:0] wchnl_req_pkt_type_to_data_mux;

    reg                                         m_axi_wvalid_i;
    reg     [                            511:0] m_axi_wdata_i;
    reg     [                             63:0] m_axi_wstrb_i;
    reg                                         m_axi_wlast_i;
    wire                                        m_axi_wready_i;

    wire                                        post_axi_tnfr_fifo_full_early;
    wire                                        st0_exit_cond;  // 写数据状态机在 state 0 的“允许推进条件”标志
    wire                                        axi_mstr_idle;

    reg     [                              1:0] axi_wchnl_cs;
    reg     [                              1:0] axi_wchnl_ns;

    reg     [                             31:0] wchnl_pkt_id_r;
    reg     [                              3:0] wchnl_req_pkt_type_r;
    reg     [                             43:0] wchnl_imm_data_r;

    wire                                        rd_post_axi_tnfr_fifo_1;
    wire                                        rd_post_axi_tnfr_fifo_2;

    wire    [POST_AXI_TNFR_FIFO_DATA_WIDTH-1:0] post_axi_tnfr_fifo_din;
    wire    [POST_AXI_TNFR_FIFO_DATA_WIDTH-1:0] post_axi_tnfr_fifo_dout;
    wire                                        wr_post_axi_tnfr_fifo;
    wire                                        rd_post_axi_tnfr_fifo;
    wire                                        post_axi_tnfr_fifo_full;
    wire                                        post_axi_tnfr_fifo_almost_full;

    wire    [                             31:0] pkt_id_post_tnfr;
    wire    [                              9:0] free_run_cnt_at_wlast;
    wire    [                              3:0] req_pkt_type_post_tnfr;
    wire    [                             43:0] imm_data_post_tnfr;

    reg                                         incr_bypass_pkt_cnt;
    reg                                         incr_send_pkt_cnt_1;
    reg                                         incr_send_req_cnt_1;
    wire                                        incr_send_req_cnt_2;
    reg                                         incr_mad_pkt_cnt_1;
    wire                                        incr_mad_pkt_cnt_2;
    reg                                         incr_in_err_sts_q_wrptr;

    reg                                         set_mad_pkt_rcvd_intr_1;
    wire                                        set_mad_pkt_rcvd_intr_2;

    wire    [           DB_FIFO_DATA_WIDTH-1:0] db_fifo_din;
    wire    [           DB_FIFO_DATA_WIDTH-1:0] db_fifo_dout;
    reg                                         wr_db_fifo;
    reg                                         rd_db_fifo;
    wire                                        db_fifo_full;

    reg                                         db_fsm_cs;
    reg                                         db_fsm_ns;

    wire    [                             31:0] pkt_id_db;

    reg                                         m_axi_bvalid_d;

    reg                                         cnct_io_db_update;
    reg                                         rq_wrptr_db_we_1;
    wire                                        ieth_immdt_fifo_full;
    //  reg db_ptr_buf_we;
    //  wire [C_QP_INDX_WIDTH-1:0] db_ptr_buf_addra;
    //  wire [15:0] db_ptr_buf_dina;
    //  wire [15:0] db_ptr_buf_doutb;
    //  reg hw_hndshk_disable_to_0_r;

    // 准备条件为： 当前没有响应传输有效 & AXI写地址通道的FIFO没满 & AXI写数据通道的FIFO没满 & IETH/IMMDT FIFO也没有满
    assign rdy_to_req_tnfr    = (!rsp_tnfr_vld && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full && !ieth_immdt_fifo_full) ? 1'b1 : 1'b0;

    assign rdy_to_rsp_tnfr    = (!axi_awchnl_fifo_full && !axi_wchnl_fifo_full) ? 1'b1 : 1'b0;

    assign wr_axi_awchnl_fifo = (!axi_awchnl_fifo_full && !axi_wchnl_fifo_full && !ieth_immdt_fifo_full && (rsp_tnfr_vld || (req_tnfr_vld && (req_pkt_type != REQ_PKT_TYPE_DISCARD)))) ? 1'b1 : 1'b0;

    assign pkt_id             = rsp_tnfr_vld ? rsp_pkt_id : req_pkt_id;
    assign imm_data           = rsp_tnfr_vld ? {12'h000, rsp_imm_data} : req_imm_data;
    assign pkt_type           = rsp_tnfr_vld ? IN_ERR_STS : req_pkt_type;
    assign pkt_ip_ver         = rsp_tnfr_vld ? IPv6 : req_pkt_ip_ver;
    assign qpid               = rsp_tnfr_vld ? 0 : {pkt_type, req_qpid};
    assign ieth_present       = rsp_tnfr_vld ? 1'b0 : req_ieth_present;
    assign immdt_present      = rsp_tnfr_vld ? 1'b0 : req_immdt_present;

    // ================ 环形队列写地址&写指针管理 ================
    // 错误缓冲区/队列
    always @(posedge clk)
        if (reset) begin
            err_buf_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            err_buf_wrptr <= 16'd0;
        end else if (xrnic_en_pulse) begin
            err_buf_ca <= i_err_buf_ba;
        end else if (req_tnfr_vld && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full && !ieth_immdt_fifo_full && (pkt_type == REQ_PKT_TYPE_ERROR)) begin
            if (err_buf_wrptr == i_err_num_bufs - 'd1) begin
                err_buf_ca    <= i_err_buf_ba;
                err_buf_wrptr <= 16'd0;
            end else begin
                err_buf_ca    <= err_buf_ca + i_err_buf_sz;
                err_buf_wrptr <= err_buf_wrptr + 'd1;
            end
        end

    // 旁路缓冲区/队列
    always @(posedge clk)
        if (reset) begin
            bypass_buf_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            bypass_buf_wrptr <= 16'd0;
        end else if (xrnic_en_pulse) begin
            bypass_buf_ca <= i_bypass_buf_ba;
        end else if (req_tnfr_vld && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full && !ieth_immdt_fifo_full && (pkt_type == REQ_PKT_TYPE_BYPASS)) begin
            if (bypass_buf_wrptr == i_bypass_num_bufs - 'd1) begin
                bypass_buf_ca    <= i_bypass_buf_ba;
                bypass_buf_wrptr <= 16'd0;
            end else begin
                bypass_buf_ca    <= bypass_buf_ca + i_bypass_buf_sz;
                bypass_buf_wrptr <= bypass_buf_wrptr + 'd1;
            end
        end

    // 传入数据包的错误状态缓冲区/队列
    always @(posedge clk)
        if (reset) begin
            in_err_sts_q_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            in_err_sts_q_wrptr <= 16'd0;
        end else if (xrnic_en_pulse) begin
            in_err_sts_q_ca <= i_in_err_sts_q_ba;
        end else if ((req_tnfr_vld || rsp_tnfr_vld) && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full && !ieth_immdt_fifo_full && (pkt_type == IN_ERR_STS)) begin
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
            REQ_PKT_TYPE_BYPASS: dest_addr = bypass_buf_ca;
            IN_ERR_STS: dest_addr = in_err_sts_q_ca;
            default: dest_addr = req_dest_addr;
        endcase
    end

    //For the always block below, it is assumed that the WR and RD pkt types
    //are not passed to the module in case the RD and WR processing is not 
    //enabled in the design. Otherwise the dut may show unexpected behaviour
    // 
    // pyld_len 为 AXI 写通道口径长度，不完全等同协议净载荷长度，
    // 
    // WR_REQ/SEND/RETH 分支按协议头剥离计算
    // IETH/IMMDT 出现时额外减 4B
    // MAD 分支为实现对齐口径：IPv4 使用补偿项（+20），并与 WDATA 首拍 20B zero padding 配套
    generate
        if (C_EN_IETH_IMMDT == 0) begin
            always @(*) begin
                case (pkt_type)
                    REQ_PKT_TYPE_DISCARD: pyld_len = req_pkt_len;
                    REQ_PKT_TYPE_ERROR: pyld_len = (i_err_buf_sz < (req_pkt_len + 'd4)) ? i_err_buf_sz : (req_pkt_len + 'd4);
                    REQ_PKT_TYPE_BYPASS: pyld_len = (i_bypass_buf_sz < {3'b0, req_pkt_len}) ? i_bypass_buf_sz : req_pkt_len;
                    REQ_PKT_TYPE_MAD: begin
                        if (pkt_ip_ver == IPv6) pyld_len = req_pkt_len - 'd18;
                        else pyld_len = req_pkt_len - 'd18 + 'd20;
                    end
                    REQ_PKT_TYPE_IPV4_WR_REQ, REQ_PKT_TYPE_IPV4_SEND: pyld_len = req_pkt_len - 'd58;
                    REQ_PKT_TYPE_IPV6_WR_REQ, REQ_PKT_TYPE_IPV6_SEND: pyld_len = req_pkt_len - 'd78;
                    REQ_DB_UPDATE: pyld_len = 13'd1;
                    IN_ERR_STS: pyld_len = 13'd1;
                    REQ_PKT_TYPE_IPV4_WR_REQ_RETH: pyld_len = req_pkt_len - 'd74;
                    REQ_PKT_TYPE_IPV6_WR_REQ_RETH: pyld_len = req_pkt_len - 'd94;
                    default: pyld_len = 0;
                endcase
            end
        end else begin  // current: C_EN_IETH_IMMDT == 1
            always @(*) begin
                case (pkt_type)
                    REQ_PKT_TYPE_DISCARD: pyld_len = req_pkt_len;
                    REQ_PKT_TYPE_ERROR: pyld_len = (i_err_buf_sz < (req_pkt_len + 'd4)) ? i_err_buf_sz : (req_pkt_len + 'd4);
                    REQ_PKT_TYPE_BYPASS: pyld_len = (i_bypass_buf_sz < {3'b0, req_pkt_len}) ? i_bypass_buf_sz : req_pkt_len;
                    REQ_PKT_TYPE_MAD: begin
                        if (pkt_ip_ver == IPv6) pyld_len = req_pkt_len - 'd18;
                        else pyld_len = req_pkt_len - 'd18 + 'd20;
                    end
                    REQ_PKT_TYPE_IPV4_WR_REQ, REQ_PKT_TYPE_IPV4_SEND: begin
                        if (ieth_present || immdt_present) pyld_len = req_pkt_len - 'd62;
                        else pyld_len = req_pkt_len - 'd58;
                    end
                    REQ_PKT_TYPE_IPV6_WR_REQ, REQ_PKT_TYPE_IPV6_SEND: begin
                        if (ieth_present || immdt_present) pyld_len = req_pkt_len - 'd82;
                        else pyld_len = req_pkt_len - 'd78;
                    end
                    REQ_DB_UPDATE: pyld_len = 13'd1;
                    IN_ERR_STS: pyld_len = 13'd1;
                    REQ_PKT_TYPE_IPV4_WR_REQ_RETH: begin
                        if (ieth_present || immdt_present) pyld_len = req_pkt_len - 'd78;
                        else pyld_len = req_pkt_len - 'd74;
                    end
                    REQ_PKT_TYPE_IPV6_WR_REQ_RETH: begin
                        if (ieth_present || immdt_present) pyld_len = req_pkt_len - 'd98;
                        else pyld_len = req_pkt_len - 'd94;
                    end
                    default: pyld_len = 0;
                endcase
            end
        end
    endgenerate

    assign awlen     = (pyld_len[5:0] == 6'd0) ? (pyld_len[12:6] - 7'd1) : pyld_len[12:6];  // 计算 AxiLen，表示一次突发传输中包含的数据传输数量，实际的长度值为 AxiLen + 1，为 0 表示突发传输 1 次
    assign lshift_by = (pkt_type == IN_ERR_STS) ? in_err_sts_q_ca[5:2] : req_dest_addr[5:2];

    generate
        if (C_SIM_DEBUG)
            assign axi_awchnl_fifo_din = {
                pkt_id,  // 32
                req_rq_pi_db,  //16
                qpid,
                dest_addr,  // C_M_AXI_ADDR_WIDTH
                awlen  // 7
            };
        else  // current: C_SIM_DEBUG == 0
            assign axi_awchnl_fifo_din = {
                req_rq_pi_db,  //16
                qpid,  // { (4 bit) pkt_type, (12 bit) QPID }
                dest_addr,  // C_M_AXI_ADDR_WIDTH
                awlen  // 7
            };
    endgenerate

    //==================================
    // AW FIFO
    // 地址元数据 FIFO { req_rq_pi_db, qpid, dest_addr, awlen }
    //==================================  
    xpm_fifo_sync_wrap #(
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
        .rd_en        (rd_axi_awchnl_fifo_raw),
        .dout         (axi_awchnl_fifo_raw_dout),
        .empty        (axi_awchnl_fifo_raw_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      ()
    );
    //==================================    

    // Full register slices retain two descriptors on sudden backpressure;
    // READY is registered, and neither metadata nor VALID bypasses storage.
    // Pop the BRAM only when the slice accepts; pop the slice on AXI handshake.
    assign rd_axi_awchnl_fifo_raw = !reset && !axi_awchnl_fifo_raw_empty && axi_awchnl_desc_ready;
    reg_slice #(.C_DATA_WIDTH(AWCHNL_FIFO_DATA_WIDTH)) awchnl_desc_slice (
        .aclk(clk), .aclken(1'b1), .areset(reset),
        .S_PAYLOAD_DATA(axi_awchnl_fifo_raw_dout),
        .S_VALID(!reset && !axi_awchnl_fifo_raw_empty),
        .S_READY(axi_awchnl_desc_ready),
        .M_PAYLOAD_DATA(axi_awchnl_fifo_dout),
        .M_VALID(axi_awchnl_desc_valid),
        .M_READY(m_axi_awready)
    );
    // Public empty is a drain indication, not slice readiness.
    assign axi_awchnl_fifo_empty = axi_awchnl_fifo_raw_empty && !axi_awchnl_desc_valid;
    assign rd_axi_awchnl_fifo = axi_awchnl_desc_valid && m_axi_awready;

    // 地址元数据 pkt_id 字段
    generate
        if (C_SIM_DEBUG) assign awchnl_pkt_id = axi_awchnl_fifo_dout[(C_M_AXI_ADDR_WIDTH+7)+:32];
        else assign awchnl_pkt_id = 32'd0;
    endgenerate

    // 地址元数据各字段
    generate
        if (C_SIM_DEBUG) assign {fifo_out_pkt_id, fifo_out_req_rq_pi_db, fifo_out_pkt_type, fifo_out_qpid, fifo_out_dest_addr, fifo_out_awlen} = axi_awchnl_fifo_dout;
        else assign {fifo_out_req_rq_pi_db, fifo_out_pkt_type, fifo_out_qpid, fifo_out_dest_addr, fifo_out_awlen} = axi_awchnl_fifo_dout;
    endgenerate

    assign m_axi_awid              = 1'b1;
    assign m_axi_awaddr            = !axi_awchnl_desc_valid ? {(C_M_AXI_ADDR_WIDTH) {1'b0}} : axi_awchnl_fifo_dout[7+:C_M_AXI_ADDR_WIDTH];  // AW FIFO dest_addr 字段

    assign m_axi_awuser[31:16]     = fifo_out_req_rq_pi_db;  // AW FIFO req_rq_pi_db字段
    assign m_axi_awuser[15:12]     = fifo_out_pkt_type;  // pkt_type
    assign m_axi_awuser[11:0]      = {{(12 - C_QP_INDX_WIDTH) {1'b0}}, fifo_out_qpid[C_QP_INDX_WIDTH-1:0]};  // QPID
    assign m_axi_awlen             = !axi_awchnl_desc_valid ? 8'b0 : {1'b0, axi_awchnl_fifo_dout[6:0]};  // AW FIFO awlen字段
    assign m_axi_awsize            = 3'b110;
    assign m_axi_awburst           = 2'b01;
    assign m_axi_awcache           = 4'b0011;
    assign m_axi_awprot            = 3'b000;
    assign m_axi_awvalid           = axi_awchnl_desc_valid;
    assign m_axi_awlock            = 1'b0;

    //assign set_wr_axi_wchnl_fifo_2 = (i_hw_hndshk_disable_to_0 && (i_rq_pi_db_hw_hndshk != 'd0)) ? 1'b1 : 1'b0;
    assign set_wr_axi_wchnl_fifo_2 = i_hw_hndshk_disable_to_0;
    /*  
  assign stale_db_update = (wr_axi_wchnl_fifo_1 && 
                           ((pkt_type == REQ_PKT_TYPE_IPV4_SEND) || (pkt_type == REQ_PKT_TYPE_IPV6_SEND)) && 
                           imm_data[38] && 
                           (imm_data[27 +:10] == rq_pi_db_addr_async)) ? 1'b1 : 1'b0;
*/

    assign stale_db_update         = 1'b0;

    assign clr_wr_axi_wchnl_fifo_2 = (~wr_axi_wchnl_fifo_1 | stale_db_update) & ~axi_wchnl_fifo_full;

    assign rq_pi_db_addr_async     = i_connect_io_qp_rq_pi_db_wptr;
    assign rq_pi_db_val_async      = i_rq_pi_db_hw_hndshk;

    // 当 AXI写地址通道的FIFO没满 && AXI写数据通道的FIFO没满 & IETH/IMMDT FIFO也没有满 & (有响应传输有效 || 有请求传输有效)，使用fifo_1数据，反之使用fifo_2数据
    assign wr_axi_wchnl_fifo_1     = (!axi_awchnl_fifo_full && !axi_wchnl_fifo_full && !ieth_immdt_fifo_full && (rsp_tnfr_vld || req_tnfr_vld)) ? 1'b1 : 1'b0;

    always @(posedge clk) begin
        //if(reset) begin
        //    wr_axi_wchnl_fifo_2 <= 1'b0;
        //    hw_hndshk_disable_to_0_r <= 1'b0;
        //end else begin
        if (set_wr_axi_wchnl_fifo_2 || clr_wr_axi_wchnl_fifo_2) wr_axi_wchnl_fifo_2 <= set_wr_axi_wchnl_fifo_2;
        //if (set_wr_axi_wchnl_fifo_2 || clr_wr_axi_wchnl_fifo_2)
        //	wr_axi_wchnl_fifo_2 <= set_wr_axi_wchnl_fifo_2 && (rq_pi_db_val_async == db_ptr_buf_doutb);

        //hw_hndshk_disable_to_0_r <= i_hw_hndshk_disable_to_0;
    end
    //end

    //assign o_connect_io_qp_rq_pi_db_rdy = ~wr_axi_wchnl_fifo_2 || hw_hndshk_disable_to_0_r;
    assign o_connect_io_qp_rq_pi_db_rdy = ~wr_axi_wchnl_fifo_2;

    // { ieth_present, immdt_present, pkt_ip_ver, pkt_type, req_buf_addr_offset, pyld_len, imm_data, lshift_by }
    generate
        if (C_SIM_DEBUG)
            if (C_EN_IETH_IMMDT == 0) begin
                assign axi_wchnl_fifo_din_1 = {
                    pkt_id,  // 32
                    pkt_ip_ver,  // 1
                    pkt_type,  // 4
                    req_buf_addr_offset,  // C_REQ_BUF_ADDR_WIDTH
                    pyld_len,  // 13
                    //imm_data,             // 40
                    imm_data,  // 44
                    lshift_by  // 4
                };
            end else begin
                assign axi_wchnl_fifo_din_1 = {
                    pkt_id,  // 32
                    ieth_present,  // 1
                    immdt_present,  // 1
                    pkt_ip_ver,  // 1
                    pkt_type,  // 4
                    req_buf_addr_offset,  // C_REQ_BUF_ADDR_WIDTH
                    pyld_len,  // 13
                    //imm_data,             // 40
                    imm_data,  // 44
                    lshift_by  // 4
                };
            end
        else if (C_EN_IETH_IMMDT == 0) begin
            assign axi_wchnl_fifo_din_1 = {
                pkt_ip_ver,  // 1
                pkt_type,  // 4
                req_buf_addr_offset,  // C_REQ_BUF_ADDR_WIDTH
                pyld_len,  // 13
                //imm_data,             // 40
                imm_data,  // 44
                lshift_by  // 4
            };
        end else begin  // current: C_SIM_DEBUG = 0 && C_EN_IETH_IMMDT = 1
            assign axi_wchnl_fifo_din_1 = {
                ieth_present,  // 1
                immdt_present,  // 1
                pkt_ip_ver,  // 1
                pkt_type,  // 4
                req_buf_addr_offset,  // C_REQ_BUF_ADDR_WIDTH
                pyld_len,  // 13
                //imm_data,             // 40
                imm_data,  // 44
                lshift_by  // 4
            };
        end
    endgenerate

    generate
        if (C_SIM_DEBUG)
            if (C_EN_IETH_IMMDT == 0) begin
                assign axi_wchnl_fifo_din_2 = {
                    32'd0,  // 32 bits
                    REQ_PKT_TYPE_DISCARD,  //  4 bits
                    {(C_REQ_BUF_ADDR_WIDTH) {1'b0}},  // C_REQ_BUF_ADDR_WIDTH  
                    13'd0,  // 13 bits
                    3'd0,  //  3 bits
                    //rq_pi_db_addr_async,  // 10 bits
                    rq_pi_db_addr_async,  // 14 bits
                    11'd0,  // 11 bits
                    rq_pi_db_val_async,  // 16 bits
                    4'd0  //  4 bits
                };
            end else begin
                assign axi_wchnl_fifo_din_2 = {
                    32'd0,  // 32 bits
                    2'b00,  // 2 bits
                    REQ_PKT_TYPE_DISCARD,  //  4 bits
                    {(C_REQ_BUF_ADDR_WIDTH) {1'b0}},  // C_REQ_BUF_ADDR_WIDTH  
                    13'd0,  // 13 bits
                    3'd0,  //  3 bits
                    //rq_pi_db_addr_async,  // 10 bits
                    rq_pi_db_addr_async,  // 14 bits
                    11'd0,  // 11 bits
                    rq_pi_db_val_async,  // 16 bits
                    4'd0  //  4 bits
                };
            end
        else if (C_EN_IETH_IMMDT == 0) begin
            assign axi_wchnl_fifo_din_2 = {
                REQ_PKT_TYPE_DISCARD,  //  4 bits
                {(C_REQ_BUF_ADDR_WIDTH) {1'b0}},  // C_REQ_BUF_ADDR_WIDTH  
                13'd0,  // 13 bits
                3'd0,  //  3 bits
                //rq_pi_db_addr_async,  // 10 bits
                rq_pi_db_addr_async,  // 14 bits
                11'd0,  // 11 bits
                rq_pi_db_val_async,  // 16 bits
                4'd0  //  4 bits
            };
        end else begin  // current: C_SIM_DEBUG = 0 && C_EN_IETH_IMMDT = 1
            assign axi_wchnl_fifo_din_2 = {
                2'b00,  // 2 bits
                REQ_PKT_TYPE_DISCARD,  //  4 bits
                {(C_REQ_BUF_ADDR_WIDTH) {1'b0}},  // C_REQ_BUF_ADDR_WIDTH  
                13'd0,  // 13 bits
                3'd0,  //  3 bits
                //rq_pi_db_addr_async,  // 10 bits
                rq_pi_db_addr_async,  // 14 bits
                11'd0,  // 11 bits
                rq_pi_db_val_async,  // 16 bits
                4'd0  //  4 bits
            };
        end
    endgenerate

    assign axi_wchnl_fifo_din = wr_axi_wchnl_fifo_1 ? axi_wchnl_fifo_din_1 : axi_wchnl_fifo_din_2;
    assign wr_axi_wchnl_fifo  = wr_axi_wchnl_fifo_1 | (~axi_wchnl_fifo_full & wr_axi_wchnl_fifo_2);

    //=================================
    // W FIFO
    // 写数据元数据通道 FIFO
    //=================================
    xpm_fifo_sync_wrap #(
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
        .rd_en        (rd_axi_wchnl_fifo_raw),
        .dout         (axi_wchnl_fifo_raw_dout),
        .empty        (axi_wchnl_fifo_raw_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      ()
    );
    //=================================

    assign rd_axi_wchnl_fifo_raw = !reset && !axi_wchnl_fifo_raw_empty && axi_wchnl_desc_ready;
    reg_slice #(.C_DATA_WIDTH(WCHNL_FIFO_DATA_WIDTH)) wchnl_desc_slice (
        .aclk(clk), .aclken(1'b1), .areset(reset),
        .S_PAYLOAD_DATA(axi_wchnl_fifo_raw_dout),
        .S_VALID(!reset && !axi_wchnl_fifo_raw_empty),
        .S_READY(axi_wchnl_desc_ready),
        .M_PAYLOAD_DATA(axi_wchnl_fifo_dout),
        .M_VALID(axi_wchnl_desc_valid),
        .M_READY(rd_axi_wchnl_fifo)
    );
    assign axi_wchnl_fifo_empty = axi_wchnl_fifo_raw_empty && !axi_wchnl_desc_valid;

    generate
        if (C_SIM_DEBUG) assign wchnl_pkt_id = axi_wchnl_fifo_dout[(C_REQ_BUF_ADDR_WIDTH+68)+:32];
        else assign wchnl_pkt_id = 32'd0;
    endgenerate

    generate
        if (C_EN_IETH_IMMDT == 0) begin
            assign wchnl_req_pkt_ieth_present  = 1'b0;
            assign wchnl_req_pkt_immdt_present = 1'b0;
        end else begin
            assign wchnl_req_pkt_ieth_present  = axi_wchnl_fifo_dout[(C_REQ_BUF_ADDR_WIDTH+67)];
            assign wchnl_req_pkt_immdt_present = axi_wchnl_fifo_dout[(C_REQ_BUF_ADDR_WIDTH+66)];
        end
    endgenerate
    assign wchnl_req_pkt_ip_ver      = axi_wchnl_fifo_dout[(C_REQ_BUF_ADDR_WIDTH+65)+:1];
    assign wchnl_req_pkt_type        = axi_wchnl_fifo_dout[(C_REQ_BUF_ADDR_WIDTH+61)+:4];
    assign wchnl_req_buf_addr_offset = axi_wchnl_fifo_dout[61+:C_REQ_BUF_ADDR_WIDTH];  // req_buf_addr_offset
    assign wchnl_pyld_len            = axi_wchnl_fifo_dout[48+:13];
    assign wchnl_imm_data            = axi_wchnl_fifo_dout[4+:44];
    assign wchnl_lshift_by           = axi_wchnl_fifo_dout[0+:4];

    // ~axi_wchnl_fifo_empty：W 元数据 FIFO 里必须有待处理项，否则 state0 不该动。
    // ~post_axi_tnfr_fifo_full：后级 post FIFO 不能满；因为本次事务完成后要往里写完成信息，满了就先背压
    // (!last_cyc | m_axi_wready_i)：如果当前不是“尾拍待发送”就可推进；如果当前是尾拍，则必须等 WREADY 握手成功，防止上一包最后一拍还没被接收就读下一条元数据。
    assign st0_exit_cond             = axi_wchnl_desc_valid & ~post_axi_tnfr_fifo_full & (!last_cyc | m_axi_wready_i);
    assign axi_mstr_idle             = axi_wchnl_fifo_empty & ~m_axi_wvalid_i;

    assign wchnl_beats               = |wchnl_pyld_len[5:0] ? wchnl_pyld_len[12:6] : (wchnl_pyld_len[12:6] - 'd1);

    //For the always block below, it is assumed that the WR and RD pkt types
    //are not passed to the module in case the RD and WR processing is not 
    //enabled in the design. Otherwise the dut may show unexpected behaviour
    always @(*) begin
        // default values
        nxt_wcnt                  = wcnt;
        req_pkt_buf_addrb         = prev_req_pkt_buf_addrb;
        req_pkt_buf_enb           = 1'b0;
        req_pkt_buf_enb_dummy     = 1'b0;
        rd_axi_wchnl_fifo         = 1'b0;
        cpy_imm_data              = 1'b0;
        set_last_cyc              = 1'b0;
        set_first_cyc             = 1'b0;
        set_single_beat_only      = 1'b0;
        m_axi_wvalid_i            = 1'b0;
        m_axi_wlast_i             = 1'b0;
        m_axi_wstrb_i             = {64{1'b0}};
        axi_wchnl_ns              = axi_wchnl_cs;
        async_db_update           = 1'b0;
        mad_pkt_ip_ver            = current_mad_pkt_ip_ver;
        set_ieth_or_immdt_present = 1'b0;

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
                        end  // REQ_PKT_TYPE_DISCARD

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
                        end  // REQ_PKT_TYPE_ERROR

                        REQ_PKT_TYPE_BYPASS: begin
                            req_pkt_buf_addrb = wchnl_req_buf_addr_offset;
                            req_pkt_buf_enb   = 1'b1;

                            if (wchnl_pyld_len <= 13'd64) begin
                                nxt_wcnt          = 7'd0;
                                set_last_cyc      = 1'b1;
                                rd_axi_wchnl_fifo = 1'b1;
                            end else begin
                                nxt_wcnt     = wchnl_beats;
                                axi_wchnl_ns = 2'd2;
                            end
                        end  // REQ_PKT_TYPE_BYPASS

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
                        end  // REQ_PKT_TYPE_MAD

                        REQ_PKT_TYPE_IPV4_WR_REQ_RETH: begin
                            req_pkt_buf_addrb = wchnl_req_buf_addr_offset + 'd1;  // 有效载荷起点相对更靠后
                            req_pkt_buf_enb   = 1'b1;

                            if (C_EN_IETH_IMMDT == 0) begin
                                if (wchnl_pyld_len <= 13'd58) begin
                                    nxt_wcnt             = 7'd0;
                                    set_last_cyc         = 1'b1;
                                    set_single_beat_only = 1'b1;
                                    rd_axi_wchnl_fifo    = 1'b1;
                                end else begin
                                    nxt_wcnt     = wchnl_beats;
                                    axi_wchnl_ns = 2'd1;
                                end
                            end else begin
                                set_ieth_or_immdt_present = wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present;
                                if (((wchnl_pyld_len <= 13'd58) && ~(wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present)) || ((wchnl_pyld_len <= 13'd54) && (wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present))) begin
                                    nxt_wcnt             = 7'd0;
                                    set_last_cyc         = 1'b1;
                                    set_single_beat_only = 1'b1;
                                    rd_axi_wchnl_fifo    = 1'b1;
                                end else begin
                                    nxt_wcnt     = wchnl_beats;
                                    axi_wchnl_ns = 2'd1;
                                end
                            end
                        end  // REQ_PKT_TYPE_IPV4_WR_REQ_RETH

                        REQ_PKT_TYPE_IPV6_WR_REQ_RETH: begin
                            req_pkt_buf_addrb = wchnl_req_buf_addr_offset + 'd1;  // 有效载荷起点相对更靠后
                            req_pkt_buf_enb   = 1'b1;

                            if (C_EN_IETH_IMMDT == 0) begin
                                if (wchnl_pyld_len <= 13'd38) begin
                                    nxt_wcnt             = 7'd0;
                                    set_last_cyc         = 1'b1;
                                    set_single_beat_only = 1'b1;
                                    rd_axi_wchnl_fifo    = 1'b1;
                                end else begin
                                    nxt_wcnt     = wchnl_beats;
                                    axi_wchnl_ns = 2'd1;
                                end
                            end else begin
                                set_ieth_or_immdt_present = wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present;
                                //if (wchnl_pyld_len <= 13'd38)
                                if (((wchnl_pyld_len <= 13'd38) && ~(wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present)) || ((wchnl_pyld_len <= 13'd34) && (wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present))) begin
                                    nxt_wcnt             = 7'd0;
                                    set_last_cyc         = 1'b1;
                                    set_single_beat_only = 1'b1;
                                    rd_axi_wchnl_fifo    = 1'b1;
                                end else begin
                                    nxt_wcnt     = wchnl_beats;
                                    axi_wchnl_ns = 2'd1;
                                end
                            end
                        end  // REQ_PKT_TYPE_IPV6_WR_REQ_RETH


                        REQ_PKT_TYPE_IPV4_WR_REQ, REQ_PKT_TYPE_IPV4_SEND: begin
                            req_pkt_buf_addrb = wchnl_req_buf_addr_offset;
                            req_pkt_buf_enb   = 1'b1;

                            if (C_EN_IETH_IMMDT == 0) begin
                                if (wchnl_pyld_len <= 13'd10) begin
                                    nxt_wcnt             = 7'd0;
                                    set_last_cyc         = 1'b1;
                                    set_single_beat_only = 1'b1;
                                    rd_axi_wchnl_fifo    = 1'b1;
                                end else begin
                                    nxt_wcnt     = wchnl_beats;
                                    axi_wchnl_ns = 2'd1;
                                end
                            end else begin
                                set_ieth_or_immdt_present = wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present;
                                //if (wchnl_pyld_len <= 13'd10)
                                if (((wchnl_pyld_len <= 13'd10) && ~(wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present)) || ((wchnl_pyld_len <= 13'd6) && (wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present))) begin
                                    nxt_wcnt             = 7'd0;
                                    set_last_cyc         = 1'b1;
                                    set_single_beat_only = 1'b1;
                                    rd_axi_wchnl_fifo    = 1'b1;
                                end else begin
                                    nxt_wcnt     = wchnl_beats;
                                    axi_wchnl_ns = 2'd1;
                                end
                            end
                        end  // REQ_PKT_TYPE_IPV4_SEND, REQ_PKT_TYPE_IPV4_WR_REQ

                        REQ_PKT_TYPE_IPV6_WR_REQ, REQ_PKT_TYPE_IPV6_SEND: begin
                            req_pkt_buf_addrb = wchnl_req_buf_addr_offset + 'd1;  // 有效载荷起点相对更靠后
                            req_pkt_buf_enb   = 1'b1;

                            if (C_EN_IETH_IMMDT == 0) begin
                                if (wchnl_pyld_len <= 13'd50) begin
                                    nxt_wcnt             = 7'd0;
                                    set_last_cyc         = 1'b1;
                                    set_single_beat_only = 1'b1;
                                    rd_axi_wchnl_fifo    = 1'b1;
                                end else begin
                                    nxt_wcnt     = wchnl_beats;
                                    axi_wchnl_ns = 2'd1;
                                end
                            end else begin
                                set_ieth_or_immdt_present = wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present;
                                if (((wchnl_pyld_len <= 13'd50) && ~(wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present)) || ((wchnl_pyld_len <= 13'd46) && (wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present))) begin
                                    nxt_wcnt             = 7'd0;
                                    set_last_cyc         = 1'b1;
                                    set_single_beat_only = 1'b1;
                                    rd_axi_wchnl_fifo    = 1'b1;
                                end else begin
                                    nxt_wcnt     = wchnl_beats;
                                    axi_wchnl_ns = 2'd1;
                                end
                            end
                        end  // REQ_PKT_TYPE_IPV6_SEND , REQ_PKT_TYPE_IPV6_WR_REQ

                        REQ_DB_UPDATE: begin
                            nxt_wcnt = 7'd0;
                            if (!last_cyc) begin
                                m_axi_wvalid_i = 1'b1;
                                m_axi_wstrb_i  = {{62{1'b0}}, {2{1'b1}}} << {wchnl_lshift_by, {2{1'b0}}};
                                m_axi_wlast_i  = 1'b1;
                                if (m_axi_wready_i) begin
                                    rd_axi_wchnl_fifo = 1'b1;
                                end
                            end
                        end  // REQ_DB_UPDATE

                        IN_ERR_STS: begin
                            nxt_wcnt = 7'd0;
                            if (!last_cyc) begin
                                m_axi_wvalid_i = 1'b1;
                                m_axi_wstrb_i  = {{60{1'b0}}, {4{1'b1}}} << {wchnl_lshift_by, {2{1'b0}}};
                                m_axi_wlast_i  = 1'b1;
                                if (m_axi_wready_i) begin
                                    rd_axi_wchnl_fifo = 1'b1;
                                end
                            end
                        end  // IN_ERR_STS

                    endcase  // case (wchnl_req_pkt_type) 
                end  // if (st0_exit_cond)
            end  // 2'd0

            2'd1: begin
                req_pkt_buf_addrb         = prev_req_pkt_buf_addrb + 'd1;
                req_pkt_buf_enb           = 1'b1;
                set_ieth_or_immdt_present = wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present;

                if (wcnt == 7'd0) begin
                    set_last_cyc      = 1'b1;
                    rd_axi_wchnl_fifo = 1'b1;
                    axi_wchnl_ns      = 2'd0;
                end else begin
                    axi_wchnl_ns = 2'd2;
                end
            end  //2'd1

            2'd2: begin
                set_ieth_or_immdt_present = wchnl_req_pkt_ieth_present || wchnl_req_pkt_immdt_present;
                m_axi_wvalid_i            = 1'b1;
                m_axi_wstrb_i             = {64{1'b1}};
                set_first_cyc             = 1'b0;
                if (m_axi_wready_i) begin
                    nxt_wcnt          = wcnt - 'd1;
                    req_pkt_buf_addrb = prev_req_pkt_buf_addrb + 'd1;
                    req_pkt_buf_enb   = 1'b1;

                    if (wcnt == 7'd1) begin
                        set_last_cyc      = 1'b1;
                        rd_axi_wchnl_fifo = 1'b1;
                        axi_wchnl_ns      = 2'd0;
                    end
                end  // if (m_axi_wready_i)       
            end  // 2'd2

            default: begin
                axi_wchnl_ns = 2'dx;
            end  // default     
        endcase  // case (axi_wchnl_cs) 
    end  // always @(*)

    always @(posedge clk)
        if (reset) axi_wchnl_cs <= 2'b00;
        else axi_wchnl_cs <= axi_wchnl_ns;

    // last_cyc / set_last_cyc 标记“当前这个包是否进入最后一个 beat 的发送阶段”
    always @(posedge clk)
        if (reset) last_cyc <= 1'b0;
        else if (set_last_cyc || m_axi_wready_i) last_cyc <= set_last_cyc;

    // first_cyc / set_first_cyc 标记“当前是否是该包首拍”，主要给特殊数据拼接用(MAD 包)
    always @(posedge clk)
        if (reset) first_cyc <= 1'b0;
        else if (set_first_cyc || m_axi_wready_i) first_cyc <= set_first_cyc;

    always @(posedge clk)
        if (reset) current_mad_pkt_ip_ver <= IPv6;
        else current_mad_pkt_ip_ver <= mad_pkt_ip_ver;

    always @(posedge clk) begin
        if (reset) begin
            single_beat_only      <= 1'b0;
            ieth_or_immdt_present <= 1'b0;
        end else begin
            if (set_single_beat_only || m_axi_wready_i) single_beat_only <= set_single_beat_only;

            if (set_ieth_or_immdt_present || m_axi_wready_i) ieth_or_immdt_present <= set_ieth_or_immdt_present;
        end
    end

    always @(posedge clk) wcnt <= nxt_wcnt;

    always @(posedge clk)
        if (req_pkt_buf_enb) prev_req_pkt_buf_addrb <= req_pkt_buf_addrb;
        else if (axi_mstr_idle) prev_req_pkt_buf_addrb <= req_pkt_buf_addra;

    always @(posedge clk)
        if (cpy_imm_data) prev_req_pkt_buf_doutb <= {wchnl_imm_data[31:0], req_pkt_buf_doutb[479:0]};  // check once again
        else if (req_pkt_buf_enb) prev_req_pkt_buf_doutb <= req_pkt_buf_doutb;

    assign wchnl_req_pkt_type_to_data_mux = last_cyc ? wchnl_req_pkt_type_r : wchnl_req_pkt_type;

    // 有效载荷拼接
    // INVALID payload is don't-care. Do not gate the wide packet-data mux
    // with VALID: doing so adds a LUT level on BRAM-to-output data paths.
    // The existing elastic slice accepts only VALID beats and holds stalls.
    always @(*) begin
        m_axi_wdata_i = 512'd0;
        case (wchnl_req_pkt_type_to_data_mux)
            REQ_PKT_TYPE_ERROR: m_axi_wdata_i = {req_pkt_buf_doutb[60*8-1:0], prev_req_pkt_buf_doutb[64*8-1:60*8]};
            REQ_PKT_TYPE_BYPASS: m_axi_wdata_i = req_pkt_buf_doutb;
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
            REQ_PKT_TYPE_IPV4_WR_REQ, REQ_PKT_TYPE_IPV4_SEND:
            if (single_beat_only) begin
                if (ieth_or_immdt_present) begin
                    m_axi_wdata_i[6*8-1:0] = req_pkt_buf_doutb[64*8-1:58*8];
                end else begin
                    m_axi_wdata_i[10*8-1:0] = req_pkt_buf_doutb[64*8-1:54*8];
                end
            end else begin
                if (ieth_or_immdt_present) begin
                    m_axi_wdata_i = {req_pkt_buf_doutb[58*8-1:0], prev_req_pkt_buf_doutb[64*8-1:58*8]};
                end else begin
                    m_axi_wdata_i = {req_pkt_buf_doutb[54*8-1:0], prev_req_pkt_buf_doutb[64*8-1:54*8]};
                end
            end
            REQ_PKT_TYPE_IPV6_WR_REQ, REQ_PKT_TYPE_IPV6_SEND:
            if (single_beat_only) begin
                if (ieth_or_immdt_present) begin
                    m_axi_wdata_i[50*8-1:0] = req_pkt_buf_doutb[64*8-1:14*8];
                end else begin
                    m_axi_wdata_i[54*8-1:0] = req_pkt_buf_doutb[64*8-1:10*8];
                end
            end else begin
                if (ieth_or_immdt_present) begin
                    m_axi_wdata_i = {req_pkt_buf_doutb[14*8-1:0], prev_req_pkt_buf_doutb[64*8-1:14*8]};
                end else begin
                    m_axi_wdata_i = {req_pkt_buf_doutb[10*8-1:0], prev_req_pkt_buf_doutb[64*8-1:10*8]};
                end
            end
            REQ_DB_UPDATE: m_axi_wdata_i = {{496{1'b0}}, wchnl_imm_data[15:0]} << {wchnl_lshift_by, {5{1'b0}}};
            IN_ERR_STS: m_axi_wdata_i = {{480{1'b0}}, wchnl_imm_data[31:0]} << {wchnl_lshift_by, {5{1'b0}}};
            REQ_PKT_TYPE_IPV4_WR_REQ_RETH:
            if (single_beat_only) begin
                if (ieth_or_immdt_present) begin
                    m_axi_wdata_i[54*8-1:0] = req_pkt_buf_doutb[64*8-1:10*8];
                end else begin
                    m_axi_wdata_i[58*8-1:0] = req_pkt_buf_doutb[64*8-1:6*8];
                end
            end else begin
                if (ieth_or_immdt_present) begin
                    m_axi_wdata_i = {req_pkt_buf_doutb[10*8-1:0], prev_req_pkt_buf_doutb[64*8-1:10*8]};
                end else begin
                    m_axi_wdata_i = {req_pkt_buf_doutb[6*8-1:0], prev_req_pkt_buf_doutb[64*8-1:6*8]};
                end
            end
            REQ_PKT_TYPE_IPV6_WR_REQ_RETH:
            if (single_beat_only) begin
                if (ieth_or_immdt_present) begin
                    m_axi_wdata_i[34*8-1:0] = req_pkt_buf_doutb[64*8-1:30*8];
                end else begin
                    m_axi_wdata_i[38*8-1:0] = req_pkt_buf_doutb[64*8-1:26*8];
                end
            end else begin
                if (ieth_or_immdt_present) begin
                    m_axi_wdata_i = {req_pkt_buf_doutb[30*8-1:0], prev_req_pkt_buf_doutb[64*8-1:30*8]};
                end else begin
                    m_axi_wdata_i = {req_pkt_buf_doutb[26*8-1:0], prev_req_pkt_buf_doutb[64*8-1:26*8]};
                end
            end
            default: m_axi_wdata_i = 512'd0;
        endcase
    end

    generate
        if (C_EN_IETH_IMMDT == 1) begin

            wire ieth_immdt_fifo_rd_en;
            wire ieth_immdt_fifo_empty;

            assign ieth_immdt_fifo_rd_en  = ieth_immdt_axis_tvalid && ieth_immdt_axis_trdy;
            assign ieth_immdt_axis_tlast  = 1'b1;
            assign ieth_immdt_axis_tvalid = !ieth_immdt_fifo_empty;

            //=================================
            // IETH/IMMDT FIFO
            //=================================
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE   ("auto"),                          //string; "auto", "block", "distributed", or "ultra";									
                .ECC_MODE           ("no_ecc"),                        //string; "no_ecc" or "en_ecc";									
                .FIFO_WRITE_DEPTH   (POST_AXI_TNFR_FIFO_DEPTH),        //positive integer									
                .WRITE_DATA_WIDTH   (64),                              //positive integer									
                .WR_DATA_COUNT_WIDTH(POST_AXI_TNFR_DATA_COUNT_WIDTH),  //positive integer									
                .PROG_FULL_THRESH   (POST_AXI_TNFR_FIFO_DEPTH - 8),    //positive integer									
                .FULL_RESET_VALUE   (0),                               //positive integer; 0 or 1									
                .READ_MODE          ("fwft"),                          //string; "std" or "fwft";									
                .FIFO_READ_LATENCY  (0),                               //positive integer;									
                .READ_DATA_WIDTH    (64),                              //positive integer									
                .RD_DATA_COUNT_WIDTH(POST_AXI_TNFR_DATA_COUNT_WIDTH),  //positive integer									
                .PROG_EMPTY_THRESH  (10),                              //positive integer									
                .DOUT_RESET_VALUE   ("0"),                             //string									
                .WAKEUP_TIME        (0)                                //positive integer; 0 or 2;	  
            ) ieth_immdt_fifo (
                .sleep        (1'b0),
                .rst          (reset),
                .wr_clk       (clk),
                .wr_en        (ieth_immdt_fifo_wr_en),
                .din          (ieth_immdt_fifo_wr_data),
                .full         (),
                .prog_full    (ieth_immdt_fifo_full),
                .wr_data_count(),
                .overflow     (),
                .wr_rst_busy  (),
                .rd_en        (ieth_immdt_fifo_rd_en),
                .dout         (ieth_immdt_axis_tdata),
                .empty        (ieth_immdt_fifo_empty),
                .prog_empty   (),
                .rd_data_count(),
                .underflow    (),
                .rd_rst_busy  (),
                .injectsbiterr(1'b0),
                .injectdbiterr(1'b0),
                .sbiterr      (),
                .dbiterr      ()
            );

        end else begin
            assign ieth_immdt_axis_tvalid = 1'b0;
            assign ieth_immdt_axis_tlast  = 1'b0;
            assign ieth_immdt_axis_tdata  = 64'd0;
            assign ieth_immdt_fifo_full   = 1'b0;
        end
    endgenerate
    reg_slice #(
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
            wchnl_pkt_id_r                <= 32'd0;
            wchnl_req_pkt_type_r          <= 4'h0;
            wchnl_imm_data_r              <= 44'd00;
            wchnl_req_pkt_immdt_present_r <= 1'b0;
            wchnl_req_pkt_ieth_present_r  <= 1'b0;
        end else begin
            if (rd_axi_wchnl_fifo) begin
                wchnl_pkt_id_r                <= wchnl_pkt_id;
                wchnl_req_pkt_type_r          <= wchnl_req_pkt_type;
                wchnl_imm_data_r              <= wchnl_imm_data;
                wchnl_req_pkt_immdt_present_r <= wchnl_req_pkt_immdt_present;
                wchnl_req_pkt_ieth_present_r  <= wchnl_req_pkt_ieth_present;
            end
        end

    generate
        if (C_SIM_DEBUG)
            assign post_axi_tnfr_fifo_din = last_cyc ? {wchnl_pkt_id_r,  // 32
                free_run_cnt[9:0],  // 10
                wchnl_req_pkt_type_r,  // 4
                wchnl_imm_data_r  // 44
                } : {wchnl_pkt_id,  // 32
                free_run_cnt[9:0],  // 10
                wchnl_req_pkt_type,  // 4  
                wchnl_imm_data  // 44      
                };
        else  // current: C_SIM_DEBUG == 0
            assign post_axi_tnfr_fifo_din = last_cyc ? {free_run_cnt[9:0],  // 10
                wchnl_req_pkt_type_r,  // 4
                wchnl_imm_data_r  // 44
                } : {free_run_cnt[9:0],  // 10
                wchnl_req_pkt_type,  // 4 
                wchnl_imm_data  // 44      
                };
    endgenerate

    assign wr_post_axi_tnfr_fifo = (m_axi_wlast_i & m_axi_wready_i) | async_db_update;

    // =====================================================================
    // POST_AXI_TNFR_FIFO (FWFT)
    // post_axi_tnfr_fifo 是“写完成事件队列”，确保只有在写事务真正完成后，才执行门铃/统计/中断这些副作用逻辑
    // =====================================================================

    reg  [POST_AXI_TNFR_DATA_COUNT_WIDTH-1:0] post_axi_tnfr_fifo_wrptr;
    reg  [POST_AXI_TNFR_DATA_COUNT_WIDTH-1:0] post_axi_tnfr_fifo_rdptr;
    wire [POST_AXI_TNFR_DATA_COUNT_WIDTH-1:0] nxt_post_axi_tnfr_fifo_wrptr;

    xpm_fifo_sync_wrap #(
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
        .dbiterr      ()
    );

    // =====================================================================  

    generate
        if (C_SIM_DEBUG) assign pkt_id_post_tnfr = post_axi_tnfr_fifo_dout[89:58];
        else assign pkt_id_post_tnfr = 32'd0;
    endgenerate

    assign free_run_cnt_at_wlast  = post_axi_tnfr_fifo_dout[57:48];

    assign req_pkt_type_post_tnfr = post_axi_tnfr_fifo_dout[47:44];
    assign imm_data_post_tnfr     = post_axi_tnfr_fifo_dout[43:0];

    always @(posedge clk)
        if (reset) begin
            m_axi_bvalid_d <= 1'b0;
        end else begin
            // Retain an accepted B until its corresponding post event can
            // retire. DB FIFO may become full one cycle after B acceptance.
            // A one-cycle pulse here loses the completion under that race.
            m_axi_bvalid_d <= (m_axi_bvalid_d && !rd_post_axi_tnfr_fifo_1) ||
                              (m_axi_bvalid && m_axi_bready);
        end

    assign rd_post_axi_tnfr_fifo_1 = (!post_axi_tnfr_fifo_empty && !db_fifo_full && (req_pkt_type_post_tnfr != REQ_PKT_TYPE_DISCARD) && m_axi_bvalid_d) ? 1'b1 : 1'b0;
    assign rd_post_axi_tnfr_fifo_2 = (!post_axi_tnfr_fifo_empty && !db_fifo_full && (req_pkt_type_post_tnfr == REQ_PKT_TYPE_DISCARD)) ? 1'b1 : 1'b0;
    assign rd_post_axi_tnfr_fifo   = rd_post_axi_tnfr_fifo_1 | rd_post_axi_tnfr_fifo_2;

    assign m_axi_bready            = !db_fifo_full && !post_axi_tnfr_fifo_empty &&
                                    (req_pkt_type_post_tnfr != REQ_PKT_TYPE_DISCARD) &&
                                    (!m_axi_bvalid_d || rd_post_axi_tnfr_fifo_1);

    //  assign db_ptr_buf_addra = imm_data_post_tnfr[16+:C_QP_INDX_WIDTH];
    //  assign db_ptr_buf_dina = imm_data_post_tnfr[15:0];
    //
    //  xpm_memory_sdpram # (  // Common module parameters
    //    .MEMORY_SIZE        (C_NUM_QP * 16), //positive integer
    //    .MEMORY_PRIMITIVE   ("auto"),                 //string; "auto", "distributed", "block" or "ultra";
    //    .CLOCKING_MODE      ("common_clock"),         //string; "common_clock", "independent_clock" 
    //    .ECC_MODE           ("no_ecc"),               //do not change
    //    .MEMORY_INIT_FILE   ("none"),                 //string; "none" or "<filename>.mem" 
    //    .MEMORY_INIT_PARAM  (""    ),                 //string;
    //    .WAKEUP_TIME        ("disable_sleep"),        //string; "disable_sleep" or "use_sleep_pin" 
    //    .MESSAGE_CONTROL    (0),                      
    //                                                  
    //    // Port A module parameters                   
    //    .WRITE_DATA_WIDTH_A (16),                    //positive integer
    //    .BYTE_WRITE_WIDTH_A (8),                      //integer; 8, 9, or WRITE_DATA_WIDTH_A value
    //    .ADDR_WIDTH_A       (C_QP_INDX_WIDTH),   //positive integer
    //                                                  
    //    // Port B module parameters                   
    //    .READ_DATA_WIDTH_B  (16),                    //positive integer
    //    .ADDR_WIDTH_B       (C_QP_INDX_WIDTH),   //positive integer
    //    .READ_RESET_VALUE_B ("0"),                    //string
    //    .READ_LATENCY_B     (1),                      //non-negative integer
    //    .WRITE_MODE_B       ("read_first")            //string; "write_first", "read_first", "no_change" 
    //  ) db_ptr_buf (
    //    // Common module ports
    //    .sleep          (1'b0),  //do not change
    //  
    //    // Port A module ports
    //    .clka           (clk),
    //    .ena            (db_ptr_buf_we),
    //    .wea            (db_ptr_buf_we),
    //    .addra          (db_ptr_buf_addra),
    //    .dina           (db_ptr_buf_dina),
    //    .injectsbiterra (1'b0),  //do not change
    //    .injectdbiterra (1'b0),  //do not change
    //  
    //    // Port B module ports
    //    .clkb           (clk),
    //    .rstb           (1'b0),
    //    .enb            (i_hw_hndshk_disable_to_0),
    //    .regceb         (1'b1),  
    //    .addrb          (i_connect_io_qpid),
    //    .doutb          (db_ptr_buf_doutb),
    //    .sbiterrb       (),      //do not change
    //    .dbiterrb       ()       //do not change
    //  );

    always @(*) begin
        // default values
        incr_inv_pkt_cnt         = 1'b0;
        set_pkt_valdn_err_intr   = 1'b0;
        incr_bypass_pkt_cnt      = 1'b0;
        set_bypass_pkt_rcvd_intr = 1'b0;
        wr_db_fifo               = 1'b0;
        incr_send_pkt_cnt_1      = 1'b0;
        incr_send_req_cnt_1      = 1'b0;
        set_qp_req_rcvd_intr_1   = 1'b0;
        incr_mad_pkt_cnt_1       = 1'b0;
        set_mad_pkt_rcvd_intr_1  = 1'b0;
        set_qp_fatal_intr        = 1'b0;
        incr_in_err_sts_q_wrptr  = 1'b0;
        cnct_io_db_update        = 1'b0;
        //db_ptr_buf_we <= 1'b0;    

        if (rd_post_axi_tnfr_fifo) begin
            case (req_pkt_type_post_tnfr)
                REQ_PKT_TYPE_DISCARD: begin
                    wr_db_fifo        = 1'b1;
                    cnct_io_db_update = 1'b1;
                end
                REQ_PKT_TYPE_ERROR: begin
                    incr_inv_pkt_cnt       = 1'b1;
                    set_pkt_valdn_err_intr = 1'b1;
                end
                REQ_PKT_TYPE_BYPASS: begin
                    incr_bypass_pkt_cnt      = 1'b1;
                    set_bypass_pkt_rcvd_intr = 1'b1;
                end
                REQ_PKT_TYPE_MAD: begin
                    if (imm_data_post_tnfr[43]) begin  // 若硬件握手使能
                        wr_db_fifo = 1'b1;  // 需要写 DB FIFO
                    end
                end

                REQ_PKT_TYPE_IPV4_WR_REQ, REQ_PKT_TYPE_IPV6_WR_REQ, REQ_PKT_TYPE_IPV4_WR_REQ_RETH, REQ_PKT_TYPE_IPV6_WR_REQ_RETH: begin
                    incr_send_pkt_cnt_1 = 1'b1;
                end

                REQ_PKT_TYPE_IPV4_SEND, REQ_PKT_TYPE_IPV6_SEND: begin
                    if (imm_data_post_tnfr[42]) begin  // SEND_ONLY or SEND_LAST
                        if (imm_data_post_tnfr[43]) begin  // HW-HNDSHK bit
                            wr_db_fifo = 1'b1;
                        end
                    end else begin
                        incr_send_pkt_cnt_1 = 1'b1;
                    end
                end

                REQ_DB_UPDATE: begin
                    if (imm_data_post_tnfr[16+:C_QP_INDX_WIDTH] == 'd1) begin  // MAD packet，QP1
                        incr_mad_pkt_cnt_1      = 1'b1;
                        set_mad_pkt_rcvd_intr_1 = 1'b1;
                    end else begin
                        set_qp_req_rcvd_intr_1 = imm_data_post_tnfr[41];
                        incr_send_pkt_cnt_1    = 1'b1;
                        incr_send_req_cnt_1    = 1'b1;
                        //db_ptr_buf_we <= 1'b1;
                    end
                end
                IN_ERR_STS: begin
                    set_qp_fatal_intr       = 1'b1;
                    incr_in_err_sts_q_wrptr = 1'b1;
                end
            endcase
        end
    end

    always @(posedge clk)
        if (reset) begin
            o_err_buf_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            o_err_buf_wrptr <= 16'd0;
        end else if (xrnic_en_pulse) begin
            o_err_buf_ca <= i_err_buf_ba;
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
            o_bypass_buf_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            o_bypass_buf_wrptr <= 16'd0;
        end else if (xrnic_en_pulse) begin
            o_bypass_buf_ca <= i_bypass_buf_ba;
        end else if (incr_bypass_pkt_cnt) begin
            if (o_bypass_buf_wrptr == i_bypass_num_bufs - 'd1) begin
                o_bypass_buf_ca    <= i_bypass_buf_ba;
                o_bypass_buf_wrptr <= 16'd0;
            end else begin
                o_bypass_buf_ca    <= o_bypass_buf_ca + i_bypass_buf_sz;
                o_bypass_buf_wrptr <= o_bypass_buf_wrptr + 'd1;
            end
        end

    always @(posedge clk)
        if (reset) begin
            o_in_err_sts_q_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            o_in_err_sts_q_wrptr <= 16'd0;
        end else if (xrnic_en_pulse) begin
            o_in_err_sts_q_ca <= i_in_err_sts_q_ba;
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
        if (reset) o_bypass_pkt_cnt <= 16'd0;
        else if (incr_bypass_pkt_cnt) o_bypass_pkt_cnt <= o_bypass_pkt_cnt + 'd1;

    always @(posedge clk)
        if (reset) o_send_pkt_cnt <= 16'd0;
        else if (incr_send_pkt_cnt_1 | incr_send_req_cnt_2) o_send_pkt_cnt <= o_send_pkt_cnt + incr_send_pkt_cnt_1 + incr_send_req_cnt_2;

    always @(posedge clk)
        if (reset) o_send_req_cnt <= 16'd0;
        else if (incr_send_req_cnt_1 | incr_send_req_cnt_2) o_send_req_cnt <= o_send_req_cnt + incr_send_req_cnt_1 + incr_send_req_cnt_2;

    always @(posedge clk)
        if (reset) o_mad_pkt_cnt <= 16'd0;
        else if (incr_mad_pkt_cnt_1 | incr_mad_pkt_cnt_2) o_mad_pkt_cnt <= o_mad_pkt_cnt + incr_mad_pkt_cnt_1 + incr_mad_pkt_cnt_2;

    assign intr_qp_indx_1        = imm_data_post_tnfr[16+:C_QP_INDX_WIDTH];

    assign set_mad_pkt_rcvd_intr = set_mad_pkt_rcvd_intr_1 | set_mad_pkt_rcvd_intr_2;

    generate
        if (C_SIM_DEBUG)
            assign db_fifo_din = {
                pkt_id_post_tnfr,
                cnct_io_db_update,
                imm_data_post_tnfr[41],  // interrupt bit
                imm_data_post_tnfr[27+:14],  // DB addr
                imm_data_post_tnfr[16+:C_QP_INDX_WIDTH],  // QP address
                imm_data_post_tnfr[0+:16]  // DB value
            };
        else  // current
            assign db_fifo_din = {
                cnct_io_db_update,
                imm_data_post_tnfr[41],  // interrupt bit
                imm_data_post_tnfr[27+:14],  // DB addr
                imm_data_post_tnfr[16+:C_QP_INDX_WIDTH],  // QP address(qp号)
                imm_data_post_tnfr[0+:16]  // DB value
            };
    endgenerate

    // =====================================================================
    // DB_FIFO (FWFT)
    // DB_FIFO 是“RQ DB 写命令队列”，负责把门铃写请求可靠地送到 RNIC 外部侧带接口(RQ DB)。
    // =====================================================================
    reg_fifo_sync #(
        .C_READ_MODE(1'b1),
        .C_FIFO_DEPTH(DB_FIFO_DEPTH),
        .C_FIFO_WRCNTR_WIDTH(DB_FIFO_DATA_COUNT_WIDTH),
        .C_FIFO_DATA_WIDTH(DB_FIFO_DATA_WIDTH)
    ) db_fifo (
        .clk        (clk),
        .reset      (reset),
        .wr_en      (wr_db_fifo),
        .din        (db_fifo_din),
        .full       (db_fifo_full),
        .almost_full(),
        .rd_en      (rd_db_fifo),
        .dout       (db_fifo_dout),
        .empty      (db_fifo_empty)
    );

    // =====================================================================  

    always @(posedge clk)
        if (reset) rq_wrptr_db_we_1 <= 1'b0;
        else rq_wrptr_db_we_1 <= rq_wrptr_db_we;

    always @(*) begin
        db_fsm_ns        = db_fsm_cs;
        rq_db_data_valid = 1'b0;
        rd_db_fifo       = 1'b0;
        case (db_fsm_cs)
            1'd0: begin
                //if (!db_fifo_empty && !(db_fifo_dout[C_QP_INDX_WIDTH+27] && rq_wrptr_db_we_1))
                if (!db_fifo_empty) begin
                    if (db_fifo_dout[15:0] != 16'd0) begin
                        rq_db_data_valid = 1'b1;
                        if (rq_db_data_ack) db_fsm_ns = 1'b1;
                    end else rd_db_fifo = 1'b1;  // 无效DB，直接弹出
                end
            end

            1'b1: begin
                db_fsm_ns        = 1'b0;
                rq_db_data_valid = 1'b0;
                rd_db_fifo       = 1'b1;
            end
        endcase
    end

    always @(posedge clk)
        if (reset) db_fsm_cs <= 1'b0;
        else db_fsm_cs <= db_fsm_ns;

    /* ***************
    Efficient implementation, but NVMOF does not support this
    ***************
    assign rd_db_fifo           = ~db_fifo_empty & rq_db_data_ack;
    assign rq_db_data_valid     = ~db_fifo_empty;
    */

    assign rq_db_data              = db_fifo_dout[15:0];  // req_imm_data[15:0] 16bit: rq_wrptr_db，即 RQ写 DB value(门铃值)
    assign rq_db_addr              = db_fifo_dout[(C_QP_INDX_WIDTH+16)+:13];  // req_imm_data[38:26] 13bit: rq_wrptr_db_addr，即 RQ写 DB addr(门铃地址)

    assign incr_send_req_cnt_2     = (db_fifo_dout[16+:C_QP_INDX_WIDTH] > 'd1) ? rd_db_fifo : 1'b0;
    assign set_qp_req_rcvd_intr_2  = incr_send_req_cnt_2 & db_fifo_dout[C_QP_INDX_WIDTH+30];
    assign incr_mad_pkt_cnt_2      = (db_fifo_dout[16+:C_QP_INDX_WIDTH] == 'd1) ? rd_db_fifo : 1'b0;
    assign set_mad_pkt_rcvd_intr_2 = incr_mad_pkt_cnt_2;
    assign intr_qp_indx_2          = db_fifo_dout[16+:C_QP_INDX_WIDTH];

    generate
        if (C_SIM_DEBUG) assign pkt_id_db = db_fifo_dout[(C_QP_INDX_WIDTH+32)+:32];
        else assign pkt_id_db = 32'd0;
    endgenerate

    // =====================================================================
    // Debug Logic
    // =====================================================================
    reg  [14:0] axi_awchnl_wr_cnt;
    reg  [14:0] axi_awchnl_rd_cnt;
    wire [15:0] axi_awchnl_fifo_occupied;
    wire [15:0] axi_awchnl_fifo_unoccupied;

    reg  [14:0] axi_wchnl_wr_cnt;
    reg  [14:0] axi_wchnl_rd_cnt;
    wire [15:0] axi_wchnl_fifo_occupied;
    wire [15:0] axi_wchnl_fifo_unoccupied;

    reg  [14:0] post_axi_tnfr_wr_cnt;
    reg  [14:0] post_axi_tnfr_rd_cnt;
    wire [15:0] post_axi_tnfr_fifo_occupied;
    wire [15:0] post_axi_tnfr_fifo_unoccupied;

    reg  [14:0] db_wr_cnt;
    reg  [14:0] db_rd_cnt;
    wire [15:0] db_fifo_occupied;
    wire [15:0] db_fifo_unoccupied;

    reg  [15:0] axi_awchnl_wait_st_cnt;
    reg  [15:0] axi_wchnl_wait_st_cnt;

    always @(posedge clk)
        if (reset) axi_awchnl_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) axi_awchnl_wr_cnt <= 15'd0;
        else if (wr_axi_awchnl_fifo) axi_awchnl_wr_cnt <= axi_awchnl_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) axi_awchnl_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) axi_awchnl_rd_cnt <= 15'd0;
        else if (rd_axi_awchnl_fifo_raw) axi_awchnl_rd_cnt <= axi_awchnl_rd_cnt + 'd1;

    assign axi_awchnl_fifo_occupied   = (axi_awchnl_rd_cnt > axi_awchnl_wr_cnt) ? ({1'b1, axi_awchnl_wr_cnt} - axi_awchnl_rd_cnt) : (axi_awchnl_wr_cnt - axi_awchnl_rd_cnt);

    assign axi_awchnl_fifo_unoccupied = AWCHNL_FIFO_DEPTH - axi_awchnl_fifo_occupied;

    always @(posedge clk)
        if (reset) axi_awchnl_fifo_unoccupied_r <= AWCHNL_FIFO_DEPTH;
        else if (i_clr_debug_sts) axi_awchnl_fifo_unoccupied_r <= AWCHNL_FIFO_DEPTH;
        else if (axi_awchnl_fifo_unoccupied_r > axi_awchnl_fifo_unoccupied) axi_awchnl_fifo_unoccupied_r <= axi_awchnl_fifo_unoccupied;

    always @(posedge clk)
        if (reset) axi_awchnl_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) axi_awchnl_fifo_full_sticky <= 1'b0;
        else if (axi_awchnl_fifo_full) axi_awchnl_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) axi_wchnl_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) axi_wchnl_wr_cnt <= 15'd0;
        else if (wr_axi_wchnl_fifo) axi_wchnl_wr_cnt <= axi_wchnl_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) axi_wchnl_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) axi_wchnl_rd_cnt <= 15'd0;
        else if (rd_axi_wchnl_fifo_raw) axi_wchnl_rd_cnt <= axi_wchnl_rd_cnt + 'd1;

    assign axi_wchnl_fifo_occupied   = (axi_wchnl_rd_cnt > axi_wchnl_wr_cnt) ? ({1'b1, axi_wchnl_wr_cnt} - axi_wchnl_rd_cnt) : (axi_wchnl_wr_cnt - axi_wchnl_rd_cnt);

    assign axi_wchnl_fifo_unoccupied = WCHNL_FIFO_DEPTH - axi_wchnl_fifo_occupied;

    always @(posedge clk)
        if (reset) axi_wchnl_fifo_unoccupied_r <= WCHNL_FIFO_DEPTH;
        else if (i_clr_debug_sts) axi_wchnl_fifo_unoccupied_r <= WCHNL_FIFO_DEPTH;
        else if (axi_wchnl_fifo_unoccupied_r > axi_wchnl_fifo_unoccupied) axi_wchnl_fifo_unoccupied_r <= axi_wchnl_fifo_unoccupied;

    always @(posedge clk)
        if (reset) axi_wchnl_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) axi_wchnl_fifo_full_sticky <= 1'b0;
        else if (axi_wchnl_fifo_full) axi_wchnl_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) post_axi_tnfr_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) post_axi_tnfr_wr_cnt <= 15'd0;
        else if (wr_post_axi_tnfr_fifo) post_axi_tnfr_wr_cnt <= post_axi_tnfr_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) post_axi_tnfr_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) post_axi_tnfr_rd_cnt <= 15'd0;
        else if (rd_post_axi_tnfr_fifo) post_axi_tnfr_rd_cnt <= post_axi_tnfr_rd_cnt + 'd1;

    assign post_axi_tnfr_fifo_occupied   = (post_axi_tnfr_rd_cnt > post_axi_tnfr_wr_cnt) ? ({1'b1, post_axi_tnfr_wr_cnt} - post_axi_tnfr_rd_cnt) : (post_axi_tnfr_wr_cnt - post_axi_tnfr_rd_cnt);

    assign post_axi_tnfr_fifo_unoccupied = POST_AXI_TNFR_FIFO_DEPTH - post_axi_tnfr_fifo_occupied;

    always @(posedge clk)
        if (reset) post_axi_tnfr_fifo_unoccupied_r <= POST_AXI_TNFR_FIFO_DEPTH;
        else if (i_clr_debug_sts) post_axi_tnfr_fifo_unoccupied_r <= POST_AXI_TNFR_FIFO_DEPTH;
        else if (post_axi_tnfr_fifo_unoccupied_r > post_axi_tnfr_fifo_unoccupied) post_axi_tnfr_fifo_unoccupied_r <= post_axi_tnfr_fifo_unoccupied;

    always @(posedge clk)
        if (reset) post_axi_tnfr_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) post_axi_tnfr_fifo_full_sticky <= 1'b0;
        else if (post_axi_tnfr_fifo_full) post_axi_tnfr_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) db_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) db_wr_cnt <= 15'd0;
        else if (wr_db_fifo) db_wr_cnt <= db_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) db_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) db_rd_cnt <= 15'd0;
        else if (rd_db_fifo) db_rd_cnt <= db_rd_cnt + 'd1;

    assign db_fifo_occupied   = (db_rd_cnt > db_wr_cnt) ? ({1'b1, db_wr_cnt} - db_rd_cnt) : (db_wr_cnt - db_rd_cnt);

    assign db_fifo_unoccupied = DB_FIFO_DEPTH - db_fifo_occupied;

    always @(posedge clk)
        if (reset) db_fifo_unoccupied_r <= DB_FIFO_DEPTH;
        else if (i_clr_debug_sts) db_fifo_unoccupied_r <= DB_FIFO_DEPTH;
        else if (db_fifo_unoccupied_r > db_fifo_unoccupied) db_fifo_unoccupied_r <= db_fifo_unoccupied;

    always @(posedge clk)
        if (reset) db_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) db_fifo_full_sticky <= 1'b0;
        else if (db_fifo_full) db_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) begin
            axi_awchnl_wait_st_cnt <= 16'd0;
            axi_awchnl_max_wait_st <= 16'd0;
        end else if (i_clr_debug_sts) begin
            axi_awchnl_wait_st_cnt <= 16'd0;
            axi_awchnl_max_wait_st <= 16'd0;
        end else if (m_axi_awvalid) begin
            if (m_axi_awready) begin
                axi_awchnl_wait_st_cnt <= 16'd0;
                if (axi_awchnl_max_wait_st < axi_awchnl_wait_st_cnt) axi_awchnl_max_wait_st <= axi_awchnl_wait_st_cnt;
            end else axi_awchnl_wait_st_cnt <= axi_awchnl_wait_st_cnt + 'd1;
        end

    always @(posedge clk)
        if (reset) begin
            axi_wchnl_wait_st_cnt <= 16'd0;
            axi_wchnl_max_wait_st <= 16'd0;
        end else if (i_clr_debug_sts) begin
            axi_wchnl_wait_st_cnt <= 16'd0;
            axi_wchnl_max_wait_st <= 16'd0;
        end else if (m_axi_wvalid) begin
            if (m_axi_wready) begin
                axi_wchnl_wait_st_cnt <= 16'd0;
                if (axi_wchnl_max_wait_st < axi_wchnl_wait_st_cnt) axi_wchnl_max_wait_st <= axi_wchnl_wait_st_cnt;
            end else axi_wchnl_wait_st_cnt <= axi_wchnl_wait_st_cnt + 'd1;
        end

    always @(posedge clk)
        if (reset) axi_bresp_max_wait_st <= 16'd0;
        else if (i_clr_debug_sts) axi_bresp_max_wait_st <= 16'd0;
        else if (rd_post_axi_tnfr_fifo_1)
            if (free_run_cnt[9:0] > free_run_cnt_at_wlast) begin
                if (axi_bresp_max_wait_st < (free_run_cnt[9:0] - free_run_cnt_at_wlast)) axi_bresp_max_wait_st <= free_run_cnt[9:0] - free_run_cnt_at_wlast;
            end else begin
                if (axi_bresp_max_wait_st < ({1'b1, free_run_cnt[9:0]} - free_run_cnt_at_wlast)) axi_bresp_max_wait_st <= {1'b1, free_run_cnt[9:0]} - free_run_cnt_at_wlast;
            end

endmodule
