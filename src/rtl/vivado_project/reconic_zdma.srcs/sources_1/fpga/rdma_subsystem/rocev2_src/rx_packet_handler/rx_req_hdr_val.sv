////-----------------------------------------------------------------------------
//// rx_req_hdr_val.v
////-----------------------------------------------------------------------------
////
//// Filename        : rx_req_hdr_val.v
//// Version         : v1.0
//// Description     : 该模块处理接收到的请求数据包
//// Verilog-Standard: Verilog'2001
////-----------------------------------------------------------------------------
//
/////////////////////////////////////////////////////////////////////////////////
//// Module Declaration
/////////////////////////////////////////////////////////////////////////////////
// Pipeline:
//  - C1：以太网/IP/UDP/BTH头部 解析，提取 QP/PD/PSN 等关键字段并做基础检查。
`timescale 1 ps / 1 ps

module rx_req_hdr_val_sv #(
    parameter C_SIM_DEBUG              = 0,    // 0
    parameter C_EN_INITIATOR_LITE      = 0,    // 1
    parameter C_M_AXI_ADDR_WIDTH       = 32,   // 32
    parameter C_QP_INDX_WIDTH          = 8,    // 5
    parameter C_PD_INDX_WIDTH          = 8,    // 11
    parameter C_NUM_QP                 = 256,  // 32
    parameter C_NUM_PD                 = 256,  // 2048
    parameter PKT_DESC_FIFO_DATA_WIDTH = 26,   // 包描述符FIFO数据位宽，27
    parameter C_VIRT_ADDR_WIDTH        = 64,   // 虚拟地址位宽，64
    parameter C_HDR_BUF_ADDR_WIDTH     = 9,    // 包头缓冲区地址位宽，9
    parameter C_REQ_BUF_ADDR_WIDTH     = 9,    // 请求缓冲区地址位宽，9
    parameter C_RSP_BUF_ADDR_WIDTH     = 10,   // 响应缓冲区地址位宽，10
    parameter C_EN_IETH_IMMDT          = 1     // 支持IETH/立即数，1
) (
    // 系统级接口
    input logic clk,
    input logic reset,

    // =========================== 请求包包描述符FIFO接口 ===========================
    // req_pkt_desc_fifo 接口（该FIFO在cmac_rx_intf模块中）
    input  logic [PKT_DESC_FIFO_DATA_WIDTH-1:0] req_pkt_desc_fifo_dout,   // 包描述符数据
    input  logic                                req_pkt_desc_fifo_empty,
    output logic                                rd_req_pkt_desc_fifo,     // 读使能

    // =========================== 请求包包头缓冲区接口--端口B ===========================
    // req_hdr_buf 端口B 接口（该BRAM在rx_pkt_handler模块中）
    output logic [C_HDR_BUF_ADDR_WIDTH-1:0] req_hdr_buf_addrb,
    output logic                            req_hdr_buf_enb,
    input  logic [                   511:0] req_hdr_buf_doutb,
    // =========================== 请求包 QP ID缓冲区接口--端口B ===========================
    input  logic [                    31:0] req_hdr_qpid_buf_doutb,  // QP ID缓冲区，格式为{bth_opcode字段, Dest QP字段/qpid}
    input  logic [                    31:0] req_hdr_rkey_buf_doutb,  // rkey字段 缓冲区

    // XRNIC 寄存器接口
    input logic                       i_bypass_en,       // XRNIC Configuration寄存器[2]: Reserved
    input logic                       i_err_buf_en,      // 错误缓冲区使能
    input logic [               47:0] i_mac_node_addr,   // XRNIC MAC地址
    input logic [               31:0] i_ipv4_node_addr,  // XRNIC IPv4地址
    input logic [              127:0] i_ipv6_node_addr,  // XRNIC IPv6地址
    input logic [C_QP_INDX_WIDTH-1:0] i_num_qp_enabled,  // 活动的 QP 数量
    input logic [                4:0] i_rnr_nak_tval,    // TIMEOUTCONFi[20:16]: RNR NACK 传出数据包的超时值
    input logic [                1:0] i_flow_credits,    // XRNIC Configuration寄存器[7:6]: Reserved

    output logic [15:0] o_inc_dup_pkt_cnt,  // 重复包计数器
    output logic [15:0] o_out_nack_pkt_cnt, // 输出NAK包计数器

    output logic [ 7:0] o_last_in_req_pkt_opcode,  // 最后一个请求包的操作码
    output logic [ 7:0] o_last_in_req_pkt_qpid,    // 最后一个请求包的QP ID
    output logic [15:0] o_last_in_req_pkt_psn,     // 最后一个请求包的PSN

    //output logic  [C_NUM_QP-1:0]                  o_rq_full,
    output logic [C_QP_INDX_WIDTH-1:0] o_rq_full_idx,
    output logic                       o_rq_full_wr_en,
    output logic                       o_rq_full_val,
    output logic [C_QP_INDX_WIDTH-1:0] o_rq_full_reset_idx,
    output logic                       o_rq_full_reset_wr_en,
    output logic                       o_rq_full_reset_val,


    // Response Handler Interface -3
    // 响应处理程序接口 -3
    output logic [                1:0] o_req_indicator,               // 0-默认, 1-读请求, 2-重复读请求
    output logic [               63:0] o_physical_addr,               // READ REQ 收到的 Phy 地址
    output logic [               31:0] o_dma_len,                     // 读请求中的 DMA 长度
    output logic [               12:0] o_rd_req_pmtu,                 // 读请求的 PMTU
    output logic [               23:0] o_rsp_last_psn,                // 最后读请求的 PSN
    //input  logic  [C_NUM_QP-1:0]                 i_rd_req_fifo_full,
    // 读请求应答队列信号组（来自外部(wqe_proc)的“置位/清位事件”输入）
    // “读请求应答队列”：把收到的 RDMA Read Request 先排队，等待后端慢慢生成 Read Response 的缓冲队列
    // i_rd_req_fifo_full_set=1   -> 将 i_rd_req_fifo_full_set_idx 对应 QP 的 full 位置 1
    input  logic [C_QP_INDX_WIDTH-1:0] i_rd_req_fifo_full_set_idx,
    input  logic                       i_rd_req_fifo_full_set,
    // i_rd_req_fifo_full_reset=1 -> 将 i_rd_req_fifo_full_reset_idx 对应 QP 的 full 位清 0
    input  logic [C_QP_INDX_WIDTH-1:0] i_rd_req_fifo_full_reset_idx,
    input  logic                       i_rd_req_fifo_full_reset,

    // Intiator 的 PD 配置
    input logic [23:0] i_pd_pd_num,  // 24 位保护域编号。
    input logic [31:0] i_pd_virt_addr_0,  // 已分配缓冲区的虚拟地址（LSB）。
    input logic [31:0] i_pd_virt_addr_1,  // 已分配缓冲区的虚拟地址（MSB）。
    input logic [31:0] i_pd_buf_base_addr_0,  // 已分配缓冲区的物理地址（LSB）。
    input logic [31:0] i_pd_buf_base_addr_1,  // 已分配缓冲区的物理地址（MSB）。
    input logic [7:0] i_pd_buf_r_key,  // r_key
    input logic [47:0] i_pd_w_r_buf_len,  // 已分配写/读缓冲区的大小，也就是给 QP（准确说 QP 绑定的 PD）可用的那块内存空间
    input logic [3:0] i_pd_access_type,  /* 缓冲区访问类型。
                                            4-bit pd_access_type:
                                            -- 4'b0000: 只读
                                            -- 4'b0001: 只写
                                            -- 4'b0010: 读写
                                            -- Other values: Not supported */

    // QP configuration
    // QP 配置
    output logic [C_QP_INDX_WIDTH-1:0] o_qp_conf_idx,  // 输出 QP索引 给Intiator，并拉高qp_conf_en，返回相应索引的QP信息(例如：o_qp_conf_idx=1，返回的 i_rq_wrptr_db_addr、i_rq_wrptr_db 等就是qp2的信息)
    output logic [C_PD_INDX_WIDTH-1:0] o_pd_conf_idx,  // 输出PD索引给Intiator，并拉高pd_conf_en，就会返回相应PD配置；o_pd_conf_idx = reth_r_key_w[18:8], pd_number = reth_r_key_w[31:8]

    input  logic [31:0] i_qp_conf,    // QP配置数据
    input  logic        qp_conf_vld,  // qp_conf_en 的值延迟一个周期 -- MSFF_R(qp_conf_vld, qp_conf_en, clk, rst_p)，因为这些配置数据均在en下一个周期后才有数据
    output logic        qp_conf_en,   // qp配置读取使能

    output logic pd_conf_en,  // pd配置读取使能；pd_conf_en = has_reth：头部若是reth头部，则会读取相应pd配置

    input  logic [C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_ba,
    output logic                          rq_buf_ba_en,

    input  logic [C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_ca,
    output logic [C_M_AXI_ADDR_WIDTH-9:0] o_rq_buf_ca,   // rq当前地址
    output logic                          rq_buf_ca_en,
    output logic                          rq_buf_ca_we,

    // 用户配置，并使用该地址用于rq硬件握手
    input  logic [C_M_AXI_ADDR_WIDTH-1:0] i_rq_wrptr_db_addr,  // RQ 写门铃地址寄存器，该寄存器提供接收队列门铃寄存器的地址。在接收到新的 RDMA SEND 数据包时，XRNIC IP 会更新寄存器所指向地址中的 RQ 门铃值。
    output logic                          rq_wrptr_db_addr_en,

    input  logic [15:0] i_q_depth,
    output logic        q_depth_en,

    input  logic [15:0] i_rq_ci_db,
    output logic        rq_ci_db_en, // rq_ci_db读取使能

    input  logic [31:0] i_last_rq_req,
    output logic [31:0] o_last_rq_req,
    output logic        last_rq_req_en,
    output logic        last_rq_req_we,

    input  logic [23:0] i_stat_qp_msn,
    output logic [23:0] o_stat_qp_msn,
    output logic        stat_qp_msn_en,
    output logic        stat_qp_msn_we,

    input  logic [47:0] i_remote_qp_mac_addr,
    output logic        remote_qp_mac_addr_en,

    input  logic [127:0] i_remote_qp_ip_addr,
    output logic         remote_qp_ip_addr_en,

    input  logic [15:0] i_rq_wrptr_db,
    output logic [15:0] o_rq_wrptr_db,
    output logic        rq_wrptr_db_en,
    output logic        rq_wrptr_db_we,

    input  logic [23:0] i_qp_pd_num,
    output logic        qp_pd_num_en,

    //input  logic [C_NUM_QP-1:0]                   i_qp_fatal,
    input  logic                       i_qp_fatal_wr_en,
    input  logic [C_QP_INDX_WIDTH-1:0] i_qp_fatal_wr_idx,
    input  logic                       i_qp_fatal_wr_data,
    output logic [C_QP_INDX_WIDTH-1:0] o_qp_fatal_idx,
    output logic                       set_qp_fatal,        // qp致命错误标志

    input logic [C_QP_INDX_WIDTH-1:0] rsp_qp_fatal_idx,
    input logic                       set_rsp_qp_fatal,

    output logic set_rnr_nack_gen_intr,

    input  logic [C_QP_INDX_WIDTH-1:0] i_qp_fatal_wqe_proc,      // fatal qp的id
    input  logic                       i_qp_fatal_wqe_proc_vld,
    output logic                       o_qp_fatal_wqe_proc_rdy,  // 直连 qp_fatal_wqe_proc_rdy

    input  logic [C_QP_INDX_WIDTH-1:0] i_qp_fatal_rsp_hndl,      // fatal qp的id
    input  logic                       i_qp_fatal_rsp_hndl_vld,
    output logic                       o_qp_fatal_rsp_hndl_rdy,  // 直连 qp_fatal_rsp_hndl_rdy

    // WQE Processor 接口
    // 作用：对请求发送响应
    // ob(outbound)：向外发响应，ob_* 不是给 rx_req_axi_mstr 写内存用的，而是给“响应发送路径”准备 ACK/NAK 和读响应任务上下文。
    output logic [C_QP_INDX_WIDTH-1:0] o_ob_rsp_qpid,           // 响应数据包的目标 QP ID
    output logic [               23:0] o_ob_rsp_psn,            // 在响应数据包中嵌入 BTH.PSN
    output logic [                7:0] o_ob_rsp_aeth_syndrome,  // 在响应数据包中嵌入 AETH.Syndrome
    output logic [               23:0] o_ob_rsp_aeth_msn,       // 在响应数据包中嵌入 AETH.MSN
    output logic                       o_ob_rsp_expl_ack,
    output logic                       o_ob_rsp_vld,            // 表示响应信号有效
    input  logic                       ob_rsp_fifo_full,        // 表示响应 FIFO 已满

    // 来自 cmac_rx_intf
    input logic pkt_discarded,

    // 通向rx_req_axi_mstr 接口，三级流水后才有数据，请求包元数据
    output logic [                    31:0] req_pkt_id,
    output logic [                     3:0] req_pkt_type,
    output logic                            req_pkt_ip_ver,
    output logic [                    15:0] req_rq_pi_db,
    output logic [  C_M_AXI_ADDR_WIDTH-1:0] req_dest_addr,        // 写入内存的目的地址
    output logic [C_REQ_BUF_ADDR_WIDTH-1:0] req_buf_addr_offset,
    output logic [                    12:0] req_pkt_len,          // 总包长度，不含 FCS; reth_dma_len_zero == 1 时为 0
    // req_imm_data字段：
    // 发生错误时：对应手册 Packet Validation Error Syndrome 或者 Error Status Format
    // 正常请求时：{ 1bit硬件握手控制, 1bitMAD标志, 1bitRQ中断使能, 14bitRQ门铃地址, 11bit{保留,QP号}, 16bit门铃值 }
    output logic [                    43:0] req_imm_data,         // 请求立即控制字段，与 req axi master 搭配
    output logic                            req_tnfr_vld,         // 握手信号(与rx_req_axi_mstr模块)，表示 该模块 的请求包元数据传输有效
    input  logic                            rdy_to_req_tnfr,      // 握手信号(与rx_req_axi_mstr模块)，表示 rx_req_axi_mstr模块 的请求包元数据传输已准备好。为 0 时本状态机要 busy 等待，防止丢握手。
    output logic [     C_QP_INDX_WIDTH-1:0] req_qpid,
    output logic                            req_immdt_present,    // Immdt头部标志
    output logic                            req_ieth_present,     // IETH头部标志

    // 为 ecn
    //output logic  [C_NUM_QP-1:0]                  ecn_vect,
    output logic [C_QP_INDX_WIDTH-1:0] ecn_vect_idx,
    output logic                       ecn_vect_vld,

    // 为 cnp
    output logic                       cnp_pkt_rcvd,
    output logic [C_QP_INDX_WIDTH-1:0] cnp_pkt_qpid,

    // 调试信号
    output logic [31:0] o_all_rd_req_cnt,
    output logic [31:0] o_good_rd_req_cnt,
    output logic [31:0] o_dup_rd_req_cnt,

    //input  logic  [C_NUM_QP-1:0]                  i_qp_enable,
    input logic                       i_qp_enable_wr_en,
    input logic [C_QP_INDX_WIDTH-1:0] i_qp_enable_wr_idx,
    input logic                       i_qp_enable_wr_data,

    output logic                       waiting_for_rq_update,
    output logic [C_QP_INDX_WIDTH-1:0] waiting_for_rq_update_on_qp_num,
    output logic [       C_NUM_QP-1:0] rnr_window_flag_reg,

    input  logic        i_clr_debug_sts,
    output logic        rq_buf_full_sticky,
    output logic        qp_fatal_req_hv_sticky,
    output logic [31:0] rq_buf_full_event_cnt,
    output logic [31:0] rnr_window_cnt,

    output logic [               63:0] ieth_immdt_fifo_data,
    output logic                       ieth_immdt_fifo_wr_en,
    input  logic                       init_start,
    input  logic [C_QP_INDX_WIDTH-1:0] init_count,
    input  logic                       qp_under_recovery_wr_en,
    input  logic [C_QP_INDX_WIDTH-1:0] qp_under_recovery_wr_idx,
    input  logic                       qp_under_recovery_wr_data
);

    // Local Parameters
    localparam PIPELINE_MODE = 1;
    localparam FATAL_PIPELINE_MODE = 0;

    // 包类型定义
    // ------------------------------------------
    // 如果 i_err_buf_en 设置为 1，则会验证所有传入的数据包，并将所有未通过数据包验证的数据包头发送到错误缓冲区（基地址由 ERRBUFBA[31:0] 指定）
    localparam [3:0] PKT_TYPE_RSVD = 4'h0;  // 其他包
    localparam [3:0] PKT_TYPE_BYPASS = 4'h1;
    localparam [3:0] PKT_TYPE_MAD = 4'h2;  // MAD包，用于建立和管理连接
    localparam [3:0] PKT_TYPE_SEND_ONLY = 4'h3;
    localparam [3:0] PKT_TYPE_SEND_FIRST = 4'h4;
    localparam [3:0] PKT_TYPE_SEND_MIDDLE = 4'h5;
    localparam [3:0] PKT_TYPE_SEND_LAST = 4'h6;
    localparam [3:0] PKT_TYPE_CNP = 4'h8;  // CNP包，用于拥塞管理
    localparam [3:0] PKT_TYPE_RDMA_WR_ONLY = 4'h9;
    localparam [3:0] PKT_TYPE_RDMA_WR_FIRST = 4'hA;
    localparam [3:0] PKT_TYPE_RDMA_WR_MIDDLE = 4'hB;
    localparam [3:0] PKT_TYPE_RDMA_WR_LAST = 4'hC;
    localparam [3:0] PKT_TYPE_RDMA_RD_REQ = 4'hD;
    localparam [3:0] PKT_TYPE_UNSUPPORTED = 4'hF;  // 无效操作码包，不支持的操作码

    // 传输包类型定义，通过 req_pkt_type 传给 rx_req_axi_mstr 模块的 pkt_type 和 fifo_out_pkt_type，类型与 rx_req_axi_mstr模块 对应
    // ------------------------------------------
    localparam [3:0] REQ_AXI_TNFR_TYPE_DISCARD = 4'd0;
    localparam [3:0] REQ_AXI_TNFR_TYPE_ERROR = 4'd1;
    localparam [3:0] REQ_AXI_TNFR_TYPE_BYPASS = 4'd2;
    localparam [3:0] REQ_AXI_TNFR_TYPE_MAD = 4'd3;
    localparam [3:0] REQ_AXI_TNFR_TYPE_IPV4_SEND = 4'd4;
    localparam [3:0] REQ_AXI_TNFR_TYPE_IPV6_SEND = 4'd5;
    localparam [3:0] REQ_AXI_TNFR_TYPE_DB_UPDATE = 4'd6;
    localparam [3:0] REQ_AXI_TNFR_TYPE_IN_ERR_STS = 4'd7;
    localparam [3:0] REQ_AXI_TNFR_TYPE_IPV4_WR_REQ = 4'd12;
    localparam [3:0] REQ_AXI_TNFR_TYPE_IPV6_WR_REQ = 4'd13;
    localparam [3:0] REQ_AXI_TNFR_TYPE_IPV4_WR_REQ_RETH = 4'd14;
    localparam [3:0] REQ_AXI_TNFR_TYPE_IPV6_WR_REQ_RETH = 4'd15;

    localparam IPv4 = 1'b0;
    localparam IPv6 = 1'b1;

    localparam [2:0] RC = 3'b000;
    localparam [2:0] UD = 3'b011;

    // BTH操作码定义
    // ------------------------------------------
    localparam [4:0] SEND_FIRST = 5'b00000;
    localparam [4:0] SEND_MIDDLE = 5'b00001;
    localparam [4:0] SEND_LAST = 5'b00010;
    localparam [4:0] SEND_LAST_IMM = 5'b00011;
    localparam [4:0] SEND_ONLY = 5'b00100;
    localparam [4:0] SEND_ONLY_IMM = 5'b00101;
    localparam [4:0] RDMA_WR_FIRST = 5'b00110;
    localparam [4:0] RDMA_WR_MIDDLE = 5'b00111;
    localparam [4:0] RDMA_WR_LAST = 5'b01000;
    localparam [4:0] RDMA_WR_LAST_IMM = 5'b01001;
    localparam [4:0] RDMA_WR_ONLY = 5'b01010;
    localparam [4:0] RDMA_WR_ONLY_IMM = 5'b01011;
    localparam [4:0] RDMA_RD_REQ = 5'b01100;
    // localparam [4:0] CMPSWAP = 5'b10011;
    // localparam [4:0] FETCHADD = 5'b10100;
    localparam [4:0] SEND_LAST_INV = 5'b10110;
    localparam [4:0] SEND_ONLY_INV = 5'b10111;

    localparam [1:0] ACK = 2'b00;
    localparam [1:0] NAK = 2'b11;
    localparam [1:0] RNR_NAK = 2'b01;  // Receiver Not Ready

    // AETH Syndrome字段[7:5] = NAK
    // NAK code
    localparam [4:0] PSN_SEQ_ERR = 5'b0_0000;  // 进入的包PSN和ePSN不匹配
    localparam [4:0] INVLD_REQ = 5'b0_0001;  // 错误的Opcode
    localparam [4:0] REMOTE_ACC_ERR = 5'b0_0010;  // R_KEY错误
    // localparam [4:0] REMOTE_OP_ERR = 5'b0_0011;

    localparam [15:0] RNR_WINDOW_TVAL = 16'd4000;  // 20uS @200MHz
    localparam [4:0] RNR_NAK_VAL_PKT_DISCARD = 5'b0_0010;  // 20uS RNR-NAK, IB Spec, pg333

    // 第三阶段状态机状态定义
    // ------------------------------------------
    localparam [2:0] HV_IDLE_STATE = 3'd0;  // 空闲状态
    localparam [2:0] HV_HDRVAL_STATE = 3'd1;  // 头部验证状态
    localparam [2:0] HV_WRITE_STATE = 3'd2;  // 写回状态
    localparam [2:0] HV_ERR_STATE = 3'd3;  // 内部错误状态(FATAL)

    logic [                  C_NUM_QP:0] i;

    logic                                hdr_ftchg_done;  // 头部获取完成标志，随着第二拍数据一起输出
    logic                                req_hdr_buf_enb_r;
    logic                                hv_data_vld;  // 头部数据有效，hv_data_vld <= hdr_ftchg_done
    logic [                       511:0] prev_req_hdr_buf_doutb;  // 上一拍数据
    logic [                      1023:0] hdr_data;  // 头部数据 -- {上一拍数据，当前拍的数据}

    logic [                         0:0] hdr_ftchg_fsm_cs;  // 头部获取-状态机当前状态
    logic [                         0:0] hdr_ftchg_fsm_ns;  // 头部获取-状态机下一状态

    logic [                        31:0] pkt_id;  // 包ID
    logic [    C_REQ_BUF_ADDR_WIDTH-1:0] pkt_src_addr_offset;
    logic [                        12:0] actual_pkt_len;  // 总包长度（不含FCS）
    logic                                pkt_fcs_err;
    logic                                ipv4_hdr_chksum_fail;
    logic                                pkt_crc_err;

    logic [                        15:0] ether_type;
    logic                                ip_ver;
    logic [                        15:0] ipv4_total_len;  // IPv4头 + IPv4 payload(含ICRC)
    logic [                        15:0] ipv4_pyld_len;  // IPv4 payload(含ICRC)
    logic [                        15:0] ipv6_pyld_len;  // IPv6 payload(含ICRC)
    logic [                        15:0] ipv4_udp_dest_port_addr;
    logic [                        15:0] ipv6_udp_dest_port_addr;
    logic [                        15:0] ipv4_udp_len;  // IPv4 UDP长度字段值（UDP头+UDP负载+ICRC）
    logic [                        15:0] ipv6_udp_len;  // IPv6 UDP长度字段值
    logic [                        15:0] udp_dest_port_addr;
    logic [                        15:0] ip_pyld_len;  // IPv4时 ipv4_pyld_len，IPv6时 ipv6_pyld_len
    logic [                        15:0] udp_len;  // IPv4时 ipv4_udp_len，IPv6时 ipv6_udp_len
    logic [                        15:0] total_pkt_len_advt;  // 长度 = UDP长度字段值 + IPv4/v6头(20/40) + MAC头(14)
    logic [                         3:0] ipv4_bth_tver;
    logic [                         3:0] ipv6_bth_tver;
    logic [                         3:0] bth_tver;
    logic [                        23:0] ipv4_bth_qp_dest_addr;
    logic [                        23:0] ipv6_bth_qp_dest_addr;
    logic [                         3:0] bth_ver;
    logic [                        23:0] bth_qp_dest_addr;  // QP ID
    logic                                ipv4_bth_expl_ack;
    logic                                ipv6_bth_expl_ack;
    logic                                bth_expl_ack;  // BTH头部A字段：请求响应方在相关 QP 上安排确认
    logic [                         7:0] ipv4_bth_opcode;
    logic [                         7:0] ipv6_bth_opcode;
    logic [                         7:0] bth_opcode;
    logic [                         1:0] ipv4_bth_pad_cnt;
    logic [                         1:0] ipv6_bth_pad_cnt;
    logic [                         1:0] bth_pad_cnt;
    logic [                        23:0] ipv4_bth_psn;
    logic [                        23:0] ipv6_bth_psn;
    logic [                        23:0] bth_psn;
    logic [                        23:0] qp_conf_min;
    logic [                        23:0] qp_conf_max;
    logic                                qp_state_fatal;

    logic [                        63:0] ipv4_reth_va;  // RETH -- Virtual Address
    logic [                        63:0] ipv6_reth_va;
    logic [                        63:0] reth_va_w;
    logic [                        63:0] reth_va;  // RETH 虚拟地址
    logic [                        63:0] reth_va_r;
    logic [                        31:0] ipv4_reth_r_key;  // RETH -- R_Key
    logic [                        31:0] ipv6_reth_r_key;
    logic [                        31:0] reth_r_key_w;  // R_Key，不区分 IPv4/IPv6
    logic [                        31:0] reth_r_key;
    logic [                        31:0] reth_r_key_r;
    logic [                        31:0] ipv4_reth_dmalen;  // RETH -- DMA Length
    logic [                        31:0] ipv6_reth_dmalen;
    logic [                        31:0] reth_dmalen_w;
    logic [                        31:0] reth_dmalen;
    logic [                        31:0] reth_dmalen_r;
    logic [                        31:0] ieth_immdt_data;
    logic [                        31:0] ipv4_ieth_immdt_data;
    logic [                        31:0] ipv4_reth_ieth_immdt_data;
    logic [                        31:0] ipv6_ieth_immdt_data;
    logic [                        31:0] ipv6_reth_ieth_immdt_data;
    logic                                has_reth;  // 是否是reth头，随着hv_data_vld一起输出一拍
    logic                                has_reth_r;
    logic                                has_immdt;  // 是否是immdt头，随着hv_data_vld一起输出一拍
    logic                                has_ieth;  // 是否是ieth头，随着hv_data_vld一起输出一拍

    logic                                reth_r_key_chk_fail;
    logic                                reth_dma_len_chk_fail;
    logic [         C_VIRT_ADDR_WIDTH:0] buffer_max_addr_from_reth;
    logic [         C_VIRT_ADDR_WIDTH:0] buffer_max_addr_from_qp_conf;
    logic [       C_VIRT_ADDR_WIDTH-1:0] va_offset_from_reth;
    logic [       C_VIRT_ADDR_WIDTH-1:0] physical_addr_from_reth_1;

    logic                                write_only_or_last;
    logic                                write_middle_or_last;
    logic                                reth_dma_len_chk_fail_r;
    logic                                reth_r_key_chk_fail_r;
    logic [       C_VIRT_ADDR_WIDTH-1:0] physical_addr_from_reth_r;
    logic [       C_VIRT_ADDR_WIDTH-1:0] physical_addr_from_reth;

    logic [       C_VIRT_ADDR_WIDTH-1:0] wr_rq_buf_ca_plus_offset;
    logic [     C_VIRT_ADDR_WIDTH-1-8:0] i_wr_rq_buf_offset;  //Granularity of the address increment is 64 bytes. Hence declaring only C_VIRT_ADDR_WIDTH-8 bits
    logic [     C_VIRT_ADDR_WIDTH-1-8:0] o_wr_rq_buf_offset;

    logic                                wr_rq_buf_offset_en;
    logic                                wr_rq_buf_offset_we;
    logic                                wr_rq_buf_offset_we_w;

    logic                                mac_dest_addr_chk_fail;  // 若包头MAC目的地址不是本节点MAC地址，则标记失败
    logic                                ipv4_ver_chk_fail;
    logic                                ipv4_ihl_chk_fail;
    logic                                ipv4_ecn_chk_fail;
    logic                                ipv4_flg_chk_fail;
    logic                                ipv4_frag_offset_chk_fail;
    logic                                ipv4_prot_chk_fail;
    logic                                ipv4_dest_addr_chk_fail;
    logic                                ipv4_length_chk_fail;
    logic                                ipv4_src_addr_chk_fail;
    logic                                ipv6_ver_chk_fail;
    logic                                ipv6_ecn_chk_fail;
    logic                                ipv6_next_hdr_chk_fail;
    logic                                ipv6_dest_addr_chk_fail;
    logic                                ipv6_length_chk_fail;
    logic                                ipv6_src_addr_chk_fail;
    logic                                udp_dest_port_chk_fail;
    logic                                udp_length_chk_fail;
    logic                                pkt_len_chk_fail;
    logic                                bth_ver_chk_fail;
    logic                                dest_qp_chk_fail;
    logic                                bth_pad_cnt_chk_fail;
    logic                                ip_length_chk_fail;
    logic                                ip_dest_addr_chk_fail;
    logic                                ip_ecn_chk_fail;
    logic                                ip_ver_chk_fail;
    logic                                ip_src_addr_chk_fail;
    logic                                mac_src_addr_chk_fail;
    logic                                ip_src_addr_chk_fail_w;
    logic                                mac_src_addr_chk_fail_w;
    logic                                src_addr_chk_fail;
    logic                                mad_pkt_len_err;
    logic                                opcode_seq_chk_fail;  // 操作码序列检查失败
    logic                                send_pkt_len_err;
    logic                                write_pkt_len_err;

    logic                                bth_pad_cnt_chk_fail_r;
    logic                                dest_qp_chk_fail_r;
    logic                                bth_ver_chk_fail_r;
    logic                                pkt_len_chk_fail_r;
    logic                                udp_length_chk_fail_r;
    logic                                ip_length_chk_fail_r;
    logic                                ipv4_hdr_chksum_fail_r;
    logic                                ip_dest_addr_chk_fail_r;
    logic                                ipv4_frag_offset_chk_fail_r;
    logic                                ipv4_flg_chk_fail_r;
    logic                                ip_ecn_chk_fail_r;
    logic                                ipv4_ihl_chk_fail_r;
    logic                                ip_ver_chk_fail_r;
    logic                                mac_dest_addr_chk_fail_r;

    logic                                ip_ver_r;
    logic [                         7:0] bth_opcode_r;
    // logic [                        23:0] bth_qp_dest_addr_r;
    logic [                        23:0] bth_psn_r;
    logic                                bth_expl_ack_r;
    logic [                        47:0] mac_src_addr_r;
    logic [                        31:0] ipv4_src_addr_r;
    logic [                       127:0] ipv6_src_addr_r;

    // RoCE 业务负载长度（用于 SEND/WRITE 长度检查），不含 L2/L3/L4/BTH，也不含 ICRC；
    // 若报文带 ImmDt/IETH（4B），该值会在后续分支里再减 4B。
    logic [                        12:0] bth_pyld_len;  // BTH负载长度(不含BTH头部的长度)，这是“最终”的有效载荷长度
    // 基础业务负载长度（不含 RETH 口径）：
    // = actual_pkt_len - (Eth+IP+UDP+BTH+ICRC)
    // IPv4: -58 = 14+20+8+16+4, IPv6: -78 = 14+40+8+16+4
    logic [                        12:0] bth_pyld_len_no_reth;
    // 带 RETH 口径的业务负载长度：
    // = bth_pyld_len_no_reth - 16B(RETH)
    // 即不含 Eth/IP/UDP/BTH/RETH/ICRC
    logic [                        12:0] bth_pyld_len_with_reth;
    logic [                        12:0] qp_pmtu_decoded;  // 把 QP 配置里的 3bit PMTU (qp_pmtu)编码解码成实际字节数（256/512/1024/2048/4096），检查包 payload 长度是否合法、更新缓冲偏移

    logic [                        31:0] err_syndr;

    logic                                pkt_vldn_err_1;
    logic                                pkt_vldn_err;

    logic                                is_req_psn_matched;
    logic [                        23:0] dup_req_psn_max;
    logic [                        23:0] dup_req_psn_min;
    logic                                is_dup_req;  // 旧包/重传包检测

    logic [                         7:0] i_last_req_opcode;
    logic [                        23:0] i_last_req_psn;
    logic [                        23:0] exp_req_psn;
    logic [                        23:0] p_exp_req_psn;

    logic                                i_qp_rq_intr_en;
    logic                                i_qp_hw_hndshk_dis;

    logic                                rq_buf_full;

    //logic qp_en_r;
    //logic qp_rq_intr_en_r;
    //logic qp_hw_hndshk_dis_r;
    //logic [2:0] qp_pmtu_r;
    //logic [15:0] qp_rq_buf_sz_r;
    //logic [C_M_AXI_ADDR_WIDTH-1:0] rq_buf_ba_r;
    //logic [C_M_AXI_ADDR_WIDTH-9:0] rq_buf_ca_r;
    logic [      C_M_AXI_ADDR_WIDTH-1:0] rq_wrptr_db_addr_r;
    //logic [15:0] q_depth_r;
    //logic [15:0] rq_ci_db_r;
    //logic [7:0] last_req_opcode_r;
    //logic [23:0] last_req_psn_r;
    //logic [23:0] stat_qp_msn_r;
    //logic ip_src_addr_chk_fail_r;
    //logic mac_src_addr_chk_fail_r;

    logic [                        31:0] qp_w_r_buf_len_r;
    logic [                        31:0] qp_virt_addr_0_r;
    logic [                        31:0] qp_virt_addr_1_r;
    logic [                        31:0] qp_buf_base_addr_0_r;
    logic [                        31:0] qp_buf_base_addr_1_r;
    logic [                        31:0] qp_buf_r_key_r;

    //logic [31:0] qp_w_r_buf_len;
    //logic [31:0] qp_virt_addr_0;
    //logic [31:0] qp_virt_addr_1;
    //logic [31:0] qp_buf_base_addr_0;
    //logic [31:0] qp_buf_base_addr_1;
    //logic [31:0] qp_buf_r_key;

    logic [                        23:0] qp_psn_coef_decoded;
    logic [                        23:0] psn_incr_for_read_req;
    logic [                        23:0] p_incr_for_read_req;

    //logic qp_en;
    //logic qp_en_for_fsm;
    logic                                qp_rq_intr_en;
    logic                                qp_hw_hndshk_dis;  // 禁止硬件握手 - 1：禁用硬件门铃握手，门铃更新通过AXI/AXIL进行交换；0：启用硬件门铃握手
    logic [                         2:0] qp_pmtu;
    logic [                        15:0] qp_rq_buf_sz;  // RQ 缓冲区大小（256B 的倍数）。这是请求中每个缓冲区元素的大小，而不是整个请求的大小。
    logic [      C_M_AXI_ADDR_WIDTH-1:0] rq_buf_ba;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] rq_buf_ca;
    logic [      C_M_AXI_ADDR_WIDTH-1:0] rq_wrptr_db_addr;
    logic [                        15:0] q_depth;
    logic [                        15:0] rq_ci_db;
    logic [                         7:0] last_req_opcode;  // 上一次请求的操作码
    logic [                        23:0] last_req_psn;  // 上一次请求的PSN
    logic [                        23:0] p_last_req_psn;
    logic [                        23:0] stat_qp_msn;
    logic [                        15:0] rq_wrptr_db;

    logic [      C_M_AXI_ADDR_WIDTH-9:0] rq_buf_ca_plus_offset;

    logic                                send_only_or_last;
    logic                                send_middle_or_last;

    logic [                         4:0] in_err_sts_code;  // 内部错误状态码(即对应手册FATAL Codes)
    logic [                         4:0] nxt_in_err_sts_code;

    logic [                         3:0] pkt_type;

    logic [      C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_offset;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] o_rq_buf_offset;
    logic                                rq_buf_offset_en;
    logic                                rq_buf_offset_we;
    logic                                rq_buf_offset_we_w;

    //logic [C_NUM_QP-1:0] nak_seq_err_sent_reg;
    logic                                rd_req_pkt_desc_fifo_comb;
    logic                                qp_fatal_combo;
    logic                                qp_recovery_combo;
    logic                                qp_fatal_combo_c1d;
    logic                                qp_recovery_combo_c1d;
    logic                                nak_seq_err_sent_reg_combo;
    logic [         C_QP_INDX_WIDTH-1:0] nak_seq_err_sent_1_wr_addr;
    logic                                nak_seq_err_sent_1_wr_en;
    logic                                nak_seq_err_sent_1_wr_val;
    logic                                nak_seq_err_sent_we;
    logic                                nak_seq_err_sent_we_w;
    logic                                i_nak_seq_err_sent;
    logic                                o_nak_seq_err_sent;  // 这是“是否进入某类序列错误抑制状态”的记忆位

    logic [                        15:0] rnr_window_tval;
    logic                                rnr_window_closed;
    logic                                rnr_window_flag;

    logic [                        16:0] credits_remain;
    logic [                         4:0] credits_to_send;
    logic [                         4:0] flow_credits;

    logic                                incr_dup_pkt_cnt;  // 重复请求包计数器增加标志
    logic                                incr_out_nack_pkt_cnt;  // NAK发出计数器增加标志

    logic                                tnfr_pend;  // 传输挂起标志，表示有数据传输请求但还未被接受处理的状态

    logic [                         1:0] hv_fsm_cs;
    logic [                         1:0] hv_fsm_ns;

    logic                                hv_fsm_busy;  // 忙碌标志：1-表示正在忙碌处理，不能接受新的数据包

    //logic [C_NUM_QP-1:0] qp_enable_r1;
    //logic [C_NUM_QP-1:0] qp_enable_r2;
    //logic [C_NUM_QP-1:0] qp_enable_edge;

    // ob_rsp_*就是响应读请求的读响应包需要使用的信号组
    // ob = outbound response（出站响应），意思是“给对端回 ACK/NAK 的响应信息”
    logic [         C_QP_INDX_WIDTH-1:0] ob_rsp_qpid;  // Destination QP ID for which response packet is targetted                      
    logic [                        23:0] ob_rsp_psn;  // BTH.PSN to embed in response packet；输出响应包使用的 PSN（这里一般用期望请求 PSN exp_req_psn）
    logic [                         7:0] ob_rsp_aeth_syndrome;  // AETH.Syndrome to embed in response packet；AETH 的应答类型/错误码（队列满分支发 {NAK, INVLD_REQ}；远程访问错误分支发 {NAK, REMOTE_ACC_ERR}；正常分支发 {ACK, flow_credits}）
    logic                                rnr_syndrome_from_rnr_window_reqd_reg;
    logic                                rnr_syndrome_from_rnr_window_reqd;
    logic [                        23:0] ob_rsp_aeth_msn;  // AETH.MSN to embed in response packet，响应中的 MSN；NAK 时通常带当前 stat_qp_msn；ACK 成功时是 stat_qp_msn + 1
    logic                                ob_rsp_expl_ack;
    logic                                ob_rsp_vld;  // Indicates the response signals are valid 
    logic [                         1:0] ob_req_indicator;
    logic [                        31:0] ob_dma_len;
    logic [                        12:0] ob_rd_req_pmtu;
    logic [                        23:0] ob_rsp_last_psn;
    logic [                        63:0] ob_physical_addr;

    logic                                set_rq_full_flag;
    logic                                clr_rq_full_flag;

    logic [         C_QP_INDX_WIDTH-1:0] rsp_qp_fatal_idx_r;
    logic                                clr_qp_fatal_flag;
    logic                                rsp_qp_fatal_flag;

    logic [         C_QP_INDX_WIDTH-1:0] fatal_qp_idx;

    //logic [C_NUM_QP-1:0]                   nxt_ecn_vect;
    logic [         C_QP_INDX_WIDTH-1:0] nxt_ecn_vect_idx;
    logic                                nxt_ecn_update_flg;
    logic                                ecn_update_flg;

    logic                                bth_psn_0;  // bth_psn == 0 ? 1: 0

    logic                                set_ext_fatal_case_flag;
    logic                                clr_ext_fatal_case_flag;
    logic                                ext_fatal_case_flag;

    logic                                memory_access_read;
    logic                                memory_access_write;
    logic [                         1:0] memory_access_permission;

    logic                                qp_psn_rem_decoded;

    logic                                read_qp_conf;  // 读QP配置信号
    logic                                write_qp_conf;  // 写QP信息信号

    logic                                update_qp_info;  // qp写回请求。update_qp_info_w 上升沿检测信号

    //logic [C_VIRT_ADDR_WIDTH-1-8:0] wr_rq_buf_offset;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] rq_buf_offset;

    logic [         C_QP_INDX_WIDTH-1:0] o_qp_conf_idx_for_offset_mem;
    logic [         C_QP_INDX_WIDTH-1:0] wrb_qp_number;

    logic [                         7:0] bth_opcode_mem;
    logic                                has_reth_mem;  // 是否为reth头部
    logic                                has_reth_mem_r;

    logic [         C_QP_INDX_WIDTH-1:0] wrb_qp_number_c;
    logic [         C_QP_INDX_WIDTH-1:0] wrb_qp_number_r;

    logic [      C_M_AXI_ADDR_WIDTH-9:0] d_rq_buf_ca;
    logic [                        31:0] d_last_rq_req;
    logic [                        15:0] d_rq_wrptr_db;
    logic [                        23:0] d_stat_qp_msn;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] d_rq_buf_offset;
    logic [     C_VIRT_ADDR_WIDTH-1-8:0] d_wr_rq_buf_offset;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] c_rq_buf_ca;
    logic [                        31:0] c_last_rq_req;
    logic [                        15:0] c_rq_wrptr_db;
    logic [                        23:0] c_stat_qp_msn;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] c_rq_buf_offset;
    logic [     C_VIRT_ADDR_WIDTH-1-8:0] c_wr_rq_buf_offset;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] r_rq_buf_ca;
    logic [                        31:0] r_last_rq_req;
    logic [                        15:0] r_rq_wrptr_db;
    logic [                        23:0] r_stat_qp_msn;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] r_rq_buf_offset;
    logic [     C_VIRT_ADDR_WIDTH-1-8:0] r_wr_rq_buf_offset;

    logic [      C_M_AXI_ADDR_WIDTH-9:0] p_rq_buf_ca;
    logic [                        31:0] p_last_rq_req;
    logic [                        15:0] p_rq_wrptr_db;
    logic [                        23:0] p_stat_qp_msn;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] p_rq_buf_offset;
    logic [     C_VIRT_ADDR_WIDTH-1-8:0] p_wr_rq_buf_offset;


    logic [         C_QP_INDX_WIDTH-1:0] dbg_o_qp_conf_idx;
    logic                                dbg_qp_conf_en;
    logic                                dbg_rq_buf_ba_en;
    logic                                dbg_rq_wrptr_db_addr_en;
    logic                                dbg_q_depth_en;
    logic                                dbg_rq_ci_db_en;
    logic                                dbg_remote_qp_mac_addr_en;
    logic                                dbg_remote_qp_ip_addr_en;
    logic [         C_PD_INDX_WIDTH-1:0] dbg_o_pd_conf_idx;
    logic                                dbg_pd_conf_en;
    logic                                qp_under_recovery_wr_en_r;


    logic                                rq_buf_ca_we_w;
    logic                                last_rq_req_we_w;
    logic                                stat_qp_msn_we_w;
    logic                                rq_wrptr_db_we_w;

    logic                                write_qp_info;  // = !read_qp_conf
    logic                                qp_fatal_wqe_proc_rdy;  // 
    logic                                qp_fatal_rsp_hndl_rdy;
    logic                                async_fatal_asserted;  // 表示有QP处于致命错误状态需要立即处理，这是一个异步的、高优先级的错误信号
    logic [         C_QP_INDX_WIDTH-1:0] async_fatal_qp;


    logic                                qp_conf_2_r;
    logic [         C_QP_INDX_WIDTH-1:0] qp_number_r;
    logic [                        15:0] rq_wrptr_db_r;
    logic [         C_QP_INDX_WIDTH-1:0] fatal_qp_idx_r;
    // 读请求应答队列信号组（模块内部 full 状态管理信号）
    logic                                rd_req_fifo_full_combo;  // rd_req_fifo_full_addr读出的full状态
    logic                                rd_req_fifo_full_data;  // 当前 QP 的“读请求应答队列满”标志（当前 QP 的最终 full 状态）。为 1 时，新的 RD Read 请求不再接收，直接 NAK INVLD_REQ。
    //logic rd_req_fifo_full_r;
    logic [       C_QP_INDX_WIDTH-1 : 0] rd_req_fifo_full_addr;  // 当前正在处理的 QP 索引（作为 LUTRAM 读地址）

    logic                                rnr_window_active;

    logic [         C_QP_INDX_WIDTH-1:0] qp_fatal_wqe_proc;  // FATAL_PIPELINE_MODE == 0 时直连 i_qp_fatal_wqe_proc
    logic                                qp_fatal_wqe_proc_vld;  // FATAL_PIPELINE_MODE == 0 时直连 i_qp_fatal_wqe_proc_vld
    logic [         C_QP_INDX_WIDTH-1:0] qp_fatal_rsp_hndl;  // FATAL_PIPELINE_MODE == 0 时直连 i_qp_fatal_rsp_hndl
    logic                                qp_fatal_rsp_hndl_vld;  // FATAL_PIPELINE_MODE == 0 时直连 i_qp_fatal_rsp_hndl_vld
    logic [                         7:0] o_ob_rsp_aeth_syndrome_int;
    logic                                pkt_discarded_r;
    logic [       C_QP_INDX_WIDTH-1 : 0] c3d_qp_number_r;
    logic [       C_QP_INDX_WIDTH-1 : 0] rsp_qp_fatal_idx_2r;
    logic                                ob_rsp_vld_r;
    logic                                c3d_rnr_window_active_r;
    logic                                rnr_window_closed_r;
    logic                                set_qp_fatal_r;
    logic                                rsp_qp_fatal_flag_r;
    logic                                clr_qp_fatal_flag_r;
    logic                                rd_req_pkt_desc_fifo_done;
    logic                                data_missed;
    logic [PKT_DESC_FIFO_DATA_WIDTH-1:0] req_pkt_desc_fifo_dout_hold;

    // QP配置结构
    typedef struct packed {
        logic [31:0]                    qp_conf;             // QP 配置(只读)，存放在 QP Configuration QPi(QPCONFi) 寄存器中
        logic [C_M_AXI_ADDR_WIDTH-1:0]  rq_buf_ba;           // RQ 环形缓冲区基地址(只读)，当 rq_buf_ca 回卷时回到 rq_buf_ba
        logic [C_M_AXI_ADDR_WIDTH-9:0]  rq_buf_ca;           // RQ 当前写入的“当前地址”，作为当前请求落内存的位置基准，配合 offset 得最终地址
        logic [C_M_AXI_ADDR_WIDTH-1:0]  rq_wrptr_db_addr;    // RQ 写门铃地址(只读)，就是存放RQ写门铃值的地址。可以走AXI addr和rx_pkt_hndler_o_rq_db_addr
        logic [15:0]                    q_depth;             // 队列深度(只读)
        logic [15:0]                    rq_ci_db;            // RQ 消费者门铃(只读)，和 rq_wrptr_db 比较判断是否队列满
        logic [31:0]                    last_rq_req;         // 上一次已接收请求的{opcode, psn}；取 last_req_opcode 做 opcode 序列合法性检查，取 last_req_psn 算期望 PSN、防重/重发判断
        logic [23:0]                    stat_qp_msn;         // QP 的 MSN（已完成请求序号），ACK/NAK 响应时带回，正常处理后 +1 写回
        logic [47:0]                    remote_qp_mac_addr;  // 远程QP MAC地址(只读)
        logic [127:0]                   remote_qp_ip_addr;   // 远程QP IP地址(只读)
        logic [15:0]                    rq_wrptr_db;         // RQ 写门铃值
        logic [23:0]                    qp_pd_number;        // QP 绑定的 PD 编号
        logic [C_M_AXI_ADDR_WIDTH-9:0]  rq_buf_offset;       // SEND 路径的接收缓冲偏移
        logic [C_VIRT_ADDR_WIDTH-1-8:0] wr_rq_buf_offset;    // 下一次WRITE的绝对地址
        logic                           nak_seq_err_sent;    // 这个 QP 是否已经因为序列错误发过一次 NAK”的标志位，用来防止重复发送同类 NAK。
    } qp_conf_t;

    // PD配置结构
    typedef struct packed {
        logic [23:0] pd_number;        // PD号
        logic [31:0] virt_addr_0;      // virtual_addr_lsb
        logic [31:0] virt_addr_1;      // virtual_addr_msb
        logic [31:0] buf_base_addr_0;  // 缓冲区基地址lsb
        logic [31:0] buf_base_addr_1;  // 缓冲区基地址msb
        logic [7:0]  buf_r_key;        // 缓冲区R_KEY
        logic [47:0] w_r_buf_len;      // 写/读缓冲区长度
        logic [1:0]  access_type;      // 访问类型
    } pd_conf_t;

    // 错误标志信息结构体: 对应手册 Packet Validation Error Syndrome 不为黄的部分
    typedef struct packed {
        // 以下错误的处理均是丢弃数据包
        logic pkt_fcs_err;  // FCS 错误
        logic pkt_crc_err;  // ICRC 错误
        logic bit_29;
        logic bit_28;
        logic [3:0] bit_27_24;
        logic [3:0] bit_23_20;
        logic bit_19;
        logic bth_pad_cnt_chk_fail; // 对于收到的FIRST和MIDDLE请求/响应，BTH Pad 位不是 2'b00
        logic bit_17;
        logic bit_16;
        logic qp_state_fatal;  // BTH 目标地址中指定的 QP 未配置或未启用
        logic dest_qp_chk_fail;  // 不支持 BTH 中的 QP 目标地址
        logic bth_ver_chk_fail;  // BTH 版本不是 4'b0000
        logic pkt_len_chk_fail;     // 数据包实际长度 与 头部长度字段值减去头部长度后的值 不一致
        logic udp_length_chk_fail; // UDP 长度字段与 IPv4 总长度或 IPv6 有效负载长度不一致
        logic ip_length_chk_fail;  // IPv4 总长度字段值不在范围内 或 IPv6 有效负载长度字段值不在范围内
        logic ipv4_hdr_chksum_fail;  // IPv4 报头校验和错误。
        logic ip_dest_addr_chk_fail;  // IPv4 报头中的 IPv4 目标地址与配置的 XRNIC IPv4 地址不匹配
        logic bit_7;
        logic ipv4_frag_offset_chk_fail;  // IPv4 中的片段偏移不为 0
        logic ipv4_flg_chk_fail;  // IPv4 中的标志位不是 3'b010
        logic ip_ecn_chk_fail;  // Reserved - 1'b0
        logic ipv4_ihl_chk_fail;  // IPv4 标头长度不等于 20 字节
        logic ip_ver_chk_fail;  // IP 版本与 XRNIC 配置不符
        logic bit_1;  // Reserved
        logic mac_dest_addr_chk_fail;  // 以太网头中接收到的 MAC 目标地址与配置的 XRNIC MAC 地址不匹配
    } err_syndr_t;

    // 第一级流水线数据
    typedef struct packed {
        logic hv_data_vld;  // 头部数据有效
        logic [C_QP_INDX_WIDTH-1:0] qp_number;  // 目标队列对号 (BTH头部中的Dest QP字段)
        logic [C_PD_INDX_WIDTH-1:0] pd_number;  // 保护域号 (Protection Domain Number)
        logic [23:0] bth_psn;  // BTH包序列号 (24位)
        logic [3:0] pkt_type;  // 数据包类型 (SEND/WRITE/READ等)
        logic bypass_en;  // Reserved
        logic ip_ver;  // IP版本 (IPv4=0, IPv6=1)
        logic reg_ip_ver;  // 寄存器化的IP版本
        logic [7:0] bth_opcode;  // BTH操作码 (定义RDMA操作类型)
        logic [7:0] reg_bth_opcode;  // 寄存器化的BTH操作码
        logic bth_expl_ack;  // BTH头部A字段：请求响应方在相关 QP 上安排确认
        logic [47:0] mac_src_addr;  // 源MAC地址 (48位)
        logic [31:0] ipv4_src_addr;  // 源IPv4地址 (32位)
        logic [127:0] ipv6_src_addr;  // 源IPv6地址 (128位)
        logic [31:0] pkt_id;  // 包ID
        logic [C_REQ_BUF_ADDR_WIDTH-1:0] pkt_src_addr_offset;  // 包源地址偏移
        logic [12:0] actual_pkt_len;  // 实际包长度（包含所有头部和ICRC，不包括FCS）
        logic rnr_window_active;  // RNR窗口活跃中
        logic nak_seq_err_sent;  // NAK序列错误已发送标志
        logic rnr_window_flag;  // RNR窗口标志
        logic send_only_or_last;  // SEND_ONLY 或 SEND_LAST
        logic send_middle_or_last;  // SEND_MIDDLE 或 SEND_LAST
        logic mad_pkt_len_err;  // MAD包长度错误
        //logic [C_NUM_QP-1:0]             ecn_vect;
        logic [C_QP_INDX_WIDTH-1:0] ecn_vect_idx;  // ECN向量索引，即 QP ID
        logic ecn_update_flg;  // ECN更新标志
        logic [12:0] bth_pyld_len;  // BTH负载长度(不含BTH头部的长度)，这是最终的有效负载长度
        logic write_only_or_last;  // WRITE_ONLY或WRITE_LAST操作
        logic write_middle_or_last;  // WRITE_MIDDLE或WRITE_LAST操作
        logic has_reth;  // 是否包含RETH头
        logic has_immdt;  // 是否包含IMMDT头
        logic has_ieth;  // 是否包含IETH头
        logic [31:0] ieth_immdt_data;  // IETH/IMMDT数据
        logic [31:0] reth_dmalen;  // RETH DMA长度
        logic [31:0] reth_r_key;  // RETH rkey
        logic [63:0] reth_va;  // RETH 虚拟地址
    } cycle_1_data_t;

    // 第二级流水线数据
    typedef struct packed {
        err_syndr_t error_syndrome;
        cycle_1_data_t c1d;
        //fatal_data_t fatal_data;
        logic qp_conf_vld;  // QP配置有效标志
        qp_conf_t qp_conf_data;  // QP配置数据
        pd_conf_t pd_conf_data;  // PD配置数据
        logic ip_src_addr_chk_fail;  // IP源地址检查失败
        logic mac_src_addr_chk_fail;  // MAC源地址检查失败
        logic pkt_vldn_err;  // 包验证错误，总的验证失败标志
        logic opcode_seq_chk_fail;  // 操作码序列检查失败
        logic reth_r_key_chk_fail;  // RETH R_KEY检查失败

        // 分片与 PSN 计算
        logic [12:0] qp_pmtu_decoded;  // QP配置中的PMTU解码值
        logic [23:0] qp_psn_coef_decoded;  // 按 PMTU 把 reth_dmalen 做“整除 PMTU”的商，计算 RDMA READ 响应最后一个 PSN
        logic qp_psn_rem_decoded;  // reth_dmalen 除 PMTU 后是否有余数（0/1 标志），和 qp_psn_coef_decoded 一起算需要多少个响应包（是否多一个尾包）

        // RETH 地址范围检查
        logic [C_VIRT_ADDR_WIDTH:0]   buffer_max_addr_from_reth;     // RETH 请求覆盖的虚拟地址上界，和 QP/PD 配置允许的上界比较，做越界检查（reth_dma_len_chk_fail）
        logic [C_VIRT_ADDR_WIDTH:0]   buffer_max_addr_from_qp_conf;  // QP/PD 配置允许的虚拟地址上界，和 buffer_max_addr_from_reth 比较，判断请求是否超出注册内存范围

        // 虚实地址转换
        logic [C_VIRT_ADDR_WIDTH-1:0] va_offset_from_reth;  // 请求虚拟地址相对 PD 虚拟基址的偏移，做 VA->PA 转换的中间量
        logic [C_VIRT_ADDR_WIDTH-1:0] physical_addr_from_reth;  // 把 RETH 里的虚拟地址换算成物理地址，真正给 DMA/读响应路径使用的起始物理地址

        // 零长度请求短路控制
        logic reth_dma_len_zero;  // RETH 存在且 reth_dmalen == 0 时置 1，零长度请求特殊处理，不发数据搬运
    } cycle_2_data_t;

    // 第三级流水线数据
    typedef struct packed {
        cycle_2_data_t c2d;
        logic reth_dma_len_chk_fail;
        logic send_pkt_len_err;
        logic write_pkt_len_err;
        logic is_req_psn_matched;
        logic is_dup_req;
        logic reth_dma_len_too_large;
        logic [12:0] req_pkt_len;
        logic rq_buf_full;
    } cycle_3_data_t;

    // QP写回数据结构 - 包含所有需要更新的QP状态
    typedef struct packed {
        // === 实际数据字段 ===
        logic [C_M_AXI_ADDR_WIDTH-9:0] rq_buf_ca;  // 接收队列当前地址
        logic [31:0] last_rq_req;  // 上一次已接收请求的操作码和PSN，{8位操作码, 24位PSN}
        logic [23:0] stat_qp_msn;  // QP消息序列号
        logic [15:0] rq_wrptr_db;  // RQ 写门铃值
        logic [C_M_AXI_ADDR_WIDTH-9:0] rq_buf_offset;  // 接收缓冲区偏移
        logic [C_VIRT_ADDR_WIDTH-1-8:0] wr_rq_buf_offset;  // 下一次WRITE的绝对地址
        logic [C_QP_INDX_WIDTH-1:0] wrb_qp_number;  // QP号(用于匹配)
        logic nak_seq_err_sent;  // NAK序列错误标志

        // === 各字段写使能，决定是否更新该字段 ===
        logic rq_buf_ca_we;         // 当前地址写使能
        logic last_rq_req_we;       // 最后请求写使能
        logic stat_qp_msn_we;       // MSN写使能
        logic rq_wrptr_db_we;       // RQ写门铃写使能
        logic rq_buf_offset_we;     // RQ缓冲区偏移写使能
        logic wr_rq_buf_offset_we;  // 写缓冲区偏移写使能
        logic nak_seq_err_sent_we;  // NAK标志写使能？
    } qp_write_back_data_t;

    cycle_1_data_t                             c1d;
    cycle_2_data_t                             c2d;
    cycle_3_data_t                             c3d;
    //fatal_data_t fatal_data;
    qp_conf_t                                  qp_conf_data;
    pd_conf_t                                  pd_conf_data;

    // 缓冲区变量
    qp_write_back_data_t                       qp_wrb_dat_0;  // 缓冲区0 - 主缓冲区
    qp_write_back_data_t                       qp_wrb_dat_1;  // 缓冲区1 - 辅助缓冲区
    qp_write_back_data_t                       qp_wrb_dat_w;  // 写入数据 - 当前要写回的qp数据结构，c3d周期计算出来、准备写回的最新值集合，是“最新待提交值”，而 qp_wrb_dat_w.xxx_we 决定这个字段这拍是否要更新
    qp_write_back_data_t                       qp_wrb_dat_r;  // 读取数据 - 根据当前要读取的QP号选择合适的缓冲区
    // 更新触发信号
    logic                                      update_qp_info_r;  // 上一周期的更新QP信息触发信号，用于update_qp_info上升沿检测
    logic                                      update_qp_info_w;  // 更新QP信息触发信号--rq_buf_ca_we_w | last_rq_req_we_w | stat_qp_msn_we_w | rq_wrptr_db_we_w | rq_buf_offset_we_w | wr_rq_buf_offset_we_w | nak_seq_err_sent_we_w
    logic                                      qp_wrb_wr_ptr;

    logic                [                1:0] wrb_entry_valid;  // 2'b00: 缓冲区空；2'b01: qp_wrb_dat_0有效；2'b11: 两个缓冲区都有数据
    logic                                      get_qp_info_from_buffer;  // 从缓冲区获取QP信息标志
    logic                                      nak_seq_err_sent;
    logic                                      i_qp_enable_wr_en_r;
    logic                [C_QP_INDX_WIDTH-1:0] i_qp_enable_wr_idx_r;
    logic                                      i_qp_enable_wr_data_r;
    logic                                      qp_enable_lutram_out;


    logic                                      incr_all_rd_req_cnt;
    logic                                      incr_good_rd_req_cnt;
    logic                                      incr_dup_rd_req_cnt;

    logic                                      dbg_len_chk_err;
    logic                                      dbg_opcode_seq_chk_err;
    logic                                      dbg_reth_dma_len_chk_err;
    logic                                      dbg_hv_data_vld;
    logic                [C_QP_INDX_WIDTH-1:0] dbg_qp_number;
    logic                [               23:0] dbg_bth_psn;
    logic                [                3:0] dbg_pkt_type;
    logic                [                7:0] dbg_psn_from_qp_conf;
    logic                [                7:0] dbg_psn_used;
    logic                                      dbg_qp_conf_vld;
    logic                                      dbg_write_qp_info;
    logic                                      dbg_wrb_qp_write;
    logic                [                1:0] dbg_wrb_entry_valid;


    assign dbg_len_chk_err                  = c3d.send_pkt_len_err | c3d.write_pkt_len_err;
    assign dbg_opcode_seq_chk_err           = c3d.c2d.opcode_seq_chk_fail;
    assign dbg_reth_dma_len_chk_err         = c3d.reth_dma_len_chk_fail;
    assign dbg_hv_data_vld                  = c3d.c2d.c1d.hv_data_vld;
    assign dbg_qp_number                    = c3d.c2d.c1d.qp_number;
    assign dbg_bth_psn                      = c3d.c2d.c1d.bth_psn;
    assign dbg_pkt_type                     = c3d.c2d.c1d.pkt_type;
    assign dbg_psn_from_qp_conf             = i_last_rq_req[7:0];
    assign dbg_qp_conf_vld                  = qp_conf_vld;
    assign dbg_write_qp_info                = write_qp_info;
    assign dbg_wrb_qp_write                 = rq_buf_ca_we | last_rq_req_we | stat_qp_msn_we | rq_wrptr_db_we | rq_buf_offset_we | wr_rq_buf_offset_we;
    assign dbg_wrb_entry_valid              = wrb_entry_valid;


    // 任一情况发生，都需要更新 Intiator 中存放的QP信息
    assign update_qp_info_w                 = rq_buf_ca_we_w | last_rq_req_we_w | stat_qp_msn_we_w | rq_wrptr_db_we_w | rq_buf_offset_we_w | wr_rq_buf_offset_we_w | nak_seq_err_sent_we_w;
    assign qp_wrb_dat_w.rq_buf_ca_we        = rq_buf_ca_we_w;  // rq_buf_ca 需要更新
    assign qp_wrb_dat_w.last_rq_req_we      = last_rq_req_we_w;  // last_rq_req 需要更新
    assign qp_wrb_dat_w.stat_qp_msn_we      = stat_qp_msn_we_w;  // stat_qp_msn 需要更新
    assign qp_wrb_dat_w.rq_wrptr_db_we      = rq_wrptr_db_we_w;  // rq_wrptr_db 需要更新
    assign qp_wrb_dat_w.rq_buf_offset_we    = rq_buf_offset_we_w;  // rq_buf_offset 需要更新
    assign qp_wrb_dat_w.wr_rq_buf_offset_we = wr_rq_buf_offset_we_w;  // wr_rq_buf_offset 需要更新
    assign qp_wrb_dat_w.nak_seq_err_sent_we = nak_seq_err_sent_we_w;  // nak_seq_err_sent 需要更新
    assign qp_wrb_dat_w.wrb_qp_number       = c3d.c2d.c1d.qp_number;

    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////// rd_req_fifo_full LUTRAM START ///////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    // 该 RAM 记录每个 QP 的读请求应答队列是否满

    reg                        rd_req_fifo_full_reset_r;
    reg  [C_QP_INDX_WIDTH-1:0] rd_req_fifo_full_reset_idx_r;
    reg                        rd_req_fifo_full_set_r;
    reg  [C_QP_INDX_WIDTH-1:0] rd_req_fifo_full_set_idx_r;
    wire                       rd_req_fifo_full_wr_en;
    wire [C_QP_INDX_WIDTH-1:0] rd_req_fifo_full_wr_idx;
    wire                       rd_req_fifo_full_wr_data;

    // *_r把外部接口打一拍，和本模块时序对齐，避免直接组合驱动 RAM 写口
    always @(posedge clk) begin
        if (reset) begin
            rd_req_fifo_full_reset_r     <= 1'b0;
            rd_req_fifo_full_reset_idx_r <= 'd0;
            rd_req_fifo_full_set_r       <= 1'b0;
            rd_req_fifo_full_set_idx_r   <= 'd0;
        end else begin
            rd_req_fifo_full_set_r     <= i_rd_req_fifo_full_set;
            rd_req_fifo_full_set_idx_r <= i_rd_req_fifo_full_set_idx;
            if (~(rd_req_fifo_full_reset_r && rd_req_fifo_full_set_r)) begin
                rd_req_fifo_full_reset_r     <= i_rd_req_fifo_full_reset;
                rd_req_fifo_full_reset_idx_r <= i_rd_req_fifo_full_reset_idx;
            end
        end
    end

    // RAM 写口合成，把 set/reset/init 三类外部来源统一成 LUTRAM 的一套写口
    assign rd_req_fifo_full_wr_en   = init_start || rd_req_fifo_full_set_r || rd_req_fifo_full_reset_r;
    assign rd_req_fifo_full_wr_idx  = init_start ? init_count : (rd_req_fifo_full_set_r ? rd_req_fifo_full_set_idx_r : rd_req_fifo_full_reset_idx_r);
    assign rd_req_fifo_full_wr_data = rd_req_fifo_full_set_r ? 1'b1 : 1'b0;

    sdp_lut_ram_wrapper #(  // 为解决读写同时问题的 rd_req_fifo_full sdpram，确保读取的是最新值
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_COMBO_BYPASS(1),  //Muxing done outside：外部进行多路复用
        .C_Init_with_1(0)
    ) inst_rd_req_fifo_full_lutram (
        .core_clk(clk),
        .core_rst(reset),
        // Write port
        .wr_en   (rd_req_fifo_full_wr_en),
        .wr_addr (rd_req_fifo_full_wr_idx),
        .wr_data (rd_req_fifo_full_wr_data),

        // Read port
        .rd_en  (1'b1),                   // 始终启用读取
        .rd_addr(rd_req_fifo_full_addr),
        .rd_data(rd_req_fifo_full_combo)  // 读出来的“存量状态”
    );

    // 这里做了同拍旁路：如果这一拍正好有外部对当前 QP 的 set/reset，直接返回新值，不等 RAM 读回
    // “即时状态”
    assign rd_req_fifo_full_data = (i_rd_req_fifo_full_set && i_rd_req_fifo_full_set_idx == rd_req_fifo_full_addr) ? 1'b1 : ((i_rd_req_fifo_full_reset && i_rd_req_fifo_full_reset_idx == rd_req_fifo_full_addr) ? 1'b0 : rd_req_fifo_full_combo);

    //////////////////////////////////////////////////////////////////////////////
    ////////////////////// rd_req_fifo_full LUTRAM END ///////////////////////////
    //////////////////////////////////////////////////////////////////////////////

    always @(posedge clk) begin
        //rd_req_fifo_full_r <= rd_req_fifo_full_combo; //Registering this signal for improving timing
        rd_req_fifo_full_addr <= c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
    end

    //always @ (posedge clk)
    //begin
    //  qp_enable_r1 <= i_qp_enable;
    //  qp_enable_r2 <= qp_enable_r1;
    //end

    //assign qp_enable_edge = ~qp_enable_r2 & qp_enable_r1;


    always @(posedge clk) begin
        update_qp_info_r <= update_qp_info_w;
    end
    // update_qp_info_w 上升沿检测
    assign update_qp_info = update_qp_info_w & ~update_qp_info_r;

    //Write back buffer logic is as described below:
    //2 locaitons of write back memory is allocated qp_wrb_dat_1 and qp_wrb_dat_0
    //It acts like a shift register data flowing from qp_wrb_dat_1 to qp_wrb_dat_0
    //When the buffer is empty the first write is done in qp_wrb_dat_0
    //Data is read from qp_wrb_dat_0 and written back into the qpn registers
    //In cases where the buffer is empty and a window is open to write the qpn registers,
    //the data from qp_wrb_dat_w is written if it has valid data (qualified by update_qp_info) 
    //When a qp read is happening and there is still data present int the buffer, the qpn of the
    //data in the buffer is checked. if the qpn matches with the read qpn, the data is read from the buffer
    // 
    //回写缓冲逻辑如下所述：
    //   分配 2 个回写存储器位置 qp_wrb_dat_1 和 qp_wrb_dat_0
    //   它就像一个移位寄存器，数据从 qp_wrb_dat_1 流向 qp_wrb_dat_0
    //   当缓冲区为空时，第一次写入在 qp_wrb_dat_0 中完成
    //   从 qp_wrb_dat_0 读取数据并写回 qpn 寄存器
    //   在缓冲区为空且打开窗口写入 qpn 寄存器的情况下、
    //   如果 qp_wrb_dat_w 中的数据有效（由 update_qp_info 限定），则写入该数据。
    //   当发生 qp 读取且缓冲区中仍有数据时，将检查缓冲区中数据的 qpn。
    //   如果 qpn 与读取的 qpn 匹配，则从缓冲区读取数据。
    // 确保在读QP配置时，能够获取到最新的QP信息
    // ===================================== 最新qp数据路由逻辑(读) =====================================
    //always @ (posedge clk)
    always @(*) begin
        qp_wrb_dat_r            = '{default: 0};
        get_qp_info_from_buffer = 0;
        if (qp_conf_vld) begin  // 当前有QP配置读取请求
            case (wrb_entry_valid)
                // 在缓冲区空的情况下，如果同时发生读写同一QP，直接使用即将写回的数据，这是最新的状态。
                2'b00: begin  // 缓冲区空
                    if (update_qp_info) begin  // 同时有写回请求
                        // 当前要写回qp的qp号与当前要读取qp的qp号相同
                        if (qp_wrb_dat_w.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_w;  // 读取的QP信息使用要写回的qp信息（最新）
                            get_qp_info_from_buffer = 1;  // 表示是从缓冲区获取QP信息的标志

                        end else begin
                            qp_wrb_dat_r            = '{default: 0};
                            get_qp_info_from_buffer = 0;
                        end

                    end else begin
                        qp_wrb_dat_r            = '{default: 0};
                        get_qp_info_from_buffer = 0;
                    end
                end  //2'b00


                2'b01: begin  // 缓冲区0有效
                    if (update_qp_info) begin  // 有新的写回请求
                        // 优先检查回写qp数据结构
                        // 当前要写回qp的qp号与当前要读取qp的qp号相同
                        if (qp_wrb_dat_w.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_w;  // 读取的QP信息使用要写回的qp信息（最新）
                            get_qp_info_from_buffer = 1;

                            // 再检查缓冲区0中qp的qp号
                        end else if (qp_wrb_dat_0.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_0;  // 使用缓冲区0数据
                            get_qp_info_from_buffer = 1;

                        end else begin
                            qp_wrb_dat_r            = '{default: 0};
                            get_qp_info_from_buffer = 0;
                        end

                    end else begin  // 无新写回，只检查缓冲区
                        // 只检查缓冲区数据
                        if (qp_wrb_dat_0.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_0;
                            get_qp_info_from_buffer = 1;
                        end else begin
                            qp_wrb_dat_r            = '{default: 0};
                            get_qp_info_from_buffer = 0;
                        end
                    end
                end  //2'b01

                2'b11: begin  // 两个条目都有效
                    if (update_qp_info) begin  // 在实际操作中，这种情况绝对不能发生。这会导致不必要的错误
                        // 当前要写回qp的qp号与当前要读取qp的qp号相同
                        if (qp_wrb_dat_w.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_w;
                            get_qp_info_from_buffer = 1;

                            // 再检查缓冲区1中qp的qp号
                        end else if (qp_wrb_dat_1.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_1;
                            get_qp_info_from_buffer = 1;

                            // 再检查缓冲区0中qp的qp号
                        end else if (qp_wrb_dat_0.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_0;
                            get_qp_info_from_buffer = 1;
                        end else begin
                            qp_wrb_dat_r            = '{default: 0};
                            get_qp_info_from_buffer = 0;
                        end

                    end else begin  // 无新写回，检查缓冲区(优先级：dat_1 > dat_0)
                        if (qp_wrb_dat_1.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_1;
                            get_qp_info_from_buffer = 1;
                        end else if (qp_wrb_dat_0.wrb_qp_number == c1d.qp_number[C_QP_INDX_WIDTH-1:0]) begin
                            qp_wrb_dat_r            = qp_wrb_dat_0;
                            get_qp_info_from_buffer = 1;
                        end else begin
                            qp_wrb_dat_r            = '{default: 0};
                            get_qp_info_from_buffer = 0;
                        end
                    end
                end  //2'b11

                default: begin
                    qp_wrb_dat_r            = '{default: 0};
                    get_qp_info_from_buffer = 0;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (reset) qp_wrb_wr_ptr <= 0;
        else begin
            if (update_qp_info) qp_wrb_wr_ptr <= ~qp_wrb_wr_ptr;
        end
    end

    // 读写互斥控制
    assign write_qp_info = !read_qp_conf;

    // The qp config That has to be written back into the qp registers is stored temporarily here and written back when the window is open for writing (i.e. read is not happening)
    // 需要写回 qp 寄存器的 qp 配置将暂时保存在缓冲区中（即不进行读取）。
    // ===================================== 缓冲区管理逻辑 =====================================
    always @(posedge clk) begin
        if (reset) begin
            qp_wrb_dat_0       <= '{default: 0};
            qp_wrb_dat_1       <= '{default: 0};
            wrb_entry_valid    <= 0;
            wrb_qp_number_r    <= 0;
            r_rq_buf_ca        <= 'd0;
            r_last_rq_req      <= 'd0;
            r_stat_qp_msn      <= 'd0;
            r_rq_wrptr_db      <= 'd0;
            r_rq_buf_offset    <= 'd0;
            r_wr_rq_buf_offset <= 'd0;
        end else begin

            // 保存当前周期的QP号和相关字段，便于在qp信息写回时使用
            wrb_qp_number_r    <= wrb_qp_number_c;
            r_rq_buf_ca        <= c_rq_buf_ca;
            r_last_rq_req      <= c_last_rq_req;
            r_stat_qp_msn      <= c_stat_qp_msn;
            r_rq_wrptr_db      <= c_rq_wrptr_db;
            r_rq_buf_offset    <= c_rq_buf_offset;
            r_wr_rq_buf_offset <= c_wr_rq_buf_offset;

            case (wrb_entry_valid)
                2'b00: begin  // 缓冲区空
                    if (update_qp_info & !write_qp_info) begin  // qp要更新 & 读取qp配置
                        qp_wrb_dat_0       <= qp_wrb_dat_w;  // 将回写qp配置写入缓冲区0
                        wrb_entry_valid[0] <= 1;
                    end
                end

                2'b01: begin  // 缓冲区0不空
                    if (update_qp_info) begin  // qp要更新
                        if (write_qp_info) begin  // 写入
                            qp_wrb_dat_0 <= qp_wrb_dat_w;  // 覆盖缓冲区0
                        end else begin  // 读取
                            qp_wrb_dat_1       <= qp_wrb_dat_w;  // 写入缓冲区1
                            wrb_entry_valid[1] <= 1;
                        end
                    end else begin  // 没有更新
                        if (write_qp_info) begin  // 写入
                            wrb_entry_valid[0] <= 0;  // 清除缓冲区0
                        end
                    end
                end

                //2'b10:
                //  begin
                //    if (update_qp_info)
                //    begin
                //      qp_wrb_dat_0 <= qp_wrb_dat_w;
                //      wrb_entry_valid[1] <= 1;
                //    end
                //    if (write_qp_info)
                //    begin
                //      wrb_entry_valid[1] <= 0;
                //    end
                //  end
                2'b11: begin  // 两个缓冲区都不空
                    if (update_qp_info) begin  // 实际上，这种情况不应该发生。如果发生这种情况，将丢失一个要写入的 qp 配置数据，并导致意外行为
                        qp_wrb_dat_1 <= qp_wrb_dat_w;  // 更新缓冲区1(可能丢失数据)
                    end else begin  // 这种情况一般会发生
                        wrb_entry_valid[1] <= 0;
                    end
                    if (write_qp_info) begin
                        qp_wrb_dat_0 <= qp_wrb_dat_1;  // 数据从缓冲区1流向缓冲区0
                    end
                end

                default: begin
                    wrb_entry_valid <= 0;
                end
            endcase
        end
    end

    always @(*) begin
        wrb_qp_number       = 0;
        d_rq_buf_ca         = 0;
        d_last_rq_req       = 0;
        d_stat_qp_msn       = 0;
        d_rq_wrptr_db       = 0;
        d_rq_buf_offset     = 0;
        d_wr_rq_buf_offset  = 0;
        rq_buf_ca_we        = 0;
        last_rq_req_we      = 0;
        stat_qp_msn_we      = 0;
        rq_wrptr_db_we      = 0;
        rq_buf_offset_we    = 0;
        wr_rq_buf_offset_we = 0;
        nak_seq_err_sent_we = 0;
        nak_seq_err_sent    = 0;
        case (wrb_entry_valid)
            2'b00: begin  // 缓冲区空
                if (write_qp_info) begin
                    if (update_qp_info) begin  // 有回写 & 有更新，使用最新数据
                        wrb_qp_number       = qp_wrb_dat_w.wrb_qp_number;
                        d_rq_buf_ca         = qp_wrb_dat_w.rq_buf_ca;
                        d_last_rq_req       = qp_wrb_dat_w.last_rq_req;
                        d_stat_qp_msn       = qp_wrb_dat_w.stat_qp_msn;
                        d_rq_wrptr_db       = qp_wrb_dat_w.rq_wrptr_db;
                        d_rq_buf_offset     = qp_wrb_dat_w.rq_buf_offset;
                        d_wr_rq_buf_offset  = qp_wrb_dat_w.wr_rq_buf_offset;

                        rq_buf_ca_we        = qp_wrb_dat_w.rq_buf_ca_we;
                        last_rq_req_we      = qp_wrb_dat_w.last_rq_req_we;
                        stat_qp_msn_we      = qp_wrb_dat_w.stat_qp_msn_we;
                        rq_wrptr_db_we      = qp_wrb_dat_w.rq_wrptr_db_we;
                        rq_buf_offset_we    = qp_wrb_dat_w.rq_buf_offset_we;
                        wr_rq_buf_offset_we = qp_wrb_dat_w.wr_rq_buf_offset_we;
                        nak_seq_err_sent_we = qp_wrb_dat_w.nak_seq_err_sent_we;
                        nak_seq_err_sent    = qp_wrb_dat_w.nak_seq_err_sent;
                    end
                end
            end

            2'b01, 2'b11: begin  // 有缓冲数据，使用缓冲区数据
                if (write_qp_info) begin
                    wrb_qp_number       = qp_wrb_dat_0.wrb_qp_number;
                    d_rq_buf_ca         = qp_wrb_dat_0.rq_buf_ca;
                    d_last_rq_req       = qp_wrb_dat_0.last_rq_req;
                    d_stat_qp_msn       = qp_wrb_dat_0.stat_qp_msn;
                    d_rq_wrptr_db       = qp_wrb_dat_0.rq_wrptr_db;
                    d_rq_buf_offset     = qp_wrb_dat_0.rq_buf_offset;
                    d_wr_rq_buf_offset  = qp_wrb_dat_0.wr_rq_buf_offset;

                    rq_buf_offset_we    = qp_wrb_dat_0.rq_buf_offset_we;
                    wr_rq_buf_offset_we = qp_wrb_dat_0.wr_rq_buf_offset_we;
                    rq_buf_ca_we        = qp_wrb_dat_0.rq_buf_ca_we;
                    last_rq_req_we      = qp_wrb_dat_0.last_rq_req_we;
                    stat_qp_msn_we      = qp_wrb_dat_0.stat_qp_msn_we;
                    rq_wrptr_db_we      = qp_wrb_dat_0.rq_wrptr_db_we;
                    nak_seq_err_sent_we = qp_wrb_dat_0.nak_seq_err_sent_we;
                    nak_seq_err_sent    = qp_wrb_dat_0.nak_seq_err_sent;
                end
            end

            default: begin
                wrb_qp_number       = 0;
                d_rq_buf_ca         = 0;
                d_last_rq_req       = 0;
                d_stat_qp_msn       = 0;
                d_rq_wrptr_db       = 0;
                d_rq_buf_offset     = 0;
                d_wr_rq_buf_offset  = 0;
                rq_buf_offset_we    = 0;
                wr_rq_buf_offset_we = 0;
                rq_buf_ca_we        = 0;
                last_rq_req_we      = 0;
                stat_qp_msn_we      = 0;
                rq_wrptr_db_we      = 0;
                nak_seq_err_sent_we = 0;
                nak_seq_err_sent    = 0;
            end
        endcase
    end
    // 这段为PIPELINE_MODE==1
    always @(*) begin
        case (wrb_entry_valid)
            2'b00: begin  // 这里不会写
                if (update_qp_info & !write_qp_info) begin
                    wrb_qp_number_c    = qp_wrb_dat_w.wrb_qp_number;
                    c_rq_buf_ca        = qp_wrb_dat_w.rq_buf_ca;
                    c_last_rq_req      = qp_wrb_dat_w.last_rq_req;
                    c_stat_qp_msn      = qp_wrb_dat_w.stat_qp_msn;
                    c_rq_wrptr_db      = qp_wrb_dat_w.rq_wrptr_db;
                    c_rq_buf_offset    = qp_wrb_dat_w.rq_buf_offset;
                    c_wr_rq_buf_offset = qp_wrb_dat_w.wr_rq_buf_offset;
                end else begin
                    if (!hv_fsm_busy) begin
                        wrb_qp_number_c    = c2d.c1d.qp_number;
                        c_rq_buf_ca        = p_rq_buf_ca;
                        c_last_rq_req      = p_last_rq_req;
                        c_stat_qp_msn      = p_stat_qp_msn;
                        c_rq_wrptr_db      = p_rq_wrptr_db;
                        c_rq_buf_offset    = p_rq_buf_offset;
                        c_wr_rq_buf_offset = p_wr_rq_buf_offset;
                    end else begin
                        wrb_qp_number_c    = qp_wrb_dat_w.wrb_qp_number;
                        c_rq_buf_ca        = qp_wrb_dat_w.rq_buf_ca;
                        c_last_rq_req      = qp_wrb_dat_w.last_rq_req;
                        c_stat_qp_msn      = qp_wrb_dat_w.stat_qp_msn;
                        c_rq_wrptr_db      = qp_wrb_dat_w.rq_wrptr_db;
                        c_rq_buf_offset    = qp_wrb_dat_w.rq_buf_offset;
                        c_wr_rq_buf_offset = qp_wrb_dat_w.wr_rq_buf_offset;
                    end
                end
            end
            2'b01: begin  // qp信息在此写回Intiator寄存器组
                if (update_qp_info & write_qp_info) begin
                    if (!hv_fsm_busy) begin
                        wrb_qp_number_c    = c2d.c1d.qp_number;
                        c_rq_buf_ca        = p_rq_buf_ca;
                        c_last_rq_req      = p_last_rq_req;
                        c_stat_qp_msn      = p_stat_qp_msn;
                        c_rq_wrptr_db      = p_rq_wrptr_db;
                        c_rq_buf_offset    = p_rq_buf_offset;
                        c_wr_rq_buf_offset = p_wr_rq_buf_offset;
                    end else begin
                        wrb_qp_number_c    = qp_wrb_dat_w.wrb_qp_number;
                        c_rq_buf_ca        = qp_wrb_dat_w.rq_buf_ca;
                        c_last_rq_req      = qp_wrb_dat_w.last_rq_req;
                        c_stat_qp_msn      = qp_wrb_dat_w.stat_qp_msn;
                        c_rq_wrptr_db      = qp_wrb_dat_w.rq_wrptr_db;
                        c_rq_buf_offset    = qp_wrb_dat_w.rq_buf_offset;
                        c_wr_rq_buf_offset = qp_wrb_dat_w.wr_rq_buf_offset;
                    end
                end else begin
                    wrb_qp_number_c    = wrb_qp_number_r;
                    c_rq_buf_ca        = r_rq_buf_ca;
                    c_last_rq_req      = r_last_rq_req;
                    c_stat_qp_msn      = r_stat_qp_msn;
                    c_rq_wrptr_db      = r_rq_wrptr_db;
                    c_rq_buf_offset    = r_rq_buf_offset;
                    c_wr_rq_buf_offset = r_wr_rq_buf_offset;
                end
            end
            2'b11: begin
                if (write_qp_info) begin
                    wrb_qp_number_c    = qp_wrb_dat_1.wrb_qp_number;
                    c_rq_buf_ca        = qp_wrb_dat_1.rq_buf_ca;
                    c_last_rq_req      = qp_wrb_dat_1.last_rq_req;
                    c_stat_qp_msn      = qp_wrb_dat_1.stat_qp_msn;
                    c_rq_wrptr_db      = qp_wrb_dat_1.rq_wrptr_db;
                    c_rq_buf_offset    = qp_wrb_dat_1.rq_buf_offset;
                    c_wr_rq_buf_offset = qp_wrb_dat_1.wr_rq_buf_offset;
                end else begin
                    wrb_qp_number_c    = wrb_qp_number_r;
                    c_rq_buf_ca        = r_rq_buf_ca;
                    c_last_rq_req      = r_last_rq_req;
                    c_stat_qp_msn      = r_stat_qp_msn;
                    c_rq_wrptr_db      = r_rq_wrptr_db;
                    c_rq_buf_offset    = r_rq_buf_offset;
                    c_wr_rq_buf_offset = r_wr_rq_buf_offset;
                end
            end
            default: begin
                wrb_qp_number_c    = wrb_qp_number_r;
                c_rq_buf_ca        = r_rq_buf_ca;
                c_last_rq_req      = r_last_rq_req;
                c_stat_qp_msn      = r_stat_qp_msn;
                c_rq_wrptr_db      = r_rq_wrptr_db;
                c_rq_buf_offset    = r_rq_buf_offset;
                c_wr_rq_buf_offset = r_wr_rq_buf_offset;
            end
        endcase
    end


    always @(posedge clk) if (set_rsp_qp_fatal) rsp_qp_fatal_idx_r <= rsp_qp_fatal_idx;

    always @(posedge clk)
        if (reset) rsp_qp_fatal_flag <= 1'b0;
        else if (set_rsp_qp_fatal || clr_qp_fatal_flag) rsp_qp_fatal_flag <= set_rsp_qp_fatal;

    generate
        if (FATAL_PIPELINE_MODE == 0) begin  // current: FATAL_PIPELINE_MODE == 0

            // 当valid和ready都为高时，捕获致命QP索引
            always @(posedge clk)
                if (i_qp_fatal_wqe_proc_vld && qp_fatal_wqe_proc_rdy) begin  // wqe处理模块检测到致命错误
                    fatal_qp_idx <= i_qp_fatal_wqe_proc;
                end else if (i_qp_fatal_rsp_hndl_vld && qp_fatal_rsp_hndl_rdy) begin  // 响应处理模块检测到致命错误
                    fatal_qp_idx <= i_qp_fatal_rsp_hndl;
                end else if (!ext_fatal_case_flag) begin
                    fatal_qp_idx <= c3d.c2d.c1d.qp_number;
                end

            always @(*) begin
                qp_fatal_wqe_proc_vld = i_qp_fatal_wqe_proc_vld;
                qp_fatal_wqe_proc     = i_qp_fatal_wqe_proc;
                qp_fatal_rsp_hndl_vld = i_qp_fatal_rsp_hndl_vld;
                qp_fatal_rsp_hndl     = i_qp_fatal_rsp_hndl;
            end

        end else begin  // FATAL_PIPELINE_MODE == 1
            always @(posedge clk)
                if (qp_fatal_wqe_proc_vld && qp_fatal_wqe_proc_rdy) begin
                    fatal_qp_idx <= qp_fatal_wqe_proc;
                end else if (qp_fatal_rsp_hndl_vld && qp_fatal_rsp_hndl_rdy) begin
                    fatal_qp_idx <= qp_fatal_rsp_hndl;
                end else if (!ext_fatal_case_flag) begin
                    fatal_qp_idx <= c3d.c2d.c1d.qp_number;
                end
            //      fatal_qp_idx <= bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0];
            always @(posedge clk) begin
                if (reset) begin
                    qp_fatal_wqe_proc_vld <= 1'b0;
                    qp_fatal_wqe_proc     <= 'd0;
                    qp_fatal_rsp_hndl_vld <= 1'b0;
                    qp_fatal_rsp_hndl     <= 'd0;
                end else begin
                    qp_fatal_wqe_proc_vld <= i_qp_fatal_wqe_proc_vld;
                    qp_fatal_wqe_proc     <= i_qp_fatal_wqe_proc;
                    qp_fatal_rsp_hndl_vld <= i_qp_fatal_rsp_hndl_vld;
                    qp_fatal_rsp_hndl     <= i_qp_fatal_rsp_hndl;
                end
            end

        end
    endgenerate

    assign o_qp_fatal_wqe_proc_rdy = qp_fatal_wqe_proc_rdy;
    assign o_qp_fatal_rsp_hndl_rdy = qp_fatal_rsp_hndl_rdy;

    generate
        if (FATAL_PIPELINE_MODE == 0) begin  // current: FATAL_PIPELINE_MODE == 0
            // qp_fatal_wqe_proc_vld: WQE处理模块检测到致命错误
            // qp_fatal_rsp_hndl_vld: 响应处理模块检测到致命错误
            // FATAL_PIPELINE_MODE == 0时：qp_fatal_wqe_proc_vld = i_qp_fatal_wqe_proc_vld
            //                            qp_fatal_rsp_hndl_vld = i_qp_fatal_rsp_hndl_vld
            assign async_fatal_asserted = i_qp_fatal_wqe_proc_vld | i_qp_fatal_rsp_hndl_vld;

        end else begin  // FATAL_PIPELINE_MODE == 1
            assign async_fatal_asserted = qp_fatal_wqe_proc_vld | qp_fatal_rsp_hndl_vld;
        end
    endgenerate
    //===============================
    // Header Fetching Logic
    // 头部获取逻辑
    //===============================  
    always @(*) begin
        hdr_ftchg_fsm_ns = hdr_ftchg_fsm_cs;
        hdr_ftchg_done   = 1'b0;
        req_hdr_buf_enb  = 1'b0;
        //rd_req_pkt_desc_fifo = 1'b0;
        case (hdr_ftchg_fsm_cs)
            1'b0:  // 空闲状态
            if (!req_pkt_desc_fifo_empty && !(hv_fsm_busy | async_fatal_asserted)) begin
                hdr_ftchg_fsm_ns = 1'b1;
                req_hdr_buf_enb  = 1'b1;  // 开始读取头部
            end

            1'b1:  // 获取状态
            if (!(hv_fsm_busy | async_fatal_asserted)) begin
                hdr_ftchg_fsm_ns = 1'b0;
                hdr_ftchg_done   = 1'b1;  // 头部获取结束，但是这时还没有拼接好
                req_hdr_buf_enb  = 1'b1;  // 要读两次，两次头部数据拼接成完整头部
                //rd_req_pkt_desc_fifo = 1'b1;
            end
        endcase
    end

    always @(posedge clk)
        if (reset) hdr_ftchg_fsm_cs <= 1'b0;
        else hdr_ftchg_fsm_cs <= hdr_ftchg_fsm_ns;

    //assign rd_req_pkt_desc_fifo = (hdr_ftchg_fsm_cs==1'b1 && !(hv_fsm_busy| async_fatal_asserted))? 1'b1:1'b0; 

    always_ff @(posedge clk) begin
        if (reset) begin
            rd_req_pkt_desc_fifo        <= 1'b0;
            req_pkt_desc_fifo_dout_hold <= 'd0;
            rd_req_pkt_desc_fifo_done   <= 1'b0;
            data_missed                 <= 1'b0;
        end else begin
            if (hdr_ftchg_fsm_ns == 1'b1 && !(hv_fsm_busy | async_fatal_asserted)) begin
                rd_req_pkt_desc_fifo_done <= 1'b1;
            end else if (hdr_ftchg_fsm_ns == 1'b0 && !(hv_fsm_busy | async_fatal_asserted)) begin
                rd_req_pkt_desc_fifo_done <= 1'b0;
            end

            // 精确控制读取时机，避免数据丢失
            if (rd_req_pkt_desc_fifo_done == 1'b0 && hdr_ftchg_fsm_ns == 1'b1 && !(hv_fsm_busy | async_fatal_asserted)) begin
                rd_req_pkt_desc_fifo        <= 1'b1;
                req_pkt_desc_fifo_dout_hold <= req_pkt_desc_fifo_dout;  // 备份数据
            end else begin
                rd_req_pkt_desc_fifo <= 1'b0;
            end

            if (rd_req_pkt_desc_fifo && (hv_fsm_busy | async_fatal_asserted)) begin  // 如果在忙碌时尝试读取，标记数据丢失
                data_missed <= 1'b1;
            end else if (hdr_ftchg_fsm_ns == 1'b0 && !(hv_fsm_busy | async_fatal_asserted)) begin
                data_missed <= 1'b0;
            end
        end
    end

    always @(posedge clk)
        if (reset) begin
            rnr_window_active    <= 1'b0;
            pkt_src_addr_offset  <= 'd0;
            actual_pkt_len       <= 'd0;
            pkt_fcs_err          <= 1'b0;
            ipv4_hdr_chksum_fail <= 1'b0;
            pkt_crc_err          <= 1'b0;
        end else if (rd_req_pkt_desc_fifo && !(hv_fsm_busy | async_fatal_asserted)) begin
            rnr_window_active    <= req_pkt_desc_fifo_dout[16+C_REQ_BUF_ADDR_WIDTH];  // rnr 窗口
            pkt_src_addr_offset  <= req_pkt_desc_fifo_dout[16+:C_REQ_BUF_ADDR_WIDTH];  // 包描述符中的 缓冲区偏移 pkt_buf_offset 字段
            actual_pkt_len       <= req_pkt_desc_fifo_dout[15:3];  // 包长度
            pkt_fcs_err          <= req_pkt_desc_fifo_dout[2];  // FCS error
            ipv4_hdr_chksum_fail <= req_pkt_desc_fifo_dout[1];  // IPv4 Header checksum error
            pkt_crc_err          <= req_pkt_desc_fifo_dout[0];  // ICRC error
        end else if (data_missed == 1'b1 && hdr_ftchg_fsm_ns == 1'b0 && !(hv_fsm_busy | async_fatal_asserted)) begin  // 使用备份数据恢复
            // 使用req_pkt_desc_fifo_dout_hold恢复数据
            rnr_window_active    <= req_pkt_desc_fifo_dout_hold[16+C_REQ_BUF_ADDR_WIDTH];
            pkt_src_addr_offset  <= req_pkt_desc_fifo_dout_hold[16+:C_REQ_BUF_ADDR_WIDTH];
            actual_pkt_len       <= req_pkt_desc_fifo_dout_hold[15:3];
            pkt_fcs_err          <= req_pkt_desc_fifo_dout_hold[2];
            ipv4_hdr_chksum_fail <= req_pkt_desc_fifo_dout_hold[1];
            pkt_crc_err          <= req_pkt_desc_fifo_dout_hold[0];
        end

    always @(posedge clk)
        if (reset) hv_data_vld <= 1'b0;
        else if (!(hv_fsm_busy | async_fatal_asserted)) hv_data_vld <= hdr_ftchg_done;  // 头部数据可用，刚好两拍拼接完成，构成完整头部

    // 头部缓冲区地址计算
    always @(posedge clk)
        if (reset) req_hdr_buf_addrb <= {(C_HDR_BUF_ADDR_WIDTH) {1'b0}};
        else if (req_hdr_buf_enb) req_hdr_buf_addrb <= req_hdr_buf_addrb + 'd1;

    // 保存上一拍数据，头部数据需要两拍获取（因为reth需要两拍）
    always @(posedge clk) if (req_hdr_buf_enb) prev_req_hdr_buf_doutb <= req_hdr_buf_doutb;

    always @(posedge clk)
        if (hdr_ftchg_fsm_cs == 1'b0 && req_hdr_buf_enb == 1'b1) req_hdr_buf_enb_r <= 1'b1;
        else req_hdr_buf_enb_r <= 1'b0;

    // 拼接两拍的数据：{目前拍的数据，上一拍的数据}，形成完整的头部信息，同时 hv_data_vld 信号拉高
    assign hdr_data = {req_hdr_buf_doutb, prev_req_hdr_buf_doutb};

    generate
        if (C_SIM_DEBUG) begin  // C_SIM_DEBUG == 1
            always @(posedge clk)
                if (reset) begin
                    pkt_id <= 32'd0;
                end else if (rd_req_pkt_desc_fifo && !(hv_fsm_busy | async_fatal_asserted)) begin
                    pkt_id <= req_pkt_desc_fifo_dout[(C_RSP_BUF_ADDR_WIDTH+17)+:32];
                end else if (data_missed == 1'b1 && hdr_ftchg_fsm_ns == 1'b0 && !(hv_fsm_busy | async_fatal_asserted)) begin
                    pkt_id <= req_pkt_desc_fifo_dout_hold[(C_RSP_BUF_ADDR_WIDTH+17)+:32];
                end
        end else begin  // current: C_SIM_DEBUG == 0
            always @(posedge clk)
                if (reset) pkt_id <= 32'd0;
                else pkt_id <= 32'd0;
        end
    endgenerate

    assign bth_opcode_mem = req_hdr_qpid_buf_doutb[31:24];  // 从QPID缓冲区获取的BTH操作码

    generate
        if (C_EN_INITIATOR_LITE) begin  // current: C_EN_INITIATOR_LITE == 1
            ////////// RETH header specific checks -- RETH 标头特定检查 /////////////////////
            if (C_EN_IETH_IMMDT == 1) begin  // current: C_EN_IETH_IMMDT == 1
                assign has_reth_mem = ((bth_opcode_mem[4:0] == RDMA_WR_ONLY) || (bth_opcode_mem[4:0] == RDMA_WR_FIRST) || (bth_opcode_mem[4:0] == RDMA_RD_REQ) || (bth_opcode_mem[4:0] == RDMA_WR_ONLY_IMM));
            end else begin  // C_EN_IETH_IMMDT == 0
                assign has_reth_mem = ((bth_opcode_mem[4:0] == RDMA_WR_ONLY) || (bth_opcode_mem[4:0] == RDMA_WR_FIRST) || (bth_opcode_mem[4:0] == RDMA_RD_REQ));
            end
        end else begin  // C_EN_INITIATOR_LITE == 0
            assign has_reth_mem = 0;
        end
    endgenerate

    // 读取QP配置使能--当“头部有效“且qp“没有致命错误处理“时，允许读取QP配置
    assign read_qp_conf          = hv_data_vld & !(qp_fatal_wqe_proc_vld | qp_fatal_rsp_hndl_vld);
    assign rq_buf_ca_en          = read_qp_conf | rq_buf_ca_we;
    assign last_rq_req_en        = read_qp_conf | last_rq_req_we;
    assign stat_qp_msn_en        = read_qp_conf | stat_qp_msn_we;
    assign rq_wrptr_db_en        = read_qp_conf | rq_wrptr_db_we;
    assign rq_buf_offset_en      = read_qp_conf | rq_buf_offset_we;
    assign wr_rq_buf_offset_en   = read_qp_conf | wr_rq_buf_offset_we;

    // =============================== 协议头解析 ===============================
    // 以太网类型提取
    assign ether_type            = {hdr_data[12*8+:8], hdr_data[13*8+:8]};
    // BTH字段提取
    assign ipv4_bth_qp_dest_addr = {hdr_data[47*8+:8], hdr_data[48*8+:8], hdr_data[49*8+:8]} >= 24'd1 ? {hdr_data[47*8+:8], hdr_data[48*8+:8], hdr_data[49*8+:8]} - 1 : 24'd0;
    assign ipv6_bth_qp_dest_addr = {hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]} >= 24'd1 ? {hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]} - 1 : 24'd0;
    assign ipv4_bth_opcode       = hdr_data[42*8+:8];
    assign ipv6_bth_opcode       = hdr_data[62*8+:8];
    // IP版本判断
    assign ip_ver                = (ether_type == 16'h0800) ? IPv4 : IPv6;  // 0: IPv4, 1: IPv6

    // 根据IP版本选择相应字段
    assign reg_bth_qp_dest_addr  = (ip_ver == IPv4) ? ipv4_bth_qp_dest_addr : ipv6_bth_qp_dest_addr;
    assign reg_bth_opcode        = (ip_ver == IPv4) ? ipv4_bth_opcode : ipv6_bth_opcode;

    generate
        if (PIPELINE_MODE == 0) begin  // PIPELINE_MODE == 0
            always @(*) begin
                bth_qp_dest_addr      = (ip_ver == IPv4) ? ipv4_bth_qp_dest_addr : ipv6_bth_qp_dest_addr;
                bth_opcode            = (ip_ver == IPv4) ? ipv4_bth_opcode : ipv6_bth_opcode;
                pd_conf_en            = has_reth;
                o_qp_conf_idx         = read_qp_conf ? bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0] : wrb_qp_number;
                o_pd_conf_idx         = reth_r_key_w[8+:C_PD_INDX_WIDTH];
                qp_conf_en            = read_qp_conf;
                rq_buf_ba_en          = read_qp_conf;
                rq_wrptr_db_addr_en   = read_qp_conf;
                q_depth_en            = read_qp_conf;
                rq_ci_db_en           = read_qp_conf;
                remote_qp_mac_addr_en = read_qp_conf;
                remote_qp_ip_addr_en  = read_qp_conf;
            end

            assign o_rq_buf_ca        = d_rq_buf_ca;
            assign o_last_rq_req      = d_last_rq_req;
            assign o_stat_qp_msn      = d_stat_qp_msn;
            assign o_rq_wrptr_db      = d_rq_wrptr_db;
            assign o_rq_buf_offset    = d_rq_buf_offset;
            assign o_wr_rq_buf_offset = d_wr_rq_buf_offset;
        end // PIPELINE_MODE == 0 end
        
        else begin  // current: PIPELINE_MODE == 1

            // qp信息写回Initiator寄存器组
            assign o_rq_buf_ca        = r_rq_buf_ca;
            assign o_last_rq_req      = r_last_rq_req;
            assign o_stat_qp_msn      = r_stat_qp_msn;
            assign o_rq_wrptr_db      = r_rq_wrptr_db;
            assign o_rq_buf_offset    = r_rq_buf_offset;
            assign o_wr_rq_buf_offset = r_wr_rq_buf_offset;

            // 时序对齐
            always @(posedge clk) if (req_hdr_buf_enb_r) bth_qp_dest_addr <= req_hdr_qpid_buf_doutb[23:0];  // 从QPID缓冲区获取的QP ID（均指目的QP ID）
            always @(posedge clk) if (req_hdr_buf_enb_r) bth_opcode <= bth_opcode_mem;  // bth操作码
            always @(posedge clk) if (req_hdr_buf_enb_r) has_reth_mem_r <= has_reth_mem;  // 标志寄存器化
            always @(posedge clk) if (req_hdr_buf_enb_r) reth_r_key_r <= req_hdr_rkey_buf_doutb;  // r_key寄存器化

        end
    endgenerate

    generate
        if (FATAL_PIPELINE_MODE == 0) begin  // current: FATAL_PIPELINE_MODE == 0

            always @(*) begin
                pd_conf_en            = has_reth;
                o_pd_conf_idx         = reth_r_key_w[8+:C_PD_INDX_WIDTH];  // o_pd_conf_idx = reth_r_key_w[18:8], pd_number = reth_r_key_w[31:8]
                o_qp_conf_idx         = read_qp_conf ? bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0] : wrb_qp_number_r;
                qp_conf_en            = read_qp_conf;
                rq_buf_ba_en          = read_qp_conf;
                rq_wrptr_db_addr_en   = read_qp_conf;
                q_depth_en            = read_qp_conf;
                rq_ci_db_en           = read_qp_conf;
                remote_qp_mac_addr_en = read_qp_conf;
                remote_qp_ip_addr_en  = read_qp_conf;
            end

        end else begin  // FATAL_PIPELINE_MODE == 1
            always @(*) begin
                dbg_pd_conf_en            = has_reth;
                dbg_o_pd_conf_idx         = reth_r_key_w[8+:C_PD_INDX_WIDTH];
                dbg_o_qp_conf_idx         = read_qp_conf ? bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0] : wrb_qp_number_r;
                dbg_qp_conf_en            = read_qp_conf;
                dbg_rq_buf_ba_en          = read_qp_conf;
                dbg_rq_wrptr_db_addr_en   = read_qp_conf;
                dbg_q_depth_en            = read_qp_conf;
                dbg_rq_ci_db_en           = read_qp_conf;
                dbg_remote_qp_mac_addr_en = read_qp_conf;
                dbg_remote_qp_ip_addr_en  = read_qp_conf;
            end


            always @(posedge clk) begin
                if (reset) begin
                    o_qp_conf_idx         <= 'd0;
                    qp_conf_en            <= 1'b0;
                    rq_buf_ba_en          <= 1'b0;
                    rq_wrptr_db_addr_en   <= 1'b0;
                    q_depth_en            <= 1'b0;
                    rq_ci_db_en           <= 1'b0;
                    remote_qp_mac_addr_en <= 1'b0;
                    remote_qp_ip_addr_en  <= 1'b0;
                    o_pd_conf_idx         <= 'd0;
                    pd_conf_en            <= 1'b0;
                end else if ((hv_fsm_busy == 1'b0 && hdr_ftchg_done == 1'b1 && async_fatal_asserted == 1'b0) || (qp_conf_en == 1'b1 && hv_data_vld == 1'b1 && hv_fsm_busy == 1'b1 && i_qp_fatal_wqe_proc_vld == 1'b0 && i_qp_fatal_rsp_hndl_vld == 1'b0)) begin
                    o_qp_conf_idx         <= req_hdr_qpid_buf_doutb[C_QP_INDX_WIDTH-1:0];
                    qp_conf_en            <= 1'b1;
                    o_pd_conf_idx         <= req_hdr_rkey_buf_doutb[8+:C_PD_INDX_WIDTH];
                    pd_conf_en            <= has_reth_mem;
                    rq_buf_ba_en          <= 1'b1;
                    rq_wrptr_db_addr_en   <= 1'b1;
                    q_depth_en            <= 1'b1;
                    rq_ci_db_en           <= 1'b1;
                    remote_qp_mac_addr_en <= 1'b1;
                    remote_qp_ip_addr_en  <= 1'b1;
                end else begin
                    o_qp_conf_idx         <= wrb_qp_number_c;
                    qp_conf_en            <= 1'b0;
                    pd_conf_en            <= 1'b0;
                    rq_buf_ba_en          <= 1'b0;
                    rq_wrptr_db_addr_en   <= 1'b0;
                    q_depth_en            <= 1'b0;
                    rq_ci_db_en           <= 1'b0;
                    remote_qp_mac_addr_en <= 1'b0;
                    remote_qp_ip_addr_en  <= 1'b0;
                end
            end

        end
    endgenerate
    // EXPR_PIPELINE 

    // ===============================  

    //==============================================================================
    // 头部有效性检查
    //==============================================================================

    //////////// MAC头验证 //////////////
    // 包头目的MAC地址 != 本节点MAC地址
    assign mac_dest_addr_chk_fail = ({hdr_data[(0*8)+:8], hdr_data[(1*8)+:8], hdr_data[(2*8)+:8], hdr_data[(3*8)+:8], hdr_data[(4*8)+:8], hdr_data[(5*8)+:8]} != i_mac_node_addr) ? 1'b1 : 1'b0;


    //////////// IPv4头验证 //////////////  
    assign ipv4_ver_chk_fail = (hdr_data[(14*8+4)+:4] != 4'h4) ? 1'b1 : 1'b0;
    assign ipv4_ihl_chk_fail = (hdr_data[(14*8)+:4] != 4'h5) ? 1'b1 : 1'b0;  // IP 报头长度，仅支持20字节
    // assign ipv4_ecn_chk_fail         = (hdr_data[(15*8  )+: 2] ==  2'h3 ) ? 1'b1 : 1'b0;
    assign ipv4_ecn_chk_fail = 1'b0;
    assign ipv4_flg_chk_fail = (hdr_data[(20*8+5)+:3] != 3'h2) ? 1'b1 : 1'b0;
    assign ipv4_frag_offset_chk_fail = ({hdr_data[(20*8)+:5], hdr_data[(21*8)+:8]} != 13'h0) ? 1'b1 : 1'b0;
    assign ipv4_prot_chk_fail = (hdr_data[(23*8)+:8] != 8'h11) ? 1'b1 : 1'b0;
    assign ipv4_dest_addr_chk_fail = (i_ipv4_node_addr != {hdr_data[(30*8)+:8], hdr_data[(31*8)+:8], hdr_data[(32*8)+:8], hdr_data[(33*8)+:8]}) ? 1'b1 : 1'b0;
    assign ipv4_total_len = {hdr_data[16*8+:8], hdr_data[17*8+:8]};
    assign ipv4_pyld_len = ipv4_total_len - 16'd20;
    assign ipv4_length_chk_fail = ((ipv4_total_len >= 16'd44) && (ipv4_total_len <= 16'd4160)) ? 1'b0 : 1'b1;

    //////////// IPv6头验证 //////////////
    assign ipv6_ver_chk_fail = (hdr_data[(14*8+4)+:4] != 4'h6) ? 1'b1 : 1'b0;
    // assign ipv6_ecn_chk_fail       = (hdr_data[(15*8+4)+:2] ==  2'h3 ) ? 1'b1 : 1'b0;
    assign ipv6_ecn_chk_fail = 1'b0;
    assign ipv6_next_hdr_chk_fail = (hdr_data[(20*8)+:8] != 8'h11) ? 1'b1 : 1'b0;
    assign ipv6_dest_addr_chk_fail = (i_ipv6_node_addr != {
                   hdr_data[(38*8)+:8], hdr_data[(39*8)+:8], hdr_data[(40*8)+:8], 
                   hdr_data[(41*8)+:8], hdr_data[(42*8)+:8], hdr_data[(43*8)+:8],                                    
                   hdr_data[(44*8)+:8], hdr_data[(45*8)+:8], hdr_data[(46*8)+:8], 
                   hdr_data[(47*8)+:8], hdr_data[(48*8)+:8], hdr_data[(49*8)+:8],                                    
                   hdr_data[(50*8)+:8], hdr_data[(51*8)+:8], hdr_data[(52*8)+:8], 
                   hdr_data[(53*8)+:8]}) ? 1'b1 : 1'b0;
    assign ipv6_pyld_len = {hdr_data[18*8+:8], hdr_data[19*8+:8]};
    assign ipv6_length_chk_fail = ((ipv6_pyld_len >= 16'd16) && (ipv6_pyld_len <= 16'd4140)) ? 1'b0 : 1'b1;

    //////////// UDP头验证 //////////////
    assign ipv4_udp_dest_port_addr = {hdr_data[36*8+:8], hdr_data[37*8+:8]};
    assign ipv6_udp_dest_port_addr = {hdr_data[56*8+:8], hdr_data[57*8+:8]};
    assign ipv4_udp_len = {hdr_data[38*8+:8], hdr_data[39*8+:8]};
    assign ipv6_udp_len = {hdr_data[58*8+:8], hdr_data[59*8+:8]};
    assign udp_dest_port_addr = (ip_ver == IPv4) ? ipv4_udp_dest_port_addr : ipv6_udp_dest_port_addr;
    assign ip_pyld_len = (ip_ver == IPv4) ? ipv4_pyld_len : ipv6_pyld_len;
    assign udp_len = (ip_ver == IPv4) ? ipv4_udp_len : ipv6_udp_len;
    assign total_pkt_len_advt = (ip_ver == IPv4) ? (ipv4_udp_len + 16'd34) : (ipv6_udp_len + 16'd54);
    assign udp_dest_port_chk_fail = (udp_dest_port_addr != 16'd4791) ? 1'b1 : 1'b0;
    assign udp_length_chk_fail = (ip_pyld_len != udp_len) ? 1'b1 : 1'b0;
    assign pkt_len_chk_fail = ((total_pkt_len_advt != {3'b000, actual_pkt_len}) || (actual_pkt_len[1:0] != 2'b10)) ? 1'b1 : 1'b0;

    /////////// BTH头验证 //////////////
    assign ipv4_bth_tver = hdr_data[43*8+:4];
    assign ipv6_bth_tver = hdr_data[63*8+:4];
    assign ipv4_bth_expl_ack = hdr_data[50*8+7];
    assign ipv6_bth_expl_ack = hdr_data[70*8+7];
    assign ipv4_bth_pad_cnt = hdr_data[(43*8+4)+:2];
    assign ipv6_bth_pad_cnt = hdr_data[(63*8+4)+:2];
    assign ipv4_bth_psn = {hdr_data[51*8+:8], hdr_data[52*8+:8], hdr_data[53*8+:8]};
    assign ipv6_bth_psn = {hdr_data[71*8+:8], hdr_data[72*8+:8], hdr_data[73*8+:8]};

    // IPv4/v6 选择
    assign bth_ver = (ip_ver == IPv4) ? ipv4_bth_tver : ipv6_bth_tver;
    assign bth_expl_ack = (ip_ver == IPv4) ? ipv4_bth_expl_ack : ipv6_bth_expl_ack;
    assign bth_pad_cnt = (ip_ver == IPv4) ? ipv4_bth_pad_cnt : ipv6_bth_pad_cnt;
    assign bth_psn = (ip_ver == IPv4) ? ipv4_bth_psn : ipv6_bth_psn;

    assign ip_length_chk_fail = (ip_ver == IPv4) ? ipv4_length_chk_fail : ipv6_length_chk_fail;
    assign ip_dest_addr_chk_fail = (ip_ver == IPv4) ? ipv4_dest_addr_chk_fail : ipv6_dest_addr_chk_fail;
    assign ip_ecn_chk_fail = (ip_ver == IPv4) ? ipv4_ecn_chk_fail : ipv6_ecn_chk_fail;
    assign ip_ver_chk_fail = (ip_ver == IPv4) ? ipv4_ver_chk_fail : ipv6_ver_chk_fail;

    assign bth_ver_chk_fail = (bth_ver != 4'd0) ? 1'b1 : 1'b0;
    assign bth_pad_cnt_chk_fail = (((bth_opcode == {RC, SEND_FIRST}) || (bth_opcode == {RC, SEND_MIDDLE}) || (bth_opcode == {RC, RDMA_WR_FIRST}) || (bth_opcode == {RC, RDMA_WR_MIDDLE})) && (bth_pad_cnt != 2'd0)) ? 1'b1 : 1'b0;

    // 检查目的QP号是否在 RNIC 已配置 QP号的有效范围内
    // 启用QP要顺序启用，不支持离散 QP
    // tip: 启用QP数量为3时，启用的QP就不能是QP4及以后的QP，只能是QP1、QP2、QP3
    // 且 QP 启用数最大为 C_NUM_QP - 1，因为qpid-1改为qpid的原因，索引0为空的
    assign qp_conf_min = 24'd0;
    assign qp_conf_max = {{(24 - C_QP_INDX_WIDTH) {1'b0}}, i_num_qp_enabled};
    assign dest_qp_chk_fail = (bth_qp_dest_addr >= qp_conf_max) ? 1'b1 : 1'b0;

    generate
        if (C_EN_INITIATOR_LITE) begin  // current: C_EN_INITIATOR_LITE == 1
            ////////// RETH头验证 /////////////////////
            if (C_EN_IETH_IMMDT == 1) begin  // current: C_EN_IETH_IMMDT == 1
                assign has_reth                  = hv_data_vld & ((bth_opcode[4:0] == RDMA_WR_ONLY) || (bth_opcode[4:0] == RDMA_WR_FIRST) || (bth_opcode[4:0] == RDMA_RD_REQ) || (bth_opcode[4:0] == RDMA_WR_ONLY_IMM));
                assign has_ieth                  = hv_data_vld & ((bth_opcode[4:0] == SEND_ONLY_INV) || (bth_opcode[4:0] == SEND_LAST_INV));
                assign has_immdt                 = hv_data_vld & ((bth_opcode[4:0] == SEND_LAST_IMM) || (bth_opcode[4:0] == SEND_ONLY_IMM) || (bth_opcode[4:0] == RDMA_WR_ONLY_IMM) || (bth_opcode[4:0] == RDMA_WR_LAST_IMM));
                assign ipv4_ieth_immdt_data      = {hdr_data[55*8-1:54*8], hdr_data[56*8-1:55*8], hdr_data[57*8-1:56*8], hdr_data[58*8-1:57*8]};
                assign ipv6_ieth_immdt_data      = {hdr_data[75*8-1:74*8], hdr_data[76*8-1:75*8], hdr_data[77*8-1:76*8], hdr_data[78*8-1:77*8]};
                assign ipv4_reth_ieth_immdt_data = {hdr_data[71*8-1:70*8], hdr_data[72*8-1:71*8], hdr_data[73*8-1:72*8], hdr_data[74*8-1:73*8]};
                assign ipv6_reth_ieth_immdt_data = {hdr_data[91*8-1:90*8], hdr_data[92*8-1:91*8], hdr_data[93*8-1:92*8], hdr_data[94*8-1:93*8]};

            end else begin  // C_EN_IETH_IMMDT == 0
                assign has_reth                  = hv_data_vld & ((bth_opcode[4:0] == RDMA_WR_ONLY) || (bth_opcode[4:0] == RDMA_WR_FIRST) || (bth_opcode[4:0] == RDMA_RD_REQ));
                assign has_ieth                  = 1'b0;
                assign has_immdt                 = 1'b0;
                assign ipv4_ieth_immdt_data      = 32'd0;
                assign ipv6_ieth_immdt_data      = 32'd0;
                assign ipv4_reth_ieth_immdt_data = 32'd0;
                assign ipv6_reth_ieth_immdt_data = 32'd0;
            end

            assign ipv4_reth_va           = {hdr_data[54*8+:8], hdr_data[55*8+:8], hdr_data[56*8+:8], hdr_data[57*8+:8], hdr_data[58*8+:8], hdr_data[59*8+:8], hdr_data[60*8+:8], hdr_data[61*8+:8]};
            assign ipv6_reth_va           = {hdr_data[74*8+:8], hdr_data[75*8+:8], hdr_data[76*8+:8], hdr_data[77*8+:8], hdr_data[78*8+:8], hdr_data[79*8+:8], hdr_data[80*8+:8], hdr_data[81*8+:8]};
            assign ipv4_reth_r_key        = {hdr_data[62*8+:8], hdr_data[63*8+:8], hdr_data[64*8+:8], hdr_data[65*8+:8]};
            assign ipv6_reth_r_key        = {hdr_data[82*8+:8], hdr_data[83*8+:8], hdr_data[84*8+:8], hdr_data[85*8+:8]};
            assign ipv4_reth_dmalen       = {hdr_data[66*8+:8], hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]};
            assign ipv6_reth_dmalen       = {hdr_data[86*8+:8], hdr_data[87*8+:8], hdr_data[88*8+:8], hdr_data[89*8+:8]};
            assign reth_r_key_w           = (ip_ver == IPv4) ? ipv4_reth_r_key : ipv6_reth_r_key;
            assign reth_dma_len_chk_fail  = c3d.reth_dma_len_chk_fail;

            // 13'd58 = 14 (Ethernet header) + 20 (IPv4 header) + 8 (UDP header) + 16 (BTH header) + 4 (ICRC)
            assign bth_pyld_len_no_reth   = (ip_ver == IPv4) ? (actual_pkt_len - 13'd58) : (actual_pkt_len - 13'd78);
            // 13'd74 = 14 (Ethernet header) + 20 (IPv4 header) + 8 (UDP header) + 16 (BTH header) + 12 (RETH header) + 4 (ICRC)
            assign bth_pyld_len_with_reth = (ip_ver == IPv4) ? (actual_pkt_len - 13'd74) : (actual_pkt_len - 13'd94);

        end else begin  // C_EN_IETH_IMMDT == 0
            assign has_reth = 0;
            ////////// RETH头验证 /////////////////////
            if (C_EN_IETH_IMMDT == 1) begin  // current: C_EN_IETH_IMMDT == 1
                assign has_ieth                  = hv_data_vld && ((bth_opcode[4:0] == SEND_ONLY_INV) || (bth_opcode[4:0] == SEND_LAST_INV));
                assign has_immdt                 = hv_data_vld && ((bth_opcode[4:0] == SEND_LAST_IMM) || (bth_opcode[4:0] == SEND_ONLY_IMM));
                assign ipv4_ieth_immdt_data      = {hdr_data[55*8-1:54*8], hdr_data[56*8-1:55*8], hdr_data[57*8-1:56*8], hdr_data[58*8-1:57*8]};
                assign ipv6_ieth_immdt_data      = {hdr_data[75*8-1:74*8], hdr_data[76*8-1:75*8], hdr_data[77*8-1:76*8], hdr_data[78*8-1:77*8]};
                assign ipv4_reth_ieth_immdt_data = 32'd0;
                assign ipv6_reth_ieth_immdt_data = 32'd0;
            end else begin
                assign has_ieth                  = 1'b0;
                assign has_immdt                 = 1'b0;
                assign ipv4_ieth_immdt_data      = 32'd0;
                assign ipv6_ieth_immdt_data      = 32'd0;
                assign ipv4_reth_ieth_immdt_data = 32'd0;
                assign ipv6_reth_ieth_immdt_data = 32'd0;
            end

            assign ipv4_reth_va           = 0;
            assign ipv6_reth_va           = 0;
            assign ipv4_reth_r_key        = 0;
            assign ipv6_reth_r_key        = 0;
            assign ipv4_reth_dmalen       = 0;
            assign ipv6_reth_dmalen       = 0;
            assign reth_r_key_w           = 0;
            assign reth_dma_len_chk_fail  = 0;
            assign bth_pyld_len_no_reth   = 0;
            assign bth_pyld_len_with_reth = 0;
        end
    endgenerate  //C_EN_INITIATOR_LITE

    // 当hv_data_vld拉高时，检查结果才正确(时序正确)，接着保存正确的检查结果
    always @(posedge clk) begin
        if (hv_data_vld) begin
            // 保存 “基础包合法性校验“ 结果
            bth_pad_cnt_chk_fail_r <= bth_pad_cnt_chk_fail;  // BTH Pad
            dest_qp_chk_fail_r <= dest_qp_chk_fail;  // 不在qp范围内
            bth_ver_chk_fail_r <= bth_ver_chk_fail;  // BTH TVer
            pkt_len_chk_fail_r <= pkt_len_chk_fail;
            udp_length_chk_fail_r <= udp_length_chk_fail;
            ip_length_chk_fail_r <= ip_length_chk_fail;
            ipv4_hdr_chksum_fail_r <= (ip_ver == IPv4) ? ipv4_hdr_chksum_fail : 1'b0;
            ip_dest_addr_chk_fail_r <= ip_dest_addr_chk_fail;  // 目的 IP 是否等于 RNIC 配置的本地 IP
            ipv4_frag_offset_chk_fail_r <= (ip_ver == IPv4) ? ipv4_frag_offset_chk_fail : 1'b0;
            ipv4_flg_chk_fail_r <= (ip_ver == IPv4) ? ipv4_flg_chk_fail : 1'b0;
            ip_ecn_chk_fail_r <= ip_ecn_chk_fail;
            ipv4_ihl_chk_fail_r <= (ip_ver == IPv4) ? ipv4_ihl_chk_fail : 1'b0;
            ip_ver_chk_fail_r <= ip_ver_chk_fail;
            mac_dest_addr_chk_fail_r <= mac_dest_addr_chk_fail;  // 目的 MAC 是否等于 RNIC 配置的本地 MAC

            // 保存
            ip_ver_r <= ip_ver;
            bth_psn_r <= bth_psn;
            bth_psn_0 <= (bth_psn == 24'd0) ? 1'b1 : 1'b0;
            bth_opcode_r <= bth_opcode;
            bth_expl_ack_r <= bth_expl_ack;
            mac_src_addr_r <= {hdr_data[(6*8)+:8], hdr_data[(7*8)+:8], hdr_data[(8*8)+:8], hdr_data[(9*8)+:8], hdr_data[(10*8)+:8], hdr_data[(11*8)+:8]};
            ipv4_src_addr_r <= {hdr_data[(26*8)+:8], hdr_data[(27*8)+:8], hdr_data[(28*8)+:8], hdr_data[(29*8)+:8]};
            ipv6_src_addr_r <= {
                hdr_data[(22*8)+:8],
                hdr_data[(23*8)+:8],
                hdr_data[(24*8)+:8],
                hdr_data[(25*8)+:8],
                hdr_data[(26*8)+:8],
                hdr_data[(27*8)+:8],
                hdr_data[(28*8)+:8],
                hdr_data[(29*8)+:8],
                hdr_data[(30*8)+:8],
                hdr_data[(31*8)+:8],
                hdr_data[(32*8)+:8],
                hdr_data[(33*8)+:8],
                hdr_data[(34*8)+:8],
                hdr_data[(35*8)+:8],
                hdr_data[(36*8)+:8],
                hdr_data[(37*8)+:8]
            };
        end
    end

    assign i_nak_seq_err_sent  = nak_seq_err_sent_reg_combo;
    assign rnr_window_flag     = rnr_window_flag_reg[c3d_qp_number_r];

    assign req_pkt_id          = c3d.c2d.c1d.pkt_id;
    assign req_buf_addr_offset = c3d.c2d.c1d.pkt_src_addr_offset;
    assign req_qpid            = c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
    assign req_pkt_len         = c3d.req_pkt_len;
    assign req_ieth_present    = c3d.c2d.c1d.has_ieth;  // IETH头部标志
    assign req_immdt_present   = c3d.c2d.c1d.has_immdt;  // Immdt头部标志

    // BTH负载长度计算 & IETH即时数据提取
    generate
        if (C_EN_INITIATOR_LITE) begin  // current: C_EN_INITIATOR_LITE == 1
            always @(*)
                //Since RETH is 16 bytes in length, we offset this for the actual payload for packets with RETH
                if (C_EN_IETH_IMMDT == 0) begin
                    ieth_immdt_data = 32'd0;
                    bth_pyld_len    = ((bth_opcode[4:0] == RDMA_WR_FIRST) || (bth_opcode[4:0] == RDMA_WR_ONLY) || (bth_opcode[4:0] == RDMA_RD_REQ)) ? bth_pyld_len_with_reth : bth_pyld_len_no_reth;
                end else begin  // current: C_EN_IETH_IMMDT == 1
                    if ((bth_opcode[4:0] == RDMA_WR_FIRST) || (bth_opcode[4:0] == RDMA_WR_ONLY) || (bth_opcode[4:0] == RDMA_RD_REQ)) begin
                        bth_pyld_len    = bth_pyld_len_with_reth;
                        ieth_immdt_data = 32'd0;
                    end else if ((bth_opcode[4:0] == RDMA_WR_ONLY_IMM)) begin
                        bth_pyld_len    = bth_pyld_len_with_reth - 13'd4;
                        ieth_immdt_data = (ip_ver == IPv4) ? ipv4_reth_ieth_immdt_data : ipv6_reth_ieth_immdt_data;
                    end else if ((bth_opcode[4:0] == SEND_ONLY_IMM) || (bth_opcode[4:0] == SEND_LAST_IMM) || (bth_opcode[4:0] == SEND_ONLY_INV) || (bth_opcode[4:0] == SEND_LAST_INV) || (bth_opcode[4:0] == RDMA_WR_LAST_IMM)) begin
                        bth_pyld_len    = bth_pyld_len_no_reth - 13'd4;
                        ieth_immdt_data = (ip_ver == IPv4) ? ipv4_ieth_immdt_data : ipv6_ieth_immdt_data;
                    end else begin
                        bth_pyld_len    = bth_pyld_len_no_reth;
                        ieth_immdt_data = 32'd0;
                    end
                end
        end else begin  // C_EN_INITIATOR_LITE == 0
            always @(*)
                if (C_EN_IETH_IMMDT == 0) begin
                    bth_pyld_len    = (ip_ver == IPv4) ? (actual_pkt_len - 13'd58) : (actual_pkt_len - 13'd78);
                    ieth_immdt_data = 32'd0;
                end else begin
                    if ((bth_opcode[4:0] == SEND_ONLY_IMM) || (bth_opcode[4:0] == SEND_LAST_IMM) || (bth_opcode[4:0] == SEND_ONLY_INV) || (bth_opcode[4:0] == SEND_LAST_INV)) begin
                        bth_pyld_len    = (ip_ver == IPv4) ? (actual_pkt_len - 13'd62) : (actual_pkt_len - 13'd82);
                        ieth_immdt_data = (ip_ver == IPv4) ? ipv4_ieth_immdt_data : ipv6_ieth_immdt_data;
                    end else begin
                        bth_pyld_len    = (ip_ver == IPv4) ? (actual_pkt_len - 13'd58) : (actual_pkt_len - 13'd78);
                        ieth_immdt_data = 32'd0;
                    end
                end
        end
    endgenerate  //C_EN_INITIATOR_LITE

    // ECN 检测
    // IP头部的ECN字段：
    //     00 – 非 ECN 能力传输，非 ECT
    //     10 – 支持 ECN 的传输，ECT(0)
    //     01 – 支持 ECN 的传输，ECT(1)
    //     11 – 遇到拥塞，CE。
    always @(*) begin
        //nxt_ecn_vect = {(C_NUM_QP){1'b0}};
        nxt_ecn_vect_idx   = {(C_QP_INDX_WIDTH) {1'b0}};
        nxt_ecn_update_flg = 1'b0;
        if (ip_ver == IPv4) begin
            if (hdr_data[(15*8)+:2] == 2'h3) begin  // bits1~0(ECT & CE)为ECN字段
                //nxt_ecn_vect[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]] = 1'b1;
                nxt_ecn_vect_idx   = bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];  // QP ID
                nxt_ecn_update_flg = 1'b1;
            end
        end else if (hdr_data[(15*8+4)+:2] == 2'h3) begin  // IPv6
            //nxt_ecn_vect[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]] = 1'b1;
            nxt_ecn_vect_idx   = bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];
            nxt_ecn_update_flg = 1'b1;
        end
    end

    // 只有在 “包整体有效且源地址校验通过” 时才上报 ECN
    assign ecn_vect_vld             = (c3d.c2d.c1d.ecn_update_flg & ~(pkt_vldn_err || src_addr_chk_fail)) ? 1'b1 : 1'b0;
    //assign ecn_vect = c3d.c2d.c1d.ecn_vect;
    assign ecn_vect_idx             = c3d.c2d.c1d.ecn_vect_idx;

    // always @(posedge clk)
    //     if (reset) bth_qp_dest_addr_r <= 24'h00_0000;
    //     else if (hv_data_vld) bth_qp_dest_addr_r <= c3d.c2d.c1d.qp_number;

    assign ipv4_src_addr_chk_fail   = (i_remote_qp_ip_addr[31:0] != c1d.ipv4_src_addr) ? 1'b1 : 1'b0;
    assign ipv6_src_addr_chk_fail   = (i_remote_qp_ip_addr != c1d.ipv6_src_addr) ? 1'b1 : 1'b0;
    assign ip_src_addr_chk_fail_w   = (ip_ver_r == IPv4) ? ipv4_src_addr_chk_fail : ipv6_src_addr_chk_fail;
    assign mac_src_addr_chk_fail_w  = (i_remote_qp_mac_addr != mac_src_addr_r) ? 1'b1 : 1'b0;

    assign memory_access_permission = c3d.c2d.pd_conf_data.access_type;
    assign memory_access_read       = (memory_access_permission == 2'b00) || (memory_access_permission == 2'b10);  // 读请求要求 memory_access_read==1
    assign memory_access_write      = (memory_access_permission == 2'b01) || (memory_access_permission == 2'b10);  // 写请求要求 memory_access_write==1
    //assign qp_en = qp_conf_vld ? i_qp_conf[0] : c2d.qp_conf_data.qp_conf[0];
    //assign qp_en_for_fsm = c3d.c2d.qp_conf_data.qp_conf[2];
    assign qp_rq_intr_en            = c3d.c2d.qp_conf_data.qp_conf[2];  // RQ 中断启用 - 启用时，允许为接收队列中收到的每个新数据包生成接收队列中断
    assign qp_hw_hndshk_dis         = c3d.c2d.qp_conf_data.qp_conf[4];  // 禁止硬件握手 - 1：禁用硬件门铃握手，门铃更新通过AXI/AXIL进行交换；0：启用硬件门铃握手
    //assign qp_pmtu = qp_conf_vld ? i_qp_conf[10:8] : qp_pmtu_r;
    //assign qp_rq_buf_sz = qp_conf_vld ? i_qp_conf[31:16] : qp_rq_buf_sz_r;
    assign rq_buf_ba                = c3d.c2d.qp_conf_data.rq_buf_ba;
    assign rq_buf_ca                = c3d.c2d.qp_conf_data.rq_buf_ca;
    assign rq_wrptr_db_addr         = c3d.c2d.qp_conf_data.rq_wrptr_db_addr;
    assign q_depth                  = c3d.c2d.qp_conf_data.q_depth;
    assign rq_ci_db                 = c3d.c2d.qp_conf_data.rq_ci_db;
    //assign last_req_opcode = qp_conf_vld ? i_last_rq_req[31:24] : last_req_opcode_r;
    assign last_req_psn             = c3d.c2d.qp_conf_data.last_rq_req[23:0];
    assign p_last_req_psn           = c2d.qp_conf_data.last_rq_req[23:0];
    assign stat_qp_msn              = c3d.c2d.qp_conf_data.stat_qp_msn;
    assign ip_src_addr_chk_fail     = c3d.c2d.ip_src_addr_chk_fail;
    assign mac_src_addr_chk_fail    = c3d.c2d.mac_src_addr_chk_fail;
    assign src_addr_chk_fail        = ip_src_addr_chk_fail | mac_src_addr_chk_fail;
    assign rq_wrptr_db              = c3d.c2d.qp_conf_data.rq_wrptr_db;
    //assign qp_state_fatal = (!(qp_conf_data.qp_conf[0])|| i_qp_fatal[c1d.qp_number]) ? 1'b1 : 1'b0;
    // qp_conf[0]：QP 启用 - 所有活动 QP 均应设置为 1。禁用的 QP 将无法接收或发送数据包。
    // qp_conf[6]：恢复中的 QP。该位需要在致命清除过程中设置。
    assign qp_state_fatal           = (!(qp_conf_data.qp_conf[0]) || qp_fatal_combo_c1d || qp_recovery_combo_c1d || qp_conf_data.qp_conf[6]) ? 1'b1 : 1'b0;
    assign mad_pkt_len_err          = c3d.c2d.c1d.mad_pkt_len_err;

    // [10:8]: Path MTU
    // ° 000 – 256B (default) 
    // ° 001 – 512B 
    // ° 010 – 1024B
    // ° 011 – 2048B
    // ° 100 - 4096B
    // ° 101 to 111 - Reserved
    assign qp_pmtu                  = qp_conf_data.qp_conf[10:8];

    assign req_rq_pi_db             = qp_wrb_dat_w.rq_wrptr_db;

    // 将 QP 配置里的 3bit PMTU 编码解码成实际字节数（256/512/1024/2048/4096）
    always @(*)
        case (qp_pmtu)
            3'b000:  qp_pmtu_decoded = 13'd0256;
            3'b001:  qp_pmtu_decoded = 13'd0512;
            3'b010:  qp_pmtu_decoded = 13'd1024;
            3'b011:  qp_pmtu_decoded = 13'd2048;
            3'b100:  qp_pmtu_decoded = 13'd4096;
            default: qp_pmtu_decoded = 13'd0256;
        endcase

    // 传输层有效载荷长度超过了为该 QP 配置的 PMTU，write_pkt_len_err 拉高
    // ================ WRITE 类报文的“分片长度合法性检查” ================
    // ONLY 不超限，FIRST/MIDDLE 必须满 PMTU，LAST 应该是尾包长度范围检查
    generate
        if (C_EN_INITIATOR_LITE) begin
            // 检查写入数据包长度错误
            if (C_EN_IETH_IMMDT == 0) begin
                always @(*) begin
                    write_pkt_len_err = 1'b0;
                    case (c2d.c1d.bth_opcode[4:0])
                        RDMA_WR_ONLY: if ((c2d.c1d.bth_pyld_len > c2d.qp_pmtu_decoded) || (c2d.c1d.bth_pyld_len > c2d.pd_conf_data.w_r_buf_len)) write_pkt_len_err = 1'b1;

                        RDMA_WR_FIRST: if ((c2d.c1d.bth_pyld_len != c2d.qp_pmtu_decoded) || (c2d.pd_conf_data.w_r_buf_len == c2d.qp_pmtu_decoded)) write_pkt_len_err = 1'b1;

                        RDMA_WR_MIDDLE: if ((c2d.c1d.bth_pyld_len != c2d.qp_pmtu_decoded)  /*|| 
                       (c2d.pd_conf_data.w_r_buf_len == ({c2d.qp_conf_data.wr_rq_buf_offset,8'd0}+c2d.qp_pmtu_decoded))*/) write_pkt_len_err = 1'b1;

                        RDMA_WR_LAST: if ((c2d.c1d.bth_pyld_len > c2d.qp_pmtu_decoded) && (c2d.c1d.bth_pyld_len < 13'd1)) write_pkt_len_err = 1'b1;
                        default: write_pkt_len_err = 1'b0;
                    endcase
                end
            end else begin  // current: C_EN_IETH_IMMDT == 1
                always @(*) begin
                    write_pkt_len_err = 1'b0;
                    case (c2d.c1d.bth_opcode[4:0])
                        RDMA_WR_ONLY, RDMA_WR_ONLY_IMM: if ((c2d.c1d.bth_pyld_len > c2d.qp_pmtu_decoded) || (c2d.c1d.bth_pyld_len > c2d.pd_conf_data.w_r_buf_len)) write_pkt_len_err = 1'b1;

                        RDMA_WR_FIRST: if ((c2d.c1d.bth_pyld_len != c2d.qp_pmtu_decoded) || (c2d.pd_conf_data.w_r_buf_len == c2d.qp_pmtu_decoded)) write_pkt_len_err = 1'b1;

                        RDMA_WR_MIDDLE: if ((c2d.c1d.bth_pyld_len != c2d.qp_pmtu_decoded)  /*|| 
                       (c2d.pd_conf_data.w_r_buf_len == ({c2d.qp_conf_data.wr_rq_buf_offset,8'd0}+c2d.qp_pmtu_decoded))*/) write_pkt_len_err = 1'b1;

                        RDMA_WR_LAST, RDMA_WR_LAST_IMM: if ((c2d.c1d.bth_pyld_len > c2d.qp_pmtu_decoded) && (c2d.c1d.bth_pyld_len < 13'd1)) write_pkt_len_err = 1'b1;
                        default: write_pkt_len_err = 1'b0;
                    endcase
                end
            end
        end else begin
            assign write_pkt_len_err = 0;
        end
    endgenerate

    // 这一段组合逻辑会在第二阶段流水线里比较 BTH 负载长度，确保其与 QP 配置的 PMTU、接收缓冲区尺寸/偏移相符；
    // 发现 SEND_ONLY/FIRST/MIDDLE/LAST（含 IMMDT、INV 变体）超出 PMTU、或在分片过程中偏移已到达缓冲区末尾，则把 send_pkt_len_err 置 1，以便后续流程判定该请求非法并走 NAK/错误路径。
    // ----------------------------------------------------------------------------------------------------------------------------------------------------------
    assign qp_rq_buf_sz = c2d.qp_conf_data.qp_conf[31:16];  // RQ 缓冲区中每个元素的大小（按256B倍数对齐）。

    //Check the payload length
    // ================ SEND 类报文的“分片长度合法性检查” ================
    // 保证 SEND 分片序列满足“FIRST/MIDDLE 必须满 PMTU、ONLY/LAST 不超 PMTU且不越 RQ 容量”的规则
    generate
        if (C_EN_IETH_IMMDT == 0) begin
            always @(*) begin
                send_pkt_len_err = 1'b0;

                case (c2d.c1d.bth_opcode[4:0])
                    SEND_ONLY: if ((c2d.c1d.bth_pyld_len > c2d.qp_pmtu_decoded) || (c2d.c1d.bth_pyld_len > {qp_rq_buf_sz, 8'd0})) send_pkt_len_err = 1'b1;

                    SEND_FIRST: if ((c2d.c1d.bth_pyld_len != c2d.qp_pmtu_decoded) || (qp_rq_buf_sz == c2d.qp_pmtu_decoded[12:8])) send_pkt_len_err = 1'b1;

                    SEND_MIDDLE: if ((c2d.c1d.bth_pyld_len != c2d.qp_pmtu_decoded) || (qp_rq_buf_sz == (c2d.qp_conf_data.rq_buf_offset + c2d.qp_pmtu_decoded[12:8]))) send_pkt_len_err = 1'b1;

                    SEND_LAST: if ((c2d.c1d.bth_pyld_len > c2d.qp_pmtu_decoded) && (c2d.c1d.bth_pyld_len < 13'd1)) send_pkt_len_err = 1'b1;
                    default:   send_pkt_len_err = 1'b0;
                endcase
            end
        end else begin  // current: C_EN_IETH_IMMDT == 1
            always @(*) begin
                send_pkt_len_err = 1'b0;

                case (c2d.c1d.bth_opcode[4:0])
                    // {qp_rq_buf_sz, 8'd0}: RQ 缓冲区中每个元素按256B倍数对齐
                    SEND_ONLY, SEND_ONLY_IMM, SEND_ONLY_INV: if ((c2d.c1d.bth_pyld_len > c2d.qp_pmtu_decoded) || (c2d.c1d.bth_pyld_len > {qp_rq_buf_sz, 8'd0})) send_pkt_len_err = 1'b1;

                    SEND_FIRST: if ((c2d.c1d.bth_pyld_len != c2d.qp_pmtu_decoded) || (qp_rq_buf_sz == c2d.qp_pmtu_decoded[12:8])) send_pkt_len_err = 1'b1;

                    SEND_MIDDLE: if ((c2d.c1d.bth_pyld_len != c2d.qp_pmtu_decoded) || (qp_rq_buf_sz == (c2d.qp_conf_data.rq_buf_offset + c2d.qp_pmtu_decoded[12:8]))) send_pkt_len_err = 1'b1;

                    SEND_LAST, SEND_LAST_IMM, SEND_LAST_INV: if ((c2d.c1d.bth_pyld_len > c2d.qp_pmtu_decoded) && (c2d.c1d.bth_pyld_len < 13'd1)) send_pkt_len_err = 1'b1;
                    default: send_pkt_len_err = 1'b0;
                endcase
            end
        end
    endgenerate  //if (C_EN_IETH_IMMDT) 

    // 这段 generate 逻辑根据上一次完成的请求 (last_req_opcode) 与当前收到的 BTH opcode 是否符合 RDMA/RoCE 对分段传输的时序要求来置位 opcode_seq_chk_fail。
    // 若遇到非法顺序（例如某个 MIDDLE 没有对应的 FIRST 先行、或上次还在 RDMA_WR 分片中却收到新的 SEND_ONLY），便将 opcode_seq_chk_fail 拉高，供后续判定该请求非法并上报错误/NACK。
    assign last_req_opcode = qp_conf_data.last_rq_req[31:24];
    ///检查 OpCode字段 的顺序
    // ================ 分片时序是否正确（FIRST -> MIDDLE -> LAST 顺序） ================
    generate
        if (C_EN_INITIATOR_LITE) begin  // current: C_EN_INITIATOR_LITE == 1

            if (C_EN_IETH_IMMDT == 0) begin
                always @(*) begin
                    opcode_seq_chk_fail = 1'b0;
                    case (c1d.bth_opcode[4:0])
                        SEND_ONLY, SEND_FIRST, RDMA_WR_ONLY, RDMA_WR_FIRST, RDMA_RD_REQ: if ((last_req_opcode[4:0] == SEND_FIRST) || (last_req_opcode[4:0] == SEND_MIDDLE) || (last_req_opcode[4:0] == RDMA_WR_FIRST) || (last_req_opcode[4:0] == RDMA_WR_MIDDLE)) opcode_seq_chk_fail = 1'b1;

                        SEND_MIDDLE, SEND_LAST: if ((last_req_opcode[4:0] != SEND_FIRST) && (last_req_opcode[4:0] != SEND_MIDDLE)) opcode_seq_chk_fail = 1'b1;

                        RDMA_WR_MIDDLE, RDMA_WR_LAST: if ((last_req_opcode[4:0] != RDMA_WR_FIRST) && (last_req_opcode[4:0] != RDMA_WR_MIDDLE)) opcode_seq_chk_fail = 1'b1;

                        default: opcode_seq_chk_fail = 1'b0;
                    endcase
                end
            end // if (C_EN_IETH_IMMDT == 0)
            
            // ---------------------------------------- 执行段 --------------------------------------------
            else begin  // current: C_EN_IETH_IMMDT == 1
                always @(*) begin
                    opcode_seq_chk_fail = 1'b0;
                    case (c1d.bth_opcode[4:0])
                        SEND_ONLY, SEND_FIRST, RDMA_WR_ONLY, RDMA_WR_FIRST, RDMA_RD_REQ, SEND_ONLY_IMM, SEND_ONLY_INV, RDMA_WR_ONLY_IMM:
                        if ((last_req_opcode[4:0] == SEND_FIRST) || (last_req_opcode[4:0] == SEND_MIDDLE) || (last_req_opcode[4:0] == RDMA_WR_FIRST) || (last_req_opcode[4:0] == RDMA_WR_MIDDLE)) opcode_seq_chk_fail = 1'b1;

                        SEND_MIDDLE, SEND_LAST, SEND_LAST_IMM, SEND_LAST_INV: if ((last_req_opcode[4:0] != SEND_FIRST) && (last_req_opcode[4:0] != SEND_MIDDLE)) opcode_seq_chk_fail = 1'b1;

                        RDMA_WR_MIDDLE, RDMA_WR_LAST, RDMA_WR_LAST_IMM: if ((last_req_opcode[4:0] != RDMA_WR_FIRST) && (last_req_opcode[4:0] != RDMA_WR_MIDDLE)) opcode_seq_chk_fail = 1'b1;

                        default: opcode_seq_chk_fail = 1'b0;
                    endcase
                end
            end  // else C_EN_IETH_IMMDT == 1

        end // if (C_EN_INITIATOR_LITE)

        else begin  // C_EN_INITIATOR_LITE == 0

            if (C_EN_IETH_IMMDT == 0) begin
                always @(*) begin
                    opcode_seq_chk_fail = 1'b0;
                    case (c1d.bth_opcode[4:0])
                        SEND_ONLY, SEND_FIRST: if ((last_req_opcode[4:0] != SEND_ONLY) && (last_req_opcode[4:0] != SEND_LAST)) opcode_seq_chk_fail = 1'b1;

                        SEND_MIDDLE, SEND_LAST: if ((last_req_opcode[4:0] != SEND_FIRST) && (last_req_opcode[4:0] != SEND_MIDDLE)) opcode_seq_chk_fail = 1'b1;
                        default: opcode_seq_chk_fail = 1'b0;
                    endcase
                end
            end // if (C_EN_IETH_IMMDT == 0)

            else begin  // C_EN_IETH_IMMDT == 1
                always @(*) begin
                    opcode_seq_chk_fail = 1'b0;
                    case (c1d.bth_opcode[4:0])
                        SEND_ONLY, SEND_FIRST, SEND_ONLY_INV, SEND_ONLY_IMM:
                        if ((last_req_opcode[4:0] != SEND_ONLY) && (last_req_opcode[4:0] != SEND_LAST) && (last_req_opcode[4:0] != SEND_ONLY_INV) && (last_req_opcode[4:0] != SEND_LAST_INV) && (last_req_opcode[4:0] != SEND_ONLY_IMM) && (last_req_opcode[4:0] != SEND_LAST_IMM))
                            opcode_seq_chk_fail = 1'b1;

                        SEND_MIDDLE, SEND_LAST, SEND_LAST_INV, SEND_LAST_IMM: if ((last_req_opcode[4:0] != SEND_FIRST) && (last_req_opcode[4:0] != SEND_MIDDLE)) opcode_seq_chk_fail = 1'b1;
                        default: opcode_seq_chk_fail = 1'b0;
                    endcase
                end
            end  // else C_EN_IETH_IMMDT == 1

        end  // else C_EN_INITIATOR_LITE == 0
    endgenerate  //C_EN_INITIATOR_LITE

    assign err_syndr = {
        pkt_fcs_err,  // bit 31
        pkt_crc_err,  // bit 30 
        1'b0,  // bit 29
        1'b0,  // bit 28
        4'b0,  // bit 24-27
        4'b0,  // bit 20-23
        1'b0,  // bit 19
        bth_pad_cnt_chk_fail_r,  // bit 18
        1'b0,  // bit 17
        1'b0,  // bit 16
        qp_state_fatal,  // bit 15
        dest_qp_chk_fail_r,  // bit 14 
        bth_ver_chk_fail_r,  // bit 13
        pkt_len_chk_fail_r,  // bit 12
        udp_length_chk_fail_r,  // bit 11
        ip_length_chk_fail_r,  // bit 10
        ipv4_hdr_chksum_fail_r,  // bit 9
        ip_dest_addr_chk_fail_r,  // bit 8
        1'b0,  // bit 7
        ipv4_frag_offset_chk_fail_r,  // bit 6
        ipv4_flg_chk_fail_r,  // bit 5
        ip_ecn_chk_fail_r,  // bit 4
        ipv4_ihl_chk_fail_r,  // bit 3
        ip_ver_chk_fail_r,  // bit 2
        1'b0,  // bit 1
        mac_dest_addr_chk_fail_r  // bit 0
    };

    always @(posedge clk)
        if (hv_data_vld)
            pkt_vldn_err_1 <= pkt_fcs_err | pkt_crc_err |
            //i_qp_fatal[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]] |
            (qp_fatal_combo || qp_recovery_combo)   |
            dest_qp_chk_fail |
            bth_ver_chk_fail |
            pkt_len_chk_fail |
            udp_length_chk_fail |
            ip_length_chk_fail |
            (~ip_ver & ipv4_hdr_chksum_fail) |
            ip_dest_addr_chk_fail |
            (~ip_ver & ipv4_frag_offset_chk_fail) |
            (~ip_ver & ipv4_flg_chk_fail) |
            ip_ecn_chk_fail |
            (~ip_ver & ipv4_ihl_chk_fail) |
            ip_ver_chk_fail |
            mac_dest_addr_chk_fail;

    assign pkt_vldn_err       = c3d.c2d.pkt_vldn_err;

    assign exp_req_psn        = last_req_psn + 'd1;  // 期望的PSN
    assign p_exp_req_psn      = p_last_req_psn + 'd1;

    assign is_req_psn_matched = c3d.is_req_psn_matched;  // PSN匹配检查

    //  always @ (posedge clk)
    //  begin
    //  rq_buf_full <= ((c2d.qp_conf_data.rq_ci_db == c2d.qp_conf_data.rq_wrptr_db + 16'd1) ||
    //                 ((c2d.qp_conf_data.rq_ci_db + c2d.qp_conf_data.q_depth) == (c2d.qp_conf_data.rq_wrptr_db + 16'd1))) ? 1'b1 : 1'b0;
    //
    //  end
    assign rq_buf_full        = c3d.rq_buf_full;
    /*assign rq_buf_full = ((c3d.c2d.qp_conf_data.rq_ci_db == c3d.c2d.qp_conf_data.rq_wrptr_db + 16'd1) ||
                        ((c3d.c2d.qp_conf_data.rq_ci_db + c3d.c2d.qp_conf_data.q_depth) == (c3d.c2d.qp_conf_data.rq_wrptr_db + 16'd1))) ? 1'b1 : 1'b0;*/
    // logic [C_NUM_QP:0] j;
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP FATAL LUTRAM START //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    // init_start 时按 init_count 把全体 QP 清 0（初始化）。这里是为了同时服务两套读地址，因为 sdpram 本质是单读口，想同周期读两个不同地址，最简单可靠是复制两份内容

    // 这一套读 bth_qp_dest_addr（当前包目标 QP），用于包有效性判断：pkt_vldn_err_1 里直接用 qp_fatal_combo || qp_recovery_combo
    // 存每个 QP 的 fatal 位，读出 qp_fatal_combo
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_fatal_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (init_start || i_qp_fatal_wr_en),
        .wr_addr (init_start ? init_count : i_qp_fatal_wr_idx),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]),
        .rd_data (qp_fatal_combo)
    );

    // 存每个 QP 的 recovery 位，读出 qp_recovery_combo
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_recovery_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (init_start || (qp_under_recovery_wr_en && !qp_under_recovery_wr_en_r)),  //qp_under_recovery is a pulse only
        .wr_addr (init_start ? init_count : qp_under_recovery_wr_idx),
        .wr_data (init_start ? 1'b0 : qp_under_recovery_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]),
        .rd_data (qp_recovery_combo)
    );


    // 这一套读 c1d.qp_number（流水线前级 QP），用于 qp_state_fatal 组合判断
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_recovery_lutram_1 (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (init_start || (qp_under_recovery_wr_en && !qp_under_recovery_wr_en_r)),  //qp_under_recovery is a pulse only
        .wr_addr (init_start ? init_count : qp_under_recovery_wr_idx),
        .wr_data (init_start ? 1'b0 : qp_under_recovery_wr_data),
        .rd_en   (1'b1),
        .rd_addr (c1d.qp_number[C_QP_INDX_WIDTH-1:0]),
        .rd_data (qp_recovery_combo_c1d)
    );

    always_ff @(posedge clk) qp_under_recovery_wr_en_r <= qp_under_recovery_wr_en;

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_fatal_lutram_c1d (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (init_start || i_qp_fatal_wr_en),
        .wr_addr (init_start ? init_count : i_qp_fatal_wr_idx),
        .wr_data (init_start ? 1'b0 : i_qp_fatal_wr_data),
        .rd_en   (1'b1),
        .rd_addr (c1d.qp_number[C_QP_INDX_WIDTH-1:0]),
        .rd_data (qp_fatal_combo_c1d)
    );
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP FATAL LUTRAM END ////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    always @(posedge clk) begin
        if (reset) begin
            //o_rq_full <= {(C_NUM_QP){1'b0}};
            o_rq_full_idx         <= {(C_QP_INDX_WIDTH) {1'b0}};
            o_rq_full_wr_en       <= 1'b0;
            o_rq_full_val         <= 1'b0;
            o_rq_full_reset_idx   <= {(C_QP_INDX_WIDTH) {1'b0}};
            o_rq_full_reset_wr_en <= 1'b0;
            o_rq_full_reset_val   <= 1'b0;
        end else begin
            o_rq_full_wr_en       <= set_rq_full_flag || clr_rq_full_flag;
            o_rq_full_val         <= set_rq_full_flag;
            o_rq_full_idx         <= c3d.c2d.c1d.qp_number;
            o_rq_full_reset_wr_en <= i_qp_fatal_wr_en && i_qp_fatal_wr_data;
            o_rq_full_reset_val   <= 1'b0;
            o_rq_full_reset_idx   <= i_qp_fatal_wr_idx;
            //if (set_rq_full_flag || clr_rq_full_flag)
            //    //o_rq_full[c3d.c2d.c1d.qp_number] <= set_rq_full_flag;
            //    o_rq_full[c3d.c2d.c1d.qp_number] <= set_rq_full_flag;
            //else if (|i_qp_fatal)
            //    o_rq_full <= o_rq_full & ~i_qp_fatal;
        end
    end

    generate
        if (C_EN_INITIATOR_LITE) begin
            always @(*)
                case (qp_pmtu)
                    3'b000: begin
                        qp_psn_coef_decoded = c1d.reth_dmalen[31:8];
                        qp_psn_rem_decoded  = (c1d.reth_dmalen[7:0] != 0) ? 1 : 0;
                    end
                    3'b001: begin
                        qp_psn_coef_decoded = c1d.reth_dmalen[31:9];
                        qp_psn_rem_decoded  = (c1d.reth_dmalen[8:0] != 0) ? 1 : 0;
                    end
                    3'b010: begin
                        qp_psn_coef_decoded = c1d.reth_dmalen[31:10];
                        qp_psn_rem_decoded  = (c1d.reth_dmalen[9:0] != 0) ? 1 : 0;
                    end
                    3'b011: begin
                        qp_psn_coef_decoded = c1d.reth_dmalen[31:11];
                        qp_psn_rem_decoded  = (c1d.reth_dmalen[10:0] != 0) ? 1 : 0;
                    end
                    3'b100: begin
                        qp_psn_coef_decoded = c1d.reth_dmalen[31:12];
                        qp_psn_rem_decoded  = (c1d.reth_dmalen[11:0] != 0) ? 1 : 0;
                    end
                    default: begin
                        qp_psn_coef_decoded = c1d.reth_dmalen[31:8];
                        qp_psn_rem_decoded  = (c1d.reth_dmalen[7:0] != 0) ? 1 : 0;
                    end
                endcase
        end else begin
            assign qp_psn_coef_decoded = 0;
            assign qp_psn_rem_decoded  = 0;
        end
    endgenerate

    // for pipeline stage
    // qp_conf[31:16]：RQ 缓冲区大小（256B 的倍数）。这是请求中每个缓冲区元素的大小，而不是整个请求的大小。
    assign p_rq_buf_ca = (c2d.qp_conf_data.rq_wrptr_db == (c2d.qp_conf_data.q_depth - 'd1)) ? c2d.qp_conf_data.rq_buf_ba : (c2d.qp_conf_data.rq_buf_ca + c2d.qp_conf_data.qp_conf[31:16]);  //qp_rq_buf_sz

    assign p_last_rq_req[31:24] = c2d.c1d.bth_opcode;
    assign p_incr_for_read_req = (c2d.c1d.bth_opcode[4:0] == RDMA_RD_REQ) ? (c2d.qp_psn_coef_decoded + c2d.qp_psn_rem_decoded - 1 + (c2d.c1d.reth_dmalen == 0)) : 0;
    assign p_last_rq_req[23:0] = p_exp_req_psn + p_incr_for_read_req;

    assign p_rq_wrptr_db = (c2d.qp_conf_data.rq_wrptr_db == c2d.qp_conf_data.q_depth) ? 16'd1 : (c2d.qp_conf_data.rq_wrptr_db + 'd1);

    assign p_stat_qp_msn = c2d.qp_conf_data.stat_qp_msn + 'd1;

    assign p_rq_buf_offset = (c2d.c1d.bth_opcode[4:0] == SEND_FIRST) ? c2d.qp_pmtu_decoded[12:8] : (c2d.qp_conf_data.rq_buf_offset + c2d.qp_pmtu_decoded[12:8]);

    assign p_wr_rq_buf_offset = (c2d.c1d.bth_opcode[4:0] == RDMA_WR_FIRST) ? (c2d.physical_addr_from_reth[C_VIRT_ADDR_WIDTH-1:8] + c2d.qp_pmtu_decoded[12:8]) : (c2d.qp_conf_data.wr_rq_buf_offset + c2d.qp_pmtu_decoded[12:8]);


    //assign o_rq_wrptr_db =  (c3d.c2d.qp_conf_data.rq_wrptr_db == c3d.c2d.qp_conf_data.q_depth) ? 16'd1 : (c3d.c2d.qp_conf_data.rq_wrptr_db + 'd1);

    //assign o_rq_buf_ca = (c3d.c2d.qp_conf_data.rq_wrptr_db == (c3d.c2d.qp_conf_data.q_depth-'d1)) ? c3d.c2d.qp_conf_data.rq_buf_ba : (c3d.c2d.qp_conf_data.rq_buf_ca + c2d.qp_conf_data.qp_conf[31:16]);//qp_rq_buf_sz
    assign qp_wrb_dat_w.rq_buf_ca = (c3d.c2d.qp_conf_data.rq_wrptr_db == (c3d.c2d.qp_conf_data.q_depth - 'd1)) ? c3d.c2d.qp_conf_data.rq_buf_ba : (c3d.c2d.qp_conf_data.rq_buf_ca + c3d.c2d.qp_conf_data.qp_conf[31:16]);  //qp_rq_buf_sz
    //assign o_last_rq_req[31:24] = c3d.c2d.c1d.bth_opcode;
    assign qp_wrb_dat_w.last_rq_req[31:24] = c3d.c2d.c1d.bth_opcode;
    assign psn_incr_for_read_req = (c3d.c2d.c1d.bth_opcode[4:0] == RDMA_RD_REQ) ? (c3d.c2d.qp_psn_coef_decoded + c3d.c2d.qp_psn_rem_decoded - 1 + (c3d.c2d.c1d.reth_dmalen == 0)) : 0;
    //assign o_last_rq_req[23:0] = exp_req_psn + psn_incr_for_read_req;  
    assign qp_wrb_dat_w.last_rq_req[23:0] = exp_req_psn + psn_incr_for_read_req;
    //assign o_stat_qp_msn = c3d.c2d.qp_conf_data.stat_qp_msn + 'd1;
    assign qp_wrb_dat_w.stat_qp_msn = c3d.c2d.qp_conf_data.stat_qp_msn + 'd1;  // 在一条消息被确认完成时加一
    //assign o_rq_wrptr_db =  (c3d.c2d.qp_conf_data.rq_wrptr_db == c3d.c2d.qp_conf_data.q_depth) ? 16'd1 : (c3d.c2d.qp_conf_data.rq_wrptr_db + 'd1);

    /* 一个直观例子（q_depth=64）：
        上一时刻：PI=64，CI=64（空）
        收到1个包后：PI回卷到1，CI=64，这时确实有1个新包
        软件处理完并回写CI=1后：PI=1，CI=1，又回到空队列
    */
    assign qp_wrb_dat_w.rq_wrptr_db        = (c3d.c2d.qp_conf_data.rq_wrptr_db == c3d.c2d.qp_conf_data.q_depth) ? 16'd1 : (c3d.c2d.qp_conf_data.rq_wrptr_db + 'd1);  // +1，初始是从0开始的，接收完了才+1写回。回卷到1是为了满足环形条件。从 64 -> 1 算一次推进，判断有没有新包，就是看有没有推进

    // rq_buf_ca 只表示这个 RQE 的起始位置，它是按 256B 粒度推进的槽位地址（每条新消息通常占一个 RQE 槽位）。
    // 一条 SEND/WRITE 可能被拆成多个包， FIRST/MIDDLE/LAST 到来时，MIDDLE/LAST 不能每次都写到槽位起始地址，否则会覆盖前面已写的数据。
    // rq_buf_offset 是“当前这条接收消息在 RQ 槽内已经写了多少个 256B 单位”的偏移计数，收到 SEND_FIRST 时，offset 置为一个 PMTU 步长。
    assign qp_wrb_dat_w.rq_buf_offset = (c3d.c2d.c1d.bth_opcode[4:0] == SEND_FIRST) ? c3d.c2d.qp_pmtu_decoded[12:8] : (c3d.c2d.qp_conf_data.rq_buf_offset + c3d.c2d.qp_pmtu_decoded[12:8]);
    // rq_buf_ca_plus_offset 定位“槽内第几个片段的数据落点”。这样才能正确重组分段报文。
    assign rq_buf_ca_plus_offset = c3d.c2d.qp_conf_data.rq_buf_ca + c3d.c2d.qp_conf_data.rq_buf_offset;

    assign qp_wrb_dat_w.nak_seq_err_sent = o_nak_seq_err_sent;
    //assign o_qp_fatal_idx = set_qp_fatal ? fatal_qp_idx_r : fatal_qp_idx ;
    //assign o_qp_fatal_idx = async_fatal_asserted ? async_fatal_qp : fatal_qp_idx ;
    assign o_qp_fatal_idx = fatal_qp_idx;
    assign ob_rsp_qpid = c3d.c2d.c1d.qp_number;  //bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0];
    assign ob_rsp_expl_ack = c3d.c2d.c1d.bth_expl_ack;
    assign cnp_pkt_qpid = c3d.c2d.c1d.qp_number;

    generate
        if (C_EN_INITIATOR_LITE) begin  // current
            //assign wr_rq_buf_ca_plus_offset = physical_addr_from_reth_1 + {i_wr_rq_buf_offset,8'd0};
            //assign wr_rq_buf_ca_plus_offset = {i_wr_rq_buf_offset,8'd0};
            assign wr_rq_buf_ca_plus_offset      = {c3d.c2d.qp_conf_data.wr_rq_buf_offset, 8'd0};  // 256B对齐，偏移后的绝对地址
            // PMTU 是协议层每个分片允许的负载大小
            // 这个 wr_rq_buf_offset 在 WR_FIRST 时就被初始化为 physical_addr_from_reth 的高位 + 一个 PMTU 步长
            // 后续每个分片继续在这个“已含 base 的值”上累加 PMTU 步长
            assign qp_wrb_dat_w.wr_rq_buf_offset = (c3d.c2d.c1d.bth_opcode[4:0] == RDMA_WR_FIRST) ? (c3d.c2d.physical_addr_from_reth[C_VIRT_ADDR_WIDTH-1:8] + c3d.c2d.qp_pmtu_decoded[12:8]) : (c3d.c2d.qp_conf_data.wr_rq_buf_offset + c3d.c2d.qp_pmtu_decoded[12:8]);
        end else begin
            assign wr_rq_buf_ca_plus_offset      = 0;
            assign qp_wrb_dat_w.wr_rq_buf_offset = 0;
        end
    endgenerate


    always @(posedge clk) in_err_sts_code <= nxt_in_err_sts_code;

    // =========================================== 检测数据包类型 ===========================================
    generate
        if (C_EN_INITIATOR_LITE) begin  // current: C_EN_INITIATOR_LITE == 1
            if (C_EN_IETH_IMMDT == 0) begin
                always @(*) begin
                    if ((bth_opcode[7:5] == UD) && (bth_qp_dest_addr == 24'd0) && (bth_opcode[4:0] == SEND_ONLY)) pkt_type = PKT_TYPE_MAD;
                    else if ((bth_opcode[7:5] == RC) && (bth_qp_dest_addr != 24'd0))
                        case (bth_opcode[4:0])
                            SEND_ONLY: pkt_type = PKT_TYPE_SEND_ONLY;
                            SEND_FIRST: pkt_type = PKT_TYPE_SEND_FIRST;
                            SEND_MIDDLE: pkt_type = PKT_TYPE_SEND_MIDDLE;
                            SEND_LAST: pkt_type = PKT_TYPE_SEND_LAST;
                            RDMA_WR_FIRST: pkt_type = PKT_TYPE_RDMA_WR_FIRST;
                            RDMA_WR_LAST: pkt_type = PKT_TYPE_RDMA_WR_LAST;
                            RDMA_WR_MIDDLE: pkt_type = PKT_TYPE_RDMA_WR_MIDDLE;
                            RDMA_WR_ONLY: pkt_type = PKT_TYPE_RDMA_WR_ONLY;
                            RDMA_RD_REQ: pkt_type = PKT_TYPE_RDMA_RD_REQ;
                            default: pkt_type = PKT_TYPE_UNSUPPORTED;
                        endcase
                    else if (bth_opcode == 8'h81) pkt_type = PKT_TYPE_CNP;
                    else pkt_type = PKT_TYPE_RSVD;
                end
            end else begin  // current: C_EN_IETH_IMMDT == 1
                always @(*) begin
                    // MAD层是基于IB传输层的UD服务类型实现的，它在UD服务类型的报文的基础上，利用其Payload部分又实现了一层协议，发送给 QP 1
                    // 基于RoCE v2的CM报文: Ethernet IP UDP IB_Transport MAD CM CRC
                    // 意味着它被当作 UD 发往 QP1 的 SEND_ONLY，不是普通 RC SEND/WRITE，也不是响应包
                    if ((bth_opcode[7:5] == UD) && (bth_qp_dest_addr == 24'd0) && (bth_opcode[4:0] == SEND_ONLY)) pkt_type = PKT_TYPE_MAD;
                    else if ((bth_opcode[7:5] == RC) && (bth_qp_dest_addr != 24'd0))
                        case (bth_opcode[4:0])
                            SEND_ONLY, SEND_ONLY_INV, SEND_ONLY_IMM: pkt_type = PKT_TYPE_SEND_ONLY;
                            SEND_FIRST: pkt_type = PKT_TYPE_SEND_FIRST;
                            SEND_MIDDLE: pkt_type = PKT_TYPE_SEND_MIDDLE;
                            SEND_LAST, SEND_LAST_INV, SEND_LAST_IMM: pkt_type = PKT_TYPE_SEND_LAST;
                            RDMA_WR_FIRST: pkt_type = PKT_TYPE_RDMA_WR_FIRST;
                            RDMA_WR_LAST, RDMA_WR_LAST_IMM: pkt_type = PKT_TYPE_RDMA_WR_LAST;
                            RDMA_WR_MIDDLE: pkt_type = PKT_TYPE_RDMA_WR_MIDDLE;
                            RDMA_WR_ONLY, RDMA_WR_ONLY_IMM: pkt_type = PKT_TYPE_RDMA_WR_ONLY;
                            RDMA_RD_REQ: pkt_type = PKT_TYPE_RDMA_RD_REQ;
                            default: pkt_type = PKT_TYPE_UNSUPPORTED;
                        endcase
                    else if (bth_opcode == 8'h81) pkt_type = PKT_TYPE_CNP;
                    else pkt_type = PKT_TYPE_RSVD;
                end
            end
        end else begin  // C_EN_INITIATOR_LITE == 0
            if (C_EN_IETH_IMMDT == 0) begin
                always @(*) begin
                    if ((bth_opcode[7:5] == UD) && (bth_qp_dest_addr == 24'd0) && (bth_opcode[4:0] == SEND_ONLY)) pkt_type = PKT_TYPE_MAD;
                    else if ((bth_opcode[7:5] == RC) && (bth_qp_dest_addr != 24'd0))
                        case (bth_opcode[4:0])
                            SEND_ONLY: pkt_type = PKT_TYPE_SEND_ONLY;
                            SEND_FIRST: pkt_type = PKT_TYPE_SEND_FIRST;
                            SEND_MIDDLE: pkt_type = PKT_TYPE_SEND_MIDDLE;
                            SEND_LAST: pkt_type = PKT_TYPE_SEND_LAST;
                            default: pkt_type = PKT_TYPE_UNSUPPORTED;
                        endcase
                    else if (bth_opcode == 8'h81) pkt_type = PKT_TYPE_CNP;
                    else pkt_type = PKT_TYPE_RSVD;
                end
            end else begin
                always @(*) begin
                    if ((bth_opcode[7:5] == UD) && (bth_qp_dest_addr == 24'd0) && (bth_opcode[4:0] == SEND_ONLY)) pkt_type = PKT_TYPE_MAD;
                    else if ((bth_opcode[7:5] == RC) && (bth_qp_dest_addr != 24'd0))
                        case (bth_opcode[4:0])
                            SEND_ONLY, SEND_ONLY_INV, SEND_ONLY_IMM: pkt_type = PKT_TYPE_SEND_ONLY;
                            SEND_FIRST: pkt_type = PKT_TYPE_SEND_FIRST;
                            SEND_MIDDLE: pkt_type = PKT_TYPE_SEND_MIDDLE;
                            SEND_LAST, SEND_LAST_INV, SEND_LAST_IMM: pkt_type = PKT_TYPE_SEND_LAST;
                            default: pkt_type = PKT_TYPE_UNSUPPORTED;
                        endcase
                    else if (bth_opcode == 8'h81) pkt_type = PKT_TYPE_CNP;
                    else pkt_type = PKT_TYPE_RSVD;
                end
            end
        end
    endgenerate
    // =========================================== 检测数据包类型 end ===========================================

    xpm_memory_spram #(  // Common module parameters
        .MEMORY_SIZE(C_NUM_QP * (C_M_AXI_ADDR_WIDTH - 8)),  // positive integer
        .MEMORY_PRIMITIVE("auto"),  // string; "auto", "distributed", "block" or "ultra";
        .ECC_MODE("no_ecc"),  // do not change
        .MEMORY_INIT_FILE("none"),  // string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),  // string;  
        .WAKEUP_TIME("disable_sleep"),  // string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL(0),  // Port A module parameters
        .WRITE_DATA_WIDTH_A(C_M_AXI_ADDR_WIDTH - 8),  // positive integer
        .READ_DATA_WIDTH_A(C_M_AXI_ADDR_WIDTH - 8),  // positive integer
        .BYTE_WRITE_WIDTH_A(C_M_AXI_ADDR_WIDTH - 8),  // integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A(C_QP_INDX_WIDTH),  // positive integer
        .READ_RESET_VALUE_A("0"),  // string
        .READ_LATENCY_A(1),  // non-negative integer
        .WRITE_MODE_A("write_first")  // string; "write_first", "read_first", "no_change" 

    ) xpm_memory_rq_buf_offset (
        // Common module ports
        .sleep         (1'b0),              //do not change  
        // Port A module ports
        .clka          (clk),
        .rsta          (reset),
        .ena           (rq_buf_offset_en),
        .regcea        (1'b1),
        .wea           (rq_buf_offset_we),
        //.addra          (o_qp_conf_idx_for_offset_mem),
        .addra         (o_qp_conf_idx),
        .dina          (o_rq_buf_offset),
        .injectsbiterra(1'b0),              //do not change
        .injectdbiterra(1'b0),              //do not change
        .douta         (i_rq_buf_offset),
        .sbiterra      (),                  //do not change
        .dbiterra      ()                   //do not change  
    );

    generate
        if (C_EN_INITIATOR_LITE) begin
            xpm_memory_spram #(  // Common module parameters
                .MEMORY_SIZE       (C_NUM_QP * (C_VIRT_ADDR_WIDTH - 8)),  // positive integer
                .MEMORY_PRIMITIVE  ("auto"),                              // string; "auto", "distributed", "block" or "ultra";
                .ECC_MODE          ("no_ecc"),                            // do not change
                .MEMORY_INIT_FILE  ("none"),                              // string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM (""),                                  // string;  
                .WAKEUP_TIME       ("disable_sleep"),                     // string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL   (0),                                   // Port A module parameters
                .WRITE_DATA_WIDTH_A(C_VIRT_ADDR_WIDTH - 8),               // positive integer
                .READ_DATA_WIDTH_A (C_VIRT_ADDR_WIDTH - 8),               // positive integer
                .BYTE_WRITE_WIDTH_A(C_VIRT_ADDR_WIDTH - 8),               // integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_QP_INDX_WIDTH),                     // positive integer
                .READ_RESET_VALUE_A("0"),                                 // string
                .READ_LATENCY_A    (1),                                   // non-negative integer
                .WRITE_MODE_A      ("write_first")                        // string; "write_first", "read_first", "no_change" 

            ) xpm_memory_wr_rq_buf_offset (
                // Common module ports
                .sleep         (1'b0),                 //do not change  
                // Port A module ports
                .clka          (clk),
                .rsta          (reset),
                .ena           (wr_rq_buf_offset_en),
                .regcea        (1'b1),
                .wea           (wr_rq_buf_offset_we),
                //.addra          (o_qp_conf_idx_for_offset_mem),
                .addra         (o_qp_conf_idx),
                .dina          (o_wr_rq_buf_offset),
                .injectsbiterra(1'b0),                 //do not change
                .injectdbiterra(1'b0),                 //do not change
                .douta         (i_wr_rq_buf_offset),
                .sbiterra      (),                     //do not change
                .dbiterra      ()                      //do not change  
            );
        end else begin
            assign i_wr_rq_buf_offset = 0;
        end
    endgenerate  //C_EN_INITIATOR_LITE

    //  always @(posedge clk)
    //    if (reset)
    //      nak_seq_err_sent_reg <= {(C_NUM_QP-1) {1'b0}};
    //    else if (set_qp_fatal)
    //       nak_seq_err_sent_reg[fatal_qp_idx_r] <= 1'b1; 
    //    else if (nak_seq_err_sent_we)
    //      //nak_seq_err_sent_reg[bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]] <= o_nak_seq_err_sent;
    //      //nak_seq_err_sent_reg[c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0]] <= o_nak_seq_err_sent;
    //      nak_seq_err_sent_reg[wrb_qp_number] <= nak_seq_err_sent;
    //    else if (rsp_qp_fatal_flag && clr_qp_fatal_flag)
    //      nak_seq_err_sent_reg[rsp_qp_fatal_idx_r] <= 1'b1;
    //    else if (|qp_enable_edge)
    //      nak_seq_err_sent_reg <= nak_seq_err_sent_reg & ~qp_enable_edge;

    // NAK序列错误标志的写入控制逻辑，管理每个QP的NAK序列错误发送状态
    always @(posedge clk) begin
        if (reset) begin
            nak_seq_err_sent_1_wr_addr <= {(C_QP_INDX_WIDTH - 1) {1'b0}};
            nak_seq_err_sent_1_wr_en   <= 1'b0;
            nak_seq_err_sent_1_wr_val  <= 1'b0;
            i_qp_enable_wr_en_r        <= 1'b0;
            i_qp_enable_wr_idx_r       <= 'd0;
            i_qp_enable_wr_data_r      <= 1'b0;
        end else begin
            i_qp_enable_wr_en_r   <= i_qp_enable_wr_en;
            i_qp_enable_wr_idx_r  <= i_qp_enable_wr_idx;
            i_qp_enable_wr_data_r <= i_qp_enable_wr_data;
            if (set_qp_fatal) begin  // QP致命错误，当QP进入致命错误状态时，立即将该QP的NAK发送标志设为1，防止后续再发送NAK
                nak_seq_err_sent_1_wr_addr <= fatal_qp_idx_r;
                nak_seq_err_sent_1_wr_en   <= 1'b1;
                nak_seq_err_sent_1_wr_val  <= 1'b1;  // 设置为1，表示该QP已发送NAK(序列错误)
            end else if (nak_seq_err_sent_we) begin  // 正常NAK状态更新
                nak_seq_err_sent_1_wr_addr <= wrb_qp_number;
                nak_seq_err_sent_1_wr_en   <= 1'b1;
                nak_seq_err_sent_1_wr_val  <= nak_seq_err_sent;
            end else if (rsp_qp_fatal_flag && clr_qp_fatal_flag) begin  // 响应处理器的QP致命错误
                nak_seq_err_sent_1_wr_addr <= rsp_qp_fatal_idx_r;
                nak_seq_err_sent_1_wr_en   <= 1'b1;
                nak_seq_err_sent_1_wr_val  <= 1'b1;
            end else if (i_qp_enable_wr_en && i_qp_enable_wr_data && ~qp_enable_lutram_out) begin
                nak_seq_err_sent_1_wr_addr <= i_qp_enable_wr_idx;
                nak_seq_err_sent_1_wr_en   <= 1'b1;
            end else begin
                nak_seq_err_sent_1_wr_en  <= 1'b0;
                nak_seq_err_sent_1_wr_val <= 1'b0;
            end
        end
    end

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// nak_seq_err_sent_reg LUTRAM START //////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_nak_seq_err_sent_reg_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (init_start || nak_seq_err_sent_1_wr_en),
        .wr_addr (init_start ? init_count : nak_seq_err_sent_1_wr_addr),
        .wr_data (init_start ? 1'b0 : nak_seq_err_sent_1_wr_val),
        .rd_en   (1'b1),
        .rd_addr (c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0]),
        .rd_data (nak_seq_err_sent_reg_combo)
    );

    sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_enable_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (init_start || i_qp_enable_wr_en_r),
        .wr_addr (init_start ? init_count : i_qp_enable_wr_idx_r),
        .wr_data (init_start ? 1'b0 : i_qp_enable_wr_data_r),
        .rd_en   (1'b1),
        .rd_addr (i_qp_enable_wr_idx),
        .rd_data (qp_enable_lutram_out)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// nak_seq_err_sent_reg LUTRAM END ////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    // 全局（全部QP）RNR窗口定时器
    always @(posedge clk)
        if (reset) rnr_window_tval <= 16'd0;
        else if (pkt_discarded) rnr_window_tval <= RNR_WINDOW_TVAL;
        else if (rnr_window_tval != 'd0) rnr_window_tval <= rnr_window_tval - 'd1;

    // 全局RNR窗口定时器计数完成，全局RNR窗口关闭
    assign rnr_window_closed = (rnr_window_tval == 'd1) ? 1'b1 : 1'b0;

    /*   always @(posedge clk)
    for (i=0; i<C_NUM_QP; i=i+1)
    begin
      if (reset)
        rnr_window_flag_reg[i] <= 1'b0;
      else if (pkt_discarded)
        rnr_window_flag_reg[i] <= 1'b1;
      //else if (rnr_window_closed || ((i[C_QP_INDX_WIDTH-1:0] == bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]) && ob_rsp_vld) || set_qp_fatal)
      else if (((i[C_QP_INDX_WIDTH-1:0] == c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0]) && ob_rsp_vld && c3d.c2d.c1d.rnr_window_active) ||
                rnr_window_closed ||
                set_qp_fatal)
        rnr_window_flag_reg[i] <= 1'b0;
      else if (rsp_qp_fatal_flag && clr_qp_fatal_flag && (i[C_QP_INDX_WIDTH-1:0] == rsp_qp_fatal_idx_r))
        rnr_window_flag_reg[i] <= 1'b0;
    end */

    always_ff @(posedge clk) begin
        pkt_discarded_r         <= pkt_discarded;
        c3d_qp_number_r         <= c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
        ob_rsp_vld_r            <= ob_rsp_vld;
        c3d_rnr_window_active_r <= c3d.c2d.c1d.rnr_window_active;
        rnr_window_closed_r     <= rnr_window_closed;
        set_qp_fatal_r          <= set_qp_fatal;
        rsp_qp_fatal_flag_r     <= rsp_qp_fatal_flag;
        clr_qp_fatal_flag_r     <= clr_qp_fatal_flag;
        rsp_qp_fatal_idx_2r     <= rsp_qp_fatal_idx_r;
    end

    always @(posedge clk)
        for (i = 0; i < C_NUM_QP; i = i + 1) begin
            if (reset) rnr_window_flag_reg[i] <= 1'b0;

            // 包丢弃时，触发全局RNR窗口
            else if (pkt_discarded_r) rnr_window_flag_reg[i] <= 1'b1;

            // ((i[C_QP_INDX_WIDTH-1:0] == c3d_qp_number_r) && ob_rsp_vld_r && c3d_rnr_window_active_r)：对这个 QP，当前流水线已经产生了一个有效的出站响应，并且该 QP 处于 RNR window active 上下文，于是清掉该 QP 的 rnr_window_flag_reg[i]
            // rnr_window_closed_r：RNR 窗口计时到期了，所有 QP 的窗口标志都收敛为 0
            // set_qp_fatal_r：发生 QP fatal 事件时，不再维持这批窗口标志，统一清掉
            else if (((i[C_QP_INDX_WIDTH-1:0] == c3d_qp_number_r) && ob_rsp_vld_r && c3d_rnr_window_active_r) || rnr_window_closed_r || set_qp_fatal_r) rnr_window_flag_reg[i] <= 1'b0;

            // 响应路径上有 fatal 标志并执行了 fatal 清除动作时，把对应 QP 的窗口标志清 0
            else if (rsp_qp_fatal_flag_r && clr_qp_fatal_flag_r && (i[C_QP_INDX_WIDTH-1:0] == rsp_qp_fatal_idx_2r)) rnr_window_flag_reg[i] <= 1'b0;
        end

    // 算剩余 credit，这里RQ是环形的，所以做了环形处理。
    // rq_wrptr_db 可以理解为生产者指针（PI），rq_ci_db 是消费者指针（CI）
    assign credits_remain  = (rq_wrptr_db < rq_ci_db) ? ({1'b1, rq_wrptr_db} - rq_ci_db) : (rq_wrptr_db - rq_ci_db);  // 没用到

    // credits_to_send=min(credits_remain,30)
    assign credits_to_send = (credits_remain > 17'd30) ? 5'b1_1110 : credits_remain[4:0];  // 没用到

    // Commented below statement as verfication is failing
    //assign flow_credits = (i_flow_credits == 2'b00) ? 5'b1_1111 : credits_to_send;
    // 基于信用的流量控制
    assign flow_credits    = (i_flow_credits == 2'b00) ? 5'b1_1111 : {3'b000, i_flow_credits};

    always @(posedge clk)
        if (reset) tnfr_pend <= 1'b0;
        else tnfr_pend <= req_tnfr_vld & ~rdy_to_req_tnfr;  // 传输挂起标志，表示有数据传输请求但还未被接受处理的状态

    always @(posedge clk)
        if (reset) ext_fatal_case_flag <= 1'b0;
        else if (set_ext_fatal_case_flag || clr_ext_fatal_case_flag) ext_fatal_case_flag <= set_ext_fatal_case_flag;



    //===============================================================================================
    // Pipeline stage 1：基本头部验证
    //===============================================================================================
    generate
        if (C_EN_IETH_IMMDT == 0) begin
            //Pipeline stage 1 hv_data_vld
            always @(posedge clk) begin
                if (reset) begin
                    c1d <= '{default: 0};
                end else begin
                    //if (hv_data_vld & !hv_fsm_busy)
                    if (!hv_fsm_busy) begin
                        if (hv_data_vld) begin
                            c1d.hv_data_vld <= hv_data_vld;
                            c1d.qp_number <= bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];
                            c1d.pd_number <= reth_r_key_w[31:8];
                            c1d.bth_psn <= bth_psn;
                            c1d.pkt_type <= pkt_type;
                            c1d.bypass_en <= i_bypass_en;
                            c1d.ip_ver <= ip_ver;
                            c1d.bth_opcode <= bth_opcode;
                            c1d.bth_expl_ack <= bth_expl_ack;
                            c1d.mac_src_addr <= {hdr_data[(6*8)+:8], hdr_data[(7*8)+:8], hdr_data[(8*8)+:8], hdr_data[(9*8)+:8], hdr_data[(10*8)+:8], hdr_data[(11*8)+:8]};
                            c1d.ipv4_src_addr <= {hdr_data[(26*8)+:8], hdr_data[(27*8)+:8], hdr_data[(28*8)+:8], hdr_data[(29*8)+:8]};
                            c1d.ipv6_src_addr <= {
                                hdr_data[(22*8)+:8],
                                hdr_data[(23*8)+:8],
                                hdr_data[(24*8)+:8],
                                hdr_data[(25*8)+:8],
                                hdr_data[(26*8)+:8],
                                hdr_data[(27*8)+:8],
                                hdr_data[(28*8)+:8],
                                hdr_data[(29*8)+:8],
                                hdr_data[(30*8)+:8],
                                hdr_data[(31*8)+:8],
                                hdr_data[(32*8)+:8],
                                hdr_data[(33*8)+:8],
                                hdr_data[(34*8)+:8],
                                hdr_data[(35*8)+:8],
                                hdr_data[(36*8)+:8],
                                hdr_data[(37*8)+:8]
                            };
                            c1d.pkt_id <= pkt_id;
                            c1d.pkt_src_addr_offset <= pkt_src_addr_offset;
                            c1d.actual_pkt_len <= actual_pkt_len;  // 包长度
                            c1d.rnr_window_active <= rnr_window_active;
                            //c1d.nak_seq_err_sent <= nak_seq_err_sent_reg[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]];
                            c1d.nak_seq_err_sent <= nak_seq_err_sent_reg_combo;
                            //c1d.rnr_window_flag <= rnr_window_flag_reg[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]];
                            c1d.rnr_window_flag <= 'd0;
                            c1d.send_only_or_last <= ((bth_opcode[4:0] == SEND_ONLY) || (bth_opcode == SEND_LAST)) ? 1'b1 : 1'b0;
                            c1d.send_middle_or_last <= ((bth_opcode[4:0] == SEND_MIDDLE) || (bth_opcode == SEND_LAST)) ? 1'b1 : 1'b0;
                            c1d.mad_pkt_len_err <= (actual_pkt_len > ((ip_ver == IPv4) ? 13'd322 : 13'd342)) ? 1'b1 : 1'b0;  // 
                            //c1d.ecn_vect <= nxt_ecn_vect;
                            c1d.ecn_vect_idx <= nxt_ecn_vect_idx;
                            c1d.ecn_update_flg <= nxt_ecn_update_flg;
                            c1d.bth_pyld_len <= bth_pyld_len;
                            c1d.write_only_or_last <= ((bth_opcode[4:0] == RDMA_WR_ONLY) || (bth_opcode == RDMA_WR_LAST)) ? 1'b1 : 1'b0;
                            c1d.write_middle_or_last <= ((bth_opcode[4:0] == RDMA_WR_MIDDLE) || (bth_opcode == RDMA_WR_LAST)) ? 1'b1 : 1'b0;
                            c1d.has_reth <= has_reth;
                            c1d.has_ieth <= has_ieth;
                            c1d.has_immdt <= has_immdt;
                            c1d.ieth_immdt_data <= ieth_immdt_data;
                            c1d.reth_dmalen <= (ip_ver == IPv4) ? ipv4_reth_dmalen : ipv6_reth_dmalen;
                            c1d.reth_r_key <= reth_r_key_w;
                            c1d.reth_va <= (ip_ver == IPv4) ? ipv4_reth_va : ipv6_reth_va;
                        end else begin
                            c1d <= '{default: 0};
                        end  //hv_data_vld
                    end  //!hv_fsm_busy
                end  //reset
            end  // always @ (posedge clk)
        end // if (C_EN_IETH_IMMDT == 0)
        else begin  // current: C_EN_IETH_IMMDT == 1
            // ========================== Pipeline stage 1：协议头部解析和基础数据保存 ==========================
            always @(posedge clk) begin
                if (reset) begin
                    c1d <= '{default: 0};
                end else begin
                    //if (hv_data_vld & !hv_fsm_busy)
                    if (!hv_fsm_busy) begin
                        if (hv_data_vld) begin  // 头部数据有效，即两拍拼接并解析好了-->保存基础数据
                            c1d.hv_data_vld <= hv_data_vld;
                            c1d.qp_number <= bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0];  // Dest QP
                            c1d.pd_number <= reth_r_key_w[31:8];  // pd_number == reth_r_key_w[31:8]
                            c1d.bth_psn <= bth_psn;
                            c1d.pkt_type <= pkt_type;
                            c1d.bypass_en <= i_bypass_en;
                            c1d.ip_ver <= ip_ver;
                            c1d.bth_opcode <= bth_opcode;
                            c1d.bth_expl_ack <= bth_expl_ack;
                            c1d.mac_src_addr <= {hdr_data[(6*8)+:8], hdr_data[(7*8)+:8], hdr_data[(8*8)+:8], hdr_data[(9*8)+:8], hdr_data[(10*8)+:8], hdr_data[(11*8)+:8]};
                            c1d.ipv4_src_addr <= {hdr_data[(26*8)+:8], hdr_data[(27*8)+:8], hdr_data[(28*8)+:8], hdr_data[(29*8)+:8]};
                            c1d.ipv6_src_addr <= {
                                hdr_data[(22*8)+:8],
                                hdr_data[(23*8)+:8],
                                hdr_data[(24*8)+:8],
                                hdr_data[(25*8)+:8],
                                hdr_data[(26*8)+:8],
                                hdr_data[(27*8)+:8],
                                hdr_data[(28*8)+:8],
                                hdr_data[(29*8)+:8],
                                hdr_data[(30*8)+:8],
                                hdr_data[(31*8)+:8],
                                hdr_data[(32*8)+:8],
                                hdr_data[(33*8)+:8],
                                hdr_data[(34*8)+:8],
                                hdr_data[(35*8)+:8],
                                hdr_data[(36*8)+:8],
                                hdr_data[(37*8)+:8]
                            };
                            c1d.pkt_id <= pkt_id;
                            c1d.pkt_src_addr_offset <= pkt_src_addr_offset;
                            c1d.actual_pkt_len <= actual_pkt_len;  // 总包长度（不含FCS）
                            c1d.rnr_window_active <= rnr_window_active;
                            //c1d.nak_seq_err_sent <= nak_seq_err_sent_reg[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]];
                            c1d.nak_seq_err_sent <= nak_seq_err_sent_reg_combo;
                            //c1d.rnr_window_flag <= rnr_window_flag_reg[bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0]];
                            c1d.rnr_window_flag <= 'd0;
                            c1d.send_only_or_last <= ((bth_opcode[4:0] == SEND_ONLY) || (bth_opcode == SEND_LAST) || (bth_opcode[4:0] == SEND_ONLY_INV) || (bth_opcode[4:0] == SEND_LAST_INV) || (bth_opcode[4:0] == SEND_ONLY_IMM) || (bth_opcode[4:0] == SEND_LAST_IMM)) ? 1'b1 : 1'b0;
                            c1d.send_middle_or_last <= ((bth_opcode[4:0] == SEND_MIDDLE) || (bth_opcode == SEND_LAST) || (bth_opcode[4:0] == SEND_LAST_INV) || (bth_opcode[4:0] == SEND_LAST_IMM)) ? 1'b1 : 1'b0;
                            // 14 Ethernet + 20 IPv4 + 8 UDP + 12 BTH + 8 DETH（UD/MAD有） + 256 MAD 数据报最大长度 + 4 ICRC
                            c1d.mad_pkt_len_err <= (actual_pkt_len > ((ip_ver == IPv4) ? 13'd322 : 13'd342)) ? 1'b1 : 1'b0;  // MAD 报文长度上限检查
                            //c1d.ecn_vect <= nxt_ecn_vect;
                            c1d.ecn_vect_idx <= nxt_ecn_vect_idx;
                            c1d.ecn_update_flg <= nxt_ecn_update_flg;
                            c1d.bth_pyld_len <= bth_pyld_len;
                            c1d.write_only_or_last <= ((bth_opcode[4:0] == RDMA_WR_ONLY) || (bth_opcode[4:0] == RDMA_WR_LAST) || (bth_opcode[4:0] == RDMA_WR_ONLY_IMM) || (bth_opcode[4:0] == RDMA_WR_LAST_IMM)) ? 1'b1 : 1'b0;
                            c1d.write_middle_or_last <= ((bth_opcode[4:0] == RDMA_WR_MIDDLE) || (bth_opcode == RDMA_WR_LAST) || (bth_opcode[4:0] == RDMA_WR_LAST_IMM)) ? 1'b1 : 1'b0;
                            c1d.has_reth <= has_reth;
                            c1d.has_ieth <= has_ieth;
                            c1d.has_immdt <= has_immdt;
                            c1d.ieth_immdt_data <= ieth_immdt_data;
                            c1d.reth_dmalen <= (ip_ver == IPv4) ? ipv4_reth_dmalen : ipv6_reth_dmalen;
                            c1d.reth_r_key <= reth_r_key_w;
                            c1d.reth_va <= (ip_ver == IPv4) ? ipv4_reth_va : ipv6_reth_va;
                        end else begin
                            c1d <= '{default: 0};
                        end  //hv_data_vld
                    end  //!hv_fsm_busy
                end  //reset
            end  // always @ (posedge clk)
        end  // if (C_EN_IETH_IMMDT == 1)
    endgenerate



    // ========================= Pipeline stage 2：QP和PD配置获取 ==========================
    // QP配置数据源选择 - 根据缓冲区状态选择最新数据
    // 时序与c1d同步
    always @(*) begin
        // 选择：从 写回缓冲区/Intiator寄存器组 中获取最新qp数据？
        // 如果缓冲区有该QP的更新数据，优先使用缓冲区数据，否则使用Intiator寄存器组数据

        // 接收队列(rq)当前地址
        qp_conf_data.rq_buf_ca          = (get_qp_info_from_buffer & qp_wrb_dat_r.rq_buf_ca_we) ? qp_wrb_dat_r.rq_buf_ca : i_rq_buf_ca;
        // 上一次请求的操作码和PSN
        qp_conf_data.last_rq_req        = (get_qp_info_from_buffer & qp_wrb_dat_r.last_rq_req_we) ? qp_wrb_dat_r.last_rq_req : i_last_rq_req;
        // QP消息序列号，MSN字段表明最后一个完成的请求序号(消息序列号)
        qp_conf_data.stat_qp_msn        = (get_qp_info_from_buffer & qp_wrb_dat_r.stat_qp_msn_we) ? qp_wrb_dat_r.stat_qp_msn : i_stat_qp_msn;
        // RQ 写门铃值
        qp_conf_data.rq_wrptr_db        = (get_qp_info_from_buffer & qp_wrb_dat_r.rq_wrptr_db_we) ? qp_wrb_dat_r.rq_wrptr_db : i_rq_wrptr_db;
        qp_conf_data.rq_buf_offset      = (get_qp_info_from_buffer & qp_wrb_dat_r.rq_buf_offset_we) ? qp_wrb_dat_r.rq_buf_offset : i_rq_buf_offset;
        qp_conf_data.wr_rq_buf_offset   = (get_qp_info_from_buffer & qp_wrb_dat_r.wr_rq_buf_offset_we) ? qp_wrb_dat_r.wr_rq_buf_offset : i_wr_rq_buf_offset;
        //qp_conf_data.nak_seq_err_sent   = (get_qp_info_from_buffer & qp_wrb_dat_r.nak_seq_err_sent_we) ? qp_wrb_dat_r.nak_seq_err_sent : nak_seq_err_sent_reg[c1d.qp_number];
        qp_conf_data.nak_seq_err_sent   = (get_qp_info_from_buffer & qp_wrb_dat_r.nak_seq_err_sent_we) ? qp_wrb_dat_r.nak_seq_err_sent : nak_seq_err_sent_reg_combo;

        // 其他只读字段直接使用寄存器值
        qp_conf_data.qp_conf            = i_qp_conf;
        qp_conf_data.rq_buf_ba          = i_rq_buf_ba;
        qp_conf_data.rq_wrptr_db_addr   = i_rq_wrptr_db_addr;
        qp_conf_data.q_depth            = i_q_depth;
        qp_conf_data.rq_ci_db           = i_rq_ci_db;
        qp_conf_data.remote_qp_mac_addr = i_remote_qp_mac_addr;
        qp_conf_data.remote_qp_ip_addr  = i_remote_qp_ip_addr;
        qp_conf_data.qp_pd_number       = i_qp_pd_num;
    end

    // ===================== 获取PD配置数据 ==========================
    // 时序与c1d同步
    generate
        if (C_EN_INITIATOR_LITE) begin  // current: C_EN_INITIATOR_LITE == 1
            always @(*) begin
                pd_conf_data.pd_number       = i_pd_pd_num;
                pd_conf_data.virt_addr_0     = i_pd_virt_addr_0;
                pd_conf_data.virt_addr_1     = i_pd_virt_addr_1;
                pd_conf_data.buf_base_addr_0 = i_pd_buf_base_addr_0;
                pd_conf_data.buf_base_addr_1 = i_pd_buf_base_addr_1;
                pd_conf_data.buf_r_key       = i_pd_buf_r_key;
                pd_conf_data.w_r_buf_len     = i_pd_w_r_buf_len;
                pd_conf_data.access_type     = i_pd_access_type;
            end
        end else begin  // C_EN_INITIATOR_LITE == 0
            assign pd_conf_data = '{default: 0};
        end
    endgenerate
    // ===================== 获取PD配置数据 end ==========================

    logic [23:0] exp_rq_psn;  // 期望接收的请求PSN
    assign exp_rq_psn = c2d.qp_conf_data.last_rq_req[23:0] + 'd1;

    // 执行各种验证
    always @(posedge clk) begin
        if (reset) begin
            c2d <= '{default: 0};
        end else begin
            if (qp_conf_vld) begin
                c2d.qp_conf_vld                  <= qp_conf_vld;
                c2d.qp_conf_data                 <= qp_conf_data;
                c2d.pd_conf_data                 <= pd_conf_data;
                // 源地址验证
                c2d.ip_src_addr_chk_fail         <= (c1d.ip_ver == IPv4) ? ipv4_src_addr_chk_fail : ipv6_src_addr_chk_fail;
                c2d.mac_src_addr_chk_fail        <= (qp_conf_data.remote_qp_mac_addr != c1d.mac_src_addr) ? 1'b1 : 1'b0;

                // 权限验证（rkey 与 PD/QP 配置匹配）
                c2d.reth_r_key_chk_fail          <= (!(c1d.reth_dmalen == 0) & ((c1d.reth_r_key[7:0] != pd_conf_data.buf_r_key[7:0]) || (qp_conf_data.qp_pd_number[23:0] != pd_conf_data.pd_number[23:0]))) ? 1'b1 : 1'b0;

                c2d.buffer_max_addr_from_qp_conf <= {pd_conf_data.virt_addr_1, pd_conf_data.virt_addr_0} + pd_conf_data.w_r_buf_len;
                c2d.va_offset_from_reth          <= c1d.reth_va - {pd_conf_data.virt_addr_1, pd_conf_data.virt_addr_0};

                // 地址转换
                c2d.physical_addr_from_reth      <= {pd_conf_data.buf_base_addr_1, pd_conf_data.buf_base_addr_0} + (c1d.reth_va - {pd_conf_data.virt_addr_1, pd_conf_data.virt_addr_0});

                // 操作码序列验证
                c2d.opcode_seq_chk_fail          <= opcode_seq_chk_fail;
                //c2d.pkt_vldn_err <= pkt_vldn_err_1 | ~(qp_conf_data.qp_conf[0]);//qp_en;
                // qp_conf_data.qp_conf[0] 即 nak_seq_err_sent
                c2d.pkt_vldn_err                 <= pkt_vldn_err_1 | ~(qp_conf_data.qp_conf[0]) | (qp_conf_data.qp_conf[6]);  // qp_en;
                c2d.error_syndrome               <= err_syndr;
                c2d.qp_pmtu_decoded              <= qp_pmtu_decoded;
                c2d.qp_psn_coef_decoded          <= qp_psn_coef_decoded;
                c2d.qp_psn_rem_decoded           <= qp_psn_rem_decoded;
            end
            //c2d.fatal_data <= fatal_data;
            if (!hv_fsm_busy) begin
                c2d.c1d                       <= c1d;
                //c2d.is_req_psn_matched <= (c1d.bth_psn == (qp_conf_data.last_rq_req[23:0] + 'd1)) ? 1'b1 : 1'b0;
                c2d.buffer_max_addr_from_reth <= c1d.reth_va + c1d.reth_dmalen;
                c2d.reth_dma_len_zero         <= ((c1d.has_reth == 1) & (c1d.reth_dmalen == 0)) ? 1 : 0;
            end  //!hv_fsm_busy
        end  // reset
    end

    // ========================= Pipeline stage 3：错误检查和状态更新 ==========================
    //always @ (*)
    always @(posedge clk) begin
        if (reset) begin
            c3d <= '{default: 0};
        end else begin
            if (!hv_fsm_busy) begin
                c3d.c2d                    <= c2d;
                // RETH地址范围不合法，检查 [reth_va, reth_va + reth_dmalen] 是否完整落在 PD/QP 配置的可访问区间内
                // !(reth_dmalen==0)：仅对非零长度请求检查，零长度放行（单独路径处理）
                // {virt_addr_1, virt_addr_0} > reth_va：PD 配置的起始虚拟地址比请求起始地址还大
                // buffer_max_addr_from_qp_conf < buffer_max_addr_from_reth：请求结束地址（reth_va + reth_dmalen）超过了配置窗口结束地址
                c3d.reth_dma_len_chk_fail  <= (!(c2d.c1d.reth_dmalen == 0) & (({c2d.pd_conf_data.virt_addr_1, c2d.pd_conf_data.virt_addr_0} > c2d.c1d.reth_va) || (c2d.buffer_max_addr_from_qp_conf < c2d.buffer_max_addr_from_reth)));
                // SEND包长度错误
                c3d.send_pkt_len_err       <= send_pkt_len_err;
                // 写包长度错误
                c3d.write_pkt_len_err      <= write_pkt_len_err;
                // PSN匹配检查
                c3d.is_req_psn_matched     <= (c2d.c1d.bth_psn == exp_rq_psn) ? 1'b1 : 1'b0;
                // 旧包/重传包检测
                // 当前 PSN 没有“向前推进”，属于老PSN区间时，is_dup_req=1
                // 当前 PSN 是“向前的新PSN”时，is_dup_req=0
                c3d.is_dup_req             <= ~c2d.qp_conf_data.last_rq_req[23] ^ c2d.c1d.bth_psn[23] ^ ((c2d.c1d.bth_psn[22:0] > c2d.qp_conf_data.last_rq_req[22:0]) ? 1'b1 : 1'b0);
                c3d.reth_dma_len_too_large <= ((c2d.c1d.reth_dmalen[31] == 1) & !(c2d.c1d.reth_dmalen[30:0] == 0));  // reth_dmalen > 2^31 ?
                c3d.req_pkt_len            <= (c2d.reth_dma_len_zero == 1) ? 0 : c2d.c1d.actual_pkt_len;
                // RQ 缓冲区满检查，根据rq写门铃和rq消费者门铃
                c3d.rq_buf_full            <= ((c2d.qp_conf_data.rq_ci_db == c2d.qp_conf_data.rq_wrptr_db + 16'd1) || ((c2d.qp_conf_data.rq_ci_db + c2d.qp_conf_data.q_depth) == (c2d.qp_conf_data.rq_wrptr_db + 16'd1))) ? 1'b1 : 1'b0;
            end  // !hv_fsm_busy
        end  //reset
    end  // always @ (posedge clk)

    // always @ (*)
    // //always @ (posedge clk)
    // begin
    // //  if (reset)
    // //  begin
    // //    c3d <= '{default:0};
    // //  end
    // //  else
    // //  begin
    // //    if (!hv_fsm_busy)
    // //    begin
    //      c3d.c2d = c2d;
    //      c3d.reth_dma_len_chk_fail = (({c2d.pd_conf_data.virt_addr_1,c2d.pd_conf_data.virt_addr_0} > c2d.c1d.reth_va) || 
    //                                    (c2d.buffer_max_addr_from_qp_conf < c2d.buffer_max_addr_from_reth));
    //      c3d.send_pkt_len_err = send_pkt_len_err;
    //      c3d.write_pkt_len_err = write_pkt_len_err;
    // //    end // !hv_fsm_busy
    // //  end //reset
    // end // always @ (posedge clk)

    always @(posedge clk) begin
        if (reset) begin
            qp_conf_2_r             <= 0;
            qp_number_r             <= 0;
            rq_wrptr_db_r           <= 0;
            fatal_qp_idx_r          <= 0;
            reth_r_key_chk_fail_r   <= 0;
            reth_dma_len_chk_fail_r <= 0;
            rq_wrptr_db_addr_r      <= 0;
        end else begin
            qp_conf_2_r             <= c3d.c2d.qp_conf_data.qp_conf[2];
            qp_number_r             <= c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
            rq_wrptr_db_r           <= qp_wrb_dat_w.rq_wrptr_db;
            fatal_qp_idx_r          <= c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
            reth_r_key_chk_fail_r   <= c3d.c2d.reth_r_key_chk_fail & c3d.c2d.c1d.has_reth;
            reth_dma_len_chk_fail_r <= c3d.reth_dma_len_chk_fail & c3d.c2d.c1d.has_reth;
            rq_wrptr_db_addr_r      <= rq_wrptr_db_addr;
        end
    end

    // 第 3 周期后采取行动（进入FSM）
    always @(*) begin
        // default values
        write_qp_conf                     = 1'b0;
        hv_fsm_busy                       = 1'b0;
        hv_fsm_ns                         = hv_fsm_cs;
        nxt_in_err_sts_code               = in_err_sts_code;
        req_pkt_type                      = REQ_AXI_TNFR_TYPE_DISCARD;
        req_pkt_ip_ver                    = IPv6;
        req_dest_addr                     = {(C_M_AXI_ADDR_WIDTH) {1'b0}};
        req_imm_data                      = 44'd0;
        req_tnfr_vld                      = 1'b0;
        o_nak_seq_err_sent                = 1'b0;
        ob_rsp_psn                        = 24'd0;
        ob_rsp_last_psn                   = 24'd0;
        ob_rsp_aeth_syndrome              = 8'd0;
        rnr_syndrome_from_rnr_window_reqd = 1'b0;
        ob_rsp_aeth_msn                   = 24'd0;
        ob_req_indicator                  = 2'b00;
        ob_physical_addr                  = 64'd0;
        ob_dma_len                        = 32'd0;
        ob_rd_req_pmtu                    = 12'd0;
        ob_rsp_vld                        = 1'b0;
        incr_dup_pkt_cnt                  = 1'b0;
        incr_out_nack_pkt_cnt             = 1'b0;
        set_rnr_nack_gen_intr             = 1'b0;
        set_qp_fatal                      = 1'b0;
        set_rq_full_flag                  = 1'b0;
        clr_rq_full_flag                  = 1'b0;
        clr_qp_fatal_flag                 = 1'b0;
        qp_fatal_wqe_proc_rdy             = 1'b0;
        qp_fatal_rsp_hndl_rdy             = 1'b0;
        cnp_pkt_rcvd                      = 1'b0;
        set_ext_fatal_case_flag           = 1'b0;
        clr_ext_fatal_case_flag           = 1'b0;
        rq_buf_ca_we_w                    = 1'b0;
        rq_wrptr_db_we_w                  = 1'b0;
        last_rq_req_we_w                  = 1'b0;
        stat_qp_msn_we_w                  = 1'b0;
        rq_buf_offset_we_w                = 1'b0;
        wr_rq_buf_offset_we_w             = 1'b0;
        nak_seq_err_sent_we_w             = 1'b0;
        incr_all_rd_req_cnt               = 0;
        incr_good_rd_req_cnt              = 0;
        incr_dup_rd_req_cnt               = 0;
        waiting_for_rq_update             = 0;
        waiting_for_rq_update_on_qp_num   = 0;
        ieth_immdt_fifo_data              = 64'h0;
        ieth_immdt_fifo_wr_en             = 1'b0;

        case (hv_fsm_cs)
            HV_IDLE_STATE: begin
                nxt_in_err_sts_code = 5'd0;

                clr_qp_fatal_flag   = 1'b1;

                //wr_rq_buf_offset_en = 1'b1;
                //if (i_qp_fatal_wqe_proc_vld)
                // 处理外部致命错误
                if (qp_fatal_wqe_proc_vld) begin
                    nxt_in_err_sts_code     = 5'b00101;  // 由于 WQE 处理器，QP 进入致命状态
                    qp_fatal_wqe_proc_rdy   = 1'b1;  // 握手确认
                    hv_fsm_ns               = HV_ERR_STATE;  // 跳转到错误状态
                    hv_fsm_busy             = 1'b1;
                    set_ext_fatal_case_flag = 1'b1;  // 标记外部致命错误
                end  //else if (i_qp_fatal_rsp_hndl_vld)
                else if (qp_fatal_rsp_hndl_vld) begin
                    nxt_in_err_sts_code     = 5'b00110;  // 由于响应处理程序，QP 进入致命状态
                    qp_fatal_rsp_hndl_rdy   = 1'b1;  // 握手确认
                    hv_fsm_ns               = HV_ERR_STATE;  // 跳转到错误状态
                    hv_fsm_busy             = 1'b1;
                    set_ext_fatal_case_flag = 1'b1;  // 标记外部致命错误
                end else if (c2d.c1d.hv_data_vld) begin  // 头部验证数据有效
                    hv_fsm_ns = HV_HDRVAL_STATE;  // 有包头数据时进入验证状态
                end  // if (c3d.c2d.c1d.hv_data_vld)
            end  //HV_IDLE_STATE

            // ==================================== 根据包类型进行不同的处理 ====================================
            HV_HDRVAL_STATE: begin
                if (c3d.c2d.qp_conf_vld || tnfr_pend) begin  // qp_conf_vld
                    case (c3d.c2d.c1d.pkt_type)  // pkt_type

                        // 其他包类型处理：若是错误缓冲区启用了，则放入错误缓冲区，否则丢弃该包
                        PKT_TYPE_RSVD: begin
                            if (i_err_buf_en) begin
                                req_pkt_type = REQ_AXI_TNFR_TYPE_ERROR;
                                req_imm_data = {12'h000, c3d.c2d.error_syndrome};
                                req_tnfr_vld = 1'b1;

                                if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                else hv_fsm_ns = HV_IDLE_STATE;
                            end else hv_fsm_ns = HV_IDLE_STATE;
                        end  // PKT_TYPE_RSVD

                        // MAD包处理：
                        PKT_TYPE_MAD: begin
                            //c3d.c2d.error_syndrome[18] == bth_pad_cnt_chk_fail
                            // 验证失败、mad包长度错误或接受队列满
                            if (pkt_vldn_err || mad_pkt_len_err || rq_buf_full) begin
                                // 若错误缓冲区使能，则将该包写入错误缓冲区；否则直接丢弃
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data     = {12'h000, c3d.c2d.error_syndrome};
                                    req_imm_data[19] = mad_pkt_len_err;  // MAD 报文长度是否超过最大限度
                                    req_imm_data[20] = rq_buf_full;  // RQ 缓冲区是否已满
                                    req_tnfr_vld     = 1'b1;
                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;  // 等待传输就绪
                                    else hv_fsm_ns = HV_IDLE_STATE;
                                end else hv_fsm_ns = HV_IDLE_STATE;
                            end else begin
                                req_pkt_type = REQ_AXI_TNFR_TYPE_MAD;
                                req_dest_addr = {rq_buf_ca, 8'h00};  // 目标地址
                                req_pkt_ip_ver = c3d.c2d.c1d.ip_ver;  // IP版本
                                req_imm_data = {
                                    ~qp_hw_hndshk_dis,  // 硬件握手使能
                                    1'b1,  // MAD标志
                                    qp_rq_intr_en,  // RQ中断使能
                                    rq_wrptr_db_addr[13:0],  // RQ写门铃地址
                                    11'd1,  // {保留字段, 固定qp号(qp1, 即1)}
                                    qp_wrb_dat_w.rq_wrptr_db  // RQ写门铃值，必须得是写回数据结构中的，因为这代表准备接收
                                };

                                req_tnfr_vld = 1'b1;

                                // 各字段写使能，更新
                                //rq_buf_ca_en = 1'b1;
                                rq_buf_ca_we_w = 1'b1;
                                //rq_wrptr_db_en = 1'b1;
                                rq_wrptr_db_we_w = 1'b1;
                                //last_rq_req_en = 1'b1;
                                last_rq_req_we_w = 1'b1;

                                if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                else begin
                                    if (qp_hw_hndshk_dis) begin
                                        hv_fsm_busy = 1'b1;
                                        hv_fsm_ns   = HV_WRITE_STATE;  // 需要额外写回操作
                                    end else hv_fsm_ns = HV_IDLE_STATE;
                                end
                            end
                        end  // PKT_TYPE_MAD

                        // PKT_TYPE_SEND系列（SEND操作）处理
                        PKT_TYPE_SEND_ONLY, PKT_TYPE_SEND_FIRST, PKT_TYPE_SEND_MIDDLE, PKT_TYPE_SEND_LAST: begin
                            if (pkt_vldn_err || src_addr_chk_fail) begin
                                // 验证失败或地址检查失败
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data     = {12'h000, c3d.c2d.error_syndrome};
                                    req_imm_data[27] = ip_src_addr_chk_fail;  // 源IP地址检查失败
                                    req_imm_data[28] = mac_src_addr_chk_fail;  // 源MAC地址检查失败
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                    else hv_fsm_ns = HV_IDLE_STATE;
                                end else hv_fsm_ns = HV_IDLE_STATE;
                            end  // if (|err_syndr || src_addr_chk_fail)

                            // PSN匹配 - 正常流程
                            else if (is_req_psn_matched) begin
                                //c3d.c2d.error_syndrome[18] == bth_pad_cnt_chk_fail
                                if (c3d.send_pkt_len_err | c3d.c2d.opcode_seq_chk_fail | c3d.c2d.error_syndrome[18]) begin
                                    // SEND分片长度错误或操作码序列错误
                                    req_pkt_type          = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data[19]      = c3d.send_pkt_len_err;
                                    req_imm_data[16]      = c3d.c2d.opcode_seq_chk_fail;
                                    req_tnfr_vld          = 1'b1;
                                    hv_fsm_busy           = 1'b1;

                                    // 清除 nak_seq_err 发送标志
                                    o_nak_seq_err_sent    = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    if (rdy_to_req_tnfr) begin
                                        // 发送 Invalid(Invalid Request)-NAK 响应
                                        ob_rsp_psn            = exp_req_psn;
                                        // INVLD_REQ 为 NAK code字段，表示错误的Opcode
                                        ob_rsp_aeth_syndrome  = {1'b0, NAK, INVLD_REQ};  // AETH Syndrome字段
                                        ob_rsp_aeth_msn       = stat_qp_msn;
                                        ob_rsp_vld            = 1'b1;

                                        incr_out_nack_pkt_cnt = 1'b1;  // NAK计数器+1

                                        // 00010: 请求数据包长度与配置的 PMTU 不符，或 填充字节数(PADCNT)字段 检查失败
                                        // 00001: 操作码序列检查失败
                                        // 00011: 操作码序列错误和数据包长度 或者 填充字节数(PADCNT)字段错误同时发生
                                        // 错误状态码
                                        nxt_in_err_sts_code   = {3'b0, (c3d.send_pkt_len_err | c3d.c2d.error_syndrome[18]), c3d.c2d.opcode_seq_chk_fail};

                                        hv_fsm_ns             = HV_ERR_STATE;
                                        //hv_fsm_busy = 1'b1;
                                    end
                                end  // if (send_pkt_len_err | opcode_seq_chk_fail) 

                                // RQ缓冲区(槽)满
                                else if (rq_buf_full) begin
                                    set_rq_full_flag = 1'b1;
                                    if (i_err_buf_en) begin
                                        req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                        req_imm_data[20] = 1'b1;
                                        req_tnfr_vld     = 1'b1;
                                        if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                    end

                                    if (!i_err_buf_en || (i_err_buf_en && rdy_to_req_tnfr)) begin
                                        // 发送RNR-NAK响应，即 Receiver Not Ready（RNR） NAK
                                        ob_rsp_psn            = exp_req_psn;
                                        ob_rsp_aeth_syndrome  = {1'b0, RNR_NAK, i_rnr_nak_tval};
                                        ob_rsp_aeth_msn       = stat_qp_msn;
                                        ob_rsp_vld            = 1'b1;

                                        incr_out_nack_pkt_cnt = 1'b1;

                                        // 设置 nak_seq_err 发送标志
                                        o_nak_seq_err_sent    = 1'b1;
                                        nak_seq_err_sent_we_w = 1'b1;

                                        // 设置 RNR-NAK 产生的中断
                                        set_rnr_nack_gen_intr = 1'b1;

                                        hv_fsm_ns             = HV_IDLE_STATE;
                                    end
                                end  // rq_buf_full

                                // 请求正常 - 正常处理
                                else begin
                                    clr_rq_full_flag = 1'b1;  // "清除 RQ 满状态"写使能
                                    req_pkt_type     = (c3d.c2d.c1d.ip_ver == IPv4) ? REQ_AXI_TNFR_TYPE_IPV4_SEND : REQ_AXI_TNFR_TYPE_IPV6_SEND;

                                    // 地址计算
                                    if (c3d.c2d.c1d.send_middle_or_last) req_dest_addr = {rq_buf_ca_plus_offset, 8'h00};  // 分段包使用偏移地址
                                    else req_dest_addr = {rq_buf_ca, 8'h00};  // FIRST包 或 单包使用槽地址

                                    // debug: 1 1 0 00000000001000 00000000011 0000000000000001
                                    // debug: 1 1 0 0008           0003        0001
                                    // 传输元数据
                                    req_imm_data = {
                                        ~qp_hw_hndshk_dis,  // 硬件握手控制
                                        c3d.c2d.c1d.send_only_or_last,  // 是否最后一个包
                                        qp_rq_intr_en,  // 中断使能
                                        rq_wrptr_db_addr[13:0],  // 写门铃地址

                                        // 11位   
                                        {('d11 - C_QP_INDX_WIDTH) {1'b0}},  // 保留
                                        c3d.c2d.c1d.qp_number,  // QP号

                                        qp_wrb_dat_w.rq_wrptr_db  // 写门铃值
                                    };
                                    req_tnfr_vld = 1'b1;

                                    // QP状态更新
                                    last_rq_req_we_w = 1'b1;
                                    o_nak_seq_err_sent = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    if (c3d.c2d.c1d.send_only_or_last) begin
                                        // ONLY包 或 LAST包，更新
                                        stat_qp_msn_we_w = 1'b1;  // 消息序列号写使能
                                        rq_buf_ca_we_w   = 1'b1;  // rq缓冲区当前地址写使能
                                        rq_wrptr_db_we_w = 1'b1;  // 写门铃值写使能
                                    end else begin
                                        // 中间包：只更新偏移
                                        rq_buf_offset_we_w = 1'b1;  // 槽内偏移写使能
                                    end

                                    // 传输控制和响应生成
                                    if (!rdy_to_req_tnfr) begin
                                        // 传输未就绪：等待
                                        hv_fsm_busy                     = 1'b1;
                                        waiting_for_rq_update           = 1'b1;
                                        waiting_for_rq_update_on_qp_num = c3d.c2d.c1d.qp_number;
                                    end else begin
                                        // 传输就绪：发送ACK响应
                                        ob_rsp_psn                      = exp_req_psn;
                                        // flow_credits 是 AETH credit count字段
                                        ob_rsp_aeth_syndrome            = {1'b0, ACK, flow_credits};
                                        ob_rsp_aeth_msn                 = c3d.c2d.c1d.send_only_or_last ? (stat_qp_msn + 'd1) : stat_qp_msn;
                                        ob_rsp_vld                      = 1'b1;

                                        waiting_for_rq_update           = 1'b0;
                                        waiting_for_rq_update_on_qp_num = 0;

                                        // IETH/立即数处理
                                        //ieth_immdt_fifo_data = {qp_wrb_dat_w.rq_wrptr_db,{(8-C_QP_INDX_WIDTH){1'b0}},c3d.c2d.c1d.qp_number,6'h00,c3d.c2d.c1d.has_immdt,c3d.c2d.c1d.has_ieth,c3d.c2d.c1d.ieth_immdt_data};
                                        ieth_immdt_fifo_data            = {qp_wrb_dat_w.rq_wrptr_db, {(11 - C_QP_INDX_WIDTH) {1'b0}}, c3d.c2d.c1d.qp_number, 3'h0, c3d.c2d.c1d.has_immdt, c3d.c2d.c1d.has_ieth, c3d.c2d.c1d.ieth_immdt_data};
                                        ieth_immdt_fifo_wr_en           = c3d.c2d.c1d.has_ieth || c3d.c2d.c1d.has_immdt;

                                        if (c3d.c2d.c1d.send_only_or_last) begin
                                            if (qp_hw_hndshk_dis) begin
                                                hv_fsm_busy = 1'b1;
                                                hv_fsm_ns   = HV_WRITE_STATE;  // 需要额外写回
                                            end else hv_fsm_ns = HV_IDLE_STATE;
                                        end else begin
                                            hv_fsm_ns = HV_IDLE_STATE;
                                        end
                                    end  // if (!rdy_to_req_tnfr)          
                                end  // request is OK
                            end // is_req_psn_matched

                            //  旧包/重传包处理
                            else if (c3d.is_dup_req) begin
                                incr_dup_pkt_cnt     = 1'b1;

                                // 重新发送上次PSN的ACK
                                ob_rsp_psn           = c3d.c2d.qp_conf_data.last_rq_req[23:0];  // 使用上次的PSN
                                ob_rsp_aeth_syndrome = {1'b0, ACK, flow_credits};
                                ob_rsp_aeth_msn      = stat_qp_msn;
                                ob_rsp_vld           = 1'b1;

                                // reset nak_seq_err_sent flag (commenting as per ROHIT)
                                //o_nak_seq_err_sent  = 1'b0;
                                //nak_seq_err_sent_we = 1'b1;

                                hv_fsm_ns            = HV_IDLE_STATE;
                            end // is_dup_req
                            
                            // else PSN-SEQ-ERROR--PSN序列错误
                            else begin
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data[21] = 1'b1;
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                end

                                if (!i_err_buf_en || (i_err_buf_en && rdy_to_req_tnfr)) begin
                                    // Schedule NAK SEQ_ERR
                                    // 发送NAK SEQ_ERR（如果尚未发送）
                                    if (!i_nak_seq_err_sent) begin
                                        ob_rsp_psn                        = exp_req_psn;
                                        rnr_syndrome_from_rnr_window_reqd = 1'b1;  // 可能被RNR窗口覆盖

                                        // PSN_SEQ_ERR表示进入的包PSN和ePSN不匹配
                                        ob_rsp_aeth_syndrome              = {1'b0, NAK, PSN_SEQ_ERR};
                                        ob_rsp_aeth_msn                   = stat_qp_msn;
                                        ob_rsp_vld                        = 1'b1;

                                        incr_out_nack_pkt_cnt             = 1'b1;
                                    end

                                    // set nak_seq_err sent flag
                                    o_nak_seq_err_sent    = 1'b1;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    hv_fsm_ns             = HV_IDLE_STATE;
                                end
                            end  // else PSN-SEQ-ERROR
                        end  // PKT_TYPE_SEND


                        // PKT_TYPE_CNP（拥塞通知包）处理
                        PKT_TYPE_CNP: begin
                            if (pkt_vldn_err || src_addr_chk_fail || (c3d.c2d.c1d.bth_psn != 0)) begin
                                if (i_err_buf_en) begin
                                    // CNP包验证失败
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data     = {12'h000, c3d.c2d.error_syndrome};
                                    req_imm_data[26] = (c3d.c2d.c1d.bth_psn != 0);  // PSN必须为0
                                    req_imm_data[27] = ip_src_addr_chk_fail;
                                    req_imm_data[28] = mac_src_addr_chk_fail;
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                    else hv_fsm_ns = HV_IDLE_STATE;
                                end else hv_fsm_ns = HV_IDLE_STATE;
                            end  // if (|err_syndr || src_addr_chk_fail)

                            else begin
                                cnp_pkt_rcvd = 1'b1;  // 标记CNP包接收，用于拥塞控制
                                hv_fsm_ns    = HV_IDLE_STATE;
                            end
                        end  // PKT_TYPE_CNP

                        PKT_TYPE_UNSUPPORTED: begin
                            if (pkt_vldn_err) begin
                                if (i_err_buf_en) begin
                                    req_pkt_type = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data = c3d.c2d.error_syndrome;  //err_syndr; 
                                    req_tnfr_vld = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                    else hv_fsm_ns = HV_IDLE_STATE;
                                end else hv_fsm_ns = HV_IDLE_STATE;
                            end else if (is_req_psn_matched) begin
                                req_pkt_type          = REQ_AXI_TNFR_TYPE_ERROR;
                                req_imm_data[17]      = 1'b1;
                                req_tnfr_vld          = 1'b1;
                                hv_fsm_busy           = 1'b1;

                                // 重置 QP 状态
                                o_nak_seq_err_sent    = 1'b0;
                                nak_seq_err_sent_we_w = 1'b1;

                                if (rdy_to_req_tnfr) begin
                                    // schedule NAK-Invalid
                                    ob_rsp_psn            = exp_req_psn;
                                    ob_rsp_aeth_syndrome  = {1'b0, NAK, INVLD_REQ};
                                    ob_rsp_aeth_msn       = stat_qp_msn;
                                    ob_rsp_vld            = 1'b1;

                                    incr_out_nack_pkt_cnt = 1'b1;

                                    nxt_in_err_sts_code   = 5'd4;


                                    hv_fsm_ns             = HV_ERR_STATE;
                                    //hv_fsm_busy = 1'b1;
                                end
                            end else if (!c3d.is_dup_req) begin
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data[21] = 1'b1;
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                end

                                if (!i_err_buf_en || (i_err_buf_en && rdy_to_req_tnfr)) begin
                                    // Schedule NAK SEQ_ERR
                                    if (!i_nak_seq_err_sent) begin
                                        ob_rsp_psn                        = exp_req_psn;

                                        //if (rnr_window_flag)
                                        //  ob_rsp_aeth_syndrome = {1'b0, RNR_NAK, RNR_NAK_VAL_PKT_DISCARD};
                                        //else
                                        rnr_syndrome_from_rnr_window_reqd = 1'b1;
                                        ob_rsp_aeth_syndrome              = {1'b0, NAK, PSN_SEQ_ERR};

                                        ob_rsp_aeth_msn                   = stat_qp_msn;
                                        ob_rsp_vld                        = 1'b1;

                                        incr_out_nack_pkt_cnt             = 1'b1;
                                    end

                                    // set nak_seq_err sent flag
                                    o_nak_seq_err_sent    = 1'b1;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    hv_fsm_ns             = HV_IDLE_STATE;
                                end
                            end else  // simply drop 
                                hv_fsm_ns = HV_IDLE_STATE;

                        end  // PKT_TYPE_UNSUPPORTED

                        // PKT_TYPE_RDMA_WR系列（RDMA写操作）处理
                        //generate if (C_EN_INITIATOR_LITE) begin
                        PKT_TYPE_RDMA_WR_ONLY, PKT_TYPE_RDMA_WR_FIRST, PKT_TYPE_RDMA_WR_MIDDLE, PKT_TYPE_RDMA_WR_LAST: begin
                            // 基本验证失败
                            // 错误处理逻辑
                            if (pkt_vldn_err || src_addr_chk_fail) begin
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data     = {12'h000, c3d.c2d.error_syndrome};  // err_syndr};
                                    req_imm_data[27] = ip_src_addr_chk_fail;
                                    req_imm_data[28] = mac_src_addr_chk_fail;
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                    else hv_fsm_ns = HV_IDLE_STATE;
                                end else hv_fsm_ns = HV_IDLE_STATE;
                            end  // if (|err_syndr || src_addr_chk_fail)

                            else if (is_req_psn_matched) begin
                                // RETH DMA长度过大，发送NAK-Invalid
                                if (c3d.reth_dma_len_too_large & c3d.c2d.c1d.has_reth) begin
                                    req_pkt_type          = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data[19]      = c3d.reth_dma_len_too_large;
                                    req_tnfr_vld          = 1'b1;
                                    hv_fsm_busy           = 1'b1;

                                    // reset QP state
                                    o_nak_seq_err_sent    = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    if (rdy_to_req_tnfr) begin
                                        // schedule NAK-Invalid
                                        ob_rsp_psn            = exp_req_psn;
                                        ob_rsp_aeth_syndrome  = {1'b0, NAK, INVLD_REQ};
                                        ob_rsp_aeth_msn       = stat_qp_msn;
                                        ob_rsp_vld            = 1'b1;

                                        incr_out_nack_pkt_cnt = 1'b1;

                                        //nxt_in_err_sts_code = {1'b0, c3d.reth_dma_len_too_large, 1'b0};


                                        nxt_in_err_sts_code   = {3'b101, c3d.reth_dma_len_too_large, 1'b0};  // 10110: RETH DMA 长度检查失败
                                        hv_fsm_ns             = HV_ERR_STATE;
                                        //hv_fsm_busy = 1'b1;
                                    end

                                    // rkey没问题的写包长度错误 或 操作码序列错误 或 error_syndrome[18] == 1，发送NAK-Invalid
                                end else if ((c3d.write_pkt_len_err & !c3d.c2d.reth_r_key_chk_fail) | c3d.c2d.opcode_seq_chk_fail | c3d.c2d.error_syndrome[18]) begin
                                    req_pkt_type          = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data[19]      = c3d.write_pkt_len_err;
                                    req_imm_data[16]      = c3d.c2d.opcode_seq_chk_fail;
                                    req_tnfr_vld          = 1'b1;
                                    hv_fsm_busy           = 1'b1;

                                    // reset QP state
                                    o_nak_seq_err_sent    = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    if (rdy_to_req_tnfr) begin
                                        // schedule NAK-Invalid
                                        ob_rsp_psn            = exp_req_psn;
                                        ob_rsp_aeth_syndrome  = {1'b0, NAK, INVLD_REQ};
                                        ob_rsp_aeth_msn       = stat_qp_msn;
                                        ob_rsp_vld            = 1'b1;

                                        incr_out_nack_pkt_cnt = 1'b1;

                                        nxt_in_err_sts_code   = {3'b100, (c3d.write_pkt_len_err | c3d.c2d.error_syndrome[18]), c3d.c2d.opcode_seq_chk_fail};


                                        hv_fsm_ns             = HV_ERR_STATE;
                                        //hv_fsm_busy = 1'b1;
                                    end
                                end  // if (write_pkt_len_err | opcode_seq_chk_fail) 

                                // RETH  R_KEY字段检查错误 或 地址范围不合法 或 不允许写。该检查仅针对 RETH
                                else if ((c3d.c2d.reth_r_key_chk_fail || c3d.reth_dma_len_chk_fail || (memory_access_write != 1)) & c3d.c2d.c1d.has_reth) begin
                                    set_rq_full_flag = 1'b1;
                                    hv_fsm_busy      = 1'b1;
                                    if (i_err_buf_en) begin
                                        req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                        req_imm_data[20] = 1'b1;
                                        req_tnfr_vld     = 1'b1;
                                    end

                                    // reset QP State
                                    o_nak_seq_err_sent    = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    if (rdy_to_req_tnfr) begin
                                        // Schedule RAC-NAK
                                        // 发送远程访问错误NAK
                                        ob_rsp_psn            = exp_req_psn;
                                        ob_rsp_aeth_syndrome  = {1'b0, NAK, REMOTE_ACC_ERR};
                                        ob_rsp_aeth_msn       = stat_qp_msn;
                                        ob_rsp_vld            = 1'b1;

                                        incr_out_nack_pkt_cnt = 1'b1;


                                        nxt_in_err_sts_code   = {3'b101, c3d.reth_dma_len_chk_fail, (c3d.c2d.reth_r_key_chk_fail | (memory_access_write != 1))};
                                        // Send the QP to fatal state
                                        hv_fsm_ns             = HV_ERR_STATE;
                                        //hv_fsm_busy = 1'b1;
                                    end
                                end  // (reth_r_key_chk_fail || reth_dma_len_chk_fail)
               
                                // 请求正常处理（request is OK）
                                else begin
                                    clr_rq_full_flag = 1'b1;

                                    // 地址选择
                                    if (c3d.c2d.c1d.write_middle_or_last) begin
                                        req_pkt_type  = (c3d.c2d.c1d.ip_ver == IPv4) ? REQ_AXI_TNFR_TYPE_IPV4_WR_REQ : REQ_AXI_TNFR_TYPE_IPV6_WR_REQ;
                                        req_dest_addr = wr_rq_buf_ca_plus_offset;  // 使用偏移后的绝对地址
                                    end else begin
                                        req_pkt_type  = (c3d.c2d.c1d.ip_ver == IPv4) ? REQ_AXI_TNFR_TYPE_IPV4_WR_REQ_RETH : REQ_AXI_TNFR_TYPE_IPV6_WR_REQ_RETH;
                                        req_dest_addr = c3d.c2d.physical_addr_from_reth;  // RETH头部虚拟地址字段转换后的物理地址
                                    end

                                    req_imm_data = {
                                        ~qp_hw_hndshk_dis,  // 1 bit
                                        1'b0,  // write_only_or_last标志（这里固定为0）
                                        qp_rq_intr_en,  // 1 bit
                                        rq_wrptr_db_addr[13:0],  // 14 bits                    
                                        {('d11 - C_QP_INDX_WIDTH) {1'b0}},  // 3 bits
                                        c3d.c2d.c1d.qp_number,  // 8 bits                   
                                        qp_wrb_dat_w.rq_wrptr_db  // 16 bits 不会更新
                                    };

                                    // 零长度检查
                                    req_tnfr_vld = c3d.c2d.reth_dma_len_zero ? 1'b0 : 1'b1;

                                    last_rq_req_we_w = 1'b1;
                                    o_nak_seq_err_sent = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    // 这里没更新写门铃，WRITE和READ都不更新，即都不通知上位机
                                    if (c3d.c2d.c1d.write_only_or_last) begin
                                        stat_qp_msn_we_w = 1'b1;  // 只有完整包或最后包才更新最后的MSN
                                    end else begin
                                        wr_rq_buf_offset_we_w = 1'b1;  // 中间包更新写偏移
                                    end

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                    else begin
                                        // 发送ACK
                                        ob_rsp_psn            = exp_req_psn;
                                        ob_rsp_aeth_syndrome  = {1'b0, ACK, flow_credits};
                                        ob_rsp_aeth_msn       = c3d.c2d.c1d.write_only_or_last ? (stat_qp_msn + 'd1) : stat_qp_msn;
                                        ob_rsp_vld            = 1'b1;

                                        // IETH/立即数处理
                                        ieth_immdt_fifo_data  = {16'h0000, {(11 - C_QP_INDX_WIDTH) {1'b0}}, c3d.c2d.c1d.qp_number, 5'h03, c3d.c2d.c1d.ieth_immdt_data};
                                        ieth_immdt_fifo_wr_en = c3d.c2d.c1d.has_ieth || c3d.c2d.c1d.has_immdt;

                                        if (c3d.c2d.c1d.write_only_or_last) begin
                                            hv_fsm_ns = HV_IDLE_STATE;
                                        end else begin
                                            hv_fsm_ns = HV_IDLE_STATE;
                                        end
                                    end  // if (!rdy_to_req_tnfr)          
                                end  // request is OK
                            end // is_req_psn_matched

                            // 旧包/重传包处理
                            else if (c3d.is_dup_req) begin
                                incr_dup_pkt_cnt     = 1'b1;
                                // schedule last ACK again (it might have lost somewhere)
                                ob_rsp_psn           = last_req_psn;
                                ob_rsp_aeth_syndrome = {1'b0, ACK, flow_credits};
                                ob_rsp_aeth_msn      = stat_qp_msn;
                                ob_rsp_vld           = 1'b1;

                                // reset nak_seq_err_sent flag (commenting as per ROHIT)
                                //o_nak_seq_err_sent  = 1'b0;
                                //nak_seq_err_sent_we = 1'b1;

                                hv_fsm_ns            = HV_IDLE_STATE;
                            end // is_dup_req

                            // PSN序列错误处理（与SEND类似）--else PSN-SEQ-ERROR
                            else begin
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data[21] = 1'b1;
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                end

                                if (!i_err_buf_en || (i_err_buf_en && rdy_to_req_tnfr)) begin
                                    // Schedule NAK SEQ_ERR
                                    if (!i_nak_seq_err_sent) begin
                                        ob_rsp_psn                        = exp_req_psn;
                                        //if (rnr_window_flag)
                                        //  ob_rsp_aeth_syndrome = {1'b0, RNR_NAK, RNR_NAK_VAL_PKT_DISCARD};
                                        //else
                                        rnr_syndrome_from_rnr_window_reqd = 1'b1;
                                        ob_rsp_aeth_syndrome              = {1'b0, NAK, PSN_SEQ_ERR};

                                        ob_rsp_aeth_msn                   = stat_qp_msn;
                                        ob_rsp_vld                        = 1'b1;

                                        incr_out_nack_pkt_cnt             = 1'b1;
                                    end

                                    // set nak_seq_err sent flag
                                    o_nak_seq_err_sent    = 1'b1;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    hv_fsm_ns             = HV_IDLE_STATE;
                                end
                            end  // else PSN-SEQ-ERROR
                        end  // PKT_TYPE_RDMA_WR

                        // PKT_TYPE_RDMA_RD_REQ（RDMA READ请求）处理
                        PKT_TYPE_RDMA_RD_REQ: begin
                            if (rdy_to_req_tnfr) incr_all_rd_req_cnt = 1;  // 统计所有读请求
                            if (pkt_vldn_err || src_addr_chk_fail) begin  // 包验证失败 或 源地址验证失败
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data     = {12'h000, c3d.c2d.error_syndrome};
                                    req_imm_data[27] = ip_src_addr_chk_fail;
                                    req_imm_data[28] = mac_src_addr_chk_fail;
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                    else hv_fsm_ns = HV_IDLE_STATE;
                                end else hv_fsm_ns = HV_IDLE_STATE;
                            end  // if (|err_syndr || src_addr_chk_fail)

                            else if (is_req_psn_matched) begin
                                if (c3d.reth_dma_len_too_large) begin  // RETH DMA长度字段值过大
                                    req_pkt_type          = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data[19]      = c3d.reth_dma_len_too_large;
                                    req_tnfr_vld          = 1'b1;
                                    hv_fsm_busy           = 1'b1;

                                    // reset QP state
                                    o_nak_seq_err_sent    = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    if (rdy_to_req_tnfr) begin
                                        // 发送NAK-Invalid
                                        ob_rsp_psn            = exp_req_psn;
                                        ob_rsp_aeth_syndrome  = {1'b0, NAK, INVLD_REQ};
                                        ob_rsp_aeth_msn       = stat_qp_msn;
                                        ob_rsp_vld            = 1'b1;

                                        incr_out_nack_pkt_cnt = 1'b1;

                                        //nxt_in_err_sts_code = {1'b0, c3d.reth_dma_len_too_large, 1'b0};


                                        nxt_in_err_sts_code   = {3'b101, c3d.reth_dma_len_too_large, 1'b0};
                                        hv_fsm_ns             = HV_ERR_STATE;
                                        //hv_fsm_busy = 1'b1;
                                    end
                                end  // if (write_pkt_len_err | opcode_seq_chk_fail)

                                // RETH  R_KEY字段检查错误 或 地址范围不合法 或 不允许读。该检查仅针对 RETH
                                else if ((c3d.c2d.reth_r_key_chk_fail || c3d.reth_dma_len_chk_fail || (memory_access_read != 1))) begin
                                    set_rq_full_flag = 1'b1;
                                    if (i_err_buf_en) begin
                                        req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                        req_imm_data[20] = 1'b1;
                                        req_tnfr_vld     = 1'b1;
                                    end
                                    hv_fsm_busy           = 1'b1;

                                    // reset QP State
                                    o_nak_seq_err_sent    = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    if (rdy_to_req_tnfr) begin
                                        // Schedule RAE
                                        // 发送远程访问错误
                                        ob_rsp_psn            = exp_req_psn;
                                        ob_rsp_aeth_syndrome  = {1'b0, NAK, REMOTE_ACC_ERR};
                                        ob_rsp_aeth_msn       = stat_qp_msn;
                                        ob_rsp_vld            = 1'b1;

                                        incr_out_nack_pkt_cnt = 1'b1;


                                        nxt_in_err_sts_code   = {3'b101, c3d.reth_dma_len_chk_fail, (c3d.c2d.reth_r_key_chk_fail | (memory_access_read != 1))};
                                        // Send the QP to fatal state
                                        hv_fsm_ns             = HV_ERR_STATE;
                                        //hv_fsm_busy = 1'b1;
                                    end
                                end  // (reth_r_key_chk_fail || reth_dma_len_chk_fail)
                                     //else if (rd_req_fifo_full_r[c3d.c2d.c1d.qp_number])

                                else if (rd_req_fifo_full_data) begin  // 读请求应答队列满
                                    //Schedule NAK Invalid
                                    ob_rsp_psn            = exp_req_psn;
                                    ob_rsp_aeth_syndrome  = {1'b0, NAK, INVLD_REQ};
                                    ob_rsp_aeth_msn       = stat_qp_msn;
                                    ob_rsp_vld            = 1'b1;

                                    incr_out_nack_pkt_cnt = 1'b1;

                                    //nxt_in_err_sts_code = {1'b0, c3d.send_pkt_len_err, c3d.c2d.opcode_seq_chk_fail};

                                    // reset QP state
                                    o_nak_seq_err_sent    = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    nxt_in_err_sts_code   = 5'b10000;
                                    hv_fsm_ns             = HV_ERR_STATE;
                                    hv_fsm_busy           = 1'b1;
                                end //

                                // 读请求正常处理
                                else begin
                                    clr_rq_full_flag      = 1'b1;

                                    last_rq_req_we_w      = 1'b1;
                                    o_nak_seq_err_sent    = 1'b0;
                                    nak_seq_err_sent_we_w = 1'b1;
                                    stat_qp_msn_we_w      = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                    else begin
                                        // 发送ACK并准备读响应中头部数据
                                        ob_rsp_psn           = exp_req_psn;
                                        ob_rsp_aeth_syndrome = {1'b0, ACK, flow_credits};
                                        ob_rsp_aeth_msn      = stat_qp_msn + 'd1;
                                        ob_rsp_vld           = 1'b1;
                                        ob_req_indicator     = 2'b01;  // 读请求
                                        ob_dma_len           = {c3d.c2d.c1d.reth_dmalen[31:0]};
                                        ob_rd_req_pmtu       = {c3d.c2d.qp_pmtu_decoded};
                                        // 计算最后一个PSN
                                        ob_rsp_last_psn      = {c3d.c2d.c1d.bth_psn} + (c3d.c2d.qp_psn_coef_decoded + c3d.c2d.qp_psn_rem_decoded - 1 + (c3d.c2d.c1d.reth_dmalen == 0));
                                        ob_physical_addr     = c3d.c2d.physical_addr_from_reth;

                                        hv_fsm_ns            = HV_IDLE_STATE;
                                        incr_good_rd_req_cnt = 1;
                                    end  // if (!rdy_to_req_tnfr)          
                                end  // request is OK
                            end // is_req_psn_matched
                            
                            // 旧包/重传包处理
                            else if (c3d.is_dup_req) begin
                                incr_dup_pkt_cnt     = 1'b1;
                                incr_dup_rd_req_cnt  = 1;
                                // schedule last ACK again (it might have lost somewhere)
                                //ob_rsp_psn                               = exp_req_psn;
                                // 重新发送读响应信息
                                ob_rsp_psn           = c3d.c2d.c1d.bth_psn;
                                ob_rsp_aeth_syndrome = {1'b0, ACK, flow_credits};
                                ob_rsp_aeth_msn      = stat_qp_msn;
                                ob_rsp_vld           = 1'b1;
                                ob_req_indicator     = 2'b10;  // 重复读请求
                                ob_dma_len           = {c3d.c2d.c1d.reth_dmalen[31:0]};
                                ob_rd_req_pmtu       = {c3d.c2d.qp_pmtu_decoded};
                                ob_rsp_last_psn      = {c3d.c2d.c1d.bth_psn} + (c3d.c2d.qp_psn_coef_decoded + c3d.c2d.qp_psn_rem_decoded - 1 + (c3d.c2d.c1d.reth_dmalen == 0));
                                ob_physical_addr     = c3d.c2d.physical_addr_from_reth;
                                //ob_physical_addr                          = physical_addr_from_reth_1; 

                                // reset nak_seq_err_sent flag (commenting as per ROHIT)
                                //o_nak_seq_err_sent  = 1'b0;
                                //nak_seq_err_sent_we = 1'b1;

                                hv_fsm_ns            = HV_IDLE_STATE;
                            end // is_dup_req

                            // PSN序列错误--else PSN-SEQ-ERROR
                            else begin
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data[21] = 1'b1;
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                                end

                                if (!i_err_buf_en || (i_err_buf_en && rdy_to_req_tnfr)) begin
                                    // Schedule NAK SEQ_ERR
                                    if (!i_nak_seq_err_sent) begin
                                        ob_rsp_psn                        = exp_req_psn;
                                        //if (rnr_window_flag)
                                        //  ob_rsp_aeth_syndrome = {1'b0, RNR_NAK, RNR_NAK_VAL_PKT_DISCARD};
                                        //else
                                        rnr_syndrome_from_rnr_window_reqd = 1'b1;
                                        ob_rsp_aeth_syndrome              = {1'b0, NAK, PSN_SEQ_ERR};

                                        ob_rsp_aeth_msn                   = stat_qp_msn;
                                        ob_rsp_vld                        = 1'b1;

                                        incr_out_nack_pkt_cnt             = 1'b1;
                                    end

                                    // set nak_seq_err sent flag
                                    o_nak_seq_err_sent    = 1'b1;
                                    nak_seq_err_sent_we_w = 1'b1;

                                    hv_fsm_ns             = HV_IDLE_STATE;
                                end
                            end  // else PSN-SEQ-ERROR
                        end  // PKT_TYPE_RDMA_RD_REQ
                        //end
                        //endgenerate //C_EN_INITIATOR_LITE

                        default: hv_fsm_ns = 2'dx;
                    endcase  // pkt_type
                end  // qp_conf_vld 
            end
            // ==================================== 根据包类型进行不同的处理 end ====================================


            HV_WRITE_STATE: begin  // 门铃写回状态
                req_pkt_type  = REQ_AXI_TNFR_TYPE_DB_UPDATE;  // 门铃更新类型
                req_dest_addr = rq_wrptr_db_addr_r;  // 门铃地址
                //req_imm_data = {2'b0, c3d.c2d.qp_conf_data.qp_conf[2], {(21-C_QP_INDX_WIDTH){1'b0}} ,c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0],  qp_wrb_dat_w.rq_wrptr_db};
                req_imm_data  = {2'b00, qp_conf_2_r,  // QP配置位2（中断使能）
{(25 - C_QP_INDX_WIDTH) {1'b0}}, qp_number_r,  // QP号
 rq_wrptr_db_r  // rq写门铃值
};

                if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                else begin
                    req_tnfr_vld = 1'b1;
                    hv_fsm_ns    = HV_IDLE_STATE;
                end
            end  // 2'd2

            HV_ERR_STATE: begin  // 致命错误状态(FATAL)
                req_pkt_type = REQ_AXI_TNFR_TYPE_IN_ERR_STS;  // 致命状态错误
                //req_imm_data = {{(24-C_QP_INDX_WIDTH){1'b0}}, fatal_qp_idx_r, 11'd0, reth_r_key_chk_fail_r, reth_dma_len_chk_fail_r, in_err_sts_code};
                // 此时 req_imm_data 对应手册 Error Status Format
                req_imm_data = {4'h0, {(24 - C_QP_INDX_WIDTH) {1'b0}}, o_qp_fatal_idx,  // 致命错误的QP索引
 11'd0, in_err_sts_code  // 内部错误状态码(即对应手册FATAL Codes)
};

                if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                else begin
                    req_tnfr_vld = 1'b1;
                    if (ext_fatal_case_flag) begin
                        clr_ext_fatal_case_flag = 1'b1;
                        hv_fsm_busy             = 1'b1;
                    end
                    set_qp_fatal = 1'b1;  // 设置QP为致命错误状态
                    hv_fsm_ns    = HV_IDLE_STATE;
                end
            end  // 2'd3

            default: begin
                hv_fsm_ns = 2'dx;
            end  // default 

        endcase
    end  // always @(*)  

    always @(posedge clk)
        if (reset) hv_fsm_cs <= 2'd0;
        else hv_fsm_cs <= hv_fsm_ns;

    always @(posedge clk)
        if (reset) o_inc_dup_pkt_cnt <= 16'd0;
        else if (incr_dup_pkt_cnt) o_inc_dup_pkt_cnt <= o_inc_dup_pkt_cnt + 'd1;

    always @(posedge clk)
        if (reset) o_out_nack_pkt_cnt <= 16'd0;
        else if (incr_out_nack_pkt_cnt) o_out_nack_pkt_cnt <= o_out_nack_pkt_cnt + 'd1;

    //assign o_last_in_req_pkt_opcode = bth_opcode_r;
    //assign o_last_in_req_pkt_qpid = bth_qp_dest_addr_r[7:0];
    //assign o_last_in_req_pkt_psn = bth_psn_r[15:0];
    assign o_last_in_req_pkt_opcode = c1d.bth_opcode;
    assign o_last_in_req_pkt_qpid   = c1d.qp_number[C_QP_INDX_WIDTH-1:0];
    assign o_last_in_req_pkt_psn    = c1d.bth_psn[15:0];

    always @(posedge clk)
        if (reset) o_ob_rsp_vld <= 1'b0;
        else o_ob_rsp_vld <= ob_rsp_vld;

    always @(posedge clk) begin
        o_ob_rsp_qpid                         <= ob_rsp_qpid;
        o_ob_rsp_psn                          <= ob_rsp_psn;
        o_ob_rsp_aeth_syndrome_int            <= ob_rsp_aeth_syndrome;
        o_ob_rsp_aeth_msn                     <= ob_rsp_aeth_msn;
        o_ob_rsp_expl_ack                     <= ob_rsp_expl_ack;
        o_req_indicator                       <= ob_req_indicator;
        o_dma_len                             <= ob_dma_len;
        o_rd_req_pmtu                         <= ob_rd_req_pmtu;
        o_rsp_last_psn                        <= ob_rsp_last_psn;
        o_physical_addr                       <= ob_physical_addr;
        rnr_syndrome_from_rnr_window_reqd_reg <= rnr_syndrome_from_rnr_window_reqd;
    end

    //due to timing fix moving some logic back and forth of a flop to break the lumpped combo logic
    assign o_ob_rsp_aeth_syndrome = (rnr_window_flag == 1'b1 && rnr_syndrome_from_rnr_window_reqd_reg == 1'b1) ? {1'b0, RNR_NAK, RNR_NAK_VAL_PKT_DISCARD} : o_ob_rsp_aeth_syndrome_int;

    // =====================================================================
    // Debug Logic
    // =====================================================================      
    always @(posedge clk)
        if (reset) rq_buf_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) rq_buf_full_sticky <= 1'b0;
        else if (qp_conf_vld && rq_buf_full) rq_buf_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) rq_buf_full_event_cnt <= 32'd0;
        else if (i_clr_debug_sts) rq_buf_full_event_cnt <= 32'd0;
        else if (qp_conf_vld && rq_buf_full) rq_buf_full_event_cnt <= rq_buf_full_event_cnt + 'd1;

    always @(posedge clk)
        if (reset) qp_fatal_req_hv_sticky <= 1'b0;
        else if (i_clr_debug_sts) qp_fatal_req_hv_sticky <= 1'b0;
        else if (set_qp_fatal) qp_fatal_req_hv_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) rnr_window_cnt <= 32'd0;
        else if (i_clr_debug_sts) rnr_window_cnt <= 32'd0;
        else if (rnr_window_closed) rnr_window_cnt <= rnr_window_cnt + 'd1;


    always @(posedge clk)
        if (reset) o_all_rd_req_cnt <= 0;
        else if (incr_all_rd_req_cnt & (o_all_rd_req_cnt != 32'hFFFFFFFF)) o_all_rd_req_cnt <= o_all_rd_req_cnt + 1;

    always @(posedge clk)
        if (reset) o_dup_rd_req_cnt <= 0;
        else if (incr_dup_rd_req_cnt & (o_dup_rd_req_cnt != 32'hFFFFFFFF)) o_dup_rd_req_cnt <= o_dup_rd_req_cnt + 1;

    always @(posedge clk)
        if (reset) o_good_rd_req_cnt <= 0;
        else if (incr_good_rd_req_cnt & (o_good_rd_req_cnt != 32'hFFFFFFFF)) o_good_rd_req_cnt <= o_good_rd_req_cnt + 1;

endmodule
