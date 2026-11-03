`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_rx_req_hdr_validator — RX 引擎请求包头部验证模块
//
// 【模块概述】
// 接收请求路径头部验证模块。请求接收侧处理 QP1/MAD
// 和 unsupported/error/fatal 路径；普通 RC RDMA WRITE/SEND/READ 请求不在接收路径处理。
// 模块包含公共 ETH/IP/UDP/BTH 校验、QP/RQ 配置读取和 MAD RQ doorbell 更新。
//
// 【C1：头部解析与基础校验】
// - 从 512 位头部缓冲区提取公共协议字段（ETH/IP/UDP/BTH）
// - 16 项格式校验：MAC 目标地址、IPv4 版本/IHL/标志/协议、
//   IPv6 版本/NextHeader、IP 长度、UDP 端口 4791/长度、BTH 版本/PadCount
// - 确定包类型：MAD/UNSUPPORTED
//
// 【C2：QP/RQ 配置获取与 MAD 公共验证】
// - 通过 Initiator 接口读取 QP BRAM 配置
// - 2 级写回缓冲区（shift register）解决连续同 QP 包的 RAW hazard
// - 源 MAC/IP 验证（防 spoofing），仅 MAD 有效包会继续进入 RQ/MAD 写回
//
// 【C3：MAD/RQ 写回准备】
// - 判断 MAD 长度、RQ 满状态和公共校验结果
// - 合法 MAD 更新 rq_buf_ca / rq_wrptr_db / last_req；其它请求按 unsupported/error 处理
//
// 【HV FSM（4 状态）】
// - HV_IDLE：检查外部 fatal（TX 引擎 / ACK 管理器），等待 C2 数据
// - HV_HDRVAL：核心决策——按 pkt_type 分支处理 MAD/UNSUPPORTED
// - HV_WRITE：门铃写回——MAD 的 DB_UPDATE
// - HV_ERR：Fatal 错误——写入错误状态缓冲区，置位 set_qp_fatal
//
// 【关键 LUTRAM】
// qp_fatal(2份)、qp_recovery(2份)、qp_enable，以及 MAD/RQ 写回 RAW hazard 缓冲
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module lrdma_rx_req_hdr_validator #(
    parameter C_M_AXI_ADDR_WIDTH       = 32,   // 32
    parameter C_QP_INDX_WIDTH          = 8,    // 5
    parameter C_NUM_QP                 = 256,  // 32
    parameter PKT_DESC_FIFO_DATA_WIDTH = 25,   // 包描述符FIFO数据位宽，25bit
    parameter C_HDR_BUF_ADDR_WIDTH     = 9,    // 包头缓冲区地址位宽，9
    parameter C_REQ_BUF_ADDR_WIDTH     = 9     // 请求缓冲区地址位宽，9
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
    // req_hdr_buf 端口B 接口（该 BRAM 在 RX 引擎中）
    output logic [C_HDR_BUF_ADDR_WIDTH-1:0] req_hdr_buf_addrb,
    output logic                            req_hdr_buf_enb,
    input  logic [                   511:0] req_hdr_buf_doutb,
    // =========================== 请求包 QP ID缓冲区接口--端口B ===========================
    input  logic [                    31:0] req_hdr_qpid_buf_doutb, // QP ID缓冲区，格式为{bth_opcode字段, Dest QP字段/qpid}

    // LRDMA 寄存器接口
    input logic                       i_err_buf_en,      // 错误缓冲区使能
    input logic [               47:0] i_mac_node_addr,   // LRDMA MAC地址
    input logic [               31:0] i_ipv4_node_addr,  // LRDMA IPv4地址
    input logic [              127:0] i_ipv6_node_addr,  // LRDMA IPv6地址
    input logic [C_QP_INDX_WIDTH:0] i_num_qp_enabled,  // 活动的 QP 数量
    input logic [                1:0] i_flow_credits,    // LRDMA Configuration 寄存器[7:6]，无功能定义

    output logic [ 7:0] o_last_in_req_pkt_opcode,  // 最后一个请求包的操作码
    output logic [ 7:0] o_last_in_req_pkt_qpid,    // 最后一个请求包的QP ID
    output logic [15:0] o_last_in_req_pkt_psn,     // 最后一个请求包的PSN

    output logic [C_QP_INDX_WIDTH-1:0] o_rq_full_idx,
    output logic                       o_rq_full_wr_en,
    output logic                       o_rq_full_val,
    output logic [C_QP_INDX_WIDTH-1:0] o_rq_full_reset_idx,
    output logic                       o_rq_full_reset_wr_en,
    output logic                       o_rq_full_reset_val,

    // QP configuration
    // QP 配置
    output logic [C_QP_INDX_WIDTH-1:0] o_qp_conf_idx,  // 输出内部 QP 索引给 Initiator；内部 qpid=0 对应软件 QP1，内部 qpid=1 对应软件 QP2

    input  logic [31:0] i_qp_conf,    // QP配置数据
    input  logic        qp_conf_vld,  // qp_conf_en 的值延迟一个周期 -- MSFF_R(qp_conf_vld, qp_conf_en, clk, rst_p)，因为这些配置数据均在en下一个周期后才有数据
    output logic        qp_conf_en,   // qp配置读取使能

    input  logic [C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_ba,
    output logic                          rq_buf_ba_en,

    input  logic [C_M_AXI_ADDR_WIDTH-9:0] i_rq_buf_ca,
    output logic [C_M_AXI_ADDR_WIDTH-9:0] o_rq_buf_ca,   // RQ 写入地址
    output logic                          rq_buf_ca_en,
    output logic                          rq_buf_ca_we,

    // 用户配置，并使用该地址用于rq硬件握手
    input  logic [C_M_AXI_ADDR_WIDTH-1:0] i_rq_wrptr_db_addr,  // RQ 写门铃地址寄存器。仅在接收到 MAD（QP1 SEND_ONLY）数据包消耗 RQ WQE 时更新。
    output logic                          rq_wrptr_db_addr_en,

    input  logic [15:0] i_q_depth,
    output logic        q_depth_en,

    input  logic [15:0] i_rq_ci_db,
    output logic        rq_ci_db_en, // rq_ci_db读取使能

    input  logic [31:0] i_last_req,
    output logic [31:0] o_last_req,
    output logic        last_req_en,
    output logic        last_req_we,

    input  logic [47:0] i_remote_qp_mac_addr,
    output logic        remote_qp_mac_addr_en,

    input  logic [127:0] i_remote_qp_ip_addr,
    output logic         remote_qp_ip_addr_en,

    input  logic [15:0] i_rq_wrptr_db,
    output logic [15:0] o_rq_wrptr_db,
    output logic        rq_wrptr_db_en,
    output logic        rq_wrptr_db_we,

    input  logic                       i_qp_fatal_wr_en,
    input  logic [C_QP_INDX_WIDTH-1:0] i_qp_fatal_wr_idx,
    input  logic                       i_qp_fatal_wr_data,
    output logic [C_QP_INDX_WIDTH-1:0] o_qp_fatal_idx,
    output logic                       set_qp_fatal,        // qp致命错误标志

    input logic [C_QP_INDX_WIDTH-1:0] rsp_qp_fatal_idx,
    input logic                       set_rsp_qp_fatal,

    input  logic [C_QP_INDX_WIDTH-1:0] i_qp_fatal_tx_engine,      // fatal qp的id
    input  logic                       i_qp_fatal_tx_engine_vld,
    output logic                       o_qp_fatal_tx_engine_rdy,  // 直连 qp_fatal_tx_engine_rdy

    input  logic [C_QP_INDX_WIDTH-1:0] i_qp_fatal_ack_mgr,      // fatal qp的id
    input  logic                       i_qp_fatal_ack_mgr_vld,
    output logic                       o_qp_fatal_ack_mgr_rdy,  // 直连 qp_fatal_ack_mgr_rdy

    // 通向rx_req_axi_mstr 接口，三级流水后才有数据，请求包元数据
    output logic [                    31:0] req_pkt_id,
    output logic [                     3:0] req_pkt_type,
    output logic                            req_pkt_ip_ver,
    output logic [                    15:0] req_rq_pi_db,
    output logic [  C_M_AXI_ADDR_WIDTH-1:0] req_dest_addr,        // 最终写入内存的目的地址
    output logic [C_REQ_BUF_ADDR_WIDTH-1:0] req_buf_addr_offset,
    output logic [                    12:0] req_pkt_len,          // 总包长度，不含 FCS
    // req_imm_data字段：
    // 发生错误时：对应手册 Packet Validation Error Syndrome 或者 Error Status Format
    // 正常请求时：{ 1bit硬件握手控制, 1bitMAD标志, 1bitRQ中断使能, 14bitRQ门铃地址, 11bit{高位填 0, QP号}, 16bit门铃值 }
    output logic [                    43:0] req_imm_data,         // 请求立即控制字段，与 req axi master 搭配
    output logic                            req_tnfr_vld,         // 握手信号(与rx_req_axi_mstr模块)，表示 该模块 的请求包元数据传输有效
    input  logic                            rdy_to_req_tnfr,      // 握手信号(与rx_req_axi_mstr模块)，表示 rx_req_axi_mstr模块 的请求包元数据传输已准备好。为 0 时本状态机要 busy 等待，防止丢握手。
    output logic [     C_QP_INDX_WIDTH-1:0] req_qpid,

    input logic                       i_qp_enable_wr_en,
    input logic [C_QP_INDX_WIDTH-1:0] i_qp_enable_wr_idx,
    input logic                       i_qp_enable_wr_data,

    output logic                       waiting_for_rq_update,
    output logic [C_QP_INDX_WIDTH-1:0] waiting_for_rq_update_on_qp_num,

    input logic                       init_start,
    input logic [C_QP_INDX_WIDTH-1:0] init_count,
    input logic                       qp_under_recovery_wr_en,
    input logic [C_QP_INDX_WIDTH-1:0] qp_under_recovery_wr_idx,
    input logic                       qp_under_recovery_wr_data
);

    // 包类型定义
    // ------------------------------------------
    // 如果 i_err_buf_en 设置为 1，则会验证所有传入的数据包，并将所有未通过数据包验证的数据包头发送到错误缓冲区（基地址由 ERRBUFBA[31:0] 指定）
    localparam [2:0] PKT_TYPE_RSVD = 3'h0;  // 其他包
    localparam [2:0] PKT_TYPE_MAD = 3'h1;  // MAD包，用于建立和管理连接
    localparam [2:0] PKT_TYPE_UNSUPPORTED = 3'h6;  // 无效操作码包，不支持的操作码

    // 传输包类型定义，通过 req_pkt_type 传给 lrdma_rx_req_axi_master 模块的 pkt_type 和 fifo_out_pkt_type，类型与 lrdma_rx_req_axi_master 模块 对应
    // ------------------------------------------
    localparam [3:0] REQ_AXI_TNFR_TYPE_DISCARD = 4'd0;
    localparam [3:0] REQ_AXI_TNFR_TYPE_ERROR = 4'd1;
    localparam [3:0] REQ_AXI_TNFR_TYPE_MAD = 4'd2;
    localparam [3:0] REQ_AXI_TNFR_TYPE_DB_UPDATE = 4'd3;
    localparam [3:0] REQ_AXI_TNFR_TYPE_IN_ERR_STS = 4'd4;

    localparam IPv4 = 1'b0;
    localparam IPv6 = 1'b1;

    localparam QP1 = 'd0;

    localparam [2:0] RC = 3'b000;
    localparam [2:0] UD = 3'b011;

    // BTH操作码定义
    // ------------------------------------------
    localparam [4:0] SEND_ONLY = 5'b00100;  // 仅用于MAD检测
    localparam [1:0] ACK = 2'b00;
    localparam [1:0] NAK = 2'b11;

    // 第三阶段状态机状态定义
    // ------------------------------------------
    localparam [1:0] HV_IDLE_STATE = 2'd0;  // 空闲状态
    localparam [1:0] HV_HDRVAL_STATE = 2'd1;  // 头部验证状态
    localparam [1:0] HV_WRITE_STATE = 2'd2;  // 写回状态
    localparam [1:0] HV_ERR_STATE = 2'd3;  // 内部错误状态(FATAL)

    logic [                  C_NUM_QP:0] i;

    logic                                hdr_ftchg_done;  // 头部获取完成标志，随着第二拍数据一起输出
    logic                                req_hdr_buf_enb_r;
    logic                                hv_data_vld;  // 头部数据有效，hv_data_vld <= hdr_ftchg_done
    logic [                       511:0] prev_req_hdr_buf_doutb;  // 上一拍数据
    logic [                      1023:0] hdr_data;  // 头部数据 -- {上一拍数据，本拍数据}

    logic [                         0:0] hdr_ftchg_fsm_cs;  // 头部获取状态机状态
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
    logic [                        23:0] bth_qp_dest_addr;  // QP ID
    logic                                ipv4_bth_expl_ack;
    logic                                ipv6_bth_expl_ack;
    logic                                bth_expl_ack;  // BTH头部A字段：请求响应方在相关 QP 上安排确认
    logic [                         7:0] bth_opcode;
    logic [                        23:0] ipv4_bth_psn;
    logic [                        23:0] ipv6_bth_psn;
    logic [                        23:0] bth_psn;
    logic [                        23:0] qp_conf_min;
    logic [                        23:0] qp_conf_max;
    logic                                qp_state_fatal;


    logic                                mac_dest_addr_chk_fail;  // 若包头MAC目的地址不是本节点MAC地址，则标记失败
    logic                                ipv4_ver_chk_fail;
    logic                                ipv4_ihl_chk_fail;
    logic                                ipv4_flg_chk_fail;
    logic                                ipv4_frag_offset_chk_fail;
    logic                                ipv4_prot_chk_fail;
    logic                                ipv4_dest_addr_chk_fail;
    logic                                ipv4_length_chk_fail;
    logic                                ipv4_src_addr_chk_fail;
    logic                                ipv6_ver_chk_fail;
    logic                                ipv6_next_hdr_chk_fail;
    logic                                ipv6_dest_addr_chk_fail;
    logic                                ipv6_length_chk_fail;
    logic                                ipv6_src_addr_chk_fail;
    logic                                udp_dest_port_chk_fail;
    logic                                udp_length_chk_fail;
    logic                                pkt_len_chk_fail;
    logic                                bth_tver_chk_fail;
    logic                                dest_qp_chk_fail;
    logic                                ip_length_chk_fail;
    logic                                ip_dest_addr_chk_fail;
    logic                                ip_ver_chk_fail;
    logic                                ip_src_addr_chk_fail;
    logic                                mac_src_addr_chk_fail;
    logic                                ip_src_addr_chk_fail_w;
    logic                                mac_src_addr_chk_fail_w;
    logic                                src_addr_chk_fail;
    logic                                mad_pkt_len_err;

    logic                                dest_qp_chk_fail_r;
    logic                                bth_tver_chk_fail_r;
    logic                                pkt_len_chk_fail_r;
    logic                                udp_length_chk_fail_r;
    logic                                ip_length_chk_fail_r;
    logic                                ipv4_hdr_chksum_fail_r;
    logic                                ip_dest_addr_chk_fail_r;
    logic                                ipv4_frag_offset_chk_fail_r;
    logic                                ipv4_flg_chk_fail_r;
    logic                                ipv4_ihl_chk_fail_r;
    logic                                ip_ver_chk_fail_r;
    logic                                mac_dest_addr_chk_fail_r;

    logic                                ip_ver_r;
    logic [                         7:0] bth_opcode_r;

    logic [                        23:0] bth_psn_r;
    logic                                bth_expl_ack_r;
    logic [                        47:0] mac_src_addr_r;
    logic [                        31:0] ipv4_src_addr_r;
    logic [                       127:0] ipv6_src_addr_r;

    // RoCE 业务负载长度，不含 L2/L3/L4/BTH，也不含 ICRC；
    // 若报文带 ImmDt/IETH（4B），该值会在后续分支里再减 4B。
    logic [                        12:0] bth_pyld_len;  // BTH负载长度(不含BTH头部的长度)，这是“最终”的有效载荷长度
    // 基础业务负载长度：
    // = actual_pkt_len - (Eth+IP+UDP+BTH+ICRC)
    // IPv4: -58 = 14+20+8+16+4, IPv6: -78 = 14+40+8+16+4
    logic [                        12:0] bth_pyld_len_base;

    logic [                        31:0] err_syndr;

    logic                                pkt_vldn_err_1;
    logic                                pkt_vldn_err;

    logic [                        23:0] exp_req_psn;
    logic [                        23:0] p_exp_req_psn;

    logic                                rq_buf_full;

    logic [      C_M_AXI_ADDR_WIDTH-1:0] rq_wrptr_db_addr_r;

    logic                                qp_rq_intr_en;
    logic [      C_M_AXI_ADDR_WIDTH-1:0] rq_buf_ba;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] rq_buf_ca;
    logic [      C_M_AXI_ADDR_WIDTH-1:0] rq_wrptr_db_addr;
    logic [                        15:0] q_depth;
    logic [                        15:0] rq_ci_db;
    logic [                         7:0] last_req_opcode;  // 上一次请求的操作码
    logic [                        23:0] last_req_psn;  // 上一次请求的PSN
    logic [                        23:0] p_last_req_psn;
    logic [                        15:0] rq_wrptr_db;

    logic [                         4:0] in_err_sts_code;  // 内部错误状态码(即对应手册FATAL Codes)
    logic [                         4:0] nxt_in_err_sts_code;

    logic [                         2:0] pkt_type;

    logic                                qp_fatal_combo;
    logic                                qp_recovery_combo;
    logic                                qp_fatal_combo_c1d;
    logic                                qp_recovery_combo_c1d;

    logic [                         4:0] flow_credits;

    logic                                tnfr_pend;  // 传输挂起标志，表示有数据传输请求但还未被接受处理的状态

    logic [                         1:0] hv_fsm_cs;
    logic [                         1:0] hv_fsm_ns;

    logic                                hv_fsm_busy;  // 忙碌标志：1-表示正在忙碌处理，不能接受新的数据包

    logic                                set_rq_full_flag;
    logic                                clr_rq_full_flag;

    logic [         C_QP_INDX_WIDTH-1:0] rsp_qp_fatal_idx_r;
    logic                                clr_qp_fatal_flag;
    logic                                rsp_qp_fatal_flag;

    logic [         C_QP_INDX_WIDTH-1:0] fatal_qp_idx;

    logic                                bth_psn_0;  // bth_psn == 0 ? 1: 0

    logic                                set_ext_fatal_case_flag;
    logic                                clr_ext_fatal_case_flag;
    logic                                ext_fatal_case_flag;

    logic                                read_qp_conf;  // 读QP配置信号
    logic                                write_qp_conf;  // 写QP信息信号

    logic                                update_qp_info;  // qp写回请求。update_qp_info_w 上升沿检测信号

    logic [         C_QP_INDX_WIDTH-1:0] wrb_qp_number;

    logic [                         7:0] bth_opcode_mem;

    logic [         C_QP_INDX_WIDTH-1:0] wrb_qp_number_c;
    logic [         C_QP_INDX_WIDTH-1:0] wrb_qp_number_r;

    logic [      C_M_AXI_ADDR_WIDTH-9:0] d_rq_buf_ca;
    logic [                        31:0] d_last_req;
    logic [                        15:0] d_rq_wrptr_db;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] c_rq_buf_ca;
    logic [                        31:0] c_last_req;
    logic [                        15:0] c_rq_wrptr_db;
    logic [      C_M_AXI_ADDR_WIDTH-9:0] r_rq_buf_ca;
    logic [                        31:0] r_last_req;
    logic [                        15:0] r_rq_wrptr_db;

    logic [      C_M_AXI_ADDR_WIDTH-9:0] p_rq_buf_ca;
    logic [                        31:0] p_last_req;
    logic [                        15:0] p_rq_wrptr_db;

    logic                                qp_under_recovery_wr_en_r;


    logic                                rq_buf_ca_we_w;
    logic                                last_req_we_w;
    logic                                rq_wrptr_db_we_w;

    logic                                write_qp_info;  // = !read_qp_conf
    logic                                qp_fatal_tx_engine_rdy;  // 
    logic                                qp_fatal_ack_mgr_rdy;
    logic                                async_fatal_asserted;  // 表示有QP处于致命错误状态需要立即处理，这是一个异步的、高优先级的错误信号

    logic                                qp_conf_2_r;
    logic [         C_QP_INDX_WIDTH-1:0] qp_number_r;
    logic [                        15:0] rq_wrptr_db_r;
    logic [         C_QP_INDX_WIDTH-1:0] fatal_qp_idx_r;

    logic [         C_QP_INDX_WIDTH-1:0] qp_fatal_tx_engine;  // 直连 i_qp_fatal_tx_engine
    logic                                qp_fatal_tx_engine_vld;  // 直连 i_qp_fatal_tx_engine_vld
    logic [         C_QP_INDX_WIDTH-1:0] qp_fatal_ack_mgr;  // 直连 i_qp_fatal_ack_mgr
    logic                                qp_fatal_ack_mgr_vld;  // 直连 i_qp_fatal_ack_mgr_vld
    logic [       C_QP_INDX_WIDTH-1 : 0] c3d_qp_number_r;
    logic [       C_QP_INDX_WIDTH-1 : 0] rsp_qp_fatal_idx_2r;
    logic                                set_qp_fatal_r;
    logic                                rsp_qp_fatal_flag_r;
    logic                                clr_qp_fatal_flag_r;
    logic                                rd_req_pkt_desc_fifo_done;
    logic                                data_missed;
    logic [PKT_DESC_FIFO_DATA_WIDTH-1:0] req_pkt_desc_fifo_dout_hold;

    // QP配置结构
    typedef struct packed {
        logic [31:0]                   qp_conf;             // QP 配置(只读)，存放在 QP Configuration QPi(QPCONFi) 寄存器中
        logic [C_M_AXI_ADDR_WIDTH-1:0] rq_buf_ba;           // RQ 环形缓冲区基地址(只读)，当 rq_buf_ca 回卷时回到 rq_buf_ba
        logic [C_M_AXI_ADDR_WIDTH-9:0] rq_buf_ca;           // RQ 写入地址，作为请求落内存的位置基准，配合 offset 得最终地址
        logic [C_M_AXI_ADDR_WIDTH-1:0] rq_wrptr_db_addr;    // RQ 写门铃地址(只读)，就是存放RQ写门铃值的地址。可以走AXI addr和rx_engine_o_rq_db_addr
        logic [15:0]                   q_depth;             // 队列深度(只读)
        logic [15:0]                   rq_ci_db;            // RQ 消费者门铃(只读)，和 rq_wrptr_db 比较判断是否队列满
        logic [31:0]                   last_req;            // MAD/RQ 上一次请求记录 {opcode, psn}；普通 RC 请求不参与该序列处理
        logic [47:0]                   remote_qp_mac_addr;  // 远程QP MAC地址(只读)
        logic [127:0]                  remote_qp_ip_addr;   // 远程QP IP地址(只读)
        logic [15:0]                   rq_wrptr_db;         // RQ 写门铃值
    } qp_conf_t;

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
        logic bit_18; // 对于收到的FIRST和MIDDLE请求/响应，BTH Pad 位不是 2'b00
        logic bit_17;
        logic bit_16;
        logic qp_state_fatal;  // BTH 目标地址中指定的 QP 未配置或未启用
        logic dest_qp_chk_fail;  // 不支持 BTH 中的 QP 目标地址
        logic bth_tver_chk_fail;  // BTH 版本不是 4'b0000
        logic pkt_len_chk_fail;     // 数据包实际长度 与 头部长度字段值减去头部长度后的值 不一致
        logic udp_length_chk_fail; // UDP 长度字段与 IPv4 总长度或 IPv6 有效负载长度不一致
        logic ip_length_chk_fail;  // IPv4 总长度字段值不在范围内 或 IPv6 有效负载长度字段值不在范围内
        logic ipv4_hdr_chksum_fail;  // IPv4 报头校验和错误。
        logic ip_dest_addr_chk_fail;  // IPv4 报头中的 IPv4 目标地址与配置的 LRDMA IPv4 地址不匹配
        logic bit_7;
        logic ipv4_frag_offset_chk_fail;  // IPv4 中的片段偏移不为 0
        logic ipv4_flg_chk_fail;  // IPv4 中的标志位不是 3'b010
        logic bit_4;  // 固定为 0
        logic ipv4_ihl_chk_fail;  // IPv4 标头长度不等于 20 字节
        logic ip_ver_chk_fail;  // IP 版本与 LRDMA 配置不符
        logic bit_1;  // 固定为 0
        logic mac_dest_addr_chk_fail;  // 以太网头中接收到的 MAC 目标地址与配置的 LRDMA MAC 地址不匹配
    } err_syndr_t;

    // 第一级流水线数据
    typedef struct packed {
        logic hv_data_vld;  // 头部数据有效
        logic [C_QP_INDX_WIDTH-1:0] qp_number;  // 目标队列对号 (BTH头部中的Dest QP字段)
        logic [23:0] bth_psn;  // BTH包序列号 (24位)
        logic [2:0] pkt_type;  // 数据包类型
        logic ip_ver;  // IP版本 (IPv4=0, IPv6=1)
        logic reg_ip_ver;  // 寄存器化的IP版本
        logic [7:0] bth_opcode;  // BTH操作码 (定义RDMA操作类型)
        logic bth_expl_ack;  // BTH头部A字段：请求响应方在相关 QP 上安排确认
        logic [47:0] mac_src_addr;  // 源MAC地址 (48位)
        logic [31:0] ipv4_src_addr;  // 源IPv4地址 (32位)
        logic [127:0] ipv6_src_addr;  // 源IPv6地址 (128位)
        logic [31:0] pkt_id;  // 包ID
        logic [C_REQ_BUF_ADDR_WIDTH-1:0] pkt_src_addr_offset;  // 包源地址偏移
        logic [12:0] actual_pkt_len;  // 实际包长度（包含所有头部和ICRC，不包括FCS）
        logic mad_pkt_len_err;  // MAD包长度错误
        logic [12:0] bth_pyld_len;  // BTH负载长度(不含BTH头部的长度)，这是最终的有效负载长度
    } cycle_1_data_t;

    // 第二级流水线数据
    typedef struct packed {
        err_syndr_t error_syndrome;
        cycle_1_data_t c1d;
        logic qp_conf_vld;  // QP配置有效标志
        qp_conf_t qp_conf_data;  // QP配置数据
        logic ip_src_addr_chk_fail;  // IP源地址检查失败
        logic mac_src_addr_chk_fail;  // MAC源地址检查失败
        logic pkt_vldn_err;  // 包验证错误，总的验证失败标志

        // 零长度请求短路控制
    } cycle_2_data_t;

    // 第三级流水线数据
    typedef struct packed {
        cycle_2_data_t c2d;
        logic [12:0] req_pkt_len;
        logic rq_buf_full;
    } cycle_3_data_t;

    // QP写回数据结构 - 包含所有需要更新的QP状态
    typedef struct packed {
        // === 实际数据字段 ===
        logic [C_M_AXI_ADDR_WIDTH-9:0] rq_buf_ca;  // 接收队列写入地址
        logic [31:0] last_req;  // MAD/RQ 上一次请求记录，{8 位 opcode, 24 位 PSN}
        logic [15:0] rq_wrptr_db;  // RQ 写门铃值
        logic [C_QP_INDX_WIDTH-1:0] wrb_qp_number;  // QP号(用于匹配)

        // === 各字段写使能，决定是否更新该字段 ===
        logic rq_buf_ca_we;    // RQ 写入地址写使能
        logic last_req_we;     // 最后请求写使能
        logic rq_wrptr_db_we;  // RQ写门铃写使能
    } qp_write_back_data_t;

    cycle_1_data_t                             c1d;
    cycle_2_data_t                             c2d;
    cycle_3_data_t                             c3d;
    qp_conf_t                                  qp_conf_data;

    // 缓冲区变量
    qp_write_back_data_t                       qp_wrb_dat_0;  // 缓冲区0 - 主缓冲区
    qp_write_back_data_t                       qp_wrb_dat_1;  // 缓冲区1 - 辅助缓冲区
    qp_write_back_data_t                       qp_wrb_dat_w;  // 写入数据 - c3d 周期计算出的待写回 QP 数据结构，qp_wrb_dat_w.xxx_we 决定字段是否更新
    qp_write_back_data_t                       qp_wrb_dat_r;  // 读取数据 - 根据读目标 QP 号选择合适的写回缓冲区
    // 更新触发信号
    logic                                      update_qp_info_r;  // 上一周期的更新QP信息触发信号，用于update_qp_info上升沿检测
    logic                                      update_qp_info_w;  // 更新QP信息触发信号--rq_buf_ca_we_w | last_req_we_w | rq_wrptr_db_we_w
    logic                                      qp_wrb_wr_ptr;

    logic                [                1:0] wrb_entry_valid;  // 2'b00: 缓冲区空；2'b01: qp_wrb_dat_0有效；2'b11: 两个缓冲区都有数据
    logic                                      get_qp_info_from_buffer;  // 从缓冲区获取QP信息标志

    logic                                      dbg_hv_data_vld;
    logic                [C_QP_INDX_WIDTH-1:0] dbg_qp_number;
    logic                [               23:0] dbg_bth_psn;
    logic                [                2:0] dbg_pkt_type;
    logic                [                7:0] dbg_psn_from_qp_conf;
    logic                                      dbg_qp_conf_vld;
    logic                                      dbg_write_qp_info;
    logic                                      dbg_wrb_qp_write;
    logic                [                1:0] dbg_wrb_entry_valid;


    assign dbg_hv_data_vld             = c3d.c2d.c1d.hv_data_vld;
    assign dbg_qp_number               = c3d.c2d.c1d.qp_number;
    assign dbg_bth_psn                 = c3d.c2d.c1d.bth_psn;
    assign dbg_pkt_type                = c3d.c2d.c1d.pkt_type;
    assign dbg_psn_from_qp_conf        = i_last_req[7:0];
    assign dbg_qp_conf_vld             = qp_conf_vld;
    assign dbg_write_qp_info           = write_qp_info;
    assign dbg_wrb_qp_write            = rq_buf_ca_we | last_req_we | rq_wrptr_db_we;
    assign dbg_wrb_entry_valid         = wrb_entry_valid;


    // 任一情况发生，都需要更新 Intiator 中存放的QP信息
    assign update_qp_info_w            = rq_buf_ca_we_w | last_req_we_w | rq_wrptr_db_we_w;
    assign qp_wrb_dat_w.rq_buf_ca_we   = rq_buf_ca_we_w;  // rq_buf_ca 需要更新
    assign qp_wrb_dat_w.last_req_we    = last_req_we_w;  // last_req 需要更新
    assign qp_wrb_dat_w.rq_wrptr_db_we = rq_wrptr_db_we_w;  // rq_wrptr_db 需要更新
    assign qp_wrb_dat_w.wrb_qp_number  = c3d.c2d.c1d.qp_number;


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
    always @(*) begin
        qp_wrb_dat_r            = '{default: 0};
        get_qp_info_from_buffer = 0;
        if (qp_conf_vld) begin  // QP 配置读取请求有效
            case (wrb_entry_valid)
                // 在缓冲区空的情况下，如果同时发生读写同一QP，直接使用即将写回的数据，这是最新的状态。
                2'b00: begin  // 缓冲区空
                    if (update_qp_info) begin  // 同时有写回请求
                        // 写回 QP 号与读取 QP 号相同
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
                        // 写回 QP 号与读取 QP 号相同
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
                        // 写回 QP 号与读取 QP 号相同
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

    // 需要写回 QP 寄存器的 QP 配置先进入写回缓冲区；读口空闲时再写回。
    // ===================================== 缓冲区管理逻辑 =====================================
    always @(posedge clk) begin
        if (reset) begin
            qp_wrb_dat_0    <= '{default: 0};
            qp_wrb_dat_1    <= '{default: 0};
            wrb_entry_valid <= 0;
            wrb_qp_number_r <= 0;
            r_rq_buf_ca     <= 'd0;
            r_last_req      <= 'd0;
            r_rq_wrptr_db   <= 'd0;
        end else begin

            // 保存本拍 QP 号和相关字段，便于在 QP 信息写回时使用
            wrb_qp_number_r <= wrb_qp_number_c;
            r_rq_buf_ca     <= c_rq_buf_ca;
            r_last_req      <= c_last_req;
            r_rq_wrptr_db   <= c_rq_wrptr_db;

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
        wrb_qp_number  = 0;
        d_rq_buf_ca    = 0;
        d_last_req     = 0;
        d_rq_wrptr_db  = 0;
        rq_buf_ca_we   = 0;
        last_req_we    = 0;
        rq_wrptr_db_we = 0;
        case (wrb_entry_valid)
            2'b00: begin  // 缓冲区空
                if (write_qp_info) begin
                    if (update_qp_info) begin  // 有回写 & 有更新，使用最新数据
                        wrb_qp_number  = qp_wrb_dat_w.wrb_qp_number;
                        d_rq_buf_ca    = qp_wrb_dat_w.rq_buf_ca;
                        d_last_req     = qp_wrb_dat_w.last_req;
                        d_rq_wrptr_db  = qp_wrb_dat_w.rq_wrptr_db;

                        rq_buf_ca_we   = qp_wrb_dat_w.rq_buf_ca_we;
                        last_req_we    = qp_wrb_dat_w.last_req_we;
                        rq_wrptr_db_we = qp_wrb_dat_w.rq_wrptr_db_we;
                    end
                end
            end

            2'b01, 2'b11: begin  // 有缓冲数据，使用缓冲区数据
                if (write_qp_info) begin
                    wrb_qp_number  = qp_wrb_dat_0.wrb_qp_number;
                    d_rq_buf_ca    = qp_wrb_dat_0.rq_buf_ca;
                    d_last_req     = qp_wrb_dat_0.last_req;
                    d_rq_wrptr_db  = qp_wrb_dat_0.rq_wrptr_db;

                    rq_buf_ca_we   = qp_wrb_dat_0.rq_buf_ca_we;
                    last_req_we    = qp_wrb_dat_0.last_req_we;
                    rq_wrptr_db_we = qp_wrb_dat_0.rq_wrptr_db_we;
                end
            end

            default: begin
                wrb_qp_number  = 0;
                d_rq_buf_ca    = 0;
                d_last_req     = 0;
                d_rq_wrptr_db  = 0;
                rq_buf_ca_we   = 0;
                last_req_we    = 0;
                rq_wrptr_db_we = 0;
            end
        endcase
    end

    always @(*) begin
        case (wrb_entry_valid)
            2'b00: begin  // 这里不会写
                if (update_qp_info & !write_qp_info) begin
                    wrb_qp_number_c = qp_wrb_dat_w.wrb_qp_number;
                    c_rq_buf_ca     = qp_wrb_dat_w.rq_buf_ca;
                    c_last_req      = qp_wrb_dat_w.last_req;
                    c_rq_wrptr_db   = qp_wrb_dat_w.rq_wrptr_db;
                end else begin
                    if (!hv_fsm_busy) begin
                        wrb_qp_number_c = c2d.c1d.qp_number;
                        c_rq_buf_ca     = p_rq_buf_ca;
                        c_last_req      = p_last_req;
                        c_rq_wrptr_db   = p_rq_wrptr_db;
                    end else begin
                        wrb_qp_number_c = qp_wrb_dat_w.wrb_qp_number;
                        c_rq_buf_ca     = qp_wrb_dat_w.rq_buf_ca;
                        c_last_req      = qp_wrb_dat_w.last_req;
                        c_rq_wrptr_db   = qp_wrb_dat_w.rq_wrptr_db;
                    end
                end
            end
            2'b01: begin  // qp信息在此写回Intiator寄存器组
                if (update_qp_info & write_qp_info) begin
                    if (!hv_fsm_busy) begin
                        wrb_qp_number_c = c2d.c1d.qp_number;
                        c_rq_buf_ca     = p_rq_buf_ca;
                        c_last_req      = p_last_req;
                        c_rq_wrptr_db   = p_rq_wrptr_db;
                    end else begin
                        wrb_qp_number_c = qp_wrb_dat_w.wrb_qp_number;
                        c_rq_buf_ca     = qp_wrb_dat_w.rq_buf_ca;
                        c_last_req      = qp_wrb_dat_w.last_req;
                        c_rq_wrptr_db   = qp_wrb_dat_w.rq_wrptr_db;
                    end
                end else begin
                    wrb_qp_number_c = wrb_qp_number_r;
                    c_rq_buf_ca     = r_rq_buf_ca;
                    c_last_req      = r_last_req;
                    c_rq_wrptr_db   = r_rq_wrptr_db;
                end
            end
            2'b11: begin
                if (write_qp_info) begin
                    wrb_qp_number_c = qp_wrb_dat_1.wrb_qp_number;
                    c_rq_buf_ca     = qp_wrb_dat_1.rq_buf_ca;
                    c_last_req      = qp_wrb_dat_1.last_req;
                    c_rq_wrptr_db   = qp_wrb_dat_1.rq_wrptr_db;
                end else begin
                    wrb_qp_number_c = wrb_qp_number_r;
                    c_rq_buf_ca     = r_rq_buf_ca;
                    c_last_req      = r_last_req;
                    c_rq_wrptr_db   = r_rq_wrptr_db;
                end
            end
            default: begin
                wrb_qp_number_c = wrb_qp_number_r;
                c_rq_buf_ca     = r_rq_buf_ca;
                c_last_req      = r_last_req;
                c_rq_wrptr_db   = r_rq_wrptr_db;
            end
        endcase
    end


    always @(posedge clk) if (set_rsp_qp_fatal) rsp_qp_fatal_idx_r <= rsp_qp_fatal_idx;

    always @(posedge clk)
        if (reset) rsp_qp_fatal_flag <= 1'b0;
        else if (set_rsp_qp_fatal || clr_qp_fatal_flag) rsp_qp_fatal_flag <= set_rsp_qp_fatal;


    // 当valid和ready都为高时，捕获致命QP索引
    always @(posedge clk)
        if (i_qp_fatal_tx_engine_vld && qp_fatal_tx_engine_rdy) begin  // TX 引擎检测到致命错误
            fatal_qp_idx <= i_qp_fatal_tx_engine;
        end else if (i_qp_fatal_ack_mgr_vld && qp_fatal_ack_mgr_rdy) begin  // ACK 管理器检测到致命错误
            fatal_qp_idx <= i_qp_fatal_ack_mgr;
        end else if (!ext_fatal_case_flag) begin
            fatal_qp_idx <= c3d.c2d.c1d.qp_number;
        end

    always @(*) begin
        qp_fatal_tx_engine_vld = i_qp_fatal_tx_engine_vld;
        qp_fatal_tx_engine     = i_qp_fatal_tx_engine;
        qp_fatal_ack_mgr_vld   = i_qp_fatal_ack_mgr_vld;
        qp_fatal_ack_mgr       = i_qp_fatal_ack_mgr;
    end

    assign o_qp_fatal_tx_engine_rdy = qp_fatal_tx_engine_rdy;
    assign o_qp_fatal_ack_mgr_rdy   = qp_fatal_ack_mgr_rdy;

    // qp_fatal_tx_engine_vld: TX 引擎检测到致命错误
    // qp_fatal_ack_mgr_vld: ACK 管理器检测到致命错误
    assign async_fatal_asserted     = i_qp_fatal_tx_engine_vld | i_qp_fatal_ack_mgr_vld;


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
            pkt_src_addr_offset  <= 'd0;
            actual_pkt_len       <= 'd0;
            pkt_fcs_err          <= 1'b0;
            ipv4_hdr_chksum_fail <= 1'b0;
            pkt_crc_err          <= 1'b0;
        end else if (rd_req_pkt_desc_fifo && !(hv_fsm_busy | async_fatal_asserted)) begin
            pkt_src_addr_offset  <= req_pkt_desc_fifo_dout[16+:C_REQ_BUF_ADDR_WIDTH];  // 包描述符中的 缓冲区偏移 pkt_buf_offset 字段
            actual_pkt_len       <= req_pkt_desc_fifo_dout[15:3];  // 包长度
            pkt_fcs_err          <= req_pkt_desc_fifo_dout[2];  // FCS error
            ipv4_hdr_chksum_fail <= req_pkt_desc_fifo_dout[1];  // IPv4 Header checksum error
            pkt_crc_err          <= req_pkt_desc_fifo_dout[0];  // ICRC error
        end else if (data_missed == 1'b1 && hdr_ftchg_fsm_ns == 1'b0 && !(hv_fsm_busy | async_fatal_asserted)) begin  // 使用备份数据恢复
            // 使用req_pkt_desc_fifo_dout_hold恢复数据
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

    // 保存上一拍数据，头部数据需要两拍获取
    always @(posedge clk) if (req_hdr_buf_enb) prev_req_hdr_buf_doutb <= req_hdr_buf_doutb;

    always @(posedge clk)
        if (hdr_ftchg_fsm_cs == 1'b0 && req_hdr_buf_enb == 1'b1) req_hdr_buf_enb_r <= 1'b1;
        else req_hdr_buf_enb_r <= 1'b0;

    // 拼接两拍数据：{本拍数据，上一拍数据}，形成完整头部信息，同时 hv_data_vld 拉高
    assign hdr_data = {req_hdr_buf_doutb, prev_req_hdr_buf_doutb};

    always @(posedge clk)
        if (reset) pkt_id <= 32'd0;
        else pkt_id <= 32'd0;

    assign bth_opcode_mem        = req_hdr_qpid_buf_doutb[31:24];  // 从QPID缓冲区获取的BTH操作码

    // 读取QP配置使能--当“头部有效“且qp“没有致命错误处理“时，允许读取QP配置
    assign read_qp_conf          = hv_data_vld & !(qp_fatal_tx_engine_vld | qp_fatal_ack_mgr_vld);
    assign rq_buf_ca_en          = read_qp_conf | rq_buf_ca_we;
    assign last_req_en           = read_qp_conf | last_req_we;
    assign rq_wrptr_db_en        = read_qp_conf | rq_wrptr_db_we;

    // =============================== 协议头解析 ===============================
    // 以太网类型提取
    assign ether_type            = {hdr_data[12*8+:8], hdr_data[13*8+:8]};
    // BTH字段提取
    assign ipv4_bth_qp_dest_addr = {hdr_data[47*8+:8], hdr_data[48*8+:8], hdr_data[49*8+:8]} >= 24'd1 ? {hdr_data[47*8+:8], hdr_data[48*8+:8], hdr_data[49*8+:8]} - 1 : 24'd0;
    assign ipv6_bth_qp_dest_addr = {hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]} >= 24'd1 ? {hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]} - 1 : 24'd0;
    // IP版本判断
    assign ip_ver                = (ether_type == 16'h0800) ? IPv4 : IPv6;  // 0: IPv4, 1: IPv6

    // qp信息写回Initiator寄存器组
    assign o_rq_buf_ca           = r_rq_buf_ca;
    assign o_last_req            = r_last_req;
    assign o_rq_wrptr_db         = r_rq_wrptr_db;

    // 时序对齐
    always @(posedge clk) if (req_hdr_buf_enb_r) bth_qp_dest_addr <= req_hdr_qpid_buf_doutb[23:0];  // 从QPID缓冲区获取的QP ID（均指目的QP ID）
    always @(posedge clk) if (req_hdr_buf_enb_r) bth_opcode <= bth_opcode_mem;  // bth操作码

    always @(*) begin
        // QP/RQ 配置用于 MAD、fatal 和 RQ DB_UPDATE 路径
        o_qp_conf_idx         = read_qp_conf ? bth_qp_dest_addr[C_QP_INDX_WIDTH-1:0] : wrb_qp_number_r;
        qp_conf_en            = read_qp_conf;
        rq_buf_ba_en          = read_qp_conf;
        rq_wrptr_db_addr_en   = read_qp_conf;
        q_depth_en            = read_qp_conf;
        rq_ci_db_en           = read_qp_conf;
        remote_qp_mac_addr_en = read_qp_conf;
        remote_qp_ip_addr_en  = read_qp_conf;
    end

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
    assign ipv4_flg_chk_fail = (hdr_data[(20*8+5)+:3] != 3'h2) ? 1'b1 : 1'b0;
    assign ipv4_frag_offset_chk_fail = ({hdr_data[(20*8)+:5], hdr_data[(21*8)+:8]} != 13'h0) ? 1'b1 : 1'b0;
    assign ipv4_prot_chk_fail = (hdr_data[(23*8)+:8] != 8'h11) ? 1'b1 : 1'b0;
    assign ipv4_dest_addr_chk_fail = (i_ipv4_node_addr != {hdr_data[(30*8)+:8], hdr_data[(31*8)+:8], hdr_data[(32*8)+:8], hdr_data[(33*8)+:8]}) ? 1'b1 : 1'b0;
    assign ipv4_total_len = {hdr_data[16*8+:8], hdr_data[17*8+:8]};
    assign ipv4_pyld_len = ipv4_total_len - 16'd20;
    assign ipv4_length_chk_fail = ((ipv4_total_len >= 16'd44) && (ipv4_total_len <= 16'd4160)) ? 1'b0 : 1'b1;

    //////////// IPv6头验证 //////////////
    assign ipv6_ver_chk_fail = (hdr_data[(14*8+4)+:4] != 4'h6) ? 1'b1 : 1'b0;
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
    assign ipv4_bth_psn = {hdr_data[51*8+:8], hdr_data[52*8+:8], hdr_data[53*8+:8]};
    assign ipv6_bth_psn = {hdr_data[71*8+:8], hdr_data[72*8+:8], hdr_data[73*8+:8]};

    // IPv4/v6 选择
    assign bth_tver = (ip_ver == IPv4) ? ipv4_bth_tver : ipv6_bth_tver;
    assign bth_expl_ack = (ip_ver == IPv4) ? ipv4_bth_expl_ack : ipv6_bth_expl_ack;
    assign bth_psn = (ip_ver == IPv4) ? ipv4_bth_psn : ipv6_bth_psn;

    assign ip_length_chk_fail = (ip_ver == IPv4) ? ipv4_length_chk_fail : ipv6_length_chk_fail;
    assign ip_dest_addr_chk_fail = (ip_ver == IPv4) ? ipv4_dest_addr_chk_fail : ipv6_dest_addr_chk_fail;
    assign ip_ver_chk_fail = (ip_ver == IPv4) ? ipv4_ver_chk_fail : ipv6_ver_chk_fail;

    assign bth_tver_chk_fail = (bth_tver != 4'd0) ? 1'b1 : 1'b0;

    // 内部 qpid 从 0 开始，内部 qpid=0 对应软件 QP1。
    // 若 i_num_qp_enabled=N，则允许内部 qpid 范围为 0 到 N-1；
    // 不支持离散启用。
    assign qp_conf_min = 24'd0;
    assign qp_conf_max = {{(23 - C_QP_INDX_WIDTH) {1'b0}}, i_num_qp_enabled};
    assign dest_qp_chk_fail = (bth_qp_dest_addr >= qp_conf_max) ? 1'b1 : 1'b0;

    // 13'd58 = 14 (Ethernet header) + 20 (IPv4 header) + 8 (UDP header) + 16 (BTH header) + 4 (ICRC)
    assign bth_pyld_len_base = (ip_ver == IPv4) ? (actual_pkt_len - 13'd58) : (actual_pkt_len - 13'd78);


    // 当hv_data_vld拉高时，检查结果才正确(时序正确)，接着保存正确的检查结果
    always @(posedge clk) begin
        if (hv_data_vld) begin
            // 保存 “基础包合法性校验“ 结果
            dest_qp_chk_fail_r <= dest_qp_chk_fail;  // 不在qp范围内
            bth_tver_chk_fail_r <= bth_tver_chk_fail;  // BTH TVer
            pkt_len_chk_fail_r <= pkt_len_chk_fail;
            udp_length_chk_fail_r <= udp_length_chk_fail;
            ip_length_chk_fail_r <= ip_length_chk_fail;
            ipv4_hdr_chksum_fail_r <= (ip_ver == IPv4) ? ipv4_hdr_chksum_fail : 1'b0;
            ip_dest_addr_chk_fail_r <= ip_dest_addr_chk_fail;  // 目的 IP 是否等于 RNIC 配置的本地 IP
            ipv4_frag_offset_chk_fail_r <= (ip_ver == IPv4) ? ipv4_frag_offset_chk_fail : 1'b0;
            ipv4_flg_chk_fail_r <= (ip_ver == IPv4) ? ipv4_flg_chk_fail : 1'b0;
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

    assign req_pkt_id = c3d.c2d.c1d.pkt_id;
    assign req_buf_addr_offset = c3d.c2d.c1d.pkt_src_addr_offset;
    assign req_qpid = c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
    assign req_pkt_len = c3d.req_pkt_len;

    assign bth_pyld_len = bth_pyld_len_base;

    assign ipv4_src_addr_chk_fail = (i_remote_qp_ip_addr[31:0] != c1d.ipv4_src_addr) ? 1'b1 : 1'b0;
    assign ipv6_src_addr_chk_fail = (i_remote_qp_ip_addr != c1d.ipv6_src_addr) ? 1'b1 : 1'b0;
    assign ip_src_addr_chk_fail_w = (ip_ver_r == IPv4) ? ipv4_src_addr_chk_fail : ipv6_src_addr_chk_fail;
    assign mac_src_addr_chk_fail_w = (i_remote_qp_mac_addr != mac_src_addr_r) ? 1'b1 : 1'b0;

    assign qp_rq_intr_en = c3d.c2d.qp_conf_data.qp_conf[2];
    assign rq_buf_ba = c3d.c2d.qp_conf_data.rq_buf_ba;
    assign rq_buf_ca = c3d.c2d.qp_conf_data.rq_buf_ca;
    assign rq_wrptr_db_addr = c3d.c2d.qp_conf_data.rq_wrptr_db_addr;
    assign q_depth = c3d.c2d.qp_conf_data.q_depth;
    assign rq_ci_db = c3d.c2d.qp_conf_data.rq_ci_db;
    assign last_req_psn = c3d.c2d.qp_conf_data.last_req[23:0];
    assign p_last_req_psn = c2d.qp_conf_data.last_req[23:0];
    assign ip_src_addr_chk_fail = c3d.c2d.ip_src_addr_chk_fail;
    assign mac_src_addr_chk_fail = c3d.c2d.mac_src_addr_chk_fail;
    assign src_addr_chk_fail = ip_src_addr_chk_fail | mac_src_addr_chk_fail;
    assign rq_wrptr_db = c3d.c2d.qp_conf_data.rq_wrptr_db;
    // qp_conf[0]：QP 启用 - 所有活动 QP 均应设置为 1。禁用的 QP 将无法接收或发送数据包。
    // qp_conf[6]：恢复中的 QP。该位需要在致命清除过程中设置。
    assign qp_state_fatal = (!(qp_conf_data.qp_conf[0]) || qp_fatal_combo_c1d || qp_recovery_combo_c1d || qp_conf_data.qp_conf[6]) ? 1'b1 : 1'b0;
    assign mad_pkt_len_err = c3d.c2d.c1d.mad_pkt_len_err;

    assign req_rq_pi_db = qp_wrb_dat_w.rq_wrptr_db;

    assign last_req_opcode = qp_conf_data.last_req[31:24];

    assign err_syndr = {
        pkt_fcs_err,  // bit 31
        pkt_crc_err,  // bit 30 
        1'b0,  // bit 29
        1'b0,  // bit 28
        4'b0,  // bit 24-27
        4'b0,  // bit 20-23
        1'b0,  // bit 19
        1'b0,  // bit 18
        1'b0,  // bit 17
        1'b0,  // bit 16
        qp_state_fatal,  // bit 15
        dest_qp_chk_fail_r,  // bit 14 
        bth_tver_chk_fail_r,  // bit 13
        pkt_len_chk_fail_r,  // bit 12
        udp_length_chk_fail_r,  // bit 11
        ip_length_chk_fail_r,  // bit 10
        ipv4_hdr_chksum_fail_r,  // bit 9
        ip_dest_addr_chk_fail_r,  // bit 8
        1'b0,  // bit 7
        ipv4_frag_offset_chk_fail_r,  // bit 6
        ipv4_flg_chk_fail_r,  // bit 5
        1'b0,  // bit 4 固定为 0
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
            bth_tver_chk_fail |
            pkt_len_chk_fail |
            udp_length_chk_fail |
            ip_length_chk_fail |
            (~ip_ver & ipv4_hdr_chksum_fail) |
            ip_dest_addr_chk_fail |
            (~ip_ver & ipv4_frag_offset_chk_fail) |
            (~ip_ver & ipv4_flg_chk_fail) |
            (~ip_ver & ipv4_ihl_chk_fail) |
            ip_ver_chk_fail |
            mac_dest_addr_chk_fail;

    assign pkt_vldn_err  = c3d.c2d.pkt_vldn_err;

    assign exp_req_psn   = last_req_psn + 'd1;  // 期望的PSN
    assign p_exp_req_psn = p_last_req_psn + 'd1;

    assign rq_buf_full   = c3d.rq_buf_full;
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP FATAL LUTRAM START //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    // init_start 时按 init_count 把全体 QP 清 0（初始化）。这里是为了同时服务两套读地址，因为 sdpram 本质是单读口，想同周期读两个不同地址，最简单可靠是复制两份内容

    // 这一套读 bth_qp_dest_addr（本包目标 QP），用于包有效性判断：pkt_vldn_err_1 里直接用 qp_fatal_combo || qp_recovery_combo
    // 存每个 QP 的 fatal 位，读出 qp_fatal_combo
    lrdma_common_sdp_lut_ram_wrapper #(
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
    lrdma_common_sdp_lut_ram_wrapper #(
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
    lrdma_common_sdp_lut_ram_wrapper #(
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

    lrdma_common_sdp_lut_ram_wrapper #(
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
        end
    end

    // for pipeline stage
    // qp_conf[31:16]：RQ 缓冲区大小（256B 的倍数）。这是请求中每个缓冲区元素的大小，而不是整个请求的大小。
    assign p_rq_buf_ca = (c2d.qp_conf_data.rq_wrptr_db == (c2d.qp_conf_data.q_depth - 'd1)) ? c2d.qp_conf_data.rq_buf_ba : (c2d.qp_conf_data.rq_buf_ca + c2d.qp_conf_data.qp_conf[31:16]);  //qp_rq_buf_sz

    assign p_last_req[31:24] = c2d.c1d.bth_opcode;
    assign p_last_req[23:0] = p_exp_req_psn;

    assign p_rq_wrptr_db = (c2d.qp_conf_data.rq_wrptr_db == c2d.qp_conf_data.q_depth) ? 16'd1 : (c2d.qp_conf_data.rq_wrptr_db + 'd1);


    assign qp_wrb_dat_w.rq_buf_ca = (c3d.c2d.qp_conf_data.rq_wrptr_db == (c3d.c2d.qp_conf_data.q_depth - 'd1)) ? c3d.c2d.qp_conf_data.rq_buf_ba : (c3d.c2d.qp_conf_data.rq_buf_ca + c3d.c2d.qp_conf_data.qp_conf[31:16]);  //qp_rq_buf_sz
    assign qp_wrb_dat_w.last_req[31:24] = c3d.c2d.c1d.bth_opcode;
    assign qp_wrb_dat_w.last_req[23:0] = exp_req_psn;
    assign qp_wrb_dat_w.rq_wrptr_db        = (c3d.c2d.qp_conf_data.rq_wrptr_db == c3d.c2d.qp_conf_data.q_depth) ? 16'd1 : (c3d.c2d.qp_conf_data.rq_wrptr_db + 'd1);  // +1，初始是从0开始的，接收完了才+1写回。回卷到1是为了满足环形条件。从 64 -> 1 算一次推进，判断有没有新包，就是看有没有推进

    assign o_qp_fatal_idx = fatal_qp_idx;

    always @(posedge clk) in_err_sts_code <= nxt_in_err_sts_code;

    // =========================================== 检测数据包类型 ===========================================
    always @(*) begin
        if ((bth_opcode[7:5] == UD) && (bth_qp_dest_addr == QP1) && (bth_opcode[4:0] == SEND_ONLY)) begin
            pkt_type = PKT_TYPE_MAD;
        end else begin
            pkt_type = PKT_TYPE_RSVD;
        end
    end

    always_ff @(posedge clk) begin
        c3d_qp_number_r     <= c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
        set_qp_fatal_r      <= set_qp_fatal;
        rsp_qp_fatal_flag_r <= rsp_qp_fatal_flag;
        clr_qp_fatal_flag_r <= clr_qp_fatal_flag;
        rsp_qp_fatal_idx_2r <= rsp_qp_fatal_idx_r;
    end

    // 基于信用的流量控制
    assign flow_credits = (i_flow_credits == 2'b00) ? 5'b1_1111 : {3'b000, i_flow_credits};

    always @(posedge clk)
        if (reset) tnfr_pend <= 1'b0;
        else tnfr_pend <= req_tnfr_vld & ~rdy_to_req_tnfr;  // 传输挂起标志，表示有数据传输请求但还未被接受处理的状态

    always @(posedge clk)
        if (reset) ext_fatal_case_flag <= 1'b0;
        else if (set_ext_fatal_case_flag || clr_ext_fatal_case_flag) ext_fatal_case_flag <= set_ext_fatal_case_flag;


    //===============================================================================================
    // Pipeline stage 1：基本头部验证
    //===============================================================================================
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

                    c1d.bth_psn <= bth_psn;
                    c1d.pkt_type <= pkt_type;
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
                    // 14 Ethernet + 20 IPv4 + 8 UDP + 12 BTH + 8 DETH（UD/MAD有） + 256 MAD 数据报最大长度 + 4 ICRC
                    c1d.mad_pkt_len_err <= (actual_pkt_len > ((ip_ver == IPv4) ? 13'd322 : 13'd342)) ? 1'b1 : 1'b0;  // MAD 报文长度上限检查
                    c1d.bth_pyld_len <= bth_pyld_len;
                end else begin
                    c1d <= '{default: 0};
                end  //hv_data_vld
            end  //!hv_fsm_busy
        end  //reset
    end  // always @ (posedge clk)



    // ========================= Pipeline stage 2：QP配置获取 ==========================
    // QP配置数据源选择 - 根据缓冲区状态选择最新数据
    // 时序与c1d同步
    always @(*) begin
        // 选择：从 写回缓冲区/Intiator寄存器组 中获取最新qp数据？
        // 如果缓冲区有该QP的更新数据，优先使用缓冲区数据，否则使用Intiator寄存器组数据

        // 接收队列(rq)写入地址
        qp_conf_data.rq_buf_ca          = (get_qp_info_from_buffer & qp_wrb_dat_r.rq_buf_ca_we) ? qp_wrb_dat_r.rq_buf_ca : i_rq_buf_ca;
        // 上一次请求的操作码和PSN
        qp_conf_data.last_req           = (get_qp_info_from_buffer & qp_wrb_dat_r.last_req_we) ? qp_wrb_dat_r.last_req : i_last_req;
        // RQ 写门铃值
        qp_conf_data.rq_wrptr_db        = (get_qp_info_from_buffer & qp_wrb_dat_r.rq_wrptr_db_we) ? qp_wrb_dat_r.rq_wrptr_db : i_rq_wrptr_db;

        // 其他只读字段直接使用寄存器值
        qp_conf_data.qp_conf            = i_qp_conf;
        qp_conf_data.rq_buf_ba          = i_rq_buf_ba;
        qp_conf_data.rq_wrptr_db_addr   = i_rq_wrptr_db_addr;
        qp_conf_data.q_depth            = i_q_depth;
        qp_conf_data.rq_ci_db           = i_rq_ci_db;
        qp_conf_data.remote_qp_mac_addr = i_remote_qp_mac_addr;
        qp_conf_data.remote_qp_ip_addr  = i_remote_qp_ip_addr;
    end

    // 执行各种验证
    always @(posedge clk) begin
        if (reset) begin
            c2d <= '{default: 0};
        end else begin
            if (qp_conf_vld) begin
                c2d.qp_conf_vld           <= qp_conf_vld;
                c2d.qp_conf_data          <= qp_conf_data;

                // MAD / 公共基础校验
                c2d.ip_src_addr_chk_fail  <= (c1d.ip_ver == IPv4) ? ipv4_src_addr_chk_fail : ipv6_src_addr_chk_fail;
                c2d.mac_src_addr_chk_fail <= (qp_conf_data.remote_qp_mac_addr != c1d.mac_src_addr) ? 1'b1 : 1'b0;
                c2d.pkt_vldn_err          <= pkt_vldn_err_1 | ~(qp_conf_data.qp_conf[0]) | qp_conf_data.qp_conf[6];
                c2d.error_syndrome        <= err_syndr;
            end

            if (!hv_fsm_busy) begin
                c2d.c1d <= c1d;
            end
        end
    end

    // ========================= Pipeline stage 3：错误检查和状态更新 ==========================
    always @(posedge clk) begin
        if (reset) begin
            c3d <= '{default: 0};
        end else begin
            if (!hv_fsm_busy) begin
                c3d.c2d         <= c2d;

                // MAD/公共路径仍可使用实际包长
                c3d.req_pkt_len <= c2d.c1d.actual_pkt_len;

                // MAD 用 RQ 满判断
                c3d.rq_buf_full <= ((c2d.qp_conf_data.rq_ci_db == c2d.qp_conf_data.rq_wrptr_db + 16'd1) || ((c2d.qp_conf_data.rq_ci_db + c2d.qp_conf_data.q_depth) == (c2d.qp_conf_data.rq_wrptr_db + 16'd1))) ? 1'b1 : 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            qp_conf_2_r        <= 0;
            qp_number_r        <= 0;
            rq_wrptr_db_r      <= 0;
            fatal_qp_idx_r     <= 0;
            rq_wrptr_db_addr_r <= 0;
        end else begin
            qp_conf_2_r        <= c3d.c2d.qp_conf_data.qp_conf[2];
            qp_number_r        <= c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
            rq_wrptr_db_r      <= qp_wrb_dat_w.rq_wrptr_db;
            fatal_qp_idx_r     <= c3d.c2d.c1d.qp_number[C_QP_INDX_WIDTH-1:0];
            rq_wrptr_db_addr_r <= rq_wrptr_db_addr;
        end
    end

    // 第 3 周期后采取行动（进入FSM）
    // 1. 外部fatal处理
    // 2. MAD处理
    // 3. RSVD/UNSUPPORTED公共错误处理
    always @(*) begin
        // default values
        write_qp_conf                   = 1'b0;
        hv_fsm_busy                     = 1'b0;
        hv_fsm_ns                       = hv_fsm_cs;
        nxt_in_err_sts_code             = in_err_sts_code;
        req_pkt_type                    = REQ_AXI_TNFR_TYPE_DISCARD;
        req_pkt_ip_ver                  = IPv6;
        req_dest_addr                   = {(C_M_AXI_ADDR_WIDTH) {1'b0}};
        req_imm_data                    = 44'd0;
        req_tnfr_vld                    = 1'b0;
        set_qp_fatal                    = 1'b0;
        set_rq_full_flag                = 1'b0;
        clr_rq_full_flag                = 1'b0;
        clr_qp_fatal_flag               = 1'b0;
        qp_fatal_tx_engine_rdy          = 1'b0;
        qp_fatal_ack_mgr_rdy            = 1'b0;
        set_ext_fatal_case_flag         = 1'b0;
        clr_ext_fatal_case_flag         = 1'b0;
        rq_buf_ca_we_w                  = 1'b0;
        rq_wrptr_db_we_w                = 1'b0;
        last_req_we_w                   = 1'b0;
        waiting_for_rq_update           = 0;
        waiting_for_rq_update_on_qp_num = 0;

        case (hv_fsm_cs)
            HV_IDLE_STATE: begin
                nxt_in_err_sts_code = 5'd0;
                clr_qp_fatal_flag   = 1'b1;

                // 处理外部 TX 引擎 fatal
                if (qp_fatal_tx_engine_vld) begin
                    nxt_in_err_sts_code     = 5'b00101;
                    qp_fatal_tx_engine_rdy  = 1'b1;
                    hv_fsm_ns               = HV_ERR_STATE;
                    hv_fsm_busy             = 1'b1;
                    set_ext_fatal_case_flag = 1'b1;
                end  // 处理外部 ACK 管理器 fatal
                        else if (qp_fatal_ack_mgr_vld) begin
                    nxt_in_err_sts_code     = 5'b00110;
                    qp_fatal_ack_mgr_rdy    = 1'b1;
                    hv_fsm_ns               = HV_ERR_STATE;
                    hv_fsm_busy             = 1'b1;
                    set_ext_fatal_case_flag = 1'b1;
                end  // 有包头验证数据，进入处理状态
                        else if (c2d.c1d.hv_data_vld) begin
                    hv_fsm_ns = HV_HDRVAL_STATE;
                end
            end

            HV_HDRVAL_STATE: begin
                if (c3d.c2d.qp_conf_vld || tnfr_pend) begin
                    case (c3d.c2d.c1d.pkt_type)

                        PKT_TYPE_RSVD, PKT_TYPE_UNSUPPORTED: begin
                            if (i_err_buf_en) begin
                                req_pkt_type = REQ_AXI_TNFR_TYPE_ERROR;
                                req_imm_data = {12'h000, c3d.c2d.error_syndrome};
                                req_tnfr_vld = 1'b1;

                                if (!rdy_to_req_tnfr) begin
                                    hv_fsm_busy = 1'b1;
                                end else begin
                                    hv_fsm_ns = HV_IDLE_STATE;
                                end
                            end else begin
                                hv_fsm_ns = HV_IDLE_STATE;
                            end
                        end

                        // MAD 包处理
                        PKT_TYPE_MAD: begin
                            if (pkt_vldn_err || mad_pkt_len_err || rq_buf_full) begin
                                if (i_err_buf_en) begin
                                    req_pkt_type     = REQ_AXI_TNFR_TYPE_ERROR;
                                    req_imm_data     = {12'h000, c3d.c2d.error_syndrome};
                                    req_imm_data[19] = mad_pkt_len_err;
                                    req_imm_data[20] = rq_buf_full;
                                    req_tnfr_vld     = 1'b1;

                                    if (!rdy_to_req_tnfr) begin
                                        hv_fsm_busy = 1'b1;
                                    end else begin
                                        hv_fsm_ns = HV_IDLE_STATE;
                                    end
                                end else begin
                                    hv_fsm_ns = HV_IDLE_STATE;
                                end
                            end else begin
                                req_pkt_type     = REQ_AXI_TNFR_TYPE_MAD;
                                req_dest_addr    = {rq_buf_ca, 8'h00};
                                req_pkt_ip_ver   = c3d.c2d.c1d.ip_ver;

                                req_imm_data     = {1'b0, 1'b1, qp_rq_intr_en, rq_wrptr_db_addr[13:0], 11'd0, qp_wrb_dat_w.rq_wrptr_db};

                                req_tnfr_vld     = 1'b1;
                                rq_buf_ca_we_w   = 1'b1;
                                rq_wrptr_db_we_w = 1'b1;
                                last_req_we_w    = 1'b1;

                                if (!rdy_to_req_tnfr) begin
                                    hv_fsm_busy = 1'b1;
                                end else begin
                                    hv_fsm_busy = 1'b1;
                                    hv_fsm_ns   = HV_WRITE_STATE;
                                end
                            end
                        end
                    endcase
                end
            end

            // MAD 门铃写回状态
            HV_WRITE_STATE: begin
                req_pkt_type  = REQ_AXI_TNFR_TYPE_DB_UPDATE;
                req_dest_addr = rq_wrptr_db_addr_r;

                req_imm_data  = {2'b00, qp_conf_2_r, {(25 - C_QP_INDX_WIDTH) {1'b0}}, qp_number_r, rq_wrptr_db_r};

                if (!rdy_to_req_tnfr) begin
                    hv_fsm_busy = 1'b1;
                end else begin
                    req_tnfr_vld = 1'b1;
                    hv_fsm_ns    = HV_IDLE_STATE;
                end
            end

            // fatal 状态
            HV_ERR_STATE: begin
                req_pkt_type = REQ_AXI_TNFR_TYPE_IN_ERR_STS;

                req_imm_data = {4'h0, {(24 - C_QP_INDX_WIDTH) {1'b0}}, o_qp_fatal_idx, 11'd0, in_err_sts_code};

                if (!rdy_to_req_tnfr) begin
                    hv_fsm_busy = 1'b1;
                end else begin
                    req_tnfr_vld = 1'b1;

                    if (ext_fatal_case_flag) begin
                        clr_ext_fatal_case_flag = 1'b1;
                        hv_fsm_busy             = 1'b1;
                    end

                    set_qp_fatal = 1'b1;
                    hv_fsm_ns    = HV_IDLE_STATE;
                end
            end

            default: begin
                hv_fsm_ns = HV_IDLE_STATE;
            end

        endcase
    end

    always @(posedge clk)
        if (reset) hv_fsm_cs <= 2'd0;
        else hv_fsm_cs <= hv_fsm_ns;

    assign o_last_in_req_pkt_opcode = c1d.bth_opcode;
    assign o_last_in_req_pkt_qpid   = c1d.qp_number[C_QP_INDX_WIDTH-1:0];
    assign o_last_in_req_pkt_psn    = c1d.bth_psn[15:0];

endmodule
