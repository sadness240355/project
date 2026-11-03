`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_rx_resp_hdr_validator — RX 引擎响应包头部验证模块
//
// 【模块概述】
// 处理所有从远端返回的响应包（对本端 RDMA 请求的确认/否定回复）。
// 从描述符 FIFO 取出包信息，通过头部缓冲区连续读取两拍 512bit，拼成 1024bit 头部数据，
// 经多阶段状态机完成协议解析和决策。
//
// 【AETH Syndrome 解析】
// - bits[6:5]=00：ACK（正常确认）
// - bits[6:5]=01：不支持的 AETH 响应类型，按 fatal 类响应处理
// - bits[6:5]=11：NAK，低 5 位区分 PSN 序列错误/无效请求/远程访问错误
//
// 【HV FSM（6 状态）】
// - 状态 0 (IDLE)：从描述符 FIFO 读取，发起头部缓冲区读
// - 状态 1：等待 QP 配置数据（qp_conf_vld）
// - 状态 2：等待 PSN 查找结果（psn_lookup_vld），确定 unack 窗口边界
// - 状态 3 (PROCESS)：核心处理——按 syndrome 分类
//   · ACK：PSN 范围验证（is_in_range），有效则输出 acknack_vld，清除 NAK 标志
//   · 不支持的响应类型：按 fatal 类响应处理
//   · NAK-SEQ-ERR：触发重传（OSQ 非空且未恢复中）
//   · NAK-FATAL：直接触发 QP Fatal
// - 状态 4 (FATAL)：写入错误状态码
// - 状态 5：丢弃（校验失败包）
//
// 【PSN 范围验证】
// - is_x_less_y(x,y)：24-bit 环绕比较
// - is_in_range(lo,hi,val)：检查 ACK PSN 是否在 (stat_resp_psn, last_ob_req_psn] 内
//
// 【LUTRAM（5 个 lrdma_common_dual_wr_sdp_lut_ram_wrapper）】
// qp_fatal、qp_recovery、osq_empty、qp_retried、osq_nacked
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps

module lrdma_rx_resp_hdr_validator #(
    parameter C_M_AXI_ADDR_WIDTH       = 32,   // AXI master 地址位宽（本模块并不直接访问，仅供子端口透传）
    parameter C_QP_INDX_WIDTH          = 8,    // QP 索引位宽 = log2(C_NUM_QP)
    parameter C_NUM_QP                 = 256,  // 支持的最大 QP 数
    parameter PKT_DESC_FIFO_DATA_WIDTH = 25,   // rsp_pkt_desc_fifo 位宽：{9'b0, actual_pkt_len[12:0], fcs_err,ipv4_chksum_fail, crc_err}
    parameter C_HDR_BUF_ADDR_WIDTH     = 9     // rsp_hdr_buf 地址位宽（一次读取 512bit，两次拼出 1024bit 头部）
) (
    // System level interface
    input wire clk,
    input wire reset,

    // ------------------ rsp 包描述符 lrdma_common_fifo (rsp_pkt_desc_fifo)接口 ------------------
    input  wire [PKT_DESC_FIFO_DATA_WIDTH-1:0] rsp_pkt_desc_fifo_dout,
    input  wire                                rsp_pkt_desc_fifo_empty,
    output reg                                 rd_rsp_pkt_desc_fifo,

    // ------------------ rsp 包头缓冲区(rsp_hdr_buf)接口 ------------------
    output reg  [C_HDR_BUF_ADDR_WIDTH-1:0] rsp_hdr_buf_addrb,
    output reg                             rsp_hdr_buf_enb,
    input  wire [                   511:0] rsp_hdr_buf_doutb,

    // ------------------ LRDMA 寄存器接口 ------------------
    input wire [               47:0] i_mac_node_addr,
    input wire [               31:0] i_ipv4_node_addr,
    input wire [              127:0] i_ipv6_node_addr,
    input wire [C_QP_INDX_WIDTH:0] i_num_qp_enabled,  // 启用的QP数量

    output reg [15:0] o_acknack_pkt_cnt,  // 传入 ACK/NAK 响应包计数
    output reg [15:0] o_inc_nack_pkt_cnt, // 传入 NAK 数据包计数

    // 最后(上一次)收到的响应数据包的信息
    output wire [ 7:0] o_last_in_rsp_pkt_opcode,  // 最后收到的响应包的 操作码
    output wire [ 7:0] o_last_in_rsp_pkt_qpid,    // 最后收到的响应包的 Dest QP ID
    output wire [15:0] o_last_in_rsp_pkt_psn,     // 最后收到的响应包的 PSN

    // ------------------ QP 配置 ------------------
    input  wire [31:0] i_qp_conf,
    input  wire        qp_conf_vld,
    output reg         qp_conf_en,

    input  wire [6:0] i_stat_nak,   // STATQPi[22:16]
    output reg        stat_nak_en,
    output reg        stat_nak_we,
    output reg  [6:0] o_stat_nak,   // STATQPi[22:16]

    output reg [C_QP_INDX_WIDTH-1:0] o_qp_conf_idx,

    input  wire [23:0] i_stat_resp_psn,   // STATRESPSNi[23:0]: 最后一个已确认的响应 PSN
    output reg         stat_resp_psn_en,
    output reg         stat_resp_psn_we,
    output reg  [23:0] o_stat_resp_psn,

    input  wire [23:0] i_sq_psn,  // SQPSNi[23:0]: SQ PSN，下一个将发出的请求的 PSN
    output reg         sq_psn_en,

    input  wire [47:0] i_remote_qp_mac_addr,  // 远端 QP MAC 地址，用于校验入包源 MAC
    output reg         remote_qp_mac_addr_en,

    input  wire [127:0] i_remote_qp_ip_addr,  // 远端 QP IP 地址，用于校验入包源 IP
    output reg          remote_qp_ip_addr_en,

    input wire [31:0] i_qp_timeout,  // [10:8]: 最大重试次数；[4:0]: 超时值；其余位无功能定义

    // ------------------ QP Fatal 标志接口 ------------------
    input  wire                       i_qp_fatal_wr_en,
    input  wire [C_QP_INDX_WIDTH-1:0] i_qp_fatal_wr_idx,
    input  wire                       i_qp_fatal_wr_data,
    output wire [C_QP_INDX_WIDTH-1:0] o_qp_fatal_idx,
    output reg                        set_qp_fatal,

    input wire [C_QP_INDX_WIDTH-1:0] req_qp_fatal_idx,
    input wire                       set_req_qp_fatal,

    // QP 进入重传状态事件
    input wire                                 i_qp_retried_set_valid,
    input wire [          C_QP_INDX_WIDTH-1:0] i_qp_retried_set_id,
    // QP 退出重传状态事件
    input wire                                 i_qp_retried_reset_valid,
    input wire [          C_QP_INDX_WIDTH-1:0] i_qp_retried_reset_id,
    // OSQ 变空事件
    input wire                                 i_osq_empty_set_valid,
    input wire [          C_QP_INDX_WIDTH-1:0] i_osq_empty_set_id,
    // OSQ 变非空事件
    input wire                                 i_osq_empty_reset_valid,
    input wire [          C_QP_INDX_WIDTH-1:0] i_osq_empty_reset_id,
    input wire [(4*C_QP_INDX_WIDTH+4) - 1 : 0] i_osq_nacked,              // "该 QP 的 OSQ 处于 nacked 状态"的 set/reset 事件总线

    // 校验通过后，通过下列信号告诉 ACK 管理器“收到了什么类型的响应，PSN 是多少”
    output reg [C_QP_INDX_WIDTH-1:0] o_qp_acknack_idx,    // ACK/NACK 对应的 QP 索引
    output reg                       o_acknack_vld,
    output reg [               23:0] o_acknack_psn,       // ACK/NACK 的 PSN
    output reg [                7:0] o_acknack_syndr,     // AETH 的 Syndrome 字段
    output reg [               23:0] o_acknack_msn,       // AETH 的 MSN 字段（仅 ACKNOWLEDGE 有意义） 
    output reg [               31:0] o_qp_timeout_per_qp, // 该 QP 的超时配置（透传给 lrdma_ack_mgr_fsm 做定时器管理）

    // ------------------ lrdma_rx_req_axi_master 接口 ------------------
    output wire [31:0] rsp_pkt_id,
    output reg  [31:0] rsp_imm_data,
    output reg         req_tnfr_vld,
    input  wire        rdy_to_req_tnfr,

    input wire                       qp_under_recovery_wr_en,
    input wire [C_QP_INDX_WIDTH-1:0] qp_under_recovery_wr_idx,
    input wire                       qp_under_recovery_wr_data,

    input wire                       init_start,
    input wire [C_QP_INDX_WIDTH-1:0] init_count
);

    //--------------------------------------------------------------------------
    // 包类型编码（pkt_type 内部使用）
    //--------------------------------------------------------------------------
    localparam [1:0] PKT_TYPE_ACKNOWLEDGE_ACK = 2'd0;  // 正常 ACK
    localparam [1:0] PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR = 2'd1;  // NAK: PSN Sequence Error，需触发重传
    localparam [1:0] PKT_TYPE_ACKNOWLEDGE_FATAL = 2'd2;  // NAK: 其它 syndrome（invld req / remote acc err），进入 Fatal
    localparam [1:0] PKT_TYPE_UNSUPPORTED_DROP = 2'd3;  // 非 ACK/NAK 响应路径包，明确丢弃，避免 X 综合差异

    localparam IPv4 = 1'b0;
    localparam IPv6 = 1'b1;

    // BTH.Opcode[4:0]
    localparam [4:0] ACKNOWLEDGE = 5'b1_0001;  // RC ACK opcode

    // AETH.syndrome[6:5] 高两位编码（AETH bits 30:29 of AETH word）
    localparam [1:0] ACK = 2'b00;  // 正常 ACK
    localparam [1:0] NAK = 2'b11;  // NAK（低 5 位细分 syndrome）

    // AETH.syndrome[4:0] NAK 细分
    localparam [4:0] PSN_SEQ_ERR = 5'b0_0000;  // PSN 序列错误（触发 GBN 重传）
    localparam [4:0] INVLD_REQ = 5'b0_0001;  // 非法请求，Fatal
    localparam [4:0] REMOTE_ACC_ERR = 5'b0_0010;  // 远端访问错误，Fatal

    //  3'd0 IDLE       ：等待 hv_data_vld，启动 QP conf 读 / psn_lookup_en
    //  3'd1 WAIT_CONF  ：等待 qp_conf_vld；若此时 psn_lookup 已回→直接跳到 3'd3
    //  3'd2 WAIT_PSN   ：qp_conf 已回，等 psn_lookup_vld_r；期间统计 ACK/NAK 计数
    //  3'd3 PROCESS    ：核心决策；按 pkt_type 分派：
    //                     ACK      → 更新 stat_resp_psn，发 acknack_vld
    //                     不支持的响应类型 → 按 fatal 类响应处理
    //                     NAK SEQ  → 通告重传（后续由 ACK 管理器处理）
    //                     NAK FATAL→ 转 3'd4 记 fatal syndrome
    //  3'd4 FATAL      ：写入 in_err_sts_code + set_qp_fatal，等 rdy_to_req_tnfr
    //  3'd5 ERROR      ：err_syndr != 0 的校验失败包，丢弃并上报 err_syndr
    localparam [2:0] IDLE = 3'd0;
    localparam [2:0] WAIT_CONF = 3'd1;
    localparam [2:0] WAIT_PSN = 3'd2;
    localparam [2:0] PROCESS = 3'd3;
    localparam [2:0] FATAL = 3'd4;
    localparam [2:0] ERROR = 3'd5;

    reg                        psn_lookup_vld_r;
    reg                        hdr_ftchg_done;
    reg                        hdr_ftchg_done_r;
    reg                        hv_data_vld;  // 为高时，hdr_data_c 有效
    reg                        hdr_ftchg_done_r1;
    reg                        hv_data_vld_r;  // 为高时，hdr_data 有效
    reg                        bth_vld_mask;
    reg  [              511:0] prev_rsp_hdr_buf_doutb;
    reg  [             1023:0] hdr_data;  // hv_data_vld_r 为高时，hdr_data 有效
    wire [             1023:0] hdr_data_c;  // hv_data_vld 为高时，hdr_data_c 有效。比 hdr_data 提前一拍

    reg  [               31:0] pkt_id;
    reg  [               12:0] actual_pkt_len;
    reg                        pkt_fcs_err;
    reg                        ipv4_hdr_chksum_fail;
    reg                        pkt_crc_err;

    wire [               15:0] ether_type;
    wire                       ip_ver;
    wire [               15:0] ipv4_total_len;
    wire [               15:0] ipv4_pyld_len;
    wire [               15:0] ipv6_pyld_len;
    wire [               15:0] ipv4_udp_dest_port_addr;
    wire [               15:0] ipv6_udp_dest_port_addr;
    wire [               15:0] ipv4_udp_len;
    wire [               15:0] ipv6_udp_len;
    wire [               15:0] udp_dest_port_addr;
    wire [               15:0] ip_pyld_len;
    wire [               15:0] udp_len;
    wire [               15:0] total_pkt_len_advt;
    wire [                3:0] ipv4_bth_tver;
    wire [                3:0] ipv6_bth_tver;
    wire [                3:0] bth_tver;
    wire [               23:0] ipv4_bth_qp_dest_addr;
    wire [               23:0] ipv6_bth_qp_dest_addr;
    wire [               23:0] ipv4_bth_qp_dest_addr_c;
    wire [               23:0] ipv6_bth_qp_dest_addr_c;
    wire [               23:0] bth_qp_dest_addr_c;
    wire [               23:0] bth_qp_dest_addr;
    wire [                7:0] ipv4_bth_opcode;
    wire [                7:0] ipv6_bth_opcode;
    wire [                7:0] bth_opcode;
    wire [                1:0] ipv4_bth_pad_cnt;
    wire [                1:0] ipv6_bth_pad_cnt;
    wire [                1:0] bth_pad_cnt;
    wire [               23:0] ipv4_bth_psn;
    wire [               23:0] ipv6_bth_psn;
    wire [               23:0] bth_psn;
    wire [                7:0] ipv4_aeth_syndr;
    wire [                7:0] ipv6_aeth_syndr;
    wire [                7:0] aeth_syndr;
    wire [               23:0] ipv4_aeth_msn;
    wire [               23:0] ipv6_aeth_msn;
    wire [               23:0] aeth_msn;
    wire [               23:0] qp_conf_min;
    wire [               23:0] qp_conf_max;
    wire [               12:0] bth_pyld_len;  // BTH 负载长度(不含 BTH 头部的长度)，也不包含 ICRC，这是“最终”的有效载荷长度

    wire                       mac_dest_addr_chk_fail;
    wire                       mac_src_addr_chk_fail;
    wire                       ipv4_ver_chk_fail;
    wire                       ipv4_ihl_chk_fail;
    wire                       ipv4_flg_chk_fail;
    wire                       ipv4_frag_offset_chk_fail;
    wire                       ipv4_prot_chk_fail;
    wire                       ipv4_dest_addr_chk_fail;
    wire                       ipv4_length_chk_fail;
    wire                       ipv4_src_addr_chk_fail;
    wire                       ipv6_ver_chk_fail;
    wire                       ipv6_next_hdr_chk_fail;
    wire                       ipv6_dest_addr_chk_fail;
    wire                       ipv6_length_chk_fail;
    wire                       ipv6_src_addr_chk_fail;
    wire                       ip_src_addr_chk_fail;
    wire                       udp_dest_port_chk_fail;
    wire                       udp_length_chk_fail;
    wire                       pkt_len_chk_fail;
    wire                       bth_tver_chk_fail;
    wire                       dest_qp_chk_fail;  // 报头目的QP不在RNIC范围
    reg                        aeth_malformed;  // AETH 字段不合法

    reg                        mac_dest_addr_chk_fail_r;
    reg                        ip_ver_chk_fail_r;
    reg                        ipv4_ihl_chk_fail_r;
    reg                        ipv4_flg_chk_fail_r;
    reg                        ipv4_frag_offset_chk_fail_r;
    reg                        ip_dest_addr_chk_fail_r;
    reg                        ipv4_hdr_chksum_fail_r;
    reg                        ip_length_chk_fail_r;
    reg                        udp_length_chk_fail_r;
    reg                        pkt_len_chk_fail_r;
    reg                        bth_tver_chk_fail_r;
    reg                        dest_qp_chk_fail_r;
    reg                        ip_ver_r;
    reg  [               23:0] bth_psn_r;  // 响应包的 BTH.PSN（已从包头解析并寄存）
    reg  [                7:0] bth_opcode_r;
    reg  [               23:0] bth_qp_dest_addr_r;
    reg  [                7:0] aeth_syndr_r;
    reg  [               23:0] aeth_msn_r;
    reg  [               31:0] pkt_id_r;
    reg  [               12:0] actual_pkt_len_r;

    wire                       qp_state_fatal;
    reg  [               12:0] qp_pmtu_decoded;
    wire [               23:0] i_last_ob_req_psn;

    reg  [               12:0] qp_pmtu_decoded_r;
    reg                        qp_state_fatal_r;
    reg                        aeth_malformed_r;
    reg                        ip_src_addr_chk_fail_r;
    reg                        mac_src_addr_chk_fail_r;
    reg  [               23:0] stat_resp_psn_r;  // 该 QP 的 STATRESPSNi = 上一次成功接收/确认的响应 PSN
    reg  [               23:0] last_ob_req_psn_r;  // 最后一个已发出的请求的 PSN

    wire [               31:0] err_syndr;
    wire [               31:0] err_syndr_muxed;

    //--------------------------------------------------------------------------
    // unACK 窗口（[stat_resp_psn+1, last_ob_req_psn]）
    //  unack_psn_min / unack_psn_min_muxed ：窗口下界 = STATRESPSNi+1
    //  unack_psn_max / unack_psn_max_muxed ：窗口上界 = SQPSNi-1（最后一个已发请求的 PSN）
    //  is_rsp_in_unack_range               ：bth_psn 是否落在窗口内
    //  osq_invld_at_vld / is_rsp_in_unack_range_at_vld：psn_lookup_vld_r 节拍采样结果
    //--------------------------------------------------------------------------
    wire [               23:0] unack_psn_min_muxed;  // 最小尚未被确认的 PSN/下一个预期收到的 PSN（unack 窗口下界）
    wire [               23:0] unack_psn_max_muxed;  // 最后一个已发出的请求的 PSN（unack 窗口上界）
    reg  [               23:0] unack_psn_min;  // 下一个预期收到的 PSN
    wire [               23:0] unack_psn_max;

    //--------------------------------------------------------------------------
    // QP 状态组合信号
    //  qp_fatal_combo     ：该 QP 是否处于 fatal（来自 qp_fatal LUTRAM）
    //  qp_recovery_combo  ：该 QP 是否处于 under-recovery（重传恢复中）
    //  qp_state_fatal     ：综合判断 — QPCONFi[0]==0(未启用) / fatal / recovery
    //  osq_nacked_combo   ：该 QP 的 OSQ 是否已 NACK（配合 *_pending 解同拍竞争）
    //  i_osq_empty_combo  ：该 QP 的 OSQ 是否空
    //  i_osq_invld        ：OSQ 无效状态 = empty | nacked（ACK/NAK 处理时的短路条件）
    //  i_qp_retried_combo ：该 QP 是否已发起过重传（重传去重）
    //--------------------------------------------------------------------------
    wire                       i_osq_empty_combo;
    wire                       i_osq_invld;
    wire                       i_qp_retried_combo;
    wire                       qp_fatal_combo;
    wire                       osq_nacked_combo_int;
    reg                        osq_nacked_combo;
    wire                       qp_recovery_combo;
    reg                        qp_under_recovery_wr_en_r;

    wire [               23:0] stat_resp_psn_muxed;
    wire [               12:0] qp_pmtu_decoded_muxed;  // QPCONFi[10:8]: PMTU

    reg                        osq_invld_at_vld;
    wire                       osq_invld;  // 该 QP 的 OSQ 是否处于"无效"状态，=1 表示该 QP 要么没有未完成请求，要么正处于重传恢复中
    reg                        is_rsp_in_unack_range_at_vld;  // 收到的响应包的 PSN 是否落在 unack 窗口 [unack_psn_min_muxed, unack_psn_max_muxed] 内，即该响应的 PSN 属于已发出但尚未确认的范围
    wire                       is_rsp_in_unack_range_resolved;
    wire                       is_rsp_in_unack_range;
    reg                        qp_retried;  // "重传去重"标志。置 1 表示该 QP 已触发重传；收到正常响应后清零，恢复再次触发重传的能力。

    reg  [                1:0] pkt_type;

    reg  [               23:0] acknack_psn;
    reg  [                7:0] acknack_syndr;
    reg                        acknack_vld;
    reg                        incr_acknack_pkt_cnt;
    reg                        incr_inc_nack_pkt_cnt;

    reg  [                4:0] in_err_sts_code;  // 手册 FATAL Codes
    reg  [                4:0] nxt_in_err_sts_code;  // 手册 FATAL Codes

    reg  [                2:0] hv_fsm_cs;
    reg  [                2:0] hv_fsm_ns;

    reg                        hv_fsm_busy;

    reg  [C_QP_INDX_WIDTH-1:0] req_qp_fatal_idx_r;
    reg                        clr_qp_fatal_flag;
    reg                        req_qp_fatal_flag;

    reg                        buf_en_int;

    reg                        stat_nak_en_int;
    reg                        stat_nak_we_int;
    wire [                6:0] o_stat_nak_int;

    reg                        stat_resp_psn_en_int;
    reg                        stat_resp_psn_we_int;
    reg  [               23:0] o_stat_resp_psn_int;

    // 时序对齐，与Intiator交互
    always @(posedge clk) begin
        if (reset) begin
            o_qp_conf_idx         <= 0;
            qp_conf_en            <= 0;

            stat_nak_en           <= 0;
            stat_nak_we           <= 0;
            o_stat_nak            <= 0;

            stat_resp_psn_en      <= 0;
            stat_resp_psn_we      <= 0;
            o_stat_resp_psn       <= 0;

            sq_psn_en             <= 0;
            remote_qp_mac_addr_en <= 0;
            remote_qp_ip_addr_en  <= 0;

            bth_vld_mask          <= 0;
        end else begin
            if (hv_data_vld == 1'b1 && bth_vld_mask == 1'b0) begin
                o_qp_conf_idx <= bth_qp_dest_addr_c[C_QP_INDX_WIDTH-1:0];
            end

            if (qp_conf_en) bth_vld_mask <= 1'b1;
            else if (!hv_fsm_busy) bth_vld_mask <= 1'b0;

            qp_conf_en            <= buf_en_int;

            sq_psn_en             <= buf_en_int;
            remote_qp_mac_addr_en <= buf_en_int;
            remote_qp_ip_addr_en  <= buf_en_int;

            stat_nak_en           <= stat_nak_en_int;
            stat_nak_we           <= stat_nak_we_int;
            o_stat_nak            <= o_stat_nak_int;

            stat_resp_psn_en      <= stat_resp_psn_en_int;
            stat_resp_psn_we      <= stat_resp_psn_we_int;
            o_stat_resp_psn       <= o_stat_resp_psn_int;
        end
    end

    always @(posedge clk) if (set_req_qp_fatal) req_qp_fatal_idx_r <= req_qp_fatal_idx;

    always @(posedge clk)
        if (reset) req_qp_fatal_flag <= 1'b0;
        else if (set_req_qp_fatal || clr_qp_fatal_flag) req_qp_fatal_flag <= set_req_qp_fatal;

    //===============================
    // Utility functions
    //===============================
    // x < y ?
    function automatic is_x_less_y;  // 用于比较两个24位有符号数的函数，判断x是否小于y
        input [23:0] x;
        input [23:0] y;
        begin
            is_x_less_y = 1'b0;

            // x[23]为符号位，1表示负数，0表示正数
            if (x[23] && ((x < y) || ({1'b0, x[22:0]} >= y))) is_x_less_y = 1'b1;
            else if (!x[23] && (x < y) && ({1'b1, x[22:0]} >= y)) is_x_less_y = 1'b1;
        end
    endfunction

    // x > y ?
    function automatic is_x_grtr_y;  // 用于比较两个24位有符号数的函数，判断x是否大于y
        input [23:0] x;
        input [23:0] y;
        begin
            is_x_grtr_y = is_x_less_y(y, x);
        end
    endfunction

    // x <= y ?
    function automatic is_x_lessorequal_y;  // 用于比较两个24位有符号数的函数，判断x是否小于等于y
        input [23:0] x;
        input [23:0] y;
        begin
            is_x_lessorequal_y = !is_x_less_y(y, x);
        end
    endfunction

    // x >= y ?
    function automatic is_x_grtrorequal_y;  // 用于比较两个24位有符号数的函数，判断x是否大于等于y
        input [23:0] x;
        input [23:0] y;
        begin
            is_x_grtrorequal_y = !is_x_less_y(x, y);
        end
    endfunction

    // a 在 [y, x] ？，这里要求 y >= x
    function automatic is_in_range;
        input [23:0] x;
        input [23:0] y;
        input [23:0] a;
        begin
            is_in_range = 1'b0;
            if (is_x_lessorequal_y(x, y)) begin
                if ((x > y) && ((a >= x) || (a <= y))) is_in_range = 1'b1;
                else if (!(x > y) && (a >= x) && (a <= y)) is_in_range = 1'b1;
            end
        end
    endfunction
    //===============================


    //------------------------------------------------------------
    // psn_lookup_vld 延迟 psn_lookup_en 6 拍，用于其在正确节拍采样 unack PSN window 相关状态
    //------------------------------------------------------------
    reg  [5:0] psn_lookup_dly;
    wire       psn_lookup_en;
    wire       psn_lookup_vld;
    always @(posedge clk) begin
        if (reset) psn_lookup_dly <= 6'b0;
        else psn_lookup_dly <= {psn_lookup_dly[4:0], psn_lookup_en};
    end
    assign psn_lookup_en  = buf_en_int;
    assign psn_lookup_vld = psn_lookup_dly[5];

    //===============================
    // 头信息获取控制信号逻辑
    //===============================  
    always @(posedge clk) begin
        if (reset) begin
            hdr_ftchg_done         <= 1'b0;
            hdr_ftchg_done_r       <= 1'b0;
            rsp_hdr_buf_enb        <= 1'b1;
            rd_rsp_pkt_desc_fifo   <= 1'b0;
            rsp_hdr_buf_addrb      <= {(C_HDR_BUF_ADDR_WIDTH) {1'b0}};
            hdr_ftchg_done_r1      <= 1'b0;
            prev_rsp_hdr_buf_doutb <= rsp_hdr_buf_doutb;
            psn_lookup_vld_r       <= 1'b0;
        end else begin
            psn_lookup_vld_r <= psn_lookup_vld;

            if (rsp_pkt_desc_fifo_empty == 1'b0 && hv_fsm_busy == 1'b0 && hdr_ftchg_done == 1'b0) begin
                rsp_hdr_buf_enb        <= 1'b1;
                rd_rsp_pkt_desc_fifo   <= 1'b1;

                hdr_ftchg_done         <= 1'b1;
                hdr_ftchg_done_r       <= 1'b0;
                hdr_ftchg_done_r1      <= 1'b0;

                rsp_hdr_buf_addrb      <= rsp_hdr_buf_addrb + 'd1;
                prev_rsp_hdr_buf_doutb <= rsp_hdr_buf_doutb;
            end else if (hdr_ftchg_done == 1'b1 && hv_fsm_busy == 1'b0 && hdr_ftchg_done_r == 1'b0 && hdr_ftchg_done_r1 == 1'b0) begin
                rsp_hdr_buf_enb      <= 1'b0;
                rd_rsp_pkt_desc_fifo <= 1'b0;
                hdr_ftchg_done_r1    <= 1'b1;
            end else if (hdr_ftchg_done == 1'b1 && psn_lookup_vld_r == 1'b1 && hdr_ftchg_done_r == 1'b0) begin
                rsp_hdr_buf_enb      <= 1'b1;
                rd_rsp_pkt_desc_fifo <= 1'b0;
                hdr_ftchg_done       <= 1'b0;
                hdr_ftchg_done_r     <= 1'b1;
                rsp_hdr_buf_addrb    <= rsp_hdr_buf_addrb + 'd1;
                hdr_ftchg_done_r1    <= 1'b0;
            end else begin
                rd_rsp_pkt_desc_fifo <= 1'b0;
                hdr_ftchg_done_r1    <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (reset) hdr_data <= 'd0;
        else if (rd_rsp_pkt_desc_fifo == 1'b1) hdr_data[511:0] <= rsp_hdr_buf_doutb;
        else if (hdr_ftchg_done_r1 == 1'b1) hdr_data[1023:512] <= rsp_hdr_buf_doutb;
    end

    assign hdr_data_c              = {rsp_hdr_buf_doutb, hdr_data[511:0]};
    assign ipv4_bth_qp_dest_addr_c = {hdr_data_c[47*8+:8], hdr_data_c[48*8+:8], hdr_data_c[49*8+:8]} >= 24'd1 ? {hdr_data_c[47*8+:8], hdr_data_c[48*8+:8], hdr_data_c[49*8+:8]} - 24'd1 : 24'd0;
    assign ipv6_bth_qp_dest_addr_c = {hdr_data_c[67*8+:8], hdr_data_c[68*8+:8], hdr_data_c[69*8+:8]} >= 24'd1 ? {hdr_data_c[67*8+:8], hdr_data_c[68*8+:8], hdr_data_c[69*8+:8]} - 24'd1 : 24'd0;
    assign bth_qp_dest_addr_c      = (ip_ver == IPv4) ? ipv4_bth_qp_dest_addr_c : ipv6_bth_qp_dest_addr_c;

    always @(posedge clk) begin
        if (reset) hv_data_vld <= 1'b0;
        else if (!hv_fsm_busy) hv_data_vld <= rd_rsp_pkt_desc_fifo;
    end

    always @(posedge clk)
        if (reset) hv_data_vld_r <= 1'b0;
        else if (!hv_fsm_busy) hv_data_vld_r <= 1'b0;
        else if (hv_data_vld) hv_data_vld_r <= 1'b1;

    always @(posedge clk) begin
        if (rsp_pkt_desc_fifo_empty == 1'b0 && hv_fsm_busy == 1'b0 && hdr_ftchg_done == 1'b0) pkt_id <= 32'd0;
    end

    always @(posedge clk) begin
        if (reset) begin
            actual_pkt_len       <= 'd0;
            pkt_fcs_err          <= 1'b0;
            ipv4_hdr_chksum_fail <= 1'b0;
            pkt_crc_err          <= 1'b0;
        end else begin
            if (rsp_pkt_desc_fifo_empty == 1'b0 && hv_fsm_busy == 1'b0 && hdr_ftchg_done == 1'b0) begin
                actual_pkt_len       <= rsp_pkt_desc_fifo_dout[15:3];
                pkt_fcs_err          <= rsp_pkt_desc_fifo_dout[2];
                ipv4_hdr_chksum_fail <= rsp_pkt_desc_fifo_dout[1];
                pkt_crc_err          <= rsp_pkt_desc_fifo_dout[0];
            end
        end
    end

    // =============================== 

    //===============================
    // 头部有效性检查
    //=============================== 

    //////////// MAC header specific checks //////////////  
    // 包头目的MAC地址 != 本节点MAC地址
    assign mac_dest_addr_chk_fail = ({hdr_data[(0*8)+:8], hdr_data[(1*8)+:8], hdr_data[(2*8)+:8], hdr_data[(3*8)+:8], hdr_data[(4*8)+:8], hdr_data[(5*8)+:8]} != i_mac_node_addr) ? 1'b1 : 1'b0;

    // 以太网类型字段
    assign ether_type = {hdr_data[12*8+:8], hdr_data[13*8+:8]};
    assign ip_ver = (ether_type == 16'h0800) ? IPv4 : IPv6;

    // 包头源MAC地址 != 该QP的目的MAC地址
    assign mac_src_addr_chk_fail = ({hdr_data[(6*8)+:8], hdr_data[(7*8)+:8], hdr_data[(8*8)+:8], hdr_data[(9*8)+:8], hdr_data[(10*8)+:8], hdr_data[(11*8)+:8]} != i_remote_qp_mac_addr) ? 1'b1 : 1'b0;

    //////////// IPv4 header specific checks //////////////  
    assign ipv4_ver_chk_fail = (hdr_data[(14*8+4)+:4] != 4'h4) ? 1'b1 : 1'b0;
    assign ipv4_ihl_chk_fail = (hdr_data[(14*8)+:4] != 4'h5) ? 1'b1 : 1'b0;
    assign ipv4_flg_chk_fail = (hdr_data[(20*8+5)+:3] != 3'h2) ? 1'b1 : 1'b0;
    assign ipv4_frag_offset_chk_fail = ({hdr_data[(20*8)+:5], hdr_data[(21*8)+:8]} != 13'h0) ? 1'b1 : 1'b0;
    assign ipv4_prot_chk_fail = (hdr_data[(23*8)+:8] != 8'h11) ? 1'b1 : 1'b0;
    assign ipv4_dest_addr_chk_fail = (i_ipv4_node_addr != {hdr_data[(30*8)+:8], hdr_data[(31*8)+:8], hdr_data[(32*8)+:8], hdr_data[(33*8)+:8]}) ? 1'b1 : 1'b0;
    assign ipv4_total_len = {hdr_data[16*8+:8], hdr_data[17*8+:8]};
    assign ipv4_pyld_len = ipv4_total_len - 16'd20;
    assign ipv4_length_chk_fail = ((ipv4_total_len >= 16'd44) && (ipv4_total_len <= 16'd4144)) ? 1'b0 : 1'b1;
    assign ipv4_src_addr_chk_fail = (i_remote_qp_ip_addr[31:0] != {hdr_data[(26*8)+:8], hdr_data[(27*8)+:8], hdr_data[(28*8)+:8], hdr_data[(29*8)+:8]}) ? 1'b1 : 1'b0;

    //////////// IPv6 header specific checks //////////////
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
    assign ipv6_length_chk_fail = ((ipv6_pyld_len >= 16'd16) && (ipv6_pyld_len <= 16'd4124)) ? 1'b0 : 1'b1;
    assign ipv6_src_addr_chk_fail  = (i_remote_qp_ip_addr != {
                   hdr_data[(22*8)+:8], hdr_data[(23*8)+:8], hdr_data[(24*8)+:8],
                   hdr_data[(25*8)+:8], hdr_data[(26*8)+:8], hdr_data[(27*8)+:8],       
                   hdr_data[(28*8)+:8], hdr_data[(29*8)+:8], hdr_data[(30*8)+:8],
                   hdr_data[(31*8)+:8], hdr_data[(32*8)+:8], hdr_data[(33*8)+:8],
                   hdr_data[(34*8)+:8], hdr_data[(35*8)+:8], hdr_data[(36*8)+:8],
                   hdr_data[(37*8)+:8]}) ? 1'b1 : 1'b0;

    assign ip_src_addr_chk_fail = (ip_ver == IPv4) ? ipv4_src_addr_chk_fail : ipv6_src_addr_chk_fail;

    //////////// UDP header specific checks //////////////
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

    /////////// BTH header specific checks //////////////
    assign ipv4_bth_tver = hdr_data[43*8+:4];
    assign ipv6_bth_tver = hdr_data[63*8+:4];
    assign ipv4_bth_qp_dest_addr = {hdr_data[47*8+:8], hdr_data[48*8+:8], hdr_data[49*8+:8]} >= 24'd1 ? {hdr_data[47*8+:8], hdr_data[48*8+:8], hdr_data[49*8+:8]} - 24'd1 : 24'd0;
    assign ipv6_bth_qp_dest_addr = {hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]} >= 24'd1 ? {hdr_data[67*8+:8], hdr_data[68*8+:8], hdr_data[69*8+:8]} - 24'd1 : 24'd0;
    assign ipv4_bth_opcode = hdr_data[42*8+:8];
    assign ipv6_bth_opcode = hdr_data[62*8+:8];
    assign ipv4_bth_pad_cnt = hdr_data[(43*8+4)+:2];
    assign ipv6_bth_pad_cnt = hdr_data[(63*8+4)+:2];
    assign ipv4_bth_psn = {hdr_data[51*8+:8], hdr_data[52*8+:8], hdr_data[53*8+:8]};
    assign ipv6_bth_psn = {hdr_data[71*8+:8], hdr_data[72*8+:8], hdr_data[73*8+:8]};
    assign ipv4_aeth_syndr = hdr_data[54*8+:8];
    assign ipv6_aeth_syndr = hdr_data[74*8+:8];
    assign ipv4_aeth_msn = {hdr_data[55*8+:8], hdr_data[56*8+:8], hdr_data[57*8+:8]};
    assign ipv6_aeth_msn = {hdr_data[75*8+:8], hdr_data[76*8+:8], hdr_data[77*8+:8]};

    assign bth_tver = (ip_ver == IPv4) ? ipv4_bth_tver : ipv6_bth_tver;  // BTH TVer字段
    assign bth_qp_dest_addr = (ip_ver == IPv4) ? ipv4_bth_qp_dest_addr : ipv6_bth_qp_dest_addr;  // BTH Destination QP字段
    assign bth_opcode = (ip_ver == IPv4) ? ipv4_bth_opcode : ipv6_bth_opcode;  // BTH Opcode字段
    assign bth_pad_cnt = (ip_ver == IPv4) ? ipv4_bth_pad_cnt : ipv6_bth_pad_cnt;  // BTH Pad字段
    assign bth_psn = (ip_ver == IPv4) ? ipv4_bth_psn : ipv6_bth_psn;  // BTH PSN字段
    assign aeth_syndr = (ip_ver == IPv4) ? ipv4_aeth_syndr : ipv6_aeth_syndr;  // AETH Syndrome字段
    assign aeth_msn = (ip_ver == IPv4) ? ipv4_aeth_msn : ipv6_aeth_msn;  // AETH MSN字段

    assign bth_tver_chk_fail = (bth_tver != 4'd0) ? 1'b1 : 1'b0;

    // 13'd58 = 14 (Ethernet header) + 20 (IPv4 header) + 8 (UDP header) + 16 (BTH header) + 4 (ICRC)
    assign bth_pyld_len = (ip_ver == IPv4) ? (actual_pkt_len - 13'd58) : (actual_pkt_len - 13'd78);

    // AETH 格式检查，检查头部是否符合规范
    always @(*) begin
        aeth_malformed = 1'b0;
        // ACK包
        if (bth_opcode[4:0] == ACKNOWLEDGE) begin
            if (bth_pyld_len != 'd4) aeth_malformed = 1'b1;  // 不符合
            else if ((aeth_syndr[7]) || (aeth_syndr[6:5] == 2'b10)) aeth_malformed = 1'b1;
            else if ((aeth_syndr[6:5] == 2'b11) && (aeth_syndr[4:0] > 5'b0_0011)) aeth_malformed = 1'b1;
        end
    end

    // 检查目的QP号是否在 RNIC 已配置 QP号的有效范围内
    // 启用QP要顺序启用，不支持离散 QP
    // tip: 启用QP数量为3时，启用的QP就不能是QP4及以后的QP，只能是QP1、QP2、QP3
    // 且 QP 启用数最大为 C_NUM_QP - 1，因为qpid-1改为qpid的原因，索引0为空的
    assign qp_conf_min      = 24'd0;
    assign qp_conf_max      = {{(23 - C_QP_INDX_WIDTH) {1'b0}}, i_num_qp_enabled};
    assign dest_qp_chk_fail = (bth_qp_dest_addr >= qp_conf_max) ? 1'b1 : 1'b0;

    always @(posedge clk) begin
        if (hv_data_vld_r) begin
            mac_dest_addr_chk_fail_r    <= mac_dest_addr_chk_fail;
            ip_ver_chk_fail_r           <= (ip_ver == IPv4) ? ipv4_ver_chk_fail : ipv6_ver_chk_fail;
            ipv4_ihl_chk_fail_r         <= (ip_ver == IPv4) ? ipv4_ihl_chk_fail : 1'b0;
            ipv4_flg_chk_fail_r         <= (ip_ver == IPv4) ? ipv4_flg_chk_fail : 1'b0;
            ipv4_frag_offset_chk_fail_r <= (ip_ver == IPv4) ? ipv4_frag_offset_chk_fail : 1'b0;
            ip_dest_addr_chk_fail_r     <= (ip_ver == IPv4) ? ipv4_dest_addr_chk_fail : ipv6_dest_addr_chk_fail;
            ipv4_hdr_chksum_fail_r      <= (ip_ver == IPv4) ? ipv4_hdr_chksum_fail : 1'b0;
            ip_length_chk_fail_r        <= (ip_ver == IPv4) ? ipv4_length_chk_fail : ipv6_length_chk_fail;
            udp_length_chk_fail_r       <= udp_length_chk_fail;
            pkt_len_chk_fail_r          <= pkt_len_chk_fail;
            bth_tver_chk_fail_r         <= bth_tver_chk_fail;
            dest_qp_chk_fail_r          <= dest_qp_chk_fail;
            ip_ver_r                    <= ip_ver;
            bth_psn_r                   <= bth_psn;
            bth_opcode_r                <= bth_opcode;
            aeth_malformed_r            <= aeth_malformed;
            aeth_syndr_r                <= aeth_syndr;
            aeth_msn_r                  <= aeth_msn;
            pkt_id_r                    <= pkt_id;
            actual_pkt_len_r            <= actual_pkt_len;
        end
    end

    always @(posedge clk) begin
        if (reset) bth_qp_dest_addr_r <= 24'h00_0000;
        else if (hv_data_vld_r) bth_qp_dest_addr_r <= bth_qp_dest_addr;
    end

    assign qp_state_fatal = (!i_qp_conf[0] || qp_fatal_combo || qp_recovery_combo) ? 1'b1 : 1'b0;
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP FATAL LUTRAM START //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
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

    always @(posedge clk) qp_under_recovery_wr_en_r <= qp_under_recovery_wr_en;
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP FATAL LUTRAM END ////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////


    always @(*)
        case (i_qp_conf[10:8])
            3'b000:  qp_pmtu_decoded = 13'd0256;
            3'b001:  qp_pmtu_decoded = 13'd0512;
            3'b010:  qp_pmtu_decoded = 13'd1024;
            3'b011:  qp_pmtu_decoded = 13'd2048;
            3'b100:  qp_pmtu_decoded = 13'd4096;
            default: qp_pmtu_decoded = 13'd0256;
        endcase

    assign i_last_ob_req_psn = i_sq_psn - 'd1;

    always @(posedge clk)
        if (reset) begin
            qp_pmtu_decoded_r       <= 'd0;
            qp_state_fatal_r        <= 'd0;
            ip_src_addr_chk_fail_r  <= 'd0;
            mac_src_addr_chk_fail_r <= 'd0;
            stat_resp_psn_r         <= 'd0;
            last_ob_req_psn_r       <= 'd0;
            o_qp_timeout_per_qp     <= 'd0;
            unack_psn_min           <= 'd0;
        end else begin
            if (qp_conf_vld) begin
                qp_pmtu_decoded_r       <= qp_pmtu_decoded;
                qp_state_fatal_r        <= qp_state_fatal;
                ip_src_addr_chk_fail_r  <= ip_src_addr_chk_fail;
                mac_src_addr_chk_fail_r <= mac_src_addr_chk_fail;
                stat_resp_psn_r         <= i_stat_resp_psn;
                unack_psn_min           <= i_stat_resp_psn + 'd1;
                last_ob_req_psn_r       <= i_last_ob_req_psn;
                o_qp_timeout_per_qp     <= i_qp_timeout;
            end
        end

    // 错误标志信息结构体: 对应手册 Packet Validation Error Syndrome
    assign err_syndr = {
        pkt_fcs_err,  // bit 31
        pkt_crc_err,  // bit 30
        1'b0,  // bit 29
        mac_src_addr_chk_fail_r,  // bit 28
        ip_src_addr_chk_fail_r,  // bit 27
        3'b0,  // bit 24-26
        aeth_malformed_r,  // bit 23
        4'b0,  // bit 19-22
        1'b0,  // bit 18
        2'b0,  // bit 16-17
        qp_state_fatal_r,  // bit 15
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

    // qp_conf_vld can coincide with psn_lookup_vld_r.  Use the fresh QP-dependent
    // checks for the transition decision in that cycle; the registered version is
    // used after the QP response has already been captured.
    assign err_syndr_muxed = {
        pkt_fcs_err,  // bit 31
        pkt_crc_err,  // bit 30
        1'b0,  // bit 29
        qp_conf_vld ? mac_src_addr_chk_fail : mac_src_addr_chk_fail_r,  // bit 28
        qp_conf_vld ? ip_src_addr_chk_fail : ip_src_addr_chk_fail_r,  // bit 27
        3'b0,  // bit 24-26
        aeth_malformed_r,  // bit 23
        4'b0,  // bit 19-22
        1'b0,  // bit 18
        2'b0,  // bit 16-17
        qp_conf_vld ? qp_state_fatal : qp_state_fatal_r,  // bit 15
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
        1'b0,  // bit 4
        ipv4_ihl_chk_fail_r,  // bit 3
        ip_ver_chk_fail_r,  // bit 2
        1'b0,  // bit 1
        mac_dest_addr_chk_fail_r  // bit 0
    };

    // qp_conf_vld can arrive in the same cycle as psn_lookup_vld_r.  Use the
    // fresh QP response for the ACK/NAK window in that cycle; otherwise the
    // first response after a WQE can be compared against stale PSN bounds.
    assign stat_resp_psn_muxed = qp_conf_vld ? i_stat_resp_psn : stat_resp_psn_r;
    assign unack_psn_min_muxed = stat_resp_psn_muxed + 'd1;
    assign unack_psn_max_muxed = qp_conf_vld ? i_last_ob_req_psn : last_ob_req_psn_r;
    assign unack_psn_max = last_ob_req_psn_r;
    assign qp_pmtu_decoded_muxed = qp_pmtu_decoded_r;

    wire                         osq_nacked_set_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_set_id;
    wire                         osq_nacked_set_valid_pending;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_set_id_pending;
    wire                         osq_nacked_reset_valid;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_reset_id;
    wire                         osq_nacked_reset_valid_pending;
    wire [C_QP_INDX_WIDTH-1 : 0] osq_nacked_reset_id_pending;

    assign {osq_nacked_set_valid, osq_nacked_set_id, osq_nacked_set_valid_pending, osq_nacked_set_id_pending, osq_nacked_reset_valid, osq_nacked_reset_id, osq_nacked_reset_valid_pending, osq_nacked_reset_id_pending} = i_osq_nacked;

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_osq_nacked_1 (
        .core_clk   (clk),
        .core_rst   (reset),
        .wr_en      (init_start || osq_nacked_set_valid),
        .wr_addr    (init_start ? init_count : osq_nacked_set_id),
        .wr_data    (init_start ? 1'b0 : 1'b1),
        .wr_en_aux  (osq_nacked_reset_valid),
        .wr_addr_aux(osq_nacked_reset_id),
        .wr_data_aux(1'b0),
        .rd_en      (1'b1),
        .rd_addr    (bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]),
        .rd_data    (osq_nacked_combo_int)
    );
    always @(*) begin
        if (osq_nacked_set_valid_pending && osq_nacked_set_id_pending == bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]) begin
            osq_nacked_combo = 1'b1;
        end else if (osq_nacked_reset_valid_pending && osq_nacked_reset_id_pending == bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]) begin
            osq_nacked_combo = 1'b0;
        end else begin
            osq_nacked_combo = osq_nacked_combo_int;
        end
    end
    assign i_osq_invld = i_osq_empty_combo | osq_nacked_combo;

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// OSQ EMPTY LUTRAM START /////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    reg                        osq_empty_reset_r;
    reg  [C_QP_INDX_WIDTH-1:0] osq_empty_reset_id_r;
    reg                        osq_empty_set_r;
    reg  [C_QP_INDX_WIDTH-1:0] osq_empty_set_id_r;
    wire                       osq_empty_wr_en;
    wire [C_QP_INDX_WIDTH-1:0] osq_empty_wr_idx;
    wire                       osq_empty_wr_data;

    assign osq_empty_wr_en   = init_start || osq_empty_set_r || osq_empty_reset_r;
    assign osq_empty_wr_idx  = init_start ? init_count : (osq_empty_set_r ? osq_empty_set_id_r : osq_empty_reset_id_r);
    assign osq_empty_wr_data = (init_start || osq_empty_set_r) ? 1'b1 : 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            osq_empty_reset_r    <= 1'b0;
            osq_empty_reset_id_r <= 'd0;
            osq_empty_set_r      <= 1'b0;
            osq_empty_set_id_r   <= 'd0;
        end else begin
            osq_empty_set_r    <= i_osq_empty_set_valid;
            osq_empty_set_id_r <= i_osq_empty_set_id;
            if (~(osq_empty_reset_r && osq_empty_set_r)) begin
                osq_empty_reset_r    <= i_osq_empty_reset_valid;
                osq_empty_reset_id_r <= i_osq_empty_reset_id;
            end
        end
    end
    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (osq_empty_wr_en),
        .wr_addr (osq_empty_wr_idx),
        .wr_data (osq_empty_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]),
        .rd_data (i_osq_empty_combo)
    );

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// OSQ EMPTY LUTRAM END ///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////



    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP RETRIED LUTRAM START ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    // qp_retried 状态采样逻辑
    reg                        qp_retried_reset_r;
    reg  [C_QP_INDX_WIDTH-1:0] qp_retried_reset_id_r;
    reg                        qp_retried_set_r;
    reg  [C_QP_INDX_WIDTH-1:0] qp_retried_set_id_r;
    wire                       qp_retried_wr_en;
    wire [C_QP_INDX_WIDTH-1:0] qp_retried_wr_idx;
    wire                       qp_retried_wr_data;

    assign qp_retried_wr_en   = init_start || qp_retried_set_r || qp_retried_reset_r;
    assign qp_retried_wr_idx  = init_start ? init_count : (qp_retried_set_r ? qp_retried_set_id_r : qp_retried_reset_id_r);
    assign qp_retried_wr_data = qp_retried_set_r ? 1'b1 : 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            qp_retried_reset_r    <= 1'b0;
            qp_retried_reset_id_r <= 'd0;
            qp_retried_set_r      <= 1'b0;
            qp_retried_set_id_r   <= 'd0;
        end else begin
            qp_retried_set_r    <= i_qp_retried_set_valid;
            qp_retried_set_id_r <= i_qp_retried_set_id;
            if (~(qp_retried_reset_r && qp_retried_set_r)) begin
                qp_retried_reset_r    <= i_qp_retried_reset_valid;
                qp_retried_reset_id_r <= i_qp_retried_reset_id;
            end
        end
    end
    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(0)
    ) inst_qp_retried_lutram (
        .core_clk(clk),
        .core_rst(reset),
        .wr_en   (qp_retried_wr_en),
        .wr_addr (qp_retried_wr_idx),
        .wr_data (qp_retried_wr_data),
        .rd_en   (1'b1),
        .rd_addr (bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0]),
        .rd_data (i_qp_retried_combo)
    );
    ///////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// QP RETRIED LUTRAM END //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////



    always @(posedge clk)
        if (reset) begin
            osq_invld_at_vld             <= 1'b0;
            is_rsp_in_unack_range_at_vld <= 1'b0;
            qp_retried                   <= 1'b0;
        end else if (psn_lookup_vld_r) begin
            osq_invld_at_vld             <= i_osq_invld;
            // 范围：[stat_resp_psn + 1, sq_psn - 1]，即整个 unACK 窗口
            is_rsp_in_unack_range_at_vld <= is_in_range(unack_psn_min_muxed, unack_psn_max_muxed, bth_psn_r);

            // 采样 qp_retried 状态
            qp_retried                   <= i_qp_retried_combo;
        end


    assign osq_invld             = osq_invld_at_vld;

    // Re-evaluate the PSN window in PROCESS after QP status has settled.  The
    // lookup-valid cycle can coincide with qp_conf_vld, so the sampled result
    // may see the previous STAT_RESP_PSN/LAST_OB_REQ_PSN window.
    assign is_rsp_in_unack_range_resolved = is_in_range(unack_psn_min_muxed, unack_psn_max_muxed, bth_psn_r) & ~osq_invld;
    assign is_rsp_in_unack_range          = (is_rsp_in_unack_range_at_vld & ~osq_invld) |
                                            ((hv_fsm_cs == PROCESS) & is_rsp_in_unack_range_resolved);

    // 响应包类型检测
    always @(posedge clk)
        if (reset) begin
            pkt_type <= PKT_TYPE_UNSUPPORTED_DROP;
        end else if (hv_data_vld_r) begin
            case (bth_opcode[4:0])
                ACKNOWLEDGE:
                case (aeth_syndr[6:5])
                    // ACK
                    2'b00: pkt_type <= PKT_TYPE_ACKNOWLEDGE_ACK;

                    // RNR NAK
                    // 不支持该 AETH 响应类型，按 fatal 类响应处理
                    2'b01: pkt_type <= PKT_TYPE_ACKNOWLEDGE_FATAL;

                    // NAK
                    2'b11: pkt_type <= (aeth_syndr[4:0] == 5'd0) ? PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR : PKT_TYPE_ACKNOWLEDGE_FATAL;

                    // 不是上述任何一种
                    default: pkt_type <= PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR;
                endcase
                default: pkt_type <= PKT_TYPE_UNSUPPORTED_DROP;
            endcase
        end

    assign o_stat_nak_int = aeth_syndr_r[6:0];

    assign o_qp_fatal_idx = bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0];

    assign o_acknack_msn  = aeth_msn_r;
    always @(posedge clk)
        if (reset) o_acknack_vld <= 1'b0;
        else o_acknack_vld <= acknack_vld;

    always @(posedge clk)
        if (acknack_vld) begin
            o_acknack_psn    <= acknack_psn;
            o_acknack_syndr  <= acknack_syndr;
            o_qp_acknack_idx <= bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0];
        end

    // 响应侧 AXI 写主站无实例，rsp_pkt_id 直接使用 pkt_id_r

    assign rsp_pkt_id = pkt_id_r;

    always @(posedge clk) in_err_sts_code <= nxt_in_err_sts_code;

    //===============================
    // 头部验证 FSM
    //===============================  
    // 头部验证 FSM（6 态）
    //  3'd0 IDLE       ：等待 hv_data_vld，启动 QP conf 读 / psn_lookup_en
    //  3'd1 WAIT_CONF  ：等待 qp_conf_vld；若此时 psn_lookup 已回→直接跳到 3'd3
    //  3'd2 WAIT_PSN   ：qp_conf 已回，等 psn_lookup_vld_r；期间统计 ACK/NAK 计数
    //  3'd3 PROCESS    ：核心决策；按 pkt_type 分派：
    //                     ACK      → 更新 stat_resp_psn，发 acknack_vld
    //                     不支持的响应类型 → 按 fatal 类响应处理
    //                     NAK SEQ  → 通告重传（后续由 ACK 管理器处理）
    //                     NAK FATAL→ 转 3'd4 记 fatal syndrome
    //  3'd4 FATAL      ：写入 in_err_sts_code + set_qp_fatal，等 rdy_to_req_tnfr
    //  3'd5 ERROR      ：err_syndr != 0 的校验失败包，丢弃并上报 err_syndr
    //  3'd6/3'd7       ：未定义状态编码（default → IDLE 安全恢复）
    //--------------------------------------------------------------------------
    // 关键决策：is_rsp_in_unack_range（bth_psn 落在 unACK 窗口）+ !osq_invld
    //   窗口外或 OSQ 无效：丢弃（rsp_imm_data = 32'h2040_0000 表示 drop 指令）
    //==========================================================================
    always @(*) begin
        // default values
        hv_fsm_busy           = 1'b0;
        hv_fsm_ns             = hv_fsm_cs;
        nxt_in_err_sts_code   = in_err_sts_code;
        stat_nak_en_int       = 1'b0;
        stat_nak_we_int       = 1'b0;
        o_stat_resp_psn_int   = bth_psn_r;
        stat_resp_psn_en_int  = 1'b0;
        stat_resp_psn_we_int  = 1'b0;
        buf_en_int            = 1'b0;
        rsp_imm_data          = 32'd0;
        req_tnfr_vld          = 1'b0;
        acknack_psn           = bth_psn_r;
        acknack_syndr         = aeth_syndr_r;
        acknack_vld           = 1'b0;
        incr_acknack_pkt_cnt  = 1'b0;
        incr_inc_nack_pkt_cnt = 1'b0;
        set_qp_fatal          = 1'b0;
        clr_qp_fatal_flag     = 1'b0;

        case (hv_fsm_cs)
            IDLE: begin
                nxt_in_err_sts_code = 5'd0;

                clr_qp_fatal_flag   = 1'b1;

                if (hv_data_vld) begin
                    stat_nak_en_int      = 1'b1;
                    stat_resp_psn_en_int = 1'b1;
                    buf_en_int           = 1'b1;

                    hv_fsm_busy          = 1'b1;
                    hv_fsm_ns            = WAIT_CONF;
                end
            end  // 3'd0

            WAIT_CONF: begin
                hv_fsm_busy = 1'b1;
                if (qp_conf_vld) begin
                    if (psn_lookup_vld_r) begin
                        if (err_syndr_muxed != 32'd0) begin
                            hv_fsm_ns = ERROR;
                        end else begin
                            hv_fsm_ns = PROCESS;
                        end

                        if ((bth_opcode == ACKNOWLEDGE) && !aeth_malformed) incr_acknack_pkt_cnt = 1'b1;

                        if ((bth_opcode == ACKNOWLEDGE) && (aeth_syndr[6:5] == NAK) && !aeth_malformed) incr_inc_nack_pkt_cnt = 1'b1;
                    end else hv_fsm_ns = WAIT_PSN;
                end  // if (qp_conf_vld)        
            end  // 3'd1

            WAIT_PSN: begin
                hv_fsm_busy = 1'b1;
                if (psn_lookup_vld_r) begin
                    if (err_syndr != 32'd0) begin  // 有错误, 进入错误处理
                        hv_fsm_ns = ERROR;
                    end else begin  // 没有错误, 继续处理
                        hv_fsm_ns = PROCESS;
                    end

                    if ((bth_opcode == ACKNOWLEDGE) && !aeth_malformed) incr_acknack_pkt_cnt = 1'b1;  // ACK&NACK 包计数器增加

                    // aeth_syndr[6:5] == NAK 表示 NAK 响应
                    if ((bth_opcode == ACKNOWLEDGE) && (aeth_syndr[6:5] == NAK) && !aeth_malformed) incr_inc_nack_pkt_cnt = 1'b1;  // NAK 包计数器增加

                end
            end  // 3'd2

            PROCESS: begin  // err_syndr == 32'd0: 没有头验证错误
                case (pkt_type)

                    // ================ ACK ================
                    PKT_TYPE_ACKNOWLEDGE_ACK: begin
                        hv_fsm_ns = IDLE;

                        if (is_rsp_in_unack_range) begin
                            // 正常 ACK 确认路径
                            acknack_psn          = bth_psn_r;
                            acknack_syndr        = aeth_syndr_r;
                            acknack_vld          = 1'b1;

                            o_stat_resp_psn_int  = bth_psn_r;
                            stat_resp_psn_en_int = 1'b1;
                            stat_resp_psn_we_int = 1'b1;

                            stat_nak_en_int      = 1'b1;
                            stat_nak_we_int      = 1'b1;

                            rsp_imm_data         = 32'h2040_0000;
                        end else begin
                            // 不在 unack 窗口内，丢弃
                            rsp_imm_data = 32'h2040_0000;
                        end
                    end  // PKT_TYPE_ACKNOWLEDGE_ACK

                    // ================ NAK SEQ ERR ================
                    PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR: begin
                        hv_fsm_ns = IDLE;

                        if (is_rsp_in_unack_range) begin
                            acknack_psn          = bth_psn_r;
                            acknack_syndr        = aeth_syndr_r;
                            acknack_vld          = 1'b1;

                            o_stat_resp_psn_int  = bth_psn_r - 'd1;
                            stat_resp_psn_en_int = 1'b1;
                            stat_resp_psn_we_int = 1'b1;

                            stat_nak_en_int      = 1'b1;
                            stat_nak_we_int      = 1'b1;

                            rsp_imm_data         = 32'h2040_0000;
                        end else begin
                            rsp_imm_data = 32'h2040_0000;
                        end
                    end  // PKT_TYPE_ACKNOWLEDGE_NAK_SEQ_ERR

                    // ================ NAK Fatal ================
                    PKT_TYPE_ACKNOWLEDGE_FATAL: begin
                        if (is_rsp_in_unack_range) begin
                            // Fatal 错误
                            rsp_imm_data        = 32'h0040_0000;
                            nxt_in_err_sts_code = 5'b01_010;
                            hv_fsm_ns           = FATAL;
                        end else begin
                            rsp_imm_data = 32'h2040_0000;
                            hv_fsm_ns    = IDLE;
                        end
                    end  // PKT_TYPE_ACKNOWLEDGE_FATAL

                    default: begin
                        // 未知类型，丢弃
                        rsp_imm_data = 32'h2040_0000;
                        hv_fsm_ns    = IDLE;
                    end

                endcase  // case(pkt_type)
            end  // 3'd3

            FATAL: begin
                rsp_imm_data = {{(16 - C_QP_INDX_WIDTH) {1'b0}}, bth_qp_dest_addr_r[C_QP_INDX_WIDTH-1:0], 11'd0, in_err_sts_code};
                req_tnfr_vld = 1'b1;

                if (!rdy_to_req_tnfr) hv_fsm_busy = 1'b1;
                else begin
                    // 写入 stat_nak（AETH syndrome）
                    stat_nak_en_int      = 1'b1;
                    stat_nak_we_int      = 1'b1;

                    // 记录进入 Fatal 状态时的响应 PSN
                    o_stat_resp_psn_int  = bth_psn_r;
                    stat_resp_psn_en_int = 1'b1;
                    stat_resp_psn_we_int = 1'b1;

                    set_qp_fatal         = 1'b1;
                    hv_fsm_ns            = IDLE;
                end
            end  // 3'd4

            ERROR: begin  // err_syndr != 32'd0
                if (err_syndr != 32'd0) begin
                    rsp_imm_data = err_syndr;
                    req_tnfr_vld = 1'b1;
                    if (!rdy_to_req_tnfr) begin
                        hv_fsm_busy = 1'b1;
                    end else begin
                        hv_fsm_ns = IDLE;
                        if (aeth_malformed_r) begin
                            stat_nak_en_int = 1'b1;
                            stat_nak_we_int = 1'b1;
                        end
                    end
                end else begin  // err_syndr == 32'd0
                    hv_fsm_ns = IDLE;
                end  // err_syndr == 32'd0
            end  // 3'd5

            default: begin
                // 上板稳定性优先：非法状态恢复到 IDLE，避免综合把 X 当作 don't-care 后生成不可控路径。
                hv_fsm_ns = IDLE;
            end  // default      
        endcase  // case (hv_fsm_cs)

    end  // always @(*)  

    //===============================
    // Header Validation FSM
    //===============================  

    always @(posedge clk)
        if (reset) hv_fsm_cs <= IDLE;
        else hv_fsm_cs <= hv_fsm_ns;

    always @(posedge clk)
        if (reset) o_acknack_pkt_cnt <= 16'd0;
        else if (incr_acknack_pkt_cnt) o_acknack_pkt_cnt <= o_acknack_pkt_cnt + 'd1;

    always @(posedge clk)
        if (reset) o_inc_nack_pkt_cnt <= 16'd0;
        else if (incr_inc_nack_pkt_cnt) o_inc_nack_pkt_cnt <= o_inc_nack_pkt_cnt + 'd1;

    assign o_last_in_rsp_pkt_opcode = bth_opcode_r;
    assign o_last_in_rsp_pkt_qpid   = bth_qp_dest_addr_r[7:0];
    assign o_last_in_rsp_pkt_psn    = bth_psn_r[15:0];

endmodule
