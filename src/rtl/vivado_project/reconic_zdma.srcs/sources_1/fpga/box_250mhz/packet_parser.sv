//==============================================================================
// 数据包解析器 - 支持以太网/IPv4/IPv6/UDP/RoCEv2协议栈解析
//
// 架构: 近线速直通(cut-through)设计，2-deep skid buffer输入
//   - 2级输入缓冲(ibuf+ibuf2)实现100%输入吞吐率，连续帧不丢拍
//   - 非RETH/IPv4包: 1拍延迟(寄存器输出)
//   - IPv4 RETH包:   2拍延迟(第二拍提取r_key/dma_length后输出第一拍)
//   - IPv6包:        2拍延迟(BTH跨beat，第二拍提取PSN/扩展头)
//   - IPv6 RETH包:   2拍延迟(RETH完全在beat1)
//   - 元数据仅在输出第一拍有效(metadata_out_valid与第一拍m_axis_tvalid对齐)
//
// IPv4 字节布局 (beat 0 = 64字节):
//   Eth(14) + IPv4(20) + UDP(8) + BTH(12) = byte 0-53, 全在beat 0
//   RETH(16): byte 54-69, 跨beat (byte 62-63 rkey高16位在beat 0, byte 0-5在beat 1)
//
// IPv6 字节布局 (beat 0 + beat 1):
//   Eth(14) + IPv6(40) + UDP(8) = byte 0-61, 全在beat 0
//   BTH(12): byte 62-73, 仅byte 62-63在beat 0, byte 64-73在beat 1
//   RETH(16): byte 74-89, 全在beat 1
//   AETH/IETH(4): byte 74-77, 全在beat 1
//
// 输出元数据格式 (263位) - 与packet_classification.sv一致:
//  [262:231] index        - 数据包索引号
//  [230:199] ip_src       - IP源地址(IPv4=完整32位, IPv6=低32位)
//  [198:167] ip_dst       - IP目标地址(IPv4=完整32位, IPv6=低32位)
//  [166:151] udp_sport    - UDP源端口
//  [150:135] udp_dport    - UDP目标端口
//  [134:130] opcode       - RoCEv2 BTH操作码
//  [129:114] pktlen       - 数据包长度
//  [113:82 ] dma_length   - DMA传输长度
//  [81 :50 ] r_key        - 远程密钥
//  [49 :49 ] se           - BTH请求事件标志
//  [48 :25 ] psn          - BTH数据包序列号
//  [24 :1  ] msn          - AETH消息序列号
//  [0  :0  ] is_rdma      - RDMA数据包标识
//==============================================================================
`timescale 1ns / 1ps

module packet_parser #(
    parameter METADATA_WIDTH  = 263,
    parameter AXIS_DATA_WIDTH = 512,
    parameter AXIS_KEEP_WIDTH = AXIS_DATA_WIDTH / 8
) (
    input s_axis_aclk,
    input s_axis_aresetn,
    input s_axil_aclk,
    input s_axil_aresetn,

    input  [METADATA_WIDTH-1:0] user_metadata_in,
    input                       user_metadata_in_valid,
    output [METADATA_WIDTH-1:0] user_metadata_out,
    output                      user_metadata_out_valid,

    input  [12:0] s_axil_awaddr,
    input         s_axil_awvalid,
    output        s_axil_awready,
    input  [31:0] s_axil_wdata,
    input         s_axil_wvalid,
    output        s_axil_wready,
    output [ 1:0] s_axil_bresp,
    output        s_axil_bvalid,
    input         s_axil_bready,
    input  [12:0] s_axil_araddr,
    input         s_axil_arvalid,
    output        s_axil_arready,
    output [31:0] s_axil_rdata,
    output        s_axil_rvalid,
    input         s_axil_rready,
    output [ 1:0] s_axil_rresp,

    output [AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output [AXIS_KEEP_WIDTH-1:0] m_axis_tkeep,
    output                       m_axis_tvalid,
    input                        m_axis_tready,
    output                       m_axis_tlast,

    input  [AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input  [AXIS_KEEP_WIDTH-1:0] s_axis_tkeep,
    input                        s_axis_tvalid,
    input                        s_axis_tlast,
    output                       s_axis_tready
);

    //=============================
    // 协议常量
    //=============================
    localparam [15:0] ETH_IPV4 = 16'h0800;
    localparam [15:0] ETH_IPV6 = 16'h86DD;
    localparam [7:0] IPPROTO_UDP = 8'h11;
    localparam [15:0] ROCE_UDP_PORT = 16'd4791;

    // BTH操作码 - RETH类
    localparam [4:0] WRITE_FIRST = 5'h06;
    localparam [4:0] WRITE_ONLY = 5'h0a;
    localparam [4:0] WRITE_ONLY_IMM = 5'h0b;
    localparam [4:0] READ_REQ = 5'h0c;
    // BTH操作码 - AETH类
    localparam [4:0] READ_RSP_FIRST = 5'h0d;
    localparam [4:0] READ_RSP_LAST = 5'h0f;
    localparam [4:0] READ_RSP_ONLY = 5'h10;
    localparam [4:0] ACK = 5'h11;
    // BTH操作码 - IETH类
    localparam [4:0] SEND_LAST_INV = 5'h16;
    localparam [4:0] SEND_ONLY_INV = 5'h17;
    // BTH操作码 - 纯BTH类 (无RETH/AETH/IETH扩展头)
    localparam [4:0] SEND_FIRST = 5'h00;
    localparam [4:0] SEND_MIDDLE = 5'h01;
    localparam [4:0] SEND_LAST = 5'h02;
    localparam [4:0] SEND_ONLY = 5'h04;
    localparam [4:0] WRITE_MIDDLE = 5'h07;
    localparam [4:0] WRITE_LAST = 5'h08;
    localparam [4:0] READ_RSP_MIDDLE = 5'h0e;

    //=============================
    // 状态机定义 (3-bit, 6 states)
    //=============================
    localparam [2:0] S_IDLE = 3'd0;
    localparam [2:0] S_V4_RETH_WAIT = 3'd1;  // IPv4 RETH: 等beat1提取rkey_lo/dmalen
    localparam [2:0] S_V4_RETH_DRAIN = 3'd2;  // IPv4 RETH: 排出beat1
    localparam [2:0] S_V6_BTH_WAIT = 3'd3;  // IPv6: 等beat1提取BTH余部+扩展头
    localparam [2:0] S_V6_RETH_DRAIN = 3'd4;  // IPv6 RETH: 排出beat1
    localparam [2:0] S_BODY = 3'd5;  // 后续body beats透传

    reg [                2:0] state_r;

    //=============================
    // 2-deep 输入 skid buffer
    // ibuf: FSM消费的前端缓冲
    // ibuf2: ibuf满时的后备缓冲
    // 稳态: FSM每拍消费ibuf, ibuf2提升到ibuf, 新数据进ibuf2, 吞吐100%
    //=============================
    reg                       in_ready_r;
    reg [AXIS_DATA_WIDTH-1:0] ibuf_tdata_r;
    reg [AXIS_KEEP_WIDTH-1:0] ibuf_tkeep_r;
    reg                       ibuf_tlast_r;
    reg                       ibuf_valid_r;
    reg [ METADATA_WIDTH-1:0] ibuf_meta_r;
    reg                       ibuf_meta_valid_r;

    reg [AXIS_DATA_WIDTH-1:0] ibuf2_tdata_r;
    reg [AXIS_KEEP_WIDTH-1:0] ibuf2_tkeep_r;
    reg                       ibuf2_tlast_r;
    reg                       ibuf2_valid_r;
    reg [ METADATA_WIDTH-1:0] ibuf2_meta_r;
    reg                       ibuf2_meta_valid_r;

    //=============================
    // 输出寄存器
    //=============================
    reg [AXIS_DATA_WIDTH-1:0] out_tdata_r;
    reg [AXIS_KEEP_WIDTH-1:0] out_tkeep_r;
    reg                       out_tlast_r;
    reg                       out_valid_r;
    reg [ METADATA_WIDTH-1:0] out_meta_r;
    reg                       out_meta_valid_r;

    // 暂存缓存 (RETH第一拍 / IPv6 beat0 / RETH_DRAIN复用为beat1)
    reg [AXIS_DATA_WIDTH-1:0] hdr_buf_tdata_r;
    reg [AXIS_KEEP_WIDTH-1:0] hdr_buf_tkeep_r;
    reg                       hdr_buf_tlast_r;
    reg [ METADATA_WIDTH-1:0] hdr_buf_meta_r;
    reg [               15:0] hdr_buf_rkey_hi_r;  // IPv4 RETH: rkey高16位
    reg                       hdr_buf_is_reth_r;  // IPv6: beat0判定的is_reth
    reg                       hdr_buf_is_aeth_r;
    reg                       hdr_buf_is_ieth_r;

    //=============================
    // 中间计算变量 (module 级别, blocking)
    //=============================
    reg [               15:0] calc_eth_type;
    reg [                3:0] calc_ip_ver;
    reg [                3:0] calc_ip_ihl;
    reg [                7:0] calc_ip_proto;
    reg [                7:0] calc_ip6_nxt_hdr;
    reg [               31:0] calc_ip_src;
    reg [               31:0] calc_ip_dst;
    reg [15:0] calc_udp_sport, calc_udp_dport;
    reg [4:0] calc_opcode;
    reg       calc_se;
    reg [23:0] calc_psn, calc_aeth_msn;
    reg [31:0] calc_ieth_rkey;
    reg [15:0] calc_rkey_hi;
    reg calc_is_ipv4, calc_is_ipv6, calc_is_ip;
    reg calc_is_udp, calc_is_roce, calc_is_reth, calc_is_aeth, calc_is_ieth;
    reg [METADATA_WIDTH-1:0] calc_meta;
    reg [              15:0] calc_rkey_lo;
    reg [              31:0] calc_dmalen;
    reg [              31:0] calc_v6_reth_rkey;
    reg [              31:0] calc_v6_reth_dmalen;
    reg [              23:0] calc_v6_aeth_msn;
    reg [              31:0] calc_v6_ieth_rkey;
    reg [              23:0] calc_v6_psn;

    // blocking 跟踪
    reg                      nxt_ibuf_valid;
    reg                      nxt_ibuf2_valid;
    reg                      nxt_out_valid;
    reg                      ibuf_consumed;  // FSM本周期是否消费了ibuf

    always @(posedge s_axis_aclk) begin
        if (~s_axis_aresetn) begin
            state_r            <= S_IDLE;
            in_ready_r         <= 1'b0;
            ibuf_valid_r       <= 1'b0;
            ibuf_meta_valid_r  <= 1'b0;
            ibuf2_valid_r      <= 1'b0;
            ibuf2_meta_valid_r <= 1'b0;
            out_valid_r        <= 1'b0;
            out_meta_valid_r   <= 1'b0;
        end else begin
            nxt_ibuf_valid  = ibuf_valid_r;
            nxt_ibuf2_valid = ibuf2_valid_r;
            nxt_out_valid   = out_valid_r;
            ibuf_consumed   = 1'b0;

            // ======== 输出被下游接受 ========
            if (out_valid_r && m_axis_tready) begin
                out_valid_r      <= 1'b0;
                out_meta_valid_r <= 1'b0;
                nxt_out_valid = 1'b0;
            end

            // ======== 状态机: 从 ibuf 处理 ========
            case (state_r)
                S_IDLE: begin
                    if (ibuf_valid_r && !nxt_out_valid && ibuf_meta_valid_r) begin
                        // --- 公共: EtherType ---
                        calc_eth_type    = {ibuf_tdata_r[103:96], ibuf_tdata_r[111:104]};

                        // --- IPv4 字段提取 (byte14起) ---
                        calc_ip_ver      = ibuf_tdata_r[119:116];
                        calc_ip_ihl      = ibuf_tdata_r[115:112];
                        calc_ip_proto    = ibuf_tdata_r[191:184];
                        calc_ip_src      = {ibuf_tdata_r[215:208], ibuf_tdata_r[223:216], ibuf_tdata_r[231:224], ibuf_tdata_r[239:232]};
                        calc_ip_dst      = {ibuf_tdata_r[247:240], ibuf_tdata_r[255:248], ibuf_tdata_r[263:256], ibuf_tdata_r[271:264]};

                        // --- IPv6 字段提取 (byte14起, 40B header) ---
                        // next_header at byte20
                        calc_ip6_nxt_hdr = ibuf_tdata_r[167:160];
                        // IPv6 src addr 低32位: byte34-37 (最后4字节 of 16B src)
                        // IPv6 src = byte22..37, 低32位 = byte34..37
                        // IPv6 dst 低32位: byte50-53 (最后4字节 of 16B dst)
                        // IPv6 dst = byte38..53, 低32位 = byte50..53

                        calc_is_ipv4     = (calc_eth_type == ETH_IPV4) && (calc_ip_ver == 4'd4) && (calc_ip_ihl == 4'd5);
                        calc_is_ipv6     = (calc_eth_type == ETH_IPV6) && (ibuf_tdata_r[119:116] == 4'd6);  // version nibble
                        calc_is_ip       = calc_is_ipv4 || calc_is_ipv6;

                        // UDP/RoCE 判定
                        if (calc_is_ipv4) begin
                            calc_is_udp    = (calc_ip_proto == IPPROTO_UDP);
                            // IPv4: UDP at byte34-41
                            calc_udp_sport = {ibuf_tdata_r[279:272], ibuf_tdata_r[287:280]};
                            calc_udp_dport = {ibuf_tdata_r[295:288], ibuf_tdata_r[303:296]};
                        end else if (calc_is_ipv6) begin
                            calc_is_udp    = (calc_ip6_nxt_hdr == IPPROTO_UDP);
                            // IPv6: UDP at byte54-61
                            calc_udp_sport = {ibuf_tdata_r[439:432], ibuf_tdata_r[447:440]};
                            calc_udp_dport = {ibuf_tdata_r[455:448], ibuf_tdata_r[463:456]};
                        end else begin
                            calc_is_udp    = 1'b0;
                            calc_udp_sport = 16'h0;
                            calc_udp_dport = 16'h0;
                        end

                        calc_is_roce = calc_is_udp && (calc_udp_dport == ROCE_UDP_PORT);

                        // BTH 提取 (IPv4/IPv6不同偏移)
                        if (calc_is_ipv4 && calc_is_roce) begin
                            // IPv4 BTH at byte42-53 (全在beat0)
                            calc_opcode    = ibuf_tdata_r[340:336];
                            calc_se        = ibuf_tdata_r[351];
                            calc_psn       = {ibuf_tdata_r[415:408], ibuf_tdata_r[423:416], ibuf_tdata_r[431:424]};
                            calc_aeth_msn  = {ibuf_tdata_r[447:440], ibuf_tdata_r[455:448], ibuf_tdata_r[463:456]};
                            calc_ieth_rkey = {ibuf_tdata_r[439:432], ibuf_tdata_r[447:440], ibuf_tdata_r[455:448], ibuf_tdata_r[463:456]};
                            calc_rkey_hi   = {ibuf_tdata_r[503:496], ibuf_tdata_r[511:504]};
                        end else if (calc_is_ipv6 && calc_is_roce) begin
                            // IPv6 BTH byte62-63在beat0: opcode + SE/padcnt/tver
                            calc_opcode    = ibuf_tdata_r[500:496];
                            calc_se        = ibuf_tdata_r[511];
                            // PSN/AETH/IETH在beat1, 此处设默认值
                            calc_psn       = 24'h0;
                            calc_aeth_msn  = 24'h0;
                            calc_ieth_rkey = 32'h0;
                            calc_rkey_hi   = 16'h0;
                        end else begin
                            calc_opcode    = 5'h0;
                            calc_se        = 1'b0;
                            calc_psn       = 24'h0;
                            calc_aeth_msn  = 24'h0;
                            calc_ieth_rkey = 32'h0;
                            calc_rkey_hi   = 16'h0;
                        end

                        calc_is_reth       = calc_is_roce && (calc_opcode == WRITE_FIRST || calc_opcode == WRITE_ONLY || calc_opcode == WRITE_ONLY_IMM || calc_opcode == READ_REQ);
                        calc_is_aeth       = calc_is_roce && (calc_opcode == READ_RSP_FIRST || calc_opcode == READ_RSP_LAST || calc_opcode == READ_RSP_ONLY || calc_opcode == ACK);
                        calc_is_ieth       = calc_is_roce && (calc_opcode == SEND_LAST_INV || calc_opcode == SEND_ONLY_INV);

                        // --- 构建部分元数据 (公共字段) ---
                        calc_meta[262:231] = ibuf_meta_r[262:231];  // index
                        calc_meta[129:114] = ibuf_meta_r[129:114];  // pktlen
                        calc_meta[166:151] = calc_is_udp ? calc_udp_sport : 16'h0;
                        calc_meta[150:135] = calc_is_udp ? calc_udp_dport : 16'h0;
                        calc_meta[134:130] = calc_is_roce ? calc_opcode : 5'h0;
                        calc_meta[0]       = calc_is_roce;

                        // --- IP 地址 ---
                        if (calc_is_ipv4) begin
                            calc_meta[230:199] = calc_ip_src;
                            calc_meta[198:167] = calc_ip_dst;
                        end else if (calc_is_ipv6) begin
                            // IPv6 src低32位: byte34-37
                            calc_meta[230:199] = {ibuf_tdata_r[279:272], ibuf_tdata_r[287:280], ibuf_tdata_r[295:288], ibuf_tdata_r[303:296]};
                            // IPv6 dst低32位: byte50-53
                            calc_meta[198:167] = {ibuf_tdata_r[407:400], ibuf_tdata_r[415:408], ibuf_tdata_r[423:416], ibuf_tdata_r[431:424]};
                        end else begin
                            calc_meta[230:199] = 32'h0;
                            calc_meta[198:167] = 32'h0;
                        end

                        // ===== 分支: IPv4 vs IPv6 =====
                        if (calc_is_ipv6 && calc_is_roce && !ibuf_tlast_r) begin
                            // IPv6 RoCE: BTH跨beat, 必须等beat1
                            // se在beat0 byte63可提取, 其余在beat1填充
                            calc_meta[49]     = calc_se;
                            calc_meta[48:25]  = 24'h0;
                            calc_meta[24:1]   = 24'h0;
                            calc_meta[113:82] = 32'h0;
                            calc_meta[81:50]  = 32'h0;
                            hdr_buf_tdata_r   <= ibuf_tdata_r;
                            hdr_buf_tkeep_r   <= ibuf_tkeep_r;
                            hdr_buf_tlast_r   <= ibuf_tlast_r;
                            hdr_buf_meta_r    <= calc_meta;
                            hdr_buf_is_reth_r <= calc_is_reth;
                            hdr_buf_is_aeth_r <= calc_is_aeth;
                            hdr_buf_is_ieth_r <= calc_is_ieth;
                            state_r           <= S_V6_BTH_WAIT;
                            ibuf_valid_r      <= 1'b0;
                            nxt_ibuf_valid = 1'b0;
                            ibuf_consumed  = 1'b1;
                        end else if (calc_is_ipv4 && calc_is_reth && !ibuf_tlast_r) begin
                            // IPv4 RETH: rkey跨beat, 等beat1
                            // 填充 IPv4 独有的字段 (beat0可提取的)
                            calc_meta[49]     = calc_se;
                            calc_meta[48:25]  = calc_psn;
                            calc_meta[24:1]   = 24'h0;
                            calc_meta[113:82] = 32'h0;
                            calc_meta[81:50]  = 32'h0;
                            hdr_buf_tdata_r   <= ibuf_tdata_r;
                            hdr_buf_tkeep_r   <= ibuf_tkeep_r;
                            hdr_buf_tlast_r   <= ibuf_tlast_r;
                            hdr_buf_meta_r    <= calc_meta;
                            hdr_buf_rkey_hi_r <= calc_rkey_hi;
                            state_r           <= S_V4_RETH_WAIT;
                            ibuf_valid_r      <= 1'b0;
                            nxt_ibuf_valid = 1'b0;
                            ibuf_consumed  = 1'b1;
                        end else begin
                            // IPv4 非RETH 或 非RoCE 或 IPv6单beat: 直接输出
                            calc_meta[49]     = calc_is_roce ? calc_se : 1'b0;
                            calc_meta[48:25]  = calc_is_roce ? calc_psn : 24'h0;
                            calc_meta[24:1]   = calc_is_aeth ? calc_aeth_msn : 24'h0;
                            calc_meta[113:82] = 32'h0;
                            calc_meta[81:50]  = calc_is_ieth ? calc_ieth_rkey : 32'h0;
                            out_tdata_r      <= ibuf_tdata_r;
                            out_tkeep_r      <= ibuf_tkeep_r;
                            out_tlast_r      <= ibuf_tlast_r;
                            out_meta_r       <= calc_meta;
                            out_valid_r      <= 1'b1;
                            out_meta_valid_r <= 1'b1;
                            nxt_out_valid = 1'b1;
                            state_r      <= ibuf_tlast_r ? S_IDLE : S_BODY;
                            ibuf_valid_r <= 1'b0;
                            nxt_ibuf_valid = 1'b0;
                            ibuf_consumed  = 1'b1;
                        end
                    end
                end

                S_V4_RETH_WAIT: begin
                    // IPv4 RETH: beat1到达, 提取rkey_lo + dmalen
                    if (ibuf_valid_r && !nxt_out_valid) begin
                        calc_rkey_lo = {ibuf_tdata_r[7:0], ibuf_tdata_r[15:8]};
                        calc_dmalen  = {ibuf_tdata_r[23:16], ibuf_tdata_r[31:24], ibuf_tdata_r[39:32], ibuf_tdata_r[47:40]};

                        out_tdata_r        <= hdr_buf_tdata_r;
                        out_tkeep_r        <= hdr_buf_tkeep_r;
                        out_tlast_r        <= hdr_buf_tlast_r;
                        out_meta_r         <= hdr_buf_meta_r;
                        out_meta_r[113:82] <= calc_dmalen;
                        out_meta_r[81:50]  <= {hdr_buf_rkey_hi_r, calc_rkey_lo};
                        out_valid_r        <= 1'b1;
                        out_meta_valid_r   <= 1'b1;
                        nxt_out_valid = 1'b1;

                        // 保存beat1到hdr_buf (DRAIN时输出)
                        hdr_buf_tdata_r <= ibuf_tdata_r;
                        hdr_buf_tkeep_r <= ibuf_tkeep_r;
                        hdr_buf_tlast_r <= ibuf_tlast_r;

                        state_r         <= S_V4_RETH_DRAIN;
                        ibuf_valid_r    <= 1'b0;
                        nxt_ibuf_valid = 1'b0;
                        ibuf_consumed  = 1'b1;
                    end
                end

                S_V4_RETH_DRAIN: begin
                    if (!nxt_out_valid) begin
                        out_tdata_r      <= hdr_buf_tdata_r;
                        out_tkeep_r      <= hdr_buf_tkeep_r;
                        out_tlast_r      <= hdr_buf_tlast_r;
                        out_valid_r      <= 1'b1;
                        out_meta_valid_r <= 1'b0;
                        nxt_out_valid = 1'b1;
                        state_r <= hdr_buf_tlast_r ? S_IDLE : S_BODY;
                    end
                end

                S_V6_BTH_WAIT: begin
                    // IPv6: beat1到达, 提取BTH余部 + 扩展头
                    if (ibuf_valid_r && !nxt_out_valid) begin
                        // BTH续: byte64-73 → beat1 offset byte0-9
                        // PSN: byte71-73 → beat1 byte7-9
                        calc_v6_psn         = {ibuf_tdata_r[63:56], ibuf_tdata_r[71:64], ibuf_tdata_r[79:72]};

                        // RETH: byte74-89 → beat1 byte10-25
                        // r_key: byte82-85 → beat1 byte18-21
                        calc_v6_reth_rkey   = {ibuf_tdata_r[151:144], ibuf_tdata_r[159:152], ibuf_tdata_r[167:160], ibuf_tdata_r[175:168]};
                        // dmalen: byte86-89 → beat1 byte22-25
                        calc_v6_reth_dmalen = {ibuf_tdata_r[183:176], ibuf_tdata_r[191:184], ibuf_tdata_r[199:192], ibuf_tdata_r[207:200]};

                        // AETH MSN: byte74 syndrome + byte75-77 msn
                        // msn: byte75-77 → beat1 byte11-13
                        calc_v6_aeth_msn    = {ibuf_tdata_r[95:88], ibuf_tdata_r[103:96], ibuf_tdata_r[111:104]};

                        // IETH r_key: byte74-77 → beat1 byte10-13
                        calc_v6_ieth_rkey   = {ibuf_tdata_r[87:80], ibuf_tdata_r[95:88], ibuf_tdata_r[103:96], ibuf_tdata_r[111:104]};

                        // 完成元数据
                        out_meta_r        <= hdr_buf_meta_r;
                        out_meta_r[49]    <= hdr_buf_meta_r[49];  // se (已从beat0 byte63提取)
                        out_meta_r[48:25] <= calc_v6_psn;
                        out_meta_r[24:1]  <= hdr_buf_is_aeth_r ? calc_v6_aeth_msn : 24'h0;

                        if (hdr_buf_is_reth_r) begin
                            out_meta_r[113:82] <= calc_v6_reth_dmalen;
                            out_meta_r[81:50]  <= calc_v6_reth_rkey;
                        end else if (hdr_buf_is_ieth_r) begin
                            out_meta_r[113:82] <= 32'h0;
                            out_meta_r[81:50]  <= calc_v6_ieth_rkey;
                        end else begin
                            out_meta_r[113:82] <= 32'h0;
                            out_meta_r[81:50]  <= 32'h0;
                        end

                        // 输出beat0
                        out_tdata_r      <= hdr_buf_tdata_r;
                        out_tkeep_r      <= hdr_buf_tkeep_r;
                        out_tlast_r      <= hdr_buf_tlast_r;
                        out_valid_r      <= 1'b1;
                        out_meta_valid_r <= 1'b1;
                        nxt_out_valid = 1'b1;

                        // 保存beat1到hdr_buf
                        hdr_buf_tdata_r <= ibuf_tdata_r;
                        hdr_buf_tkeep_r <= ibuf_tkeep_r;
                        hdr_buf_tlast_r <= ibuf_tlast_r;

                        state_r         <= S_V6_RETH_DRAIN;
                        ibuf_valid_r    <= 1'b0;
                        nxt_ibuf_valid = 1'b0;
                        ibuf_consumed  = 1'b1;
                    end
                end

                S_V6_RETH_DRAIN: begin
                    // 排出IPv6 beat1 (与V4_RETH_DRAIN逻辑相同)
                    if (!nxt_out_valid) begin
                        out_tdata_r      <= hdr_buf_tdata_r;
                        out_tkeep_r      <= hdr_buf_tkeep_r;
                        out_tlast_r      <= hdr_buf_tlast_r;
                        out_valid_r      <= 1'b1;
                        out_meta_valid_r <= 1'b0;
                        nxt_out_valid = 1'b1;
                        state_r <= hdr_buf_tlast_r ? S_IDLE : S_BODY;
                    end
                end

                S_BODY: begin
                    if (ibuf_valid_r && !nxt_out_valid) begin
                        out_tdata_r      <= ibuf_tdata_r;
                        out_tkeep_r      <= ibuf_tkeep_r;
                        out_tlast_r      <= ibuf_tlast_r;
                        out_valid_r      <= 1'b1;
                        out_meta_valid_r <= 1'b0;
                        nxt_out_valid = 1'b1;
                        ibuf_valid_r <= 1'b0;
                        nxt_ibuf_valid = 1'b0;
                        ibuf_consumed  = 1'b1;
                        state_r <= ibuf_tlast_r ? S_IDLE : S_BODY;
                    end
                end
            endcase

            // ======== ibuf2 → ibuf 提升 ========
            if (ibuf_consumed && ibuf2_valid_r) begin
                ibuf_tdata_r      <= ibuf2_tdata_r;
                ibuf_tkeep_r      <= ibuf2_tkeep_r;
                ibuf_tlast_r      <= ibuf2_tlast_r;
                ibuf_valid_r      <= 1'b1;
                ibuf_meta_r       <= ibuf2_meta_r;
                ibuf_meta_valid_r <= ibuf2_meta_valid_r;
                nxt_ibuf_valid = 1'b1;
                ibuf2_valid_r      <= 1'b0;
                ibuf2_meta_valid_r <= 1'b0;
                nxt_ibuf2_valid = 1'b0;
            end

            // ======== 输入接收 ========
            if (s_axis_tvalid && in_ready_r) begin
                if (!nxt_ibuf_valid) begin
                    // ibuf空: 直接写入ibuf
                    ibuf_tdata_r      <= s_axis_tdata;
                    ibuf_tkeep_r      <= s_axis_tkeep;
                    ibuf_tlast_r      <= s_axis_tlast;
                    ibuf_valid_r      <= 1'b1;
                    ibuf_meta_r       <= user_metadata_in;
                    ibuf_meta_valid_r <= user_metadata_in_valid;
                    nxt_ibuf_valid = 1'b1;
                end else begin
                    // ibuf满: 写入ibuf2
                    ibuf2_tdata_r      <= s_axis_tdata;
                    ibuf2_tkeep_r      <= s_axis_tkeep;
                    ibuf2_tlast_r      <= s_axis_tlast;
                    ibuf2_valid_r      <= 1'b1;
                    ibuf2_meta_r       <= user_metadata_in;
                    ibuf2_meta_valid_r <= user_metadata_in_valid;
                    nxt_ibuf2_valid = 1'b1;
                end
            end

            // ======== in_ready_r: 下一拍至少有一个空位 ========
            in_ready_r <= !(nxt_ibuf_valid && nxt_ibuf2_valid);
        end
    end

    //=============================
    // AXI-Stream 输出
    //=============================
    assign m_axis_tdata            = out_tdata_r;
    assign m_axis_tkeep            = out_tkeep_r;
    assign m_axis_tvalid           = out_valid_r;
    assign m_axis_tlast            = out_tlast_r;

    assign user_metadata_out       = out_meta_r;
    assign user_metadata_out_valid = out_valid_r & out_meta_valid_r;

    assign s_axis_tready           = in_ready_r;

    //=============================
    // AXI-Lite接口占位
    //=============================
    axi_lite_slave #(
        .REG_ADDR_W(13),
        .REG_PREFIX(16'h5A5A)
    ) reg_inst (
        .s_axil_awvalid(s_axil_awvalid),
        .s_axil_awaddr (s_axil_awaddr),
        .s_axil_awready(s_axil_awready),
        .s_axil_wvalid (s_axil_wvalid),
        .s_axil_wdata  (s_axil_wdata),
        .s_axil_wready (s_axil_wready),
        .s_axil_bvalid (s_axil_bvalid),
        .s_axil_bresp  (s_axil_bresp),
        .s_axil_bready (s_axil_bready),
        .s_axil_arvalid(s_axil_arvalid),
        .s_axil_araddr (s_axil_araddr),
        .s_axil_arready(s_axil_arready),
        .s_axil_rvalid (s_axil_rvalid),
        .s_axil_rdata  (s_axil_rdata),
        .s_axil_rresp  (s_axil_rresp),
        .s_axil_rready (s_axil_rready),

        .aclk   (s_axil_aclk),
        .aresetn(s_axil_aresetn)
    );

endmodule
