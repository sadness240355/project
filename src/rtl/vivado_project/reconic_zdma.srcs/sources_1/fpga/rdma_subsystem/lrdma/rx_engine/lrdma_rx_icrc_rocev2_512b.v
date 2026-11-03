`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_rx_icrc_rocev2_512b — RX 引擎 RoCEv2 ICRC 校验模块（512 位数据路径）
//
// 支持：
// - IPv4 RoCEv2 RX ICRC check
// - IPv6 RoCEv2 RX ICRC check
//
// 说明：
// RX 输入 AXI-Stream 数据已经包含 4B ICRC 字段，因此 pkt_len 必须包含 ICRC。
// IPv4: pkt_len = Ethernet 14B + IPv4 Total Length
// IPv6: pkt_len = Ethernet 14B + IPv6 Header 40B + IPv6 Payload Length
///////////////////////////////////////////////////////////////////////////////
module lrdma_rx_icrc_rocev2_512b (
    input wire clk,
    input wire reset,

    input wire i_pkt_discarded,  // one cycle delayed w.r.t. s_axis_tvalid

    input wire         s_axis_tvalid,
    input wire [511:0] s_axis_tdata,
    input wire [ 63:0] s_axis_tkeep,
    input wire         s_axis_tlast,

    output wire        o_crc_calc_done,
    output reg  [31:0] o_crc_out,
    output reg         o_crc_err
);

    localparam [15:0] ETH_TYPE_IPV4 = 16'h0800;
    localparam [15:0] ETH_TYPE_IPV6 = 16'h86DD;

    integer         i;

    wire    [ 15:0] eth_type;
    wire    [ 15:0] ipv4_total_len;
    wire    [ 15:0] ipv6_pyld_len;
    wire    [ 15:0] pkt_len;
    wire    [  5:0] lbytes_first;
    reg     [  5:0] lbytes_first_r;
    wire    [  5:0] lbytes;
    wire    [  5:0] rbytes_first;
    reg     [  5:0] rbytes_first_r;
    wire    [  5:0] rbytes;

    reg             s_axis_tidle;
    wire            s_axis_tfirst;

    reg             eth_type_ipv6;

    reg     [511:0] crc_data;
    reg     [511:0] prev_crc_data;
    wire    [511:0] crc_data_la;
    wire    [511:0] prev_crc_data_ra;
    reg     [511:0] crc_data_a;

    reg             tvalid_d1;

    reg             tlast_d1;
    reg             tlast_d2;

    reg     [  1:0] cur_st;
    reg     [  1:0] nxt_st;
    reg     [  1:0] prv_st;

    reg     [ 31:0] init_seed;
    reg     [ 31:0] cur_seed;
    reg     [ 31:0] nxt_seed;

    reg     [ 31:0] lfsr_out;
    reg     [ 31:0] line_crc32;
    wire    [ 31:0] nxt_line_crc32;

    assign eth_type       = {s_axis_tdata[12*8+:8], s_axis_tdata[13*8+:8]};
    assign ipv4_total_len = {s_axis_tdata[16*8+:8], s_axis_tdata[17*8+:8]};
    assign ipv6_pyld_len  = {s_axis_tdata[18*8+:8], s_axis_tdata[19*8+:8]};

    // RX 数据流已经包含 ICRC：
    // IPv4 full frame length = Ethernet 14B + IPv4 Total Length
    // IPv6 full frame length = Ethernet 14B + IPv6 Header 40B + IPv6 Payload Length
    assign pkt_len        = (eth_type == ETH_TYPE_IPV4) ? (ipv4_total_len + 16'd14) : (eth_type == ETH_TYPE_IPV6) ? (ipv6_pyld_len + 16'd54) : (ipv4_total_len + 16'd14);

    assign lbytes_first   = ~({pkt_len[5:2], 2'b10}) + 6'd1;
    assign rbytes_first   = {pkt_len[5:2], 2'b10};

    always @(posedge clk) begin
        if (reset) begin
            s_axis_tidle <= 1'b1;
        end else if (s_axis_tvalid) begin
            s_axis_tidle <= s_axis_tlast;
        end
    end

    assign s_axis_tfirst = s_axis_tidle & s_axis_tvalid;

    // IPv6 的 BTH 第 4 字节屏蔽字段位于绝对 byte66，也就是第二个 512-bit beat 的 byte2。
    // 首拍锁存本包是否 IPv6；第一个非首拍有效 beat 用 eth_type_ipv6 做 mask；之后清零。
    always @(posedge clk) begin
        if (reset) begin
            eth_type_ipv6 <= 1'b0;
        end else if (s_axis_tvalid) begin
            if (s_axis_tidle) begin
                eth_type_ipv6 <= (eth_type == ETH_TYPE_IPV6);
            end else begin
                eth_type_ipv6 <= 1'b0;
            end
        end
    end

    always @(*) begin
        crc_data[7:0] = s_axis_tdata[7:0];
        for (i = 1; i < 64; i = i + 1) begin
            crc_data[8*i+:8] = s_axis_tkeep[i] ? s_axis_tdata[8*i+:8] : 8'h00;
        end

        if (s_axis_tidle) begin
            // Ethernet invariant fields
            crc_data[6*8+:64] = {64{1'b1}};  // Src MAC + EtherType
            crc_data[47:0]    = {48{1'b0}};  // Dst MAC

            if (eth_type == ETH_TYPE_IPV4) begin
                // IPv4 layout:
                // Ethernet 0~13, IPv4 14~33, UDP 34~41, BTH 42~53
                crc_data[46*8+:8]  = {8{1'b1}};  // BTH 第 4 字节屏蔽字段
                crc_data[40*8+:16] = {16{1'b1}};  // UDP Checksum
                crc_data[24*8+:16] = {16{1'b1}};  // IPv4 Header Checksum
                crc_data[22*8+:8]  = {8{1'b1}};  // IPv4 TTL
                crc_data[15*8+:8]  = {8{1'b1}};  // IPv4 DSCP/ECN
            end else if (eth_type == ETH_TYPE_IPV6) begin
                // IPv6 layout:
                // Ethernet 0~13, IPv6 14~53, UDP 54~61, BTH starts at 62
                crc_data[14*8+:4]  = {4{1'b1}};  // IPv6 Traffic Class high part, byte14 low nibble
                crc_data[15*8+:24] = {24{1'b1}};  // IPv6 Traffic Class low part + Flow Label
                crc_data[21*8+:8]  = {8{1'b1}};  // IPv6 Hop Limit
                crc_data[60*8+:16] = {16{1'b1}};  // UDP Checksum

                // IPv6 BTH 第 4 字节屏蔽字段位于绝对 byte66，也就是第二个 512-bit beat 的 byte2。
                // 该字段在下面的 eth_type_ipv6 分支中屏蔽。
            end
        end else if (eth_type_ipv6) begin
            // IPv6 BTH 第 4 字节屏蔽字段：
            // BTH 从绝对 byte62 开始，该字段位于 byte62+4=66。
            // byte66 是第二个 512-bit beat 的 byte2。
            crc_data[2*8+:8] = {8{1'b1}};
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            prev_crc_data <= {512{1'b0}};
        end else if (s_axis_tvalid) begin
            prev_crc_data <= s_axis_tlast ? {512{1'b0}} : crc_data;
        end
    end

    always @(posedge clk) begin
        if (s_axis_tvalid && s_axis_tidle) begin
            lbytes_first_r <= lbytes_first;
            rbytes_first_r <= rbytes_first;
        end
    end

    assign lbytes           = s_axis_tidle ? lbytes_first : lbytes_first_r;
    assign rbytes           = s_axis_tidle ? rbytes_first : rbytes_first_r;

    assign crc_data_la      = crc_data << {lbytes, 3'b0};
    assign prev_crc_data_ra = prev_crc_data >> {rbytes, 3'b0};

    always @(posedge clk) begin
        if (s_axis_tvalid) begin
            crc_data_a <= crc_data_la | prev_crc_data_ra;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            tvalid_d1 <= 1'b0;
        end else begin
            tvalid_d1 <= s_axis_tvalid;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            tlast_d1 <= 1'b0;
            tlast_d2 <= 1'b0;
        end else begin
            tlast_d1 <= s_axis_tlast & s_axis_tvalid;
            tlast_d2 <= tlast_d1 & ~i_pkt_discarded;
        end
    end

    always @(*) begin
        lfsr_out = 32'd0;
        for (i = 0; i < 512; i = i + 1) begin
            lfsr_out = {lfsr_out[30:0], 1'b0} ^ ({32{lfsr_out[31] ^ crc_data_a[i]}} & 32'h04C1_1DB7);
        end
    end

    always @(*) begin
        nxt_st = 2'd0;
        case (cur_st)
            2'd0, 2'd1, 2'd3: begin
                if (s_axis_tidle) begin
                    nxt_st = ((pkt_len[12:6] != 7'd0) && (pkt_len[5:1] == 5'd1)) ? 2'd2 : 2'd1;
                end else begin
                    nxt_st = 2'd0;
                end
            end
            2'd2: begin
                nxt_st = 2'd3;
            end
            default: begin
                nxt_st = 2'd0;
            end
        endcase
    end

    always @(posedge clk) begin
        if (reset) begin
            cur_st <= 2'd0;
        end else if (s_axis_tvalid) begin
            cur_st <= nxt_st;
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            prv_st <= 2'd0;
        end else begin
            prv_st <= cur_st;
        end
    end

    always @(posedge clk) begin
        if (s_axis_tvalid && s_axis_tidle) begin
            case (pkt_len[5:2])
                4'd01:   init_seed <= 32'hffff_ffff;
                4'd02:   init_seed <= 32'hc704_dd7b;
                4'd03:   init_seed <= 32'h6904_bb59;
                4'd04:   init_seed <= 32'h099c_5421;
                4'd05:   init_seed <= 32'h552d_22c8;
                4'd06:   init_seed <= 32'h4e26_540f;
                4'd07:   init_seed <= 32'hfbac_7c3a;
                4'd08:   init_seed <= 32'h6811_f1fe;
                4'd09:   init_seed <= 32'h4a55_af67;
                4'd10:   init_seed <= 32'h54b2_92a9;
                4'd11:   init_seed <= 32'h7243_c868;
                4'd12:   init_seed <= 32'hc799_db3e;
                4'd13:   init_seed <= 32'h5632_eeb0;
                4'd14:   init_seed <= 32'hf20f_2bcc;
                4'd15:   init_seed <= 32'h6d5a_ec34;
                4'd00:   init_seed <= 32'h46af_6449;
                default: init_seed <= 32'h46af_6449;
            endcase
        end
    end

    always @(*) begin
        cur_seed = nxt_seed;
        case (cur_st)
            2'd0: begin
                cur_seed = nxt_seed;
            end
            2'd1, 2'd2: begin
                cur_seed = init_seed;
            end
            2'd3: begin
                cur_seed = 32'hef6e_b7df;
            end
            default: begin
                cur_seed = nxt_seed;
            end
        endcase
    end

    assign nxt_line_crc32 = cur_seed ^ lfsr_out;

    always @(posedge clk) begin
        if (tvalid_d1) begin
            line_crc32 <= nxt_line_crc32;
        end
    end

    always @(*) begin
        nxt_seed = line_crc32;
        for (i = 0; i < 512; i = i + 1) begin
            nxt_seed = {nxt_seed[30:0], 1'b0} ^ ({32{nxt_seed[31]}} & 32'h04C1_1DB7);
        end
    end

    assign o_crc_calc_done = tlast_d2;

    always @(posedge clk) begin
        if (reset) begin
            o_crc_err <= 1'b1;
        end else if (tlast_d1) begin
            o_crc_err <= (nxt_line_crc32 != 32'hc704_dd7b) ? 1'b1 : 1'b0;
        end
    end

    always @(posedge clk) begin
        if (tlast_d1) begin
            for (i = 0; i < 8; i = i + 1) begin
                o_crc_out[i]    <= ~nxt_line_crc32[31-i];
                o_crc_out[8+i]  <= ~nxt_line_crc32[23-i];
                o_crc_out[16+i] <= ~nxt_line_crc32[15-i];
                o_crc_out[24+i] <= ~nxt_line_crc32[7-i];
            end
        end
    end

endmodule
