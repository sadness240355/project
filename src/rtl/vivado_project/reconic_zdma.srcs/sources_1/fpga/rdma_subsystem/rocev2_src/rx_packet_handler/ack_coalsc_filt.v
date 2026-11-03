// 根据 rsp_pkt_buf 未使用空间是否 < 70 ，决定是否过滤ACK包
// 在 rsp_pkt_buf 剩余空间紧张时识别 IPv4/IPv6 ACK 包并抑制其向下游传递，缓解拥塞。
module ack_coalsc_filt (
    input  wire         clk,
    input  wire         reset,
    input  wire         ack_coalsc_en,    // 1'b1, 使能ACK包过滤功能
    input  wire         ack_coalsc_cond,  // rsp_pkt_buf未使用空间 < 70 个单元时，允许过滤ACK包
    input  wire         s_axis_tvalid,
    input  wire [511:0] s_axis_tdata,
    input  wire [ 63:0] s_axis_tkeep,
    input  wire         s_axis_tlast,
    input  wire [  0:0] s_axis_tuser,
    output wire         m_axis_tvalid,
    output reg  [511:0] m_axis_tdata,
    output reg  [ 63:0] m_axis_tkeep,
    output reg          m_axis_tlast,
    output reg  [  0:0] m_axis_tuser
);

    localparam [15:0] ETH_TYPE_IPv6 = 16'hDD86;
    localparam [7:0] ACKNOWLEDGE = 8'h11;
    localparam [1:0] ACK = 2'b00;

    reg          s_axis_tidle;  // 包到来时，除包第一拍，其余都为低；包没到来时，均为高

    reg          ipv6_det;  // 当前包为IPv6标志
    reg          nxt_ipv6_det;  // next IPv6标志
    wire         r1_en;  // 第一级寄存器使能

    reg          tvalid_pre_gt;
    // 时序与s_axis_*对齐，有延迟
    wire         tvalid;
    reg  [511:0] tdata;
    reg  [ 63:0] tkeep;
    reg          tlast;
    reg  [  0:0] tuser;
    reg          tidle;

    reg          ack_coalsc_on;

    reg          m_axis_tvalid_pre_gt;

    // 包到来时，除包第一拍，其余都为低；
    // 包没到来时，均为高。
    // 目的：为了检测包头第一拍。
    always @(posedge clk)
        if (reset) s_axis_tidle <= 1'b1;
        else if (s_axis_tvalid) s_axis_tidle <= s_axis_tlast;

    // 检测包头第一拍的以太网类型字段是否为IPv6包
    always @(*) begin
        nxt_ipv6_det = ipv6_det;
        if (ipv6_det) begin
            if (s_axis_tvalid) nxt_ipv6_det = 1'b0;  // 不是IPv6
        end else if (s_axis_tidle && s_axis_tvalid && (s_axis_tdata[12*8+:16] == ETH_TYPE_IPv6)) begin  // 头部第一拍中以太网类型为IPv6
            nxt_ipv6_det = 1'b1;  // 是IPv6
        end
    end

    always @(posedge clk)
        if (reset) ipv6_det <= 1'b0;
        else ipv6_det <= nxt_ipv6_det;

    // 一级寄存器使能逻辑：非IPv6包直接通过，IPv6包需等待有效数据
    assign r1_en = ~ipv6_det | (ipv6_det & s_axis_tvalid);

    always @(posedge clk) begin
        if (reset) begin
            tvalid_pre_gt <= 1'b0;
            tdata         <= 512'd0;
            tkeep         <= 64'd0;
            tlast         <= 1'b0;
            tuser         <= 1'b0;
        end else if (r1_en) begin
            tvalid_pre_gt <= s_axis_tvalid;
            tdata         <= s_axis_tdata;
            tkeep         <= s_axis_tkeep;
            tlast         <= s_axis_tlast;
            tuser         <= s_axis_tuser;
        end
    end

    assign tvalid = r1_en & tvalid_pre_gt;

    always @(posedge clk)
        if (reset) tidle <= 1'b1;
        else if (tvalid) tidle <= tlast;  // 延迟后的有效tlast，为了检测包头第一拍

    always @(posedge clk) begin
        if (reset) begin
            ack_coalsc_on <= 1'b0;
        end else if (tidle && tvalid) begin
            // ACKNOWLEDGE 是 BTH opcode 字段
            // ACK 是 AETH Syndrome 字段
            if (tdata[12*8+:16] == ETH_TYPE_IPv6) begin  // IPv6包
                if ((tdata[62*8+:8] == ACKNOWLEDGE) && (s_axis_tdata[(13*8+5)+:2] == ACK)) ack_coalsc_on <= ack_coalsc_en & ack_coalsc_cond;
                else ack_coalsc_on <= 1'b0;
            end else begin  // IPv4包(不支持Options,即IP头长度固定为20字节)
                if ((tdata[42*8+:8] == ACKNOWLEDGE) && (tdata[(57*8+5)+:2] == ACK)) ack_coalsc_on <= ack_coalsc_en & ack_coalsc_cond;
                else ack_coalsc_on <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            m_axis_tvalid_pre_gt <= 1'b0;
            m_axis_tdata         <= 512'd0;
            m_axis_tkeep         <= 64'd0;
            m_axis_tlast         <= 1'b0;
            m_axis_tuser         <= 1'b0;
        end else begin
            m_axis_tvalid_pre_gt <= tvalid;
            m_axis_tdata         <= tdata;
            m_axis_tkeep         <= tkeep;
            m_axis_tlast         <= tlast;
            m_axis_tuser         <= tuser;
        end
    end

    // ack_coalsc_on拉高，达到过滤ACK包的目的
    assign m_axis_tvalid = m_axis_tvalid_pre_gt & ~ack_coalsc_on;

endmodule
