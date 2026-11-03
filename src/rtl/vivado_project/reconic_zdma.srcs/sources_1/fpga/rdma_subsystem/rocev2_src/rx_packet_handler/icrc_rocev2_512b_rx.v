module icrc_rocev2_512b_rx (
    input wire clk,
    input wire reset,

    input wire        i_pkt_discarded,  // one cycle delayed w.r.t. s_axis_tvalid
    input wire [31:0] i_pkt_id,         // one cycle delayed w.r.t. s_axis_tvalid

    input wire         s_axis_tvalid,
    input wire [511:0] s_axis_tdata,
    input wire [ 63:0] s_axis_tkeep,
    input wire         s_axis_tlast,

    output wire        o_crc_calc_done,  // ICRC计算完成标志：1-完成，0-未完成
    output wire [31:0] o_pkt_id,         // pkt_id
    output reg  [31:0] o_crc_out,
    output reg         o_crc_err         // ICRC校验和标志：1-错误，0-正确
);

    integer         i;

    wire    [ 15:0] eth_type;
    wire    [ 15:0] pkt_len;
    wire    [  5:0] lbytes_first;
    reg     [  5:0] lbytes_first_r;
    wire    [  5:0] lbytes;
    wire    [  5:0] rbytes_first;
    reg     [  5:0] rbytes_first_r;
    wire    [  5:0] rbytes;

    reg             s_axis_tidle;
    wire            s_axis_tfirst;


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

    reg     [ 31:0] pkt_id;

    assign eth_type     = {s_axis_tdata[12*8+:8], s_axis_tdata[13*8+:8]};
    assign pkt_len      = {s_axis_tdata[16*8+:8], s_axis_tdata[17*8+:8]} + 16'd14;
    assign lbytes_first = ~({pkt_len[5:2], 2'b10}) + 6'd1;
    assign rbytes_first = {pkt_len[5:2], 2'b10};

    always @(posedge clk) begin
        if (reset) begin
            s_axis_tidle <= 1'b1;
        end else if (s_axis_tvalid) begin
            s_axis_tidle <= s_axis_tlast;
        end
    end

    assign s_axis_tfirst = s_axis_tidle & s_axis_tvalid;

    always @(*) begin
        crc_data[7:0] = s_axis_tdata[7:0];
        for (i = 1; i < 64; i = i + 1) begin
            crc_data[8*i+:8] = s_axis_tkeep[i] ? s_axis_tdata[8*i+:8] : 8'h00;
        end

        if (s_axis_tidle) begin
            crc_data[6*8+:64]  = {64{1'b1}};  // CA17-22(a)
            crc_data[47:0]     = {48{1'b0}};
            crc_data[46*8+:8]  = {8{1'b1}};  // BTH RSVD8
            crc_data[40*8+:16] = {16{1'b1}};  // UDP Checksum
            crc_data[24*8+:16] = {16{1'b1}};  // IP Header Checksum
            crc_data[22*8+:8]  = {8{1'b1}};  // Time to Leave
            crc_data[15*8+:8]  = {8{1'b1}};  // DSCP, ECN      

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
        if (s_axis_tidle) begin
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
        endcase
    end

    always @(posedge clk)
        if (reset) cur_st <= 2'd0;
        else if (s_axis_tvalid) cur_st <= nxt_st;

    always @(posedge clk)
        if (reset) prv_st <= 2'd0;
        else prv_st <= cur_st;

    always @(posedge clk) begin
        if (s_axis_tidle) begin
            case (pkt_len[5:2])
                4'd01:   init_seed <= 32'hffffffff;
                4'd02:   init_seed <= 32'hc704dd7b;
                4'd03:   init_seed <= 32'h6904bb59;
                4'd04:   init_seed <= 32'h099c5421;
                4'd05:   init_seed <= 32'h552d22c8;
                4'd06:   init_seed <= 32'h4e26540f;
                4'd07:   init_seed <= 32'hfbac7c3a;
                4'd08:   init_seed <= 32'h6811f1fe;
                4'd09:   init_seed <= 32'h4a55af67;
                4'd10:   init_seed <= 32'h54b292a9;
                4'd11:   init_seed <= 32'h7243c868;
                4'd12:   init_seed <= 32'hc799db3e;
                4'd13:   init_seed <= 32'h5632eeb0;
                4'd14:   init_seed <= 32'hf20f2bcc;
                4'd15:   init_seed <= 32'h6d5aec34;
                default: init_seed <= 32'hx;
            endcase
        end
    end

    always @(*) begin
        case (cur_st)
            2'd0: cur_seed = nxt_seed;
            2'd1, 2'd2: cur_seed = init_seed;
            2'd3: cur_seed = 32'hef6eb7df;
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
            o_crc_err <= (nxt_line_crc32 != 32'hc704dd7b) ? 1'b1 : 1'b0;  // ICRC校验和标志：1-错误，0-正确
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

    always @(posedge clk) pkt_id <= i_pkt_id;

    assign o_pkt_id = 32'h0;

endmodule
