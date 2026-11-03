`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_tx_buffer_mgr — TX 引擎缓冲区管理模块
//
// 【模块概述】
// 在 hdr_gen（头部生成）与 dma（搬运）之间进行数据缓冲和描述符管理。
// 接收 128B 包头数据分两拍写入 HDR BRAM，同时生成 SGL 描述符写入 SGL BRAM。
//
// 【HDR BRAM】（512-bit × 2 × C_MAX_HDR_DEPTH）
// 每包头 128B 分两拍（低/高 512b）写入
//
// 【SGL BRAM】（83-bit × C_MAX_SGL_DEPTH）
// 格式：{QP1_HDR_VAL[82], EOF[81], SOF[80], payload_len/local_offset/hdr_addr[79:0]}
// - 纯头部包（ACK/零 payload）：一条 SGL（SOF=1, EOF=1）
// - 需 DMA payload 的 WRITE：两条 SGL
//   第一条（SOF=1, EOF=0）：头部信息 + HDR BRAM 地址
//   第二条（SOF=0, EOF=1）：DMA 源地址 + 数据长度
// - QP1（MAD）：一条 QP1_HDR_VAL=1 的 SGL（不写 HDR BRAM）
//
// 【FSM】
// IDLE → WR_HDR → IDLE（完成后 o_buf_mngr_rdy 握手）
//
// 【环形缓冲指针管理】
// sgl_head/tail_ptr 和 hdr_head/tail_ptr 各 16-bit（bit[15]=回绕标志）
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_tx_buffer_mgr #(
    parameter C_MAX_QP = 8,
    parameter C_MAX_QID_WIDTH = 3,
    parameter C_MAX_HDR_DEPTH = 512,
    parameter C_MAX_SGL_DEPTH = 32,
    parameter C_ENABLE_EARLY_BUF_MGR_READY = 1
) (
    input core_clk,
    input core_rst,

    // 与 hdr_gen 的接口，hdr_gen 生成的包头
    output reg          o_buf_mngr_rdy,  // 本次写入ram流程完成，常规数据包路径下通常是 hdr+sgl 都完成后拉高；特殊分支下不一定两者都写
    input      [1023:0] i_hdr_data,      // 头
    input      [   7:0] i_hdr_len,       // 头长度信息
    input               i_hdr_valid,     // 头有效

    input i_reg_ip_ver,

    output [19:0] buf_mgr_ptr,

    // SGL Buffer signals
    output reg [15:0] sgl_head_ptr,
    input      [14:0] sgl_bram_rd_addr,
    input             sgl_bram_rd_en,
    output     [82:0] sgl_bram_rd_data,

    // Header buffer signals
    input  [ 14:0] hdr_bram_rd_addr,
    input          hdr_bram_rd_en,
    output [511:0] hdr_bram_rd_data,
    input          hdr_bram_pop
);
    `include "lrdma_defs.vh"
    localparam QP1 = 'd0;

    localparam SGL_BRAM_ADDR_WIDTH = clog2(C_MAX_SGL_DEPTH);
    localparam HDR_BRAM_ADDR_WIDTH = clog2(C_MAX_HDR_DEPTH);
    localparam IDLE = 3'b001;  // 空闲。等 hdr_gen 推一拍带 i_hdr_valid 的包头进来，同拍 o_buf_mngr_rdy 维持高（表示可接收）
    localparam WR_HDR = 3'b010;  // 忙。把 1024 bit 包头分两拍写进 HDR BRAM（低 512b、高 512b），同时在 SGL BRAM
                                 // 里插入 1–2 条 SGL 描述符。全部写完后回 IDLE，o_buf_mngr_rdy 重新拉高

    reg  [                    2:0] buf_mgr_cs;
    reg  [                   15:0] hdr_head_ptr;
    reg  [                   15:0] hdr_tail_ptr;
    reg  [                   15:0] sgl_head_ptr_prev;
    reg  [                   15:0] sgl_tail_ptr;
    reg  [                   82:0] sgl_bram_wr_data;
    wire [SGL_BRAM_ADDR_WIDTH-1:0] sgl_bram_wr_addr;
    reg                            sgl_bram_wr_en;
    reg                            sgl_bram_rd_en_r;
    reg                            hdr_bram_rd_en_r;
    reg                            sgl_wr_done;
    reg  [                  511:0] hdr_bram_wr_data;
    reg  [  HDR_BRAM_ADDR_WIDTH:0] hdr_bram_wr_addr;
    reg                            hdr_bram_wr_en;
    reg                            sgl_buf_full;
    reg                            hdr_buf_full;

    assign sgl_bram_wr_addr = sgl_head_ptr[SGL_BRAM_ADDR_WIDTH-1:0];
    assign buf_mgr_ptr      = {sgl_head_ptr[7:0], sgl_tail_ptr[7:0], buf_mgr_cs, 1'b0};


    always @(posedge core_clk) begin
        if (core_rst) begin
            o_buf_mngr_rdy   <= 1'b0;
            hdr_head_ptr     <= 16'h0000;
            sgl_bram_wr_en   <= 1'b0;
            sgl_bram_wr_data <= 83'd000;
            sgl_wr_done      <= 1'b1;
            buf_mgr_cs       <= IDLE;
            hdr_bram_wr_addr <= {HDR_BRAM_ADDR_WIDTH + 1{1'b0}};
        end else begin
            case (buf_mgr_cs)

                IDLE: begin
                    o_buf_mngr_rdy <= 1'b0;

                    // 路径 A — 普通 QP（i_hdr_data[943:928] != QP1）
                    if (~sgl_buf_full && ~hdr_buf_full && (i_hdr_valid && ~o_buf_mngr_rdy) && (i_hdr_data[942:928] != QP1)) begin
                        sgl_bram_wr_data[82]   <= 1'b0;
                        hdr_bram_wr_en         <= 1'b1;
                        hdr_bram_wr_addr       <= {hdr_head_ptr[HDR_BRAM_ADDR_WIDTH-1:0], 1'b0};
                        hdr_bram_wr_data       <= i_hdr_data[511:0];
                        buf_mgr_cs             <= WR_HDR;
                        sgl_bram_wr_en         <= 1'b1;
                        sgl_bram_wr_data[79:0] <= {8'h00, i_hdr_len, 32'd0, (32'h00000000 + (hdr_head_ptr[14:0] * 16'h80))};
                        sgl_bram_wr_data[80]   <= 1'b1;

                        // IPv6
                        if (i_reg_ip_ver) begin
                            // 单 SGL，不需要 DMA 搬数据，仅用去 HDR BRAM 读零长度 payload 包头
                            if (~|i_hdr_data[1023:1008]) begin
                                sgl_bram_wr_data[81] <= 1'b1;
                                sgl_wr_done          <= 1'b1;
                            end //
                            
                            // 双 SGL（需要 DMA payload 的操作，即 DMA payload 操作是指需要从本地内存读取数据（通过 AXI DMA）拼接到包里发出去的 RDMA 操作）
                            else begin
                                sgl_bram_wr_data[81] <= 1'b0;
                                sgl_wr_done          <= 1'b0;
                            end
                        end //

                        // IPv4
                        else begin
                            // 单 SGL，不需要 DMA 搬数据，仅用去 HDR BRAM 读零长度 payload 包头
                            if (~|i_hdr_data[1023:1008]) begin
                                sgl_bram_wr_data[81] <= 1'b1;
                                sgl_wr_done          <= 1'b1;
                            end //
                
                            // 双 SGL（需要 DMA payload 的操作，即 DMA payload 操作是指需要从本地内存读取数据（通过 AXI DMA）拼接到包里发出去的 RDMA 操作）
                            else begin
                                sgl_bram_wr_data[81] <= 1'b0;
                                sgl_wr_done          <= 1'b0;
                            end
                        end
                    end //

                    // 路径 B — QP1（MAD/UD）
                    else if (~sgl_buf_full && ~hdr_buf_full && (i_hdr_valid && ~o_buf_mngr_rdy) && (i_hdr_data[942:928] == QP1)) begin
                        sgl_bram_wr_data[82]    <= 1'b1;  // QP1_HDR_VAL
                        sgl_bram_wr_en          <= 1'b1;  // 只写一条 SGL
                        sgl_wr_done             <= 1'b1;
                        sgl_bram_wr_data[81:80] <= 2'b11;
                        sgl_bram_wr_data[79:0]  <= {i_hdr_data[1023:1008], i_hdr_data[1007:944]};  // hdr_gen 的{payload_len, local_offset}
                        buf_mgr_cs              <= WR_HDR;
                        hdr_bram_wr_en          <= 1'b0;  // 不写 HDR BRAM
                    end else begin
                        hdr_bram_wr_en <= 1'b0;
                        sgl_bram_wr_en <= 1'b0;
                    end
                end

                WR_HDR: begin
                    // 写 HDR BRAM
                    hdr_bram_wr_en   <= (i_hdr_data[942:928] != QP1);  // 不是 QP1，才把 hdr 写入 ram
                    hdr_bram_wr_addr <= {hdr_head_ptr[HDR_BRAM_ADDR_WIDTH-1:0], 1'b1};
                    hdr_bram_wr_data <= i_hdr_data[1023:512];

                    // QP1 的处理: 不把 hdr 写入 hdr ram，只写 sgl ram(hdr 头指针不 +1)
                    // 其余 QP 的处理: 正常 hdr 头指针 +1
                    if (sgl_wr_done || (i_hdr_data[942:928] == QP1)) begin
                        o_buf_mngr_rdy <= 1'b1;
                        buf_mgr_cs     <= IDLE;
                        if ((i_hdr_data[942:928] != QP1) && (hdr_head_ptr[14:0] == C_MAX_HDR_DEPTH - 1'b1)) begin
                            hdr_head_ptr[14:0] <= 15'h0000;
                            hdr_head_ptr[15]   <= ~hdr_head_ptr[15];
                        end else if ((i_hdr_data[942:928] != QP1)) begin
                            hdr_head_ptr[14:0] <= hdr_head_ptr[14:0] + 1'b1;
                        end
                    end

                    // Write the payload SGL immediately after the high header beat.
                    // In low-latency mode, the buffer manager can release hdr_gen in
                    // this same cycle because both the high header BRAM write and the
                    // payload SGL write have been issued. Legacy mode keeps the extra
                    // WR_HDR cycle for one-to-one timing comparison.
                    if (~sgl_wr_done && ~((sgl_tail_ptr[14:0] == sgl_head_ptr[14:0]) && (sgl_tail_ptr[15] != sgl_head_ptr[15]))) begin
                        sgl_bram_wr_en          <= 1'b1;
                        sgl_wr_done             <= 1'b1;
                        sgl_bram_wr_data[81:80] <= 2'b10;
                        sgl_bram_wr_data[79:0]  <= {i_hdr_data[1023:1008], i_hdr_data[1007:944]};
                        if (C_ENABLE_EARLY_BUF_MGR_READY != 0) begin
                            o_buf_mngr_rdy <= 1'b1;
                            buf_mgr_cs     <= IDLE;
                            if (hdr_head_ptr[14:0] == C_MAX_HDR_DEPTH - 1'b1) begin
                                hdr_head_ptr[14:0] <= 15'h0000;
                                hdr_head_ptr[15]   <= ~hdr_head_ptr[15];
                            end else begin
                                hdr_head_ptr[14:0] <= hdr_head_ptr[14:0] + 1'b1;
                            end
                        end
                    end else begin
                        sgl_bram_wr_en <= 1'b0;
                    end
                end

            endcase
        end
    end



    // ======== hdr 和 sgl ram 指针(地址)管理 ========
    always @(posedge core_clk) begin
        if (core_rst) begin
            sgl_head_ptr      <= 16'h0000;
            sgl_head_ptr_prev <= 16'h0000;
            sgl_tail_ptr      <= 16'h0000;
            sgl_buf_full      <= 1'b0;
            hdr_buf_full      <= 1'b0;
            hdr_tail_ptr      <= 16'h0000;
            sgl_bram_rd_en_r  <= 1'b0;
            hdr_bram_rd_en_r  <= 1'b0;
        end else begin
            sgl_buf_full <= (sgl_tail_ptr[14:0] == sgl_head_ptr[14:0]) && (sgl_tail_ptr[15] != sgl_head_ptr[15]);
            hdr_buf_full <= (hdr_tail_ptr[14:0] == hdr_head_ptr[14:0]) && (hdr_tail_ptr[15] != hdr_head_ptr[15]);
            if (sgl_bram_wr_en) begin  // write后，头指针+1
                sgl_head_ptr_prev <= sgl_head_ptr;
                if (sgl_head_ptr[14:0] == C_MAX_SGL_DEPTH - 1'b1) begin
                    sgl_head_ptr[14:0] <= 15'h0000;
                    sgl_head_ptr[15]   <= ~sgl_head_ptr[15];
                end else begin
                    sgl_head_ptr[14:0] <= sgl_head_ptr[14:0] + 1'b1;
                end
            end
            sgl_bram_rd_en_r <= sgl_bram_rd_en;
            hdr_bram_rd_en_r <= hdr_bram_rd_en;

            if (sgl_bram_rd_en) begin  // read后，尾指针+1
                if (sgl_tail_ptr[14:0] == C_MAX_SGL_DEPTH - 1'b1) begin
                    sgl_tail_ptr[14:0] <= 15'h0000;
                    sgl_tail_ptr[15]   <= ~sgl_tail_ptr[15];
                end else begin
                    sgl_tail_ptr[14:0] <= sgl_tail_ptr[14:0] + 1'b1;
                end
            end

            if (hdr_bram_pop) begin  // pop后，尾指针+1
                if (hdr_tail_ptr[14:0] == C_MAX_HDR_DEPTH - 1'b1) begin
                    hdr_tail_ptr[14:0] <= 15'h0000;
                    hdr_tail_ptr[15]   <= ~hdr_tail_ptr[15];
                end else begin
                    hdr_tail_ptr[14:0] <= hdr_tail_ptr[14:0] + 1'b1;
                end
            end
        end
    end

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (83 * C_MAX_SGL_DEPTH),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),        //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                    //string;
        .USE_MEM_INIT     (1),                     //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),       //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                     //integer; 0,1
        .ECC_MODE         ("no_ecc"),              //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                     // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(83),                  //positive integer
        .BYTE_WRITE_WIDTH_A(83),                  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (SGL_BRAM_ADDR_WIDTH), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (83),                   //positive integer
        .ADDR_WIDTH_B      (SGL_BRAM_ADDR_WIDTH),  //positive integer
        .READ_RESET_VALUE_B("0"),                  //string
        .READ_LATENCY_B    (1),                    //non-negative integer
        .WRITE_MODE_B      ("read_first")          //string; "write_first", "read_first", "no_change" 

    ) sgl_buffer_xpm (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (sgl_bram_wr_en),
        .wea           (1'b1),
        .addra         (sgl_bram_wr_addr),
        .dina          (sgl_bram_wr_data),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (core_rst),
        .enb     (sgl_bram_rd_en),
        .regceb  (1'b1),
        .addrb   (sgl_bram_rd_addr[SGL_BRAM_ADDR_WIDTH-1:0]),
        .doutb   (sgl_bram_rd_data),
        .sbiterrb(),
        .dbiterrb()

    );

    xpm_memory_sdpram #(

        // Common module parameters
        .MEMORY_SIZE      (512 * 2 * C_MAX_HDR_DEPTH),  //positive integer
        .MEMORY_PRIMITIVE ("auto"),                     //string; "auto", "distributed", "block" or "ultra";
        .CLOCKING_MODE    ("common_clock"),             //string; "common_clock", "independent_clock" 
        .MEMORY_INIT_FILE ("none"),                     //string; "none" or "<filename>.mem" 
        .MEMORY_INIT_PARAM(""),                         //string;
        .USE_MEM_INIT     (1),                          //integer; 0,1
        .WAKEUP_TIME      ("disable_sleep"),            //string; "disable_sleep" or "use_sleep_pin" 
        .MESSAGE_CONTROL  (0),                          //integer; 0,1
        .ECC_MODE         ("no_ecc"),                   //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
        .AUTO_SLEEP_TIME  (0),                          // 自动 sleep 延迟为 0

        // Port A module parameters
        .WRITE_DATA_WIDTH_A(512),                     //positive integer
        .BYTE_WRITE_WIDTH_A(512),                     //integer; 8, 9, or WRITE_DATA_WIDTH_A value
        .ADDR_WIDTH_A      (HDR_BRAM_ADDR_WIDTH + 1), //positive integer

        // Port B module parameters
        .READ_DATA_WIDTH_B (512),                      //positive integer
        .ADDR_WIDTH_B      (HDR_BRAM_ADDR_WIDTH + 1),  //positive integer
        .READ_RESET_VALUE_B("0"),                      //string
        .READ_LATENCY_B    (1),                        //non-negative integer
        .WRITE_MODE_B      ("read_first")              //string; "write_first", "read_first", "no_change" 

    ) hdr_buffer_xpm (

        // Common module ports
        .sleep(1'b0),

        // Port A module ports
        .clka          (core_clk),
        .ena           (hdr_bram_wr_en),
        .wea           (1'b1),
        .addra         (hdr_bram_wr_addr),
        .dina          (hdr_bram_wr_data),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),

        // Port B module ports
        .clkb    (core_clk),
        .rstb    (core_rst),
        .enb     (hdr_bram_rd_en),                           //bus2ip_rd_req),
        .regceb  (1'b1),
        .addrb   (hdr_bram_rd_addr[HDR_BRAM_ADDR_WIDTH:0]),
        .doutb   (hdr_bram_rd_data),
        .sbiterrb(),
        .dbiterrb()

    );

    //-------------------------------------------------------------------------------------------------

endmodule
