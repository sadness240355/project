///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_common_sdp_lut_ram_wrapper — 基于 XPM 的简单双口 LUT RAM 封装（零读延迟）
//
// 【模块概述】
// LRDMA 中使用最广泛的基础存储原语封装。配置为分布式 RAM
// （MEMORY_PRIMITIVE="distributed"），READ_LATENCY_B=0，
// 支持组合逻辑级别的零时钟周期读取。
//
// 【同地址读写冲突旁路（C_COMBO_BYPASS=0 时生效）】
// 同周期读地址==写地址且写使能有效：读输出直接选择写入数据
// 保证读到的永远是最新值（对“先写后即刻检查”场景至关重要）
//
// 【参数】
// C_Init_with_1：初始值可选（通过 MEMORY_INIT_PARAM 设置）
// C_COMBO_BYPASS=1：禁用旁路，直接输出 RAM 读取值
//
// 【使用场景】
// QP 状态位、计数器、标志寄存器等小型快速查找表
///////////////////////////////////////////////////////////////////////////////
`default_nettype wire `timescale 1ps / 1ps
module lrdma_common_sdp_lut_ram_wrapper #(
    parameter C_RAM_WORD_WIDTH = 1,
    parameter C_RAM_DEPTH = 2048,
    parameter C_Init_with_1 = 1'b0,  // 选择是否走“初始化为1”的分支
    parameter C_COMBO_BYPASS = 1'b0
) (
    input core_clk,
    input core_rst,

    input                            wr_en,
    input [clog2(C_RAM_DEPTH)-1 : 0] wr_addr,
    input [  C_RAM_WORD_WIDTH-1 : 0] wr_data,

    input                             rd_en,
    input  [clog2(C_RAM_DEPTH)-1 : 0] rd_addr,
    output [  C_RAM_WORD_WIDTH-1 : 0] rd_data   // 无读延迟
);
    `include "lrdma_defs.vh"

    localparam C_ADDR_WIDTH = clog2(C_RAM_DEPTH);
    // 当重置发生时，这些参数适用于 Ram Init 吗？
    localparam Init_value_1 = {C_RAM_DEPTH - 1{"1,"}};
    localparam Init_str_1 = {Init_value_1, {"1,"}};

    generate
        if (C_Init_with_1 == 1'b0) begin  // 初始化为0

            wire [C_RAM_WORD_WIDTH-1 : 0] rd_data_int;
            // 同周期同地址读写冲突，输出直接旁路 wr_data，这层旁路用于保证“读到最新写入值”
            assign rd_data = ((rd_addr == wr_addr) && (wr_en == 1'b1) && (C_COMBO_BYPASS == 0)) ? wr_data : rd_data_int;

            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_RAM_WORD_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),                   // 使用分布式RAM（LUT RAM）
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(""),
                .USE_MEM_INIT     (1),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_RAM_WORD_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),                   // 零延迟读取，支持组合逻辑读取，无需时钟延迟
                .WRITE_MODE_B      ("read_first")         // 读优先模式

            ) dist_sdp_ram_inst_0 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en),
                .wea           (wr_en),
                .addra         (wr_addr),
                .dina          (wr_data),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (rd_en),        // 读取延迟为 0 时，无需启用读取端口，否则映射 rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_data_int),
                .sbiterrb(),
                .dbiterrb()

            );
        end
    endgenerate

    generate
        if (C_Init_with_1 == 1'b1) begin  // 初始化为1

            wire [C_RAM_WORD_WIDTH-1 : 0] rd_data_int;
            assign rd_data = ((rd_addr == wr_addr) && (wr_en == 1'b1) && (C_COMBO_BYPASS == 0)) ? wr_data : rd_data_int;

            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_RAM_WORD_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_1),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_RAM_WORD_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                //.READ_RESET_VALUE_B ("0"),                           			
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) dist_sdp_ram_inst_0 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en),
                .wea           (wr_en),
                .addra         (wr_addr),
                .dina          (wr_data),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (rd_en),        // 读取延迟为 0 时，无需启用读取端口，否则映射 rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_data_int),
                .sbiterrb(),
                .dbiterrb()

            );
        end
    endgenerate


endmodule
