`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_pointer_queue_mgr — 基于指针的通用环形队列管理模块
//
// 【模块概述】
// 实现经典的环形缓冲区指针管理逻辑，主要用于管理 LRDMA 的 SQ（发送队列）FIFO。
// 包含读指针和写指针，每个指针比实际索引多一位溢出位（wrap-around bit），
// 用于区分队列满和队列空状态：读写指针完全相等时为空；溢出位不同但索引位相等时为满。
//
// 【工作模式】
// 通过 generate 语句支持两种模式（由 C_PUSH_POP_1 参数控制）：
// - 单元素推入/弹出模式（默认）：每次操作一个条目
// - 多元素推入/弹出模式：每次可操作 i_push/pop_num_entries 个条目
//
// 【地址生成】
// 采用基地址加指针偏移方式，偏移量通过条目大小（C_ENTRY_SIZE）自动计算 ADDR_SHIFT。
// o_rd_addr = i_base_addr + (rd_ptr << ADDR_SHIFT)
// o_wr_addr = i_base_addr + (wr_ptr << ADDR_SHIFT)
//
// 【流控信号】
// - o_q_almost_full：基于 C_THRESHOLD 阈值
// - o_num_valid_entries / o_num_free_entries：精确条目计数
// - 支持同时 push+pop 操作
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_pointer_queue_mgr #(
    parameter C_Q_DEPTH    = 32,  // 每个队列中的条目数，4
    parameter C_ENTRY_SIZE = 64,  // 每个条目的大小（字节）。SQ 大小为 64B，512
    parameter C_PTR_WIDTH  = 5,   // 访问队列深度所需的指针位宽，2
    parameter C_THRESHOLD  = 4,   // 产生几乎满载的阈值--用于流量控制，4
    parameter C_PUSH_POP_1 = 1,   // 是否只支持单元素推入/弹出，1
    parameter C_ADDR_WIDTH = 32   // 系统地址总线宽度，32
) (
    // Clock and Reset
    input core_clk,  // Escape Mode Transmit Clock  
    input core_rst,  // Active high core reset

    // 队列指针和状态
    output [C_ADDR_WIDTH -1:0] o_rd_addr,            // 读地址（已与 i_base_addr 合成，下游 AXI 可直接使用）
    output [C_ADDR_WIDTH -1:0] o_wr_addr,            // 写地址（同上）
    output [ C_PTR_WIDTH -1:0] o_rd_ptr,             // 读指针
    output [ C_PTR_WIDTH -1:0] o_wr_ptr,             // 写指针
    output                     o_q_empty,            // 队列空标志
    output                     o_q_full,             // 队列满标志
    output                     o_q_almost_full,      // 队列几乎满标志
    output [  C_PTR_WIDTH : 0] o_num_valid_entries,  // 队列中有效条目的数量
    output [  C_PTR_WIDTH : 0] o_num_free_entries,   // 队列中的空闲条目数

    // 配置信号
    input [C_ADDR_WIDTH -1:0] i_base_addr,  // 队列基地址，32'b0
    input [  C_PTR_WIDTH : 0] i_q_depth,    // 队列深度，WQE_FIFO_DEPTH，4
    input                     i_init,       // 初始化信号，1'b0

    // 推入/弹出操作
    input                   i_q_push,           // 启用队列的推送功能
    input [C_PTR_WIDTH : 0] i_push_num_entries, // 要推送的条目数，3'b1

    input                   i_q_pop,           // 队列的弹出启用
    input [C_PTR_WIDTH : 0] i_pop_num_entries  // 要弹出的条目数，3'b1

);
    `include "lrdma_defs.vh"

    reg  [  C_PTR_WIDTH:0] diff_in_entries;
    reg  [C_PTR_WIDTH : 0] rd_ptr_ff;  // Read pointer with roll over bit--带滚动位的读取指针
    wire [C_PTR_WIDTH : 0] rd_ptr_nxt;  // Read pointer with roll over bit--带滚动位的读取指针

    reg  [C_PTR_WIDTH : 0] wr_ptr;  // Write pointer with roll over bit--带滚动位的写入指针

    reg  [C_PTR_WIDTH : 0] num_valid_entries;  // 队列中有效条目的数量
    reg  [C_PTR_WIDTH : 0] num_free_entries;  // 队列中的空闲条目数
    wire                   fifo_full;
    wire                   fifo_almost_full;
    wire                   fifo_empty;
    wire                   valid_pop;
    wire                   valid_push;

    // 地址偏移计算
    /*
    功能: 根据条目大小（字节）计算 << 偏移位数，用于“指针×条目大小”
          1B  → 偏移 0 位
          4B  → 偏移 2 位
          64B → 偏移 6 位   （SQ 条目为 64B，ADDR_SHIFT=6）
          512B→ 偏移 9 位
    不是 2 的幂或不在列表内的大小会退化到 9，使用者需自行保证 C_ENTRY_SIZE 合法。
    */
    // localparam ADDR_SHIFT = C_ENTRY_SIZE == 1 ? 0 : (C_ENTRY_SIZE == 4 ? 2 : (C_ENTRY_SIZE == 8 ? 3 : (C_ENTRY_SIZE == 16 ? 4 : (C_ENTRY_SIZE == 32 ? 5 : (C_ENTRY_SIZE == 64 ? 6 : (C_ENTRY_SIZE == 128 ? 7 : 8))))));  // 8
    localparam ADDR_SHIFT = C_ENTRY_SIZE == 1 ? 0 : (C_ENTRY_SIZE == 4 ? 2 : (C_ENTRY_SIZE == 8 ? 3 : (C_ENTRY_SIZE == 16 ? 4 : (C_ENTRY_SIZE == 32 ? 5 : (C_ENTRY_SIZE == 64 ? 6 : (C_ENTRY_SIZE == 128 ? 7 : (C_ENTRY_SIZE == 256 ? 8 : 9)))))));  // 9

    // 读指针管理(环形指针)
    assign rd_ptr_nxt[C_PTR_WIDTH-1:0] = valid_pop ? ((rd_ptr_ff[C_PTR_WIDTH-1:0] + i_pop_num_entries < i_q_depth) ? rd_ptr_ff[C_PTR_WIDTH-1:0] + i_pop_num_entries : (rd_ptr_ff[C_PTR_WIDTH-1:0] + i_pop_num_entries - i_q_depth)) : rd_ptr_ff[C_PTR_WIDTH-1:0];
    assign rd_ptr_nxt[C_PTR_WIDTH]     = valid_pop ? ((rd_ptr_ff[C_PTR_WIDTH-1:0] + i_pop_num_entries < i_q_depth) ? rd_ptr_ff[C_PTR_WIDTH] : ~rd_ptr_ff[C_PTR_WIDTH]) : rd_ptr_ff[C_PTR_WIDTH];

    `MSFF_R(rd_ptr_ff, rd_ptr_nxt, core_clk, core_rst)  // rd_ptr_ff <= rd_ptr_nxt

    // 队列有效性检查
    // 弹出时检查队列非空且请求数量不超过有效条目数
    assign valid_pop  = i_q_pop && !fifo_empty && i_pop_num_entries <= num_valid_entries;
    // 推入时检查队列非满且请求数量不超过空闲条目数
    assign valid_push = i_q_push && !fifo_full && i_push_num_entries <= num_free_entries;

    // 写指针管理
    generate
        if (~C_PUSH_POP_1) begin  // 多元素推入/弹出
            always @(posedge core_clk) begin
                if (core_rst) begin
                    wr_ptr <= 'b0;
                end else begin
                    if (valid_push) begin
                        if (wr_ptr[C_PTR_WIDTH-1:0] + i_push_num_entries < i_q_depth) begin
                            wr_ptr[C_PTR_WIDTH-1 : 0] <= wr_ptr[C_PTR_WIDTH-1:0] + i_push_num_entries;
                        end else begin  // roll over after i_q_depth
                            wr_ptr[C_PTR_WIDTH-1 : 0] <= wr_ptr[C_PTR_WIDTH-1:0] + i_push_num_entries - i_q_depth;
                            wr_ptr[C_PTR_WIDTH]       <= ~wr_ptr[C_PTR_WIDTH];
                        end
                    end
                end
            end
        end else begin  // 单元素模式：每次只操作 1 个元素
            always @(posedge core_clk) begin
                if (core_rst) begin
                    wr_ptr <= 'b0;
                end else begin
                    if (valid_push) begin
                        if (wr_ptr[C_PTR_WIDTH-1:0] + 1'b1 < i_q_depth) begin
                            wr_ptr[C_PTR_WIDTH-1 : 0] <= wr_ptr[C_PTR_WIDTH-1:0] + 1'b1;
                        end else begin  // 回绕到队列开始
                            wr_ptr[C_PTR_WIDTH-1 : 0] <= 'b0;
                            wr_ptr[C_PTR_WIDTH]       <= ~wr_ptr[C_PTR_WIDTH];
                        end
                    end
                end
            end
        end
    endgenerate

    // 条目数量管理
    generate
        if (~C_PUSH_POP_1) begin  // 多元素操作
            always @(posedge core_clk) begin
                if (core_rst) begin
                    num_valid_entries <= 'b0;
                    num_free_entries  <= C_Q_DEPTH;
                end else begin
                    if (i_init) begin
                        num_free_entries  <= i_q_depth;
                        num_valid_entries <= 'b0;
                    end else begin
                        case ({
                            valid_pop, valid_push
                        })
                            2'b01: begin  // 仅推入
                                num_valid_entries <= num_valid_entries + i_push_num_entries;
                                num_free_entries  <= num_free_entries - i_push_num_entries;
                            end
                            2'b10: begin  // 仅弹出
                                num_valid_entries <= num_valid_entries - i_pop_num_entries;
                                num_free_entries  <= num_free_entries + i_pop_num_entries;
                            end
                            2'b11: begin  // 同时推入和弹出
                                num_valid_entries <= num_valid_entries - i_pop_num_entries + i_push_num_entries;
                                num_free_entries  <= num_free_entries + i_pop_num_entries - i_push_num_entries;
                            end
                            2'b00: begin  // 无操作
                                num_valid_entries <= num_valid_entries;
                                num_free_entries  <= num_free_entries;
                            end
                        endcase
                    end

                end
            end
        end else begin  // 单元素模式：每次只操作 1 个元素
            always @(posedge core_clk) begin
                if (core_rst) begin
                    num_valid_entries <= 'b0;
                    num_free_entries  <= C_Q_DEPTH;
                end else begin
                    if (i_init) begin
                        num_free_entries  <= i_q_depth;
                        num_valid_entries <= 'b0;
                    end else begin
                        case ({
                            valid_pop, valid_push
                        })
                            2'b01: begin
                                num_valid_entries <= num_valid_entries + 1'b1;
                                num_free_entries  <= num_free_entries - 1'b1;
                            end
                            2'b10: begin
                                num_valid_entries <= num_valid_entries - 1'b1;
                                num_free_entries  <= num_free_entries + 1'b1;
                            end
                            2'b11, 2'b00: begin
                                num_valid_entries <= num_valid_entries;
                                num_free_entries  <= num_free_entries;
                            end
                        endcase
                    end

                end
            end
        end
    endgenerate

    // 空标志：读写指针完全相等（包括溢出位）
    assign fifo_empty       = (wr_ptr == rd_ptr_ff) ? 1'b1 : 1'b0;

    // 满标志：溢出位不同但指针位置相同
    assign fifo_full        = (wr_ptr[C_PTR_WIDTH] != rd_ptr_ff[C_PTR_WIDTH]) && (wr_ptr[C_PTR_WIDTH-1:0] == rd_ptr_ff[C_PTR_WIDTH-1:0]);

    // almost_full 阈值检测：剩余可写条目数 (= i_q_depth - num_valid) ≤ C_THRESHOLD
    // 主要用于上游提前反压，避免在类似 NVMe SQ/CQ 这种不允许“写满最后一格”
    // 的场景下踩过边界。
    assign fifo_almost_full = (diff_in_entries <= C_THRESHOLD);

    ////////////////////////////////For timing fix in LV devices/////////////////////
    // 时序优化：差值计算寄存
    always @(posedge core_clk) begin
        if (core_rst) diff_in_entries <= {(C_PTR_WIDTH + 1) {1'b0}};
        else diff_in_entries <= i_q_depth - num_valid_entries;
    end
    ///////////////////////////////////////////////////////////////////////////

    // 地址生成: 基地址 + (指针位置 × 条目大小)
    assign o_wr_addr           = i_base_addr + (wr_ptr[C_PTR_WIDTH-1:0] << ADDR_SHIFT);
    assign o_rd_addr           = i_base_addr + (rd_ptr_ff[C_PTR_WIDTH-1:0] << ADDR_SHIFT);
    assign o_wr_ptr            = wr_ptr[C_PTR_WIDTH-1:0];
    assign o_rd_ptr            = rd_ptr_ff[C_PTR_WIDTH-1:0];
    assign o_q_full            = fifo_full;
    assign o_q_almost_full     = fifo_almost_full;
    assign o_q_empty           = fifo_empty;
    assign o_num_valid_entries = num_valid_entries;
    assign o_num_free_entries  = num_free_entries;


endmodule
