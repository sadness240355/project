// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_common_reg_slice — AXI Register Slice（双向打拍寄存器切片）
//
// 【模块概述】
// Full Registered 模式，同时切断前向（valid+data）和反向（ready）信号路径。
// 双缓冲结构（storage_data1 + storage_data2）避免数据丢失或重复。
//
// 【三态 FSM】
// - ZERO（空）：无数据，接受任何输入
// - ONE（单缓冲）：storage_data1 有数据，等待下游接收
//   下游未就绪但上游有新数据 → TWO 态并停止接受
// - TWO（双缓冲）：两个缓冲区都满，下游读走后回 ONE 态
//
// 【数据加载优先级】
// TWO 态：storage_data1 从 storage_data2 加载（FIFO 顺序）
// 其他态：从输入端加载
//
// 【输出】
// M_VALID = state[0]（ONE/TWO 态均有效）
// 支持 aclken 时钟使能；areset 在 aclk 下同步采样
///////////////////////////////////////////////////////////////////////////////
`default_nettype wire `timescale 1ps / 1ps
module lrdma_common_reg_slice #(
    parameter C_DATA_WIDTH = 32
) (
    // System Signals
    input aclk,
    input areset,
    input aclken,

    // Slave side
    input  [C_DATA_WIDTH-1:0] S_PAYLOAD_DATA,
    input                     S_VALID,
    output                    S_READY,

    // Master side
    output [C_DATA_WIDTH-1:0] M_PAYLOAD_DATA,
    output                    M_VALID,
    input                     M_READY
);

    reg [1:0] state;
    localparam [1:0] ZERO = 2'b10,  // 空闲：无数据缓存
    ONE = 2'b11,  // 单缓存：storage_data1有数据
    TWO = 2'b01;  // 双缓存：两个存储都有数据

    // 双缓冲存储结构
    reg  [C_DATA_WIDTH-1:0] storage_data1;  // 主缓存，总是连接到输出端 M_PAYLOAD_DATA
    reg  [C_DATA_WIDTH-1:0] storage_data2;  // 辅助缓存，当 storage_data1 忙时接收新来的数据
    reg                     load_s1;
    wire                    load_s2;
    wire                    load_s1_from_s2;
    reg                     s_ready_i;  //local signal of output
    wire                    m_valid_i;  //local signal of output

    // assign local signal to its output signal
    assign S_READY = s_ready_i;
    assign M_VALID = m_valid_i;

    (* equivalent_register_removal = "no" *) reg [1:0] areset_d;  // Reset delay register
    always @(posedge aclk) begin
        if (aclken) begin
            areset_d <= {areset_d[0], areset};
        end
    end

    // Load storage1 with either slave side data or from storage2
    // 数据源选择：优先从 storage_data2 加载（TWO状态时），否则从输入端加载
    always @(posedge aclk) begin
        if (aclken) begin
            storage_data1 <= ~load_s1 ? storage_data1 : load_s1_from_s2 ? storage_data2 : S_PAYLOAD_DATA;
        end
    end

    // Load storage2 with slave side data
    always @(posedge aclk) begin
        if (aclken) begin
            storage_data2 <= load_s2 ? S_PAYLOAD_DATA : storage_data2;
        end
    end

    assign M_PAYLOAD_DATA = storage_data1;

    // Always load s2 on a valid transaction even if it's unnecessary
    // storage_data2 加载条件
    assign load_s2        = S_VALID & s_ready_i;  // 只要有有效输入且ready就加载

    // Loading s1
    // storage_data1 加载条件
    always @(*) begin
        if (((state == ZERO) && (S_VALID == 1)) ||  // Load when empty on slave transaction
            // Load when ONE if we both have read and write at the same time
            ((state == ONE) && (S_VALID == 1) && (M_READY == 1)) ||
            // Load when TWO and we have a transaction on Master side
            ((state == TWO) && (M_READY == 1)))
            load_s1 = 1'b1;
        else load_s1 = 1'b0;
    end  // always @ *

    assign load_s1_from_s2 = (state == TWO);

    // State Machine for handling output signals
    // 处理输出信号的状态机
    always @(posedge aclk) begin
        if (areset) begin
            s_ready_i <= 1'b0;
            state     <= ZERO;
        end else if (aclken && areset_d == 2'b10) begin
            s_ready_i <= 1'b1;
            state     <= ZERO;
        end else if (aclken && areset_d == 2'b00) begin
            case (state)
                // No transaction stored locally
                // 无数据缓存
                ZERO: if (S_VALID) state <= ONE;  // 当输入有效数据时，转到 ONE 状态

                // One transaction stored locally
                // 单缓存
                ONE: begin
                    if (~M_READY & S_VALID) begin
                        state     <= TWO;  // 下游未就绪但上游有数据 → 双缓存
                        s_ready_i <= 1'b0;  // 停止接收新数据
                    end else if (M_READY & ~S_VALID) begin
                        state <= ZERO;  // 下游读走数据且上游无新数据 → 空闲
                    end
                end
                // 如果 M_READY & S_VALID：同时读写，保持 ONE 状态

                // TWO transaction stored locally
                // 双缓存
                TWO:
                if (M_READY) begin
                    state     <= ONE;  // 下游读走一个数据
                    s_ready_i <= 1'b1;  // 重新允许接收数据
                end

                default: state <= ZERO;

            endcase  // case (state)
        end
    end  // always @ (posedge aclk)

    assign m_valid_i = state[0];

endmodule  // axisc_register_slice
