// =====================================================================
// lrdma_common_xpm_memory_tdpram_qp1
// LRDMA RQ 配置寄存器（仅 QP1/MAD 使用）的 depth=1 等价替换。
// 说明：
// 1. 本版本只适用于原 clka / clkb 确认属于同一个时钟域的情况。
// 2. 只有地址 1 有效。
// 3. 地址不为 1 时，读输出为 0，写入被忽略。
// 4. A/B 两口同周期同时写地址 1 时，A 口优先。
// 5. 同周期读写地址 1 时，读出旧值，等价 read-first 行为。
// =====================================================================
module lrdma_common_xpm_memory_tdpram_qp1 #(
    parameter integer WIDTH      = 32,
    parameter integer ADDR_WIDTH = 8,
    parameter integer LATENCY_A  = 2,
    parameter integer LATENCY_B  = 1
) (
    input wire clk,
    input wire rst,

    input  wire                  wea,
    input  wire [ADDR_WIDTH-1:0] addra,
    input  wire [     WIDTH-1:0] dina,
    output wire [     WIDTH-1:0] douta,

    input  wire                  web,
    input  wire [ADDR_WIDTH-1:0] addrb,
    input  wire [     WIDTH-1:0] dinb,
    output wire [     WIDTH-1:0] doutb
);

    localparam [ADDR_WIDTH-1:0] QP1_ADDR = 1;

    reg  [WIDTH-1:0] reg_q1;

    wire             addra_is_q1 = (addra == QP1_ADDR);
    wire             addrb_is_q1 = (addrb == QP1_ADDR);

    // ================================================================
    // depth=1 存储体
    // 只有地址 1 有效。
    // 地址不为 1 时，写入忽略。
    // 如果 A/B 口同周期都写地址 1，则 A 口优先。
    // ================================================================
    always @(posedge clk) begin
        if (rst) begin
            reg_q1 <= {WIDTH{1'b0}};
        end else begin
            if (wea && addra_is_q1) begin
                reg_q1 <= dina;
            end else if (web && addrb_is_q1) begin
                reg_q1 <= dinb;
            end
        end
    end

    // ================================================================
    // A 口读延迟
    // 地址不为 1 时，经过 LATENCY_A 后输出 0。
    // ================================================================
    reg     [WIDTH-1:0] porta_pipe[0:LATENCY_A-1];

    integer             pa;

    always @(posedge clk) begin
        if (rst) begin
            for (pa = 0; pa < LATENCY_A; pa = pa + 1) begin
                porta_pipe[pa] <= {WIDTH{1'b0}};
            end
        end else begin
            porta_pipe[0] <= addra_is_q1 ? reg_q1 : {WIDTH{1'b0}};

            for (pa = 1; pa < LATENCY_A; pa = pa + 1) begin
                porta_pipe[pa] <= porta_pipe[pa-1];
            end
        end
    end

    assign douta = porta_pipe[LATENCY_A-1];

    // ================================================================
    // B 口读延迟
    // 地址不为 1 时，经过 LATENCY_B 后输出 0。
    // ================================================================
    reg     [WIDTH-1:0] portb_pipe[0:LATENCY_B-1];

    integer             pb;

    always @(posedge clk) begin
        if (rst) begin
            for (pb = 0; pb < LATENCY_B; pb = pb + 1) begin
                portb_pipe[pb] <= {WIDTH{1'b0}};
            end
        end else begin
            portb_pipe[0] <= addrb_is_q1 ? reg_q1 : {WIDTH{1'b0}};

            for (pb = 1; pb < LATENCY_B; pb = pb + 1) begin
                portb_pipe[pb] <= portb_pipe[pb-1];
            end
        end
    end

    assign doutb = portb_pipe[LATENCY_B-1];

endmodule
