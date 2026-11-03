///////////////////////////////////////////////////////////////////////////////
// Macros for Flip Flops
///////////////////////////////////////////////////////////////////////////////
// 说明：
//   下面的一组宏统一封装了"主-从"D 触发器的写法，便于在每个 RTL 模块中
//   直接以一行 `MSFF_*(q, d, clk, rst)` 完成寄存器例化。命名规则为：
//     ARN  -> Async  Reset, active-low      (异步低有效复位)
//     AR   -> Async  Reset, active-high     (异步高有效复位)
//     RN   -> Sync   Reset, active-low      (同步低有效复位)
//     R    -> Sync   Reset, active-high     (同步高有效复位)
//     RL   -> Sync   Reset 且 Reset 时载入指定初值 (rst_load)
//     无后缀 -> 不带任何复位
///////////////////////////////////////////////////////////////////////////////

// Master Slave flip-flop w/ Async_reset_n
`define MSFF_ARN(q, d, clk, rst_n) \
always @(posedge clk, negedge rst_n) begin \
    if (!rst_n) q <= 'b0; \
    else        q <= (d); \
end

// Master Slave flip-flop w/ Async_reset_p
`define MSFF_AR(q, d, clk, rst_p) \
always @(posedge clk, posedge rst_p) begin \
    if (rst_p)  q <= 'b0; \
    else        q <= (d); \
end

// Master Slave flip-flop w/ Sync_reset_n
`define MSFF_RN(q, d, clk, rst_n) \
always @(posedge clk) begin \
    if (!rst_n) q <= 'b0; \
    else        q <= (d); \
end

// Master Slave flip-flop w/ Sync_reset_p
`define MSFF_R(q, d, clk, rst_p) \
always @(posedge clk) begin \
    if (rst_p)  q <= 'b0; \
    else        q <= (d); \
end

// Master Slave flip-flop w/ Sync_reset_p and initial load value on reset
`define MSFF_RL(q, d, clk, rst_p, rst_load) \
always @(posedge clk) begin \
    if (rst_p)  q <= (rst_load); \
    else        q <= (d); \
end


// Master Slave flip-flop w/o reset
`define MSFF(q, d, clk) \
always @(posedge clk) begin \
    q <= (d); \
end

//------------------------------------------------------------------------------
// 函数 clog2 (向上取整)
// 此函数返回以 2 为底 x 的对数的整数 向上取整，即大于或等于 log2(x) 的最小整数。
//------------------------------------------------------------------------------
// coverage off
function integer clog2;
    input [31:0] value;
    reg [31:0] value_tmp;
    begin
        value_tmp = (value == 1) ? value : (value - 1);
        for (clog2 = 0; value_tmp > 0; clog2 = clog2 + 1) value_tmp = value_tmp >> 1;
    end
endfunction

