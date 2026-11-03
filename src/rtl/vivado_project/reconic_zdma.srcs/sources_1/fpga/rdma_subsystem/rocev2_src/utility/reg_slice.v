//  (c) Copyright 2010-2011, 2013-2014 Xilinx, Inc. All rights reserved.
//
//  This file contains confidential and proprietary information
//  of Xilinx, Inc. and is protected under U.S. and
//  international copyright and other intellectual property
//  laws.
//
//  DISCLAIMER
//  This disclaimer is not a license and does not grant any
//  rights to the materials distributed herewith. Except as
//  otherwise provided in a valid license issued to you by
//  Xilinx, and to the maximum extent permitted by applicable
//  law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
//  WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
//  AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
//  BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
//  INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
//  (2) Xilinx shall not be liable (whether in contract or tort,
//  including negligence, or under any other theory of
//  liability) for any loss or damage of any kind or nature
//  related to, arising under or in connection with these
//  materials, including for any direct, or any indirect,
//  special, incidental, or consequential loss or damage
//  (including loss of data, profits, goodwill, or any type of
//  loss or damage suffered as a result of any action brought
//  by a third party) even if such damage or loss was
//  reasonably foreseeable or Xilinx had been advised of the
//  possibility of the same.
//
//  CRITICAL APPLICATIONS
//  Xilinx products are not designed or intended to be fail-
//  safe, or for use in any application requiring fail-safe
//  performance, such as life-support or safety devices or
//  systems, Class III medical devices, nuclear facilities,
//  applications related to the deployment of airbags, or any
//  other applications that could lead to death, personal
//  injury, or severe property or environmental damage
//  (individually and collectively, "Critical
//  Applications"). Customer assumes the sole risk and
//  liability of any use of Xilinx products in Critical
//  Applications, subject only to applicable laws and
//  regulations governing limitations on product liability.
//
//  THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
//  PART OF THIS FILE AT ALL TIMES. 
//-----------------------------------------------------------------------------
//
// Register Slice
//   Generic single-channel AXI pipeline register on forward and/or reverse signal path
//
// Verilog-standard:  Verilog 2001
//--------------------------------------------------------------------------
//
// Structure:
//   axic_register_slice
//
//--------------------------------------------------------------------------
// 在芯片设计中遇到时序违例的问题时，通常的解决方式是插入寄存器（即打拍），这对一般的信号是有用的。
// 但是对于握手信号来说（例如AXI总线中的ready和valid信号），直接打拍会导致valid和ready不满足正确的握手关系，从而导致数据漏采样或重复采样。
// 因此，需要使用特殊的方法来对握手信号进行打拍，在长组合逻辑路径中插入寄存器提高频率（原始AXI总线由于信号传输路径过长（如valid/ready握手信号跨多个逻辑层次），会成为制约时钟频率提升的关键路径
// Register Slice将长路径切割为多个较短路径，每个段只需在一个时钟周期内完成时序收敛）。
// 对AXI总线中握手信号的打拍通常称为AXI register slice，可以降低关键路径的组合逻辑延迟，提高系统工作频率（因为信号传输路径的长度直接影响电路的时序性能，进而限制时钟频率的提升）。
// 
// register slice通常有四种模式：
//  1.Pass Through，直连，也就是不使用register slice。
//  2.Forward Registered(前向打拍) ：对valid和data信号打拍。
//  3.Backward Registered(后向打拍) ：对ready信号打拍。
//  4.Full Registered(双向打拍||双缓冲)：同时对valid，data信号和ready信号进行打拍。（该模块实现）

// Full Registered模式：
//  1.输入端和输出端的信号都经过寄存器缓存(双缓冲)，完全打断了前后级的组合逻辑路径，保证timing收敛，同时实现pipeline的高性能效果，但是会增加时延。
//    输入端：S_VALID, S_PAYLOAD_DATA → 经过寄存器 → M_VALID, M_PAYLOAD_DATA：输出端
//    反压信号：M_READY → 经过逻辑处理 → S_READY
//  2.
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module reg_slice #(
    parameter C_DATA_WIDTH = 32
) (
    // System Signals
    input wire aclk,
    input wire areset,
    input wire aclken,

    // Slave side
    input  wire [C_DATA_WIDTH-1:0] S_PAYLOAD_DATA,
    input  wire                    S_VALID,
    output wire                    S_READY,

    // Master side
    output wire [C_DATA_WIDTH-1:0] M_PAYLOAD_DATA,
    output wire                    M_VALID,
    input  wire                    M_READY
);

    ////////////////////////////////////////////////////////////////////
    //
    // C_REG_CONFIG = 8 (used to be 1 or 8)
    // Both FWD and REV mode
    //
    ////////////////////////////////////////////////////////////////////
    reg [1:0] state;
    localparam [1:0] ZERO = 2'b10,  // 空闲：无数据缓存
    ONE = 2'b11,  // 单缓存：storage_data1有数据
    TWO = 2'b01;  // 双缓存：两个存储都有数据

    // 双缓冲存储结构
    reg  [C_DATA_WIDTH-1:0] storage_data1;  // 主缓存，总是连接到输出端 M_PAYLOAD_DATA
    reg  [C_DATA_WIDTH-1:0] storage_data2;  // 辅助缓存，当 storage_data1 忙时，临时存储新来的数据
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
