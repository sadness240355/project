`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_tx_flow_ctrl_mgr — LRDMA 流控管理器，负责 TX 流量汇聚、仲裁与优先级流控（PFC）
//
// 【模块概述】
// 位于 LRDMA 引擎/DMA 引擎与 box250MHz 之间，承担三项核心职能：
// 双源包级仲裁、跨时钟域包级缓冲、IEEE 802.1Qbb 优先级流控。
//
// 【双源包级仲裁】（7 状态 FSM，运行在 m_axi_aclk 域）
// - IDLE                : 起始/空状态，按 LRDMA 优先选源（LRDMA 没数据再看 DMA）
// - RD_LRDMA_FIFO       : 正在送 LRDMA 包，持续读直到 tlast
// - RD_DMA_FIFO         : 正在送 DMA 包，持续读直到 tlast
// - CHK_CMAC_PROG_FULL_1: LRDMA 包尾时 CMAC 输出 FIFO 接近满，停下来等其排空
// - CHK_CMAC_PROG_FULL_2: DMA 包尾时同上
// - CHK_PFC_DMA         : 刚结束 LRDMA 包，下一拍 DMA 优先（与 IDLE 的 LRDMA 优先互补，避免单源饿死）
// - CHK_PFC_LRDMA       : 刚结束 DMA 包，下一拍 LRDMA 优先（同上）
// 关键优化：包尾当拍若 cmac_prog_full=0 且对端 valid，直接进入 RD_*_FIFO 或
//           CHK_PFC_*，跳过 CHK_CMAC_PROG_FULL_*，省 1 拍。
// 后端再接一级 CMAC Packet FIFO（深度 256，PACKET_FIFO 模式，整包就绪才被 box250MHz 读出）。
//
// 【IEEE 802.1Qbb PFC 流控】
// - 发送方向：RoCE RX 缓冲区满时，通过 ctl_tx_pause_req 向 CMAC 发送 PFC pause 帧
//   （使用 binary_to_onehot 函数将 4-bit 优先级转为 9-bit 独热码）
// - 接收方向：解析 stat_rx_pause_req，匹配 RoCE/非 RoCE 优先级后置位
//   stop_roce_tx / stop_non_roce_tx，仲裁 FSM 据此阻止相应源的调度
// - stat_rx_pause_req[8] 为全局 pause 位，置位时两方向都停止
// - disable_priority_chk_s 模式下任何 pause 都暂停发送（调试用）
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps


module lrdma_tx_flow_ctrl_mgr #(
    parameter C_ENABLE_LRDMA_TX_LOW_LATENCY_BYPASS = 1
) (
    input m_axi_aclk,
    input m_axi_aresetn,

    input cmac_clk,
    input cmac_rstn,

    // CMAC TX 路径 (m_axi_aclk 域)
    output [511 : 0] cmac_m_axis_tdata,
    output [   63:0] cmac_m_axis_tkeep,
    output           cmac_m_axis_tvalid,
    input            cmac_m_axis_tready,
    output           cmac_m_axis_tlast,
    output           cmac_m_axis_tuser,

    // QDMA H2C路径
    input          qdma_s_axis_tvalid,
    input  [511:0] qdma_s_axis_tdata,
    input  [ 63:0] qdma_s_axis_tkeep,
    input          qdma_s_axis_tlast,
    input  [  0:0] qdma_s_axis_tuser,
    output         qdma_s_axis_tready,

    // LRDMA TX路径
    input          lrdma_s_axis_tvalid,
    input  [511:0] lrdma_s_axis_tdata,
    input  [ 63:0] lrdma_s_axis_tkeep,
    input          lrdma_s_axis_tlast,
    output         lrdma_s_axis_tready,

    // CMAC IP核 PFC接口（cmac_clk 域）
    // RX
    // 其余信号: CTL_RX_PAUSE_ENABLE[8:0]、CTL_RX_PAUSE_ACK[8:0]在驱动onic_main.c已经配置好
    //  CTL_RX_PAUSE_ENABLE[8:0] = 0x1FF，（所有优先级 + 全局 pause 使能）
    //  CTL_RX_PAUSE_ACK[8:0] = 0x1FF，（告诉CMAC，用户逻辑已经接受这个 pause 请求）
    input  [8:0] stat_rx_pause_req,  // CMAC IP核已接收到暂停包
    // TX
    // 依赖 ctl_tx_pause_req 和 refresh timer 自动重发，不做手动强制重发
    output [8:0] ctl_tx_pause_req,   // 发送给 CMAC IP核，表示需要 CMAC IP核 发送暂停包

    // LRDMA PFC 配置 (一定要确保 PFC 配置只在初始化阶段设置，之后运行中不改，否则可能会出现有损采样)
    input [3:0] roce_priority_s,        // RoCE 优先级编号
    input       pfc_en_roce_s,          // RoCE PFC 使能
    input       pfc_en_non_roce_s,      // 非 RoCE PFC 使能
    input [3:0] non_roce_priority_s,    // 非 RoCE 优先级编号
    input       disable_priority_chk_s, // 禁用优先级检查（任意 pause 则停）

    // LRDMA RX 缓冲区满
    input roce_rx_buf_full_s
);
    // Local Parameters —— 仲裁 FSM 状态（共 7 态，包含 IDLE）
    localparam [2:0] IDLE = 3'b000;  // 起始状态：LRDMA 优先选源；都没数据就停在这里
    localparam [2:0] RD_LRDMA_FIFO = 3'b001;  // 正在透传 LRDMA 包，等 tlast
    localparam [2:0] CHK_CMAC_PROG_FULL_1 = 3'b010;  // LRDMA 包尾遇到 CMAC FIFO 接近满，等其排空
    localparam [2:0] CHK_PFC_DMA = 3'b011;  // 上一包是 LRDMA，这拍优先尝试 DMA（让另一路有机会发）
    localparam [2:0] RD_DMA_FIFO = 3'b100;  // 正在透传 DMA 包，等 tlast
    localparam [2:0] CHK_CMAC_PROG_FULL_2 = 3'b101;  // DMA 包尾遇到 CMAC FIFO 接近满，等其排空
    localparam [2:0] CHK_PFC_LRDMA = 3'b110;  // 上一包是 DMA，这拍优先尝试 LRDMA

    localparam [1:0] OUT_IDLE = 2'b00;
    localparam [1:0] OUT_LRDMA_BYPASS = 2'b01;
    localparam [1:0] OUT_CMAC_FIFO = 2'b10;

    wire         cmac_fifo_s_axis_tready;
    wire         cmac_fifo_m_axis_tready;
    wire [511:0] cmac_fifo_m_axis_tdata;
    wire [ 63:0] cmac_fifo_m_axis_tkeep;
    wire         cmac_fifo_m_axis_tlast;
    wire         cmac_fifo_m_axis_tuser;
    wire         cmac_fifo_m_axis_tvalid;

    wire         qdma_fifo_ready;
    wire         qdma_s_axis_tready_fifo;
    wire         qdma_s_axis_tvalid_fifo;
    reg          qdma_fifo_ready_out;
    wire [511:0] qdma_fifo_tdata_out;
    wire [ 63:0] qdma_fifo_tkeep_out;
    wire         qdma_fifo_tlast_out;
    wire         qdma_fifo_tvalid_out;
    wire [511:0] qdma_fifo_tdata;
    wire [ 63:0] qdma_fifo_tkeep;
    wire         qdma_fifo_tlast;
    wire         qdma_fifo_tvalid;
    wire [  3:0] roce_priority;
    wire         pfc_en_roce;
    wire [  4:0] pfc_configuration;
    wire         roce_rx_buf_full;

    wire         lrdma_fifo_ready;
    wire         lrdma_s_axis_tready_fifo;
    wire         lrdma_s_axis_tvalid_fifo;
    reg          lrdma_fifo_ready_out;
    wire [511:0] lrdma_fifo_tdata_out;
    wire [ 63:0] lrdma_fifo_tkeep_out;
    wire         lrdma_fifo_tlast_out;
    wire         lrdma_fifo_tvalid_out;
    wire [511:0] lrdma_fifo_tdata;
    wire [ 63:0] lrdma_fifo_tkeep;
    wire         lrdma_fifo_tlast;
    wire         lrdma_fifo_tvalid;

    wire [511:0] lrdma_reg_tdata_in;
    wire [ 63:0] lrdma_reg_tkeep_in;
    wire         lrdma_reg_tlast_in;
    wire         lrdma_reg_tvalid_in;
    wire         lrdma_reg_tready_in;

    wire [511:0] qdma_reg_tdata_in;
    wire [ 63:0] qdma_reg_tkeep_in;
    wire         qdma_reg_tlast_in;
    wire         qdma_reg_tvalid_in;
    wire         qdma_reg_tready_in;
    wire [576:0] qdma_reg_fifo_in;
    wire [576:0] lrdma_reg_fifo_in;

    wire [576:0] qdma_reg_out;
    wire [576:0] lrdma_reg_out;

    wire [  8:0] roce_pause;
    wire         stop_non_roce_tx;
    wire         stop_roce_tx;
    reg  [  2:0] state;

    wire [  8:0] rx_pause_req;
    wire         roce_priority_valid;
    wire         non_roce_priority_valid;

    wire         cmac_prog_full;
    wire         empty_qdma_fifo;
    wire         empty_lrdma_fifo;
    wire         low_latency_bypass_en;
    wire         bypass_lrdma_start_valid;
    wire         bypass_lrdma_axis_valid;
    wire [  1:0] out_sel;
    wire         out_sel_lrdma_bypass;
    wire         out_sel_cmac_fifo;
    wire         out_axis_tvalid;
    wire         out_axis_tlast;
    wire         out_axis_fire;
    reg  [  1:0] out_state;
    reg          out_prefer_cmac_fifo;

    assign low_latency_bypass_en     = (C_ENABLE_LRDMA_TX_LOW_LATENCY_BYPASS != 0);
    assign lrdma_s_axis_tvalid_fifo  = low_latency_bypass_en ? 1'b0 : lrdma_s_axis_tvalid;
    assign qdma_s_axis_tvalid_fifo   = qdma_s_axis_tvalid;
    // PFC only gates scheduling of a new LRDMA packet. Once a bypass packet
    // has been selected, keep forwarding until TLAST; otherwise a pause
    // request in the middle of a packet could accept an upstream beat without
    // presenting a downstream valid beat.
    assign bypass_lrdma_start_valid  = lrdma_s_axis_tvalid & ~stop_roce_tx;
    assign bypass_lrdma_axis_valid   = lrdma_s_axis_tvalid;
    assign out_sel                   = (out_state != OUT_IDLE) ? out_state :
                                       (out_prefer_cmac_fifo & cmac_fifo_m_axis_tvalid) ? OUT_CMAC_FIFO :
                                       bypass_lrdma_start_valid ? OUT_LRDMA_BYPASS :
                                       cmac_fifo_m_axis_tvalid ? OUT_CMAC_FIFO : OUT_IDLE;
    assign out_sel_lrdma_bypass      = low_latency_bypass_en & (out_sel == OUT_LRDMA_BYPASS);
    assign out_sel_cmac_fifo         = (~low_latency_bypass_en) | (out_sel == OUT_CMAC_FIFO);
    assign out_axis_tvalid           = out_sel_lrdma_bypass ? bypass_lrdma_axis_valid :
                                       out_sel_cmac_fifo ? cmac_fifo_m_axis_tvalid : 1'b0;
    assign out_axis_tlast            = out_sel_lrdma_bypass ? lrdma_s_axis_tlast :
                                       out_sel_cmac_fifo ? cmac_fifo_m_axis_tlast : 1'b0;
    assign out_axis_fire             = out_axis_tvalid & cmac_m_axis_tready;
    assign lrdma_s_axis_tready       = low_latency_bypass_en ? (out_sel_lrdma_bypass & cmac_m_axis_tready) : lrdma_s_axis_tready_fifo;
    assign qdma_s_axis_tready        = qdma_s_axis_tready_fifo;
    assign cmac_fifo_m_axis_tready   = out_sel_cmac_fifo & cmac_m_axis_tready;
    assign cmac_m_axis_tdata         = out_sel_lrdma_bypass ? lrdma_s_axis_tdata : cmac_fifo_m_axis_tdata;
    assign cmac_m_axis_tkeep         = out_sel_lrdma_bypass ? lrdma_s_axis_tkeep : cmac_fifo_m_axis_tkeep;
    assign cmac_m_axis_tlast         = out_sel_lrdma_bypass ? lrdma_s_axis_tlast : cmac_fifo_m_axis_tlast;
    assign cmac_m_axis_tuser         = out_sel_lrdma_bypass ? 1'b0 : cmac_fifo_m_axis_tuser;
    assign cmac_m_axis_tvalid        = low_latency_bypass_en ? out_axis_tvalid : cmac_fifo_m_axis_tvalid;

    // LRDMA PFC配置 CDC转换 -> cmac_clk 域
    xpm_cdc_array_single #(
        .DEST_SYNC_FF(4),  // DECIMAL; range: 2-10
        .INIT_SYNC_FF(0),  // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .SRC_INPUT_REG(1),  // DECIMAL; 0=do not register input, 1=register input
        .WIDTH(5)  // DECIMAL; range: 1-1024
    ) xpm_cdc_array_single_inst (
        .dest_out(pfc_configuration),                // WIDTH-bit output: src_in synchronized to the destination clock domain. This
        // output is registered.
        .dest_clk(cmac_clk),                         // 1-bit input: Clock signal for the destination clock domain.
        .src_clk (m_axi_aclk),                       // 1-bit input: optional; required when SRC_INPUT_REG = 1
        .src_in  ({roce_priority_s, pfc_en_roce_s})  // WIDTH-bit input: Input single-bit array to be synchronized to destination clock
        // 传输二进制值无损跨两个时钟域，改用 XPM_CDC_GRAY
    );
    assign roce_priority = pfc_configuration[4:1];
    assign pfc_en_roce   = pfc_configuration[0];

    // LRDMA RX缓冲区满信号 CDC转换 -> cmac_clk 域
    xpm_cdc_array_single #(
        .DEST_SYNC_FF(4),  // DECIMAL; range: 2-10
        .INIT_SYNC_FF(0),  // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .SRC_INPUT_REG(1),  // DECIMAL; 0=do not register input, 1=register input
        .WIDTH(1)  // DECIMAL; range: 1-1024
    ) xpm_cdc_array_single_inst_1 (
        .dest_out(roce_rx_buf_full),   // 1-bit output: synchronized to cmac_clk
        .dest_clk(cmac_clk),
        .src_clk (m_axi_aclk),
        .src_in  (roce_rx_buf_full_s)
    );

    xpm_cdc_array_single #(
        .DEST_SYNC_FF(4),  // DECIMAL; range: 2-10
        .INIT_SYNC_FF(0),  // DECIMAL; integer; 0=disable simulation init values, 1=enable simulation init
        // values
        .SIM_ASSERT_CHK(0),  // DECIMAL; integer; 0=disable simulation messages, 1=enable simulation messages
        .SRC_INPUT_REG(1),  // DECIMAL; 0=do not register input, 1=register input
        .WIDTH(9)  // DECIMAL; range: 1-1024
    ) xpm_cdc_array_single_inst_2 (
        .dest_out(rx_pause_req),      // WIDTH-bit output: src_in synchronized to the destination clock domain. This
        // output is registered.
        .dest_clk(m_axi_aclk),        // 1-bit input: Clock signal for the destination clock domain.
        .src_clk (cmac_clk),          // 1-bit input: optional; required when SRC_INPUT_REG = 1
        .src_in  (stat_rx_pause_req)  // WIDTH-bit input: Input single-bit array to be synchronized to destination clock
        // 传输二进制值无损跨两个时钟域，改用 XPM_CDC_GRAY
    );

    // xpm_fifo_axis: AXI Stream FIFO
    xpm_fifo_axis #(
        .CDC_SYNC_STAGES(2),  // DECIMAL
        .CLOCKING_MODE("common_clock"),  // String
        .ECC_MODE("no_ecc"),  // String
        .FIFO_DEPTH(256),  // DECIMAL
        .FIFO_MEMORY_TYPE("auto"),  // String
        .PACKET_FIFO("false"),  // String
        .PROG_EMPTY_THRESH(10),  // DECIMAL
        .PROG_FULL_THRESH(10),  // DECIMAL
        .RD_DATA_COUNT_WIDTH(9),  // DECIMAL
        .RELATED_CLOCKS(0),  // DECIMAL
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .TDATA_WIDTH(512),  // DECIMAL
        .TDEST_WIDTH(1),  // DECIMAL
        .TID_WIDTH(1),  // DECIMAL
        .TUSER_WIDTH(1),  // DECIMAL
        .USE_ADV_FEATURES("1F0F"),  // String
        .WR_DATA_COUNT_WIDTH(9)  // DECIMAL
    ) qdma_xpm_fifo_axis_inst (
        .almost_empty_axis (empty_qdma_fifo),     // 1-bit output: Almost Empty : When asserted, this signal
        // indicates that only one more read can be performed before the
        // FIFO goes to empty.
        .almost_full_axis  (),                    // 1-bit output: Almost Full: When asserted, this signal
        // indicates that only one more write can be performed before
        // the FIFO is full.
        .dbiterr_axis      (),                    // 1-bit output: Double Bit Error- Indicates that the ECC
        // decoder detected a double-bit error and data in the FIFO core
        // is corrupted.
        .m_axis_tdata      (qdma_fifo_tdata),     // TDATA_WIDTH-bit output: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .m_axis_tdest      (),                    // TDEST_WIDTH-bit output: TDEST: Provides routing information
        // for the data stream.
        .m_axis_tid        (),                    // TID_WIDTH-bit output: TID: The data stream identifier that
        // indicates different streams of data.
        .m_axis_tkeep      (qdma_fifo_tkeep),     // TDATA_WIDTH/8-bit output: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // TKEEP 为 0 的字节视为空字节，可从数据流移除；AXI-Stream 数据宽度为 512 bit，TKEEP[63:0] 对应 64 个字节。
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .m_axis_tlast      (qdma_fifo_tlast),     // 1-bit output: TLAST: Indicates the boundary of a packet.
        .m_axis_tstrb      (),                    // TDATA_WIDTH/8-bit output: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .m_axis_tuser      (),                    // TUSER_WIDTH-bit output: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .m_axis_tvalid     (qdma_fifo_tvalid),    // 1-bit output: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
        .prog_empty_axis   (),                    // 1-bit output: Programmable Empty- This signal is asserted
        // when the number of words in the FIFO is less than or equal to
        // the programmable empty threshold value. It is de-asserted
        // when the number of words in the FIFO exceeds the programmable
        // empty threshold value.
        .prog_full_axis    (),                    // 1-bit output: Programmable Full: This signal is asserted when
        // the number of words in the FIFO is greater than or equal to
        // the programmable full threshold value. It is de-asserted when
        // the number of words in the FIFO is less than the programmable
        // full threshold value.
        .rd_data_count_axis(),                    // RD_DATA_COUNT_WIDTH-bit output: Read Data Count- This bus
        // indicates the number of words available for reading in the
        // FIFO.
        .s_axis_tready     (qdma_reg_tready_in),  // 1-bit output: TREADY: Indicates that the slave can accept a
        // 本周期可接受一次传输。
        .sbiterr_axis      (),                    // 1-bit output: Single Bit Error- Indicates that the ECC
        // decoder detected and fixed a single-bit error.
        .wr_data_count_axis(),                    // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus
        // indicates the number of words written into the FIFO.
        .injectdbiterr_axis(1'b0),                // 1-bit input: Double Bit Error Injection- Injects a double bit
        // error if the ECC feature is used.
        .injectsbiterr_axis(1'b0),                // 1-bit input: Single Bit Error Injection- Injects a single bit
        // error if the ECC feature is used.
        .m_aclk            (m_axi_aclk),          // 1-bit input: Master Interface Clock: All signals on master
        // interface are sampled on the rising edge of this clock.
        .m_axis_tready     (qdma_fifo_ready),     // 1-bit input: TREADY: Indicates that the slave can accept a
        // 本周期可接受一次传输。
        .s_aclk            (m_axi_aclk),          // 1-bit input: Slave Interface Clock: All signals on slave
        // interface are sampled on the rising edge of this clock.
        .s_aresetn         (m_axi_aresetn),       // 1-bit input: Active low asynchronous reset.
        .s_axis_tdata      (qdma_reg_tdata_in),   // TDATA_WIDTH-bit input: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .s_axis_tdest      (1'b0),                // TDEST_WIDTH-bit input: TDEST: Provides routing information
        // for the data stream.
        .s_axis_tid        (1'b0),                // TID_WIDTH-bit input: TID: The data stream identifier that
        // indicates different streams of data.
        .s_axis_tkeep      (qdma_reg_tkeep_in),   // TDATA_WIDTH/8-bit input: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // TKEEP 为 0 的字节视为空字节，可从数据流移除；AXI-Stream 数据宽度为 512 bit，TKEEP[63:0] 对应 64 个字节。
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .s_axis_tlast      (qdma_reg_tlast_in),   // 1-bit input: TLAST: Indicates the boundary of a packet.
        .s_axis_tstrb      ({64{1'b1}}),          // TDATA_WIDTH/8-bit input: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .s_axis_tuser      (1'b0),                // TUSER_WIDTH-bit input: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .s_axis_tvalid     (qdma_reg_tvalid_in)   // 1-bit input: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
    );
    // End of xpm_fifo_axis_inst instantiation

    // xpm_fifo_axis: AXI Stream FIFO
    // m_axi_aclk 域 -> cmac_clk 域
    xpm_fifo_axis #(
        .CDC_SYNC_STAGES(2),  // DECIMAL
        .CLOCKING_MODE("common_clock"),  // String
        .ECC_MODE("no_ecc"),  // String
        .FIFO_DEPTH(256),  // DECIMAL
        .FIFO_MEMORY_TYPE("auto"),  // String
        .PACKET_FIFO("false"),  // String
        .PROG_EMPTY_THRESH(10),  // DECIMAL
        .PROG_FULL_THRESH(10),  // DECIMAL
        .RD_DATA_COUNT_WIDTH(9),  // DECIMAL
        .RELATED_CLOCKS(0),  // DECIMAL
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .TDATA_WIDTH(512),  // DECIMAL
        .TDEST_WIDTH(1),  // DECIMAL
        .TID_WIDTH(1),  // DECIMAL
        .TUSER_WIDTH(1),  // DECIMAL
        .USE_ADV_FEATURES("1F0F"),  // String
        .WR_DATA_COUNT_WIDTH(9)  // DECIMAL
    ) lrdma_xpm_fifo_axis_inst (
        .almost_empty_axis (empty_lrdma_fifo),     // 1-bit output: Almost Empty : When asserted, this signal
        // indicates that only one more read can be performed before the
        // FIFO goes to empty.
        .almost_full_axis  (),                     // 1-bit output: Almost Full: When asserted, this signal
        // indicates that only one more write can be performed before
        // the FIFO is full.
        .dbiterr_axis      (),                     // 1-bit output: Double Bit Error- Indicates that the ECC
        // decoder detected a double-bit error and data in the FIFO core
        // is corrupted.
        .m_axis_tdata      (lrdma_fifo_tdata),     // TDATA_WIDTH-bit output: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .m_axis_tdest      (),                     // TDEST_WIDTH-bit output: TDEST: Provides routing information
        // for the data stream.
        .m_axis_tid        (),                     // TID_WIDTH-bit output: TID: The data stream identifier that
        // indicates different streams of data.
        .m_axis_tkeep      (lrdma_fifo_tkeep),     // TDATA_WIDTH/8-bit output: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // TKEEP 为 0 的字节视为空字节，可从数据流移除；AXI-Stream 数据宽度为 512 bit，TKEEP[63:0] 对应 64 个字节。
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .m_axis_tlast      (lrdma_fifo_tlast),     // 1-bit output: TLAST: Indicates the boundary of a packet.
        .m_axis_tstrb      (),                     // TDATA_WIDTH/8-bit output: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .m_axis_tuser      (),                     // TUSER_WIDTH-bit output: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .m_axis_tvalid     (lrdma_fifo_tvalid),    // 1-bit output: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
        .prog_empty_axis   (),                     // 1-bit output: Programmable Empty- This signal is asserted
        // when the number of words in the FIFO is less than or equal to
        // the programmable empty threshold value. It is de-asserted
        // when the number of words in the FIFO exceeds the programmable
        // empty threshold value.
        .prog_full_axis    (),                     // 1-bit output: Programmable Full: This signal is asserted when
        // the number of words in the FIFO is greater than or equal to
        // the programmable full threshold value. It is de-asserted when
        // the number of words in the FIFO is less than the programmable
        // full threshold value.
        .rd_data_count_axis(),                     // RD_DATA_COUNT_WIDTH-bit output: Read Data Count- This bus
        // indicates the number of words available for reading in the
        // FIFO.
        .s_axis_tready     (lrdma_reg_tready_in),  // 1-bit output: TREADY: Indicates that the slave can accept a
        // 本周期可接受一次传输。
        .sbiterr_axis      (),                     // 1-bit output: Single Bit Error- Indicates that the ECC
        // decoder detected and fixed a single-bit error.
        .wr_data_count_axis(),                     // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus
        // indicates the number of words written into the FIFO.
        .injectdbiterr_axis(1'b0),                 // 1-bit input: Double Bit Error Injection- Injects a double bit
        // error if the ECC feature is used.
        .injectsbiterr_axis(1'b0),                 // 1-bit input: Single Bit Error Injection- Injects a single bit
        // error if the ECC feature is used.
        .m_aclk            (m_axi_aclk),           // 1-bit input: Master Interface Clock: All signals on master
        // interface are sampled on the rising edge of this clock.
        .m_axis_tready     (lrdma_fifo_ready),
        // 1-bit input: TREADY: Indicates that the slave can accept a
        // 本周期可接受一次传输。
        .s_aclk            (m_axi_aclk),           // 1-bit input: Slave Interface Clock: All signals on slave
        // interface are sampled on the rising edge of this clock.
        .s_aresetn         (m_axi_aresetn),        // 1-bit input: Active low asynchronous reset.
        .s_axis_tdata      (lrdma_reg_tdata_in),   // TDATA_WIDTH-bit input: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .s_axis_tdest      (1'b0),                 // TDEST_WIDTH-bit input: TDEST: Provides routing information
        // for the data stream.
        .s_axis_tid        (1'b0),                 // TID_WIDTH-bit input: TID: The data stream identifier that
        // indicates different streams of data.
        .s_axis_tkeep      (lrdma_reg_tkeep_in),   // TDATA_WIDTH/8-bit input: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // TKEEP 为 0 的字节视为空字节，可从数据流移除；AXI-Stream 数据宽度为 512 bit，TKEEP[63:0] 对应 64 个字节。
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .s_axis_tlast      (lrdma_reg_tlast_in),   // 1-bit input: TLAST: Indicates the boundary of a packet.
        .s_axis_tstrb      ({64{1'b1}}),           // TDATA_WIDTH/8-bit input: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .s_axis_tuser      (1'b0),                 // TUSER_WIDTH-bit input: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .s_axis_tvalid     (lrdma_reg_tvalid_in)   // 1-bit input: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
    );



    // End of xpm_fifo_axis_inst instantiation
    lrdma_common_reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_lrdma_in_inst (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({lrdma_s_axis_tlast, lrdma_s_axis_tkeep, lrdma_s_axis_tdata}),
        .S_VALID       (lrdma_s_axis_tvalid_fifo),
        .S_READY       (lrdma_s_axis_tready_fifo),
        .M_PAYLOAD_DATA(lrdma_reg_fifo_in),
        .M_VALID       (lrdma_reg_tvalid_in),
        .M_READY       (lrdma_reg_tready_in)
    );
    assign lrdma_reg_tdata_in = lrdma_reg_fifo_in[511:0];
    assign lrdma_reg_tkeep_in = lrdma_reg_fifo_in[575:512];
    assign lrdma_reg_tlast_in = lrdma_reg_fifo_in[576];

    lrdma_common_reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_lrdma_inst (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({lrdma_fifo_tlast, lrdma_fifo_tkeep, lrdma_fifo_tdata}),
        .S_VALID       (lrdma_fifo_tvalid),
        .S_READY       (lrdma_fifo_ready),
        .M_PAYLOAD_DATA(lrdma_reg_out),
        .M_VALID       (lrdma_fifo_tvalid_out),
        .M_READY       (lrdma_fifo_ready_out & cmac_fifo_s_axis_tready)
    );
    assign lrdma_fifo_tdata_out = lrdma_reg_out[511:0];
    assign lrdma_fifo_tkeep_out = lrdma_reg_out[575:512];
    assign lrdma_fifo_tlast_out = lrdma_reg_out[576];



    lrdma_common_reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_qdma_in_inst (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({qdma_s_axis_tlast, qdma_s_axis_tkeep, qdma_s_axis_tdata}),
        .S_VALID       (qdma_s_axis_tvalid_fifo),
        .S_READY       (qdma_s_axis_tready_fifo),
        .M_PAYLOAD_DATA(qdma_reg_fifo_in),
        .M_VALID       (qdma_reg_tvalid_in),
        .M_READY       (qdma_reg_tready_in)
    );
    assign qdma_reg_tdata_in = qdma_reg_fifo_in[511:0];
    assign qdma_reg_tkeep_in = qdma_reg_fifo_in[575:512];
    assign qdma_reg_tlast_in = qdma_reg_fifo_in[576];

    lrdma_common_reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_qdma_inst (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({qdma_fifo_tlast, qdma_fifo_tkeep, qdma_fifo_tdata}),
        .S_VALID       (qdma_fifo_tvalid),
        .S_READY       (qdma_fifo_ready),
        .M_PAYLOAD_DATA(qdma_reg_out),
        .M_VALID       (qdma_fifo_tvalid_out),
        .M_READY       (qdma_fifo_ready_out & cmac_fifo_s_axis_tready)
    );
    assign qdma_fifo_tdata_out = qdma_reg_out[511:0];
    assign qdma_fifo_tkeep_out = qdma_reg_out[575:512];
    assign qdma_fifo_tlast_out = qdma_reg_out[576];




    // xpm_fifo_axis: AXI Stream FIFO
    // Original Vendor Parameterized Macro, version 2020.1
    xpm_fifo_axis #(
        .CDC_SYNC_STAGES(2),  // DECIMAL
        .CLOCKING_MODE("common_clock"),  // String
        .ECC_MODE("no_ecc"),  // String
        .FIFO_DEPTH(256),  // DECIMAL
        .FIFO_MEMORY_TYPE("auto"),  // String
        .PACKET_FIFO("true"),  // String
        .PROG_EMPTY_THRESH(10),  // DECIMAL
        .PROG_FULL_THRESH(256 - 66),  // DECIMAL
        .RD_DATA_COUNT_WIDTH(9),  // DECIMAL
        .RELATED_CLOCKS(0),  // DECIMAL
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .TDATA_WIDTH(512),  // DECIMAL
        .TDEST_WIDTH(1),  // DECIMAL
        .TID_WIDTH(1),  // DECIMAL
        .TUSER_WIDTH(1),  // DECIMAL
        .USE_ADV_FEATURES("1F1F"),  // String
        .WR_DATA_COUNT_WIDTH(9)  // DECIMAL
    ) cmac_xpm_fifo_axis_inst (
        .almost_empty_axis (),                                                                                                 // 1-bit output: Almost Empty : When asserted, this signal
        // indicates that only one more read can be performed before the
        // FIFO goes to empty.
        .almost_full_axis  (),                                                                                                 // 1-bit output: Almost Full: When asserted, this signal
        // indicates that only one more write can be performed before
        // the FIFO is full.
        .dbiterr_axis      (),                                                                                                 // 1-bit output: Double Bit Error- Indicates that the ECC
        // decoder detected a double-bit error and data in the FIFO core
        // is corrupted.
        .m_axis_tdata      (cmac_fifo_m_axis_tdata),                                                                           // TDATA_WIDTH-bit output: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .m_axis_tdest      (),                                                                                                 // TDEST_WIDTH-bit output: TDEST: Provides routing information
        // for the data stream.
        .m_axis_tid        (),                                                                                                 // TID_WIDTH-bit output: TID: The data stream identifier that
        // indicates different streams of data.
        .m_axis_tkeep      (cmac_fifo_m_axis_tkeep),                                                                           // TDATA_WIDTH/8-bit output: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // TKEEP 为 0 的字节视为空字节，可从数据流移除；AXI-Stream 数据宽度为 512 bit，TKEEP[63:0] 对应 64 个字节。
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .m_axis_tlast      (cmac_fifo_m_axis_tlast),                                                                           // 1-bit output: TLAST: Indicates the boundary of a packet.
        .m_axis_tstrb      (),                                                                                                 // TDATA_WIDTH/8-bit output: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .m_axis_tuser      (cmac_fifo_m_axis_tuser),                                                                           // TUSER_WIDTH-bit output: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .m_axis_tvalid     (cmac_fifo_m_axis_tvalid),                                                                          // 1-bit output: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
        .prog_empty_axis   (),                                                                                                 // 1-bit output: Programmable Empty- This signal is asserted
        // when the number of words in the FIFO is less than or equal to
        // the programmable empty threshold value. It is de-asserted
        // when the number of words in the FIFO exceeds the programmable
        // empty threshold value.
        .prog_full_axis    (cmac_prog_full),                                                                                   // 1-bit output: Programmable Full: This signal is asserted when
        // the number of words in the FIFO is greater than or equal to
        // the programmable full threshold value. It is de-asserted when
        // the number of words in the FIFO is less than the programmable
        // full threshold value.
        .rd_data_count_axis(),                                                                                                 // RD_DATA_COUNT_WIDTH-bit output: Read Data Count- This bus
        // indicates the number of words available for reading in the
        // FIFO.
        .s_axis_tready     (cmac_fifo_s_axis_tready),                                                                          // 1-bit output: TREADY: Indicates that the slave can accept a
        // 本周期可接受一次传输。
        .sbiterr_axis      (),                                                                                                 // 1-bit output: Single Bit Error- Indicates that the ECC
        // decoder detected and fixed a single-bit error.
        .wr_data_count_axis(),                                                                                                 // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus
        // indicates the number of words written into the FIFO.
        .injectdbiterr_axis(1'b0),                                                                                             // 1-bit input: Double Bit Error Injection- Injects a double bit
        // error if the ECC feature is used.
        .injectsbiterr_axis(1'b0),                                                                                             // 1-bit input: Single Bit Error Injection- Injects a single bit
        // error if the ECC feature is used.
        .m_aclk            (m_axi_aclk),                                                                                       // 1-bit input: Master Interface Clock: All signals on master
        // interface are sampled on the rising edge of this clock.
        .m_axis_tready     (cmac_fifo_m_axis_tready),                                                                          // 1-bit input: TREADY: Indicates that the slave can accept a
        // 本周期可接受一次传输。
        .s_aclk            (m_axi_aclk),                                                                                       // 1-bit input: Slave Interface Clock: All signals on slave
        // interface are sampled on the rising edge of this clock.
        .s_aresetn         (m_axi_aresetn),                                                                                    // 1-bit input: Active low asynchronous reset.
        .s_axis_tdata      (qdma_fifo_ready_out ? qdma_fifo_tdata_out : lrdma_fifo_tdata_out),                                 // TDATA_WIDTH-bit input: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .s_axis_tdest      (1'b0),                                                                                             // TDEST_WIDTH-bit input: TDEST: Provides routing information
        // for the data stream.
        .s_axis_tid        (1'b0),                                                                                             // TID_WIDTH-bit input: TID: The data stream identifier that
        // indicates different streams of data.
        .s_axis_tkeep      (qdma_fifo_ready_out ? qdma_fifo_tkeep_out : lrdma_fifo_tkeep_out),                                 // TDATA_WIDTH/8-bit input: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // TKEEP 为 0 的字节视为空字节，可从数据流移除；AXI-Stream 数据宽度为 512 bit，TKEEP[63:0] 对应 64 个字节。
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .s_axis_tlast      (qdma_fifo_ready_out ? qdma_fifo_tlast_out : lrdma_fifo_tlast_out),                                 // 1-bit input: TLAST: Indicates the boundary of a packet.
        .s_axis_tstrb      ({64{1'b1}}),                                                                                       // TDATA_WIDTH/8-bit input: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .s_axis_tuser      (1'b0),                                                                                             // TUSER 固定为 0
        // information that can be transmitted alongside the data
        // stream.
        .s_axis_tvalid     (qdma_fifo_ready_out ? qdma_fifo_tvalid_out : lrdma_fifo_ready_out ? lrdma_fifo_tvalid_out : 1'b0)  // 1-bit input: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
    );
    // End of xpm_fifo_axis_inst instantiation 

    // Packet-level output selection for low-latency LRDMA bypass mode.
    // Once a source is selected, hold it until TLAST to avoid QDMA/LRDMA beat interleaving.
    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            out_state            <= OUT_IDLE;
            out_prefer_cmac_fifo <= 1'b0;
        end else if (!low_latency_bypass_en) begin
            out_state            <= OUT_IDLE;
            out_prefer_cmac_fifo <= 1'b0;
        end else begin
            case (out_state)
                OUT_IDLE: begin
                    if (out_sel != OUT_IDLE) begin
                        if (out_axis_fire && out_axis_tlast) begin
                            out_state            <= OUT_IDLE;
                            out_prefer_cmac_fifo <= (out_sel == OUT_LRDMA_BYPASS);
                        end else begin
                            out_state <= out_sel;
                        end
                    end
                end

                OUT_LRDMA_BYPASS: begin
                    if (out_axis_fire && out_axis_tlast) begin
                        out_state            <= OUT_IDLE;
                        out_prefer_cmac_fifo <= 1'b1;
                    end
                end

                OUT_CMAC_FIFO: begin
                    if (out_axis_fire && out_axis_tlast) begin
                        out_state            <= OUT_IDLE;
                        out_prefer_cmac_fifo <= 1'b0;
                    end
                end

                default: begin
                    out_state            <= OUT_IDLE;
                    out_prefer_cmac_fifo <= 1'b0;
                end
            endcase
        end
    end

    // Arbiter
    // ------------------------------------------------------------------
    // 包级仲裁 FSM（运行在 m_axi_aclk 域）
    // ------------------------------------------------------------------
    // 仅在两侧 CDC FIFO 的输出端（lrdma_common_reg_slice 之后）做选择，源切换以
    // tlast 为边界，确保不会把两路数据交织到 CMAC 输入侧。
    //
    // 状态语义：
    //   IDLE                : 空闲，按“LRDMA 优先”策略选源
    //   RD_LRDMA_FIFO       : 正在送 LRDMA 包，遇到 tlast 切换
    //   RD_DMA_FIFO         : 正在送 DMA 包，遇到 tlast 切换
    //   CHK_CMAC_PROG_FULL_1: 上一个 LRDMA 包尾时 CMAC 输出 FIFO 接近满，
    //                          停下来等其排空
    //   CHK_CMAC_PROG_FULL_2: 同上，DMA 包尾时使用
    //   CHK_PFC_DMA         : 刚结束 LRDMA 包，下一拍优先看 DMA（公平）
    //   CHK_PFC_LRDMA       : 刚结束 DMA 包，下一拍优先看 LRDMA（公平）
    //
    // 关键优化：包结束当拍若 cmac_prog_full=0 且对端有数据，可零延迟切换，
    //           避免每包都额外消耗 1 拍走 CHK_CMAC_PROG_FULL_*。
    // ------------------------------------------------------------------
    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            state                <= IDLE;
            qdma_fifo_ready_out  <= 1'b0;
            lrdma_fifo_ready_out <= 1'b0;
        end else begin
            case (state)
                IDLE:

                if (lrdma_fifo_tvalid_out && !stop_roce_tx) begin
                    state                <= RD_LRDMA_FIFO;
                    lrdma_fifo_ready_out <= 1'b1;
                end else if (qdma_fifo_tvalid_out && !stop_non_roce_tx) begin
                    state               <= RD_DMA_FIFO;
                    qdma_fifo_ready_out <= 1'b1;
                end else begin
                    state                <= IDLE;
                    lrdma_fifo_ready_out <= 1'b0;
                    qdma_fifo_ready_out  <= 1'b0;
                end

                RD_LRDMA_FIFO:
                // (tvalid & tlast 同拍反映同一 beat, 联合判断符合 AXI-Stream)
                if (lrdma_fifo_tvalid_out && lrdma_fifo_tlast_out && cmac_fifo_s_axis_tready) begin
                    // 优化：跳过 CHK_CMAC_PROG_FULL_1（同拍已检查），直接到 CHK_PFC_DMA，省 1 拍
                    if (cmac_prog_full) begin
                        state                <= CHK_CMAC_PROG_FULL_1;
                        lrdma_fifo_ready_out <= 1'b0;
                        qdma_fifo_ready_out  <= 1'b0;
                    end else if (qdma_fifo_tvalid_out && !stop_non_roce_tx) begin
                        // 跨源切换（LRDMA→DMA）：dma_fifo_tvalid_out 独立于已选输出 FIFO
                        state                <= RD_DMA_FIFO;
                        lrdma_fifo_ready_out <= 1'b0;
                        qdma_fifo_ready_out  <= 1'b1;
                    end else begin
                        // LRDMA 自继续或无数据：走 CHK_PFC_DMA（下一拍 FIFO 输出已更新，判断可靠）
                        state                <= CHK_PFC_DMA;
                        lrdma_fifo_ready_out <= 1'b0;
                        qdma_fifo_ready_out  <= 1'b0;
                    end

                end else begin
                    state <= RD_LRDMA_FIFO;
                end

                CHK_CMAC_PROG_FULL_1:
                if (!cmac_prog_full) begin
                    state <= CHK_PFC_DMA;
                end else begin
                    state <= CHK_CMAC_PROG_FULL_1;
                end

                CHK_PFC_DMA:
                if (qdma_fifo_tvalid_out && !stop_non_roce_tx) begin
                    state               <= RD_DMA_FIFO;
                    qdma_fifo_ready_out <= 1'b1;
                end else if (lrdma_fifo_tvalid_out && !stop_roce_tx) begin
                    state                <= RD_LRDMA_FIFO;
                    lrdma_fifo_ready_out <= 1'b1;
                end else begin
                    state <= IDLE;
                end

                RD_DMA_FIFO:
                if (qdma_fifo_tvalid_out && qdma_fifo_tlast_out && cmac_fifo_s_axis_tready) begin
                    // 优化：同 RD_LRDMA_FIFO，跳过 CHK_CMAC_PROG_FULL_2，省 1 拍
                    if (cmac_prog_full) begin
                        state                <= CHK_CMAC_PROG_FULL_2;
                        qdma_fifo_ready_out  <= 1'b0;
                        lrdma_fifo_ready_out <= 1'b0;
                    end else if (lrdma_fifo_tvalid_out && !stop_roce_tx) begin
                        // 跨源切换（DMA→LRDMA）：lrdma_fifo_tvalid_out 独立于 DMA FIFO，可靠
                        state                <= RD_LRDMA_FIFO;
                        lrdma_fifo_ready_out <= 1'b1;
                        qdma_fifo_ready_out  <= 1'b0;
                    end else begin
                        state                <= CHK_PFC_LRDMA;
                        qdma_fifo_ready_out  <= 1'b0;
                        lrdma_fifo_ready_out <= 1'b0;
                    end

                end else begin
                    state <= RD_DMA_FIFO;
                end

                CHK_CMAC_PROG_FULL_2:
                if (!cmac_prog_full) begin
                    state <= CHK_PFC_LRDMA;
                end else begin
                    state <= CHK_CMAC_PROG_FULL_2;
                end

                CHK_PFC_LRDMA:
                if (lrdma_fifo_tvalid_out && !stop_roce_tx) begin
                    state                <= RD_LRDMA_FIFO;
                    lrdma_fifo_ready_out <= 1'b1;
                end else if (qdma_fifo_tvalid_out && !stop_non_roce_tx) begin
                    state               <= RD_DMA_FIFO;
                    qdma_fifo_ready_out <= 1'b1;
                end else begin
                    state <= IDLE;
                end

                default: begin
                    state                <= IDLE;
                    lrdma_fifo_ready_out <= 1'b0;
                    qdma_fifo_ready_out  <= 1'b0;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------
    // PFC（IEEE 802.1Qbb）发送/接收方向流控生成
    // ------------------------------------------------------------------
    // binary_to_onehot：把优先级编号转成 9-bit one-hot —— 当 binary_no == i 时
    //   binary_to_onehot[i] = 1，i ∈ 0..8。正常 RoCE/非 RoCE 优先级在 0..7 内，
    //   会落到对应位；CMAC PFC 接口的 bit[8] 在协议层是“全局 pause”位，本函数
    //   遵循同一编号约定（priority=8 时输出 bit[8]=1）。binary_no ≥ 9 全 0。
    // ------------------------------------------------------------------
    // define binary to one hot function
    function [8:0] binary_to_onehot;
        input [3:0] binary_no;
        reg [8:0] i;
        begin
            for (i = 0; i < 9; i = i + 1) begin
                binary_to_onehot[i] = (binary_no == i);
            end
        end
    endfunction

    // 发送方向：本端 RoCE RX 缓冲快满时，向 CMAC 请求发出 PFC pause 帧。(这些信号均工作在 cmac_clk 域)
    //   - 必须先打开 pfc_en_roce 才生效；
    //   - 仅暂停 RoCE 优先级，不影响其它流量；
    assign roce_pause       = (pfc_en_roce) ? (roce_rx_buf_full) ? binary_to_onehot(roce_priority) : 9'b000000000 : 9'b000000000;
    assign ctl_tx_pause_req = roce_pause;

    // 接收方向：解析对端发来的 PFC pause 请求，决定本端是否暂停发送。(这些信号均工作在 m_axi_aclk 域)
    //   - bit[8]=1 表示所有优先级都停；
    //   - disable_priority_chk_s=1 时只要任一优先级被 pause 就停（调试用）；
    //   - 否则只看本端配置的对应优先级位。
    // priority 配置寄存器是 4 bit，但 stat_rx_pause_req 只有 [8:0]。
    // 若软件误写 9~15，直接动态索引会在仿真中产生 X，并可能让 TX 停发判断不确定；
    // 与 binary_to_onehot 的策略一致，非法 priority 视为没有命中任何 pause 位。
    assign roce_priority_valid     = (roce_priority_s < 4'd9);
    assign non_roce_priority_valid = (non_roce_priority_s < 4'd9);
    assign stop_roce_tx            = (pfc_en_roce_s) ? (rx_pause_req[8] == 1'b1) ? 1'b1 : (disable_priority_chk_s) ? (|rx_pause_req[7:0]) : (roce_priority_valid ? rx_pause_req[roce_priority_s] : 1'b0) : 1'b0;
    assign stop_non_roce_tx        = (pfc_en_non_roce_s) ? (rx_pause_req[8] == 1'b1) ? 1'b1 : (disable_priority_chk_s) ? (|rx_pause_req[7:0]) : (non_roce_priority_valid ? rx_pause_req[non_roce_priority_s] : 1'b0) : 1'b0;

endmodule
