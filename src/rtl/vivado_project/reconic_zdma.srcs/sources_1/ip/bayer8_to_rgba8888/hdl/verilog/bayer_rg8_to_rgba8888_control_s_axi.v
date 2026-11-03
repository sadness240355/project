// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.2 (64-bit)
// Tool Version Limit: 2019.12
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// ==============================================================
`timescale 1ns/1ps
module bayer_rg8_to_rgba8888_control_s_axi
#(parameter
    C_S_AXI_ADDR_WIDTH = 8,
    C_S_AXI_DATA_WIDTH = 32
)(
    input  wire                          ACLK,
    input  wire                          ARESET,
    input  wire                          ACLK_EN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] AWADDR,
    input  wire                          AWVALID,
    output wire                          AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0] WDATA,
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0] WSTRB,
    input  wire                          WVALID,
    output wire                          WREADY,
    output wire [1:0]                    BRESP,
    output wire                          BVALID,
    input  wire                          BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] ARADDR,
    input  wire                          ARVALID,
    output wire                          ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0] RDATA,
    output wire [1:0]                    RRESP,
    output wire                          RVALID,
    input  wire                          RREADY,
    output wire                          interrupt,
    output wire [15:0]                   width,
    output wire [15:0]                   height,
    output wire [3:0]                    isp_enable,
    output wire [7:0]                    black_level,
    output wire [15:0]                   wb_gain_r,
    output wire [15:0]                   wb_gain_g,
    output wire [15:0]                   wb_gain_b,
    output wire [15:0]                   ccm_00,
    output wire [15:0]                   ccm_01,
    output wire [15:0]                   ccm_02,
    output wire [15:0]                   ccm_10,
    output wire [15:0]                   ccm_11,
    output wire [15:0]                   ccm_12,
    output wire [15:0]                   ccm_20,
    output wire [15:0]                   ccm_21,
    output wire [15:0]                   ccm_22,
    output wire                          ap_start,
    input  wire                          ap_done,
    input  wire                          ap_ready,
    input  wire                          ap_idle
);
//------------------------Address Info-------------------
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read/COR)
//        bit 7  - auto_restart (Read/Write)
//        bit 9  - interrupt (Read)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0 - enable ap_done interrupt (Read/Write)
//        bit 1 - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0 - ap_done (Read/TOW)
//        bit 1 - ap_ready (Read/TOW)
//        others - reserved
// 0x10 : Data signal of width
//        bit 15~0 - width[15:0] (Read/Write)
//        others   - reserved
// 0x14 : reserved
// 0x18 : Data signal of height
//        bit 15~0 - height[15:0] (Read/Write)
//        others   - reserved
// 0x1c : reserved
// 0x20 : Data signal of isp_enable
//        bit 3~0 - isp_enable[3:0] (Read/Write)
//        others  - reserved
// 0x24 : reserved
// 0x28 : Data signal of black_level
//        bit 7~0 - black_level[7:0] (Read/Write)
//        others  - reserved
// 0x2c : reserved
// 0x30 : Data signal of wb_gain_r
//        bit 15~0 - wb_gain_r[15:0] (Read/Write)
//        others   - reserved
// 0x34 : reserved
// 0x38 : Data signal of wb_gain_g
//        bit 15~0 - wb_gain_g[15:0] (Read/Write)
//        others   - reserved
// 0x3c : reserved
// 0x40 : Data signal of wb_gain_b
//        bit 15~0 - wb_gain_b[15:0] (Read/Write)
//        others   - reserved
// 0x44 : reserved
// 0x48 : Data signal of ccm_00
//        bit 15~0 - ccm_00[15:0] (Read/Write)
//        others   - reserved
// 0x4c : reserved
// 0x50 : Data signal of ccm_01
//        bit 15~0 - ccm_01[15:0] (Read/Write)
//        others   - reserved
// 0x54 : reserved
// 0x58 : Data signal of ccm_02
//        bit 15~0 - ccm_02[15:0] (Read/Write)
//        others   - reserved
// 0x5c : reserved
// 0x60 : Data signal of ccm_10
//        bit 15~0 - ccm_10[15:0] (Read/Write)
//        others   - reserved
// 0x64 : reserved
// 0x68 : Data signal of ccm_11
//        bit 15~0 - ccm_11[15:0] (Read/Write)
//        others   - reserved
// 0x6c : reserved
// 0x70 : Data signal of ccm_12
//        bit 15~0 - ccm_12[15:0] (Read/Write)
//        others   - reserved
// 0x74 : reserved
// 0x78 : Data signal of ccm_20
//        bit 15~0 - ccm_20[15:0] (Read/Write)
//        others   - reserved
// 0x7c : reserved
// 0x80 : Data signal of ccm_21
//        bit 15~0 - ccm_21[15:0] (Read/Write)
//        others   - reserved
// 0x84 : reserved
// 0x88 : Data signal of ccm_22
//        bit 15~0 - ccm_22[15:0] (Read/Write)
//        others   - reserved
// 0x8c : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

//------------------------Parameter----------------------
localparam
    ADDR_AP_CTRL            = 8'h00,
    ADDR_GIE                = 8'h04,
    ADDR_IER                = 8'h08,
    ADDR_ISR                = 8'h0c,
    ADDR_WIDTH_DATA_0       = 8'h10,
    ADDR_WIDTH_CTRL         = 8'h14,
    ADDR_HEIGHT_DATA_0      = 8'h18,
    ADDR_HEIGHT_CTRL        = 8'h1c,
    ADDR_ISP_ENABLE_DATA_0  = 8'h20,
    ADDR_ISP_ENABLE_CTRL    = 8'h24,
    ADDR_BLACK_LEVEL_DATA_0 = 8'h28,
    ADDR_BLACK_LEVEL_CTRL   = 8'h2c,
    ADDR_WB_GAIN_R_DATA_0   = 8'h30,
    ADDR_WB_GAIN_R_CTRL     = 8'h34,
    ADDR_WB_GAIN_G_DATA_0   = 8'h38,
    ADDR_WB_GAIN_G_CTRL     = 8'h3c,
    ADDR_WB_GAIN_B_DATA_0   = 8'h40,
    ADDR_WB_GAIN_B_CTRL     = 8'h44,
    ADDR_CCM_00_DATA_0      = 8'h48,
    ADDR_CCM_00_CTRL        = 8'h4c,
    ADDR_CCM_01_DATA_0      = 8'h50,
    ADDR_CCM_01_CTRL        = 8'h54,
    ADDR_CCM_02_DATA_0      = 8'h58,
    ADDR_CCM_02_CTRL        = 8'h5c,
    ADDR_CCM_10_DATA_0      = 8'h60,
    ADDR_CCM_10_CTRL        = 8'h64,
    ADDR_CCM_11_DATA_0      = 8'h68,
    ADDR_CCM_11_CTRL        = 8'h6c,
    ADDR_CCM_12_DATA_0      = 8'h70,
    ADDR_CCM_12_CTRL        = 8'h74,
    ADDR_CCM_20_DATA_0      = 8'h78,
    ADDR_CCM_20_CTRL        = 8'h7c,
    ADDR_CCM_21_DATA_0      = 8'h80,
    ADDR_CCM_21_CTRL        = 8'h84,
    ADDR_CCM_22_DATA_0      = 8'h88,
    ADDR_CCM_22_CTRL        = 8'h8c,
    WRIDLE                  = 2'd0,
    WRDATA                  = 2'd1,
    WRRESP                  = 2'd2,
    WRRESET                 = 2'd3,
    RDIDLE                  = 2'd0,
    RDDATA                  = 2'd1,
    RDRESET                 = 2'd2,
    ADDR_BITS                = 8;

//------------------------Local signal-------------------
    reg  [1:0]                    wstate = WRRESET;
    reg  [1:0]                    wnext;
    reg  [ADDR_BITS-1:0]          waddr;
    wire [C_S_AXI_DATA_WIDTH-1:0] wmask;
    wire                          aw_hs;
    wire                          w_hs;
    reg  [1:0]                    rstate = RDRESET;
    reg  [1:0]                    rnext;
    reg  [C_S_AXI_DATA_WIDTH-1:0] rdata;
    wire                          ar_hs;
    wire [ADDR_BITS-1:0]          raddr;
    // internal registers
    reg                           int_ap_idle;
    reg                           int_ap_ready = 1'b0;
    wire                          task_ap_ready;
    reg                           int_ap_done = 1'b0;
    wire                          task_ap_done;
    reg                           int_task_ap_done = 1'b0;
    reg                           int_ap_start = 1'b0;
    reg                           int_interrupt = 1'b0;
    reg                           int_auto_restart = 1'b0;
    reg                           auto_restart_status = 1'b0;
    wire                          auto_restart_done;
    reg                           int_gie = 1'b0;
    reg  [1:0]                    int_ier = 2'b0;
    reg  [1:0]                    int_isr = 2'b0;
    reg  [15:0]                   int_width = 'b0;
    reg  [15:0]                   int_height = 'b0;
    reg  [3:0]                    int_isp_enable = 'b0;
    reg  [7:0]                    int_black_level = 'b0;
    reg  [15:0]                   int_wb_gain_r = 'b0;
    reg  [15:0]                   int_wb_gain_g = 'b0;
    reg  [15:0]                   int_wb_gain_b = 'b0;
    reg  [15:0]                   int_ccm_00 = 'b0;
    reg  [15:0]                   int_ccm_01 = 'b0;
    reg  [15:0]                   int_ccm_02 = 'b0;
    reg  [15:0]                   int_ccm_10 = 'b0;
    reg  [15:0]                   int_ccm_11 = 'b0;
    reg  [15:0]                   int_ccm_12 = 'b0;
    reg  [15:0]                   int_ccm_20 = 'b0;
    reg  [15:0]                   int_ccm_21 = 'b0;
    reg  [15:0]                   int_ccm_22 = 'b0;

//------------------------Instantiation------------------


//------------------------AXI write fsm------------------
assign AWREADY = (wstate == WRIDLE);
assign WREADY  = (wstate == WRDATA);
assign BRESP   = 2'b00;  // OKAY
assign BVALID  = (wstate == WRRESP);
assign wmask   = { {8{WSTRB[3]}}, {8{WSTRB[2]}}, {8{WSTRB[1]}}, {8{WSTRB[0]}} };
assign aw_hs   = AWVALID & AWREADY;
assign w_hs    = WVALID & WREADY;

// wstate
always @(posedge ACLK) begin
    if (ARESET)
        wstate <= WRRESET;
    else if (ACLK_EN)
        wstate <= wnext;
end

// wnext
always @(*) begin
    case (wstate)
        WRIDLE:
            if (AWVALID)
                wnext = WRDATA;
            else
                wnext = WRIDLE;
        WRDATA:
            if (WVALID)
                wnext = WRRESP;
            else
                wnext = WRDATA;
        WRRESP:
            if (BREADY)
                wnext = WRIDLE;
            else
                wnext = WRRESP;
        default:
            wnext = WRIDLE;
    endcase
end

// waddr
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (aw_hs)
            waddr <= AWADDR[ADDR_BITS-1:0];
    end
end

//------------------------AXI read fsm-------------------
assign ARREADY = (rstate == RDIDLE);
assign RDATA   = rdata;
assign RRESP   = 2'b00;  // OKAY
assign RVALID  = (rstate == RDDATA);
assign ar_hs   = ARVALID & ARREADY;
assign raddr   = ARADDR[ADDR_BITS-1:0];

// rstate
always @(posedge ACLK) begin
    if (ARESET)
        rstate <= RDRESET;
    else if (ACLK_EN)
        rstate <= rnext;
end

// rnext
always @(*) begin
    case (rstate)
        RDIDLE:
            if (ARVALID)
                rnext = RDDATA;
            else
                rnext = RDIDLE;
        RDDATA:
            if (RREADY & RVALID)
                rnext = RDIDLE;
            else
                rnext = RDDATA;
        default:
            rnext = RDIDLE;
    endcase
end

// rdata
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (ar_hs) begin
            rdata <= 'b0;
            case (raddr)
                ADDR_AP_CTRL: begin
                    rdata[0] <= int_ap_start;
                    rdata[1] <= int_task_ap_done;
                    rdata[2] <= int_ap_idle;
                    rdata[3] <= int_ap_ready;
                    rdata[7] <= int_auto_restart;
                    rdata[9] <= int_interrupt;
                end
                ADDR_GIE: begin
                    rdata <= int_gie;
                end
                ADDR_IER: begin
                    rdata <= int_ier;
                end
                ADDR_ISR: begin
                    rdata <= int_isr;
                end
                ADDR_WIDTH_DATA_0: begin
                    rdata <= int_width[15:0];
                end
                ADDR_HEIGHT_DATA_0: begin
                    rdata <= int_height[15:0];
                end
                ADDR_ISP_ENABLE_DATA_0: begin
                    rdata <= int_isp_enable[3:0];
                end
                ADDR_BLACK_LEVEL_DATA_0: begin
                    rdata <= int_black_level[7:0];
                end
                ADDR_WB_GAIN_R_DATA_0: begin
                    rdata <= int_wb_gain_r[15:0];
                end
                ADDR_WB_GAIN_G_DATA_0: begin
                    rdata <= int_wb_gain_g[15:0];
                end
                ADDR_WB_GAIN_B_DATA_0: begin
                    rdata <= int_wb_gain_b[15:0];
                end
                ADDR_CCM_00_DATA_0: begin
                    rdata <= int_ccm_00[15:0];
                end
                ADDR_CCM_01_DATA_0: begin
                    rdata <= int_ccm_01[15:0];
                end
                ADDR_CCM_02_DATA_0: begin
                    rdata <= int_ccm_02[15:0];
                end
                ADDR_CCM_10_DATA_0: begin
                    rdata <= int_ccm_10[15:0];
                end
                ADDR_CCM_11_DATA_0: begin
                    rdata <= int_ccm_11[15:0];
                end
                ADDR_CCM_12_DATA_0: begin
                    rdata <= int_ccm_12[15:0];
                end
                ADDR_CCM_20_DATA_0: begin
                    rdata <= int_ccm_20[15:0];
                end
                ADDR_CCM_21_DATA_0: begin
                    rdata <= int_ccm_21[15:0];
                end
                ADDR_CCM_22_DATA_0: begin
                    rdata <= int_ccm_22[15:0];
                end
            endcase
        end
    end
end


//------------------------Register logic-----------------
assign interrupt         = int_interrupt;
assign ap_start          = int_ap_start;
assign task_ap_done      = (ap_done && !auto_restart_status) || auto_restart_done;
assign task_ap_ready     = ap_ready && !int_auto_restart;
assign auto_restart_done = auto_restart_status && (ap_idle && !int_ap_idle);
assign width             = int_width;
assign height            = int_height;
assign isp_enable        = int_isp_enable;
assign black_level       = int_black_level;
assign wb_gain_r         = int_wb_gain_r;
assign wb_gain_g         = int_wb_gain_g;
assign wb_gain_b         = int_wb_gain_b;
assign ccm_00            = int_ccm_00;
assign ccm_01            = int_ccm_01;
assign ccm_02            = int_ccm_02;
assign ccm_10            = int_ccm_10;
assign ccm_11            = int_ccm_11;
assign ccm_12            = int_ccm_12;
assign ccm_20            = int_ccm_20;
assign ccm_21            = int_ccm_21;
assign ccm_22            = int_ccm_22;
// int_interrupt
always @(posedge ACLK) begin
    if (ARESET)
        int_interrupt <= 1'b0;
    else if (ACLK_EN) begin
        if (int_gie && (|int_isr))
            int_interrupt <= 1'b1;
        else
            int_interrupt <= 1'b0;
    end
end

// int_ap_start
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_start <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0] && WDATA[0])
            int_ap_start <= 1'b1;
        else if (ap_ready)
            int_ap_start <= int_auto_restart; // clear on handshake/auto restart
    end
end

// int_ap_done
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_done <= 1'b0;
    else if (ACLK_EN) begin
            int_ap_done <= ap_done;
    end
end

// int_task_ap_done
always @(posedge ACLK) begin
    if (ARESET)
        int_task_ap_done <= 1'b0;
    else if (ACLK_EN) begin
        if (task_ap_done)
            int_task_ap_done <= 1'b1;
        else if (ar_hs && raddr == ADDR_AP_CTRL)
            int_task_ap_done <= 1'b0; // clear on read
    end
end

// int_ap_idle
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_idle <= 1'b0;
    else if (ACLK_EN) begin
            int_ap_idle <= ap_idle;
    end
end

// int_ap_ready
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_ready <= 1'b0;
    else if (ACLK_EN) begin
        if (task_ap_ready)
            int_ap_ready <= 1'b1;
        else if (ar_hs && raddr == ADDR_AP_CTRL)
            int_ap_ready <= 1'b0;
    end
end

// int_auto_restart
always @(posedge ACLK) begin
    if (ARESET)
        int_auto_restart <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0])
            int_auto_restart <=  WDATA[7];
    end
end

// auto_restart_status
always @(posedge ACLK) begin
    if (ARESET)
        auto_restart_status <= 1'b0;
    else if (ACLK_EN) begin
        if (int_auto_restart)
            auto_restart_status <= 1'b1;
        else if (ap_idle)
            auto_restart_status <= 1'b0;
    end
end

// int_gie
always @(posedge ACLK) begin
    if (ARESET)
        int_gie <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_GIE && WSTRB[0])
            int_gie <= WDATA[0];
    end
end

// int_ier
always @(posedge ACLK) begin
    if (ARESET)
        int_ier <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_IER && WSTRB[0])
            int_ier <= WDATA[1:0];
    end
end

// int_isr[0]
always @(posedge ACLK) begin
    if (ARESET)
        int_isr[0] <= 1'b0;
    else if (ACLK_EN) begin
        if (int_ier[0] & ap_done)
            int_isr[0] <= 1'b1;
        else if (w_hs && waddr == ADDR_ISR && WSTRB[0])
            int_isr[0] <= int_isr[0] ^ WDATA[0]; // toggle on write
    end
end

// int_isr[1]
always @(posedge ACLK) begin
    if (ARESET)
        int_isr[1] <= 1'b0;
    else if (ACLK_EN) begin
        if (int_ier[1] & ap_ready)
            int_isr[1] <= 1'b1;
        else if (w_hs && waddr == ADDR_ISR && WSTRB[0])
            int_isr[1] <= int_isr[1] ^ WDATA[1]; // toggle on write
    end
end

// int_width[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_width[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_WIDTH_DATA_0)
            int_width[15:0] <= (WDATA[31:0] & wmask) | (int_width[15:0] & ~wmask);
    end
end

// int_height[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_height[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_HEIGHT_DATA_0)
            int_height[15:0] <= (WDATA[31:0] & wmask) | (int_height[15:0] & ~wmask);
    end
end

// int_isp_enable[3:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_isp_enable[3:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_ISP_ENABLE_DATA_0)
            int_isp_enable[3:0] <= (WDATA[31:0] & wmask) | (int_isp_enable[3:0] & ~wmask);
    end
end

// int_black_level[7:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_black_level[7:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_BLACK_LEVEL_DATA_0)
            int_black_level[7:0] <= (WDATA[31:0] & wmask) | (int_black_level[7:0] & ~wmask);
    end
end

// int_wb_gain_r[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_wb_gain_r[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_WB_GAIN_R_DATA_0)
            int_wb_gain_r[15:0] <= (WDATA[31:0] & wmask) | (int_wb_gain_r[15:0] & ~wmask);
    end
end

// int_wb_gain_g[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_wb_gain_g[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_WB_GAIN_G_DATA_0)
            int_wb_gain_g[15:0] <= (WDATA[31:0] & wmask) | (int_wb_gain_g[15:0] & ~wmask);
    end
end

// int_wb_gain_b[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_wb_gain_b[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_WB_GAIN_B_DATA_0)
            int_wb_gain_b[15:0] <= (WDATA[31:0] & wmask) | (int_wb_gain_b[15:0] & ~wmask);
    end
end

// int_ccm_00[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_00[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_00_DATA_0)
            int_ccm_00[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_00[15:0] & ~wmask);
    end
end

// int_ccm_01[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_01[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_01_DATA_0)
            int_ccm_01[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_01[15:0] & ~wmask);
    end
end

// int_ccm_02[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_02[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_02_DATA_0)
            int_ccm_02[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_02[15:0] & ~wmask);
    end
end

// int_ccm_10[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_10[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_10_DATA_0)
            int_ccm_10[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_10[15:0] & ~wmask);
    end
end

// int_ccm_11[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_11[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_11_DATA_0)
            int_ccm_11[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_11[15:0] & ~wmask);
    end
end

// int_ccm_12[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_12[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_12_DATA_0)
            int_ccm_12[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_12[15:0] & ~wmask);
    end
end

// int_ccm_20[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_20[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_20_DATA_0)
            int_ccm_20[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_20[15:0] & ~wmask);
    end
end

// int_ccm_21[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_21[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_21_DATA_0)
            int_ccm_21[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_21[15:0] & ~wmask);
    end
end

// int_ccm_22[15:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ccm_22[15:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CCM_22_DATA_0)
            int_ccm_22[15:0] <= (WDATA[31:0] & wmask) | (int_ccm_22[15:0] & ~wmask);
    end
end

//synthesis translate_off
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (int_gie & ~int_isr[0] & int_ier[0] & ap_done)
            $display ("// Interrupt Monitor : interrupt for ap_done detected @ \"%0t\"", $time);
        if (int_gie & ~int_isr[1] & int_ier[1] & ap_ready)
            $display ("// Interrupt Monitor : interrupt for ap_ready detected @ \"%0t\"", $time);
    end
end
//synthesis translate_on

//------------------------Memory logic-------------------

endmodule
