// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.2 (64-bit)
// Tool Version Limit: 2019.12
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// control
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

#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_AP_CTRL          0x00
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_GIE              0x04
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_IER              0x08
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_ISR              0x0c
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WIDTH_DATA       0x10
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_WIDTH_DATA       16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_HEIGHT_DATA      0x18
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_HEIGHT_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_ISP_ENABLE_DATA  0x20
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_ISP_ENABLE_DATA  4
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_BLACK_LEVEL_DATA 0x28
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_BLACK_LEVEL_DATA 8
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_R_DATA   0x30
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_WB_GAIN_R_DATA   16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_G_DATA   0x38
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_WB_GAIN_G_DATA   16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_B_DATA   0x40
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_WB_GAIN_B_DATA   16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_00_DATA      0x48
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_00_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_01_DATA      0x50
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_01_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_02_DATA      0x58
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_02_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_10_DATA      0x60
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_10_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_11_DATA      0x68
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_11_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_12_DATA      0x70
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_12_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_20_DATA      0x78
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_20_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_21_DATA      0x80
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_21_DATA      16
#define XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_22_DATA      0x88
#define XBAYER_RG8_TO_RGBA8888_CONTROL_BITS_CCM_22_DATA      16

