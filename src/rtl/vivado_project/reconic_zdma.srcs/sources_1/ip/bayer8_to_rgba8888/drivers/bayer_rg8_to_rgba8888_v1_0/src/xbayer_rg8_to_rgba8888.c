// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.2 (64-bit)
// Tool Version Limit: 2019.12
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xbayer_rg8_to_rgba8888.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XBayer_rg8_to_rgba8888_CfgInitialize(XBayer_rg8_to_rgba8888 *InstancePtr, XBayer_rg8_to_rgba8888_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XBayer_rg8_to_rgba8888_Start(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_AP_CTRL) & 0x80;
    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XBayer_rg8_to_rgba8888_IsDone(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XBayer_rg8_to_rgba8888_IsIdle(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XBayer_rg8_to_rgba8888_IsReady(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XBayer_rg8_to_rgba8888_EnableAutoRestart(XBayer_rg8_to_rgba8888 *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XBayer_rg8_to_rgba8888_DisableAutoRestart(XBayer_rg8_to_rgba8888 *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_AP_CTRL, 0);
}

void XBayer_rg8_to_rgba8888_Set_width(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WIDTH_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_width(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WIDTH_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_height(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_HEIGHT_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_height(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_HEIGHT_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_isp_enable(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_ISP_ENABLE_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_isp_enable(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_ISP_ENABLE_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_black_level(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_BLACK_LEVEL_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_black_level(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_BLACK_LEVEL_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_wb_gain_r(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_R_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_wb_gain_r(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_R_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_wb_gain_g(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_G_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_wb_gain_g(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_G_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_wb_gain_b(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_B_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_wb_gain_b(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_WB_GAIN_B_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_00(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_00_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_00(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_00_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_01(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_01_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_01(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_01_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_02(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_02_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_02(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_02_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_10(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_10_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_10(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_10_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_11(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_11_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_11(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_11_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_12(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_12_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_12(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_12_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_20(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_20_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_20(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_20_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_21(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_21_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_21(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_21_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_Set_ccm_22(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_22_DATA, Data);
}

u32 XBayer_rg8_to_rgba8888_Get_ccm_22(XBayer_rg8_to_rgba8888 *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_CCM_22_DATA);
    return Data;
}

void XBayer_rg8_to_rgba8888_InterruptGlobalEnable(XBayer_rg8_to_rgba8888 *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_GIE, 1);
}

void XBayer_rg8_to_rgba8888_InterruptGlobalDisable(XBayer_rg8_to_rgba8888 *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_GIE, 0);
}

void XBayer_rg8_to_rgba8888_InterruptEnable(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_IER);
    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_IER, Register | Mask);
}

void XBayer_rg8_to_rgba8888_InterruptDisable(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_IER);
    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_IER, Register & (~Mask));
}

void XBayer_rg8_to_rgba8888_InterruptClear(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XBayer_rg8_to_rgba8888_WriteReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_ISR, Mask);
}

u32 XBayer_rg8_to_rgba8888_InterruptGetEnabled(XBayer_rg8_to_rgba8888 *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_IER);
}

u32 XBayer_rg8_to_rgba8888_InterruptGetStatus(XBayer_rg8_to_rgba8888 *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XBayer_rg8_to_rgba8888_ReadReg(InstancePtr->Control_BaseAddress, XBAYER_RG8_TO_RGBA8888_CONTROL_ADDR_ISR);
}

