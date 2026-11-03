// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.2 (64-bit)
// Tool Version Limit: 2019.12
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XBAYER_RG8_TO_RGBA8888_H
#define XBAYER_RG8_TO_RGBA8888_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xbayer_rg8_to_rgba8888_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u64 Control_BaseAddress;
} XBayer_rg8_to_rgba8888_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XBayer_rg8_to_rgba8888;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XBayer_rg8_to_rgba8888_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XBayer_rg8_to_rgba8888_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XBayer_rg8_to_rgba8888_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XBayer_rg8_to_rgba8888_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XBayer_rg8_to_rgba8888_Initialize(XBayer_rg8_to_rgba8888 *InstancePtr, u16 DeviceId);
XBayer_rg8_to_rgba8888_Config* XBayer_rg8_to_rgba8888_LookupConfig(u16 DeviceId);
int XBayer_rg8_to_rgba8888_CfgInitialize(XBayer_rg8_to_rgba8888 *InstancePtr, XBayer_rg8_to_rgba8888_Config *ConfigPtr);
#else
int XBayer_rg8_to_rgba8888_Initialize(XBayer_rg8_to_rgba8888 *InstancePtr, const char* InstanceName);
int XBayer_rg8_to_rgba8888_Release(XBayer_rg8_to_rgba8888 *InstancePtr);
#endif

void XBayer_rg8_to_rgba8888_Start(XBayer_rg8_to_rgba8888 *InstancePtr);
u32 XBayer_rg8_to_rgba8888_IsDone(XBayer_rg8_to_rgba8888 *InstancePtr);
u32 XBayer_rg8_to_rgba8888_IsIdle(XBayer_rg8_to_rgba8888 *InstancePtr);
u32 XBayer_rg8_to_rgba8888_IsReady(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_EnableAutoRestart(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_DisableAutoRestart(XBayer_rg8_to_rgba8888 *InstancePtr);

void XBayer_rg8_to_rgba8888_Set_width(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_width(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_height(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_height(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_isp_enable(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_isp_enable(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_black_level(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_black_level(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_wb_gain_r(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_wb_gain_r(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_wb_gain_g(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_wb_gain_g(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_wb_gain_b(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_wb_gain_b(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_00(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_00(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_01(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_01(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_02(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_02(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_10(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_10(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_11(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_11(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_12(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_12(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_20(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_20(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_21(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_21(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_Set_ccm_22(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Data);
u32 XBayer_rg8_to_rgba8888_Get_ccm_22(XBayer_rg8_to_rgba8888 *InstancePtr);

void XBayer_rg8_to_rgba8888_InterruptGlobalEnable(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_InterruptGlobalDisable(XBayer_rg8_to_rgba8888 *InstancePtr);
void XBayer_rg8_to_rgba8888_InterruptEnable(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Mask);
void XBayer_rg8_to_rgba8888_InterruptDisable(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Mask);
void XBayer_rg8_to_rgba8888_InterruptClear(XBayer_rg8_to_rgba8888 *InstancePtr, u32 Mask);
u32 XBayer_rg8_to_rgba8888_InterruptGetEnabled(XBayer_rg8_to_rgba8888 *InstancePtr);
u32 XBayer_rg8_to_rgba8888_InterruptGetStatus(XBayer_rg8_to_rgba8888 *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
