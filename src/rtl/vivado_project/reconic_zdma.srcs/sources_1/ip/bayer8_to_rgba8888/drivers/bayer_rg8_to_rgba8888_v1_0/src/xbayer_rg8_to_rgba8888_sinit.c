// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.2 (64-bit)
// Tool Version Limit: 2019.12
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xbayer_rg8_to_rgba8888.h"

extern XBayer_rg8_to_rgba8888_Config XBayer_rg8_to_rgba8888_ConfigTable[];

XBayer_rg8_to_rgba8888_Config *XBayer_rg8_to_rgba8888_LookupConfig(u16 DeviceId) {
	XBayer_rg8_to_rgba8888_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XBAYER_RG8_TO_RGBA8888_NUM_INSTANCES; Index++) {
		if (XBayer_rg8_to_rgba8888_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XBayer_rg8_to_rgba8888_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XBayer_rg8_to_rgba8888_Initialize(XBayer_rg8_to_rgba8888 *InstancePtr, u16 DeviceId) {
	XBayer_rg8_to_rgba8888_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XBayer_rg8_to_rgba8888_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XBayer_rg8_to_rgba8888_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

