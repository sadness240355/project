# ==============================================================
# Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.2 (64-bit)
# Tool Version Limit: 2019.12
# Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
# ==============================================================

cd -- "$(dirname -- "$0")"
"${VIVADO:-vivado}"  -notrace -mode batch -source ipi_example.tcl -tclargs xcku15p-ffve1517-2-i ../xilinx_com_hls_bayer_rg8_to_rgba8888_1_0.zip
