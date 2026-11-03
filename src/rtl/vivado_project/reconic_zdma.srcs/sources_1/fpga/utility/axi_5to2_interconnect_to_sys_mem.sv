// *************************************************************************
//
// Copyright 2022 Xilinx, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// *************************************************************************
`timescale 1ns/1ps

module axi_6to2_interconnect_to_sys_mem #(
  parameter C_AXI_DATA_WIDTH = 512,
  parameter C_AXI_ADDR_WIDTH = 64
) (
  input             s_axi_rdma_get_wqe_awid,
  input    [63 : 0] s_axi_rdma_get_wqe_awaddr,
  input     [3 : 0] s_axi_rdma_get_wqe_awqos,
  input     [7 : 0] s_axi_rdma_get_wqe_awlen,
  input     [2 : 0] s_axi_rdma_get_wqe_awsize,
  input     [1 : 0] s_axi_rdma_get_wqe_awburst,
  input     [3 : 0] s_axi_rdma_get_wqe_awcache,
  input     [2 : 0] s_axi_rdma_get_wqe_awprot,
  input             s_axi_rdma_get_wqe_awvalid,
  output            s_axi_rdma_get_wqe_awready,
  input   [511 : 0] s_axi_rdma_get_wqe_wdata,
  input    [63 : 0] s_axi_rdma_get_wqe_wstrb,
  input             s_axi_rdma_get_wqe_wlast,
  input             s_axi_rdma_get_wqe_wvalid,
  output            s_axi_rdma_get_wqe_wready,
  input             s_axi_rdma_get_wqe_awlock,
  output            s_axi_rdma_get_wqe_bid,
  output    [1 : 0] s_axi_rdma_get_wqe_bresp,
  output            s_axi_rdma_get_wqe_bvalid,
  input             s_axi_rdma_get_wqe_bready,
  input             s_axi_rdma_get_wqe_arid,
  input    [63 : 0] s_axi_rdma_get_wqe_araddr,
  input     [7 : 0] s_axi_rdma_get_wqe_arlen,
  input     [2 : 0] s_axi_rdma_get_wqe_arsize,
  input     [1 : 0] s_axi_rdma_get_wqe_arburst,
  input     [3 : 0] s_axi_rdma_get_wqe_arcache,
  input     [2 : 0] s_axi_rdma_get_wqe_arprot,
  input             s_axi_rdma_get_wqe_arvalid,
  output            s_axi_rdma_get_wqe_arready,
  output            s_axi_rdma_get_wqe_rid,
  output  [511 : 0] s_axi_rdma_get_wqe_rdata,
  output    [1 : 0] s_axi_rdma_get_wqe_rresp,
  output            s_axi_rdma_get_wqe_rlast,
  output            s_axi_rdma_get_wqe_rvalid,
  input             s_axi_rdma_get_wqe_rready,
  input             s_axi_rdma_get_wqe_arlock,
  input     [3 : 0] s_axi_rdma_get_wqe_arqos,

  input             s_axi_rdma_get_payload_awid,
  input    [63 : 0] s_axi_rdma_get_payload_awaddr,
  input     [3 : 0] s_axi_rdma_get_payload_awqos,
  input     [7 : 0] s_axi_rdma_get_payload_awlen,
  input     [2 : 0] s_axi_rdma_get_payload_awsize,
  input     [1 : 0] s_axi_rdma_get_payload_awburst,
  input     [3 : 0] s_axi_rdma_get_payload_awcache,
  input     [2 : 0] s_axi_rdma_get_payload_awprot,
  input             s_axi_rdma_get_payload_awvalid,
  output            s_axi_rdma_get_payload_awready,
  input   [511 : 0] s_axi_rdma_get_payload_wdata,
  input    [63 : 0] s_axi_rdma_get_payload_wstrb,
  input             s_axi_rdma_get_payload_wlast,
  input             s_axi_rdma_get_payload_wvalid,
  output            s_axi_rdma_get_payload_wready,
  input             s_axi_rdma_get_payload_awlock,
  output            s_axi_rdma_get_payload_bid,
  output    [1 : 0] s_axi_rdma_get_payload_bresp,
  output            s_axi_rdma_get_payload_bvalid,
  input             s_axi_rdma_get_payload_bready,
  input             s_axi_rdma_get_payload_arid,
  input    [63 : 0] s_axi_rdma_get_payload_araddr,
  input     [7 : 0] s_axi_rdma_get_payload_arlen,
  input     [2 : 0] s_axi_rdma_get_payload_arsize,
  input     [1 : 0] s_axi_rdma_get_payload_arburst,
  input     [3 : 0] s_axi_rdma_get_payload_arcache,
  input     [2 : 0] s_axi_rdma_get_payload_arprot,
  input             s_axi_rdma_get_payload_arvalid,
  output            s_axi_rdma_get_payload_arready,
  output            s_axi_rdma_get_payload_rid,
  output  [511 : 0] s_axi_rdma_get_payload_rdata,
  output    [1 : 0] s_axi_rdma_get_payload_rresp,
  output            s_axi_rdma_get_payload_rlast,
  output            s_axi_rdma_get_payload_rvalid,
  input             s_axi_rdma_get_payload_rready,
  input             s_axi_rdma_get_payload_arlock,
  input     [3 : 0] s_axi_rdma_get_payload_arqos,

  input             s_axi_rdma_completion_awid,
  input    [63 : 0] s_axi_rdma_completion_awaddr,
  input     [3 : 0] s_axi_rdma_completion_awqos,
  input     [7 : 0] s_axi_rdma_completion_awlen,
  input     [2 : 0] s_axi_rdma_completion_awsize,
  input     [1 : 0] s_axi_rdma_completion_awburst,
  input     [3 : 0] s_axi_rdma_completion_awcache,
  input     [2 : 0] s_axi_rdma_completion_awprot,
  input             s_axi_rdma_completion_awvalid,
  output            s_axi_rdma_completion_awready,
  input   [511 : 0] s_axi_rdma_completion_wdata,
  input    [63 : 0] s_axi_rdma_completion_wstrb,
  input             s_axi_rdma_completion_wlast,
  input             s_axi_rdma_completion_wvalid,
  output            s_axi_rdma_completion_wready,
  input             s_axi_rdma_completion_awlock,
  output            s_axi_rdma_completion_bid,
  output    [1 : 0] s_axi_rdma_completion_bresp,
  output            s_axi_rdma_completion_bvalid,
  input             s_axi_rdma_completion_bready,
  input             s_axi_rdma_completion_arid,
  input    [63 : 0] s_axi_rdma_completion_araddr,
  input     [7 : 0] s_axi_rdma_completion_arlen,
  input     [2 : 0] s_axi_rdma_completion_arsize,
  input     [1 : 0] s_axi_rdma_completion_arburst,
  input     [3 : 0] s_axi_rdma_completion_arcache,
  input     [2 : 0] s_axi_rdma_completion_arprot,
  input             s_axi_rdma_completion_arvalid,
  output            s_axi_rdma_completion_arready,
  output            s_axi_rdma_completion_rid,
  output  [511 : 0] s_axi_rdma_completion_rdata,
  output    [1 : 0] s_axi_rdma_completion_rresp,
  output            s_axi_rdma_completion_rlast,
  output            s_axi_rdma_completion_rvalid,
  input             s_axi_rdma_completion_rready,
  input             s_axi_rdma_completion_arlock,
  input     [3 : 0] s_axi_rdma_completion_arqos,

  input             s_axi_rdma_send_write_payload_awid,
  input    [63 : 0] s_axi_rdma_send_write_payload_awaddr,
  input     [3 : 0] s_axi_rdma_send_write_payload_awqos,
  input     [7 : 0] s_axi_rdma_send_write_payload_awlen,
  input     [2 : 0] s_axi_rdma_send_write_payload_awsize,
  input     [1 : 0] s_axi_rdma_send_write_payload_awburst,
  input     [3 : 0] s_axi_rdma_send_write_payload_awcache,
  input     [2 : 0] s_axi_rdma_send_write_payload_awprot,
  input             s_axi_rdma_send_write_payload_awvalid,
  output            s_axi_rdma_send_write_payload_awready,
  input   [511 : 0] s_axi_rdma_send_write_payload_wdata,
  input    [63 : 0] s_axi_rdma_send_write_payload_wstrb,
  input             s_axi_rdma_send_write_payload_wlast,
  input             s_axi_rdma_send_write_payload_wvalid,
  output            s_axi_rdma_send_write_payload_wready,
  input             s_axi_rdma_send_write_payload_awlock,
  output            s_axi_rdma_send_write_payload_bid,
  output    [1 : 0] s_axi_rdma_send_write_payload_bresp,
  output            s_axi_rdma_send_write_payload_bvalid,
  input             s_axi_rdma_send_write_payload_bready,
  input             s_axi_rdma_send_write_payload_arid,
  input    [63 : 0] s_axi_rdma_send_write_payload_araddr,
  input     [7 : 0] s_axi_rdma_send_write_payload_arlen,
  input     [2 : 0] s_axi_rdma_send_write_payload_arsize,
  input     [1 : 0] s_axi_rdma_send_write_payload_arburst,
  input     [3 : 0] s_axi_rdma_send_write_payload_arcache,
  input     [2 : 0] s_axi_rdma_send_write_payload_arprot,
  input             s_axi_rdma_send_write_payload_arvalid,
  output            s_axi_rdma_send_write_payload_arready,
  output            s_axi_rdma_send_write_payload_rid,
  output  [511 : 0] s_axi_rdma_send_write_payload_rdata,
  output    [1 : 0] s_axi_rdma_send_write_payload_rresp,
  output            s_axi_rdma_send_write_payload_rlast,
  output            s_axi_rdma_send_write_payload_rvalid,
  input             s_axi_rdma_send_write_payload_rready,
  input             s_axi_rdma_send_write_payload_arlock,
  input     [3 : 0] s_axi_rdma_send_write_payload_arqos,

  input             s_axi_rdma_rsp_payload_awid,
  input    [63 : 0] s_axi_rdma_rsp_payload_awaddr,
  input     [3 : 0] s_axi_rdma_rsp_payload_awqos,
  input     [7 : 0] s_axi_rdma_rsp_payload_awlen,
  input     [2 : 0] s_axi_rdma_rsp_payload_awsize,
  input     [1 : 0] s_axi_rdma_rsp_payload_awburst,
  input     [3 : 0] s_axi_rdma_rsp_payload_awcache,
  input     [2 : 0] s_axi_rdma_rsp_payload_awprot,
  input             s_axi_rdma_rsp_payload_awvalid,
  output            s_axi_rdma_rsp_payload_awready,
  input   [511 : 0] s_axi_rdma_rsp_payload_wdata,
  input    [63 : 0] s_axi_rdma_rsp_payload_wstrb,
  input             s_axi_rdma_rsp_payload_wlast,
  input             s_axi_rdma_rsp_payload_wvalid,
  output            s_axi_rdma_rsp_payload_wready,
  input             s_axi_rdma_rsp_payload_awlock,
  output            s_axi_rdma_rsp_payload_bid,
  output    [1 : 0] s_axi_rdma_rsp_payload_bresp,
  output            s_axi_rdma_rsp_payload_bvalid,
  input             s_axi_rdma_rsp_payload_bready,
  input             s_axi_rdma_rsp_payload_arid,
  input    [63 : 0] s_axi_rdma_rsp_payload_araddr,
  input     [7 : 0] s_axi_rdma_rsp_payload_arlen,
  input     [2 : 0] s_axi_rdma_rsp_payload_arsize,
  input     [1 : 0] s_axi_rdma_rsp_payload_arburst,
  input     [3 : 0] s_axi_rdma_rsp_payload_arcache,
  input     [2 : 0] s_axi_rdma_rsp_payload_arprot,
  input             s_axi_rdma_rsp_payload_arvalid,
  output            s_axi_rdma_rsp_payload_arready,
  output            s_axi_rdma_rsp_payload_rid,
  output  [511 : 0] s_axi_rdma_rsp_payload_rdata,
  output    [1 : 0] s_axi_rdma_rsp_payload_rresp,
  output            s_axi_rdma_rsp_payload_rlast,
  output            s_axi_rdma_rsp_payload_rvalid,
  input             s_axi_rdma_rsp_payload_rready,
  input             s_axi_rdma_rsp_payload_arlock,
  input     [3 : 0] s_axi_rdma_rsp_payload_arqos,

  input             s_axi_qdma_mm_awid,
  input    [63 : 0] s_axi_qdma_mm_awaddr,
  input     [3 : 0] s_axi_qdma_mm_awqos,
  input     [7 : 0] s_axi_qdma_mm_awlen,
  input     [2 : 0] s_axi_qdma_mm_awsize,
  input     [1 : 0] s_axi_qdma_mm_awburst,
  input     [3 : 0] s_axi_qdma_mm_awcache,
  input     [2 : 0] s_axi_qdma_mm_awprot,
  input             s_axi_qdma_mm_awvalid,
  output            s_axi_qdma_mm_awready,
  input   [511 : 0] s_axi_qdma_mm_wdata,
  input    [63 : 0] s_axi_qdma_mm_wstrb,
  input             s_axi_qdma_mm_wlast,
  input             s_axi_qdma_mm_wvalid,
  output            s_axi_qdma_mm_wready,
  input             s_axi_qdma_mm_awlock,
  output            s_axi_qdma_mm_bid,
  output    [1 : 0] s_axi_qdma_mm_bresp,
  output            s_axi_qdma_mm_bvalid,
  input             s_axi_qdma_mm_bready,
  input             s_axi_qdma_mm_arid,
  input    [63 : 0] s_axi_qdma_mm_araddr,
  input     [7 : 0] s_axi_qdma_mm_arlen,
  input     [2 : 0] s_axi_qdma_mm_arsize,
  input     [1 : 0] s_axi_qdma_mm_arburst,
  input     [3 : 0] s_axi_qdma_mm_arcache,
  input     [2 : 0] s_axi_qdma_mm_arprot,
  input             s_axi_qdma_mm_arvalid,
  output            s_axi_qdma_mm_arready,
  output            s_axi_qdma_mm_rid,
  output  [511 : 0] s_axi_qdma_mm_rdata,
  output    [1 : 0] s_axi_qdma_mm_rresp,
  output            s_axi_qdma_mm_rlast,
  output            s_axi_qdma_mm_rvalid,
  input             s_axi_qdma_mm_rready,
  input             s_axi_qdma_mm_arlock,
  input     [3 : 0] s_axi_qdma_mm_arqos,

  input     [2 : 0] s_axi_camera_awid,
  input    [63 : 0] s_axi_camera_awaddr,
  input     [3 : 0] s_axi_camera_awqos,
  input     [7 : 0] s_axi_camera_awlen,
  input     [2 : 0] s_axi_camera_awsize,
  input     [1 : 0] s_axi_camera_awburst,
  input     [3 : 0] s_axi_camera_awcache,
  input     [2 : 0] s_axi_camera_awprot,
  input             s_axi_camera_awvalid,
  output            s_axi_camera_awready,
  input   [511 : 0] s_axi_camera_wdata,
  input    [63 : 0] s_axi_camera_wstrb,
  input             s_axi_camera_wlast,
  input             s_axi_camera_wvalid,
  output            s_axi_camera_wready,
  input             s_axi_camera_awlock,
  output    [2 : 0] s_axi_camera_bid,
  output    [1 : 0] s_axi_camera_bresp,
  output            s_axi_camera_bvalid,
  input             s_axi_camera_bready,
  // Independent ISP-to-host write port (Crossbar SI7).
  input [63:0] s_axi_camera_host_awaddr,
  input [7:0] s_axi_camera_host_awlen,
  input s_axi_camera_host_awvalid,
  output s_axi_camera_host_awready,
  input [511:0] s_axi_camera_host_wdata,
  input [63:0] s_axi_camera_host_wstrb,
  input s_axi_camera_host_wlast,s_axi_camera_host_wvalid,
  output s_axi_camera_host_wready,
  output [1:0] s_axi_camera_host_bresp,
  output s_axi_camera_host_bvalid,
  input s_axi_camera_host_bready,

  output      [2:0] m_axi_sys_mem_awid,
  output     [63:0] m_axi_sys_mem_awaddr,
  output      [7:0] m_axi_sys_mem_awlen,
  output      [2:0] m_axi_sys_mem_awsize,
  output      [1:0] m_axi_sys_mem_awburst,
  output            m_axi_sys_mem_awlock,
  output      [3:0] m_axi_sys_mem_awqos,
  output      [3:0] m_axi_sys_mem_awregion,
  output      [3:0] m_axi_sys_mem_awcache,
  output      [2:0] m_axi_sys_mem_awprot,
  output            m_axi_sys_mem_awvalid,
  input             m_axi_sys_mem_awready,
  output    [511:0] m_axi_sys_mem_wdata,
  output     [63:0] m_axi_sys_mem_wstrb,
  output            m_axi_sys_mem_wlast,
  output            m_axi_sys_mem_wvalid,
  input             m_axi_sys_mem_wready,
  input       [2:0] m_axi_sys_mem_bid,
  input       [1:0] m_axi_sys_mem_bresp,
  input             m_axi_sys_mem_bvalid,
  output            m_axi_sys_mem_bready,
  output      [2:0] m_axi_sys_mem_arid,
  output     [63:0] m_axi_sys_mem_araddr,
  output      [7:0] m_axi_sys_mem_arlen,
  output      [2:0] m_axi_sys_mem_arsize,
  output      [1:0] m_axi_sys_mem_arburst,
  output            m_axi_sys_mem_arlock,
  output      [3:0] m_axi_sys_mem_arqos,
  output      [3:0] m_axi_sys_mem_arregion,
  output      [3:0] m_axi_sys_mem_arcache,
  output      [2:0] m_axi_sys_mem_arprot,
  output            m_axi_sys_mem_arvalid,
  input             m_axi_sys_mem_arready,
  input       [2:0] m_axi_sys_mem_rid,
  input     [511:0] m_axi_sys_mem_rdata,
  input       [1:0] m_axi_sys_mem_rresp,
  input             m_axi_sys_mem_rlast,
  input             m_axi_sys_mem_rvalid,
  output            m_axi_sys_mem_rready,

  output      [2:0] m_axi_sys_to_dev_crossbar_awid,
  output     [63:0] m_axi_sys_to_dev_crossbar_awaddr,
  output      [7:0] m_axi_sys_to_dev_crossbar_awlen,
  output      [2:0] m_axi_sys_to_dev_crossbar_awsize,
  output      [1:0] m_axi_sys_to_dev_crossbar_awburst,
  output            m_axi_sys_to_dev_crossbar_awlock,
  output      [3:0] m_axi_sys_to_dev_crossbar_awqos,
  output      [3:0] m_axi_sys_to_dev_crossbar_awregion,
  output      [3:0] m_axi_sys_to_dev_crossbar_awcache,
  output      [2:0] m_axi_sys_to_dev_crossbar_awprot,
  output            m_axi_sys_to_dev_crossbar_awvalid,
  input             m_axi_sys_to_dev_crossbar_awready,
  output    [511:0] m_axi_sys_to_dev_crossbar_wdata,
  output     [63:0] m_axi_sys_to_dev_crossbar_wstrb,
  output            m_axi_sys_to_dev_crossbar_wlast,
  output            m_axi_sys_to_dev_crossbar_wvalid,
  input             m_axi_sys_to_dev_crossbar_wready,
  input       [2:0]  m_axi_sys_to_dev_crossbar_bid,
  input       [1:0] m_axi_sys_to_dev_crossbar_bresp,
  input             m_axi_sys_to_dev_crossbar_bvalid,
  output            m_axi_sys_to_dev_crossbar_bready,
  output      [2:0]   m_axi_sys_to_dev_crossbar_arid,
  output     [63:0] m_axi_sys_to_dev_crossbar_araddr,
  output      [7:0] m_axi_sys_to_dev_crossbar_arlen,
  output      [2:0] m_axi_sys_to_dev_crossbar_arsize,
  output      [1:0] m_axi_sys_to_dev_crossbar_arburst,
  output            m_axi_sys_to_dev_crossbar_arlock,
  output      [3:0] m_axi_sys_to_dev_crossbar_arqos,
  output      [3:0] m_axi_sys_to_dev_crossbar_arregion,
  output      [3:0] m_axi_sys_to_dev_crossbar_arcache,
  output      [2:0] m_axi_sys_to_dev_crossbar_arprot,
  output            m_axi_sys_to_dev_crossbar_arvalid,
  input             m_axi_sys_to_dev_crossbar_arready,
  input       [2:0] m_axi_sys_to_dev_crossbar_rid,
  input     [511:0] m_axi_sys_to_dev_crossbar_rdata,
  input       [1:0] m_axi_sys_to_dev_crossbar_rresp,
  input             m_axi_sys_to_dev_crossbar_rlast,
  input             m_axi_sys_to_dev_crossbar_rvalid,
  output            m_axi_sys_to_dev_crossbar_rready,

  input axis_aclk,
  input axis_arestn
);

wire [519:0] rdma_get_payload_rword;
wire rdma_get_payload_rvalid_pipe;
wire [519:0] qdma_mm_rword;
wire qdma_mm_rvalid_pipe;


localparam C_NUM_MASTERS = 8;
localparam C_NUM_SLAVES = 2;


localparam C_RDMA_GET_WQE_IDX     = 0;
localparam C_RDMA_GET_PAYLOAD_IDX = 1;
localparam C_RDMA_COMPLETION_IDX  = 2;
localparam C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX = 3;
localparam C_RDMA_RSP_PAYLOAD_IDX = 4;
localparam C_QDMA_MM_IDX = 5;
localparam C_CAMERA_IDX = 6;
localparam C_CAMERA_HOST_IDX = 7;

localparam C_SYS_MEM_IDX     = 0;
localparam C_SYS_TO_DEV_CROSSBAR_IDX = 1;

logic   [C_NUM_MASTERS*3-1 : 0] s_axi_awid;
logic  [C_NUM_MASTERS*64-1 : 0] s_axi_awaddr;
logic   [C_NUM_MASTERS*8-1 : 0] s_axi_awlen;
logic   [C_NUM_MASTERS*3-1 : 0] s_axi_awsize;
logic   [C_NUM_MASTERS*2-1 : 0] s_axi_awburst;
logic     [C_NUM_MASTERS-1 : 0] s_axi_awlock;
logic   [C_NUM_MASTERS*4-1 : 0] s_axi_awcache;
logic   [C_NUM_MASTERS*3-1 : 0] s_axi_awprot;
logic   [C_NUM_MASTERS*4-1 : 0] s_axi_awqos;
logic     [C_NUM_MASTERS-1 : 0] s_axi_awvalid;
logic     [C_NUM_MASTERS-1 : 0] s_axi_awready;
logic [C_NUM_MASTERS*512-1 : 0] s_axi_wdata;
logic  [C_NUM_MASTERS*64-1 : 0] s_axi_wstrb;
logic     [C_NUM_MASTERS-1 : 0] s_axi_wlast;
logic     [C_NUM_MASTERS-1 : 0] s_axi_wvalid;
logic     [C_NUM_MASTERS-1 : 0] s_axi_wready;
logic   [C_NUM_MASTERS*3-1 : 0] s_axi_bid;
logic   [C_NUM_MASTERS*2-1 : 0] s_axi_bresp;
logic     [C_NUM_MASTERS-1 : 0] s_axi_bvalid;
logic     [C_NUM_MASTERS-1 : 0] s_axi_bready;
logic   [C_NUM_MASTERS*3-1 : 0] s_axi_arid;
logic  [C_NUM_MASTERS*64-1 : 0] s_axi_araddr;
logic   [C_NUM_MASTERS*8-1 : 0] s_axi_arlen;
logic   [C_NUM_MASTERS*3-1 : 0] s_axi_arsize;
logic   [C_NUM_MASTERS*2-1 : 0] s_axi_arburst;
logic     [C_NUM_MASTERS-1 : 0] s_axi_arlock;
logic   [C_NUM_MASTERS*4-1 : 0] s_axi_arcache;
logic   [C_NUM_MASTERS*3-1 : 0] s_axi_arprot;
logic   [C_NUM_MASTERS*4-1 : 0] s_axi_arqos;
logic     [C_NUM_MASTERS-1 : 0] s_axi_arvalid;
logic     [C_NUM_MASTERS-1 : 0] s_axi_arready;
logic   [C_NUM_MASTERS*3-1 : 0] s_axi_rid;
logic [C_NUM_MASTERS*512-1 : 0] s_axi_rdata;
logic   [C_NUM_MASTERS*2-1 : 0] s_axi_rresp;
logic     [C_NUM_MASTERS-1 : 0] s_axi_rlast;
logic     [C_NUM_MASTERS-1 : 0] s_axi_rvalid;
logic     [C_NUM_MASTERS-1 : 0] s_axi_rready;

logic   [C_NUM_SLAVES*3-1 : 0] m_axi_awid;
logic  [C_NUM_SLAVES*64-1 : 0] m_axi_awaddr;
logic   [C_NUM_SLAVES*8-1 : 0] m_axi_awlen;
logic   [C_NUM_SLAVES*3-1 : 0] m_axi_awsize;
logic   [C_NUM_SLAVES*2-1 : 0] m_axi_awburst;
logic     [C_NUM_SLAVES-1 : 0] m_axi_awlock;
logic   [C_NUM_SLAVES*4-1 : 0] m_axi_awcache;
logic   [C_NUM_SLAVES*3-1 : 0] m_axi_awprot;
logic   [C_NUM_SLAVES*4-1 : 0] m_axi_awqos;
logic     [C_NUM_SLAVES-1 : 0] m_axi_awvalid;
logic     [C_NUM_SLAVES-1 : 0] m_axi_awready;
logic [C_NUM_SLAVES*512-1 : 0] m_axi_wdata;
logic  [C_NUM_SLAVES*64-1 : 0] m_axi_wstrb;
logic     [C_NUM_SLAVES-1 : 0] m_axi_wlast;
logic     [C_NUM_SLAVES-1 : 0] m_axi_wvalid;
logic     [C_NUM_SLAVES-1 : 0] m_axi_wready;
logic   [C_NUM_SLAVES*3-1 : 0] m_axi_bid;
logic   [C_NUM_SLAVES*2-1 : 0] m_axi_bresp;
logic     [C_NUM_SLAVES-1 : 0] m_axi_bvalid;
logic     [C_NUM_SLAVES-1 : 0] m_axi_bready;
logic   [C_NUM_SLAVES*3-1 : 0] m_axi_arid;
logic  [C_NUM_SLAVES*64-1 : 0] m_axi_araddr;
logic   [C_NUM_SLAVES*8-1 : 0] m_axi_arlen;
logic   [C_NUM_SLAVES*3-1 : 0] m_axi_arsize;
logic   [C_NUM_SLAVES*2-1 : 0] m_axi_arburst;
logic     [C_NUM_SLAVES-1 : 0] m_axi_arlock;
logic   [C_NUM_SLAVES*4-1 : 0] m_axi_arcache;
logic   [C_NUM_SLAVES*3-1 : 0] m_axi_arprot;
logic   [C_NUM_SLAVES*4-1 : 0] m_axi_arqos;
logic     [C_NUM_SLAVES-1 : 0] m_axi_arvalid;
logic     [C_NUM_SLAVES-1 : 0] m_axi_arready;
logic   [C_NUM_SLAVES*3-1 : 0] m_axi_rid;
logic [C_NUM_SLAVES*512-1 : 0] m_axi_rdata;
logic   [C_NUM_SLAVES*2-1 : 0] m_axi_rresp;
logic     [C_NUM_SLAVES-1 : 0] m_axi_rlast;
logic     [C_NUM_SLAVES-1 : 0] m_axi_rvalid;
logic     [C_NUM_SLAVES-1 : 0] m_axi_rready;

// AXI slave signals for getting wqe from system memory
//assign s_axi_awid   [C_RDMA_GET_WQE_IDX *2 +: 2]         = {1'b0, s_axi_rdma_get_wqe_awid};
assign s_axi_awid   [C_RDMA_GET_WQE_IDX *3 +: 3]         = s_axi_rdma_get_wqe_awvalid ? 3'd0 : 3'd0;
assign s_axi_awaddr [C_RDMA_GET_WQE_IDX *64 +: 64]       = s_axi_rdma_get_wqe_awaddr;
assign s_axi_awqos  [C_RDMA_GET_WQE_IDX *4 +: 4]         = s_axi_rdma_get_wqe_awqos;
assign s_axi_awlen  [C_RDMA_GET_WQE_IDX *8 +: 8]         = s_axi_rdma_get_wqe_awlen;
assign s_axi_awsize [C_RDMA_GET_WQE_IDX *3 +: 3]         = s_axi_rdma_get_wqe_awsize;
assign s_axi_awburst[C_RDMA_GET_WQE_IDX *2 +: 2]         = s_axi_rdma_get_wqe_awburst;
assign s_axi_awcache[C_RDMA_GET_WQE_IDX *4 +: 4]         = s_axi_rdma_get_wqe_awcache;
assign s_axi_awprot [C_RDMA_GET_WQE_IDX *3 +: 3]         = s_axi_rdma_get_wqe_awprot;
assign s_axi_awvalid[C_RDMA_GET_WQE_IDX *1 +: 1]         = s_axi_rdma_get_wqe_awvalid;
assign s_axi_rdma_get_wqe_awready                        = s_axi_awready[C_RDMA_GET_WQE_IDX *1 +: 1];
assign s_axi_wdata  [C_RDMA_GET_WQE_IDX *512 +: 512]     = s_axi_rdma_get_wqe_wdata;
assign s_axi_wstrb  [C_RDMA_GET_WQE_IDX *64 +: 64]       = s_axi_rdma_get_wqe_wstrb;
assign s_axi_wlast  [C_RDMA_GET_WQE_IDX *1 +: 1]         = s_axi_rdma_get_wqe_wlast;
assign s_axi_wvalid [C_RDMA_GET_WQE_IDX *1 +: 1]         = s_axi_rdma_get_wqe_wvalid;
assign s_axi_rdma_get_wqe_wready                         = s_axi_wready[C_RDMA_GET_WQE_IDX *1 +: 1];
assign s_axi_awlock [C_RDMA_GET_WQE_IDX *1 +: 1]         = s_axi_rdma_get_wqe_awlock;
assign s_axi_rdma_get_wqe_bid                            = s_axi_bid[C_RDMA_GET_WQE_IDX *3 +: 1];
assign s_axi_rdma_get_wqe_bresp                          = s_axi_bresp[C_RDMA_GET_WQE_IDX *2 +: 2];
assign s_axi_rdma_get_wqe_bvalid                         = s_axi_bvalid[C_RDMA_GET_WQE_IDX *1 +: 1];
assign s_axi_bready [C_RDMA_GET_WQE_IDX *1 +: 1]         = s_axi_rdma_get_wqe_bready;
//assign s_axi_arid   [C_RDMA_GET_WQE_IDX *2 +: 2]         = {1'b0, s_axi_rdma_get_wqe_arid};
assign s_axi_arid   [C_RDMA_GET_WQE_IDX *3 +: 3]         = s_axi_rdma_get_wqe_arvalid ? 3'd0 : 3'd0;
assign s_axi_araddr [C_RDMA_GET_WQE_IDX *64 +: 64]       = s_axi_rdma_get_wqe_araddr;
assign s_axi_arlen  [C_RDMA_GET_WQE_IDX *8  +: 8]        = s_axi_rdma_get_wqe_arlen;
assign s_axi_arsize [C_RDMA_GET_WQE_IDX *3  +: 3]        = s_axi_rdma_get_wqe_arsize;
assign s_axi_arburst[C_RDMA_GET_WQE_IDX *2  +: 2]        = s_axi_rdma_get_wqe_arburst;
assign s_axi_arcache[C_RDMA_GET_WQE_IDX *4  +: 4]        = s_axi_rdma_get_wqe_arcache;
assign s_axi_arprot [C_RDMA_GET_WQE_IDX *3  +: 3]        = s_axi_rdma_get_wqe_arprot;
assign s_axi_arvalid[C_RDMA_GET_WQE_IDX *1  +: 1]        = s_axi_rdma_get_wqe_arvalid;
assign s_axi_rdma_get_wqe_arready                        = s_axi_arready[C_RDMA_GET_WQE_IDX *1 +: 1];
assign s_axi_rdma_get_wqe_rid                            = s_axi_rid[C_RDMA_GET_WQE_IDX *3 +: 1];
assign s_axi_rdma_get_wqe_rdata                          = s_axi_rdata[C_RDMA_GET_WQE_IDX *512 +: 512];
assign s_axi_rdma_get_wqe_rresp                          = s_axi_rresp[C_RDMA_GET_WQE_IDX *2 +: 2];
assign s_axi_rdma_get_wqe_rlast                          = s_axi_rlast[C_RDMA_GET_WQE_IDX *1 +: 1];
assign s_axi_rdma_get_wqe_rvalid                         = s_axi_rvalid[C_RDMA_GET_WQE_IDX *1 +: 1];
assign s_axi_rready [C_RDMA_GET_WQE_IDX *1 +: 1]         = s_axi_rdma_get_wqe_rready;
assign s_axi_arlock [C_RDMA_GET_WQE_IDX *1 +: 1]         = s_axi_rdma_get_wqe_arlock;
assign s_axi_arqos  [C_RDMA_GET_WQE_IDX *4 +: 4]         = s_axi_rdma_get_wqe_arqos;

// AXI slave signals for getting payload from system memory
//assign axi_awid   [C_RDMA_GET_PAYLOAD_IDX*2 +: 2]      = {1'b0, s_axi_rdma_get_payload_awid};
assign s_axi_awid   [C_RDMA_GET_PAYLOAD_IDX*3 +: 3]      = s_axi_rdma_get_payload_awvalid ? 3'd1 : 3'd0;
assign s_axi_awaddr [C_RDMA_GET_PAYLOAD_IDX *64 +: 64]   = s_axi_rdma_get_payload_awaddr;
assign s_axi_awqos  [C_RDMA_GET_PAYLOAD_IDX *4 +: 4]     = s_axi_rdma_get_payload_awqos;
assign s_axi_awlen  [C_RDMA_GET_PAYLOAD_IDX *8 +: 8]     = s_axi_rdma_get_payload_awlen;
assign s_axi_awsize [C_RDMA_GET_PAYLOAD_IDX *3 +: 3]     = s_axi_rdma_get_payload_awsize;
assign s_axi_awburst[C_RDMA_GET_PAYLOAD_IDX *2 +: 2]     = s_axi_rdma_get_payload_awburst;
assign s_axi_awcache[C_RDMA_GET_PAYLOAD_IDX *4 +: 4]     = s_axi_rdma_get_payload_awcache;
assign s_axi_awprot [C_RDMA_GET_PAYLOAD_IDX *3 +: 3]     = s_axi_rdma_get_payload_awprot;
assign s_axi_awvalid[C_RDMA_GET_PAYLOAD_IDX *1 +: 1]     = s_axi_rdma_get_payload_awvalid;
assign s_axi_rdma_get_payload_awready                    = s_axi_awready[C_RDMA_GET_PAYLOAD_IDX *1 +: 1];
assign s_axi_wdata  [C_RDMA_GET_PAYLOAD_IDX *512 +: 512] = s_axi_rdma_get_payload_wdata;
assign s_axi_wstrb  [C_RDMA_GET_PAYLOAD_IDX *64 +: 64]   = s_axi_rdma_get_payload_wstrb;
assign s_axi_wlast  [C_RDMA_GET_PAYLOAD_IDX *1 +: 1]     = s_axi_rdma_get_payload_wlast;
assign s_axi_wvalid [C_RDMA_GET_PAYLOAD_IDX *1 +: 1]     = s_axi_rdma_get_payload_wvalid;
assign s_axi_rdma_get_payload_wready                     = s_axi_wready[C_RDMA_GET_PAYLOAD_IDX *1 +: 1];
assign s_axi_awlock [C_RDMA_GET_PAYLOAD_IDX *1 +: 1]     = s_axi_rdma_get_payload_awlock;
assign s_axi_rdma_get_payload_bid                        = s_axi_bid[C_RDMA_GET_PAYLOAD_IDX *3 +: 1];
assign s_axi_rdma_get_payload_bresp                      = s_axi_bresp[C_RDMA_GET_PAYLOAD_IDX *2 +: 2];
assign s_axi_rdma_get_payload_bvalid                     = s_axi_bvalid[C_RDMA_GET_PAYLOAD_IDX *1 +: 1];
assign s_axi_bready [C_RDMA_GET_PAYLOAD_IDX *1 +: 1]     = s_axi_rdma_get_payload_bready;
//assign s_axi_arid   [C_RDMA_GET_PAYLOAD_IDX *2 +: 2]     = {1'b0, s_axi_rdma_get_payload_arid};
assign s_axi_arid   [C_RDMA_GET_PAYLOAD_IDX *3 +: 3]     = s_axi_rdma_get_payload_arvalid ? 3'd1: 3'd0;
assign s_axi_araddr [C_RDMA_GET_PAYLOAD_IDX *64 +: 64]   = s_axi_rdma_get_payload_araddr;
assign s_axi_arlen  [C_RDMA_GET_PAYLOAD_IDX *8  +: 8]    = s_axi_rdma_get_payload_arlen;
assign s_axi_arsize [C_RDMA_GET_PAYLOAD_IDX *3  +: 3]    = s_axi_rdma_get_payload_arsize;
assign s_axi_arburst[C_RDMA_GET_PAYLOAD_IDX *2  +: 2]    = s_axi_rdma_get_payload_arburst;
assign s_axi_arcache[C_RDMA_GET_PAYLOAD_IDX *4  +: 4]    = s_axi_rdma_get_payload_arcache;
assign s_axi_arprot [C_RDMA_GET_PAYLOAD_IDX *3  +: 3]    = s_axi_rdma_get_payload_arprot;
assign s_axi_arvalid[C_RDMA_GET_PAYLOAD_IDX *1  +: 1]    = s_axi_rdma_get_payload_arvalid;
assign s_axi_rdma_get_payload_arready                    = s_axi_arready[C_RDMA_GET_PAYLOAD_IDX *1 +: 1];
assign s_axi_rdma_get_payload_rid = rdma_get_payload_rword[515];
assign s_axi_rdma_get_payload_rdata = rdma_get_payload_rword[511:0];
assign s_axi_rdma_get_payload_rresp = rdma_get_payload_rword[514:513];
assign s_axi_rdma_get_payload_rlast = rdma_get_payload_rword[512];
assign s_axi_rdma_get_payload_rvalid = rdma_get_payload_rvalid_pipe;
// RREADY is driven by the local response slice below.
assign s_axi_arlock [C_RDMA_GET_PAYLOAD_IDX *1 +: 1]     = s_axi_rdma_get_payload_arlock;
assign s_axi_arqos  [C_RDMA_GET_PAYLOAD_IDX *4 +: 4]     = s_axi_rdma_get_payload_arqos;

// AXI slave signals for data access from qdma mm channel
//assign axi_awid   [C_RDMA_COMPLETION_IDX*2 +: 2]       = {1'b0, s_axi_rdma_completion_awid};
assign s_axi_awid   [C_RDMA_COMPLETION_IDX*3 +: 3]       = s_axi_rdma_completion_awvalid ? 3'd2 : 3'd0;
assign s_axi_awaddr [C_RDMA_COMPLETION_IDX *64 +: 64]    = s_axi_rdma_completion_awaddr;
assign s_axi_awqos  [C_RDMA_COMPLETION_IDX *4 +: 4]      = s_axi_rdma_completion_awqos;
assign s_axi_awlen  [C_RDMA_COMPLETION_IDX *8 +: 8]      = s_axi_rdma_completion_awlen;
assign s_axi_awsize [C_RDMA_COMPLETION_IDX *3 +: 3]      = s_axi_rdma_completion_awsize;
assign s_axi_awburst[C_RDMA_COMPLETION_IDX *2 +: 2]      = s_axi_rdma_completion_awburst;
assign s_axi_awcache[C_RDMA_COMPLETION_IDX *4 +: 4]      = s_axi_rdma_completion_awcache;
assign s_axi_awprot [C_RDMA_COMPLETION_IDX *3 +: 3]      = s_axi_rdma_completion_awprot;
assign s_axi_awvalid[C_RDMA_COMPLETION_IDX *1 +: 1]      = s_axi_rdma_completion_awvalid;
assign s_axi_rdma_completion_awready                     = s_axi_awready[C_RDMA_COMPLETION_IDX *1 +: 1];
assign s_axi_wdata  [C_RDMA_COMPLETION_IDX *512 +: 512]  = s_axi_rdma_completion_wdata;
assign s_axi_wstrb  [C_RDMA_COMPLETION_IDX *64 +: 64]    = s_axi_rdma_completion_wstrb;
assign s_axi_wlast  [C_RDMA_COMPLETION_IDX *1 +: 1]      = s_axi_rdma_completion_wlast;
assign s_axi_wvalid [C_RDMA_COMPLETION_IDX *1 +: 1]      = s_axi_rdma_completion_wvalid;
assign s_axi_rdma_completion_wready                      = s_axi_wready[C_RDMA_COMPLETION_IDX *1 +: 1];
assign s_axi_awlock [C_RDMA_COMPLETION_IDX *1 +: 1]      = s_axi_rdma_completion_awlock;
assign s_axi_rdma_completion_bid                         = s_axi_bid[C_RDMA_COMPLETION_IDX *3 +: 1];
assign s_axi_rdma_completion_bresp                       = s_axi_bresp[C_RDMA_COMPLETION_IDX *2 +: 2];
assign s_axi_rdma_completion_bvalid                      = s_axi_bvalid[C_RDMA_COMPLETION_IDX *1 +: 1];
assign s_axi_bready [C_RDMA_COMPLETION_IDX *1 +: 1]      = s_axi_rdma_completion_bready;
//assign s_axi_arid   [C_RDMA_COMPLETION_IDX *2 +: 2]      = {1'b0, s_axi_rdma_completion_arid};
assign s_axi_arid   [C_RDMA_COMPLETION_IDX *3 +: 3]      = s_axi_rdma_completion_arvalid ? 3'd2 : 3'd0;
assign s_axi_araddr [C_RDMA_COMPLETION_IDX *64 +: 64]    = s_axi_rdma_completion_araddr;
assign s_axi_arlen  [C_RDMA_COMPLETION_IDX *8  +: 8]     = s_axi_rdma_completion_arlen;
assign s_axi_arsize [C_RDMA_COMPLETION_IDX *3  +: 3]     = s_axi_rdma_completion_arsize;
assign s_axi_arburst[C_RDMA_COMPLETION_IDX *2  +: 2]     = s_axi_rdma_completion_arburst;
assign s_axi_arcache[C_RDMA_COMPLETION_IDX *4  +: 4]     = s_axi_rdma_completion_arcache;
assign s_axi_arprot [C_RDMA_COMPLETION_IDX *3  +: 3]     = s_axi_rdma_completion_arprot;
assign s_axi_arvalid[C_RDMA_COMPLETION_IDX *1  +: 1]     = s_axi_rdma_completion_arvalid;
assign s_axi_rdma_completion_arready                     = s_axi_arready[C_RDMA_COMPLETION_IDX *1 +: 1];
assign s_axi_rdma_completion_rid                         = s_axi_rid[C_RDMA_COMPLETION_IDX *3 +: 1];
assign s_axi_rdma_completion_rdata                       = s_axi_rdata[C_RDMA_COMPLETION_IDX *512 +: 512];
assign s_axi_rdma_completion_rresp                       = s_axi_rresp[C_RDMA_COMPLETION_IDX *2 +: 2];
assign s_axi_rdma_completion_rlast                       = s_axi_rlast[C_RDMA_COMPLETION_IDX *1 +: 1];
assign s_axi_rdma_completion_rvalid                      = s_axi_rvalid[C_RDMA_COMPLETION_IDX *1 +: 1];
assign s_axi_rready [C_RDMA_COMPLETION_IDX *1 +: 1]      = s_axi_rdma_completion_rready;
assign s_axi_arlock [C_RDMA_COMPLETION_IDX *1 +: 1]      = s_axi_rdma_completion_arlock;
assign s_axi_arqos  [C_RDMA_COMPLETION_IDX *4 +: 4]      = s_axi_rdma_completion_arqos;

// AXI slave signals for storing payload from RDMA send or write
//assign axi_awid   [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *2 +: 2]         = {1'b0, s_axi_rdma_send_write_payload_awid};
assign s_axi_awid   [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *3 +: 3]         = s_axi_rdma_send_write_payload_awvalid ? 3'd4 : 3'd0;
assign s_axi_awaddr [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *64 +: 64]       = s_axi_rdma_send_write_payload_awaddr;
assign s_axi_awqos  [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *4 +: 4]         = s_axi_rdma_send_write_payload_awqos;
assign s_axi_awlen  [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *8 +: 8]         = s_axi_rdma_send_write_payload_awlen;
assign s_axi_awsize [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *3 +: 3]         = s_axi_rdma_send_write_payload_awsize;
assign s_axi_awburst[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *2 +: 2]         = s_axi_rdma_send_write_payload_awburst;
assign s_axi_awcache[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *4 +: 4]         = s_axi_rdma_send_write_payload_awcache;
assign s_axi_awprot [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *3 +: 3]         = s_axi_rdma_send_write_payload_awprot;
assign s_axi_awvalid[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1]         = s_axi_rdma_send_write_payload_awvalid;
assign s_axi_rdma_send_write_payload_awready                            = s_axi_awready[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1];
assign s_axi_wdata  [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *512 +: 512]     = s_axi_rdma_send_write_payload_wdata;
assign s_axi_wstrb  [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *64 +: 64]       = s_axi_rdma_send_write_payload_wstrb;
assign s_axi_wlast  [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1]         = s_axi_rdma_send_write_payload_wlast;
assign s_axi_wvalid [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1]         = s_axi_rdma_send_write_payload_wvalid;
assign s_axi_rdma_send_write_payload_wready                             = s_axi_wready[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1];
assign s_axi_awlock [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1]         = s_axi_rdma_send_write_payload_awlock;
assign s_axi_rdma_send_write_payload_bid                                = s_axi_bid[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *3 +: 1];
assign s_axi_rdma_send_write_payload_bresp                              = s_axi_bresp[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *2 +: 2];
assign s_axi_rdma_send_write_payload_bvalid                             = s_axi_bvalid[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1];
assign s_axi_bready [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1]         = s_axi_rdma_send_write_payload_bready;
//assign s_axi_arid   [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *2 +: 2]         = {1'b0, s_axi_rdma_send_write_payload_arid};
assign s_axi_arid   [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *3 +: 3]         = s_axi_rdma_send_write_payload_arvalid ? 3'd4 : 3'd0;
assign s_axi_araddr [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *64 +: 64]       = s_axi_rdma_send_write_payload_araddr;
assign s_axi_arlen  [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *8  +: 8]        = s_axi_rdma_send_write_payload_arlen;
assign s_axi_arsize [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *3  +: 3]        = s_axi_rdma_send_write_payload_arsize;
assign s_axi_arburst[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *2  +: 2]        = s_axi_rdma_send_write_payload_arburst;
assign s_axi_arcache[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *4  +: 4]        = s_axi_rdma_send_write_payload_arcache;
assign s_axi_arprot [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *3  +: 3]        = s_axi_rdma_send_write_payload_arprot;
assign s_axi_arvalid[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1  +: 1]        = s_axi_rdma_send_write_payload_arvalid;
assign s_axi_rdma_send_write_payload_arready                            = s_axi_arready[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1];
assign s_axi_rdma_send_write_payload_rid                                = s_axi_rid[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *3 +: 1];
assign s_axi_rdma_send_write_payload_rdata                              = s_axi_rdata[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *512 +: 512];
assign s_axi_rdma_send_write_payload_rresp                              = s_axi_rresp[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *2 +: 2];
assign s_axi_rdma_send_write_payload_rlast                              = s_axi_rlast[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1];
assign s_axi_rdma_send_write_payload_rvalid                             = s_axi_rvalid[C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1];
assign s_axi_rready [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1]         = s_axi_rdma_send_write_payload_rready;
assign s_axi_arlock [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *1 +: 1]         = s_axi_rdma_send_write_payload_arlock;
assign s_axi_arqos  [C_RDMA_GET_SEND_WRITE_PAYLOAD_IDX *4 +: 4]         = s_axi_rdma_send_write_payload_arqos;

// AXI slave signals for storing payload from RDMA read response
//assign axi_awid   [C_RDMA_RSP_PAYLOAD_IDX*2 +: 2]       = {1'b0, s_axi_rdma_rsp_payload_awid};
assign s_axi_awid   [C_RDMA_RSP_PAYLOAD_IDX*3 +: 3]       = s_axi_rdma_rsp_payload_awvalid ? 3'd3 : 3'd0;
assign s_axi_awaddr [C_RDMA_RSP_PAYLOAD_IDX *64 +: 64]    = s_axi_rdma_rsp_payload_awaddr;
assign s_axi_awqos  [C_RDMA_RSP_PAYLOAD_IDX *4 +: 4]      = s_axi_rdma_rsp_payload_awqos;
assign s_axi_awlen  [C_RDMA_RSP_PAYLOAD_IDX *8 +: 8]      = s_axi_rdma_rsp_payload_awlen;
assign s_axi_awsize [C_RDMA_RSP_PAYLOAD_IDX *3 +: 3]      = s_axi_rdma_rsp_payload_awsize;
assign s_axi_awburst[C_RDMA_RSP_PAYLOAD_IDX *2 +: 2]      = s_axi_rdma_rsp_payload_awburst;
assign s_axi_awcache[C_RDMA_RSP_PAYLOAD_IDX *4 +: 4]      = s_axi_rdma_rsp_payload_awcache;
assign s_axi_awprot [C_RDMA_RSP_PAYLOAD_IDX *3 +: 3]      = s_axi_rdma_rsp_payload_awprot;
assign s_axi_awvalid[C_RDMA_RSP_PAYLOAD_IDX *1 +: 1]      = s_axi_rdma_rsp_payload_awvalid;
assign s_axi_rdma_rsp_payload_awready                     = s_axi_awready[C_RDMA_RSP_PAYLOAD_IDX *1 +: 1];
assign s_axi_wdata  [C_RDMA_RSP_PAYLOAD_IDX *512 +: 512]  = s_axi_rdma_rsp_payload_wdata;
assign s_axi_wstrb  [C_RDMA_RSP_PAYLOAD_IDX *64 +: 64]    = s_axi_rdma_rsp_payload_wstrb;
assign s_axi_wlast  [C_RDMA_RSP_PAYLOAD_IDX *1 +: 1]      = s_axi_rdma_rsp_payload_wlast;
assign s_axi_wvalid [C_RDMA_RSP_PAYLOAD_IDX *1 +: 1]      = s_axi_rdma_rsp_payload_wvalid;
assign s_axi_rdma_rsp_payload_wready                      = s_axi_wready[C_RDMA_RSP_PAYLOAD_IDX *1 +: 1];
assign s_axi_awlock [C_RDMA_RSP_PAYLOAD_IDX *1 +: 1]      = s_axi_rdma_rsp_payload_awlock;
assign s_axi_rdma_rsp_payload_bid                         = s_axi_bid[C_RDMA_RSP_PAYLOAD_IDX *3 +: 1];
assign s_axi_rdma_rsp_payload_bresp                       = s_axi_bresp[C_RDMA_RSP_PAYLOAD_IDX *2 +: 2];
assign s_axi_rdma_rsp_payload_bvalid                      = s_axi_bvalid[C_RDMA_RSP_PAYLOAD_IDX *1 +: 1];
assign s_axi_bready [C_RDMA_RSP_PAYLOAD_IDX *1 +: 1]      = s_axi_rdma_rsp_payload_bready;
//assign axi_arid   [C_RDMA_RSP_PAYLOAD_IDX *2 +: 2]      = {1'b0, s_axi_rdma_rsp_payload_arid};
assign s_axi_arid   [C_RDMA_RSP_PAYLOAD_IDX *3 +: 3]      = s_axi_rdma_rsp_payload_arvalid ? 3'd3 : 3'd0;
assign s_axi_araddr [C_RDMA_RSP_PAYLOAD_IDX *64 +: 64]    = s_axi_rdma_rsp_payload_araddr;
assign s_axi_arlen  [C_RDMA_RSP_PAYLOAD_IDX *8  +: 8]     = s_axi_rdma_rsp_payload_arlen;
assign s_axi_arsize [C_RDMA_RSP_PAYLOAD_IDX *3  +: 3]     = s_axi_rdma_rsp_payload_arsize;
assign s_axi_arburst[C_RDMA_RSP_PAYLOAD_IDX *2  +: 2]     = s_axi_rdma_rsp_payload_arburst;
assign s_axi_arcache[C_RDMA_RSP_PAYLOAD_IDX *4  +: 4]     = s_axi_rdma_rsp_payload_arcache;
assign s_axi_arprot [C_RDMA_RSP_PAYLOAD_IDX *3  +: 3]     = s_axi_rdma_rsp_payload_arprot;
assign s_axi_arvalid[C_RDMA_RSP_PAYLOAD_IDX *1  +: 1]     = s_axi_rdma_rsp_payload_arvalid;
assign s_axi_rdma_rsp_payload_arready                     = s_axi_arready[C_RDMA_RSP_PAYLOAD_IDX *1 +: 1];
assign s_axi_rdma_rsp_payload_rid                         = s_axi_rid[C_RDMA_RSP_PAYLOAD_IDX *3 +: 1];
assign s_axi_rdma_rsp_payload_rdata                       = s_axi_rdata[C_RDMA_RSP_PAYLOAD_IDX *512 +: 512];
assign s_axi_rdma_rsp_payload_rresp                       = s_axi_rresp[C_RDMA_RSP_PAYLOAD_IDX *2 +: 2];
assign s_axi_rdma_rsp_payload_rlast                       = s_axi_rlast[C_RDMA_RSP_PAYLOAD_IDX *1 +: 1];
assign s_axi_rdma_rsp_payload_rvalid                      = s_axi_rvalid[C_RDMA_RSP_PAYLOAD_IDX *1 +: 1];
assign s_axi_rready [C_RDMA_RSP_PAYLOAD_IDX *1 +: 1]      = s_axi_rdma_rsp_payload_rready;
assign s_axi_arlock [C_RDMA_RSP_PAYLOAD_IDX *1 +: 1]      = s_axi_rdma_rsp_payload_arlock;
assign s_axi_arqos  [C_RDMA_RSP_PAYLOAD_IDX *4 +: 4]      = s_axi_rdma_rsp_payload_arqos;

// AXI slave signals for storing payload from RDMA read response
//assign axi_awid   [C_QDMA_MM_IDX*2 +: 2]       = {1'b0, s_axi_qdma_mm_awid};
assign s_axi_awid   [C_QDMA_MM_IDX*3 +: 3]       = s_axi_qdma_mm_awvalid ? 3'd3 : 3'd0;
assign s_axi_awaddr [C_QDMA_MM_IDX *64 +: 64]    = s_axi_qdma_mm_awaddr;
assign s_axi_awqos  [C_QDMA_MM_IDX *4 +: 4]      = s_axi_qdma_mm_awqos;
assign s_axi_awlen  [C_QDMA_MM_IDX *8 +: 8]      = s_axi_qdma_mm_awlen;
assign s_axi_awsize [C_QDMA_MM_IDX *3 +: 3]      = s_axi_qdma_mm_awsize;
assign s_axi_awburst[C_QDMA_MM_IDX *2 +: 2]      = s_axi_qdma_mm_awburst;
assign s_axi_awcache[C_QDMA_MM_IDX *4 +: 4]      = s_axi_qdma_mm_awcache;
assign s_axi_awprot [C_QDMA_MM_IDX *3 +: 3]      = s_axi_qdma_mm_awprot;
assign s_axi_awvalid[C_QDMA_MM_IDX *1 +: 1]      = s_axi_qdma_mm_awvalid;
assign s_axi_qdma_mm_awready                     = s_axi_awready[C_QDMA_MM_IDX *1 +: 1];
assign s_axi_wdata  [C_QDMA_MM_IDX *512 +: 512]  = s_axi_qdma_mm_wdata;
assign s_axi_wstrb  [C_QDMA_MM_IDX *64 +: 64]    = s_axi_qdma_mm_wstrb;
assign s_axi_wlast  [C_QDMA_MM_IDX *1 +: 1]      = s_axi_qdma_mm_wlast;
assign s_axi_wvalid [C_QDMA_MM_IDX *1 +: 1]      = s_axi_qdma_mm_wvalid;
assign s_axi_qdma_mm_wready                      = s_axi_wready[C_QDMA_MM_IDX *1 +: 1];
assign s_axi_awlock [C_QDMA_MM_IDX *1 +: 1]      = s_axi_qdma_mm_awlock;
assign s_axi_qdma_mm_bid                         = s_axi_bid[C_QDMA_MM_IDX *3 +: 1];
assign s_axi_qdma_mm_bresp                       = s_axi_bresp[C_QDMA_MM_IDX *2 +: 2];
assign s_axi_qdma_mm_bvalid                      = s_axi_bvalid[C_QDMA_MM_IDX *1 +: 1];
assign s_axi_bready [C_QDMA_MM_IDX *1 +: 1]      = s_axi_qdma_mm_bready;
//assign axi_arid   [C_QDMA_MM_IDX *2 +: 2]      = {1'b0, s_axi_qdma_mm_arid};
assign s_axi_arid   [C_QDMA_MM_IDX *3 +: 3]      = s_axi_qdma_mm_arvalid ? 3'd3 : 3'd0;
assign s_axi_araddr [C_QDMA_MM_IDX *64 +: 64]    = s_axi_qdma_mm_araddr;
assign s_axi_arlen  [C_QDMA_MM_IDX *8  +: 8]     = s_axi_qdma_mm_arlen;
assign s_axi_arsize [C_QDMA_MM_IDX *3  +: 3]     = s_axi_qdma_mm_arsize;
assign s_axi_arburst[C_QDMA_MM_IDX *2  +: 2]     = s_axi_qdma_mm_arburst;
assign s_axi_arcache[C_QDMA_MM_IDX *4  +: 4]     = s_axi_qdma_mm_arcache;
assign s_axi_arprot [C_QDMA_MM_IDX *3  +: 3]     = s_axi_qdma_mm_arprot;
assign s_axi_arvalid[C_QDMA_MM_IDX *1  +: 1]     = s_axi_qdma_mm_arvalid;
assign s_axi_qdma_mm_arready                     = s_axi_arready[C_QDMA_MM_IDX *1 +: 1];
assign s_axi_qdma_mm_rid = qdma_mm_rword[515];
assign s_axi_qdma_mm_rdata = qdma_mm_rword[511:0];
assign s_axi_qdma_mm_rresp = qdma_mm_rword[514:513];
assign s_axi_qdma_mm_rlast = qdma_mm_rword[512];
assign s_axi_qdma_mm_rvalid = qdma_mm_rvalid_pipe;
// RREADY is driven by the local response slice below.
assign s_axi_arlock [C_QDMA_MM_IDX *1 +: 1]      = s_axi_qdma_mm_arlock;
assign s_axi_arqos  [C_QDMA_MM_IDX *4 +: 4]      = s_axi_qdma_mm_arqos;

// AXI slave signals for the CameraLink capture writer.  The capture path is
// write-only, so the read channel is tied idle.
assign s_axi_awid   [C_CAMERA_IDX *3 +: 3]       = s_axi_camera_awid;
assign s_axi_awaddr [C_CAMERA_IDX *64 +: 64]     = s_axi_camera_awaddr;
assign s_axi_awqos  [C_CAMERA_IDX *4 +: 4]       = s_axi_camera_awqos;
assign s_axi_awlen  [C_CAMERA_IDX *8 +: 8]       = s_axi_camera_awlen;
assign s_axi_awsize [C_CAMERA_IDX *3 +: 3]       = s_axi_camera_awsize;
assign s_axi_awburst[C_CAMERA_IDX *2 +: 2]       = s_axi_camera_awburst;
assign s_axi_awcache[C_CAMERA_IDX *4 +: 4]       = s_axi_camera_awcache;
assign s_axi_awprot [C_CAMERA_IDX *3 +: 3]       = s_axi_camera_awprot;
assign s_axi_awvalid[C_CAMERA_IDX *1 +: 1]       = s_axi_camera_awvalid;
assign s_axi_camera_awready                      = s_axi_awready[C_CAMERA_IDX *1 +: 1];
assign s_axi_wdata  [C_CAMERA_IDX *512 +: 512]   = s_axi_camera_wdata;
assign s_axi_wstrb  [C_CAMERA_IDX *64 +: 64]     = s_axi_camera_wstrb;
assign s_axi_wlast  [C_CAMERA_IDX *1 +: 1]       = s_axi_camera_wlast;
assign s_axi_wvalid [C_CAMERA_IDX *1 +: 1]       = s_axi_camera_wvalid;
assign s_axi_camera_wready                       = s_axi_wready[C_CAMERA_IDX *1 +: 1];
assign s_axi_awlock [C_CAMERA_IDX *1 +: 1]       = s_axi_camera_awlock;
assign s_axi_camera_bid                          = s_axi_bid[C_CAMERA_IDX *3 +: 3];
assign s_axi_camera_bresp                        = s_axi_bresp[C_CAMERA_IDX *2 +: 2];
assign s_axi_camera_bvalid                       = s_axi_bvalid[C_CAMERA_IDX *1 +: 1];
assign s_axi_bready [C_CAMERA_IDX *1 +: 1]       = s_axi_camera_bready;
assign s_axi_arid   [C_CAMERA_IDX *3 +: 3]       = 3'd0;
assign s_axi_araddr [C_CAMERA_IDX *64 +: 64]     = 64'd0;
assign s_axi_arlen  [C_CAMERA_IDX *8  +: 8]      = 8'd0;
assign s_axi_arsize [C_CAMERA_IDX *3  +: 3]      = 3'd0;
assign s_axi_arburst[C_CAMERA_IDX *2  +: 2]      = 2'd0;
assign s_axi_arcache[C_CAMERA_IDX *4  +: 4]      = 4'd0;
assign s_axi_arprot [C_CAMERA_IDX *3  +: 3]      = 3'd0;
assign s_axi_arvalid[C_CAMERA_IDX *1  +: 1]      = 1'b0;
assign s_axi_rready [C_CAMERA_IDX *1 +: 1]       = 1'b1;
assign s_axi_arlock [C_CAMERA_IDX *1 +: 1]       = 1'b0;
assign s_axi_arqos  [C_CAMERA_IDX *4 +: 4]       = 4'd0;


// Independent host write port to the PCIe address range.
assign s_axi_awid[C_CAMERA_HOST_IDX * 3 +: 3] = 3'd0;
assign s_axi_awaddr[C_CAMERA_HOST_IDX * 64 +: 64] = s_axi_camera_host_awaddr;
assign s_axi_awqos[C_CAMERA_HOST_IDX * 4 +: 4] = 4'd0;
assign s_axi_awlen[C_CAMERA_HOST_IDX * 8 +: 8] = s_axi_camera_host_awlen;
assign s_axi_awsize[C_CAMERA_HOST_IDX * 3 +: 3] = 3'd6;
assign s_axi_awburst[C_CAMERA_HOST_IDX * 2 +: 2] = 2'b01;
assign s_axi_awcache[C_CAMERA_HOST_IDX * 4 +: 4] = 4'b0011;
assign s_axi_awprot[C_CAMERA_HOST_IDX * 3 +: 3] = 3'd0;
assign s_axi_awvalid[C_CAMERA_HOST_IDX] = s_axi_camera_host_awvalid;
assign s_axi_wdata[C_CAMERA_HOST_IDX * 512 +: 512] = s_axi_camera_host_wdata;
assign s_axi_wstrb[C_CAMERA_HOST_IDX * 64 +: 64] = s_axi_camera_host_wstrb;
assign s_axi_wlast[C_CAMERA_HOST_IDX] = s_axi_camera_host_wlast;
assign s_axi_wvalid[C_CAMERA_HOST_IDX] = s_axi_camera_host_wvalid;
assign s_axi_awlock[C_CAMERA_HOST_IDX] = 1'b0;
assign s_axi_bready[C_CAMERA_HOST_IDX] = s_axi_camera_host_bready;
assign s_axi_arid[C_CAMERA_HOST_IDX * 3 +: 3] = 3'd0;
assign s_axi_araddr[C_CAMERA_HOST_IDX * 64 +: 64] = 64'd0;
assign s_axi_arlen[C_CAMERA_HOST_IDX * 8 +: 8] = 8'd0;
assign s_axi_arsize[C_CAMERA_HOST_IDX * 3 +: 3] = 3'd0;
assign s_axi_arburst[C_CAMERA_HOST_IDX * 2 +: 2] = 2'd0;
assign s_axi_arcache[C_CAMERA_HOST_IDX * 4 +: 4] = 4'd0;
assign s_axi_arprot[C_CAMERA_HOST_IDX * 3 +: 3] = 3'd0;
assign s_axi_arvalid[C_CAMERA_HOST_IDX] = 1'b0;
assign s_axi_rready[C_CAMERA_HOST_IDX] = 1'b1;
assign s_axi_arlock[C_CAMERA_HOST_IDX] = 1'b0;
assign s_axi_arqos[C_CAMERA_HOST_IDX * 4 +: 4] = 4'd0;
assign s_axi_camera_host_awready = s_axi_awready[C_CAMERA_HOST_IDX];
assign s_axi_camera_host_wready = s_axi_wready[C_CAMERA_HOST_IDX];
assign s_axi_camera_host_bresp = s_axi_bresp[C_CAMERA_HOST_IDX * 2 +: 2];
assign s_axi_camera_host_bvalid = s_axi_bvalid[C_CAMERA_HOST_IDX];

//AXI signals to system memory
assign m_axi_sys_mem_awid                     = m_axi_awid   [C_SYS_MEM_IDX *3 +: 3];
assign m_axi_sys_mem_awaddr                   = m_axi_awaddr [C_SYS_MEM_IDX *64 +: 64];
assign m_axi_sys_mem_awqos                    = m_axi_awqos  [C_SYS_MEM_IDX *4 +: 4];
assign m_axi_sys_mem_awlen                    = m_axi_awlen  [C_SYS_MEM_IDX *8 +: 8];
assign m_axi_sys_mem_awsize                   = m_axi_awsize [C_SYS_MEM_IDX *3 +: 3];
assign m_axi_sys_mem_awburst                  = m_axi_awburst[C_SYS_MEM_IDX *2 +: 2];
assign m_axi_sys_mem_awcache                  = m_axi_awcache[C_SYS_MEM_IDX *4 +: 4];
assign m_axi_sys_mem_awprot                   = m_axi_awprot [C_SYS_MEM_IDX *3 +: 3];
assign m_axi_sys_mem_awvalid                  = m_axi_awvalid[C_SYS_MEM_IDX *1 +: 1];
assign m_axi_awready[C_SYS_MEM_IDX *1 +: 1]   = m_axi_sys_mem_awready;
assign m_axi_sys_mem_wdata                    = m_axi_wdata  [C_SYS_MEM_IDX *512 +: 512];
assign m_axi_sys_mem_wstrb                    = m_axi_wstrb  [C_SYS_MEM_IDX *64 +: 64];
assign m_axi_sys_mem_wlast                    = m_axi_wlast  [C_SYS_MEM_IDX *1 +: 1];
assign m_axi_sys_mem_wvalid                   = m_axi_wvalid [C_SYS_MEM_IDX *1 +: 1];
assign m_axi_wready[C_SYS_MEM_IDX *1 +: 1]    = m_axi_sys_mem_wready;
assign m_axi_sys_mem_awlock                   = m_axi_awlock [C_SYS_MEM_IDX *1 +: 1];
assign m_axi_bid[C_SYS_MEM_IDX *3 +: 3]       = m_axi_sys_mem_bid;
assign m_axi_bresp[C_SYS_MEM_IDX *2 +: 2]     = m_axi_sys_mem_bresp;
assign m_axi_bvalid[C_SYS_MEM_IDX *1 +: 1]    = m_axi_sys_mem_bvalid;
assign m_axi_sys_mem_bready                   = m_axi_bready [C_SYS_MEM_IDX *1 +: 1];
//assign m_axi_sys_mem_arid                   = m_axi_arid   [C_SYS_MEM_IDX *2 +: 2];
assign m_axi_sys_mem_arid                     = m_axi_arid   [C_SYS_MEM_IDX *3 +: 3];
assign m_axi_sys_mem_araddr                   = m_axi_araddr [C_SYS_MEM_IDX *64 +: 64];
assign m_axi_sys_mem_arlen                    = m_axi_arlen  [C_SYS_MEM_IDX *8  +: 8];
assign m_axi_sys_mem_arsize                   = m_axi_arsize [C_SYS_MEM_IDX *3  +: 3];
assign m_axi_sys_mem_arburst                  = m_axi_arburst[C_SYS_MEM_IDX *2  +: 2];
assign m_axi_sys_mem_arcache                  = m_axi_arcache[C_SYS_MEM_IDX *4  +: 4];
assign m_axi_sys_mem_arprot                   = m_axi_arprot [C_SYS_MEM_IDX *3  +: 3];
assign m_axi_sys_mem_arvalid                  = m_axi_arvalid[C_SYS_MEM_IDX *1  +: 1];
assign m_axi_arready[C_SYS_MEM_IDX *1 +: 1]   = m_axi_sys_mem_arready;
assign m_axi_rid[C_SYS_MEM_IDX *3 +: 3]       = m_axi_sys_mem_rid;
assign m_axi_rdata[C_SYS_MEM_IDX *512 +: 512] = m_axi_sys_mem_rdata;
assign m_axi_rresp[C_SYS_MEM_IDX *2 +: 2]     = m_axi_sys_mem_rresp;
assign m_axi_rlast[C_SYS_MEM_IDX *1 +: 1]     = m_axi_sys_mem_rlast;
assign m_axi_rvalid[C_SYS_MEM_IDX *1 +: 1]    = m_axi_sys_mem_rvalid;
assign m_axi_sys_mem_rready                   = m_axi_rready [C_SYS_MEM_IDX *1 +: 1];
assign m_axi_sys_mem_arlock                   = m_axi_arlock [C_SYS_MEM_IDX *1 +: 1];
assign m_axi_sys_mem_arqos                    = m_axi_arqos  [C_SYS_MEM_IDX *4 +: 4];

//AXI signals to device memory
//assign m_axi_awid   [C_SYS_TO_DEV_CROSSBAR_IDX *2 +: 2]         = {1'b0, m_axi_sys_to_dev_crossbar_awid};
assign m_axi_sys_to_dev_crossbar_awid         = m_axi_awid   [C_SYS_TO_DEV_CROSSBAR_IDX *3 +: 3];
assign m_axi_sys_to_dev_crossbar_awaddr       = m_axi_awaddr [C_SYS_TO_DEV_CROSSBAR_IDX *64 +: 64];
assign m_axi_sys_to_dev_crossbar_awqos        = m_axi_awqos  [C_SYS_TO_DEV_CROSSBAR_IDX *4 +: 4];
assign m_axi_sys_to_dev_crossbar_awlen        = m_axi_awlen  [C_SYS_TO_DEV_CROSSBAR_IDX *8 +: 8];
assign m_axi_sys_to_dev_crossbar_awsize       = m_axi_awsize [C_SYS_TO_DEV_CROSSBAR_IDX *3 +: 3];
assign m_axi_sys_to_dev_crossbar_awburst      = m_axi_awburst[C_SYS_TO_DEV_CROSSBAR_IDX *2 +: 2];
assign m_axi_sys_to_dev_crossbar_awcache      = m_axi_awcache[C_SYS_TO_DEV_CROSSBAR_IDX *4 +: 4];
assign m_axi_sys_to_dev_crossbar_awprot       = m_axi_awprot [C_SYS_TO_DEV_CROSSBAR_IDX *3 +: 3];
assign m_axi_sys_to_dev_crossbar_awvalid      = m_axi_awvalid[C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1];
assign m_axi_awready[C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1]          = m_axi_sys_to_dev_crossbar_awready;
assign m_axi_sys_to_dev_crossbar_wdata        = m_axi_wdata  [C_SYS_TO_DEV_CROSSBAR_IDX *512 +: 512];
assign m_axi_sys_to_dev_crossbar_wstrb        = m_axi_wstrb  [C_SYS_TO_DEV_CROSSBAR_IDX *64 +: 64];
assign m_axi_sys_to_dev_crossbar_wlast        = m_axi_wlast  [C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1];
assign m_axi_sys_to_dev_crossbar_wvalid       = m_axi_wvalid [C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1];
assign m_axi_wready[C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1]           = m_axi_sys_to_dev_crossbar_wready;
assign m_axi_sys_to_dev_crossbar_awlock       = m_axi_awlock [C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1];
assign m_axi_bid[C_SYS_TO_DEV_CROSSBAR_IDX *3 +: 3]              = m_axi_sys_to_dev_crossbar_bid;
assign m_axi_bresp[C_SYS_TO_DEV_CROSSBAR_IDX *2 +: 2]            = m_axi_sys_to_dev_crossbar_bresp;
assign m_axi_bvalid[C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1]           = m_axi_sys_to_dev_crossbar_bvalid;
assign m_axi_sys_to_dev_crossbar_bready       = m_axi_bready [C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1];
//assign m_axi_sys_to_dev_crossbar_arid         = m_axi_arid   [C_SYS_TO_DEV_CROSSBAR_IDX *2 +: 2];
assign m_axi_sys_to_dev_crossbar_arid         = m_axi_arid   [C_SYS_TO_DEV_CROSSBAR_IDX *3 +: 3];
assign m_axi_sys_to_dev_crossbar_araddr       = m_axi_araddr [C_SYS_TO_DEV_CROSSBAR_IDX *64 +: 64];
assign m_axi_sys_to_dev_crossbar_arlen        = m_axi_arlen  [C_SYS_TO_DEV_CROSSBAR_IDX *8  +: 8];
assign m_axi_sys_to_dev_crossbar_arsize       = m_axi_arsize [C_SYS_TO_DEV_CROSSBAR_IDX *3  +: 3];
assign m_axi_sys_to_dev_crossbar_arburst      = m_axi_arburst[C_SYS_TO_DEV_CROSSBAR_IDX *2  +: 2];
assign m_axi_sys_to_dev_crossbar_arcache      = m_axi_arcache[C_SYS_TO_DEV_CROSSBAR_IDX *4  +: 4];
assign m_axi_sys_to_dev_crossbar_arprot       = m_axi_arprot [C_SYS_TO_DEV_CROSSBAR_IDX *3  +: 3];
assign m_axi_sys_to_dev_crossbar_arvalid      = m_axi_arvalid[C_SYS_TO_DEV_CROSSBAR_IDX *1  +: 1];
assign m_axi_arready[C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1]          = m_axi_sys_to_dev_crossbar_arready;
assign m_axi_rid[C_SYS_TO_DEV_CROSSBAR_IDX *3 +: 3]              = m_axi_sys_to_dev_crossbar_rid;
assign m_axi_rdata[C_SYS_TO_DEV_CROSSBAR_IDX *512 +: 512]        = m_axi_sys_to_dev_crossbar_rdata;
assign m_axi_rresp[C_SYS_TO_DEV_CROSSBAR_IDX *2 +: 2]            = m_axi_sys_to_dev_crossbar_rresp;
assign m_axi_rlast[C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1]            = m_axi_sys_to_dev_crossbar_rlast;
assign m_axi_rvalid[C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1]           = m_axi_sys_to_dev_crossbar_rvalid;
assign m_axi_sys_to_dev_crossbar_rready       = m_axi_rready [C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1];
assign m_axi_sys_to_dev_crossbar_arlock       = m_axi_arlock [C_SYS_TO_DEV_CROSSBAR_IDX *1 +: 1];
assign m_axi_sys_to_dev_crossbar_arqos        = m_axi_arqos  [C_SYS_TO_DEV_CROSSBAR_IDX *4 +: 4];

sys_mem_5to2_axi_crossbar sys_mem_5to2_axi_crossbar_inst (
  .m_axi_awaddr    (m_axi_awaddr),
  .m_axi_awprot    (m_axi_awprot),
  .m_axi_awvalid   (m_axi_awvalid),
  .m_axi_awready   (m_axi_awready),
  .m_axi_awsize    (m_axi_awsize),
  .m_axi_awburst   (m_axi_awburst),
  .m_axi_awcache   (m_axi_awcache),
  .m_axi_awlen     (m_axi_awlen),
  .m_axi_awlock    (m_axi_awlock),
  .m_axi_awqos     (m_axi_awqos),
  .m_axi_awregion  (m_axi_awregion),
  .m_axi_awid      (m_axi_awid),
  .m_axi_wdata     (m_axi_wdata),
  .m_axi_wstrb     (m_axi_wstrb),
  .m_axi_wvalid    (m_axi_wvalid),
  .m_axi_wready    (m_axi_wready),
  .m_axi_wlast     (m_axi_wlast),
  .m_axi_bresp     (m_axi_bresp),
  .m_axi_bvalid    (m_axi_bvalid),
  .m_axi_bready    (m_axi_bready),
  .m_axi_bid       (m_axi_bid),
  .m_axi_araddr    (m_axi_araddr),
  .m_axi_arprot    (m_axi_arprot),
  .m_axi_arvalid   (m_axi_arvalid),
  .m_axi_arready   (m_axi_arready),
  .m_axi_arsize    (m_axi_arsize),
  .m_axi_arburst   (m_axi_arburst),
  .m_axi_arcache   (m_axi_arcache),
  .m_axi_arlock    (m_axi_arlock),
  .m_axi_arlen     (m_axi_arlen),
  .m_axi_arqos     (m_axi_arqos),
  .m_axi_arregion  (m_axi_arregion),
  .m_axi_arid      (m_axi_arid),
  .m_axi_rdata     (m_axi_rdata),
  .m_axi_rresp     (m_axi_rresp),
  .m_axi_rvalid    (m_axi_rvalid),
  .m_axi_rready    (m_axi_rready),
  .m_axi_rlast     (m_axi_rlast),
  .m_axi_rid       (m_axi_rid),

  .s_axi_awid      (s_axi_awid),
  .s_axi_awaddr    (s_axi_awaddr),
  .s_axi_awqos     (s_axi_awqos),
  .s_axi_awlen     (s_axi_awlen),
  .s_axi_awsize    (s_axi_awsize),
  .s_axi_awburst   (s_axi_awburst),
  .s_axi_awcache   (s_axi_awcache),
  .s_axi_awprot    (s_axi_awprot),
  .s_axi_awvalid   (s_axi_awvalid),
  .s_axi_awready   (s_axi_awready),
  .s_axi_wdata     (s_axi_wdata),
  .s_axi_wstrb     (s_axi_wstrb),
  .s_axi_wlast     (s_axi_wlast),
  .s_axi_wvalid    (s_axi_wvalid),
  .s_axi_wready    (s_axi_wready),
  .s_axi_awlock    (s_axi_awlock),
  .s_axi_bid       (s_axi_bid),
  .s_axi_bresp     (s_axi_bresp),
  .s_axi_bvalid    (s_axi_bvalid),
  .s_axi_bready    (s_axi_bready),
  .s_axi_arid      (s_axi_arid),
  .s_axi_araddr    (s_axi_araddr),
  .s_axi_arlen     (s_axi_arlen),
  .s_axi_arsize    (s_axi_arsize),
  .s_axi_arburst   (s_axi_arburst),
  .s_axi_arcache   (s_axi_arcache),
  .s_axi_arprot    (s_axi_arprot),
  .s_axi_arvalid   (s_axi_arvalid),
  .s_axi_arready   (s_axi_arready),
  .s_axi_rid       (s_axi_rid),
  .s_axi_rdata     (s_axi_rdata),
  .s_axi_rresp     (s_axi_rresp),
  .s_axi_rlast     (s_axi_rlast),
  .s_axi_rvalid    (s_axi_rvalid),
  .s_axi_rready    (s_axi_rready),
  .s_axi_arlock    (s_axi_arlock),
  .s_axi_arqos     (s_axi_arqos),

  .aclk   (axis_aclk),
  .aresetn(axis_arestn)
);


// Independent BRAM R-channel burst buffer absorbs short consumer pauses.
// Prevent a blocked RDMA/PCIe reader from immediately stalling shared DDR R.
// 1024 x 520 bits = 64 KiB payload plus RID/RRESP/RLAST per reader.
// This is a bounded AXI response FIFO, never a latest-frame/drop queue.
// XPM 2022.2 idle-reset assertions have false startup/empty transitions;
// use default SIM_ASSERT_CHK=0, with explicit AXI ordering/stall/reset tests.
wire [519:0] rdma_get_payload_r_fifo_word;
wire rdma_get_payload_r_fifo_valid, rdma_get_payload_r_fifo_ready;
xpm_fifo_axis #(
    .CLOCKING_MODE("common_clock"),
    .FIFO_MEMORY_TYPE("block"),
    .FIFO_DEPTH(1024),
    .PACKET_FIFO("false"),
    .TDATA_WIDTH(520), .TDEST_WIDTH(1), .TID_WIDTH(1), .TUSER_WIDTH(1),
    .WR_DATA_COUNT_WIDTH(11), .RD_DATA_COUNT_WIDTH(11),
    .PROG_FULL_THRESH(1016), .PROG_EMPTY_THRESH(10),
    .USE_ADV_FEATURES("0000"), .ECC_MODE("no_ecc"), .SIM_ASSERT_CHK(0)
) rdma_get_payload_r_burst_fifo_inst (
    .s_aclk(axis_aclk), .m_aclk(axis_aclk), .s_aresetn(axis_arestn),
    .s_axis_tvalid(s_axi_rvalid[C_RDMA_GET_PAYLOAD_IDX]),
    .s_axis_tready(s_axi_rready[C_RDMA_GET_PAYLOAD_IDX]),
    .s_axis_tdata({2'b00, s_axi_rid[C_RDMA_GET_PAYLOAD_IDX *3 +:3],
                    s_axi_rresp[C_RDMA_GET_PAYLOAD_IDX *2 +:2],
                    s_axi_rlast[C_RDMA_GET_PAYLOAD_IDX], s_axi_rdata[C_RDMA_GET_PAYLOAD_IDX *512 +:512]}),
    .s_axis_tkeep({65{1'b1}}), .s_axis_tstrb({65{1'b1}}),
    .s_axis_tlast(1'b0), .s_axis_tid(1'b0), .s_axis_tdest(1'b0), .s_axis_tuser(1'b0),
    .m_axis_tvalid(rdma_get_payload_r_fifo_valid), .m_axis_tready(rdma_get_payload_r_fifo_ready),
    .m_axis_tdata(rdma_get_payload_r_fifo_word),
    .m_axis_tkeep(), .m_axis_tstrb(), .m_axis_tlast(),
    .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser(),
    .almost_empty_axis(), .almost_full_axis(),
    .rd_data_count_axis(), .wr_data_count_axis(),
    .prog_empty_axis(), .prog_full_axis(),
    .sbiterr_axis(), .dbiterr_axis(), .injectsbiterr_axis(1'b0), .injectdbiterr_axis(1'b0)
);

// Keep the two-slot output slice: downstream READY remains registered.
// Wide payload memories are not reset; only validity/ownership is reset.
axi_stream_register_slice #(
    .TDATA_W(520), .TID_W(1), .TDEST_W(1), .TUSER_W(1),
    .MODE("full"), .RESET_PAYLOAD(0)
) rdma_get_payload_r_pipe_inst (
    .aclk(axis_aclk), .aresetn(axis_arestn),
    .s_axis_tvalid(rdma_get_payload_r_fifo_valid),
    .s_axis_tdata(rdma_get_payload_r_fifo_word),
    .s_axis_tkeep(65'b0), .s_axis_tlast(1'b0),
    .s_axis_tid(1'b0), .s_axis_tdest(1'b0), .s_axis_tuser(1'b0),
    .s_axis_tready(rdma_get_payload_r_fifo_ready),
    .m_axis_tvalid(rdma_get_payload_rvalid_pipe),
    .m_axis_tdata(rdma_get_payload_rword),
    .m_axis_tready(s_axi_rdma_get_payload_rready),
    .m_axis_tkeep(), .m_axis_tlast(), .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser()
);

// Independent BRAM R-channel burst buffer absorbs short consumer pauses.
// Prevent a blocked RDMA/PCIe reader from immediately stalling shared DDR R.
// 1024 x 520 bits = 64 KiB payload plus RID/RRESP/RLAST per reader.
// This is a bounded AXI response FIFO, never a latest-frame/drop queue.
// XPM 2022.2 idle-reset assertions have false startup/empty transitions;
// use default SIM_ASSERT_CHK=0, with explicit AXI ordering/stall/reset tests.
wire [519:0] qdma_mm_r_fifo_word;
wire qdma_mm_r_fifo_valid, qdma_mm_r_fifo_ready;
xpm_fifo_axis #(
    .CLOCKING_MODE("common_clock"),
    .FIFO_MEMORY_TYPE("block"),
    .FIFO_DEPTH(1024),
    .PACKET_FIFO("false"),
    .TDATA_WIDTH(520), .TDEST_WIDTH(1), .TID_WIDTH(1), .TUSER_WIDTH(1),
    .WR_DATA_COUNT_WIDTH(11), .RD_DATA_COUNT_WIDTH(11),
    .PROG_FULL_THRESH(1016), .PROG_EMPTY_THRESH(10),
    .USE_ADV_FEATURES("0000"), .ECC_MODE("no_ecc"), .SIM_ASSERT_CHK(0)
) qdma_mm_r_burst_fifo_inst (
    .s_aclk(axis_aclk), .m_aclk(axis_aclk), .s_aresetn(axis_arestn),
    .s_axis_tvalid(s_axi_rvalid[C_QDMA_MM_IDX]),
    .s_axis_tready(s_axi_rready[C_QDMA_MM_IDX]),
    .s_axis_tdata({2'b00, s_axi_rid[C_QDMA_MM_IDX *3 +:3],
                    s_axi_rresp[C_QDMA_MM_IDX *2 +:2],
                    s_axi_rlast[C_QDMA_MM_IDX], s_axi_rdata[C_QDMA_MM_IDX *512 +:512]}),
    .s_axis_tkeep({65{1'b1}}), .s_axis_tstrb({65{1'b1}}),
    .s_axis_tlast(1'b0), .s_axis_tid(1'b0), .s_axis_tdest(1'b0), .s_axis_tuser(1'b0),
    .m_axis_tvalid(qdma_mm_r_fifo_valid), .m_axis_tready(qdma_mm_r_fifo_ready),
    .m_axis_tdata(qdma_mm_r_fifo_word),
    .m_axis_tkeep(), .m_axis_tstrb(), .m_axis_tlast(),
    .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser(),
    .almost_empty_axis(), .almost_full_axis(),
    .rd_data_count_axis(), .wr_data_count_axis(),
    .prog_empty_axis(), .prog_full_axis(),
    .sbiterr_axis(), .dbiterr_axis(), .injectsbiterr_axis(1'b0), .injectdbiterr_axis(1'b0)
);

// Keep the two-slot output slice: downstream READY remains registered.
// Wide payload memories are not reset; only validity/ownership is reset.
axi_stream_register_slice #(
    .TDATA_W(520), .TID_W(1), .TDEST_W(1), .TUSER_W(1),
    .MODE("full"), .RESET_PAYLOAD(0)
) qdma_mm_r_pipe_inst (
    .aclk(axis_aclk), .aresetn(axis_arestn),
    .s_axis_tvalid(qdma_mm_r_fifo_valid),
    .s_axis_tdata(qdma_mm_r_fifo_word),
    .s_axis_tkeep(65'b0), .s_axis_tlast(1'b0),
    .s_axis_tid(1'b0), .s_axis_tdest(1'b0), .s_axis_tuser(1'b0),
    .s_axis_tready(qdma_mm_r_fifo_ready),
    .m_axis_tvalid(qdma_mm_rvalid_pipe),
    .m_axis_tdata(qdma_mm_rword),
    .m_axis_tready(s_axi_qdma_mm_rready),
    .m_axis_tkeep(), .m_axis_tlast(), .m_axis_tid(), .m_axis_tdest(), .m_axis_tuser()
);

endmodule: axi_6to2_interconnect_to_sys_mem
