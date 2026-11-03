// *************************************************************************
//
// Copyright 2020 Xilinx, Inc.
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
// This module delays the input stream for a single cycle.  It provides two
// working mode: fully-registered mode and forward-only mode, specified using
// the mode string "full" and "forward" respectively.
//
// The fully-registered mode completely isolates the slave and master interface
// from a timing perspective.  It uses two sets of registers, one of which
// buffers data when back-pressure happens.  All the output signals are driven
// by flip-flops.
//
// The forward-only mode uses only one set of registers.  Its `s_axis_tready` is
// driven by combinational logic derived from `m_axis_tready` and internal
// state.  This mode makes it more likely to create a long, timing-critical,
// combinational path on `m_axis_tready`.  Hence, it probably should be used in
// regions with relaxed slack, or in the case that a single cycle delay is
// needed for synchronization purpose.
`timescale 1ns/1ps
module axi_stream_register_slice #(
  parameter int TDATA_W = 512,
  parameter int TID_W   = 8,
  parameter int TDEST_W = 4,
  parameter int TUSER_W = 8,
  parameter     MODE    = "full",
  parameter bit RESET_PAYLOAD = 1,
  // Partition the CMAC TX head-register write enable into 32-bit lanes.
  // Leave the default implementation unchanged for every other slice.
  parameter bit SPLIT_HEAD_WEN = 0
) (
  input                    s_axis_tvalid,
  input      [TDATA_W-1:0] s_axis_tdata,
  input  [(TDATA_W/8)-1:0] s_axis_tkeep,
  input                    s_axis_tlast,
  input        [TID_W-1:0] s_axis_tid,
  input      [TDEST_W-1:0] s_axis_tdest,
  input      [TUSER_W-1:0] s_axis_tuser,
  output                   s_axis_tready,

  output                   m_axis_tvalid,
  output     [TDATA_W-1:0] m_axis_tdata,
  output [(TDATA_W/8)-1:0] m_axis_tkeep,
  output                   m_axis_tlast,
  output       [TID_W-1:0] m_axis_tid,
  output     [TDEST_W-1:0] m_axis_tdest,
  output     [TUSER_W-1:0] m_axis_tuser,
  input                    m_axis_tready,

  input                    aclk,
  input                    aresetn
);

  // Only control/valid must reset. RESET_PAYLOAD=0 avoids thousands of
  // unnecessary reset sinks in wide streaming datapaths; invalid data is ignored.
  initial begin
    if (MODE != "full" && MODE != "forward")
      $fatal(1, "[%m] Unknown mode %s", MODE);
  end

  generate if (MODE == "forward") begin: forward_mode
    reg [TDATA_W-1:0] axis_tdata;
    reg [(TDATA_W/8)-1:0] axis_tkeep;
    reg axis_tlast;
    reg [TUSER_W-1:0] axis_tuser;
    reg [TID_W-1:0] axis_tid;
    reg [TDEST_W-1:0] axis_tdest;
    (* max_fanout = 32 *) reg filled;

    wire accept = s_axis_tvalid && s_axis_tready;
    assign s_axis_tready = !filled || m_axis_tready;
    assign m_axis_tvalid = filled;
    assign {m_axis_tdata,m_axis_tkeep,m_axis_tlast,m_axis_tuser,m_axis_tid,m_axis_tdest} =
           {axis_tdata,axis_tkeep,axis_tlast,axis_tuser,axis_tid,axis_tdest};

    always @(posedge aclk) begin
      if (!aresetn) filled <= 1'b0;
      else if (s_axis_tready) filled <= s_axis_tvalid;
    end
    always @(posedge aclk) begin
      if (RESET_PAYLOAD && !aresetn) begin
        axis_tdata <= 0; axis_tkeep <= 0; axis_tlast <= 0;
        axis_tuser <= 0; axis_tid <= 0; axis_tdest <= 0;
      end else if (accept) begin
        axis_tdata <= s_axis_tdata; axis_tkeep <= s_axis_tkeep;
        axis_tlast <= s_axis_tlast; axis_tuser <= s_axis_tuser;
        axis_tid <= s_axis_tid; axis_tdest <= s_axis_tdest;
      end
    end
  end else begin: full_mode
    reg [TDATA_W-1:0] axis_tdata[0:1];
    reg [(TDATA_W/8)-1:0] axis_tkeep[0:1];
    reg axis_tlast[0:1];
    reg [TUSER_W-1:0] axis_tuser[0:1];
    reg [TID_W-1:0] axis_tid[0:1];
    reg [TDEST_W-1:0] axis_tdest[0:1];
    (* max_fanout = 32 *) reg [1:0] filled;

    wire accept = s_axis_tvalid && s_axis_tready;
    wire consume = m_axis_tvalid && m_axis_tready;
    wire head_from_input = accept && (!filled[0] || m_axis_tready);
    wire tail_from_input = accept && filled[0] && !m_axis_tready;
    wire head_from_tail = filled[1] && m_axis_tready;

    assign s_axis_tready = !filled[1];
    assign m_axis_tvalid = filled[0];
    assign {m_axis_tdata,m_axis_tkeep,m_axis_tlast,m_axis_tuser,m_axis_tid,m_axis_tdest} =
           {axis_tdata[0],axis_tkeep[0],axis_tlast[0],axis_tuser[0],axis_tid[0],axis_tdest[0]};

    always @(posedge aclk) begin
      if (!aresetn) filled <= 2'b00;
      else begin
        case (filled)
          2'b00: if (accept) filled <= 2'b01;
          2'b01: case ({accept,consume})
            2'b10: filled <= 2'b11;
            2'b01: filled <= 2'b00;
            default: begin end
          endcase
          2'b11: if (consume) filled <= 2'b01;
          default: filled <= 2'b00;
        endcase
      end
    end
    // On CMAC TX, a single ready-dependent CE otherwise drives all 512 head
    // data bits plus sideband registers at 322 MHz.  Preserve the two-entry
    // elastic-buffer protocol, but give each 32-bit lane its own physical
    // write-enable and tail-select LUT.  DONT_TOUCH prevents Vivado from
    // merging these identical LUTs back into one high-fanout CE.
    if (SPLIT_HEAD_WEN) begin: split_head_wen
      initial begin
        if (TDATA_W % 32 != 0)
          $fatal(1, "[%m] SPLIT_HEAD_WEN requires TDATA_W divisible by 32");
      end
      for (genvar lane = 0; lane < TDATA_W/32; lane = lane + 1) begin: lane_gen
        (* keep = "true" *) wire lane_write;
        (* keep = "true" *) wire lane_from_tail;
        // I0=valid, I1=head_full, I2=tail_full, I3=downstream_ready.
        // write = tail_full & ready |
        //         valid & !tail_full & (!head_full | ready).
        (* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hFA02)) write_lut (
          .I0(s_axis_tvalid), .I1(filled[0]),
          .I2(filled[1]), .I3(m_axis_tready), .O(lane_write)
        );
        (* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) tail_lut (
          .I0(filled[1]), .I1(m_axis_tready), .O(lane_from_tail)
        );
        always @(posedge aclk) begin
          if (RESET_PAYLOAD && !aresetn) begin
            axis_tdata[0][lane*32 +: 32] <= 0;
            axis_tkeep[0][lane*4 +: 4] <= 0;
          end else if (lane_write) begin
            axis_tdata[0][lane*32 +: 32] <= lane_from_tail ?
                axis_tdata[1][lane*32 +: 32] : s_axis_tdata[lane*32 +: 32];
            axis_tkeep[0][lane*4 +: 4] <= lane_from_tail ?
                axis_tkeep[1][lane*4 +: 4] : s_axis_tkeep[lane*4 +: 4];
          end
        end
      end
      always @(posedge aclk) begin
        if (RESET_PAYLOAD && !aresetn) begin
          axis_tlast[0] <= 0; axis_tuser[0] <= 0;
          axis_tid[0] <= 0; axis_tdest[0] <= 0;
        end else if (head_from_tail) begin
          axis_tlast[0] <= axis_tlast[1]; axis_tuser[0] <= axis_tuser[1];
          axis_tid[0] <= axis_tid[1]; axis_tdest[0] <= axis_tdest[1];
        end else if (head_from_input) begin
          axis_tlast[0] <= s_axis_tlast; axis_tuser[0] <= s_axis_tuser;
          axis_tid[0] <= s_axis_tid; axis_tdest[0] <= s_axis_tdest;
        end
      end
    end else begin: shared_head_wen
    // Keep reset out of the payload write enables when RESET_PAYLOAD=0.
    always @(posedge aclk) begin
      if (RESET_PAYLOAD && !aresetn) begin
        axis_tdata[0] <= 0; axis_tkeep[0] <= 0; axis_tlast[0] <= 0;
        axis_tuser[0] <= 0; axis_tid[0] <= 0; axis_tdest[0] <= 0;
      end else if (head_from_tail) begin
        axis_tdata[0] <= axis_tdata[1]; axis_tkeep[0] <= axis_tkeep[1];
        axis_tlast[0] <= axis_tlast[1]; axis_tuser[0] <= axis_tuser[1];
        axis_tid[0] <= axis_tid[1]; axis_tdest[0] <= axis_tdest[1];
      end else if (head_from_input) begin
        axis_tdata[0] <= s_axis_tdata; axis_tkeep[0] <= s_axis_tkeep;
        axis_tlast[0] <= s_axis_tlast; axis_tuser[0] <= s_axis_tuser;
        axis_tid[0] <= s_axis_tid; axis_tdest[0] <= s_axis_tdest;
      end
    end
    end
    always @(posedge aclk) begin
      if (RESET_PAYLOAD && !aresetn) begin
        axis_tdata[1] <= 0; axis_tkeep[1] <= 0; axis_tlast[1] <= 0;
        axis_tuser[1] <= 0; axis_tid[1] <= 0; axis_tdest[1] <= 0;
      end else if (tail_from_input) begin
        axis_tdata[1] <= s_axis_tdata; axis_tkeep[1] <= s_axis_tkeep;
        axis_tlast[1] <= s_axis_tlast; axis_tuser[1] <= s_axis_tuser;
        axis_tid[1] <= s_axis_tid; axis_tdest[1] <= s_axis_tdest;
      end
    end
  end
  endgenerate
endmodule: axi_stream_register_slice
