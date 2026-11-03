//////////////////////////////////////////////////-
//  DISCLAIMER OF LIABILITY
// -----------------------------------------------------------------
// (c) Copyright 2014-2015 Xilinx, Inc. All rights reserved.
//
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
//
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
//
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
//
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
////////////////////////////////////////////////////////////////////////

// Author : himaniu

// Description of module
//   This module is used for the evaluation flow. It is a series of counters, 
//   when enabled, will count up to about 4 hours and then turn off the whole
//   core. This will allow hardware evaluation without requiring the purchased
//   core.
//


// Revision History 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// 1.1.2.1  - Initial Release
// 1.1.2.2  - Change in Library Name
// 1.1.2.3  - Library name changed
// History:
//  himaniu:       Copied from the CAN design and updated for
//                            ERNIC.
//  tfischaber 05 July 2002 : Copied from Douglas "gooby" Grant and updated for
//                            GFP. Initial design from Eric Crabill. 
//                   http://scotland/~cc/reports/eval_timeout/eval_timeout.html
////////////////////////////////////////////////////////////////////////////////
`timescale 1 ns / 1 ps `default_nettype none (* DowngradeIPIdentifiedWarnings="yes" *)

module arbit #(
    parameter C_HWTIMEOUT = 1
) (
    input  wire s_axi_lite_aclk,
    input  wire m_axi_aclk,
    input  wire s_axi_lite_aresetn,
    output reg  s_axi_lite_aresetn_sync1,
    output wire s_axi_lite_aresetn_1
    //   input wire TXE_TX_EN,
    //   output wire TXE_ARB_LOST
);

    ////////////////////////////////////////////////////////////////////////////////
    // Definition of Ports
    // s_axi_lite_aclk       : Input clock
    // m_axi_aclk  : input clock for CDC synchronization
    ////////////////////////////////////////////////////////////////////////////////

    wire arb_1;
    wire arb_2;
    wire arb_3;
    wire arb_4;
    wire arb_5;
    wire arb_6;
    reg  arb_1q;
    reg  arb_2q;
    reg  arb_3q;
    reg  arb_4q;
    reg  arb_5q;
    reg  arb_6q;

    localparam zero = 1'b0;
    localparam one = 1'b1;

    generate
        if (C_HWTIMEOUT == 1) begin : ARBGEN1

            arbchk arb1 (
                .C  (s_axi_lite_aclk),
                .CLR(zero),
                .CE (one),
                .Q  (),
                .TC (),
                .CEO(arb_1)
            );

            arbchk arb2 (
                .C  (s_axi_lite_aclk),
                .CLR(zero),
                .CE (arb_1q),
                .Q  (),
                .TC (),
                .CEO(arb_2)
            );

            arbchk arb3 (
                .C  (s_axi_lite_aclk),
                .CLR(zero),
                .CE (arb_2q),
                .Q  (),
                .TC (),
                .CEO(arb_3)
            );

            arbchk arb4 (
                .C  (s_axi_lite_aclk),
                .CLR(zero),
                .CE (arb_3q),
                .Q  (),
                .TC (),
                .CEO(arb_4)
            );

            arbchk arb5 (
                .C  (s_axi_lite_aclk),
                .CLR(zero),
                .CE (arb_4q),
                .Q  (),
                .TC (),
                .CEO(arb_5)
            );

            arbchk arb6 (
                .C  (s_axi_lite_aclk),
                .CLR(zero),
                .CE (arb_5q),
                .Q  (),
                .TC (),
                .CEO(arb_6)
            );

            always @(posedge s_axi_lite_aclk) begin
                arb_1q <= arb_1;
                arb_2q <= arb_2;
                arb_3q <= arb_3;
                arb_4q <= arb_4;
                arb_5q <= arb_5;
                arb_6q <= arb_6;

            end
            assign s_axi_lite_aresetn_1 = arb_6q;

            //end
            //endgenerate

            //CDC synchronizer
            reg s_axi_lite_aresetn_sync;
            //reg s_axi_lite_aresetn_sync1;
            always @(posedge m_axi_aclk) begin
                s_axi_lite_aresetn_sync  <= arb_6q;
                s_axi_lite_aresetn_sync1 <= s_axi_lite_aresetn_sync;
            end
        end
    endgenerate



    generate
        if (C_HWTIMEOUT == 0) begin : ARBGEN0
            always @(posedge s_axi_lite_aclk) begin
                s_axi_lite_aresetn_sync1 <= 1'b0;
            end
            assign s_axi_lite_aresetn_1 = 1'b0;
        end
    endgenerate


endmodule
