////////////////////////////////////////////////////////////////////////////////
//
// Evaluation Circuitry // VHDL RTL Design
//////////////////////////////////////////////////-a
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
//
// Author : Himani Upadhyay

// Description:
//   This module is used for the evaluation flow. It creates the CC8CE entity
//   using RTL code, rather than instantiating the CC8CE component. This is
//   because you can't instantiate a macro in HDL without having the underlying
//   EDIF as well.
//


// Revision History 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// 1.1.2.1  - Initial Release

// History:
//  himaniu :       Copied from the can_v5_0 design and updated for
//                            ETRNIC. 
//////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

`timescale 1 ns / 1 ps
//`default_nettype none
//(* DowngradeIPIdentifiedWarnings="yes" *)

module arbchk (
    input  wire       C,
    input  wire       CLR,
    input  wire       CE,
    output wire [6:0] Q,
    output wire       CEO,
    output wire       TC
);

    ////////////////////////////////////////////////////////////////////////////////
    // Definition of Ports
    //   C   : Clock
    //   CLR : Asynchronous clear
    //   CE  : Clock Enable
    //   Q   : Count output
    //   CEO : Clock Enable Output
    //   TC  : Terminal Count
    ////////////////////////////////////////////////////////////////////////////////

    localparam [6:0] TERMINAL_COUNT = 7'b1111111;
    // Must initialize Q_i for simulation, since CLR never gets asserted.
    reg [6:0] Q_i = 7'b0000000;
    reg       TC_i;

    assign TC  = TC_i;
    assign Q   = Q_i;
    assign CEO = TC_i & CE;

    // Create the CC8CE component. This code is based on the libraries
    // guide on s.x.c.
    always @(posedge C) begin
        if (CLR) Q_i <= 7'b00000000;
        else if (CE) Q_i <= Q_i + 1;
    end

    always @(*) begin
        if (Q_i == TERMINAL_COUNT) TC_i = 1'b1;
        else TC_i = 1'b0;
    end


endmodule
