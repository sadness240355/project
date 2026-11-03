//==========================================================================
// Change history
//==========================================================================
// 10-Aug-2020 : first version of RTL


// (c) Copyright 2018 Xilinx, Inc. All rights reserved.
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
////////////////////////////////////////////////////////////////////////////////
//
// Vendor         : Xilinx
// Revision       : $Revision: $
// Date           : $DateTime: $
// Last Author    : $Author: ckallak $
//
////////////////////////////////////////////////////////////////////////////////
// Hierarchy :
//    super_module :
//      sub_module :
//
// Description :
// This module is arbitrates between multiple requests in a round-robin
////////////////////////////////////////////////////////////////////////////////
`timescale 1ps/1ps
(* DowngradeIPIdentifiedWarnings="yes" *)
module arbiter #(parameter NUM_REQS   = 4,
                             parameter BUS_HOLD   = 0,
                             parameter NEW_ARB    = 0,
                             parameter SPLIT      = 0,
                             parameter NUM_SPLITS = 4
                             )
   (
    input                         clk_arb,
    input                         rst_n_arb,

    input [NUM_REQS - 1:0]        reqs,
    input                         arb_done,

    output logic [NUM_REQS - 1:0] gnt,
    output logic                  gnt_valid
    );

   //==========================================================================
   // genvars
   //==========================================================================
   genvar                         i, j;

   //==========================================================================
   // local parameters
   //==========================================================================
   localparam REQ_ID      = (NUM_REQS == 1) ? 1 : $clog2(NUM_REQS);
   //localparam REQ_PER_ARB = (SPLIT == 1) ? NUM_REQS/NUM_SPLITS : 1;
   localparam REQ_PER_ARB = (SPLIT == 1) ? (NUM_REQS%NUM_SPLITS != 0) ? (NUM_REQS/NUM_SPLITS) + 1 :
			    (NUM_REQS/NUM_SPLITS) : 1;
   localparam SPLIT_GNTS  = REQ_PER_ARB * NUM_SPLITS;
   
   //==========================================================================
   // wires and register declarations
   //==========================================================================
   
   logic [REQ_ID - 1:0]      curr_gnt_bit;
   logic [REQ_ID - 1:0]      max_req_bit;
   logic [NUM_REQS - 1:0]    next_reqs_curr_lss_max;
   logic [NUM_REQS - 1:0]    next_gnt_curr_lss_max;
   logic [SPLIT_GNTS - 1:0]  int_gnt;
   
   generate //{
      if (NEW_ARB == 0)
        begin : old_arbiter //{

           //==========================================================================
           // current grant bit
           //==========================================================================
           assign curr_gnt_bit = (|gnt) ? setbit_lsb (gnt) : {REQ_ID {1'b0}};

           //==========================================================================
           // MSB which is set in request
           //==========================================================================
           assign max_req_bit = (|reqs) ? setbit_msb (reqs) : {REQ_ID {1'b0}};

           //==========================================================================
           // when there are more pending requests at MSB, than the current grant,
           // grant for the next highest bit
           //==========================================================================
           assign next_reqs_curr_lss_max = {NUM_REQS {1'b1}} << (curr_gnt_bit + 1'b1);
           assign next_gnt_curr_lss_max  = (|(next_reqs_curr_lss_max & reqs)) ? setbit_lsb (next_reqs_curr_lss_max & reqs) :
                                           {REQ_ID {1'b0}};

           always_ff @ (posedge clk_arb)
             begin //{
                if (!rst_n_arb)
                  begin //{
                     gnt <= {NUM_REQS {1'b0}};
                  end //}
                //==========================================================================
                // when there are no current grants, choose the LSB request and grant
                //==========================================================================
                else if ((gnt == {NUM_REQS {1'b0}}) | !reqs[curr_gnt_bit])
                  begin //{
                     for (int i = 0; i < NUM_REQS; i++)
                       begin //{
                          gnt[i] <= (i == setbit_lsb(reqs)) & reqs[i];
                       end //}
                  end //}
                else if (arb_done)
                  begin //{
                     //==========================================================================
                     // if current granted request is the only one:
                     // BUS_HOLD == 0 : next grant = 0
                     // BUS_HOLD == 1 : next grant = current grant
                     //==========================================================================
                     if (gnt == reqs)
                       begin //{
                          gnt <= (BUS_HOLD == 1) ? gnt : {NUM_REQS {1'b0}};
                       end //}
                     //==========================================================================
                     // if there are pending requests at a higher bit number than the current
                     // grant, then assign the next bit that is 1 as the next grant
                     //==========================================================================
                     else if (max_req_bit > curr_gnt_bit)
                       begin //{
                          for (int i = 0; i < NUM_REQS; i++)
                            begin //{
                               gnt[i] <= (i == next_gnt_curr_lss_max);
                            end //}
                       end //}
                     //==========================================================================
                     // if there are no requests higher than the current bit value, go to the LSB
                     // request and give it a grant
                     //==========================================================================
                     else
                       begin //{
                          for (int i = 0; i < NUM_REQS; i++)
                            begin //{
                               gnt[i] <= (i == setbit_lsb(reqs)) & reqs[i];
                            end //}
                       end //}
                  end //}
             end //}
           
           //==========================================================================
           // if at least one grant bit is set then only it's valid
           //==========================================================================
           //generate //{
           if (BUS_HOLD == 0)
             begin : no_bus_hold //{
                assign gnt_valid = |gnt;
             end //}
           else
             begin : en_bus_hold //{
                assign gnt_valid = |((gnt) & (reqs));
             end //}
           //endgenerate //}

           //==========================================================================
           // function to identify the request number
           //==========================================================================
           function [REQ_ID - 1:0] setbit_lsb (input [NUM_REQS - 1:0] in_reqs);
              begin :setbit_lsb_idfy_blk //{
                 for (int i = 0; i < NUM_REQS; i++)
                   begin : calc_block //{
                      if (in_reqs[i])
                        begin //{
                           setbit_lsb = i;
                           disable setbit_lsb_idfy_blk;
                        end //}
                   end //}
              end //}
           endfunction // if

           //==========================================================================
           // function to identify the request number
           //==========================================================================
           function [REQ_ID - 1:0] setbit_msb (input [NUM_REQS - 1:0] in_reqs);
              begin :setbit_msb_idfy_blk //{
                 for (int i = NUM_REQS - 1; i >= 0; i--)
                   begin : calc_block //{
                      if (in_reqs[i])
                        begin //{
                           setbit_msb = i;
                           disable setbit_msb_idfy_blk;
                        end //}
                   end //}
              end //}
           endfunction // if
        end //}
      else if (SPLIT == 0)
        begin : timing_arbiter //{
           round_robin #(.NUM_REQ       (NUM_REQS),
                                     .BUS_PARK_CTRL (BUS_HOLD)
                                     ) u_arbiter
             (
              .clk_arb                       (clk_arb),
              .rst_n_arb                     (rst_n_arb),
              .no_req                        (1'b0),
              .wround_robin_en               (1'b0),
              .round_robin_en                (1'b1),
              .req                           (reqs),
              .done                          ({NUM_REQS {arb_done}}),
              .gnt                           (gnt)
              );

           assign gnt_valid = (BUS_HOLD == 1) ? ((|gnt) & (|reqs)) : |gnt;
           
           
        end //}
      else
        //=====================================================================
        // divides the requests into multiple arbiters depending on NUM_SPLITS
        // to have a better timing
        //=====================================================================
        begin : timing_split_arbiter //{
	   logic [REQ_PER_ARB - 1:0] reqs_for_each_arb [NUM_SPLITS];
	   logic [NUM_SPLITS - 1:0]  gnt_valid_l0;
	   logic [REQ_PER_ARB - 1:0] gnt_l0 [NUM_SPLITS];
	   logic [NUM_SPLITS - 1:0]  arb_done_l0;
	   logic [NUM_SPLITS - 1:0]  gnt_l1;
	   
           for (i = 0; i < NUM_SPLITS; i++)
             begin : split_req //{
                for (j = 0; j < REQ_PER_ARB; j++)
                  begin : num_reqs_pe_arbiter //{
		     if ( ((i * REQ_PER_ARB) + j) >= NUM_REQS)
		       begin //{
			  assign reqs_for_each_arb[i][j] = 1'b0;
		       end //}
		     else
		       begin //{
			  assign reqs_for_each_arb[i][j] = reqs[(i * REQ_PER_ARB) + j];
		       end //}
                  end //}
             end //}

           for (i = 0; i < NUM_SPLITS; i++)
             begin : level0_arbiter //{
		
		
                round_robin #(.NUM_REQ       (REQ_PER_ARB),
                                          .BUS_PARK_CTRL (BUS_HOLD)
                                          ) u_arbiter_l0
                    (
                     .clk_arb                       (clk_arb),
                     .rst_n_arb                     (rst_n_arb),
                     .no_req                        (1'b0),
                     .wround_robin_en               (1'b0),
                     .round_robin_en                (1'b1),
                     .req                           (reqs_for_each_arb[i]),
                     .done                          ({REQ_PER_ARB {arb_done_l0[i]}}), // CK
                     .gnt                           (gnt_l0[i])
                     );

                assign gnt_valid_l0[i] = (BUS_HOLD == 1) ? ((|gnt_l0[i]) & (|reqs_for_each_arb[i])) : 
                                         |gnt_l0[i];

                //=============================================================
                // if grant of level 1 arbiter is for this level 0 arbiter and
		// a done is received then this 
                //=============================================================
                assign arb_done_l0[i] = gnt_l1[i] & arb_done;
             end //}

           //==================================================================
           // final arbiter at level 1
           //==================================================================
           round_robin #(.NUM_REQ       (NUM_SPLITS),
                                     .BUS_PARK_CTRL (BUS_HOLD)
                                     ) u_arbiter_l1
             (
              .clk_arb                       (clk_arb),
              .rst_n_arb                     (rst_n_arb),
              .no_req                        (1'b0),
              .wround_robin_en               (1'b0),
              .round_robin_en                (1'b1),
              .req                           (gnt_valid_l0),
              .done                          ({NUM_SPLITS {arb_done}}),
              .gnt                           (gnt_l1)
              );

           assign gnt_valid = (BUS_HOLD == 1) ? ((|gnt_l1) & (|gnt_valid_l0)) : |gnt_l1;
	   for (i = 0; i < NUM_SPLITS; i++)
	     begin : final_gnt //{
		assign int_gnt[(i * REQ_PER_ARB) +: REQ_PER_ARB] = gnt_l1[i] ? gnt_l0[i] : 'h0;
	     end //}
	   assign gnt = int_gnt[NUM_REQS - 1:0];
	   
        end //}
      endgenerate //}

   
endmodule // arbiter