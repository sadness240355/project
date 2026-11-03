///////////////////////////////////////////////////////////////////
// (c) Copyright 2013 - 2015 Xilinx, Inc. All rights reserved.   
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
// ////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// xrnic.v
//-----------------------------------------------------------------------------
//
// Filename        : xrnic.v
// Version         : v1.0
// Description     : XRNIC IP top level file 
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
// Naming Conventions:
//      active low signals:                     "*_n"
//      clock signals:                          "clk", "clk_div#", "clk_#x" 
//      reset signals:                          "rst", "rst_n" 
//      parameters:                             "C_*" 
//      user defined types:                     "*_TYPE" 
//      state machine next state:               "*_ns" 
//      state machine current state:            "*_cs" 
//      combinatorial signals:                  "*_com" 
//      pipelined or register delay signals:    "*_d#" 
//      counter signals:                        "*cnt*"
//      clock enable signals:                   "*_ce" 
//      internal version of output port         "*_i"
//      device pins:                            "*_pin" 
//      component instantiations:               "<MODULE>I_<#|FUNC>
//-----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1 ps


module flow_ctrl_mgr (
    input m_axi_aclk,
    input m_axi_aresetn,
    input cmac_rx_clk,
    input cmac_rx_rst,
    input cmac_tx_clk,
    input cmac_tx_rst,

    //AXI streaming interface signals to cmac engine
    output wire [511 : 0] cmac_m_axis_tdata,
    output wire [   63:0] cmac_m_axis_tkeep,
    output wire           cmac_m_axis_tvalid,
    input  wire           cmac_m_axis_tready,
    output wire           cmac_m_axis_tlast,
    output wire           cmac_m_axis_tuser,

    input  wire         dma_s_axis_tvalid,
    input  wire [511:0] dma_s_axis_tdata,
    input  wire [ 63:0] dma_s_axis_tkeep,
    input  wire         dma_s_axis_tlast,
    input  wire [  0:0] dma_s_axis_tuser,
    output wire         dma_s_axis_tready,

    input  wire         ernic_s_axis_tvalid,
    input  wire [511:0] ernic_s_axis_tdata,
    input  wire [ 63:0] ernic_s_axis_tkeep,
    input  wire         ernic_s_axis_tlast,
    output wire         ernic_s_axis_tready,


    // Prioroty flow control
    input  wire [8:0] stat_rx_pause_req,
    output wire [8:0] ctl_tx_pause_req,
    output wire       ctl_tx_resend_pause,

    input wire [ 3:0] roce_priority_s,
    input wire        pfc_en_roce_s,
    input wire        pfc_en_non_roce_s,
    input wire [ 3:0] non_roce_priority_s,
    input wire        roce_rx_buf_full,
    input wire [ 8:0] non_roce_rx_buf_count,
    input wire        non_roce_rx_fifo_full,
    input wire [15:0] non_roce_pause_on_offset,
    input wire [15:0] non_roce_pause_off_offset,
    input wire        disable_priority_chk_s

);
    `include "../../rdma_macros.vh"

    // Local Parameters
    localparam [2:0] IDLE = 3'b000;
    localparam [2:0] RD_ERNIC_FIFO = 3'b001;
    localparam [2:0] CHK_CMAC_PROG_FULL_1 = 3'b010;
    localparam [2:0] CHK_PFC_DMA = 3'b011;
    localparam [2:0] RD_DMA_FIFO = 3'b100;
    localparam [2:0] CHK_CMAC_PROG_FULL_2 = 3'b101;
    localparam [2:0] CHK_PFC_ERNIC = 3'b110;

    wire         dma_fifo_ready;
    reg          dma_fifo_ready_out;
    wire [511:0] dma_fifo_tdata_out;
    wire [ 63:0] dma_fifo_tkeep_out;
    wire         dma_fifo_tlast_out;
    wire         dma_fifo_tvalid_out;
    wire [511:0] dma_fifo_tdata;
    wire [ 63:0] dma_fifo_tkeep;
    wire         dma_fifo_tlast;
    wire         dma_fifo_tvalid;
    wire [  3:0] roce_priority;
    wire         pfc_en_roce;
    wire         pfc_en_non_roce;
    wire [  3:0] non_roce_priority;
    wire         disable_priority_chk;
    wire [ 10:0] pfc_configuration;
    wire         roce_rx_buf_full_s;
    wire         non_roce_rx_buf_full_s;
    wire [  1:0] buf_full;

    wire         ernic_fifo_ready;
    reg          ernic_fifo_ready_out;
    wire [511:0] ernic_fifo_tdata_out;
    wire [ 63:0] ernic_fifo_tkeep_out;
    wire         ernic_fifo_tlast_out;
    wire         ernic_fifo_tvalid_out;
    wire [511:0] ernic_fifo_tdata;
    wire [ 63:0] ernic_fifo_tkeep;
    wire         ernic_fifo_tlast;
    wire         ernic_fifo_tvalid;

    wire [511:0] ernic_reg_tdata_in;
    wire [ 63:0] ernic_reg_tkeep_in;
    wire         ernic_reg_tlast_in;
    wire         ernic_reg_tvalid_in;
    wire         ernic_reg_tready_in;

    wire [511:0] dma_reg_tdata_in;
    wire [ 63:0] dma_reg_tkeep_in;
    wire         dma_reg_tlast_in;
    wire         dma_reg_tvalid_in;
    wire         dma_reg_tready_in;
    wire [576:0] dma_reg_fifo_in;
    wire [576:0] ernic_reg_fifo_in;




    wire [576:0] dma_reg_out;
    wire [576:0] ernic_reg_out;

    wire [  8:0] roce_pause;
    wire [  8:0] non_roce_pause;
    wire         stop_non_roce_tx;
    wire         stop_roce_tx;
    reg  [  2:0] state;

    wire         cmac_prog_full;
    wire         empty_dma_fifo;
    wire         empty_ernic_fifo;
    reg          non_roce_rx_buf_full;
    wire [  8:0] rd_data_count_axis_ernic;
    wire [  8:0] wr_data_count_axis_ernic;
    wire [  8:0] rd_data_count_axis_dma;


    assign ctl_tx_resend_pause = 1'b0;

    // xpm_cdc_array_single: Single-bit Array Synchronizer
    // Xilinx Parameterized Macro, version 2020.1
    xpm_cdc_array_single #(
        .DEST_SYNC_FF(4),  // DECIMAL; range: 2-10
        .INIT_SYNC_FF(0),  // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .SRC_INPUT_REG(1),  // DECIMAL; 0=do not register input, 1=register input
        .WIDTH(11)  // DECIMAL; range: 1-1024
    ) xpm_cdc_array_single_inst (
        .dest_out(pfc_configuration),                                                                                // WIDTH-bit output: src_in synchronized to the destination clock domain. This
        // output is registered.
        .dest_clk(cmac_tx_clk),                                                                                      // 1-bit input: Clock signal for the destination clock domain.
        .src_clk (m_axi_aclk),                                                                                       // 1-bit input: optional; required when SRC_INPUT_REG = 1
        .src_in  ({disable_priority_chk_s, non_roce_priority_s, pfc_en_non_roce_s, roce_priority_s, pfc_en_roce_s})  // WIDTH-bit input: Input single-bit array to be synchronized to destination clock
        // domain. It is assumed that each bit of the array is unrelated to the others. This
        // is reflected in the constraints applied to this macro. To transfer a binary value
        // losslessly across the two clock domains, use the XPM_CDC_GRAY macro instead.
    );
    // End of xpm_cdc_array_single_inst instantiation

    xpm_cdc_array_single #(
        .DEST_SYNC_FF(4),  // DECIMAL; range: 2-10
        .INIT_SYNC_FF(0),  // DECIMAL; 0=disable simulation init values, 1=enable simulation init values
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .SRC_INPUT_REG(1),  // DECIMAL; 0=do not register input, 1=register input
        .WIDTH(2)  // DECIMAL; range: 1-1024
    ) xpm_cdc_array_single_inst_1 (
        .dest_out(buf_full),                                 // WIDTH-bit output: src_in synchronized to the destination clock domain. This
        // output is registered.
        .dest_clk(cmac_tx_clk),                              // 1-bit input: Clock signal for the destination clock domain.
        .src_clk (m_axi_aclk),                               // 1-bit input: optional; required when SRC_INPUT_REG = 1
        .src_in  ({non_roce_rx_buf_full, roce_rx_buf_full})  // WIDTH-bit input: Input single-bit array to be synchronized to destination clock
        // domain. It is assumed that each bit of the array is unrelated to the others. This
        // is reflected in the constraints applied to this macro. To transfer a binary value
        // losslessly across the two clock domains, use the XPM_CDC_GRAY macro instead.
    );
    // End of xpm_cdc_array_single_inst instantiation

    assign roce_rx_buf_full_s     = buf_full[0];
    assign non_roce_rx_buf_full_s = buf_full[1];
    assign roce_priority          = pfc_configuration[4:1];
    assign pfc_en_roce            = pfc_configuration[0];
    assign pfc_en_non_roce        = pfc_configuration[5];
    assign non_roce_priority      = pfc_configuration[9:6];
    assign disable_priority_chk   = pfc_configuration[10];

    // xpm_fifo_axis: AXI Stream FIFO
    // Xilinx Parameterized Macro, version 2020.1
    xpm_fifo_axis #(
        .CDC_SYNC_STAGES(2),  // DECIMAL
        .CLOCKING_MODE("independent_clock"),  // String
        .ECC_MODE("no_ecc"),  // String
        .FIFO_DEPTH(256),  // DECIMAL
        .FIFO_MEMORY_TYPE("auto"),  // String
        .PACKET_FIFO("false"),  // String
        .PROG_EMPTY_THRESH(10),  // DECIMAL
        .PROG_FULL_THRESH(10),  // DECIMAL
        .RD_DATA_COUNT_WIDTH(9),  // DECIMAL
        .RELATED_CLOCKS(0),  // DECIMAL
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .TDATA_WIDTH(512),  // DECIMAL
        .TDEST_WIDTH(1),  // DECIMAL
        .TID_WIDTH(1),  // DECIMAL
        .TUSER_WIDTH(1),  // DECIMAL
        .USE_ADV_FEATURES("1F0F"),  // String
        .WR_DATA_COUNT_WIDTH(9)  // DECIMAL
    ) dma_xpm_fifo_axis_inst (
        .almost_empty_axis (empty_dma_fifo),          // 1-bit output: Almost Empty : When asserted, this signal
        // indicates that only one more read can be performed before the
        // FIFO goes to empty.
        .almost_full_axis  (),                        // 1-bit output: Almost Full: When asserted, this signal
        // indicates that only one more write can be performed before
        // the FIFO is full.
        .dbiterr_axis      (),                        // 1-bit output: Double Bit Error- Indicates that the ECC
        // decoder detected a double-bit error and data in the FIFO core
        // is corrupted.
        .m_axis_tdata      (dma_fifo_tdata),          // TDATA_WIDTH-bit output: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .m_axis_tdest      (),                        // TDEST_WIDTH-bit output: TDEST: Provides routing information
        // for the data stream.
        .m_axis_tid        (),                        // TID_WIDTH-bit output: TID: The data stream identifier that
        // indicates different streams of data.
        .m_axis_tkeep      (dma_fifo_tkeep),          // TDATA_WIDTH/8-bit output: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // and can be removed from the data stream. For a 64-bit DATA,
        // bit 0 corresponds to the least significant byte on DATA, and
        // bit 7 corresponds to the most significant byte. For example:
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .m_axis_tlast      (dma_fifo_tlast),          // 1-bit output: TLAST: Indicates the boundary of a packet.
        .m_axis_tstrb      (),                        // TDATA_WIDTH/8-bit output: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .m_axis_tuser      (),                        // TUSER_WIDTH-bit output: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .m_axis_tvalid     (dma_fifo_tvalid),         // 1-bit output: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
        .prog_empty_axis   (),                        // 1-bit output: Programmable Empty- This signal is asserted
        // when the number of words in the FIFO is less than or equal to
        // the programmable empty threshold value. It is de-asserted
        // when the number of words in the FIFO exceeds the programmable
        // empty threshold value.
        .prog_full_axis    (),                        // 1-bit output: Programmable Full: This signal is asserted when
        // the number of words in the FIFO is greater than or equal to
        // the programmable full threshold value. It is de-asserted when
        // the number of words in the FIFO is less than the programmable
        // full threshold value.
        .rd_data_count_axis(rd_data_count_axis_dma),  // RD_DATA_COUNT_WIDTH-bit output: Read Data Count- This bus
        // indicates the number of words available for reading in the
        // FIFO.
        .s_axis_tready     (dma_reg_tready_in),       // 1-bit output: TREADY: Indicates that the slave can accept a
        // transfer in the current cycle.
        .sbiterr_axis      (),                        // 1-bit output: Single Bit Error- Indicates that the ECC
        // decoder detected and fixed a single-bit error.
        .wr_data_count_axis(),                        // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus
        // indicates the number of words written into the FIFO.
        .injectdbiterr_axis(1'b0),                    // 1-bit input: Double Bit Error Injection- Injects a double bit
        // error if the ECC feature is used.
        .injectsbiterr_axis(1'b0),                    // 1-bit input: Single Bit Error Injection- Injects a single bit
        // error if the ECC feature is used.
        .m_aclk            (cmac_tx_clk),             // 1-bit input: Master Interface Clock: All signals on master
        // interface are sampled on the rising edge of this clock.
        .m_axis_tready     (dma_fifo_ready),          // 1-bit input: TREADY: Indicates that the slave can accept a
                                                      // transfer in the current cycle.
        .s_aclk            (m_axi_aclk),              // 1-bit input: Slave Interface Clock: All signals on slave
        // interface are sampled on the rising edge of this clock.
        .s_aresetn         (m_axi_aresetn),           // 1-bit input: Active low asynchronous reset.
        .s_axis_tdata      (dma_reg_tdata_in),        // TDATA_WIDTH-bit input: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .s_axis_tdest      (1'b0),                    // TDEST_WIDTH-bit input: TDEST: Provides routing information
        // for the data stream.
        .s_axis_tid        (1'b0),                    // TID_WIDTH-bit input: TID: The data stream identifier that
        // indicates different streams of data.
        .s_axis_tkeep      (dma_reg_tkeep_in),        // TDATA_WIDTH/8-bit input: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // and can be removed from the data stream. For a 64-bit DATA,
        // bit 0 corresponds to the least significant byte on DATA, and
        // bit 7 corresponds to the most significant byte. For example:
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .s_axis_tlast      (dma_reg_tlast_in),        // 1-bit input: TLAST: Indicates the boundary of a packet.
        .s_axis_tstrb      ({64{1'b1}}),              // TDATA_WIDTH/8-bit input: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .s_axis_tuser      (dma_reg_tuser_in),        // TUSER_WIDTH-bit input: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .s_axis_tvalid     (dma_reg_tvalid_in)        // 1-bit input: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
    );
    // End of xpm_fifo_axis_inst instantiation

    // xpm_fifo_axis: AXI Stream FIFO
    // Xilinx Parameterized Macro, version 2020.1
    xpm_fifo_axis #(
        .CDC_SYNC_STAGES(2),  // DECIMAL
        .CLOCKING_MODE("independent_clock"),  // String
        .ECC_MODE("no_ecc"),  // String
        .FIFO_DEPTH(256),  // DECIMAL
        .FIFO_MEMORY_TYPE("auto"),  // String
        .PACKET_FIFO("false"),  // String
        .PROG_EMPTY_THRESH(10),  // DECIMAL
        .PROG_FULL_THRESH(10),  // DECIMAL
        .RD_DATA_COUNT_WIDTH(9),  // DECIMAL
        .RELATED_CLOCKS(0),  // DECIMAL
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .TDATA_WIDTH(512),  // DECIMAL
        .TDEST_WIDTH(1),  // DECIMAL
        .TID_WIDTH(1),  // DECIMAL
        .TUSER_WIDTH(1),  // DECIMAL
        .USE_ADV_FEATURES("1F0F"),  // String
        .WR_DATA_COUNT_WIDTH(9)  // DECIMAL
    ) ernic_xpm_fifo_axis_inst (
        .almost_empty_axis (empty_ernic_fifo),          // 1-bit output: Almost Empty : When asserted, this signal
        // indicates that only one more read can be performed before the
        // FIFO goes to empty.
        .almost_full_axis  (),                          // 1-bit output: Almost Full: When asserted, this signal
        // indicates that only one more write can be performed before
        // the FIFO is full.
        .dbiterr_axis      (),                          // 1-bit output: Double Bit Error- Indicates that the ECC
        // decoder detected a double-bit error and data in the FIFO core
        // is corrupted.
        .m_axis_tdata      (ernic_fifo_tdata),          // TDATA_WIDTH-bit output: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .m_axis_tdest      (),                          // TDEST_WIDTH-bit output: TDEST: Provides routing information
        // for the data stream.
        .m_axis_tid        (),                          // TID_WIDTH-bit output: TID: The data stream identifier that
        // indicates different streams of data.
        .m_axis_tkeep      (ernic_fifo_tkeep),          // TDATA_WIDTH/8-bit output: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // and can be removed from the data stream. For a 64-bit DATA,
        // bit 0 corresponds to the least significant byte on DATA, and
        // bit 7 corresponds to the most significant byte. For example:
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .m_axis_tlast      (ernic_fifo_tlast),          // 1-bit output: TLAST: Indicates the boundary of a packet.
        .m_axis_tstrb      (),                          // TDATA_WIDTH/8-bit output: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .m_axis_tuser      (),                          // TUSER_WIDTH-bit output: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .m_axis_tvalid     (ernic_fifo_tvalid),         // 1-bit output: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
        .prog_empty_axis   (),                          // 1-bit output: Programmable Empty- This signal is asserted
        // when the number of words in the FIFO is less than or equal to
        // the programmable empty threshold value. It is de-asserted
        // when the number of words in the FIFO exceeds the programmable
        // empty threshold value.
        .prog_full_axis    (),                          // 1-bit output: Programmable Full: This signal is asserted when
        // the number of words in the FIFO is greater than or equal to
        // the programmable full threshold value. It is de-asserted when
        // the number of words in the FIFO is less than the programmable
        // full threshold value.
        .rd_data_count_axis(rd_data_count_axis_ernic),  // RD_DATA_COUNT_WIDTH-bit output: Read Data Count- This bus
        // indicates the number of words available for reading in the
        // FIFO.
        .s_axis_tready     (ernic_reg_tready_in),       // 1-bit output: TREADY: Indicates that the slave can accept a
        // transfer in the current cycle.
        .sbiterr_axis      (),                          // 1-bit output: Single Bit Error- Indicates that the ECC
        // decoder detected and fixed a single-bit error.
        .wr_data_count_axis(wr_data_count_axis_ernic),  // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus
        // indicates the number of words written into the FIFO.
        .injectdbiterr_axis(1'b0),                      // 1-bit input: Double Bit Error Injection- Injects a double bit
        // error if the ECC feature is used.
        .injectsbiterr_axis(1'b0),                      // 1-bit input: Single Bit Error Injection- Injects a single bit
        // error if the ECC feature is used.
        .m_aclk            (cmac_tx_clk),               // 1-bit input: Master Interface Clock: All signals on master
        // interface are sampled on the rising edge of this clock.
        .m_axis_tready     (ernic_fifo_ready),
        // 1-bit input: TREADY: Indicates that the slave can accept a
        // transfer in the current cycle.
        .s_aclk            (m_axi_aclk),                // 1-bit input: Slave Interface Clock: All signals on slave
        // interface are sampled on the rising edge of this clock.
        .s_aresetn         (m_axi_aresetn),             // 1-bit input: Active low asynchronous reset.
        .s_axis_tdata      (ernic_reg_tdata_in),        // TDATA_WIDTH-bit input: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .s_axis_tdest      (1'b0),                      // TDEST_WIDTH-bit input: TDEST: Provides routing information
        // for the data stream.
        .s_axis_tid        (1'b0),                      // TID_WIDTH-bit input: TID: The data stream identifier that
        // indicates different streams of data.
        .s_axis_tkeep      (ernic_reg_tkeep_in),        // TDATA_WIDTH/8-bit input: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // and can be removed from the data stream. For a 64-bit DATA,
        // bit 0 corresponds to the least significant byte on DATA, and
        // bit 7 corresponds to the most significant byte. For example:
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .s_axis_tlast      (ernic_reg_tlast_in),        // 1-bit input: TLAST: Indicates the boundary of a packet.
        .s_axis_tstrb      ({64{1'b1}}),                // TDATA_WIDTH/8-bit input: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .s_axis_tuser      (1'b0),                      // TUSER_WIDTH-bit input: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .s_axis_tvalid     (ernic_reg_tvalid_in)        // 1-bit input: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
    );
    // End of xpm_fifo_axis_inst instantiation
    reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_dma_in_inst (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({ernic_s_axis_tlast, ernic_s_axis_tkeep, ernic_s_axis_tdata}),
        .S_VALID       (ernic_s_axis_tvalid),
        .S_READY       (ernic_s_axis_tready),
        .M_PAYLOAD_DATA(ernic_reg_fifo_in),
        .M_VALID       (ernic_reg_tvalid_in),
        .M_READY       (ernic_reg_tready_in)
    );
    assign ernic_reg_tdata_in = ernic_reg_fifo_in[511:0];
    assign ernic_reg_tkeep_in = ernic_reg_fifo_in[575:512];
    assign ernic_reg_tlast_in = ernic_reg_fifo_in[576];


    reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_ernic_in_inst (
        .aclk          (m_axi_aclk),
        .areset        (~m_axi_aresetn),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({dma_s_axis_tlast, dma_s_axis_tkeep, dma_s_axis_tdata}),
        .S_VALID       (dma_s_axis_tvalid),
        .S_READY       (dma_s_axis_tready),
        .M_PAYLOAD_DATA(dma_reg_fifo_in),
        .M_VALID       (dma_reg_tvalid_in),
        .M_READY       (dma_reg_tready_in)
    );
    assign dma_reg_tdata_in = dma_reg_fifo_in[511:0];
    assign dma_reg_tkeep_in = dma_reg_fifo_in[575:512];
    assign dma_reg_tlast_in = dma_reg_fifo_in[576];


    reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_dma_inst (
        .aclk          (cmac_tx_clk),
        .areset        (cmac_tx_rst),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({dma_fifo_tlast, dma_fifo_tkeep, dma_fifo_tdata}),
        .S_VALID       (dma_fifo_tvalid),
        .S_READY       (dma_fifo_ready),
        .M_PAYLOAD_DATA(dma_reg_out),
        .M_VALID       (dma_fifo_tvalid_out),
        .M_READY       (dma_fifo_ready_out)
    );
    assign dma_fifo_tdata_out = dma_reg_out[511:0];
    assign dma_fifo_tkeep_out = dma_reg_out[575:512];
    assign dma_fifo_tlast_out = dma_reg_out[576];


    reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_ernic_inst (
        .aclk          (cmac_tx_clk),
        .areset        (cmac_tx_rst),
        .aclken        (1'b1),
        .S_PAYLOAD_DATA({ernic_fifo_tlast, ernic_fifo_tkeep, ernic_fifo_tdata}),
        .S_VALID       (ernic_fifo_tvalid),
        .S_READY       (ernic_fifo_ready),
        .M_PAYLOAD_DATA(ernic_reg_out),
        .M_VALID       (ernic_fifo_tvalid_out),
        .M_READY       (ernic_fifo_ready_out)
    );
    assign ernic_fifo_tdata_out = ernic_reg_out[511:0];
    assign ernic_fifo_tkeep_out = ernic_reg_out[575:512];
    assign ernic_fifo_tlast_out = ernic_reg_out[576];



    // xpm_fifo_axis: AXI Stream FIFO
    // Xilinx Parameterized Macro, version 2020.1
    xpm_fifo_axis #(
        .CDC_SYNC_STAGES(2),  // DECIMAL
        .CLOCKING_MODE("common_clock"),  // String
        .ECC_MODE("no_ecc"),  // String
        .FIFO_DEPTH(256),  // DECIMAL
        .FIFO_MEMORY_TYPE("auto"),  // String
        .PACKET_FIFO("true"),  // String
        .PROG_EMPTY_THRESH(10),  // DECIMAL
        .PROG_FULL_THRESH(256 - 66),  // DECIMAL
        .RD_DATA_COUNT_WIDTH(9),  // DECIMAL
        .RELATED_CLOCKS(0),  // DECIMAL
        .SIM_ASSERT_CHK(0),  // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
        .TDATA_WIDTH(512),  // DECIMAL
        .TDEST_WIDTH(1),  // DECIMAL
        .TID_WIDTH(1),  // DECIMAL
        .TUSER_WIDTH(1),  // DECIMAL
        .USE_ADV_FEATURES("1F1F"),  // String
        .WR_DATA_COUNT_WIDTH(9)  // DECIMAL
    ) cmac_xpm_fifo_axis_inst (
        .almost_empty_axis (),                                                                                               // 1-bit output: Almost Empty : When asserted, this signal
        // indicates that only one more read can be performed before the
        // FIFO goes to empty.
        .almost_full_axis  (),                                                                                               // 1-bit output: Almost Full: When asserted, this signal
        // indicates that only one more write can be performed before
        // the FIFO is full.
        .dbiterr_axis      (),                                                                                               // 1-bit output: Double Bit Error- Indicates that the ECC
        // decoder detected a double-bit error and data in the FIFO core
        // is corrupted.
        .m_axis_tdata      (cmac_m_axis_tdata),                                                                              // TDATA_WIDTH-bit output: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .m_axis_tdest      (),                                                                                               // TDEST_WIDTH-bit output: TDEST: Provides routing information
        // for the data stream.
        .m_axis_tid        (),                                                                                               // TID_WIDTH-bit output: TID: The data stream identifier that
        // indicates different streams of data.
        .m_axis_tkeep      (cmac_m_axis_tkeep),                                                                              // TDATA_WIDTH/8-bit output: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // and can be removed from the data stream. For a 64-bit DATA,
        // bit 0 corresponds to the least significant byte on DATA, and
        // bit 7 corresponds to the most significant byte. For example:
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .m_axis_tlast      (cmac_m_axis_tlast),                                                                              // 1-bit output: TLAST: Indicates the boundary of a packet.
        .m_axis_tstrb      (),                                                                                               // TDATA_WIDTH/8-bit output: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .m_axis_tuser      (cmac_m_axis_tuser),                                                                              // TUSER_WIDTH-bit output: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .m_axis_tvalid     (cmac_m_axis_tvalid),                                                                             // 1-bit output: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
        .prog_empty_axis   (),                                                                                               // 1-bit output: Programmable Empty- This signal is asserted
        // when the number of words in the FIFO is less than or equal to
        // the programmable empty threshold value. It is de-asserted
        // when the number of words in the FIFO exceeds the programmable
        // empty threshold value.
        .prog_full_axis    (cmac_prog_full),                                                                                 // 1-bit output: Programmable Full: This signal is asserted when
                                                                                                                             // the number of words in the FIFO is greater than or equal to
                                                                                                                             // the programmable full threshold value. It is de-asserted when
                                                                                                                             // the number of words in the FIFO is less than the programmable
                                                                                                                             // full threshold value.
        .rd_data_count_axis(),                                                                                               // RD_DATA_COUNT_WIDTH-bit output: Read Data Count- This bus
        // indicates the number of words available for reading in the
        // FIFO.
        .s_axis_tready     (cmac_fifo_s_axis_tready),                                                                        // 1-bit output: TREADY: Indicates that the slave can accept a
        // transfer in the current cycle.
        .sbiterr_axis      (),                                                                                               // 1-bit output: Single Bit Error- Indicates that the ECC
        // decoder detected and fixed a single-bit error.
        .wr_data_count_axis(),                                                                                               // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus
        // indicates the number of words written into the FIFO.
        .injectdbiterr_axis(1'b0),                                                                                           // 1-bit input: Double Bit Error Injection- Injects a double bit
        // error if the ECC feature is used.
        .injectsbiterr_axis(1'b0),                                                                                           // 1-bit input: Single Bit Error Injection- Injects a single bit
        // error if the ECC feature is used.
        .m_aclk            (cmac_tx_clk),                                                                                    // 1-bit input: Master Interface Clock: All signals on master
        // interface are sampled on the rising edge of this clock.
        .m_axis_tready     (cmac_m_axis_tready),                                                                             // 1-bit input: TREADY: Indicates that the slave can accept a
        // transfer in the current cycle.
        .s_aclk            (cmac_tx_clk),                                                                                    // 1-bit input: Slave Interface Clock: All signals on slave
        // interface are sampled on the rising edge of this clock.
        .s_aresetn         (~cmac_tx_rst),                                                                                   // 1-bit input: Active low asynchronous reset.
        .s_axis_tdata      (dma_fifo_ready_out ? dma_fifo_tdata_out : ernic_fifo_tdata_out),                                 // TDATA_WIDTH-bit input: TDATA: The primary payload that is
        // used to provide the data that is passing across the
        // interface. The width of the data payload is an integer number
        // of bytes.
        .s_axis_tdest      (1'b0),                                                                                           // TDEST_WIDTH-bit input: TDEST: Provides routing information
        // for the data stream.
        .s_axis_tid        (1'b0),                                                                                           // TID_WIDTH-bit input: TID: The data stream identifier that
        // indicates different streams of data.
        .s_axis_tkeep      (dma_fifo_ready_out ? dma_fifo_tkeep_out : ernic_fifo_tkeep_out),                                 // TDATA_WIDTH/8-bit input: TKEEP: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as part of the data stream. Associated bytes
        // that have the TKEEP byte qualifier deasserted are null bytes
        // and can be removed from the data stream. For a 64-bit DATA,
        // bit 0 corresponds to the least significant byte on DATA, and
        // bit 7 corresponds to the most significant byte. For example:
        // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
        // DATA[63:56] is a NULL byte
        .s_axis_tlast      (dma_fifo_ready_out ? dma_fifo_tlast_out : ernic_fifo_tlast_out),                                 // 1-bit input: TLAST: Indicates the boundary of a packet.
        .s_axis_tstrb      ({64{1'b1}}),                                                                                     // TDATA_WIDTH/8-bit input: TSTRB: The byte qualifier that
        // indicates whether the content of the associated byte of TDATA
        // is processed as a data byte or a position byte. For a 64-bit
        // DATA, bit 0 corresponds to the least significant byte on
        // DATA, and bit 0 corresponds to the least significant byte on
        // DATA, and bit 7 corresponds to the most significant byte. For
        // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
        // DATA[63:56] is not valid
        .s_axis_tuser      (),                                                                                               // TUSER_WIDTH-bit input: TUSER: The user-defined sideband
        // information that can be transmitted alongside the data
        // stream.
        .s_axis_tvalid     (dma_fifo_ready_out ? dma_fifo_tvalid_out : ernic_fifo_ready_out ? ernic_fifo_tvalid_out : 1'b0)  // 1-bit input: TVALID: Indicates that the master is driving a
        // valid transfer. A transfer takes place when both TVALID and
        // TREADY are asserted
    );
    // End of xpm_fifo_axis_inst instantiation 

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            non_roce_rx_buf_full <= 1'b0;
        end else begin
            if (((256 - non_roce_rx_buf_count) < non_roce_pause_on_offset) | non_roce_rx_buf_count[8]) non_roce_rx_buf_full <= 1'b1;
            else if ((256 - non_roce_rx_buf_count) >= non_roce_pause_off_offset) non_roce_rx_buf_full <= 1'b0;
        end
    end

    // Arbiter
    always @(posedge cmac_tx_clk) begin
        if (cmac_tx_rst) begin
            state                <= IDLE;
            dma_fifo_ready_out   <= 1'b0;
            ernic_fifo_ready_out <= 1'b0;
        end else begin
            case (state)
                IDLE:

                if (ernic_fifo_tvalid_out && !stop_roce_tx) begin
                    state                <= RD_ERNIC_FIFO;
                    ernic_fifo_ready_out <= 1'b1;
                end else if (dma_fifo_tvalid_out && !stop_non_roce_tx) begin
                    state              <= RD_DMA_FIFO;
                    dma_fifo_ready_out <= 1'b1;
                end else begin
                    state                <= IDLE;
                    ernic_fifo_ready_out <= 1'b0;
                    dma_fifo_ready_out   <= 1'b0;
                end

                RD_ERNIC_FIFO:

                if (ernic_fifo_tlast_out) begin

                    state                <= CHK_CMAC_PROG_FULL_1;
                    dma_fifo_ready_out   <= 1'b0;
                    ernic_fifo_ready_out <= 1'b0;


                end else begin
                    state <= RD_ERNIC_FIFO;
                end
                CHK_CMAC_PROG_FULL_1:
                if (!cmac_prog_full) begin
                    state <= CHK_PFC_DMA;
                end else begin
                    state <= CHK_CMAC_PROG_FULL_1;
                end
                CHK_PFC_DMA:
                if (dma_fifo_tvalid_out && !stop_non_roce_tx) begin
                    state              <= RD_DMA_FIFO;
                    dma_fifo_ready_out <= 1'b1;
                end else if (ernic_fifo_tvalid_out && !stop_roce_tx) begin
                    state                <= RD_ERNIC_FIFO;
                    ernic_fifo_ready_out <= 1'b1;
                end else begin
                    state <= IDLE;
                end
                RD_DMA_FIFO:

                if (dma_fifo_tlast_out) begin

                    state                <= CHK_CMAC_PROG_FULL_2;
                    dma_fifo_ready_out   <= 1'b0;
                    ernic_fifo_ready_out <= 1'b0;

                end else begin
                    state <= RD_DMA_FIFO;
                end
                CHK_CMAC_PROG_FULL_2:
                if (!cmac_prog_full) begin
                    state <= CHK_PFC_ERNIC;
                end else begin
                    state <= CHK_CMAC_PROG_FULL_2;
                end

                CHK_PFC_ERNIC:
                if (ernic_fifo_tvalid_out && !stop_roce_tx) begin
                    state                <= RD_ERNIC_FIFO;
                    ernic_fifo_ready_out <= 1'b1;
                end else if (dma_fifo_tvalid_out && !stop_non_roce_tx) begin
                    state              <= RD_DMA_FIFO;
                    dma_fifo_ready_out <= 1'b1;
                end else begin
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

    // define binary to one hot function
    function [8:0] binary_to_onehot;
        input [3:0] binary_no;
        reg [8:0] i;
        begin
            for (i = 0; i < 9; i = i + 1) begin
                binary_to_onehot[i] = (binary_no == i);
            end
        end
    endfunction

    assign roce_pause       = (pfc_en_roce) ? (roce_rx_buf_full_s) ? binary_to_onehot(roce_priority) : 9'b000000000 : 9'b000000000;
    assign non_roce_pause   = (pfc_en_non_roce) ? (non_roce_rx_buf_full_s) ? binary_to_onehot(non_roce_priority) : 9'b000000000 : 9'b000000000;
    assign ctl_tx_pause_req = (roce_pause | non_roce_pause);

    assign stop_roce_tx     = (pfc_en_roce) ? (disable_priority_chk) ? (|stat_rx_pause_req) ? 1'b1 : 1'b0 : (stat_rx_pause_req[8] == 1'b1) ? 1'b1 : (stat_rx_pause_req[roce_priority] == 1'b1) ? 1'b1 : 1'b0 : 1'b0;
    assign stop_non_roce_tx = (pfc_en_non_roce) ? (disable_priority_chk) ? (|stat_rx_pause_req) ? 1'b1 : 1'b0 : (stat_rx_pause_req[8] == 1'b1) ? 1'b1 : (stat_rx_pause_req[non_roce_priority] == 1'b1) ? 1'b1 : 1'b0 : 1'b0;


endmodule
