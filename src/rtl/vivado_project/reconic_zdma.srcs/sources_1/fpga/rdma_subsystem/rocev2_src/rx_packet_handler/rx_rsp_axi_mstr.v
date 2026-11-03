////////////////////////////////////////////////////////////////////
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
// rx_rsp_axi_mstr.v
//-----------------------------------------------------------------------------
//
// Filename        : rx_rsp_axi_mstr.v
// Version         : v1.0
// Description     : Writes the incoming response packets to system memory or RDMAIF 
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

module rx_rsp_axi_mstr #(
    parameter C_SIM_DEBUG = 0,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_QP_INDX_WIDTH = 8,
    parameter NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN = 0,
    parameter C_RSP_BUF_ADDR_WIDTH = 10
) (

    // System level interface
    input wire clk,
    input wire reset,

    // rsp_pkt_hdr_val interface
    input  wire [                    31:0] pkt_id,               // packet number to debug
    input  wire [                     2:0] rsp_pkt_type,         // encoding is given below
    input  wire [  C_M_AXI_ADDR_WIDTH-1:0] rsp_dest_addr,        // destination address
    input  wire [C_RSP_BUF_ADDR_WIDTH-1:0] rsp_buf_addr_offset,  // block RAM source address
    input  wire [                    12:0] rsp_pkt_len,          // total packet length
    input  wire [                    31:0] imm_data,             // immediate data to transfer
                                                                 // holds error syndrome
    input  wire                            tnfr_vld,             // transfer descriptor is valid
    output wire                            rdy_to_tnfr,          // handshake to ready to transfer

    // XRNIC register interface
    input  wire                          xrnic_en_pulse,
    input  wire [C_M_AXI_ADDR_WIDTH-1:0] i_err_buf_ba,
    input  wire [                  15:0] i_err_buf_sz,
    input  wire [                  15:0] i_err_num_bufs,
    output reg  [C_M_AXI_ADDR_WIDTH-1:0] o_err_buf_ca,
    output reg  [                  15:0] o_err_buf_wrptr,
    output reg  [                  15:0] o_rd_req_pkt_cnt,
    output reg  [                  15:0] o_rd_rsp_pkt_cnt,

    output wire incr_inv_pkt_cnt,

    // to interrupt controller
    output wire set_pkt_valdn_err_intr,

    // Response buffer interface
    input  wire [C_RSP_BUF_ADDR_WIDTH-1:0] rsp_pkt_buf_addra,
    output reg                             rsp_pkt_buf_enb,
    output reg  [C_RSP_BUF_ADDR_WIDTH-1:0] rsp_pkt_buf_addrb,
    input  wire [                   511:0] rsp_pkt_buf_doutb,

    // Response handler interface
    output wire                          o_rd_rsp_wr_cmpltd,
    output wire [   C_QP_INDX_WIDTH-1:0] o_rsp_qp_id,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] o_rsp_dest_addr_for_hw_hndshk,

    // AXI Master interface to DDR/NVMOF
    output wire [                   0:0] m_axi_awid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [                   7:0] m_axi_awlen,
    output wire [                   2:0] m_axi_awsize,
    output wire [                   1:0] m_axi_awburst,
    output wire [                   3:0] m_axi_awcache,
    output wire [                   2:0] m_axi_awprot,
    output wire                          m_axi_awvalid,
    input  wire                          m_axi_awready,
    output wire [                 511:0] m_axi_wdata,
    output wire [                  63:0] m_axi_wstrb,
    output wire                          m_axi_wlast,
    output wire                          m_axi_wvalid,
    input  wire                          m_axi_wready,
    output wire                          m_axi_awlock,
    input  wire [                   0:0] m_axi_bid,
    input  wire [                   1:0] m_axi_bresp,
    input  wire                          m_axi_bvalid,
    output wire                          m_axi_bready,
    output wire [                   0:0] m_axi_arid,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [                   7:0] m_axi_arlen,
    output wire [                   2:0] m_axi_arsize,
    output wire [                   1:0] m_axi_arburst,
    output wire [                   3:0] m_axi_arcache,
    output wire [                   2:0] m_axi_arprot,
    output wire                          m_axi_arvalid,
    input  wire                          m_axi_arready,
    input  wire [                   0:0] m_axi_rid,
    input  wire [                 511:0] m_axi_rdata,
    input  wire [                   1:0] m_axi_rresp,
    input  wire                          m_axi_rlast,
    input  wire                          m_axi_rvalid,
    output wire                          m_axi_rready,
    output wire                          m_axi_arlock,

    // Debug signals
    input wire [31:0] free_run_cnt,

    input  wire        i_clr_debug_sts,
    output reg  [15:0] axi_awchnl_fifo_unoccupied_r,
    output reg  [15:0] axi_awchnl_max_wait_st,
    output reg  [15:0] axi_wchnl_fifo_unoccupied_r,
    output reg  [15:0] axi_wchnl_max_wait_st,
    output wire        axi_awchnl_fifo_empty,
    output reg         axi_awchnl_fifo_full_sticky,
    output wire        axi_wchnl_fifo_empty,
    output reg         axi_wchnl_fifo_full_sticky,
    output reg  [15:0] post_axi_tnfr_fifo_unoccupied_r,
    output wire        post_axi_tnfr_fifo_empty,
    output reg         post_axi_tnfr_fifo_full_sticky,
    output reg  [15:0] axi_bresp_max_wait_st
);

    // Local Parameters
    localparam [2:0] RSP_PKT_TYPE_DISCARD = 3'b000;
    localparam [2:0] RSP_PKT_TYPE_ERROR = 3'b001;
    localparam [2:0] RSP_PKT_TYPE_IPV4_RD_RSP_ONLY_OR_LAST = 3'b010;
    localparam [2:0] RSP_PKT_TYPE_IPV4_RD_RSP_FIRST = 3'b011;
    localparam [2:0] RSP_PKT_TYPE_IPV4_RD_RSP_MIDDLE = 3'b100;
    localparam [2:0] RSP_PKT_TYPE_IPV6_RD_RSP_ONLY_OR_LAST = 3'b101;
    localparam [2:0] RSP_PKT_TYPE_IPV6_RD_RSP_FIRST = 3'b110;
    localparam [2:0] RSP_PKT_TYPE_IPV6_RD_RSP_MIDDLE = 3'b111;

    localparam AWCHNL_FIFO_DEPTH = 512;
    localparam AWCHNL_FIFO_DATA_WIDTH = C_SIM_DEBUG ? (C_M_AXI_ADDR_WIDTH + 39) : (C_M_AXI_ADDR_WIDTH + 7);
    localparam AWCHNL_DATA_COUNT_WIDTH = 10;  // log2(AWCHNL_FIFO_DEPTH) + 1

    localparam WCHNL_FIFO_DEPTH = 512;
    //  generate 
    //   begin
    //    if (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN)
    localparam WCHNL_FIFO_DATA_WIDTH = (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN) ? (C_SIM_DEBUG ? (C_RSP_BUF_ADDR_WIDTH + C_M_AXI_ADDR_WIDTH + 80) : (C_RSP_BUF_ADDR_WIDTH + C_M_AXI_ADDR_WIDTH + 48)) : (C_SIM_DEBUG ? (C_RSP_BUF_ADDR_WIDTH + 80) : (C_RSP_BUF_ADDR_WIDTH + 48));
    //    else
    //     localparam WCHNL_FIFO_DATA_WIDTH    = C_SIM_DEBUG ? (C_RSP_BUF_ADDR_WIDTH + 80) : (C_RSP_BUF_ADDR_WIDTH + 48);
    //  end
    //  endgenerate

    localparam WCHNL_DATA_COUNT_WIDTH = 10;  // log2(WCHNL_FIFO_DEPTH) + 1  

    localparam POST_AXI_TNFR_FIFO_DEPTH = 512;
    //  generate 
    //   begin
    //     if (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN)
    localparam POST_AXI_TNFR_FIFO_DATA_WIDTH = NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN ? (C_SIM_DEBUG ? (C_QP_INDX_WIDTH + C_M_AXI_ADDR_WIDTH + 45) : (C_QP_INDX_WIDTH + C_M_AXI_ADDR_WIDTH + 13)) : (C_SIM_DEBUG ? (C_QP_INDX_WIDTH + 45) : (C_QP_INDX_WIDTH + 13));
    //     else
    //       localparam POST_AXI_TNFR_FIFO_DATA_WIDTH  = C_SIM_DEBUG ? (C_QP_INDX_WIDTH+45) : (C_QP_INDX_WIDTH+13);
    //   end
    //  endgenerate 
    localparam POST_AXI_TNFR_DATA_COUNT_WIDTH = 10;  // log2(POST_AXI_TNFR_FIFO_DEPTH) + 1  

    integer                                     i;

    reg     [                             12:0] pyld_len;
    wire    [                              6:0] awlen;
    wire                                        zero_len_pkt;

    wire    [           C_M_AXI_ADDR_WIDTH-1:0] dest_addr;

    reg     [           C_M_AXI_ADDR_WIDTH-1:0] err_buf_ca;
    reg     [                             15:0] err_buf_wrptr;

    wire    [         AWCHNL_FIFO_DATA_WIDTH:0] axi_awchnl_fifo_din;  // One bit for zero length packets
    wire    [         AWCHNL_FIFO_DATA_WIDTH:0] axi_awchnl_fifo_dout;  // One bit for zero length packets
    wire                                        wr_axi_awchnl_fifo;
    wire                                        rd_axi_awchnl_fifo;
    wire                                        axi_awchnl_fifo_full;

    wire    [                             31:0] awchnl_pkt_id;

    wire    [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_din;
    wire    [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_dout;
    wire    [        WCHNL_FIFO_DATA_WIDTH-1:0] axi_wchnl_fifo_raw_dout;
    wire axi_wchnl_fifo_raw_empty;
    wire axi_wchnl_desc_ready;
    wire axi_wchnl_desc_valid;
    wire rd_axi_wchnl_fifo_raw;

    wire                                        wr_axi_wchnl_fifo;
    reg                                         rd_axi_wchnl_fifo;
    wire                                        axi_wchnl_fifo_full;

    wire    [                             31:0] wchnl_pkt_id;
    //  generate 
    //   begin
    //    if (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN )
    //     begin
    wire    [           C_M_AXI_ADDR_WIDTH-1:0] wchnl_dest_addr;
    reg     [           C_M_AXI_ADDR_WIDTH-1:0] wchnl_dest_addr_r;
    //     end
    //   end
    //  endgenerate

    wire    [                              2:0] wchnl_rsp_pkt_type;
    wire    [         C_RSP_BUF_ADDR_WIDTH-1:0] wchnl_rsp_buf_addr_offset;
    wire    [                             12:0] wchnl_pyld_len;
    wire    [                             31:0] wchnl_imm_data;
    reg     [                             31:0] wchnl_imm_data_r;

    reg     [                              6:0] wcnt;
    reg     [                              6:0] nxt_wcnt;

    wire    [                              6:0] wchnl_beats;

    wire                                        post_axi_tnfr_fifo_full_early;
    wire                                        st0_exit_cond;
    wire                                        axi_mstr_idle;
    reg     [         C_RSP_BUF_ADDR_WIDTH-1:0] prev_rsp_pkt_buf_addrb;
    reg                                         rsp_pkt_buf_enb_dummy;
    reg     [                            511:0] prev_rsp_pkt_buf_doutb;
    reg                                         cpy_imm_data;
    reg                                         set_last_cyc;
    reg                                         last_cyc;
    reg                                         set_single_beat_only;
    reg                                         single_beat_only;

    reg                                         m_axi_wvalid_i;
    reg     [                            511:0] m_axi_wdata_i;
    reg     [                             63:0] m_axi_wstrb_i;
    reg                                         m_axi_wlast_i;
    wire                                        m_axi_wready_i;

    reg     [                              1:0] axi_wchnl_cs;
    reg     [                              1:0] axi_wchnl_ns;

    reg     [                             31:0] wchnl_pkt_id_r;
    reg     [                              2:0] wchnl_rsp_pkt_type_r;
    wire    [                              2:0] wchnl_rsp_pkt_type_to_data_mux;

    wire    [POST_AXI_TNFR_FIFO_DATA_WIDTH-1:0] post_axi_tnfr_fifo_din;
    wire    [POST_AXI_TNFR_FIFO_DATA_WIDTH-1:0] post_axi_tnfr_fifo_dout;
    wire                                        wr_post_axi_tnfr_fifo;
    wire                                        rd_post_axi_tnfr_fifo;
    wire                                        post_axi_tnfr_fifo_full;
    wire                                        post_axi_tnfr_fifo_almost_full;

    wire    [                             31:0] pkt_id_post_tnfr;
    wire    [                              9:0] free_run_cnt_at_wlast;
    wire    [                              2:0] rsp_pkt_type_post_tnfr;

    wire                                        incr_rd_rsp_cnt;
    wire                                        incr_rd_req_cnt;
    wire                                        axi_awchnl_zero_len_pkt;

    reg                                         m_axi_bvalid_d;

    always @(*) begin
        pyld_len = 13'd0;
        case (rsp_pkt_type)
            RSP_PKT_TYPE_DISCARD: pyld_len = rsp_pkt_len;
            RSP_PKT_TYPE_ERROR: pyld_len = (i_err_buf_sz < (rsp_pkt_len + 'd4)) ? i_err_buf_sz : (rsp_pkt_len + 'd4);
            RSP_PKT_TYPE_IPV4_RD_RSP_ONLY_OR_LAST, RSP_PKT_TYPE_IPV4_RD_RSP_FIRST: pyld_len = rsp_pkt_len - 'd62;
            RSP_PKT_TYPE_IPV4_RD_RSP_MIDDLE: pyld_len = rsp_pkt_len - 'd58;
            RSP_PKT_TYPE_IPV6_RD_RSP_ONLY_OR_LAST, RSP_PKT_TYPE_IPV6_RD_RSP_FIRST: pyld_len = rsp_pkt_len - 'd82;
            RSP_PKT_TYPE_IPV6_RD_RSP_MIDDLE: pyld_len = rsp_pkt_len - 'd78;
        endcase
    end

    assign awlen        = (pyld_len[5:0] == 6'd0) ? (pyld_len[12:6] - 7'd1) : pyld_len[12:6];
    assign zero_len_pkt = (rsp_pkt_type == RSP_PKT_TYPE_ERROR) && (rsp_pkt_len == 0);

    always @(posedge clk)
        if (reset) begin
            err_buf_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            err_buf_wrptr <= 16'd0;
        end else if (xrnic_en_pulse) begin
            err_buf_ca <= i_err_buf_ba;
        end else if (tnfr_vld && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full && (rsp_pkt_type == RSP_PKT_TYPE_ERROR) && (rsp_pkt_len != 0)) begin
            if (err_buf_wrptr == i_err_num_bufs - 'd1) begin
                err_buf_ca    <= i_err_buf_ba;
                err_buf_wrptr <= 16'd0;
            end else begin
                err_buf_ca    <= err_buf_ca + i_err_buf_sz;
                err_buf_wrptr <= err_buf_wrptr + 'd1;
            end
        end

    assign dest_addr = (rsp_pkt_type == RSP_PKT_TYPE_ERROR) ? err_buf_ca : rsp_dest_addr;

    generate
        if (C_SIM_DEBUG)
            assign axi_awchnl_fifo_din = {
                pkt_id,  // 32
                zero_len_pkt,
                dest_addr,  // C_M_AXI_ADDR_WIDTH
                awlen  // 7
            };
        else
            assign axi_awchnl_fifo_din = {
                zero_len_pkt,
                dest_addr,  // C_M_AXI_ADDR_WIDTH
                awlen  // 7
            };
    endgenerate

    assign wr_axi_awchnl_fifo = (tnfr_vld && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full && (rsp_pkt_type != RSP_PKT_TYPE_DISCARD)) ? 1'b1 : 1'b0;

    //==================================
    // Write Address Channel FIFO
    //==================================  
    xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),                      //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),                    //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (AWCHNL_FIFO_DEPTH),           //positive integer									
        .WRITE_DATA_WIDTH   (AWCHNL_FIFO_DATA_WIDTH + 1),  //positive integer									
        .WR_DATA_COUNT_WIDTH(AWCHNL_DATA_COUNT_WIDTH),     //positive integer									
        .PROG_FULL_THRESH   (10),                          //positive integer									
        .FULL_RESET_VALUE   (0),                           //positive integer; 0 or 1									
        .READ_MODE          ("fwft"),                      //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (0),                           //positive integer;									
        .READ_DATA_WIDTH    (AWCHNL_FIFO_DATA_WIDTH + 1),  //positive integer									
        .RD_DATA_COUNT_WIDTH(AWCHNL_DATA_COUNT_WIDTH),     //positive integer									
        .PROG_EMPTY_THRESH  (10),                          //positive integer									
        .DOUT_RESET_VALUE   ("0"),                         //string									
        .WAKEUP_TIME        (0)                            //positive integer; 0 or 2;	  
    ) axi_awchnl_fifo (
        .sleep        (1'b0),
        .rst          (reset),
        .wr_clk       (clk),
        .wr_en        (wr_axi_awchnl_fifo),
        .din          (axi_awchnl_fifo_din),
        .full         (axi_awchnl_fifo_full),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (rd_axi_awchnl_fifo),
        .dout         (axi_awchnl_fifo_dout),
        .empty        (axi_awchnl_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      ()
    );
    //==================================  

    assign rd_axi_awchnl_fifo = (~axi_awchnl_fifo_empty & m_axi_awready) | (axi_awchnl_zero_len_pkt);

    generate
        if (C_SIM_DEBUG) assign awchnl_pkt_id = axi_awchnl_fifo_dout[(C_M_AXI_ADDR_WIDTH+8)+:32];
        else assign awchnl_pkt_id = 32'd0;
    endgenerate

    assign m_axi_awid              = 1'b0;
    assign m_axi_awaddr            = axi_awchnl_fifo_empty ? {(C_M_AXI_ADDR_WIDTH) {1'b0}} : axi_awchnl_fifo_dout[7+:C_M_AXI_ADDR_WIDTH];
    assign m_axi_awlen             = axi_awchnl_fifo_empty ? 8'b0 : {1'b0, axi_awchnl_fifo_dout[6:0]};
    assign m_axi_awsize            = 3'b110;
    assign m_axi_awburst           = 2'b01;
    assign m_axi_awcache           = 4'b0011;
    assign m_axi_awprot            = 3'b000;
    assign m_axi_awvalid           = ~axi_awchnl_fifo_empty & ~axi_awchnl_zero_len_pkt;
    assign m_axi_awlock            = 1'b0;
    assign axi_awchnl_zero_len_pkt = axi_awchnl_fifo_empty ? 0 : axi_awchnl_fifo_dout[C_M_AXI_ADDR_WIDTH+7];

    generate
        if (C_SIM_DEBUG && (!NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN))
            assign axi_wchnl_fifo_din = {
                pkt_id,  // 32
                rsp_pkt_type,  // 3
                rsp_buf_addr_offset,  // C_RSP_BUF_ADDR_WIDTH
                pyld_len,  // 13
                imm_data  // 32
            };
        else if ((!C_SIM_DEBUG) && (!NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN))
            assign axi_wchnl_fifo_din = {
                rsp_pkt_type,  // 3
                rsp_buf_addr_offset,  // C_RSP_BUF_ADDR_WIDTH
                pyld_len,  // 13
                imm_data  // 32
            };
        else if (C_SIM_DEBUG && (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN))
            assign axi_wchnl_fifo_din = {
                pkt_id,  // 32
                dest_addr,  // C_M_AXI_ADDR_WIDTH
                rsp_pkt_type,  // 3
                rsp_buf_addr_offset,  // C_RSP_BUF_ADDR_WIDTH
                pyld_len,  // 13
                imm_data  // 32
            };
        else if ((!C_SIM_DEBUG) && (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN))
            assign axi_wchnl_fifo_din = {
                dest_addr,  // C_M_AXI_ADDR_WIDTH
                rsp_pkt_type,  // 3
                rsp_buf_addr_offset,  // C_RSP_BUF_ADDR_WIDTH
                pyld_len,  // 13
                imm_data  // 32
            };
    endgenerate

    assign wr_axi_wchnl_fifo = (tnfr_vld && !axi_awchnl_fifo_full && !axi_wchnl_fifo_full) ? 1'b1 : 1'b0;

    //=================================
    // Write Channel FIFO
    //=================================
    xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),                  //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),                //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (WCHNL_FIFO_DEPTH),        //positive integer									
        .WRITE_DATA_WIDTH   (WCHNL_FIFO_DATA_WIDTH),   //positive integer									
        .WR_DATA_COUNT_WIDTH(WCHNL_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_FULL_THRESH   (10),                      //positive integer									
        .FULL_RESET_VALUE   (0),                       //positive integer; 0 or 1									
        .READ_MODE          ("fwft"),                  //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (0),                       //positive integer;									
        .READ_DATA_WIDTH    (WCHNL_FIFO_DATA_WIDTH),   //positive integer									
        .RD_DATA_COUNT_WIDTH(WCHNL_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_EMPTY_THRESH  (10),                      //positive integer									
        .DOUT_RESET_VALUE   ("0"),                     //string									
        .WAKEUP_TIME        (0)                        //positive integer; 0 or 2;	  
    ) axi_wchnl_fifo (
        .sleep        (1'b0),
        .rst          (reset),
        .wr_clk       (clk),
        .wr_en        (wr_axi_wchnl_fifo),
        .din          (axi_wchnl_fifo_din),
        .full         (axi_wchnl_fifo_full),
        .prog_full    (),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (rd_axi_wchnl_fifo_raw),
        .dout         (axi_wchnl_fifo_raw_dout),
        .empty        (axi_wchnl_fifo_raw_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      ()
    );
    //=================================  

    // Register the narrow descriptor before its type/length controls the
    // 512-bit payload mux.  Both payload and VALID are elastic: retain the
    // descriptor until the original W FSM consumes it, including W backpressure.
    // This adds startup latency, not a bubble between payload beats.
    assign rd_axi_wchnl_fifo_raw = !reset && !axi_wchnl_fifo_raw_empty && axi_wchnl_desc_ready;
    reg_slice #(.C_DATA_WIDTH(WCHNL_FIFO_DATA_WIDTH)) wchnl_desc_slice (
        .aclk(clk), .aclken(1'b1), .areset(reset),
        .S_PAYLOAD_DATA(axi_wchnl_fifo_raw_dout),
        .S_VALID(!reset && !axi_wchnl_fifo_raw_empty),
        .S_READY(axi_wchnl_desc_ready),
        .M_PAYLOAD_DATA(axi_wchnl_fifo_dout),
        .M_VALID(axi_wchnl_desc_valid),
        .M_READY(rd_axi_wchnl_fifo)
    );
    // The public empty/idle indication includes descriptors still in the FIFO
    // while the slice is filling.  The W FSM may start only on slice VALID.
    assign axi_wchnl_fifo_empty = axi_wchnl_fifo_raw_empty && !axi_wchnl_desc_valid;

    assign rdy_to_tnfr = (!axi_awchnl_fifo_full && !axi_wchnl_fifo_full) ? 1'b1 : 1'b0;

    generate
        if (C_SIM_DEBUG && (!NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN)) assign wchnl_pkt_id = axi_wchnl_fifo_dout[(C_RSP_BUF_ADDR_WIDTH+48)+:32];
        else if (C_SIM_DEBUG && (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN)) assign wchnl_pkt_id = axi_wchnl_fifo_dout[(C_RSP_BUF_ADDR_WIDTH+C_M_AXI_ADDR_WIDTH+48)+:32];
        else assign wchnl_pkt_id = 32'd0;
    endgenerate

    generate
        if (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN) assign wchnl_dest_addr = axi_wchnl_fifo_dout[(C_RSP_BUF_ADDR_WIDTH+45+3)+:C_M_AXI_ADDR_WIDTH];
    endgenerate


    assign wchnl_rsp_pkt_type        = axi_wchnl_fifo_dout[(C_RSP_BUF_ADDR_WIDTH+45)+:3];
    assign wchnl_rsp_buf_addr_offset = axi_wchnl_fifo_dout[45+:C_RSP_BUF_ADDR_WIDTH];
    assign wchnl_pyld_len            = axi_wchnl_fifo_dout[32+:13];
    assign wchnl_imm_data            = axi_wchnl_fifo_dout[0+:32];

    assign st0_exit_cond             = axi_wchnl_desc_valid & ~post_axi_tnfr_fifo_full & (!last_cyc | m_axi_wready_i);
    assign axi_mstr_idle             = axi_wchnl_fifo_empty & ~m_axi_wvalid_i;

    assign wchnl_beats               = |wchnl_pyld_len[5:0] ? wchnl_pyld_len[12:6] : (wchnl_pyld_len[12:6] - 'd1);

    always @(*) begin
        // default values
        nxt_wcnt              = wcnt;
        rsp_pkt_buf_addrb     = prev_rsp_pkt_buf_addrb;
        rsp_pkt_buf_enb       = 1'b0;
        rsp_pkt_buf_enb_dummy = 1'b0;
        rd_axi_wchnl_fifo     = 1'b0;
        cpy_imm_data          = 1'b0;
        set_last_cyc          = 1'b0;
        set_single_beat_only  = 1'b0;
        m_axi_wvalid_i        = 1'b0;
        m_axi_wlast_i         = 1'b0;
        m_axi_wstrb_i         = {64{1'b0}};
        axi_wchnl_ns          = axi_wchnl_cs;

        case (axi_wchnl_cs)
            2'd0: begin
                if (last_cyc) begin
                    m_axi_wvalid_i = 1'b1;
                    m_axi_wstrb_i  = {64{1'b1}};
                    m_axi_wlast_i  = 1'b1;
                end

                if (st0_exit_cond) begin
                    case (wchnl_rsp_pkt_type)
                        RSP_PKT_TYPE_DISCARD: begin
                            rd_axi_wchnl_fifo = 1'b1;
                        end  // RSP_PKT_TYPE_DISCARD

                        RSP_PKT_TYPE_ERROR: begin
                            rsp_pkt_buf_addrb = wchnl_rsp_buf_addr_offset;
                            rsp_pkt_buf_enb   = 1'b1;
                            cpy_imm_data      = 1'b1;

                            if (wchnl_pyld_len == 13'h4) begin
                                nxt_wcnt          = 7'd0;
                                set_last_cyc      = 1'b0;
                                rd_axi_wchnl_fifo = 1'b1;
                            end else if (wchnl_pyld_len <= 13'd64) begin
                                nxt_wcnt          = 7'd0;
                                set_last_cyc      = 1'b1;
                                rd_axi_wchnl_fifo = 1'b1;
                            end else begin
                                nxt_wcnt     = wchnl_beats;
                                axi_wchnl_ns = 2'd2;
                            end
                        end  // RSP_PKT_TYPE_ERROR

                        RSP_PKT_TYPE_IPV4_RD_RSP_ONLY_OR_LAST, RSP_PKT_TYPE_IPV4_RD_RSP_FIRST: begin
                            rsp_pkt_buf_addrb = wchnl_rsp_buf_addr_offset;
                            rsp_pkt_buf_enb   = 1'b1;

                            if (wchnl_pyld_len <= 13'd6) begin
                                nxt_wcnt             = 7'd0;
                                set_last_cyc         = 1'b1;
                                set_single_beat_only = 1'b1;
                                rd_axi_wchnl_fifo    = 1'b1;
                            end else begin
                                nxt_wcnt     = wchnl_beats;
                                axi_wchnl_ns = 2'd1;
                            end
                        end  // RSP_PKT_TYPE_IPV4_RD_RSP_ONLY_OR_LAST

                        RSP_PKT_TYPE_IPV4_RD_RSP_MIDDLE: begin
                            rsp_pkt_buf_addrb = wchnl_rsp_buf_addr_offset;
                            rsp_pkt_buf_enb   = 1'b1;

                            if (wchnl_pyld_len <= 13'd10) begin
                                nxt_wcnt             = 7'd0;
                                set_last_cyc         = 1'b1;
                                set_single_beat_only = 1'b1;
                                rd_axi_wchnl_fifo    = 1'b1;
                            end else begin
                                nxt_wcnt     = wchnl_beats;
                                axi_wchnl_ns = 2'd1;
                            end
                        end  // RSP_PKT_TYPE_IPV4_RD_RSP_MIDDLE

                        RSP_PKT_TYPE_IPV6_RD_RSP_ONLY_OR_LAST, RSP_PKT_TYPE_IPV6_RD_RSP_FIRST: begin
                            rsp_pkt_buf_addrb = wchnl_rsp_buf_addr_offset + 'd1;
                            rsp_pkt_buf_enb   = 1'b1;

                            if (wchnl_pyld_len <= 13'd50) begin
                                nxt_wcnt             = 7'd0;
                                set_last_cyc         = 1'b1;
                                set_single_beat_only = 1'b1;
                                rd_axi_wchnl_fifo    = 1'b1;
                            end else begin
                                nxt_wcnt     = wchnl_beats;
                                axi_wchnl_ns = 2'd1;
                            end
                        end  // RSP_PKT_TYPE_IPV6_RD_RSP_ONLY_OR_LAST

                        RSP_PKT_TYPE_IPV6_RD_RSP_MIDDLE: begin
                            rsp_pkt_buf_addrb = wchnl_rsp_buf_addr_offset + 'd1;
                            rsp_pkt_buf_enb   = 1'b1;

                            if (wchnl_pyld_len <= 13'd54) begin
                                nxt_wcnt             = 7'd0;
                                set_last_cyc         = 1'b1;
                                set_single_beat_only = 1'b1;
                                rd_axi_wchnl_fifo    = 1'b1;
                            end else begin
                                nxt_wcnt     = wchnl_beats;
                                axi_wchnl_ns = 2'd1;
                            end
                        end  // RSP_PKT_TYPE_IPV6_RD_RSP_MIDDLE                  

                    endcase  // case (wchnl_rsp_pkt_type)         
                end  // if (st0_exit_cond)
            end  // 2'd0

            2'd1: begin
                rsp_pkt_buf_addrb = prev_rsp_pkt_buf_addrb + 'd1;
                rsp_pkt_buf_enb   = 1'b1;

                if (wcnt == 7'd0) begin
                    set_last_cyc      = 1'b1;
                    rd_axi_wchnl_fifo = 1'b1;
                    axi_wchnl_ns      = 2'd0;
                end else begin
                    axi_wchnl_ns = 2'd2;
                end
            end  //2'd1

            2'd2: begin
                m_axi_wvalid_i = 1'b1;
                m_axi_wstrb_i  = {64{1'b1}};
                if (m_axi_wready_i) begin
                    nxt_wcnt          = wcnt - 'd1;
                    rsp_pkt_buf_addrb = prev_rsp_pkt_buf_addrb + 'd1;
                    rsp_pkt_buf_enb   = 1'b1;

                    if (wcnt == 7'd1) begin
                        set_last_cyc      = 1'b1;
                        rd_axi_wchnl_fifo = 1'b1;
                        axi_wchnl_ns      = 2'd0;
                    end
                end  // if (m_axi_wready_i)       
            end  // 2'd2

            default: begin
                axi_wchnl_ns = 2'dx;
            end  // default     
        endcase  // case (axi_wchnl_cs)
    end

    always @(posedge clk)
        if (reset) axi_wchnl_cs <= 2'b00;
        else axi_wchnl_cs <= axi_wchnl_ns;

    always @(posedge clk)
        if (reset) last_cyc <= 1'b0;
        else if (set_last_cyc || m_axi_wready_i) last_cyc <= set_last_cyc;

    always @(posedge clk)
        if (reset) single_beat_only <= 1'b0;
        else if (set_single_beat_only || m_axi_wready_i) single_beat_only <= set_single_beat_only;

    always @(posedge clk) wcnt <= nxt_wcnt;

    always @(posedge clk)
        if (rsp_pkt_buf_enb) prev_rsp_pkt_buf_addrb <= rsp_pkt_buf_addrb;
        else if (axi_mstr_idle) prev_rsp_pkt_buf_addrb <= rsp_pkt_buf_addra;

    always @(posedge clk)
        if (cpy_imm_data) prev_rsp_pkt_buf_doutb <= {wchnl_imm_data, rsp_pkt_buf_doutb[479:0]};  // check once again
        else if (rsp_pkt_buf_enb) prev_rsp_pkt_buf_doutb <= rsp_pkt_buf_doutb;

    assign wchnl_rsp_pkt_type_to_data_mux = last_cyc ? wchnl_rsp_pkt_type_r : wchnl_rsp_pkt_type;

    // INVALID payload is don't-care. Do not gate the wide packet-data mux
    // with VALID: doing so adds a LUT level on BRAM-to-output data paths.
    // The existing elastic slice accepts only VALID beats and holds stalls.
    always @(*) begin
        m_axi_wdata_i = 512'd0;
        case (wchnl_rsp_pkt_type_to_data_mux)
            RSP_PKT_TYPE_ERROR: m_axi_wdata_i = {rsp_pkt_buf_doutb[60*8-1:0], prev_rsp_pkt_buf_doutb[64*8-1:60*8]};
            RSP_PKT_TYPE_IPV4_RD_RSP_ONLY_OR_LAST:
            if (single_beat_only) m_axi_wdata_i[6*8-1:0] = rsp_pkt_buf_doutb[64*8-1:58*8];
            else m_axi_wdata_i = {rsp_pkt_buf_doutb[58*8-1:0], prev_rsp_pkt_buf_doutb[64*8-1:58*8]};
            RSP_PKT_TYPE_IPV4_RD_RSP_FIRST:
            if (single_beat_only) m_axi_wdata_i[6*8-1:0] = rsp_pkt_buf_doutb[64*8-1:58*8];
            else m_axi_wdata_i = {rsp_pkt_buf_doutb[58*8-1:0], prev_rsp_pkt_buf_doutb[64*8-1:58*8]};
            RSP_PKT_TYPE_IPV4_RD_RSP_MIDDLE:
            if (single_beat_only) m_axi_wdata_i[10*8-1:0] = rsp_pkt_buf_doutb[64*8-1:54*8];
            else m_axi_wdata_i = {rsp_pkt_buf_doutb[54*8-1:0], prev_rsp_pkt_buf_doutb[64*8-1:54*8]};
            RSP_PKT_TYPE_IPV6_RD_RSP_ONLY_OR_LAST:
            if (single_beat_only) m_axi_wdata_i[50*8-1:0] = rsp_pkt_buf_doutb[64*8-1:14*8];
            else m_axi_wdata_i = {rsp_pkt_buf_doutb[14*8-1:0], prev_rsp_pkt_buf_doutb[64*8-1:14*8]};
            RSP_PKT_TYPE_IPV6_RD_RSP_FIRST:
            if (single_beat_only) m_axi_wdata_i[50*8-1:0] = rsp_pkt_buf_doutb[64*8-1:14*8];
            else m_axi_wdata_i = {rsp_pkt_buf_doutb[14*8-1:0], prev_rsp_pkt_buf_doutb[64*8-1:14*8]};
            RSP_PKT_TYPE_IPV6_RD_RSP_MIDDLE:
            if (single_beat_only) m_axi_wdata_i[54*8-1:0] = rsp_pkt_buf_doutb[64*8-1:10*8];
            else m_axi_wdata_i = {rsp_pkt_buf_doutb[10*8-1:0], prev_rsp_pkt_buf_doutb[64*8-1:10*8]};
            default: m_axi_wdata_i = 512'd0;
        endcase
    end

    reg_slice #(
        .C_DATA_WIDTH(577)
    ) reg_slice_inst (
        .aclk          (clk),
        .aclken        (1'b1),
        .areset        (reset),
        .S_PAYLOAD_DATA({m_axi_wdata_i, m_axi_wstrb_i, m_axi_wlast_i}),
        .S_VALID       (m_axi_wvalid_i),
        .S_READY       (m_axi_wready_i),
        .M_PAYLOAD_DATA({m_axi_wdata, m_axi_wstrb, m_axi_wlast}),
        .M_VALID       (m_axi_wvalid),
        .M_READY       (m_axi_wready)
    );

    assign m_axi_bready  = 1'b1;
    assign m_axi_arid    = 1'b0;
    assign m_axi_araddr  = {(C_M_AXI_ADDR_WIDTH) {1'b0}};
    assign m_axi_arlen   = 8'h00;
    assign m_axi_arsize  = 3'b110;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arvalid = 1'b0;
    assign m_axi_rready  = 1'b1;
    assign m_axi_arlock  = 1'b0;

    generate
        begin
            if (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN) begin
                always @(posedge clk)
                    if (rd_axi_wchnl_fifo) begin
                        wchnl_pkt_id_r       <= wchnl_pkt_id;
                        wchnl_rsp_pkt_type_r <= wchnl_rsp_pkt_type;
                        wchnl_imm_data_r     <= wchnl_imm_data;
                        wchnl_dest_addr_r    <= wchnl_dest_addr;
                    end
            end else begin
                always @(posedge clk)
                    if (rd_axi_wchnl_fifo) begin
                        wchnl_pkt_id_r       <= wchnl_pkt_id;
                        wchnl_rsp_pkt_type_r <= wchnl_rsp_pkt_type;
                        wchnl_imm_data_r     <= wchnl_imm_data;
                    end
            end
        end
    endgenerate

    generate
        if (C_SIM_DEBUG && (!NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN))
            assign post_axi_tnfr_fifo_din = {
                wchnl_pkt_id_r,  // 32
                free_run_cnt[9:0],  // 10
                wchnl_rsp_pkt_type_r,  // 3
                wchnl_imm_data_r[16+:C_QP_INDX_WIDTH]  // C_QP_INDX_WIDTH
            };
        else if ((!C_SIM_DEBUG) && (!NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN))
            assign post_axi_tnfr_fifo_din = {
                free_run_cnt[9:0],  // 10
                wchnl_rsp_pkt_type_r,  // 3   
                wchnl_imm_data_r[16+:C_QP_INDX_WIDTH]  // C_QP_INDX_WIDTH
            };
        else if (C_SIM_DEBUG && (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN))
            assign post_axi_tnfr_fifo_din = {
                wchnl_pkt_id_r,  // 32
                wchnl_dest_addr_r,  // C_M_AXI_ADDR_WIDTH
                free_run_cnt[9:0],  // 10
                wchnl_rsp_pkt_type_r,  // 3
                wchnl_imm_data_r[16+:C_QP_INDX_WIDTH]  // C_QP_INDX_WIDTH
            };
        else if ((!C_SIM_DEBUG) && (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN))
            assign post_axi_tnfr_fifo_din = {
                wchnl_dest_addr_r,  // C_M_AXI_ADDR_WIDTH
                free_run_cnt[9:0],  // 10
                wchnl_rsp_pkt_type_r,  // 3   
                wchnl_imm_data_r[16+:C_QP_INDX_WIDTH]  // C_QP_INDX_WIDTH
            };

    endgenerate

    assign wr_post_axi_tnfr_fifo = m_axi_wlast_i & m_axi_wready_i;

    // =====================================================================
    // POST_AXI_TNFR_FIFO (FWFT)
    // ===================================================================== 
    /*
  reg_fifo_sync #(
    .C_READ_MODE (1'b1),
    .C_FIFO_DEPTH (POST_AXI_TNFR_FIFO_DEPTH),
    .C_FIFO_WRCNTR_WIDTH (POST_AXI_TNFR_DATA_COUNT_WIDTH),
    .C_FIFO_DATA_WIDTH(POST_AXI_TNFR_FIFO_DATA_WIDTH)
  ) post_axi_tnfr_fifo (
    .clk          (clk),
    .reset        (reset),
    .wr_en        (wr_post_axi_tnfr_fifo),
    .din          (post_axi_tnfr_fifo_din),  
    .full         (post_axi_tnfr_fifo_full),  
    .almost_full  (post_axi_tnfr_fifo_almost_full), 
    .rd_en        (rd_post_axi_tnfr_fifo),
    .dout         (post_axi_tnfr_fifo_dout),
    .empty        (post_axi_tnfr_fifo_empty)
  );
  */

    reg  [POST_AXI_TNFR_DATA_COUNT_WIDTH-1:0] post_axi_tnfr_fifo_wrptr;
    reg  [POST_AXI_TNFR_DATA_COUNT_WIDTH-1:0] post_axi_tnfr_fifo_rdptr;
    wire [POST_AXI_TNFR_DATA_COUNT_WIDTH-1:0] nxt_post_axi_tnfr_fifo_wrptr;

    xpm_fifo_sync_wrap #(
        .FIFO_MEMORY_TYPE   ("auto"),                          //string; "auto", "block", "distributed", or "ultra";									
        .ECC_MODE           ("no_ecc"),                        //string; "no_ecc" or "en_ecc";									
        .FIFO_WRITE_DEPTH   (POST_AXI_TNFR_FIFO_DEPTH),        //positive integer									
        .WRITE_DATA_WIDTH   (POST_AXI_TNFR_FIFO_DATA_WIDTH),   //positive integer									
        .WR_DATA_COUNT_WIDTH(POST_AXI_TNFR_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_FULL_THRESH   (POST_AXI_TNFR_FIFO_DEPTH - 8),    //positive integer									
        .FULL_RESET_VALUE   (0),                               //positive integer; 0 or 1									
        .READ_MODE          ("fwft"),                          //string; "std" or "fwft";									
        .FIFO_READ_LATENCY  (0),                               //positive integer;									
        .READ_DATA_WIDTH    (POST_AXI_TNFR_FIFO_DATA_WIDTH),   //positive integer									
        .RD_DATA_COUNT_WIDTH(POST_AXI_TNFR_DATA_COUNT_WIDTH),  //positive integer									
        .PROG_EMPTY_THRESH  (10),                              //positive integer									
        .DOUT_RESET_VALUE   ("0"),                             //string									
        .WAKEUP_TIME        (0)                                //positive integer; 0 or 2;	  
    ) post_axi_tnfr_fifo (
        .sleep        (1'b0),
        .rst          (reset),
        .wr_clk       (clk),
        .wr_en        (wr_post_axi_tnfr_fifo),
        .din          (post_axi_tnfr_fifo_din),
        .full         (),
        .prog_full    (post_axi_tnfr_fifo_full),
        .wr_data_count(),
        .overflow     (),
        .wr_rst_busy  (),
        .rd_en        (rd_post_axi_tnfr_fifo),
        .dout         (post_axi_tnfr_fifo_dout),
        .empty        (post_axi_tnfr_fifo_empty),
        .prog_empty   (),
        .rd_data_count(),
        .underflow    (),
        .rd_rst_busy  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      ()
    );

    // =====================================================================

    generate
        if (C_SIM_DEBUG && (!NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN)) assign pkt_id_post_tnfr = post_axi_tnfr_fifo_dout[(13+C_QP_INDX_WIDTH)+:32];
        else if ((!C_SIM_DEBUG) && (!NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN)) assign pkt_id_post_tnfr = 32'd0;
        else if (C_SIM_DEBUG && (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN)) assign pkt_id_post_tnfr = post_axi_tnfr_fifo_dout[(13+C_QP_INDX_WIDTH+C_M_AXI_ADDR_WIDTH)+:32];
        else if ((!C_SIM_DEBUG) && (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN)) assign pkt_id_post_tnfr = 32'd0;
    endgenerate

    assign free_run_cnt_at_wlast  = post_axi_tnfr_fifo_dout[(3+C_QP_INDX_WIDTH)+:10];
    assign rsp_pkt_type_post_tnfr = post_axi_tnfr_fifo_dout[C_QP_INDX_WIDTH+:3];
    assign o_rsp_qp_id            = post_axi_tnfr_fifo_dout[0+:C_QP_INDX_WIDTH];
    generate
        if (NVMOF_SYSTEM_ONLY_FEATURE_CQ_DB_CNT_EN) assign o_rsp_dest_addr_for_hw_hndshk = post_axi_tnfr_fifo_dout[(13+C_QP_INDX_WIDTH)+:C_M_AXI_ADDR_WIDTH];
        else assign o_rsp_dest_addr_for_hw_hndshk = 'b0;

    endgenerate

    always @(posedge clk)
        if (reset) begin
            m_axi_bvalid_d <= 1'b0;
        end else begin
            m_axi_bvalid_d <= m_axi_bvalid && m_axi_bready;
        end


    assign rd_post_axi_tnfr_fifo  = ~post_axi_tnfr_fifo_empty & m_axi_bvalid_d;

    assign incr_rd_rsp_cnt        = (rd_post_axi_tnfr_fifo && (rsp_pkt_type_post_tnfr >= 3'b010)) ? 1'b1 : 1'b0;
    assign incr_rd_req_cnt        = (rd_post_axi_tnfr_fifo && ((rsp_pkt_type_post_tnfr == RSP_PKT_TYPE_IPV4_RD_RSP_ONLY_OR_LAST) || (rsp_pkt_type_post_tnfr == RSP_PKT_TYPE_IPV6_RD_RSP_ONLY_OR_LAST))) ? 1'b1 : 1'b0;
    assign incr_inv_pkt_cnt       = (rd_post_axi_tnfr_fifo && (rsp_pkt_type_post_tnfr == RSP_PKT_TYPE_ERROR)) ? 1'b1 : 1'b0;
    assign set_pkt_valdn_err_intr = incr_inv_pkt_cnt;
    assign o_rd_rsp_wr_cmpltd     = incr_rd_req_cnt;

    always @(posedge clk)
        if (reset) begin
            o_err_buf_ca    <= {(C_M_AXI_ADDR_WIDTH) {1'b0}};
            o_err_buf_wrptr <= 16'd0;
        end else if (xrnic_en_pulse) begin
            o_err_buf_ca <= i_err_buf_ba;
        end else if (incr_inv_pkt_cnt) begin
            if (o_err_buf_wrptr == i_err_num_bufs - 'd1) begin
                o_err_buf_ca    <= i_err_buf_ba;
                o_err_buf_wrptr <= 16'd0;
            end else begin
                o_err_buf_ca    <= o_err_buf_ca + i_err_buf_sz;
                o_err_buf_wrptr <= o_err_buf_wrptr + 'd1;
            end
        end

    always @(posedge clk)
        if (reset) o_rd_req_pkt_cnt <= 16'd0;
        else if (incr_rd_req_cnt) o_rd_req_pkt_cnt <= o_rd_req_pkt_cnt + 'd1;

    always @(posedge clk)
        if (reset) o_rd_rsp_pkt_cnt <= 16'd0;
        else if (incr_rd_rsp_cnt) o_rd_rsp_pkt_cnt <= o_rd_rsp_pkt_cnt + 'd1;

    // =====================================================================
    // Debug Logic
    // =====================================================================
    reg  [14:0] axi_awchnl_wr_cnt;
    reg  [14:0] axi_awchnl_rd_cnt;
    wire [15:0] axi_awchnl_fifo_occupied;
    wire [15:0] axi_awchnl_fifo_unoccupied;

    reg  [14:0] axi_wchnl_wr_cnt;
    reg  [14:0] axi_wchnl_rd_cnt;
    wire [15:0] axi_wchnl_fifo_occupied;
    wire [15:0] axi_wchnl_fifo_unoccupied;

    reg  [14:0] post_axi_tnfr_wr_cnt;
    reg  [14:0] post_axi_tnfr_rd_cnt;
    wire [15:0] post_axi_tnfr_fifo_occupied;
    wire [15:0] post_axi_tnfr_fifo_unoccupied;

    reg  [15:0] axi_awchnl_wait_st_cnt;
    reg  [15:0] axi_wchnl_wait_st_cnt;
    reg  [15:0] axi_bresp_wait_st_cnt;

    always @(posedge clk)
        if (reset) axi_awchnl_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) axi_awchnl_wr_cnt <= 15'd0;
        else if (wr_axi_awchnl_fifo) axi_awchnl_wr_cnt <= axi_awchnl_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) axi_awchnl_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) axi_awchnl_rd_cnt <= 15'd0;
        else if (rd_axi_awchnl_fifo) axi_awchnl_rd_cnt <= axi_awchnl_rd_cnt + 'd1;

    assign axi_awchnl_fifo_occupied   = (axi_awchnl_rd_cnt > axi_awchnl_wr_cnt) ? ({1'b1, axi_awchnl_wr_cnt} - axi_awchnl_rd_cnt) : (axi_awchnl_wr_cnt - axi_awchnl_rd_cnt);

    assign axi_awchnl_fifo_unoccupied = AWCHNL_FIFO_DEPTH - axi_awchnl_fifo_occupied;

    always @(posedge clk)
        if (reset) axi_awchnl_fifo_unoccupied_r <= AWCHNL_FIFO_DEPTH;
        else if (i_clr_debug_sts) axi_awchnl_fifo_unoccupied_r <= AWCHNL_FIFO_DEPTH;
        else if (axi_awchnl_fifo_unoccupied_r > axi_awchnl_fifo_unoccupied) axi_awchnl_fifo_unoccupied_r <= axi_awchnl_fifo_unoccupied;

    always @(posedge clk)
        if (reset) axi_awchnl_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) axi_awchnl_fifo_full_sticky <= 1'b0;
        else if (axi_awchnl_fifo_full) axi_awchnl_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) axi_wchnl_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) axi_wchnl_wr_cnt <= 15'd0;
        else if (wr_axi_wchnl_fifo) axi_wchnl_wr_cnt <= axi_wchnl_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) axi_wchnl_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) axi_wchnl_rd_cnt <= 15'd0;
        else if (rd_axi_wchnl_fifo_raw) axi_wchnl_rd_cnt <= axi_wchnl_rd_cnt + 'd1;

    assign axi_wchnl_fifo_occupied   = (axi_wchnl_rd_cnt > axi_wchnl_wr_cnt) ? ({1'b1, axi_wchnl_wr_cnt} - axi_wchnl_rd_cnt) : (axi_wchnl_wr_cnt - axi_wchnl_rd_cnt);

    assign axi_wchnl_fifo_unoccupied = WCHNL_FIFO_DEPTH - axi_wchnl_fifo_occupied;

    always @(posedge clk)
        if (reset) axi_wchnl_fifo_unoccupied_r <= WCHNL_FIFO_DEPTH;
        else if (i_clr_debug_sts) axi_wchnl_fifo_unoccupied_r <= WCHNL_FIFO_DEPTH;
        else if (axi_wchnl_fifo_unoccupied_r > axi_wchnl_fifo_unoccupied) axi_wchnl_fifo_unoccupied_r <= axi_wchnl_fifo_unoccupied;

    always @(posedge clk)
        if (reset) axi_wchnl_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) axi_wchnl_fifo_full_sticky <= 1'b0;
        else if (axi_wchnl_fifo_full) axi_wchnl_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) begin
            axi_awchnl_wait_st_cnt <= 16'd0;
            axi_awchnl_max_wait_st <= 16'd0;
        end else if (i_clr_debug_sts) begin
            axi_awchnl_wait_st_cnt <= 16'd0;
            axi_awchnl_max_wait_st <= 16'd0;
        end else if (m_axi_awvalid) begin
            if (m_axi_awready) begin
                axi_awchnl_wait_st_cnt <= 16'd0;
                if (axi_awchnl_max_wait_st < axi_awchnl_wait_st_cnt) axi_awchnl_max_wait_st <= axi_awchnl_wait_st_cnt;
            end else axi_awchnl_wait_st_cnt <= axi_awchnl_wait_st_cnt + 'd1;
        end

    always @(posedge clk)
        if (reset) begin
            axi_wchnl_wait_st_cnt <= 16'd0;
            axi_wchnl_max_wait_st <= 16'd0;
        end else if (i_clr_debug_sts) begin
            axi_wchnl_wait_st_cnt <= 16'd0;
            axi_wchnl_max_wait_st <= 16'd0;
        end else if (m_axi_wvalid) begin
            if (m_axi_wready) begin
                axi_wchnl_wait_st_cnt <= 16'd0;
                if (axi_wchnl_max_wait_st < axi_wchnl_wait_st_cnt) axi_wchnl_max_wait_st <= axi_wchnl_wait_st_cnt;
            end else axi_wchnl_wait_st_cnt <= axi_wchnl_wait_st_cnt + 'd1;
        end

    always @(posedge clk)
        if (reset) post_axi_tnfr_wr_cnt <= 15'd0;
        else if (i_clr_debug_sts) post_axi_tnfr_wr_cnt <= 15'd0;
        else if (wr_post_axi_tnfr_fifo) post_axi_tnfr_wr_cnt <= post_axi_tnfr_wr_cnt + 'd1;

    always @(posedge clk)
        if (reset) post_axi_tnfr_rd_cnt <= 15'd0;
        else if (i_clr_debug_sts) post_axi_tnfr_rd_cnt <= 15'd0;
        else if (rd_post_axi_tnfr_fifo) post_axi_tnfr_rd_cnt <= post_axi_tnfr_rd_cnt + 'd1;

    assign post_axi_tnfr_fifo_occupied   = (post_axi_tnfr_rd_cnt > post_axi_tnfr_wr_cnt) ? ({1'b1, post_axi_tnfr_wr_cnt} - post_axi_tnfr_rd_cnt) : (post_axi_tnfr_wr_cnt - post_axi_tnfr_rd_cnt);

    assign post_axi_tnfr_fifo_unoccupied = POST_AXI_TNFR_FIFO_DEPTH - post_axi_tnfr_fifo_occupied;

    always @(posedge clk)
        if (reset) post_axi_tnfr_fifo_unoccupied_r <= POST_AXI_TNFR_FIFO_DEPTH;
        else if (i_clr_debug_sts) post_axi_tnfr_fifo_unoccupied_r <= POST_AXI_TNFR_FIFO_DEPTH;
        else if (post_axi_tnfr_fifo_unoccupied_r > post_axi_tnfr_fifo_unoccupied) post_axi_tnfr_fifo_unoccupied_r <= post_axi_tnfr_fifo_unoccupied;

    always @(posedge clk)
        if (reset) post_axi_tnfr_fifo_full_sticky <= 1'b0;
        else if (i_clr_debug_sts) post_axi_tnfr_fifo_full_sticky <= 1'b0;
        else if (post_axi_tnfr_fifo_full) post_axi_tnfr_fifo_full_sticky <= 1'b1;

    always @(posedge clk)
        if (reset) axi_bresp_max_wait_st <= 16'd0;
        else if (i_clr_debug_sts) axi_bresp_max_wait_st <= 16'd0;
        else if (rd_post_axi_tnfr_fifo)
            if (free_run_cnt[9:0] > free_run_cnt_at_wlast) begin
                if (axi_bresp_max_wait_st < (free_run_cnt[9:0] - free_run_cnt_at_wlast)) axi_bresp_max_wait_st <= free_run_cnt[9:0] - free_run_cnt_at_wlast;
            end else begin
                if (axi_bresp_max_wait_st < ({1'b1, free_run_cnt[9:0]} - free_run_cnt_at_wlast)) axi_bresp_max_wait_st <= {1'b1, free_run_cnt[9:0]} - free_run_cnt_at_wlast;
            end

endmodule
