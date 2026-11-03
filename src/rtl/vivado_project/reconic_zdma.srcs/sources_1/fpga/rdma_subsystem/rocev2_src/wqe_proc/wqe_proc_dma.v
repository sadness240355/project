// /////////////////////////////////////////////////////////////////
// (c) Copyright 2013 - 2014 Xilinx, Inc. All rights reserved.	 
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
// wqe_proc_dma.v
//-----------------------------------------------------------------------------
//
// Filename        : wqe_proc_dma.v
// Version         : v1.0
// Description     : WQE处理器DMA模块包含从 SGL BRAM 中获取单个格式的逻辑，并根据格式类型读取数据或头部信息。
//                   数据通过AXI 4接口获取，头部信息则从BRAM中读取。
//                   获取的数据/头部信息通过AXI流接口传输。 
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
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module wqe_proc_dma #(
    parameter C_M_AXI_DATA_WIDTH = 512,
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_M_AXI_ID_WIDTH = 4,
    parameter C_AXIS_DATA_WIDTH = 512,
    parameter C_MAX_SGL_DEPTH = 128,
    parameter C_SGL_DATA_WIDTH = 82,
    parameter C_HDR_BUF_ADDR_WIDTH = 32,
    parameter C_HDR_BUF_DEPTH = 128,
    parameter C_VIVADO_VER = 1
) (
    input core_clk,
    input core_rst,

    // AXI-4 Master signals
    output     [ C_M_AXI_ID_WIDTH-1 : 0] m_axi_dma_arid,
    output reg [ C_M_AXI_ADDR_WIDTH-1:0] m_axi_dma_araddr,
    output reg [                    7:0] m_axi_dma_arlen,
    output     [                    2:0] m_axi_dma_arsize,
    output     [                    1:0] m_axi_dma_arburst,
    output     [                    3:0] m_axi_dma_arcache,
    output     [                    2:0] m_axi_dma_arprot,
    output                               m_axi_dma_arvalid,
    input                                m_axi_dma_arready,
    input      [ C_M_AXI_ID_WIDTH-1 : 0] m_axi_dma_rid,
    input      [ C_M_AXI_DATA_WIDTH-1:0] m_axi_dma_rdata,
    input      [                    1:0] m_axi_dma_rresp,
    input                                m_axi_dma_rlast,
    input                                m_axi_dma_rvalid,
    output                               m_axi_dma_rready,
    output                               m_axi_dma_arlock,
    //AXI-4 Stream signals
    output reg [C_AXIS_DATA_WIDTH-1 : 0] m_axis_dma_tdata,
    output reg [C_AXIS_DATA_WIDTH/8-1:0] m_axis_dma_tkeep,
    output                               m_axis_dma_tvalid,
    input                                m_axis_dma_tready,
    output reg                           m_axis_dma_tlast,
    //SGL Buffer signals
    input      [                   15:0] sgl_head_ptr,
    output     [                   14:0] sgl_bram_rd_addr,
    output                               sgl_bram_rd_en,
    input      [   C_SGL_DATA_WIDTH-1:0] sgl_bram_rd_data,
    //Header buffer signals
    output     [                   14:0] hdr_bram_rd_addr,
    output                               hdr_bram_rd_en,
    input      [                  511:0] hdr_bram_rd_data,
    //From Config space
    output reg                           o_dma_in_idle,
    output reg                           hdr_bram_pop
);

    `include "../../rdma_macros.vh"
    //AXIS header and data write control FSM states
    localparam WRITE_HDR_1 = 2'b00;
    localparam WRITE_HDR_2 = 2'b01;
    localparam WRITE_DATA = 2'b10;
    /////////////////////////
    localparam SGL_TAIL_PTR_WIDTH = 16;
    localparam SGL_BRAM_ADDR_WIDTH = clog2(C_MAX_SGL_DEPTH);



    localparam SGL_ADDR_WIDTH = 64;
    localparam SGL_ADDR_LSB = 0;
    localparam SGL_LEN_WIDTH = 16;
    localparam SGL_LEN_LSB = SGL_ADDR_WIDTH;
    localparam SGL_SOF_LSB = SGL_LEN_LSB + SGL_LEN_WIDTH;
    localparam SGL_EOF_LSB = SGL_SOF_LSB + 1;
    localparam SGL_QP1_HDR_VAL_LSB = 82;

    ///////////////////////////////////////////////
    //      AXI stream fifo bit arrangements     //
    // tlast    tkeep      tdata                 //
    //  576     [575:512]  [511:0]               //
    ///////////////////////////////////////////////
    localparam AXIS_FIFO_TDATA_LSB = 0;
    localparam AXIS_FIFO_TDATA_W = 512;
    localparam AXIS_FIFO_TKEEP_LSB = 512;
    localparam AXIS_FIFO_TKEEP_W = 64;
    localparam AXIS_FIFO_TLAST_LSB = 576;
    localparam AXIS_FIFO_TLAST_W = 1;

    //AXI REQ FIFO Parameters
    // 32(ADDR)+8(LEN)+SOF_EOF= 42
    localparam AXI_REQ_FIFO_WIDTH = C_M_AXI_ADDR_WIDTH + 13;

    //AXIS DATA FIFO Parameters  
    //Width = 512(DATA)+64(TKEEP)+1(TLAST) = 577
    localparam AXIS_DATA_FIFO_WIDTH = 512 + 64 + 1;

    //AXI LEN FIFO Parameters  
    //width = 6(LEN) + 1(EOF)= 7 
    localparam AXI_LEN_FIFO_WIDTH = 7;
    localparam AXI_LEN_FIFO_EOF_LSB = 6;

    //HDR REQ FIFO Parameters
    // C_HDR_BUF_ADDR_WIDTH + 6(LEN) + 1(len>64) + 1(len[6]) +1(EOF) + 1(QP1_HDR_VAL)
    localparam HDR_REQ_FIFO_LEN_W = 6;
    localparam HDR_REQ_FIFO_LEN_LSB = (C_HDR_BUF_ADDR_WIDTH + 1);
    localparam HDR_REQ_FIFO_EOF_LSB = (C_HDR_BUF_ADDR_WIDTH + 1) + HDR_REQ_FIFO_LEN_W + 2;
    localparam HDR_QP1_HDR_VAL_LSB = (C_HDR_BUF_ADDR_WIDTH + 1) + HDR_REQ_FIFO_LEN_W + 3;
    localparam HDR_REQ_FIFO_WIDTH = (C_HDR_BUF_ADDR_WIDTH + 1) + HDR_REQ_FIFO_LEN_W + 1 + 1 + 1 + 1;

    ////////
    localparam VALID_DIS = 1'b0;
    localparam VALID_EN = 1'b1;

    reg axi_valid_ps;
    reg axi_valid_ns;

    localparam TVALID_DIS = 1'b0;
    localparam TVALID_EN = 1'b1;

    reg                                axis_valid_ps;
    reg                                axis_valid_ns;
    //Declarations
    reg  [     SGL_TAIL_PTR_WIDTH-1:0] sgl_tail_ptr;
    wire                               sgl_hd_ptr_neq_tail_ptr;  // sgl头指针 != sgl尾指针
    reg                                sgl_data_push;

    wire [     AXI_REQ_FIFO_WIDTH-1:0] axi_req_fifo_data_in;
    wire [     AXI_LEN_FIFO_WIDTH-1:0] axi_len_fifo_data_in;
    wire [     HDR_REQ_FIFO_WIDTH-1:0] hdr_req_fifo_data_in;
    wire [AXIS_DATA_FIFO_WIDTH-1+32:0] axis_data_fifo_data_in;

    wire [     AXI_REQ_FIFO_WIDTH-1:0] axi_req_fifo_data_out;
    wire [     AXI_LEN_FIFO_WIDTH-1:0] axi_len_fifo_data_out;
    wire [     HDR_REQ_FIFO_WIDTH-1:0] hdr_req_fifo_data_out;
    wire [AXIS_DATA_FIFO_WIDTH-1+32:0] axis_data_fifo_data_out;
    wire [AXIS_DATA_FIFO_WIDTH-1+32:0] axis_data_stream_data;
    wire axis_data_fifo_data_valid;


    wire                               axi_req_fifo_push;
    wire                               axi_len_fifo_push;
    wire                               hdr_req_fifo_push;
    wire                               axis_data_fifo_push;

    reg                                axi_req_fifo_pop;
    wire                               axi_len_fifo_pop;
    wire                               hdr_req_fifo_pop;
    wire                               axis_data_fifo_pop;

    wire                               axi_req_fifo_full;
    wire                               axi_len_fifo_full;
    wire                               hdr_req_fifo_full;
    wire                               axis_data_fifo_full;
    wire                               axis_data_fifo_prog_full;
    reg  [                       11:0] data_count;



    wire                               axi_req_fifo_empty;
    wire                               axi_len_fifo_empty;
    wire                               hdr_req_fifo_empty;
    wire                               axis_data_fifo_empty;
    reg                                hdr_req_fifo_empty_reg;

    wire                               axi_req_fifo_wr_rst_busy;
    wire                               axi_len_fifo_wr_rst_busy;
    wire                               hdr_req_fifo_wr_rst_busy;
    wire                               axis_data_fifo_wr_rst_busy;

    wire                               axi_req_fifo_rd_rst_busy;
    wire                               axi_len_fifo_rd_rst_busy;
    wire                               hdr_req_fifo_rd_rst_busy;
    wire                               axis_data_fifo_rd_rst_busy;

    wire                               hdr_wr_en;
    reg                                axis_hdr_push;
    reg                                stg_val;
    reg  [                      576:0] stg_data;
    wire [                       63:0] axis_hdr_data_tkeep;
    wire [                       63:0] axis_payld_data_tkeep;
    wire                               axis_hdr_data_tlast;
    wire                               axis_payld_data_tlast;
    reg  [                        1:0] axis_data_ctrl_ps;
    reg  [                        1:0] axis_data_ctrl_ns;
    wire [       HDR_REQ_FIFO_LEN_W:0] hdr_len;
    wire                               hdr_len_or_bit_7_8;
    wire [       HDR_REQ_FIFO_LEN_W:0] hdr_len_r;
    wire                               hdr_len_or_bit_7_8_r;
    reg                                hdr_data_ps;
    reg                                hdr_data_ns;
    reg  [                        8:0] hdr_req_fifo_data_out_r;
    wire                               axi_dma_rlast_int;
    wire                               m_axis_dma_tvalid_int;
    reg                                m_axi_dma_arvalid_int;
    wire [                      576:0] hdr_data;
    wire [                       31:0] curr_pkt_count;
    reg  [                       31:0] pkt_count;
    reg  [                        3:0] wr_count;
    reg  [                       31:0] rlast_count;

    reg  [                       31:0] arvalid_count;

    ////////////////   

    /////////////////////////////////////
    //          SGL Format             //
    // EOF   SOF    LEN       ADDR     //
    // 81    80     [79:64]   [63:0]   //
    /////////////////////////////////////

    always @(posedge core_clk) begin
        if (core_rst) arvalid_count <= 32'd0;
        else if (m_axi_dma_arvalid && m_axi_dma_arready) arvalid_count <= arvalid_count + 1'b1;
    end

    always @(posedge core_clk) begin
        if (core_rst) rlast_count <= 32'd0;
        else if (axi_dma_rlast_int && m_axi_dma_rready) rlast_count <= rlast_count + 1'b1;
    end


    always @(posedge core_clk) begin
        if (core_rst) wr_count <= 4'h0;
        else begin
            case ({
                hdr_req_fifo_pop, hdr_req_fifo_push
            })
                2'b01:   wr_count <= wr_count + 1'b1;
                2'b10:   wr_count <= wr_count - 1'b1;
                default: wr_count <= wr_count;
            endcase  // case ({hdr_req_fifo_pop,hdr_req_fifo_push})
        end
    end

    assign hdr_req_fifo_full = (wr_count >= 4'hD);

    //tail pointer logic
    always @(posedge core_clk) begin
        if (core_rst) sgl_tail_ptr <= {SGL_TAIL_PTR_WIDTH{1'b0}};
        else if (sgl_bram_rd_en) begin
            if (sgl_tail_ptr[(SGL_TAIL_PTR_WIDTH-2):0] == (C_MAX_SGL_DEPTH - 1'b1)) begin
                sgl_tail_ptr[(SGL_TAIL_PTR_WIDTH-2):0] <= {(SGL_TAIL_PTR_WIDTH - 1) {1'b0}};
                sgl_tail_ptr[SGL_TAIL_PTR_WIDTH-1]     <= ~sgl_tail_ptr[SGL_TAIL_PTR_WIDTH-1];
            end else sgl_tail_ptr[(SGL_TAIL_PTR_WIDTH-2):0] <= sgl_tail_ptr + {{(SGL_TAIL_PTR_WIDTH - 2) {1'b0}}, 1'b1};
        end
    end



    // =========================== SGL BRAM 信号分配 ===========================
    // sgl头指针 != sgl尾指针
    assign sgl_hd_ptr_neq_tail_ptr = (sgl_tail_ptr != sgl_head_ptr);
    // sgl 读使能
    assign sgl_bram_rd_en          = sgl_hd_ptr_neq_tail_ptr & (~axi_req_fifo_full) & (~axi_len_fifo_full) & (~hdr_req_fifo_full);
    // sgl 读地址 
    assign sgl_bram_rd_addr        = sgl_tail_ptr[SGL_BRAM_ADDR_WIDTH-1:0];
    // BRAM 读取延迟一个周期，以作为读取延迟为 1 的 BRAM 数据的有效读取方式
    always @(posedge core_clk) begin
        if (core_rst) sgl_data_push <= 1'b0;
        else sgl_data_push <= sgl_bram_rd_en;
    end
    //axi request fifo push
    assign axi_req_fifo_push = sgl_data_push & ((~sgl_bram_rd_data[SGL_SOF_LSB]) | (sgl_bram_rd_data[SGL_QP1_HDR_VAL_LSB]));
    //axi length fifo push
    assign axi_len_fifo_push = axi_req_fifo_push;
    //header fifo push
    assign hdr_req_fifo_push = sgl_data_push & sgl_bram_rd_data[SGL_SOF_LSB];

    //axi request fifo data
    assign axi_req_fifo_data_in = {sgl_bram_rd_data[(SGL_LEN_LSB+SGL_LEN_WIDTH-4):(SGL_LEN_LSB)], sgl_bram_rd_data[SGL_ADDR_LSB+:C_M_AXI_ADDR_WIDTH]};

    //axi length fifo data
    assign axi_len_fifo_data_in = {sgl_bram_rd_data[SGL_EOF_LSB], sgl_bram_rd_data[SGL_LEN_LSB+:6]};

    //header fifo data
    assign hdr_req_fifo_data_in = {sgl_bram_rd_data[SGL_QP1_HDR_VAL_LSB], sgl_bram_rd_data[SGL_EOF_LSB], (|sgl_bram_rd_data[(SGL_LEN_LSB+8):(SGL_LEN_LSB+7)]), sgl_bram_rd_data[SGL_LEN_LSB+:7], sgl_bram_rd_data[(SGL_ADDR_LSB+6)+:(C_HDR_BUF_ADDR_WIDTH+1)]};


    //axi request fifo pop
    //  assign axi_req_fifo_pop = m_axi_dma_arready && (!axi_req_fifo_empty);
    //axi length fifo pop
    assign axi_len_fifo_pop = (axis_data_ctrl_ps == WRITE_DATA) && axi_dma_rlast_int && (!axis_data_fifo_prog_full) && (!hdr_wr_en);
    //header fifo pop
    assign hdr_req_fifo_pop = (!hdr_req_fifo_empty)    ? (((axis_data_ctrl_ps == WRITE_HDR_1) && (( hdr_len <= 7'd64) && (!hdr_len_or_bit_7_8) 
                                                      && hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB]) && (!axis_data_fifo_prog_full)
                                                      && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB]))
                                                     ||((axis_data_ctrl_ps == WRITE_HDR_2) && (!axis_data_fifo_prog_full) 
                                                        && hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB])
                                                     ||((axis_data_ctrl_ps == WRITE_DATA) && axi_dma_rlast_int && (!hdr_wr_en) && (!axis_data_fifo_prog_full))) 
                                                     : 1'b0;

    assign hdr_len = hdr_req_fifo_data_out[HDR_REQ_FIFO_LEN_LSB+:(HDR_REQ_FIFO_LEN_W+1)];
    assign hdr_len_or_bit_7_8 = hdr_req_fifo_data_out[HDR_REQ_FIFO_LEN_LSB+HDR_REQ_FIFO_LEN_W+1];

    always @(posedge core_clk) begin
        if (core_rst) begin
            data_count <= 12'd0;
        end else begin
            case ({
                axis_data_fifo_pop, axis_data_fifo_push
            })
                2'b01:   data_count <= data_count + 12'd1;
                2'b10:   data_count <= data_count - 12'd1;
                default: data_count <= data_count;
            endcase  // case (axis_data_fifo_pop,axis_data_fifo_push)
        end
    end


    assign axis_data_fifo_prog_full = data_count >= 12'd2040;


    always @(posedge core_clk)
        if (core_rst) hdr_req_fifo_data_out_r <= 9'd0;
        else hdr_req_fifo_data_out_r <= hdr_req_fifo_data_out[HDR_REQ_FIFO_LEN_LSB+:(HDR_REQ_FIFO_LEN_W+3)];

    assign hdr_len_r            = hdr_req_fifo_data_out_r[0+:7];
    assign hdr_len_or_bit_7_8_r = hdr_req_fifo_data_out_r[7];

    //Axi-4 read interface signals
    //static assignments
    assign m_axi_dma_arid       = {C_M_AXI_ID_WIDTH{1'b1}};
    assign m_axi_dma_arsize     = 3'b110;
    assign m_axi_dma_arlock     = 1'b0;
    assign m_axi_dma_arburst    = 2'b01;
    assign m_axi_dma_arcache    = 4'h3;
    assign m_axi_dma_arprot     = 3'b000;
    assign axi_dma_rlast_int    = m_axi_dma_rvalid ? m_axi_dma_rlast : 1'b0;

    //Dynamic assignments

    always @(*) begin
        m_axi_dma_araddr = axi_req_fifo_data_out[SGL_ADDR_LSB+:C_M_AXI_ADDR_WIDTH];
        m_axi_dma_arlen  = {1'b0, axi_req_fifo_data_out[(C_M_AXI_ADDR_WIDTH+6)+:7]} - {{7{1'b0}}, (~|axi_req_fifo_data_out[C_M_AXI_ADDR_WIDTH+:6])};
    end

    always @(*) begin
        if (((!m_axi_dma_arvalid_int) && (!axi_req_fifo_empty))) axi_req_fifo_pop = 1'b1;
        else if (m_axi_dma_arvalid_int && m_axi_dma_arready && (!axi_req_fifo_empty)) axi_req_fifo_pop = 1'b1;
        else axi_req_fifo_pop = 1'b0;
    end

    always @(posedge core_clk) begin
        if (core_rst) begin
            m_axi_dma_arvalid_int <= 1'b0;
        end else if ((!axi_req_fifo_empty) && ((!m_axi_dma_arvalid_int) || (m_axi_dma_arvalid_int && m_axi_dma_arready))) begin
            m_axi_dma_arvalid_int <= 1'b1;
        end else if (m_axi_dma_arvalid_int && m_axi_dma_arready) begin
            m_axi_dma_arvalid_int <= 1'b0;
        end
    end
    assign m_axi_dma_arvalid = m_axi_dma_arvalid_int;

    assign m_axi_dma_rready  = (~axis_data_fifo_prog_full) & (~hdr_wr_en);


    //Axi stream signal assignments
    ///////////////////////////////////////////////
    //      AXI stream fifo bit arrangements     //
    // tlast    tkeep      tdata                 //
    //  576     [575:512]  [511:0]               //
    ///////////////////////////////////////////////

    always @(*) begin
        m_axis_dma_tdata = axis_data_stream_data[AXIS_FIFO_TDATA_LSB+:AXIS_FIFO_TDATA_W];
        m_axis_dma_tkeep = axis_data_stream_data[AXIS_FIFO_TKEEP_LSB+:AXIS_FIFO_TKEEP_W];
        m_axis_dma_tlast = axis_data_stream_data[AXIS_FIFO_TLAST_LSB];
    end

    // Three-cycle XPM output pipeline enables local URAM output registers.
    // Elastic credits include in-flight reads, so a sudden DRE stall is safe.
    // DEPTH=8 covers the read latency without throttling steady-state traffic.
    fifo_read_elastic #(
        .DATA_WIDTH(AXIS_DATA_FIFO_WIDTH + 32),
        .DEPTH(8)
    ) axis_data_read_pipe_inst (
        .clk(core_clk),
        .rst(core_rst),
        .fifo_busy(axis_data_fifo_rd_rst_busy || axis_data_fifo_wr_rst_busy),
        .fifo_empty(axis_data_fifo_empty),
        .fifo_rd_en(axis_data_fifo_pop),
        .fifo_data(axis_data_fifo_data_out),
        .fifo_data_valid(axis_data_fifo_data_valid),
        .m_axis_data(axis_data_stream_data),
        .m_axis_valid(m_axis_dma_tvalid_int),
        .m_axis_ready(m_axis_dma_tready)
    );

    assign m_axis_dma_tvalid = m_axis_dma_tvalid_int;

    //Axi read transactions fifo

    //Fifo comprising of axi address and length
    //32(ADDR)+8(LEN)+SOF_EOF= 42
    generate
        if (C_VIVADO_VER == 1'b1) begin : XPM_16_3_0
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE   ("auto"),              //string; "auto", "block", "distributed", or "ultra";									
                .ECC_MODE           ("no_ecc"),            //string; "no_ecc" or "en_ecc";									
                .FIFO_WRITE_DEPTH   (32),                  //positive integer									
                .WRITE_DATA_WIDTH   (AXI_REQ_FIFO_WIDTH),  //positive integer									
                .WR_DATA_COUNT_WIDTH(6),                   //positive integer									
                .PROG_FULL_THRESH   (27),                  //positive integer									
                .FULL_RESET_VALUE   (0),                   //positive integer; 0 or 1									
                .READ_MODE          ("std"),               //string; "std" or "fwft";									
                .FIFO_READ_LATENCY  (1),                   //positive integer;									
                .READ_DATA_WIDTH    (AXI_REQ_FIFO_WIDTH),  //positive integer									
                .RD_DATA_COUNT_WIDTH(6),                   //positive integer									
                .PROG_EMPTY_THRESH  (5),                   //positive integer									
                .DOUT_RESET_VALUE   ("0"),                 //string									
                .WAKEUP_TIME        (0)                    //positive integer; 0 or 2;									
            ) axi_req_fifo (
                .sleep        (1'b0),
                .rst          (core_rst),
                .wr_clk       (core_clk),
                .wr_en        (axi_req_fifo_push),
                .din          (axi_req_fifo_data_in),
                .full         (axi_req_fifo_full),
                .prog_full    (),
                .wr_data_count(),
                .overflow     (),
                .wr_rst_busy  (axi_req_fifo_wr_rst_busy),
                .rd_en        (axi_req_fifo_pop),
                .dout         (axi_req_fifo_data_out),
                .empty        (axi_req_fifo_empty),
                .prog_empty   (),
                .rd_data_count(),
                .underflow    (),
                .rd_rst_busy  (axi_req_fifo_rd_rst_busy),
                .injectsbiterr(1'b0),
                .injectdbiterr(1'b0),
                .sbiterr      (),
                .dbiterr      ()
            );
        end else begin  //2016.1 version
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE ("auto"),              //string; "auto", "bram", "lutram", "uram" or "builtin";
                .FIFO_WRITE_DEPTH (32),                  //positive integer
                .WRITE_DATA_WIDTH (AXI_REQ_FIFO_WIDTH),  //positive integer
                .READ_MODE        ("std"),               //string; "std" or "fwft";
                .FIFO_READ_LATENCY(1),                   //positive integer; 0 or 1;
                .READ_DATA_WIDTH  (AXI_REQ_FIFO_WIDTH),  //positive integer
                .WRCOUNT_TYPE     ("disable_wr_dc"),     //do not change
                .PROG_FULL_THRESH (27),                  //do not change
                .PROG_EMPTY_THRESH(5),                   //do not change
                .DOUT_RESET_VALUE ("0"),                 //do not change
                .ECC_MODE         ("no_ecc"),            //do not change
                .EN_ECC_PIPE      (0),                   //do not change
                .WAKEUP_TIME      (0),                   //do not change
                .AUTO_SLEEP_TIME  (0)                    //do not change
            ) axi_req_fifo (

                .rst          (core_rst),
                .wr_clk       (core_clk),
                .wr_en        (axi_req_fifo_push),
                .din          (axi_req_fifo_data_in),
                .full         (axi_req_fifo_full),
                .overflow     (),
                .wr_rst_busy  (axi_req_fifo_wr_rst_busy),
                .rd_en        (axi_req_fifo_pop),
                .dout         (axi_req_fifo_data_out),
                .empty        (axi_req_fifo_empty),
                .underflow    (),
                .rd_rst_busy  (axi_req_fifo_rd_rst_busy),
                .prog_full    (),                          // do not change
                .wr_data_count(),                          // do not change
                .prog_empty   (),                          // do not change
                .rd_data_count(),                          // do not change
                .sleep        (1'b0),                      // do not change
                .injectsbiterr(1'b0),                      // do not change
                .injectdbiterr(1'b0),                      // do not change
                .sbiterr      (),                          // do not change
                .dbiterr      ()                           // do not change
            );
        end  // else: !if(C_VIVADO_VER == 1'b1)
    endgenerate



    //axi read data fifo(depth 2x64--512 width) to store atleast two axi 4K transactions

    //Width = 512(DATA)+64(TKEEP)+1(TLAST) = 577
    generate
        if (C_VIVADO_VER == 1'b1) begin : XPM_16_3_1
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE   ("ultra"),                     //string; "auto", "block", "distributed", or "ultra";									
                .ECC_MODE           ("no_ecc"),
                .USE_ADV_FEATURES   ("1000"), // data_valid for registered responses									
                .FIFO_WRITE_DEPTH   (2048),                       //positive integer									
                .WRITE_DATA_WIDTH   (AXIS_DATA_FIFO_WIDTH + 32),  //positive integer									
                .WR_DATA_COUNT_WIDTH(12),                         //positive integer									
                .PROG_FULL_THRESH   (2040),                       //positive integer									
                .FULL_RESET_VALUE   (0),                          //positive integer; 0 or 1									
                .READ_MODE          ("std"),                      //string; "std" or "fwft";									
                .FIFO_READ_LATENCY  (3),                          //positive integer;									
                .READ_DATA_WIDTH    (AXIS_DATA_FIFO_WIDTH + 32),  //positive integer									
                .RD_DATA_COUNT_WIDTH(12),                          //positive integer									
                .PROG_EMPTY_THRESH  (12),                         //positive integer									
                .DOUT_RESET_VALUE   ("0"),                        //string									
                .WAKEUP_TIME        (0)                           //positive integer; 0 or 2;									
            ) axis_data_fifo (
                .sleep        (1'b0),
                .rst          (core_rst),
                .wr_clk       (core_clk),
                .wr_en        (axis_data_fifo_push),
                .din          (axis_data_fifo_data_in),
                .full         (),
                .prog_full    (),
                .wr_data_count(),
                .overflow     (),
                .wr_rst_busy  (axis_data_fifo_wr_rst_busy),
                .rd_en        (axis_data_fifo_pop),
                .dout         (axis_data_fifo_data_out),
                .data_valid   (axis_data_fifo_data_valid),
                .empty        (axis_data_fifo_empty),
                .prog_empty   (),
                .rd_data_count(),
                .underflow    (),
                .rd_rst_busy  (axis_data_fifo_rd_rst_busy),
                .injectsbiterr(1'b0),
                .injectdbiterr(1'b0),
                .sbiterr      (),
                .dbiterr      ()
            );
        end // block: XPM_16_3_1
    else begin
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE ("ultra"),                     //string; "auto", "bram", "lutram", "uram" or "builtin";
                .FIFO_WRITE_DEPTH (2048),                       //positive integer
                .WRITE_DATA_WIDTH (AXIS_DATA_FIFO_WIDTH + 32),  //positive integer
                .READ_MODE        ("std"),                      //string; "std" or "fwft";
                .FIFO_READ_LATENCY(3),                          //positive integer; 0 or 1;
                .READ_DATA_WIDTH  (AXIS_DATA_FIFO_WIDTH + 32),  //positive integer
                .WRCOUNT_TYPE     ("disable_wr_dc"),            //do not change
                .PROG_FULL_THRESH (2040),                       //do not change
                .PROG_EMPTY_THRESH(5),                          //do not change
                .DOUT_RESET_VALUE ("0"),                        //do not change
                .ECC_MODE         ("no_ecc"),
                .USE_ADV_FEATURES ("1000"),                   //do not change
                .EN_ECC_PIPE      (0),                          //do not change
                .WAKEUP_TIME      (0),                          //do not change
                .AUTO_SLEEP_TIME  (0)                           //do not change
            ) axis_data_fifo (

                .rst          (core_rst),
                .wr_clk       (core_clk),
                .wr_en        (axis_data_fifo_push),
                .din          (axis_data_fifo_data_in),
                .full         (),
                .overflow     (),
                .wr_rst_busy  (axis_data_fifo_wr_rst_busy),
                .rd_en        (axis_data_fifo_pop),
                .dout         (axis_data_fifo_data_out),
                .data_valid   (axis_data_fifo_data_valid),
                .empty        (axis_data_fifo_empty),
                .underflow    (),
                .rd_rst_busy  (axis_data_fifo_rd_rst_busy),
                .prog_full    (),                            // do not change
                .wr_data_count(),                            // do not change
                .prog_empty   (),                            // do not change
                .rd_data_count(),                            // do not change
                .sleep        (1'b0),                        // do not change
                .injectsbiterr(1'b0),                        // do not change
                .injectdbiterr(1'b0),                        // do not change
                .sbiterr      (),                            // do not change
                .dbiterr      ()                             // do not change
            );
        end  // else: !if(C_VIVADO_VER == 1'b1)
    endgenerate

    //axi stream interface logic      
    //derive tkeep from residue length on the last beat of the AXI-4 transaction.

    //Fifo comprising of axi length which will be used to calculate the AXIS interface 
    //tkeep for last beat of AXI-4 transaction. The axi length width is 6 bits as
    //64 beats are at max possible for 512 bit wide data bus to be within 4K range.
    //The bits [5:0] of the length will represent the valid data bytes in last beat
    //ehich will be used to generate the appropriate TKEEP.

    //width = 6(LEN) + 1(EOF)= 7 
    generate
        if (C_VIVADO_VER == 1'b1) begin : XPM_16_3_2
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE   ("auto"),              //string; "auto", "block", "distributed", or "ultra";									
                .ECC_MODE           ("no_ecc"),            //string; "no_ecc" or "en_ecc";									
                .FIFO_WRITE_DEPTH   (32),                  //positive integer									
                .WRITE_DATA_WIDTH   (AXI_LEN_FIFO_WIDTH),  //positive integer									
                .WR_DATA_COUNT_WIDTH(6),                   //positive integer									
                .PROG_FULL_THRESH   (27),                  //positive integer									
                .FULL_RESET_VALUE   (0),                   //positive integer; 0 or 1									
                .READ_MODE          ("fwft"),              //string; "std" or "fwft";									
                .FIFO_READ_LATENCY  (0),                   //positive integer;									
                .READ_DATA_WIDTH    (AXI_LEN_FIFO_WIDTH),  //positive integer									
                .RD_DATA_COUNT_WIDTH(6),                   //positive integer									
                .PROG_EMPTY_THRESH  (5),                   //positive integer									
                .DOUT_RESET_VALUE   ("0"),                 //string									
                .WAKEUP_TIME        (0)                    //positive integer; 0 or 2;									
            ) axi_len_fifo (
                .sleep        (1'b0),
                .rst          (core_rst),
                .wr_clk       (core_clk),
                .wr_en        (axi_len_fifo_push),
                .din          (axi_len_fifo_data_in),
                .full         (axi_len_fifo_full),
                .prog_full    (),
                .wr_data_count(),
                .overflow     (),
                .wr_rst_busy  (axi_len_fifo_wr_rst_busy),
                .rd_en        (axi_len_fifo_pop),
                .dout         (axi_len_fifo_data_out),
                .empty        (axi_len_fifo_empty),
                .prog_empty   (),
                .rd_data_count(),
                .underflow    (),
                .rd_rst_busy  (axi_len_fifo_rd_rst_busy),
                .injectsbiterr(1'b0),
                .injectdbiterr(1'b0),
                .sbiterr      (),
                .dbiterr      ()
            );
        end // block: XPM_16_3_2
    else begin
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE ("auto"),              //string; "auto", "bram", "lutram", "uram" or "builtin";
                .FIFO_WRITE_DEPTH (32),                  //positive integer
                .WRITE_DATA_WIDTH (AXI_LEN_FIFO_WIDTH),  //positive integer
                .READ_MODE        ("fwft"),              //string; "std" or "fwft";
                .FIFO_READ_LATENCY(0),                   //positive integer; 0 or 1;
                .READ_DATA_WIDTH  (AXI_LEN_FIFO_WIDTH),  //positive integer
                .WRCOUNT_TYPE     ("disable_wr_dc"),     //do not change
                .PROG_FULL_THRESH (27),                  //do not change
                .PROG_EMPTY_THRESH(5),                   //do not change
                .DOUT_RESET_VALUE ("0"),                 //do not change
                .ECC_MODE         ("no_ecc"),            //do not change
                .EN_ECC_PIPE      (0),                   //do not change
                .WAKEUP_TIME      (0),                   //do not change
                .AUTO_SLEEP_TIME  (0)                    //do not change
            ) axi_len_fifo (

                .rst          (core_rst),
                .wr_clk       (core_clk),
                .wr_en        (axi_len_fifo_push),
                .din          (axi_len_fifo_data_in),
                .full         (axi_len_fifo_full),
                .overflow     (),
                .wr_rst_busy  (axi_len_fifo_wr_rst_busy),
                .rd_en        (axi_len_fifo_pop),
                .dout         (axi_len_fifo_data_out),
                .empty        (axi_len_fifo_empty),
                .underflow    (),
                .rd_rst_busy  (axi_len_fifo_rd_rst_busy),
                .prog_full    (),                          // do not change
                .wr_data_count(),                          // do not change
                .prog_empty   (),                          // do not change
                .rd_data_count(),                          // do not change
                .sleep        (1'b0),                      // do not change
                .injectsbiterr(1'b0),                      // do not change
                .injectdbiterr(1'b0),                      // do not change
                .sbiterr      (),                          // do not change
                .dbiterr      ()                           // do not change
            );
        end  // else: !if(C_VIVADO_VER == 1'b1)
    endgenerate

    //Header request fifo to sore the header address and length
    // C_HDR_BUF_ADDR_WIDTH + 8(LEN) + 1(EOF)
    generate
        if (C_VIVADO_VER == 1'b1) begin : XPM_16_3_3
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE   ("auto"),              //string; "auto", "block", "distributed", or "ultra";									
                .ECC_MODE           ("no_ecc"),            //string; "no_ecc" or "en_ecc";									
                .FIFO_WRITE_DEPTH   (32),                  //positive integer									
                .WRITE_DATA_WIDTH   (HDR_REQ_FIFO_WIDTH),  //positive integer									
                .WR_DATA_COUNT_WIDTH(6),                   //positive integer									
                .PROG_FULL_THRESH   (27),                  //positive integer									
                .FULL_RESET_VALUE   (0),                   //positive integer; 0 or 1									
                .READ_MODE          ("fwft"),              //string; "std" or "fwft";									
                .FIFO_READ_LATENCY  (0),                   //positive integer;									
                .READ_DATA_WIDTH    (HDR_REQ_FIFO_WIDTH),  //positive integer									
                .RD_DATA_COUNT_WIDTH(6),                   //positive integer									
                .PROG_EMPTY_THRESH  (5),                   //positive integer									
                .DOUT_RESET_VALUE   ("0"),                 //string									
                .WAKEUP_TIME        (0)                    //positive integer; 0 or 2;									
            ) hdr_req_fifo (
                .sleep        (1'b0),
                .rst          (core_rst),
                .wr_clk       (core_clk),
                .wr_en        (hdr_req_fifo_push),
                .din          (hdr_req_fifo_data_in),
                .full         (),
                .prog_full    (),
                .wr_data_count(),
                .overflow     (),
                .wr_rst_busy  (hdr_req_fifo_wr_rst_busy),
                .rd_en        (hdr_req_fifo_pop),
                .dout         (hdr_req_fifo_data_out),
                .empty        (hdr_req_fifo_empty),
                .prog_empty   (),
                .rd_data_count(),
                .underflow    (),
                .rd_rst_busy  (hdr_req_fifo_rd_rst_busy),
                .injectsbiterr(1'b0),
                .injectdbiterr(1'b0),
                .sbiterr      (),
                .dbiterr      ()
            );
        end // block: XPM_16_3_3
    else begin
            xpm_fifo_sync_wrap #(
                .FIFO_MEMORY_TYPE ("auto"),              //string; "auto", "bram", "lutram", "uram" or "builtin";
                .FIFO_WRITE_DEPTH (32),                  //positive integer
                .WRITE_DATA_WIDTH (HDR_REQ_FIFO_WIDTH),  //positive integer
                .READ_MODE        ("fwft"),              //string; "std" or "fwft";
                .FIFO_READ_LATENCY(0),                   //positive integer; 0 or 1;
                .READ_DATA_WIDTH  (HDR_REQ_FIFO_WIDTH),  //positive integer
                .WRCOUNT_TYPE     ("disable_wr_dc"),     //do not change
                .PROG_FULL_THRESH (27),                  //do not change
                .PROG_EMPTY_THRESH(5),                   //do not change
                .DOUT_RESET_VALUE ("0"),                 //do not change
                .ECC_MODE         ("no_ecc"),            //do not change
                .EN_ECC_PIPE      (0),                   //do not change
                .WAKEUP_TIME      (0),                   //do not change
                .AUTO_SLEEP_TIME  (0)                    //do not change
            ) hdr_req_fifo (

                .rst          (core_rst),
                .wr_clk       (core_clk),
                .wr_en        (hdr_req_fifo_push),
                .din          (hdr_req_fifo_data_in),
                .full         (),
                .overflow     (),
                .wr_rst_busy  (hdr_req_fifo_wr_rst_busy),
                .rd_en        (hdr_req_fifo_pop),
                .dout         (hdr_req_fifo_data_out),
                .empty        (hdr_req_fifo_empty),
                .underflow    (),
                .rd_rst_busy  (hdr_req_fifo_rd_rst_busy),
                .prog_full    (),                          // do not change
                .wr_data_count(),                          // do not change
                .prog_empty   (),                          // do not change
                .rd_data_count(),                          // do not change
                .sleep        (1'b0),                      // do not change
                .injectsbiterr(1'b0),                      // do not change
                .injectdbiterr(1'b0),                      // do not change
                .sbiterr      (),                          // do not change
                .dbiterr      ()                           // do not change
            );
        end  // else: !if(C_VIVADO_VER == 1'b1)
    endgenerate

    //AXIS data write control FSM

    always @(posedge core_clk) begin
        if (core_rst) axis_data_ctrl_ps <= WRITE_HDR_1;
        else if ((!axis_data_fifo_prog_full)) axis_data_ctrl_ps <= axis_data_ctrl_ns;
    end

    always @(*) begin
        case (axis_data_ctrl_ps)
            WRITE_HDR_1: begin
                if (!hdr_req_fifo_empty) begin
                    if (((hdr_len > 7'd64) || hdr_len_or_bit_7_8) && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB])) axis_data_ctrl_ns = WRITE_HDR_2;
                    else if (hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB] && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB])) axis_data_ctrl_ns = WRITE_HDR_1;
                    else axis_data_ctrl_ns = WRITE_DATA;
                end else axis_data_ctrl_ns = WRITE_HDR_1;
            end
            WRITE_HDR_2: begin
                if (hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB]) axis_data_ctrl_ns = WRITE_HDR_1;
                else axis_data_ctrl_ns = WRITE_DATA;
            end
            WRITE_DATA: begin
                if (axi_dma_rlast_int && (!hdr_wr_en)) begin
                    //          if(!hdr_req_fifo_empty)begin
                    //            if((hdr_len > 7'd64) || hdr_len_or_bit_7_8)
                    //              axis_data_ctrl_ns = WRITE_HDR_2;
                    //            else if(hdr_req_fifo_data_out[HDR_REQ_FIFO_EOF_LSB])
                    axis_data_ctrl_ns = WRITE_HDR_1;
                    //            else  
                    //              axis_data_ctrl_ns = WRITE_DATA;
                    //          end
                    //          else 
                    //            axis_data_ctrl_ns = WRITE_HDR_1;                  
                end else axis_data_ctrl_ns = WRITE_DATA;
            end
            default: begin
                axis_data_ctrl_ns = WRITE_HDR_1;
            end
        endcase  // case (axis_data_ctrl_ps)
    end

    assign hdr_wr_en = ((axis_data_ctrl_ps == WRITE_HDR_1) && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB])) || (axis_data_ctrl_ps == WRITE_HDR_2) || axis_hdr_push;

    always @(posedge core_clk) begin
        if (core_rst) axis_hdr_push <= 1'b0;
        else axis_hdr_push <= hdr_bram_rd_en;
    end

    assign axis_data_fifo_push = ((~axis_data_fifo_prog_full) & m_axi_dma_rvalid & (~hdr_wr_en)) | axis_hdr_push;

    //HEADER BRAM interface Signals

    assign hdr_bram_rd_en =  (!axis_data_fifo_prog_full) && (!hdr_req_fifo_empty) && (!stg_val) && (!hdr_req_fifo_data_out[HDR_QP1_HDR_VAL_LSB])
                           && ((axis_data_ctrl_ps == WRITE_HDR_1) || (axis_data_ctrl_ps == WRITE_HDR_2)); //|| ((axis_data_ctrl_ps == WRITE_DATA) && axi_dma_rlast_int && (!hdr_wr_en)));

    assign hdr_bram_rd_addr = {hdr_req_fifo_data_out[C_HDR_BUF_ADDR_WIDTH:1], (axis_data_ctrl_ps == WRITE_HDR_2)};

    always @(posedge core_clk) begin
        if (core_rst) begin
            hdr_bram_pop <= 1'b0;
        end else begin
            hdr_bram_pop <= hdr_bram_rd_en && ((axis_data_ctrl_ps == WRITE_HDR_2) || ((hdr_len <= 7'd64) && (!hdr_len_or_bit_7_8)));
        end
    end

    //Header buffering logic to save the extra rd_en given to header bram when the fifo is full.
    // 头缓冲逻辑，用于在 fifo 已满时保存给头 bram 的额外 rd_en。
    always @(posedge core_clk) begin
        if (core_rst) begin
            stg_val  <= 1'b0;
            stg_data <= {577{1'b0}};
        end else if ((!axis_data_fifo_prog_full) && stg_val) begin
            stg_val  <= 1'b0;
            stg_data <= {577{1'b0}};
        end else if (axis_data_fifo_prog_full && hdr_bram_rd_en) begin
            stg_val  <= 1'b1;
            stg_data <= {axis_hdr_data_tlast, axis_hdr_data_tkeep, hdr_data};  //hdr_bram_rd_data;
        end
    end

    always @(posedge core_clk)
        if (core_rst) hdr_data_ps <= 1'b0;
        else hdr_data_ps <= hdr_data_ns;

    always @(*) begin
        case (hdr_data_ps)
            1'b0: begin
                if (axis_hdr_push && ((hdr_len_r > 7'd64) || hdr_len_or_bit_7_8_r)) hdr_data_ns = 1'b1;
                else hdr_data_ns = 1'b0;
            end
            1'b1: begin
                if (axis_hdr_push) hdr_data_ns = 1'b0;
                else hdr_data_ns = 1'b1;
            end
        endcase  // case (hdr_data_ps)
    end

    always @(posedge core_clk)
        if (core_rst) pkt_count <= 32'd0;
        else if (axis_data_fifo_push) pkt_count <= pkt_count + 1'b1;

    genvar i;

    generate
        for (i = 0; i < 64; i = i + 1) begin : loop1
            assign axis_hdr_data_tkeep[i]   = ((!hdr_data_ps) && ((hdr_len_r > 7'd64) || hdr_len_or_bit_7_8_r)) ? 1'b1 : ((hdr_len_r[5:0] >= (i + 1)) || (~|hdr_len_r[5:0])) ? 1'b1 : 1'b0;
            assign axis_payld_data_tkeep[i] = axi_dma_rlast_int ? ((axi_len_fifo_data_out[5:0] >= (i + 1)) || (~|axi_len_fifo_data_out[5:0])) ? 1'b1 : 1'b0 : 1'b1;
        end
    endgenerate

    assign hdr_data               = {axis_hdr_data_tlast, axis_hdr_data_tkeep, hdr_bram_rd_data};

    assign axis_hdr_data_tlast    = (((!hdr_data_ps) && (!((hdr_len_r > 7'd64) || hdr_len_or_bit_7_8_r))) || hdr_data_ps) ? hdr_req_fifo_data_out_r[8] : 1'b0;
    assign axis_payld_data_tlast  = axi_len_fifo_data_out[AXI_LEN_FIFO_EOF_LSB] & axi_dma_rlast_int;
    assign axis_data_fifo_data_in = (hdr_wr_en) ? {pkt_count, hdr_data} : {pkt_count, axis_payld_data_tlast, axis_payld_data_tkeep, m_axi_dma_rdata};
    assign curr_pkt_count         = axis_data_stream_data[AXIS_DATA_FIFO_WIDTH+:32];


    always @(posedge core_clk) begin
        if (core_rst) hdr_req_fifo_empty_reg <= 1'b0;
        else hdr_req_fifo_empty_reg <= hdr_req_fifo_empty;
    end
    always @(posedge core_clk) begin
        if (core_rst) o_dma_in_idle <= 1'b1;
        else if (sgl_hd_ptr_neq_tail_ptr) o_dma_in_idle <= 1'b0;
        else if ((!sgl_hd_ptr_neq_tail_ptr) && (!hdr_req_fifo_empty_reg) && hdr_req_fifo_empty) o_dma_in_idle <= 1'b1;
    end


endmodule  // wqe_proc_dma


// Standard-read response bridge is co-located with its only caller.
// Existing Vivado sessions already compile this file; no added-source dependency.
`timescale 1ns/1ps

// Convert a standard-read FIFO with registered, variable-latency responses
// to ready/valid. Reserve storage at read issue, not when data comes back.
// Credits cover BOTH pending reads and queued responses; backpressure cannot
// overwrite unread words. Payload RAM has no reset; valid/counts gate all reads.
module fifo_read_elastic #(
    parameter integer DATA_WIDTH = 609,
    parameter integer DEPTH      = 8
) (
    input  wire                  clk,
    input  wire                  rst,

    input  wire                  fifo_busy,
    input  wire                  fifo_empty,
    output wire                  fifo_rd_en,
    input  wire [DATA_WIDTH-1:0]  fifo_data,
    input  wire                  fifo_data_valid,

    output wire [DATA_WIDTH-1:0]  m_axis_data,
    output wire                  m_axis_valid,
    input  wire                  m_axis_ready
);
    localparam integer PTR_WIDTH = $clog2(DEPTH);
    localparam integer CNT_WIDTH = $clog2(DEPTH + 1);
    localparam [CNT_WIDTH-1:0] CAPACITY = DEPTH;

    (* ram_style = "distributed" *)
    reg [DATA_WIDTH-1:0] payload [0:DEPTH-1];
    reg [PTR_WIDTH-1:0] wr_ptr, rd_ptr;
    reg [CNT_WIDTH-1:0] reserved, queued;

    wire reset_active = rst || fifo_busy;
    wire returned = fifo_data_valid && !reset_active;
    wire consumed = m_axis_valid && m_axis_ready;

    // No combinational READY path back to the memory request port.
    assign fifo_rd_en = !reset_active && !fifo_empty && (reserved < CAPACITY);
    assign m_axis_valid = !reset_active && (queued != 0);
    assign m_axis_data = payload[rd_ptr];

    always @(posedge clk) begin
        // Reset flushes ownership/credits; stale writes during reset are unobservable.
        // Keep the global reset out of the wide payload RAM write enable.
        if (fifo_data_valid) payload[wr_ptr] <= fifo_data;
    end

    always @(posedge clk) begin
        if (reset_active) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            reserved <= 0;
            queued <= 0;
        end else begin
            if (returned) wr_ptr <= wr_ptr + 1'b1;
            if (consumed) rd_ptr <= rd_ptr + 1'b1;
            case ({fifo_rd_en, consumed})
                2'b10: reserved <= reserved + 1'b1;
                2'b01: reserved <= reserved - 1'b1;
                default: begin end
            endcase
            case ({returned, consumed})
                2'b10: queued <= queued + 1'b1;
                2'b01: queued <= queued - 1'b1;
                default: begin end
            endcase
        end
    end

    // Elaboration/simulation guards only; no timing exceptions or fake ready.
    initial begin
        if (DATA_WIDTH < 1 || DEPTH < 4 || (DEPTH & (DEPTH-1)) != 0)
            $fatal(1, "fifo_read_elastic requires power-of-two DEPTH >= 4");
    end
`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (!reset_active) begin
            if (reserved > CAPACITY || queued > reserved)
                $fatal(1, "FIFO credit invariant failed");
            if (returned && (reserved == 0))
                $fatal(1, "FIFO response without reserved credit");
            if (returned && queued == CAPACITY && !consumed)
                $fatal(1, "FIFO response overflow");
        end
    end
`endif
endmodule
