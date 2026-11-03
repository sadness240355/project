/////////////////////////////////////////////////////////////////
// (c) Copyright 1984 / 2012 Xilinx, Inc. All rights reserved.	 
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
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON/	     
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
// Xilinx products are not designed or intended to be fail/	     
// safe, or for use in any application requiring fail/safe	     
// performance, such as life/support or safety devices or	     
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
/////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// axi_master 
///////////////////////////////////////////////////////////////////////////////
//
// Filename:     qp_mgr_axi_master.v
// Version:      v1.0
// Description:  This file has all the logic to initiate the AXI master
//               transactions. Length decrement logic is implemented to
//               indicate when to terminate the transaction.
// Verilog-Standard: Verilog'2001
///////////////////////////////////////////////////////////////////////////////
// Naming Conventions:
//      active low signals:                     "*_n"
//      clock signals:                          "clk", "clk_div#", "clk_#x"
//      reset signals:                          "rst", "rst_n"
//      generics:                               "C_*"
//      user defined types:                     "*_TYPE"
//      state machine next state:               "*_ns"
//      state machine current state:            "*_cs"
//      combinatorial signals:                  "*_cmb"
//      pipelined or register delay signals:    "*_d#"
//      counter signals:                        "*cnt*"
//      clock enable signals:                   "*_ce"
//      internal version of output port         "*_i"
//      device pins:                            "*_pin"
//      ports:                                  / Names begin with Uppercase
//      processes:                              "*_PROCESS"
//      component instantiations:               "<ENTITY_>I_<#|FUNC>
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
// 该文件包含启动 AXI 主事务的所有逻辑。长度递减逻辑用于指示何时终止事务。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module qp_mgr_axi_master #(
    parameter C_M_AXI_ADDR_WIDTH      = 32,
    parameter C_M_AXI_DATA_WIDTH      = 512,
    parameter C_M_AXI_THREAD_ID_WIDTH = 1,
    parameter IP2BUS_LEN_WIDTH        = 12,
    parameter C_QP_INDX_WIDTH         = 5,
    parameter C_EN_OUTSTANDING_WRITE  = 0
) (
    // 负责从 DDR 中读取数据
    // AXI System signals
    input                                  m_axi_aclk,
    input                                  m_axi_aresetn,
    // AXI Master signals
    output [C_M_AXI_THREAD_ID_WIDTH-1 : 0] m_axi_awid,
    output [       C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output [                          7:0] m_axi_awlen,
    output [                          2:0] m_axi_awsize,
    output [                          1:0] m_axi_awburst,
    output [                          3:0] m_axi_awcache,
    output [                          2:0] m_axi_awprot,
    output                                 m_axi_awvalid,
    input                                  m_axi_awready,
    output [       C_M_AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output [     C_M_AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output                                 m_axi_wlast,
    output                                 m_axi_wvalid,
    input                                  m_axi_wready,
    output                                 m_axi_awlock,
    input  [C_M_AXI_THREAD_ID_WIDTH-1 : 0] m_axi_bid,
    input  [                          1:0] m_axi_bresp,
    input                                  m_axi_bvalid,
    output                                 m_axi_bready,
    output [C_M_AXI_THREAD_ID_WIDTH-1 : 0] m_axi_arid,
    output [       C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output [                          7:0] m_axi_arlen,
    output [                          2:0] m_axi_arsize,
    output [                          1:0] m_axi_arburst,
    output [                          3:0] m_axi_arcache,
    output [                          2:0] m_axi_arprot,
    output                                 m_axi_arvalid,
    input                                  m_axi_arready,
    input  [C_M_AXI_THREAD_ID_WIDTH-1 : 0] m_axi_rid,
    input  [       C_M_AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  [                          1:0] m_axi_rresp,
    input                                  m_axi_rlast,
    input                                  m_axi_rvalid,
    output                                 m_axi_rready,
    output                                 m_axi_arlock,

    input  [     C_QP_INDX_WIDTH-1:0] i_qp_sq_arbitrated_idx,  // cache 这次读请求绑定的 QP ID，也就是在 cache 内仲裁完的 SQ QP ID
    output [C_M_AXI_DATA_WIDTH/8-1:0] bus2ip_byte_en,          // 字节有效
    output [  C_M_AXI_DATA_WIDTH-1:0] bus2ip_data,             // 从 DDR 取出来的 SQ WQE 数据
    output [     C_QP_INDX_WIDTH-1:0] o_qp_sq_idx,             // i_qp_sq_arbitrated_idx 进入 FIFO 后出来 o_qp_sq_idx
    output                            bus2ip_dvalid,           // R 通道数据有效回执，cache 用它维护在途计数/统计
    output                            bus2ip_arready,          // AR 握手成功回执，cache 用它判断“地址已被 axi_master 接收”
    input  [  C_M_AXI_DATA_WIDTH-1:0] ip2bus_data,             // 512'b0
    output                            bus2ip_data_rdy,         // 1'b0
    input                             axi_m_en,                // cache 发起“可读”请求，打一拍后驱动 AXI 事务启动
    input                             wr_rdn,                  // 1'b0；1 = write; 0 = read
    input  [  C_M_AXI_ADDR_WIDTH-1:0] ip2bus_addr,             // cache 计算出的 DDR 地址送给 axi_master 的 AR 地址通道
    input  [    IP2BUS_LEN_WIDTH-1:0] ip2bus_len,              // WQE_SIZE_IN_BYTES(64), 长度（以字节为单位）

    // Status indicators to the core IP 
    output axi_master_done,
    output axi_master_busy,
    output axi_master_bvalid,
    output axi_master_error

);

    //------------------------------------------------------------------------------
    // Function clogb2
    // This function returns the integer ceiling of the base 2 logarithm of x,
    // i.e., the least integer greater than or equal to log2(x).
    //------------------------------------------------------------------------------
    function integer clogb2;
        input [31:0] value;  // Input value
        begin
            for (clogb2 = 0; value > 1; clogb2 = clogb2 + 1) value = value >> 1;
        end
    endfunction

    localparam LEN_LSB = clogb2(C_M_AXI_DATA_WIDTH / 8);
    localparam rd_addr_Idle = 2'b00;
    localparam rd_request = 2'b01;
    localparam rd_clear_addr_cycle = 2'b10;

    localparam wr_addr_Idle = 2'b00;
    localparam wr_request = 2'b01;
    localparam wr_clear_addr_cycle = 2'b10;

    localparam write_idle = 3'b000;
    localparam wait_write_trans0 = 3'b001;
    localparam wait_write_trans1 = 3'b010;
    localparam wait_write_trans2 = 3'b011;
    localparam write_request = 3'b100;
    localparam write_dataphase = 3'b101;
    localparam handle_wrbterm = 3'b110;
    localparam wait_busy_deassert = 3'b111;

    localparam read_idle = 3'b000;
    localparam wait_read_trans1 = 3'b001;
    localparam wait_read_trans2 = 3'b010;
    localparam read_request = 3'b011;
    localparam read_request_one = 3'b100;
    localparam read_dataphase = 3'b101;
    localparam wait_rd_one_cycle = 3'b110;
    localparam wait_writephase_comp = 3'b111;

    reg  [                     1:0] rd_addr_state_machine_cs;
    reg  [                     1:0] wr_addr_state_machine_cs;
    reg  [                     2:0] write_state_machine_cs;
    reg  [                     2:0] read_state_machine_cs;
    // Read S/M signals
    reg                             start_read;
    reg                             dma_rderr_hold_cs;
    reg                             dma_error_rd_cs;
    reg                             dma_done_rd_cs;
    reg                             dma_busy_rd_cs;

    // Writ S/M signals
    reg                             start_write;
    reg                             dma_wrerr_hold_cs;
    reg                             dma_error_wr_cs;
    reg                             dma_done_wr_cs;
    reg                             dma_busy_wr_cs;
    reg                             length_wrcount_en_em;
    reg  [                     7:0] wr_rd_load;
    wire                            wrbuffer_addr_zero;
    wire                            wrbuffer_addr_one;

    reg                             m_axi_wlast_cs;
    reg                             m_axi_wvalid_cs;
    reg                             temp_wvalid;
    reg                             wready_r;
    reg                             temp_flag;
    reg                             m_axi_rready_cs;
    reg                             m_axi_bready_cs;
    reg                             m_axi_awvalid_cs;
    reg                             m_axi_arvalid_cs;
    reg  [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr_cs;
    reg  [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr_cs;
    reg  [                     7:0] m_awlen_in_wr_cs;
    reg  [                     7:0] m_awlen_in_rd_cs;
    reg  [C_M_AXI_DATA_WIDTH/8-1:0] byte_en_i;
    reg  [    IP2BUS_LEN_WIDTH-1:0] Length_Reg;
    reg                             in_write_req;
    reg                             single_beat;
    reg  [  C_M_AXI_DATA_WIDTH-1:0] wdata;
    reg  [C_M_AXI_DATA_WIDTH/8-1:0] wstrb;

    wire                            length_fifo_empty;
    wire                            length_fifo_full;
    wire [    IP2BUS_LEN_WIDTH-1:0] length_fifo_dout;
    wire                            length_fifo_push;
    wire                            length_fifo_pop;

    ///////////////////////////////////////////////////////////////////////////////
    // Combinatorial operations
    ///////////////////////////////////////////////////////////////////////////////
    // Fixed signals
    assign m_axi_awid        = {C_M_AXI_THREAD_ID_WIDTH{1'b0}};
    assign m_axi_awvalid     = m_axi_awvalid_cs;
    assign m_axi_awaddr      = m_axi_awaddr_cs;
    assign m_axi_awlen       = m_awlen_in_wr_cs;
    assign m_axi_awsize      = (ip2bus_len > 4'h4) ? LEN_LSB : 'h2;
    assign m_axi_awburst     = 2'b01;
    assign m_axi_awcache     = 4'h3;
    assign m_axi_awprot      = 3'b000;
    assign m_axi_wlast       = m_axi_wlast_cs;
    assign m_axi_wstrb       = single_beat ? wstrb : byte_en_i;
    assign m_axi_wvalid      = m_axi_wvalid_cs;
    assign m_axi_wdata       = single_beat ? wdata : ip2bus_data;  //for multiple beats it is expected that data is aligned with address
    assign m_axi_bready      = m_axi_bready_cs;
    assign m_axi_arid        = {C_M_AXI_THREAD_ID_WIDTH{1'b0}};
    assign m_axi_arvalid     = m_axi_arvalid_cs;
    assign m_axi_araddr      = m_axi_araddr_cs;
    assign m_axi_arlen       = m_awlen_in_rd_cs;
    assign m_axi_arsize      = LEN_LSB;
    assign m_axi_arburst     = 2'b01;
    assign m_axi_arcache     = 4'h3;
    assign m_axi_arprot      = 3'b000;
    assign m_axi_rready      = m_axi_rready_cs;
    assign m_axi_awlock      = 1'b0;
    assign m_axi_arlock      = 1'b0;
    assign axi_master_error  = dma_error_rd_cs || dma_error_wr_cs;
    assign axi_master_done   = dma_done_rd_cs || dma_done_wr_cs;
    assign axi_master_busy   = dma_busy_rd_cs || dma_busy_wr_cs;
    assign axi_master_bvalid = m_axi_bvalid && m_axi_bready_cs;
    assign bus2ip_data_rdy   = m_axi_wvalid && m_axi_wready;

    assign length_fifo_push  = axi_m_en && ~wr_rdn;

    // {仲裁结果的QP ID，长度} FIFO
    // 这个 FIFO 在这里是“数据常显
    fifo #(
        .C_Q_DEPTH     (16),                                  // FIFO Depth
        .C_DATA_SIZE   (IP2BUS_LEN_WIDTH + C_QP_INDX_WIDTH),  // FIFO Data Width
        .C_THRESHOLD   (1),                                   // Threshold value for almost full
        .C_BRAM_N_FLOP (0),                                   // 1- BRAM, 0 - FLOPS
        .C_READ_LATENCY(0)                                    // 1- Fallthough FIFO, 0-Non-fall through FIFO
    ) length_fifo (
        .core_clk           (m_axi_aclk),
        .core_rst           (!m_axi_aresetn),
        .o_q_empty          (length_fifo_empty),
        .o_q_full           (length_fifo_full),
        .o_q_almost_full    (),
        .o_num_valid_entries(),
        .o_num_free_entries (),
        .i_q_push           (length_fifo_push),
        .i_q_push_data      ({i_qp_sq_arbitrated_idx, ip2bus_len}),
        .i_q_pop            (length_fifo_pop),                       // POP 的作用是“确认消费并前进到下一个队头”
        .o_q_pop_data       ({o_qp_sq_idx, length_fifo_dout}),       // 这里数据是常显的（队头预览），pop后读指针才前进
        .o_q_pop_data_vld   (),
        .tb_sim_eot         (1'b0)
    );

    ///////////////////////////////////////////////////////////////////////////////
    // byte enables for the read data
    // during the Controller's read from the address location 
    ///////////////////////////////////////////////////////////////////////////////
    genvar i;
    generate
        for (i = 0; i < C_M_AXI_DATA_WIDTH / 8; i = i + 1) begin : BYTE_EN_GEN
            assign bus2ip_byte_en[i] = (m_axi_rvalid && m_axi_rready_cs) && byte_en_i[i];

            ///////////////////////////////////////////////////////////////////////////////
            // Byte enables for read data during a read or write data during a write
            ///////////////////////////////////////////////////////////////////////////////
            always @(posedge m_axi_aclk) begin
                if (~m_axi_aresetn) begin
                    byte_en_i[i] <= 1'b0;
                end else begin
                    if (axi_m_en && ip2bus_len > i) begin
                        byte_en_i[i] <= 1'b1;
                    end else if (dma_done_wr_cs || dma_done_rd_cs) begin
                        byte_en_i[i] <= 1'b0;
                    end
                end
            end
        end
    endgenerate
    assign bus2ip_data     = m_axi_rdata;
    assign bus2ip_dvalid   = m_axi_rvalid && m_axi_rready_cs;
    assign bus2ip_arready  = m_axi_arready & m_axi_arvalid;
    assign Length_Count_En = length_wrcount_en_em || (m_axi_rvalid && m_axi_rready_cs);
    assign length_fifo_pop = m_axi_rvalid & m_axi_rready_cs;

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            wstrb       <= {C_M_AXI_DATA_WIDTH / 8{1'b0}};
            wdata       <= {C_M_AXI_DATA_WIDTH{1'b0}};
            single_beat <= 1'b0;
        end else begin
            wstrb <= byte_en_i << ip2bus_addr[LEN_LSB-1:0];
            wdata <= ip2bus_data << (8 * ip2bus_addr[LEN_LSB-1:0]);
            if (axi_m_en && ~axi_master_busy) begin
                single_beat <= (ip2bus_len <= C_M_AXI_DATA_WIDTH / 8);
            end
        end
    end

    ///////////////////////////////////////////////////////////////////////////////
    //Length decrement counter ////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    always @(posedge m_axi_aclk) begin
        if (!m_axi_aresetn) begin
            Length_Reg <= {IP2BUS_LEN_WIDTH{1'b0}};
        end else begin
            if (read_state_machine_cs == read_idle && write_state_machine_cs == write_idle) begin
                if (~(|length_fifo_dout[LEN_LSB-1 : 0])) begin
                    Length_Reg[IP2BUS_LEN_WIDTH-1:LEN_LSB] <= length_fifo_dout[IP2BUS_LEN_WIDTH-1:LEN_LSB] - 1'b1;
                    Length_Reg[LEN_LSB-1:0]                <= length_fifo_dout[LEN_LSB-1:0] - 1'b1;
                end else begin
                    Length_Reg <= length_fifo_dout;
                end
            end else if (Length_Count_En && (|Length_Reg[IP2BUS_LEN_WIDTH-1:LEN_LSB])) begin
                Length_Reg[IP2BUS_LEN_WIDTH-1:LEN_LSB] <= Length_Reg[IP2BUS_LEN_WIDTH-1:LEN_LSB] - 1'b1;
                Length_Reg[LEN_LSB-1:0]                <= {LEN_LSB{1'b0}};
            end else if (Length_Count_En && |Length_Reg[LEN_LSB-1:0]) begin
                Length_Reg[LEN_LSB-1:0] <= {LEN_LSB{1'b0}};
            end
        end
    end

    ///////////////////////////////////////////////////////////////////////////////
    // WR_BUFFER_ADDR_COUNT_I is a down counter. Initially the counter is
    // loaded with the length of the transfer and the counter is decremented
    // for every correct  write data acknowledge of the transaction.
    ///////////////////////////////////////////////////////////////////////////////

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            wr_rd_load <= 8'h00;
            //end else if (start_write || start_read) begin
        end else if (axi_m_en) begin
            if (~(|ip2bus_len[LEN_LSB-1 : 0])) begin
                wr_rd_load <= ip2bus_len[IP2BUS_LEN_WIDTH-1:LEN_LSB] - 8'h01;
            end else begin
                wr_rd_load <= ip2bus_len[IP2BUS_LEN_WIDTH-1:LEN_LSB];
            end
        end else if ((length_wrcount_en_em || (m_axi_rvalid && m_axi_rready_cs)) && (wr_rd_load != 8'h00)) begin
            wr_rd_load <= wr_rd_load - 8'h01;
        end else begin
            wr_rd_load <= wr_rd_load;
        end
    end

    assign wrbuffer_addr_zero = (wr_rd_load == 8'h00);

    assign wrbuffer_addr_one  = (wr_rd_load == 8'h01);

    ///////////////////////////////////////////////////////////////////////////////
    // AXI Read Address Phase state machine
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Address state machine is initiated whenever the read statemachine or write
    // statemachine want to initiate a transaction on the AXI Bus. The
    // statemachine tries to get hold of the bus ownership until slave replies with
    // an address ack for the address cycle.In the event of slave rearbitration
    // signal assertion master statemachine deasserts its requesting signals for
    // one clock.There is an extra state clear_addr_cycle added to come out of the
    // statemachine gracefully after the completion of the Address cycle of the AXI
    // transaction.Once any read request or write request is finished the
    // statemachine returns to the addr_Idle state
    ///////////////////////////////////////////////////////////////////////////////

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            rd_addr_state_machine_cs <= rd_addr_Idle;
            m_axi_arvalid_cs         <= 1'b0;
            m_axi_araddr_cs          <= {C_M_AXI_ADDR_WIDTH{1'b0}};
            m_awlen_in_rd_cs         <= 8'h00;
        end else begin
            case (rd_addr_state_machine_cs)
                rd_addr_Idle: begin
                    m_axi_araddr_cs <= ip2bus_addr;
                    //if (start_read ) begin                
                    if (axi_m_en) begin
                        rd_addr_state_machine_cs <= rd_request;
                    end else begin
                        m_awlen_in_rd_cs         <= 8'h00;
                        rd_addr_state_machine_cs <= rd_addr_Idle;
                    end
                end

                rd_request: begin
                    m_axi_arvalid_cs <= 1'b1;
                    if (~(|ip2bus_len[LEN_LSB-1:0])) begin
                        m_awlen_in_rd_cs <= ip2bus_len[IP2BUS_LEN_WIDTH-1 : LEN_LSB] - 1'b1;
                    end else begin
                        m_awlen_in_rd_cs <= ip2bus_len[IP2BUS_LEN_WIDTH-1 : LEN_LSB];
                    end
                    rd_addr_state_machine_cs <= rd_clear_addr_cycle;
                end

                rd_clear_addr_cycle: begin
                    m_axi_arvalid_cs <= 1'b1;
                    if (m_axi_arready) begin
                        m_axi_arvalid_cs         <= 1'b0;
                        m_axi_araddr_cs          <= {C_M_AXI_ADDR_WIDTH{1'b0}};
                        m_awlen_in_rd_cs         <= 8'h00;
                        rd_addr_state_machine_cs <= rd_addr_Idle;
                    end else begin
                        rd_addr_state_machine_cs <= rd_clear_addr_cycle;
                    end
                end
                // coverage off    
                default: begin
                    rd_addr_state_machine_cs <= rd_addr_Idle;
                end
                // coverage on    
            endcase
        end
    end

    ///////////////////////////////////////////////////////////////////////////////
    // AXI Read Phase state machine
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // AXI Read phase state machine is initiated whenever the Length Register is
    // written.If the length register programmed is a non zero value then the read
    // state machine requests the address statemachine to put a request on the AXI
    // bus.The Read phase state machine waits until the address cycle of the AXI
    // transaction was completed sucessfully.
    ///////////////////////////////////////////////////////////////////////////////

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            read_state_machine_cs <= read_idle;
            dma_done_rd_cs        <= 1'b0;
            dma_busy_rd_cs        <= 1'b0;
            start_read            <= 1'b0;
            dma_rderr_hold_cs     <= 1'b0;
            m_axi_rready_cs       <= 1'b0;
        end else begin
            case (read_state_machine_cs)
                read_idle: begin
                    dma_done_rd_cs  <= 1'b0;
                    dma_error_rd_cs <= 1'b0;
                    start_read      <= 1'b0;
                    //if (axi_m_en && ~wr_rdn) begin
                    if (!length_fifo_empty) begin
                        read_state_machine_cs <= wait_read_trans1;
                        dma_busy_rd_cs        <= 1'b1;
                    end else begin
                        read_state_machine_cs <= read_idle;
                        dma_busy_rd_cs        <= 1'b0;
                    end
                end
                ///////////////////////////////////////////////////////////////////////
                // Read phase state machine is initiated whenever the Length Register
                // is written.If the length register is programmed with a non zero
                // value then the Read state machine requests the address statemachine
                // to put a request on the AXI bus.
                ///////////////////////////////////////////////////////////////////////
                wait_read_trans1: begin
                    if (|Length_Reg) begin
                        start_read            <= 1'b1;
                        read_state_machine_cs <= wait_read_trans2;
                    end else begin
                        read_state_machine_cs <= read_idle;
                        dma_done_rd_cs        <= 1'b1;
                    end
                end

                wait_read_trans2: begin
                    start_read            <= 1'b0;
                    dma_done_rd_cs        <= 1'b0;
                    read_state_machine_cs <= read_request;
                end

                read_request: begin
                    read_state_machine_cs <= read_dataphase;
                end

                ///////////////////////////////////////////////////////////////////////
                // The Read phase state machine waits until the address cycle of the
                // AXI transaction was completed sucessfully.
                ///////////////////////////////////////////////////////////////////////
                read_dataphase: begin
                    dma_done_rd_cs <= 1'b0;
                    if (m_axi_rvalid && m_axi_rready_cs && wrbuffer_addr_one) begin
                        m_axi_rready_cs       <= 1'b0;
                        read_state_machine_cs <= read_request_one;
                        if (m_axi_rresp != 2'b00) begin
                            dma_rderr_hold_cs <= 1'b1;
                        end
                    end else if (m_axi_rvalid && m_axi_rready_cs) begin
                        if (m_axi_rresp != 2'b00) begin
                            dma_rderr_hold_cs <= 1'b1;
                        end
                        if (m_axi_rlast) begin
                            m_axi_rready_cs       <= 1'b0;
                            read_state_machine_cs <= wait_rd_one_cycle;
                        end else begin
                            m_axi_rready_cs <= 1'b1;
                        end
                    end else begin
                        m_axi_rready_cs <= 1'b1;
                    end
                end

                read_request_one: begin
                    if (m_axi_rvalid && m_axi_rready_cs) begin
                        m_axi_rready_cs <= 1'b0;
                        if (m_axi_rresp != 2'b00) begin
                            dma_rderr_hold_cs <= 1'b1;
                        end
                        read_state_machine_cs <= wait_rd_one_cycle;
                    end else begin
                        m_axi_rready_cs       <= 1'b1;
                        read_state_machine_cs <= read_request_one;
                    end
                end

                wait_rd_one_cycle: begin
                    read_state_machine_cs <= wait_writephase_comp;
                end

                ///////////////////////////////////////////////////////////////////////
                // Each cycle in the read data phase was is valid if the data cycle
                // ends with an read data ack with no error signal asserted along with
                // it.The read data phase is interrupted with slave ending the
                // transaction with read burst terminate signal,master continues to
                // complete the transfer with a new request on the AXI
                ///////////////////////////////////////////////////////////////////////
                wait_writephase_comp: begin
                    if (~(|Length_Reg)) begin
                        dma_busy_rd_cs        <= 1'b0;
                        read_state_machine_cs <= read_idle;
                        if (dma_rderr_hold_cs) begin
                            dma_error_rd_cs   <= 1'b1;
                            dma_rderr_hold_cs <= 1'b0;
                            dma_done_rd_cs    <= 1'b0;
                        end else begin
                            dma_error_rd_cs   <= 1'b0;
                            dma_rderr_hold_cs <= 1'b0;
                            dma_done_rd_cs    <= 1'b1;
                        end
                    end
                end
                // coverage off    
                default: begin
                    read_state_machine_cs <= read_idle;
                end
                // coverage on    
            endcase
        end
    end

    ///////////////////////////////////////////////////////////////////////////////
    // AXI Write Address Phase state machine
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // Address state machine is initiated whenever the read statemachine or write
    // statemachine want to initiate a transaction on the AXI Bus. The
    // statemachine tries to get hold of the bus ownership until slave replies with
    // an address ack for the address cycle.In the event of slave rearbitration
    // signal assertion master statemachine deasserts its requesting signals for
    // one clock.There is an extra state clear_addr_cycle added to come out of the
    // statemachine gracefully after the completion of the Address cycle of the AXI
    // transaction.Once any read request or write request is finished the
    // statemachine returns to the addr_Idle state
    ///////////////////////////////////////////////////////////////////////////////

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            wr_addr_state_machine_cs <= wr_addr_Idle;
            m_axi_awvalid_cs         <= 1'b0;
            m_axi_awaddr_cs          <= {C_M_AXI_ADDR_WIDTH{1'b0}};
            m_awlen_in_wr_cs         <= 8'h00;
        end else begin
            case (wr_addr_state_machine_cs)
                wr_addr_Idle: begin
                    m_axi_awaddr_cs <= ip2bus_addr;
                    if (start_write) begin
                        wr_addr_state_machine_cs <= wr_request;
                        m_axi_awvalid_cs         <= 1'b1;
                        if (~(|ip2bus_len[LEN_LSB-1:0])) begin
                            m_awlen_in_wr_cs <= ip2bus_len[IP2BUS_LEN_WIDTH-1:LEN_LSB] - 1'b1;
                        end else begin
                            m_awlen_in_wr_cs <= ip2bus_len[IP2BUS_LEN_WIDTH-1:LEN_LSB];
                        end
                    end else begin
                        wr_addr_state_machine_cs <= wr_addr_Idle;
                        m_axi_awvalid_cs         <= 1'b0;
                        m_awlen_in_wr_cs         <= 8'h00;
                    end
                end

                wr_request: begin
                    m_axi_awvalid_cs <= 1'b1;
                    if (m_axi_awready) begin
                        wr_addr_state_machine_cs <= wr_clear_addr_cycle;
                        m_axi_awvalid_cs         <= 1'b0;
                        m_axi_awaddr_cs          <= {C_M_AXI_ADDR_WIDTH{1'b0}};
                        m_awlen_in_wr_cs         <= 8'h00;
                    end else begin
                        wr_addr_state_machine_cs <= wr_request;
                    end
                end

                wr_clear_addr_cycle: begin
                    m_awlen_in_wr_cs         <= 8'h00;
                    wr_addr_state_machine_cs <= wr_addr_Idle;
                end
                // coverage off    
                default: begin
                    wr_addr_state_machine_cs <= wr_addr_Idle;
                end
                // coverage on    
            endcase
        end
    end

    ///////////////////////////////////////////////////////////////////////////////
    // AXI Write state machine
    ///////////////////////////////////////////////////////////////////////////////

    always @(posedge m_axi_aclk) begin
        if (~m_axi_aresetn) begin
            write_state_machine_cs <= write_idle;
            dma_done_wr_cs         <= 1'b0;
            dma_busy_wr_cs         <= 1'b0;
            dma_wrerr_hold_cs      <= 1'b0;
            dma_error_wr_cs        <= 1'b0;
            start_write            <= 1'b0;
            m_axi_wvalid_cs        <= 1'b0;
            m_axi_bready_cs        <= 1'b0;
            m_axi_wlast_cs         <= 1'b0;
            length_wrcount_en_em   <= 1'b0;
            in_write_req           <= 1'b0;
            temp_wvalid            <= 1'b0;
            wready_r               <= 1'b0;
            temp_flag              <= 1'b0;
        end else begin
            wready_r <= m_axi_wready;
            case (write_state_machine_cs)

                write_idle: begin
                    length_wrcount_en_em <= 1'b0;
                    dma_error_wr_cs      <= 1'b0;
                    dma_done_wr_cs       <= 1'b0;
                    start_write          <= 1'b0;
                    if (wr_rdn && axi_m_en) begin
                        dma_busy_wr_cs         <= 1'b1;
                        write_state_machine_cs <= wait_write_trans0;
                    end else begin
                        dma_busy_wr_cs         <= 1'b0;
                        write_state_machine_cs <= write_idle;
                    end
                end

                wait_write_trans0: begin
                    length_wrcount_en_em <= 1'b0;
                    if (~(|Length_Reg)) begin
                        dma_wrerr_hold_cs <= 1'b0;
                        dma_busy_wr_cs    <= 1'b0;
                        //write_state_machine_cs  <= write_idle;
                        if (dma_wrerr_hold_cs) begin
                            dma_error_wr_cs <= 1'b1;
                            dma_done_wr_cs  <= 1'b0;
                        end else if (wr_addr_state_machine_cs == wr_addr_Idle) begin
                            dma_error_wr_cs        <= 1'b0;
                            dma_done_wr_cs         <= 1'b1;
                            write_state_machine_cs <= write_idle;
                        end
                    end else begin
                        start_write            <= 1'b1;
                        write_state_machine_cs <= wait_write_trans2;
                    end
                end

                wait_write_trans2: begin
                    start_write            <= 1'b0;
                    write_state_machine_cs <= wait_write_trans1;
                    length_wrcount_en_em   <= 1'b0;
                end

                wait_write_trans1: begin
                    length_wrcount_en_em <= 1'b1;
                    m_axi_wvalid_cs      <= 1'b1;
                    m_axi_bready_cs      <= 1'b1;
                    if ((wrbuffer_addr_zero && ~temp_flag) || (wrbuffer_addr_zero && temp_flag && m_axi_wready)) begin
                        write_state_machine_cs <= handle_wrbterm;
                        m_axi_wlast_cs         <= 1'b1;
                        temp_flag              <= 1'b0;
                    end else if (wrbuffer_addr_one) begin
                        write_state_machine_cs <= wait_write_trans1;
                        if (m_axi_wvalid_cs && m_axi_wready) begin
                            m_axi_wlast_cs         <= 1'b1;
                            write_state_machine_cs <= handle_wrbterm;
                        end else begin
                            m_axi_wlast_cs <= 1'b0;
                        end
                        temp_flag <= ~m_axi_wready;
                    end else if (~temp_flag) begin
                        write_state_machine_cs <= write_request;
                        m_axi_wlast_cs         <= 1'b0;
                    end
                end

                handle_wrbterm: begin
                    in_write_req         <= 1'b0;
                    length_wrcount_en_em <= 1'b0;
                    m_axi_bready_cs      <= 1'b1;
                    if (m_axi_wlast_cs && m_axi_wready) begin
                        m_axi_wlast_cs         <= 1'b0;
                        m_axi_wvalid_cs        <= 1'b0;
                        write_state_machine_cs <= wait_busy_deassert;
                    end else begin
                        write_state_machine_cs <= handle_wrbterm;
                    end
                end

                write_request: begin
                    m_axi_bready_cs <= 1'b1;
                    in_write_req    <= 1'b1;
                    if ((m_axi_wvalid_cs && m_axi_wready)) begin
                        if ((wrbuffer_addr_one && wready_r) || wrbuffer_addr_zero) begin
                            m_axi_wlast_cs         <= 1'b0;
                            m_axi_wvalid_cs        <= 1'b0;
                            length_wrcount_en_em   <= 1'b1;
                            write_state_machine_cs <= wait_write_trans2;
                        end else begin
                            length_wrcount_en_em   <= 1'b1;
                            m_axi_wvalid_cs        <= 1'b1;
                            write_state_machine_cs <= write_request;
                        end
                    end else begin
                        length_wrcount_en_em   <= 1'b0;
                        write_state_machine_cs <= write_request;
                    end
                end

                wait_busy_deassert: begin
                    length_wrcount_en_em <= 1'b0;
                    m_axi_bready_cs      <= 1'b1;
                    if ((m_axi_bvalid == 1'b1 && C_EN_OUTSTANDING_WRITE == 0) || (C_EN_OUTSTANDING_WRITE == 1)) begin
                        if (|m_axi_bresp) begin
                            dma_wrerr_hold_cs <= 1'b1;
                            dma_done_wr_cs    <= 1'b0;
                            if (C_EN_OUTSTANDING_WRITE == 0) begin
                                m_axi_bready_cs <= 1'b0;
                            end
                            write_state_machine_cs <= wait_write_trans0;
                        end else begin
                            if (C_EN_OUTSTANDING_WRITE == 0) begin
                                m_axi_bready_cs <= 1'b0;
                            end
                            write_state_machine_cs <= wait_write_trans0;
                        end
                    end else begin
                        write_state_machine_cs <= wait_busy_deassert;
                    end
                end

                // coverage off       
                default: begin
                    length_wrcount_en_em   <= 1'b0;
                    write_state_machine_cs <= write_idle;
                end
                // coverage on     
            endcase
        end
    end

endmodule
