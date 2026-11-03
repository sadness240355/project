`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_qp_mgr_axi_master — QP 管理器 AXI4 Master 读接口模块
//
// 【模块概述】
// 专门负责从 DDR 中读取 SQ 的 WQE（Work Queue Entry）数据。仅使用 AXI 读通道
// （AR/R），写通道（AW/W/B）全部 tie-off 为无效值。
//
// 【工作流程】
// 1. lrdma_qp_wqe_cache 模块通过 axi_m_en 触发读请求，锁存地址 ip2bus_addr 和长度 ip2bus_len
// 2. 同时将 {QP_ID, length} 推入内部 length_fifo
// 3. 读地址 FSM 将地址放上 AR 通道，等待 arready 握手
// 4. 读数据 FSM 通过 rvalid/rready 握手接收数据
// 5. 内部长度递减计数器 wr_rd_load 每收一拍减 1，归零标志本次传输完成
//
// 【内部 FSM】
// - 读地址 FSM（3 状态）：rd_addr_Idle → rd_request → rd_clear_addr_cycle → rd_addr_Idle
//   负责 AR 握手：rd_request 拍拉 arvalid 并算好 arlen，rd_clear_addr_cycle 等 arready=1
//   后再把 arvalid 拉低、回 Idle。
// - 读数据 FSM（8 状态）：read_idle → wait_read_trans1 → wait_read_trans2 → read_request
//   → read_dataphase → read_request_one → wait_rd_one_cycle → wait_writephase_comp → read_idle
//   read_dataphase 是主体：逐拍拉 rready 接收 rdata；当只剩最后一拍（wrbuffer_addr_one=1）
//   时切到 read_request_one 单独等那一拍 rvalid，便于把错误状态/收尾干净处理。
//
// 【特性】
// - WQE 大小 64 字节，AXI 数据总线 512 位（64 字节），通常一次 burst 仅需 1 拍
// - length_fifo 缓存 {QP_ID, burst_len}，确保乱序场景下数据与 QP 正确关联
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_qp_mgr_axi_master #(
    parameter C_M_AXI_ADDR_WIDTH      = 32,
    parameter C_M_AXI_DATA_WIDTH      = 512,
    parameter C_M_AXI_THREAD_ID_WIDTH = 1,
    parameter IP2BUS_LEN_WIDTH        = 12,
    parameter C_QP_INDX_WIDTH         = 5
) (
    // 负责从 DDR 中读取数据
    // AXI System signals
    input                                  m_axi_aclk,
    input                                  m_axi_aresetn,
    // AXI Master signals
    // 写通道和写响应通道固定拉低，QP WQE 读取路径只发起读事务
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
    // 只用到了读通道
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
    output [C_M_AXI_DATA_WIDTH/8-1:0] original_wqe_byte_en,    // 字节有效
    output [  C_M_AXI_DATA_WIDTH-1:0] original_wqe_data,       // 从 DDR 取出来的 SQ WQE 数据
    output [     C_QP_INDX_WIDTH-1:0] o_qp_sq_idx,             // i_qp_sq_arbitrated_idx 进入 FIFO 后出来 o_qp_sq_idx
    output                            original_wqe_dvalid,     // R 通道数据有效回执，cache 用它维护在途计数/统计
    output                            original_wqe_arready,    // AR 握手成功回执，cache 用它判断“地址已被 lrdma_qp_mgr_axi_master 接收”
    output                            original_wqe_data_rdy,   // 1'b0
    input                             axi_m_en,                // cache 发起"可读"请求，打一拍后驱动 AXI 事务启动
    input  [  C_M_AXI_ADDR_WIDTH-1:0] ip2bus_addr,             // cache 计算出的 DDR 地址送给 lrdma_qp_mgr_axi_master 的 AR 地址通道
    input  [    IP2BUS_LEN_WIDTH-1:0] ip2bus_len,              // WQE_SIZE_IN_BYTES(64), 长度（以字节为单位）

    // Status indicators to the core IP 
    output lrdma_qp_mgr_axi_master_done,
    output lrdma_qp_mgr_axi_master_busy,
    output lrdma_qp_mgr_axi_master_bvalid,
    output lrdma_qp_mgr_axi_master_error

);

    //------------------------------------------------------------------------------
    // Function clogb2
    // 当值> 0时，该函数返回floor(log2(value))。
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

    localparam read_idle = 3'b000;
    localparam wait_read_trans1 = 3'b001;
    localparam wait_read_trans2 = 3'b010;
    localparam read_request = 3'b011;
    localparam read_request_one = 3'b100;
    localparam read_dataphase = 3'b101;
    localparam wait_rd_one_cycle = 3'b110;
    localparam wait_writephase_comp = 3'b111;

    reg  [                     1:0] rd_addr_state_machine_cs;
    reg  [                     2:0] read_state_machine_cs;
    // Read S/M signals
    reg                             dma_rderr_hold_cs;
    reg                             dma_error_rd_cs;
    reg                             dma_done_rd_cs;
    reg                             dma_busy_rd_cs;


    reg  [                     7:0] wr_rd_load;
    wire                            wrbuffer_addr_zero;
    wire                            wrbuffer_addr_one;

    reg                             m_axi_rready_cs;
    reg                             m_axi_arvalid_cs;
    reg  [  C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr_cs;
    reg  [                     7:0] m_awlen_in_rd_cs;
    reg  [C_M_AXI_DATA_WIDTH/8-1:0] byte_en_i;
    reg  [    IP2BUS_LEN_WIDTH-1:0] length_reg;

    wire                            length_count_en;

    wire                            length_fifo_empty;
    wire                            length_fifo_full;
    wire [    IP2BUS_LEN_WIDTH-1:0] length_fifo_dout;
    wire                            length_fifo_push;
    wire                            length_fifo_pop;

    ///////////////////////////////////////////////////////////////////////////////
    // Combinatorial operations
    ///////////////////////////////////////////////////////////////////////////////
    // AW 通道固定拉低，读模式下不发起写地址请求
    assign m_axi_awid                     = {C_M_AXI_THREAD_ID_WIDTH{1'b0}};
    assign m_axi_awvalid                  = 1'b0;
    assign m_axi_awaddr                   = {C_M_AXI_ADDR_WIDTH{1'b0}};
    assign m_axi_awlen                    = 8'h00;
    assign m_axi_awsize                   = 3'b000;
    assign m_axi_awburst                  = 2'b01;
    assign m_axi_awcache                  = 4'h3;
    assign m_axi_awprot                   = 3'b000;
    assign m_axi_awlock                   = 1'b0;
    // W 通道固定拉低，读模式下不发起写数据请求
    assign m_axi_wlast                    = 1'b0;
    assign m_axi_wstrb                    = {C_M_AXI_DATA_WIDTH / 8{1'b0}};
    assign m_axi_wvalid                   = 1'b0;
    assign m_axi_wdata                    = {C_M_AXI_DATA_WIDTH{1'b0}};
    // B 通道固定拉低，读模式下不接收写响应
    assign m_axi_bready                   = 1'b0;
    // AR channel
    assign m_axi_arid                     = {C_M_AXI_THREAD_ID_WIDTH{1'b0}};
    assign m_axi_arvalid                  = m_axi_arvalid_cs;
    assign m_axi_araddr                   = m_axi_araddr_cs;
    assign m_axi_arlen                    = m_awlen_in_rd_cs;
    assign m_axi_arsize                   = LEN_LSB;
    assign m_axi_arburst                  = 2'b01;
    assign m_axi_arcache                  = 4'h3;
    assign m_axi_arprot                   = 3'b000;
    assign m_axi_rready                   = m_axi_rready_cs;
    assign m_axi_arlock                   = 1'b0;
    // 状态输出来自读路径
    assign lrdma_qp_mgr_axi_master_error  = dma_error_rd_cs;
    assign lrdma_qp_mgr_axi_master_done   = dma_done_rd_cs;
    assign lrdma_qp_mgr_axi_master_busy   = dma_busy_rd_cs;
    assign lrdma_qp_mgr_axi_master_bvalid = 1'b0;
    assign original_wqe_data_rdy          = 1'b0;

    assign length_fifo_push               = axi_m_en;

    // {仲裁 QP ID, 读长度} 存进 FIFO，以便读数据返回时能正确还原该 WQE 属于哪个 QP，
    // 同时供下面的长度递减计数器使用。
    // 该 FIFO 使用"数据常显"模式（C_READ_LATENCY=0，Fallthrough）：o_q_pop_data 总是预览队头，
    // i_q_pop 仅负责"确认消费并让读指针前进"。
    lrdma_common_fifo #(
        .C_Q_DEPTH     (16),                                  // FIFO Depth
        .C_DATA_SIZE   (IP2BUS_LEN_WIDTH + C_QP_INDX_WIDTH),  // FIFO Data Width
        .C_THRESHOLD   (1),                                   // Threshold value for almost full
        .C_BRAM_N_FLOP (0),                                   // 1- BRAM, 0 - FLOPS
        .C_READ_LATENCY(0)                                    // 0 - Fall-through FIFO, nonzero - non-fall-through FIFO
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
            assign original_wqe_byte_en[i] = (m_axi_rvalid && m_axi_rready_cs) && byte_en_i[i];

            ///////////////////////////////////////////////////////////////////////////////
            // Byte enables for read data during a read or write data during a write
            ///////////////////////////////////////////////////////////////////////////////
            always @(posedge m_axi_aclk) begin
                if (~m_axi_aresetn) begin
                    byte_en_i[i] <= 1'b0;
                end else begin
                    if (axi_m_en && ip2bus_len > i) begin
                        byte_en_i[i] <= 1'b1;
                    end else if (dma_done_rd_cs) begin
                        byte_en_i[i] <= 1'b0;
                    end
                end
            end
        end
    endgenerate
    assign original_wqe_data    = m_axi_rdata;
    assign original_wqe_dvalid  = m_axi_rvalid && m_axi_rready_cs;
    assign original_wqe_arready = m_axi_arready & m_axi_arvalid;
    assign length_count_en      = m_axi_rvalid && m_axi_rready_cs;
    assign length_fifo_pop      = m_axi_rvalid & m_axi_rready_cs;


    ///////////////////////////////////////////////////////////////////////////////
    //Length decrement counter ////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    always @(posedge m_axi_aclk) begin
        if (!m_axi_aresetn) begin
            length_reg <= {IP2BUS_LEN_WIDTH{1'b0}};
        end else begin
            if (read_state_machine_cs == read_idle) begin
                if (~(|length_fifo_dout[LEN_LSB-1 : 0])) begin
                    length_reg[IP2BUS_LEN_WIDTH-1:LEN_LSB] <= length_fifo_dout[IP2BUS_LEN_WIDTH-1:LEN_LSB] - 1'b1;
                    length_reg[LEN_LSB-1:0]                <= length_fifo_dout[LEN_LSB-1:0] - 1'b1;
                end else begin
                    length_reg <= length_fifo_dout;
                end
            end else if (length_count_en && (|length_reg[IP2BUS_LEN_WIDTH-1:LEN_LSB])) begin
                length_reg[IP2BUS_LEN_WIDTH-1:LEN_LSB] <= length_reg[IP2BUS_LEN_WIDTH-1:LEN_LSB] - 1'b1;
                length_reg[LEN_LSB-1:0]                <= {LEN_LSB{1'b0}};
            end else if (length_count_en && |length_reg[LEN_LSB-1:0]) begin
                length_reg[LEN_LSB-1:0] <= {LEN_LSB{1'b0}};
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
        end else if (axi_m_en) begin
            if (~(|ip2bus_len[LEN_LSB-1 : 0])) begin
                wr_rd_load <= ip2bus_len[IP2BUS_LEN_WIDTH-1:LEN_LSB] - 8'h01;
            end else begin
                wr_rd_load <= ip2bus_len[IP2BUS_LEN_WIDTH-1:LEN_LSB];
            end
        end else if ((m_axi_rvalid && m_axi_rready_cs) && (wr_rd_load != 8'h00)) begin
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
            dma_error_rd_cs       <= 1'b0;
            dma_rderr_hold_cs     <= 1'b0;
            m_axi_rready_cs       <= 1'b0;
        end else begin
            case (read_state_machine_cs)
                read_idle: begin
                    dma_done_rd_cs  <= 1'b0;
                    dma_error_rd_cs <= 1'b0;
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
                    if (|length_reg) begin
                        read_state_machine_cs <= wait_read_trans2;
                    end else begin
                        read_state_machine_cs <= read_idle;
                        dma_done_rd_cs        <= 1'b1;
                    end
                end

                wait_read_trans2: begin
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
                    if (~(|length_reg)) begin
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

endmodule
