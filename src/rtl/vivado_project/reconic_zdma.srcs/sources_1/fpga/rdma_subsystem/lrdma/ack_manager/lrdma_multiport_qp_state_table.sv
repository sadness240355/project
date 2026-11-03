`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_multiport_qp_state_table — 多端口 per-QP 状态表封装
//
// 【模块功能】
// 该模块把一组单端写、单端读的 per-QP 状态 RAM 封装成“多写请求、多读请求”的
// 仲裁式状态表。ACK 管理器用它保存 ACK/NAK PSN 状态和 ACK/NAK MSN 状态。
// 模块只负责按 qpid 存取数据，不解释数据内容，也不参与 ACK/NAK 判断。
//
// 【写侧】
// - 支持 NUM_WR_REQS 个写请求源；
// - 每个写请求源可以选择是否经过输入 FIFO；
// - 写仲裁器每次选择一个非空请求，并把 qpid/data 写入 RAM A 端；
// - 写请求经过一级寄存器，保证 RAM 写入时序清晰。
//
// 【读侧】
// - 支持 NUM_RD_REQS 个读请求源；
// - 每个读请求源可以选择是否经过输入 FIFO；
// - 读仲裁器选择一个请求后读取 RAM B 端；
// - 如果同一拍存在写同一 qpid 的操作，读请求会被保持，避免返回旧数据；
// - 读数据和 rd_req_done 通过流水线对齐后返回给对应请求源。
//
// 【存储器】
// 底层使用 xpm_memory_sdpram，A 端写，B 端读。DATA_WIDTH 和 C_NUM_QP 由实例参数决定。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps
module lrdma_multiport_qp_state_table #(
    parameter DATA_WIDTH      = 24,
    parameter NUM_WR_REQS     = 3,
    parameter WR_FIFO_EN      = 1,
    parameter RD_FIFO_EN      = 1,
    parameter NUM_RD_REQS     = 3,
    parameter C_NUM_QP        = 256,
    parameter C_QP_INDX_WIDTH = $clog2(C_NUM_QP)
) (
    // 时钟和复位
    input core_clk,
    input core_rstn,

    // 写路径
    input        [    NUM_WR_REQS - 1:0] wr_reqs,
    input        [C_QP_INDX_WIDTH - 1:0] wr_req_qpid[NUM_WR_REQS],
    input        [     DATA_WIDTH - 1:0] wr_req_data[NUM_WR_REQS],
    output logic [    NUM_WR_REQS - 1:0] wr_req_done,

    // 读路径
    input        [    NUM_RD_REQS - 1:0] rd_reqs,
    input        [C_QP_INDX_WIDTH - 1:0] rd_req_qpid[NUM_RD_REQS],
    output logic [     DATA_WIDTH - 1:0] rd_req_data[NUM_RD_REQS],
    output logic [    NUM_RD_REQS - 1:0] rd_req_done
);

    `include "lrdma_defs.vh"

    //==========================================================================
    // local parameters
    //==========================================================================
    localparam LOG2_NUM_WR_REQS = $clog2(NUM_WR_REQS);
    localparam LOG2_NUM_RD_REQS = $clog2(NUM_RD_REQS);
    localparam WR_FIFO_WIDTH = DATA_WIDTH + C_QP_INDX_WIDTH;
    localparam RD_FIFO_WIDTH = C_QP_INDX_WIDTH;

    //==========================================================================
    // wires and registers
    //==========================================================================
    logic [     NUM_WR_REQS - 1:0] wrreq_fifo_empty;
    logic [     NUM_WR_REQS - 1:0] wrreq_fifo_rden;
    logic [      DATA_WIDTH - 1:0] wrreq_fifo_dout_data[NUM_WR_REQS];
    logic [ C_QP_INDX_WIDTH - 1:0] wrreq_fifo_dout_qpid[NUM_WR_REQS];
    logic [     NUM_RD_REQS - 1:0] rdreq_fifo_empty;
    logic [     NUM_RD_REQS - 1:0] rdreq_fifo_rden;
    logic [ C_QP_INDX_WIDTH - 1:0] rdreq_fifo_dout_qpid[NUM_RD_REQS];
    logic [     NUM_WR_REQS - 1:0] wr_req_arb_reqs;
    logic [     NUM_WR_REQS - 1:0] wr_req_arb_gnt;
    logic                          wr_req_arb_done;
    logic [LOG2_NUM_WR_REQS - 1:0] wr_req_num;
    logic [     NUM_RD_REQS - 1:0] rd_req_arb_reqs;
    logic [     NUM_RD_REQS - 1:0] rd_req_arb_gnt;
    logic [     NUM_RD_REQS - 1:0] rd_req_arb_gnt_q;
    logic [     NUM_RD_REQS - 1:0] rd_req_arb_gnt_2q;
    logic                          rd_req_arb_done;
    logic [LOG2_NUM_RD_REQS - 1:0] rd_req_num;
    logic                          mem_wren_nxt;
    logic [ C_QP_INDX_WIDTH - 1:0] mem_wraddr_nxt;
    logic [      DATA_WIDTH - 1:0] mem_wrdata_nxt;
    logic                          mem_wren;
    logic [ C_QP_INDX_WIDTH - 1:0] mem_wraddr;
    logic [      DATA_WIDTH - 1:0] mem_wrdata;
    logic                          mem_rden_nxt;
    logic [ C_QP_INDX_WIDTH - 1:0] mem_rdaddr_nxt;
    logic [      DATA_WIDTH - 1:0] mem_rddata;
    logic                          mem_rden;
    logic [ C_QP_INDX_WIDTH - 1:0] mem_rdaddr;
    logic                          mem_rddata_valid;

    /***************************************************************************
   *                                                                          *
   *                                                                          *
   *                  Write arbitration and memory write logic                *
   *                                                                          *
   *                                                                          *
   ***************************************************************************/

    generate
        if (NUM_WR_REQS > 1) begin : wr_arb_inst
            if (WR_FIFO_EN == 1) begin : inputs_in_fifo
                for (genvar i = 0; i < NUM_WR_REQS; i++) begin : wrreq_fifo_insts
                    xpm_fifo_sync #(
                        .DOUT_RESET_VALUE   ("0"),
                        .ECC_MODE           ("no_ecc"),
                        .FIFO_MEMORY_TYPE   ("auto"),
                        .FIFO_READ_LATENCY  (1),
                        .FIFO_WRITE_DEPTH   (16),
                        .FULL_RESET_VALUE   (0),
                        .PROG_EMPTY_THRESH  (10),
                        .PROG_FULL_THRESH   (10),
                        .RD_DATA_COUNT_WIDTH(1),
                        .READ_DATA_WIDTH    (WR_FIFO_WIDTH),
                        .READ_MODE          ("fwft"),
                        .SIM_ASSERT_CHK     (0),
                        .USE_ADV_FEATURES   ("0707"),
                        .WAKEUP_TIME        (0),
                        .WRITE_DATA_WIDTH   (WR_FIFO_WIDTH),
                        .WR_DATA_COUNT_WIDTH(1)
                    ) u_in_wrreq_fifo (
                        .wr_clk       (core_clk),
                        .rst          (!core_rstn),
                        .wr_en        (wr_reqs[i]),
                        .din          ({wr_req_data[i], wr_req_qpid[i]}),
                        .rd_en        (wrreq_fifo_rden[i]),
                        .empty        (wrreq_fifo_empty[i]),
                        .dout         ({wrreq_fifo_dout_data[i], wrreq_fifo_dout_qpid[i]}),
                        .almost_empty (),
                        .almost_full  (),
                        .data_valid   (),
                        .dbiterr      (),
                        .full         (),
                        .overflow     (),
                        .prog_empty   (),
                        .prog_full    (),
                        .rd_data_count(),
                        .rd_rst_busy  (),
                        .sbiterr      (),
                        .underflow    (),
                        .wr_ack       (),
                        .wr_data_count(),
                        .wr_rst_busy  (),
                        .injectdbiterr(1'b0),
                        .injectsbiterr(1'b0),
                        .sleep        (1'b0)
                    );

                    assign wr_req_arb_reqs[i] = !wrreq_fifo_empty[i];
                    assign wrreq_fifo_rden[i] = !wrreq_fifo_empty[i] & wr_req_arb_gnt[i];

                end
            end // inputs_in_fifo
        else
	    begin : inputs_direct
                assign wr_req_arb_reqs = wr_reqs;
                for (genvar i = 0; i < NUM_WR_REQS; i++) begin : direct_data_qpid
                    assign wrreq_fifo_dout_qpid[i] = wr_req_qpid[i];
                    assign wrreq_fifo_dout_data[i] = wr_req_data[i];
                end
            end  // inputs_direct

            // 仲裁器实例在各种请求之间进行仲裁
            lrdma_common_arbiter #(
                .NUM_REQS(NUM_WR_REQS),
                .BUS_HOLD(0),
                .NEW_ARB (1),
                .SPLIT   (0)
            ) u_wr_req_arb (
                .clk_arb  (core_clk),
                .rst_n_arb(core_rstn),
                .reqs     (wr_req_arb_reqs),
                .arb_done (wr_req_arb_done),
                .gnt      (wr_req_arb_gnt),
                .gnt_valid()
            );

            //==========================================================================
            // 根据写仲裁 grant 找到被选中的写请求编号
            //==========================================================================
            function [LOG2_NUM_WR_REQS - 1:0] wr_gnt_num_idfy(input [NUM_WR_REQS - 1:0] grant_bit);
                begin : wr_gnt_num_idfy_blk
                    for (int i = 0; i < NUM_WR_REQS; i++) begin : calc_block
                        if (grant_bit[i]) begin
                            wr_gnt_num_idfy = i;
                            disable wr_gnt_num_idfy_blk;
                        end
                    end
                end
            endfunction  // wr_gnt_num_idfy

            // 根据被选中的写请求编号选择写地址和写数据
            assign wr_req_num     = wr_gnt_num_idfy(wr_req_arb_gnt);
            assign mem_wraddr_nxt = wrreq_fifo_dout_qpid[wr_req_num];
            assign mem_wrdata_nxt = wrreq_fifo_dout_data[wr_req_num];
            assign mem_wren_nxt   = |wr_req_arb_gnt;

            // 写地址、写数据和写使能打一拍，保证 RAM 写入时序清晰
            `MSFF_RN(mem_wraddr, mem_wraddr_nxt, core_clk, core_rstn)
            `MSFF_RN(mem_wrdata, mem_wrdata_nxt, core_clk, core_rstn)
            `MSFF_RN(mem_wren, mem_wren_nxt, core_clk, core_rstn)

            // 写侧每拍都允许重新仲裁
            assign wr_req_arb_done = 1'b1;
            assign wr_req_done     = wr_req_arb_gnt;
        end else begin : no_wr_arb
            assign mem_wraddr     = wr_req_qpid[0];
            assign mem_wrdata     = wr_req_data[0];
            assign mem_wren       = wr_reqs[0];
            assign wr_req_done[0] = mem_wren;
        end
    endgenerate

    /***************************************************************************
   *                                                                          *
   *                                                                          *
   *                  Read arbitration and memory read logic                  *
   *                                                                          *
   *                                                                          *
   ***************************************************************************/

    generate
        if (NUM_RD_REQS > 1) begin : rd_arb_inst
            if (RD_FIFO_EN == 1) begin : rd_inputs_in_fifo
                for (genvar i = 0; i < NUM_RD_REQS; i++) begin : wrreq_fifo_insts
                    xpm_fifo_sync #(
                        .DOUT_RESET_VALUE   ("0"),
                        .ECC_MODE           ("no_ecc"),
                        .FIFO_MEMORY_TYPE   ("auto"),
                        .FIFO_READ_LATENCY  (1),
                        .FIFO_WRITE_DEPTH   (16),
                        .FULL_RESET_VALUE   (0),
                        .PROG_EMPTY_THRESH  (10),
                        .PROG_FULL_THRESH   (10),
                        .RD_DATA_COUNT_WIDTH(1),
                        .READ_DATA_WIDTH    (RD_FIFO_WIDTH),
                        .READ_MODE          ("fwft"),
                        .SIM_ASSERT_CHK     (0),
                        .USE_ADV_FEATURES   ("0707"),
                        .WAKEUP_TIME        (0),
                        .WRITE_DATA_WIDTH   (RD_FIFO_WIDTH),
                        .WR_DATA_COUNT_WIDTH(1)
                    ) u_in_rdreq_fifo (
                        .wr_clk       (core_clk),
                        .rst          (!core_rstn),
                        .wr_en        (rd_reqs[i]),
                        .din          (rd_req_qpid[i]),
                        .rd_en        (rdreq_fifo_rden[i]),
                        .empty        (rdreq_fifo_empty[i]),
                        .dout         (rdreq_fifo_dout_qpid[i]),
                        .almost_empty (),
                        .almost_full  (),
                        .data_valid   (),
                        .dbiterr      (),
                        .full         (),
                        .overflow     (),
                        .prog_empty   (),
                        .prog_full    (),
                        .rd_data_count(),
                        .rd_rst_busy  (),
                        .sbiterr      (),
                        .underflow    (),
                        .wr_ack       (),
                        .wr_data_count(),
                        .wr_rst_busy  (),
                        .injectdbiterr(1'b0),
                        .injectsbiterr(1'b0),
                        .sleep        (1'b0)
                    );

                    assign rd_req_arb_reqs[i] = !rdreq_fifo_empty[i];
                    assign rdreq_fifo_rden[i] = !rdreq_fifo_empty[i] & rd_req_arb_gnt[i] & rd_req_arb_done;

                end
            end // inputs_in_fifo
	   else
	     begin : rd_inputs_direct
                assign rd_req_arb_reqs = rd_reqs;
                for (genvar i = 0; i < NUM_RD_REQS; i++) begin : rd_direct_data_qpid
                    assign rdreq_fifo_dout_qpid[i] = rd_req_qpid[i];
                end
            end  // rd_inputs_direct

            // 读仲裁器实例，在多个读请求之间选择一个访问 RAM
            lrdma_common_arbiter #(
                .NUM_REQS(NUM_RD_REQS),
                .BUS_HOLD(0),
                .NEW_ARB (1),
                .SPLIT   (0)
            ) u_rd_req_arb (
                .clk_arb  (core_clk),
                .rst_n_arb(core_rstn),
                .reqs     (rd_req_arb_reqs),
                .arb_done (rd_req_arb_done),
                .gnt      (rd_req_arb_gnt),
                .gnt_valid()
            );
            //==========================================================================
            // 根据读仲裁 grant 找到被选中的读请求编号
            //==========================================================================
            function [LOG2_NUM_RD_REQS - 1:0] rd_gnt_num_idfy(input [NUM_RD_REQS - 1:0] grant_bit);
                begin : rd_gnt_num_idfy_blk
                    for (int i = 0; i < NUM_RD_REQS; i++) begin : calc_block
                        if (grant_bit[i]) begin
                            rd_gnt_num_idfy = i;
                            disable rd_gnt_num_idfy_blk;
                        end
                    end
                end
            endfunction  // rd_gnt_num_idfy

            // 根据被选中的读请求编号选择读地址；若同拍写同一 qpid，则暂停本次读，等待写入先完成
            assign rd_req_num     = rd_gnt_num_idfy(rd_req_arb_gnt);
            assign mem_rdaddr_nxt = rdreq_fifo_dout_qpid[rd_req_num];
            assign mem_rden_nxt   = (mem_wren_nxt & (mem_wraddr_nxt == mem_rdaddr_nxt)) ? 1'b0 : |rd_req_arb_gnt;

            // 读地址和读使能打一拍，mem_rddata_valid 再与延迟后的 grant 对齐
            `MSFF_RN(mem_rdaddr, mem_rdaddr_nxt, core_clk, core_rstn)
            `MSFF_RN(mem_rden, mem_rden_nxt, core_clk, core_rstn)
            `MSFF_RN(mem_rddata_valid, mem_rden, core_clk, core_rstn)

            assign rd_req_arb_done = !(mem_wren_nxt & (mem_wraddr_nxt == mem_rdaddr_nxt));
            //`MSFF_RN (rd_req_done, ({NUM_RD_REQS {mem_rddata_valid}} & rd_req_arb_gnt_q), core_clk, core_rstn);
            assign rd_req_done     = ({NUM_RD_REQS{mem_rddata_valid}} & rd_req_arb_gnt_2q);

            // grant 延迟两拍，与 RAM 读数据返回时序对齐
            `MSFF_RN(rd_req_arb_gnt_q, rd_req_arb_gnt, core_clk, core_rstn)
            `MSFF_RN(rd_req_arb_gnt_2q, rd_req_arb_gnt_q, core_clk, core_rstn)

            for (genvar i = 0; i < NUM_RD_REQS; i++) begin : rd_data_gen
                // 只有两拍前被 grant 的读端口返回 mem_rddata，其它读端口输出 0。
                assign rd_req_data[i] = rd_req_arb_gnt_2q[i] ? mem_rddata : {DATA_WIDTH{1'b0}};
            end  // rd_data_gen
        end else begin : no_rd_arb
            assign mem_rdaddr = rd_req_qpid[0];
            assign mem_rden   = (mem_wren & (mem_wraddr == mem_rdaddr)) ? 1'b0 : rd_reqs[0];
            `MSFF_RN(rd_req_done[0], mem_rden, core_clk, core_rstn)
            assign rd_req_data[0] = mem_rddata;
        end
    endgenerate

    /***************************************************************************
   *                                                                          *
   *                                                                          *
   *                  Memory instantiation to store data                      *
   *                                                                          *
   *                                                                          *
   ***************************************************************************/

    xpm_memory_sdpram #(  // "A" side parameters
        .ADDR_WIDTH_A           (C_QP_INDX_WIDTH),
        .RST_MODE_A             ("SYNC"),
        .BYTE_WRITE_WIDTH_A     (DATA_WIDTH),
        .WRITE_DATA_WIDTH_A     (DATA_WIDTH),
        // "B" side parameters
        .ADDR_WIDTH_B           (C_QP_INDX_WIDTH),
        .RST_MODE_B             ("SYNC"),
        .READ_RESET_VALUE_B     ("0"),
        .READ_LATENCY_B         (1),
        .READ_DATA_WIDTH_B      (DATA_WIDTH),
        .WRITE_MODE_B           ("read_first"),
        // Common parameters
        .CLOCKING_MODE          ("common_clock"),
        .CASCADE_HEIGHT         (0),
        .ECC_MODE               ("no_ecc"),
        .MEMORY_INIT_FILE       ("none"),
        .MEMORY_INIT_PARAM      ("0"),
        .MEMORY_OPTIMIZATION    ("true"),
        .MEMORY_PRIMITIVE       ("auto"),
        .MEMORY_SIZE            (C_NUM_QP * DATA_WIDTH),
        .MESSAGE_CONTROL        (0),
        .AUTO_SLEEP_TIME        (0),
        .SIM_ASSERT_CHK         (0),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT           (1),
        .WAKEUP_TIME            ("disable_sleep")
    ) u_data_memory (
        .clka          (core_clk),
        .ena           (mem_wren),
        .wea           (mem_wren),
        .addra         (mem_wraddr),
        .dina          (mem_wrdata),
        .injectdbiterra(1'b0),
        .injectsbiterra(1'b0),
        .clkb          (core_clk),
        .rstb          (!core_rstn),
        .enb           (mem_rden),
        .addrb         (mem_rdaddr),
        .regceb        (1'b1),
        .doutb         (mem_rddata),
        .sleep         (1'b0),
        .dbiterrb      (),
        .sbiterrb      ()
    );

endmodule  // lrdma_multiport_qp_state_table
