`timescale 1ps / 1ps `default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_common_arbiter — 高度可配置的多请求仲裁器
//
// 【三种工作模式（通过 generate 选择）】
// 1. NEW_ARB=0（旧式优先级仲裁器）：
//    无有效授权时选最低位请求，arb_done 后选下一个更高位请求并回绕
//    BUS_HOLD 控制单一请求时是否保持授权
//
// 2. NEW_ARB=1, SPLIT=0（单级 lrdma_common_round_robin_arbiter）：
//    使用 lrdma_common_round_robin_arbiter 子模块实现标准轮转，时序更优
//
// 3. NEW_ARB=1, SPLIT=1（两级分层 lrdma_common_round_robin_arbiter）：
//    NUM_REQS 个请求按 NUM_SPLITS 组分配到多个一级 lrdma_common_round_robin_arbiter，
//    组授权作为二级 lrdma_common_round_robin_arbiter 输入，最终组合产生 one-hot 授权
//    关键路径从 O(N) 降到 O(sqrt(N))
//
// 【输出】
// gnt：one-hot 授权向量
// gnt_valid：存在有效授权
///////////////////////////////////////////////////////////////////////////////
module lrdma_common_arbiter #(
    parameter NUM_REQS   = 4,
    parameter BUS_HOLD   = 0,
    parameter NEW_ARB    = 0,
    parameter SPLIT      = 0,
    parameter NUM_SPLITS = 4
) (
    input clk_arb,
    input rst_n_arb,

    input [NUM_REQS - 1:0] reqs,
    input                  arb_done,

    output logic [NUM_REQS - 1:0] gnt,
    output logic                  gnt_valid
);

    //==========================================================================
    // genvars
    //==========================================================================
    genvar i, j;

    //==========================================================================
    // local parameters
    //==========================================================================
    localparam REQ_ID = (NUM_REQS == 1) ? 1 : $clog2(NUM_REQS);
    //localparam REQ_PER_ARB = (SPLIT == 1) ? NUM_REQS/NUM_SPLITS : 1;
    localparam REQ_PER_ARB = (SPLIT == 1) ? (NUM_REQS % NUM_SPLITS != 0) ? (NUM_REQS / NUM_SPLITS) + 1 : (NUM_REQS / NUM_SPLITS) : 1;
    localparam SPLIT_GNTS = REQ_PER_ARB * NUM_SPLITS;

    //==========================================================================
    // wires and register declarations
    //==========================================================================

    logic [    REQ_ID - 1:0] curr_gnt_bit;
    logic [    REQ_ID - 1:0] max_req_bit;
    logic [  NUM_REQS - 1:0] next_reqs_curr_lss_max;
    logic [  NUM_REQS - 1:0] next_gnt_curr_lss_max;
    logic [SPLIT_GNTS - 1:0] int_gnt;

    generate
        if (NEW_ARB == 0) begin : old_arbiter

            //==========================================================================
            // 授权位索引
            //==========================================================================
            assign curr_gnt_bit           = (|gnt) ? setbit_lsb(gnt) : {REQ_ID{1'b0}};

            //==========================================================================
            // MSB which is set in request
            //==========================================================================
            assign max_req_bit            = (|reqs) ? setbit_msb(reqs) : {REQ_ID{1'b0}};

            //==========================================================================
            // 当更高位还有待处理请求时，授权给下一个更高位请求
            //==========================================================================
            assign next_reqs_curr_lss_max = {NUM_REQS{1'b1}} << (curr_gnt_bit + 1'b1);
            assign next_gnt_curr_lss_max  = (|(next_reqs_curr_lss_max & reqs)) ? setbit_lsb(next_reqs_curr_lss_max & reqs) : {REQ_ID{1'b0}};

            always_ff @(posedge clk_arb) begin
                if (!rst_n_arb) begin
                    gnt <= {NUM_REQS{1'b0}};
                end  //==========================================================================
                     // 无有效授权时，选择最低位请求并授权
                     //==========================================================================
                else if ((gnt == {NUM_REQS{1'b0}}) | !reqs[curr_gnt_bit]) begin
                    for (int i = 0; i < NUM_REQS; i++) begin
                        gnt[i] <= (i == setbit_lsb(reqs)) & reqs[i];
                    end
                end else if (arb_done) begin
                    //==========================================================================
                    // 如果已授权请求是唯一请求：
                    // BUS_HOLD == 0 : next grant = 0
                    // BUS_HOLD == 1 : next grant = 保持该授权
                    //==========================================================================
                    if (gnt == reqs) begin
                        gnt <= (BUS_HOLD == 1) ? gnt : {NUM_REQS{1'b0}};
                    end  //==========================================================================
                         // 如果更高位有待处理请求，则把下一个为 1 的位作为授权
                         //==========================================================================
                    else if (max_req_bit > curr_gnt_bit) begin
                        for (int i = 0; i < NUM_REQS; i++) begin
                            gnt[i] <= (i == next_gnt_curr_lss_max);
                        end
                    end  //==========================================================================
                         // 如果更高位没有请求，则回到最低位请求并授权
                         //==========================================================================
                    else begin
                        for (int i = 0; i < NUM_REQS; i++) begin
                            gnt[i] <= (i == setbit_lsb(reqs)) & reqs[i];
                        end
                    end
                end
            end

            //==========================================================================
            // if at least one grant bit is set then only it's valid
            //==========================================================================
            //generate 
            if (BUS_HOLD == 0) begin : no_bus_hold
                assign gnt_valid = |gnt;
            end else begin : en_bus_hold
                assign gnt_valid = |((gnt) & (reqs));
            end
            //endgenerate

            //==========================================================================
            // function to identify the request number
            //==========================================================================
            function [REQ_ID - 1:0] setbit_lsb(input [NUM_REQS - 1:0] in_reqs);
                begin : setbit_lsb_idfy_blk
                    for (int i = 0; i < NUM_REQS; i++) begin : calc_block
                        if (in_reqs[i]) begin
                            setbit_lsb = i;
                            disable setbit_lsb_idfy_blk;
                        end
                    end
                end
            endfunction  // if

            //==========================================================================
            // function to identify the request number
            //==========================================================================
            function [REQ_ID - 1:0] setbit_msb(input [NUM_REQS - 1:0] in_reqs);
                begin : setbit_msb_idfy_blk
                    for (int i = NUM_REQS - 1; i >= 0; i--) begin : calc_block
                        if (in_reqs[i]) begin
                            setbit_msb = i;
                            disable setbit_msb_idfy_blk;
                        end
                    end
                end
            endfunction  // if
        end else if (SPLIT == 0) begin : timing_arbiter
            lrdma_common_round_robin_arbiter #(
                .NUM_REQ      (NUM_REQS),
                .BUS_PARK_CTRL(BUS_HOLD)
            ) u_arbiter (
                .clk_arb        (clk_arb),
                .rst_n_arb      (rst_n_arb),
                .no_req         (1'b0),
                .wround_robin_en(1'b0),
                .round_robin_en (1'b1),
                .req            (reqs),
                .done           ({NUM_REQS{arb_done}}),
                .gnt            (gnt)
            );

            assign gnt_valid = (BUS_HOLD == 1) ? ((|gnt) & (|reqs)) : |gnt;


        end else
        //=====================================================================
        // divides the requests into multiple arbiters depending on NUM_SPLITS
        // to have a better timing
        //=====================================================================
        begin : timing_split_arbiter
            logic [REQ_PER_ARB - 1:0] reqs_for_each_arb[NUM_SPLITS];
            logic [ NUM_SPLITS - 1:0] gnt_valid_l0;
            logic [REQ_PER_ARB - 1:0] gnt_l0           [NUM_SPLITS];
            logic [ NUM_SPLITS - 1:0] arb_done_l0;
            logic [ NUM_SPLITS - 1:0] gnt_l1;

            for (i = 0; i < NUM_SPLITS; i++) begin : split_req
                for (j = 0; j < REQ_PER_ARB; j++) begin : num_reqs_pe_arbiter
                    if (((i * REQ_PER_ARB) + j) >= NUM_REQS) begin
                        assign reqs_for_each_arb[i][j] = 1'b0;
                    end else begin
                        assign reqs_for_each_arb[i][j] = reqs[(i*REQ_PER_ARB)+j];
                    end
                end
            end

            for (i = 0; i < NUM_SPLITS; i++) begin : level0_arbiter


                lrdma_common_round_robin_arbiter #(
                    .NUM_REQ      (REQ_PER_ARB),
                    .BUS_PARK_CTRL(BUS_HOLD)
                ) u_arbiter_l0 (
                    .clk_arb        (clk_arb),
                    .rst_n_arb      (rst_n_arb),
                    .no_req         (1'b0),
                    .wround_robin_en(1'b0),
                    .round_robin_en (1'b1),
                    .req            (reqs_for_each_arb[i]),
                    .done           ({REQ_PER_ARB{arb_done_l0[i]}}),  // CK
                    .gnt            (gnt_l0[i])
                );

                assign gnt_valid_l0[i] = (BUS_HOLD == 1) ? ((|gnt_l0[i]) & (|reqs_for_each_arb[i])) : |gnt_l0[i];

                //=============================================================
                // if grant of level 1 lrdma_common_arbiter is for this level 0 lrdma_common_arbiter and
                // a done is received then this 
                //=============================================================
                assign arb_done_l0[i]  = gnt_l1[i] & arb_done;
            end

            //==================================================================
            // final lrdma_common_arbiter at level 1
            //==================================================================
            lrdma_common_round_robin_arbiter #(
                .NUM_REQ      (NUM_SPLITS),
                .BUS_PARK_CTRL(BUS_HOLD)
            ) u_arbiter_l1 (
                .clk_arb        (clk_arb),
                .rst_n_arb      (rst_n_arb),
                .no_req         (1'b0),
                .wround_robin_en(1'b0),
                .round_robin_en (1'b1),
                .req            (gnt_valid_l0),
                .done           ({NUM_SPLITS{arb_done}}),
                .gnt            (gnt_l1)
            );

            assign gnt_valid = (BUS_HOLD == 1) ? ((|gnt_l1) & (|gnt_valid_l0)) : |gnt_l1;
            for (i = 0; i < NUM_SPLITS; i++) begin : final_gnt
                assign int_gnt[(i*REQ_PER_ARB)+:REQ_PER_ARB] = gnt_l1[i] ? gnt_l0[i] : 'h0;
            end
            assign gnt = int_gnt[NUM_REQS-1:0];

        end
    endgenerate


endmodule  // lrdma_common_arbiter
