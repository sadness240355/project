`default_nettype wire `timescale 1ps / 1ps
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_common_round_robin_arbiter — 标准轮转（Round-Robin）仲裁算法核心
//
// 【模块概述】
// lrdma_common_arbiter 模块在 NEW_ARB=1 模式下的核心子模块。
// 基于旋转优先级指针(rotate_ptr)实现公平仲裁。
//
// 【算法】
// 1. req_msk = req & rotate_ptr（屏蔽后请求）
// 2. gnt_msk = 取 req_msk 的最低有效位
// 3. 若 no_mask_req，回退到 gnt_nomsk = 取原始 req 最低位
// 4. gnt_nxt = gnt_msk | (gnt_nomsk & {N{no_mask_req}})
// 5. 完成后 rotate_ptr 旋转到本次授权的下一位
//
// 【BUS_PARK_CTRL】
// 无新请求时决定是否维持最后授权（总线停靠）
///////////////////////////////////////////////////////////////////////////////
module lrdma_common_round_robin_arbiter #(
    parameter NUM_REQ       = 4,
    parameter BUS_PARK_CTRL = 0
) (
    input                        clk_arb,
    input                        rst_n_arb,
    input                        no_req,
    input                        wround_robin_en,
    input                        round_robin_en,
    input        [NUM_REQ - 1:0] req,
    input        [NUM_REQ - 1:0] done,
    output logic [NUM_REQ - 1:0] gnt
);
    //==========================================================================
    // local parameters
    //==========================================================================
    localparam LOG2_NUM_REQ = (NUM_REQ == 1) ? 1 : $clog2(NUM_REQ);
    localparam TOTAL_LOG2_REQ = NUM_REQ * LOG2_NUM_REQ;

    //==========================================================================
    // genvars
    //==========================================================================
    genvar i;

    //==========================================================================
    // register and wire declarations
    //==========================================================================
    logic [NUM_REQ -1:0] gnt_msk;
    logic [NUM_REQ -1:0] gnt_nomsk;
    logic [NUM_REQ -1:0] rotate_ptr;
    logic [NUM_REQ -1:0] rotate_ptr_nxt;
    logic [NUM_REQ -1:0] gnt_zero;
    logic [NUM_REQ -1:0] req_int;
    logic [NUM_REQ -1:0] req_msk;
    logic                no_mask_req;
    logic [NUM_REQ -1:0] gnt_nxt;
    logic                arb_start;
    logic                update_ptr;


    assign gnt_zero = (((~(done & gnt)) & req) == {NUM_REQ{1'b0}}) ? {NUM_REQ{1'b1}} : {NUM_REQ{1'b0}};
    assign req_int  = BUS_PARK_CTRL ? (((~(done & gnt)) & req) | (gnt_zero & req)) : ((~(done & gnt)) & req);

    // mask grant generation logic
    assign req_msk  = req_int & rotate_ptr_nxt;

    // Grant for masked request
    generate  //{
        assign gnt_msk[0]   = req_msk[0];
        assign gnt_nomsk[0] = req_int[0];
        for (i = 1; i < NUM_REQ; i++) begin : gnt_gen  //{
            assign gnt_msk[i]   = (~|req_msk[i-1:0]) & req_msk[i];
            assign gnt_nomsk[i] = (~|req_int[i-1:0]) & req_int[i];
        end  //}
    endgenerate  //}

    //grant generation (combined mask + non mask grants)
    assign no_mask_req = ~|req_msk;
    assign gnt_nxt     = gnt_msk | (gnt_nomsk & {NUM_REQ{no_mask_req}});

    always_ff @(posedge clk_arb) begin  //{
        if (!rst_n_arb) begin  //{
            arb_start <= 1'b1;
        end//}
	else if(no_req|(~|req_int))
	  begin//{
            arb_start <= 1'b1;
        end//}
	else if (|req_int)
	  begin//{
            arb_start <= 1'b0;
        end  //}
    end  //}


    // rotate pointer update logic
    assign update_ptr = (round_robin_en & (|(gnt & done)));

    always_ff @(posedge clk_arb) begin  //{
        if (!rst_n_arb) begin  //{
            rotate_ptr <= {NUM_REQ{1'b1}};
        end//}
	else
	  begin//{
            rotate_ptr <= rotate_ptr_nxt;
        end  //}
    end  //}

    always_comb begin  //{
        if (update_ptr) begin  //{
                               // note: N must be at least 2
            rotate_ptr_nxt[0] = gnt[NUM_REQ-1];
            if (NUM_REQ > 1'b1) begin  //{
                rotate_ptr_nxt[1] = gnt[NUM_REQ-1] | gnt[0];
            end  //}
        end//}
	else if(wround_robin_en & no_req)
	  begin//{
            rotate_ptr_nxt[0] = 1'b1;
            if (NUM_REQ > 1'b1) begin  //{
                rotate_ptr_nxt[1] = 1'b1;
            end  //}
        end//}
	else
	  begin//{
            rotate_ptr_nxt[0] = !round_robin_en ? 1'b1 : rotate_ptr[0];
            if (NUM_REQ > 1'b1) begin  //{
                rotate_ptr_nxt[1] = !round_robin_en ? 1'b1 : rotate_ptr[1];
            end  //}
        end  //}
    end  //}

    //registered the Grant 
    always_ff @(posedge clk_arb) begin  //{
        if (!rst_n_arb) begin  //{
            gnt <= {NUM_REQ{1'b0}};
        end//}
	else if(update_ptr | arb_start | (~round_robin_en & (|((gnt&done)))) | (|(~req&gnt)))
	  begin//{
            gnt <= gnt_nxt;
        end//}
	else
	  begin//{
            gnt <= gnt & req;
        end  //}
    end  //}

    generate  //{
        for (i = 2; i < NUM_REQ; i = i + 1) begin  //{
            always_comb begin  //{
                if (update_ptr) begin  //{
                    rotate_ptr_nxt[i] = gnt[NUM_REQ-1] | (|gnt[i-1:0]);
                end//}
		else if(wround_robin_en & no_req)
		  begin//{
                    rotate_ptr_nxt[i] = 1'b1;
                end//}
		else
		  begin//{
                    rotate_ptr_nxt[i] = !round_robin_en ? 1'b1 : rotate_ptr[i];
                end  //}
            end  //}
        end  //}
    endgenerate  //}
    //wire [NUM_REQ-1:0] weight_cnt_en = wround_robin_en ? (gnt&done) : {NUM_REQ{1'b0}};

endmodule  // lrdma_common_round_robin_arbiter
