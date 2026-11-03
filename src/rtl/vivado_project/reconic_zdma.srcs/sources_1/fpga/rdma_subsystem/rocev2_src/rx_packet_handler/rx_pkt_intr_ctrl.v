`timescale 1ps / 1ps
module rx_pkt_intr_ctrl #(
    parameter C_NUM_QP        = 256,
    parameter C_QP_INDX_WIDTH = 8
) (
    // System level inputs & outputs
    input wire clk,
    input wire rst_n,

    input wire xrnic_enabled,

    input  wire pkt_valdn_err_intr_en,
    output wire pkt_valdn_err_intr,
    output reg  pkt_valdn_err_intr_sts,
    input  wire clr_pkt_valdn_err_intr,

    input  wire mad_pkt_rcvd_intr_en,
    output wire mad_pkt_rcvd_intr,
    output reg  mad_pkt_rcvd_intr_sts,
    input  wire clr_mad_pkt_rcvd_intr,

    input  wire bypass_pkt_rcvd_intr_en,
    output wire bypass_pkt_rcvd_intr,
    output reg  bypass_pkt_rcvd_intr_sts,
    input  wire clr_bypass_pkt_rcvd_intr,

    // input  wire                       in_errsts_intr_en,       
    // output wire                       in_errsts_intr,          
    // output reg                        in_errsts_intr_sts,      
    // input  wire                       clr_in_errsts_intr,

    input  wire rnr_nack_gen_intr_en,   // connect to o_intr_en_rnr_nak_gen
    output wire rnr_nack_gen_intr,      // 
    output reg  rnr_nack_gen_intr_sts,  // connect to i_rnr_nack_gen_intr
    input  wire clr_rnr_nack_gen_intr,  // connect to o_intr_clr_rnr_nak_gen

    input  wire fatal_err_intr_en,   // connect to qp_mgr_o_intr_en_fatal_err
    output wire fatal_err_intr,      // connect to rx_pkt_hndler_o_fatal_err_intr
    output reg  fatal_err_intr_sts,  // 
    input  wire clr_fatal_err_intr,  // connect to qp_mgr_o_qp_clr_fatal_err  

    input  wire                       qp_pkt_rcvd_intr_en,
    output wire                       qp_pkt_rcvd_intr,
    //output reg  [C_NUM_QP-1:0]        qp_pkt_rcvd_intr_sts,    
    output reg  [               31:0] qp_pkt_rcvd_intr_sts,
    output reg  [               31:0] qp_pkt_rcvd_intr_sts_wr_en,
    output reg  [C_QP_INDX_WIDTH-1:0] qp_pkt_rcvd_intr_sts_wr_addr,
    //input  wire [C_NUM_QP-1:0]        clr_qp_pkt_rcvd_intr,    
    input  wire [               31:0] clr_qp_pkt_rcvd_intr,
    input  wire [C_QP_INDX_WIDTH-1:0] clr_qp_pkt_rcvd_intr_wr_addr,

    input wire set_pkt_valdn_err_intr,
    input wire set_mad_pkt_rcvd_intr,
    input wire set_bypass_pkt_rcvd_intr,
    //input  wire                       set_in_errsts_intr,
    input wire set_rnr_nack_gen_intr,
    input wire set_fatal_err_intr,

    input wire [C_QP_INDX_WIDTH-1:0] pkt_rcvd_qpid_1,
    input wire                       set_qp_pkt_rcvd_intr_1,

    input wire [C_QP_INDX_WIDTH-1:0] pkt_rcvd_qpid_2,
    input wire                       set_qp_pkt_rcvd_intr_2,

    input  wire                       cnp_sched_intr_en,
    input  wire                       set_cnp_sched_intr,
    input  wire                       clr_cnp_sched_intr,
    output reg                        cnp_sched_intr_sts,
    output wire                       cnp_sched_intr,
    input                             init_start,
    input       [C_QP_INDX_WIDTH-1:0] init_count
);

    wire pkt_valdn_err_intr_sts_en;
    wire mad_pkt_rcvd_intr_sts_en;
    wire bypass_pkt_rcvd_intr_sts_en;
    // wire                in_errsts_intr_sts_en; 
    //wire [C_NUM_QP-1:0] set_qp_pkt_rcvd_intr_i;
    //wire [C_NUM_QP-1:0] qp_pkt_rcvd_intr_sts_en; 
    wire rnr_nack_gen_intr_sts_en;
    wire fatal_err_intr_sts_en;
    wire cnp_sched_intr_sts_en;

    genvar i;

    assign pkt_valdn_err_intr_sts_en   = set_pkt_valdn_err_intr | clr_pkt_valdn_err_intr;
    assign mad_pkt_rcvd_intr_sts_en    = set_mad_pkt_rcvd_intr | clr_mad_pkt_rcvd_intr;
    assign bypass_pkt_rcvd_intr_sts_en = set_bypass_pkt_rcvd_intr | clr_bypass_pkt_rcvd_intr;
    // assign in_errsts_intr_sts_en       = set_in_errsts_intr       | clr_in_errsts_intr;
    assign rnr_nack_gen_intr_sts_en    = set_rnr_nack_gen_intr | clr_rnr_nack_gen_intr;
    assign fatal_err_intr_sts_en       = set_fatal_err_intr | clr_fatal_err_intr;
    assign cnp_sched_intr_sts_en       = set_cnp_sched_intr | clr_cnp_sched_intr;

    reg                        set_qp_pkt_rcvd_intr_1_r;
    reg                        set_qp_pkt_rcvd_intr_2_r;
    reg  [C_QP_INDX_WIDTH-1:0] pkt_rcvd_qpid_1_r;
    reg  [C_QP_INDX_WIDTH-1:0] pkt_rcvd_qpid_2_r;
    reg  [               31:0] clr_qp_pkt_rcvd_intr_r;
    reg  [C_QP_INDX_WIDTH-1:0] clr_qp_pkt_rcvd_intr_wr_addr_r;
    wire [               31:0] set_qp_pkt_rcvd_sts_combo;
    reg                        check_intr_sts;
    reg  [C_QP_INDX_WIDTH-1:0] pkt_rcvd_check_qpid;
    reg                        pkt_rcvd_intr_assert;

    generate
        if (C_NUM_QP < 64) begin
            reg [31:0] qp_pkt_rcvd_intr_sts_sticky;

            for (i = 0; i < 32; i = i + 1) begin
                always @(posedge clk) begin
                    if (~rst_n) begin
                        qp_pkt_rcvd_intr_sts_sticky[i] <= 32'd0;
                    end else begin
                        if (qp_pkt_rcvd_intr_sts_wr_en[i]) begin
                            qp_pkt_rcvd_intr_sts_sticky[i] <= qp_pkt_rcvd_intr_sts[i];
                        end
                    end
                end
            end
            always @(*) begin
                pkt_rcvd_intr_assert <= |qp_pkt_rcvd_intr_sts_sticky;
            end
        end else begin
            ///////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////// qp_pkt_rcvd LUTRAM START ///////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////
            for (i = 0; i < 32; i = i + 1) begin
                sdp_lut_ram_wrapper #(
                    .C_RAM_WORD_WIDTH(1),
                    .C_RAM_DEPTH(C_NUM_QP / 32),
                    .C_Init_with_1(0)
                ) inst_qp_fatal_lutram (
                    .core_clk(clk),
                    .core_rst(!rst_n),
                    .wr_en   (init_start || qp_pkt_rcvd_intr_sts_wr_en[i]),
                    .wr_addr (init_start ? init_count[C_QP_INDX_WIDTH-1:5] : qp_pkt_rcvd_intr_sts_wr_addr[C_QP_INDX_WIDTH-1:5]),
                    .wr_data (init_start ? 1'b0 : qp_pkt_rcvd_intr_sts[i]),
                    .rd_en   (1'b1),
                    .rd_addr (pkt_rcvd_check_qpid[C_QP_INDX_WIDTH-1:5]),
                    .rd_data (set_qp_pkt_rcvd_sts_combo[i])
                );
            end
            ///////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////// qp_pkt_rcvd LUTRAM END /////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////
            always @(posedge clk) begin
                if (~rst_n) begin
                    check_intr_sts       <= 1'b0;
                    pkt_rcvd_check_qpid  <= 'd0;
                    pkt_rcvd_intr_assert <= 1'b0;
                end else begin
                    if (|clr_qp_pkt_rcvd_intr) begin
                        pkt_rcvd_intr_assert <= 1'b0;
                    end else if (|(qp_pkt_rcvd_intr_sts_wr_en & qp_pkt_rcvd_intr_sts)) begin
                        pkt_rcvd_intr_assert <= 1'b1;
                    end else if (check_intr_sts && |set_qp_pkt_rcvd_sts_combo) begin
                        pkt_rcvd_intr_assert <= 1'b1;
                    end

                    if (|clr_qp_pkt_rcvd_intr) begin
                        check_intr_sts      <= 1'b1;
                        pkt_rcvd_check_qpid <= 'd0;
                    end else if (check_intr_sts && |set_qp_pkt_rcvd_sts_combo) begin
                        check_intr_sts      <= 1'b0;
                        pkt_rcvd_check_qpid <= 'd0;
                    end else if (check_intr_sts) begin
                        check_intr_sts      <= (pkt_rcvd_check_qpid == 'd63) ? 1'b0 : 1'b1;
                        pkt_rcvd_check_qpid <= pkt_rcvd_check_qpid + {{C_QP_INDX_WIDTH - 1{1'b0}}, 1'b1};
                    end
                end
            end
        end
    endgenerate

    always @(*) begin
        if (set_qp_pkt_rcvd_intr_1_r) begin
            qp_pkt_rcvd_intr_sts_wr_addr = pkt_rcvd_qpid_1_r[C_QP_INDX_WIDTH-1:0];
        end else if (set_qp_pkt_rcvd_intr_2_r) begin
            qp_pkt_rcvd_intr_sts_wr_addr = pkt_rcvd_qpid_2_r[C_QP_INDX_WIDTH-1:0];
        end else begin
            qp_pkt_rcvd_intr_sts_wr_addr = clr_qp_pkt_rcvd_intr_wr_addr_r;
        end
    end


    generate
        if (C_NUM_QP < 32) begin
            for (i = 0; i < 32; i = i + 1) begin
                always @(*) begin
                    if (i >= C_NUM_QP) begin
                        qp_pkt_rcvd_intr_sts[i]       = 1'b0;
                        qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b0;
                    end else if (set_qp_pkt_rcvd_intr_1_r) begin
                        if (pkt_rcvd_qpid_1_r[C_QP_INDX_WIDTH-1:0] == i[C_QP_INDX_WIDTH-1:0]) begin
                            qp_pkt_rcvd_intr_sts[i]       = 1'b1;
                            qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b1;
                        end else begin
                            qp_pkt_rcvd_intr_sts[i]       = 1'b0;
                            qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b0;
                        end
                    end else if (set_qp_pkt_rcvd_intr_2_r) begin
                        if (pkt_rcvd_qpid_2_r[C_QP_INDX_WIDTH-1:0] == i[C_QP_INDX_WIDTH-1:0]) begin
                            qp_pkt_rcvd_intr_sts[i]       = 1'b1;
                            qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b1;
                        end else begin
                            qp_pkt_rcvd_intr_sts[i]       = 1'b0;
                            qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b0;
                        end
                    end else begin
                        qp_pkt_rcvd_intr_sts[i]       = 1'b0;
                        qp_pkt_rcvd_intr_sts_wr_en[i] = clr_qp_pkt_rcvd_intr_r[i];
                    end
                end
            end
        end else begin
            for (i = 0; i < 32; i = i + 1) begin
                always @(*) begin
                    if (set_qp_pkt_rcvd_intr_1_r) begin
                        if (pkt_rcvd_qpid_1_r[4:0] == i[4:0]) begin
                            qp_pkt_rcvd_intr_sts[i]       = 1'b1;
                            qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b1;
                        end else begin
                            qp_pkt_rcvd_intr_sts[i]       = 1'b0;
                            qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b0;
                        end
                    end else if (set_qp_pkt_rcvd_intr_2_r) begin
                        if (pkt_rcvd_qpid_2_r[4:0] == i[4:0]) begin
                            qp_pkt_rcvd_intr_sts[i]       = 1'b1;
                            qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b1;
                        end else begin
                            qp_pkt_rcvd_intr_sts[i]       = 1'b0;
                            qp_pkt_rcvd_intr_sts_wr_en[i] = 1'b0;
                        end
                    end else begin
                        qp_pkt_rcvd_intr_sts[i]       = 1'b0;
                        qp_pkt_rcvd_intr_sts_wr_en[i] = clr_qp_pkt_rcvd_intr_r[i];
                    end
                end
            end
        end
    endgenerate

    always @(posedge clk) begin
        if (!rst_n) begin
            set_qp_pkt_rcvd_intr_2_r       <= 1'b0;
            set_qp_pkt_rcvd_intr_1_r       <= 1'b0;
            pkt_rcvd_qpid_2_r              <= 'd0;
            pkt_rcvd_qpid_1_r              <= 'd0;
            clr_qp_pkt_rcvd_intr_r         <= 32'd0;
            clr_qp_pkt_rcvd_intr_wr_addr_r <= 32'd0;
        end else begin
            set_qp_pkt_rcvd_intr_1_r <= set_qp_pkt_rcvd_intr_1;
            pkt_rcvd_qpid_1_r        <= pkt_rcvd_qpid_1;
            if (~(set_qp_pkt_rcvd_intr_1_r && set_qp_pkt_rcvd_intr_2_r)) begin
                set_qp_pkt_rcvd_intr_2_r <= set_qp_pkt_rcvd_intr_2;
                pkt_rcvd_qpid_2_r        <= pkt_rcvd_qpid_2;
            end
            if (~((set_qp_pkt_rcvd_intr_1_r || set_qp_pkt_rcvd_intr_2_r) && |clr_qp_pkt_rcvd_intr_r)) begin
                clr_qp_pkt_rcvd_intr_r         <= clr_qp_pkt_rcvd_intr;
                clr_qp_pkt_rcvd_intr_wr_addr_r <= clr_qp_pkt_rcvd_intr_wr_addr;
            end
        end
    end
    //generate for (i=0; i<C_NUM_QP;  i=i+1)
    //begin : gen_qp_pkt_rcvd_intr_sts
    //  assign set_qp_pkt_rcvd_intr_i[i]  = ((i == pkt_rcvd_qpid_1) ? set_qp_pkt_rcvd_intr_1 : 1'b0) ||
    //                                      ((i == pkt_rcvd_qpid_2) ? set_qp_pkt_rcvd_intr_2 : 1'b0);
    //  assign qp_pkt_rcvd_intr_sts_en[i] = set_qp_pkt_rcvd_intr_i[i] | clr_qp_pkt_rcvd_intr[i];
    //  
    //  always @(posedge clk)
    //    if (!rst_n || xrnic_enabled)
    //      qp_pkt_rcvd_intr_sts[i] <= 1'b0;
    //    else if (qp_pkt_rcvd_intr_sts_en[i])
    //      qp_pkt_rcvd_intr_sts[i] <= set_qp_pkt_rcvd_intr_i[i];
    //end    
    //endgenerate

    always @(posedge clk)
        if (!rst_n || xrnic_enabled) begin
            pkt_valdn_err_intr_sts   <= 1'b0;
            mad_pkt_rcvd_intr_sts    <= 1'b0;
            bypass_pkt_rcvd_intr_sts <= 1'b0;
            //in_errsts_intr_sts       <= 1'b0;
            rnr_nack_gen_intr_sts    <= 1'b0;
            fatal_err_intr_sts       <= 1'b0;
            cnp_sched_intr_sts       <= 1'b0;
        end else begin
            if (pkt_valdn_err_intr_sts_en) pkt_valdn_err_intr_sts <= set_pkt_valdn_err_intr;

            if (mad_pkt_rcvd_intr_sts_en) mad_pkt_rcvd_intr_sts <= set_mad_pkt_rcvd_intr;

            if (bypass_pkt_rcvd_intr_sts_en) bypass_pkt_rcvd_intr_sts <= set_bypass_pkt_rcvd_intr;

            //if(in_errsts_intr_sts_en)      
            // in_errsts_intr_sts  <= set_in_errsts_intr; 

            if (rnr_nack_gen_intr_sts_en) rnr_nack_gen_intr_sts <= set_rnr_nack_gen_intr;

            if (fatal_err_intr_sts_en) fatal_err_intr_sts <= set_fatal_err_intr;

            if (cnp_sched_intr_sts_en) cnp_sched_intr_sts <= set_cnp_sched_intr;
        end

    assign pkt_valdn_err_intr   = pkt_valdn_err_intr_en & pkt_valdn_err_intr_sts;
    assign mad_pkt_rcvd_intr    = mad_pkt_rcvd_intr_en & mad_pkt_rcvd_intr_sts;
    assign bypass_pkt_rcvd_intr = bypass_pkt_rcvd_intr_en & bypass_pkt_rcvd_intr_sts;
    //assign in_errsts_intr       = in_errsts_intr_en       & in_errsts_intr_sts;   
    //assign qp_pkt_rcvd_intr     = qp_pkt_rcvd_intr_en     & |(qp_pkt_rcvd_intr_sts);        
    assign qp_pkt_rcvd_intr     = qp_pkt_rcvd_intr_en & pkt_rcvd_intr_assert;
    assign rnr_nack_gen_intr    = rnr_nack_gen_intr_en & rnr_nack_gen_intr_sts;
    assign fatal_err_intr       = fatal_err_intr_en & fatal_err_intr_sts;
    assign cnp_sched_intr       = cnp_sched_intr_en & cnp_sched_intr_sts;

endmodule
