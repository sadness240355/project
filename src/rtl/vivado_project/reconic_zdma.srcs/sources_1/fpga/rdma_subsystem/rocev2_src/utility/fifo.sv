// ////////////////////////////////////////////////////////////////////
//-----------------------------------------------------------------------------
// init_fifo.v
//-----------------------------------------------------------------------------
//
// Filename        : fifo.v
// Version         : v1.0
// Description     : FIFO module
//
// Verilog-Standard: Verilog'2001
//-----------------------------------------------------------------------------
//-- Structure:
//--               -- blk_allocator_init_fifo.v 
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
module fifo #(
    parameter C_Q_DEPTH      = 32,                                       // FIFO Depth
    parameter C_DATA_SIZE    = 256,                                      // FIFO Data Width
    parameter C_THRESHOLD    = 1,                                        // Threshold value for almost full
    parameter C_BRAM_N_FLOP  = 1,                                        // 1- BRAM, 0 - FLOPS
    parameter C_READ_LATENCY = 0,                                        // 0- Fallthough FIFO, others = > Non-fall through FIFO
    parameter C_PTR_WIDTH    = (C_Q_DEPTH == 1) ? 1 : $clog2(C_Q_DEPTH)  // Pointer size reqd for accessing the Q depth 
) (
    input                     core_clk,             // Clock  
    input                     core_rst,             // Active high reset
    output                    o_q_empty,            // Q is empty
    output                    o_q_full,             // Q is full
    output                    o_q_almost_full,      // Q is almost full
    output [ C_PTR_WIDTH : 0] o_num_valid_entries,  // Number of valid entries in the queue
    output [ C_PTR_WIDTH : 0] o_num_free_entries,   // Number of free entries in the queue
    input                     i_q_push,             // Push enable for a queue
    input  [C_DATA_SIZE -1:0] i_q_push_data,        // Data to be pushed
    input                     i_q_pop,              // Pop enable for a queue
    output [C_DATA_SIZE -1:0] o_q_pop_data,         // Pop Data
    output                    o_q_pop_data_vld,

    input tb_sim_eot  //End of SIMULATION TEST

);

    logic   [ C_PTR_WIDTH : 0] rd_ptr;  // Read pointer with roll over bit
    logic   [ C_PTR_WIDTH : 0] wr_ptr;  // Write pointer with roll over bit
    logic   [ C_PTR_WIDTH : 0] num_valid_entries;  // Number of valid entries in the queue
    logic   [ C_PTR_WIDTH : 0] num_free_entries;  // Number of free entries in the queue
    logic                      fifo_full;
    logic   [C_DATA_SIZE -1:0] bram_rdata;
    logic   [C_DATA_SIZE -1:0] bram_rdata_d;
    logic                      fifo_empty;
    logic                      valid_pop;
    logic                      valid_pop_d;
    logic                      valid_push;
    logic                      push_num_entries;
    logic                      pop_num_entries;
    logic   [C_DATA_SIZE -1:0] q_push_data;  // Data to be pushed
    logic   [ C_PTR_WIDTH : 0] i_q_depth;
    logic   [ C_DATA_SIZE-1:0] flop_rdata_d;

    integer                    i;

    assign i_q_depth        = C_Q_DEPTH;
    assign push_num_entries = 1'b1;
    assign pop_num_entries  = 1'b1;

    always @(posedge core_clk) begin
        if (core_rst) begin
            rd_ptr <= 'b0;
        end else begin
            if (valid_pop) begin
                if (rd_ptr[C_PTR_WIDTH-1:0] + pop_num_entries < i_q_depth) rd_ptr[C_PTR_WIDTH-1:0] <= rd_ptr[C_PTR_WIDTH-1:0] + pop_num_entries;
                else begin  // roll over after i_q_depth
                    rd_ptr[C_PTR_WIDTH-1:0] <= rd_ptr[C_PTR_WIDTH-1:0] + pop_num_entries - i_q_depth;
                    rd_ptr[C_PTR_WIDTH]     <= ~rd_ptr[C_PTR_WIDTH];
                end
            end
        end
    end



    assign valid_pop  = i_q_pop && !o_q_empty;  //&& pop_num_entries <= num_valid_entries;
    assign valid_push = i_q_push && !o_q_full;  // && push_num_entries <= num_free_entries;


    always @(posedge core_clk) begin
        if (core_rst) begin
            wr_ptr <= 'b0;
        end else begin
            if (valid_push) begin
                if (wr_ptr[C_PTR_WIDTH-1:0] + push_num_entries < i_q_depth) wr_ptr[C_PTR_WIDTH-1 : 0] <= wr_ptr[C_PTR_WIDTH-1:0] + push_num_entries;
                else begin  // roll over ater i_q_depth
                    wr_ptr[C_PTR_WIDTH-1 : 0] <= wr_ptr[C_PTR_WIDTH-1:0] + push_num_entries - i_q_depth;
                    wr_ptr[C_PTR_WIDTH]       <= ~wr_ptr[C_PTR_WIDTH];
                end
            end
        end
    end



    generate
        if (C_BRAM_N_FLOP == 1) begin : fifo_in_bram

            xpm_memory_sdpram #(

                // Common module parameters
                .MEMORY_SIZE      (C_DATA_SIZE * C_Q_DEPTH),  //positive integer
                .MEMORY_PRIMITIVE ("auto"),                   //string; "auto", "distributed", "block" or "ultra";
                .CLOCKING_MODE    ("common_clock"),           //string; "common_clock", "independent_clock" 
                .MEMORY_INIT_FILE ("none"),                   //string; "none" or "<filename>.mem" 
                .MEMORY_INIT_PARAM(""),                       //string;
                .USE_MEM_INIT     (1),                        //integer; 0,1
                .WAKEUP_TIME      ("disable_sleep"),          //string; "disable_sleep" or "use_sleep_pin" 
                .MESSAGE_CONTROL  (0),                        //integer; 0,1
                .ECC_MODE         ("no_ecc"),                 //string; "no_ecc", "encode_only", "decode_only" or "both_encode_and_decode" 
                .AUTO_SLEEP_TIME  (0),                        //Do not Change

                // Port A module parameters
                .WRITE_DATA_WIDTH_A(C_DATA_SIZE),  //positive integer
                .BYTE_WRITE_WIDTH_A(C_DATA_SIZE),  //integer; 8, 9, or WRITE_DATA_WIDTH_A value
                .ADDR_WIDTH_A      (C_PTR_WIDTH),  //positive integer

                // Port B module parameters
                .READ_DATA_WIDTH_B (C_DATA_SIZE),     //positive integer
                .ADDR_WIDTH_B      (C_PTR_WIDTH),     //positive integer
                .READ_RESET_VALUE_B("0"),             //string
                .READ_LATENCY_B    (C_READ_LATENCY),  //non-negative integer
                .WRITE_MODE_B      ("read_first")     //string; "write_first", "read_first", "no_change" 

            ) inst_bram_fifo (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (1'b1),
                .wea           (valid_push),
                .addra         (wr_ptr[C_PTR_WIDTH-1:0]),
                .dina          (i_q_push_data),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),
                .regceb  (1'b1),
                .addrb   (rd_ptr[C_PTR_WIDTH-1:0]),
                .doutb   (bram_rdata),
                .sbiterrb(),
                .dbiterrb()

            );

            if (C_READ_LATENCY != 0) assign o_q_pop_data = valid_pop_d ? bram_rdata : bram_rdata_d;
            else assign o_q_pop_data = bram_rdata;


            always @(posedge core_clk)
                if (core_rst) bram_rdata_d <= 'h0;
                else if (valid_pop_d) bram_rdata_d <= bram_rdata;

            always @(posedge core_clk)
                if (core_rst) valid_pop_d <= 'h0;
                else valid_pop_d <= valid_pop;

        end else begin : fifo_in_regs

            logic [C_DATA_SIZE-1:0] mem[0 : C_Q_DEPTH-1];

            always @(posedge core_clk)
                if (core_rst) for (i = 0; i < C_Q_DEPTH; i = i + 1) mem[i] <= 'h0;
                else if (valid_push) mem[wr_ptr[C_PTR_WIDTH-1:0]] <= i_q_push_data;

            always @(posedge core_clk)
                if (core_rst) flop_rdata_d <= 'h0;
                //else if(valid_push & (rd_ptr == wr_ptr))
                //  flop_rdata_d <= i_q_push_data;
                else if (valid_pop) flop_rdata_d <= mem[rd_ptr[C_PTR_WIDTH-1:0]];

            if (C_READ_LATENCY != 0) assign o_q_pop_data = flop_rdata_d;
            else assign o_q_pop_data = mem[rd_ptr[C_PTR_WIDTH-1:0]];


        end
    endgenerate

    logic fifo_full_r, fifo_empty_r, fifo_afull_r;

    always @(posedge core_clk) begin
        if (core_rst) begin
            num_valid_entries <= 'b0;
            num_free_entries  <= C_Q_DEPTH;
            fifo_full_r       <= 1'b0;
            fifo_afull_r      <= 1'b0;
            fifo_empty_r      <= 1'b1;
        end else begin
            if (valid_push & !valid_pop) begin
                num_valid_entries <= num_valid_entries + push_num_entries;
                num_free_entries  <= num_free_entries - push_num_entries;
                fifo_empty_r      <= 1'b0;
                fifo_full_r       <= (num_free_entries == 1);
                fifo_afull_r      <= (C_THRESHOLD >= (num_free_entries - 1));
            end else if (valid_pop & !valid_push) begin
                num_valid_entries <= num_valid_entries - pop_num_entries;
                num_free_entries  <= num_free_entries + pop_num_entries;
                fifo_empty_r      <= (num_valid_entries == 1);
                fifo_full_r       <= 1'b0;
                fifo_afull_r      <= (C_THRESHOLD >= (num_free_entries + 1));
            end else if (valid_pop & valid_push) begin
                num_valid_entries <= num_valid_entries - pop_num_entries + push_num_entries;
                num_free_entries  <= num_free_entries + pop_num_entries - push_num_entries;
                fifo_empty_r      <= 1'b0;
                fifo_full_r       <= 1'b0;
                fifo_afull_r      <= (C_THRESHOLD >= num_free_entries);
            end
        end
    end

    assign fifo_empty          = (wr_ptr == rd_ptr) ? 1'b1 : 1'b0;
    assign fifo_full           = (wr_ptr[C_PTR_WIDTH] != rd_ptr[C_PTR_WIDTH]) && (wr_ptr[C_PTR_WIDTH-1:0] == rd_ptr[C_PTR_WIDTH-1:0]);
    assign o_q_full            = fifo_full;
    assign o_q_almost_full     = (C_THRESHOLD >= num_free_entries) ? 1'b1 : 1'b0;
    assign o_q_empty           = fifo_empty;
    assign o_num_valid_entries = num_valid_entries;
    assign o_num_free_entries  = num_free_entries;

    logic [C_READ_LATENCY:0] rvld;

    always @(*) rvld[0] = valid_pop;
    genvar ii;
    generate
        for (ii = 1; ii <= C_READ_LATENCY; ii = ii + 1) begin : rvla_delay
            always @(posedge core_clk)
                if (core_rst) rvld[ii] <= 1'b0;
                else rvld[ii] <= rvld[ii-1];
        end
    endgenerate

    assign o_q_pop_data_vld = rvld[C_READ_LATENCY];


    //`ifdef SIMULATION
    //assert property (@(posedge clk) disable iff (core_rst) (tb_sim_eot |-> o_q_empty)) else $error("***ERROR: FIFO is not empty at end of Simulation\n")
    //assert property (@(posedge clk) disable iff (core_rst) (o_q_empty & i_q_pop)) else $error("***ERROR: Entry pop on FIFO empty condition\n")
    //assert property (@(posedge clk) disable iff (core_rst) (o_q_full & i_q_push)) else $error("***ERROR: Entry push on FIFO full condition \n")
    //`endif

endmodule
