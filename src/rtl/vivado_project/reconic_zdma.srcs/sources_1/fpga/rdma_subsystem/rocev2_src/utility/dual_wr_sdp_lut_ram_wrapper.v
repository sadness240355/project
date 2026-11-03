///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module dual_wr_sdp_lut_ram_wrapper #(
    parameter C_RAM_WORD_WIDTH = 1,
    parameter C_RAM_DEPTH = 2048,
    parameter C_Init_with_1 = 1'b0,  //to initialize all luts with 1 at each location
    parameter C_COMBO_BYPASS = 1'b0  //to bypass combinatorial output assignment if required
) (
    input                             core_clk,
    input                             core_rst,
    input                             wr_en,
    input  [clog2(C_RAM_DEPTH)-1 : 0] wr_addr,
    input  [  C_RAM_WORD_WIDTH-1 : 0] wr_data,
    input                             wr_en_aux,
    input  [clog2(C_RAM_DEPTH)-1 : 0] wr_addr_aux,
    input  [  C_RAM_WORD_WIDTH-1 : 0] wr_data_aux,
    input                             rd_en,
    input  [clog2(C_RAM_DEPTH)-1 : 0] rd_addr,
    output [  C_RAM_WORD_WIDTH-1 : 0] rd_data

);
    `include "../../rdma_macros.vh"

    localparam C_ADDR_WIDTH = clog2(C_RAM_DEPTH);
    localparam C_STATUS_WIDTH = 1;
    // Are these parameters applicable to Ram Init when reset happens ??
    localparam Init_value_1 = {C_RAM_DEPTH - 1{"1,"}};
    localparam Init_value_0 = {C_RAM_DEPTH - 1{"0,"}};
    localparam Init_str_1 = {Init_value_1, "1"};
    localparam Init_str_0 = {Init_value_0, "0"};


    generate
        if (C_Init_with_1 == 1'b0) begin

            wire [  C_STATUS_WIDTH-1 : 0] status_main;
            wire [  C_STATUS_WIDTH-1 : 0] status_aux;
            wire [  C_STATUS_WIDTH-1 : 0] status_aux_for_main;
            wire [  C_STATUS_WIDTH-1 : 0] status_main_for_aux;
            wire [  C_STATUS_WIDTH-1 : 0] rd_status_main;
            wire [  C_STATUS_WIDTH-1 : 0] rd_status_aux;
            wire [  C_STATUS_WIDTH-1 : 0] mem_status;
            reg  [C_RAM_WORD_WIDTH-1 : 0] rd_data_int;
            wire [C_RAM_WORD_WIDTH-1 : 0] rd_data_ram_main;
            wire [C_RAM_WORD_WIDTH-1 : 0] rd_data_ram_aux;

            assign status_main = status_aux_for_main;
            assign status_aux  = ~status_main_for_aux;
            assign mem_status  = rd_status_main ^ rd_status_aux;  //0 means main lut ram has latest data, 1 means aux lut lut ram has latest data

            // To take care of 1 clk write latency
            assign rd_data     = rd_data_int;

            always @(*) begin
                if (rd_addr == wr_addr && wr_en == 1'b1 && C_COMBO_BYPASS == 0) begin
                    rd_data_int = wr_data;
                end else if (rd_addr == wr_addr_aux && wr_en_aux == 1'b1 && C_COMBO_BYPASS == 0) begin
                    rd_data_int = wr_data_aux;
                end else if (mem_status == 1'b1) begin
                    rd_data_int = rd_data_ram_aux;
                end else begin
                    rd_data_int = rd_data_ram_main;
                end
            end


            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_RAM_WORD_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_RAM_WORD_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) dist_sdp_ram_inst_main (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en),
                .wea           (wr_en),
                .addra         (wr_addr),
                .dina          (wr_data),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (rd_en),             // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_data_ram_main),
                .sbiterrb(),
                .dbiterrb()

            );

            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_RAM_WORD_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_RAM_WORD_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) dist_sdp_ram_inst_aux (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en_aux),
                .wea           (wr_en_aux),
                .addra         (wr_addr_aux),
                .dina          (wr_data_aux),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (rd_en),            // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_data_ram_aux),
                .sbiterrb(),
                .dbiterrb()

            );

            /********** status sdp ********************************************/
            /******************************************************************/
            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_STATUS_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_STATUS_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_STATUS_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_STATUS_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) staus_main_0 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en),
                .wea           (wr_en),
                .addra         (wr_addr),
                .dina          (status_main),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),            // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_status_main),
                .sbiterrb(),
                .dbiterrb()

            );


            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_STATUS_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_STATUS_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_STATUS_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_STATUS_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) staus_main_1 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en),
                .wea           (wr_en),
                .addra         (wr_addr),
                .dina          (status_main),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),                 // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (wr_addr_aux),
                .doutb   (status_main_for_aux),
                .sbiterrb(),
                .dbiterrb()

            );

            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_STATUS_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_STATUS_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_STATUS_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_STATUS_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) staus_aux_0 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en_aux),
                .wea           (wr_en_aux),
                .addra         (wr_addr_aux),
                .dina          (status_aux),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),           // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_status_aux),
                .sbiterrb(),
                .dbiterrb()

            );

            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_STATUS_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_STATUS_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_STATUS_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_STATUS_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) staus_aux_1 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en_aux),
                .wea           (wr_en_aux),
                .addra         (wr_addr_aux),
                .dina          (status_aux),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),                 // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (wr_addr),
                .doutb   (status_aux_for_main),
                .sbiterrb(),
                .dbiterrb()

            );




        end
    endgenerate

    generate
        if (C_Init_with_1 == 1'b1) begin

            wire [  C_STATUS_WIDTH-1 : 0] status_main;
            wire [  C_STATUS_WIDTH-1 : 0] status_aux;
            wire [  C_STATUS_WIDTH-1 : 0] status_aux_for_main;
            wire [  C_STATUS_WIDTH-1 : 0] status_main_for_aux;
            wire [  C_STATUS_WIDTH-1 : 0] rd_status_main;
            wire [  C_STATUS_WIDTH-1 : 0] rd_status_aux;
            wire [  C_STATUS_WIDTH-1 : 0] mem_status;
            reg  [C_RAM_WORD_WIDTH-1 : 0] rd_data_int;
            wire [C_RAM_WORD_WIDTH-1 : 0] rd_data_ram_main;
            wire [C_RAM_WORD_WIDTH-1 : 0] rd_data_ram_aux;

            assign status_main = status_aux_for_main;
            assign status_aux  = ~status_main_for_aux;
            assign mem_status  = rd_status_main ^ rd_status_aux;  //0 means main lut ram has latest data, 1 means aux lut lut ram has latest data

            // To take care of 1 clk write latency
            assign rd_data     = rd_data_int;

            always @(*) begin
                if (rd_addr == wr_addr && wr_en == 1'b1 && C_COMBO_BYPASS == 0) begin
                    rd_data_int = wr_data;
                end else if (rd_addr == wr_addr_aux && wr_en_aux == 1'b1 && C_COMBO_BYPASS == 0) begin
                    rd_data_int = wr_data_aux;
                end else if (mem_status == 1'b1) begin
                    rd_data_int = rd_data_ram_aux;
                end else begin
                    rd_data_int = rd_data_ram_main;
                end
            end


            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_RAM_WORD_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_1),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_RAM_WORD_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) dist_sdp_ram_inst_main (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en),
                .wea           (wr_en),
                .addra         (wr_addr),
                .dina          (wr_data),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (rd_en),             // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_data_ram_main),
                .sbiterrb(),
                .dbiterrb()

            );

            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_RAM_WORD_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_1),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_RAM_WORD_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_RAM_WORD_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) dist_sdp_ram_inst_aux (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en_aux),
                .wea           (wr_en_aux),
                .addra         (wr_addr_aux),
                .dina          (wr_data_aux),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (rd_en),            // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_data_ram_aux),
                .sbiterrb(),
                .dbiterrb()

            );

            /********** status sdp ********************************************/
            /******************************************************************/
            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_STATUS_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_STATUS_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_STATUS_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_STATUS_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) staus_main_0 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en),
                .wea           (wr_en),
                .addra         (wr_addr),
                .dina          (status_main),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),            // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_status_main),
                .sbiterrb(),
                .dbiterrb()

            );


            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_STATUS_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_STATUS_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_STATUS_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_STATUS_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) staus_main_1 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en),
                .wea           (wr_en),
                .addra         (wr_addr),
                .dina          (status_main),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),                 // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (wr_addr_aux),
                .doutb   (status_main_for_aux),
                .sbiterrb(),
                .dbiterrb()

            );

            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_STATUS_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_STATUS_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_STATUS_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_STATUS_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) staus_aux_0 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en_aux),
                .wea           (wr_en_aux),
                .addra         (wr_addr_aux),
                .dina          (status_aux),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),           // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (rd_addr),
                .doutb   (rd_status_aux),
                .sbiterrb(),
                .dbiterrb()

            );

            xpm_memory_sdpram #(
                // Common module parameters
                .MEMORY_SIZE      (C_RAM_DEPTH * C_STATUS_WIDTH),
                .MEMORY_PRIMITIVE ("distributed"),
                .CLOCKING_MODE    ("common_clock"),
                .MEMORY_INIT_FILE ("none"),
                .MEMORY_INIT_PARAM(Init_str_0),
                .USE_MEM_INIT     (0),
                .WAKEUP_TIME      ("disable_sleep"),
                .MESSAGE_CONTROL  (0),
                .ECC_MODE         ("no_ecc"),
                .AUTO_SLEEP_TIME  (0),

                // Port A module parameters                                     
                .WRITE_DATA_WIDTH_A(C_STATUS_WIDTH),
                .BYTE_WRITE_WIDTH_A(C_STATUS_WIDTH),
                .ADDR_WIDTH_A      (clog2(C_RAM_DEPTH)),
                // Port B module parameters                                     
                .READ_DATA_WIDTH_B (C_STATUS_WIDTH),
                .ADDR_WIDTH_B      (clog2(C_RAM_DEPTH)),
                .READ_RESET_VALUE_B("0"),
                .READ_LATENCY_B    (0),
                .WRITE_MODE_B      ("read_first")

            ) staus_aux_1 (

                // Common module ports
                .sleep(1'b0),

                // Port A module ports
                .clka          (core_clk),
                .ena           (wr_en_aux),
                .wea           (wr_en_aux),
                .addra         (wr_addr_aux),
                .dina          (status_aux),
                .injectsbiterra(1'b0),
                .injectdbiterra(1'b0),

                // Port B module ports
                .clkb    (core_clk),
                .rstb    (core_rst),
                .enb     (1'b1),                 // for read latency 0, no need to enable the read port else map rd_en
                .regceb  (1'b1),
                .addrb   (wr_addr),
                .doutb   (status_aux_for_main),
                .sbiterrb(),
                .dbiterrb()

            );




        end
    endgenerate


endmodule
