///////////////////////////////////////////////////////////////////////////////
// Module Declaration
// 带去重计数和回推能力的 QPID 调度 FIFO
//      1、先把待处理 QPID 入队
//      2、每次按顺序吐出一个 QPID
//      3、支持把当前 QPID 重新塞回队列（pushback）
//      4、用每 QPID 计数限制同一 QPID 在队列里的重复数量（阈值 2）
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps (* DowngradeIPIdentifiedWarnings="yes" *)
module FIFO_QPID_Arbiter #(
    parameter C_MAX_QID_WIDTH = 3,  // 宽度
    parameter C_MAX_QP = 8  // 数量
) (
    input                              core_clk,
    input                              core_rst,
    input                              init_start,
    input      [C_MAX_QID_WIDTH-1 : 0] init_count,
    // 输入新事件，valid_in + Qpid_in 表示有一个 QPID 需要被处理
    input                              valid_in,
    input      [C_MAX_QID_WIDTH-1 : 0] Qpid_in,
    // 请求“拿下一个 QPID”，上层给 fetch_next_id 脉冲后，模块会在后续给出 arbitration_done=1 和 arbitrated_Qpid
    input                              fetch_next_id,     // give a 1 clk pulse only when need next entry
    // 支持推回，如果上层（比如 FSM）发现“这个 QPID 现在不该继续处理”，可通过 pushback_valid + pushback_Qpid 把它放回队列
    input      [C_MAX_QID_WIDTH-1 : 0] pushback_Qpid,
    input                              pushback_valid,
    output reg                         arbitration_done,  // 本次仲裁结果有效
    output reg [C_MAX_QID_WIDTH-1 : 0] arbitrated_Qpid,   // 仲裁出的 QPID
    output reg                         fifo_busy          // 1clk脉冲，当fifo_busy == 1时，pushback_valid将无效
    // 因此当 fifo_busy 为 1 时，pushback_Qpid 和 Pushback_valid 应保持 1 个 clk
);

    localparam NUM_COUNT_BITS = 2;
    localparam COUNT_THRESHOLD = 2;


    reg  [C_MAX_QID_WIDTH+1 : 0] Qpid_buffer_rd_ptr;
    wire [C_MAX_QID_WIDTH-1 : 0] arb_Qpid_buf;
    wire [  C_MAX_QID_WIDTH : 0] buf_addr_B;
    reg                          Qpid_buf_wr_en;
    reg                          Qpid_buf_wr_en_aux;
    reg                          Qpid_buf_rd_en;
    reg                          Qpid_buf_rd_en_r;
    reg                          transaction_in_progresss;  // 单条仲裁事务占用标志。0：空闲，可启动下一次读仲裁。1：进行中，等待本次结果完成后再启动下一次
    wire                         Qpid_buffer_empty;
    wire                         Qpid_buffer_full;

    // Qpid_cout_val 控制重复度，通过 Qpid_count_val 限制同一 QPID 最多同时保留 2 个待处理项（COUNT_THRESHOLD=2），避免某个 QP 挤爆队列
    reg                          Qpid_cout_val_wr_en;
    reg  [C_MAX_QID_WIDTH-1 : 0] Qpid_cout_val_wr_addr;
    reg  [ NUM_COUNT_BITS-1 : 0] Qpid_cout_val_wr_data;
    reg                          Qpid_cout_val_wr_en_aux;
    reg  [C_MAX_QID_WIDTH-1 : 0] Qpid_cout_val_wr_addr_aux;
    reg  [ NUM_COUNT_BITS-1 : 0] Qpid_cout_val_wr_data_aux;
    wire [ NUM_COUNT_BITS-1 : 0] Qpid_count_val_0;
    wire [ NUM_COUNT_BITS-1 : 0] Qpid_count_val_1;

    reg  [C_MAX_QID_WIDTH-1 : 0] cached_Qpid;
    reg  [C_MAX_QID_WIDTH-1 : 0] Qpid_buf_in;
    reg  [C_MAX_QID_WIDTH-1 : 0] Qpid_buf_in_aux;
    reg  [C_MAX_QID_WIDTH+1 : 0] Qpid_buffer_wr_ptr;
    wire [C_MAX_QID_WIDTH-1 : 0] Qpid_count_rd_addr;
    reg                          cached_valid;
    reg                          arbitration_done_r;


    always @(posedge core_clk) begin : output_process
        if (core_rst) begin
            arbitration_done   <= 1'b0;
            cached_valid       <= 1'b0;
            arbitration_done_r <= 1'b0;
            fifo_busy          <= 1'b0;
            arbitrated_Qpid    <= {C_MAX_QID_WIDTH{1'b0}};
            cached_Qpid        <= {C_MAX_QID_WIDTH{1'b0}};
        end else begin
            arbitration_done_r <= arbitration_done;
            fifo_busy          <= Qpid_buf_rd_en;
            if (fetch_next_id) begin
                arbitration_done <= 1'b0;
            end else if (~arbitration_done && cached_valid) begin
                arbitration_done <= 1'b1;
                arbitrated_Qpid  <= cached_Qpid;
            end

            if (arbitration_done && ~arbitration_done_r) begin
                cached_valid <= 1'b0;
            end else if (Qpid_buf_rd_en_r) begin
                cached_valid <= 1'b1;
                cached_Qpid  <= arb_Qpid_buf;
            end
        end
    end

    always @(posedge core_clk) begin
        if (core_rst) begin
            Qpid_buf_wr_en            <= 1'b0;
            Qpid_buf_wr_en_aux        <= 1'b0;
            Qpid_buffer_wr_ptr        <= {C_MAX_QID_WIDTH + 2{1'b0}};
            Qpid_buf_rd_en            <= 1'b0;
            Qpid_buf_rd_en_r          <= 1'b0;
            Qpid_buffer_rd_ptr        <= {C_MAX_QID_WIDTH + 2{1'b0}};
            transaction_in_progresss  <= 1'b0;
            Qpid_cout_val_wr_en       <= 1'b0;
            Qpid_cout_val_wr_addr     <= {C_MAX_QID_WIDTH{1'b0}};
            Qpid_cout_val_wr_data     <= {NUM_COUNT_BITS{1'b0}};
            Qpid_cout_val_wr_en_aux   <= 1'b0;
            Qpid_cout_val_wr_addr_aux <= {C_MAX_QID_WIDTH{1'b0}};
            Qpid_cout_val_wr_data_aux <= {NUM_COUNT_BITS{1'b0}};
            Qpid_buf_in               <= {C_MAX_QID_WIDTH{1'b0}};
            Qpid_buf_in_aux           <= {C_MAX_QID_WIDTH{1'b0}};
        end else begin
            Qpid_buf_rd_en_r <= Qpid_buf_rd_en;
            // ================== 入队 ==================
            // write side
            // 若 valid_in=1 且该 QPID 计数 <2，Qpid_buf_wr_en=1 写入队列
            if (valid_in && Qpid_count_val_0 < COUNT_THRESHOLD) begin
                Qpid_buf_wr_en <= 1'b1;
                Qpid_buf_in    <= Qpid_in;
            end else begin
                Qpid_buf_wr_en <= 1'b0;
            end

            // write back
            // 若 pushback_valid=1 且 ~fifo_busy 且计数 <2，Qpid_buf_wr_en_aux=1 执行回推写入
            if (pushback_valid && ~fifo_busy && Qpid_count_val_1 < COUNT_THRESHOLD && ~(valid_in && Qpid_in == pushback_Qpid)) begin
                Qpid_buf_wr_en_aux <= 1'b1;
                Qpid_buf_in_aux    <= pushback_Qpid;
            end else begin
                Qpid_buf_wr_en_aux <= 1'b0;
            end
            // 写和推回写入同时成立时，写指针加 2；单独成立加 1
            if (Qpid_buf_wr_en && Qpid_buf_wr_en_aux) begin
                Qpid_buffer_wr_ptr <= Qpid_buffer_wr_ptr + 2;
            end else if (Qpid_buf_wr_en || Qpid_buf_wr_en_aux) begin
                Qpid_buffer_wr_ptr <= Qpid_buffer_wr_ptr + 1;
            end

            // ================== 出队 ==================
            // read side
            // 
            if (~transaction_in_progresss && ~Qpid_buffer_empty && ~(pushback_valid && ~fifo_busy && Qpid_count_val_1 < COUNT_THRESHOLD && ~(valid_in && Qpid_in == pushback_Qpid))) begin
                Qpid_buf_rd_en <= 1'b1;
            end else begin
                Qpid_buf_rd_en <= 1'b0;
            end

            if (arbitration_done && ~arbitration_done_r) begin  // 刚仲裁完
                transaction_in_progresss <= 1'b0;
            end else if (~transaction_in_progresss && ~Qpid_buffer_empty && ~(pushback_valid && ~fifo_busy && Qpid_count_val_1 < COUNT_THRESHOLD && ~(valid_in && Qpid_in == pushback_Qpid))) begin
                transaction_in_progresss <= 1'b1;
            end

            if (Qpid_buf_rd_en) begin
                Qpid_buffer_rd_ptr <= Qpid_buffer_rd_ptr + 1;
            end

            //existing entry counter per QP logic
            // change due to write and read on the same clock for same Qpid (no change in effective count value)
            if (valid_in && Qpid_count_val_0 < COUNT_THRESHOLD && Qpid_buf_rd_en_r && Qpid_in == arb_Qpid_buf) begin
                Qpid_cout_val_wr_en     <= 1'b0;
                Qpid_cout_val_wr_en_aux <= 1'b0;
                // change due to write and write back on the same clock for same Qpid (only 1 entry has to be saved)	
            end else if (valid_in && Qpid_count_val_0 < COUNT_THRESHOLD && pushback_valid && ~fifo_busy && Qpid_in == pushback_Qpid) begin
                Qpid_cout_val_wr_en     <= 1'b1;
                Qpid_cout_val_wr_addr   <= Qpid_in;
                Qpid_cout_val_wr_data   <= Qpid_count_val_0 + 1;
                Qpid_cout_val_wr_en_aux <= 1'b0;
            end else begin
                if (valid_in && Qpid_count_val_0 < COUNT_THRESHOLD) begin  //write side 
                    Qpid_cout_val_wr_en   <= 1'b1;
                    Qpid_cout_val_wr_addr <= Qpid_in;
                    Qpid_cout_val_wr_data <= Qpid_count_val_0 + 1;
                end else begin
                    Qpid_cout_val_wr_en <= 1'b0;
                end

                if (Qpid_buf_rd_en_r && Qpid_count_val_1 > 0) begin  // read side
                    Qpid_cout_val_wr_en_aux   <= 1'b1;
                    Qpid_cout_val_wr_addr_aux <= arb_Qpid_buf;
                    Qpid_cout_val_wr_data_aux <= Qpid_count_val_1 - 1;
                end else if (pushback_valid && ~fifo_busy && Qpid_count_val_1 < COUNT_THRESHOLD) begin  // write_back
                    Qpid_cout_val_wr_en_aux   <= 1'b1;
                    Qpid_cout_val_wr_addr_aux <= pushback_Qpid;
                    Qpid_cout_val_wr_data_aux <= Qpid_count_val_1 + 1;
                end else begin
                    Qpid_cout_val_wr_en_aux <= 1'b0;
                end
            end
        end
    end

    assign Qpid_buffer_empty = (Qpid_buffer_wr_ptr == Qpid_buffer_rd_ptr) ? 1'b1 : 1'b0;
    assign Qpid_buffer_full  = (Qpid_buffer_wr_ptr[C_MAX_QID_WIDTH+1] != Qpid_buffer_rd_ptr[C_MAX_QID_WIDTH+1] && Qpid_buffer_wr_ptr[C_MAX_QID_WIDTH : 0] == Qpid_buffer_rd_ptr[C_MAX_QID_WIDTH : 0]) ? 1'b1 : 1'b0;
    assign buf_addr_B        = Qpid_buf_wr_en_aux ? (Qpid_buf_wr_en ? Qpid_buffer_wr_ptr[C_MAX_QID_WIDTH : 0] + 1 : Qpid_buffer_wr_ptr[C_MAX_QID_WIDTH : 0]) : Qpid_buffer_rd_ptr[C_MAX_QID_WIDTH : 0];


    // QPID 环形队列
    xpm_memory_tdpram #(
        .ADDR_WIDTH_A           (C_MAX_QID_WIDTH + 1),
        .ADDR_WIDTH_B           (C_MAX_QID_WIDTH + 1),
        .AUTO_SLEEP_TIME        (0),
        .BYTE_WRITE_WIDTH_A     (C_MAX_QID_WIDTH),
        .BYTE_WRITE_WIDTH_B     (C_MAX_QID_WIDTH),
        .CASCADE_HEIGHT         (0),
        .CLOCKING_MODE          ("common_clock"),
        .ECC_MODE               ("no_ecc"),
        .MEMORY_INIT_FILE       ("none"),
        .MEMORY_INIT_PARAM      ("0"),
        .MEMORY_OPTIMIZATION    ("true"),
        .MEMORY_PRIMITIVE       ("auto"),
        .MEMORY_SIZE            (2 * C_MAX_QID_WIDTH * C_MAX_QP),
        .MESSAGE_CONTROL        (0),
        .READ_DATA_WIDTH_A      (C_MAX_QID_WIDTH),
        .READ_DATA_WIDTH_B      (C_MAX_QID_WIDTH),
        .READ_LATENCY_A         (1),
        .READ_LATENCY_B         (1),
        .READ_RESET_VALUE_A     ("0"),
        .READ_RESET_VALUE_B     ("0"),
        .RST_MODE_A             ("SYNC"),
        .RST_MODE_B             ("SYNC"),
        .SIM_ASSERT_CHK         (0),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT           (0),
        .WAKEUP_TIME            ("disable_sleep"),
        .WRITE_DATA_WIDTH_A     (C_MAX_QID_WIDTH),
        .WRITE_DATA_WIDTH_B     (C_MAX_QID_WIDTH),
        .WRITE_MODE_A           ("read_first"),
        .WRITE_MODE_B           ("read_first")
    ) inst_Qpid_buffer (
        // A 口用于正常写
        .clka  (core_clk),
        .rsta  (core_rst),
        .regcea(1),
        .ena   (Qpid_buf_wr_en),
        .wea   (Qpid_buf_wr_en),
        .addra (Qpid_buffer_wr_ptr[C_MAX_QID_WIDTH : 0]),
        .dina  (Qpid_buf_in),
        .douta (),

        // B 口用于推回写入或读
        .clkb  (core_clk),
        .rstb  (core_rst),
        .regceb(1'b1),
        .enb   (Qpid_buf_wr_en_aux || Qpid_buf_rd_en),
        .web   (Qpid_buf_wr_en_aux || (~Qpid_buf_rd_en)),
        .addrb (buf_addr_B),
        .dinb  (Qpid_buf_in_aux),
        .doutb (arb_Qpid_buf)
    );


    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(NUM_COUNT_BITS),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qpid_count_value_0 (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || Qpid_cout_val_wr_en),
        .wr_addr    (init_start ? init_count : Qpid_cout_val_wr_addr),
        .wr_data    (init_start ? {NUM_COUNT_BITS{1'b0}} : Qpid_cout_val_wr_data),
        .wr_en_aux  (Qpid_cout_val_wr_en_aux),
        .wr_addr_aux(Qpid_cout_val_wr_addr_aux),
        .wr_data_aux(Qpid_cout_val_wr_data_aux),
        .rd_en      (1'b1),
        .rd_addr    (Qpid_in),
        .rd_data    (Qpid_count_val_0)
    );

    dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(NUM_COUNT_BITS),
        .C_RAM_DEPTH(C_MAX_QP)
    ) inst_qpid_count_value_1 (
        .core_clk   (core_clk),
        .core_rst   (core_rst),
        .wr_en      (init_start || Qpid_cout_val_wr_en),
        .wr_addr    (init_start ? init_count : Qpid_cout_val_wr_addr),
        .wr_data    (init_start ? {NUM_COUNT_BITS{1'b0}} : Qpid_cout_val_wr_data),
        .wr_en_aux  (Qpid_cout_val_wr_en_aux),
        .wr_addr_aux(Qpid_cout_val_wr_addr_aux),
        .wr_data_aux(Qpid_cout_val_wr_data_aux),
        .rd_en      (1'b1),
        .rd_addr    ((pushback_valid && ~fifo_busy) ? pushback_Qpid : arb_Qpid_buf),
        .rd_data    (Qpid_count_val_1)
    );

endmodule
