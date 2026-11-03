`default_nettype wire
///////////////////////////////////////////////////////////////////////////////
// Module Declaration
///////////////////////////////////////////////////////////////////////////////
// lrdma_ack_retry_timer — ACK 管理器 per-QP 重试定时器
//
// 【模块功能】
// 本模块为每个普通 QP 保存重试超时计数。WQE 首包登记或 ACK/NAK 到达时会把 reload
// 请求写入 FIFO，仲裁后更新对应 QP 的 timer RAM。扫描 FSM 周期性读取每个 QP 的计数，
// 在 OSQ 非空、timer 未禁用、所有 outstanding 响应尚未收齐、且没有 NACK/fatal/recovery
// 屏蔽时递减计数；到期后输出 o_q_timed_out_valid 和对应 qpid。
//
// 【扫描方式】
// 主扫描 FSM 按 RD_CNTR → MOD_CNTR → WR_CNTR 访问 timer RAM。qpid_proc 从 1 开始，
// 因此内部 qpid0（软件 QP1/MAD 路径）不参与普通重试定时。
//
// 【reload 和屏蔽】
// WQE reload 与 ACK/NAK reload 分别进入 FIFO，再由 round-robin 仲裁写入 RAM。reload
// 进入 FIFO 后到真正写入 RAM 前有延迟，timeout_mask 会在这段窗口内屏蔽该 QP 的 timeout
// 输出，避免 reload 尚未生效时误触发重传。
//
// 【计数补偿】
// 每个 reload 项携带 trigger_count。扫描修改计数时会根据扫描触发点 trigger_count 与 reload 时的
// trigger_count 差值做补偿，减少不同 QP 被扫描时刻不同造成的计时偏差。
///////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps

module lrdma_ack_retry_timer #(
    parameter C_NUM_QP           = 256,
    parameter C_QP_INDX_WIDTH    = 8,
    parameter LOG2_NUM_QP        = $clog2(C_NUM_QP),
    parameter C_TIMER_VAL_32_7us = 6554,
    parameter C_TIMER_VAL_4_1us  = 820,
    parameter SUBTRACT_VAL       = 1
) (
    // 时钟和复位
    input wire core_clk,
    input wire core_rstn,

    output logic                           o_q_timed_out_valid,
    output logic [      LOG2_NUM_QP-1 : 0] o_q_timed_out_wr_addr,
    // per-QP 状态输入采用打包格式，包含 qpid、valid/data 和 pending 信息
    input  wire  [(2*LOG2_NUM_QP+3)-1 : 0] osq_empty,
    input  wire  [(2*LOG2_NUM_QP+4)-1 : 0] timer_disabled_per_qp,
    input  wire  [(2*LOG2_NUM_QP+2)-1 : 0] all_outstanding_resp_received,
    input  wire  [  (LOG2_NUM_QP+2)-1 : 0] i_qp_fatal,
    input  wire                            i_reload_timer_wqe,
    input  wire  [      LOG2_NUM_QP - 1:0] i_reload_timer_wqe_qpid,
    input  wire                            i_reload_timer_acknack,
    input  wire  [      LOG2_NUM_QP - 1:0] i_reload_timer_acknack_qpid,
    input  wire  [                    4:0] i_reload_value_wqe,
    input  wire  [                    4:0] i_reload_value_acknack,
    input  wire                            init_start,
    input  wire  [      LOG2_NUM_QP-1 : 0] init_count,
    input  wire                            osq_nacked_timer_mem_rdaddr,
    output wire  [      LOG2_NUM_QP-1 : 0] o_timer_mem_rdaddr,
    input  wire                            osq_nacked_ff_rd_data,
    input  wire                            i_qp_under_recovery_wr_en,
    input  wire  [      LOG2_NUM_QP-1 : 0] i_qp_under_recovery_wr_idx,
    input  wire                            i_qp_under_recovery_wr_data
);

    `include "lrdma_defs.vh"
    //==========================================================================
    // local parameters
    //==========================================================================
    localparam LUTRAM_WIDTH = 5 + 5;
    localparam W_4_1us_CNT = 4;
    localparam INPUT_FIFO_WIDTH = LOG2_NUM_QP + 5 + W_4_1us_CNT;
    localparam NUM_REQS = 2;
    localparam GNT_ACKNACK = 0;
    localparam GNT_WQE = 1;
    localparam ARB_GNT_ACKNACK = 1;
    localparam ARB_GNT_WQE = 2;
    localparam MEM_DATA_WIDTH = 43;
    localparam C_RES_VAL = 4'b1001;

    //==========================================================================
    // structures
    //==========================================================================
    typedef struct packed {
        logic [W_4_1us_CNT-1:0] trigger_count;
        logic [4:0]             init_value;
        logic                   cntr_value_0;
        logic                   final_count;
        logic [31:0]            cntr_value;
    } memdata;

    memdata mem_a_wr_data, mem_b_wr_data, mem_b_rd_data;


    logic                        all_outstanding_resp_received_rd_data;
    //==========================================================================
    // registers and wire declarations
    //==========================================================================

    logic                        wqe_reload_fifo_empty;
    logic                        wqe_reload_fifo_rden;
    logic [   LOG2_NUM_QP - 1:0] wqe_reload_fifo_qpid_out;
    logic [                 4:0] wqe_reload_fifo_cntr_out;
    logic                        acknack_reload_fifo_wren;
    logic [   LOG2_NUM_QP - 1:0] acknack_reload_fifo_qpid_in;
    logic [                 4:0] acknack_reload_fifo_cntr_in;
    logic                        acknack_reload_fifo_empty;
    logic                        acknack_reload_fifo_rden;
    logic [   LOG2_NUM_QP - 1:0] acknack_reload_fifo_qpid_out;
    logic [                 4:0] acknack_reload_fifo_cntr_out;
    logic [      NUM_REQS - 1:0] arb_reqs;
    logic [      NUM_REQS - 1:0] arb_gnt;
    logic                        arb_done;
    logic [   LOG2_NUM_QP - 1:0] qpid_to_update;
    logic [               4 : 0] cntr_to_update;
    logic                        timer_enable;
    logic [                12:0] free_running_timer_32_7us;
    logic [                 9:0] free_running_timer_4_1_us;
    logic [     W_4_1us_CNT-1:0] trigger_count_4_1us;
    logic [     W_4_1us_CNT-1:0] trigger_count_4_1us_out;
    logic [     W_4_1us_CNT-1:0] trigger_count_4_1us_acknack_out;
    logic [   LOG2_NUM_QP - 1:0] wqe_timer_qpid;
    logic                        timer_mem_wren;
    logic [   LOG2_NUM_QP - 1:0] timer_mem_wraddr;
    logic [MEM_DATA_WIDTH - 1:0] timer_mem_wrdata;
    logic                        terminal_count;
    logic                        terminal_count_4_1;
    logic                        qpid_proc_done;
    logic [   LOG2_NUM_QP - 1:0] qpid_proc;
    logic                        timer_mem_rden;
    logic [   LOG2_NUM_QP - 1:0] timer_mem_rdaddr;
    reg   [   LOG2_NUM_QP - 1:0] timer_mem_rdaddr_comb;
    logic [MEM_DATA_WIDTH - 1:0] timer_mem_rddata;
    logic [MEM_DATA_WIDTH - 1:0] timer_mem_mod_rddata;
    logic                        timer_mem_mod_wren;
    logic                        rd_qp_incr;

    logic [41:0] global_timer, global_timer_ff, global_timer_toggled;
    reg                       timer_en;
    reg                       timer_reset;
    wire                      timer_reload_pending_int;
    wire                      timer_reload_pending;
    wire                      osq_empty_rd_data_int_0;
    wire                      osq_empty_rd_data_0;
    wire                      timer_disabled_per_qp_rd_data_int;
    wire                      timer_disabled_per_qp_rd_data;
    reg                       timeout_mask_set_valid;
    reg   [LOG2_NUM_QP-1 : 0] timeout_mask_set_id;
    reg                       timeout_mask_reset_valid;
    reg   [LOG2_NUM_QP-1 : 0] timeout_mask_reset_id;
    reg                       timeout_mask_set_valid_pending;
    reg   [LOG2_NUM_QP-1 : 0] timeout_mask_set_id_pending;
    wire                      reset_timer_osq_nacked;
    wire                      qp_under_recovery_rd_data;
    wire                      qp_fatal_rd_data;
    logic [W_4_1us_CNT-1 : 0] trigger_count;


    //==========================================================================
    // fsm states
    //==========================================================================
    typedef enum logic [1:0] {
        RD_CNTR,
        MOD_CNTR,
        WR_CNTR
    } states_t;
    states_t curr_st, next_st;

    `MSFF_R(global_timer, (global_timer + 1'b1), core_clk, ~core_rstn)
    `MSFF_R(global_timer_ff, (global_timer), core_clk, ~core_rstn)

    // global_timer 每一位翻转都会在对应 bit 上形成单拍脉冲
    assign global_timer_toggled = global_timer ^ global_timer_ff;


    assign timer_mem_wrdata     = mem_a_wr_data;
    assign mem_b_rd_data        = timer_mem_rddata;
    assign timer_mem_mod_rddata = mem_b_wr_data;

    // 主扫描周期计数器。周期由 C_TIMER_VAL_32_7us 决定。
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            free_running_timer_32_7us <= 13'h0;
        end else if (free_running_timer_32_7us == C_TIMER_VAL_32_7us - 1) begin
            free_running_timer_32_7us <= 13'h0;
        end else begin
            free_running_timer_32_7us <= free_running_timer_32_7us + 1'b1;
        end
    end

    // 计数到参数设定值前一拍时产生 terminal_count
    assign terminal_count = (free_running_timer_32_7us == C_TIMER_VAL_32_7us - 1);

    // 子周期计数器。周期由 C_TIMER_VAL_4_1us 决定。
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            free_running_timer_4_1_us <= 10'h0;
        end else if ((free_running_timer_4_1_us == C_TIMER_VAL_4_1us - 1) || terminal_count) begin
            free_running_timer_4_1_us <= 10'h0;
        end else begin
            free_running_timer_4_1_us <= free_running_timer_4_1_us + 1'b1;
        end
    end

    // 计数到参数设定值前一拍时产生 terminal_count
    assign terminal_count_4_1 = (free_running_timer_4_1_us == C_TIMER_VAL_4_1us - 1);

    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            trigger_count_4_1us <= 'd0;
        end else if (terminal_count) begin
            trigger_count_4_1us <= 'd0;
        end else if (terminal_count_4_1) begin
            trigger_count_4_1us <= trigger_count_4_1us + 1;
        end
    end

    /***************************************************************************
   *                                                                          *
   * 									      *
   *     	 FIFO based logic to arbitrate between requests		      *
   * 									      *
   * 									      *
   ***************************************************************************/

    //==========================================================================
    // WQE reload FIFO：WQE 首次发送后写入，表示该 QP 需要启动/刷新定时器
    //==========================================================================

    assign wqe_timer_qpid = i_reload_timer_wqe_qpid;

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
        .READ_DATA_WIDTH    (INPUT_FIFO_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (INPUT_FIFO_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_wqe_reload (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (i_reload_timer_wqe),
        .din          ({trigger_count_4_1us, i_reload_value_wqe, wqe_timer_qpid}),
        .rd_en        (wqe_reload_fifo_rden),
        .empty        (wqe_reload_fifo_empty),
        .dout         ({trigger_count_4_1us_out, wqe_reload_fifo_cntr_out, wqe_reload_fifo_qpid_out}),
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

    //==========================================================================
    // ACK/NAK reload FIFO：收到 ACK/NAK 后写入，表示该 QP 需要刷新定时器
    //==========================================================================

    // ACK/NAK reload 作为独立来源进入 FIFO，后续与 WQE reload 做仲裁
    assign acknack_reload_fifo_wren    = i_reload_timer_acknack;
    assign acknack_reload_fifo_qpid_in = i_reload_timer_acknack_qpid;
    assign acknack_reload_fifo_cntr_in = i_reload_value_acknack;


    // reload 请求进入 FIFO 后，到真正写入 timer RAM 前存在延迟。
    // timeout_mask 在这段窗口内屏蔽对应 QP 的超时输出，避免误触发重传。
    always_ff @(posedge core_clk) begin : timeout_mask
        if (!core_rstn) begin
            timeout_mask_set_valid         <= 1'b0;
            timeout_mask_set_id            <= {LOG2_NUM_QP{1'b0}};
            timeout_mask_reset_valid       <= 1'b0;
            timeout_mask_reset_id          <= {LOG2_NUM_QP{1'b0}};
            timeout_mask_set_valid_pending <= 1'b0;
            timeout_mask_set_id_pending    <= {LOG2_NUM_QP{1'b0}};
        end else begin
            if (i_reload_timer_wqe) begin
                timeout_mask_set_valid <= 1'b1;
                timeout_mask_set_id    <= i_reload_timer_wqe_qpid;
            end else if (i_reload_timer_acknack) begin
                timeout_mask_set_valid <= 1'b1;
                timeout_mask_set_id    <= i_reload_timer_acknack_qpid;
            end else if (timeout_mask_set_valid_pending && !(timer_mem_wren && timer_mem_wraddr == timeout_mask_set_id_pending)) begin
                timeout_mask_set_valid <= 1'b1;
                timeout_mask_set_id    <= timeout_mask_set_id_pending;
            end else begin
                timeout_mask_set_valid <= 1'b0;
            end

            if (i_reload_timer_wqe && i_reload_timer_acknack && i_reload_timer_wqe_qpid != i_reload_timer_acknack_qpid) begin
                timeout_mask_set_valid_pending <= 1'b1;
                timeout_mask_set_id_pending    <= i_reload_timer_acknack_qpid;
            end else begin
                timeout_mask_set_valid_pending <= 1'b0;
            end

            if (timer_mem_wren && !(i_reload_timer_wqe && timer_mem_wraddr == i_reload_timer_wqe_qpid) && !(i_reload_timer_acknack && timer_mem_wraddr == i_reload_timer_acknack_qpid)) begin
                timeout_mask_reset_valid <= 1'b1;
                timeout_mask_reset_id    <= timer_mem_wraddr;
            end else begin
                timeout_mask_reset_valid <= 1'b0;
            end
        end
    end

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_reload_timer_wqe (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || timeout_mask_reset_valid),
        .wr_addr    (init_start ? init_count : timeout_mask_reset_id),
        .wr_data    (1'b0),
        .wr_en_aux  (timeout_mask_set_valid),
        .wr_addr_aux(timeout_mask_set_id),
        .wr_data_aux(1'b1),
        .rd_en      (1'b1),
        .rd_addr    (timer_mem_rdaddr),
        .rd_data    (timer_reload_pending_int)
    );
    assign timer_reload_pending = (timeout_mask_set_valid_pending && timeout_mask_set_id_pending == timer_mem_rdaddr) ? 1'b1 : timer_reload_pending_int;

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
        .READ_DATA_WIDTH    (INPUT_FIFO_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (0),
        .USE_ADV_FEATURES   ("0707"),
        .WAKEUP_TIME        (0),
        .WRITE_DATA_WIDTH   (INPUT_FIFO_WIDTH),
        .WR_DATA_COUNT_WIDTH(1)
    ) u_acknack_reload (
        .wr_clk       (core_clk),
        .rst          (!core_rstn),
        .wr_en        (acknack_reload_fifo_wren),
        .din          ({trigger_count_4_1us, acknack_reload_fifo_cntr_in, acknack_reload_fifo_qpid_in}),
        .rd_en        (acknack_reload_fifo_rden),
        .empty        (acknack_reload_fifo_empty),
        .dout         ({trigger_count_4_1us_acknack_out, acknack_reload_fifo_cntr_out, acknack_reload_fifo_qpid_out}),
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

    /***************************************************************************
   *                                                                          *
   * 									      *
   * 		Arbiter to arbitrate between interfaces			      *
   * 									      *
   * 									      *
   ***************************************************************************/

    assign arb_reqs = {!wqe_reload_fifo_empty, !acknack_reload_fifo_empty};
    assign arb_done = 1'b1;

    // 在 ACK/NAK reload FIFO 和 WQE reload FIFO 之间做仲裁
    lrdma_common_arbiter #(
        .NUM_REQS(NUM_REQS),
        .BUS_HOLD(0),
        .NEW_ARB (1),
        .SPLIT   (0)
    ) u_fifo_arbiter (
        .clk_arb  (core_clk),
        .rst_n_arb(core_rstn),
        .reqs     (arb_reqs),
        .arb_done (arb_done),
        .gnt      (arb_gnt),
        .gnt_valid()
    );

    // 被仲裁到的 FIFO 弹出一项 reload 请求
    assign acknack_reload_fifo_rden = arb_gnt[GNT_ACKNACK];
    assign wqe_reload_fifo_rden     = arb_gnt[GNT_WQE];
    // 根据仲裁结果选择要更新的 qpid、reload 值和 trigger_count
    always_comb begin
        case (arb_gnt)
            ARB_GNT_ACKNACK: begin
                qpid_to_update = acknack_reload_fifo_qpid_out;
                cntr_to_update = acknack_reload_fifo_cntr_out;
                trigger_count  = trigger_count_4_1us_acknack_out;
            end
            default: begin
                qpid_to_update = wqe_reload_fifo_qpid_out;
                cntr_to_update = wqe_reload_fifo_cntr_out;
                trigger_count  = trigger_count_4_1us_out;
            end
        endcase
    end


    // 将选中的 reload 信息写入对应 QP 的 timer RAM 条目
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            timer_mem_wren   <= 1'b0;
            timer_mem_wraddr <= {LOG2_NUM_QP{1'b0}};
            mem_a_wr_data    <= {MEM_DATA_WIDTH{1'b0}};
        end else if (|arb_gnt) begin
            timer_mem_wren              <= 1'b1;
            timer_mem_wraddr            <= qpid_to_update;
            mem_a_wr_data.init_value    <= cntr_to_update;
            mem_a_wr_data.cntr_value_0  <= (cntr_to_update == 5'h0);
            mem_a_wr_data.final_count   <= 1'b0;
            mem_a_wr_data.cntr_value    <= (1 << cntr_to_update);
            mem_a_wr_data.trigger_count <= trigger_count;
        end else begin
            timer_mem_wren   <= 1'b0;
            timer_mem_wraddr <= {LOG2_NUM_QP{1'b0}};
            mem_a_wr_data    <= {MEM_DATA_WIDTH{1'b0}};
        end
    end

    // per-QP timer RAM：A 端写 reload，新值由 FIFO 仲裁产生；B 端用于扫描读改写。
    xpm_memory_tdpram #(
        .ADDR_WIDTH_A           (LOG2_NUM_QP),
        .ADDR_WIDTH_B           (LOG2_NUM_QP),
        .AUTO_SLEEP_TIME        (0),
        .BYTE_WRITE_WIDTH_A     (MEM_DATA_WIDTH),
        .BYTE_WRITE_WIDTH_B     (MEM_DATA_WIDTH),
        .CASCADE_HEIGHT         (0),
        .CLOCKING_MODE          ("common_clock"),
        .ECC_MODE               ("no_ecc"),
        .MEMORY_INIT_FILE       ("none"),
        .MEMORY_INIT_PARAM      ("0"),
        .MEMORY_OPTIMIZATION    ("true"),
        .MEMORY_PRIMITIVE       ("auto"),
        .MEMORY_SIZE            (C_NUM_QP * MEM_DATA_WIDTH),
        .MESSAGE_CONTROL        (0),
        .READ_DATA_WIDTH_A      (MEM_DATA_WIDTH),
        .READ_DATA_WIDTH_B      (MEM_DATA_WIDTH),
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
        .WRITE_DATA_WIDTH_A     (MEM_DATA_WIDTH),
        .WRITE_DATA_WIDTH_B     (MEM_DATA_WIDTH),
        .WRITE_MODE_A           ("no_change"),
        .WRITE_MODE_B           ("no_change")
    ) u_per_qp_counter (
        .clka          (core_clk),
        .rsta          (!core_rstn),
        .ena           (timer_mem_wren),
        .addra         (timer_mem_wraddr),
        .wea           (timer_mem_wren),
        .dina          (timer_mem_wrdata),
        .douta         (),                                     // A 端只写 reload，读数据未接出
        .clkb          (core_clk),
        .rstb          (!core_rstn),
        .web           (timer_mem_mod_wren),
        .regcea        (1'b1),
        .addrb         (timer_mem_rdaddr),
        .regceb        (1'b1),
        .doutb         (timer_mem_rddata),
        .dinb          (timer_mem_mod_rddata),
        .enb           (timer_mem_rden | timer_mem_mod_wren),
        .injectdbiterra(1'b0),
        .injectdbiterrb(1'b0),
        .injectsbiterra(1'b0),
        .injectsbiterrb(1'b0),
        .sleep         (1'b0),
        .sbiterra      (),
        .sbiterrb      (),
        .dbiterra      (),
        .dbiterrb      ()
    );

    // 状态寄存器
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            curr_st <= RD_CNTR;
        end else begin
            curr_st <= next_st;
        end
    end

    // 每个主周期扫描一遍普通 QP。若 reload 写口正在写被扫描 qpid，则跳过这次读改写，
    // 避免用读返回值覆盖刚写入的新计数。
    always_comb begin
        case (curr_st)
            RD_CNTR: begin
                if (terminal_count | !qpid_proc_done) begin
                    // 同拍 reload 写同一个 QP 时，本轮不做读改写，避免覆盖 reload 新值。
                    if (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) begin
                        next_st = RD_CNTR;
                    end else begin
                        next_st = MOD_CNTR;
                    end
                end else begin
                    next_st = RD_CNTR;
                end
            end
            MOD_CNTR: begin
                next_st = (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) ? RD_CNTR : WR_CNTR;
            end
            default: begin  // WR_CNTR
                next_st = RD_CNTR;
            end
        endcase
    end

    always_comb begin
        case (curr_st)
            RD_CNTR: begin
                // terminal_count 到来后开始新一轮扫描；qpid_proc_done 在本轮各 QP 处理完成前保持为 0。
                if (terminal_count | !qpid_proc_done) begin
                    // 同拍 reload 写同一个 QP 时，本轮不做读改写，避免覆盖 reload 新值。
                    if (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) begin
                        rd_qp_incr = 1'b1;
                    end else begin
                        rd_qp_incr = 1'b0;
                    end
                end else begin
                    rd_qp_incr = 1'b0;
                end
            end
            MOD_CNTR: begin
                rd_qp_incr = (timer_mem_wren & (timer_mem_wraddr == qpid_proc));
            end
            default: begin  // WR_CNTR:
                rd_qp_incr = 1'b1;
            end
        endcase
    end

    // qpid_proc_done 表示被扫描 QP 是否处理完。新一轮开始后清 0，被扫描 QP 写回结束后置 1。
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            qpid_proc_done <= 1'b1;
        end  // reset proc done when terminal count is set
        else if (terminal_count) begin
            qpid_proc_done <= 1'b0;
        end  // when the last QPID is processed, set qpid_proc_done
        else if (qpid_proc == (C_NUM_QP - 1)) begin
            case (curr_st)
                RD_CNTR: begin
                    if (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) begin
                        qpid_proc_done <= 1'b1;
                    end else begin
                        qpid_proc_done <= 1'b0;
                    end
                end
                MOD_CNTR: begin
                    qpid_proc_done <= (timer_mem_wren & (timer_mem_wraddr == qpid_proc)) ? 1'b1 : 1'b0;
                end
                default: begin
                    qpid_proc_done <= 1'b1;
                end
            endcase
        end
    end

    // 被扫描的 QP；从 1 开始，扫描到 C_NUM_QP-1 后回到 1。
    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            qpid_proc <= {{(LOG2_NUM_QP - 2) {1'b0}}, 2'b01};
        end else if (rd_qp_incr) begin
            if (qpid_proc == C_NUM_QP - 1) begin
                qpid_proc <= {{(LOG2_NUM_QP - 2) {1'b0}}, 2'b01};
            end else begin
                qpid_proc <= qpid_proc + 1'b1;
            end
        end
    end

    // 只有 reload 写口没有写同一个 qpid 时，B 端才发起 timer RAM 读。
    assign timer_mem_rden   = ((curr_st == RD_CNTR) & (terminal_count | !qpid_proc_done) & !(timer_mem_wren & (timer_mem_wraddr == qpid_proc)));
    assign timer_mem_rdaddr = qpid_proc;

    always_comb begin
        if (rd_qp_incr && qpid_proc == C_NUM_QP - 1) begin
            timer_mem_rdaddr_comb = {{(LOG2_NUM_QP - 2) {1'b0}}, 2'b01};
        end else if (rd_qp_incr) begin
            timer_mem_rdaddr_comb = qpid_proc + 1;
        end else begin
            timer_mem_rdaddr_comb = qpid_proc;
        end
    end

    // timer_en 形成逻辑：按被扫描 qpid 读取 empty、timer_disabled 和 all_outstanding_resp_received 状态
    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP),
        .C_Init_with_1(1)
    ) inst_osq_empty_0 (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || osq_empty[C_QP_INDX_WIDTH+1]),
        .wr_addr (init_start ? init_count : osq_empty[C_QP_INDX_WIDTH : 1]),
        .wr_data (init_start ? 1'b1 : osq_empty[0]),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr_comb),
        .rd_data (osq_empty_rd_data_int_0)
    );
    assign osq_empty_rd_data_0 = (osq_empty[2*C_QP_INDX_WIDTH+2] && osq_empty[2*C_QP_INDX_WIDTH+1 : C_QP_INDX_WIDTH+2] == timer_mem_rdaddr_comb) ? 1'b0 : osq_empty_rd_data_int_0;

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_timer_disable_per_qp (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || timer_disabled_per_qp[2*C_QP_INDX_WIDTH+3]),
        .wr_addr (init_start ? init_count : timer_disabled_per_qp[2*C_QP_INDX_WIDTH+2 : C_QP_INDX_WIDTH+3]),
        .wr_data (init_start ? 1'b0 : timer_disabled_per_qp[C_QP_INDX_WIDTH+2]),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr_comb),
        .rd_data (timer_disabled_per_qp_rd_data_int)
    );
    assign timer_disabled_per_qp_rd_data = (timer_disabled_per_qp[C_QP_INDX_WIDTH+1] && timer_disabled_per_qp[C_QP_INDX_WIDTH : 1] == timer_mem_rdaddr_comb) ? timer_disabled_per_qp[0] : timer_disabled_per_qp_rd_data_int;

    lrdma_common_dual_wr_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_all_outstanding_resp_received (
        .core_clk   (core_clk),
        .core_rst   (~core_rstn),
        .wr_en      (init_start || all_outstanding_resp_received[2*C_QP_INDX_WIDTH+1]),
        .wr_addr    (init_start ? init_count : all_outstanding_resp_received[2*C_QP_INDX_WIDTH : C_QP_INDX_WIDTH+1]),
        .wr_data    (1'b0),
        .wr_en_aux  (all_outstanding_resp_received[C_QP_INDX_WIDTH]),
        .wr_addr_aux(all_outstanding_resp_received[C_QP_INDX_WIDTH-1 : 0]),
        .wr_data_aux(1'b1),
        .rd_en      (1'b1),
        .rd_addr    (timer_mem_rdaddr_comb),
        .rd_data    (all_outstanding_resp_received_rd_data)
    );

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_fatal (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_fatal[C_QP_INDX_WIDTH+1]),
        .wr_addr (init_start ? init_count : i_qp_fatal[C_QP_INDX_WIDTH : 1]),
        .wr_data (init_start ? 1'b0 : i_qp_fatal[0]),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr),
        .rd_data (qp_fatal_rd_data)
    );

    lrdma_common_sdp_lut_ram_wrapper #(
        .C_RAM_WORD_WIDTH(1),
        .C_RAM_DEPTH(C_NUM_QP)
    ) inst_i_qp_under_recovery (
        .core_clk(core_clk),
        .core_rst(~core_rstn),
        .wr_en   (init_start || i_qp_under_recovery_wr_en),
        .wr_addr (init_start ? init_count : i_qp_under_recovery_wr_idx),
        .wr_data (init_start ? 1'b0 : i_qp_under_recovery_wr_data),
        .rd_en   (1'b1),
        .rd_addr (timer_mem_rdaddr),
        .rd_data (qp_under_recovery_rd_data)
    );

    // timer_en = ~osq_empty && ~timer_disabled && ~all_outstanding_resp_received
    `MSFF_R(timer_en, (!osq_empty_rd_data_0 && !timer_disabled_per_qp_rd_data && !all_outstanding_resp_received_rd_data), core_clk, ~core_rstn)

    assign o_timer_mem_rdaddr     = timer_mem_rdaddr_comb;

    // reset_timer_osq_nacked 用于屏蔽被扫描 qpid 的计数递减/timeout 输出：
    // NACK sticky 或 OSQ empty 会打一拍形成 timer_reset；fatal/under_recovery 且该 qpid 不是已 NACK/timeout QP 时也会屏蔽。
    assign reset_timer_osq_nacked = (timer_reset || ((qp_fatal_rd_data || qp_under_recovery_rd_data) && !osq_nacked_timer_mem_rdaddr));
    `MSFF_R(timer_reset, (osq_nacked_ff_rd_data || osq_empty_rd_data_0), core_clk, !core_rstn)

    // 只有 timer_en 为 1 且没有 reset/mask 条件时，被扫描 QP 的 timer 才真正参与递减。
    assign timer_enable       = timer_en && !reset_timer_osq_nacked;

    // 计数递减后的值通过 B 端写回 timer RAM。同拍 A 端 reload 同一个 qpid 时跳过本次修改，避免覆盖新值。
    assign timer_mem_mod_wren = ((curr_st == WR_CNTR) & !(timer_mem_wren & (timer_mem_wraddr == qpid_proc))) & (timer_enable & !mem_b_rd_data.cntr_value_0);

    // MOD_CNTR 根据读出的计数和 trigger_count 计算下一次写回值。
    (*USE_DSP48 = "yes"*)

    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            mem_b_wr_data <= {MEM_DATA_WIDTH{1'b0}};
        end else if (curr_st == MOD_CNTR) begin
            // final_count=1 表示上一个扫描周期已经到期；这里把计数重新装载为初始值，等待下一轮判断。
            if (mem_b_rd_data.final_count) begin
                mem_b_wr_data.init_value    <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0  <= 1'b0;
                mem_b_wr_data.final_count   <= 1'b0;
                mem_b_wr_data.cntr_value    <= (1 << mem_b_rd_data.init_value);
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end  // if the last count is reached, then terminal count is set to
            // 到达最后一次计数时置 final_count，并把计数值清 0。
            else if (timer_enable && mem_b_rd_data.trigger_count == C_RES_VAL && (mem_b_rd_data.cntr_value <= (SUBTRACT_VAL << 3))) begin
                mem_b_wr_data.init_value    <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0  <= 1'b0;
                mem_b_wr_data.final_count   <= 1'b1;
                mem_b_wr_data.cntr_value    <= 32'h0;
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end else if (timer_enable && mem_b_rd_data.trigger_count != C_RES_VAL && trigger_count_4_1us > mem_b_rd_data.trigger_count && mem_b_rd_data.cntr_value <= (SUBTRACT_VAL * (trigger_count_4_1us - mem_b_rd_data.trigger_count))) begin
                mem_b_wr_data.init_value    <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0  <= 1'b0;
                mem_b_wr_data.final_count   <= 1'b1;
                mem_b_wr_data.cntr_value    <= 32'h0;
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end else if (timer_enable && mem_b_rd_data.trigger_count != C_RES_VAL && mem_b_rd_data.cntr_value <= (SUBTRACT_VAL * (8 + trigger_count_4_1us - mem_b_rd_data.trigger_count))) begin
                mem_b_wr_data.init_value    <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0  <= 1'b0;
                mem_b_wr_data.final_count   <= 1'b1;
                mem_b_wr_data.cntr_value    <= 32'h0;
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end  // 其它情况按 SUBTRACT_VAL 递减；若 reload 发生在本轮扫描中间，则按 trigger_count 差值补偿。
            else begin
                mem_b_wr_data.init_value   <= mem_b_rd_data.init_value;
                mem_b_wr_data.cntr_value_0 <= 1'b0;
                mem_b_wr_data.final_count  <= 1'b0;
                if (mem_b_rd_data.trigger_count != C_RES_VAL) begin  // reload 发生在本轮 32.768us 扫描周期内部，需要按子周期差值补偿
                    if (trigger_count_4_1us > mem_b_rd_data.trigger_count) begin
                        mem_b_wr_data.cntr_value <= mem_b_rd_data.cntr_value - (SUBTRACT_VAL * (trigger_count_4_1us - mem_b_rd_data.trigger_count));
                    end else begin
                        mem_b_wr_data.cntr_value <= mem_b_rd_data.cntr_value - (SUBTRACT_VAL * (8 + trigger_count_4_1us - mem_b_rd_data.trigger_count));
                    end
                end else begin
                    mem_b_wr_data.cntr_value <= mem_b_rd_data.cntr_value - (SUBTRACT_VAL << 3);
                end
                mem_b_wr_data.trigger_count <= C_RES_VAL;
            end
        end
    end

    always_ff @(posedge core_clk) begin
        if (!core_rstn) begin
            o_q_timed_out_valid   <= 1'b0;
            o_q_timed_out_wr_addr <= {LOG2_NUM_QP{1'b0}};
        end  // 当上一拍计算出的写回值已经回到初始计数值，说明该 QP 的 timeout 周期到期。
        // 若 timer_reload_pending=1，说明 reload 还在等待写入，暂不输出 timeout。
        else if (timer_enable & timer_mem_mod_wren & !mem_b_wr_data.final_count & (mem_b_wr_data.cntr_value == (1 << mem_b_wr_data.init_value))) begin
            o_q_timed_out_valid   <= !timer_reload_pending;
            //o_q_timed_out_valid <= 1'b1;
            o_q_timed_out_wr_addr <= timer_mem_rdaddr;
        end else begin
            o_q_timed_out_valid <= 1'b0;
        end
    end

endmodule  // lrdma_ack_retry_timer
