/* SPDX-License-Identifier: MIT
 * BAR0 direct-CQ register block for software-scheduled multi-queue SGDMA.
 *
 * 512-queue scalable implementation:
 *   - no 512-wide priority encoder/round-robin scan;
 *   - host doorbells push qids into a pending FIFO;
 *   - no per-queue pointer mirror arrays; only pending/busy/resched bitmaps remain in flops;
 *   - scheduler-facing active context is stored in one compact XPM RAM;
 *   - slow/status/performance register space is stored in a second XPM RAM;
 *   - queue BAR aperture keeps the old 0x100 stride, and BAR0 decoding masks
 *     the CQ full PCIe address down to a BAR-local offset.
 */

`resetall 
`timescale 1ns / 1ps
`default_nettype none

`include "edev_sgdma_defs.vh"

module edev_sgdma_regs #(
    parameter AXIS_PCIE_DATA_WIDTH = 256,
    parameter AXIS_PCIE_KEEP_WIDTH = (AXIS_PCIE_DATA_WIDTH / 32),
    parameter AXIS_PCIE_CQ_USER_WIDTH = 88,
    parameter AXIS_PCIE_CC_USER_WIDTH = 33,
    parameter RING_INDEX_WIDTH = 16,
    parameter MAX_QUEUES = 512,
    parameter QID_WIDTH = 9,
    parameter AXI_BR_TABLE_ENTRIES = 4,
    // BAR0 register decode uses an offset inside the BAR aperture, not the
    // full PCIe address delivered in the CQ descriptor. 512 queues require
    // 0x1000 + 512*0x100 = 0x21000 bytes, so the default 18-bit window
    // covers a 256 KiB BAR0 aperture.
    parameter BAR0_ADDR_WIDTH = 18,
    parameter PENDING_FIFO_ADDR_WIDTH = QID_WIDTH,
    parameter PENDING_FIFO_DEPTH = (1 << PENDING_FIFO_ADDR_WIDTH),
    parameter CAP_VALUE = 32'h0051_1000,
    parameter ACTIVE_CTX_READ_LATENCY = 2,
    parameter SHADOW_CTX_READ_LATENCY = 2,
    parameter CTX_DATA_WIDTH = QID_WIDTH + 352,
    parameter CTX_UPDATE_WIDTH = 4 + QID_WIDTH + RING_INDEX_WIDTH * 2 + 32 * 6
) (
    input wire clk,
    input wire rst,

    input  wire [   AXIS_PCIE_DATA_WIDTH-1:0] s_axis_cq_tdata,
    input  wire [   AXIS_PCIE_KEEP_WIDTH-1:0] s_axis_cq_tkeep,
    input  wire                               s_axis_cq_tvalid,
    output wire                               s_axis_cq_tready,
    input  wire                               s_axis_cq_tlast,
    input  wire [AXIS_PCIE_CQ_USER_WIDTH-1:0] s_axis_cq_tuser,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_cc_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_cc_tkeep,
    output wire                               m_axis_cc_tvalid,
    input  wire                               m_axis_cc_tready,
    output wire                               m_axis_cc_tlast,
    output wire [AXIS_PCIE_CC_USER_WIDTH-1:0] m_axis_cc_tuser,

    input wire [15:0] completer_id,
    input wire        completer_id_enable,

    output wire               global_enable,
    output wire               global_reset,
    output wire               global_pause,
    output wire [       31:0] global_mode,
    output wire [       31:0] global_irq_enable,
    output wire [QID_WIDTH:0] enabled_queue_count,

    output wire                      ctx_valid,
    input  wire                      ctx_ready,
    output wire [CTX_DATA_WIDTH-1:0] ctx_data,

    input wire                        ctx_update_valid,
    input wire [CTX_UPDATE_WIDTH-1:0] ctx_update_data,
    input wire                        sched_busy,
    input wire [       QID_WIDTH-1:0] sched_active_qid,

    input  wire [31:0] irq_status_set,
    output wire [31:0] irq_status,

    output wire [AXI_BR_TABLE_ENTRIES-1:0] axi_bridge_table_enable,
    output wire [AXI_BR_TABLE_ENTRIES*64-1:0] axi_bridge_table_axi_base,
    output wire [AXI_BR_TABLE_ENTRIES*64-1:0] axi_bridge_table_pcie_base,
    output wire [AXI_BR_TABLE_ENTRIES*64-1:0] axi_bridge_table_mask,

    output wire status_error_cor,
    output wire status_error_uncor
);

    localparam integer BAR0_REQUIRED_BYTES = `EDEV_REG_Q_BASE + (MAX_QUEUES << 8);
    localparam [QID_WIDTH:0] MAX_QUEUE_COUNT_VALUE = MAX_QUEUES;
    localparam [31:0] BAR0_ADDR_MASK = 32'hffff_ffff >> (32 - BAR0_ADDR_WIDTH);
    localparam [4:0] AXI_BR_OFF_CTRL         = `EDEV_AXI_BR_CTRL;
    localparam [4:0] AXI_BR_OFF_AXI_BASE_LO  = `EDEV_AXI_BR_AXI_BASE_LO;
    localparam [4:0] AXI_BR_OFF_AXI_BASE_HI  = `EDEV_AXI_BR_AXI_BASE_HI;
    localparam [4:0] AXI_BR_OFF_PCIE_BASE_LO = `EDEV_AXI_BR_PCIE_BASE_LO;
    localparam [4:0] AXI_BR_OFF_PCIE_BASE_HI = `EDEV_AXI_BR_PCIE_BASE_HI;
    localparam [4:0] AXI_BR_OFF_MASK_LO      = `EDEV_AXI_BR_MASK_LO;
    localparam [4:0] AXI_BR_OFF_MASK_HI      = `EDEV_AXI_BR_MASK_HI;

    initial begin
        if (AXIS_PCIE_DATA_WIDTH != 256) begin
            $error("edev_sgdma_regs expects 256-bit PCIe AXIS interface (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_KEEP_WIDTH != 8) begin
            $error("edev_sgdma_regs expects 8-bit tkeep for 256-bit interface (instance %m)");
            $finish;
        end
        if (MAX_QUEUES > (1 << QID_WIDTH)) begin
            $error("MAX_QUEUES exceeds QID_WIDTH capacity (instance %m)");
            $finish;
        end
        if (PENDING_FIFO_DEPTH < MAX_QUEUES) begin
            $error("PENDING_FIFO_DEPTH must be at least MAX_QUEUES (instance %m)");
            $finish;
        end
        if (BAR0_ADDR_WIDTH < 8 || BAR0_ADDR_WIDTH > 32) begin
            $error("BAR0_ADDR_WIDTH must be in the range 8..32 (instance %m)");
            $finish;
        end
        if ((64'd1 << BAR0_ADDR_WIDTH) < BAR0_REQUIRED_BYTES) begin
            $error("BAR0_ADDR_WIDTH is too small for the configured queue aperture (instance %m)");
            $finish;
        end
        if (AXI_BR_TABLE_ENTRIES != 4) begin
            $error("edev_sgdma_regs currently exposes four AXI bridge translation entries (instance %m)");
            $finish;
        end
    end

    localparam [3:0] REQ_MEM_READ = 4'b0000;
    localparam [3:0] REQ_MEM_WRITE = 4'b0001;
    localparam [3:0] REQ_IO_READ = 4'b0010;
    localparam [3:0] REQ_IO_WRITE = 4'b0011;

    localparam [2:0] CPL_STATUS_SC = 3'b000;
    localparam [2:0] CPL_STATUS_UR = 3'b001;
    localparam [2:0] CPL_STATUS_CA = 3'b100;

    localparam [1:0] STATE_IDLE = 2'd0;
    localparam [1:0] STATE_WAIT_END = 2'd1;
    localparam [1:0] STATE_QREAD_WAIT = 2'd2;

    // Compact active-context RAM: only the fields repeatedly consumed by the scheduler.
    localparam integer ACT_WORD_CTRL = 0;
    localparam integer ACT_WORD_MODE = 1;
    localparam integer ACT_WORD_IRQ_ENABLE = 2;
    localparam integer ACT_WORD_DESC_BASE_LO = 3;
    localparam integer ACT_WORD_DESC_BASE_HI = 4;
    localparam integer ACT_WORD_CMPL_BASE_LO = 5;
    localparam integer ACT_WORD_CMPL_BASE_HI = 6;
    localparam integer ACT_WORD_RING_SIZE = 7;
    localparam integer ACT_WORD_DESC_PIDX = 8;
    localparam integer ACT_WORD_DESC_CIDX = 9;
    localparam integer ACT_WORD_CMPL_PIDX = 10;
    localparam integer ACTIVE_CTX_WORDS = 11;
    localparam integer ACTIVE_CTX_WIDTH = ACTIVE_CTX_WORDS * 32;
    localparam integer ACT_RAM_WORD_CTRL = 0;
    localparam integer ACT_RAM_WORD_MODE = 1;
    localparam integer ACT_RAM_WORD_IRQ_ENABLE = 2;
    localparam integer ACT_RAM_WORD_DESC_BASE_LO = 3;
    localparam integer ACT_RAM_WORD_DESC_BASE_HI = 4;
    localparam integer ACT_RAM_WORD_CMPL_BASE_LO = 5;
    localparam integer ACT_RAM_WORD_CMPL_BASE_HI = 6;
    localparam integer ACT_RAM_WORD_RING_SIZE = 7;
    localparam integer ACT_RAM_WORD_DESC_CIDX = 8;
    localparam integer ACT_RAM_WORD_CMPL_PIDX = 9;
    localparam integer ACTIVE_RAM_WORDS = 10;
    localparam integer ACTIVE_RAM_WIDTH = ACTIVE_RAM_WORDS * 32;
    localparam integer ACTIVE_RAM_BYTES = ACTIVE_RAM_WIDTH / 8;

    // Slow shadow/status RAM: fields that do not participate in the hot scheduler path.
    localparam integer SH_WORD_STATUS = 0;
    localparam integer SH_WORD_ERROR_CODE = 1;
    localparam integer SH_WORD_PERF_DESC_FETCH = 2;
    localparam integer SH_WORD_PERF_DMA_SUBMIT = 3;
    localparam integer SH_WORD_PERF_WB_WRITE = 4;
    localparam integer SH_WORD_PERF_BYTES = 5;
    localparam integer SHADOW_CTX_WORDS = 6;
    localparam integer SHADOW_CTX_WIDTH = SHADOW_CTX_WORDS * 32;
    localparam integer SHADOW_CTX_BYTES = SHADOW_CTX_WIDTH / 8;

    // Packed scheduler context bus.  Keep qid in the low bits so the engine can
    // unpack with constant part-selects and no per-field top-level ports.
    localparam integer CTX_LSB_QID = 0;
    localparam integer CTX_LSB_ACTIVE = CTX_LSB_QID + QID_WIDTH;

    // Packed scheduler update bus.
    localparam integer UPD_LSB_OP_DONE = 0;
    localparam integer UPD_LSB_REQUEUE = 1;
    localparam integer UPD_LSB_DESC_WE = 2;
    localparam integer UPD_LSB_CMPL_WE = 3;
    localparam integer UPD_LSB_QID = 4;
    localparam integer UPD_LSB_DESC_CIDX = UPD_LSB_QID + QID_WIDTH;
    localparam integer UPD_LSB_CMPL_PIDX = UPD_LSB_DESC_CIDX + RING_INDEX_WIDTH;
    localparam integer UPD_LSB_STATUS = UPD_LSB_CMPL_PIDX + RING_INDEX_WIDTH;
    localparam integer UPD_LSB_ERROR_CODE = UPD_LSB_STATUS + 32;
    localparam integer UPD_LSB_PERF_DESC = UPD_LSB_ERROR_CODE + 32;
    localparam integer UPD_LSB_PERF_DMA = UPD_LSB_PERF_DESC + 32;
    localparam integer UPD_LSB_PERF_WB = UPD_LSB_PERF_DMA + 32;
    localparam integer UPD_LSB_PERF_BYTES = UPD_LSB_PERF_WB + 32;

    // Local synchronous reset for context-RAM output registers. The global
    // clear sweep blocks CQ/ctx activity until every queue is initialized,
    // so one extra reset cycle cannot expose stale RAM output.
    (* max_fanout = 64 *) reg context_ram_reset_reg = 1'b1;
    always @(posedge clk) context_ram_reset_reg <= rst;

    reg [                        1:0] state_reg = STATE_IDLE;

    reg [                       31:0] global_ctrl_reg = 32'd0;
    reg [                       31:0] global_mode_reg = 32'd0;
    reg [                       31:0] global_irq_enable_reg = 32'd0;
    reg [                QID_WIDTH:0] enabled_queue_count_reg = MAX_QUEUE_COUNT_VALUE;
    reg [                       31:0] irq_status_reg = 32'd0;
    reg [                       31:0] global_error_code_reg = 32'd0;

    reg [AXI_BR_TABLE_ENTRIES-1:0] axi_bridge_table_enable_reg = {AXI_BR_TABLE_ENTRIES{1'b0}};
    reg [AXI_BR_TABLE_ENTRIES*64-1:0] axi_bridge_table_axi_base_reg = {(AXI_BR_TABLE_ENTRIES*64){1'b0}};
    reg [AXI_BR_TABLE_ENTRIES*64-1:0] axi_bridge_table_pcie_base_reg = {(AXI_BR_TABLE_ENTRIES*64){1'b0}};
    reg [AXI_BR_TABLE_ENTRIES*64-1:0] axi_bridge_table_mask_reg = {(AXI_BR_TABLE_ENTRIES*64){1'b0}};

    // Queue scheduler metadata only.  Full queue context is stored in RAM; keep
    // only one-bit state vectors to suppress duplicate pending FIFO entries.
    reg [             MAX_QUEUES-1:0] q_pending_reg = {MAX_QUEUES{1'b0}};
    reg [             MAX_QUEUES-1:0] q_busy_reg = {MAX_QUEUES{1'b0}};
    reg [             MAX_QUEUES-1:0] q_resched_reg = {MAX_QUEUES{1'b0}};

    // DESC_PIDX is written by the host while DESC_CIDX/CMPL_PIDX are written
    // by the scheduler.  Keeping PIDX outside the wide active-context RAM
    // removes same-row dual-port write collisions without keeping a 512x16 FF
    // array in the register block.
    localparam integer DESC_PIDX_RAM_WIDTH = 32;
    localparam integer DESC_PIDX_RAM_BYTES = DESC_PIDX_RAM_WIDTH / 8;

    // Pending FIFO replaces the 512-wide RR priority encoder.
    reg [              QID_WIDTH-1:0] pending_fifo_mem                                                [0:PENDING_FIFO_DEPTH-1];
    reg [PENDING_FIFO_ADDR_WIDTH-1:0] pending_fifo_wr_ptr_reg = {PENDING_FIFO_ADDR_WIDTH{1'b0}};
    reg [PENDING_FIFO_ADDR_WIDTH-1:0] pending_fifo_rd_ptr_reg = {PENDING_FIFO_ADDR_WIDTH{1'b0}};
    reg [  PENDING_FIFO_ADDR_WIDTH:0] pending_fifo_count_reg = {(PENDING_FIFO_ADDR_WIDTH + 1) {1'b0}};
    localparam [PENDING_FIFO_ADDR_WIDTH:0] PENDING_FIFO_DEPTH_COUNT = PENDING_FIFO_DEPTH;
    wire                               pending_fifo_empty = pending_fifo_count_reg == 0;
    wire                               pending_fifo_full = pending_fifo_count_reg == PENDING_FIFO_DEPTH_COUNT;

    reg                                pf_push;
    reg                                pf_pop;
    reg  [              QID_WIDTH-1:0] pf_push_qid;
    reg  [              QID_WIDTH-1:0] pf_pop_qid;

    // Active-context RAM host port A.
    reg  [              QID_WIDTH-1:0] active_a_addr_reg = {QID_WIDTH{1'b0}};
    reg                                active_a_en_reg = 1'b0;
    reg  [       ACTIVE_RAM_BYTES-1:0] active_a_we_reg = {ACTIVE_RAM_BYTES{1'b0}};
    reg  [       ACTIVE_RAM_WIDTH-1:0] active_a_din_reg = {ACTIVE_RAM_WIDTH{1'b0}};
    wire [       ACTIVE_RAM_WIDTH-1:0] active_a_dout;

    // Active-context RAM scheduler port B.
    reg  [              QID_WIDTH-1:0] active_b_addr_reg = {QID_WIDTH{1'b0}};
    reg                                active_b_en_reg = 1'b0;
    reg  [       ACTIVE_RAM_BYTES-1:0] active_b_we_reg = {ACTIVE_RAM_BYTES{1'b0}};
    reg  [       ACTIVE_RAM_WIDTH-1:0] active_b_din_reg = {ACTIVE_RAM_WIDTH{1'b0}};
    wire [       ACTIVE_RAM_WIDTH-1:0] active_b_dout;

    // DESC_PIDX RAM: port A is used by host/qread, port B by scheduler ctx fetch
    // and global clear scan.
    reg  [              QID_WIDTH-1:0] desc_pidx_a_addr_reg = {QID_WIDTH{1'b0}};
    reg                                desc_pidx_a_en_reg = 1'b0;
    reg  [    DESC_PIDX_RAM_BYTES-1:0] desc_pidx_a_we_reg = {DESC_PIDX_RAM_BYTES{1'b0}};
    reg  [    DESC_PIDX_RAM_WIDTH-1:0] desc_pidx_a_din_reg = {DESC_PIDX_RAM_WIDTH{1'b0}};
    wire [    DESC_PIDX_RAM_WIDTH-1:0] desc_pidx_a_dout;

    reg  [              QID_WIDTH-1:0] desc_pidx_b_addr_reg = {QID_WIDTH{1'b0}};
    reg                                desc_pidx_b_en_reg = 1'b0;
    reg  [    DESC_PIDX_RAM_BYTES-1:0] desc_pidx_b_we_reg = {DESC_PIDX_RAM_BYTES{1'b0}};
    reg  [    DESC_PIDX_RAM_WIDTH-1:0] desc_pidx_b_din_reg = {DESC_PIDX_RAM_WIDTH{1'b0}};
    wire [    DESC_PIDX_RAM_WIDTH-1:0] desc_pidx_b_dout;

    // Shadow RAM host port A.
    reg  [              QID_WIDTH-1:0] shadow_a_addr_reg = {QID_WIDTH{1'b0}};
    reg                                shadow_a_en_reg = 1'b0;
    reg  [       SHADOW_CTX_BYTES-1:0] shadow_a_we_reg = {SHADOW_CTX_BYTES{1'b0}};
    reg  [       SHADOW_CTX_WIDTH-1:0] shadow_a_din_reg = {SHADOW_CTX_WIDTH{1'b0}};
    wire [       SHADOW_CTX_WIDTH-1:0] shadow_a_dout;

    // Shadow RAM scheduler port B.
    reg  [              QID_WIDTH-1:0] shadow_b_addr_reg = {QID_WIDTH{1'b0}};
    reg                                shadow_b_en_reg = 1'b0;
    reg  [       SHADOW_CTX_BYTES-1:0] shadow_b_we_reg = {SHADOW_CTX_BYTES{1'b0}};
    reg  [       SHADOW_CTX_WIDTH-1:0] shadow_b_din_reg = {SHADOW_CTX_WIDTH{1'b0}};
    wire [       SHADOW_CTX_WIDTH-1:0] shadow_b_dout;

    // Scheduler context-read pipeline.
    reg                                ctx_fetch_busy_reg = 1'b0;
    reg  [              QID_WIDTH-1:0] ctx_fetch_qid_reg = {QID_WIDTH{1'b0}};
    reg  [                        3:0] ctx_fetch_count_reg = 4'd0;
    reg                                ctx_hold_valid_reg = 1'b0;
    reg  [         CTX_DATA_WIDTH-1:0] ctx_hold_data_reg = {CTX_DATA_WIDTH{1'b0}};

    // Host queue-read pipeline.
    reg  [                        3:0] qread_count_reg = 4'd0;
    reg  [              QID_WIDTH-1:0] qread_qid_reg = {QID_WIDTH{1'b0}};
    reg  [                        7:0] qread_qoff_reg = 8'd0;
    reg  [                       15:0] qread_requester_id_reg = 16'd0;
    reg  [                        7:0] qread_tag_reg = 8'd0;
    reg  [                        2:0] qread_tc_reg = 3'd0;
    reg  [                        2:0] qread_attr_reg = 3'd0;
    reg  [                        3:0] qread_first_be_reg = 4'hf;
    reg  [                       31:0] qread_addr_reg = 32'd0;

    reg  [   AXIS_PCIE_DATA_WIDTH-1:0] cc_tdata_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg  [   AXIS_PCIE_KEEP_WIDTH-1:0] cc_tkeep_reg = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
    reg                                cc_tvalid_reg = 1'b0;
    reg                                cc_tlast_reg = 1'b0;
    reg  [AXIS_PCIE_CC_USER_WIDTH-1:0] cc_tuser_reg = {AXIS_PCIE_CC_USER_WIDTH{1'b0}};

    reg                                status_error_cor_reg = 1'b0;
    reg                                status_error_uncor_reg = 1'b0;
    reg                                wait_error_cpl_reg = 1'b0;
    reg  [                        2:0] wait_cpl_status_reg = CPL_STATUS_UR;
    reg  [                       15:0] wait_requester_id_reg = 16'd0;
    reg  [                        7:0] wait_tag_reg = 8'd0;
    reg  [                        2:0] wait_tc_reg = 3'd0;
    reg  [                        2:0] wait_attr_reg = 3'd0;
    reg  [                        3:0] wait_first_be_reg = 4'hf;
    reg  [                       31:0] wait_addr_reg = 32'd0;

    reg                                clear_active_reg = 1'b0;
    reg  [              QID_WIDTH-1:0] clear_qid_reg = {QID_WIDTH{1'b0}};

    // Decode accepted CQ address before the 512-way queue bitmap selection.
    reg cq_req_valid_reg = 1'b0;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] cq_req_data_reg = 0;
    reg [AXIS_PCIE_CQ_USER_WIDTH-1:0] cq_req_user_reg = 0;
    reg cq_req_last_reg = 1'b0;
    reg [QID_WIDTH-1:0] cq_qid_reg = 0;
    reg [7:0] cq_qoff_reg = 0;
    wire [31:0] cq_input_addr = s_axis_cq_tdata[31:0] & BAR0_ADDR_MASK;
    // Predecode the accepted doorbell alongside the CQ payload. A queue-local
    // doorbell carries its QID in the address; the global doorbell carries it
    // in the first payload dword. Validity stays owned by cq_req_valid_reg.
    wire cq_input_is_queue = (cq_input_addr >= `EDEV_REG_Q_BASE) &&
                             (cq_input_addr < (`EDEV_REG_Q_BASE + MAX_QUEUES*256));
    wire [QID_WIDTH-1:0] cq_input_enqueue_qid = cq_input_is_queue ?
        ((cq_input_addr-`EDEV_REG_Q_BASE) >> 8) : s_axis_cq_tdata[128+:QID_WIDTH];
    reg [MAX_QUEUES-1:0] cq_enqueue_onehot;
    genvar host_queue;
    generate for (host_queue=0; host_queue<MAX_QUEUES; host_queue=host_queue+1) begin: host_qid_decode
        always @(posedge clk)
            if (s_axis_cq_tvalid && s_axis_cq_tready)
                cq_enqueue_onehot[host_queue] <= (cq_input_enqueue_qid == host_queue);
    end endgenerate
    // Capture operation decoding with the accepted CQ payload. The held
    // request cannot be overtaken by another CQ write to queue-count control.
    // This removes repeated header/range comparators from bitmap D inputs.
    reg cq_read_type_reg=1'b0, cq_write_type_reg=1'b0;
    reg cq_single_dword_reg=1'b0, cq_queue_addr_reg=1'b0;
    reg cq_enqueue_allowed_reg=1'b0;
    always @(posedge clk) begin
        if (s_axis_cq_tvalid && s_axis_cq_tready) begin
            cq_read_type_reg <= (s_axis_cq_tdata[78:75] == REQ_MEM_READ || s_axis_cq_tdata[78:75] == REQ_IO_READ);
            cq_write_type_reg <= (s_axis_cq_tdata[78:75] == REQ_MEM_WRITE || s_axis_cq_tdata[78:75] == REQ_IO_WRITE);
            cq_single_dword_reg <= s_axis_cq_tlast && s_axis_cq_tdata[74:64] == 11'd1;
            cq_queue_addr_reg <= cq_input_is_queue;
            cq_enqueue_allowed_reg <= ({1'b0,cq_input_enqueue_qid} < enabled_queue_count_reg);
        end
    end
    wire cq_service_ready;
    wire cq_exec_fire = cq_req_valid_reg && cq_service_ready;

    // Snapshot the 512-way status selection ahead of completion formatting.
    // The existing RAM-read wait provides time for the selected QID to settle.
    reg qread_busy_pipe_reg = 1'b0;
    always @(posedge clk) begin
        if (rst) qread_busy_pipe_reg <= 1'b0;
        else qread_busy_pipe_reg <= q_pending_reg[qread_qid_reg] |
                                    q_busy_reg[qread_qid_reg] |
                                    q_resched_reg[qread_qid_reg];
    end

    // Register the scheduler update and predecode its QID before touching the
    // 512-bit queue-state arrays. Preserve raw+pending interlocks: no host
    // request or new context fetch can race the delayed completion write.
    (* max_fanout = 32 *) reg ctx_update_pipe_valid = 1'b0;
    reg [CTX_UPDATE_WIDTH-1:0] ctx_update_pipe_data;
    reg [MAX_QUEUES-1:0] ctx_update_onehot;
    genvar update_queue;
    generate for (update_queue=0; update_queue<MAX_QUEUES; update_queue=update_queue+1) begin: update_qid_decode
        always @(posedge clk)
            ctx_update_onehot[update_queue] <=
                (ctx_update_data[UPD_LSB_QID+:QID_WIDTH] == update_queue);
    end endgenerate
    always @(posedge clk) begin
        // Payload has no CE/reset fanout; only valid grants ownership.
        ctx_update_pipe_data <= ctx_update_data;
        if (rst || global_ctrl_reg[`EDEV_CTRL_RESET] || clear_active_reg)
            ctx_update_pipe_valid <= 1'b0;
        else ctx_update_pipe_valid <= ctx_update_valid;
    end

    // Reuse registered one-hot decoders for selected state queries. Each
    // local state bit enters one reduction tree instead of broadcasting a
    // nine-bit selector through several independent 512-to-1 muxes.
    wire selected_host_busy = |(cq_enqueue_onehot & (q_pending_reg | q_busy_reg));
    wire selected_update_pending = |(ctx_update_onehot & q_pending_reg);
    wire selected_update_resched = |(ctx_update_onehot & q_resched_reg);

    wire                               ctx_fire = ctx_valid && ctx_ready;
    /*
     * Do not read a queue context in the cycle immediately following a BAR0
     * context write.  Doorbell writes update DESC_PIDX through active port A and
     * enqueue the qid in the same clock domain; without this guard port B can
     * fetch the old context image and hand the scheduler a zero-pending op,
     * leaving the queue busy without issuing descriptor MRds.
     */
    wire                               active_write_idle = !active_a_en_reg && !shadow_a_en_reg &&
                                                            !active_b_en_reg && !shadow_b_en_reg &&
                                                            !desc_pidx_a_en_reg && !desc_pidx_b_en_reg;
    wire                               ctx_can_start_fetch = global_ctrl_reg[`EDEV_CTRL_ENABLE] && !global_ctrl_reg[`EDEV_CTRL_PAUSE] &&
                                                             !ctx_hold_valid_reg && !ctx_fetch_busy_reg && !pending_fifo_empty &&
                                                             !clear_active_reg && !ctx_update_valid && !ctx_update_pipe_valid && active_write_idle &&
                                                             !cq_req_valid_reg;

    // A ready scheduler fetch wins before accepting the next CQ beat.
    // Unlike gating fetch on host TVALID, this permits progress under a
    // continuously asserted host request stream and prevents same-edge
    // context read / new host write acceptance races.
    assign cq_service_ready  = (state_reg == STATE_IDLE) ? (!cc_tvalid_reg && !active_a_en_reg && !shadow_a_en_reg && !desc_pidx_a_en_reg && !ctx_update_valid && !ctx_update_pipe_valid && !clear_active_reg && !ctx_can_start_fetch) : (state_reg == STATE_WAIT_END);
    assign s_axis_cq_tready = cq_service_ready && !cq_req_valid_reg;
    assign m_axis_cc_tdata     = cc_tdata_reg;
    assign m_axis_cc_tkeep     = cc_tkeep_reg;
    assign m_axis_cc_tvalid    = cc_tvalid_reg;
    assign m_axis_cc_tlast     = cc_tlast_reg;
    assign m_axis_cc_tuser     = cc_tuser_reg;

    assign global_enable       = global_ctrl_reg[`EDEV_CTRL_ENABLE];
    assign global_reset        = global_ctrl_reg[`EDEV_CTRL_RESET];
    assign global_pause        = global_ctrl_reg[`EDEV_CTRL_PAUSE];
    assign global_mode         = global_mode_reg;
    assign global_irq_enable   = global_irq_enable_reg;
    assign enabled_queue_count = enabled_queue_count_reg;

    assign ctx_valid           = ctx_hold_valid_reg;
    assign ctx_data            = ctx_hold_data_reg;

    assign irq_status          = irq_status_reg;
    assign axi_bridge_table_enable = axi_bridge_table_enable_reg;
    assign axi_bridge_table_axi_base = axi_bridge_table_axi_base_reg;
    assign axi_bridge_table_pcie_base = axi_bridge_table_pcie_base_reg;
    assign axi_bridge_table_mask = axi_bridge_table_mask_reg;
    assign status_error_cor    = status_error_cor_reg;
    assign status_error_uncor  = status_error_uncor_reg;

    xpm_memory_tdpram #(
        .ADDR_WIDTH_A(QID_WIDTH),
        .ADDR_WIDTH_B(QID_WIDTH),
        .AUTO_SLEEP_TIME(0),
        .BYTE_WRITE_WIDTH_A(8),
        .BYTE_WRITE_WIDTH_B(8),
        .CASCADE_HEIGHT(0),
        .CLOCKING_MODE("common_clock"),
        .ECC_MODE("no_ecc"),
        .MEMORY_INIT_FILE("none"),
        .MEMORY_INIT_PARAM("0"),
        .MEMORY_OPTIMIZATION("true"),
        .MEMORY_PRIMITIVE("block"),
        .MEMORY_SIZE(ACTIVE_RAM_WIDTH * MAX_QUEUES),
        .MESSAGE_CONTROL(0),
        .READ_DATA_WIDTH_A(ACTIVE_RAM_WIDTH),
        .READ_DATA_WIDTH_B(ACTIVE_RAM_WIDTH),
        .READ_LATENCY_A(ACTIVE_CTX_READ_LATENCY),
        .READ_LATENCY_B(ACTIVE_CTX_READ_LATENCY),
        .READ_RESET_VALUE_A("0"),
        .READ_RESET_VALUE_B("0"),
        .RST_MODE_A("SYNC"),
        .RST_MODE_B("SYNC"),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT(0),
        .USE_MEM_INIT_MMI(0),
        .WAKEUP_TIME("disable_sleep"),
        .WRITE_DATA_WIDTH_A(ACTIVE_RAM_WIDTH),
        .WRITE_DATA_WIDTH_B(ACTIVE_RAM_WIDTH),
        .WRITE_MODE_A("no_change"),
        .WRITE_MODE_B("no_change")
    ) active_context_ram_inst (
        .clka          (clk),
        .rsta          (context_ram_reset_reg),
        .ena           (active_a_en_reg),
        .regcea        (1'b1),
        .wea           (active_a_we_reg),
        .addra         (active_a_addr_reg),
        .dina          (active_a_din_reg),
        .douta         (active_a_dout),
        .clkb          (clk),
        .rstb          (context_ram_reset_reg),
        .enb           (active_b_en_reg),
        .regceb        (1'b1),
        .web           (active_b_we_reg),
        .addrb         (active_b_addr_reg),
        .dinb          (active_b_din_reg),
        .doutb         (active_b_dout),
        .sleep         (1'b0),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .sbiterra      (),
        .dbiterra      (),
        .sbiterrb      (),
        .dbiterrb      ()
    );

    xpm_memory_tdpram #(
        .ADDR_WIDTH_A(QID_WIDTH),
        .ADDR_WIDTH_B(QID_WIDTH),
        .AUTO_SLEEP_TIME(0),
        .BYTE_WRITE_WIDTH_A(8),
        .BYTE_WRITE_WIDTH_B(8),
        .CASCADE_HEIGHT(0),
        .CLOCKING_MODE("common_clock"),
        .ECC_MODE("no_ecc"),
        .MEMORY_INIT_FILE("none"),
        .MEMORY_INIT_PARAM("0"),
        .MEMORY_OPTIMIZATION("true"),
        .MEMORY_PRIMITIVE("block"),
        .MEMORY_SIZE(DESC_PIDX_RAM_WIDTH * MAX_QUEUES),
        .MESSAGE_CONTROL(0),
        .READ_DATA_WIDTH_A(DESC_PIDX_RAM_WIDTH),
        .READ_DATA_WIDTH_B(DESC_PIDX_RAM_WIDTH),
        .READ_LATENCY_A(ACTIVE_CTX_READ_LATENCY),
        .READ_LATENCY_B(ACTIVE_CTX_READ_LATENCY),
        .READ_RESET_VALUE_A("0"),
        .READ_RESET_VALUE_B("0"),
        .RST_MODE_A("SYNC"),
        .RST_MODE_B("SYNC"),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT(0),
        .USE_MEM_INIT_MMI(0),
        .WAKEUP_TIME("disable_sleep"),
        .WRITE_DATA_WIDTH_A(DESC_PIDX_RAM_WIDTH),
        .WRITE_DATA_WIDTH_B(DESC_PIDX_RAM_WIDTH),
        .WRITE_MODE_A("no_change"),
        .WRITE_MODE_B("no_change")
    ) desc_pidx_ram_inst (
        .clka          (clk),
        .rsta          (context_ram_reset_reg),
        .ena           (desc_pidx_a_en_reg),
        .regcea        (1'b1),
        .wea           (desc_pidx_a_we_reg),
        .addra         (desc_pidx_a_addr_reg),
        .dina          (desc_pidx_a_din_reg),
        .douta         (desc_pidx_a_dout),
        .clkb          (clk),
        .rstb          (context_ram_reset_reg),
        .enb           (desc_pidx_b_en_reg),
        .regceb        (1'b1),
        .web           (desc_pidx_b_we_reg),
        .addrb         (desc_pidx_b_addr_reg),
        .dinb          (desc_pidx_b_din_reg),
        .doutb         (desc_pidx_b_dout),
        .sleep         (1'b0),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .sbiterra      (),
        .dbiterra      (),
        .sbiterrb      (),
        .dbiterrb      ()
    );

    xpm_memory_tdpram #(
        .ADDR_WIDTH_A(QID_WIDTH),
        .ADDR_WIDTH_B(QID_WIDTH),
        .AUTO_SLEEP_TIME(0),
        .BYTE_WRITE_WIDTH_A(8),
        .BYTE_WRITE_WIDTH_B(8),
        .CASCADE_HEIGHT(0),
        .CLOCKING_MODE("common_clock"),
        .ECC_MODE("no_ecc"),
        .MEMORY_INIT_FILE("none"),
        .MEMORY_INIT_PARAM("0"),
        .MEMORY_OPTIMIZATION("true"),
        .MEMORY_PRIMITIVE("block"),
        .MEMORY_SIZE(SHADOW_CTX_WIDTH * MAX_QUEUES),
        .MESSAGE_CONTROL(0),
        .READ_DATA_WIDTH_A(SHADOW_CTX_WIDTH),
        .READ_DATA_WIDTH_B(SHADOW_CTX_WIDTH),
        .READ_LATENCY_A(SHADOW_CTX_READ_LATENCY),
        .READ_LATENCY_B(SHADOW_CTX_READ_LATENCY),
        .READ_RESET_VALUE_A("0"),
        .READ_RESET_VALUE_B("0"),
        .RST_MODE_A("SYNC"),
        .RST_MODE_B("SYNC"),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT(0),
        .USE_MEM_INIT_MMI(0),
        .WAKEUP_TIME("disable_sleep"),
        .WRITE_DATA_WIDTH_A(SHADOW_CTX_WIDTH),
        .WRITE_DATA_WIDTH_B(SHADOW_CTX_WIDTH),
        .WRITE_MODE_A("no_change"),
        .WRITE_MODE_B("no_change")
    ) shadow_context_ram_inst (
        .clka          (clk),
        .rsta          (context_ram_reset_reg),
        .ena           (shadow_a_en_reg),
        .regcea        (1'b1),
        .wea           (shadow_a_we_reg),
        .addra         (shadow_a_addr_reg),
        .dina          (shadow_a_din_reg),
        .douta         (shadow_a_dout),
        .clkb          (clk),
        .rstb          (context_ram_reset_reg),
        .enb           (shadow_b_en_reg),
        .regceb        (1'b1),
        .web           (shadow_b_we_reg),
        .addrb         (shadow_b_addr_reg),
        .dinb          (shadow_b_din_reg),
        .doutb         (shadow_b_dout),
        .sleep         (1'b0),
        .injectsbiterra(1'b0),
        .injectdbiterra(1'b0),
        .injectsbiterrb(1'b0),
        .injectdbiterrb(1'b0),
        .sbiterra      (),
        .dbiterra      (),
        .sbiterrb      (),
        .dbiterrb      ()
    );

    wire [31:0] cq_addr_wire = cq_req_data_reg[31:0] & BAR0_ADDR_MASK;
    wire [31:0] cq_addr_aligned_wire = {cq_addr_wire[31:2], 2'b00};
    wire [31:0] cq_write_data_wire = cq_req_data_reg[159:128];
    wire [ 3:0] cq_wstrb_wire = cq_req_user_reg[3:0];
    wire [31:0] cq_wstrb_mask_wire = {
        {8{cq_wstrb_wire[3]}},
        {8{cq_wstrb_wire[2]}},
        {8{cq_wstrb_wire[1]}},
        {8{cq_wstrb_wire[0]}}
    };
    wire [31:0] cq_wstrb_zero_wire = cq_write_data_wire & cq_wstrb_mask_wire;
    wire        cq_is_queue_addr_wire = cq_queue_addr_reg;
    wire [QID_WIDTH-1:0] cq_qid_wire = cq_qid_reg;
    wire [7:0]           cq_qoff_wire = cq_qoff_reg;
    wire                 cq_is_axi_bridge_addr_wire = (cq_addr_wire >= `EDEV_REG_AXI_BR_BASE) &&
                                                      (cq_addr_wire < (`EDEV_REG_AXI_BR_BASE + AXI_BR_TABLE_ENTRIES*`EDEV_AXI_BR_ENTRY_SIZE));
    wire [1:0]           cq_axi_bridge_idx_wire = cq_addr_wire[6:5];
    wire [4:0]           cq_axi_bridge_off_wire = cq_addr_wire[4:0];
    wire [31:0]          enabled_queue_count_ext_wire = {{(31 - QID_WIDTH) {1'b0}}, enabled_queue_count_reg};
    wire [31:0]          cq_queue_count_write_wire = (enabled_queue_count_ext_wire & ~cq_wstrb_mask_wire) |
                                                     (cq_write_data_wire & cq_wstrb_mask_wire);
    wire [QID_WIDTH:0]   cq_queue_count_clamped_wire =
        (cq_queue_count_write_wire == 0) ? {{QID_WIDTH{1'b0}}, 1'b1} :
        (cq_queue_count_write_wire > MAX_QUEUES) ? MAX_QUEUE_COUNT_VALUE :
        cq_queue_count_write_wire[QID_WIDTH:0];

    reg [31:0] axi_bridge_read_data_comb;
    reg [31:0] global_status_word_comb;
    reg [31:0] global_read_data_comb;
    reg [31:0] qread_status_word_comb;
    reg [31:0] qread_active_word_comb;
    reg [31:0] qread_shadow_word_comb;
    reg [31:0] qread_data_comb;

    always @* begin
        axi_bridge_read_data_comb = 32'd0;
        case (cq_axi_bridge_idx_wire)
            2'd0: begin
                case (cq_axi_bridge_off_wire)
                    AXI_BR_OFF_CTRL:         axi_bridge_read_data_comb = {31'd0, axi_bridge_table_enable_reg[0]};
                    AXI_BR_OFF_AXI_BASE_LO:  axi_bridge_read_data_comb = axi_bridge_table_axi_base_reg[31:0];
                    AXI_BR_OFF_AXI_BASE_HI:  axi_bridge_read_data_comb = axi_bridge_table_axi_base_reg[63:32];
                    AXI_BR_OFF_PCIE_BASE_LO: axi_bridge_read_data_comb = axi_bridge_table_pcie_base_reg[31:0];
                    AXI_BR_OFF_PCIE_BASE_HI: axi_bridge_read_data_comb = axi_bridge_table_pcie_base_reg[63:32];
                    AXI_BR_OFF_MASK_LO:      axi_bridge_read_data_comb = axi_bridge_table_mask_reg[31:0];
                    AXI_BR_OFF_MASK_HI:      axi_bridge_read_data_comb = axi_bridge_table_mask_reg[63:32];
                    default:                 axi_bridge_read_data_comb = 32'd0;
                endcase
            end
            2'd1: begin
                case (cq_axi_bridge_off_wire)
                    AXI_BR_OFF_CTRL:         axi_bridge_read_data_comb = {31'd0, axi_bridge_table_enable_reg[1]};
                    AXI_BR_OFF_AXI_BASE_LO:  axi_bridge_read_data_comb = axi_bridge_table_axi_base_reg[95:64];
                    AXI_BR_OFF_AXI_BASE_HI:  axi_bridge_read_data_comb = axi_bridge_table_axi_base_reg[127:96];
                    AXI_BR_OFF_PCIE_BASE_LO: axi_bridge_read_data_comb = axi_bridge_table_pcie_base_reg[95:64];
                    AXI_BR_OFF_PCIE_BASE_HI: axi_bridge_read_data_comb = axi_bridge_table_pcie_base_reg[127:96];
                    AXI_BR_OFF_MASK_LO:      axi_bridge_read_data_comb = axi_bridge_table_mask_reg[95:64];
                    AXI_BR_OFF_MASK_HI:      axi_bridge_read_data_comb = axi_bridge_table_mask_reg[127:96];
                    default:                 axi_bridge_read_data_comb = 32'd0;
                endcase
            end
            2'd2: begin
                case (cq_axi_bridge_off_wire)
                    AXI_BR_OFF_CTRL:         axi_bridge_read_data_comb = {31'd0, axi_bridge_table_enable_reg[2]};
                    AXI_BR_OFF_AXI_BASE_LO:  axi_bridge_read_data_comb = axi_bridge_table_axi_base_reg[159:128];
                    AXI_BR_OFF_AXI_BASE_HI:  axi_bridge_read_data_comb = axi_bridge_table_axi_base_reg[191:160];
                    AXI_BR_OFF_PCIE_BASE_LO: axi_bridge_read_data_comb = axi_bridge_table_pcie_base_reg[159:128];
                    AXI_BR_OFF_PCIE_BASE_HI: axi_bridge_read_data_comb = axi_bridge_table_pcie_base_reg[191:160];
                    AXI_BR_OFF_MASK_LO:      axi_bridge_read_data_comb = axi_bridge_table_mask_reg[159:128];
                    AXI_BR_OFF_MASK_HI:      axi_bridge_read_data_comb = axi_bridge_table_mask_reg[191:160];
                    default:                 axi_bridge_read_data_comb = 32'd0;
                endcase
            end
            default: begin
                case (cq_axi_bridge_off_wire)
                    AXI_BR_OFF_CTRL:         axi_bridge_read_data_comb = {31'd0, axi_bridge_table_enable_reg[3]};
                    AXI_BR_OFF_AXI_BASE_LO:  axi_bridge_read_data_comb = axi_bridge_table_axi_base_reg[223:192];
                    AXI_BR_OFF_AXI_BASE_HI:  axi_bridge_read_data_comb = axi_bridge_table_axi_base_reg[255:224];
                    AXI_BR_OFF_PCIE_BASE_LO: axi_bridge_read_data_comb = axi_bridge_table_pcie_base_reg[223:192];
                    AXI_BR_OFF_PCIE_BASE_HI: axi_bridge_read_data_comb = axi_bridge_table_pcie_base_reg[255:224];
                    AXI_BR_OFF_MASK_LO:      axi_bridge_read_data_comb = axi_bridge_table_mask_reg[223:192];
                    AXI_BR_OFF_MASK_HI:      axi_bridge_read_data_comb = axi_bridge_table_mask_reg[255:224];
                    default:                 axi_bridge_read_data_comb = 32'd0;
                endcase
            end
        endcase
    end

    always @* begin
        global_status_word_comb = 32'd0;
        global_status_word_comb[`EDEV_G_STATUS_ENABLED]       = global_ctrl_reg[`EDEV_CTRL_ENABLE];
        global_status_word_comb[`EDEV_G_STATUS_BUSY]          = sched_busy;
        global_status_word_comb[`EDEV_G_STATUS_SCHED_READY]   = ctx_ready;
        global_status_word_comb[`EDEV_G_STATUS_DOORBELL_PEND] = !pending_fifo_empty;
        global_status_word_comb[`EDEV_G_STATUS_FATAL_ERROR]   = (global_error_code_reg != 0);

        case (cq_addr_aligned_wire)
            `EDEV_REG_G_CTRL:            global_read_data_comb = global_ctrl_reg;
            `EDEV_REG_G_STATUS:          global_read_data_comb = global_status_word_comb;
            `EDEV_REG_G_MODE:            global_read_data_comb = global_mode_reg;
            `EDEV_REG_G_IRQ_ENABLE:      global_read_data_comb = global_irq_enable_reg;
            `EDEV_REG_G_IRQ_STATUS:      global_read_data_comb = irq_status_reg;
            `EDEV_REG_G_QUEUE_COUNT:     global_read_data_comb = enabled_queue_count_ext_wire;
            `EDEV_REG_G_ACTIVE_QID:      global_read_data_comb = {{(32 - QID_WIDTH) {1'b0}}, sched_active_qid};
            `EDEV_REG_G_ERROR_CODE:      global_read_data_comb = global_error_code_reg;
            `EDEV_REG_G_CAP:             global_read_data_comb = CAP_VALUE | ((MAX_QUEUES & 16'hffff) << 16);
            `EDEV_REG_G_CONTEXT_SIZE:    global_read_data_comb = {16'd0, `EDEV_Q_CONTEXT_SIZE};
            `EDEV_REG_G_PERF_DESC_FETCH: global_read_data_comb = 32'd0;
            `EDEV_REG_G_PERF_DMA_SUBMIT: global_read_data_comb = 32'd0;
            `EDEV_REG_G_PERF_WB_WRITE:   global_read_data_comb = 32'd0;
            `EDEV_REG_G_PERF_IRQ:        global_read_data_comb = 32'd0;
            default: begin
                if (cq_is_axi_bridge_addr_wire) begin
                    global_read_data_comb = axi_bridge_read_data_comb;
                end else begin
                    global_read_data_comb = 32'd0;
                end
            end
        endcase
    end

    always @* begin
        qread_status_word_comb                           = shadow_a_dout[SH_WORD_STATUS*32+:32];
        qread_status_word_comb[`EDEV_STATUS_ENABLED]     = active_a_dout[ACT_RAM_WORD_CTRL*32+`EDEV_CTRL_ENABLE];
        qread_status_word_comb[`EDEV_STATUS_BUSY]        = qread_busy_pipe_reg;
        qread_status_word_comb[`EDEV_STATUS_HALTED]      = (shadow_a_dout[SH_WORD_ERROR_CODE*32+:32] != 0);
        qread_status_word_comb[`EDEV_STATUS_RING_EMPTY]  =
            (active_a_dout[ACT_RAM_WORD_DESC_CIDX*32+:RING_INDEX_WIDTH] == desc_pidx_a_dout[RING_INDEX_WIDTH-1:0]);
        qread_status_word_comb[`EDEV_STATUS_FATAL_ERROR] = (shadow_a_dout[SH_WORD_ERROR_CODE*32+:32] != 0);

        case (qread_qoff_reg)
            `EDEV_QREG_CTRL:         qread_active_word_comb = active_a_dout[ACT_RAM_WORD_CTRL*32+:32];
            `EDEV_QREG_MODE:         qread_active_word_comb = active_a_dout[ACT_RAM_WORD_MODE*32+:32];
            `EDEV_QREG_IRQ_ENABLE:   qread_active_word_comb = active_a_dout[ACT_RAM_WORD_IRQ_ENABLE*32+:32];
            `EDEV_QREG_DESC_BASE_LO: qread_active_word_comb = active_a_dout[ACT_RAM_WORD_DESC_BASE_LO*32+:32];
            `EDEV_QREG_DESC_BASE_HI: qread_active_word_comb = active_a_dout[ACT_RAM_WORD_DESC_BASE_HI*32+:32];
            `EDEV_QREG_CMPL_BASE_LO: qread_active_word_comb = active_a_dout[ACT_RAM_WORD_CMPL_BASE_LO*32+:32];
            `EDEV_QREG_CMPL_BASE_HI: qread_active_word_comb = active_a_dout[ACT_RAM_WORD_CMPL_BASE_HI*32+:32];
            `EDEV_QREG_RING_SIZE:    qread_active_word_comb = active_a_dout[ACT_RAM_WORD_RING_SIZE*32+:32];
            `EDEV_QREG_DESC_PIDX:    qread_active_word_comb = desc_pidx_a_dout;
            `EDEV_QREG_DESC_CIDX:    qread_active_word_comb = active_a_dout[ACT_RAM_WORD_DESC_CIDX*32+:32];
            `EDEV_QREG_CMPL_PIDX:    qread_active_word_comb = active_a_dout[ACT_RAM_WORD_CMPL_PIDX*32+:32];
            default:                 qread_active_word_comb = 32'd0;
        endcase

        case (qread_qoff_reg)
            `EDEV_QREG_ERROR_CODE:      qread_shadow_word_comb = shadow_a_dout[SH_WORD_ERROR_CODE*32+:32];
            `EDEV_QREG_PERF_DESC_FETCH: qread_shadow_word_comb = shadow_a_dout[SH_WORD_PERF_DESC_FETCH*32+:32];
            `EDEV_QREG_PERF_DMA_SUBMIT: qread_shadow_word_comb = shadow_a_dout[SH_WORD_PERF_DMA_SUBMIT*32+:32];
            `EDEV_QREG_PERF_WB_WRITE:   qread_shadow_word_comb = shadow_a_dout[SH_WORD_PERF_WB_WRITE*32+:32];
            `EDEV_QREG_PERF_BYTES:      qread_shadow_word_comb = shadow_a_dout[SH_WORD_PERF_BYTES*32+:32];
            default:                    qread_shadow_word_comb = 32'd0;
        endcase

        case (qread_qoff_reg)
            `EDEV_QREG_STATUS: qread_data_comb = qread_status_word_comb;
            `EDEV_QREG_DESC_PIDX: qread_data_comb = desc_pidx_a_dout;
            `EDEV_QREG_ERROR_CODE,
            `EDEV_QREG_PERF_DESC_FETCH,
            `EDEV_QREG_PERF_DMA_SUBMIT,
            `EDEV_QREG_PERF_WB_WRITE,
            `EDEV_QREG_PERF_BYTES: qread_data_comb = qread_shadow_word_comb;
            default: qread_data_comb = qread_active_word_comb;
        endcase
    end

    task write_axi_bridge_reg;
        input [31:0] addr;
        input [31:0] data;
        input [3:0] strobe;
        reg [31:0] wmask;
        reg [31:0] tmp32;
        begin
            wmask = {{8{strobe[3]}}, {8{strobe[2]}}, {8{strobe[1]}}, {8{strobe[0]}}};
            case (addr[6:5])
                2'd0: begin
                    case (addr[4:0])
                        AXI_BR_OFF_CTRL: begin
                            tmp32 = ({31'd0, axi_bridge_table_enable_reg[0]} & ~wmask) | (data & wmask);
                            axi_bridge_table_enable_reg[0] <= tmp32[0];
                        end
                        AXI_BR_OFF_AXI_BASE_LO:  axi_bridge_table_axi_base_reg[31:0] <= (axi_bridge_table_axi_base_reg[31:0] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_AXI_BASE_HI:  axi_bridge_table_axi_base_reg[63:32] <= (axi_bridge_table_axi_base_reg[63:32] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_PCIE_BASE_LO: axi_bridge_table_pcie_base_reg[31:0] <= (axi_bridge_table_pcie_base_reg[31:0] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_PCIE_BASE_HI: axi_bridge_table_pcie_base_reg[63:32] <= (axi_bridge_table_pcie_base_reg[63:32] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_MASK_LO:      axi_bridge_table_mask_reg[31:0] <= (axi_bridge_table_mask_reg[31:0] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_MASK_HI:      axi_bridge_table_mask_reg[63:32] <= (axi_bridge_table_mask_reg[63:32] & ~wmask) | (data & wmask);
                        default: begin end
                    endcase
                end
                2'd1: begin
                    case (addr[4:0])
                        AXI_BR_OFF_CTRL: begin
                            tmp32 = ({31'd0, axi_bridge_table_enable_reg[1]} & ~wmask) | (data & wmask);
                            axi_bridge_table_enable_reg[1] <= tmp32[0];
                        end
                        AXI_BR_OFF_AXI_BASE_LO:  axi_bridge_table_axi_base_reg[95:64] <= (axi_bridge_table_axi_base_reg[95:64] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_AXI_BASE_HI:  axi_bridge_table_axi_base_reg[127:96] <= (axi_bridge_table_axi_base_reg[127:96] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_PCIE_BASE_LO: axi_bridge_table_pcie_base_reg[95:64] <= (axi_bridge_table_pcie_base_reg[95:64] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_PCIE_BASE_HI: axi_bridge_table_pcie_base_reg[127:96] <= (axi_bridge_table_pcie_base_reg[127:96] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_MASK_LO:      axi_bridge_table_mask_reg[95:64] <= (axi_bridge_table_mask_reg[95:64] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_MASK_HI:      axi_bridge_table_mask_reg[127:96] <= (axi_bridge_table_mask_reg[127:96] & ~wmask) | (data & wmask);
                        default: begin end
                    endcase
                end
                2'd2: begin
                    case (addr[4:0])
                        AXI_BR_OFF_CTRL: begin
                            tmp32 = ({31'd0, axi_bridge_table_enable_reg[2]} & ~wmask) | (data & wmask);
                            axi_bridge_table_enable_reg[2] <= tmp32[0];
                        end
                        AXI_BR_OFF_AXI_BASE_LO:  axi_bridge_table_axi_base_reg[159:128] <= (axi_bridge_table_axi_base_reg[159:128] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_AXI_BASE_HI:  axi_bridge_table_axi_base_reg[191:160] <= (axi_bridge_table_axi_base_reg[191:160] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_PCIE_BASE_LO: axi_bridge_table_pcie_base_reg[159:128] <= (axi_bridge_table_pcie_base_reg[159:128] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_PCIE_BASE_HI: axi_bridge_table_pcie_base_reg[191:160] <= (axi_bridge_table_pcie_base_reg[191:160] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_MASK_LO:      axi_bridge_table_mask_reg[159:128] <= (axi_bridge_table_mask_reg[159:128] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_MASK_HI:      axi_bridge_table_mask_reg[191:160] <= (axi_bridge_table_mask_reg[191:160] & ~wmask) | (data & wmask);
                        default: begin end
                    endcase
                end
                default: begin
                    case (addr[4:0])
                        AXI_BR_OFF_CTRL: begin
                            tmp32 = ({31'd0, axi_bridge_table_enable_reg[3]} & ~wmask) | (data & wmask);
                            axi_bridge_table_enable_reg[3] <= tmp32[0];
                        end
                        AXI_BR_OFF_AXI_BASE_LO:  axi_bridge_table_axi_base_reg[223:192] <= (axi_bridge_table_axi_base_reg[223:192] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_AXI_BASE_HI:  axi_bridge_table_axi_base_reg[255:224] <= (axi_bridge_table_axi_base_reg[255:224] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_PCIE_BASE_LO: axi_bridge_table_pcie_base_reg[223:192] <= (axi_bridge_table_pcie_base_reg[223:192] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_PCIE_BASE_HI: axi_bridge_table_pcie_base_reg[255:224] <= (axi_bridge_table_pcie_base_reg[255:224] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_MASK_LO:      axi_bridge_table_mask_reg[223:192] <= (axi_bridge_table_mask_reg[223:192] & ~wmask) | (data & wmask);
                        AXI_BR_OFF_MASK_HI:      axi_bridge_table_mask_reg[255:224] <= (axi_bridge_table_mask_reg[255:224] & ~wmask) | (data & wmask);
                        default: begin end
                    endcase
                end
            endcase
        end
    endtask

    task build_completion;
        input [2:0] cpl_status;
        input [15:0] requester_id;
        input [7:0] tag;
        input [2:0] tc;
        input [2:0] attr;
        input [3:0] first_be;
        input [31:0] addr;
        input [31:0] data;
        input has_data;
        reg [AXIS_PCIE_DATA_WIDTH-1:0] tdata;
        reg [12:0] cpl_byte_count;
        reg [6:0] cpl_lower_addr;
        begin
            casez (first_be)
                4'b0000: cpl_byte_count = 13'd1;
                4'b0001, 4'b0010, 4'b0100, 4'b1000: cpl_byte_count = 13'd1;
                4'b0011, 4'b0110, 4'b1100: cpl_byte_count = 13'd2;
                4'b01z1, 4'b1z10: cpl_byte_count = 13'd3;
                4'b1zz1: cpl_byte_count = 13'd4;
                default: cpl_byte_count = 13'd4;
            endcase

            casez (first_be)
                4'b0000: cpl_lower_addr = {addr[6:2], 2'b00};
                4'bzzz1: cpl_lower_addr = {addr[6:2], 2'b00};
                4'bzz10: cpl_lower_addr = {addr[6:2], 2'b01};
                4'bz100: cpl_lower_addr = {addr[6:2], 2'b10};
                4'b1000: cpl_lower_addr = {addr[6:2], 2'b11};
                default: cpl_lower_addr = {addr[6:2], 2'b00};
            endcase

            tdata         = {AXIS_PCIE_DATA_WIDTH{1'b0}};
            tdata[6:0]    = cpl_lower_addr;
            tdata[9:8]    = 2'b00;
            tdata[28:16]  = has_data ? cpl_byte_count : 13'd0;
            tdata[42:32]  = has_data ? 11'd1 : 11'd0;
            tdata[45:43]  = cpl_status;
            tdata[63:48]  = requester_id;
            tdata[71:64]  = tag;
            tdata[87:72]  = completer_id;
            tdata[88]     = completer_id_enable;
            tdata[91:89]  = tc;
            tdata[94:92]  = attr;
            tdata[95]     = 1'b0;
            tdata[127:96] = data;
            cc_tdata_reg  <= tdata;
            cc_tkeep_reg  <= has_data ? 8'b0000_1111 : 8'b0000_0111;
            cc_tlast_reg  <= 1'b1;
            cc_tuser_reg  <= {AXIS_PCIE_CC_USER_WIDTH{1'b0}};
            cc_tvalid_reg <= 1'b1;
        end
    endtask

    task active_port_a_write_word;
        input [QID_WIDTH-1:0] qid;
        input [7:0] qoff;
        input [31:0] wdata;
        input [3:0] wstrb;
        begin
            case (qoff)
                `EDEV_QREG_CTRL: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_CTRL*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_CTRL*32+:32] <= wdata;
                end
                `EDEV_QREG_MODE: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_MODE*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_MODE*32+:32] <= wdata;
                end
                `EDEV_QREG_IRQ_ENABLE: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_IRQ_ENABLE*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_IRQ_ENABLE*32+:32] <= wdata;
                end
                `EDEV_QREG_DESC_BASE_LO: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_DESC_BASE_LO*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_DESC_BASE_LO*32+:32] <= wdata;
                end
                `EDEV_QREG_DESC_BASE_HI: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_DESC_BASE_HI*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_DESC_BASE_HI*32+:32] <= wdata;
                end
                `EDEV_QREG_CMPL_BASE_LO: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_CMPL_BASE_LO*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_CMPL_BASE_LO*32+:32] <= wdata;
                end
                `EDEV_QREG_CMPL_BASE_HI: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_CMPL_BASE_HI*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_CMPL_BASE_HI*32+:32] <= wdata;
                end
                `EDEV_QREG_RING_SIZE: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_RING_SIZE*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_RING_SIZE*32+:32] <= wdata;
                end
                `EDEV_QREG_DESC_PIDX: begin
                    desc_pidx_a_addr_reg <= qid;
                    desc_pidx_a_en_reg   <= 1'b1;
                    desc_pidx_a_we_reg   <= wstrb;
                    desc_pidx_a_din_reg  <= wdata;
                end
                `EDEV_QREG_DESC_CIDX: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_DESC_CIDX*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_DESC_CIDX*32+:32] <= wdata;
                end
                `EDEV_QREG_CMPL_PIDX: begin
                    active_a_addr_reg <= qid;
                    active_a_en_reg   <= 1'b1;
                    active_a_we_reg[ACT_RAM_WORD_CMPL_PIDX*4+:4] <= wstrb;
                    active_a_din_reg[ACT_RAM_WORD_CMPL_PIDX*32+:32] <= wdata;
                end
                default: begin end
            endcase
        end
    endtask

    task clear_queue_port_a;
        input [QID_WIDTH-1:0] qid;
        begin
            active_a_addr_reg <= qid;
            active_a_en_reg   <= 1'b1;
            active_a_we_reg   <= {ACTIVE_RAM_BYTES{1'b1}};
            active_a_din_reg  <= {ACTIVE_RAM_WIDTH{1'b0}};
            shadow_a_addr_reg <= qid;
            shadow_a_en_reg   <= 1'b1;
            shadow_a_we_reg   <= {SHADOW_CTX_BYTES{1'b1}};
            shadow_a_din_reg  <= {SHADOW_CTX_WIDTH{1'b0}};
            desc_pidx_a_addr_reg <= qid;
            desc_pidx_a_en_reg   <= 1'b1;
            desc_pidx_a_we_reg   <= {DESC_PIDX_RAM_BYTES{1'b1}};
            desc_pidx_a_din_reg  <= {DESC_PIDX_RAM_WIDTH{1'b0}};
        end
    endtask

    task enqueue_queue;
        input [QID_WIDTH-1:0] qid;
        input [7:0] minor;
        integer slot;
        begin
            if (cq_enqueue_allowed_reg) begin
                // Each bitmap bit uses only its own state. The selected-bit
                // mux below controls FIFO insertion/error reporting, never
                // the D input of another queue's state bit.
                for (slot=0; slot<MAX_QUEUES; slot=slot+1) begin
                    if (cq_enqueue_onehot[slot]) begin
                        if (q_pending_reg[slot] || q_busy_reg[slot])
                            q_resched_reg[slot] <= 1'b1;
                        else if (!(pending_fifo_full && !pf_pop)) begin
                            q_pending_reg[slot] <= 1'b1;
                            q_resched_reg[slot] <= 1'b0;
                        end
                    end
                end
                if (!selected_host_busy) begin
                    if (pending_fifo_full && !pf_pop) begin
                        global_error_code_reg           <= {`EDEV_ERR_DOORBELL_BUSY, minor, {{(16 - QID_WIDTH) {1'b0}}, qid}};
                        irq_status_reg[`EDEV_IRQ_ERROR] <= 1'b1;
                    end else begin
                        pf_push     = 1'b1;
                        pf_push_qid = qid;
                    end
                end
            end
        end
    endtask

    task enqueue_queue_after_done;
        input [QID_WIDTH-1:0] qid;
        input [7:0] minor;
        begin
            if ({1'b0, qid} < enabled_queue_count_reg) begin
                if (!q_pending_reg[qid]) begin
                    if (pending_fifo_full && !pf_pop) begin
                        global_error_code_reg           <= {`EDEV_ERR_DOORBELL_BUSY, minor, {{(16 - QID_WIDTH) {1'b0}}, qid}};
                        irq_status_reg[`EDEV_IRQ_ERROR] <= 1'b1;
                    end else begin
                        q_pending_reg[qid] <= 1'b1;
                        q_resched_reg[qid] <= 1'b0;
                        pf_push     = 1'b1;
                        pf_push_qid = qid;
                    end
                end
            end
        end
    endtask

    task update_queue_state_on_host_write;
        input [QID_WIDTH-1:0] qid;
        input [7:0] qoff;
        input [31:0] wdata;
        input [3:0] wstrb;
        reg [31:0] new_ctrl;
        begin
            case (qoff)
                `EDEV_QREG_CTRL: begin
                    new_ctrl = 32'd0;
                    if (wstrb[0]) new_ctrl[7:0] = wdata[7:0];
                    if (wstrb[1]) new_ctrl[15:8] = wdata[15:8];
                    if (wstrb[2]) new_ctrl[23:16] = wdata[23:16];
                    if (wstrb[3]) new_ctrl[31:24] = wdata[31:24];
                    if (new_ctrl[`EDEV_CTRL_RESET]) begin
                        q_pending_reg[qid]      <= 1'b0;
                        q_busy_reg[qid]         <= 1'b0;
                        q_resched_reg[qid]      <= 1'b0;
                        clear_queue_port_a(qid);
                    end
                end
                `EDEV_QREG_DESC_PIDX: begin
                    enqueue_queue(qid, 8'h00);
                end
                default: begin
                end
            endcase
        end
    endtask

    integer                        i;

    wire                           upd_op_done = ctx_update_pipe_data[UPD_LSB_OP_DONE];
    wire                           upd_requeue = ctx_update_pipe_data[UPD_LSB_REQUEUE];
    wire                           upd_desc_we = ctx_update_pipe_data[UPD_LSB_DESC_WE];
    wire                           upd_cmpl_we = ctx_update_pipe_data[UPD_LSB_CMPL_WE];
    wire    [       QID_WIDTH-1:0] upd_qid = ctx_update_pipe_data[UPD_LSB_QID+:QID_WIDTH];
    wire    [RING_INDEX_WIDTH-1:0] upd_hw_desc_cidx = ctx_update_pipe_data[UPD_LSB_DESC_CIDX+:RING_INDEX_WIDTH];
    wire    [RING_INDEX_WIDTH-1:0] upd_cmpl_pidx = ctx_update_pipe_data[UPD_LSB_CMPL_PIDX+:RING_INDEX_WIDTH];
    wire    [                31:0] upd_status = ctx_update_pipe_data[UPD_LSB_STATUS+:32];
    wire    [                31:0] upd_error_code = ctx_update_pipe_data[UPD_LSB_ERROR_CODE+:32];
    wire    [                31:0] upd_perf_desc_fetch = ctx_update_pipe_data[UPD_LSB_PERF_DESC+:32];
    wire    [                31:0] upd_perf_dma_submit = ctx_update_pipe_data[UPD_LSB_PERF_DMA+:32];
    wire    [                31:0] upd_perf_wb_write = ctx_update_pipe_data[UPD_LSB_PERF_WB+:32];
    wire    [                31:0] upd_perf_bytes = ctx_update_pipe_data[UPD_LSB_PERF_BYTES+:32];
    always @(posedge clk) begin
        pf_push     = 1'b0;
        pf_pop      = 1'b0;
        pf_push_qid = {QID_WIDTH{1'b0}};
        pf_pop_qid  = {QID_WIDTH{1'b0}};

        active_a_en_reg        <= 1'b0;
        active_a_we_reg        <= {ACTIVE_RAM_BYTES{1'b0}};
        active_a_din_reg       <= {ACTIVE_RAM_WIDTH{1'b0}};
        active_b_en_reg        <= 1'b0;
        active_b_we_reg        <= {ACTIVE_RAM_BYTES{1'b0}};
        active_b_din_reg       <= {ACTIVE_RAM_WIDTH{1'b0}};
        desc_pidx_a_en_reg     <= 1'b0;
        desc_pidx_a_we_reg     <= {DESC_PIDX_RAM_BYTES{1'b0}};
        desc_pidx_a_din_reg    <= {DESC_PIDX_RAM_WIDTH{1'b0}};
        desc_pidx_b_en_reg     <= 1'b0;
        desc_pidx_b_we_reg     <= {DESC_PIDX_RAM_BYTES{1'b0}};
        desc_pidx_b_din_reg    <= {DESC_PIDX_RAM_WIDTH{1'b0}};
        shadow_a_en_reg        <= 1'b0;
        shadow_a_we_reg        <= {SHADOW_CTX_BYTES{1'b0}};
        shadow_a_din_reg       <= {SHADOW_CTX_WIDTH{1'b0}};
        shadow_b_en_reg        <= 1'b0;
        shadow_b_we_reg        <= {SHADOW_CTX_BYTES{1'b0}};
        shadow_b_din_reg       <= {SHADOW_CTX_WIDTH{1'b0}};

        status_error_cor_reg   <= 1'b0;
        status_error_uncor_reg <= 1'b0;
        irq_status_reg         <= irq_status_reg | irq_status_set;

        if (cc_tvalid_reg && m_axis_cc_tready) begin
            cc_tvalid_reg <= 1'b0;
        end

        // Port-B priority: global clear -> scheduler update -> pending FIFO context read.
        if (clear_active_reg) begin
            active_b_addr_reg <= clear_qid_reg;
            active_b_en_reg   <= 1'b1;
            active_b_we_reg   <= {ACTIVE_RAM_BYTES{1'b1}};
            active_b_din_reg  <= {ACTIVE_RAM_WIDTH{1'b0}};
            shadow_b_addr_reg <= clear_qid_reg;
            shadow_b_en_reg   <= 1'b1;
            shadow_b_we_reg   <= {SHADOW_CTX_BYTES{1'b1}};
            shadow_b_din_reg  <= {SHADOW_CTX_WIDTH{1'b0}};
            desc_pidx_b_addr_reg <= clear_qid_reg;
            desc_pidx_b_en_reg   <= 1'b1;
            desc_pidx_b_we_reg   <= {DESC_PIDX_RAM_BYTES{1'b1}};
            desc_pidx_b_din_reg  <= {DESC_PIDX_RAM_WIDTH{1'b0}};
            if ({1'b0, clear_qid_reg} == enabled_queue_count_reg - 1'b1) begin
                clear_active_reg <= 1'b0;
                clear_qid_reg    <= {QID_WIDTH{1'b0}};
            end else begin
                clear_qid_reg <= clear_qid_reg + 1'b1;
            end
        end else if (ctx_update_pipe_valid && upd_qid < MAX_QUEUES) begin
            active_b_addr_reg                           <= upd_qid;
            active_b_en_reg                             <= 1'b1;
            active_b_we_reg                             <= {ACTIVE_RAM_BYTES{1'b0}};
            active_b_din_reg                            <= {ACTIVE_RAM_WIDTH{1'b0}};
            active_b_we_reg[ACT_RAM_WORD_DESC_CIDX*4+:4] <= upd_desc_we ? 4'hf : 4'h0;
            active_b_we_reg[ACT_RAM_WORD_CMPL_PIDX*4+:4] <= upd_cmpl_we ? 4'hf : 4'h0;
            active_b_din_reg[ACT_RAM_WORD_DESC_CIDX*32+:32] <= {{(32 - RING_INDEX_WIDTH) {1'b0}}, upd_hw_desc_cidx};
            active_b_din_reg[ACT_RAM_WORD_CMPL_PIDX*32+:32] <= {{(32 - RING_INDEX_WIDTH) {1'b0}}, upd_cmpl_pidx};

            shadow_b_addr_reg                           <= upd_qid;
            shadow_b_en_reg                             <= 1'b1;
            shadow_b_we_reg                             <= {SHADOW_CTX_BYTES{1'b0}};
            shadow_b_din_reg                            <= {SHADOW_CTX_WIDTH{1'b0}};
            shadow_b_we_reg[SH_WORD_STATUS*4+:4]           <= 4'hf;
            shadow_b_we_reg[SH_WORD_ERROR_CODE*4+:4]       <= 4'hf;
            shadow_b_we_reg[SH_WORD_PERF_DESC_FETCH*4+:4]  <= 4'hf;
            shadow_b_we_reg[SH_WORD_PERF_DMA_SUBMIT*4+:4]  <= 4'hf;
            shadow_b_we_reg[SH_WORD_PERF_WB_WRITE*4+:4]    <= 4'hf;
            shadow_b_we_reg[SH_WORD_PERF_BYTES*4+:4]       <= 4'hf;
            shadow_b_din_reg[SH_WORD_STATUS*32+:32]        <= upd_status;
            shadow_b_din_reg[SH_WORD_ERROR_CODE*32+:32]    <= upd_error_code;
            shadow_b_din_reg[SH_WORD_PERF_DESC_FETCH*32+:32] <= upd_perf_desc_fetch;
            shadow_b_din_reg[SH_WORD_PERF_DMA_SUBMIT*32+:32] <= upd_perf_dma_submit;
            shadow_b_din_reg[SH_WORD_PERF_WB_WRITE*32+:32]   <= upd_perf_wb_write;
            shadow_b_din_reg[SH_WORD_PERF_BYTES*32+:32]      <= upd_perf_bytes;

            if (upd_op_done) begin
                // Queue bitmap D inputs use only their own state bits.  The
                // selected-bit mux below drives the FIFO push only, avoiding
                // a 512-way priority chain from QID to FIFO write enable.
                for (i = 0; i < MAX_QUEUES; i = i + 1) begin
                    if (ctx_update_onehot[i]) begin
                        q_busy_reg[i] <= 1'b0;
                        if ((upd_requeue || q_resched_reg[i]) &&
                            ({1'b0, upd_qid} < enabled_queue_count_reg) &&
                            !q_pending_reg[i] &&
                            !(pending_fifo_full && !pf_pop)) begin
                            q_pending_reg[i] <= 1'b1;
                            q_resched_reg[i] <= 1'b0;
                        end
                    end
                end
                if ((upd_requeue || selected_update_resched) &&
                    ({1'b0, upd_qid} < enabled_queue_count_reg) &&
                    !selected_update_pending) begin
                    if (pending_fifo_full && !pf_pop) begin
                        global_error_code_reg <= {`EDEV_ERR_DOORBELL_BUSY,8'h02,{{(16-QID_WIDTH){1'b0}},upd_qid}};
                        irq_status_reg[`EDEV_IRQ_ERROR] <= 1'b1;
                    end else begin
                        pf_push = 1'b1;
                        pf_push_qid = upd_qid;
                    end
                end
            end
        end else if (ctx_can_start_fetch) begin
            pf_pop     = 1'b1;
            pf_pop_qid = pending_fifo_mem[pending_fifo_rd_ptr_reg];
            active_b_addr_reg                                        <= pending_fifo_mem[pending_fifo_rd_ptr_reg];
            active_b_en_reg                                          <= 1'b1;
            active_b_we_reg                                          <= {ACTIVE_RAM_BYTES{1'b0}};
            desc_pidx_b_addr_reg                                     <= pending_fifo_mem[pending_fifo_rd_ptr_reg];
            desc_pidx_b_en_reg                                       <= 1'b1;
            ctx_fetch_busy_reg                                       <= 1'b1;
            ctx_fetch_qid_reg                                        <= pending_fifo_mem[pending_fifo_rd_ptr_reg];
            ctx_fetch_count_reg                                      <= ACTIVE_CTX_READ_LATENCY;
            q_pending_reg[pending_fifo_mem[pending_fifo_rd_ptr_reg]] <= 1'b0;
            q_busy_reg[pending_fifo_mem[pending_fifo_rd_ptr_reg]]    <= 1'b1;
        end

        if (ctx_fetch_busy_reg) begin
            if (ctx_fetch_count_reg != 0) begin
                ctx_fetch_count_reg <= ctx_fetch_count_reg - 1'b1;
            end else begin
                ctx_fetch_busy_reg                                           <= 1'b0;
                ctx_hold_valid_reg                                           <= 1'b1;
                ctx_hold_data_reg[CTX_LSB_QID+:QID_WIDTH] <= ctx_fetch_qid_reg;
                ctx_hold_data_reg[CTX_LSB_ACTIVE+:ACTIVE_CTX_WIDTH] <= {ACTIVE_CTX_WIDTH{1'b0}};
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_CTRL*32+:32] <= active_b_dout[ACT_RAM_WORD_CTRL*32+:32];
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_MODE*32+:32] <= active_b_dout[ACT_RAM_WORD_MODE*32+:32];
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_IRQ_ENABLE*32+:32] <= active_b_dout[ACT_RAM_WORD_IRQ_ENABLE*32+:32] & global_irq_enable_reg;
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_DESC_BASE_LO*32+:32] <= active_b_dout[ACT_RAM_WORD_DESC_BASE_LO*32+:32];
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_DESC_BASE_HI*32+:32] <= active_b_dout[ACT_RAM_WORD_DESC_BASE_HI*32+:32];
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_CMPL_BASE_LO*32+:32] <= active_b_dout[ACT_RAM_WORD_CMPL_BASE_LO*32+:32];
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_CMPL_BASE_HI*32+:32] <= active_b_dout[ACT_RAM_WORD_CMPL_BASE_HI*32+:32];
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_RING_SIZE*32+:32] <= active_b_dout[ACT_RAM_WORD_RING_SIZE*32+:32];
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_DESC_PIDX*32+:32] <= desc_pidx_b_dout;
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_DESC_CIDX*32+:32] <= active_b_dout[ACT_RAM_WORD_DESC_CIDX*32+:32];
                ctx_hold_data_reg[CTX_LSB_ACTIVE+ACT_WORD_CMPL_PIDX*32+:32] <= active_b_dout[ACT_RAM_WORD_CMPL_PIDX*32+:32];
            end
        end

        if (ctx_fire) begin
            ctx_hold_valid_reg <= 1'b0;
        end

        if (state_reg == STATE_QREAD_WAIT) begin
            if (qread_count_reg != 0) begin
                qread_count_reg <= qread_count_reg - 1'b1;
            end else begin
                build_completion(CPL_STATUS_SC, qread_requester_id_reg, qread_tag_reg, qread_tc_reg, qread_attr_reg, qread_first_be_reg, qread_addr_reg, qread_data_comb, 1'b1);
                state_reg <= STATE_IDLE;
            end
        end

        if (s_axis_cq_tvalid && s_axis_cq_tready) begin
            cq_req_valid_reg <= 1'b1;
            cq_req_data_reg <= s_axis_cq_tdata;
            cq_req_user_reg <= s_axis_cq_tuser;
            cq_req_last_reg <= s_axis_cq_tlast;
            cq_qid_reg <= (cq_input_addr - `EDEV_REG_Q_BASE) >> 8;
            cq_qoff_reg <= (cq_input_addr - `EDEV_REG_Q_BASE) & 32'hff;
        end else if (cq_exec_fire) begin
            cq_req_valid_reg <= 1'b0;
        end

        if (cq_exec_fire) begin
            // The CQ descriptor carries the full PCIe byte address.  The CQ demux
            // selects BAR0 by bar_id but does not rewrite the address, so decode only
            // the low bits inside the BAR0 aperture.
            if (state_reg == STATE_IDLE) begin
                if (cq_read_type_reg) begin
                    if (cq_single_dword_reg) begin
                        if (cq_is_queue_addr_wire) begin
                            active_a_addr_reg      <= cq_qid_wire;
                            active_a_en_reg        <= 1'b1;
                            active_a_we_reg        <= {ACTIVE_RAM_BYTES{1'b0}};
                            shadow_a_addr_reg      <= cq_qid_wire;
                            shadow_a_en_reg        <= 1'b1;
                            shadow_a_we_reg        <= {SHADOW_CTX_BYTES{1'b0}};
                            desc_pidx_a_addr_reg   <= cq_qid_wire;
                            desc_pidx_a_en_reg     <= 1'b1;
                            qread_count_reg        <= (ACTIVE_CTX_READ_LATENCY > SHADOW_CTX_READ_LATENCY) ? ACTIVE_CTX_READ_LATENCY : SHADOW_CTX_READ_LATENCY;
                            qread_qid_reg          <= cq_qid_wire;
                            qread_qoff_reg         <= cq_qoff_wire;
                            qread_requester_id_reg <= cq_req_data_reg[95:80];
                            qread_tag_reg          <= cq_req_data_reg[103:96];
                            qread_tc_reg           <= cq_req_data_reg[123:121];
                            qread_attr_reg         <= cq_req_data_reg[126:124];
                            qread_first_be_reg     <= cq_req_user_reg[3:0];
                            qread_addr_reg         <= cq_addr_wire;
                            state_reg              <= STATE_QREAD_WAIT;
                        end else begin
                            build_completion(CPL_STATUS_SC, cq_req_data_reg[95:80], cq_req_data_reg[103:96], cq_req_data_reg[123:121], cq_req_data_reg[126:124], cq_req_user_reg[3:0], cq_addr_wire, global_read_data_comb, 1'b1);
                        end
                    end else begin
                        status_error_cor_reg <= 1'b1;
                        if (cq_req_last_reg) begin
                            build_completion(CPL_STATUS_CA, cq_req_data_reg[95:80], cq_req_data_reg[103:96], cq_req_data_reg[123:121], cq_req_data_reg[126:124], cq_req_user_reg[3:0], cq_addr_wire, 32'd0, 1'b0);
                        end else begin
                            wait_error_cpl_reg    <= 1'b1;
                            wait_cpl_status_reg   <= CPL_STATUS_CA;
                            wait_requester_id_reg <= cq_req_data_reg[95:80];
                            wait_tag_reg          <= cq_req_data_reg[103:96];
                            wait_tc_reg           <= cq_req_data_reg[123:121];
                            wait_attr_reg         <= cq_req_data_reg[126:124];
                            wait_first_be_reg     <= cq_req_user_reg[3:0];
                            wait_addr_reg         <= cq_addr_wire;
                            state_reg             <= STATE_WAIT_END;
                        end
                    end
                end else if (cq_write_type_reg) begin
                    if (cq_single_dword_reg) begin
                        if (cq_is_queue_addr_wire) begin
                            active_port_a_write_word(cq_qid_wire, cq_qoff_wire, cq_write_data_wire, cq_wstrb_wire);
                            update_queue_state_on_host_write(cq_qid_wire, cq_qoff_wire, cq_write_data_wire, cq_wstrb_wire);
                        end else begin
                            if (cq_is_axi_bridge_addr_wire) begin
                                write_axi_bridge_reg(cq_addr_wire, cq_write_data_wire, cq_wstrb_wire);
                            end else begin
                                case (cq_addr_aligned_wire)
                                    `EDEV_REG_G_CTRL:        global_ctrl_reg <= (global_ctrl_reg & ~cq_wstrb_mask_wire) | (cq_write_data_wire & cq_wstrb_mask_wire);
                                `EDEV_REG_G_MODE:        global_mode_reg <= (global_mode_reg & ~cq_wstrb_mask_wire) | (cq_write_data_wire & cq_wstrb_mask_wire);
                                `EDEV_REG_G_IRQ_ENABLE:  global_irq_enable_reg <= (global_irq_enable_reg & ~cq_wstrb_mask_wire) | (cq_write_data_wire & cq_wstrb_mask_wire);
                                `EDEV_REG_G_IRQ_STATUS:  irq_status_reg <= (irq_status_reg | irq_status_set) & ~cq_wstrb_zero_wire;
                                `EDEV_REG_G_QUEUE_COUNT: enabled_queue_count_reg <= cq_queue_count_clamped_wire;
                                `EDEV_REG_G_DOORBELL: begin
                                    enqueue_queue(cq_write_data_wire[QID_WIDTH-1:0], 8'h01);
                                end
                                    default: begin
                                    end
                                endcase
                            end
                        end

                        if (cq_req_data_reg[78:75] == REQ_IO_WRITE) begin
                            build_completion(CPL_STATUS_SC, cq_req_data_reg[95:80], cq_req_data_reg[103:96], cq_req_data_reg[123:121], cq_req_data_reg[126:124], cq_req_user_reg[3:0], cq_addr_wire, 32'd0, 1'b0);
                        end
                    end else begin
                        if (cq_req_data_reg[78:75] == REQ_MEM_WRITE) begin
                            status_error_uncor_reg <= 1'b1;
                            if (!cq_req_last_reg) begin
                                wait_error_cpl_reg <= 1'b0;
                                state_reg          <= STATE_WAIT_END;
                            end
                        end else begin
                            status_error_cor_reg <= 1'b1;
                            if (cq_req_last_reg) begin
                                build_completion(CPL_STATUS_CA, cq_req_data_reg[95:80], cq_req_data_reg[103:96], cq_req_data_reg[123:121], cq_req_data_reg[126:124], cq_req_user_reg[3:0], cq_addr_wire, 32'd0, 1'b0);
                            end else begin
                                wait_error_cpl_reg    <= 1'b1;
                                wait_cpl_status_reg   <= CPL_STATUS_CA;
                                wait_requester_id_reg <= cq_req_data_reg[95:80];
                                wait_tag_reg          <= cq_req_data_reg[103:96];
                                wait_tc_reg           <= cq_req_data_reg[123:121];
                                wait_attr_reg         <= cq_req_data_reg[126:124];
                                wait_first_be_reg     <= cq_req_user_reg[3:0];
                                wait_addr_reg         <= cq_addr_wire;
                                state_reg             <= STATE_WAIT_END;
                            end
                        end
                    end
                end else begin
                    status_error_cor_reg <= 1'b1;
                    if (cq_req_last_reg) begin
                        build_completion(CPL_STATUS_UR, cq_req_data_reg[95:80], cq_req_data_reg[103:96], cq_req_data_reg[123:121], cq_req_data_reg[126:124], cq_req_user_reg[3:0], cq_addr_wire, 32'd0, 1'b0);
                    end else begin
                        wait_error_cpl_reg    <= 1'b1;
                        wait_cpl_status_reg   <= CPL_STATUS_UR;
                        wait_requester_id_reg <= cq_req_data_reg[95:80];
                        wait_tag_reg          <= cq_req_data_reg[103:96];
                        wait_tc_reg           <= cq_req_data_reg[123:121];
                        wait_attr_reg         <= cq_req_data_reg[126:124];
                        wait_first_be_reg     <= cq_req_user_reg[3:0];
                        wait_addr_reg         <= cq_addr_wire;
                        state_reg             <= STATE_WAIT_END;
                    end
                end
            end else if (state_reg == STATE_WAIT_END && cq_req_last_reg) begin
                if (wait_error_cpl_reg && !cc_tvalid_reg) begin
                    build_completion(wait_cpl_status_reg, wait_requester_id_reg, wait_tag_reg, wait_tc_reg, wait_attr_reg, wait_first_be_reg, wait_addr_reg, 32'd0, 1'b0);
                end
                state_reg <= STATE_IDLE;
            end
        end

        if (pf_pop) begin
            pending_fifo_rd_ptr_reg <= pending_fifo_rd_ptr_reg + 1'b1;
        end
        if (pf_push) begin
            pending_fifo_mem[pending_fifo_wr_ptr_reg] <= pf_push_qid;
            pending_fifo_wr_ptr_reg                   <= pending_fifo_wr_ptr_reg + 1'b1;
        end
        case ({
            pf_push, pf_pop
        })
            2'b10:   pending_fifo_count_reg <= pending_fifo_count_reg + 1'b1;
            2'b01:   pending_fifo_count_reg <= pending_fifo_count_reg - 1'b1;
            default: pending_fifo_count_reg <= pending_fifo_count_reg;
        endcase

        if (rst || global_ctrl_reg[`EDEV_CTRL_RESET]) begin
            cq_req_valid_reg <= 1'b0;
            state_reg              <= STATE_IDLE;
            cc_tvalid_reg          <= 1'b0;
            status_error_cor_reg   <= 1'b0;
            status_error_uncor_reg <= 1'b0;
            wait_error_cpl_reg     <= 1'b0;
            global_ctrl_reg        <= 32'd0;
            global_mode_reg        <= 32'd0;
            global_irq_enable_reg  <= 32'd0;
            if (rst) enabled_queue_count_reg <= MAX_QUEUE_COUNT_VALUE;
            irq_status_reg             <= 32'd0;
            global_error_code_reg      <= 32'd0;
            q_pending_reg              <= {MAX_QUEUES{1'b0}};
            q_busy_reg                 <= {MAX_QUEUES{1'b0}};
            q_resched_reg              <= {MAX_QUEUES{1'b0}};
            for (i = 0; i < MAX_QUEUES; i = i + 1) begin
            end
            pending_fifo_wr_ptr_reg    <= {PENDING_FIFO_ADDR_WIDTH{1'b0}};
            pending_fifo_rd_ptr_reg    <= {PENDING_FIFO_ADDR_WIDTH{1'b0}};
            pending_fifo_count_reg     <= {(PENDING_FIFO_ADDR_WIDTH + 1) {1'b0}};
            ctx_fetch_busy_reg         <= 1'b0;
            ctx_fetch_count_reg        <= 4'd0;
            ctx_hold_valid_reg         <= 1'b0;
            clear_active_reg           <= 1'b1;
            clear_qid_reg              <= {QID_WIDTH{1'b0}};
            qread_count_reg            <= 4'd0;
        end
    end

endmodule

`resetall
