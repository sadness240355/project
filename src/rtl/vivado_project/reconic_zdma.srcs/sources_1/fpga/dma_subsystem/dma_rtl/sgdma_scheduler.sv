`resetall
`timescale 1ns / 1ps
`default_nettype none

/* SPDX-License-Identifier: MIT
 * Credit-window multi-queue SGDMA scheduler.
 *
 * Architectural change from the legacy single-active-op scheduler:
 *   - accepts several queue contexts into a small active queue-state table;
 *   - descriptor fetch runs as an independent, credit-limited round-robin
 *     scheduler and can look past queues that are stalled at payload issue;
 *   - fetched descriptors are decoded into four independent descriptor FIFOs
 *     (MM H2D, MM D2H, ST TX, ST RX);
 *   - payload issue runs from those FIFOs and is independent from descriptor
 *     fetch;
 *   - completion/writeback is handled by a separate queue so descriptor fetch
 *     is not back-pressured by writeback latency.
 *
 * Scheduler prefetch ports were intentionally removed. Descriptor look-ahead is
 * now implemented by the descriptor fetch scheduler and per-path FIFOs.
 */

`include "edev_sgdma_defs.vh"

module edev_sgdma_scheduler #(
    parameter PCIE_ADDR_WIDTH           = 64,
    parameter AXI_ADDR_WIDTH            = 64,
    parameter LEN_WIDTH                 = 24,
    parameter TAG_WIDTH                 = 12,
    parameter RING_INDEX_WIDTH          = 16,
    parameter QID_WIDTH                 = 9,
    parameter IRQ_VECTOR_COUNT          = 512,
    parameter DESC_WIDTH                = 256,
    parameter MAX_QUEUES                = 512,
    parameter CMPL_WIDTH                = 256,
    parameter OP_FIFO_DEPTH             = 16,
    parameter OP_FIFO_ADDR_WIDTH        = 4,
    parameter OP_FIFO_READ_LATENCY      = 2,
    parameter OUTSTANDING_OP_COUNT      = 16,
    parameter ISSUE_QUANTUM             = 16,
    parameter DESC_FETCH_MAX_COUNT      = 8,
    parameter DESC_FETCH_TIMEOUT_CYCLES = 24'd1000000,
    parameter PATH_FIFO_DEPTH           = 16,
    parameter PATH_FIFO_ADDR_WIDTH      = 4,
    parameter WB_FIFO_DEPTH             = 16,
    parameter WB_FIFO_ADDR_WIDTH        = 4,
    parameter ST_STAT_FIFO_DEPTH        = 8,
    parameter ST_STAT_FIFO_ADDR_WIDTH   = 3,
    // When set, IRQ-generating writebacks are followed by a host read fence
    // before the MSI-X event is emitted.  This is conservative but adds a
    // non-posted MRd on the latency-sensitive hostmem RC path.  The default
    // relies on PCIe posted-write ordering between the WB MWr and MSI/MSI-X
    // MWr, avoiding fence MRd pressure on Tegra root ports.
    parameter IRQ_READBACK_FENCE_ENABLE = 0,
    parameter CTX_DATA_WIDTH            = QID_WIDTH + 352,
    parameter CTX_UPDATE_WIDTH          = 4 + QID_WIDTH + RING_INDEX_WIDTH * 2 + 32 * 6
) (
    input wire clk,
    input wire rst,

    input wire        global_enable,
    input wire        global_pause,
    input wire [31:0] global_mode,
    input wire [ 2:0] max_read_request_size,

    input  wire                      ctx_valid,
    output wire                      ctx_ready,
    input  wire [CTX_DATA_WIDTH-1:0] ctx_data,

    output wire                        ctx_update_valid,
    output wire [CTX_UPDATE_WIDTH-1:0] ctx_update_data,
    output wire                        sched_busy,
    output wire [       QID_WIDTH-1:0] sched_active_qid,

    output wire [                31:0] irq_status_set,
    output wire [       QID_WIDTH-1:0] irq_qid,
    output wire                       irq_valid,

    output wire [PCIE_ADDR_WIDTH-1:0] desc_fetch_addr,
    output wire [      TAG_WIDTH-1:0] desc_fetch_tag,
    output wire [                4:0] desc_fetch_count,
    output wire                       desc_fetch_is_st,
    output wire                       desc_fetch_valid,
    input  wire                       desc_fetch_ready,


    input  wire [ TAG_WIDTH-1:0] desc_resp_tag,
    input  wire [DESC_WIDTH-1:0] desc_resp_data,
    input  wire [           3:0] desc_resp_error,
    input  wire                  desc_resp_valid,
    output wire                  desc_resp_ready,

    output wire [PCIE_ADDR_WIDTH-1:0] cmpl_write_addr,
    output wire [     CMPL_WIDTH-1:0] cmpl_write_data,
    output wire                       cmpl_write_valid,
    output wire                       cmpl_write_fence,
    input  wire                       cmpl_write_ready,
    input  wire                       cmpl_write_done,

    output wire [PCIE_ADDR_WIDTH-1:0] dma_read_desc_pcie_addr,
    output wire [ AXI_ADDR_WIDTH-1:0] dma_read_desc_axi_addr,
    output wire [      LEN_WIDTH-1:0] dma_read_desc_len,
    output wire [      TAG_WIDTH-1:0] dma_read_desc_tag,
    output wire [       QID_WIDTH-1:0] dma_read_desc_qid,
    output wire                       dma_read_desc_valid,
    input  wire                       dma_read_desc_ready,
    input  wire [      TAG_WIDTH-1:0] dma_read_status_tag,
    input  wire [                3:0] dma_read_status_error,
    input  wire                       dma_read_status_valid,

    output wire [PCIE_ADDR_WIDTH-1:0] dma_write_desc_pcie_addr,
    output wire [ AXI_ADDR_WIDTH-1:0] dma_write_desc_axi_addr,
    output wire [      LEN_WIDTH-1:0] dma_write_desc_len,
    output wire [      TAG_WIDTH-1:0] dma_write_desc_tag,
    output wire [       QID_WIDTH-1:0] dma_write_desc_qid,
    output wire                       dma_write_desc_valid,
    input  wire                       dma_write_desc_ready,
    input  wire [      TAG_WIDTH-1:0] dma_write_status_tag,
    input  wire [                3:0] dma_write_status_error,
    input  wire                       dma_write_status_valid,

    output wire [PCIE_ADDR_WIDTH-1:0] st_tx_desc_host_addr,
    output wire [      LEN_WIDTH-1:0] st_tx_desc_len,
    output wire [      TAG_WIDTH-1:0] st_tx_desc_tag,
    output wire [       QID_WIDTH-1:0] st_tx_desc_qid,
    output wire [RING_INDEX_WIDTH-1:0] st_tx_desc_cidx_after,
    output wire [                15:0] st_tx_desc_flags,
    output wire [                15:0] st_tx_desc_swtag,
    output wire [                31:0] st_tx_desc_user,
    output wire                       st_tx_desc_valid,
    input  wire                       st_tx_desc_ready,
    input  wire [      TAG_WIDTH-1:0] st_tx_status_tag,
    input  wire [       QID_WIDTH-1:0] st_tx_status_qid,
    input  wire [RING_INDEX_WIDTH-1:0] st_tx_status_cidx_after,
    input  wire [                31:0] st_tx_status_bytes,
    input  wire [                31:0] st_tx_status_error_code,
    input  wire [                15:0] st_tx_status_swtag,
    input  wire [                31:0] st_tx_status_user,
    input  wire                       st_tx_status_valid,
    output wire                       st_tx_status_ready,

    output wire [PCIE_ADDR_WIDTH-1:0] st_rx_desc_host_addr,
    output wire [      LEN_WIDTH-1:0] st_rx_desc_len,
    output wire [      TAG_WIDTH-1:0] st_rx_desc_tag,
    output wire [       QID_WIDTH-1:0] st_rx_desc_qid,
    output wire [RING_INDEX_WIDTH-1:0] st_rx_desc_cidx_after,
    output wire [                15:0] st_rx_desc_flags,
    output wire [                15:0] st_rx_desc_swtag,
    output wire [                31:0] st_rx_desc_user,
    output wire [PCIE_ADDR_WIDTH-1:0] st_rx_desc_wb_base,
    output wire [                31:0] st_rx_desc_irq_enable,
    output wire                       st_rx_desc_valid,
    input  wire                       st_rx_desc_ready,
    input  wire [      TAG_WIDTH-1:0] st_rx_status_tag,
    input  wire [       QID_WIDTH-1:0] st_rx_status_qid,
    input  wire [RING_INDEX_WIDTH-1:0] st_rx_status_cidx_after,
    input  wire [                31:0] st_rx_status_bytes,
    input  wire [                31:0] st_rx_status_error_code,
    input  wire [                15:0] st_rx_status_swtag,
    input  wire [                31:0] st_rx_status_user,
    input  wire [PCIE_ADDR_WIDTH-1:0] st_rx_status_wb_base,
    input  wire [                31:0] st_rx_status_irq_enable,
    input  wire                       st_rx_status_valid,
    output wire                       st_rx_status_ready
);

    // The timing-oriented static implementation below intentionally uses fixed
    // scalar banks and explicit case muxes.  Keep these internal dimensions
    // fixed even if an older top-level still overrides the legacy parameters.
    // This prevents Vivado from elaborating 32-entry pointer widths while the
    // static case banks are 16-entry wide.
    localparam integer STATIC_OP_FIFO_DEPTH           = 16;
    localparam integer STATIC_OP_FIFO_ADDR_WIDTH      = 4;
    localparam integer STATIC_PATH_FIFO_DEPTH         = 16;
    localparam integer STATIC_PATH_FIFO_ADDR_WIDTH    = 4;
    localparam integer STATIC_WB_FIFO_DEPTH           = 16;
    localparam integer STATIC_WB_FIFO_ADDR_WIDTH      = 4;
    localparam integer STATIC_ST_STAT_FIFO_DEPTH      = 8;
    localparam integer STATIC_ST_STAT_FIFO_ADDR_WIDTH = 3;

    localparam integer SLOT_COUNT = STATIC_OP_FIFO_DEPTH;
    localparam integer SLOT_INDEX_WIDTH = STATIC_OP_FIFO_ADDR_WIDTH;
    localparam [STATIC_PATH_FIFO_ADDR_WIDTH:0] PATH_FIFO_LEVEL_MAX = STATIC_PATH_FIFO_DEPTH;
    localparam [4:0] PATH_FIFO_FETCH_LIMIT = 5'd16;
    localparam integer ISSUE_QUANTUM_LIMIT = (ISSUE_QUANTUM > 16) ? 16 : ISSUE_QUANTUM;
    localparam integer DESC_FETCH_MAX_COUNT_LIMIT =
        (DESC_FETCH_MAX_COUNT < 1) ? 1 :
        (DESC_FETCH_MAX_COUNT > 16) ? 16 : DESC_FETCH_MAX_COUNT;
    localparam integer MM_TAG_COUNT = 16;
    localparam integer MM_TAG_INDEX_WIDTH = 4;
    localparam integer MM_TAG_NUM_WIDTH = 5;
    localparam integer GLOBAL_DESC_FETCH_LIMIT_BYTES = 65536;
    localparam integer GLOBAL_DESC_FETCH_COUNT_WIDTH = 17;

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
    localparam integer CTX_LSB_QID = 0;
    localparam integer CTX_LSB_ACTIVE = CTX_LSB_QID + QID_WIDTH;

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

    localparam [1:0] PATH_MM_H2D = 2'd0;
    localparam [1:0] PATH_MM_D2H = 2'd1;
    localparam [1:0] PATH_ST_TX  = 2'd2;
    localparam [1:0] PATH_ST_RX  = 2'd3;
    /* ---------------------------------------------------------------------
     * Vivado-safe internal entry formats.
     *
     * The previous timing-oriented refactor used packed SystemVerilog structs
     * inside unpacked FIFO memories.  Vivado 2022.2 can elaborate that pattern
     * with very high memory use and may terminate abnormally during synthesis.
     * These entries are now plain packed vectors with constant range aliases;
     * all access remains explicit and static, with no functions/tasks.
     * ------------------------------------------------------------------ */
    localparam integer DESC_ENTRY_HOST_ADDR_LSB = 0;
    localparam integer DESC_ENTRY_HOST_ADDR_MSB = DESC_ENTRY_HOST_ADDR_LSB + PCIE_ADDR_WIDTH - 1;
    localparam integer DESC_ENTRY_DEV_ADDR_LSB = DESC_ENTRY_HOST_ADDR_MSB + 1;
    localparam integer DESC_ENTRY_DEV_ADDR_MSB = DESC_ENTRY_DEV_ADDR_LSB + AXI_ADDR_WIDTH - 1;
    localparam integer DESC_ENTRY_LEN_LSB = DESC_ENTRY_DEV_ADDR_MSB + 1;
    localparam integer DESC_ENTRY_LEN_MSB = DESC_ENTRY_LEN_LSB + LEN_WIDTH - 1;
    localparam integer DESC_ENTRY_WB_BYTES_LSB = DESC_ENTRY_LEN_MSB + 1;
    localparam integer DESC_ENTRY_WB_BYTES_MSB = DESC_ENTRY_WB_BYTES_LSB + 32 - 1;
    localparam integer DESC_ENTRY_QID_LSB = DESC_ENTRY_WB_BYTES_MSB + 1;
    localparam integer DESC_ENTRY_QID_MSB = DESC_ENTRY_QID_LSB + QID_WIDTH - 1;
    localparam integer DESC_ENTRY_SLOT_LSB = DESC_ENTRY_QID_MSB + 1;
    localparam integer DESC_ENTRY_SLOT_MSB = DESC_ENTRY_SLOT_LSB + SLOT_INDEX_WIDTH - 1;
    localparam integer DESC_ENTRY_WB_BASE_LSB = DESC_ENTRY_SLOT_MSB + 1;
    localparam integer DESC_ENTRY_WB_BASE_MSB = DESC_ENTRY_WB_BASE_LSB + PCIE_ADDR_WIDTH - 1;
    localparam integer DESC_ENTRY_IRQ_ENABLE_LSB = DESC_ENTRY_WB_BASE_MSB + 1;
    localparam integer DESC_ENTRY_IRQ_ENABLE_MSB = DESC_ENTRY_IRQ_ENABLE_LSB + 32 - 1;
    localparam integer DESC_ENTRY_CIDX_AFTER_LSB = DESC_ENTRY_IRQ_ENABLE_MSB + 1;
    localparam integer DESC_ENTRY_CIDX_AFTER_MSB = DESC_ENTRY_CIDX_AFTER_LSB + RING_INDEX_WIDTH - 1;
    /* Retained packed-field position for compatibility with the existing
     * descriptor-entry layout.  The sync-only ABI uses one fixed WB record per
     * queue, so this field is forced to zero below and is not an address index.
     */
    localparam integer DESC_ENTRY_CMPL_SLOT_LSB = DESC_ENTRY_CIDX_AFTER_MSB + 1;
    localparam integer DESC_ENTRY_CMPL_SLOT_MSB = DESC_ENTRY_CMPL_SLOT_LSB + RING_INDEX_WIDTH - 1;
    localparam integer DESC_ENTRY_TARGET_PIDX_LSB = DESC_ENTRY_CMPL_SLOT_MSB + 1;
    localparam integer DESC_ENTRY_TARGET_PIDX_MSB = DESC_ENTRY_TARGET_PIDX_LSB + RING_INDEX_WIDTH - 1;
    localparam integer DESC_ENTRY_FLAGS_LSB = DESC_ENTRY_TARGET_PIDX_MSB + 1;
    localparam integer DESC_ENTRY_FLAGS_MSB = DESC_ENTRY_FLAGS_LSB + 16 - 1;
    localparam integer DESC_ENTRY_SWTAG_LSB = DESC_ENTRY_FLAGS_MSB + 1;
    localparam integer DESC_ENTRY_SWTAG_MSB = DESC_ENTRY_SWTAG_LSB + 16 - 1;
    localparam integer DESC_ENTRY_DIR_LSB = DESC_ENTRY_SWTAG_MSB + 1;
    localparam integer DESC_ENTRY_DIR_MSB = DESC_ENTRY_DIR_LSB + 1 - 1;
    localparam integer DESC_ENTRY_IS_ST_LSB = DESC_ENTRY_DIR_MSB + 1;
    localparam integer DESC_ENTRY_IS_ST_MSB = DESC_ENTRY_IS_ST_LSB + 1 - 1;
    localparam integer DESC_ENTRY_TERMINAL_LSB = DESC_ENTRY_IS_ST_MSB + 1;
    localparam integer DESC_ENTRY_TERMINAL_MSB = DESC_ENTRY_TERMINAL_LSB + 1 - 1;
    localparam integer DESC_ENTRY_ERROR_CODE_LSB = DESC_ENTRY_TERMINAL_MSB + 1;
    localparam integer DESC_ENTRY_ERROR_CODE_MSB = DESC_ENTRY_ERROR_CODE_LSB + 32 - 1;
    localparam integer DESC_ENTRY_WIDTH = DESC_ENTRY_ERROR_CODE_MSB + 1;

    localparam integer WB_ENTRY_WB_BASE_LSB = 0;
    localparam integer WB_ENTRY_WB_BASE_MSB = WB_ENTRY_WB_BASE_LSB + PCIE_ADDR_WIDTH - 1;
    localparam integer WB_ENTRY_IRQ_ENABLE_LSB = WB_ENTRY_WB_BASE_MSB + 1;
    localparam integer WB_ENTRY_IRQ_ENABLE_MSB = WB_ENTRY_IRQ_ENABLE_LSB + 32 - 1;
    localparam integer WB_ENTRY_QID_LSB = WB_ENTRY_IRQ_ENABLE_MSB + 1;
    localparam integer WB_ENTRY_QID_MSB = WB_ENTRY_QID_LSB + QID_WIDTH - 1;
    localparam integer WB_ENTRY_CIDX_AFTER_LSB = WB_ENTRY_QID_MSB + 1;
    localparam integer WB_ENTRY_CIDX_AFTER_MSB = WB_ENTRY_CIDX_AFTER_LSB + RING_INDEX_WIDTH - 1;
    localparam integer WB_ENTRY_BYTES_LSB = WB_ENTRY_CIDX_AFTER_MSB + 1;
    localparam integer WB_ENTRY_BYTES_MSB = WB_ENTRY_BYTES_LSB + 32 - 1;
    localparam integer WB_ENTRY_ERROR_CODE_LSB = WB_ENTRY_BYTES_MSB + 1;
    localparam integer WB_ENTRY_ERROR_CODE_MSB = WB_ENTRY_ERROR_CODE_LSB + 32 - 1;
    localparam integer WB_ENTRY_SWTAG_LSB = WB_ENTRY_ERROR_CODE_MSB + 1;
    localparam integer WB_ENTRY_SWTAG_MSB = WB_ENTRY_SWTAG_LSB + 16 - 1;
    localparam integer WB_ENTRY_USER_LSB = WB_ENTRY_SWTAG_MSB + 1;
    localparam integer WB_ENTRY_USER_MSB = WB_ENTRY_USER_LSB + 32 - 1;
    localparam integer WB_ENTRY_DIR_LSB = WB_ENTRY_USER_MSB + 1;
    localparam integer WB_ENTRY_DIR_MSB = WB_ENTRY_DIR_LSB + 1 - 1;
    localparam integer WB_ENTRY_IS_ST_LSB = WB_ENTRY_DIR_MSB + 1;
    localparam integer WB_ENTRY_IS_ST_MSB = WB_ENTRY_IS_ST_LSB + 1 - 1;
    localparam integer WB_ENTRY_SOP_LSB = WB_ENTRY_IS_ST_MSB + 1;
    localparam integer WB_ENTRY_SOP_MSB = WB_ENTRY_SOP_LSB + 1 - 1;
    localparam integer WB_ENTRY_EOP_LSB = WB_ENTRY_SOP_MSB + 1;
    localparam integer WB_ENTRY_EOP_MSB = WB_ENTRY_EOP_LSB + 1 - 1;
    localparam integer WB_ENTRY_IRQ_ON_DONE_LSB = WB_ENTRY_EOP_MSB + 1;
    localparam integer WB_ENTRY_IRQ_ON_DONE_MSB = WB_ENTRY_IRQ_ON_DONE_LSB + 1 - 1;
    localparam integer WB_ENTRY_WB_MERGED_ERROR_LSB = WB_ENTRY_IRQ_ON_DONE_MSB + 1;
    localparam integer WB_ENTRY_WB_MERGED_ERROR_MSB = WB_ENTRY_WB_MERGED_ERROR_LSB + 1 - 1;
    localparam integer WB_ENTRY_WIDTH = WB_ENTRY_WB_MERGED_ERROR_MSB + 1;

    localparam integer ST_STATUS_ENTRY_SLOT_LSB = 0;
    localparam integer ST_STATUS_ENTRY_SLOT_MSB = ST_STATUS_ENTRY_SLOT_LSB + SLOT_INDEX_WIDTH - 1;
    localparam integer ST_STATUS_ENTRY_WB_BASE_LSB = ST_STATUS_ENTRY_SLOT_MSB + 1;
    localparam integer ST_STATUS_ENTRY_WB_BASE_MSB = ST_STATUS_ENTRY_WB_BASE_LSB + PCIE_ADDR_WIDTH - 1;
    localparam integer ST_STATUS_ENTRY_IRQ_ENABLE_LSB = ST_STATUS_ENTRY_WB_BASE_MSB + 1;
    localparam integer ST_STATUS_ENTRY_IRQ_ENABLE_MSB = ST_STATUS_ENTRY_IRQ_ENABLE_LSB + 32 - 1;
    localparam integer ST_STATUS_ENTRY_QID_LSB = ST_STATUS_ENTRY_IRQ_ENABLE_MSB + 1;
    localparam integer ST_STATUS_ENTRY_QID_MSB = ST_STATUS_ENTRY_QID_LSB + QID_WIDTH - 1;
    localparam integer ST_STATUS_ENTRY_CIDX_AFTER_LSB = ST_STATUS_ENTRY_QID_MSB + 1;
    localparam integer ST_STATUS_ENTRY_CIDX_AFTER_MSB = ST_STATUS_ENTRY_CIDX_AFTER_LSB + RING_INDEX_WIDTH - 1;
    localparam integer ST_STATUS_ENTRY_BYTES_LSB = ST_STATUS_ENTRY_CIDX_AFTER_MSB + 1;
    localparam integer ST_STATUS_ENTRY_BYTES_MSB = ST_STATUS_ENTRY_BYTES_LSB + 32 - 1;
    localparam integer ST_STATUS_ENTRY_ERROR_CODE_LSB = ST_STATUS_ENTRY_BYTES_MSB + 1;
    localparam integer ST_STATUS_ENTRY_ERROR_CODE_MSB = ST_STATUS_ENTRY_ERROR_CODE_LSB + 32 - 1;
    localparam integer ST_STATUS_ENTRY_SWTAG_LSB = ST_STATUS_ENTRY_ERROR_CODE_MSB + 1;
    localparam integer ST_STATUS_ENTRY_SWTAG_MSB = ST_STATUS_ENTRY_SWTAG_LSB + 16 - 1;
    localparam integer ST_STATUS_ENTRY_USER_LSB = ST_STATUS_ENTRY_SWTAG_MSB + 1;
    localparam integer ST_STATUS_ENTRY_USER_MSB = ST_STATUS_ENTRY_USER_LSB + 32 - 1;
    localparam integer ST_STATUS_ENTRY_IS_RX_LSB = ST_STATUS_ENTRY_USER_MSB + 1;
    localparam integer ST_STATUS_ENTRY_IS_RX_MSB = ST_STATUS_ENTRY_IS_RX_LSB + 1 - 1;
    localparam integer ST_STATUS_ENTRY_WIDTH = ST_STATUS_ENTRY_IS_RX_MSB + 1;


    /* ---------------------------------------------------------------------
     * Queue-state table.  This is the lightweight per-active-queue context
     * table.  The table is intentionally limited to OP_FIFO_DEPTH slots and is
     * scanned one slot per cycle to avoid a 512-qid high-fanout ready bitmap.
     * ------------------------------------------------------------------ */
    (* shreg_extract = "no" *) reg                         slot_valid_reg      [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [QID_WIDTH-1:0]         slot_qid_reg        [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [31:0]                  slot_ctrl_reg       [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [31:0]                  slot_mode_reg       [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [31:0]                  slot_irq_enable_reg [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [PCIE_ADDR_WIDTH-1:0]   slot_desc_base_reg  [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [PCIE_ADDR_WIDTH-1:0]   slot_wb_base_reg    [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [RING_INDEX_WIDTH-1:0]  slot_ring_size_reg  [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [RING_INDEX_WIDTH-1:0]  slot_target_pidx_reg[0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [RING_INDEX_WIDTH-1:0]  slot_fetch_cidx_reg [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [RING_INDEX_WIDTH-1:0]  slot_issue_cidx_reg [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [5:0]                   slot_fetch_inflight_reg [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg                         slot_fetch_stop_reg [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [31:0]                  slot_perf_desc_reg  [0:SLOT_COUNT-1];
    (* shreg_extract = "no" *) reg [31:0]                  slot_perf_dma_reg   [0:SLOT_COUNT-1];

    reg [SLOT_INDEX_WIDTH-1:0]  free_slot_index_reg = 0;
    reg [SLOT_INDEX_WIDTH-1:0]  free_scan_ptr_reg = 0;
    reg                         free_slot_valid_reg = 1'b1;
    reg [SLOT_INDEX_WIDTH:0]    slot_occupancy_count_reg = 0;

    wire [QID_WIDTH-1:0] ctx_bus_qid = ctx_data[(CTX_LSB_QID)+(QID_WIDTH)-1:(CTX_LSB_QID)];
    wire [ACTIVE_CTX_WIDTH-1:0] ctx_bus_active = ctx_data[(CTX_LSB_ACTIVE)+(ACTIVE_CTX_WIDTH)-1:(CTX_LSB_ACTIVE)];
    assign ctx_ready = global_enable && !global_pause && free_slot_valid_reg;
    wire ctx_fire = ctx_valid && ctx_ready;

    /* ---------------------------------------------------------------------
     * Path descriptor FIFOs.  MM H2D, MM D2H, ST TX and ST RX have independent
     * FIFOs so one congested path cannot block descriptor fetch for another.
     * ------------------------------------------------------------------ */
    (* ram_style = "distributed" *) reg [DESC_ENTRY_WIDTH-1:0] mm_h2d_fifo_mem [0:STATIC_PATH_FIFO_DEPTH-1];
    (* ram_style = "distributed" *) reg [DESC_ENTRY_WIDTH-1:0] mm_d2h_fifo_mem [0:STATIC_PATH_FIFO_DEPTH-1];
    (* ram_style = "distributed" *) reg [DESC_ENTRY_WIDTH-1:0] st_tx_fifo_mem  [0:STATIC_PATH_FIFO_DEPTH-1];
    (* ram_style = "distributed" *) reg [DESC_ENTRY_WIDTH-1:0] st_rx_fifo_mem  [0:STATIC_PATH_FIFO_DEPTH-1];

    reg [STATIC_PATH_FIFO_ADDR_WIDTH-1:0] mm_h2d_wr_ptr_reg = 0, mm_h2d_rd_ptr_reg = 0;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH-1:0] mm_d2h_wr_ptr_reg = 0, mm_d2h_rd_ptr_reg = 0;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH-1:0] st_tx_wr_ptr_reg  = 0, st_tx_rd_ptr_reg  = 0;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH-1:0] st_rx_wr_ptr_reg  = 0, st_rx_rd_ptr_reg  = 0;

    reg [DESC_ENTRY_WIDTH-1:0] mm_h2d_fifo_rd_mux;
    reg [DESC_ENTRY_WIDTH-1:0] mm_d2h_fifo_rd_mux;
    reg [DESC_ENTRY_WIDTH-1:0] st_tx_fifo_rd_mux;
    reg [DESC_ENTRY_WIDTH-1:0] st_rx_fifo_rd_mux;

    always @* begin
        mm_h2d_fifo_rd_mux = mm_h2d_fifo_mem[mm_h2d_rd_ptr_reg];
        mm_d2h_fifo_rd_mux = mm_d2h_fifo_mem[mm_d2h_rd_ptr_reg];
        st_tx_fifo_rd_mux  = st_tx_fifo_mem[st_tx_rd_ptr_reg];
        st_rx_fifo_rd_mux  = st_rx_fifo_mem[st_rx_rd_ptr_reg];
    end


    /* Path FIFO level counters are kept local to the FIFO owner.  The
     * descriptor-fetch and payload-issue schedulers consume registered
     * shadow signals instead of subtracting from *_count_reg in their own
     * combinational cones.  This removes the mm_*_count_reg fanout into the
     * fetch limit, response ready and issue select paths without changing the
     * FIFO throughput semantics.
     */
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] mm_h2d_count_reg = 0, mm_d2h_count_reg = 0;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] st_tx_count_reg = 0, st_rx_count_reg = 0;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] mm_h2d_space_reg = PATH_FIFO_LEVEL_MAX;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] mm_d2h_space_reg = PATH_FIFO_LEVEL_MAX;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] st_tx_space_reg = PATH_FIFO_LEVEL_MAX;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] st_rx_space_reg = PATH_FIFO_LEVEL_MAX;
    reg                         mm_h2d_not_empty_reg = 1'b0;
    reg                         mm_d2h_not_empty_reg = 1'b0;
    reg                         st_tx_not_empty_reg  = 1'b0;
    reg                         st_rx_not_empty_reg  = 1'b0;
    reg                         mm_h2d_space_avail_reg = 1'b1;
    reg                         mm_d2h_space_avail_reg = 1'b1;
    reg                         st_tx_space_avail_reg  = 1'b1;
    reg                         st_rx_space_avail_reg  = 1'b1;
    reg [4:0]                   mm_h2d_fetch_space_reg = PATH_FIFO_FETCH_LIMIT;
    reg [4:0]                   mm_d2h_fetch_space_reg = PATH_FIFO_FETCH_LIMIT;
    reg [4:0]                   st_tx_fetch_space_reg  = PATH_FIFO_FETCH_LIMIT;
    reg [4:0]                   st_rx_fetch_space_reg  = PATH_FIFO_FETCH_LIMIT;
    reg [4:0]                   mm_fetch_min_space_reg = PATH_FIFO_FETCH_LIMIT;

    wire mm_h2d_empty = !mm_h2d_not_empty_reg;
    wire mm_d2h_empty = !mm_d2h_not_empty_reg;
    wire st_tx_empty  = !st_tx_not_empty_reg;
    wire st_rx_empty  = !st_rx_not_empty_reg;

    reg [1:0] issue_rr_reg = 2'd0;
    reg [1:0] issue_path_reg = 2'd0;
    reg [DESC_ENTRY_WIDTH-1:0] issue_entry_reg;
    reg issue_entry_valid_reg = 1'b0;

    /* ---------------------------------------------------------------------
     * Descriptor fetch scheduler.
     * It scans one slot per cycle, computes a bounded fetch count, checks the
     * global descriptor-fetch credit and emits a demand descriptor request.
     * No payload issue or writeback state is in this process.
     * ------------------------------------------------------------------ */
    reg [SLOT_INDEX_WIDTH-1:0] fetch_scan_ptr_reg = 0;
    reg [SLOT_INDEX_WIDTH-1:0] fetch_req_slot_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] fetch_req_cidx_reg = 0;
    reg [4:0] fetch_req_count_reg = 0;
    reg fetch_req_is_st_reg = 1'b0;

    reg [PCIE_ADDR_WIDTH-1:0] desc_fetch_addr_reg = 0;
    reg [TAG_WIDTH-1:0]       desc_fetch_tag_reg = 0;
    reg [4:0]                 desc_fetch_count_reg = 0;
    reg                       desc_fetch_is_st_reg = 1'b0;
    reg                       desc_fetch_valid_reg = 1'b0;

    reg                       fetch_resp_active_reg = 1'b0;
    reg [SLOT_INDEX_WIDTH-1:0] fetch_resp_slot_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] fetch_resp_cidx_reg = 0;
    reg [4:0]                 fetch_resp_left_reg = 0;
    reg                       fetch_resp_drain_reg = 1'b0;
    reg                       fetch_resp_is_st_reg = 1'b0;

    reg [GLOBAL_DESC_FETCH_COUNT_WIDTH-1:0] global_desc_fetch_bytes_reg = 0;

    wire desc_fetch_accept = desc_fetch_valid_reg && desc_fetch_ready;
    wire desc_resp_fire = desc_resp_valid && desc_resp_ready;
    reg [RING_INDEX_WIDTH-1:0] fetch_boundary_comb;
    reg [RING_INDEX_WIDTH-1:0] fetch_limit_wide_comb;
    reg [RING_INDEX_WIDTH-1:0] fetch_idx_comb;
    reg [PCIE_ADDR_WIDTH-1:0] fetch_offset_comb;
    reg [RING_INDEX_WIDTH:0] fetch_sum_comb;
    reg [RING_INDEX_WIDTH-1:0] fetch_req_next_cidx_comb;
    reg [RING_INDEX_WIDTH-1:0] fetch_resp_next_cidx_comb;
    reg                        resp_terminal_comb;
    reg [4:0]                  fetch_pipe_limit_eval_comb;
    reg [PCIE_ADDR_WIDTH-1:0]  fetch_pipe_addr_eval_comb;
    reg [4:0]                  fetch_mrrs_desc_limit_comb;
    reg [12:0]                 fetch_4kb_bytes_left_comb;
    reg [8:0]                  fetch_4kb_desc_limit_wide_comb;
    reg [4:0]                  fetch_4kb_desc_limit_comb;
    reg [4:0]                  fetch_pipe_limit_pcie_comb;

    reg [2:0] fetch_state_reg = 3'd0;
    localparam [2:0] FETCH_STATE_SCAN  = 3'd0;
    localparam [2:0] FETCH_STATE_EVAL0 = 3'd1;
    localparam [2:0] FETCH_STATE_EVAL1 = 3'd2;
    localparam [2:0] FETCH_STATE_EVAL2 = 3'd3;

    reg [SLOT_INDEX_WIDTH-1:0] fetch_pipe_slot_reg = 0;
    reg                        fetch_pipe_valid_reg = 1'b0;
    reg [QID_WIDTH-1:0]        fetch_pipe_qid_reg = 0;
    reg [31:0]                 fetch_pipe_ctrl_reg = 0;
    reg [31:0]                 fetch_pipe_mode_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0]  fetch_pipe_desc_base_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0]  fetch_pipe_wb_base_reg = 0;
    reg [31:0]                 fetch_pipe_irq_enable_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] fetch_pipe_ring_size_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] fetch_pipe_target_pidx_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] fetch_pipe_cidx_reg = 0;
    reg [5:0]                  fetch_pipe_inflight_reg = 0;
    reg                        fetch_pipe_stop_reg = 1'b0;
    reg                        fetch_pipe_is_st_reg = 1'b0;
    reg                        fetch_pipe_bad_dir_reg = 1'b0;
    reg [RING_INDEX_WIDTH-1:0] fetch_pipe_pending_reg = 0;
    reg [4:0]                  fetch_pipe_quantum_reg = 0;
    reg [4:0]                  fetch_pipe_fifo_limit_reg = 0;
    reg [4:0]                  fetch_pipe_credit_limit_reg = 0;
    reg [4:0]                  fetch_pipe_limit_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0]  fetch_pipe_addr_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] fetch_req_ring_size_reg = 0;

    /* Response-side metadata is captured when a descriptor fetch request is
     * accepted.  The descriptor response path therefore does not need to
     * dynamically read the active slot table for qid/mode/ring/wb fields on
     * every completion beat.  This keeps the slot BRAM read cone out of
     * desc_resp_ready and response-decode timing.
     */
    reg [QID_WIDTH-1:0]        fetch_resp_qid_reg = 0;
    reg [31:0]                 fetch_resp_mode_reg = 0;
    reg [31:0]                 fetch_resp_irq_enable_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0]  fetch_resp_wb_base_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] fetch_resp_ring_size_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] fetch_resp_target_pidx_reg = 0;

    /* Static active-slot read muxes.  These replace variable-index reads on
     * the fetch/free hot paths with explicit one-of-16 muxes.  The generated
     * logic is wider than a dynamic array read, but avoids inferred LUTRAM
     * read-port timing surprises and follows the no-subroutine/no-dynamic-index
     * coding rule for the scheduler control path.
     */
    reg fetch_slot_valid_mux;
    reg [QID_WIDTH-1:0] fetch_slot_qid_mux;
    reg [31:0] fetch_slot_ctrl_mux;
    reg [31:0] fetch_slot_mode_mux;
    reg [PCIE_ADDR_WIDTH-1:0] fetch_slot_desc_base_mux;
    reg [PCIE_ADDR_WIDTH-1:0] fetch_slot_wb_base_mux;
    reg [31:0] fetch_slot_irq_enable_mux;
    reg [RING_INDEX_WIDTH-1:0] fetch_slot_ring_size_mux;
    reg [RING_INDEX_WIDTH-1:0] fetch_slot_target_pidx_mux;
    reg [RING_INDEX_WIDTH-1:0] fetch_slot_fetch_cidx_mux;
    reg [RING_INDEX_WIDTH-1:0] fetch_slot_issue_cidx_mux;
    reg [5:0] fetch_slot_fetch_inflight_mux;
    reg fetch_slot_fetch_stop_mux;
    reg [31:0] fetch_slot_perf_desc_mux;
    reg [31:0] fetch_slot_perf_dma_mux;
    reg free_slot_valid_mux;

    always @* begin
        fetch_slot_valid_mux = 1'b0;
        fetch_slot_qid_mux = {QID_WIDTH{1'b0}};
        fetch_slot_ctrl_mux = 32'd0;
        fetch_slot_mode_mux = 32'd0;
        fetch_slot_desc_base_mux = {PCIE_ADDR_WIDTH{1'b0}};
        fetch_slot_wb_base_mux = {PCIE_ADDR_WIDTH{1'b0}};
        fetch_slot_irq_enable_mux = 32'd0;
        fetch_slot_ring_size_mux = {RING_INDEX_WIDTH{1'b0}};
        fetch_slot_target_pidx_mux = {RING_INDEX_WIDTH{1'b0}};
        fetch_slot_fetch_cidx_mux = {RING_INDEX_WIDTH{1'b0}};
        fetch_slot_issue_cidx_mux = {RING_INDEX_WIDTH{1'b0}};
        fetch_slot_fetch_inflight_mux = 6'd0;
        fetch_slot_fetch_stop_mux = 1'b0;
        fetch_slot_perf_desc_mux = 32'd0;
        fetch_slot_perf_dma_mux = 32'd0;
        case (fetch_scan_ptr_reg)
            4'd0: begin
                fetch_slot_valid_mux = slot_valid_reg[0];
                fetch_slot_qid_mux = slot_qid_reg[0];
                fetch_slot_ctrl_mux = slot_ctrl_reg[0];
                fetch_slot_mode_mux = slot_mode_reg[0];
                fetch_slot_desc_base_mux = slot_desc_base_reg[0];
                fetch_slot_wb_base_mux = slot_wb_base_reg[0];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[0];
                fetch_slot_ring_size_mux = slot_ring_size_reg[0];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[0];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[0];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[0];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[0];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[0];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[0];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[0];
            end
            4'd1: begin
                fetch_slot_valid_mux = slot_valid_reg[1];
                fetch_slot_qid_mux = slot_qid_reg[1];
                fetch_slot_ctrl_mux = slot_ctrl_reg[1];
                fetch_slot_mode_mux = slot_mode_reg[1];
                fetch_slot_desc_base_mux = slot_desc_base_reg[1];
                fetch_slot_wb_base_mux = slot_wb_base_reg[1];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[1];
                fetch_slot_ring_size_mux = slot_ring_size_reg[1];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[1];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[1];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[1];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[1];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[1];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[1];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[1];
            end
            4'd2: begin
                fetch_slot_valid_mux = slot_valid_reg[2];
                fetch_slot_qid_mux = slot_qid_reg[2];
                fetch_slot_ctrl_mux = slot_ctrl_reg[2];
                fetch_slot_mode_mux = slot_mode_reg[2];
                fetch_slot_desc_base_mux = slot_desc_base_reg[2];
                fetch_slot_wb_base_mux = slot_wb_base_reg[2];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[2];
                fetch_slot_ring_size_mux = slot_ring_size_reg[2];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[2];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[2];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[2];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[2];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[2];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[2];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[2];
            end
            4'd3: begin
                fetch_slot_valid_mux = slot_valid_reg[3];
                fetch_slot_qid_mux = slot_qid_reg[3];
                fetch_slot_ctrl_mux = slot_ctrl_reg[3];
                fetch_slot_mode_mux = slot_mode_reg[3];
                fetch_slot_desc_base_mux = slot_desc_base_reg[3];
                fetch_slot_wb_base_mux = slot_wb_base_reg[3];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[3];
                fetch_slot_ring_size_mux = slot_ring_size_reg[3];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[3];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[3];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[3];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[3];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[3];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[3];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[3];
            end
            4'd4: begin
                fetch_slot_valid_mux = slot_valid_reg[4];
                fetch_slot_qid_mux = slot_qid_reg[4];
                fetch_slot_ctrl_mux = slot_ctrl_reg[4];
                fetch_slot_mode_mux = slot_mode_reg[4];
                fetch_slot_desc_base_mux = slot_desc_base_reg[4];
                fetch_slot_wb_base_mux = slot_wb_base_reg[4];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[4];
                fetch_slot_ring_size_mux = slot_ring_size_reg[4];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[4];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[4];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[4];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[4];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[4];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[4];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[4];
            end
            4'd5: begin
                fetch_slot_valid_mux = slot_valid_reg[5];
                fetch_slot_qid_mux = slot_qid_reg[5];
                fetch_slot_ctrl_mux = slot_ctrl_reg[5];
                fetch_slot_mode_mux = slot_mode_reg[5];
                fetch_slot_desc_base_mux = slot_desc_base_reg[5];
                fetch_slot_wb_base_mux = slot_wb_base_reg[5];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[5];
                fetch_slot_ring_size_mux = slot_ring_size_reg[5];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[5];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[5];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[5];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[5];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[5];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[5];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[5];
            end
            4'd6: begin
                fetch_slot_valid_mux = slot_valid_reg[6];
                fetch_slot_qid_mux = slot_qid_reg[6];
                fetch_slot_ctrl_mux = slot_ctrl_reg[6];
                fetch_slot_mode_mux = slot_mode_reg[6];
                fetch_slot_desc_base_mux = slot_desc_base_reg[6];
                fetch_slot_wb_base_mux = slot_wb_base_reg[6];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[6];
                fetch_slot_ring_size_mux = slot_ring_size_reg[6];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[6];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[6];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[6];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[6];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[6];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[6];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[6];
            end
            4'd7: begin
                fetch_slot_valid_mux = slot_valid_reg[7];
                fetch_slot_qid_mux = slot_qid_reg[7];
                fetch_slot_ctrl_mux = slot_ctrl_reg[7];
                fetch_slot_mode_mux = slot_mode_reg[7];
                fetch_slot_desc_base_mux = slot_desc_base_reg[7];
                fetch_slot_wb_base_mux = slot_wb_base_reg[7];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[7];
                fetch_slot_ring_size_mux = slot_ring_size_reg[7];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[7];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[7];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[7];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[7];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[7];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[7];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[7];
            end
            4'd8: begin
                fetch_slot_valid_mux = slot_valid_reg[8];
                fetch_slot_qid_mux = slot_qid_reg[8];
                fetch_slot_ctrl_mux = slot_ctrl_reg[8];
                fetch_slot_mode_mux = slot_mode_reg[8];
                fetch_slot_desc_base_mux = slot_desc_base_reg[8];
                fetch_slot_wb_base_mux = slot_wb_base_reg[8];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[8];
                fetch_slot_ring_size_mux = slot_ring_size_reg[8];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[8];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[8];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[8];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[8];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[8];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[8];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[8];
            end
            4'd9: begin
                fetch_slot_valid_mux = slot_valid_reg[9];
                fetch_slot_qid_mux = slot_qid_reg[9];
                fetch_slot_ctrl_mux = slot_ctrl_reg[9];
                fetch_slot_mode_mux = slot_mode_reg[9];
                fetch_slot_desc_base_mux = slot_desc_base_reg[9];
                fetch_slot_wb_base_mux = slot_wb_base_reg[9];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[9];
                fetch_slot_ring_size_mux = slot_ring_size_reg[9];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[9];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[9];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[9];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[9];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[9];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[9];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[9];
            end
            4'd10: begin
                fetch_slot_valid_mux = slot_valid_reg[10];
                fetch_slot_qid_mux = slot_qid_reg[10];
                fetch_slot_ctrl_mux = slot_ctrl_reg[10];
                fetch_slot_mode_mux = slot_mode_reg[10];
                fetch_slot_desc_base_mux = slot_desc_base_reg[10];
                fetch_slot_wb_base_mux = slot_wb_base_reg[10];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[10];
                fetch_slot_ring_size_mux = slot_ring_size_reg[10];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[10];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[10];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[10];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[10];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[10];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[10];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[10];
            end
            4'd11: begin
                fetch_slot_valid_mux = slot_valid_reg[11];
                fetch_slot_qid_mux = slot_qid_reg[11];
                fetch_slot_ctrl_mux = slot_ctrl_reg[11];
                fetch_slot_mode_mux = slot_mode_reg[11];
                fetch_slot_desc_base_mux = slot_desc_base_reg[11];
                fetch_slot_wb_base_mux = slot_wb_base_reg[11];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[11];
                fetch_slot_ring_size_mux = slot_ring_size_reg[11];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[11];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[11];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[11];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[11];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[11];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[11];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[11];
            end
            4'd12: begin
                fetch_slot_valid_mux = slot_valid_reg[12];
                fetch_slot_qid_mux = slot_qid_reg[12];
                fetch_slot_ctrl_mux = slot_ctrl_reg[12];
                fetch_slot_mode_mux = slot_mode_reg[12];
                fetch_slot_desc_base_mux = slot_desc_base_reg[12];
                fetch_slot_wb_base_mux = slot_wb_base_reg[12];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[12];
                fetch_slot_ring_size_mux = slot_ring_size_reg[12];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[12];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[12];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[12];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[12];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[12];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[12];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[12];
            end
            4'd13: begin
                fetch_slot_valid_mux = slot_valid_reg[13];
                fetch_slot_qid_mux = slot_qid_reg[13];
                fetch_slot_ctrl_mux = slot_ctrl_reg[13];
                fetch_slot_mode_mux = slot_mode_reg[13];
                fetch_slot_desc_base_mux = slot_desc_base_reg[13];
                fetch_slot_wb_base_mux = slot_wb_base_reg[13];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[13];
                fetch_slot_ring_size_mux = slot_ring_size_reg[13];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[13];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[13];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[13];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[13];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[13];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[13];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[13];
            end
            4'd14: begin
                fetch_slot_valid_mux = slot_valid_reg[14];
                fetch_slot_qid_mux = slot_qid_reg[14];
                fetch_slot_ctrl_mux = slot_ctrl_reg[14];
                fetch_slot_mode_mux = slot_mode_reg[14];
                fetch_slot_desc_base_mux = slot_desc_base_reg[14];
                fetch_slot_wb_base_mux = slot_wb_base_reg[14];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[14];
                fetch_slot_ring_size_mux = slot_ring_size_reg[14];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[14];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[14];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[14];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[14];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[14];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[14];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[14];
            end
            4'd15: begin
                fetch_slot_valid_mux = slot_valid_reg[15];
                fetch_slot_qid_mux = slot_qid_reg[15];
                fetch_slot_ctrl_mux = slot_ctrl_reg[15];
                fetch_slot_mode_mux = slot_mode_reg[15];
                fetch_slot_desc_base_mux = slot_desc_base_reg[15];
                fetch_slot_wb_base_mux = slot_wb_base_reg[15];
                fetch_slot_irq_enable_mux = slot_irq_enable_reg[15];
                fetch_slot_ring_size_mux = slot_ring_size_reg[15];
                fetch_slot_target_pidx_mux = slot_target_pidx_reg[15];
                fetch_slot_fetch_cidx_mux = slot_fetch_cidx_reg[15];
                fetch_slot_issue_cidx_mux = slot_issue_cidx_reg[15];
                fetch_slot_fetch_inflight_mux = slot_fetch_inflight_reg[15];
                fetch_slot_fetch_stop_mux = slot_fetch_stop_reg[15];
                fetch_slot_perf_desc_mux = slot_perf_desc_reg[15];
                fetch_slot_perf_dma_mux = slot_perf_dma_reg[15];
            end
            default: begin end
        endcase
    end

    always @* begin
        free_slot_valid_mux = 1'b1;
        case (free_scan_ptr_reg)
            4'd0: free_slot_valid_mux = slot_valid_reg[0];
            4'd1: free_slot_valid_mux = slot_valid_reg[1];
            4'd2: free_slot_valid_mux = slot_valid_reg[2];
            4'd3: free_slot_valid_mux = slot_valid_reg[3];
            4'd4: free_slot_valid_mux = slot_valid_reg[4];
            4'd5: free_slot_valid_mux = slot_valid_reg[5];
            4'd6: free_slot_valid_mux = slot_valid_reg[6];
            4'd7: free_slot_valid_mux = slot_valid_reg[7];
            4'd8: free_slot_valid_mux = slot_valid_reg[8];
            4'd9: free_slot_valid_mux = slot_valid_reg[9];
            4'd10: free_slot_valid_mux = slot_valid_reg[10];
            4'd11: free_slot_valid_mux = slot_valid_reg[11];
            4'd12: free_slot_valid_mux = slot_valid_reg[12];
            4'd13: free_slot_valid_mux = slot_valid_reg[13];
            4'd14: free_slot_valid_mux = slot_valid_reg[14];
            4'd15: free_slot_valid_mux = slot_valid_reg[15];
            default: free_slot_valid_mux = 1'b1;
        endcase
    end

    always @* begin
        fetch_sum_comb = {1'b0, fetch_req_cidx_reg} + {{(RING_INDEX_WIDTH+1-5){1'b0}}, fetch_req_count_reg};
        if (fetch_req_ring_size_reg != 0 && fetch_sum_comb >= {1'b0, fetch_req_ring_size_reg}) begin
            fetch_req_next_cidx_comb = fetch_sum_comb[RING_INDEX_WIDTH-1:0] - fetch_req_ring_size_reg;
        end else begin
            fetch_req_next_cidx_comb = fetch_sum_comb[RING_INDEX_WIDTH-1:0];
        end

        if (fetch_resp_ring_size_reg != 0 && fetch_resp_cidx_reg == fetch_resp_ring_size_reg - 1'b1) begin
            fetch_resp_next_cidx_comb = {RING_INDEX_WIDTH{1'b0}};
        end else begin
            fetch_resp_next_cidx_comb = fetch_resp_cidx_reg + 1'b1;
        end
    end

    /* PCIe Memory Read Requests are not allowed to cross a 4 KiB address
     * boundary and must not exceed the negotiated max read request size.
     * Descriptor fetches are issued as host-memory MRd TLPs, so clamp only the
     * batch count that would violate transport limits.  The normal
     * DESC_FETCH_MAX_COUNT/ISSUE_QUANTUM path is unchanged away from those
     * limits.
     */
    always @* begin
        if (fetch_pipe_is_st_reg) begin
            case (max_read_request_size)
                3'd0: fetch_mrrs_desc_limit_comb = 5'd8;   // 128 B / 16 B
                default: fetch_mrrs_desc_limit_comb = 5'd16;
            endcase
        end else begin
            case (max_read_request_size)
                3'd0: fetch_mrrs_desc_limit_comb = 5'd4;   // 128 B / 32 B
                3'd1: fetch_mrrs_desc_limit_comb = 5'd8;   // 256 B / 32 B
                default: fetch_mrrs_desc_limit_comb = 5'd16;
            endcase
        end

        fetch_4kb_bytes_left_comb = 13'd4096 - {1'b0, fetch_pipe_addr_reg[11:0]};
        if (fetch_pipe_is_st_reg) begin
            fetch_4kb_desc_limit_wide_comb = fetch_4kb_bytes_left_comb[12:4];
        end else begin
            fetch_4kb_desc_limit_wide_comb = {1'b0, fetch_4kb_bytes_left_comb[12:5]};
        end

        if (fetch_4kb_desc_limit_wide_comb > 9'd16) begin
            fetch_4kb_desc_limit_comb = 5'd16;
        end else begin
            fetch_4kb_desc_limit_comb = fetch_4kb_desc_limit_wide_comb[4:0];
        end

        if (fetch_pipe_limit_reg > fetch_4kb_desc_limit_comb) begin
            fetch_pipe_limit_pcie_comb = fetch_4kb_desc_limit_comb;
        end else begin
            fetch_pipe_limit_pcie_comb = fetch_pipe_limit_reg;
        end

        if (fetch_pipe_limit_pcie_comb > fetch_mrrs_desc_limit_comb) begin
            fetch_pipe_limit_pcie_comb = fetch_mrrs_desc_limit_comb;
        end
    end

    always @* begin
        if (fetch_pipe_ring_size_reg == 0 || fetch_pipe_pending_reg == 0) begin
            fetch_pipe_limit_eval_comb = 5'd0;
        end else begin
            fetch_boundary_comb = fetch_pipe_ring_size_reg - fetch_pipe_cidx_reg;
            fetch_limit_wide_comb = fetch_pipe_pending_reg;
            if (fetch_limit_wide_comb > {{(RING_INDEX_WIDTH-5){1'b0}}, fetch_pipe_quantum_reg}) begin
                fetch_limit_wide_comb = {{(RING_INDEX_WIDTH-5){1'b0}}, fetch_pipe_quantum_reg};
            end
            if (fetch_limit_wide_comb > fetch_boundary_comb) begin
                fetch_limit_wide_comb = fetch_boundary_comb;
            end
            if (fetch_limit_wide_comb > {{(RING_INDEX_WIDTH-5){1'b0}}, 5'd16}) begin
                fetch_pipe_limit_eval_comb = 5'd16;
            end else if (fetch_limit_wide_comb[4:0] > fetch_pipe_fifo_limit_reg) begin
                fetch_pipe_limit_eval_comb = fetch_pipe_fifo_limit_reg;
            end else if (fetch_limit_wide_comb[4:0] > fetch_pipe_credit_limit_reg) begin
                fetch_pipe_limit_eval_comb = fetch_pipe_credit_limit_reg;
            end else begin
                fetch_pipe_limit_eval_comb = fetch_limit_wide_comb[4:0];
            end
        end

        fetch_idx_comb = (fetch_pipe_ring_size_reg == 0) ? fetch_pipe_cidx_reg :
                         (fetch_pipe_cidx_reg & (fetch_pipe_ring_size_reg - 1'b1));
        fetch_offset_comb = {{(PCIE_ADDR_WIDTH-RING_INDEX_WIDTH){1'b0}}, fetch_idx_comb};
        if (fetch_pipe_is_st_reg) begin
            fetch_pipe_addr_eval_comb = fetch_pipe_desc_base_reg + (fetch_offset_comb << 4);
        end else begin
            fetch_pipe_addr_eval_comb = fetch_pipe_desc_base_reg + (fetch_offset_comb << 5);
        end
    end

    always @(posedge clk) begin : descriptor_fetch_scheduler
        if (desc_fetch_valid_reg && desc_fetch_ready) begin
            desc_fetch_valid_reg <= 1'b0;
            fetch_resp_active_reg <= 1'b1;
            fetch_resp_drain_reg <= 1'b0;
            fetch_resp_slot_reg <= fetch_req_slot_reg;
            fetch_resp_cidx_reg <= fetch_req_cidx_reg;
            fetch_resp_left_reg <= fetch_req_count_reg;
            fetch_resp_is_st_reg <= fetch_req_is_st_reg;
            fetch_resp_qid_reg <= fetch_pipe_qid_reg;
            fetch_resp_mode_reg <= fetch_pipe_mode_reg;
            fetch_resp_irq_enable_reg <= fetch_pipe_irq_enable_reg;
            fetch_resp_wb_base_reg <= fetch_pipe_wb_base_reg;
            fetch_resp_ring_size_reg <= fetch_pipe_ring_size_reg;
            fetch_resp_target_pidx_reg <= fetch_pipe_target_pidx_reg;
        end

        if (desc_resp_fire) begin
            if (!fetch_resp_drain_reg) begin
                fetch_resp_cidx_reg <= fetch_resp_next_cidx_comb;
            end
            if (fetch_resp_left_reg <= 5'd1) begin
                fetch_resp_left_reg <= 5'd0;
                fetch_resp_active_reg <= 1'b0;
                fetch_resp_drain_reg <= 1'b0;
            end else begin
                fetch_resp_left_reg <= fetch_resp_left_reg - 1'b1;
                if (!fetch_resp_drain_reg && resp_terminal_comb) begin
                    fetch_resp_drain_reg <= 1'b1;
                end
            end
        end

        if (!desc_fetch_valid_reg && !fetch_resp_active_reg) begin
            case (fetch_state_reg)
                FETCH_STATE_SCAN: begin
                    fetch_pipe_slot_reg <= fetch_scan_ptr_reg;
                    fetch_pipe_valid_reg <= fetch_slot_valid_mux;
                    fetch_pipe_qid_reg <= fetch_slot_qid_mux;
                    fetch_pipe_ctrl_reg <= fetch_slot_ctrl_mux;
                    fetch_pipe_mode_reg <= fetch_slot_mode_mux;
                    fetch_pipe_desc_base_reg <= fetch_slot_desc_base_mux;
                    fetch_pipe_wb_base_reg <= fetch_slot_wb_base_mux;
                    fetch_pipe_irq_enable_reg <= fetch_slot_irq_enable_mux;
                    fetch_pipe_ring_size_reg <= fetch_slot_ring_size_mux;
                    fetch_pipe_target_pidx_reg <= fetch_slot_target_pidx_mux;
                    fetch_pipe_cidx_reg <= fetch_slot_fetch_cidx_mux;
                    fetch_pipe_inflight_reg <= fetch_slot_fetch_inflight_mux;
                    fetch_pipe_stop_reg <= fetch_slot_fetch_stop_mux;
                    fetch_scan_ptr_reg <= fetch_scan_ptr_reg + 1'b1;
                    fetch_state_reg <= FETCH_STATE_EVAL0;
                end

                FETCH_STATE_EVAL0: begin
                    fetch_pipe_is_st_reg <= fetch_pipe_mode_reg[`EDEV_QMODE_TYPE_ST];
                    fetch_pipe_bad_dir_reg <= fetch_pipe_mode_reg[`EDEV_QMODE_TYPE_ST] &&
                                              (fetch_pipe_mode_reg[`EDEV_QMODE_H2D] == fetch_pipe_mode_reg[`EDEV_QMODE_D2H]);

                    if (fetch_pipe_ring_size_reg == 0) begin
                        fetch_pipe_pending_reg <= 0;
                    end else if (fetch_pipe_target_pidx_reg >= fetch_pipe_cidx_reg) begin
                        fetch_pipe_pending_reg <= fetch_pipe_target_pidx_reg - fetch_pipe_cidx_reg;
                    end else begin
                        fetch_pipe_pending_reg <= (fetch_pipe_ring_size_reg - fetch_pipe_cidx_reg) + fetch_pipe_target_pidx_reg;
                    end

                    fetch_pipe_quantum_reg <= DESC_FETCH_MAX_COUNT_LIMIT;

                    if (fetch_pipe_mode_reg[`EDEV_QMODE_TYPE_ST]) begin
                        if (fetch_pipe_mode_reg[`EDEV_QMODE_H2D] && !fetch_pipe_mode_reg[`EDEV_QMODE_D2H]) begin
                            fetch_pipe_fifo_limit_reg <= st_tx_fetch_space_reg;
                        end else begin
                            fetch_pipe_fifo_limit_reg <= st_rx_fetch_space_reg;
                        end
                    end else begin
                        fetch_pipe_fifo_limit_reg <= mm_fetch_min_space_reg;
                    end

                    if (global_desc_fetch_bytes_reg >= GLOBAL_DESC_FETCH_LIMIT_BYTES) begin
                        fetch_pipe_credit_limit_reg <= 5'd0;
                    end else if (fetch_pipe_mode_reg[`EDEV_QMODE_TYPE_ST]) begin
                        fetch_pipe_credit_limit_reg <= (((GLOBAL_DESC_FETCH_LIMIT_BYTES - global_desc_fetch_bytes_reg) >> 4) > 5'd16) ? 5'd16 :
                                                       ((GLOBAL_DESC_FETCH_LIMIT_BYTES - global_desc_fetch_bytes_reg) >> 4);
                    end else begin
                        fetch_pipe_credit_limit_reg <= (((GLOBAL_DESC_FETCH_LIMIT_BYTES - global_desc_fetch_bytes_reg) >> 5) > 5'd16) ? 5'd16 :
                                                       ((GLOBAL_DESC_FETCH_LIMIT_BYTES - global_desc_fetch_bytes_reg) >> 5);
                    end

                    fetch_state_reg <= FETCH_STATE_EVAL1;
                end

                FETCH_STATE_EVAL1: begin
                    fetch_pipe_limit_reg <= fetch_pipe_limit_eval_comb;
                    fetch_pipe_addr_reg <= fetch_pipe_addr_eval_comb;
                    fetch_state_reg <= FETCH_STATE_EVAL2;
                end

                default: begin
                    if (fetch_pipe_valid_reg && global_enable && !global_pause &&
                        fetch_pipe_ctrl_reg[`EDEV_CTRL_ENABLE] && !fetch_pipe_ctrl_reg[`EDEV_CTRL_PAUSE] &&
                        fetch_pipe_ring_size_reg != 0 && !fetch_pipe_stop_reg && !fetch_pipe_bad_dir_reg &&
                        fetch_pipe_inflight_reg < 6'd32 && fetch_pipe_pending_reg != 0 && fetch_pipe_limit_pcie_comb != 0) begin
                        desc_fetch_addr_reg <= fetch_pipe_addr_reg;
                        desc_fetch_tag_reg <= {{(TAG_WIDTH-SLOT_INDEX_WIDTH){1'b0}}, fetch_pipe_slot_reg};
                        desc_fetch_count_reg <= fetch_pipe_limit_pcie_comb;
                        desc_fetch_is_st_reg <= fetch_pipe_is_st_reg;
                        desc_fetch_valid_reg <= 1'b1;
                        fetch_req_slot_reg <= fetch_pipe_slot_reg;
                        fetch_req_cidx_reg <= fetch_pipe_cidx_reg;
                        fetch_req_count_reg <= fetch_pipe_limit_pcie_comb;
                        fetch_req_is_st_reg <= fetch_pipe_is_st_reg;
                        fetch_req_ring_size_reg <= fetch_pipe_ring_size_reg;
                    end
                    fetch_state_reg <= FETCH_STATE_SCAN;
                end
            endcase
        end

        if (rst) begin
            fetch_scan_ptr_reg <= 0;
            fetch_state_reg <= FETCH_STATE_SCAN;
            desc_fetch_addr_reg <= 0;
            desc_fetch_tag_reg <= 0;
            desc_fetch_count_reg <= 0;
            desc_fetch_is_st_reg <= 1'b0;
            desc_fetch_valid_reg <= 1'b0;
            fetch_resp_active_reg <= 1'b0;
            fetch_resp_left_reg <= 0;
            fetch_resp_drain_reg <= 1'b0;
            fetch_resp_qid_reg <= 0;
            fetch_resp_mode_reg <= 0;
            fetch_resp_irq_enable_reg <= 0;
            fetch_resp_wb_base_reg <= 0;
            fetch_resp_ring_size_reg <= 0;
            fetch_resp_target_pidx_reg <= 0;
        end
    end

    /* Descriptor response decode. */
    reg [DESC_ENTRY_WIDTH-1:0] resp_entry_comb;
    reg [1:0] resp_path_comb;
    reg resp_push_valid_comb;
    reg resp_path_space_ok_comb;
    reg [15:0] resp_flags_comb;
    reg resp_dir_comb;
    reg [31:0] resp_error_comb;
    reg [QID_WIDTH-1:0] resp_st_qid_comb;
    reg [RING_INDEX_WIDTH-1:0] resp_next_cidx_comb;
    reg [RING_INDEX_WIDTH-1:0] resp_cmpl_slot_comb;

    always @* begin
        resp_entry_comb = {DESC_ENTRY_WIDTH{1'b0}};
        resp_path_comb = PATH_MM_H2D;
        resp_push_valid_comb = fetch_resp_active_reg && !fetch_resp_drain_reg;
        resp_path_space_ok_comb = 1'b0;
        resp_error_comb = 32'd0;
        if (fetch_resp_ring_size_reg != 0 && fetch_resp_cidx_reg == fetch_resp_ring_size_reg - 1'b1) begin
            resp_next_cidx_comb = {RING_INDEX_WIDTH{1'b0}};
        end else begin
            resp_next_cidx_comb = fetch_resp_cidx_reg + 1'b1;
        end

        /*
         * The software ABI allocates exactly one 32-byte coherent WB record
         * per queue and programs QREG_CMPL_BASE to that record.  Completion
         * index advancement is reported inside the record; it is not a
         * completion-ring slot selector.  Keep the legacy field at zero so a
         * descriptor-ring cidx can never be converted into a host WB offset.
         */
        resp_cmpl_slot_comb = {RING_INDEX_WIDTH{1'b0}};

        if (fetch_resp_mode_reg[`EDEV_QMODE_TYPE_ST]) begin
            resp_flags_comb = 16'd0;
            resp_flags_comb[`EDEV_DESC_VALID] = 1'b1;
            resp_flags_comb[`EDEV_DESC_SOP] = 1'b1;
            resp_flags_comb[`EDEV_DESC_EOP] = 1'b1;
            resp_flags_comb[`EDEV_DESC_IRQ_ON_DONE] = fetch_resp_irq_enable_reg[`EDEV_IRQ_CMPL];
            resp_dir_comb = fetch_resp_mode_reg[`EDEV_QMODE_D2H];
            resp_st_qid_comb = desc_resp_data[(112)+(QID_WIDTH)-1:(112)];
            if (desc_resp_error != 4'd0) begin
                resp_error_comb = {`EDEV_ERR_DESC_FETCH_CPL, 8'h00, {12'd0, desc_resp_error}};
            end else if (desc_resp_tag[SLOT_INDEX_WIDTH-1:0] != fetch_resp_slot_reg) begin
                resp_error_comb = {`EDEV_ERR_QID_MISMATCH, 8'h11, {{(16-SLOT_INDEX_WIDTH){1'b0}}, fetch_resp_slot_reg}};
            end else if (resp_st_qid_comb != fetch_resp_qid_reg) begin
                resp_error_comb = {`EDEV_ERR_QID_MISMATCH, 8'h12, {{(16-QID_WIDTH){1'b0}}, fetch_resp_qid_reg}};
            end else if (desc_resp_data[64+LEN_WIDTH-1:64] == 0) begin
                resp_error_comb = {`EDEV_ERR_DESC_LENGTH, fetch_resp_cidx_reg[7:0], 16'd0};
            end
            resp_entry_comb[DESC_ENTRY_HOST_ADDR_MSB:DESC_ENTRY_HOST_ADDR_LSB] = desc_resp_data[PCIE_ADDR_WIDTH-1:0];
            resp_entry_comb[DESC_ENTRY_DEV_ADDR_MSB:DESC_ENTRY_DEV_ADDR_LSB] = {AXI_ADDR_WIDTH{1'b0}};
            resp_entry_comb[DESC_ENTRY_LEN_MSB:DESC_ENTRY_LEN_LSB] = desc_resp_data[64+LEN_WIDTH-1:64];
            resp_entry_comb[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB] = {{(32-LEN_WIDTH){1'b0}}, desc_resp_data[64+LEN_WIDTH-1:64]};
            resp_entry_comb[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB] = desc_resp_data[111:96];
            resp_path_comb = resp_dir_comb ? PATH_ST_RX : PATH_ST_TX;
            resp_path_space_ok_comb = resp_dir_comb ? st_rx_space_avail_reg : st_tx_space_avail_reg;
        end else begin
            resp_flags_comb = desc_resp_data[191:176];
            resp_dir_comb = desc_resp_data[176+`EDEV_DESC_DIR];
            if (desc_resp_error != 4'd0) begin
                resp_error_comb = {`EDEV_ERR_DESC_FETCH_CPL, 8'h00, {12'd0, desc_resp_error}};
            end else if (desc_resp_tag[SLOT_INDEX_WIDTH-1:0] != fetch_resp_slot_reg) begin
                resp_error_comb = {`EDEV_ERR_QID_MISMATCH, 8'h21, {{(16-SLOT_INDEX_WIDTH){1'b0}}, fetch_resp_slot_reg}};
            end else if (!desc_resp_data[176+`EDEV_DESC_VALID]) begin
                resp_error_comb = {`EDEV_ERR_DESC_INVALID, fetch_resp_cidx_reg[7:0], desc_resp_data[191:176]};
            end else if (desc_resp_data[128+LEN_WIDTH-1:128] == 0) begin
                resp_error_comb = {`EDEV_ERR_DESC_LENGTH, fetch_resp_cidx_reg[7:0], 16'd0};
            end else if (!resp_dir_comb && !fetch_resp_mode_reg[`EDEV_QMODE_H2D]) begin
                resp_error_comb = {`EDEV_ERR_QUEUE_DIR, 8'h00, {15'd0, 1'b0}};
            end else if (resp_dir_comb && !fetch_resp_mode_reg[`EDEV_QMODE_D2H]) begin
                resp_error_comb = {`EDEV_ERR_QUEUE_DIR, 8'h00, {15'd0, 1'b1}};
            end
            resp_entry_comb[DESC_ENTRY_HOST_ADDR_MSB:DESC_ENTRY_HOST_ADDR_LSB] = desc_resp_data[PCIE_ADDR_WIDTH-1:0];
            resp_entry_comb[DESC_ENTRY_DEV_ADDR_MSB:DESC_ENTRY_DEV_ADDR_LSB] = desc_resp_data[(64)+(AXI_ADDR_WIDTH)-1:(64)];
            resp_entry_comb[DESC_ENTRY_LEN_MSB:DESC_ENTRY_LEN_LSB] = desc_resp_data[128+LEN_WIDTH-1:128];
            if (desc_resp_data[255:224] != 32'd0) begin
                resp_entry_comb[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB] = desc_resp_data[255:224];
            end else begin
                resp_entry_comb[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB] = {{(32-LEN_WIDTH){1'b0}}, desc_resp_data[128+LEN_WIDTH-1:128]};
            end
            resp_entry_comb[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB] = desc_resp_data[175:160];
            resp_path_comb = resp_dir_comb ? PATH_MM_D2H : PATH_MM_H2D;
            resp_path_space_ok_comb = resp_dir_comb ? mm_d2h_space_avail_reg : mm_h2d_space_avail_reg;
        end

        resp_terminal_comb = (resp_error_comb != 0) ||
                             (resp_next_cidx_comb == fetch_resp_target_pidx_reg) ||
                             resp_flags_comb[`EDEV_DESC_EOP] ||
                             fetch_resp_mode_reg[`EDEV_QMODE_TYPE_ST];

        resp_entry_comb[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB] = fetch_resp_qid_reg;
        resp_entry_comb[DESC_ENTRY_SLOT_MSB:DESC_ENTRY_SLOT_LSB] = fetch_resp_slot_reg;
        resp_entry_comb[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB] = fetch_resp_wb_base_reg;
        resp_entry_comb[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB] = fetch_resp_irq_enable_reg;
        resp_entry_comb[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB] = resp_next_cidx_comb;
        resp_entry_comb[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB] = resp_cmpl_slot_comb;
        resp_entry_comb[DESC_ENTRY_TARGET_PIDX_MSB:DESC_ENTRY_TARGET_PIDX_LSB] = fetch_resp_target_pidx_reg;
        resp_entry_comb[DESC_ENTRY_FLAGS_MSB:DESC_ENTRY_FLAGS_LSB] = resp_flags_comb;
        resp_entry_comb[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB] = resp_dir_comb;
        resp_entry_comb[DESC_ENTRY_IS_ST_MSB:DESC_ENTRY_IS_ST_LSB] = fetch_resp_mode_reg[`EDEV_QMODE_TYPE_ST];
        resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB] = resp_terminal_comb;
        resp_entry_comb[DESC_ENTRY_ERROR_CODE_MSB:DESC_ENTRY_ERROR_CODE_LSB] = resp_error_comb;
    end

    assign desc_resp_ready = fetch_resp_active_reg && (fetch_resp_drain_reg || resp_path_space_ok_comb);

    /* Path FIFO push/pop owner.
     * Counts are updated with explicit per-FIFO push/pop cases so a response
     * arriving on the same path that the issue side pops in the same cycle does
     * not corrupt the level counter.  This is intentionally local to four small
     * path FIFOs, avoiding a high-fanout per-qid descriptor FIFO structure.
     */
    wire issue_pop = issue_entry_valid_reg &&
                     ((issue_path_reg == PATH_MM_H2D && dma_read_desc_valid && dma_read_desc_ready) ||
                      (issue_path_reg == PATH_MM_D2H && dma_write_desc_valid && dma_write_desc_ready) ||
                      (issue_path_reg == PATH_ST_TX  && st_tx_desc_valid && st_tx_desc_ready) ||
                      (issue_path_reg == PATH_ST_RX  && st_rx_desc_valid && st_rx_desc_ready));

    reg issue_drop_error_event_reg = 1'b0;
    wire issue_error_pop = issue_drop_error_event_reg;
    wire path_pop = issue_pop || issue_error_pop;
    wire issue_mm_requeue_after_terminal =
        issue_entry_reg[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB] &&
        !issue_entry_reg[DESC_ENTRY_IS_ST_MSB:DESC_ENTRY_IS_ST_LSB] &&
        (issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB] !=
         issue_entry_reg[DESC_ENTRY_TARGET_PIDX_MSB:DESC_ENTRY_TARGET_PIDX_LSB]);
    wire issue_st_rx_requeue_after_terminal =
        issue_entry_reg[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB] &&
        issue_entry_reg[DESC_ENTRY_IS_ST_MSB:DESC_ENTRY_IS_ST_LSB] &&
        issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB] &&
        (issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB] !=
         issue_entry_reg[DESC_ENTRY_TARGET_PIDX_MSB:DESC_ENTRY_TARGET_PIDX_LSB]);
    wire mm_h2d_push = desc_resp_fire && resp_push_valid_comb && resp_path_comb == PATH_MM_H2D;
    wire mm_d2h_push = desc_resp_fire && resp_push_valid_comb && resp_path_comb == PATH_MM_D2H;
    wire st_tx_push  = desc_resp_fire && resp_push_valid_comb && resp_path_comb == PATH_ST_TX;
    wire st_rx_push  = desc_resp_fire && resp_push_valid_comb && resp_path_comb == PATH_ST_RX;
    wire mm_h2d_pop  = path_pop && issue_path_reg == PATH_MM_H2D;
    wire mm_d2h_pop  = path_pop && issue_path_reg == PATH_MM_D2H;
    wire st_tx_pop   = path_pop && issue_path_reg == PATH_ST_TX;
    wire st_rx_pop   = path_pop && issue_path_reg == PATH_ST_RX;

    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] mm_h2d_count_next_comb;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] mm_d2h_count_next_comb;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] st_tx_count_next_comb;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] st_rx_count_next_comb;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] mm_h2d_space_next_comb;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] mm_d2h_space_next_comb;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] st_tx_space_next_comb;
    reg [STATIC_PATH_FIFO_ADDR_WIDTH:0] st_rx_space_next_comb;
    reg [4:0]                   mm_h2d_fetch_space_next_comb;
    reg [4:0]                   mm_d2h_fetch_space_next_comb;
    reg [4:0]                   st_tx_fetch_space_next_comb;
    reg [4:0]                   st_rx_fetch_space_next_comb;
    reg [4:0]                   mm_fetch_min_space_next_comb;

    always @* begin
        mm_h2d_count_next_comb = mm_h2d_count_reg;
        mm_d2h_count_next_comb = mm_d2h_count_reg;
        st_tx_count_next_comb = st_tx_count_reg;
        st_rx_count_next_comb = st_rx_count_reg;
        mm_h2d_space_next_comb = mm_h2d_space_reg;
        mm_d2h_space_next_comb = mm_d2h_space_reg;
        st_tx_space_next_comb = st_tx_space_reg;
        st_rx_space_next_comb = st_rx_space_reg;

        case ({mm_h2d_push, mm_h2d_pop})
            2'b10: begin mm_h2d_count_next_comb = mm_h2d_count_reg + 1'b1; mm_h2d_space_next_comb = mm_h2d_space_reg - 1'b1; end
            2'b01: begin mm_h2d_count_next_comb = mm_h2d_count_reg - 1'b1; mm_h2d_space_next_comb = mm_h2d_space_reg + 1'b1; end
            default: begin end
        endcase
        case ({mm_d2h_push, mm_d2h_pop})
            2'b10: begin mm_d2h_count_next_comb = mm_d2h_count_reg + 1'b1; mm_d2h_space_next_comb = mm_d2h_space_reg - 1'b1; end
            2'b01: begin mm_d2h_count_next_comb = mm_d2h_count_reg - 1'b1; mm_d2h_space_next_comb = mm_d2h_space_reg + 1'b1; end
            default: begin end
        endcase
        case ({st_tx_push, st_tx_pop})
            2'b10: begin st_tx_count_next_comb = st_tx_count_reg + 1'b1; st_tx_space_next_comb = st_tx_space_reg - 1'b1; end
            2'b01: begin st_tx_count_next_comb = st_tx_count_reg - 1'b1; st_tx_space_next_comb = st_tx_space_reg + 1'b1; end
            default: begin end
        endcase
        case ({st_rx_push, st_rx_pop})
            2'b10: begin st_rx_count_next_comb = st_rx_count_reg + 1'b1; st_rx_space_next_comb = st_rx_space_reg - 1'b1; end
            2'b01: begin st_rx_count_next_comb = st_rx_count_reg - 1'b1; st_rx_space_next_comb = st_rx_space_reg + 1'b1; end
            default: begin end
        endcase

        mm_h2d_fetch_space_next_comb = (mm_h2d_space_next_comb > 5'd16) ? 5'd16 : mm_h2d_space_next_comb[4:0];
        mm_d2h_fetch_space_next_comb = (mm_d2h_space_next_comb > 5'd16) ? 5'd16 : mm_d2h_space_next_comb[4:0];
        st_tx_fetch_space_next_comb  = (st_tx_space_next_comb  > 5'd16) ? 5'd16 : st_tx_space_next_comb[4:0];
        st_rx_fetch_space_next_comb  = (st_rx_space_next_comb  > 5'd16) ? 5'd16 : st_rx_space_next_comb[4:0];

        if (mm_h2d_fetch_space_next_comb < mm_d2h_fetch_space_next_comb) begin
            mm_fetch_min_space_next_comb = mm_h2d_fetch_space_next_comb;
        end else begin
            mm_fetch_min_space_next_comb = mm_d2h_fetch_space_next_comb;
        end
    end

    always @(posedge clk) begin : descriptor_path_fifos
        if (mm_h2d_push) begin
            mm_h2d_fifo_mem[mm_h2d_wr_ptr_reg] <= resp_entry_comb;
            mm_h2d_wr_ptr_reg <= mm_h2d_wr_ptr_reg + 1'b1;
        end
        if (mm_d2h_push) begin
            mm_d2h_fifo_mem[mm_d2h_wr_ptr_reg] <= resp_entry_comb;
            mm_d2h_wr_ptr_reg <= mm_d2h_wr_ptr_reg + 1'b1;
        end
        if (st_tx_push) begin
            st_tx_fifo_mem[st_tx_wr_ptr_reg] <= resp_entry_comb;
            st_tx_wr_ptr_reg <= st_tx_wr_ptr_reg + 1'b1;
        end
        if (st_rx_push) begin
            st_rx_fifo_mem[st_rx_wr_ptr_reg] <= resp_entry_comb;
            st_rx_wr_ptr_reg <= st_rx_wr_ptr_reg + 1'b1;
        end

        if (mm_h2d_pop) mm_h2d_rd_ptr_reg <= mm_h2d_rd_ptr_reg + 1'b1;
        if (mm_d2h_pop) mm_d2h_rd_ptr_reg <= mm_d2h_rd_ptr_reg + 1'b1;
        if (st_tx_pop)  st_tx_rd_ptr_reg  <= st_tx_rd_ptr_reg  + 1'b1;
        if (st_rx_pop)  st_rx_rd_ptr_reg  <= st_rx_rd_ptr_reg  + 1'b1;

        mm_h2d_count_reg <= mm_h2d_count_next_comb;
        mm_d2h_count_reg <= mm_d2h_count_next_comb;
        st_tx_count_reg  <= st_tx_count_next_comb;
        st_rx_count_reg  <= st_rx_count_next_comb;
        mm_h2d_space_reg <= mm_h2d_space_next_comb;
        mm_d2h_space_reg <= mm_d2h_space_next_comb;
        st_tx_space_reg  <= st_tx_space_next_comb;
        st_rx_space_reg  <= st_rx_space_next_comb;

        mm_h2d_not_empty_reg <= (mm_h2d_count_next_comb != 0);
        mm_d2h_not_empty_reg <= (mm_d2h_count_next_comb != 0);
        st_tx_not_empty_reg  <= (st_tx_count_next_comb != 0);
        st_rx_not_empty_reg  <= (st_rx_count_next_comb != 0);
        mm_h2d_space_avail_reg <= (mm_h2d_space_next_comb != 0);
        mm_d2h_space_avail_reg <= (mm_d2h_space_next_comb != 0);
        st_tx_space_avail_reg  <= (st_tx_space_next_comb != 0);
        st_rx_space_avail_reg  <= (st_rx_space_next_comb != 0);
        mm_h2d_fetch_space_reg <= mm_h2d_fetch_space_next_comb;
        mm_d2h_fetch_space_reg <= mm_d2h_fetch_space_next_comb;
        st_tx_fetch_space_reg  <= st_tx_fetch_space_next_comb;
        st_rx_fetch_space_reg  <= st_rx_fetch_space_next_comb;
        mm_fetch_min_space_reg <= mm_fetch_min_space_next_comb;

        if (rst) begin
            mm_h2d_wr_ptr_reg <= 0; mm_h2d_rd_ptr_reg <= 0; mm_h2d_count_reg <= 0;
            mm_d2h_wr_ptr_reg <= 0; mm_d2h_rd_ptr_reg <= 0; mm_d2h_count_reg <= 0;
            st_tx_wr_ptr_reg <= 0;  st_tx_rd_ptr_reg <= 0;  st_tx_count_reg <= 0;
            st_rx_wr_ptr_reg <= 0;  st_rx_rd_ptr_reg <= 0;  st_rx_count_reg <= 0;
            mm_h2d_space_reg <= PATH_FIFO_LEVEL_MAX;
            mm_d2h_space_reg <= PATH_FIFO_LEVEL_MAX;
            st_tx_space_reg  <= PATH_FIFO_LEVEL_MAX;
            st_rx_space_reg  <= PATH_FIFO_LEVEL_MAX;
            mm_h2d_not_empty_reg <= 1'b0;
            mm_d2h_not_empty_reg <= 1'b0;
            st_tx_not_empty_reg  <= 1'b0;
            st_rx_not_empty_reg  <= 1'b0;
            mm_h2d_space_avail_reg <= 1'b1;
            mm_d2h_space_avail_reg <= 1'b1;
            st_tx_space_avail_reg  <= 1'b1;
            st_rx_space_avail_reg  <= 1'b1;
            mm_h2d_fetch_space_reg <= PATH_FIFO_FETCH_LIMIT;
            mm_d2h_fetch_space_reg <= PATH_FIFO_FETCH_LIMIT;
            st_tx_fetch_space_reg  <= PATH_FIFO_FETCH_LIMIT;
            st_rx_fetch_space_reg  <= PATH_FIFO_FETCH_LIMIT;
            mm_fetch_min_space_reg <= PATH_FIFO_FETCH_LIMIT;
        end
    end

    /* ---------------------------------------------------------------------
     * Payload issue scheduler.  This state is independent from descriptor
     * fetch.  It selects from path FIFOs, holds one descriptor until the target
     * accepts it, and only uses tag/free-tag state for MM descriptors.
     * ------------------------------------------------------------------ */
    reg [PCIE_ADDR_WIDTH-1:0] issue_host_addr_reg = 0;
    reg [AXI_ADDR_WIDTH-1:0]  issue_dev_addr_reg = 0;
    reg [LEN_WIDTH-1:0]       issue_len_reg = 0;
    reg [TAG_WIDTH-1:0]       issue_dma_tag_reg = 0;
    reg [QID_WIDTH-1:0]       issue_qid_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] issue_cidx_after_reg = 0;
    reg [15:0]                issue_flags_reg = 0;
    reg [15:0]                issue_swtag_reg = 0;
    reg                       dma_read_valid_reg = 1'b0;
    reg                       dma_write_valid_reg = 1'b0;
    reg                       st_tx_valid_reg = 1'b0;
    reg                       st_rx_valid_reg = 1'b0;
    reg [MM_TAG_INDEX_WIDTH-1:0] issue_tag_index_reg = 0;
    reg [MM_TAG_NUM_WIDTH-1:0] issue_tag_num_reg = 0;

    reg [MM_TAG_COUNT-1:0] tag_bitmap_reg = {MM_TAG_COUNT{1'b0}};
    reg [MM_TAG_COUNT-1:0] tag_alloc_pending_reg = {MM_TAG_COUNT{1'b0}};
    reg                         tag_commit_clear_valid_reg = 1'b0;
    reg [MM_TAG_INDEX_WIDTH-1:0] tag_commit_clear_index_reg = 0;
    reg [MM_TAG_COUNT-1:0] tag_dir_reg = {MM_TAG_COUNT{1'b0}};
    reg [MM_TAG_COUNT-1:0] wb_ready_bitmap_reg = {MM_TAG_COUNT{1'b0}};
    /* MM completions can return out of order across PCIe/AXI tags.
     * Keep a small explicit issue-order FIFO so queue writebacks are
     * emitted in descriptor issue order.  This protects the driver's
     * single-WB-per-queue ABI from observing a later EOP before an older
     * descriptor has really retired, without reducing the 16-tag payload
     * concurrency.
     */
    reg [MM_TAG_COUNT-1:0] tag_done_bitmap_reg = {MM_TAG_COUNT{1'b0}};
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_0_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_1_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_2_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_3_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_4_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_5_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_6_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_7_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_8_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_9_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_10_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_11_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_12_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_13_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_14_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_tag_15_reg = 0;
    reg [MM_TAG_INDEX_WIDTH:0] mm_order_wr_ptr_reg = 0;
    reg [MM_TAG_INDEX_WIDTH:0] mm_order_rd_ptr_reg = 0;
    reg [MM_TAG_INDEX_WIDTH:0] mm_order_count_reg = 0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_order_head_index_comb;
    reg mm_order_head_done_comb;
    reg mm_order_head_wb_ready_comb;
    reg [MM_TAG_COUNT-1:0] tag_sop_reg = {MM_TAG_COUNT{1'b0}};
    reg [MM_TAG_COUNT-1:0] tag_eop_reg = {MM_TAG_COUNT{1'b0}};
    reg [MM_TAG_COUNT-1:0] tag_irq_reg = {MM_TAG_COUNT{1'b0}};
    reg [QID_WIDTH-1:0] tag_qid_reg [0:MM_TAG_COUNT-1];
    reg [PCIE_ADDR_WIDTH-1:0] tag_wb_base_reg [0:MM_TAG_COUNT-1];
    reg [31:0] tag_irq_enable_reg [0:MM_TAG_COUNT-1];
    reg [15:0] tag_swtag_reg [0:MM_TAG_COUNT-1];
    reg [RING_INDEX_WIDTH-1:0] tag_cidx_after_reg [0:MM_TAG_COUNT-1];
    reg [RING_INDEX_WIDTH-1:0] tag_cmpl_slot_reg [0:MM_TAG_COUNT-1];
    reg [31:0] tag_bytes_reg [0:MM_TAG_COUNT-1];
    reg [31:0] tag_error_code_reg [0:MM_TAG_COUNT-1];

    reg mm_free_tag_valid_comb;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_free_tag_index_comb;
    reg [MM_TAG_NUM_WIDTH-1:0] mm_free_tag_num_comb;

    always @* begin
        mm_free_tag_valid_comb = 1'b0;
        mm_free_tag_index_comb = 0;
        mm_free_tag_num_comb = 1;
        if (!tag_bitmap_reg[0] && !tag_alloc_pending_reg[0] && !wb_ready_bitmap_reg[0] && !tag_done_bitmap_reg[0]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd0;
            mm_free_tag_num_comb = 5'd1;
        end else if (!tag_bitmap_reg[1] && !tag_alloc_pending_reg[1] && !wb_ready_bitmap_reg[1] && !tag_done_bitmap_reg[1]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd1;
            mm_free_tag_num_comb = 5'd2;
        end else if (!tag_bitmap_reg[2] && !tag_alloc_pending_reg[2] && !wb_ready_bitmap_reg[2] && !tag_done_bitmap_reg[2]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd2;
            mm_free_tag_num_comb = 5'd3;
        end else if (!tag_bitmap_reg[3] && !tag_alloc_pending_reg[3] && !wb_ready_bitmap_reg[3] && !tag_done_bitmap_reg[3]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd3;
            mm_free_tag_num_comb = 5'd4;
        end else if (!tag_bitmap_reg[4] && !tag_alloc_pending_reg[4] && !wb_ready_bitmap_reg[4] && !tag_done_bitmap_reg[4]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd4;
            mm_free_tag_num_comb = 5'd5;
        end else if (!tag_bitmap_reg[5] && !tag_alloc_pending_reg[5] && !wb_ready_bitmap_reg[5] && !tag_done_bitmap_reg[5]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd5;
            mm_free_tag_num_comb = 5'd6;
        end else if (!tag_bitmap_reg[6] && !tag_alloc_pending_reg[6] && !wb_ready_bitmap_reg[6] && !tag_done_bitmap_reg[6]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd6;
            mm_free_tag_num_comb = 5'd7;
        end else if (!tag_bitmap_reg[7] && !tag_alloc_pending_reg[7] && !wb_ready_bitmap_reg[7] && !tag_done_bitmap_reg[7]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd7;
            mm_free_tag_num_comb = 5'd8;
        end else if (!tag_bitmap_reg[8] && !tag_alloc_pending_reg[8] && !wb_ready_bitmap_reg[8] && !tag_done_bitmap_reg[8]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd8;
            mm_free_tag_num_comb = 5'd9;
        end else if (!tag_bitmap_reg[9] && !tag_alloc_pending_reg[9] && !wb_ready_bitmap_reg[9] && !tag_done_bitmap_reg[9]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd9;
            mm_free_tag_num_comb = 5'd10;
        end else if (!tag_bitmap_reg[10] && !tag_alloc_pending_reg[10] && !wb_ready_bitmap_reg[10] && !tag_done_bitmap_reg[10]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd10;
            mm_free_tag_num_comb = 5'd11;
        end else if (!tag_bitmap_reg[11] && !tag_alloc_pending_reg[11] && !wb_ready_bitmap_reg[11] && !tag_done_bitmap_reg[11]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd11;
            mm_free_tag_num_comb = 5'd12;
        end else if (!tag_bitmap_reg[12] && !tag_alloc_pending_reg[12] && !wb_ready_bitmap_reg[12] && !tag_done_bitmap_reg[12]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd12;
            mm_free_tag_num_comb = 5'd13;
        end else if (!tag_bitmap_reg[13] && !tag_alloc_pending_reg[13] && !wb_ready_bitmap_reg[13] && !tag_done_bitmap_reg[13]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd13;
            mm_free_tag_num_comb = 5'd14;
        end else if (!tag_bitmap_reg[14] && !tag_alloc_pending_reg[14] && !wb_ready_bitmap_reg[14] && !tag_done_bitmap_reg[14]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd14;
            mm_free_tag_num_comb = 5'd15;
        end else if (!tag_bitmap_reg[15] && !tag_alloc_pending_reg[15] && !wb_ready_bitmap_reg[15] && !tag_done_bitmap_reg[15]) begin
            mm_free_tag_valid_comb = 1'b1;
            mm_free_tag_index_comb = 4'd15;
            mm_free_tag_num_comb = 5'd16;
        end
    end
    reg [1:0] next_issue_path_comb;
    reg [DESC_ENTRY_WIDTH-1:0] next_issue_entry_comb;
    reg next_issue_valid_comb;

    always @* begin
        next_issue_valid_comb = 1'b0;
        next_issue_path_comb = issue_rr_reg;
        next_issue_entry_comb = {DESC_ENTRY_WIDTH{1'b0}};
        case (issue_rr_reg)
            PATH_MM_H2D: begin
                if (!mm_h2d_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_MM_H2D; next_issue_entry_comb = mm_h2d_fifo_rd_mux; end
                else if (!mm_d2h_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_MM_D2H; next_issue_entry_comb = mm_d2h_fifo_rd_mux; end
                else if (!st_tx_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_ST_TX; next_issue_entry_comb = st_tx_fifo_rd_mux; end
                else if (!st_rx_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_ST_RX; next_issue_entry_comb = st_rx_fifo_rd_mux; end
            end
            PATH_MM_D2H: begin
                if (!mm_d2h_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_MM_D2H; next_issue_entry_comb = mm_d2h_fifo_rd_mux; end
                else if (!st_tx_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_ST_TX; next_issue_entry_comb = st_tx_fifo_rd_mux; end
                else if (!st_rx_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_ST_RX; next_issue_entry_comb = st_rx_fifo_rd_mux; end
                else if (!mm_h2d_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_MM_H2D; next_issue_entry_comb = mm_h2d_fifo_rd_mux; end
            end
            PATH_ST_TX: begin
                if (!st_tx_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_ST_TX; next_issue_entry_comb = st_tx_fifo_rd_mux; end
                else if (!st_rx_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_ST_RX; next_issue_entry_comb = st_rx_fifo_rd_mux; end
                else if (!mm_h2d_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_MM_H2D; next_issue_entry_comb = mm_h2d_fifo_rd_mux; end
                else if (!mm_d2h_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_MM_D2H; next_issue_entry_comb = mm_d2h_fifo_rd_mux; end
            end
            default: begin
                if (!st_rx_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_ST_RX; next_issue_entry_comb = st_rx_fifo_rd_mux; end
                else if (!mm_h2d_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_MM_H2D; next_issue_entry_comb = mm_h2d_fifo_rd_mux; end
                else if (!mm_d2h_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_MM_D2H; next_issue_entry_comb = mm_d2h_fifo_rd_mux; end
                else if (!st_tx_empty) begin next_issue_valid_comb = 1'b1; next_issue_path_comb = PATH_ST_TX; next_issue_entry_comb = st_tx_fifo_rd_mux; end
            end
        endcase
    end

    reg update_valid_reg = 1'b0;
    reg update_pend_valid_reg = 1'b0;
    reg update_pend_requeue_reg = 1'b0;
    reg [QID_WIDTH-1:0] update_pend_qid_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] update_pend_desc_cidx_reg = 0;
    reg [31:0] update_pend_status_reg = 0;
    reg [31:0] update_pend_error_code_reg = 0;
    reg [31:0] update_pend_perf_desc_reg = 0;
    reg [31:0] update_pend_perf_dma_reg = 0;

    reg [31:0] issue_perf_desc_mux;
    reg [31:0] issue_perf_dma_mux;
    reg [31:0] next_issue_perf_desc_mux;
    reg [31:0] next_issue_perf_dma_mux;

    always @* begin
        issue_perf_desc_mux = 32'd0;
        issue_perf_dma_mux = 32'd0;
        case (issue_entry_reg[DESC_ENTRY_SLOT_MSB:DESC_ENTRY_SLOT_LSB])
            4'd0: begin issue_perf_desc_mux = slot_perf_desc_reg[0]; issue_perf_dma_mux = slot_perf_dma_reg[0]; end
            4'd1: begin issue_perf_desc_mux = slot_perf_desc_reg[1]; issue_perf_dma_mux = slot_perf_dma_reg[1]; end
            4'd2: begin issue_perf_desc_mux = slot_perf_desc_reg[2]; issue_perf_dma_mux = slot_perf_dma_reg[2]; end
            4'd3: begin issue_perf_desc_mux = slot_perf_desc_reg[3]; issue_perf_dma_mux = slot_perf_dma_reg[3]; end
            4'd4: begin issue_perf_desc_mux = slot_perf_desc_reg[4]; issue_perf_dma_mux = slot_perf_dma_reg[4]; end
            4'd5: begin issue_perf_desc_mux = slot_perf_desc_reg[5]; issue_perf_dma_mux = slot_perf_dma_reg[5]; end
            4'd6: begin issue_perf_desc_mux = slot_perf_desc_reg[6]; issue_perf_dma_mux = slot_perf_dma_reg[6]; end
            4'd7: begin issue_perf_desc_mux = slot_perf_desc_reg[7]; issue_perf_dma_mux = slot_perf_dma_reg[7]; end
            4'd8: begin issue_perf_desc_mux = slot_perf_desc_reg[8]; issue_perf_dma_mux = slot_perf_dma_reg[8]; end
            4'd9: begin issue_perf_desc_mux = slot_perf_desc_reg[9]; issue_perf_dma_mux = slot_perf_dma_reg[9]; end
            4'd10: begin issue_perf_desc_mux = slot_perf_desc_reg[10]; issue_perf_dma_mux = slot_perf_dma_reg[10]; end
            4'd11: begin issue_perf_desc_mux = slot_perf_desc_reg[11]; issue_perf_dma_mux = slot_perf_dma_reg[11]; end
            4'd12: begin issue_perf_desc_mux = slot_perf_desc_reg[12]; issue_perf_dma_mux = slot_perf_dma_reg[12]; end
            4'd13: begin issue_perf_desc_mux = slot_perf_desc_reg[13]; issue_perf_dma_mux = slot_perf_dma_reg[13]; end
            4'd14: begin issue_perf_desc_mux = slot_perf_desc_reg[14]; issue_perf_dma_mux = slot_perf_dma_reg[14]; end
            4'd15: begin issue_perf_desc_mux = slot_perf_desc_reg[15]; issue_perf_dma_mux = slot_perf_dma_reg[15]; end
            default: begin end
        endcase
    end

    always @* begin
        next_issue_perf_desc_mux = 32'd0;
        next_issue_perf_dma_mux = 32'd0;
        case (next_issue_entry_comb[DESC_ENTRY_SLOT_MSB:DESC_ENTRY_SLOT_LSB])
            4'd0: begin next_issue_perf_desc_mux = slot_perf_desc_reg[0]; next_issue_perf_dma_mux = slot_perf_dma_reg[0]; end
            4'd1: begin next_issue_perf_desc_mux = slot_perf_desc_reg[1]; next_issue_perf_dma_mux = slot_perf_dma_reg[1]; end
            4'd2: begin next_issue_perf_desc_mux = slot_perf_desc_reg[2]; next_issue_perf_dma_mux = slot_perf_dma_reg[2]; end
            4'd3: begin next_issue_perf_desc_mux = slot_perf_desc_reg[3]; next_issue_perf_dma_mux = slot_perf_dma_reg[3]; end
            4'd4: begin next_issue_perf_desc_mux = slot_perf_desc_reg[4]; next_issue_perf_dma_mux = slot_perf_dma_reg[4]; end
            4'd5: begin next_issue_perf_desc_mux = slot_perf_desc_reg[5]; next_issue_perf_dma_mux = slot_perf_dma_reg[5]; end
            4'd6: begin next_issue_perf_desc_mux = slot_perf_desc_reg[6]; next_issue_perf_dma_mux = slot_perf_dma_reg[6]; end
            4'd7: begin next_issue_perf_desc_mux = slot_perf_desc_reg[7]; next_issue_perf_dma_mux = slot_perf_dma_reg[7]; end
            4'd8: begin next_issue_perf_desc_mux = slot_perf_desc_reg[8]; next_issue_perf_dma_mux = slot_perf_dma_reg[8]; end
            4'd9: begin next_issue_perf_desc_mux = slot_perf_desc_reg[9]; next_issue_perf_dma_mux = slot_perf_dma_reg[9]; end
            4'd10: begin next_issue_perf_desc_mux = slot_perf_desc_reg[10]; next_issue_perf_dma_mux = slot_perf_dma_reg[10]; end
            4'd11: begin next_issue_perf_desc_mux = slot_perf_desc_reg[11]; next_issue_perf_dma_mux = slot_perf_dma_reg[11]; end
            4'd12: begin next_issue_perf_desc_mux = slot_perf_desc_reg[12]; next_issue_perf_dma_mux = slot_perf_dma_reg[12]; end
            4'd13: begin next_issue_perf_desc_mux = slot_perf_desc_reg[13]; next_issue_perf_dma_mux = slot_perf_dma_reg[13]; end
            4'd14: begin next_issue_perf_desc_mux = slot_perf_desc_reg[14]; next_issue_perf_dma_mux = slot_perf_dma_reg[14]; end
            4'd15: begin next_issue_perf_desc_mux = slot_perf_desc_reg[15]; next_issue_perf_dma_mux = slot_perf_dma_reg[15]; end
            default: begin end
        endcase
    end


    reg update_op_done_reg = 1'b0;
    reg update_requeue_reg = 1'b0;
    reg update_desc_we_reg = 1'b0;
    reg update_cmpl_we_reg = 1'b0;
    reg [QID_WIDTH-1:0] update_qid_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] update_hw_desc_cidx_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] update_cmpl_pidx_reg = 0;
    reg [31:0] update_status_reg = 0;
    reg [31:0] update_error_code_reg = 0;
    reg [31:0] update_perf_desc_fetch_reg = 0;
    reg [31:0] update_perf_dma_submit_reg = 0;
    reg [31:0] update_perf_wb_write_reg = 0;
    reg [31:0] update_perf_bytes_reg = 0;

    reg [STATIC_WB_FIFO_ADDR_WIDTH:0] wb_fifo_count_reg = 0;
    wire wb_fifo_full = wb_fifo_count_reg == STATIC_WB_FIFO_DEPTH;
    wire wb_fifo_empty = wb_fifo_count_reg == 0;

    reg [STATIC_ST_STAT_FIFO_ADDR_WIDTH:0] st_stat_count_reg = 0;
    reg st_status_rr_reg = 1'b0;
    wire st_stat_fifo_full = st_stat_count_reg == STATIC_ST_STAT_FIFO_DEPTH;
    wire st_stat_fifo_empty = st_stat_count_reg == 0;
    wire st_status_sel_rx = st_rx_status_valid && (!st_tx_status_valid || st_status_rr_reg);
    wire [SLOT_INDEX_WIDTH-1:0] st_tx_status_slot = st_tx_status_tag[SLOT_INDEX_WIDTH-1:0];
    wire st_status_accept = (st_tx_status_valid || st_rx_status_valid) && !st_stat_fifo_full;
    wire st_tx_status_accept = st_status_accept && !st_status_sel_rx;
    wire st_rx_status_accept = st_status_accept && st_status_sel_rx;
    assign st_tx_status_ready = st_tx_status_accept;
    assign st_rx_status_ready = st_rx_status_accept;

    /* Maintenance scan handles contexts that produce no descriptor response:
     * disabled/paused queues, zero-size rings, invalid ST direction, or an
     * already-empty ring.  It scans one slot per cycle instead of building a
     * wide qid bitmap or high-fanout priority encoder.
     */
    reg [SLOT_INDEX_WIDTH-1:0] maint_scan_ptr_reg = 0;
    reg maint_clear_valid_reg = 1'b0;
    reg [SLOT_INDEX_WIDTH-1:0] maint_clear_slot_reg = 0;
    reg maint_slot_valid_mux;
    reg [QID_WIDTH-1:0] maint_slot_qid_mux;
    reg [31:0] maint_slot_ctrl_mux;
    reg [31:0] maint_slot_mode_mux;
    reg [RING_INDEX_WIDTH-1:0] maint_slot_ring_size_mux;
    reg [RING_INDEX_WIDTH-1:0] maint_slot_target_pidx_mux;
    reg [RING_INDEX_WIDTH-1:0] maint_slot_fetch_cidx_mux;
    reg [RING_INDEX_WIDTH-1:0] maint_slot_issue_cidx_mux;
    reg [5:0] maint_slot_fetch_inflight_mux;
    reg [31:0] maint_slot_perf_desc_mux;
    reg [31:0] maint_slot_perf_dma_mux;

    always @* begin
        maint_slot_valid_mux = 1'b0;
        maint_slot_qid_mux = {QID_WIDTH{1'b0}};
        maint_slot_ctrl_mux = 32'd0;
        maint_slot_mode_mux = 32'd0;
        maint_slot_ring_size_mux = {RING_INDEX_WIDTH{1'b0}};
        maint_slot_target_pidx_mux = {RING_INDEX_WIDTH{1'b0}};
        maint_slot_fetch_cidx_mux = {RING_INDEX_WIDTH{1'b0}};
        maint_slot_issue_cidx_mux = {RING_INDEX_WIDTH{1'b0}};
        maint_slot_fetch_inflight_mux = 6'd0;
        maint_slot_perf_desc_mux = 32'd0;
        maint_slot_perf_dma_mux = 32'd0;
        case (maint_scan_ptr_reg)
            4'd0: begin
                maint_slot_valid_mux = slot_valid_reg[0];
                maint_slot_qid_mux = slot_qid_reg[0];
                maint_slot_ctrl_mux = slot_ctrl_reg[0];
                maint_slot_mode_mux = slot_mode_reg[0];
                maint_slot_ring_size_mux = slot_ring_size_reg[0];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[0];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[0];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[0];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[0];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[0];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[0];
            end
            4'd1: begin
                maint_slot_valid_mux = slot_valid_reg[1];
                maint_slot_qid_mux = slot_qid_reg[1];
                maint_slot_ctrl_mux = slot_ctrl_reg[1];
                maint_slot_mode_mux = slot_mode_reg[1];
                maint_slot_ring_size_mux = slot_ring_size_reg[1];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[1];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[1];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[1];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[1];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[1];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[1];
            end
            4'd2: begin
                maint_slot_valid_mux = slot_valid_reg[2];
                maint_slot_qid_mux = slot_qid_reg[2];
                maint_slot_ctrl_mux = slot_ctrl_reg[2];
                maint_slot_mode_mux = slot_mode_reg[2];
                maint_slot_ring_size_mux = slot_ring_size_reg[2];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[2];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[2];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[2];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[2];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[2];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[2];
            end
            4'd3: begin
                maint_slot_valid_mux = slot_valid_reg[3];
                maint_slot_qid_mux = slot_qid_reg[3];
                maint_slot_ctrl_mux = slot_ctrl_reg[3];
                maint_slot_mode_mux = slot_mode_reg[3];
                maint_slot_ring_size_mux = slot_ring_size_reg[3];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[3];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[3];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[3];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[3];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[3];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[3];
            end
            4'd4: begin
                maint_slot_valid_mux = slot_valid_reg[4];
                maint_slot_qid_mux = slot_qid_reg[4];
                maint_slot_ctrl_mux = slot_ctrl_reg[4];
                maint_slot_mode_mux = slot_mode_reg[4];
                maint_slot_ring_size_mux = slot_ring_size_reg[4];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[4];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[4];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[4];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[4];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[4];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[4];
            end
            4'd5: begin
                maint_slot_valid_mux = slot_valid_reg[5];
                maint_slot_qid_mux = slot_qid_reg[5];
                maint_slot_ctrl_mux = slot_ctrl_reg[5];
                maint_slot_mode_mux = slot_mode_reg[5];
                maint_slot_ring_size_mux = slot_ring_size_reg[5];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[5];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[5];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[5];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[5];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[5];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[5];
            end
            4'd6: begin
                maint_slot_valid_mux = slot_valid_reg[6];
                maint_slot_qid_mux = slot_qid_reg[6];
                maint_slot_ctrl_mux = slot_ctrl_reg[6];
                maint_slot_mode_mux = slot_mode_reg[6];
                maint_slot_ring_size_mux = slot_ring_size_reg[6];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[6];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[6];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[6];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[6];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[6];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[6];
            end
            4'd7: begin
                maint_slot_valid_mux = slot_valid_reg[7];
                maint_slot_qid_mux = slot_qid_reg[7];
                maint_slot_ctrl_mux = slot_ctrl_reg[7];
                maint_slot_mode_mux = slot_mode_reg[7];
                maint_slot_ring_size_mux = slot_ring_size_reg[7];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[7];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[7];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[7];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[7];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[7];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[7];
            end
            4'd8: begin
                maint_slot_valid_mux = slot_valid_reg[8];
                maint_slot_qid_mux = slot_qid_reg[8];
                maint_slot_ctrl_mux = slot_ctrl_reg[8];
                maint_slot_mode_mux = slot_mode_reg[8];
                maint_slot_ring_size_mux = slot_ring_size_reg[8];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[8];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[8];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[8];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[8];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[8];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[8];
            end
            4'd9: begin
                maint_slot_valid_mux = slot_valid_reg[9];
                maint_slot_qid_mux = slot_qid_reg[9];
                maint_slot_ctrl_mux = slot_ctrl_reg[9];
                maint_slot_mode_mux = slot_mode_reg[9];
                maint_slot_ring_size_mux = slot_ring_size_reg[9];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[9];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[9];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[9];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[9];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[9];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[9];
            end
            4'd10: begin
                maint_slot_valid_mux = slot_valid_reg[10];
                maint_slot_qid_mux = slot_qid_reg[10];
                maint_slot_ctrl_mux = slot_ctrl_reg[10];
                maint_slot_mode_mux = slot_mode_reg[10];
                maint_slot_ring_size_mux = slot_ring_size_reg[10];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[10];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[10];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[10];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[10];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[10];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[10];
            end
            4'd11: begin
                maint_slot_valid_mux = slot_valid_reg[11];
                maint_slot_qid_mux = slot_qid_reg[11];
                maint_slot_ctrl_mux = slot_ctrl_reg[11];
                maint_slot_mode_mux = slot_mode_reg[11];
                maint_slot_ring_size_mux = slot_ring_size_reg[11];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[11];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[11];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[11];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[11];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[11];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[11];
            end
            4'd12: begin
                maint_slot_valid_mux = slot_valid_reg[12];
                maint_slot_qid_mux = slot_qid_reg[12];
                maint_slot_ctrl_mux = slot_ctrl_reg[12];
                maint_slot_mode_mux = slot_mode_reg[12];
                maint_slot_ring_size_mux = slot_ring_size_reg[12];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[12];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[12];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[12];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[12];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[12];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[12];
            end
            4'd13: begin
                maint_slot_valid_mux = slot_valid_reg[13];
                maint_slot_qid_mux = slot_qid_reg[13];
                maint_slot_ctrl_mux = slot_ctrl_reg[13];
                maint_slot_mode_mux = slot_mode_reg[13];
                maint_slot_ring_size_mux = slot_ring_size_reg[13];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[13];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[13];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[13];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[13];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[13];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[13];
            end
            4'd14: begin
                maint_slot_valid_mux = slot_valid_reg[14];
                maint_slot_qid_mux = slot_qid_reg[14];
                maint_slot_ctrl_mux = slot_ctrl_reg[14];
                maint_slot_mode_mux = slot_mode_reg[14];
                maint_slot_ring_size_mux = slot_ring_size_reg[14];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[14];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[14];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[14];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[14];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[14];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[14];
            end
            4'd15: begin
                maint_slot_valid_mux = slot_valid_reg[15];
                maint_slot_qid_mux = slot_qid_reg[15];
                maint_slot_ctrl_mux = slot_ctrl_reg[15];
                maint_slot_mode_mux = slot_mode_reg[15];
                maint_slot_ring_size_mux = slot_ring_size_reg[15];
                maint_slot_target_pidx_mux = slot_target_pidx_reg[15];
                maint_slot_fetch_cidx_mux = slot_fetch_cidx_reg[15];
                maint_slot_issue_cidx_mux = slot_issue_cidx_reg[15];
                maint_slot_fetch_inflight_mux = slot_fetch_inflight_reg[15];
                maint_slot_perf_desc_mux = slot_perf_desc_reg[15];
                maint_slot_perf_dma_mux = slot_perf_dma_reg[15];
            end
            default: begin end
        endcase
    end

    wire maint_slot_is_st = maint_slot_mode_mux[`EDEV_QMODE_TYPE_ST];
    wire maint_slot_st_bad_dir = maint_slot_is_st &&
                                  (maint_slot_mode_mux[`EDEV_QMODE_H2D] ==
                                   maint_slot_mode_mux[`EDEV_QMODE_D2H]);
    wire maint_slot_no_fifo_shadow = maint_slot_fetch_inflight_mux == 0 &&
                                     maint_slot_fetch_cidx_mux == maint_slot_issue_cidx_mux;
    wire maint_slot_ring_empty = maint_slot_fetch_cidx_mux == maint_slot_target_pidx_mux &&
                                 maint_slot_issue_cidx_mux == maint_slot_target_pidx_mux;
    wire maint_slot_invalid = maint_slot_valid_mux && global_enable && !global_pause &&
                              maint_slot_no_fifo_shadow &&
                              (!maint_slot_ctrl_mux[`EDEV_CTRL_ENABLE] ||
                               maint_slot_ctrl_mux[`EDEV_CTRL_PAUSE] ||
                               maint_slot_ring_size_mux == 0 ||
                               maint_slot_st_bad_dir ||
                               maint_slot_ring_empty);

    /* Events into completion/writeback block. */
    reg mm_issue_event_reg = 1'b0;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_issue_tag_index_reg = 0;
    reg [MM_TAG_NUM_WIDTH-1:0] mm_issue_tag_num_reg = 0;
    reg [DESC_ENTRY_WIDTH-1:0] mm_issue_entry_reg;
    reg issue_error_wb_event_reg = 1'b0;
    reg issue_error_update_event_reg = 1'b0;
    reg [DESC_ENTRY_WIDTH-1:0] issue_error_entry_reg;
    reg [31:0] issue_error_perf_desc_reg = 0;
    reg [31:0] issue_error_perf_dma_reg = 0;

    wire issue_candidate_valid = next_issue_valid_comb && !issue_entry_valid_reg && !update_pend_valid_reg && !issue_error_update_event_reg;
    wire issue_mm_needs_tag = next_issue_valid_comb && !next_issue_entry_comb[DESC_ENTRY_IS_ST_MSB:DESC_ENTRY_IS_ST_LSB] && next_issue_entry_comb[DESC_ENTRY_ERROR_CODE_MSB:DESC_ENTRY_ERROR_CODE_LSB] == 0;
    wire mm_order_space_av;
    wire issue_start_allowed = issue_candidate_valid &&
                               (next_issue_entry_comb[DESC_ENTRY_ERROR_CODE_MSB:DESC_ENTRY_ERROR_CODE_LSB] == 0 || !wb_fifo_full) &&
                               (!issue_mm_needs_tag || (mm_free_tag_valid_comb && mm_order_space_av));

    reg [MM_TAG_COUNT-1:0] tag_alloc_set_mask_comb;
    reg [MM_TAG_COUNT-1:0] tag_alloc_clear_mask_comb;
    always @* begin
        tag_alloc_set_mask_comb = {MM_TAG_COUNT{1'b0}};
        if (issue_start_allowed && issue_mm_needs_tag) begin
            case (mm_free_tag_index_comb)
                4'd0:  tag_alloc_set_mask_comb = 16'h0001;
                4'd1:  tag_alloc_set_mask_comb = 16'h0002;
                4'd2:  tag_alloc_set_mask_comb = 16'h0004;
                4'd3:  tag_alloc_set_mask_comb = 16'h0008;
                4'd4:  tag_alloc_set_mask_comb = 16'h0010;
                4'd5:  tag_alloc_set_mask_comb = 16'h0020;
                4'd6:  tag_alloc_set_mask_comb = 16'h0040;
                4'd7:  tag_alloc_set_mask_comb = 16'h0080;
                4'd8:  tag_alloc_set_mask_comb = 16'h0100;
                4'd9:  tag_alloc_set_mask_comb = 16'h0200;
                4'd10: tag_alloc_set_mask_comb = 16'h0400;
                4'd11: tag_alloc_set_mask_comb = 16'h0800;
                4'd12: tag_alloc_set_mask_comb = 16'h1000;
                4'd13: tag_alloc_set_mask_comb = 16'h2000;
                4'd14: tag_alloc_set_mask_comb = 16'h4000;
                default: tag_alloc_set_mask_comb = 16'h8000;
            endcase
        end

        tag_alloc_clear_mask_comb = {MM_TAG_COUNT{1'b0}};
        if (tag_commit_clear_valid_reg) begin
            case (tag_commit_clear_index_reg)
                4'd0:  tag_alloc_clear_mask_comb = 16'h0001;
                4'd1:  tag_alloc_clear_mask_comb = 16'h0002;
                4'd2:  tag_alloc_clear_mask_comb = 16'h0004;
                4'd3:  tag_alloc_clear_mask_comb = 16'h0008;
                4'd4:  tag_alloc_clear_mask_comb = 16'h0010;
                4'd5:  tag_alloc_clear_mask_comb = 16'h0020;
                4'd6:  tag_alloc_clear_mask_comb = 16'h0040;
                4'd7:  tag_alloc_clear_mask_comb = 16'h0080;
                4'd8:  tag_alloc_clear_mask_comb = 16'h0100;
                4'd9:  tag_alloc_clear_mask_comb = 16'h0200;
                4'd10: tag_alloc_clear_mask_comb = 16'h0400;
                4'd11: tag_alloc_clear_mask_comb = 16'h0800;
                4'd12: tag_alloc_clear_mask_comb = 16'h1000;
                4'd13: tag_alloc_clear_mask_comb = 16'h2000;
                4'd14: tag_alloc_clear_mask_comb = 16'h4000;
                default: tag_alloc_clear_mask_comb = 16'h8000;
            endcase
        end
    end

    always @(posedge clk) begin : payload_issue_scheduler
        mm_issue_event_reg <= 1'b0;
        issue_error_wb_event_reg <= 1'b0;
        issue_error_update_event_reg <= 1'b0;
        issue_drop_error_event_reg <= 1'b0;
        maint_clear_valid_reg <= 1'b0;
        update_valid_reg <= 1'b0;
        maint_scan_ptr_reg <= maint_scan_ptr_reg + 1'b1;

        /*
         * Preload the update payload every cycle.  ctx_update_data is sampled
         * only while update_valid_reg is asserted, so holding these registers
         * behind update_pend_valid_reg is unnecessary.  In the implemented
         * design that enable was replicated onto 174 register CEs and formed
         * a routing-only critical path.  Keeping only the valid pulse gated
         * preserves the interface semantics while removing that high-fanout
         * clock-enable network.
         */
        update_op_done_reg <= 1'b1;
        update_requeue_reg <= update_pend_requeue_reg;
        update_desc_we_reg <= 1'b1;
        update_cmpl_we_reg <= 1'b0;
        update_qid_reg <= update_pend_qid_reg;
        update_hw_desc_cidx_reg <= update_pend_desc_cidx_reg;
        update_cmpl_pidx_reg <= 0;
        update_status_reg <= update_pend_status_reg;
        update_error_code_reg <= update_pend_error_code_reg;
        update_perf_desc_fetch_reg <= update_pend_perf_desc_reg;
        update_perf_dma_submit_reg <= update_pend_perf_dma_reg;
        update_perf_wb_write_reg <= 32'd0;
        update_perf_bytes_reg <= 32'd0;

        tag_alloc_pending_reg <= (tag_alloc_pending_reg | tag_alloc_set_mask_comb) & ~tag_alloc_clear_mask_comb;
        if (tag_commit_clear_valid_reg) begin
            tag_commit_clear_valid_reg <= 1'b0;
        end

        if (update_pend_valid_reg) begin
            update_valid_reg <= 1'b1;
            update_pend_valid_reg <= 1'b0;
        end

        if (issue_error_update_event_reg && !update_pend_valid_reg) begin
            update_pend_valid_reg <= 1'b1;
            update_pend_requeue_reg <= 1'b0;
            update_pend_qid_reg <= issue_error_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
            update_pend_desc_cidx_reg <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
            update_pend_status_reg <= 32'd1 << `EDEV_STATUS_FATAL_ERROR;
            update_pend_error_code_reg <= issue_error_entry_reg[DESC_ENTRY_ERROR_CODE_MSB:DESC_ENTRY_ERROR_CODE_LSB];
            update_pend_perf_desc_reg <= issue_error_perf_desc_reg;
            update_pend_perf_dma_reg <= issue_error_perf_dma_reg;
        end

        if (dma_read_valid_reg && dma_read_desc_ready) begin
            dma_read_valid_reg <= 1'b0;
            issue_entry_valid_reg <= 1'b0;
            mm_issue_event_reg <= 1'b1;
            /* Keep the scheduler tag reserved for one extra cycle after the
             * downstream DMA engine accepts the descriptor.  The completion
             * block consumes mm_issue_event_reg on the following clock edge;
             * releasing tag_alloc_pending_reg here creates a one-cycle hole in
             * which the same MM tag can be reissued while its metadata has not
             * yet been committed to tag_bitmap_reg/tag_*_reg.
             */
            tag_commit_clear_valid_reg <= 1'b1;
            tag_commit_clear_index_reg <= issue_tag_index_reg;
            mm_issue_tag_index_reg <= issue_tag_index_reg;
            mm_issue_tag_num_reg <= issue_tag_num_reg;
            mm_issue_entry_reg <= issue_entry_reg;
            if (issue_entry_reg[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                update_pend_valid_reg <= 1'b1;
                update_pend_requeue_reg <= issue_mm_requeue_after_terminal;
                update_pend_qid_reg <= issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                update_pend_desc_cidx_reg <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                update_pend_status_reg <= 32'd0;
                update_pend_error_code_reg <= 32'd0;
                update_pend_perf_desc_reg <= issue_perf_desc_mux;
                update_pend_perf_dma_reg <= issue_perf_dma_mux + 1'b1;
            end
        end

        if (dma_write_valid_reg && dma_write_desc_ready) begin
            dma_write_valid_reg <= 1'b0;
            issue_entry_valid_reg <= 1'b0;
            mm_issue_event_reg <= 1'b1;
            tag_commit_clear_valid_reg <= 1'b1;
            tag_commit_clear_index_reg <= issue_tag_index_reg;
            mm_issue_tag_index_reg <= issue_tag_index_reg;
            mm_issue_tag_num_reg <= issue_tag_num_reg;
            mm_issue_entry_reg <= issue_entry_reg;
            if (issue_entry_reg[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                update_pend_valid_reg <= 1'b1;
                update_pend_requeue_reg <= issue_mm_requeue_after_terminal;
                update_pend_qid_reg <= issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                update_pend_desc_cidx_reg <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                update_pend_status_reg <= 32'd0;
                update_pend_error_code_reg <= 32'd0;
                update_pend_perf_desc_reg <= issue_perf_desc_mux;
                update_pend_perf_dma_reg <= issue_perf_dma_mux + 1'b1;
            end
        end

        if (st_tx_valid_reg && st_tx_desc_ready) begin
            st_tx_valid_reg <= 1'b0;
            issue_entry_valid_reg <= 1'b0;
            if (issue_entry_reg[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                update_pend_valid_reg <= 1'b1;
                update_pend_requeue_reg <= 1'b0;
                update_pend_qid_reg <= issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                update_pend_desc_cidx_reg <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                update_pend_status_reg <= 32'd0;
                update_pend_error_code_reg <= 32'd0;
                update_pend_perf_desc_reg <= issue_perf_desc_mux;
                update_pend_perf_dma_reg <= issue_perf_dma_mux + 1'b1;
            end
        end

        if (st_rx_valid_reg && st_rx_desc_ready) begin
            st_rx_valid_reg <= 1'b0;
            issue_entry_valid_reg <= 1'b0;
            if (issue_entry_reg[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                update_pend_valid_reg <= 1'b1;
                update_pend_requeue_reg <= issue_st_rx_requeue_after_terminal;
                update_pend_qid_reg <= issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                update_pend_desc_cidx_reg <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                update_pend_status_reg <= 32'd0;
                update_pend_error_code_reg <= 32'd0;
                update_pend_perf_desc_reg <= issue_perf_desc_mux;
                update_pend_perf_dma_reg <= issue_perf_dma_mux + 1'b1;
            end
        end

        if (maint_slot_invalid && !update_pend_valid_reg && !issue_error_update_event_reg && !issue_entry_valid_reg && !issue_candidate_valid) begin
            update_pend_valid_reg <= 1'b1;
            update_pend_requeue_reg <= 1'b0;
            update_pend_qid_reg <= maint_slot_qid_mux;
            update_pend_desc_cidx_reg <= maint_slot_issue_cidx_mux;
            if (maint_slot_ring_size_mux == 0) begin
                update_pend_status_reg <= 32'd1 << `EDEV_STATUS_RING_EMPTY;
                update_pend_error_code_reg <= {`EDEV_ERR_DESC_LENGTH, 8'h00, 16'd0};
            end else if (maint_slot_st_bad_dir) begin
                update_pend_status_reg <= 32'd1 << `EDEV_STATUS_FATAL_ERROR;
                update_pend_error_code_reg <= {`EDEV_ERR_QUEUE_DIR, 8'h02, 16'd0};
            end else begin
                update_pend_status_reg <= 32'd0;
                update_pend_error_code_reg <= 32'd0;
            end
            update_pend_perf_desc_reg <= maint_slot_perf_desc_mux;
            update_pend_perf_dma_reg <= maint_slot_perf_dma_mux;
            maint_clear_valid_reg <= 1'b1;
            maint_clear_slot_reg <= maint_scan_ptr_reg;
        end

        if (issue_start_allowed) begin
            issue_path_reg <= next_issue_path_comb;
            issue_rr_reg <= next_issue_path_comb + 2'd1;
            issue_entry_reg <= next_issue_entry_comb;
            issue_qid_reg <= next_issue_entry_comb[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
            issue_host_addr_reg <= next_issue_entry_comb[DESC_ENTRY_HOST_ADDR_MSB:DESC_ENTRY_HOST_ADDR_LSB];
            issue_dev_addr_reg <= next_issue_entry_comb[DESC_ENTRY_DEV_ADDR_MSB:DESC_ENTRY_DEV_ADDR_LSB];
            issue_len_reg <= next_issue_entry_comb[DESC_ENTRY_LEN_MSB:DESC_ENTRY_LEN_LSB];
            issue_cidx_after_reg <= next_issue_entry_comb[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
            issue_flags_reg <= next_issue_entry_comb[DESC_ENTRY_FLAGS_MSB:DESC_ENTRY_FLAGS_LSB];
            issue_swtag_reg <= next_issue_entry_comb[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
            issue_entry_valid_reg <= 1'b1;

            if (next_issue_entry_comb[DESC_ENTRY_ERROR_CODE_MSB:DESC_ENTRY_ERROR_CODE_LSB] != 0) begin
                issue_error_wb_event_reg <= 1'b1;
                issue_error_update_event_reg <= 1'b1;
                issue_error_entry_reg <= next_issue_entry_comb;
                issue_error_perf_desc_reg <= next_issue_perf_desc_mux;
                issue_error_perf_dma_reg <= next_issue_perf_dma_mux;
                issue_entry_valid_reg <= 1'b0;
                issue_drop_error_event_reg <= 1'b1;
            end else if (!next_issue_entry_comb[DESC_ENTRY_IS_ST_MSB:DESC_ENTRY_IS_ST_LSB]) begin
                issue_tag_index_reg <= mm_free_tag_index_comb;
                issue_tag_num_reg <= mm_free_tag_num_comb;
                issue_dma_tag_reg <= {{(TAG_WIDTH-MM_TAG_NUM_WIDTH){1'b0}}, mm_free_tag_num_comb};
                if (next_issue_path_comb == PATH_MM_H2D) begin
                    dma_read_valid_reg <= 1'b1;
                end else begin
                    dma_write_valid_reg <= 1'b1;
                end
            end else begin
                issue_dma_tag_reg <= {{(TAG_WIDTH-SLOT_INDEX_WIDTH){1'b0}}, next_issue_entry_comb[DESC_ENTRY_SLOT_MSB:DESC_ENTRY_SLOT_LSB]};
                if (next_issue_path_comb == PATH_ST_TX) begin
                    st_tx_valid_reg <= 1'b1;
                end else begin
                    st_rx_valid_reg <= 1'b1;
                end
            end
        end

        if (rst) begin
            issue_rr_reg <= 0;
            issue_entry_valid_reg <= 1'b0;
            dma_read_valid_reg <= 1'b0;
            dma_write_valid_reg <= 1'b0;
            st_tx_valid_reg <= 1'b0;
            st_rx_valid_reg <= 1'b0;
            update_valid_reg <= 1'b0;
            update_pend_valid_reg <= 1'b0;
            maint_scan_ptr_reg <= 0;
            maint_clear_valid_reg <= 1'b0;
            tag_alloc_pending_reg <= {MM_TAG_COUNT{1'b0}};
            tag_commit_clear_valid_reg <= 1'b0;
            tag_commit_clear_index_reg <= 0;
            mm_issue_event_reg <= 1'b0;
            issue_error_wb_event_reg <= 1'b0;
            issue_error_update_event_reg <= 1'b0;
        end
    end

    wire issue_slot_reclaim = issue_pop && issue_entry_reg[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB] &&
                              (!issue_entry_reg[DESC_ENTRY_IS_ST_MSB:DESC_ENTRY_IS_ST_LSB] || issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB]);
    wire slot_clear_maint_event = maint_clear_valid_reg;
    /* MM descriptors and ST RX descriptors release the active scheduler slot
     * as soon as the downstream engine accepts the descriptor.  ST TX keeps
     * the slot until its active transmit engine returns status.  RX packet
     * completion is passive and is reclaimed from the RX descriptor pool, not
     * from this scheduler slot table.
     */
    wire slot_clear_issue_event = issue_slot_reclaim;
    wire slot_clear_error_event = issue_error_pop;
    wire slot_clear_st_event = st_tx_status_accept;
    wire slot_free_event = slot_clear_maint_event || slot_clear_issue_event ||
                           slot_clear_error_event || slot_clear_st_event;

    /* Queue-state owner: context load, fetch credit accounting and issue cidx.
     * All active-slot reads on scan paths are taken from explicit case muxes
     * above; all writes below use explicit case decode as well.  This avoids
     * variable-index active-slot accesses in the scheduler control path while
     * preserving event priority from the previous implementation.
     */
    always @(posedge clk) begin : queue_state_table_owner
        if (!free_slot_valid_reg) begin
            if (!free_slot_valid_mux) begin
                free_slot_valid_reg <= 1'b1;
                free_slot_index_reg <= free_scan_ptr_reg;
            end else begin
                free_scan_ptr_reg <= free_scan_ptr_reg + 1'b1;
            end
        end

        case ({ctx_fire, slot_free_event})
            2'b10: slot_occupancy_count_reg <= slot_occupancy_count_reg + 1'b1;
            2'b01: begin
                if (slot_occupancy_count_reg != 0) begin
                    slot_occupancy_count_reg <= slot_occupancy_count_reg - 1'b1;
                end
            end
            default: slot_occupancy_count_reg <= slot_occupancy_count_reg;
        endcase

        if (ctx_fire) begin
            free_slot_valid_reg <= 1'b0;
            free_scan_ptr_reg <= free_slot_index_reg + 1'b1;
            case (free_slot_index_reg)
                4'd0: begin
                    slot_valid_reg[0] <= 1'b1;
                    slot_qid_reg[0] <= ctx_bus_qid;
                    slot_ctrl_reg[0] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[0] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[0] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[0] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[0] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[0] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[0] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[0] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[0] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[0] <= 6'd0;
                    slot_fetch_stop_reg[0] <= 1'b0;
                    slot_perf_desc_reg[0] <= 32'd0;
                    slot_perf_dma_reg[0] <= 32'd0;
                end
                4'd1: begin
                    slot_valid_reg[1] <= 1'b1;
                    slot_qid_reg[1] <= ctx_bus_qid;
                    slot_ctrl_reg[1] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[1] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[1] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[1] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[1] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[1] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[1] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[1] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[1] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[1] <= 6'd0;
                    slot_fetch_stop_reg[1] <= 1'b0;
                    slot_perf_desc_reg[1] <= 32'd0;
                    slot_perf_dma_reg[1] <= 32'd0;
                end
                4'd2: begin
                    slot_valid_reg[2] <= 1'b1;
                    slot_qid_reg[2] <= ctx_bus_qid;
                    slot_ctrl_reg[2] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[2] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[2] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[2] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[2] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[2] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[2] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[2] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[2] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[2] <= 6'd0;
                    slot_fetch_stop_reg[2] <= 1'b0;
                    slot_perf_desc_reg[2] <= 32'd0;
                    slot_perf_dma_reg[2] <= 32'd0;
                end
                4'd3: begin
                    slot_valid_reg[3] <= 1'b1;
                    slot_qid_reg[3] <= ctx_bus_qid;
                    slot_ctrl_reg[3] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[3] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[3] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[3] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[3] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[3] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[3] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[3] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[3] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[3] <= 6'd0;
                    slot_fetch_stop_reg[3] <= 1'b0;
                    slot_perf_desc_reg[3] <= 32'd0;
                    slot_perf_dma_reg[3] <= 32'd0;
                end
                4'd4: begin
                    slot_valid_reg[4] <= 1'b1;
                    slot_qid_reg[4] <= ctx_bus_qid;
                    slot_ctrl_reg[4] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[4] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[4] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[4] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[4] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[4] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[4] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[4] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[4] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[4] <= 6'd0;
                    slot_fetch_stop_reg[4] <= 1'b0;
                    slot_perf_desc_reg[4] <= 32'd0;
                    slot_perf_dma_reg[4] <= 32'd0;
                end
                4'd5: begin
                    slot_valid_reg[5] <= 1'b1;
                    slot_qid_reg[5] <= ctx_bus_qid;
                    slot_ctrl_reg[5] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[5] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[5] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[5] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[5] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[5] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[5] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[5] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[5] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[5] <= 6'd0;
                    slot_fetch_stop_reg[5] <= 1'b0;
                    slot_perf_desc_reg[5] <= 32'd0;
                    slot_perf_dma_reg[5] <= 32'd0;
                end
                4'd6: begin
                    slot_valid_reg[6] <= 1'b1;
                    slot_qid_reg[6] <= ctx_bus_qid;
                    slot_ctrl_reg[6] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[6] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[6] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[6] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[6] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[6] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[6] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[6] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[6] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[6] <= 6'd0;
                    slot_fetch_stop_reg[6] <= 1'b0;
                    slot_perf_desc_reg[6] <= 32'd0;
                    slot_perf_dma_reg[6] <= 32'd0;
                end
                4'd7: begin
                    slot_valid_reg[7] <= 1'b1;
                    slot_qid_reg[7] <= ctx_bus_qid;
                    slot_ctrl_reg[7] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[7] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[7] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[7] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[7] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[7] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[7] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[7] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[7] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[7] <= 6'd0;
                    slot_fetch_stop_reg[7] <= 1'b0;
                    slot_perf_desc_reg[7] <= 32'd0;
                    slot_perf_dma_reg[7] <= 32'd0;
                end
                4'd8: begin
                    slot_valid_reg[8] <= 1'b1;
                    slot_qid_reg[8] <= ctx_bus_qid;
                    slot_ctrl_reg[8] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[8] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[8] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[8] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[8] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[8] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[8] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[8] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[8] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[8] <= 6'd0;
                    slot_fetch_stop_reg[8] <= 1'b0;
                    slot_perf_desc_reg[8] <= 32'd0;
                    slot_perf_dma_reg[8] <= 32'd0;
                end
                4'd9: begin
                    slot_valid_reg[9] <= 1'b1;
                    slot_qid_reg[9] <= ctx_bus_qid;
                    slot_ctrl_reg[9] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[9] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[9] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[9] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[9] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[9] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[9] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[9] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[9] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[9] <= 6'd0;
                    slot_fetch_stop_reg[9] <= 1'b0;
                    slot_perf_desc_reg[9] <= 32'd0;
                    slot_perf_dma_reg[9] <= 32'd0;
                end
                4'd10: begin
                    slot_valid_reg[10] <= 1'b1;
                    slot_qid_reg[10] <= ctx_bus_qid;
                    slot_ctrl_reg[10] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[10] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[10] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[10] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[10] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[10] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[10] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[10] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[10] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[10] <= 6'd0;
                    slot_fetch_stop_reg[10] <= 1'b0;
                    slot_perf_desc_reg[10] <= 32'd0;
                    slot_perf_dma_reg[10] <= 32'd0;
                end
                4'd11: begin
                    slot_valid_reg[11] <= 1'b1;
                    slot_qid_reg[11] <= ctx_bus_qid;
                    slot_ctrl_reg[11] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[11] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[11] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[11] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[11] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[11] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[11] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[11] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[11] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[11] <= 6'd0;
                    slot_fetch_stop_reg[11] <= 1'b0;
                    slot_perf_desc_reg[11] <= 32'd0;
                    slot_perf_dma_reg[11] <= 32'd0;
                end
                4'd12: begin
                    slot_valid_reg[12] <= 1'b1;
                    slot_qid_reg[12] <= ctx_bus_qid;
                    slot_ctrl_reg[12] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[12] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[12] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[12] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[12] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[12] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[12] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[12] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[12] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[12] <= 6'd0;
                    slot_fetch_stop_reg[12] <= 1'b0;
                    slot_perf_desc_reg[12] <= 32'd0;
                    slot_perf_dma_reg[12] <= 32'd0;
                end
                4'd13: begin
                    slot_valid_reg[13] <= 1'b1;
                    slot_qid_reg[13] <= ctx_bus_qid;
                    slot_ctrl_reg[13] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[13] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[13] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[13] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[13] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[13] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[13] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[13] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[13] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[13] <= 6'd0;
                    slot_fetch_stop_reg[13] <= 1'b0;
                    slot_perf_desc_reg[13] <= 32'd0;
                    slot_perf_dma_reg[13] <= 32'd0;
                end
                4'd14: begin
                    slot_valid_reg[14] <= 1'b1;
                    slot_qid_reg[14] <= ctx_bus_qid;
                    slot_ctrl_reg[14] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[14] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[14] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[14] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[14] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[14] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[14] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[14] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[14] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[14] <= 6'd0;
                    slot_fetch_stop_reg[14] <= 1'b0;
                    slot_perf_desc_reg[14] <= 32'd0;
                    slot_perf_dma_reg[14] <= 32'd0;
                end
                4'd15: begin
                    slot_valid_reg[15] <= 1'b1;
                    slot_qid_reg[15] <= ctx_bus_qid;
                    slot_ctrl_reg[15] <= ctx_bus_active[(ACT_WORD_CTRL*32)+(32)-1:(ACT_WORD_CTRL*32)];
                    slot_mode_reg[15] <= ctx_bus_active[(ACT_WORD_MODE*32)+(32)-1:(ACT_WORD_MODE*32)];
                    slot_irq_enable_reg[15] <= ctx_bus_active[(ACT_WORD_IRQ_ENABLE*32)+(32)-1:(ACT_WORD_IRQ_ENABLE*32)];
                    slot_desc_base_reg[15] <= {ctx_bus_active[(ACT_WORD_DESC_BASE_HI*32)+(32)-1:(ACT_WORD_DESC_BASE_HI*32)], ctx_bus_active[(ACT_WORD_DESC_BASE_LO*32)+(32)-1:(ACT_WORD_DESC_BASE_LO*32)]};
                    slot_wb_base_reg[15] <= {ctx_bus_active[(ACT_WORD_CMPL_BASE_HI*32)+(32)-1:(ACT_WORD_CMPL_BASE_HI*32)], ctx_bus_active[(ACT_WORD_CMPL_BASE_LO*32)+(32)-1:(ACT_WORD_CMPL_BASE_LO*32)]};
                    slot_ring_size_reg[15] <= ctx_bus_active[(ACT_WORD_RING_SIZE*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_RING_SIZE*32)];
                    slot_target_pidx_reg[15] <= ctx_bus_active[(ACT_WORD_DESC_PIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_PIDX*32)];
                    slot_fetch_cidx_reg[15] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_issue_cidx_reg[15] <= ctx_bus_active[(ACT_WORD_DESC_CIDX*32)+(RING_INDEX_WIDTH)-1:(ACT_WORD_DESC_CIDX*32)];
                    slot_fetch_inflight_reg[15] <= 6'd0;
                    slot_fetch_stop_reg[15] <= 1'b0;
                    slot_perf_desc_reg[15] <= 32'd0;
                    slot_perf_dma_reg[15] <= 32'd0;
                end
                default: begin end
            endcase
        end

        if (maint_clear_valid_reg) begin
            case (maint_clear_slot_reg)
                4'd0: slot_valid_reg[0] <= 1'b0;
                4'd1: slot_valid_reg[1] <= 1'b0;
                4'd2: slot_valid_reg[2] <= 1'b0;
                4'd3: slot_valid_reg[3] <= 1'b0;
                4'd4: slot_valid_reg[4] <= 1'b0;
                4'd5: slot_valid_reg[5] <= 1'b0;
                4'd6: slot_valid_reg[6] <= 1'b0;
                4'd7: slot_valid_reg[7] <= 1'b0;
                4'd8: slot_valid_reg[8] <= 1'b0;
                4'd9: slot_valid_reg[9] <= 1'b0;
                4'd10: slot_valid_reg[10] <= 1'b0;
                4'd11: slot_valid_reg[11] <= 1'b0;
                4'd12: slot_valid_reg[12] <= 1'b0;
                4'd13: slot_valid_reg[13] <= 1'b0;
                4'd14: slot_valid_reg[14] <= 1'b0;
                4'd15: slot_valid_reg[15] <= 1'b0;
                default: begin end
            endcase
            if (!free_slot_valid_reg) begin
                free_slot_valid_reg <= 1'b1;
                free_slot_index_reg <= maint_clear_slot_reg;
            end
        end

        if (desc_fetch_accept) begin
            case (fetch_req_slot_reg)
                4'd0: begin
                    slot_fetch_cidx_reg[0] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[0] <= slot_fetch_inflight_reg[0] + fetch_req_count_reg;
                    slot_perf_desc_reg[0] <= slot_perf_desc_reg[0] + fetch_req_count_reg;
                end
                4'd1: begin
                    slot_fetch_cidx_reg[1] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[1] <= slot_fetch_inflight_reg[1] + fetch_req_count_reg;
                    slot_perf_desc_reg[1] <= slot_perf_desc_reg[1] + fetch_req_count_reg;
                end
                4'd2: begin
                    slot_fetch_cidx_reg[2] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[2] <= slot_fetch_inflight_reg[2] + fetch_req_count_reg;
                    slot_perf_desc_reg[2] <= slot_perf_desc_reg[2] + fetch_req_count_reg;
                end
                4'd3: begin
                    slot_fetch_cidx_reg[3] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[3] <= slot_fetch_inflight_reg[3] + fetch_req_count_reg;
                    slot_perf_desc_reg[3] <= slot_perf_desc_reg[3] + fetch_req_count_reg;
                end
                4'd4: begin
                    slot_fetch_cidx_reg[4] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[4] <= slot_fetch_inflight_reg[4] + fetch_req_count_reg;
                    slot_perf_desc_reg[4] <= slot_perf_desc_reg[4] + fetch_req_count_reg;
                end
                4'd5: begin
                    slot_fetch_cidx_reg[5] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[5] <= slot_fetch_inflight_reg[5] + fetch_req_count_reg;
                    slot_perf_desc_reg[5] <= slot_perf_desc_reg[5] + fetch_req_count_reg;
                end
                4'd6: begin
                    slot_fetch_cidx_reg[6] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[6] <= slot_fetch_inflight_reg[6] + fetch_req_count_reg;
                    slot_perf_desc_reg[6] <= slot_perf_desc_reg[6] + fetch_req_count_reg;
                end
                4'd7: begin
                    slot_fetch_cidx_reg[7] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[7] <= slot_fetch_inflight_reg[7] + fetch_req_count_reg;
                    slot_perf_desc_reg[7] <= slot_perf_desc_reg[7] + fetch_req_count_reg;
                end
                4'd8: begin
                    slot_fetch_cidx_reg[8] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[8] <= slot_fetch_inflight_reg[8] + fetch_req_count_reg;
                    slot_perf_desc_reg[8] <= slot_perf_desc_reg[8] + fetch_req_count_reg;
                end
                4'd9: begin
                    slot_fetch_cidx_reg[9] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[9] <= slot_fetch_inflight_reg[9] + fetch_req_count_reg;
                    slot_perf_desc_reg[9] <= slot_perf_desc_reg[9] + fetch_req_count_reg;
                end
                4'd10: begin
                    slot_fetch_cidx_reg[10] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[10] <= slot_fetch_inflight_reg[10] + fetch_req_count_reg;
                    slot_perf_desc_reg[10] <= slot_perf_desc_reg[10] + fetch_req_count_reg;
                end
                4'd11: begin
                    slot_fetch_cidx_reg[11] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[11] <= slot_fetch_inflight_reg[11] + fetch_req_count_reg;
                    slot_perf_desc_reg[11] <= slot_perf_desc_reg[11] + fetch_req_count_reg;
                end
                4'd12: begin
                    slot_fetch_cidx_reg[12] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[12] <= slot_fetch_inflight_reg[12] + fetch_req_count_reg;
                    slot_perf_desc_reg[12] <= slot_perf_desc_reg[12] + fetch_req_count_reg;
                end
                4'd13: begin
                    slot_fetch_cidx_reg[13] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[13] <= slot_fetch_inflight_reg[13] + fetch_req_count_reg;
                    slot_perf_desc_reg[13] <= slot_perf_desc_reg[13] + fetch_req_count_reg;
                end
                4'd14: begin
                    slot_fetch_cidx_reg[14] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[14] <= slot_fetch_inflight_reg[14] + fetch_req_count_reg;
                    slot_perf_desc_reg[14] <= slot_perf_desc_reg[14] + fetch_req_count_reg;
                end
                4'd15: begin
                    slot_fetch_cidx_reg[15] <= fetch_req_next_cidx_comb;
                    slot_fetch_inflight_reg[15] <= slot_fetch_inflight_reg[15] + fetch_req_count_reg;
                    slot_perf_desc_reg[15] <= slot_perf_desc_reg[15] + fetch_req_count_reg;
                end
                default: begin end
            endcase
            global_desc_fetch_bytes_reg <= global_desc_fetch_bytes_reg + ({{(GLOBAL_DESC_FETCH_COUNT_WIDTH-5){1'b0}}, fetch_req_count_reg} << (fetch_req_is_st_reg ? 4 : 5));
        end

        if (desc_resp_fire) begin
            case (fetch_resp_slot_reg)
                4'd0: begin
                    if (slot_fetch_inflight_reg[0] != 0) begin
                        slot_fetch_inflight_reg[0] <= slot_fetch_inflight_reg[0] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[0] <= 1'b1;
                    end
                end
                4'd1: begin
                    if (slot_fetch_inflight_reg[1] != 0) begin
                        slot_fetch_inflight_reg[1] <= slot_fetch_inflight_reg[1] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[1] <= 1'b1;
                    end
                end
                4'd2: begin
                    if (slot_fetch_inflight_reg[2] != 0) begin
                        slot_fetch_inflight_reg[2] <= slot_fetch_inflight_reg[2] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[2] <= 1'b1;
                    end
                end
                4'd3: begin
                    if (slot_fetch_inflight_reg[3] != 0) begin
                        slot_fetch_inflight_reg[3] <= slot_fetch_inflight_reg[3] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[3] <= 1'b1;
                    end
                end
                4'd4: begin
                    if (slot_fetch_inflight_reg[4] != 0) begin
                        slot_fetch_inflight_reg[4] <= slot_fetch_inflight_reg[4] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[4] <= 1'b1;
                    end
                end
                4'd5: begin
                    if (slot_fetch_inflight_reg[5] != 0) begin
                        slot_fetch_inflight_reg[5] <= slot_fetch_inflight_reg[5] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[5] <= 1'b1;
                    end
                end
                4'd6: begin
                    if (slot_fetch_inflight_reg[6] != 0) begin
                        slot_fetch_inflight_reg[6] <= slot_fetch_inflight_reg[6] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[6] <= 1'b1;
                    end
                end
                4'd7: begin
                    if (slot_fetch_inflight_reg[7] != 0) begin
                        slot_fetch_inflight_reg[7] <= slot_fetch_inflight_reg[7] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[7] <= 1'b1;
                    end
                end
                4'd8: begin
                    if (slot_fetch_inflight_reg[8] != 0) begin
                        slot_fetch_inflight_reg[8] <= slot_fetch_inflight_reg[8] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[8] <= 1'b1;
                    end
                end
                4'd9: begin
                    if (slot_fetch_inflight_reg[9] != 0) begin
                        slot_fetch_inflight_reg[9] <= slot_fetch_inflight_reg[9] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[9] <= 1'b1;
                    end
                end
                4'd10: begin
                    if (slot_fetch_inflight_reg[10] != 0) begin
                        slot_fetch_inflight_reg[10] <= slot_fetch_inflight_reg[10] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[10] <= 1'b1;
                    end
                end
                4'd11: begin
                    if (slot_fetch_inflight_reg[11] != 0) begin
                        slot_fetch_inflight_reg[11] <= slot_fetch_inflight_reg[11] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[11] <= 1'b1;
                    end
                end
                4'd12: begin
                    if (slot_fetch_inflight_reg[12] != 0) begin
                        slot_fetch_inflight_reg[12] <= slot_fetch_inflight_reg[12] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[12] <= 1'b1;
                    end
                end
                4'd13: begin
                    if (slot_fetch_inflight_reg[13] != 0) begin
                        slot_fetch_inflight_reg[13] <= slot_fetch_inflight_reg[13] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[13] <= 1'b1;
                    end
                end
                4'd14: begin
                    if (slot_fetch_inflight_reg[14] != 0) begin
                        slot_fetch_inflight_reg[14] <= slot_fetch_inflight_reg[14] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[14] <= 1'b1;
                    end
                end
                4'd15: begin
                    if (slot_fetch_inflight_reg[15] != 0) begin
                        slot_fetch_inflight_reg[15] <= slot_fetch_inflight_reg[15] - 1'b1;
                    end
                    if (resp_entry_comb[DESC_ENTRY_TERMINAL_MSB:DESC_ENTRY_TERMINAL_LSB]) begin
                        slot_fetch_stop_reg[15] <= 1'b1;
                    end
                end
                default: begin end
            endcase
            if (global_desc_fetch_bytes_reg >= (fetch_resp_is_st_reg ? 17'd16 : 17'd32)) begin
                global_desc_fetch_bytes_reg <= global_desc_fetch_bytes_reg - (fetch_resp_is_st_reg ? 17'd16 : 17'd32);
            end
        end

        if (issue_pop) begin
            case (issue_entry_reg[DESC_ENTRY_SLOT_MSB:DESC_ENTRY_SLOT_LSB])
                4'd0: begin
                    slot_issue_cidx_reg[0] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[0] <= slot_perf_dma_reg[0] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[0] <= 1'b0;
                    end
                end
                4'd1: begin
                    slot_issue_cidx_reg[1] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[1] <= slot_perf_dma_reg[1] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[1] <= 1'b0;
                    end
                end
                4'd2: begin
                    slot_issue_cidx_reg[2] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[2] <= slot_perf_dma_reg[2] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[2] <= 1'b0;
                    end
                end
                4'd3: begin
                    slot_issue_cidx_reg[3] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[3] <= slot_perf_dma_reg[3] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[3] <= 1'b0;
                    end
                end
                4'd4: begin
                    slot_issue_cidx_reg[4] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[4] <= slot_perf_dma_reg[4] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[4] <= 1'b0;
                    end
                end
                4'd5: begin
                    slot_issue_cidx_reg[5] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[5] <= slot_perf_dma_reg[5] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[5] <= 1'b0;
                    end
                end
                4'd6: begin
                    slot_issue_cidx_reg[6] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[6] <= slot_perf_dma_reg[6] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[6] <= 1'b0;
                    end
                end
                4'd7: begin
                    slot_issue_cidx_reg[7] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[7] <= slot_perf_dma_reg[7] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[7] <= 1'b0;
                    end
                end
                4'd8: begin
                    slot_issue_cidx_reg[8] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[8] <= slot_perf_dma_reg[8] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[8] <= 1'b0;
                    end
                end
                4'd9: begin
                    slot_issue_cidx_reg[9] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[9] <= slot_perf_dma_reg[9] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[9] <= 1'b0;
                    end
                end
                4'd10: begin
                    slot_issue_cidx_reg[10] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[10] <= slot_perf_dma_reg[10] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[10] <= 1'b0;
                    end
                end
                4'd11: begin
                    slot_issue_cidx_reg[11] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[11] <= slot_perf_dma_reg[11] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[11] <= 1'b0;
                    end
                end
                4'd12: begin
                    slot_issue_cidx_reg[12] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[12] <= slot_perf_dma_reg[12] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[12] <= 1'b0;
                    end
                end
                4'd13: begin
                    slot_issue_cidx_reg[13] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[13] <= slot_perf_dma_reg[13] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[13] <= 1'b0;
                    end
                end
                4'd14: begin
                    slot_issue_cidx_reg[14] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[14] <= slot_perf_dma_reg[14] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[14] <= 1'b0;
                    end
                end
                4'd15: begin
                    slot_issue_cidx_reg[15] <= issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_perf_dma_reg[15] <= slot_perf_dma_reg[15] + 1'b1;
                    if (issue_slot_reclaim) begin
                        slot_valid_reg[15] <= 1'b0;
                    end
                end
                default: begin end
            endcase
            if (issue_slot_reclaim) begin
                if (!free_slot_valid_reg) begin
                    free_slot_valid_reg <= 1'b1;
                    free_slot_index_reg <= issue_entry_reg[DESC_ENTRY_SLOT_MSB:DESC_ENTRY_SLOT_LSB];
                end
            end
        end

        if (issue_error_pop) begin
            case (issue_error_entry_reg[DESC_ENTRY_SLOT_MSB:DESC_ENTRY_SLOT_LSB])
                4'd0: begin
                    slot_issue_cidx_reg[0] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[0] <= 1'b1;
                    slot_valid_reg[0] <= 1'b0;
                end
                4'd1: begin
                    slot_issue_cidx_reg[1] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[1] <= 1'b1;
                    slot_valid_reg[1] <= 1'b0;
                end
                4'd2: begin
                    slot_issue_cidx_reg[2] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[2] <= 1'b1;
                    slot_valid_reg[2] <= 1'b0;
                end
                4'd3: begin
                    slot_issue_cidx_reg[3] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[3] <= 1'b1;
                    slot_valid_reg[3] <= 1'b0;
                end
                4'd4: begin
                    slot_issue_cidx_reg[4] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[4] <= 1'b1;
                    slot_valid_reg[4] <= 1'b0;
                end
                4'd5: begin
                    slot_issue_cidx_reg[5] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[5] <= 1'b1;
                    slot_valid_reg[5] <= 1'b0;
                end
                4'd6: begin
                    slot_issue_cidx_reg[6] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[6] <= 1'b1;
                    slot_valid_reg[6] <= 1'b0;
                end
                4'd7: begin
                    slot_issue_cidx_reg[7] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[7] <= 1'b1;
                    slot_valid_reg[7] <= 1'b0;
                end
                4'd8: begin
                    slot_issue_cidx_reg[8] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[8] <= 1'b1;
                    slot_valid_reg[8] <= 1'b0;
                end
                4'd9: begin
                    slot_issue_cidx_reg[9] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[9] <= 1'b1;
                    slot_valid_reg[9] <= 1'b0;
                end
                4'd10: begin
                    slot_issue_cidx_reg[10] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[10] <= 1'b1;
                    slot_valid_reg[10] <= 1'b0;
                end
                4'd11: begin
                    slot_issue_cidx_reg[11] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[11] <= 1'b1;
                    slot_valid_reg[11] <= 1'b0;
                end
                4'd12: begin
                    slot_issue_cidx_reg[12] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[12] <= 1'b1;
                    slot_valid_reg[12] <= 1'b0;
                end
                4'd13: begin
                    slot_issue_cidx_reg[13] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[13] <= 1'b1;
                    slot_valid_reg[13] <= 1'b0;
                end
                4'd14: begin
                    slot_issue_cidx_reg[14] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[14] <= 1'b1;
                    slot_valid_reg[14] <= 1'b0;
                end
                4'd15: begin
                    slot_issue_cidx_reg[15] <= issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    slot_fetch_stop_reg[15] <= 1'b1;
                    slot_valid_reg[15] <= 1'b0;
                end
                default: begin end
            endcase
            if (!free_slot_valid_reg) begin
                free_slot_valid_reg <= 1'b1;
                free_slot_index_reg <= issue_error_entry_reg[DESC_ENTRY_SLOT_MSB:DESC_ENTRY_SLOT_LSB];
            end
        end

        if (st_tx_status_accept) begin
            case (st_tx_status_slot)
                4'd0: slot_valid_reg[0] <= 1'b0;
                4'd1: slot_valid_reg[1] <= 1'b0;
                4'd2: slot_valid_reg[2] <= 1'b0;
                4'd3: slot_valid_reg[3] <= 1'b0;
                4'd4: slot_valid_reg[4] <= 1'b0;
                4'd5: slot_valid_reg[5] <= 1'b0;
                4'd6: slot_valid_reg[6] <= 1'b0;
                4'd7: slot_valid_reg[7] <= 1'b0;
                4'd8: slot_valid_reg[8] <= 1'b0;
                4'd9: slot_valid_reg[9] <= 1'b0;
                4'd10: slot_valid_reg[10] <= 1'b0;
                4'd11: slot_valid_reg[11] <= 1'b0;
                4'd12: slot_valid_reg[12] <= 1'b0;
                4'd13: slot_valid_reg[13] <= 1'b0;
                4'd14: slot_valid_reg[14] <= 1'b0;
                4'd15: slot_valid_reg[15] <= 1'b0;
                default: begin end
            endcase
            if (!free_slot_valid_reg) begin
                free_slot_valid_reg <= 1'b1;
                free_slot_index_reg <= st_tx_status_slot;
            end
        end

        if (rst) begin
            global_desc_fetch_bytes_reg <= 0;
            free_slot_valid_reg <= 1'b1;
            free_slot_index_reg <= 0;
            free_scan_ptr_reg <= 0;
            slot_occupancy_count_reg <= 0;
                slot_valid_reg[0] <= 1'b0;
                slot_qid_reg[0] <= 0;
                slot_ctrl_reg[0] <= 0;
                slot_mode_reg[0] <= 0;
                slot_irq_enable_reg[0] <= 0;
                slot_desc_base_reg[0] <= 0;
                slot_wb_base_reg[0] <= 0;
                slot_ring_size_reg[0] <= 0;
                slot_target_pidx_reg[0] <= 0;
                slot_fetch_cidx_reg[0] <= 0;
                slot_issue_cidx_reg[0] <= 0;
                slot_fetch_inflight_reg[0] <= 0;
                slot_fetch_stop_reg[0] <= 1'b0;
                slot_perf_desc_reg[0] <= 0;
                slot_perf_dma_reg[0] <= 0;
                slot_valid_reg[1] <= 1'b0;
                slot_qid_reg[1] <= 0;
                slot_ctrl_reg[1] <= 0;
                slot_mode_reg[1] <= 0;
                slot_irq_enable_reg[1] <= 0;
                slot_desc_base_reg[1] <= 0;
                slot_wb_base_reg[1] <= 0;
                slot_ring_size_reg[1] <= 0;
                slot_target_pidx_reg[1] <= 0;
                slot_fetch_cidx_reg[1] <= 0;
                slot_issue_cidx_reg[1] <= 0;
                slot_fetch_inflight_reg[1] <= 0;
                slot_fetch_stop_reg[1] <= 1'b0;
                slot_perf_desc_reg[1] <= 0;
                slot_perf_dma_reg[1] <= 0;
                slot_valid_reg[2] <= 1'b0;
                slot_qid_reg[2] <= 0;
                slot_ctrl_reg[2] <= 0;
                slot_mode_reg[2] <= 0;
                slot_irq_enable_reg[2] <= 0;
                slot_desc_base_reg[2] <= 0;
                slot_wb_base_reg[2] <= 0;
                slot_ring_size_reg[2] <= 0;
                slot_target_pidx_reg[2] <= 0;
                slot_fetch_cidx_reg[2] <= 0;
                slot_issue_cidx_reg[2] <= 0;
                slot_fetch_inflight_reg[2] <= 0;
                slot_fetch_stop_reg[2] <= 1'b0;
                slot_perf_desc_reg[2] <= 0;
                slot_perf_dma_reg[2] <= 0;
                slot_valid_reg[3] <= 1'b0;
                slot_qid_reg[3] <= 0;
                slot_ctrl_reg[3] <= 0;
                slot_mode_reg[3] <= 0;
                slot_irq_enable_reg[3] <= 0;
                slot_desc_base_reg[3] <= 0;
                slot_wb_base_reg[3] <= 0;
                slot_ring_size_reg[3] <= 0;
                slot_target_pidx_reg[3] <= 0;
                slot_fetch_cidx_reg[3] <= 0;
                slot_issue_cidx_reg[3] <= 0;
                slot_fetch_inflight_reg[3] <= 0;
                slot_fetch_stop_reg[3] <= 1'b0;
                slot_perf_desc_reg[3] <= 0;
                slot_perf_dma_reg[3] <= 0;
                slot_valid_reg[4] <= 1'b0;
                slot_qid_reg[4] <= 0;
                slot_ctrl_reg[4] <= 0;
                slot_mode_reg[4] <= 0;
                slot_irq_enable_reg[4] <= 0;
                slot_desc_base_reg[4] <= 0;
                slot_wb_base_reg[4] <= 0;
                slot_ring_size_reg[4] <= 0;
                slot_target_pidx_reg[4] <= 0;
                slot_fetch_cidx_reg[4] <= 0;
                slot_issue_cidx_reg[4] <= 0;
                slot_fetch_inflight_reg[4] <= 0;
                slot_fetch_stop_reg[4] <= 1'b0;
                slot_perf_desc_reg[4] <= 0;
                slot_perf_dma_reg[4] <= 0;
                slot_valid_reg[5] <= 1'b0;
                slot_qid_reg[5] <= 0;
                slot_ctrl_reg[5] <= 0;
                slot_mode_reg[5] <= 0;
                slot_irq_enable_reg[5] <= 0;
                slot_desc_base_reg[5] <= 0;
                slot_wb_base_reg[5] <= 0;
                slot_ring_size_reg[5] <= 0;
                slot_target_pidx_reg[5] <= 0;
                slot_fetch_cidx_reg[5] <= 0;
                slot_issue_cidx_reg[5] <= 0;
                slot_fetch_inflight_reg[5] <= 0;
                slot_fetch_stop_reg[5] <= 1'b0;
                slot_perf_desc_reg[5] <= 0;
                slot_perf_dma_reg[5] <= 0;
                slot_valid_reg[6] <= 1'b0;
                slot_qid_reg[6] <= 0;
                slot_ctrl_reg[6] <= 0;
                slot_mode_reg[6] <= 0;
                slot_irq_enable_reg[6] <= 0;
                slot_desc_base_reg[6] <= 0;
                slot_wb_base_reg[6] <= 0;
                slot_ring_size_reg[6] <= 0;
                slot_target_pidx_reg[6] <= 0;
                slot_fetch_cidx_reg[6] <= 0;
                slot_issue_cidx_reg[6] <= 0;
                slot_fetch_inflight_reg[6] <= 0;
                slot_fetch_stop_reg[6] <= 1'b0;
                slot_perf_desc_reg[6] <= 0;
                slot_perf_dma_reg[6] <= 0;
                slot_valid_reg[7] <= 1'b0;
                slot_qid_reg[7] <= 0;
                slot_ctrl_reg[7] <= 0;
                slot_mode_reg[7] <= 0;
                slot_irq_enable_reg[7] <= 0;
                slot_desc_base_reg[7] <= 0;
                slot_wb_base_reg[7] <= 0;
                slot_ring_size_reg[7] <= 0;
                slot_target_pidx_reg[7] <= 0;
                slot_fetch_cidx_reg[7] <= 0;
                slot_issue_cidx_reg[7] <= 0;
                slot_fetch_inflight_reg[7] <= 0;
                slot_fetch_stop_reg[7] <= 1'b0;
                slot_perf_desc_reg[7] <= 0;
                slot_perf_dma_reg[7] <= 0;
                slot_valid_reg[8] <= 1'b0;
                slot_qid_reg[8] <= 0;
                slot_ctrl_reg[8] <= 0;
                slot_mode_reg[8] <= 0;
                slot_irq_enable_reg[8] <= 0;
                slot_desc_base_reg[8] <= 0;
                slot_wb_base_reg[8] <= 0;
                slot_ring_size_reg[8] <= 0;
                slot_target_pidx_reg[8] <= 0;
                slot_fetch_cidx_reg[8] <= 0;
                slot_issue_cidx_reg[8] <= 0;
                slot_fetch_inflight_reg[8] <= 0;
                slot_fetch_stop_reg[8] <= 1'b0;
                slot_perf_desc_reg[8] <= 0;
                slot_perf_dma_reg[8] <= 0;
                slot_valid_reg[9] <= 1'b0;
                slot_qid_reg[9] <= 0;
                slot_ctrl_reg[9] <= 0;
                slot_mode_reg[9] <= 0;
                slot_irq_enable_reg[9] <= 0;
                slot_desc_base_reg[9] <= 0;
                slot_wb_base_reg[9] <= 0;
                slot_ring_size_reg[9] <= 0;
                slot_target_pidx_reg[9] <= 0;
                slot_fetch_cidx_reg[9] <= 0;
                slot_issue_cidx_reg[9] <= 0;
                slot_fetch_inflight_reg[9] <= 0;
                slot_fetch_stop_reg[9] <= 1'b0;
                slot_perf_desc_reg[9] <= 0;
                slot_perf_dma_reg[9] <= 0;
                slot_valid_reg[10] <= 1'b0;
                slot_qid_reg[10] <= 0;
                slot_ctrl_reg[10] <= 0;
                slot_mode_reg[10] <= 0;
                slot_irq_enable_reg[10] <= 0;
                slot_desc_base_reg[10] <= 0;
                slot_wb_base_reg[10] <= 0;
                slot_ring_size_reg[10] <= 0;
                slot_target_pidx_reg[10] <= 0;
                slot_fetch_cidx_reg[10] <= 0;
                slot_issue_cidx_reg[10] <= 0;
                slot_fetch_inflight_reg[10] <= 0;
                slot_fetch_stop_reg[10] <= 1'b0;
                slot_perf_desc_reg[10] <= 0;
                slot_perf_dma_reg[10] <= 0;
                slot_valid_reg[11] <= 1'b0;
                slot_qid_reg[11] <= 0;
                slot_ctrl_reg[11] <= 0;
                slot_mode_reg[11] <= 0;
                slot_irq_enable_reg[11] <= 0;
                slot_desc_base_reg[11] <= 0;
                slot_wb_base_reg[11] <= 0;
                slot_ring_size_reg[11] <= 0;
                slot_target_pidx_reg[11] <= 0;
                slot_fetch_cidx_reg[11] <= 0;
                slot_issue_cidx_reg[11] <= 0;
                slot_fetch_inflight_reg[11] <= 0;
                slot_fetch_stop_reg[11] <= 1'b0;
                slot_perf_desc_reg[11] <= 0;
                slot_perf_dma_reg[11] <= 0;
                slot_valid_reg[12] <= 1'b0;
                slot_qid_reg[12] <= 0;
                slot_ctrl_reg[12] <= 0;
                slot_mode_reg[12] <= 0;
                slot_irq_enable_reg[12] <= 0;
                slot_desc_base_reg[12] <= 0;
                slot_wb_base_reg[12] <= 0;
                slot_ring_size_reg[12] <= 0;
                slot_target_pidx_reg[12] <= 0;
                slot_fetch_cidx_reg[12] <= 0;
                slot_issue_cidx_reg[12] <= 0;
                slot_fetch_inflight_reg[12] <= 0;
                slot_fetch_stop_reg[12] <= 1'b0;
                slot_perf_desc_reg[12] <= 0;
                slot_perf_dma_reg[12] <= 0;
                slot_valid_reg[13] <= 1'b0;
                slot_qid_reg[13] <= 0;
                slot_ctrl_reg[13] <= 0;
                slot_mode_reg[13] <= 0;
                slot_irq_enable_reg[13] <= 0;
                slot_desc_base_reg[13] <= 0;
                slot_wb_base_reg[13] <= 0;
                slot_ring_size_reg[13] <= 0;
                slot_target_pidx_reg[13] <= 0;
                slot_fetch_cidx_reg[13] <= 0;
                slot_issue_cidx_reg[13] <= 0;
                slot_fetch_inflight_reg[13] <= 0;
                slot_fetch_stop_reg[13] <= 1'b0;
                slot_perf_desc_reg[13] <= 0;
                slot_perf_dma_reg[13] <= 0;
                slot_valid_reg[14] <= 1'b0;
                slot_qid_reg[14] <= 0;
                slot_ctrl_reg[14] <= 0;
                slot_mode_reg[14] <= 0;
                slot_irq_enable_reg[14] <= 0;
                slot_desc_base_reg[14] <= 0;
                slot_wb_base_reg[14] <= 0;
                slot_ring_size_reg[14] <= 0;
                slot_target_pidx_reg[14] <= 0;
                slot_fetch_cidx_reg[14] <= 0;
                slot_issue_cidx_reg[14] <= 0;
                slot_fetch_inflight_reg[14] <= 0;
                slot_fetch_stop_reg[14] <= 1'b0;
                slot_perf_desc_reg[14] <= 0;
                slot_perf_dma_reg[14] <= 0;
                slot_valid_reg[15] <= 1'b0;
                slot_qid_reg[15] <= 0;
                slot_ctrl_reg[15] <= 0;
                slot_mode_reg[15] <= 0;
                slot_irq_enable_reg[15] <= 0;
                slot_desc_base_reg[15] <= 0;
                slot_wb_base_reg[15] <= 0;
                slot_ring_size_reg[15] <= 0;
                slot_target_pidx_reg[15] <= 0;
                slot_fetch_cidx_reg[15] <= 0;
                slot_issue_cidx_reg[15] <= 0;
                slot_fetch_inflight_reg[15] <= 0;
                slot_fetch_stop_reg[15] <= 1'b0;
                slot_perf_desc_reg[15] <= 0;
                slot_perf_dma_reg[15] <= 0;
        end
    end

    /* ---------------------------------------------------------------------
     * Completion/writeback engine.  It owns MM tag state, ST status buffering,
     * writeback request FIFO and IRQ generation.  Descriptor fetch never waits
     * on this engine.
     * ------------------------------------------------------------------ */
    (* ram_style = "distributed" *) reg [WB_ENTRY_WIDTH-1:0] wb_fifo_mem [0:STATIC_WB_FIFO_DEPTH-1];
    reg [STATIC_WB_FIFO_ADDR_WIDTH-1:0] wb_fifo_wr_ptr_reg = 0, wb_fifo_rd_ptr_reg = 0;

    (* ram_style = "distributed" *) reg [ST_STATUS_ENTRY_WIDTH-1:0] st_stat_fifo_mem [0:STATIC_ST_STAT_FIFO_DEPTH-1];
    reg [STATIC_ST_STAT_FIFO_ADDR_WIDTH-1:0] st_stat_wr_ptr_reg = 0, st_stat_rd_ptr_reg = 0;

    reg [WB_ENTRY_WIDTH-1:0] wb_fifo_rd_mux;
    reg [ST_STATUS_ENTRY_WIDTH-1:0] st_stat_fifo_rd_mux;

    always @* begin
        wb_fifo_rd_mux = wb_fifo_mem[wb_fifo_rd_ptr_reg];
        st_stat_fifo_rd_mux = st_stat_fifo_mem[st_stat_rd_ptr_reg];
    end

    reg [PCIE_ADDR_WIDTH-1:0] st_tx_slot_wb_base_mux;
    reg [31:0] st_tx_slot_irq_enable_mux;

    always @* begin
        st_tx_slot_wb_base_mux = {PCIE_ADDR_WIDTH{1'b0}};
        st_tx_slot_irq_enable_mux = 32'd0;
        case (st_tx_status_slot)
            4'd0: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[0]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[0]; end
            4'd1: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[1]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[1]; end
            4'd2: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[2]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[2]; end
            4'd3: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[3]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[3]; end
            4'd4: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[4]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[4]; end
            4'd5: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[5]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[5]; end
            4'd6: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[6]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[6]; end
            4'd7: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[7]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[7]; end
            4'd8: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[8]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[8]; end
            4'd9: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[9]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[9]; end
            4'd10: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[10]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[10]; end
            4'd11: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[11]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[11]; end
            4'd12: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[12]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[12]; end
            4'd13: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[13]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[13]; end
            4'd14: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[14]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[14]; end
            4'd15: begin st_tx_slot_wb_base_mux = slot_wb_base_reg[15]; st_tx_slot_irq_enable_mux = slot_irq_enable_reg[15]; end
            default: begin end
        endcase
    end

    reg [PCIE_ADDR_WIDTH-1:0] wb_addr_reg = 0;
    reg [CMPL_WIDTH-1:0] wb_data_reg = 0;
    reg wb_valid_reg = 1'b0;
    reg wb_inflight_reg = 1'b0;
    reg wb_fence_reg = 1'b0;
    reg [QID_WIDTH-1:0] wb_qid_reg = 0;
    reg [31:0] wb_irq_enable_reg = 0;
    reg wb_irq_pending_reg = 1'b0;
    reg wb_error_pending_reg = 1'b0;

    reg [31:0] irq_status_set_reg = 0;
    reg [QID_WIDTH-1:0] irq_qid_reg = 0;
    reg irq_valid_reg = 1'b0;

    reg [MM_TAG_COUNT-1:0] rd_status_onehot_comb;
    reg [MM_TAG_COUNT-1:0] wr_status_onehot_comb;
    reg rd_status_known_comb;
    reg wr_status_known_comb;
    reg mm_wb_valid_comb;
    reg [MM_TAG_INDEX_WIDTH-1:0] mm_wb_index_comb;
    wire [MM_TAG_NUM_WIDTH-1:0] rd_status_tag_num = dma_read_status_tag[MM_TAG_NUM_WIDTH-1:0];
    wire [MM_TAG_NUM_WIDTH-1:0] wr_status_tag_num = dma_write_status_tag[MM_TAG_NUM_WIDTH-1:0];
    wire mm_order_empty_wire = (mm_order_count_reg == 0);
    wire mm_order_head_done_wire = !mm_order_empty_wire && mm_order_head_done_comb;
    wire mm_order_pop_no_wb = mm_order_head_done_wire && !mm_order_head_wb_ready_comb;
    wire wb_fifo_push_mm;
    wire mm_order_pop_any = mm_order_pop_no_wb || wb_fifo_push_mm;
    assign mm_order_space_av = (mm_order_count_reg < MM_TAG_COUNT) || mm_order_pop_any;

    reg [QID_WIDTH-1:0] mm_wb_qid_mux;
    reg [PCIE_ADDR_WIDTH-1:0] mm_wb_base_mux;
    reg [31:0] mm_wb_irq_enable_mux;
    reg [15:0] mm_wb_swtag_mux;
    reg [RING_INDEX_WIDTH-1:0] mm_wb_cidx_after_mux;
    reg [RING_INDEX_WIDTH-1:0] mm_wb_cmpl_slot_mux;
    reg [31:0] mm_wb_bytes_mux;
    reg [31:0] mm_wb_error_code_mux;
    reg mm_wb_dir_mux;
    reg mm_wb_sop_mux;
    reg mm_wb_eop_mux;
    reg mm_wb_irq_mux;
    /* Sync-only ABI: one fixed 32-byte WB record per queue.  Never add the
     * descriptor slot to CMPL_BASE; the completed descriptor index is carried
     * in WB_ENTRY_CIDX_AFTER instead.  Keeping this as a direct wire also
     * removes a wide address adder from the WB critical path.
     */
    wire [PCIE_ADDR_WIDTH-1:0] mm_wb_ring_addr_wire = mm_wb_base_mux;

    always @* begin
        mm_wb_qid_mux = {QID_WIDTH{1'b0}};
        mm_wb_base_mux = {PCIE_ADDR_WIDTH{1'b0}};
        mm_wb_irq_enable_mux = 32'd0;
        mm_wb_swtag_mux = 16'd0;
        mm_wb_cidx_after_mux = {RING_INDEX_WIDTH{1'b0}};
        mm_wb_cmpl_slot_mux = {RING_INDEX_WIDTH{1'b0}};
        mm_wb_bytes_mux = 32'd0;
        mm_wb_error_code_mux = 32'd0;
        mm_wb_dir_mux = 1'b0;
        mm_wb_sop_mux = 1'b0;
        mm_wb_eop_mux = 1'b0;
        mm_wb_irq_mux = 1'b0;
        case (mm_wb_index_comb)
            4'd0: begin
                mm_wb_qid_mux = tag_qid_reg[0];
                mm_wb_base_mux = tag_wb_base_reg[0];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[0];
                mm_wb_swtag_mux = tag_swtag_reg[0];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[0];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[0];
                mm_wb_bytes_mux = tag_bytes_reg[0];
                mm_wb_error_code_mux = tag_error_code_reg[0];
                mm_wb_dir_mux = tag_dir_reg[0];
                mm_wb_sop_mux = tag_sop_reg[0];
                mm_wb_eop_mux = tag_eop_reg[0];
                mm_wb_irq_mux = tag_irq_reg[0];
            end
            4'd1: begin
                mm_wb_qid_mux = tag_qid_reg[1];
                mm_wb_base_mux = tag_wb_base_reg[1];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[1];
                mm_wb_swtag_mux = tag_swtag_reg[1];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[1];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[1];
                mm_wb_bytes_mux = tag_bytes_reg[1];
                mm_wb_error_code_mux = tag_error_code_reg[1];
                mm_wb_dir_mux = tag_dir_reg[1];
                mm_wb_sop_mux = tag_sop_reg[1];
                mm_wb_eop_mux = tag_eop_reg[1];
                mm_wb_irq_mux = tag_irq_reg[1];
            end
            4'd2: begin
                mm_wb_qid_mux = tag_qid_reg[2];
                mm_wb_base_mux = tag_wb_base_reg[2];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[2];
                mm_wb_swtag_mux = tag_swtag_reg[2];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[2];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[2];
                mm_wb_bytes_mux = tag_bytes_reg[2];
                mm_wb_error_code_mux = tag_error_code_reg[2];
                mm_wb_dir_mux = tag_dir_reg[2];
                mm_wb_sop_mux = tag_sop_reg[2];
                mm_wb_eop_mux = tag_eop_reg[2];
                mm_wb_irq_mux = tag_irq_reg[2];
            end
            4'd3: begin
                mm_wb_qid_mux = tag_qid_reg[3];
                mm_wb_base_mux = tag_wb_base_reg[3];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[3];
                mm_wb_swtag_mux = tag_swtag_reg[3];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[3];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[3];
                mm_wb_bytes_mux = tag_bytes_reg[3];
                mm_wb_error_code_mux = tag_error_code_reg[3];
                mm_wb_dir_mux = tag_dir_reg[3];
                mm_wb_sop_mux = tag_sop_reg[3];
                mm_wb_eop_mux = tag_eop_reg[3];
                mm_wb_irq_mux = tag_irq_reg[3];
            end
            4'd4: begin
                mm_wb_qid_mux = tag_qid_reg[4];
                mm_wb_base_mux = tag_wb_base_reg[4];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[4];
                mm_wb_swtag_mux = tag_swtag_reg[4];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[4];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[4];
                mm_wb_bytes_mux = tag_bytes_reg[4];
                mm_wb_error_code_mux = tag_error_code_reg[4];
                mm_wb_dir_mux = tag_dir_reg[4];
                mm_wb_sop_mux = tag_sop_reg[4];
                mm_wb_eop_mux = tag_eop_reg[4];
                mm_wb_irq_mux = tag_irq_reg[4];
            end
            4'd5: begin
                mm_wb_qid_mux = tag_qid_reg[5];
                mm_wb_base_mux = tag_wb_base_reg[5];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[5];
                mm_wb_swtag_mux = tag_swtag_reg[5];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[5];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[5];
                mm_wb_bytes_mux = tag_bytes_reg[5];
                mm_wb_error_code_mux = tag_error_code_reg[5];
                mm_wb_dir_mux = tag_dir_reg[5];
                mm_wb_sop_mux = tag_sop_reg[5];
                mm_wb_eop_mux = tag_eop_reg[5];
                mm_wb_irq_mux = tag_irq_reg[5];
            end
            4'd6: begin
                mm_wb_qid_mux = tag_qid_reg[6];
                mm_wb_base_mux = tag_wb_base_reg[6];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[6];
                mm_wb_swtag_mux = tag_swtag_reg[6];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[6];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[6];
                mm_wb_bytes_mux = tag_bytes_reg[6];
                mm_wb_error_code_mux = tag_error_code_reg[6];
                mm_wb_dir_mux = tag_dir_reg[6];
                mm_wb_sop_mux = tag_sop_reg[6];
                mm_wb_eop_mux = tag_eop_reg[6];
                mm_wb_irq_mux = tag_irq_reg[6];
            end
            4'd7: begin
                mm_wb_qid_mux = tag_qid_reg[7];
                mm_wb_base_mux = tag_wb_base_reg[7];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[7];
                mm_wb_swtag_mux = tag_swtag_reg[7];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[7];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[7];
                mm_wb_bytes_mux = tag_bytes_reg[7];
                mm_wb_error_code_mux = tag_error_code_reg[7];
                mm_wb_dir_mux = tag_dir_reg[7];
                mm_wb_sop_mux = tag_sop_reg[7];
                mm_wb_eop_mux = tag_eop_reg[7];
                mm_wb_irq_mux = tag_irq_reg[7];
            end
            4'd8: begin
                mm_wb_qid_mux = tag_qid_reg[8];
                mm_wb_base_mux = tag_wb_base_reg[8];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[8];
                mm_wb_swtag_mux = tag_swtag_reg[8];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[8];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[8];
                mm_wb_bytes_mux = tag_bytes_reg[8];
                mm_wb_error_code_mux = tag_error_code_reg[8];
                mm_wb_dir_mux = tag_dir_reg[8];
                mm_wb_sop_mux = tag_sop_reg[8];
                mm_wb_eop_mux = tag_eop_reg[8];
                mm_wb_irq_mux = tag_irq_reg[8];
            end
            4'd9: begin
                mm_wb_qid_mux = tag_qid_reg[9];
                mm_wb_base_mux = tag_wb_base_reg[9];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[9];
                mm_wb_swtag_mux = tag_swtag_reg[9];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[9];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[9];
                mm_wb_bytes_mux = tag_bytes_reg[9];
                mm_wb_error_code_mux = tag_error_code_reg[9];
                mm_wb_dir_mux = tag_dir_reg[9];
                mm_wb_sop_mux = tag_sop_reg[9];
                mm_wb_eop_mux = tag_eop_reg[9];
                mm_wb_irq_mux = tag_irq_reg[9];
            end
            4'd10: begin
                mm_wb_qid_mux = tag_qid_reg[10];
                mm_wb_base_mux = tag_wb_base_reg[10];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[10];
                mm_wb_swtag_mux = tag_swtag_reg[10];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[10];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[10];
                mm_wb_bytes_mux = tag_bytes_reg[10];
                mm_wb_error_code_mux = tag_error_code_reg[10];
                mm_wb_dir_mux = tag_dir_reg[10];
                mm_wb_sop_mux = tag_sop_reg[10];
                mm_wb_eop_mux = tag_eop_reg[10];
                mm_wb_irq_mux = tag_irq_reg[10];
            end
            4'd11: begin
                mm_wb_qid_mux = tag_qid_reg[11];
                mm_wb_base_mux = tag_wb_base_reg[11];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[11];
                mm_wb_swtag_mux = tag_swtag_reg[11];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[11];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[11];
                mm_wb_bytes_mux = tag_bytes_reg[11];
                mm_wb_error_code_mux = tag_error_code_reg[11];
                mm_wb_dir_mux = tag_dir_reg[11];
                mm_wb_sop_mux = tag_sop_reg[11];
                mm_wb_eop_mux = tag_eop_reg[11];
                mm_wb_irq_mux = tag_irq_reg[11];
            end
            4'd12: begin
                mm_wb_qid_mux = tag_qid_reg[12];
                mm_wb_base_mux = tag_wb_base_reg[12];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[12];
                mm_wb_swtag_mux = tag_swtag_reg[12];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[12];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[12];
                mm_wb_bytes_mux = tag_bytes_reg[12];
                mm_wb_error_code_mux = tag_error_code_reg[12];
                mm_wb_dir_mux = tag_dir_reg[12];
                mm_wb_sop_mux = tag_sop_reg[12];
                mm_wb_eop_mux = tag_eop_reg[12];
                mm_wb_irq_mux = tag_irq_reg[12];
            end
            4'd13: begin
                mm_wb_qid_mux = tag_qid_reg[13];
                mm_wb_base_mux = tag_wb_base_reg[13];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[13];
                mm_wb_swtag_mux = tag_swtag_reg[13];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[13];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[13];
                mm_wb_bytes_mux = tag_bytes_reg[13];
                mm_wb_error_code_mux = tag_error_code_reg[13];
                mm_wb_dir_mux = tag_dir_reg[13];
                mm_wb_sop_mux = tag_sop_reg[13];
                mm_wb_eop_mux = tag_eop_reg[13];
                mm_wb_irq_mux = tag_irq_reg[13];
            end
            4'd14: begin
                mm_wb_qid_mux = tag_qid_reg[14];
                mm_wb_base_mux = tag_wb_base_reg[14];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[14];
                mm_wb_swtag_mux = tag_swtag_reg[14];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[14];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[14];
                mm_wb_bytes_mux = tag_bytes_reg[14];
                mm_wb_error_code_mux = tag_error_code_reg[14];
                mm_wb_dir_mux = tag_dir_reg[14];
                mm_wb_sop_mux = tag_sop_reg[14];
                mm_wb_eop_mux = tag_eop_reg[14];
                mm_wb_irq_mux = tag_irq_reg[14];
            end
            4'd15: begin
                mm_wb_qid_mux = tag_qid_reg[15];
                mm_wb_base_mux = tag_wb_base_reg[15];
                mm_wb_irq_enable_mux = tag_irq_enable_reg[15];
                mm_wb_swtag_mux = tag_swtag_reg[15];
                mm_wb_cidx_after_mux = tag_cidx_after_reg[15];
                mm_wb_cmpl_slot_mux = tag_cmpl_slot_reg[15];
                mm_wb_bytes_mux = tag_bytes_reg[15];
                mm_wb_error_code_mux = tag_error_code_reg[15];
                mm_wb_dir_mux = tag_dir_reg[15];
                mm_wb_sop_mux = tag_sop_reg[15];
                mm_wb_eop_mux = tag_eop_reg[15];
                mm_wb_irq_mux = tag_irq_reg[15];
            end
            default: begin end
        endcase
    end

    always @* begin
        rd_status_onehot_comb = {MM_TAG_COUNT{1'b0}};
        wr_status_onehot_comb = {MM_TAG_COUNT{1'b0}};
        rd_status_known_comb = 1'b0;
        wr_status_known_comb = 1'b0;
        mm_wb_valid_comb = 1'b0;
        mm_wb_index_comb = 0;
        mm_order_head_index_comb = 0;
        mm_order_head_done_comb = 1'b0;
        mm_order_head_wb_ready_comb = 1'b0;
        case (mm_order_rd_ptr_reg[MM_TAG_INDEX_WIDTH-1:0])
            4'd0: mm_order_head_index_comb = mm_order_tag_0_reg;
            4'd1: mm_order_head_index_comb = mm_order_tag_1_reg;
            4'd2: mm_order_head_index_comb = mm_order_tag_2_reg;
            4'd3: mm_order_head_index_comb = mm_order_tag_3_reg;
            4'd4: mm_order_head_index_comb = mm_order_tag_4_reg;
            4'd5: mm_order_head_index_comb = mm_order_tag_5_reg;
            4'd6: mm_order_head_index_comb = mm_order_tag_6_reg;
            4'd7: mm_order_head_index_comb = mm_order_tag_7_reg;
            4'd8: mm_order_head_index_comb = mm_order_tag_8_reg;
            4'd9: mm_order_head_index_comb = mm_order_tag_9_reg;
            4'd10: mm_order_head_index_comb = mm_order_tag_10_reg;
            4'd11: mm_order_head_index_comb = mm_order_tag_11_reg;
            4'd12: mm_order_head_index_comb = mm_order_tag_12_reg;
            4'd13: mm_order_head_index_comb = mm_order_tag_13_reg;
            4'd14: mm_order_head_index_comb = mm_order_tag_14_reg;
            4'd15: mm_order_head_index_comb = mm_order_tag_15_reg;
            default: mm_order_head_index_comb = mm_order_tag_15_reg;
        endcase
        case (mm_order_head_index_comb)
            4'd0: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[0];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[0];
            end
            4'd1: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[1];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[1];
            end
            4'd2: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[2];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[2];
            end
            4'd3: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[3];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[3];
            end
            4'd4: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[4];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[4];
            end
            4'd5: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[5];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[5];
            end
            4'd6: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[6];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[6];
            end
            4'd7: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[7];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[7];
            end
            4'd8: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[8];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[8];
            end
            4'd9: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[9];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[9];
            end
            4'd10: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[10];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[10];
            end
            4'd11: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[11];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[11];
            end
            4'd12: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[12];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[12];
            end
            4'd13: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[13];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[13];
            end
            4'd14: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[14];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[14];
            end
            4'd15: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[15];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[15];
            end
            default: begin
                mm_order_head_done_comb = tag_done_bitmap_reg[15];
                mm_order_head_wb_ready_comb = wb_ready_bitmap_reg[15];
            end
        endcase
        if (!mm_order_empty_wire && mm_order_head_done_comb && mm_order_head_wb_ready_comb) begin
            mm_wb_valid_comb = 1'b1;
            mm_wb_index_comb = mm_order_head_index_comb;
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd1) begin
            rd_status_onehot_comb[0] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[0] || wb_ready_bitmap_reg[0] || tag_done_bitmap_reg[0];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd2) begin
            rd_status_onehot_comb[1] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[1] || wb_ready_bitmap_reg[1] || tag_done_bitmap_reg[1];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd3) begin
            rd_status_onehot_comb[2] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[2] || wb_ready_bitmap_reg[2] || tag_done_bitmap_reg[2];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd4) begin
            rd_status_onehot_comb[3] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[3] || wb_ready_bitmap_reg[3] || tag_done_bitmap_reg[3];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd5) begin
            rd_status_onehot_comb[4] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[4] || wb_ready_bitmap_reg[4] || tag_done_bitmap_reg[4];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd6) begin
            rd_status_onehot_comb[5] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[5] || wb_ready_bitmap_reg[5] || tag_done_bitmap_reg[5];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd7) begin
            rd_status_onehot_comb[6] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[6] || wb_ready_bitmap_reg[6] || tag_done_bitmap_reg[6];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd8) begin
            rd_status_onehot_comb[7] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[7] || wb_ready_bitmap_reg[7] || tag_done_bitmap_reg[7];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd9) begin
            rd_status_onehot_comb[8] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[8] || wb_ready_bitmap_reg[8] || tag_done_bitmap_reg[8];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd10) begin
            rd_status_onehot_comb[9] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[9] || wb_ready_bitmap_reg[9] || tag_done_bitmap_reg[9];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd11) begin
            rd_status_onehot_comb[10] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[10] || wb_ready_bitmap_reg[10] || tag_done_bitmap_reg[10];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd12) begin
            rd_status_onehot_comb[11] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[11] || wb_ready_bitmap_reg[11] || tag_done_bitmap_reg[11];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd13) begin
            rd_status_onehot_comb[12] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[12] || wb_ready_bitmap_reg[12] || tag_done_bitmap_reg[12];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd14) begin
            rd_status_onehot_comb[13] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[13] || wb_ready_bitmap_reg[13] || tag_done_bitmap_reg[13];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd15) begin
            rd_status_onehot_comb[14] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[14] || wb_ready_bitmap_reg[14] || tag_done_bitmap_reg[14];
        end
        if (dma_read_status_valid && rd_status_tag_num == 5'd16) begin
            rd_status_onehot_comb[15] = 1'b1;
            rd_status_known_comb = tag_bitmap_reg[15] || wb_ready_bitmap_reg[15] || tag_done_bitmap_reg[15];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd1) begin
            wr_status_onehot_comb[0] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[0] || wb_ready_bitmap_reg[0] || tag_done_bitmap_reg[0];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd2) begin
            wr_status_onehot_comb[1] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[1] || wb_ready_bitmap_reg[1] || tag_done_bitmap_reg[1];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd3) begin
            wr_status_onehot_comb[2] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[2] || wb_ready_bitmap_reg[2] || tag_done_bitmap_reg[2];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd4) begin
            wr_status_onehot_comb[3] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[3] || wb_ready_bitmap_reg[3] || tag_done_bitmap_reg[3];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd5) begin
            wr_status_onehot_comb[4] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[4] || wb_ready_bitmap_reg[4] || tag_done_bitmap_reg[4];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd6) begin
            wr_status_onehot_comb[5] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[5] || wb_ready_bitmap_reg[5] || tag_done_bitmap_reg[5];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd7) begin
            wr_status_onehot_comb[6] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[6] || wb_ready_bitmap_reg[6] || tag_done_bitmap_reg[6];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd8) begin
            wr_status_onehot_comb[7] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[7] || wb_ready_bitmap_reg[7] || tag_done_bitmap_reg[7];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd9) begin
            wr_status_onehot_comb[8] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[8] || wb_ready_bitmap_reg[8] || tag_done_bitmap_reg[8];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd10) begin
            wr_status_onehot_comb[9] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[9] || wb_ready_bitmap_reg[9] || tag_done_bitmap_reg[9];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd11) begin
            wr_status_onehot_comb[10] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[10] || wb_ready_bitmap_reg[10] || tag_done_bitmap_reg[10];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd12) begin
            wr_status_onehot_comb[11] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[11] || wb_ready_bitmap_reg[11] || tag_done_bitmap_reg[11];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd13) begin
            wr_status_onehot_comb[12] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[12] || wb_ready_bitmap_reg[12] || tag_done_bitmap_reg[12];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd14) begin
            wr_status_onehot_comb[13] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[13] || wb_ready_bitmap_reg[13] || tag_done_bitmap_reg[13];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd15) begin
            wr_status_onehot_comb[14] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[14] || wb_ready_bitmap_reg[14] || tag_done_bitmap_reg[14];
        end
        if (dma_write_status_valid && wr_status_tag_num == 5'd16) begin
            wr_status_onehot_comb[15] = 1'b1;
            wr_status_known_comb = tag_bitmap_reg[15] || wb_ready_bitmap_reg[15] || tag_done_bitmap_reg[15];
        end
    end
    reg [WB_ENTRY_WIDTH-1:0] tmp_wb_entry;
    reg [ST_STATUS_ENTRY_WIDTH-1:0] tmp_st_entry;

    wire global_irq_mode = global_mode[`EDEV_MODE_IRQ];
    /* Error completions obey the same single-record ABI as successful MM and
     * ST completions.  A fixed base is essential because the driver allocates
     * no completion ring behind this address.
     */
    wire [PCIE_ADDR_WIDTH-1:0] issue_error_wb_addr_wire =
        issue_error_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];

    reg [WB_ENTRY_WIDTH-1:0] wb_rd_entry_comb;
    reg [31:0] wb_rd_status_word_comb;
    reg [CMPL_WIDTH-1:0] wb_rd_data_comb;
    reg wb_rd_fence_comb;

    always @* begin
        wb_rd_entry_comb = wb_fifo_rd_mux;
        wb_rd_status_word_comb = 32'd0;
        wb_rd_status_word_comb[`EDEV_WB_VALID] = 1'b1;
        wb_rd_status_word_comb[`EDEV_WB_ERROR] = (wb_rd_entry_comb[WB_ENTRY_ERROR_CODE_MSB:WB_ENTRY_ERROR_CODE_LSB] != 0) || wb_rd_entry_comb[WB_ENTRY_WB_MERGED_ERROR_MSB:WB_ENTRY_WB_MERGED_ERROR_LSB];
        wb_rd_status_word_comb[`EDEV_WB_DIR] = wb_rd_entry_comb[WB_ENTRY_DIR_MSB:WB_ENTRY_DIR_LSB];
        wb_rd_status_word_comb[`EDEV_WB_EOP] = wb_rd_entry_comb[WB_ENTRY_EOP_MSB:WB_ENTRY_EOP_LSB];
        wb_rd_status_word_comb[`EDEV_WB_SOP] = wb_rd_entry_comb[WB_ENTRY_SOP_MSB:WB_ENTRY_SOP_LSB];
        wb_rd_status_word_comb[`EDEV_WB_TARGET_ST] = wb_rd_entry_comb[WB_ENTRY_IS_ST_MSB:WB_ENTRY_IS_ST_LSB];
        /* Mirror ST RX failure classes into the ABI flags consumed by the
         * network driver.  These compares are on the writeback-control path,
         * not on the payload AXI-Stream path. */
        wb_rd_status_word_comb[`EDEV_WB_TRUNC] =
            wb_rd_entry_comb[WB_ENTRY_IS_ST_MSB:WB_ENTRY_IS_ST_LSB] &&
            wb_rd_entry_comb[WB_ENTRY_DIR_MSB:WB_ENTRY_DIR_LSB] &&
            (wb_rd_entry_comb[WB_ENTRY_ERROR_CODE_MSB -: 8] == `EDEV_ERR_ST_RX_TRUNC);
        wb_rd_status_word_comb[`EDEV_WB_DROP] =
            wb_rd_entry_comb[WB_ENTRY_IS_ST_MSB:WB_ENTRY_IS_ST_LSB] &&
            wb_rd_entry_comb[WB_ENTRY_DIR_MSB:WB_ENTRY_DIR_LSB] &&
            ((wb_rd_entry_comb[WB_ENTRY_ERROR_CODE_MSB -: 8] == `EDEV_ERR_ST_RX_BAD_FRAME) ||
             (wb_rd_entry_comb[WB_ENTRY_ERROR_CODE_MSB -: 8] == `EDEV_ERR_ST_RX_NO_DESC));
        wb_rd_status_word_comb[`EDEV_WB_MERGED_ERROR] = wb_rd_entry_comb[WB_ENTRY_WB_MERGED_ERROR_MSB:WB_ENTRY_WB_MERGED_ERROR_LSB];
        wb_rd_data_comb = {wb_rd_entry_comb[WB_ENTRY_USER_MSB:WB_ENTRY_USER_LSB],
                           {16'd0, wb_rd_entry_comb[WB_ENTRY_SWTAG_MSB:WB_ENTRY_SWTAG_LSB]},
                           {{(32-QID_WIDTH){1'b0}}, wb_rd_entry_comb[WB_ENTRY_QID_MSB:WB_ENTRY_QID_LSB]},
                           wb_rd_entry_comb[WB_ENTRY_ERROR_CODE_MSB:WB_ENTRY_ERROR_CODE_LSB],
                           wb_rd_entry_comb[WB_ENTRY_BYTES_MSB:WB_ENTRY_BYTES_LSB],
                           {{(32-RING_INDEX_WIDTH){1'b0}}, wb_rd_entry_comb[WB_ENTRY_CIDX_AFTER_MSB:WB_ENTRY_CIDX_AFTER_LSB]},
                           {{(32-RING_INDEX_WIDTH){1'b0}}, wb_rd_entry_comb[WB_ENTRY_CIDX_AFTER_MSB:WB_ENTRY_CIDX_AFTER_LSB]},
                           wb_rd_status_word_comb};
        wb_rd_fence_comb = IRQ_READBACK_FENCE_ENABLE && global_irq_mode &&
                            (((wb_rd_entry_comb[WB_ENTRY_ERROR_CODE_MSB:WB_ENTRY_ERROR_CODE_LSB] != 0) || wb_rd_entry_comb[WB_ENTRY_WB_MERGED_ERROR_MSB:WB_ENTRY_WB_MERGED_ERROR_LSB]) ? wb_rd_entry_comb[WB_ENTRY_IRQ_ENABLE_LSB+`EDEV_IRQ_ERROR] :
                                                                                                         (wb_rd_entry_comb[WB_ENTRY_IRQ_ON_DONE_MSB:WB_ENTRY_IRQ_ON_DONE_LSB] && wb_rd_entry_comb[WB_ENTRY_IRQ_ENABLE_LSB+`EDEV_IRQ_CMPL]));
    end

    wire wb_fifo_push_issue_error = issue_error_wb_event_reg && !wb_fifo_full;
    assign wb_fifo_push_mm = !issue_error_wb_event_reg && mm_wb_valid_comb && !wb_fifo_full;
    wire wb_fifo_push_st = !issue_error_wb_event_reg && !mm_wb_valid_comb && !st_stat_fifo_empty && !wb_fifo_full;
    wire wb_fifo_push_any = wb_fifo_push_issue_error || wb_fifo_push_mm || wb_fifo_push_st;
    wire wb_fifo_pop_to_writer = !wb_valid_reg && !wb_inflight_reg && !wb_fifo_empty;
    wire st_stat_fifo_push_any = st_status_accept;
    wire st_stat_fifo_pop_to_wb = wb_fifo_push_st;

    always @(posedge clk) begin : completion_writeback_engine
        irq_status_set_reg <= 32'd0;
        irq_valid_reg <= 1'b0;

        if (mm_issue_event_reg) begin
            case (mm_order_wr_ptr_reg[MM_TAG_INDEX_WIDTH-1:0])
                4'd0: mm_order_tag_0_reg <= mm_issue_tag_index_reg;
                4'd1: mm_order_tag_1_reg <= mm_issue_tag_index_reg;
                4'd2: mm_order_tag_2_reg <= mm_issue_tag_index_reg;
                4'd3: mm_order_tag_3_reg <= mm_issue_tag_index_reg;
                4'd4: mm_order_tag_4_reg <= mm_issue_tag_index_reg;
                4'd5: mm_order_tag_5_reg <= mm_issue_tag_index_reg;
                4'd6: mm_order_tag_6_reg <= mm_issue_tag_index_reg;
                4'd7: mm_order_tag_7_reg <= mm_issue_tag_index_reg;
                4'd8: mm_order_tag_8_reg <= mm_issue_tag_index_reg;
                4'd9: mm_order_tag_9_reg <= mm_issue_tag_index_reg;
                4'd10: mm_order_tag_10_reg <= mm_issue_tag_index_reg;
                4'd11: mm_order_tag_11_reg <= mm_issue_tag_index_reg;
                4'd12: mm_order_tag_12_reg <= mm_issue_tag_index_reg;
                4'd13: mm_order_tag_13_reg <= mm_issue_tag_index_reg;
                4'd14: mm_order_tag_14_reg <= mm_issue_tag_index_reg;
                4'd15: mm_order_tag_15_reg <= mm_issue_tag_index_reg;
                default: mm_order_tag_15_reg <= mm_issue_tag_index_reg;
            endcase
            mm_order_wr_ptr_reg <= mm_order_wr_ptr_reg + 1'b1;

            case (mm_issue_tag_index_reg)
                4'd0: begin
                    tag_bitmap_reg[0] <= 1'b1;
                    tag_done_bitmap_reg[0] <= 1'b0;
                    tag_qid_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[0] <= 32'd0;
                    tag_dir_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[0] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd1: begin
                    tag_bitmap_reg[1] <= 1'b1;
                    tag_done_bitmap_reg[1] <= 1'b0;
                    tag_qid_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[1] <= 32'd0;
                    tag_dir_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[1] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd2: begin
                    tag_bitmap_reg[2] <= 1'b1;
                    tag_done_bitmap_reg[2] <= 1'b0;
                    tag_qid_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[2] <= 32'd0;
                    tag_dir_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[2] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd3: begin
                    tag_bitmap_reg[3] <= 1'b1;
                    tag_done_bitmap_reg[3] <= 1'b0;
                    tag_qid_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[3] <= 32'd0;
                    tag_dir_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[3] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd4: begin
                    tag_bitmap_reg[4] <= 1'b1;
                    tag_done_bitmap_reg[4] <= 1'b0;
                    tag_qid_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[4] <= 32'd0;
                    tag_dir_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[4] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd5: begin
                    tag_bitmap_reg[5] <= 1'b1;
                    tag_done_bitmap_reg[5] <= 1'b0;
                    tag_qid_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[5] <= 32'd0;
                    tag_dir_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[5] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd6: begin
                    tag_bitmap_reg[6] <= 1'b1;
                    tag_done_bitmap_reg[6] <= 1'b0;
                    tag_qid_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[6] <= 32'd0;
                    tag_dir_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[6] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd7: begin
                    tag_bitmap_reg[7] <= 1'b1;
                    tag_done_bitmap_reg[7] <= 1'b0;
                    tag_qid_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[7] <= 32'd0;
                    tag_dir_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[7] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd8: begin
                    tag_bitmap_reg[8] <= 1'b1;
                    tag_done_bitmap_reg[8] <= 1'b0;
                    tag_qid_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[8] <= 32'd0;
                    tag_dir_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[8] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd9: begin
                    tag_bitmap_reg[9] <= 1'b1;
                    tag_done_bitmap_reg[9] <= 1'b0;
                    tag_qid_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[9] <= 32'd0;
                    tag_dir_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[9] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd10: begin
                    tag_bitmap_reg[10] <= 1'b1;
                    tag_done_bitmap_reg[10] <= 1'b0;
                    tag_qid_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[10] <= 32'd0;
                    tag_dir_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[10] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd11: begin
                    tag_bitmap_reg[11] <= 1'b1;
                    tag_done_bitmap_reg[11] <= 1'b0;
                    tag_qid_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[11] <= 32'd0;
                    tag_dir_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[11] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd12: begin
                    tag_bitmap_reg[12] <= 1'b1;
                    tag_done_bitmap_reg[12] <= 1'b0;
                    tag_qid_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[12] <= 32'd0;
                    tag_dir_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[12] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd13: begin
                    tag_bitmap_reg[13] <= 1'b1;
                    tag_done_bitmap_reg[13] <= 1'b0;
                    tag_qid_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[13] <= 32'd0;
                    tag_dir_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[13] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd14: begin
                    tag_bitmap_reg[14] <= 1'b1;
                    tag_done_bitmap_reg[14] <= 1'b0;
                    tag_qid_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[14] <= 32'd0;
                    tag_dir_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[14] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                4'd15: begin
                    tag_bitmap_reg[15] <= 1'b1;
                    tag_done_bitmap_reg[15] <= 1'b0;
                    tag_qid_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
                    tag_wb_base_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
                    tag_irq_enable_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
                    tag_swtag_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
                    tag_cidx_after_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
                    tag_cmpl_slot_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_CMPL_SLOT_MSB:DESC_ENTRY_CMPL_SLOT_LSB];
                    tag_bytes_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_WB_BYTES_MSB:DESC_ENTRY_WB_BYTES_LSB];
                    tag_error_code_reg[15] <= 32'd0;
                    tag_dir_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
                    tag_sop_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
                    tag_eop_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_EOP];
                    tag_irq_reg[15] <= mm_issue_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_IRQ_ON_DONE];
                end
                default: begin end
            endcase
        end

        if (mm_order_pop_any) begin
            mm_order_rd_ptr_reg <= mm_order_rd_ptr_reg + 1'b1;
            case (mm_order_head_index_comb)
                4'd0: begin
                    tag_done_bitmap_reg[0] <= 1'b0;
                    wb_ready_bitmap_reg[0] <= 1'b0;
                end
                4'd1: begin
                    tag_done_bitmap_reg[1] <= 1'b0;
                    wb_ready_bitmap_reg[1] <= 1'b0;
                end
                4'd2: begin
                    tag_done_bitmap_reg[2] <= 1'b0;
                    wb_ready_bitmap_reg[2] <= 1'b0;
                end
                4'd3: begin
                    tag_done_bitmap_reg[3] <= 1'b0;
                    wb_ready_bitmap_reg[3] <= 1'b0;
                end
                4'd4: begin
                    tag_done_bitmap_reg[4] <= 1'b0;
                    wb_ready_bitmap_reg[4] <= 1'b0;
                end
                4'd5: begin
                    tag_done_bitmap_reg[5] <= 1'b0;
                    wb_ready_bitmap_reg[5] <= 1'b0;
                end
                4'd6: begin
                    tag_done_bitmap_reg[6] <= 1'b0;
                    wb_ready_bitmap_reg[6] <= 1'b0;
                end
                4'd7: begin
                    tag_done_bitmap_reg[7] <= 1'b0;
                    wb_ready_bitmap_reg[7] <= 1'b0;
                end
                4'd8: begin
                    tag_done_bitmap_reg[8] <= 1'b0;
                    wb_ready_bitmap_reg[8] <= 1'b0;
                end
                4'd9: begin
                    tag_done_bitmap_reg[9] <= 1'b0;
                    wb_ready_bitmap_reg[9] <= 1'b0;
                end
                4'd10: begin
                    tag_done_bitmap_reg[10] <= 1'b0;
                    wb_ready_bitmap_reg[10] <= 1'b0;
                end
                4'd11: begin
                    tag_done_bitmap_reg[11] <= 1'b0;
                    wb_ready_bitmap_reg[11] <= 1'b0;
                end
                4'd12: begin
                    tag_done_bitmap_reg[12] <= 1'b0;
                    wb_ready_bitmap_reg[12] <= 1'b0;
                end
                4'd13: begin
                    tag_done_bitmap_reg[13] <= 1'b0;
                    wb_ready_bitmap_reg[13] <= 1'b0;
                end
                4'd14: begin
                    tag_done_bitmap_reg[14] <= 1'b0;
                    wb_ready_bitmap_reg[14] <= 1'b0;
                end
                4'd15: begin
                    tag_done_bitmap_reg[15] <= 1'b0;
                    wb_ready_bitmap_reg[15] <= 1'b0;
                end
                default: begin
                    tag_done_bitmap_reg[15] <= 1'b0;
                    wb_ready_bitmap_reg[15] <= 1'b0;
                end
            endcase
        end

        if (wb_fifo_push_issue_error) begin
            tmp_wb_entry = {WB_ENTRY_WIDTH{1'b0}};
            tmp_wb_entry[WB_ENTRY_WB_BASE_MSB:WB_ENTRY_WB_BASE_LSB] = issue_error_wb_addr_wire;
            tmp_wb_entry[WB_ENTRY_IRQ_ENABLE_MSB:WB_ENTRY_IRQ_ENABLE_LSB] = issue_error_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
            tmp_wb_entry[WB_ENTRY_QID_MSB:WB_ENTRY_QID_LSB] = issue_error_entry_reg[DESC_ENTRY_QID_MSB:DESC_ENTRY_QID_LSB];
            tmp_wb_entry[WB_ENTRY_CIDX_AFTER_MSB:WB_ENTRY_CIDX_AFTER_LSB] = issue_error_entry_reg[DESC_ENTRY_CIDX_AFTER_MSB:DESC_ENTRY_CIDX_AFTER_LSB];
            tmp_wb_entry[WB_ENTRY_BYTES_MSB:WB_ENTRY_BYTES_LSB] = 32'd0;
            tmp_wb_entry[WB_ENTRY_ERROR_CODE_MSB:WB_ENTRY_ERROR_CODE_LSB] = issue_error_entry_reg[DESC_ENTRY_ERROR_CODE_MSB:DESC_ENTRY_ERROR_CODE_LSB];
            tmp_wb_entry[WB_ENTRY_SWTAG_MSB:WB_ENTRY_SWTAG_LSB] = issue_error_entry_reg[DESC_ENTRY_SWTAG_MSB:DESC_ENTRY_SWTAG_LSB];
            tmp_wb_entry[WB_ENTRY_USER_MSB:WB_ENTRY_USER_LSB] = 32'd0;
            tmp_wb_entry[WB_ENTRY_DIR_MSB:WB_ENTRY_DIR_LSB] = issue_error_entry_reg[DESC_ENTRY_DIR_MSB:DESC_ENTRY_DIR_LSB];
            tmp_wb_entry[WB_ENTRY_IS_ST_MSB:WB_ENTRY_IS_ST_LSB] = issue_error_entry_reg[DESC_ENTRY_IS_ST_MSB:DESC_ENTRY_IS_ST_LSB];
            tmp_wb_entry[WB_ENTRY_SOP_MSB:WB_ENTRY_SOP_LSB] = issue_error_entry_reg[DESC_ENTRY_FLAGS_LSB+`EDEV_DESC_SOP];
            tmp_wb_entry[WB_ENTRY_EOP_MSB:WB_ENTRY_EOP_LSB] = 1'b1;
            tmp_wb_entry[WB_ENTRY_IRQ_ON_DONE_MSB:WB_ENTRY_IRQ_ON_DONE_LSB] = 1'b0;
            tmp_wb_entry[WB_ENTRY_WB_MERGED_ERROR_MSB:WB_ENTRY_WB_MERGED_ERROR_LSB] = 1'b0;
            wb_fifo_mem[wb_fifo_wr_ptr_reg] <= tmp_wb_entry;
            wb_fifo_wr_ptr_reg <= wb_fifo_wr_ptr_reg + 1'b1;
        end

        if (dma_read_status_valid) begin
            if (!rd_status_known_comb) begin
                irq_status_set_reg[`EDEV_IRQ_ERROR] <= 1'b1;
                irq_qid_reg <= {QID_WIDTH{1'b0}};
                irq_valid_reg <= 1'b1;
            end
            if (rd_status_onehot_comb[0]) begin
                tag_bitmap_reg[0] <= 1'b0;
                tag_done_bitmap_reg[0] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[0]) begin
                    tag_error_code_reg[0] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[0] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[1]) begin
                tag_bitmap_reg[1] <= 1'b0;
                tag_done_bitmap_reg[1] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[1]) begin
                    tag_error_code_reg[1] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[1] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[2]) begin
                tag_bitmap_reg[2] <= 1'b0;
                tag_done_bitmap_reg[2] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[2]) begin
                    tag_error_code_reg[2] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[2] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[3]) begin
                tag_bitmap_reg[3] <= 1'b0;
                tag_done_bitmap_reg[3] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[3]) begin
                    tag_error_code_reg[3] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[3] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[4]) begin
                tag_bitmap_reg[4] <= 1'b0;
                tag_done_bitmap_reg[4] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[4]) begin
                    tag_error_code_reg[4] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[4] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[5]) begin
                tag_bitmap_reg[5] <= 1'b0;
                tag_done_bitmap_reg[5] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[5]) begin
                    tag_error_code_reg[5] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[5] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[6]) begin
                tag_bitmap_reg[6] <= 1'b0;
                tag_done_bitmap_reg[6] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[6]) begin
                    tag_error_code_reg[6] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[6] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[7]) begin
                tag_bitmap_reg[7] <= 1'b0;
                tag_done_bitmap_reg[7] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[7]) begin
                    tag_error_code_reg[7] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[7] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[8]) begin
                tag_bitmap_reg[8] <= 1'b0;
                tag_done_bitmap_reg[8] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[8]) begin
                    tag_error_code_reg[8] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[8] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[9]) begin
                tag_bitmap_reg[9] <= 1'b0;
                tag_done_bitmap_reg[9] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[9]) begin
                    tag_error_code_reg[9] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[9] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[10]) begin
                tag_bitmap_reg[10] <= 1'b0;
                tag_done_bitmap_reg[10] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[10]) begin
                    tag_error_code_reg[10] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[10] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[11]) begin
                tag_bitmap_reg[11] <= 1'b0;
                tag_done_bitmap_reg[11] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[11]) begin
                    tag_error_code_reg[11] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[11] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[12]) begin
                tag_bitmap_reg[12] <= 1'b0;
                tag_done_bitmap_reg[12] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[12]) begin
                    tag_error_code_reg[12] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[12] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[13]) begin
                tag_bitmap_reg[13] <= 1'b0;
                tag_done_bitmap_reg[13] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[13]) begin
                    tag_error_code_reg[13] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[13] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[14]) begin
                tag_bitmap_reg[14] <= 1'b0;
                tag_done_bitmap_reg[14] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[14]) begin
                    tag_error_code_reg[14] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[14] <= 1'b1;
                end
            end
            if (rd_status_onehot_comb[15]) begin
                tag_bitmap_reg[15] <= 1'b0;
                tag_done_bitmap_reg[15] <= 1'b1;
                if (dma_read_status_error != 0 || tag_eop_reg[15]) begin
                    tag_error_code_reg[15] <= (dma_read_status_error != 0) ? {`EDEV_ERR_PAYLOAD_READ, 8'h00, {12'd0, dma_read_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[15] <= 1'b1;
                end
            end
        end

        if (dma_write_status_valid) begin
            if (!wr_status_known_comb) begin
                irq_status_set_reg[`EDEV_IRQ_ERROR] <= 1'b1;
                irq_qid_reg <= {QID_WIDTH{1'b0}};
                irq_valid_reg <= 1'b1;
            end
            if (wr_status_onehot_comb[0]) begin
                tag_bitmap_reg[0] <= 1'b0;
                tag_done_bitmap_reg[0] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[0]) begin
                    tag_error_code_reg[0] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[0] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[1]) begin
                tag_bitmap_reg[1] <= 1'b0;
                tag_done_bitmap_reg[1] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[1]) begin
                    tag_error_code_reg[1] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[1] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[2]) begin
                tag_bitmap_reg[2] <= 1'b0;
                tag_done_bitmap_reg[2] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[2]) begin
                    tag_error_code_reg[2] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[2] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[3]) begin
                tag_bitmap_reg[3] <= 1'b0;
                tag_done_bitmap_reg[3] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[3]) begin
                    tag_error_code_reg[3] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[3] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[4]) begin
                tag_bitmap_reg[4] <= 1'b0;
                tag_done_bitmap_reg[4] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[4]) begin
                    tag_error_code_reg[4] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[4] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[5]) begin
                tag_bitmap_reg[5] <= 1'b0;
                tag_done_bitmap_reg[5] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[5]) begin
                    tag_error_code_reg[5] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[5] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[6]) begin
                tag_bitmap_reg[6] <= 1'b0;
                tag_done_bitmap_reg[6] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[6]) begin
                    tag_error_code_reg[6] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[6] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[7]) begin
                tag_bitmap_reg[7] <= 1'b0;
                tag_done_bitmap_reg[7] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[7]) begin
                    tag_error_code_reg[7] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[7] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[8]) begin
                tag_bitmap_reg[8] <= 1'b0;
                tag_done_bitmap_reg[8] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[8]) begin
                    tag_error_code_reg[8] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[8] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[9]) begin
                tag_bitmap_reg[9] <= 1'b0;
                tag_done_bitmap_reg[9] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[9]) begin
                    tag_error_code_reg[9] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[9] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[10]) begin
                tag_bitmap_reg[10] <= 1'b0;
                tag_done_bitmap_reg[10] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[10]) begin
                    tag_error_code_reg[10] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[10] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[11]) begin
                tag_bitmap_reg[11] <= 1'b0;
                tag_done_bitmap_reg[11] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[11]) begin
                    tag_error_code_reg[11] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[11] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[12]) begin
                tag_bitmap_reg[12] <= 1'b0;
                tag_done_bitmap_reg[12] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[12]) begin
                    tag_error_code_reg[12] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[12] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[13]) begin
                tag_bitmap_reg[13] <= 1'b0;
                tag_done_bitmap_reg[13] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[13]) begin
                    tag_error_code_reg[13] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[13] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[14]) begin
                tag_bitmap_reg[14] <= 1'b0;
                tag_done_bitmap_reg[14] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[14]) begin
                    tag_error_code_reg[14] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[14] <= 1'b1;
                end
            end
            if (wr_status_onehot_comb[15]) begin
                tag_bitmap_reg[15] <= 1'b0;
                tag_done_bitmap_reg[15] <= 1'b1;
                if (dma_write_status_error != 0 || tag_eop_reg[15]) begin
                    tag_error_code_reg[15] <= (dma_write_status_error != 0) ? {`EDEV_ERR_PAYLOAD_WRITE, 8'h00, {12'd0, dma_write_status_error}} : 32'd0;
                    wb_ready_bitmap_reg[15] <= 1'b1;
                end
            end
        end

        if (st_status_accept) begin
            tmp_st_entry = {ST_STATUS_ENTRY_WIDTH{1'b0}};
            tmp_st_entry[ST_STATUS_ENTRY_SLOT_MSB:ST_STATUS_ENTRY_SLOT_LSB] = st_status_sel_rx ? {SLOT_INDEX_WIDTH{1'b0}} : st_tx_status_slot;
            tmp_st_entry[ST_STATUS_ENTRY_WB_BASE_MSB:ST_STATUS_ENTRY_WB_BASE_LSB] = st_status_sel_rx ? st_rx_status_wb_base : st_tx_slot_wb_base_mux;
            tmp_st_entry[ST_STATUS_ENTRY_IRQ_ENABLE_MSB:ST_STATUS_ENTRY_IRQ_ENABLE_LSB] = st_status_sel_rx ? st_rx_status_irq_enable : st_tx_slot_irq_enable_mux;
            tmp_st_entry[ST_STATUS_ENTRY_QID_MSB:ST_STATUS_ENTRY_QID_LSB] = st_status_sel_rx ? st_rx_status_qid : st_tx_status_qid;
            tmp_st_entry[ST_STATUS_ENTRY_CIDX_AFTER_MSB:ST_STATUS_ENTRY_CIDX_AFTER_LSB] = st_status_sel_rx ? st_rx_status_cidx_after : st_tx_status_cidx_after;
            tmp_st_entry[ST_STATUS_ENTRY_BYTES_MSB:ST_STATUS_ENTRY_BYTES_LSB] = st_status_sel_rx ? st_rx_status_bytes : st_tx_status_bytes;
            tmp_st_entry[ST_STATUS_ENTRY_ERROR_CODE_MSB:ST_STATUS_ENTRY_ERROR_CODE_LSB] = st_status_sel_rx ? st_rx_status_error_code : st_tx_status_error_code;
            tmp_st_entry[ST_STATUS_ENTRY_SWTAG_MSB:ST_STATUS_ENTRY_SWTAG_LSB] = st_status_sel_rx ? st_rx_status_swtag : st_tx_status_swtag;
            tmp_st_entry[ST_STATUS_ENTRY_USER_MSB:ST_STATUS_ENTRY_USER_LSB] = st_status_sel_rx ? st_rx_status_user : st_tx_status_user;
            tmp_st_entry[ST_STATUS_ENTRY_IS_RX_MSB:ST_STATUS_ENTRY_IS_RX_LSB] = st_status_sel_rx;
            st_stat_fifo_mem[st_stat_wr_ptr_reg] <= tmp_st_entry;
            st_stat_wr_ptr_reg <= st_stat_wr_ptr_reg + 1'b1;
            st_status_rr_reg <= ~st_status_rr_reg;
        end

        if (wb_fifo_push_mm) begin
            tmp_wb_entry = {WB_ENTRY_WIDTH{1'b0}};
            tmp_wb_entry[WB_ENTRY_WB_BASE_MSB:WB_ENTRY_WB_BASE_LSB] = mm_wb_ring_addr_wire;
            tmp_wb_entry[WB_ENTRY_IRQ_ENABLE_MSB:WB_ENTRY_IRQ_ENABLE_LSB] = mm_wb_irq_enable_mux;
            tmp_wb_entry[WB_ENTRY_QID_MSB:WB_ENTRY_QID_LSB] = mm_wb_qid_mux;
            tmp_wb_entry[WB_ENTRY_CIDX_AFTER_MSB:WB_ENTRY_CIDX_AFTER_LSB] = mm_wb_cidx_after_mux;
            tmp_wb_entry[WB_ENTRY_BYTES_MSB:WB_ENTRY_BYTES_LSB] = mm_wb_bytes_mux;
            tmp_wb_entry[WB_ENTRY_ERROR_CODE_MSB:WB_ENTRY_ERROR_CODE_LSB] = mm_wb_error_code_mux;
            tmp_wb_entry[WB_ENTRY_SWTAG_MSB:WB_ENTRY_SWTAG_LSB] = mm_wb_swtag_mux;
            tmp_wb_entry[WB_ENTRY_USER_MSB:WB_ENTRY_USER_LSB] = 32'd0;
            tmp_wb_entry[WB_ENTRY_DIR_MSB:WB_ENTRY_DIR_LSB] = mm_wb_dir_mux;
            tmp_wb_entry[WB_ENTRY_IS_ST_MSB:WB_ENTRY_IS_ST_LSB] = 1'b0;
            tmp_wb_entry[WB_ENTRY_SOP_MSB:WB_ENTRY_SOP_LSB] = mm_wb_sop_mux;
            tmp_wb_entry[WB_ENTRY_EOP_MSB:WB_ENTRY_EOP_LSB] = mm_wb_eop_mux || mm_wb_error_code_mux != 0;
            tmp_wb_entry[WB_ENTRY_IRQ_ON_DONE_MSB:WB_ENTRY_IRQ_ON_DONE_LSB] = mm_wb_irq_mux;
            tmp_wb_entry[WB_ENTRY_WB_MERGED_ERROR_MSB:WB_ENTRY_WB_MERGED_ERROR_LSB] = 1'b0;

            wb_fifo_mem[wb_fifo_wr_ptr_reg] <= tmp_wb_entry;
            wb_fifo_wr_ptr_reg <= wb_fifo_wr_ptr_reg + 1'b1;
            case (mm_wb_index_comb)
                4'd0: wb_ready_bitmap_reg[0] <= 1'b0;
                4'd1: wb_ready_bitmap_reg[1] <= 1'b0;
                4'd2: wb_ready_bitmap_reg[2] <= 1'b0;
                4'd3: wb_ready_bitmap_reg[3] <= 1'b0;
                4'd4: wb_ready_bitmap_reg[4] <= 1'b0;
                4'd5: wb_ready_bitmap_reg[5] <= 1'b0;
                4'd6: wb_ready_bitmap_reg[6] <= 1'b0;
                4'd7: wb_ready_bitmap_reg[7] <= 1'b0;
                4'd8: wb_ready_bitmap_reg[8] <= 1'b0;
                4'd9: wb_ready_bitmap_reg[9] <= 1'b0;
                4'd10: wb_ready_bitmap_reg[10] <= 1'b0;
                4'd11: wb_ready_bitmap_reg[11] <= 1'b0;
                4'd12: wb_ready_bitmap_reg[12] <= 1'b0;
                4'd13: wb_ready_bitmap_reg[13] <= 1'b0;
                4'd14: wb_ready_bitmap_reg[14] <= 1'b0;
                4'd15: wb_ready_bitmap_reg[15] <= 1'b0;
                default: begin end
            endcase
        end else if (wb_fifo_push_st) begin
            tmp_st_entry = st_stat_fifo_rd_mux;
            tmp_wb_entry = {WB_ENTRY_WIDTH{1'b0}};
            tmp_wb_entry[WB_ENTRY_WB_BASE_MSB:WB_ENTRY_WB_BASE_LSB] = tmp_st_entry[ST_STATUS_ENTRY_WB_BASE_MSB:ST_STATUS_ENTRY_WB_BASE_LSB];
            tmp_wb_entry[WB_ENTRY_IRQ_ENABLE_MSB:WB_ENTRY_IRQ_ENABLE_LSB] = tmp_st_entry[ST_STATUS_ENTRY_IRQ_ENABLE_MSB:ST_STATUS_ENTRY_IRQ_ENABLE_LSB];
            tmp_wb_entry[WB_ENTRY_QID_MSB:WB_ENTRY_QID_LSB] = tmp_st_entry[ST_STATUS_ENTRY_QID_MSB:ST_STATUS_ENTRY_QID_LSB];
            tmp_wb_entry[WB_ENTRY_CIDX_AFTER_MSB:WB_ENTRY_CIDX_AFTER_LSB] = tmp_st_entry[ST_STATUS_ENTRY_CIDX_AFTER_MSB:ST_STATUS_ENTRY_CIDX_AFTER_LSB];
            tmp_wb_entry[WB_ENTRY_BYTES_MSB:WB_ENTRY_BYTES_LSB] = tmp_st_entry[ST_STATUS_ENTRY_BYTES_MSB:ST_STATUS_ENTRY_BYTES_LSB];
            tmp_wb_entry[WB_ENTRY_ERROR_CODE_MSB:WB_ENTRY_ERROR_CODE_LSB] = tmp_st_entry[ST_STATUS_ENTRY_ERROR_CODE_MSB:ST_STATUS_ENTRY_ERROR_CODE_LSB];
            tmp_wb_entry[WB_ENTRY_SWTAG_MSB:WB_ENTRY_SWTAG_LSB] = tmp_st_entry[ST_STATUS_ENTRY_SWTAG_MSB:ST_STATUS_ENTRY_SWTAG_LSB];
            tmp_wb_entry[WB_ENTRY_USER_MSB:WB_ENTRY_USER_LSB] = tmp_st_entry[ST_STATUS_ENTRY_USER_MSB:ST_STATUS_ENTRY_USER_LSB];
            tmp_wb_entry[WB_ENTRY_DIR_MSB:WB_ENTRY_DIR_LSB] = tmp_st_entry[ST_STATUS_ENTRY_IS_RX_MSB:ST_STATUS_ENTRY_IS_RX_LSB];
            tmp_wb_entry[WB_ENTRY_IS_ST_MSB:WB_ENTRY_IS_ST_LSB] = 1'b1;
            tmp_wb_entry[WB_ENTRY_SOP_MSB:WB_ENTRY_SOP_LSB] = 1'b1;
            tmp_wb_entry[WB_ENTRY_EOP_MSB:WB_ENTRY_EOP_LSB] = 1'b1;
            tmp_wb_entry[WB_ENTRY_IRQ_ON_DONE_MSB:WB_ENTRY_IRQ_ON_DONE_LSB] = 1'b1;
            tmp_wb_entry[WB_ENTRY_WB_MERGED_ERROR_MSB:WB_ENTRY_WB_MERGED_ERROR_LSB] = 1'b0;
            wb_fifo_mem[wb_fifo_wr_ptr_reg] <= tmp_wb_entry;
            wb_fifo_wr_ptr_reg <= wb_fifo_wr_ptr_reg + 1'b1;
            st_stat_rd_ptr_reg <= st_stat_rd_ptr_reg + 1'b1;
        end

        if (wb_valid_reg && !wb_inflight_reg && cmpl_write_ready) begin
            wb_valid_reg <= 1'b0;
            wb_inflight_reg <= 1'b1;
        end
        if (wb_inflight_reg && cmpl_write_done) begin
            wb_inflight_reg <= 1'b0;
            if (wb_error_pending_reg && wb_irq_enable_reg[`EDEV_IRQ_ERROR]) begin
                irq_status_set_reg[`EDEV_IRQ_ERROR] <= 1'b1;
                irq_qid_reg <= wb_qid_reg;
                irq_valid_reg <= 1'b1;
            end else if (wb_irq_pending_reg && wb_irq_enable_reg[`EDEV_IRQ_CMPL]) begin
                irq_status_set_reg[`EDEV_IRQ_CMPL] <= 1'b1;
                irq_qid_reg <= wb_qid_reg;
                irq_valid_reg <= 1'b1;
            end
        end
        if (wb_fifo_pop_to_writer) begin
            wb_addr_reg <= wb_rd_entry_comb[WB_ENTRY_WB_BASE_MSB:WB_ENTRY_WB_BASE_LSB];
            wb_data_reg <= wb_rd_data_comb;
            wb_fence_reg <= wb_rd_fence_comb;
            wb_valid_reg <= 1'b1;
            wb_qid_reg <= wb_rd_entry_comb[WB_ENTRY_QID_MSB:WB_ENTRY_QID_LSB];
            wb_irq_enable_reg <= wb_rd_entry_comb[WB_ENTRY_IRQ_ENABLE_MSB:WB_ENTRY_IRQ_ENABLE_LSB];
            wb_irq_pending_reg <= global_irq_mode && wb_rd_entry_comb[WB_ENTRY_IRQ_ON_DONE_MSB:WB_ENTRY_IRQ_ON_DONE_LSB];
            wb_error_pending_reg <= global_irq_mode && ((wb_rd_entry_comb[WB_ENTRY_ERROR_CODE_MSB:WB_ENTRY_ERROR_CODE_LSB] != 0) || wb_rd_entry_comb[WB_ENTRY_WB_MERGED_ERROR_MSB:WB_ENTRY_WB_MERGED_ERROR_LSB]);
            wb_fifo_rd_ptr_reg <= wb_fifo_rd_ptr_reg + 1'b1;
        end

        case ({wb_fifo_push_any, wb_fifo_pop_to_writer})
            2'b10: wb_fifo_count_reg <= wb_fifo_count_reg + 1'b1;
            2'b01: wb_fifo_count_reg <= wb_fifo_count_reg - 1'b1;
            default: wb_fifo_count_reg <= wb_fifo_count_reg;
        endcase

        case ({mm_issue_event_reg, mm_order_pop_any})
            2'b10: mm_order_count_reg <= mm_order_count_reg + 1'b1;
            2'b01: mm_order_count_reg <= mm_order_count_reg - 1'b1;
            default: mm_order_count_reg <= mm_order_count_reg;
        endcase

        case ({st_stat_fifo_push_any, st_stat_fifo_pop_to_wb})
            2'b10: st_stat_count_reg <= st_stat_count_reg + 1'b1;
            2'b01: st_stat_count_reg <= st_stat_count_reg - 1'b1;
            default: st_stat_count_reg <= st_stat_count_reg;
        endcase

        if (rst) begin
            tag_bitmap_reg <= {MM_TAG_COUNT{1'b0}};
            wb_ready_bitmap_reg <= {MM_TAG_COUNT{1'b0}};
            tag_done_bitmap_reg <= {MM_TAG_COUNT{1'b0}};
            mm_order_wr_ptr_reg <= 0;
            mm_order_rd_ptr_reg <= 0;
            mm_order_count_reg <= 0;
            tag_dir_reg <= {MM_TAG_COUNT{1'b0}};
            tag_sop_reg <= {MM_TAG_COUNT{1'b0}};
            tag_eop_reg <= {MM_TAG_COUNT{1'b0}};
            tag_irq_reg <= {MM_TAG_COUNT{1'b0}};
            tag_error_code_reg[0] <= 32'd0;
            tag_error_code_reg[1] <= 32'd0;
            tag_error_code_reg[2] <= 32'd0;
            tag_error_code_reg[3] <= 32'd0;
            tag_error_code_reg[4] <= 32'd0;
            tag_error_code_reg[5] <= 32'd0;
            tag_error_code_reg[6] <= 32'd0;
            tag_error_code_reg[7] <= 32'd0;
            tag_error_code_reg[8] <= 32'd0;
            tag_error_code_reg[9] <= 32'd0;
            tag_error_code_reg[10] <= 32'd0;
            tag_error_code_reg[11] <= 32'd0;
            tag_error_code_reg[12] <= 32'd0;
            tag_error_code_reg[13] <= 32'd0;
            tag_error_code_reg[14] <= 32'd0;
            tag_error_code_reg[15] <= 32'd0;
            wb_fifo_wr_ptr_reg <= 0;
            wb_fifo_rd_ptr_reg <= 0;
            wb_fifo_count_reg <= 0;
            st_stat_wr_ptr_reg <= 0;
            st_stat_rd_ptr_reg <= 0;
            st_stat_count_reg <= 0;
            wb_valid_reg <= 1'b0;
            wb_inflight_reg <= 1'b0;
            wb_fence_reg <= 1'b0;
            irq_status_set_reg <= 32'd0;
            irq_valid_reg <= 1'b0;
            st_status_rr_reg <= 1'b0;
        end
    end

    /* Output wiring. */
    assign desc_fetch_addr = desc_fetch_addr_reg;
    assign desc_fetch_tag = desc_fetch_tag_reg;
    assign desc_fetch_count = desc_fetch_count_reg;
    assign desc_fetch_is_st = desc_fetch_is_st_reg;
    assign desc_fetch_valid = desc_fetch_valid_reg;


    assign dma_read_desc_pcie_addr = issue_host_addr_reg;
    assign dma_read_desc_axi_addr = issue_dev_addr_reg;
    assign dma_read_desc_len = issue_len_reg;
    assign dma_read_desc_tag = issue_dma_tag_reg;
    assign dma_read_desc_qid = issue_qid_reg;
    assign dma_read_desc_valid = dma_read_valid_reg;

    assign dma_write_desc_pcie_addr = issue_host_addr_reg;
    assign dma_write_desc_axi_addr = issue_dev_addr_reg;
    assign dma_write_desc_len = issue_len_reg;
    assign dma_write_desc_tag = issue_dma_tag_reg;
    assign dma_write_desc_qid = issue_qid_reg;
    assign dma_write_desc_valid = dma_write_valid_reg;

    assign st_tx_desc_host_addr = issue_host_addr_reg;
    assign st_tx_desc_len = issue_len_reg;
    assign st_tx_desc_tag = issue_dma_tag_reg;
    assign st_tx_desc_qid = issue_qid_reg;
    assign st_tx_desc_cidx_after = issue_cidx_after_reg;
    assign st_tx_desc_flags = issue_flags_reg;
    assign st_tx_desc_swtag = issue_swtag_reg;
    assign st_tx_desc_user = 32'd0;
    assign st_tx_desc_valid = st_tx_valid_reg;

    assign st_rx_desc_host_addr = issue_host_addr_reg;
    assign st_rx_desc_len = issue_len_reg;
    assign st_rx_desc_tag = issue_dma_tag_reg;
    assign st_rx_desc_qid = issue_qid_reg;
    assign st_rx_desc_cidx_after = issue_cidx_after_reg;
    assign st_rx_desc_flags = issue_flags_reg;
    assign st_rx_desc_swtag = issue_swtag_reg;
    assign st_rx_desc_user = 32'd0;
    assign st_rx_desc_wb_base = issue_entry_reg[DESC_ENTRY_WB_BASE_MSB:DESC_ENTRY_WB_BASE_LSB];
    assign st_rx_desc_irq_enable = issue_entry_reg[DESC_ENTRY_IRQ_ENABLE_MSB:DESC_ENTRY_IRQ_ENABLE_LSB];
    assign st_rx_desc_valid = st_rx_valid_reg;

    assign cmpl_write_addr = wb_addr_reg;
    assign cmpl_write_data = wb_data_reg;
    assign cmpl_write_valid = wb_valid_reg && !wb_inflight_reg;
    assign cmpl_write_fence = wb_fence_reg;

    assign ctx_update_valid = update_valid_reg;
    assign ctx_update_data[UPD_LSB_OP_DONE] = update_op_done_reg;
    assign ctx_update_data[UPD_LSB_REQUEUE] = update_requeue_reg;
    assign ctx_update_data[UPD_LSB_DESC_WE] = update_desc_we_reg;
    assign ctx_update_data[UPD_LSB_CMPL_WE] = update_cmpl_we_reg;
    assign ctx_update_data[(UPD_LSB_QID)+(QID_WIDTH)-1:(UPD_LSB_QID)] = update_qid_reg;
    assign ctx_update_data[(UPD_LSB_DESC_CIDX)+(RING_INDEX_WIDTH)-1:(UPD_LSB_DESC_CIDX)] = update_hw_desc_cidx_reg;
    assign ctx_update_data[(UPD_LSB_CMPL_PIDX)+(RING_INDEX_WIDTH)-1:(UPD_LSB_CMPL_PIDX)] = update_cmpl_pidx_reg;
    assign ctx_update_data[(UPD_LSB_STATUS)+(32)-1:(UPD_LSB_STATUS)] = update_status_reg;
    assign ctx_update_data[(UPD_LSB_ERROR_CODE)+(32)-1:(UPD_LSB_ERROR_CODE)] = update_error_code_reg;
    assign ctx_update_data[(UPD_LSB_PERF_DESC)+(32)-1:(UPD_LSB_PERF_DESC)] = update_perf_desc_fetch_reg;
    assign ctx_update_data[(UPD_LSB_PERF_DMA)+(32)-1:(UPD_LSB_PERF_DMA)] = update_perf_dma_submit_reg;
    assign ctx_update_data[(UPD_LSB_PERF_WB)+(32)-1:(UPD_LSB_PERF_WB)] = update_perf_wb_write_reg;
    assign ctx_update_data[(UPD_LSB_PERF_BYTES)+(32)-1:(UPD_LSB_PERF_BYTES)] = update_perf_bytes_reg;

    assign irq_status_set = irq_status_set_reg;
    assign irq_qid = irq_qid_reg;
    assign irq_valid = irq_valid_reg;

    assign sched_busy = (slot_occupancy_count_reg != 0) || desc_fetch_valid_reg || fetch_resp_active_reg ||
                        !mm_h2d_empty || !mm_d2h_empty || !st_tx_empty || !st_rx_empty ||
                        issue_entry_valid_reg || dma_read_valid_reg || dma_write_valid_reg ||
                        st_tx_valid_reg || st_rx_valid_reg || (|tag_bitmap_reg) || (|wb_ready_bitmap_reg) ||
                        !wb_fifo_empty || wb_valid_reg || wb_inflight_reg || !st_stat_fifo_empty ||
                        update_pend_valid_reg;
    assign sched_active_qid = issue_entry_valid_reg ? issue_qid_reg :
                              (fetch_resp_active_reg ? fetch_resp_qid_reg :
                               (desc_fetch_valid_reg ? fetch_pipe_qid_reg : {QID_WIDTH{1'b0}}));

endmodule

`resetall
