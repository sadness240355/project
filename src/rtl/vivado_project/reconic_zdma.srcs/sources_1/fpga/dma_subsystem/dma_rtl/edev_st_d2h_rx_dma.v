/* SPDX-License-Identifier: MIT
 * ST C2H/RX DMA core with native ST descriptor interface.
 *
 * The former MM-compatible wrapper has been removed from this source path.
 * edev_sgdma_scheduler now emits native ST descriptors and receives full ST
 * completion metadata, so this core is instantiated directly by fpga_core.
 */
`resetall
`timescale 1ns / 1ps
`default_nettype none

`include "edev_sgdma_defs.vh"

module edev_st_d2h_rx_dma_core #(
    parameter AXIS_PCIE_DATA_WIDTH = 256,
    parameter AXIS_PCIE_KEEP_WIDTH = AXIS_PCIE_DATA_WIDTH/32,
    parameter AXIS_ST_MTY_WIDTH = 6,
    parameter AXIS_PCIE_RQ_USER_WIDTH = AXIS_PCIE_DATA_WIDTH < 512 ? 60 : 137,
    parameter PCIE_ADDR_WIDTH = 64,
    parameter LEN_WIDTH = 24,
    parameter TAG_WIDTH = 12,
    parameter QID_WIDTH = 9,
    parameter RING_INDEX_WIDTH = 16,
    parameter FIFO_DEPTH = 32,
    parameter FIFO_ADDR_WIDTH = 5,
    parameter DESC_POOL_DEPTH = (1 << QID_WIDTH)
)(
    input  wire clk,
    input  wire rst,

    input  wire enable,
    input  wire [15:0] requester_id,
    input  wire requester_id_enable,
    input  wire [2:0] cfg_max_payload,

    /* Descriptor from target-aware scheduler */
    input  wire [PCIE_ADDR_WIDTH-1:0] s_axis_desc_host_addr,
    input  wire [LEN_WIDTH-1:0]       s_axis_desc_len,
    input  wire [TAG_WIDTH-1:0]       s_axis_desc_tag,
    input  wire [QID_WIDTH-1:0]       s_axis_desc_qid,
    input  wire [RING_INDEX_WIDTH-1:0]s_axis_desc_cidx_after,
    input  wire [15:0]                s_axis_desc_flags,
    input  wire [15:0]                s_axis_desc_swtag,
    input  wire [31:0]                s_axis_desc_user,
    input  wire [PCIE_ADDR_WIDTH-1:0] s_axis_desc_wb_base,
    input  wire [31:0]                s_axis_desc_irq_enable,
    input  wire                       s_axis_desc_valid,
    output wire                       s_axis_desc_ready,

    /* External stream input */
    input  wire [AXIS_PCIE_DATA_WIDTH-1:0] s_axis_st_rx_tdata,
    input  wire [AXIS_ST_MTY_WIDTH-1:0]    s_axis_st_rx_mty,
    input  wire                            s_axis_st_rx_tvalid,
    output wire                            s_axis_st_rx_tready,
    input  wire                            s_axis_st_rx_tlast,
    input  wire [31:0]                     s_axis_st_rx_tuser,
    input  wire [QID_WIDTH-1:0]            s_axis_st_rx_tqid,
    input  wire [LEN_WIDTH-1:0]            s_axis_st_rx_tlen,

    /* RQ Memory Write output */
    output wire [AXIS_PCIE_DATA_WIDTH-1:0]     m_axis_rq_tdata,
    output wire [AXIS_PCIE_KEEP_WIDTH-1:0]     m_axis_rq_tkeep,
    output wire                                m_axis_rq_tvalid,
    input  wire                                m_axis_rq_tready,
    output wire                                m_axis_rq_tlast,
    output wire [AXIS_PCIE_RQ_USER_WIDTH-1:0]  m_axis_rq_tuser,

    /* Completion/status back to scheduler */
    output wire [TAG_WIDTH-1:0]        m_axis_status_tag,
    output wire [QID_WIDTH-1:0]        m_axis_status_qid,
    output wire [RING_INDEX_WIDTH-1:0] m_axis_status_cidx_after,
    output wire [31:0]                 m_axis_status_bytes,
    output wire [31:0]                 m_axis_status_error_code,
    output wire [15:0]                 m_axis_status_swtag,
    output wire [31:0]                 m_axis_status_user,
    output wire [PCIE_ADDR_WIDTH-1:0]  m_axis_status_wb_base,
    output wire [31:0]                 m_axis_status_irq_enable,
    output wire                        m_axis_status_valid,
    input  wire                        m_axis_status_ready,

    output wire status_busy,
    output wire [31:0] stat_rx_packets,
    output wire [31:0] stat_rx_bytes,
    output wire [31:0] stat_trunc_packets,
    output wire [31:0] stat_drop_packets
);

    localparam integer FIFO_COUNT_WIDTH = FIFO_ADDR_WIDTH+1;
    localparam [FIFO_COUNT_WIDTH-1:0] FIFO_DEPTH_COUNT = FIFO_DEPTH;
    localparam integer AXIS_ST_BYTE_COUNT = AXIS_PCIE_DATA_WIDTH/8;
    localparam integer RQ_HEADER_BYTE_COUNT = 16;
    localparam integer FIRST_PAYLOAD_BYTE_COUNT = (AXIS_ST_BYTE_COUNT > RQ_HEADER_BYTE_COUNT) ?
                                                  (AXIS_ST_BYTE_COUNT - RQ_HEADER_BYTE_COUNT) : 0;
    localparam [7:0] AXIS_ST_BYTE_WIDTH = AXIS_ST_BYTE_COUNT;
    localparam [7:0] RQ_HEADER_BYTES = RQ_HEADER_BYTE_COUNT;
    localparam [7:0] FIRST_PAYLOAD_BYTES = FIRST_PAYLOAD_BYTE_COUNT;
    localparam [12:0] PCIE_4K_BYTES = 13'd4096;
    localparam [3:0] REQ_MEM_WRITE = 4'b0001;

    initial begin
        if (FIFO_DEPTH != (1 << FIFO_ADDR_WIDTH)) begin
            $error("FIFO_DEPTH must equal 2**FIFO_ADDR_WIDTH (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_DATA_WIDTH != 256) begin
            $error("aggregated ST_RX RQ path currently requires AXIS_PCIE_DATA_WIDTH=256 (instance %m)");
            $finish;
        end
        if (AXIS_ST_MTY_WIDTH != 6) begin
            $error("AXIS_ST_MTY_WIDTH must be 6 to match the QDMA mty convention (instance %m)");
            $finish;
        end
        if (DESC_POOL_DEPTH < (1 << QID_WIDTH)) begin
            $error("DESC_POOL_DEPTH must cover all qid values (instance %m)");
            $finish;
        end
    end

    /* RX stream contract used for timing closure:
     *  - every non-TLAST beat is a full beat, so mty is 0;
     *  - only the TLAST beat can be partial;
     *  - mty is the number of invalid bytes in the final beat.
     *
     * Decode helpers are expressed as explicit case/assign logic so Vivado
     * does not inline large helper cones into the RQ issue path.
     */
    wire [7:0] rx_mty_empty_bytes = {{(8-AXIS_ST_MTY_WIDTH){1'b0}}, s_axis_st_rx_mty};
    wire [7:0] rx_last_valid_bytes = (rx_mty_empty_bytes >= AXIS_ST_BYTE_WIDTH) ?
                                     8'd0 : (AXIS_ST_BYTE_WIDTH - rx_mty_empty_bytes);

    /* Payload FIFO storage.  The old version read fifo_data[fifo_rd_ptr] and
     * fifo_data[fifo_rd_ptr+1] as continuous assignments.  Vivado then had to
     * flatten the whole small RAM into large variable-index LUT muxes before
     * LUT mapping.  Keep the storage as synchronous RAM and expose only two
     * registered head words to the RQ packer.
     */
    (* ram_style="block" *) reg [AXIS_PCIE_DATA_WIDTH-1:0] fifo_data [0:FIFO_DEPTH-1];
    (* ram_style="block" *) reg [PCIE_ADDR_WIDTH-1:0]      fifo_addr [0:FIFO_DEPTH-1];
    (* ram_style="distributed" *) reg [7:0]                fifo_bytes[0:FIFO_DEPTH-1];

    reg [FIFO_ADDR_WIDTH-1:0] fifo_wr_ptr_reg = 0;
    reg [FIFO_ADDR_WIDTH-1:0] fifo_rd_ptr_reg = 0;
    reg [FIFO_COUNT_WIDTH-1:0] fifo_count_reg = 0;
    reg [15:0] fifo_byte_count_reg = 0;

    reg [AXIS_PCIE_DATA_WIDTH-1:0] fifo_cur_data_reg = 0;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] fifo_next_data_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0] fifo_cur_addr_reg = 0;
    reg [7:0] fifo_cur_bytes_reg = 0;
    reg [7:0] fifo_next_bytes_reg = 0;

    wire fifo_full = fifo_count_reg == FIFO_DEPTH_COUNT;
    wire fifo_bytes_empty = fifo_byte_count_reg == 0;

    wire [FIFO_ADDR_WIDTH-1:0] fifo_rd_ptr_next = fifo_rd_ptr_reg + 1'b1;
    wire [FIFO_ADDR_WIDTH-1:0] fifo_rd_ptr_plus2 = fifo_rd_ptr_reg + 2'd2;
    wire [FIFO_ADDR_WIDTH-1:0] fifo_rd_ptr_plus3 = fifo_rd_ptr_reg + 2'd3;
    wire [AXIS_PCIE_DATA_WIDTH-1:0] fifo_cur_data = fifo_cur_data_reg;
    wire [AXIS_PCIE_DATA_WIDTH-1:0] fifo_next_data = fifo_next_data_reg;
    wire [PCIE_ADDR_WIDTH-1:0] fifo_cur_addr = fifo_cur_addr_reg;
    wire [7:0] fifo_cur_bytes = fifo_cur_bytes_reg;
    wire [7:0] fifo_next_bytes = fifo_next_bytes_reg;

    reg active_desc_reg = 1'b0;
    reg [PCIE_ADDR_WIDTH-1:0] active_host_addr_reg = 0;
    reg [LEN_WIDTH-1:0] active_len_reg = 0;
    reg [LEN_WIDTH-1:0] active_offset_reg = 0;
    reg [TAG_WIDTH-1:0] active_tag_reg = 0;
    reg [QID_WIDTH-1:0] active_qid_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] active_cidx_after_reg = 0;
    reg [15:0] active_swtag_reg = 0;
    reg [31:0] active_user_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0] active_wb_base_reg = 0;
    reg [31:0] active_irq_enable_reg = 0;

    /* Per-queue ST RX descriptor pool.  Doorbell-fetched descriptors are
     * parked by qid until the external stream presents a packet for that qid.
     * A packet-start lookup is followed by a dedicated activation cycle that
     * copies the selected descriptor into active_* registers before any stream
     * beat is accepted.  This prevents the qid-indexed descriptor RAM address
     * from feeding the active_offset/status-byte cones while preserving the
     * one-beat-per-cycle payload path after the descriptor has been selected.
     */
    localparam integer DESC_POOL_LSB_HOST_ADDR  = 0;
    localparam integer DESC_POOL_LSB_LEN        = DESC_POOL_LSB_HOST_ADDR + PCIE_ADDR_WIDTH;
    localparam integer DESC_POOL_LSB_TAG        = DESC_POOL_LSB_LEN + LEN_WIDTH;
    localparam integer DESC_POOL_LSB_CIDX_AFTER = DESC_POOL_LSB_TAG + TAG_WIDTH;
    localparam integer DESC_POOL_LSB_SWTAG      = DESC_POOL_LSB_CIDX_AFTER + RING_INDEX_WIDTH;
    localparam integer DESC_POOL_LSB_USER       = DESC_POOL_LSB_SWTAG + 16;
    localparam integer DESC_POOL_LSB_WB_BASE    = DESC_POOL_LSB_USER + 32;
    localparam integer DESC_POOL_LSB_IRQ_ENABLE = DESC_POOL_LSB_WB_BASE + PCIE_ADDR_WIDTH;
    localparam integer DESC_POOL_ENTRY_WIDTH    = DESC_POOL_LSB_IRQ_ENABLE + 32;

    localparam integer DESC_WINDOW_SLOTS = 2;
    localparam integer DESC_WINDOW_PTR_WIDTH = 1;
    localparam integer DESC_WINDOW_COUNT_WIDTH = 2;
    localparam [DESC_WINDOW_COUNT_WIDTH-1:0] DESC_WINDOW_COUNT_MAX = 2'd2;
    localparam integer DESC_POOL_ADDR_WIDTH = QID_WIDTH + DESC_WINDOW_PTR_WIDTH;
    localparam integer DESC_POOL_TOTAL_DEPTH = DESC_POOL_DEPTH * DESC_WINDOW_SLOTS;

    /* The driver ABI has one writeback record and therefore one live RX
     * operation per queue.  Keep one active descriptor plus one elastic slot;
     * the synchronous UltraRAM-style store still accepts a following descriptor
     * without provisioning sixteen unreachable entries per queue. */
    (* ram_style="ultra" *) reg [DESC_POOL_ENTRY_WIDTH-1:0] desc_pool_mem [0:DESC_POOL_TOTAL_DEPTH-1];
    (* ram_style="distributed" *) reg [DESC_WINDOW_COUNT_WIDTH-1:0] desc_pool_count_mem [0:DESC_POOL_DEPTH-1];
    (* ram_style="distributed" *) reg [DESC_WINDOW_PTR_WIDTH-1:0] desc_pool_wr_ptr_mem [0:DESC_POOL_DEPTH-1];
    (* ram_style="distributed" *) reg [DESC_WINDOW_PTR_WIDTH-1:0] desc_pool_rd_ptr_mem [0:DESC_POOL_DEPTH-1];

    reg [DESC_POOL_ADDR_WIDTH-1:0] desc_pool_rd_addr_reg = {DESC_POOL_ADDR_WIDTH{1'b0}};
    reg [DESC_POOL_ENTRY_WIDTH-1:0] desc_pool_rd_data_reg = {DESC_POOL_ENTRY_WIDTH{1'b0}};

    localparam [2:0] DESC_STATE_CLEAR     = 3'd0;
    localparam [2:0] DESC_STATE_IDLE      = 3'd1;
    localparam [2:0] DESC_STATE_PUSH_RD   = 3'd2;
    localparam [2:0] DESC_STATE_PUSH_WR   = 3'd3;
    localparam [2:0] DESC_STATE_LOOK_RD   = 3'd4;
    localparam [2:0] DESC_STATE_LOOK_DATA = 3'd5;

    reg [2:0] desc_state_reg = DESC_STATE_CLEAR;
    reg [QID_WIDTH-1:0] desc_op_qid_reg = {QID_WIDTH{1'b0}};
    reg [DESC_WINDOW_PTR_WIDTH-1:0] desc_op_ptr_reg = {DESC_WINDOW_PTR_WIDTH{1'b0}};
    reg [DESC_WINDOW_COUNT_WIDTH-1:0] desc_op_count_reg = {DESC_WINDOW_COUNT_WIDTH{1'b0}};
    reg [QID_WIDTH-1:0] desc_clear_qid_reg = {QID_WIDTH{1'b0}};
    wire desc_window_idle = desc_state_reg == DESC_STATE_IDLE;
    wire desc_window_clearing = desc_state_reg == DESC_STATE_CLEAR;

    /* One-entry registered descriptor intake.  The scheduler-facing ready no
     * longer depends on the 512-entry qid-indexed pool-active vector, so a
     * blocked RX queue cannot create a scheduler -> RX-pool -> scheduler
     * combinational path.  If a descriptor arrives for a qid whose local RX
     * descriptor window is full, it waits here until that per-qid window has
     * space again.
     */
    reg desc_in_valid_reg = 1'b0;
    reg [PCIE_ADDR_WIDTH-1:0] desc_in_host_addr_reg = {PCIE_ADDR_WIDTH{1'b0}};
    reg [LEN_WIDTH-1:0] desc_in_len_reg = {LEN_WIDTH{1'b0}};
    reg [TAG_WIDTH-1:0] desc_in_tag_reg = {TAG_WIDTH{1'b0}};
    reg [QID_WIDTH-1:0] desc_in_qid_reg = {QID_WIDTH{1'b0}};
    reg [RING_INDEX_WIDTH-1:0] desc_in_cidx_after_reg = {RING_INDEX_WIDTH{1'b0}};
    reg [15:0] desc_in_swtag_reg = 16'd0;
    reg [31:0] desc_in_user_reg = 32'd0;
    reg [PCIE_ADDR_WIDTH-1:0] desc_in_wb_base_reg = {PCIE_ADDR_WIDTH{1'b0}};
    reg [31:0] desc_in_irq_enable_reg = 32'd0;

    reg [QID_WIDTH-1:0] rx_lookup_qid_reg = {QID_WIDTH{1'b0}};
    reg rx_lookup_pending_reg = 1'b0;
    reg rx_lookup_done_reg = 1'b0;
    reg rx_lookup_hit_reg = 1'b0;

    wire [PCIE_ADDR_WIDTH-1:0] rx_lookup_host_addr  = desc_pool_rd_data_reg[DESC_POOL_LSB_HOST_ADDR +: PCIE_ADDR_WIDTH];
    wire [LEN_WIDTH-1:0]       rx_lookup_len        = desc_pool_rd_data_reg[DESC_POOL_LSB_LEN +: LEN_WIDTH];
    wire [TAG_WIDTH-1:0]       rx_lookup_tag        = desc_pool_rd_data_reg[DESC_POOL_LSB_TAG +: TAG_WIDTH];
    wire [RING_INDEX_WIDTH-1:0]rx_lookup_cidx_after = desc_pool_rd_data_reg[DESC_POOL_LSB_CIDX_AFTER +: RING_INDEX_WIDTH];
    wire [15:0]                rx_lookup_swtag      = desc_pool_rd_data_reg[DESC_POOL_LSB_SWTAG +: 16];
    wire [31:0]                rx_lookup_user       = desc_pool_rd_data_reg[DESC_POOL_LSB_USER +: 32];
    wire [PCIE_ADDR_WIDTH-1:0] rx_lookup_wb_base    = desc_pool_rd_data_reg[DESC_POOL_LSB_WB_BASE +: PCIE_ADDR_WIDTH];
    wire [31:0]                rx_lookup_irq_enable = desc_pool_rd_data_reg[DESC_POOL_LSB_IRQ_ENABLE +: 32];

    reg [31:0] packet_user_reg = 0;
    reg [LEN_WIDTH-1:0] packet_len_reg = {LEN_WIDTH{1'b0}};
    reg [LEN_WIDTH-1:0] packet_guard_len_reg = {LEN_WIDTH{1'b0}};
    reg packet_meta_valid_reg = 1'b0;
    reg packet_len_over_desc_reg = 1'b0;
    reg in_packet_reg = 1'b0;
    reg packet_trunc_reg = 1'b0;
    reg rx_closing_reg = 1'b0;
    reg rx_drop_packet_reg = 1'b0;

    /* Stage RX0: register one accepted ST beat before the FIFO/write-status
     * work is performed.  This removes the tail-keep decode, overflow check,
     * packet bookkeeping, FIFO write, FIFO byte-count update, and RQ pop
     * update from one monolithic clocked block.  The pipe is refillable in the
     * same cycle it is consumed, so the RX input can still accept one beat per
     * cycle when the FIFO has space.
     */
    reg rx_pipe_valid_reg = 1'b0;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] rx_pipe_tdata_reg = 0;
    reg [7:0] rx_pipe_bytes_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0] rx_pipe_addr_reg = 0;
    reg rx_pipe_last_reg = 1'b0;
    reg rx_pipe_overflow_reg = 1'b0;
    reg [TAG_WIDTH-1:0] rx_pipe_status_tag_reg = 0;
    reg [QID_WIDTH-1:0] rx_pipe_status_qid_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] rx_pipe_status_cidx_after_reg = 0;
    reg [31:0] rx_pipe_status_bytes_reg = 0;
    reg [31:0] rx_pipe_status_error_code_reg = 0;
    reg [15:0] rx_pipe_status_swtag_reg = 0;
    reg [31:0] rx_pipe_status_user_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0] rx_pipe_status_wb_base_reg = 0;
    reg [31:0] rx_pipe_status_irq_enable_reg = 0;

    reg pending_status_reg = 1'b0;
    reg [TAG_WIDTH-1:0] status_tag_reg = 0;
    reg [QID_WIDTH-1:0] status_qid_reg = 0;
    reg [RING_INDEX_WIDTH-1:0] status_cidx_after_reg = 0;
    reg [31:0] status_bytes_reg = 0;
    reg [31:0] status_error_code_reg = 0;
    reg [15:0] status_swtag_reg = 0;
    reg [31:0] status_user_reg = 0;
    reg [PCIE_ADDR_WIDTH-1:0] status_wb_base_reg = 0;
    reg [31:0] status_irq_enable_reg = 0;
    reg status_valid_reg = 1'b0;

    reg [31:0] stat_rx_packets_reg = 0;
    reg [31:0] stat_rx_bytes_reg = 0;
    reg [31:0] stat_trunc_packets_reg = 0;
    reg [31:0] stat_drop_packets_reg = 0;

    localparam [1:0] TX_STATE_IDLE = 2'd0;
    localparam [1:0] TX_STATE_DATA = 2'd1;

    reg [1:0] tx_state_reg = TX_STATE_IDLE;
    reg [1:0] tx_state_next;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] rq_tdata_reg = 0;
    reg [AXIS_PCIE_KEEP_WIDTH-1:0] rq_tkeep_reg = 0;
    reg rq_tvalid_reg = 1'b0;
    reg rq_tlast_reg = 1'b0;
    reg [AXIS_PCIE_RQ_USER_WIDTH-1:0] rq_tuser_reg = 0;

    reg [10:0] tx_payload_rem_reg = 0;
    /* Number of bytes already consumed from fifo_cur_data_reg.  A 4KB split
     * can fall in the middle of a 32-byte ST beat, so the entry must remain at
     * the FIFO head until all of its valid bytes have been emitted. */
    reg [5:0] tx_head_offset_reg = 0;
    reg [10:0] cfg_mps_bytes_reg = 11'd128;

    /* Three-stage look-ahead planner for the next Memory Write TLP.
     *
     * The page/MPS calculation used to sit directly between fifo_cur_addr_reg
     * and the FIFO-head update mux.  A boundary decision therefore fed back
     * through payload sizing, byte consumption, head-offset arithmetic, and
     * the registered FIFO window in one 250 MHz cycle.  Keep the byte address
     * and request limit in dedicated registers, then register the exact launch
     * length after enough unreserved bytes are available.  For a normal
     * MPS-sized request the following plan is prepared while the current TLP
     * data beats are being emitted, so this pipeline adds startup latency but
     * no steady-state bubble between 128-byte TLPs.
     */
    reg [PCIE_ADDR_WIDTH-1:0] tx_plan_addr_reg = 0;
    reg [12:0] tx_plan_limit_reg = 13'd0;
    reg tx_plan_valid_reg = 1'b0;
    reg [PCIE_ADDR_WIDTH-1:0] tx_plan_pending_addr_reg = 0;
    reg tx_plan_pending_valid_reg = 1'b0;
    reg [PCIE_ADDR_WIDTH-1:0] tx_launch_addr_reg = 0;
    reg [10:0] tx_launch_payload_len_reg = 11'd0;
    reg tx_launch_is_tail_reg = 1'b0;
    reg tx_launch_valid_reg = 1'b0;

    wire rq_ready_for_issue = !rq_tvalid_reg || m_axis_rq_tready;

    wire [7:0] rx_raw_accept_bytes = s_axis_st_rx_tlast ? rx_last_valid_bytes : AXIS_ST_BYTE_WIDTH;
    wire [QID_WIDTH-1:0] rx_sel_qid = s_axis_st_rx_tqid;
    wire rx_start_packet = !in_packet_reg && !active_desc_reg && !rx_drop_packet_reg;
    wire rx_lookup_busy = rx_lookup_pending_reg || rx_lookup_done_reg || !desc_window_idle;
    wire rx_start_needs_lookup = rx_start_packet && !rx_lookup_busy;
    wire rx_have_desc = active_desc_reg;
    wire rx_start_len_valid = s_axis_st_rx_tlen != {LEN_WIDTH{1'b0}};
    wire [LEN_WIDTH-1:0] rx_start_guard_len = (rx_start_len_valid && (s_axis_st_rx_tlen < active_len_reg)) ?
                                             s_axis_st_rx_tlen : active_len_reg;
    wire [LEN_WIDTH-1:0] rx_guard_len = packet_guard_len_reg;
    wire [LEN_WIDTH-1:0] rx_remaining_bytes = (active_offset_reg < rx_guard_len) ?
                                              (rx_guard_len - active_offset_reg) : {LEN_WIDTH{1'b0}};
    wire rx_raw_over_guard = {{(LEN_WIDTH-8){1'b0}}, rx_raw_accept_bytes} > rx_remaining_bytes;
    wire [7:0] rx_accept_bytes = rx_raw_over_guard ? rx_remaining_bytes[7:0] : rx_raw_accept_bytes;
    wire rx_decl_len_over_desc = !in_packet_reg && packet_len_over_desc_reg;
    wire rx_accept_overflow = rx_have_desc && (rx_raw_over_guard || rx_decl_len_over_desc);
    wire [LEN_WIDTH-1:0] rx_accept_next_offset = active_offset_reg + {{(LEN_WIDTH-8){1'b0}}, rx_accept_bytes};
    wire [31:0] rx_accept_status_user = packet_user_reg;
    wire [31:0] rx_accept_status_bytes = {{(32-LEN_WIDTH){1'b0}}, rx_accept_next_offset};
    wire [31:0] rx_accept_status_error = (packet_trunc_reg || rx_accept_overflow) ?
                                         {`EDEV_ERR_ST_RX_TRUNC, 8'h00, 16'd0} : 32'd0;

    wire rx_pipe_can_push_fifo = rx_pipe_valid_reg && !rx_pipe_overflow_reg && (rx_pipe_bytes_reg != 0);
    wire rx_pipe_process = rx_pipe_valid_reg && (!rx_pipe_can_push_fifo || !fifo_full);
    wire rx_pipe_ready = !rx_pipe_valid_reg || rx_pipe_process;

    assign s_axis_desc_ready = enable && !desc_window_clearing && !desc_in_valid_reg;
    wire desc_fire = s_axis_desc_valid && s_axis_desc_ready;
    wire stream_fire = s_axis_st_rx_tvalid && s_axis_st_rx_tready;
    wire rx_lookup_start = enable && s_axis_st_rx_tvalid && rx_start_needs_lookup &&
                           desc_window_idle && !desc_fire && !desc_in_valid_reg && !pending_status_reg &&
                           rx_pipe_ready && !fifo_full && !rx_closing_reg;
    wire rx_packet_meta_load = enable && s_axis_st_rx_tvalid && active_desc_reg &&
                               !in_packet_reg && !packet_meta_valid_reg &&
                               !rx_drop_packet_reg && !pending_status_reg && !rx_closing_reg;
    assign s_axis_st_rx_tready = enable && !rx_closing_reg &&
                                 !pending_status_reg && rx_pipe_ready && !fifo_full &&
                                 (in_packet_reg || rx_drop_packet_reg ||
                                  (active_desc_reg && packet_meta_valid_reg));

    /* Compute every MWr payload from the byte address that has actually been
     * consumed, not merely from the base address of the current FIFO entry.
     * For a DWORD-aligned descriptor this is exactly
     *
     *   min(remaining, MPS, 4096-address[11:0]).
     *
     * Byte-unaligned descriptors additionally reserve address[1:0] disabled
     * bytes in the first payload DWORD so the encoded DWORD span still obeys
     * MPS.  This is the same rule used by the generic PCIe write engine.
     */
    wire [PCIE_ADDR_WIDTH-1:0] tx_fifo_head_addr =
        fifo_cur_addr + {{(PCIE_ADDR_WIDTH-6){1'b0}}, tx_head_offset_reg};
    wire [12:0] tx_plan_boundary_bytes =
        PCIE_4K_BYTES - {1'b0, tx_plan_pending_addr_reg[11:0]};
    wire [12:0] tx_plan_mps_bytes = {2'b00, cfg_mps_bytes_reg};
    wire [12:0] tx_plan_mps_payload_limit =
        (tx_plan_mps_bytes > {11'd0, tx_plan_pending_addr_reg[1:0]}) ?
        (tx_plan_mps_bytes - {11'd0, tx_plan_pending_addr_reg[1:0]}) : 13'd1;
    wire [12:0] tx_plan_request_limit =
        (tx_plan_mps_payload_limit < tx_plan_boundary_bytes) ?
        tx_plan_mps_payload_limit : tx_plan_boundary_bytes;

    wire [15:0] tx_plan_limit_ext = {3'd0, tx_plan_limit_reg};
    wire [15:0] tx_reserved_payload_bytes =
        (tx_state_reg == TX_STATE_DATA) ? {5'd0, tx_payload_rem_reg} : 16'd0;
    wire [15:0] tx_plan_available_bytes =
        (fifo_byte_count_reg > tx_reserved_payload_bytes) ?
        (fifo_byte_count_reg - tx_reserved_payload_bytes) : 16'd0;
    wire tx_plan_has_full_request = tx_plan_available_bytes >= tx_plan_limit_ext;
    /* An unaligned/page-split head can occupy one FIFO entry after some of
     * its bytes have been consumed.  At 1024-byte MPS the 32-word FIFO can
     * then be full with fewer than 1024 usable bytes.  Waiting for the full
     * MPS would deadlock RX against TX.  PCIe permits a smaller MWr: reserve
     * the available positive byte count under FIFO pressure, using the same
     * page/MPS bound and ownership token as ordinary requests. */
    wire tx_plan_has_pressure_request = fifo_full &&
                                        (tx_plan_available_bytes != 16'd0);
    wire tx_plan_has_tail_request = pending_status_reg &&
                                    (tx_plan_available_bytes != 16'd0);
    wire [10:0] tx_plan_payload_len = tx_plan_has_full_request ?
                                      tx_plan_limit_reg[10:0] :
                                      tx_plan_available_bytes[10:0];
    wire tx_plan_is_packet_tail = pending_status_reg &&
                                  (tx_plan_available_bytes <= tx_plan_limit_ext);

    wire [PCIE_ADDR_WIDTH-1:0] tx_current_addr = tx_launch_addr_reg;
    wire [10:0] tx_start_payload_len = tx_launch_payload_len_reg;
    /* The launch register is an ownership token for already buffered bytes.
     * Its producer reserves only fifo_byte_count - current_TLP_remaining;
     * until this token is consumed, only that current TLP can remove bytes.
     * Thus, when IDLE consumes the token, its entire nonzero payload is still
     * buffered.  Rechecking a 16-bit byte count here is redundant and closes
     * a long byte-counter -> head-window update loop.  Reset clears the token
     * together with the FIFO.  Keep availability checking at token creation.
     */
    wire tx_can_start = tx_launch_valid_reg;

    wire [7:0] tx_first_payload_capacity = 8'd16 - {6'd0, tx_current_addr[1:0]};
    wire [7:0] tx_first_payload_bytes =
        (tx_start_payload_len > {3'd0, tx_first_payload_capacity}) ?
        tx_first_payload_capacity : tx_start_payload_len[7:0];
    wire [7:0] tx_first_total_bytes =
        8'd16 + {6'd0, tx_current_addr[1:0]} + tx_first_payload_bytes;
    wire [10:0] tx_remaining_payload = tx_payload_rem_reg;
    wire [7:0] tx_data_payload_bytes = (tx_remaining_payload > 11'd32) ? 8'd32 : tx_remaining_payload[7:0];
    wire tx_data_last = tx_remaining_payload <= 11'd32;

    /* A registered two-entry FIFO window feeds one byte-alignment mux.  The
     * mux terminates at the registered RQ output, while the FIFO accounting
     * below retains any bytes after a page/MPS split. */
    wire [2*AXIS_PCIE_DATA_WIDTH-1:0] tx_fifo_data_window = {fifo_next_data, fifo_cur_data};
    wire [2*AXIS_PCIE_DATA_WIDTH-1:0] tx_fifo_data_aligned =
        tx_fifo_data_window >> {tx_head_offset_reg, 3'b000};
    wire [127:0] tx_first_payload_aligned =
        tx_fifo_data_aligned[127:0] << {tx_current_addr[1:0], 3'b000};
    wire [8:0] tx_cur_remaining_bytes =
        ({1'b0, fifo_cur_bytes} > {3'd0, tx_head_offset_reg}) ?
        ({1'b0, fifo_cur_bytes} - {3'd0, tx_head_offset_reg}) : 9'd0;

    reg [127:0] tx_header;
    reg [AXIS_PCIE_RQ_USER_WIDTH-1:0] tx_tuser;
    reg [10:0] tx_dword_count;
    reg [3:0] tx_first_be;
    reg [3:0] tx_last_be;
    reg [AXIS_PCIE_KEEP_WIDTH-1:0] tx_first_keep;
    reg [AXIS_PCIE_KEEP_WIDTH-1:0] tx_data_keep;
    reg [4:0] tx_first_keep_dw;
    reg [4:0] tx_data_keep_dw;

    reg tx_issue_valid;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] tx_issue_tdata;
    reg [AXIS_PCIE_KEEP_WIDTH-1:0] tx_issue_tkeep;
    reg tx_issue_tlast;
    reg [AXIS_PCIE_RQ_USER_WIDTH-1:0] tx_issue_tuser;
    reg [1:0] tx_issue_pop_count;
    reg [10:0] tx_issue_consume_bytes;
    reg [FIFO_ADDR_WIDTH-1:0] tx_issue_rd_ptr_next;
    reg [10:0] tx_payload_rem_next;
    reg [5:0] tx_head_offset_next;
    reg tx_issue_extra_pop;

    /* Registered delayed correction for the only two-entry consumption
     * case.  At most two FIFO entries can be retired by one 32-byte RQ payload
     * beat.  Registering the second pop cuts that rare tail correction out of
     * the fifo_count_reg input cone.
     */
    reg tx_extra_pop_pending_reg = 1'b0;

    assign m_axis_rq_tdata = rq_tdata_reg;
    assign m_axis_rq_tkeep = rq_tkeep_reg;
    assign m_axis_rq_tvalid = rq_tvalid_reg;
    assign m_axis_rq_tlast = rq_tlast_reg;
    assign m_axis_rq_tuser = rq_tuser_reg;

    assign m_axis_status_tag = status_tag_reg;
    assign m_axis_status_qid = status_qid_reg;
    assign m_axis_status_cidx_after = status_cidx_after_reg;
    assign m_axis_status_bytes = status_bytes_reg;
    assign m_axis_status_error_code = status_error_code_reg;
    assign m_axis_status_swtag = status_swtag_reg;
    assign m_axis_status_user = status_user_reg;
    assign m_axis_status_wb_base = status_wb_base_reg;
    assign m_axis_status_irq_enable = status_irq_enable_reg;
    assign m_axis_status_valid = status_valid_reg;

    assign status_busy = desc_in_valid_reg || !desc_window_idle || rx_lookup_pending_reg || rx_lookup_done_reg ||
                         active_desc_reg || packet_meta_valid_reg || rx_drop_packet_reg ||
                         rx_pipe_valid_reg || pending_status_reg ||
                         status_valid_reg || !fifo_bytes_empty || rq_tvalid_reg ||
                         tx_plan_valid_reg || tx_plan_pending_valid_reg || tx_launch_valid_reg ||
                         tx_state_reg != TX_STATE_IDLE;
    assign stat_rx_packets = stat_rx_packets_reg;
    assign stat_rx_bytes = stat_rx_bytes_reg;
    assign stat_trunc_packets = stat_trunc_packets_reg;
    assign stat_drop_packets = stat_drop_packets_reg;

    wire _unused_desc_flags = &{1'b0, s_axis_desc_flags, 1'b0};

    /* TX/RQ decode stage.  Header and sideband generation are isolated from
     * the sequential update block.  The registered RQ output still follows the
     * AXI rule that valid data is held stable until tready accepts it.
     */
    always @* begin
        tx_dword_count =
            (tx_start_payload_len + {9'd0, tx_current_addr[1:0]} + 11'd3) >> 2;

        tx_first_be = 4'b1111 << tx_current_addr[1:0];
        case ((tx_current_addr[1:0] + tx_start_payload_len[1:0] - 1'b1) & 2'b11)
            2'd0: tx_last_be = 4'b0001;
            2'd1: tx_last_be = 4'b0011;
            2'd2: tx_last_be = 4'b0111;
            default: tx_last_be = 4'b1111;
        endcase

        tx_first_keep_dw = (tx_first_total_bytes + 8'd3) >> 2;
        tx_data_keep_dw = (tx_data_payload_bytes + 8'd3) >> 2;
        tx_first_keep = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
        tx_data_keep = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
        case (tx_first_keep_dw)
            5'd0: tx_first_keep = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
            5'd1: tx_first_keep = {{(AXIS_PCIE_KEEP_WIDTH-1){1'b0}}, 1'b1};
            5'd2: tx_first_keep = {{(AXIS_PCIE_KEEP_WIDTH-2){1'b0}}, 2'b11};
            5'd3: tx_first_keep = {{(AXIS_PCIE_KEEP_WIDTH-3){1'b0}}, 3'b111};
            5'd4: tx_first_keep = {{(AXIS_PCIE_KEEP_WIDTH-4){1'b0}}, 4'hf};
            5'd5: tx_first_keep = {{(AXIS_PCIE_KEEP_WIDTH-5){1'b0}}, 5'h1f};
            5'd6: tx_first_keep = {{(AXIS_PCIE_KEEP_WIDTH-6){1'b0}}, 6'h3f};
            5'd7: tx_first_keep = {{(AXIS_PCIE_KEEP_WIDTH-7){1'b0}}, 7'h7f};
            default: tx_first_keep = {AXIS_PCIE_KEEP_WIDTH{1'b1}};
        endcase
        case (tx_data_keep_dw)
            5'd0: tx_data_keep = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
            5'd1: tx_data_keep = {{(AXIS_PCIE_KEEP_WIDTH-1){1'b0}}, 1'b1};
            5'd2: tx_data_keep = {{(AXIS_PCIE_KEEP_WIDTH-2){1'b0}}, 2'b11};
            5'd3: tx_data_keep = {{(AXIS_PCIE_KEEP_WIDTH-3){1'b0}}, 3'b111};
            5'd4: tx_data_keep = {{(AXIS_PCIE_KEEP_WIDTH-4){1'b0}}, 4'hf};
            5'd5: tx_data_keep = {{(AXIS_PCIE_KEEP_WIDTH-5){1'b0}}, 5'h1f};
            5'd6: tx_data_keep = {{(AXIS_PCIE_KEEP_WIDTH-6){1'b0}}, 6'h3f};
            5'd7: tx_data_keep = {{(AXIS_PCIE_KEEP_WIDTH-7){1'b0}}, 7'h7f};
            default: tx_data_keep = {AXIS_PCIE_KEEP_WIDTH{1'b1}};
        endcase

        tx_header = 128'd0;
        tx_header[1:0] = 2'b00;
        tx_header[63:2] = tx_current_addr[PCIE_ADDR_WIDTH-1:2];
        tx_header[74:64] = tx_dword_count;
        tx_header[78:75] = REQ_MEM_WRITE;
        tx_header[79] = 1'b0;
        tx_header[95:80] = requester_id;
        tx_header[103:96] = 8'd0;
        tx_header[119:104] = 16'd0;
        tx_header[120] = requester_id_enable;
        tx_header[123:121] = 3'b000;
        tx_header[126:124] = 3'b000;
        tx_header[127] = 1'b0;

        tx_tuser = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
        if (tx_dword_count == 1) begin
            tx_tuser[3:0] = tx_first_be & tx_last_be;
            tx_tuser[7:4] = 4'b0000;
        end else begin
            tx_tuser[3:0] = tx_first_be;
            tx_tuser[7:4] = tx_last_be;
        end
        tx_tuser[10:8] = 3'd0;
        tx_tuser[11] = 1'b0;
        tx_tuser[12] = 1'b0;
        tx_tuser[14:13] = 2'b00;
        tx_tuser[15] = 1'b0;
        tx_tuser[23:16] = 8'd0;
        tx_tuser[27:24] = 4'd0;
    end

    /* TX/RQ next-state and action decode. */
    always @* begin
        tx_state_next = tx_state_reg;
        tx_issue_valid = 1'b0;
        tx_issue_tdata = {AXIS_PCIE_DATA_WIDTH{1'b0}};
        tx_issue_tkeep = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
        tx_issue_tlast = 1'b0;
        tx_issue_tuser = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
        tx_issue_pop_count = 2'd0;
        tx_issue_consume_bytes = 11'd0;
        tx_issue_rd_ptr_next = fifo_rd_ptr_reg;
        tx_payload_rem_next = tx_payload_rem_reg;
        tx_head_offset_next = tx_head_offset_reg;
        tx_issue_extra_pop = 1'b0;

        case (tx_state_reg)
            TX_STATE_IDLE: begin
                if (tx_can_start) begin
                    tx_issue_valid = 1'b1;
                    tx_issue_tdata = {tx_first_payload_aligned, tx_header};
                    tx_issue_tkeep = tx_first_keep;
                    tx_issue_tlast = (tx_start_payload_len <= {3'd0, tx_first_payload_capacity});
                    tx_issue_tuser = tx_tuser;
                    tx_issue_consume_bytes = {3'd0, tx_first_payload_bytes};
                    tx_payload_rem_next = tx_start_payload_len - {3'd0, tx_first_payload_bytes};

                    if (tx_start_payload_len <= {3'd0, tx_first_payload_capacity}) begin
                        tx_state_next = TX_STATE_IDLE;
                    end else begin
                        tx_state_next = TX_STATE_DATA;
                    end
                end
            end
            TX_STATE_DATA: begin
                tx_issue_valid = 1'b1;
                tx_issue_tdata = tx_fifo_data_aligned[AXIS_PCIE_DATA_WIDTH-1:0];
                tx_issue_tkeep = tx_data_keep;
                tx_issue_tlast = tx_data_last;
                tx_issue_tuser = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                tx_issue_consume_bytes = {3'd0, tx_data_payload_bytes};
                tx_payload_rem_next = tx_data_last ? 11'd0 :
                                      (tx_payload_rem_reg - {3'd0, tx_data_payload_bytes});

                tx_state_next = tx_data_last ? TX_STATE_IDLE : TX_STATE_DATA;
            end
            default: begin
                tx_state_next = TX_STATE_IDLE;
            end
        endcase

        /* Advance within the current raw ST beat first; retire a FIFO entry
         * only when all of its valid bytes have been consumed.  This is the
         * key state that preserves data when a 4KB boundary falls mid-beat. */
        if (tx_issue_valid && tx_issue_consume_bytes != 0) begin
            if ({1'b0, tx_issue_consume_bytes} < tx_cur_remaining_bytes) begin
                tx_head_offset_next = tx_head_offset_reg + tx_issue_consume_bytes[5:0];
            end else if ({1'b0, tx_issue_consume_bytes} == tx_cur_remaining_bytes) begin
                tx_issue_pop_count = 2'd1;
                tx_issue_rd_ptr_next = fifo_rd_ptr_next;
                tx_head_offset_next = 6'd0;
            end else begin
                tx_issue_pop_count = 2'd1;
                tx_issue_rd_ptr_next = fifo_rd_ptr_next;
                tx_head_offset_next = tx_issue_consume_bytes[5:0] - tx_cur_remaining_bytes[5:0];
                if (({1'b0, tx_issue_consume_bytes} - tx_cur_remaining_bytes) >=
                    {1'b0, fifo_next_bytes}) begin
                    tx_issue_pop_count = 2'd2;
                    tx_issue_rd_ptr_next = fifo_rd_ptr_plus2;
                    tx_head_offset_next = 6'd0;
                    tx_issue_extra_pop = 1'b1;
                end
            end
        end
    end

    /* Sequential state/update stage. */
    reg push_fifo;
    reg [1:0] pop_fifo_count;
    reg [10:0] consume_payload_bytes;
    reg tx_extra_pop_apply;

    wire tx_window_update = rq_ready_for_issue && tx_issue_valid;

    always @(posedge clk) begin
        push_fifo = 1'b0;
        pop_fifo_count = 2'd0;
        consume_payload_bytes = 11'd0;
        tx_extra_pop_apply = tx_extra_pop_pending_reg;
        tx_extra_pop_pending_reg <= 1'b0;
        case (cfg_max_payload)
            3'd0: cfg_mps_bytes_reg <= 11'd128;
            3'd1: cfg_mps_bytes_reg <= 11'd256;
            3'd2: cfg_mps_bytes_reg <= 11'd512;
            3'd3: cfg_mps_bytes_reg <= 11'd1024;
            default: cfg_mps_bytes_reg <= 11'd1024;
        endcase

        /* TLP planning pipeline.  Capturing the FIFO-head address is a short
         * register-to-register path.  Page/MPS subtraction and comparison end
         * at tx_plan_limit_reg one cycle later and cannot feed the FIFO window
         * update in the same cycle. */
        if (tx_plan_pending_valid_reg && !tx_plan_valid_reg) begin
            tx_plan_addr_reg <= tx_plan_pending_addr_reg;
            tx_plan_limit_reg <= tx_plan_request_limit;
            tx_plan_valid_reg <= 1'b1;
            tx_plan_pending_valid_reg <= 1'b0;
        end else if (!tx_plan_valid_reg && !tx_plan_pending_valid_reg &&
                     !tx_launch_valid_reg && tx_state_reg == TX_STATE_IDLE &&
                     !fifo_bytes_empty) begin
            tx_plan_pending_addr_reg <= tx_fifo_head_addr;
            tx_plan_pending_valid_reg <= 1'b1;
        end

        /* Launch stage: register the exact payload length only after bytes not
         * reserved by the current TLP are available.  A completed short tail
         * uses the remaining byte count; all other requests use the registered
         * page/MPS limit. */
        if (tx_plan_valid_reg && !tx_launch_valid_reg &&
            (tx_plan_has_full_request || tx_plan_has_tail_request ||
             tx_plan_has_pressure_request)) begin
            tx_launch_addr_reg <= tx_plan_addr_reg;
            tx_launch_payload_len_reg <= tx_plan_payload_len;
            tx_launch_is_tail_reg <= tx_plan_is_packet_tail;
            tx_launch_valid_reg <= 1'b1;
            tx_plan_valid_reg <= 1'b0;
        end

        case (desc_state_reg)
            DESC_STATE_CLEAR: begin
                desc_pool_count_mem[desc_clear_qid_reg] <= {DESC_WINDOW_COUNT_WIDTH{1'b0}};
                desc_pool_wr_ptr_mem[desc_clear_qid_reg] <= {DESC_WINDOW_PTR_WIDTH{1'b0}};
                desc_pool_rd_ptr_mem[desc_clear_qid_reg] <= {DESC_WINDOW_PTR_WIDTH{1'b0}};
                if (desc_clear_qid_reg == {QID_WIDTH{1'b1}}) begin
                    desc_state_reg <= DESC_STATE_IDLE;
                end else begin
                    desc_clear_qid_reg <= desc_clear_qid_reg + 1'b1;
                end
            end
            DESC_STATE_IDLE: begin
                if (desc_in_valid_reg) begin
                    desc_op_qid_reg <= desc_in_qid_reg;
                    desc_state_reg <= DESC_STATE_PUSH_RD;
                end else if (rx_lookup_start) begin
                    desc_op_qid_reg <= rx_sel_qid;
                    rx_lookup_qid_reg <= rx_sel_qid;
                    rx_lookup_pending_reg <= 1'b1;
                    rx_lookup_done_reg <= 1'b0;
                    rx_lookup_hit_reg <= 1'b0;
                    desc_state_reg <= DESC_STATE_LOOK_RD;
                end
            end
            DESC_STATE_PUSH_RD: begin
                desc_op_count_reg <= desc_pool_count_mem[desc_op_qid_reg];
                desc_op_ptr_reg <= desc_pool_wr_ptr_mem[desc_op_qid_reg];
                desc_state_reg <= DESC_STATE_PUSH_WR;
            end
            DESC_STATE_PUSH_WR: begin
                if (desc_op_count_reg < DESC_WINDOW_COUNT_MAX) begin
                    desc_pool_mem[{desc_op_qid_reg, desc_op_ptr_reg}] <= {
                        desc_in_irq_enable_reg,
                        desc_in_wb_base_reg,
                        desc_in_user_reg,
                        desc_in_swtag_reg,
                        desc_in_cidx_after_reg,
                        desc_in_tag_reg,
                        desc_in_len_reg,
                        desc_in_host_addr_reg
                    };
                    desc_pool_wr_ptr_mem[desc_op_qid_reg] <= desc_op_ptr_reg + 1'b1;
                    desc_pool_count_mem[desc_op_qid_reg] <= desc_op_count_reg + 1'b1;
                    desc_in_valid_reg <= 1'b0;
                end
                desc_state_reg <= DESC_STATE_IDLE;
            end
            DESC_STATE_LOOK_RD: begin
                desc_op_count_reg <= desc_pool_count_mem[desc_op_qid_reg];
                desc_op_ptr_reg <= desc_pool_rd_ptr_mem[desc_op_qid_reg];
                desc_pool_rd_addr_reg <= {desc_op_qid_reg, desc_pool_rd_ptr_mem[desc_op_qid_reg]};
                desc_state_reg <= DESC_STATE_LOOK_DATA;
            end
            DESC_STATE_LOOK_DATA: begin
                desc_pool_rd_data_reg <= desc_pool_mem[desc_pool_rd_addr_reg];
                rx_lookup_pending_reg <= 1'b0;
                rx_lookup_done_reg <= 1'b1;
                rx_lookup_hit_reg <= desc_op_count_reg != 0;
                if (desc_op_count_reg != 0) begin
                    desc_pool_rd_ptr_mem[desc_op_qid_reg] <= desc_op_ptr_reg + 1'b1;
                    desc_pool_count_mem[desc_op_qid_reg] <= desc_op_count_reg - 1'b1;
                end
                desc_state_reg <= DESC_STATE_IDLE;
            end
            default: begin
                desc_state_reg <= DESC_STATE_CLEAR;
            end
        endcase

        if (desc_fire) begin
            desc_in_valid_reg <= 1'b1;
            desc_in_host_addr_reg <= s_axis_desc_host_addr;
            desc_in_len_reg <= s_axis_desc_len;
            desc_in_tag_reg <= s_axis_desc_tag;
            desc_in_qid_reg <= s_axis_desc_qid;
            desc_in_cidx_after_reg <= s_axis_desc_cidx_after;
            desc_in_swtag_reg <= s_axis_desc_swtag;
            desc_in_user_reg <= s_axis_desc_user;
            desc_in_wb_base_reg <= s_axis_desc_wb_base;
            desc_in_irq_enable_reg <= s_axis_desc_irq_enable;
        end


        /* Descriptor lookup is intentionally split from stream acceptance.
         * The pool RAM address register only feeds the pool output register;
         * a later cycle transfers the decoded descriptor into active_* regs.
         * This removes desc_pool_rd_addr_reg from the active_offset/status-byte
         * cones reported by timing while preserving full beat-rate throughput
         * once a packet is active.
         */
        if (rx_lookup_done_reg && !active_desc_reg && !rx_drop_packet_reg) begin
            rx_lookup_done_reg <= 1'b0;
            if (rx_lookup_hit_reg) begin
                active_desc_reg       <= 1'b1;
                active_host_addr_reg  <= rx_lookup_host_addr;
                active_len_reg        <= rx_lookup_len;
                active_offset_reg     <= {LEN_WIDTH{1'b0}};
                active_tag_reg        <= rx_lookup_tag;
                active_qid_reg        <= rx_lookup_qid_reg;
                active_cidx_after_reg <= rx_lookup_cidx_after;
                active_swtag_reg      <= rx_lookup_swtag;
                active_user_reg       <= rx_lookup_user;
                active_wb_base_reg    <= rx_lookup_wb_base;
                active_irq_enable_reg <= rx_lookup_irq_enable;
                packet_len_reg        <= {LEN_WIDTH{1'b0}};
                packet_guard_len_reg  <= {LEN_WIDTH{1'b0}};
                packet_meta_valid_reg <= 1'b0;
                packet_len_over_desc_reg <= 1'b0;
                packet_trunc_reg      <= 1'b0;
            end else begin
                rx_drop_packet_reg <= 1'b1;
                packet_len_reg <= {LEN_WIDTH{1'b0}};
                packet_guard_len_reg <= {LEN_WIDTH{1'b0}};
                packet_meta_valid_reg <= 1'b0;
                packet_len_over_desc_reg <= 1'b0;
                packet_trunc_reg <= 1'b0;
            end
        end


        if (rx_packet_meta_load) begin
            packet_user_reg <= s_axis_st_rx_tuser;
            packet_len_reg <= s_axis_st_rx_tlen;
            packet_guard_len_reg <= rx_start_guard_len;
            packet_meta_valid_reg <= 1'b1;
            packet_len_over_desc_reg <= rx_start_len_valid && (s_axis_st_rx_tlen > active_len_reg);
            packet_trunc_reg <= 1'b0;
        end

        if (rx_pipe_process) begin
            if (rx_pipe_can_push_fifo) begin
                fifo_data[fifo_wr_ptr_reg] <= rx_pipe_tdata_reg;
                fifo_addr[fifo_wr_ptr_reg] <= rx_pipe_addr_reg;
                fifo_bytes[fifo_wr_ptr_reg] <= rx_pipe_bytes_reg;
                fifo_wr_ptr_reg <= fifo_wr_ptr_reg + 1'b1;
                push_fifo = 1'b1;
            end

            if (rx_pipe_last_reg) begin
                pending_status_reg <= 1'b1;
                status_tag_reg <= rx_pipe_status_tag_reg;
                status_qid_reg <= rx_pipe_status_qid_reg;
                status_cidx_after_reg <= rx_pipe_status_cidx_after_reg;
                status_bytes_reg <= rx_pipe_status_bytes_reg;
                status_error_code_reg <= rx_pipe_status_error_code_reg;
                status_swtag_reg <= rx_pipe_status_swtag_reg;
                status_user_reg <= rx_pipe_status_user_reg;
                status_wb_base_reg <= rx_pipe_status_wb_base_reg;
                status_irq_enable_reg <= rx_pipe_status_irq_enable_reg;
                active_desc_reg <= 1'b0;
                rx_closing_reg <= 1'b0;
            end
        end

        if (stream_fire) begin
            if (rx_have_desc) begin
                rx_pipe_valid_reg <= 1'b1;
                rx_pipe_tdata_reg <= s_axis_st_rx_tdata;
                rx_pipe_bytes_reg <= rx_accept_bytes;
                rx_pipe_addr_reg <= active_host_addr_reg + {{(PCIE_ADDR_WIDTH-LEN_WIDTH){1'b0}}, active_offset_reg};
                rx_pipe_last_reg <= s_axis_st_rx_tlast;
                rx_pipe_overflow_reg <= rx_accept_overflow;
                rx_pipe_status_tag_reg <= active_tag_reg;
                rx_pipe_status_qid_reg <= active_qid_reg;
                rx_pipe_status_cidx_after_reg <= active_cidx_after_reg;
                rx_pipe_status_bytes_reg <= rx_accept_status_bytes;
                rx_pipe_status_error_code_reg <= rx_accept_status_error;
                rx_pipe_status_swtag_reg <= active_swtag_reg;
                rx_pipe_status_user_reg <= rx_accept_status_user;
                rx_pipe_status_wb_base_reg <= active_wb_base_reg;
                rx_pipe_status_irq_enable_reg <= active_irq_enable_reg;

                active_offset_reg <= rx_accept_next_offset;

                if (!in_packet_reg) begin
                    in_packet_reg        <= !s_axis_st_rx_tlast;
                    packet_trunc_reg     <= rx_accept_overflow;
                end else if (rx_accept_overflow) begin
                    packet_trunc_reg <= 1'b1;
                end

                if (in_packet_reg && s_axis_st_rx_tlast) begin
                    in_packet_reg <= 1'b0;
                end

                if (s_axis_st_rx_tlast) begin
                    rx_closing_reg <= 1'b1;
                    packet_meta_valid_reg <= 1'b0;
                    packet_len_over_desc_reg <= 1'b0;
                end
            end else begin
                /* No active descriptor for this qid: accept/drop the packet.
                 * tready remains a pure pipeline/fifo backpressure signal,
                 * not a qid-availability signal.
                 */
                rx_pipe_valid_reg <= 1'b0;
                if (!in_packet_reg && !rx_drop_packet_reg) begin
                    rx_drop_packet_reg <= !s_axis_st_rx_tlast;
                    packet_user_reg <= s_axis_st_rx_tuser;
                    packet_len_reg <= s_axis_st_rx_tlen;
                    packet_guard_len_reg <= {LEN_WIDTH{1'b0}};
                    packet_meta_valid_reg <= 1'b0;
                    packet_len_over_desc_reg <= 1'b0;
                end
                if (s_axis_st_rx_tlast) begin
                    rx_drop_packet_reg <= 1'b0;
                    stat_drop_packets_reg <= stat_drop_packets_reg + 1'b1;
                end
            end
        end else if (rx_pipe_process) begin
            rx_pipe_valid_reg <= 1'b0;
        end

        if (rq_tvalid_reg && m_axis_rq_tready) begin
            rq_tvalid_reg <= 1'b0;
        end

        if (rq_ready_for_issue) begin
            if (tx_issue_valid) begin
                rq_tdata_reg <= tx_issue_tdata;
                rq_tkeep_reg <= tx_issue_tkeep;
                rq_tvalid_reg <= 1'b1;
                rq_tlast_reg <= tx_issue_tlast;
                rq_tuser_reg <= tx_issue_tuser;

                fifo_rd_ptr_reg <= tx_issue_rd_ptr_next;
                pop_fifo_count = tx_issue_pop_count != 0 ? 2'd1 : 2'd0;
                consume_payload_bytes = tx_issue_consume_bytes;
                tx_extra_pop_pending_reg <= tx_issue_extra_pop;
                tx_payload_rem_reg <= tx_payload_rem_next;
                tx_head_offset_reg <= tx_head_offset_next;
                tx_state_reg <= tx_state_next;

                /* Consume the current plan on its header beat.  Unless this
                 * request drains the completed packet, enqueue the sequential
                 * byte address for the next plan immediately.  MPS-sized TLPs
                 * have enough DATA cycles to finish the limit stage before the
                 * next header is needed. */
                if (tx_state_reg == TX_STATE_IDLE) begin
                    tx_launch_valid_reg <= 1'b0;
                    if (tx_launch_is_tail_reg) begin
                        tx_plan_pending_valid_reg <= 1'b0;
                    end else begin
                        tx_plan_pending_addr_reg <=
                            tx_current_addr +
                            {{(PCIE_ADDR_WIDTH-11){1'b0}}, tx_start_payload_len};
                        tx_plan_pending_valid_reg <= 1'b1;
                    end
                end
            end else begin
                rq_tdata_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
                rq_tkeep_reg <= {AXIS_PCIE_KEEP_WIDTH{1'b0}};
                rq_tlast_reg <= 1'b0;
                rq_tuser_reg <= {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                tx_state_reg <= tx_state_next;
            end
        end

        fifo_count_reg <= fifo_count_reg +
                          (push_fifo ? {{(FIFO_COUNT_WIDTH-1){1'b0}}, 1'b1} : {FIFO_COUNT_WIDTH{1'b0}}) -
                          ({{(FIFO_COUNT_WIDTH-2){1'b0}}, pop_fifo_count} +
                           {{(FIFO_COUNT_WIDTH-1){1'b0}}, tx_extra_pop_apply});
        fifo_byte_count_reg <= fifo_byte_count_reg +
                               (push_fifo ? {8'd0, rx_pipe_bytes_reg} : 16'd0) -
                               {5'd0, consume_payload_bytes};

        /* Registered FIFO head window.  A normal 128-byte TLP shifts this
         * window once per DATA beat exactly as before.  Boundary fragments may
         * leave the pointer in place (only tx_head_offset_reg advances), while a
         * final short beat can retire two entries. */
        if (tx_window_update && tx_issue_pop_count == 2'd1) begin
            fifo_cur_data_reg <= fifo_next_data_reg;
            fifo_cur_addr_reg <= fifo_addr[fifo_rd_ptr_next];
            fifo_cur_bytes_reg <= fifo_next_bytes_reg;
            fifo_next_data_reg <= fifo_data[fifo_rd_ptr_plus2];
            fifo_next_bytes_reg <= fifo_bytes[fifo_rd_ptr_plus2];

            if (push_fifo && fifo_wr_ptr_reg == fifo_rd_ptr_next) begin
                fifo_cur_data_reg <= rx_pipe_tdata_reg;
                fifo_cur_addr_reg <= rx_pipe_addr_reg;
                fifo_cur_bytes_reg <= rx_pipe_bytes_reg;
            end
            if (push_fifo && fifo_wr_ptr_reg == fifo_rd_ptr_plus2) begin
                fifo_next_data_reg <= rx_pipe_tdata_reg;
                fifo_next_bytes_reg <= rx_pipe_bytes_reg;
            end
        end else if (tx_window_update && tx_issue_pop_count == 2'd2) begin
            fifo_cur_data_reg <= fifo_data[fifo_rd_ptr_plus2];
            fifo_cur_addr_reg <= fifo_addr[fifo_rd_ptr_plus2];
            fifo_cur_bytes_reg <= fifo_bytes[fifo_rd_ptr_plus2];
            fifo_next_data_reg <= fifo_data[fifo_rd_ptr_plus3];
            fifo_next_bytes_reg <= fifo_bytes[fifo_rd_ptr_plus3];

            if (push_fifo && fifo_wr_ptr_reg == fifo_rd_ptr_plus2) begin
                fifo_cur_data_reg <= rx_pipe_tdata_reg;
                fifo_cur_addr_reg <= rx_pipe_addr_reg;
                fifo_cur_bytes_reg <= rx_pipe_bytes_reg;
            end
            if (push_fifo && fifo_wr_ptr_reg == fifo_rd_ptr_plus3) begin
                fifo_next_data_reg <= rx_pipe_tdata_reg;
                fifo_next_bytes_reg <= rx_pipe_bytes_reg;
            end
        end else begin
            fifo_cur_data_reg <= fifo_data[fifo_rd_ptr_reg];
            fifo_next_data_reg <= fifo_data[fifo_rd_ptr_next];
            fifo_cur_addr_reg <= fifo_addr[fifo_rd_ptr_reg];
            fifo_cur_bytes_reg <= fifo_bytes[fifo_rd_ptr_reg];
            fifo_next_bytes_reg <= fifo_bytes[fifo_rd_ptr_next];

            if (push_fifo && fifo_wr_ptr_reg == fifo_rd_ptr_reg) begin
                fifo_cur_data_reg <= rx_pipe_tdata_reg;
                fifo_cur_addr_reg <= rx_pipe_addr_reg;
                fifo_cur_bytes_reg <= rx_pipe_bytes_reg;
            end
            if (push_fifo && fifo_wr_ptr_reg == fifo_rd_ptr_next) begin
                fifo_next_data_reg <= rx_pipe_tdata_reg;
                fifo_next_bytes_reg <= rx_pipe_bytes_reg;
            end
        end

        if (pending_status_reg && fifo_bytes_empty && tx_state_reg == TX_STATE_IDLE &&
            !rq_tvalid_reg && !status_valid_reg && !rx_pipe_valid_reg) begin
            status_valid_reg <= 1'b1;
            pending_status_reg <= 1'b0;
            tx_plan_valid_reg <= 1'b0;
            tx_plan_pending_valid_reg <= 1'b0;
            tx_launch_valid_reg <= 1'b0;
            stat_rx_packets_reg <= stat_rx_packets_reg + 1;
            stat_rx_bytes_reg <= stat_rx_bytes_reg + status_bytes_reg;
            if (status_error_code_reg != 0)
                stat_trunc_packets_reg <= stat_trunc_packets_reg + 1;
        end

        if (status_valid_reg && m_axis_status_ready) begin
            status_valid_reg <= 1'b0;
        end

        if (rst) begin
            fifo_wr_ptr_reg <= 0;
            fifo_rd_ptr_reg <= 0;
            fifo_count_reg <= 0;
            fifo_byte_count_reg <= 0;
            fifo_cur_data_reg <= 0;
            fifo_next_data_reg <= 0;
            fifo_cur_addr_reg <= 0;
            fifo_cur_bytes_reg <= 0;
            fifo_next_bytes_reg <= 0;
            desc_in_valid_reg <= 1'b0;
            desc_in_host_addr_reg <= {PCIE_ADDR_WIDTH{1'b0}};
            desc_in_len_reg <= {LEN_WIDTH{1'b0}};
            desc_in_tag_reg <= {TAG_WIDTH{1'b0}};
            desc_in_qid_reg <= {QID_WIDTH{1'b0}};
            desc_in_cidx_after_reg <= {RING_INDEX_WIDTH{1'b0}};
            desc_in_swtag_reg <= 16'd0;
            desc_in_user_reg <= 32'd0;
            desc_in_wb_base_reg <= {PCIE_ADDR_WIDTH{1'b0}};
            desc_in_irq_enable_reg <= 32'd0;
            active_desc_reg <= 1'b0;
            active_host_addr_reg <= 0;
            active_len_reg <= 0;
            active_offset_reg <= 0;
            active_tag_reg <= 0;
            active_qid_reg <= 0;
            active_cidx_after_reg <= 0;
            active_swtag_reg <= 0;
            active_user_reg <= 0;
            active_wb_base_reg <= 0;
            active_irq_enable_reg <= 0;
            packet_user_reg <= 0;
            packet_len_reg <= {LEN_WIDTH{1'b0}};
            packet_guard_len_reg <= {LEN_WIDTH{1'b0}};
            packet_meta_valid_reg <= 1'b0;
            packet_len_over_desc_reg <= 1'b0;
            in_packet_reg <= 1'b0;
            packet_trunc_reg <= 1'b0;
            rx_closing_reg <= 1'b0;
            rx_drop_packet_reg <= 1'b0;
            desc_state_reg <= DESC_STATE_CLEAR;
            desc_op_qid_reg <= {QID_WIDTH{1'b0}};
            desc_op_ptr_reg <= {DESC_WINDOW_PTR_WIDTH{1'b0}};
            desc_op_count_reg <= {DESC_WINDOW_COUNT_WIDTH{1'b0}};
            desc_clear_qid_reg <= {QID_WIDTH{1'b0}};
            desc_pool_rd_addr_reg <= {DESC_POOL_ADDR_WIDTH{1'b0}};
            desc_pool_rd_data_reg <= {DESC_POOL_ENTRY_WIDTH{1'b0}};
            rx_lookup_qid_reg <= {QID_WIDTH{1'b0}};
            rx_lookup_pending_reg <= 1'b0;
            rx_lookup_done_reg <= 1'b0;
            rx_lookup_hit_reg <= 1'b0;
            rx_pipe_valid_reg <= 1'b0;
            rx_pipe_tdata_reg <= 0;
            rx_pipe_bytes_reg <= 0;
            rx_pipe_addr_reg <= 0;
            rx_pipe_last_reg <= 1'b0;
            rx_pipe_overflow_reg <= 1'b0;
            rx_pipe_status_tag_reg <= 0;
            rx_pipe_status_qid_reg <= 0;
            rx_pipe_status_cidx_after_reg <= 0;
            rx_pipe_status_bytes_reg <= 0;
            rx_pipe_status_error_code_reg <= 0;
            rx_pipe_status_swtag_reg <= 0;
            rx_pipe_status_user_reg <= 0;
            rx_pipe_status_wb_base_reg <= 0;
            rx_pipe_status_irq_enable_reg <= 0;
            pending_status_reg <= 1'b0;
            status_tag_reg <= 0;
            status_qid_reg <= 0;
            status_cidx_after_reg <= 0;
            status_bytes_reg <= 0;
            status_error_code_reg <= 0;
            status_swtag_reg <= 0;
            status_user_reg <= 0;
            status_wb_base_reg <= 0;
            status_irq_enable_reg <= 0;
            status_valid_reg <= 1'b0;
            tx_state_reg <= TX_STATE_IDLE;
            tx_payload_rem_reg <= 0;
            tx_head_offset_reg <= 0;
            tx_extra_pop_pending_reg <= 1'b0;
            cfg_mps_bytes_reg <= 11'd128;
            tx_plan_addr_reg <= 0;
            tx_plan_limit_reg <= 13'd0;
            tx_plan_valid_reg <= 1'b0;
            tx_plan_pending_addr_reg <= 0;
            tx_plan_pending_valid_reg <= 1'b0;
            tx_launch_addr_reg <= 0;
            tx_launch_payload_len_reg <= 11'd0;
            tx_launch_is_tail_reg <= 1'b0;
            tx_launch_valid_reg <= 1'b0;
            rq_tdata_reg <= 0;
            rq_tkeep_reg <= 0;
            rq_tvalid_reg <= 1'b0;
            rq_tlast_reg <= 1'b0;
            rq_tuser_reg <= 0;
            stat_rx_packets_reg <= 0;
            stat_rx_bytes_reg <= 0;
            stat_trunc_packets_reg <= 0;
            stat_drop_packets_reg <= 0;
        end
    end

endmodule

`resetall
