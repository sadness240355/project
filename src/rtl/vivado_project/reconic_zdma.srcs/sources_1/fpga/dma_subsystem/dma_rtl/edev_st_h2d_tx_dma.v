/* SPDX-License-Identifier: MIT
 * ST H2D/TX DMA core with native ST descriptor interface.
 *
 * The former MM-compatible wrapper has been removed from this source path.
 * edev_sgdma_scheduler now emits native ST descriptors and receives full ST
 * completion metadata, so this core is instantiated directly by fpga_core.
 */
`resetall
`timescale 1ns / 1ps
`default_nettype none

`include "edev_sgdma_defs.vh"

module edev_st_h2d_tx_dma_core #(
    parameter AXIS_PCIE_DATA_WIDTH    = 256,
    parameter AXIS_PCIE_KEEP_WIDTH    = AXIS_PCIE_DATA_WIDTH/32,
    parameter AXIS_PCIE_RC_USER_WIDTH = 75,
    parameter AXIS_PCIE_RQ_USER_WIDTH = 62,
    parameter AXIS_ST_MTY_WIDTH       = 6,
    parameter PCIE_ADDR_WIDTH         = 64,
    parameter LEN_WIDTH               = 24,
    parameter TAG_WIDTH               = 12,
    parameter QID_WIDTH               = 9,
    parameter RING_INDEX_WIDTH        = 16,
    parameter FIFO_ADDR_WIDTH         = 9,
    parameter ST_TX_PCIE_TAG          = 8'h80,
    parameter MAX_READ_REQ_BYTES      = 256,
    parameter OUTSTANDING_READS       = 8
) (
    input  wire                             clk,
    input  wire                             rst,

    input  wire                             enable,
    input  wire [15:0]                      requester_id,
    input  wire                             requester_id_enable,
    input  wire [2:0]                       max_read_request_size,

    input  wire [PCIE_ADDR_WIDTH-1:0]       s_axis_desc_host_addr,
    input  wire [LEN_WIDTH-1:0]             s_axis_desc_len,
    input  wire [TAG_WIDTH-1:0]             s_axis_desc_tag,
    input  wire [QID_WIDTH-1:0]             s_axis_desc_qid,
    input  wire [RING_INDEX_WIDTH-1:0]      s_axis_desc_cidx_after,
    input  wire [15:0]                      s_axis_desc_flags,
    input  wire [15:0]                      s_axis_desc_swtag,
    input  wire [31:0]                      s_axis_desc_user,
    input  wire                             s_axis_desc_valid,
    output wire                             s_axis_desc_ready,

    output wire [AXIS_PCIE_DATA_WIDTH-1:0]  m_axis_rq_tdata,
    output wire [AXIS_PCIE_KEEP_WIDTH-1:0]  m_axis_rq_tkeep,
    output wire                             m_axis_rq_tvalid,
    input  wire                             m_axis_rq_tready,
    output wire                             m_axis_rq_tlast,
    output wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] m_axis_rq_tuser,

    input  wire [AXIS_PCIE_DATA_WIDTH-1:0]  s_axis_rc_tdata,
    input  wire [AXIS_PCIE_KEEP_WIDTH-1:0]  s_axis_rc_tkeep,
    input  wire                             s_axis_rc_tvalid,
    output wire                             s_axis_rc_tready,
    input  wire                             s_axis_rc_tlast,
    input  wire [AXIS_PCIE_RC_USER_WIDTH-1:0] s_axis_rc_tuser,

    output wire [AXIS_PCIE_DATA_WIDTH-1:0]  m_axis_st_tx_tdata,
    output wire [AXIS_ST_MTY_WIDTH-1:0]     m_axis_st_tx_mty,
    output wire                             m_axis_st_tx_tvalid,
    input  wire                             m_axis_st_tx_tready,
    output wire                             m_axis_st_tx_tlast,
    output wire [31:0]                      m_axis_st_tx_tuser,
    output wire [QID_WIDTH-1:0]             m_axis_st_tx_tqid,
    output wire [LEN_WIDTH-1:0]             m_axis_st_tx_tlen,

    output wire [TAG_WIDTH-1:0]             m_axis_status_tag,
    output wire [QID_WIDTH-1:0]             m_axis_status_qid,
    output wire [RING_INDEX_WIDTH-1:0]      m_axis_status_cidx_after,
    output wire [31:0]                      m_axis_status_bytes,
    output wire [31:0]                      m_axis_status_error_code,
    output wire [15:0]                      m_axis_status_swtag,
    output wire [31:0]                      m_axis_status_user,
    output wire                             m_axis_status_valid,
    input  wire                             m_axis_status_ready,

    output wire                             status_busy,
    output wire [31:0]                      stat_tx_packets,
    output wire [31:0]                      stat_tx_bytes,
    output wire [31:0]                      stat_tx_rc_errors,
    output wire [31:0]                      stat_tx_desc_errors
);

    localparam AXIS_BYTE_WIDTH = AXIS_PCIE_DATA_WIDTH/8;
    localparam FIFO_DEPTH = 1 << FIFO_ADDR_WIDTH;
    localparam FIFO_COUNT_WIDTH = FIFO_ADDR_WIDTH+1;
    localparam [FIFO_COUNT_WIDTH-1:0] FIFO_DEPTH_COUNT = FIFO_DEPTH;
    /* One ST descriptor is one Ethernet frame.  PCIe reads may still be split at
     * 4KB address boundaries, but those internal read boundaries must never create
     * extra AXI-Stream TLAST events.
     */
    localparam [12:0] PCIE_4K_BYTES = 13'd4096;
    localparam [LEN_WIDTH-1:0] ST_TX_MAX_PACKET_BYTES = 9600;
    localparam ST_TX_FIFO_BYTES = FIFO_DEPTH * AXIS_BYTE_WIDTH;
    localparam [12:0] MAX_READ_REQ_BYTES_CONST = MAX_READ_REQ_BYTES;
    localparam [3:0] REQ_MEM_READ = 4'b0000;

    localparam OUTSTANDING_INDEX_WIDTH = OUTSTANDING_READS <= 2 ? 1 : $clog2(OUTSTANDING_READS);
    localparam OUTSTANDING_COUNT_WIDTH = OUTSTANDING_INDEX_WIDTH+1;
    localparam [OUTSTANDING_COUNT_WIDTH-1:0] OUTSTANDING_READS_COUNT = OUTSTANDING_READS;

    localparam [2:0]
        ISSUE_IDLE   = 3'd0,
        ISSUE_CALC0  = 3'd1,
        ISSUE_CALC1  = 3'd2,
        ISSUE_CALC2  = 3'd3,
        ISSUE_BUILD  = 3'd4,
        ISSUE_SEND   = 3'd5,
        ISSUE_UPDATE = 3'd6;

    localparam [0:0]
        TX_OUT_IDLE = 1'b0,
        TX_OUT_SEND = 1'b1;

    integer i;

    initial begin
        if (AXIS_PCIE_DATA_WIDTH != 256) begin
            $error("edev_st_h2d_tx_dma currently targets the 256-bit non-straddled PG213 interface used by this project (instance %m)");
            $finish;
        end
        if (AXIS_ST_MTY_WIDTH != 6) begin
            $error("AXIS_ST_MTY_WIDTH must be 6 to match the QDMA mty convention (instance %m)");
            $finish;
        end
        if (MAX_READ_REQ_BYTES < 32 || MAX_READ_REQ_BYTES > 1024) begin
            $error("MAX_READ_REQ_BYTES must be between 32 and 1024 (instance %m)");
            $finish;
        end
        if (ST_TX_FIFO_BYTES < ST_TX_MAX_PACKET_BYTES) begin
            $error("ST TX packet FIFO must hold one complete 9600-byte Ethernet frame (instance %m)");
            $finish;
        end
        if (OUTSTANDING_READS != 2 && OUTSTANDING_READS != 4 && OUTSTANDING_READS != 8 && OUTSTANDING_READS != 16) begin
            $error("OUTSTANDING_READS must be one of 2, 4, 8 or 16 so the ST tag window remains contiguous (instance %m)");
            $finish;
        end
        if (OUTSTANDING_READS > 16) begin
            $error("OUTSTANDING_READS cannot exceed the 16-tag ST_TX demux window (instance %m)");
            $finish;
        end
        if (ST_TX_PCIE_TAG[3:0] != 4'd0) begin
            $error("ST_TX_PCIE_TAG low nibble must be zero because low bits encode the outstanding slot (instance %m)");
            $finish;
        end
    end

    /* One-entry registered descriptor intake.  The scheduler can hand off one
     * ST_TX descriptor without waiting for the PCIe read/RC/stream packer to
     * become fully idle.  This removes a wide idle-condition cone from the
     * scheduler ready path while keeping the core itself strictly one packet
     * at a time.
     */
    reg                                desc_in_valid_reg = 1'b0;
    reg  [        PCIE_ADDR_WIDTH-1:0] desc_in_host_addr_reg = {PCIE_ADDR_WIDTH{1'b0}};
    reg  [              LEN_WIDTH-1:0] desc_in_len_reg = {LEN_WIDTH{1'b0}};
    reg  [              TAG_WIDTH-1:0] desc_in_tag_reg = {TAG_WIDTH{1'b0}};
    reg  [              QID_WIDTH-1:0] desc_in_qid_reg = {QID_WIDTH{1'b0}};
    reg  [       RING_INDEX_WIDTH-1:0] desc_in_cidx_after_reg = {RING_INDEX_WIDTH{1'b0}};
    reg  [                       15:0] desc_in_flags_reg = 16'd0;
    reg  [                       15:0] desc_in_swtag_reg = 16'd0;
    reg  [                       31:0] desc_in_user_reg = 32'd0;

    reg                                desc_active_reg = 1'b0;
    reg  [        PCIE_ADDR_WIDTH-1:0] desc_host_addr_reg = {PCIE_ADDR_WIDTH{1'b0}};
    reg  [              LEN_WIDTH-1:0] desc_len_reg = {LEN_WIDTH{1'b0}};
    reg  [              LEN_WIDTH-1:0] issued_bytes_reg = {LEN_WIDTH{1'b0}};
    reg  [                       31:0] output_bytes_reg = 32'd0;
    reg  [              TAG_WIDTH-1:0] desc_tag_reg = {TAG_WIDTH{1'b0}};
    reg  [              QID_WIDTH-1:0] desc_qid_reg = {QID_WIDTH{1'b0}};
    reg  [       RING_INDEX_WIDTH-1:0] desc_cidx_after_reg = {RING_INDEX_WIDTH{1'b0}};
    reg  [                       15:0] desc_flags_reg = 16'd0;
    reg  [                       15:0] desc_swtag_reg = 16'd0;
    reg  [                       31:0] desc_user_reg = 32'd0;
    reg  [                       31:0] desc_error_code_reg = 32'd0;

    /* FIFO-side packet state.  Data is fetched for the whole descriptor while
     * PCIe read requests independently obey MRRS and 4KB boundaries.
     * The packet length sideband stored with every FIFO beat is the complete
     * descriptor/Ethernet-frame length.
     */
    reg  [              LEN_WIDTH-1:0] pack_desc_rem_reg = {LEN_WIDTH{1'b0}};
    reg  [                       13:0] pack_packet_len_reg = 14'd0;
    reg  [                       13:0] pack_packet_rem_reg = 14'd0;

    reg [2:0] issue_state_reg = ISSUE_IDLE;
    reg [12:0] issue_mrrs_bytes_reg = 13'd0;
    reg [12:0] issue_mrrs_payload_limit_reg = 13'd0;
    reg [PCIE_ADDR_WIDTH-1:0] issue_addr_reg = {PCIE_ADDR_WIDTH{1'b0}};
    reg [12:0] issue_remain_bytes_reg = 13'd0;
    reg [12:0] issue_boundary_bytes_reg = 13'd0;
    reg [12:0] issue_chunk_len_reg = 13'd0;
    reg [LEN_WIDTH-1:0] issue_chunk_len_len_reg = {LEN_WIDTH{1'b0}};
    reg [10:0] issue_dw_count_reg = 11'd0;
    reg [3:0] issue_first_be_reg = 4'd0;
    reg [3:0] issue_last_be_reg = 4'd0;
    reg issue_last_chunk_reg = 1'b0;
    reg [OUTSTANDING_INDEX_WIDTH-1:0] issue_slot_reg = {OUTSTANDING_INDEX_WIDTH{1'b0}};

    reg                                rq_valid_reg = 1'b0;
    reg  [   AXIS_PCIE_DATA_WIDTH-1:0] rq_tdata_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg  [   AXIS_PCIE_KEEP_WIDTH-1:0] rq_tkeep_reg = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
    reg                                rq_tlast_reg = 1'b0;
    reg  [AXIS_PCIE_RQ_USER_WIDTH-1:0] rq_tuser_reg = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};

    reg [OUTSTANDING_INDEX_WIDTH-1:0] issue_ptr_reg = {OUTSTANDING_INDEX_WIDTH{1'b0}};
    reg [OUTSTANDING_INDEX_WIDTH-1:0] rc_ptr_reg = {OUTSTANDING_INDEX_WIDTH{1'b0}};
    reg [OUTSTANDING_COUNT_WIDTH-1:0] outstanding_count_reg = {OUTSTANDING_COUNT_WIDTH{1'b0}};

    /* Predecoded outstanding-count state.
     *
     * outstanding_count_reg is intentionally kept as the architectural counter,
     * but it no longer fans out directly into the RC ready path, descriptor
     * start/idle cone, and request-window control.  These one-bit mirrors are
     * updated from the same small increment/decrement result as the counter.
     * This turns the former multi-bit compare fanout into local flag loads while
     * preserving cycle accuracy of the issue/RC bypass behavior.
     */
    reg outstanding_empty_reg = 1'b1;
    reg outstanding_one_reg = 1'b0;
    reg outstanding_gt_two_reg = 1'b0;
    reg outstanding_full_reg = 1'b0;

    /* In-order request metadata FIFO.  No payload is buffered per tag. */
    reg [12:0] req_len_reg [0:OUTSTANDING_READS-1];
    reg [OUTSTANDING_READS-1:0] req_last_chunk_reg = {OUTSTANDING_READS{1'b0}};

    reg rc_frame_active_reg = 1'b0;
    reg rc_req_active_reg = 1'b0;
    reg [12:0] rc_req_len_reg = 13'd0;
    reg [12:0] rc_req_rcvd_bytes_reg = 13'd0;
    reg rc_req_last_chunk_reg = 1'b0;
    reg [7:0] rc_req_tag_reg = 8'd0;
    reg rc_next_req_valid_reg = 1'b0;
    reg [12:0] rc_next_req_len_reg = 13'd0;
    reg rc_next_req_last_chunk_reg = 1'b0;
    reg [7:0] rc_next_req_tag_reg = 8'd0;
    reg rc_drop_frame_reg = 1'b0;

    /* RC timing pipe.
     * Stage 0 accepts the PCIe RC beat and resolves payload byte count from
     * registered current-request metadata.  A second registered metadata slot is
     * kept for the next in-order request, so the no-bubble boundary bypass never
     * reads req_len_reg with rc_ptr_reg in the same cycle as rc_pipe_* capture.
     * This cuts both the old RC tkeep -> registered RC byte count path and the later
     * rc_ptr_reg -> rc_pipe_req_done_reg path while keeping one RC beat/cycle
     * throughput across adjacent completion TLPs.
     */
    reg rc_meta_valid_reg = 1'b0;
    reg [12:0] rc_meta_rcvd_after_reg = 13'd0;
    reg rc_meta_req_done_reg = 1'b0;

    reg rc_pipe_valid_reg = 1'b0;
    reg rc_pipe_payload_valid_reg = 1'b0;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] rc_pipe_payload_data_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg [5:0] rc_pipe_payload_count_reg = 6'd0;
    reg rc_pipe_req_done_reg = 1'b0;
    reg rc_pipe_last_chunk_reg = 1'b0;
    reg rc_pipe_header_error_reg = 1'b0;
    reg rc_pipe_discontinue_reg = 1'b0;
    reg [31:0] rc_pipe_error_code_reg = 32'd0;

    reg [AXIS_PCIE_DATA_WIDTH-1:0] stream_pack_data_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg [5:0] stream_pack_count_reg = 6'd0;
    reg tail_flush_valid_reg = 1'b0;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] tail_flush_data_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg [5:0] tail_flush_bytes_reg = 6'd0;

    reg                                status_valid_reg = 1'b0;
    reg  [                       31:0] status_bytes_reg = 32'd0;
    reg  [                       31:0] status_error_code_reg = 32'd0;
    reg  [                       31:0] status_user_reg = 32'd0;

    reg  [                       31:0] stat_tx_packets_reg = 32'd0;
    reg  [                       31:0] stat_tx_bytes_reg = 32'd0;
    reg  [                       31:0] stat_tx_rc_errors_reg = 32'd0;
    reg  [                       31:0] stat_tx_desc_errors_reg = 32'd0;

    (* ram_style="block" *) reg [AXIS_PCIE_DATA_WIDTH-1:0] fifo_tdata [0:FIFO_DEPTH-1];
    (* ram_style="distributed" *) reg fifo_tlast [0:FIFO_DEPTH-1];
    (* ram_style="distributed" *) reg [31:0] fifo_tuser [0:FIFO_DEPTH-1];
    (* ram_style="distributed" *) reg [5:0] fifo_tbytes [0:FIFO_DEPTH-1];
    (* ram_style="distributed" *) reg [AXIS_ST_MTY_WIDTH-1:0] fifo_tmty [0:FIFO_DEPTH-1];
    (* ram_style="distributed" *) reg [13:0] fifo_tlen [0:FIFO_DEPTH-1];
    reg [FIFO_ADDR_WIDTH-1:0] fifo_wr_ptr_reg = {FIFO_ADDR_WIDTH{1'b0}};
    reg [FIFO_ADDR_WIDTH-1:0] fifo_rd_ptr_reg = {FIFO_ADDR_WIDTH{1'b0}};
    reg [FIFO_COUNT_WIDTH-1:0] fifo_count_reg = {FIFO_COUNT_WIDTH{1'b0}};

    /* Registered write skid stage in front of the output FIFO.
     *
     * The RC packer/tail-flush logic is intentionally kept out of the
     * fifo_count_reg input cone.  fifo_count_reg is now driven only by the
     * registered stage-valid bit and the registered output-pop condition,
     * while the wide payload packing logic terminates at this one-beat stage.
     * When the stage writes to the FIFO, it can accept the next packed beat in
     * the same clock, so steady-state one-beat-per-cycle throughput is kept.
     */
    reg [AXIS_PCIE_DATA_WIDTH-1:0] fifo_stage_tdata_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg fifo_stage_tlast_reg = 1'b0;
    reg [31:0] fifo_stage_tuser_reg = 32'd0;
    reg [5:0] fifo_stage_tbytes_reg = 6'd0;
    reg [AXIS_ST_MTY_WIDTH-1:0] fifo_stage_tmty_reg = {AXIS_ST_MTY_WIDTH{1'b0}};
    reg [13:0] fifo_stage_tlen_reg = 14'd0;
    reg fifo_stage_valid_reg = 1'b0;

    /* Packet-forwarding gate.  Small frames wait for TLAST; large frames may
     * start after a 4 KiB prefix is buffered so PCIe reads overlap network
     * transmission.  Once started, the stream stays in the same Ethernet frame
     * until the descriptor-end TLAST; FIFO starvation only inserts legal AXIS
     * TVALID gaps and never creates an extra packet boundary. */
    reg tx_out_state_reg = TX_OUT_IDLE;
    reg [FIFO_COUNT_WIDTH-1:0] packet_ready_count_reg = {FIFO_COUNT_WIDTH{1'b0}};

    reg [AXIS_PCIE_DATA_WIDTH-1:0] axis_tdata_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg [AXIS_ST_MTY_WIDTH-1:0] axis_tmty_reg = {AXIS_ST_MTY_WIDTH{1'b0}};
    reg [5:0] axis_tbytes_reg = 6'd0;
    reg axis_tvalid_reg = 1'b0;
    reg axis_tlast_reg = 1'b0;
    reg [31:0] axis_tuser_reg = 32'd0;
    reg [LEN_WIDTH-1:0] axis_tlen_reg = {LEN_WIDTH{1'b0}};

    reg [AXIS_PCIE_DATA_WIDTH-1:0] fifo_stage_load_data;
    reg fifo_stage_load_last;
    reg [31:0] fifo_stage_load_user;
    reg [5:0] fifo_stage_load_bytes;
    reg [AXIS_ST_MTY_WIDTH-1:0] fifo_stage_load_mty;
    reg [13:0] fifo_stage_load_tlen;
    reg fifo_stage_load;
    reg status_should_complete;
    reg [31:0] output_bytes_this_beat;
    reg [31:0] output_bytes_after_fire;
    reg [LEN_WIDTH-1:0] pack_desc_rem_after_load;
    reg [12:0] cfg_mrrs_bytes_comb;
    reg [3:0] issue_first_be_comb;
    reg [3:0] issue_last_be_comb;
    reg [1:0] issue_last_byte_lsb_comb;
    reg [127:0] issue_rq_header_comb;
    reg [AXIS_PCIE_RQ_USER_WIDTH-1:0] issue_rq_tuser_comb;
    reg [31:0] pack_keep_mask;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] pack_byte_mask;
    reg [5:0] pack_mask_count;
    reg [6:0] fifo_stage_empty_bytes_comb;

    wire [LEN_WIDTH-1:0] issue_desc_remain = desc_len_reg - issued_bytes_reg;
    /* PCIe Length is expressed in DWORDs and the request address is rounded
     * down to a DWORD boundary.  Therefore an unaligned host address consumes
     * address[1:0] bytes of the MRRS window even though those leading bytes are
     * disabled by First BE.  Capping only the valid payload at MRRS can create
     * a 129-DW request for a 512-byte MRRS (for example, 512 valid bytes at
     * address +2), which is malformed and may never receive a completion.
     * Limit the first unaligned request so its complete DWORD span remains at
     * or below MRRS.  The following request is then DWORD aligned and retains
     * the full configured request size/throughput.
     */
    wire [12:0] issue_chunk_mrrs_min =
        (issue_remain_bytes_reg < issue_mrrs_payload_limit_reg) ?
        issue_remain_bytes_reg : issue_mrrs_payload_limit_reg;
    wire [12:0] issue_chunk_len_calc = (issue_chunk_mrrs_min < issue_boundary_bytes_reg) ? issue_chunk_mrrs_min : issue_boundary_bytes_reg;
    wire [12:0] issue_chunk_len_safe = issue_chunk_len_calc == 0 ? 13'd1 : issue_chunk_len_calc;
    wire [LEN_WIDTH-1:0] issue_chunk_len_safe_len = {{(LEN_WIDTH-13){1'b0}}, issue_chunk_len_safe};
    wire issue_window_available = !outstanding_full_reg;

    always @* begin
        case (max_read_request_size)
            3'd0: cfg_mrrs_bytes_comb = 13'd128;
            3'd1: cfg_mrrs_bytes_comb = 13'd256;
            3'd2: cfg_mrrs_bytes_comb = 13'd512;
            3'd3: cfg_mrrs_bytes_comb = 13'd1024;
            3'd4: cfg_mrrs_bytes_comb = 13'd2048;
            default: cfg_mrrs_bytes_comb = 13'd4096;
        endcase
    end

    always @* begin
        case (issue_addr_reg[1:0])
            2'd0: issue_first_be_comb = 4'b1111;
            2'd1: issue_first_be_comb = 4'b1110;
            2'd2: issue_first_be_comb = 4'b1100;
            default: issue_first_be_comb = 4'b1000;
        endcase

        issue_last_byte_lsb_comb = (issue_addr_reg[1:0] + issue_chunk_len_safe[1:0] - 1'b1) & 2'b11;
        case (issue_last_byte_lsb_comb)
            2'd0: issue_last_be_comb = 4'b0001;
            2'd1: issue_last_be_comb = 4'b0011;
            2'd2: issue_last_be_comb = 4'b0111;
            default: issue_last_be_comb = 4'b1111;
        endcase
    end

    wire [12:0] issue_mrrs_limited_comb = (cfg_mrrs_bytes_comb > MAX_READ_REQ_BYTES_CONST) ? MAX_READ_REQ_BYTES_CONST : cfg_mrrs_bytes_comb;
    wire [13:0] desc_in_packet_len_comb = desc_in_len_reg[13:0];
    wire [OUTSTANDING_INDEX_WIDTH-1:0] issue_ptr_next_wire =
        (issue_ptr_reg == OUTSTANDING_READS-1) ? {OUTSTANDING_INDEX_WIDTH{1'b0}} : (issue_ptr_reg + 1'b1);
    wire [OUTSTANDING_INDEX_WIDTH-1:0] rc_next_ptr =
        (rc_ptr_reg == OUTSTANDING_READS-1) ? {OUTSTANDING_INDEX_WIDTH{1'b0}} : (rc_ptr_reg + 1'b1);
    wire [OUTSTANDING_INDEX_WIDTH-1:0] rc_after_next_ptr =
        (rc_next_ptr == OUTSTANDING_READS-1) ? {OUTSTANDING_INDEX_WIDTH{1'b0}} : (rc_next_ptr + 1'b1);
    wire [7:0] issue_slot_tag_wire = ST_TX_PCIE_TAG[7:0] + {{(8-OUTSTANDING_INDEX_WIDTH){1'b0}}, issue_slot_reg};
    wire [7:0] rc_after_next_tag_wire = ST_TX_PCIE_TAG[7:0] + {{(8-OUTSTANDING_INDEX_WIDTH){1'b0}}, rc_after_next_ptr};

    always @* begin
        issue_rq_header_comb = 128'd0;
        issue_rq_header_comb[1:0] = 2'b00;
        issue_rq_header_comb[63:2] = issue_addr_reg[PCIE_ADDR_WIDTH-1:2];
        issue_rq_header_comb[74:64] = issue_dw_count_reg;
        issue_rq_header_comb[78:75] = REQ_MEM_READ;
        issue_rq_header_comb[79] = 1'b0;
        issue_rq_header_comb[95:80] = requester_id;
        issue_rq_header_comb[103:96] = issue_slot_tag_wire;
        issue_rq_header_comb[119:104] = 16'd0;
        issue_rq_header_comb[120] = requester_id_enable;
        issue_rq_header_comb[123:121] = 3'b000;
        issue_rq_header_comb[126:124] = 3'b000;
        issue_rq_header_comb[127] = 1'b0;

        issue_rq_tuser_comb = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
        if (issue_dw_count_reg == 1) begin
            issue_rq_tuser_comb[3:0] = issue_first_be_reg & issue_last_be_reg;
            issue_rq_tuser_comb[7:4] = 4'd0;
        end else begin
            issue_rq_tuser_comb[3:0] = issue_first_be_reg;
            issue_rq_tuser_comb[7:4] = issue_last_be_reg;
        end
        issue_rq_tuser_comb[10:8] = 3'd0;
        issue_rq_tuser_comb[11] = 1'b0;
        issue_rq_tuser_comb[12] = 1'b0;
        issue_rq_tuser_comb[14:13] = 2'b00;
        issue_rq_tuser_comb[15] = 1'b0;
        issue_rq_tuser_comb[23:16] = 8'd0;
        issue_rq_tuser_comb[27:24] = 4'd0;
        issue_rq_tuser_comb[59:28] = 32'd0;
        if (AXIS_PCIE_RQ_USER_WIDTH > 60) begin
            issue_rq_tuser_comb[61:60] = 2'd0;
        end
    end

    wire fifo_empty = fifo_count_reg == 0;
    wire [FIFO_COUNT_WIDTH-1:0] fifo_space = FIFO_DEPTH_COUNT - fifo_count_reg;
    wire fifo_push_possible = fifo_space != 0;
    wire axis_output_ready = !axis_tvalid_reg || m_axis_st_tx_tready;
    wire axis_output_fire = axis_tvalid_reg && m_axis_st_tx_tready;
    wire axis_output_last_fire = axis_output_fire && axis_tlast_reg;
    localparam integer TX_PREFETCH_BEATS = 4096 / AXIS_BYTE_WIDTH;
    wire tx_prefetch_ready = (packet_ready_count_reg != 0) ||
                             (fifo_count_reg >= TX_PREFETCH_BEATS);
    wire tx_packet_start = tx_out_state_reg == TX_OUT_IDLE && tx_prefetch_ready && !fifo_empty;
    wire tx_packet_continue = tx_out_state_reg == TX_OUT_SEND && (!axis_tvalid_reg || !axis_tlast_reg);
    wire tx_packet_next_start = axis_output_last_fire && packet_ready_count_reg > 1;
    wire tx_packet_active = tx_packet_start || tx_packet_continue || tx_packet_next_start;
    wire axis_load_fifo = axis_output_ready && tx_packet_active && !fifo_empty;
    wire fifo_stage_push = fifo_stage_valid_reg && fifo_push_possible;
    wire fifo_stage_push_last = fifo_stage_push && fifo_stage_tlast_reg;
    wire fifo_stage_ready = !fifo_stage_valid_reg || fifo_stage_push;

    wire issue_update_fire = issue_state_reg == ISSUE_UPDATE;
    wire rc_meta_done_pending = rc_meta_valid_reg && rc_meta_req_done_reg;
    wire rc_meta_done_fire = rc_meta_done_pending;
    wire rc_meta_same_req_pending = rc_meta_valid_reg && !rc_meta_req_done_reg;
    wire rc_meta_done_has_next = rc_meta_done_pending && rc_next_req_valid_reg;
    wire rc_meta_done_no_next = rc_meta_done_pending && !rc_next_req_valid_reg;
    wire [7:0] rc_expected_tag = rc_meta_done_has_next ? rc_next_req_tag_reg : rc_req_tag_reg;
    wire rc_input_first = !rc_frame_active_reg;
    wire rc_has_request = rc_meta_done_has_next || (rc_req_active_reg && !rc_meta_done_no_next);
    wire rc_pipe_drain = rc_pipe_valid_reg && fifo_stage_ready && !tail_flush_valid_reg;
    wire rc_pipe_ready = !rc_pipe_valid_reg || rc_pipe_drain;
    wire rc_can_store = !tail_flush_valid_reg && rc_pipe_ready;
    wire rc_input_fire = s_axis_rc_tvalid && s_axis_rc_tready;

    wire tx_core_idle = !desc_active_reg && !status_valid_reg && !axis_tvalid_reg &&
                        fifo_empty && !fifo_stage_valid_reg && packet_ready_count_reg == 0 && tx_out_state_reg == TX_OUT_IDLE &&
                        outstanding_empty_reg && issue_state_reg == ISSUE_IDLE && !rq_valid_reg &&
                        !rc_frame_active_reg && !rc_req_active_reg && !rc_next_req_valid_reg &&
                        !rc_meta_valid_reg && !rc_pipe_valid_reg &&
                        stream_pack_count_reg == 0 && !tail_flush_valid_reg;
    assign s_axis_desc_ready = enable && !desc_in_valid_reg;
    wire desc_fire = s_axis_desc_valid && s_axis_desc_ready;
    wire desc_start = enable && desc_in_valid_reg && tx_core_idle;

    assign m_axis_rq_tdata  = rq_tdata_reg;
    assign m_axis_rq_tkeep  = rq_tkeep_reg;
    assign m_axis_rq_tvalid = rq_valid_reg;
    assign m_axis_rq_tlast  = rq_tlast_reg;
    assign m_axis_rq_tuser  = rq_tuser_reg;

    assign s_axis_rc_tready = enable && desc_active_reg && rc_has_request && rc_can_store;

    assign m_axis_st_tx_tdata  = axis_tdata_reg;
    assign m_axis_st_tx_mty = axis_tmty_reg;
    assign m_axis_st_tx_tvalid = axis_tvalid_reg;
    assign m_axis_st_tx_tlast  = axis_tlast_reg;
    assign m_axis_st_tx_tuser  = axis_tuser_reg;
    assign m_axis_st_tx_tqid   = desc_qid_reg;
    assign m_axis_st_tx_tlen   = axis_tlen_reg;

    assign m_axis_status_tag        = desc_tag_reg;
    assign m_axis_status_qid        = desc_qid_reg;
    assign m_axis_status_cidx_after = desc_cidx_after_reg;
    assign m_axis_status_bytes      = status_bytes_reg;
    assign m_axis_status_error_code = status_error_code_reg;
    assign m_axis_status_swtag      = desc_swtag_reg;
    assign m_axis_status_user       = status_user_reg;
    assign m_axis_status_valid      = status_valid_reg;

    assign status_busy = desc_in_valid_reg || desc_active_reg || issue_state_reg != ISSUE_IDLE || rq_valid_reg || !outstanding_empty_reg ||
                         rc_frame_active_reg || rc_req_active_reg || rc_next_req_valid_reg ||
                         rc_meta_valid_reg || rc_pipe_valid_reg ||
                         stream_pack_count_reg != 0 || tail_flush_valid_reg || fifo_stage_valid_reg ||
                         packet_ready_count_reg != 0 || tx_out_state_reg != TX_OUT_IDLE ||
                         !fifo_empty || axis_tvalid_reg || status_valid_reg;
    assign stat_tx_packets = stat_tx_packets_reg;
    assign stat_tx_bytes = stat_tx_bytes_reg;
    assign stat_tx_rc_errors = stat_tx_rc_errors_reg;
    assign stat_tx_desc_errors = stat_tx_desc_errors_reg;

    reg [5:0] rc_valid_byte_count;
    reg [5:0] rc_payload_offset;
    reg [12:0] rc_active_len_comb;
    reg [12:0] rc_active_rcvd_comb;
    reg rc_active_last_chunk_comb;
    reg [12:0] rc_payload_capacity;
    reg [12:0] rc_payload_remaining;
    reg [5:0] rc_payload_count;
    reg [AXIS_PCIE_DATA_WIDTH-1:0] rc_payload_data;
    reg [12:0] rc_rcvd_after_beat;
    reg rc_tag_bad;
    reg rc_header_error;
    reg rc_discontinue;
    reg rc_payload_completes_req;
    reg rc_req_done_after_beat;

    reg [AXIS_PCIE_DATA_WIDTH*2-1:0] pack_combined_data;
    reg [6:0] pack_combined_count;
    reg [5:0] pack_leftover_count;
    reg pack_desc_last;

    always @* begin
        case (s_axis_rc_tkeep)
            8'h00: rc_valid_byte_count = 6'd0;
            8'h01: rc_valid_byte_count = 6'd4;
            8'h03: rc_valid_byte_count = 6'd8;
            8'h07: rc_valid_byte_count = 6'd12;
            8'h0f: rc_valid_byte_count = 6'd16;
            8'h1f: rc_valid_byte_count = 6'd20;
            8'h3f: rc_valid_byte_count = 6'd24;
            8'h7f: rc_valid_byte_count = 6'd28;
            default: rc_valid_byte_count = 6'd32;
        endcase
        rc_payload_offset = rc_input_first ? (6'd12 + {4'd0, s_axis_rc_tdata[1:0]}) : 6'd0;
        if (rc_meta_done_has_next) begin
            rc_active_len_comb = rc_next_req_len_reg;
            rc_active_rcvd_comb = 13'd0;
            rc_active_last_chunk_comb = rc_next_req_last_chunk_reg;
        end else begin
            rc_active_len_comb = rc_req_len_reg;
            rc_active_rcvd_comb = rc_meta_same_req_pending ? rc_meta_rcvd_after_reg : rc_req_rcvd_bytes_reg;
            rc_active_last_chunk_comb = rc_req_last_chunk_reg;
        end
        rc_payload_capacity = 13'd0;
        rc_payload_remaining = 13'd0;
        rc_payload_count = 6'd0;
        rc_payload_data = {AXIS_PCIE_DATA_WIDTH{1'b0}};
        rc_rcvd_after_beat = rc_active_rcvd_comb;
        rc_tag_bad = rc_input_first && (s_axis_rc_tdata[71:64] != rc_expected_tag);
        rc_header_error = rc_input_first && (rc_tag_bad || s_axis_rc_tdata[15:12] != 4'd0);
        rc_discontinue = (AXIS_PCIE_RC_USER_WIDTH > 42) ? s_axis_rc_tuser[42] : 1'b0;
        rc_payload_completes_req = 1'b0;
        rc_req_done_after_beat = 1'b0;

        if (rc_active_len_comb > rc_active_rcvd_comb) begin
            rc_payload_remaining = rc_active_len_comb - rc_active_rcvd_comb;
        end

        if (rc_valid_byte_count > rc_payload_offset) begin
            rc_payload_capacity = {7'd0, (rc_valid_byte_count - rc_payload_offset)};
        end

        if (!rc_header_error && !rc_drop_frame_reg && !rc_discontinue && rc_payload_remaining != 0 && rc_payload_capacity != 0) begin
            if (rc_payload_capacity > rc_payload_remaining) begin
                rc_payload_count = rc_payload_remaining[5:0];
            end else begin
                rc_payload_count = rc_payload_capacity[5:0];
            end
            rc_rcvd_after_beat = rc_active_rcvd_comb + {7'd0, rc_payload_count};
            case (rc_payload_offset)
                6'd0:  rc_payload_data = s_axis_rc_tdata;
                6'd12: rc_payload_data = {96'd0,  s_axis_rc_tdata[255:96]};
                6'd13: rc_payload_data = {104'd0, s_axis_rc_tdata[255:104]};
                6'd14: rc_payload_data = {112'd0, s_axis_rc_tdata[255:112]};
                6'd15: rc_payload_data = {120'd0, s_axis_rc_tdata[255:120]};
                default: rc_payload_data = {AXIS_PCIE_DATA_WIDTH{1'b0}};
            endcase
        end

        /* rc_rcvd_after_beat >= rc_active_len_comb is equivalent to the
         * accepted payload covering all bytes that remained before this beat.
         * Compare those two small values directly so the request-done flag no
         * longer sits behind the received-byte adder/carry chain. */
        rc_payload_completes_req = {7'd0, rc_payload_count} >= rc_payload_remaining;
        rc_req_done_after_beat = s_axis_rc_tlast &&
                                 (rc_payload_completes_req || rc_header_error || rc_discontinue);
    end

    always @(posedge clk) begin
        fifo_stage_load = 1'b0;
        fifo_stage_load_data = {AXIS_PCIE_DATA_WIDTH{1'b0}};
        fifo_stage_load_last = 1'b0;
        fifo_stage_load_user = desc_user_reg;
        fifo_stage_load_bytes = 6'd0;
        fifo_stage_load_mty = {AXIS_ST_MTY_WIDTH{1'b0}};
        fifo_stage_load_tlen = pack_packet_len_reg;
        fifo_stage_empty_bytes_comb = 7'd0;
        status_should_complete = 1'b0;
        output_bytes_this_beat = 32'd0;
        output_bytes_after_fire = output_bytes_reg;
        pack_desc_rem_after_load = pack_desc_rem_reg;
        pack_keep_mask = 32'd0;
        pack_byte_mask = {AXIS_PCIE_DATA_WIDTH{1'b0}};
        pack_mask_count = 6'd0;

        if (status_valid_reg && m_axis_status_ready) begin
            status_valid_reg <= 1'b0;
            desc_active_reg <= 1'b0;
            desc_error_code_reg <= 32'd0;
        end

        if (desc_fire) begin
            desc_in_valid_reg <= 1'b1;
            desc_in_host_addr_reg <= s_axis_desc_host_addr;
            desc_in_len_reg <= s_axis_desc_len;
            desc_in_tag_reg <= s_axis_desc_tag;
            desc_in_qid_reg <= s_axis_desc_qid;
            desc_in_cidx_after_reg <= s_axis_desc_cidx_after;
            desc_in_flags_reg <= s_axis_desc_flags;
            desc_in_swtag_reg <= s_axis_desc_swtag;
            desc_in_user_reg <= s_axis_desc_user;
        end

        if (desc_start) begin
            desc_in_valid_reg <= 1'b0;
            desc_active_reg <= 1'b1;
            desc_host_addr_reg <= desc_in_host_addr_reg;
            desc_len_reg <= desc_in_len_reg;
            issued_bytes_reg <= {LEN_WIDTH{1'b0}};
            output_bytes_reg <= 32'd0;
            desc_tag_reg <= desc_in_tag_reg;
            desc_qid_reg <= desc_in_qid_reg;
            desc_cidx_after_reg <= desc_in_cidx_after_reg;
            desc_flags_reg <= desc_in_flags_reg;
            desc_swtag_reg <= desc_in_swtag_reg;
            desc_user_reg <= desc_in_user_reg;
            desc_error_code_reg <= 32'd0;
            pack_desc_rem_reg <= desc_in_len_reg;
            pack_packet_len_reg <= desc_in_packet_len_comb;
            pack_packet_rem_reg <= desc_in_packet_len_comb;
            issue_ptr_reg <= {OUTSTANDING_INDEX_WIDTH{1'b0}};
            rc_ptr_reg <= {OUTSTANDING_INDEX_WIDTH{1'b0}};
            outstanding_count_reg <= {OUTSTANDING_COUNT_WIDTH{1'b0}};
            stream_pack_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            stream_pack_count_reg <= 6'd0;
            tail_flush_valid_reg <= 1'b0;
            tail_flush_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            tail_flush_bytes_reg <= 6'd0;
            packet_ready_count_reg <= {FIFO_COUNT_WIDTH{1'b0}};
            tx_out_state_reg <= TX_OUT_IDLE;
            rc_frame_active_reg <= 1'b0;
            rc_req_active_reg <= 1'b0;
            rc_req_rcvd_bytes_reg <= 13'd0;
            rc_req_len_reg <= 13'd0;
            rc_req_last_chunk_reg <= 1'b0;
            rc_req_tag_reg <= 8'd0;
            rc_next_req_valid_reg <= 1'b0;
            rc_next_req_len_reg <= 13'd0;
            rc_next_req_last_chunk_reg <= 1'b0;
            rc_next_req_tag_reg <= 8'd0;
            rc_drop_frame_reg <= 1'b0;
            rc_pipe_valid_reg <= 1'b0;
            rc_pipe_payload_valid_reg <= 1'b0;
            rc_pipe_payload_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            rc_pipe_payload_count_reg <= 6'd0;
            rc_pipe_req_done_reg <= 1'b0;
            rc_pipe_last_chunk_reg <= 1'b0;
            rc_pipe_header_error_reg <= 1'b0;
            rc_pipe_discontinue_reg <= 1'b0;
            rc_pipe_error_code_reg <= 32'd0;
            if (desc_in_len_reg == 0) begin
                status_should_complete = 1'b1;
            end else if (desc_in_len_reg > ST_TX_MAX_PACKET_BYTES) begin
                /* The driver submits one skb per descriptor.  Reject an
                 * unsupported oversize descriptor instead of silently
                 * turning it into multiple Ethernet frames. */
                desc_error_code_reg <= {`EDEV_ERR_ST_TX_DESC, 8'h01, desc_in_len_reg[15:0]};
            end
        end

        case (issue_state_reg)
            ISSUE_IDLE: begin
                if (enable && desc_active_reg && desc_error_code_reg == 0 && issued_bytes_reg < desc_len_reg &&
                    !rq_valid_reg && issue_window_available) begin
                    issue_state_reg <= ISSUE_CALC0;
                end
            end
            ISSUE_CALC0: begin
                issue_mrrs_bytes_reg <= issue_mrrs_limited_comb;
                issue_addr_reg <= desc_host_addr_reg + {{(PCIE_ADDR_WIDTH-LEN_WIDTH){1'b0}}, issued_bytes_reg};
                issue_remain_bytes_reg <= issue_desc_remain > {{(LEN_WIDTH-13){1'b0}}, PCIE_4K_BYTES} ? PCIE_4K_BYTES : issue_desc_remain[12:0];
                issue_slot_reg <= issue_ptr_reg;
                issue_state_reg <= ISSUE_CALC1;
            end
            ISSUE_CALC1: begin
                issue_mrrs_payload_limit_reg <=
                    (issue_mrrs_bytes_reg > {11'd0, issue_addr_reg[1:0]}) ?
                    (issue_mrrs_bytes_reg - {11'd0, issue_addr_reg[1:0]}) : 13'd1;
                issue_boundary_bytes_reg <= PCIE_4K_BYTES - {1'b0, issue_addr_reg[11:0]};
                issue_state_reg <= ISSUE_CALC2;
            end
            ISSUE_CALC2: begin
                issue_chunk_len_reg <= issue_chunk_len_safe;
                issue_chunk_len_len_reg <= issue_chunk_len_safe_len;
                issue_dw_count_reg <= (issue_chunk_len_safe + issue_addr_reg[1:0] + 3) >> 2;
                issue_first_be_reg <= issue_first_be_comb;
                issue_last_be_reg <= issue_last_be_comb;
                issue_last_chunk_reg <= (issued_bytes_reg + issue_chunk_len_safe_len) >= desc_len_reg;
                issue_state_reg <= ISSUE_BUILD;
            end
            ISSUE_BUILD: begin
                rq_tdata_reg[127:0] <= issue_rq_header_comb;
                rq_tdata_reg[255:128] <= 128'd0;
                rq_tkeep_reg <= 8'h0f;
                rq_tlast_reg <= 1'b1;
                rq_tuser_reg <= issue_rq_tuser_comb;
                rq_valid_reg <= 1'b1;
                issue_state_reg <= ISSUE_SEND;
            end
            ISSUE_SEND: begin
                if (rq_valid_reg && m_axis_rq_tready) begin
                    rq_valid_reg <= 1'b0;
                    issue_state_reg <= ISSUE_UPDATE;
                end
            end
            ISSUE_UPDATE: begin
                req_len_reg[issue_slot_reg] <= issue_chunk_len_reg;
                req_last_chunk_reg[issue_slot_reg] <= issue_last_chunk_reg;
                /* Feed the registered RC metadata window at issue time.  The
                 * first outstanding request becomes the current RC target; the
                 * second outstanding request is held in the next-target slot.
                 * Deeper requests remain in req_len_reg and are prefetched only
                 * after the current target is retired.
                 */
                if (outstanding_empty_reg && !rc_req_active_reg && !rc_meta_done_pending) begin
                    rc_req_active_reg <= 1'b1;
                    rc_req_len_reg <= issue_chunk_len_reg;
                    rc_req_rcvd_bytes_reg <= 13'd0;
                    rc_req_last_chunk_reg <= issue_last_chunk_reg;
                    rc_req_tag_reg <= issue_slot_tag_wire;
                end else if (outstanding_one_reg && !rc_next_req_valid_reg && !rc_meta_done_pending) begin
                    rc_next_req_valid_reg <= 1'b1;
                    rc_next_req_len_reg <= issue_chunk_len_reg;
                    rc_next_req_last_chunk_reg <= issue_last_chunk_reg;
                    rc_next_req_tag_reg <= issue_slot_tag_wire;
                end

                issue_ptr_reg <= issue_ptr_next_wire;
                issued_bytes_reg <= issued_bytes_reg + issue_chunk_len_len_reg;
                issue_state_reg <= ISSUE_IDLE;
            end
            default: begin
                issue_state_reg <= ISSUE_IDLE;
            end
        endcase

        /* Request-progress commit stage.  rc_meta_* was captured from the
         * previous accepted RC beat, so s_axis_rc_tkeep no longer drives
         * rc_req_len_reg in the same cycle.  When a request completes and
         * another request is already outstanding, the RC path promotes the
         * registered next-target slot; it does not index req_len_reg from
         * rc_ptr_reg in the same cycle as rc_pipe_req_done_reg capture.
         */
        if (rc_meta_valid_reg) begin
            rc_meta_valid_reg <= 1'b0;
            if (rc_meta_req_done_reg) begin
                rc_ptr_reg <= rc_next_ptr;
                if (rc_next_req_valid_reg) begin
                    /* Promote the registered next-target slot.  No req_len_reg
                     * read is on the RC input/pipe capture path.
                     */
                    rc_req_active_reg <= 1'b1;
                    rc_req_len_reg <= rc_next_req_len_reg;
                    rc_req_rcvd_bytes_reg <= 13'd0;
                    rc_req_last_chunk_reg <= rc_next_req_last_chunk_reg;
                    rc_req_tag_reg <= rc_next_req_tag_reg;

                    if (outstanding_gt_two_reg) begin
                        rc_next_req_valid_reg <= 1'b1;
                        rc_next_req_len_reg <= req_len_reg[rc_after_next_ptr];
                        rc_next_req_last_chunk_reg <= req_last_chunk_reg[rc_after_next_ptr];
                        rc_next_req_tag_reg <= rc_after_next_tag_wire;
                    end else if (issue_update_fire) begin
                        /* The old count was exactly two and ISSUE_UPDATE added
                         * one more request in this same cycle.  That new request
                         * is now the registered next target.
                         */
                        rc_next_req_valid_reg <= 1'b1;
                        rc_next_req_len_reg <= issue_chunk_len_reg;
                        rc_next_req_last_chunk_reg <= issue_last_chunk_reg;
                        rc_next_req_tag_reg <= issue_slot_tag_wire;
                    end else begin
                        rc_next_req_valid_reg <= 1'b0;
                        rc_next_req_len_reg <= 13'd0;
                        rc_next_req_last_chunk_reg <= 1'b0;
                        rc_next_req_tag_reg <= 8'd0;
                    end
                end else if (issue_update_fire) begin
                    /* The current request retired while a new request was issued.
                     * There was no old next-target slot, so the new request
                     * becomes current after the clock edge.
                     */
                    rc_req_active_reg <= 1'b1;
                    rc_req_len_reg <= issue_chunk_len_reg;
                    rc_req_rcvd_bytes_reg <= 13'd0;
                    rc_req_last_chunk_reg <= issue_last_chunk_reg;
                    rc_req_tag_reg <= issue_slot_tag_wire;
                    rc_next_req_valid_reg <= 1'b0;
                    rc_next_req_len_reg <= 13'd0;
                    rc_next_req_last_chunk_reg <= 1'b0;
                    rc_next_req_tag_reg <= 8'd0;
                end else begin
                    rc_req_active_reg <= 1'b0;
                    rc_req_len_reg <= 13'd0;
                    rc_req_rcvd_bytes_reg <= 13'd0;
                    rc_req_last_chunk_reg <= 1'b0;
                    rc_req_tag_reg <= 8'd0;
                    rc_next_req_valid_reg <= 1'b0;
                    rc_next_req_len_reg <= 13'd0;
                    rc_next_req_last_chunk_reg <= 1'b0;
                    rc_next_req_tag_reg <= 8'd0;
                end
            end else begin
                rc_req_rcvd_bytes_reg <= rc_meta_rcvd_after_reg;
            end
        end

        if (tail_flush_valid_reg && fifo_stage_ready) begin
            fifo_stage_load = 1'b1;
            fifo_stage_load_data = tail_flush_data_reg;
            fifo_stage_load_bytes = tail_flush_bytes_reg;
            fifo_stage_load_last = pack_packet_rem_reg <= {8'd0, tail_flush_bytes_reg};
            fifo_stage_load_tlen = pack_packet_len_reg;
            fifo_stage_load_user = desc_user_reg;
            tail_flush_valid_reg <= 1'b0;
            tail_flush_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            tail_flush_bytes_reg <= 6'd0;
        end else if (rc_pipe_drain) begin
            /* Stage 1: stream pack a registered RC payload beat.  The request
             * length metadata path has already been registered in Stage 0.
             */
            rc_pipe_valid_reg <= 1'b0;

            if (rc_pipe_header_error_reg || rc_pipe_discontinue_reg) begin
                if (desc_error_code_reg == 0) begin
                    desc_error_code_reg <= rc_pipe_error_code_reg;
                end
                stream_pack_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
                stream_pack_count_reg <= 6'd0;
                stat_tx_rc_errors_reg <= stat_tx_rc_errors_reg + 1'b1;
            end else if (rc_pipe_payload_valid_reg && desc_error_code_reg == 0) begin
                pack_combined_data = {{AXIS_PCIE_DATA_WIDTH{1'b0}}, stream_pack_data_reg};
                pack_combined_data = pack_combined_data | ({{AXIS_PCIE_DATA_WIDTH{1'b0}}, rc_pipe_payload_data_reg} << (stream_pack_count_reg * 8));
                pack_combined_count = {1'b0, stream_pack_count_reg} + {1'b0, rc_pipe_payload_count_reg};
                pack_desc_last = rc_pipe_last_chunk_reg && rc_pipe_req_done_reg;
                if (pack_combined_count >= AXIS_BYTE_WIDTH) begin
                    pack_mask_count = pack_combined_count - AXIS_BYTE_WIDTH;
                end else begin
                    pack_mask_count = pack_combined_count[5:0];
                end
                case (pack_mask_count)
                    6'd0:  pack_keep_mask = 32'h00000000;
                    6'd1:  pack_keep_mask = 32'h00000001;
                    6'd2:  pack_keep_mask = 32'h00000003;
                    6'd3:  pack_keep_mask = 32'h00000007;
                    6'd4:  pack_keep_mask = 32'h0000000f;
                    6'd5:  pack_keep_mask = 32'h0000001f;
                    6'd6:  pack_keep_mask = 32'h0000003f;
                    6'd7:  pack_keep_mask = 32'h0000007f;
                    6'd8:  pack_keep_mask = 32'h000000ff;
                    6'd9:  pack_keep_mask = 32'h000001ff;
                    6'd10: pack_keep_mask = 32'h000003ff;
                    6'd11: pack_keep_mask = 32'h000007ff;
                    6'd12: pack_keep_mask = 32'h00000fff;
                    6'd13: pack_keep_mask = 32'h00001fff;
                    6'd14: pack_keep_mask = 32'h00003fff;
                    6'd15: pack_keep_mask = 32'h00007fff;
                    6'd16: pack_keep_mask = 32'h0000ffff;
                    6'd17: pack_keep_mask = 32'h0001ffff;
                    6'd18: pack_keep_mask = 32'h0003ffff;
                    6'd19: pack_keep_mask = 32'h0007ffff;
                    6'd20: pack_keep_mask = 32'h000fffff;
                    6'd21: pack_keep_mask = 32'h001fffff;
                    6'd22: pack_keep_mask = 32'h003fffff;
                    6'd23: pack_keep_mask = 32'h007fffff;
                    6'd24: pack_keep_mask = 32'h00ffffff;
                    6'd25: pack_keep_mask = 32'h01ffffff;
                    6'd26: pack_keep_mask = 32'h03ffffff;
                    6'd27: pack_keep_mask = 32'h07ffffff;
                    6'd28: pack_keep_mask = 32'h0fffffff;
                    6'd29: pack_keep_mask = 32'h1fffffff;
                    6'd30: pack_keep_mask = 32'h3fffffff;
                    6'd31: pack_keep_mask = 32'h7fffffff;
                    default: pack_keep_mask = 32'hffffffff;
                endcase
                pack_byte_mask = {
                    {8{pack_keep_mask[31]}}, {8{pack_keep_mask[30]}}, {8{pack_keep_mask[29]}}, {8{pack_keep_mask[28]}},
                    {8{pack_keep_mask[27]}}, {8{pack_keep_mask[26]}}, {8{pack_keep_mask[25]}}, {8{pack_keep_mask[24]}},
                    {8{pack_keep_mask[23]}}, {8{pack_keep_mask[22]}}, {8{pack_keep_mask[21]}}, {8{pack_keep_mask[20]}},
                    {8{pack_keep_mask[19]}}, {8{pack_keep_mask[18]}}, {8{pack_keep_mask[17]}}, {8{pack_keep_mask[16]}},
                    {8{pack_keep_mask[15]}}, {8{pack_keep_mask[14]}}, {8{pack_keep_mask[13]}}, {8{pack_keep_mask[12]}},
                    {8{pack_keep_mask[11]}}, {8{pack_keep_mask[10]}}, {8{pack_keep_mask[9]}},  {8{pack_keep_mask[8]}},
                    {8{pack_keep_mask[7]}},  {8{pack_keep_mask[6]}},  {8{pack_keep_mask[5]}},  {8{pack_keep_mask[4]}},
                    {8{pack_keep_mask[3]}},  {8{pack_keep_mask[2]}},  {8{pack_keep_mask[1]}},  {8{pack_keep_mask[0]}}
                };

                if (pack_combined_count >= AXIS_BYTE_WIDTH) begin
                    pack_leftover_count = pack_combined_count - AXIS_BYTE_WIDTH;
                    fifo_stage_load = 1'b1;
                    fifo_stage_load_data = pack_combined_data[AXIS_PCIE_DATA_WIDTH-1:0];
                    fifo_stage_load_bytes = 6'd32;
                    fifo_stage_load_last = pack_packet_rem_reg <= 14'd32;
                    fifo_stage_load_tlen = pack_packet_len_reg;
                    fifo_stage_load_user = desc_user_reg;

                    if (pack_leftover_count != 0) begin
                        if (pack_desc_last) begin
                            tail_flush_valid_reg <= 1'b1;
                            tail_flush_data_reg <= pack_combined_data[AXIS_PCIE_DATA_WIDTH*2-1:AXIS_PCIE_DATA_WIDTH] & pack_byte_mask;
                            tail_flush_bytes_reg <= pack_leftover_count;
                            stream_pack_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
                            stream_pack_count_reg <= 6'd0;
                        end else begin
                            stream_pack_data_reg <= pack_combined_data[AXIS_PCIE_DATA_WIDTH*2-1:AXIS_PCIE_DATA_WIDTH] & pack_byte_mask;
                            stream_pack_count_reg <= pack_leftover_count;
                        end
                    end else begin
                        stream_pack_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
                        stream_pack_count_reg <= 6'd0;
                    end
                end else begin
                    if (pack_desc_last && pack_combined_count != 0) begin
                        tail_flush_valid_reg <= 1'b1;
                        tail_flush_data_reg <= pack_combined_data[AXIS_PCIE_DATA_WIDTH-1:0] & pack_byte_mask;
                        tail_flush_bytes_reg <= pack_combined_count[5:0];
                        stream_pack_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
                        stream_pack_count_reg <= 6'd0;
                    end else begin
                        stream_pack_data_reg <= pack_combined_data[AXIS_PCIE_DATA_WIDTH-1:0] & pack_byte_mask;
                        stream_pack_count_reg <= pack_combined_count[5:0];
                    end
                end
            end
        end

        if (rc_input_fire) begin
            /* Stage 0: resolve RC payload length and request progress, then
             * register the result before the wide stream packer.
             */
            rc_pipe_valid_reg <= rc_header_error || rc_discontinue || (!rc_drop_frame_reg && rc_payload_count != 0);
            rc_pipe_payload_valid_reg <= !rc_header_error && !rc_discontinue && !rc_drop_frame_reg && rc_payload_count != 0;
            rc_pipe_payload_data_reg <= rc_payload_data;
            rc_pipe_payload_count_reg <= rc_payload_count;
            rc_pipe_req_done_reg <= rc_req_done_after_beat;
            rc_pipe_last_chunk_reg <= rc_active_last_chunk_comb;
            rc_pipe_header_error_reg <= rc_header_error;
            rc_pipe_discontinue_reg <= rc_discontinue;
            rc_pipe_error_code_reg <= rc_tag_bad ? {`EDEV_ERR_ST_TX_BAD_TAG, 8'h00, s_axis_rc_tdata[71:64], rc_expected_tag} :
                                      rc_discontinue ? {`EDEV_ERR_ST_TX_DISCONTINUE, 24'd0} :
                                                       {`EDEV_ERR_ST_TX_RC_ERROR, 8'h00, 12'd0, s_axis_rc_tdata[15:12]};

            if (s_axis_rc_tlast) begin
                rc_frame_active_reg <= 1'b0;
                rc_drop_frame_reg <= 1'b0;
            end else begin
                rc_frame_active_reg <= 1'b1;
            end

            if (rc_header_error || rc_discontinue) begin
                rc_drop_frame_reg <= !s_axis_rc_tlast;
            end

            rc_meta_valid_reg <= 1'b1;
            rc_meta_rcvd_after_reg <= rc_rcvd_after_beat;
            rc_meta_req_done_reg <= rc_req_done_after_beat;
        end
        if (fifo_stage_push) begin
            fifo_tdata[fifo_wr_ptr_reg] <= fifo_stage_tdata_reg;
            fifo_tlast[fifo_wr_ptr_reg] <= fifo_stage_tlast_reg;
            fifo_tuser[fifo_wr_ptr_reg] <= fifo_stage_tuser_reg;
            fifo_tbytes[fifo_wr_ptr_reg] <= fifo_stage_tbytes_reg;
            fifo_tmty[fifo_wr_ptr_reg] <= fifo_stage_tmty_reg;
            fifo_tlen[fifo_wr_ptr_reg] <= fifo_stage_tlen_reg;
            fifo_wr_ptr_reg <= fifo_wr_ptr_reg + 1'b1;
        end

        if (fifo_stage_load) begin
            if (fifo_stage_load_last && fifo_stage_load_bytes != 0 && fifo_stage_load_bytes < AXIS_BYTE_WIDTH) begin
                fifo_stage_empty_bytes_comb = AXIS_BYTE_WIDTH - fifo_stage_load_bytes;
                fifo_stage_load_mty = fifo_stage_empty_bytes_comb[AXIS_ST_MTY_WIDTH-1:0];
            end else begin
                fifo_stage_load_mty = {AXIS_ST_MTY_WIDTH{1'b0}};
            end
            fifo_stage_tdata_reg <= fifo_stage_load_data;
            fifo_stage_tlast_reg <= fifo_stage_load_last;
            fifo_stage_tuser_reg <= fifo_stage_load_user;
            fifo_stage_tbytes_reg <= fifo_stage_load_bytes;
            fifo_stage_tmty_reg <= fifo_stage_load_mty;
            fifo_stage_tlen_reg <= fifo_stage_load_tlen;
            fifo_stage_valid_reg <= 1'b1;
        end else if (fifo_stage_push) begin
            fifo_stage_valid_reg <= 1'b0;
            fifo_stage_tlast_reg <= 1'b0;
            fifo_stage_tbytes_reg <= 6'd0;
            fifo_stage_tmty_reg <= {AXIS_ST_MTY_WIDTH{1'b0}};
            fifo_stage_tlen_reg <= 14'd0;
        end

        if (fifo_stage_load) begin
            if (pack_desc_rem_reg > {{(LEN_WIDTH-6){1'b0}}, fifo_stage_load_bytes}) begin
                pack_desc_rem_after_load = pack_desc_rem_reg - {{(LEN_WIDTH-6){1'b0}}, fifo_stage_load_bytes};
            end else begin
                pack_desc_rem_after_load = {LEN_WIDTH{1'b0}};
            end
            pack_desc_rem_reg <= pack_desc_rem_after_load;

            if (fifo_stage_load_last) begin
                pack_packet_len_reg <= 14'd0;
                pack_packet_rem_reg <= 14'd0;
            end else begin
                pack_packet_rem_reg <= pack_packet_rem_reg - {8'd0, fifo_stage_load_bytes};
            end
        end

        if (axis_output_fire) begin
            output_bytes_this_beat = {26'd0, axis_tbytes_reg};
            output_bytes_after_fire = output_bytes_reg + output_bytes_this_beat;
            output_bytes_reg <= output_bytes_after_fire;
            if (desc_error_code_reg == 0) begin
                stat_tx_bytes_reg <= stat_tx_bytes_reg + output_bytes_this_beat;
                if (axis_output_last_fire) begin
                    stat_tx_packets_reg <= stat_tx_packets_reg + 1'b1;
                end
            end
            /* The normal completion condition is the accepted descriptor byte count.
             * Keep the byte-count guard as a secondary protection against a
             * malformed frame that reaches the programmed length without TLAST.
             */
            if (output_bytes_after_fire >= {{(32-LEN_WIDTH){1'b0}}, desc_len_reg}) begin
                status_should_complete = 1'b1;
            end
        end

        if (axis_output_ready) begin
            if (axis_load_fifo) begin
                axis_tdata_reg <= fifo_tdata[fifo_rd_ptr_reg];
                axis_tlast_reg <= fifo_tlast[fifo_rd_ptr_reg];
                axis_tuser_reg <= fifo_tuser[fifo_rd_ptr_reg];
                axis_tlen_reg <= {{(LEN_WIDTH-14){1'b0}}, fifo_tlen[fifo_rd_ptr_reg]};
                axis_tmty_reg <= fifo_tmty[fifo_rd_ptr_reg];
                axis_tbytes_reg <= fifo_tbytes[fifo_rd_ptr_reg];
                axis_tvalid_reg <= 1'b1;
                fifo_rd_ptr_reg <= fifo_rd_ptr_reg + 1'b1;
                tx_out_state_reg <= TX_OUT_SEND;
            end else if (m_axis_st_tx_tready) begin
                axis_tvalid_reg <= 1'b0;
                axis_tlast_reg <= 1'b0;
                axis_tmty_reg <= {AXIS_ST_MTY_WIDTH{1'b0}};
                axis_tbytes_reg <= 6'd0;
                axis_tlen_reg <= {LEN_WIDTH{1'b0}};
            end
        end

        if (axis_output_last_fire && !(axis_load_fifo && packet_ready_count_reg > 1)) begin
            tx_out_state_reg <= TX_OUT_IDLE;
        end

        case ({fifo_stage_push_last, axis_output_last_fire})
            2'b10: packet_ready_count_reg <= packet_ready_count_reg + 1'b1;
            2'b01: packet_ready_count_reg <= packet_ready_count_reg - 1'b1;
            default: packet_ready_count_reg <= packet_ready_count_reg;
        endcase

        case ({fifo_stage_push, axis_load_fifo})
            2'b10: fifo_count_reg <= fifo_count_reg + 1'b1;
            2'b01: fifo_count_reg <= fifo_count_reg - 1'b1;
            default: fifo_count_reg <= fifo_count_reg;
        endcase

        if (desc_start) begin
            outstanding_count_reg <= {OUTSTANDING_COUNT_WIDTH{1'b0}};
            outstanding_empty_reg <= 1'b1;
            outstanding_one_reg <= 1'b0;
            outstanding_gt_two_reg <= 1'b0;
            outstanding_full_reg <= 1'b0;
        end else begin
            case ({issue_update_fire, rc_meta_done_fire})
                2'b10: begin
                    outstanding_count_reg <= outstanding_count_reg + 1'b1;
                    outstanding_empty_reg <= 1'b0;
                    outstanding_one_reg <= outstanding_empty_reg;
                    outstanding_gt_two_reg <= outstanding_count_reg >= 2;
                    outstanding_full_reg <= outstanding_count_reg == (OUTSTANDING_READS_COUNT - 1'b1);
                end
                2'b01: begin
                    outstanding_count_reg <= outstanding_count_reg - 1'b1;
                    outstanding_empty_reg <= outstanding_one_reg;
                    outstanding_one_reg <= outstanding_count_reg == 2;
                    outstanding_gt_two_reg <= outstanding_count_reg > 3;
                    outstanding_full_reg <= 1'b0;
                end
                default: begin
                    outstanding_count_reg <= outstanding_count_reg;
                    outstanding_empty_reg <= outstanding_empty_reg;
                    outstanding_one_reg <= outstanding_one_reg;
                    outstanding_gt_two_reg <= outstanding_gt_two_reg;
                    outstanding_full_reg <= outstanding_full_reg;
                end
            endcase
        end

        if (desc_active_reg && desc_error_code_reg != 0 && outstanding_empty_reg && issue_state_reg == ISSUE_IDLE &&
            fifo_count_reg == 0 && !fifo_stage_valid_reg && packet_ready_count_reg == 0 && tx_out_state_reg == TX_OUT_IDLE && !axis_tvalid_reg &&
            !tail_flush_valid_reg && !rc_pipe_valid_reg &&
            stream_pack_count_reg == 0 && !status_valid_reg) begin
            status_should_complete = 1'b1;
        end

        if (status_should_complete && !status_valid_reg) begin
            status_valid_reg <= 1'b1;
            status_bytes_reg <= desc_error_code_reg != 0 ? (axis_output_fire ? output_bytes_after_fire : output_bytes_reg) : {{(32-LEN_WIDTH){1'b0}}, desc_len_reg};
            status_error_code_reg <= desc_error_code_reg;
            status_user_reg <= desc_user_reg;
            if (desc_error_code_reg != 0) begin
                stat_tx_desc_errors_reg <= stat_tx_desc_errors_reg + 1'b1;
            end
        end

        if (rst) begin
            desc_in_valid_reg <= 1'b0;
            desc_in_host_addr_reg <= {PCIE_ADDR_WIDTH{1'b0}};
            desc_in_len_reg <= {LEN_WIDTH{1'b0}};
            desc_in_tag_reg <= {TAG_WIDTH{1'b0}};
            desc_in_qid_reg <= {QID_WIDTH{1'b0}};
            desc_in_cidx_after_reg <= {RING_INDEX_WIDTH{1'b0}};
            desc_in_flags_reg <= 16'd0;
            desc_in_swtag_reg <= 16'd0;
            desc_in_user_reg <= 32'd0;
            desc_active_reg <= 1'b0;
            desc_host_addr_reg <= {PCIE_ADDR_WIDTH{1'b0}};
            desc_len_reg <= {LEN_WIDTH{1'b0}};
            issued_bytes_reg <= {LEN_WIDTH{1'b0}};
            output_bytes_reg <= 32'd0;
            desc_tag_reg <= {TAG_WIDTH{1'b0}};
            desc_qid_reg <= {QID_WIDTH{1'b0}};
            desc_cidx_after_reg <= {RING_INDEX_WIDTH{1'b0}};
            desc_flags_reg <= 16'd0;
            desc_swtag_reg <= 16'd0;
            desc_user_reg <= 32'd0;
            desc_error_code_reg <= 32'd0;
            pack_desc_rem_reg <= {LEN_WIDTH{1'b0}};
            pack_packet_len_reg <= 14'd0;
            pack_packet_rem_reg <= 14'd0;
            issue_state_reg <= ISSUE_IDLE;
            issue_mrrs_bytes_reg <= 13'd0;
            issue_mrrs_payload_limit_reg <= 13'd0;
            issue_addr_reg <= {PCIE_ADDR_WIDTH{1'b0}};
            issue_remain_bytes_reg <= 13'd0;
            issue_boundary_bytes_reg <= 13'd0;
            issue_chunk_len_reg <= 13'd0;
            issue_chunk_len_len_reg <= {LEN_WIDTH{1'b0}};
            issue_dw_count_reg <= 11'd0;
            issue_first_be_reg <= 4'd0;
            issue_last_be_reg <= 4'd0;
            issue_last_chunk_reg <= 1'b0;
            issue_slot_reg <= {OUTSTANDING_INDEX_WIDTH{1'b0}};
            rq_valid_reg <= 1'b0;
            rq_tdata_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            rq_tkeep_reg <= {AXIS_PCIE_KEEP_WIDTH{1'b0}};
            rq_tlast_reg <= 1'b0;
            rq_tuser_reg <= {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
            issue_ptr_reg <= {OUTSTANDING_INDEX_WIDTH{1'b0}};
            rc_ptr_reg <= {OUTSTANDING_INDEX_WIDTH{1'b0}};
            outstanding_count_reg <= {OUTSTANDING_COUNT_WIDTH{1'b0}};
            outstanding_empty_reg <= 1'b1;
            outstanding_one_reg <= 1'b0;
            outstanding_gt_two_reg <= 1'b0;
            outstanding_full_reg <= 1'b0;
            req_last_chunk_reg <= {OUTSTANDING_READS{1'b0}};
            for (i = 0; i < OUTSTANDING_READS; i = i + 1) begin
                req_len_reg[i] <= 13'd0;
            end
            rc_frame_active_reg <= 1'b0;
            rc_req_active_reg <= 1'b0;
            rc_req_len_reg <= 13'd0;
            rc_req_rcvd_bytes_reg <= 13'd0;
            rc_req_last_chunk_reg <= 1'b0;
            rc_req_tag_reg <= 8'd0;
            rc_next_req_valid_reg <= 1'b0;
            rc_next_req_len_reg <= 13'd0;
            rc_next_req_last_chunk_reg <= 1'b0;
            rc_next_req_tag_reg <= 8'd0;
            rc_drop_frame_reg <= 1'b0;
            rc_meta_valid_reg <= 1'b0;
            rc_meta_rcvd_after_reg <= 13'd0;
            rc_meta_req_done_reg <= 1'b0;
            rc_pipe_valid_reg <= 1'b0;
            rc_pipe_payload_valid_reg <= 1'b0;
            rc_pipe_payload_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            rc_pipe_payload_count_reg <= 6'd0;
            rc_pipe_req_done_reg <= 1'b0;
            rc_pipe_last_chunk_reg <= 1'b0;
            rc_pipe_header_error_reg <= 1'b0;
            rc_pipe_discontinue_reg <= 1'b0;
            rc_pipe_error_code_reg <= 32'd0;
            stream_pack_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            stream_pack_count_reg <= 6'd0;
            tail_flush_valid_reg <= 1'b0;
            tail_flush_data_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            tail_flush_bytes_reg <= 6'd0;
            status_valid_reg <= 1'b0;
            status_bytes_reg <= 32'd0;
            status_error_code_reg <= 32'd0;
            status_user_reg <= 32'd0;
            fifo_wr_ptr_reg <= {FIFO_ADDR_WIDTH{1'b0}};
            fifo_rd_ptr_reg <= {FIFO_ADDR_WIDTH{1'b0}};
            fifo_count_reg <= {FIFO_COUNT_WIDTH{1'b0}};
            fifo_stage_tdata_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            fifo_stage_tlast_reg <= 1'b0;
            fifo_stage_tuser_reg <= 32'd0;
            fifo_stage_tbytes_reg <= 6'd0;
            fifo_stage_tmty_reg <= {AXIS_ST_MTY_WIDTH{1'b0}};
            fifo_stage_tlen_reg <= 14'd0;
            fifo_stage_valid_reg <= 1'b0;
            packet_ready_count_reg <= {FIFO_COUNT_WIDTH{1'b0}};
            tx_out_state_reg <= TX_OUT_IDLE;
            axis_tdata_reg <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            axis_tmty_reg <= {AXIS_ST_MTY_WIDTH{1'b0}};
            axis_tbytes_reg <= 6'd0;
            axis_tvalid_reg <= 1'b0;
            axis_tlast_reg <= 1'b0;
            axis_tuser_reg <= 32'd0;
            axis_tlen_reg <= {LEN_WIDTH{1'b0}};
            stat_tx_packets_reg <= 32'd0;
            stat_tx_bytes_reg <= 32'd0;
            stat_tx_rc_errors_reg <= 32'd0;
            stat_tx_desc_errors_reg <= 32'd0;
        end
    end

endmodule

`resetall
