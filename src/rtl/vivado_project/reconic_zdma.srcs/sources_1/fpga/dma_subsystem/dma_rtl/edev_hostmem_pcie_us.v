/* SPDX-License-Identifier: MIT
 * High-performance host-memory descriptor fetch / completion writeback engine.
 *
 * This version is intended for the credit-window scheduler.  It removes the old
 * one-entry prefetch/cache path and turns host-memory descriptor fetch into a
 * pipelined multi-request engine:
 *   - scheduler-side descriptor fetch requests are accepted into a small FIFO;
 *   - PCIe MRd requests use a reserved HMEM tag range, one tag per outstanding
 *     descriptor fetch slot;
 *   - RC completions are parsed with a small state machine using tdata/tkeep/tlast
 *     only; rc_tuser is intentionally not used on the timing path;
 *   - descriptor responses are written into a decoupling FIFO and streamed back
 *     with the scheduler tag carried by the original request;
 *   - completion writeback uses an independent FIFO and shares the RQ output
 *     through a small state machine.
 *
 * Variable Dword extraction and descriptor assembly are implemented with case
 * statements to avoid dynamic part-select timing paths.
 */

`resetall 
`timescale 1ns / 1ps 
`default_nettype none

module edev_hostmem_pcie_us #(
    parameter AXIS_PCIE_DATA_WIDTH = 256,
    parameter AXIS_PCIE_KEEP_WIDTH = AXIS_PCIE_DATA_WIDTH / 32,
    parameter AXIS_PCIE_RC_USER_WIDTH = 75,
    parameter AXIS_PCIE_RQ_USER_WIDTH = 62,
    parameter PCIE_ADDR_WIDTH = 64,
    parameter TAG_WIDTH = 8,
    parameter DESC_WIDTH = 256,
    parameter CMPL_WIDTH = 256,
    /* DESC_FETCH_PCIE_TAG is treated as a tag-range selector.  With the default
     * top-level value 8'hff, descriptor MRd tags are 8'hf0..8'hf6 and the fence
     * tag is 8'hf7.  The RC demux must route this HMEM tag range by mask.
     */
    parameter DESC_FETCH_PCIE_TAG = 8'hff,
    parameter DESC_TAG_SLOT_COUNT = 8,
    parameter DESC_TAG_SLOT_WIDTH = 3,
    parameter FETCH_REQ_FIFO_DEPTH = 16,
    parameter FETCH_REQ_FIFO_ADDR_WIDTH = 4,
    parameter CMPL_FIFO_DEPTH = 8,
    parameter CMPL_FIFO_ADDR_WIDTH = 3,
    parameter DESC_FIFO_DEPTH = 64,
    parameter DESC_FIFO_ADDR_WIDTH = 6,
    parameter TX_FC_ENABLE = 1
) (
    input wire clk,
    input wire rst,

    input  wire [PCIE_ADDR_WIDTH-1:0] desc_fetch_addr,
    input  wire [      TAG_WIDTH-1:0] desc_fetch_tag,
    input  wire [                4:0] desc_fetch_count,
    input  wire                       desc_fetch_is_st,
    input  wire                       desc_fetch_valid,
    output wire                       desc_fetch_ready,

    output wire [ TAG_WIDTH-1:0] desc_resp_tag,
    output wire [DESC_WIDTH-1:0] desc_resp_data,
    output wire [           3:0] desc_resp_error,
    output wire                  desc_resp_valid,
    input  wire                  desc_resp_ready,

    input  wire [PCIE_ADDR_WIDTH-1:0] cmpl_write_addr,
    input  wire [     CMPL_WIDTH-1:0] cmpl_write_data,
    input  wire                       cmpl_write_valid,
    input  wire                       cmpl_write_fence,
    output wire                       cmpl_write_ready,
    output wire                       cmpl_write_done,

    input  wire [   AXIS_PCIE_DATA_WIDTH-1:0] s_axis_rc_tdata,
    input  wire [   AXIS_PCIE_KEEP_WIDTH-1:0] s_axis_rc_tkeep,
    input  wire                               s_axis_rc_tvalid,
    output wire                               s_axis_rc_tready,
    input  wire                               s_axis_rc_tlast,
    input  wire [AXIS_PCIE_RC_USER_WIDTH-1:0] s_axis_rc_tuser,

    output wire [   AXIS_PCIE_DATA_WIDTH-1:0] m_axis_rq_tdata,
    output wire [   AXIS_PCIE_KEEP_WIDTH-1:0] m_axis_rq_tkeep,
    output wire                               m_axis_rq_tvalid,
    input  wire                               m_axis_rq_tready,
    output wire                               m_axis_rq_tlast,
    output wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] m_axis_rq_tuser,

    input wire [ 7:0] pcie_tx_fc_nph_av,
    input wire [ 7:0] pcie_tx_fc_ph_av,
    input wire [11:0] pcie_tx_fc_pd_av,

    input wire [15:0] requester_id,
    input wire        requester_id_enable,

    output wire status_error_cor,
    output wire status_error_uncor
);

    initial begin
        if (AXIS_PCIE_DATA_WIDTH != 256) begin
            $error("edev_hostmem_pcie_us_hp requires 256-bit AXI-ST PCIe interface (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_KEEP_WIDTH != 8) begin
            $error("edev_hostmem_pcie_us_hp requires 8-DW tkeep for 256-bit interface (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_RQ_USER_WIDTH != 60 && AXIS_PCIE_RQ_USER_WIDTH != 62) begin
            $error("edev_hostmem_pcie_us_hp supports UltraScale+ 256-bit RQ tuser width 60/62 only (instance %m)");
            $finish;
        end
        if (DESC_WIDTH != 256 || CMPL_WIDTH != 256) begin
            $error("edev_hostmem_pcie_us_hp expects 32B descriptor and 32B completion records (instance %m)");
            $finish;
        end
        if (DESC_TAG_SLOT_COUNT != 8 || DESC_TAG_SLOT_WIDTH != 3) begin
            $error("edev_hostmem_pcie_us_hp currently implements 8 HMEM PCIe tags (instance %m)");
            $finish;
        end
        if (FETCH_REQ_FIFO_DEPTH != (1 << FETCH_REQ_FIFO_ADDR_WIDTH)) begin
            $error("FETCH_REQ_FIFO_DEPTH must equal 2**FETCH_REQ_FIFO_ADDR_WIDTH (instance %m)");
            $finish;
        end
        if (CMPL_FIFO_DEPTH != (1 << CMPL_FIFO_ADDR_WIDTH)) begin
            $error("CMPL_FIFO_DEPTH must equal 2**CMPL_FIFO_ADDR_WIDTH (instance %m)");
            $finish;
        end
        if (DESC_FIFO_DEPTH != (1 << DESC_FIFO_ADDR_WIDTH)) begin
            $error("DESC_FIFO_DEPTH must equal 2**DESC_FIFO_ADDR_WIDTH (instance %m)");
            $finish;
        end
        if (DESC_FIFO_DEPTH < 64) begin
            $error("DESC_FIFO_DEPTH should be at least 64 descriptors for the windowed scheduler (instance %m)");
            $finish;
        end
    end

    localparam [3:0] REQ_MEM_READ = 4'b0000;
    localparam [3:0] REQ_MEM_WRITE = 4'b0001;

    localparam [2:0] RQ_IDLE = 3'd0;
    localparam [2:0] RQ_WR_BEAT1 = 3'd1;
    localparam [2:0] RQ_FENCE_REQ = 3'd2;
    localparam [2:0] RQ_FENCE_WAIT = 3'd3;
    localparam [2:0] RQ_WR_WAIT = 3'd4;

    localparam [2:0] RC_IDLE = 3'd0;
    localparam [2:0] RC_DW_SCAN = 3'd1;
    localparam [2:0] RC_DRAIN = 3'd2;
    localparam [2:0] RC_ERROR_FILL = 3'd3;
    localparam [2:0] RC_UPDATE_WAIT = 3'd4;

    localparam [DESC_TAG_SLOT_WIDTH-1:0] FENCE_SLOT = DESC_TAG_SLOT_COUNT - 1;
    localparam [7:0] HMEM_TAG_BASE = {DESC_FETCH_PCIE_TAG[7:4], 4'h0};
    localparam integer FETCH_SLOT_COUNT = DESC_TAG_SLOT_COUNT - 1;

    localparam integer FETCH_REQ_ENTRY_WIDTH = PCIE_ADDR_WIDTH + TAG_WIDTH + 5 + 1;
    localparam integer CMPL_FIFO_ENTRY_WIDTH = PCIE_ADDR_WIDTH + CMPL_WIDTH + 1;
    localparam integer DESC_FIFO_ENTRY_WIDTH = TAG_WIDTH + DESC_WIDTH + 4;

    localparam [FETCH_REQ_FIFO_ADDR_WIDTH:0] FETCH_REQ_FIFO_DEPTH_COUNT = FETCH_REQ_FIFO_DEPTH;
    localparam [CMPL_FIFO_ADDR_WIDTH:0] CMPL_FIFO_DEPTH_COUNT = CMPL_FIFO_DEPTH;
    localparam [DESC_FIFO_ADDR_WIDTH:0] DESC_FIFO_DEPTH_COUNT = DESC_FIFO_DEPTH;

    reg  [   DESC_FIFO_ADDR_WIDTH:0] desc_fifo_count_reg = 0;
    reg  [   DESC_FIFO_ADDR_WIDTH:0] desc_fifo_reserved_reg = 0;
    wire [       DESC_FIFO_ADDR_WIDTH:0] desc_fifo_reserved_plus_count = desc_fifo_count_reg + desc_fifo_reserved_reg;
    wire [       DESC_FIFO_ADDR_WIDTH:0] desc_fifo_space_for_req = DESC_FIFO_DEPTH_COUNT - desc_fifo_reserved_plus_count;

    reg  [   AXIS_PCIE_DATA_WIDTH-1:0] rq_tdata_reg = 0;
    reg  [   AXIS_PCIE_KEEP_WIDTH-1:0] rq_tkeep_reg = 0;
    reg                                rq_tvalid_reg = 1'b0;
    reg                                rq_tlast_reg = 1'b0;
    reg  [AXIS_PCIE_RQ_USER_WIDTH-1:0] rq_tuser_reg = 0;
    reg  [                        2:0] rq_state_reg = RQ_IDLE;
    reg                                rq_rr_reg = 1'b0;

    reg                                status_error_cor_reg = 1'b0;
    reg                                status_error_uncor_reg = 1'b0;
    reg                                cmpl_write_done_reg = 1'b0;

    wire                               rq_fire = rq_tvalid_reg && m_axis_rq_tready;

    assign m_axis_rq_tdata    = rq_tdata_reg;
    assign m_axis_rq_tkeep    = rq_tkeep_reg;
    assign m_axis_rq_tvalid   = rq_tvalid_reg;
    assign m_axis_rq_tlast    = rq_tlast_reg;
    assign m_axis_rq_tuser    = rq_tuser_reg;

    assign status_error_cor   = status_error_cor_reg;
    assign status_error_uncor = status_error_uncor_reg;
    assign cmpl_write_done    = cmpl_write_done_reg;
    reg  hmem_nph_credit_seen_reg = 1'b0;
    reg  hmem_ph_credit_seen_reg = 1'b0;
    reg  hmem_pd_credit_seen_reg = 1'b0;

    wire hmem_mrd_credit_ok = !TX_FC_ENABLE || !hmem_nph_credit_seen_reg || (pcie_tx_fc_nph_av != 8'd0);
    wire hmem_mwr_credit_ok = !TX_FC_ENABLE ||
                               ((!hmem_ph_credit_seen_reg || (pcie_tx_fc_ph_av != 8'd0)) &&
                                (!hmem_pd_credit_seen_reg || (pcie_tx_fc_pd_av != 12'd0)));

    always @(posedge clk) begin
        if (pcie_tx_fc_nph_av != 8'd0) begin
            hmem_nph_credit_seen_reg <= 1'b1;
        end
        if (pcie_tx_fc_ph_av != 8'd0) begin
            hmem_ph_credit_seen_reg <= 1'b1;
        end
        if (pcie_tx_fc_pd_av != 12'd0) begin
            hmem_pd_credit_seen_reg <= 1'b1;
        end

        if (rst) begin
            hmem_nph_credit_seen_reg <= 1'b0;
            hmem_ph_credit_seen_reg  <= 1'b0;
            hmem_pd_credit_seen_reg  <= 1'b0;
        end
    end

    /* ---------------- Scheduler fetch request FIFO ---------------- */

    (* ram_style = "distributed" *)reg  [    FETCH_REQ_ENTRY_WIDTH-1:0] fetch_req_fifo_mem                                                                             [0:FETCH_REQ_FIFO_DEPTH-1];
    reg  [FETCH_REQ_FIFO_ADDR_WIDTH-1:0] fetch_req_wr_ptr_reg = 0;
    reg  [FETCH_REQ_FIFO_ADDR_WIDTH-1:0] fetch_req_rd_ptr_reg = 0;
    reg  [  FETCH_REQ_FIFO_ADDR_WIDTH:0] fetch_req_count_reg = 0;

    wire [                          4:0] fetch_count_nonzero = (desc_fetch_count == 0) ? 5'd1 : desc_fetch_count;
    wire [       DESC_FIFO_ADDR_WIDTH:0] fetch_count_reserve = fetch_count_nonzero;
    wire                                 fetch_req_fifo_full = fetch_req_count_reg == FETCH_REQ_FIFO_DEPTH_COUNT;
    wire                                 fetch_req_fifo_empty = fetch_req_count_reg == 0;
    wire                                 fetch_count_fits = ({1'b0, fetch_count_nonzero} <= desc_fifo_space_for_req);
    wire                                 desc_fetch_accept = desc_fetch_valid && desc_fetch_ready;

    reg                                  fetch_req_pop_reg = 1'b0;
    wire [    FETCH_REQ_ENTRY_WIDTH-1:0] fetch_req_rd_entry = fetch_req_fifo_mem[fetch_req_rd_ptr_reg];
    wire [          PCIE_ADDR_WIDTH-1:0] fetch_req_rd_addr = fetch_req_rd_entry[PCIE_ADDR_WIDTH-1:0];
    wire [                TAG_WIDTH-1:0] fetch_req_rd_sched_tag = fetch_req_rd_entry[PCIE_ADDR_WIDTH+TAG_WIDTH-1:PCIE_ADDR_WIDTH];
    wire [                          4:0] fetch_req_rd_count = fetch_req_rd_entry[PCIE_ADDR_WIDTH+TAG_WIDTH+4:PCIE_ADDR_WIDTH+TAG_WIDTH];
    wire                                 fetch_req_rd_is_st = fetch_req_rd_entry[PCIE_ADDR_WIDTH+TAG_WIDTH+5];

    /* Accept scheduler descriptor-fetch requests into the local FIFO as soon as
     * there is internal buffering.  PCIe TX flow-control is applied later when
     * the RQ request is actually issued.  Gating desc_fetch_ready directly with
     * NPH credits can deadlock the scheduler in a visible state where
     * sched_active_qid points at a queue, QSTATUS.BUSY remains set, but
     * PERF_DESC_FETCH never increments.  This is especially easy to hit on
     * Tegra/root-port systems where cfg_fc_nph_av can transiently report zero
     * after the hostmem engine has seen credits once.
     */
    assign desc_fetch_ready = !fetch_req_fifo_full && fetch_count_fits;

    always @(posedge clk) begin
        if (desc_fetch_accept) begin
            fetch_req_fifo_mem[fetch_req_wr_ptr_reg] <= {desc_fetch_is_st, fetch_count_nonzero, desc_fetch_tag, desc_fetch_addr};
            fetch_req_wr_ptr_reg                     <= fetch_req_wr_ptr_reg + 1'b1;
        end
        if (fetch_req_pop_reg) begin
            fetch_req_rd_ptr_reg <= fetch_req_rd_ptr_reg + 1'b1;
        end
        case ({
            desc_fetch_accept, fetch_req_pop_reg
        })
            2'b10:   fetch_req_count_reg <= fetch_req_count_reg + 1'b1;
            2'b01:   fetch_req_count_reg <= fetch_req_count_reg - 1'b1;
            default: fetch_req_count_reg <= fetch_req_count_reg;
        endcase

        if (rst) begin
            fetch_req_wr_ptr_reg <= 0;
            fetch_req_rd_ptr_reg <= 0;
            fetch_req_count_reg  <= 0;
        end
    end

    /* ---------------- Descriptor response FIFO and credit accounting ---------------- */

    (* ram_style = "distributed" *)reg  [DESC_FIFO_ENTRY_WIDTH-1:0] desc_fifo_mem                                                                      [0:DESC_FIFO_DEPTH-1];
    reg  [ DESC_FIFO_ADDR_WIDTH-1:0] desc_fifo_wr_ptr_reg = 0;
    reg  [ DESC_FIFO_ADDR_WIDTH-1:0] desc_fifo_rd_ptr_reg = 0;

    reg                              desc_push_valid_reg = 1'b0;
    reg  [            TAG_WIDTH-1:0] desc_push_tag_reg = 0;
    reg  [           DESC_WIDTH-1:0] desc_push_data_reg = 0;
    reg  [                      3:0] desc_push_error_reg = 0;

    reg                              resp_valid_reg = 1'b0;
    reg  [            TAG_WIDTH-1:0] resp_tag_reg = 0;
    reg  [           DESC_WIDTH-1:0] resp_data_reg = 0;
    reg  [                      3:0] resp_error_reg = 0;

    wire                             desc_output_pop = (!resp_valid_reg || desc_resp_ready) && desc_fifo_count_reg != 0;
    wire                             desc_resp_fire = resp_valid_reg && desc_resp_ready;

    assign desc_resp_valid = resp_valid_reg;
    assign desc_resp_tag   = resp_tag_reg;
    assign desc_resp_data  = resp_data_reg;
    assign desc_resp_error = resp_error_reg;


    always @(posedge clk) begin
        if (desc_push_valid_reg) begin
            desc_fifo_mem[desc_fifo_wr_ptr_reg] <= {desc_push_tag_reg, desc_push_error_reg, desc_push_data_reg};
            desc_fifo_wr_ptr_reg                <= desc_fifo_wr_ptr_reg + 1'b1;
        end

        if (!resp_valid_reg || desc_resp_ready) begin
            if (desc_fifo_count_reg != 0) begin
                resp_data_reg        <= desc_fifo_mem[desc_fifo_rd_ptr_reg][DESC_WIDTH-1:0];
                resp_error_reg       <= desc_fifo_mem[desc_fifo_rd_ptr_reg][DESC_WIDTH+3:DESC_WIDTH];
                resp_tag_reg         <= desc_fifo_mem[desc_fifo_rd_ptr_reg][DESC_WIDTH+4+TAG_WIDTH-1:DESC_WIDTH+4];
                resp_valid_reg       <= 1'b1;
                desc_fifo_rd_ptr_reg <= desc_fifo_rd_ptr_reg + 1'b1;
            end else begin
                resp_valid_reg <= 1'b0;
            end
        end

        case ({
            desc_push_valid_reg, desc_output_pop
        })
            2'b10:   desc_fifo_count_reg <= desc_fifo_count_reg + 1'b1;
            2'b01:   desc_fifo_count_reg <= desc_fifo_count_reg - 1'b1;
            default: desc_fifo_count_reg <= desc_fifo_count_reg;
        endcase

        case ({
            desc_fetch_accept, desc_push_valid_reg
        })
            2'b10:   desc_fifo_reserved_reg <= desc_fifo_reserved_reg + fetch_count_reserve;
            2'b01:   desc_fifo_reserved_reg <= desc_fifo_reserved_reg - 1'b1;
            2'b11:   desc_fifo_reserved_reg <= desc_fifo_reserved_reg + fetch_count_reserve - 1'b1;
            default: desc_fifo_reserved_reg <= desc_fifo_reserved_reg;
        endcase

        if (rst) begin
            desc_fifo_wr_ptr_reg   <= 0;
            desc_fifo_rd_ptr_reg   <= 0;
            desc_fifo_count_reg    <= 0;
            desc_fifo_reserved_reg <= 0;
            resp_valid_reg         <= 1'b0;
            resp_tag_reg           <= 0;
            resp_data_reg          <= 0;
            resp_error_reg         <= 0;
        end
    end

    /* ---------------- Completion writeback FIFO ---------------- */

    (* ram_style = "distributed" *)reg  [CMPL_FIFO_ENTRY_WIDTH-1:0] cmpl_fifo_mem                                                 [0:CMPL_FIFO_DEPTH-1];
    reg  [ CMPL_FIFO_ADDR_WIDTH-1:0] cmpl_fifo_wr_ptr_reg = 0;
    reg  [ CMPL_FIFO_ADDR_WIDTH-1:0] cmpl_fifo_rd_ptr_reg = 0;
    reg  [   CMPL_FIFO_ADDR_WIDTH:0] cmpl_fifo_count_reg = 0;

    wire                             cmpl_fifo_full = cmpl_fifo_count_reg == CMPL_FIFO_DEPTH_COUNT;
    wire                             cmpl_fifo_empty = cmpl_fifo_count_reg == 0;
    wire                             cmpl_push = cmpl_write_valid && cmpl_write_ready;
    reg                              cmpl_pop_reg = 1'b0;
    wire [CMPL_FIFO_ENTRY_WIDTH-1:0] cmpl_rd_entry = cmpl_fifo_mem[cmpl_fifo_rd_ptr_reg];

    assign cmpl_write_ready = !cmpl_fifo_full;

    always @(posedge clk) begin
        if (cmpl_push) begin
            cmpl_fifo_mem[cmpl_fifo_wr_ptr_reg] <= {cmpl_write_fence, cmpl_write_addr, cmpl_write_data};
            cmpl_fifo_wr_ptr_reg                <= cmpl_fifo_wr_ptr_reg + 1'b1;
        end
        if (cmpl_pop_reg) begin
            cmpl_fifo_rd_ptr_reg <= cmpl_fifo_rd_ptr_reg + 1'b1;
        end
        case ({
            cmpl_push, cmpl_pop_reg
        })
            2'b10:   cmpl_fifo_count_reg <= cmpl_fifo_count_reg + 1'b1;
            2'b01:   cmpl_fifo_count_reg <= cmpl_fifo_count_reg - 1'b1;
            default: cmpl_fifo_count_reg <= cmpl_fifo_count_reg;
        endcase

        if (rst) begin
            cmpl_fifo_wr_ptr_reg <= 0;
            cmpl_fifo_rd_ptr_reg <= 0;
            cmpl_fifo_count_reg  <= 0;
        end
    end

    /* ---------------- PCIe tag slot table ---------------- */

    reg [FETCH_SLOT_COUNT-1:0] slot_valid_reg = 0;
    reg [TAG_WIDTH-1:0] slot_sched_tag_0 = 0, slot_sched_tag_1 = 0, slot_sched_tag_2 = 0, slot_sched_tag_3 = 0;
    reg [TAG_WIDTH-1:0] slot_sched_tag_4 = 0, slot_sched_tag_5 = 0, slot_sched_tag_6 = 0;
    reg slot_is_st_0 = 1'b0, slot_is_st_1 = 1'b0, slot_is_st_2 = 1'b0, slot_is_st_3 = 1'b0;
    reg slot_is_st_4 = 1'b0, slot_is_st_5 = 1'b0, slot_is_st_6 = 1'b0;
    reg [4:0] slot_count_0 = 0, slot_count_1 = 0, slot_count_2 = 0, slot_count_3 = 0;
    reg [4:0] slot_count_4 = 0, slot_count_5 = 0, slot_count_6 = 0;
    reg [7:0] slot_dws_left_0 = 0, slot_dws_left_1 = 0, slot_dws_left_2 = 0, slot_dws_left_3 = 0;
    reg [7:0] slot_dws_left_4 = 0, slot_dws_left_5 = 0, slot_dws_left_6 = 0;

    reg [DESC_TAG_SLOT_WIDTH-1:0] slot_scan_ptr_reg = 0;
    reg                           free_slot_valid_comb;
    reg [DESC_TAG_SLOT_WIDTH-1:0] free_slot_index_comb;

    always @* begin
        free_slot_valid_comb = 1'b0;
        free_slot_index_comb = 0;
        case (slot_scan_ptr_reg)
            3'd0: begin
                if (!slot_valid_reg[0]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd0;
                end else if (!slot_valid_reg[1]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd1;
                end else if (!slot_valid_reg[2]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd2;
                end else if (!slot_valid_reg[3]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd3;
                end else if (!slot_valid_reg[4]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd4;
                end else if (!slot_valid_reg[5]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd5;
                end else if (!slot_valid_reg[6]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd6;
                end
            end
            3'd1: begin
                if (!slot_valid_reg[1]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd1;
                end else if (!slot_valid_reg[2]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd2;
                end else if (!slot_valid_reg[3]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd3;
                end else if (!slot_valid_reg[4]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd4;
                end else if (!slot_valid_reg[5]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd5;
                end else if (!slot_valid_reg[6]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd6;
                end else if (!slot_valid_reg[0]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd0;
                end
            end
            3'd2: begin
                if (!slot_valid_reg[2]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd2;
                end else if (!slot_valid_reg[3]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd3;
                end else if (!slot_valid_reg[4]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd4;
                end else if (!slot_valid_reg[5]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd5;
                end else if (!slot_valid_reg[6]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd6;
                end else if (!slot_valid_reg[0]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd0;
                end else if (!slot_valid_reg[1]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd1;
                end
            end
            3'd3: begin
                if (!slot_valid_reg[3]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd3;
                end else if (!slot_valid_reg[4]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd4;
                end else if (!slot_valid_reg[5]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd5;
                end else if (!slot_valid_reg[6]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd6;
                end else if (!slot_valid_reg[0]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd0;
                end else if (!slot_valid_reg[1]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd1;
                end else if (!slot_valid_reg[2]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd2;
                end
            end
            3'd4: begin
                if (!slot_valid_reg[4]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd4;
                end else if (!slot_valid_reg[5]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd5;
                end else if (!slot_valid_reg[6]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd6;
                end else if (!slot_valid_reg[0]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd0;
                end else if (!slot_valid_reg[1]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd1;
                end else if (!slot_valid_reg[2]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd2;
                end else if (!slot_valid_reg[3]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd3;
                end
            end
            3'd5: begin
                if (!slot_valid_reg[5]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd5;
                end else if (!slot_valid_reg[6]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd6;
                end else if (!slot_valid_reg[0]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd0;
                end else if (!slot_valid_reg[1]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd1;
                end else if (!slot_valid_reg[2]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd2;
                end else if (!slot_valid_reg[3]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd3;
                end else if (!slot_valid_reg[4]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd4;
                end
            end
            default: begin
                if (!slot_valid_reg[6]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd6;
                end else if (!slot_valid_reg[0]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd0;
                end else if (!slot_valid_reg[1]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd1;
                end else if (!slot_valid_reg[2]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd2;
                end else if (!slot_valid_reg[3]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd3;
                end else if (!slot_valid_reg[4]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd4;
                end else if (!slot_valid_reg[5]) begin
                    free_slot_valid_comb = 1'b1;
                    free_slot_index_comb = 3'd5;
                end
            end
        endcase
    end

    reg                           slot_alloc_valid_reg = 1'b0;
    reg [DESC_TAG_SLOT_WIDTH-1:0] slot_alloc_index_reg = 0;
    reg [          TAG_WIDTH-1:0] slot_alloc_sched_tag_reg = 0;
    reg                           slot_alloc_is_st_reg = 1'b0;
    reg [                    4:0] slot_alloc_count_reg = 0;
    reg [                    7:0] slot_alloc_dw_count_reg = 0;
    reg                           slot_free_valid_reg = 1'b0;
    reg [DESC_TAG_SLOT_WIDTH-1:0] slot_free_index_reg = 0;
    reg                           slot_update_valid_reg = 1'b0;
    reg [DESC_TAG_SLOT_WIDTH-1:0] slot_update_index_reg = 0;
    reg [                    4:0] slot_update_count_reg = 0;
    reg [                    7:0] slot_update_dws_left_reg = 0;

    always @(posedge clk) begin
        if (slot_free_valid_reg) begin
            case (slot_free_index_reg)
                3'd0: slot_valid_reg[0] <= 1'b0;
                3'd1: slot_valid_reg[1] <= 1'b0;
                3'd2: slot_valid_reg[2] <= 1'b0;
                3'd3: slot_valid_reg[3] <= 1'b0;
                3'd4: slot_valid_reg[4] <= 1'b0;
                3'd5: slot_valid_reg[5] <= 1'b0;
                3'd6: slot_valid_reg[6] <= 1'b0;
                default: slot_valid_reg <= slot_valid_reg;
            endcase
        end

        if (slot_update_valid_reg) begin
            case (slot_update_index_reg)
                3'd0: begin
                    slot_count_0    <= slot_update_count_reg;
                    slot_dws_left_0 <= slot_update_dws_left_reg;
                end
                3'd1: begin
                    slot_count_1    <= slot_update_count_reg;
                    slot_dws_left_1 <= slot_update_dws_left_reg;
                end
                3'd2: begin
                    slot_count_2    <= slot_update_count_reg;
                    slot_dws_left_2 <= slot_update_dws_left_reg;
                end
                3'd3: begin
                    slot_count_3    <= slot_update_count_reg;
                    slot_dws_left_3 <= slot_update_dws_left_reg;
                end
                3'd4: begin
                    slot_count_4    <= slot_update_count_reg;
                    slot_dws_left_4 <= slot_update_dws_left_reg;
                end
                3'd5: begin
                    slot_count_5    <= slot_update_count_reg;
                    slot_dws_left_5 <= slot_update_dws_left_reg;
                end
                default: begin
                    slot_count_6    <= slot_update_count_reg;
                    slot_dws_left_6 <= slot_update_dws_left_reg;
                end
            endcase
        end

        if (slot_alloc_valid_reg) begin
            case (slot_alloc_index_reg)
                3'd0: begin
                    slot_valid_reg[0] <= 1'b1;
                    slot_sched_tag_0  <= slot_alloc_sched_tag_reg;
                    slot_is_st_0      <= slot_alloc_is_st_reg;
                    slot_count_0      <= slot_alloc_count_reg;
                    slot_dws_left_0   <= slot_alloc_dw_count_reg;
                end
                3'd1: begin
                    slot_valid_reg[1] <= 1'b1;
                    slot_sched_tag_1  <= slot_alloc_sched_tag_reg;
                    slot_is_st_1      <= slot_alloc_is_st_reg;
                    slot_count_1      <= slot_alloc_count_reg;
                    slot_dws_left_1   <= slot_alloc_dw_count_reg;
                end
                3'd2: begin
                    slot_valid_reg[2] <= 1'b1;
                    slot_sched_tag_2  <= slot_alloc_sched_tag_reg;
                    slot_is_st_2      <= slot_alloc_is_st_reg;
                    slot_count_2      <= slot_alloc_count_reg;
                    slot_dws_left_2   <= slot_alloc_dw_count_reg;
                end
                3'd3: begin
                    slot_valid_reg[3] <= 1'b1;
                    slot_sched_tag_3  <= slot_alloc_sched_tag_reg;
                    slot_is_st_3      <= slot_alloc_is_st_reg;
                    slot_count_3      <= slot_alloc_count_reg;
                    slot_dws_left_3   <= slot_alloc_dw_count_reg;
                end
                3'd4: begin
                    slot_valid_reg[4] <= 1'b1;
                    slot_sched_tag_4  <= slot_alloc_sched_tag_reg;
                    slot_is_st_4      <= slot_alloc_is_st_reg;
                    slot_count_4      <= slot_alloc_count_reg;
                    slot_dws_left_4   <= slot_alloc_dw_count_reg;
                end
                3'd5: begin
                    slot_valid_reg[5] <= 1'b1;
                    slot_sched_tag_5  <= slot_alloc_sched_tag_reg;
                    slot_is_st_5      <= slot_alloc_is_st_reg;
                    slot_count_5      <= slot_alloc_count_reg;
                    slot_dws_left_5   <= slot_alloc_dw_count_reg;
                end
                default: begin
                    slot_valid_reg[6] <= 1'b1;
                    slot_sched_tag_6  <= slot_alloc_sched_tag_reg;
                    slot_is_st_6      <= slot_alloc_is_st_reg;
                    slot_count_6      <= slot_alloc_count_reg;
                    slot_dws_left_6   <= slot_alloc_dw_count_reg;
                end
            endcase
            slot_scan_ptr_reg <= (slot_alloc_index_reg == 3'd6) ? 3'd0 : slot_alloc_index_reg + 1'b1;
        end

        if (rst) begin
            slot_valid_reg    <= 0;
            slot_scan_ptr_reg <= 0;
        end
    end

    /* ---------------- RQ issue state machine ---------------- */

    reg  [             CMPL_WIDTH-1:0] wr_data_hold_reg = 0;
    reg  [        PCIE_ADDR_WIDTH-1:0] wr_addr_hold_reg = 0;
    reg                                wr_fence_hold_reg = 1'b0;
    reg                                fence_wait_reg = 1'b0;
    reg                                fence_done_pulse_reg = 1'b0;

    wire                               fetch_issue_ready = !fetch_req_fifo_empty && free_slot_valid_comb;
    wire                               cmpl_issue_ready = !cmpl_fifo_empty && !fence_wait_reg;


    wire                               issue_fetch_now = fetch_issue_ready && hmem_mrd_credit_ok && (!cmpl_issue_ready || !hmem_mwr_credit_ok || !rq_rr_reg);
    wire                               issue_cmpl_now = cmpl_issue_ready && hmem_mwr_credit_ok && (!fetch_issue_ready || !hmem_mrd_credit_ok || rq_rr_reg);

    reg  [                       10:0] rq_dw_len_reg;
    reg  [                        7:0] rq_pcie_tag_reg;
    reg  [                      127:0] rq_header_next_reg;
    reg  [AXIS_PCIE_RQ_USER_WIDTH-1:0] rq_tuser_next_reg;
    reg  [                        7:0] cmpl_tag_tmp;

    always @(posedge clk) begin
        cmpl_write_done_reg  <= 1'b0;
        slot_alloc_valid_reg <= 1'b0;
        fetch_req_pop_reg    <= 1'b0;
        cmpl_pop_reg         <= 1'b0;
        rq_header_next_reg   <= 128'd0;
        rq_tuser_next_reg    <= {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};

        if (rq_fire) begin
            rq_tvalid_reg <= 1'b0;
        end

        case (rq_state_reg)
            RQ_IDLE: begin
                if (!rq_tvalid_reg && issue_fetch_now) begin
                    rq_dw_len_reg              = fetch_req_rd_is_st ? ({6'd0, fetch_req_rd_count} << 2) : ({6'd0, fetch_req_rd_count} << 3);
                    rq_pcie_tag_reg            = HMEM_TAG_BASE | {5'd0, free_slot_index_comb};
                    rq_header_next_reg         = 128'd0;
                    rq_header_next_reg[1:0]    = 2'b00;
                    rq_header_next_reg[63:2]   = fetch_req_rd_addr[PCIE_ADDR_WIDTH-1:2];
                    rq_header_next_reg[74:64]  = rq_dw_len_reg;
                    rq_header_next_reg[78:75]  = REQ_MEM_READ;
                    rq_header_next_reg[95:80]  = requester_id;
                    rq_header_next_reg[103:96] = rq_pcie_tag_reg;
                    rq_header_next_reg[120]    = requester_id_enable;
                    rq_tuser_next_reg          = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                    rq_tuser_next_reg[3:0]     = 4'hf;
                    rq_tuser_next_reg[7:4]     = 4'hf;

                    rq_tdata_reg[127:0]      <= rq_header_next_reg;
                    rq_tdata_reg[255:128]    <= 128'd0;
                    rq_tkeep_reg             <= 8'h0f;
                    rq_tlast_reg             <= 1'b1;
                    rq_tuser_reg             <= rq_tuser_next_reg;
                    rq_tvalid_reg            <= 1'b1;

                    slot_alloc_valid_reg     <= 1'b1;
                    slot_alloc_index_reg     <= free_slot_index_comb;
                    slot_alloc_sched_tag_reg <= fetch_req_rd_sched_tag;
                    slot_alloc_is_st_reg     <= fetch_req_rd_is_st;
                    slot_alloc_count_reg     <= fetch_req_rd_count;
                    slot_alloc_dw_count_reg  <= rq_dw_len_reg[7:0];
                    fetch_req_pop_reg        <= 1'b1;
                    rq_rr_reg                <= 1'b1;
                end else if (!rq_tvalid_reg && issue_cmpl_now) begin
                    wr_data_hold_reg  <= cmpl_rd_entry[CMPL_WIDTH-1:0];
                    wr_addr_hold_reg  <= cmpl_rd_entry[PCIE_ADDR_WIDTH+CMPL_WIDTH-1:CMPL_WIDTH];
                    wr_fence_hold_reg <= cmpl_rd_entry[PCIE_ADDR_WIDTH+CMPL_WIDTH];
                    cmpl_pop_reg      <= 1'b1;

                    rq_header_next_reg         = 128'd0;
                    rq_header_next_reg[1:0]    = 2'b00;
                    rq_header_next_reg[63:2]   = cmpl_rd_entry[PCIE_ADDR_WIDTH+CMPL_WIDTH-1:CMPL_WIDTH+2];
                    rq_header_next_reg[74:64]  = 11'd8;
                    rq_header_next_reg[78:75]  = REQ_MEM_WRITE;
                    rq_header_next_reg[95:80]  = requester_id;
                    rq_header_next_reg[103:96] = 8'd0;
                    rq_header_next_reg[120]    = requester_id_enable;
                    rq_tuser_next_reg          = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                    rq_tuser_next_reg[3:0]     = 4'hf;
                    rq_tuser_next_reg[7:4]     = 4'hf;

                    rq_tdata_reg[127:0]   <= rq_header_next_reg;
                    rq_tdata_reg[255:128] <= cmpl_rd_entry[127:0];
                    rq_tkeep_reg          <= 8'hff;
                    rq_tlast_reg          <= 1'b0;
                    rq_tuser_reg          <= rq_tuser_next_reg;
                    rq_tvalid_reg         <= 1'b1;
                    rq_state_reg          <= RQ_WR_BEAT1;
                    rq_rr_reg             <= 1'b0;
                end
            end

            RQ_WR_BEAT1: begin
                if (!rq_tvalid_reg) begin
                    rq_tuser_next_reg      = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                    rq_tuser_next_reg[3:0] = 4'hf;
                    rq_tuser_next_reg[7:4] = 4'hf;
                    rq_tdata_reg[127:0]   <= wr_data_hold_reg[255:128];
                    rq_tdata_reg[255:128] <= 128'd0;
                    rq_tkeep_reg          <= 8'h0f;
                    rq_tlast_reg          <= 1'b1;
                    rq_tuser_reg          <= rq_tuser_next_reg;
                    rq_tvalid_reg         <= 1'b1;
                    rq_state_reg          <= RQ_WR_WAIT;
                end
            end

            RQ_WR_WAIT: begin
                if (rq_fire) begin
                    if (wr_fence_hold_reg) begin
                        rq_state_reg <= RQ_FENCE_REQ;
                    end else begin
                        cmpl_write_done_reg <= 1'b1;
                        rq_state_reg        <= RQ_IDLE;
                    end
                end
            end

            RQ_FENCE_REQ: begin
                if (!rq_tvalid_reg && hmem_mrd_credit_ok) begin
                    rq_pcie_tag_reg            = HMEM_TAG_BASE | {5'd0, FENCE_SLOT};
                    rq_header_next_reg         = 128'd0;
                    rq_header_next_reg[1:0]    = 2'b00;
                    rq_header_next_reg[63:2]   = wr_addr_hold_reg[PCIE_ADDR_WIDTH-1:2];
                    rq_header_next_reg[74:64]  = 11'd1;
                    rq_header_next_reg[78:75]  = REQ_MEM_READ;
                    rq_header_next_reg[95:80]  = requester_id;
                    rq_header_next_reg[103:96] = rq_pcie_tag_reg;
                    rq_header_next_reg[120]    = requester_id_enable;
                    rq_tuser_next_reg          = {AXIS_PCIE_RQ_USER_WIDTH{1'b0}};
                    rq_tuser_next_reg[3:0]     = 4'hf;
                    rq_tuser_next_reg[7:4]     = 4'h0;
                    rq_tdata_reg[127:0]   <= rq_header_next_reg;
                    rq_tdata_reg[255:128] <= 128'd0;
                    rq_tkeep_reg          <= 8'h0f;
                    rq_tlast_reg          <= 1'b1;
                    rq_tuser_reg          <= rq_tuser_next_reg;
                    rq_tvalid_reg         <= 1'b1;
                    fence_wait_reg        <= 1'b1;
                    rq_state_reg          <= RQ_FENCE_WAIT;
                end
            end

            RQ_FENCE_WAIT: begin
                if (fence_done_pulse_reg) begin
                    fence_wait_reg      <= 1'b0;
                    cmpl_write_done_reg <= 1'b1;
                    rq_state_reg        <= RQ_IDLE;
                end
            end

            default: rq_state_reg <= RQ_IDLE;
        endcase

        if (rst) begin
            // Clear ownership only. Every valid beat overwrites its payload
            // and each fence owns a freshly captured completion address.
            rq_state_reg        <= RQ_IDLE;
            rq_tvalid_reg       <= 1'b0;
            rq_rr_reg           <= 1'b0;
            wr_fence_hold_reg   <= 1'b0;
            fence_wait_reg      <= 1'b0;
            cmpl_write_done_reg <= 1'b0;
        end
    end

    /* ---------------- RC parser: no rc_tuser parsing ---------------- */

    reg  [                     2:0] rc_state_reg = RC_IDLE;
    reg  [AXIS_PCIE_DATA_WIDTH-1:0] rc_data_reg = 0;
    reg  [AXIS_PCIE_KEEP_WIDTH-1:0] rc_keep_reg = 0;
    reg                             rc_last_reg = 1'b0;
    reg                             rc_header_beat_reg = 1'b0;
    reg                             rc_need_header_reg = 1'b1;
    reg  [                     2:0] rc_step_reg = 0;
    reg  [ DESC_TAG_SLOT_WIDTH-1:0] rc_slot_reg = 0;
    reg                             rc_is_fence_reg = 1'b0;
    reg                             rc_known_slot_reg = 1'b0;
    reg  [           TAG_WIDTH-1:0] rc_sched_tag_reg = 0;
    reg                             rc_is_st_reg = 1'b0;
    reg  [                     4:0] rc_descs_left_reg = 0;
    reg  [                     7:0] rc_dws_left_reg = 0;
    reg  [                     2:0] rc_desc_dw_count_reg = 0;
    reg  [                     2:0] rc_desc_dw_last_reg = 3'd7;
    reg  [                   255:0] rc_desc_data_reg = 0;
    reg  [                     3:0] rc_error_code_reg = 0;
    reg  [                     4:0] rc_error_fill_left_reg = 0;
    reg  [                    31:0] rc_dw_reg = 0;
    reg                             rc_dw_keep_reg = 1'b0;
    reg  [                   255:0] rc_desc_next_reg = 0;
    reg                             rc_desc_complete_reg = 1'b0;
    reg                             rc_payload_done_reg = 1'b0;
    reg  [                     7:0] rc_dws_left_tmp_reg = 0;
    reg  [                     4:0] rc_descs_left_tmp_reg = 0;

    wire                            rc_fire = s_axis_rc_tvalid && s_axis_rc_tready;
    wire [                     7:0] rc_header_tag = s_axis_rc_tdata[71:64];
    wire [                     3:0] rc_header_status = s_axis_rc_tdata[15:12];
    wire                            rc_header_tag_in_range = (rc_header_tag[7:4] == HMEM_TAG_BASE[7:4]);
    wire                            rc_header_is_fence = rc_header_tag_in_range && rc_header_tag[2:0] == FENCE_SLOT;
    wire                            rc_header_is_fetch_slot = rc_header_tag_in_range && rc_header_tag[2:0] < 3'd7;

    assign s_axis_rc_tready = (rc_state_reg == RC_IDLE) || (rc_state_reg == RC_DRAIN);

    always @(posedge clk) begin
        desc_push_valid_reg    <= 1'b0;
        slot_free_valid_reg    <= 1'b0;
        slot_update_valid_reg  <= 1'b0;
        fence_done_pulse_reg   <= 1'b0;
        status_error_cor_reg   <= 1'b0;
        status_error_uncor_reg <= 1'b0;
        rc_desc_complete_reg   <= 1'b0;
        rc_payload_done_reg    <= 1'b0;
        rc_desc_next_reg       <= rc_desc_data_reg;

        case (rc_state_reg)
            RC_IDLE: begin
                if (rc_fire) begin
                    rc_data_reg        <= s_axis_rc_tdata;
                    rc_keep_reg        <= s_axis_rc_tkeep;
                    rc_last_reg        <= s_axis_rc_tlast;
                    rc_header_beat_reg <= rc_need_header_reg;
                    rc_step_reg        <= 3'd0;

                    if (!rc_need_header_reg) begin
                        rc_state_reg <= RC_DW_SCAN;
                    end else begin
                        rc_slot_reg          <= rc_header_tag[2:0];
                        rc_is_fence_reg      <= rc_header_is_fence;
                        rc_known_slot_reg    <= rc_header_is_fetch_slot;
                        rc_error_code_reg    <= (rc_header_status == 4'd0) ? 4'hf : rc_header_status;
                        rc_desc_data_reg     <= 256'd0;
                        rc_desc_dw_count_reg <= 3'd0;

                        if (rc_header_is_fence) begin
                            if (s_axis_rc_tlast) begin
                                fence_done_pulse_reg <= 1'b1;
                                rc_state_reg         <= RC_IDLE;
                            end else begin
                                rc_state_reg <= RC_DRAIN;
                            end
                        end else if (!rc_header_is_fetch_slot) begin
                            status_error_uncor_reg <= 1'b1;
                            rc_state_reg           <= s_axis_rc_tlast ? RC_IDLE : RC_DRAIN;
                        end else begin
                            case (rc_header_tag[2:0])
                                3'd0: begin
                                    rc_sched_tag_reg    <= slot_sched_tag_0;
                                    rc_is_st_reg        <= slot_is_st_0;
                                    rc_descs_left_reg   <= slot_count_0;
                                    rc_dws_left_reg     <= slot_dws_left_0;
                                    rc_desc_dw_last_reg <= slot_is_st_0 ? 3'd3 : 3'd7;
                                end
                                3'd1: begin
                                    rc_sched_tag_reg    <= slot_sched_tag_1;
                                    rc_is_st_reg        <= slot_is_st_1;
                                    rc_descs_left_reg   <= slot_count_1;
                                    rc_dws_left_reg     <= slot_dws_left_1;
                                    rc_desc_dw_last_reg <= slot_is_st_1 ? 3'd3 : 3'd7;
                                end
                                3'd2: begin
                                    rc_sched_tag_reg    <= slot_sched_tag_2;
                                    rc_is_st_reg        <= slot_is_st_2;
                                    rc_descs_left_reg   <= slot_count_2;
                                    rc_dws_left_reg     <= slot_dws_left_2;
                                    rc_desc_dw_last_reg <= slot_is_st_2 ? 3'd3 : 3'd7;
                                end
                                3'd3: begin
                                    rc_sched_tag_reg    <= slot_sched_tag_3;
                                    rc_is_st_reg        <= slot_is_st_3;
                                    rc_descs_left_reg   <= slot_count_3;
                                    rc_dws_left_reg     <= slot_dws_left_3;
                                    rc_desc_dw_last_reg <= slot_is_st_3 ? 3'd3 : 3'd7;
                                end
                                3'd4: begin
                                    rc_sched_tag_reg    <= slot_sched_tag_4;
                                    rc_is_st_reg        <= slot_is_st_4;
                                    rc_descs_left_reg   <= slot_count_4;
                                    rc_dws_left_reg     <= slot_dws_left_4;
                                    rc_desc_dw_last_reg <= slot_is_st_4 ? 3'd3 : 3'd7;
                                end
                                3'd5: begin
                                    rc_sched_tag_reg    <= slot_sched_tag_5;
                                    rc_is_st_reg        <= slot_is_st_5;
                                    rc_descs_left_reg   <= slot_count_5;
                                    rc_dws_left_reg     <= slot_dws_left_5;
                                    rc_desc_dw_last_reg <= slot_is_st_5 ? 3'd3 : 3'd7;
                                end
                                default: begin
                                    rc_sched_tag_reg    <= slot_sched_tag_6;
                                    rc_is_st_reg        <= slot_is_st_6;
                                    rc_descs_left_reg   <= slot_count_6;
                                    rc_dws_left_reg     <= slot_dws_left_6;
                                    rc_desc_dw_last_reg <= slot_is_st_6 ? 3'd3 : 3'd7;
                                end
                            endcase

                            if (rc_header_status != 4'd0) begin
                                status_error_uncor_reg <= 1'b1;
                                case (rc_header_tag[2:0])
                                    3'd0: rc_error_fill_left_reg <= slot_count_0;
                                    3'd1: rc_error_fill_left_reg <= slot_count_1;
                                    3'd2: rc_error_fill_left_reg <= slot_count_2;
                                    3'd3: rc_error_fill_left_reg <= slot_count_3;
                                    3'd4: rc_error_fill_left_reg <= slot_count_4;
                                    3'd5: rc_error_fill_left_reg <= slot_count_5;
                                    default: rc_error_fill_left_reg <= slot_count_6;
                                endcase
                                rc_state_reg <= s_axis_rc_tlast ? RC_ERROR_FILL : RC_DRAIN;
                            end else begin
                                rc_state_reg <= RC_DW_SCAN;
                            end
                        end
                    end
                end
            end

            RC_DW_SCAN: begin
                rc_dw_keep_reg = 1'b0;
                rc_dw_reg      = 32'd0;
                case (rc_step_reg)
                    3'd0: begin
                        rc_dw_keep_reg = rc_keep_reg[0];
                        rc_dw_reg      = rc_data_reg[31:0];
                    end
                    3'd1: begin
                        rc_dw_keep_reg = rc_keep_reg[1];
                        rc_dw_reg      = rc_data_reg[63:32];
                    end
                    3'd2: begin
                        rc_dw_keep_reg = rc_keep_reg[2];
                        rc_dw_reg      = rc_data_reg[95:64];
                    end
                    3'd3: begin
                        rc_dw_keep_reg = rc_keep_reg[3];
                        rc_dw_reg      = rc_data_reg[127:96];
                    end
                    3'd4: begin
                        rc_dw_keep_reg = rc_keep_reg[4];
                        rc_dw_reg      = rc_data_reg[159:128];
                    end
                    3'd5: begin
                        rc_dw_keep_reg = rc_keep_reg[5];
                        rc_dw_reg      = rc_data_reg[191:160];
                    end
                    3'd6: begin
                        rc_dw_keep_reg = rc_keep_reg[6];
                        rc_dw_reg      = rc_data_reg[223:192];
                    end
                    default: begin
                        rc_dw_keep_reg = rc_keep_reg[7];
                        rc_dw_reg      = rc_data_reg[255:224];
                    end
                endcase

                rc_desc_next_reg      = rc_desc_data_reg;
                rc_desc_complete_reg  = 1'b0;
                rc_payload_done_reg   = 1'b0;
                rc_dws_left_tmp_reg   = rc_dws_left_reg;
                rc_descs_left_tmp_reg = rc_descs_left_reg;

                if (rc_dw_keep_reg && (!rc_header_beat_reg || rc_step_reg >= 3'd3) && rc_dws_left_reg != 0 && rc_descs_left_reg != 0) begin
                    case (rc_desc_dw_count_reg)
                        3'd0: rc_desc_next_reg[31:0] = rc_dw_reg;
                        3'd1: rc_desc_next_reg[63:32] = rc_dw_reg;
                        3'd2: rc_desc_next_reg[95:64] = rc_dw_reg;
                        3'd3: rc_desc_next_reg[127:96] = rc_dw_reg;
                        3'd4: rc_desc_next_reg[159:128] = rc_dw_reg;
                        3'd5: rc_desc_next_reg[191:160] = rc_dw_reg;
                        3'd6: rc_desc_next_reg[223:192] = rc_dw_reg;
                        default: rc_desc_next_reg[255:224] = rc_dw_reg;
                    endcase

                    rc_dws_left_tmp_reg = rc_dws_left_reg - 1'b1;
                    rc_dws_left_reg <= rc_dws_left_tmp_reg;
                    rc_payload_done_reg = (rc_dws_left_tmp_reg == 8'd0);

                    if (rc_desc_dw_count_reg == rc_desc_dw_last_reg) begin
                        desc_push_valid_reg  <= 1'b1;
                        desc_push_tag_reg    <= rc_sched_tag_reg;
                        desc_push_error_reg  <= 4'd0;
                        desc_push_data_reg   <= rc_is_st_reg ? {{128{1'b0}}, rc_desc_next_reg[127:0]} : rc_desc_next_reg;
                        rc_desc_data_reg     <= 256'd0;
                        rc_desc_dw_count_reg <= 3'd0;
                        rc_descs_left_tmp_reg = rc_descs_left_reg - 1'b1;
                        rc_descs_left_reg <= rc_descs_left_tmp_reg;
                        rc_desc_complete_reg = 1'b1;
                    end else begin
                        rc_desc_data_reg     <= rc_desc_next_reg;
                        rc_desc_dw_count_reg <= rc_desc_dw_count_reg + 1'b1;
                    end
                end

                if (rc_step_reg == 3'd7) begin
                    if (rc_last_reg) begin
                        rc_need_header_reg <= 1'b1;
                        if (rc_payload_done_reg || rc_dws_left_tmp_reg == 0) begin
                            slot_free_valid_reg <= 1'b1;
                            slot_free_index_reg <= rc_slot_reg;
                            rc_state_reg        <= RC_IDLE;
                        end else begin
                            slot_update_valid_reg    <= 1'b1;
                            slot_update_index_reg    <= rc_slot_reg;
                            slot_update_count_reg    <= rc_descs_left_tmp_reg;
                            slot_update_dws_left_reg <= rc_dws_left_tmp_reg;
                            rc_state_reg             <= RC_UPDATE_WAIT;
                        end
                    end else begin
                        rc_need_header_reg <= 1'b0;
                        rc_state_reg       <= RC_IDLE;
                    end
                end else begin
                    rc_step_reg <= rc_step_reg + 1'b1;
                end
            end

            RC_DRAIN: begin
                if (rc_fire && s_axis_rc_tlast) begin
                    if (rc_is_fence_reg) begin
                        fence_done_pulse_reg <= 1'b1;
                        rc_state_reg         <= RC_IDLE;
                    end else if (rc_error_fill_left_reg != 0) begin
                        rc_state_reg <= RC_ERROR_FILL;
                    end else begin
                        if (rc_known_slot_reg) begin
                            slot_free_valid_reg <= 1'b1;
                            slot_free_index_reg <= rc_slot_reg;
                        end
                        rc_state_reg <= RC_IDLE;
                    end
                end
            end

            RC_ERROR_FILL: begin
                if (rc_error_fill_left_reg != 0) begin
                    desc_push_valid_reg    <= 1'b1;
                    desc_push_tag_reg      <= rc_sched_tag_reg;
                    desc_push_error_reg    <= rc_error_code_reg;
                    desc_push_data_reg     <= {DESC_WIDTH{1'b0}};
                    rc_error_fill_left_reg <= rc_error_fill_left_reg - 1'b1;
                end else begin
                    slot_free_valid_reg <= 1'b1;
                    slot_free_index_reg <= rc_slot_reg;
                    rc_state_reg        <= RC_IDLE;
                end
            end

            RC_UPDATE_WAIT: begin
                rc_state_reg <= RC_IDLE;
            end

            default: rc_state_reg <= RC_IDLE;
        endcase

        if (rst) begin
            rc_state_reg           <= RC_IDLE;
            rc_data_reg            <= 0;
            rc_keep_reg            <= 0;
            rc_last_reg            <= 1'b0;
            rc_header_beat_reg     <= 1'b0;
            rc_need_header_reg     <= 1'b1;
            rc_step_reg            <= 0;
            rc_slot_reg            <= 0;
            rc_is_fence_reg        <= 1'b0;
            rc_known_slot_reg      <= 1'b0;
            rc_sched_tag_reg       <= 0;
            rc_is_st_reg           <= 1'b0;
            rc_descs_left_reg      <= 0;
            rc_dws_left_reg        <= 0;
            rc_desc_dw_count_reg   <= 0;
            rc_desc_dw_last_reg    <= 3'd7;
            rc_desc_data_reg       <= 0;
            rc_error_code_reg      <= 0;
            rc_error_fill_left_reg <= 0;
            desc_push_valid_reg    <= 1'b0;
            slot_free_valid_reg    <= 1'b0;
            slot_update_valid_reg  <= 1'b0;
            fence_done_pulse_reg   <= 1'b0;
            status_error_cor_reg   <= 1'b0;
            status_error_uncor_reg <= 1'b0;
        end
    end


endmodule

`resetall
