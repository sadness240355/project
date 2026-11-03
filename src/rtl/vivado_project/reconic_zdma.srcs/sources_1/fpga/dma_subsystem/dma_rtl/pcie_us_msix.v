/* SPDX-License-Identifier: MIT
 * PCIe MSI-X external-table interrupt controller.
 *
 * Synthesis-safe implementation for 512 queue/vector designs.
 *
 * Key changes from the previous inferred-RAM version:
 *   - The MSI-X table is a single explicit 128-bit x 512 XPM true dual-port RAM.
 *   - Port A is used only by BAR4 CQ table accesses.
 *   - Port B is used only by the MSI-X issue engine.
 *   - The table is no longer inferred from four reg arrays with multiple read ports.
 *   - Incoming queue IRQs are sampled and converted to rising-edge events, MSI-like.
 */

`resetall 
`timescale 1ns / 1ps 
`default_nettype none
module pcie_us_msix #(
    parameter VECTOR_COUNT = 512,
    parameter AXIS_PCIE_DATA_WIDTH = 256,
    parameter AXIS_PCIE_KEEP_WIDTH = (AXIS_PCIE_DATA_WIDTH / 32),
    parameter AXIS_PCIE_CQ_USER_WIDTH = 88,
    parameter AXIS_PCIE_CC_USER_WIDTH = 33,
    parameter BAR4_ADDR_WIDTH = 16,
    parameter PBA_OFFSET = 32'h0000_8000,
    parameter TABLE_READ_LATENCY = 2
) (
    input wire clk,
    input wire rst,

    input wire        msix_irq_valid,
    input wire [8:0]  msix_irq_qid,
    input wire [9:0]  active_vector_count,

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

    input  wire [ 3:0] cfg_interrupt_msix_enable,
    input  wire [ 3:0] cfg_interrupt_msix_mask,
    output wire [63:0] cfg_interrupt_msix_address,
    output wire [31:0] cfg_interrupt_msix_data,
    output wire        cfg_interrupt_msix_int,
    output wire [ 1:0] cfg_interrupt_msix_vec_pending,
    input  wire        cfg_interrupt_msix_vec_pending_status,
    input  wire        cfg_interrupt_msi_sent,
    input  wire        cfg_interrupt_msi_fail,
    output wire [ 7:0] cfg_interrupt_msi_function_number,

    output wire status_error_cor,
    output wire status_error_uncor
);

    initial begin
        if (VECTOR_COUNT != 512) begin
            $error("pcie_us_msix synth-safe version is intended for VECTOR_COUNT=512 (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_DATA_WIDTH != 256 || AXIS_PCIE_KEEP_WIDTH != 8) begin
            $error("pcie_us_msix direct-CQ version expects a 256-bit PCIe AXIS CQ/CC interface (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_CQ_USER_WIDTH != 88 && AXIS_PCIE_CQ_USER_WIDTH != 85) begin
            $error("pcie_us_msix direct-CQ version expects CQ tuser width 85 or 88 (instance %m)");
            $finish;
        end
        if (AXIS_PCIE_CC_USER_WIDTH != 33) begin
            $error("pcie_us_msix direct-CQ version expects CC tuser width 33 (instance %m)");
            $finish;
        end
        if (BAR4_ADDR_WIDTH < 16 || BAR4_ADDR_WIDTH > 32) begin
            $error("BAR4_ADDR_WIDTH must be in the range 16..32 (instance %m)");
            $finish;
        end
    end

    localparam VECTOR_INDEX_WIDTH = 9;
    localparam [9:0] VECTOR_COUNT_VALUE = VECTOR_COUNT;
    localparam PBA_WORD_COUNT = 16;
    localparam PBA_WORD_INDEX_WIDTH = 4;
    localparam integer TABLE_ENTRY_WIDTH = 128;
    localparam integer TABLE_ENTRY_BYTES = TABLE_ENTRY_WIDTH / 8;
    localparam integer TABLE_BYTES = VECTOR_COUNT * 16;
    localparam [31:0] TABLE_BYTES_ADDR = TABLE_BYTES;
    localparam [31:0] PBA_END_ADDR = PBA_OFFSET + PBA_WORD_COUNT * 4;
    localparam [31:0] BAR4_ADDR_MASK = 32'hffff_ffff >> (32 - BAR4_ADDR_WIDTH);

    localparam [3:0] REQ_MEM_READ = 4'b0000, REQ_MEM_WRITE = 4'b0001, REQ_IO_READ = 4'b0010, REQ_IO_WRITE = 4'b0011;

    localparam [2:0] CPL_STATUS_SC = 3'b000, CPL_STATUS_UR = 3'b001, CPL_STATUS_CA = 3'b100;

    localparam [1:0] CQ_STATE_IDLE = 2'd0, CQ_STATE_WAIT_END = 2'd1, CQ_STATE_READ_WAIT = 2'd2;

    // MSI-X issue engine.  The original implementation scanned one vector bit
    // per cycle.  This version scans one 32-bit PBA word per cycle and encodes
    // the selected word in a second registered stage.  Worst-case search drops
    // from 512 cycles to 16 word scans, while avoiding a 512-bit combinational
    // priority encoder.
    localparam [2:0] MSIX_STATE_SCAN = 3'd0, MSIX_STATE_SCAN_WORD = 3'd1, MSIX_STATE_LOOKUP_WAIT = 3'd2, MSIX_STATE_WAIT = 3'd3;

    localparam [3:0] TABLE_READ_LATENCY_CYCLES = TABLE_READ_LATENCY;

    reg  [                        1:0] cq_state_reg = CQ_STATE_IDLE;
    reg  [                        3:0] cq_read_count_reg = 4'd0;

    reg  [                       15:0] cpl_requester_id_reg = 16'd0;
    reg  [                        7:0] cpl_tag_reg = 8'd0;
    reg  [                        2:0] cpl_tc_reg = 3'd0;
    reg  [                        2:0] cpl_attr_reg = 3'd0;
    reg  [                        3:0] cpl_first_be_reg = 4'hf;
    reg  [                       31:0] cpl_addr_reg = 32'd0;
    reg  [                        2:0] cpl_status_reg = CPL_STATUS_SC;
    reg                                cpl_has_data_reg = 1'b0;

    reg  [                        1:0] read_word_reg = 2'd0;
    reg                                read_is_table_reg = 1'b0;
    reg                                read_is_pba_reg = 1'b0;
    reg                                read_ctrl_written_reg = 1'b0;
    reg  [   PBA_WORD_INDEX_WIDTH-1:0] read_pba_word_reg = {PBA_WORD_INDEX_WIDTH{1'b0}};

    reg                                wait_error_cpl_reg = 1'b0;
    reg  [                        2:0] wait_cpl_status_reg = CPL_STATUS_UR;
    reg  [                       15:0] wait_requester_id_reg = 16'd0;
    reg  [                        7:0] wait_tag_reg = 8'd0;
    reg  [                        2:0] wait_tc_reg = 3'd0;
    reg  [                        2:0] wait_attr_reg = 3'd0;
    reg  [                        3:0] wait_first_be_reg = 4'hf;
    reg  [                       31:0] wait_addr_reg = 32'd0;
    reg                                wait_has_data_reg = 1'b0;

    reg  [   AXIS_PCIE_DATA_WIDTH-1:0] cc_tdata_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    reg  [   AXIS_PCIE_KEEP_WIDTH-1:0] cc_tkeep_reg = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
    reg                                cc_tvalid_reg = 1'b0;
    reg                                cc_tlast_reg = 1'b0;
    reg  [AXIS_PCIE_CC_USER_WIDTH-1:0] cc_tuser_reg = {AXIS_PCIE_CC_USER_WIDTH{1'b0}};

    reg                                status_error_cor_reg = 1'b0;
    reg                                status_error_uncor_reg = 1'b0;

    /* One explicit MSI-X table RAM, entry layout:
     * [ 31:  0] Message Address Low
     * [ 63: 32] Message Address High
     * [ 95: 64] Message Data
     * [127: 96] Vector Control
     */
    reg  [     VECTOR_INDEX_WIDTH-1:0] table_a_addr_reg = {VECTOR_INDEX_WIDTH{1'b0}};
    reg                                table_a_en_reg = 1'b0;
    reg  [      TABLE_ENTRY_BYTES-1:0] table_a_we_reg = {TABLE_ENTRY_BYTES{1'b0}};
    reg  [      TABLE_ENTRY_WIDTH-1:0] table_a_din_reg = {TABLE_ENTRY_WIDTH{1'b0}};
    wire [      TABLE_ENTRY_WIDTH-1:0] table_a_dout;

    reg  [     VECTOR_INDEX_WIDTH-1:0] table_b_addr_reg = {VECTOR_INDEX_WIDTH{1'b0}};
    reg                                table_b_en_reg = 1'b0;
    wire [      TABLE_ENTRY_WIDTH-1:0] table_b_dout;

    /* XPM RAM resets to zero.  ctrl_written=0 means vector is treated as
     * masked and BAR4 reads of Vector Control return bit0=1 until the host
     * explicitly writes that word.
     */
    reg  [           VECTOR_COUNT-1:0] table_ctrl_written_reg = {VECTOR_COUNT{1'b0}};
    // Keep reset fanout local to sixteen 32-vector groups. The PCIe hard-block
    // reset previously drove all 512 table-valid bits across the device.
    // Pause CQ admission during the one extra reset-release clock, so no new
    // control-table write can race a group being cleared.
    (* dont_touch = "yes" *) reg [PBA_WORD_COUNT-1:0] table_reset_bank_reg = {PBA_WORD_COUNT{1'b1}};
    always @(posedge clk)
        table_reset_bank_reg <= {PBA_WORD_COUNT{rst}};
    integer reset_bank_i;


    xpm_memory_tdpram #(
        .ADDR_WIDTH_A(VECTOR_INDEX_WIDTH),
        .ADDR_WIDTH_B(VECTOR_INDEX_WIDTH),
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
        .MEMORY_SIZE(TABLE_ENTRY_WIDTH * VECTOR_COUNT),
        .MESSAGE_CONTROL(0),
        .READ_DATA_WIDTH_A(TABLE_ENTRY_WIDTH),
        .READ_DATA_WIDTH_B(TABLE_ENTRY_WIDTH),
        .READ_LATENCY_A(TABLE_READ_LATENCY),
        .READ_LATENCY_B(TABLE_READ_LATENCY),
        .READ_RESET_VALUE_A("0"),
        .READ_RESET_VALUE_B("0"),
        .RST_MODE_A("SYNC"),
        .RST_MODE_B("SYNC"),
        .USE_EMBEDDED_CONSTRAINT(0),
        .USE_MEM_INIT(0),
        .USE_MEM_INIT_MMI(0),
        .WAKEUP_TIME("disable_sleep"),
        .WRITE_DATA_WIDTH_A(TABLE_ENTRY_WIDTH),
        .WRITE_DATA_WIDTH_B(TABLE_ENTRY_WIDTH),
        .WRITE_MODE_A("no_change"),
        .WRITE_MODE_B("no_change")
    ) msix_table_ram_inst (
        .clka          (clk),
        .rsta          (rst),
        .ena           (table_a_en_reg),
        .regcea        (1'b1),
        .wea           (table_a_we_reg),
        .addra         (table_a_addr_reg),
        .dina          (table_a_din_reg),
        .douta         (table_a_dout),
        .clkb          (clk),
        .rstb          (rst),
        .enb           (table_b_en_reg),
        .regceb        (1'b1),
        .web           ({TABLE_ENTRY_BYTES{1'b0}}),
        .addrb         (table_b_addr_reg),
        .dinb          ({TABLE_ENTRY_WIDTH{1'b0}}),
        .doutb         (table_b_dout),
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

    reg  [                    31:0] pba_word_reg                                       [0:PBA_WORD_COUNT-1];

    reg                              msix_irq_valid_reg = 1'b0;
    reg  [  VECTOR_INDEX_WIDTH-1:0] msix_irq_qid_reg = {VECTOR_INDEX_WIDTH{1'b0}};

    reg  [                     2:0] msix_state_reg = MSIX_STATE_SCAN;
    reg  [  VECTOR_INDEX_WIDTH-1:0] scan_vec_reg = {VECTOR_INDEX_WIDTH{1'b0}};
    reg  [PBA_WORD_INDEX_WIDTH-1:0] scan_word_index_reg = {PBA_WORD_INDEX_WIDTH{1'b0}};
    reg  [                    31:0] scan_word_data_reg = 32'd0;
    reg  [  VECTOR_INDEX_WIDTH-1:0] lookup_vec_reg = {VECTOR_INDEX_WIDTH{1'b0}};
    reg  [  VECTOR_INDEX_WIDTH-1:0] active_vec_reg = {VECTOR_INDEX_WIDTH{1'b0}};
    reg  [                     3:0] msix_lookup_count_reg = 4'd0;

    reg  [                    63:0] msix_address_reg = 64'd0;
    reg  [                    31:0] msix_data_reg = 32'd0;
    reg                             msix_int_reg = 1'b0;

    assign s_axis_cq_tready                  = !rst && !table_reset_bank_reg[0] &&
        ((cq_state_reg == CQ_STATE_IDLE) ? !cc_tvalid_reg : (cq_state_reg == CQ_STATE_WAIT_END));

    assign m_axis_cc_tdata                   = cc_tdata_reg;
    assign m_axis_cc_tkeep                   = cc_tkeep_reg;
    assign m_axis_cc_tvalid                  = cc_tvalid_reg;
    assign m_axis_cc_tlast                   = cc_tlast_reg;
    assign m_axis_cc_tuser                   = cc_tuser_reg;

    assign cfg_interrupt_msix_address        = msix_address_reg;
    assign cfg_interrupt_msix_data           = msix_data_reg;
    assign cfg_interrupt_msix_int            = msix_int_reg;

    assign cfg_interrupt_msix_vec_pending    = 2'b00;
    assign cfg_interrupt_msi_function_number = 8'd0;

    assign status_error_cor                  = status_error_cor_reg;
    assign status_error_uncor                = status_error_uncor_reg;

    wire                            msix_function_enabled = cfg_interrupt_msix_enable[0];
    wire                            msix_function_masked = cfg_interrupt_msix_mask[0];
    wire [                     9:0] active_vector_count_clamped = (active_vector_count == 0) ? VECTOR_COUNT_VALUE : (active_vector_count > VECTOR_COUNT_VALUE) ? VECTOR_COUNT_VALUE : active_vector_count;
    wire [                     4:0] active_word_count = active_vector_count_clamped[4:0] == 5'd0 ? active_vector_count_clamped[9:5] : active_vector_count_clamped[9:5] + 1'b1;

    wire [PBA_WORD_INDEX_WIDTH-1:0] scan_word = scan_vec_reg[8:5];

    wire [PBA_WORD_INDEX_WIDTH-1:0] lookup_word = lookup_vec_reg[8:5];
    wire [                     4:0] lookup_bit = lookup_vec_reg[4:0];
    wire                            lookup_pending = pba_word_reg[lookup_word][lookup_bit];

    wire [PBA_WORD_INDEX_WIDTH-1:0] active_word = active_vec_reg[8:5];
    wire [                     4:0] active_bit = active_vec_reg[4:0];

    wire                            cq_fire = s_axis_cq_tvalid && s_axis_cq_tready;
    wire [                    31:0] cq_addr_wire = s_axis_cq_tdata[31:0] & BAR4_ADDR_MASK;
    wire [                     3:0] cq_type_wire = s_axis_cq_tdata[78:75];
    wire [                    10:0] cq_dword_count_wire = s_axis_cq_tdata[74:64];

    wire [PBA_WORD_INDEX_WIDTH-1:0] scan_word_index_next_active =
        ({1'b0, scan_word_index_reg} >= active_word_count - 1'b1) ? {PBA_WORD_INDEX_WIDTH{1'b0}} :
        (scan_word_index_reg + 1'b1);
    wire [PBA_WORD_INDEX_WIDTH-1:0] lookup_word_next_active =
        ({1'b0, lookup_vec_reg[8:5]} >= active_word_count - 1'b1) ? {PBA_WORD_INDEX_WIDTH{1'b0}} :
        (lookup_vec_reg[8:5] + 1'b1);
    wire [PBA_WORD_INDEX_WIDTH-1:0] active_word_next_active =
        ({1'b0, active_vec_reg[8:5]} >= active_word_count - 1'b1) ? {PBA_WORD_INDEX_WIDTH{1'b0}} :
        (active_vec_reg[8:5] + 1'b1);

    wire [PBA_WORD_INDEX_WIDTH-1:0] scan_word_sel_comb =
        ({1'b0, scan_word} < active_word_count) ? scan_word : {PBA_WORD_INDEX_WIDTH{1'b0}};
    reg [31:0] scan_word_active_mask_comb;
    reg [4:0] scan_word_first_one_comb;
    reg scan_word_first_one_found_comb;
    integer mask_i;
    integer first_i;
    reg [9:0] scan_mask_bit_index_comb;

    always @* begin
        scan_word_active_mask_comb = 32'd0;
        for (mask_i = 0; mask_i < 32; mask_i = mask_i + 1) begin
            scan_mask_bit_index_comb = {scan_word_sel_comb, 5'd0} + mask_i[9:0];
            if (scan_mask_bit_index_comb < active_vector_count_clamped) begin
                scan_word_active_mask_comb[mask_i] = 1'b1;
            end
        end

        scan_word_first_one_comb = 5'd0;
        scan_word_first_one_found_comb = 1'b0;
        for (first_i = 0; first_i < 32; first_i = first_i + 1) begin
            if (!scan_word_first_one_found_comb && scan_word_data_reg[first_i]) begin
                scan_word_first_one_comb = first_i[4:0];
                scan_word_first_one_found_comb = 1'b1;
            end
        end
    end

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

    task table_write_word;
        input [31:0] addr;
        input [31:0] wdata;
        input [3:0] wstrb;
        reg [VECTOR_INDEX_WIDTH-1:0] vec;
        reg [                   1:0] word;
        begin
            vec  = addr[12:4];
            word = addr[3:2];
            if (addr < TABLE_BYTES_ADDR) begin
                table_a_addr_reg <= vec;
                table_a_en_reg   <= 1'b1;
                table_a_din_reg  <= {TABLE_ENTRY_WIDTH{1'b0}};
                table_a_we_reg   <= {TABLE_ENTRY_BYTES{1'b0}};
                case (word)
                    2'd0: begin
                        table_a_din_reg[31:0] <= wdata;
                        table_a_we_reg[3:0]   <= wstrb;
                    end
                    2'd1: begin
                        table_a_din_reg[63:32] <= wdata;
                        table_a_we_reg[7:4]    <= wstrb;
                    end
                    2'd2: begin
                        table_a_din_reg[95:64] <= wdata;
                        table_a_we_reg[11:8]   <= wstrb;
                    end
                    2'd3: begin
                        table_a_din_reg[127:96]     <= wdata;
                        table_a_we_reg[15:12]       <= wstrb;
                        table_ctrl_written_reg[vec] <= 1'b1;
                    end
                endcase
            end
        end
    endtask

    integer                            wi;
    integer                            pba_mask_i;
    reg     [                    31:0] pba_update_word;
    reg     [                     9:0] pba_mask_bit_index;
    reg     [                    31:0] cq_addr;
    reg     [                    31:0] write_data;
    reg     [  VECTOR_INDEX_WIDTH-1:0] read_vec;
    reg     [                    31:0] read_pba_byte_off;
    reg     [                    31:0] read_data_selected;

    always @(posedge clk) begin
        table_a_en_reg         <= 1'b0;
        table_a_we_reg         <= {TABLE_ENTRY_BYTES{1'b0}};
        table_a_din_reg        <= {TABLE_ENTRY_WIDTH{1'b0}};
        table_b_en_reg         <= 1'b0;

        status_error_cor_reg   <= 1'b0;
        status_error_uncor_reg <= 1'b0;
        msix_int_reg           <= 1'b0;

        msix_irq_valid_reg     <= msix_irq_valid;
        msix_irq_qid_reg       <= msix_irq_qid;

        if (cc_tvalid_reg && m_axis_cc_tready) begin
            cc_tvalid_reg <= 1'b0;
        end

        for (wi = 0; wi < PBA_WORD_COUNT; wi = wi + 1) begin
            pba_update_word = pba_word_reg[wi];
            for (pba_mask_i = 0; pba_mask_i < 32; pba_mask_i = pba_mask_i + 1) begin
                pba_mask_bit_index = {wi[PBA_WORD_INDEX_WIDTH-1:0], 5'd0} + pba_mask_i[9:0];
                if (pba_mask_bit_index >= active_vector_count_clamped) begin
                    pba_update_word[pba_mask_i] = 1'b0;
                end
            end
            if (msix_state_reg == MSIX_STATE_WAIT && cfg_interrupt_msi_sent && active_word == wi[PBA_WORD_INDEX_WIDTH-1:0]) begin
                pba_update_word[active_bit] = 1'b0;
            end
            if (msix_irq_valid_reg && (msix_irq_qid_reg < active_vector_count_clamped) && (msix_irq_qid_reg[8:5] == wi[PBA_WORD_INDEX_WIDTH-1:0])) begin
                pba_update_word[msix_irq_qid_reg[4:0]] = 1'b1;
            end
            pba_word_reg[wi] <= pba_update_word;
        end

        case (cq_state_reg)
            CQ_STATE_IDLE: begin
                if (cq_fire) begin
                    cq_addr    = cq_addr_wire;
                    write_data = s_axis_cq_tdata[159:128];

                    cpl_requester_id_reg <= s_axis_cq_tdata[95:80];
                    cpl_tag_reg          <= s_axis_cq_tdata[103:96];
                    cpl_tc_reg           <= s_axis_cq_tdata[123:121];
                    cpl_attr_reg         <= s_axis_cq_tdata[126:124];
                    cpl_first_be_reg     <= s_axis_cq_tuser[3:0];
                    cpl_addr_reg         <= cq_addr;

                    if (cq_type_wire == REQ_MEM_READ || cq_type_wire == REQ_IO_READ) begin
                        if (s_axis_cq_tlast && cq_dword_count_wire == 11'd1) begin
                            read_vec          = cq_addr[12:4];
                            read_pba_byte_off = cq_addr - PBA_OFFSET;

                            read_word_reg         <= cq_addr[3:2];
                            read_is_table_reg     <= (cq_addr < TABLE_BYTES_ADDR);
                            read_is_pba_reg       <= (cq_addr >= PBA_OFFSET) && (cq_addr < PBA_END_ADDR);
                            read_pba_word_reg     <= read_pba_byte_off[5:2];
                            read_ctrl_written_reg <= table_ctrl_written_reg[read_vec];
                            table_a_addr_reg      <= read_vec;
                            table_a_en_reg        <= (cq_addr < TABLE_BYTES_ADDR);
                            cpl_status_reg        <= CPL_STATUS_SC;
                            cpl_has_data_reg      <= 1'b1;
                            cq_read_count_reg     <= TABLE_READ_LATENCY_CYCLES;
                            cq_state_reg          <= CQ_STATE_READ_WAIT;
                        end else begin
                            status_error_cor_reg <= 1'b1;
                            if (s_axis_cq_tlast) begin
                                build_completion(CPL_STATUS_CA, s_axis_cq_tdata[95:80], s_axis_cq_tdata[103:96], s_axis_cq_tdata[123:121], s_axis_cq_tdata[126:124], s_axis_cq_tuser[3:0], cq_addr, 32'd0, 1'b0);
                            end else begin
                                wait_error_cpl_reg    <= 1'b1;
                                wait_cpl_status_reg   <= CPL_STATUS_CA;
                                wait_requester_id_reg <= s_axis_cq_tdata[95:80];
                                wait_tag_reg          <= s_axis_cq_tdata[103:96];
                                wait_tc_reg           <= s_axis_cq_tdata[123:121];
                                wait_attr_reg         <= s_axis_cq_tdata[126:124];
                                wait_first_be_reg     <= s_axis_cq_tuser[3:0];
                                wait_addr_reg         <= cq_addr;
                                wait_has_data_reg     <= 1'b0;
                                cq_state_reg          <= CQ_STATE_WAIT_END;
                            end
                        end
                    end else if (cq_type_wire == REQ_MEM_WRITE || cq_type_wire == REQ_IO_WRITE) begin
                        if (s_axis_cq_tlast && cq_dword_count_wire == 11'd1) begin
                            table_write_word(cq_addr, write_data, s_axis_cq_tuser[3:0]);
                            if (cq_type_wire == REQ_IO_WRITE) begin
                                build_completion(CPL_STATUS_SC, s_axis_cq_tdata[95:80], s_axis_cq_tdata[103:96], s_axis_cq_tdata[123:121], s_axis_cq_tdata[126:124], s_axis_cq_tuser[3:0], cq_addr, 32'd0, 1'b0);
                            end
                        end else begin
                            if (cq_type_wire == REQ_MEM_WRITE) begin
                                status_error_uncor_reg <= 1'b1;
                                if (!s_axis_cq_tlast) begin
                                    wait_error_cpl_reg <= 1'b0;
                                    cq_state_reg       <= CQ_STATE_WAIT_END;
                                end
                            end else begin
                                status_error_cor_reg <= 1'b1;
                                if (s_axis_cq_tlast) begin
                                    build_completion(CPL_STATUS_CA, s_axis_cq_tdata[95:80], s_axis_cq_tdata[103:96], s_axis_cq_tdata[123:121], s_axis_cq_tdata[126:124], s_axis_cq_tuser[3:0], cq_addr, 32'd0, 1'b0);
                                end else begin
                                    wait_error_cpl_reg    <= 1'b1;
                                    wait_cpl_status_reg   <= CPL_STATUS_CA;
                                    wait_requester_id_reg <= s_axis_cq_tdata[95:80];
                                    wait_tag_reg          <= s_axis_cq_tdata[103:96];
                                    wait_tc_reg           <= s_axis_cq_tdata[123:121];
                                    wait_attr_reg         <= s_axis_cq_tdata[126:124];
                                    wait_first_be_reg     <= s_axis_cq_tuser[3:0];
                                    wait_addr_reg         <= cq_addr;
                                    wait_has_data_reg     <= 1'b0;
                                    cq_state_reg          <= CQ_STATE_WAIT_END;
                                end
                            end
                        end
                    end else begin
                        status_error_cor_reg <= 1'b1;
                        if (s_axis_cq_tlast) begin
                            build_completion(CPL_STATUS_UR, s_axis_cq_tdata[95:80], s_axis_cq_tdata[103:96], s_axis_cq_tdata[123:121], s_axis_cq_tdata[126:124], s_axis_cq_tuser[3:0], cq_addr, 32'd0, 1'b0);
                        end else begin
                            wait_error_cpl_reg    <= 1'b1;
                            wait_cpl_status_reg   <= CPL_STATUS_UR;
                            wait_requester_id_reg <= s_axis_cq_tdata[95:80];
                            wait_tag_reg          <= s_axis_cq_tdata[103:96];
                            wait_tc_reg           <= s_axis_cq_tdata[123:121];
                            wait_attr_reg         <= s_axis_cq_tdata[126:124];
                            wait_first_be_reg     <= s_axis_cq_tuser[3:0];
                            wait_addr_reg         <= cq_addr;
                            wait_has_data_reg     <= 1'b0;
                            cq_state_reg          <= CQ_STATE_WAIT_END;
                        end
                    end
                end
            end
            CQ_STATE_READ_WAIT: begin
                if (cq_read_count_reg != 0) begin
                    cq_read_count_reg <= cq_read_count_reg - 1'b1;
                end else if (!cc_tvalid_reg) begin
                    read_data_selected = 32'd0;
                    if (read_is_table_reg) begin
                        case (read_word_reg)
                            2'd0: read_data_selected = table_a_dout[31:0];
                            2'd1: read_data_selected = table_a_dout[63:32];
                            2'd2: read_data_selected = table_a_dout[95:64];
                            2'd3: read_data_selected = read_ctrl_written_reg ? table_a_dout[127:96] : 32'h0000_0001;
                        endcase
                    end else if (read_is_pba_reg) begin
                        read_data_selected = pba_word_reg[read_pba_word_reg];
                    end
                    build_completion(cpl_status_reg, cpl_requester_id_reg, cpl_tag_reg, cpl_tc_reg, cpl_attr_reg, cpl_first_be_reg, cpl_addr_reg, read_data_selected, cpl_has_data_reg);
                    cq_state_reg <= CQ_STATE_IDLE;
                end
            end
            CQ_STATE_WAIT_END: begin
                if (cq_fire && s_axis_cq_tlast) begin
                    if (wait_error_cpl_reg && !cc_tvalid_reg) begin
                        build_completion(wait_cpl_status_reg, wait_requester_id_reg, wait_tag_reg, wait_tc_reg, wait_attr_reg, wait_first_be_reg, wait_addr_reg, 32'd0, wait_has_data_reg);
                    end
                    cq_state_reg <= CQ_STATE_IDLE;
                end
            end
            default: begin
                cq_state_reg <= CQ_STATE_IDLE;
            end
        endcase

        case (msix_state_reg)
            MSIX_STATE_SCAN: begin
                if (msix_function_enabled && !msix_function_masked) begin
                    scan_word_index_reg <= scan_word_sel_comb;
                    scan_word_data_reg  <= pba_word_reg[scan_word_sel_comb] & scan_word_active_mask_comb;
                    msix_state_reg      <= MSIX_STATE_SCAN_WORD;
                end
            end
            MSIX_STATE_SCAN_WORD: begin
                if (msix_function_enabled && !msix_function_masked && scan_word_data_reg != 32'd0) begin
                    lookup_vec_reg        <= {scan_word_index_reg, scan_word_first_one_comb};
                    table_b_addr_reg      <= {scan_word_index_reg, scan_word_first_one_comb};
                    table_b_en_reg        <= 1'b1;
                    msix_lookup_count_reg <= TABLE_READ_LATENCY_CYCLES;
                    msix_state_reg        <= MSIX_STATE_LOOKUP_WAIT;
                end else begin
                    scan_vec_reg   <= {scan_word_index_next_active, 5'd0};
                    msix_state_reg <= MSIX_STATE_SCAN;
                end
            end
            MSIX_STATE_LOOKUP_WAIT: begin
                if (msix_lookup_count_reg != 0) begin
                    msix_lookup_count_reg <= msix_lookup_count_reg - 1'b1;
                end else begin
                    if (msix_function_enabled && !msix_function_masked && lookup_pending && table_ctrl_written_reg[lookup_vec_reg] && !table_b_dout[96]) begin
                        active_vec_reg   <= lookup_vec_reg;
                        msix_address_reg <= table_b_dout[63:0];
                        msix_data_reg    <= table_b_dout[95:64];
                        msix_int_reg     <= 1'b1;
                        msix_state_reg   <= MSIX_STATE_WAIT;
                    end else begin
                        scan_vec_reg   <= {lookup_word_next_active, 5'd0};
                        msix_state_reg <= MSIX_STATE_SCAN;
                    end
                end
            end
            MSIX_STATE_WAIT: begin
                if (cfg_interrupt_msi_sent || cfg_interrupt_msi_fail) begin
                    scan_vec_reg   <= {active_word_next_active, 5'd0};
                    msix_state_reg <= MSIX_STATE_SCAN;
                end
            end
            default: begin
                msix_state_reg <= MSIX_STATE_SCAN;
            end
        endcase

        if (rst) begin
            for (wi = 0; wi < PBA_WORD_COUNT; wi = wi + 1) begin
                pba_word_reg[wi] <= 32'd0;
            end

            msix_irq_valid_reg     <= 1'b0;
            msix_irq_qid_reg       <= {VECTOR_INDEX_WIDTH{1'b0}};
            cq_state_reg           <= CQ_STATE_IDLE;
            cq_read_count_reg      <= 4'd0;
            cpl_requester_id_reg   <= 16'd0;
            cpl_tag_reg            <= 8'd0;
            cpl_tc_reg             <= 3'd0;
            cpl_attr_reg           <= 3'd0;
            cpl_first_be_reg       <= 4'hf;
            cpl_addr_reg           <= 32'd0;
            cpl_status_reg         <= CPL_STATUS_SC;
            cpl_has_data_reg       <= 1'b0;
            read_word_reg          <= 2'd0;
            read_is_table_reg      <= 1'b0;
            read_is_pba_reg        <= 1'b0;
            read_ctrl_written_reg  <= 1'b0;
            read_pba_word_reg      <= {PBA_WORD_INDEX_WIDTH{1'b0}};
            wait_error_cpl_reg     <= 1'b0;
            wait_cpl_status_reg    <= CPL_STATUS_UR;
            wait_requester_id_reg  <= 16'd0;
            wait_tag_reg           <= 8'd0;
            wait_tc_reg            <= 3'd0;
            wait_attr_reg          <= 3'd0;
            wait_first_be_reg      <= 4'hf;
            wait_addr_reg          <= 32'd0;
            wait_has_data_reg      <= 1'b0;
            cc_tdata_reg           <= {AXIS_PCIE_DATA_WIDTH{1'b0}};
            cc_tkeep_reg           <= {AXIS_PCIE_KEEP_WIDTH{1'b0}};
            cc_tvalid_reg          <= 1'b0;
            cc_tlast_reg           <= 1'b0;
            cc_tuser_reg           <= {AXIS_PCIE_CC_USER_WIDTH{1'b0}};
            status_error_cor_reg   <= 1'b0;
            status_error_uncor_reg <= 1'b0;
            table_a_addr_reg       <= {VECTOR_INDEX_WIDTH{1'b0}};
            table_a_en_reg         <= 1'b0;
            table_a_we_reg         <= {TABLE_ENTRY_BYTES{1'b0}};
            table_a_din_reg        <= {TABLE_ENTRY_WIDTH{1'b0}};
            table_b_addr_reg       <= {VECTOR_INDEX_WIDTH{1'b0}};
            table_b_en_reg         <= 1'b0;
            msix_state_reg         <= MSIX_STATE_SCAN;
            scan_vec_reg           <= {VECTOR_INDEX_WIDTH{1'b0}};
            scan_word_index_reg    <= {PBA_WORD_INDEX_WIDTH{1'b0}};
            scan_word_data_reg     <= 32'd0;
            lookup_vec_reg         <= {VECTOR_INDEX_WIDTH{1'b0}};
            active_vec_reg         <= {VECTOR_INDEX_WIDTH{1'b0}};
            msix_lookup_count_reg  <= 4'd0;
            msix_address_reg       <= 64'd0;
            msix_data_reg          <= 32'd0;
            msix_int_reg           <= 1'b0;
        end
        // This is the sole reset assignment for table_ctrl_written_reg.
        // It follows the ordinary per-vector write assignment in this same
        // sequential block; each group reset therefore has final priority.
        for (reset_bank_i = 0; reset_bank_i < PBA_WORD_COUNT; reset_bank_i = reset_bank_i + 1)
            if (table_reset_bank_reg[reset_bank_i])
                table_ctrl_written_reg[reset_bank_i*32 +: 32] <= 32'd0;
    end

    wire unused_s_axis_cq_tkeep = ^s_axis_cq_tkeep;
    wire unused_cfg_interrupt_msix_vec_pending_status = cfg_interrupt_msix_vec_pending_status;

endmodule

`resetall
