// axis_dwconv_c2h_512_to_256_xpm_fifo_sync.v
// 512b (user/subsystem) -> 256b (QDMA IP) width down-converter for C2H stream
//
// Key change vs. the simple "hold" version:
//   - Add a Xilinx XPM_FIFO_SYNC (synchronous FIFO primitive) on the 512b ingress.
//   - This breaks combinational READY paths and provides elastic buffering, which
//     helps avoid timing/handshake corner cases when connected to QDMA.
//   - Uses READ_MODE="fwft" (First-Word-Fall-Through).
//
// Notes:
//   - s_axis_mty is only meaningful when s_axis_tlast=1; output m_axis_mty is only
//     asserted on the 256b beat where m_axis_tlast=1.
//   - ctrl_* sideband is replicated on both 256b beats (kept stable for the packet).
//
// Requires: Vivado XPM library (xpm_fifo_sync).
`timescale 1ns / 1ps
module axis_dwconv_c2h_512_to_256 (
    input wire aclk,
    input wire aresetn,

    // s_axis: from subsystem (512b)
    input  wire         s_axis_tvalid,
    input  wire [511:0] s_axis_tdata,
    input  wire [ 31:0] s_axis_tcrc,
    input  wire         s_axis_tlast,
    input  wire         s_axis_ctrl_marker,
    input  wire [  2:0] s_axis_ctrl_port_id,
    input  wire [  6:0] s_axis_ctrl_ecc,
    input  wire [ 15:0] s_axis_ctrl_len,
    input  wire [ 10:0] s_axis_ctrl_qid,
    input  wire         s_axis_ctrl_has_cmpt,
    input  wire [  5:0] s_axis_mty,            // empty bytes in last 512b beat
    output wire         s_axis_tready,

    // m_axis: to QDMA IP (256b)
    output wire         m_axis_tvalid,
    output wire [255:0] m_axis_tdata,
    output wire [ 31:0] m_axis_tcrc,
    output wire         m_axis_tlast,
    output wire         m_axis_ctrl_marker,
    output wire [  2:0] m_axis_ctrl_port_id,
    output wire [  6:0] m_axis_ctrl_ecc,
    output wire [ 15:0] m_axis_ctrl_len,
    output wire [ 10:0] m_axis_ctrl_qid,
    output wire         m_axis_ctrl_has_cmpt,
    output wire [  5:0] m_axis_mty,            // empty bytes in last 256b beat
    input  wire         m_axis_tready
);

    //--------------------------------------------------------------------------
    // 0) Reset for XPM FIFO
    //    NOTE: XPM_FIFO_SYNC requires rst to be synchronous to wr_clk.
    //    If your aresetn is asynchronous, please synchronize it before feeding here.
    //--------------------------------------------------------------------------
    // Asynchronous assert, synchronous deassert reset synchronizer for XPM FIFO
    // XPM_FIFO_SYNC expects rst synchronous to wr_clk (aclk). (See UG974/UG953)
    reg [1:0] rst_pipe;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) rst_pipe <= 2'b11;
        else rst_pipe <= {rst_pipe[0], 1'b0};
    end
    wire rst_sync = rst_pipe[1];  // active-high, synchronous to aclk

    //--------------------------------------------------------------------------
    // 1) XPM FIFO on 512b ingress (native interface)
    //    Pack {tdata,tcrc,tlast,ctrl*,mty} into a single wide word.
    //--------------------------------------------------------------------------
    localparam integer FIFO_WIDTH = 512 + 32 + 1 + 1 + 3 + 7 + 16 + 11 + 1 + 6;  // 590
    localparam integer FIFO_DEPTH = 32;  // power of two, >=16

    wire [FIFO_WIDTH-1:0] fifo_din;
    wire [FIFO_WIDTH-1:0] fifo_dout;

    // Pack order (MSB -> LSB)
    assign fifo_din = {
        s_axis_tdata,  // [589:78]
        s_axis_tcrc,  // [77:46]
        s_axis_tlast,  // [45]
        s_axis_ctrl_marker,  // [44]
        s_axis_ctrl_port_id,  // [43:41]
        s_axis_ctrl_ecc,  // [40:34]
        s_axis_ctrl_len,  // [33:18]
        s_axis_ctrl_qid,  // [17:7]
        s_axis_ctrl_has_cmpt,  // [6]
        s_axis_mty  // [5:0]
    };

    wire fifo_full, fifo_empty, fifo_data_valid;
    wire fifo_wr_rst_busy, fifo_rd_rst_busy;
    wire fifo_busy = rst_sync | fifo_wr_rst_busy | fifo_rd_rst_busy;

    // In FWFT mode, some tool versions require enabling data_valid via USE_ADV_FEATURES[12].
    // Be robust: treat FIFO output as having a word when either data_valid=1 OR empty=0.
    wire fifo_has_word = fifo_data_valid | (~fifo_empty);

    // AXIS-style write handshake into FIFO (READY only depends on FIFO space)
    assign s_axis_tready = (~rst_sync) & (~fifo_full) & (~fifo_busy);
    wire fifo_wr_en = (~rst_sync) & s_axis_tvalid & s_axis_tready;

    // Read enable is asserted when we "consume" one 512b word from FIFO into hold regs
    reg  fifo_rd_en;

    // Xilinx XPM synchronous FIFO primitive
    xpm_fifo_sync #(
        .DOUT_RESET_VALUE   ("0"),
        .ECC_MODE           ("no_ecc"),
        .FIFO_MEMORY_TYPE   ("block"),
        // FWFT mode => FIFO_READ_LATENCY must be 0
        .FIFO_READ_LATENCY  (0),
        .FIFO_WRITE_DEPTH   (FIFO_DEPTH),
        .FULL_RESET_VALUE   (0),
        .PROG_EMPTY_THRESH  (10),
        .PROG_FULL_THRESH   (10),
        .RD_DATA_COUNT_WIDTH($clog2(FIFO_DEPTH) + 1),
        .READ_DATA_WIDTH    (FIFO_WIDTH),
        .READ_MODE          ("fwft"),
        .SIM_ASSERT_CHK     (1),
        .USE_ADV_FEATURES   ("1707"),                  // set bit[12]=1 to enable data_valid,
        .WAKEUP_TIME        (0),
        .WR_DATA_COUNT_WIDTH($clog2(FIFO_DEPTH) + 1),
        .WRITE_DATA_WIDTH   (FIFO_WIDTH)
    ) u_c2h_ingress_fifo (
        .rst          (rst_sync),
        .wr_clk       (aclk),
        .din          (fifo_din),
        .wr_en        (fifo_wr_en),
        .full         (fifo_full),
        .wr_ack       (),
        .overflow     (),
        .wr_data_count(),
        .wr_rst_busy  (fifo_wr_rst_busy),

        .rd_en        (fifo_rd_en),
        .dout         (fifo_dout),
        .empty        (fifo_empty),
        .data_valid   (fifo_data_valid),
        .underflow    (),
        .rd_data_count(),
        .rd_rst_busy  (fifo_rd_rst_busy),

        .almost_full  (),
        .almost_empty (),
        .prog_full    (),
        .prog_empty   (),
        .dbiterr      (),
        .sbiterr      (),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .sleep        (1'b0)
    );

    //--------------------------------------------------------------------------
    // 2) 512 -> 2x256 splitter (registered hold word)
    //--------------------------------------------------------------------------
    reg          hold_v;
    reg  [511:0] hold_d;
    reg  [ 31:0] hold_c;
    reg          hold_last;
    reg          hold_marker;
    reg  [  2:0] hold_port;
    reg  [  6:0] hold_ecc;
    reg  [ 15:0] hold_len;
    reg  [ 10:0] hold_qid;
    reg          hold_has_cmpt;
    reg  [  5:0] hold_mty;

    reg          half_sel;  // 0: lower 256, 1: upper 256

    // For last 512b beat: valid_bytes = 64 - hold_mty
    // Need upper 256 beat if (not last) OR (last && valid_bytes > 32)
    wire         need_upper = (~hold_last) ? 1'b1 : (hold_mty < 6'd32);

    wire         out_fire = hold_v && m_axis_tready;

    // current output
    assign m_axis_tvalid = hold_v;
    assign m_axis_tdata  = (half_sel == 1'b0) ? hold_d[255:0] : hold_d[511:256];
    assign m_axis_tcrc   = hold_c;

    // TLAST placement
    wire out_last = (hold_last) && ((half_sel == 1'b1) ||  // upper half always last if we send it
    ((half_sel == 1'b0) && !need_upper)  // only lower half exists
    );
    assign m_axis_tlast         = out_last;

    // MTY for 256b: only meaningful when out_last=1
    // - if last ends in lower half: hold_mty in [32..63], mty_256 = hold_mty - 32
    // - if last ends in upper half: hold_mty in [0..31],  mty_256 = hold_mty
    assign m_axis_mty           = out_last ? ((half_sel == 1'b0) ? (hold_mty - 6'd32) : hold_mty) : 6'd0;

    // Replicate ctrl sideband every beat (keep stable for the packet)
    assign m_axis_ctrl_marker   = hold_marker;
    assign m_axis_ctrl_port_id  = hold_port;
    assign m_axis_ctrl_ecc      = hold_ecc;
    assign m_axis_ctrl_len      = hold_len;
    assign m_axis_ctrl_qid      = hold_qid;
    assign m_axis_ctrl_has_cmpt = hold_has_cmpt;

    // One 512b word is "done" when we fired the final 256b half
    wire last_half = (half_sel == 1'b1) || ((half_sel == 1'b0) && !need_upper);
    wire word_done = out_fire && last_half;

    // Load next 512b word from FIFO when:
    //   - we are idle (hold_v=0), OR
    //   - we just completed a word this cycle (word_done=1)
    // FIFO is FWFT: fifo_data_valid indicates fifo_dout is valid.
    wire want_load = (~rst_sync) && ((~hold_v) || word_done) && fifo_has_word && (~fifo_busy);

    // FIFO pop: assert rd_en when we take fifo_dout into hold regs
    always @(*) begin
        fifo_rd_en = want_load;
    end

    // Unpack FIFO word
    wire [511:0] nxt_d;
    wire [ 31:0] nxt_c;
    wire         nxt_last;
    wire         nxt_marker;
    wire [  2:0] nxt_port;
    wire [  6:0] nxt_ecc;
    wire [ 15:0] nxt_len;
    wire [ 10:0] nxt_qid;
    wire         nxt_has_cmpt;
    wire [  5:0] nxt_mty;

    assign {nxt_d, nxt_c, nxt_last, nxt_marker, nxt_port, nxt_ecc, nxt_len, nxt_qid, nxt_has_cmpt, nxt_mty} = fifo_dout;

    always @(posedge aclk) begin
        if (rst_sync) begin
            hold_v   <= 1'b0;
            half_sel <= 1'b0;
        end else begin
            // advance output half when fired (unless we are reloading a new word immediately)
            if (out_fire && hold_v && !want_load) begin
                if (half_sel == 1'b0) begin
                    if (need_upper) begin
                        half_sel <= 1'b1;  // go send upper half next
                    end else begin
                        hold_v   <= 1'b0;  // done (only lower half)
                        half_sel <= 1'b0;
                    end
                end else begin
                    // upper half sent -> done
                    hold_v   <= 1'b0;
                    half_sel <= 1'b0;
                end
            end else if (word_done && !want_load) begin
                // safety (covered above)
                hold_v   <= 1'b0;
                half_sel <= 1'b0;
            end

            // Load a new 512b word from FIFO (can happen same cycle as finishing previous word)
            if (want_load) begin
                hold_v        <= 1'b1;
                half_sel      <= 1'b0;
                hold_d        <= nxt_d;
                hold_c        <= nxt_c;
                hold_last     <= nxt_last;
                hold_marker   <= nxt_marker;
                hold_port     <= nxt_port;
                hold_ecc      <= nxt_ecc;
                hold_len      <= nxt_len;
                hold_qid      <= nxt_qid;
                hold_has_cmpt <= nxt_has_cmpt;
                hold_mty      <= nxt_mty;
            end
        end
    end

endmodule
