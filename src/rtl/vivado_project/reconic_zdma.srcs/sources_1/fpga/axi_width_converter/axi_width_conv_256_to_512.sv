`timescale 1ns/1ps

// ============================================================
// Small register FIFO for metadata only
// ============================================================
module axi_small_fifo #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 4
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 wr_en,
    input  wire [WIDTH-1:0]     din,
    output wire                 full,
    input  wire                 rd_en,
    output wire [WIDTH-1:0]     dout,
    output wire                 empty,
    output wire [31:0]          count
);
    function integer clog2;
        input integer value;
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            clog2 = (i == 0) ? 1 : i;
        end
    endfunction

    localparam integer AW = clog2(DEPTH);
    localparam integer CW = clog2(DEPTH + 1);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [AW-1:0]    wptr;
    reg [AW-1:0]    rptr;
    reg [CW-1:0]    cnt;

    integer k;

    wire do_write = wr_en && !full;
    wire do_read  = rd_en && !empty;

    assign full  = (cnt == DEPTH);
    assign empty = (cnt == 0);
    assign count = {{(32-CW){1'b0}},cnt};
    assign dout  = mem[rptr];

    always @(posedge clk) begin
        if (!rst_n) begin
            wptr <= {AW{1'b0}};
            rptr <= {AW{1'b0}};
            cnt  <= {CW{1'b0}};
            // Payload is unreachable while empty. Reset only ownership;
            // every occupied location is written before it is exposed.
        end else begin
            if (do_write)
                mem[wptr] <= din;

            case ({do_write, do_read})
                2'b10: begin
                    wptr <= (wptr == DEPTH-1) ? {AW{1'b0}} : (wptr + 1'b1);
                    cnt  <= cnt + 1'b1;
                end
                2'b01: begin
                    rptr <= (rptr == DEPTH-1) ? {AW{1'b0}} : (rptr + 1'b1);
                    cnt  <= cnt - 1'b1;
                end
                2'b11: begin
                    wptr <= (wptr == DEPTH-1) ? {AW{1'b0}} : (wptr + 1'b1);
                    rptr <= (rptr == DEPTH-1) ? {AW{1'b0}} : (rptr + 1'b1);
                end
                default: begin
                end
            endcase
        end
    end
endmodule

// ============================================================
// AXI4 width converter: SI=256-bit, MI=512-bit, synchronous
//
// Design focus:
// 1) Write: pack two 256b beats -> one 512b beat
// 2) Read : 512b return -> xpm_fifo_sync -> split to 256b
// 3) Outstanding depth = 3 (accepted bursts not yet fully responded)
// 4) Burst support: INCR burst path, max burst len = 256 on SI side
// 5) Timing-oriented structure, single writer per reg
//
// Practical assumptions of this RTL:
// - single clock domain
// - SI data width is fixed 256b, so s_awsize/s_arsize should be 3'b101
// - current implementation targets INCR bursts
// - SI start address should be 32-byte aligned
// - MI side is forced to one ID thread: m_awid/m_arid are tied to 0
// ============================================================
module axi_width_conv_256_to_512 #(
    parameter integer ADDR_WIDTH       = 64,
    parameter integer ID_WIDTH         = 4,
    parameter integer OUTSTANDING      = 3,
    parameter integer RDATA_FIFO_DEPTH = 128
) (
    input  wire                        aclk,
    input  wire                        aresetn,

    // -----------------------------
    // Slave side (256-bit)
    // -----------------------------
    input  wire [ID_WIDTH-1:0]         s_awid,
    input  wire [ADDR_WIDTH-1:0]       s_awaddr,
    input  wire [7:0]                  s_awlen,
    input  wire [2:0]                  s_awsize,
    input  wire [1:0]                  s_awburst,
    input  wire                        s_awlock,
    input  wire [3:0]                  s_awcache,
    input  wire [2:0]                  s_awprot,
    input  wire [3:0]                  s_awqos,
    input  wire [3:0]                  s_awregion,
    input  wire                        s_awvalid,
    output wire                        s_awready,

    input  wire [255:0]                s_wdata,
    input  wire [31:0]                 s_wstrb,
    input  wire                        s_wlast,
    input  wire                        s_wvalid,
    output wire                        s_wready,

    output wire [ID_WIDTH-1:0]         s_bid,
    output wire [1:0]                  s_bresp,
    output wire                        s_bvalid,
    input  wire                        s_bready,

    input  wire [ID_WIDTH-1:0]         s_arid,
    input  wire [ADDR_WIDTH-1:0]       s_araddr,
    input  wire [7:0]                  s_arlen,
    input  wire [2:0]                  s_arsize,
    input  wire [1:0]                  s_arburst,
    input  wire                        s_arlock,
    input  wire [3:0]                  s_arcache,
    input  wire [2:0]                  s_arprot,
    input  wire [3:0]                  s_arqos,
    input  wire [3:0]                  s_arregion,
    input  wire                        s_arvalid,
    output wire                        s_arready,

    output wire [ID_WIDTH-1:0]         s_rid,
    output wire [255:0]                s_rdata,
    output wire [1:0]                  s_rresp,
    output wire                        s_rlast,
    output wire                        s_rvalid,
    input  wire                        s_rready,

    // -----------------------------
    // Master side (512-bit)
    // -----------------------------
    output wire [ID_WIDTH-1:0]         m_awid,
    output wire [ADDR_WIDTH-1:0]       m_awaddr,
    output wire [7:0]                  m_awlen,
    output wire [2:0]                  m_awsize,
    output wire [1:0]                  m_awburst,
    output wire                        m_awlock,
    output wire [3:0]                  m_awcache,
    output wire [2:0]                  m_awprot,
    output wire [3:0]                  m_awqos,
    output wire [3:0]                  m_awregion,
    output wire                        m_awvalid,
    input  wire                        m_awready,

    output wire [511:0]                m_wdata,
    output wire [63:0]                 m_wstrb,
    output wire                        m_wlast,
    output wire                        m_wvalid,
    input  wire                        m_wready,

    input  wire [ID_WIDTH-1:0]         m_bid,
    input  wire [1:0]                  m_bresp,
    input  wire                        m_bvalid,
    output wire                        m_bready,

    output wire [ID_WIDTH-1:0]         m_arid,
    output wire [ADDR_WIDTH-1:0]       m_araddr,
    output wire [7:0]                  m_arlen,
    output wire [2:0]                  m_arsize,
    output wire [1:0]                  m_arburst,
    output wire                        m_arlock,
    output wire [3:0]                  m_arcache,
    output wire [2:0]                  m_arprot,
    output wire [3:0]                  m_arqos,
    output wire [3:0]                  m_arregion,
    output wire                        m_arvalid,
    input  wire                        m_arready,

    input  wire [ID_WIDTH-1:0]         m_rid,
    input  wire [511:0]                m_rdata,
    input  wire [1:0]                  m_rresp,
    input  wire                        m_rlast,
    input  wire                        m_rvalid,
    output wire                        m_rready
);

    function integer clog2;
        input integer value;
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            clog2 = (i == 0) ? 1 : i;
        end
    endfunction

    localparam integer WR_META_W = ID_WIDTH + ADDR_WIDTH + 8 + 8 + 2 + 1 + 4 + 3 + 4 + 4;
    localparam integer WR_ID_W   = ID_WIDTH;
    localparam integer RD_META_W = ID_WIDTH + 8 + 1;
    localparam integer RD_ISS_W  = ADDR_WIDTH + 8 + 2 + 1 + 4 + 3 + 4 + 4;
    localparam integer RFIFO_W   = 512 + 2 + 1;

    // ========================================================
    // Common derived values
    // ========================================================
    wire [7:0] s_awbeats  = s_awlen + 8'd1;
    wire [7:0] s_arbeats  = s_arlen + 8'd1;

    // Number of MI-side 512b beats after packing / before splitting
    wire [7:0] aw_mibeats = (s_awbeats + {7'd0, s_awaddr[5]} + 8'd1) >> 1;
    wire [7:0] ar_mibeats = (s_arbeats + {7'd0, s_araddr[5]} + 8'd1) >> 1;

    // ========================================================
    // Write metadata packing
    // ========================================================
    wire [WR_META_W-1:0] wr_meta_in = {
        s_awid,
        {s_awaddr[ADDR_WIDTH-1:6], 6'b0}, // align to 64B for MI
        s_awbeats,                        // SI beats
        aw_mibeats,                       // MI beats
        s_awburst,
        s_awaddr[5],                      // first half select: 0=low, 1=high
        s_awcache,
        s_awprot,
        s_awqos,
        s_awregion
    };

    // ========================================================
    // Read metadata packing
    // ========================================================
    wire [RD_META_W-1:0] rd_meta_in = {
        s_arid,
        s_arbeats,
        s_araddr[5]
    };

    wire [RD_ISS_W-1:0] rd_iss_in = {
        {s_araddr[ADDR_WIDTH-1:6], 6'b0}, // align to 64B for MI
        ar_mibeats,
        s_arburst,
        s_arlock,
        s_arcache,
        s_arprot,
        s_arqos,
        s_arregion
    };

    // ========================================================
    // Small FIFOs
    // ========================================================
    wire [WR_META_W-1:0] wr_wmeta_dout;
    wire [WR_META_W-1:0] wr_awmeta_dout;
    wire [WR_ID_W-1:0]   wr_bid_dout;
    wire [RD_META_W-1:0] rd_meta_dout;
    wire [RD_ISS_W-1:0]  rd_iss_dout;

    wire wr_wmeta_full, wr_wmeta_empty;
    wire wr_awmeta_full, wr_awmeta_empty;
    wire wr_bid_full,   wr_bid_empty;
    wire rd_meta_full,  rd_meta_empty;
    wire rd_iss_full,   rd_iss_empty;

    wire [31:0] wr_wmeta_count, wr_awmeta_count, wr_bid_count, rd_meta_count, rd_iss_count;

    wire wr_wmeta_pop;
    wire wr_awmeta_pop;
    wire wr_bid_pop;
    wire rd_meta_pop;
    wire rd_iss_pop;

    reg  [7:0] wr_pending_count;
    reg  [7:0] rd_pending_count;

    wire wr_aw_push = s_awvalid && s_awready;
    wire rd_ar_push = s_arvalid && s_arready;

    assign s_awready = (wr_pending_count < OUTSTANDING) &&
                       !wr_wmeta_full && !wr_awmeta_full && !wr_bid_full;

    assign s_arready = (rd_pending_count < OUTSTANDING) &&
                       !rd_meta_full && !rd_iss_full;

    axi_small_fifo #(.WIDTH(WR_META_W), .DEPTH(OUTSTANDING+1)) u_wr_wmeta_fifo (
        .clk   (aclk),
        .rst_n (aresetn),
        .wr_en (wr_aw_push),
        .din   (wr_meta_in),
        .full  (wr_wmeta_full),
        .rd_en (wr_wmeta_pop),
        .dout  (wr_wmeta_dout),
        .empty (wr_wmeta_empty),
        .count (wr_wmeta_count)
    );

    axi_small_fifo #(.WIDTH(WR_META_W), .DEPTH(OUTSTANDING+1)) u_wr_awmeta_fifo (
        .clk   (aclk),
        .rst_n (aresetn),
        .wr_en (wr_aw_push),
        .din   (wr_meta_in),
        .full  (wr_awmeta_full),
        .rd_en (wr_awmeta_pop),
        .dout  (wr_awmeta_dout),
        .empty (wr_awmeta_empty),
        .count (wr_awmeta_count)
    );

    axi_small_fifo #(.WIDTH(WR_ID_W), .DEPTH(OUTSTANDING+1)) u_wr_bid_fifo (
        .clk   (aclk),
        .rst_n (aresetn),
        .wr_en (wr_aw_push),
        .din   (s_awid),
        .full  (wr_bid_full),
        .rd_en (wr_bid_pop),
        .dout  (wr_bid_dout),
        .empty (wr_bid_empty),
        .count (wr_bid_count)
    );

    axi_small_fifo #(.WIDTH(RD_META_W), .DEPTH(OUTSTANDING+1)) u_rd_meta_fifo (
        .clk   (aclk),
        .rst_n (aresetn),
        .wr_en (rd_ar_push),
        .din   (rd_meta_in),
        .full  (rd_meta_full),
        .rd_en (rd_meta_pop),
        .dout  (rd_meta_dout),
        .empty (rd_meta_empty),
        .count (rd_meta_count)
    );

    axi_small_fifo #(.WIDTH(RD_ISS_W), .DEPTH(OUTSTANDING+1)) u_rd_issue_fifo (
        .clk   (aclk),
        .rst_n (aresetn),
        .wr_en (rd_ar_push),
        .din   (rd_iss_in),
        .full  (rd_iss_full),
        .rd_en (rd_iss_pop),
        .dout  (rd_iss_dout),
        .empty (rd_iss_empty),
        .count (rd_iss_count)
    );

    // ========================================================
    // Write address channel
    // ========================================================
    wire [ID_WIDTH-1:0]   wr_awmeta_id;
    wire [ADDR_WIDTH-1:0] wr_awmeta_addr;
    wire [7:0]            wr_awmeta_sbeats;
    wire [7:0]            wr_awmeta_mbeats;
    wire [1:0]            wr_awmeta_burst;
    wire                  wr_awmeta_half;
    wire [3:0]            wr_awmeta_cache;
    wire [2:0]            wr_awmeta_prot;
    wire [3:0]            wr_awmeta_qos;
    wire [3:0]            wr_awmeta_region;

    assign {
        wr_awmeta_id,
        wr_awmeta_addr,
        wr_awmeta_sbeats,
        wr_awmeta_mbeats,
        wr_awmeta_burst,
        wr_awmeta_half,
        wr_awmeta_cache,
        wr_awmeta_prot,
        wr_awmeta_qos,
        wr_awmeta_region
    } = wr_awmeta_dout;

    assign m_awid     = {ID_WIDTH{1'b0}};
    assign m_awaddr   = wr_awmeta_addr;
    assign m_awlen    = wr_awmeta_mbeats - 8'd1;
    assign m_awsize   = 3'b110; // 64 bytes / beat
    assign m_awburst  = wr_awmeta_burst;
    assign m_awlock   = 1'b0;   // exclusive path intentionally not implemented
    assign m_awcache  = wr_awmeta_cache;
    assign m_awprot   = wr_awmeta_prot;
    assign m_awqos    = wr_awmeta_qos;
    assign m_awregion = wr_awmeta_region;
    assign m_awvalid  = !wr_awmeta_empty;

    assign wr_awmeta_pop = m_awvalid && m_awready;

    // ========================================================
    // Write data path: 256b -> 512b
    // ========================================================
    wire [ID_WIDTH-1:0]   wr_wmeta_id;
    wire [ADDR_WIDTH-1:0] wr_wmeta_addr;
    wire [7:0]            wr_wmeta_sbeats;
    wire [7:0]            wr_wmeta_mbeats;
    wire [1:0]            wr_wmeta_burst;
    wire                  wr_wmeta_half;
    wire [3:0]            wr_wmeta_cache;
    wire [2:0]            wr_wmeta_prot;
    wire [3:0]            wr_wmeta_qos;
    wire [3:0]            wr_wmeta_region;

    assign {
        wr_wmeta_id,
        wr_wmeta_addr,
        wr_wmeta_sbeats,
        wr_wmeta_mbeats,
        wr_wmeta_burst,
        wr_wmeta_half,
        wr_wmeta_cache,
        wr_wmeta_prot,
        wr_wmeta_qos,
        wr_wmeta_region
    } = wr_wmeta_dout;

    reg        wr_w_active;
    reg [7:0]  wr_cur_sbeats_left;
    reg [7:0]  wr_cur_mbeats_left;
    reg        wr_cur_half;

    reg [511:0] wr_pack_data;
    reg [63:0]  wr_pack_strb;

    reg         m_wvalid_r;
    reg [511:0] m_wdata_r;
    reg [63:0]  m_wstrb_r;
    reg         m_wlast_r;

    wire wr_load_new_burst = !wr_w_active && !wr_wmeta_empty && !m_wvalid_r;
    assign wr_wmeta_pop    = wr_load_new_burst;

    assign s_wready = wr_w_active && !m_wvalid_r;

    assign m_wvalid = m_wvalid_r;
    assign m_wdata  = m_wdata_r;
    assign m_wstrb  = m_wstrb_r;
    assign m_wlast  = m_wlast_r;

    wire s_w_hs = s_wvalid && s_wready;
    wire m_w_hs = m_wvalid && m_wready;

    reg [511:0] next_pack_data;
    reg [63:0]  next_pack_strb;
    reg         emit_now;
    reg         last_mi_word;

    always @(*) begin
        next_pack_data = wr_pack_data;
        next_pack_strb = wr_pack_strb;
        emit_now       = 1'b0;
        last_mi_word   = 1'b0;

        if (s_w_hs) begin
            if (!wr_cur_half) begin
                next_pack_data[255:0] = s_wdata;
                next_pack_strb[31:0]  = s_wstrb;
                if (wr_cur_sbeats_left == 8'd1) begin
                    next_pack_data[511:256] = 256'd0;
                    next_pack_strb[63:32]   = 32'd0;
                    emit_now = 1'b1;
                end
            end else begin
                next_pack_data[511:256] = s_wdata;
                next_pack_strb[63:32]   = s_wstrb;
                emit_now = 1'b1;
            end

            if (emit_now && (wr_cur_mbeats_left == 8'd1))
                last_mi_word = 1'b1;
        end
    end

    wire wr_b_hs = s_bvalid && s_bready;

    always @(posedge aclk) begin
        if (!aresetn) begin
            wr_pending_count   <= 8'd0;
            wr_w_active        <= 1'b0;
            wr_cur_sbeats_left <= 8'd0;
            wr_cur_mbeats_left <= 8'd0;
            wr_cur_half        <= 1'b0;
            wr_pack_data       <= 512'd0;
            wr_pack_strb       <= 64'd0;
            m_wvalid_r         <= 1'b0;
            m_wdata_r          <= 512'd0;
            m_wstrb_r          <= 64'd0;
            m_wlast_r          <= 1'b0;
        end else begin
            case ({wr_aw_push, wr_b_hs})
                2'b10: wr_pending_count <= wr_pending_count + 1'b1;
                2'b01: wr_pending_count <= wr_pending_count - 1'b1;
                default: begin
                end
            endcase

            if (m_w_hs)
                m_wvalid_r <= 1'b0;

            if (wr_load_new_burst) begin
                wr_w_active        <= 1'b1;
                wr_cur_sbeats_left <= wr_wmeta_sbeats;
                wr_cur_mbeats_left <= wr_wmeta_mbeats;
                wr_cur_half        <= wr_wmeta_half;
                wr_pack_data       <= 512'd0;
                wr_pack_strb       <= 64'd0;
            end

            if (s_w_hs) begin
                wr_pack_data <= next_pack_data;
                wr_pack_strb <= next_pack_strb;

                if (emit_now) begin
                    m_wvalid_r <= 1'b1;
                    m_wdata_r  <= next_pack_data;
                    m_wstrb_r  <= next_pack_strb;
                    m_wlast_r  <= last_mi_word;

                    wr_pack_data       <= 512'd0;
                    wr_pack_strb       <= 64'd0;
                    wr_cur_mbeats_left <= wr_cur_mbeats_left - 1'b1;
                    wr_cur_half        <= 1'b0;
                end else begin
                    wr_cur_half <= 1'b1;
                end

                wr_cur_sbeats_left <= wr_cur_sbeats_left - 1'b1;
                if (wr_cur_sbeats_left == 8'd1)
                    wr_w_active <= 1'b0;
            end
        end
    end

    // ========================================================
    // Write response
    // ========================================================
    assign s_bid   = wr_bid_dout;
    assign s_bresp = m_bresp;
    assign s_bvalid = !wr_bid_empty && m_bvalid;
    assign m_bready = !wr_bid_empty && s_bready;
    assign wr_bid_pop = s_bvalid && s_bready;

    // ========================================================
    // Read address issue
    // ========================================================
    wire [ADDR_WIDTH-1:0] rd_iss_addr;
    wire [7:0]            rd_iss_mbeats;
    wire [1:0]            rd_iss_burst;
    wire                  rd_iss_lock;
    wire [3:0]            rd_iss_cache;
    wire [2:0]            rd_iss_prot;
    wire [3:0]            rd_iss_qos;
    wire [3:0]            rd_iss_region;

    assign {
        rd_iss_addr,
        rd_iss_mbeats,
        rd_iss_burst,
        rd_iss_lock,
        rd_iss_cache,
        rd_iss_prot,
        rd_iss_qos,
        rd_iss_region
    } = rd_iss_dout;

    assign m_arid     = {ID_WIDTH{1'b0}};
    assign m_araddr   = rd_iss_addr;
    assign m_arlen    = rd_iss_mbeats - 8'd1;
    assign m_arsize   = 3'b110; // 64 bytes / beat
    assign m_arburst  = rd_iss_burst;
    assign m_arlock   = rd_iss_lock;
    assign m_arcache  = rd_iss_cache;
    assign m_arprot   = rd_iss_prot;
    assign m_arqos    = rd_iss_qos;
    assign m_arregion = rd_iss_region;
    assign m_arvalid  = !rd_iss_empty;

    assign rd_iss_pop = m_arvalid && m_arready;

    // ========================================================
    // Read return path:
    // 512b MI R -> xpm_fifo_sync -> 256b SI R
    // ========================================================
    wire [RFIFO_W-1:0] rfifo_din  = {m_rresp, m_rlast, m_rdata};
    wire [RFIFO_W-1:0] rfifo_dout;
    wire rfifo_full, rfifo_empty;
    wire rfifo_wr_rst_busy, rfifo_rd_rst_busy;
    wire rfifo_wr_en = m_rvalid && m_rready;
    wire rfifo_rd_en;

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE ("0"),
        .ECC_MODE         ("no_ecc"),
        .FIFO_MEMORY_TYPE ("block"),
        .FIFO_READ_LATENCY(0),
        .FIFO_WRITE_DEPTH (RDATA_FIFO_DEPTH),
        .FULL_RESET_VALUE (0),
        .PROG_EMPTY_THRESH(10),
        .PROG_FULL_THRESH (RDATA_FIFO_DEPTH-10),
        .READ_DATA_WIDTH  (RFIFO_W),
        .READ_MODE        ("fwft"),
        .SIM_ASSERT_CHK   (0),
        .USE_ADV_FEATURES ("1000"),
        .WAKEUP_TIME      (0),
        .WRITE_DATA_WIDTH (RFIFO_W),
        .WR_DATA_COUNT_WIDTH(clog2(RDATA_FIFO_DEPTH)+1)
    ) u_rdata_fifo (
        .almost_empty (),
        .almost_full  (),
        .data_valid   (),
        .dbiterr      (),
        .dout         (rfifo_dout),
        .empty        (rfifo_empty),
        .full         (rfifo_full),
        .overflow     (),
        .prog_empty   (),
        .prog_full    (),
        .rd_data_count(),
        .rd_rst_busy  (rfifo_rd_rst_busy),
        .sbiterr      (),
        .underflow    (),
        .wr_ack       (),
        .wr_data_count(),
        .wr_rst_busy  (rfifo_wr_rst_busy),
        .din          (rfifo_din),
        .injectdbiterr(1'b0),
        .injectsbiterr(1'b0),
        .rd_en        (rfifo_rd_en),
        .rst          (!aresetn),
        .sleep        (1'b0),
        .wr_clk       (aclk),
        .wr_en        (rfifo_wr_en)
    );

    assign m_rready = !rfifo_full && !rfifo_wr_rst_busy && !rfifo_rd_rst_busy;

    wire [ID_WIDTH-1:0] rd_meta_id;
    wire [7:0]          rd_meta_sbeats;
    wire                rd_meta_half;

    assign {
        rd_meta_id,
        rd_meta_sbeats,
        rd_meta_half
    } = rd_meta_dout;

    reg                rd_active;
    reg [ID_WIDTH-1:0] rd_cur_id;
    reg [7:0]          rd_cur_sbeats_left;
    reg                rd_cur_half;

    reg                rd_word_valid;
    reg [511:0]        rd_word_data;
    reg [1:0]          rd_word_resp;

    reg                s_rvalid_r;
    reg [ID_WIDTH-1:0] s_rid_r;
    reg [255:0]        s_rdata_r;
    reg [1:0]          s_rresp_r;
    reg                s_rlast_r;

    wire load_rd_ctx = !rd_active && !rd_meta_empty;
    assign rd_meta_pop = load_rd_ctx;

    wire        rfifo_have_word = !rfifo_empty;
    wire [1:0]  rfifo_head_resp = rfifo_dout[514:513];
    wire        rfifo_head_last = rfifo_dout[512];
    wire [511:0] rfifo_head_data = rfifo_dout[511:0];

    wire        rd_output_ready = !s_rvalid_r || s_rready;

    reg         consume_rfifo_word;
    reg         generate_rbeat;
    reg [255:0] next_rdata;
    reg [1:0]   next_rresp;
    reg         next_rlast;
    reg         next_half_after;
    reg         next_word_valid_after;

    always @(*) begin
        consume_rfifo_word    = 1'b0;
        generate_rbeat        = 1'b0;
        next_rdata            = 256'd0;
        next_rresp            = 2'b00;
        next_rlast            = 1'b0;
        next_half_after       = rd_cur_half;
        next_word_valid_after = rd_word_valid;

        if (rd_output_ready && rd_active) begin
            // Case 1: no staged 512b word, try using FIFO head directly
            if (!rd_word_valid && rfifo_have_word) begin
                generate_rbeat = 1'b1;
                next_rresp     = rfifo_head_resp;
                next_rlast     = (rd_cur_sbeats_left == 8'd1);

                if (!rd_cur_half) begin
                    // send lower 256b first
                    next_rdata = rfifo_head_data[255:0];

                    if (rd_cur_sbeats_left == 8'd1) begin
                        // only one 256b beat needed from this 512b word
                        consume_rfifo_word    = 1'b1;
                        next_half_after       = 1'b0;
                        next_word_valid_after = 1'b0;
                    end else begin
                        // keep same 512b word staged for upper half
                        next_half_after       = 1'b1;
                        next_word_valid_after = 1'b1;
                    end
                end else begin
                    // send upper 256b first, then whole 512b word is consumed
                    next_rdata = rfifo_head_data[511:256];
                    consume_rfifo_word    = 1'b1;
                    next_half_after       = 1'b0;
                    next_word_valid_after = 1'b0;
                end
            end
            // Case 2: staged 512b word already exists
            else if (rd_word_valid) begin
                generate_rbeat = 1'b1;
                next_rresp     = rd_word_resp;
                next_rlast     = (rd_cur_sbeats_left == 8'd1);

                if (!rd_cur_half) begin
                    next_rdata = rd_word_data[255:0];
                    if (rd_cur_sbeats_left == 8'd1) begin
                        next_half_after       = 1'b0;
                        next_word_valid_after = 1'b0;
                    end else begin
                        next_half_after       = 1'b1;
                        next_word_valid_after = 1'b1;
                    end
                end else begin
                    next_rdata = rd_word_data[511:256];
                    // now truly consume the underlying FIFO entry
                    consume_rfifo_word    = 1'b1;
                    next_half_after       = 1'b0;
                    next_word_valid_after = 1'b0;
                end
            end
        end
    end

    assign rfifo_rd_en = consume_rfifo_word;
    wire rd_done = s_rvalid_r && s_rready && s_rlast_r;

    always @(posedge aclk) begin
        if (!aresetn) begin
            rd_pending_count   <= 8'd0;
            rd_active          <= 1'b0;
            rd_cur_id          <= {ID_WIDTH{1'b0}};
            rd_cur_sbeats_left <= 8'd0;
            rd_cur_half        <= 1'b0;
            rd_word_valid      <= 1'b0;
            rd_word_data       <= 512'd0;
            rd_word_resp       <= 2'b00;
            s_rvalid_r         <= 1'b0;
            s_rid_r            <= {ID_WIDTH{1'b0}};
            s_rdata_r          <= 256'd0;
            s_rresp_r          <= 2'b00;
            s_rlast_r          <= 1'b0;
        end else begin
            case ({rd_ar_push, rd_done})
                2'b10: rd_pending_count <= rd_pending_count + 1'b1;
                2'b01: rd_pending_count <= rd_pending_count - 1'b1;
                default: begin
                end
            endcase

            if (load_rd_ctx) begin
                rd_active          <= 1'b1;
                rd_cur_id          <= rd_meta_id;
                rd_cur_sbeats_left <= rd_meta_sbeats;
                rd_cur_half        <= rd_meta_half;
                rd_word_valid      <= 1'b0;
            end

            // stage FIFO head when lower half is sent first and upper half is still needed
            if (!rd_word_valid && rfifo_have_word && generate_rbeat &&
                !rd_cur_half && (rd_cur_sbeats_left != 8'd1)) begin
                rd_word_valid <= 1'b1;
                rd_word_data  <= rfifo_head_data;
                rd_word_resp  <= rfifo_head_resp;
            end

            if (generate_rbeat) begin
                s_rvalid_r <= 1'b1;
                s_rid_r    <= rd_cur_id;
                s_rdata_r  <= next_rdata;
                s_rresp_r  <= next_rresp;
                s_rlast_r  <= next_rlast;

                rd_cur_half        <= next_half_after;
                rd_word_valid      <= next_word_valid_after;
                rd_cur_sbeats_left <= rd_cur_sbeats_left - 1'b1;

                if (rd_cur_sbeats_left == 8'd1)
                    rd_active <= 1'b0;
            end

            if (s_rvalid_r && s_rready && !generate_rbeat)
                s_rvalid_r <= 1'b0;
        end
    end

    assign s_rid   = s_rid_r;
    assign s_rdata = s_rdata_r;
    assign s_rresp = s_rresp_r;
    assign s_rlast = s_rlast_r;
    assign s_rvalid = s_rvalid_r;

`ifndef SYNTHESIS
    always @(posedge aclk) begin
        if (aresetn) begin
            if (s_awvalid && s_awready) begin
                if (s_awsize != 3'b101)
                    $display("WARN: s_awsize should be 3'b101 for 256-bit SI.");
                if (s_awburst != 2'b01)
                    $display("WARN: current RTL targets INCR bursts.");
                if (s_awaddr[4:0] != 5'd0)
                    $display("WARN: current RTL expects 32-byte aligned SI write address.");
            end

            if (s_wvalid && s_wready) begin
                if ((wr_cur_sbeats_left == 8'd1) && !s_wlast)
                    $display("WARN: expected s_wlast on final SI write beat.");
                if ((wr_cur_sbeats_left != 8'd1) && s_wlast)
                    $display("WARN: unexpected early s_wlast on SI write channel.");
            end

            if (s_arvalid && s_arready) begin
                if (s_arsize != 3'b101)
                    $display("WARN: s_arsize should be 3'b101 for 256-bit SI.");
                if (s_arburst != 2'b01)
                    $display("WARN: current RTL targets INCR bursts.");
                if (s_araddr[4:0] != 5'd0)
                    $display("WARN: current RTL expects 32-byte aligned SI read address.");
            end
        end
    end
`endif

endmodule