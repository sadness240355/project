// axis_dwconv_h2c_256_to_512.v
`timescale 1ns / 1ps
module axis_dwconv_h2c_256_to_512 (
    input wire aclk,
    input wire aresetn,

    // s_axis: from QDMA IP (256b)
    input  wire         s_axis_tvalid,
    input  wire [255:0] s_axis_tdata,
    input  wire [ 31:0] s_axis_tcrc,
    input  wire         s_axis_tlast,
    input  wire [ 10:0] s_axis_tuser_qid,
    input  wire [  2:0] s_axis_tuser_port_id,
    input  wire         s_axis_tuser_err,
    input  wire [ 31:0] s_axis_tuser_mdata,
    input  wire [  5:0] s_axis_tuser_mty,        // empty bytes in last 256b beat
    input  wire         s_axis_tuser_zero_byte,
    output wire         s_axis_tready,

    // m_axis: to subsystem (512b)
    output wire         m_axis_tvalid,
    output wire [511:0] m_axis_tdata,
    output wire [ 31:0] m_axis_tcrc,
    output wire         m_axis_tlast,
    output wire [ 10:0] m_axis_tuser_qid,
    output wire [  2:0] m_axis_tuser_port_id,
    output wire         m_axis_tuser_err,
    output wire [ 31:0] m_axis_tuser_mdata,
    output wire [  5:0] m_axis_tuser_mty,        // empty bytes in last 512b beat
    output wire         m_axis_tuser_zero_byte,
    input  wire         m_axis_tready
);

    // 2-entry FIFO for 256b beats
    reg v0, v1;

    reg [255:0] d0, d1;
    reg [31:0] c0, c1;
    reg l0, l1;
    reg [10:0] q0, q1;
    reg [2:0] p0, p1;
    reg e0, e1;
    reg [31:0] m0, m1;
    reg [5:0] y0, y1;
    reg z0, z1;

    // output register (512b beat)
    reg          out_v;
    reg  [511:0] out_d;
    reg  [ 31:0] out_c;
    reg          out_l;
    reg  [ 10:0] out_q;
    reg  [  2:0] out_p;
    reg          out_e;
    reg  [ 31:0] out_m;
    reg  [  5:0] out_y;
    reg          out_z;

    wire         out_fire = out_v && m_axis_tready;
    wire         out_slotfree = (~out_v) || out_fire;

    wire         can_form = v0 && (l0 || v1);  // have 1 beat with TLAST, or have 2 beats
    wire         will_load_out = out_slotfree && can_form;

    wire         pop_one = will_load_out && l0;  // single-beat packet ends in first 256
    wire         pop_two = will_load_out && (~l0);  // pack two 256 beats

    // ready if FIFO not full, OR full but we will pop this cycle
    assign s_axis_tready = (~v1) || (v1 && (pop_one || pop_two));

    wire in_fire = s_axis_tvalid && s_axis_tready;

    // outputs
    assign m_axis_tvalid          = out_v;
    assign m_axis_tdata           = out_d;
    assign m_axis_tcrc            = out_c;
    assign m_axis_tlast           = out_l;
    assign m_axis_tuser_qid       = out_q;
    assign m_axis_tuser_port_id   = out_p;
    assign m_axis_tuser_err       = out_e;
    assign m_axis_tuser_mdata     = out_m;
    assign m_axis_tuser_mty       = out_y;
    assign m_axis_tuser_zero_byte = out_z;

    always @(posedge aclk) begin
        if (!aresetn) begin
            v0    <= 1'b0;
            v1    <= 1'b0;
            out_v <= 1'b0;
        end else begin
            // -----------------------------
            // 1) consume output if fired
            // -----------------------------
            if (out_fire && !will_load_out) begin
                out_v <= 1'b0;
            end

            // -----------------------------
            // 2) load next output beat
            // -----------------------------
            if (will_load_out) begin
                out_v <= 1'b1;

                if (l0) begin
                    // only one 256b beat -> 512b (upper padded with zeros)
                    out_d <= {256'd0, d0};
                    out_l <= 1'b1;

                    // mty_512 = 32 (upper empty) + mty_256(lower empty)
                    out_y <= 6'd32 + y0;

                    out_c <= c0;  // not used in your downstream currently
                    out_q <= q0;
                    out_p <= p0;
                    out_e <= e0;
                    out_m <= m0;
                    out_z <= z0;
                end else begin
                    // two 256b beats -> one 512b beat
                    out_d <= {d1, d0};
                    out_l <= l1;
                    out_y <= (l1) ? y1 : 6'd0;

                    out_c <= c1;
                    out_q <= q0;  // take sideband from first beat (should be stable per pkt)
                    out_p <= p0;
                    out_e <= e0;
                    out_m <= m0;
                    out_z <= z0;
                end
            end

            // -----------------------------
            // 3) FIFO pop (based on old v0/v1)
            // -----------------------------
            // compute intermediate fifo after pop
            // (we do it inline with careful ordering)
            if (pop_two) begin
                v0 <= 1'b0;
                v1 <= 1'b0;
            end else if (pop_one) begin
                if (v1) begin
                    // shift entry1 -> entry0
                    d0 <= d1;
                    c0 <= c1;
                    l0 <= l1;
                    q0 <= q1;
                    p0 <= p1;
                    e0 <= e1;
                    m0 <= m1;
                    y0 <= y1;
                    z0 <= z1;
                    v0 <= 1'b1;
                    v1 <= 1'b0;
                end else begin
                    v0 <= 1'b0;
                    v1 <= 1'b0;
                end
            end

            // -----------------------------
            // 4) FIFO push (after pop effect)
            // -----------------------------
            if (in_fire) begin
                // decide current emptiness after pop
                // (re-derive by looking at updated v0/v1 if pop happened this cycle)
                // To keep it simple and safe, use a conservative insertion:
                if (!((pop_two) ? 1'b0 : (pop_one ? (v1 ? 1'b1 : 1'b0) : v0))) begin
                    // slot0 empty
                    d0 <= s_axis_tdata;
                    c0 <= s_axis_tcrc;
                    l0 <= s_axis_tlast;
                    q0 <= s_axis_tuser_qid;
                    p0 <= s_axis_tuser_port_id;
                    e0 <= s_axis_tuser_err;
                    m0 <= s_axis_tuser_mdata;
                    y0 <= s_axis_tuser_mty;
                    z0 <= s_axis_tuser_zero_byte;
                    v0 <= 1'b1;
                end else if (!((pop_two) ? 1'b0 : (pop_one ? 1'b0 : v1))) begin
                    // slot1 empty
                    d1 <= s_axis_tdata;
                    c1 <= s_axis_tcrc;
                    l1 <= s_axis_tlast;
                    q1 <= s_axis_tuser_qid;
                    p1 <= s_axis_tuser_port_id;
                    e1 <= s_axis_tuser_err;
                    m1 <= s_axis_tuser_mdata;
                    y1 <= s_axis_tuser_mty;
                    z1 <= s_axis_tuser_zero_byte;
                    v1 <= 1'b1;
                end
            end
        end
    end

endmodule
