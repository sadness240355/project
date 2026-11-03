// A registered-ready skid boundary isolates ISP CE from downstream HLS/packing.
// Full eight-pixel beats. SOF and EOF are buffered alongside every pixel beat.
module camera_isp_stream #(
 parameter LANES=8, CHANNELS=1, DATA_WIDTH=LANES*CHANNELS*8
)(
 input wire clk,rst,input wire [15:0] width,input wire bayer,
 input wire s_valid,output wire s_ready,input wire [DATA_WIDTH-1:0] s_data,input wire s_sof,s_last,
 output wire m_valid,input wire m_ready,output wire [DATA_WIDTH-1:0] m_data,output wire m_sof,m_last,
 input wire [271:0] config_data,input wire config_clk,lut_we,input wire [7:0] lut_addr,lut_data
);
 wire valid,ready,sof,last;wire [DATA_WIDTH-1:0] data;
 uip_axis #(.LANES(LANES),.CHANNELS(CHANNELS)) pixels (
 .clk(clk),.rst(rst),.width(width),.bayer(bayer),
 .s_valid(s_valid),.s_ready(s_ready),.s_data(s_data),.s_sof(s_sof),.s_last(s_last),
 .m_valid(valid),.m_ready(ready),.m_data(data),.m_sof(sof),.m_last(last),
 .config_data(config_data),.config_clk(config_clk),.lut_we(lut_we),.lut_addr(lut_addr),.lut_data(lut_data));
 axis_register #(.DATA_WIDTH(DATA_WIDTH),.KEEP_ENABLE(0),.KEEP_WIDTH(1),.LAST_ENABLE(1),
 .ID_ENABLE(0),.ID_WIDTH(1),.DEST_ENABLE(0),.DEST_WIDTH(1),.USER_ENABLE(1),.USER_WIDTH(1),.REG_TYPE(2)) boundary (
 .clk(clk),.rst(rst),.s_axis_tdata(data),.s_axis_tkeep(1'b1),.s_axis_tvalid(valid),.s_axis_tready(ready),
 .s_axis_tlast(last),.s_axis_tid(1'b0),.s_axis_tdest(1'b0),.s_axis_tuser(sof),
 .m_axis_tdata(m_data),.m_axis_tkeep(),.m_axis_tvalid(m_valid),.m_axis_tready(m_ready),
 .m_axis_tlast(m_last),.m_axis_tid(),.m_axis_tdest(),.m_axis_tuser(m_sof));
endmodule
