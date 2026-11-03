// SPDX-License-Identifier: MIT
// AXI stream adapter. TUSER=SOF, TLAST=EOF (the ERNIC/HLS contract).
// Full beats only; all pipeline state freezes together under downstream backpressure.
module uip_axis #(
 parameter LANES=8,CHANNELS=1,DATA_WIDTH=LANES*CHANNELS*8
)(
 input wire clk,rst,input wire [15:0] width,input wire bayer,
 input wire s_valid,output wire s_ready,input wire [DATA_WIDTH-1:0] s_data,input wire s_sof,s_last,
 output wire m_valid,input wire m_ready,output wire [DATA_WIDTH-1:0] m_data,output wire m_sof,m_last,
 input wire [271:0] config_data,input wire config_clk,lut_we,input wire [7:0] lut_addr,lut_data
);
 wire advance=!m_valid || m_ready;
 assign s_ready=advance && !rst;
 reg [15:0] x=0;
 wire [15:0] current_x=s_sof ? 16'd0:x;
 localparam [15:0] STEP=LANES;
 wire eol=current_x+STEP>=width;
 always @(posedge clk)begin
  if(rst)begin x<=0;end
  else if(advance)begin
   if(s_valid)x<=eol ? 16'd0:current_x+STEP;
  end
 end
 uip_core #(.LANES(LANES),.CHANNELS(CHANNELS)) pixels (
  .clk(clk),.rst(rst),.ce(advance),.s_valid(s_valid),.s_sof(s_sof),.s_eol(eol),.s_data(s_data),
  .s_last(s_last),.m_last(m_last),
  .m_valid(m_valid),.m_sof(m_sof),.m_eol(),.m_data(m_data),.bayer(bayer),.config_data(config_data),
  .config_clk(config_clk),.lut_we(lut_we),.lut_addr(lut_addr),.lut_data(lut_data));
endmodule
