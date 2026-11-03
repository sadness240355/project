`timescale 1ns/1ps
// Append the BTH-declared zero padding before ICRC. RoCE headers (including
// Ethernet) occupy 2 mod 4 bytes. A registered elastic stage preserves packets
// under arbitrary downstream stalls, including padding spilling to a new beat.
module lrdma_tx_padding(
 input wire clk,rst,
 input wire [511:0] s_data,input wire [63:0] s_keep,
 input wire s_valid,s_last,output wire s_ready,
 output reg [511:0] m_data,output reg [63:0] m_keep,
 output reg m_valid,m_last,input wire m_ready
);
 reg extra;reg [2:0] extra_bytes;
 integer n,j,p,total;
 assign s_ready=(!m_valid||m_ready)&&!extra;
 always @(posedge clk)begin
  if(rst)begin m_data<=0;m_keep<=0;m_valid<=0;m_last<=0;extra<=0;extra_bytes<=0;end
  else if(!m_valid||m_ready)begin
   m_valid<=0;
   if(extra)begin
    m_data<=0;m_keep<=64'hffffffffffffffff>>(64-extra_bytes);
    m_valid<=1;m_last<=1;extra<=0;
   end else if(s_valid)begin
    n=0;for(j=0;j<64;j=j+1)if(s_keep[j])n=j+1;
    p=(2-n)&3;
    total=n+(s_last?p:0);
    for(j=0;j<64;j=j+1)m_data[j*8+:8]<=s_keep[j]?s_data[j*8+:8]:8'd0;
    m_keep<=s_last?(total>=64?64'hffffffffffffffff:64'hffffffffffffffff>>(64-total)):s_keep;
    m_valid<=1;m_last<=s_last&&total<=64;
    if(s_last&&total>64)begin extra<=1;extra_bytes<=total-64;end
   end
  end
 end
endmodule
