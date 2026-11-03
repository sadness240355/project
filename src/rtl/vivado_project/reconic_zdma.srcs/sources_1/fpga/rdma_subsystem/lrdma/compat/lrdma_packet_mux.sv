`timescale 1ns/1ps
// Packet-granular arbitration, preserving an offered transfer under stall.
module lrdma_packet_mux(
 input wire clk,rstn,
 input wire [511:0] a_data,b_data,input wire [63:0] a_keep,b_keep,
 input wire a_valid,a_last,b_valid,b_last,output wire a_ready,b_ready,
 output wire [511:0] m_data,output wire [63:0] m_keep,
 output wire m_valid,m_last,input wire m_ready
);
 reg [1:0] grant;wire [1:0] selected=grant!=0?grant:(b_valid?2'd2:2'd1);
 assign m_data=selected==2?b_data:a_data;assign m_keep=selected==2?b_keep:a_keep;
 assign m_valid=selected==2?b_valid:a_valid;assign m_last=selected==2?b_last:a_last;
 assign a_ready=selected==1&&m_ready;assign b_ready=selected==2&&m_ready;
 always @(posedge clk)begin
  if(!rstn)grant<=0;
  else if(m_valid)begin
   if(m_ready&&m_last)grant<=0;else grant<=selected;
  end
 end
endmodule
