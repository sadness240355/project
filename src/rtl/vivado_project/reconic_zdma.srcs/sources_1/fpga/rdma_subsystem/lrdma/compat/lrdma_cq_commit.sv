`timescale 1ns/1ps
// Preserve the current software completion ABI: CQE payload first, then the
// absolute CQ head doorbell, then acknowledge the original AXI write. Both
// writes must return OKAY. A failed first write never publishes a success DB.
module lrdma_cq_commit(
 input wire clk,rstn,input wire [63:0] db_addr,input wire [15:0] head,
 input wire [63:0] s_awaddr,input wire [7:0] s_awlen,input wire [2:0] s_awsize,
 input wire [1:0] s_awburst,input wire [3:0] s_awcache,input wire [2:0] s_awprot,
 input wire s_awvalid,output wire s_awready,
 input wire [511:0] s_wdata,input wire [63:0] s_wstrb,input wire s_wlast,s_wvalid,output wire s_wready,
 output wire [1:0] s_bresp,output wire s_bvalid,input wire s_bready,
 output wire [63:0] m_awaddr,output wire [7:0] m_awlen,output wire [2:0] m_awsize,
 output wire [1:0] m_awburst,output wire [3:0] m_awcache,output wire [2:0] m_awprot,
 output wire m_awvalid,input wire m_awready,
 output wire [511:0] m_wdata,output wire [63:0] m_wstrb,output wire m_wlast,m_wvalid,input wire m_wready,
 input wire [1:0] m_bresp,input wire m_bvalid,output wire m_bready
);
 localparam PASS=0,FIRST_B=1,DB=2,DB_B=3,DONE=4;
 reg [2:0] state;reg aw_done,w_done;reg [63:0] addr;reg [15:0] count;reg [1:0] resp;
 wire db_mode=state==DB;
 assign m_awaddr=db_mode?addr:s_awaddr;assign m_awlen=db_mode?8'd0:s_awlen;
 assign m_awsize=db_mode?3'd2:s_awsize;assign m_awburst=db_mode?2'b01:s_awburst;
 assign m_awcache=db_mode?4'b0011:s_awcache;assign m_awprot=db_mode?3'd0:s_awprot;
 assign m_awvalid=(state==PASS?s_awvalid:db_mode)&&!aw_done;
 assign s_awready=state==PASS&&!aw_done&&m_awready;
 // Absolute doorbell is 32-bit, aligned within the 512-bit AXI beat.
 assign m_wdata=db_mode?({480'd0,16'd0,count} << (addr[5:0]*8)):s_wdata;
 assign m_wstrb=db_mode?(64'hf << addr[5:0]):s_wstrb;
 assign m_wlast=db_mode?1'b1:s_wlast;
 assign m_wvalid=(state==PASS?s_wvalid:db_mode)&&!w_done;
 assign s_wready=state==PASS&&!w_done&&m_wready;
 assign m_bready=state==FIRST_B||state==DB_B;
 assign s_bvalid=state==DONE;assign s_bresp=resp;
 always @(posedge clk)begin
  if(!rstn)begin state<=PASS;aw_done<=0;w_done<=0;addr<=0;count<=0;resp<=0;end
  else case(state)
   PASS:begin
    if(s_awvalid&&s_awready)begin aw_done<=1;addr<=db_addr;count<=head;end
    if(s_wvalid&&s_wready&&s_wlast)w_done<=1;
    if((aw_done||(s_awvalid&&s_awready))&&(w_done||(s_wvalid&&s_wready&&s_wlast)))state<=FIRST_B;
   end
   FIRST_B:if(m_bvalid)begin
    resp<=m_bresp;
    if(m_bresp!=0||addr==0)state<=DONE;
    else begin state<=DB;aw_done<=0;w_done<=0;end
   end
   DB:begin
    if(m_awvalid&&m_awready)aw_done<=1;
    if(m_wvalid&&m_wready)w_done<=1;
    if((aw_done||(m_awvalid&&m_awready))&&(w_done||(m_wvalid&&m_wready)))state<=DB_B;
   end
   DB_B:if(m_bvalid)begin resp<=m_bresp;state<=DONE;end
   DONE:if(s_bready)begin state<=PASS;aw_done<=0;w_done<=0;end
   default:state<=PASS;
  endcase
 end
endmodule
