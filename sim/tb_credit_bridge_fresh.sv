`timescale 1ns/1ps
module tb; parameter AW_SIZE=3;
reg clk=0;always #2 clk=~clk;
reg rstn=0;reg [511:0] rx_data=0;reg [63:0] rx_keep=0;reg rx_valid=0,rx_last=0;
wire [4:0] lookup_qp;wire psn_load;wire [63:0] awaddr;wire awvalid;wire [511:0] wdata;wire wvalid;
wire awready,wready,bvalid;wire [1:0] bresp;wire bready;wire [511:0] tx_data;wire [63:0] tx_keep;wire tx_valid;reg tx_ready=0;
wire [31:0] committed,dropped,duplicates,write_errors;
reg [31:0] cfg_aw=0,cfg_w=0;reg cfg_awv=0,cfg_wv=0;
wire cfg_enabled;wire [47:0] cfg_lmac,cfg_rmac;wire [31:0] cfg_lip,cfg_rip,cfg_qpc,cfg_key,cfg_len,cfg_access;wire [23:0] cfg_rqpn,cfg_last,cfg_load_v;wire [15:0] cfg_pkey;wire [63:0] cfg_va,cfg_pa,cfg_db;wire [4:0] cfg_load_q;
lrdma_compat_config config_dut(.clk(clk),.rstn(rstn),.awaddr(cfg_aw),.wdata(cfg_w),.wstrb(4'hf),.awvalid(cfg_awv),.awready(1'b1),.wvalid(cfg_wv),.wready(1'b1),.rx_qp(lookup_qp),.db_qp(5'd2),.enabled(cfg_enabled),.local_mac(cfg_lmac),.remote_mac(cfg_rmac),.local_ip(cfg_lip),.remote_ip(cfg_rip),.rkey(cfg_key),.access(cfg_access),.virtual_base(cfg_va),.dma_base(cfg_pa),.db_base(cfg_db),.region_len(cfg_len),.qp_conf(cfg_qpc),.remote_qpn(cfg_rqpn),.last_psn(cfg_last),.pkey(cfg_pkey),.psn_load(psn_load),.psn_load_qp(cfg_load_q),.psn_load_value(cfg_load_v));
task config_write(input [31:0] a,d);begin @(negedge clk);cfg_aw=a;cfg_w=d;cfg_awv=1;cfg_wv=1;@(negedge clk);cfg_awv=0;cfg_wv=0;repeat(4)@(negedge clk);end endtask
lrdma_credit_rx dut(.clk(clk),.rstn(rstn),.rx_data(rx_data),.rx_keep(rx_keep),.rx_valid(rx_valid),.rx_last(rx_last),.rx_error(1'b0),.lookup_qp(lookup_qp),
.enabled(cfg_enabled),.local_mac(cfg_lmac),.remote_mac(cfg_rmac),.local_ip(cfg_lip),.remote_ip(cfg_rip),.qp_conf(cfg_qpc),.remote_qpn(cfg_rqpn),.pkey(cfg_pkey),
.virtual_base(cfg_va),.dma_base(cfg_pa),.rkey(cfg_key),.region_len(cfg_len),.access(cfg_access),.psn_load(psn_load),.psn_load_qp(cfg_load_q),.psn_load_value(cfg_load_v),
.awaddr(awaddr),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wvalid(wvalid),.wready(wready),.bresp(bresp),.bvalid(bvalid),.bready(bready),.tx_data(tx_data),.tx_keep(tx_keep),.tx_valid(tx_valid),.tx_ready(tx_ready),.committed(committed),.dropped(dropped),.duplicates(duplicates),.write_errors(write_errors));
reg [7:0] bytes[0:81];integer j,t;integer writes=0;
wire [63:0] cv_awaddr;wire [7:0] cv_awlen;wire [2:0] cv_awsize;wire [1:0] cv_awburst;
wire [3:0] cv_awid,cv_bid;wire cv_awvalid,cv_awready,cv_wvalid,cv_wready,cv_wlast,cv_bvalid,cv_bready;
wire [255:0] cv_wdata;wire [31:0] cv_wstrb;wire [1:0] cv_bresp;
wire [255:0] rq_data;wire [7:0] rq_keep;wire [61:0] rq_user;wire rq_valid,rq_last;
reg rq_ready=0;integer rq_count=0;
axi4_dw_512_to_256 converter(.aclk(clk),.aresetn(rstn),
.s_axi_awid(4'd0),.s_axi_awaddr(awaddr),.s_axi_awlen(8'd0),.s_axi_awsize(AW_SIZE[2:0]),.s_axi_awburst(2'b01),.s_axi_awvalid(awvalid),.s_axi_awready(awready),
.s_axi_wdata(wdata),.s_axi_wstrb(64'hff),.s_axi_wlast(1'b1),.s_axi_wvalid(wvalid),.s_axi_wready(wready),.s_axi_bresp(bresp),.s_axi_bvalid(bvalid),.s_axi_bready(bready),
.s_axi_arid(4'd0),.s_axi_araddr(64'd0),.s_axi_arlen(8'd0),.s_axi_arsize(3'd6),.s_axi_arburst(2'b01),.s_axi_arvalid(1'b0),.s_axi_rready(1'b1),
.m_axi_awaddr(cv_awaddr),.m_axi_awid(cv_awid),.m_axi_awlen(cv_awlen),.m_axi_awsize(cv_awsize),.m_axi_awburst(cv_awburst),.m_axi_awvalid(cv_awvalid),.m_axi_awready(cv_awready),
.m_axi_wdata(cv_wdata),.m_axi_wstrb(cv_wstrb),.m_axi_wlast(cv_wlast),.m_axi_wvalid(cv_wvalid),.m_axi_wready(cv_wready),.m_axi_bresp(cv_bresp),.m_axi_bid(cv_bid),.m_axi_bvalid(cv_bvalid),.m_axi_bready(cv_bready),
.m_axi_arready(1'b0),.m_axi_rdata(256'd0),.m_axi_rresp(2'd0),.m_axi_rid(4'd0),.m_axi_rlast(1'b0),.m_axi_rvalid(1'b0));
edev_axi_pcie_bridge bridge(.clk(clk),.rst(!rstn),.enable(1'b1),.table_enable(4'b0001),.table_axi_base(256'd0),.table_pcie_base(256'd0),.table_mask(256'hffffffff00000000),
.requester_id(16'd0),.requester_id_enable(1'b0),.ext_tag_enable(1'b0),.max_read_request_size(3'd2),
.s_axi_awid(cv_awid),.s_axi_awaddr(cv_awaddr),.s_axi_awlen(cv_awlen),.s_axi_awsize(cv_awsize),.s_axi_awburst(cv_awburst),.s_axi_awvalid(cv_awvalid),.s_axi_awready(cv_awready),
.s_axi_wdata(cv_wdata),.s_axi_wstrb(cv_wstrb),.s_axi_wlast(cv_wlast),.s_axi_wvalid(cv_wvalid),.s_axi_wready(cv_wready),.s_axi_bid(cv_bid),.s_axi_bresp(cv_bresp),.s_axi_bvalid(cv_bvalid),.s_axi_bready(cv_bready),
.s_axi_arid(4'd0),.s_axi_araddr(64'd0),.s_axi_arlen(8'd0),.s_axi_arsize(3'd5),.s_axi_arburst(2'b01),.s_axi_arvalid(1'b0),.s_axi_rready(1'b1),
.m_axis_rq_tdata(rq_data),.m_axis_rq_tkeep(rq_keep),.m_axis_rq_tvalid(rq_valid),.m_axis_rq_tready(rq_ready),.m_axis_rq_tlast(rq_last),.m_axis_rq_tuser(rq_user),
.s_axis_rc_tdata(256'd0),.s_axis_rc_tkeep(8'd0),.s_axis_rc_tvalid(1'b0),.s_axis_rc_tlast(1'b0),.s_axis_rc_tuser(75'd0));
always @(posedge clk)if(rstn)begin
 if(wvalid&&wready)writes<=writes+1;
 if(rq_valid&&rq_ready)begin
  if(rq_count>1||!rq_last||rq_keep!=8'h1f||rq_data[78:75]!=4'd1||rq_data[74:64]!=11'd1||rq_user[7:0]!=8'h0f)$fatal(1,"unexpected TLP or byte enables");
  if({rq_data[63:2],2'b00}!=(64'hfe124000+4*rq_count))$fatal(1,"write outside target eight bytes");
  if(rq_data[159:128]!=(rq_count==0?32'd1:32'd0))$fatal(1,"wrong payload DWORD");
  rq_count<=rq_count+1;
 end
end
task packet;begin
 @(negedge clk);rx_data=0;for(j=0;j<64;j=j+1)rx_data[j*8+:8]=bytes[j];rx_keep=~64'd0;rx_valid=1;rx_last=0;
 @(negedge clk);rx_data=0;for(j=0;j<18;j=j+1)rx_data[j*8+:8]=bytes[j+64];rx_keep=64'h3ffff;rx_last=1;
 @(negedge clk);rx_valid=0;rx_last=0;
end endtask
initial begin
 $dumpfile("credit.vcd");$dumpvars(0,tb);$readmemh("packet.hex",bytes);
 repeat(5)@(negedge clk);rstn=1;
 config_write(32'h300010,32'h35000100);config_write(32'h300014,10);config_write(32'h300070,32'hc0a8010a);
 config_write(32'h200004,32'h8ee34000);config_write(32'h200008,32'hffff);config_write(32'h20000c,32'hfe124000);config_write(32'h200010,0);config_write(32'h200014,32'h100);config_write(32'h200018,4096);config_write(32'h20001c,2);
 config_write(32'h3802b0,0);config_write(32'h380204,32'hffff4000);config_write(32'h380250,32'hf68393be);config_write(32'h380254,32'hb8ce);config_write(32'h380260,32'hc0a80114);config_write(32'h380248,32'h15d);config_write(32'h380244,32'h0a023455);config_write(32'h380200,32'h0200043d);config_write(32'h300000,32'h00c00021);
 packet();
 if(AW_SIZE==3)begin
  repeat(600)@(negedge clk);
  if(dut.state!=6||writes||rq_count||committed||dropped||!dut.fields_ok_reg)$fatal(1,"baseline mismatch not reproduced");
  $display("BASELINE_BRIDGE_STALL_REPRODUCED AWSIZE3 state=WRITE unsupported=1");$finish;
 end
 repeat(300)begin @(negedge clk);if(tx_valid||committed)$fatal(1,"ACK before PCIe request acceptance");end
 rq_ready=1;t=0;
 while(!tx_valid)begin @(negedge clk);t=t+1;if(t>1000)$fatal(1,"fixed pipeline did not ACK state=%d",dut.state);end
 if(rq_count!=2||writes!=1||committed!=1||write_errors||dropped||tx_data[42*8+:8]!=8'h11)$fatal(1,"bad fixed completion");
 tx_ready=1;repeat(5)@(negedge clk);packet();t=0;
 while(duplicates!=1||!tx_valid)begin @(negedge clk);t=t+1;if(t>1000)$fatal(1,"duplicate missing");end
 if(rq_count!=2||writes!=1||committed!=1)$fatal(1,"duplicate wrote again");
 $display("CREDIT_WIDTH_BRIDGE_PASS REAL_RTL_TWO_DW_EXACT_8_BYTES_ACK_AFTER_RESPONSE_DUPLICATE_NO_REWRITE");$finish;
end
initial begin #20000;$fatal(1,"timeout");end
endmodule
