`timescale 1ns/1ps
module tb; parameter EXPECT_REJECT=0;
reg clk=0;always #2 clk=~clk;
reg rstn=0;reg [511:0] rx_data=0;reg [63:0] rx_keep=0;reg rx_valid=0,rx_last=0;
wire [4:0] lookup_qp;wire psn_load;wire [63:0] awaddr;wire awvalid;wire [511:0] wdata;wire wvalid;
reg awready=1,wready=1,bvalid=0;wire bready;wire [511:0] tx_data;wire [63:0] tx_keep;wire tx_valid;reg tx_ready=0;
wire [31:0] committed,dropped,duplicates,write_errors;
reg [31:0] cfg_aw=0,cfg_w=0;reg cfg_awv=0,cfg_wv=0;
wire cfg_enabled;wire [47:0] cfg_lmac,cfg_rmac;wire [31:0] cfg_lip,cfg_rip,cfg_qpc,cfg_key,cfg_len,cfg_access;wire [23:0] cfg_rqpn,cfg_last,cfg_load_v;wire [15:0] cfg_pkey;wire [63:0] cfg_va,cfg_pa,cfg_db;wire [4:0] cfg_load_q;
lrdma_compat_config config_dut(.clk(clk),.rstn(rstn),.awaddr(cfg_aw),.wdata(cfg_w),.wstrb(4'hf),.awvalid(cfg_awv),.awready(1'b1),.wvalid(cfg_wv),.wready(1'b1),.rx_qp(lookup_qp),.db_qp(5'd2),.enabled(cfg_enabled),.local_mac(cfg_lmac),.remote_mac(cfg_rmac),.local_ip(cfg_lip),.remote_ip(cfg_rip),.rkey(cfg_key),.access(cfg_access),.virtual_base(cfg_va),.dma_base(cfg_pa),.db_base(cfg_db),.region_len(cfg_len),.qp_conf(cfg_qpc),.remote_qpn(cfg_rqpn),.last_psn(cfg_last),.pkey(cfg_pkey),.psn_load(psn_load),.psn_load_qp(cfg_load_q),.psn_load_value(cfg_load_v));
task config_write(input [31:0] a,d);begin @(negedge clk);cfg_aw=a;cfg_w=d;cfg_awv=1;cfg_wv=1;@(negedge clk);cfg_awv=0;cfg_wv=0;repeat(4)@(negedge clk);end endtask
lrdma_credit_rx dut(.clk(clk),.rstn(rstn),.rx_data(rx_data),.rx_keep(rx_keep),.rx_valid(rx_valid),.rx_last(rx_last),.rx_error(1'b0),.lookup_qp(lookup_qp),
.enabled(cfg_enabled),.local_mac(cfg_lmac),.remote_mac(cfg_rmac),.local_ip(cfg_lip),.remote_ip(cfg_rip),.qp_conf(cfg_qpc),.remote_qpn(cfg_rqpn),.pkey(cfg_pkey),
.virtual_base(cfg_va),.dma_base(cfg_pa),.rkey(cfg_key),.region_len(cfg_len),.access(cfg_access),.psn_load(psn_load),.psn_load_qp(cfg_load_q),.psn_load_value(cfg_load_v),
.awaddr(awaddr),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wvalid(wvalid),.wready(wready),.bresp(2'b0),.bvalid(bvalid),.bready(bready),.tx_data(tx_data),.tx_keep(tx_keep),.tx_valid(tx_valid),.tx_ready(tx_ready),.committed(committed),.dropped(dropped),.duplicates(duplicates),.write_errors(write_errors));
reg [7:0] bytes[0:81];integer j,t;reg seen_aw=0,seen_w=0;integer writes=0;
always @(posedge clk)if(rstn)begin
 if(awvalid&&awready)begin if(awaddr!=64'hfe124000)$fatal(1,"bad DMA address");seen_aw<=1;end
 if(wvalid&&wready)begin if(wdata!=512'd1)$fatal(1,"bad credit payload");seen_w<=1;writes<=writes+1;end
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
 if(EXPECT_REJECT)begin
  t=0;while(dropped==0)begin @(negedge clk);t=t+1;if(t>300)$fatal(1,"baseline did not reject expected flags");end
  if(writes||committed||tx_valid)$fatal(1,"baseline unexpectedly accepted actual packet");
  $display("BASELINE_REAL_PACKET_REJECT_PASS flags=0x40 fields_ok=%d",dut.fields_ok_reg);$finish;
 end
 t=0;while(!seen_aw||!seen_w)begin @(negedge clk);t=t+1;if(t>300)$fatal(1,"credit rejected dropped=%d state=%d fields=%d",dropped,dut.state,dut.fields_ok_reg);end
 repeat(20)begin @(negedge clk);if(tx_valid||committed)$fatal(1,"ACK before B response");end
 bvalid=1;@(negedge clk);bvalid=0;
 t=0;while(!tx_valid)begin @(negedge clk);t=t+1;if(t>300)$fatal(1,"ACK absent");end
 if(committed!=1||tx_keep!=64'h3fffffffffffffff||tx_data[42*8+:8]!=8'h11)$fatal(1,"bad ACK");
 repeat(10)begin @(negedge clk);if(!tx_valid)$fatal(1,"ACK lost while stalled");end
 tx_ready=1;repeat(3)@(negedge clk);packet();t=0;
 while(duplicates!=1)begin @(negedge clk);t=t+1;if(t>300)$fatal(1,"duplicate failed");end
 if(writes!=1||write_errors||dropped)$fatal(1,"duplicate wrote twice or error");
 t=0;while(!tx_valid)begin @(negedge clk);t=t+1;if(t>300)$fatal(1,"duplicate ACK absent");end
 repeat(3)@(negedge clk);bytes[43]=8'h50;packet();repeat(40)@(negedge clk);if(dropped!=1||writes!=1)$fatal(1,"bad padding accepted");
 bytes[43]=8'h41;packet();repeat(40)@(negedge clk);if(dropped!=2||writes!=1)$fatal(1,"bad version accepted");
 $display("REAL_PACKET_FIXED_PASS cases=4 ACCEPT_MIG_DUPLICATE_NO_REWRITE_REJECT_PAD_AND_VERSION");$finish;
end
initial begin #10000;$fatal(1,"timeout");end
endmodule
