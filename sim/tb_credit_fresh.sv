`timescale 1ns/1ps
module tb;
reg clk=0;always #2 clk=~clk;
reg rstn=0;reg [511:0] rx_data=0;reg [63:0] rx_keep=0;reg rx_valid=0,rx_last=0;
wire [4:0] lookup_qp;reg psn_load=0;wire [63:0] awaddr;wire awvalid;wire [511:0] wdata;wire wvalid;
reg awready=1,wready=1,bvalid=0;wire bready;wire [511:0] tx_data;wire [63:0] tx_keep;wire tx_valid;reg tx_ready=0;
wire [31:0] committed,dropped,duplicates,write_errors;
lrdma_credit_rx dut(.clk(clk),.rstn(rstn),.rx_data(rx_data),.rx_keep(rx_keep),.rx_valid(rx_valid),.rx_last(rx_last),.rx_error(1'b0),.lookup_qp(lookup_qp),
.enabled(1'b1),.local_mac(48'h000a35000100),.remote_mac(48'hb8cef68393be),.local_ip(32'hc0a8010a),.remote_ip(32'hc0a80114),.qp_conf(32'h0200043d),.remote_qpn(24'h15d),.pkey(16'hffff),
.virtual_base(64'h0000ffffba517000),.dma_base(64'heef01000),.rkey(32'h100),.region_len(32'd4096),.access(32'd2),.psn_load(psn_load),.psn_load_qp(5'd2),.psn_load_value(24'habc),
.awaddr(awaddr),.awvalid(awvalid),.awready(awready),.wdata(wdata),.wvalid(wvalid),.wready(wready),.bresp(2'b0),.bvalid(bvalid),.bready(bready),.tx_data(tx_data),.tx_keep(tx_keep),.tx_valid(tx_valid),.tx_ready(tx_ready),.committed(committed),.dropped(dropped),.duplicates(duplicates),.write_errors(write_errors));
reg [7:0] bytes[0:81];integer j,t;reg seen_aw=0,seen_w=0;integer writes=0;
always @(posedge clk)if(rstn)begin
 if(awvalid&&awready)begin if(awaddr!=64'heef01000)$fatal(1,"bad DMA address");seen_aw<=1;end
 if(wvalid&&wready)begin if(wdata!=512'd1)$fatal(1,"bad credit payload");seen_w<=1;writes<=writes+1;end
end
task packet;begin
 @(negedge clk);rx_data=0;for(j=0;j<64;j=j+1)rx_data[j*8+:8]=bytes[j];rx_keep=~64'd0;rx_valid=1;rx_last=0;
 @(negedge clk);rx_data=0;for(j=0;j<18;j=j+1)rx_data[j*8+:8]=bytes[j+64];rx_keep=64'h3ffff;rx_last=1;
 @(negedge clk);rx_valid=0;rx_last=0;
end endtask
initial begin
 $dumpfile("credit.vcd");$dumpvars(0,tb);$readmemh("packet.hex",bytes);
 repeat(5)@(negedge clk);rstn=1;psn_load=1;@(negedge clk);psn_load=0;
 packet();t=0;while(!seen_aw||!seen_w)begin @(negedge clk);t=t+1;if(t>300)$fatal(1,"credit rejected dropped=%d state=%d fields=%d",dropped,dut.state,dut.fields_ok_reg);end
 repeat(20)begin @(negedge clk);if(tx_valid||committed)$fatal(1,"ACK before B response");end
 bvalid=1;@(negedge clk);bvalid=0;
 t=0;while(!tx_valid)begin @(negedge clk);t=t+1;if(t>300)$fatal(1,"ACK absent");end
 if(committed!=1||tx_keep!=64'h3fffffffffffffff||tx_data[42*8+:8]!=8'h11)$fatal(1,"bad ACK");
 repeat(10)begin @(negedge clk);if(!tx_valid)$fatal(1,"ACK lost while stalled");end
 tx_ready=1;repeat(3)@(negedge clk);packet();t=0;
 while(duplicates!=1)begin @(negedge clk);t=t+1;if(t>300)$fatal(1,"duplicate failed");end
 if(writes!=1||write_errors||dropped)$fatal(1,"duplicate wrote twice or error");
 $display("CREDIT_UNIT_PASS cases=2 ACK_AFTER_B_AND_DUPLICATE_NO_SECOND_WRITE");$finish;
end
initial begin #10000;$fatal(1,"timeout");end
endmodule
