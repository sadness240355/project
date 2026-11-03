`timescale 1ns/1ps

// The configuration aperture is written rarely, but is read combinationally
// by the packet engines. Byte-wide distributed memories keep that read
// contract while avoiding thousands of resettable flip-flops and write muxes.
module lrdma_cfg_table #(
 parameter [31:0] RESET_VALUE = 32'd0
)(
 input wire clk, rstn, wen,
 input wire [4:0] waddr, raddr,
 input wire [31:0] wdata,
 input wire [3:0] wstrb,
 output wire [31:0] rdata
);
 reg [3:0] valid [0:31];
 integer i;
 always @(posedge clk) begin
  if (!rstn) begin
   for (i=0;i<32;i=i+1) valid[i] <= 4'd0;
  end else if (wen) begin
   valid[waddr] <= valid[waddr] | wstrb;
  end
 end
 wire [3:0] read_valid = valid[raddr];
 genvar b;
 generate for (b=0;b<4;b=b+1) begin: byte_lane
  (* ram_style = "distributed" *) reg [7:0] mem [0:31];
  always @(posedge clk) begin
   if (wen && wstrb[b] && rstn) mem[waddr] <= wdata[b*8+:8];
  end
  assign rdata[b*8+:8] = read_valid[b] ? mem[raddr] : RESET_VALUE[b*8+:8];
 end endgenerate
endmodule

module lrdma_compat_config(
 input wire clk,rstn,
 input wire [31:0] awaddr,wdata,input wire [3:0] wstrb,
 input wire awvalid,awready,wvalid,wready,
 input wire [4:0] rx_qp,db_qp,
 output wire enabled,output wire [47:0] local_mac,remote_mac,
 output wire [31:0] local_ip,remote_ip,rkey,access,
 output wire [63:0] virtual_base,dma_base,db_base,
 output wire [31:0] region_len,qp_conf,
 output wire [23:0] remote_qpn,last_psn,output wire [15:0] pkey,
 output reg psn_load,output reg [4:0] psn_load_qp,
 output reg [23:0] psn_load_value
);
 reg [31:0] global_conf,mac_lo,mac_hi,ip;
 reg [31:0] rpsn[0:31];
 reg ap,wp;reg [31:0] a,d;reg [3:0] st;
 integer i;
 function automatic [31:0] merge(input [31:0] old,input [31:0] value,input [3:0] mask);
  integer b;begin for(b=0;b<4;b=b+1)merge[b*8+:8]=mask[b]?value[b*8+:8]:old[b*8+:8];end
 endfunction
 wire [4:0] n=a[12:8];
 wire commit=ap&&wp;
 wire qp_write=commit&&(a[20:13]==8'hc0);
 wire pd_write=commit&&(a[20:13]==8'h00);
 wire [31:0] qp_rd,adv_rd,pd_rd,rmlo_rd,rmhi_rd,rip_rd,rqp_rd;
 wire [31:0] dblo_rd,dbhi_rd,vlo_rd,vhi_rd,dlo_rd,dhi_rd,key_rd,len_rd,perm_rd;
 wire [4:0] p=pd_rd[4:0];
 wire valid_pd=pd_rd<32;

 lrdma_cfg_table qp_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'h00),.waddr(n),.raddr(rx_qp),.wdata(d),.wstrb(st),.rdata(qp_rd));
 lrdma_cfg_table adv_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'h04),.waddr(n),.raddr(rx_qp),.wdata(d),.wstrb(st),.rdata(adv_rd));
 lrdma_cfg_table #(.RESET_VALUE(32'hffffffff)) pd_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'hb0),.waddr(n),.raddr(rx_qp),.wdata(d),.wstrb(st),.rdata(pd_rd));
 lrdma_cfg_table rmlo_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'h50),.waddr(n),.raddr(rx_qp),.wdata(d),.wstrb(st),.rdata(rmlo_rd));
 lrdma_cfg_table rmhi_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'h54),.waddr(n),.raddr(rx_qp),.wdata(d),.wstrb(st),.rdata(rmhi_rd));
 lrdma_cfg_table rip_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'h60),.waddr(n),.raddr(rx_qp),.wdata(d),.wstrb(st),.rdata(rip_rd));
 lrdma_cfg_table rqp_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'h48),.waddr(n),.raddr(rx_qp),.wdata(d),.wstrb(st),.rdata(rqp_rd));
 lrdma_cfg_table dblo_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'h28),.waddr(n),.raddr(db_qp),.wdata(d),.wstrb(st),.rdata(dblo_rd));
 lrdma_cfg_table dbhi_store(.clk(clk),.rstn(rstn),.wen(qp_write&&a[7:0]==8'h2c),.waddr(n),.raddr(db_qp),.wdata(d),.wstrb(st),.rdata(dbhi_rd));
 lrdma_cfg_table vlo_store(.clk(clk),.rstn(rstn),.wen(pd_write&&a[7:0]==8'h04),.waddr(n),.raddr(p),.wdata(d),.wstrb(st),.rdata(vlo_rd));
 lrdma_cfg_table vhi_store(.clk(clk),.rstn(rstn),.wen(pd_write&&a[7:0]==8'h08),.waddr(n),.raddr(p),.wdata(d),.wstrb(st),.rdata(vhi_rd));
 lrdma_cfg_table dlo_store(.clk(clk),.rstn(rstn),.wen(pd_write&&a[7:0]==8'h0c),.waddr(n),.raddr(p),.wdata(d),.wstrb(st),.rdata(dlo_rd));
 lrdma_cfg_table dhi_store(.clk(clk),.rstn(rstn),.wen(pd_write&&a[7:0]==8'h10),.waddr(n),.raddr(p),.wdata(d),.wstrb(st),.rdata(dhi_rd));
 lrdma_cfg_table key_store(.clk(clk),.rstn(rstn),.wen(pd_write&&a[7:0]==8'h14),.waddr(n),.raddr(p),.wdata(d),.wstrb(st),.rdata(key_rd));
 lrdma_cfg_table len_store(.clk(clk),.rstn(rstn),.wen(pd_write&&a[7:0]==8'h18),.waddr(n),.raddr(p),.wdata(d),.wstrb(st),.rdata(len_rd));
 lrdma_cfg_table perm_store(.clk(clk),.rstn(rstn),.wen(pd_write&&a[7:0]==8'h1c),.waddr(n),.raddr(p),.wdata(d),.wstrb(st),.rdata(perm_rd));

 assign enabled=global_conf[0];assign local_mac={mac_hi[15:0],mac_lo};assign local_ip=ip;
 assign qp_conf=qp_rd;assign pkey=adv_rd[31:16];assign remote_mac={rmhi_rd[15:0],rmlo_rd};
 assign remote_ip=rip_rd;assign remote_qpn=rqp_rd[23:0];assign last_psn=rpsn[rx_qp][23:0];
 assign virtual_base=valid_pd?{vhi_rd,vlo_rd}:64'd0;
 assign dma_base=valid_pd?{dhi_rd,dlo_rd}:64'd0;
 assign rkey=valid_pd?key_rd:32'd0;assign access=valid_pd?perm_rd:32'd0;
 assign region_len=valid_pd?len_rd:32'd0;assign db_base={dbhi_rd,dblo_rd};
 always @(posedge clk)begin
  if(!rstn)begin
   global_conf<=0;mac_lo<=0;mac_hi<=0;ip<=0;ap<=0;wp<=0;psn_load<=0;psn_load_qp<=0;psn_load_value<=0;
   for(i=0;i<32;i=i+1)rpsn[i]<=0;
  end else begin
   psn_load<=0;
   if(awvalid&&awready)begin a<=awaddr;ap<=1;end
   if(wvalid&&wready)begin d<=wdata;st<=wstrb;wp<=1;end
   if(commit)begin
    ap<=0;wp<=0;
    if(a[20:0]==21'h100000)global_conf<=merge(global_conf,d,st);
    if(a[20:0]==21'h100010)mac_lo<=merge(mac_lo,d,st);
    if(a[20:0]==21'h100014)mac_hi<=merge(mac_hi,d,st);
    if(a[20:0]==21'h100070)ip<=merge(ip,d,st);
    if(qp_write&&a[7:0]==8'h44)begin
     rpsn[n]<=merge(rpsn[n],d,st);psn_load<=1;psn_load_qp<=n;psn_load_value<=merge(rpsn[n],d,st);
    end
   end
  end
 end
endmodule
