`timescale 1ns/1ps
// Deliberately restricted RC responder for the existing video/result credit
// mailboxes. IPv4, WRITE_ONLY, exactly 8 payload bytes at the registered base.
// All other incoming operations remain unsupported. No ACK precedes an OKAY
// AXI write response. Duplicate PSNs ACK again without writing a second time.
// Packets arriving while busy are dropped (no RX backpressure); RC retries.
module lrdma_credit_rx(
 input wire clk,rstn,input wire [511:0] rx_data,input wire [63:0] rx_keep,
 input wire rx_valid,rx_last,rx_error,
 output wire [4:0] lookup_qp,input wire enabled,
 input wire [47:0] local_mac,remote_mac,input wire [31:0] local_ip,remote_ip,qp_conf,
 input wire [23:0] remote_qpn,input wire [15:0] pkey,
 input wire [63:0] virtual_base,dma_base,input wire [31:0] rkey,region_len,access,
 input wire psn_load,input wire [4:0] psn_load_qp,input wire [23:0] psn_load_value,
 output reg [63:0] awaddr,output reg awvalid,input wire awready,
 output reg [511:0] wdata,output reg wvalid,input wire wready,
 input wire [1:0] bresp,input wire bvalid,output wire bready,
 output reg [511:0] tx_data,output wire [63:0] tx_keep,output reg tx_valid,input wire tx_ready,
 output reg [31:0] committed,dropped,duplicates,write_errors,
 output wire [31:0] debug_flags
);
 localparam IDLE=0,SECOND=1,CHECK=2,SEED=3,CRC_RX=4,VERIFY=5,
            WRITE=6,WAIT_B=7,BUILD_ACK=8,SEED_ACK=9,CRC_ACK=10,PUBLISH=11,WAIT_TX=12,
            CHECK_DECIDE=13,CHECK_PSN=14;
 reg [3:0] state;reg in_frame;
 reg [1023:0] frame;reg [495:0] ack;
 reg [23:0] last[0:31],msn[0:31];reg [31:0] crc;
 reg [31:0] psn_valid,has_commit;
 reg [6:0] byte_index;reg [3:0] seed_index;
 reg [16:0] ip_pair_sum[0:4];
 reg [19:0] ip_sum;
 reg fields_ok_reg;
 reg psn_valid_stage,repeat_ok_stage;
 reg [23:0] expected_psn;
 reg [63:0] dma_base_stage;
 reg aw_done,w_done,duplicate;
 reg [23:0] peer_qpn,packet_psn;reg [4:0] q;
 integer i;
 function automatic [7:0] b(input [1023:0] packet,input integer n);b=packet[n*8+:8];endfunction
 function automatic [15:0] be16(input [1023:0] packet,input integer n);be16={b(packet,n),b(packet,n+1)};endfunction
 function automatic [23:0] be24(input [1023:0] packet,input integer n);be24={b(packet,n),b(packet,n+1),b(packet,n+2)};endfunction
 function automatic [31:0] be32(input [1023:0] packet,input integer n);be32={b(packet,n),b(packet,n+1),b(packet,n+2),b(packet,n+3)};endfunction
 function automatic [63:0] be64(input [1023:0] packet,input integer n);be64={be32(packet,n),be32(packet,n+4)};endfunction
 function automatic [47:0] be48(input [1023:0] packet,input integer n);be48={be16(packet,n),be32(packet,n+2)};endfunction
 function automatic [31:0] crc_byte(input [31:0] old,input [7:0] data);
  reg [31:0] c;integer k;begin c=old^{24'd0,data};for(k=0;k<8;k=k+1)c=(c>>1)^((c[0])?32'hedb88320:32'd0);crc_byte=c;end
 endfunction
 function automatic [7:0] masked(input [7:0] value,input integer index);
  masked=(index==15||index==22||index==24||index==25||index==40||index==41||index==46)?8'hff:value;
 endfunction
 function automatic [495:0] make_ack(input [1023:0] packet,input [23:0] dest,psn,seq);
  reg [495:0] a;reg [31:0] sum;integer j;
  begin
   a=0;
   for(j=0;j<6;j=j+1)begin a[j*8+:8]=b(packet,j+6);a[(j+6)*8+:8]=b(packet,j);end
   a[12*8+:16]=16'h0008;a[14*8+:8]=8'h45;a[16*8+:16]=16'h3000;
   a[20*8+:16]=16'h0040;a[22*8+:8]=8'h40;a[23*8+:8]=8'h11;
   for(j=0;j<4;j=j+1)begin a[(26+j)*8+:8]=b(packet,30+j);a[(30+j)*8+:8]=b(packet,26+j);end
   sum=0;for(j=14;j<34;j=j+2)sum=sum+{a[j*8+:8],a[(j+1)*8+:8]};
   sum=(sum&65535)+(sum>>16);sum=(sum&65535)+(sum>>16);sum=~sum;
   a[24*8+:8]=sum[15:8];a[25*8+:8]=sum[7:0];
   a[34*8+:16]=16'hb712;a[36*8+:16]=16'hb712;a[38*8+:16]=16'h1c00;
   a[42*8+:8]=8'h11;a[44*8+:8]=b(packet,44);a[45*8+:8]=b(packet,45);
   a[47*8+:8]=dest[23:16];a[48*8+:8]=dest[15:8];a[49*8+:8]=dest[7:0];
   a[51*8+:8]=psn[23:16];a[52*8+:8]=psn[15:8];a[53*8+:8]=psn[7:0];
   a[54*8+:8]=8'h1f;
   a[55*8+:8]=seq[23:16];a[56*8+:8]=seq[15:8];a[57*8+:8]=seq[7:0];
   make_ack=a;
  end
 endfunction
 assign lookup_qp={frame[47*8+:8],frame[48*8+:8],frame[49*8+:8]}-1'b1;
 assign bready=state==WAIT_B;
 assign tx_keep=64'h3fffffffffffffff;
 wire [18:0] ip_quad01={2'b0,ip_pair_sum[0]}+{2'b0,ip_pair_sum[1]};
 wire [18:0] ip_quad23={2'b0,ip_pair_sum[2]}+{2'b0,ip_pair_sum[3]};
 wire [16:0] ip_fold1={1'b0,ip_sum[15:0]}+{13'b0,ip_sum[19:16]};
 wire [16:0] ip_fold2={1'b0,ip_fold1[15:0]}+{16'b0,ip_fold1[16]};
 logic fields_ok;
 always_comb begin
 fields_ok=enabled&&qp_conf[0]&&!qp_conf[7]&&
  ({frame[376+:8],frame[384+:8],frame[392+:8]}==3||{frame[376+:8],frame[384+:8],frame[392+:8]}==4)&&{frame[0+:8],frame[8+:8],frame[16+:8],frame[24+:8],frame[32+:8],frame[40+:8]}==local_mac&&{frame[48+:8],frame[56+:8],frame[64+:8],frame[72+:8],frame[80+:8],frame[88+:8]}==remote_mac&&
  b(frame,14)==8'h45&&{frame[96+:8],frame[104+:8]}==16'h0800&&{frame[128+:8],frame[136+:8]}==68&&
  ({frame[160+:8],frame[168+:8]}&16'hbfff)==0&&b(frame,23)==17&&{frame[208+:8],frame[216+:8],frame[224+:8],frame[232+:8]}==remote_ip&&{frame[240+:8],frame[248+:8],frame[256+:8],frame[264+:8]}==local_ip&&
  {frame[288+:8],frame[296+:8]}==4791&&{frame[304+:8],frame[312+:8]}==48&&{frame[320+:8],frame[328+:8]}==0&&b(frame,42)==8'h0a&&(b(frame,43)&8'hbf)==0&&{frame[352+:8],frame[360+:8]}==pkey&&
  {frame[432+:8],frame[440+:8],frame[448+:8],frame[456+:8],frame[464+:8],frame[472+:8],frame[480+:8],frame[488+:8]}==virtual_base&&{frame[496+:8],frame[504+:8],frame[512+:8],frame[520+:8]}==rkey&&rkey!=0&&{frame[528+:8],frame[536+:8],frame[544+:8],frame[552+:8]}==8&&
  region_len>=8&&access[1]&&access[31:16]==0&&dma_base[5:0]==0;
 end
 // Read-only diagnostic outputs; do not alter receive/commit decisions.
 assign debug_flags={11'd0,state,fields_ok_reg,(~crc==frame[78*8+:32]),
  (ip_fold2[15:0]==16'hffff),(packet_psn==expected_psn||repeat_ok_stage),psn_valid_stage,
  (dma_base[5:0]==0),(access[1]&&access[31:16]==0),(region_len>=8),
  (rkey!=0&&{frame[496+:8],frame[504+:8],frame[512+:8],frame[520+:8]}==rkey),
  ({frame[432+:8],frame[440+:8],frame[448+:8],frame[456+:8],frame[464+:8],frame[472+:8],frame[480+:8],frame[488+:8]}==virtual_base),
  (b(frame,42)==8'h0a&&(b(frame,43)&8'hbf)==0),
  ({frame[208+:8],frame[216+:8],frame[224+:8],frame[232+:8]}==remote_ip&&{frame[240+:8],frame[248+:8],frame[256+:8],frame[264+:8]}==local_ip),
  ({frame[48+:8],frame[56+:8],frame[64+:8],frame[72+:8],frame[80+:8],frame[88+:8]}==remote_mac),
  ({frame[0+:8],frame[8+:8],frame[16+:8],frame[24+:8],frame[32+:8],frame[40+:8]}==local_mac),
  !qp_conf[7],qp_conf[0],enabled};
 always @(posedge clk)begin
  if(!rstn)begin
   state<=IDLE;in_frame<=0;frame<=0;awvalid<=0;wvalid<=0;tx_valid<=0;
   awaddr<=0;wdata<=0;tx_data<=0;committed<=0;dropped<=0;duplicates<=0;write_errors<=0;
   psn_valid<=0;has_commit<=0;fields_ok_reg<=0;ip_sum<=0;
   psn_valid_stage<=0;repeat_ok_stage<=0;expected_psn<=0;dma_base_stage<=0;
   for(i=0;i<5;i=i+1)ip_pair_sum[i]<=0;
   for(i=0;i<32;i=i+1)begin last[i]<=0;msn[i]<=0;end
  end else begin
   if(psn_load)begin last[psn_load_qp]<=psn_load_value;msn[psn_load_qp]<=0;psn_valid[psn_load_qp]<=1;has_commit[psn_load_qp]<=0;end
   if(rx_valid)in_frame<=!rx_last;
   case(state)
    IDLE:if(rx_valid&&!in_frame&&rx_data[42*8+:8]==8'h0a)begin
     if(!rx_last&&rx_keep==64'hffffffffffffffff)begin frame[511:0]<=rx_data;state<=SECOND;end
     else dropped<=dropped+1'b1;
    end
    SECOND:if(rx_valid)begin
     if(rx_last&&rx_keep==64'h3ffff&&!rx_error)begin
      frame[1023:512]<=rx_data;
      // Pairwise IPv4 header sums are computed from the first beat, which
      // has been stable since IDLE.  This removes the ten-word carry chain
      // from the packet-decision path without changing checksum semantics.
      ip_pair_sum[0]<={1'b0,be16(frame,14)}+{1'b0,be16(frame,16)};
      ip_pair_sum[1]<={1'b0,be16(frame,18)}+{1'b0,be16(frame,20)};
      ip_pair_sum[2]<={1'b0,be16(frame,22)}+{1'b0,be16(frame,24)};
      ip_pair_sum[3]<={1'b0,be16(frame,26)}+{1'b0,be16(frame,28)};
      ip_pair_sum[4]<={1'b0,be16(frame,30)}+{1'b0,be16(frame,32)};
      state<=CHECK;
     end
     else begin dropped<=dropped+1'b1;state<=IDLE;end
    end
    CHECK:begin
     q<=lookup_qp;packet_psn<={frame[408+:8],frame[416+:8],frame[424+:8]};peer_qpn<=remote_qpn;
     duplicate<={frame[408+:8],frame[416+:8],frame[424+:8]}==last[lookup_qp];
     ip_sum<={1'b0,ip_quad01}+{1'b0,ip_quad23}+{3'b0,ip_pair_sum[4]};
     fields_ok_reg<=fields_ok;
     state<=CHECK_PSN;
    end
    CHECK_PSN:begin
     // Capture the same QP state that the original CHECK_DECIDE cycle read.
     // The registered comparison operands remove the QP mux and PSN adder
     // from the CRC seed/control path without changing AXI valid/ready.
     psn_valid_stage<=psn_valid[q];
     expected_psn<=last[q]+24'd1;
     repeat_ok_stage<=duplicate&&has_commit[q];
     dma_base_stage<=dma_base;
     state<=CHECK_DECIDE;
    end
    CHECK_DECIDE:begin
     if(fields_ok_reg&&ip_fold2[15:0]==16'hffff&&psn_valid_stage&&
        (packet_psn==expected_psn||repeat_ok_stage))begin
      awaddr<=dma_base_stage;wdata<={448'd0,frame[70*8+:64]};crc<=32'hffffffff;seed_index<=0;state<=SEED;
     end else begin dropped<=dropped+1'b1;state<=IDLE;end
    end
    SEED:begin crc<=crc_byte(crc,8'hff);if(seed_index==7)begin byte_index<=14;state<=CRC_RX;end else seed_index<=seed_index+1'b1;end
    CRC_RX:begin crc<=crc_byte(crc,masked(b(frame,byte_index),byte_index));if(byte_index==77)state<=VERIFY;else byte_index<=byte_index+1'b1;end
    VERIFY:if((~crc)==frame[78*8+:32])begin
     if(duplicate)begin duplicates<=duplicates+1'b1;state<=BUILD_ACK;end
     else begin awvalid<=1;wvalid<=1;aw_done<=0;w_done<=0;state<=WRITE;end
    end else begin dropped<=dropped+1'b1;state<=IDLE;end
    WRITE:begin
     if(awvalid&&awready)begin awvalid<=0;aw_done<=1;end
     if(wvalid&&wready)begin wvalid<=0;w_done<=1;end
     if((aw_done||(awvalid&&awready))&&(w_done||(wvalid&&wready)))state<=WAIT_B;
    end
    WAIT_B:if(bvalid)begin
     if(bresp==0)begin last[q]<=packet_psn;has_commit[q]<=1;msn[q]<=msn[q]+1'b1;committed<=committed+1'b1;state<=BUILD_ACK;end
     else begin write_errors<=write_errors+1'b1;state<=IDLE;end
    end
    BUILD_ACK:begin ack<=make_ack(frame,peer_qpn,packet_psn,msn[q]);crc<=32'hffffffff;seed_index<=0;state<=SEED_ACK;end
    SEED_ACK:begin crc<=crc_byte(crc,8'hff);if(seed_index==7)begin byte_index<=14;state<=CRC_ACK;end else seed_index<=seed_index+1'b1;end
    CRC_ACK:begin crc<=crc_byte(crc,masked(ack[byte_index*8+:8],byte_index));if(byte_index==57)state<=PUBLISH;else byte_index<=byte_index+1'b1;end
    PUBLISH:begin tx_data<={16'd0,~crc,ack[463:0]};tx_valid<=1;state<=WAIT_TX;end
    WAIT_TX:if(tx_ready)begin tx_valid<=0;state<=IDLE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
