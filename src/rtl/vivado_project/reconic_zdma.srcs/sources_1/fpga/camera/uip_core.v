// SPDX-License-Identifier: MIT
// UIP1: 25 enabled-clock stages, one LANES-pixel beat per enabled clock.
// Every data and sideband stage freezes together under downstream backpressure.
// Configuration/LUT changes remain stopped-session operations.
// Arithmetic is split at difference, comparison, mux, product and rounding
// boundaries; do not merge these registers to recover the old combinational path.
(* keep_hierarchy="yes" *) module uip_core #(
 parameter LANES=8, CHANNELS=1, MAX_WIDTH=2432,
 parameter DATA_WIDTH=LANES*CHANNELS*8,
 parameter WORDS=(MAX_WIDTH+LANES-1)/LANES
)(
 input wire clk,rst,ce,
 input wire s_valid,s_sof,s_eol,input wire [DATA_WIDTH-1:0] s_data,
 output wire m_valid,m_sof,m_eol,output wire [DATA_WIDTH-1:0] m_data,
 input wire bayer,input wire [271:0] config_data,
 input wire config_clk,lut_we,input wire [7:0] lut_addr,lut_data,
 // EOF is delayed inside the pixel pipeline, not in a separately sized adapter.
 input wire s_last,output wire m_last
);
 localparam N=LANES*CHANNELS;
 localparam AW=$clog2(WORDS);
 localparam PIPE_STAGES=25;
 localparam [AW:0] WORD_LIMIT=WORDS;
 reg [AW-1:0] column=0;
 reg [15:0] row=0;
 wire [AW-1:0] col=s_sof ? {AW{1'b0}}:column;
 wire [15:0] y=s_sof ? 16'd0:row;
 wire [N*8-1:0] scaled;
 reg [N*8-1:0] previous=0;
 wire [N*8-1:0] corrected;
 reg [PIPE_STAGES-1:0] valid_pipe=0,sof_pipe=0,eol_pipe=0,last_pipe=0;
 assign m_valid=valid_pipe[PIPE_STAGES-1];
 assign m_sof=sof_pipe[PIPE_STAGES-1];
 assign m_eol=eol_pipe[PIPE_STAGES-1];
 assign m_last=last_pipe[PIPE_STAGES-1];
 always @(posedge clk)begin
  if(rst)begin
   column<=0;row<=0;previous<=0;
   valid_pipe<=0;sof_pipe<=0;eol_pipe<=0;last_pipe<=0;
  end else if(ce)begin
   valid_pipe<={valid_pipe[PIPE_STAGES-2:0],s_valid};
   sof_pipe<={sof_pipe[PIPE_STAGES-2:0],s_valid && s_sof};
   eol_pipe<={eol_pipe[PIPE_STAGES-2:0],s_valid && s_eol};
   last_pipe<={last_pipe[PIPE_STAGES-2:0],s_valid && s_last};
   if(s_valid)begin
    column<=s_eol ? {AW{1'b0}}:col+1'b1;
    row<=s_eol ? y+1'b1:y;
    previous<=corrected;
   end
  end
 end
 // Session-constant threshold predecode: keep two's-complement generation
 // out of the per-pixel compare path. Samples reach the compare stages only
 // after these registers have advanced; profile COMMIT waits 32 pixel clocks.
 reg signed [8:0] defect_hi=0,defect_lo=0,noise_hi=0,noise_lo=0;
 reg signed [9:0] coring_hi=0,coring_lo=0;
 always @(posedge clk)if(ce)begin
  defect_hi<=$signed({1'b0,config_data[87:80]});
  defect_lo<=-$signed({1'b0,config_data[87:80]});
  noise_hi<=$signed({1'b0,config_data[71:64]});
  noise_lo<=-$signed({1'b0,config_data[71:64]});
  coring_hi<=$signed({4'b0,config_data[79:74]});
  coring_lo<=-$signed({4'b0,config_data[79:74]});
 end
 genvar n;
 generate for(n=0;n<N;n=n+1)begin: sample_lanes
  wire [7:0] raw=s_data[n*8 +: 8];
  // Profiles change only after drain/settle. Keep operands per lane so the
  // distant profile/reset net does not drive all subtractors and DSP ports.
  (* DONT_TOUCH="TRUE" *) reg [7:0] black_level_local=0;
  (* DONT_TOUCH="TRUE" *) reg black_enable_local=0;
  (* DONT_TOUCH="TRUE" *) reg [7:0] detail_gain8=0;
  always @(posedge clk) if (ce) begin
   black_level_local<=config_data[15:8];
   black_enable_local<=config_data[0];
   detail_gain8<=config_data[95:88];
  end
  assign corrected[n*8 +: 8]=black_enable_local ?
      (raw>black_level_local ? raw-black_level_local:8'd0):raw;
  wire [7:0] current=corrected[n*8 +: 8];
  (* ram_style="block" *) reg [7:0] line0[0:WORDS-1];
  (* ram_style="block" *) reg [7:0] line1[0:WORDS-1];
  reg [7:0] above0=0,above1=0,pixel0=0,left0=0;
  reg upper_bank0=0,left_ok0=0,above_ok0=0;
  wire [7:0] above=upper_bank0 ? above1:above0;
  wire odd_x=((n/CHANNELS)%2 != 0) ^ config_data[72];
  wire odd_y=y[0] ^ config_data[73];
  wire [11:0] selected_gain=CHANNELS==3 ?
      (n%3==0 ? config_data[27:16]:n%3==1 ? config_data[43:32]:config_data[59:48]):
      !bayer ? 12'd256:(!odd_y && !odd_x ? config_data[27:16]:
                       odd_y && odd_x ? config_data[59:48]:config_data[43:32]);
  wire [7:0] left_raw;
  if(CHANNELS==3)begin
   assign left_raw=n>=3 ? corrected[(n>=3 ? n-3:0)*8 +: 8]:
                         previous[(N-3+n%3)*8 +: 8];
  end else begin
   wire [7:0] left_one=n>=1 ? corrected[(n>=1 ? n-1:0)*8 +: 8]:previous[(N-1)*8 +: 8];
   wire [7:0] left_two=n>=2 ? corrected[(n>=2 ? n-2:0)*8 +: 8]:previous[(N-2+n%2)*8 +: 8];
   assign left_raw=bayer ? left_two:left_one;
  end
  wire read_word=ce && s_valid && {1'b0,col}<WORD_LIMIT;
  wire upper_bank=CHANNELS==1 && bayer && y[0];
  // Read before write. Invalid neighbours are masked at row/column boundaries.
  always @(posedge clk)if(read_word && !upper_bank)begin above0<=line0[col];line0[col]<=current;end
  always @(posedge clk)if(read_word && upper_bank)begin above1<=line1[col];line1[col]<=current;end

  // 0: corrected centre/neighbours and Bayer-dependent gain.
  reg [11:0] gain_pipe[0:13];
  // Keep the final operand local to each lane. Equivalent gains in RGB lanes
  // must not collapse to one distant, high-fanout register driving many DSPs.
  (* DONT_TOUCH="TRUE" *) reg [11:0] gain_local14=12'd256;
  integer gi,init_gain;
  initial for(init_gain=0;init_gain<14;init_gain=init_gain+1)gain_pipe[init_gain]=12'd256;
  // 1: signed differences and neighbour sum. No abs/subtract/mux chain.
  reg signed [8:0] dl1=0,du1=0,lu1=0;
  reg [8:0] replacement1=0;
  reg [7:0] center1=0,left1=0,up1=0;
  reg lok1=0,uok1=0;
  // 2: threshold result, separately registered from both subtract and data mux.
  reg defect2=0;
  reg [7:0] replacement2=0,center2=0,left2=0,up2=0;
  reg lok2=0,uok2=0;
  // 3: select corrected centre.
  reg [7:0] center3=0,left3=0,up3=0;
  reg lok3=0,uok3=0;
  // 4: denoise differences.
  reg signed [8:0] dl4=0,du4=0;
  reg [7:0] center4=0,left4=0,up4=0;
  reg lok4=0,uok4=0;
  // 5: edge-preserving neighbour selection.
  reg [7:0] center5=0,left5=0,up5=0;
  // 6/7: two-level averaging adder tree.
  reg [9:0] average_left6=0,average_up6=0;
  reg [7:0] center6=0,center7=0,average7=0;
  wire [9:0] average_sum=average_left6+average_up6;
  // 8: detail subtraction. Coring comparison and product follow one clock later.
  reg signed [9:0] detail8=0;
  reg [7:0] base8=0,base9=0,base10=0,base11=0;
  // 9: product; 10: rounding; 11: limit; 12: add; 13: saturate.
  (* use_dsp="yes" *) reg signed [18:0] detail_product9=0;
  reg detail_enable9=0;
  reg signed [18:0] detail_round10=0;
  reg signed [10:0] detail_limit11=0,enhanced12=0;
  reg [7:0] detailed13=0,wb_pixel14=0;
  // 14: dedicated DSP input; 15: product; 16: round; 17: output.
  (* use_dsp="yes" *) reg [19:0] product15=0;
  reg [19:0] rounded16=0;
  reg saturate16=0;
  reg [7:0] scaled17=0;

  always @(posedge clk)if(ce)begin
   pixel0<=current;left0<=left_raw;upper_bank0<=upper_bank;
   left_ok0<=col!=0 || (n/CHANNELS)>=(CHANNELS==1 && bayer ? 2:1);
   above_ok0<=y>=(CHANNELS==1 && bayer ? 2:1);
   gain_pipe[0]<=config_data[1] ? selected_gain:12'd256;
   for(gi=1;gi<14;gi=gi+1)gain_pipe[gi]<=gain_pipe[gi-1];
   gain_local14<=gain_pipe[13];

   dl1<=$signed({1'b0,pixel0})-$signed({1'b0,left0});
   du1<=$signed({1'b0,pixel0})-$signed({1'b0,above});
   lu1<=$signed({1'b0,left0})-$signed({1'b0,above});
   replacement1<={1'b0,left0}+{1'b0,above}+9'd1;
   center1<=pixel0;left1<=left0;up1<=above;lok1<=left_ok0;uok1<=above_ok0;

   defect2<=config_data[4] && lok1 && uok1 &&
       ((dl1>defect_hi && du1>defect_hi)||(dl1<defect_lo && du1<defect_lo)) &&
       lu1>=defect_lo && lu1<=defect_hi;
   replacement2<=replacement1[8:1];center2<=center1;
   left2<=left1;up2<=up1;lok2<=lok1;uok2<=uok1;

   center3<=defect2 ? replacement2:center2;
   left3<=left2;up3<=up2;lok3<=lok2;uok3<=uok2;

   dl4<=$signed({1'b0,center3})-$signed({1'b0,left3});
   du4<=$signed({1'b0,center3})-$signed({1'b0,up3});
   center4<=center3;left4<=left3;up4<=up3;lok4<=lok3;uok4<=uok3;

   center5<=center4;
   left5<=lok4 && dl4>=noise_lo && dl4<=noise_hi ? left4:center4;
   up5<=uok4 && du4>=noise_lo && du4<=noise_hi ? up4:center4;

   average_left6<={1'b0,center5,1'b0}+{2'b0,left5};
   average_up6<={2'b0,up5}+10'd2;center6<=center5;
   average7<=average_sum[9:2];center7<=center6;
   detail8<=$signed({2'b0,center7})-$signed({2'b0,average7});
   base8<=config_data[2] ? average7:center7;

   detail_product9<=detail8*$signed({1'b0,detail_gain8});
   detail_enable9<=config_data[5] && !bayer && (detail8>coring_hi || detail8<coring_lo);
   base9<=base8;
   detail_round10<=detail_enable9 ? (detail_product9+19'sd8)>>>4:19'sd0;
   base10<=base9;
   detail_limit11<=detail_round10>19'sd32 ? 11'sd32:
                   detail_round10< -19'sd32 ? -11'sd32:detail_round10[10:0];
   base11<=base10;
   enhanced12<=$signed({3'b0,base11})+detail_limit11;
   detailed13<=enhanced12<0 ? 8'd0:enhanced12>11'sd255 ? 8'd255:enhanced12[7:0];
   wb_pixel14<=detailed13;
   product15<=wb_pixel14*gain_local14;
   rounded16<=product15+20'd128;saturate16<=product15>20'd65280;
   scaled17<=saturate16 ? 8'd255:rounded16[15:8];
  end
  assign scaled[n*8 +: 8]=scaled17;
 end endgenerate

 // 18: RGB/coefficient operands; 19: multiply; 20/21: balanced sum;
 // 22: Q3.12 rounding; 23: saturate/bypass; 24: LUT/output.
 // Bypass paths have exactly the same latency as active processing.
 generate for(n=0;n<N;n=n+1)begin: color_lanes
  reg [7:0] bypass18=0,bypass19=0,bypass20=0,bypass21=0,bypass22=0;
  reg [7:0] color23=0,result24=0;
  if(CHANNELS==3)begin: rgb_matrix
   reg signed [8:0] r18=0,g18=0,b18=0;
   // Preserve the existing stage-18 coefficient registers per DSP lane.
   (* DONT_TOUCH="TRUE" *) reg signed [15:0] c0_18=0,c1_18=0,c2_18=0;
   (* use_dsp="yes" *) reg signed [24:0] p0_19=0,p1_19=0,p2_19=0;
   reg signed [26:0] sum01_20=0,p2_20=0,sum21=0,rounded22=0;
   always @(posedge clk)if(ce)begin
    r18<=$signed({1'b0,scaled[(n/3*3)*8 +:8]});
    g18<=$signed({1'b0,scaled[(n/3*3+1)*8 +:8]});
    b18<=$signed({1'b0,scaled[(n/3*3+2)*8 +:8]});
    c0_18<=config_data[128+((n%3)*3)*16 +:16];
    c1_18<=config_data[128+((n%3)*3+1)*16 +:16];
    c2_18<=config_data[128+((n%3)*3+2)*16 +:16];
    p0_19<=r18*c0_18;p1_19<=g18*c1_18;p2_19<=b18*c2_18;
    sum01_20<={{2{p0_19[24]}},p0_19}+{{2{p1_19[24]}},p1_19};
    p2_20<={{2{p2_19[24]}},p2_19};
    sum21<=sum01_20+p2_20;
    rounded22<=(sum21+27'sd2048)>>>12;
    color23<=!config_data[6] ? bypass22:
             rounded22<0 ? 8'd0:rounded22>27'sd255 ? 8'd255:rounded22[7:0];
   end
  end else begin
   always @(posedge clk)if(ce)color23<=bypass22;
  end
  (* ram_style="distributed" *) reg [7:0] tone[0:255];
  integer j;initial for(j=0;j<256;j=j+1)tone[j]=j[7:0];
  always @(posedge config_clk)if(lut_we)tone[lut_addr]<=lut_data;
  always @(posedge clk)if(ce)begin
   bypass18<=scaled[n*8 +:8];bypass19<=bypass18;
   bypass20<=bypass19;bypass21<=bypass20;bypass22<=bypass21;
   result24<=config_data[3] ? tone[color23]:color23;
  end
  assign m_data[n*8 +:8]=result24;
 end endgenerate
endmodule
