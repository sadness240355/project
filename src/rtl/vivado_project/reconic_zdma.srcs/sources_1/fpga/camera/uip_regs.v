// SPDX-License-Identifier: MIT
// UIP1 bus-neutral register bank. Address is relative to the 2 KiB UIP window.
// cfg config_mailbox: source holds all fields until the pixel clock acknowledges COMMIT.
module uip_regs (
 input wire clk,rst,idle,
 input wire wr_en,input wire [10:0] wr_addr,input wire [31:0] wr_data,
 output reg wr_error,
 input wire [10:0] rd_addr,output reg [31:0] rd_data,
 input wire pixel_clk,pixel_rst,
 output reg [271:0] pixel_config,
 output wire lut_we,output wire [7:0] lut_addr,lut_data,
 output wire ready,output wire [31:0] generation,output wire [6:0] active_mask
);
 localparam [271:0] DEFAULT_CONFIG={16'd4096,48'd0,16'd4096,48'd0,16'd4096,32'd0,22'd0,2'd0,8'd16,4'd0,12'd256,4'd0,12'd256,4'd0,12'd256,16'd0};
 initial pixel_config=DEFAULT_CONFIG;
 reg [271:0] shadow=DEFAULT_CONFIG,config_mailbox=DEFAULT_CONFIG;
 reg request=0,ack=0,dirty=0;
 reg [2:0] settle=0;reg initialized=0,applied_toggle=0;reg [5:0] drain_count=0;
 (* ASYNC_REG="TRUE" *) reg [1:0] req_sync=0,ack_sync=0;
 wire pending=request!=ack_sync[1];
 assign ready=!pending && !dirty;
 assign generation=config_mailbox[127:96];assign active_mask=config_mailbox[6:0];
 (* ram_style="distributed" *) reg [7:0] lut_copy[0:255];
 integer i;initial for(i=0;i<256;i=i+1)lut_copy[i]=i[7:0];
 wire lut_select=wr_addr>=11'h400 && wr_addr<=11'h7fc && wr_addr[1:0]==0;
 assign lut_we=wr_en && !wr_error && lut_select;
 assign lut_addr=wr_addr[9:2];assign lut_data=wr_data[7:0];
 always @* begin
  wr_error=!idle || pending || wr_addr[1:0]!=0;
  if(lut_select)begin if(wr_data>255)wr_error=1;end
  else case(wr_addr)
   'h10:if(wr_data>127)wr_error=1;
   'h14,'h24:if(wr_data>255)wr_error=1;
   'h18,'h1c,'h20:if(wr_data>4095)wr_error=1;
   'h28:if(wr_data>3)wr_error=1;
   'h2c:if(wr_data>255)wr_error=1;
   'h38:if(wr_data>32)wr_error=1;
   'h3c:if(wr_data>63)wr_error=1;
   'h30:if(wr_data!=1)wr_error=1;
   default:if(!(wr_addr>='h80 && wr_addr<='ha0) || wr_data[31:16]!={16{wr_data[15]}})wr_error=1;
  endcase
 end
 always @* begin
  rd_data=0;
  case(rd_addr)
   'h00:rd_data=32'h55495031;
   'h04:rd_data=32'h00010001;
   'h08:rd_data=32'h0000007f;
   'h0c:rd_data={29'd0,dirty,pending,!idle};
   'h10:rd_data={25'd0,shadow[6:0]};'h14:rd_data={24'd0,shadow[15:8]};
   'h18:rd_data={20'd0,shadow[27:16]};'h1c:rd_data={20'd0,shadow[43:32]};'h20:rd_data={20'd0,shadow[59:48]};
   'h24:rd_data={24'd0,shadow[71:64]};'h28:rd_data={30'd0,shadow[73:72]};
   'h2c:rd_data={24'd0,shadow[87:80]};'h38:rd_data={24'd0,shadow[95:88]};'h3c:rd_data={26'd0,shadow[79:74]};
   'h6c:rd_data={24'd0,config_mailbox[87:80]};'h70:rd_data={24'd0,config_mailbox[95:88]};'h74:rd_data={26'd0,config_mailbox[79:74]};
   'h34:rd_data=generation;
   'h50:rd_data={25'd0,config_mailbox[6:0]};'h54:rd_data={24'd0,config_mailbox[15:8]};
   'h58:rd_data={20'd0,config_mailbox[27:16]};'h5c:rd_data={20'd0,config_mailbox[43:32]};'h60:rd_data={20'd0,config_mailbox[59:48]};
   'h64:rd_data={24'd0,config_mailbox[71:64]};'h68:rd_data={30'd0,config_mailbox[73:72]};
   default:if(rd_addr>='h80 && rd_addr<='ha0 && rd_addr[1:0]==0)rd_data={{16{shadow[128+((rd_addr-'h80)>>2)*16+15]}},shadow[128+((rd_addr-'h80)>>2)*16 +:16]};
   else if(rd_addr>='hc0 && rd_addr<='he0 && rd_addr[1:0]==0)rd_data={{16{config_mailbox[128+((rd_addr-'hc0)>>2)*16+15]}},config_mailbox[128+((rd_addr-'hc0)>>2)*16 +:16]};
   else if(rd_addr>='h400 && rd_addr<='h7fc && rd_addr[1:0]==0)rd_data={24'd0,lut_copy[rd_addr[9:2]]};
  endcase
 end
 always @(posedge clk)begin
  ack_sync<={ack_sync[0],ack};
  if(rst)begin shadow<=DEFAULT_CONFIG;config_mailbox<=DEFAULT_CONFIG;request<=0;dirty<=0;ack_sync<=0;end
  else if(wr_en && !wr_error)begin
   if(lut_select)begin lut_copy[lut_addr]<=lut_data;dirty<=1;end
   else if(wr_addr=='h30)begin config_mailbox<=shadow;config_mailbox[127:96]<=config_mailbox[127:96]+1'b1;request<=~request;dirty<=0;end
   else begin
    dirty<=1;
    case(wr_addr)
     'h10:shadow[6:0]<=wr_data[6:0];'h14:shadow[15:8]<=wr_data[7:0];
     'h18:shadow[27:16]<=wr_data[11:0];'h1c:shadow[43:32]<=wr_data[11:0];'h20:shadow[59:48]<=wr_data[11:0];
     'h2c:shadow[87:80]<=wr_data[7:0];'h38:shadow[95:88]<=wr_data[7:0];'h3c:shadow[79:74]<=wr_data[5:0];
     default:if(wr_addr>='h80 && wr_addr<='ha0)shadow[128+((wr_addr-'h80)>>2)*16 +:16]<=wr_data[15:0];
     'h24:shadow[71:64]<=wr_data[7:0];'h28:shadow[73:72]<=wr_data[1:0];
    endcase
   end
  end
 end
 always @(posedge pixel_clk)begin
  req_sync<={req_sync[0],request};
  if(pixel_rst)begin req_sync<=0;ack<=0;pixel_config<=DEFAULT_CONFIG;settle<=0;initialized<=0;applied_toggle<=0;drain_count<=0;end
  else begin
   settle<={settle[1:0],1'b1};
   // Wait beyond the 25-stage datapath plus the two-beat skid boundary.
   // START cannot admit a delayed SOF containing pre-commit pixels.
   if(settle[2] && (!initialized || req_sync[1]!=applied_toggle))begin
    pixel_config<=config_mailbox;applied_toggle<=req_sync[1];initialized<=1;drain_count<=6'd32;
   end else if(drain_count!=0)begin
    drain_count<=drain_count-1'b1;if(drain_count==1)ack<=applied_toggle;
   end
  end
 end
endmodule
