`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/06/09 11:52:06
// Design Name: 
// Module Name: cmlk_decode
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module cmlk_decode (
    input             clkin2_p,
    input             clkin2_n,
    input      [11:0] datain2_p,
    input      [11:0] datain2_n,
    input             rx_reset,
    input             ref_clk,
    input             ref_clk_locked,
    output            pix_clk,
    output            pix_rst,
    output            link_ready,
    output     [31:0] align_loss_count,
    output reg [ 2:0] sync_x,
    output reg [ 2:0] sync_y,
    output reg [ 2:0] sync_z,
    output reg        l_val,
    output reg        f_val,
    output reg        d_val,
    output reg [ 7:0] port_a,
    output reg [ 7:0] port_b,
    output reg [ 7:0] port_c,
    output reg [ 7:0] port_d,
    output reg [ 7:0] port_e,
    output reg [ 7:0] port_f,
    output reg [ 7:0] port_g,
    output reg [ 7:0] port_h,
    output reg [ 7:0] port_i,
    output reg [ 7:0] port_j
);

    localparam SIM_DEVICE   = "ULTRASCALE_PLUS";  // Set for the family <ULTRASCALE | ULTRASCALE_PLUS>
    localparam real REF_FREQ = 400.0;              // IDELAYCTRL/IDELAYE3 calibration clock frequency in MHz
    wire rx2_cmt_locked;
    wire rx_reset_int;
    wire idly_reset_req;
    wire idly_reset_int;
    assign rx_reset_int   = rx_reset;
    // IDELAYCTRL is clocked by ref_clk (400 MHz), while rx_reset arrives from
    // the independent 200 MHz Camera Link reference domain.  Driving RST
    // directly from that signal creates an asynchronous recovery/removal path;
    // routed hold fixing can also turn it into a long LUT chain.  Assert reset
    // asynchronously, but release it only on ref_clk.  The receiver CMT lock is
    // deliberately not part of the IDELAYCTRL reset: IDELAY calibration depends
    // only on its own stable reference clock, and the receive channel already
    // qualifies operation with rx2_cmt_locked and rx_idelay_rdy.
    assign idly_reset_req = rx_reset_int | !ref_clk_locked;

    xpm_cdc_async_rst #(
        .DEST_SYNC_FF    (4),
        .INIT_SYNC_FF    (0),
        .RST_ACTIVE_HIGH (1)
    ) idelay_reset_cdc_inst (
        .src_arst  (idly_reset_req),
        .dest_clk  (ref_clk),
        .dest_arst (idly_reset_int)
    );
    wire rx_idelay_rdy;
    //
    //  Idelay control block
    //
    IDELAYCTRL #(  // Instantiate input delay control block
        .SIM_DEVICE("ULTRASCALE")
    ) icontrol (
        .REFCLK(ref_clk),
        .RST   (idly_reset_int),
        .RDY   (rx_idelay_rdy)
    );



    wire        rx2_px_clk;
    wire [83:0] rx2_px_data;
    wire        rx2_px_ready;
    rx_channel_1to7 #(
        .LINES       (12),          // Number of data lines
        .CLKIN_PERIOD(11.765),      // Clock period (ns) of input clock on clkin_p
        .REF_FREQ    (REF_FREQ),    // Reference frequency used by idelay controller
        .DIFF_TERM   ("TRUE"),      // Enable internal differential termination
        .USE_PLL     ("TRUE"),      // Enable PLL use rather than MMCM
        .DATA_FORMAT ("PER_LINE"),  // PER_CLOCK or PER_LINE data formatting
        .CLK_PATTERN (7'b1100011),  // Clock bit pattern
        .RX_SWAP_MASK(16'b0),       // Allows P/N inputs to be invered to ease PCB routing
        .SIM_DEVICE  (SIM_DEVICE)
    ) rx_channel2 (
        .clkin_p   (clkin2_p),        // Input from LVDS clock receiver pin
        .clkin_n   (clkin2_n),        // Input from LVDS clock receiver pin
        .datain_p  (datain2_p),       // Input from LVDS data pins 
        .datain_n  (datain2_n),       // Input from LVDS data pins 
        .reset     (rx_reset_int),    // Reset line
        .cmt_locked(rx2_cmt_locked),  // PLL/MMCM locked
        .idelay_rdy(rx_idelay_rdy),   // Input delay control ready
        .px_clk    (rx2_px_clk),      // Pixel clock output
        .px_data   (rx2_px_data),     // Pixel data
        .px_ready  (rx2_px_ready),    // Pixel data ready
        .px_align_loss_count(align_loss_count)
    );


    wire [27:0] raw_data_0;
    wire [27:0] raw_data_1;
    wire [27:0] raw_data_2;
    assign raw_data_0 = rx2_px_data[27:0];
    assign raw_data_1 = rx2_px_data[55:28];
    assign raw_data_2 = rx2_px_data[83:56];

    // rx_channel_1to7 is instantiated with DATA_FORMAT("PER_LINE"), so
    // raw_data_x[6:0] are the seven serial bits received on LVDS lane 0,
    // raw_data_x[13:7] on lane 1, raw_data_x[20:14] on lane 2, and
    // raw_data_x[27:21] on lane 3.  Convert that lane-major order back to
    // Channel-Link TxIN/RxOUT bit numbering before applying the Camera Link
    // port and video-sync mapping.
    //
    // DS90CR287/DS90CR288 28-bit order per pixel clock:
    //   lane0: TxIN7,  TxIN6,  TxIN4,  TxIN3,  TxIN2,  TxIN1,  TxIN0
    //   lane1: TxIN18, TxIN15, TxIN14, TxIN13, TxIN12, TxIN9,  TxIN8
    //   lane2: TxIN26, TxIN25, TxIN24, TxIN22, TxIN21, TxIN20, TxIN19
    //   lane3: TxIN23, TxIN17, TxIN16, TxIN11, TxIN10, TxIN5,  TxIN27
    function [27:0] cl28_per_line_to_txin;
        input [27:0] raw;
        begin
            cl28_per_line_to_txin[0]  = raw[6];   // lane0 bit6: TxIN0
            cl28_per_line_to_txin[1]  = raw[5];   // lane0 bit5: TxIN1
            cl28_per_line_to_txin[2]  = raw[4];   // lane0 bit4: TxIN2
            cl28_per_line_to_txin[3]  = raw[3];   // lane0 bit3: TxIN3
            cl28_per_line_to_txin[4]  = raw[2];   // lane0 bit2: TxIN4
            cl28_per_line_to_txin[5]  = raw[26];  // lane3 bit5: TxIN5
            cl28_per_line_to_txin[6]  = raw[1];   // lane0 bit1: TxIN6
            cl28_per_line_to_txin[7]  = raw[0];   // lane0 bit0: TxIN7
            cl28_per_line_to_txin[8]  = raw[13];  // lane1 bit6: TxIN8
            cl28_per_line_to_txin[9]  = raw[12];  // lane1 bit5: TxIN9
            cl28_per_line_to_txin[10] = raw[25];  // lane3 bit4: TxIN10
            cl28_per_line_to_txin[11] = raw[24];  // lane3 bit3: TxIN11
            cl28_per_line_to_txin[12] = raw[11];  // lane1 bit4: TxIN12
            cl28_per_line_to_txin[13] = raw[10];  // lane1 bit3: TxIN13
            cl28_per_line_to_txin[14] = raw[9];   // lane1 bit2: TxIN14
            cl28_per_line_to_txin[15] = raw[8];   // lane1 bit1: TxIN15
            cl28_per_line_to_txin[16] = raw[23];  // lane3 bit2: TxIN16
            cl28_per_line_to_txin[17] = raw[22];  // lane3 bit1: TxIN17
            cl28_per_line_to_txin[18] = raw[7];   // lane1 bit0: TxIN18
            cl28_per_line_to_txin[19] = raw[20];  // lane2 bit6: TxIN19
            cl28_per_line_to_txin[20] = raw[19];  // lane2 bit5: TxIN20
            cl28_per_line_to_txin[21] = raw[18];  // lane2 bit4: TxIN21
            cl28_per_line_to_txin[22] = raw[17];  // lane2 bit3: TxIN22
            cl28_per_line_to_txin[23] = raw[21];  // lane3 bit0: TxIN23
            cl28_per_line_to_txin[24] = raw[16];  // lane2 bit2: TxIN24 / LVAL
            cl28_per_line_to_txin[25] = raw[15];  // lane2 bit1: TxIN25 / FVAL
            cl28_per_line_to_txin[26] = raw[14];  // lane2 bit0: TxIN26 / DVAL
            cl28_per_line_to_txin[27] = raw[27];  // lane3 bit6: TxIN27 / spare
        end
    endfunction

    wire [27:0] flx_data_0;
    wire [27:0] flx_data_1;
    wire [27:0] flx_data_2;

    assign flx_data_0 = cl28_per_line_to_txin(raw_data_0);
    assign flx_data_1 = cl28_per_line_to_txin(raw_data_1);
    assign flx_data_2 = cl28_per_line_to_txin(raw_data_2);

    /*
     * Camera Link 2.0 80-bit (Deca), 8-bit x 10-tap mapping. The three
     * 28-bit Channel Link receivers carry ten byte ports A..J plus FVAL and
     * three physical LVAL copies. DVAL is omitted in 80-bit mode.
     *
     * All outputs are registered at the receiver pixel clock. This keeps the
     * new mapping off the capture-counter timing path and preserves one
     * ten-pixel input word per camera clock.
     */
    reg l_val_1 = 1'b0;
    reg f_val_1 = 1'b0;
    reg d_val_1 = 1'b0;
    reg l_val_2 = 1'b0;
    reg f_val_2 = 1'b0;
    reg d_val_2 = 1'b0;

    always @(posedge rx2_px_clk) begin
        if (!rx2_px_ready) begin
            sync_x  <= 3'b000;
            sync_y  <= 3'b000;
            sync_z  <= 3'b000;
            l_val   <= 1'b0;
            f_val   <= 1'b0;
            d_val   <= 1'b0;
            l_val_1 <= 1'b0;
            f_val_1 <= 1'b0;
            d_val_1 <= 1'b0;
            l_val_2 <= 1'b0;
            f_val_2 <= 1'b0;
            d_val_2 <= 1'b0;
            port_a  <= 8'd0;
            port_b  <= 8'd0;
            port_c  <= 8'd0;
            port_d  <= 8'd0;
            port_e  <= 8'd0;
            port_f  <= 8'd0;
            port_g  <= 8'd0;
            port_h  <= 8'd0;
            port_i  <= 8'd0;
            port_j  <= 8'd0;
        end else begin
            // Present comparable logical sync vectors to the existing link
            // integrity monitor. Only LVAL is physically replicated in Deca;
            // FVAL comes from X and is copied for comparison purposes.
            sync_x <= {1'b0, flx_data_0[25], flx_data_0[24]};
            sync_y <= {1'b0, flx_data_0[25], flx_data_1[27]};
            sync_z <= {1'b0, flx_data_0[25], flx_data_2[27]};

            l_val <= flx_data_0[24];
            f_val <= flx_data_0[25];
            d_val <= 1'b1;

            l_val_1 <= flx_data_1[27];
            f_val_1 <= flx_data_0[25];
            d_val_1 <= 1'b1;
            l_val_2 <= flx_data_2[27];
            f_val_2 <= flx_data_0[25];
            d_val_2 <= 1'b1;

            port_a <= flx_data_0[7:0];
            port_b <= flx_data_0[15:8];
            port_c <= flx_data_0[23:16];
            port_d <= {flx_data_1[5:0], flx_data_0[27:26]};
            port_e <= flx_data_1[13:6];
            port_f <= flx_data_1[21:14];
            port_g <= {flx_data_2[2:0], flx_data_1[26:22]};
            port_h <= flx_data_2[10:3];
            port_i <= flx_data_2[18:11];
            port_j <= flx_data_2[26:19];
        end
    end

    reg img_f_val = 1'b0;
    reg img_d_val = 1'b0;
    reg img_l_val = 1'b0;

    always @(posedge rx2_px_clk) begin
        img_f_val <= f_val;
        img_d_val <= d_val;
        img_l_val <= l_val;
    end
    wire f_val_upedge;
    wire h_val_fledge;

    wire f_val_fledge;
    assign f_val_upedge = (~img_f_val && f_val);
    assign f_val_fledge = (img_f_val && ~f_val);

    assign h_val_fledge = (img_l_val && ~l_val);



    reg [31:0] cnt_frame_cyc = 32'd0;

    always @(posedge rx2_px_clk) begin
        if (f_val_upedge) cnt_frame_cyc <= 32'd0;
        else cnt_frame_cyc <= cnt_frame_cyc + 32'd1;
    end

    reg [15:0] hs_cnt = 16'd0;
    reg [ 9:0] hs_cnt_delay = 10'd0;
    reg [31:0] hs_pix_cnt = 32'd0;
    reg [31:0] vs_pix_cnt = 32'd0;
    always @(posedge rx2_px_clk) begin
        if (f_val_upedge) begin
            hs_cnt <= 16'd0;
        end else if (h_val_fledge) begin
            hs_cnt <= hs_cnt + 16'd1;
        end else begin
            hs_cnt <= hs_cnt;
        end
    end

    always @(posedge rx2_px_clk) begin
        hs_cnt_delay <= hs_cnt;
    end


    always @(posedge rx2_px_clk)
        if (f_val_fledge) vs_pix_cnt <= 32'd0;
        else if (img_l_val) vs_pix_cnt <= vs_pix_cnt + 32'd10;
        else vs_pix_cnt <= vs_pix_cnt;




    // ila_cmlk_full inst_ila_cmlk_full (
    // 	.clk(rx2_px_clk), // input wire clk


    // 	.probe0({rx2_px_data,rx2_px_ready}), // input wire [84:0] probe0
    // 	.probe1({
    // 			l_val,
    // 	        f_val,
    // 	        d_val,
    // 			l_val_1	,
    // 			f_val_1	,
    // 			d_val_1	,
    // 			l_val_2	,
    // 			f_val_2	,
    // 			d_val_2	,
    // 			port_a	,
    // 			port_b	,
    // 			port_c	,
    // 			port_d	,
    // 			port_e	,
    // 			port_f	,
    // 			port_g	,
    // 			port_h
    // 	}), // input wire [84:0] probe0
    // 	.probe2({img_f_val,
    // 			 img_l_val,
    // 			 cnt_frame_cyc,
    // 			 hs_cnt,
    // 			 vs_pix_cnt		
    // 			 }),
    // 	.probe3(rx2_cmt_locked)
    // ); 
    assign pix_clk    = rx2_px_clk;
    assign link_ready = rx2_cmt_locked & rx2_px_ready;
    assign pix_rst    = ~link_ready;

endmodule
