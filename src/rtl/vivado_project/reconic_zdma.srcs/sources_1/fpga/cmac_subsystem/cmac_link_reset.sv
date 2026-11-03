`timescale 1ns/1ps
// User-side AXIS reset follows the CMAC reset outputs and GT power state.
// Assertion does not require a running CMAC clock; release takes three edges.
module cmac_link_reset (
    input  wire       cmac_clk,
    input  wire       shell_aresetn,
    input  wire [3:0] gt_powergood,
    input  wire       usr_tx_reset,
    input  wire       usr_rx_reset,
    output wire       tx_aresetn,
    output wire       rx_aresetn
);
    wire tx_reset_req = !shell_aresetn || !(&gt_powergood) || usr_tx_reset;
    wire rx_reset_req = !shell_aresetn || !(&gt_powergood) || usr_rx_reset;
    (* ASYNC_REG = "TRUE" *) reg [2:0] tx_release = 3'b000;
    (* ASYNC_REG = "TRUE" *) reg [2:0] rx_release = 3'b000;
    always @(posedge cmac_clk or posedge tx_reset_req)
        if (tx_reset_req) tx_release <= 3'b000;
        else tx_release <= {tx_release[1:0], 1'b1};
    always @(posedge cmac_clk or posedge rx_reset_req)
        if (rx_reset_req) rx_release <= 3'b000;
        else rx_release <= {rx_release[1:0], 1'b1};
    assign tx_aresetn = tx_release[2];
    assign rx_aresetn = rx_release[2];
endmodule
