`timescale 1ns/1ps
// Read-only extension in the previously unused QSFP placeholder page.
// Every address outside 0xf00..0xf14 preserves the original prefix response.
module cmac_link_diag #(
    parameter logic [15:0] REG_PREFIX = 16'hc028
)(
    input wire s_axil_awvalid, input wire [31:0] s_axil_awaddr,
    output wire s_axil_awready,
    input wire s_axil_wvalid, input wire [31:0] s_axil_wdata,
    output wire s_axil_wready, output wire s_axil_bvalid,
    output wire [1:0] s_axil_bresp, input wire s_axil_bready,
    input wire s_axil_arvalid, input wire [31:0] s_axil_araddr,
    output wire s_axil_arready, output wire s_axil_rvalid,
    output wire [31:0] s_axil_rdata, output wire [1:0] s_axil_rresp,
    input wire s_axil_rready,
    input wire [31:0] link_flags,
    input wire [19:0] rx_block_lock, input wire [19:0] rx_synced,
    input wire aclk, input wire aresetn
);
    wire reg_en, reg_we;
    wire [11:0] reg_addr;
    wire [31:0] reg_din;
    reg [31:0] reg_dout;
    axi_lite_register #(.CLOCKING_MODE("common_clock"), .ADDR_W(12), .DATA_W(32)) bridge (
        .s_axil_awvalid(s_axil_awvalid), .s_axil_awaddr(s_axil_awaddr[11:0]),
        .s_axil_awready(s_axil_awready), .s_axil_wvalid(s_axil_wvalid),
        .s_axil_wdata(s_axil_wdata), .s_axil_wready(s_axil_wready),
        .s_axil_bvalid(s_axil_bvalid), .s_axil_bresp(s_axil_bresp), .s_axil_bready(s_axil_bready),
        .s_axil_arvalid(s_axil_arvalid), .s_axil_araddr(s_axil_araddr[11:0]),
        .s_axil_arready(s_axil_arready), .s_axil_rvalid(s_axil_rvalid),
        .s_axil_rdata(s_axil_rdata), .s_axil_rresp(s_axil_rresp), .s_axil_rready(s_axil_rready),
        .reg_en(reg_en), .reg_we(reg_we), .reg_addr(reg_addr), .reg_din(reg_din), .reg_dout(reg_dout),
        .axil_aclk(aclk), .axil_aresetn(aresetn), .reg_clk(aclk), .reg_rstn(aresetn)
    );
    always @(posedge aclk) begin
        if (!aresetn) reg_dout <= 32'b0;
        else if (reg_en && !reg_we) begin
            case (reg_addr)
                12'hf00: reg_dout <= 32'h4c4e4b35; // "LNK5"
                12'hf04: reg_dout <= 32'h00010000;
                12'hf08: reg_dout <= link_flags;
                12'hf0c: reg_dout <= {12'b0, rx_block_lock};
                12'hf10: reg_dout <= {12'b0, rx_synced};
                12'hf14: reg_dout <= 32'h00000003;
                default: reg_dout <= {REG_PREFIX, 4'b0, reg_addr};
            endcase
        end
    end
endmodule
