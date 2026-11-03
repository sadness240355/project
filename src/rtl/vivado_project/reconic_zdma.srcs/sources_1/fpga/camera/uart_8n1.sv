`timescale 1ns / 1ps

// Camera Link serial UART, 8 data bits, no parity, one stop bit.
//
// The clock-domain default is the OpenNIC AXI-Lite control clock (125 MHz).
// baud_div_i is expressed in clk cycles per serial bit and is latched at the
// start of each RX/TX character so a software divider update cannot corrupt an
// in-flight character.
module uart_8n1 #(
    parameter integer CLK_HZ  = 125000000,
    parameter integer BAUD    = 9600,
    parameter integer CNT_W   = 32,
    parameter integer MIN_DIV = 8
) (
    input  wire             clk,
    input  wire             rst,
    input  wire [CNT_W-1:0] baud_div_i,
    input  wire             rx_i,
    output logic [7:0]      rx_data_o,
    output logic            rx_valid_o,
    output logic            rx_busy_o,
    output logic            rx_frame_error_o,
    output logic            tx_o,
    input  wire [7:0]       tx_data_i,
    input  wire             tx_valid_i,
    output logic            tx_ready_o,
    output logic            tx_busy_o
);
    localparam integer DEFAULT_DIV_I = (CLK_HZ + (BAUD/2)) / BAUD;
    localparam logic [CNT_W-1:0] DEFAULT_DIV = DEFAULT_DIV_I[CNT_W-1:0];
    localparam logic [CNT_W-1:0] MIN_DIV_L = MIN_DIV[CNT_W-1:0];

    wire [CNT_W-1:0] requested_div =
        (baud_div_i >= MIN_DIV_L) ? baud_div_i : DEFAULT_DIV;

    // Camera serial input is asynchronous to clk.  The ASYNC_REG attribute
    // keeps the synchronizer stages together during implementation.
    (* ASYNC_REG = "TRUE" *) logic [2:0] rx_sync;
    logic rx_sync_d;
    wire  rx_s = rx_sync[2];
    wire  rx_falling = rx_sync_d && !rx_s;

    always_ff @(posedge clk) begin
        if (rst) begin
            rx_sync   <= 3'b111;
            rx_sync_d <= 1'b1;
        end else begin
            rx_sync   <= {rx_sync[1:0], rx_i};
            rx_sync_d <= rx_sync[2];
        end
    end

    typedef enum logic [1:0] {RX_IDLE, RX_START, RX_DATA, RX_STOP} rx_state_t;
    rx_state_t rx_state;
    logic [CNT_W-1:0] rx_div_latched;
    logic [CNT_W-1:0] rx_cnt;
    logic [2:0]       rx_bit_idx;
    logic [7:0]       rx_shift;
    logic             rx_sample_a;
    logic             rx_sample_b;

    wire rx_majority = (rx_sample_a & rx_sample_b) |
                       (rx_sample_a & rx_s) |
                       (rx_sample_b & rx_s);

    always_comb begin
        rx_busy_o = (rx_state != RX_IDLE);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            rx_state         <= RX_IDLE;
            rx_div_latched   <= DEFAULT_DIV;
            rx_cnt           <= '0;
            rx_bit_idx       <= 3'd0;
            rx_shift         <= 8'd0;
            rx_sample_a      <= 1'b1;
            rx_sample_b      <= 1'b1;
            rx_data_o        <= 8'd0;
            rx_valid_o       <= 1'b0;
            rx_frame_error_o <= 1'b0;
        end else begin
            rx_valid_o       <= 1'b0;
            rx_frame_error_o <= 1'b0;

            case (rx_state)
            RX_IDLE: begin
                rx_cnt      <= '0;
                rx_bit_idx  <= 3'd0;
                rx_sample_a <= 1'b1;
                rx_sample_b <= 1'b1;
                if (rx_falling) begin
                    rx_div_latched <= requested_div;
                    // Re-sample the start bit near its centre.
                    rx_cnt <= (requested_div >> 1) - 1'b1;
                    rx_state <= RX_START;
                end
            end

            RX_START: begin
                if (rx_cnt != 0) begin
                    rx_cnt <= rx_cnt - 1'b1;
                end else if (!rx_s) begin
                    rx_cnt      <= rx_div_latched - 1'b1;
                    rx_bit_idx  <= 3'd0;
                    rx_sample_a <= 1'b0;
                    rx_sample_b <= 1'b0;
                    rx_state    <= RX_DATA;
                end else begin
                    // Glitch/false start.
                    rx_state <= RX_IDLE;
                end
            end

            RX_DATA: begin
                if (rx_cnt > 2) begin
                    rx_cnt <= rx_cnt - 1'b1;
                end else if (rx_cnt == 2) begin
                    rx_sample_a <= rx_s;
                    rx_cnt <= rx_cnt - 1'b1;
                end else if (rx_cnt == 1) begin
                    rx_sample_b <= rx_s;
                    rx_cnt <= rx_cnt - 1'b1;
                end else begin
                    // Three adjacent 125 MHz samples around the bit centre.
                    rx_shift[rx_bit_idx] <= rx_majority;
                    rx_sample_a <= 1'b1;
                    rx_sample_b <= 1'b1;
                    rx_cnt <= rx_div_latched - 1'b1;
                    if (rx_bit_idx == 3'd7) begin
                        rx_state <= RX_STOP;
                    end else begin
                        rx_bit_idx <= rx_bit_idx + 1'b1;
                    end
                end
            end

            RX_STOP: begin
                if (rx_cnt > 2) begin
                    rx_cnt <= rx_cnt - 1'b1;
                end else if (rx_cnt == 2) begin
                    rx_sample_a <= rx_s;
                    rx_cnt <= rx_cnt - 1'b1;
                end else if (rx_cnt == 1) begin
                    rx_sample_b <= rx_s;
                    rx_cnt <= rx_cnt - 1'b1;
                end else begin
                    rx_state <= RX_IDLE;
                    if (rx_majority) begin
                        rx_data_o  <= rx_shift;
                        rx_valid_o <= 1'b1;
                    end else begin
                        rx_frame_error_o <= 1'b1;
                    end
                end
            end

            default: rx_state <= RX_IDLE;
            endcase
        end
    end

    typedef enum logic [1:0] {TX_IDLE, TX_START, TX_DATA, TX_STOP} tx_state_t;
    tx_state_t tx_state;
    logic [CNT_W-1:0] tx_div_latched;
    logic [CNT_W-1:0] tx_cnt;
    logic [2:0]       tx_bit_idx;
    logic [7:0]       tx_shift;

    always_comb begin
        tx_ready_o = (tx_state == TX_IDLE);
        tx_busy_o  = (tx_state != TX_IDLE);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            tx_state       <= TX_IDLE;
            tx_div_latched <= DEFAULT_DIV;
            tx_cnt         <= '0;
            tx_bit_idx     <= 3'd0;
            tx_shift       <= 8'd0;
            tx_o           <= 1'b1;
        end else begin
            case (tx_state)
            TX_IDLE: begin
                tx_o       <= 1'b1;
                tx_cnt     <= '0;
                tx_bit_idx <= 3'd0;
                if (tx_valid_i) begin
                    tx_div_latched <= requested_div;
                    tx_shift       <= tx_data_i;
                    tx_o           <= 1'b0;
                    tx_cnt         <= requested_div - 1'b1;
                    tx_state       <= TX_START;
                end
            end

            TX_START: begin
                if (tx_cnt != 0) begin
                    tx_cnt <= tx_cnt - 1'b1;
                end else begin
                    tx_o       <= tx_shift[0];
                    tx_shift   <= {1'b0, tx_shift[7:1]};
                    tx_cnt     <= tx_div_latched - 1'b1;
                    tx_bit_idx <= 3'd0;
                    tx_state   <= TX_DATA;
                end
            end

            TX_DATA: begin
                if (tx_cnt != 0) begin
                    tx_cnt <= tx_cnt - 1'b1;
                end else begin
                    tx_cnt <= tx_div_latched - 1'b1;
                    if (tx_bit_idx == 3'd7) begin
                        tx_o     <= 1'b1;
                        tx_state <= TX_STOP;
                    end else begin
                        tx_o       <= tx_shift[0];
                        tx_shift   <= {1'b0, tx_shift[7:1]};
                        tx_bit_idx <= tx_bit_idx + 1'b1;
                    end
                end
            end

            TX_STOP: begin
                if (tx_cnt != 0)
                    tx_cnt <= tx_cnt - 1'b1;
                else
                    tx_state <= TX_IDLE;
            end

            default: tx_state <= TX_IDLE;
            endcase
        end
    end
endmodule
