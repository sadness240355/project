`timescale 1ns / 1ps

// Camera-side control plane for Camera Link.
// The module is clocked by the OpenNIC 125 MHz AXI-Lite clock.
// Exposes a tiny AXI-Lite CSR UART plus static CC1..CC4 outputs.
// Hardware does not parse ASCII responses; software drains RX FIFO and validates values.
module camera_control_subsystem #(
    parameter integer CLK_HZ = 125000000,
    parameter integer UART_BAUD = 9600,
    parameter integer RX_FIFO_DEPTH = 512,
    parameter integer INIT_GAP_CLKS = CLK_HZ / 20,
    parameter integer DEFAULT_SER_RX_INVERT = 0,
    parameter integer DEFAULT_SER_TX_INVERT = 0,
    parameter [3:0] DEFAULT_CC_INVERT_MASK = 4'h0
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        wr_en,
    input  wire [7:0]  wr_addr,
    input  wire [31:0] wr_data,

    input  wire        rd_en,
    input  wire [7:0]  rd_addr,
    output logic [31:0] rd_data,

    input  wire        ser_rx,
    output wire        ser_tx,
    output logic [3:0] cc
);
    localparam [7:0] REG_SERIAL_CTRL      = 8'h80;
    localparam [7:0] REG_SERIAL_STATUS    = 8'h84;
    localparam [7:0] REG_SERIAL_BAUD_DIV  = 8'h88;
    localparam [7:0] REG_SERIAL_TXDATA    = 8'h8c;
    localparam [7:0] REG_SERIAL_RXDATA    = 8'h90;
    localparam [7:0] REG_SERIAL_CC        = 8'h94;
    localparam [7:0] REG_SERIAL_INIT_CTRL = 8'h98;
    localparam [7:0] REG_SERIAL_INIT_STAT = 8'h9c;
    localparam [7:0] REG_SERIAL_RX_COUNT  = 8'ha0;
    localparam [7:0] REG_SERIAL_TX_COUNT  = 8'ha4;
    localparam [7:0] REG_SERIAL_POLARITY  = 8'ha8;
    localparam [7:0] REG_SERIAL_CLOCK_HZ = 8'hac;

    localparam integer RX_COUNT_W_I = $clog2(RX_FIFO_DEPTH+1);
    localparam integer RX_PTR_W_I   = $clog2(RX_FIFO_DEPTH);
    localparam integer INIT_LEN     = 88;
    localparam [7:0] INIT_LEN_U8 = 8'd88;
    localparam integer MIN_BAUD_DIV = 8;
    localparam [31:0] INIT_GAP_CLKS_U32 = INIT_GAP_CLKS;
    localparam [RX_COUNT_W_I-1:0] RX_FIFO_DEPTH_L = RX_FIFO_DEPTH;

    logic [31:0] baud_div_reg;
    logic        local_reset_pulse;
    logic        clear_status_pulse;
    logic        start_init_pulse;
    logic        auto_init_enable;
    logic        ser_rx_invert_reg;
    logic        ser_tx_invert_reg;
    logic [3:0]  cc_invert_reg;
    logic [3:0]  cc_raw;
    logic [2:0]  ser_rx_status_sync;

    logic [7:0]  rx_fifo [0:RX_FIFO_DEPTH-1];
    logic [RX_PTR_W_I-1:0] rx_wr_ptr;
    logic [RX_PTR_W_I-1:0] rx_rd_ptr;
    logic [RX_COUNT_W_I-1:0] rx_count;
    logic        rx_overflow_sticky;
    logic        rx_frame_error_sticky;
    logic        tx_overrun_sticky;
    logic        baud_div_error_sticky;
    logic [31:0] rx_total_count;
    logic [31:0] tx_total_count;

    wire         rx_fifo_full  = (rx_count == RX_FIFO_DEPTH_L);
    wire         rx_fifo_empty = (rx_count == {RX_COUNT_W_I{1'b0}});

    logic [7:0]  uart_rx_data;
    logic        uart_rx_valid;
    logic        uart_rx_busy;
    logic        uart_rx_frame_error;
    logic        uart_tx_ready;
    logic        uart_tx_busy;
    logic [7:0]  uart_tx_data;
    logic        uart_tx_valid;
    logic        uart_tx_pin;

    wire rst_uart = rst || local_reset_pulse;
    wire ser_rx_for_uart = ser_rx ^ ser_rx_invert_reg;
    wire ser_rx_status_raw = ser_rx_status_sync[2];
    wire ser_rx_status_effective = ser_rx_status_raw ^ ser_rx_invert_reg;

    assign ser_tx = uart_tx_pin ^ ser_tx_invert_reg;
    assign cc = cc_raw ^ cc_invert_reg;

    always_ff @(posedge clk) begin
        if (rst_uart)
            ser_rx_status_sync <= 3'b111;
        else
            ser_rx_status_sync <= {ser_rx_status_sync[1:0], ser_rx};
    end

    uart_8n1 #(
        .CLK_HZ (CLK_HZ),
        .BAUD   (UART_BAUD)
    ) uart_inst (
        .clk              (clk),
        .rst              (rst_uart),
        .baud_div_i       (baud_div_reg),
        .rx_i             (ser_rx_for_uart),
        .rx_data_o        (uart_rx_data),
        .rx_valid_o       (uart_rx_valid),
        .rx_busy_o        (uart_rx_busy),
        .rx_frame_error_o (uart_rx_frame_error),
        .tx_o             (uart_tx_pin),
        .tx_data_i        (uart_tx_data),
        .tx_valid_i       (uart_tx_valid),
        .tx_ready_o       (uart_tx_ready),
        .tx_busy_o        (uart_tx_busy)
    );

    // Read-only probe sequence.  It deliberately does not write Width, Height,
    // AcquisitionFrameRate or transport geometry, so software-selected camera
    // settings cannot be overwritten by an accidental hardware init request.
    function automatic [7:0] init_rom(input [7:0] idx);
        begin
            init_rom = 8'h00;
            case (idx)
            8'd0: init_rom = 8'h72; // r
            8'd1: init_rom = 8'h20; //  
            8'd2: init_rom = 8'h57; // W
            8'd3: init_rom = 8'h69; // i
            8'd4: init_rom = 8'h64; // d
            8'd5: init_rom = 8'h74; // t
            8'd6: init_rom = 8'h68; // h
            8'd7: init_rom = 8'h0d; // 
            8'd8: init_rom = 8'h72; // r
            8'd9: init_rom = 8'h20; //  
            8'd10: init_rom = 8'h48; // H
            8'd11: init_rom = 8'h65; // e
            8'd12: init_rom = 8'h69; // i
            8'd13: init_rom = 8'h67; // g
            8'd14: init_rom = 8'h68; // h
            8'd15: init_rom = 8'h74; // t
            8'd16: init_rom = 8'h0d; // 
            8'd17: init_rom = 8'h72; // r
            8'd18: init_rom = 8'h20; //  
            8'd19: init_rom = 8'h41; // A
            8'd20: init_rom = 8'h63; // c
            8'd21: init_rom = 8'h71; // q
            8'd22: init_rom = 8'h75; // u
            8'd23: init_rom = 8'h69; // i
            8'd24: init_rom = 8'h73; // s
            8'd25: init_rom = 8'h69; // i
            8'd26: init_rom = 8'h74; // t
            8'd27: init_rom = 8'h69; // i
            8'd28: init_rom = 8'h6f; // o
            8'd29: init_rom = 8'h6e; // n
            8'd30: init_rom = 8'h46; // F
            8'd31: init_rom = 8'h72; // r
            8'd32: init_rom = 8'h61; // a
            8'd33: init_rom = 8'h6d; // m
            8'd34: init_rom = 8'h65; // e
            8'd35: init_rom = 8'h52; // R
            8'd36: init_rom = 8'h61; // a
            8'd37: init_rom = 8'h74; // t
            8'd38: init_rom = 8'h65; // e
            8'd39: init_rom = 8'h0d; // 
            8'd40: init_rom = 8'h72; // r
            8'd41: init_rom = 8'h20; //  
            8'd42: init_rom = 8'h50; // P
            8'd43: init_rom = 8'h69; // i
            8'd44: init_rom = 8'h78; // x
            8'd45: init_rom = 8'h65; // e
            8'd46: init_rom = 8'h6c; // l
            8'd47: init_rom = 8'h46; // F
            8'd48: init_rom = 8'h6f; // o
            8'd49: init_rom = 8'h72; // r
            8'd50: init_rom = 8'h6d; // m
            8'd51: init_rom = 8'h61; // a
            8'd52: init_rom = 8'h74; // t
            8'd53: init_rom = 8'h0d; // 
            8'd54: init_rom = 8'h72; // r
            8'd55: init_rom = 8'h20; //  
            8'd56: init_rom = 8'h44; // D
            8'd57: init_rom = 8'h65; // e
            8'd58: init_rom = 8'h76; // v
            8'd59: init_rom = 8'h69; // i
            8'd60: init_rom = 8'h63; // c
            8'd61: init_rom = 8'h65; // e
            8'd62: init_rom = 8'h54; // T
            8'd63: init_rom = 8'h61; // a
            8'd64: init_rom = 8'h70; // p
            8'd65: init_rom = 8'h47; // G
            8'd66: init_rom = 8'h65; // e
            8'd67: init_rom = 8'h6f; // o
            8'd68: init_rom = 8'h6d; // m
            8'd69: init_rom = 8'h65; // e
            8'd70: init_rom = 8'h74; // t
            8'd71: init_rom = 8'h72; // r
            8'd72: init_rom = 8'h79; // y
            8'd73: init_rom = 8'h0d; // 
            8'd74: init_rom = 8'h72; // r
            8'd75: init_rom = 8'h20; //  
            8'd76: init_rom = 8'h54; // T
            8'd77: init_rom = 8'h72; // r
            8'd78: init_rom = 8'h69; // i
            8'd79: init_rom = 8'h67; // g
            8'd80: init_rom = 8'h67; // g
            8'd81: init_rom = 8'h65; // e
            8'd82: init_rom = 8'h72; // r
            8'd83: init_rom = 8'h4d; // M
            8'd84: init_rom = 8'h6f; // o
            8'd85: init_rom = 8'h64; // d
            8'd86: init_rom = 8'h65; // e
            8'd87: init_rom = 8'h0d; // 
            default: init_rom = 8'h00;
            endcase
        end
    endfunction

    logic        init_busy;
    logic        init_done;
    logic [7:0]  init_idx;
    logic [31:0] init_gap_cnt;
    logic        init_tx_pending;
    logic [7:0]  init_tx_data;
    logic [31:0] init_start_count;

    wire host_tx_write = wr_en && (wr_addr == REG_SERIAL_TXDATA);
    wire host_tx_valid = host_tx_write && !init_busy && uart_tx_ready;
    wire init_tx_valid = init_tx_pending && uart_tx_ready && !host_tx_write;

    always_comb begin
        uart_tx_valid = 1'b0;
        uart_tx_data  = 8'd0;
        if (host_tx_valid) begin
            uart_tx_valid = 1'b1;
            uart_tx_data  = wr_data[7:0];
        end else if (init_tx_valid) begin
            uart_tx_valid = 1'b1;
            uart_tx_data  = init_tx_data;
        end
    end

    wire rx_pop = rd_en && (rd_addr == REG_SERIAL_RXDATA) && !rx_fifo_empty;

    always_comb begin
        rd_data = 32'd0;
        case (rd_addr)
        REG_SERIAL_CTRL:      rd_data = {15'd0, auto_init_enable, 16'd0};
        REG_SERIAL_STATUS:    rd_data = {21'd0, baud_div_error_sticky, uart_rx_busy, init_done, init_busy, tx_overrun_sticky, rx_frame_error_sticky, rx_overflow_sticky, rx_fifo_full, !rx_fifo_empty, uart_tx_busy, uart_tx_ready};
        REG_SERIAL_BAUD_DIV:  rd_data = baud_div_reg;
        REG_SERIAL_TXDATA:    rd_data = 32'd0;
        REG_SERIAL_RXDATA:    rd_data = {23'd0, !rx_fifo_empty, rx_fifo_empty ? 8'd0 : rx_fifo[rx_rd_ptr]};
        REG_SERIAL_CC:        rd_data = {20'd0, cc, cc_invert_reg, cc_raw};
        REG_SERIAL_INIT_CTRL: rd_data = {15'd0, auto_init_enable, 7'd0, init_busy, 8'd0};
        REG_SERIAL_INIT_STAT: rd_data = {6'd0, init_start_count[7:0], init_idx, 6'd0, init_done, init_busy};
        REG_SERIAL_RX_COUNT:  rd_data = {{(32-RX_COUNT_W_I){1'b0}}, rx_count};
        REG_SERIAL_TX_COUNT:  rd_data = tx_total_count;
        REG_SERIAL_POLARITY:  rd_data = {14'd0, ser_rx_status_effective, ser_rx_status_raw, 8'd0, cc_invert_reg, 2'd0, ser_tx_invert_reg, ser_rx_invert_reg};
        REG_SERIAL_CLOCK_HZ: rd_data = CLK_HZ;
        default:              rd_data = 32'd0;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            baud_div_reg          <= ((CLK_HZ + (UART_BAUD/2)) / UART_BAUD);
            local_reset_pulse     <= 1'b0;
            clear_status_pulse    <= 1'b0;
            start_init_pulse      <= 1'b0;
            auto_init_enable      <= 1'b0;
            ser_rx_invert_reg     <= (DEFAULT_SER_RX_INVERT != 0);
            ser_tx_invert_reg     <= (DEFAULT_SER_TX_INVERT != 0);
            cc_invert_reg         <= DEFAULT_CC_INVERT_MASK;
            cc_raw                <= 4'd0;
            rx_wr_ptr             <= '0;
            rx_rd_ptr             <= '0;
            rx_count              <= '0;
            rx_overflow_sticky    <= 1'b0;
            rx_frame_error_sticky <= 1'b0;
            tx_overrun_sticky     <= 1'b0;
            baud_div_error_sticky <= 1'b0;
            rx_total_count        <= 32'd0;
            tx_total_count        <= 32'd0;
            init_busy             <= 1'b0;
            init_done             <= 1'b0;
            init_idx              <= 8'd0;
            init_gap_cnt          <= 32'd0;
            init_tx_pending       <= 1'b0;
            init_tx_data          <= 8'd0;
            init_start_count      <= 32'd0;
        end else begin
            local_reset_pulse  <= 1'b0;
            clear_status_pulse <= 1'b0;
            start_init_pulse   <= 1'b0;

            if (wr_en) begin
                case (wr_addr)
                REG_SERIAL_CTRL: begin
                    local_reset_pulse  <= wr_data[0];
                    clear_status_pulse <= wr_data[4];
                    start_init_pulse   <= wr_data[8];
                end
                REG_SERIAL_BAUD_DIV: begin
                    // Divider is cycles/bit.  Reset the UART and flush stale RX
                    // data so a runtime baud change cannot split a character.
                    if (wr_data >= MIN_BAUD_DIV) begin
                        baud_div_reg       <= wr_data;
                        local_reset_pulse  <= 1'b1;
                        clear_status_pulse <= 1'b1;
                        init_busy          <= 1'b0;
                        init_tx_pending    <= 1'b0;
                    end else begin
                        baud_div_error_sticky <= 1'b1;
                    end
                end
                REG_SERIAL_TXDATA: begin
                    if (init_busy || !uart_tx_ready)
                        tx_overrun_sticky <= 1'b1;
                end
                REG_SERIAL_CC: begin
                    cc_raw <= wr_data[3:0];
                end
                REG_SERIAL_POLARITY: begin
                    // Force the new polarity even if the RX UART currently reports busy.
                    // A wrong RX inversion makes an idle-high TTL line look like a permanent
                    // low/break condition, so gating this write with !uart_rx_busy prevents
                    // software from recovering without reloading the FPGA.
                    ser_rx_invert_reg  <= wr_data[0];
                    ser_tx_invert_reg  <= wr_data[1];
                    cc_invert_reg      <= wr_data[7:4];
                    local_reset_pulse  <= 1'b1;
                    clear_status_pulse <= 1'b1;
                    init_busy          <= 1'b0;
                    init_tx_pending    <= 1'b0;
                    init_idx           <= 8'd0;
                    init_gap_cnt       <= 32'd0;
                    rx_wr_ptr          <= '0;
                    rx_rd_ptr          <= '0;
                    rx_count           <= '0;
                end
                REG_SERIAL_INIT_CTRL: begin
                    auto_init_enable <= wr_data[16];
                    start_init_pulse <= wr_data[8] || wr_data[0];
                end
                default: begin end
                endcase
            end

            if (clear_status_pulse) begin
                rx_overflow_sticky    <= 1'b0;
                rx_frame_error_sticky <= 1'b0;
                tx_overrun_sticky     <= 1'b0;
                baud_div_error_sticky <= 1'b0;
                init_done             <= 1'b0;
            end

            if (local_reset_pulse) begin
                rx_wr_ptr       <= '0;
                rx_rd_ptr       <= '0;
                rx_count        <= '0;
                init_busy       <= 1'b0;
                init_tx_pending <= 1'b0;
                init_idx        <= 8'd0;
                init_gap_cnt    <= 32'd0;
            end else begin
                if (uart_rx_valid && !rx_pop) begin
                    if (!rx_fifo_full) begin
                        rx_fifo[rx_wr_ptr] <= uart_rx_data;
                        rx_wr_ptr <= rx_wr_ptr + 1'b1;
                        rx_count <= rx_count + 1'b1;
                        rx_total_count <= rx_total_count + 1'b1;
                    end else begin
                        rx_overflow_sticky <= 1'b1;
                    end
                end else if (!uart_rx_valid && rx_pop) begin
                    rx_rd_ptr <= rx_rd_ptr + 1'b1;
                    rx_count <= rx_count - 1'b1;
                end else if (uart_rx_valid && rx_pop) begin
                    if (rx_fifo_empty) begin
                        rx_fifo[rx_wr_ptr] <= uart_rx_data;
                        rx_wr_ptr <= rx_wr_ptr + 1'b1;
                        rx_count <= rx_count + 1'b1;
                    end else begin
                        rx_fifo[rx_wr_ptr] <= uart_rx_data;
                        rx_wr_ptr <= rx_wr_ptr + 1'b1;
                        rx_rd_ptr <= rx_rd_ptr + 1'b1;
                    end
                    rx_total_count <= rx_total_count + 1'b1;
                end

                if (uart_rx_frame_error)
                    rx_frame_error_sticky <= 1'b1;

                if (uart_tx_valid)
                    tx_total_count <= tx_total_count + 1'b1;

                if ((start_init_pulse || (auto_init_enable && !init_done && !init_busy)) && !init_busy) begin
                    init_busy       <= 1'b1;
                    init_done       <= 1'b0;
                    init_idx        <= 8'd0;
                    init_gap_cnt    <= 32'd0;
                    init_tx_pending <= 1'b0;
                    init_tx_data    <= 8'd0;
                    init_start_count <= init_start_count + 1'b1;
                end else if (init_busy) begin
                    if (init_gap_cnt != 32'd0) begin
                        init_gap_cnt <= init_gap_cnt - 1'b1;
                    end else if (!init_tx_pending) begin
                        if (init_idx >= INIT_LEN_U8) begin
                            init_busy <= 1'b0;
                            init_done <= 1'b1;
                        end else begin
                            init_tx_data <= init_rom(init_idx);
                            init_tx_pending <= 1'b1;
                        end
                    end else if (init_tx_valid) begin
                        init_tx_pending <= 1'b0;
                        init_idx <= init_idx + 1'b1;
                        if (init_tx_data == 8'h0d)
                            init_gap_cnt <= INIT_GAP_CLKS_U32;
                    end
                end
            end
        end
    end
endmodule
