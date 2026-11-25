// ============================================================================
// USB HID Composite Device - 单驱动版本，使用握手信号
// ============================================================================
module usb_hid_composite (
    input  wire        clk,
    input  wire        rst_n,
    output wire [7:0]  core_tx_data,
    output wire        core_tx_valid,
    input  wire        core_tx_ready,
    input  wire [7:0]  core_rx_data,
    input  wire        core_rx_valid,
    input  wire        core_rx_last,
    input  wire        core_rx_error,
    input  wire [104:0] all_keys,
    input  wire         keys_changed,
    output reg          led_caps,
    output reg          led_num,
    output reg          led_scroll
);

    // ---- Constants (same as before) ----
    localparam REQ_GET_DESCRIPTOR = 8'h06,
               REQ_SET_ADDRESS    = 8'h05,
               REQ_SET_CONFIG     = 8'h09,
               REQ_SET_REPORT     = 8'h09;
    localparam DESC_DEVICE = 8'd1, DESC_CONFIG = 8'd2, DESC_HID_REPORT = 8'd34;
    localparam HID_REPORT_DESC_SIZE = 7'd76;
    localparam CONFIG_DESC_SIZE = 7'd59;

    // ---- States ----
    localparam IDLE=4'd0, WAIT_SETUP=4'd1, PROCESS_SETUP=4'd2,
               SEND_DATA=4'd3, SEND_STATUS=4'd4, WAIT_STATUS=4'd5;
    reg [3:0] state, next_state;

    reg [7:0]  bmRequestType, bRequest;
    reg [15:0] wValue, wIndex, wLength;

    reg [7:0]  send_buffer [0:127];
    reg [6:0]  send_len;
    reg        send_req;          // 主状态机请求发送（仅主状态机驱动）
    reg        send_ack;          // TX状态机应答（仅TX状态机驱动）
    reg        configured;
    reg [7:0]  output_report;

    // ---- Descriptor functions (fill with actual data) ----
    function [7:0] device_desc;
        input [5:0] index;
        begin
            case (index)
                0: device_desc=8'h12; 1: device_desc=8'h01;
                2: device_desc=8'h00; 3: device_desc=8'h02;
                4: device_desc=8'h00; 5: device_desc=8'h00;
                6: device_desc=8'h00; 7: device_desc=8'h40;
                8: device_desc=8'h34; 9: device_desc=8'h12;
                10: device_desc=8'h56; 11: device_desc=8'h78;
                12: device_desc=8'h00; 13: device_desc=8'h01;
                14: device_desc=8'h01; 15: device_desc=8'h02;
                16: device_desc=8'h00; 17: device_desc=8'h01;
                default: device_desc=8'h00;
            endcase
        end
    endfunction

    function [7:0] config_desc;
        input [5:0] index;
        begin
            // 59 bytes, fill as before
            case (index)
                0: config_desc=8'h09; 1: config_desc=8'h02;
                2: config_desc=8'h3B; 3: config_desc=8'h00;
                4: config_desc=8'h02; 5: config_desc=8'h01;
                6: config_desc=8'h00; 7: config_desc=8'h80;
                8: config_desc=8'h32;
                9: config_desc=8'h09; 10: config_desc=8'h04;
                11: config_desc=8'h00; 12: config_desc=8'h00;
                13: config_desc=8'h01; 14: config_desc=8'h03;
                15: config_desc=8'h01; 16: config_desc=8'h01;
                17: config_desc=8'h00;
                18: config_desc=8'h09; 19: config_desc=8'h21;
                20: config_desc=8'h01; 21: config_desc=8'h11;
                22: config_desc=8'h00; 23: config_desc=8'h01;
                24: config_desc=8'h22; 25: config_desc=8'h4C;
                26: config_desc=8'h00;
                27: config_desc=8'h07; 28: config_desc=8'h05;
                29: config_desc=8'h81; 30: config_desc=8'h03;
                31: config_desc=8'h0A; 32: config_desc=8'h00;
                33: config_desc=8'h01;
                34: config_desc=8'h09; 35: config_desc=8'h04;
                36: config_desc=8'h01; 37: config_desc=8'h00;
                38: config_desc=8'h01; 39: config_desc=8'h03;
                40: config_desc=8'h01; 41: config_desc=8'h01;
                42: config_desc=8'h00;
                43: config_desc=8'h09; 44: config_desc=8'h21;
                45: config_desc=8'h01; 46: config_desc=8'h11;
                47: config_desc=8'h00; 48: config_desc=8'h01;
                49: config_desc=8'h22; 50: config_desc=8'h4C;
                51: config_desc=8'h00;
                52: config_desc=8'h07; 53: config_desc=8'h05;
                54: config_desc=8'h82; 55: config_desc=8'h03;
                56: config_desc=8'h0A; 57: config_desc=8'h00;
                58: config_desc=8'h01;
                default: config_desc=8'h00;
            endcase
        end
    endfunction

    function [7:0] hid_report_desc;
        input [6:0] index;
        begin
            // 76 bytes, fill as before
            case (index)
                0: hid_report_desc=8'h05; 1: hid_report_desc=8'h01;
                2: hid_report_desc=8'h09; 3: hid_report_desc=8'h06;
                4: hid_report_desc=8'hA1; 5: hid_report_desc=8'h01;
                6: hid_report_desc=8'h85; 7: hid_report_desc=8'h01;
                8: hid_report_desc=8'h05; 9: hid_report_desc=8'h07;
                10: hid_report_desc=8'h19; 11: hid_report_desc=8'h00;
                12: hid_report_desc=8'h29; 13: hid_report_desc=8'hFF;
                14: hid_report_desc=8'h15; 15: hid_report_desc=8'h00;
                16: hid_report_desc=8'h25; 17: hid_report_desc=8'hFF;
                18: hid_report_desc=8'h75; 19: hid_report_desc=8'h08;
                20: hid_report_desc=8'h95; 21: hid_report_desc=8'h08;
                22: hid_report_desc=8'h81; 23: hid_report_desc=8'h00;
                24: hid_report_desc=8'h05; 25: hid_report_desc=8'h08;
                26: hid_report_desc=8'h19; 27: hid_report_desc=8'h01;
                28: hid_report_desc=8'h29; 29: hid_report_desc=8'h03;
                30: hid_report_desc=8'h15; 31: hid_report_desc=8'h00;
                32: hid_report_desc=8'h25; 33: hid_report_desc=8'h01;
                34: hid_report_desc=8'h75; 35: hid_report_desc=8'h01;
                36: hid_report_desc=8'h95; 37: hid_report_desc=8'h03;
                38: hid_report_desc=8'h91; 39: hid_report_desc=8'h02;
                40: hid_report_desc=8'h85; 41: hid_report_desc=8'h02;
                42: hid_report_desc=8'h05; 43: hid_report_desc=8'h07;
                44: hid_report_desc=8'h19; 45: hid_report_desc=8'h00;
                46: hid_report_desc=8'h29; 47: hid_report_desc=8'hFF;
                48: hid_report_desc=8'h15; 49: hid_report_desc=8'h00;
                50: hid_report_desc=8'h25; 51: hid_report_desc=8'hFF;
                52: hid_report_desc=8'h75; 53: hid_report_desc=8'h08;
                54: hid_report_desc=8'h95; 55: hid_report_desc=8'h08;
                56: hid_report_desc=8'h81; 57: hid_report_desc=8'h00;
                58: hid_report_desc=8'h85; 59: hid_report_desc=8'h03;
                60: hid_report_desc=8'h05; 61: hid_report_desc=8'h08;
                62: hid_report_desc=8'h19; 63: hid_report_desc=8'h01;
                64: hid_report_desc=8'h29; 65: hid_report_desc=8'h03;
                66: hid_report_desc=8'h15; 67: hid_report_desc=8'h00;
                68: hid_report_desc=8'h25; 69: hid_report_desc=8'h01;
                70: hid_report_desc=8'h75; 71: hid_report_desc=8'h01;
                72: hid_report_desc=8'h95; 73: hid_report_desc=8'h03;
                74: hid_report_desc=8'h91; 75: hid_report_desc=8'h02;
                76: hid_report_desc=8'hC0;
                default: hid_report_desc=8'h00;
            endcase
        end
    endfunction

    // ---- Main state machine (only drives send_req, not send_ack) ----
    integer i;
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: if (core_rx_valid && !core_rx_error) next_state = WAIT_SETUP;
            WAIT_SETUP: if (core_rx_last) next_state = PROCESS_SETUP;
            PROCESS_SETUP: begin
                if (bRequest == REQ_SET_REPORT) next_state = WAIT_STATUS;
                else if (wLength > 16'h0000) next_state = SEND_DATA;
                else next_state = SEND_STATUS;
            end
            SEND_DATA: if (send_ack) next_state = WAIT_STATUS;  // 等待TX应答
            SEND_STATUS: next_state = IDLE;
            WAIT_STATUS: if (core_rx_valid) next_state = IDLE;
            default: next_state = IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bmRequestType <= 0; bRequest <= 0;
            wValue <= 0; wIndex <= 0; wLength <= 0;
            send_len <= 0;
            send_req <= 0;
            configured <= 0;
            output_report <= 0;
        end else begin
            state <= next_state;

            case (state)
                WAIT_SETUP: begin
                    if (core_rx_valid) begin
                        // 模拟请求（实际应解析core_rx_data）
                        bmRequestType <= 8'h80;
                        bRequest      <= REQ_GET_DESCRIPTOR;
                        wValue        <= 16'h0100;
                        wIndex        <= 16'h0000;
                        wLength       <= 16'h0040;
                    end
                end

                PROCESS_SETUP: begin
                    // 准备发送数据
                    if (bRequest == REQ_GET_DESCRIPTOR) begin
                        case (wValue[15:8])
                            DESC_DEVICE: begin
                                send_len <= 7'd18;
                                for (i=0; i<18; i=i+1) send_buffer[i] = device_desc(i);
                            end
                            DESC_CONFIG: begin
                                send_len <= CONFIG_DESC_SIZE;
                                for (i=0; i<CONFIG_DESC_SIZE; i=i+1) send_buffer[i] = config_desc(i);
                            end
                            DESC_HID_REPORT: begin
                                send_len <= HID_REPORT_DESC_SIZE;
                                for (i=0; i<HID_REPORT_DESC_SIZE; i=i+1) send_buffer[i] = hid_report_desc(i);
                            end
                            default: send_len <= 0;
                        endcase
                    end else if (bRequest == REQ_SET_ADDRESS) begin
                        send_len <= 0;
                    end else if (bRequest == REQ_SET_CONFIG) begin
                        configured <= 1;
                        send_len <= 0;
                    end else if (bRequest == REQ_SET_REPORT) begin
                        send_len <= 0;
                    end else begin
                        send_len <= 0;
                    end
                    // 如果长度>0，发起发送请求
                    if (send_len > 0) send_req <= 1;
                end

                SEND_DATA: begin
                    // 等待TX完成，收到应答后清除请求
                    if (send_ack) begin
                        send_req <= 0;
                    end
                end

                WAIT_STATUS: begin
                    if (core_rx_valid) begin
                        output_report <= core_rx_data;
                    end
                end

                default: ;
            endcase
        end
    end

    // ---- Keyboard report generation (unchanged, single driver) ----
    integer j;
    function [7:0] keycode_lookup;
        input [6:0] key_index;
        begin
            keycode_lookup = 8'h04 + key_index[6:0]; // example
        end
    endfunction

    reg kbd1_updated, kbd2_updated;
    reg kbd1_sent, kbd2_sent;       // 由TX状态机驱动
    reg [3:0] kbd1_key_count, kbd2_key_count;
    reg [7:0] kbd1_report [0:9], kbd2_report [0:9];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kbd1_updated <= 0; kbd2_updated <= 0;
            kbd1_key_count <= 0; kbd2_key_count <= 0;
            for (j=0; j<10; j=j+1) begin
                kbd1_report[j] <= 0;
                kbd2_report[j] <= 0;
            end
        end else begin
            // 清除更新标志（由TX发送完成脉冲）
            if (kbd1_sent) kbd1_updated <= 0;
            if (kbd2_sent) kbd2_updated <= 0;

            if (keys_changed) begin
                // 清空报告
                for (j=2; j<10; j=j+1) begin
                    kbd1_report[j] <= 0;
                    kbd2_report[j] <= 0;
                end
                kbd1_key_count <= 0;
                kbd2_key_count <= 0;

                for (j=0; j<104; j=j+1) begin
                    if (all_keys[j]) begin
                        if (kbd1_key_count < 4'd8) begin
                            kbd1_report[2 + kbd1_key_count] <= keycode_lookup(j);
                            kbd1_key_count <= kbd1_key_count + 4'h1;
                        end else if (kbd2_key_count < 4'd8) begin
                            kbd2_report[2 + kbd2_key_count] <= keycode_lookup(j);
                            kbd2_key_count <= kbd2_key_count + 4'h1;
                        end
                    end
                end
                kbd1_updated <= 1;
                kbd2_updated <= 1;
            end
        end
    end

    // ---- Unified Transmit State Machine (drives tx signals, send_ack, kbd1_sent, kbd2_sent) ----
    reg [7:0]  tx_data_reg;
    reg        tx_valid_reg;
    reg [3:0]  tx_state;
    reg [6:0]  tx_byte_cnt;
    reg        tx_busy;

    localparam TX_IDLE=4'd0, TX_DESC=4'd1, TX_KBD1=4'd2, TX_KBD2=4'd3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_data_reg <= 0;
            tx_valid_reg <= 0;
            tx_state <= TX_IDLE;
            tx_byte_cnt <= 0;
            send_ack <= 0;
            kbd1_sent <= 0;
            kbd2_sent <= 0;
        end else begin
            // 默认清除脉冲
            send_ack <= 0;
            kbd1_sent <= 0;
            kbd2_sent <= 0;

            case (tx_state)
                TX_IDLE: begin
                    tx_valid_reg <= 0;
                    // 优先响应描述符请求
                    if (send_req && core_tx_ready) begin
                        tx_byte_cnt <= 0;
                        tx_data_reg <= send_buffer[0];
                        tx_valid_reg <= 1;
                        tx_state <= TX_DESC;
                    end
                    // 键盘报告1
                    else if (kbd1_updated && core_tx_ready) begin
                        tx_byte_cnt <= 0;
                        tx_data_reg <= 8'h01;   // Report ID
                        tx_valid_reg <= 1;
                        tx_state <= TX_KBD1;
                    end
                    // 键盘报告2
                    else if (kbd2_updated && core_tx_ready) begin
                        tx_byte_cnt <= 0;
                        tx_data_reg <= 8'h02;
                        tx_valid_reg <= 1;
                        tx_state <= TX_KBD2;
                    end
                end

                TX_DESC: begin
                    if (core_tx_ready) begin
                        if (tx_byte_cnt < send_len - 1) begin
                            tx_byte_cnt <= tx_byte_cnt + 1;
                            tx_data_reg <= send_buffer[tx_byte_cnt + 1];
                            tx_valid_reg <= 1;
                        end else begin
                            // 发送完成
                            tx_valid_reg <= 0;
                            tx_state <= TX_IDLE;
                            send_ack <= 1;      // 应答主状态机
                        end
                    end
                end

                TX_KBD1: begin
                    if (core_tx_ready) begin
                        if (tx_byte_cnt < 7'd9) begin
                            tx_byte_cnt <= tx_byte_cnt + 1;
                            tx_data_reg <= kbd1_report[tx_byte_cnt + 1];
                            tx_valid_reg <= 1;
                        end else begin
                            tx_valid_reg <= 0;
                            tx_state <= TX_IDLE;
                            kbd1_sent <= 1;     // 清除更新标志
                        end
                    end
                end

                TX_KBD2: begin
                    if (core_tx_ready) begin
                        if (tx_byte_cnt < 7'd9) begin
                            tx_byte_cnt <= tx_byte_cnt + 1;
                            tx_data_reg <= kbd2_report[tx_byte_cnt + 1];
                            tx_valid_reg <= 1;
                        end else begin
                            tx_valid_reg <= 0;
                            tx_state <= TX_IDLE;
                            kbd2_sent <= 1;
                        end
                    end
                end

                default: begin
                    tx_valid_reg <= 0;
                    tx_state <= TX_IDLE;
                end
            endcase
        end
    end

    // ---- Outputs ----
    assign core_tx_data  = tx_data_reg;
    assign core_tx_valid = tx_valid_reg;

    // ---- LEDs ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            led_caps <= 0; led_num <= 0; led_scroll <= 0;
        end else begin
            led_caps   <= output_report[0];
            led_num    <= output_report[1];
            led_scroll <= output_report[2];
        end
    end

endmodule