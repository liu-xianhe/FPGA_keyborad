// usb_link_layer_device.v
module usb_link_layer_device (
    input  wire        clk,
    input  wire        rst_n,

    // Interface to ULPI Wrapper
    output wire [7:0]  ulpi_tx_data,
    output wire        ulpi_tx_valid,
    input  wire        ulpi_tx_ready,
    output wire        ulpi_tx_cmd,
    input  wire [7:0]  ulpi_rx_data,
    input  wire        ulpi_rx_valid,
    input  wire        ulpi_rx_active,
    input  wire        ulpi_rx_error,

    // Interface to Application Layer (Keyboard)
    input  wire        app_tx_valid,
    input  wire [7:0]  app_tx_data,
    input  wire        app_tx_last,
    output wire        app_tx_ready,

    output wire        app_rx_valid,
    output wire [7:0]  app_rx_data,
    output wire        app_rx_last,
    output wire        app_rx_error
);

    // --- 常量定义 ---
    localparam PID_SETUP    = 8'h2D,
               PID_IN       = 8'h69,
               PID_OUT      = 8'hE1,
               PID_DATA0    = 8'hC3,
               PID_DATA1    = 8'h4B,
               PID_ACK      = 8'hD2,
               PID_NAK      = 8'h5A,
               PID_STALL    = 8'h1E;

    // --- 状态机 ---
    localparam IDLE          = 4'd0,
               RX_TOKEN      = 4'd1,
               RX_ADDR_ENDP  = 4'd2,
               RX_DATA       = 4'd3,
               RX_CRC        = 4'd4,
               TX_HANDSHAKE  = 4'd5,
               TX_DATA       = 4'd6,
               TX_CRC        = 4'd7,
               TX_WAIT       = 4'd8;
    reg [3:0] state, next_state;

    // --- 内部信号 ---
    reg [7:0]  rx_pid;
    reg [6:0]  rx_addr;
    reg [3:0]  rx_endpoint;
    reg [10:0] rx_byte_cnt;
    reg        crc5_error, crc16_error;
    reg [4:0]  crc5;
    reg [15:0] crc16;
    reg        token_valid;       // 收到有效令牌标志

    reg [7:0]  tx_pid;
    reg [10:0] tx_byte_cnt;
    reg [10:0] tx_total_bytes;
    reg        tx_data_pending;   // 有待发送数据

    // --- 组合逻辑：下一状态 ---
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (ulpi_rx_active && ulpi_rx_valid) begin
                    next_state = RX_TOKEN; // 检测到包开始
                end else if (app_tx_valid && ulpi_tx_ready) begin
                    // 有数据需要发送给主机
                    next_state = TX_DATA;
                end
            end

            RX_TOKEN: begin
                if (ulpi_rx_valid) begin
                    case (ulpi_rx_data)
                        PID_SETUP, PID_IN, PID_OUT: next_state = RX_ADDR_ENDP;
                        PID_DATA0, PID_DATA1:       next_state = RX_DATA;
                        PID_ACK, PID_NAK, PID_STALL: next_state = IDLE; // 握手包，直接结束
                        default: next_state = IDLE;
                    endcase
                end
            end

            RX_ADDR_ENDP: begin
                if (ulpi_rx_valid && rx_byte_cnt == 1) begin
                    next_state = RX_CRC; // 接收完地址和端点，进入CRC
                end
            end

            RX_DATA: begin
                if (ulpi_rx_valid) begin
                    if (rx_byte_cnt == 10'd0) // 简化: 假设数据长度固定
                        next_state = RX_CRC;
                end
            end

            RX_CRC: begin
                if (ulpi_rx_valid) begin
                    next_state = IDLE;
                end
            end

            TX_DATA: begin
                if (ulpi_tx_ready && ulpi_tx_valid) begin
                    if (tx_byte_cnt == tx_total_bytes - 1)
                        next_state = TX_CRC;
                end
            end

            TX_CRC: begin
                if (ulpi_tx_ready && ulpi_tx_valid) begin
                    next_state = IDLE;
                end
            end

            TX_WAIT: begin
                if (ulpi_tx_ready) next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

    // --- 时序逻辑 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    // --- 接收路径：解析令牌 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_pid        <= 8'h00;
            rx_addr       <= 7'h00;
            rx_endpoint   <= 4'h0;
            rx_byte_cnt   <= 11'h000;
            crc5          <= 5'h1F;
            crc16         <= 16'hFFFF;
            crc5_error    <= 1'b0;
            crc16_error   <= 1'b0;
            token_valid   <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    crc5_error  <= 1'b0;
                    crc16_error <= 1'b0;
                    rx_byte_cnt <= 11'h000;
                    crc5        <= 5'h1F;
                    crc16       <= 16'hFFFF;
                    token_valid <= 1'b0;
                end

                RX_TOKEN: begin
                    if (ulpi_rx_valid) begin
                        rx_pid <= ulpi_rx_data;
                        // PID校验
                        if (ulpi_rx_data[7:4] != ~ulpi_rx_data[3:0])
                            crc5_error <= 1'b1;
                    end
                end

                RX_ADDR_ENDP: begin
                    if (ulpi_rx_valid) begin
                        if (rx_byte_cnt == 0) begin
                            rx_addr <= ulpi_rx_data[6:0]; // 提取地址
                            crc5 <= crc5_calc(crc5, ulpi_rx_data);
                            rx_byte_cnt <= rx_byte_cnt + 1;
                        end else if (rx_byte_cnt == 1) begin
                            rx_endpoint <= ulpi_rx_data[3:0]; // ★ 提取端点号 ★
                            crc5 <= crc5_calc(crc5, ulpi_rx_data);
                            rx_byte_cnt <= rx_byte_cnt + 1;
                            token_valid <= 1'b1; // 令牌解析完成
                        end
                    end
                end

                RX_DATA: begin
                    if (ulpi_rx_valid) begin
                        crc16 <= crc16_calc(crc16, ulpi_rx_data);
                        rx_byte_cnt <= rx_byte_cnt + 1;
                    end
                end

                RX_CRC: begin
                    if (ulpi_rx_valid) begin
                        if (crc5 != 5'h00) crc5_error  <= 1'b1;
                        if (crc16 != 16'h0000) crc16_error <= 1'b1;
                    end
                end

                default: ;
            endcase
        end
    end

    // --- 发送路径：生成响应 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_pid          <= 8'h00;
            tx_byte_cnt     <= 11'h000;
            tx_total_bytes  <= 11'h000;
            tx_data_pending <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    tx_byte_cnt <= 11'h000;
                    if (app_tx_valid) begin
                        tx_pid          <= PID_DATA0; // 默认使用DATA0
                        tx_total_bytes  <= 11'd8;      // HID报告长度
                        tx_data_pending <= 1'b1;
                    end else if (token_valid) begin
                        // 根据收到的令牌自动回复ACK
                        tx_pid <= PID_ACK;
                        tx_data_pending <= 1'b1;
                    end
                end

                TX_DATA: begin
                    if (ulpi_tx_ready && ulpi_tx_valid) begin
                        tx_byte_cnt <= tx_byte_cnt + 1;
                    end
                end

                TX_CRC, TX_WAIT: begin
                    tx_data_pending <= 1'b0;
                end

                default: ;
            endcase
        end
    end

    // --- 输出信号连接 ---
    assign ulpi_tx_valid = (state == TX_DATA) || (state == TX_CRC);
    assign ulpi_tx_cmd   = (state == TX_CRC); // CRC作为命令发送

    always @(*) begin
        case (state)
            TX_DATA: ulpi_tx_data = app_tx_data;
            TX_CRC:  ulpi_tx_data = {crc16[7:0]}; // 仅发送CRC低8位
            default: ulpi_tx_data = 8'h00;
        endcase
    end

    assign app_tx_ready = (state == IDLE) || (state == TX_WAIT);

    assign app_rx_valid = (state == RX_DATA) && ulpi_rx_valid;
    assign app_rx_data  = ulpi_rx_data;
    assign app_rx_last  = (state == RX_DATA) && (rx_byte_cnt == 10'd0); // 简化
    assign app_rx_error = crc5_error || crc16_error || ulpi_rx_error;

    // --- CRC计算函数（简化） ---
    function [4:0] crc5_calc;
        input [4:0] crc_in;
        input [7:0] data;
        // ... (完整的CRC5计算逻辑)
    endfunction

    function [15:0] crc16_calc;
        input [15:0] crc_in;
        input [7:0] data;
        // ... (完整的CRC16计算逻辑)
    endfunction

endmodule