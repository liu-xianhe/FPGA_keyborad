// ============================================================================
//  ULPI Wrapper - UTMI到ULPI协议转换
//  连接外部ULPI PHY (如USB3300)
// ============================================================================

module ulpi_wrapper (
    input  wire        clk,
    input  wire        rst_n,

    // ULPI物理接口
    output wire        ulpi_stp,
    input  wire        ulpi_dir,
    input  wire        ulpi_nxt,
    inout  wire [7:0]  ulpi_data,

    // 发送接口 (到USB Core)
    input  wire [7:0]  tx_data,
    input  wire        tx_valid,
    output wire        tx_ready,
    input  wire        tx_cmd,

    // 接收接口 (到USB Core)
    output wire [7:0]  rx_data,
    output wire        rx_active,
    output wire        rx_valid,
    output wire        rx_error,

    // 状态指示
    output wire [1:0]  line_state,
    output wire        host_disconnect
);

    // ================================================================
    // 内部信号
    // ================================================================

    reg         data_out_en;
    reg  [7:0]  data_out;
    wire [7:0]  data_in;

    // 状态机
    localparam IDLE         = 3'd0,
               TX_CMD       = 3'd1,
               TX_DATA      = 3'd2,
               TX_WAIT      = 3'd3,
               RX_ACTIVE    = 3'd4,
               RX_DATA      = 3'd5,
               RX_WAIT      = 3'd6;

    reg [2:0]  state, next_state;

    reg        tx_ready_reg;
    reg [7:0]  tx_data_reg;
    reg        tx_cmd_reg;
    reg [7:0]  rx_data_reg;
    reg        rx_valid_reg;
    reg        rx_active_reg;
    reg        rx_error_reg;

    // ================================================================
    // 三态总线控制
    // ================================================================

    assign ulpi_data = data_out_en ? data_out : 8'hZZ;
    assign data_in   = ulpi_data;

    assign ulpi_stp  = (state == TX_WAIT) || (state == RX_WAIT);
    assign tx_ready  = tx_ready_reg;
    assign rx_data   = rx_data_reg;
    assign rx_active = rx_active_reg;
    assign rx_valid  = rx_valid_reg;
    assign rx_error  = rx_error_reg;
    assign line_state       = 2'b00;
    assign host_disconnect  = 1'b0;

    // ================================================================
    // 状态机 - 组合逻辑
    // ================================================================

    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (ulpi_dir) begin
                    next_state = RX_ACTIVE;
                end else if (tx_valid) begin
                    next_state = (tx_cmd) ? TX_CMD : TX_DATA;
                end
            end

            TX_CMD, TX_DATA: begin
                if (ulpi_nxt) next_state = TX_WAIT;
            end

            TX_WAIT: begin
                if (!ulpi_nxt) begin
                    if (tx_valid) begin
                        next_state = (tx_cmd) ? TX_CMD : TX_DATA;
                    end else begin
                        next_state = IDLE;
                    end
                end
            end

            RX_ACTIVE: begin
                if (ulpi_nxt) begin
                    next_state = RX_DATA;
                end else if (!ulpi_dir) begin
                    next_state = IDLE;
                end
            end

            RX_DATA: begin
                if (!ulpi_dir) begin
                    next_state = IDLE;
                end else if (ulpi_nxt) begin
                    next_state = RX_DATA;
                end else begin
                    next_state = RX_WAIT;
                end
            end

            RX_WAIT: begin
                if (ulpi_nxt) begin
                    next_state = RX_DATA;
                end else if (!ulpi_dir) begin
                    next_state = IDLE;
                end
            end

            default: next_state = IDLE;
        endcase
    end

    // ================================================================
    // 时序逻辑
    // ================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    // 数据输出与发送控制
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out_en  <= 1'b0;
            data_out     <= 8'h00;
            tx_ready_reg <= 1'b1;
            tx_data_reg  <= 8'h00;
            tx_cmd_reg   <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    data_out_en  <= 1'b0;
                    tx_ready_reg <= 1'b1;
                    if (tx_valid && !ulpi_dir) begin
                        tx_data_reg <= tx_data;
                        tx_cmd_reg  <= tx_cmd;
                    end
                end

                TX_CMD, TX_DATA: begin
                    data_out_en <= 1'b1;
                    if (state == TX_CMD) begin
                        data_out <= {6'h00, tx_cmd_reg, 1'b0};
                    end else begin
                        data_out <= tx_data_reg;
                    end
                    tx_ready_reg <= 1'b0;
                end

                TX_WAIT: begin
                    data_out_en <= 1'b0;
                    tx_ready_reg <= 1'b1;
                    if (tx_valid && !ulpi_dir) begin
                        tx_data_reg <= tx_data;
                        tx_cmd_reg  <= tx_cmd;
                    end
                end

                RX_ACTIVE, RX_DATA, RX_WAIT: begin
                    data_out_en <= 1'b0;
                    tx_ready_reg <= 1'b1;
                end

                default: begin
                    data_out_en <= 1'b0;
                    tx_ready_reg <= 1'b1;
                end
            endcase
        end
    end

    // 接收数据路径
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_data_reg   <= 8'h00;
            rx_valid_reg  <= 1'b0;
            rx_active_reg <= 1'b0;
            rx_error_reg  <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    rx_valid_reg  <= 1'b0;
                    rx_active_reg <= 1'b0;
                    rx_error_reg  <= 1'b0;
                end

                RX_ACTIVE: begin
                    rx_active_reg <= 1'b1;
                    if (ulpi_nxt) begin
                        rx_data_reg  <= data_in;
                        rx_valid_reg <= 1'b1;
                    end else begin
                        rx_valid_reg <= 1'b0;
                    end
                    rx_error_reg <= 1'b0;
                end

                RX_DATA: begin
                    rx_active_reg <= 1'b1;
                    if (ulpi_nxt) begin
                        rx_data_reg  <= data_in;
                        rx_valid_reg <= 1'b1;
                    end else begin
                        rx_valid_reg <= 1'b0;
                    end
                    rx_error_reg <= 1'b0;
                end

                RX_WAIT: begin
                    rx_active_reg <= 1'b1;
                    rx_valid_reg  <= 1'b0;
                    if (ulpi_nxt) begin
                        rx_data_reg <= data_in;
                    end
                    rx_error_reg <= 1'b0;
                end

                default: begin
                    rx_valid_reg  <= 1'b0;
                    rx_active_reg <= 1'b0;
                    rx_error_reg  <= 1'b0;
                end
            endcase
        end
    end

endmodule