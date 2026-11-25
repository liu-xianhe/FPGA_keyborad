// ============================================================================
// USB 2.0 High-Speed Full-Size Keyboard with Internal Debounce
//  - 按键低电平有效，内部上拉
//  - 内部消抖：5ms采样，连续3次稳定
// ============================================================================

module usb_device_top (
    input  wire        clk_60m,
    input  wire        rst_n,
    // ULPI
    output wire        ulpi_stp,
    input  wire        ulpi_dir,
    input  wire        ulpi_nxt,
    inout  wire [7:0]  ulpi_data,
    // 按键输入（低电平有效）
    input  wire [103:0] keys,
    input  wire         fn_key,
    // LED状态（低电平有效）
    output wire         led_caps,
    output wire         led_num,
    output wire         led_scroll
);

    // ----------------------------------------------------------------
    // 内部信号
    // ----------------------------------------------------------------
    wire [7:0]  phy_tx_data;
    wire        phy_tx_valid;
    wire        phy_tx_ready;
    wire        phy_tx_cmd;
    wire [7:0]  phy_rx_data;
    wire        phy_rx_valid;
    wire        phy_rx_active;
    wire        phy_rx_error;

    wire [7:0]  core_tx_data;
    wire        core_tx_valid;
    wire        core_tx_ready;
    wire [7:0]  core_rx_data;
    wire        core_rx_valid;
    wire        core_rx_last;
    wire        core_rx_error;

    // 消抖后的按键状态（高电平有效表示按下）
    wire [104:0] key_stable;

    // ----------------------------------------------------------------
    // 消抖模块实例化（单独模块）
    // ----------------------------------------------------------------
    debounce_105 u_debounce (
        .clk         (clk_60m),
        .rst_n       (rst_n),
        .keys_in     ({fn_key, keys}),   // {fn, 104 keys}
        .keys_stable (key_stable),
        .changed     (keys_changed)
    );

    // ----------------------------------------------------------------
    // 实例化 ULPI Wrapper
    // ----------------------------------------------------------------
    ulpi_wrapper u_ulpi_wrapper (
        .clk   (clk_60m),
        .rst_n (rst_n),
        .ulpi_stp  (ulpi_stp),
        .ulpi_dir  (ulpi_dir),
        .ulpi_nxt  (ulpi_nxt),
        .ulpi_data (ulpi_data),
        .tx_data   (phy_tx_data),
        .tx_valid  (phy_tx_valid),
        .tx_ready  (phy_tx_ready),
        .tx_cmd    (phy_tx_cmd),
        .rx_data   (phy_rx_data),
        .rx_active (phy_rx_active),
        .rx_valid  (phy_rx_valid),
        .rx_error  (phy_rx_error),
        .line_state(),
        .host_disconnect()
    );

    // ----------------------------------------------------------------
    // 实例化 USB Device Core
    // ----------------------------------------------------------------
    usb_device_core u_usb_device_core (
        .clk          (clk_60m),
        .rst_n        (rst_n),
        .phy_tx_data  (phy_tx_data),
        .phy_tx_valid (phy_tx_valid),
        .phy_tx_ready (phy_tx_ready),
        .phy_tx_cmd   (phy_tx_cmd),
        .phy_rx_data  (phy_rx_data),
        .phy_rx_valid (phy_rx_valid),
        .phy_rx_active(phy_rx_active),
        .phy_rx_error (phy_rx_error),
        .app_tx_data  (core_tx_data),
        .app_tx_valid (core_tx_valid),
        .app_tx_ready (core_tx_ready),
        .app_rx_data  (core_rx_data),
        .app_rx_valid (core_rx_valid),
        .app_rx_last  (core_rx_last),
        .app_rx_error (core_rx_error)
    );

    // ----------------------------------------------------------------
    // 实例化 HID 复合设备
    // ----------------------------------------------------------------
    usb_hid_composite u_usb_hid_composite (
        .clk           (clk_60m),
        .rst_n         (rst_n),
        .core_tx_data  (core_tx_data),
        .core_tx_valid (core_tx_valid),
        .core_tx_ready (core_tx_ready),
        .core_rx_data  (core_rx_data),
        .core_rx_valid (core_rx_valid),
        .core_rx_last  (core_rx_last),
        .core_rx_error (core_rx_error),
        .all_keys      (key_stable),
        .keys_changed  (keys_changed),
        .led_caps      (led_caps),
        .led_num       (led_num),
        .led_scroll    (led_scroll)
    );

endmodule

// ============================================================================
// 独立消抖模块：105位按键，低电平有效，内部上拉取反
// ============================================================================
module debounce_105 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [104:0] keys_in,    // 低电平有效（外部按键）
    output reg  [104:0] keys_stable, // 高电平有效（消抖后）
    output reg          changed
);
    localparam DEBOUNCE_MS = 5;
    localparam CNT_MAX = 60_000_000 / 1000 * DEBOUNCE_MS; // 300000

    reg [18:0] debounce_cnt;
    reg        sample_tick;

    // 每个按键的消抖状态
    reg [104:0] key_sampled;      // 当前采样（高电平有效）
    reg [104:0] key_debounced;    // 上一次稳定值
    reg [1:0]   same_cnt [104:0]; // 连续相同次数

    integer i;

    // 定时器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debounce_cnt <= 0;
            sample_tick <= 1'b0;
        end else begin
            if (debounce_cnt == CNT_MAX - 1) begin
                debounce_cnt <= 0;
                sample_tick <= 1'b1;
            end else begin
                debounce_cnt <= debounce_cnt + 1;
                sample_tick <= 1'b0;
            end
        end
    end

    // 消抖逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sampled <= 0;
            key_debounced <= 0;
            keys_stable <= 0;
            changed <= 0;
            for (i=0; i<105; i=i+1) same_cnt[i] <= 2'd0;
        end else begin
            changed <= 1'b0;
            if (sample_tick) begin
                for (i=0; i<105; i=i+1) begin
                    // 采样并取反（低有效转高有效）
                    key_sampled[i] <= ~keys_in[i];

                    // 比较与上次稳定值
                    if (key_sampled[i] == key_debounced[i]) begin
                        if (same_cnt[i] < 2'd3)
                            same_cnt[i] <= same_cnt[i] + 2'd1;
                        if (same_cnt[i] == 2'd2) begin   // 连续3次相同
                            if (keys_stable[i] != key_debounced[i]) begin
                                keys_stable[i] <= key_debounced[i];
                                changed <= 1'b1;
                            end
                        end
                    end else begin
                        same_cnt[i] <= 2'd0;
                        key_debounced[i] <= key_sampled[i];
                    end
                end
            end
        end
    end

endmodule