//=============================================================================
// 第 4 层（应用层 A）：kbd_gpio.v  ——  按键 IO 直连采集 + 消抖
//------------------------------------------------------------------------------
//  与矩阵扫描的区别：
//    * 每根 IO 对应一个物理按键，没有行驱动、没有扫描节拍，
//      所有按键"同时"可见，天然没有鬼键、没有扫描延迟。
//    * 代价是 IO 数量 = 按键数量。GW5A-LV25MG121NES（121 脚）在扣掉
//      ULPI 9 根 + 时钟 + 复位 + PHY_RESETB + 3 个 LED + 调试脚之后，
//      大约还剩 70 根可用 IO，直连上限在 60~70 键之间。
//      键数更多只能换更大封装（如 MG196/MG256）或加 IO 扩展器。
//
//  电气约定：
//    * 按键一端接 IO，另一端接 GND；IO 内部上拉（PULL_MODE=UP）。
//    * 松开 = 1，按下 = 0（ACTIVE_LOW=1 把它翻成内部"1 = 按下"）。
//    * 若你的板子是按键接 VCC + 下拉电阻，把 ACTIVE_LOW 设为 0。
//
//  消抖：
//    * 两级同步器 → 每 SAMPLE_US 微秒采样一次 → 连续 DB_SAMPLES 次
//      与当前稳定值不一致才接受新值（默认 1ms × 4 = 4ms）。
//    * 每个键有独立的 4 位计数器，互不影响；快速交替敲击也不会被"平均掉"。
//    * key_state 任一位变化时输出 1 周期 changed 脉冲，通知 kbd_hid 重算报告。
//=============================================================================
`timescale 1ns/1ps

module kbd_gpio #(
    parameter integer KEY_NUM    = 61,          // 按键数量 = 占用 IO 数量
    parameter integer CLK_FREQ   = 60_000_000,
    parameter integer SAMPLE_US  = 1000,        // 采样周期 (us)
    parameter integer DB_SAMPLES = 4,           // 连续一致次数才认可
    parameter         ACTIVE_LOW = 1            // 1 = 按下为低（上拉+接地）
)(
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire [KEY_NUM-1:0]      key_in,      // 每键一根 IO

    output reg  [KEY_NUM-1:0]      key_state,   // 1 = 按下（已消抖）
    output reg                     changed      // 单周期：按键状态发生变化
);

    //------------------------------------------------------------------
    // 1. 两级同步（按键是异步信号，必须过同步器）
    //------------------------------------------------------------------
    reg [KEY_NUM-1:0] sync0, sync1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync0 <= {KEY_NUM{1'b0}};
            sync1 <= {KEY_NUM{1'b0}};
        end else begin
            sync0 <= key_in;
            sync1 <= sync0;
        end
    end

    // 极性归一：统一成 "1 = 按下"
    wire [KEY_NUM-1:0] raw = ACTIVE_LOW ? ~sync1 : sync1;

    //------------------------------------------------------------------
    // 2. 采样节拍（SAMPLE_US 微秒）
    //------------------------------------------------------------------
    localparam integer TICK_DIV = (CLK_FREQ / 1_000_000) * SAMPLE_US;

    reg [19:0] div_cnt;
    reg        tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 20'd0;
            tick    <= 1'b0;
        end else if (div_cnt == (TICK_DIV - 1)) begin
            div_cnt <= 20'd0;
            tick    <= 1'b1;
        end else begin
            div_cnt <= div_cnt + 20'd1;
            tick    <= 1'b0;
        end
    end

    //------------------------------------------------------------------
    // 3. 逐键独立消抖
    //    每键一个 4 位计数器：采样值与稳定值相同则清零，
    //    连续 DB_SAMPLES 次不同则接受新值。
    //------------------------------------------------------------------
    genvar g;

    generate
    for (g = 0; g < KEY_NUM; g = g + 1) begin : g_db
        reg [3:0] cnt;

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                cnt          <= 4'd0;
                key_state[g] <= 1'b0;
            end else if (tick) begin
                if (raw[g] == key_state[g]) begin
                    cnt <= 4'd0;                       // 一致，清计数
                end else if (cnt >= (DB_SAMPLES - 1)) begin
                    key_state[g] <= raw[g];            // 抖动期结束，接受
                    cnt          <= 4'd0;
                end else begin
                    cnt <= cnt + 4'd1;
                end
            end
        end
    end
    endgenerate

    //------------------------------------------------------------------
    // 4. 变化检测：任一位变化 → 1 周期脉冲
    //------------------------------------------------------------------
    reg [KEY_NUM-1:0] prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev    <= {KEY_NUM{1'b0}};
            changed <= 1'b0;
        end else begin
            changed <= 1'b0;
            if (key_state != prev) begin
                changed <= 1'b1;
                prev    <= key_state;
            end
        end
    end

endmodule
