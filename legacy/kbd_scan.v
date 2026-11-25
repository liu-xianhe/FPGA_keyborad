//=============================================================================
// 第 4 层（应用层 A）：kbd_scan.v  ——  键盘矩阵扫描 + 消抖
//------------------------------------------------------------------------------
//  8 行 × 14 列 = 112 键位（可按 PCB 修改 ROWS/COLS）
//    * 行线低电平有效，逐行扫描；列线上拉，按下时读到 0（带二极管防鬼键）
//    * 每行驻留 SCAN_US 微秒，整帧 1.6ms
//    * 消抖：连续 2 帧（3.2ms）采样一致才更新，输出 1 周期 changed 脉冲
//    * DIRECT_MODE=1 时跳过矩阵，直接把 keys_flat 当作并行按键输入
//      （1 = 按下），方便用拨码开关/按钮做功能验证
//=============================================================================
`timescale 1ns/1ps

module kbd_scan #(
    parameter integer CLK_FREQ    = 60_000_000,
    parameter integer SCAN_US     = 200,     // 每行驻留时间(us)
    parameter         DIRECT_MODE = 0
)(
    input  wire         clk,
    input  wire         rst_n,

    output wire [7:0]   key_row,      // 行驱动（低有效）
    input  wire [13:0]  key_col,      // 列输入（上拉，按下=0）
    input  wire [111:0] keys_flat,    // 直接模式下的按键输入

    output reg  [111:0] key_state,    // 1 = 按下（已消抖）
    output reg          changed       // 单周期：按键状态发生变化
);

    //------------------------------------------------------------------
    // 行扫描节拍
    //------------------------------------------------------------------
    localparam integer ROW_DIV = (CLK_FREQ / 1_000_000) * SCAN_US;

    reg [15:0] div_cnt;
    reg        row_tick;
    reg [2:0]  row_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt  <= 16'd0;
            row_tick <= 1'b0;
            row_sel  <= 3'd0;
        end else begin
            if (div_cnt == (ROW_DIV - 1)) begin
                div_cnt  <= 16'd0;
                row_tick <= 1'b1;
            end else begin
                div_cnt  <= div_cnt + 16'd1;
                row_tick <= 1'b0;
            end

            if (row_tick) row_sel <= row_sel + 3'd1;   // 0..7 循环
        end
    end

    assign key_row = ~(8'h01 << row_sel);

    //------------------------------------------------------------------
    // 采样：在本行驻留结束的瞬间采样（此时电平已稳定 SCAN_US）
    //------------------------------------------------------------------
    reg [111:0] raw_scan;
    integer     k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raw_scan <= 112'd0;
        end else if (row_tick) begin
            // 按位写入（避免变基址的部分选择，兼容性最好）
            for (k = 0; k < 14; k = k + 1)
                raw_scan[row_sel*14 + k] <= ~key_col[k];
        end
    end

    //------------------------------------------------------------------
    // 直接模式旁路
    //------------------------------------------------------------------
    wire [111:0] raw_all = DIRECT_MODE ? keys_flat : raw_scan;

    //------------------------------------------------------------------
    // 消抖：连续 2 帧一致才接受
    //------------------------------------------------------------------
    reg [111:0] hist1, hist2;
    reg         frame_tick;

    always @(*) frame_tick = row_tick && (row_sel == 3'd7);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hist1     <= 112'd0;
            hist2     <= 112'd0;
            key_state <= 112'd0;
            changed   <= 1'b0;
        end else begin
            changed <= 1'b0;
            if (frame_tick) begin
                if ((raw_all == hist1) && (hist1 == hist2) && (raw_all != key_state)) begin
                    key_state <= raw_all;
                    changed   <= 1'b1;
                end
                hist2 <= hist1;
                hist1 <= raw_all;
            end
        end
    end

endmodule
