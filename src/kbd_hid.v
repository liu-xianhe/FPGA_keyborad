//=============================================================================
// 第 4 层（应用层 B）：kbd_hid.v  ——  键位表 + HID Boot Keyboard 报告生成
//------------------------------------------------------------------------------
//  输入：KEY_NUM 位按键状态（来自 kbd_gpio，1 = 按下）
//        索引 = 按键所占用的那根 IO 的序号，不再有“行号/列号”的概念
//  输出：8 字节 HID 报告 + 报告序号（序号变化即表示“有新报告要上报”）
//        report[7:0]   = 修饰键位图 (bit0=LCtrl ... bit7=RGUI)
//        report[15:8]  = 保留 (0x00)
//        report[23:16] = 第 1 个按键 Usage ... report[63:56] = 第 6 个
//
//  6KRO：同时按下超过 6 个非修饰键时，6 个位置全部填 0x01(ErrorRollOver)
//  Fn 层：按住 Fn 时对部分按键做二次映射（默认把 WASD 映射成方向键）
//  大小写/数字锁：由主机根据 LED 状态自行处理，设备只上报 Usage + 修饰键
//=============================================================================
`timescale 1ns/1ps

module kbd_hid #(
    parameter integer KEY_NUM  = 87,          // 按键数量（= kbd_gpio 的 IO 数）
    parameter integer FN_INDEX = 60           // Fn 键的 IO 序号（主区底排最后一个）
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [KEY_NUM-1:0] key_state,
    input  wire         changed,          // kbd_gpio 的按键变化脉冲

    output reg  [63:0]  report,
    output reg  [7:0]   report_seq
);

    //==================================================================
    // 键位表：IO 序号 → HID Usage
    //   87 键 TKL（Tenkeyless）ANSI 配列，按三段编号：
    //
    //   一、主键区 0..60（就是 60% 配列，和上一版完全一致，已接的线不用动）
    //     0..13   Esc 1..0 - = Backspace
    //     14..27  Tab Q..P [ ] backslash
    //     28..40  Caps A..L ; ' Enter
    //     41..52  LShift Z..M , . / RShift
    //     53..60  LCtrl LGui LAlt Space RAlt RGui RCtrl Fn
    //
    //   二、功能键行 61..76
    //     61..72  F1..F12
    //     73..75  PrintScreen ScrollLock Pause
    //     76      Menu（主键区底排省下来的那个键）
    //
    //   三、编辑 / 方向区 77..86
    //     77..82  Insert Home PageUp Delete End PageDown
    //     83..86  Up Down Left Right
    //
    //   接 PCB 时按这个顺序焊到 key_in[0]..key_in[86] 即可；
    //   若走线顺序不同，只改这张表，其余逻辑一行都不用动。
    //   键数不是 87 时：改 KEY_NUM，并裁剪 / 增加本表的 case 分支。
    //==================================================================
    function [7:0] keymap;
        input [6:0] idx;
        begin
            case (idx)
            //---------------- 第 1 排：Esc + 数字行
            7'd0 : keymap = 8'h29;   // Esc
            7'd1 : keymap = 8'h1E;   // 1
            7'd2 : keymap = 8'h1F;   // 2
            7'd3 : keymap = 8'h20;   // 3
            7'd4 : keymap = 8'h21;   // 4
            7'd5 : keymap = 8'h22;   // 5
            7'd6 : keymap = 8'h23;   // 6
            7'd7 : keymap = 8'h24;   // 7
            7'd8 : keymap = 8'h25;   // 8
            7'd9 : keymap = 8'h26;   // 9
            7'd10: keymap = 8'h27;   // 0
            7'd11: keymap = 8'h2D;   // -
            7'd12: keymap = 8'h2E;   // =
            7'd13: keymap = 8'h2A;   // Backspace
            //---------------- 第 2 排：Tab + QWERTY
            7'd14: keymap = 8'h2B;   // Tab
            7'd15: keymap = 8'h14;   // Q
            7'd16: keymap = 8'h1A;   // W
            7'd17: keymap = 8'h08;   // E
            7'd18: keymap = 8'h15;   // R
            7'd19: keymap = 8'h17;   // T
            7'd20: keymap = 8'h1C;   // Y
            7'd21: keymap = 8'h18;   // U
            7'd22: keymap = 8'h0C;   // I
            7'd23: keymap = 8'h12;   // O
            7'd24: keymap = 8'h13;   // P
            7'd25: keymap = 8'h2F;   // [
            7'd26: keymap = 8'h30;   // ]
            7'd27: keymap = 8'h31;   // backslash
            //---------------- 第 3 排：Caps + ASDF
            7'd28: keymap = 8'h39;   // CapsLock
            7'd29: keymap = 8'h04;   // A
            7'd30: keymap = 8'h16;   // S
            7'd31: keymap = 8'h07;   // D
            7'd32: keymap = 8'h09;   // F
            7'd33: keymap = 8'h0A;   // G
            7'd34: keymap = 8'h0B;   // H
            7'd35: keymap = 8'h0D;   // J
            7'd36: keymap = 8'h0E;   // K
            7'd37: keymap = 8'h0F;   // L
            7'd38: keymap = 8'h33;   // ;
            7'd39: keymap = 8'h34;   // '
            7'd40: keymap = 8'h28;   // Enter
            //---------------- 第 4 排：LShift + ZXCV
            7'd41: keymap = 8'hE1;   // LShift
            7'd42: keymap = 8'h1D;   // Z
            7'd43: keymap = 8'h1B;   // X
            7'd44: keymap = 8'h06;   // C
            7'd45: keymap = 8'h19;   // V
            7'd46: keymap = 8'h05;   // B
            7'd47: keymap = 8'h11;   // N
            7'd48: keymap = 8'h10;   // M
            7'd49: keymap = 8'h36;   // ,
            7'd50: keymap = 8'h37;   // .
            7'd51: keymap = 8'h38;   // /
            7'd52: keymap = 8'hE5;   // RShift
            //---------------- 第 5 排：功能键 + 空格
            7'd53: keymap = 8'hE0;   // LCtrl
            7'd54: keymap = 8'hE3;   // LGui
            7'd55: keymap = 8'hE2;   // LAlt
            7'd56: keymap = 8'h2C;   // Space
            7'd57: keymap = 8'hE6;   // RAlt
            7'd58: keymap = 8'hE7;   // RGui
            7'd59: keymap = 8'hE4;   // RCtrl
            7'd60: keymap = 8'h00;   // Fn 本身不上报（只作为二次映射的修饰）
            //---------------- 功能键行：F1..F12 + 三个系统键
            7'd61: keymap = 8'h3A;   // F1
            7'd62: keymap = 8'h3B;   // F2
            7'd63: keymap = 8'h3C;   // F3
            7'd64: keymap = 8'h3D;   // F4
            7'd65: keymap = 8'h3E;   // F5
            7'd66: keymap = 8'h3F;   // F6
            7'd67: keymap = 8'h40;   // F7
            7'd68: keymap = 8'h41;   // F8
            7'd69: keymap = 8'h42;   // F9
            7'd70: keymap = 8'h43;   // F10
            7'd71: keymap = 8'h44;   // F11
            7'd72: keymap = 8'h45;   // F12
            7'd73: keymap = 8'h46;   // PrintScreen
            7'd74: keymap = 8'h47;   // ScrollLock
            7'd75: keymap = 8'h48;   // Pause
            7'd76: keymap = 8'h65;   // Menu
            //---------------- 编辑 / 方向区
            7'd77: keymap = 8'h49;   // Insert
            7'd78: keymap = 8'h4A;   // Home
            7'd79: keymap = 8'h4B;   // PageUp
            7'd80: keymap = 8'h4C;   // Delete
            7'd81: keymap = 8'h4D;   // End
            7'd82: keymap = 8'h4E;   // PageDown
            7'd83: keymap = 8'h52;   // Up
            7'd84: keymap = 8'h51;   // Down
            7'd85: keymap = 8'h50;   // Left
            7'd86: keymap = 8'h4F;   // Right
            default: keymap = 8'h00;
            endcase
        end
    endfunction

    //==================================================================
    // Fn 二次映射（可按需扩展）
    //==================================================================
    function [7:0] keymap_fn;
        input [6:0] idx;
        begin
            case (idx)
            //---------------- Fn + WASD → 方向键
            7'd16: keymap_fn = 8'h52;   // Fn + W → Up
            7'd29: keymap_fn = 8'h50;   // Fn + A → Left
            7'd30: keymap_fn = 8'h51;   // Fn + S → Down
            7'd31: keymap_fn = 8'h4F;   // Fn + D → Right
            //---------------- Fn + 数字行 → F1..F12（60% 配列必备）
            7'd1 : keymap_fn = 8'h3A;   // Fn + 1  → F1
            7'd2 : keymap_fn = 8'h3B;   // Fn + 2  → F2
            7'd3 : keymap_fn = 8'h3C;   // Fn + 3  → F3
            7'd4 : keymap_fn = 8'h3D;   // Fn + 4  → F4
            7'd5 : keymap_fn = 8'h3E;   // Fn + 5  → F5
            7'd6 : keymap_fn = 8'h3F;   // Fn + 6  → F6
            7'd7 : keymap_fn = 8'h40;   // Fn + 7  → F7
            7'd8 : keymap_fn = 8'h41;   // Fn + 8  → F8
            7'd9 : keymap_fn = 8'h42;   // Fn + 9  → F9
            7'd10: keymap_fn = 8'h43;   // Fn + 0  → F10
            7'd11: keymap_fn = 8'h44;   // Fn + -  → F11
            7'd12: keymap_fn = 8'h45;   // Fn + =  → F12
            //---------------- Fn + Backspace → Delete
            7'd13: keymap_fn = 8'h4C;
            default: keymap_fn = keymap(idx);
            endcase
        end
    endfunction

    //==================================================================
    // 报告生成状态机：每次按键变化，扫一遍 112 个键
    //==================================================================
    localparam [1:0] K_IDLE = 2'd0,
                     K_SCAN = 2'd1,
                     K_FIN  = 2'd2;

    reg [1:0] ks;
    reg [6:0] idx;
    reg [7:0] mods;
    reg [7:0] code0, code1, code2, code3, code4, code5;
    reg [2:0] ncode;
    reg       overflow;
    reg       dirty;

    wire [7:0] u_now = (key_state[FN_INDEX]) ? keymap_fn(idx) : keymap(idx);

    // 组合出待比较的新报告
    wire [47:0] codes_c = overflow ? {6{8'h01}}
                                   : {code5, code4, code3, code2, code1, code0};
    wire [63:0] rep_c   = {codes_c, 8'h00, mods};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ks         <= K_IDLE;
            idx        <= 7'd0;
            mods       <= 8'h00;
            code0      <= 8'h00;  code1 <= 8'h00;  code2 <= 8'h00;
            code3      <= 8'h00;  code4 <= 8'h00;  code5 <= 8'h00;
            ncode      <= 3'd0;
            overflow   <= 1'b0;
            dirty      <= 1'b0;
            report     <= 64'd0;
            report_seq <= 8'd0;
        end else begin
            case (ks)
            //----------------------------------------------------------
            K_IDLE: begin
                if (changed) begin
                    idx      <= 7'd0;
                    mods     <= 8'h00;
                    ncode    <= 3'd0;
                    overflow <= 1'b0;
                    dirty    <= 1'b0;
                    code0    <= 8'h00; code1 <= 8'h00; code2 <= 8'h00;
                    code3    <= 8'h00; code4 <= 8'h00; code5 <= 8'h00;
                    ks       <= K_SCAN;
                end
            end
            //----------------------------------------------------------
            K_SCAN: begin
                if (changed) dirty <= 1'b1;      // 扫描期间又变了，收尾后重来

                if (key_state[idx]) begin
                    if (u_now >= 8'hE0 && u_now <= 8'hE7) begin
                        mods[u_now[2:0]] <= 1'b1;      // 修饰键进 bit0..bit7
                    end else if (u_now != 8'h00) begin
                        case (ncode)
                        3'd0: code0 <= u_now;
                        3'd1: code1 <= u_now;
                        3'd2: code2 <= u_now;
                        3'd3: code3 <= u_now;
                        3'd4: code4 <= u_now;
                        3'd5: code5 <= u_now;
                        default: overflow <= 1'b1;     // 超过 6 键
                        endcase
                        if (ncode != 3'd6) ncode <= ncode + 3'd1;
                    end
                end

                if (idx == (KEY_NUM - 1)) ks <= K_FIN;
                idx <= idx + 7'd1;
            end
            //----------------------------------------------------------
            K_FIN: begin
                if (rep_c != report) begin
                    report     <= rep_c;
                    report_seq <= report_seq + 8'd1;   // 通知协议层上报
                end

                if (dirty) begin
                    idx      <= 7'd0;
                    mods     <= 8'h00;
                    ncode    <= 3'd0;
                    overflow <= 1'b0;
                    dirty    <= 1'b0;
                    code0    <= 8'h00; code1 <= 8'h00; code2 <= 8'h00;
                    code3    <= 8'h00; code4 <= 8'h00; code5 <= 8'h00;
                    ks       <= K_SCAN;
                end else begin
                    ks <= K_IDLE;
                end
            end
            //----------------------------------------------------------
            default: ks <= K_IDLE;
            endcase
        end
    end

endmodule
