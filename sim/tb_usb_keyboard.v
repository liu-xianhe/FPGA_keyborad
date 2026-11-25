//=============================================================================
// tb_usb_keyboard.v  ——  ULPI PHY + USB 主机 行为模型，跑完整枚举流程
//------------------------------------------------------------------------------
//  验证内容：
//    1. 上电时序 / FUNC_CTRL 寄存器写
//    2. 控制传输：GET_DESCRIPTOR(device 8/18 字节)、SET_ADDRESS、
//       GET_DESCRIPTOR(config)、GET_DESCRIPTOR(HID report)、SET_CONFIGURATION
//    3. HID 中断端点：按键 → 报告 → 主机轮询 EP1 IN 拿到 0x04('A')
//    4. HID 类请求 SET_REPORT → LED 输出
//
//  模型时序约定：
//    * 驱动 DUT 输入用 posedge
//    * 采样 DUT 输出用 negedge（稳定点）
//    * 表项：hpkt/hlen 待发，dpkt/dlen/dpid 已收，rdbuf/rd_len 数据阶段快照
//=============================================================================
`timescale 1ns/1ps

module tb_usb_keyboard;

    //==================================================================
    // 时钟 / 复位
    //==================================================================
    reg clk = 1'b0;
    always #8.333 clk = ~clk;              // 60MHz

    reg rst_btn_n;

    //==================================================================
    // ULPI 总线
    //==================================================================
    wire        ulpi_stp;
    wire        ulpi_dir;
    wire        ulpi_nxt;
    wire [7:0]  ulpi_data;

    reg         dir_drv, nxt_drv;
    reg  [7:0]  data_drv;
    reg         data_oe;

    assign ulpi_dir  = dir_drv;
    assign ulpi_nxt  = nxt_drv;
    assign ulpi_data = data_oe ? data_drv : 8'bz;

    //==================================================================
    // 按键（IO 直连，每键一根）/ LED
    //==================================================================
    localparam integer KEYN = 87;

    reg  [KEYN-1:0] key_in;      // 1 = 松开（内部上拉），0 = 按下
    wire        led_caps, led_num, led_scroll, dbg_configured;

    initial key_in = {KEYN{1'b1}};   // 上电全部松开

    //==================================================================
    // DUT
    //==================================================================
    usb_keyboard_top #(
        .CLK_FREQ       (60_000_000),
        .FUNC_CTRL_INIT (8'h08),
        .LED_ACTIVE_LOW (1),
        .PHY_RST_MS     (1),
        .INIT_MS        (1),
        .KEY_NUM        (KEYN)
    ) dut (
        .clk_ulpi      (clk),
        .rst_btn_n     (rst_btn_n),
        .phy_reset_n   (),
        .ulpi_stp      (ulpi_stp),
        .ulpi_dir      (ulpi_dir),
        .ulpi_nxt      (ulpi_nxt),
        .ulpi_data     (ulpi_data),
        .key_in        (key_in),
        .led_caps      (led_caps),
        .led_num       (led_num),
        .led_scroll    (led_scroll),
        .dbg_configured(dbg_configured)
    );

    //==================================================================
    // 主机侧缓冲
    //==================================================================
    reg [7:0] hpkt  [0:63];      // 待发送
    integer   hlen;
    reg [7:0] dpkt  [0:63];      // 刚收到
    integer   dlen;
    reg [3:0] dpid;
    reg [7:0] rdbuf [0:63];      // 数据阶段快照
    integer   rd_len;
    reg [6:0] cur_addr;

    //------------------------------------------------------------------
    // 发送一包：hpkt[0]=PID, hpkt[1..hlen-1]=负载
    //------------------------------------------------------------------
    task host_send;
        integer i;
        begin
            @(posedge clk);
            data_oe  <= 1'b1;
            dir_drv  <= 1'b1;
            nxt_drv  <= 1'b1;
            data_drv <= hpkt[0];
            for (i = 1; i < hlen; i = i + 1) begin
                @(posedge clk);
                data_drv <= hpkt[i];
            end
            @(posedge clk);
            dir_drv  <= 1'b0;
            data_oe  <= 1'b0;
            nxt_drv  <= 1'b0;
            data_drv <= 8'h00;
        end
    endtask

    //------------------------------------------------------------------
    // 接收设备发来的一包
    //------------------------------------------------------------------
    task host_recv;
        integer n;
        begin
            dlen = 0;
            dpid = 4'hF;
            @(posedge clk);
            nxt_drv <= 1'b1;

            n = 0;
            while (ulpi_stp !== 1'b1 && n < 400) begin
                @(negedge clk);
                n = n + 1;
            end
            if (ulpi_stp === 1'b1) begin
                dpid = {4'h0, ulpi_data[3:0]};
                @(negedge clk);
                // 只有 DATA0/DATA1 带负载；握手包只有 PID。
                // 用 DUT 的 phy_tx_valid 精确判定“总线上是负载字节”，
                // 这样零长度包（ZLP）不会被误采到 Idle 拍。
                if (dpid == 4'h3 || dpid == 4'hB) begin
                    while (ulpi_stp === 1'b0 && dlen < 64) begin
                        if (dut.phy_tx_valid === 1'b1) begin
                            dpkt[dlen] = ulpi_data;
                            dlen = dlen + 1;
                        end
                        @(negedge clk);
                    end
                end
            end
            @(posedge clk);
            nxt_drv <= 1'b0;
        end
    endtask

    task send_token;
        input [7:0] pid;
        input [6:0] addr;
        input [3:0] ep;
        begin
            hpkt[0] = pid;
            hpkt[1] = {1'b0, addr};
            hpkt[2] = {5'h00, ep[2:0]};
            hlen    = 3;
            host_send;
        end
    endtask

    task send_ack;
        begin
            hpkt[0] = 8'hD2;
            hlen    = 1;
            host_send;
        end
    endtask

    //------------------------------------------------------------------
    // SETUP 阶段：令牌 + DATA0 + 8 字节
    //------------------------------------------------------------------
    task usb_setup;
        input [7:0]  bmrt;
        input [7:0]  breq;
        input [15:0] wval;
        input [15:0] widx;
        input [15:0] wlen;
        begin
            send_token(8'h2D, cur_addr, 4'd0);

            hpkt[0]  = 8'hC3;              // DATA0
            hpkt[1]  = bmrt;
            hpkt[2]  = breq;
            hpkt[3]  = wval[7:0];
            hpkt[4]  = wval[15:8];
            hpkt[5]  = widx[7:0];
            hpkt[6]  = widx[15:8];
            hpkt[7]  = wlen[7:0];
            hpkt[8]  = wlen[15:8];
            hpkt[9]  = 8'h00;              // CRC16 占位（真实 CRC 由 PHY 生成）
            hpkt[10] = 8'h00;
            hlen     = 11;
            host_send;

            host_recv;                     // 设备 ACK
            if (dpid !== 4'h2) $display("  [WARN] SETUP 未被 ACK, pid=%h", dpid);
        end
    endtask

    //------------------------------------------------------------------
    // 发 IN 令牌，返回设备响应 PID；收到 DATA 自动 ACK
    //------------------------------------------------------------------
    task do_in;
        input  [3:0] ep;
        output [3:0] o_pid;
        begin
            send_token(8'h69, cur_addr, ep);
            host_recv;
            o_pid = dpid;
            if (dpid == 4'h3 || dpid == 4'hB) send_ack;
        end
    endtask

    //------------------------------------------------------------------
    // IN 轮询直到拿到数据（NAK 重试），数据存入 dpkt/dlen
    //------------------------------------------------------------------
    task in_wait_data;
        input  [3:0] ep;
        output [3:0] o_pid;
        integer      g;
        reg   [3:0]  q;
        begin
            q = 4'hF;
            g = 0;
            while ((q !== 4'h3 && q !== 4'hB) && g < 40) begin
                do_in(ep, q);
                g = g + 1;
            end
            o_pid = q;
        end
    endtask

    //------------------------------------------------------------------
    // 控制读：SETUP + 数据阶段（快照到 rdbuf）+ 状态阶段
    //------------------------------------------------------------------
    task ctrl_read;
        input [7:0]  bmrt;
        input [7:0]  breq;
        input [15:0] wval;
        input [15:0] widx;
        input [15:0] wlen;
        reg   [3:0]  p;
        integer      i;
        begin
            usb_setup(bmrt, breq, wval, widx, wlen);

            in_wait_data(4'd0, p);
            rd_len = dlen;
            for (i = 0; i < 64; i = i + 1) rdbuf[i] = dpkt[i];

            // 状态阶段：OUT 零长度包（DATA1，只有 2 字节 CRC）
            send_token(8'hE1, cur_addr, 4'd0);
            hpkt[0] = 8'h4B;               // DATA1
            hpkt[1] = 8'h00;               // CRC16
            hpkt[2] = 8'h00;
            hlen    = 3;
            host_send;
            host_recv;
            if (dpid !== 4'h2) $display("  [WARN] 状态阶段未被 ACK, pid=%h", dpid);
        end
    endtask

    //==================================================================
    // 主流程
    //==================================================================
    integer   ok, fail, guard;
    reg [3:0] p;
    reg [7:0] rep_a, rep_mod;
    integer   i;

    initial begin
        $dumpfile("tb_usb_keyboard.vcd");
        $dumpvars(0, tb_usb_keyboard);

        rst_btn_n = 1'b0;
        dir_drv   = 1'b0;
        nxt_drv   = 1'b0;
        data_oe   = 1'b0;
        data_drv  = 8'h00;
        key_in    = {KEYN{1'b1}};
        cur_addr  = 7'd0;
        hlen      = 0;
        ok        = 0;
        fail      = 0;

        $display("=========================================================");
        $display(" USB3300 分层键盘 —— 仿真开始");
        $display("=========================================================");

        #200;
        rst_btn_n = 1'b1;

        #4500000;                     // 等上电初始化（PHY 复位 + FUNC_CTRL + 内核释放）
        $display("[T=%0t] 初始化完成，configured=%b", $time, dbg_configured);
        if (dbg_configured !== 1'b0) begin
            $display("  [FAIL] 配置前 configured 应为 0");
            fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[1] GET_DESCRIPTOR(Device, 8) @addr0");
        ctrl_read(8'h80, 8'h06, 16'h0100, 16'h0000, 16'd8);
        $display("    收到 %0d 字节: %h %h %h %h", rd_len, rdbuf[0], rdbuf[1], rdbuf[2], rdbuf[3]);
        if (rd_len == 8 && rdbuf[0] == 8'h12 && rdbuf[1] == 8'h01) begin
            $display("    [PASS]"); ok = ok + 1;
        end else begin
            $display("    [FAIL]"); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[2] SET_ADDRESS(5)");
        usb_setup(8'h00, 8'h05, 16'h0005, 16'h0000, 16'd0);
        in_wait_data(4'd0, p);
        if (p == 4'h3 || p == 4'hB) begin
            $display("    状态阶段 IN ZLP 正常 (pid=%h)", p); ok = ok + 1;
        end else begin
            $display("    [FAIL] 状态阶段 pid=%h", p); fail = fail + 1;
        end
        cur_addr = 7'd5;

        //----------------------------------------------------------
        $display("\n[3] GET_DESCRIPTOR(Device, 18) @addr5");
        ctrl_read(8'h80, 8'h06, 16'h0100, 16'h0000, 16'd18);
        $display("    收到 %0d 字节, VID=%h%h PID=%h%h", rd_len,
                 rdbuf[9], rdbuf[8], rdbuf[11], rdbuf[10]);
        if (rd_len == 18 && rdbuf[9] == 8'h12 && rdbuf[8] == 8'h34 &&
            rdbuf[11] == 8'h56 && rdbuf[10] == 8'h78) begin
            $display("    [PASS]"); ok = ok + 1;
        end else begin
            $display("    [FAIL]"); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[4] GET_DESCRIPTOR(Config, 34)");
        ctrl_read(8'h80, 8'h06, 16'h0200, 16'h0000, 16'd34);
        $display("    收到 %0d 字节, wTotalLength=%h%h, bInterfaceClass=%h",
                 rd_len, rdbuf[3], rdbuf[2], rdbuf[14]);
        if (rd_len == 34 && rdbuf[2] == 8'h22 && rdbuf[14] == 8'h03) begin
            $display("    [PASS]"); ok = ok + 1;
        end else begin
            $display("    [FAIL]"); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[5] GET_DESCRIPTOR(HID Report, 63)");
        ctrl_read(8'h81, 8'h06, 16'h2200, 16'h0000, 16'd63);
        $display("    收到 %0d 字节, 首字节=%h 末字节=%h", rd_len, rdbuf[0], rdbuf[62]);
        if (rd_len == 63 && rdbuf[0] == 8'h05 && rdbuf[62] == 8'hC0) begin
            $display("    [PASS]"); ok = ok + 1;
        end else begin
            $display("    [FAIL]"); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[6] SET_CONFIGURATION(1)");
        usb_setup(8'h00, 8'h09, 16'h0001, 16'h0000, 16'd0);
        in_wait_data(4'd0, p);
        #20000;
        if (dbg_configured === 1'b1) begin
            $display("    [PASS] configured=1"); ok = ok + 1;
        end else begin
            $display("    [FAIL] configured 仍为 0"); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[7] 轮询 EP1 IN（无按键）");
        do_in(4'd1, p);
        if (p === 4'hA) begin
            $display("    [PASS] NAK (pid=%h)", p); ok = ok + 1;
        end else begin
            $display("    [FAIL] 期望 NAK，实际 pid=%h", p); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[8] 按下 'A' 键 (key_in[29]，60%% 配列第 3 排)");
        key_in[29] = 1'b0;

        p = 4'hF; guard = 0;
        while ((p !== 4'h3 && p !== 4'hB) && guard < 60) begin
            #300000;                      // 每 300us 轮询一次
            do_in(4'd1, p);
            guard = guard + 1;
        end
        rep_mod = dpkt[0];
        rep_a   = dpkt[2];
        $display("    轮询 %0d 次后: pid=%h 长度=%0d 报告=[%h %h %h %h %h %h %h %h]",
                 guard, p, dlen, dpkt[7], dpkt[6], dpkt[5], dpkt[4],
                 dpkt[3], dpkt[2], dpkt[1], dpkt[0]);
        if ((p === 4'h3 || p === 4'hB) && dlen == 8 &&
            rep_a == 8'h04 && rep_mod == 8'h00) begin
            $display("    [PASS] 报告含 Usage 0x04 ('A')"); ok = ok + 1;
        end else begin
            $display("    [FAIL]"); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[9] 松开 'A'");
        key_in[29] = 1'b1;
        p = 4'hF; guard = 0;
        while ((p !== 4'h3 && p !== 4'hB) && guard < 60) begin
            #300000;
            do_in(4'd1, p);
            guard = guard + 1;
        end
        if ((p === 4'h3 || p === 4'hB) && dpkt[2] == 8'h00 && dpkt[0] == 8'h00) begin
            $display("    [PASS] 报告已清空"); ok = ok + 1;
        end else begin
            $display("    [FAIL] 报告未清空: %h %h", dpkt[2], dpkt[0]); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n[10] 按住 Fn + W（Fn 层应产出方向键 Up = 0x52）");
        key_in[60] = 1'b0;               // Fn（60% 配列最后一个键）
        key_in[16] = 1'b0;               // W

        p = 4'hF; guard = 0;
        while ((p !== 4'h3 && p !== 4'hB) && guard < 60) begin
            #300000;
            do_in(4'd1, p);
            guard = guard + 1;
        end
        $display("    轮询 %0d 次: pid=%h 修饰=%h 首键=%h", guard, p, dpkt[0], dpkt[2]);
        if ((p === 4'h3 || p === 4'hB) && dpkt[0] == 8'h00 && dpkt[2] == 8'h52) begin
            $display("    [PASS] Fn+W → Usage 0x52 (Up)，Fn 本身不上报"); ok = ok + 1;
        end else begin
            $display("    [FAIL] 期望 Usage 0x52，实际 %h", dpkt[2]); fail = fail + 1;
        end

        key_in[16] = 1'b1;               // 松开 W
        key_in[60] = 1'b1;               // 松开 Fn
        p = 4'hF; guard = 0;
        while ((p !== 4'h3 && p !== 4'hB) && guard < 60) begin
            #300000;
            do_in(4'd1, p);
            guard = guard + 1;
        end

        //----------------------------------------------------------
        $display("\n[11] 按下 F1（key_in[61]，TKL 功能键行首键）");
        key_in[61] = 1'b0;

        p = 4'hF; guard = 0;
        while ((p !== 4'h3 && p !== 4'hB) && guard < 60) begin
            #300000;
            do_in(4'd1, p);
            guard = guard + 1;
        end
        $display("    轮询 %0d 次: pid=%h 首键=%h", guard, p, dpkt[2]);
        if ((p === 4'h3 || p === 4'hB) && dpkt[2] == 8'h3A) begin
            $display("    [PASS] F1 → Usage 0x3A"); ok = ok + 1;
        end else begin
            $display("    [FAIL] 期望 Usage 0x3A，实际 %h", dpkt[2]); fail = fail + 1;
        end

        key_in[61] = 1'b1;               // 松开 F1
        p = 4'hF; guard = 0;
        while ((p !== 4'h3 && p !== 4'hB) && guard < 60) begin
            #300000;
            do_in(4'd1, p);
            guard = guard + 1;
        end

        //----------------------------------------------------------
        $display("\n[12] SET_REPORT(LED, caps=1)");
        usb_setup(8'h21, 8'h09, 16'h0200, 16'h0000, 16'd1);

        // OUT 数据阶段：设备可能还没准备好（NAK），主机重试
        p = 4'hF;
        guard = 0;
        while (p !== 4'h2 && guard < 20) begin
            send_token(8'hE1, cur_addr, 4'd0);
            hpkt[0] = 8'h4B;              // DATA1
            hpkt[1] = 8'h01;              // caps lock
            hpkt[2] = 8'h00;              // CRC16
            hlen    = 3;
            host_send;
            host_recv;
            p = dpid;
            guard = guard + 1;
            if (p !== 4'h2) #5000;
        end
        $display("    OUT 数据响应 pid=%h（重试 %0d 次）", p, guard);

        in_wait_data(4'd0, p);            // 状态阶段 IN ZLP
        #20000;
        $display("    led_caps=%b (低有效)", led_caps);
        if (led_caps === 1'b0) begin
            $display("    [PASS]"); ok = ok + 1;
        end else begin
            $display("    [FAIL]"); fail = fail + 1;
        end

        //----------------------------------------------------------
        $display("\n=========================================================");
        $display(" 结果: PASS=%0d  FAIL=%0d", ok, fail);
        if (fail == 0) $display(" >>> 全部测试通过 <<<");
        else           $display(" >>> 存在失败项 <<<");
        $display("=========================================================");
        $finish;
    end

    // 全局超时
    initial begin
        #200000000;
        $display("\n[ERROR] 仿真超时");
        $finish;
    end

endmodule
