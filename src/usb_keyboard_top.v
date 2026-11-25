//=============================================================================
// usb_keyboard_top.v  ——  USB 2.0 全速键盘（USB3300 / ULPI）顶层
//------------------------------------------------------------------------------
//  分层结构：
//
//     ┌───────────────────────────────────────────────────┐
//     │  应用层  kbd_gpio (IO 直连/逐键消抖)  kbd_hid (报告) │
//     ├───────────────────────────────────────────────────┤
//     │  协议层  usb_proto  (EP0 控制传输 / 描述符 / HID 类) │
//     ├───────────────────────────────────────────────────┤
//     │  链路层  usb_link   (PID/令牌/应答/DATA 翻转/重发)   │
//     ├───────────────────────────────────────────────────┤
//     │  驱动层  ulpi_driver(ULPI 字节时序 / STP/NXT/DIR)   │
//     └───────────────────────────────────────────────────┘
//                              ↕ ULPI 8bit @60MHz
//                          USB3300 (HS/FS PHY)
//
//  时钟：直接用 USB3300 的 CLKOUT(60MHz)，全设计单时钟域，无需 PLL/CDC。
//  复位：上电 → PHY_RESETB 拉低 10ms → 释放 → 2ms 后写 FUNC_CTRL 选择
//        收发器速度 → 再等 1ms 释放内核复位，开始枚举。
//  按键：每键一根 IO 直连（无矩阵、无扫描），键数由 KEY_NUM 决定。
//        默认 87 键 = TKL（Tenkeyless）ANSI 配列。
//        目标器件 GW5A-LV25LQ144NES（144 脚 LQFP）可用 IO 约 105~110 根，
//        扣掉 USB3300 的 11 根 + 时钟/复位/PHY_RSTB + 3 个 LED + 调试脚，
//        87 键刚好放得下；若 pinout 核对后发现差几根，见 .cst 头部的裁剪说明。
//=============================================================================
`timescale 1ns/1ps

module usb_keyboard_top #(
    parameter integer CLK_FREQ       = 60_000_000,
    parameter [7:0]   FUNC_CTRL_INIT = 8'h08,   // 0x08=全速(FS)；0x00=高速(需 chirp)
    parameter         LED_ACTIVE_LOW = 1,
    parameter integer PHY_RST_MS     = 10,      // RESETB 保持低电平时间
    parameter integer INIT_MS        = 2,       // PHY 稳定后到写寄存器的等待
    parameter integer KEY_NUM        = 87,      // 按键数 = key_in 的位宽
    parameter integer FN_KEY_IDX     = 60       // Fn 键的序号（主键区底排最后一个）
)(
    //---------------- 时钟与复位 ----------------
    input  wire        clk_ulpi,      // USB3300 CLKOUT，60MHz
    input  wire        rst_btn_n,     // 复位按键/上电复位，低有效
    output wire        phy_reset_n,   // 接 USB3300 RESETB

    //---------------- ULPI ----------------
    output wire        ulpi_stp,
    input  wire        ulpi_dir,
    input  wire        ulpi_nxt,
    inout  wire [7:0]  ulpi_data,

    //---------------- 键盘（每键一根 IO，内部上拉，按下为低） ----------------
    input  wire [KEY_NUM-1:0] key_in,  // 1 = 松开，0 = 按下

    //---------------- 指示灯 ----------------
    output wire        led_caps,
    output wire        led_num,
    output wire        led_scroll,

    //---------------- 调试 ----------------
    output wire        dbg_configured
);

    //==================================================================
    // 层间信号（先声明，后使用）
    //==================================================================
    // 驱动层 ↔ 链路层
    wire [7:0]  phy_rx_data;
    wire        phy_rx_valid, phy_rx_eop, phy_rx_err;
    wire [1:0]  phy_line_state;
    wire [3:0]  phy_tx_pid;
    wire        phy_tx_start, phy_tx_valid, phy_tx_last, phy_tx_ready, phy_tx_done;
    wire [7:0]  phy_tx_data;
    wire        reg_wr_done;

    // 链路层 ↔ 协议层
    wire        rx_pkt_valid, rx_pkt_setup;
    wire [3:0]  rx_pkt_dpid;
    wire [6:0]  rx_pkt_len;
    wire [5:0]  rx_raddr, txw_addr;
    wire [7:0]  rx_rdata, txw_data;
    wire        txw_en;
    wire [3:0]  in_ep, in_pid;
    wire [6:0]  in_len;
    wire        in_valid, xfer_ack, xfer_timeout;
    wire [6:0]  dev_addr;
    wire        ep0_stall, out_ready, bus_reset, configured;
    wire [10:0] frame_no;

    // 应用层
    wire [KEY_NUM-1:0] key_state;
    wire         keys_changed;
    wire [63:0]  hid_rep;
    wire [7:0]   hid_seq;
    wire [7:0]   led_rep;

    //==================================================================
    // 复位同步 + 上电初始化时序
    //==================================================================
    reg [1:0] rst_sync;

    always @(posedge clk_ulpi or negedge rst_btn_n) begin
        if (!rst_btn_n) rst_sync <= 2'b00;
        else            rst_sync <= {rst_sync[0], 1'b1};
    end

    wire arst_n = rst_sync[1];

    // 1ms 节拍
    localparam integer MS_DIV = CLK_FREQ / 1000;

    reg [15:0] ms_div;
    reg        ms_tick;

    always @(posedge clk_ulpi or negedge arst_n) begin
        if (!arst_n) begin
            ms_div  <= 16'd0;
            ms_tick <= 1'b0;
        end else if (ms_div == (MS_DIV - 1)) begin
            ms_div  <= 16'd0;
            ms_tick <= 1'b1;
        end else begin
            ms_div  <= ms_div + 16'd1;
            ms_tick <= 1'b0;
        end
    end

    localparam [2:0] I_PHYRST = 3'd0,   // 保持 PHY 复位
                     I_WAIT   = 3'd1,   // 等 PHY 稳定
                     I_WR     = 3'd2,   // 写 FUNC_CTRL
                     I_READY  = 3'd3,   // 等内核稳定
                     I_DONE   = 3'd4;

    reg [2:0]  ist;
    reg [10:0] ms_cnt;
    reg        phy_rst_n_r;
    reg        core_rst_n;

    always @(posedge clk_ulpi or negedge arst_n) begin
        if (!arst_n) begin
            ist         <= I_PHYRST;
            ms_cnt      <= 11'd0;
            phy_rst_n_r <= 1'b0;
            core_rst_n  <= 1'b0;
        end else begin
            case (ist)
            I_PHYRST: begin
                phy_rst_n_r <= 1'b0;
                if (ms_tick) begin
                    if (ms_cnt == (PHY_RST_MS - 1)) begin   // RESETB 保持低
                        ms_cnt      <= 11'd0;
                        phy_rst_n_r <= 1'b1;
                        ist         <= I_WAIT;
                    end else ms_cnt <= ms_cnt + 11'd1;
                end
            end
            I_WAIT: if (ms_tick) begin
                if (ms_cnt == (INIT_MS - 1)) begin
                    ms_cnt <= 11'd0;
                    ist    <= I_WR;
                end else ms_cnt <= ms_cnt + 11'd1;
            end
            I_WR: if (reg_wr_done) begin
                ms_cnt <= 11'd0;
                ist    <= I_READY;
            end
            I_READY: if (ms_tick) begin
                if (ms_cnt == 11'd1) begin
                    ms_cnt     <= 11'd0;
                    core_rst_n <= 1'b1;
                    ist        <= I_DONE;
                end else ms_cnt <= ms_cnt + 11'd1;
            end
            default: ;
            endcase
        end
    end

    assign phy_reset_n = phy_rst_n_r;

    //==================================================================
    // 第 1 层：ULPI 驱动
    //==================================================================
    ulpi_driver #(
        .FUNC_CTRL_INIT (FUNC_CTRL_INIT)
    ) u_ulpi (
        .clk             (clk_ulpi),
        .rst_n           (arst_n),
        .ulpi_stp        (ulpi_stp),
        .ulpi_dir        (ulpi_dir),
        .ulpi_nxt        (ulpi_nxt),
        .ulpi_data       (ulpi_data),
        .ulpi_rx_active  (),
        .tx_pid          (phy_tx_pid),
        .tx_start        (phy_tx_start),
        .tx_data         (phy_tx_data),
        .tx_valid        (phy_tx_valid),
        .tx_last         (phy_tx_last),
        .tx_ready        (phy_tx_ready),
        .tx_done         (phy_tx_done),
        .rx_data         (phy_rx_data),
        .rx_valid        (phy_rx_valid),
        .rx_eop          (phy_rx_eop),
        .rx_err          (phy_rx_err),
        .rx_linestate    (phy_line_state),
        .vbus_valid      (),
        .host_disconnect (),
        .reg_wr_req      (ist == I_WR),
        .reg_wr_addr     (6'h04),            // FUNC_CTRL
        .reg_wr_data     (FUNC_CTRL_INIT),
        .reg_wr_done     (reg_wr_done)
    );

    //==================================================================
    // 第 2 层：链路层
    //==================================================================
    usb_link #(
        .CLK_FREQ (CLK_FREQ)
    ) u_link (
        .clk            (clk_ulpi),
        .rst_n          (core_rst_n),
        .phy_rx_data    (phy_rx_data),
        .phy_rx_valid   (phy_rx_valid),
        .phy_rx_eop     (phy_rx_eop),
        .phy_rx_err     (phy_rx_err),
        .phy_rx_active  (ulpi_dir),
        .phy_line_state (phy_line_state),
        .phy_tx_pid     (phy_tx_pid),
        .phy_tx_start   (phy_tx_start),
        .phy_tx_data    (phy_tx_data),
        .phy_tx_valid   (phy_tx_valid),
        .phy_tx_last    (phy_tx_last),
        .phy_tx_ready   (phy_tx_ready),
        .phy_tx_done    (phy_tx_done),
        .dev_addr       (dev_addr),
        .in_ep          (in_ep),
        .in_len         (in_len),
        .in_pid         (in_pid),
        .in_valid       (in_valid),
        .ep_stall       (ep0_stall),
        .txw_en         (txw_en),
        .txw_addr       (txw_addr),
        .txw_data       (txw_data),
        .rxr_addr       (rx_raddr),
        .rxr_data       (rx_rdata),
        .rx_pkt_valid   (rx_pkt_valid),
        .rx_pkt_setup   (rx_pkt_setup),
        .rx_pkt_dpid    (rx_pkt_dpid),
        .rx_pkt_len     (rx_pkt_len),
        .xfer_ack       (xfer_ack),
        .xfer_timeout   (xfer_timeout),
        .out_ready      (out_ready),
        .bus_reset      (bus_reset),
        .frame_no       (frame_no)
    );

    //==================================================================
    // 第 3 层：协议层
    //==================================================================
    usb_proto u_proto (
        .clk          (clk_ulpi),
        .rst_n        (core_rst_n),
        .rx_pkt_valid (rx_pkt_valid),
        .rx_pkt_setup (rx_pkt_setup),
        .rx_pkt_dpid  (rx_pkt_dpid),
        .rx_pkt_len   (rx_pkt_len),
        .rx_raddr     (rx_raddr),
        .rx_rdata     (rx_rdata),
        .txw_en       (txw_en),
        .txw_addr     (txw_addr),
        .txw_data     (txw_data),
        .in_ep        (in_ep),
        .in_len       (in_len),
        .in_pid       (in_pid),
        .in_valid     (in_valid),
        .xfer_ack     (xfer_ack),
        .xfer_timeout (xfer_timeout),
        .dev_addr     (dev_addr),
        .ep0_stall    (ep0_stall),
        .out_ready    (out_ready),
        .bus_reset    (bus_reset),
        .hid_rep      (hid_rep),
        .hid_seq      (hid_seq),
        .led_rep      (led_rep),
        .configured   (configured)
    );

    //==================================================================
    // 第 4 层：应用层
    //==================================================================
    kbd_gpio #(
        .KEY_NUM    (KEY_NUM),
        .CLK_FREQ   (CLK_FREQ),
        .SAMPLE_US  (1000),        // 1ms 采样
        .DB_SAMPLES (4),           // 连续 4 次一致 → 4ms 消抖
        .ACTIVE_LOW (1)            // 按下为低
    ) u_gpio (
        .clk       (clk_ulpi),
        .rst_n     (core_rst_n),
        .key_in    (key_in),
        .key_state (key_state),
        .changed   (keys_changed)
    );

    kbd_hid #(
        .KEY_NUM  (KEY_NUM),
        .FN_INDEX (FN_KEY_IDX)     // Fn 在主键区底排最后一个位置
    ) u_hid (
        .clk        (clk_ulpi),
        .rst_n      (core_rst_n),
        .key_state  (key_state),
        .changed    (keys_changed),
        .report     (hid_rep),
        .report_seq (hid_seq)
    );

    //==================================================================
    // 指示灯
    //==================================================================
    assign led_caps   = LED_ACTIVE_LOW ? ~led_rep[0] : led_rep[0];
    assign led_num    = LED_ACTIVE_LOW ? ~led_rep[1] : led_rep[1];
    assign led_scroll = LED_ACTIVE_LOW ? ~led_rep[2] : led_rep[2];

    assign dbg_configured = configured;

endmodule
