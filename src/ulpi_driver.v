//=============================================================================
// 第 1 层：ulpi_driver.v  ——  ULPI PHY 驱动 IP（USB3300）
//------------------------------------------------------------------------------
//  职责（只做“字节搬运”，不关心 USB 语义）：
//    1. TX：STP + TX_CMD(=PID) → 逐字节输出 → 再拉一次 STP 表示包结束。
//    2. RX：DIR/NXT 时序。NXT=1 → 数据字节；NXT=0 → RXCMD（状态信息）。
//    3. 包边界：DIR 上升 = 包开始；DIR 下降 = EOP。
//    4. ULPI 寄存器写（用于上电配置 FUNC_CTRL，选择 FS/HS 收发器）。
//
//  为什么这一层不需要 CRC / 位填充 / NRZI：
//    USB3300 是 UTMI+ Level 3 的 PHY，SYNC、位填充、NRZI、CRC5/CRC16 与 EOP
//    全部由 PHY 内部完成；CRC 出错通过 RXCMD 的 RXERROR 上报。
//    TX_CMD 里必须带 PID，正是为了让 PHY 知道包类型（令牌/数据/握手），
//    从而自己决定要不要补 CRC5/CRC16。
//
//  ULPI TX_CMD 编码（USB3300 数据手册）：
//    [7:6]=00 & [3:0]=PID  → 发送一个 USB 包
//    [7:6]=10, [5:0]=addr  → 立即寄存器写
//    [7:6]=11, [5:0]=addr  → 立即寄存器读
//
//  ULPI RX_CMD 编码：
//    [1:0] Linestate (00=SE0 01=J 10=K 11=SE1)
//    [3:2] VBUS 状态 (11 = VbusValid)
//    [5:4] RxEvent   (00=空闲 01=RXACTIVE 11=RXACTIVE+RXERROR 10=HostDisconnect)
//    [6]   ID 引脚
//
//  时序要点（仿真已对齐）：
//    * STP 只在“命令拍”和“结束拍”各为高 1 拍，中间必有一拍为低；
//    * 数据字节组合直通到 DATA 总线，PHY 在 NXT 为高的那一拍采样。
//=============================================================================
`timescale 1ns/1ps

module ulpi_driver #(
    // 上电写入 FUNC_CTRL 的值：
    //   bit2   TermSelect  : 0 = FS 1.5k 上拉 / 1 = HS 45Ω 端接
    //   bit4:3 XcvrSelect  : 00 = HS, 01 = FS, 10 = LS
    //   bit5   SuspendM    : 1 = 允许 PHY 在总线空闲时挂起
    //   bit6   Reset       : 1 = PHY 软复位（自清零）
    // 8'h08 = XcvrSelect=FS + TermSelect=0 → 以全速设备身份枚举（推荐）
    parameter [7:0] FUNC_CTRL_INIT = 8'h08
)(
    input  wire        clk,          // ULPI CLKOUT, 60MHz
    input  wire        rst_n,

    // ---------------- ULPI 物理接口 ----------------
    output wire        ulpi_stp,
    input  wire        ulpi_dir,
    input  wire        ulpi_nxt,
    inout  wire [7:0]  ulpi_data,
    output wire        ulpi_rx_active,

    // ---------------- 发送（来自链路层） ----------------
    input  wire [3:0]  tx_pid,
    input  wire        tx_start,     // 单周期脉冲：启动一次发送
    input  wire [7:0]  tx_data,      // 当前字节（组合输出，随链路层索引变化）
    input  wire        tx_valid,     // 0 表示本包只有 PID（握手包 / 零长度包）
    input  wire        tx_last,      // 当前字节是本包最后一个
    output wire        tx_ready,     // 本拍已采样当前字节，链路层可推进索引
    output reg         tx_done,      // 包已交给 PHY（含 EOP）

    // ---------------- 接收（送往链路层） ----------------
    output reg  [7:0]  rx_data,
    output reg         rx_valid,     // 单周期：一个数据字节有效
    output reg         rx_eop,       // 单周期：DIR 下降，包结束
    output reg         rx_err,       // 本包出现 RXERROR
    output reg  [1:0]  rx_linestate,
    output reg         vbus_valid,
    output reg         host_disconnect,

    // ---------------- ULPI 寄存器写（上电初始化） ----------------
    input  wire        reg_wr_req,
    input  wire [5:0]  reg_wr_addr,
    input  wire [7:0]  reg_wr_data,
    output reg         reg_wr_done
);

    //==================================================================
    // 状态编码
    //==================================================================
    localparam [2:0] S_IDLE   = 3'd0,
                     S_TXCMD  = 3'd1,   // STP=1，总线=TX_CMD
                     S_TXDATA = 3'd2,   // 逐字节发送
                     S_TXCMD2 = 3'd3,   // 只有 PID 时的过渡拍（保证 STP 之间有空拍）
                     S_TXEND  = 3'd4,   // STP=1，总线=Idle，PHY 在此补 CRC+EOP
                     S_RCMD   = 3'd5,   // 寄存器写：命令拍
                     S_RDATA  = 3'd6,   // 寄存器写：数据拍
                     S_REND   = 3'd7;   // 寄存器写：结束拍

    reg [2:0]  st;
    reg [7:0]  tmo;                 // 防死等看门狗
    reg        oe;                  // 1 = 本层驱动总线
    reg        stp;
    reg [7:0]  txcmd_r;

    reg  [7:0] bus_out;
    always @(*) begin
        case (st)
            S_TXCMD:  bus_out = txcmd_r;     // STP=1 时总线必须是命令字节
            S_TXDATA: bus_out = tx_data;     // 组合直通
            S_RCMD:   bus_out = txcmd_r;
            S_RDATA:  bus_out = reg_wr_data;
            default:  bus_out = 8'h00;
        endcase
    end

    wire [7:0] din = ulpi_data;

    assign ulpi_data      = oe ? bus_out : 8'bz;
    assign ulpi_stp       = stp;
    assign ulpi_rx_active = ulpi_dir;

    // PHY 在 NXT 为高的那一拍采样数据，因此这就是“字节被取走”的时刻
    assign tx_ready = (st == S_TXDATA) && ulpi_nxt;

    //==================================================================
    // 发送 / 寄存器写状态机
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            stp         <= 1'b0;
            oe          <= 1'b0;
            txcmd_r     <= 8'h00;
            tx_done     <= 1'b0;
            reg_wr_done <= 1'b0;
            tmo         <= 8'd0;
        end else begin
            tx_done     <= 1'b0;
            reg_wr_done <= 1'b0;

            case (st)
            //----------------------------------------------------------
            S_IDLE: begin
                tmo <= 8'd0;
                if (reg_wr_req && !ulpi_dir) begin
                    txcmd_r <= {2'b10, reg_wr_addr};
                    oe      <= 1'b1;
                    stp     <= 1'b1;
                    st      <= S_RCMD;
                end else if (tx_start && !ulpi_dir) begin
                    txcmd_r <= {4'b0000, tx_pid};
                    oe      <= 1'b1;
                    stp     <= 1'b1;
                    st      <= S_TXCMD;
                end else begin
                    stp <= 1'b0;
                    oe  <= 1'b0;               // 释放总线 = ULPI Idle
                end
            end
            //----------------------------------------------------------
            S_TXCMD: begin
                stp <= 1'b0;
                st  <= tx_valid ? S_TXDATA : S_TXCMD2;
            end
            //----------------------------------------------------------
            S_TXCMD2: begin
                stp <= 1'b1;                   // 只有 PID 的包：结束拍
                st  <= S_TXEND;
            end
            //----------------------------------------------------------
            S_TXDATA: begin
                if (ulpi_nxt) begin
                    tmo <= 8'd0;
                    if (tx_last) begin
                        stp <= 1'b1;           // 最后一字节已被采样 → 结束拍
                        st  <= S_TXEND;
                    end
                end else if (tmo == 8'hFF) begin
                    stp <= 1'b1;
                    st  <= S_TXEND;            // PHY 无响应，强制收尾
                end else begin
                    tmo <= tmo + 8'd1;
                end
            end
            //----------------------------------------------------------
            S_TXEND: begin
                stp     <= 1'b0;
                oe      <= 1'b0;
                tx_done <= 1'b1;
                st      <= S_IDLE;
            end
            //----------------------------------------------------------
            S_RCMD: begin
                stp <= 1'b0;
                st  <= S_RDATA;
            end
            //----------------------------------------------------------
            S_RDATA: begin
                if (ulpi_nxt) begin
                    stp <= 1'b1;
                    st  <= S_REND;
                end else if (tmo == 8'hFF) begin
                    stp <= 1'b1;
                    st  <= S_REND;
                end else begin
                    tmo <= tmo + 8'd1;
                end
            end
            //----------------------------------------------------------
            S_REND: begin
                stp         <= 1'b0;
                oe          <= 1'b0;
                reg_wr_done <= 1'b1;
                st          <= S_IDLE;
            end
            //----------------------------------------------------------
            default: st <= S_IDLE;
            endcase
        end
    end

    //==================================================================
    // 接收：DIR/NXT → 字节流 + 包边界
    //==================================================================
    reg dir_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir_d           <= 1'b0;
            rx_data         <= 8'h00;
            rx_valid        <= 1'b0;
            rx_eop          <= 1'b0;
            rx_err          <= 1'b0;
            rx_linestate    <= 2'b01;      // FS 空闲 = J
            vbus_valid      <= 1'b0;
            host_disconnect <= 1'b0;
        end else begin
            dir_d    <= ulpi_dir;
            rx_valid <= 1'b0;
            rx_eop   <= 1'b0;

            if (ulpi_dir) begin
                if (ulpi_nxt) begin
                    // NXT=1：总线上是数据字节
                    rx_data  <= din;
                    rx_valid <= 1'b1;
                end else begin
                    // NXT=0：总线上是 RXCMD
                    rx_linestate    <= din[1:0];
                    vbus_valid      <= (din[3:2] == 2'b11);
                    host_disconnect <= (din[5:4] == 2'b10);
                    if (din[5:4] == 2'b11) rx_err <= 1'b1;   // RXERROR
                end
            end else if (!dir_d && !oe && (din != 8'h00)) begin
                // 总线空闲时也可能出现 RXCMD（linestate / VBUS 变化）
                rx_linestate    <= din[1:0];
                vbus_valid      <= (din[3:2] == 2'b11);
                host_disconnect <= (din[5:4] == 2'b10);
            end

            if (!dir_d && ulpi_dir) rx_err <= 1'b0;   // 新包开始，清错误
            if (dir_d && !ulpi_dir) rx_eop <= 1'b1;   // DIR 下降 = EOP
        end
    end

endmodule
