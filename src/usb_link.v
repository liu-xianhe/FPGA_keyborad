//=============================================================================
// 第 2 层：usb_link.v  ——  USB 2.0 链路层 / SIE
//------------------------------------------------------------------------------
//  职责（字节包 → USB 事务）：
//    1. 包解析：PID 分类 → 令牌(3 字节) / 数据(负载+CRC16) / 握手(1 字节)
//    2. 地址过滤：令牌 ADDR 必须等于 dev_addr，否则完全静默（不响应）
//    3. 事务应答（device 侧）：
//         SETUP/OUT → ACK（SETUP 无条件 ACK，符合规范）
//         IN        → 有数据发 DATA0/DATA1，无数据回 NAK，端点不存在回 STALL
//    4. DATA0/DATA1 由协议层给出（in_pid），链路层只负责原样发出
//    5. 发送后等主机 ACK，超时自动重发 3 次；3 次失败上报 xfer_timeout
//    6. 总线复位检测（SE0 > 2.5us）与 SOF 帧计数
//
//  关键时序：应答必须“预先生成”。
//    令牌第 2 字节到达时（地址/端点已知）就把 resp_kind/resp_pid 算好，
//    等 EOP（DIR 下降）后立即拉 STP 发送，避免在 EOP 之后才做判断。
//    全速(12Mbps)下设备应答窗口 ≈ 6.5 bit time = 542ns（32 个 60MHz 时钟），
//    余量充足；高速(480Mbps)只有 ~16 UI ≈ 33ns，必须靠这种预计算 + 时序收敛。
//=============================================================================
`timescale 1ns/1ps

module usb_link #(
    parameter integer CLK_FREQ = 60_000_000
)(
    input  wire        clk,
    input  wire        rst_n,

    //-------------------- 驱动层（ULPI）接口 --------------------
    input  wire [7:0]  phy_rx_data,
    input  wire        phy_rx_valid,
    input  wire        phy_rx_eop,
    input  wire        phy_rx_err,
    input  wire        phy_rx_active,   // = ULPI DIR，用来避免发送与接收撞车
    input  wire [1:0]  phy_line_state,

    output reg  [3:0]  phy_tx_pid,
    output reg         phy_tx_start,
    output wire [7:0]  phy_tx_data,
    output wire        phy_tx_valid,
    output wire        phy_tx_last,
    input  wire        phy_tx_ready,
    input  wire        phy_tx_done,

    //-------------------- 协议层接口 --------------------
    input  wire [6:0]  dev_addr,        // 当前设备地址
    input  wire [3:0]  in_ep,           // 已装载数据的端点号
    input  wire [6:0]  in_len,          // 已装载数据长度
    input  wire [3:0]  in_pid,          // DATA0 / DATA1
    input  wire        in_valid,        // 有数据待发
    input  wire        ep_stall,        // EP0 STALL 标志

    input  wire        txw_en,          // 协议层写 IN 缓冲
    input  wire [5:0]  txw_addr,
    input  wire [7:0]  txw_data,
    input  wire [5:0]  rxr_addr,        // 协议层读接收缓冲
    output wire [7:0]  rxr_data,

    output reg         rx_pkt_valid,    // 收到一个完整数据包（已 ACK），单周期
    output reg         rx_pkt_setup,    // 该数据包由 SETUP 令牌引导
    output reg  [3:0]  rx_pkt_dpid,     // DATA0 / DATA1
    output reg  [6:0]  rx_pkt_len,

    output reg         xfer_ack,        // 设备发出的 DATA 被主机正确 ACK
    output reg         xfer_timeout,    // 重发 3 次仍无 ACK

    input  wire        out_ready,       // 协议层可接收 OUT 数据
    output reg         bus_reset,       // 总线复位脉冲
    output reg  [10:0] frame_no         // SOF 帧计数（调试）
);

    //==================================================================
    // PID 常量（接收为带校验的完整字节，发送为 4bit 半字节）
    //==================================================================
    localparam [7:0] PIDB_OUT   = 8'hE1,
                     PIDB_IN    = 8'h69,
                     PIDB_SOF   = 8'hA5,
                     PIDB_SETUP = 8'h2D,
                     PIDB_DATA0 = 8'hC3,
                     PIDB_DATA1 = 8'h4B,
                     PIDB_ACK   = 8'hD2,
                     PIDB_NAK   = 8'h5A,
                     PIDB_STALL = 8'h1E;

    localparam [3:0] PID_DATA0 = 4'h3,
                     PID_DATA1 = 4'hB,
                     PID_ACK   = 4'h2,
                     PID_NAK   = 4'hA,
                     PID_STALL = 4'hE;

    // 应答种类
    localparam [1:0] RK_NONE = 2'd0,   // 不响应
                     RK_HS   = 2'd1,   // 发握手包（ACK/NAK/STALL）
                     RK_DATA = 2'd2;   // 发数据包

    //==================================================================
    // 包缓冲（各 64 字节，LUT-RAM）
    //==================================================================
    reg [7:0] tbuf [0:63];      // IN 端点数据（协议层写入）
    reg [7:0] rbuf [0:63];      // 接收到的 OUT/SETUP 数据

    assign rxr_data = rbuf[rxr_addr];

    always @(posedge clk) begin
        if (txw_en) tbuf[txw_addr] <= txw_data;
    end

    //==================================================================
    // 接收状态机
    //==================================================================
    localparam [1:0] RS_IDLE  = 2'd0,
                     RS_TOKEN = 2'd1,
                     RS_DATA  = 2'd2,
                     RS_OTHER = 2'd3;

    reg [1:0]  rs;
    reg [6:0]  bcnt;
    reg [7:0]  pid_b;
    reg [7:0]  hdr1;
    reg [7:0]  tok_pid;             // 本事务的令牌 PID
    reg [3:0]  tok_ep_r;
    reg        addr_ok;             // 令牌地址匹配
    reg [1:0]  resp_kind;
    reg [3:0]  resp_pid;
    reg        resp_wait_data;      // 应答要等数据包结束才发
    reg        resp_req;            // 单周期：请求发送应答（由 RX 段驱动）
    reg        resp_clr;            // 单周期：撤销应答（总线复位）

    // 令牌第 2 字节到达时用到的组合值
    wire [6:0] w_tok_addr = hdr1[6:0];
    wire [3:0] w_tok_ep   = {hdr1[7], phy_rx_data[4:0]};
    wire       w_addr_ok  = (w_tok_addr == dev_addr);
    wire       w_is_setup = (pid_b == PIDB_SETUP);
    wire       w_is_out   = (pid_b == PIDB_OUT);
    wire       w_is_in    = (pid_b == PIDB_IN);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rs             <= RS_IDLE;
            bcnt           <= 7'd0;
            pid_b          <= 8'h00;
            hdr1           <= 8'h00;
            tok_pid        <= 8'h00;
            tok_ep_r       <= 4'd0;
            addr_ok        <= 1'b0;
            resp_kind      <= RK_NONE;
            resp_pid       <= 4'd0;
            resp_wait_data <= 1'b0;
            resp_req       <= 1'b0;
            resp_clr       <= 1'b0;
            rx_pkt_valid   <= 1'b0;
            rx_pkt_setup   <= 1'b0;
            rx_pkt_dpid    <= 4'd0;
            rx_pkt_len     <= 7'd0;
            frame_no       <= 11'd0;
        end else begin
            rx_pkt_valid <= 1'b0;
            resp_req     <= 1'b0;
            resp_clr     <= 1'b0;

            // ---------------- 字节级解析 ----------------
            if (phy_rx_valid) begin
                case (rs)
                RS_IDLE: begin
                    pid_b <= phy_rx_data;
                    bcnt  <= 7'd0;
                    if (phy_rx_data == PIDB_SOF) begin
                        frame_no <= frame_no + 11'd1;
                        rs       <= RS_OTHER;
                    end else begin
                        case (phy_rx_data)
                            PIDB_SETUP, PIDB_OUT, PIDB_IN: rs <= RS_TOKEN;
                            PIDB_DATA0, PIDB_DATA1:        rs <= RS_DATA;
                            default:                       rs <= RS_OTHER;
                        endcase
                    end
                end

                RS_TOKEN: begin
                    if (bcnt == 7'd0) begin
                        hdr1 <= phy_rx_data;               // ADDR[6:0],EP[3]
                        bcnt <= 7'd1;
                    end else if (bcnt == 7'd1) begin
                        // 第 3 字节：ENDP[2:0] + CRC5
                        tok_pid  <= pid_b;
                        tok_ep_r <= w_tok_ep;
                        addr_ok  <= w_addr_ok;
                        bcnt     <= 7'd2;

                        // ---- 预先生成应答计划 ----
                        resp_wait_data <= w_is_setup | w_is_out;
                        if (!w_addr_ok) begin
                            resp_kind <= RK_NONE;          // 不是发给我的：静默
                        end else if (w_is_setup) begin
                            resp_kind <= RK_HS;            // SETUP 无条件 ACK
                            resp_pid  <= PID_ACK;
                        end else if (w_is_out) begin
                            if (w_tok_ep == 4'd0 && out_ready && !ep_stall) begin
                                resp_kind <= RK_HS;
                                resp_pid  <= PID_ACK;
                            end else if (w_tok_ep == 4'd0 && ep_stall) begin
                                resp_kind <= RK_HS;
                                resp_pid  <= PID_STALL;
                            end else if (w_tok_ep == 4'd0) begin
                                resp_kind <= RK_HS;
                                resp_pid  <= PID_NAK;
                            end else begin
                                resp_kind <= RK_HS;        // 端点不存在
                                resp_pid  <= PID_STALL;
                            end
                        end else begin // IN
                            if (in_valid && (in_ep == w_tok_ep) && !ep_stall) begin
                                resp_kind <= RK_DATA;
                                resp_pid  <= in_pid;
                            end else if (ep_stall && (w_tok_ep == 4'd0)) begin
                                resp_kind <= RK_HS;
                                resp_pid  <= PID_STALL;
                            end else if (w_tok_ep == 4'd0 || w_tok_ep == 4'd1) begin
                                resp_kind <= RK_HS;
                                resp_pid  <= PID_NAK;
                            end else begin
                                resp_kind <= RK_HS;
                                resp_pid  <= PID_STALL;
                            end
                        end
                    end
                end

                RS_DATA: begin
                    if (bcnt < 7'd64) rbuf[bcnt[5:0]] <= phy_rx_data;
                    bcnt <= bcnt + 7'd1;
                end

                default: ;
                endcase
            end

            // ---------------- 包结束 ----------------
            if (phy_rx_eop) begin
                if (rs == RS_DATA) begin
                    // 数据包：bcnt = 负载 + 2 字节 CRC16
                    rx_pkt_len   <= (bcnt >= 7'd2) ? (bcnt - 7'd2) : 7'd0;
                    rx_pkt_dpid  <= pid_b[3:0];
                    rx_pkt_setup <= (tok_pid == PIDB_SETUP);
                    // 只有“确实被本设备 ACK 的包”才交给协议层
                    if (addr_ok && !phy_rx_err && (tok_ep_r == 4'd0) && (bcnt >= 7'd2) &&
                        (resp_kind == RK_HS) && (resp_pid == PID_ACK))
                        rx_pkt_valid <= 1'b1;
                    // 收到数据后才回 ACK/NAK/STALL
                    resp_req <= (resp_kind != RK_NONE) && resp_wait_data;
                end else if ((rs == RS_TOKEN) && (bcnt == 7'd2)) begin
                    // 令牌包结束：IN 令牌需要立刻回数据
                    resp_req <= (resp_kind != RK_NONE) && !resp_wait_data;
                end
                rs <= RS_IDLE;
            end

            // 总线复位时丢弃一切待发应答
            if (bus_reset) begin
                resp_clr  <= 1'b1;
                resp_kind <= RK_NONE;
                rs        <= RS_IDLE;
            end
        end
    end

    //==================================================================
    // 发送状态机（应答/数据）
    //==================================================================
    localparam [2:0] TS_IDLE    = 3'd0,
                     TS_BODY    = 3'd1,
                     TS_TXDONE  = 3'd2,
                     TS_WAITACK = 3'd3;

    reg [2:0]  ts;
    reg [6:0]  tx_idx;
    reg [6:0]  tx_len;
    reg [2:0]  retry;
    reg [8:0]  wtmo;
    reg [1:0]  kind_saved;
    reg        pend;                // 有待发送的应答（仅本段驱动）

    assign phy_tx_data  = tbuf[tx_idx[5:0]];
    assign phy_tx_valid = (ts == TS_BODY) && (tx_len != 7'd0) && (tx_idx < tx_len);
    assign phy_tx_last  = (tx_len != 7'd0) && (tx_idx == (tx_len - 7'd1));

    // 本次要发送的负载长度（握手包为 0，即只有 PID）
    wire [6:0] tx_len_c = (resp_kind == RK_DATA)
                          ? ((in_len > 7'd64) ? 7'd64 : in_len)
                          : 7'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ts           <= TS_IDLE;
            pend         <= 1'b0;
            phy_tx_start <= 1'b0;
            phy_tx_pid   <= 4'd0;
            tx_idx       <= 7'd0;
            tx_len       <= 7'd0;
            retry        <= 3'd0;
            wtmo         <= 9'd0;
            kind_saved   <= RK_NONE;
            xfer_ack     <= 1'b0;
            xfer_timeout <= 1'b0;
        end else begin
            xfer_ack     <= 1'b0;
            xfer_timeout <= 1'b0;

            case (ts)
            //----------------------------------------------------------
            TS_IDLE: begin
                phy_tx_start <= 1'b0;
                if (resp_clr) begin
                    pend <= 1'b0;
                end else if (resp_req) begin
                    pend <= 1'b1;          // 下一拍开始发送
                end else if (pend) begin
                    if (phy_rx_err) begin
                        pend <= 1'b0;      // 收到的包有错，不应答
                    end else if (phy_rx_active) begin
                        ;                  // 总线上还有包在收，等 DIR 释放
                    end else begin
                        phy_tx_pid   <= resp_pid;
                        phy_tx_start <= 1'b1;
                        tx_len       <= tx_len_c;
                        tx_idx       <= 7'd0;
                        kind_saved   <= resp_kind;
                        wtmo         <= 9'd0;
                        pend         <= 1'b0;
                        // 零长度包（握手/ZLP）没有数据相，直接等 tx_done
                        ts           <= (tx_len_c == 7'd0) ? TS_TXDONE : TS_BODY;
                    end
                end
            end
            //----------------------------------------------------------
            TS_BODY: begin
                phy_tx_start <= 1'b0;
                if (phy_tx_ready) begin
                    wtmo    <= 9'd0;
                    tx_idx  <= tx_idx + 7'd1;
                    if ((tx_idx + 7'd1) >= tx_len) ts <= TS_TXDONE;
                end else if (wtmo == 9'd255) begin
                    wtmo <= 9'd0;
                    ts   <= TS_TXDONE;           // 驱动层异常，收尾避免死锁
                end else begin
                    wtmo <= wtmo + 9'd1;
                end
            end
            //----------------------------------------------------------
            TS_TXDONE: begin
                phy_tx_start <= 1'b0;          // 启动脉冲只保留一拍
                if (phy_tx_done) begin
                    pend <= 1'b0;
                    if (kind_saved == RK_DATA) begin
                        retry <= 3'd0;
                        wtmo  <= 9'd0;
                        ts    <= TS_WAITACK;
                    end else begin
                        ts    <= TS_IDLE;   // 握手包无需等应答
                    end
                end
            end
            //----------------------------------------------------------
            TS_WAITACK: begin
                if (phy_rx_valid && (phy_rx_data == PIDB_ACK)) begin
                    xfer_ack <= 1'b1;
                    ts       <= TS_IDLE;
                end else if (phy_rx_valid && (phy_rx_data == PIDB_NAK)) begin
                    ts       <= TS_IDLE;              // 主机 NAK：放弃本次
                end else if (wtmo == 9'd400) begin   // ≈6.7us 超时
                    wtmo <= 9'd0;
                    if (retry < 3'd2) begin
                        retry <= retry + 3'd1;
                        pend  <= 1'b1;                // 用同样的 PID/数据重发
                        ts    <= TS_IDLE;
                    end else begin
                        xfer_timeout <= 1'b1;
                        ts           <= TS_IDLE;
                    end
                end else begin
                    wtmo <= wtmo + 9'd1;
                end
            end
            //----------------------------------------------------------
            default: ts <= TS_IDLE;
            endcase
        end
    end

    //==================================================================
    // 总线复位检测：SE0 持续 2.5us
    //==================================================================
    reg [9:0] se0_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            se0_cnt   <= 10'd0;
            bus_reset <= 1'b0;
        end else begin
            bus_reset <= 1'b0;
            if (phy_line_state == 2'b00) begin
                if (se0_cnt == 10'd149) begin
                    bus_reset <= 1'b1;      // 2.5us @60MHz
                    se0_cnt   <= se0_cnt;   // 保持，避免重复触发
                end else begin
                    se0_cnt <= se0_cnt + 10'd1;
                end
            end else begin
                se0_cnt <= 10'd0;
            end
        end
    end

endmodule
