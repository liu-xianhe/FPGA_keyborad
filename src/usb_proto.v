//=============================================================================
// 第 3 层：usb_proto.v  ——  USB 设备协议层（EP0 控制传输 + HID 类请求）
//------------------------------------------------------------------------------
//  职责：
//    1. 解析 SETUP 包（8 字节），实现 USB 2.0 第 9 章标准请求
//       GET_DESCRIPTOR / SET_ADDRESS / SET_CONFIGURATION / GET_STATUS /
//       GET_CONFIGURATION / SET_IDLE / SET_INTERFACE / CLEAR_FEATURE
//    2. HID 类请求 SET_REPORT（主机下发 LED 状态）
//    3. 控制传输三阶段状态机（SETUP → DATA/STATUS → STATUS），
//       处理 DATA0/DATA1 翻转、ZLP、短包、STALL
//    4. 中断端点 EP1 IN：按“报告序号”把应用层新报告搬进链路缓冲
//
//  与链路层的约定：
//    * rx_pkt_valid 出现时，链路层已经把该数据包 ACK 掉了，数据在 rbuf 中，
//      协议层用 rx_raddr/rx_rdata 异步读取。
//    * in_valid=1 表示“链路缓冲里有一包待发的 IN 数据”，in_ep 指出属于哪个端点；
//      IN 令牌到来时链路层自行取用，若端点不符则回 NAK。
//    * xfer_ack 表示设备发出的 DATA 被主机正确 ACK；xfer_timeout 表示重试耗尽。
//=============================================================================
`timescale 1ns/1ps

module usb_proto (
    input  wire        clk,
    input  wire        rst_n,

    //-------------------- 链路层接口 --------------------
    input  wire        rx_pkt_valid,
    input  wire        rx_pkt_setup,
    input  wire [3:0]  rx_pkt_dpid,
    input  wire [6:0]  rx_pkt_len,
    output reg  [5:0]  rx_raddr,
    input  wire [7:0]  rx_rdata,

    output reg         txw_en,
    output reg  [5:0]  txw_addr,
    output reg  [7:0]  txw_data,

    output reg  [3:0]  in_ep,
    output reg  [6:0]  in_len,
    output reg  [3:0]  in_pid,
    output reg         in_valid,
    input  wire        xfer_ack,
    input  wire        xfer_timeout,

    output reg  [6:0]  dev_addr,
    output reg         ep0_stall,
    output reg         out_ready,
    input  wire        bus_reset,

    //-------------------- 应用层接口 --------------------
    input  wire [63:0] hid_rep,      // 8 字节 HID 报告
    input  wire [7:0]  hid_seq,      // 报告版本号（变化则 +1）
    output reg  [7:0]  led_rep,      // bit0 Caps / bit1 Num / bit2 Scroll
    output reg         configured
);

    //==================================================================
    // 常量
    //==================================================================
    localparam [3:0] PID_DATA0 = 4'h3,
                     PID_DATA1 = 4'hB;

    localparam [1:0] ACT_IN   = 2'd0,   // 需要发数据（描述符/状态字）
                     ACT_SIN  = 2'd1,   // 只有状态阶段，方向 IN
                     ACT_DOUT = 2'd2;   // 控制写：先收 OUT 数据

    localparam [2:0] SEL_DEV  = 3'd0,
                     SEL_CFG  = 3'd1,
                     SEL_REP  = 3'd2,
                     SEL_HID  = 3'd3,
                     SEL_STR  = 3'd4,
                     SEL_ZERO = 3'd5,
                     SEL_CFG1 = 3'd6;

    //==================================================================
    // 主状态机状态
    //==================================================================
    localparam [3:0] C_IDLE   = 4'd0,
                     C_PARSE  = 4'd1,
                     C_DECODE = 4'd2,
                     C_LOAD   = 4'd3,
                     C_DIN    = 4'd4,   // 数据阶段（IN）
                     C_SOUT   = 4'd5,   // 状态阶段（OUT ZLP）
                     C_DOUT   = 4'd6,   // 数据阶段（OUT），等主机数据
                     C_DOUT2  = 4'd7,   // 取回 OUT 数据
                     C_SIN    = 4'd8,   // 状态阶段（IN ZLP）
                     C_EPLOAD = 4'd9,   // 装载 EP1 报告
                     C_EPSEND = 4'd10;  // 等 EP1 传输完成

    reg [3:0]  st;
    reg [63:0] setup_latch;
    reg [3:0]  p_cnt;
    reg [6:0]  ld_idx;
    reg [6:0]  ld_len;
    reg [2:0]  d_sel;
    reg [6:0]  pend_addr;
    reg        pend_cfg;
    reg        act_setaddr;
    reg        act_setcfg;
    reg        ep0_tgl_in;
    reg        ep0_tgl_out;
    reg        ep1_tgl;
    reg [7:0]  sent_seq;
    reg [7:0]  cap_seq;

    //==================================================================
    // SETUP 字段拆解
    //  setup_latch[63:56]=byte0 ... [7:0]=byte7
    //==================================================================
    wire [7:0]  bmrt = setup_latch[63:56];
    wire [7:0]  breq = setup_latch[55:48];
    wire [15:0] wval = {setup_latch[39:32], setup_latch[47:40]};
    wire [15:0] widx = {setup_latch[23:16], setup_latch[31:24]};
    wire [15:0] wlen = {setup_latch[7:0],   setup_latch[15:8]};

    //==================================================================
    // 请求译码（组合）
    //==================================================================
    reg [1:0] dec_act;
    reg [2:0] dec_sel;
    reg [6:0] dec_len;
    reg [6:0] dec_addr;
    reg       dec_cfg;
    reg       dec_setaddr;
    reg       dec_setcfg;
    reg       dec_stall;

    always @(*) begin
        dec_act     = ACT_SIN;
        dec_sel     = SEL_ZERO;
        dec_len     = 7'd0;
        dec_addr    = dev_addr;
        dec_cfg     = 1'b0;
        dec_setaddr = 1'b0;
        dec_setcfg  = 1'b0;
        dec_stall   = 1'b0;

        case (breq)
        //--------------------------------------------------- GET_DESCRIPTOR
        8'h06: begin
            case (wval[15:8])
                8'h01: begin dec_sel = SEL_DEV; dec_len = (wlen < 16'd18) ? wlen[6:0] : 7'd18; end
                8'h02: begin dec_sel = SEL_CFG; dec_len = (wlen < 16'd34) ? wlen[6:0] : 7'd34; end
                8'h03: begin dec_sel = SEL_STR; dec_len = (wlen < 16'd4)  ? wlen[6:0] : 7'd4;  end
                8'h21: begin dec_sel = SEL_HID; dec_len = (wlen < 16'd9)  ? wlen[6:0] : 7'd9;  end
                8'h22: begin dec_sel = SEL_REP; dec_len = (wlen < 16'd63) ? wlen[6:0] : 7'd63; end
                default: dec_stall = 1'b1;
            endcase
            dec_act = (dec_len == 7'd0) ? ACT_SIN : ACT_IN;
        end
        //--------------------------------------------------- SET_ADDRESS
        8'h05: begin
            dec_act     = ACT_SIN;
            dec_setaddr = 1'b1;
            dec_addr    = wval[6:0];
        end
        //--------------------------------------------------- SET_CONFIGURATION
        8'h09: begin
            if (bmrt[6:5] == 2'b01) begin            // 类请求 = HID SET_REPORT
                if (wlen == 16'd1) dec_act = ACT_DOUT;
                else               dec_stall = 1'b1;
            end else begin
                dec_act    = ACT_SIN;
                dec_setcfg = 1'b1;
                dec_cfg    = (wval[7:0] != 8'h00);
            end
        end
        //--------------------------------------------------- SET_IDLE / SET_INTERFACE
        8'h0A, 8'h0B: dec_act = ACT_SIN;
        //--------------------------------------------------- GET_STATUS
        8'h00: begin
            dec_sel = SEL_ZERO;
            dec_len = (wlen < 16'd2) ? wlen[6:0] : 7'd2;
            dec_act = (dec_len == 7'd0) ? ACT_SIN : ACT_IN;
        end
        //--------------------------------------------------- GET_CONFIGURATION
        8'h08: begin
            dec_sel = SEL_CFG1;
            dec_len = (wlen < 16'd1) ? wlen[6:0] : 7'd1;
            dec_act = (dec_len == 7'd0) ? ACT_SIN : ACT_IN;
        end
        //--------------------------------------------------- CLEAR/SET_FEATURE
        8'h01, 8'h03: dec_act = ACT_SIN;
        //--------------------------------------------------- 其它：STALL
        default: dec_stall = 1'b1;
        endcase
    end

    //==================================================================
    // 描述符 ROM
    //==================================================================
    wire [7:0] rom_dout;

    usb_desc_rom u_rom (
        .sel     (d_sel),
        .addr    (ld_idx),
        .cfg_val (configured),
        .dout    (rom_dout)
    );

    //==================================================================
    // out_ready：只有真正准备接收 OUT 数据时才置 1
    // （SETUP 包由链路层无条件 ACK，不受此信号影响）
    //==================================================================
    always @(*) begin
        case (st)
            C_IDLE, C_SOUT, C_DOUT: out_ready = 1'b1;
            default:                out_ready = 1'b0;
        endcase
    end

    // EP1 报告按字节取出（case 选择，避免变基址部分选择）
    reg [7:0] rep_byte;
    always @(*) begin
        case (ld_idx[2:0])
            3'd0: rep_byte = hid_rep[7:0];
            3'd1: rep_byte = hid_rep[15:8];
            3'd2: rep_byte = hid_rep[23:16];
            3'd3: rep_byte = hid_rep[31:24];
            3'd4: rep_byte = hid_rep[39:32];
            3'd5: rep_byte = hid_rep[47:40];
            3'd6: rep_byte = hid_rep[55:48];
            default: rep_byte = hid_rep[63:56];
        endcase
    end

    //==================================================================
    // 主状态机
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st           <= C_IDLE;
            setup_latch  <= 64'd0;
            p_cnt        <= 4'd0;
            ld_idx       <= 7'd0;
            ld_len       <= 7'd0;
            d_sel        <= SEL_ZERO;
            rx_raddr     <= 6'd0;
            txw_en       <= 1'b0;
            txw_addr     <= 6'd0;
            txw_data     <= 8'h00;
            in_ep        <= 4'd0;
            in_len       <= 7'd0;
            in_pid       <= PID_DATA1;
            in_valid     <= 1'b0;
            dev_addr     <= 7'd0;
            ep0_stall    <= 1'b0;
            ep0_tgl_in   <= 1'b0;
            ep0_tgl_out  <= 1'b0;
            ep1_tgl      <= 1'b0;
            sent_seq     <= 8'd0;
            cap_seq      <= 8'd0;
            led_rep      <= 8'h00;
            configured   <= 1'b0;
            pend_addr    <= 7'd0;
            pend_cfg     <= 1'b0;
            act_setaddr  <= 1'b0;
            act_setcfg   <= 1'b0;
        end else begin
            txw_en <= 1'b0;      // 默认不写缓冲

            //----------------------------------------------------------
            // 总线复位
            //----------------------------------------------------------
            if (bus_reset) begin
                dev_addr    <= 7'd0;
                configured  <= 1'b0;
                ep0_stall   <= 1'b0;
                in_valid    <= 1'b0;
                ep0_tgl_in  <= 1'b0;
                ep0_tgl_out <= 1'b0;
                ep1_tgl     <= 1'b0;
                st          <= C_IDLE;
            end
            //----------------------------------------------------------
            // 任何状态下收到 SETUP：重新开始一次控制传输
            //----------------------------------------------------------
            else if (rx_pkt_valid && rx_pkt_setup) begin
                in_valid    <= 1'b0;
                setup_latch <= 64'd0;
                rx_raddr    <= 6'd0;
                p_cnt       <= 4'd0;
                ep0_stall   <= 1'b0;
                st          <= C_PARSE;
            end
            //----------------------------------------------------------
            else begin
                case (st)
                //------------------------------------------------------
                C_IDLE: begin
                    // 有新的键盘报告且已配置 → 装进 EP1
                    if (configured && (hid_seq != sent_seq)) begin
                        ld_idx  <= 7'd0;
                        cap_seq <= hid_seq;
                        st      <= C_EPLOAD;
                    end
                end
                //------------------------------------------------------
                C_PARSE: begin
                    rx_raddr    <= p_cnt + 6'd1;
                    setup_latch <= {setup_latch[55:0], rx_rdata};
                    p_cnt       <= p_cnt + 4'd1;
                    if (p_cnt == 4'd7) st <= C_DECODE;
                end
                //------------------------------------------------------
                C_DECODE: begin
                    if (dec_stall) begin
                        ep0_stall <= 1'b1;          // 不支持的请求 → STALL
                        st        <= C_IDLE;
                    end else begin
                        d_sel        <= dec_sel;
                        ld_len       <= dec_len;
                        ld_idx       <= 7'd0;
                        pend_addr    <= dec_addr;
                        pend_cfg     <= dec_cfg;
                        act_setaddr  <= dec_setaddr;
                        act_setcfg   <= dec_setcfg;
                        case (dec_act)
                        ACT_IN: begin
                            if (dec_len == 7'd0) begin
                                in_len   <= 7'd0;
                                in_pid   <= PID_DATA1;
                                in_ep    <= 4'd0;
                                in_valid <= 1'b1;
                                st       <= C_SIN;
                            end else begin
                                st <= C_LOAD;
                            end
                        end
                        ACT_SIN: begin
                            // 无数据阶段，直接 IN 状态阶段（DATA1 ZLP）
                            in_len   <= 7'd0;
                            in_pid   <= PID_DATA1;
                            in_ep    <= 4'd0;
                            in_valid <= 1'b1;
                            st       <= C_SIN;
                        end
                        default: st <= C_DOUT;      // 控制写
                        endcase
                    end
                end
                //------------------------------------------------------
                C_LOAD: begin
                    txw_en   <= 1'b1;
                    txw_addr <= ld_idx[5:0];
                    txw_data <= rom_dout;
                    ld_idx   <= ld_idx + 7'd1;
                    if ((ld_idx + 7'd1) >= ld_len) begin
                        in_len     <= ld_len;
                        in_pid     <= PID_DATA1;    // SETUP 之后第一包用 DATA1
                        in_ep      <= 4'd0;
                        in_valid   <= 1'b1;
                        st         <= C_DIN;
                    end
                end
                //------------------------------------------------------
                C_DIN: begin
                    if (xfer_ack) begin
                        ep0_tgl_in <= ~ep0_tgl_in;
                        in_valid   <= 1'b0;
                        st         <= C_SOUT;       // 读传输的状态阶段是 OUT ZLP
                    end else if (xfer_timeout) begin
                        in_valid   <= 1'b0;
                        st         <= C_IDLE;
                    end
                end
                //------------------------------------------------------
                C_SOUT: begin
                    if (rx_pkt_valid && (rx_pkt_len == 7'd0)) begin
                        ep0_tgl_out <= ~ep0_tgl_out;
                        st          <= C_IDLE;
                    end
                end
                //------------------------------------------------------
                C_DOUT: begin
                    if (rx_pkt_valid) begin
                        rx_raddr <= 6'd0;
                        st       <= C_DOUT2;
                    end
                end
                //------------------------------------------------------
                C_DOUT2: begin
                    led_rep <= rx_rdata;            // SET_REPORT 只有 1 字节
                    in_len  <= 7'd0;
                    in_pid  <= PID_DATA1;
                    in_ep   <= 4'd0;
                    in_valid<= 1'b1;
                    st      <= C_SIN;
                end
                //------------------------------------------------------
                C_SIN: begin
                    if (xfer_ack) begin
                        in_valid <= 1'b0;
                        ep0_tgl_in <= ~ep0_tgl_in;
                        if (act_setaddr) begin
                            dev_addr   <= pend_addr;    // 状态阶段完成后才生效
                            act_setaddr<= 1'b0;
                        end
                        if (act_setcfg) begin
                            configured <= pend_cfg;
                            act_setcfg <= 1'b0;
                        end
                        st <= C_IDLE;
                    end else if (xfer_timeout) begin
                        in_valid <= 1'b0;
                        st       <= C_IDLE;
                    end
                end
                //------------------------------------------------------
                C_EPLOAD: begin
                    txw_en   <= 1'b1;
                    txw_addr <= ld_idx[5:0];
                    txw_data <= rep_byte;
                    ld_idx   <= ld_idx + 7'd1;
                    if (ld_idx == 7'd7) begin
                        in_len   <= 7'd8;
                        in_pid   <= ep1_tgl ? PID_DATA1 : PID_DATA0;
                        in_ep    <= 4'd1;
                        in_valid <= 1'b1;
                        st       <= C_EPSEND;
                    end
                end
                //------------------------------------------------------
                C_EPSEND: begin
                    if (xfer_ack) begin
                        ep1_tgl  <= ~ep1_tgl;       // 只有被 ACK 才翻转
                        sent_seq <= cap_seq;
                        in_valid <= 1'b0;
                        st       <= C_IDLE;
                    end else if (xfer_timeout) begin
                        in_valid <= 1'b0;           // 未确认：下次重发
                        st       <= C_IDLE;
                    end
                end
                //------------------------------------------------------
                default: st <= C_IDLE;
                endcase
            end
        end
    end

endmodule
