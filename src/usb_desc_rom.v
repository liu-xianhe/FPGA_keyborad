//=============================================================================
// usb_desc_rom.v  —  描述符表（只读，供协议层取用）
//------------------------------------------------------------------------------
//  设备: HID Boot Keyboard, 1 个配置, 1 个接口, EP1 IN (Interrupt)
//  所有描述符合计 <= 64 字节，可用单包传完，简化 EP0 分包逻辑
//=============================================================================
`timescale 1ns/1ps

module usb_desc_rom (
    input  wire [2:0] sel,
    input  wire [6:0] addr,
    input  wire       cfg_val,     // 当前配置值（GET_CONFIGURATION 返回）
    output reg  [7:0] dout
);

    localparam SEL_DEV  = 3'd0,   // 设备描述符      18 字节
               SEL_CFG  = 3'd1,   // 配置描述符集合  34 字节
               SEL_REP  = 3'd2,   // HID 报告描述符  63 字节
               SEL_HID  = 3'd3,   // HID 描述符       9 字节
               SEL_STR  = 3'd4,   // 字符串 0         4 字节
               SEL_ZERO = 3'd5,   // 恒 0（GET_STATUS 用）
               SEL_CFG1 = 3'd6;   // 当前配置值

    //------------------------------------------------------------------
    // 设备描述符 18 字节
    //  bLength=18 bDescriptorType=1 bcdUSB=0x0200 bDeviceClass=0
    //  bMaxPacketSize0=64 VID=0x1234 PID=0x5678 bcdDevice=0x0100
    //  iMfr=0 iProduct=0 iSerial=0 bNumConfigurations=1
    //------------------------------------------------------------------
    function [7:0] dev_byte;
        input [6:0] a;
        begin
            case (a)
                7'd0 : dev_byte = 8'h12;   // bLength
                7'd1 : dev_byte = 8'h01;   // DEVICE
                7'd2 : dev_byte = 8'h00;   // bcdUSB L
                7'd3 : dev_byte = 8'h02;   // bcdUSB H  -> 2.00
                7'd4 : dev_byte = 8'h00;   // bDeviceClass
                7'd5 : dev_byte = 8'h00;   // bDeviceSubClass
                7'd6 : dev_byte = 8'h00;   // bDeviceProtocol
                7'd7 : dev_byte = 8'h40;   // bMaxPacketSize0 = 64
                7'd8 : dev_byte = 8'h34;   // idVendor  L
                7'd9 : dev_byte = 8'h12;   // idVendor  H
                7'd10: dev_byte = 8'h78;   // idProduct L
                7'd11: dev_byte = 8'h56;   // idProduct H
                7'd12: dev_byte = 8'h00;   // bcdDevice L
                7'd13: dev_byte = 8'h01;   // bcdDevice H
                7'd14: dev_byte = 8'h00;   // iManufacturer
                7'd15: dev_byte = 8'h00;   // iProduct
                7'd16: dev_byte = 8'h00;   // iSerialNumber
                7'd17: dev_byte = 8'h01;   // bNumConfigurations
                default: dev_byte = 8'h00;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // 配置描述符集合 34 字节 = 配置(9) + 接口(9) + HID(9) + 端点(7)
    //  wTotalLength = 0x22 = 34
    //------------------------------------------------------------------
    function [7:0] cfg_byte;
        input [6:0] a;
        begin
            case (a)
                // ---- 配置描述符 ----
                7'd0 : cfg_byte = 8'h09;   // bLength
                7'd1 : cfg_byte = 8'h02;   // CONFIGURATION
                7'd2 : cfg_byte = 8'h22;   // wTotalLength L = 34
                7'd3 : cfg_byte = 8'h00;   // wTotalLength H
                7'd4 : cfg_byte = 8'h01;   // bNumInterfaces
                7'd5 : cfg_byte = 8'h01;   // bConfigurationValue
                7'd6 : cfg_byte = 8'h00;   // iConfiguration
                7'd7 : cfg_byte = 8'h80;   // bmAttributes: bus powered
                7'd8 : cfg_byte = 8'h32;   // bMaxPower = 100mA
                // ---- 接口描述符 ----
                7'd9 : cfg_byte = 8'h09;
                7'd10: cfg_byte = 8'h04;   // INTERFACE
                7'd11: cfg_byte = 8'h00;   // bInterfaceNumber
                7'd12: cfg_byte = 8'h00;   // bAlternateSetting
                7'd13: cfg_byte = 8'h01;   // bNumEndpoints = 1
                7'd14: cfg_byte = 8'h03;   // bInterfaceClass = HID
                7'd15: cfg_byte = 8'h01;   // bInterfaceSubClass = Boot
                7'd16: cfg_byte = 8'h01;   // bInterfaceProtocol = Keyboard
                7'd17: cfg_byte = 8'h00;   // iInterface
                // ---- HID 描述符 ----
                7'd18: cfg_byte = 8'h09;
                7'd19: cfg_byte = 8'h21;   // HID
                7'd20: cfg_byte = 8'h11;   // bcdHID L = 0x0111
                7'd21: cfg_byte = 8'h01;   // bcdHID H
                7'd22: cfg_byte = 8'h00;   // bCountryCode
                7'd23: cfg_byte = 8'h01;   // bNumDescriptors
                7'd24: cfg_byte = 8'h22;   // Report descriptor
                7'd25: cfg_byte = 8'h3F;   // wDescriptorLength L = 63
                7'd26: cfg_byte = 8'h00;   // wDescriptorLength H
                // ---- 端点描述符 EP1 IN ----
                7'd27: cfg_byte = 8'h07;
                7'd28: cfg_byte = 8'h05;   // ENDPOINT
                7'd29: cfg_byte = 8'h81;   // bEndpointAddress: EP1 IN
                7'd30: cfg_byte = 8'h03;   // bmAttributes: Interrupt
                7'd31: cfg_byte = 8'h08;   // wMaxPacketSize = 8
                7'd32: cfg_byte = 8'h00;
                7'd33: cfg_byte = 8'h0A;   // bInterval = 10ms (FS)
                default: cfg_byte = 8'h00;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // HID 报告描述符（Boot Keyboard, 63 字节）
    //  字节 0 : 8 位修饰键
    //  字节 1 : 保留
    //  字节 2-7 : 6 个按键 Usage
    //  输出 1 字节 : LED (Caps/Num/Scroll)
    //------------------------------------------------------------------
    function [7:0] rep_byte;
        input [6:0] a;
        begin
            case (a)
                7'd0 : rep_byte = 8'h05;  7'd1 : rep_byte = 8'h01; // Usage Page (Generic Desktop)
                7'd2 : rep_byte = 8'h09;  7'd3 : rep_byte = 8'h06; // Usage (Keyboard)
                7'd4 : rep_byte = 8'hA1;  7'd5 : rep_byte = 8'h01; // Collection (Application)
                7'd6 : rep_byte = 8'h05;  7'd7 : rep_byte = 8'h07; //   Usage Page (Keyboard)
                7'd8 : rep_byte = 8'h19;  7'd9 : rep_byte = 8'hE0; //   Usage Min (0xE0)
                7'd10: rep_byte = 8'h29;  7'd11: rep_byte = 8'hE7; //   Usage Max (0xE7)
                7'd12: rep_byte = 8'h15;  7'd13: rep_byte = 8'h00; //   Logical Min 0
                7'd14: rep_byte = 8'h25;  7'd15: rep_byte = 8'h01; //   Logical Max 1
                7'd16: rep_byte = 8'h75;  7'd17: rep_byte = 8'h01; //   Report Size 1
                7'd18: rep_byte = 8'h95;  7'd19: rep_byte = 8'h08; //   Report Count 8
                7'd20: rep_byte = 8'h81;  7'd21: rep_byte = 8'h02; //   Input(Data,Var,Abs)
                7'd22: rep_byte = 8'h95;  7'd23: rep_byte = 8'h01; //   Report Count 1
                7'd24: rep_byte = 8'h75;  7'd25: rep_byte = 8'h08; //   Report Size 8
                7'd26: rep_byte = 8'h81;  7'd27: rep_byte = 8'h01; //   Input(Const) 保留字节
                7'd28: rep_byte = 8'h95;  7'd29: rep_byte = 8'h05; //   Report Count 5
                7'd30: rep_byte = 8'h75;  7'd31: rep_byte = 8'h01; //   Report Size 1
                7'd32: rep_byte = 8'h05;  7'd33: rep_byte = 8'h08; //   Usage Page (LEDs)
                7'd34: rep_byte = 8'h19;  7'd35: rep_byte = 8'h01; //   Usage Min 1
                7'd36: rep_byte = 8'h29;  7'd37: rep_byte = 8'h05; //   Usage Max 5
                7'd38: rep_byte = 8'h91;  7'd39: rep_byte = 8'h02; //   Output(Data,Var,Abs)
                7'd40: rep_byte = 8'h95;  7'd41: rep_byte = 8'h01; //   Report Count 1
                7'd42: rep_byte = 8'h75;  7'd43: rep_byte = 8'h03; //   Report Size 3
                7'd44: rep_byte = 8'h91;  7'd45: rep_byte = 8'h01; //   Output(Const) 补齐
                7'd46: rep_byte = 8'h95;  7'd47: rep_byte = 8'h06; //   Report Count 6
                7'd48: rep_byte = 8'h75;  7'd49: rep_byte = 8'h08; //   Report Size 8
                7'd50: rep_byte = 8'h15;  7'd51: rep_byte = 8'h00; //   Logical Min 0
                7'd52: rep_byte = 8'h25;  7'd53: rep_byte = 8'h65; //   Logical Max 101
                7'd54: rep_byte = 8'h05;  7'd55: rep_byte = 8'h07; //   Usage Page (Keyboard)
                7'd56: rep_byte = 8'h19;  7'd57: rep_byte = 8'h00; //   Usage Min 0
                7'd58: rep_byte = 8'h29;  7'd59: rep_byte = 8'h65; //   Usage Max 101
                7'd60: rep_byte = 8'h81;  7'd61: rep_byte = 8'h00; //   Input(Data,Array)
                7'd62: rep_byte = 8'hC0;                          // End Collection
                default: rep_byte = 8'h00;
            endcase
        end
    endfunction

    always @(*) begin
        case (sel)
            SEL_DEV : dout = dev_byte(addr);
            SEL_CFG : dout = cfg_byte(addr);
            SEL_REP : dout = rep_byte(addr);
            SEL_HID : dout = cfg_byte(addr + 7'd18);              // HID 描述符在配置集合内
            SEL_STR : dout = (addr == 7'd0) ? 8'h04 :
                             (addr == 7'd1) ? 8'h03 :
                             (addr == 7'd2) ? 8'h09 :
                             (addr == 7'd3) ? 8'h04 : 8'h00;      // LangID 0x0409
            SEL_ZERO: dout = 8'h00;
            SEL_CFG1: dout = (addr == 7'd0) ? {7'd0, cfg_val} : 8'h00;
            default : dout = 8'h00;
        endcase
    end

endmodule
