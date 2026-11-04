`default_nettype none
`timescale 1ns / 1ps

module tb ();
    initial begin
        $dumpfile("tb.fst");
        $dumpvars(0, tb);
        #1;
    end

    reg clk, rst_n, ena;
    reg [7:0] uio_in;
    reg miso_l, miso_h, drdy_l, drdy_h;
    reg host_sck, host_mosi, host_cs_n;

    wire [7:0] uo_out, uio_out, uio_oe;
    wire s_sck  = uo_out[0];
    wire s_mosi = uo_out[1];
    wire cs_l_n = uo_out[2];
    wire cs_h_n = uo_out[3];
    wire host_miso = uo_out[4];
    wire irq    = uo_out[5];
    wire sat    = uo_out[6];
    wire running = uo_out[7];

    // shared sensor MISO: each sensor drives only while selected
    wire s_miso = !cs_l_n ? miso_l : !cs_h_n ? miso_h : 1'b1;
    wire [7:0] ui_in = {2'b00, host_cs_n, host_mosi, host_sck, drdy_h, drdy_l, s_miso};

`ifdef GL_TEST
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    tt_um_saikumarpv2643_accel_v2
`ifndef GL_TEST
        #(.BOOT_CYCLES(50))
`endif
    user_project (
`ifdef GL_TEST
        .VPWR(VPWR), .VGND(VGND),
`endif
        .ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in), .uio_out(uio_out),
        .uio_oe(uio_oe), .ena(ena), .clk(clk), .rst_n(rst_n));
endmodule
