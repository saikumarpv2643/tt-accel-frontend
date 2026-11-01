`default_nettype none
`timescale 1ns / 1ps

// Testbench wrapper: breaks the Tiny Tapeout pin vectors out into named
// signals so the cocotb models can wait on single-bit edges.
module tb ();
    initial begin
        $dumpfile("tb.fst");
        $dumpvars(0, tb);
        #1;
    end

    reg clk;
    reg rst_n;
    reg ena;
    reg [7:0] uio_in;

    // driven by the cocotb models
    reg sens_miso;
    reg sens_drdy;
    reg host_sck;
    reg host_mosi;
    reg host_cs_n;

    wire [7:0] ui_in = {3'b000, host_cs_n, host_mosi, host_sck, sens_drdy, sens_miso};
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    wire sens_sck  = uo_out[0];
    wire sens_mosi = uo_out[1];
    wire sens_cs_n = uo_out[2];
    wire host_miso = uo_out[3];
    wire irq       = uo_out[4];
    wire sensor_ok = uo_out[5];
    wire overflow  = uo_out[6];
    wire running   = uo_out[7];

`ifdef GL_TEST
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    tt_um_saikumarpv2643_accel_if
`ifndef GL_TEST
        #(.BOOT_CYCLES(200))
`endif
    user_project (
`ifdef GL_TEST
        .VPWR(VPWR),
        .VGND(VGND),
`endif
        .ui_in  (ui_in),
        .uo_out (uo_out),
        .uio_in (uio_in),
        .uio_out(uio_out),
        .uio_oe (uio_oe),
        .ena    (ena),
        .clk    (clk),
        .rst_n  (rst_n)
    );
endmodule
