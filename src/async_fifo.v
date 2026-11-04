// Asynchronous FIFO, Gray-code pointers (Cummings, SNUG 2002), write side only
// plus the shared memory. The read pointer lives in the host SPI domain
// (host_spi.v); it arrives here already synchronised (rq_gray).
`default_nettype none

module async_fifo_wr #(
    parameter integer W  = 80,
    parameter integer AW = 3
) (
    input  wire          wclk,
    input  wire          rst_n,
    input  wire          clear,
    input  wire          wr,
    input  wire [W-1:0]  wdata,
    input  wire [AW:0]   rq_gray,     // read pointer, synchronised into the core
    output wire          full,
    output reg  [AW:0]   wbin,
    output reg  [AW:0]   wgray,
    output wire [W-1:0]  rd0,         // memory read ports for the host domain
    input  wire [AW-1:0] ra0,
    output wire [W-1:0]  rd1,
    input  wire [AW-1:0] ra1
);
    reg [W-1:0] mem [0:(1<<AW)-1];

    wire [AW:0] wbin_n  = wbin + 1'b1;
    wire [AW:0] wgray_n = (wbin_n >> 1) ^ wbin_n;
    assign full = (wgray == {~rq_gray[AW:AW-1], rq_gray[AW-2:0]});

    assign rd0 = mem[ra0];
    assign rd1 = mem[ra1];

    always @(posedge wclk) if (wr && !full && !clear) mem[wbin[AW-1:0]] <= wdata;

    always @(posedge wclk or negedge rst_n) begin
        if (!rst_n)       begin wbin <= 0; wgray <= 0; end
        else if (wr && !full && !clear) begin wbin <= wbin_n; wgray <= wgray_n; end
    end
endmodule
