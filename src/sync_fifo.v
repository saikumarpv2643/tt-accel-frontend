// Synchronous FIFO, 2^AW entries of W bits, first-word fall-through.
// A write while full is dropped and sets the sticky overflow flag.
`default_nettype none

module sync_fifo #(
    parameter integer W  = 48,
    parameter integer AW = 3
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         clear,
    input  wire         wr,
    input  wire [W-1:0] wdata,
    input  wire         rd,
    output wire [W-1:0] rdata,
    output wire         empty,
    output wire         full,
    output wire [AW:0]  level,
    output reg          overflow,
    input  wire         ovf_clr
);
    localparam integer DEPTH = 1 << AW;

    reg [W-1:0] mem [0:DEPTH-1];
    reg [AW:0]  wp, rp;

    assign level = wp - rp;
    assign empty = (level == 0);
    assign full  = (level == DEPTH);
    assign rdata = mem[rp[AW-1:0]];

    always @(posedge clk) begin
        if (wr && !full && !clear) mem[wp[AW-1:0]] <= wdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp       <= 0;
            rp       <= 0;
            overflow <= 1'b0;
        end else if (clear) begin
            wp       <= 0;
            rp       <= 0;
            overflow <= 1'b0;
        end else begin
            if (ovf_clr) overflow <= 1'b0;
            if (wr) begin
                if (!full) wp <= wp + 1'b1;
                else       overflow <= 1'b1;   // set wins over clear
            end
            if (rd && !empty) rp <= rp + 1'b1;
        end
    end
endmodule
