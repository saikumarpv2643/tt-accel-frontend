// Clock gate. Design rule: every gated block must work correctly with its
// gate forced on, so gating only saves power and never changes behaviour.
//   ASIC (define SKY130):   sky130 integrated clock gate cell
//   FPGA (define FPGA) or FORCE_GATES_ON: clock passes straight through
//   otherwise (simulation): latch + AND, the textbook ICG
`default_nettype none

module clk_gate (
    input  wire clk,
    input  wire en,
    output wire gclk
);
`ifdef SKY130
    sky130_fd_sc_hd__dlclkp_1 u_icg (.CLK(clk), .GATE(en), .GCLK(gclk));
`elsif FPGA
    assign gclk = clk;
    wire _unused = en;
`elsif FORCE_GATES_ON
    assign gclk = clk;
    wire _unused = en;
`else
    reg en_l;
    always @(clk or en) if (!clk) en_l = en;   // transparent while clk is low
    assign gclk = clk & en_l;
`endif
endmodule
