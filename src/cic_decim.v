// CIC decimator for three axes, sharing one adder.
//
// Order N, decimation R = 2^r_log2 (r_log2 = 0..RL; 0 = pass-through),
// differential delay 1. Gain R^N is a power of two, so the output is
// normalised with an arithmetic shift of N*r_log2 bits.
//
// Area: samples arrive at 1 kHz but the clock is MHz, so instead of
// 3 axes x 2N adders the filter runs one add per clock through a small
// sequencer. Registers wrap modulo 2^W (W = 16 + N*RL); a CIC filter gives the
// right answer in modular arithmetic as long as W covers the bit growth
// (Hogenauer, 1981).
//
// Busy for 3*(2N+1) clocks per input sample; inputs that arrive while busy
// are dropped (cannot happen with the sensor read taking hundreds of clocks).
// in_data must stay stable while busy (sensor_ctrl holds it until its next
// read), so it is not copied.
`default_nettype none

module cic_decim #(
    parameter integer N  = 2,    // filter order
    parameter integer RL = 4     // max log2(R)
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire [2:0]  r_log2,
    input  wire        in_valid,
    input  wire [47:0] in_data,   // {z, y, x}, signed 16-bit each
    output reg         out_valid,
    output reg  [47:0] out_data
);
    localparam integer W  = 16 + N * RL;
    localparam integer NS = 3 * N;                 // state words per bank
    localparam integer KW = (N > 1) ? $clog2(N) : 1;

    localparam [1:0] PH_INT = 2'd0, PH_COMB = 2'd1, PH_OUT = 2'd2;

    reg [NS*W-1:0] integ;     // integrator states, word (axis*N + k)
    reg [NS*W-1:0] dly;       // comb delay states
    reg [31:0]     outbuf;    // x and y results while z is computed
    reg [W-1:0]    acc;
    reg [RL-1:0]   cnt;
    reg [1:0]      axis;
    reg [1:0]      phase;
    reg [KW-1:0]   k;
    reg            busy;
    reg            dec;

    wire [2:0] r_eff = (r_log2 > RL) ? RL : r_log2;
    wire [RL-1:0] mask = ~({RL{1'b1}} << r_eff);

    // current state word index (axis*N + k), computed without a multiplier
    reg [7:0] widx;
    always @(*) begin
        case (axis)
            2'd0:    widx = k;
            2'd1:    widx = N + k;
            default: widx = 2 * N + k;
        endcase
    end

    wire [W-1:0]  integ_w = integ[widx*W +: W];
    wire [W-1:0]  dly_w   = dly[widx*W +: W];
    wire [15:0]   x_axis  = in_data[axis*16 +: 16];
    wire [W-1:0]  x_ext   = {{(W-16){x_axis[15]}}, x_axis};

    // the one shared adder: integrator add or comb subtract
    wire [W-1:0] add_a = (phase == PH_INT) ? integ_w : acc;
    wire [W-1:0] add_b = (phase == PH_INT) ? ((k == 0) ? x_ext : acc) : ~dly_w;
    wire         add_c = (phase != PH_INT);
    wire [W-1:0] sum   = add_a + add_b + {{(W-1){1'b0}}, add_c};

    wire signed [W-1:0] acc_s   = acc;
    wire signed [W-1:0] shifted = acc_s >>> (N * r_eff);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integ     <= {(NS*W){1'b0}};
            dly       <= {(NS*W){1'b0}};
            outbuf    <= 32'd0;
            acc       <= {W{1'b0}};
            cnt       <= {RL{1'b0}};
            axis      <= 2'd0;
            phase     <= PH_INT;
            k         <= {KW{1'b0}};
            busy      <= 1'b0;
            dec       <= 1'b0;
            out_valid <= 1'b0;
            out_data  <= 48'd0;
        end else begin
            out_valid <= 1'b0;
            if (clear) begin
                integ <= {(NS*W){1'b0}};
                dly   <= {(NS*W){1'b0}};
                cnt   <= {RL{1'b0}};
                busy  <= 1'b0;
            end else if (!busy) begin
                if (in_valid) begin
                    dec   <= (cnt == mask);
                    cnt   <= (cnt == mask) ? {RL{1'b0}} : cnt + 1'b1;
                    busy  <= 1'b1;
                    axis  <= 2'd0;
                    phase <= PH_INT;
                    k     <= {KW{1'b0}};
                end
            end else begin
                case (phase)
                    PH_INT: begin
                        integ[widx*W +: W] <= sum;
                        acc <= sum;
                        if (k == N - 1) begin
                            k <= {KW{1'b0}};
                            if (dec) begin
                                phase <= PH_COMB;
                            end else if (axis == 2'd2) begin
                                busy <= 1'b0;
                            end else begin
                                axis <= axis + 2'd1;
                            end
                        end else begin
                            k <= k + 1'b1;
                        end
                    end
                    PH_COMB: begin
                        dly[widx*W +: W] <= acc;
                        acc <= sum;
                        if (k == N - 1) begin
                            k     <= {KW{1'b0}};
                            phase <= PH_OUT;
                        end else begin
                            k <= k + 1'b1;
                        end
                    end
                    default: begin   // PH_OUT
                        phase <= PH_INT;
                        if (axis == 2'd2) begin
                            busy      <= 1'b0;
                            out_valid <= 1'b1;
                            out_data  <= {shifted[15:0], outbuf};
                        end else begin
                            outbuf[axis[0]*16 +: 16] <= shifted[15:0];
                            axis <= axis + 2'd1;
                        end
                    end
                endcase
            end
        end
    end
endmodule
