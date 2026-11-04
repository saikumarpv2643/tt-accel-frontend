// Merge datapath (gated clock). Holds the latest sample of each sensor,
// maps axes, scales the high-g value and picks per axis which sensor to use.
//   l14 = mapped low-g >>> 2           (1.952 mg/LSB)
//   hs  = ((mapped high-g >>> 4) * GAIN) >>> 8 + OFFSET, saturated to 20 bits
//   per axis: switch to high-g when |l14| >= THR, back when |l14| < THR - HYST
// Modes: 00 merged (record per high-g sample), 01 low-g only (record per
// low-g sample), 10 high-g only (record per high-g sample).
// One multiplier is shared by the three axes (one axis per clock).
`default_nettype none

module merge_dp (
    input  wire        gclk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire [1:0]  mode,
    input  wire [15:0] thr,
    input  wire [15:0] hyst,
    input  wire [47:0] gain,        // {z, y, x} unsigned Q8.8
    input  wire [47:0] offs,        // {z, y, x} signed, 1.952 mg/LSB
    input  wire [5:0]  perm_l,      // output axis k takes input axis perm[2k+1:2k]
    input  wire [5:0]  perm_h,
    input  wire [2:0]  sign_l,      // 1 = negate output axis k
    input  wire [2:0]  sign_h,
    input  wire        l_valid,
    input  wire        h_valid,
    input  wire [47:0] rx,
    input  wire [15:0] l_ts,
    input  wire [15:0] h_ts,
    output wire        busy,
    output reg         wr,
    output reg  [79:0] wdata,
    input  wire        full,
    output reg         drop,        // pulse: a record was lost (FIFO full)
    output reg  [2:0]  st           // per-axis: 1 = on the high-g sensor
);
    reg [47:0] l_hold, h_hold;
    reg [15:0] ts;
    reg [59:0] vals;
    reg [2:0]  src;
    reg [1:0]  k;
    reg        run, gap, use_h;

    // Keeps the clock gate open until wr and drop have returned to 0. Without
    // drop here, the gate closed with drop still high, and it stayed high until
    // the next sample. That blocked the host's overflow clear in project.v.
    assign busy = run || wr || drop;

    function [15:0] pick(input [47:0] v, input [1:0] p);
        pick = (p == 2'd0) ? v[15:0] : (p == 2'd1) ? v[31:16] : v[47:32];
    endfunction
    function [15:0] neg_sat(input [15:0] x);
        neg_sat = (x == 16'h8000) ? 16'h7FFF : (~x + 16'd1);
    endfunction

    wire [15:0] lr = pick(l_hold, perm_l[k*2 +: 2]);
    wire [15:0] hr = pick(h_hold, perm_h[k*2 +: 2]);
    wire [15:0] lm = sign_l[k] ? neg_sat(lr) : lr;
    wire [15:0] hm = sign_h[k] ? neg_sat(hr) : hr;

    wire signed [19:0] l14  = {{6{lm[15]}}, lm[15:2]};
    wire signed [11:0] h12  = hm[15:4];
    wire signed [16:0] g    = {1'b0, gain[k*16 +: 16]};
    wire signed [28:0] prod = h12 * g;
    wire signed [20:0] ps   = prod >>> 8;
    wire signed [15:0] off  = offs[k*16 +: 16];
    wire signed [21:0] hsum = {ps[20], ps} + {{6{off[15]}}, off};
    wire signed [19:0] hs   = (hsum > 22'sd524287)  ? 20'sd524287 :
                              (hsum < -22'sd524288) ? -20'sd524288 : hsum[19:0];

    wire [15:0] abs_l  = l14[19] ? (~l14[15:0] + 16'd1) : l14[15:0];
    wire [15:0] thr_lo = (thr > hyst) ? (thr - hyst) : 16'd0;
    wire        st_k   = st[k];
    wire        st_new = (abs_l >= thr) ? 1'b1 : (abs_l < thr_lo) ? 1'b0 : st_k;
    wire        sel_h  = (mode == 2'b10) ? 1'b1 : (mode == 2'b01) ? 1'b0 : st_new;
    wire [19:0] out    = sel_h ? hs : l14;

    always @(posedge gclk or negedge rst_n) begin
        if (!rst_n) begin
            l_hold <= 48'd0; h_hold <= 48'd0; ts <= 16'd0; vals <= 60'd0; src <= 3'd0;
            k <= 2'd0; run <= 1'b0; gap <= 1'b0; use_h <= 1'b0;
            wr <= 1'b0; wdata <= 80'd0; drop <= 1'b0; st <= 3'd0;
        end else if (clear) begin
            l_hold <= 48'd0; h_hold <= 48'd0; run <= 1'b0; wr <= 1'b0;
            gap <= 1'b0; st <= 3'd0; drop <= 1'b0;
        end else begin
            wr   <= 1'b0;
            drop <= 1'b0;
            if (l_valid) l_hold <= rx;
            if (h_valid) h_hold <= rx;
            if (!run) begin
                if ((h_valid && mode != 2'b01) || (l_valid && mode == 2'b01)) begin
                    run <= 1'b1;
                    k   <= 2'd0;
                    ts  <= h_valid ? h_ts : l_ts;
                end
            end else begin
                vals[k*20 +: 20] <= out;
                src[k]           <= sel_h;
                if (mode == 2'b00) st[k] <= st_new;
                if (k == 2'd2) begin
                    run <= 1'b0;
                    if (full) begin
                        drop <= 1'b1;
                        gap  <= 1'b1;
                    end else begin
                        wr    <= 1'b1;
                        wdata <= {gap, sel_h, src[1:0], out, vals[39:0], ts};
                        gap   <= 1'b0;
                    end
                end else begin
                    k <= k + 2'd1;
                end
            end
        end
    end
endmodule
