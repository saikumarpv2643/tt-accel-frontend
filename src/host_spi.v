// Host SPI slave, mode 0, clocked by the host's SCK (no oversampling, so
// nothing toggles while the host is not reading). Owns the register file and
// the read side of the asynchronous FIFO.
//
// Frame: CS low, {R/W, addr[6:0]} (1 = read), data bytes, auto-increment.
// FIFO_DATA (0x20) streams 10-byte records; a record is popped on the last
// bit of its 10th byte, and the next record's first byte comes from the
// following FIFO slot in the same clock, so a burst never repeats a record.
//
// Crossings out of this domain: settings are quasi-static (see SPEC §7);
// cfg_tgl, snap_tgl and oclr_tgl toggle to tell the core; rgray goes to the
// core's synchroniser. Into this domain: wgray and status bits, each through
// two SCK flops. Values seen here can be stale by the last few SCK edges,
// always on the safe side (fewer records reported than present).
`default_nettype none

module host_spi #(
    parameter integer AW = 3
) (
    input  wire          sck,
    input  wire          mosi,
    input  wire          cs_n,
    input  wire          rst_n,
    output reg           miso,
    // registers
    output reg  [7:0]    ctrl,
    output reg  [7:0]    wm,
    output reg  [7:0]    irqcfg,
    output reg  [15:0]   thr,
    output reg  [15:0]   hyst,
    output reg  [47:0]   gain,
    output reg  [47:0]   offs,
    output reg  [5:0]    perm_l,
    output reg  [2:0]    sign_l,
    output reg  [5:0]    perm_h,
    output reg  [2:0]    sign_h,
    output reg           cfg_tgl,
    output reg           snap_tgl,
    output reg           oclr_tgl,
    input  wire [3:0]    status_async,   // {running, overflow, ok_h, ok_l}
    input  wire [31:0]   time_hold,      // quasi-static snapshot
    // FIFO read side
    input  wire [AW:0]   wgray,
    output reg  [AW:0]   rgray,
    output wire [AW-1:0] ra0,
    output wire [AW-1:0] ra1,
    input  wire [79:0]   rd0,
    input  wire [79:0]   rd1
);
    localparam [6:0] A_ID = 7'h00, A_CTRL = 7'h01, A_STATUS = 7'h02, A_LEVEL = 7'h03,
                     A_WM = 7'h04, A_IRQ = 7'h05, A_THR = 7'h06, A_HYST = 7'h08,
                     A_GAIN = 7'h0A, A_OFFS = 7'h10, A_PERM_L = 7'h16, A_SIGN_L = 7'h17,
                     A_PERM_H = 7'h18, A_SIGN_H = 7'h19, A_TIME = 7'h1C, A_FIFO = 7'h20;

    // ---- synchronisers into the SCK domain
    reg [AW:0] wq1, wq2;
    reg [3:0]  sq1, sq2;
    always @(posedge sck or negedge rst_n)
        if (!rst_n) begin wq1 <= 0; wq2 <= 0; sq1 <= 0; sq2 <= 0; end
        else begin wq1 <= wgray; wq2 <= wq1; sq1 <= status_async; sq2 <= sq1; end

    function [AW:0] g2b(input [AW:0] g);
        integer i;
        begin
            g2b[AW] = g[AW];
            for (i = AW - 1; i >= 0; i = i - 1) g2b[i] = g2b[i+1] ^ g[i];
        end
    endfunction

    reg  [AW:0] rbin;
    wire [AW:0] wbin_s = g2b(wq2);
    wire [AW:0] level  = wbin_s - rbin;
    wire        empty0 = (level == 0);
    wire        empty1 = (level <= 1);
    assign ra0 = rbin[AW-1:0];
    assign ra1 = rbin[AW-1:0] + 1'b1;

    // ---- transaction state (reset while CS is high)
    reg [2:0] bitcnt;
    reg [6:0] rxsh;
    reg       first, rw, word_ok;
    reg [6:0] addr;
    reg [3:0] rbyte;
    reg [7:0] cur;
    wire [7:0] b = {rxsh, mosi};
    wire       byte_done = (bitcnt == 3'd7);

    function [7:0] reg_rd(input [6:0] a);
        case (a)
            A_ID:       reg_rd = 8'hA6;
            A_CTRL:     reg_rd = ctrl;
            A_STATUS:   reg_rd = {4'd0, sq2[3], sq2[2], sq2[1], sq2[0]};
            A_LEVEL:    reg_rd = {{(7-AW){1'b0}}, level};
            A_WM:       reg_rd = wm;
            A_IRQ:      reg_rd = irqcfg;
            7'h06:      reg_rd = thr[7:0];   7'h07: reg_rd = thr[15:8];
            7'h08:      reg_rd = hyst[7:0];  7'h09: reg_rd = hyst[15:8];
            7'h0A:      reg_rd = gain[7:0];  7'h0B: reg_rd = gain[15:8];
            7'h0C:      reg_rd = gain[23:16]; 7'h0D: reg_rd = gain[31:24];
            7'h0E:      reg_rd = gain[39:32]; 7'h0F: reg_rd = gain[47:40];
            7'h10:      reg_rd = offs[7:0];  7'h11: reg_rd = offs[15:8];
            7'h12:      reg_rd = offs[23:16]; 7'h13: reg_rd = offs[31:24];
            7'h14:      reg_rd = offs[39:32]; 7'h15: reg_rd = offs[47:40];
            A_PERM_L:   reg_rd = {2'd0, perm_l};
            A_SIGN_L:   reg_rd = {5'd0, sign_l};
            A_PERM_H:   reg_rd = {2'd0, perm_h};
            A_SIGN_H:   reg_rd = {5'd0, sign_h};
            7'h1C:      reg_rd = time_hold[7:0];  7'h1D: reg_rd = time_hold[15:8];
            7'h1E:      reg_rd = time_hold[23:16]; 7'h1F: reg_rd = time_hold[31:24];
            default:    reg_rd = 8'h00;
        endcase
    endfunction

    wire pop = !cs_n && byte_done && !first && rw && addr == A_FIFO && rbyte == 4'd9 && word_ok;

    always @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            bitcnt <= 3'd0; rxsh <= 7'd0; first <= 1'b1; rw <= 1'b0;
            addr <= 7'd0; rbyte <= 4'd0; cur <= 8'd0; word_ok <= 1'b0;
        end else begin
            rxsh   <= b[6:0];
            bitcnt <= bitcnt + 3'd1;
            if (byte_done) begin
                first <= 1'b0;
                if (first) begin
                    rw <= b[7]; addr <= b[6:0]; rbyte <= 4'd0;
                    if (b[7] && b[6:0] == A_FIFO) begin
                        word_ok <= !empty0;
                        cur     <= empty0 ? 8'd0 : rd0[7:0];
                    end else begin
                        cur <= b[7] ? reg_rd(b[6:0]) : 8'd0;
                    end
                end else if (rw) begin
                    if (addr == A_FIFO) begin
                        if (rbyte == 4'd9) begin
                            rbyte <= 4'd0;
                            if (word_ok) begin
                                word_ok <= !empty1;
                                cur     <= empty1 ? 8'd0 : rd1[7:0];
                            end else begin
                                word_ok <= !empty0;
                                cur     <= empty0 ? 8'd0 : rd0[7:0];
                            end
                        end else begin
                            rbyte <= rbyte + 4'd1;
                            cur   <= word_ok ? rd0[(rbyte + 1) * 8 +: 8] : 8'd0;
                        end
                    end else begin
                        addr <= addr + 7'd1;
                        cur  <= reg_rd(addr + 7'd1);
                    end
                end else begin
                    addr <= addr + 7'd1;
                end
            end
        end
    end

    always @(negedge sck or posedge cs_n)
        if (cs_n) miso <= 1'b0;
        else      miso <= cur[3'd7 - bitcnt];

    // ---- register file, FIFO read pointer, toggles (not reset by CS)
    wire wr_now = !cs_n && byte_done && !first && !rw;
    wire rd_status_done = !cs_n && byte_done && !first && rw && addr == A_STATUS;

    always @(posedge sck or negedge rst_n) begin
        if (!rst_n) begin
            ctrl <= 8'h01; wm <= 8'd0; irqcfg <= 8'h01;
            thr <= 16'd7800; hyst <= 16'd800;
            gain <= {3{16'h6400}}; offs <= 48'd0;
            perm_l <= 6'b10_01_00; sign_l <= 3'd0; perm_h <= 6'b10_01_00; sign_h <= 3'd0;
            cfg_tgl <= 1'b0; snap_tgl <= 1'b0; oclr_tgl <= 1'b0;
            rbin <= 0; rgray <= 0;
        end else begin
            if (pop) begin
                rbin  <= rbin + 1'b1;
                rgray <= ((rbin + 1'b1) >> 1) ^ (rbin + 1'b1);
            end
            if (rd_status_done) oclr_tgl <= ~oclr_tgl;
            if (wr_now) begin
                case (addr)
                    A_CTRL:   begin ctrl <= b; cfg_tgl <= ~cfg_tgl; end
                    A_WM:     wm <= b;
                    A_IRQ:    irqcfg <= b;
                    7'h06: thr[7:0] <= b;    7'h07: thr[15:8] <= b;
                    7'h08: hyst[7:0] <= b;   7'h09: hyst[15:8] <= b;
                    7'h0A: gain[7:0] <= b;   7'h0B: gain[15:8] <= b;
                    7'h0C: gain[23:16] <= b; 7'h0D: gain[31:24] <= b;
                    7'h0E: gain[39:32] <= b; 7'h0F: gain[47:40] <= b;
                    7'h10: offs[7:0] <= b;   7'h11: offs[15:8] <= b;
                    7'h12: offs[23:16] <= b; 7'h13: offs[31:24] <= b;
                    7'h14: offs[39:32] <= b; 7'h15: offs[47:40] <= b;
                    A_PERM_L: perm_l <= b[5:0];
                    A_SIGN_L: sign_l <= b[2:0];
                    A_PERM_H: perm_h <= b[5:0];
                    A_SIGN_H: sign_h <= b[2:0];
                    A_TIME:   snap_tgl <= ~snap_tgl;     // any write takes a snapshot
                    default: ;
                endcase
            end
        end
    end
endmodule
