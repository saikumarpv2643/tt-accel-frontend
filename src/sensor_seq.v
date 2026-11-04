// Sequencer for two sensors on one SPI bus (always-on domain).
//   H = H3LIS331DL (high-g, 1000 Hz, record pacer), L = IIS2DLPC (low-g, 1600 Hz)
// Boot: wait, read both WHO_AM_I (0x32 / 0x44), retry until both match,
// then write both configurations. Run: read whichever DRDY is high, H first.
// Each sample's timestamp is taken when its DRDY rises, not when it is read.
`default_nettype none

module sensor_seq #(
    parameter integer BOOT_CYCLES = 5000,   // 5 ms at 1 MHz
    parameter integer GAP_CYCLES  = 2
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [1:0]  hrange,      // 00 100 g, 01 200 g, 1x 400 g
    input  wire        reconfig,
    input  wire        drdy_l,      // synchronised levels
    input  wire        drdy_h,
    input  wire [15:0] now,
    output wire        spi_active,  // clock-gate enable for the SPI master
    output reg         spi_start,
    output reg  [7:0]  spi_tx,
    input  wire [7:0]  spi_rx,
    input  wire        spi_done,
    output reg         cs_l_n,
    output reg         cs_h_n,
    output reg         ok_l,
    output reg         ok_h,
    output reg         cfg_done,
    output reg         l_valid,
    output reg         h_valid,
    output reg  [47:0] rxbuf,       // {z, y, x}, held until the next read
    output reg  [15:0] l_ts,
    output reg  [15:0] h_ts
);
    localparam [2:0] K_WHO_H = 3'd0, K_WHO_L = 3'd1, K_CFG_H = 3'd2,
                     K_CFG_L = 3'd3, K_RD_H  = 3'd4, K_RD_L  = 3'd5;
    localparam [2:0] ST_BOOT = 3'd0, ST_IDLE = 3'd1, ST_CS = 3'd2,
                     ST_SEND = 3'd3, ST_WAIT = 3'd4, ST_GAP = 3'd5;
    localparam integer BW = $clog2(BOOT_CYCLES + 1);

    reg [2:0]    state, kind;
    reg [2:0]    idx;
    reg [BW-1:0] timer;
    reg          cfg_pend;
    reg          prev_l, prev_h;
    reg [15:0]   l_cap, h_cap;

    assign spi_active = (state == ST_SEND) || (state == ST_WAIT);

    wire is_l = (kind == K_WHO_L) || (kind == K_CFG_L) || (kind == K_RD_L);
    wire [1:0] fs = (hrange == 2'b10) ? 2'b11 : hrange;

    reg [2:0] last_idx;
    always @(*) begin
        case (kind)
            K_WHO_H, K_WHO_L: last_idx = 3'd1;
            K_CFG_H:          last_idx = 3'd4;
            default:          last_idx = 3'd6;   // CFG_L and reads
        endcase
    end

    reg [7:0] tx_sel;
    always @(*) begin
        tx_sel = 8'h00;
        case (kind)
            K_WHO_H, K_WHO_L: if (idx == 0) tx_sel = 8'h8F;
            K_CFG_H: case (idx)                        // 0x20.. auto-increment (MS bit)
                3'd0: tx_sel = 8'h60;
                3'd1: tx_sel = enable ? 8'h3F : 8'h07;   // normal 1000 Hz / power-down
                3'd2: tx_sel = 8'h00;
                3'd3: tx_sel = 8'h02;                    // DRDY on INT1
                default: tx_sel = {2'b10, fs, 4'b0000};  // BDU, range
            endcase
            K_CFG_L: case (idx)                        // [verify] IIS2DLPC 0x20..0x25
                3'd0: tx_sel = 8'h20;
                3'd1: tx_sel = enable ? 8'h94 : 8'h00;   // 1600 Hz high-perf / power-down
                3'd2: tx_sel = 8'h0E;                    // BDU, auto-inc, I2C off
                3'd3: tx_sel = 8'h00;
                3'd4: tx_sel = 8'h01;                    // DRDY on INT1
                3'd5: tx_sel = 8'h00;
                default: tx_sel = 8'h34;                 // +-16 g, low noise
            endcase
            K_RD_H: if (idx == 0) tx_sel = 8'hE8;
            default: if (idx == 0) tx_sel = 8'hA8;     // K_RD_L
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_BOOT; kind <= K_WHO_H; idx <= 3'd0; timer <= {BW{1'b0}};
            cfg_pend <= 1'b0; prev_l <= 1'b0; prev_h <= 1'b0;
            l_cap <= 16'd0; h_cap <= 16'd0; l_ts <= 16'd0; h_ts <= 16'd0;
            spi_start <= 1'b0; spi_tx <= 8'd0; cs_l_n <= 1'b1; cs_h_n <= 1'b1;
            ok_l <= 1'b0; ok_h <= 1'b0; cfg_done <= 1'b0;
            l_valid <= 1'b0; h_valid <= 1'b0; rxbuf <= 48'd0;
        end else begin
            spi_start <= 1'b0;
            l_valid   <= 1'b0;
            h_valid   <= 1'b0;
            prev_l    <= drdy_l;
            prev_h    <= drdy_h;
            if (drdy_l && !prev_l) l_cap <= now;
            if (drdy_h && !prev_h) h_cap <= now;
            if (reconfig) cfg_pend <= 1'b1;

            case (state)
                ST_BOOT: if (timer == BOOT_CYCLES) begin
                    timer <= {BW{1'b0}}; kind <= K_WHO_H; state <= ST_CS;
                end else timer <= timer + 1'b1;

                ST_IDLE: begin
                    if (cfg_pend) begin
                        cfg_pend <= reconfig; kind <= K_CFG_H; state <= ST_CS;
                    end else if (cfg_done && enable && drdy_h) begin
                        kind <= K_RD_H; state <= ST_CS;
                    end else if (cfg_done && enable && drdy_l) begin
                        kind <= K_RD_L; state <= ST_CS;
                    end
                end

                ST_CS: begin
                    if (is_l) cs_l_n <= 1'b0; else cs_h_n <= 1'b0;
                    idx <= 3'd0; state <= ST_SEND;
                end

                ST_SEND: begin spi_tx <= tx_sel; spi_start <= 1'b1; state <= ST_WAIT; end

                ST_WAIT: if (spi_done) begin
                    if (idx != 0) rxbuf <= {spi_rx, rxbuf[47:8]};
                    if (idx == last_idx) begin
                        cs_l_n <= 1'b1; cs_h_n <= 1'b1; timer <= {BW{1'b0}}; state <= ST_GAP;
                    end else begin
                        idx <= idx + 3'd1; state <= ST_SEND;
                    end
                end

                ST_GAP: if (timer == GAP_CYCLES) begin
                    timer <= {BW{1'b0}};
                    state <= ST_IDLE;
                    case (kind)
                        K_WHO_H: begin ok_h <= (rxbuf[47:40] == 8'h32); kind <= K_WHO_L; state <= ST_CS; end
                        K_WHO_L: begin
                            ok_l <= (rxbuf[47:40] == 8'h44);
                            if (rxbuf[47:40] == 8'h44 && ok_h) cfg_pend <= 1'b1;
                            else state <= ST_BOOT;               // retry after the boot delay
                        end
                        K_CFG_H: begin kind <= K_CFG_L; state <= ST_CS; end
                        K_CFG_L: cfg_done <= 1'b1;
                        K_RD_H:  begin h_valid <= 1'b1; h_ts <= h_cap; end
                        default: begin l_valid <= 1'b1; l_ts <= l_cap; end
                    endcase
                end else timer <= timer + 1'b1;

                default: state <= ST_BOOT;
            endcase
        end
    end
endmodule
