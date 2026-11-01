// H3LIS331DL controller.
//
// Sequence after reset:
//   1. wait BOOT_CYCLES (sensor boot time is 5 ms, datasheet table 4)
//   2. read WHO_AM_I (0x0F); expect 0x32. On a mismatch wait and retry.
//   3. write CTRL_REG1..4 in one auto-increment burst:
//        REG1 = 0x3F  normal mode, ODR 1000 Hz, X/Y/Z on   (0x07 = power-down)
//        REG2 = 0x00  no high-pass filter
//        REG3 = 0x02  INT1 = data ready
//        REG4 = BDU | FS<<4   (FS 00 = 100 g, 01 = 200 g, 11 = 400 g)
//   4. whenever DRDY (INT1) is high: read OUT_X_L..OUT_Z_H (0x28..0x2D) in one
//      burst and emit one 48-bit sample {Z, Y, X}, little-endian per axis.
// A reconfig pulse (host wrote CTRL) repeats step 3 before the next read.
`default_nettype none

module sensor_ctrl #(
    parameter integer BOOT_CYCLES = 50000,   // 5 ms at 10 MHz
    parameter integer GAP_CYCLES  = 8        // CS high time between transactions
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [1:0]  range,       // 00 100 g, 01 200 g, 1x 400 g
    input  wire        reconfig,
    input  wire        drdy,        // synchronised INT1
    // SPI master byte interface
    output reg         spi_start,
    output reg  [7:0]  spi_tx,
    input  wire [7:0]  spi_rx,
    input  wire        spi_done,
    output reg         cs_n,
    // status and data
    output reg  [7:0]  whoami,
    output reg         whoami_ok,
    output reg         cfg_ok,
    output reg         sample_valid,
    output wire [47:0] sample       // valid with sample_valid, held until the next read
);
    localparam [7:0] WHO_AM_I_VAL = 8'h32;

    localparam [1:0] K_WHO = 2'd0, K_CFG = 2'd1, K_READ = 2'd2;

    localparam [2:0] ST_BOOT = 3'd0, ST_IDLE = 3'd1, ST_CS = 3'd2,
                     ST_SEND = 3'd3, ST_WAIT = 3'd4, ST_GAP = 3'd5;

    localparam integer BW = (BOOT_CYCLES > 1) ? $clog2(BOOT_CYCLES + 1) : 1;

    reg [2:0]    state;
    reg [1:0]    kind;
    reg [2:0]    idx;          // byte index inside the transaction
    reg [BW-1:0] timer;
    reg [47:0]   rxbuf;
    assign sample = rxbuf;
    reg          cfg_pending;

    wire [1:0] fs   = (range == 2'b10) ? 2'b11 : range;   // 10 is reserved
    wire [7:0] reg1 = enable ? 8'h3F : 8'h07;
    wire [7:0] reg4 = {1'b1, 1'b0, fs, 4'b0000};

    // number of bytes in the transaction, address byte included
    reg [2:0] last_idx;
    always @(*) begin
        case (kind)
            K_WHO:   last_idx = 3'd1;
            K_CFG:   last_idx = 3'd4;
            default: last_idx = 3'd6;
        endcase
    end

    // byte to send for (kind, idx)
    reg [7:0] tx_sel;
    always @(*) begin
        tx_sel = 8'h00;
        case (kind)
            K_WHO:  if (idx == 3'd0) tx_sel = 8'h8F;          // read 0x0F
            K_CFG:  case (idx)
                        3'd0: tx_sel = 8'h60;                 // write, auto-inc, 0x20
                        3'd1: tx_sel = reg1;
                        3'd2: tx_sel = 8'h00;
                        3'd3: tx_sel = 8'h02;
                        default: tx_sel = reg4;
                    endcase
            default: if (idx == 3'd0) tx_sel = 8'hE8;         // read, auto-inc, 0x28
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_BOOT;
            kind         <= K_WHO;
            idx          <= 3'd0;
            timer        <= {BW{1'b0}};
            rxbuf        <= 48'd0;
            cfg_pending  <= 1'b0;
            spi_start    <= 1'b0;
            spi_tx       <= 8'd0;
            cs_n         <= 1'b1;
            whoami       <= 8'd0;
            whoami_ok    <= 1'b0;
            cfg_ok       <= 1'b0;
            sample_valid <= 1'b0;
        end else begin
            spi_start    <= 1'b0;
            sample_valid <= 1'b0;
            if (reconfig) cfg_pending <= 1'b1;

            case (state)
                ST_BOOT: begin
                    if (timer == BOOT_CYCLES) begin
                        timer <= {BW{1'b0}};
                        kind  <= K_WHO;
                        state <= ST_CS;
                    end else begin
                        timer <= timer + 1'b1;
                    end
                end

                ST_IDLE: begin
                    if (cfg_pending) begin
                        cfg_pending <= reconfig;   // keep a request that arrives now
                        kind        <= K_CFG;
                        state       <= ST_CS;
                    end else if (cfg_ok && enable && drdy) begin
                        kind  <= K_READ;
                        state <= ST_CS;
                    end
                end

                ST_CS: begin
                    cs_n  <= 1'b0;
                    idx   <= 3'd0;
                    state <= ST_SEND;
                end

                ST_SEND: begin
                    spi_tx    <= tx_sel;
                    spi_start <= 1'b1;
                    state     <= ST_WAIT;
                end

                ST_WAIT: begin
                    if (spi_done) begin
                        if (idx != 3'd0) rxbuf <= {spi_rx, rxbuf[47:8]};
                        if (idx == last_idx) begin
                            cs_n  <= 1'b1;
                            timer <= {BW{1'b0}};
                            state <= ST_GAP;
                        end else begin
                            idx   <= idx + 3'd1;
                            state <= ST_SEND;
                        end
                    end
                end

                ST_GAP: begin
                    if (timer == GAP_CYCLES) begin
                        timer <= {BW{1'b0}};
                        state <= ST_IDLE;
                        case (kind)
                            K_WHO: begin
                                whoami <= rxbuf[47:40];
                                if (rxbuf[47:40] == WHO_AM_I_VAL) begin
                                    whoami_ok   <= 1'b1;
                                    cfg_pending <= 1'b1;
                                end else begin
                                    whoami_ok <= 1'b0;
                                    state     <= ST_BOOT;   // retry after the boot delay
                                end
                            end
                            K_CFG:  cfg_ok <= 1'b1;
                            default: begin
                                sample_valid <= 1'b1;
                            end
                        endcase
                    end else begin
                        timer <= timer + 1'b1;
                    end
                end

                default: state <= ST_BOOT;
            endcase
        end
    end
endmodule
