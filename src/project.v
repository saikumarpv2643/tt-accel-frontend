// Accelerometer interface: H3LIS331DL SPI master -> CIC decimator -> FIFO ->
// SPI slave for the host. Tiny Tapeout top level.
//
// Pins
//   ui_in[0]  SENS_MISO   sensor SDO
//   ui_in[1]  SENS_DRDY   sensor INT1 (data ready)
//   ui_in[2]  HOST_SCK
//   ui_in[3]  HOST_MOSI
//   ui_in[4]  HOST_CS_N
//   uo_out[0] SENS_SCK
//   uo_out[1] SENS_MOSI   sensor SDI
//   uo_out[2] SENS_CS_N
//   uo_out[3] HOST_MISO
//   uo_out[4] IRQ         FIFO level >= IRQ_THR
//   uo_out[5] SENSOR_OK   WHO_AM_I matched
//   uo_out[6] OVERFLOW
//   uo_out[7] RUNNING
//   uio       unused (inputs)
`default_nettype none

module tt_um_saikumarpv2643_accel_if #(
    parameter integer BOOT_CYCLES = 50000,   // 5 ms at 10 MHz
    parameter integer SPI_DIV     = 4,       // sensor SCK = clk / 8
    parameter integer CIC_N       = 2,
    parameter integer CIC_RL      = 4,       // decimation up to 16
    parameter integer FIFO_AW     = 3        // 8 samples
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // data-ready from the sensor is asynchronous: two-flop synchroniser
    reg [1:0] drdy_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) drdy_r <= 2'b00;
        else        drdy_r <= {drdy_r[0], ui_in[1]};

    // host registers
    wire [7:0] ctrl;
    wire [3:0] irq_thr;
    wire       ctrl_wr;
    wire       enable = ctrl[0];
    wire [1:0] range  = ctrl[2:1];
    wire [2:0] r_log2 = ctrl[6:4];

    // sensor side
    wire       spi_start, spi_done, spi_busy;
    wire [7:0] spi_tx, spi_rx;
    wire       sens_sck, sens_mosi, sens_cs_n;
    wire [7:0] whoami;
    wire       whoami_ok, cfg_ok, raw_valid;
    wire [47:0] raw;

    spi_master #(.DIV(SPI_DIV)) u_spi (
        .clk(clk), .rst_n(rst_n),
        .start(spi_start), .tx_byte(spi_tx), .rx_byte(spi_rx),
        .done(spi_done), .busy(spi_busy),
        .sck(sens_sck), .mosi(sens_mosi), .miso(ui_in[0])
    );

    sensor_ctrl #(.BOOT_CYCLES(BOOT_CYCLES)) u_ctrl (
        .clk(clk), .rst_n(rst_n),
        .enable(enable), .range(range), .reconfig(ctrl_wr), .drdy(drdy_r[1]),
        .spi_start(spi_start), .spi_tx(spi_tx), .spi_rx(spi_rx), .spi_done(spi_done),
        .cs_n(sens_cs_n),
        .whoami(whoami), .whoami_ok(whoami_ok), .cfg_ok(cfg_ok),
        .sample_valid(raw_valid), .sample(raw)
    );

    // decimator
    wire        dec_valid;
    wire [47:0] dec_data;

    cic_decim #(.N(CIC_N), .RL(CIC_RL)) u_cic (
        .clk(clk), .rst_n(rst_n), .clear(ctrl_wr), .r_log2(r_log2),
        .in_valid(raw_valid), .in_data(raw),
        .out_valid(dec_valid), .out_data(dec_data)
    );

    // FIFO
    wire [47:0]      fifo_rdata;
    wire             fifo_empty, fifo_full, overflow, fifo_pop, status_rd;
    wire [FIFO_AW:0] level;

    sync_fifo #(.W(48), .AW(FIFO_AW)) u_fifo (
        .clk(clk), .rst_n(rst_n), .clear(ctrl_wr),
        .wr(dec_valid), .wdata(dec_data),
        .rd(fifo_pop), .rdata(fifo_rdata),
        .empty(fifo_empty), .full(fifo_full), .level(level),
        .overflow(overflow), .ovf_clr(status_rd)
    );

    // host side
    wire       running = whoami_ok & cfg_ok & enable;
    wire [3:0] level4  = (level > 15) ? 4'd15 : level[3:0];
    wire [7:0] status  = {level4, 1'b0, running, overflow, whoami_ok};
    wire       host_miso;

    host_if u_host (
        .clk(clk), .rst_n(rst_n),
        .sck_in(ui_in[2]), .mosi_in(ui_in[3]), .cs_n_in(ui_in[4]),
        .miso(host_miso),
        .ctrl(ctrl), .irq_thr(irq_thr), .ctrl_wr(ctrl_wr),
        .status(status), .whoami(whoami),
        .fifo_rdata(fifo_rdata), .fifo_empty(fifo_empty), .fifo_pop(fifo_pop),
        .status_rd(status_rd)
    );

    wire irq = (irq_thr != 4'd0) && (level >= irq_thr);

    assign uo_out  = {running, overflow, whoami_ok, irq,
                      host_miso, sens_cs_n, sens_mosi, sens_sck};
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    wire _unused = &{ena, uio_in, ui_in[7:5], spi_busy, fifo_full, 1'b0};
endmodule
