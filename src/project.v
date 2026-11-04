// accel-asic v2: low-power dual-sensor front-end. Tiny Tapeout top level.
// See ../SPEC.md for pins, registers, record format and clock domains.
`default_nettype none

module tt_um_saikumarpv2643_accel_v2 #(
    parameter integer BOOT_CYCLES = 5000,   // 5 ms at 1 MHz
    parameter integer FIFO_AW     = 3       // 8 records
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,        // 1 MHz core clock
    input  wire       rst_n
);
    wire s_miso = ui_in[0];
    wire host_sck = ui_in[3], host_mosi = ui_in[4], host_cs_n = ui_in[5];

    // ---------------- always-on domain
    reg [1:0] dl, dh;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin dl <= 0; dh <= 0; end
        else begin dl <= {dl[0], ui_in[1]}; dh <= {dh[0], ui_in[2]}; end

    reg [31:0] now;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) now <= 0; else now <= now + 1;

    // host -> core toggles
    wire cfg_tgl, snap_tgl, oclr_tgl;
    reg [2:0] cfg_s, snap_s, oclr_s;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin cfg_s <= 0; snap_s <= 0; oclr_s <= 0; end
        else begin
            cfg_s  <= {cfg_s[1:0], cfg_tgl};
            snap_s <= {snap_s[1:0], snap_tgl};
            oclr_s <= {oclr_s[1:0], oclr_tgl};
        end
    wire cfg_upd  = cfg_s[2]  ^ cfg_s[1];
    wire snap_upd = snap_s[2] ^ snap_s[1];
    wire oclr_upd = oclr_s[2] ^ oclr_s[1];

    wire [7:0] ctrl_h;                 // host-domain CTRL
    reg  [7:0] ctrl;                   // core shadow, copied on cfg_upd
    reg        reconfig;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin ctrl <= 8'h01; reconfig <= 1'b0; end
        else begin
            reconfig <= 1'b0;
            if (cfg_upd) begin ctrl <= ctrl_h; reconfig <= 1'b1; end
        end
    wire       enable = ctrl[0];
    wire [1:0] mode   = ctrl[2:1];
    wire [1:0] hrange = ctrl[4:3];

    reg [31:0] time_hold;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) time_hold <= 0; else if (snap_upd) time_hold <= now;

    // ---------------- sensor sequencer + gated SPI master
    wire spi_active, spi_start, spi_done, spi_busy;
    wire [7:0] spi_tx, spi_rx;
    wire sck_o, mosi_o, cs_l_n, cs_h_n, ok_l, ok_h, cfg_done, l_valid, h_valid;
    wire [47:0] rxbuf;
    wire [15:0] l_ts, h_ts;
    wire gclk_spi;

    clk_gate u_g_spi (.clk(clk), .en(spi_active), .gclk(gclk_spi));

    spi_master #(.DIV(1)) u_spi (
        .clk(gclk_spi), .rst_n(rst_n), .start(spi_start), .tx_byte(spi_tx),
        .rx_byte(spi_rx), .done(spi_done), .busy(spi_busy),
        .sck(sck_o), .mosi(mosi_o), .miso(s_miso));

    sensor_seq #(.BOOT_CYCLES(BOOT_CYCLES)) u_seq (
        .clk(clk), .rst_n(rst_n), .enable(enable), .hrange(hrange), .reconfig(reconfig),
        .drdy_l(dl[1]), .drdy_h(dh[1]), .now(now[15:0]),
        .spi_active(spi_active), .spi_start(spi_start), .spi_tx(spi_tx),
        .spi_rx(spi_rx), .spi_done(spi_done), .cs_l_n(cs_l_n), .cs_h_n(cs_h_n),
        .ok_l(ok_l), .ok_h(ok_h), .cfg_done(cfg_done),
        .l_valid(l_valid), .h_valid(h_valid), .rxbuf(rxbuf), .l_ts(l_ts), .h_ts(h_ts));

    // ---------------- gated merge datapath + FIFO write side
    wire [7:0]  wm, irqcfg;
    wire [15:0] thr, hyst;
    wire [47:0] gain, offs;
    wire [5:0]  perm_l, perm_h;
    wire [2:0]  sign_l, sign_h;
    wire        dp_busy, fifo_wr, fifo_full, drop;
    wire [79:0] fifo_wdata;
    wire [2:0]  st;
    wire        gclk_dp;

    clk_gate u_g_dp (.clk(clk), .en(l_valid | h_valid | dp_busy | reconfig), .gclk(gclk_dp));

    merge_dp u_dp (
        .gclk(gclk_dp), .rst_n(rst_n), .clear(reconfig), .mode(mode),
        .thr(thr), .hyst(hyst), .gain(gain), .offs(offs),
        .perm_l(perm_l), .perm_h(perm_h), .sign_l(sign_l), .sign_h(sign_h),
        .l_valid(l_valid), .h_valid(h_valid), .rx(rxbuf), .l_ts(l_ts), .h_ts(h_ts),
        .busy(dp_busy), .wr(fifo_wr), .wdata(fifo_wdata), .full(fifo_full),
        .drop(drop), .st(st));

    // read pointer from the host domain, synchronised in the always-on domain
    wire [FIFO_AW:0] rgray, wgray, wbin;
    reg  [FIFO_AW:0] rq1, rq2;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin rq1 <= 0; rq2 <= 0; end else begin rq1 <= rgray; rq2 <= rq1; end

    wire [79:0] rd0, rd1;
    wire [FIFO_AW-1:0] ra0, ra1;

    async_fifo_wr #(.W(80), .AW(FIFO_AW)) u_fifo (
        .wclk(gclk_dp), .rst_n(rst_n), .clear(1'b0), .wr(fifo_wr), .wdata(fifo_wdata),
        .rq_gray(rq2), .full(fifo_full), .wbin(wbin), .wgray(wgray),
        .rd0(rd0), .ra0(ra0), .rd1(rd1), .ra1(ra1));

    // level and IRQ in the always-on domain
    reg [FIFO_AW:0] rbin_c;
    integer i;
    always @(*) begin
        rbin_c[FIFO_AW] = rq2[FIFO_AW];
        for (i = FIFO_AW - 1; i >= 0; i = i - 1) rbin_c[i] = rbin_c[i+1] ^ rq2[i];
    end
    wire [FIFO_AW:0] level_c = wbin - rbin_c;

    reg overflow;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) overflow <= 1'b0;
        else if (drop) overflow <= 1'b1;
        else if (oclr_upd) overflow <= 1'b0;

    wire running = enable & cfg_done;
    wire irq = (irqcfg[0] && wm != 0 && {{(7-FIFO_AW){1'b0}}, level_c} >= wm)
             | (irqcfg[1] && |st);

    // ---------------- host domain
    wire host_miso;
    host_spi #(.AW(FIFO_AW)) u_host (
        .sck(host_sck), .mosi(host_mosi), .cs_n(host_cs_n), .rst_n(rst_n), .miso(host_miso),
        .ctrl(ctrl_h), .wm(wm), .irqcfg(irqcfg), .thr(thr), .hyst(hyst),
        .gain(gain), .offs(offs), .perm_l(perm_l), .sign_l(sign_l),
        .perm_h(perm_h), .sign_h(sign_h),
        .cfg_tgl(cfg_tgl), .snap_tgl(snap_tgl), .oclr_tgl(oclr_tgl),
        .status_async({running, overflow, ok_h, ok_l}), .time_hold(time_hold),
        .wgray(wgray), .rgray(rgray), .ra0(ra0), .ra1(ra1), .rd0(rd0), .rd1(rd1));

    assign uo_out  = {running, |st, irq, host_miso, cs_h_n, cs_l_n, mosi_o, sck_o};
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;
    wire _unused = &{ena, uio_in, ui_in[7:6], spi_busy, 1'b0};
endmodule
