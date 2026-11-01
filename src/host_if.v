// Host interface: SPI slave, mode 0 (CPOL=0, CPHA=0), MSB first.
//
// Frame: CS low, command byte {R/W, addr[6:0]} (1 = read), then data bytes.
// Register reads and writes auto-increment the address. Reading address
// 0x10 (FIFO_DATA) streams samples: X_L X_H Y_L Y_H Z_L Z_H, then the next
// sample. A FIFO entry is popped only after its sixth byte has been clocked
// out, so a prefetch at the end of a frame never loses a sample.
//
// SCK, MOSI and CS are synchronised into clk; host SCK must be at most clk/8.
//
// Registers
//   0x00 ID        RO  0xA5
//   0x01 CTRL      RW  [0] enable  [2:1] range (00 100 g, 01 200 g, 11 400 g)
//                      [6:4] log2 decimation (0..4)   reset 0x01
//   0x02 STATUS    RO  [0] sensor ok  [1] overflow (cleared when read)
//                      [2] running   [7:4] FIFO level
//   0x03 IRQ_THR   RW  [3:0] IRQ when FIFO level >= value (0 = off)
//   0x04 WHOAMI    RO  last WHO_AM_I byte read from the sensor
//   0x10 FIFO_DATA RO  sample stream
`default_nettype none

module host_if (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sck_in,
    input  wire        mosi_in,
    input  wire        cs_n_in,
    output reg         miso,
    output reg  [7:0]  ctrl,
    output reg  [3:0]  irq_thr,
    output reg         ctrl_wr,     // pulse: CTRL was written
    input  wire [7:0]  status,
    input  wire [7:0]  whoami,
    input  wire [47:0] fifo_rdata,
    input  wire        fifo_empty,
    output reg         fifo_pop,
    output reg         status_rd    // pulse: STATUS byte was clocked out
);
    localparam [6:0] A_ID = 7'h00, A_CTRL = 7'h01, A_STATUS = 7'h02,
                     A_THR = 7'h03, A_WHO = 7'h04, A_FIFO = 7'h10;

    // synchronisers
    reg [2:0] sck_r;
    reg [1:0] mosi_r, cs_r;
    wire cs_n = cs_r[1];
    wire rise = (sck_r[2:1] == 2'b01);
    wire fall = (sck_r[2:1] == 2'b10);

    reg [2:0]  bit_cnt;
    reg [7:0]  rx_sh, tx_sh;
    reg        first;       // next completed byte is the command
    reg        rw;
    reg [6:0]  addr;        // address of the byte being shifted out / written
    reg [2:0]  fptr;        // byte index inside a FIFO sample
    reg        word_ok;     // the FIFO head is being sent (pop it when done);
                            // the head stays put until popped, so no copy is kept
    reg        load_req;    // byte completed: fetch the next one
    reg        load_go;     // one clock later, after a FIFO pop has taken effect

    wire [7:0] rx_byte = {rx_sh[6:0], mosi_r[1]};

    reg [7:0] reg_rd;
    always @(*) begin
        case (addr)
            A_ID:     reg_rd = 8'hA5;
            A_CTRL:   reg_rd = ctrl;
            A_STATUS: reg_rd = status;
            A_THR:    reg_rd = {4'd0, irq_thr};
            A_WHO:    reg_rd = whoami;
            default:  reg_rd = 8'h00;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_r     <= 3'b000;
            mosi_r    <= 2'b00;
            cs_r      <= 2'b11;
            miso      <= 1'b0;
            ctrl      <= 8'h01;
            irq_thr   <= 4'd0;
            ctrl_wr   <= 1'b0;
            fifo_pop  <= 1'b0;
            status_rd <= 1'b0;
            bit_cnt   <= 3'd0;
            rx_sh     <= 8'd0;
            tx_sh     <= 8'd0;
            first     <= 1'b1;
            rw        <= 1'b0;
            addr      <= 7'd0;
            fptr      <= 3'd0;
            word_ok   <= 1'b0;
            load_req  <= 1'b0;
            load_go   <= 1'b0;
        end else begin
            sck_r     <= {sck_r[1:0], sck_in};
            mosi_r    <= {mosi_r[0], mosi_in};
            cs_r      <= {cs_r[0], cs_n_in};
            ctrl_wr   <= 1'b0;
            fifo_pop  <= 1'b0;
            status_rd <= 1'b0;
            load_req  <= 1'b0;
            load_go   <= load_req;

            if (cs_n) begin
                bit_cnt <= 3'd0;
                first   <= 1'b1;
                tx_sh   <= 8'd0;
                word_ok <= 1'b0;
            end else begin
                // two clocks after a byte completes: fetch the next byte to send
                if (load_go) begin
                    if (addr == A_FIFO) begin
                        if (fptr == 3'd0) word_ok <= !fifo_empty;
                        if (fptr == 3'd0 && fifo_empty) tx_sh <= 8'd0;
                        else if (fptr != 3'd0 && !word_ok) tx_sh <= 8'd0;
                        else tx_sh <= fifo_rdata[fptr*8 +: 8];
                    end else begin
                        tx_sh <= reg_rd;
                    end
                end

                if (fall) begin
                    miso  <= tx_sh[7];
                    tx_sh <= {tx_sh[6:0], 1'b0};
                end

                if (rise) begin
                    rx_sh   <= rx_byte;
                    bit_cnt <= bit_cnt + 3'd1;
                    if (bit_cnt == 3'd7) begin
                        first <= 1'b0;
                        if (first) begin
                            rw   <= rx_byte[7];
                            addr <= rx_byte[6:0];
                            fptr <= 3'd0;
                            if (rx_byte[7]) load_req <= 1'b1;
                        end else if (rw) begin
                            // a read byte has just been clocked out
                            if (addr == A_FIFO) begin
                                if (fptr == 3'd5) begin
                                    fifo_pop <= word_ok;
                                    word_ok  <= 1'b0;
                                    fptr     <= 3'd0;
                                end else begin
                                    fptr <= fptr + 3'd1;
                                end
                            end else begin
                                if (addr == A_STATUS) status_rd <= 1'b1;
                                addr <= addr + 7'd1;
                            end
                            load_req <= 1'b1;
                        end else begin
                            // write
                            case (addr)
                                A_CTRL: begin
                                    ctrl    <= rx_byte;
                                    ctrl_wr <= 1'b1;
                                end
                                A_THR:  irq_thr <= rx_byte[3:0];
                                default: ;
                            endcase
                            addr <= addr + 7'd1;
                        end
                    end
                end
            end
        end
    end
endmodule
