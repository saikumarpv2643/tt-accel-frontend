// SPI master, mode 3 (CPOL=1, CPHA=1), MSB first, one byte per start pulse.
// SCK idles high. MOSI changes on the falling edge, MISO is sampled on the
// rising edge, as the H3LIS331DL expects (datasheet: SPI read/write timing).
// SCK frequency = clk / (2 * DIV).
`default_nettype none

module spi_master #(
    parameter integer DIV = 4
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,      // 1-cycle pulse, ignored while busy
    input  wire [7:0] tx_byte,
    output reg  [7:0] rx_byte,
    output reg        done,       // 1-cycle pulse when rx_byte is valid
    output wire       busy,
    output reg        sck,
    output reg        mosi,
    input  wire       miso
);
    localparam integer CW = (DIV > 1) ? $clog2(DIV) : 1;

    reg [CW-1:0] div_cnt;
    reg [2:0]    bit_cnt;
    reg [7:0]    tx_sh;
    reg [7:0]    rx_sh;
    reg          active;

    assign busy = active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck     <= 1'b1;
            mosi    <= 1'b1;
            active  <= 1'b0;
            done    <= 1'b0;
            div_cnt <= {CW{1'b0}};
            bit_cnt <= 3'd0;
            tx_sh   <= 8'd0;
            rx_sh   <= 8'd0;
            rx_byte <= 8'd0;
        end else begin
            done <= 1'b0;
            if (!active) begin
                if (start) begin
                    active  <= 1'b1;
                    tx_sh   <= tx_byte;
                    bit_cnt <= 3'd0;
                    div_cnt <= {CW{1'b0}};
                end
            end else if (div_cnt == DIV - 1) begin
                div_cnt <= {CW{1'b0}};
                if (sck) begin
                    // falling edge: present the next bit
                    sck   <= 1'b0;
                    mosi  <= tx_sh[7];
                    tx_sh <= {tx_sh[6:0], 1'b0};
                end else begin
                    // rising edge: sample MISO
                    sck   <= 1'b1;
                    rx_sh <= {rx_sh[6:0], miso};
                    if (bit_cnt == 3'd7) begin
                        active  <= 1'b0;
                        done    <= 1'b1;
                        rx_byte <= {rx_sh[6:0], miso};
                    end
                    bit_cnt <= bit_cnt + 3'd1;
                end
            end else begin
                div_cnt <= div_cnt + 1'b1;
            end
        end
    end
endmodule
