## How it works

The chip sits between an ST H3LIS331DL accelerometer (±100/200/400 g) and a
host microcontroller, so the host no longer has to service the sensor at
1 kHz.

1. **Sensor SPI master** (mode 3, SCK = clk/8). After reset the chip waits
   5 ms, reads WHO_AM_I (must be 0x32, otherwise it retries), then writes
   CTRL_REG1..4: normal mode, 1000 Hz, X/Y/Z on, data-ready on INT1, block
   data update, and the range from the CTRL register.
2. Each time INT1 (data ready) goes high it reads the six output registers
   in one burst.
3. **CIC decimator**, order 2, decimation 1/2/4/8/16. The three axes share one
   adder through a small sequencer (one add per clock), since samples arrive
   at 1 kHz and the clock is 10 MHz. Output is normalised by the exact
   R^2 gain.
4. **FIFO**: 8 samples of X/Y/Z (48 bits). A sample that arrives while the
   FIFO is full is dropped and sets OVERFLOW.
5. **Host SPI slave** (mode 0, SCK at most clk/8). Command byte
   `{R/W, addr[6:0]}` (1 = read), auto-increment.

| Addr | Name      | Access | Bits |
|------|-----------|--------|------|
| 0x00 | ID        | R  | 0xA5 |
| 0x01 | CTRL      | RW | [0] enable, [2:1] range (00 100 g, 01 200 g, 11 400 g), [6:4] log2 decimation (0..4). Reset 0x01. A write reconfigures the sensor and clears the filter and FIFO |
| 0x02 | STATUS    | R  | [0] sensor OK, [1] overflow (cleared by reading), [2] running, [7:4] FIFO level |
| 0x03 | IRQ_THR   | RW | IRQ high while FIFO level >= value (0 = off) |
| 0x04 | WHOAMI    | R  | last WHO_AM_I value read from the sensor |
| 0x10 | FIFO_DATA | R  | X_L X_H Y_L Y_H Z_L Z_H, then the next sample. An entry is removed only after its sixth byte has been clocked out |

## How to test

1. Wire an H3LIS331DL breakout (3.3 V, SPI mode: CS jumper cut) to
   SENS_SCK/MOSI/MISO/CS_N and INT1 to SENS_DRDY.
2. After reset SENSOR_OK and RUNNING go high.
3. From the host, read STATUS (`0x82 0x00`) to get the FIFO level, then read
   `0x90` followed by 6 × level dummy bytes to get the samples.
4. Write `0x01 0x41` to select decimation by 16 (62.5 Hz output).

## External hardware

H3LIS331DL breakout (e.g. SparkFun SEN-14480) and any SPI host (RP2040 on the
Tiny Tapeout demo board, Raspberry Pi, nRF5340 DK).
