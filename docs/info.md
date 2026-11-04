## How it works

The chip sits between two accelerometers and a host microcontroller, so the host does not have to wake for every sample. It reads an ST IIS2DLPC (±16 g) and an ST H3LIS331DL (±400 g) on their data-ready pins, stamps each sample with a hardware timer, and merges them per axis into one 20-bit stream at 1000 records per second. An axis uses the low-g sensor (1.952 mg per count) until it nears full scale, then switches with hysteresis to the high-g sensor (about 195 mg per count).

Records wait in an asynchronous Gray-code FIFO (8 records). The host reads them over an SPI port clocked by its own SCK, and an IRQ pin can signal a FIFO watermark. The SPI master and datapath are clock-gated. Full details are in [spec.md](spec.md).

## How to test

Connect both sensors, supply a 1 MHz clock, and release reset. RUNNING goes high once both sensors answer with the correct ID and are configured. Then read the FIFO level and records through the host SPI port; the register map is in [spec.md](spec.md).

The cocotb tests in `test/` run the design against sensor models on independent clocks and a bit-exact Python reference.

## External hardware

- ST IIS2DLPC and ST H3LIS331DL accelerometers on a shared SPI bus, each with its own chip select and data-ready line
- A host microcontroller with an SPI master
