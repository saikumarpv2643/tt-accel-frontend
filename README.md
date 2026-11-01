![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# Accelerometer Interface (H3LIS331DL) — Tiny Tapeout

A small digital chip that reads an ST H3LIS331DL high-g accelerometer over SPI,
filters and decimates the X/Y/Z data with a CIC filter, buffers it in a FIFO
and hands it to a host microcontroller over an SPI slave port.

- [Datasheet: how it works, register map, how to test](docs/info.md)
- RTL in `src/`, cocotb tests in `test/` (behavioural sensor model and a
  bit-exact filter reference)

Made from the [Tiny Tapeout](https://tinytapeout.com) Verilog template
(`ttsky-verilog-template`), SkyWater 130 nm.
