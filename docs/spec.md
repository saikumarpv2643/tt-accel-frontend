# accel-asic v2 — low-power dual-sensor front-end (spec draft, M1)

Status: **RTL implemented, 2026-11-01** (decisions: 1 kHz records, 2x2+ tiles OK, 16-bit timestamps). 9/9 cocotb tests pass; not hardened, not on hardware. Items marked
**[verify]** were written from memory. On 2026-11-04 the register addresses,
ID, bit fields and sensitivity were checked against ST's own driver
(`iis2dlpc_reg.h/.c` in the nRF Connect SDK) and match. The PDF datasheet is
still not in the repo, so timing and the 1600 Hz data rate's bandwidth are not checked.

## 1. What it does

Reads an IIS2DLPC (±16 g, low-g) and an H3LIS331DL (±400 g, high-g) on their
data-ready pins, stamps each sample with a hardware timer, merges them per
axis into one 20-bit ±400 g channel, and buffers the records for a host that
wakes only at a FIFO watermark. Every block except a small always-on part is
clock-gated.

## 2. Clocks and power domains

| Domain | Clock | Runs when | Contents |
|---|---|---|---|
| Always-on | `clk`, 1 MHz | always (stopped when CTRL.enable = 0) | DRDY synchronisers, 32-bit timestamp counter, scheduler FSM, IRQ |
| Sensor SPI | gated `clk` | during a sensor read (~0.12 ms per read) | SPI master, byte sequencer, read buffers |
| Datapath | gated `clk` | a few cycles per sample | axis map, gain/offset, merge, FIFO write |
| Host | host `SCK` | only while the host clocks a transfer | SPI slave, register file, FIFO read side |

- Clock gates: one wrapper `clk_gate.v` — a latch-based behavioural model for
  simulation, `sky130_fd_sc_hd__dlclkp` for the ASIC, and a plain enable on FPGA.
- Crossings: FIFO = asynchronous FIFO with Gray-code pointers (core → host).
  Settings = host-domain registers, copied into core shadow registers by a
  toggle handshake raised on the last bit of a write. Status bits = 2-flop
  synchronisers. Every crossing is listed in §8.
- Sensor SCK = clk / 2 = 500 kHz (both sensors allow up to 10 MHz).

## 3. Pins (Tiny Tapeout)

| Pin | Signal | Pin | Signal |
|---|---|---|---|
| ui[0] | S_MISO (shared sensor SDO) | uo[0] | S_SCK |
| ui[1] | DRDY_L (IIS2DLPC INT1) | uo[1] | S_MOSI |
| ui[2] | DRDY_H (H3LIS331DL INT1) | uo[2] | CS_L_N |
| ui[3] | HOST_SCK (second clock) | uo[3] | CS_H_N |
| ui[4] | HOST_MOSI | uo[4] | HOST_MISO |
| ui[5] | HOST_CS_N | uo[5] | IRQ |
| | | uo[6] | SAT (an axis is on the high-g sensor) |
| | | uo[7] | RUNNING |

The sensors share SCK/MOSI/MISO; their SDO pins are high-impedance while CS
is high.

## 4. Sensor setup

- **H3LIS331DL** (as v1): WHO_AM_I 0x0F = 0x32; CTRL_REG1 = 0x3F (normal,
  1000 Hz); CTRL_REG3 = 0x02 (DRDY on INT1); CTRL_REG4 = BDU | FS; read 0xE8
  + 6 bytes; 12-bit data left-justified.
- **IIS2DLPC [verify]**: WHO_AM_I 0x0F = 0x44; CTRL1 0x20 = 0x94 (1600 Hz,
  high-performance); CTRL2 0x21 = 0x0E (BDU, auto-increment, I²C off);
  CTRL4_INT1 0x23 = 0x01 (DRDY on INT1); CTRL6 0x25 = 0x34 (±16 g, low
  noise); read 0xA8 + 6 bytes (no MS bit; auto-increment from CTRL2); 14-bit
  data left-justified, 1.952 mg/LSB at ±16 g.

## 5. Merge (per axis)

```
l = low-g value >> 2          (14-bit, 1.952 mg/LSB)
h = high-g value >> 4         (12-bit, 195 mg/LSB at ±400 g)
h' = (h * GAIN) >> 8 + OFFSET (GAIN unsigned Q8.8, default 100.0; OFFSET signed, LSB = 1.952 mg)
state = HIGH if |l| >= THR; LOW if |l| < THR - HYST   (defaults 7800 / 800)
out = (state == LOW) ? l : h'            (20-bit signed, 1.952 mg/LSB, ±1023 g)
```

- Axis map per sensor: 3-way permutation + 3 sign bits, applied before the merge.
- Records are paced by the H3LIS331DL's DRDY: **1000 records/s**, timestamp =
  high-g DRDY time. The IIS2DLPC runs at 1600 Hz (it has no 1000 Hz rate
  [verify]); each record uses the latest low-g sample, up to 0.625 ms old.
- Modes (CTRL[2:1]): 00 merged (default), 01 low-g only (records at 1600 Hz,
  low-g timestamps), 10 high-g only.

## 6. Record (80 bits, 10 bytes, little-endian on the host port)

| Bits | Field |
|---|---|
| 15:0 | timestamp, µs, low 16 bits of the 32-bit counter |
| 35:16 / 55:36 / 75:56 | X / Y / Z, 20-bit signed |
| 78:76 | src per axis (1 = high-g) |
| 79 | gap: records were dropped before this one (FIFO overflow) |

FIFO depth: parameter, 8 on Tiny Tapeout, 32–64 on a whole die.

## 7. Host registers (SPI mode 0, `{R/W, addr[6:0]}`, auto-increment)

| Addr | Name | Notes |
|---|---|---|
| 0x00 | ID | 0xA6 |
| 0x01 | CTRL | [0] enable [2:1] mode [4:3] high-g range |
| 0x02 | STATUS | [0] L ok [1] H ok [2] overflow (clear on read) [3] running |
| 0x03 | LEVEL | FIFO records available |
| 0x04 | WATERMARK | IRQ when LEVEL ≥ value; 0 = off |
| 0x05 | IRQ_CFG | [0] IRQ on watermark [1] IRQ at once on a switch to high-g |
| 0x06–0x09 | THR, HYST | 16-bit each |
| 0x0A–0x0F | GAIN X/Y/Z | Q8.8 |
| 0x10–0x15 | OFFSET X/Y/Z | signed 16-bit |
| 0x16 / 0x17 | PERM_L / SIGN_L | output axis k takes input axis PERM[2k+1:2k]; SIGN[k] negates it |
| 0x18 / 0x19 | PERM_H / SIGN_H | same for the high-g sensor |
| 0x1C–0x1F | TIME | write 0x1C (any value) to snapshot the 32-bit counter; read it in a later frame |
| 0x20 | FIFO | record stream, 10 bytes each |

## 8. Clock-domain crossings

| Signal | From → to | Method |
|---|---|---|
| DRDY_L, DRDY_H | pin → core | 2-flop sync, edge detect |
| FIFO data | core → host | async FIFO, Gray pointers |
| settings | host → core | toggle handshake + shadow registers |
| overflow clear | host → core | toggle handshake |
| status bits | core → host | 2-flop sync (quasi-static) |

## 9. Verification plan

- Python reference: merge, gain/offset, axis map, record packing (bit-exact).
- cocotb: two sensor models with independent DRDY rates and jitter; host model
  with SCK unrelated to `clk` (e.g. 4.1 MHz vs 1 MHz).
- Tests: boot both sensors; wrong ID retry; timestamps vs model times (±1 µs);
  merge switch with hysteresis at the threshold; gain/offset; axis maps; modes;
  watermark IRQ; overflow and gap flag; host reads at random SCK rates.
- Formal (SymbiYosys): async FIFO never loses/duplicates data and never reads
  when empty; handshake never drops a settings update.
- Power: OpenSTA `report_power` with VCD activity from a 1 kHz run, clock gates
  on vs forced on.

## 10. As built: rules and limits

- Settings other than CTRL are quasi-static: the core reads them directly, so
  change gain, offset, thresholds or axis maps while streaming only if one
  odd record is acceptable. CTRL is copied into the core by a handshake and
  restarts the sensors and clears the merge state; the FIFO is not cleared.
- LEVEL and STATUS seen by the host can be a few records stale (never more
  records than really present).
- Boot retries until both sensors answer, in every mode.
- Size (generic Yosys): 6560 cells, 1402 flip-flops/latches (FIFO 640). Likely
  more than 2x2 Tiny Tapeout tiles: hardening will tell; FIFO_AW=2 halves the FIFO.
- Gating measured in simulation: SPI master clocked 34.6 % of cycles,
  datapath 0.7 % (nobody reading, so the FIFO is full and dropping records).
- Any signal on a gated clock that must return to 0 has to hold its clock gate
  open until it does. `drop` missed this once (fixed 2026-11-04): see README.

## 11. Open questions

1. Clock source in a real product (always-on 1 MHz oscillator vs on-demand ring
   oscillator) — v3.
2. IIS2DLPC at 1600 Hz vs 800 Hz (lower power, less bandwidth).
3. FIFO depth on Tiny Tapeout: 8 records = 640 flip-flops; may need 2x2+ tiles.
4. Should records carry the full 32-bit timestamp instead of 16 bits (+2 bytes)?
