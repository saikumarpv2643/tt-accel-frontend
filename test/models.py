"""Behavioural models for the cocotb tests.

H3LIS331DL  - SPI slave, mode 3, register file, data-ready on INT1, BDU.
HostSPI     - SPI master, mode 0, driving the design's host interface.
cic_ref     - bit-exact reference of the CIC decimator.
"""
import random

from cocotb.triggers import ClockCycles, FallingEdge, First, RisingEdge

OUT_X_L = 0x28
OUT_Z_H = 0x2D


class H3LIS331DL:
    def __init__(self, dut, whoami=0x32, period=1000, seed=1):
        self.dut = dut
        self.whoami = whoami
        self.period = period          # clock cycles between samples (ODR)
        self.regs = [0] * 0x40
        self.regs[0x20] = 0x07        # CTRL_REG1 reset value: power-down
        self.rng = random.Random(seed)
        self.source = None            # optional callable -> (x, y, z) raw 16-bit
        self.produced = []            # every sample made available, in order
        self.overruns = 0             # samples overwritten before being read
        self.pause = False
        self.transactions = 0
        dut.sens_miso.value = 0
        dut.sens_drdy.value = 0

    def _read(self, addr):
        return self.whoami if addr == 0x0F else self.regs[addr]

    @property
    def active(self):
        return (self.regs[0x20] >> 5) == 0b001   # normal power mode

    async def spi(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.sens_cs_n)
            self.transactions += 1
            nbits = cur = byte_idx = rw = ms = addr = out = 0
            while True:
                await First(FallingEdge(dut.sens_sck), RisingEdge(dut.sens_cs_n))
                if dut.sens_cs_n.value == 1:
                    break
                if byte_idx >= 1 and rw:
                    dut.sens_miso.value = (out >> (7 - nbits)) & 1
                await RisingEdge(dut.sens_sck)
                cur = ((cur << 1) | int(dut.sens_mosi.value)) & 0xFF
                nbits += 1
                if nbits < 8:
                    continue
                nbits = 0
                if byte_idx == 0:
                    rw, ms, addr = cur >> 7, (cur >> 6) & 1, cur & 0x3F
                    if rw:
                        out = self._read(addr)
                elif rw:
                    if addr == OUT_Z_H:
                        dut.sens_drdy.value = 0
                    if ms:
                        addr = (addr + 1) & 0x3F
                    out = self._read(addr)
                else:
                    self.regs[addr] = cur
                    if ms:
                        addr = (addr + 1) & 0x3F
                byte_idx += 1

    def _next_sample(self):
        if self.source is not None:
            return self.source()
        # 12-bit samples, left-justified in 16 bits as the sensor outputs them
        return tuple(self.rng.randint(-2048, 2047) << 4 for _ in range(3))

    async def producer(self):
        dut = self.dut
        while True:
            await ClockCycles(dut.clk, self.period)
            if self.pause or not self.active:
                continue
            # block data update: never change OUT registers mid-read
            while dut.sens_cs_n.value == 0:
                await RisingEdge(dut.sens_cs_n)
            if dut.sens_drdy.value == 1:
                self.overruns += 1
            s = self._next_sample()
            for i, v in enumerate(s):
                u = v & 0xFFFF
                self.regs[OUT_X_L + 2 * i] = u & 0xFF
                self.regs[OUT_X_L + 2 * i + 1] = u >> 8
            self.produced.append(s)
            dut.sens_drdy.value = 1


class HostSPI:
    def __init__(self, dut, half=8):
        self.dut = dut
        self.half = half              # SCK half period in clock cycles
        dut.host_sck.value = 0
        dut.host_mosi.value = 0
        dut.host_cs_n.value = 1

    async def xfer(self, data):
        dut, h = self.dut, self.half
        dut.host_cs_n.value = 0
        await ClockCycles(dut.clk, h)
        rx = []
        for b in data:
            r = 0
            for i in range(8):
                dut.host_mosi.value = (b >> (7 - i)) & 1
                await ClockCycles(dut.clk, h)
                dut.host_sck.value = 1
                r = (r << 1) | int(dut.host_miso.value)
                await ClockCycles(dut.clk, h)
                dut.host_sck.value = 0
            rx.append(r)
        await ClockCycles(dut.clk, h)
        dut.host_cs_n.value = 1
        await ClockCycles(dut.clk, 2 * h)
        return rx

    async def read(self, addr, n=1):
        r = await self.xfer([0x80 | addr] + [0] * n)
        return r[1:]

    async def write(self, addr, val):
        await self.xfer([addr & 0x7F, val & 0xFF])

    async def status(self):
        s = (await self.read(0x02))[0]
        return {"ok": s & 1, "ovf": (s >> 1) & 1, "run": (s >> 2) & 1, "level": s >> 4}

    async def samples(self, k):
        r = await self.read(0x10, 6 * k)
        out = []
        for j in range(k):
            b = r[6 * j:6 * j + 6]
            out.append(tuple(to_s16(b[2 * i] | (b[2 * i + 1] << 8)) for i in range(3)))
        return out


def to_s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _wrap(v, w):
    v &= (1 << w) - 1
    return v - (1 << w) if v >> (w - 1) else v


def cic_ref(xs, n=2, r_log2=0, rl=4):
    """Bit-exact model of cic_decim.v for one axis."""
    w = 16 + n * rl
    r = 1 << r_log2
    integ, dly, out, cnt = [0] * n, [0] * n, [], 0
    for x in xs:
        v = x
        for k in range(n):
            integ[k] = _wrap(integ[k] + v, w)
            v = integ[k]
        dec = cnt == r - 1
        cnt = 0 if dec else cnt + 1
        if dec:
            for k in range(n):
                new = _wrap(v - dly[k], w)
                dly[k] = v
                v = new
            out.append(to_s16(v >> (n * r_log2)))
    return out
