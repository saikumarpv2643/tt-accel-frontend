"""Models for the v2 tests.

Sensor     - SPI slave (mode 3) for an H3LIS331DL ('H') or IIS2DLPC ('L'),
             with its own DRDY rate, block data update, and a log of every
             sample read by the design, in bus order.
HostSPI    - mode 0 master on its own clock, unrelated to the chip clock.
reference  - bit-exact replay of merge_dp.v over the logged reads.
"""
import math

from cocotb.triggers import FallingEdge, First, RisingEdge, Timer
from cocotb.utils import get_sim_time

LSB_L = 0.001952    # g per 14-bit count, IIS2DLPC +-16 g
LSB_H = 0.195       # g per 12-bit count, H3LIS331DL +-400 g


def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


class Sensor:
    def __init__(self, dut, kind, period_ns, accel, bus_log, phase_ns=0):
        self.dut, self.kind = dut, kind
        self.cs = dut.cs_l_n if kind == "L" else dut.cs_h_n
        self.miso = dut.miso_l if kind == "L" else dut.miso_h
        self.drdy = dut.drdy_l if kind == "L" else dut.drdy_h
        self.whoami = 0x44 if kind == "L" else 0x32
        self.period_ns, self.phase_ns = period_ns, phase_ns
        self.accel = accel            # f(t_ns) -> (gx, gy, gz)
        self.bus_log = bus_log        # shared list of (kind, raw16 tuple, drdy_time_ns)
        self.regs = [0] * 0x40
        self.out_raw = (0, 0, 0)
        self.out_time = 0
        self.drdy_times = []
        self.miso.value = 1
        self.drdy.value = 0

    def active(self):
        if self.kind == "H":
            return (self.regs[0x20] >> 5) == 1
        return (self.regs[0x20] >> 4) != 0

    def raw(self, t):
        out = []
        for a in self.accel(t):
            if self.kind == "L":
                c = max(-8192, min(8191, round(a / LSB_L)))
                out.append((c << 2) & 0xFFFF)
            else:
                c = max(-2048, min(2047, round(a / LSB_H)))
                out.append((c << 4) & 0xFFFF)
        return tuple(out)

    def _read(self, a):
        if a == 0x0F:
            return self.whoami
        if 0x28 <= a <= 0x2D:
            v = self.out_raw[(a - 0x28) // 2]
            return (v >> 8) & 0xFF if (a - 0x28) % 2 else v & 0xFF
        return self.regs[a]

    async def spi(self):
        dut = self.dut
        while True:
            await FallingEdge(self.cs)
            nbits = cur = idx = rw = ms = addr = out = 0
            while True:
                await First(FallingEdge(dut.s_sck), RisingEdge(self.cs))
                if self.cs.value == 1:
                    break
                if idx >= 1 and rw:
                    self.miso.value = (out >> (7 - nbits)) & 1
                await RisingEdge(dut.s_sck)
                cur = ((cur << 1) | int(dut.s_mosi.value)) & 0xFF
                nbits += 1
                if nbits < 8:
                    continue
                nbits = 0
                if idx == 0:
                    rw = cur >> 7
                    if self.kind == "H":
                        ms, addr = (cur >> 6) & 1, cur & 0x3F
                    else:                          # IIS2DLPC: auto-increment always
                        ms, addr = 1, cur & 0x7F
                    if rw:
                        out = self._read(addr)
                elif rw:
                    if addr == 0x2D:
                        self.drdy.value = 0
                        self.bus_log.append((self.kind, self.out_raw, self.out_time))
                    if ms:
                        addr += 1
                    out = self._read(addr & 0x3F)
                else:
                    self.regs[addr & 0x3F] = cur
                    if ms:
                        addr += 1
                idx += 1
            self.miso.value = 1

    async def producer(self):
        if self.phase_ns:
            await Timer(self.phase_ns, "ns")
        t = self.phase_ns
        while True:
            await Timer(self.period_ns, "ns")
            t += self.period_ns
            if not self.active():
                continue
            while self.cs.value == 0:              # block data update
                await RisingEdge(self.cs)
            now = get_sim_time("ns")                 # absolute DRDY time
            self.out_raw = self.raw(now)
            self.out_time = now
            self.drdy_times.append(now)
            self.drdy.value = 1


class HostSPI:
    def __init__(self, dut, half_ns=137):
        self.dut, self.half = dut, half_ns
        dut.host_sck.value = 0
        dut.host_mosi.value = 0
        dut.host_cs_n.value = 1

    async def xfer(self, data):
        dut, h = self.dut, self.half
        dut.host_cs_n.value = 0
        await Timer(h, "ns")
        rx = []
        for b in data:
            r = 0
            for i in range(8):
                dut.host_mosi.value = (b >> (7 - i)) & 1
                await Timer(h, "ns")
                dut.host_sck.value = 1
                r = (r << 1) | int(dut.host_miso.value)
                await Timer(h, "ns")
                dut.host_sck.value = 0
            rx.append(r)
        await Timer(h, "ns")
        dut.host_cs_n.value = 1
        await Timer(2 * h, "ns")
        return rx

    async def read(self, addr, n=1):
        return (await self.xfer([0x80 | addr] + [0] * n))[1:]

    async def write(self, addr, *vals):
        await self.xfer([addr & 0x7F] + [v & 0xFF for v in vals])

    async def records(self, k):
        r = await self.read(0x20, 10 * k)
        recs = []
        for j in range(k):
            v = int.from_bytes(bytes(r[10 * j:10 * j + 10]), "little")
            axes = []
            for a in range(3):
                x = (v >> (16 + 20 * a)) & 0xFFFFF
                axes.append(x - (1 << 20) if x >> 19 else x)
            recs.append({"ts": v & 0xFFFF, "axes": tuple(axes),
                         "src": (v >> 76) & 7, "gap": v >> 79})
        return recs


def reference(bus_log, mode=0, thr=7800, hyst=800, gain=(25600,) * 3,
              offs=(0, 0, 0), perm_l=(0, 1, 2), perm_h=(0, 1, 2),
              sign_l=(0, 0, 0), sign_h=(0, 0, 0)):
    """Bit-exact replay of merge_dp.v. Returns [(axes, src, drdy_time_ns)]."""
    def mapv(raw, perm, sign):
        out = []
        for k in range(3):
            v = s16(raw[perm[k]])
            if sign[k]:
                v = 32767 if v == -32768 else -v
            out.append(v)
        return out

    l_hold, h_hold, st, recs = (0, 0, 0), (0, 0, 0), [0, 0, 0], []
    for kind, raw, t in bus_log:
        if kind == "L":
            l_hold = raw
            if mode != 1:
                continue
        else:
            h_hold = raw
            if mode == 1:
                continue
        lm, hm = mapv(l_hold, perm_l, sign_l), mapv(h_hold, perm_h, sign_h)
        axes, src = [], 0
        for k in range(3):
            l14 = lm[k] >> 2
            h12 = hm[k] >> 4
            hs = ((h12 * gain[k]) >> 8) + offs[k]
            hs = max(-524288, min(524287, hs))
            a = abs(l14)
            lo = thr - hyst if thr > hyst else 0
            new = 1 if a >= thr else 0 if a < lo else st[k]
            sel = 1 if mode == 2 else 0 if mode == 1 else new
            if mode == 0:
                st[k] = new
            axes.append(hs if sel else l14)
            src |= sel << k
        recs.append((tuple(axes), src, t))
    return recs


def shock(t_ns, t0_ns, peak_g, width_ns, rest=(0.0, 0.0, 1.0)):
    """Half-sine shock on X on top of a resting 1 g on Z."""
    x = rest[0]
    if t0_ns <= t_ns <= t0_ns + width_ns:
        x += peak_g * math.sin(math.pi * (t_ns - t0_ns) / width_ns)
    return (x, rest[1] + 0.3 * math.sin(t_ns * 2e-6), rest[2])
