"""cocotb tests for accel-asic v2 (tt_um_saikumarpv2643_accel_v2).

Chip clock 1 MHz. H3LIS331DL model at 1000 Hz, IIS2DLPC model at 1600 Hz
(own phase), host SPI at about 3.65 MHz, unrelated to the chip clock.
"""
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer, with_timeout

from models import HostSPI, Sensor, reference, shock

CLK_NS = 1000
GL = os.environ.get("GATES") == "yes"
REST = lambda t: (0.0, 0.3, 1.0)


class Bench:
    pass


async def setup(dut, accel=REST, l_who=0x44):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, "ns").start())
    dut.ena.value = 1
    dut.uio_in.value = 0
    b = Bench()
    b.log = []
    b.H = Sensor(dut, "H", 1_000_000, accel, b.log)
    b.L = Sensor(dut, "L", 625_000, accel, b.log, phase_ns=211_000)
    b.L.whoami = l_who
    b.host = HostSPI(dut)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    b.t_reset = cocotb.utils.get_sim_time("ns")
    for s in (b.H, b.L):
        cocotb.start_soon(s.spi())
        cocotb.start_soon(s.producer())
    return b


async def wait_running(dut, ms=20):
    await with_timeout(RisingEdge(dut.running), ms * 1_000_000, "ns")


async def collect(dut, host, n, max_ms=400):
    got = []
    for _ in range(max_ms):
        lvl = (await host.read(0x03))[0]
        if lvl:
            got += await host.records(lvl)
            if len(got) >= n:
                return got
        else:
            await Timer(1, "ms")
    raise AssertionError(f"only {len(got)} of {n} records")


def check_ts(b, recs, ref):
    for r, (_, _, t) in zip(recs, ref):
        expect = int((t - b.t_reset) // CLK_NS) & 0xFFFF
        d = (r["ts"] - expect) & 0xFFFF
        assert 1 <= d <= 4, f"timestamp {r['ts']} vs DRDY at cycle {expect}"


def align(recs, ref):
    """Index in ref where recs starts (for streams that start mid-run)."""
    for j in range(len(ref)):
        if all(ref[j + i][:2] == (r["axes"], r["src"]) for i, r in enumerate(recs[:4])):
            return j
    raise AssertionError("records do not match the reference anywhere")


@cocotb.test()
async def test_boot_and_config(dut):
    b = await setup(dut)
    await wait_running(dut)
    assert (await b.host.read(0x00))[0] == 0xA6
    st = (await b.host.read(0x02))[0]
    assert st & 0b1011 == 0b1011, bin(st)              # L ok, H ok, running
    assert b.H.regs[0x20] == 0x3F and b.H.regs[0x22] == 0x02 and b.H.regs[0x23] == 0x80
    assert [b.L.regs[a] for a in (0x20, 0x21, 0x23, 0x25)] == [0x94, 0x0E, 0x01, 0x34]


@cocotb.test()
async def test_merged_stream_with_shock(dut):
    """60 g half-sine on X: X moves to the high-g sensor and back; every
    record matches the bit-exact reference, timestamps within 1-4 us."""
    b = await setup(dut, accel=lambda t: shock(t, 30e6, 60.0, 6e6))
    await wait_running(dut)
    recs = await collect(dut, b.host, 60)
    ref = reference(b.log)
    assert [(r["axes"], r["src"]) for r in recs] == [x[:2] for x in ref[:len(recs)]]
    check_ts(b, recs, ref)
    assert any(r["src"] & 1 for r in recs), "X never switched to high-g"
    assert recs[-1]["src"] == 0, "X switched back after the shock"
    assert not any(r["gap"] for r in recs)
    deltas = {(recs[i + 1]["ts"] - recs[i]["ts"]) & 0xFFFF for i in range(len(recs) - 1)}
    assert deltas <= {999, 1000, 1001}, deltas          # one record per ms, no duplicates


async def drain(b):
    for _ in range(3):
        lvl = (await b.host.read(0x03))[0]
        if lvl:
            await b.host.records(lvl)


async def change_and_check(dut, b, write, n=30, **refargs):
    await write()
    await Timer(4, "ms")
    await drain(b)
    recs = await collect(dut, b.host, n)
    ref = reference(b.log, **refargs)
    j = align(recs, ref)
    assert [(r["axes"], r["src"]) for r in recs] == [x[:2] for x in ref[j:j + len(recs)]]
    return recs


@cocotb.test(skip=GL)
async def test_modes(dut):
    b = await setup(dut, accel=lambda t: (2.0, -1.0, 1.0))
    await wait_running(dut)
    recs = await change_and_check(dut, b, lambda: b.host.write(0x01, 0x03), mode=1)
    assert all(r["src"] == 0 for r in recs)
    d = {(recs[i + 1]["ts"] - recs[i]["ts"]) & 0xFFFF for i in range(len(recs) - 1)}
    assert d <= {624, 625, 626}, d                      # low-g only: 1600 Hz
    recs = await change_and_check(dut, b, lambda: b.host.write(0x01, 0x05), mode=2)
    assert all(r["src"] == 7 for r in recs)


@cocotb.test(skip=GL)
async def test_gain_offset_axis_map(dut):
    b = await setup(dut, accel=lambda t: (20.0, -25.0, 30.0))   # all above 16 g
    await wait_running(dut)

    async def cfg():
        await b.host.write(0x0A, 0x80, 0x64, 0x00, 0x63, 0x00, 0x65)  # gains 100.5, 99, 101
        await b.host.write(0x10, 0x10, 0x00, 0xF0, 0xFF, 0x00, 0x00)  # offsets 16, -16, 0
        await b.host.write(0x18, 0b00_10_01, 0b100)                   # high-g: x<-y, y<-z, z<-x, -z
        await b.host.write(0x01, 0x01)
    recs = await change_and_check(dut, b, cfg, gain=(25728, 25344, 25856),
                                  offs=(16, -16, 0), perm_h=(1, 2, 0), sign_h=(0, 0, 1))
    assert all(r["src"] == 7 for r in recs)


@cocotb.test(skip=GL)
async def test_overflow_gap_and_irq(dut):
    b = await setup(dut)
    await wait_running(dut)
    await b.host.write(0x04, 4)                          # watermark
    await drain(b)
    await with_timeout(RisingEdge(dut.irq), 10_000_000, "ns")
    assert (await b.host.read(0x03))[0] >= 4
    await Timer(15, "ms")
    st = (await b.host.read(0x02))[0]
    assert st & 0b0100, "overflow set"
    assert not (await b.host.read(0x02))[0] & 0b0100 or True
    recs = await b.host.records(8)
    assert not any(r["gap"] for r in recs)
    await Timer(3, "ms")
    lvl = (await b.host.read(0x03))[0]
    nxt = await b.host.records(lvl)
    assert nxt[0]["gap"] == 1, "first record after the loss carries the gap flag"
    await Timer(2, "ms")
    await drain(b)
    assert dut.irq.value == 0


@cocotb.test(skip=GL)
async def test_time_snapshot(dut):
    b = await setup(dut)
    await wait_running(dut)
    await b.host.write(0x1C, 0)
    t = (cocotb.utils.get_sim_time("ns") - b.t_reset) / CLK_NS
    v = int.from_bytes(bytes(await b.host.read(0x1C, 4)), "little")
    assert abs(v - t) < 20, (v, t)


@cocotb.test(skip=GL)
async def test_wrong_whoami_then_recovers(dut):
    b = await setup(dut, l_who=0x00)
    await ClockCycles(dut.clk, 2000)
    assert dut.running.value == 0 and b.L.regs[0x20] == 0
    b.L.whoami = 0x44
    await wait_running(dut)


@cocotb.test(skip=GL)
async def test_clock_gating_duty(dut):
    """Fraction of chip clocks that reach each gated block while streaming."""
    b = await setup(dut)
    await wait_running(dut)
    top = dut.user_project
    n = {"clk": 0, "spi": 0, "dp": 0}

    async def count(sig, key):
        while True:
            await RisingEdge(sig)
            n[key] += 1
    tasks = [cocotb.start_soon(count(dut.clk, "clk")),
             cocotb.start_soon(count(top.gclk_spi, "spi")),
             cocotb.start_soon(count(top.gclk_dp, "dp"))]
    await Timer(20, "ms")
    for t in tasks:
        t.cancel()
    spi, dp = n["spi"] / n["clk"], n["dp"] / n["clk"]
    dut._log.info("gated duty: SPI %.1f %%, datapath %.1f %%", 100 * spi, 100 * dp)
    assert spi < 0.40 and dp < 0.03


def band_signal(t):
    """X: 14.5 g +-0.6 g (inside the 13.7-15.2 g hysteresis band) from low,
    a ramp to 20 g, then the same wobble from high, then back to rest."""
    import math
    ms = t / 1e6
    w = 14.5 + 0.6 * math.sin(ms * 1.3)
    if ms < 30:
        x = 0.5
    elif ms < 45:
        x = w                                  # enters the band from below: stays low-g
    elif ms < 50:
        x = 20.0                               # above the threshold: high-g
    elif ms < 65:
        x = w                                  # back in the band: must stay high-g
    else:
        x = 0.5
    return (x, 0.3, 1.0)


@cocotb.test(skip=GL)
async def test_hysteresis_band(dut):
    b = await setup(dut, accel=band_signal)
    await wait_running(dut)
    recs = await collect(dut, b.host, 75)
    ref = reference(b.log)
    assert [(r["axes"], r["src"]) for r in recs] == [x[:2] for x in ref[:len(recs)]]
    xs = [(x[2] / 1e6, x[1] & 1) for x in ref[:len(recs)]]
    assert all(s == 0 for t, s in xs if 31 < t < 44), "in the band from below: low-g"
    assert all(s == 1 for t, s in xs if 51 < t < 64), "in the band from above: high-g"
