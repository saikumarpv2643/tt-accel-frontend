"""cocotb tests for tt_um_saikumarpv2643_accel_if.

The design talks to a behavioural H3LIS331DL (models.py) and is read through
its host SPI port, the same way a microcontroller would use the chip.
"""
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, with_timeout

from models import H3LIS331DL, HostSPI, cic_ref

CLK_NS = 100          # 10 MHz
PERIOD = 1000         # sensor sample every 1000 clocks (100 us, sped up from 1 ms)

# Gate-level run (make GATES=yes): the netlist has no parameters, so the real
# 5 ms boot delay applies, and simulation is much slower. Long tests are skipped.
GL = os.environ.get("GATES") == "yes"
BOOT_WAIT = 80000 if GL else 20000


async def setup(dut, whoami=0x32, period=PERIOD, seed=1):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, "ns").start())
    dut.ena.value = 1
    dut.uio_in.value = 0
    sensor = H3LIS331DL(dut, whoami=whoami, period=period, seed=seed)
    host = HostSPI(dut)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    cocotb.start_soon(sensor.spi())
    cocotb.start_soon(sensor.producer())
    return sensor, host


async def wait_running(dut, cycles=BOOT_WAIT):
    await with_timeout(RisingEdge(dut.running), cycles * CLK_NS, "ns")


async def collect(dut, host, n, max_polls=2000):
    got = []
    for _ in range(max_polls):
        st = await host.status()
        if st["level"]:
            got += await host.samples(st["level"])
            if len(got) >= n:
                return got[:n]
        else:
            await ClockCycles(dut.clk, 300)
    raise AssertionError(f"only {len(got)} of {n} samples arrived")


async def apply_ctrl(dut, sensor, host, ctrl):
    """Write CTRL at a quiet moment and return the index of the first sensor
    sample that enters the (cleared) pipeline after the write."""
    sensor.pause = True
    while dut.sens_drdy.value == 1:          # let the design take the last one
        await FallingEdge(dut.sens_drdy)
    await ClockCycles(dut.clk, 200)
    await host.write(0x01, ctrl)
    await ClockCycles(dut.clk, 1500)         # sensor reconfiguration burst
    start = len(sensor.produced)
    sensor.pause = False
    return start


@cocotb.test()
async def test_boot_and_config(dut):
    """Reset -> WHO_AM_I check -> CTRL_REG1..4 written -> streaming."""
    sensor, host = await setup(dut)
    await wait_running(dut)
    assert (await host.read(0x00))[0] == 0xA5, "ID register"
    assert (await host.read(0x04))[0] == 0x32, "WHO_AM_I copy"
    st = await host.status()
    assert st["ok"] and st["run"], st
    assert sensor.regs[0x20] == 0x3F, hex(sensor.regs[0x20])   # normal, 1 kHz, XYZ
    assert sensor.regs[0x21] == 0x00
    assert sensor.regs[0x22] == 0x02, hex(sensor.regs[0x22])   # INT1 = data ready
    assert sensor.regs[0x23] == 0x80, hex(sensor.regs[0x23])   # BDU, 100 g


@cocotb.test()
async def test_stream_passthrough(dut):
    """Decimation 1: every sensor sample comes out unchanged and in order."""
    sensor, host = await setup(dut)
    await wait_running(dut)
    n = 10 if GL else 40
    got = await collect(dut, host, n)
    assert got == sensor.produced[:n]
    assert sensor.overruns == 0


@cocotb.test(skip=GL)
async def test_cic_decimation(dut):
    """Decimation 2..16 matches the bit-exact reference on all three axes."""
    sensor, host = await setup(dut, seed=7)
    await wait_running(dut)
    for r_log2 in (1, 2, 3, 4):
        start = await apply_ctrl(dut, sensor, host, 0x01 | (r_log2 << 4))
        n_out = 6
        got = await collect(dut, host, n_out)
        xs = sensor.produced[start:]
        for axis in range(3):
            ref = cic_ref([s[axis] for s in xs], n=2, r_log2=r_log2)
            assert [g[axis] for g in got] == ref[:n_out], (r_log2, axis)
        dut._log.info("R=%d: %d outputs match", 1 << r_log2, n_out)
    assert sensor.overruns == 0


@cocotb.test()
async def test_cic_full_scale_dc(dut):
    """Full-scale DC at R=16: no overflow inside the filter, output = input."""
    sensor, host = await setup(dut)
    sensor.source = lambda: (32752, -32768, 0)
    await wait_running(dut)
    await apply_ctrl(dut, sensor, host, 0x01 | (4 << 4))
    got = await collect(dut, host, 4)
    assert got[-1] == (32752, -32768, 0), got   # settled after the transient


@cocotb.test()
async def test_fifo_overflow(dut):
    """Host stops reading: FIFO fills to 8, overflow sets, oldest data kept,
    flag clears on read."""
    sensor, host = await setup(dut)
    await wait_running(dut)
    start = await apply_ctrl(dut, sensor, host, 0x01)
    await ClockCycles(dut.clk, PERIOD * 12)
    assert dut.overflow.value == 1
    # stop the sensor so no new sample overflows again while we check
    sensor.pause = True
    while dut.sens_drdy.value == 1:
        await FallingEdge(dut.sens_drdy)
    await ClockCycles(dut.clk, 200)
    st = await host.status()
    assert st["level"] == 8 and st["ovf"] == 1, st
    assert dut.overflow.value == 0, "overflow clears when STATUS is read"
    got = await host.samples(8)
    assert got == sensor.produced[start:start + 8], "the oldest 8 are kept"
    st = await host.status()
    assert st["level"] == 0 and st["ovf"] == 0, st


@cocotb.test(skip=GL)
async def test_range_and_disable(dut):
    """CTRL range bits reach CTRL_REG4; enable = 0 powers the sensor down."""
    sensor, host = await setup(dut)
    await wait_running(dut)
    for ctrl, reg4 in ((0x03, 0x90), (0x07, 0xB0), (0x05, 0xB0), (0x01, 0x80)):
        await apply_ctrl(dut, sensor, host, ctrl)
        assert sensor.regs[0x23] == reg4, (hex(ctrl), hex(sensor.regs[0x23]))
    await apply_ctrl(dut, sensor, host, 0x00)
    assert sensor.regs[0x20] == 0x07, "power-down"
    assert dut.running.value == 0
    await ClockCycles(dut.clk, PERIOD * 5)
    assert (await host.status())["level"] == 0


@cocotb.test(skip=GL)
async def test_wrong_whoami_then_recovers(dut):
    """A missing/wrong sensor is never configured; the design retries and
    starts once the right part answers."""
    sensor, host = await setup(dut, whoami=0x00)
    await ClockCycles(dut.clk, 3000)
    st = await host.status()
    assert st["ok"] == 0 and st["run"] == 0, st
    assert sensor.regs[0x20] == 0x07, "not configured"
    sensor.whoami = 0x32
    await wait_running(dut)
    got = await collect(dut, host, 5)
    assert got == sensor.produced[:5]


@cocotb.test()
async def test_irq_threshold(dut):
    """IRQ rises when the FIFO holds IRQ_THR samples and falls after reading."""
    sensor, host = await setup(dut)
    await wait_running(dut)
    await host.write(0x03, 4)
    assert (await host.read(0x03))[0] == 4
    await apply_ctrl(dut, sensor, host, 0x01)
    await with_timeout(RisingEdge(dut.irq), PERIOD * 10 * CLK_NS, "ns")
    st = await host.status()
    assert st["level"] >= 4, st
    await host.samples(st["level"])
    assert dut.irq.value == 0
