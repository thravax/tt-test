"""
Cocotb testbench for tt_um_ro_puf_trng
RO-PUF + TRNG (8 ring oscillators, no CRO-PUF)

RO[i] half-period = (i + 2) cycles.
All counters are deterministic → tests verify FSM, interface, and
comparison logic.  Real entropy (jitter) only appears on silicon.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles

# ---------------------------------------------------------------
# Constants
# ---------------------------------------------------------------
NUM_ROS    = 8
CNT_BITS   = 8
MODE_ROPUF = 0b00
MODE_TRNG  = 0b10

DONE_BIT = 0x80
BUSY_BIT = 0x40


# ---------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------
async def reset(dut, cycles=3):
    dut.rst_n.value  = 0
    dut.ui_in.value  = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, cycles)
    dut.rst_n.value  = 1
    await RisingEdge(dut.clk)


def build_uio(we=0, re=0, start=0, mode=0, addr_clr=0):
    return (we & 1) | ((re & 1) << 1) | ((start & 1) << 2) \
         | ((mode & 3) << 3) | ((addr_clr & 1) << 5)


async def write_byte(dut, data):
    """Pulse we=1 with data on ui_in for one cycle."""
    dut.ui_in.value  = data
    dut.uio_in.value = build_uio(we=1)
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0
    dut.ui_in.value  = 0


async def read_byte(dut):
    """Pulse re=1, return uo_out captured after the rising edge."""
    dut.uio_in.value = build_uio(re=1)
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0
    await FallingEdge(dut.clk)
    return int(dut.uo_out.value)


async def addr_clr(dut):
    dut.uio_in.value = build_uio(addr_clr=1)
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0


async def start_measurement(dut, mode):
    dut.uio_in.value = build_uio(start=1, mode=mode)
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0


async def wait_done(dut, timeout=4096):
    """Wait for done bit in uio_out[7]."""
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.uio_out.value) & DONE_BIT:
            return True
    return False


async def configure(dut, window=64):
    """Write window config register."""
    await addr_clr(dut)
    await write_byte(dut, window & 0xFF)   # addr 0: window


async def read_result(dut):
    """Read result byte (addr 0)."""
    await addr_clr(dut)
    return await read_byte(dut)


# ---------------------------------------------------------------
# Simulation model: expected counter values
# ---------------------------------------------------------------
def sim_counts(window):
    """
    Compute expected counter values from the behavioral RO model.
    RO[i] half-period = (i+2) cycles.
    Rising edges counted for MEASURE cycles 0..(window-2).
    """
    counts = []
    for i in range(NUM_ROS):
        half_p = i + 2
        edges = 0
        t = half_p - 1   # first toggle; edge detected one cycle later
        while t < window - 2:
            edges += 1
            t += 2 * half_p
        counts.append(edges)
    return counts


def sim_puf_bits(window):
    counts = sim_counts(window)
    bits = 0
    for p in range(4):
        if counts[2*p] > counts[2*p+1]:
            bits |= (1 << p)
    return bits


# ---------------------------------------------------------------
# Tests
# ---------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """After reset: FSM=IDLE, done=0, busy=0."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)
    assert (int(dut.uio_out.value) & DONE_BIT) == 0, "done should be 0 after reset"
    assert (int(dut.uio_out.value) & BUSY_BIT) == 0, "busy should be 0 after reset"


@cocotb.test()
async def test_ro_puf_basic(dut):
    """RO-PUF: measurement completes and result matches sim model."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    window = 64
    await configure(dut, window=window)
    await start_measurement(dut, mode=MODE_ROPUF)

    done = await wait_done(dut, timeout=window * 4)
    assert done, "RO-PUF: done never asserted"

    result = await read_result(dut)

    expected_bits = sim_puf_bits(window)
    assert result == expected_bits, \
        f"RO-PUF result 0x{result:02x} != expected 0x{expected_bits:02x}"

    dut._log.info(f"RO-PUF result=0x{result:02x}")


@cocotb.test()
async def test_ro_puf_wider_window(dut):
    """Wider measurement window gives same PUF result."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    for window in (64, 128, 200):
        await configure(dut, window=window)
        await start_measurement(dut, mode=MODE_ROPUF)
        done = await wait_done(dut, timeout=window * 4)
        assert done, f"window={window}: done never asserted"

        result = await read_result(dut)
        expected = sim_puf_bits(window)
        assert result == expected, \
            f"window={window}: 0x{result:02x} != 0x{expected:02x}"


@cocotb.test()
async def test_puf_upper_nibble_zero(dut):
    """RO-PUF result upper nibble must always be zero (only 4 PUF pairs)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    await configure(dut, window=64)
    await start_measurement(dut, mode=MODE_ROPUF)
    await wait_done(dut)

    result = await read_result(dut)
    assert (result & 0xF0) == 0, \
        f"Upper nibble non-zero: 0x{result:02x}"


@cocotb.test()
async def test_puf_consistency(dut):
    """Same config → same result on repeated measurements."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    window = 64
    results = []
    for _ in range(5):
        await configure(dut, window=window)
        await start_measurement(dut, mode=MODE_ROPUF)
        await wait_done(dut)
        results.append(await read_result(dut))

    assert len(set(results)) == 1, \
        f"PUF not consistent: {[hex(r) for r in results]}"


@cocotb.test()
async def test_timing_ropuf(dut):
    """Busy asserts during measurement, done after window expires."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    window = 32
    await configure(dut, window=window)
    await start_measurement(dut, mode=MODE_ROPUF)

    await RisingEdge(dut.clk)
    assert int(dut.uio_out.value) & BUSY_BIT, "busy not set during measurement"

    done = await wait_done(dut, timeout=window * 4)
    assert done, "done never asserted"
    assert not (int(dut.uio_out.value) & BUSY_BIT), "busy still set after done"


@cocotb.test()
async def test_trng_completes(dut):
    """TRNG: done asserts after 8 × window cycles (+overhead)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    window = 32
    await configure(dut, window=window)
    await start_measurement(dut, mode=MODE_TRNG)

    done = await wait_done(dut, timeout=window * 10)
    assert done, "TRNG: done never asserted"

    result = await read_result(dut)
    dut._log.info(f"TRNG result = 0x{result:02x}")


@cocotb.test()
async def test_trng_value(dut):
    """TRNG result is deterministic: two runs from reset produce same byte."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    window = 32
    results = []
    for _ in range(2):
        await reset(dut)
        await configure(dut, window=window)
        await start_measurement(dut, mode=MODE_TRNG)
        done = await wait_done(dut, timeout=window * 10)
        assert done, "TRNG did not complete"
        results.append(await read_result(dut))

    assert results[0] == results[1], \
        f"TRNG not deterministic: {[hex(r) for r in results]}"
    dut._log.info(f"TRNG deterministic value = 0x{results[0]:02x}")


@cocotb.test()
async def test_trng_multiple_bytes(dut):
    """Request 4 TRNG bytes sequentially."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    window = 32
    bytes_out = []
    for _ in range(4):
        await configure(dut, window=window)
        await start_measurement(dut, mode=MODE_TRNG)
        done = await wait_done(dut, timeout=window * 10)
        assert done
        bytes_out.append(await read_result(dut))

    dut._log.info(f"TRNG bytes: {[hex(b) for b in bytes_out]}")


@cocotb.test()
async def test_addr_clr(dut):
    """addr_clr resets the address counter — result reads same twice."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    await configure(dut, window=64)
    await start_measurement(dut, mode=MODE_ROPUF)
    await wait_done(dut)

    await addr_clr(dut)
    r0 = await read_byte(dut)
    await addr_clr(dut)
    r1 = await read_byte(dut)
    assert r0 == r1, f"addr_clr: reads differ {r0} vs {r1}"


@cocotb.test()
async def test_start_from_done(dut):
    """Start a new measurement directly from DONE state."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    window = 64
    await configure(dut, window=window)
    await start_measurement(dut, mode=MODE_ROPUF)
    await wait_done(dut)
    r0 = await read_result(dut)

    await configure(dut, window=window)
    await start_measurement(dut, mode=MODE_ROPUF)
    done = await wait_done(dut, timeout=window * 4)
    assert done
    r1 = await read_result(dut)

    assert r0 == r1, f"Results differ: 0x{r0:02x} vs 0x{r1:02x}"


@cocotb.test()
async def test_write_default_window(dut):
    """Default window (64) is used when no config is written."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    await start_measurement(dut, mode=MODE_ROPUF)
    done = await wait_done(dut, timeout=64 * 4)
    assert done, "Default window measurement did not complete"

    result = await read_result(dut)
    expected = sim_puf_bits(64)
    assert result == expected, \
        f"Default window result 0x{result:02x} != 0x{expected:02x}"


@cocotb.test()
async def test_reset_during_measure(dut):
    """Reset during MEASURE returns FSM to IDLE."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset(dut)

    await configure(dut, window=255)
    await start_measurement(dut, mode=MODE_ROPUF)
    await ClockCycles(dut.clk, 10)

    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    assert not (int(dut.uio_out.value) & DONE_BIT), "done set after reset"
    assert not (int(dut.uio_out.value) & BUSY_BIT), "busy set after reset"

    await configure(dut, window=64)
    await start_measurement(dut, mode=MODE_ROPUF)
    done = await wait_done(dut, timeout=64 * 4)
    assert done, "Measurement after reset did not complete"
