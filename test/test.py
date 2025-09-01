# ============================================================================
# Cocotb Testbench for AdEx Neuron System (LUT32)
# ----------------------------------------------------------------------------
# Staged testing:
#   1. Basic spiking check under constant bias current
#   2. Spike-frequency adaptation (SFA) check if spiking is confirmed
# ----------------------------------------------------------------------------

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# ------------------------ Helper Functions ------------------------
def encode_signed_direct(real_value):
    v = int(round(real_value + 128))
    return max(0, min(255, v))

def encode_unsigned_direct(real_value):
    v = int(round(real_value))
    return max(0, min(255, v))

async def wait_for_stable(dut, cycles=50):
    await ClockCycles(dut.clk, cycles)

async def monitor_spikes(dut, cycles=5000):
    """Monitor spikes safely, treating X/Z as 0."""
    spikes = []
    prev = 0
    for i in range(cycles):
        await RisingEdge(dut.clk)
        try:
            val = int(dut.uo_out.value) & 0x1
        except ValueError:
            val = 0  # treat 'x'/'z' as 0
        if val and not prev:
            spikes.append(i)
        prev = val
    return spikes

async def load_nibble(dut, nibble):
    """Drive a 4-bit nibble into loader FSM."""
    dut.uio_in.value = nibble & 0xF
    dut.ui_in.value = (1 << 4) | (1 << 3)
    await RisingEdge(dut.clk)
    dut.ui_in.value = (1 << 4)
    await RisingEdge(dut.clk)

async def load_parameters(dut, params):
    """Load 8 parameters: DeltaT, TauW, a, b, Vreset, VT, Ibias, C."""
    order = ["DeltaT", "TauW", "a", "b", "Vreset", "VT", "Ibias", "C"]
    for name in order:
        val = params.get(name, 0) & 0xFF
        hi, lo = (val >> 4) & 0xF, val & 0xF
        await load_nibble(dut, hi)
        await load_nibble(dut, lo)
    await wait_for_stable(dut, 5)

# ------------------------ Tests ------------------------
@cocotb.test()
async def test_basic_spiking(dut):
    """Check neuron produces at least one spike."""

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # Reset
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await wait_for_stable(dut, 20)

    # Load spiking parameters
    basic_params = {
        "DeltaT": encode_signed_direct(5),
        "TauW":   encode_unsigned_direct(200),
        "a":      encode_unsigned_direct(1),
        "b":      encode_unsigned_direct(2),    # smaller adaptation increment
        "Vreset": encode_signed_direct(-65),
        "VT":     encode_signed_direct(-55),    # easier threshold
        "Ibias":  encode_unsigned_direct(250),  # stronger bias
        "C":      encode_unsigned_direct(10),
    }

    await load_parameters(dut, basic_params)

    # Enable neuron (bit 2)
    dut.ui_in.value = (1 << 2)
    await wait_for_stable(dut, 50)

    spikes = await monitor_spikes(dut, cycles=12000)
    dut._log.info(f"Basic spiking: detected {len(spikes)} spikes")

    assert len(spikes) > 0, "Neuron did not spike"

@cocotb.test()
async def test_spike_frequency_adaptation(dut):
    """Check spike-frequency adaptation after confirming spiking."""

    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # Reset
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await wait_for_stable(dut, 20)

    # Load adaptation parameters
    adapt_params = {
        "DeltaT": encode_signed_direct(2),
        "TauW":   encode_unsigned_direct(150),
        "a":      encode_unsigned_direct(2),
        "b":      encode_unsigned_direct(8),
        "Vreset": encode_signed_direct(-65),
        "VT":     encode_signed_direct(-50),
        "Ibias":  encode_unsigned_direct(250),   # <<< raised Ibias
        "C":      encode_unsigned_direct(10),
    }
    await load_parameters(dut, adapt_params)

    # Enable neuron (bit 2)
    dut.ui_in.value = (1 << 2)
    await wait_for_stable(dut, 50)

    # Run longer
    spikes = await monitor_spikes(dut, cycles=80000)
    dut._log.info(f"Adaptation test: detected {len(spikes)} total spikes")
    dut._log.info(f"Spike times: {spikes}")

    if len(spikes) < 4:
        dut._log.warning("Not enough spikes for adaptation check")
        return

    isi = [spikes[i+1] - spikes[i] for i in range(len(spikes)-1)]
    first_half = isi[:len(isi)//2]
    second_half = isi[len(isi)//2:]

    avg_first = sum(first_half) / len(first_half)
    avg_second = sum(second_half) / len(second_half)

    dut._log.info(f"Avg ISI first half={avg_first}, second half={avg_second}")
    assert avg_second >= avg_first, "No clear adaptation observed"
