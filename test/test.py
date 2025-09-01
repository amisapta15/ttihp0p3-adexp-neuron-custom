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

    # Enable neuron (bit 2)
    dut.ui_in.value = (1 << 2)
    await wait_for_stable(dut, 50)

    spikes = await monitor_spikes(dut, cycles=8000)
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

    # Enable neuron (bit 2)
    dut.ui_in.value = (1 << 2)
    await wait_for_stable(dut, 50)

    spikes = await monitor_spikes(dut, cycles=15000)
    dut._log.info(f"Adaptation test: detected {len(spikes)} spikes")

    if len(spikes) < 4:
        dut._log.warning("Not enough spikes for adaptation check")
        return

    isi = [spikes[i+1] - spikes[i] for i in range(len(spikes)-1)]
    first_half = isi[:len(isi)//2]
    second_half = isi[len(isi)//2:]

    avg_first = sum(first_half)/len(first_half)
    avg_second = sum(second_half)/len(second_half)

    dut._log.info(f"Avg ISI first half={avg_first}, second half={avg_second}")
    assert avg_second >= avg_first, "No clear adaptation observed"
