import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# ------------------------ Encoding ------------------------
def encode_signed_direct(val):
    return max(0, min(255, int(round(val + 128))))

def encode_unsigned_direct(val):
    return max(0, min(255, int(round(val))))

# ------------------------ Parameter Loader ------------------------
async def load_nibble(dut, nibble):
    current_uio = int(dut.uio_in.value) & 0xF0
    dut.uio_in.value = current_uio | (nibble & 0xF)
    dut.ui_in.value = (1 << 4) | (1 << 3)  # load_mode + load_enable
    await RisingEdge(dut.clk)
    dut.ui_in.value = (1 << 4)  # load_mode only
    await RisingEdge(dut.clk)

async def load_parameters(dut, params):
    dut.ui_in.value = (1 << 4)  # load_mode
    await ClockCycles(dut.clk, 2)
    for p in ["DeltaT","TauW","a","b","Vreset","VT","Ibias","C"]:
        val = params[p] & 0xFF
        hi = (val >> 4) & 0xF
        lo = val & 0xF
        await load_nibble(dut, hi)
        await load_nibble(dut, lo)
    await load_nibble(dut, 0xF)  # footer
    await ClockCycles(dut.clk,2)
    dut.ui_in.value = 0
    await ClockCycles(dut.clk,2)

# ------------------------ Spike Monitor ------------------------
async def monitor_spikes(dut, cycles=5000):
    spikes = []
    for i in range(cycles):
        await RisingEdge(dut.clk)
        if int(dut.uo_out.value) & 1:
            spikes.append(i)
    return spikes

# ------------------------ Testbench ------------------------
@cocotb.test()
async def test_regular_spiking(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="us").start())

    # Reset
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Regular Spiking Parameters
    rs_params = {
        "DeltaT": encode_signed_direct(2),
        "TauW":   encode_unsigned_direct(200),
        "a":      encode_unsigned_direct(1),
        "b":      encode_unsigned_direct(5),
        "Vreset": encode_signed_direct(-65),
        "VT":     encode_signed_direct(-50),
        "Ibias":  encode_unsigned_direct(120),
        "C":      encode_unsigned_direct(10)
    }

    await load_parameters(dut, rs_params)
    dut.ui_in.value = (1 << 2)  # enable core
    await ClockCycles(dut.clk, 5)

    spikes = await monitor_spikes(dut, cycles=8000)
    dut._log.info(f"Regular Spiking spikes detected: {len(spikes)}")
    assert len(spikes) > 0, "Neuron did not spike"
