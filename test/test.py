import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# ------------------------ Encoding ------------------------
def encode_signed_direct(val):
    # map real signed value (mV or bias units in your convention) into 0..255
    return max(0, min(255, int(round(val + 128))))

def encode_unsigned_direct(val):
    return max(0, min(255, int(round(val))))

# ------------------------ Parameter Loader ------------------------
async def load_nibble(dut, nibble):
    current_uio = int(dut.uio_in.value) & 0xF0
    dut.uio_in.value = current_uio | (nibble & 0xF)
    # set load_mode + load_enable
    dut.ui_in.value = (1 << 4) | (1 << 3)
    await RisingEdge(dut.clk)
    # deassert
    dut.ui_in.value = 0
    await RisingEdge(dut.clk)

async def load_parameters(dut, params_dict):
    # loader expects lower nibble then upper nibble per byte (matches existing RTL loader)
    keys = ["DeltaT", "TauW", "a", "b", "Vreset", "VT", "Ibias", "C"]
    for k in keys:
        v = params_dict.get(k, 0) & 0xFF
        lo = v & 0xF
        hi = (v >> 4) & 0xF
        # send lower nibble then upper nibble
        await load_nibble(dut, lo)
        await load_nibble(dut, hi)
    # send footer nibble 0xF to commit
    await load_nibble(dut, 0xF)
    await ClockCycles(dut.clk, 1)

# ------------------------ Monitors ------------------------
async def monitor_spikes(dut, cycles=5000):
    spikes = []
    for _ in range(cycles):
        await RisingEdge(dut.clk)
        if int(dut.uo_out.value) & 0x1:
            spikes.append(1)
    return spikes

# ------------------------ Tests ------------------------
@cocotb.test()
async def test_regular_spiking(dut):
    """Try a small sweep of Ibias values (and one alternate DeltaT) until we detect spikes."""
    cocotb.start_soon(Clock(dut.clk, 1000).start())
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 2)

    # Base params (matches LUT16 conventions)
    base_params = {
        "DeltaT": encode_signed_direct(2),
        "TauW":   encode_unsigned_direct(100),
        "a":      encode_unsigned_direct(2),
        "b":      encode_unsigned_direct(40),
        "Vreset": encode_signed_direct(-65),
        "VT":     encode_signed_direct(-50),
        "C":      encode_unsigned_direct(200)
    }

    # Ibias sweep (signed values). These are in the same units your RTL expects after encode_signed_direct.
    ibias_candidates = [5, 15, 30, 60, 120]   # try small -> larger currents
    found_spike = False
    for ib in ibias_candidates:
        params = dict(base_params)
        params["Ibias"] = encode_signed_direct(ib)
        dut._log.info(f"Trying Ibias={ib} (encoded {params['Ibias']})")
        await load_parameters(dut, params)
        # enable core
        dut.ui_in.value = (1 << 2)
        # give it some cycles to settle and spike
        await ClockCycles(dut.clk, 50)
        spikes = await monitor_spikes(dut, cycles=4000)
        dut._log.info(f"Ibias={ib} => spikes={len(spikes)}")
        if len(spikes) > 0:
            found_spike = True
            break

    assert found_spike, "No spikes detected for any Ibias in sweep (increase sweep range or consider RTL tuning)"
    dut._log.info(f"Test PASSED with Ibias={ib} (encoded {params['Ibias']}) => spikes={len(spikes)}")