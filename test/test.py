import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge


def encode_signed_direct(real_value):
    """Maps real_value to [0,255] where 128 = 0."""
    v = int(round(real_value + 128))
    return max(0, min(255, v))


def encode_unsigned_direct(real_value):
    """Encodes unsigned value, clamps to [0,255]."""
    v = int(round(real_value))
    return max(0, min(255, v))


def calculate_stats(values):
    """Mean and std without numpy."""
    if len(values) == 0:
        return 0, 0
    mean = sum(values) / len(values)
    variance = sum((x - mean) ** 2 for x in values) / len(values)
    std = variance ** 0.5
    return mean, std


async def load_nibble(dut, nibble):
    """Robust nibble loader: set nibble, assert load_enable for one cycle."""
    current_uio = int(dut.uio_in.value) & 0xF0
    dut.uio_in.value = current_uio | (int(nibble) & 0xF)

    # Keep load_mode high during loading
    current_ui = int(dut.ui_in.value)
    current_ui = (current_ui | (1 << 4))  # set load_mode
    dut.ui_in.value = current_ui

    await RisingEdge(dut.clk)

    # Assert load_enable for one cycle
    dut.ui_in.value = current_ui | (1 << 3)
    await RisingEdge(dut.clk)

    # Deassert
    dut.ui_in.value = current_ui & ~(1 << 3)
    await RisingEdge(dut.clk)


async def load_parameters(dut, params):
    dut.ui_in.value = (1 << 4)  # load_mode = 1
    await ClockCycles(dut.clk, 2)

    param_order = ["DeltaT", "TauW", "a", "b", "Vreset", "VT", "Ibias", "C"]
    for pname in param_order:
        val = params.get(pname, 0) & 0xFF
        hi_nibble = (val >> 4) & 0xF
        lo_nibble = val & 0xF
        await load_nibble(dut, hi_nibble)
        await load_nibble(dut, lo_nibble)

    await load_nibble(dut, 0xF)  # footer
    await ClockCycles(dut.clk, 2)

    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_regular_spiking_mode(dut):
    dut._log.info("=== Testing Regular Spiking Mode ===")

    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Parameters tuned for reliable spiking
    regular_params = {
        "DeltaT": encode_signed_direct(2),
        "TauW":   encode_unsigned_direct(200),
        "a":      encode_unsigned_direct(1),
        "b":      encode_unsigned_direct(5),
        "Vreset": encode_signed_direct(-65),
        "VT":     encode_signed_direct(-50),
        "Ibias":  encode_unsigned_direct(120),  # stronger drive
        "C":      encode_unsigned_direct(10)  # smaller capacitance
    }

    await load_parameters(dut, regular_params)
    dut.ui_in.value = (1 << 2)  # enable core
    await ClockCycles(dut.clk, 5)

    spike_times = []
    for i in range(8000):
        await RisingEdge(dut.clk)
        try:
            uo_val = int(dut.uo_out.value)
            spike = uo_val & 1
            if spike:
                spike_times.append(i)
                dut._log.info(f"Spike at cycle {i}")
        except ValueError:
            continue

        # Debug Vm every 500 cycles
        if i % 500 == 0:
            uo_val = int(dut.uo_out.value)
            vm = (uo_val >> 1) & 0x3F
            dut._log.info(f"Cycle {i}: Vm_out={vm}, raw_out={uo_val:08b}")

    if len(spike_times) >= 3:
        isis = [spike_times[i+1] - spike_times[i] for i in range(len(spike_times)-1)]
        mean_isi, std_isi = calculate_stats(isis)
        cv = std_isi / mean_isi if mean_isi > 0 else 0

        dut._log.info(f"Regular spiking: {len(spike_times)} spikes")
        dut._log.info(f"Mean ISI: {mean_isi:.2f} cycles")
        dut._log.info(f"CV: {cv:.3f}")

        assert len(spike_times) >= 5, "Insufficient spikes"
        assert cv < 0.4, f"CV too high: {cv}"
        dut._log.info("Regular spiking test PASSED")
    else:
        dut._log.info(f"Only {len(spike_times)} spikes detected")
        assert len(spike_times) >= 1, "No spikes detected"
