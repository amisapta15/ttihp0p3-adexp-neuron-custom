import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge
import numpy as np

def encode_signed_direct(real_value):
    """Maps real_value to [0,255] where 128 = 0mV."""
    v = int(round(real_value + 128))
    return max(0, min(255, v))

def encode_unsigned_direct(real_value):
    """Encodes unsigned value, clamps to [0,255]."""
    v = int(round(real_value))
    return max(0, min(255, v))

async def load_nibble(dut, nibble):
    """Loads a single nibble using FSM protocol."""
    current_uio = int(dut.uio_in.value) & 0xF0
    dut.uio_in.value = current_uio | (int(nibble) & 0xF)
    
    current_ui = int(dut.ui_in.value)
    dut.ui_in.value = current_ui | (1 << 3)  # Set load_enable
    await RisingEdge(dut.clk)
    
    current_ui = int(dut.ui_in.value)
    dut.ui_in.value = current_ui & ~(1 << 3)  # Clear load_enable
    await RisingEdge(dut.clk)

async def load_parameters(dut, params):
    """Loads all parameters using nibble-by-nibble FSM."""
    dut.ui_in.value = (1 << 4)  # Set load_mode
    await ClockCycles(dut.clk, 2)
    
    param_order = ["DeltaT", "TauW", "a", "b", "Vreset", "VT", "Ibias", "C"]
    
    for param_name in param_order:
        val = params.get(param_name, 0) & 0xFF
        hi_nibble = (val >> 4) & 0xF
        lo_nibble = val & 0xF
        
        await load_nibble(dut, hi_nibble)
        await load_nibble(dut, lo_nibble)
    
    await load_nibble(dut, 0xF)  # Footer
    
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 3)

@cocotb.test()
async def test_regular_spiking_mode(dut):
    """Test regular spiking mode - consistent ISIs with low CV."""
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
    
    # Parameters for regular spiking
    regular_params = {
        "DeltaT": encode_unsigned_direct(2),
        "TauW":   encode_unsigned_direct(200),
        "a":      encode_unsigned_direct(1),
        "b":      encode_unsigned_direct(5),
        "Vreset": encode_signed_direct(-65),
        "VT":     encode_signed_direct(-50),
        "Ibias":  encode_signed_direct(25),
        "C":      encode_unsigned_direct(100)
    }
    
    await load_parameters(dut, regular_params)
    dut.ui_in.value = (1 << 2)  # Enable core
    await ClockCycles(dut.clk, 5)
    
    # Collect spikes
    spike_times = []
    
    for i in range(6000):
        await RisingEdge(dut.clk)
        
        try:
            uo_val = int(dut.uo_out.value)
            spike = uo_val & 1
            
            if spike:
                spike_times.append(i)
                dut._log.info(f"Spike at cycle {i}")
        except ValueError:
            continue
    
    # Calculate ISI statistics
    if len(spike_times) >= 3:
        isis = [spike_times[i+1] - spike_times[i] for i in range(len(spike_times)-1)]
        mean_isi = np.mean(isis)
        std_isi = np.std(isis)
        cv = std_isi / mean_isi if mean_isi > 0 else 0
        
        dut._log.info(f"Regular spiking: {len(spike_times)} spikes")
        dut._log.info(f"Mean ISI: {mean_isi:.2f} cycles")
        dut._log.info(f"CV: {cv:.3f}")
        
        assert len(spike_times) >= 5, "Insufficient spikes"
        assert cv < 0.4, f"CV too high for regular spiking: {cv}"
        
        dut._log.info("Regular spiking test PASSED")
    else:
        dut._log.error(f"Only {len(spike_times)} spikes detected - test fasiled")
        assert False, "Insufficient spikes for regular spiking test"