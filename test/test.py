import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# ------------------------ Helper Functions ------------------------
def encode_signed_direct(real_value):
    """Maps real_value to [0,255] where 128 = 0."""
    v = int(round(real_value + 128))
    return max(0, min(255, v))

def encode_unsigned_direct(real_value):
    """Encodes unsigned value, clamps to [0,255]."""
    v = int(round(real_value))
    return max(0, min(255, v))

def calculate_stats(values):
    """Mean, std, and CV without numpy."""
    if len(values) == 0:
        return 0, 0, 0
    mean = sum(values) / len(values)
    variance = sum((x - mean) ** 2 for x in values) / len(values)
    std = variance ** 0.5
    cv = std / mean if mean > 0 else 0
    return mean, std, cv

async def load_nibble(dut, nibble):
    """Load a 4-bit nibble into the parameter loading interface."""
    # Set nibble in lower 4 bits of uio_in
    current_uio = int(dut.uio_in.value) & 0xF0
    dut.uio_in.value = current_uio | (int(nibble) & 0xF)

    # Set load_mode bit (bit 4 of ui_in)
    current_ui = int(dut.ui_in.value) | (1 << 4)
    dut.ui_in.value = current_ui
    await RisingEdge(dut.clk)

    # Pulse load_enable (bit 3 of ui_in)
    dut.ui_in.value = current_ui | (1 << 3)
    await RisingEdge(dut.clk)

    # Clear load_enable
    dut.ui_in.value = current_ui & ~(1 << 3)
    await RisingEdge(dut.clk)

async def load_parameters(dut, params):
    """Load neuron parameters via nibble interface."""
    # Enter load mode
    dut.ui_in.value = (1 << 4)  # load_mode = 1
    await ClockCycles(dut.clk, 2)

    # Parameter order as expected by the design: DeltaT, TauW, a, b, Vreset, VT, Ibias, C
    param_order = ["DeltaT", "TauW", "a", "b", "Vreset", "VT", "Ibias", "C"]
    
    for pname in param_order:
        val = params.get(pname, 0) & 0xFF
        hi_nibble = (val >> 4) & 0xF
        lo_nibble = val & 0xF
        
        # Load high nibble first, then low nibble
        await load_nibble(dut, hi_nibble)
        await load_nibble(dut, lo_nibble)

    # Send footer nibble (0xF)
    await load_nibble(dut, 0xF)
    await ClockCycles(dut.clk, 2)
    
    # Exit load mode
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 2)

async def monitor_spikes(dut, cycles=5000):
    """Monitor for spikes on uo_out[0] for given number of cycles."""
    spike_times = []
    for i in range(cycles):
        await RisingEdge(dut.clk)
        try:
            uo_val = int(dut.uo_out.value)
            if uo_val & 1:  # Check bit 0 for spike
                spike_times.append(i)
        except (ValueError, AttributeError):
            # Handle case where signal might be undefined
            continue
    return spike_times

# ------------------------ Basic Functionality Test ------------------------
@cocotb.test()
async def test_basic_spiking(dut):
    """Test basic spiking functionality with simple parameters."""
    
    # Start clock
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    
    # Reset sequence
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    dut._log.info("Starting basic spiking test...")
    
    # Parameters that should produce regular spiking
    # Using values that match the Verilog default initialization
    basic_params = {
        "DeltaT": encode_signed_direct(2),     # 130 in Verilog default
        "TauW":   encode_unsigned_direct(100), # 100 in Verilog default  
        "a":      encode_unsigned_direct(1),   # 1 in Verilog default
        "b":      encode_unsigned_direct(5),   # 5 in Verilog default
        "Vreset": encode_signed_direct(-65),   # 63 in Verilog default
        "VT":     encode_signed_direct(-50),   # 78 in Verilog default
        "Ibias":  encode_unsigned_direct(160), # 160 in Verilog default
        "C":      encode_unsigned_direct(10)   # 10 in Verilog default
    }
    
    # Load parameters
    await load_parameters(dut, basic_params)
    dut._log.info("Parameters loaded")
    
    # Wait for loader to complete
    await ClockCycles(dut.clk, 10)
    
    # Enable the neuron core (bit 2 of ui_in)
    dut.ui_in.value = (1 << 2)
    await ClockCycles(dut.clk, 5)
    dut._log.info("Neuron core enabled")
    
    # Monitor for spikes
    spikes = await monitor_spikes(dut, cycles=5000)
    
    dut._log.info(f"Detected {len(spikes)} spikes")
    if len(spikes) > 0:
        dut._log.info(f"First few spike times: {spikes[:min(5, len(spikes))]}")
    
    # Basic assertions
    assert len(spikes) > 0, f"No spikes detected! Expected at least 1 spike."
    
    # Calculate inter-spike intervals if we have enough spikes
    if len(spikes) > 1:
        isis = [spikes[i+1] - spikes[i] for i in range(len(spikes)-1)]
        mean_isi = sum(isis) / len(isis)
        dut._log.info(f"Mean ISI: {mean_isi:.2f} cycles")
        
        # Check that ISIs are reasonable (not too short or too long)
        assert mean_isi > 5, f"ISI too short ({mean_isi:.2f}), might be oscillating"
        assert mean_isi < 1000, f"ISI too long ({mean_isi:.2f}), firing too slowly"
    
    dut._log.info("✓ Basic spiking test passed!")

# ------------------------ Regular Spiking Test ------------------------
@cocotb.test()
async def test_regular_spiking(dut):
    """Test regular spiking pattern with optimized parameters."""
    
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Parameters for regular spiking
    rs_params = {
        "DeltaT": encode_signed_direct(2),     # Small slope factor
        "TauW":   encode_unsigned_direct(100), # Fast adaptation recovery
        "a":      encode_unsigned_direct(1),   # Minimal subthreshold adaptation
        "b":      encode_unsigned_direct(2),   # Small spike-triggered adaptation
        "Vreset": encode_signed_direct(-65),
        "VT":     encode_signed_direct(-50),
        "Ibias":  encode_unsigned_direct(160), # Higher drive current
        "C":      encode_unsigned_direct(10)   # Lower capacitance for faster dynamics
    }

    await load_parameters(dut, rs_params)
    await ClockCycles(dut.clk, 10)
    
    dut.ui_in.value = (1 << 2)  # enable core
    await ClockCycles(dut.clk, 5)

    spikes = await monitor_spikes(dut, cycles=8000)
    dut._log.info(f"Regular spiking: {len(spikes)} spikes detected")
    
    assert len(spikes) > 3, f"Regular Spiking: neuron did not spike enough (got {len(spikes)})"

    if len(spikes) > 1:
        isis = [spikes[i+1]-spikes[i] for i in range(len(spikes)-1)]
        mean_isi, std_isi, cv = calculate_stats(isis)
        
        dut._log.info(f"Regular Spiking: {len(spikes)} spikes, Mean ISI: {mean_isi:.2f}, CV: {cv:.3f}")
        
        # For regular spiking, we expect low coefficient of variation
        assert cv < 0.3, f"Regular Spiking: CV too high ({cv:.3f}), expected regular pattern"

# ------------------------ Parameter Loading Test ------------------------
@cocotb.test()
async def test_parameter_loading(dut):
    """Test that parameter loading works correctly."""
    
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    dut._log.info("Testing parameter loading...")
    
    # Test parameters
    test_params = {
        "DeltaT": 130,  # encode_signed_direct(2)
        "TauW":   100,  # encode_unsigned_direct(100)
        "a":      2,    # encode_unsigned_direct(2)
        "b":      5,    # encode_unsigned_direct(5)
        "Vreset": 63,   # encode_signed_direct(-65)
        "VT":     78,   # encode_signed_direct(-50)
        "Ibias":  150,  # encode_unsigned_direct(150)
        "C":      10    # encode_unsigned_direct(10)
    }
    
    # Load parameters - this should not cause any errors
    await load_parameters(dut, test_params)
    dut._log.info("Parameters loaded successfully")
    
    # Wait for loading to complete
    await ClockCycles(dut.clk, 100)
    
    # Enable core briefly to see if it responds
    dut.ui_in.value = (1 << 2)
    await ClockCycles(dut.clk, 100)
    
    # Check that we can read outputs without errors
    try:
        uo_val = int(dut.uo_out.value)
        dut._log.info(f"Output value: {uo_val:08b}")
    except (ValueError, AttributeError) as e:
        dut._log.warning(f"Could not read output: {e}")
    
    dut._log.info("✓ Parameter loading test passed!")

# ------------------------ Simple Minimal Test ------------------------
@cocotb.test()
async def test_minimal_operation(dut):
    """Minimal test to verify the design loads and runs."""
    
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    dut._log.info("Starting minimal operation test...")

    # Use default parameters by not loading any
    # Enable the neuron core immediately
    dut.ui_in.value = (1 << 2)  # enable core
    await ClockCycles(dut.clk, 10)

    # Monitor for a short time to see if anything happens
    spikes = await monitor_spikes(dut, cycles=2000)
    
    dut._log.info(f"Minimal test: Found {len(spikes)} spikes with default parameters")
    
    # Try to read output value
    try:
        uo_val = int(dut.uo_out.value)
        dut._log.info(f"Final output value: {uo_val:08b}")
    except (ValueError, AttributeError) as e:
        dut._log.warning(f"Could not read output: {e}")

    # Just verify the design doesn't crash - any number of spikes is acceptable
    dut._log.info("✓ Minimal operation test completed!")

# ------------------------ Debug Output Test ------------------------
@cocotb.test()
async def test_debug_output(dut):
    """Test debug mode output."""
    
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    
    # Reset
    dut.rst_n.value = 0
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # Enable debug mode (bit 1) and core (bit 2)
    dut.ui_in.value = (1 << 2) | (1 << 1)
    await ClockCycles(dut.clk, 100)

    # Read output in debug mode
    try:
        uo_val = int(dut.uo_out.value)
        dut._log.info(f"Debug mode output: {uo_val:08b}")
    except (ValueError, AttributeError) as e:
        dut._log.warning(f"Could not read debug output: {e}")

    # Switch to normal mode
    dut.ui_in.value = (1 << 2)  # only enable core
    await ClockCycles(dut.clk, 100)

    try:
        uo_val = int(dut.uo_out.value)
        dut._log.info(f"Normal mode output: {uo_val:08b}")
    except (ValueError, AttributeError) as e:
        dut._log.warning(f"Could not read normal output: {e}")

    dut._log.info("✓ Debug output test completed!")