import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge

# ---------------- Helper Functions (copied from your test file) ----------------
def encode_signed_direct(real_value):
    """Encodes a real-world signed value directly for hardware. Maps to [0,255] where 128 = 0mV."""
    v = int(round(real_value + 128))
    if v < 0: v = 0
    if v > 255: v = 255
    return v

def encode_unsigned_direct(real_value):
    """Encodes an unsigned value directly. Clamps to [0,255]."""
    v = int(round(real_value))
    if v < 0: v = 0
    if v > 255: v = 255
    return v

async def load_nibble(dut, nibble):
    """Loads a single nibble using the FSM protocol."""
    current_uio = int(dut.uio_in.value) & 0xF0
    dut.uio_in.value = current_uio | (int(nibble) & 0xF)
    
    current_ui = int(dut.ui_in.value)
    dut.ui_in.value = current_ui | (1 << 3)  # Set load_enable
    
    await RisingEdge(dut.clk)
    
    current_ui = int(dut.ui_in.value)
    dut.ui_in.value = current_ui & ~(1 << 3)  # Clear load_enable
    
    await RisingEdge(dut.clk)

async def load_parameters(dut, params):
    """Loads all parameters using the nibble-by-nibble FSM."""
    dut._log.info("Loading parameters...")
    
    dut.ui_in.value = (1 << 4)  # Set load_mode
    await ClockCycles(dut.clk, 2)
    
    param_order = ["DeltaT", "TauW", "a", "b", "Vreset", "VT", "Ibias", "C"]
    
    for param_name in param_order:
        val = params.get(param_name, 0) & 0xFF
        hi_nibble = (val >> 4) & 0xF
        lo_nibble = val & 0xF
        
        await load_nibble(dut, hi_nibble)
        await load_nibble(dut, lo_nibble)
    
    await load_nibble(dut, 0xF) # Footer
    
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 3)

# ---------------- The Simple Stimulus Test ----------------

@cocotb.test()
async def test_simple_stimulus_spike(dut):
    """
    Tests if the neuron can produce a single spike with a strong, constant stimulus.
    This is a basic sanity check for the 12-bit core logic.
    """
    dut._log.info("=== Starting Simple Stimulus Sanity Check ===")
    
    # Start the clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # 1. Reset the DUT
    dut._log.info("Resetting DUT...")
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    
    # 2. Define and load "force-a-spike" parameters
    dut._log.info("Loading strong stimulus parameters...")
    force_spike_params = {
        "DeltaT": encode_unsigned_direct(2),     # Standard value
        "TauW":   encode_unsigned_direct(100),   # Standard value
        "a":      encode_unsigned_direct(1),     # Minimal subthreshold adaptation
        "b":      encode_unsigned_direct(1),     # Minimal spike-triggered adaptation
        "Vreset": encode_signed_direct(-65),     # Standard reset potential
        "VT":     encode_signed_direct(-50),     # Standard threshold
        "Ibias":  encode_signed_direct(90),      # VERY HIGH driving current to force a spike
        "C":      encode_unsigned_direct(15)     # LOW capacitance for fast charging
    }
    
    await load_parameters(dut, force_spike_params)
    
    # 3. Enable the neuron core
    dut._log.info("Enabling neuron core...")
    dut.ui_in.value = (1 << 2)  # Enable core
    await ClockCycles(dut.clk, 10)
    
    # 4. Monitor for a spike
    dut._log.info("Monitoring for first spike...")
    spike_detected = False
    # Check for 5000 cycles, which should be more than enough time
    for i in range(5000):
        await RisingEdge(dut.clk)
        
        try:
            # The spike signal is the LSB of the uo_out port
            if (int(dut.uo_out.value) & 1) == 1:
                dut._log.info(f"✅ SUCCESS: Spike detected at cycle {i}!")
                spike_detected = True
                break
        except ValueError:
            # Catch 'x' or 'z' states, just in case
            continue
            
    # 5. Assert that a spike was found
    if not spike_detected:
        dut._log.error("❌ FAILURE: No spike was detected after 5000 cycles.")
    
    assert spike_detected, "Neuron failed to produce any spike even with a strong stimulus."
    
    dut._log.info("=== Simple Stimulus Sanity Check PASSED ===")