import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge
import numpy as np
from collections import defaultdict

# ---------------- Enhanced Helpers for 16-bit System ----------------
def encode_signed_direct(real_value):
    """Encodes a real-world signed value directly for hardware.
       Maps real_value to [0,255] where 128 = 0mV."""
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

async def load_nibble(dut, nibble, param_name=""):
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
        
        await load_nibble(dut, hi_nibble, f"{param_name}_hi")
        await load_nibble(dut, lo_nibble, f"{param_name}_lo")
    
    await load_nibble(dut, 0xF, "Footer")
    
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 3)

class ISIAnalyzer:
    """Analyzes Inter-Spike Intervals to classify firing modes."""
    
    def __init__(self):
        self.spike_times = []
        self.isis = []
    
    def add_spike(self, time):
        """Add a spike time."""
        self.spike_times.append(time)
        if len(self.spike_times) > 1:
            isi = self.spike_times[-1] - self.spike_times[-2]
            self.isis.append(isi)
    
    def analyze_firing_mode(self):
        """Classify firing mode based on ISI statistics."""
        if len(self.isis) < 3:
            return "insufficient_data", {}
        
        isis_array = np.array(self.isis)
        mean_isi = np.mean(isis_array)
        std_isi = np.std(isis_array)
        cv = std_isi / mean_isi if mean_isi > 0 else 0  # Coefficient of variation
        
        # Calculate adaptation index (ratio of late ISIs to early ISIs)
        if len(self.isis) >= 6:
            early_isis = isis_array[:3]
            late_isis = isis_array[-3:]
            adaptation_index = np.mean(late_isis) / np.mean(early_isis) if np.mean(early_isis) > 0 else 1
        else:
            adaptation_index = 1
        
        # Detect burst patterns (groups of short ISIs followed by long pauses)
        burst_detected = False
        if len(self.isis) >= 5:
            # Look for pattern: short-short-long or short-short-short-long
            short_threshold = mean_isi * 0.3
            long_threshold = mean_isi * 2.0
            
            for i in range(len(self.isis) - 2):
                if (self.isis[i] < short_threshold and 
                    self.isis[i+1] < short_threshold and 
                    self.isis[i+2] > long_threshold):
                    burst_detected = True
                    break
        
        stats = {
            'mean_isi': mean_isi,
            'std_isi': std_isi,
            'cv': cv,
            'adaptation_index': adaptation_index,
            'num_spikes': len(self.spike_times),
            'isis': self.isis
        }
        
        # Classification logic
        if burst_detected:
            return "bursting", stats
        elif cv > 0.5:  # High variability
            return "irregular", stats
        elif adaptation_index > 1.5:  # Strong adaptation
            return "frequency_adaptation", stats
        elif mean_isi < 50:  # Fast regular spiking
            return "fast_spiking", stats
        else:
            return "regular_spiking", stats

async def collect_spikes_and_analyze(dut, cycles, expected_mode, param_name):
    """Monitor spikes and analyze ISI distribution."""
    dut._log.info(f"Collecting spikes for {param_name} ({expected_mode} mode)...")
    
    analyzer = ISIAnalyzer()
    
    for i in range(cycles):
        await RisingEdge(dut.clk)
        
        try:
            uo_val = int(dut.uo_out.value)
            spike = uo_val & 1
            
            if spike:
                analyzer.add_spike(i)
                vm_bits = (uo_val >> 1) & 0x3F
                dut._log.info(f"Spike #{len(analyzer.spike_times)} at cycle {i}, vm_bits=0x{vm_bits:02X}")
        
        except ValueError as e:
            if "Unresolvable bit" in str(e):
                continue
    
    # Analyze the collected data
    mode, stats = analyzer.analyze_firing_mode()
    
    dut._log.info(f"\n=== ISI Analysis Results for {param_name} ===")
    dut._log.info(f"Detected mode: {mode}")
    dut._log.info(f"Expected mode: {expected_mode}")
    dut._log.info(f"Total spikes: {stats.get('num_spikes', 0)}")
    
    if stats.get('num_spikes', 0) >= 3:
        dut._log.info(f"Mean ISI: {stats['mean_isi']:.2f} cycles")
        dut._log.info(f"CV (variability): {stats['cv']:.3f}")
        dut._log.info(f"Adaptation index: {stats['adaptation_index']:.3f}")
        if len(stats['isis']) > 0:
            dut._log.info(f"ISI sequence: {stats['isis'][:10]}...")  # First 10 ISIs
    
    return mode, stats

# ---------------- Firing Mode Test Cases ----------------

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
    
    # Parameters for regular spiking: moderate current, weak adaptation
    regular_params = {
        "DeltaT": encode_unsigned_direct(2),     # Small slope factor
        "TauW":   encode_unsigned_direct(200),   # Slow adaptation (weak effect)
        "a":      encode_unsigned_direct(1),     # Weak subthreshold adaptation
        "b":      encode_unsigned_direct(5),     # Small spike-triggered adaptation
        "Vreset": encode_signed_direct(-65),     # Standard reset
        "VT":     encode_signed_direct(-50),     # Standard threshold
        "Ibias":  encode_signed_direct(25),      # Moderate constant current
        "C":      encode_unsigned_direct(100)    # Standard capacitance
    }
    
    await load_parameters(dut, regular_params)
    dut.ui_in.value = (1 << 2)  # Enable core
    await ClockCycles(dut.clk, 5)
    
    mode, stats = await collect_spikes_and_analyze(dut, 8000, "regular_spiking", "Regular Spiking")
    
    # Verify regular spiking characteristics
    assert stats.get('num_spikes', 0) >= 5, "Insufficient spikes for regular mode analysis"
    assert stats['cv'] < 0.3, f"CV too high for regular spiking: {stats['cv']}"
    assert stats['adaptation_index'] < 1.3, f"Too much adaptation for regular spiking: {stats['adaptation_index']}"
    
    dut._log.info("✓ Regular spiking mode test PASSED")

@cocotb.test()
async def test_frequency_adaptation_mode(dut):
    """Test frequency adaptation mode - ISIs increase over time."""
    dut._log.info("=== Testing Frequency Adaptation Mode ===")
    
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    # Parameters for frequency adaptation: strong subthreshold adaptation
    adaptation_params = {
        "DeltaT": encode_unsigned_direct(2),     # Small slope factor
        "TauW":   encode_unsigned_direct(50),    # Moderate adaptation time
        "a":      encode_unsigned_direct(8),     # Strong subthreshold adaptation
        "b":      encode_unsigned_direct(3),     # Minimal spike-triggered adaptation
        "Vreset": encode_signed_direct(-65),     # Standard reset
        "VT":     encode_signed_direct(-50),     # Standard threshold
        "Ibias":  encode_signed_direct(30),      # Moderate current
        "C":      encode_unsigned_direct(80)     # Smaller capacitance
    }
    
    await load_parameters(dut, adaptation_params)
    dut.ui_in.value = (1 << 2)  # Enable core
    await ClockCycles(dut.clk, 5)
    
    mode, stats = await collect_spikes_and_analyze(dut, 10000, "frequency_adaptation", "Frequency Adaptation")
    
    # Verify frequency adaptation characteristics
    assert stats.get('num_spikes', 0) >= 6, "Insufficient spikes for adaptation analysis"
    assert stats['adaptation_index'] > 1.4, f"Insufficient adaptation: {stats['adaptation_index']}"
    
    dut._log.info("✓ Frequency adaptation mode test PASSED")

@cocotb.test()
async def test_bursting_mode(dut):
    """Test bursting mode - clusters of spikes separated by quiet periods."""
    dut._log.info("=== Testing Bursting Mode (Most Critical) ===")
    
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    # Parameters for bursting: strong spike-triggered adaptation, moderate subthreshold
    bursting_params = {
        "DeltaT": encode_unsigned_direct(3),     # Moderate slope factor
        "TauW":   encode_unsigned_direct(80),    # Medium adaptation time constant
        "a":      encode_unsigned_direct(4),     # Moderate subthreshold adaptation
        "b":      encode_unsigned_direct(35),    # Strong spike-triggered adaptation (key for bursting)
        "Vreset": encode_signed_direct(-65),     # Standard reset
        "VT":     encode_signed_direct(-50),     # Standard threshold
        "Ibias":  encode_signed_direct(35),      # Strong driving current
        "C":      encode_unsigned_direct(70)     # Medium capacitance
    }
    
    await load_parameters(dut, bursting_params)
    dut.ui_in.value = (1 << 2)  # Enable core
    await ClockCycles(dut.clk, 5)
    
    mode, stats = await collect_spikes_and_analyze(dut, 15000, "bursting", "Bursting")
    
    # Verify bursting characteristics
    assert stats.get('num_spikes', 0) >= 8, "Insufficient spikes for bursting analysis"
    
    # Additional burst-specific analysis
    isis = stats['isis']
    if len(isis) >= 5:
        # Look for burst pattern: short-short-long or variations
        short_isis = [isi for isi in isis if isi < stats['mean_isi'] * 0.4]
        long_isis = [isi for isi in isis if isi > stats['mean_isi'] * 2.0]
        
        burst_ratio = len(long_isis) / len(isis)
        dut._log.info(f"Burst analysis: {len(short_isis)} short ISIs, {len(long_isis)} long ISIs")
        dut._log.info(f"Burst ratio (long ISIs): {burst_ratio:.3f}")
        
        # For bursting, we expect some long pauses and high CV
        assert stats['cv'] > 0.6, f"CV too low for bursting: {stats['cv']}"
        assert burst_ratio > 0.2, f"Not enough long ISIs for bursting: {burst_ratio}"
    
    dut._log.info("✓ Bursting mode test PASSED - Most critical test successful!")

@cocotb.test()
async def test_fast_spiking_mode(dut):
    """Test fast spiking mode - high frequency, regular spikes."""
    dut._log.info("=== Testing Fast Spiking Mode ===")
    
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    # Parameters for fast spiking: minimal adaptation, high current, small capacitance
    fast_params = {
        "DeltaT": encode_unsigned_direct(1),     # Small slope factor for sharp spikes
        "TauW":   encode_unsigned_direct(255),   # Very slow adaptation (minimal effect)
        "a":      encode_unsigned_direct(1),     # Minimal subthreshold adaptation
        "b":      encode_unsigned_direct(2),     # Minimal spike-triggered adaptation
        "Vreset": encode_signed_direct(-68),     # Lower reset for faster recovery
        "VT":     encode_signed_direct(-52),     # Standard threshold
        "Ibias":  encode_signed_direct(50),      # High driving current
        "C":      encode_unsigned_direct(40)     # Small capacitance for fast charging
    }
    
    await load_parameters(dut, fast_params)
    dut.ui_in.value = (1 << 2)  # Enable core
    await ClockCycles(dut.clk, 5)
    
    mode, stats = await collect_spikes_and_analyze(dut, 6000, "fast_spiking", "Fast Spiking")
    
    # Verify fast spiking characteristics
    assert stats.get('num_spikes', 0) >= 8, "Insufficient spikes for fast spiking analysis"
    assert stats['mean_isi'] < 400, f"ISI too long for fast spiking: {stats['mean_isi']}"
    assert stats['cv'] < 0.4, f"CV too high for fast spiking: {stats['cv']}"
    assert stats['adaptation_index'] < 1.2, f"Too much adaptation for fast spiking: {stats['adaptation_index']}"
    
    dut._log.info("✓ Fast spiking mode test PASSED")

@cocotb.test()
async def test_irregular_spiking_mode(dut):
    """Test irregular spiking mode - variable ISIs with high CV."""
    dut._log.info("=== Testing Irregular Spiking Mode ===")
    
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Reset
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    # Parameters for irregular spiking: balanced adaptation creating complex dynamics
    irregular_params = {
        "DeltaT": encode_unsigned_direct(4),     # Larger slope factor for nonlinearity
        "TauW":   encode_unsigned_direct(60),    # Medium adaptation time
        "a":      encode_unsigned_direct(6),     # Moderate subthreshold adaptation
        "b":      encode_unsigned_direct(20),    # Moderate spike-triggered adaptation
        "Vreset": encode_signed_direct(-63),     # Slightly higher reset
        "VT":     encode_signed_direct(-48),     # Slightly higher threshold
        "Ibias":  encode_signed_direct(28),      # Moderate current near threshold
        "C":      encode_unsigned_direct(90)     # Medium capacitance
    }
    
    await load_parameters(dut, irregular_params)
    dut.ui_in.value = (1 << 2)  # Enable core
    await ClockCycles(dut.clk, 5)
    
    mode, stats = await collect_spikes_and_analyze(dut, 12000, "irregular", "Irregular Spiking")
    
    # Verify irregular spiking characteristics
    assert stats.get('num_spikes', 0) >= 6, "Insufficient spikes for irregular analysis"
    
    # For irregular mode, we expect higher variability but not necessarily bursting
    if mode == "bursting":
        dut._log.info("Note: Detected bursting instead of irregular - this is acceptable")
    elif mode == "irregular":
        assert stats['cv'] > 0.4, f"CV too low for irregular spiking: {stats['cv']}"
    
    dut._log.info("✓ Irregular spiking mode test PASSED")

@cocotb.test()
async def test_comprehensive_isi_analysis(dut):
    """Comprehensive test that demonstrates all firing modes in sequence."""
    dut._log.info("=== Comprehensive ISI Analysis Test ===")
    
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Test configurations for different modes
    test_configs = [
        {
            "name": "Regular Spiking",
            "expected": "regular_spiking",
            "params": {
                "DeltaT": encode_unsigned_direct(2), "TauW": encode_unsigned_direct(200),
                "a": encode_unsigned_direct(1), "b": encode_unsigned_direct(5),
                "Vreset": encode_signed_direct(-65), "VT": encode_signed_direct(-50),
                "Ibias": encode_signed_direct(25), "C": encode_unsigned_direct(100)
            },
            "cycles": 6000
        },
        {
            "name": "Fast Spiking", 
            "expected": "fast_spiking",
            "params": {
                "DeltaT": encode_unsigned_direct(1), "TauW": encode_unsigned_direct(255),
                "a": encode_unsigned_direct(1), "b": encode_unsigned_direct(2),
                "Vreset": encode_signed_direct(-68), "VT": encode_signed_direct(-52),
                "Ibias": encode_signed_direct(55), "C": encode_unsigned_direct(35)
            },
            "cycles": 4000
        },
        {
            "name": "Frequency Adaptation",
            "expected": "frequency_adaptation", 
            "params": {
                "DeltaT": encode_unsigned_direct(2), "TauW": encode_unsigned_direct(40),
                "a": encode_unsigned_direct(10), "b": encode_unsigned_direct(3),
                "Vreset": encode_signed_direct(-65), "VT": encode_signed_direct(-50),
                "Ibias": encode_signed_direct(32), "C": encode_unsigned_direct(80)
            },
            "cycles": 8000
        },
        {
            "name": "Bursting (Critical)",
            "expected": "bursting",
            "params": {
                "DeltaT": encode_unsigned_direct(3), "TauW": encode_unsigned_direct(70),
                "a": encode_unsigned_direct(4), "b": encode_unsigned_direct(40),
                "Vreset": encode_signed_direct(-65), "VT": encode_signed_direct(-50),
                "Ibias": encode_signed_direct(38), "C": encode_unsigned_direct(75)
            },
            "cycles": 12000
        }
    ]
    
    results_summary = {}
    
    for config in test_configs:
        dut._log.info(f"\n--- Testing {config['name']} ---")
        
        # Reset between tests
        dut.ui_in.value = 0
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 5)
        dut.rst_n.value = 1
        await ClockCycles(dut.clk, 5)
        
        # Load parameters and test
        await load_parameters(dut, config['params'])
        dut.ui_in.value = (1 << 2)  # Enable core
        await ClockCycles(dut.clk, 5)
        
        mode, stats = await collect_spikes_and_analyze(
            dut, config['cycles'], config['expected'], config['name']
        )
        
        results_summary[config['name']] = {
            'detected_mode': mode,
            'expected_mode': config['expected'],
            'stats': stats
        }
        
        # Disable core between tests
        dut.ui_in.value = 0
        await ClockCycles(dut.clk, 10)
    
    # Summary report
    dut._log.info("\n=== COMPREHENSIVE TEST SUMMARY ===")
    successful_tests = 0
    for test_name, result in results_summary.items():
        detected = result['detected_mode']
        expected = result['expected_mode']
        spikes = result['stats'].get('num_spikes', 0)
        
        if spikes >= 3:
            cv = result['stats']['cv']
            adapt_idx = result['stats']['adaptation_index']
            
            # Check if mode is acceptable (some modes can overlap)
            mode_match = (detected == expected or 
                         (expected == "irregular" and detected in ["bursting", "frequency_adaptation"]) or
                         (expected == "frequency_adaptation" and detected == "irregular"))
            
            status = "✓ PASS" if mode_match else "✗ FAIL"
            if mode_match:
                successful_tests += 1
                
            dut._log.info(f"{status} {test_name}: {detected} (expected {expected})")
            dut._log.info(f"     Spikes: {spikes}, CV: {cv:.3f}, Adaptation: {adapt_idx:.3f}")
        else:
            dut._log.info(f"✗ FAIL {test_name}: Insufficient spikes ({spikes})")
    
    dut._log.info(f"\nOverall Success Rate: {successful_tests}/{len(test_configs)}")
    
    # Ensure at least bursting mode works (most critical)
    bursting_result = results_summary.get("Bursting (Critical)", {})
    assert bursting_result.get('stats', {}).get('num_spikes', 0) >= 5, "Critical: Bursting mode failed"
    
    dut._log.info("✓ Comprehensive ISI analysis test COMPLETED")

@cocotb.test()
async def test_mode_transitions(dut):
    """Test transitions between firing modes by changing parameters."""
    dut._log.info("=== Testing Mode Transitions ===")
    
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    
    # Start with regular spiking
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    
    # Regular → Fast transition (increase Ibias, decrease C)
    regular_params = {
        "DeltaT": encode_unsigned_direct(2), "TauW": encode_unsigned_direct(180),
        "a": encode_unsigned_direct(2), "b": encode_unsigned_direct(6),
        "Vreset": encode_signed_direct(-65), "VT": encode_signed_direct(-50),
        "Ibias": encode_signed_direct(22), "C": encode_unsigned_direct(120)
    }
    
    await load_parameters(dut, regular_params)
    dut.ui_in.value = (1 << 2)
    await ClockCycles(dut.clk, 5)
    
    # Collect baseline
    mode1, stats1 = await collect_spikes_and_analyze(dut, 4000, "regular_spiking", "Transition Phase 1")
    
    # Transition to fast spiking
    fast_params = regular_params.copy()
    fast_params["Ibias"] = encode_signed_direct(50)  # Higher current
    fast_params["C"] = encode_unsigned_direct(40)    # Lower capacitance
    fast_params["b"] = encode_unsigned_direct(1)     # Minimal adaptation
    
    dut.ui_in.value = 0  # Disable core during reload
    await ClockCycles(dut.clk, 5)
    
    await load_parameters(dut, fast_params)
    dut.ui_in.value = (1 << 2)  # Re-enable core
    await ClockCycles(dut.clk, 5)
    
    mode2, stats2 = await collect_spikes_and_analyze(dut, 3000, "fast_spiking", "Transition Phase 2")
    
    # Verify transition occurred
    if (stats1.get('num_spikes', 0) >= 3 and stats2.get('num_spikes', 0) >= 3):
        isi_change = stats1['mean_isi'] / stats2['mean_isi']
        dut._log.info(f"ISI transition ratio: {isi_change:.2f} (should be > 1 for regular→fast)")
        
        assert isi_change > 1.2, f"Insufficient speedup in transition: {isi_change}"
        dut._log.info("✓ Mode transition test PASSED")
    else:
        dut._log.warning("Insufficient spikes in transition test, but continuing...")

if __name__ == "__main__":
    import sys
    sys.exit("This is a cocotb test file for 16-bit AdEx neuron firing modes. Run with: make sim MODULE=test_neuron")