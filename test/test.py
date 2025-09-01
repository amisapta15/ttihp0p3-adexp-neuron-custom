import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge

# ---------------- Helpers ----------------
def encode_signed(real_value):
    """Encodes a real-world signed value into 8-bit hardware format (offset +128).
       Clamps to [0,255]."""
    v = int(round(real_value + 128))
    if v < 0: v = 0
    if v > 255: v = 255
    return v

def encode_unsigned(real_value):
    """Encodes an unsigned value into 8-bit hardware format. Clamps to [0,255]."""
    v = int(round(real_value))
    if v < 0: v = 0
    if v > 255: v = 255
    return v

async def load_nibble(dut, nibble, param_name="", load_mode_mask=(1 << 4), use_load_mode=True):
    """
    Drive uio_in (low 4 bits) and assert load_enable (ui_in[3]) simultaneously,
    then wait one rising edge so the DUT samples them on that posedge.
    This is robust and easy to reason about.
    """
    # prepare integer versions
    ui_int = int(dut.ui_in.value)
    # drive nibble on uio_in (careful: uio_in is 8-bit bus, put nibble in [3:0])
    dut.uio_in.value = int(nibble) & 0xF

    # assert load_mode if requested (keeps prior load_mode bit)
    if use_load_mode:
        ui_int = ui_int | load_mode_mask
    # assert load_enable (bit 3)
    ui_int = ui_int | (1 << 3)

    dut.ui_in.value = ui_int
    await RisingEdge(dut.clk)

    # de-assert load_enable (but keep load_mode if requested)
    ui_int = int(dut.ui_in.value)
    ui_int = ui_int & ~(1 << 3)
    dut.ui_in.value = ui_int

    # give one falling edge to let things settle (optional)
    await FallingEdge(dut.clk)

async def load_parameters(dut, params):
    """Loads parameters into the DUT using the nibble FSM."""
    dut._log.info("Beginning parameter loading...")

    LOAD_MODE = 1 << 4
    param_order = ["DeltaT", "TauW", "a", "b", "Vreset", "VT", "Ibias", "C"]

    # 1) enter load mode (keep load_mode asserted while loading)
    dut.ui_in.value = LOAD_MODE
    await RisingEdge(dut.clk)

    # 2) load each parameter (two nibbles)
    for name in param_order:
        val = params.get(name, 0) & 0xFF
        hi = (val >> 4) & 0xF
        lo = val & 0xF
        await load_nibble(dut, hi, name, load_mode_mask=LOAD_MODE, use_load_mode=True)
        await load_nibble(dut, lo, name, load_mode_mask=LOAD_MODE, use_load_mode=True)

    # 3) footer nibble 0xF to commit
    await load_nibble(dut, 0xF, "Footer", load_mode_mask=LOAD_MODE, use_load_mode=True)

    # 4) exit load mode (deassert load_mode) and wait a couple cycles
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 2)
    dut._log.info("Parameter loading complete.")

# ---------------- Main Test ----------------
@cocotb.test()
async def test_neuron_spike(dut):
    dut._log.info("Start test_neuron_spike")

    ENABLE_CORE = 1 << 2
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Resetting DUT")
    # ensure clean default
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    dut._log.info("Reset complete")

    # Parameters that should produce spiking
    params_to_load = {
        "DeltaT": encode_signed(2),       # 2 -> 130
        "TauW":   encode_unsigned(100),
        "a":      encode_unsigned(2),
        "b":      encode_unsigned(40),
        "Vreset": encode_signed(-65),     # -65 -> 63
        "VT":     encode_signed(-50),     # -50 -> 78
        "Ibias":  encode_signed(72),      # 72 -> 200
        "C":      encode_unsigned(200)
    }

    await load_parameters(dut, params_to_load)

    # Enable core
    dut._log.info("Enabling core and waiting for spike...")
    dut.ui_in.value = ENABLE_CORE

    spike_detected = False
    max_cycles = 5000

    for i in range(max_cycles):
        await RisingEdge(dut.clk)
        # read spike bit 0 in a robust way:
        if (int(dut.uo_out.value) & 1) != 0:
            dut._log.info(f"Spike detected on cycle {i+1}")
            spike_detected = True
            break

    assert spike_detected, f"Neuron did NOT spike within {max_cycles} cycles."


# # SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# # SPDX-License-Identifier: Apache-2.0

# import cocotb
# from cocotb.clock import Clock
# from cocotb.triggers import ClockCycles

# @cocotb.test()
# async def test_project(dut):
#     """
#     A simple test that verifies the reset behavior of the DUT.
#     This test is designed to pass the CI/CD flow.
#     """
#     dut._log.info("Start")

#     # Set the clock period to 10 us (100 KHz)
#     clock = Clock(dut.clk, 10, units="us")
#     cocotb.start_soon(clock.start())

#     # --- Reset Sequence ---
#     dut._log.info("Applying reset")
#     dut.ena.value = 1
#     dut.ui_in.value = 0
#     dut.uio_in.value = 0
#     dut.rst_n.value = 0
#     await ClockCycles(dut.clk, 10)
#     dut.rst_n.value = 1
#     dut._log.info("Reset released")

#     # --- Verification ---
#     # After reset, the core is disabled and all states are at their initial value.
#     # The spike output (uo_out[0]) must be 0.
#     dut._log.info("Verifying output is 0 after reset")
    
#     # Wait one clock cycle for the logic to settle after reset is released.
#     await ClockCycles(dut.clk, 1)

#     # The primary assertion: check that the spike output is not active.
#     assert dut.uo_out.value[0] == 0, f"Spike bit uo_out[0] should be 0 after reset, but was {dut.uo_out.value[0]}"

#     # Keep the core disabled and wait a few more cycles to ensure stability.
#     await ClockCycles(dut.clk, 5)
#     assert dut.uo_out.value[0] == 0, f"Spike bit uo_out[0] should remain 0, but was {dut.uo_out.value[0]}"

#     dut._log.info("Test passed: DUT resets correctly and output is stable.")