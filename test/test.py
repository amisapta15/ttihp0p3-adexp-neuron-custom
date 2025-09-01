# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge

# Helper function to load a single 4-bit nibble into the DUT
async def load_nibble(dut, nibble):
    """Drives the uio_in bus and pulses load_enable for one clock cycle."""
    dut.uio_in.value = nibble
    await RisingEdge(dut.clk)
    dut.ui_in.value |= (1 << 3) # Assert load_enable
    await RisingEdge(dut.clk)
    dut.ui_in.value &= ~(1 << 3) # De-assert load_enable
    await FallingEdge(dut.clk) # Wait for signals to settle

@cocotb.test()
async def test_neuron_spike(dut):
    """
    Tests the AdEx neuron by loading appropriate parameter values
    and waiting for it to spike.
    """
    dut._log.info("Start")

    # Define control bit positions from the Verilog code
    LOAD_MODE   = 1 << 4
    ENABLE_CORE = 1 << 2

    # Set the clock period to 10 us (100 KHz)
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # --- Reset Sequence ---
    dut._log.info("Resetting DUT")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    dut._log.info("Reset complete")

    # --- Parameter Loading Sequence ---
    # We'll use more appropriate parameter values
    dut._log.info("Beginning parameter loading...")
    
    # 1. Enter load mode and pulse load_enable to start the loader FSM
    dut.ui_in.value = LOAD_MODE
    await load_nibble(dut, 0) # This first pulse is just to enter the SHIFT state.

    # 2. Load appropriate values for all 8 parameters
    # Parameter order: DeltaT, TauW, a, b, Vreset, VT, Ibias, C
    parameter_values = [
        0x82,  # DeltaT: 130 (appropriate value)
        0x64,  # TauW: 100 (appropriate value)
        0x02,  # a: 2 (appropriate value)
        0x28,  # b: 40 (appropriate value)
        0x3F,  # Vreset: 63 (appropriate value)
        0x4E,  # VT: 78 (appropriate value)
        0x90,  # Ibias: 144 (moderate positive current)
        0xC8   # C: 200 (default capacitance value)
    ]

    # Load all parameters
    for param_value in parameter_values:
        high_nibble = (param_value >> 4) & 0xF
        low_nibble = param_value & 0xF
        
        await load_nibble(dut, high_nibble)
        await load_nibble(dut, low_nibble)

    # 5. Load the footer nibble (0xF) to commit all parameters
    dut._log.info("Loading footer to commit...")
    await load_nibble(dut, 0xF)
    
    # 6. Exit load mode
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 2)
    dut._log.info("Parameter loading complete.")

    # --- Run and Wait for Spike ---
    dut._log.info("Enabling core and waiting for a spike...")
    dut.ui_in.value = ENABLE_CORE

    spike_detected = False
    max_cycles = 5000  # Increased max cycles to allow more time for spiking

    for i in range(max_cycles):
        await RisingEdge(dut.clk)
        # uo_out[0] is the spike indicator
        if dut.uo_out.value[0] == 1:
            dut._log.info(f"Spike detected on cycle {i+1}!")
            spike_detected = True
            break
    
    # Assert that a spike was actually detected
    assert spike_detected, f"Neuron did not spike within {max_cycles} cycles."

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