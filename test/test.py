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
    Tests the AdEx neuron by loading a strong positive input current (Ibias)
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
    # We will load a strong Ibias to guarantee a spike, plus the new C parameter.
    # Ibias is param index 6. C is param index 7.
    # New Ibias: 200 (0xC8), which is 200-128=72pA, a strong positive current.
    # New C: 200 (0xC8), the default capacitance value.
    dut._log.info("Beginning parameter loading...")
    
    # 1. Enter load mode and pulse load_enable to start the loader FSM
    dut.ui_in.value = LOAD_MODE
    await load_nibble(dut, 0) # This first pulse is just to enter the SHIFT state.

    # 2. Load dummy values for the first 6 parameters (indices 0-5 -> 12 nibbles)
    dut._log.info("Loading 6 dummy parameters...")
    for i in range(12):
        await load_nibble(dut, 0)

    # 3. Load the Ibias value (200 = 0xC8 = nibbles 0xC and 0x8)
    dut._log.info("Loading Ibias (param 6) = 200 (0xC8)...")
    await load_nibble(dut, 0xC) # High nibble
    await load_nibble(dut, 0x8) # Low nibble

    # 4. Load the C value (200 = 0xC8 = nibbles 0xC and 0x8)
    dut._log.info("Loading C (param 7) = 200 (0xC8)...")
    await load_nibble(dut, 0xC) # High nibble
    await load_nibble(dut, 0x8) # Low nibble

    # 5. Load the footer nibble (0xF) to commit all 8 parameters
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
    max_cycles = 1000  # Max time to wait for a spike

    for i in range(max_cycles):
        await RisingEdge(dut.clk)
        # uo_out[0] is the spike indicator
        if dut.uo_out.value[0] == 1:
            dut._log.info(f"Spike detected on cycle {i+1}!")
            spike_detected = True
            break
    
    # Assert that a spike was actually detected
    assert spike_detected, f"Neuron did not spike within {max_cycles} cycles."