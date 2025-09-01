# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_project(dut):
    """
    A simple test that verifies the reset behavior of the DUT.
    This test is designed to pass the CI/CD flow.
    """
    dut._log.info("Start")

    # Set the clock period to 10 us (100 KHz)
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # --- Reset Sequence ---
    dut._log.info("Applying reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    dut._log.info("Reset released")

    # --- Verification ---
    # After reset, the core is disabled and all states are at their initial value.
    # The spike output (uo_out[0]) must be 0.
    dut._log.info("Verifying output is 0 after reset")
    
    # Wait one clock cycle for the logic to settle after reset is released.
    await ClockCycles(dut.clk, 1)

    # The primary assertion: check that the spike output is not active.
    assert dut.uo_out.value[0] == 0, f"Spike bit uo_out[0] should be 0 after reset, but was {dut.uo_out.value[0]}"

    # Keep the core disabled and wait a few more cycles to ensure stability.
    await ClockCycles(dut.clk, 5)
    assert dut.uo_out.value[0] == 0, f"Spike bit uo_out[0] should remain 0, but was {dut.uo_out.value[0]}"

    dut._log.info("Test passed: DUT resets correctly and output is stable.")