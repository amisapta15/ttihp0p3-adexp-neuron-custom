/*
 * Copyright (c) 2025 Saptarshi Ghosh
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Uncomment to switch back to LUT16
//`define USE_LUT16

module tt_um_dpi_adexp (
    input  wire [7:0] ui_in,     // dedicated inputs
    output wire [7:0] uo_out,    // dedicated outputs
    input  wire [7:0] uio_in,    // bidirectional inputs
    output wire [7:0] uio_out,   // bidirectional outputs
    output wire [7:0] uio_oe,    // bidirectional enable
    input  wire       ena,       // enable signal
    input  wire       clk,       // clock
    input  wire       rst_n      // active-low reset
);

    // tie off unused bidirectional outputs
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // instantiate the neuron core
    adex_neuron_system_tt_lut32 core (
        .clk    (clk),
        .rst_n  (rst_n),
        .ui_in  (ui_in),
        .uo_out (uo_out),
        .uio_in (uio_in)
        // no uio_out/uio_oe needed in core, tied at wrapper
    );

endmodule