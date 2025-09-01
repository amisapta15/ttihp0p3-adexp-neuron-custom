/*
 * Copyright (c) 2025 Saptarshi Ghosh
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Uncomment to switch back to LUT16
//`define USE_LUT16

module tt_um_dpi_adexp (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

// List all unused inputs to prevent warnings
wire _unused = &{ena, 1'b0};

adex_neuron_system_tt_lut16 core (
    .clk(clk),
    .rst_n(rst_n),
    .ui_in(ui_in),
    .uo_out(uo_out),
    .uio_in(uio_in),
    .uio_out(uio_out),
    .uio_oe(uio_oe)
);

endmodule
