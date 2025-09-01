/*
 * Copyright (c) 2024 Your Name
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

`ifdef USE_LUT16
    adex_neuron_system_tt_lut16 core (
        .clk(clk), .rst_n(rst_n),
        .ui_in(ui_in),
        .uio_in(uio_in),
        .uo_out(uo_out),
        .uio_out(uio_out),
        .uio_oe(uio_oe)
    );
`else
    adex_neuron_system_tt_lut32 core (
        .clk(clk), .rst_n(rst_n),
        .ui_in(ui_in),
        .uio_in(uio_in),
        .uo_out(uo_out),
        .uio_out(uio_out),
        .uio_oe(uio_oe)
    );
`endif
endmodule