`default_nettype none
`timescale 1ns / 1ps

/* This testbench instantiates the DUT and connects wires for cocotb.
*/
module tb ();

  // Dump signals to a VCD file for debugging with gtkwave.
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
    #1;
  end

  // DUT signals
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // IMPORTANT: Replace 'tt_um_your_github_username_adexp_neuron' with the
  // exact name of your top-level Verilog module.
  tt_um_dpi_adexp dut (
      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

endmodule