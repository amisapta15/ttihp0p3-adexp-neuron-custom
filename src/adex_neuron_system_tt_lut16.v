`default_nettype none
module adex_neuron_system_tt_lut16 (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] ui_in,
    input  wire [7:0] uio_in,
    output wire [7:0] uo_out,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe
);
    // Example state regs (vmem + adaptation)
    reg signed [15:0] vmem;
    reg signed [15:0] w;
    reg spike;

    // coarse exp() LUT: 16-entry
    function signed [15:0] exp_q;
        input signed [15:0] arg;
        reg integer idx;
        reg signed [15:0] val;
        integer span;
        reg signed [15:0] RANGE_MIN;
        reg signed [15:0] RANGE_MAX;
        begin
            RANGE_MIN = (-16'sd4) <<< 12;
            RANGE_MAX = ( 16'sd8) <<< 12;
            span = 16;
            if (arg < RANGE_MIN) begin
                idx = 0;
            end else if (arg > RANGE_MAX) begin
                idx = span - 1;
            end else begin
                idx = ((arg - RANGE_MIN) * span) / (RANGE_MAX - RANGE_MIN + 1);
            end

            case (idx)
                0:  val = (16'sd0)   <<< 12;
                1:  val = (16'sd1)   <<< 12;
                2:  val = (16'sd2)   <<< 12;
                3:  val = (16'sd4)   <<< 12;
                4:  val = (16'sd6)   <<< 12;
                5:  val = (16'sd9)   <<< 12;
                6:  val = (16'sd13)  <<< 12;
                7:  val = (16'sd18)  <<< 12;
                8:  val = (16'sd25)  <<< 12;
                9:  val = (16'sd35)  <<< 12;
                10: val = (16'sd50)  <<< 12;
                11: val = (16'sd70)  <<< 12;
                12: val = (16'sd100) <<< 12;
                13: val = (16'sd140) <<< 12;
                14: val = (16'sd200) <<< 12;
                default: val = (16'sd280) <<< 12;
            endcase
            exp_q = val;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vmem <= 0;
            w    <= 0;
            spike <= 0;
        end else begin
            vmem <= vmem + $signed(ui_in);
            w    <= w + $signed(uio_in);
            spike <= (vmem > 1000);
        end
    end

    assign uo_out  = {spike, vmem[6:0]};
    assign uio_out = w[7:0];
    assign uio_oe  = 8'hFF;
endmodule
