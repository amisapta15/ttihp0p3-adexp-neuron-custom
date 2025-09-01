`default_nettype none
module adex_neuron_system_tt_lut32 (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] ui_in,
    input  wire [7:0] uio_in,
    output wire [7:0] uo_out,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe
);
    reg signed [15:0] vmem;
    reg signed [15:0] w;
    reg spike;

    // finer exp() LUT: 32-entry
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
            span = 32;
            if (arg < RANGE_MIN) begin
                idx = 0;
            end else if (arg > RANGE_MAX) begin
                idx = span - 1;
            end else begin
                idx = ((arg - RANGE_MIN) * span) / (RANGE_MAX - RANGE_MIN + 1);
            end

            case (idx)
                0:  val = (16'sd0)    <<< 12;
                1:  val = (16'sd1)    <<< 12;
                2:  val = (16'sd1)    <<< 12;
                3:  val = (16'sd2)    <<< 12;
                4:  val = (16'sd3)    <<< 12;
                5:  val = (16'sd4)    <<< 12;
                6:  val = (16'sd6)    <<< 12;
                7:  val = (16'sd8)    <<< 12;
                8:  val = (16'sd11)   <<< 12;
                9:  val = (16'sd15)   <<< 12;
                10: val = (16'sd20)   <<< 12;
                11: val = (16'sd27)   <<< 12;
                12: val = (16'sd36)   <<< 12;
                13: val = (16'sd48)   <<< 12;
                14: val = (16'sd64)   <<< 12;
                15: val = (16'sd85)   <<< 12;
                16: val = (16'sd113)  <<< 12;
                17: val = (16'sd150)  <<< 12;
                18: val = (16'sd200)  <<< 12;
                19: val = (16'sd267)  <<< 12;
                20: val = (16'sd356)  <<< 12;
                21: val = (16'sd475)  <<< 12;
                22: val = (16'sd635)  <<< 12;
                23: val = (16'sd849)  <<< 12;
                24: val = (16'sd1135) <<< 12;
                25: val = (16'sd1517) <<< 12;
                26: val = (16'sd2027) <<< 12;
                27: val = (16'sd2709) <<< 12;
                28: val = (16'sd3624) <<< 12;
                29: val = (16'sd4849) <<< 12;
                30: val = (16'sd6495) <<< 12;
                default: val = (16'sd8700) <<< 12;
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
