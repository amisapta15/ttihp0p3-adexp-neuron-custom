// ============================================================================
// Adaptive Exponential (AdEx) Neuron System with LUT32 Exponential Approximation
// Cleaned and Lint-Safe Version
// ----------------------------------------------------------------------------
// Fixes:
// - LUT widened to 32-bit signed (no truncation warnings)
// - Reset port renamed to rst_n (active-low, matches harness)
// - All temps widened to 32-bit
// - Functions marked automatic
// ----------------------------------------------------------------------------
module adex_neuron_system_tt_lut32 (
    input             clk,
    input             rst_n,       // active-low reset
    input      [7:0]  ui_in,
    output reg [7:0]  uo_out,
    input      [7:0]  uio_in,
    output reg [7:0]  uio_out
);

    // Internal registers
    reg [7:0] vm8_reg, w8_reg;
    reg [7:0] params [0:7];

    // Loader FSM
    reg [3:0] load_nibble;
    reg [2:0] param_index;
    reg       loading;

    // Membrane variables (Q8.7 fixed-point)
    reg signed [31:0] v_q;
    reg signed [31:0] w_q;

    // Parameter wires
    wire signed [31:0] DeltaT_q  = u8_to_signed_q(params[0]);
    wire        [31:0] TauW_q    = u8_to_q_unsigned(params[1]);
    wire        [31:0] a_q       = u8_to_q_unsigned(params[2]);
    wire        [31:0] b_q       = u8_to_q_unsigned(params[3]);
    wire signed [31:0] Vreset_q  = u8_to_signed_q(params[4]);
    wire signed [31:0] VT_q      = u8_to_signed_q(params[5]);
    wire signed [31:0] Ibias_q   = u8_to_signed_q(params[6]);
    wire        [31:0] C_q       = u8_to_q_unsigned(params[7]);

    // Local constants
    localparam signed [31:0] EL_q = -32'sd70 <<< 7;
    localparam signed [31:0] gL_q =  32'sd10 <<< 7;

    // Spike detection
    wire spike = (v_q > VT_q);

    // ---------------- Functions ----------------

    function automatic signed [31:0] exp_q_optimized(input signed [31:0] arg_in);
        integer idx;
        reg signed [31:0] RANGE_MIN;
        reg signed [31:0] RANGE_MAX;
        reg signed [31:0] lut [0:31];
        begin
            RANGE_MIN = -32'sd6 <<< 7;
            RANGE_MAX =  32'sd6 <<< 7;

            lut[0]  = 32'sd1 <<< 7;
            lut[1]  = 32'sd2 <<< 7;
            lut[2]  = 32'sd3 <<< 7;
            lut[3]  = 32'sd4 <<< 7;
            lut[4]  = 32'sd6 <<< 7;
            lut[5]  = 32'sd8 <<< 7;
            lut[6]  = 32'sd11 <<< 7;
            lut[7]  = 32'sd16 <<< 7;
            lut[8]  = 32'sd22 <<< 7;
            lut[9]  = 32'sd30 <<< 7;
            lut[10] = 32'sd45 <<< 7;
            lut[11] = 32'sd65 <<< 7;
            lut[12] = 32'sd95 <<< 7;
            lut[13] = 32'sd135 <<< 7;
            lut[14] = 32'sd200 <<< 7;
            lut[15] = 32'sd300 <<< 7;
            lut[16] = 32'sd440 <<< 7;
            lut[17] = 32'sd650 <<< 7;
            lut[18] = 32'sd950 <<< 7;
            lut[19] = 32'sd1400 <<< 7;
            lut[20] = 32'sd2000 <<< 7;
            lut[21] = 32'sd3000 <<< 7;
            lut[22] = 32'sd4500 <<< 7;
            lut[23] = 32'sd6500 <<< 7;
            lut[24] = 32'sd9500 <<< 7;
            lut[25] = 32'sd14000 <<< 7;
            lut[26] = 32'sd20000 <<< 7;
            lut[27] = 32'sd30000 <<< 7;
            lut[28] = 32'sd45000 <<< 7;
            lut[29] = 32'sd65000 <<< 7;
            lut[30] = 32'sd95000 <<< 7;
            lut[31] = 32'sd130000 <<< 7;

            if (arg_in <= RANGE_MIN)
                exp_q_optimized = lut[0];
            else if (arg_in >= RANGE_MAX)
                exp_q_optimized = lut[31];
            else begin
                idx = (($signed(arg_in) - $signed(RANGE_MIN)) * 32) /
                      ($signed(RANGE_MAX) - $signed(RANGE_MIN));
                exp_q_optimized = lut[idx];
            end
        end
    endfunction

    function automatic signed [31:0] qmul_eff(input signed [31:0] a, input signed [31:0] b);
        reg signed [63:0] tmp;
        begin
            tmp = a * b;
            qmul_eff = tmp >>> 7;
        end
    endfunction

    function automatic signed [31:0] qdiv_eff(input signed [31:0] a, input signed [31:0] b);
        reg signed [63:0] tmp;
        begin
            tmp = (a <<< 7) / b;
            qdiv_eff = tmp[31:0];
        end
    endfunction

    function automatic signed [31:0] u8_to_signed_q(input [7:0] x);
        begin
            u8_to_signed_q = ($signed({1'b0, x}) - 32'sd128) <<< 7;
        end
    endfunction

    function automatic signed [31:0] u8_to_q_unsigned(input [7:0] x);
        begin
            u8_to_q_unsigned = $signed({1'b0, x}) <<< 7;
        end
    endfunction

    function automatic [7:0] sat_to_u8(input signed [31:0] x);
        reg signed [31:0] u;
        begin
            u = (x >>> 7) + 32'sd128;
            if (u < 0) sat_to_u8 = 8'd0;
            else if (u > 255) sat_to_u8 = 8'd255;
            else sat_to_u8 = u[7:0];
        end
    endfunction

    // ---------------- Loader ----------------

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            load_nibble <= 4'd0;
            param_index <= 3'd0;
            loading     <= 1'b0;
        end else begin
            if (ui_in[4] && ui_in[3]) begin
                load_nibble <= uio_in[3:0];
                loading     <= 1'b1;
            end else if (loading) begin
                params[param_index] <= {load_nibble, uio_in[3:0]};
                param_index <= param_index + 1;
                loading <= 1'b0;
            end
        end
    end

    // ---------------- Neuron Dynamics ----------------
    reg signed [31:0] leak, expterm, drive, adap;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_q <= -32'sd65 <<< 7;
            w_q <= 32'sd0;
        end else begin
            if (ui_in[2]) begin
                if (spike) begin
                    v_q <= Vreset_q;
                    w_q <= w_q + b_q;
                end else begin
                    leak    = qmul_eff(gL_q, (EL_q - v_q));
                    expterm = qmul_eff(gL_q,
                                    qmul_eff(DeltaT_q,
                                        exp_q_optimized(qdiv_eff(v_q - VT_q, DeltaT_q))));
                    drive   = leak + expterm - w_q + Ibias_q;
                    v_q     <= v_q + qdiv_eff(drive, C_q);

                    adap = qdiv_eff(qmul_eff(a_q, (v_q - EL_q)) - w_q, TauW_q);
                    w_q  <= w_q + adap;
                end
            end
        end
    end

    // ---------------- Outputs ----------------
    always @(posedge clk) begin
        vm8_reg <= sat_to_u8(v_q);
        w8_reg  <= sat_to_u8(w_q);
        uo_out  <= {7'b0, spike};
        uio_out <= vm8_reg;
    end

    /* verilator lint_off UNUSED */
    reg r_ready;
    /* verilator lint_on UNUSED */

endmodule
