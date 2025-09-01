// ============================================================================
// Adaptive Exponential (AdEx) Neuron System with LUT32 Exponential Approximation
// Cleaned and Lint-Safe Version (Q8.7 arithmetic)
// ----------------------------------------------------------------------------
// - Ports match wrapper: clk, rst_n, ui_in[7:0], uo_out[7:0], uio_in[7:0]
// - Active-low reset (rst_n)
// - Exponential LUT is Q8.7, domain clamped to ±4.5 so all entries fit 15-bit
// - All helper functions are 'automatic' and use 32-bit intermediates
// - No inline initializations inside functions
// - Known outputs on reset; no X propagation
// ----------------------------------------------------------------------------
module adex_neuron_system_tt_lut32 (
    input        clk,
    input        rst_n,     // active-low reset
    input  [7:0] ui_in,     // [4]=load_mode, [3]=load_strobe, [2]=enable
    output reg [7:0] uo_out, // bit0=spike, others 0
    input  [7:0] uio_in     // load nibble on [3:0]
);

    // ---------------- Fixed-Point Format ----------------
    // Q8.7 signed for state/params: 15-bit signed range [-16384 .. +16383]
    // 1.0 -> 128
    // ----------------------------------------------------

    // ---------------- Internal State --------------------
    reg signed [14:0] v_q;        // membrane potential (Q8.7)
    reg signed [14:0] w_q;        // adaptation variable (Q8.7)

    // Parameter bytes loaded via nibble interface (8 params)
    reg [7:0] params [0:7];

    // User-visible compact debug registers (optional)
    reg [7:0] vm8_reg;
    reg [7:0] w8_reg;

    // Loader FSM
    reg [3:0]  nibble_hi;
    reg        nibble_phase;      // 0: expect HI nibble, 1: expect LO nibble
    reg [2:0]  param_index;

    // ---------------- Constants (Q8.7) ------------------
    // Leak reversal and conductance (tunable if desired via params a/gL)
    localparam signed [14:0] EL_q = -15'sd8960;  // -70 * 128
    localparam signed [14:0] gL_q =  15'sd1280;  //  10 * 128

    // ---------------- Parameter Mapping -----------------
    // params[0] : DeltaT (signed)
    // params[1] : TauW   (unsigned, time constant)
    // params[2] : a      (unsigned)
    // params[3] : b      (unsigned)
    // params[4] : Vreset (signed)
    // params[5] : VT     (signed)
    // params[6] : Ibias  (signed)
    // params[7] : C      (unsigned, capacitance)
    wire signed [14:0] DeltaT_q  = u8_to_signed_q(params[0]);
    wire        [14:0] TauW_q    = u8_to_q_unsigned(params[1]);
    wire        [14:0] a_q       = u8_to_q_unsigned(params[2]);
    wire        [14:0] b_q       = u8_to_q_unsigned(params[3]);
    wire signed [14:0] Vreset_q  = u8_to_signed_q(params[4]);
    wire signed [14:0] VT_q      = u8_to_signed_q(params[5]);
    wire signed [14:0] Ibias_q   = u8_to_signed_q(params[6]);
    wire        [14:0] C_q       = u8_to_q_unsigned(params[7]);

    // Spike detection (combinational)
    wire spike = (v_q > VT_q);

    // ---------------- Helper Functions ------------------

    // Saturating conversion Q8.7 signed -> u8 (0..255), with 128 representing 0.0
    function automatic [7:0] sat_to_u8(input signed [14:0] x);
        reg signed [14:0] t;
        begin
            // x>>>7 is integer value; +128 recenters around 0 => 128
            t = (x >>> 7) + 15'sd128;
            if (t < 0)       sat_to_u8 = 8'd0;
            else if (t > 255) sat_to_u8 = 8'd255;
            else              sat_to_u8 = t[7:0];
        end
    endfunction

    // Unsigned u8 -> Q8.7 signed (0..255) maps to (0..255)*128 - 128*128
    function automatic signed [14:0] u8_to_signed_q(input [7:0] x);
        // Map 128 -> 0.0
        reg signed [15:0] sx;
        begin
            sx = $signed({1'b0, x}) - 16'sd128;
            u8_to_signed_q = $signed(sx) <<< 7;
        end
    endfunction

    // Unsigned u8 -> Q8.7 unsigned (0..255)*128
    function automatic signed [14:0] u8_to_q_unsigned(input [7:0] x);
        begin
            u8_to_q_unsigned = $signed({1'b0, x}) <<< 7;
        end
    endfunction

    // Q8.7 * Q8.7 -> Q8.7
    function automatic signed [14:0] qmul_eff(input signed [14:0] a, input signed [14:0] b);
        reg signed [31:0] tmp;
        begin
            tmp = $signed(a) * $signed(b); // Q16.14
            qmul_eff = tmp >>> 7;          // back to Q8.7
        end
    endfunction

    // Q8.7 / Q8.7 -> Q8.7
    function automatic signed [14:0] qdiv_eff(input signed [14:0] a, input signed [14:0] b);
        reg signed [31:0] tmp;
        begin
            // Protect against divide-by-zero (return 0)
            if (b == 0) begin
                qdiv_eff = 15'sd0;
            end else begin
                tmp = ($signed(a) <<< 7) / $signed(b);
                qdiv_eff = tmp[14:0];
            end
        end
    endfunction

    // Exponential approximation: exp(x), where x is Q8.7
    // LUT32 over [-4.5, +4.5] (Q8.7: [-576, +576]), entries are Q8.7 of e^x
    function automatic signed [14:0] exp_q_optimized(input signed [14:0] arg_in);
        integer idx;
        reg signed [14:0] RANGE_MIN;
        reg signed [14:0] RANGE_MAX;
        reg signed [14:0] lut [0:31];  // Q8.7 values (no shifts)
        reg signed [31:0] num, den;
        begin
            RANGE_MIN = -15'sd576;  // -4.5 * 128
            RANGE_MAX =  15'sd576;  // +4.5 * 128

            // e^x * 128 for x ∈ [-4.5, +4.5], 32 uniform steps
            // (All values fit in signed 15-bit)
            lut[0]  = 15'sd1;     lut[1]  = 15'sd2;     lut[2]  = 15'sd3;     lut[3]  = 15'sd3;
            lut[4]  = 15'sd5;     lut[5]  = 15'sd6;     lut[6]  = 15'sd8;     lut[7]  = 15'sd11;
            lut[8]  = 15'sd15;    lut[9]  = 15'sd19;    lut[10] = 15'sd26;    lut[11] = 15'sd35;
            lut[12] = 15'sd46;    lut[13] = 15'sd62;    lut[14] = 15'sd83;    lut[15] = 15'sd111;
            lut[16] = 15'sd148;   lut[17] = 15'sd198;   lut[18] = 15'sd264;   lut[19] = 15'sd354;
            lut[20] = 15'sd473;   lut[21] = 15'sd632;   lut[22] = 15'sd845;   lut[23] = 15'sd1129;
            lut[24] = 15'sd1510;  lut[25] = 15'sd2018;  lut[26] = 15'sd2698;  lut[27] = 15'sd3607;
            lut[28] = 15'sd4823;  lut[29] = 15'sd6447;  lut[30] = 15'sd8619;  lut[31] = 15'sd11522;

            if (arg_in <= RANGE_MIN) begin
                exp_q_optimized = lut[0];
            end else if (arg_in >= RANGE_MAX) begin
                exp_q_optimized = lut[31];
            end else begin
                num = ($signed(arg_in) - $signed(RANGE_MIN)) * 32;
                den = ($signed(RANGE_MAX) - $signed(RANGE_MIN)); // 1152
                idx = num / den; // 0..31
                if (idx < 0) idx = 0; else if (idx > 31) idx = 31;
                exp_q_optimized = lut[idx];
            end
        end
    endfunction

    // ---------------- Nibble Loader ---------------------
    // Protocol:
    //   ui_in[4] = load_mode (hold high during loading)
    //   ui_in[3] = load_strobe: each pulse clocks a 4-bit nibble from uio_in[3:0]
    //   Two strobes per parameter: HI nibble first, then LO nibble.
    //   params[0..7] get filled sequentially; extra writes wrap around.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nibble_hi    <= 4'd0;
            nibble_phase <= 1'b0;
            param_index  <= 3'd0;
        end else begin
            if (ui_in[4]) begin
                if (ui_in[3]) begin
                    if (!nibble_phase) begin
                        nibble_hi    <= uio_in[3:0];
                        nibble_phase <= 1'b1;
                    end else begin
                        params[param_index] <= {nibble_hi, uio_in[3:0]};
                        param_index         <= param_index + 3'd1;
                        nibble_phase        <= 1'b0;
                    end
                end
            end else begin
                // leaving load mode resets sequence
                nibble_phase <= 1'b0;
                param_index  <= 3'd0;
            end
        end
    end

    // ---------------- Neuron Dynamics -------------------
    // v' = ( -gL*(v-EL) + gL*DeltaT*exp((v-VT)/DeltaT) - w + Ibias ) / C
    // w' = ( a*(v-EL) - w ) / TauW
    // Discrete Euler step per clock
    reg signed [31:0] leak32, exp32, drive32, adap32;
    wire signed [14:0] exp_arg  = qdiv_eff(v_q - VT_q, DeltaT_q);
    wire signed [14:0] exp_term = exp_q_optimized(exp_arg); // Q8.7

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_q     <= -15'sd8320; // -65 * 128
            w_q     <=  15'sd0;
            vm8_reg <= 8'd0;
            w8_reg  <= 8'd0;
            uo_out  <= 8'd0;
        end else begin
            // default outputs each cycle
            vm8_reg <= sat_to_u8(v_q);
            w8_reg  <= sat_to_u8(w_q);
            uo_out  <= {7'b0, spike};

            if (ui_in[2]) begin
                if (spike) begin
                    // Reset & spike-triggered adaptation
                    v_q <= Vreset_q;
                    w_q <= w_q + b_q;
                end else begin
                    // Compute dV
                    leak32 = qmul_eff(gL_q, (EL_q - v_q));                       // Q8.7
                    exp32  = qmul_eff(gL_q, qmul_eff(DeltaT_q, exp_term));       // Q8.7
                    drive32= $signed(leak32) + $signed(exp32) - $signed(w_q) + $signed(Ibias_q);
                    v_q    <= v_q + qdiv_eff(drive32[14:0], C_q);

                    // Compute dW
                    adap32 = qdiv_eff( qmul_eff(a_q, (v_q - EL_q)) - w_q, TauW_q );
                    w_q    <= w_q + adap32[14:0];
                end
            end
        end
    end

endmodule
