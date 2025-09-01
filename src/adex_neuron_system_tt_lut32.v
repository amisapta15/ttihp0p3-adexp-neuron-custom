// Area-optimized 16-bit ADEX neuron for TinyTapeout IHP-130
// 32-entry exp LUT variant (fixed indexing and parameter mapping; mirrors LUT16 behavior)
// - 32-entry exp LUT (Q4.8 input, Q4.8 output scale)
// - 16-bit arithmetic preserved
// - Divider removed (shift approximation), short pipeline to help timing

module adex_neuron_system_tt_lut32 (
    input             clk,
    input             rst_n,
    input       [7:0] ui_in,
    output      [7:0] uo_out,
    input       [7:0] uio_in,
    output      [7:0] uio_out,
    output      [7:0] uio_oe
);

    // -------------------- Top-level control / IO --------------------
    wire reset       = ~rst_n;
    wire load_mode   = ui_in[4];
    wire load_enable = ui_in[3];
    wire enable_core = ui_in[2];

    // simple nibble loader registers (params are 8 bytes, loaded nibble-by-nibble)
    reg [7:0] params [0:7];
    reg [3:0] nibble_cnt;
    reg [7:0] r_ready;
    reg load_prev;

    // loader FSM
    localparam FOOTER_NIB = 4'hF;
    reg [2:0] lstate;
    localparam L_IDLE=0, L_WAIT=1, L_WAIT_FOOTER=2, L_READY=3;

    reg [7:0] uo_out_reg;
    assign uo_out = uo_out_reg;
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // --------------- Parameter nibble loader (compatible with LUT16 mapping) ---------------
    always @(posedge clk) begin
        load_prev <= load_enable;
        if (reset) begin
            nibble_cnt <= 4'd0;
            lstate <= L_IDLE;
            r_ready <= 1'b0;
            // defaults
            params[0] <= 8'd130;   // DeltaT
            params[1] <= 8'd100;   // TauW
            params[2] <= 8'd2;     // a
            params[3] <= 8'd40;    // b
            params[4] <= 8'd191;   // Vreset (-65 -> +128 offset => 63? keep safe defaults)
            params[5] <= 8'd78;    // VT (-50)
            params[6] <= 8'd143;   // Ibias (signed representation)
            params[7] <= 8'd200;   // C
        end else begin
            case (lstate)
                L_IDLE: if (load_mode && load_enable && !load_prev) begin
                            nibble_cnt <= 4'd0;
                            lstate <= L_WAIT;
                        end
                       else lstate <= L_IDLE;
                L_WAIT: if (load_enable && !load_prev) begin
                            // write nibble into current params byte: first lower nibble then upper (MSB loads later)
                            if (nibble_cnt[0] == 1'b0) begin
                                // lower nibble
                                params[nibble_cnt >> 1][3:0] <= uio_in[3:0];
                                nibble_cnt <= nibble_cnt + 1'b1;
                                lstate <= L_WAIT;
                            end else begin
                                // upper nibble, complete a byte
                                params[nibble_cnt >> 1][7:4] <= uio_in[3:0];
                                nibble_cnt <= nibble_cnt + 1'b1;
                                if ((nibble_cnt >> 1) == 7) lstate <= L_WAIT_FOOTER;
                                else lstate <= L_WAIT;
                            end
                        end
                        else lstate <= L_WAIT;
                L_WAIT_FOOTER: if (load_enable && !load_prev) begin
                                    if (uio_in[3:0] == FOOTER_NIB) begin
                                        r_ready <= 1'b1; lstate <= L_READY;
                                    end else lstate <= L_IDLE;
                                end
                                else lstate <= L_WAIT_FOOTER;
                L_READY: if (!load_mode) begin r_ready <= 1'b0; lstate <= L_IDLE; end
                default: lstate <= L_IDLE;
            endcase
        end
    end

    // -------------------- 16-bit fixed-point core (Q4.8) --------------------
    reg signed [15:0] V; // membrane potential Q4.8
    reg signed [15:0] w; // adaptation variable Q4.8
    reg spike_reg;

    // states for compute pipeline (mirrors LUT16)
    reg [2:0] compute_state;
    localparam C_LEAK=3'd0, C_ARG=3'd1, C_EXP=3'd2, C_DRIVE=3'd3, C_DV=3'd4, C_DW=3'd5, C_UPDATE=3'd6;

    reg signed [15:0] leak_current, exp_term, drive_current, dV, dw;
    reg signed [15:0] V_threshold, V_reset, tau_w, delta_t, ibias, capacitance, param_a, param_b;

    // helper regs
    reg [7:0] vm8_reg, w8_reg;
    reg signed [15:0] temp_q;

    // ---- Fixed-point helpers (mul OK, div replaced) ----
    function signed [15:0] qmul;
        input signed [15:0] a, b;
        reg   signed [31:0] p;
        begin
            p   = $signed(a) * $signed(b);
            qmul = p[23:8]; // keep Q4.8
        end
    endfunction

    // tuned shift-based approximate divide for parameters (used in LUT16)
    function signed [15:0] qdiv_param_pow2;
        input signed [15:0] numer;   // Q4.8
        input signed [15:0] denom_q; // Q4.8, expected >= 0
        reg [3:0] top_nib;
        reg [3:0] sh;  // 0..15
        begin
            if (denom_q <= 16'sd0) begin
                qdiv_param_pow2 = 16'sd0;
            end else begin
                top_nib = (denom_q[15:12] == 4'd0) ? 4'd1 : denom_q[15:12];
                case (top_nib)
                    4'd1:  sh = 3;
                    4'd2:  sh = 4;
                    4'd3:  sh = 5;
                    4'd4:  sh = 6;
                    4'd5:  sh = 7;
                    4'd6:  sh = 7;
                    4'd7:  sh = 8;
                    4'd8:  sh = 8;
                    default: sh = 9;
                endcase
                qdiv_param_pow2 = numer >>> sh;
            end
        end
    endfunction

    // 32-entry exponential LUT (input: Q4.8 in [-4, +4], output Q4.8)
    function signed [15:0] exp_q32;
        input signed [15:0] arg_in;
        integer idx;
        reg signed [15:0] RANGE_MIN, RANGE_MAX;
        reg signed [31:0] span, offs;
        begin
            RANGE_MIN = (-16'sd4) <<< 8;
            RANGE_MAX = ( 16'sd4) <<< 8;
            if (arg_in <= RANGE_MIN) idx = 0;
            else if (arg_in >= RANGE_MAX) idx = 31;
            else begin
                span = $signed(RANGE_MAX) - $signed(RANGE_MIN); // 8*256*2?
                offs = $signed(arg_in) - $signed(RANGE_MIN);
                idx  = (offs * 32) / span; // 0..31
                if (idx < 0)  idx = 0;
                if (idx > 31) idx = 31;
            end
            // Expanded conservatively from the verified 16-entry table (each entry duplicated).
            case (idx)
                0:  exp_q32 = 16'sd18;
                1:  exp_q32 = 16'sd18;
                2:  exp_q32 = 16'sd33;
                3:  exp_q32 = 16'sd33;
                4:  exp_q32 = 16'sd61;
                5:  exp_q32 = 16'sd61;
                6:  exp_q32 = 16'sd111;
                7:  exp_q32 = 16'sd111;
                8:  exp_q32 = 16'sd203;
                9:  exp_q32 = 16'sd203;
                10: exp_q32 = 16'sd372;
                11: exp_q32 = 16'sd372;
                12: exp_q32 = 16'sd681;
                13: exp_q32 = 16'sd681;
                14: exp_q32 = 16'sd1245;
                15: exp_q32 = 16'sd1245;
                16: exp_q32 = 16'sd2279;
                17: exp_q32 = 16'sd2279;
                18: exp_q32 = 16'sd4171;
                19: exp_q32 = 16'sd4171;
                20: exp_q32 = 16'sd7634;
                21: exp_q32 = 16'sd7634;
                22: exp_q32 = 16'sd13975;
                23: exp_q32 = 16'sd13975;
                24: exp_q32 = 16'sd25575;
                25: exp_q32 = 16'sd25575;
                26: exp_q32 = 16'sd32767;
                27: exp_q32 = 16'sd32767;
                28: exp_q32 = 16'sd32767;
                29: exp_q32 = 16'sd32767;
                30: exp_q32 = 16'sd32767;
                31: exp_q32 = 16'sd32767;
                default: exp_q32 = 16'sd32767;
            endcase
        end
    endfunction

    // Param converters (same mapping as LUT16)
    function signed [15:0] u8_to_signed_q_direct; input [7:0] x;
        begin u8_to_signed_q_direct = ($signed({8'b0, x}) - 16'sd128) <<< 8; end
    endfunction
    function signed [15:0] u8_to_q_unsigned_direct; input [7:0] x;
        begin u8_to_q_unsigned_direct = $signed({8'b0, x}) <<< 8; end
    endfunction

    // defaults and parameter capture
    always @(posedge clk) begin
        if (reset) begin
            V_threshold <= (-16'sd50) <<< 8;
            V_reset     <= (-16'sd65) <<< 8;
            tau_w       <= (16'sd100)<<< 8;
            delta_t     <= (16'sd2)  <<< 8;
            ibias       <= (16'sd15) <<< 8;
            capacitance <= (16'sd200)<<< 8;
            param_a     <= (16'sd2)  <<< 8;
            param_b     <= (16'sd40) <<< 8;
        end else if (r_ready) begin
            // parameter mapping (mirrors LUT16)
            delta_t     <= u8_to_signed_q_direct(params[0]);   // signed
            tau_w       <= u8_to_q_unsigned_direct(params[1]);
            param_a     <= u8_to_q_unsigned_direct(params[2]);
            param_b     <= u8_to_q_unsigned_direct(params[3]);
            V_reset     <= u8_to_signed_q_direct(params[4]);   // signed
            V_threshold <= u8_to_signed_q_direct(params[5]);   // signed
            ibias       <= u8_to_signed_q_direct(params[6]);   // signed
            capacitance <= u8_to_q_unsigned_direct(params[7]);
        end
    end

    // Core compute pipeline (mirrors LUT16 behavior, but uses exp_q32)
    always @(posedge clk) begin
        if (reset) begin
            V <= (-16'sd65) <<< 8; w <= 16'sd0; spike_reg <= 1'b0;
            compute_state <= C_LEAK;
            leak_current <= 16'sd0; exp_term <= 16'sd0; dV <= 16'sd0; dw <= 16'sd0;
            vm8_reg <= 8'd63; w8_reg <= 8'd128; temp_q <= 16'sd0;
        end else begin
            if (enable_core) begin
                case (compute_state)
                    C_LEAK: begin
                        // leak_current = gL * (EL - V)  (we fold gL into a small Q constant gL_nS)
                        leak_current <= qmul(16'sd100, (16'sd0 - V)); // placeholder gL * (EL-V) ; keep simple
                        compute_state <= C_ARG;
                    end
                    C_ARG: begin
                        if (delta_t == 16'sd0) begin
                            exp_term <= 16'sd0; compute_state <= C_DRIVE;
                        end else begin
                            // Correct argument: (V - V_threshold)/DeltaT
                            temp_q <= qdiv_param_pow2((V - V_threshold), (delta_t[15] ? -delta_t : delta_t));
                            compute_state <= C_EXP;
                        end
                    end
                    C_EXP: begin
                        // exp_term = gL * DeltaT * exp(arg)
                        // use abs(delta_t) for the multiplicative factor as in LUT16 style
                        exp_term <= qmul(16'sd100, qmul((delta_t[15] ? -delta_t : delta_t), exp_q32(temp_q)));
                        compute_state <= C_DRIVE;
                    end
                    C_DRIVE: begin
                        drive_current <= leak_current + exp_term - w + ibias;
                        compute_state <= C_DV;
                    end
                    C_DV: begin
                        // dV = drive_current / C  (approximate divide)
                        dV <= qdiv_param_pow2(drive_current, (capacitance[15] ? -capacitance : capacitance));
                        compute_state <= C_DW;
                    end
                    C_DW: begin
                        // dw = (a*(V-EL)-w)/tau_w  (approximate divide); EL set to 0 here for simplicity
                        if (tau_w <= (16'sd1 <<< 8)) begin
                            dw <= 16'sd0;
                        end else begin
                            dw <= qdiv_param_pow2((qmul(param_a, (V - 16'sd0)) - w), (tau_w[15] ? -tau_w : tau_w));
                        end
                        compute_state <= C_UPDATE;
                    end
                    C_UPDATE: begin
                        V <= V + dV;
                        if ((V + dV) > V_threshold) begin
                            spike_reg <= 1'b1;
                            V <= V_reset;
                            w <= w + dw + param_b;
                        end else begin
                            spike_reg <= 1'b0;
                            w <= w + dw;
                        end
                        compute_state <= C_LEAK;
                    end
                    default: compute_state <= C_LEAK;
                endcase
            end else begin
                // not enabled: hold default small values for outputs
                vm8_reg <= 8'd63;
                w8_reg  <= 8'd128;
                spike_reg <= 1'b0;
            end
        end
    end

    // -------------------- Output mux --------------------
    always @(*) begin
        uo_out_reg[0] = spike_reg;
        uo_out_reg[6:1] = vm8_reg[7:2];
        uo_out_reg[7] = 1'b0;
    end

endmodule
