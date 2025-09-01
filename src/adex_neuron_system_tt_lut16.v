// Area-optimized 16-bit ADEX neuron for TinyTapeout IHP-130
// - 16-entry exp LUT (Q4.8 input, Q4.8 output scale)
// - 16-bit arithmetic preserved
// - Divider removed (shift approximation), short pipeline to help timing

module adex_neuron_system_tt_lut16 (
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
    wire debug_mode  = ui_in[1];

    wire [3:0] nibble_in = uio_in[3:0];
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    // -------------------- Parameter loader --------------------
    localparam L_IDLE=3'd0, L_SHIFT=3'd1, L_LATCH=3'd2, L_WAIT_FOOTER=3'd3, L_READY=3'd4;
    reg [2:0]  lstate;
    reg [7:0]  byte_acc;
    reg        nibble_cnt;
    reg [2:0]  param_idx;   // 0..7
    reg [11:0] watchdog_cnt;
    localparam [11:0] WATCHDOG_MAX = 12'd4000;
    localparam [3:0]  FOOTER_NIB   = 4'hF;
    reg        load_prev;

    // 8x8-bit param bank
    reg [7:0] params [0:7];
    reg       r_ready;

    // packed out
    reg  [6:0] uo_out_reg;
    assign uo_out = {1'b0, uo_out_reg};

    initial uo_out_reg = 7'b0;

    always @(posedge clk) begin
        if (reset) begin
            lstate <= L_IDLE; byte_acc <= 8'd0; nibble_cnt <= 1'b0;
            param_idx <= 3'd0; watchdog_cnt <= 12'd0; r_ready <= 1'b0; load_prev <= 1'b0;

            // Defaults (match your previous ones)
            params[0] <= 8'd130;   // DeltaT
            params[1] <= 8'd228;   // TauW
            params[2] <= 8'd130;   // a
            params[3] <= 8'd168;   // b
            params[4] <= 8'd63;    // Vreset
            params[5] <= 8'd78;    // VT
            params[6] <= 8'd143;   // Ibias
            params[7] <= 8'd200;   // C
        end else begin
            load_prev <= load_enable;

            if (lstate != L_IDLE) begin
                if (watchdog_cnt < WATCHDOG_MAX) watchdog_cnt <= watchdog_cnt + 12'd1;
                else begin
                    lstate <= L_IDLE; nibble_cnt <= 1'b0; param_idx <= 3'd0; watchdog_cnt <= 12'd0;
                end
            end

            case (lstate)
                L_IDLE: if (load_mode && load_enable && !load_prev) begin
                    lstate <= L_SHIFT; nibble_cnt <= 1'b0; byte_acc <= 8'd0; param_idx <= 3'd0; watchdog_cnt <= 12'd0;
                end

                L_SHIFT: begin
                    if (load_enable && !load_prev) begin
                        if (!nibble_cnt) begin
                            byte_acc[7:4] <= nibble_in; nibble_cnt <= 1'b1;
                        end else begin
                            byte_acc[3:0] <= nibble_in; nibble_cnt <= 1'b0; lstate <= L_LATCH;
                        end
                        watchdog_cnt <= 12'd0;
                    end
                    if (!load_mode) begin
                        lstate <= L_IDLE; nibble_cnt <= 1'b0; param_idx <= 3'd0;
                    end
                end

                L_LATCH: begin
                    params[param_idx] <= byte_acc;
                    if (param_idx == 3'd7) lstate <= L_WAIT_FOOTER;
                    else begin
                        param_idx <= param_idx + 3'd1; lstate <= L_SHIFT;
                    end
                end

                L_WAIT_FOOTER: if (load_enable && !load_prev) begin
                    if (nibble_in == FOOTER_NIB) begin
                        r_ready<=1'b1; lstate <= L_READY;
                    end else lstate <= L_IDLE;
                end

                L_READY: if (!load_mode) begin r_ready <= 1'b0; lstate <= L_IDLE; end
                default: lstate <= L_IDLE;
            endcase
        end
    end

    // -------------------- 16-bit Core --------------------
    // Q4.8 signed
    reg  signed [15:0] V, w;
    reg                spike_reg;

    // constants
    localparam signed [15:0] gL_nS = (16'sd10)  <<< 8;   // 10 * 2^8
    localparam signed [15:0] EL_mV = (-16'sd70) <<< 8;   // -70 * 2^8

    // pipeline registers
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

    // Approximate division by a positive Q8 parameter using shifts:
    // We use the top nibble of the (unsigned) magnitude (param >>> 12) to pick a shift.
    // This keeps hardware tiny and timing easy; accuracy is acceptable for TT scale.
    function signed [15:0] qdiv_param_pow2;
        input signed [15:0] numer;   // Q4.8
        input signed [15:0] denom_q; // Q4.8, expected >= 0
        reg [3:0] top_nib;
        reg [3:0] sh;  // 0..15
        begin
            if (denom_q <= 16'sd0) begin
                qdiv_param_pow2 = 16'sd0;
            end else begin
                // top nibble of integer part
                top_nib = (denom_q[15:12] == 4'd0) ? 4'd1 : denom_q[15:12];
                // map nibble → shift. Larger denom => larger shift.
                // Tuned to roughly emulate 1/x over practical ranges.
                case (top_nib)
                    4'd1:  sh = 3;  // ~1/2..1 -> >>3
                    4'd2:  sh = 4;
                    4'd3:  sh = 5;
                    4'd4:  sh = 6;
                    4'd5:  sh = 7;
                    4'd6:  sh = 7;
                    4'd7:  sh = 8;
                    4'd8:  sh = 8;
                    default: sh = 9; // very large denom
                endcase
                // Keep Q4.8: shift right by 'sh' after compensating numerator by 8 fractional bits.
                // numer is already Q4.8, so just >> sh (acts like divide)
                qdiv_param_pow2 = numer >>> sh;
            end
        end
    endfunction

    // 16-entry exponential LUT (input: Q4.8 in [-4, +4])
    function signed [15:0] exp_q16;
        input signed [15:0] arg_in;
        integer idx;
        reg signed [15:0] RANGE_MIN, RANGE_MAX;
        reg signed [31:0] span, offs;
        begin
            RANGE_MIN = (-16'sd4) <<< 8;
            RANGE_MAX = ( 16'sd4) <<< 8;
            if (arg_in <= RANGE_MIN) idx = 0;
            else if (arg_in >= RANGE_MAX) idx = 15;
            else begin
                span = $signed(RANGE_MAX) - $signed(RANGE_MIN); // 8*256
                offs = $signed(arg_in) - $signed(RANGE_MIN);
                idx  = (offs * 16) / span; // 0..15
                if (idx < 0)  idx = 0;
                if (idx > 15) idx = 15;
            end
            // Down-sampled from your 32-entry table (Q4.8 gain folded later via qmul)
            case (idx)
                0:  exp_q16 = 16'sd18;
                1:  exp_q16 = 16'sd33;
                2:  exp_q16 = 16'sd61;
                3:  exp_q16 = 16'sd111;
                4:  exp_q16 = 16'sd203;
                5:  exp_q16 = 16'sd372;
                6:  exp_q16 = 16'sd681;
                7:  exp_q16 = 16'sd1245;
                8:  exp_q16 = 16'sd2279;
                9:  exp_q16 = 16'sd4171;
                10: exp_q16 = 16'sd7634;
                11: exp_q16 = 16'sd13975;
                12: exp_q16 = 16'sd25575;
                13: exp_q16 = 16'sd32767;
                14: exp_q16 = 16'sd32767;
                15: exp_q16 = 16'sd32767;
                default: exp_q16 = 16'sd32767;
            endcase
        end
    endfunction

    // Param converters (unchanged behavior)
    function signed [15:0] u8_to_signed_q_direct; input [7:0] x;
        begin u8_to_signed_q_direct = ($signed({8'b0, x}) - 16'sd128) <<< 8; end
    endfunction
    function signed [15:0] u8_to_q_unsigned_direct; input [7:0] x;
        begin u8_to_q_unsigned_direct = $signed({8'b0, x}) <<< 8; end
    endfunction
    function [7:0] sat_to_u8_fixed; input signed [15:0] x;
        reg signed [15:0] u; begin
            u = (x >>> 8) + 16'sd128;
            if (u < 16'sd0) u = 16'sd0; else if (u > 16'sd255) u = 16'sd255;
            sat_to_u8_fixed = u[7:0];
        end endfunction

    // Pre-convert parameters upon successful load
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
            V_threshold <= u8_to_signed_q_direct(params[5]);
            V_reset     <= u8_to_signed_q_direct(params[4]);
            tau_w       <= u8_to_q_unsigned_direct(params[1]);
            delta_t     <= u8_to_signed_q_direct(params[0]);
            ibias       <= u8_to_signed_q_direct(params[6]);
            capacitance <= u8_to_q_unsigned_direct(params[7]);
            param_a     <= u8_to_q_unsigned_direct(params[2]);
            param_b     <= u8_to_q_unsigned_direct(params[3]);
        end
    end

    // Core compute pipeline
    always @(posedge clk) begin
        if (reset) begin
            V <= (-16'sd65) <<< 8; w <= 16'sd0; spike_reg <= 1'b0;
            compute_state <= C_LEAK;
            leak_current <= 16'sd0; exp_term <= 16'sd0; drive_current <= 16'sd0; dV <= 16'sd0; dw <= 16'sd0;
            vm8_reg <= 8'd63; w8_reg <= 8'd128; temp_q <= 16'sd0;
        end else begin
            if (enable_core) begin
                case (compute_state)
                    C_LEAK: begin
                        leak_current <= qmul(gL_nS, (EL_mV - V));
                        compute_state <= C_ARG;
                    end
                    C_ARG: begin
                        if (delta_t == 16'sd0) begin
                            exp_term <= 16'sd0; compute_state <= C_DRIVE;
                        end else begin
                            // (V - V_threshold)/DeltaT  -> use shift approx for divide
                            temp_q <= qdiv_param_pow2((V - V_threshold), (delta_t[15] ? -delta_t : delta_t));
                            compute_state <= C_EXP;
                        end
                    end
                    C_EXP: begin
                        exp_term <= qmul(gL_nS, qmul((delta_t[15] ? -delta_t : delta_t), exp_q16(temp_q)));
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
                        // dw = (a*(V-EL)-w)/tau_w  (approximate divide)
                        if (tau_w <= (16'sd1 <<< 8)) begin
                            dw <= 16'sd0;
                        end else begin
                            dw <= qdiv_param_pow2( (qmul(param_a, (V - EL_mV)) - w),
                                                   (tau_w[15] ? -tau_w : tau_w));
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

                        // clipping (prevents runaway; gate-friendly)
                        if ((V + dV) < (-16'sd150 <<< 8)) V <= (-16'sd150 <<< 8);
                        else if ((V + dV) > (16'sd100 <<< 8)) V <= (16'sd100 <<< 8);

                        if ((w + dw) < (-16'sd100 <<< 8)) w <= (-16'sd100 <<< 8);
                        else if ((w + dw) > (16'sd127 <<< 8)) w <= (16'sd127 <<< 8);

                        vm8_reg <= sat_to_u8_fixed(V + dV);
                        w8_reg  <= sat_to_u8_fixed(w + dw);

                        compute_state <= C_LEAK;
                    end
                    default: compute_state <= C_LEAK;
                endcase
            end else begin
                compute_state <= C_LEAK;
                vm8_reg <= 8'd63;
                w8_reg  <= 8'd128;
                spike_reg <= 1'b0;
            end
        end
    end

    // -------------------- Output mux --------------------
    always @(*) begin
        uo_out_reg[0] = spike_reg;
        uo_out_reg[6:1] = (!debug_mode) ? vm8_reg[7:2] : w8_reg[7:2];
    end

endmodule
