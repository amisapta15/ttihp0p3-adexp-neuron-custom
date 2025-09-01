// Area-optimized 12-bit adex_neuron_system_tt_lut32.v
module adex_neuron_system_tt_lut32 (
    input             clk,
    input             rst_n,
    input       [7:0] ui_in,
    output      [7:0] uo_out,
    input       [7:0] uio_in,
    output      [7:0] uio_out,
    output      [7:0] uio_oe
);

wire reset       = ~rst_n;
wire load_mode   = ui_in[4];
wire load_enable = ui_in[3];
wire enable_core = ui_in[2];
wire debug_mode  = ui_in[1];
wire [3:0] nibble_in;
assign nibble_in = uio_in[3:0];
assign uio_out = 8'b0;
assign uio_oe  = 8'b0;

// Parameter loader (unchanged for compatibility)
localparam L_IDLE=3'd0, L_SHIFT=3'd1, L_LATCH=3'd2, L_WAIT_FOOTER=3'd3, L_READY=3'd4;
reg [2:0]  lstate;
reg [7:0]  byte_acc;
reg        nibble_cnt;
reg [3:0]  param_idx;
reg [15:0] watchdog_cnt;
parameter WATCHDOG_MAX = 16'd50000;
parameter FOOTER_NIB = 4'b1111;
reg        load_prev;
reg [7:0] s_DeltaT, s_TauW, s_a, s_b, s_Vreset, s_VT, s_Ibias, s_C;
reg [7:0] r_DeltaT, r_TauW, r_a, r_b, r_Vreset, r_VT, r_Ibias, r_C;
reg       r_ready;
reg  [6:0] uo_out_reg;
assign uo_out = {1'b0, uo_out_reg};

initial begin
    uo_out_reg = 7'b0;
end
wire [7:0] p_DeltaT = r_DeltaT, p_TauW = r_TauW, p_a = r_a, p_b = r_b;
wire [7:0] p_Vreset = r_Vreset, p_VT = r_VT, p_Ibias = r_Ibias, p_C = r_C;
wire       params_ready = r_ready;

always @(posedge clk) begin
    if (reset) begin
        lstate <= L_IDLE; byte_acc <= 8'd0; nibble_cnt <= 1'b0;
        param_idx <= 4'd0; watchdog_cnt <= 16'd0; r_ready <= 1'b0; load_prev <= 1'b0;
        s_DeltaT <= 8'd0; s_TauW <= 8'd0; s_a <= 8'd0; s_b <= 8'd0; s_Vreset <= 8'd0;
        s_VT <= 8'd0; s_Ibias <= 8'd0; s_C <= 8'd0;
        r_DeltaT <= 8'd0; r_TauW <= 8'd0; r_a <= 8'd0; r_b <= 8'd0; r_Vreset <= 8'd0;
        r_VT <= 8'd0; r_Ibias <= 8'd0; r_C <= 8'd0;
    end else begin
        load_prev <= load_enable;
        if (lstate != L_IDLE) begin
            if (watchdog_cnt < WATCHDOG_MAX) watchdog_cnt <= watchdog_cnt + 1'b1;
            else begin lstate <= L_IDLE; nibble_cnt <= 1'b0; param_idx <= 4'd0; watchdog_cnt <= 16'd0; end
        end
        case (lstate)
            L_IDLE: if (load_mode && load_enable && !load_prev) begin lstate <= L_SHIFT; nibble_cnt <= 1'b0; byte_acc <= 8'd0; param_idx <= 4'd0; watchdog_cnt <= 16'd0; end
            L_SHIFT: begin
                if (load_enable && !load_prev) begin
                    if (nibble_cnt == 1'b0) begin byte_acc[7:4] <= nibble_in; nibble_cnt <= 1'b1; end
                    else begin byte_acc[3:0] <= nibble_in; nibble_cnt <= 1'b0; lstate <= L_LATCH; end
                    watchdog_cnt <= 16'd0;
                end
                if (!load_mode) begin lstate <= L_IDLE; nibble_cnt <= 1'b0; param_idx <= 4'd0; end
            end
            L_LATCH: begin
                case (param_idx)
                    4'd0: s_DeltaT <= byte_acc; 4'd1: s_TauW <= byte_acc;
                    4'd2: s_a      <= byte_acc; 4'd3: s_b    <= byte_acc;
                    4'd4: s_Vreset <= byte_acc; 4'd5: s_VT   <= byte_acc;
                    4'd6: s_Ibias  <= byte_acc; 4'd7: s_C    <= byte_acc;
                    default: ;
                endcase
                if (param_idx == 4'd7) lstate <= L_WAIT_FOOTER;
                else begin param_idx <= param_idx + 1'b1; lstate <= L_SHIFT; end
            end
            L_WAIT_FOOTER: if (load_enable && !load_prev) begin
                if (nibble_in == FOOTER_NIB) begin
                    r_DeltaT<=s_DeltaT; r_TauW<=s_TauW; r_a<=s_a; r_b<=s_b;
                    r_Vreset<=s_Vreset; r_VT<=s_VT; r_Ibias<=s_Ibias; r_C<=s_C;
                    r_ready<=1'b1; lstate <= L_READY;
                end else lstate <= L_IDLE;
            end
            L_READY: if (!load_mode) begin r_ready <= 1'b0; lstate <= L_IDLE; end
            default: lstate <= L_IDLE;
        endcase
    end
end

// ---------------------- 12-bit Core Neuron Logic ----------------------
// Reduced from 16-bit to 12-bit arithmetic with Q4.6 format (4 integer, 6 fractional bits)
reg signed [11:0] V, w;
reg spike_reg;
reg signed [11:0] DeltaT_q, TauW_q, a_q, b_q, Vreset_q, VT_q, Ibias_q, C_q;

// Fixed constants in Q4.6 format
localparam signed [11:0] gL_nS = (12'sd10)  << 6;      // 10 * 2^6
localparam signed [11:0] EL_mV = (-12'sd70) << 6;      // -70 * 2^6

// Reduced intermediate registers to 12-bit (major area savings)
reg signed [11:0] leak, expterm, drive;
reg signed [11:0] dV, dw, arg;
reg [7:0] vm8_reg, w8_reg;

// 12-bit Q arithmetic helpers with Q4.6 format
function signed [11:0] qmul;
    input signed [11:0] a, b;
    reg signed [23:0] temp;
    begin
        temp = $signed(a) * $signed(b);
        qmul = temp >>> 6;  // Shift by 6 for Q4.6
    end
endfunction

function signed [11:0] qdiv;
    input signed [11:0] a, b;
    reg signed [23:0] temp;
    begin
        if (b == 0) qdiv = 12'sd0;
        else begin
            temp = ($signed(a) << 6);  // Shift by 6 for Q4.6
            qdiv = temp / $signed(b);
        end
    end
endfunction

// Optimized exponential function - 32 entries with 12-bit output
function signed [11:0] exp_q;
    input signed [11:0] arg_in;
    integer idx; reg signed [11:0] val;
    reg signed [11:0] RANGE_MIN, RANGE_MAX;
    reg signed [23:0] temp_calc, range_diff;
    begin
        RANGE_MIN = (-12'sd4)<<6; RANGE_MAX = (12'sd4)<<6;  // Q4.6 format
        if(arg_in < RANGE_MIN) idx = 0;
        else if(arg_in > RANGE_MAX) idx = 31;
        else begin
            temp_calc = $signed(arg_in) - $signed(RANGE_MIN);
            range_diff = $signed(RANGE_MAX) - $signed(RANGE_MIN);
            idx = (temp_calc * 32) / range_diff;
            if (idx > 31) idx = 31;
            if (idx < 0) idx = 0;
        end
        // Exponential LUT values scaled for 12-bit Q4.6 format
        case(idx)
            0: val=1;    1: val=2;    2: val=2;    3: val=3;
            4: val=4;    5: val=5;    6: val=7;    7: val=9;
            8: val=13;   9: val=17;   10: val=23;  11: val=31;
           12: val=43;   13: val=58;  14: val=78;  15: val=105;
           16: val=142;  17: val=193; 18: val=261; 19: val=353;
           20: val=478;  21: val=647; 22: val=875; 23: val=1184;
           24: val=1602; 25: val=2047;26: val=2047;27: val=2047; // Saturate at 12-bit max
           28: val=2047; 29: val=2047;30: val=2047;31: val=2047;
           default: val = 2047;
        endcase
        exp_q = val;
    end
endfunction

// 12-bit converters with Q4.6 format
function signed [11:0] u8_to_signed_q_direct;
    input [7:0] x;
    begin
        u8_to_signed_q_direct = ($signed({4'b0, x}) - 12'sd128) << 6;  // Q4.6 format
    end
endfunction

function signed [11:0] u8_to_q_unsigned_direct;
    input [7:0] x;
    begin
        u8_to_q_unsigned_direct = $signed({4'b0, x}) << 6;  // Q4.6 format
    end
endfunction

function [7:0] sat_to_u8_fixed;
    input signed [11:0] x;
    reg signed [11:0] u;
    begin
        u = (x >>> 6) + 12'sd128;  // Convert from Q4.6 and add offset
        if (u < 0) u = 0;
        if (u > 255) u = 255;
        sat_to_u8_fixed = u[7:0];
    end
endfunction

// State machine for neuron computation (reduces combinational logic)
reg [1:0] comp_state;
localparam C_LEAK=0, C_EXP=1, C_UPDATE=2, C_SPIKE=3;

always @(posedge clk) begin
    if (reset) begin
        // Initialize with 12-bit values in Q4.6 format
        V <= (-12'sd65) << 6;  // Start at -65mV in Q4.6
        w <= 12'sd0; 
        spike_reg <= 1'b0;
        leak <= 12'sd0; expterm <= 12'sd0; drive <= 12'sd0;
        dV <= 12'sd0; dw <= 12'sd0; arg <= 12'sd0;
        vm8_reg <= 8'd63; w8_reg <= 8'd128;
        comp_state <= C_LEAK;
        // Default parameters in Q4.6 format
        DeltaT_q <= (12'sd2) << 6;      // 2mV
        TauW_q <= (12'sd100) << 6;      // 100ms
        a_q <= (12'sd2) << 6;           // 2nS
        b_q <= (12'sd40) << 6;          // 40pA
        Vreset_q <= (-12'sd65) << 6;    // -65mV
        VT_q <= (-12'sd50) << 6;        // -50mV
        Ibias_q <= (12'sd15) << 6;      // 15pA
        C_q <= (12'sd200) << 6;         // 200pF
    end else begin
        if (params_ready) begin
            DeltaT_q <= u8_to_signed_q_direct(p_DeltaT); 
            TauW_q <= u8_to_q_unsigned_direct(p_TauW);
            a_q <= u8_to_q_unsigned_direct(p_a); 
            b_q <= u8_to_q_unsigned_direct(p_b);
            Vreset_q <= u8_to_signed_q_direct(p_Vreset); 
            VT_q <= u8_to_signed_q_direct(p_VT);
            Ibias_q <= u8_to_signed_q_direct(p_Ibias); 
            C_q <= u8_to_q_unsigned_direct(p_C);
        end

        if (enable_core) begin
            // Pipeline the computation across multiple cycles (reduces comb logic)
            case (comp_state)
                C_LEAK: begin
                    // Calculate leak current
                    leak <= qmul(gL_nS, (EL_mV - V));
                    comp_state <= C_EXP;
                end
                C_EXP: begin
                    // Calculate exponential term
                    if (DeltaT_q == 0) begin
                        arg <= 12'sd0;
                        expterm <= 12'sd0;
                    end else begin
                        arg <= qdiv((V - VT_q), DeltaT_q);
                        expterm <= qmul(gL_nS, qmul(DeltaT_q, exp_q(arg)));
                    end
                    comp_state <= C_UPDATE;
                end
                C_UPDATE: begin
                    // Total drive current and derivatives
                    drive <= leak + expterm - w + Ibias_q;
                    
                    // Calculate dV with minimum capacitance protection
                    if (C_q < (12'sd10 << 6)) begin  // Minimum 10pF in Q4.6
                        dV <= qdiv(drive, (12'sd10 << 6));
                    end else begin
                        dV <= qdiv(drive, C_q);
                    end
                    
                    // Calculate dw with protection
                    if (TauW_q < (12'sd1 << 6)) begin  // Minimum 1ms in Q4.6
                        dw <= 12'sd0;
                    end else begin
                        dw <= qdiv((qmul(a_q, (V - EL_mV)) - w), TauW_q);
                    end
                    
                    comp_state <= C_SPIKE;
                end
                C_SPIKE: begin
                    // Update state variables
                    V <= V + dV;
                    w <= w + dw;

                    // Spike detection and reset
                    if (V > VT_q) begin
                        spike_reg <= 1'b1;
                        V <= Vreset_q;
                        w <= w + b_q;
                    end else begin
                        spike_reg <= 1'b0;
                    end

                    // Saturate V and w to 12-bit bounds
                    if (V > ( 12'sd100 << 6)) V <= ( 12'sd100 << 6);   // +100mV
                    if (V < (-12'sd150 << 6)) V <= (-12'sd150 << 6);   // -150mV
                    if (w > ( 12'sd127 << 6)) w <= ( 12'sd127 << 6);   
                    if (w < (-12'sd100 << 6)) w <= (-12'sd100 << 6);
                    
                    // Update output registers
                    vm8_reg <= sat_to_u8_fixed(V);
                    w8_reg  <= sat_to_u8_fixed(w);
                    
                    comp_state <= C_LEAK;
                end
            endcase
        end else begin
            // Keep previous values when core disabled
            if (vm8_reg === 8'bxxxxxxxx) vm8_reg <= 8'd63;
            if (w8_reg === 8'bxxxxxxxx) w8_reg <= 8'd128;
            comp_state <= C_LEAK;
        end
    end
end

always @(*) begin
    uo_out_reg[0] = spike_reg;
    if (!debug_mode) uo_out_reg[6:1] = vm8_reg[7:2];
    else uo_out_reg[6:1] = w8_reg[7:2];
end

endmodule