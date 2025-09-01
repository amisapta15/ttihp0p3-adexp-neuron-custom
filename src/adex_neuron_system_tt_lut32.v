// Optimized 16-bit adex_neuron_system_tt_lut32.v
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

// ---------------------- 16-bit Core Neuron Logic ----------------------
// Changed from 32-bit to 16-bit arithmetic with Q4.8 format (4 integer, 8 fractional bits)
reg signed [15:0] V, w;
reg spike_reg;
reg signed [15:0] DeltaT_q, TauW_q, a_q, b_q, Vreset_q, VT_q, Ibias_q, C_q;

// Fixed constants in Q4.8 format
localparam signed [15:0] gL_nS = (16'sd10)  <<< 8;      // 10 * 2^8
localparam signed [15:0] EL_mV = (-16'sd70) <<< 8;      // -70 * 2^8

// Reduced intermediate registers to 16-bit
reg signed [15:0] leak, arg, expterm, drive, dV, dw;
reg [7:0] vm8_reg, w8_reg;

// 16-bit Q arithmetic helpers with Q4.8 format
function signed [15:0] qmul;
    input signed [15:0] a, b;
    reg signed [31:0] temp;
    begin
        temp = $signed(a) * $signed(b);
        qmul = temp >>> 8;  // Shift by 8 instead of 12
    end
endfunction

function signed [15:0] qdiv;
    input signed [15:0] a, b;
    reg signed [31:0] temp;
    begin
        if (b == 0) qdiv = 16'sd0;
        else begin
            temp = ($signed(a) <<< 8);  // Shift by 8 instead of 12
            qdiv = temp / $signed(b);
        end
    end
endfunction

// Optimized exponential function - still 32 entries but with 16-bit output
function signed [15:0] exp_q;
    input signed [15:0] arg_in;
    integer idx; reg signed [15:0] val;
    reg signed [15:0] RANGE_MIN, RANGE_MAX;
    reg signed [31:0] temp_calc, range_diff;
    begin
        RANGE_MIN = (-16'sd4)<<<8; RANGE_MAX = (16'sd4)<<<8;  // Q4.8 format
        if(arg_in < RANGE_MIN) idx = 0;
        else if(arg_in > RANGE_MAX) idx = 31;
        else begin
            temp_calc = $signed(arg_in) - $signed(RANGE_MIN);
            range_diff = $signed(RANGE_MAX) - $signed(RANGE_MIN);
            idx = (temp_calc * 32) / range_diff;
            if (idx > 31) idx = 31;
            if (idx < 0) idx = 0;
        end
        // Exponential LUT values scaled for 16-bit Q4.8 format
        case(idx)
            0: val=18;   1: val=25;   2: val=33;   3: val=45;
            4: val=61;   5: val=82;   6: val=111;  7: val=150;
            8: val=203;  9: val=275;  10: val=372; 11: val=503;
           12: val=681; 13: val=921; 14: val=1245;15: val=1684;
           16: val=2279;17: val=3084;18: val=4171;19: val=5644;
           20: val=7634;21: val=10332;22: val=13975;23: val=18906;
           24: val=25575;25: val=32767;26: val=32767;27: val=32767; // Saturate to prevent overflow
           28: val=32767;29: val=32767;30: val=32767;31: val=32767;
           default: val = 32767;
        endcase
        exp_q = val; // Values already appropriately scaled for Q4.8
    end
endfunction

// 16-bit converters with Q4.8 format
function signed [15:0] u8_to_signed_q_direct;
    input [7:0] x;
    begin
        u8_to_signed_q_direct = ($signed({8'b0, x}) - 16'sd128) <<< 8;  // Q4.8 format
    end
endfunction

function signed [15:0] u8_to_q_unsigned_direct;
    input [7:0] x;
    begin
        u8_to_q_unsigned_direct = $signed({8'b0, x}) <<< 8;  // Q4.8 format
    end
endfunction

function [7:0] sat_to_u8_fixed;
    input signed [15:0] x;
    reg signed [15:0] u;
    begin
        u = (x >>> 8) + 16'sd128;  // Convert from Q4.8 and add offset
        if (u < 0) u = 0;
        if (u > 255) u = 255;
        sat_to_u8_fixed = u[7:0];
    end
endfunction

always @(posedge clk) begin
    if (reset) begin
        // Initialize with 16-bit values in Q4.8 format
        V <= (-16'sd65) <<< 8;  // Start at -65mV in Q4.8
        w <= 16'sd0; 
        spike_reg <= 1'b0;
        leak <= 16'sd0;
        arg <= 16'sd0;
        expterm <= 16'sd0;
        drive <= 16'sd0;
        dV <= 16'sd0;
        dw <= 16'sd0;
        vm8_reg <= 8'd63;
        w8_reg <= 8'd128;
        // Default parameters in Q4.8 format
        DeltaT_q <= (16'sd2) <<< 8;      // 2mV
        TauW_q <= (16'sd100) <<< 8;      // 100ms
        a_q <= (16'sd2) <<< 8;           // 2nS
        b_q <= (16'sd40) <<< 8;          // 40pA
        Vreset_q <= (-16'sd65) <<< 8;    // -65mV
        VT_q <= (-16'sd50) <<< 8;        // -50mV
        Ibias_q <= (16'sd15) <<< 8;      // 15pA
        C_q <= (16'sd200) <<< 8;         // 200pF
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
            // Calculate leak current
            leak <= qmul(gL_nS, (EL_mV - V));
            
            // Calculate exponential term with protection
            if (DeltaT_q == 0) begin
                arg <= 16'sd0;
                expterm <= 16'sd0;
            end else begin
                arg <= qdiv((V - VT_q), DeltaT_q);
                expterm <= qmul(gL_nS, qmul(DeltaT_q, exp_q(arg)));
            end
            
            // Total drive current
            drive <= leak + expterm - w + Ibias_q;
            
            // Calculate dV with minimum capacitance protection
            if (C_q < (16'sd10 <<< 8)) begin  // Minimum 10pF in Q4.8
                dV <= qdiv(drive, (16'sd10 <<< 8));
            end else begin
                dV <= qdiv(drive, C_q);
            end
            
            // Calculate dw with protection
            if (TauW_q < (16'sd1 <<< 8)) begin  // Minimum 1ms in Q4.8
                dw <= 16'sd0;
            end else begin
                dw <= qdiv((qmul(a_q, (V - EL_mV)) - w), TauW_q);
            end

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

            // Saturate V and w to 16-bit bounds
            if (V > ( 16'sd100 <<< 8)) V <= ( 16'sd100 <<< 8);   // +100mV
            if (V < (-16'sd150 <<< 8)) V <= (-16'sd150 <<< 8);   // -150mV
            if (w > ( 16'sd127 <<< 8)) w <= ( 16'sd127 <<< 8);   // Reduced bound for 16-bit
            if (w < (-16'sd100 <<< 8)) w <= (-16'sd100 <<< 8);
            
            // Update output registers
            vm8_reg <= sat_to_u8_fixed(V);
            w8_reg  <= sat_to_u8_fixed(w);
        end else begin
            // Keep previous values when core disabled
            if (vm8_reg === 8'bxxxxxxxx) vm8_reg <= 8'd63;
            if (w8_reg === 8'bxxxxxxxx) w8_reg <= 8'd128;
        end
    end
end

always @(*) begin
    uo_out_reg[0] = spike_reg;
    if (!debug_mode) uo_out_reg[6:1] = vm8_reg[7:2];
    else uo_out_reg[6:1] = w8_reg[7:2];
end

endmodule