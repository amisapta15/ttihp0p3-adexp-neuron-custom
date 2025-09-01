// Compact 16-bit AdEx Neuron System
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

// Parameter loader (simplified)
localparam L_IDLE=2'd0, L_SHIFT=2'd1, L_LATCH=2'd2, L_READY=2'd3;
reg [1:0]  lstate;
reg [7:0]  byte_acc;
reg        nibble_cnt;
reg [2:0]  param_idx;  // Reduced to 3 bits for 8 parameters
reg [7:0] s_Vreset, s_VT, s_Ibias, s_C;  // Only essential parameters
reg [7:0] r_Vreset, r_VT, r_Ibias, r_C;
reg       r_ready;
reg [6:0] uo_out_reg;
assign uo_out = {1'b0, uo_out_reg};

// Initialize output to prevent 'x' values
initial begin
    uo_out_reg = 7'b0;
end

wire [7:0] p_Vreset = r_Vreset, p_VT = r_VT, p_Ibias = r_Ibias, p_C = r_C;
wire       params_ready = r_ready;

always @(posedge clk) begin
    if (reset) begin
        lstate <= L_IDLE; byte_acc <= 8'd0; nibble_cnt <= 1'b0;
        param_idx <= 3'd0; r_ready <= 1'b0;
        s_Vreset <= 8'd63; s_VT <= 8'd78; s_Ibias <= 8'd158; s_C <= 8'd100;
        r_Vreset <= 8'd63; r_VT <= 8'd78; r_Ibias <= 8'd158; r_C <= 8'd100;
    end else begin
        case (lstate)
            L_IDLE: if (load_mode && load_enable) begin 
                lstate <= L_SHIFT; nibble_cnt <= 1'b0; byte_acc <= 8'd0; param_idx <= 3'd0;
            end
            L_SHIFT: begin
                if (load_enable) begin
                    if (nibble_cnt == 1'b0) begin 
                        byte_acc[7:4] <= nibble_in; nibble_cnt <= 1'b1; 
                    end else begin 
                        byte_acc[3:0] <= nibble_in; nibble_cnt <= 1'b0; lstate <= L_LATCH; 
                    end
                end
                if (!load_mode) lstate <= L_IDLE;
            end
            L_LATCH: begin
                case (param_idx)
                    3'd0: s_Vreset <= byte_acc; 
                    3'd1: s_VT <= byte_acc;
                    3'd2: s_Ibias <= byte_acc; 
                    3'd3: s_C <= byte_acc;
                    default: ;
                endcase
                if (param_idx == 3'd3) begin
                    r_Vreset <= s_Vreset; r_VT <= s_VT; r_Ibias <= s_Ibias; r_C <= s_C;
                    r_ready <= 1'b1; lstate <= L_READY;
                end else begin 
                    param_idx <= param_idx + 1'b1; lstate <= L_SHIFT; 
                end
            end
            L_READY: if (!load_mode) begin r_ready <= 1'b0; lstate <= L_IDLE; end
            default: lstate <= L_IDLE;
        endcase
    end
end

// ---------------------- Compact 16-bit Neuron Core ----------------------
reg signed [15:0] V, w;
reg spike_reg;
reg signed [15:0] Vreset_q, VT_q, Ibias_q, C_q;

// Reduced precision constants (Q4.4 format)
localparam signed [15:0] gL_nS = 16'sd160;      // 10 * 2^4
localparam signed [15:0] EL_mV = -16'sd1120;    // -70 * 2^4

reg signed [15:0] leak, drive, dV;
reg [7:0] vm8_reg, w8_reg;

// Simplified Q4.4 arithmetic
function signed [15:0] qmul_simple;
    input signed [15:0] a, b;
    reg signed [31:0] temp;
    begin
        temp = $signed(a) * $signed(b);
        qmul_simple = temp >>> 4;  // Q4.4 format
    end
endfunction

function signed [15:0] qdiv_simple;
    input signed [15:0] a, b;
    reg signed [31:0] temp;
    begin
        if (b == 0) qdiv_simple = 16'sd0;
        else begin
            temp = ($signed(a) <<< 4);
            qdiv_simple = temp / $signed(b);
        end
    end
endfunction

// Much simpler exponential approximation - just a small boost when above threshold
function signed [15:0] simple_exp;
    input signed [15:0] v_diff;
    begin
        if (v_diff > 16'sd32) simple_exp = 16'sd256;  // Significant boost above VT
        else if (v_diff > 16'sd16) simple_exp = 16'sd64;   // Moderate boost
        else if (v_diff > 16'sd0) simple_exp = 16'sd16;    // Small boost
        else simple_exp = 16'sd0;  // No boost below threshold
    end
endfunction

// Direct parameter conversion
function signed [15:0] u8_to_signed_q4;
    input [7:0] x;
    begin
        u8_to_signed_q4 = ($signed({8'b0, x}) - 16'sd128) <<< 4;
    end
endfunction

function signed [15:0] u8_to_unsigned_q4;
    input [7:0] x;
    begin
        u8_to_unsigned_q4 = $signed({8'b0, x}) <<< 4;
    end
endfunction

function [7:0] sat_to_u8_simple;
    input signed [15:0] x;
    reg signed [15:0] u;
    begin
        u = (x >>> 4) + 16'sd128;
        if (u < 0) u = 0;
        if (u > 255) u = 255;
        sat_to_u8_simple = u[7:0];
    end
endfunction

always @(posedge clk) begin
    if (reset) begin
        V <= -16'sd1040;  // -65mV in Q4.4
        w <= 16'sd0; 
        spike_reg <= 1'b0;
        leak <= 16'sd0;
        drive <= 16'sd0;
        dV <= 16'sd0;
        vm8_reg <= 8'd63;
        w8_reg <= 8'd128;
        // Default parameters in Q4.4
        Vreset_q <= -16'sd1040;    // -65mV
        VT_q <= -16'sd800;         // -50mV  
        Ibias_q <= 16'sd320;       // 20pA
        C_q <= 16'sd1600;          // 100pF
    end else begin
        if (params_ready) begin
            Vreset_q <= u8_to_signed_q4(p_Vreset); 
            VT_q <= u8_to_signed_q4(p_VT);
            Ibias_q <= u8_to_signed_q4(p_Ibias); 
            C_q <= u8_to_unsigned_q4(p_C);
        end

        if (enable_core) begin
            // Simplified dynamics
            leak <= qmul_simple(gL_nS, (EL_mV - V));
            drive <= leak + simple_exp(V - VT_q) + Ibias_q - (w >>> 2); // w/4 for adaptation
            
            // Calculate dV with protection
            if (C_q < 16'sd160) begin  // Minimum 10pF
                dV <= qdiv_simple(drive, 16'sd160);
            end else begin
                dV <= qdiv_simple(drive, C_q);
            end

            // Update voltage
            V <= V + dV;
            
            // Simple adaptation: w decays and increases on spike
            w <= w - (w >>> 6);  // Decay w by 1/64 each step

            // Spike detection and reset
            if (V > VT_q) begin
                spike_reg <= 1'b1;
                V <= Vreset_q;
                w <= w + 16'sd200;  // Add fixed adaptation on spike
            end else begin
                spike_reg <= 1'b0;
            end

            // Saturate V and w
            if (V > 16'sd1600) V <= 16'sd1600;     // 100mV
            if (V < -16'sd2400) V <= -16'sd2400;   // -150mV
            if (w > 16'sd4000) w <= 16'sd4000;     // Cap adaptation
            if (w < 16'sd0) w <= 16'sd0;           // Keep adaptation positive
            
            // Update output registers
            vm8_reg <= sat_to_u8_simple(V);
            w8_reg <= sat_to_u8_simple(w);
        end else begin
            // Ensure no 'x' values when disabled
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